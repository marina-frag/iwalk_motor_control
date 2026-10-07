#include "iwalk_hardware/vesc_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace iwalk_hardware
{
namespace
{

uint32_t decode_u32_be(const uint8_t * input)
{
  return (uint32_t(input[0]) << 24) | (uint32_t(input[1]) << 16) |
         (uint32_t(input[2]) << 8) | uint32_t(input[3]);
}

uint16_t decode_u16_be(const uint8_t * input)
{
  return static_cast<uint16_t>((uint16_t(input[0]) << 8) | uint16_t(input[1]));
}

int32_t signed_u32(uint32_t value)
{
  if (value <= static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
    return static_cast<int32_t>(value);
  }
  const int64_t signed_value = static_cast<int64_t>(value) - (int64_t{1} << 32);
  return static_cast<int32_t>(signed_value);
}

int64_t round_half_to_even(long double value)
{
  const bool negative = value < 0.0L;
  const long double magnitude = std::fabs(value);
  const long double lower_ld = std::floor(magnitude);
  int64_t lower = static_cast<int64_t>(lower_ld);
  const long double fraction = magnitude - lower_ld;
  // The input started as a double wheel command. Treat its sub-ULP propagation
  // through the conversion factor as an exact half tie, matching Python round().
  const long double tie_tolerance =
    64.0L * static_cast<long double>(std::numeric_limits<double>::epsilon()) *
    std::max(1.0L, magnitude);
  const bool half_tie = std::fabs(fraction - 0.5L) <= tie_tolerance;
  if ((!half_tie && fraction > 0.5L) || (half_tie && (lower & 1LL) != 0)) {
    ++lower;
  }
  return negative ? -lower : lower;
}

uint8_t bitrate_code(uint32_t bitrate)
{
  switch (bitrate) {
    case 1000000U: return 0x01;
    case 800000U: return 0x02;
    case 500000U: return 0x03;
    case 400000U: return 0x04;
    case 250000U: return 0x05;
    case 200000U: return 0x06;
    case 125000U: return 0x07;
    case 100000U: return 0x08;
    case 50000U: return 0x09;
    case 20000U: return 0x0A;
    case 10000U: return 0x0B;
    case 5000U: return 0x0C;
    default:
      throw std::invalid_argument("unsupported Seeed/Waveshare CAN bitrate");
  }
}

}  // namespace

void encode_i32_be(uint8_t * output, int32_t value)
{
  const uint32_t raw = static_cast<uint32_t>(value);
  output[0] = static_cast<uint8_t>(raw >> 24);
  output[1] = static_cast<uint8_t>(raw >> 16);
  output[2] = static_cast<uint8_t>(raw >> 8);
  output[3] = static_cast<uint8_t>(raw);
}

int32_t decode_i32_be(const uint8_t * input)
{
  return signed_u32(decode_u32_be(input));
}

int16_t decode_i16_be(const uint8_t * input)
{
  const uint16_t raw = decode_u16_be(input);
  if (raw <= static_cast<uint16_t>(std::numeric_limits<int16_t>::max())) {
    return static_cast<int16_t>(raw);
  }
  return static_cast<int16_t>(static_cast<int32_t>(raw) - (1 << 16));
}

CanMessage make_vesc_i32_command(uint8_t vesc_id, uint32_t packet_id, int32_t value)
{
  CanMessage message;
  message.id = (packet_id << 8) | uint32_t(vesc_id);
  message.extended = true;
  message.dlc = 4;
  encode_i32_be(message.data.data(), value);
  return message;
}

bool decode_status1(const CanMessage & message, uint8_t vesc_id, Status1 & status)
{
  if (!message.extended || message.remote || message.dlc != 8 ||
    message.id != ((kCanPacketStatus1 << 8) | uint32_t(vesc_id)))
  {
    return false;
  }
  status.erpm = decode_i32_be(message.data.data());
  status.current_amps = static_cast<double>(decode_i16_be(message.data.data() + 4)) / 10.0;
  status.duty = static_cast<double>(decode_i16_be(message.data.data() + 6)) / 1000.0;
  return true;
}

bool decode_status5(const CanMessage & message, uint8_t vesc_id, Status5 & status)
{
  if (!message.extended || message.remote || message.dlc != 8 ||
    message.id != ((kCanPacketStatus5 << 8) | uint32_t(vesc_id)))
  {
    return false;
  }
  status.tachometer = decode_i32_be(message.data.data());
  status.input_voltage = static_cast<double>(decode_i16_be(message.data.data() + 4)) / 10.0;
  // Bytes 6-7 are reserved in current VESC firmware and are deliberately ignored.
  return true;
}

double erpm_to_wheel_rad_s(
  int32_t erpm, int direction, int pole_pairs, double gear_ratio)
{
  return static_cast<double>(direction) * static_cast<double>(erpm) * kTwoPi /
         (60.0 * static_cast<double>(pole_pairs) * gear_ratio);
}

int32_t wheel_rad_s_to_erpm(
  double wheel_rad_s, int direction, int pole_pairs, double gear_ratio,
  double max_erpm)
{
  if (!std::isfinite(wheel_rad_s)) {
    throw std::invalid_argument("wheel velocity command is NaN or infinite");
  }
  const long double raw = static_cast<long double>(direction) *
    static_cast<long double>(wheel_rad_s) * static_cast<long double>(gear_ratio) *
    static_cast<long double>(pole_pairs) * 60.0L /
    static_cast<long double>(kTwoPi);
  const long double limit = static_cast<long double>(max_erpm);
  const long double clamped = std::clamp(raw, -limit, limit);
  const int64_t rounded = round_half_to_even(clamped);
  return static_cast<int32_t>(rounded);
}

double tachometer_delta_to_wheel_rad(
  int64_t delta, int direction, int pole_pairs, double gear_ratio)
{
  return static_cast<double>(direction) * static_cast<double>(delta) * kTwoPi /
         (6.0 * static_cast<double>(pole_pairs) * gear_ratio);
}

int64_t signed_tachometer_delta(uint32_t current, uint32_t previous)
{
  const uint32_t difference = current - previous;
  if (difference <= static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
    return static_cast<int64_t>(difference);
  }
  return static_cast<int64_t>(difference) - (int64_t{1} << 32);
}

CanMessage make_bridge_heartbeat(uint16_t sequence)
{
  CanMessage message;
  message.id = kBridgeHeartbeatCanId;
  message.extended = true;
  message.dlc = 8;
  message.data = {
    static_cast<uint8_t>('I'), static_cast<uint8_t>('W'),
    static_cast<uint8_t>('B'), static_cast<uint8_t>('R'),
    1U, 1U, static_cast<uint8_t>(sequence >> 8), static_cast<uint8_t>(sequence)};
  return message;
}

bool is_bridge_heartbeat(const CanMessage & message)
{
  return message.extended && !message.remote && message.id == kBridgeHeartbeatCanId &&
         message.dlc == 8 && message.data[0] == 'I' && message.data[1] == 'W' &&
         message.data[2] == 'B' && message.data[3] == 'R' && message.data[4] == 1U &&
         message.data[5] == 1U;
}

std::array<uint8_t, 20> make_seeed_init_frame(uint32_t can_bitrate)
{
  std::array<uint8_t, 20> frame{};
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = 0x12;
  frame[3] = bitrate_code(can_bitrate);
  frame[4] = 0x02;  // Extended frames.
  // Bytes 5-12: disabled acceptance filter and mask, matching python-can.
  frame[13] = 0x00;  // Normal operation mode.
  frame[14] = 0x01;  // "Send once", matching the vendor application/python-can.
  // Bytes 15-18: unused manual bitrate fields.
  uint32_t checksum = 0;
  for (std::size_t i = 2; i < frame.size() - 1; ++i) {
    checksum += frame[i];
  }
  frame[19] = static_cast<uint8_t>(checksum & 0xFFU);
  return frame;
}

std::vector<uint8_t> serialize_seeed_frame(const CanMessage & message)
{
  if (message.dlc > 8 || (!message.extended && message.id > 0x7FFU) ||
    (message.extended && message.id > 0x1FFFFFFFU))
  {
    throw std::invalid_argument("invalid classic CAN frame");
  }
  std::vector<uint8_t> output;
  output.reserve(7U + message.dlc);
  output.push_back(0xAA);
  uint8_t type = static_cast<uint8_t>(0xC0U | message.dlc);
  if (message.extended) {
    type = static_cast<uint8_t>(type | 0x20U);
  }
  if (message.remote) {
    type = static_cast<uint8_t>(type | 0x10U);
  }
  output.push_back(type);
  const std::size_t id_bytes = message.extended ? 4U : 2U;
  for (std::size_t i = 0; i < id_bytes; ++i) {
    output.push_back(static_cast<uint8_t>(message.id >> (8U * i)));
  }
  output.insert(output.end(), message.data.begin(), message.data.begin() + message.dlc);
  output.push_back(0x55);
  return output;
}

std::vector<CanMessage> SeeedSerialParser::feed(const uint8_t * data, std::size_t size)
{
  if (size > kMaximumBufferedBytes) {
    dropped_bytes_ += size - kMaximumBufferedBytes;
    data += size - kMaximumBufferedBytes;
    size = kMaximumBufferedBytes;
    buffer_.clear();
  } else if (buffer_.size() + size > kMaximumBufferedBytes) {
    const std::size_t remove = buffer_.size() + size - kMaximumBufferedBytes;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(remove));
    dropped_bytes_ += remove;
  }
  buffer_.insert(buffer_.end(), data, data + size);

  std::vector<CanMessage> messages;
  while (!buffer_.empty()) {
    const auto start = std::find(buffer_.begin(), buffer_.end(), uint8_t{0xAA});
    if (start != buffer_.begin()) {
      const std::size_t remove = static_cast<std::size_t>(start - buffer_.begin());
      dropped_bytes_ += remove;
      buffer_.erase(buffer_.begin(), start);
      if (buffer_.empty()) {
        break;
      }
    }
    if (buffer_.size() < 2) {
      break;
    }

    if (buffer_[1] == 0x55) {
      constexpr std::size_t status_size = 20;
      if (buffer_.size() < status_size) {
        break;
      }
      uint32_t checksum = 0;
      for (std::size_t i = 2; i < status_size - 1; ++i) {
        checksum += buffer_[i];
      }
      if (static_cast<uint8_t>(checksum) != buffer_[status_size - 1]) {
        buffer_.erase(buffer_.begin());
        ++dropped_bytes_;
        continue;
      }
      buffer_.erase(buffer_.begin(), buffer_.begin() + status_size);
      continue;
    }

    const uint8_t type = buffer_[1];
    const uint8_t dlc = static_cast<uint8_t>(type & 0x0FU);
    if ((type & 0xC0U) != 0xC0U || dlc > 8U) {
      buffer_.erase(buffer_.begin());
      ++dropped_bytes_;
      continue;
    }
    const bool extended = (type & 0x20U) != 0;
    const bool remote = (type & 0x10U) != 0;
    const std::size_t id_bytes = extended ? 4U : 2U;
    const std::size_t frame_size = 2U + id_bytes + dlc + 1U;
    if (buffer_.size() < frame_size) {
      break;
    }
    if (buffer_[frame_size - 1] != 0x55) {
      buffer_.erase(buffer_.begin());
      ++dropped_bytes_;
      continue;
    }

    CanMessage message;
    message.extended = extended;
    message.remote = remote;
    message.dlc = dlc;
    for (std::size_t i = 0; i < id_bytes; ++i) {
      message.id |= uint32_t(buffer_[2U + i]) << (8U * i);
    }
    const uint32_t maximum_id = extended ? 0x1FFFFFFFU : 0x7FFU;
    if (message.id > maximum_id) {
      buffer_.erase(buffer_.begin());
      ++dropped_bytes_;
      continue;
    }
    std::copy_n(buffer_.begin() + static_cast<std::ptrdiff_t>(2U + id_bytes), dlc,
      message.data.begin());
    messages.push_back(message);
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
  return messages;
}

}  // namespace iwalk_hardware
