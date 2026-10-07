#ifndef IWALK_HARDWARE__VESC_PROTOCOL_HPP_
#define IWALK_HARDWARE__VESC_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace iwalk_hardware
{

constexpr uint32_t kCanPacketSetDuty = 0U;
constexpr uint32_t kCanPacketSetCurrent = 1U;
constexpr uint32_t kCanPacketSetCurrentBrake = 2U;
constexpr uint32_t kCanPacketSetRpm = 3U;
constexpr uint32_t kCanPacketStatus1 = 9U;
constexpr uint32_t kCanPacketStatus5 = 27U;
constexpr uint32_t kBridgeHeartbeatCanId = 0x1FFFFFFEU;
constexpr double kTwoPi = 6.283185307179586476925286766559;

struct CanMessage
{
  uint32_t id = 0;
  bool extended = true;
  bool remote = false;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
};

struct Status1
{
  int32_t erpm = 0;
  double current_amps = 0.0;
  double duty = 0.0;
};

struct Status5
{
  int32_t tachometer = 0;
  double input_voltage = 0.0;
};

void encode_i32_be(uint8_t * output, int32_t value);
int32_t decode_i32_be(const uint8_t * input);
int16_t decode_i16_be(const uint8_t * input);

CanMessage make_vesc_i32_command(uint8_t vesc_id, uint32_t packet_id, int32_t value);
bool decode_status1(const CanMessage & message, uint8_t vesc_id, Status1 & status);
bool decode_status5(const CanMessage & message, uint8_t vesc_id, Status5 & status);

double erpm_to_wheel_rad_s(
  int32_t erpm, int direction, int pole_pairs, double gear_ratio);
int32_t wheel_rad_s_to_erpm(
  double wheel_rad_s, int direction, int pole_pairs, double gear_ratio,
  double max_erpm);
double tachometer_delta_to_wheel_rad(
  int64_t delta, int direction, int pole_pairs, double gear_ratio);
int64_t signed_tachometer_delta(uint32_t current, uint32_t previous);

CanMessage make_bridge_heartbeat(uint16_t sequence);
bool is_bridge_heartbeat(const CanMessage & message);

std::array<uint8_t, 20> make_seeed_init_frame(uint32_t can_bitrate);
std::vector<uint8_t> serialize_seeed_frame(const CanMessage & message);

class SeeedSerialParser
{
public:
  static constexpr std::size_t kMaximumBufferedBytes = 4096;

  std::vector<CanMessage> feed(const uint8_t * data, std::size_t size);
  std::size_t buffered_bytes() const noexcept {return buffer_.size();}
  std::size_t dropped_bytes() const noexcept {return dropped_bytes_;}

private:
  std::vector<uint8_t> buffer_;
  std::size_t dropped_bytes_ = 0;
};

}  // namespace iwalk_hardware

#endif  // IWALK_HARDWARE__VESC_PROTOCOL_HPP_
