#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "iwalk_hardware/vesc_protocol.hpp"

namespace iwalk_hardware
{
namespace
{

TEST(VescProtocol, EncodesAndDecodesSignedCanPayloads)
{
  const auto command = make_vesc_i32_command(1, kCanPacketSetRpm, -1500);
  EXPECT_TRUE(command.extended);
  EXPECT_EQ(command.id, 0x301U);
  EXPECT_EQ(command.dlc, 4U);
  EXPECT_EQ(command.data[0], 0xFFU);
  EXPECT_EQ(command.data[1], 0xFFU);
  EXPECT_EQ(command.data[2], 0xFAU);
  EXPECT_EQ(command.data[3], 0x24U);
  EXPECT_EQ(decode_i32_be(command.data.data()), -1500);

  CanMessage status_message;
  status_message.id = (kCanPacketStatus1 << 8) | 1U;
  status_message.dlc = 8;
  encode_i32_be(status_message.data.data(), -1500);
  status_message.data[4] = 0xFF;
  status_message.data[5] = 0x85;  // -12.3 A
  status_message.data[6] = 0xFF;
  status_message.data[7] = 0x38;  // -0.200 duty
  Status1 status;
  ASSERT_TRUE(decode_status1(status_message, 1, status));
  EXPECT_EQ(status.erpm, -1500);
  EXPECT_DOUBLE_EQ(status.current_amps, -12.3);
  EXPECT_DOUBLE_EQ(status.duty, -0.2);
}

TEST(VescProtocol, MatchesRequestedConversionVectors)
{
  EXPECT_NEAR(erpm_to_wheel_rad_s(1500, 1, 15, 1.0), 10.471975511966, 1e-12);
  EXPECT_EQ(wheel_rad_s_to_erpm(10.471975511966, 1, 15, 1.0, 7500.0), 1500);
  EXPECT_NEAR(tachometer_delta_to_wheel_rad(90, 1, 15, 1.0), kTwoPi, 1e-12);
}

TEST(VescProtocol, PreservesNegativeRotationAndDirection)
{
  EXPECT_NEAR(erpm_to_wheel_rad_s(-1500, 1, 15, 1.0), -10.471975511966, 1e-12);
  EXPECT_NEAR(erpm_to_wheel_rad_s(1500, -1, 15, 1.0), -10.471975511966, 1e-12);
  EXPECT_EQ(wheel_rad_s_to_erpm(10.471975511966, -1, 15, 1.0, 7500.0), -1500);
  EXPECT_NEAR(tachometer_delta_to_wheel_rad(-90, -1, 15, 1.0), kTwoPi, 1e-12);
}

TEST(VescProtocol, UsesPythonCompatibleNearestEvenRoundingAndClamp)
{
  const auto rad_s_for_erpm = [](double erpm) {
      return erpm * kTwoPi / (60.0 * 15.0);
    };
  EXPECT_EQ(wheel_rad_s_to_erpm(rad_s_for_erpm(2.5), 1, 15, 1.0, 7500.0), 2);
  EXPECT_EQ(wheel_rad_s_to_erpm(rad_s_for_erpm(3.5), 1, 15, 1.0, 7500.0), 4);
  EXPECT_EQ(wheel_rad_s_to_erpm(rad_s_for_erpm(-2.5), 1, 15, 1.0, 7500.0), -2);
  EXPECT_EQ(wheel_rad_s_to_erpm(1.0e100, 1, 15, 1.0, 7500.0), 7500);
  EXPECT_THROW(
    wheel_rad_s_to_erpm(
      std::numeric_limits<double>::quiet_NaN(), 1, 15, 1.0, 7500.0),
    std::invalid_argument);
}

TEST(VescProtocol, HandlesSignedInt32TachometerRollover)
{
  EXPECT_EQ(signed_tachometer_delta(0x80000000U, 0x7FFFFFFFU), 1);
  EXPECT_EQ(signed_tachometer_delta(0x7FFFFFFFU, 0x80000000U), -1);
  EXPECT_EQ(signed_tachometer_delta(5U, 0xFFFFFFFBU), 10);
  EXPECT_EQ(signed_tachometer_delta(0xFFFFFFFBU, 5U), -10);
}

TEST(SeeedProtocol, InitializationMatchesPythonCanBackend)
{
  const auto frame = make_seeed_init_frame(500000);
  EXPECT_EQ(frame[0], 0xAA);
  EXPECT_EQ(frame[1], 0x55);
  EXPECT_EQ(frame[2], 0x12);
  EXPECT_EQ(frame[3], 0x03);
  EXPECT_EQ(frame[4], 0x02);
  EXPECT_EQ(frame[13], 0x00);
  EXPECT_EQ(frame[14], 0x01);
  uint32_t checksum = 0;
  for (std::size_t index = 2; index < 19; ++index) {
    checksum += frame[index];
  }
  EXPECT_EQ(frame[19], static_cast<uint8_t>(checksum));
}

TEST(SeeedProtocol, ReassemblesEveryPossibleFragmentation)
{
  CanMessage input = make_vesc_i32_command(2, kCanPacketSetRpm, -123456);
  const auto bytes = serialize_seeed_frame(input);
  for (std::size_t split = 0; split < bytes.size(); ++split) {
    SeeedSerialParser parser;
    auto first = parser.feed(bytes.data(), split);
    auto second = parser.feed(bytes.data() + split, bytes.size() - split);
    EXPECT_TRUE(first.empty());
    ASSERT_EQ(second.size(), 1U);
    EXPECT_EQ(second.front().id, input.id);
    EXPECT_EQ(second.front().dlc, input.dlc);
    EXPECT_EQ(decode_i32_be(second.front().data.data()), -123456);
  }
  SeeedSerialParser parser;
  const auto complete = parser.feed(bytes.data(), bytes.size());
  ASSERT_EQ(complete.size(), 1U);
}

TEST(SeeedProtocol, ResynchronizesAndKeepsBufferBounded)
{
  const auto valid = serialize_seeed_frame(make_vesc_i32_command(1, kCanPacketSetRpm, 42));
  std::vector<uint8_t> stream{0x10, 0x20, 0xAA, 0xE8, 1, 2, 3, 4, 0, 0, 0, 0, 0x00};
  stream.insert(stream.end(), valid.begin(), valid.end());
  SeeedSerialParser parser;
  const auto messages = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(messages.size(), 1U);
  EXPECT_EQ(decode_i32_be(messages.front().data.data()), 42);
  EXPECT_GT(parser.dropped_bytes(), 0U);

  std::vector<uint8_t> garbage(SeeedSerialParser::kMaximumBufferedBytes * 2U, 0x42);
  (void)parser.feed(garbage.data(), garbage.size());
  EXPECT_LE(parser.buffered_bytes(), SeeedSerialParser::kMaximumBufferedBytes);
}

TEST(SeeedProtocol, DoesNotTreatBridgeHeartbeatAsVescTraffic)
{
  const auto heartbeat = make_bridge_heartbeat(7);
  EXPECT_TRUE(is_bridge_heartbeat(heartbeat));
  const auto serialized = serialize_seeed_frame(heartbeat);
  SeeedSerialParser parser;
  const auto messages = parser.feed(serialized.data(), serialized.size());
  ASSERT_EQ(messages.size(), 1U);
  EXPECT_TRUE(is_bridge_heartbeat(messages.front()));
}

}  // namespace
}  // namespace iwalk_hardware
