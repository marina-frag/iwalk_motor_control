#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "hardware_interface/hardware_info.hpp"
#include "iwalk_hardware/vesc_protocol.hpp"
#include "iwalk_hardware/vesc_system.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace iwalk_hardware
{
namespace
{
using namespace std::chrono_literals;

hardware_interface::InterfaceInfo interface(const std::string & name)
{
  hardware_interface::InterfaceInfo result{};
  result.name = name;
  result.size = 1;
  result.enable_limits = true;
  return result;
}

hardware_interface::HardwareInfo make_info(std::size_t wheel_count, bool monitor_only = false)
{
  hardware_interface::HardwareInfo info{};
  info.name = "IWalkVescSystem";
  info.type = "system";
  info.rw_rate = 100;
  info.is_async = false;
  info.hardware_plugin_name = "iwalk_hardware/VescSystem";
  info.hardware_parameters = {
    {"can_interface", "vcan0"},
    {"feedback_timeout", "0.2"},
    {"monitor_only", monitor_only ? "true" : "false"},
  };
  for (std::size_t index = 0; index < wheel_count; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = index == 0 ? "left_rear_wheel_joint" : "right_rear_wheel_joint";
    joint.type = "joint";
    joint.command_interfaces = {interface("velocity")};
    joint.state_interfaces = {interface("position"), interface("velocity")};
    joint.parameters = {
      {"vesc_id", std::to_string(index + 1)},
      {"pole_pairs", "15"},
      {"gear_ratio", "1.0"},
      {"direction", "1"},
      {"max_erpm", "7500"},
      {"encoder_sensor", "hall"},
      {"feedback_source", "vesc_status"},
    };
    info.joints.push_back(joint);
  }
  return info;
}

hardware_interface::HardwareComponentInterfaceParams make_params(
  hardware_interface::HardwareInfo info)
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = std::move(info);
  return params;
}

class FakeCanPeers
{
public:
  explicit FakeCanPeers(std::size_t count)
  : count_(count), tachometers_(count, 0), last_rpm_commands_(count, 0)
  {
    socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_ < 0) {
      return;
    }
    const int own = 0;
    (void)::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof(own));
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(::if_nametoindex("vcan0"));
    if (address.can_ifindex == 0 ||
      ::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
      ::close(socket_);
      socket_ = -1;
      return;
    }
    thread_ = std::thread([this]() {run();});
  }

  ~FakeCanPeers()
  {
    running_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  bool valid() const {return socket_ >= 0;}
  void stop_feedback() {publish_feedback_.store(false);}
  int motor_command_count() const {return motor_command_count_.load();}
  int32_t rpm_command(std::size_t index) const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return last_rpm_commands_.at(index);
  }

private:
  void send(const CanMessage & message)
  {
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | message.id;
    frame.can_dlc = message.dlc;
    std::copy_n(message.data.begin(), message.dlc, frame.data);
    (void)::write(socket_, &frame, sizeof(frame));
  }

  void publish()
  {
    send(make_bridge_heartbeat(sequence_++));
    for (std::size_t index = 0; index < count_; ++index) {
      const uint8_t id = static_cast<uint8_t>(index + 1);
      CanMessage status1;
      status1.id = (kCanPacketStatus1 << 8) | id;
      status1.dlc = 8;
      encode_i32_be(status1.data.data(), index == 0 ? 1500 : -1500);
      send(status1);

      CanMessage status5;
      status5.id = (kCanPacketStatus5 << 8) | id;
      status5.dlc = 8;
      encode_i32_be(status5.data.data(), tachometers_[index]++);
      status5.data[4] = 0;
      status5.data[5] = 240;
      send(status5);
    }
  }

  void receive_commands()
  {
    for (int attempt = 0; attempt < 64; ++attempt) {
      can_frame frame{};
      const ssize_t size = ::read(socket_, &frame, sizeof(frame));
      if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      if (size != static_cast<ssize_t>(sizeof(frame)) ||
        (frame.can_id & CAN_EFF_FLAG) == 0 || (frame.can_id & CAN_ERR_FLAG) != 0)
      {
        return;
      }
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      const uint32_t packet = id >> 8;
      const uint32_t node = id & 0xFFU;
      if (node < 1U || node > count_ || frame.can_dlc != 4) {
        continue;
      }
      if (packet == kCanPacketSetDuty || packet == kCanPacketSetCurrent ||
        packet == kCanPacketSetCurrentBrake || packet == kCanPacketSetRpm)
      {
        motor_command_count_.fetch_add(1);
      }
      if (packet == kCanPacketSetRpm) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_rpm_commands_[node - 1U] = decode_i32_be(frame.data);
      }
    }
  }

  void run()
  {
    auto next_publish = std::chrono::steady_clock::now();
    while (running_.load()) {
      receive_commands();
      if (publish_feedback_.load() && std::chrono::steady_clock::now() >= next_publish) {
        publish();
        next_publish = std::chrono::steady_clock::now() + 20ms;
      }
      std::this_thread::sleep_for(1ms);
    }
    receive_commands();
  }

  int socket_ = -1;
  std::size_t count_;
  std::vector<int32_t> tachometers_;
  mutable std::mutex command_mutex_;
  std::vector<int32_t> last_rpm_commands_;
  std::atomic_bool running_{true};
  std::atomic_bool publish_feedback_{true};
  std::atomic_int motor_command_count_{0};
  uint16_t sequence_ = 0;
  std::thread thread_;
};

TEST(VescSystemConfiguration, RejectsMissingParameter)
{
  auto info = make_info(1);
  info.joints[0].parameters.erase("gear_ratio");
  VescSystem system;
  EXPECT_EQ(system.on_init(make_params(info)), hardware_interface::CallbackReturn::ERROR);
}

TEST(VescSystemConfiguration, RejectsDuplicateIds)
{
  auto info = make_info(2);
  info.joints[1].parameters["vesc_id"] = "1";
  VescSystem system;
  EXPECT_EQ(system.on_init(make_params(info)), hardware_interface::CallbackReturn::ERROR);
}

TEST(VescSystemConfiguration, RejectsUnsupportedEncoder)
{
  auto info = make_info(1);
  info.joints[0].parameters["encoder_sensor"] = "abi";
  VescSystem system;
  EXPECT_EQ(system.on_init(make_params(info)), hardware_interface::CallbackReturn::ERROR);
}

TEST(VescSystemConfiguration, RejectsNanAndKeepsExportedAddressesStable)
{
  auto invalid = make_info(1);
  invalid.joints[0].parameters["gear_ratio"] = "nan";
  VescSystem invalid_system;
  EXPECT_EQ(
    invalid_system.on_init(make_params(invalid)), hardware_interface::CallbackReturn::ERROR);

  VescSystem system;
  ASSERT_EQ(
    system.on_init(make_params(make_info(2))), hardware_interface::CallbackReturn::SUCCESS);
  auto first_export = system.export_command_interfaces();
  auto second_export = system.export_command_interfaces();
  ASSERT_EQ(first_export.size(), 2U);
  ASSERT_EQ(second_export.size(), 2U);
  ASSERT_TRUE(first_export[0].set_value(3.25));
  ASSERT_TRUE(second_export[0].get_optional<double>().has_value());
  EXPECT_DOUBLE_EQ(*second_export[0].get_optional<double>(), 3.25);
}

TEST(VescSystemIntegration, ActivatesOneMotorWritesAndDetectsStaleFeedback)
{
  if (::if_nametoindex("vcan0") == 0) {
    GTEST_SKIP() << "vcan0 is not available";
  }
  VescSystem system;
  ASSERT_EQ(
    system.on_init(make_params(make_info(1))), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  FakeCanPeers peers(1);
  ASSERT_TRUE(peers.valid());
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = system.export_command_interfaces();
  ASSERT_TRUE(commands[0].set_value(10.471975511966));
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    hardware_interface::return_type::OK);
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(peers.rpm_command(0), 1500);

  peers.stop_feedback();
  std::this_thread::sleep_for(250ms);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    hardware_interface::return_type::ERROR);
}

TEST(VescSystemIntegration, ActivatesOnlyAfterBothDeclaredMotorsAreFresh)
{
  if (::if_nametoindex("vcan0") == 0) {
    GTEST_SKIP() << "vcan0 is not available";
  }
  VescSystem system;
  ASSERT_EQ(
    system.on_init(make_params(make_info(2))), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  FakeCanPeers peers(2);
  ASSERT_TRUE(peers.valid());
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  auto states = system.export_state_interfaces();
  ASSERT_EQ(states.size(), 4U);
  ASSERT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    hardware_interface::return_type::OK);
  EXPECT_NEAR(*states[1].get_optional<double>(), 10.471975511966, 1e-9);
  EXPECT_NEAR(*states[3].get_optional<double>(), -10.471975511966, 1e-9);
}

TEST(VescSystemIntegration, MonitorOnlyNeverSendsMotorCommands)
{
  if (::if_nametoindex("vcan0") == 0) {
    GTEST_SKIP() << "vcan0 is not available";
  }
  FakeCanPeers peers(1);
  ASSERT_TRUE(peers.valid());
  {
    VescSystem system;
    ASSERT_EQ(
      system.on_init(make_params(make_info(1, true))), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(
      system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(
      system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    auto commands = system.export_command_interfaces();
    ASSERT_TRUE(commands[0].set_value(5.0));
    EXPECT_EQ(
      system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      hardware_interface::return_type::OK);
    EXPECT_EQ(
      system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(
      system.on_shutdown(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  }
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(peers.motor_command_count(), 0);
}

TEST(WaveshareBridgeIntegration, SocketOwnFramesAreNotForwardedBack)
{
  const unsigned int index = ::if_nametoindex("vcan0");
  if (index == 0) {
    GTEST_SKIP() << "vcan0 is not available";
  }
  const int bridge_socket = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  const int observer_socket = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  ASSERT_GE(bridge_socket, 0);
  ASSERT_GE(observer_socket, 0);
  const int own = 0;
  ASSERT_EQ(
    ::setsockopt(
      bridge_socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof(own)), 0);
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(index);
  ASSERT_EQ(::bind(bridge_socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  ASSERT_EQ(::bind(observer_socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);

  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | 0x901U;
  frame.can_dlc = 8;
  ASSERT_EQ(::write(bridge_socket, &frame, sizeof(frame)), static_cast<ssize_t>(sizeof(frame)));
  pollfd bridge_poll{bridge_socket, POLLIN, 0};
  pollfd observer_poll{observer_socket, POLLIN, 0};
  EXPECT_EQ(::poll(&bridge_poll, 1, 30), 0);
  EXPECT_EQ(::poll(&observer_poll, 1, 30), 1);
  ::close(observer_socket);
  ::close(bridge_socket);
}

}  // namespace
}  // namespace iwalk_hardware
