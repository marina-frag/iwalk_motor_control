#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace iwalk_hardware
{
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;
using Clock = std::chrono::steady_clock;

class VescSystem : public hardware_interface::SystemInterface
{
  struct Wheel
  {
    std::string name;
    int id = 0;
    double poles = 0.0;
    double ratio = 0.0;
    double direction = 1.0;
    double max_erpm = 0.0;

    double command = 0.0;
    double position = 0.0;
    double velocity = 0.0;

    uint32_t previous_tacho = 0;
    bool has_rpm = false;
    bool has_tacho = false;
    Clock::time_point rpm_time{};
    Clock::time_point tacho_time{};
  };

  static constexpr double TWO_PI = 6.28318530717958647692;
  std::array<Wheel, 2> wheels_{};
  std::string can_interface_;
  double feedback_timeout_ = 0.2;
  int socket_ = -1;
  bool active_ = false;

  static uint32_t decode_u32(const uint8_t * p)
  {
    return (uint32_t(p[0]) << 24) |
           (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8) |
           uint32_t(p[3]);
  }

  static int64_t signed_u32(uint32_t value)
  {
    return value <= 0x7fffffffU ?
           int64_t(value) : int64_t(value) - 0x100000000LL;
  }

  static void encode_i32(uint8_t * p, int32_t value)
  {
    const auto u = static_cast<uint32_t>(value);
    p[0] = static_cast<uint8_t>(u >> 24);
    p[1] = static_cast<uint8_t>(u >> 16);
    p[2] = static_cast<uint8_t>(u >> 8);
    p[3] = static_cast<uint8_t>(u);
  }

  bool send_rpm(const Wheel & wheel, int32_t erpm)
  {
    if (socket_ < 0) {
      return false;
    }
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | (3U << 8) | uint32_t(wheel.id);
    frame.can_dlc = 4;
    encode_i32(frame.data, erpm);
    return ::write(socket_, &frame, sizeof(frame)) ==
           static_cast<ssize_t>(sizeof(frame));
  }

  void stop_commands()
  {
    active_ = false;
    for (auto & wheel : wheels_) {
      wheel.command = 0.0;
      if (socket_ >= 0) {
        send_rpm(wheel, 0);
      }
    }
  }

  void close_socket()
  {
    if (socket_ >= 0) {
      ::close(socket_);
      socket_ = -1;
    }
  }

  bool receive_feedback()
  {
    if (socket_ < 0) {
      return false;
    }

    // Bounded, non-blocking receive loop.
    for (int n = 0; n < 256; ++n) {
      can_frame frame{};
      const ssize_t size = ::read(socket_, &frame, sizeof(frame));
      if (size < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return true;
        }
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      if (size != static_cast<ssize_t>(sizeof(frame))) {
        return false;
      }
      if (
        !(frame.can_id & CAN_EFF_FLAG) ||
        (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) ||
        frame.can_dlc != 8)
      {
        continue;
      }

      const uint32_t identifier = frame.can_id & CAN_EFF_MASK;
      const uint32_t packet = identifier >> 8;
      const int id = int(identifier & 0xffU);
      const auto now = Clock::now();

      for (auto & wheel : wheels_) {
        if (wheel.id != id) {
          continue;
        }

        if (packet == 9U) {
          const double erpm = double(signed_u32(decode_u32(frame.data)));
          wheel.velocity =
            wheel.direction * erpm * TWO_PI /
            (60.0 * wheel.poles * wheel.ratio);
          wheel.has_rpm = true;
          wheel.rpm_time = now;
        } else if (packet == 27U) {
          const uint32_t current = decode_u32(frame.data);
          if (wheel.has_tacho) {
            // Modular subtraction handles signed int32 rollover.
            const int64_t delta =
              signed_u32(current - wheel.previous_tacho);
            wheel.position +=
              wheel.direction * double(delta) * TWO_PI /
              (6.0 * wheel.poles * wheel.ratio);
          }
          // First sample establishes the position reference.
          wheel.previous_tacho = current;
          wheel.has_tacho = true;
          wheel.tacho_time = now;
        }
      }
    }
    return true;
  }

  bool feedback_fresh() const
  {
    const auto now = Clock::now();
    for (const auto & wheel : wheels_) {
      if (!wheel.has_rpm || !wheel.has_tacho) {
        return false;
      }
      if (
        std::chrono::duration<double>(now - wheel.rpm_time).count() >
        feedback_timeout_ ||
        std::chrono::duration<double>(now - wheel.tacho_time).count() >
        feedback_timeout_)
      {
        return false;
      }
    }
    return true;
  }

public:
  ~VescSystem() override
  {
    stop_commands();
    close_socket();
  }

   CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    try {
      if (info_.joints.size() != 2) {
        throw std::runtime_error("Exactly two driven wheel joints are required");
      }
      can_interface_ = info_.hardware_parameters.at("can_interface");
      feedback_timeout_ =
        std::stod(info_.hardware_parameters.at("feedback_timeout"));

      if (
        !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0.0 ||
        can_interface_.empty() || can_interface_.size() >= IFNAMSIZ)
      {
        throw std::runtime_error("Invalid CAN interface or feedback timeout");
      }

      for (size_t i = 0; i < wheels_.size(); ++i) {
        const auto & joint = info_.joints[i];
        auto & wheel = wheels_[i];

        if (
          joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != "velocity" ||
          joint.state_interfaces.size() != 2)
        {
          throw std::runtime_error("Expected velocity command and position/velocity states");
        }
        bool position = false;
        bool velocity = false;
        for (const auto & state : joint.state_interfaces) {
          position |= state.name == "position";
          velocity |= state.name == "velocity";
        }
        if (!position || !velocity) {
          throw std::runtime_error("Missing position/velocity state interface");
        }

        wheel.name = joint.name;
        wheel.id = std::stoi(joint.parameters.at("vesc_id"));
        wheel.poles = std::stod(joint.parameters.at("pole_pairs"));
        wheel.ratio = std::stod(joint.parameters.at("gear_ratio"));
        wheel.direction = std::stod(joint.parameters.at("direction"));
        wheel.max_erpm = std::stod(joint.parameters.at("max_erpm"));

        if (
          wheel.id < 0 || wheel.id > 254 ||
          !std::isfinite(wheel.poles) || wheel.poles < 1.0 ||
          std::floor(wheel.poles) != wheel.poles ||
          !std::isfinite(wheel.ratio) || wheel.ratio <= 0.0 ||
          (wheel.direction != 1.0 && wheel.direction != -1.0) ||
          !std::isfinite(wheel.max_erpm) ||
          wheel.max_erpm <= 0.0 || wheel.max_erpm > 2147483647.0)
        {
          throw std::runtime_error("Invalid wheel parameters");
        }
      }
      if (wheels_[0].id == wheels_[1].id) {
        throw std::runtime_error("VESC IDs must be different");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("iwalk_hardware"), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> result;
    for (auto & wheel : wheels_) {
      result.emplace_back(wheel.name, "position", &wheel.position);
      result.emplace_back(wheel.name, "velocity", &wheel.velocity);
    }
    return result;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> result;
    for (auto & wheel : wheels_) {
      result.emplace_back(wheel.name, "velocity", &wheel.command);
    }
    return result;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    stop_commands();
    close_socket();

    socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_ < 0) {
      return CallbackReturn::ERROR;
    }

    const unsigned int index = if_nametoindex(can_interface_.c_str());
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);

    std::array<can_filter, 4> filters{};
    size_t n = 0;
    for (const auto & wheel : wheels_) {
      for (const uint32_t packet : {9U, 27U}) {
        filters[n].can_id =
          CAN_EFF_FLAG | (packet << 8) | uint32_t(wheel.id);
        filters[n].can_mask =
          CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK;
        ++n;
      }
    }

    if (
      index == 0 ||
      ::setsockopt(
        socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
        filters.data(), sizeof(filters)) < 0 ||
      ::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("iwalk_hardware"),
        "Cannot configure SocketCAN interface %s", can_interface_.c_str());
      close_socket();
      return CallbackReturn::ERROR;
    }

    for (auto & wheel : wheels_) {
      wheel.command = 0.0;
      wheel.position = 0.0;
      wheel.velocity = 0.0;
      wheel.has_rpm = false;
      wheel.has_tacho = false;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    stop_commands();

    // Require fresh feedback from BOTH VESCs before enabling commands.
    for (auto & wheel : wheels_) {
      wheel.has_rpm = false;
      wheel.has_tacho = false;
    }

    const auto deadline = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < deadline) {
      if (!receive_feedback()) {
        return CallbackReturn::ERROR;
      }
      if (feedback_fresh()) {
        active_ = true;
        return CallbackReturn::SUCCESS;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("iwalk_hardware"),
      "Activation failed: fresh STATUS and STATUS_5 required from both VESCs");
    return CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop_commands();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    stop_commands();
    close_socket();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    stop_commands();
    close_socket();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    stop_commands();
    close_socket();
    return CallbackReturn::SUCCESS;
  }

  ReturnType read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!receive_feedback() || (active_ && !feedback_fresh())) {
      stop_commands();
      RCLCPP_ERROR(
        rclcpp::get_logger("iwalk_hardware"),
        "CAN receive failure or stale VESC feedback");
      return ReturnType::ERROR;
    }
    return ReturnType::OK;
  }

  ReturnType write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {
      return ReturnType::OK;
    }

    std::array<int32_t, 2> commands{};
    for (size_t i = 0; i < wheels_.size(); ++i) {
      const auto & wheel = wheels_[i];
      double erpm =
        wheel.direction * wheel.command *
        wheel.ratio * wheel.poles * 60.0 / TWO_PI;

      if (!std::isfinite(erpm)) {
        stop_commands();
        return ReturnType::ERROR;
      }
      erpm = std::clamp(erpm, -wheel.max_erpm, wheel.max_erpm);
      commands[i] = static_cast<int32_t>(erpm);
    }

    for (size_t i = 0; i < wheels_.size(); ++i) {
      if (!send_rpm(wheels_[i], commands[i])) {
        stop_commands();
        return ReturnType::ERROR;
      }
    }
    return ReturnType::OK;
  }
};
}  // namespace iwalk_hardware

PLUGINLIB_EXPORT_CLASS(
  iwalk_hardware::VescSystem,
  hardware_interface::SystemInterface)
