#include "iwalk_hardware/vesc_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "iwalk_hardware/vesc_protocol.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace iwalk_hardware
{
namespace
{

const auto kLogger = rclcpp::get_logger("iwalk_hardware.VescSystem");

std::string require_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & owner)
{
  const auto found = parameters.find(name);
  if (found == parameters.end() || found->second.empty()) {
    throw std::runtime_error(owner + " parameter '" + name + "' is missing or empty");
  }
  return found->second;
}

int parse_integer(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & owner)
{
  const std::string text = require_parameter(parameters, name, owner);
  int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::runtime_error(
            owner + " parameter '" + name + "' must be an integer, got '" + text + "'");
  }
  return value;
}

double parse_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & owner)
{
  const std::string text = require_parameter(parameters, name, owner);
  char * end = nullptr;
  errno = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (errno == ERANGE || end != text.c_str() + text.size() || !std::isfinite(value)) {
    throw std::runtime_error(
            owner + " parameter '" + name + "' must be a finite number, got '" + text + "'");
  }
  return value;
}

bool parse_boolean(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & owner)
{
  const std::string text = require_parameter(parameters, name, owner);
  if (text == "true" || text == "1") {
    return true;
  }
  if (text == "false" || text == "0") {
    return false;
  }
  throw std::runtime_error(
          owner + " parameter '" + name + "' must be true or false, got '" + text + "'");
}

bool valid_interface_name(const std::string & name)
{
  if (name.empty() || name.size() >= IFNAMSIZ) {
    return false;
  }
  return std::all_of(name.begin(), name.end(), [](unsigned char character) {
      return std::isalnum(character) != 0 || character == '_' || character == '-' ||
             character == '.';
    });
}

CanMessage from_linux_can_frame(const can_frame & frame)
{
  CanMessage message;
  message.extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  message.remote = (frame.can_id & CAN_RTR_FLAG) != 0;
  message.id = frame.can_id & (message.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  message.dlc = std::min<uint8_t>(frame.can_dlc, 8U);
  std::copy_n(frame.data, message.dlc, message.data.begin());
  return message;
}

}  // namespace

VescSystem::~VescSystem()
{
  stop_commands("destructor");
  close_socket();
}

VescSystem::CallbackReturn VescSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    if (info_.joints.empty() || info_.joints.size() > 2U) {
      throw std::runtime_error(
              "hardware must declare one or two VESC wheel joints; found " +
              std::to_string(info_.joints.size()));
    }

    const std::string hardware_owner = "hardware '" + info_.name + "'";
    can_interface_ = require_parameter(
      info_.hardware_parameters, "can_interface", hardware_owner);
    if (!valid_interface_name(can_interface_)) {
      throw std::runtime_error(
              hardware_owner + " parameter 'can_interface' has invalid Linux interface name '" +
              can_interface_ + "'");
    }
    feedback_timeout_ = parse_double(
      info_.hardware_parameters, "feedback_timeout", hardware_owner);
    if (feedback_timeout_ <= 0.0 || feedback_timeout_ > 10.0) {
      throw std::runtime_error(
              hardware_owner + " parameter 'feedback_timeout' must be in (0, 10] seconds");
    }
    monitor_only_ = parse_boolean(
      info_.hardware_parameters, "monitor_only", hardware_owner);
    bridge_heartbeat_required_ = can_interface_.rfind("vcan", 0) == 0;

    wheels_.clear();
    wheels_.reserve(info_.joints.size());
    std::set<int> ids;
    for (const auto & joint : info_.joints) {
      const std::string owner = "joint '" + joint.name + "'";
      if (joint.command_interfaces.size() != 1U ||
        joint.command_interfaces.front().name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::runtime_error(
                owner + " must declare exactly one 'velocity' command interface");
      }
      if (joint.state_interfaces.size() != 2U) {
        throw std::runtime_error(
                owner + " must declare exactly 'position' and 'velocity' state interfaces");
      }
      bool has_position = false;
      bool has_velocity = false;
      for (const auto & state : joint.state_interfaces) {
        has_position = has_position || state.name == hardware_interface::HW_IF_POSITION;
        has_velocity = has_velocity || state.name == hardware_interface::HW_IF_VELOCITY;
      }
      if (!has_position || !has_velocity) {
        throw std::runtime_error(
                owner + " must declare 'position' and 'velocity' state interfaces");
      }

      Wheel wheel;
      wheel.name = joint.name;
      const int id = parse_integer(joint.parameters, "vesc_id", owner);
      if (id < 0 || id > 254) {
        throw std::runtime_error(owner + " parameter 'vesc_id' must be in [0, 254]");
      }
      if (!ids.insert(id).second) {
        throw std::runtime_error(
                owner + " parameter 'vesc_id' duplicates VESC ID " + std::to_string(id));
      }
      wheel.id = static_cast<uint8_t>(id);

      wheel.pole_pairs = parse_integer(joint.parameters, "pole_pairs", owner);
      if (wheel.pole_pairs < 1 || wheel.pole_pairs > 1000) {
        throw std::runtime_error(owner + " parameter 'pole_pairs' must be in [1, 1000]");
      }
      wheel.gear_ratio = parse_double(joint.parameters, "gear_ratio", owner);
      if (wheel.gear_ratio <= 0.0 || wheel.gear_ratio > 1.0e6) {
        throw std::runtime_error(owner + " parameter 'gear_ratio' must be in (0, 1e6]");
      }
      wheel.direction = parse_integer(joint.parameters, "direction", owner);
      if (wheel.direction != -1 && wheel.direction != 1) {
        throw std::runtime_error(owner + " parameter 'direction' must be -1 or 1");
      }
      wheel.max_erpm = parse_double(joint.parameters, "max_erpm", owner);
      if (wheel.max_erpm <= 0.0 ||
        wheel.max_erpm > static_cast<double>(std::numeric_limits<int32_t>::max()) ||
        std::floor(wheel.max_erpm) != wheel.max_erpm)
      {
        throw std::runtime_error(
                owner + " parameter 'max_erpm' must be an integer in [1, INT32_MAX]");
      }

      const std::string sensor = require_parameter(
        joint.parameters, "encoder_sensor", owner);
      if (sensor != "hall") {
        throw std::runtime_error(
                owner + " parameter 'encoder_sensor'='" + sensor +
                "' is unsupported; this implementation requires 'hall'");
      }
      const std::string feedback = require_parameter(
        joint.parameters, "feedback_source", owner);
      if (feedback != "vesc_status") {
        throw std::runtime_error(
                owner + " parameter 'feedback_source'='" + feedback +
                "' is unsupported; this implementation requires 'vesc_status'");
      }

      wheels_.push_back(std::move(wheel));
    }
    // No operation after this point resizes wheels_; exported value addresses stay stable.
    pending_erpm_commands_.assign(wheels_.size(), 0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "VESC configuration error: %s", error.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    kLogger, "Configured %zu VESC joint(s) on %s (monitor_only=%s%s)",
    wheels_.size(), can_interface_.c_str(), monitor_only_ ? "true" : "false",
    bridge_heartbeat_required_ ? ", Waveshare bridge heartbeat required" : "");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VescSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(wheels_.size() * 2U);
  for (auto & wheel : wheels_) {
    interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_POSITION, &wheel.position);
    interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> VescSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(wheels_.size());
  for (auto & wheel : wheels_) {
    interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.command);
  }
  return interfaces;
}

bool VescSystem::open_socket()
{
  socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_ < 0) {
    RCLCPP_ERROR(kLogger, "Cannot create SocketCAN socket: %s", std::strerror(errno));
    return false;
  }

  const int receive_own_messages = 0;
  if (::setsockopt(
      socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
      &receive_own_messages, sizeof(receive_own_messages)) < 0)
  {
    RCLCPP_ERROR(kLogger, "Cannot disable own-message reception: %s", std::strerror(errno));
    close_socket();
    return false;
  }

  std::vector<can_filter> filters;
  filters.reserve(wheels_.size() * 2U + (bridge_heartbeat_required_ ? 1U : 0U));
  for (const auto & wheel : wheels_) {
    for (const uint32_t packet : {kCanPacketStatus1, kCanPacketStatus5}) {
      can_filter filter{};
      filter.can_id = CAN_EFF_FLAG | (packet << 8) | uint32_t(wheel.id);
      filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK;
      filters.push_back(filter);
    }
  }
  if (bridge_heartbeat_required_) {
    can_filter filter{};
    filter.can_id = CAN_EFF_FLAG | kBridgeHeartbeatCanId;
    filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK;
    filters.push_back(filter);
  }

  const unsigned int index = if_nametoindex(can_interface_.c_str());
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(index);
  if (index == 0 ||
    ::setsockopt(
      socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0 ||
    ::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
  {
    RCLCPP_ERROR(
      kLogger, "Cannot configure SocketCAN interface '%s': %s",
      can_interface_.c_str(), index == 0 ? "interface not found" : std::strerror(errno));
    close_socket();
    return false;
  }
  return true;
}

void VescSystem::close_socket()
{
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool VescSystem::send_i32_command(const Wheel & wheel, uint32_t packet_id, int32_t value)
{
  if (socket_ < 0 || monitor_only_) {
    return monitor_only_;
  }
  const CanMessage message = make_vesc_i32_command(wheel.id, packet_id, value);
  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | message.id;
  frame.can_dlc = message.dlc;
  std::copy_n(message.data.begin(), message.dlc, frame.data);
  const ssize_t written = ::write(socket_, &frame, sizeof(frame));
  return written == static_cast<ssize_t>(sizeof(frame));
}

void VescSystem::stop_commands(const char * reason)
{
  active_ = false;
  for (auto & wheel : wheels_) {
    wheel.command = 0.0;
  }
  if (monitor_only_ || socket_ < 0) {
    return;
  }

  bool all_sent = true;
  // SET_CURRENT 0 requests zero motor current (coast with the default VESC configuration).
  // It is intentionally not described as braking; VESC timeout configuration remains essential.
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (const auto & wheel : wheels_) {
      all_sent = send_i32_command(wheel, kCanPacketSetCurrent, 0) && all_sent;
    }
  }
  if (!all_sent) {
    RCLCPP_WARN(
      kLogger,
      "Could not enqueue every SET_CURRENT 0 stop frame during %s; physical CAN delivery is not guaranteed",
      reason);
  }
}

void VescSystem::reset_feedback_references()
{
  bridge_heartbeat_seen_ = false;
  feedback_fault_latched_ = false;
  for (auto & wheel : wheels_) {
    wheel.has_status1 = false;
    wheel.has_status5 = false;
    wheel.needs_tachometer_reference = true;
  }
}

bool VescSystem::receive_feedback()
{
  if (socket_ < 0) {
    return false;
  }
  for (int count = 0; count < 256; ++count) {
    can_frame frame{};
    const ssize_t size = ::read(socket_, &frame, sizeof(frame));
    if (size < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return !feedback_fault_latched_;
      }
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(kLogger, "SocketCAN read failed: %s", std::strerror(errno));
      return false;
    }
    if (size != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR(kLogger, "SocketCAN returned a truncated classic CAN frame");
      return false;
    }
    if ((frame.can_id & CAN_ERR_FLAG) != 0) {
      RCLCPP_ERROR(kLogger, "SocketCAN reported CAN error frame 0x%08X", frame.can_id);
      return false;
    }

    const CanMessage message = from_linux_can_frame(frame);
    const auto now = Clock::now();
    if (is_bridge_heartbeat(message)) {
      bridge_heartbeat_seen_ = true;
      bridge_heartbeat_time_ = now;
      continue;
    }

    for (auto & wheel : wheels_) {
      Status1 status1;
      if (decode_status1(message, wheel.id, status1)) {
        wheel.velocity = erpm_to_wheel_rad_s(
          status1.erpm, wheel.direction, wheel.pole_pairs, wheel.gear_ratio);
        wheel.has_status1 = true;
        wheel.status1_time = now;
        break;
      }

      Status5 status5;
      if (!decode_status5(message, wheel.id, status5)) {
        continue;
      }
      const uint32_t current = static_cast<uint32_t>(status5.tachometer);
      if (wheel.needs_tachometer_reference) {
        // Preserve accumulated ROS position; only establish a new VESC counter reference.
        wheel.previous_tachometer = current;
        wheel.needs_tachometer_reference = false;
      } else {
        const int64_t delta = signed_tachometer_delta(current, wheel.previous_tachometer);
        const double elapsed = std::chrono::duration<double>(now - wheel.status5_time).count();
        const double plausible_counts =
          wheel.max_erpm * std::max(elapsed, 0.0) / 10.0 * 1.5 + 12.0;
        if (std::fabs(static_cast<double>(delta)) > plausible_counts) {
          RCLCPP_ERROR(
            kLogger,
            "Joint '%s' STATUS_5 tachometer discontinuity: delta=%ld exceeds %.1f plausible counts; preserving position and requiring reactivation",
            wheel.name.c_str(), static_cast<long>(delta), plausible_counts);
          wheel.previous_tachometer = current;
          wheel.needs_tachometer_reference = true;
          wheel.has_status5 = false;
          feedback_fault_latched_ = true;
          return false;
        }
        wheel.position += tachometer_delta_to_wheel_rad(
          delta, wheel.direction, wheel.pole_pairs, wheel.gear_ratio);
        wheel.previous_tachometer = current;
      }
      wheel.has_status5 = true;
      wheel.status5_time = now;
      break;
    }
  }
  return !feedback_fault_latched_;
}

bool VescSystem::feedback_fresh(std::string * reason) const
{
  const auto now = Clock::now();
  if (feedback_fault_latched_) {
    if (reason != nullptr) {
      *reason = "tachometer discontinuity is latched";
    }
    return false;
  }
  if (bridge_heartbeat_required_ &&
    (!bridge_heartbeat_seen_ ||
    std::chrono::duration<double>(now - bridge_heartbeat_time_).count() > feedback_timeout_))
  {
    if (reason != nullptr) {
      *reason = "Waveshare bridge heartbeat is missing or stale";
    }
    return false;
  }
  for (const auto & wheel : wheels_) {
    if (!wheel.has_status1) {
      if (reason != nullptr) {
        *reason = "joint '" + wheel.name + "' has no STATUS_1";
      }
      return false;
    }
    if (!wheel.has_status5) {
      if (reason != nullptr) {
        *reason = "joint '" + wheel.name + "' has no STATUS_5";
      }
      return false;
    }
    if (std::chrono::duration<double>(now - wheel.status1_time).count() > feedback_timeout_) {
      if (reason != nullptr) {
        *reason = "joint '" + wheel.name + "' STATUS_1 is stale";
      }
      return false;
    }
    if (std::chrono::duration<double>(now - wheel.status5_time).count() > feedback_timeout_) {
      if (reason != nullptr) {
        *reason = "joint '" + wheel.name + "' STATUS_5 is stale";
      }
      return false;
    }
  }
  return true;
}

VescSystem::CallbackReturn VescSystem::on_configure(const rclcpp_lifecycle::State &)
{
  stop_commands("configure");
  close_socket();
  if (!open_socket()) {
    return CallbackReturn::ERROR;
  }
  for (auto & wheel : wheels_) {
    wheel.command = 0.0;
    wheel.velocity = 0.0;
    // wheel.position is deliberately preserved across reconfigure/reactivate.
  }
  reset_feedback_references();
  return CallbackReturn::SUCCESS;
}

VescSystem::CallbackReturn VescSystem::on_activate(const rclcpp_lifecycle::State &)
{
  stop_commands("activate");
  reset_feedback_references();

  const auto deadline = Clock::now() + std::chrono::seconds(2);
  std::string reason;
  while (Clock::now() < deadline) {
    if (!receive_feedback()) {
      stop_commands("activation receive failure");
      return CallbackReturn::ERROR;
    }
    if (feedback_fresh(&reason)) {
      active_ = true;
      RCLCPP_INFO(
        kLogger, "Activated with fresh STATUS_1 and STATUS_5 from %zu declared VESC(s)",
        wheels_.size());
      return CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  stop_commands("activation timeout");
  RCLCPP_ERROR(kLogger, "Activation failed: %s", reason.c_str());
  return CallbackReturn::ERROR;
}

VescSystem::CallbackReturn VescSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_commands("deactivate");
  return CallbackReturn::SUCCESS;
}

VescSystem::CallbackReturn VescSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_commands("cleanup");
  close_socket();
  return CallbackReturn::SUCCESS;
}

VescSystem::CallbackReturn VescSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_commands("shutdown");
  close_socket();
  return CallbackReturn::SUCCESS;
}

VescSystem::CallbackReturn VescSystem::on_error(const rclcpp_lifecycle::State &)
{
  stop_commands("error");
  close_socket();
  return CallbackReturn::SUCCESS;
}

VescSystem::ReturnType VescSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string reason;
  const bool received = receive_feedback();
  const bool fresh = !active_ || feedback_fresh(&reason);
  if (!received || !fresh) {
    if (reason.empty()) {
      (void)feedback_fresh(&reason);
    }
    stop_commands("read error");
    RCLCPP_ERROR(
      kLogger, "VESC feedback failure on '%s': %s",
      can_interface_.c_str(), reason.empty() ? "SocketCAN receive failed" : reason.c_str());
    return ReturnType::ERROR;
  }
  return ReturnType::OK;
}

VescSystem::ReturnType VescSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || monitor_only_) {
    return ReturnType::OK;
  }
  std::string reason;
  if (!feedback_fresh(&reason)) {
    stop_commands("write with stale feedback");
    RCLCPP_ERROR(kLogger, "Refusing VESC commands: %s", reason.c_str());
    return ReturnType::ERROR;
  }

  try {
    for (std::size_t index = 0; index < wheels_.size(); ++index) {
      const auto & wheel = wheels_[index];
      pending_erpm_commands_[index] = wheel_rad_s_to_erpm(
        wheel.command, wheel.direction, wheel.pole_pairs, wheel.gear_ratio,
        wheel.max_erpm);
    }
  } catch (const std::exception & error) {
    stop_commands("invalid command");
    RCLCPP_ERROR(kLogger, "Invalid wheel velocity command: %s", error.what());
    return ReturnType::ERROR;
  }

  for (std::size_t index = 0; index < wheels_.size(); ++index) {
    if (!send_i32_command(wheels_[index], kCanPacketSetRpm, pending_erpm_commands_[index])) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to enqueue SET_RPM for joint '%s' on %s; this does not confirm physical CAN delivery",
        wheels_[index].name.c_str(), can_interface_.c_str());
      stop_commands("SET_RPM write failure");
      return ReturnType::ERROR;
    }
  }
  return ReturnType::OK;
}

}  // namespace iwalk_hardware

PLUGINLIB_EXPORT_CLASS(
  iwalk_hardware::VescSystem,
  hardware_interface::SystemInterface)
