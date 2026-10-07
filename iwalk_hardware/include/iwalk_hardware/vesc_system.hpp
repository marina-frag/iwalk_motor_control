#ifndef IWALK_HARDWARE__VESC_SYSTEM_HPP_
#define IWALK_HARDWARE__VESC_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace iwalk_hardware
{

class VescSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  using ReturnType = hardware_interface::return_type;

  ~VescSystem() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ReturnType read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  ReturnType write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  struct Wheel
  {
    std::string name;
    uint8_t id = 0;
    int pole_pairs = 0;
    double gear_ratio = 0.0;
    int direction = 1;
    double max_erpm = 0.0;

    double command = 0.0;
    double position = 0.0;
    double velocity = 0.0;

    uint32_t previous_tachometer = 0;
    bool needs_tachometer_reference = true;
    bool has_status1 = false;
    bool has_status5 = false;
    Clock::time_point status1_time{};
    Clock::time_point status5_time{};
  };

  bool open_socket();
  void close_socket();
  bool receive_feedback();
  bool feedback_fresh(std::string * reason = nullptr) const;
  bool send_i32_command(const Wheel & wheel, uint32_t packet_id, int32_t value);
  void stop_commands(const char * reason);
  void reset_feedback_references();

  std::vector<Wheel> wheels_;
  std::vector<int32_t> pending_erpm_commands_;
  std::string can_interface_;
  double feedback_timeout_ = 0.2;
  bool monitor_only_ = false;
  bool bridge_heartbeat_required_ = false;
  bool bridge_heartbeat_seen_ = false;
  bool feedback_fault_latched_ = false;
  Clock::time_point bridge_heartbeat_time_{};
  int socket_ = -1;
  bool active_ = false;
};

}  // namespace iwalk_hardware

#endif  // IWALK_HARDWARE__VESC_SYSTEM_HPP_
