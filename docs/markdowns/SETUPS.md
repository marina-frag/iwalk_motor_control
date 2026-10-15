# Setups: copy-paste commands

The workspace root is `~/ROS2/vesc_sim`, and the ROS packages are in
`iwalk_motor_control/`. **Every new terminal** needs both lines below. Without the second one you get
`Package 'iwalk_bringup' not found`.

```bash
source /opt/ros/lyrical/setup.bash
source ~/ROS2/vesc_sim/install/setup.bash   # after the first build
```

For the meaning of each piece, see [UNDERSTANDING.md](UNDERSTANDING.md).

---

## 1. One-time install and build

### 1.1 Packages

```bash
sudo apt install ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
  ros-lyrical-xacro ros-lyrical-robot-state-publisher \
  python3-can can-utils python3-rosdep

# rosdep, first time only on this machine:
sudo rosdep init
rosdep update
```

```bash
cd ~/ROS2/vesc_sim
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths iwalk_motor_control --ignore-src -y   # optional, catches missing deps
```

### 1.2 Two fixes needed before the first build

**a) `iwalk_description/meshes/` is missing.**

[.gitignore](iwalk_motor_control/.gitignore) excludes `*.STL`. Git doesn't
track empty folders, so `meshes/` never arrives with a clone. The install step
in `iwalk_description/CMakeLists.txt` then fails with
`ament_cmake_symlink_install_directory() can't find …/meshes`.

- **Best:** get `iwalk_body.STL` from whoever has it, and put it at
  `iwalk_motor_control/iwalk_description/meshes/iwalk_body.STL`.
- **Quick workaround:** create the folder empty:

  ```bash
  mkdir -p iwalk_motor_control/iwalk_description/meshes
  ```

  The build and ros2_control work fine without the STL; only RViz will show
  the robot body as missing.

**b) `ament_target_dependencies` no longer exists in Lyrical.**

The error is `Unknown CMake command "ament_target_dependencies"` in
`iwalk_hardware`. In
[iwalk_hardware/CMakeLists.txt](iwalk_motor_control/iwalk_hardware/CMakeLists.txt),
replace both calls:

```cmake
# was: ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_link_libraries(${PROJECT_NAME}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# inside if(BUILD_TESTING), was: ament_target_dependencies(test_vesc_system hardware_interface rclcpp rclcpp_lifecycle)
target_link_libraries(test_vesc_system
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
```

This was verified on Lyrical: it builds with no warnings and the tests pass.

The `CMake Deprecation Warning … Compatibility with CMake < 3.10` message was
harmless. It is now fixed: both `iwalk_description` and `iwalk_hardware` use
`cmake_minimum_required(VERSION 3.10)`.

### 1.3 Build

```bash
colcon build --symlink-install --base-paths iwalk_motor_control --packages-up-to iwalk_bringup
source install/setup.bash
```

> When one package fails, colcon marks the packages that were building in
> parallel as **Aborted**. Those are not separate errors; fix the **Failed**
> package first.

Run the unit tests. They need vcan0 (§2). Without it, the integration tests are
skipped.

```bash
colcon test --base-paths iwalk_motor_control --packages-select iwalk_hardware \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: 19 tests, 0 failures, and 0 skipped when vcan0 exists. No hardware
is needed. The tests create their own fake VESCs on vcan0, so `candump vcan0`
during a test run shows *test* traffic:

- `0x101` / `0x102`: SET_CURRENT 0
- `0x301`: SET_RPM 1500
- `0x901` / `0x902`: STATUS_1 (±1500)
- `0x1B01` / `0x1B02`: STATUS_5
- `0x1FFFFFFE`: heartbeat

That traffic does not come from a real motor.

---

## 2. Create vcan0 (laptop, once per boot)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set dev vcan0 up
ip -details link show vcan0
```

---

## 3. VESC Tool checklist (each VESC, once)

- App Settings → General → **CAN Mode** = `VESC`.
- **VESC ID**: left = `1`, right = `2`. They must be unique.
- **CAN Baud Rate** = `500K`. This must match the bridge's `--can-bitrate` and
  the can0 bitrate.
- **CAN Status Message Rate**: enable **STATUS 1** and **STATUS 5**, e.g.
  50 Hz / 20 Hz. Both must be faster than every 0.2 s.
- Keep the **timeout** enabled (default 0.5 s).
- The bus needs **120 Ω termination** at both ends.

---

## 4. Setup A: laptop + Waveshare + 1 real motor (development)

Wheels off the ground. E-stop within reach.

**Terminal 1, the bridge.** It needs vcan0 from §2, and nothing else may have
`/dev/ttyUSB0` open.

```bash
cd ~/ROS2/vesc_sim && source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 run iwalk_hardware vesc_waveshare_bridge \
  --can-interface vcan0 --serial-port /dev/ttyUSB0 \
  --serial-baudrate 2000000 --can-bitrate 500000
```

> If permission is denied on `/dev/ttyUSB0`, run
> `sudo usermod -aG dialout $USER`, then log out and back in.

**Terminal 2, check that real traffic is on vcan0 before starting ROS:**

```bash
candump vcan0
# expect 0x0901 (STATUS_1, ID 1), 0x1B01 (STATUS_5, ID 1), 0x1FFFFFFE (bridge heartbeat)
```

Or use the Python script on vcan0 (it reads the same traffic through the
bridge):

```bash
python3 single_motor_ws/can_test/can_send_test_no_socketcan.py \
  --interface socketcan --port vcan0 --vesc-id 1 monitor
```

**Terminal 3, the ros2_control stack.** Use `monitor_only:=true` the first
time: it reads feedback and never sends motor commands.

```bash
cd ~/ROS2/vesc_sim && source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 launch iwalk_bringup vesc_control.launch.py \
  can_interface:=vcan0 single_motor:=true monitor_only:=true \
  left_vesc_id:=1 left_direction:=1
```

Check:

```bash
ros2 topic echo /joint_states          # position/velocity change when you turn the wheel by hand
ros2 control list_hardware_components  # IWalkVescSystem should be active
```

Then relaunch with `monitor_only:=false` and command the wheel speed in rad/s:

```bash
ros2 topic pub --rate 20 /test_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
# then e.g. [7.0]  (≈1000 ERPM)
```

> **Small values do nothing.** With 15 pole pairs and gear ratio 1, 1 rad/s
> becomes only 143 ERPM. The VESC speed controller ignores SET_RPM below its
> **Minimum ERPM** setting (VESC Tool → Motor Settings → PID Controllers →
> Speed Controller; default 900). Use at least ~7 rad/s (≈1000 ERPM) or lower
> that setting. The maximum is 52.36 rad/s = 7500 ERPM.
>
> You can check that the command leaves the plugin with `candump vcan0`. Look
> for `00000301 [4] ..`, where the 4 bytes are the ERPM, big-endian signed. For
> example, `00 00 00 8F` = 143 and `00 00 03 EB` = 1003.

**Shutdown order:**

1. Stop publishing commands, or send `0.0`.
2. Ctrl-C the launch.
3. Ctrl-C the bridge.

---

## 5. Setup B: laptop, no hardware (full simulation)

Needs vcan0 (§2). **Do not run the bridge at the same time.**

```bash
cd ~/ROS2/vesc_sim && source /opt/ros/lyrical/setup.bash && source install/setup.bash
# one motor
ros2 launch iwalk_bringup fake_control.launch.py single_motor:=true can_interface:=vcan0
ros2 topic pub --rate 20 /test_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [2.0]}"

# two motors + diff drive
ros2 launch iwalk_bringup fake_control.launch.py single_motor:=false can_interface:=vcan0
ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

---

## 6. Setup C: Jetson + native can0 (deployment)

**No bridge, no vcan.** The Jetson CAN pins need an external CAN transceiver
(e.g. SN65HVD230) between the Jetson and the bus. Check the pinmux and wiring
for your carrier board.

Bring up can0 once per boot:

```bash
sudo modprobe can && sudo modprobe can_raw && sudo modprobe mttcan
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0   # state should be ERROR-ACTIVE
candump can0                             # should show 0x0901, 0x1B01 (and 0x0902, 0x1B02 with two VESCs)
```

Launch. Start with `monitor_only:=true`, as in §4.

```bash
cd ~/ROS2/vesc_sim && source /opt/ros/lyrical/setup.bash && source install/setup.bash
# one motor
ros2 launch iwalk_bringup vesc_control.launch.py \
  can_interface:=can0 single_motor:=true monitor_only:=false \
  left_vesc_id:=1 left_direction:=1

# two motors (production)
ros2 launch iwalk_bringup vesc_control.launch.py \
  can_interface:=can0 single_motor:=false monitor_only:=false \
  left_vesc_id:=1 right_vesc_id:=2 left_direction:=1 right_direction:=1

ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

If a wheel spins the wrong way, flip `left_direction` / `right_direction` to
`-1`. Don't swap the motor wires to fix it.

---

## 7. Launch arguments (`vesc_control.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `can_interface` | `vcan0` | `vcan0` (laptop) or `can0` (Jetson) |
| `single_motor` | `true` | `true`: left wheel only + ForwardCommandController. `false`: both wheels + DiffDriveController |
| `monitor_only` | `false` | `true`: read feedback, never send motor commands |
| `feedback_timeout` | `0.2` | Seconds before STATUS frames count as stale, which stops the motors |
| `left_vesc_id` / `right_vesc_id` | `1` / `2` | VESC IDs set in VESC Tool |
| `left_direction` / `right_direction` | `1` / `1` | `1` or `-1` |

The motor constants (`motor_pole_pairs`=15, `motor_gear_ratio`=1.0,
`motor_max_erpm`=7500) are xacro defaults in
[iwalk.urdf.xacro](iwalk_motor_control/iwalk_description/urdf/iwalk.urdf.xacro#L20-L22).
They are not launch arguments.

---

## 8. Quick troubleshooting

| Symptom | Likely cause |
|---|---|
| Bridge: `cannot bind vcan0: interface not found` | vcan0 not created (§2) |
| Bridge: `cannot exclusively own /dev/ttyUSB0` | The Python script, or another bridge, has the port open |
| Plugin: `Waveshare bridge heartbeat is missing or stale` | You are on vcan0 and the bridge isn't running, or it crashed |
| Plugin: `joint '…' has no STATUS_1/STATUS_5` | Status messages not enabled in VESC Tool, wrong VESC ID, wrong bitrate, or no termination |
| `candump can0` empty, `ip link` shows BUS-OFF / ERROR-PASSIVE | Wiring, transceiver, termination, or bitrate mismatch |
| Motor stops by itself after 0.5 s | No command reached the VESC (VESC timeout). Check that the controller is active and someone is publishing |
