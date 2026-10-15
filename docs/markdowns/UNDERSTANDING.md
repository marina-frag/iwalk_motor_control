# How the VESC CAN stack works (and what to correct in the mental model)

This explains what the code in `iwalk_motor_control/` actually does, compared
with the plan as you described it. Each point links to where it happens.
Commands for every setup are in [SETUPS.md](SETUPS.md).

---

## 1. The big picture, as implemented

```text
DEVELOPMENT (laptop, 1 real motor)

  ros2_control_node
    └─ VescSystem plugin (C++)  ──SocketCAN──►  vcan0  ◄──SocketCAN──  vesc_waveshare_bridge (C++)
                                                                          │
                                                                  /dev/ttyUSB0 (serial, 2 Mbaud)
                                                                          │
                                                               Waveshare USB-CAN-A ── CAN bus ── VESC 1 ── motor

DEPLOYMENT (Jetson, 2 motors)

  ros2_control_node
    └─ VescSystem plugin (C++)  ──SocketCAN──►  can0 (Jetson CAN controller + transceiver) ── CAN bus ── VESC 1, VESC 2

SIMULATION (laptop, no hardware at all)

  ros2_control_node
    └─ VescSystem plugin (C++)  ──SocketCAN──►  vcan0  ◄──  fake VESC node(s) + fake bridge heartbeat (Python)
```

The **same plugin binary** is used in all three. The only thing that changes is
the `can_interface` launch argument (`vcan0` or `can0`) and `single_motor`
(`true` or `false`).

---

## 2. What you understood correctly ✅

### 2.1 "A program reads the Waveshare traffic and exposes it on vcan0"

Correct. This program is
[waveshare_bridge.cpp](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp),
built as the executable `vesc_waveshare_bridge`. It is a bidirectional relay:

| Direction | What happens | Where |
|---|---|---|
| Adapter → vcan0 | Reads serial bytes, reassembles Waveshare/Seeed frames, writes them to vcan0 as normal `can_frame`s | [`read_serial()`](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L385), parser [`SeeedSerialParser::feed`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L241) |
| vcan0 → Adapter | Reads frames the plugin wrote to vcan0, serializes them to the Waveshare serial format, writes to `/dev/ttyUSB0` | [`read_can()`](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L334), [`serialize_seeed_frame`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L214) |
| Adapter setup | Sends the 20-byte init frame (500 kbit/s, extended IDs, normal mode), byte-for-byte what python-can's `seeedstudio` backend sends | [`make_seeed_init_frame`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L194), called in the [`Bridge` constructor](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L253) |

It does **no** VESC logic: no ERPM, no tachometer, no unit conversion. It only
moves frames. The bridge also sets `CAN_RAW_RECV_OWN_MSGS = 0`
([line 180](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L180)),
so frames it writes to vcan0 are not read back and sent to the adapter again in
a loop.

### 2.2 "The hardware interface only speaks SocketCAN, so it works on vcan0 and can0"

Correct. [vesc_system.cpp](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp)
opens a plain `PF_CAN / SOCK_RAW` socket on whatever interface name it is given
([`open_socket()`](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L267)).
It does not know about the Waveshare adapter or serial ports. The name comes
from the URDF parameter `can_interface`
([iwalk.urdf.xacro:456](iwalk_motor_control/iwalk_description/urdf/iwalk.urdf.xacro#L456)),
which comes from the launch argument
([vesc_control.launch.py:92](iwalk_motor_control/iwalk_bringup/launch/vesc_control.launch.py#L92)).

The plugin sets a kernel receive filter, so it only gets STATUS_1 and STATUS_5
from the declared VESC IDs, plus the bridge heartbeat
([lines 285–300](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L285-L300)).

### 2.3 "On the Jetson we just switch to can0"

Correct. You don't run the bridge on the Jetson. You configure `can0` with
`ip link` and launch with `can_interface:=can0`. No code changes and no rebuild
for that switch.

### 2.4 "It must work with 1 motor now and 2 motors later"

Correct, and it's already built in:

- The plugin accepts **1 or 2** joints
  ([`on_init` check](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L132)).
  It reads each joint's `vesc_id` and rejects duplicate IDs.
- The xacro adds the right wheel only when `single_motor:=false`
  ([iwalk.urdf.xacro:469](iwalk_motor_control/iwalk_description/urdf/iwalk.urdf.xacro#L469)).
- The launch file picks the matching controller config:
  - [`controllers_single.yaml`](iwalk_motor_control/iwalk_bringup/config/controllers_single.yaml)
    uses a `ForwardCommandController` on one wheel, so you send rad/s directly.
  - [`controllers_diff.yaml`](iwalk_motor_control/iwalk_bringup/config/controllers_diff.yaml)
    uses a `DiffDriveController` on both wheels and takes `cmd_vel` input.

### 2.5 "Send commands, receive STATUS_1 (ERPM) and STATUS_5 (tachometer)"

Correct. All of this is in
[vesc_protocol.cpp](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp),
called from the plugin:

| ros2_control step | CAN traffic | Code |
|---|---|---|
| `write()` | Converts wheel rad/s to ERPM, clamps it to `max_erpm`, sends **SET_RPM (3)** | [`write()`](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L596), [`wheel_rad_s_to_erpm`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L140), [`make_vesc_i32_command`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L97) |
| `read()`, velocity | **STATUS_1 (9)** ERPM → wheel rad/s | [`decode_status1`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L107), used at [vesc_system.cpp:417](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L417) |
| `read()`, position | **STATUS_5 (27)** tachometer delta → wheel rad (6 counts per electrical revolution), handles int32 rollover | [`decode_status5`](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L120), [vesc_system.cpp:429–453](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L429-L453) |
| Stop (deactivate, error, stale feedback) | **SET_CURRENT 0**, sent 3 times, so the motor coasts | [`stop_commands`](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L343) |

The CAN IDs, byte order and scaling match your Python script
([`send_cmd`](single_motor_ws/can_test/can_send_test_no_socketcan.py#L60),
[`update_status`](single_motor_ws/can_test/can_send_test_no_socketcan.py#L79)).

---

## 3. What to correct ⚠️

### 3.1 The Python script is not part of the ROS stack

[can_send_test_no_socketcan.py](single_motor_ws/can_test/can_send_test_no_socketcan.py)
is a **standalone test tool**. The ROS system doesn't run it or import it, and
it does not create vcan0. Its *logic* was ported to C++:

- The Waveshare serial framing went into `vesc_protocol.cpp` (Seeed parser,
  serializer, init frame). The bridge uses it.
- The VESC packet encoding and decoding went into `vesc_protocol.cpp`. The
  plugin uses it.

The script is still useful. Because it supports `--interface socketcan`, you
can run it against `vcan0` **while the bridge is running**. That gives you a
second listener on the real traffic (see SETUPS.md §4).

> The script and the bridge can't both open `/dev/ttyUSB0`. The bridge takes an
> exclusive lock on the port
> ([open_serial](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L136)).

### 3.2 Nobody "creates" vcan0 automatically

The bridge **requires** vcan0 to exist already. If it doesn't, the bridge fails
with `interface not found`
([open_can](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L189-L199)).
You create it once per boot with `modprobe vcan` + `ip link add`
(SETUPS.md §2).

### 3.3 "Exactly the same code" is ~99% true. The differences:

**a) There is a deliberate vcan-only code path: the bridge heartbeat.**

If the interface name starts with `vcan`, the plugin also requires a heartbeat
frame `0x1FFFFFFE` ("IWBR") to be fresh:

- the check is switched on at
  [vesc_system.cpp:154](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L154);
- it is evaluated in
  [`feedback_fresh`](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L471).

The bridge sends the heartbeat every 50 ms
([waveshare_bridge.cpp:313](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L313))
and never forwards it to the real bus
([line 352](iwalk_motor_control/iwalk_hardware/src/waveshare_bridge.cpp#L352)).

The reason: on vcan, writes always "succeed" even if the bridge has died, so
the plugin needs proof that something is actually relaying. On `can0` this
check is **off**, so that branch is never exercised in development.

**b) Error behaviour differs.**

- On vcan0, a SET_RPM `write()` always succeeds, even if no VESC is connected.
  The real bus errors (no ACK, bus-off, wiring problems) happen in the adapter.
  The plugin never sees them, except as stale STATUS frames.
- On can0, the Jetson's CAN controller can go error-passive or bus-off, and
  socket writes can fail with `ENOBUFS`. The plugin treats a failed write as an
  error and stops.

  The plugin does not enable `CAN_RAW_ERR_FILTER`, so it never receives CAN
  error frames. It detects bus problems indirectly: STATUS_1/STATUS_5 go stale
  after `feedback_timeout` (0.2 s) and it stops.

  `restart-ms 100` in the can0 setup lets the controller recover automatically
  from bus-off.

  Freshness is measured from each frame's **kernel arrival timestamp**
  (`SO_TIMESTAMPNS`, read in `receive_feedback`), not from when `read()` runs.
  So frames that sat in the socket queue during a stall are not mistaken for
  new feedback. This works the same on vcan0 and can0.

**c) Timing differs.**

- USB serial adds latency and jitter that can0 doesn't have. If it works over
  the bridge, it should work at least as well on can0.
- Bus load is higher with two VESCs. Each VESC sends its own STATUS_1 and
  STATUS_5 at its configured rate, so bus load roughly doubles.

**d) Two-motor behaviour can't be tested on real hardware yet.**

With one motor you test the 1-joint path. The 2-joint path is covered only by
the unit tests with fake VESCs
([test_vesc_system.cpp](iwalk_motor_control/iwalk_hardware/test/test_vesc_system.cpp),
`ActivatesOnlyAfterBothDeclaredMotorsAreFresh`) and by the simulation launch.

### 3.4 The VESCs send status, not commands

The plugin *sends* commands (SET_RPM, SET_CURRENT) and *receives* **status**
(STATUS_1, STATUS_5). The VESCs only send status packets if you enable them in
VESC Tool: App Settings → General → CAN Status Message Rate (STATUS 1 and
STATUS 5). Both must arrive comfortably faster than every 0.2 s, or the plugin
won't activate (it waits 2 s for fresh feedback in
[`on_activate`](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L525)).

### 3.5 There is a third mode you may not have known about: full simulation

[fake_control.launch.py](iwalk_motor_control/iwalk_bringup/launch/fake_control.launch.py)
runs the same plugin on vcan0 against Python **fake VESCs**
([vesc_node.py](iwalk_motor_control/iwalk_fake_vesc/iwalk_fake_vesc/vesc_node.py)).
These answer SET_RPM with simulated STATUS frames, and a **fake heartbeat**
node ([bridge_heartbeat_node.py](iwalk_motor_control/iwalk_fake_vesc/iwalk_fake_vesc/bridge_heartbeat_node.py))
stands in for the bridge. Both fakes refuse to run on anything other than
`vcan*`.

> Don't run the fake launch and the real bridge on the same vcan0 at the same
> time. Fake VESC 1 and real VESC 1 would both answer as ID 1.

### 3.6 Small things in the repo

- [iwalk_control.urdf.xacro](iwalk_motor_control/iwalk_bringup/urdf/iwalk_control.urdf.xacro)
  and [controllers.yaml](iwalk_motor_control/iwalk_bringup/config/controllers.yaml)
  are leftovers that the current launch files don't use. The real
  `<ros2_control>` block is in
  [iwalk.urdf.xacro:452](iwalk_motor_control/iwalk_description/urdf/iwalk.urdf.xacro#L452).
- Motor constants live in xacro args:
  - `motor_pole_pairs`=15
  - `motor_gear_ratio`=1.0
  - `motor_max_erpm`=7500

  ([iwalk.urdf.xacro:20–22](iwalk_motor_control/iwalk_description/urdf/iwalk.urdf.xacro#L20-L22)).
  `vesc_control.launch.py` does **not** expose them as launch arguments. To
  change them, edit the xacro defaults or add them to the `mappings` in the
  launch file.
- `encoder_sensor` must be `hall` and `feedback_source` must be `vesc_status`.
  Anything else is rejected
  ([vesc_system.cpp, on_init](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L124)).

---

## 4. Jazzy → Lyrical: what's different

I checked against the Lyrical packages available on this machine:

- `ros-lyrical-hardware-interface` / `ros-lyrical-controller-manager` 6.10.1
- `ros-lyrical-diff-drive-controller` 6.9.0
- `ament_cmake` 2.8.8

I compiled `iwalk_hardware` against Lyrical. A copy with the CMake fix below
builds, and its tests pass: 19 tests, 0 failures, 4 skipped because vcan0 was
absent.

| Area | Jazzy | Lyrical | Impact on this code |
|---|---|---|---|
| `on_init(const HardwareComponentInterfaceParams&)` | Present (recent Jazzy releases) | Present | ✅ Already uses the new signature. Nothing to do. |
| `export_state_interfaces()` / `export_command_interfaces()` returning `std::vector<StateInterface>`, and `StateInterface(name, iface, double*)` | Deprecated in recent Jazzy | **Deprecated** (`[[deprecated]]` in `hardware_component_interface.hpp` and `handle.hpp`) | ⚠️ Compiles with warnings. The plugin uses them at [vesc_system.cpp:246–265](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L246-L265). The modern way: delete both functions and let the framework export the interfaces from the URDF. Then in `read()`/`write()` use `set_state(name, value)` and `get_command(name)` instead of raw `double` members. Optional for now, but this API is planned for removal. |
| `ament_target_dependencies()` in CMake | Normal | **Not available**: `find_package(ament_cmake)` no longer provides it, so the build fails with `Unknown CMake command "ament_target_dependencies"` | ❌ **Build-breaking.** Used 2× in [CMakeLists.txt](iwalk_motor_control/iwalk_hardware/CMakeLists.txt). Replace with `target_link_libraries(... hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)`. Exact patch in [SETUPS.md §1](SETUPS.md#1-one-time-install-and-build). With that change it builds on Lyrical with no warnings and passes its tests (verified). |
| `robot_description` passed as a **parameter** to `ros2_control_node` | Deprecated, but still works | Controller manager reads the URDF from the `/robot_description` **topic** only | ✅ Still works, because `robot_state_publisher` publishes that topic. The `{'robot_description': robot_xml}` passed to `ros2_control_node` in both launch files is just ignored and can be removed. |
| `diff_drive_controller` `publish_rate` | Exists | **Removed** (not in the 6.9 parameter list) | ⚠️ In [controllers_diff.yaml:26](iwalk_motor_control/iwalk_bringup/config/controllers_diff.yaml#L26) it is ignored. Odometry is published at the controller update rate (100 Hz). All other parameters used are still valid. `cmd_vel` is `TwistStamped`, which the docs already use. |
| `spawner` flags (`-c`, `-p`, `--controller-manager-timeout`) | ✓ | ✓ | ✅ No change. |
| `iwalk_description` installs `meshes/` | — | — | ❌ Not a Lyrical issue. The `.gitignore` excludes `*.STL`, so `meshes/` never arrives with a clone, and `install(DIRECTORY meshes)` fails. See [SETUPS.md §1](SETUPS.md#1-one-time-install-and-build). |
| Commands in docs | `source /opt/ros/jazzy/setup.bash` | `source /opt/ros/lyrical/setup.bash` | ⚠️ README and `real-vesc-control.md` still say `jazzy`. |
| OS / Python | Ubuntu 24.04, Python 3.12 | Ubuntu 26.04, Python 3.14 | ✅ python-can 4.6.1 (apt `python3-can`) works. |
| Linux SocketCAN code | — | — | ✅ Kernel API, not ROS. Identical. |

**Jetson caveat (not a code issue).** Lyrical needs Ubuntu 26.04 on arm64.
Before relying on this, check that your JetPack/L4T image for the Orin Nano
Super actually provides 26.04 together with the `mttcan` CAN driver.
