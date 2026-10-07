# Real VESC control

## Architecture and active Xacro

~~~text
Laptop: ROS controller -> VescSystem C++ -> vcan0
        -> vesc_waveshare_bridge C++ -> /dev/ttyUSB0 -> USB-CAN-A -> VESC
Jetson: ROS controller -> VescSystem C++ -> can0 -> VESC
~~~

Το vcan0 δεν προσομοιώνει motor. Η bridge μεταφέρει classic extended CAN
frames αμφίδρομα και δεν κάνει ERPM/tachometer/radian conversion.

Τα fake_control.launch.py και vesc_control.launch.py επεκτείνουν απευθείας
το iwalk_description/urdf/iwalk.urdf.xacro. Το iwalk_control.urdf.xacro είναι
μόνο compatibility wrapper, χωρίς δεύτερο ros2_control block.

Το παραγόμενο URDF περνά στο hardware_info:

- hardware: can_interface, feedback_timeout, monitor_only
- joint: vesc_id, pole_pairs, gear_ratio, direction, max_erpm,
  encoder_sensor, feedback_source

Υποστηρίζονται μόνο encoder_sensor=hall και feedback_source=vesc_status. Οι
Hall sensors ανήκουν στο VESC· το plugin δεν διαβάζει GPIO. Το Xacro δεν
αλλάζει VESC sensor mode, firmware, detection ή calibration. Με
single_motor=true δηλώνεται μόνο ID 1 και επιλέγεται controllers_single.yaml.
Με false δηλώνονται IDs 1/2 και επιλέγεται controllers_diff.yaml.

Το Xacro έχει 15 pole pairs, ratio 1 και max_erpm 7500. Το URDF wheel limit
παράγεται από max_erpm × 2π / (60 × pole_pairs × gear_ratio), περίπου
52.3599 rad/s. Τα χαμηλότερα controller motion limits παραμένουν.

## Protocol, tachometer and conversions

Extended ID = (packet_id << 8) | vesc_id.

- SET_RPM (3): signed int32 big-endian ERPM.
- STATUS_1 (9): signed int32 ERPM, signed int16 current ×10, signed int16
  duty ×1000.
- STATUS_5 (27): signed int32 tachometer, signed int16 voltage ×10 και δύο
  reserved bytes.

Το firmware στέλνει mc_interface_get_tachometer_value(false). Η επίσημη
περιγραφή ορίζει scale 6 EREV: cumulative signed electrical-step counter,
6 counts ανά electrical revolution, όχι raw Hall pulses.

~~~text
ERPM = direction * wheel_rad_s * gear_ratio * pole_pairs * 60 / (2π)
wheel_rad_s = direction * ERPM * 2π / (60 * pole_pairs * gear_ratio)
Δwheel_position = direction * Δtachometer * 2π
                  / (6 * pole_pairs * gear_ratio)
~~~

Τα tests επιβεβαιώνουν 1500 ERPM = 10.4719755 rad/s και 90 counts = 2π rad
για 15 pole pairs, ratio 1. Διατηρούνται signed feedback, negative rotation,
direction=-1 και int32 rollover.

Το πρώτο tachometer sample θέτει reference χωρίς jump. Σε reactivation
διατηρείται η σωρευμένη ROS position και τίθεται νέο counter reference, άρα
δεν ανακατασκευάζεται κίνηση όσο ήταν inactive. Ένα μη φυσικά εφικτό delta
γίνεται latched error και νέα αναφορά. Μικρό reset μπορεί να μοιάζει με
πραγματική κίνηση και δεν διακρίνεται αξιόπιστα μόνο από τον counter.

Η αποστολή κάνει clamp στο max_erpm και deterministic nearest/ties-to-even
rounding, αντίστοιχο του Python int(round(...)), πριν το int32 conversion.

Πηγές:

- [VESC CAN protocol, status rates και timeout](https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md)
- [STATUS_5 στο firmware](https://github.com/vedderb/bldc/blob/master/comm/comm_can.c#L1170-L1177)
- [Επίσημο tachometer distance scale](https://github.com/vedderb/bldc/blob/master/motor/mc_interface.c#L1540-L1576)
- [python-can 4.6.1 Seeed backend](https://github.com/hardbyte/python-can/blob/v4.6.1/can/interfaces/seeedstudio/seeedstudio.py)

Η ακριβής firmware έκδοση των φυσικών VESC δεν υπάρχει στο repository και
πρέπει να καταγραφεί πριν χαρακτηριστεί η λύση hardware-tested.

## Bridge and fail-safe behavior

Η bridge παίρνει exclusive file lock και χρησιμοποιεί το python-can framing:
serial 2,000,000, CAN 500,000, EXT, normal mode, disabled filter/mask. Κρατά
partial reads, ολοκληρώνει partial writes, έχει bounded parser/queues και
resynchronization. Serial stall πάνω από 100 ms ή disconnect τερματίζει τη
bridge χωρίς reconnect ή command replay.

CAN_RAW_RECV_OWN_MSGS=0 αποτρέπει loop. Το internal vcan-only heartbeat
0x1FFFFFFE#495742520101ssss δεν προωθείται στο adapter. Σε vcan* το plugin
απαιτεί fresh heartbeat. Write σε vcan/serial δεν αποδεικνύει physical ACK·
η freshness STATUS_1 και STATUS_5 είναι η end-to-end ένδειξη.

monitor_only=true δεν στέλνει motor command σε κανένα lifecycle path.
Στο normal mode το stop στέλνει τρεις φορές SET_CURRENT 0: zero current/coast
με τη συνήθη ρύθμιση, όχι εγγυημένο braking. Κράτησε ενεργό το VESC
communication timeout (συνήθως 0.5 s) και επίλεξε συνειδητά timeout brake
current. Η επίσημη οδηγία προτείνει command περίπου 50 Hz· εδώ το loop είναι
100 Hz.

STATUS_1 και STATUS_5 μπορούν να έχουν διαφορετικά VESC Tool rate groups,
π.χ. 50 Hz και 20 Hz. Και οι δύο περίοδοι πρέπει να έχουν περιθώριο κάτω από
feedback_timeout=0.2 s· μην αυξάνεις το timeout για να κρύψεις frame loss.

## Build and vcan

~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/iwalk_motor_ws
colcon build --symlink-install --packages-up-to iwalk_bringup
source install/setup.bash
~~~

~~~bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set dev vcan0 up
ip -details link show vcan0
~~~

## Laptop with real Waveshare/VESC

Μόνο ένας process επιτρέπεται να κατέχει το serial port:

~~~bash
ros2 run iwalk_hardware vesc_waveshare_bridge -- \
  --can-interface vcan0 --serial-port /dev/ttyUSB0 \
  --serial-baudrate 2000000 --can-bitrate 500000
~~~

Single motor, ID 1:

~~~bash
ros2 launch iwalk_bringup vesc_control.launch.py \
  single_motor:=true monitor_only:=false can_interface:=vcan0 \
  left_vesc_id:=1 left_direction:=1

ros2 topic pub --rate 20 /test_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
~~~

Production, IDs 1/2:

~~~bash
ros2 launch iwalk_bringup vesc_control.launch.py \
  single_motor:=false monitor_only:=false can_interface:=vcan0 \
  left_vesc_id:=1 right_vesc_id:=2 \
  left_direction:=1 right_direction:=1

ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
~~~

Οι αρχικές commands είναι σκόπιμα μηδενικές. Μη μηδενικό wheel rad/s δίνεται
μόνο με σηκωμένους τροχούς, emergency stop, επιβεβαιωμένες directions και
σωστά VESC limits. Τερμάτισε πρώτα controller/zero command και μετά bridge.

## Jetson with native can0

Δεν εκτελείται bridge:

~~~bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
~~~

~~~bash
# One motor
ros2 launch iwalk_bringup vesc_control.launch.py \
  single_motor:=true monitor_only:=false can_interface:=can0 \
  left_vesc_id:=1 left_direction:=1

# Two motors
ros2 launch iwalk_bringup vesc_control.launch.py \
  single_motor:=false monitor_only:=false can_interface:=can0 \
  left_vesc_id:=1 right_vesc_id:=2 \
  left_direction:=1 right_direction:=1
~~~

## Tests and remaining physical checks

~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/iwalk_motor_ws
colcon test --packages-select iwalk_hardware --event-handlers console_direct+
colcon test-result --verbose
~~~

Καλύπτονται protocol, conversions, negative/direction/rollover, serial
fragmentation/resync/bounds, configuration errors, stable exported values,
one/two fake VESC freshness, monitor-only suppression και own-frame loop
avoidance. Τα tests δεν κινούν physical motor.

Παραμένουν: firmware version, VESC IDs/bitrates/status rates, directions,
timeout behavior, adapter disconnect, CAN termination/wiring και safe
low-speed rotation εκτός εδάφους. Τα battery specs δεν γίνονται motor-current
limits και το commercial optical encoder 1024 ppr δεν χρησιμοποιείται.
