# How wheel position and velocity are reported (hall sensors, VESC, `/joint_states`)

This answers three questions:

1. Hall sensors are absolute, and 15 pole pairs should give 90 positions per turn. Why does `/joint_states` look like it has more resolution than that?
2. Why does the position always start at 0, if the sensors are absolute?
3. How do we calibrate position and velocity?

Short answers:

- **Your 90 is correct.** The position in `/joint_states` really has a 4° step. It looks finer because it is printed as a double such as `0.4886921905584123`, but that number is exactly 7 × 4°.
- **The velocity is not computed from those 90 steps.** It comes from a separate VESC message (ERPM) with a step of about 0.007 rad/s. That is why velocity looks smooth.
- **Hall sensors are only absolute within one electrical revolution,** which is 1/15 of a wheel turn (24°). They cannot tell which of the 15 sectors the rotor is in. On top of that, the driver uses only *changes* in a counter, so the position starts at 0 by design.

Numbers used below: `pole_pairs = 15` and `gear_ratio = 1.0` (from [iwalk.urdf.xacro:20-21](iwalk_motor_control/iwalk_description/urdf/iwalk.urdf.xacro#L20-L21)), and `wheel_radius = 0.10 m` (from [controllers.yaml](iwalk_motor_control/iwalk_bringup/config/controllers.yaml)).

---

## 1. What the hall sensors actually measure

A BLDC motor with hall sensors has 3 digital sensors. Together they give 6 valid states per **electrical** revolution, so each state covers 60° electrical.

```text
one electrical revolution = 1 / pole_pairs of a mechanical turn = 360° / 15 = 24°

hall state:  101 → 100 → 110 → 010 → 011 → 001 → 101 → ...
             |60°e|60°e|60°e|60°e|60°e|60°e|
             | 4° | 4° | 4° | 4° | 4° | 4° |   (mechanical)
```

| Quantity | Value |
|---|---|
| Hall states per electrical revolution | 6 |
| Electrical revolutions per wheel turn | 15 × gear_ratio = 15 |
| **Hall steps per wheel turn** | 6 × 15 = **90** |
| One step, in angle | 2π / 90 = **0.0698 rad = 4.0°** |
| One step at the tyre (r = 0.10 m) | **≈ 7.0 mm** |

The same 6-state pattern repeats 15 times per wheel turn. So the hall reading tells you where you are **within** a 24° sector, but not **which** sector. That is all the VESC needs: to commutate, it only needs the electrical angle. For the wheel, though, the sensors are relative, not absolute. A truly absolute encoder would need a pattern that does not repeat within one mechanical turn.

---

## 2. What the VESC sends over CAN

The VESC does **not** send the raw hall state in its status messages. It sends two derived values, and the driver uses both:

| CAN message | Field | Meaning | Used for |
|---|---|---|---|
| STATUS_1 (ID `0x09xx`) | `erpm` (int32) | Speed estimate in electrical RPM | `velocity` |
| STATUS_5 (ID `0x1Bxx`) | `tachometer` (int32) | Counter: +1 or −1 for every 60° electrical step | `position` |

### 2.1 Position: tachometer counting

The tachometer is a running count of hall steps. It increases going forward and decreases going backward, so it changes by 90 per wheel turn. It is set to 0 when the VESC boots, and it is never an absolute angle.

In VESC FOC firmware, the tachometer is computed from the estimated rotor phase, bucketed into six 60° sectors ("resolution = 60 deg as for BLDC"). At low speed that phase comes from the halls. Above the sensorless transition ERPM it comes from the observer. Either way it is still 6 counts per electrical revolution.

The driver turns counter **deltas** into radians:

```cpp
// vesc_protocol.cpp  tachometer_delta_to_wheel_rad()
rad = direction * delta * 2π / (6 * pole_pairs * gear_ratio)   // = delta * 0.0698
```

See [vesc_protocol.cpp:157-162](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L157-L162). It is accumulated in [vesc_system.cpp:474-497](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L474-L497):

```cpp
if (wheel.needs_tachometer_reference) {
  wheel.previous_tachometer = current;      // first STATUS_5: remember it, add nothing
} else {
  delta = current - previous;               // int32 rollover-safe
  wheel.position += delta * 2π/90;          // accumulate
}
```

So `position` is always an **integer multiple of 0.0698 rad**, plus a constant offset if the plugin was reactivated. You can check this in the terminal:

```bash
ros2 topic echo /joint_states --field position
# then divide by 0.06981317007977318 and you get an integer, e.g.
# 0.4886921905584123 / 0.0698131700797732 = 7.000
```

### 2.2 Velocity: ERPM, which is not differentiated position

`velocity` comes directly from STATUS_1, in [vesc_protocol.cpp:133-138](iwalk_motor_control/iwalk_hardware/src/vesc_protocol.cpp#L133-L138):

```cpp
rad_s = direction * erpm * 2π / (60 * pole_pairs * gear_ratio)   // 1 ERPM = 0.00698 rad/s
```

The VESC computes ERPM internally, from the time between hall edges and/or the FOC observer, at the motor-control loop rate (tens of kHz). It is **not** "count steps in the last 20 ms". So its resolution is 1 ERPM ≈ **0.007 rad/s**, which is 10× finer than one position step per second. That is why velocity looks continuous while position moves in 4° jumps.

**Accuracy is a different matter.** Resolution is how small a change you can see. Accuracy is how close the value is to the truth. At low speed the hall edges are far apart (at 1 rad/s, one edge every 70 ms), so the VESC's speed estimate is coarse and laggy even though it is printed with 0.007 rad/s granularity. Section 5 measures this.

### 2.3 How each update looks

At 50 Hz status messages and 7 rad/s (≈ 1000 ERPM):

- the wheel moves 7 / 50 = 0.14 rad per message, so **2 tachometer counts**. Position jumps 0 → 0.14 → 0.28 → … sometimes 1 or 3 counts, depending on phase.
- velocity reads something like `6.98, 7.01, 6.99 …` (ERPM 1000 ± a few).

### 2.4 The simulator does the same

[vesc_node.py:194-200](iwalk_motor_control/iwalk_fake_vesc/iwalk_fake_vesc/vesc_node.py#L194-L200) models the same behaviour. The fake encoder has 4096 counts per revolution, but the fake VESC quantises it to the real tachometer resolution before sending:

```python
erpm = int(self.velocity * self.poles * 60.0 / (2.0 * math.pi))   # 1-ERPM resolution, truncated
tachometer = (self.last_count * 6 * self.poles) // self.cpr        # 90 counts / rev
```

So in the simulator too, position has a 4° step and velocity has 0.007 rad/s steps. What you see in sim matches what the real VESC will report.

---

## 3. Why position always starts at 0

There are three independent reasons. Any one of them is enough.

1. **The sensors cannot give a mechanical absolute angle** (Section 1). With 15 repeats per turn, the best possible is "somewhere in a 4° slot, in one of 15 sectors".
2. **The VESC does not expose an absolute angle on CAN.** It sends a tachometer that starts at 0 at VESC power-up and then counts. If the VESC reboots, the counter returns to 0. If the wheel turns while the VESC is off, nothing is counted.
3. **The driver deliberately ignores the absolute tachometer value.** The first STATUS_5 after configure/activate only sets the reference ([vesc_system.cpp:475-478](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L475-L478)), and `position` starts at `0.0` when `ros2_control_node` starts. This makes position independent of how long the VESC has been on. It also avoids a jump if the VESC resets mid-run: a large jump is caught by the plausibility check at [vesc_system.cpp:481-493](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L481-L493) and latches a fault.

Two consequences worth knowing:

- Across **deactivate → activate** within the same process, position is kept ([vesc_system.cpp:564](iwalk_motor_control/iwalk_hardware/src/vesc_system.cpp#L564)). However, any wheel motion **while inactive is not counted**, because a new reference is taken on reactivation.
- The very first reading is ambiguous by up to ±1 step (±4°), because you don't know where inside the current hall slot the rotor was.

**For drive wheels this is fine.** The wheel's absolute angle has no meaning. What matters is how far it rolled (odometry) and how fast it turns. `diff_drive_controller` only uses changes in position. If you ever do need an absolute joint angle (for example on a steering joint), you need extra hardware, such as an absolute magnetic encoder on the output shaft or a homing switch/index mark plus a homing routine.

---

## 4. Calibration plan: position

Goal: make sure 1 wheel turn reads exactly 2π rad, with the right sign, and no counts are lost. Then calibrate the distance per radian (effective wheel radius).

Use the single-motor setup from [SETUPS.md §4](SETUPS.md). Start with `monitor_only:=true` for P1–P3 so the motor never drives.

### P0. VESC-side prerequisites (VESC Tool)

- [ ] Run **FOC → Hall sensor detection** with the wheel free, and write the hall table. A bad table gives missing or double tachometer steps and rough low-speed motion.
- [ ] Note the configured **motor pole count** in VESC Tool. VESC Tool uses *poles* (30); the driver uses *pole pairs* (15). Only ERPM ↔ RPM conversion depends on it. The tachometer itself is independent of it.
- [ ] **App Settings → General → CAN Status Message Rate**: STATUS_1 and STATUS_5 enabled, 50 Hz or more. The driver goes stale after `feedback_timeout`.

### P1. Counts per wheel turn (verifies `pole_pairs × gear_ratio`)

1. Put a tape mark on the tyre and a fixed mark on the frame. Lift the wheel.
2. Record the starting `position`, or the raw tachometer: `candump can0,00001B01:1FFFFFFF` (or `vcan0` through the bridge). The first 4 bytes are the big-endian int32 tachometer.
3. Turn the wheel **exactly N = 10 turns** forward by hand, slowly.
4. Expected Δtachometer = 6 × 15 × gear_ratio × N = **900**, which means Δposition = 10 × 2π = **62.83 rad**.
5. Repeat backwards and check that you get back to the same count. If not, steps are being lost: look at the hall table and wiring.

| Result | Meaning | Fix |
|---|---|---|
| 900 ± 1 | Correct | nothing |
| Consistently other value, e.g. 840 or 1200 | `pole_pairs` or `gear_ratio` wrong | set `motor_pole_pairs` / `motor_gear_ratio` so that 6 × pp × gear = Δcounts / N |
| Different forward vs. back, or drifts per run | Lost/extra hall edges | redo hall detection, check hall wiring/shielding/pull-ups |

> If turning by hand does not move the tachometer, the VESC isn't tracking with the motor released. In that case do P1 under power at a slow commanded speed instead, counting turns of the mark.

### P2. Direction sign

1. Turning the wheel in the robot's **forward** direction must give increasing `position`.
2. If not, set `left_direction:=-1` (or `right_direction`). On a differential drive the two sides are usually mirrored, so one of them is normally `-1`.
3. `direction` applies to both position and velocity, and to the commands sent out, so one setting fixes all three.

### P3. Rollover and discontinuity

- Already covered by unit tests (`signed_tachometer_delta`). On hardware, just confirm there is no `tachometer discontinuity` error after a long run, and none after turning the wheel quickly by hand.

### P4. Distance per radian (effective wheel radius) — on the ground

This calibrates `wheel_radius` in the controller config, not the encoder, but it is what makes position useful for odometry.

1. Mark a straight 5–10 m line on the floor.
2. Drive straight slowly with `cmd_vel` and record Δposition for both wheels.
3. `wheel_radius_eff = distance / mean(Δposition_left, Δposition_right)`.
4. Repeat 3× each direction and average. Expect slightly **less** than the nominal 0.10 m on soft tyres.
5. Then calibrate `wheel_separation`: spin in place by N full turns (measured on the floor), and set `wheel_separation_eff = 2 × r × (Δθ_right − Δθ_left) / (2π N)` (signs as reported).

### P5. (Optional) Odometry quality note

[controllers.yaml](iwalk_motor_control/iwalk_bringup/config/controllers.yaml) uses `position_feedback: true`, so the odometry twist is computed from position *differences* at the controller rate. With 4° steps that twist will be jumpy, especially at low speed. Pose is still correct, because counts are exact. If the twist noise is a problem, compare against `position_feedback: false`, which uses ERPM velocity, after V2 below.

---

## 5. Calibration plan: velocity

Goal: know how reported velocity (from ERPM) relates to true wheel speed, over the speed range you use. Also learn the minimum speed where it can be trusted and how much it lags.

The **reference** is the tachometer slope over a long window. Counts are exact, so over 10 s at 7 rad/s that is ~1000 counts, giving 0.1 % accuracy. An optical tachometer, or a phone slow-motion video of the tape mark, is an independent check.

### V0. Setup

- Wheel lifted (V1–V3), then on the ground (V4).
- `monitor_only:=false`, single motor. Remember the VESC **Minimum ERPM** (default 900 ≈ 6.3 rad/s), see [SETUPS.md §4](SETUPS.md). Lower it in VESC Tool if you need to test slower speeds.
- Record: `ros2 bag record /joint_states /test_velocity_controller/commands`.

### V1. Static zero

- No command, wheel still: velocity must be exactly 0.0, position constant. Record 30 s.
- Nudge the wheel by hand: the velocity sign must agree with the position change.

### V2. Steady-state scale (main calibration)

For each commanded speed `ω_cmd` ∈ {±7, ±10, ±15, ±25, ±40, ±50} rad/s, keeping within the max 52.36 rad/s = 7500 ERPM:

1. Command it, wait 3 s to settle, then record 10 s.
2. Compute:
   - `ω_reported` = mean of `velocity` over the window, plus its std-dev (noise).
   - `ω_true` = (position_end − position_start) / (t_end − t_start), from the tachometer.
   - Optional: `ω_optical` from an external tachometer.
3. Fit `ω_reported = k · ω_true + b` across all points.

| Result | Meaning | Fix |
|---|---|---|
| k ≈ 1.00, b ≈ 0 | ERPM and tachometer use consistent pole/gear numbers | nothing |
| k ≠ 1 but P1 passed | VESC Tool motor pole count ≠ 2 × `pole_pairs` | fix VESC Tool **motor poles**, re-run |
| `ω_true` ≠ `ω_cmd` | VESC speed loop not tracking (PID, load, min ERPM, current limits) | tune VESC speed PID; not an encoder issue |
| b ≠ 0 / asymmetric ± | hall timing offset/asymmetry | redo hall detection; check `foc_hall_interp_erpm` / sensorless transition settings |

### V3. Low-speed floor, noise and lag

1. Ramp slowly from 0 to 10 rad/s (if Minimum ERPM was lowered). Find the speed below which `velocity` gets noisy, stair-steps, or reads 0 while the wheel still turns. Below that speed, don't trust `velocity` for control or odometry.
2. Step response: command 0 → 15 rad/s. Compare when `velocity` reaches 63 % with when the tachometer slope does. The difference is the estimator + CAN lag. Also note the total rise time (the VESC speed-loop tuning).
3. Note the velocity std-dev at each V2 speed. This is the number to use if you ever feed wheel velocity into an EKF (`robot_localization`).

### V4. Under load

Repeat two or three V2 points on the ground. Reported vs. tachometer-slope should still match, since both measure the motor. If the **ground distance** disagrees, the cause is wheel slip or radius (P4), not the encoder.

### Analysis helper

A short script for V2 (run it against an exported bag CSV, or adapt it to subscribe live):

```python
import numpy as np
# t [s], pos [rad], vel [rad/s] for one 10 s steady-state window
w_true = np.polyfit(t, pos, 1)[0]          # tachometer slope (least squares, robust to 4° steps)
w_rep, w_std = vel.mean(), vel.std()
print(f"true={w_true:.3f}  reported={w_rep:.3f} ±{w_std:.3f}  ratio={w_rep / w_true:.4f}")
```

---

## 6. Expected values summary

| Item | Expected |
|---|---|
| Position step | 0.0698 rad (4.0°), ≈ 7 mm at the tyre |
| Counts per wheel turn | 90 (× gear_ratio) |
| Velocity step | 0.00698 rad/s (1 ERPM) |
| Max wheel speed | 52.36 rad/s (7500 ERPM) |
| Min commandable speed (default VESC) | ≈ 6.3 rad/s (900 ERPM) |
| Position at startup | 0.0, by design (relative) |
| Position after reactivate | kept, but motion while inactive is lost |
