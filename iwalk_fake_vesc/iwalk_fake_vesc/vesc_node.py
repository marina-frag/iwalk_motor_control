import math
import struct
import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64


SET_RPM = 3
STATUS = 9
STATUS_5 = 27


class FakeVesc(Node):
    def __init__(self):
        super().__init__('fake_vesc')

        defaults = {
            'can_interface': 'vcan0',
            'vesc_id': 1,
            'pole_pairs': 7,
            'counts_per_revolution': 4096,
            'kp': 0.05,
            'ki': 0.10,
            'max_torque': 1.0,
            'max_erpm': 10000.0,
            'command_timeout': 0.5,
            'encoder_timeout': 0.2,
            'velocity_filter_tau': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.vesc_id = param('vesc_id')
        self.poles = param('pole_pairs')
        self.cpr = param('counts_per_revolution')
        if not 0 <= self.vesc_id <= 254:
            raise ValueError('vesc_id must be between 0 and 254')
        if self.poles <= 0 or self.cpr <= 0:
            raise ValueError('pole_pairs and counts_per_revolution must be positive')

        self.kp = float(param('kp'))
        self.ki = float(param('ki'))
        self.limit = float(param('max_torque'))
        self.max_erpm = float(param('max_erpm'))
        self.cmd_timeout = float(param('command_timeout'))
        self.enc_timeout = float(param('encoder_timeout'))
        self.filter_tau = float(param('velocity_filter_tau'))

        positive = [
            self.limit, self.max_erpm, self.cmd_timeout,
            self.enc_timeout, self.filter_tau,
        ]
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError('Limits and time parameters must be finite and positive')
        if not all(math.isfinite(x) and x >= 0 for x in [self.kp, self.ki]):
            raise ValueError('PI gains must be finite and nonnegative')

        interface = param('can_interface')
        # This development node is deliberately restricted to virtual CAN.
        if not interface.startswith('vcan'):
            raise ValueError('Use a vcan interface for this fake VESC')

        self.bus = can.Bus(
            interface='socketcan',
            channel=interface,
            receive_own_messages=False,
        )
        self.bus.set_filters([{
            'can_id': (SET_RPM << 8) | self.vesc_id,
            'can_mask': 0x1FFFFFFF,
            'extended': True,
        }])

        self.target = 0.0
        self.velocity = 0.0
        self.integral = 0.0
        self.last_command = None
        self.last_encoder = None
        self.last_count = None
        self.velocity_ready = False
        self.last_tick = time.monotonic()

        self.torque_pub = self.create_publisher(
            Float64, 'torque_command', 10)
        self.speed_pub = self.create_publisher(
            Float64, 'vesc_estimated_velocity', 10)
        self.target_pub = self.create_publisher(
            Float64, 'vesc_target_velocity', 10)

        self.create_subscription(
            Int64, 'encoder_counts', self.on_encoder, 10)
        self.create_timer(0.01, self.update)
        self.create_timer(0.02, self.publish_can_status)
        self.get_logger().info(
            f'Fake VESC ID {self.vesc_id} listening on {interface}; '
            f'namespace={self.get_namespace()}'
        )

    def on_encoder(self, msg):
        now = time.monotonic()
        if self.last_encoder is not None:
            dt = now - self.last_encoder
            if 0.0 < dt < self.enc_timeout:
                delta = msg.data - self.last_count
                raw_velocity = delta * 2.0 * math.pi / self.cpr / dt
                alpha = dt / (self.filter_tau + dt)
                self.velocity += alpha * (raw_velocity - self.velocity)
                self.velocity_ready = True
            else:
                self.velocity = 0.0
                self.velocity_ready = False

        self.last_count = msg.data
        self.last_encoder = now

    def update(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        # Bound the work per callback; never block waiting for a CAN frame.
        for _ in range(100):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            if (
                not msg.is_extended_id
                or msg.is_remote_frame
                or msg.is_error_frame
                or msg.is_fd
                or msg.arbitration_id != ((SET_RPM << 8) | self.vesc_id)
                or msg.dlc != 4
                or len(msg.data) != 4
            ):
                continue

            erpm = struct.unpack('>i', msg.data)[0]
            erpm = max(-self.max_erpm, min(self.max_erpm, erpm))
            self.target = erpm * 2.0 * math.pi / (60.0 * self.poles)
            self.last_command = now

        fresh = (
            self.last_command is not None
            and now - self.last_command < self.cmd_timeout
            and self.last_encoder is not None
            and now - self.last_encoder < self.enc_timeout
            and self.velocity_ready
            and 0.0 < dt < self.enc_timeout
        )

        torque = 0.0
        if fresh:
            error = self.target - self.velocity
            proposed_i = self.integral + self.ki * error * dt
            proposed_output = self.kp * error + proposed_i

            # Conditional integration to avoid wind-up at torque saturation.
            if (
                abs(proposed_output) <= self.limit
                or (proposed_output > self.limit and error < 0)
                or (proposed_output < -self.limit and error > 0)
            ):
                self.integral = proposed_i

            torque = max(
                -self.limit,
                min(self.limit, self.kp * error + self.integral),
            )
        else:
            self.integral = 0.0

        self.torque_pub.publish(Float64(data=torque))
        self.speed_pub.publish(Float64(data=self.velocity))
        self.target_pub.publish(Float64(data=self.target))
    def publish_can_status(self):
        now = time.monotonic()

        # Do not advertise stale encoder data as fresh CAN feedback.
        if (
            self.last_encoder is None
            or self.last_count is None
            or not self.velocity_ready
            or now - self.last_encoder >= self.enc_timeout
        ):
            return

        # Mechanical rotor rad/s -> electrical RPM.
        erpm = int(self.velocity * self.poles * 60.0 / (2.0 * math.pi))
        erpm = max(-(2**31), min(2**31 - 1, erpm))

        # Model VESC tachometer resolution:
        # 6 steps per electrical revolution = 6 * pole_pairs per rotor turn.
        tachometer = (self.last_count * 6 * self.poles) // self.cpr

        # Encode the cumulative counter as signed int32 with wraparound.
        tachometer = ((tachometer + 2**31) % 2**32) - 2**31

        # Current and duty are not simulated yet: zero placeholders.
        status_payload = struct.pack('>ihh', erpm, 0, 0)

        # Simulated supply: 24.0 V, encoded in tenths of a volt.
        # Final int16 is reserved.
        status5_payload = struct.pack('>ihh', tachometer, 240, 0)

        try:
            for packet_type, payload in (
                (STATUS, status_payload),
                (STATUS_5, status5_payload),
            ):
                self.bus.send(
                    can.Message(
                        arbitration_id=(packet_type << 8) | self.vesc_id,
                        is_extended_id=True,
                        data=payload,
                    ),
                    timeout=0.0,
                )
        except can.CanError as exc:
            self.get_logger().warning(
                f'CAN status transmission failed: {exc}',
                throttle_duration_sec=2.0,
            )
    def close(self):
        self.bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeVesc()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
