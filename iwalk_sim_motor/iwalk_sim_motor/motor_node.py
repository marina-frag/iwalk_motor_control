import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimMotor(Node):
    def __init__(self):
        super().__init__('sim_motor')

        self.declare_parameter('inertia', 0.01)
        self.declare_parameter('damping', 0.02)
        self.declare_parameter('max_torque', 1.0)
        self.declare_parameter('update_rate', 200.0)
        self.declare_parameter('command_timeout', 0.5)

        self.inertia = float(self.get_parameter('inertia').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_torque = float(self.get_parameter('max_torque').value)
        rate = float(self.get_parameter('update_rate').value)
        self.timeout = float(self.get_parameter('command_timeout').value)

        values = [self.inertia, self.damping, self.max_torque, rate, self.timeout]
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError('Motor parameters must be finite and positive')

        self.angle = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.last_command = None
        self.last_update = time.monotonic()

        self.create_subscription(
            Float64, 'torque_command', self.on_command, 10)
        self.angle_pub = self.create_publisher(Float64, 'rotor_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, 'rotor_velocity', 10)
        self.create_timer(1.0 / rate, self.update)

    def advance(self, now):
        # Split at command expiry so delayed callbacks preserve the timeout.
        end = self.last_update
        if self.last_command is not None:
            end = min(now, max(self.last_update,
                              self.last_command + self.timeout))
        self.integrate(end - self.last_update, self.torque)
        self.integrate(now - end, 0.0)
        self.last_update = now

    def integrate(self, dt, torque):
        if dt <= 0.0:
            return
        # Exact solution for constant torque over this interval.
        decay = -math.expm1(-self.damping * dt / self.inertia)
        steady_velocity = torque / self.damping
        previous_velocity = self.velocity
        self.angle += (
            steady_velocity * dt
            + (previous_velocity - steady_velocity)
            * self.inertia / self.damping * decay
        )
        self.velocity += (steady_velocity - previous_velocity) * decay

    def on_command(self, msg):
        if not math.isfinite(msg.data):
            self.get_logger().warning('Ignoring non-finite torque')
            return
        now = time.monotonic()
        self.advance(now)
        self.torque = max(-self.max_torque, min(self.max_torque, msg.data))
        self.last_command = now

    def update(self):
        self.advance(time.monotonic())
        self.angle_pub.publish(Float64(data=self.angle))
        self.velocity_pub.publish(Float64(data=self.velocity))


def main(args=None):
    rclpy.init(args=args)
    node = SimMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
