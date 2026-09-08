import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64
# Later implementation to support ABI / Hall / Absolute encoder

class FakeEncoder(Node):
    def __init__(self):
        super().__init__('fake_encoder')

        self.declare_parameter('counts_per_revolution', 4096)
        self.cpr = self.get_parameter('counts_per_revolution').value
        if not isinstance(self.cpr, int) or self.cpr <= 0:
            raise ValueError('counts_per_revolution must be a positive integer')

        self.count_pub = self.create_publisher(Int64, 'encoder_counts', 10)
        self.create_subscription(
            Float64, 'rotor_angle', self.on_angle, 10)

    def on_angle(self, msg):
        if not math.isfinite(msg.data):
            return

        # Signed cumulative count: no wrapping at one revolution.
        count = math.floor(msg.data * self.cpr / (2.0 * math.pi))
        if not -(2**63) <= count < 2**63:
            self.get_logger().error('Encoder count exceeds Int64 range')
            return
        self.count_pub.publish(Int64(data=count))


def main(args=None):
    rclpy.init(args=args)
    node = FakeEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
