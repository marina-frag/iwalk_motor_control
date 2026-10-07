import struct

import can
import rclpy
from rclpy.node import Node


BRIDGE_HEARTBEAT_ID = 0x1FFFFFFE


class FakeBridgeHeartbeat(Node):
    """Test-only heartbeat; this does not emulate or replace the real bridge."""

    def __init__(self):
        super().__init__('fake_bridge_heartbeat')
        self.declare_parameter('can_interface', 'vcan0')
        interface = self.get_parameter('can_interface').value
        if not interface.startswith('vcan'):
            raise ValueError('fake bridge heartbeat is restricted to vcan')
        self.bus = can.Bus(
            interface='socketcan',
            channel=interface,
            receive_own_messages=False,
        )
        self.sequence = 0
        self.create_timer(0.05, self.publish_heartbeat)

    def publish_heartbeat(self):
        payload = b'IWBR' + bytes((1, 1)) + struct.pack('>H', self.sequence)
        self.sequence = (self.sequence + 1) & 0xFFFF
        self.bus.send(can.Message(
            arbitration_id=BRIDGE_HEARTBEAT_ID,
            is_extended_id=True,
            data=payload,
        ), timeout=0.0)

    def close(self):
        self.bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FakeBridgeHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
