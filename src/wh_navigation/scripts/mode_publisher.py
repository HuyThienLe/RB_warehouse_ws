#!/usr/bin/env python3
"""mode_publisher.py – Phát chế độ vận hành kho hàng (navigation | mapping) lên topic /wh/mode.
Sử dụng QoS TRANSIENT_LOCAL + RELIABLE để các client kết nối muộn (như WebBridge) nhận được ngay lập tức.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class ModePublisher(Node):
    def __init__(self):
        super().__init__("mode_publisher_node")
        self.declare_parameter("mode", "navigation")
        self.mode = str(self.get_parameter("mode").value).strip().lower()

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_mode = self.create_publisher(String, "/wh/mode", qos)

        # Phát thông điệp định kỳ 1.0s
        self.timer = self.create_timer(1.0, self.publish_mode)
        self.publish_mode()
        self.get_logger().info(f"ModePublisher khởi động thành công. Chế độ hiện tại: '{self.mode.upper()}'")

    def publish_mode(self):
        msg = String(data=self.mode)
        self.pub_mode.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
