#!/usr/bin/env python3
"""auto_initial_pose.py – Tự động phát khởi tạo vị trí ban đầu (HOME) cho AMCL."""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__("auto_initial_pose")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.declare_parameter("x", -2.4)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)

        # Phát 5 lần trong 3 giây đầu để đảm bảo AMCL và cánh tay nhận được
        self.count = 0
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.get_logger().info("AutoInitialPose node khởi động.")

    def publish_pose(self):
        x = self.get_parameter("x").value
        y = self.get_parameter("y").value
        yaw = self.get_parameter("yaw").value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Chuyển yaw thành Quaternion
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Độ bất định nhỏ ban đầu (cov = 0.05m, 0.05 rad)
        cov = [0.0] * 36
        cov[0] = 0.05 * 0.05    # var(x)
        cov[7] = 0.05 * 0.05    # var(y)
        cov[35] = 0.05 * 0.05   # var(yaw)
        msg.pose.covariance = cov

        self.pub.publish(msg)

        # Gập tay gọn gàng (Tucked / Stowed posture) để không che khuất LiDAR và đảm bảo an toàn khi di chuyển
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [0.050, -1.5708, -2.3562]
        pt.time_from_start.sec = 1
        arm_msg.points = [pt]
        self.arm_pub.publish(arm_msg)

        self.count += 1
        self.get_logger().info(f"Đã phát initial pose ({self.count}/5): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} + stowed arm")

        if self.count >= 5:
            self.timer.cancel()
            self.get_logger().info("Hoàn tất phát initial pose và stowed arm.")


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
