#!/usr/bin/env python3
"""auto_initial_pose.py – Tự động phát khởi tạo vị trí ban đầu (HOME) cho AMCL."""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__("auto_initial_pose")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.sub_joint = self.create_subscription(JointState, "/joint_states", self.cb_joint_states, 10)
        self.current_joints = {"lift_joint": 0.0, "shoulder_joint": 0.0, "elbow_joint": 0.0, "wrist_joint": 0.0}
        self.has_joint_states = False

        self.declare_parameter("x", -2.4)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)

        # Phát 5 lần trong 3 giây đầu để đảm bảo AMCL và cánh tay nhận được
        self.count = 0
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.get_logger().info("AutoInitialPose node khởi động.")

    def cb_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joints:
                self.current_joints[name] = pos
        self.has_joint_states = True

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

        # Gập tay an toàn (STOW: lift=0.160m, shoulder=-1.5708, elbow=-2.3562, wrist=0.7854)
        # Luôn nâng Z lên 0.160m trước khi xoay để tuyệt đối không quẹt vào thành khay xe
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

        d_curr = self.current_joints["lift_joint"]
        q1_curr = self.current_joints["shoulder_joint"]
        q2_curr = self.current_joints["elbow_joint"]
        q3_curr = self.current_joints["wrist_joint"]

        if d_curr < 0.150:
            p1 = JointTrajectoryPoint()
            p1.positions = [0.160, q1_curr, q2_curr, q3_curr]
            p1.time_from_start.sec = 0
            p1.time_from_start.nanosec = 800000000

            p2 = JointTrajectoryPoint()
            p2.positions = [0.160, -1.5708, -2.3562, 0.7854]
            p2.time_from_start.sec = 2
            p2.time_from_start.nanosec = 300000000
            arm_msg.points = [p1, p2]
        else:
            p = JointTrajectoryPoint()
            p.positions = [0.160, -1.5708, -2.3562, 0.7854]
            p.time_from_start.sec = 1
            p.time_from_start.nanosec = 500000000
            arm_msg.points = [p]

        self.arm_pub.publish(arm_msg)

        self.count += 1
        self.get_logger().info(f"Đã phát initial pose ({self.count}/5): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} + stowed arm (lift=0.160m)")

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
