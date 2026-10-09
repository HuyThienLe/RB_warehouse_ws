#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from wh_description.mecanum_kinematics import inverse

class MecanumWheelStateNode(Node):
    def __init__(self):
        super().__init__("mecanum_wheel_state_node")
        
        # Declare parameters
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("half_track_width", 0.19)
        self.declare_parameter("half_wheel_base", 0.15)
        self.declare_parameter("publish_rate", 50.0)
        
        self.r = self.get_parameter("wheel_radius").value
        lx = self.get_parameter("half_wheel_base").value
        ly = self.get_parameter("half_track_width").value
        self.L = lx + ly
        
        # Current wheel positions and velocities
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        self.joint_names = [
            "wheel_fl_joint",
            "wheel_fr_joint",
            "wheel_rl_joint",
            "wheel_rr_joint"
        ]
        
        self.last_cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.timeout_sec = 0.5
        
        # ROS Communications
        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.pub_joints = self.create_publisher(JointState, "/joint_states", 10)
        
        # Timer loop for integration and publishing
        rate = self.get_parameter("publish_rate").value
        self.timer_period = 1.0 / rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("MecanumWheelStateNode initialized successfully.")

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        if dt <= 0.0 or dt > 1.0:
            dt = self.timer_period

        # Timeout: if no cmd_vel received recently, stop wheels
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout_sec:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx = self.last_cmd_vel.linear.x
            vy = self.last_cmd_vel.linear.y
            wz = self.last_cmd_vel.angular.z

        # Calculate wheel velocities (FL, FR, RL, RR)
        w = inverse(vx, vy, wz, self.r, self.L)
        self.wheel_velocities = list(w)

        # Integrate wheel angles
        for i in range(4):
            self.wheel_positions[i] += self.wheel_velocities[i] * dt
            # Normalize angle to [-pi, pi] to avoid numeric overflow
            self.wheel_positions[i] = (self.wheel_positions[i] + 3.14159265359) % (2.0 * 3.14159265359) - 3.14159265359

        # Publish JointState
        js_msg = JointState()
        js_msg.header.stamp = now.to_msg()
        js_msg.name = self.joint_names
        js_msg.position = self.wheel_positions
        js_msg.velocity = self.wheel_velocities
        
        self.pub_joints.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumWheelStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
