#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class SlipOdomNode(Node):
    def __init__(self):
        super().__init__("slip_odom_node")
        
        # Declare parameters
        self.declare_parameter("enable_slip", True)
        self.declare_parameter("k_vx", 0.98)
        self.declare_parameter("k_vy", 0.92)
        self.declare_parameter("k_wz", 0.97)
        self.declare_parameter("noise_ratio", 0.02)
        self.declare_parameter("noise_fixed", 0.002)
        
        # Accumulated odometry pose (dead reckoning from degraded velocities)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Random walk biases
        self.bias_vx = 0.0
        self.bias_vy = 0.0
        self.bias_wz = 0.0
        
        self.last_time = None
        self.initialized = False
        
        # Publishers and Subscribers
        self.sub_gt = self.create_subscription(
            Odometry, "/odom_gt", self.odom_gt_callback, 20
        )
        self.pub_odom = self.create_publisher(Odometry, "/odom", 20)
        
        self.get_logger().info("SlipOdomNode started (publishes degraded /odom from /odom_gt).")

    def odom_gt_callback(self, msg: Odometry):
        curr_time = self.get_clock().now()
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if not self.initialized:
            # Initialize pose matching ground truth initial pose
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)
            self.last_time = stamp_sec
            self.initialized = True
            return

        dt = stamp_sec - self.last_time
        self.last_time = stamp_sec
        
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02  # Fallback to nominal 50Hz

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        enable_slip = self.get_parameter("enable_slip").value
        if enable_slip:
            k_vx = self.get_parameter("k_vx").value
            k_vy = self.get_parameter("k_vy").value
            k_wz = self.get_parameter("k_wz").value
            noise_ratio = self.get_parameter("noise_ratio").value
            noise_fixed = self.get_parameter("noise_fixed").value

            # Update slow random walk bias
            self.bias_vx += float(np.random.normal(0.0, 0.0001))
            self.bias_vy += float(np.random.normal(0.0, 0.0001))
            self.bias_wz += float(np.random.normal(0.0, 0.00005))
            self.bias_vx = np.clip(self.bias_vx, -0.01, 0.01)
            self.bias_vy = np.clip(self.bias_vy, -0.01, 0.01)
            self.bias_wz = np.clip(self.bias_wz, -0.01, 0.01)

            # Apply scale factor and noise if moving
            sigma_vx = noise_ratio * abs(vx) + noise_fixed if abs(vx) > 1e-4 else 0.0
            sigma_vy = noise_ratio * abs(vy) + noise_fixed if abs(vy) > 1e-4 else 0.0
            sigma_wz = noise_ratio * abs(wz) + (noise_fixed * 0.5) if abs(wz) > 1e-4 else 0.0

            vx_deg = k_vx * vx + float(np.random.normal(0.0, sigma_vx)) + (self.bias_vx if abs(vx) > 1e-4 else 0.0)
            vy_deg = k_vy * vy + float(np.random.normal(0.0, sigma_vy)) + (self.bias_vy if abs(vy) > 1e-4 else 0.0)
            wz_deg = k_wz * wz + float(np.random.normal(0.0, sigma_wz)) + (self.bias_wz if abs(wz) > 1e-4 else 0.0)
        else:
            vx_deg, vy_deg, wz_deg = vx, vy, wz

        # Dead-reckoning integration in odometry frame
        delta_yaw = wz_deg * dt
        yaw_mid = self.yaw + delta_yaw / 2.0
        
        delta_x = (vx_deg * math.cos(yaw_mid) - vy_deg * math.sin(yaw_mid)) * dt
        delta_y = (vx_deg * math.sin(yaw_mid) + vy_deg * math.cos(yaw_mid)) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Build degraded Odometry message
        out_msg = Odometry()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = "odom"
        out_msg.child_frame_id = "base_footprint"

        out_msg.pose.pose.position.x = float(self.x)
        out_msg.pose.pose.position.y = float(self.y)
        out_msg.pose.pose.position.z = 0.0
        out_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        # Pose covariance: larger variance in y due to Mecanum lateral drift
        pose_cov = np.zeros(36, dtype=np.float64)
        pose_cov[0] = 0.05    # var(x)
        pose_cov[7] = 0.12    # var(y) (vy trượt nhiều hơn)
        pose_cov[14] = 1e-6   # var(z)
        pose_cov[21] = 1e-6   # var(roll)
        pose_cov[28] = 1e-6   # var(pitch)
        pose_cov[35] = 0.05   # var(yaw)
        out_msg.pose.covariance = pose_cov.tolist()

        out_msg.twist.twist.linear.x = float(vx_deg)
        out_msg.twist.twist.linear.y = float(vy_deg)
        out_msg.twist.twist.linear.z = 0.0
        out_msg.twist.twist.angular.x = 0.0
        out_msg.twist.twist.angular.y = 0.0
        out_msg.twist.twist.angular.z = float(wz_deg)

        # Twist covariance: vy variance significantly larger than vx
        twist_cov = np.zeros(36, dtype=np.float64)
        twist_cov[0] = 0.01   # var(vx)
        twist_cov[7] = 0.04   # var(vy)
        twist_cov[14] = 1e-6
        twist_cov[21] = 1e-6
        twist_cov[28] = 1e-6
        twist_cov[35] = 0.01  # var(wz)
        out_msg.twist.covariance = twist_cov.tolist()

        self.pub_odom.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlipOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
