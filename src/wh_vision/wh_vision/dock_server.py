#!/usr/bin/env python3
"""dock_server.py – Action Server DockToSlot (Visual Servoing PBVS cho robot Mecanum).

Nhiệm vụ:
- Nhận diện ô kệ qua ArUco tag ID trên /vision/tags
- Tính sai lệch trong khung base_link:
    e_dist = px - target_standoff
    e_lat  = py - 0
    e_yaw  = wrap(atan2(n_y, n_x) - pi)
- Điều khiển PBVS 3 bậc tự do (vx, vy, wz) ra topic /cmd_vel_servo
- Ưu tiên khử lệch góc và ngang trước, tiến/lùi sau
- Khóa an toàn khoảng cách: px < 0.28m dừng ngay lập tức
- Điều kiện hoàn tất: sai số <= 5 mm, <= 2 độ liên tục 10 frame
- Hàm verify_slot(expected_id) chống nhầm lẫn ô kệ
"""

import math
import os
import time
import numpy as np
import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wh_interfaces.action import DockToSlot
from wh_interfaces.msg import TagArray, TagDetection


def wrap_angle(angle: float) -> float:
    """Đưa góc về khoảng [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_rotmat(qx, qy, qz, qw):
    """Chuyển đổi quaternion sang ma trận quay 3x3."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-8:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),       2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw),       1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw),       2.0 * (qy * qz + qx * qw),       1.0 - 2.0 * (qx * qx + qy * qy)]
    ], dtype=np.float64)


class DockServer(Node):
    """Action Server thực thi visual servoing căn chỉnh tinh trước ô kệ."""

    def __init__(self):
        super().__init__(
            "dock_server",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )

        # Tham số PBVS
        self.declare_parameter("target_standoff", 0.32)
        self.declare_parameter("tol_dist", 0.005)
        self.declare_parameter("tol_lat", 0.005)
        self.declare_parameter("tol_yaw", 0.0349)  # ~2 độ
        self.declare_parameter("v_max", 0.08)
        self.declare_parameter("w_max", 0.15)
        self.declare_parameter("safety_min_px", 0.28)
        self.declare_parameter("kp_x", 0.8)
        self.declare_parameter("kp_y", 0.8)
        self.declare_parameter("kp_w", 1.0)
        self.declare_parameter("sign_x", 1.0)
        self.declare_parameter("sign_y", 1.0)
        self.declare_parameter("sign_w", 1.0)
        self.declare_parameter("consecutive_frames_stop", 10)
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("tag_lost_timeout_sec", 1.0)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("cmd_topic", "/cmd_vel_servo")

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # Lưu trữ tag detection mới nhất
        self.latest_tags = {}
        self.last_tag_time = 0.0

        self.callback_group = ReentrantCallbackGroup()
        self.tags_sub = self.create_subscription(
            TagArray, "/vision/tags", self.tags_cb, 10, callback_group=self.callback_group
        )
        self._action_server = ActionServer(
            self,
            DockToSlot,
            "dock_to_slot",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self._action_server_wh = ActionServer(
            self,
            DockToSlot,
            "/wh/dock_to_slot",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            f"DockServer đã khởi động. Lệnh servo ra: {self.cmd_topic}, "
            f"Standoff={self.get_parameter('target_standoff').value}m, "
            f"Safety min={self.get_parameter('safety_min_px').value}m"
        )

    def tags_cb(self, msg: TagArray):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        for tag in msg.tags:
            self.latest_tags[tag.id] = (tag, now_sec)
        self.last_tag_time = now_sec

    def verify_slot(self, expected_id: int) -> bool:
        """Kiểm tra tag ID ô kệ hiện tại có đúng id yêu cầu không."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if expected_id not in self.latest_tags:
            return False
        tag, stamp = self.latest_tags[expected_id]
        if (now_sec - stamp) > 1.0:
            return False
        return tag.id == expected_id

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Nhận yêu cầu DockToSlot: slot_tag_id={goal_request.slot_tag_id}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Nhận yêu cầu hủy DockToSlot.")
        self.stop_robot()
        return CancelResponse.ACCEPT

    def stop_robot(self):
        """Dừng robot khẩn cấp bằng cách phát Twist 0."""
        stop_cmd = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop_cmd)
            time.sleep(0.01)

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        slot_id = int(goal.slot_tag_id)
        target_standoff = float(goal.target_standoff) if goal.target_standoff > 0.0 else float(self.get_parameter("target_standoff").value)
        tol_lat = float(goal.tol_lateral) if goal.tol_lateral > 0.0 else float(self.get_parameter("tol_lat").value)
        tol_yaw = float(goal.tol_yaw) if goal.tol_yaw > 0.0 else float(self.get_parameter("tol_yaw").value)
        tol_dist = float(self.get_parameter("tol_dist").value)

        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)
        safety_min_px = float(self.get_parameter("safety_min_px").value)

        kp_x = float(self.get_parameter("kp_x").value)
        kp_y = float(self.get_parameter("kp_y").value)
        kp_w = float(self.get_parameter("kp_w").value)

        sign_x = float(self.get_parameter("sign_x").value)
        sign_y = float(self.get_parameter("sign_y").value)
        sign_w = float(self.get_parameter("sign_w").value)

        consec_target = int(self.get_parameter("consecutive_frames_stop").value)
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        tag_lost_timeout = float(self.get_parameter("tag_lost_timeout_sec").value)
        max_retries = int(self.get_parameter("max_retries").value)

        self.get_logger().info(
            f">>> Bắt đầu Visual Servoing vào ô tag ID={slot_id}: "
            f"standoff={target_standoff:.3f}m, tol_lat={tol_lat*1000:.1f}mm, tol_yaw={math.degrees(tol_yaw):.1f}deg"
        )

        result = DockToSlot.Result()
        feedback = DockToSlot.Feedback()

        start_time = self.get_clock().now().nanoseconds * 1e-9
        self.latest_tags.pop(slot_id, None)
        iteration = 0
        consec_ok = 0
        retries = 0

        rate_hz = 20.0
        dt = 1.0 / rate_hz

        # Vùng chết (deadband = nửa dung sai)
        deadband_lat = tol_lat * 0.5
        deadband_dist = tol_dist * 0.5
        deadband_yaw = tol_yaw * 0.5

        final_lat_err = 0.0
        final_dist_err = 0.0
        final_yaw_err = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = "Đã hủy bởi người dùng."
                return result

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if (now_sec - start_time) > timeout_sec:
                self.stop_robot()
                goal_handle.abort()
                result.success = False
                result.final_lateral_err = final_lat_err
                result.final_dist_err = final_dist_err
                result.final_yaw_err = final_yaw_err
                result.message = f"TIMEOUT vượt quá {timeout_sec}s!"
                self.get_logger().error(result.message)
                return result

            # Tra cứu tag yêu cầu
            tag_data = self.latest_tags.get(slot_id, None)
            is_tag_fresh = (tag_data is not None) and ((now_sec - tag_data[1]) < tag_lost_timeout)

            if not is_tag_fresh:
                consec_ok = 0
                if (now_sec - tag_data[1] if tag_data else now_sec - start_time) > tag_lost_timeout:
                    if retries < max_retries:
                        retries += 1
                        self.get_logger().warn(f"Mất tag {slot_id} > {tag_lost_timeout}s! Hồi phục lần {retries}: lùi nhẹ...")
                        # Lùi nhẹ 1s để tìm lại tag
                        cmd_back = Twist()
                        cmd_back.linear.x = -0.05
                        for _ in range(int(rate_hz)):
                            self.cmd_pub.publish(cmd_back)
                            time.sleep(dt)
                        self.stop_robot()
                        continue
                    else:
                        self.stop_robot()
                        goal_handle.abort()
                        result.success = False
                        result.message = "TAG_LOST: Không tìm thấy tag sau số lần thử lại tối đa!"
                        self.get_logger().error(result.message)
                        return result
                time.sleep(dt)
                continue

            tag_msg, _ = tag_data
            p = tag_msg.pose_base.pose.position
            q = tag_msg.pose_base.pose.orientation
            px, py = p.x, p.y

            # Khóa an toàn khoảng cách: px < 0.28m dừng khẩn cấp
            if px < safety_min_px:
                self.stop_robot()
                goal_handle.abort()
                result.success = False
                result.final_lateral_err = float(py)
                result.final_dist_err = float(px - target_standoff)
                result.final_yaw_err = final_yaw_err
                result.message = f"SAFETY_STOP: px={px:.3f}m < {safety_min_px}m! Nguy cơ va chạm kệ hàng!"
                self.get_logger().error(result.message)
                return result

            # Tính sai lệch
            e_dist = px - target_standoff
            e_lat = py
            R_base_tag = quat_to_rotmat(q.x, q.y, q.z, q.w)
            # Pháp tuyến n của tag (hướng ra khỏi mặt kệ về phía robot)
            n = R_base_tag.dot(np.array([0.0, 0.0, 1.0]))
            n_angle = math.atan2(n[1], n[0])
            e_yaw = wrap_angle(n_angle - math.pi)

            final_lat_err = float(e_lat)
            final_dist_err = float(e_dist)
            final_yaw_err = float(e_yaw)

            # Gửi feedback
            iteration += 1
            feedback.lateral_err = final_lat_err
            feedback.dist_err = final_dist_err
            feedback.yaw_err = final_yaw_err
            feedback.iteration = iteration
            goal_handle.publish_feedback(feedback)

            # Kiểm tra điều kiện hoàn tất
            in_tol_lat = abs(e_lat) < tol_lat
            in_tol_dist = abs(e_dist) < tol_dist
            in_tol_yaw = abs(e_yaw) < tol_yaw

            if in_tol_lat and in_tol_dist and in_tol_yaw:
                consec_ok += 1
                if consec_ok >= consec_target:
                    self.stop_robot()
                    # Xác minh lại ID sau docking
                    if not self.verify_slot(slot_id):
                        goal_handle.abort()
                        result.success = False
                        result.message = "SLOT_MISMATCH: Sai lệch ID ô kệ sau docking!"
                        self.get_logger().error(result.message)
                        return result

                    goal_handle.succeed()
                    result.success = True
                    result.final_lateral_err = final_lat_err
                    result.final_dist_err = final_dist_err
                    result.final_yaw_err = final_yaw_err
                    result.message = (
                        f"DOCK_SUCCESS: Đạt dung sai! "
                        f"lat_err={final_lat_err*1000:.1f}mm, dist_err={final_dist_err*1000:.1f}mm, "
                        f"yaw_err={math.degrees(final_yaw_err):.2f}deg trong {iteration} vòng lặp."
                    )
                    self.get_logger().info(result.message)
                    return result
            else:
                consec_ok = 0

            # Áp dụng deadband
            ctrl_dist = 0.0 if abs(e_dist) <= deadband_dist else e_dist
            ctrl_lat = 0.0 if abs(e_lat) <= deadband_lat else e_lat
            ctrl_yaw = 0.0 if abs(e_yaw) <= deadband_yaw else e_yaw

            # Tính toán vận tốc PBVS
            # e_dist > 0 -> robot xa hơn mục tiêu -> cần tiến (+x, vx > 0)
            vx_cmd = np.clip(sign_x * kp_x * ctrl_dist, -v_max, v_max)
            # e_lat > 0 -> tag ở bên trái robot -> cần dịch trái (+y, vy > 0)
            vy_cmd = np.clip(sign_y * kp_y * ctrl_lat, -v_max, v_max)
            # e_yaw > 0 -> cần xoay CCW (+z, wz > 0)
            wz_cmd = np.clip(sign_w * kp_w * ctrl_yaw, -w_max, w_max)

            # Ưu tiên an toàn: Nếu sai lệch góc hoặc ngang còn lớn, tạm dừng tiến/lùi (vx = 0)
            if abs(e_lat) > 0.02 or abs(e_yaw) > math.radians(4.0):
                vx_cmd = 0.0

            cmd = Twist()
            cmd.linear.x = float(vx_cmd)
            cmd.linear.y = float(vy_cmd)
            cmd.angular.z = float(wz_cmd)
            self.cmd_pub.publish(cmd)

            time.sleep(dt)


def main(args=None):
    rclpy.init(args=args)
    server = DockServer()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop_robot()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
