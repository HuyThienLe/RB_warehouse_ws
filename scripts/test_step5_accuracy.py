#!/usr/bin/env python3
"""test_step5_accuracy.py – Đo kiểm định lượng độ chính xác nhận diện ArUco tag.

Thực hiện:
- Đặt robot lần lượt cách tag ô kệ (ID=2) ở các khoảng cách: 0.20, 0.30, 0.40, 0.50 m
- Đợi bộ lọc TagFilter thu thập đủ 10 mẫu để ổn định
- So sánh khoảng cách và độ lệch ngang đo được với ground truth (/odom_gt và vị trí thật của tag)
- Lưu bảng kết quả ra docs/results/tag_accuracy.csv
- Đánh giá tiêu chuẩn: sai số tại 0.30 m <= 3.0 mm
"""

import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from nav_msgs.msg import Odometry
from wh_interfaces.msg import TagArray


TAG_WORLD = {
    "x": 1.75,
    "y": 0.599,
    "z": 0.360,
    "id": 5
}


class AccuracyTester(Node):
    def __init__(self):
        super().__init__(
            "accuracy_tester",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.set_state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.odom_sub = self.create_subscription(Odometry, "/odom_gt", self.odom_cb, 10)
        self.tags_sub = self.create_subscription(TagArray, "/vision/tags", self.tags_cb, 10)

        self.robot_pose = None
        self.latest_tag = None
        self.tag_history = []

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Tính yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = (p.x, p.y, p.z, yaw)

    def tags_cb(self, msg: TagArray):
        for t in msg.tags:
            if t.id == TAG_WORLD["id"]:
                self.latest_tag = t
                self.tag_history.append((
                    t.pose_base.pose.position.x,
                    t.pose_base.pose.position.y,
                    t.pose_base.pose.position.z,
                    t.reproj_error_px
                ))
                if len(self.tag_history) > 30:
                    self.tag_history.pop(0)

    def teleport_robot(self, x: float, y: float, yaw: float):
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Dịch vụ /gazebo/set_entity_state chưa sẵn sàng!")
            return False

        req = SetEntityState.Request()
        req.state.name = "warehouse_robot"
        req.state.reference_frame = "world"
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = 0.02
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        future = self.set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def run_tests(self):
        distances = [0.30, 0.35, 0.40, 0.50]
        results = []

        self.get_logger().info("=== BẮT ĐẦU ĐO KIỂM ĐỘ CHÍNH XÁC NHẬN TAG (MỤC 5.3) ===")

        for dist in distances:
            # Robot nhìn theo trục +y về phía tag ô kệ
            # Tag ở y = 0.60 -> robot đứng ở y = 0.60 - dist
            rx = TAG_WORLD["x"]
            ry = TAG_WORLD["y"] - dist
            ryaw = math.pi / 2.0

            self.get_logger().info(f"Đang di chuyển robot đến vị trí cách tag {dist:.2f} m (x={rx:.2f}, y={ry:.2f})...")
            self.teleport_robot(rx, ry, ryaw)
            self.tag_history.clear()

            # Chờ để nhận ít nhất 10 khung hình ảnh
            start_wait = time.time()
            while time.time() - start_wait < 3.0 and len(self.tag_history) < 15:
                rclpy.spin_once(self, timeout_sec=0.1)

            if len(self.tag_history) < 5 or self.robot_pose is None:
                self.get_logger().error(f"Không nhận được tag ở khoảng cách {dist:.2f} m! (history={len(self.tag_history)})")
                continue

            # Ground truth: vị trí tag trong base_link
            rob_x, rob_y, rob_z, rob_yaw = self.robot_pose
            dx = TAG_WORLD["x"] - rob_x
            dy = TAG_WORLD["y"] - rob_y
            gt_px = dx * math.cos(rob_yaw) + dy * math.sin(rob_yaw)
            gt_py = -dx * math.sin(rob_yaw) + dy * math.cos(rob_yaw)

            # Đo đạc sau lọc: trung vị các mẫu gần nhất
            xs = [h[0] for h in self.tag_history[-10:]]
            ys = [h[1] for h in self.tag_history[-10:]]
            reprojs = [h[3] for h in self.tag_history[-10:]]

            meas_px = float(sorted(xs)[len(xs) // 2])
            meas_py = float(sorted(ys)[len(ys) // 2])
            avg_reproj = float(sum(reprojs) / len(reprojs))

            err_dist_mm = abs(meas_px - gt_px) * 1000.0
            err_lat_mm = abs(meas_py - gt_py) * 1000.0
            total_err_mm = math.sqrt(err_dist_mm**2 + err_lat_mm**2)

            self.get_logger().info(
                f"Khoảng cách {dist:.2f} m: "
                f"GT=(px={gt_px:.4f}, py={gt_py:.4f}) | "
                f"Đo=(px={meas_px:.4f}, py={meas_py:.4f}) | "
                f"Sai số dist={err_dist_mm:.2f} mm, lat={err_lat_mm:.2f} mm, tổng={total_err_mm:.2f} mm | "
                f"Reproj err={avg_reproj:.3f} px"
            )

            results.append({
                "target_dist_m": dist,
                "gt_px_m": gt_px,
                "gt_py_m": gt_py,
                "meas_px_m": meas_px,
                "meas_py_m": meas_py,
                "err_dist_mm": err_dist_mm,
                "err_lat_mm": err_lat_mm,
                "total_err_mm": total_err_mm,
                "avg_reproj_px": avg_reproj,
                "samples": len(xs)
            })

        # Lưu CSV
        os.makedirs("/home/lehuythien/warehouse_ws/docs/results", exist_ok=True)
        csv_path = "/home/lehuythien/warehouse_ws/docs/results/tag_accuracy.csv"
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "target_dist_m", "gt_px_m", "gt_py_m", "meas_px_m", "meas_py_m",
                "err_dist_mm", "err_lat_mm", "total_err_mm", "avg_reproj_px", "samples"
            ])
            writer.writeheader()
            for r in results:
                writer.writerow(r)

        self.get_logger().info(f"Đã lưu bảng kết quả vào {csv_path}")

        # Kiểm tra điều kiện nghiệm thu tại khoảng cách 0.30 m
        r30 = next((r for r in results if abs(r["target_dist_m"] - 0.30) < 0.01), None)
        if r30:
            if r30["total_err_mm"] <= 3.0:
                self.get_logger().info(f"[NGHIỆM THU ĐẠT] Sai số tại 0.30 m là {r30['total_err_mm']:.2f} mm <= 3.0 mm!")
                return True
            else:
                self.get_logger().warn(f"[CHƯA ĐẠT] Sai số tại 0.30 m là {r30['total_err_mm']:.2f} mm > 3.0 mm!")
                return False
        return False


def main():
    rclpy.init()
    tester = AccuracyTester()
    ok = tester.run_tests()
    tester.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
