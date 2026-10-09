#!/usr/bin/env python3
"""benchmark_docking.py – Thí nghiệm định lượng 30 lần căn chỉnh DockToSlot (Mục 5.6).

Quy trình mỗi lượt:
1. Đặt robot trước ô kệ (ví dụ Slot 2: Tag 2 tại x=1.75, y=0.60) ở điểm chờ
2. Dịch lệch ngẫu nhiên: ngang +-5 cm, dọc +-3 cm, góc +-10 độ
3. Ghi nhận sai số "trước" (Ground truth so với vị trí lý tưởng: y_target = 0.60 - 0.32 = 0.28m, x=1.75m, yaw=pi/2)
4. Gọi action DockToSlot
5. Ghi nhận sai số "sau" (Ground truth) và thời gian thực thi
6. Lặp lại 30 lần, xuất CSV và vẽ đồ thị boxplot / histogram
"""

import csv
import math
import os
import random
import sys
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from wh_interfaces.action import DockToSlot


TAG_SLOT_5 = {
    "id": 5,
    "x": 1.75,
    "y": 0.599,
    "z": 0.36,
}

TARGET_STANDOFF = 0.32
IDEAL_ROBOT_Y = TAG_SLOT_5["y"] - TARGET_STANDOFF  # 0.279 m
IDEAL_ROBOT_X = TAG_SLOT_5["x"]                   # 1.75 m
IDEAL_ROBOT_YAW = math.pi / 2.0                    # 1.5708 rad


class DockingBenchmarkNode(Node):
    def __init__(self):
        super().__init__(
            "docking_benchmark",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.set_state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.odom_sub = self.create_subscription(Odometry, "/odom_gt", self.odom_cb, 10)
        self.dock_client = ActionClient(self, DockToSlot, "dock_to_slot")

        self.robot_pose = None

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = (p.x, p.y, p.z, yaw)

    def set_robot_pose(self, x: float, y: float, yaw: float):
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
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
        time.sleep(1.0)
        # Bơm sự kiện spin để nhận odom_gt và làm mới buffer
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def run_dock_action(self, slot_id=5, standoff=0.32, tol_lat=0.003, tol_yaw=0.025):
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("DockToSlot action server chưa sẵn sàng!")
            return False, 0.0, []

        goal = DockToSlot.Goal()
        goal.slot_tag_id = slot_id
        goal.target_standoff = standoff
        goal.tol_lateral = tol_lat
        goal.tol_yaw = tol_yaw

        trace_data = []

        def feedback_cb(fb_msg):
            fb = fb_msg.feedback
            trace_data.append((
                time.time(),
                fb.lateral_err * 1000.0,
                fb.dist_err * 1000.0,
                math.degrees(fb.yaw_err)
            ))

        send_future = self.dock_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Mục tiêu DockToSlot bị từ chối!")
            return False, 0.0, trace_data

        t_start = time.time()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        duration = time.time() - t_start

        res = result_future.result().result
        return res.success, duration, trace_data

    def compute_ground_truth_errors(self):
        if self.robot_pose is None:
            return 999.0, 999.0, 999.0
        rx, ry, _, ryaw = self.robot_pose
        dx = TAG_SLOT_5["x"] - rx
        dy = TAG_SLOT_5["y"] - ry
        gt_px = dx * math.cos(ryaw) + dy * math.sin(ryaw)
        gt_py = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
        e_dist_mm = abs(gt_px - TARGET_STANDOFF) * 1000.0
        e_lat_mm = abs(gt_py) * 1000.0
        dyaw = math.atan2(math.sin(ryaw - IDEAL_ROBOT_YAW), math.cos(ryaw - IDEAL_ROBOT_YAW))
        e_yaw_deg = abs(math.degrees(dyaw))
        return e_lat_mm, e_dist_mm, e_yaw_deg

    def run_benchmark(self, num_runs=30):
        self.get_logger().info(f"=== BẮT ĐẦU BENCHMARK DOCKING {num_runs} LẦN ===")
        random.seed(42)

        results = []
        sample_trace = None

        for i in range(1, num_runs + 1):
            # Tạo độ lệch ngẫu nhiên: ngang +-5 cm, dọc +-3 cm, góc +-10 độ
            dx = random.uniform(-0.05, 0.05)
            dy = random.uniform(-0.03, 0.03)
            dyaw = math.radians(random.uniform(-10.0, 10.0))

            start_x = IDEAL_ROBOT_X + dx
            start_y = 0.08 + dy     # Đứng tại vùng chờ (~0.5m trước kệ)
            start_yaw = IDEAL_ROBOT_YAW + dyaw

            self.set_robot_pose(start_x, start_y, start_yaw)
            lat_before, dist_before, yaw_before = self.compute_ground_truth_errors()

            self.get_logger().info(
                f"Lần {i:02d}/{num_runs}: Đặt lệch dx={dx*100:+.1f}cm, dy={dy*100:+.1f}cm, dyaw={math.degrees(dyaw):+.1f}° | "
                f"Trước: lat={lat_before:.1f}mm, dist={dist_before:.1f}mm, yaw={yaw_before:.1f}°"
            )

            # Chạy servoing
            success, dur, trace = self.run_dock_action(slot_id=TAG_SLOT_5["id"])

            if sample_trace is None and trace:
                sample_trace = trace

            # Đợi robot dừng hẳn
            time.sleep(0.5)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)

            lat_after, dist_after, yaw_after = self.compute_ground_truth_errors()

            is_pass = success and (lat_after <= 5.0) and (dist_after <= 5.0) and (yaw_after <= 2.0)
            status_str = "PASS" if is_pass else "FAIL"

            self.get_logger().info(
                f"        Kết quả {i:02d}: {status_str} (t={dur:.2f}s) | "
                f"Sau: lat={lat_after:.2f}mm, dist={dist_after:.2f}mm, yaw={yaw_after:.2f}°"
            )

            results.append({
                "trial": i,
                "lat_before_mm": lat_before,
                "dist_before_mm": dist_before,
                "yaw_before_deg": yaw_before,
                "lat_after_mm": lat_after,
                "dist_after_mm": dist_after,
                "yaw_after_deg": yaw_after,
                "duration_s": dur,
                "success": success,
                "pass_criteria": is_pass
            })

        # Lưu CSV
        os.makedirs("/home/lehuythien/warehouse_ws/docs/results", exist_ok=True)
        csv_file = "/home/lehuythien/warehouse_ws/docs/results/docking_errors.csv"
        with open(csv_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "trial", "lat_before_mm", "dist_before_mm", "yaw_before_deg",
                "lat_after_mm", "dist_after_mm", "yaw_after_deg", "duration_s",
                "success", "pass_criteria"
            ])
            writer.writeheader()
            for r in results:
                writer.writerow(r)

        self.get_logger().info(f"Đã lưu toàn bộ {len(results)} lượt đo vào {csv_file}")

        # Thống kê
        success_count = sum(1 for r in results if r["success"])
        pass_count = sum(1 for r in results if r["pass_criteria"])
        avg_lat_before = np.mean([r["lat_before_mm"] for r in results])
        avg_lat_after = np.mean([r["lat_after_mm"] for r in results])
        avg_dist_before = np.mean([r["dist_before_mm"] for r in results])
        avg_dist_after = np.mean([r["dist_after_mm"] for r in results])
        avg_yaw_before = np.mean([r["yaw_before_deg"] for r in results])
        avg_yaw_after = np.mean([r["yaw_after_deg"] for r in results])
        avg_dur = np.mean([r["duration_s"] for r in results])

        self.get_logger().info(
            f"\n=== TỔNG KẾT NGHIỆM THU BENCHMARK ({num_runs} LẦN) ===\n"
            f"- Số lần thành công: {success_count}/{num_runs} ({success_count/num_runs*100:.1f}%)\n"
            f"- Số lần đạt chuẩn (<=5mm, <=2°): {pass_count}/{num_runs} ({pass_count/num_runs*100:.1f}%)\n"
            f"- Sai số ngang TB: {avg_lat_before:.1f} mm -> {avg_lat_after:.2f} mm (giảm {(1-avg_lat_after/avg_lat_before)*100:.1f}%)\n"
            f"- Sai số dọc TB:   {avg_dist_before:.1f} mm -> {avg_dist_after:.2f} mm (giảm {(1-avg_dist_after/avg_dist_before)*100:.1f}%)\n"
            f"- Sai số góc TB:   {avg_yaw_before:.2f}° -> {avg_yaw_after:.2f}° (giảm {(1-avg_yaw_after/avg_yaw_before)*100:.1f}%)\n"
            f"- Thời gian TB:    {avg_dur:.2f} s"
        )

        # Vẽ biểu đồ
        self.plot_results(results, sample_trace)
        return pass_count >= int(num_runs * 0.90) and success_count >= (num_runs - 1)

    def plot_results(self, results, sample_trace):
        # 1. Boxplots Trước vs Sau
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle("So sánh sai số trước và sau Visual Servoing (30 lần)", fontsize=14, fontweight="bold")

        lat_b = [r["lat_before_mm"] for r in results]
        lat_a = [r["lat_after_mm"] for r in results]
        axes[0].boxplot([lat_b, lat_a], labels=["Trước", "Sau"])
        axes[0].axhline(5.0, color="r", linestyle="--", label="Ngưỡng dung sai (5 mm)")
        axes[0].set_title("Sai số ngang (Lateral Error)")
        axes[0].set_ylabel("Sai số (mm)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        dist_b = [r["dist_before_mm"] for r in results]
        dist_a = [r["dist_after_mm"] for r in results]
        axes[1].boxplot([dist_b, dist_a], labels=["Trước", "Sau"])
        axes[1].axhline(5.0, color="r", linestyle="--", label="Ngưỡng dung sai (5 mm)")
        axes[1].set_title("Sai số khoảng cách (Distance Error)")
        axes[1].set_ylabel("Sai số (mm)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        yaw_b = [r["yaw_before_deg"] for r in results]
        yaw_a = [r["yaw_after_deg"] for r in results]
        axes[2].boxplot([yaw_b, yaw_a], labels=["Trước", "Sau"])
        axes[2].axhline(2.0, color="r", linestyle="--", label="Ngưỡng dung sai (2°)")
        axes[2].set_title("Sai số góc (Heading Error)")
        axes[2].set_ylabel("Sai số (độ)")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        fig.savefig("/home/lehuythien/warehouse_ws/docs/results/docking_boxplots.png", dpi=200)
        plt.close(fig)

        # 2. Histograms sai số Sau
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
        fig.suptitle("Phân phối sai số sau Visual Servoing (30 lần)", fontsize=14, fontweight="bold")

        axes[0].hist(lat_a, bins=8, color="#2ca02c", edgecolor="black", alpha=0.7)
        axes[0].axvline(5.0, color="r", linestyle="--", label="Ngưỡng 5 mm")
        axes[0].set_title("Sai số ngang sau (mm)")
        axes[0].set_xlabel("mm")
        axes[0].set_ylabel("Số lần")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].hist(dist_a, bins=8, color="#1f77b4", edgecolor="black", alpha=0.7)
        axes[1].axvline(5.0, color="r", linestyle="--", label="Ngưỡng 5 mm")
        axes[1].set_title("Sai số khoảng cách sau (mm)")
        axes[1].set_xlabel("mm")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        axes[2].hist(yaw_a, bins=8, color="#ff7f0e", edgecolor="black", alpha=0.7)
        axes[2].axvline(2.0, color="r", linestyle="--", label="Ngưỡng 2°")
        axes[2].set_title("Sai số góc sau (độ)")
        axes[2].set_xlabel("độ")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        fig.savefig("/home/lehuythien/warehouse_ws/docs/results/docking_histograms.png", dpi=200)
        plt.close(fig)

        # 3. Trace đồ thị hội tụ theo thời gian
        if sample_trace and len(sample_trace) > 1:
            t0 = sample_trace[0][0]
            ts = [s[0] - t0 for s in sample_trace]
            lats = [s[1] for s in sample_trace]
            dists = [s[2] for s in sample_trace]
            yaws = [s[3] for s in sample_trace]

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            fig.suptitle("Đồ thị hội tụ sai số Visual Servoing theo thời gian", fontsize=13, fontweight="bold")

            ax1.plot(ts, lats, label="Sai số ngang e_lat (mm)", color="green", linewidth=2)
            ax1.plot(ts, dists, label="Sai số khoảng cách e_dist (mm)", color="blue", linewidth=2)
            ax1.axhline(5.0, color="gray", linestyle="--", alpha=0.7)
            ax1.axhline(-5.0, color="gray", linestyle="--", alpha=0.7)
            ax1.set_ylabel("Sai số tịnh tiến (mm)")
            ax1.legend()
            ax1.grid(True, alpha=0.3)

            ax2.plot(ts, yaws, label="Sai số góc e_yaw (độ)", color="orange", linewidth=2)
            ax2.axhline(2.0, color="gray", linestyle="--", alpha=0.7)
            ax2.axhline(-2.0, color="gray", linestyle="--", alpha=0.7)
            ax2.set_xlabel("Thời gian (giây)")
            ax2.set_ylabel("Sai số góc (độ)")
            ax2.legend()
            ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            fig.savefig("/home/lehuythien/warehouse_ws/docs/results/docking_trace.png", dpi=200)
            plt.close(fig)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Docking Benchmark")
    parser.add_argument("--runs", type=int, default=30, help="Số lần chạy thí nghiệm docking (mặc định 30)")
    args = parser.parse_args()

    rclpy.init()
    node = DockingBenchmarkNode()
    passed = node.run_benchmark(num_runs=args.runs)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
