#!/usr/bin/env python3
"""benchmark_nav.py – Đo kiểm & nghiệm thu hệ thống Điều hướng Nav2 (Step 04).

Thực hiện:
1. Test 20 mục tiêu ngẫu nhiên giữa các waypoint (HOME, IN, OUT, 12 điểm chờ ô kệ).
   - Đo thời gian, tỷ lệ thành công (yêu cầu >= 19/20), sai số vị trí (mm) và hướng (độ) so với /odom_gt.
   - Lưu kết quả: docs/results/nav_waypoints.csv
2. Test vật cản bất ngờ (5 lần):
   - Thả vật cản bất ngờ bằng /wh/spawn_obstacle khi robot đang di chuyển.
   - Đo khoảng cách nhỏ nhất tới vật cản (yêu cầu >= 0.05m, 0 va chạm).
   - Lưu kết quả: docs/results/nav_obstacles.csv
3. Test E-Stop:
   - Phát /estop = true khi robot đang đi, kiểm tra /cmd_vel về 0 trong < 0.3s.
   - Lưu kết quả: docs/results/nav_estop.csv
4. Xuất biểu đồ:
   - docs/results/nav_error_histogram.png
   - docs/results/nav_trajectories.png
"""

import argparse
import csv
import math
import os
import random
import sys
import time
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

try:
    from wh_interfaces.srv import SpawnObstacle
    HAS_SPAWN_OBS = True
except ImportError:
    HAS_SPAWN_OBS = False


WS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WP_FILE = os.path.join(WS_DIR, "src", "wh_navigation", "config", "waypoints.yaml")
RESULTS_DIR = os.path.join(WS_DIR, "docs", "results")


class NavBenchmarker(Node):
    def __init__(self):
        super().__init__(
            "nav_benchmarker",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        os.makedirs(RESULTS_DIR, exist_ok=True)

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.odom_sub = self.create_subscription(Odometry, "/odom_gt", self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)

        if HAS_SPAWN_OBS:
            self.spawn_obs_cli = self.create_client(SpawnObstacle, "/wh/spawn_obstacle")
            self.clear_obs_cli = self.create_client(Trigger, "/wh/clear_obstacle")
        else:
            self.spawn_obs_cli = None
            self.clear_obs_cli = None

        self.current_pose = None  # (x, y, yaw)
        self.min_scan_range = 99.0
        self.last_cmd_vel = None
        self.trajectory_history = []  # List of (x, y)
        self.waypoints = self.load_waypoints()

        self.get_logger().info("NavBenchmarker node sẵn sàng.")

    def load_waypoints(self):
        if not os.path.exists(WP_FILE):
            self.get_logger().error(f"Không tìm thấy {WP_FILE}")
            return {}
        with open(WP_FILE, "r") as f:
            data = yaml.safe_load(f)
        return data.get("waypoints", {})

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)
        self.trajectory_history.append((p.x, p.y))

    def scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid:
            self.min_scan_range = min(valid)

    def cmd_cb(self, msg: Twist):
        v = math.hypot(msg.linear.x, msg.linear.y)
        self.last_cmd_vel = v

    def start_nav_goal_async(self, x: float, y: float, yaw: float):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            return None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result().accepted:
            return future.result()
        return None

    def send_nav_goal(self, x: float, y: float, yaw: float, timeout_sec: float = 60.0):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            return False, "Server not available"

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = rclpy.time.Time().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or not future.result().accepted:
            return False, "Goal rejected"

        goal_handle = future.result()
        res_future = goal_handle.get_result_async()

        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if res_future.done():
                status = res_future.result().status
                return (status == GoalStatus.STATUS_SUCCEEDED), f"Status {status}"

        # Timeout -> hủy goal
        goal_handle.cancel_goal_async()
        return False, "Timeout"


def run_benchmark_waypoints(bench: NavBenchmarker, num_trials: int = 20):
    print("\n" + "=" * 65)
    print(f"  PHẦN 1: THỬ NGHIỆM {num_trials} MỤC TIÊU NGẪU NHIÊN GIỮA CÁC WAYPOINT")
    print("=" * 65)

    wp_keys = list(bench.waypoints.keys())
    if len(wp_keys) < 2:
        print("[ERROR] Không đủ waypoints để benchmark!")
        return []

    # Danh sách các cặp hành trình kế tiếp liên tục trong kho
    csv_rows = []
    success_count = 0
    all_pos_errors = []
    all_yaw_errors = []
    trajectories_plot = []

    rng = random.Random(42)
    pairs = []
    current_wp = "HOME"
    for _ in range(num_trials):
        candidates = [k for k in wp_keys if k != current_wp]
        dst = rng.choice(candidates)
        pairs.append((current_wp, dst))
        current_wp = dst

    for i, (src, dst) in enumerate(pairs, 1):
        target = bench.waypoints[dst]
        tx, ty, tyaw = float(target["x"]), float(target["y"]), float(target["yaw"])

        print(f"\n[Trial {i:02d}/{num_trials}] Đi từ {src} -> {dst} (Target: {tx:.3f}, {ty:.3f}, yaw={tyaw:.3f})...")
        time.sleep(0.5)
        bench.trajectory_history = []
        start_time = time.time()

        success, msg = bench.send_nav_goal(tx, ty, tyaw, timeout_sec=65.0)
        elapsed = time.time() - start_time

        # Tính sai số so với ground truth /odom_gt
        pos_err_mm = 0.0
        yaw_err_deg = 0.0
        if bench.current_pose is not None:
            cx, cy, cyaw = bench.current_pose
            pos_err_mm = math.hypot(tx - cx, ty - cy) * 1000.0
            dyaw = math.atan2(math.sin(tyaw - cyaw), math.cos(tyaw - cyaw))
            yaw_err_deg = abs(math.degrees(dyaw))

        status_str = "PASS" if success else "FAIL"
        if success:
            success_count += 1
            all_pos_errors.append(pos_err_mm)
            all_yaw_errors.append(yaw_err_deg)
            if i in [1, 5, 10, 15, 20]:
                trajectories_plot.append((dst, list(bench.trajectory_history)))

        print(f"  -> Kết quả: {status_str} trong {elapsed:.2f}s | Sai số vị trí: {pos_err_mm:.1f} mm | Sai số góc: {yaw_err_deg:.2f}°")

        csv_rows.append({
            "trial": i,
            "source": src,
            "target": dst,
            "target_x": tx,
            "target_y": ty,
            "target_yaw": tyaw,
            "success": success,
            "elapsed_sec": round(elapsed, 2),
            "pos_error_mm": round(pos_err_mm, 1),
            "yaw_error_deg": round(yaw_err_deg, 2),
            "status_msg": msg,
        })

    # Lưu CSV
    csv_file = os.path.join(RESULTS_DIR, "nav_waypoints.csv")
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_rows[0].keys())
        writer.writeheader()
        writer.writerows(csv_rows)
    print(f"\n[OK] Đã ghi kết quả {len(csv_rows)} trials vào: {csv_file}")

    rate = (success_count / num_trials) * 100.0
    print(f"Tỷ lệ thành công: {success_count}/{num_trials} ({rate:.1f}%) [Yêu cầu >= 19/20: {'ĐẠT' if success_count >= 19 else 'CHƯA ĐẠT'}]")
    if all_pos_errors:
        print(f"Sai số vị trí trung bình: {sum(all_pos_errors)/len(all_pos_errors):.1f} mm (Max: {max(all_pos_errors):.1f} mm)")
        print(f"Sai số góc trung bình   : {sum(all_yaw_errors)/len(all_yaw_errors):.2f}° (Max: {max(all_yaw_errors):.2f}°)")

    return all_pos_errors, all_yaw_errors, trajectories_plot


def run_benchmark_obstacles(bench: NavBenchmarker, num_trials: int = 5):
    print("\n" + "=" * 65)
    print(f"  PHẦN 2: THỬ NGHIỆM TRÁNH VẬT CẢN BẤT NGỜ ({num_trials} LẦN)")
    print("=" * 65)

    csv_rows = []
    collisions = 0

    test_points = [
        ((-2.2, 0.0), (1.5, 0.0), (0.0, 0.0)),
        ((1.5, 0.0), (-2.0, 0.0), (-0.5, 0.0)),
        ((-2.0, 0.3), (1.5, 0.3), (0.2, 0.3)),
        ((1.5, -0.3), (-2.0, -0.3), (-0.2, -0.3)),
        ((-2.0, 0.0), (1.25, 0.0), (0.5, 0.0)),
    ]

    for i in range(num_trials):
        start_pt, target_pt, obs_pt = test_points[i % len(test_points)]
        tx, ty = target_pt
        ox, oy = obs_pt

        # Xóa vật cản cũ nếu có
        if bench.clear_obs_cli and bench.clear_obs_cli.wait_for_service(timeout_sec=2.0):
            bench.clear_obs_cli.call_async(Trigger.Request())
            time.sleep(0.5)

        print(f"\n[Obs Trial {i+1}/{num_trials}] Robot đi tới ({tx}, {ty}), sẽ thả vật cản tại ({ox}, {oy})...")
        bench.min_scan_range = 99.0

        # Gửi mục tiêu bất đồng bộ
        goal_handle = bench.start_nav_goal_async(tx, ty, 0.0)

        # Chờ robot chạy được 2 giây
        t_wait = time.time()
        while time.time() - t_wait < 2.0:
            rclpy.spin_once(bench, timeout_sec=0.1)

        print(f"  >>> Bơm vật cản bất ngờ tại ({ox:.2f}, {oy:.2f})!")
        if bench.spawn_obs_cli and bench.spawn_obs_cli.wait_for_service(timeout_sec=2.0):
            req = SpawnObstacle.Request()
            req.x, req.y, req.z = float(ox), float(oy), 0.15
            req.sx, req.sy, req.sz = 0.3, 0.3, 0.3
            bench.spawn_obs_cli.call_async(req)

        # Theo dõi khoảng cách an toàn trong 8 giây tiếp theo
        min_dist_observed = 99.0
        start_monitor = time.time()
        while time.time() - start_monitor < 8.0:
            rclpy.spin_once(bench, timeout_sec=0.1)
            if bench.min_scan_range < min_dist_observed:
                min_dist_observed = bench.min_scan_range

        has_collision = (min_dist_observed < 0.05)
        if has_collision:
            collisions += 1
        print(f"  -> Khoảng cách nhỏ nhất quan sát được: {min_dist_observed:.3f} m | Va chạm: {'CÓ' if has_collision else 'KHÔNG (0 va chạm)'}")

        csv_rows.append({
            "trial": i + 1,
            "target": f"({tx}, {ty})",
            "obstacle_pos": f"({ox}, {oy})",
            "min_distance_m": round(min_dist_observed, 3),
            "collision": has_collision,
            "status": "PASS" if not has_collision else "FAIL",
        })

        if goal_handle is not None:
            goal_handle.cancel_goal_async()
            time.sleep(0.5)

        # Dọn vật cản sau thử nghiệm
        if bench.clear_obs_cli and bench.clear_obs_cli.wait_for_service(timeout_sec=2.0):
            bench.clear_obs_cli.call_async(Trigger.Request())
            time.sleep(0.5)

    csv_file = os.path.join(RESULTS_DIR, "nav_obstacles.csv")
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_rows[0].keys())
        writer.writeheader()
        writer.writerows(csv_rows)
    print(f"\n[OK] Đã ghi kết quả thử nghiệm vật cản vào: {csv_file}")
    print(f"Tổng số lần va chạm: {collisions} (Yêu cầu: 0 va chạm -> {'ĐẠT' if collisions == 0 else 'CHƯA ĐẠT'})")


def run_benchmark_estop(bench: NavBenchmarker):
    print("\n" + "=" * 65)
    print("  PHẦN 3: THỬ NGHIỆM KHÓA DỪNG KHẨN CẤP E-STOP (/estop)")
    print("=" * 65)

    # Đặt lệnh chạy thử
    target_x = 1.5 if (bench.current_pose and bench.current_pose[0] < 0.0) else -2.0
    goal_handle = bench.start_nav_goal_async(target_x, 0.0, 0.0)

    # Chờ robot bắt đầu chuyển động
    t_start = time.time()
    while time.time() - t_start < 6.0:
        rclpy.spin_once(bench, timeout_sec=0.05)
        if bench.last_cmd_vel is not None and bench.last_cmd_vel > 0.05:
            break

    print("  >>> Phát lệnh /estop = true giữa hành trình...")
    bench.estop_pub.publish(Bool(data=True))
    t_stop_send = time.time()

    # Theo dõi thời gian /cmd_vel về 0
    t_stopped = None
    while time.time() - t_stop_send < 2.0:
        rclpy.spin_once(bench, timeout_sec=0.01)
        if bench.last_cmd_vel is not None and bench.last_cmd_vel < 0.01:
            t_stopped = time.time()
            break

    response_time = (t_stopped - t_stop_send) if t_stopped else 2.0
    pass_estop = response_time < 0.30

    print(f"  -> Thời gian phản hồi dừng xe: {response_time*1000:.1f} ms (Yêu cầu < 300 ms -> {'ĐẠT' if pass_estop else 'CHƯA ĐẠT'})")

    if goal_handle is not None:
        goal_handle.cancel_goal_async()

    # Mở khóa E-stop
    time.sleep(0.5)
    print("  >>> Mở khóa /estop = false để khôi phục nhiệm vụ...")
    bench.estop_pub.publish(Bool(data=False))

    csv_file = os.path.join(RESULTS_DIR, "nav_estop.csv")
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["test", "response_time_ms", "threshold_ms", "result"])
        writer.writeheader()
        writer.writerow({
            "test": "estop_trigger",
            "response_time_ms": round(response_time * 1000, 1),
            "threshold_ms": 300.0,
            "result": "PASS" if pass_estop else "FAIL",
        })
    print(f"[OK] Đã ghi kết quả E-stop vào: {csv_file}")


def generate_plots(pos_errors, yaw_errors, trajectories):
    print("\n[VẼ BIỂU ĐỒ] Đang xuất đồ thị phân phối sai số và quỹ đạo di chuyển...")
    # 1. Histogram sai số vị trí và góc
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    if pos_errors:
        ax1.hist(pos_errors, bins=8, color="#1f77b4", edgecolor="black", alpha=0.8)
        ax1.set_title("Phân phối sai số vị trí (Position Error)")
        ax1.set_xlabel("Sai số (mm)")
        ax1.set_ylabel("Số lần lặp (Trials)")
        ax1.grid(True, linestyle="--", alpha=0.5)

    if yaw_errors:
        ax2.hist(yaw_errors, bins=8, color="#ff7f0e", edgecolor="black", alpha=0.8)
        ax2.set_title("Phân phối sai số hướng góc (Yaw Error)")
        ax2.set_xlabel("Sai số (độ)")
        ax2.set_ylabel("Số lần lặp (Trials)")
        ax2.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    hist_path = os.path.join(RESULTS_DIR, "nav_error_histogram.png")
    fig.savefig(hist_path, dpi=150)
    plt.close(fig)
    print(f"  [OK] Đã lưu biểu đồ sai số: {hist_path}")

    # 2. Quỹ đạo mặt bằng
    fig2, ax = plt.subplots(figsize=(8, 6))
    # Vẽ tường kho
    ax.plot([-3.0, 3.0, 3.0, -3.0, -3.0], [-2.0, -2.0, 2.0, 2.0, -2.0], "k-", lw=2, label="Tường bao")
    # Vẽ 2 giá kệ
    ax.fill([0.5, 2.0, 2.0, 0.5], [0.6, 0.6, 0.82, 0.82], color="gray", alpha=0.6, label="Kệ A")
    ax.fill([0.5, 2.0, 2.0, 0.5], [-0.82, -0.82, -0.6, -0.6], color="gray", alpha=0.6, label="Kệ B")
    # Trạm IN / OUT / HOME
    ax.plot(-2.4, 1.2, "gs", markersize=10, label="Bàn IN")
    ax.plot(-2.4, -1.2, "ms", markersize=10, label="Bàn OUT")
    ax.plot(-2.4, 0.0, "bs", markersize=10, label="HOME")

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    for idx, (name, traj) in enumerate(trajectories):
        if traj:
            xs, ys = zip(*traj)
            c = colors[idx % len(colors)]
            ax.plot(xs, ys, color=c, lw=1.8, label=f"Lộ trình {name}")

    ax.set_title("Mặt bằng kho & Quỹ đạo điều hướng Nav2 của Robot")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-2.5, 2.5)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(loc="upper right", fontsize=8)

    traj_path = os.path.join(RESULTS_DIR, "nav_trajectories.png")
    fig2.savefig(traj_path, dpi=150)
    plt.close(fig2)
    print(f"  [OK] Đã lưu bản đồ quỹ đạo: {traj_path}")


def main():
    parser = argparse.ArgumentParser(description="Nav2 Benchmark & Testing")
    parser.add_argument("--trials", type=int, default=20, help="Số lần thử nghiệm mục tiêu ngẫu nhiên (mặc định 20)")
    args = parser.parse_args()

    rclpy.init()
    bench = NavBenchmarker()

    pos_errors, yaw_errors, trajectories = run_benchmark_waypoints(bench, num_trials=args.trials)
    run_benchmark_obstacles(bench, num_trials=5)
    run_benchmark_estop(bench)
    generate_plots(pos_errors, yaw_errors, trajectories)

    bench.destroy_node()
    rclpy.shutdown()
    print("\n" + "=" * 65)
    print("  HOÀN THÀNH TOÀN BỘ NGHIỆM THU ĐIỀU HƯỚNG BƯỚC 4!")
    print("=" * 65)


if __name__ == "__main__":
    main()
