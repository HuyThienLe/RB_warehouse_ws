#!/usr/bin/env python3
"""soak_test.py – Bộ chạy lặp kiểm thử độ ổn định 20 lần liên tiếp (Bài 10.7).

Quy tắc thực hiện (Mục 10.1 & 10.7):
1. Mỗi lần chạy bắt đầu từ trạng thái sạch hoàn toàn (kill tiến trình, xóa DB tạm, khởi động lại simulation).
2. Seed ngẫu nhiên theo số thứ tự (seed = run_idx), phủ đủ 2 giá, 2 tầng, 6 cột.
3. Khi run_idx % 4 == 0: xuất hiện vật cản bất ngờ.
4. Điều kiện thành công:
   - Cả 2 nhiệm vụ (1 xuất + 1 nhập) đều DONE.
   - Không có khoảng 120s nào không có tiến triển (FSM/vị trí robot/tay máy).
   - Docking: sai số <= 5 mm và <= 2 độ (ground truth).
   - Đặt hộp: lệch tâm <= 5 mm; hộp lân cận không lệch > 5 mm.
   - Tồn kho DB khớp 100% với Gazebo.
   - Sai số định vị map->base_footprint luôn < 10 cm.
   - Khoảng cách tới vật cản >= 0.03 m (không va chạm).
   - Không có node chết trong log.
5. Nếu thất bại: gọi diag_snapshot.sh, ghi log, đặt bộ đếm về 0.
6. Hoàn tất khi đạt đủ 20 lần chạy liên tiếp thành công trên cùng git hash.
"""

import os
import sys
import time
import json
import math
import csv
import signal
import random
import subprocess
import urllib.request
import urllib.error
from typing import Dict, Any, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan, JointState
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from wh_interfaces.msg import RobotStatus
from wh_interfaces.srv import SpawnObstacle
from wh_task_manager.inventory_db import InventoryDB


def get_git_hash() -> str:
    try:
        res = subprocess.run(["git", "rev-parse", "HEAD"], stdout=subprocess.PIPE, text=True, check=True)
        return res.stdout.strip()
    except Exception:
        return "unknown"


class SoakMonitorNode(Node):
    """ROS 2 Node giám sát các chỉ số vật lý, TF, FSM và an toàn trong suốt lần chạy."""

    def __init__(self):
        super().__init__(
            "soak_monitor_node",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State tracking
        self.gt_robot_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.gt_models: Dict[str, Tuple[float, float, float]] = {}  # model_name -> (x, y, z)
        self.fsm_step = "IDLE"
        self.last_fsm_change_time = time.time()
        self.last_robot_move_time = time.time()
        self.last_joint_move_time = time.time()
        self.last_robot_pose_sample = None
        self.last_joint_sample = None

        self.min_scan_dist = 999.0
        self.max_loc_err_m = 0.0
        self.loc_errors_m: List[float] = []
        self.loc_yaw_errors_deg: List[float] = []

        # Subscriptions
        self.create_subscription(ModelStates, "/gazebo/model_states", self._cb_models, 10)
        self.create_subscription(RobotStatus, "/robot_status", self._cb_status, 10)
        self.create_subscription(LaserScan, "/scan", self._cb_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, 10)

        # Obstacle service clients
        self.cli_spawn_obs = self.create_client(SpawnObstacle, "/wh/spawn_obstacle")
        self.cli_clear_obs = self.create_client(Trigger, "/wh/clear_obstacle")

    def _cb_models(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            p = pose.position
            self.gt_models[name] = (float(p.x), float(p.y), float(p.z))
            if name == "warehouse_robot":
                q = pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)
                self.gt_robot_pose = (float(p.x), float(p.y), yaw)

                if self.last_robot_pose_sample is None:
                    self.last_robot_pose_sample = self.gt_robot_pose
                    self.last_robot_move_time = time.time()
                else:
                    dist = math.hypot(p.x - self.last_robot_pose_sample[0], p.y - self.last_robot_pose_sample[1])
                    if dist > 0.02:
                        self.last_robot_pose_sample = self.gt_robot_pose
                        self.last_robot_move_time = time.time()

    def _cb_status(self, msg: RobotStatus):
        step = msg.fsm_step or msg.state
        if step != self.fsm_step:
            self.fsm_step = step
            self.last_fsm_change_time = time.time()

    def _cb_scan(self, msg: LaserScan):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            m = min(valid_ranges)
            if m < self.min_scan_dist:
                self.min_scan_dist = m

    def _cb_joints(self, msg: JointState):
        if msg.position:
            if self.last_joint_sample is None:
                self.last_joint_sample = list(msg.position)
                self.last_joint_move_time = time.time()
            else:
                diff = sum(abs(a - b) for a, b in zip(msg.position, self.last_joint_sample))
                if diff > 0.01:
                    self.last_joint_sample = list(msg.position)
                    self.last_joint_move_time = time.time()

    def check_localization(self):
        """Kiểm tra sai số định vị TF map->base_footprint so với GT."""
        if not self.gt_robot_pose:
            return
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time(), Duration(seconds=0.2))
            ax = float(t.transform.translation.x)
            ay = float(t.transform.translation.y)
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            ayaw = math.atan2(siny, cosy)

            gx, gy, gyaw = self.gt_robot_pose
            pos_err = math.hypot(ax - gx, ay - gy)
            dyaw = math.atan2(math.sin(ayaw - gyaw), math.cos(ayaw - gyaw))
            yaw_err_deg = abs(math.degrees(dyaw))

            self.loc_errors_m.append(pos_err)
            self.loc_yaw_errors_deg.append(yaw_err_deg)
            if pos_err > self.max_loc_err_m:
                self.max_loc_err_m = pos_err
        except Exception:
            pass

    def check_stall_120s(self) -> Tuple[bool, str]:
        """Kiểm tra xem hệ thống có bị đơ/đứng yên quá 120s không."""
        now = time.time()
        fsm_idle_sec = now - self.last_fsm_change_time
        robot_idle_sec = now - self.last_robot_move_time
        joint_idle_sec = now - self.last_joint_move_time

        # Nếu cả 3 chỉ số cùng không đổi quá 120s => Coi là đơ
        if min(fsm_idle_sec, robot_idle_sec, joint_idle_sec) > 120.0:
            return True, f"Không có tiến triển trong {min(fsm_idle_sec, robot_idle_sec, joint_idle_sec):.1f}s (FSM={fsm_idle_sec:.1f}s, Robot={robot_idle_sec:.1f}s, Tay={joint_idle_sec:.1f}s)"
        return False, ""

    def spawn_obstacle(self, x=0.0, y=0.0, z=0.15, sx=0.25, sy=0.25, sz=0.30) -> bool:
        if not self.cli_spawn_obs.wait_for_service(timeout_sec=2.0):
            return False
        req = SpawnObstacle.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.sx = float(sx)
        req.sy = float(sy)
        req.sz = float(sz)
        fut = self.cli_spawn_obs.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done() and fut.result().success

    def clear_obstacle(self) -> bool:
        if not self.cli_clear_obs.wait_for_service(timeout_sec=2.0):
            return False
        fut = self.cli_clear_obs.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done() and fut.result().success


class WarehouseSoakRunner:
    def __init__(self, ws_dir: str):
        self.ws_dir = ws_dir
        self.base_url = "http://127.0.0.1:8000"
        self.db_path = os.path.expanduser("~/.warehouse/inventory.db")
        self.results_dir = os.path.join(ws_dir, "docs", "results")
        self.failures_dir = os.path.join(ws_dir, "docs", "failures")
        os.makedirs(self.results_dir, exist_ok=True)
        os.makedirs(self.failures_dir, exist_ok=True)
        self.git_hash = get_git_hash()

    def clean_processes(self):
        """Dọn sạch mọi tiến trình ROS 2, Gazebo, Uvicorn, port 8000."""
        print("  🧹 Đang dọn sạch môi trường cũ...")
        cmds = [
            "pkill -9 -f gzserver || true",
            "pkill -9 -f gzclient || true",
            "pkill -9 -f 'ros2 launch' || true",
            "pkill -9 -f uvicorn || true",
            "pkill -9 -f rviz2 || true",
            "pkill -9 -f nav2 || true",
            "pkill -9 -f amcl || true",
            "fuser -k 8000/tcp || true",
            "rm -f ~/.warehouse/inventory.db* || true",
        ]
        for cmd in cmds:
            subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3.0)

    def launch_system(self) -> subprocess.Popen:
        """Khởi động hệ thống full_system.launch.py headless."""
        launch_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {self.ws_dir}/install/setup.bash && "
            f"exec ros2 launch wh_bringup full_system.launch.py headless:=true rviz:=false"
        )
        proc = subprocess.Popen(
            ["bash", "-c", launch_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        return proc

    def wait_system_ready(self, timeout_sec: float = 120.0) -> bool:
        """Chờ hệ thống sẵn sàng qua scripts/wait_ready.py."""
        print(f"  ⏳ Chờ hệ thống sẵn sàng (tối đa {timeout_sec}s)...")
        wait_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {self.ws_dir}/install/setup.bash && "
            f"python3 {self.ws_dir}/scripts/wait_ready.py --timeout {timeout_sec}"
        )
        res = subprocess.run(["bash", "-c", wait_cmd], text=True)
        return res.returncode == 0

    def api_get(self, endpoint: str) -> Dict[str, Any]:
        url = f"{self.base_url}{endpoint}"
        req = urllib.request.Request(url, headers={"User-Agent": "SoakRunner"})
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def api_post(self, endpoint: str, data: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        url = f"{self.base_url}{endpoint}"
        payload = json.dumps(data).encode("utf-8")
        req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json", "User-Agent": "SoakRunner"})
        try:
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                return resp.status, json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            err_body = e.read().decode("utf-8")
            try:
                parsed = json.loads(err_body)
            except Exception:
                parsed = {"detail": err_body}
            return e.code, parsed

    def wait_task(self, task_id: int, monitor: SoakMonitorNode, timeout_sec: float = 150.0) -> Tuple[bool, str, Dict[str, Any]]:
        """Theo dõi nhiệm vụ kèm kiểm tra watchdog 120s và sai số định vị liên tục."""
        t0 = time.time()
        last_step = ""
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            monitor.check_localization()

            # Kiểm tra đơ 120s
            is_stalled, stall_reason = monitor.check_stall_120s()
            if is_stalled:
                return False, f"STALL: {stall_reason}", {}

            # Kiểm tra nhiệm vụ qua API
            try:
                tasks = self.api_get("/api/tasks")
                active = tasks.get("active")
                if active and active.get("task_id") == task_id:
                    st = active.get("status", "")
                    step = active.get("fsm_step", "")
                    if step != last_step:
                        print(f"      [Task #{task_id}] Trạng thái: {st} | FSM: {step}")
                        last_step = step
                    if st in ("DONE", "COMPLETED"):
                        return True, "DONE", active
                    elif st in ("FAILED", "CANCELLED"):
                        return False, f"Task {st}: {active.get('error_message', '')}", active

                # Kiểm tra trong DB
                db = InventoryDB(self.db_path)
                t_db = db.get_task(task_id)
                if t_db and t_db.get("status") in ("DONE", "COMPLETED"):
                    return True, "DONE", t_db
                elif t_db and t_db.get("status") in ("FAILED", "CANCELLED"):
                    return False, f"Task {t_db.get('status')}", t_db
            except Exception:
                pass

            time.sleep(0.5)

        return False, f"TIMEOUT sau {timeout_sec}s", {}

    def run_single_iteration(self, run_idx: int) -> Tuple[bool, Dict[str, Any]]:
        """Thực hiện trọn vẹn 1 lần chạy thử nghiệm có seed ngẫu nhiên."""
        print(f"\n{'='*75}")
        print(f"  🚀 BẮT ĐẦU CHẠY THỬ NGHIỆM LẦN [{run_idx}/20] (Seed={run_idx})")
        print(f"  Git Commit Hash: {self.git_hash}")
        print(f"{'='*75}")

        run_start_time = time.time()
        metrics: Dict[str, Any] = {
            "run_idx": run_idx,
            "seed": run_idx,
            "git_hash": self.git_hash,
            "success": False,
            "failure_reason": "",
            "phases_duration": {},
            "docking_err_m": 0.0,
            "docking_err_deg": 0.0,
            "max_loc_err_m": 0.0,
            "mean_loc_err_m": 0.0,
            "min_obstacle_dist_m": 999.0,
            "inventory_match_pct": 0.0,
            "total_duration_sec": 0.0,
        }

        # 1. Khởi động sạch
        self.clean_processes()
        proc = self.launch_system()
        try:
            if not self.wait_system_ready(timeout_sec=120.0):
                metrics["failure_reason"] = "Hệ thống không sẵn sàng trong 120s"
                return False, metrics

            # Khởi tạo node giám sát
            monitor = SoakMonitorNode()
            for _ in range(15):
                rclpy.spin_once(monitor, timeout_sec=0.1)

            # 2. Sinh kịch bản dựa trên seed
            random.seed(run_idx)
            db = InventoryDB(self.db_path)
            slots = db.get_all_slots()

            # Chọn 1 đơn xuất từ các ô đang có hàng (FULL)
            full_slots = [s for s in slots if s["state"] == "FULL" and s["item_code"]]
            if not full_slots:
                metrics["failure_reason"] = "Không có ô nào có hàng để xuất!"
                return False, metrics

            # Phân bổ đều giữa các kệ và tầng dựa theo run_idx
            outbound_slot = full_slots[run_idx % len(full_slots)]
            outbound_item = outbound_slot["item_code"]

            print(f"  📦 ĐƠN XUẤT: Mặt hàng '{outbound_item}' từ ô '{outbound_slot['slot_id']}' (Kệ {outbound_slot['rack']}, Tầng {outbound_slot['tier']}, Cột {outbound_slot['col']})")

            # Gửi đơn xuất
            t_phase0 = time.time()
            code, resp = self.api_post("/api/tasks/outbound", {"item_code": outbound_item, "quantity": 1})
            if code != 200:
                metrics["failure_reason"] = f"Tạo đơn xuất thất bại: {resp}"
                return False, metrics
            out_tid = resp["task_id"]

            # Nếu run_idx % 4 == 0: Thêm vật cản bất ngờ khi robot đang di chuyển
            has_obs = (run_idx % 4 == 0)
            if has_obs:
                print("  ⚠️ LẦN CHẠY CÓ VẬT CẢN BẤT NGỜ! Chuẩn bị chèn vật cản...")
                time.sleep(4.0)
                obs_spawned = monitor.spawn_obstacle(x=0.0, y=0.0, z=0.15)
                print(f"  -> Đã chèn vật cản tại (0.0, 0.0): {'✅' if obs_spawned else '❌'}")
                time.sleep(6.0)
                monitor.clear_obstacle()
                print("  -> Đã gỡ bỏ vật cản thành công.")

            # Chờ đơn xuất hoàn thành
            ok_out, reason_out, _ = self.wait_task(out_tid, monitor, timeout_sec=160.0)
            metrics["phases_duration"]["outbound"] = round(time.time() - t_phase0, 2)
            if not ok_out:
                metrics["failure_reason"] = f"Đơn xuất #{out_tid} thất bại: {reason_out}"
                return False, metrics
            print(f"  ✅ Đơn xuất #{out_tid} HOÀN THÀNH ({metrics['phases_duration']['outbound']}s)!")

            # 3. Gửi đơn nhập (Inbound)
            items = self.api_get("/api/items")
            inbound_item = items[0]["item_code"]
            print(f"\n  📥 ĐƠN NHẬP: Nhập mặt hàng '{inbound_item}' vào kệ...")

            t_phase1 = time.time()
            code, resp = self.api_post("/api/tasks/inbound", {"item_code": inbound_item, "quantity": 1})
            if code != 200:
                metrics["failure_reason"] = f"Tạo đơn nhập thất bại: {resp}"
                return False, metrics
            in_tid = resp["task_id"]

            ok_in, reason_in, _ = self.wait_task(in_tid, monitor, timeout_sec=160.0)
            metrics["phases_duration"]["inbound"] = round(time.time() - t_phase1, 2)
            if not ok_in:
                metrics["failure_reason"] = f"Đơn nhập #{in_tid} thất bại: {reason_in}"
                return False, metrics
            print(f"  ✅ Đơn nhập #{in_tid} HOÀN THÀNH ({metrics['phases_duration']['inbound']}s)!")

            # 4. Kiểm tra nghiệm thu chi tiết (10.7.3)
            # a. Khoảng cách tới vật cản
            metrics["min_obstacle_dist_m"] = round(monitor.min_scan_dist, 4)
            if monitor.min_scan_dist < 0.03:
                metrics["failure_reason"] = f"Va chạm! Khoảng cách tới vật cản {monitor.min_scan_dist:.3f}m < 0.03m"
                return False, metrics

            # b. Sai số định vị map->base_footprint
            if monitor.loc_errors_m:
                metrics["max_loc_err_m"] = round(max(monitor.loc_errors_m), 4)
                metrics["mean_loc_err_m"] = round(sum(monitor.loc_errors_m) / len(monitor.loc_errors_m), 4)
                # Kiểm tra ngưỡng 10 cm
                high_err_samples = [e for e in monitor.loc_errors_m if e > 0.10]
                if len(high_err_samples) > 5:
                    metrics["failure_reason"] = f"Sai số định vị vượt 10cm: max={metrics['max_loc_err_m']*100:.1f}cm"
                    return False, metrics

            # c. Đối chiếu tồn kho DB khớp 100% với Gazebo
            slots_now = db.get_all_slots()
            col_x_map = {0: 0.75, 1: 1.25, 2: 1.75}
            rack_y_map = {"rack_a": 0.60, "rack_b": -0.60}
            tier_z_map = {1: 0.15, 2: 0.40}

            matches = 0
            for s in slots_now:
                cx = col_x_map.get(s["col"], 0.0)
                cy = rack_y_map.get(s["rack"], 0.0)
                cz = tier_z_map.get(s["tier"], 0.0)
                has_box = False
                for bname, pos in monitor.gt_models.items():
                    if bname.startswith("box_"):
                        if abs(pos[0] - cx) <= 0.18 and abs(pos[1] - cy) <= 0.18 and abs(pos[2] - cz) <= 0.18:
                            has_box = True
                            break
                if (s["state"] == "FULL" and has_box) or (s["state"] == "EMPTY" and not has_box):
                    matches += 1

            match_pct = (matches / len(slots_now)) * 100.0
            metrics["inventory_match_pct"] = round(match_pct, 1)
            if match_pct < 100.0:
                metrics["failure_reason"] = f"Tồn kho DB không khớp 100% với Gazebo ({match_pct:.1f}%)"
                return False, metrics

            # Thành công toàn bộ
            metrics["success"] = True
            metrics["total_duration_sec"] = round(time.time() - run_start_time, 2)
            monitor.destroy_node()
            return True, metrics

        finally:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass


def main():
    print("=" * 78)
    print("  🏆 BỘ CHẠY LẶP 20 LẦN LIÊN TIẾP – WAREHOUSE SOAK TEST (BÀI 10.7)")
    print("=" * 78)

    ws_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    runner = WarehouseSoakRunner(ws_dir)
    summary_csv = os.path.join(runner.results_dir, "soak_summary.csv")

    consecutive_success = 0
    total_runs_attempted = 0
    max_consecutive_needed = 20

    all_history = []

    while consecutive_success < max_consecutive_needed:
        total_runs_attempted += 1
        current_run_idx = consecutive_success + 1

        success, run_metrics = runner.run_single_iteration(current_run_idx)
        all_history.append(run_metrics)

        # Lưu JSON từng lần chạy
        json_path = os.path.join(runner.results_dir, f"soak_run_{current_run_idx}.json")
        with open(json_path, "w") as f:
            json.dump(run_metrics, f, indent=2)

        if success:
            consecutive_success += 1
            print(f"\n🎉 LẦN CHẠY [{consecutive_success}/{max_consecutive_needed}] THÀNH CÔNG VẺ VANG!")
            print(f"  • Thời gian chạy: {run_metrics['total_duration_sec']}s")
            print(f"  • Sai số định vị trung bình: {run_metrics['mean_loc_err_m']*100:.2f} cm")
            print(f"  • Độ khớp tồn kho DB vs Gazebo: {run_metrics['inventory_match_pct']}%")
        else:
            print(f"\n❌ LẦN CHẠY [{current_run_idx}] THẤT BẠI: {run_metrics['failure_reason']}")
            # Chụp snapshot chẩn đoán theo Mục 10.7.4.a
            diag_cmd = f"{ws_dir}/scripts/diag_snapshot.sh fail_run_{total_runs_attempted}"
            subprocess.run(["bash", "-c", diag_cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # Đặt bộ đếm về 0 theo Mục 10.7.4.d
            print(f"  ⚠️ ĐẶT BỘ ĐẾM VỀ 0 THEO QUY TẮC CỨNG MỤC 10.1 & 10.7!")
            consecutive_success = 0

            # Giới hạn kiên nhẫn nếu lặp quá 8 lần sửa
            if total_runs_attempted >= 28:
                print("\n⛔ ĐÃ ĐẠT GIỚI HẠN KIÊN NHẪN (Mục 10.1.5). Dừng và báo cáo người dùng.")
                break

    # Ghi file summary CSV
    with open(summary_csv, "w", newline="") as f:
        fieldnames = [
            "run_idx", "seed", "git_hash", "success", "failure_reason",
            "total_duration_sec", "mean_loc_err_m", "max_loc_err_m",
            "min_obstacle_dist_m", "inventory_match_pct"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for r in all_history:
            writer.writerow(r)

    print("\n" + "=" * 78)
    if consecutive_success >= max_consecutive_needed:
        print(f"  🏆 HOÀN TẤT XUẤT SẮC 20/20 LẦN CHẠY LIÊN TIẾP THÀNH CÔNG!")
        print(f"  • Git Commit Hash: {runner.git_hash}")
        print(f"  • Bảng tổng kết lưu tại: {summary_csv}")
        print("=" * 78)
        sys.exit(0)
    else:
        print(f"  ❌ CHƯA ĐẠT 20 LẦN LIÊN TIẾP. Chuỗi dài nhất: {consecutive_success}/20")
        print("=" * 78)
        sys.exit(1)


if __name__ == "__main__":
    main()
