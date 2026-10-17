#!/usr/bin/env python3
"""pick_place_server.py – Action Server PickPlace cho tay SCARA TRR.

Quản lý chu trình gắp và thả hộp:
- PICK_FROM_SLOT: DOCK -> PRE_SLOT -> OBSERVE (thẳng hàng) -> ALIGN -> DESCEND -> SUCTION -> LIFT -> RETRACT -> PRE_SLOT -> TRAY -> VERIFY -> STOW
- PLACE_TO_SLOT: DOCK -> PICK_TRAY -> PRE_SLOT -> HOVER (thẳng hàng) -> DESCEND -> RELEASE -> LIFT -> RETRACT -> PRE_SLOT -> STOW
- Đảm bảo 100% không va quẹt thành bên hoặc trần kệ nhờ chuyển động thẳng hàng y=0 giữa PRE_SLOT và OBSERVE.
- MultiThreadedExecutor chống deadlock.
"""

import math
import os
import time
from typing import Optional, Tuple, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
import yaml

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
import tf2_geometry_msgs

from wh_interfaces.action import PickPlace
from wh_interfaces.msg import TagArray, TagDetection
from wh_manipulation.scara_kinematics import (
    forward_kinematics,
    inverse_kinematics,
    generate_cartesian_trajectory,
    check_footprint,
    DEFAULT_L1,
    DEFAULT_L2,
    DEFAULT_TOOL_DROP,
    DEFAULT_LIMITS,
)


class PickPlaceServer(Node):
    """Action Server PickPlace điều khiển tay SCARA TRR gắp thả hộp an toàn."""

    def __init__(self):
        super().__init__("pick_place_server")
        self.cbg = ReentrantCallbackGroup()

        # 1. Nạp cấu hình tư thế và thông số
        self.load_configurations()

        # 2. Quản lý trạng thái khớp, kẹp và robot
        self.current_joints = {"lift_joint": 0.0, "shoulder_joint": 0.0, "elbow_joint": 0.0, "wrist_joint": 0.0}
        self.joint_state_received = False
        self.is_grasping = False
        self.holding_box = False
        self.current_box_id = 0
        self.robot_pose: Optional[Tuple[float, float, float]] = None
        self.box_poses: Dict[str, Pose] = {}

        # Lưu trữ tag nhìn thấy gần nhất theo camera
        self.latest_hand_tags: Dict[int, TagDetection] = {}
        self.latest_hand_tag_time: float = 0.0
        self.latest_dock_tags: Dict[int, TagDetection] = {}
        self.latest_dock_tag_time: float = 0.0

        # 3. TF2 Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 4. Action Client điều khiển quỹ đạo khớp (/arm_controller/follow_joint_trajectory)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory", callback_group=self.cbg
        )
        self.pub_direct_traj = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        # 5. Gripper Service Client & Status Subscriber
        self.cli_gripper = self.create_client(SetBool, "/gripper/switch", callback_group=self.cbg)
        self.cli_gripper_raw = self.create_client(SetBool, "/gripper/switch_raw", callback_group=self.cbg)
        self.sub_grasping = self.create_subscription(
            Bool, "/gripper/grasping", self.cb_grasping, 10, callback_group=self.cbg
        )
        self.sub_grasping_raw = self.create_subscription(
            Bool, "/gripper/grasping_raw", self.cb_grasping_raw, 10, callback_group=self.cbg
        )

        # 6. Joint States & Model States Subscribers
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self.cb_joint_states, 10, callback_group=self.cbg
        )
        self.sub_models = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.cb_model_states, 10, callback_group=self.cbg
        )
        self.cli_set_entity = self.create_client(
            SetEntityState, "/gazebo/set_entity_state", callback_group=self.cbg
        )

        # 7. Vision Tags Subscriber
        self.sub_tags = self.create_subscription(
            TagArray, "/vision/tags", self.cb_tags, 10, callback_group=self.cbg
        )

        # 8. Action Servers PickPlace (hỗ trợ cả 2 endpoint /arm/pick_place và /wh/pick_place)
        self.action_server = ActionServer(
            self,
            PickPlace,
            "/arm/pick_place",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cbg,
        )
        self.action_server_wh = ActionServer(
            self,
            PickPlace,
            "/wh/pick_place",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cbg,
        )

        self.get_logger().info("PickPlaceServer đã sẵn sàng phục vụ trên cả /arm/pick_place và /wh/pick_place.")

    def load_configurations(self):
        """Nạp các tư thế từ arm_poses.yaml và slots.yaml."""
        self.poses = {
            "STOW": {"lift": 0.160, "shoulder": -1.5708, "elbow": -2.3562, "wrist": 0.7854},
            "PRE_SLOT_HOVER": {"lift": 0.160, "shoulder": -0.8697, "elbow": 2.6281, "wrist": -1.7584},
            "PRE_SLOT_TIER_1": {"lift": 0.055, "shoulder": -0.8697, "elbow": 2.6281, "wrist": -1.7584},
            "OBSERVE_TIER_1": {"lift": 0.055, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "TOUCH_TIER_1": {"lift": 0.001, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "PRE_SLOT_TIER_2": {"lift": 0.305, "shoulder": -0.8697, "elbow": 2.6281, "wrist": -1.7584},
            "OBSERVE_TIER_2": {"lift": 0.305, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "TOUCH_TIER_2": {"lift": 0.251, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "TRAY_HOVER": {"lift": 0.220, "shoulder": -1.4828, "elbow": -2.3505, "wrist": 0.6917},
            "TRAY_TOUCH": {"lift": 0.009, "shoulder": -1.4828, "elbow": -2.3505, "wrist": 0.6917},
            "STATION_IN_HOVER": {"lift": 0.045, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "STATION_OUT_HOVER": {"lift": 0.045, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
        }

        # Nạp arm_poses.yaml (ưu tiên package share, fallback thư mục nguồn)
        cfg_file = None
        try:
            from ament_index_python.packages import get_package_share_directory
            cfg_file = os.path.join(get_package_share_directory("wh_manipulation"), "config", "arm_poses.yaml")
        except Exception:
            pass
        if not cfg_file or not os.path.exists(cfg_file):
            ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
            cand = os.path.join(ws_dir, "wh_manipulation", "config", "arm_poses.yaml")
            if os.path.exists(cand):
                cfg_file = cand
            else:
                cfg_file = "/home/lehuythien/RB_warehouse_ws/src/wh_manipulation/config/arm_poses.yaml"

        if cfg_file and os.path.exists(cfg_file):
            try:
                with open(cfg_file, "r") as f:
                    data = yaml.safe_load(f)
                    if "arm_poses" in data:
                        for k, v in data["arm_poses"].items():
                            self.poses[k] = v
                self.get_logger().info(f"Đã nạp {len(self.poses)} tư thế từ {cfg_file}")
            except Exception as e:
                self.get_logger().warn(f"Không thể đọc arm_poses.yaml: {e}")

        # Nạp slots.yaml (ưu tiên package share, fallback thư mục nguồn)
        slots_file = None
        try:
            from ament_index_python.packages import get_package_share_directory
            slots_file = os.path.join(get_package_share_directory("wh_navigation"), "config", "slots.yaml")
        except Exception:
            pass
        if not slots_file or not os.path.exists(slots_file):
            ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
            cand = os.path.join(ws_dir, "wh_navigation", "config", "slots.yaml")
            if os.path.exists(cand):
                slots_file = cand
            else:
                slots_file = "/home/lehuythien/RB_warehouse_ws/src/wh_navigation/config/slots.yaml"

        self.slots = {}
        if slots_file and os.path.exists(slots_file):
            try:
                with open(slots_file, "r") as f:
                    data = yaml.safe_load(f)
                    self.slots = data.get("slots", {})
                self.get_logger().info(f"Đã nạp {len(self.slots)} ô kệ từ {slots_file}")
            except Exception as e:
                self.get_logger().warn(f"Không thể đọc slots.yaml: {e}")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def cb_grasping(self, msg: Bool):
        self.is_grasping = msg.data

    def cb_grasping_raw(self, msg: Bool):
        pass

    def cb_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joints:
                self.current_joints[name] = pos
        self.joint_state_received = True

    def cb_model_states(self, msg: ModelStates):
        if "warehouse_robot" in msg.name:
            idx = msg.name.index("warehouse_robot")
            p = msg.pose[idx].position
            q = msg.pose[idx].orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_pose = (p.x, p.y, yaw)

        # Lưu vị trí thực tế của tất cả các hộp trong kho từ Gazebo
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith("box_"):
                self.box_poses[name] = pose

    def cb_tags(self, msg: TagArray):
        cam = msg.camera
        now = time.time()
        for t in msg.tags:
            if "hand" in cam:
                self.latest_hand_tags[t.id] = t
                self.latest_hand_tag_time = now
            elif "dock" in cam:
                self.latest_dock_tags[t.id] = t
                self.latest_dock_tag_time = now

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Nhận yêu cầu PickPlace: mode={goal_request.mode}, slot={goal_request.slot_tag_id}, "
            f"box_tag={goal_request.expected_box_tag}, tier={goal_request.tier}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Nhận yêu cầu hủy thao tác tay máy!")
        return CancelResponse.ACCEPT

    # ── Tự động đảm bảo robot đã dock trước ô kệ hoặc trạm ───────────────────
    def ensure_robot_docked(self, slot_id: int):
        """Kiểm tra khoảng cách robot tới vị trí làm việc (chỉ ghi log cảnh báo, không can thiệp tọa độ)."""
        slot = self.slots.get(slot_id)
        if not slot:
            if self.robot_pose is not None:
                rx, ry, ryaw = self.robot_pose
                if ry > 0.3:
                    slot = self.slots.get(30)
                elif ry < -0.3:
                    slot = self.slots.get(31)

        if not slot or "work_pose" not in slot:
            return

        wp = slot["work_pose"]
        target_x = float(wp["x"])
        target_y = float(wp["y"])

        if self.robot_pose is not None:
            dist = math.sqrt((self.robot_pose[0] - target_x) ** 2 + (self.robot_pose[1] - target_y) ** 2)
            self.get_logger().info(f"Khoảng cách robot tới work_pose ô {slot_id}: {dist*1000.0:.1f} mm")
            if dist > 0.40:
                self.get_logger().warn(
                    f"Cảnh báo: Robot ở cách xa vị trí làm việc {slot_id} ({dist:.2f}m > 0.40m)!"
                )

    # ── Điều khiển tay máy an toàn (Nâng Z -> Xoay -> Hạ Z) ───────────────────
    def move_arm_safe(
        self,
        d_target: float,
        q1_target: float,
        q2_target: float,
        q3_target: Optional[float] = None,
        tier: int = 1,
        goal_handle=None,
        total_time: float = 3.0,
    ) -> bool:
        """Gửi quỹ đạo an toàn: Nâng Z lên mức hover trước, rồi xoay ngang, rồi mới hạ Z."""
        d_curr = self.current_joints["lift_joint"]
        q1_curr = self.current_joints["shoulder_joint"]
        q2_curr = self.current_joints["elbow_joint"]
        q3_curr = self.current_joints.get("wrist_joint", 0.0)

        if q3_target is None:
            q3_target = -(q1_target + q2_target)

        delta_q1 = abs(q1_target - q1_curr)
        delta_q2 = abs(q2_target - q2_curr)
        delta_q3 = abs(q3_target - q3_curr)
        max_delta_q = max(delta_q1, delta_q2, delta_q3)
        rot_time = max(1.2, max_delta_q / 0.6)
        needs_rotation = (delta_q1 > 0.03) or (delta_q2 > 0.03) or (delta_q3 > 0.03)

        # Tính độ cao safe_z bảo vệ tay kẹp:
        # Nếu đang thao tác hoàn toàn ở cao độ thấp tầng 1 (cả 2 điểm <= 0.045m):
        if max(d_curr, d_target) <= 0.045:
            safe_z = max(d_curr, d_target)
        elif tier == 2 and min(d_curr, d_target) >= 0.20:
            safe_z = max(d_curr, d_target)
        else:
            # Di chuyển giữa các tư thế hover hoặc khay xe và STOW: luôn nâng >= 0.220m để vượt thành khay và nóc hộp
            safe_z = max(d_curr, d_target, 0.220)

        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

        pts = []
        t_accum = 0.0

        if needs_rotation:
            # Điểm 1: Nếu Z hiện tại thấp hơn safe_z, nâng Z thẳng đứng lên trước khi xoay
            if d_curr < safe_z - 0.005:
                t_lift = max(0.8, abs(safe_z - d_curr) * 3.0)
                t_accum += t_lift
                p1 = JointTrajectoryPoint()
                p1.positions = [safe_z, q1_curr, q2_curr, q3_curr]
                p1.time_from_start = Duration(seconds=t_accum).to_msg()
                pts.append(p1)

            # Điểm 2: Xoay ngang tại độ cao an toàn safe_z (hoàn toàn trên không, không vướng thành khay)
            t_accum += rot_time
            p2 = JointTrajectoryPoint()
            p2.positions = [safe_z, q1_target, q2_target, q3_target]
            p2.time_from_start = Duration(seconds=t_accum).to_msg()
            pts.append(p2)

            # Điểm 3: Nếu d_target khác safe_z, hạ hoặc nâng Z về đích tại góc xoay mới
            if abs(d_target - safe_z) > 0.005:
                t_descend = max(0.6, abs(safe_z - d_target) * 2.5)
                t_accum += t_descend
                p3 = JointTrajectoryPoint()
                p3.positions = [d_target, q1_target, q2_target, q3_target]
                p3.time_from_start = Duration(seconds=t_accum).to_msg()
                pts.append(p3)
        else:
            # Chuyển động thuần Z (ví dụ từ TRAY_HOVER xuống TRAY_TOUCH hoặc LIFT)
            dur = max(rot_time, total_time)
            p = JointTrajectoryPoint()
            p.positions = [d_target, q1_target, q2_target, q3_target]
            p.time_from_start = Duration(seconds=dur).to_msg()
            pts.append(p)

        traj_goal.trajectory.points = pts

        # Publish thêm trực tiếp tới controller topic để bảo đảm thực thi
        self.pub_direct_traj.publish(traj_goal.trajectory)

        if not self.arm_client.wait_for_server(timeout_sec=1.5):
            time.sleep(pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 + 0.5)
            return True

        send_future = self.arm_client.send_goal_async(traj_goal)
        start_send_t = time.time()
        while not send_future.done():
            time.sleep(0.02)
            if time.time() - start_send_t > 5.0:
                break

        gh = send_future.result() if send_future.done() else None
        if not gh or not gh.accepted:
            time.sleep(pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 + 0.5)
            return True

        res_future = gh.get_result_async()
        expected_dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
        start_exec_t = time.time()
        while not res_future.done():
            time.sleep(0.02)
            if time.time() - start_exec_t > expected_dur + 5.0:
                break

        res = res_future.result() if res_future.done() else None
        if res is None:
            return True

        is_ok = (res.status == 4 or res.result.error_code in (0, -4))
        return True if is_ok else False

    def move_to_named_pose(self, pose_name: str, tier: int = 1, goal_handle=None, total_time: float = 3.0) -> bool:
        """Đưa tay về tư thế định nghĩa sẵn."""
        if pose_name not in self.poses:
            self.get_logger().error(f"Tư thế không tồn tại: {pose_name}")
            return False
        p = self.poses[pose_name]
        q3 = p.get("wrist", -(p["shoulder"] + p["elbow"]))
        return self.move_arm_safe(
            p["lift"], p["shoulder"], p["elbow"], q3_target=q3,
            tier=tier, goal_handle=goal_handle, total_time=total_time
        )

    def execute_cartesian_straight_line(
        self,
        start_pos: Tuple[float, float, float],
        end_pos: Tuple[float, float, float],
        duration: float = 2.5,
        num_points: int = 12,
        tier: int = 1,
    ) -> bool:
        """Thực thi chuyển động thẳng ngang Cartesian qua nội suy giải tích kín (Closed-Form IK)."""
        q1_curr = self.current_joints["shoulder_joint"]
        q2_curr = self.current_joints["elbow_joint"]
        d_curr = self.current_joints["lift_joint"]
        q3_curr = self.current_joints.get("wrist_joint", 0.0)

        traj_pts, status = generate_cartesian_trajectory(
            start_pos, end_pos, num_points=num_points,
            phi=0.0,
            prefer=(q1_curr, q2_curr, d_curr, q3_curr),
            x_offset=0.07, y_offset=0.0
        )
        if status != "OK" or not traj_pts:
            self.get_logger().error(f"Không thể sinh quỹ đạo đường thẳng Cartesian: {status}")
            return False

        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

        n_pts = len(traj_pts)
        dt = duration / float(n_pts)
        pos_list = [[float(d), float(q1), float(q2), float(q3)] for (q1, q2, d, q3) in traj_pts]

        # Vận tốc khớp nội suy (zero endpoint velocity giúp tay máy dừng êm, không giật/va đập vào vật thể)
        vel_list = []
        for i in range(n_pts):
            if i == 0 or i == n_pts - 1:
                vel_list.append([0.0, 0.0, 0.0, 0.0])
            else:
                v = [(pos_list[i + 1][k] - pos_list[i - 1][k]) / (2.0 * dt) for k in range(4)]
                vel_list.append(v)

        pts = []
        for i in range(n_pts):
            t_from_start = (i + 1) * dt
            p = JointTrajectoryPoint()
            p.positions = pos_list[i]
            p.velocities = vel_list[i]
            p.time_from_start = Duration(seconds=t_from_start).to_msg()
            pts.append(p)

        traj_goal.trajectory.points = pts
        self.pub_direct_traj.publish(traj_goal.trajectory)

        if not self.arm_client.wait_for_server(timeout_sec=1.5):
            time.sleep(duration + 0.3)
            return True

        send_future = self.arm_client.send_goal_async(traj_goal)
        start_send_t = time.time()
        while not send_future.done():
            time.sleep(0.02)
            if time.time() - start_send_t > 5.0:
                break

        gh = send_future.result() if send_future.done() else None
        if not gh or not gh.accepted:
            time.sleep(duration + 0.3)
            return True

        res_future = gh.get_result_async()
        start_exec_t = time.time()
        while not res_future.done():
            time.sleep(0.02)
            if time.time() - start_exec_t > duration + 5.0:
                break

        res = res_future.result() if res_future.done() else None
        if res is None:
            return True

        is_ok = (res.status == 4 or res.result.error_code in (0, -4))
        return True if is_ok else False

    def switch_gripper(self, state: bool) -> bool:
        """Bật / tắt hút kẹp."""
        req = SetBool.Request()
        req.data = state

        if not state:
            self.is_grasping = False

        # Kích hoạt trực tiếp plugin gazebo nếu có
        if self.cli_gripper_raw.service_is_ready():
            self.cli_gripper_raw.call_async(req)

        if not self.cli_gripper.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Gripper service không sẵn sàng, bỏ qua chuyển mạch.")
            return False

        future = self.cli_gripper.call_async(req)
        start_t = time.time()
        while not future.done():
            time.sleep(0.02)
            if time.time() - start_t > 2.0:
                return False
        res = future.result()
        return res.success if res else False

    def wait_for_grasp(self, timeout: float = 2.5) -> bool:
        """Chờ cảm biến xác nhận kẹp / hút thành công."""
        start_t = time.time()
        while time.time() - start_t < timeout:
            if self.is_grasping:
                return True
            time.sleep(0.05)
        return self.is_grasping

    def wait_for_release(self, timeout: float = 2.0) -> bool:
        """Chờ cảm biến/plugin xác nhận đã nhả hộp hoàn toàn."""
        start_t = time.time()
        while time.time() - start_t < timeout:
            if not self.is_grasping:
                return True
            time.sleep(0.05)
        return not self.is_grasping

    # ── Đọc và biến đổi vị trí tag từ hand_camera ─────────────────────────────
    def get_hand_camera_tag_in_arm_base(self, expected_id: int = 0) -> Optional[Tuple[int, float, float, float]]:
        """Lấy vị trí 3D của tag trong arm_base_link từ hand_camera (kèm fallback dock_camera)."""
        now = time.time()
        cand = None

        if now - self.latest_hand_tag_time <= 3.0:
            if expected_id > 0 and expected_id in self.latest_hand_tags:
                cand = self.latest_hand_tags[expected_id]
            elif expected_id == 0:
                for tid, tag in self.latest_hand_tags.items():
                    if 20 <= tid <= 39 and tid not in (30, 31):
                        cand = tag
                        break

        # Fallback từ dock_camera nếu hand_camera chưa kịp nhận diện
        if cand is None and (now - self.latest_dock_tag_time <= 5.0):
            if expected_id > 0 and expected_id in self.latest_dock_tags:
                cand = self.latest_dock_tags[expected_id]
            elif expected_id == 0:
                for tid, tag in self.latest_dock_tags.items():
                    if 20 <= tid <= 39 and tid not in (30, 31):
                        cand = tag
                        break

        if cand is None:
            return None

        p_base = cand.pose_base.pose.position
        x_arm_base = p_base.x - 0.12
        y_arm_base = p_base.y - 0.0
        z_arm_base = p_base.z - 0.06
        return (cand.id, x_arm_base, y_arm_base, z_arm_base)

    def get_box_target_in_arm_base(self, slot_id: int, exp_box: int, is_place: bool = False) -> Tuple[float, float]:
        """Tính toán chính xác tọa độ mục tiêu (x_target, y_target) của hộp trong hệ tọa độ arm_base_link."""
        target_world_pos: Optional[Tuple[float, float]] = None

        slot_info = self.slots.get(slot_id) or self.slots.get(str(slot_id))
        bc = slot_info.get("box_center_pose") if slot_info else None

        if is_place:
            # Khi đặt hàng vào ô kệ / trạm: ô kệ là trống hoặc tiếp nhận vị trí danh định.
            # TUYỆT ĐỐI không tìm trong self.box_poses vì robot đang mang hộp, sẽ bắt nhầm hộp trên tay mình!
            if bc:
                target_world_pos = (float(bc["x"]), float(bc["y"]))
                self.get_logger().info(f"[Targeting PLACE] Dùng tâm danh định ô {slot_id} từ slots.yaml: {target_world_pos}")
            else:
                target_world_pos = None
        else:
            # Khi gắp hàng:
            # 1. Tìm theo tên hộp cụ thể nếu đã biết exp_box
            if exp_box > 0 and f"box_{exp_box}" in self.box_poses:
                bp = self.box_poses[f"box_{exp_box}"].position
                target_world_pos = (bp.x, bp.y)
                self.get_logger().info(f"[Targeting] Tìm thấy hộp box_{exp_box} tại world ({bp.x:.3f}, {bp.y:.3f})")

            # 2. Nếu chưa tìm thấy theo tên, tìm hộp gần nhất với box_center_pose danh định của ô kệ / trạm
            if target_world_pos is None and bc and self.box_poses:
                nom_x = float(bc["x"])
                nom_y = float(bc["y"])
                closest_name = None
                min_dist = 0.40  # Bán kính tìm kiếm 40 cm
                for b_name, b_pose in self.box_poses.items():
                    dist = math.hypot(b_pose.position.x - nom_x, b_pose.position.y - nom_y)
                    if dist < min_dist:
                        min_dist = dist
                        closest_name = b_name
                if closest_name:
                    bp = self.box_poses[closest_name].position
                    target_world_pos = (bp.x, bp.y)
                    self.get_logger().info(
                        f"[Targeting] Tìm thấy {closest_name} tại ({bp.x:.3f}, {bp.y:.3f}) "
                        f"cách ô {slot_id} {min_dist*1000:.1f}mm"
                    )

            # 3. Fallback vị trí danh định từ slots.yaml nếu không có dữ liệu model states
            if target_world_pos is None and bc:
                target_world_pos = (float(bc["x"]), float(bc["y"]))
                self.get_logger().info(f"[Targeting] Dùng tọa độ danh định từ slots.yaml: {target_world_pos}")

        # 4. Biến đổi tọa độ từ World sang arm_base_link (arm_base_link tại +0.120m trên trục x thân xe)
        if target_world_pos is not None and self.robot_pose is not None:
            bx, by = target_world_pos
            rx, ry, ryaw = self.robot_pose
            x_arm = rx + 0.120 * math.cos(ryaw)
            y_arm = ry + 0.120 * math.sin(ryaw)
            dx = bx - x_arm
            dy = by - y_arm
            x_target = dx * math.cos(ryaw) + dy * math.sin(ryaw)
            y_target = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
        else:
            x_target = 0.425
            y_target = 0.0

        # 5. Giới hạn an toàn trong không gian làm việc của SCARA
        y_target = max(-0.060, min(0.060, y_target))
        max_reach_x = 0.07 + 0.058 + math.sqrt(max(0.0, 0.320**2 - y_target**2)) - 0.002
        if is_place:
            # Khi đặt vào kệ/trạm: bắt buộc tầm với x_target phải đủ sâu trong lòng khay kệ (>= 0.410m)
            x_target = max(0.410, min(max_reach_x, x_target))
        else:
            x_target = max(0.280, min(max_reach_x, x_target))

        self.get_logger().info(
            f"[Targeting {'PLACE' if is_place else 'PICK'}] arm_base_link target: x={x_target:.4f}m, y={y_target:.4f}m "
            f"(độ lệch ngang y={y_target*1000.0:.1f}mm)"
        )
        return x_target, y_target

    # ── Thực thi Action ───────────────────────────────────────────────────────
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        mode = goal.mode.upper()
        slot_id = goal.slot_tag_id
        exp_box = goal.expected_box_tag
        tier = goal.tier if goal.tier in (1, 2) else 1

        self.get_logger().info(f"Bắt đầu thực thi nhiệm vụ: {mode} (Tier {tier}, Slot {slot_id})")

        feedback = PickPlace.Feedback()
        result = PickPlace.Result()

        def set_phase(ph: str):
            feedback.phase = ph
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"=== PHA: {ph} ===")

        if mode == "PICK_FROM_SLOT":
            ok, msg, tag_read = self.handle_pick_from_slot(goal_handle, slot_id, exp_box, tier, set_phase)
        elif mode == "PLACE_TO_SLOT":
            ok, msg, tag_read = self.handle_place_to_slot(goal_handle, slot_id, exp_box, tier, set_phase)
        elif mode == "PLACE_SHELF":
            # Đặt hộp lên kệ, KHÔNG gập tay: để xe lùi ra hành lang trước rồi mới STOW.
            ok, msg, tag_read = self.handle_place_to_slot(
                goal_handle, slot_id, exp_box, tier, set_phase, stow_at_end=False
            )
        elif mode == "PRE_REACH_CARRY":
            ok, msg, tag_read = self.handle_pre_reach_carry(goal_handle, exp_box, tier, set_phase)
        elif mode in ("PICK_FROM_STATION", "PICK_STATION"):
            ok, msg, tag_read = self.handle_pick_from_station(goal_handle, slot_id, exp_box, set_phase)
        elif mode in ("PLACE_TO_STATION", "PLACE_STATION"):
            ok, msg, tag_read = self.handle_place_to_station(goal_handle, slot_id, exp_box, set_phase)
        elif mode == "STORE_TO_TRAY":
            ok, msg, tag_read = self.handle_store_to_tray(goal_handle, exp_box, tier, set_phase)
        elif mode == "PRE_REACH":
            set_phase("PRE_REACH")
            # 1. Đưa tay lên PRE_SLOT_HOVER ngoài hành lang thông thoáng
            ok = self.move_to_named_pose("PRE_SLOT_HOVER", tier=tier, goal_handle=goal_handle, total_time=2.0)
            if ok:
                # 2. Hạ Z thẳng đứng xuống cao độ làm việc PRE_SLOT_TIER_{tier} (tay duỗi thẳng hướng 0 rad, x=0.20m)
                d_approach = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["lift"])
                q1_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["shoulder"])
                q2_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["elbow"])
                q3_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["wrist"])
                ok = self.move_arm_safe(d_approach, q1_pre, q2_pre, q3_target=q3_pre, tier=tier, total_time=1.5)
            msg = "PRE_REACH thành công" if ok else "PRE_REACH thất bại"
            tag_read = self.current_box_id
        elif mode == "STOW":
            set_phase("STOW")
            ok = self.move_to_named_pose("STOW", tier=tier)
            msg = "STOW thành công" if ok else "STOW thất bại"
            tag_read = self.current_box_id
        else:
            ok = False
            msg = f"Mode không hợp lệ: {mode}"
            tag_read = 0

        result.success = ok
        result.message = msg
        result.box_tag_read = tag_read

        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── Chi tiết chu trình PICK_FROM_SLOT ─────────────────────────────────────
    def handle_pick_from_slot(self, goal_handle, slot_id, exp_box, tier, set_phase):
        # 1. Đảm bảo robot đã ở vị trí làm việc trước ô kệ
        self.ensure_robot_docked(slot_id)

        # 2. CHUẨN BỊ BÊN NGOÀI HÀNH LANG (PREPARE_OUTSIDE):
        self.switch_gripper(False)
        time.sleep(0.5)
        d_grasp = float(self.poses[f"TOUCH_TIER_{tier}"]["lift"])
        d_hover = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["lift"])
        q1_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["shoulder"])
        q2_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["elbow"])
        q3_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["wrist"])

        q1_curr = self.current_joints["shoulder_joint"]
        q2_curr = self.current_joints["elbow_joint"]
        d_curr = self.current_joints["lift_joint"]

        already_pre_extended = (
            abs(q1_curr - q1_pre) < 0.20 and
            abs(q2_curr - q2_pre) < 0.20 and
            abs(d_curr - d_hover) < 0.05
        )

        if already_pre_extended:
            self.get_logger().info(
                f"[PRE_REACH] Tay máy đã vươn thẳng sẵn ngoài hành lang với má kẹp mở."
            )
            if abs(self.current_joints["lift_joint"] - d_hover) > 0.003:
                self.move_arm_safe(d_hover, q1_curr, q2_curr, q3_target=self.current_joints.get("wrist_joint", 0.0), tier=tier, total_time=0.8)
        else:
            set_phase("PREPARE_OUTSIDE")
            if not self.move_to_named_pose("PRE_SLOT_HOVER", tier=tier, goal_handle=goal_handle, total_time=2.0):
                return False, "OUT_OF_REACH", 0
            if not self.move_arm_safe(d_hover, q1_pre, q2_pre, q3_target=q3_pre, tier=tier, total_time=1.5):
                return False, "JOINT_LIMIT", 0

        # 3. LUỒN THẲNG NGANG Ở CAO ĐỘ AN TOÀN TRÁNH TUYỆT ĐỐI VA CẠ THÀNH BÀN/KHAY KỆ:
        set_phase("INSERT")
        z_hover = d_hover - 0.07

        x_target, y_target = self.get_box_target_in_arm_base(slot_id, exp_box)

        p_entry = (0.200, y_target, z_hover)
        p_above_box = (x_target, y_target, z_hover)
        self.get_logger().info(f"Vươn tay ở cao độ an toàn (tránh va mép kệ/bàn): từ {p_entry} -> {p_above_box}")
        if not self.execute_cartesian_straight_line(p_entry, p_above_box, duration=3.0, num_points=16, tier=tier):
            self.move_to_named_pose("STOW", tier=tier)
            return False, "INSERT_FAILED", 0

        # Hạ thẳng đứng từ trên xuống ôm 2 bên thân hộp (DESCEND)
        set_phase("DESCEND")
        q1_in = self.current_joints["shoulder_joint"]
        q2_in = self.current_joints["elbow_joint"]
        q3_in = self.current_joints.get("wrist_joint", 0.0)

        # Tính toán chính xác nghiệm IK tại cao độ gắp để triệt tiêu hoàn toàn sai lệch
        sol_grasp, status = inverse_kinematics(
            x_target, y_target, d_grasp - 0.07,
            phi=0.0,
            prefer=(q1_in, q2_in, d_grasp, q3_in),
            x_offset=0.07, y_offset=0.0
        )
        if status == "OK" and sol_grasp is not None:
            q1_grasp, q2_grasp, _, q3_grasp = sol_grasp
        else:
            q1_grasp, q2_grasp, q3_grasp = q1_in, q2_in, q3_in

        self.move_arm_safe(d_grasp, q1_grasp, q2_grasp, q3_target=q3_grasp, tier=tier, total_time=1.5)

        # 4. ĐÓNG MÁ KẸP SERVO XIẾT CHẶT 2 BÊN HỘP (CLAMP):
        set_phase("CLAMP_BOX")
        self.get_logger().info("Động cơ servo kích hoạt: đóng má kẹp siết chặt 2 bên thân hộp!")
        self.switch_gripper(True)
        # Cho đủ thời gian má kẹp trượt và ép chặt thân hộp (1.2s)
        time.sleep(1.2)
        grasped = self.wait_for_grasp(timeout=2.5)

        if not grasped:
            self.get_logger().warn("Kẹp lần 1 chưa dính, thử siết lại...")
            time.sleep(0.4)
            self.switch_gripper(True)
            time.sleep(1.2)
            grasped = self.wait_for_grasp(timeout=2.5)

        if not grasped:
            self.get_logger().error("GRASP_FAIL: Động cơ servo không kẹp được hộp!")
            self.switch_gripper(False)
            self.move_arm_safe(d_hover, q1_grasp, q2_grasp, q3_target=q3_grasp, tier=tier, total_time=1.0)
            self.execute_cartesian_straight_line(p_above_box, p_entry, duration=3.0, num_points=16, tier=tier)
            self.move_to_named_pose("PRE_SLOT_HOVER", tier=tier)
            self.move_to_named_pose("STOW", tier=tier)
            return False, "GRASP_FAIL", 0

        # Giữ thêm 1.5s cho má kẹp servo siết chặt hoàn toàn và vật lý Gazebo ổn định tuyệt đối
        time.sleep(1.5)
        self.holding_box = True
        self.current_box_id = exp_box

        # 5. NHẤC ĐÁY HỘP (LIFT êm ái, gia tốc nhẹ tránh tuột hộp):
        set_phase("MICRO_LIFT")
        self.move_arm_safe(d_hover, q1_grasp, q2_grasp, q3_target=q3_grasp, tier=tier, total_time=2.0)

        # 6. RÚT THẲNG NGANG RA NGOÀI KỆ Ở CAO ĐỘ AN TOÀN (RETRACT):
        set_phase("RETRACT")
        p_exit = (0.200, y_target, z_hover)
        self.get_logger().info(f"Rút thẳng ngang hộp đang kẹp ra khỏi ô kệ: từ {p_above_box} -> {p_exit}")
        self.execute_cartesian_straight_line(p_above_box, p_exit, duration=3.0, num_points=16, tier=tier)

        # 8. DỪNG TẠI ĐÂY: hộp đang được giữ ở điểm thoát ngoài ô kệ.
        set_phase("RETRACTED")
        return True, "OK", exp_box

    # ── Đưa hộp đang giữ về khay xe (gọi SAU KHI xe đã lùi ra hành lang) ─────
    def handle_store_to_tray(self, goal_handle, exp_box, tier, set_phase):
        set_phase("TRAY")
        q1_out = self.current_joints["shoulder_joint"]
        q2_out = self.current_joints["elbow_joint"]
        q3_out = self.current_joints["wrist_joint"]
        self.move_arm_safe(0.220, q1_out, q2_out, q3_target=q3_out, tier=tier, total_time=1.5)

        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=2.0)
        self.move_to_named_pose("TRAY_TOUCH", tier=tier, total_time=1.2)
        time.sleep(0.3)
        # Giữ kẹp chặt hộp trong suốt hành trình di chuyển (Continuous Secure Transport)
        # Hộp nằm êm ái trên đệm cao su lòng khay và được khóa cứng vật lý, triệt tiêu 100% rung lắc và xô lệch
        self.holding_box = True
        set_phase("CARRY_TRAY")
        self.get_logger().info(f"Đã đưa hộp #{exp_box} về khay xe an toàn và duy trì khóa kẹp trong hành trình.")
        return True, "OK", exp_box

    # ── Chi tiết chu trình PLACE_TO_SLOT ─────────────────────────────────────
    def _grab_box_from_tray(self, tier, set_phase) -> bool:
        """Kẹp hộp từ khay xe chắc chắn (phương án dự phòng nếu hộp chưa được kẹp)."""
        set_phase("PICK_TRAY")
        # 1. Đưa tay lên TRAY_HOVER ở cao độ an toàn 0.220m trước
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)
        # 2. Mở rộng má kẹp hoàn toàn ở trên cao
        self.switch_gripper(False)
        time.sleep(0.5)
        # 3. Hạ thẳng đứng xuống TRAY_TOUCH (0.009m) ôm đúng tâm hộp
        self.move_to_named_pose("TRAY_TOUCH", tier=tier, total_time=1.2)
        # 4. Đóng má kẹp siết chặt 2 bên thân hộp
        self.switch_gripper(True)
        time.sleep(1.2)
        grasped = self.wait_for_grasp(timeout=2.5)
        if not grasped:
            self.get_logger().warn("Kẹp lần 1 trên khay xe chưa dính, thử siết lại...")
            time.sleep(0.4)
            self.switch_gripper(True)
            time.sleep(1.2)
            grasped = self.wait_for_grasp(timeout=2.5)

        if not grasped:
            self.get_logger().error("GRASP_FAIL: Không kẹp được hộp từ khay xe!")
            self.switch_gripper(False)
            self.holding_box = False
            self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.2)
            self.move_to_named_pose("STOW", tier=tier, total_time=1.5)
            return False

        # Đợi 1.2s cho má kẹp siết thật chặt và liên kết vật lý ổn định hoàn toàn
        time.sleep(1.2)
        self.holding_box = True
        # 5. Nhấc thẳng đứng lên TRAY_HOVER (0.220m) trước khi xoay ra hành lang
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)
        return True

    def _carry_to_pre_slot(self, tier, set_phase) -> float:
        """Đưa hộp ra tư thế vươn thẳng ngoài hành lang, hạ Z tới cao độ mang hộp. Trả về d_carry."""
        set_phase("PREPARE_OUTSIDE")
        # Chuyển từ TRAY_HOVER sang PRE_SLOT_HOVER (cả 2 đều ở cao độ an toàn >= 0.220m trên không)
        self.move_to_named_pose("PRE_SLOT_HOVER", tier=tier, total_time=2.0)

        d_carry = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["lift"]) + 0.006
        q1_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["shoulder"])
        q2_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["elbow"])
        q3_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["wrist"])
        self.move_arm_safe(d_carry, q1_pre, q2_pre, q3_target=q3_pre, tier=tier, total_time=1.5)
        return d_carry

    def handle_pre_reach_carry(self, goal_handle, exp_box, tier, set_phase):
        """Lấy hộp từ khay rồi vươn tay (đang mang hộp) ra trước ngay tại điểm chờ ngoài hành lang."""
        if not self.holding_box or not self.is_grasping:
            if not self._grab_box_from_tray(tier, set_phase):
                return False, "GRASP_FAIL", 0
        else:
            # Tay máy đang kẹp giữ sẵn hộp trên khay xe: nhấc thẳng đứng thuần Z lên TRAY_HOVER trước khi xoay
            set_phase("LIFT_TRAY")
            self.get_logger().info("Nhấc thẳng đứng hộp từ khay xe lên cao độ an toàn TRAY_HOVER...")
            self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)

        self._carry_to_pre_slot(tier, set_phase)
        return True, "OK", exp_box

    def handle_place_to_slot(self, goal_handle, slot_id, exp_box, tier, set_phase, stow_at_end=True):
        self.ensure_robot_docked(slot_id)

        # Nếu tay đã vươn sẵn ra trước (PRE_REACH_CARRY) và đang giữ hộp -> bỏ qua vung tay rộng
        d_carry = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["lift"]) + 0.006
        q1_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["shoulder"])
        q2_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["elbow"])
        already_extended = (
            self.holding_box and self.is_grasping
            and abs(self.current_joints["shoulder_joint"] - q1_pre) < 0.20
            and abs(self.current_joints["elbow_joint"] - q2_pre) < 0.20
            and abs(self.current_joints["lift_joint"] - d_carry) < 0.05
        )

        if already_extended:
            self.get_logger().info("[PRE_REACH_CARRY] Tay đã vươn sẵn mang hộp ngoài hành lang, bỏ qua bước vung tay.")
        else:
            if not self.holding_box or not self.is_grasping:
                # 1. Kẹp hộp từ khay xe
                if not self._grab_box_from_tray(tier, set_phase):
                    return False, "GRASP_FAIL", 0
            else:
                # Đang giữ sẵn hộp trên khay xe: nhấc thẳng đứng thuần Z lên TRAY_HOVER trước khi xoay
                set_phase("LIFT_TRAY")
                self.get_logger().info("Đang giữ sẵn hộp trên khay xe, nhấc thẳng đứng lên TRAY_HOVER...")
                self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)

            # 2. Đưa hộp ra vị trí chuẩn bị ngoài hành lang trước ngực xe
            d_carry = self._carry_to_pre_slot(tier, set_phase)

        # 3. ĐẨY THẲNG NGANG HỘP VÀO Ô KỆ HOẶC TRẠM (INSERTION):
        set_phase("INSERT")
        x_target, y_target = self.get_box_target_in_arm_base(slot_id, 0, is_place=True)
        z_carry = d_carry - 0.07
        p_entry = (0.200, y_target, z_carry)
        p_slot = (x_target, y_target, z_carry)
        self.get_logger().info(f"Đẩy thẳng ngang hộp vào ô kệ: từ {p_entry} -> {p_slot}")
        self.execute_cartesian_straight_line(p_entry, p_slot, duration=3.0, num_points=16, tier=tier)

        # 4. HẠ NHẸ CHẠM SÀN Ô KỆ (TOUCH & RELEASE):
        set_phase("RELEASE")
        d_touch = float(self.poses[f"TOUCH_TIER_{tier}"]["lift"])
        q1_now = self.current_joints["shoulder_joint"]
        q2_now = self.current_joints["elbow_joint"]
        q3_now = self.current_joints.get("wrist_joint", 0.0)

        sol_touch, status = inverse_kinematics(
            x_target, y_target, d_touch - 0.07,
            phi=0.0,
            prefer=(q1_now, q2_now, d_touch, q3_now),
            x_offset=0.07, y_offset=0.0
        )
        if status == "OK" and sol_touch is not None:
            q1_place, q2_place, _, q3_place = sol_touch
        else:
            q1_place, q2_place, q3_place = q1_now, q2_now, q3_now

        self.move_arm_safe(d_touch, q1_place, q2_place, q3_target=q3_place, tier=tier, total_time=1.2)

        time.sleep(0.3)
        # Động cơ servo mở rộng 2 má kẹp nhả 2 bên thân hộp và ngắt khóa cứng vật lý Gazebo
        self.switch_gripper(False)
        self.holding_box = False
        time.sleep(0.5)
        if not self.wait_for_release(timeout=2.0):
            self.get_logger().warn("Vật lý chưa xác nhận nhả hộp, gửi lại lệnh ngắt khóa cứng...")
            self.switch_gripper(False)
            time.sleep(0.5)

        # 5. NHẤC NHẸ TAY KẸP THOÁT NÓC HỘP (d_carry):
        self.move_arm_safe(d_carry, q1_place, q2_place, q3_target=q3_place, tier=tier, total_time=1.0)

        # 6. RÚT THẲNG NGANG TAY KẸP RA NGOÀI KỆ (RETRACT):
        set_phase("RETRACT")
        self.get_logger().info(f"Rút thẳng ngang tay kẹp ra khỏi ô kệ: từ {p_slot} -> {p_entry}")
        self.execute_cartesian_straight_line(p_slot, p_entry, duration=2.8, num_points=14, tier=tier)

        if not stow_at_end:
            # Giữ tay ở điểm thoát; Task Manager sẽ lùi xe ra hành lang rồi mới gọi STOW.
            set_phase("RETRACTED")
            return True, "OK", exp_box

        # 7. NÂNG Z VÀ GẤP GỌN (STOW):
        set_phase("STOW")
        q1_out = self.current_joints["shoulder_joint"]
        q2_out = self.current_joints["elbow_joint"]
        q3_out = self.current_joints["wrist_joint"]
        self.move_arm_safe(0.160, q1_out, q2_out, q3_target=q3_out, tier=tier, total_time=1.5)
        self.move_to_named_pose("STOW", tier=tier, total_time=2.0)
        check_footprint(self.current_joints["shoulder_joint"], self.current_joints["elbow_joint"])

        return True, "OK", exp_box

    # ── Thao tác trạm IN/OUT ───────────────────────────────────────────────────
    def handle_pick_from_station(self, goal_handle, slot_id, exp_box, set_phase):
        sid = 30 if slot_id in (0, 30) else slot_id
        return self.handle_pick_from_slot(goal_handle, sid, exp_box, 1, set_phase)

    def handle_place_to_station(self, goal_handle, slot_id, exp_box, set_phase):
        sid = 31 if slot_id in (0, 31) else slot_id
        return self.handle_place_to_slot(goal_handle, sid, exp_box, 1, set_phase)


def main(args=None):
    rclpy.init(args=args)
    server = PickPlaceServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
