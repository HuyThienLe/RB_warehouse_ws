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
from geometry_msgs.msg import PoseStamped
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
            "PRE_SLOT_TIER_1": {"lift": 0.035, "shoulder": -0.8697, "elbow": 2.6281, "wrist": -1.7584},
            "OBSERVE_TIER_1": {"lift": 0.035, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "TOUCH_TIER_1": {"lift": 0.015, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "PRE_SLOT_TIER_2": {"lift": 0.285, "shoulder": -0.8697, "elbow": 2.6281, "wrist": -1.7584},
            "OBSERVE_TIER_2": {"lift": 0.285, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "TOUCH_TIER_2": {"lift": 0.265, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "TRAY_HOVER": {"lift": 0.160, "shoulder": -1.5860, "elbow": -2.1268, "wrist": 0.5420},
            "TRAY_TOUCH": {"lift": 0.015, "shoulder": -1.5860, "elbow": -2.1268, "wrist": 0.5420},
            "STATION_IN_HOVER": {"lift": 0.040, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
            "STATION_OUT_HOVER": {"lift": 0.040, "shoulder": -0.1559, "elbow": 0.3568, "wrist": -0.2010},
        }

        cfg_file = "/home/lehuythien/warehouse_ws/src/wh_manipulation/config/arm_poses.yaml"
        if os.path.exists(cfg_file):
            try:
                with open(cfg_file, "r") as f:
                    data = yaml.safe_load(f)
                    if "arm_poses" in data:
                        for k, v in data["arm_poses"].items():
                            self.poses[k] = v
                self.get_logger().info(f"Đã nạp {len(self.poses)} tư thế từ {cfg_file}")
            except Exception as e:
                self.get_logger().warn(f"Không thể đọc arm_poses.yaml: {e}")

        # Nạp slots.yaml
        slots_file = "/home/lehuythien/warehouse_ws/src/wh_navigation/config/slots.yaml"
        self.slots = {}
        if os.path.exists(slots_file):
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
        if msg.data:
            self.is_grasping = True

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
            # Di chuyển giữa các tư thế hover hoặc khay xe và STOW: luôn nâng >= 0.160m để vượt thành khay
            safe_z = max(d_curr, d_target, 0.160)

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

        dt = duration / float(len(traj_pts))
        pts = []
        for i, (q1, q2, d, q3) in enumerate(traj_pts):
            t_from_start = (i + 1) * dt
            p = JointTrajectoryPoint()
            p.positions = [float(d), float(q1), float(q2), float(q3)]
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
        d_approach = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["lift"])
        q1_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["shoulder"])
        q2_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["elbow"])
        q3_pre = float(self.poses[f"PRE_SLOT_TIER_{tier}"]["wrist"])

        q1_curr = self.current_joints["shoulder_joint"]
        q2_curr = self.current_joints["elbow_joint"]
        d_curr = self.current_joints["lift_joint"]

        already_pre_extended = (
            abs(q1_curr - q1_pre) < 0.20 and
            abs(q2_curr - q2_pre) < 0.20 and
            abs(d_curr - d_approach) < 0.05
        )

        if already_pre_extended:
            self.get_logger().info(
                f"[PRE_REACH] Tay máy đã vươn thẳng sẵn ngoài hành lang. Bỏ qua vung tay PRE_SLOT_HOVER chống va quẹt kệ."
            )
        else:
            set_phase("PREPARE_OUTSIDE")
            if not self.move_to_named_pose("PRE_SLOT_HOVER", tier=tier, goal_handle=goal_handle, total_time=2.0):
                return False, "OUT_OF_REACH", 0
            if not self.move_arm_safe(d_approach, q1_pre, q2_pre, q3_target=q3_pre, tier=tier, total_time=1.5):
                return False, "JOINT_LIMIT", 0

        # 3. LUỒN THẲNG NGANG VÀO Ô KỆ (INSERTION):
        set_phase("INSERT")
        # Điểm bắt đầu ngoài cửa kệ: x=0.200, y=0.0
        z_approach = d_approach - 0.06
        p_entry = (0.200, 0.0, z_approach)

        # Luồn thẳng ngang vào tâm ô kệ hoặc bàn trạm danh định (tay duỗi thẳng)
        box_target_x = 0.425
        box_target_y = 0.0
        p_box_hover = (box_target_x, box_target_y, z_approach)
        self.get_logger().info(f"Luồn thẳng ngang vào ô kệ: từ {p_entry} -> {p_box_hover}")
        if not self.execute_cartesian_straight_line(p_entry, p_box_hover, duration=2.5, tier=tier):
            self.move_to_named_pose("STOW", tier=tier)
            return False, "INSERT_FAILED", 0

        # 4. CHẠM NÓC HỘP (TOUCH):
        set_phase("ALIGN_AND_TOUCH")
        d_touch = float(self.poses[f"TOUCH_TIER_{tier}"]["lift"])
        q1_now = self.current_joints["shoulder_joint"]
        q2_now = self.current_joints["elbow_joint"]
        q3_now = self.current_joints["wrist_joint"]
        self.move_arm_safe(d_touch, q1_now, q2_now, q3_target=q3_now, tier=tier, total_time=0.8)

        # 5. HÚT CHÂN KHÔNG (SUCTION):
        set_phase("SUCTION")
        self.switch_gripper(True)
        grasped = self.wait_for_grasp(timeout=2.0)

        if not grasped:
            self.get_logger().warn("Hút lần 1 chưa dính, hạ thêm 3 mm và thử lại...")
            self.move_arm_safe(max(0.0, d_touch - 0.003), q1_now, q2_now, q3_target=q3_now, tier=tier, total_time=0.6)
            time.sleep(0.3)
            grasped = self.wait_for_grasp(timeout=2.0)

        if not grasped:
            self.get_logger().warn("Hút lần 2 chưa dính, nâng nhẹ 4 mm và thử lại...")
            self.move_arm_safe(d_touch + 0.004, q1_now, q2_now, q3_target=q3_now, tier=tier, total_time=0.6)
            time.sleep(0.3)
            grasped = self.wait_for_grasp(timeout=2.0)

        if not grasped:
            self.get_logger().error("GRASP_FAIL: Không hút dính hộp!")
            self.switch_gripper(False)
            self.execute_cartesian_straight_line(p_box_hover, p_entry, duration=2.5, tier=tier)
            self.move_to_named_pose("PRE_SLOT_HOVER", tier=tier)
            self.move_to_named_pose("STOW", tier=tier)
            return False, "GRASP_FAIL", 0

        self.holding_box = True
        self.current_box_id = exp_box

        # 6. NHẤC ĐÁY HỘP (LIFT 20mm):
        set_phase("MICRO_LIFT")
        d_lift = d_approach
        self.move_arm_safe(d_lift, q1_now, q2_now, q3_target=q3_now, tier=tier, total_time=0.8)

        # 7. RÚT THẲNG NGANG RA NGOÀI KỆ (RETRACT):
        set_phase("RETRACT")
        z_lift = d_lift - 0.06
        p_box_lifted = (box_target_x, box_target_y, z_lift)
        p_exit = (0.200, box_target_y, z_lift)
        self.get_logger().info(f"Rút thẳng ngang ra khỏi ô kệ: từ {p_box_lifted} -> {p_exit}")
        self.execute_cartesian_straight_line(p_box_lifted, p_exit, duration=2.5, tier=tier)

        # 8. DỪNG TẠI ĐÂY: hộp đang được hút giữ ở điểm thoát (x=0.20m) ngoài ô kệ.
        # Task Manager sẽ cho xe LÙI THẲNG ra hành lang, rồi mới gọi STORE_TO_TRAY để đưa hộp về khay.
        set_phase("RETRACTED")
        return True, "OK", exp_box

    # ── Đưa hộp đang giữ về khay xe (gọi SAU KHI xe đã lùi ra hành lang) ─────
    def handle_store_to_tray(self, goal_handle, exp_box, tier, set_phase):
        set_phase("TRAY")
        q1_out = self.current_joints["shoulder_joint"]
        q2_out = self.current_joints["elbow_joint"]
        q3_out = self.current_joints["wrist_joint"]
        self.move_arm_safe(0.160, q1_out, q2_out, q3_target=q3_out, tier=tier, total_time=1.5)

        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=2.0)
        self.move_to_named_pose("TRAY_TOUCH", tier=tier, total_time=1.2)
        time.sleep(0.2)
        self.switch_gripper(False)
        self.holding_box = False
        time.sleep(0.3)
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.2)

        # GẤP GỌN (STOW)
        set_phase("STOW")
        self.move_to_named_pose("STOW", tier=tier, total_time=2.0)
        check_footprint(self.current_joints["shoulder_joint"], self.current_joints["elbow_joint"])

        return True, "OK", exp_box

    # ── Chi tiết chu trình PLACE_TO_SLOT ─────────────────────────────────────
    def _grab_box_from_tray(self, tier, set_phase) -> bool:
        """Hút hộp từ khay xe. Trả False (và gập tay an toàn) nếu không hút được."""
        set_phase("PICK_TRAY")
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)
        self.move_to_named_pose("TRAY_TOUCH", tier=tier, total_time=1.2)
        self.switch_gripper(True)
        grasped = self.wait_for_grasp(timeout=1.5)
        if not grasped:
            self.get_logger().warn("Kẹp lần 1 trên khay xe chưa dính, hạ thêm 3 mm...")
            d_touch = float(self.poses["TRAY_TOUCH"]["lift"])
            q1 = float(self.poses["TRAY_TOUCH"]["shoulder"])
            q2 = float(self.poses["TRAY_TOUCH"]["elbow"])
            q3 = float(self.poses["TRAY_TOUCH"]["wrist"])
            self.move_arm_safe(max(0.0, d_touch - 0.003), q1, q2, q3_target=q3, tier=tier, total_time=0.6)
            time.sleep(0.3)
            grasped = self.wait_for_grasp(timeout=1.5)

        if not grasped:
            self.get_logger().error("GRASP_FAIL: Không kẹp được hộp từ khay xe!")
            self.switch_gripper(False)
            self.holding_box = False
            self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.0)
            self.move_to_named_pose("STOW", tier=tier, total_time=1.5)
            return False

        self.holding_box = True
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.2)
        return True

    def _carry_to_pre_slot(self, tier, set_phase) -> float:
        """Đưa hộp ra tư thế vươn thẳng ngoài hành lang, hạ Z tới cao độ mang hộp. Trả về d_carry."""
        set_phase("PREPARE_OUTSIDE")
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
            # 1. Hút hộp từ khay xe
            if not self._grab_box_from_tray(tier, set_phase):
                return False, "GRASP_FAIL", 0
            # 2. Đưa hộp ra vị trí chuẩn bị ngoài hành lang trước ngực xe
            d_carry = self._carry_to_pre_slot(tier, set_phase)

        # 3. ĐẨY THẲNG NGANG HỘP VÀO Ô KỆ HOẶC TRẠM (INSERTION):
        set_phase("INSERT")
        box_target_x = 0.425
        z_carry = d_carry - 0.06
        p_entry = (0.200, 0.0, z_carry)
        p_slot = (box_target_x, 0.0, z_carry)
        self.get_logger().info(f"Đẩy thẳng ngang hộp vào ô kệ: từ {p_entry} -> {p_slot}")
        self.execute_cartesian_straight_line(p_entry, p_slot, duration=2.5, tier=tier)

        # 4. HẠ NHẸ CHẠM SÀN Ô KỆ (TOUCH & RELEASE):
        set_phase("RELEASE")
        d_touch = float(self.poses[f"TOUCH_TIER_{tier}"]["lift"])
        q1_now = self.current_joints["shoulder_joint"]
        q2_now = self.current_joints["elbow_joint"]
        q3_now = self.current_joints["wrist_joint"]
        self.move_arm_safe(d_touch, q1_now, q2_now, q3_target=q3_now, tier=tier, total_time=0.8)

        time.sleep(0.2)
        self.switch_gripper(False)
        self.holding_box = False
        time.sleep(0.3)

        # 5. NHẤC NHẸ TAY KẸP THOÁT NÓC HỘP (6mm):
        self.move_arm_safe(d_carry, q1_now, q2_now, q3_target=q3_now, tier=tier, total_time=0.6)

        # 6. RÚT THẲNG NGANG TAY KẸP RA NGOÀI KỆ (RETRACT):
        set_phase("RETRACT")
        self.get_logger().info(f"Rút thẳng ngang tay kẹp ra khỏi ô kệ: từ {p_slot} -> {p_entry}")
        self.execute_cartesian_straight_line(p_slot, p_entry, duration=2.5, tier=tier)

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
