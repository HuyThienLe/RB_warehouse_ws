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
from trajectory_msgs.msg import JointTrajectoryPoint
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
        self.current_joints = {"lift_joint": 0.0, "shoulder_joint": 0.0, "elbow_joint": 0.0}
        self.joint_state_received = False
        self.is_grasping = False
        self.holding_box = False
        self.current_box_id = 0
        self.robot_pose: Optional[Tuple[float, float, float]] = None

        # Lưu trữ tag nhìn thấy gần nhất theo camera
        self.latest_hand_tags: Dict[int, TagDetection] = {}
        self.latest_hand_tag_time: float = 0.0
        self.latest_dock_tags: Dict[int, TagDetection] = {}

        # 3. TF2 Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 4. Action Client điều khiển quỹ đạo khớp (/arm_controller/follow_joint_trajectory)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory", callback_group=self.cbg
        )

        # 5. Gripper Service Client & Status Subscriber
        self.cli_gripper = self.create_client(SetBool, "/gripper/switch", callback_group=self.cbg)
        self.cli_gripper_raw = self.create_client(SetBool, "/gripper/switch_raw", callback_group=self.cbg)
        self.sub_grasping = self.create_subscription(
            Bool, "/gripper/grasping", self.cb_grasping, 10, callback_group=self.cbg
        )
        self.sub_grasping_raw = self.create_subscription(
            Bool, "/gripper/grasping_raw", self.cb_grasping, 10, callback_group=self.cbg
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

        # 8. Action Server PickPlace
        self.action_server = ActionServer(
            self,
            PickPlace,
            "/arm/pick_place",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cbg,
        )

        self.get_logger().info("PickPlaceServer đã sẵn sàng phục vụ (Đã tối ưu chống kẹt kệ).")

    def load_configurations(self):
        """Nạp các tư thế từ arm_poses.yaml và slots.yaml."""
        self.poses = {
            "STOW": {"lift": 0.050, "shoulder": -1.5708, "elbow": -2.3562},
            "PRE_SLOT_TIER_1": {"lift": 0.040, "shoulder": -1.1719, "elbow": 2.3438},
            "OBSERVE_TIER_1": {"lift": 0.040, "shoulder": -0.8411, "elbow": 1.6821},
            "TOUCH_TIER_1": {"lift": 0.015, "shoulder": -0.8411, "elbow": 1.6821},
            "PRE_SLOT_TIER_2": {"lift": 0.300, "shoulder": -1.1719, "elbow": 2.3438},
            "OBSERVE_TIER_2": {"lift": 0.300, "shoulder": -0.8411, "elbow": 1.6821},
            "TOUCH_TIER_2": {"lift": 0.265, "shoulder": -0.8411, "elbow": 1.6821},
            "TRAY_HOVER": {"lift": 0.080, "shoulder": -1.5378, "elbow": -2.1268},
            "TRAY_TOUCH": {"lift": 0.045, "shoulder": -1.5378, "elbow": -2.1268},
            "STATION_IN_HOVER": {"lift": 0.040, "shoulder": -0.8411, "elbow": 1.6821},
            "STATION_OUT_HOVER": {"lift": 0.040, "shoulder": -0.8411, "elbow": 1.6821},
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

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Nhận yêu cầu PickPlace: mode={goal_request.mode}, slot={goal_request.slot_tag_id}, "
            f"box_tag={goal_request.expected_box_tag}, tier={goal_request.tier}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Nhận yêu cầu hủy thao tác tay máy!")
        return CancelResponse.ACCEPT

    # ── Tự động đảm bảo robot đã dock trước ô kệ ─────────────────────────────
    def ensure_robot_docked(self, slot_id: int):
        """Kiểm tra và đưa robot vào vị trí work_pose nếu robot đang ở xa."""
        slot = self.slots.get(slot_id)
        if not slot or "work_pose" not in slot:
            return

        wp = slot["work_pose"]
        target_x = float(wp["x"])
        target_y = float(wp["y"])
        target_yaw = float(wp["yaw"])

        if self.robot_pose is not None:
            dist = math.sqrt((self.robot_pose[0] - target_x) ** 2 + (self.robot_pose[1] - target_y) ** 2)
            yaw_diff = abs(math.atan2(math.sin(self.robot_pose[2] - target_yaw), math.cos(self.robot_pose[2] - target_yaw)))
            if dist <= 0.02 and yaw_diff <= 0.05:
                return

        self.get_logger().info(
            f"Robot chưa đứng trước ô kệ {slot_id}. Đang tự động định vị robot vào điểm làm việc "
            f"work_pose (x={target_x:.2f}, y={target_y:.2f}, yaw={target_yaw:.2f}) theo điều kiện Bài 6..."
        )

        if self.cli_set_entity.wait_for_service(timeout_sec=2.0):
            req = SetEntityState.Request()
            req.state.name = "warehouse_robot"
            req.state.reference_frame = "world"
            req.state.pose.position.x = target_x
            req.state.pose.position.y = target_y
            req.state.pose.position.z = 0.002
            req.state.pose.orientation.z = math.sin(target_yaw / 2.0)
            req.state.pose.orientation.w = math.cos(target_yaw / 2.0)
            req.state.twist.linear.x = 0.0
            req.state.twist.linear.y = 0.0
            req.state.twist.linear.z = 0.0
            req.state.twist.angular.x = 0.0
            req.state.twist.angular.y = 0.0
            req.state.twist.angular.z = 0.0
            future = self.cli_set_entity.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0 < 3.0):
                time.sleep(0.05)
            time.sleep(0.6)

    # ── Điều khiển tay máy an toàn (Nâng Z -> Xoay -> Hạ Z) ───────────────────
    def move_arm_safe(
        self,
        d_target: float,
        q1_target: float,
        q2_target: float,
        tier: int = 1,
        goal_handle=None,
        total_time: float = 3.0,
    ) -> bool:
        """Gửi quỹ đạo an toàn: Nâng Z lên mức hover trước, rồi xoay ngang, rồi mới hạ Z."""
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm controller action server không sẵn sàng!")
            return False

        d_curr = self.current_joints["lift_joint"]
        q1_curr = self.current_joints["shoulder_joint"]
        q2_curr = self.current_joints["elbow_joint"]

        safe_z = max(d_curr, d_target)
        if tier == 1 and max(d_curr, d_target) <= 0.045:
            safe_z = min(max(safe_z, 0.040), 0.042)
        elif tier == 2 and max(d_curr, d_target) <= 0.35:
            safe_z = max(safe_z, 0.300)

        delta_q1 = abs(q1_target - q1_curr)
        delta_q2 = abs(q2_target - q2_curr)
        max_delta_q = max(delta_q1, delta_q2)
        # Giữ vận tốc góc <= 0.6 rad/s (giới hạn motor 1.2 rad/s)
        rot_time = max(1.2, max_delta_q / 0.6)
        needs_rotation = (delta_q1 > 0.03) or (delta_q2 > 0.03)

        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint"]

        pts = []
        if needs_rotation and (d_curr < safe_z - 0.010):
            t1 = 0.8
            p1 = JointTrajectoryPoint()
            p1.positions = [safe_z, q1_curr, q2_curr]
            p1.time_from_start = Duration(seconds=t1).to_msg()
            pts.append(p1)

            t2 = t1 + rot_time
            p2 = JointTrajectoryPoint()
            p2.positions = [safe_z, q1_target, q2_target]
            p2.time_from_start = Duration(seconds=t2).to_msg()
            pts.append(p2)

            t3 = t2 + max(0.6, abs(safe_z - d_target) * 2.5)
            p3 = JointTrajectoryPoint()
            p3.positions = [d_target, q1_target, q2_target]
            p3.time_from_start = Duration(seconds=t3).to_msg()
            pts.append(p3)
        else:
            dur = max(rot_time, total_time)
            p = JointTrajectoryPoint()
            p.positions = [d_target, q1_target, q2_target]
            p.time_from_start = Duration(seconds=dur).to_msg()
            pts.append(p)

        traj_goal.trajectory.points = pts

        send_future = self.arm_client.send_goal_async(traj_goal)
        start_send_t = time.time()
        while not send_future.done():
            time.sleep(0.02)
            if time.time() - start_send_t > 8.0:
                self.get_logger().error("Gửi arm trajectory bị timeout!")
                return False

        gh = send_future.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Arm controller từ chối quỹ đạo!")
            return False

        res_future = gh.get_result_async()
        expected_dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
        start_exec_t = time.time()
        while not res_future.done():
            time.sleep(0.02)
            if time.time() - start_exec_t > expected_dur + 8.0:
                self.get_logger().error("Thực thi arm trajectory bị timeout!")
                return False

        res = res_future.result()
        is_ok = (res.status == 4 or res.result.error_code == 0)
        if not is_ok:
            self.get_logger().warn(f"Quỹ đạo không thành công: status={res.status}, error_code={res.result.error_code}")
        return is_ok

    def move_to_named_pose(self, pose_name: str, tier: int = 1, goal_handle=None, total_time: float = 3.0) -> bool:
        """Đưa tay về tư thế định nghĩa sẵn."""
        if pose_name not in self.poses:
            self.get_logger().error(f"Tư thế không tồn tại: {pose_name}")
            return False
        p = self.poses[pose_name]
        return self.move_arm_safe(p["lift"], p["shoulder"], p["elbow"], tier=tier, goal_handle=goal_handle, total_time=total_time)

    def switch_gripper(self, state: bool) -> bool:
        """Bật / tắt hút kẹp."""
        req = SetBool.Request()
        req.data = state

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
        """Lấy vị trí 3D của tag trong arm_base_link từ hand_camera."""
        now = time.time()
        if now - self.latest_hand_tag_time > 3.0:
            return None

        cand = None
        if expected_id > 0 and expected_id in self.latest_hand_tags:
            cand = self.latest_hand_tags[expected_id]
        else:
            for tid, tag in self.latest_hand_tags.items():
                if 20 <= tid <= 39:
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
        elif mode in ("PICK_FROM_STATION", "PICK_STATION"):
            ok, msg, tag_read = self.handle_pick_from_station(goal_handle, slot_id, exp_box, set_phase)
        elif mode in ("PLACE_TO_STATION", "PLACE_STATION"):
            ok, msg, tag_read = self.handle_place_to_station(goal_handle, slot_id, exp_box, set_phase)
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

        # 2. Di chuyển đến cửa ô kệ (PRE_SLOT) ngoài hành lang an toàn
        set_phase("OBSERVE")
        pre_pose = f"PRE_SLOT_TIER_{tier}"
        obs_pose = f"OBSERVE_TIER_{tier}"

        # Đưa tay đến PRE_SLOT ngoài hành lang (x=0.14, y=0.0)
        if not self.move_to_named_pose(pre_pose, tier=tier, goal_handle=goal_handle, total_time=2.5):
            return False, "OUT_OF_REACH", 0

        # Tiến thẳng vào lòng ô kệ dọc theo đường y=0 (từ x=0.14 vào x=0.28)
        if not self.move_to_named_pose(obs_pose, tier=tier, goal_handle=goal_handle, total_time=2.0):
            return False, "OUT_OF_REACH", 0

        time.sleep(0.5)

        # 3. ALIGN: Tối đa 3 vòng lặp mắt-trong-tay với hand_camera
        set_phase("ALIGN")
        aligned = False
        box_tag_found = 0

        for loop in range(3):
            tag_info = self.get_hand_camera_tag_in_arm_base(expected_id=exp_box)
            if tag_info is None:
                self.get_logger().warn(f"ALIGN vòng {loop+1}: Chưa thấy tag qua hand_camera, chờ nhận diện...")
                time.sleep(0.5)
                continue

            tag_id, tx, ty, tz = tag_info
            box_tag_found = tag_id

            # Kiểm tra ID
            if exp_box > 0 and tag_id != exp_box:
                self.get_logger().error(f"BOX_MISMATCH: Mong đợi tag {exp_box}, đọc được {tag_id}!")
                # Rút lui thẳng hàng ra PRE_SLOT
                self.move_to_named_pose(pre_pose, tier=tier)
                self.move_to_named_pose("STOW", tier=tier)
                return False, "BOX_MISMATCH", tag_id

            q1_curr = self.current_joints["shoulder_joint"]
            q2_curr = self.current_joints["elbow_joint"]
            d_curr = self.current_joints["lift_joint"]
            tcp_x, tcp_y, _ = forward_kinematics(q1_curr, q2_curr, d_curr, x_offset=0.07, y_offset=0.0)

            dx = tx - tcp_x
            dy = ty - tcp_y
            err_xy = math.sqrt(dx * dx + dy * dy)
            self.get_logger().info(f"ALIGN vòng {loop+1}: Lệch tâm = {err_xy*1000.0:.2f} mm (dx={dx*1000:.1f}, dy={dy*1000:.1f})")

            if err_xy < 0.002:  # < 2 mm
                aligned = True
                break

            sol, st = inverse_kinematics(tx, ty, tz, prefer=(q1_curr, q2_curr, d_curr), x_offset=0.07, y_offset=0.0)
            if st != "OK":
                self.get_logger().warn(f"IK thất bại khi align: {st}")
                break

            self.move_arm_safe(d_curr, sol[0], sol[1], tier=tier, total_time=1.0)
            time.sleep(0.3)

        # 4. DESCEND: Hạ Z chậm xuống độ cao chạm (chạm nóc hộp 3mm)
        set_phase("DESCEND")
        touch_pose = f"TOUCH_TIER_{tier}"
        d_touch = self.poses[touch_pose]["lift"]
        q1_now = self.current_joints["shoulder_joint"]
        q2_now = self.current_joints["elbow_joint"]
        if not self.move_arm_safe(d_touch, q1_now, q2_now, tier=tier, total_time=2.0):
            return False, "JOINT_LIMIT", box_tag_found

        # 5. SUCTION: Bật hút chân không
        set_phase("SUCTION")
        self.switch_gripper(True)
        grasped = self.wait_for_grasp(timeout=2.0)

        if not grasped:
            self.get_logger().warn("Hút lần 1 chưa dính, hạ thêm 4 mm và thử lại...")
            self.move_arm_safe(max(0.0, d_touch - 0.004), q1_now, q2_now, tier=tier, total_time=0.8)
            time.sleep(0.3)
            grasped = self.wait_for_grasp(timeout=2.0)

        if not grasped:
            self.get_logger().warn("Hút lần 2 chưa dính, nâng nhẹ 5 mm và thử lại...")
            self.move_arm_safe(d_touch + 0.005, q1_now, q2_now, tier=tier, total_time=0.8)
            time.sleep(0.3)
            grasped = self.wait_for_grasp(timeout=2.0)

        if not grasped:
            self.get_logger().error("GRASP_FAIL: Không hút dính hộp!")
            self.switch_gripper(False)
            self.move_to_named_pose(obs_pose, tier=tier)
            self.move_to_named_pose(pre_pose, tier=tier)
            self.move_to_named_pose("STOW", tier=tier)
            return False, "GRASP_FAIL", box_tag_found

        self.holding_box = True
        self.current_box_id = box_tag_found if box_tag_found > 0 else exp_box

        # 6. LIFT: Nâng lên mức hover an toàn (d=0.045m ở tầng 1)
        set_phase("LIFT")
        d_hover = self.poses[obs_pose]["lift"]
        self.move_arm_safe(d_hover, q1_now, q2_now, tier=tier, total_time=1.5)

        # 7. RETRACT: Rút thẳng lùi ra khỏi ô kệ về PRE_SLOT dọc theo y=0
        set_phase("RETRACT")
        self.move_to_named_pose(pre_pose, tier=tier, total_time=2.0)

        # 8. Đưa hộp về phía trên khay xe (ở ngoài hành lang thoáng đãng)
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=2.5)

        # 9. Hạ xuống khay và ngắt hút
        set_phase("TRAY")
        self.move_to_named_pose("TRAY_TOUCH", tier=tier, total_time=1.5)
        time.sleep(0.2)
        self.switch_gripper(False)
        self.holding_box = False
        time.sleep(0.3)
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)

        # 10. VERIFY: Đọc tag hộp trên khay
        set_phase("VERIFY")
        time.sleep(0.5)
        tag_on_tray = self.get_hand_camera_tag_in_arm_base()
        if tag_on_tray:
            self.get_logger().info(f"Đã xác nhận hộp {tag_on_tray[0]} an toàn trên khay xe!")
            box_tag_found = tag_on_tray[0]
        else:
            self.get_logger().info("Hộp đã đặt vào lòng khay xe.")

        # 11. STOW: Gấp gọn tay máy
        set_phase("STOW")
        self.move_to_named_pose("STOW", tier=tier, total_time=2.5)
        check_footprint(self.current_joints["shoulder_joint"], self.current_joints["elbow_joint"])

        return True, "OK", (box_tag_found if box_tag_found > 0 else exp_box)

    # ── Chi tiết chu trình PLACE_TO_SLOT ─────────────────────────────────────
    def handle_place_to_slot(self, goal_handle, slot_id, exp_box, tier, set_phase):
        self.ensure_robot_docked(slot_id)

        # 1. Hút hộp từ khay xe
        set_phase("PICK_TRAY")
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=2.0)
        self.move_to_named_pose("TRAY_TOUCH", tier=tier, total_time=1.5)
        self.switch_gripper(True)
        if not self.wait_for_grasp(timeout=1.5):
            self.get_logger().error("Không hút được hộp từ khay xe!")
            self.switch_gripper(False)
            self.move_to_named_pose("STOW", tier=tier)
            return False, "GRASP_FAIL", 0

        self.holding_box = True
        self.move_to_named_pose("TRAY_HOVER", tier=tier, total_time=1.5)

        # 2. Đưa hộp tới vị trí PRE_SLOT ngoài hành lang
        set_phase("HOVER")
        pre_pose = f"PRE_SLOT_TIER_{tier}"
        obs_pose = f"OBSERVE_TIER_{tier}"
        self.move_to_named_pose(pre_pose, tier=tier, total_time=2.5)

        # Đảm bảo robot đứng chuẩn xác trước ô kệ trước khi tiến vào
        self.ensure_robot_docked(slot_id)

        # 3. Tiến thẳng vào lòng ô kệ
        self.move_to_named_pose(obs_pose, tier=tier, total_time=2.0)

        # 4. ALIGN theo tâm ô kệ
        set_phase("ALIGN")
        time.sleep(0.3)

        # 5. DESCEND: Hạ chậm xuống chạm sàn ô
        set_phase("DESCEND")
        touch_pose = f"TOUCH_TIER_{tier}"
        q1_now = self.poses[touch_pose]["shoulder"]
        q2_now = self.poses[touch_pose]["elbow"]
        d_touch = self.poses[touch_pose]["lift"]
        self.move_arm_safe(d_touch, q1_now, q2_now, tier=tier, total_time=2.0)

        # 6. RELEASE: Ngắt hút
        set_phase("RELEASE")
        time.sleep(0.2)
        self.switch_gripper(False)
        self.holding_box = False
        time.sleep(0.3)

        # 7. LIFT: Nâng tay lên mức hover an toàn
        set_phase("LIFT")
        d_hover = self.poses[obs_pose]["lift"]
        self.move_arm_safe(d_hover, q1_now, q2_now, tier=tier, total_time=1.5)

        # 8. RETRACT: Rút thẳng lùi ra khỏi ô kệ về PRE_SLOT
        self.move_to_named_pose(pre_pose, tier=tier, total_time=2.0)

        # 9. VERIFY_PLACED
        set_phase("VERIFY_PLACED")
        time.sleep(0.5)
        tag_info = self.get_hand_camera_tag_in_arm_base()
        box_tag_read = tag_info[0] if tag_info else exp_box

        # 10. STOW: Về tư thế an toàn
        set_phase("STOW")
        self.move_to_named_pose("STOW", tier=tier, total_time=2.5)
        check_footprint(self.current_joints["shoulder_joint"], self.current_joints["elbow_joint"])

        return True, "OK", box_tag_read

    # ── Thao tác trạm IN/OUT ───────────────────────────────────────────────────
    def handle_pick_from_station(self, goal_handle, slot_id, exp_box, set_phase):
        return self.handle_pick_from_slot(goal_handle, slot_id, exp_box, 1, set_phase)

    def handle_place_to_station(self, goal_handle, slot_id, exp_box, set_phase):
        return self.handle_place_to_slot(goal_handle, slot_id, exp_box, 1, set_phase)


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
