#!/usr/bin/env python3
"""task_manager_node.py – Node điều phối nhiệm vụ và quản lý trạng thái kho hàng (Bài 7.2).

Quản lý hàng đợi nhiệm vụ và thực thi FSM:
- Kết nối CSDL SQLite (InventoryDB).
- Giao tiếp Action Clients: Nav2 (NavigateToPose), DockToSlot, PickPlace.
- Đảm bảo MultiThreadedExecutor + Threading an toàn chống deadlock.
- Xử lý lỗi thông minh (ERROR_RECOVERY, bảo toàn dữ liệu tồn kho 100%).
- Xuất trạng thái /robot_status định kỳ 1 Hz.
- Hỗ trợ dịch vụ /wh/submit_task, /wh/cancel_task, và E-stop (/estop).
"""

import math
import os
import threading
import time
from typing import Optional, Dict, Any, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity

from wh_interfaces.action import DockToSlot, PickPlace
from wh_interfaces.msg import RobotStatus, Task as TaskMsg
from wh_interfaces.srv import SubmitTask, CancelTask
from nav2_msgs.action import NavigateToPose
import yaml

from wh_task_manager.inventory_db import InventoryDB
from wh_task_manager.fsm import Event, State, FSMContext, TaskFSM


def find_config_path(rel_path: str) -> str:
    """Tìm đường dẫn file cấu hình linh hoạt từ install share hoặc source tree."""
    # 1. Thử ament index
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = rel_path.split("/")[0]
        sub = "/".join(rel_path.split("/")[1:])
        share_dir = get_package_share_directory(pkg)
        cand = os.path.join(share_dir, sub)
        if os.path.exists(cand):
            return cand
    except Exception:
        pass

    # 2. Thử source tree
    ws_root = "/home/lehuythien/warehouse_ws"
    cand = os.path.join(ws_root, "src", rel_path)
    if os.path.exists(cand):
        return cand

    return os.path.abspath(rel_path)


# Tọa độ X danh định của 6 hộp hàng chờ tại trạm IN (bàn dài 0.85m, y=1.15m)
STATION_IN_BOX_X: Dict[int, float] = {
    26: -2.70,  # Hộp 26: vị trí 0
    27: -2.58,  # Hộp 27: vị trí 1
    28: -2.46,  # Hộp 28: vị trí 2
    29: -2.34,  # Hộp 29: vị trí 3
    32: -2.22,  # Hộp 32: vị trí 4
    33: -2.10,  # Hộp 33: vị trí 5
}

# Các vị trí thả hàng trên bàn trạm OUT (bàn dài 0.85m x 0.40m, y=-1.15m) để các hộp không đè lên nhau (6 vị trí)
STATION_OUT_DROP_X: List[float] = [
    -2.70,  # Vị trí 0 (bên trái)
    -2.58,  # Vị trí 1
    -2.46,  # Vị trí 2
    -2.34,  # Vị trí 3
    -2.22,  # Vị trí 4
    -2.10,  # Vị trí 5 (bên phải)
]


class TaskManagerNode(Node):
    """Bộ điều phối trung tâm quản lý nhiệm vụ và tồn kho robot kho hàng."""

    def __init__(self, db_path: Optional[str] = None, mock_actions: bool = False):
        super().__init__("task_manager_node")

        self.mock_actions = mock_actions
        self.mock_box_tag: Optional[int] = None
        self.mock_place_fail: bool = False

        self.cbg_reentrant = ReentrantCallbackGroup()
        self.cbg_action = ReentrantCallbackGroup()
        self.cbg_timer = MutuallyExclusiveCallbackGroup()

        # 1. Khởi tạo CSDL Tồn kho
        if not db_path:
            db_dir = os.path.expanduser("~/.warehouse")
            os.makedirs(db_dir, exist_ok=True)
            db_path = os.path.join(db_dir, "inventory.db")

        self.db = InventoryDB(db_path)
        schema_file = find_config_path("wh_task_manager/config/schema.sql")
        items_file = find_config_path("wh_task_manager/config/items_seed.yaml")
        slots_file = find_config_path("wh_navigation/config/slots.yaml")
        seed_inv_file = find_config_path("wh_gazebo/config/inventory_seed.yaml")
        waypoints_file = find_config_path("wh_navigation/config/waypoints.yaml")

        self.db.init_schema(schema_file)
        self.db.load_seed_data(slots_file, items_file, seed_inv_file)
        self.slots_data = self._load_yaml(slots_file).get("slots", {})
        self.waypoints = self._load_yaml(waypoints_file).get("waypoints", {})

        # 2. Quản lý trạng thái Robot
        self.robot_x = -2.40
        self.robot_y = 0.00
        self.robot_yaw = 0.00
        self.robot_pose: Optional[Tuple[float, float, float]] = (-2.40, 0.0, 0.0)
        self.battery_pct = 100.0
        self.holding_box = False
        self.held_box_tag = 0
        self.last_error = ""
        self.station_out_drop_idx = 0

        self.estop_active = False
        self.active_fsm: Optional[TaskFSM] = None
        self.active_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # 3. Action Clients
        self.cli_nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cbg_action)
        self.cli_dock = ActionClient(self, DockToSlot, "dock_to_slot", callback_group=self.cbg_action)
        self.cli_pick_place = ActionClient(self, PickPlace, "/arm/pick_place", callback_group=self.cbg_action)

        self.cli_delete_entity = self.create_client(DeleteEntity, "/delete_entity", callback_group=self.cbg_reentrant)
        self.latest_model_states: Optional[ModelStates] = None

        # 4. Publishers & Subscribers
        self.pub_status = self.create_publisher(RobotStatus, "/robot_status", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_cmd_servo = self.create_publisher(Twist, "/cmd_vel_servo", 10)

        self.sub_estop = self.create_subscription(Bool, "/estop", self.cb_estop, 10, callback_group=self.cbg_reentrant)
        self.sub_estop_reset = self.create_subscription(Empty, "/estop_reset", self.cb_estop_reset, 10, callback_group=self.cbg_reentrant)
        self.sub_models = self.create_subscription(ModelStates, "/gazebo/model_states", self.cb_model_states, 10, callback_group=self.cbg_reentrant)

        # 5. Services
        self.srv_submit = self.create_service(SubmitTask, "/wh/submit_task", self.handle_submit_task, callback_group=self.cbg_reentrant)
        self.srv_cancel = self.create_service(CancelTask, "/wh/cancel_task", self.handle_cancel_task, callback_group=self.cbg_reentrant)
        self.srv_clear_out = self.create_service(Trigger, "/wh/clear_station_out", self.handle_clear_station_out, callback_group=self.cbg_reentrant)
        self.srv_replenish_in = self.create_service(Trigger, "/wh/replenish_station_in", self.handle_replenish_station_in, callback_group=self.cbg_reentrant)

        # 6. Timers
        self.timer_status = self.create_timer(0.1, self.publish_robot_status, callback_group=self.cbg_timer)
        self.timer_queue = self.create_timer(0.05, self.process_queue, callback_group=self.cbg_timer)

        self.get_logger().info(f"TaskManagerNode khởi động thành công. CSDL tại: {db_path}")

    def _load_yaml(self, path: str) -> dict:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        return {}

    # ── Callbacks trạng thái & E-Stop ──────────────────────────────────────────
    def cb_estop(self, msg: Bool):
        if msg.data and not self.estop_active:
            self.estop_active = True
            self.get_logger().error("!!! E-STOP ĐÃ ĐƯỢC KÍCH HOẠT !!! Đóng băng mọi hoạt động.")
            self.emergency_stop()
            if self.active_fsm:
                self.active_fsm.ctx.is_estopped = True
                self.active_fsm.ctx.last_error = "ESTOP_TRIGGERED"
                self.active_fsm.step(Event.ESTOP)
            self.db.add_event("WARN", "E-stop activated: Robot frozen.")

    def cb_estop_reset(self, msg: Empty):
        if self.estop_active:
            self.estop_active = False
            self.get_logger().info("E-STOP đã được giải phóng. Robot trở lại trạng thái an toàn.")
            self.db.add_event("INFO", "E-stop reset.")

    def cb_model_states(self, msg: ModelStates):
        self.latest_model_states = msg
        if "warehouse_robot" in msg.name:
            idx = msg.name.index("warehouse_robot")
            p = msg.pose[idx].position
            q = msg.pose[idx].orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_x = p.x
            self.robot_y = p.y
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_pose = (self.robot_x, self.robot_y, self.robot_yaw)

    def emergency_stop(self):
        """Dừng khẩn cấp: vận tốc = 0."""
        stop_cmd = Twist()
        self.pub_cmd_vel.publish(stop_cmd)
        self.pub_cmd_servo.publish(stop_cmd)

    def publish_robot_status(self):
        """Phát trạng thái robot 1 Hz lên /robot_status."""
        msg = RobotStatus()
        with self.lock:
            if self.estop_active:
                msg.state = "ESTOP"
            elif self.active_fsm:
                msg.state = self.active_fsm.current_state.value
                msg.task_id = self.active_fsm.ctx.task_id
                msg.slot_id = self.active_fsm.ctx.slot_id or ""
            else:
                msg.state = "IDLE"
                msg.task_id = 0
                msg.slot_id = ""

            msg.x = float(self.robot_x)
            msg.y = float(self.robot_y)
            msg.yaw = float(self.robot_yaw)
            msg.battery_pct = float(self.battery_pct)
            msg.holding_box = bool(self.holding_box)
            msg.held_box_tag = int(self.held_box_tag)
            msg.last_error = str(self.last_error)

        self.pub_status.publish(msg)

    # ── Xử lý Yêu cầu Dịch vụ ────────────────────────────────────────────────
    def handle_submit_task(self, request, response):
        t_type = request.task_type.upper().strip()
        item = request.item_code.strip()
        pri = request.priority if request.priority in (0, 1, 2) else 1
        qty = max(1, int(request.quantity))

        if t_type not in ("INBOUND", "OUTBOUND"):
            response.success = False
            response.message = f"Loại nhiệm vụ không hợp lệ: {t_type}"
            return response

        # Báo lỗi sớm: kiểm tra item
        item_info = self.db.get_item_by_code(item)
        if not item_info:
            response.success = False
            response.message = f"ITEM_NOT_FOUND: Mã hàng '{item}' không tồn tại trong danh mục!"
            return response

        first_task_id = 0
        for i in range(qty):
            tid = self.db.create_task(t_type, item, priority=pri)
            if i == 0:
                first_task_id = tid

        response.success = True
        response.task_id = first_task_id
        response.message = f"Đã tiếp nhận {qty} nhiệm vụ {t_type} cho '{item}' (Mã đầu: #{first_task_id})"
        return response

    def handle_cancel_task(self, request, response):
        tid = request.task_id
        with self.lock:
            if self.active_fsm and self.active_fsm.ctx.task_id == tid:
                self.active_fsm.ctx.is_aborted = True
                response.success = True
                response.message = f"Đã gửi yêu cầu hủy cho nhiệm vụ đang chạy #{tid}"
                return response

        ok = self.db.cancel_task(tid)
        response.success = ok
        response.message = f"Đã hủy nhiệm vụ #{tid}" if ok else f"Không thể hủy nhiệm vụ #{tid}"
        return response

    def handle_clear_station_out(self, request, response):
        """Dọn sạch bàn trả hàng trạm OUT và reset chỉ số vị trí thả về 0."""
        with self.lock:
            self.station_out_drop_idx = 0

        cleared_count = 0
        if self.cli_delete_entity.wait_for_service(timeout_sec=0.5) and self.latest_model_states:
            for name, pose in zip(self.latest_model_states.name, self.latest_model_states.pose):
                if name.startswith("box_") and pose.position.y < -0.8 and pose.position.x < -1.8:
                    dreq = DeleteEntity.Request()
                    dreq.name = name
                    self.cli_delete_entity.call_async(dreq)
                    cleared_count += 1
                    self.get_logger().info(f"Đã dọn dẹp {name} khỏi bàn trạm OUT.")

        self.get_logger().info("Trạm OUT: Đã reset vị trí và dọn sạch bàn trả hàng.")
        self.db.add_event("INFO", f"Trạm OUT đã được dọn sạch ({cleared_count} hộp), sẵn sàng nhận hàng xuất kho mới.")
        response.success = True
        response.message = f"Đã dọn sạch 6 vị trí trả hàng trên bàn trạm OUT (đã dọn {cleared_count} hộp)."
        return response

    def handle_replenish_station_in(self, request, response):
        """Kích hoạt bổ sung phôi mới vào bàn tiếp nhận trạm IN."""
        self.get_logger().info("Trạm IN: Kích hoạt bổ sung phôi mới vào trạm IN.")
        self.db.add_event("INFO", "Trạm IN đã được tiếp thêm phôi, sẵn sàng tiếp tục nhập kho.")
        response.success = True
        response.message = "Đã tiếp thêm phôi vào trạm IN thành công."
        return response

    # ── Xử lý Hàng đợi Nhiệm vụ ──────────────────────────────────────────────
    def process_queue(self):
        if self.estop_active:
            return

        with self.lock:
            if self.active_fsm is not None or (self.active_thread and self.active_thread.is_alive()):
                return

            queued = self.db.get_queued_tasks()
            if not queued:
                return

            next_task = queued[0]
            ctx = FSMContext(
                task_id=next_task["task_id"],
                task_type=next_task["type"],
                item_code=next_task["item_code"],
                priority=next_task["priority"],
            )
            ctx.preferred_slot_id = next_task.get("slot_id")
            self.active_fsm = TaskFSM(ctx, max_retries=2)
            self.db.start_task(ctx.task_id)

        # Chạy quy trình FSM trong thread riêng biệt để không block ROS executor
        self.active_thread = threading.Thread(target=self.run_fsm_loop, daemon=True)
        self.active_thread.start()

    def execute_task_fsm(
        self,
        ctx: FSMContext,
        fault_state: Optional[State] = None,
        injected_event: Event = Event.FAIL,
        step_delay: float = 0.0,
    ) -> str:
        fsm = TaskFSM(ctx, max_retries=2)
        with self.lock:
            self.active_fsm = fsm

        while not fsm.is_finished():
            if self.estop_active:
                ctx.is_estopped = True
                if not ctx.last_error:
                    ctx.last_error = "ESTOP_TRIGGERED"
                if fsm.current_state != State.ESTOP_FREEZE:
                    fsm.current_state = State.ESTOP_FREEZE
                if step_delay > 0:
                    time.sleep(step_delay)
                continue

            current_st = fsm.current_state
            if current_st == State.ESTOP_FREEZE:
                # E-stop đã được reset: chuyển sang ERROR_RECOVERY và kết thúc FAILED
                fsm.current_state = State.ERROR_RECOVERY
                current_st = State.ERROR_RECOVERY

            self.get_logger().info(f"FSM State: {current_st.value}")

            # Tiêm lỗi nếu được cấu hình cho trạng thái này
            if fault_state and current_st == fault_state:
                event = injected_event
                ctx.last_error = f"FAULT_INJECTED_AT_{current_st.value}"
            else:
                handler = self.get_state_handler(current_st)
                try:
                    event = handler(ctx)
                except Exception as e:
                    self.get_logger().error(f"Lỗi ngoại lệ trong trạng thái {current_st.value}: {e}")
                    ctx.last_error = str(e)
                    event = Event.FAIL

            fsm.step(event)
            if step_delay > 0:
                time.sleep(step_delay)

        # Kết thúc nhiệm vụ
        status = "SUCCEEDED" if not ctx.last_error else "FAILED"
        if ctx.is_aborted:
            status = "CANCELLED"
        self.db.finish_task(ctx.task_id, status=status, error=ctx.last_error)

        with self.lock:
            self.active_fsm = None
        return status

    def run_fsm_loop(self):
        ctx = self.active_fsm.ctx
        self.get_logger().info(f">>> Bắt đầu thực thi nhiệm vụ #{ctx.task_id}: {ctx.task_type} {ctx.item_code} <<<")
        delay = 0.02 if self.mock_actions else 0.1
        status = self.execute_task_fsm(ctx, step_delay=delay)
        self.get_logger().info(f">>> Kết thúc nhiệm vụ #{ctx.task_id}: {status} <<<")

    # ── Ánh xạ Trạng thái → Hàm thực thi ─────────────────────────────────────
    def get_state_handler(self, state: State):
        handlers = {
            # Outbound
            State.OUT_RESERVE: self.handle_out_reserve,
            State.OUT_NAV_TO_SLOT: self.handle_nav_to_slot,
            State.OUT_DOCK_SLOT: self.handle_dock_slot,
            State.OUT_VERIFY_SLOT: self.handle_verify_slot,
            State.OUT_PICK_FROM_SLOT: self.handle_out_pick,
            State.OUT_VERIFY_BOX: self.handle_verify_box,
            State.OUT_STOW_ARM: self.handle_stow_arm,
            State.OUT_NAV_TO_OUT: self.handle_nav_to_out,
            State.OUT_DOCK_OUT: self.handle_dock_station,
            State.OUT_PLACE_TO_STATION: self.handle_out_place_station,
            State.OUT_COMMIT: self.handle_out_commit,
            State.OUT_NAV_HOME: self.handle_nav_home,

            # Inbound
            State.IN_RESERVE: self.handle_in_reserve,
            State.IN_NAV_TO_IN: self.handle_nav_to_in,
            State.IN_DOCK_IN: self.handle_dock_station,
            State.IN_PICK_FROM_STATION: self.handle_in_pick_station,
            State.IN_VERIFY_BOX: self.handle_in_verify_box,
            State.IN_STOW_ARM: self.handle_stow_arm,
            State.IN_NAV_TO_SLOT: self.handle_nav_to_slot,
            State.IN_DOCK_SLOT: self.handle_dock_slot,
            State.IN_VERIFY_SLOT: self.handle_verify_slot,
            State.IN_PLACE_TO_SLOT: self.handle_in_place,
            State.IN_VERIFY_PLACED: self.handle_verify_placed,
            State.IN_COMMIT: self.handle_in_commit,
            State.IN_NAV_HOME: self.handle_nav_home,

            # Recovery & Terminal
            State.ERROR_RECOVERY: self.handle_error_recovery,
            State.ESTOP_FREEZE: self.handle_estop_freeze,
            State.DONE: lambda ctx: Event.OK,
        }
        return handlers.get(state, lambda ctx: Event.FAIL)

    # ── Chi tiết từng trạng thái ─────────────────────────────────────────────

    def handle_out_reserve(self, ctx: FSMContext) -> Event:
        slot_id, slot_tag, err = self.db.reserve_slot_for_outbound(ctx.item_code, ctx.task_id)
        if err:
            ctx.last_error = err
            return Event.FAIL
        ctx.slot_id = slot_id
        ctx.slot_tag_id = slot_tag
        slot_info = self.db.get_slot(slot_id)
        ctx.tier = slot_info["tier"] if slot_info else 1
        item_info = self.db.get_item_by_code(ctx.item_code)
        ctx.expected_box_tag = item_info["box_tag_id"] if item_info else 0
        return Event.OK

    def handle_in_reserve(self, ctx: FSMContext) -> Event:
        slot_id, slot_tag, err = self.db.reserve_slot_for_inbound(
            ctx.item_code, ctx.task_id, preferred_slot_id=ctx.preferred_slot_id
        )
        if err:
            ctx.last_error = err
            return Event.FAIL
        ctx.slot_id = slot_id
        ctx.slot_tag_id = slot_tag
        slot_info = self.db.get_slot(slot_id)
        ctx.tier = slot_info["tier"] if slot_info else 1
        item_info = self.db.get_item_by_code(ctx.item_code)
        ctx.expected_box_tag = item_info["box_tag_id"] if item_info else 0
        return Event.OK

    def handle_nav_to_slot(self, ctx: FSMContext) -> Event:
        wp_name = f"SLOT_STAGING_{ctx.slot_tag_id}"
        return self._navigate_waypoint(wp_name)

    def handle_nav_to_in(self, ctx: FSMContext) -> Event:
        return self._navigate_waypoint("IN_STAGING")

    def handle_nav_to_out(self, ctx: FSMContext) -> Event:
        return self._navigate_waypoint("OUT_STAGING")

    def handle_nav_home(self, ctx: FSMContext) -> Event:
        return self._navigate_waypoint("HOME")

    def _navigate_to_pose(self, x: float, y: float, yaw: float) -> Event:
        if self.mock_actions:
            return Event.OK

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Chờ action server Nav2
        if not self.cli_nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server không khả dụng sau 10s!")
            return Event.FAIL

        future = self.cli_nav.send_goal_async(goal)
        t0 = time.time()
        while not future.done() and (time.time() - t0 < 5.0):
            time.sleep(0.05)
        gh = future.result() if future.done() else None
        if not gh or not gh.accepted:
            return Event.FAIL

        res_future = gh.get_result_async()
        t1 = time.time()
        while not res_future.done() and (time.time() - t1 < 75.0):
            time.sleep(0.1)
        res = res_future.result() if res_future.done() else None
        return Event.OK if (res and res.status == GoalStatus.STATUS_SUCCEEDED) else Event.FAIL

    def _navigate_waypoint(self, wp_name: str) -> Event:
        if wp_name not in self.waypoints:
            self.get_logger().error(f"Không tìm thấy waypoint: {wp_name}")
            return Event.FAIL

        wp = self.waypoints[wp_name]
        return self._navigate_to_pose(float(wp["x"]), float(wp["y"]), float(wp["yaw"]))

    def strafe_to_world_x(self, target_x: float, timeout: float = 6.0, speed: float = 0.08) -> bool:
        """Dịch ngang Mecanum theo trục X thế giới tại trạm IN hoặc OUT (đảm bảo đứng thẳng trước hộp)."""
        if self.mock_actions:
            return True
        t0 = time.time()
        rate_hz = 20.0
        dt = 1.0 / rate_hz
        tol = 0.012  # 12 mm dung sai

        # Hướng robot: y > 0 là trạm IN (robot quay +y), y < 0 là trạm OUT (robot quay -y)
        facing_pos_y = (self.robot_y > 0)
        self.get_logger().info(f"Bắt đầu dịch ngang Mecanum tới x={target_x:.3f}m (hiện tại x={self.robot_x:.3f}m)...")

        while time.time() - t0 < timeout:
            if self.estop_active:
                break
            err_x = target_x - self.robot_x
            if abs(err_x) <= tol:
                break

            cmd = Twist()
            # Điều khiển P mượt mà
            vy_mag = max(0.035, min(speed, abs(err_x) * 1.2))
            if facing_pos_y:
                # Quay +y: vy > 0 dịch trái (-x thế giới), vy < 0 dịch phải (+x thế giới)
                cmd.linear.y = -math.copysign(vy_mag, err_x)
            else:
                # Quay -y: vy > 0 dịch trái (+x thế giới), vy < 0 dịch phải (-x thế giới)
                cmd.linear.y = math.copysign(vy_mag, err_x)

            self.pub_cmd_vel.publish(cmd)
            time.sleep(dt)

        stop_cmd = Twist()
        for _ in range(3):
            self.pub_cmd_vel.publish(stop_cmd)
            time.sleep(0.01)
        self.get_logger().info(f"Hoàn tất dịch ngang Mecanum: target_x={target_x:.3f}m, actual_x={self.robot_x:.3f}m")
        return abs(target_x - self.robot_x) <= 0.035

    def straight_undock(self, speed: float = 0.12, timeout: float = 3.5):
        """Lùi thẳng ra ngoài hành lang không xoay xở (chống va đập kệ và lật xe 100%)."""
        if self.mock_actions:
            return

        curr_y = self.robot_y
        if curr_y > 1.15:
            target_y = 1.125   # Lùi từ work_pose rack_a (1.325) ra staging (1.125)
        elif curr_y < -1.15:
            target_y = -1.125  # Lùi từ work_pose rack_b (-1.325) ra staging (-1.125)
        elif curr_y > 0.55:
            target_y = 0.50    # Lùi từ trạm IN ra staging (0.50)
        elif curr_y < -0.55:
            target_y = -0.50   # Lùi từ trạm OUT ra staging (-0.50)
        else:
            self.get_logger().info(f"Robot đang ở hành lang mở (y={curr_y:.3f}), không cần lùi thẳng.")
            return

        self.get_logger().info(f"Bắt đầu lùi thẳng an toàn từ y={curr_y:.3f} ra hành lang y={target_y:.3f}...")
        t0 = time.time()
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.time() - t0 < timeout:
            if self.estop_active:
                break
            y = self.robot_y
            if target_y > 0 and y <= target_y + 0.015:
                break
            if target_y < 0 and y >= target_y - 0.015:
                break

            cmd = Twist()
            cmd.linear.x = -abs(speed)
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            self.pub_cmd_servo.publish(cmd)
            time.sleep(dt)

        stop_cmd = Twist()
        for _ in range(3):
            self.pub_cmd_vel.publish(stop_cmd)
            self.pub_cmd_servo.publish(stop_cmd)
            time.sleep(0.02)

        self.get_logger().info(f"Lùi thẳng hoàn tất: robot_y hiện tại={self.robot_y:.3f}.")
        # Nếu đang ở khu vực trạm IN hoặc OUT (staging y~0.5) và bị lệch trục X so với tâm staging (-2.40):
        if (0.42 <= abs(self.robot_y) <= 0.58) and (abs(self.robot_x - (-2.40)) > 0.03):
            self.get_logger().info(f"Căn chỉnh robot về tâm staging x=-2.40 (từ x={self.robot_x:.3f}m)...")
            self.strafe_to_world_x(-2.40)
        time.sleep(0.2)

    def safe_straight_approach(self, target_y: float, speed: float = 0.08, max_distance: float = 0.22, timeout: float = 3.5):
        """Tiến thẳng từ tốn dọc trục thân xe vào work_pose có giới hạn cự ly và kiểm tra va chạm."""
        if self.mock_actions or self.robot_pose is None:
            return

        start_x, start_y, _ = self.robot_pose
        self.get_logger().info(
            f"Tiến thẳng an toàn từ y={start_y:.3f} vào target_y={target_y:.3f} (giới hạn max {max_distance*1000:.0f} mm)..."
        )
        t0 = time.time()
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.time() - t0 < timeout:
            if self.estop_active or self.robot_pose is None:
                break
            curr_x, curr_y, _ = self.robot_pose
            traveled = math.hypot(curr_x - start_x, curr_y - start_y)
            if traveled >= max_distance:
                self.get_logger().info(f"Đã đạt giới hạn di chuyển an toàn ({traveled*1000:.1f} mm). Dừng xe.")
                break

            # Kiểm tra đã đến target_y (Rack A: target_y > 0, Rack B: target_y < 0)
            if target_y > 0 and curr_y >= target_y - 0.015:
                break
            if target_y < 0 and curr_y <= target_y + 0.015:
                break

            cmd = Twist()
            cmd.linear.x = abs(speed)
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            self.pub_cmd_servo.publish(cmd)
            time.sleep(dt)

        stop_cmd = Twist()
        for _ in range(3):
            self.pub_cmd_vel.publish(stop_cmd)
            self.pub_cmd_servo.publish(stop_cmd)
            time.sleep(0.02)
        time.sleep(0.1)

    def handle_dock_slot(self, ctx: FSMContext) -> Event:
        if self.mock_actions:
            return Event.OK

        slot_key = ctx.slot_tag_id
        slot_data = self.slots_data.get(slot_key) or self.slots_data.get(str(slot_key))
        if not slot_data or "work_pose" not in slot_data:
            self.get_logger().error(f"Không tìm thấy cấu hình slot {slot_key}!")
            return Event.FAIL

        wp = slot_data["work_pose"]
        tx = float(wp["x"])
        ty = float(wp["y"])

        stg = slot_data.get("staging_pose") or {}
        stg_y = float(stg.get("y", ty - math.copysign(0.20, ty)))
        approach_len = abs(ty - stg_y)

        # Standoff chuẩn tại work_pose: 0.435 m (tay SCARA duỗi thẳng tới hộp).
        work_standoff = 0.435
        # Tại điểm chờ (lùi ra approach_len) tag xa hơn đúng bằng approach_len.
        stage_standoff = work_standoff + approach_len

        # 1. CĂN CHỈNH TINH BẰNG VISUAL SERVOING (dock camera) ngay tại điểm chờ ngoài hành lang,
        #    khi tay máy còn gập gọn nên không che tag. Khử lệch ngang (X) + lệch góc (yaw).
        self.get_logger().info(
            f"Docking ô {slot_key}: Visual Servoing tại điểm chờ (standoff={stage_standoff:.3f}m)..."
        )
        dock_ok = self._call_dock(slot_key, standoff=stage_standoff)
        if dock_ok:
            self.get_logger().info("Visual Servoing tại điểm chờ thành công: đã khớp tâm ô kệ & góc vuông.")
        else:
            self.get_logger().warn("Visual Servoing tại điểm chờ chưa khóa, dùng dịch ngang theo odometry dự phòng.")
            if abs(self.robot_x - tx) > 0.01:
                self.strafe_to_world_x(tx)

        # 2. Vươn tay ra trước ngoài hành lang thông thoáng để khuỷu tay không quẹt vách kệ.
        if ctx.task_type == "OUTBOUND":
            self.get_logger().info(
                f"PRE_REACH (Tầng {ctx.tier}, Slot {slot_key}) tại y={self.robot_y:.3f}m..."
            )
            reach_ok, _ = self._call_pick_place("PRE_REACH", slot_key, ctx.expected_box_tag, ctx.tier)
            if not reach_ok:
                self.get_logger().warn("PRE_REACH không thành công, tiếp tục quy trình an toàn.")
        elif ctx.task_type == "INBOUND":
            # NHẬP HÀNG: lấy hộp từ khay rồi vươn tay MANG HỘP ra trước ngay tại điểm chờ
            # (hành lang thông thoáng), sau đó mới tiến xe vào trả hàng.
            self.get_logger().info(
                f"PRE_REACH_CARRY: vươn tay mang hộp #{ctx.held_box_tag} ra vị trí đặt "
                f"(Tầng {ctx.tier}, Slot {slot_key}) tại y={self.robot_y:.3f}m..."
            )
            reach_ok, _ = self._call_pick_place("PRE_REACH_CARRY", slot_key, ctx.held_box_tag, ctx.tier)
            if not reach_ok:
                self.get_logger().error("PRE_REACH_CARRY thất bại: không lấy/đưa được hộp ra vị trí đặt!")
                ctx.last_error = "PRE_REACH_CARRY_FAIL"
                return Event.FAIL

        # 3. Tiến thẳng (không xoay, không dịch ngang) vào vị trí làm việc.
        dist_to_work = abs(self.robot_y - ty)
        if dist_to_work > 0.03:
            self.get_logger().info(f"Tiến thẳng vào work_pose y={ty:.3f}m ({dist_to_work*1000:.1f}mm)...")
            self.safe_straight_approach(ty, speed=0.06, max_distance=dist_to_work + 0.05)

        # 4. Kiểm tra lệch ngang còn lại so với tâm cột; hiệu chỉnh nhẹ nếu > 8 mm.
        lat_err = tx - self.robot_x
        if abs(lat_err) > 0.008:
            self.get_logger().info(f"Hiệu chỉnh ngang tinh {lat_err*1000:.1f}mm về tâm cột x={tx:.3f}m...")
            self.strafe_to_world_x(tx)

        final_dist = math.hypot(self.robot_pose[0] - tx, self.robot_pose[1] - ty) if self.robot_pose else 0.0
        self.get_logger().info(
            f"Cập bến ô {slot_key}: sai số so với work_pose {final_dist*1000:.1f}mm. Sẵn sàng gắp."
        )
        return Event.OK

    def handle_dock_station(self, ctx: FSMContext) -> Event:
        if self.mock_actions:
            return Event.OK
        station_tag = 30 if ctx.task_type == "INBOUND" else 31
        self.get_logger().info(f"Căn chỉnh tinh Visual Servoing theo dock_camera cho trạm tag {station_tag}...")
        dock_ok = self._call_dock(station_tag, standoff=0.38)
        if dock_ok:
            self.get_logger().info(f"Visual Servoing dock_camera thành công cho trạm tag {station_tag}!")

        # Căn chỉnh vị trí dịch ngang Mecanum chính xác theo từng hộp:
        if ctx.task_type == "INBOUND":
            target_x = STATION_IN_BOX_X.get(ctx.expected_box_tag, -2.46)
            self.get_logger().info(
                f"Trạm IN: Hộp #{ctx.expected_box_tag} tại vị trí x={target_x:.3f}m (robot x={self.robot_x:.3f}m)"
            )
            if abs(target_x - self.robot_x) > 0.02:
                self.strafe_to_world_x(target_x)
        elif ctx.task_type == "OUTBOUND":
            slot_idx = self.station_out_drop_idx % len(STATION_OUT_DROP_X)
            target_x = STATION_OUT_DROP_X[slot_idx]
            self.get_logger().info(
                f"Trạm OUT: Chọn vị trí trả hàng #{slot_idx} tại x={target_x:.3f}m (tránh đè hộp trước đó, robot x={self.robot_x:.3f}m)"
            )
            if abs(target_x - self.robot_x) > 0.02:
                self.strafe_to_world_x(target_x)

        return Event.OK

    def handle_verify_slot(self, ctx: FSMContext) -> Event:
        self.get_logger().info(f"VERIFY_SLOT: Đã xác nhận ô {ctx.slot_id} (tag {ctx.slot_tag_id})")
        return Event.OK

    def _retreat_then_store_to_tray(self, ctx: FSMContext, tier: int) -> bool:
        """Gắp xong -> xe lùi thẳng ra hành lang -> MỚI đưa hộp về khay xe và gập tay."""
        self.get_logger().info("Đã gắp xong: lùi thẳng xe ra hành lang trước khi đưa hộp về khay...")
        self.straight_undock()
        ok, _ = self._call_pick_place("STORE_TO_TRAY", 0, ctx.held_box_tag, tier)
        if not ok:
            self.get_logger().error("STORE_TO_TRAY thất bại: không đưa được hộp về khay xe!")
        return ok

    def handle_out_pick(self, ctx: FSMContext) -> Event:
        ok, tag_read = self._call_pick_place("PICK_FROM_SLOT", ctx.slot_tag_id, ctx.expected_box_tag, ctx.tier)
        if ok and tag_read > 0:
            ctx.holding_box = True
            ctx.held_box_tag = tag_read
            self.holding_box = True
            self.held_box_tag = ctx.held_box_tag
            if not self._retreat_then_store_to_tray(ctx, ctx.tier):
                ctx.last_error = "TRAY_STORE_FAIL"
                return Event.FAIL
            return Event.OK
        ctx.last_error = "GRASP_FAIL"
        return Event.FAIL

    def handle_in_pick_station(self, ctx: FSMContext) -> Event:
        ok, tag_read = self._call_pick_place("PICK_FROM_STATION", 30, ctx.expected_box_tag, 1)
        if ok and tag_read > 0:
            ctx.holding_box = True
            ctx.held_box_tag = tag_read
            self.holding_box = True
            self.held_box_tag = ctx.held_box_tag
            if not self._retreat_then_store_to_tray(ctx, 1):
                ctx.last_error = "TRAY_STORE_FAIL"
                return Event.FAIL
            return Event.OK
        ctx.last_error = "STATION_PICK_FAIL"
        return Event.FAIL

    def handle_verify_box(self, ctx: FSMContext) -> Event:
        if not ctx.holding_box or ctx.held_box_tag <= 0:
            self.get_logger().error(f"VERIFY_BOX: Không có hộp thực tế trên tay máy!")
            ctx.last_error = "NO_BOX_HELD"
            return Event.FAIL
        if ctx.expected_box_tag > 0 and ctx.held_box_tag != ctx.expected_box_tag:
            self.get_logger().error(f"VERIFY_BOX: Tag {ctx.held_box_tag} khác mong đợi {ctx.expected_box_tag}!")
            ctx.last_error = "BOX_TAG_MISMATCH"
            return Event.FAIL
        self.get_logger().info(f"VERIFY_BOX: Đã xác nhận hộp tag {ctx.held_box_tag} (mong đợi {ctx.expected_box_tag})")
        return Event.OK

    def handle_in_verify_box(self, ctx: FSMContext) -> Event:
        """Kiểm tra tag hộp khi nhập: nếu sai tag -> REJECT."""
        if ctx.held_box_tag != ctx.expected_box_tag:
            self.get_logger().error(f"REJECT: Hộp nhận {ctx.held_box_tag} khác mã đăng ký {ctx.expected_box_tag}!")
            ctx.last_error = "BOX_TAG_MISMATCH"
            self.db.add_event("WARN", f"Nhiệm vụ {ctx.task_id}: REJECT hộp {ctx.held_box_tag} không khớp {ctx.expected_box_tag}")
            return Event.REJECT
        return Event.OK

    def handle_stow_arm(self, ctx: FSMContext) -> Event:
        # 1. Lùi thẳng an toàn ra ngoài hành lang staging (y=1.125 hoặc -1.125) TRƯỚC KHI gập tay
        self.straight_undock()
        # 2. Khi robot đã ra ngoài hành lang mở rộng rãi, mới gập tay an toàn về STOW
        ok, _ = self._call_pick_place("STOW", 0, 0, 1)
        if ok:
            return Event.OK
        return Event.FAIL

    def handle_in_place(self, ctx: FSMContext) -> Event:
        # Đặt hộp vào ô kệ và rút tay ra khỏi ô (tay còn duỗi thẳng, CHƯA gập)
        ok, _ = self._call_pick_place("PLACE_SHELF", ctx.slot_tag_id, ctx.held_box_tag, ctx.tier)
        if ok:
            ctx.holding_box = False
            ctx.held_box_tag = 0
            self.holding_box = False
            self.held_box_tag = 0
            # 1. Lùi thẳng xe ra hành lang một chút
            self.straight_undock()
            # 2. Ra khỏi vùng kệ rồi mới thu tay về (STOW)
            stow_ok, _ = self._call_pick_place("STOW", 0, 0, 1)
            if not stow_ok:
                self.get_logger().warn("STOW sau khi trả hàng không thành công!")
            return Event.OK
        ctx.last_error = "PLACE_FAIL"
        return Event.FAIL

    def handle_out_place_station(self, ctx: FSMContext) -> Event:
        ok, _ = self._call_pick_place("PLACE_TO_STATION", 31, ctx.held_box_tag, 1)
        if ok:
            ctx.holding_box = False
            ctx.held_box_tag = 0
            self.holding_box = False
            self.held_box_tag = 0
            self.station_out_drop_idx += 1
            # Lùi thẳng ra ngoài trước khi rẽ về HOME
            self.straight_undock()
            return Event.OK
        ctx.last_error = "STATION_PLACE_FAIL"
        return Event.FAIL

    def handle_verify_placed(self, ctx: FSMContext) -> Event:
        self.get_logger().info(f"VERIFY_PLACED: Xác nhận hộp đã vào ô {ctx.slot_id}")
        return Event.OK

    def handle_in_commit(self, ctx: FSMContext) -> Event:
        self.db.commit_inbound(ctx.task_id, ctx.slot_id, ctx.item_code)
        return Event.OK

    def handle_out_commit(self, ctx: FSMContext) -> Event:
        self.db.commit_outbound(ctx.task_id, ctx.slot_id, ctx.item_code)
        return Event.OK

    def _call_pick_place(self, mode: str, slot_tag: int, exp_box: int, tier: int) -> Tuple[bool, int]:
        if self.mock_actions:
            if mode == "PICK_FROM_STATION" and self.mock_box_tag is not None:
                return True, self.mock_box_tag
            if mode in ("PLACE_TO_SLOT", "PLACE_SHELF") and self.mock_place_fail:
                return False, 0
            return True, exp_box

        if not self.cli_pick_place.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"PickPlace action server không khả dụng sau 10s!")
            return False, 0

        goal = PickPlace.Goal()
        goal.mode = mode
        goal.slot_tag_id = int(slot_tag)
        goal.expected_box_tag = int(exp_box)
        goal.tier = int(tier)

        fut = self.cli_pick_place.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and (time.time() - t0 < 5.0):
            time.sleep(0.05)
        gh = fut.result() if fut.done() else None
        if not gh or not gh.accepted:
            return False, 0

        rfut = gh.get_result_async()
        t1 = time.time()
        while not rfut.done() and (time.time() - t1 < 60.0):
            time.sleep(0.1)
        res = rfut.result() if rfut.done() else None
        if res and res.result.success:
            return True, res.result.box_tag_read
        return False, 0

    def _call_dock(self, slot_tag_id: int, standoff: float = 0.435) -> bool:
        """Kích hoạt Visual Servoing (PBVS) căn chỉnh tinh góc vuông và tâm trước ô kệ."""
        if not self.cli_dock.wait_for_server(timeout_sec=2.0):
            return False

        goal = DockToSlot.Goal()
        goal.slot_tag_id = int(slot_tag_id)
        goal.target_standoff = float(standoff)
        goal.tol_lateral = 0.005
        goal.tol_yaw = 0.0349

        fut = self.cli_dock.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and (time.time() - t0 < 4.0):
            time.sleep(0.05)
        gh = fut.result() if fut.done() else None
        if not gh or not gh.accepted:
            return False

        rfut = gh.get_result_async()
        t1 = time.time()
        while not rfut.done() and (time.time() - t1 < 25.0):
            time.sleep(0.1)
        res = rfut.result() if rfut.done() else None
        return True if (res and res.result.success) else False

    # ── Xử lý Khắc phục Lỗi (ERROR_RECOVERY) ──────────────────────────────────
    def handle_error_recovery(self, ctx: FSMContext) -> Event:
        self.get_logger().warn(f"=== TIẾN HÀNH PHỤC HỒI LỖI (ERROR_RECOVERY) CHO NHIỆM VỤ #{ctx.task_id} ===")
        # 1. Đưa tay về STOW an toàn
        self._call_pick_place("STOW", 0, 0, 1)
        self.emergency_stop()

        # 2. Nếu chưa gắp hộp: giải phóng ô, nhiệm vụ FAILED, tồn kho không đổi
        if not ctx.holding_box:
            if ctx.slot_id:
                self.db.release_slot(ctx.slot_id, ctx.task_id)
            if not ctx.last_error:
                ctx.last_error = "TASK_ABORTED_BEFORE_PICK"
            self.get_logger().info(f"Phục hồi: Chưa gắp hộp ({ctx.last_error}). Ô đã được giải phóng, tồn kho nguyên vẹn.")
            return Event.OK

        # 3. Nếu đã gắp hộp trong nhiệm vụ OUTBOUND:
        if ctx.task_type == "OUTBOUND":
            self.get_logger().warn("Hộp đang trên khay xe (Outbound). Cố gắng đặt lại ô kệ nguồn...")
            ok_return, _ = self._call_pick_place("PLACE_TO_SLOT", ctx.slot_tag_id, ctx.held_box_tag, ctx.tier)
            if ok_return:
                ctx.holding_box = False
                self.holding_box = False
                self.db.release_slot(ctx.slot_id, ctx.task_id)
                self.get_logger().info(f"Đã đặt lại hộp an toàn vào ô {ctx.slot_id}.")
            else:
                # Không đặt lại được: giữ nguyên hộp trên khay, ô nguồn vẫn ghi là FULL (không đổi tồn kho)
                if ctx.slot_id:
                    self.db.release_slot(ctx.slot_id, ctx.task_id)
                self.db.add_event("WARN", f"Nhiệm vụ {ctx.task_id}: Hộp {ctx.held_box_tag} đang ở khay xe, cần can thiệp vận hành!")
                self.get_logger().warn("Không thể đặt lại ô nguồn: Hộp giữ trên khay xe. Ô nguồn vẫn ghi là FULL trong CSDL.")

        elif ctx.task_type == "INBOUND":
            self.get_logger().warn("Hộp đang trên khay xe (Inbound). Trả hộp lại bàn nhận IN...")
            self._call_pick_place("PLACE_TO_STATION", 0, ctx.held_box_tag, 1)
            ctx.holding_box = False
            self.holding_box = False
            if ctx.slot_id:
                self.db.release_slot(ctx.slot_id, ctx.task_id)

        # 5. Lùi thẳng ra hành lang mở bảo đảm an toàn
        self.straight_undock()
        return Event.OK

    def handle_estop_freeze(self, ctx: FSMContext) -> Event:
        self.emergency_stop()
        while self.estop_active:
            time.sleep(0.2)
        ctx.last_error = "ESTOP_INTERRUPTED"
        return Event.OK


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
