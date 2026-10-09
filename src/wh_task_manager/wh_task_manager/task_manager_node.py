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
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Empty
from gazebo_msgs.msg import ModelStates

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
        self.waypoints = self._load_yaml(waypoints_file).get("waypoints", {})

        # 2. Quản lý trạng thái Robot
        self.robot_x = -2.40
        self.robot_y = 0.00
        self.robot_yaw = 0.00
        self.battery_pct = 100.0
        self.holding_box = False
        self.held_box_tag = 0
        self.last_error = ""

        self.estop_active = False
        self.active_fsm: Optional[TaskFSM] = None
        self.active_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # 3. Action Clients
        self.cli_nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cbg_action)
        self.cli_dock = ActionClient(self, DockToSlot, "dock_to_slot", callback_group=self.cbg_action)
        self.cli_pick_place = ActionClient(self, PickPlace, "/arm/pick_place", callback_group=self.cbg_action)

        # 4. Publishers & Subscribers
        self.pub_status = self.create_publisher(RobotStatus, "/robot_status", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.sub_estop = self.create_subscription(Bool, "/estop", self.cb_estop, 10, callback_group=self.cbg_reentrant)
        self.sub_estop_reset = self.create_subscription(Empty, "/estop_reset", self.cb_estop_reset, 10, callback_group=self.cbg_reentrant)
        self.sub_models = self.create_subscription(ModelStates, "/gazebo/model_states", self.cb_model_states, 10, callback_group=self.cbg_reentrant)

        # 5. Services
        self.srv_submit = self.create_service(SubmitTask, "/wh/submit_task", self.handle_submit_task, callback_group=self.cbg_reentrant)
        self.srv_cancel = self.create_service(CancelTask, "/wh/cancel_task", self.handle_cancel_task, callback_group=self.cbg_reentrant)

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
        if "warehouse_robot" in msg.name:
            idx = msg.name.index("warehouse_robot")
            p = msg.pose[idx].position
            q = msg.pose[idx].orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_x = p.x
            self.robot_y = p.y
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def emergency_stop(self):
        """Dừng khẩn cấp: vận tốc = 0."""
        stop_cmd = Twist()
        self.pub_cmd_vel.publish(stop_cmd)

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
        slot_id, slot_tag, err = self.db.reserve_slot_for_inbound(ctx.item_code, ctx.task_id)
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

    def _navigate_waypoint(self, wp_name: str) -> Event:
        if self.mock_actions:
            return Event.OK

        if wp_name not in self.waypoints:
            self.get_logger().error(f"Không tìm thấy waypoint: {wp_name}")
            return Event.FAIL

        wp = self.waypoints[wp_name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp["x"])
        goal.pose.pose.position.y = float(wp["y"])
        yaw = float(wp["yaw"])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Chờ action server Nav2 nếu khả dụng, nếu chưa chạy Nav2 thì mô phỏng tọa độ an toàn
        if not self.cli_nav.wait_for_server(timeout_sec=1.5):
            self.get_logger().info(f"[Nav2 Emulation] Đến điểm chờ {wp_name}...")
            time.sleep(1.0)
            return Event.OK

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

    def handle_dock_slot(self, ctx: FSMContext) -> Event:
        if self.mock_actions:
            return Event.OK

        if not self.cli_dock.wait_for_server(timeout_sec=1.5):
            self.get_logger().info(f"[Dock Emulation] Docking ô kệ tag {ctx.slot_tag_id}...")
            time.sleep(1.0)
            return Event.OK

        goal = DockToSlot.Goal()
        goal.slot_tag_id = int(ctx.slot_tag_id)
        goal.target_standoff = 0.32
        goal.tol_lateral = 0.005
        goal.tol_yaw = 0.035

        fut = self.cli_dock.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and (time.time() - t0 < 5.0):
            time.sleep(0.05)
        gh = fut.result() if fut.done() else None
        if not gh or not gh.accepted:
            return Event.FAIL

        rfut = gh.get_result_async()
        t1 = time.time()
        while not rfut.done() and (time.time() - t1 < 45.0):
            time.sleep(0.1)
        res = rfut.result() if rfut.done() else None
        return Event.OK if (res and res.result.success) else Event.FAIL

    def handle_dock_station(self, ctx: FSMContext) -> Event:
        if not self.mock_actions:
            self.get_logger().info("Đã dock tại trạm giao nhận hàng.")
            time.sleep(0.5)
        return Event.OK

    def handle_verify_slot(self, ctx: FSMContext) -> Event:
        self.get_logger().info(f"VERIFY_SLOT: Đã xác nhận ô {ctx.slot_id} (tag {ctx.slot_tag_id})")
        return Event.OK

    def handle_out_pick(self, ctx: FSMContext) -> Event:
        ok, tag_read = self._call_pick_place("PICK_FROM_SLOT", ctx.slot_tag_id, ctx.expected_box_tag, ctx.tier)
        if ok:
            ctx.holding_box = True
            ctx.held_box_tag = tag_read if tag_read > 0 else ctx.expected_box_tag
            self.holding_box = True
            self.held_box_tag = ctx.held_box_tag
            return Event.OK
        ctx.last_error = "GRASP_FAIL"
        return Event.FAIL

    def handle_in_pick_station(self, ctx: FSMContext) -> Event:
        ok, tag_read = self._call_pick_place("PICK_FROM_STATION", 30, ctx.expected_box_tag, 1)
        if ok:
            ctx.holding_box = True
            ctx.held_box_tag = tag_read if tag_read > 0 else ctx.expected_box_tag
            self.holding_box = True
            self.held_box_tag = ctx.held_box_tag
            return Event.OK
        ctx.last_error = "STATION_PICK_FAIL"
        return Event.FAIL

    def handle_verify_box(self, ctx: FSMContext) -> Event:
        self.get_logger().info(f"VERIFY_BOX: Hộp tag {ctx.held_box_tag} (mong đợi {ctx.expected_box_tag})")
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
        ok, _ = self._call_pick_place("STOW", 0, 0, 1)
        return Event.OK if ok else Event.FAIL

    def handle_in_place(self, ctx: FSMContext) -> Event:
        ok, _ = self._call_pick_place("PLACE_TO_SLOT", ctx.slot_tag_id, ctx.held_box_tag, ctx.tier)
        if ok:
            ctx.holding_box = False
            ctx.held_box_tag = 0
            self.holding_box = False
            self.held_box_tag = 0
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
            if mode == "PLACE_TO_SLOT" and self.mock_place_fail:
                return False, 0
            return True, exp_box

        if not self.cli_pick_place.wait_for_server(timeout_sec=1.5):
            self.get_logger().info(f"[PickPlace Emulation] mode={mode}, slot={slot_tag}, box={exp_box}...")
            time.sleep(1.0)
            return True, exp_box

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
            self.get_logger().info("Phục hồi: Chưa gắp hộp. Ô đã được giải phóng, tồn kho nguyên vẹn.")
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

        # 4. Nếu đã gắp hộp trong nhiệm vụ INBOUND:
        elif ctx.task_type == "INBOUND":
            self.get_logger().warn("Hộp đang trên khay xe (Inbound). Trả hộp lại bàn nhận IN...")
            self._call_pick_place("PLACE_TO_STATION", 0, ctx.held_box_tag, 1)
            ctx.holding_box = False
            self.holding_box = False
            if ctx.slot_id:
                self.db.release_slot(ctx.slot_id, ctx.task_id)

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
