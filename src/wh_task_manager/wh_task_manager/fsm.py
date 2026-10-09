#!/usr/bin/env python3
"""fsm.py – Máy trạng thái hữu hạn (FSM) cho điều phối nhiệm vụ Nhập/Xuất kho (Bài 7.2).

Dạng bảng chuyển trạng thái (Table-driven State Machine):
- Mỗi trạng thái trả về Event (OK, FAIL, TIMEOUT, ABORT, REJECT, ESTOP).
- Bảng transitions[(state, event)] = next_state.
- Quy tắc phục hồi lỗi:
  + Thử lại tối đa MAX_RETRIES = 2 lần cho từng trạng thái có thể thử lại.
  + Nếu thất bại vượt quá số lần thử -> ERROR_RECOVERY.
  + ERROR_RECOVERY xử lý thu hồi ô kệ, hạ/cất tay an toàn, bảo toàn 100% dữ liệu tồn kho.
"""

from enum import Enum
from typing import Dict, Tuple, Optional, Any, Callable


class Event(str, Enum):
    OK = "OK"
    FAIL = "FAIL"
    TIMEOUT = "TIMEOUT"
    ABORT = "ABORT"
    RETRY = "RETRY"
    REJECT = "REJECT"
    ESTOP = "ESTOP"


class State(str, Enum):
    IDLE = "IDLE"

    # OUTBOUND sequence
    OUT_RESERVE = "OUT_RESERVE"
    OUT_NAV_TO_SLOT = "OUT_NAV_TO_SLOT"
    OUT_DOCK_SLOT = "OUT_DOCK_SLOT"
    OUT_VERIFY_SLOT = "OUT_VERIFY_SLOT"
    OUT_PICK_FROM_SLOT = "OUT_PICK_FROM_SLOT"
    OUT_VERIFY_BOX = "OUT_VERIFY_BOX"
    OUT_STOW_ARM = "OUT_STOW_ARM"
    OUT_NAV_TO_OUT = "OUT_NAV_TO_OUT"
    OUT_DOCK_OUT = "OUT_DOCK_OUT"
    OUT_PLACE_TO_STATION = "OUT_PLACE_TO_STATION"
    OUT_COMMIT = "OUT_COMMIT"
    OUT_NAV_HOME = "OUT_NAV_HOME"

    # INBOUND sequence
    IN_RESERVE = "IN_RESERVE"
    IN_NAV_TO_IN = "IN_NAV_TO_IN"
    IN_DOCK_IN = "IN_DOCK_IN"
    IN_PICK_FROM_STATION = "IN_PICK_FROM_STATION"
    IN_VERIFY_BOX = "IN_VERIFY_BOX"
    IN_STOW_ARM = "IN_STOW_ARM"
    IN_NAV_TO_SLOT = "IN_NAV_TO_SLOT"
    IN_DOCK_SLOT = "IN_DOCK_SLOT"
    IN_VERIFY_SLOT = "IN_VERIFY_SLOT"
    IN_PLACE_TO_SLOT = "IN_PLACE_TO_SLOT"
    IN_VERIFY_PLACED = "IN_VERIFY_PLACED"
    IN_COMMIT = "IN_COMMIT"
    IN_NAV_HOME = "IN_NAV_HOME"

    # Error recovery & terminal
    ERROR_RECOVERY = "ERROR_RECOVERY"
    ESTOP_FREEZE = "ESTOP_FREEZE"
    DONE = "DONE"


class FSMContext:
    """Ngữ cảnh thực thi của nhiệm vụ."""

    def __init__(
        self,
        task_id: int,
        task_type: str,
        item_code: str,
        priority: int = 1,
    ):
        self.task_id = task_id
        self.task_type = task_type.upper()  # INBOUND hoặc OUTBOUND
        self.item_code = item_code
        self.priority = priority

        self.slot_id: Optional[str] = None
        self.slot_tag_id: Optional[int] = None
        self.tier: int = 1
        self.expected_box_tag: Optional[int] = None
        self.box_tag_read: Optional[int] = None

        self.holding_box: bool = False
        self.held_box_tag: int = 0

        self.retries: Dict[State, int] = {}
        self.last_error: str = ""
        self.is_aborted: bool = False
        self.is_estopped: bool = False

    def can_retry(self, state: State, max_retries: int = 2) -> bool:
        current = self.retries.get(state, 0)
        if current < max_retries:
            self.retries[state] = current + 1
            return True
        return False


def build_transition_table() -> Dict[Tuple[State, Event], State]:
    """Khởi tạo bảng chuyển trạng thái chuẩn xác theo đặc tả Bài 7.2."""
    t: Dict[Tuple[State, Event], State] = {}

    # ── OUTBOUND CHUYỂN TRẠNG THÁI ──────────────────────────────────────────
    t[(State.OUT_RESERVE, Event.OK)] = State.OUT_NAV_TO_SLOT
    t[(State.OUT_NAV_TO_SLOT, Event.OK)] = State.OUT_DOCK_SLOT
    t[(State.OUT_DOCK_SLOT, Event.OK)] = State.OUT_VERIFY_SLOT
    t[(State.OUT_VERIFY_SLOT, Event.OK)] = State.OUT_PICK_FROM_SLOT
    t[(State.OUT_PICK_FROM_SLOT, Event.OK)] = State.OUT_VERIFY_BOX
    t[(State.OUT_VERIFY_BOX, Event.OK)] = State.OUT_STOW_ARM
    t[(State.OUT_STOW_ARM, Event.OK)] = State.OUT_NAV_TO_OUT
    t[(State.OUT_NAV_TO_OUT, Event.OK)] = State.OUT_DOCK_OUT
    t[(State.OUT_DOCK_OUT, Event.OK)] = State.OUT_PLACE_TO_STATION
    t[(State.OUT_PLACE_TO_STATION, Event.OK)] = State.OUT_COMMIT
    t[(State.OUT_COMMIT, Event.OK)] = State.OUT_NAV_HOME
    t[(State.OUT_NAV_HOME, Event.OK)] = State.DONE

    # ── INBOUND CHUYỂN TRẠNG THÁI ───────────────────────────────────────────
    t[(State.IN_RESERVE, Event.OK)] = State.IN_NAV_TO_IN
    t[(State.IN_NAV_TO_IN, Event.OK)] = State.IN_DOCK_IN
    t[(State.IN_DOCK_IN, Event.OK)] = State.IN_PICK_FROM_STATION
    t[(State.IN_PICK_FROM_STATION, Event.OK)] = State.IN_VERIFY_BOX
    t[(State.IN_VERIFY_BOX, Event.OK)] = State.IN_STOW_ARM
    t[(State.IN_STOW_ARM, Event.OK)] = State.IN_NAV_TO_SLOT
    t[(State.IN_NAV_TO_SLOT, Event.OK)] = State.IN_DOCK_SLOT
    t[(State.IN_DOCK_SLOT, Event.OK)] = State.IN_VERIFY_SLOT
    t[(State.IN_VERIFY_SLOT, Event.OK)] = State.IN_PLACE_TO_SLOT
    t[(State.IN_PLACE_TO_SLOT, Event.OK)] = State.IN_VERIFY_PLACED
    t[(State.IN_VERIFY_PLACED, Event.OK)] = State.IN_COMMIT
    t[(State.IN_COMMIT, Event.OK)] = State.IN_NAV_HOME
    t[(State.IN_NAV_HOME, Event.OK)] = State.DONE

    # Xử lý REJECT (nhập sai tag hộp)
    t[(State.IN_VERIFY_BOX, Event.REJECT)] = State.ERROR_RECOVERY

    # Mọi lỗi (FAIL, TIMEOUT, ABORT) khi hết lượt thử đều chuyển sang ERROR_RECOVERY
    all_states = list(State)
    for st in all_states:
        if st not in (State.IDLE, State.DONE, State.ERROR_RECOVERY, State.ESTOP_FREEZE):
            t[(st, Event.FAIL)] = State.ERROR_RECOVERY
            t[(st, Event.TIMEOUT)] = State.ERROR_RECOVERY
            t[(st, Event.ABORT)] = State.ERROR_RECOVERY
            t[(st, Event.ESTOP)] = State.ESTOP_FREEZE

    t[(State.ERROR_RECOVERY, Event.OK)] = State.DONE
    t[(State.ERROR_RECOVERY, Event.FAIL)] = State.DONE
    t[(State.ESTOP_FREEZE, Event.OK)] = State.ERROR_RECOVERY

    return t


class TaskFSM:
    """Quản lý chu trình chạy FSM dựa trên bảng chuyển trạng thái."""

    def __init__(self, context: FSMContext, max_retries: int = 2):
        self.ctx = context
        self.max_retries = max_retries
        self.transitions = build_transition_table()
        self.current_state = (
            State.IN_RESERVE if self.ctx.task_type == "INBOUND" else State.OUT_RESERVE
        )
        self.history: list = [(self.current_state, "START")]

    def step(self, event: Event) -> State:
        """Thực hiện một bước chuyển trạng thái dựa trên sự kiện trả về."""
        # Kiểm tra thử lại nếu gặp lỗi có thể retry
        if event in (Event.FAIL, Event.TIMEOUT):
            if self.ctx.can_retry(self.current_state, self.max_retries):
                event = Event.RETRY

        if event == Event.RETRY:
            next_state = self.current_state  # Lặp lại trạng thái hiện tại
        else:
            key = (self.current_state, event)
            if key in self.transitions:
                next_state = self.transitions[key]
            else:
                next_state = State.ERROR_RECOVERY

        self.history.append((next_state, event.value))
        self.current_state = next_state
        return self.current_state

    def is_finished(self) -> bool:
        return self.current_state == State.DONE
