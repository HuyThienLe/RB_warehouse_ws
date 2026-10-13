#!/usr/bin/env python3
"""scara_kinematics.py – Động học thuận, nghịch và kiểm tra footprint cho tay SCARA TRR.

Cấu hình SCARA TRR:
- Khớp 1 (lift_joint): Tịnh tiến dọc trục Z (d trong [0, z_stroke])
- Khớp 2 (shoulder_joint): Quay trong mặt phẳng ngang quanh trục Z (q1)
- Khớp 3 (elbow_joint): Quay trong mặt phẳng ngang quanh trục Z (q2)

Hệ tọa độ:
- Gốc tính toán trong arm_base_link (tâm trục quay vai đặt tại x_shoulder, y_shoulder).
- l1 = 0.18 m, l2 = 0.18 m
- tool_drop = 0.06 m (đầu kẹp/hút treo dưới mặt phẳng tay máy)
"""

import math
from typing import Optional, Tuple, List, Dict, Any


DEFAULT_L1 = 0.18
DEFAULT_L2 = 0.14
DEFAULT_L3 = 0.04
DEFAULT_TOOL_DROP = 0.06

DEFAULT_LIMITS = {
    "q1": (-math.radians(95.0), math.radians(95.0)),
    "q2": (-math.radians(160.0), math.radians(160.0)),
    "q3": (-math.radians(180.0), math.radians(180.0)),
    "d":  (0.0, 0.52),
}

# Footprint thân xe AMR (chiều dài 0.45 m, chiều rộng 0.32 m tính từ base_link)
# arm_base_link đặt tại x = +0.12 (hoặc +0.15) so với base_link
CHASSIS_X_HALF = 0.45 / 2.0  # 0.225 m
CHASSIS_Y_HALF = 0.32 / 2.0  # 0.160 m
FOOTPRINT_MARGIN = 0.010     # 1 cm biên an toàn


def forward_kinematics(
    q1: float,
    q2: float,
    d: float,
    q3: Optional[float] = None,
    l1: float = DEFAULT_L1,
    l2: float = DEFAULT_L2,
    l3: float = DEFAULT_L3,
    tool_drop: float = DEFAULT_TOOL_DROP,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> Tuple[float, float, float]:
    """Tính động học thuận SCARA TRRR (kèm góc xoay cổ tay q3).
    
    Args:
        q1: Góc khớp vai (rad)
        q2: Góc khớp khuỷu (rad)
        d: Độ nâng lift_joint (m)
        q3: Góc khớp cổ tay (rad), nếu None tự động tính q3 = -(q1 + q2) để đầu kẹp hướng thẳng 0 rad
        l1, l2, l3: Chiều dài các khâu (m)
        tool_drop: Độ rơi đầu công tác dưới mặt phẳng tay (m)
        x_offset, y_offset: Tọa độ tâm vai so với gốc tham chiếu (m)
        
    Returns:
        (x, y, z): Tọa độ điểm thao tác TCP (m)
    """
    if q3 is None:
        q3 = -(q1 + q2)
    x = x_offset + l1 * math.cos(q1) + l2 * math.cos(q1 + q2) + l3 * math.cos(q1 + q2 + q3)
    y = y_offset + l1 * math.sin(q1) + l2 * math.sin(q1 + q2) + l3 * math.sin(q1 + q2 + q3)
    z = d - tool_drop
    return x, y, z


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    phi: float = 0.0,
    l1: float = DEFAULT_L1,
    l2: float = DEFAULT_L2,
    l3: float = DEFAULT_L3,
    tool_drop: float = DEFAULT_TOOL_DROP,
    limits: Dict[str, Tuple[float, float]] = DEFAULT_LIMITS,
    prefer: Optional[Tuple] = None,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> Tuple[Optional[Tuple[float, float, float, float]], str]:
    """Tính động học nghịch SCARA TRRR dạng giải tích với góc hướng đầu kẹp mong muốn phi.
    
    Args:
        x, y, z: Tọa độ mục tiêu cần với tới (m)
        phi: Góc hướng mong muốn của đầu kẹp so với trục robot (rad, mặc định 0.0 hướng thẳng)
        l1, l2, l3: Chiều dài các khâu (m)
        tool_drop: Độ rơi đầu công tác (m)
        limits: Giới hạn các khớp {'q1': (min, max), 'q2': (min, max), 'q3': (min, max), 'd': (min, max)}
        prefer: Tư thế ưu tiên gần nhất (q1_pref, q2_pref, d_pref, ...)
        x_offset, y_offset: Tọa độ tâm vai so với gốc tham chiếu (m)
        
    Returns:
        ((q1, q2, d, q3), status): Nghiệm khớp và mã trạng thái ("OK", "OUT_OF_REACH", "JOINT_LIMIT")
    """
    # Tọa độ tâm khớp cổ tay wrist
    dx = (x - x_offset) - l3 * math.cos(phi)
    dy = (y - y_offset) - l3 * math.sin(phi)
    r2 = dx * dx + dy * dy
    r = math.sqrt(r2)

    # Kiểm tra tầm với hình học tới tâm cổ tay
    reach_max = l1 + l2
    reach_min = abs(l1 - l2)

    if r > (reach_max + 1e-12) or r < (reach_min - 1e-12):
        return None, "OUT_OF_REACH"

    c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c2 = max(-1.0, min(1.0, c2))

    sols: List[Tuple[float, float, float, float]] = []
    d = z + tool_drop

    for sgn in (+1.0, -1.0):
        q2 = sgn * math.acos(c2)
        q1 = math.atan2(dy, dx) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        q1 = math.atan2(math.sin(q1), math.cos(q1))
        
        # Góc cổ tay q3 để đầu kẹp đạt đúng góc phi = q1 + q2 + q3
        q3 = phi - (q1 + q2)
        q3 = math.atan2(math.sin(q3), math.cos(q3))

        q1_lim = limits.get("q1", (-math.pi, math.pi))
        q2_lim = limits.get("q2", (-math.pi, math.pi))
        q3_lim = limits.get("q3", (-math.pi, math.pi))
        d_lim  = limits.get("d",  (0.0, 1.0))

        if (q1_lim[0] - 1e-9 <= q1 <= q1_lim[1] + 1e-9) and \
           (q2_lim[0] - 1e-9 <= q2 <= q2_lim[1] + 1e-9) and \
           (q3_lim[0] - 1e-9 <= q3 <= q3_lim[1] + 1e-9) and \
           (d_lim[0]  - 1e-9 <= d  <= d_lim[1]  + 1e-9):
            sols.append((q1, q2, d, q3))

    if not sols:
        return None, "JOINT_LIMIT"

    if prefer is not None:
        def dist(sol):
            dq1 = math.atan2(math.sin(sol[0] - prefer[0]), math.cos(sol[0] - prefer[0]))
            dq2 = math.atan2(math.sin(sol[1] - prefer[1]), math.cos(sol[1] - prefer[1]))
            dd  = sol[2] - prefer[2]
            return dq1 * dq1 + dq2 * dq2 + 10.0 * dd * dd
        sols.sort(key=dist)
        return sols[0], "OK"
    else:
        sols.sort(key=lambda s: abs(s[1]))
        return sols[0], "OK"


def check_footprint(
    q1: float,
    q2: float,
    arm_base_x: float = 0.12,
    arm_base_y: float = 0.0,
    l1: float = DEFAULT_L1,
    l2: float = DEFAULT_L2,
    chassis_x_half: float = CHASSIS_X_HALF,
    chassis_y_half: float = CHASSIS_Y_HALF,
    margin: float = FOOTPRINT_MARGIN,
) -> bool:
    """Kiểm tra toàn bộ các khâu của tay máy có nằm trọn trong footprint thân xe hay không.
    
    Hình chữ nhật footprint cho phép:
    x_base in [-(chassis_x_half - margin), +(chassis_x_half - margin)]
    y_base in [-(chassis_y_half - margin), +(chassis_y_half - margin)]
    """
    x_max = chassis_x_half - margin
    x_min = -(chassis_x_half - margin)
    y_max = chassis_y_half - margin
    y_min = -(chassis_y_half - margin)

    # Lấy các điểm kiểm tra trên khâu 1 và khâu 2 (chia nhỏ từng đoạn 2 cm)
    points = []
    
    # Khâu vai: tâm quay vai tại arm_base_x + 0.07, arm_base_y
    x_shoulder = arm_base_x + 0.07
    y_shoulder = arm_base_y
    points.append((x_shoulder, y_shoulder))

    # Các điểm dọc Link 1
    num_samples = 10
    for i in range(1, num_samples + 1):
        frac = i / float(num_samples)
        px = x_shoulder + frac * l1 * math.cos(q1)
        py = y_shoulder + frac * l1 * math.sin(q1)
        points.append((px, py))

    # Điểm khớp khuỷu
    x_elbow = x_shoulder + l1 * math.cos(q1)
    y_elbow = y_shoulder + l1 * math.sin(q1)

    # Các điểm dọc Link 2 (kèm cả tool/gripper)
    for i in range(1, num_samples + 1):
        frac = i / float(num_samples)
        px = x_elbow + frac * l2 * math.cos(q1 + q2)
        py = y_elbow + frac * l2 * math.sin(q1 + q2)
        points.append((px, py))

    for px, py in points:
        if not (x_min <= px <= x_max and y_min <= py <= y_max):
            return False

    return True


def generate_cartesian_trajectory(
    start_pos: Tuple[float, float, float],
    end_pos: Tuple[float, float, float],
    num_points: int = 10,
    phi: float = 0.0,
    l1: float = DEFAULT_L1,
    l2: float = DEFAULT_L2,
    l3: float = DEFAULT_L3,
    tool_drop: float = DEFAULT_TOOL_DROP,
    limits: Dict[str, Tuple[float, float]] = DEFAULT_LIMITS,
    prefer: Optional[Tuple] = None,
    x_offset: float = 0.07,
    y_offset: float = 0.0,
) -> Tuple[Optional[List[Tuple[float, float, float, float]]], str]:
    """Sinh quỹ đạo đường thẳng Cartesian từ start_pos tới end_pos qua giải nghịch từng điểm.

    Args:
        start_pos: (x, y, z) điểm bắt đầu trong khung arm_base_link (m)
        end_pos: (x, y, z) điểm kết thúc trong khung arm_base_link (m)
        num_points: Số điểm nội suy trên đường thẳng (tối thiểu 2)
        phi: Góc hướng mong muốn của đầu kẹp (rad, mặc định 0.0 giữ thẳng theo trục xe)
        l1, l2, l3: Chiều dài các khâu (m)
        tool_drop: Độ rơi đầu kẹp (m)
        limits: Giới hạn các khớp
        prefer: Nghiệm ưu tiên ban đầu
        x_offset, y_offset: Tọa độ tâm vai so với arm_base_link (m)

    Returns:
        (trajectory_points, status): Danh sách các bộ nghiệm [(q1, q2, d, q3), ...] và mã trạng thái
    """
    if num_points < 2:
        num_points = 2

    x0, y0, z0 = start_pos
    x1, y1, z1 = end_pos

    points: List[Tuple[float, float, float, float]] = []
    prev_sol = prefer

    for i in range(num_points):
        alpha = float(i) / float(num_points - 1)
        x_curr = x0 + alpha * (x1 - x0)
        y_curr = y0 + alpha * (y1 - y0)
        z_curr = z0 + alpha * (z1 - z0)

        sol, st = inverse_kinematics(
            x_curr, y_curr, z_curr, phi=phi,
            l1=l1, l2=l2, l3=l3, tool_drop=tool_drop,
            limits=limits, prefer=prev_sol,
            x_offset=x_offset, y_offset=y_offset
        )
        if st != "OK" or sol is None:
            return None, f"FAIL_AT_STEP_{i}_{st}"

        points.append(sol)
        prev_sol = sol

    return points, "OK"

