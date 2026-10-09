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
DEFAULT_L2 = 0.18
DEFAULT_TOOL_DROP = 0.06

DEFAULT_LIMITS = {
    "q1": (-math.radians(95.0), math.radians(95.0)),
    "q2": (-math.radians(140.0), math.radians(140.0)),
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
    l1: float = DEFAULT_L1,
    l2: float = DEFAULT_L2,
    tool_drop: float = DEFAULT_TOOL_DROP,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> Tuple[float, float, float]:
    """Tính động học thuận SCARA TRR.
    
    Args:
        q1: Góc khớp vai (rad)
        q2: Góc khớp khuỷu (rad)
        d: Độ nâng lift_joint (m)
        l1, l2: Chiều dài các khâu (m)
        tool_drop: Độ rơi đầu công tác dưới mặt phẳng tay (m)
        x_offset, y_offset: Tọa độ tâm vai so với gốc tham chiếu (m)
        
    Returns:
        (x, y, z): Tọa độ điểm thao tác TCP (m)
    """
    x = x_offset + l1 * math.cos(q1) + l2 * math.cos(q1 + q2)
    y = y_offset + l1 * math.sin(q1) + l2 * math.sin(q1 + q2)
    z = d - tool_drop
    return x, y, z


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    l1: float = DEFAULT_L1,
    l2: float = DEFAULT_L2,
    tool_drop: float = DEFAULT_TOOL_DROP,
    limits: Dict[str, Tuple[float, float]] = DEFAULT_LIMITS,
    prefer: Optional[Tuple[float, float, float]] = None,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> Tuple[Optional[Tuple[float, float, float]], str]:
    """Tính động học nghịch SCARA TRR dạng giải tích (Closed-form).
    
    Args:
        x, y, z: Tọa độ mục tiêu cần với tới (m)
        l1, l2: Chiều dài các khâu (m)
        tool_drop: Độ rơi đầu công tác (m)
        limits: Giới hạn các khớp {'q1': (min, max), 'q2': (min, max), 'd': (min, max)}
        prefer: Tư thế ưu tiên gần nhất (q1_pref, q2_pref, d_pref)
        x_offset, y_offset: Tọa độ tâm vai so với gốc tham chiếu (m)
        
    Returns:
        ((q1, q2, d), status): Nghiệm khớp và mã trạng thái ("OK", "OUT_OF_REACH", "JOINT_LIMIT")
    """
    dx = x - x_offset
    dy = y - y_offset
    r2 = dx * dx + dy * dy
    r = math.sqrt(r2)

    # Kiểm tra tầm với hình học
    reach_max = l1 + l2
    reach_min = abs(l1 - l2)

    # Dung sai số học dấu phẩy động 1e-12
    if r > (reach_max + 1e-12) or r < (reach_min - 1e-12):
        return None, "OUT_OF_REACH"

    # Giới hạn cos(q2) trong [-1.0, 1.0] tránh lỗi số học
    c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c2 = max(-1.0, min(1.0, c2))

    sols: List[Tuple[float, float, float]] = []
    d = z + tool_drop

    # Hai nghiệm khuỷu (sgn = +1 khuỷu cong trái, sgn = -1 khuỷu cong phải)
    for sgn in (+1.0, -1.0):
        q2 = sgn * math.acos(c2)
        q1 = math.atan2(dy, dx) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        
        # Chuẩn hóa q1 trong [-pi, pi]
        q1 = math.atan2(math.sin(q1), math.cos(q1))

        # Kiểm tra giới hạn khớp
        q1_lim = limits.get("q1", (-math.pi, math.pi))
        q2_lim = limits.get("q2", (-math.pi, math.pi))
        d_lim  = limits.get("d",  (0.0, 1.0))

        if (q1_lim[0] - 1e-9 <= q1 <= q1_lim[1] + 1e-9) and \
           (q2_lim[0] - 1e-9 <= q2 <= q2_lim[1] + 1e-9) and \
           (d_lim[0]  - 1e-9 <= d  <= d_lim[1]  + 1e-9):
            sols.append((q1, q2, d))

    if not sols:
        return None, "JOINT_LIMIT"

    # Chọn nghiệm tối ưu:
    if prefer is not None:
        # Chọn nghiệm có khoảng cách góc nhỏ nhất so với prefer
        def dist(sol):
            dq1 = math.atan2(math.sin(sol[0] - prefer[0]), math.cos(sol[0] - prefer[0]))
            dq2 = math.atan2(math.sin(sol[1] - prefer[1]), math.cos(sol[1] - prefer[1]))
            dd  = sol[2] - prefer[2]
            return dq1 * dq1 + dq2 * dq2 + 10.0 * dd * dd
        sols.sort(key=dist)
        return sols[0], "OK"
    else:
        # Mặc định ưu tiên nghiệm khuỷu cong ra phía ngoài (q2 > 0 hoặc độ lớn nhỏ)
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
