#!/usr/bin/env python3
"""test_scara_kinematics.py – Bộ kiểm thử Pytest định lượng cho động học SCARA TRR."""

import math
import random
import pytest
from wh_manipulation.scara_kinematics import (
    forward_kinematics,
    inverse_kinematics,
    check_footprint,
    DEFAULT_L1,
    DEFAULT_L2,
    DEFAULT_L3,
    DEFAULT_TOOL_DROP,
    DEFAULT_LIMITS,
)


def test_roundtrip_1000_random_reachable_points():
    """Kiểm thử thuận o nghịch với 1000 điểm ngẫu nhiên trong vùng làm việc: sai số < 1e-9."""
    random.seed(1337)
    l1 = DEFAULT_L1
    l2 = DEFAULT_L2
    tool_drop = DEFAULT_TOOL_DROP

    q1_min, q1_max = DEFAULT_LIMITS["q1"]
    q2_min, q2_max = DEFAULT_LIMITS["q2"]
    d_min, d_max = DEFAULT_LIMITS["d"]

    tested = 0
    while tested < 1000:
        # Chọn góc ngẫu nhiên trong giới hạn
        q1_gen = random.uniform(q1_min + 0.05, q1_max - 0.05)
        q2_gen = random.uniform(q2_min + 0.05, q2_max - 0.05)
        d_gen = random.uniform(d_min + 0.01, d_max - 0.01)

        # Tính FK
        x, y, z = forward_kinematics(q1_gen, q2_gen, d_gen, l1=l1, l2=l2, tool_drop=tool_drop)

        # Tính IK với prefer là góc ban đầu
        sol, status = inverse_kinematics(
            x, y, z, phi=0.0, l1=l1, l2=l2, tool_drop=tool_drop, limits=DEFAULT_LIMITS, prefer=(q1_gen, q2_gen, d_gen, -(q1_gen + q2_gen))
        )

        assert status == "OK", f"IK thất bại tại ({x}, {y}, {z}): {status}"
        assert sol is not None
        q1_calc, q2_calc, d_calc, q3_calc = sol

        # Kiểm tra FK ngược lại từ nghiệm tính toán
        x_rec, y_rec, z_rec = forward_kinematics(q1_calc, q2_calc, d_calc, q3=q3_calc, l1=l1, l2=l2, tool_drop=tool_drop)

        err_x = abs(x_rec - x)
        err_y = abs(y_rec - y)
        err_z = abs(z_rec - z)

        assert err_x < 1e-9, f"Sai số x={err_x} >= 1e-9"
        assert err_y < 1e-9, f"Sai số y={err_y} >= 1e-9"
        assert err_z < 1e-9, f"Sai số z={err_z} >= 1e-9"

        # Sai số khớp so với góc sinh ban đầu
        dq1 = math.atan2(math.sin(q1_calc - q1_gen), math.cos(q1_calc - q1_gen))
        dq2 = math.atan2(math.sin(q2_calc - q2_gen), math.cos(q2_calc - q2_gen))
        dd = abs(d_calc - d_gen)

        assert abs(dq1) < 1e-7, f"Sai số q1={dq1}"
        assert abs(dq2) < 1e-7, f"Sai số q2={dq2}"
        assert dd < 1e-9, f"Sai số d={dd}"

        tested += 1


def test_boundary_reach_max():
    """Kiểm thử điểm biên ngoài cùng r = l1 + l2 + l3 (tay duỗi thẳng q2 = 0)."""
    l1 = DEFAULT_L1
    l2 = DEFAULT_L2
    l3 = DEFAULT_L3
    r_max = l1 + l2 + l3
    x = r_max
    y = 0.0
    z = 0.20

    sol, status = inverse_kinematics(x, y, z, phi=0.0, l1=l1, l2=l2, l3=l3)
    assert status == "OK"
    assert sol is not None
    q1, q2, d, q3 = sol
    assert abs(q1) < 1e-7
    assert abs(q2) < 1e-7


def test_boundary_reach_min():
    """Kiểm thử điểm biên trong cùng r = |l1 - l2| = 0 (tay gập hoàn toàn q2 = pi hoặc -pi)."""
    l1 = DEFAULT_L1
    l2 = DEFAULT_L2
    x = 0.0
    y = 0.0
    z = 0.20

    # Khi l1 == l2, r = 0 thì c2 = -1 -> q2 = +/- pi
    # Vì giới hạn q2 là +/- 140 deg (+/- 2.44 rad) nên q2 = pi vượt giới hạn khớp
    sol, status = inverse_kinematics(x, y, z, l1, l2)
    assert status == "JOINT_LIMIT" or status == "OK"


def test_out_of_reach():
    """Điểm vượt quá tầm với r > l1 + l2 phải trả về OUT_OF_REACH."""
    l1 = DEFAULT_L1
    l2 = DEFAULT_L2
    # r = 0.40 > 0.36
    sol, status = inverse_kinematics(0.40, 0.0, 0.20, l1, l2)
    assert status == "OUT_OF_REACH"
    assert sol is None

    # r = 0.50
    sol, status = inverse_kinematics(0.30, 0.40, 0.20, l1, l2)
    assert status == "OUT_OF_REACH"
    assert sol is None


def test_joint_limits():
    """Điểm vượt quá giới hạn z hoặc góc quay phải báo JOINT_LIMIT."""
    l1 = DEFAULT_L1
    l2 = DEFAULT_L2
    # d = z + tool_drop = -0.10 + 0.06 = -0.04 < 0.0
    sol, status = inverse_kinematics(0.25, 0.0, -0.10, l1, l2)
    assert status == "JOINT_LIMIT"
    assert sol is None

    # d = 0.60 + 0.06 = 0.66 > 0.52
    sol, status = inverse_kinematics(0.25, 0.0, 0.60, l1, l2)
    assert status == "JOINT_LIMIT"
    assert sol is None


def test_stow_footprint():
    """Tư thế STOW gấp gọn phải nằm 100% trong footprint thân xe."""
    # Tư thế stowed gấp gọn về sau: q1 = -2.60 rad (-149 deg), q2 = -2.40 rad (-137.5 deg)
    is_inside = check_footprint(q1=-2.60, q2=-2.40)
    assert is_inside is True, "Tư thế STOW phải nằm gọn trong footprint thân xe"


def test_cartesian_straight_line_trajectory():
    """Kiểm thử sinh quỹ đạo đường thẳng ngang Cartesian luồn vào ô kệ."""
    from wh_manipulation.scara_kinematics import generate_cartesian_trajectory
    p_start = (0.20, 0.0, -0.042)
    p_end = (0.31, 0.005, -0.042)
    traj, status = generate_cartesian_trajectory(p_start, p_end, num_points=12, x_offset=0.07, y_offset=0.0)
    assert status == "OK"
    assert traj is not None
    assert len(traj) == 12

    # Kiểm tra sai số bám đường thẳng của từng điểm
    for i, (q1, q2, d, q3) in enumerate(traj):
        alpha = i / 11.0
        exp_x = p_start[0] + alpha * (p_end[0] - p_start[0])
        exp_y = p_start[1] + alpha * (p_end[1] - p_start[1])
        exp_z = p_start[2]
        fk_x, fk_y, fk_z = forward_kinematics(q1, q2, d, q3=q3, x_offset=0.07, y_offset=0.0)
        assert abs(fk_x - exp_x) < 1e-4
        assert abs(fk_y - exp_y) < 1e-4
        assert abs(fk_z - exp_z) < 1e-4


