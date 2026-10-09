import pytest
import random
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from wh_description.mecanum_kinematics import inverse, forward

R = 0.05
L = 0.15 + 0.19  # 0.34 m

def test_random_velocities_roundtrip():
    random.seed(42)
    for _ in range(1000):
        vx = random.uniform(-1.5, 1.5)
        vy = random.uniform(-1.5, 1.5)
        wz = random.uniform(-3.0, 3.0)
        
        w = inverse(vx, vy, wz, R, L)
        vx_rec, vy_rec, wz_rec = forward(w, R, L)
        
        assert math.isclose(vx, vx_rec, abs_tol=1e-9), f"vx mismatch: {vx} vs {vx_rec}"
        assert math.isclose(vy, vy_rec, abs_tol=1e-9), f"vy mismatch: {vy} vs {vy_rec}"
        assert math.isclose(wz, wz_rec, abs_tol=1e-9), f"wz mismatch: {wz} vs {wz_rec}"

def test_straight_forward():
    vx = 0.5
    vy = 0.0
    wz = 0.0
    w = inverse(vx, vy, wz, R, L)
    w_expected = vx / R
    assert w == (w_expected, w_expected, w_expected, w_expected)
    rec = forward(w, R, L)
    assert rec == (vx, vy, wz)

def test_lateral_move():
    vx = 0.0
    vy = 0.1
    wz = 0.0
    w_fl, w_fr, w_rl, w_rr = inverse(vx, vy, wz, R, L)
    # vy = +0.1: FR, RL quay thuận (+); FL, RR quay ngược (-)
    assert w_fl < 0
    assert w_rr < 0
    assert w_fr > 0
    assert w_rl > 0
    assert math.isclose(abs(w_fl), abs(w_fr), abs_tol=1e-9)
    assert math.isclose(abs(w_fl), abs(w_rl), abs_tol=1e-9)
    assert math.isclose(abs(w_fl), abs(w_rr), abs_tol=1e-9)

def test_pure_rotation():
    vx = 0.0
    vy = 0.0
    wz = 1.0
    w_fl, w_fr, w_rl, w_rr = inverse(vx, vy, wz, R, L)
    # wz > 0: right wheels forward (+), left wheels backward (-)
    assert w_fl < 0
    assert w_rl < 0
    assert w_fr > 0
    assert w_rr > 0
