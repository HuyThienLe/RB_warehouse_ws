"""Mecanum robot forward and inverse kinematics.

Wheel order: (FL, FR, RL, RR)
L = lx + ly (half track width + half wheelbase)
"""

def inverse(vx: float, vy: float, wz: float, r: float, L: float):
    """Calculate 4 wheel angular velocities from chassis velocity (vx, vy, wz)."""
    w_fl = (vx - vy - L * wz) / r
    w_fr = (vx + vy + L * wz) / r
    w_rl = (vx + vy - L * wz) / r
    w_rr = (vx - vy + L * wz) / r
    return (w_fl, w_fr, w_rl, w_rr)

def forward(w: tuple, r: float, L: float):
    """Calculate chassis velocity (vx, vy, wz) from 4 wheel angular velocities."""
    fl, fr, rl, rr = w
    vx = (r / 4.0) * (fl + fr + rl + rr)
    vy = (r / 4.0) * (-fl + fr + rl - rr)
    wz = (r / (4.0 * L)) * (-fl + fr - rl + rr)
    return (vx, vy, wz)
