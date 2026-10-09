"""Camera calibration and intrinsics utilities."""
import math

def compute_intrinsics(width: int = 640, height: int = 480, hfov_rad: float = 1.2):
    """
    Compute intrinsic matrix parameters:
    fx = fy = (W / 2) / tan(hfov / 2)
    cx = W / 2
    cy = H / 2
    """
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    k_matrix = [
        fx,  0.0, cx,
        0.0, fy,  cy,
        0.0, 0.0, 1.0
    ]
    return {
        "width": width,
        "height": height,
        "hfov": hfov_rad,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "k": k_matrix
    }

if __name__ == "__main__":
    for hfov in [1.0, 1.2]:
        res = compute_intrinsics(640, 480, hfov)
        fx_val = res["fx"]
        fy_val = res["fy"]
        cx_val = res["cx"]
        cy_val = res["cy"]
        print(f"hfov: {hfov:.2f} rad -> fx={fx_val:.2f}, fy={fy_val:.2f}, cx={cx_val}, cy={cy_val}")
