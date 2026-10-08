#!/usr/bin/env bash
set -e

echo "=== KIEM TRA PLUGIN GAZEBO & MOI TRUONG ==="
MISSING=0

PLUGINS=(
  "libgazebo_ros_planar_move.so"
  "libgazebo_ros_vacuum_gripper.so"
  "libgazebo_ros2_control.so"
  "libgazebo_ros_ray_sensor.so"
  "libgazebo_ros_camera.so"
  "libgazebo_ros_imu_sensor.so"
  "libgazebo_ros_state.so"
)

echo "--- 1. Kiem tra cac thu vien .so trong /opt/ros/humble/lib ---"
for plugin in "${PLUGINS[@]}"; do
  found=$(find /opt/ros/humble/lib -name "$plugin" 2>/dev/null | head -n 1)
  if [ -n "$found" ]; then
    echo "[OK] Tim thay: $plugin -> $found"
  else
    echo "[FAIL] THIEU: $plugin"
    MISSING=$((MISSING + 1))
  fi
done

echo "--- 2. Kiem tra OpenCV va cv2.aruco ---"
python3 -c "import cv2; print(\"OpenCV version:\", cv2.__version__, \"| Has aruco:\", hasattr(cv2,\"aruco\")); assert hasattr(cv2, \"aruco\")" || {
  echo "[FAIL] cv2.aruco khong ton tai!"
  MISSING=$((MISSING + 1))
}

echo "--- 3. Kiem tra cac package ROS 2 (mppi, collision_monitor, robot_localization, slam_toolbox) ---"
source /opt/ros/humble/setup.bash
PKGS=$(ros2 pkg list | grep -E "mppi|collision_monitor|robot_localization|slam_toolbox" || true)
echo "$PKGS"

for req_pkg in nav2_mppi_controller nav2_collision_monitor robot_localization slam_toolbox; do
  if echo "$PKGS" | grep -q "$req_pkg"; then
    echo "[OK] Goi $req_pkg da duoc cai dat"
  else
    echo "[FAIL] THIEU goi $req_pkg"
    MISSING=$((MISSING + 1))
  fi
done

if [ "$MISSING" -gt 0 ]; then
  echo "============================================="
  echo "[ERROR] Thieu $MISSING thanh phan yeu cau! Dung lai."
  exit 1
else
  echo "============================================="
  echo "[SUCCESS] Tat ca plugin, aruco va package dieu huong da day du!"
  exit 0
fi
