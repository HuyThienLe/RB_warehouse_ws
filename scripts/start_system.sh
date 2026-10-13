#!/usr/bin/env bash
# start_system.sh – Khởi chạy toàn bộ hệ thống Kho thông minh AMR

set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

# Nạp môi trường ROS 2
source /opt/ros/humble/setup.bash

# Build cập nhật (nếu có chỉnh sửa)
colcon build --symlink-install

# Nạp workspace
source install/setup.bash

# Reset CSDL nếu cần
# rm -f ~/.warehouse/inventory.db

echo "Khởi chạy toàn bộ hệ thống Kho thông minh AMR..."
ros2 launch wh_bringup full_system.launch.py gui:=true rviz:=false
