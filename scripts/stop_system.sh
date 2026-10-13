#!/usr/bin/env bash
# stop_system.sh – Tắt sạch toàn bộ mô phỏng Gazebo, ROS 2 nodes và Web server

echo "Đang dừng toàn bộ hệ thống Kho thông minh..."
killall -9 gzserver gzclient 2>/dev/null || true
pkill -9 -f "wh_bringup|wh_task_manager|wh_vision|wh_manipulation|wh_web|nav2|amcl|robot_state_publisher|auto_initial_pose|box_manager|gripper_node" 2>/dev/null || true
sleep 1
echo "Đã tắt sạch hệ thống thành công!"
