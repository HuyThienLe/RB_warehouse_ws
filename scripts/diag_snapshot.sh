#!/usr/bin/env bash
# diag_snapshot.sh – Thu thập thông tin chẩn đoán lỗi khi hệ thống gặp sự cố (Bài 10.2).
# Sử dụng: ./scripts/diag_snapshot.sh <RUN_ID>

RUN_ID=${1:-"manual_$(date +%Y%m%d_%H%M%S)"}
TARGET_DIR="docs/failures/run_${RUN_ID}"
mkdir -p "$TARGET_DIR"

echo "=========================================================="
echo "  📸 ĐANG LƯU ẢNH CHẨN ĐOÁN HỆ THỐNG VÀO: $TARGET_DIR"
echo "=========================================================="

source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# 1. Danh sách node, topic, action, service
ros2 node list > "$TARGET_DIR/nodes.txt" 2>&1 || true
ros2 topic list -t > "$TARGET_DIR/topics.txt" 2>&1 || true
ros2 action list > "$TARGET_DIR/actions.txt" 2>&1 || true
ros2 service list > "$TARGET_DIR/services.txt" 2>&1 || true

# 2. Nav2 lifecycle
{
  echo "=== NAV2 LIFECYCLE ==="
  for n in /map_server /amcl /controller_server /planner_server /behavior_server /bt_navigator; do
    echo -n "$n: "
    ros2 lifecycle get "$n" 2>&1 || echo "failed"
  done
} > "$TARGET_DIR/nav2_lifecycle.txt"

# 3. TF frames tree (view_frames)
echo "Generating TF frames graph..."
(
  cd "$TARGET_DIR" || exit
  timeout 5s ros2 run tf2_tools view_frames 2>&1 || true
  if [ -f frames.pdf ]; then
    pdftoppm -png -r 150 frames.pdf frames_graph 2>/dev/null || true
  fi
)

# 4. Ghi 10 giây cmd_vel*
echo "Lấy mẫu 10s các topic /cmd_vel*..."
timeout 10s ros2 topic echo /cmd_vel > "$TARGET_DIR/cmd_vel.txt" 2>&1 &
timeout 10s ros2 topic echo /cmd_vel_nav > "$TARGET_DIR/cmd_vel_nav.txt" 2>&1 &
timeout 10s ros2 topic echo /cmd_vel_servo > "$TARGET_DIR/cmd_vel_servo.txt" 2>&1 &
timeout 10s ros2 topic echo /cmd_vel_muxed > "$TARGET_DIR/cmd_vel_muxed.txt" 2>&1 &

# 5. Trạng thái FSM & robot status
timeout 2s ros2 topic echo --once /robot_status > "$TARGET_DIR/robot_status.txt" 2>&1 || true

# 6. Ảnh chụp từ các camera
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rclpy.init()
node = Node('diag_cam_saver')
bridge = CvBridge()

def save_img(topic, filename):
    msg = None
    def cb(m):
        nonlocal msg
        msg = m
    sub = node.create_subscription(Image, topic, cb, 1)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if msg:
            break
    if msg:
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite('$TARGET_DIR/' + filename, cv_img)
            print(f'Saved {filename}')
        except Exception as e:
            print(f'Error saving {filename}: {e}')
    else:
        print(f'Timeout waiting for {topic}')

save_img('/dock_camera/image_raw', 'dock_camera.png')
save_img('/hand_camera/image_raw', 'hand_camera.png')
node.destroy_node()
rclpy.shutdown()
" > "$TARGET_DIR/cam_capture.log" 2>&1 || true

# 7. Vị trí thật trong Gazebo (robot + các hộp)
python3 -c "
import subprocess
try:
    res = subprocess.run(['gz', 'model', '-m', 'warehouse_robot', '-p'], stdout=subprocess.PIPE, text=True, timeout=3)
    with open('$TARGET_DIR/gazebo_robot_pose.txt', 'w') as f:
        f.write(res.stdout)
except Exception as e:
    with open('$TARGET_DIR/gazebo_robot_pose.txt', 'w') as f:
        f.write(str(e))
" 2>/dev/null || true

# 8. 200 dòng cuối log hệ thống
if [ -d "$HOME/.ros/log/latest" ]; then
  mkdir -p "$TARGET_DIR/ros_logs"
  for lf in "$HOME/.ros/log/latest/"*.log; do
    if [ -f "$lf" ]; then
      tail -n 200 "$lf" > "$TARGET_DIR/ros_logs/$(basename "$lf")"
    fi
  done
fi

wait
echo "✅ Hoàn tất lưu chẩn đoán vào $TARGET_DIR."
