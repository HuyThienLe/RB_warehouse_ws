# TEST REPORT - WAREHOUSE AMR ROBOT SYSTEM

## Bước 1: Môi trường và Khung Workspace

### 1.1. Kiểm tra máy
- **Lệnh thực thi:**
  ```bash
  lsb_release -a
  echo $ROS_DISTRO
  gazebo --version
  python3 --version
  nproc; free -h
  ```
- **Kết quả thực tế:**
  - OS: Ubuntu 22.04.5 LTS (jammy)
  - ROS Distro: `humble`
  - Gazebo version: `11.10.2`
  - Python version: `3.10.12`
  - CPU cores: `12`
  - RAM: Total `15Gi`, Used `3.1Gi`, Free `4.5Gi`, Available `11Gi`, Swap `24Gi`
  - Đánh giá: ĐẠT toàn bộ yêu cầu phần cứng và phiên bản hệ điều hành / ROS 2 Humble.

### 1.2. Cài đặt và xác nhận gói phụ thuộc
- **Tập lệnh setup:** `scripts/setup_env.sh`
- **Kiểm tra thư viện Python:**
  - `numpy`: 1.26.4 (< 2.0, bảo toàn tương thích với cv_bridge)
  - `fastapi`: 0.142.4
  - `uvicorn`: 0.54.0
  - `pandas`: 2.3.3
  - `matplotlib`: 3.5.1
  - `pytest`: 9.1.1
  - `httpx`: 0.28.1
  - `cv2`: 4.9.0
  - `scipy`: 1.8.0
  - Đánh giá: ĐẠT.

### 1.3. Khởi tạo 9 Package trong Workspace
- Danh sách 9 package tại `~/warehouse_ws/src`:
  1. `wh_interfaces` (ament_cmake)
  2. `wh_description` (ament_cmake)
  3. `wh_gazebo` (ament_cmake)
  4. `wh_navigation` (ament_cmake)
  5. `wh_bringup` (ament_cmake)
  6. `wh_manipulation` (ament_python)
  7. `wh_vision` (ament_python)
  8. `wh_task_manager` (ament_python)
  9. `wh_web` (ament_python)
- Cấu hình thư mục và `CMakeLists.txt` install directive:
  - `wh_description`: launch, config, urdf, rviz, models
  - `wh_gazebo`: launch, config, worlds, models
  - `wh_navigation`: launch, config, maps, rviz
  - `wh_bringup`: launch, config, rviz, scripts
- Đánh giá: ĐẠT.

### 1.4. Xây dựng và kiểm tra Interfaces (`wh_interfaces`)
- **Tệp tin đã tạo:**
  - `msg/TagDetection.msg`
  - `msg/TagArray.msg`
  - `msg/RobotStatus.msg`
  - `msg/Task.msg`
  - `action/DockToSlot.action`
  - `action/PickPlace.action`
- **Lệnh kiểm tra:**
  ```bash
  ros2 interface show wh_interfaces/action/PickPlace
  ```
- **Kết quả thực tế:**
  ```text
  string mode
  uint32 slot_tag_id
  uint32 expected_box_tag
  uint8 tier
  ---
  bool success
  uint32 box_tag_read
  string message
  ---
  string phase
  ```
- Đánh giá: ĐẠT.

### 1.5. Nguồn tham số gốc (Single Source of Truth)
- `wh_description/config/robot_params.yaml`: Chứa đầy đủ kích thước thân xe ($0.45 \times 0.32 \times 0.12$ m), khối lượng 6 kg, bánh Mecanum ($r=0.05$ m, bề rộng $0.04$ m, vị trí tâm bánh $x=\pm0.15, y=\pm0.19$), tay máy SCARA TRR ($l_1=l_2=0.18$ m, giới hạn góc $q_1 \in [-150^\circ, 150^\circ], q_2 \in [-140^\circ, 140^\circ]$), thông số LiDAR, IMU, camera và khoảng cách standoff (chờ $0.55$ m, làm việc $0.32$ m).
- `wh_gazebo/config/world_params.yaml`: Chứa kích thước kho ($6 \times 4$ m), tường bao $0.5$ m, giá kệ A và B với cao độ tầng 1 ($0.15$ m), tầng 2 ($0.40$ m), danh sách ArUco ID kệ (0-11), trạm IN (tag 30), trạm OUT (tag 31), trạm HOME, kích thước hộp ($60 \times 60 \times 40$ mm, 0.2 kg, tag ID 20-39).
- Đánh giá: ĐẠT.

### 1.6. Kiểm tra Gazebo Plugin và Môi trường
- **Lệnh thực thi:**
  ```bash
  /home/lehuythien/warehouse_ws/scripts/check_env.sh
  ```
- **Kết quả thực tế:**
  ```text
  === KIEM TRA PLUGIN GAZEBO & MOI TRUONG ===
  --- 1. Kiem tra cac thu vien .so trong /opt/ros/humble/lib ---
  [OK] Tim thay: libgazebo_ros_planar_move.so -> /opt/ros/humble/lib/libgazebo_ros_planar_move.so
  [OK] Tim thay: libgazebo_ros_vacuum_gripper.so -> /opt/ros/humble/lib/libgazebo_ros_vacuum_gripper.so
  [OK] Tim thay: libgazebo_ros2_control.so -> /opt/ros/humble/lib/libgazebo_ros2_control.so
  [OK] Tim thay: libgazebo_ros_ray_sensor.so -> /opt/ros/humble/lib/libgazebo_ros_ray_sensor.so
  [OK] Tim thay: libgazebo_ros_camera.so -> /opt/ros/humble/lib/libgazebo_ros_camera.so
  [OK] Tim thay: libgazebo_ros_imu_sensor.so -> /opt/ros/humble/lib/libgazebo_ros_imu_sensor.so
  [OK] Tim thay: libgazebo_ros_state.so -> /opt/ros/humble/lib/libgazebo_ros_state.so
  --- 2. Kiem tra OpenCV va cv2.aruco ---
  OpenCV version: 4.9.0 | Has aruco: True
  --- 3. Kiem tra cac package ROS 2 (mppi, collision_monitor, robot_localization, slam_toolbox) ---
  nav2_collision_monitor
  nav2_mppi_controller
  robot_localization
  slam_toolbox
  [OK] Goi nav2_mppi_controller da duoc cai dat
  [OK] Goi nav2_collision_monitor da duoc cai dat
  [OK] Goi robot_localization da duoc cai dat
  [OK] Goi slam_toolbox da duoc cai dat
  =============================================
  [SUCCESS] Tat ca plugin, aruco va package dieu huong da day du!
  ```
- Đánh giá: ĐẠT.

### 1.7. Biên dịch và Nghiệm thu Toàn bộ
- **Lệnh thực thi:**
  ```bash
  cd ~/warehouse_ws && colcon build --symlink-install
  ```
- **Kết quả thực tế:**
  `Summary: 9 packages finished [7.79s]` - Không có lỗi biên dịch.
- Đánh giá: ĐẠT nghiệm thu Giai đoạn 1.
