# WAREHOUSE AMR + SCARA AUTOMATION SYSTEM

Hệ thống xe tự hành đa hướng (AGV/AMR Mecanum) tích hợp tay máy SCARA (TRR Prismatic-Revolute-Revolute) và gắp hút chân không phục vụ quản lý xuất/nhập kho thông minh. Hệ thống ứng dụng kiến trúc điều khiển ba tầng độ chính xác: Điều hướng toàn cục Nav2, Căn chỉnh vị sai thị giác (Visual Servoing) bằng ArUco và Thao tác gắp/đặt khép kín mắt-trong-tay (Eye-in-Hand).

---

## 1. YÊU CẦU HỆ THỐNG PHẦN CỨNG & PHẦN MỀM

- **Hệ điều hành:** Ubuntu 22.04 LTS (Jammy Jellyfish) x86_64.
- **Middleware:** ROS 2 Humble Hawksbill (Desktop Install).
- **Mô phỏng vật lý:** Gazebo 11 (Gazebo Classic 11.10.2+).
- **Phần cứng đề xuất:**
  - CPU: Tối thiểu 4 nhân (khuyến nghị 8+ nhân, ví dụ Intel Core i5/i7/i9 hoặc AMD Ryzen).
  - RAM: Tối thiểu 8 GB (khuyến nghị 16 GB).
  - GPU: Hỗ trợ OpenGL 3.0+ (NVIDIA GPU với driver chính thức cho kết quả mô phỏng tốt nhất).
  - Dung lượng ổ cứng: Tối thiểu 10 GB trống.

---

## 2. LỆNH CÀI ĐẶT CHÍNH XÁC (SYSTEM INSTALLATION)

### 2.1. Cài đặt các gói phụ thuộc ROS 2 & Gazebo qua APT
```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-robot-localization \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-tf-transformations \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-twist-mux \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 2.2. Cài đặt các thư viện Python qua PIP
```bash
python3 -m pip install --upgrade pip
pip install \
  fastapi==0.115.0 \
  uvicorn[standard]==0.31.0 \
  websockets==13.1 \
  httpx==0.27.2 \
  pytest==8.3.3 \
  psutil==6.0.0 \
  matplotlib==3.9.2 \
  pandas==2.2.3 \
  numpy==1.26.4 \
  opencv-python-headless==4.10.0.84 \
  scipy==1.14.1 \
  transforms3d==0.4.2
```
> **Lưu ý:** Giữ `numpy < 2.0` để đảm bảo tính tương thích nhị phân với `cv_bridge` và `rclpy`.

---

## 3. BIÊN DỊCH HỆ THỐNG (BUILD WORKSPACE)

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash

# Biên dịch sạch toàn bộ 9 package
colcon build --symlink-install

# Nạp môi trường làm việc
source install/setup.bash
```

---

## 4. HƯỚNG DẪN CHẠY TỪNG GIAI ĐOẠN (STEP-BY-STEP WORKFLOW)

Hệ thống có thể được chạy và nghiệm thu độc lập theo từng bài toán kỹ thuật từ 02 đến 08b:

### Bài 02: Mô hình Robot, Bánh Mecanum & Cảm biến
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_description robot_state_publisher.launch.py
  ```
- **Lệnh nghiệm thu:**
  ```bash
  python3 scripts/test_step2.py
  ```

### Bài 03: Môi trường Kho Gazebo, Giá kệ & Tag ArUco
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_gazebo warehouse.launch.py
  ```
- **Lệnh nghiệm thu:**
  ```bash
  python3 scripts/test_step3.py
  ```

### Bài 04: Điều hướng Tự hành Nav2 & Bản đồ Tĩnh
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_navigation nav.launch.py mode:=navigation
  ```
- **Lệnh nghiệm thu:**
  ```bash
  python3 scripts/benchmark_nav.py --trials 20
  ```

### Bài 05: Thị giác Máy tính & Căn chỉnh Ô kệ (Visual Servoing Docking)
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_vision vision.launch.py
  # và khởi động dock server:
  ros2 run wh_navigation dock_server_node
  ```
- **Lệnh nghiệm thu:**
  ```bash
  python3 scripts/benchmark_docking.py --runs 30
  ```

### Bài 06: Tay máy SCARA, Động học & Gắp/Đặt Hút Chân không
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_manipulation manipulation.launch.py
  ```
- **Lệnh nghiệm thu:**
  ```bash
  python3 scripts/benchmark_pick.py --runs-per-tier 30
  ```

### Bài 07: Quản lý Nhiệm vụ FSM & Cơ sở Dữ liệu Kho SQLite
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_task_manager task_manager.launch.py
  ```
- **Lệnh nghiệm thu:**
  ```bash
  pytest src/wh_task_manager/test/
  ```

### Bài 08: Giao diện Web Quản lý Kho & REST API
- **Lệnh chạy:**
  ```bash
  ros2 launch wh_web web.launch.py
  ```
- **Lệnh nghiệm thu:**
  ```bash
  pytest src/wh_web/test/
  python3 src/wh_web/scripts/benchmark_latency.py
  ```

### Bài 08b: Bản đồ SLAM Thời gian thực & Teleop trên Web
- **Lệnh chạy chế độ dựng map:**
  ```bash
  ros2 launch wh_navigation nav.launch.py mode:=mapping
  ```
- **Lệnh nghiệm thu:**
  ```bash
  python3 scripts/benchmark_slam_load.py
  ```

---

## 5. CHẠY TOÀN BỘ HỆ THỐNG MỘT LỆNH DUY NHẤT (FULL SYSTEM LAUNCH)

Khởi động đồng bộ toàn bộ Gazebo, Robot, Nav2, Vision, SCARA, Task Manager, Web Server và RViz:

```bash
cd ~/warehouse_ws
source install/setup.bash
ros2 launch wh_bringup full_system.launch.py
```

### Các tham số cấu hình linh hoạt:
- `headless:=true` : Chạy không mở cửa sổ Gazebo GUI (tiết kiệm CPU/RAM).
- `rviz:=true`     : Tự động mở RViz2 với cấu hình chuẩn `full_system.rviz`.
- `mode:=navigation` (mặc định) hoặc `mode:=mapping` (dựng bản đồ SLAM).
- `enable_slip:=true` : Bật mô phỏng trượt bánh Mecanum ngẫu nhiên.
- `use_sim_time:=true` : Sử dụng thời gian mô phỏng Gazebo.

### Kiểm tra hệ thống sẵn sàng tự động:
Mở một terminal khác và chạy:
```bash
python3 scripts/wait_ready.py
```
Khi mọi topic, action server và REST API sẵn sàng, script sẽ thông báo thành công và thoát với mã `0`.

---

## 6. TRUY CẬP GIAO DIỆN WEB TRÊN THIẾT BỊ DI ĐỘNG

Giao diện web hoàn toàn Responsive, tương thích từ màn hình PC đến điện thoại (iOS Safari, Android Chrome):

1. Xác định địa chỉ IP máy trạm chạy ROS:
   ```bash
   hostname -I | awk '{print $1}'
   ```
   *(Ví dụ: `192.168.1.100`)*
2. Đảm bảo điện thoại kết nối **cùng mạng Wi-Fi** với máy trạm.
3. Mở trình duyệt trên điện thoại và truy cập:
   ```text
   http://192.168.1.100:8000
   ```
4. Các tính năng chính trên điện thoại:
   - **Mặt bằng kho sống (Canvas):** Theo dõi robot di chuyển, các lớp LiDAR, quỹ đạo và vị trí ô kệ.
   - **Tồn kho:** Tra cứu danh mục, tình trạng chiếm chỗ 12 ô kệ trực quan theo màu (Trống: Xanh lá, Đầy: Xanh dương, Khóa: Cam).
   - **Nhập / Xuất kho:** Tạo đơn hàng tức thời chỉ bằng vài chạm.
   - **D-pad Teleop:** Cụm phím điều khiển từ xa có Watchdog an toàn tự dừng khi thả tay.
   - **Nút DỪNG KHẨN CẤP (E-STOP):** Nút đỏ cố định trên thanh điều hướng phản hồi trong $< 2\,\text{ms}$.

---

## 7. KỊCH BẢN DEMO TỰ ĐỘNG & BỘ ĐO CHỈ TIÊU BÁO CÁO

### 7.1. Chạy Demo Tự động Không Can thiệp Tay (Bài 09.2)
Chạy toàn bộ kịch bản 7 bước (Xuất 3 đơn, Nhập 3 đơn, Né vật cản bất ngờ, Từ chối sai mã, Phục hồi lỗi, Từ chối hết hàng, và Đối chiếu 100% CSDL vs Gazebo):
```bash
python3 scripts/demo_run.py
```
Kết quả được xuất ra bảng tổng kết tại terminal và lưu file: `docs/results/demo_summary.json`.

### 7.2. Chạy Toàn bộ Bộ Đo Chỉ tiêu (Bài 09.3)
```bash
# Chạy nhanh kiểm tra luồng:
./scripts/run_all_benchmarks.sh --quick

# Hoặc chạy kiểm tra đầy đủ số lượt thống kê:
./scripts/run_all_benchmarks.sh --full
```
Các kết quả định lượng CSV và biểu đồ 150 DPI sẽ được tổng hợp đầy đủ trong thư mục `docs/results/`.

---

## 8. XỬ LÝ SỰ CỐ & CÁC LỖI THƯỜNG GẶP (TROUBLESHOOTING)

1. **Lỗi `RMW_IMPLEMENTATION` hoặc không nhận Topic giữa các tiến trình:**
   - Đảm bảo thiết lập chung Domain ID: `export ROS_DOMAIN_ID=42`.
   - Sử dụng CycloneDDS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
2. **Gazebo không tìm thấy Model vật liệu / Tag ArUco:**
   - Nạp biến môi trường model Gazebo:
     ```bash
     export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/warehouse_ws/src/wh_gazebo/models
     export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/warehouse_ws/src/wh_gazebo/models
     ```
3. **Nav2 không nhận diện vị trí ban đầu (Robot đứng yên):**
   - Đảm bảo topic `/initialpose` đã được phát hoặc chờ `full_system.launch.py` tự động kích hoạt pose HOME tại `(-2.4, 0.0, 0.0)`.
4. **Xung đột TF `map -> odom`:**
   - Không được chạy đồng thời `amcl` và `slam_toolbox`. Sử dụng tham số `mode:=navigation` hoặc `mode:=mapping` trong launch file.
5. **Cổng Web 8000 bị chiếm dụng (`Address already in use`):**
   - Giải phóng cổng bằng lệnh: `fuser -k 8000/tcp`.

---

## 9. KHÁC BIỆT GIỮA MÔ PHỎNG VÀ ROBOT THẬT (SIM-TO-REAL GAPS)

Nhằm đảm bảo tính trung thực khoa học, hệ thống ghi nhận rõ ràng các khác biệt cốt lõi giữa môi trường ảo Gazebo và điều kiện phần cứng thực tế:

1. **Hệ dẫn động Mecanum:**
   - *Mô phỏng:* Sử dụng plugin `planar_move` tích hợp lực gia tốc ảo trực tiếp lên thân xe. Trượt bánh được giả lập bằng mô hình tham số ngẫu nhiên (`wheel_slip_model`).
   - *Thực tế:* Chuyển động phụ thuộc hoàn toàn vào lực ma sát tiếp xúc động lực học của từng con lăn nghiêng 45° trên bề mặt sàn, chịu ảnh hưởng lớn bởi độ phẳng của sàn và tải trọng phân bố không đều.
2. **Cơ cấu chấp hành gắp hút chân không:**
   - *Mô phỏng:* Sử dụng plugin `gazebo_ros_vacuum_gripper` tạo liên kết cứng tức thời (fixed joint) khi đầu hút tiếp xúc bề mặt hộp.
   - *Thực tế:* Cần bơm hút chân không cơ điện (vacuum pump), van điện từ đảo chiều (solenoid release valve), giác hút cao su linh hoạt. Cần tính đến thời gian trễ tạo áp suất âm ($\sim 0.2 - 0.5\,\text{s}$), rò rỉ khí khi bề mặt hộp có nếp gấp hoặc rung lắc khi xe di chuyển.
3. **Hệ thống thị giác và Camera:**
   - *Mô phỏng:* Camera Gazebo cung cấp ảnh hoàn hảo, không có hiện tượng mờ do chuyển động (motion blur), không nhiễu hạt cảm biến nhiệt độ, không méo hình học thấu kính.
   - *Thực tế:* Bắt buộc phải thực hiện hiệu chuẩn bàn cờ (Checkerboard Intrinsic Calibration) để khử méo thấu kính và hiệu chuẩn Hand-Eye Calibration để xác định chính xác ma trận biến đổi tọa độ giữa khâu chấp hành và camera.
4. **Điều kiện ánh sáng môi trường:**
   - *Mô phỏng:* Gazebo duy trì nguồn sáng ổn định, đồng nhất từ trần ảo.
   - *Thực tế:* Môi trường nhà kho công nghiệp chịu ảnh hưởng bởi ánh sáng tự nhiên thay đổi theo giờ, bóng đổ từ các tầng giá kệ và hiện tượng chói lóa từ băng dính hoặc bề mặt nhựa hộp, đòi hỏi phải trang bị đèn LED vòng trợ sáng chủ động (active LED ring).
5. **Động lực học và Sai số Cơ khí:**
   - *Mô phỏng:* Các khâu nối cứng tuyệt đối, không có độ rơ bánh răng (backlash), không biến dạng đàn hồi khi vươn xa mang tải nặng.
   - *Thực tế:* Xuất hiện độ rung lắc đàn hồi của thanh trượt SCARA, trễ truyền thông qua bus CAN/UART và suy hao động cơ theo thời gian.
6. **Phiên bản Hệ điều hành và ROS:**
   - Khuyến nghị chốt một phiên bản duy nhất: **ROS 2 Humble LTS trên nền tảng Ubuntu 22.04 LTS ARM64** (chạy trên NVIDIA Jetson Orin Nano/NX hoặc Raspberry Pi 5).

---

## 10. DANH SÁCH VIỆC CẦN LÀM KHI CHUYỂN SANG ROBOT THẬT (HARDWARE TRANSFER ROADMAP)

Để đưa thiết kế vào sản xuất và vận hành trên phần cứng thực tế, các bước kỹ thuật cần thực hiện gồm:

1. **Bo mạch vi điều khiển cấp thấp (Low-level MCU):**
   - Thiết kế hoặc sử dụng bo mạch STM32F4/F7 hoặc ESP32 chạy giao thức **micro-ROS** giao tiếp với máy tính nhúng qua UART/USB hoặc CAN bus.
2. **Điều khiển bám vận tốc 4 bánh Mecanum:**
   - Cài đặt thuật toán giải động học ngược 4 bánh Mecanum trên MCU kết hợp vòng điều khiển PID số tốc độ bánh xe vòng kín từ encoder quang học ở tần số $\ge 100\,\text{Hz}$.
3. **Hiệu chuẩn Thị giác Đầy đủ:**
   - Hiệu chuẩn tham số nội ($f_x, f_y, c_x, c_y$) và hệ số méo thấu kính ($k_1, k_2, p_1, p_2$) cho cả 2 camera (camera sườn và camera bàn tay).
   - Thực hiện hiệu chuẩn Hand-Eye Calibration (phương pháp Tsai-Lenz) để tìm biến đổi chính xác `tool_link -> hand_camera_optical_frame`.
4. **Cảm biến và Giám sát Chân không:**
   - Lắp đặt cảm biến áp suất số (Digital Vacuum Pressure Sensor, ví dụ SMC ZSE30) nối về chân ngắt GPIO của MCU để xác nhận chắc chắn lực giữ hộp trước khi nhấc tay máy.
5. **Dựng Môi trường Kho Thực tế:**
   - Gia công kết cấu kệ kho đúng kích thước danh định, dán mã ArUco in trên decal nhám mờ chống phản quang, dán vạch dẫn hướng sàn.
6. **Kiểm thử và Rào chắn An toàn:**
   - Tích hợp rơ-le ngắt nguồn khẩn cấp vật lý (Hardware E-Stop Relay) ngắt toàn bộ mạch động lực động cơ khi ấn nút.
   - Lắp đặt cản va đập an toàn (Safety Bumper Switches) ở 4 mặt sườn xe tự hành.
