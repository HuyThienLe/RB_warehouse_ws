# NHẬT KÝ SỬA LỖI HỆ THỐNG (BUGLOG.md) - BÀI 10

Theo quy tắc cứng mục 10.1:
- Mỗi lần sửa lỗi phải ghi nhận đầy đủ: Triệu chứng, Bằng chứng, Nguyên nhân gốc, Cách sửa, Cách kiểm chứng, Git commit hash.
- Mọi lần sửa mã đều **đặt bộ đếm vòng lặp 20 lần về 0**.

---

## 1. BUG-01: Camera Cập Nhật Sai, Chưa Gắp Được Hộp (Triệu Chứng 1)

- **Triệu chứng:** Tay gắp SCARA không gắp được vật; camera đầu tay không nhìn thấy ArUco tag đúng góc độ; cánh tay bị giật hoặc gắp lệch tâm.
- **Bằng chứng thu thập:**
  1. Hình ảnh thu từ `/hand_camera/image_raw` bị che khuất một phần bởi đĩa kim loại của `tool_link`.
  2. Khảo sát URDF trong `src/wh_description/urdf/sensors.xacro`: khớp `hand_camera_joint` đặt tại `xyz="0.0 0.0 -0.042"` trùng tâm đầu hút, không đúng yêu cầu Mục 10.4.D.11 (phải nhìn thẳng xuống và lệch về sau 4 cm so với đầu hút).
  3. Lệnh đưa tay về vị trí stowed trong `src/wh_navigation/scripts/auto_initial_pose.py` gửi `[0.07, -1.8, 2.2]`, trong khi giới hạn khớp `shoulder_joint` trong URDF là `[-1.5708, +1.5708]` (vi phạm giới hạn phần cứng, làm controller fail).
  4. Node `dock_server` chỉ phục vụ action `dock_to_slot` mà thiếu alias `/wh/dock_to_slot`.
- **Nguyên nhân gốc:** Khung quang học và bracket camera trên đầu tay bị sai offset và gây tự che khuất; tư thế gấp tay ban đầu vượt giới hạn góc xoay của khớp vai; thiếu alias action server.
- **Cách sửa:**
  1. `src/wh_description/urdf/sensors.xacro`: Điều chỉnh `hand_camera_joint` sang `xyz="-0.040 0.0 -0.042"` (lùi về sau 4 cm), dời bracket ra sau cảm biến `<pose>0.010 0 0 0 0 0</pose>`.
  2. `src/wh_navigation/scripts/auto_initial_pose.py`: Sửa tư thế stow hợp lệ: `[0.050, -1.5708, -2.3562]`.
  3. `src/wh_vision/wh_vision/dock_server.py`: Đăng ký cả hai endpoint action `/wh/dock_to_slot` và `dock_to_slot`.
- **Cách kiểm chứng:**
  - Tầng L2 PASS: Tần số hand camera 26.4 Hz, pixel mean 57.6 > 10.0, ảnh rõ nét, không bị che khuất.
  - Chạy `test_step5_accuracy.py`: Nhận diện tag chính xác, sai số < 3 mm.
- **Git Commit:** `a6b834e`

---

## 2. BUG-02: Vị Trí Robot Trên Bản Đồ Bị Sai / Giật Lệch (Triệu Chứng 2)

- **Triệu chứng:** Trên giao diện Web Portal và RViz, robot bị nhảy cóc hoặc trôi dần (drift) khi di chuyển, vệt di chuyển (trail) bị lệch khỏi hành lang thực tế.
- **Bằng chứng thu thập:**
  1. Kiểm tra mã nguồn `src/wh_web/wh_web/web_bridge.py`: Node lấy tọa độ robot trực tiếp từ `/odometry/filtered` và `/odom`. Do xe dùng bánh Mecanum trượt ngang, khung `odom` bị tích lũy trôi theo thời gian so với khung `map`.
  2. `web_bridge_node` thiếu cờ `use_sim_time:=True`, gây lệch pha thời gian so với Gazebo `/clock`.
  3. Trong `src/wh_navigation/config/nav2_params.yaml`: AMCL cấu hình `transform_tolerance: 1.0` khiến timestamp của transform `map -> odom` bị đóng dấu 1.0s trong tương lai, gây lỗi `ExtrapolationException: Lookup would require extrapolation into the future/past` khi TF buffer tra cứu tại thời điểm hiện tại `Time()`.
- **Nguyên nhân gốc:** WebBridge hiển thị pose từ khung `odom` thay vì tra cứu TF `map -> base_footprint`; AMCL `transform_tolerance` quá lớn gây lỗi đồng bộ TF2.
- **Cách sửa:**
  1. `src/wh_web/wh_web/web_bridge.py`: Bổ sung `use_sim_time=True`, thêm `tf2_ros.Buffer` và `TransformListener`, tạo timer 20 Hz tra cứu trực tiếp transform `map -> base_footprint`.
  2. `src/wh_navigation/config/nav2_params.yaml`: Điều chỉnh AMCL `transform_tolerance: 0.2`, giảm độ nhiễu hạt `alpha1..5: 0.05`, tăng `max_beams: 100` để quét hạt chính xác hơn.
- **Cách kiểm chứng:**
  - Chạy `scripts/test_localization.py`: Robot hoàn thành 10/10 chặng, sai số vị trí trung bình 3.69 cm (< 5.0 cm PASS), sai số góc trung bình 0.40° (< 5.0° PASS).
- **Git Commit:** `a6b834e`

---

## 3. BUG-03: Robot Bị Khựng / Đứng Yên Khi Làm Nhiệm Vụ (Triệu Chứng 3)

- **Triệu chứng:** Khi tiếp cận giá kệ để dock hoặc khi di chuyển qua lối đi hẹp, robot đột ngột dừng lại, lệnh vận tốc `/cmd_vel` biến mất về 0, FSM bị treo.
- **Bằng chứng thu thập:**
  1. Kiểm tra chuỗi vận tốc qua `ros2 topic echo`: Tại thời điểm khựng, `cmd_vel_nav` hoặc `cmd_vel_servo` vẫn phát tín hiệu đều đặn, nhưng đầu ra `/cmd_vel` bị kéo về 0 bởi `collision_monitor`.
  2. Kiểm tra `src/wh_navigation/config/collision_monitor_params.yaml`: Vùng `StopZone` được thiết lập nhô ra 35 cm phía trước đầu xe. Khi robot thực hiện docking visual servoing với khoảng cách mục tiêu `standoff = 0.32 m` (32 cm), mép dưới của kệ rơi thẳng vào vùng `StopZone`, khiến robot tự khóa khẩn cấp.
  3. Tham số `simulate_ahead_time` trong Nav2 `behavior_server` đặt ở mức 2.0s gây tính toán sai không gian xoay trong lối đi hẹp 1.2m giữa hai kệ.
  4. Timeout của action client trong `src/wh_task_manager/wh_task_manager/task_manager_node.py` đặt 30s quá ngắn cho các hành trình dài.
- **Nguyên nhân gốc:** Vùng dừng khẩn cấp StopZone xâm lấn vùng làm việc bình thường của docking; timeout tác vụ Nav2 và docking chưa đủ dự phòng.
- **Cách sửa:**
  1. Thu gọn vùng `StopZone` trong `collision_monitor_params.yaml` xuống sát đầu xe (cách bumper 2 cm: `[0.245, 0.14, 0.245, -0.14, 0.225, -0.14, 0.225, 0.14]`).
  2. Giảm `simulate_ahead_time` trong `nav2_params.yaml` xuống `0.8s`.
  3. Tăng timeout trong `task_manager_node.py`: nav=75s, dock=45s, pick_place=60s.
- **Cách kiểm chứng:**
  - Robot hoàn thành trọn vẹn cả 10 chặng thử nghiệm và các tác vụ docking không bị gián đoạn.
- **Git Commit:** `a6b834e`

---

## 4. BUG-04: Mất Tag Tầng 1 Khi Docking & Lỗi Luồng WebBridge (Triệu Chứng 1 & 3)

- **Triệu chứng:** Khi tiếp cận điểm chờ trước ô kệ tầng 1 (ví dụ slot 0), robot dừng lại và báo lỗi `TAG_LOST`, liên tục lùi thử lại và FSM bị treo; luồng ROS của WebBridge báo lỗi crash `reproj_error`.
- **Bằng chứng thu thập:**
  1. Trích xuất ảnh từ `/dock_camera/image_raw` tại vị trí robot (x=0.836, y=0.048): Camera nằm ở độ cao Z=0.34m và hướng ngang hoàn toàn (`rpy="0 0 0"`). Ở khoảng cách 0.32m tới kệ, góc nhìn dưới của camera chỉ đạt Z=0.172m, trong khi tag ô kệ tầng 1 (Tag 0) ở độ cao Z=0.11m (thấp hơn mép nhìn 6.2cm). Tag 0 chỉ hiển thị 8 pixel sát mép đáy và bị bộ phát hiện ArUco xếp vào nhóm `rejected candidate`.
  2. Node `dock_server` không nhận được tag 0 nên hủy và báo `TAG_LOST`, FSM thử lại vô hạn.
  3. Log Uvicorn: `AttributeError: 'TagDetection' object has no attribute 'reproj_error'` do trường đúng trong định nghĩa `.msg` là `reproj_error_px`.
  4. Bộ kiểm thử `soak_test.py` dùng `tasks.get("active")` trên kết quả dạng danh sách và chỉ chờ trạng thái `"DONE"`, trong khi hệ thống gán trạng thái hoàn tất là `"SUCCEEDED"`.
- **Nguyên nhân gốc:** Khớp camera `dock_camera_joint` thiếu góc nghiêng chúc xuống để bao quát tầm thấp; sai tên thuộc tính trong `web_bridge.py`; thiếu chuẩn hóa trạng thái hoàn thành trong harness test.
- **Cách sửa:**
  1. `src/wh_description/urdf/sensors.xacro`: Điều chỉnh `dock_camera_joint` nghiêng chúc xuống 0.10 rad (~5.7°) (`rpy="0 0.10 0"`), mở rộng trường nhìn thấp, thu trọn vẹn và cân bằng cả tag tầng 1 (Z=0.11m, y ≈ 408px) và tầng 2 (Z=0.36m, y ≈ 87px).
  2. `src/wh_web/wh_web/web_bridge.py`: Truy cập an toàn trường `reproj_error_px`.
  3. `scripts/soak_test.py`: Hỗ trợ duyệt cả list và dict từ `/api/tasks`, chấp nhận trạng thái `"SUCCEEDED"`, dọn sạch triệt để các tiến trình bằng `pkill -9 -f warehouse_ws`.
- **Cách kiểm chứng:**
  - Tag 0 và Tag 3 đều hiển thị đầy đủ, nguyên vẹn trên ảnh camera dock trong khung hình 480p.
  - Tác vụ docking và đơn xuất/nhập hoàn thành trọn vẹn.
- **Git Commit:** `90bd219`


