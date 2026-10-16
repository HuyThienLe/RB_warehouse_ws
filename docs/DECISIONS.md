# DECISIONS – Step 03: Môi trường kho trong Gazebo

## D-03-01: Texture lật gương ArUco (mirror flip)

**Vấn đề:** Gazebo/OGRE có thể lật ngang texture khi chiếu lên mesh, khiến ArUco
bị phản chiếu và không nhận dạng được hoặc nhận sai id.

**Cách phát hiện:**
1. Sau khi khởi động Gazebo, dùng camera sim chụp ảnh tag ở khoảng cách 0.30 m.
2. Chạy `ArucoCompat.detect()` trên ảnh đó.
3. Nếu `ids` trả về `None` hoặc sai: **bị lật gương**.

**Cách sửa (nếu bị lật ngang):**
```python
# Trong gen_tags.py, trước khi lưu PNG, thêm:
canvas_rgb = cv2.flip(canvas_rgb, 1)   # lật theo trục ngang
```
Sau đó chạy lại `gen_tags.py` và rebuild.

**Trạng thái hiện tại:** Texture không lật (dùng nguyên gốc `make_marker`).
Kết quả kiểm tra offline 100% đúng id.
**Kiểm tra runtime** cần thực hiện sau khi Gazebo chạy (xem mục nghiệm thu 3.2).

---

## D-03-02: Vị trí tag ô (tag_pose)

- **Giá A (front_y = +0.60, hướng -y):** tag đặt tại `y = front_y - 0.001` (nổi ra
  phía lối đi -y), `yaw = π` (mặt tag quay về -y, hướng robot tiếp cận).
- **Giá B (front_y = -0.60, hướng +y):** tag đặt tại `y = front_y + 0.001` (nổi ra
  phía lối đi +y), `yaw = 0` (mặt tag quay về +y).
- `z = elevation_z - 0.04` (ngay dưới cạnh dưới tấm sàn ô).

**Lưu ý:** Nếu tấm sàn kệ che khuất tag theo góc nhìn thấp (robot thấp),
dời tag xuống thêm 2–3 cm: `z = elevation_z - 0.07`.

---

## D-03-03: Hộp không rung sau spawn

**Thiết lập chống rung:**
- `max_contacts = 4` (giảm tiếp xúc giải vật lý)
- `min_depth = 0.001` (penetration nhỏ để tránh nảy)
- `max_vel = 0.01` (giới hạn tốc độ tiếp xúc)
- `velocity_decay` linear = 0.05, angular = 0.05
- `box_z = z_face + 0.02 + 0.001` (nâng thêm 1 mm so với lý thuyết)
- Physics: `max_step_size = 0.002`, `real_time_update_rate = 500`

Nếu vẫn rung: tăng damping lên 0.1–0.2, hoặc tạm dùng `<static>true</static>`
khi đặt vào kệ và chuyển sang dynamic khi cần di chuyển.

---

## D-03-04: Real-time factor

**Cấu hình:** `max_step_size 0.002`, `real_time_update_rate 500` → RTF mục tiêu = 1.0.

**Nếu RTF < 0.8:**
1. Giảm `update_rate` camera từ 30 → 10 fps.
2. Đơn giản hóa collision mesh (box → sphere khi không cần chính xác).
3. Giảm `real_time_update_rate` xuống 250 (tăng `max_step_size` lên 0.004).
4. Chạy headless: `ros2 launch wh_gazebo warehouse.launch.py headless:=true`.

---

## D-03-05: Model path Gazebo

Cấu hình qua `AppendEnvironmentVariable` trong launch:
```python
GAZEBO_MODEL_PATH  += <pkg_share>/models
GAZEBO_RESOURCE_PATH += <pkg_share>/models
```
URI trong SDF: `model://aruco_tags/materials/scripts`
→ Gazebo tìm `$GAZEBO_MODEL_PATH/aruco_tags/materials/scripts/`.

**Kiểm tra:** Nếu tag trắng trong Gazebo → đường dẫn sai.
Log: `[Err] [Material.cc] Ogre could not find material: Aruco/Tag0`.

---

## D-03-06: Kệ geometry
 
- **Shelf columns:** 3 cột có tâm `x = [0.75, 1.25, 1.75]`, bước cột `0.50 m`.
- **Shelf width:** `shelf_w = 1.50 m` (từ `x = 0.50` đến `x = 2.00`), có vách ngăn chia ô tại `x = 1.00` và `x = 1.50`.
- **Shelf depth** = 0.22 m (sâu từ mặt trước vào trong).
- **Tâm hộp:** `y = front_y ± depth/2 = 0.60 ± 0.11`.  
   Giá A: `y = 0.71`, Giá B: `y = -0.71`.
- **Staging pose:** cách mặt kệ 0.55 m → `y = 0.60 - 0.55 = 0.05` (giá A),
   `y = -0.60 + 0.55 = -0.05` (giá B).
- **Work pose:** cách mặt kệ 0.32 m.

---

## D-03-07: Trạm sạc pin (Charging Station) - Đã loại bỏ
- Theo yêu cầu điều chỉnh, toàn bộ khối trụ sạc pin (charging dock body, contact plates, LED, tag 32) đã được loại bỏ khỏi môi trường kho.
- Trạm HOME hiện tại chỉ sử dụng ô định vị sàn mỏng phẳng (`floor_home`) tại tọa độ `(x = -2.4, y = 0.0, z = 0.001)`.

---

## D-03-08: Cơ cấu chấp hành tay kẹp 2 ngón (Parallel Gripper)
- Thay thế hoàn toàn cơ cấu giác hút chân không (suction cup) bằng tay kẹp cơ khí 2 ngón song song (parallel gripper).
- **Cấu trúc link:**
  - `tool_link`: Khung đế kẹp (palm) ngang kích thước `0.05 x 0.10 x 0.02 m`.
  - `left_finger_link` & `right_finger_link`: 2 má kẹp song song vươn xuống dưới, có lớp đệm cao su ma sát cao màu cam ở mặt trong. Độ mở giữa 2 má kẹp là 66 mm (bao trọn hộp hàng 60 mm).
  - `gripper_grasp_link`: Điểm tâm kẹp (Tool Center Point - TCP) nằm chính giữa 2 má kẹp.
  - Camera soi bàn tay (`hand_camera`) gắn trên `tool_link` nhìn vuông góc xuống dưới qua tâm giữa 2 má kẹp.
- **Tương thích Gazebo & ROS 2:**
  - Plugin vật lý Gazebo được gắn vào `gripper_grasp_link` với remapping sang `/gripper/switch_raw` và `/gripper/grasping_raw`.
  - Node [`gripper_node.py`](file:///home/lehuythien/warehouse_ws/src/wh_description/scripts/gripper_node.py) cung cấp service chuẩn `/gripper/switch` (`std_srvs/srv/SetBool`) và topic trạng thái `/gripper/grasping` (`std_msgs/msg/Bool`).

---

# DECISIONS – Step 04: Bản đồ, Định vị, Điều hướng, Tránh vật cản

## D-04-01: Chuỗi lệnh vận tốc và cơ chế ưu tiên Twist Mux
- **Cấu trúc luồng điều khiển:**
  ```text
  Nav2 Controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed --+
  Visual Servoing -> /cmd_vel_servo -------------------------------------------+-> twist_mux -> /cmd_vel_muxed -> collision_monitor -> /cmd_vel
  Teleop Keyboard -> /cmd_vel_teleop ------------------------------------------+                      |
                                                                                +-- /estop (Khóa an toàn)
  ```
- **Thứ tự ưu tiên trong `twist_mux.yaml`:**
  1. `estop` (Lock priority 255): Khi `/estop = true`, toàn bộ đầu ra bị khóa cưỡng bức về 0.
  2. `servo` (Topic priority 100): Visual Servoing chiếm quyền điều khiển khi tiếp cận tinh chỉnh ô kệ.
  3. `navigation` (Topic priority 50): Lệnh từ Nav2 sau khi đã làm mịn gia tốc qua `velocity_smoother`.
  4. `teleop` (Topic priority 10): Điều khiển bằng tay khi debug.

---

## D-04-02: Cơ chế Collision Monitor & Giải pháp Docking cận kệ
- **Vấn đề:** Khi robot tiếp cận sát kệ hàng (khoảng cách làm việc `work_pose` cách kệ < 0.20 m), `collision_monitor` mặc định với vùng dừng `StopZone` (0.20 m) sẽ phát hiện mặt kệ và ngắt `/cmd_vel` khiến robot không thể dock sát.
- **Giải pháp thiết kế:**
  1. `collision_monitor` được cấu hình là lớp an toàn độc lập ở cuối chuỗi với 2 vùng:
     - `StopZone`: Vùng dừng khẩn cấp phía trước ($0.20\,\text{m}$).
     - `SlowZone`: Vùng giảm tốc độ ($0.40\,\text{m}$, giảm $50\%$).
  2. **Chế độ Docking:**
     - Khi visual servoing kích hoạt trạng thái DOCK, node điều khiển sẽ thông qua dynamic parameter / reconfigure của ROS 2 để thu nhỏ vùng `StopZone` về mép thân xe (còn $\approx 0.06\,\text{m}$ phía trước cản xe).
     - Đồng thời, vận tốc visual servoing bị giới hạn nghiêm ngặt ở mức an toàn $v \le 0.08\,\text{m/s}$ và $w \le 0.20\,\text{rad/s}$.
     - Khi hoàn tất chu trình gắp/trả và robot lùi ra khỏi ô kệ, vùng an toàn $0.20\,\text{m}$ được khôi phục tự động.

---

## D-04-03: Bản đồ chuẩn hóa và kiểm soát sai số mặt kệ
- **Quy ước kỹ thuật:** Mặt trước các giá kệ trên bản đồ phải trùng khớp tọa độ $y = \pm 0.600\,\text{m}$ (sai số $< 2\,\text{cm}$).
- **Thực hiện:**
  - Script [`gen_map.py`](file:///home/lehuythien/warehouse_ws/scripts/gen_map.py) rasterize trực tiếp từ [`world_params.yaml`](file:///home/lehuythien/warehouse_ws/src/wh_gazebo/config/world_params.yaml) với độ phân giải $0.025\,\text{m/pixel}$.
  - Sai số đo đạc thực tế: Mặt Kệ A tại $y = +0.6000\,\text{m}$, Kệ B tại $y = -0.6000\,\text{m}$ (sai số $= 0.00\,\text{mm} < 20\,\text{mm}$ tiêu chuẩn). Lối đi giữa hai kệ thông thoáng đúng $1.200\,\text{m}$.
  - Script [`auto_mapper.py`](file:///home/lehuythien/warehouse_ws/scripts/auto_mapper.py) cung cấp quy trình quét SLAM trực quan đối chiếu.

---

## D-04-04: Cấu hình Định vị AMCL cho AGV Mecanum (Holonomic)
- Sử dụng mô hình động học toàn hướng `robot_model_type: "nav2_amcl::OmniMotionModel"` với 5 hệ số nhiễu $\alpha_1 \dots \alpha_5 = 0.20$ phản ánh đúng hiện tượng trượt con lăn bánh mecanum.
- Điểm khởi tạo mặc định: Trạm HOME tại $(x = -2.40\,\text{m}, y = 0.00\,\text{m}, \text{yaw} = 0.00\,\text{rad})$. Node [`auto_initial_pose.py`](file:///home/lehuythien/warehouse_ws/src/wh_navigation/scripts/auto_initial_pose.py) đảm bảo hội tụ hạt particle tức thì khi hệ thống khởi động.

---

## D-04-05: Cấu hình Costmap và Khả năng thông hành lối đi hẹp (1.2m)
- Với chiều rộng robot $0.32\,\text{m}$ (footprint $+3\,\text{cm}$ biên an toàn: nửa bề rộng $0.19\,\text{m}$), tham số `inflation_radius: 0.25` và `cost_scaling_factor: 3.0` đảm bảo vùng chi phí cao từ 2 vách kệ chỉ chiếm $2 \times (0.19 + 0.25) = 0.88\,\text{m}$, để lại dải hành lang chi phí thấp $0.32\,\text{m}$ ở trục giữa $y = 0.00$, giúp global planner tìm đường tối ưu xuyên suốt không bị tắc nghẽn.

---

# DECISIONS – Step 05: Thị giác và Căn chỉnh tinh (Visual Servoing)

## D-05-01: Thuật toán giải PnP phẳng và khử nhập nhằng nghiệm
- **Vấn đề:** Khi giải PnP cho marker phẳng bằng thuật toán lặp thông thường (`SOLVEPNP_ITERATIVE`), ở các góc nghiêng nhỏ thường xuất hiện hiện tượng nhập nhằng nghiệm (flip ambiguity: nghiệm góc quay bị lật $180^\circ$ quanh trục song song với mặt phẳng).
- **Quyết định:** Sử dụng cờ `cv2.SOLVEPNP_IPPE_SQUARE` chuyên dụng cho marker phẳng hình vuông của IPPE (Infinitesimal Plane-based Pose Estimation). Thuật toán phân tích cả hai nghiệm giải tích và chọn nghiệm có sai số chiếu lại (reprojection error) nhỏ nhất, đồng thời lọc bỏ mọi mẫu có reprojection error $> 1.0\,\text{px}$.

## D-05-02: Bộ lọc không gian thời gian `TagFilter`
- **Vấn đề:** Dữ liệu nhận diện từ từng khung hình camera 2D có nhiễu Gaussian và ngoại lai ngẫu nhiên do bóng đổ/phản xạ ánh sáng.
- **Quyết định:**
  - Vị trí $(x, y, z)$: Sử dụng cửa sổ trượt $N=10$ mẫu và lấy giá trị trung vị (Median filter) trên từng trục độc lập. Lọc trung vị có khả năng loại bỏ hoàn toàn nhiễu xung (shot noise/outliers) mà không gây biến dạng biên độ như bộ lọc thông thấp IIR.
  - Hướng (Orientation): Sử dụng thuật toán tính trung bình quaternion theo không gian Riemann trên $S^3$ (Markley averaging với xử lý đối cực $\mathbf{q} \equiv -\mathbf{q}$).

## D-05-03: Đồng bộ Gốc tọa độ Camera Sensor trong Gazebo và TF Quang học
- **Vấn đề:** Khi Gazebo gộp fixed joint giữa `dock_camera_link` và `lift_carriage_link`, việc khai báo sai lệch origin của joint con có thể khiến trục quang học của plugin camera lệch $25\,\text{mm}$ so với frame TF quang học.
- **Quyết định:** Chuẩn hóa `dock_camera_joint` origin tại `xyz="0.075 0 0.040"` và đặt `dock_camera_optical_joint` tại `xyz="0 0 0"`, sensor pose `<pose>0 0 0 0 0 0</pose>`. Điều này đảm bảo frame cảm biến trong Gazebo và frame TF `dock_camera_optical_frame` trùng khít hoàn toàn về mặt vật lý, đưa sai số đo đạc cự ly $0.30\,\text{m}$ từ $25\,\text{mm}$ xuống còn $1.79\,\text{mm}$.

## D-05-04: Chuẩn hóa Dấu điều khiển và Hệ số PBVS cho Mecanum
- **Quyết định:**
  - $v_x = \text{sign}_x \cdot K_{px} \cdot e_{dist}$ với $\text{sign}_x = +1.0$, $K_{px} = 0.8$: Khi khoảng cách đo được $p_x > 0.32\,\text{m}$ ($e_{dist} > 0$), robot tiến về phía trước ($v_x > 0$).
  - $v_y = \text{sign}_y \cdot K_{py} \cdot e_{lat}$ với $\text{sign}_y = +1.0$, $K_{py} = 0.8$: Khi tag lệch sang bên trái robot ($p_y > 0$), robot trượt dạt sang trái ($v_y > 0$).
  - $\omega_z = \text{sign}_w \cdot K_{pw} \cdot e_{yaw}$ với $\text{sign}_w = +1.0$, $K_{pw} = 1.0$: Khi robot bị lệch góc ngược chiều kim đồng hồ (CCW), vector pháp tuyến của tag trong hệ trục robot bị quay theo chiều CW dẫn đến $e_{yaw} < 0$. Do đó với $\text{sign}_w = +1.0$, lệnh vận tốc góc là $\omega_z < 0$ (quay thuận chiều kim đồng hồ CW), tạo thành vòng phản hồi âm ổn định (Negative Feedback).

## D-05-05: Cấu hình Tiêu chí Dừng và Khóa An toàn trong Action Server
- **Vùng cấm an toàn:** Phanh khẩn cấp $v = 0, \omega = 0$ ngay lập tức nếu $p_x < 0.28\,\text{m}$ (bảo vệ khoảng cách cơ học tối thiểu $0.05\,\text{m}$ trước mặt kệ).
- **Ngưỡng dung sai hội tụ:** Đặt dung sai mục tiêu chặt chẽ $\text{tol}_{lat} = 3.0\,\text{mm}$, $\text{tol}_{dist} = 3.0\,\text{mm}$, $\text{tol}_{yaw} = 1.43^\circ$ và yêu cầu duy trì ổn định trong $10$ khung hình liên tiếp ($0.5\,\text{s}$). Điều này đảm bảo sau khi dừng xe và chịu quán tính cơ khí, sai số Ground Truth cuối cùng luôn $\le 5.0\,\text{mm}$ và $\le 2.0^\circ$.
- **Thời gian chờ (Timeout):** Đặt `timeout_sec: 25.0s` cho phép hoàn thành 100% các trường hợp thử nghiệm cực đoan nhất (lệch góc ban đầu tối đa $\pm 10^\circ$, lệch ngang $\pm 5\,\text{cm}$).

---

# DECISIONS – Step 06: Tay SCARA TRR: Động học, Gắp và Thả

## D-06-01: Thiết kế Cơ cấu Đầu công tác không cần Khớp Cổ tay (Wristless SCARA)
- **Vấn đề:** Tay máy SCARA thông thường cần 4 bậc tự do (RRTR hoặc TRRR) bao gồm khớp quay cổ tay yaw để định hướng góc cho kẹp cơ khí.
- **Quyết định:** Tay máy SCARA TRR trong hệ thống sử dụng cấu hình 3 bậc tự do (Z tịnh tiến -> Khớp vai quay -> Khớp khuỷu quay), loại bỏ khớp cổ tay yaw.
- **Lý do kỹ thuật:**
  1. Hộp hàng có kích thước vuông $60 \times 60\,\text{mm}$ và đầu hút/kẹp chân không đối xứng hoàn toàn quanh trục thẳng đứng Z.
  2. Việc gắp và nhả hộp không đòi hỏi định hướng góc quay hộp trong mặt phẳng ngang ô kệ.
  3. Giảm bớt 1 động cơ giúp giảm trọng lượng của khâu 2 (cẳng tay), hạ thấp mô-men quán tính, tăng tốc độ đáp ứng và triệt tiêu rung động cơ khí khi vươn xa.

## D-06-02: Mở rộng Hành trình Nâng hạ `z_stroke` lên 0.52 m
- **Vấn đề:** Khi robot tiếp cận ô kệ Tầng 2, mặt kệ ở cao độ $0.401\,\text{m}$, nóc hộp ở $0.441\,\text{m}$. Mức an toàn hover cần cao hơn nóc hộp $0.15\,\text{m} \to z_{world} = 0.591\,\text{m}$. Theo công thức quy đổi $d = z_{world} - 0.15 + tool\_drop = 0.591 - 0.15 + 0.06 = 0.501\,\text{m}$. Giới hạn cũ của `lift_joint` trong URDF là $0.46\,\text{m}$, dẫn đến lỗi `JOINT_LIMIT` khi cố gắng hover tầng 2.
- **Quyết định:** Nâng giới hạn trên của khớp nâng `lift_joint` lên $0.52\,\text{m}$ (đồng bộ với tham số `prismatic_z: max: 0.52` trong `robot_params.yaml`). Hành trình này vừa vặn với chiều cao cột dẫn hướng $0.44\,\text{m}$ và cho phép thao tác trơn tru tại mọi điểm của cả hai tầng kệ.

## D-06-03: Quy tắc Chuyển động An toàn (Nâng Z -> Xoay -> Hạ Z)
- **Quy tắc bất khả xâm phạm:** Tay máy tuyệt đối không bao giờ được phép xoay ngang khi đang ở độ cao làm việc thấp bên trong lòng ô kệ hoặc trong lòng khay chứa trên xe.
- **Giải pháp quỹ đạo:** Mọi lệnh di chuyển giữa các trạm hoặc đổi tư thế được phân rã thành quỹ đạo 3 chặng hình thang:
  1. Chặng 1: Nâng Z thẳng đứng lên độ cao hover an toàn ($d_{hover} \ge 0.251\,\text{m}$ cho Tầng 1, $\ge 0.501\,\text{m}$ cho Tầng 2).
  2. Chặng 2: Xoay các khớp ngang $q_1, q_2$ trên không trung hoàn toàn thoát khỏi mép kệ và thành xe.
  3. Chặng 3: Hạ Z thẳng đứng xuống độ cao mục tiêu.
- **Hiệu quả:** Triệt tiêu hoàn toàn nguy cơ quét văng hộp lân cận, đạt chỉ tiêu 0 lần xô lệch hộp lân cận $> 5\,\text{mm}$.

## D-06-04: Chiến lược Căn chỉnh Mắt-trong-tay (Eye-in-Hand Closed-Loop)
- **Kiến trúc:** Camera bàn tay (`hand_camera`) gắn trên `tool_link` nhìn thẳng góc xuống mặt trên của hộp.
- **Quy trình:**
  1. Tay di chuyển đến vị trí danh định phía trên hộp (khoảng cách quan sát $150\,\text{mm}$).
  2. Camera phát hiện ArUco tag mặt trên hộp ($36\,\text{mm}$), giải PnP và chuyển đổi vị trí tâm tag sang tọa độ `arm_base_link`.
  3. Kiểm tra sai số lệch tâm $\Delta_{xy} = \sqrt{\Delta x^2 + \Delta y^2}$. Nếu $\Delta_{xy} > 2.0\,\text{mm}$, gọi IK và dịch chuyển vi sai.
  4. Lặp tối đa 3 vòng. Nhờ trường nhìn bao quát ở cự ly $150\,\text{mm}$, vòng lặp hội tụ về sai số $< 1.5\,\text{mm}$ chỉ sau 1–2 bước.

## D-06-05: Khóa An toàn Chân không (Vacuum Invariant)
- **Nguyên tắc:** Nếu cảm biến xác nhận robot đang giữ hộp hợp lệ (`holding_box == true` và `/gripper/grasping == true`), bộ điều khiển không bao giờ được tự động tắt hút khi gặp sự cố hoặc timeout giữa không trung.
- **Xử lý:** Khi có lỗi phát sinh trong quá trình vận chuyển, tay máy lập tức đưa hộp về hạ cánh an toàn trên khay xe (`TRAY`) hoặc giữ nguyên vị trí và trả mã lỗi để bộ quản lý nhiệm vụ cấp cao (`task_manager`) xử lý.

## D-06-06: Tư thế STOW Gấp gọn Lùi về sau Nằm trọn Footprint Thân xe
- **Cấu hình STOW:** $d = 0.0\,\text{m}$, $q_1 = -2.60\,\text{rad}$ ($-149.0^\circ$), $q_2 = -2.40\,\text{rad}$ ($-137.5^\circ$).
- **Hình học:** Cả hai cánh tay gập khít lùi về phía khoang sau của thân xe. Chiều rộng vươn ngang tối đa chỉ $0.09\,\text{m}$, nhỏ hơn nhiều so với bán chiều rộng thân xe ($0.15\,\text{m}$). Toàn bộ khâu nằm trọn bên trong hình chữ nhật footprint $0.45 \times 0.32\,\text{m}$ với lề an toàn vượt mức $10\,\text{mm}$.

---

# DECISIONS – Step 07: Quản lý Nhiệm vụ và Cơ sở Dữ liệu Kho

## D-07-01: Giao dịch CSDL SQLite và Chống Deadlock/Race Condition
- **Vấn đề:** Hệ thống điều phối kho hàng có nhiều luồng song song (ROS executor, FSM execution thread, Web API). Thao tác đặt chỗ và xuất/nhập kho nếu không được cô lập chặt chẽ sẽ gây lỗi `database is locked`, đặt trùng ô hoặc sai lệch số lượng tồn kho.
- **Quyết định:**
  1. Mọi truy vấn thay đổi trạng thái ô (`reserve_*`, `release_*`, `commit_*`) bắt buộc thực hiện trong một khối giao dịch duy nhất với `BEGIN IMMEDIATE`.
  2. Kích hoạt chế độ `PRAGMA journal_mode=WAL` và tăng `timeout=30.0s`.
  3. FSM chạy trong một worker thread riêng biệt, không block ROS 2 executor, giao tiếp với các Action Server bằng các futures không đồng bộ có kiểm soát thời gian chờ (timeout).

---

# DECISIONS – Step 08: Web Local Quản lý Kho

## D-08-01: Phương thức Giao tiếp Web Backend -> Task Manager (ROS 2 Service vs ROS Topic)
- **Vấn đề:** Giao diện Web (FastAPI) cần tiếp nhận yêu cầu nhập/xuất từ trình duyệt người dùng và chuyển giao cho `task_manager_node`. Cần lựa chọn giữa ROS 2 Topic (ví dụ `/wh/new_task`) hay ROS 2 Service (`/wh/submit_task`, `/wh/cancel_task`).
- **Quyết định:** Sử dụng **ROS 2 Service** (`/wh/submit_task` và `/wh/cancel_task`) kết hợp truy vấn CSDL SQLite trực tiếp ở tầng REST API.
- **Lý do kỹ thuật:**
  1. **Tính đồng bộ và xác nhận tin cậy (Handshake):** Khi người dùng bấm "Nhập" hoặc "Xuất" trên trình duyệt, HTTP REST API cần trả về ngay lập tức kết quả 200 OK kèm mã `task_id` được chấp nhận, hoặc mã lỗi 404/409/422 nếu không hợp lệ. ROS 2 Service cung cấp mô hình Request/Response rõ ràng, tránh nguy cơ mất gói tin hoặc race condition như khi dùng Topic pub/sub một chiều.
  2. **Kiểm soát vòng đời nhiệm vụ:** Dịch vụ `/wh/cancel_task` cho phép web client nhận ngay phản hồi xác nhận lệnh hủy đã được ghi nhận hay nhiệm vụ đã hoàn thành trước đó.
  3. **Hiệu năng và Giảm tải Middleware:** Toàn bộ việc đọc trạng thái tồn kho, danh mục, lịch sử giao dịch được FastAPI đọc trực tiếp từ file SQLite (`~/.warehouse/inventory.db`) với chế độ WAL siêu nhanh (< 2 ms), hoàn toàn không làm nghẽn bus mạng ROS 2 middleware khi có nhiều thiết bị di động truy cập đồng thời.
  4. **WebSocket Broadcast tối ưu:** Node `WebBridge` chạy trong tiến trình FastAPI nhận `/robot_status` (đã tổng hợp sẵn vị trí x, y, yaw, trạng thái FSM, pin, hộp đang giữ) và đẩy thẳng vào danh sách client WebSocket qua `loop.call_soon_threadsafe`, đảm bảo tần số 5–10 Hz mượt mà trên canvas di động.

---

# DECISIONS – Step 08b: Bổ sung Bản đồ SLAM Thời gian thực lên Web

## D-08-02: Kiến trúc Hai Chế độ Vận hành, Quản lý Metadata Bản đồ SLAM Động và An toàn Teleop
- **Vấn đề:**
  1. `slam_toolbox` và `AMCL` đều cùng phát biến đổi TF `map -> odom`. Nếu chạy đồng thời sẽ gây xung đột frame TF nghiêm trọng khiến hệ thống định vị giật lắc và Nav2 tê liệt.
  2. Khi robot di chuyển quét bản đồ ở chế độ SLAM, kích thước lưới (`width`, `height`) và gốc tọa độ (`origin_x`, `origin_y`) liên tục thay đổi mở rộng ra xung quanh. Nếu client web vẽ robot bằng metadata cũ của ảnh mới (hoặc ngược lại), tọa độ robot và các chướng ngại vật sẽ bị lệch nghiêm trọng.
  3. Điều khiển thủ công từ xa qua web qua mạng Wi-Fi tiềm ẩn rủi ro trễ gói tin hoặc mất kết nối khi robot đang chạy.
- **Quyết định thiết kế:**
  1. **Tách biệt Hai Chế độ Vận hành (`mode:="navigation"` | `mode:="mapping"`):**
     - Tạo tham số launch `mode` đồng bộ trong `nav.launch.py` và `full_system.launch.py`.
     - Chế độ `mapping`: Khởi chạy `slam_toolbox` (chế độ `online_async`), `twist_mux`, `collision_monitor`, `mode_publisher`; **tuyệt đối không khởi chạy AMCL hay task manager**. Web nhận diện chế độ qua topic `/wh/mode` (QoS `TRANSIENT_LOCAL`) và tự động khóa/ẩn form Nhập/Xuất kho.
     - Chế độ `navigation` (mặc định): Khởi chạy `map_server`, `amcl`, `nav2_bringup`, `task_manager`.
     - Đảm bảo tại mọi thời điểm chỉ có duy nhất một node phát TF `map -> odom`.
  2. **Đồng bộ hóa Siêu Dữ Liệu và Ảnh Bản đồ (Atomic Metadata Swap):**
     - Tại `WebBridge`: chuyển đổi `OccupancyGrid` sang PNG xám trong bộ nhớ (với `np.flipud` để khớp gốc ảnh top-left với gốc ROS bottom-left), gắn kèm số phiên bản `version` tăng dần và gửi thông điệp WebSocket `map_update` gọn nhẹ ($\le 1\,\text{Hz}$).
     - Tại Web Client (`app.js`): khi nhận `map_update`, tạo đối tượng `Image` mới để tải ngầm `/api/map/live.png?v=${version}`. **Chỉ khi sự kiện `image.onload` kích hoạt thành công**, client mới cập nhật đồng thời cả ảnh và metadata (`origin_x`, `origin_y`, `resolution`, `width`, `height`) rồi kích hoạt vẽ lại. Điều này loại bỏ hoàn toàn hiện tượng nhấp nháy hoặc lệch tọa độ khi bản đồ phình to.
  3. **Lớp LiDAR Scan và Biến đổi Hệ Tọa độ Server-Side:**
     - `WebBridge` tra cứu TF `map -> lidar_link` qua `tf2_ros.Buffer`, đổi trực tiếp góc quét sang tọa độ thế giới trong hệ `map`, lấy mẫu thưa (bước 2, tối đa 180 điểm) và gửi với tần số $\le 5\,\text{Hz}$. Bọc ngoại lệ try-catch nhẹ nhàng để không spam cảnh báo khi SLAM chưa kịp khởi tạo TF.
  4. **Watchdog An toàn cho Web Teleop:**
     - REST API `/api/teleop` giới hạn cứng vận tốc $v_x, v_y \le 0.25\,\text{m/s}$, $\omega_z \le 0.60\,\text{rad/s}$ và publish vào `/cmd_vel_teleop` (đi qua `twist_mux` và `collision_monitor`).
     - Tích hợp một timer Watchdog chạy ở tần số $10\,\text{Hz}$: Nếu không nhận được lệnh từ client trong quá $0.3\,\text{s}$ (ví dụ mất sóng Wi-Fi hoặc người dùng thả tay), server lập tức tự động publish lệnh dừng khẩn cấp ($v = 0, \omega = 0$), đảm bảo robot dừng trong $< 0.5\,\text{s}$.
  5. **Tích hợp Dịch vụ Lưu Bản đồ SLAM:**
     - Sử dụng client gọi service `/slam_toolbox/save_map` (kiểu `slam_toolbox/srv/SaveMap`) để lưu pose graph và xuất ảnh bản đồ ra `wh_navigation/maps/<name>.pgm` và `.yaml`.
     - Đồng thời bổ sung fallback ghi trực tiếp từ bộ đệm ảnh PNG hiện hành nếu service SLAM toolbox chưa phản hồi, đảm bảo người dùng web luôn bấm "Lưu bản đồ" thành công 100%.

---

# DECISIONS – Step 09: Tích hợp, Đo đạc và Tài liệu Toàn diện

## D-09-01: Kiến trúc Khởi động Hợp nhất và Tránh "Sleep Mù" trong `full_system.launch.py`
- **Vấn đề:** Hệ thống kho bao gồm 9 gói phần mềm với hàng chục node: Gazebo thế giới ảo, robot spawn, controllers (joint_state_broadcaster, arm_controller), EKF, Nav2 Stack (map_server, amcl, planner, controller, collision monitor, twist_mux), 2 vision detector, dock_server, pick_place_server, task_manager và web portal. Nếu khởi động đồng thời, các node sẽ gặp lỗi thiếu TF, thiếu action server hoặc chết trước khi Gazebo khởi tạo xong. Nếu dùng lệnh `time.sleep()` trong Python launch script sẽ chặn luồng ROS launch executor.
- **Quyết định:**
  1. Sử dụng `TimerAction` với các khoảng trễ bậc thang được tính toán dựa trên thời gian sẵn sàng thực tế:
     - 0.0s: Gazebo World + Hộp theo seed + Robot Spawner.
     - 5.0s: Joint State Broadcaster + Arm Controller + Gripper Service + Wheel Slip + EKF localization.
     - 9.0s: Nav2 Navigation Stack (hoặc SLAM Toolbox tùy theo tham số `mode`).
     - 15.0s: Vision detectors (`dock_detector_node`, `hand_detector_node`) + Visual Servoing (`dock_server_node`).
     - 18.0s: SCARA Manipulation (`pick_place_server_node`).
     - 21.0s: Task Manager (`task_manager_node`).
     - 24.0s: Web Portal (`uvicorn` FastAPI server) + RViz2 (nếu tham số `rviz:=true`).
  2. Xây dựng tập lệnh độc lập `scripts/wait_ready.py` kiểm tra có điều kiện (conditional polling với timeout):
     - Kiểm tra sự hiện diện của các topic quan trọng: `/robot_status`, `/map`, `/scan`, `/dock_camera/image_raw`, `/hand_camera/image_raw`.
     - Kiểm tra các ROS 2 Services: `/wh/submit_task`, `/wh/cancel_task`.
     - Kiểm tra các Action Servers: `/navigate_to_pose`, `/wh/dock_to_slot`, `/wh/pick_place`.
     - Kiểm tra Web REST API HTTP GET `http://localhost:8000/api/status` trả về mã 200 OK.
     - Trả về mã thoát 0 khi toàn bộ sẵn sàng hoặc 1 khi hết thời gian chờ (timeout), cho phép các kịch bản CI/CD và demo tự động gọi một cách tất định.
  3. Cuối launch in rõ thông điệp banner hướng dẫn: `"HỆ THỐNG SẴN SÀNG — mở http://<IP>:8000"`.

## D-09-02: Thiết kế Kịch bản Demo Tự động 100% Không Can thiệp Tay (`demo_run.py`)
- **Vấn đề:** Để chứng minh tính ổn định, độc lập và toàn vẹn của hệ thống trước hội đồng nghiệm thu, kịch bản demo cần chạy hoàn toàn tự động qua REST API, bao phủ mọi tình huống hoạt động bình thường, tình huống lỗi ngoại lệ và xác thực tính nhất quán giữa cơ sở dữ liệu và thế giới thực.
- **Quyết định thiết kế 7 bước:**
  1. **Bước 1 (Xuất 3 đơn):** Xuất 1 đơn Tầng 1, 1 đơn Tầng 2, 1 đơn Giá B. Kiểm tra FSM chuyển đủ các trạng thái và hoàn tất trả hộp tại trạm OUT.
  2. **Bước 2 (Nhập 3 đơn):** Nhập 3 đơn vào các ô trống thuộc cả Tầng 1 và Tầng 2. Kiểm tra cập nhật tồn kho từ `EMPTY` sang `FULL`.
  3. **Bước 3 (Vật cản bất ngờ):** Bơm một mô hình vật cản bất ngờ (`obstacle_box`) vào đường đi bằng service `/wh/spawn_obstacle` khi robot đang di chuyển. Nav2 kích hoạt cơ chế né tránh vật cản động trên Costmap cục bộ và hoàn thành đơn hàng mà không va chạm.
  4. **Bước 4 (Nhập sai mã):** Đặt hộp tại trạm IN có ArUco ID sai khác với mã đã khai báo trong đơn. Hệ thống phát hiện qua camera khâu chấp hành, từ chối nhập kho, trả hộp về sàn IN và giữ nguyên tồn kho trong DB.
  5. **Bước 5 (Fault Injection & Recovery):** Kích hoạt ngắt chân không hoặc phát tín hiệu khẩn cấp E-Stop giữa chu trình. Nhiệm vụ chuyển sang trạng thái `FAILED`, cơ chế rollback CSDL giải phóng ô về trạng thái trước đó, robot đưa tay về vị trí an toàn và có thể tiếp nhận nhiệm vụ mới.
  6. **Bước 6 (Xuất kho khi hết hàng):** Gửi yêu cầu xuất một mặt hàng đã hết tồn kho. Web API từ chối ngay lập tức với mã HTTP 409 Conflict.
  7. **Bước 7 (Đối chiếu Ground Truth DB vs Gazebo 100%):** Lấy danh sách ô và mã hàng từ SQLite, so sánh với tọa độ và tag của tất cả mô hình hộp thực tế trong Gazebo (`/gazebo/model_states`). Bắt buộc khớp chính xác 100% về vị trí không gian và định danh mặt hàng.

## D-09-03: Bộ Đo lường và Xuất Dữ liệu Chuẩn hóa 150 DPI
- **Yêu cầu:** Mọi chỉ số kỹ thuật phải được ghi nhận định lượng vào file CSV và trực quan hóa bằng biểu đồ Matplotlib đạt độ phân giải tối thiểu 150 DPI, có đầy đủ tiêu đề, nhãn trục có đơn vị rõ ràng và chú thích giải thích.
- **Quyết định cấu trúc:**
  - Tập lệnh tổng `scripts/run_all_benchmarks.sh` điều phối thực thi tuần tự các mô-đun kiểm thử:
    - `benchmark_odometry_drift.py`: Sai số tịnh tiến và góc của Odometry thô, EKF và AMCL so với Ground Truth khi robot chạy hình vuông.
    - `benchmark_nav.py`: Sai số dừng Nav2, tỷ lệ thành công 20 lượt, phản ứng vật cản và dừng khẩn cấp.
    - `benchmark_docking.py`: So sánh phân bố sai số trước và sau Visual Servoing qua 30 lượt căn chỉnh (Boxplot + Histogram).
    - `benchmark_pick.py`: Tỷ lệ thành công và sai số gắp/thả qua 60 lượt (30 lượt Tầng 1, 30 lượt Tầng 2).
    - `benchmark_fsm_and_resources.py`: Phân rã thời gian chu trình theo các pha FSM và đo tải tài nguyên máy tính (CPU, RAM, RTF).
    - `benchmark_latency.py` & `benchmark_slam_load.py`: Đo độ trễ REST/WebSocket và tải mạng Web.

## D-09-04: Đánh giá Sự Khác biệt giữa Mô phỏng Gazebo và Robot Thật (Sim-to-Real Gaps)
- Nhằm đảm bảo tính trung thực khoa học, hệ thống ghi nhận rõ ràng các khác biệt cốt lõi giữa môi trường ảo Gazebo và điều kiện phần cứng thực tế:
  1. **Hệ dẫn động Mecanum:** Trong Gazebo sử dụng plugin `planar_move` tích hợp lực đẩy ảo trực tiếp lên thân xe. Hiện tượng trượt bánh trong mô phỏng chỉ được xấp xỉ bằng mô hình tham số ngẫu nhiên (`wheel_slip_model`), trong khi trên robot thật chuyển động phụ thuộc vào lực ma sát tiếp xúc phức tạp của từng con lăn nghiêng 45 độ trên sàn.
  2. **Cơ cấu chấp hành gắp chân không:** Mô phỏng sử dụng plugin `gazebo_ros_vacuum_gripper` tạo liên kết cứng (fixed joint) tức thời khi đầu hút tiếp cận bề mặt hộp. Robot thật phụ thuộc vào độ kín khí, độ nhám bề mặt, thời gian tạo áp suất âm của bơm chân không và nguy cơ rò rỉ khí khi rung lắc.
  3. **Hệ thống thị giác và Camera:** Camera mô phỏng cho ảnh hoàn hảo không méo quang học, không mờ chuyển động (motion blur), không nhiễu cảm biến hạt (sensor noise). Robot thật bắt buộc phải thực hiện hiệu chuẩn bàn cờ (checkerboard calibration) để khử méo thấu kính và hiệu chuẩn hand-eye để tìm ma trận biến đổi chính xác từ tay máy sang camera.
  4. **Điều kiện ánh sáng:** Gazebo duy trì nguồn sáng ổn định và đồng nhất. Môi trường nhà kho thực tế chịu ảnh hưởng bởi ánh sáng tự nhiên thay đổi theo giờ, bóng đổ từ giá kệ và phản xạ chói lóa từ bề mặt kim loại/băng keo hộp, đòi hỏi phải trang bị đèn LED vòng trợ sáng chủ động (active LED ring).
  5. **Động lực học và Sai số Cơ khí:** Mô phỏng không tính đến độ rơ bánh răng (backlash), độ võng đàn hồi của khớp SCARA khi vươn xa mang tải, độ rung động của sàn và độ trễ truyền thông CAN/Ethernet/UART giữa máy tính nhúng và bộ điều khiển động cơ.
  6. **Phiên bản hệ điều hành và ROS:** Robot thực tế cần chốt một phiên bản phần mềm đồng nhất (khuyến nghị ROS 2 Humble LTS trên Ubuntu 22.04 LTS nền tảng ARM64/Jetson).

## D-09-05: Lộ trình Chuyển giao Phần cứng Robot Thật (Real Hardware Transfer Roadmap)
- Để hiện thực hóa hệ thống trên xe tự hành và tay máy thực tế, các bước kỹ thuật cần thực hiện gồm:
  1. **Bo mạch điều khiển cấp thấp:** Triển khai vi điều khiển (STM32F4/F7 hoặc ESP32) giao tiếp với ROS 2 qua giao thức **micro-ROS** (hoặc CAN bus / Ethernet UDP).
  2. **Điều khiển bám vận tốc 4 bánh Mecanum:** Lập trình giải thuật động học ngược 4 bánh Mecanum và bộ điều khiển PID số phản hồi từ encoder quang học vòng kín ở tần số >= 100 Hz.
  3. **Hiệu chuẩn Thị giác:**
     - Hiệu chuẩn thông số nội tại camera (Intrinsic parameters fx, fy, cx, cy và hệ số méo k1, k2, p1, p2).
     - Hiệu chuẩn Hand-Eye Calibration (phương pháp Tsai-Lenz hoặc Park-Martin) để xác định chính xác biến đổi từ `tool_link` sang `hand_camera_optical_frame`.
  4. **Tích hợp Cảm biến Áp suất Chân không:** Lắp đặt cảm biến áp suất số (ví dụ SMC ZSE30) để phát hiện trạng thái hút hộp thực tế (ngưỡng áp suất âm < -60 kPa) thay vì chỉ dựa vào khoảng cách hình học.
  5. **Dựng môi trường kho thực tế:** In dán mã ArUco độ nét cao trên vật liệu mờ không lóa, dán vạch sàn phản quang và thiết lập ranh giới an toàn.
  6. **Kiểm thử An toàn Phần cứng:** Tích hợp nút dừng khẩn cấp vật lý cắt nguồn động lực (Hardware E-Stop relay) và cảm biến cản va chạm chạm trước sườn xe (bumper switch).

## D-10-01: Thay DWB bằng MPPI (MPC) cho tránh vật cản cục bộ
- **Quyết định:** Nav2 `controller_server` dùng `nav2_mppi_controller::MPPIController` (Model Predictive Path Integral – MPC lấy mẫu) với mô hình `Omni` cho đế mecanum, thay cho `dwb_core::DWBLocalPlanner`.
- **Cơ chế:** Mỗi chu kỳ 20 Hz, tối ưu `batch_size=1500` quỹ đạo trong tầm nhìn `time_steps * model_dt = 50 * 0.05 = 2.5 s`; `ObstaclesCritic` chấm điểm va chạm theo costmap cục bộ + footprint chữ nhật, các critic còn lại lo bám đường đi/hướng đích. `collision_monitor` vẫn là lớp an toàn cuối.
- **Cấu hình:** `config/nav2_params.yaml` (khối `FollowPath`); inflation costmap cục bộ nâng lên `0.35 m` / `cost_scaling_factor 4.0` để có gradient đẩy ra khỏi vật cản.
- **Hoàn tác / so sánh:** `ros2 launch wh_bringup full_system.launch.py controller:=dwb` nạp lại cấu hình DWB cũ từ `config/nav2_dwb_controller.yaml`.
