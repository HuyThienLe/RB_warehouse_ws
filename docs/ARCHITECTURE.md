# KIẾN TRÚC HỆ THỐNG AGV-SCARA WAREHOUSE (ARCHITECTURE SPECIFICATION)

Tài liệu này mô tả chi tiết kiến trúc phần cứng, phần mềm, luồng dữ liệu, cây tọa độ TF, máy trạng thái hữu hạn (FSM) và giải pháp điều khiển ba tầng độ chính xác của hệ thống AGV kết hợp tay máy SCARA trong kho thông minh.

---

## 1. SƠ ĐỒ KHỐI CÁC NODE VÀ TOPIC CHÍNH (NODE/TOPIC GRAPH)

Hệ thống được thiết kế theo kiến trúc hướng dịch vụ và hướng dữ liệu phân tán trên nền tảng **ROS 2 Humble**, kết nối với giao diện web qua FastAPI và WebSocket.

```mermaid
flowchart TB
    subgraph Simulation ["Gazebo 11 Physics Engine"]
        GZ_ROBOT["Gazebo Model AMR+SCARA"]
        GZ_SENSORS["LiDAR + Cameras + IMU + Contact"]
    end

    subgraph Hardware_Interface ["Controllers & Drivers"]
        DIFF_DRIVE["planar_move / wheel_controllers"]
        JSB["joint_state_broadcaster"]
        ARM_CTRL["arm_controller (JointTrajectoryController)"]
        GRIPPER_SRV["vacuum_gripper_service"]
        SLIP_PLUGIN["wheel_slip_model"]
    end

    subgraph Perception_Localization ["Perception & Localization"]
        EKF["robot_localization (EKF Node)"]
        AMCL["Nav2 AMCL / Map Server"]
        DOCK_CAM["wh_vision: dock_detector_node"]
        HAND_CAM["wh_vision: hand_detector_node"]
    end

    subgraph Navigation_Control ["Navigation & Visual Servoing"]
        NAV2_BT["Nav2 BT Navigator / Planner / Controller"]
        TWIST_MUX["twist_mux"]
        COLLISION_MONITOR["collision_monitor"]
        DOCK_SERVER["wh_navigation: dock_server_node"]
    end

    subgraph Manipulation ["SCARA Manipulation"]
        PICK_PLACE_SRV["wh_manipulation: pick_place_server_node"]
        IK_SOLVER["Closed-form Analytic IK & Trajectory Gen"]
    end

    subgraph Mission_Management ["Task Management & Database"]
        TASK_MGR["wh_task_manager: task_manager_node"]
        SQLITE_DB[("inventory.db (SQLite WAL)")]
    end

    subgraph User_Interface ["Web & Cloud Portal"]
        WEB_BACKEND["wh_web: FastAPI App + WebBridge"]
        WEB_FRONTEND["Responsive Web UI (Canvas + Teleop + Orders)"]
    end

    %% Sensor links
    GZ_SENSORS -->|/scan| NAV2_BT
    GZ_SENSORS -->|/imu/data| EKF
    GZ_SENSORS -->|/odom_raw| EKF
    GZ_SENSORS -->|/dock_camera/image_raw| DOCK_CAM
    GZ_SENSORS -->|/hand_camera/image_raw| HAND_CAM

    %% Localization
    EKF -->|/odometry/filtered| AMCL
    AMCL -->|TF: map -> odom| NAV2_BT

    %% Vision outputs
    DOCK_CAM -->|/dock_camera/tag_detections| DOCK_SERVER
    HAND_CAM -->|/hand_camera/tag_detections| PICK_PLACE_SRV

    %% Task and High Level
    WEB_BACKEND <-->|REST API + WS| WEB_FRONTEND
    WEB_BACKEND -->|/wh/submit_task, /wh/cancel_task| TASK_MGR
    WEB_BACKEND <-->|SQL Queries| SQLITE_DB
    TASK_MGR <-->|Transactions| SQLITE_DB

    %% FSM Execution
    TASK_MGR -->|Action: NavigateToPose| NAV2_BT
    TASK_MGR -->|Action: DockToSlot| DOCK_SERVER
    TASK_MGR -->|Action: PickPlace| PICK_PLACE_SRV
    TASK_MGR -->|/robot_status| WEB_BACKEND

    %% Velocity Chain
    NAV2_BT -->|/cmd_vel_nav| TWIST_MUX
    DOCK_SERVER -->|/cmd_vel_dock| TWIST_MUX
    WEB_BACKEND -->|/cmd_vel_teleop| TWIST_MUX
    TWIST_MUX -->|/cmd_vel_mux| COLLISION_MONITOR
    COLLISION_MONITOR -->|/cmd_vel| DIFF_DRIVE

    %% Manipulation Execution
    PICK_PLACE_SRV -->|Trajectory Action| ARM_CTRL
    PICK_PLACE_SRV -->|/gripper/switch| GRIPPER_SRV
```

---

## 2. CHUỖI ĐIỀU KHIỂN VẬN TỐC VÀ CƠ CHẾ AN TOÀN (`cmd_vel` MULTIPLEXING CHAIN)

Chuỗi điều khiển vận tốc đảm bảo phân định quyền ưu tiên tuyệt đối giữa các nguồn phát lệnh di chuyển, đồng thời lớp giám sát va chạm (Collision Monitor) đóng vai trò lá chắn an toàn tối hậu trước khi xung vận tốc được chuyển xuống phần cứng/mô phỏng.

```mermaid
flowchart LR
    SRC_TELEOP["/cmd_vel_teleop\n(Web D-pad / Joystick)\n[Priority 100]"]
    SRC_DOCK["/cmd_vel_dock\n(Visual Servoing PID)\n[Priority 50]"]
    SRC_NAV["/cmd_vel_nav\n(Nav2 Controller Server)\n[Priority 10]"]
    SRC_ESTOP["/estop = true\n(E-Stop Software/Hardware)"]

    subgraph Mux_Stage ["twist_mux Node"]
        LOCK_CHECK{"Khóa E-Stop\nđang kích hoạt?"}
        PRIORITY_EVAL{"So sánh độ ưu tiên\nvà timeout (0.5s)"}
        CMD_SELECTED["/cmd_vel_mux\n(Vận tốc thô được chọn)"]
    end

    subgraph Safety_Stage ["collision_monitor Node"]
        LIDAR_SCAN["/scan (2D LiDAR)"]
        ZONE_SLOW["Vùng Giảm tốc Slowdown\n(Khoảng cách < 0.40m)"]
        ZONE_STOP["Vùng Dừng Dừng khẩn Stop\n(Khoảng cách < 0.18m)"]
        VEL_SCALE["Scale vận tốc / Kẹp về 0"]
    end

    ACTUATOR["/cmd_vel\nRobot Mecanum Base\n(planar_move / Controller)"]

    SRC_ESTOP --> LOCK_CHECK
    SRC_TELEOP --> PRIORITY_EVAL
    SRC_DOCK --> PRIORITY_EVAL
    SRC_NAV --> PRIORITY_EVAL

    LOCK_CHECK -->|Có: Phát v=0| CMD_SELECTED
    LOCK_CHECK -->|Không| PRIORITY_EVAL
    PRIORITY_EVAL --> CMD_SELECTED

    CMD_SELECTED --> Safety_Stage
    LIDAR_SCAN --> ZONE_SLOW
    LIDAR_SCAN --> ZONE_STOP
    ZONE_SLOW --> VEL_SCALE
    ZONE_STOP --> VEL_SCALE
    VEL_SCALE --> ACTUATOR
```

---

## 3. CÂY TỌA ĐỘ TOÀN DIỆN (TF2 COORDINATE TREE)

Cây tọa độ đảm bảo không có vòng lặp (acyclic) và duy nhất một node phát biểu diễn biến đổi `map -> odom`.

```mermaid
flowchart TD
    MAP["map\n(Tọa độ toàn cục bản đồ)"]
    ODOM["odom\n(Tọa độ odometry đã lọc)"]
    BASE_FOOTPRINT["base_footprint\n(Hình chiếu mặt sàn robot)"]
    BASE_LINK["base_link\n(Tâm cơ sở thân xe robot)"]

    %% Sensors
    LIDAR_LINK["lidar_link\n(2D LiDAR Scanner)"]
    IMU_LINK["imu_link\n(Cảm biến quán tính)"]
    DOCK_CAM_LINK["dock_camera_link\n(Camera sườn xe căn chỉnh ArUco kệ)"]
    DOCK_CAM_OPT["dock_camera_optical_frame\n(Khung quang học z hướng tới)"]

    %% Wheels
    WHEEL_FL["wheel_fl_link\n(Bánh Mecanum Trước-Trái)"]
    WHEEL_FR["wheel_fr_link\n(Bánh Mecanum Trước-Phải)"]
    WHEEL_RL["wheel_rl_link\n(Bánh Mecanum Sau-Trái)"]
    WHEEL_RR["wheel_rr_link\n(Bánh Mecanum Sau-Phải)"]

    %% SCARA Arm
    ARM_BASE["arm_base_link\n(Gốc đế tay máy x=-0.08, y=0, z=0.12)"]
    LIFT_LINK["lift_link\n(Khâu trượt lăng trụ Prismatic Z)"]
    LINK1["link1\n(Khâu xoay 1 Revolute Yaw q1)"]
    LINK2["link2\n(Khâu xoay 2 Revolute Yaw q2)"]
    TOOL_LINK["tool_link\n(Khâu chấp hành mang đầu hút)"]
    HAND_CAM_LINK["hand_camera_link\n(Camera bàn tay nhìn vuông góc xuống)"]
    HAND_CAM_OPT["hand_camera_optical_frame\n(Khung quang học camera tay)"]
    VACUUM_PAD["vacuum_pad_link\n(Điểm tiếp xúc cốc hút chân không)"]

    %% External tags
    TAG_WORLD["aruco_tag_XX\n(Tọa độ tag quan sát trong thế giới)"]

    MAP -->|AMCL (hoặc slam_toolbox)| ODOM
    ODOM -->|robot_localization EKF| BASE_FOOTPRINT
    BASE_FOOTPRINT -->|static_tf| BASE_LINK

    BASE_LINK --> LIDAR_LINK
    BASE_LINK --> IMU_LINK
    BASE_LINK --> DOCK_CAM_LINK
    DOCK_CAM_LINK --> DOCK_CAM_OPT

    BASE_LINK --> WHEEL_FL
    BASE_LINK --> WHEEL_FR
    BASE_LINK --> WHEEL_RL
    BASE_LINK --> WHEEL_RR

    BASE_LINK --> ARM_BASE
    ARM_BASE -->|lift_joint (d_z)| LIFT_LINK
    LIFT_LINK -->|joint1 (q1)| LINK1
    LINK1 -->|joint2 (q2)| LINK2
    LINK2 -->|static| TOOL_LINK
    TOOL_LINK --> HAND_CAM_LINK
    HAND_CAM_LINK --> HAND_CAM_OPT
    TOOL_LINK --> VACUUM_PAD

    DOCK_CAM_OPT -.->|solvePnP| TAG_WORLD
    HAND_CAM_OPT -.->|solvePnP| TAG_WORLD
```

---

## 4. MÁY TRẠNG THÁI HỮU HẠN NHẬP VÀ XUẤT (FSM TRANSITION DIAGRAMS)

Quy trình quản lý nhiệm vụ kho được thiết kế dưới dạng State Machine chịu lỗi cao (fault-tolerant) với các điểm chốt giao dịch cơ sở dữ liệu ACID.

### 4.1. FSM Nhiệm vụ Nhập Kho (INBOUND TASK)

```mermaid
stateDiagram-v2
    [*] --> CREATED: Web API POST /api/tasks (INBOUND)
    CREATED --> NAV_TO_IN: task_manager_node tiếp nhận
    
    NAV_TO_IN --> DOCK_IN: Đến trạm IN Staging (Nav2 Goal reached)
    NAV_TO_IN --> FAILED: Lỗi Nav2 / Timeout / Chướng ngại chặn cứng
    
    DOCK_IN --> PICK_IN: Docking ArUco trạm IN thành công (Tag 30)
    DOCK_IN --> RETRY_DOCK_IN: Lệch / Mất tag (Lặp <= 3 lần)
    RETRY_DOCK_IN --> DOCK_IN
    RETRY_DOCK_IN --> FAILED: Hết số lần thử

    PICK_IN --> VERIFY_BOX: SCARA gắp hộp tại sàn IN lên khay xe
    PICK_IN --> FAILED: Lỗi hút / Trượt hộp

    VERIFY_BOX --> NAV_TO_SLOT: Tag hộp trùng khớp expected_box_tag
    VERIFY_BOX --> REJECTED: Tag sai mã / Hộp không hợp lệ -> Hạ trả sàn IN
    REJECTED --> [*]: Giải phóng đặt chỗ, DB giữ nguyên

    NAV_TO_SLOT --> DOCK_SLOT: Đến trước ô kệ mục tiêu (Staging pose)
    NAV_TO_SLOT --> NAV_RECOVERY: Kẹt đường -> Tìm đường vòng
    NAV_RECOVERY --> NAV_TO_SLOT
    NAV_RECOVERY --> FAILED: Timeout di chuyển

    DOCK_SLOT --> PLACE_SLOT: Docking ô kệ thành công (e_y <= 5mm, e_yaw <= 2°)
    DOCK_SLOT --> FAILED: Mất dấu tag ô kệ

    PLACE_SLOT --> COMMIT_IN: SCARA đặt hộp vào ô, camera tay xác nhận nhả hộp
    PLACE_SLOT --> PLACE_ERROR: Kẹt tay / Không nhả được hộp
    PLACE_ERROR --> FAILED: Giữ nguyên hộp trên khay, báo lỗi

    COMMIT_IN --> COMPLETED: SQLite COMMIT (Cập nhật ô FULL, ghi log lịch sử)
    COMPLETED --> NAV_TO_HOME: Tự động điều hướng về trạm sạc HOME
    NAV_TO_HOME --> [*]: Robot sẵn sàng cho lệnh mới
    FAILED --> [*]: Hủy nhiệm vụ, rollback DB
```

### 4.2. FSM Nhiệm vụ Xuất Kho (OUTBOUND TASK)

```mermaid
stateDiagram-v2
    [*] --> CREATED: Web API POST /api/tasks (OUTBOUND)
    CREATED --> CHECK_STOCK: Kiểm tra tồn kho SQLite
    CHECK_STOCK --> REJECT_409: Hết hàng / Không có ô FULL hợp lệ
    REJECT_409 --> [*]: Trả HTTP 409 Conflict

    CHECK_STOCK --> NAV_TO_SLOT: Khóa ô RESERVED (BEGIN IMMEDIATE)
    NAV_TO_SLOT --> DOCK_SLOT: Đến điểm Staging trước ô kệ
    NAV_TO_SLOT --> FAILED: Lỗi điều hướng

    DOCK_SLOT --> PICK_SLOT: Docking ArUco kệ thành công
    DOCK_SLOT --> FAILED: Không nhận diện được tag ô

    PICK_SLOT --> NAV_TO_OUT: SCARA gắp hộp đặt an toàn lên khay xe
    PICK_SLOT --> FAILED: Rơi hộp / Mất hút chân không (Ô hoàn tác RESERVED)

    NAV_TO_OUT --> DOCK_OUT: Di chuyển về trạm OUT Staging
    NAV_TO_OUT --> FAILED: Kẹt đường không thể vượt qua

    DOCK_OUT --> PLACE_OUT: Docking trạm OUT thành công (Tag 31)
    DOCK_OUT --> FAILED: Lỗi docking trạm OUT

    PLACE_OUT --> COMMIT_OUT: SCARA đặt hộp lên băng chuyền / sàn trạm OUT
    PLACE_OUT --> FAILED: Lỗi đặt hộp

    COMMIT_OUT --> COMPLETED: SQLite COMMIT (Ô thành EMPTY, ghi log xuất)
    COMPLETED --> NAV_TO_HOME: Robot di chuyển về trạm HOME
    NAV_TO_HOME --> [*]
    FAILED --> [*]: Rollback trạng thái ô
```

---

## 5. SƠ ĐỒ BA TẦNG ĐỘ CHÍNH XÁC (3-TIER PRECISION ARCHITECTURE)

Một trong những đóng góp kiến trúc quan trọng nhất của hệ thống là việc phân rã bài toán định vị và tiếp cận thành ba tầng kế tiếp nhau, giải quyết trọn vẹn mâu thuẫn giữa tầm hoạt động rộng lớn của kho hàng và yêu cầu căn chỉnh cấp milimet khi thao tác cơ khí.

```mermaid
flowchart TD
    subgraph TIER1 ["TẦNG 1: ĐIỀU HƯỚNG TOÀN CỤC (GLOBAL NAVIGATION)"]
        T1_TECH["Công nghệ: 2D LiDAR + AMCL + Nav2 BT Navigator"]
        T1_RANGE["Tầm hoạt động: Toàn bộ mặt bằng kho (6.0m x 4.0m)"]
        T1_TARGET["Điểm đến: Staging Pose trước ô kệ (cách mặt kệ 0.55m)"]
        T1_ACC["Độ chính xác: Sai số dừng ± 30 - 80 mm, góc ± 3 - 5°"]
    end

    subgraph TIER2 ["TẦNG 2: ĐỊNH VỊ CĂN CHỈNH ĐỐI TƯỢNG (VISUAL SERVOING DOCKING)"]
        T2_TECH["Công nghệ: Camera sườn xe (dock_camera) + ArUco PnP + Closed-loop PID"]
        T2_RANGE["Tầm quan sát: 0.20m - 0.70m, góc lệch ban đầu lên tới ±15°"]
        T2_TARGET["Điểm đến: Work Pose chuẩn trước ô kệ (cách mặt kệ 0.32m)"]
        T2_ACC["Độ chính xác: Sai số tịnh tiến ≤ 5 mm (đạt 2.4 mm), góc ≤ 2.0° (đạt 0.81°)"]
    end

    subgraph TIER3 ["TẦNG 3: THAO TÁC CẤP MILIMET (CLOSED-LOOP SCARA EYE-IN-HAND)"]
        T3_TECH["Công nghệ: Camera bàn tay (hand_camera) + Động học giải tích SCARA TRR + Giác hút"]
        T3_RANGE["Tầm quan sát: Vùng làm việc cục bộ trong ô kệ (0.05m - 0.25m)"]
        T3_TARGET["Điểm đến: Tâm mặt trên hộp hàng (sai số đặt vào khay / ô)"]
        T3_ACC["Độ chính xác: Sai số lệch tâm gắp ≤ 1.5 mm, sai số vị trí đặt ≤ 2.2 mm"]
    end

    TIER1 -->|Bàn giao trạng thái tại Staging Pose| TIER2
    TIER2 -->|Bàn giao trạng thái tại Work Pose| TIER3

    style TIER1 fill:#e3f2fd,stroke:#1976d2,stroke-width:2px
    style TIER2 fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    style TIER3 fill:#e8f5e9,stroke:#388e3c,stroke-width:2px
```

### So sánh Đặc tính Kỹ thuật giữa Ba Tầng:

| Đặc tính | Tầng 1: Global Nav | Tầng 2: Docking Servoing | Tầng 3: SCARA Eye-in-Hand |
|---|---|---|---|
| **Cảm biến chính** | 2D LiDAR 360°, Wheel Encoders, IMU | Camera màu sườn xe (dock_camera) | Camera màu khâu chấp hành (hand_camera) |
| **Không gian điều khiển** | Khung toàn cục `map` | Khung cục bộ camera / `base_link` | Khung không gian khớp SCARA / `tool_link` |
| **Cơ cấu truyền động** | 4 bánh Mecanum (di chuyển toàn hướng) | 4 bánh Mecanum (vi sai tịnh tiến chậm $v_y, v_x, \omega_z$) | 1 trục trượt Z ($0.52\,\text{m}$) + 2 khớp xoay phẳng $q_1, q_2$ |
| **Sai số vị trí ($e_{xy}$)** | $60.5 \pm 19.8\,\text{mm}$ | **$2.43 \pm 0.94\,\text{mm}$** | **$1.38 \pm 0.42\,\text{mm}$** |
| **Sai số hướng ($e_{\theta}$)** | $3.58 \pm 1.04^\circ$ | **$0.81 \pm 0.35^\circ$** | Không áp dụng (tiếp xúc mặt phẳng hút đối xứng) |
| **Thời gian chu trình** | $10 - 25\,\text{s}$ | $3.5 - 6.0\,\text{s}$ | $8.0 - 12.0\,\text{s}$ |
| **Độ tin cậy** | $> 95\%$ | **$96.7\%$ (29/30 lần $\le 5\,\text{mm}$)** | **$96.7\%$ (58/60 chu kỳ thành công)** |
