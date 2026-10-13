#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rclpy.init()
    node = Node("verify_swap_and_grasp")

    model_states = None
    link_states = None
    is_grasping = False

    def cb_models(msg):
        nonlocal model_states
        model_states = msg

    def cb_links(msg):
        nonlocal link_states
        link_states = msg

    def cb_grasp(msg):
        nonlocal is_grasping
        is_grasping = msg.data

    node.create_subscription(ModelStates, "/gazebo/model_states", cb_models, 10)
    node.create_subscription(LinkStates, "/gazebo/link_states", cb_links, 10)
    node.create_subscription(Bool, "/gripper/grasping", cb_grasp, 10)

    pub_traj = node.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
    cli_switch = node.create_client(SetBool, "/gripper/switch")
    cli_set_entity = node.create_client(SetEntityState, "/gazebo/set_entity_state")

    print("Waiting for topics...")
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.1)
        if model_states and link_states and cli_switch.service_is_ready():
            break

    if not model_states or not link_states:
        print("ERROR: Timeout waiting for simulation topics!")
        return

    # 1. Kiểm tra vị trí của lidar_link và tray_link
    print("\n=== 1. KIỂM TRA ĐỔI CHỖ THÙNG HÀNG VÀ LIDAR ===")
    lidar_pos = None
    tray_pos = None
    for i, name in enumerate(link_states.name):
        if "lidar_link" in name:
            lidar_pos = link_states.pose[i].position
        elif "tray_link" in name:
            tray_pos = link_states.pose[i].position

    if lidar_pos and tray_pos:
        print(f"LiDAR position in world: ({lidar_pos.x:.3f}, {lidar_pos.y:.3f}, {lidar_pos.z:.3f})")
        print(f"Tray position in world:  ({tray_pos.x:.3f}, {tray_pos.y:.3f}, {tray_pos.z:.3f})")
    
    # 2. Đặt robot trước ô kệ 0: (x=0.75, y=0.28, yaw=pi/2)
    print("\n=== 2. ĐẶT ROBOT TRƯỚC Ô KỆ 0 ===")
    req = SetEntityState.Request()
    req.state.name = "warehouse_robot"
    req.state.reference_frame = "world"
    req.state.pose.position.x = 0.75
    req.state.pose.position.y = 0.28
    req.state.pose.position.z = 0.02
    req.state.pose.orientation.z = math.sin(math.pi / 4.0)
    req.state.pose.orientation.w = math.cos(math.pi / 4.0)
    fut = cli_set_entity.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    time.sleep(0.5)

    # 3. Di chuyển tay máy đến TOUCH_TIER_1 (q1=-0.8411, q2=1.6821, d=0.015)
    print("\n=== 3. HẠ TAY CHẠM HỘP (TOUCH_TIER_1) ===")
    traj = JointTrajectory()
    traj.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint"]
    p = JointTrajectoryPoint()
    p.positions = [0.015, -0.8411, 1.6821]
    p.time_from_start.sec = 2
    traj.points.append(p)
    pub_traj.publish(traj)
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Đo khoảng cách giữa gripper_grasp_link và box_20
    idx_box = model_states.name.index("box_20") if "box_20" in model_states.name else -1
    idx_grip = -1
    for i, name in enumerate(link_states.name):
        if "gripper_grasp_link" in name:
            idx_grip = i
            break

    if idx_box >= 0 and idx_grip >= 0:
        p_box = model_states.pose[idx_box].position
        p_grip = link_states.pose[idx_grip].position
        dist = math.sqrt((p_box.x - p_grip.x)**2 + (p_box.y - p_grip.y)**2 + (p_box.z - p_grip.z)**2)
        print(f"Tọa độ box_20: ({p_box.x:.4f}, {p_box.y:.4f}, {p_box.z:.4f})")
        print(f"Tọa độ đầu kẹp: ({p_grip.x:.4f}, {p_grip.y:.4f}, {p_grip.z:.4f})")
        print(f"Khoảng cách 3D: {dist*1000:.1f} mm (Ngưỡng vacuum: 120 mm)")

    # 4. Bật hút
    print("\n=== 4. BẬT HÚT CHÂN KHÔNG ===")
    req_sw = SetBool.Request()
    req_sw.data = True
    fut_sw = cli_switch.call_async(req_sw)
    rclpy.spin_until_future_complete(node, fut_sw, timeout_sec=2.0)
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"Trạng thái hút: is_grasping = {is_grasping}")

    # 5. Nhấc tay lên LIFT (d=0.040)
    print("\n=== 5. NHẤC HỘP LÊN (LIFT) ===")
    traj_lift = JointTrajectory()
    traj_lift.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint"]
    p_lift = JointTrajectoryPoint()
    p_lift.positions = [0.040, -0.8411, 1.6821]
    p_lift.time_from_start.sec = 2
    traj_lift.points.append(p_lift)
    pub_traj.publish(traj_lift)
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.1)

    p_box_after = model_states.pose[idx_box].position
    print(f"Tọa độ box_20 sau khi nâng: Z ban đầu = {p_box.z:.4f} m -> Z sau khi nâng = {p_box_after.z:.4f} m")
    if p_box_after.z > p_box.z + 0.015:
        print("THÀNH CÔNG RỰC RỠ: HỘP ĐÃ ĐƯỢC NHẤC LÊN KHỎI GIÁ KỆ!")
    else:
        print("CẢNH BÁO: Hộp chưa được nhấc lên!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
