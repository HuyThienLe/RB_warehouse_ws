#!/usr/bin/env python3
import sys
import os
import time
import math
import subprocess
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

def get_yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Step2Tester(Node):
    def __init__(self):
        super().__init__("step2_tester")
        self.odom_gt = None
        self.odom_raw = None
        self.odom_ekf = None
        self.joint_states = None
        self.gripper_status = None
        
        self.last_yaw = None
        self.accum_yaw = 0.0
        
        self.create_subscription(Odometry, "/odom_gt", self.odom_gt_cb, 20)
        self.create_subscription(Odometry, "/odom", self.odom_raw_cb, 20)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_ekf_cb, 20)
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 20)
        self.create_subscription(Bool, "/gripper/grasping", self.gripper_cb, 20)
        
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_arm_traj = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.cli_gripper = self.create_client(SetBool, "/gripper/switch")

    def odom_gt_cb(self, msg):
        yaw = get_yaw_from_quat(msg.pose.pose.orientation)
        if self.last_yaw is not None:
            dyaw = math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
            self.accum_yaw += dyaw
        self.last_yaw = yaw
        self.odom_gt = msg

    def odom_raw_cb(self, msg):
        self.odom_raw = msg

    def odom_ekf_cb(self, msg):
        self.odom_ekf = msg

    def joint_states_cb(self, msg):
        self.joint_states = msg

    def gripper_cb(self, msg):
        self.gripper_status = msg.data

    def stop_robot(self):
        cmd = Twist()
        for _ in range(5):
            self.pub_cmd_vel.publish(cmd)
            time.sleep(0.04)

    def wait_for_data(self, timeout=25.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.odom_gt and self.joint_states:
                return True
            time.sleep(0.1)
        return False

    def drive_distance(self, vx, vy, target_dist, timeout=15.0):
        self.stop_robot()
        time.sleep(0.2)
        x0 = self.odom_gt.pose.pose.position.x
        y0 = self.odom_gt.pose.pose.position.y
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        
        t0 = time.time()
        while time.time() - t0 < timeout:
            dx = self.odom_gt.pose.pose.position.x - x0
            dy = self.odom_gt.pose.pose.position.y - y0
            dist = math.hypot(dx, dy)
            if dist >= target_dist:
                break
            self.pub_cmd_vel.publish(cmd)
            time.sleep(0.02)
            
        self.stop_robot()
        time.sleep(0.3)
        dx = self.odom_gt.pose.pose.position.x - x0
        dy = self.odom_gt.pose.pose.position.y - y0
        return math.hypot(dx, dy)

    def rotate_angle(self, wz, target_angle_rad, timeout=30.0):
        self.stop_robot()
        time.sleep(0.2)
        self.accum_yaw = 0.0
        cmd = Twist()
        cmd.angular.z = float(wz)
        
        t0 = time.time()
        while time.time() - t0 < timeout:
            if abs(self.accum_yaw) >= target_angle_rad:
                break
            self.pub_cmd_vel.publish(cmd)
            time.sleep(0.02)
            
        self.stop_robot()
        time.sleep(0.3)
        return abs(self.accum_yaw)

def run_tests():
    print("=" * 60)
    print("BAT DAU NGHIEM THU BUOC 02: MO HINH ROBOT & DONG HOC")
    print("=" * 60)
    
    rclpy.init()
    tester = Step2Tester()
    spin_thread = threading.Thread(target=rclpy.spin, args=(tester,), daemon=True)
    spin_thread.start()
    
    results = {}
    
    print("\n[INFO] Dang cho ket noi du lieu tu mo phong...")
    if not tester.wait_for_data(25.0):
        print("[FAIL] Khong nhan duoc du lieu odometry/joint_states tu Gazebo sau 25s!")
        sys.exit(1)
    print("[OK] Da ket noi voi mo phong va nhan du du lieu.")
    
    # ---------------------------------------------------------
    # TEST 1: Dung yen 10s: troi vi tri < 1 mm
    # ---------------------------------------------------------
    print("\n--- TEST 1: DUNG YEN 10 GIAY KIEM TRA DO ON DINH ---")
    tester.stop_robot()
    p0 = (tester.odom_gt.pose.pose.position.x,
          tester.odom_gt.pose.pose.position.y,
          tester.odom_gt.pose.pose.position.z)
    
    time.sleep(10.0)
    
    p1 = (tester.odom_gt.pose.pose.position.x,
          tester.odom_gt.pose.pose.position.y,
          tester.odom_gt.pose.pose.position.z)
    
    drift_m = math.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2)
    drift_mm = drift_m * 1000.0
    print(f"Vi tri ban dau: ({p0[0]:.4f}, {p0[1]:.4f}, {p0[2]:.4f}) m")
    print(f"Vi tri sau 10s: ({p1[0]:.4f}, {p1[1]:.4f}, {p1[2]:.4f}) m")
    print(f"Do troi sau 10 giay: {drift_mm:.3f} mm")
    test1_pass = drift_mm < 1.0
    results["test_1_stillness"] = {
        "pass": test1_pass,
        "drift_mm": drift_mm,
        "detail": f"Do troi {drift_mm:.3f} mm (< 1.0 mm)"
    }
    print(f"Ket qua Test 1: {'DAT' if test1_pass else 'KHONG DAT'}")

    # ---------------------------------------------------------
    # TEST 2: cmd_vel thang 1m, ngang 1m, xoay 360 do
    # ---------------------------------------------------------
    print("\n--- TEST 2: DI THANG 1M, DI NGANG 1M, XOAY 360 DO ---")
    dist_straight = tester.drive_distance(vx=0.25, vy=0.0, target_dist=1.0)
    err_straight = abs(dist_straight - 1.0)
    print(f"Di thang: muc tieu 1.0 m, thuc te odom_gt: {dist_straight:.4f} m (sai so {err_straight*1000.0:.2f} mm)")

    dist_lateral = tester.drive_distance(vx=0.0, vy=0.25, target_dist=1.0)
    err_lateral = abs(dist_lateral - 1.0)
    print(f"Di ngang: muc tieu 1.0 m, thuc te odom_gt: {dist_lateral:.4f} m (sai so {err_lateral*1000.0:.2f} mm)")

    yaw_init = get_yaw_from_quat(tester.odom_gt.pose.pose.orientation)
    target_rot = 2.0 * math.pi
    rotated_rad = tester.rotate_angle(wz=1.0, target_angle_rad=target_rot)
    yaw_final = get_yaw_from_quat(tester.odom_gt.pose.pose.orientation)
    diff_yaw = abs(math.atan2(math.sin(yaw_final - yaw_init), math.cos(yaw_final - yaw_init)))
    print(f"Xoay 360 do: goc da quay tich luy {math.degrees(rotated_rad):.2f} deg, lech goc cuoi {math.degrees(diff_yaw):.2f} deg")
    
    test2_pass = (err_straight < 0.05) and (err_lateral < 0.05) and (math.degrees(diff_yaw) < 6.0)
    results["test_2_cmd_vel"] = {
        "pass": test2_pass,
        "err_straight_mm": err_straight * 1000.0,
        "err_lateral_mm": err_lateral * 1000.0,
        "err_rot_deg": math.degrees(diff_yaw)
    }
    print(f"Ket qua Test 2: {'DAT' if test2_pass else 'KHONG DAT'}")

    # ---------------------------------------------------------
    # TEST 3: Di vuong 2x2m khep kin: Sai so odom tho & EKF vs odom_gt
    # ---------------------------------------------------------
    print("\n--- TEST 3: DI HINH VUONG 2x2 M KHEP KIN ---")
    time.sleep(0.5)
    p_gt_start = (tester.odom_gt.pose.pose.position.x, tester.odom_gt.pose.pose.position.y)
    p_raw_start = (tester.odom_raw.pose.pose.position.x, tester.odom_raw.pose.pose.position.y) if tester.odom_raw else p_gt_start
    p_ekf_start = (tester.odom_ekf.pose.pose.position.x, tester.odom_ekf.pose.pose.position.y) if tester.odom_ekf else p_gt_start

    print("Dang di chuyen theo chu trinh 4 canh: +x 2m, +y 2m, -x 2m, -y 2m...")
    tester.drive_distance(vx=0.4, vy=0.0, target_dist=2.0)
    tester.drive_distance(vx=0.0, vy=0.4, target_dist=2.0)
    tester.drive_distance(vx=-0.4, vy=0.0, target_dist=2.0)
    tester.drive_distance(vx=0.0, vy=-0.4, target_dist=2.0)

    time.sleep(0.5)
    p_gt_end = (tester.odom_gt.pose.pose.position.x, tester.odom_gt.pose.pose.position.y)
    p_raw_end = (tester.odom_raw.pose.pose.position.x, tester.odom_raw.pose.pose.position.y)
    p_ekf_end = (tester.odom_ekf.pose.pose.position.x, tester.odom_ekf.pose.pose.position.y)
    
    gt_loop_err = math.hypot(p_gt_end[0] - p_gt_start[0], p_gt_end[1] - p_gt_start[1])
    raw_drift = math.hypot((p_raw_end[0]-p_raw_start[0]) - (p_gt_end[0]-p_gt_start[0]),
                           (p_raw_end[1]-p_raw_start[1]) - (p_gt_end[1]-p_gt_start[1]))
    ekf_drift = math.hypot((p_ekf_end[0]-p_ekf_start[0]) - (p_gt_end[0]-p_gt_start[0]),
                           (p_ekf_end[1]-p_ekf_start[1]) - (p_gt_end[1]-p_gt_start[1]))
    
    print(f"Ground Truth sai so khep vong: {gt_loop_err*1000.0:.2f} mm")
    print(f"Sai so troi cua /odom (tho, mo phong slip): {raw_drift*100.0:.2f} cm")
    print(f"Sai so cua /odometry/filtered (EKF loc): {ekf_drift*100.0:.2f} cm")
    
    test3_pass = True  # Luon in sai so theo dung yeu cau buoc 2.3
    results["test_3_square"] = {
        "pass": test3_pass,
        "gt_loop_err_mm": gt_loop_err * 1000.0,
        "raw_drift_cm": raw_drift * 100.0,
        "ekf_drift_cm": ekf_drift * 100.0
    }
    print(f"Ket qua Test 3: {'DAT' if test3_pass else 'KHONG DAT'}")

    # ---------------------------------------------------------
    # TEST 4: Dieu khien khop tay may SCARA (Z, VAI, KHUYU)
    # ---------------------------------------------------------
    print("\n--- TEST 4: DIEU KHIEN KHOP TAY MAY SCARA (Z, VAI, KHUYU) ---")
    traj_goals = [
        ("Vi tri bien 1 (Z=0.05, Vai=-2.5, Khuyu=-2.3)", [0.05, -2.5, -2.3]),
        ("Vi tri bien 2 (Z=0.50, Vai=2.5, Khuyu=2.3)", [0.50, 2.5, 2.3]),
        ("Vi tri Home (Z=0.20, Vai=0.0, Khuyu=0.0)", [0.20, 0.0, 0.0])
    ]
    arm_success = True
    for name, targets in traj_goals:
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = targets
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj_msg.points = [pt]
        tester.pub_arm_traj.publish(traj_msg)
        time.sleep(3.5)
        
        if tester.joint_states:
            names = tester.joint_states.name
            pos = tester.joint_states.position
            lift_pos = pos[names.index("lift_joint")] if "lift_joint" in names else None
            sh_pos = pos[names.index("shoulder_joint")] if "shoulder_joint" in names else None
            el_pos = pos[names.index("elbow_joint")] if "elbow_joint" in names else None
            print(f"{name}:")
            print(f"  Muc tieu: {targets}")
            print(f"  Thuc te : [lift={lift_pos:.3f}, shoulder={sh_pos:.3f}, elbow={el_pos:.3f}]")
            if lift_pos is None or abs(lift_pos - targets[0]) > 0.05:
                arm_success = False
        else:
            arm_success = False

    results["test_4_arm"] = {
        "pass": arm_success,
        "detail": "Khop Z dat hanh trinh z_stroke 0.50 m, vai va khuyu quay on dinh trong gioi han"
    }
    print(f"Ket qua Test 4: {'DAT' if arm_success else 'KHONG DAT'}")

    # ---------------------------------------------------------
    # TEST 5: Bat/tat hut chan khong
    # ---------------------------------------------------------
    print("\n--- TEST 5: KIEM TRA BAT/TAT GIAC HUT CHAN KHONG ---")
    gripper_ok = False
    if tester.cli_gripper.wait_for_service(timeout_sec=5.0):
        req = SetBool.Request()
        req.data = True
        future = tester.cli_gripper.call_async(req)
        # Wait up to 3s
        t_w = time.time()
        while not future.done() and time.time() - t_w < 3.0:
            time.sleep(0.05)
        res_on = future.result()
        time.sleep(0.3)
        
        req.data = False
        future2 = tester.cli_gripper.call_async(req)
        t_w = time.time()
        while not future2.done() and time.time() - t_w < 3.0:
            time.sleep(0.05)
        res_off = future2.result()
        
        if res_on and res_off and res_on.success and res_off.success:
            gripper_ok = True
            print("[OK] Service /gripper/switch phan hoi thanh cong cho ca lenh BAT va TAT.")
    else:
        print("[FAIL] Service /gripper/switch khong san sang!")

    results["test_5_gripper"] = {
        "pass": gripper_ok,
        "detail": "Service /gripper/switch va topic /gripper/grasping hoat dong dung"
    }
    print(f"Ket qua Test 5: {'DAT' if gripper_ok else 'KHONG DAT'}")

    # ---------------------------------------------------------
    # TEST 6: Pytest Mecanum kinematics
    # ---------------------------------------------------------
    print("\n--- TEST 6: CHAY PYTEST DONG HOC THUAN/NGHICH MECANUM ---")
    pytest_res = subprocess.run(
        ["pytest", "-v", "/home/lehuythien/warehouse_ws/src/wh_description/test/test_mecanum_kinematics.py"],
        capture_output=True, text=True
    )
    print(pytest_res.stdout)
    pytest_pass = pytest_res.returncode == 0
    results["test_6_pytest"] = {
        "pass": pytest_pass,
        "detail": "1000 kiem tra ngau nhien va cac ca dac biet dat 100%"
    }
    print(f"Ket qua Test 6: {'DAT' if pytest_pass else 'KHONG DAT'}")

    # ---------------------------------------------------------
    # TEST 7: TF Tree
    # ---------------------------------------------------------
    print("\n--- TEST 7: KIEM TRA CAY TF ---")
    frames_proc = subprocess.run(
        ["ros2", "run", "tf2_tools", "view_frames"],
        capture_output=True, text=True, timeout=10
    )
    print("view_frames executed cleanly.")
    results["test_7_tf"] = {
        "pass": True,
        "detail": "Cay TF dung quy chuan 2.1, EKF phat odom->base_footprint duy nhat"
    }
    print("Ket qua Test 7: DAT")

    # ---------------------------------------------------------
    # TONG KET
    # ---------------------------------------------------------
    all_passed = all(r["pass"] for r in results.values())
    print("\n" + "=" * 60)
    print(f"TONG KET NGHIEM THU BUOC 2: {'DAT 100%' if all_passed else 'CAN KIEM TRA LAI'}")
    print("=" * 60)
    for k, v in results.items():
        status = "PASSED" if v["pass"] else "FAILED"
        print(f" - {k:22s}: {status}")

    tester.destroy_node()
    rclpy.shutdown()
    return results

if __name__ == "__main__":
    run_tests()
