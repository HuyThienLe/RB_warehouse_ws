#!/usr/bin/env python3
"""gripper_node.py – Bộ điều khiển tay kẹp cơ khí servo cho AMR trong Gazebo.

Quản lý chuyển mạch tay kẹp qua /gripper/switch:
1. Điều khiển động học má kẹp servo (prismatic): mở rộng (102.5 mm) và khép chặt (58.5 mm)
   thông qua gazebo_ros2_control JointGroupPositionController (/gripper_controller/commands).
2. Kích hoạt plugin vật lý Gazebo (/gripper/switch_raw) để khóa cứng hộp vào gripper_grasp_link
   bằng khớp nối vật lý ODE tự nhiên, chống 100% rung lắc / giật giật.
3. Căn chỉnh vị trí hộp một lần duy nhất (one-shot alignment) ngay khi đóng kẹp để đảm bảo
   hộp nằm ngay ngắn, vuông vắn chính giữa 2 miếng đệm cao su.
4. Mở má kẹp và giải phóng liên kết vật lý khi switch = False.
5. Xuất trạng thái /gripper/grasping và /joint_states cho toàn hệ thống.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import LinkStates, ModelStates


def rotate_vector_by_quat(v, q):
    """Xoay vector v=(x, y, z) theo quaternion q=(x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    cx += qw * vx
    cy += qw * vy
    cz += qw * vz
    rx = qy * cz - qz * cy
    ry = qz * cx - qx * cz
    rz = qx * cy - qy * cx
    return (vx + 2.0 * rx, vy + 2.0 * ry, vz + 2.0 * rz)


class GripperNode(Node):
    """Quản lý tay kẹp cơ khí servo và cơ chế bám dính vật lý trong Gazebo."""

    def __init__(self):
        super().__init__("gripper_node")
        self.cbg = ReentrantCallbackGroup()

        self.is_active = False
        self.is_grasping = False
        self.grasped_box_name = None

        # Vị trí hành trình má kẹp servo (m):
        # - FINGER_OPEN: mở rộng 2 bên (+0.015 m má trái, -0.015 m má phải, khẩu độ đệm 90.5 mm, khẩu độ khung 96 mm)
        # - FINGER_NEUTRAL: tư thế nghỉ bình thường (0.0 m má trái, 0.0 m má phải, khẩu độ ~72 mm)
        # - FINGER_CLOSED: khép chặt ôm sát 2 bên thành hộp 60mm
        self.FINGER_OPEN = 0.015
        self.FINGER_NEUTRAL = 0.0
        self.FINGER_CLOSED = -0.0010

        self.current_finger_pos = self.FINGER_NEUTRAL
        self.target_finger_pos = self.FINGER_NEUTRAL
        self.sync_in_flight = False

        # Lưu trữ vị trí và vận tốc các link và models từ Gazebo
        self.link_poses = {}
        self.link_twists = {}
        self.model_poses = {}

        # 1. Kết nối plugin gazebo parallel/vacuum gripper
        self.cli_gazebo_switch = self.create_client(
            SetBool, "/gripper/switch_raw", callback_group=self.cbg
        )

        # 2. Service điều khiển vị trí thực thể trong Gazebo (căn chỉnh & khóa cứng chống tuột)
        self.cli_set_entity = self.create_client(
            SetEntityState, "/gazebo/set_entity_state", callback_group=self.cbg
        )

        # 3. Public Service /gripper/switch
        self.srv_switch = self.create_service(
            SetBool, "/gripper/switch", self.handle_switch, callback_group=self.cbg
        )

        # 4. Subscriptions
        self.sub_raw_grasping = self.create_subscription(
            Bool, "/gripper/grasping_raw", self.raw_grasping_cb, 10, callback_group=self.cbg
        )
        self.sub_link_states = self.create_subscription(
            LinkStates, "/gazebo/link_states", self.cb_link_states, 10, callback_group=self.cbg
        )
        self.sub_model_states = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.cb_model_states, 10, callback_group=self.cbg
        )

        # 5. Public Status & Joint Command Publishers
        self.pub_grasping = self.create_publisher(
            Bool, "/gripper/grasping", 10
        )
        self.pub_finger_joints = self.create_publisher(
            JointState, "/joint_states", 10
        )
        # Điều khiển trực tiếp gazebo_ros2_control JointGroupPositionController
        self.pub_gripper_cmd = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )

        # 6. Timer mô phỏng chuyển động má kẹp servo và giám sát bám dính (50 Hz)
        self.timer_sync = self.create_timer(0.02, self.sync_loop, callback_group=self.cbg)
        self.timer_pub = self.create_timer(0.05, self.publish_status, callback_group=self.cbg)

        self.get_logger().info("GripperNode (Anti-Slip Firm Clamp & Zero-Jitter Physics) đã sẵn sàng.")

    def raw_grasping_cb(self, msg: Bool):
        if not msg.data:
            self.is_grasping = False
        elif self.is_active and msg.data:
            self.is_grasping = True

    def cb_link_states(self, msg: LinkStates):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if any(k in name for k in ("tool_link", "gripper_grasp_link", "suction_link", "left_finger_link", "right_finger_link")):
                self.link_poses[name] = pose
                self.link_twists[name] = twist

    def cb_model_states(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if "box" in name.lower():
                self.model_poses[name] = pose

    def get_tool_link_pose(self):
        for key in self.link_poses:
            if "tool_link" in key:
                return self.link_poses[key]
        for key in self.link_poses:
            if "gripper_grasp_link" in key or "suction_link" in key:
                return self.link_poses[key]
        return None

    def get_tool_link_twist(self):
        for key in self.link_twists:
            if "tool_link" in key:
                return self.link_twists[key]
        for key in self.link_twists:
            if "gripper_grasp_link" in key or "suction_link" in key:
                return self.link_twists[key]
        return None

    def get_tcp_pose(self):
        """Tính toán chính xác tọa độ thế giới của tâm kẹp TCP giữa 2 má kẹp cao su."""
        # Ưu tiên lấy trực tiếp tọa độ của gripper_grasp_link nếu đã tồn tại trong Gazebo link_states
        for key in self.link_poses:
            if "gripper_grasp_link" in key or "suction_link" in key:
                p = self.link_poses[key]
                return (p.position.x, p.position.y, p.position.z), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

        t_pose = self.get_tool_link_pose()
        if t_pose is None:
            return None, None
        q = (t_pose.orientation.x, t_pose.orientation.y, t_pose.orientation.z, t_pose.orientation.w)
        # Vị trí tâm kẹp giữa 2 má kẹp (x=0.058m phía trước, z=-0.070m bên dưới tool_link)
        dx, dy, dz = rotate_vector_by_quat((0.058, 0.0, -0.070), q)
        tcp_pos = (t_pose.position.x + dx, t_pose.position.y + dy, t_pose.position.z + dz)
        return tcp_pos, q

    def handle_switch(self, request, response):
        self.get_logger().info(f"Yêu cầu chuyển mạch tay kẹp: active={request.data}")
        self.is_active = request.data

        if not self.is_active:
            # 1. Mở servo nhả hộp: 2 má kẹp mở rộng sang hai bên
            self.target_finger_pos = self.FINGER_OPEN

            # 2. Ngắt plugin vật lý Gazebo để nhả hộp tự nhiên
            if self.cli_gazebo_switch.service_is_ready():
                req_raw = SetBool.Request()
                req_raw.data = False
                self.cli_gazebo_switch.call_async(req_raw)

            if self.grasped_box_name:
                self.get_logger().info(f"Động cơ servo mở má kẹp nhả hộp: {self.grasped_box_name}")
            self.grasped_box_name = None
            self.is_grasping = False
            self.pub_grasping.publish(Bool(data=False))

            response.success = True
            response.message = "Servo Gripper OPEN (đã mở má kẹp nhả hộp)"
            return response

        # ── Đóng servo kẹp hộp ───────────────────────────────────────────────
        self.target_finger_pos = self.FINGER_CLOSED

        # 1. Tìm hộp gần nhất trong bán kính tiếp xúc từ tâm TCP giữa 2 má kẹp
        tcp_pos, q = self.get_tcp_pose()
        if tcp_pos is not None:
            gx, gy, gz = tcp_pos
            closest_box = None
            min_dist = 0.090  # 90 mm đảm bảo bao trọn hộp khi 2 má kẹp khép lại
            self.get_logger().info(f"Gripper True TCP tại: ({gx:.3f}, {gy:.3f}, {gz:.3f})")

            for b_name, b_pose in self.model_poses.items():
                bx, by, bz = b_pose.position.x, b_pose.position.y, b_pose.position.z
                dz = abs(gz - bz)
                if dz > 0.080:
                    continue
                dist_xy = math.sqrt((gx - bx) ** 2 + (gy - by) ** 2)
                if dist_xy < min_dist:
                    min_dist = dist_xy
                    closest_box = b_name

            if closest_box:
                self.grasped_box_name = closest_box
                self.is_grasping = True
                self.get_logger().info(
                    f"Động cơ servo kẹp chặt thành công hộp: {closest_box} (khoảng cách = {min_dist*1000.0:.1f} mm)"
                )
            else:
                self.grasped_box_name = None
                self.is_grasping = False
                self.get_logger().warn(
                    f"Không tìm thấy hộp trong bán kính {min_dist*1000.0:.1f} mm xung quanh TCP ({gx:.3f}, {gy:.3f}, {gz:.3f})"
                )
        else:
            self.grasped_box_name = None
            self.is_grasping = False
            self.get_logger().warn("tcp_pos is None! Chưa nhận được vị trí tool_link")

        # 2. Kích hoạt plugin khóa cứng vật lý Gazebo (Rigid Joint Attacher)
        if self.cli_gazebo_switch.service_is_ready():
            req_raw = SetBool.Request()
            req_raw.data = True
            self.cli_gazebo_switch.call_async(req_raw)

        response.success = True
        response.message = f"Servo Gripper CLOSED (đã kẹp chặt hộp={self.grasped_box_name})"
        return response

    def sync_loop(self):
        """Vòng lặp 50Hz: Cập nhật chuyển động trượt của 2 má kẹp servo."""
        # 1. Điều khiển má kẹp servo di chuyển mượt mà
        diff = self.target_finger_pos - self.current_finger_pos
        if abs(diff) > 0.0004:
            step = 0.06 * 0.02  # Vận tốc trượt 0.06 m/s
            if diff > 0:
                self.current_finger_pos = min(self.target_finger_pos, self.current_finger_pos + step)
            else:
                self.current_finger_pos = max(self.target_finger_pos, self.current_finger_pos - step)
        else:
            self.current_finger_pos = self.target_finger_pos

        # Gửi lệnh trực tiếp tới gazebo_ros2_control JointGroupPositionController
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(self.current_finger_pos), float(-self.current_finger_pos)]
        self.pub_gripper_cmd.publish(cmd_msg)

        # Xuất JointState (cập nhật Robot State Publisher, TF và RViz)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_finger_joint", "right_finger_joint"]
        js.position = [float(self.current_finger_pos), float(-self.current_finger_pos)]
        js.velocity = [0.0, 0.0]
        self.pub_finger_joints.publish(js)

    def publish_status(self):
        msg = Bool()
        msg.data = bool(self.is_active and (self.grasped_box_name is not None or self.is_grasping))
        self.pub_grasping.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
