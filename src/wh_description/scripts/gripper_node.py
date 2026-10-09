#!/usr/bin/env python3
"""gripper_node.py – Bộ điều khiển tay kẹp / đầu hút chân không cho AMR trong Gazebo.

Quản lý chuyển mạch tay kẹp qua /gripper/switch:
1. Chuyển tiếp tới plugin mô phỏng Gazebo (/gripper/switch_raw).
2. Tự động phát hiện và khóa cứng hộp mục tiêu vào gripper_grasp_link khi hút (khoảng cách <= 8 cm).
3. Duy trì vị trí hộp di chuyển mượt mà theo tay máy qua /gazebo/set_entity_state ở tần số 50 Hz.
4. Nhả hộp tự nhiên khi switch = False.
5. Xuất trạng thái /gripper/grasping chính xác cho Action Server Bài 6.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import LinkStates, ModelStates


class GripperNode(Node):
    """Quản lý tay kẹp và cơ chế bám dính vật lý trong Gazebo."""

    def __init__(self):
        super().__init__("gripper_node")
        self.cbg = ReentrantCallbackGroup()

        self.is_active = False
        self.is_grasping = False
        self.grasped_box_name = None

        # Lưu trữ vị trí các link và models từ Gazebo
        self.link_poses = {}
        self.model_poses = {}

        # 1. Kết nối plugin gazebo vacuum gripper
        self.cli_gazebo_switch = self.create_client(
            SetBool, "/gripper/switch_raw", callback_group=self.cbg
        )

        # 2. Service điều khiển vị trí thực thể trong Gazebo
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

        # 5. Public Status Publisher
        self.pub_grasping = self.create_publisher(
            Bool, "/gripper/grasping", 10
        )

        # 6. Timer bám dính và đồng bộ vị trí hộp (50 Hz)
        self.timer_sync = self.create_timer(0.02, self.sync_grasped_box, callback_group=self.cbg)
        self.timer_pub = self.create_timer(0.05, self.publish_status, callback_group=self.cbg)

        self.get_logger().info("GripperNode (Hard-Lock Enhanced) đã sẵn sàng.")

    def raw_grasping_cb(self, msg: Bool):
        if msg.data and not self.is_grasping:
            self.is_grasping = True

    def cb_link_states(self, msg: LinkStates):
        for name, pose in zip(msg.name, msg.pose):
            if "gripper_grasp_link" in name or "suction_link" in name or "tool_link" in name:
                self.link_poses[name] = pose

    def cb_model_states(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith("box_"):
                self.model_poses[name] = pose

    def get_gripper_pose(self):
        for key in self.link_poses:
            if "gripper_grasp_link" in key:
                return self.link_poses[key]
        for key in self.link_poses:
            if "suction_link" in key:
                return self.link_poses[key]
        return None

    def handle_switch(self, request, response):
        self.get_logger().info(f"Yêu cầu chuyển mạch tay kẹp: active={request.data}")
        self.is_active = request.data

        # Chuyển tiếp tới plugin thô
        if self.cli_gazebo_switch.service_is_ready():
            req_raw = SetBool.Request()
            req_raw.data = request.data
            self.cli_gazebo_switch.call_async(req_raw)

        if not self.is_active:
            # Nhả hộp
            if self.grasped_box_name:
                self.get_logger().info(f"Đã nhả hộp: {self.grasped_box_name}")
            self.grasped_box_name = None
            self.is_grasping = False
            response.success = True
            response.message = "Gripper OFF (đã nhả hộp)"
            return response

        # Khi bật kẹp, tìm hộp gần nhất trong bán kính 8 cm
        g_pose = self.get_gripper_pose()
        if g_pose is not None:
            gx, gy, gz = g_pose.position.x, g_pose.position.y, g_pose.position.z
            closest_box = None
            min_dist = 0.085  # 8.5 cm

            for b_name, b_pose in self.model_poses.items():
                bx, by, bz = b_pose.position.x, b_pose.position.y, b_pose.position.z
                dist = math.sqrt((gx - bx) ** 2 + (gy - by) ** 2 + (gz - bz) ** 2)
                if dist < min_dist:
                    min_dist = dist
                    closest_box = b_name

            if closest_box:
                self.grasped_box_name = closest_box
                self.is_grasping = True
                self.get_logger().info(
                    f"Kẹp dính thành công hộp: {closest_box} (khoảng cách = {min_dist*1000.0:.1f} mm)"
                )

        response.success = True
        response.message = f"Gripper ON (hộp={self.grasped_box_name})"
        return response

    def sync_grasped_box(self):
        """Khóa cứng tọa độ hộp theo gripper khi đang kẹp."""
        if not self.is_active or not self.grasped_box_name:
            return

        g_pose = self.get_gripper_pose()
        if g_pose is None or not self.cli_set_entity.service_is_ready():
            return

        req = SetEntityState.Request()
        req.state.name = self.grasped_box_name
        req.state.reference_frame = "world"
        # Đặt tâm hộp cách tâm gripper 1.5 cm về phía dưới
        req.state.pose.position.x = g_pose.position.x
        req.state.pose.position.y = g_pose.position.y
        req.state.pose.position.z = g_pose.position.z - 0.015
        req.state.pose.orientation = g_pose.orientation
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        self.cli_set_entity.call_async(req)
        self.is_grasping = True

    def publish_status(self):
        msg = Bool()
        msg.data = bool(self.is_grasping if self.is_active else False)
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
