#!/usr/bin/env python3
"""tag_detector_node.py – Nhận diện ArUco tag, ước lượng pose 3D và lọc nhiễu.

Chức năng:
- Đọc camera info (hoặc khởi tạo mặc định theo FOV)
- Phát hiện tag ArUco bằng ArucoCompat
- Giải PnP bằng cv2.SOLVEPNP_IPPE_SQUARE theo kích thước cạnh thật (marker_sizes.yaml)
- Lọc nhiễu qua TagFilter: vị trí trung vị (median), hướng trung bình quaternion (window_size=10)
- Biến đổi tọa độ sang base_link qua TF2 tại đúng timestamp ảnh
- Publish TagArray lên /vision/tags và ảnh debug lên /vision/debug_<camera>
"""

import math
import os
import time
from collections import deque
import cv2
import numpy as np
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge

from wh_interfaces.msg import TagArray, TagDetection
from wh_vision.aruco_compat import ArucoCompat


def rvec_to_quat(rvec):
    """Chuyển đổi rotation vector (Rodrigues) sang quaternion [x, y, z, w]."""
    rvec = np.asarray(rvec, dtype=np.float64).flatten()
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    axis = rvec / theta
    half = theta * 0.5
    s = math.sin(half)
    return [float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(math.cos(half))]


def average_quaternions(quats):
    """Tính trung bình của danh sách quaternion [x, y, z, w]."""
    if not quats:
        return [0.0, 0.0, 0.0, 1.0]
    q0 = np.array(quats[0], dtype=np.float64)
    q_sum = np.zeros(4, dtype=np.float64)
    for q in quats:
        q_arr = np.array(q, dtype=np.float64)
        if np.dot(q_arr, q0) < 0:
            q_arr = -q_arr
        q_sum += q_arr
    norm = np.linalg.norm(q_sum)
    if norm < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    q_avg = q_sum / norm
    return [float(q_avg[0]), float(q_avg[1]), float(q_avg[2]), float(q_avg[3])]


def estimate_pose(corners4x2, side, K, D):
    """Ước lượng 3D pose của marker bằng SOLVEPNP_IPPE_SQUARE và tính reprojection error."""
    s = side / 2.0
    # Thứ tự 4 góc ArUco: top-left, top-right, bottom-right, bottom-left
    obj = np.array([[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]], dtype=np.float32)
    corners_f = corners4x2.astype(np.float32).reshape(4, 2)
    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(obj, corners_f, K, D, flags=flags)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.mean(np.linalg.norm(proj.reshape(4, 2) - corners_f, axis=1)))
    return rvec, tvec, err


class TagFilter:
    """Bộ lọc trung vị vị trí và trung bình hướng theo cửa sổ trượt window_size."""

    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        # id -> deque([ (px, py, pz, qx, qy, qz, qw, timestamp) ])
        self.buffers = {}
        self.last_seen = {}

    def update(self, tag_id: int, pose: PoseStamped, current_time: float):
        if tag_id not in self.buffers:
            self.buffers[tag_id] = deque(maxlen=self.window_size)
        p = pose.pose.position
        q = pose.pose.orientation
        self.buffers[tag_id].append((p.x, p.y, p.z, q.x, q.y, q.z, q.w, current_time))
        self.last_seen[tag_id] = current_time

    def get_filtered_pose(self, tag_id: int, base_frame: str = "base_link") -> PoseStamped:
        if tag_id not in self.buffers or len(self.buffers[tag_id]) == 0:
            return None
        samples = list(self.buffers[tag_id])
        positions = [s[:3] for s in samples]
        quats = [s[3:7] for s in samples]

        # Vị trí = trung vị (median)
        med_pos = np.median(positions, axis=0)
        # Hướng = trung bình quaternion
        avg_q = average_quaternions(quats)

        pose = PoseStamped()
        pose.header.frame_id = base_frame
        pose.pose.position.x = float(med_pos[0])
        pose.pose.position.y = float(med_pos[1])
        pose.pose.position.z = float(med_pos[2])
        pose.pose.orientation.x = avg_q[0]
        pose.pose.orientation.y = avg_q[1]
        pose.pose.orientation.z = avg_q[2]
        pose.pose.orientation.w = avg_q[3]
        return pose

    def is_stable(self, tag_id: int, min_samples: int = 5, current_time: float = None) -> bool:
        if tag_id not in self.buffers:
            return False
        if len(self.buffers[tag_id]) < min(self.window_size, min_samples):
            return False
        if current_time is not None:
            if current_time - self.last_seen.get(tag_id, 0.0) > 0.5:
                return False
        return True

    def is_lost(self, tag_id: int, current_time: float, timeout_sec: float = 0.5) -> bool:
        if tag_id not in self.last_seen:
            return True
        return (current_time - self.last_seen[tag_id]) > timeout_sec

    def clear(self, tag_id: int = None):
        if tag_id is None:
            self.buffers.clear()
            self.last_seen.clear()
        elif tag_id in self.buffers:
            del self.buffers[tag_id]
            if tag_id in self.last_seen:
                del self.last_seen[tag_id]


class TagDetectorNode(Node):
    """ROS 2 Node nhận diện ArUco và ước lượng pose 3D."""

    def __init__(self):
        super().__init__(
            "tag_detector_node",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )

        # Khai báo tham số
        self.declare_parameter("camera_name", "dock")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("window_size", 10)
        self.declare_parameter("max_reproj_px", 1.0)
        self.declare_parameter("config_file", "")

        self.declare_parameter("optical_frame", "")
        self.camera_name = self.get_parameter("camera_name").value
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.window_size = self.get_parameter("window_size").value
        self.max_reproj_px = self.get_parameter("max_reproj_px").value
        config_file = self.get_parameter("config_file").value
        opt_frame_param = self.get_parameter("optical_frame").value

        # Tự động gán topic và frame theo camera_name nếu để trống
        prefix = self.camera_name if self.camera_name.endswith("_camera") else f"{self.camera_name}_camera"
        if not image_topic:
            image_topic = f"/{prefix}/image_raw"
        if not camera_info_topic:
            camera_info_topic = f"/{prefix}/camera_info"
        self.optical_frame = opt_frame_param if opt_frame_param else f"{prefix}_optical_frame"

        # Nạp bảng kích thước marker
        self.marker_sizes = self.load_marker_sizes(config_file)
        self.filter = TagFilter(window_size=self.window_size)
        self.aruco = ArucoCompat("DICT_4X4_50")
        self.bridge = CvBridge()

        # Ma trận camera mặc định từ FOV 1.2 rad (640x480) đề phòng chưa có camera_info
        w, h = 640.0, 480.0
        fx = (w / 2.0) / math.tan(1.2 / 2.0)
        fy = fx
        self.K = np.array([[fx, 0.0, w / 2.0], [0.0, fy, h / 2.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.zeros(5, dtype=np.float64)
        self.has_camera_info = False

        # TF2 Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.tag_pub = self.create_publisher(TagArray, "/vision/tags", 10)
        self.debug_pub = self.create_publisher(Image, f"/vision/debug_{self.camera_name}", 10)

        # Subscriptions
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.info_cb, 10)
        self.img_sub = self.create_subscription(Image, image_topic, self.image_cb, 10)

        self.get_logger().info(
            f"TagDetectorNode ({self.camera_name}) đã khởi động. "
            f"Image: {image_topic}, Optical frame: {self.optical_frame}, "
            f"Window: {self.window_size}, Max reproj: {self.max_reproj_px} px"
        )

    def load_marker_sizes(self, config_file: str) -> dict:
        """Nạp bảng ánh xạ ID -> marker side (m)."""
        mapping = {}
        default_size = 0.050

        candidates = [
            config_file,
            os.path.join(os.path.dirname(__file__), "..", "config", "marker_sizes.yaml"),
            "/home/lehuythien/warehouse_ws/src/wh_vision/config/marker_sizes.yaml",
            "/home/lehuythien/warehouse_ws/install/wh_vision/share/wh_vision/config/marker_sizes.yaml",
        ]

        found = None
        for c in candidates:
            if c and os.path.exists(c):
                found = c
                break

        if found:
            try:
                with open(found, "r") as f:
                    data = yaml.safe_load(f)
                sizes_cfg = data.get("marker_sizes", {})
                default_size = float(data.get("default_size", 0.050))
                for group, val in sizes_cfg.items():
                    s = float(val.get("size", default_size))
                    for tag_id in val.get("ids", []):
                        mapping[int(tag_id)] = s
                self.get_logger().info(f"Đã nạp {len(mapping)} tag sizes từ {found}.")
            except Exception as e:
                self.get_logger().warn(f"Không thể đọc {found}: {e}. Dùng mặc định.")
        else:
            self.get_logger().warn("Không tìm thấy marker_sizes.yaml, dùng ánh xạ mặc định.")
            for i in range(12): mapping[i] = 0.050
            mapping[30] = 0.050
            mapping[31] = 0.050
            for i in range(20, 40):
                if i not in (30, 31):
                    mapping[i] = 0.036

        self.default_marker_size = default_size
        return mapping

    def get_marker_size(self, tag_id: int) -> float:
        return self.marker_sizes.get(tag_id, self.default_marker_size)

    def info_cb(self, msg: CameraInfo):
        if not self.has_camera_info:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            if len(msg.d) >= 5:
                self.D = np.array(msg.d[:5], dtype=np.float64)
            self.has_camera_info = True
            self.get_logger().info(f"Đã cập nhật CameraInfo ({self.camera_name}). fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}")

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Lỗi cv_bridge: {e}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids = self.aruco.detect(gray)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        tag_array_msg = TagArray()
        tag_array_msg.header = msg.header
        tag_array_msg.camera = self.camera_name

        debug_img = cv_img.copy()

        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                tag_id = int(marker_id)
                side = self.get_marker_size(tag_id)
                c4x2 = corners[i].reshape(4, 2)

                res = estimate_pose(c4x2, side, self.K, self.D)
                if res is None:
                    continue
                rvec, tvec, reproj_err = res

                # Loại bỏ ngoại lai nếu reprojection error vượt ngưỡng
                if reproj_err > self.max_reproj_px:
                    continue

                # Tạo PoseStamped trong camera optical frame
                q_cam = rvec_to_quat(rvec)
                pose_cam = PoseStamped()
                pose_cam.header = msg.header
                pose_cam.header.frame_id = self.optical_frame
                pose_cam.pose.position.x = float(tvec[0])
                pose_cam.pose.position.y = float(tvec[1])
                pose_cam.pose.position.z = float(tvec[2])
                pose_cam.pose.orientation = Quaternion(
                    x=q_cam[0], y=q_cam[1], z=q_cam[2], w=q_cam[3]
                )

                # Biến đổi sang base_link qua TF2 tại timestamp của ảnh (với fallback Time(0))
                pose_base = None
                try:
                    tf_trans = self.tf_buffer.lookup_transform(
                        "base_link",
                        self.optical_frame,
                        pose_cam.header.stamp,
                        Duration(seconds=0.1)
                    )
                    pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, tf_trans)
                except Exception:
                    try:
                        tf_trans = self.tf_buffer.lookup_transform(
                            "base_link",
                            self.optical_frame,
                            Time(),
                            Duration(seconds=0.1)
                        )
                        pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, tf_trans)
                    except Exception as tf_ex:
                        self.get_logger().debug(f"TF lookup thất bại: {tf_ex}")
                        continue

                # Cập nhật bộ lọc TagFilter (lọc trên khung base_link)
                self.filter.update(tag_id, pose_base, now_sec)
                filtered_base = self.filter.get_filtered_pose(tag_id, base_frame="base_link")
                if filtered_base is not None:
                    filtered_base.header.stamp = msg.header.stamp
                else:
                    filtered_base = pose_base

                # Đóng gói TagDetection
                detection = TagDetection()
                detection.id = tag_id
                detection.pose_camera = pose_cam
                detection.pose_base = filtered_base
                detection.reproj_error_px = float(reproj_err)
                tag_array_msg.tags.append(detection)

                # Vẽ debug trên ảnh
                cv2.polylines(
                    debug_img,
                    [c4x2.astype(np.int32)],
                    isClosed=True,
                    color=(0, 255, 0),
                    thickness=2
                )
                try:
                    cv2.drawFrameAxes(debug_img, self.K, self.D, rvec, tvec, side * 0.75, 2)
                except Exception:
                    pass

                dist_m = float(np.linalg.norm(tvec))
                center_pt = tuple(np.mean(c4x2, axis=0).astype(int))
                info_text = f"ID:{tag_id} d:{dist_m:.2f}m err:{reproj_err:.2f}px"
                cv2.putText(
                    debug_img,
                    info_text,
                    (center_pt[0] - 40, center_pt[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA
                )

        # Publish TagArray
        self.tag_pub.publish(tag_array_msg)

        # Publish ảnh debug
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
