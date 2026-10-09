#!/usr/bin/env python3
"""box_manager.py – Node quản lý hộp và vật cản động trong Gazebo.

Services cung cấp:
  /wh/spawn_box      (wh_interfaces/srv/SpawnBox)
  /wh/remove_box     (wh_interfaces/srv/RemoveBox)
  /wh/spawn_obstacle (wh_interfaces/srv/SpawnObstacle)
  /wh/clear_obstacle (std_srvs/srv/Trigger)
  /wh/get_box_pose   (wh_interfaces/srv/GetBoxPose)

Dùng /spawn_entity và /delete_entity của gazebo_ros.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelStates
try:
    from wh_interfaces.srv import SpawnObstacle
    HAS_WH_IFACES = True
except ImportError:
    HAS_WH_IFACES = False

import yaml

# ── SDF template cho hộp ─────────────────────────────────────────────────────
BOX_SDF_TEMPLATE = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="box_{bid}">
    <pose>{px} {py} {pz} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <!-- Hộp 0.06×0.06×0.04  m=0.2 kg -->
          <!-- Ix = m/12*(y^2+z^2) -->
          <ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.6f}</iyy><iyz>0</iyz>
          <izz>{izz:.6f}</izz>
        </inertia>
      </inertial>
      <collision name="col">
        <max_contacts>4</max_contacts>
        <geometry><box><size>0.06 0.06 0.04</size></box></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
          <contact><ode><max_vel>0.01</max_vel><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>
      <!-- Visual thân hộp -->
      <visual name="vis_body">
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>0.06 0.06 0.04</size></box></geometry>
        <material>
          <ambient>0.9 0.7 0.3 1.0</ambient>
          <diffuse>0.9 0.7 0.3 1.0</diffuse>
        </material>
      </visual>
      <!-- Tag mặt trên (tag_id = bid + 20) -->
      <visual name="vis_tag_top">
        <cast_shadows>false</cast_shadows>
        <pose>0 0 0.0203 0 0 0</pose>
        <geometry><box><size>0.036 0.036 0.0005</size></box></geometry>
        <material>
          <script>
            <uri>model://aruco_tags/materials/scripts</uri>
            <uri>model://aruco_tags/materials/textures</uri>
            <name>Aruco/Tag{top_tag_id}</name>
          </script>
        </material>
      </visual>
      <!-- Tag mặt trước (-y quay ra ngoài) -->
      <visual name="vis_tag_front">
        <cast_shadows>false</cast_shadows>
        <pose>0 -0.0303 0 0 0 3.141593</pose>
        <geometry><box><size>0.036 0.0005 0.036</size></box></geometry>
        <material>
          <script>
            <uri>model://aruco_tags/materials/scripts</uri>
            <uri>model://aruco_tags/materials/textures</uri>
            <name>Aruco/Tag{front_tag_id}</name>
          </script>
        </material>
      </visual>
      <!-- Tag mặt sau (+y) -->
      <visual name="vis_tag_back">
        <cast_shadows>false</cast_shadows>
        <pose>0 0.0303 0 0 0 0</pose>
        <geometry><box><size>0.036 0.0005 0.036</size></box></geometry>
        <material>
          <script>
            <uri>model://aruco_tags/materials/scripts</uri>
            <uri>model://aruco_tags/materials/textures</uri>
            <name>Aruco/Tag{front_tag_id}</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.05</linear>
        <angular>0.05</angular>
      </velocity_decay>
    </link>
  </model>
</sdf>
"""

# ── SDF template cho vật cản động ────────────────────────────────────────────
OBS_SDF_TEMPLATE = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="obstacle_{oid}">
    <static>true</static>
    <pose>{px} {py} {pz} 0 0 0</pose>
    <link name="link">
      <collision name="col">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="vis">
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.9 0.3 0.1 1.0</ambient>
          <diffuse>0.9 0.3 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _box_inertia(lx=0.06, ly=0.06, lz=0.04, m=0.2):
    ixx = m / 12.0 * (ly**2 + lz**2)
    iyy = m / 12.0 * (lx**2 + lz**2)
    izz = m / 12.0 * (lx**2 + ly**2)
    return ixx, iyy, izz


class BoxManager(Node):

    def __init__(self):
        super().__init__("box_manager")
        self.cbg = ReentrantCallbackGroup()

        # ── State ──────────────────────────────────────────────────────────
        self._boxes: dict = {}        # tag_id → model_name
        self._obstacles: dict = {}    # oid → model_name
        self._obs_counter = 0
        self._model_states: ModelStates = None

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            ModelStates, "/gazebo/model_states",
            self._cb_model_states, 10
        )

        # ── Gazebo service clients ─────────────────────────────────────────
        self._spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        # ── Load slots yaml ────────────────────────────────────────────────
        pkg_share = os.path.join(
            os.path.dirname(__file__), "..", "..", "wh_navigation", "config"
        )
        slots_file = os.path.join(pkg_share, "slots.yaml")
        self._slots = {}
        if os.path.exists(slots_file):
            with open(slots_file) as f:
                data = yaml.safe_load(f)
                self._slots = data.get("slots", {})
            self.get_logger().info(f"Loaded {len(self._slots)} slots from {slots_file}")
        else:
            self.get_logger().warn(f"slots.yaml not found at {slots_file}")

        # ── Services (dùng std_srvs/Trigger + tham số đơn giản) ───────────
        # Vì wh_interfaces chưa sinh, dùng Trigger+param approach thông qua
        # ROS2 parameters để spawn box. Khi wh_interfaces có, thay thế.
        self.create_service(Trigger, "/wh/clear_obstacle",
                            self._svc_clear_obstacle, callback_group=self.cbg)
        if HAS_WH_IFACES:
            self.create_service(SpawnObstacle, "/wh/spawn_obstacle",
                                self._svc_spawn_obstacle, callback_group=self.cbg)

        self.get_logger().info("BoxManager node ready.")
        self.get_logger().info(
            "Use CLI: ros2 service call /wh/spawn_box_by_id ..."
        )

        # ── Timer để spawn seed boxes sau khi Gazebo sẵn sàng ────────────
        self._seed_timer = self.create_timer(5.0, self._spawn_seed_boxes)
        self._seed_done = False

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_model_states(self, msg: ModelStates):
        self._model_states = msg

    # ── Wait for Gazebo ───────────────────────────────────────────────────────
    def _wait_gazebo(self, timeout=30.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if (self._spawn_cli.wait_for_service(timeout_sec=1.0) and
                    self._delete_cli.wait_for_service(timeout_sec=1.0)):
                return True
        return False

    # ── Spawn box ─────────────────────────────────────────────────────────────
    def spawn_box(self, box_tag_id: int, x: float, y: float, z: float) -> bool:
        """Spawn hộp với tag_id cho hộp.  top_tag = box_tag_id (20-39)."""
        if not self._wait_gazebo(timeout=10.0):
            self.get_logger().error("Gazebo /spawn_entity not available")
            return False

        ixx, iyy, izz = _box_inertia()
        model_name = f"box_{box_tag_id}"

        # tag mặt trên và mặt trước dùng cùng id nếu chỉ có 1 tag id
        # Theo spec: box có 2 tag (top + front) – dùng box_tag_id cho top,
        # box_tag_id+1 cho front nếu có, hoặc cùng id.
        front_tag_id = box_tag_id  # đơn giản: dùng cùng id

        sdf = BOX_SDF_TEMPLATE.format(
            bid=box_tag_id,
            px=x, py=y, pz=z,
            ixx=ixx, iyy=iyy, izz=izz,
            top_tag_id=box_tag_id,
            front_tag_id=front_tag_id,
        )

        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = sdf
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self._spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self._boxes[box_tag_id] = model_name
            self.get_logger().info(
                f"Spawned box_{box_tag_id} at ({x:.3f},{y:.3f},{z:.3f})"
            )
            return True
        else:
            err = future.result().status_message if future.result() else "timeout"
            self.get_logger().error(f"Failed to spawn box_{box_tag_id}: {err}")
            return False

    def spawn_box_in_slot(self, box_tag_id: int, slot_tag_id: int) -> bool:
        """Spawn hộp vào ô kệ có slot_tag_id."""
        slot = self._slots.get(slot_tag_id)
        if slot is None:
            self.get_logger().error(f"Slot {slot_tag_id} not found in slots.yaml")
            return False
        bc = slot["box_center_pose"]
        return self.spawn_box(box_tag_id, bc["x"], bc["y"], bc["z"])

    # ── Remove box ────────────────────────────────────────────────────────────
    def remove_box(self, box_tag_id: int) -> bool:
        model_name = self._boxes.get(box_tag_id, f"box_{box_tag_id}")
        if not self._wait_gazebo(timeout=10.0):
            return False

        req = DeleteEntity.Request()
        req.name = model_name
        future = self._delete_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self._boxes.pop(box_tag_id, None)
            self.get_logger().info(f"Removed {model_name}")
            return True
        else:
            err = future.result().status_message if future.result() else "timeout"
            self.get_logger().error(f"Failed to remove {model_name}: {err}")
            return False

    # ── Spawn obstacle ────────────────────────────────────────────────────────
    def spawn_obstacle(self, x: float, y: float, z: float,
                       sx: float = 0.3, sy: float = 0.3, sz: float = 0.3) -> int:
        self._obs_counter += 1
        oid = self._obs_counter

        if not self._wait_gazebo(timeout=10.0):
            return -1

        sdf = OBS_SDF_TEMPLATE.format(
            oid=oid,
            px=x, py=y, pz=z + sz / 2.0,
            sx=sx, sy=sy, sz=sz
        )

        req = SpawnEntity.Request()
        req.name = f"obstacle_{oid}"
        req.xml = sdf
        future = self._spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self._obstacles[oid] = f"obstacle_{oid}"
            self.get_logger().info(f"Spawned obstacle_{oid} at ({x},{y},{z})")
            return oid
        return -1

    def _svc_spawn_obstacle(self, req, resp):
        sx = req.sx if req.sx > 0 else 0.3
        sy = req.sy if req.sy > 0 else 0.3
        sz = req.sz if req.sz > 0 else 0.3
        oid = self.spawn_obstacle(req.x, req.y, req.z, sx, sy, sz)
        if oid > 0:
            resp.success = True
            resp.obstacle_id = oid
            resp.message = f"Spawned obstacle_{oid} at ({req.x:.2f}, {req.y:.2f}, {req.z:.2f})"
        else:
            resp.success = False
            resp.obstacle_id = -1
            resp.message = "Failed to spawn obstacle"
        return resp

    # ── Clear obstacles ───────────────────────────────────────────────────────
    def _svc_clear_obstacle(self, req, resp):
        count = 0
        for oid, name in list(self._obstacles.items()):
            dreq = DeleteEntity.Request()
            dreq.name = name
            future = self._delete_cli.call_async(dreq)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().success:
                self._obstacles.pop(oid)
                count += 1
        resp.success = True
        resp.message = f"Cleared {count} obstacles"
        return resp

    # ── Get box pose ──────────────────────────────────────────────────────────
    def get_box_pose(self, box_tag_id: int):
        if self._model_states is None:
            return None
        name = f"box_{box_tag_id}"
        try:
            idx = self._model_states.name.index(name)
            return self._model_states.pose[idx]
        except ValueError:
            return None

    # ── Seed spawn ────────────────────────────────────────────────────────────
    def _spawn_seed_boxes(self):
        """Chạy một lần sau 5s để thả hộp theo inventory_seed.yaml."""
        if self._seed_done:
            return
        self._seed_done = True
        self._seed_timer.cancel()

        seed_file = os.path.join(
            os.path.dirname(__file__), "..", "config", "inventory_seed.yaml"
        )
        if not os.path.exists(seed_file):
            self.get_logger().warn("inventory_seed.yaml not found, skipping seed spawn")
            return

        with open(seed_file) as f:
            seed = yaml.safe_load(f)

        for entry in seed.get("boxes", []):
            box_id = entry["box_id"]
            slot_id = entry.get("slot_id")
            if slot_id is not None:
                self.spawn_box_in_slot(box_id, slot_id)
            else:
                pose = entry.get("pose", {})
                self.spawn_box(box_id,
                               pose.get("x", 0.0),
                               pose.get("y", 0.0),
                               pose.get("z", 0.1))
            time.sleep(0.3)   # tránh flood service


def main(args=None):
    rclpy.init(args=args)
    node = BoxManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
