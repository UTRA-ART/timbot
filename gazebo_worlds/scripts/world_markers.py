#!/usr/bin/env python3
"""Publish a Gazebo .world file's static scenery as markers, for Foxglove/RViz.

Reads the SDF and publishes two latched MarkerArrays:
  /gazebo_world         boxes and cylinders as-is, and mesh visuals (e.g. the
                        construction barrels) as their collision shape
  /gazebo_world/ground  the ground plane; the IGVC pavement as a textured mesh
                        (worlds/pavement_plane.dae, the heaviest part to render)
Viewers can hide either topic in Foxglove/RViz; a hidden topic isn't sent.
The markers live in a `gazebo_world` frame, attached to `map` through the
robot's spawn pose (map starts where the robot spawns), so any offset between
the scenery and the sensor data is localization drift.

Robot: /robot_lowpoly draws the robot at its estimated pose (its TF link
frames) from the low-poly meshes in description/rover_model/meshes/lowpoly,
~25x lighter than the full meshes /robot_description would load.

Ground truth: if true_pose_topic is set (Gazebo's dynamic_pose/info bridged
to a tf2_msgs/TFMessage), the same low-poly robot is drawn as a green ghost at
the robot's true pose on /gazebo_world/ground_truth (in gazebo_world, 20 Hz).
Where the two robots separate, the localization estimate is off. Gazebo's pose
list is only read while something (e.g. Foxglove) subscribes to the ghost.

Parameters: world_file (name in gazebo_worlds/worlds or absolute path),
spawn_x, spawn_y, spawn_yaw (same values as the launch config's spawn stage),
true_pose_topic (default: none), robot_name (Gazebo model name, default timbot).
"""
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'gazebo_world'
PAVEMENT_TEXTURE = 'IGVC_pavement.png'
PAVEMENT_MESH = 'package://gazebo_worlds/worlds/pavement_plane.dae'
DEFAULT_RGBA = (0.6, 0.6, 0.6, 1.0)
MESH_STANDIN_RGBA = (1.0, 0.45, 0.0, 1.0)  # barrels
ROBOT_RGBA = (0.75, 0.75, 0.78, 1.0)
GHOST_RGBA = (0.2, 1.0, 0.3, 0.45)
LOWPOLY = 'package://description/rover_model/meshes/lowpoly/{}.STL'
# Low-poly robot parts: (link, xyz offset from base_link, pitch), offsets summed
# from the URDF's joints (the ghost has no TF frames of its own, so it needs them).
ROBOT_PARTS = [('chassis_link', (0.2573, 0.0, 0.1301), 0.0),
               ('left_wheel_link', (0.2573, 0.3372, 0.1301), 0.0),
               ('right_wheel_link', (0.2573, -0.3372, 0.1301), 0.0),
               ('caster_link', (-0.4157, 0.0, 0.2003), 0.0),
               ('stand_link', (0.1305, 0.0, 0.7849), 0.0),
               ('zed_link', (0.2041, 0.0, 0.8449), 0.4712389),
               ('top_lidar_link', (0.4665, 0.0005, 0.4771), 0.0),
               ('bottom_lidar_link', (0.5108, 0.0, 0.2739), 0.0)]


def robot_markers(ns, rgba, base=None):
    """Low-poly robot: on each link's own TF frame (base=None, the estimate), or placed
    in FRAME from a 4x4 base_link pose (the ghost)."""
    out = MarkerArray()
    for i, (link, offset, pitch) in enumerate(ROBOT_PARTS):
        m = Marker()
        m.ns, m.id, m.action, m.type = ns, i, Marker.ADD, Marker.MESH_RESOURCE
        m.mesh_resource = LOWPOLY.format(link)
        m.frame_locked = True  # follow the (moving) frame, not its pose when published
        if base is None:
            m.header.frame_id = link
            m.pose.orientation.w = 1.0
        else:
            m.header.frame_id = FRAME
            part = base @ pose_matrix(f'{offset[0]} {offset[1]} {offset[2]} 0 {pitch} 0')
            m.pose.position.x, m.pose.position.y, m.pose.position.z = part[:3, 3]
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = quaternion(part[:3, :3])
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        out.markers.append(m)
    return out


def pose_matrix(text):
    """SDF <pose> 'x y z roll pitch yaw' -> 4x4 transform (identity if absent)."""
    v = [float(s) for s in text.split()] if text else [0.0] * 6
    x, y, z, r, p, yw = (v + [0.0] * 6)[:6]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = [x, y, z]
    return m


def quaternion(rot):
    """3x3 rotation matrix -> (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1.0 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + rot[0, 0] - rot[1, 1] - rot[2, 2])) / 2, rot[2, 1] - rot[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - rot[0, 0] + rot[1, 1] - rot[2, 2])) / 2, rot[0, 2] - rot[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - rot[0, 0] - rot[1, 1] + rot[2, 2])) / 2, rot[1, 0] - rot[0, 1])
    return x, y, z, w


def visual_rgba(visual):
    for tag in ('diffuse', 'ambient'):
        text = visual.findtext(f'material/{tag}')
        if text:
            c = [float(s) for s in text.split()]
            return tuple((c + [1.0])[:4])
    return DEFAULT_RGBA


class WorldMarkers(Node):
    def __init__(self):
        super().__init__('gazebo_world_markers')
        world = self.declare_parameter('world_file', 'track.world').value
        # The launch configs store the spawn pose as strings, so accept any type.
        any_type = ParameterDescriptor(dynamic_typing=True)
        spawn = [float(self.declare_parameter(n, d, any_type).value)
                 for n, d in (('spawn_x', -18.53), ('spawn_y', 0.0), ('spawn_yaw', 1.5708))]
        if not os.path.isabs(world):
            world = os.path.join(get_package_share_directory('gazebo_worlds'), 'worlds', world)

        self.markers = MarkerArray()  # objects
        self.ground = MarkerArray()
        self.ids = {}
        for model in ET.parse(world).getroot().find('world').findall('model'):
            self.add_model(model, np.eye(4))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(MarkerArray, 'gazebo_world', latched).publish(self.markers)
        self.create_publisher(MarkerArray, 'gazebo_world/ground', latched).publish(self.ground)

        # map is where the robot spawned, so gazebo_world = inverse(spawn pose) in map.
        x, y, yaw = spawn
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = FRAME
        tf.transform.translation.x = -(math.cos(yaw) * x + math.sin(yaw) * y)
        tf.transform.translation.y = math.sin(yaw) * x - math.cos(yaw) * y
        tf.transform.rotation.z = math.sin(-yaw / 2)
        tf.transform.rotation.w = math.cos(-yaw / 2)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster.sendTransform(tf)
        truth_topic = self.declare_parameter('true_pose_topic', '').value
        self.robot_name = self.declare_parameter('robot_name', 'timbot').value
        if truth_topic:
            self.last_truth = 0.0
            self.truth_topic, self.truth_sub = truth_topic, None
            self.ghost_pub = self.create_publisher(MarkerArray, 'gazebo_world/ground_truth', latched)
            # Until the first true pose arrives, show the ghost at the spawn pose.
            self.ghost_pub.publish(robot_markers('ground_truth', GHOST_RGBA,
                                                 pose_matrix(f'{x} {y} 0 0 0 {yaw}')))
            # Unpacking Gazebo's pose list costs ~1/3 of a core, so only do it while
            # someone (e.g. Foxglove) is subscribed to the ghost.
            self.create_timer(1.0, self.update_truth_subscription)
        self.create_publisher(MarkerArray, 'robot_lowpoly', latched).publish(
            robot_markers('robot', ROBOT_RGBA))

        self.get_logger().info(
            f'Published {len(self.markers.markers)} objects and {len(self.ground.markers)} ground planes '
            f'from {os.path.basename(world)} on /gazebo_world and /gazebo_world/ground')

    def update_truth_subscription(self):
        watched = self.ghost_pub.get_subscription_count() > 0
        if watched and self.truth_sub is None:
            self.truth_sub = self.create_subscription(TFMessage, self.truth_topic, self.on_true_poses, 10)
        elif not watched and self.truth_sub is not None:
            self.destroy_subscription(self.truth_sub)
            self.truth_sub = None

    def on_true_poses(self, msg):
        now = time.monotonic()
        if now - self.last_truth < 0.05:
            return  # 20 Hz is plenty
        for t in msg.transforms:
            if t.child_frame_id != self.robot_name:
                continue
            self.last_truth = now
            p, q = t.transform.translation, t.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            base = np.eye(4)
            base[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            base[:3, 3] = [p.x, p.y, p.z]
            self.ghost_pub.publish(robot_markers('ground_truth', GHOST_RGBA, base))
            return

    def add_model(self, model, parent):
        # Namespace by model name without its number: "Construction Barrel_12" -> "Construction Barrel"
        ns = re.sub(r'[_ ]?\d+$', '', model.get('name', 'model'))
        model_tf = parent @ pose_matrix(model.findtext('pose'))
        for nested in model.findall('model'):
            self.add_model(nested, model_tf)
        for link in model.findall('link'):
            link_tf = model_tf @ pose_matrix(link.findtext('pose'))
            for visual in link.findall('visual'):
                geom = visual.find('geometry')
                if geom is None or len(geom) == 0:
                    continue
                shape = geom[0]
                tf = link_tf @ pose_matrix(visual.findtext('pose'))
                if shape.tag == 'mesh':
                    # Meshes from model:// paths can't be served to Foxglove; draw the collision shape.
                    collision = next((c for c in link.findall('collision')
                                      if c.find('geometry/box') is not None
                                      or c.find('geometry/cylinder') is not None), None)
                    if collision is None:
                        continue
                    shape = collision.find('geometry')[0]
                    tf = link_tf @ pose_matrix(collision.findtext('pose'))
                    self.add(ns, shape, tf, MESH_STANDIN_RGBA)
                else:
                    self.add(ns, shape, tf, visual_rgba(visual), visual)

    def add(self, ns, shape, tf, rgba, visual=None):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = ns
        m.id = self.ids[ns] = self.ids.get(ns, -1) + 1
        m.action = Marker.ADD
        if shape.tag == 'box':
            m.type = Marker.CUBE
            m.scale.x, m.scale.y, m.scale.z = (float(s) for s in shape.findtext('size').split())
        elif shape.tag == 'cylinder':
            m.type = Marker.CYLINDER
            d = 2 * float(shape.findtext('radius'))
            m.scale.x = m.scale.y = d
            m.scale.z = float(shape.findtext('length'))
        elif shape.tag == 'plane':
            sx, sy = (float(s) for s in shape.findtext('size').split())
            texture = visual.findtext('material/pbr/metal/albedo_map') if visual is not None else None
            if texture and os.path.basename(texture) == PAVEMENT_TEXTURE:
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = PAVEMENT_MESH
                m.mesh_use_embedded_materials = True
                rgba = (1.0, 1.0, 1.0, 1.0)
                m.scale.z = 1.0
            else:
                m.type = Marker.CUBE
                m.scale.z = 0.001
                rgba = (0.35, 0.35, 0.35, 1.0)
            m.scale.x, m.scale.y = sx, sy
            tf = tf.copy()
            tf[2, 3] -= 0.02  # just under /map and the costmaps, which sit at z=0
        else:
            return
        m.pose.position.x, m.pose.position.y, m.pose.position.z = tf[:3, 3]
        q = quaternion(tf[:3, :3])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        (self.ground if shape.tag == 'plane' else self.markers).markers.append(m)


def main():
    rclpy.init()
    node = WorldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
