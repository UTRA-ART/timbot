#!/usr/bin/env python3
"""Simulated ZED depth by ray-casting the Gazebo world on the CPU (sim_depth_source: raycast).

Gazebo's depth camera is what drags the simulation below real time when it renders on
the CPU (no GPU, e.g. Docker on macOS): with it the sim ran at ~0.4x real time, with a
colour-only camera at 1.0x. In raycast mode the URDF camera is colour-only and this node
supplies the depth half instead. For every colour frame it casts one ray per pixel from
the camera's true pose in Gazebo, using Open3D's RaycastingScene (Embree), and publishes
a depth image and point cloud with the colour frame's stamp. lane_detection's RGB+depth
sync and depth_detection's cloud input work unchanged. ~8 ms per 640x320 frame.

Output matches Gazebo's rgbd_camera (checked pixel by pixel: identical depth):
  depth image  32FC1, metres along the camera's forward axis, +inf where nothing was hit
  point cloud  xyz32 in the camera link frame (x forward, y left, z up)
Differences: the scene is every <visual> in the world file (box, cylinder, sphere, plane,
mesh) loaded once at startup, so the rover's own body is missing (Gazebo's depth shows
part of it at the bottom of the image) and models that move during the run, e.g. a
barrel the rover pushes, stay where the world file put them.

Needs Open3D (pip install open3d==0.18.0 trimesh pycollada; see docker/Dockerfile) and
the robot's true pose: Gazebo's dynamic_pose/info bridged to a tf2_msgs/TFMessage
(spawn.launch.py does this when sim_depth_source is raycast).

Parameters: world_file (name in gazebo_worlds/worlds or absolute path), model_paths
(':'-separated model:// search path; default: IGN_GAZEBO_RESOURCE_PATH, else what
gazebo.launch.py uses), robot_name, base_frame, camera_frame, image_topic, info_topic,
depth_topic, points_topic, true_pose_topic, near_clip, far_clip (match the URDF sensor),
noise_stddev (default 0: Gazebo's depth has none), threads (0 = all cores).
"""
import os

# numpy's OpenBLAS otherwise keeps one busy-waiting thread per core after every
# matrix multiply (measured: ~10 cores spinning for ~10 ms of real work per frame).
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import array  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from collections import deque  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

try:
    import open3d as o3d
    import trimesh
except ImportError as exc:
    raise SystemExit(f'depth_raycast needs Open3D and trimesh ({exc}); '
                     'pip install open3d==0.18.0 trimesh pycollada') from exc


def pose_mat(text):
    """4x4 transform from an SDF <pose> 'x y z roll pitch yaw'."""
    v = [float(x) for x in (text.split() if text else [])] or [0.0] * 6
    x, y, z, r, p, yw = v
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = [x, y, z]
    return T


def tf_mat(t, q):
    """4x4 transform from a geometry_msgs translation + quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class WorldScene:
    """Every <visual> of a world file as one Open3D raycasting scene."""

    def __init__(self, world_file, model_paths, logger):
        self.model_paths, self.log, self.cache = model_paths, logger, {}
        self.scene = o3d.t.geometry.RaycastingScene()
        self.n_obj = self.n_tri = 0
        world = ET.parse(world_file).getroot().find('world')
        for model in world.findall('model'):
            self.add_model(model, np.eye(4))
        if world.find('include') is not None:
            self.log.warn('<include> elements are not loaded into the depth scene')

    def resolve(self, uri):
        rel = uri.strip().replace('model://', '')
        for base in self.model_paths:
            path = os.path.join(base, rel)
            if os.path.exists(path):
                return path
        raise FileNotFoundError(f'{uri} not found in {self.model_paths}')

    def mesh(self, g):
        if (b := g.find('box')) is not None:
            return trimesh.creation.box(extents=[float(s) for s in b.findtext('size').split()])
        if (c := g.find('cylinder')) is not None:
            return trimesh.creation.cylinder(radius=float(c.findtext('radius')),
                                             height=float(c.findtext('length')), sections=32)
        if (s := g.find('sphere')) is not None:
            return trimesh.creation.icosphere(radius=float(s.findtext('radius')))
        if (pl := g.find('plane')) is not None:
            sx, sy = [float(v) for v in (pl.findtext('size') or '1000 1000').split()]
            n = np.array([float(v) for v in (pl.findtext('normal') or '0 0 1').split()])
            box = trimesh.creation.box(extents=[sx, sy, 1e-3])
            box.apply_translation([0, 0, -5e-4])  # top face exactly on the plane
            if not np.allclose(n, [0, 0, 1]):
                box.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], n))
            return box
        if (m := g.find('mesh')) is not None:
            path = self.resolve(m.findtext('uri'))
            if path not in self.cache:
                tm = trimesh.load(path, force='mesh')
                if path.lower().endswith('.dae'):
                    unit = ET.parse(path).getroot().find('.//{*}asset/{*}unit')
                    if unit is not None:  # Gazebo applies the COLLADA <unit>; trimesh doesn't
                        tm.apply_scale(float(unit.get('meter', 1.0)))
                self.cache[path] = tm
            tm = self.cache[path].copy()
            if m.findtext('scale'):
                tm.apply_scale([float(v) for v in m.findtext('scale').split()])
            return tm
        return None

    def add_model(self, model, parent):
        Tm = parent @ pose_mat(model.findtext('pose'))
        for nested in model.findall('model'):
            self.add_model(nested, Tm)
        for link in model.findall('link'):
            Tl = Tm @ pose_mat(link.findtext('pose'))
            for vis in link.findall('visual'):
                geom = vis.find('geometry')
                tm = self.mesh(geom) if geom is not None else None
                if tm is None:
                    continue
                tm.apply_transform(Tl @ pose_mat(vis.findtext('pose')))
                self.scene.add_triangles(o3d.core.Tensor(np.asarray(tm.vertices, np.float32)),
                                         o3d.core.Tensor(np.asarray(tm.faces, np.uint32)))
                self.n_obj += 1
                self.n_tri += len(tm.faces)


class DepthRaycast(Node):
    def __init__(self):
        super().__init__('depth_raycast')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        share = get_package_share_directory('gazebo_worlds')
        world_file = p('world_file', 'track.world')
        if not os.path.isabs(world_file):
            world_file = os.path.join(share, 'worlds', world_file)
        default_paths = os.environ.get('IGN_GAZEBO_RESOURCE_PATH') or ':'.join(
            [os.path.expanduser('~/.gazebo/models'), os.path.join(share, 'worlds')])
        model_paths = [m for m in (p('model_paths', '') or default_paths).split(':') if m]
        self.robot = p('robot_name', 'timbot')
        self.base_frame = p('base_frame', 'base_link')
        self.camera_frame = p('camera_frame', 'left_camera_link')
        self.near, self.far = p('near_clip', 0.3), p('far_clip', 300.0)
        self.noise = p('noise_stddev', 0.0)
        self.threads = p('threads', 0)
        self.max_pose_age = p('max_pose_age', 0.05)

        t0 = time.perf_counter()
        self.world = WorldScene(world_file, model_paths, self.get_logger())
        self.get_logger().info(
            f'Depth scene from {os.path.basename(world_file)}: {self.world.n_obj} visuals, '
            f'{self.world.n_tri} triangles in {time.perf_counter() - t0:.2f} s')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_base_cam = None
        self.poses = deque(maxlen=200)  # (stamp_ns, T_world_base)
        self.dirs_cam = None
        self.size = None
        self.rng = np.random.default_rng()
        self.stats = deque(maxlen=100)

        self.pub_depth = self.create_publisher(Image, p('depth_topic', '/zed_node/left/depth_image'), 10)
        self.pub_points = self.create_publisher(PointCloud2, p('points_topic', '/zed_node/left/points'), 10)
        self.create_subscription(TFMessage, p('true_pose_topic', '/gazebo/_true_poses'), self.on_poses, 50)
        self.create_subscription(CameraInfo, p('info_topic', '/zed_node/left/camera_info'), self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, p('image_topic', '/zed_node/left/image'), self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(30.0, self.report)

    def on_poses(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self.robot:
                # The bridged dynamic_pose/info carries no stamps; fall back to arrival time.
                stamp = Time.from_msg(t.header.stamp).nanoseconds or self.get_clock().now().nanoseconds
                self.poses.append((stamp, tf_mat(t.transform.translation, t.transform.rotation)))
                return

    def on_info(self, msg):
        if self.size == (msg.width, msg.height):
            return
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.size = (msg.width, msg.height)
        uu, vv = np.meshgrid(np.arange(msg.width, dtype=np.float32), np.arange(msg.height, dtype=np.float32))
        # Camera link frame: x forward, y left, z up; x component 1, so t_hit == depth.
        # Gazebo renders pixel u through its centre u + 0.5 while publishing cx = width / 2,
        # so cast through the centre too, or depth lands half a pixel off the colour image.
        self.dirs_cam = np.stack([np.ones(uu.size, np.float32), -((uu + 0.5 - cx) / fx).ravel(),
                                  -((vv + 0.5 - cy) / fy).ravel()], 1)
        self.get_logger().info(f'Camera {msg.width}x{msg.height}, fx={fx:.1f} fy={fy:.1f}')

    def pose_at(self, stamp_ns):
        if not self.poses:
            return None, None
        stamp, T = min(self.poses, key=lambda st: abs(st[0] - stamp_ns))
        return T, (stamp - stamp_ns) * 1e-9

    def on_image(self, img):
        if self.dirs_cam is None or (img.width, img.height) != self.size:
            return
        if self.T_base_cam is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
            except Exception as exc:  # noqa: BLE001 (tf2 raises several types)
                self.get_logger().warn(f'waiting for {self.base_frame}->{self.camera_frame}: {exc}',
                                       throttle_duration_sec=5.0)
                return
            self.T_base_cam = tf_mat(tf.transform.translation, tf.transform.rotation)
        T_wb, dt = self.pose_at(Time.from_msg(img.header.stamp).nanoseconds)
        if T_wb is None:
            self.get_logger().warn('no true pose from Gazebo yet', throttle_duration_sec=5.0)
            return
        if abs(dt) > self.max_pose_age:
            self.get_logger().warn(f'nearest true pose is {dt * 1e3:.0f} ms from the image',
                                   throttle_duration_sec=5.0)

        s = time.perf_counter()
        T = T_wb @ self.T_base_cam
        rays = np.empty((len(self.dirs_cam), 6), np.float32)
        rays[:, :3] = T[:3, 3]
        rays[:, 3:] = self.dirs_cam @ T[:3, :3].T.astype(np.float32)
        depth = self.world.scene.cast_rays(o3d.core.Tensor(rays), nthreads=self.threads)['t_hit'].numpy()
        ok = (depth >= self.near) & (depth <= self.far)
        if self.noise > 0:
            depth[ok] += self.rng.normal(0.0, self.noise, int(ok.sum())).astype(np.float32)
        depth[~ok] = np.inf

        w, h = self.size
        dm = Image(header=img.header, height=h, width=w, encoding='32FC1', is_bigendian=0, step=w * 4)
        # array.array, not bytes: rclpy checks a bytes value one element at a time (~60 ms/MB).
        dm.data = array.array('B', depth.astype(np.float32).tobytes())
        self.pub_depth.publish(dm)

        d = depth[ok]
        pc = PointCloud2(header=img.header, height=1, width=len(d), is_bigendian=False,
                         point_step=12, row_step=12 * len(d), is_dense=True)
        pc.header.frame_id = self.camera_frame
        pc.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                     for i, n in enumerate('xyz')]
        pc.data = array.array('B', (self.dirs_cam[ok] * d[:, None]).astype(np.float32).tobytes())
        self.pub_points.publish(pc)
        self.stats.append(((time.perf_counter() - s) * 1e3, abs(dt) * 1e3))

    def report(self):
        if self.stats:
            a = np.array(self.stats)
            self.get_logger().info(
                f'last {len(a)} frames: {np.median(a[:, 0]):.1f} ms per frame (median); '
                f'true pose {np.median(a[:, 1]):.0f} ms from the image (median), {a[:, 1].max():.0f} ms max')


def main():
    rclpy.init()
    node = DepthRaycast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
