#!/usr/bin/env python3
"""
scan_3d_mapper.py

Builds a 3D point cloud out of a 2D lidar by exploiting the one extra degree of
freedom a drone has that a ground robot does not: altitude.

The RPLIDAR C1 sweeps a single horizontal plane. At a fixed height that plane
only ever traces one contour of the room. Change altitude and each sweep cuts
the room at a different height; stack the sweeps in the map frame and you get a
genuine 3D reconstruction of the walls, door frames, furniture edges and
ceiling features you flew past.

The catch is that stacking only works if every point is placed at the height it
was actually measured at. The 2D TF chain (map -> odom -> base_link) is planar,
so it would pile every sweep onto z = 0. This node therefore uses
`base_link_3d` from pose_3d_node, which carries PX4's altitude, as the frame it
transforms scans out of.

Storage is a voxel hash: each point is snapped to a `voxel_size` grid cell and
kept once. That bounds memory no matter how long you fly, and it is what keeps
this affordable on a Pi -- a 20-minute flight collapses to tens of thousands of
voxels instead of millions of raw points.

Topics
------
    sub   /scan                  sensor_msgs/LaserScan
    pub   /map_3d                sensor_msgs/PointCloud2  (frame: map)
    srv   ~/save                 std_srvs/Trigger  -> writes a .ply
    srv   ~/clear                std_srvs/Trigger  -> empties the voxel map

Save the cloud with:
    ros2 service call /scan_3d_mapper/save std_srvs/srv/Trigger
"""
import math
import os
import time
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)


class Scan3DMapper(Node):

    def __init__(self):
        super().__init__('scan_3d_mapper')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        # Frame published by pose_3d_node -- carries the real altitude.
        self.declare_parameter('body_frame', 'base_link_3d')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('max_voxels', 400000)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        # Integrating every scan at 10 Hz is wasted work: consecutive sweeps
        # from a hovering drone land in the same voxels.
        self.declare_parameter('integrate_hz', 4.0)
        self.declare_parameter('publish_hz', 0.5)
        # Skip integration entirely below this altitude, where the lidar is
        # mostly seeing the floor and its own landing gear.
        self.declare_parameter('min_altitude', 0.15)
        self.declare_parameter('save_dir', os.path.expanduser('~/maps'))

        gp = lambda n: self.get_parameter(n).value
        self.map_frame = gp('map_frame')
        self.body_frame = gp('body_frame')
        self.laser_frame = gp('laser_frame')
        self.base_frame = gp('base_frame')
        self.voxel = float(gp('voxel_size'))
        self.max_voxels = int(gp('max_voxels'))
        self.rmin = float(gp('range_min'))
        self.rmax = float(gp('range_max'))
        self.min_alt = float(gp('min_altitude'))
        self.save_dir = gp('save_dir')
        integrate_hz = float(gp('integrate_hz'))
        publish_hz = float(gp('publish_hz'))

        self._min_period = 1.0 / integrate_hz if integrate_hz > 0 else 0.0
        self._last_integrate = 0.0
        # OrderedDict, not set: when the cap is hit we evict the oldest voxels
        # so the map keeps following the drone instead of freezing.
        self.voxels = OrderedDict()
        self.dirty = False
        self._laser_offset = None    # base_link -> laser, resolved once
        self._last_warn = 0.0
        self._scans_used = 0
        self._scans_skipped = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, gp('scan_topic'),
                                 self._scan_cb, scan_qos)

        # Transient local so RViz shows the accumulated cloud the moment it
        # subscribes, instead of waiting for the next publish tick.
        cloud_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/map_3d', cloud_qos)

        self.create_service(Trigger, '~/save', self._save_cb)
        self.create_service(Trigger, '~/clear', self._clear_cb)

        if publish_hz > 0:
            self.create_timer(1.0 / publish_hz, self._publish_cloud)
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f'scan_3d_mapper up: {self.voxel*100:.0f} cm voxels, integrating at '
            f'{integrate_hz} Hz from {self.body_frame}. Fly at different '
            f'altitudes to fill in the third dimension.')

    # ------------------------------------------------------------------
    def _laser_transform(self):
        """base_link -> laser. Static, so resolve it once and cache it."""
        if self._laser_offset is not None:
            return self._laser_offset
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.laser_frame,
                                                 rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        self._laser_offset = (np.array([t.x, t.y, t.z]),
                              R.from_quat([q.x, q.y, q.z, q.w]).as_matrix())
        return self._laser_offset

    def _scan_cb(self, msg: LaserScan):
        now = time.time()
        if self._min_period and now - self._last_integrate < self._min_period:
            return

        offset = self._laser_transform()
        if offset is None:
            self._warn('waiting for the base_link->laser transform '
                       '(is odom_bridge_node running?)')
            return

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.body_frame,
                                                 rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self._warn(f'no {self.map_frame}->{self.body_frame} TF ({e}). '
                       f'Start pose_3d_node.')
            return

        t = tf.transform.translation
        if t.z < self.min_alt:
            self._scans_skipped += 1
            return

        self._last_integrate = now

        # --- scan -> cartesian points in the laser frame -------------------
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        lo = max(self.rmin, msg.range_min)
        hi = min(self.rmax, msg.range_max)
        good = np.isfinite(ranges) & (ranges > lo) & (ranges < hi)
        if not np.any(good):
            return
        r = ranges[good]
        a = angles[good]
        pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)

        # --- laser -> base_link -> map ------------------------------------
        off_xyz, off_rot = offset
        pts = pts @ off_rot.T + off_xyz

        q = tf.transform.rotation
        body_rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pts = pts @ body_rot.T + np.array([t.x, t.y, t.z])

        # --- voxel hash ----------------------------------------------------
        keys = np.floor(pts / self.voxel).astype(np.int32)
        centers = (keys.astype(np.float32) + 0.5) * self.voxel
        added = 0
        for key, c in zip(map(tuple, keys), centers):
            if key in self.voxels:
                continue
            self.voxels[key] = (float(c[0]), float(c[1]), float(c[2]))
            added += 1
        if added:
            self.dirty = True
            while len(self.voxels) > self.max_voxels:
                self.voxels.popitem(last=False)
        self._scans_used += 1

    # ------------------------------------------------------------------
    def _publish_cloud(self):
        if not self.dirty:
            return
        self.dirty = False
        # An empty cloud is still worth publishing once: it is how a ~/clear
        # wipes the latched map out of RViz instead of leaving it on screen.
        pts = np.fromiter(
            (v for xyz in self.voxels.values() for v in xyz),
            dtype=np.float32, count=len(self.voxels) * 3).reshape(-1, 3)
        # Fourth channel is height, so RViz can colour the cloud by altitude --
        # which is the whole point of a 3D map built this way.
        data = np.empty((pts.shape[0], 4), dtype=np.float32)
        data[:, :3] = pts
        data[:, 3] = pts[:, 2]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = data.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * data.shape[0]
        msg.is_dense = True
        msg.data = data.tobytes()
        self.cloud_pub.publish(msg)

    def _report(self):
        self.get_logger().info(
            f'{len(self.voxels)} voxels | {self._scans_used} scans integrated, '
            f'{self._scans_skipped} skipped (below {self.min_alt} m)')

    def _warn(self, text):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_warn > 5.0:
            self.get_logger().warn(text)
            self._last_warn = now

    # ------------------------------------------------------------------
    def _save_cb(self, request, response):
        if not self.voxels:
            response.success = False
            response.message = 'nothing to save -- the voxel map is empty'
            return response
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            path = os.path.join(self.save_dir,
                                time.strftime('map3d_%Y%m%d_%H%M%S.ply'))
            with open(path, 'w') as f:
                f.write('ply\nformat ascii 1.0\n')
                f.write(f'element vertex {len(self.voxels)}\n')
                f.write('property float x\nproperty float y\nproperty float z\n')
                f.write('end_header\n')
                for x, y, z in self.voxels.values():
                    f.write(f'{x:.4f} {y:.4f} {z:.4f}\n')
            response.success = True
            response.message = f'saved {len(self.voxels)} points to {path}'
        except Exception as e:
            response.success = False
            response.message = f'save failed: {e!r}'
        self.get_logger().info(response.message)
        return response

    def _clear_cb(self, request, response):
        n = len(self.voxels)
        self.voxels.clear()
        self.dirty = True
        self._publish_cloud()
        response.success = True
        response.message = f'cleared {n} voxels'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Scan3DMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
