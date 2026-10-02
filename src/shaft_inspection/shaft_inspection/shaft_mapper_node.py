#!/usr/bin/env python3
"""
shaft_mapper_node
=================

Builds the 3D product by MAPPING WITH KNOWN POSE -- no SLAM.

Each 2D scan is one cross-section of the bore.  Stack the slices along the
descent and you have the shaft.  The pose is already known (attitude from
EKF2, depth from the descent datum), so there is nothing for a SLAM back end
to solve, and running one here would only add the failure modes of scan
matching in a geometry that barely constrains it.

Two products are written on shutdown:

  shaft_cloud.pcd     the accumulated point cloud
  shaft_profile.csv   clearance / mean radius / extent versus depth

The CSV is the one that survives yaw drift.  Yaw is unobservable in a
symmetric bore, so it wanders, smearing the cloud AZIMUTHALLY -- a feature at
bearing 40 deg may be logged at 55 deg after a long descent.  Radius-versus-
depth does not care: bulges, spalling and blockages all show up in it
regardless.  Treat the cloud as a visual aid and the profile as the metric
deliverable, unless you add a yaw reference.
"""
import math
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, String

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

from .px4_topics import px4_qos, resolve
from .scan_geometry import (center_max_clearance, cross_section_metrics,
                            detilt_points)


def quat_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class ShaftMapper(Node):

    def __init__(self):
        super().__init__('shaft_mapper')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('out_dir', os.path.expanduser('~/shaft_maps'))
        self.declare_parameter('slice_thickness', 0.10)
        self.declare_parameter('max_points', 1500000)
        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('min_depth_step', 0.05)
        self.declare_parameter('self_filter_radius', 0.25)
        self.declare_parameter('lidar_yaw_offset_deg', 0.0)
        self.declare_parameter('lidar_upside_down', False)

        g = lambda k: self.get_parameter(k).value
        self.out_dir = str(g('out_dir'))
        self.slice_h = float(g('slice_thickness'))
        self.max_points = int(g('max_points'))
        self.min_step = float(g('min_depth_step'))
        self.self_filter = float(g('self_filter_radius'))
        self.yaw_off = math.radians(float(g('lidar_yaw_offset_deg')))
        self.upside_down = bool(g('lidar_upside_down'))

        qos = px4_qos()
        att_t = resolve(self, '/fmu/out/vehicle_attitude')
        lp_t = resolve(self, '/fmu/out/vehicle_local_position')
        self.create_subscription(VehicleAttitude, att_t, self._on_att, qos)
        self.create_subscription(VehicleLocalPosition, lp_t, self._on_lp, qos)
        self.create_subscription(LaserScan, g('scan_topic'), self._on_scan, 10)
        self.create_subscription(Float32, '/shaft/depth', self._on_depth, 10)
        self.create_subscription(String, '/shaft/state', self._on_state, 10)
        self._saved_for_done = False

        self.pub_cloud = self.create_publisher(PointCloud2, '/shaft/cloud', 1)

        self.roll = self.pitch = self.yaw = 0.0
        self.have_att = False
        self.xy = np.zeros(2)
        self.depth = None
        self.last_logged_depth = None

        self.points = []            # list of (N,3) arrays, shaft frame
        self.n_points = 0
        self.profile = []           # depth, clearance, r_mean, r_max, roundness
        self.publish_cloud = bool(g('publish_cloud'))

        os.makedirs(self.out_dir, exist_ok=True)
        self.create_timer(2.0, self._republish)
        self.get_logger().info(
            f'shaft_mapper up — mapping with known pose, writing to {self.out_dir}')

    # ---------------- callbacks ----------------
    def _on_att(self, m):
        self.roll, self.pitch, self.yaw = quat_to_rpy(m.q)
        self.have_att = True

    def _on_lp(self, m):
        self.xy = np.array([float(m.x), float(m.y)])

    def _on_state(self, m):
        # Save as soon as the mission finishes rather than relying on a clean
        # shutdown, which a killed launch or a flat battery never delivers.
        if m.data == 'DONE' and not self._saved_for_done:
            self._saved_for_done = True
            self.save()

    def _on_depth(self, m):
        self.depth = float(m.data)

    def _on_scan(self, scan: LaserScan):
        if not self.have_att or self.depth is None:
            return
        if self.n_points >= self.max_points:
            return
        # One slice per depth step: no point stacking 10 scans at the same z.
        if (self.last_logged_depth is not None and
                abs(self.depth - self.last_logged_depth) < self.min_step):
            return
        self.last_logged_depth = self.depth

        ranges, a_min, a_inc = scan.ranges, scan.angle_min, scan.angle_increment
        if self.upside_down:
            ranges = list(reversed(scan.ranges))
            a_min = -(scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1))
        pts = detilt_points(ranges, a_min + self.yaw_off, a_inc,
                            self.roll, self.pitch,
                            max(scan.range_min, self.self_filter, 1e-3),
                            scan.range_max)
        if pts.shape[0] < 20:
            return

        # Per-slice geometry, logged against depth.
        c = center_max_clearance(pts)
        if c['ok']:
            met = cross_section_metrics(pts, c['cx'], c['cy'])
            self.profile.append((self.depth, c['clearance'], met['r_mean'],
                                 met['r_max'], met['roundness']))

        # Level body -> shaft frame: rotate by yaw, offset by horizontal
        # position, and place at -depth.  z grows downward as depth grows.
        cy_, sy_ = math.cos(self.yaw), math.sin(self.yaw)
        gx = pts[:, 0] * cy_ - pts[:, 1] * sy_ + self.xy[0]
        gy = pts[:, 0] * sy_ + pts[:, 1] * cy_ + self.xy[1]
        gz = np.full(gx.shape, -self.depth)

        self.points.append(np.stack([gx, gy, gz], axis=1).astype(np.float32))
        self.n_points += gx.size

    # ---------------- output ----------------
    def _republish(self):
        if not self.publish_cloud or not self.points:
            return
        cloud = np.concatenate(self.points, axis=0)
        if cloud.shape[0] > 200000:                 # keep RViz responsive
            cloud = cloud[::max(1, cloud.shape[0] // 200000)]
        self.pub_cloud.publish(self._to_pc2(cloud))

    def _to_pc2(self, arr):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'shaft'
        msg.height = 1
        msg.width = arr.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * arr.shape[0]
        msg.is_dense = True
        msg.data = arr.astype(np.float32).tobytes()
        return msg

    def save(self):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        if self.points:
            cloud = np.concatenate(self.points, axis=0)
            pcd = os.path.join(self.out_dir, f'shaft_cloud_{stamp}.pcd')
            with open(pcd, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n'
                        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                        'TYPE F F F\nCOUNT 1 1 1\n'
                        f'WIDTH {cloud.shape[0]}\nHEIGHT 1\n'
                        'VIEWPOINT 0 0 0 1 0 0 0\n'
                        f'POINTS {cloud.shape[0]}\nDATA ascii\n')
                np.savetxt(f, cloud, fmt='%.4f')
            self.get_logger().info(f'wrote {pcd}  ({cloud.shape[0]} points)')

        if self.profile:
            csv = os.path.join(self.out_dir, f'shaft_profile_{stamp}.csv')
            with open(csv, 'w') as f:
                f.write('depth_m,clearance_m,r_mean_m,r_max_m,roundness\n')
                for row in sorted(self.profile):
                    f.write('{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n'.format(*row))
            self.get_logger().info(f'wrote {csv}  ({len(self.profile)} slices)')


def main(args=None):
    rclpy.init(args=args)
    node = ShaftMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._saved_for_done:
            node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
