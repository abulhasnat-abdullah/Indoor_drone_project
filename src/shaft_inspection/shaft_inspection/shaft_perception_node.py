#!/usr/bin/env python3
"""
shaft_perception_node
=====================

Turns a single horizontal 2D lidar into everything the shaft mission needs:

  * where the shaft axis is relative to the vehicle (robust circle fit)
  * an absolute horizontal position fix, injected into EKF2 as external vision
  * per-sector clearances and a repulsion vector for obstacle avoidance

Centring is SHAPE-AGNOSTIC.  Real bores are round, rectangular, D-shaped or
just ragged, and the cross-section changes with depth, so nothing here assumes
a circle.  The centre is the point of maximum clearance to the wall, with ties
settled by the cross-section's area centroid (see scan_geometry).  A circle fit
is still run, but only as a DESCRIPTOR - it reports a radius when the bore
happens to be round, and is ignored for control when it is not.

Why external vision rather than optical flow: the shaft wall is a FIXED
reference.  Measuring displacement from the bore centre involves no
integration, so it does not drift the way flow does.  Flow also needs light and
texture, which an underground shaft does not supply.

Caveat worth knowing: for a bore whose cross-section CHANGES SHAPE with depth,
the centre itself moves, so the fix is only as absolute as the bore is
prismatic.  position_variance is set accordingly rather than optimistically.

Yaw note: a symmetric bore yields no yaw information, so we publish position
only (EKF2_EV_CTRL bit 0).  Yaw drift rotates the published fix by
|offset| * drift, which vanishes as the vehicle centres.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOdometry

from .px4_topics import px4_qos, px4_timestamp_us, resolve
from .scan_geometry import (center_max_clearance, cross_section_metrics,
                            detilt_points, fit_circle_robust,
                            repulsion_vector, sector_min_ranges)


def quat_to_rpy(q):
    """PX4 quaternion (w, x, y, z) -> roll, pitch, yaw."""
    w, x, y, z = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class ShaftPerception(Node):

    def __init__(self):
        super().__init__('shaft_perception')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('n_sectors', 36)
        self.declare_parameter('inlier_tol', 0.25)
        self.declare_parameter('min_inliers', 40)
        self.declare_parameter('max_fit_rms', 0.15)
        self.declare_parameter('min_clearance_valid', 0.35)
        self.declare_parameter('max_clearance_valid', 8.0)
        self.declare_parameter('min_coverage', 0.85)
        self.declare_parameter('search_window', 1.2)
        self.declare_parameter('roundness_for_radius', 0.93)
        self.declare_parameter('repulse_influence', 0.8)
        self.declare_parameter('repulse_gain', 0.35)
        self.declare_parameter('repulse_max', 0.6)
        self.declare_parameter('anchor_samples', 30)
        self.declare_parameter('publish_ev', True)
        # Returns closer than this are the airframe itself: lidar mount, GPS
        # mast, prop tips.  On the real vehicle, without this every scan
        # contains a "wall" a few cm away and the mission aborts on the ground.
        self.declare_parameter('self_filter_radius', 0.25)
        # Mounting.  The code needs scan angle 0 = vehicle FORWARD, CCW positive.
        # A real RPLIDAR's 0 deg is wherever the housing points, so measure it:
        # a wrong offset rotates every centring command and the vehicle
        # spirals into the wall instead of centring.  Use tools/lidar_mount_check.
        self.declare_parameter('lidar_yaw_offset_deg', 0.0)
        self.declare_parameter('lidar_upside_down', False)

        gp = self.get_parameter
        self.n_sectors = int(gp('n_sectors').value)
        self.inlier_tol = float(gp('inlier_tol').value)
        self.min_inliers = int(gp('min_inliers').value)
        self.max_fit_rms = float(gp('max_fit_rms').value)
        self.clr_min = float(gp('min_clearance_valid').value)
        self.clr_max = float(gp('max_clearance_valid').value)
        self.min_cov = float(gp('min_coverage').value)
        self.search_w = float(gp('search_window').value)
        self.round_thresh = float(gp('roundness_for_radius').value)
        self.rep_inf = float(gp('repulse_influence').value)
        self.rep_gain = float(gp('repulse_gain').value)
        self.rep_max = float(gp('repulse_max').value)
        self.anchor_samples = int(gp('anchor_samples').value)
        self.publish_ev = bool(gp('publish_ev').value)
        self.self_filter = float(gp('self_filter_radius').value)
        self.yaw_off = math.radians(float(gp('lidar_yaw_offset_deg').value))
        self.upside_down = bool(gp('lidar_upside_down').value)

        qos = px4_qos()
        att_topic = resolve(self, '/fmu/out/vehicle_attitude')
        lp_topic = resolve(self, '/fmu/out/vehicle_local_position')
        self.get_logger().info(f'attitude topic: {att_topic}')
        self.get_logger().info(f'local position topic: {lp_topic}')

        self.create_subscription(VehicleAttitude, att_topic, self._on_att, qos)
        self.create_subscription(VehicleLocalPosition, lp_topic, self._on_lp, qos)
        self.create_subscription(
            LaserScan, gp('scan_topic').value, self._on_scan, 10)

        self.pub_offset = self.create_publisher(Vector3Stamped, '/shaft/offset', 10)
        self.pub_radius = self.create_publisher(Float32, '/shaft/radius', 10)
        self.pub_clear = self.create_publisher(Float32, '/shaft/clearance', 10)
        self.pub_round = self.create_publisher(Float32, '/shaft/roundness', 10)
        self.pub_valid = self.create_publisher(Bool, '/shaft/valid', 10)
        self.pub_sectors = self.create_publisher(Float32MultiArray, '/shaft/sector_min', 10)
        self.pub_repulse = self.create_publisher(Vector3Stamped, '/shaft/repulsion', 10)
        self.pub_ev = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos)

        self.roll = self.pitch = self.yaw = 0.0
        self.have_att = False
        self.lp_xy = None          # EKF local NED position
        self.lp_z = 0.0
        self.lp_valid = False

        # Shaft axis expressed in the EKF's own local NED frame.  Anchored once
        # from the first good fits so the published fix agrees with EKF2 at
        # startup (no position jump) while staying absolute thereafter.
        self.axis_ned = None
        self._anchor_acc = []

        self.get_logger().info('shaft_perception up (lidar-only, no optical flow)')

    # ---------------- callbacks ----------------
    def _on_att(self, msg: VehicleAttitude):
        self.roll, self.pitch, self.yaw = quat_to_rpy(msg.q)
        self.have_att = True

    def _on_lp(self, msg: VehicleLocalPosition):
        self.lp_xy = (float(msg.x), float(msg.y))
        self.lp_z = float(msg.z)
        self.lp_valid = bool(msg.xy_valid) or bool(msg.v_xy_valid)

    def _on_scan(self, scan: LaserScan):
        if not self.have_att:
            return

        ranges, a_min, a_inc = scan.ranges, scan.angle_min, scan.angle_increment
        if self.upside_down:
            # mirrored about the forward axis: angles flip sign
            ranges = list(reversed(scan.ranges))
            a_min = -(scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1))
        pts = detilt_points(ranges, a_min + self.yaw_off, a_inc,
                            self.roll, self.pitch,
                            max(scan.range_min, self.self_filter, 1e-3),
                            scan.range_max)

        stamp = self.get_clock().now().to_msg()

        sectors = sector_min_ranges(pts, self.n_sectors)
        sm = Float32MultiArray()
        sm.data = [float(v) if np.isfinite(v) else float('inf') for v in sectors]
        self.pub_sectors.publish(sm)

        rvx, rvy = repulsion_vector(pts, self.rep_inf, 0.25,
                                    self.rep_gain, self.rep_max)
        rv = Vector3Stamped()
        rv.header.stamp = stamp
        rv.header.frame_id = 'base_link_level'
        rv.vector.x, rv.vector.y = rvx, rvy
        self.pub_repulse.publish(rv)

        # ---- shape-agnostic centre (this drives control) ----
        c = center_max_clearance(pts, search=self.search_w,
                                 min_coverage=self.min_cov)
        ok = (c['ok'] and self.clr_min < c['clearance'] < self.clr_max)
        self.pub_valid.publish(Bool(data=bool(ok)))
        if not ok:
            self.get_logger().warn(
                f"no usable bore fix (coverage {c.get('coverage', 0.0):.2f}, "
                f"clearance {c.get('clearance', 0.0):.2f} m)",
                throttle_duration_sec=2.0)
            return

        self.pub_clear.publish(Float32(data=float(c['clearance'])))

        met = cross_section_metrics(pts, c['cx'], c['cy'])
        self.pub_round.publish(Float32(data=float(met['roundness'])))

        # Circle fit is a descriptor, not a controller input: only meaningful
        # when this slice of the bore is actually round.
        if met['roundness'] >= self.round_thresh:
            fit = fit_circle_robust(pts, inlier_tol=self.inlier_tol,
                                    min_inliers=self.min_inliers)
            if fit['ok'] and fit['rms'] < self.max_fit_rms:
                self.pub_radius.publish(Float32(data=float(fit['r'])))

        # (cx, cy) points from the vehicle TO the bore centre, so the vehicle's
        # displacement from that centre is its negation.
        off_x = -c['cx']
        off_y = -c['cy']
        off = Vector3Stamped()
        off.header.stamp = stamp
        off.header.frame_id = 'base_link_level'
        off.vector.x, off.vector.y = off_x, off_y
        self.pub_offset.publish(off)

        if self.publish_ev:
            self._publish_ev(off_x, off_y)

    # ---------------- external vision ----------------
    def _offset_to_ned(self, off_x, off_y):
        """Level-body FLU offset -> NED offset, using the current yaw."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        north = off_x * c + off_y * s
        east = off_x * s - off_y * c
        return north, east

    def _publish_ev(self, off_x, off_y):
        north, east = self._offset_to_ned(off_x, off_y)

        if self.axis_ned is None:
            if self.lp_xy is None:
                return
            # axis = current EKF position - displacement from axis
            self._anchor_acc.append((self.lp_xy[0] - north, self.lp_xy[1] - east))
            if len(self._anchor_acc) < self.anchor_samples:
                return
            arr = np.array(self._anchor_acc)
            self.axis_ned = (float(np.median(arr[:, 0])), float(np.median(arr[:, 1])))
            self.get_logger().info(
                f'shaft axis anchored at EKF local NED '
                f'({self.axis_ned[0]:.2f}, {self.axis_ned[1]:.2f})')

        msg = VehicleOdometry()
        now_us = px4_timestamp_us()
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(self.axis_ned[0] + north),
                        float(self.axis_ned[1] + east),
                        # EKF2 drops the WHOLE sample unless all three
                        # components are finite, so z cannot be NaN.  Echo
                        # EKF2's own height: EKF2_EV_CTRL=1 fuses horizontal
                        # position only, so this value is never used.
                        float(self.lp_z)]
        msg.q = [float('nan')] * 4             # a round bore carries no yaw
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.velocity = [float('nan')] * 3
        msg.angular_velocity = [float('nan')] * 3
        # Deliberately loose: the bore centre is only a rigid reference to the
        # extent the shaft is prismatic (see module docstring).
        msg.position_variance = [0.09, 0.09, 0.0]
        msg.orientation_variance = [0.0, 0.0, 0.0]
        msg.velocity_variance = [0.0, 0.0, 0.0]
        msg.reset_counter = 0
        msg.quality = 0
        self.pub_ev.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShaftPerception()
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
