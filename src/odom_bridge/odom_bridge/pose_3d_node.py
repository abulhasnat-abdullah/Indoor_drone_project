#!/usr/bin/env python3
"""
pose_3d_node.py

Gives RViz a real 3D pose for the drone.

Why this node exists
--------------------
slam_toolbox is a 2D SLAM system. The map->odom correction it publishes, and
therefore the composed map->base_link transform, is planar: x, y and yaw only.
That is why RViz shows the drone sliding around at z = 0 no matter how high it
actually is.

PX4 already knows the missing pieces. This node takes:

    x, y, yaw      from the SLAM chain  (map -> base_link, via tf2)
    z              from PX4             (/fmu/out/vehicle_local_position)
    roll, pitch    from PX4             (/fmu/out/vehicle_attitude)

and publishes the combination as a separate frame, `base_link_3d`, parented
directly to `map`. It is a separate frame on purpose -- `base_link` already has
`odom` as its parent and a frame may only have one parent, so overwriting it
would corrupt the TF tree that rf2o, slam_toolbox and Nav2 all depend on.
Nothing in the 2D navigation stack sees this frame; it exists for
visualisation and for scan_3d_mapper.

Outputs
-------
    TF            map -> base_link_3d
    PoseStamped   /drone/pose_3d
    Path          /drone/path_3d      (trail, capped)
    Marker        /drone/marker_3d    (body + altitude drop line)

Altitude source (`altitude_source` parameter):
    "odometry"      -- -vehicle_odometry.position[2], height above the EKF's
                       local origin. Drifts with the estimator but is smooth
                       and continuous. Default.
    "dist_bottom"   -- the HFlow rangefinder's AGL reading. Absolute and crisp
                       over flat floors, but it steps when you fly over
                       furniture and goes invalid out of range. Costs an extra
                       subscription (see the CPU note below).

A note on CPU, because this matters on a Pi
-------------------------------------------
rclpy pays a real per-message cost in Python for every message it delivers,
whatever the callback does with it. PX4 publishes vehicle_attitude at ~170 Hz
and vehicle_local_position at ~110 Hz; subscribing to both costs roughly a
fifth of a core before this node does any work at all. So it subscribes to
exactly one topic, /fmu/out/vehicle_odometry (~110 Hz), which carries the
altitude *and* the attitude quaternion together, and converts them only at the
publish rate. Selecting altitude_source "dist_bottom" adds the second
subscription back and roughly doubles this node's CPU -- worth knowing if you
are already tight.

This node is visualisation-only. Nothing in the flight or navigation stack
depends on it, so it is always safe to leave off when you need the headroom.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from std_msgs.msg import ColorRGBA
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformBroadcaster, TransformListener)
from visualization_msgs.msg import Marker

# NED -> ENU and FRD -> FLU. Both are involutory (M @ M == I), matching the
# convention already used by odom_bridge_node.py and vision_odom_bridge.py.
WORLD_T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
BODY_T = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


class Pose3DNode(Node):

    def __init__(self):
        super().__init__('pose_3d_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_frame', 'base_link_3d')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('altitude_source', 'odometry')
        self.declare_parameter('path_max_poses', 600)
        # Only extend the trail once the drone has actually moved this far --
        # keeps the Path message small while hovering.
        self.declare_parameter('path_min_step_m', 0.05)
        self.declare_parameter('publish_marker', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.out_frame = self.get_parameter('output_frame').value
        self.alt_source = self.get_parameter('altitude_source').value
        self.path_max = int(self.get_parameter('path_max_poses').value)
        self.path_step = float(self.get_parameter('path_min_step_m').value)
        self.want_marker = bool(self.get_parameter('publish_marker').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        # One subscription carries both the altitude and the attitude.
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self._odometry_cb, px4_qos)
        # Only pay for the second topic if the rangefinder was actually asked for.
        if self.alt_source == 'dist_bottom':
            self.create_subscription(VehicleLocalPosition,
                                     '/fmu/out/vehicle_local_position',
                                     self._local_position_cb, px4_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose_3d', 10)
        self.path_pub = self.create_publisher(Path, '/drone/path_3d', 1)
        self.marker_pub = self.create_publisher(Marker, '/drone/marker_3d', 1)

        self.altitude = 0.0
        self.altitude_valid = False
        self._q_att = None
        self.have_attitude = False
        self._agl_stamp = None

        self.path = Path()
        self.path.header.frame_id = self.map_frame

        self._last_warn = 0.0
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'pose_3d_node up: {self.map_frame}->{self.out_frame} at {rate} Hz, '
            f'altitude from "{self.alt_source}".')

    # ------------------------------------------------------------------
    def _odometry_cb(self, msg: VehicleOdometry):
        z = msg.position[2]
        if not math.isnan(z):
            # dist_bottom, when selected and valid, wins; this is the fallback
            # so the drone never freezes at its last good rangefinder reading.
            if self.alt_source != 'dist_bottom' or not self._agl_fresh():
                self.altitude = float(-z)      # NED down -> ENU up
                self.altitude_valid = True
        self._attitude_cb(msg)

    def _agl_fresh(self) -> bool:
        return (self._agl_stamp is not None
                and time.monotonic() - self._agl_stamp < 0.5)

    def _local_position_cb(self, msg: VehicleLocalPosition):
        if msg.dist_bottom_valid and not math.isnan(msg.dist_bottom):
            self.altitude = float(msg.dist_bottom)
            self.altitude_valid = True
            self._agl_stamp = time.monotonic()

    def _attitude_cb(self, msg):
        # Stash the raw quaternion and nothing more. Converting it here, at
        # ~110 Hz, would throw away 80% of the work; _tick() converts the one
        # sample it actually needs, at the publish rate.
        self._q_att = msg.q
        self.have_attitude = True

    def _roll_pitch(self):
        q = self._q_att  # PX4 order: w, x, y, z (NED/FRD)
        if q is None or any(math.isnan(v) for v in q):
            return 0.0, 0.0
        R_ned_frd = R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        R_enu_flu = WORLD_T @ R_ned_frd @ BODY_T
        roll, pitch, _ = R.from_matrix(R_enu_flu).as_euler('xyz')
        return float(roll), float(pitch)

    # ------------------------------------------------------------------
    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                 rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_warn > 5.0:
                self.get_logger().warn(
                    f'no {self.map_frame}->{self.base_frame} TF yet ({e}). '
                    f'Is the SLAM stack running?')
                self._last_warn = now
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        # SLAM's yaw is the only rotational component we trust from the 2D
        # chain; roll/pitch there are identically zero.
        _, _, yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')

        roll, pitch = self._roll_pitch() if self.have_attitude else (0.0, 0.0)
        z = self.altitude if self.altitude_valid else 0.0
        qx, qy, qz, qw = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        stamp = self.get_clock().now().to_msg()

        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.out_frame
        out.transform.translation.x = float(t.x)
        out.transform.translation.y = float(t.y)
        out.transform.translation.z = float(z)
        out.transform.rotation.x = float(qx)
        out.transform.rotation.y = float(qy)
        out.transform.rotation.z = float(qz)
        out.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(out)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(t.x)
        pose.pose.position.y = float(t.y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        self.pose_pub.publish(pose)

        self._extend_path(pose)
        if self.want_marker:
            self._publish_marker(pose)

    def _extend_path(self, pose: PoseStamped):
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            d = math.dist((last.x, last.y, last.z),
                          (pose.pose.position.x, pose.pose.position.y,
                           pose.pose.position.z))
            if d < self.path_step:
                return
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_max:
            del self.path.poses[0]
        self.path.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path)

    def _publish_marker(self, pose: PoseStamped):
        """A body box plus a vertical line down to z=0, which is what actually
        makes altitude readable in a 3D view."""
        m = Marker()
        m.header = pose.header
        m.ns = 'drone'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.8)
        p = pose.pose.position
        m.points = [Point(x=p.x, y=p.y, z=p.z), Point(x=p.x, y=p.y, z=0.0)]
        m.pose.orientation.w = 1.0
        self.marker_pub.publish(m)

        body = Marker()
        body.header = pose.header
        body.ns = 'drone'
        body.id = 1
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose = pose.pose
        body.scale.x, body.scale.y, body.scale.z = 0.35, 0.35, 0.10
        body.color = ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.95)
        self.marker_pub.publish(body)


def main(args=None):
    rclpy.init(args=args)
    node = Pose3DNode()
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
