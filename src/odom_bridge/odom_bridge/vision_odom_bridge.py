#!/usr/bin/env python3
"""
vision_odom_bridge.py

Reads the map -> base_link transform (composed by tf2 from slam_toolbox's
map->odom correction + rf2o's odom->base_link odometry) and republishes it
as a px4_msgs/VehicleOdometry message on /fmu/in/vehicle_visual_odometry,
so PX4's EKF2 can fuse it as an external vision aiding source.

Frame conversion is the INVERSE of odom_converter.py's NED/FRD -> ENU/FLU
transform. Both world_transform and body_transform used there are
involutory (M @ M == identity), so the same two matrices convert back.

pose_frame is set to POSE_FRAME_FRD, not POSE_FRAME_NED: slam_toolbox's
map frame is anchored to wherever the vehicle started, with no absolute
heading/North reference, which is exactly what PX4's FRD pose frame means
for external vision/mocap systems (locked to initial heading, not rotating
with true North).

NOTE: verify px4_msgs/msg/VehicleOdometry field names on your installed
px4_msgs version with:  ros2 interface show px4_msgs/msg/VehicleOdometry
Field names/enums have shifted across PX4 releases.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from px4_msgs.msg import VehicleOdometry
import numpy as np


class VisionOdomBridge(Node):

    POSE_FRAME_FRD = 2  # per px4_msgs VehicleOdometry.msg pose_frame enum

    def __init__(self):
        super().__init__('vision_odom_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.map_frame  = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos)

        # Same involutory transform matrices as odom_converter.py
        self.world_transform = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ])
        self.body_transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ])

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_vision_odom)
        self._last_warn_time = 0.0

        self.get_logger().info(
            f'vision_odom_bridge started — reading {self.map_frame}->'
            f'{self.base_frame} TF, publishing to /fmu/in/vehicle_visual_odometry '
            f'at {rate_hz} Hz.')

    def publish_vision_odom(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Don't spam — slam_toolbox/rf2o may not have published yet at startup
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_warn_time > 5.0:
                self.get_logger().warn(f'TF not yet available ({self.map_frame}->'
                                        f'{self.base_frame}): {e}')
                self._last_warn_time = now
            return

        # --- Position: ENU -> PX4 local frame ---
        p = tf.transform.translation
        pos_enu = np.array([p.x, p.y, p.z])
        pos_px4 = self.world_transform @ pos_enu

        # --- Orientation: FLU -> FRD ---
        q = tf.transform.rotation  # ROS order x,y,z,w
        R_enu_flu = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        # Inverse of world_transform @ R_ned_frd @ body_transform, and since
        # both are involutory: R_ned_frd = world_transform @ R_enu_flu @ body_transform
        R_ned_frd = self.world_transform @ R_enu_flu @ self.body_transform
        qx, qy, qz, qw = self._matrix_to_quat(R_ned_frd)

        msg = VehicleOdometry()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = msg.timestamp
        msg.pose_frame = self.POSE_FRAME_FRD
        msg.position = [float(pos_px4[0]), float(pos_px4[1]), float(pos_px4[2])]
        msg.q = [float(qw), float(qx), float(qy), float(qz)]  # PX4 order w,x,y,z

        # No velocity estimate from pose alone — mark as NaN so EKF2 only
        # fuses position/orientation, not velocity, from this source
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN if hasattr(
            VehicleOdometry, 'VELOCITY_FRAME_UNKNOWN') else 0
        msg.velocity = [float('nan')] * 3
        msg.angular_velocity = [float('nan')] * 3

        msg.position_variance = [0.02, 0.02, 0.02]
        msg.orientation_variance = [0.01, 0.01, 0.01]
        msg.velocity_variance = [float('nan')] * 3

        msg.quality = 0  # 0 = unknown/not reported, per px4_msgs default

        self.pub.publish(msg)

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def _matrix_to_quat(Rm):
        tr = np.trace(Rm)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (Rm[2, 1] - Rm[1, 2]) / S
            qy = (Rm[0, 2] - Rm[2, 0]) / S
            qz = (Rm[1, 0] - Rm[0, 1]) / S
        elif Rm[0, 0] > Rm[1, 1] and Rm[0, 0] > Rm[2, 2]:
            S = np.sqrt(1.0 + Rm[0, 0] - Rm[1, 1] - Rm[2, 2]) * 2
            qw = (Rm[2, 1] - Rm[1, 2]) / S
            qx = 0.25 * S
            qy = (Rm[0, 1] + Rm[1, 0]) / S
            qz = (Rm[0, 2] + Rm[2, 0]) / S
        elif Rm[1, 1] > Rm[2, 2]:
            S = np.sqrt(1.0 + Rm[1, 1] - Rm[0, 0] - Rm[2, 2]) * 2
            qw = (Rm[0, 2] - Rm[2, 0]) / S
            qx = (Rm[0, 1] + Rm[1, 0]) / S
            qy = 0.25 * S
            qz = (Rm[1, 2] + Rm[2, 1]) / S
        else:
            S = np.sqrt(1.0 + Rm[2, 2] - Rm[0, 0] - Rm[1, 1]) * 2
            qw = (Rm[1, 0] - Rm[0, 1]) / S
            qx = (Rm[0, 2] + Rm[2, 0]) / S
            qy = (Rm[1, 2] + Rm[2, 1]) / S
            qz = 0.25 * S
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = VisionOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
