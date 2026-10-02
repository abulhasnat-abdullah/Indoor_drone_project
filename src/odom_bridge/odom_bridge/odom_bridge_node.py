#!/usr/bin/env python3
"""
odom_bridge_node.py

Bridges PX4's /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, NED/FRD)
into nav_msgs/Odometry (ENU/FLU) on /odom, and publishes the static
base_link -> lidar_sensor_link TF.

This merges the two prior scripts:
  - odom_converter.py's proper rotation-matrix frame conversion (handles
    roll/pitch correctly, not just yaw), velocity_frame handling, angular
    velocity, and covariance — kept as-is, it was the more correct of the
    two.
  - px4_odom_bridge.py's static LiDAR TF publish.

IMPORTANT — dynamic odom->base_link TF:
  If you are running rf2o_laser_odometry + slam_toolbox (see the
  rf2o/slam/EKF2-vision guide), rf2o owns the odom->base_link TF edge.
  Leave `publish_dynamic_tf` at its default of False so this node doesn't
  fight rf2o for that transform. Set it True only if you are NOT running
  rf2o and want this node's own PX4-derived TF instead (e.g. for a quick
  RViz check before the SLAM chain is wired up).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomBridge(Node):

    def __init__(self):
        super().__init__('odom_bridge_node')

        # ── Params ──────────────────────────────────────────────────────
        self.declare_parameter('publish_dynamic_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_link_name', 'laser')
        self.declare_parameter('lidar_offset_xyz', [0.0, 0.0, 0.07])
        self.declare_parameter('lidar_offset_rpy', [0.0, 0.0, 0.0])

        self.publish_dynamic_tf = self.get_parameter('publish_dynamic_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_link = self.get_parameter('lidar_link_name').value
        self.lidar_xyz = self.get_parameter('lidar_offset_xyz').value
        self.lidar_rpy = self.get_parameter('lidar_offset_rpy').value

        # ── Coordinate transforms (involutory: M @ M == identity) ─────────
        # NED -> ENU
        self.world_transform = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ])
        # FRD -> FLU
        self.body_transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ])

        # ── PX4 odometry subscriber ────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.listener_callback, qos)

        self.publisher = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_lidar_tf()

        self.get_logger().info(
            f'odom_bridge_node started — publishing /odom. '
            f'Dynamic {self.odom_frame}->{self.base_frame} TF: '
            f'{"ON (this node)" if self.publish_dynamic_tf else "OFF (expecting rf2o or another source to own it)"}. '
            f'Static {self.base_frame}->{self.lidar_link} TF published.')

    # ──────────────────────────────────────────────────────────────────
    def _publish_static_lidar_tf(self):
        x, y, z = self.lidar_xyz
        roll, pitch, yaw = self.lidar_rpy
        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()  # x,y,z,w

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_link
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.static_tf_broadcaster.sendTransform(t)

    # ──────────────────────────────────────────────────────────────────
    def create_covariance_matrix(self, diagonal_values):
        cov = np.zeros((6, 6))
        np.fill_diagonal(cov, diagonal_values)
        return cov.flatten().tolist()

    def transform_vector(self, vec, matrix):
        return matrix @ vec

    def transform_orientation(self, px4_quaternion):
        # px4_quaternion is (w, x, y, z) -> scipy wants (x, y, z, w)
        q_scipy = [px4_quaternion[1], px4_quaternion[2], px4_quaternion[3], px4_quaternion[0]]
        R_ned_frd = R.from_quat(q_scipy).as_matrix()
        R_enu_flu = self.world_transform @ R_ned_frd @ self.body_transform
        q_enu = R.from_matrix(R_enu_flu).as_quat()  # x,y,z,w
        if q_enu[3] < 0:
            q_enu = [-c for c in q_enu]
        return q_enu

    # ──────────────────────────────────────────────────────────────────
    def listener_callback(self, msg: VehicleOdometry):
        if any(math.isnan(v) for v in (msg.position[0], msg.position[1], msg.position[2])):
            return  # estimator not yet initialized

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        position_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        position_enu = self.transform_vector(position_ned, self.world_transform)
        odom.pose.pose.position.x = float(position_enu[0])
        odom.pose.pose.position.y = float(position_enu[1])
        odom.pose.pose.position.z = float(position_enu[2])

        q_enu = self.transform_orientation(msg.q)
        odom.pose.pose.orientation.x = float(q_enu[0])
        odom.pose.pose.orientation.y = float(q_enu[1])
        odom.pose.pose.orientation.z = float(q_enu[2])
        odom.pose.pose.orientation.w = float(q_enu[3])

        velocity_ned = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        if msg.velocity_frame == 1:      # VELOCITY_FRAME_NED
            velocity_enu = self.transform_vector(velocity_ned, self.world_transform)
        elif msg.velocity_frame == 3:    # VELOCITY_FRAME_BODY_FRD
            velocity_enu = self.transform_vector(velocity_ned, self.body_transform)
        else:
            velocity_enu = velocity_ned
        odom.twist.twist.linear.x = float(velocity_enu[0])
        odom.twist.twist.linear.y = float(velocity_enu[1])
        odom.twist.twist.linear.z = float(velocity_enu[2])

        av_frd = np.array(msg.angular_velocity)
        av_flu = self.transform_vector(av_frd, self.body_transform)
        odom.twist.twist.angular.x = float(av_flu[0])
        odom.twist.twist.angular.y = float(av_flu[1])
        odom.twist.twist.angular.z = float(av_flu[2])

        odom.pose.covariance = self.create_covariance_matrix([0.01] * 6)
        odom.twist.covariance = self.create_covariance_matrix([0.01] * 6)

        self.publisher.publish(odom)

        if self.publish_dynamic_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
