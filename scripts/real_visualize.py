#!/usr/bin/env python3
"""
REAL HARDWARE VERSION (Pixhawk 6C + Raspberry Pi 5 + RPLIDAR C1)

Bridges PX4's /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, FRD/NED frame)
into nav_msgs/Odometry (ENU/base_link) + an odom->base_link->link TF chain,
so everything lines up in RViz2 for real-world SLAM.

DIFFERENCES FROM THE SIM VERSION:
  - use_sim_time defaults to False and should stay False. There is no Gazebo
    /clock on real hardware — system wall-clock is the shared time base for
    PX4 (via MicroXRCEAgent), the RPLIDAR driver, this node, and RViz2.
    Do NOT pass --ros-args -p use_sim_time:=true when running this on the
    real drone; if you do, TF lookups for /scan will fail exactly like the
    odom->base_link mismatch you saw in sim.
  - lidar_offset_xyz / lidar_offset_rpy default to 0 as PLACEHOLDERS. On the
    real airframe these MUST be measured by hand (ruler/calipers) from the
    Pixhawk's IMU/base_link origin to the RPLIDAR C1's rotation axis, then
    passed in via launch params. The sim SDF numbers (0.12, 0, 0.26) applied
    only to the virtual x500_gps_denied model and do NOT carry over.
  - lidar_link_name stays 'link' to match your RPLIDAR C1 launch, which you
    should start with `frame_id:=link` (see sllidar_c1_launch.py args), so
    /scan's frame_id lines up with the transform this node publishes without
    any further changes needed on either side.
  - The scan itself is NOT bridged here. It comes directly from the
    sllidar_ros2 driver node on the Pi, publishing /scan natively — there is
    no ros_gz_bridge step on real hardware.

TF root is still 'odom', no 'map' frame published here. Fixed Frame in RViz2
= 'odom'. slam_toolbox will publish map->odom once you launch it.

Sensor pose parameters are declared, NOT hardcoded — pass your measured
offsets at launch, e.g.:
  ros2 run <pkg> px4_odom_bridge_hardware.py --ros-args \\
    -p lidar_offset_xyz:="[0.08, 0.0, 0.05]" \\
    -p lidar_offset_rpy:="[0.0, 0.0, 0.0]"
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


def frd_to_enu_pos(x, y, z):
    # PX4 NED -> ROS ENU: (x,y,z)_NED -> (y,x,-z)_ENU relative to local origin
    return (y, x, -z)


def frd_to_enu_quat(qw, qx, qy, qz):
    # Rotate NED->ENU frame (simple axis swap approximation, good enough for visualization)
    return (qx, qy, -qz, qw)  # ROS order is (x,y,z,w)


class PX4OdomBridgeHW(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')

        # ── use_sim_time is auto-declared by every rclpy Node — do NOT
        # re-declare it. On real hardware this MUST stay False (the default).
        # Do not launch this node with -p use_sim_time:=true here.

        # ── LiDAR mount offset — MEASURE ON YOUR REAL AIRFRAME ────────────
        # These are placeholders (0,0,0). Replace via launch params with the
        # actual measured offset from base_link (Pixhawk) to the RPLIDAR C1's
        # rotation axis.
        self.declare_parameter('lidar_link_name', 'link')
        self.declare_parameter('lidar_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('lidar_offset_rpy', [0.0, 0.0, 0.0])

        lidar_link = self.get_parameter('lidar_link_name').value
        lidar_xyz  = self.get_parameter('lidar_offset_xyz').value
        lidar_rpy  = self.get_parameter('lidar_offset_rpy').value

        self.lidar_link = lidar_link
        self.lidar_xyz  = lidar_xyz
        self.lidar_rpy  = lidar_rpy

        if list(lidar_xyz) == [0.0, 0.0, 0.0]:
            self.get_logger().warn(
                'lidar_offset_xyz is still [0,0,0] (placeholder). '
                'Measure the real mount offset from base_link to the RPLIDAR '
                'C1 rotation axis and pass it via -p lidar_offset_xyz:="[x,y,z]" '
                'for accurate SLAM/mapping results.')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── PX4 odometry subscriber (via MicroXRCEAgent, over serial/UDP) ─
        self.sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ── TF broadcasters ──────────────────────────────────────────────
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish the fixed base_link -> link transform once. This is what
        # makes /scan, published directly by the sllidar_ros2 driver node
        # on the Pi (frame_id:=link), resolve correctly against base_link
        # and odom in RViz2 / slam_toolbox.
        self._publish_static_lidar_tf()

        self.get_logger().info(
            f'PX4 odom bridge (HARDWARE) started — publishing /odom, '
            f'TF odom->base_link, and static base_link->{self.lidar_link} '
            f'(offset xyz={self.lidar_xyz}, rpy={self.lidar_rpy}). '
            f'use_sim_time=False, TF root is "odom", no map frame published. '
            f'Scan comes directly from sllidar_ros2 on /scan.')

    # ──────────────────────────────────────────────────────────────────
    def _publish_static_lidar_tf(self):
        x, y, z = self.lidar_xyz
        roll, pitch, yaw = self.lidar_rpy
        qx, qy, qz, qw = self._rpy_to_quat(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = self.lidar_link
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

    # ──────────────────────────────────────────────────────────────────
    def odom_cb(self, msg: VehicleOdometry):
        if any(math.isnan(v) for v in (msg.position[0], msg.position[1], msg.position[2])):
            return  # estimator not yet initialized (no GPS/vision aiding), skip this sample

        now = self.get_clock().now().to_msg()

        ex, ey, ez = frd_to_enu_pos(
            float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
        qx, qy, qz, qw = frd_to_enu_quat(
            float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3]))

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = ex
        odom.pose.pose.position.y = ey
        odom.pose.pose.position.z = ez
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        vx, vy, vz = frd_to_enu_pos(
            float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2]))
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = ex
        t.transform.translation.y = ey
        t.transform.translation.z = ez
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = PX4OdomBridgeHW()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()