#!/usr/bin/env python3
"""
Bridges PX4's /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, FRD/NED frame)
into nav_msgs/Odometry (ENU/base_link) + an odom->base_link->lidar_sensor_link
TF chain, so everything lines up in RViz2.

NOTE: No 'map' frame is published here. The TF root is 'odom'. Set RViz2's
Fixed Frame to 'odom'. If you later add SLAM, slam_toolbox will publish
map->odom itself and 'map' will appear naturally at that point — don't
publish a placeholder for it before then.

On startup this node also spawns the Gazebo clock bridge and LiDAR scan
bridge as ros_gz_bridge parameter_bridge subprocesses (previously these
were separate entries in the launch file). It does not subscribe to or
republish the scan itself — it only publishes the static
base_link -> lidar_sensor_link transform so /scan's frame_id resolves
correctly in RViz2's TF tree.

Sensor poses below are taken directly from x500_gps_denied.sdf:
  2D LiDAR    : xyz = 0.12  0  0.26   rpy = 0 0 0   (relative to base_link)
  Optical flow: xyz = 0.101 0 -0.05   rpy = 0 0 0   (relative to base_link)
  Rangefinder : xyz = 0.101 0 -0.05   rpy = 0 1.5707 0 (relative to base_link)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import subprocess
import atexit


# ── Set this to your actual Gazebo LiDAR topic ───────────────────────────
# Confirm once the sim is running with:  gz topic -l | grep -i lidar
GZ_LIDAR_TOPIC = '/world/hospital/model/x500_gps_denied_0/link/link/sensor/lidar_2d_v2/scan'


def frd_to_enu_pos(x, y, z):
    # PX4 NED -> ROS ENU: (x,y,z)_NED -> (y,x,-z)_ENU relative to local origin
    return (y, x, -z)

def frd_to_enu_quat(qw, qx, qy, qz):
    # Rotate NED->ENU frame (simple axis swap approximation, good enough for visualization)
    return (qx, qy, -qz, qw)  # ROS order is (x,y,z,w)


class PX4OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')

        # ── use_sim_time is auto-declared by every rclpy Node — do NOT
        # re-declare it (that throws ParameterAlreadyDeclaredException).
        # It defaults to False; override at launch with:
        #   --ros-args -p use_sim_time:=true

        # ── LiDAR mount offset — from x500_gps_denied.sdf ────────────────
        # <include ...lidar_2d_v2 .../><pose>.12 0 .26 0 0 0</pose>
        self.declare_parameter('lidar_link_name', 'link')
        self.declare_parameter('lidar_offset_xyz', [0.12, 0.0, 0.26])
        self.declare_parameter('lidar_offset_rpy', [0.0, 0.0, 0.0])

        lidar_link = self.get_parameter('lidar_link_name').value
        lidar_xyz  = self.get_parameter('lidar_offset_xyz').value
        lidar_rpy  = self.get_parameter('lidar_offset_rpy').value

        self.lidar_link = lidar_link
        self.lidar_xyz  = lidar_xyz
        self.lidar_rpy  = lidar_rpy

        # ── Spawn the Gazebo clock bridge + LiDAR scan bridge ─────────────
        # (previously separate Node entries in the launch file — now
        # started as subprocesses from here so this script is self-contained)
        self._bridge_procs = []
        self._start_gz_bridges()
        atexit.register(self._stop_gz_bridges)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── PX4 odometry subscriber ──────────────────────────────────────
        self.sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ── TF broadcasters ──────────────────────────────────────────────
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish the fixed base_link -> lidar_sensor_link transform once
        # (this is what makes /scan, published by the gz_lidar_bridge
        # subprocess spawned above, resolve correctly against base_link/odom)
        self._publish_static_lidar_tf()

        self.get_logger().info(
            f'PX4 odom bridge started — publishing /odom, TF odom->base_link, '
            f'and static base_link->{self.lidar_link} '
            f'(offset xyz={self.lidar_xyz}, rpy={self.lidar_rpy}). '
            f'TF root is "odom" — no map frame published. '
            f'Gazebo clock bridge and LiDAR scan bridge spawned as subprocesses.')

    # ──────────────────────────────────────────────────────────────────
    def _start_gz_bridges(self):
        # Clock bridge: gz -> ROS 2 /clock
        clock_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args', '-p', 'use_sim_time:=true',
        ]
        # LiDAR scan bridge: gz LaserScan -> ROS 2 /scan
        lidar_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{GZ_LIDAR_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-r', f'{GZ_LIDAR_TOPIC}:=/scan',
        ]

        for name, cmd in (('gz_clock_bridge', clock_cmd), ('gz_lidar_bridge', lidar_cmd)):
            try:
                proc = subprocess.Popen(cmd)
                self._bridge_procs.append(proc)
                self.get_logger().info(f'Spawned {name} (pid={proc.pid})')
            except Exception as e:
                self.get_logger().error(f'Failed to spawn {name}: {e}')

    def _stop_gz_bridges(self):
        for proc in self._bridge_procs:
            if proc.poll() is None:
                proc.terminate()

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
            return  # estimator not yet initialized, skip this sample

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
    node = PX4OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()