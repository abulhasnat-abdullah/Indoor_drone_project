#!/usr/bin/env python3

import time, math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleAttitude
)
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SETPOINT_RATE_HZ = 50.0
SETPOINT_WARMUP_COUNT = 30
INACTIVITY_TIMEOUT = 5.0  # seconds
VEL_LIMIT_XY = 3.0
VEL_LIMIT_Z = 2.0
YAW_RATE_LIMIT = 2.0


class PX4Offboard(Node):
    def __init__(self):
        super().__init__('px4_offboard_teleop')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.ctrl_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos)
        self.create_subscription(Twist, '/offboard_velocity_cmd', self.vel_cb, qos)
        self.create_subscription(Bool, '/arm_message', self.arm_cb, qos)

        # Internal state
        self.vx = self.vy = self.vz = self.yawspeed = 0.0
        self.yaw = 0.0  # Current yaw from vehicle
        self.target_yaw = 0.0  # Desired yaw setpoint
        self.use_yaw_position = False  # Flag to switch between rate and position control
        self.status = VehicleStatus()
        self.offboard_active = False
        self.armed = False
        self.warmed_up = False
        self.last_cmd_time = time.time()
        self.last_yaw_update = time.time()

        # Timer
        self.create_timer(1.0 / SETPOINT_RATE_HZ, self.loop)
        self.get_logger().info("PX4 Offboard Teleop Node initialized.")

    # -------------------- Callbacks --------------------
    def status_cb(self, msg):
        self.status = msg

    def att_cb(self, msg):
        q = msg.q
        if len(q) == 4:
            w, x, y, z = q
            self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def vel_cb(self, msg: Twist):
        """Handle velocity commands from teleop."""
        self.last_cmd_time = time.time()

        self.vx = max(-VEL_LIMIT_XY, min(VEL_LIMIT_XY, msg.linear.x))
        self.vy = max(-VEL_LIMIT_XY, min(VEL_LIMIT_XY, msg.linear.y))
        self.vz = max(-VEL_LIMIT_Z, min(VEL_LIMIT_Z, msg.linear.z))
        self.yawspeed = max(-YAW_RATE_LIMIT, min(YAW_RATE_LIMIT, msg.angular.z))

        # Start offboard mode on first velocity command (if armed)
        if not self.warmed_up and self.armed:
            self.start_offboard()

    def arm_cb(self, msg: Bool):
        """Handle external arm/disarm command"""
        if msg.data and not self.armed:
            self.get_logger().info("ARM command received")
            self.start_offboard()  # Prepare offboard mode
            time.sleep(0.2)
            self.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.armed = True
            self.get_logger().info("ARMED - Ready to fly!")
            
        elif not msg.data and self.armed:
            self.get_logger().info("DISARM command received")
            self.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.armed = False
            self.warmed_up = False
            self.offboard_active = False
            self.vx = self.vy = self.vz = self.yawspeed = 0.0
            self.get_logger().info("DISARMED")

    # -------------------- Helpers --------------------
    def micros(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.micros()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.ctrl_mode_pub.publish(msg)

    def publish_setpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = self.micros()
        msg.position = [float('nan')] * 3
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')  # We're using yawspeed, not absolute yaw
        msg.yawspeed = yawspeed
        self.setpoint_pub.publish(msg)

    def publish_cmd(self, cmd, **params):
        m = VehicleCommand()
        m.timestamp = self.micros()
        m.command = cmd
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        for k in ["param1","param2","param3","param4","param5","param6","param7"]:
            setattr(m, k, params.get(k, 0.0))
        self.cmd_pub.publish(m)

    # -------------------- Sequence Control --------------------
    def start_offboard(self):
        """Send dummy setpoints before activating OFFBOARD."""
        if self.warmed_up:
            return
        self.get_logger().info("Warming up Offboard mode...")
        for _ in range(SETPOINT_WARMUP_COUNT):
            self.publish_mode()
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(1.0 / SETPOINT_RATE_HZ)
        
        self.get_logger().info("Requesting OFFBOARD mode...")
        self.publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.warmed_up = True
        time.sleep(0.2)
        self.get_logger().info("Offboard mode ready!")

    # -------------------- Main Loop --------------------
    def loop(self):
        now = time.time()
        self.publish_mode()

        # # Check if PX4 reports Offboard mode
        # if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.offboard_active = True

        # # If not yet Offboard or not armed, hold still
        # if not self.offboard_active or not self.armed:
        #     self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
        #     return
        if not self.armed:
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # Inactivity safety timeout - hover if no commands
        if (now - self.last_cmd_time) > INACTIVITY_TIMEOUT:
            self.get_logger().warn("No teleop input - entering hover failsafe...", throttle_duration_sec=2.0)
            self.vx = self.vy = self.vz = self.yawspeed = 0.0

        # Rotate body frame (FLU) → world frame (NED coordinate system)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        wx = self.vx * cy - self.vy * sy
        wy = self.vx * sy + self.vy * cy
        wz = self.vz
        
        self.publish_setpoint(wx, wy, wz, self.yawspeed)

        # Debug logging
        if abs(self.yawspeed) > 0.01:
            self.get_logger().info(
                f"ROTATING: yaw_rate={self.yawspeed:.3f} rad/s | current_yaw={math.degrees(self.yaw):.1f}°",
                throttle_duration_sec=0.3,
            )
        else:
            self.get_logger().info(
                f"Setpoint: vx={wx:.2f}, vy={wy:.2f}, vz={wz:.2f}",
                throttle_duration_sec=0.5,
            )


def main(args=None):
    rclpy.init(args=args)
    node = PX4Offboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PX4 Offboard Teleop node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()