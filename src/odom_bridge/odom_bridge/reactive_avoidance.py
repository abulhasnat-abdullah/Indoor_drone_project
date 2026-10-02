#!/usr/bin/env python3
"""
reactive_avoidance.py

Reactive obstacle avoidance for a 2D lidar, applied in the velocity-command
path on the companion computer.

WHY NOT PX4's COLLISION PREVENTION
----------------------------------
PX4's Collision Prevention (the thing /fmu/in/obstacle_distance feeds) only
runs in **Position mode**. In the firmware it lives in FlightTaskManualPosition,
and Offboard mode uses FlightTaskOffboard, which never calls it. This vehicle
flies Offboard -- cmd_vel_teleop_v2.py streams TrajectorySetpoint -- so the
obstacle_distance bridge, however correct, was being published into a feature
that was never going to execute. That is why it "did nothing".

Publishing obstacle_distance is still worth keeping for the case where you fly
Position mode manually, but it cannot protect an Offboard flight. Avoidance for
Offboard has to happen before the setpoint reaches PX4 -- which is here.

WHERE THIS SITS
---------------
    nav2  ──/cmd_vel──────────────┐
                                  ├──> [reactive_avoidance] ──> /cmd_vel_safe
    web_teleop ──/offboard_velocity_cmd──┘                 └──> /offboard_velocity_cmd_safe
                                                                      │
                                            cmd_vel_teleop_v2.py <────┘
                                              (launched with -r remaps)

No existing script is modified. cmd_vel_teleop_v2.py calls rclpy.init(args=None),
so it honours standard ROS remapping:

    python3 ~/cmd_vel_teleop_v2.py --ros-args \\
        -r /cmd_vel:=/cmd_vel_safe \\
        -r /offboard_velocity_cmd:=/offboard_velocity_cmd_safe

HOW IT DECIDES
--------------
Everything is in the body FLU frame, where the scan's 0 rad and cmd_vel's +x
are both "forward", so no transform is needed.

For a candidate travel direction phi, the vehicle sweeps a corridor of width
2*(robot_radius + margin). A scan point at (r, theta) obstructs that corridor
when its perpendicular offset |r*sin(theta-phi)| is inside the corridor and it
is ahead of us (r*cos(theta-phi) > 0); the distance at which we would hit it is
then r*cos(theta-phi). The minimum over all such points is the clearance in
that direction. This is computed for every candidate direction at once as one
numpy array.

Clearance becomes a speed limit through a braking curve,
v_allow = sqrt(2*a*(clearance - stop_distance)), i.e. the fastest you may go and
still stop in the space available. Then:

  * if the commanded direction already allows the commanded speed, the command
    passes through untouched -- this node is invisible in open space;
  * otherwise it searches within +-max_steer for the direction maximising
    progress along the original heading, which is what makes it slide along a
    wall instead of stopping dead in front of it;
  * and the speed is capped by that direction's braking limit.

Repulsion (actively pushing away from obstacles when you commanded nothing) is
OFF by default: injecting motion the operator did not ask for fights the
offboard controller's position hold and is surprising in flight. Set
repulsion_gain > 0 if you want it.

FAILS SAFE
----------
No scan, or a scan older than scan_timeout, publishes zero velocity. A stale
lidar must not mean "fly blind at full speed".
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ReactiveAvoidance(Node):

    def __init__(self):
        super().__init__('reactive_avoidance')

        # --- geometry ----------------------------------------------------
        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('safety_margin', 0.15)
        # Below this, brake to a full stop regardless of direction.
        self.declare_parameter('stop_distance', 0.45)
        # Returns closer than this are CLAMPED to it, not discarded -- they
        # are treated as an obstacle at this range. Discarding them (the
        # obvious way to reject prop/frame self-returns) also makes a genuinely
        # close obstacle invisible, and the vehicle then drives into it at full
        # speed because nothing is reported in the way. Fail toward "something
        # is there". Use blind_sectors_deg for parts of your own airframe --
        # those sit at fixed angles, so angle is the right way to reject them,
        # not range.
        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 8.0)
        # Angles permanently ignored, as [start_deg, end_deg, ...] pairs in the
        # body frame -- use for a mast, antenna or arm that the lidar can see.
        self.declare_parameter('blind_sectors_deg', [0.0, 0.0])

        # --- dynamics ----------------------------------------------------
        self.declare_parameter('max_decel', 1.0)        # m/s^2, braking curve
        # Dead time between deciding to brake and the airframe actually
        # decelerating: offboard publishes at 20 Hz, PX4's controller and the
        # airframe's own inertia add more. Without this the vehicle commits to
        # a speed it cannot shed in the distance available, and brakes far too
        # late. At 1 m/s, 0.4 s is 0.4 m of travel before anything happens.
        self.declare_parameter('reaction_time', 0.4)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steer_deg', 70.0)   # how far it may deviate
        self.declare_parameter('sector_deg', 5.0)       # candidate resolution
        self.declare_parameter('repulsion_gain', 0.0)   # 0 = off (see docstring)
        # Small bonus for last cycle's choice, to stop it dithering between two
        # equally good ways around the same obstacle.
        self.declare_parameter('hysteresis', 0.10)

        # --- plumbing ----------------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('enabled', True)
        self.declare_parameter('status_hz', 2.0)

        gp = lambda n: self.get_parameter(n).value
        self.R = float(gp('robot_radius'))
        self.margin = float(gp('safety_margin'))
        self.stop_d = float(gp('stop_distance'))
        self.rmin = float(gp('range_min'))
        self.rmax = float(gp('range_max'))
        self.decel = float(gp('max_decel'))
        self.t_react = float(gp('reaction_time'))
        self.vmax = float(gp('max_speed'))
        self.max_steer = math.radians(float(gp('max_steer_deg')))
        self.repulsion = float(gp('repulsion_gain'))
        self.hysteresis = float(gp('hysteresis'))
        self.scan_timeout = float(gp('scan_timeout'))
        self.enabled = bool(gp('enabled'))

        blind = list(gp('blind_sectors_deg'))
        self.blind = [(math.radians(blind[i]), math.radians(blind[i + 1]))
                      for i in range(0, len(blind) - 1, 2)
                      if blind[i] != blind[i + 1]]

        # Candidate travel directions, -pi..pi.
        step = math.radians(float(gp('sector_deg')))
        n = max(8, int(round(2 * math.pi / step)))
        self.phis = np.linspace(-math.pi, math.pi, n, endpoint=False)
        self.corridor = self.R + self.margin

        # --- state -------------------------------------------------------
        self.scan_pts = None          # (r, theta) of valid returns
        self.scan_stamp = 0.0
        self.clearance = None         # per-candidate-direction, metres
        self.last_phi = 0.0
        self.n_limited = 0
        self.n_steered = 0
        self.n_stopped = 0
        self.last_reason = 'idle'

        # --- I/O ---------------------------------------------------------
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, gp('scan_topic'),
                                 self._scan_cb, scan_qos)

        # QoS deliberately mirrors cmd_vel_teleop_v2.py's two subscribers, so
        # the remapped topics match without any edit on that side.
        nav_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        self.nav_pub = self.create_publisher(Twist, '/cmd_vel_safe', nav_qos)
        self.man_pub = self.create_publisher(
            Twist, '/offboard_velocity_cmd_safe', px4_qos)
        self.create_subscription(Twist, '/cmd_vel',
                                 lambda m: self._filter(m, self.nav_pub), nav_qos)
        self.create_subscription(Twist, '/offboard_velocity_cmd',
                                 lambda m: self._filter(m, self.man_pub), px4_qos)

        self.status_pub = self.create_publisher(String, '/avoidance/status', 10)
        self.create_service(SetBool, '~/enable', self._enable_cb)
        hz = float(gp('status_hz'))
        if hz > 0:
            self.create_timer(1.0 / hz, self._publish_status)

        self.get_logger().info(
            f'reactive_avoidance up: radius {self.R:.2f}+{self.margin:.2f} m, '
            f'stop at {self.stop_d:.2f} m, brake {self.decel:.1f} m/s^2 '
            f'after {self.t_react:.2f} s reaction, '
            f'steer up to {math.degrees(self.max_steer):.0f} deg, '
            f'{len(self.phis)} candidate directions, '
            f'returns under {self.rmin:.2f} m clamped (not ignored). '
            f'Repulsion {"ON" if self.repulsion > 0 else "OFF"}. '
            f'{"ENABLED" if self.enabled else "BYPASSED (passthrough)"}.')

    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
        if r.size == 0:
            return
        th = (msg.angle_min +
              np.arange(r.size, dtype=np.float32) * msg.angle_increment)

        # Only the sensor's own floor marks "no return"; our range_min is a
        # clamp, applied below, not a filter.
        hi = min(self.rmax, msg.range_max)
        ok = np.isfinite(r) & (r > max(1e-3, msg.range_min)) & (r < hi)
        for a, b in self.blind:
            wrapped = np.arctan2(np.sin(th), np.cos(th))
            ok &= ~((wrapped >= a) & (wrapped <= b))
        if not np.any(ok):
            # A completely empty scan is legitimate (open space), but it is
            # also what a dead lidar looks like. Treat it as open and let
            # scan_timeout catch an actually dead sensor.
            self.scan_pts = (np.empty(0, np.float32), np.empty(0, np.float32))
            self.clearance = np.full(self.phis.size, np.inf, dtype=np.float32)
            self.scan_stamp = time.time()
            return

        r = np.maximum(r[ok], self.rmin)   # too-close reads become close obstacles
        th = th[ok]
        # Subsampling caps the cost of the (directions x points) array. 0.5 deg
        # native resolution is far finer than the vehicle's own width needs.
        if r.size > 400:
            k = int(np.ceil(r.size / 400))
            r, th = r[::k], th[::k]
        self.scan_pts = (r, th)
        self.clearance = self._clearances(r, th)
        self.scan_stamp = time.time()

    def _clearances(self, r, th):
        """Distance-to-collision for every candidate direction, vectorised.

        For direction phi, a point at (r, theta) lies in the swept corridor
        when its lateral offset from the travel line is inside the corridor
        half-width and it is in front of us; the collision distance is then its
        along-track component.
        """
        d = th[None, :] - self.phis[:, None]          # (dirs, points)
        along = r[None, :] * np.cos(d)
        perp = np.abs(r[None, :] * np.sin(d))
        blocking = (perp < self.corridor) & (along > 0.0)
        return np.min(np.where(blocking, along, np.inf), axis=1)

    # ------------------------------------------------------------------
    def _allowed_speed(self, clearance):
        """Fastest speed from which we can still stop inside `clearance`.

        Solves  d = v*t_react + v^2/(2a)  for v, i.e. the vehicle coasts for
        t_react before braking begins and then decelerates at a. Ignoring the
        coast term makes the limit far too optimistic at exactly the moment it
        matters.
        """
        usable = np.maximum(0.0, clearance - self.stop_d)
        at = self.decel * self.t_react
        v = -at + np.sqrt(at * at + 2.0 * self.decel * usable)
        return np.minimum(self.vmax, v)

    def _filter(self, msg: Twist, pub):
        out = Twist()
        out.angular.z = msg.angular.z          # yaw is never restricted: a 360
                                               # deg lidar means turning in place
                                               # cannot drive us into anything.
        if not self.enabled:
            self.last_reason = 'bypassed'
            pub.publish(msg)
            return

        age = time.time() - self.scan_stamp
        if self.clearance is None or age > self.scan_timeout:
            self.last_reason = f'NO SCAN ({age:.1f}s old) - stopped'
            self.get_logger().warn(self.last_reason, throttle_duration_sec=2.0)
            pub.publish(out)               # zero xy, yaw preserved
            return

        vx, vy = float(msg.linear.x), float(msg.linear.y)
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            self.last_reason = 'idle'
            if self.repulsion > 0.0:
                rx, ry = self._repulsion_vector()
                out.linear.x, out.linear.y = rx, ry
            pub.publish(out)
            return

        phi_d = math.atan2(vy, vx)
        v_allow = self._allowed_speed(self.clearance)

        i_d = int(np.argmin(np.abs(self._wrap(self.phis - phi_d))))
        if v_allow[i_d] >= speed - 1e-3:
            # Open ahead: pass the command through untouched.
            self.last_reason = f'clear ({self.clearance[i_d]:.1f} m)'
            self.last_phi = phi_d
            out.linear.x, out.linear.y = vx, vy
            pub.publish(out)
            return

        # Blocked: pick the reachable direction that makes the most progress
        # along the direction actually asked for.
        dev = np.abs(self._wrap(self.phis - phi_d))
        # Viability is decided on the braking limit alone. The hysteresis bonus
        # is a tie-breaker between directions that are *already* usable -- if it
        # were added before this test it could make a direction with zero
        # allowed speed look like forward progress.
        viable = (dev <= self.max_steer) & (v_allow > 0.05)
        if not np.any(viable):
            self.n_stopped += 1
            self.last_reason = f'BLOCKED ({self.clearance[i_d]:.2f} m) - stopped'
            if self.repulsion > 0.0:
                out.linear.x, out.linear.y = self._repulsion_vector()
            pub.publish(out)
            return

        progress = np.where(viable, v_allow * np.cos(self._wrap(self.phis - phi_d)), -1.0)
        if self.hysteresis > 0.0:
            near_last = np.abs(self._wrap(self.phis - self.last_phi)) < math.radians(10.0)
            progress = progress + (near_last & viable) * self.hysteresis
        best = int(np.argmax(progress))

        if progress[best] <= 0.0:
            self.n_stopped += 1
            self.last_reason = f'BLOCKED ({self.clearance[i_d]:.2f} m) - stopped'
            if self.repulsion > 0.0:
                out.linear.x, out.linear.y = self._repulsion_vector()
            pub.publish(out)
            return

        phi = float(self.phis[best])
        v = float(min(speed, v_allow[best]))
        self.last_phi = phi
        if abs(self._wrap(phi - phi_d)) > math.radians(2.0):
            self.n_steered += 1
            self.last_reason = (f'steering {math.degrees(self._wrap(phi - phi_d)):+.0f} deg, '
                                f'{v:.2f}/{speed:.2f} m/s')
        else:
            self.n_limited += 1
            self.last_reason = (f'braking {v:.2f}/{speed:.2f} m/s '
                                f'({self.clearance[i_d]:.2f} m ahead)')

        out.linear.x = v * math.cos(phi)
        out.linear.y = v * math.sin(phi)
        pub.publish(out)

    def _repulsion_vector(self):
        """Push away from anything inside stop_distance. Off by default."""
        if self.scan_pts is None:
            return 0.0, 0.0
        r, th = self.scan_pts
        close = r < self.stop_d
        if not np.any(close):
            return 0.0, 0.0
        w = (self.stop_d - r[close]) / self.stop_d
        fx = -np.sum(w * np.cos(th[close]))
        fy = -np.sum(w * np.sin(th[close]))
        n = math.hypot(fx, fy)
        if n < 1e-6:
            return 0.0, 0.0
        mag = min(self.vmax, self.repulsion * n / close.size * 10.0)
        return float(mag * fx / n), float(mag * fy / n)

    @staticmethod
    def _wrap(a):
        return np.arctan2(np.sin(a), np.cos(a))

    # ------------------------------------------------------------------
    def _publish_status(self):
        age = time.time() - self.scan_stamp
        c = self.clearance
        ahead = '-'
        if c is not None and np.isfinite(c).any():
            i = int(np.argmin(np.abs(self.phis)))
            ahead = f'{c[i]:.2f}' if np.isfinite(c[i]) else 'inf'
        msg = String()
        msg.data = (f'state={self.last_reason}|enabled={self.enabled}|'
                    f'scan_age={age:.2f}|fwd_clear={ahead}|'
                    f'braked={self.n_limited}|steered={self.n_steered}|'
                    f'stopped={self.n_stopped}')
        self.status_pub.publish(msg)

    def _enable_cb(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = ('avoidance ENABLED' if self.enabled
                            else 'avoidance BYPASSED - commands pass through unfiltered')
        self.get_logger().warn(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidance()
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
