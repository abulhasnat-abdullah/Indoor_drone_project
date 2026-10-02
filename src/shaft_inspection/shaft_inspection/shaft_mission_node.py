#!/usr/bin/env python3
"""
shaft_mission_node
==================

Autonomous descent / bottom-detect / ascent for a vertical shaft, flying PX4
Offboard on lidar-derived state alone.

Obstacle avoidance runs in THREE layers, all of them here.  PX4's own
CollisionPrevention is wired only into the ManualPosition flight task, so it
constrains Position-mode sticks but does nothing for the Offboard setpoints
this node publishes.  CP_DIST is still set in the airframe, so a pilot taking
over in Position mode keeps that protection -- the two are complementary, not
redundant.

  Layer 1  CENTRING       servo to the fitted bore axis, so the vehicle never
                          approaches a wall to begin with.  Proactive.
  Layer 2  REPULSION      potential-field push-off from anything close, which
                          catches what the circle fit discards as an outlier:
                          a ledge, a hanging cable, a spalled patch.
  Layer 3  VELOCITY CLAMP no commanded velocity may exceed what can still be
                          braked inside the measured free distance.  This is
                          the hard floor and it runs last, on the final vector.

Plus two gates that are really part of avoidance:
  * descent is inhibited unless the vehicle is centred and settled -- descending
    while off-axis is how you clip a narrowing bore;
  * stale scan data stops lateral motion and descent immediately.

Vertical control uses the RAW downward rangefinder, never EKF2's z, because the
terrain reference jumps when the vehicle leaves the launch platform.
"""
import math
from enum import IntEnum

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from collections import deque

from px4_msgs.msg import (BatteryStatus, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from .px4_topics import px4_qos, px4_timestamp_us, resolve
from .scan_geometry import clamp_velocity_for_obstacles


class St(IntEnum):
    WAIT = 0
    ARM = 1
    TAKEOFF = 2
    CENTER = 3
    DESCEND = 4
    ASCEND = 7
    EXIT = 8
    RETURN = 9
    TOUCHDOWN = 10
    LAND = 11
    DONE = 12
    ABORT = 13


class ShaftMission(Node):

    def __init__(self):
        super().__init__('shaft_mission')

        p = self.declare_parameter
        p('control_hz', 20.0)
        # --- centring ---
        p('kp_lateral', 1.2)
        p('kd_lateral', 0.45)
        p('max_lateral_speed', 0.6)
        p('center_tol', 0.12)
        p('center_settle_speed', 0.15)
        # --- vertical ---
        p('descend_speed', 0.35)
        p('ascend_speed', 0.5)
        p('takeoff_clearance', 1.6)
        p('kp_vertical', 0.6)
        p('max_vertical_speed', 0.8)
        # --- mission limits ---
        p('bottom_range', 1.2)
        p('bottom_confirm_n', 15)
        p('max_depth', 0.0)              # 0 = no depth limit (turnaround, not abort)
        p('dwell_seconds', 6.0)
        p('mission_timeout', 0.0)        # 0 = off; hard ABORT, last resort only
        # ---- return budget: turn around in time to climb back out ----
        # The depth is NOT needed in advance.  The descent continues until the
        # floor is found OR the remaining battery / flight time would no longer
        # cover the climb back to the mouth, whichever comes first.
        p('battery_reserve', 0.30)       # fraction left when back at the mouth
        p('return_margin', 1.5)          # safety factor on the climb estimate
        p('exit_buffer_s', 60.0)         # time for the pilot to fly home and land
        p('drain_window_s', 60.0)        # battery drain-rate estimation window
        p('flight_time_budget_s', 0.0)   # usable endurance from arming; 0 = off
        # --- avoidance ---
        p('stop_dist', 0.35)
        p('critical_dist', 0.30)
        p('brake_delay', 0.4)
        p('brake_decel', 1.0)
        p('n_sectors', 36)
        p('scan_timeout', 0.5)
        p('use_repulsion', True)
        p('auto_descend', True)
        p('descent_min_clearance', 0.60)
        p('center_hold_s', 1.5)
        p('return_tol', 0.12)
        p('touchdown_speed', 0.3)
        p('touchdown_range', 0.30)
        p('start_mode', 'auto_launch')   # auto_launch (sim) | pilot_handover (real)
        p('slowdown_range', 2.5)         # start braking this far above the floor
        # Where the floor range comes from:
        #   laserscan      -> /down_range (Gazebo bridge, simulation)
        #   local_position -> vehicle_local_position.dist_bottom (real vehicle;
        #                     PX4 1.14 does not export distance_sensor over DDS,
        #                     but dist_bottom is EKF2's range-fused height above
        #                     ground and is exported)
        p('range_source', 'laserscan')
        # dry_run: run every state and compute every command, but publish
        # NOTHING to PX4.  For props-off bench tests on the real vehicle.
        p('dry_run', False)        # false = hover-centre test, never descends

        g = lambda k: self.get_parameter(k).value
        self.hz = float(g('control_hz'))
        self.kp = float(g('kp_lateral'))
        self.kd = float(g('kd_lateral'))
        self.v_lat_max = float(g('max_lateral_speed'))
        self.center_tol = float(g('center_tol'))
        self.settle_speed = float(g('center_settle_speed'))
        self.v_desc = float(g('descend_speed'))
        self.v_asc = float(g('ascend_speed'))
        self.takeoff_clr = float(g('takeoff_clearance'))
        self.kp_z = float(g('kp_vertical'))
        self.v_z_max = float(g('max_vertical_speed'))
        self.bottom_range = float(g('bottom_range'))
        self.bottom_n = int(g('bottom_confirm_n'))
        self.max_depth = float(g('max_depth'))
        self.dwell_s = float(g('dwell_seconds'))
        self.timeout_s = float(g('mission_timeout'))
        self.bat_reserve = float(g('battery_reserve'))
        self.ret_margin = float(g('return_margin'))
        self.exit_buffer = float(g('exit_buffer_s'))
        self.drain_window = float(g('drain_window_s'))
        self.time_budget = float(g('flight_time_budget_s'))
        self.stop_dist = float(g('stop_dist'))
        self.crit_dist = float(g('critical_dist'))
        self.brake_delay = float(g('brake_delay'))
        self.brake_decel = float(g('brake_decel'))
        self.n_sectors = int(g('n_sectors'))
        self.scan_timeout = float(g('scan_timeout'))
        self.use_repulsion = bool(g('use_repulsion'))
        self.auto_descend = bool(g('auto_descend'))
        self.descent_min_clr = float(g('descent_min_clearance'))
        self.center_hold_s = float(g('center_hold_s'))
        self._centered_since = None
        self.return_tol = float(g('return_tol'))
        self.td_speed = float(g('touchdown_speed'))
        self.td_range = float(g('touchdown_range'))
        self.launch_off_ned = None
        self.start_mode = str(g('start_mode'))
        self.slowdown_range = float(g('slowdown_range'))
        self.range_source = str(g('range_source'))
        self.dry_run = bool(g('dry_run'))
        if self.dry_run:
            self.get_logger().warn('DRY RUN: nothing will be sent to PX4')

        qos = px4_qos()
        st_topic = resolve(self, '/fmu/out/vehicle_status')
        lp_topic = resolve(self, '/fmu/out/vehicle_local_position')
        self.get_logger().info(f'status topic: {st_topic}')

        self.create_subscription(VehicleStatus, st_topic, self._on_status, qos)
        self.create_subscription(VehicleLocalPosition, lp_topic, self._on_lp, qos)
        self.create_subscription(Vector3Stamped, '/shaft/offset', self._on_offset, 10)
        self.create_subscription(Bool, '/shaft/valid', self._on_valid, 10)
        self.create_subscription(Float32MultiArray, '/shaft/sector_min', self._on_sectors, 10)
        self.create_subscription(Vector3Stamped, '/shaft/repulsion', self._on_repulse, 10)
        if self.range_source == 'laserscan':
            self.create_subscription(LaserScan, '/down_range', self._on_down, 10)

        self.pub_ocm = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_state = self.create_publisher(String, '/shaft/state', 10)
        self.pub_depth = self.create_publisher(Float32, '/shaft/depth', 10)
        # [battery_remaining, battery_needed, time_used_s, time_limit_s, climb_s]
        # (-1 where unavailable)
        self.pub_budget = self.create_publisher(Float32MultiArray, '/shaft/budget', 10)
        bat_topic = resolve(self, '/fmu/out/battery_status', wait_s=3.0)
        self.create_subscription(BatteryStatus, bat_topic, self._on_battery, qos)
        self.bat_hist = deque(maxlen=2000)       # (t, remaining)
        self.bat_remaining = None
        self.t_armed = None
        self.budget_reason = None
        self.pub_cmd_dbg = self.create_publisher(Vector3Stamped, '/shaft/cmd_vel_ned', 10)

        # state
        self.state = St.WAIT
        self.nav_state = None
        self.arming_state = None
        self.offset = np.zeros(2)
        self.offset_ok = False
        self.offset_t = 0.0
        self.sectors = np.full(self.n_sectors, np.inf)
        self.sectors_t = 0.0
        self.repulse = np.zeros(2)
        self.down_r = float('nan')
        self.vel_ned = np.zeros(2)
        self.yaw = 0.0
        self.yaw_lock = None
        self.depth = 0.0
        self.start_z = None          # EKF2 NED z at the top of the bore
        self.z = 0.0
        self.bottom_hits = 0
        self.t_state = self._now()
        self.t_start = self._now()
        self.ocm_count = 0
        self.abort_reason = ''

        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info('shaft_mission up — 3-layer avoidance active (Offboard)')

    # ---------------- helpers ----------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _goto(self, s, why=''):
        if s != self.state:
            self.get_logger().info(
                f'{self.state.name} -> {s.name}' + (f'  ({why})' if why else ''))
            self.state = s
            self.t_state = self._now()

    # ---------------- callbacks ----------------
    def _on_status(self, m):
        armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed and self.t_armed is None:
            self.t_armed = self._now()
        elif not armed:
            self.t_armed = None
        self.nav_state = m.nav_state
        self.arming_state = m.arming_state

    def _on_battery(self, m):
        # PX4 v1.14 does not export battery_status over DDS; see the hardware
        # guide for the one-line firmware change, or use flight_time_budget_s.
        if not m.connected or not (0.0 <= m.remaining <= 1.0):
            return
        self.bat_remaining = float(m.remaining)
        self.bat_hist.append((self._now(), float(m.remaining)))

    def _on_lp(self, m):
        self.z = float(m.z)
        if self.range_source == 'local_position':
            # Accept dist_bottom only when a RANGE sensor backs it; a
            # flow-derived estimate must not decide the turnaround.
            from_range = bool(m.dist_bottom_sensor_bitfield & m.DIST_BOTTOM_SENSOR_RANGE)
            self.down_r = (float(m.dist_bottom)
                           if (m.dist_bottom_valid and from_range) else float('nan'))
        self.vel_ned = np.array([float(m.vx), float(m.vy)])
        self.yaw = float(m.heading)

    def _on_offset(self, m):
        self.offset = np.array([m.vector.x, m.vector.y])
        self.offset_t = self._now()

    def _on_valid(self, m):
        self.offset_ok = bool(m.data)

    def _on_sectors(self, m):
        d = np.array(m.data, dtype=float)
        if d.size == self.n_sectors:
            self.sectors = d
            self.sectors_t = self._now()

    def _on_repulse(self, m):
        self.repulse = np.array([m.vector.x, m.vector.y])

    @staticmethod
    def _first_range(scan: LaserScan):
        if not scan.ranges:
            return float('nan')
        v = scan.ranges[0]
        if not np.isfinite(v) or v <= scan.range_min or v >= scan.range_max:
            return float('nan')
        return float(v)

    def _on_down(self, m):
        self.down_r = self._first_range(m)


    # ---------------- PX4 plumbing ----------------
    def _send_cmd(self, command, p1=0.0, p2=0.0):
        if self.dry_run:
            self.get_logger().info(f'[dry run] would send command {command} ({p1}, {p2})',
                                   throttle_duration_sec=2.0)
            return
        m = VehicleCommand()
        m.timestamp = px4_timestamp_us()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _publish_ocm(self):
        if self.dry_run:
            return
        m = OffboardControlMode()
        m.timestamp = px4_timestamp_us()
        m.position = False
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_ocm.publish(m)

    def _publish_vel(self, vn, ve, vd):
        dbg = Vector3Stamped()
        dbg.vector.x, dbg.vector.y, dbg.vector.z = float(vn), float(ve), float(vd)
        self.pub_cmd_dbg.publish(dbg)
        if self.dry_run:
            return
        m = TrajectorySetpoint()
        m.timestamp = px4_timestamp_us()
        m.position = [float('nan')] * 3
        m.velocity = [float(vn), float(ve), float(vd)]
        m.acceleration = [float('nan')] * 3
        m.jerk = [float('nan')] * 3
        m.yaw = float(self.yaw_lock if self.yaw_lock is not None else self.yaw)
        m.yawspeed = float('nan')
        self.pub_sp.publish(m)

    # ---------------- avoidance ----------------
    def _data_fresh(self):
        t = self._now()
        return ((t - self.offset_t) < self.scan_timeout and
                (t - self.sectors_t) < self.scan_timeout)

    def _min_clearance(self):
        finite = self.sectors[np.isfinite(self.sectors)]
        return float(finite.min()) if finite.size else float('inf')

    def _lateral_velocity(self):
        """Layers 1 + 2, expressed in the LEVEL BODY frame."""
        # Layer 1: centring. offset is displacement FROM the axis, so drive
        # against it. Damping uses measured velocity rotated into level body.
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = self.vel_ned[0] * c + self.vel_ned[1] * s
        vy_b = self.vel_ned[0] * s - self.vel_ned[1] * c

        vx = -self.kp * self.offset[0] - self.kd * vx_b
        vy = -self.kp * self.offset[1] - self.kd * vy_b

        # Layer 2: repulsion from whatever the circle fit threw away.
        if self.use_repulsion:
            vx += self.repulse[0]
            vy += self.repulse[1]

        mag = math.hypot(vx, vy)
        if mag > self.v_lat_max and mag > 1e-9:
            vx *= self.v_lat_max / mag
            vy *= self.v_lat_max / mag
        return vx, vy

    def _apply_clamp(self, vx, vy):
        """Layer 3: never command a speed you cannot brake out of."""
        return clamp_velocity_for_obstacles(
            vx, vy, self.sectors, self.n_sectors,
            stop_dist=self.stop_dist, delay=self.brake_delay,
            decel=self.brake_decel)

    def _level_to_ned(self, vx, vy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return vx * c + vy * s, vx * s - vy * c

    def _centered(self):
        return (self.offset_ok and
                np.linalg.norm(self.offset) < self.center_tol and
                np.linalg.norm(self.vel_ned) < self.settle_speed)

    # ---------------- return budget ----------------
    def _drain_rate(self):
        """Battery fraction used per second, least squares over the window.

        `remaining` is quantised and noisy, so a two-point difference is
        useless; a fit over the last minute of flight is stable."""
        if len(self.bat_hist) < 10:
            return None
        t_now = self._now()
        pts = [(t, r) for t, r in self.bat_hist if t_now - t <= self.drain_window]
        if len(pts) < 10 or pts[-1][0] - pts[0][0] < 0.5 * self.drain_window:
            return None
        t = np.array([p[0] for p in pts]); r = np.array([p[1] for p in pts])
        slope = np.polyfit(t - t[0], r, 1)[0]
        return max(0.0, -float(slope))

    def _return_budget(self):
        """Check whether the vehicle must start climbing now.

        Returns (turn_around, reason, telemetry list)."""
        climb_s = self.depth / max(0.05, self.v_asc) + self.exit_buffer
        need_climb_s = climb_s * self.ret_margin

        have_b, need_b = -1.0, -1.0
        rate = self._drain_rate()
        if self.bat_remaining is not None and rate is not None:
            have_b = self.bat_remaining
            need_b = self.bat_reserve + rate * need_climb_s
        used_s, limit_s = -1.0, -1.0
        if self.time_budget > 0.0 and self.t_armed is not None:
            used_s = self._now() - self.t_armed
            limit_s = self.time_budget

        telemetry = [have_b, need_b, used_s, limit_s, climb_s]
        if need_b >= 0.0 and have_b <= need_b:
            return True, (f'battery {have_b*100:.0f}% left, climb out needs '
                          f'{need_b*100:.0f}% (reserve {self.bat_reserve*100:.0f}%)'), telemetry
        if limit_s > 0.0 and used_s + need_climb_s >= limit_s:
            return True, (f'flight time {used_s:.0f} s of {limit_s:.0f} s, '
                          f'climb out needs {need_climb_s:.0f} s'), telemetry
        if self.max_depth > 0.0 and self.depth >= self.max_depth:
            return True, f'max depth {self.max_depth:.1f} m reached', telemetry
        return False, None, telemetry

    # ---------------- main loop ----------------
    def _tick(self):
        self._publish_ocm()
        self.ocm_count += 1
        self.pub_state.publish(String(data=self.state.name))

        # Depth comes from EKF2's height, which is continuous.  The raw
        # rangefinder is NOT used for it: it jumps by the full shaft depth the
        # moment the vehicle crosses the collar edge, and returns nothing at
        # all while the floor is beyond its range.  The rangefinder's only job
        # is finding the floor.
        if self.start_z is not None:
            self.depth = max(0.0, self.z - self.start_z)
            self.pub_depth.publish(Float32(data=float(self.depth)))

        # ---- global safety checks ----
        # LAND is excluded: landing next to a wall must not bounce back into
        # ABORT, which then climbs, "reaches the mouth", and lands again.
        if self.state not in (St.WAIT, St.ARM, St.TOUCHDOWN, St.LAND, St.DONE, St.ABORT):
            if self.timeout_s > 0.0 and (self._now() - self.t_start) > self.timeout_s:
                self._abort('mission timeout')
            elif self._data_fresh() and self._min_clearance() < self.crit_dist:
                self._abort(f'wall inside critical distance '
                            f'({self._min_clearance():.2f} m)')

        # Return budget: evaluated continuously, acted on while going down.
        turn, why, tel = self._return_budget()
        self.pub_budget.publish(Float32MultiArray(data=[float(v) for v in tel]))
        if turn and self.state in (St.CENTER, St.DESCEND):
            self.get_logger().warn(f'RETURN BUDGET — turning around at depth '
                                   f'{self.depth:.1f} m: {why}')
            self._goto(St.ASCEND, 'return budget')

        if (self.start_mode == 'pilot_handover'
                and self.state not in (St.WAIT, St.DONE)
                and self.nav_state is not None
                and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.get_logger().warn('pilot left Offboard — mission handed back')
            self._goto(St.DONE, 'pilot override')

        handler = {
            St.WAIT: self._s_wait, St.ARM: self._s_arm,
            St.TAKEOFF: self._s_takeoff, St.CENTER: self._s_center,
            St.DESCEND: self._s_descend, St.ASCEND: self._s_ascend,
            St.EXIT: self._s_exit, St.RETURN: self._s_return,
            St.TOUCHDOWN: self._s_touchdown, St.LAND: self._s_land,
            St.DONE: self._s_idle, St.ABORT: self._s_abort,
        }[self.state]
        handler()

    def _abort(self, why):
        if self.state != St.ABORT:
            self.abort_reason = why
            self.get_logger().error(f'ABORT: {why} — ascending blind')
            self._goto(St.ABORT)

    # ---- states ----
    def _s_wait(self):
        self._publish_vel(0.0, 0.0, 0.0)
        if self.start_mode == 'pilot_handover':
            # Real vehicle: the pilot takes off (Position mode, flow-aided),
            # flies over the mouth, and switches to Offboard.  We never arm.
            # The setpoint stream above is what lets PX4 accept the switch.
            armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offboard = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if armed and offboard and self.offset_ok and self._data_fresh():
                self.yaw_lock = self.yaw
                self.get_logger().info('pilot handed over in Offboard — centring')
                self._goto(St.CENTER, 'pilot handover')
            elif armed and offboard:
                self.get_logger().warn('Offboard engaged but no bore fix — holding',
                                       throttle_duration_sec=2.0)
            return

        if self.offset_ok and self._data_fresh() and np.isfinite(self.down_r):
            if self.ocm_count > 20:          # PX4 wants a setpoint stream first
                self.yaw_lock = self.yaw
                self.launch_off_ned = np.array(
                    self._level_to_ned(self.offset[0], self.offset[1]))
                self.get_logger().info(
                    f'launch spot recorded {np.linalg.norm(self.launch_off_ned):.2f} m '
                    f'from the bore centre')
                self.get_logger().info(
                    f'bore fix good, launch range {self.down_r:.2f} m, '
                    f'yaw locked {math.degrees(self.yaw):.1f} deg')
                self._goto(St.ARM)

    def _s_arm(self):
        self._publish_vel(0.0, 0.0, 0.0)
        if (self._now() - self.t_state) > 0.5:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._goto(St.TAKEOFF, 'armed')

    def _s_takeoff(self):
        # Climb straight up off the platform. No lateral motion yet: the
        # platform edge is right there and centring would drag us over it.
        vd = -min(0.4, self.v_z_max)
        self._publish_vel(0.0, 0.0, vd)
        if np.isfinite(self.down_r) and self.down_r > self.takeoff_clr:
            self._goto(St.CENTER, f'clear of platform at {self.down_r:.2f} m')
        elif (self._now() - self.t_state) > 25.0:
            self._abort('takeoff did not gain clearance')

    def _s_center(self):
        if not self._data_fresh():
            self._publish_vel(0.0, 0.0, 0.0)
            return
        vx, vy = self._apply_clamp(*self._lateral_velocity())
        vn, ve = self._level_to_ned(vx, vy)
        # hold height on the raw range while we settle
        vd = 0.0
        self._publish_vel(vn, ve, vd)
        # Require centring to HOLD, not merely be touched: the first pass
        # through the axis is usually mid-overshoot.
        if self._centered():
            if self._centered_since is None:
                self._centered_since = self._now()
        else:
            self._centered_since = None
        held = (self._centered_since is not None and
                (self._now() - self._centered_since) > self.center_hold_s)
        if held:
            # Capture the depth datum HERE, over the open bore -- not at
            # launch, where the rangefinder was reading the platform 5 cm below
            # and the floor was still occluded.
            if self.start_z is None:
                self.start_z = self.z
                floor = (f'{self.down_r:.2f} m below' if np.isfinite(self.down_r)
                         else 'beyond rangefinder range')
                self.get_logger().info(f'depth datum set at the mouth; floor {floor}')
            if self.auto_descend:
                if self.bat_remaining is None and self.time_budget <= 0.0 and self.max_depth <= 0.0:
                    self.get_logger().warn(
                        'no return budget: no battery data, flight_time_budget_s=0, '
                        'max_depth=0 -- descent is limited ONLY by finding the floor')
                self._goto(St.DESCEND, 'centred and settled')

    def _s_descend(self):
        if not self._data_fresh():
            self._publish_vel(0.0, 0.0, 0.0)
            self.get_logger().warn('scan stale — holding', throttle_duration_sec=2.0)
            return

        vx, vy = self._apply_clamp(*self._lateral_velocity())
        vn, ve = self._level_to_ned(vx, vy)

        # Descent gate: only sink while centred, settled and with clearance.
        off = float(np.linalg.norm(self.offset))
        # Hysteresis: sink only while within center_tol; stop sinking beyond
        # 2x.  In between, keep whatever we were doing.
        if off < self.center_tol:
            self._descent_ok = True
        elif off > self.center_tol * 2.0:
            self._descent_ok = False
        gate = (self.offset_ok and getattr(self, '_descent_ok', False) and
                self._min_clearance() > self.descent_min_clr)
        vd = self.v_desc if gate else 0.0
        if not gate:
            self.get_logger().info(
                f'descent gated (offset {off:.2f} m, clearance '
                f'{self._min_clearance():.2f} m)', throttle_duration_sec=3.0)

        # Brake approaching the floor so the turnaround height is not overshot.
        if vd > 0.0 and np.isfinite(self.down_r) and self.down_r < self.slowdown_range:
            span = max(1e-3, self.slowdown_range - self.bottom_range)
            frac = max(0.0, min(1.0, (self.down_r - self.bottom_range) / span))
            vd = max(0.08, self.v_desc * frac)
        self._publish_vel(vn, ve, vd)

        if np.isfinite(self.down_r) and self.down_r < self.bottom_range:
            self.bottom_hits += 1
            if self.bottom_hits >= self.bottom_n:
                # No landing, no dwell: turn straight around.  Touching down in
                # a shaft kicks up dust that blinds the lidar and risks the
                # props on debris.
                self.get_logger().info(
                    f'BOTTOM — turning around {self.down_r:.2f} m above the floor, '
                    f'depth {self.depth:.2f} m')
                self._goto(St.ASCEND, 'floor reached')
        else:
            self.bottom_hits = 0

    def _s_ascend(self):
        if self._data_fresh():
            vx, vy = self._apply_clamp(*self._lateral_velocity())
            vn, ve = self._level_to_ned(vx, vy)
        else:
            vn = ve = 0.0
        self._publish_vel(vn, ve, -self.v_asc)

        # An OPEN shaft gives the up-looking rangefinder nothing to hit, so
        # exit is detected from the sensors that do return: the floor is back
        # at its datum distance, or the bore walls have dropped out of the
        # horizontal scan entirely (we are above the collar).
        near_top = self.start_z is not None and self.z < (self.start_z + 0.3)
        walls_gone = (self._now() - self.offset_t) > 1.5
        if near_top or walls_gone:
            self._goto(St.EXIT, 'back at the mouth' if near_top else 'bore walls lost')

    def _s_exit(self):
        # Climb back to the hover height used for centring before any lateral
        # move, so the props clear the collar ledge.
        if self.start_z is not None and self.z > self.start_z:
            self._publish_vel(0.0, 0.0, -0.3)
            return
        self._publish_vel(0.0, 0.0, 0.0)
        if (self._now() - self.t_state) > 2.0:
            if self.start_mode == 'pilot_handover':
                self._goto(St.DONE, 'hovering at the mouth — hand back to the pilot')
            else:
                self._goto(St.RETURN, 'above the collar')

    def _ned_to_level(self, vn, ve):
        # the level<->NED rotation used here is its own inverse
        return self._level_to_ned(vn, ve)

    def _s_return(self):
        # NEVER use PX4 land mode above the bore: it descends straight down,
        # i.e. back to the bottom of the shaft.  Fly to the recorded launch
        # spot on the ledge first, using the chamber walls for the fix.
        if self.launch_off_ned is None or not self._data_fresh():
            self._publish_vel(0.0, 0.0, 0.0)
            self.get_logger().warn('no fix for return — holding', throttle_duration_sec=2.0)
            return
        cur = np.array(self._level_to_ned(self.offset[0], self.offset[1]))
        err = self.launch_off_ned - cur
        v = self.kp * err
        mag = float(np.linalg.norm(v))
        if mag > self.v_lat_max:
            v *= self.v_lat_max / mag
        vx, vy = self._ned_to_level(float(v[0]), float(v[1]))
        vx, vy = self._apply_clamp(vx, vy)
        vn, ve = self._level_to_ned(vx, vy)
        self._publish_vel(vn, ve, 0.0)
        if np.linalg.norm(err) < self.return_tol and np.linalg.norm(self.vel_ned) < self.settle_speed:
            self._goto(St.TOUCHDOWN, f'over launch spot ({np.linalg.norm(err):.2f} m)')

    def _s_touchdown(self):
        vn = ve = 0.0
        if self._data_fresh() and self.launch_off_ned is not None:
            cur = np.array(self._level_to_ned(self.offset[0], self.offset[1]))
            v = self.kp * (self.launch_off_ned - cur)
            mag = float(np.linalg.norm(v))
            if mag > 0.3:
                v *= 0.3 / mag
            vn, ve = float(v[0]), float(v[1])
        self._publish_vel(vn, ve, self.td_speed)
        if np.isfinite(self.down_r) and self.down_r < self.td_range:
            self._goto(St.LAND, f'on the ledge ({self.down_r:.2f} m)')

    def _s_land(self):
        # By now RETURN has put the vehicle over the launch ledge and the
        # rangefinder has confirmed ground 30 cm below, so PX4's own land mode
        # is safe here (it is NOT safe above the bore).  Pushing an Offboard
        # velocity into the ground does not satisfy PX4's land detector, so
        # the vehicle never registers as landed and refuses to disarm.
        if (self._now() - self.t_state) < 0.2 or int((self._now() - self.t_state) * 2) % 4 == 0:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self._goto(St.DONE, 'landed and disarmed on the launch ledge')
            return
        # Last resort: physically on the ground by the rangefinder but the
        # land detector still disagrees.  Force-disarm only then.
        on_ground = np.isfinite(self.down_r) and self.down_r < 0.22
        if (self._now() - self.t_state) > 15.0 and on_ground:
            self.get_logger().warn('land detector never confirmed — force-disarming on the ledge')
            self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        if (self._now() - self.t_state) > 30.0:
            self._goto(St.DONE, 'disarm timeout — check the vehicle')

    def _s_abort(self):
        # Telemetry-denied: nobody is coming. Climb blind at a modest rate,
        # still honouring the clamp if data is good, then return to launch.
        if self._data_fresh():
            vx, vy = self._apply_clamp(*self._lateral_velocity())
            vn, ve = self._level_to_ned(vx, vy)
        else:
            vn = ve = 0.0
        self._publish_vel(vn, ve, -min(0.4, self.v_asc))
        near_top = self.start_z is not None and self.z < (self.start_z + 0.3)
        if near_top:
            self._goto(St.EXIT, 'abort climb reached the mouth')
        elif self.start_z is None and (self._now() - self.t_state) > 3.0:
            # Aborted before the depth datum existed, i.e. still up at the
            # collar: nothing to climb out of, so go straight home.
            self._goto(St.RETURN, 'abort before descent')

    def _s_idle(self):
        self._publish_vel(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ShaftMission()
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
