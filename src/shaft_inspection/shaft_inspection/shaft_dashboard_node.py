#!/usr/bin/env python3
"""
shaft_dashboard
===============

Read-only web dashboard for the shaft stack: who is flying (pilot / mission /
failsafe), joystick input, PX4 mode and estimator health, the bore as the lidar
sees it, mission progress, and an event timeline.

Open http://<host>:8080 from any browser on the same network (on the vehicle:
the Pi's address).  Nothing on the page can command the vehicle -- by design,
a browser tab must never be able to change a flight.
"""
import asyncio
import json
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from px4_msgs.msg import (FailsafeFlags, ManualControlSetpoint, VehicleAttitude,
                          VehicleLocalPosition, VehicleStatus)

from .px4_topics import px4_qos, resolve

from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

NAV_NAMES = {0: 'MANUAL', 1: 'ALTITUDE', 2: 'POSITION', 3: 'MISSION', 4: 'HOLD', 5: 'RETURN',
             6: 'POSITION SLOW', 10: 'ACRO', 12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD',
             15: 'STABILIZED', 17: 'TAKEOFF', 18: 'LAND', 19: 'FOLLOW', 20: 'PRECISION LAND', 21: 'ORBIT'}
ARM_NAMES = {1: 'DISARMED', 2: 'ARMED'}
MISSION_STATES = ['WAIT', 'ARM', 'TAKEOFF', 'CENTER', 'DESCEND', 'ASCEND', 'EXIT',
                  'RETURN', 'TOUCHDOWN', 'LAND', 'DONE', 'ABORT']
WATCHED_NODES = ('shaft_mission', 'shaft_perception', 'shaft_mapper')


def clean(o):
    """NaN/inf -> None, recursively.  PX4 uses NaN for 'invalid' and strict JSON
    has no NaN, so one invalid dist_bottom would otherwise kill the stream."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    return o


def quat_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Dashboard(Node):

    def __init__(self):
        super().__init__('shaft_dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('scan_bins', 180)
        self.port = int(self.get_parameter('port').value)
        self.host = str(self.get_parameter('host').value)
        self.bins = int(self.get_parameter('scan_bins').value)

        self.lock = threading.Lock()
        self.t_last = {}                       # topic -> wall time last seen
        self.rate = {}                         # topic -> deque of arrival times
        self.s = dict(nav=None, nav_user=None, arming=None, failsafe=False,
                      fs_flags=[], mc=None, lp=None, att=None, mission='—',
                      depth=None, offset=None, clearance=None, roundness=None,
                      valid=None, cmd=None, sector_min=None, scan=None, radius=None, budget=None)
        self.events = deque(maxlen=200)
        self.hist = deque(maxlen=1200)         # (t, depth, clearance, dist_bottom)
        self.t0 = time.time()
        self._prev_nav = None
        self._last_sticks_moving = 0.0

        q = px4_qos()
        self.create_subscription(VehicleStatus, resolve(self, '/fmu/out/vehicle_status'), self._on_status, q)
        self.create_subscription(VehicleLocalPosition, resolve(self, '/fmu/out/vehicle_local_position'), self._on_lp, q)
        self.create_subscription(VehicleAttitude, resolve(self, '/fmu/out/vehicle_attitude'), self._on_att, q)
        self.create_subscription(ManualControlSetpoint, resolve(self, '/fmu/out/manual_control_setpoint'), self._on_mc, q)
        self.create_subscription(FailsafeFlags, resolve(self, '/fmu/out/failsafe_flags'), self._on_fs, q)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/shaft/state', self._on_mission, 10)
        self.create_subscription(Float32, '/shaft/depth', lambda m: self._set('depth', m.data, '/shaft/depth'), 10)
        self.create_subscription(Float32, '/shaft/clearance', lambda m: self._set('clearance', m.data, '/shaft/clearance'), 10)
        self.create_subscription(Float32, '/shaft/roundness', lambda m: self._set('roundness', m.data, None), 10)
        self.create_subscription(Float32, '/shaft/radius', lambda m: self._set('radius', m.data, None), 10)
        self.create_subscription(Bool, '/shaft/valid', lambda m: self._set('valid', m.data, '/shaft/valid'), 10)
        self.create_subscription(Vector3Stamped, '/shaft/offset',
                                 lambda m: self._set('offset', [m.vector.x, m.vector.y], None), 10)
        self.create_subscription(Vector3Stamped, '/shaft/cmd_vel_ned',
                                 lambda m: self._set('cmd', [m.vector.x, m.vector.y, m.vector.z], None), 10)
        self.create_subscription(Float32MultiArray, '/shaft/sector_min',
                                 lambda m: self._set('sector_min', [None if not math.isfinite(v) else round(v, 3) for v in m.data], None), 10)
        self.create_subscription(Float32MultiArray, '/shaft/budget',
                                 lambda m: self._set('budget', list(m.data), None), 10)
        rosout_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Log, '/rosout', self._on_rosout, rosout_qos)

        self.create_timer(0.5, self._sample_history)
        self._event('info', 'dashboard', f'dashboard up on port {self.port}')

    # ---------------- helpers ----------------
    def _seen(self, key):
        now = time.time()
        self.t_last[key] = now
        d = self.rate.setdefault(key, deque(maxlen=60))
        d.append(now)

    def _hz(self, key):
        d = self.rate.get(key)
        if not d or len(d) < 2 or time.time() - d[-1] > 2.0:
            return 0.0
        span = d[-1] - d[0]
        return (len(d) - 1) / span if span > 0 else 0.0

    def _set(self, k, v, topic):
        with self.lock:
            self.s[k] = v
            if topic:
                self._seen(topic)

    def _event(self, level, source, text):
        with self.lock:
            self.events.appendleft(dict(t=round(time.time() - self.t0, 1),
                                        wall=time.strftime('%H:%M:%S'),
                                        level=level, source=source, text=text))

    # ---------------- callbacks ----------------
    def _on_status(self, m):
        with self.lock:
            self._seen('status')
            self.s['nav'] = int(m.nav_state)
            self.s['nav_user'] = int(m.nav_state_user_intention)
            self.s['arming'] = int(m.arming_state)
            self.s['failsafe'] = bool(m.failsafe)
        nav = int(m.nav_state)
        if self._prev_nav is not None and nav != self._prev_nav:
            a, b = NAV_NAMES.get(self._prev_nav, self._prev_nav), NAV_NAMES.get(nav, nav)
            why = ''
            if self._prev_nav == 14 and nav in (1, 2) and time.time() - self._last_sticks_moving < 2.0:
                why = ' — pilot took over with the sticks'
            elif nav == 14:
                why = ' — control handed to the mission'
            elif m.failsafe:
                why = ' — FAILSAFE'
            level = 'warn' if (m.failsafe or 'took over' in why) else 'mode'
            self._event(level, 'PX4', f'{a} → {b}{why}')
        self._prev_nav = nav

    def _on_lp(self, m):
        with self.lock:
            self._seen('local_position')
            self.s['lp'] = dict(x=m.x, y=m.y, z=m.z, vx=m.vx, vy=m.vy, vz=m.vz,
                                heading=m.heading, xy_valid=bool(m.xy_valid), z_valid=bool(m.z_valid),
                                v_xy_valid=bool(m.v_xy_valid),
                                dist_bottom=m.dist_bottom, dist_bottom_valid=bool(m.dist_bottom_valid),
                                dist_bottom_range=bool(m.dist_bottom_sensor_bitfield & 1))

    def _on_att(self, m):
        r, p, y = quat_rpy(m.q)
        with self.lock:
            self._seen('attitude')
            self.s['att'] = dict(roll=math.degrees(r), pitch=math.degrees(p), yaw=math.degrees(y))

    def _on_mc(self, m):
        if m.sticks_moving:
            self._last_sticks_moving = time.time()
        with self.lock:
            self._seen('manual_control')
            self.s['mc'] = dict(roll=m.roll, pitch=m.pitch, yaw=m.yaw, throttle=m.throttle,
                                valid=bool(m.valid), sticks_moving=bool(m.sticks_moving),
                                buttons=int(m.buttons), source=int(m.data_source))

    def _on_fs(self, m):
        flags = [f for f in FailsafeFlags.get_fields_and_field_types()
                 if isinstance(getattr(m, f), bool) and getattr(m, f)]
        ignore = {'auto_mission_missing', 'home_position_invalid', 'global_position_invalid',
                  'global_position_invalid_relaxed', 'gcs_connection_lost', 'remote_id_unhealthy', 'gnss_lost'}
        with self.lock:
            self._seen('failsafe_flags')
            self.s['fs_flags'] = [f for f in flags if f not in ignore]

    def _on_mission(self, m):
        with self.lock:
            self._seen('mission')
            self.s['mission'] = m.data

    def _on_scan(self, scan):
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + scan.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        out = np.full(self.bins, np.nan)
        if np.any(ok):
            idx = ((a[ok] % (2 * math.pi)) / (2 * math.pi) * self.bins).astype(int) % self.bins
            np.fmin.at(out, idx, r[ok])
        with self.lock:
            self._seen('scan')
            self.s['scan'] = [None if not math.isfinite(v) else round(float(v), 3) for v in out]

    def _on_rosout(self, m):
        if m.name not in WATCHED_NODES:
            return
        level = {10: 'debug', 20: 'info', 30: 'warn', 40: 'error', 50: 'error'}.get(m.level, 'info')
        if level == 'debug':
            return
        text = m.msg
        if m.name == 'shaft_mission' and '->' in text:
            level = 'mission'
        self._event(level, m.name.replace('shaft_', ''), text)

    def _sample_history(self):
        with self.lock:
            lp = self.s['lp'] or {}
            self.hist.append((round(time.time() - self.t0, 1), self.s['depth'], self.s['clearance'],
                              lp.get('dist_bottom') if lp.get('dist_bottom_valid') else None))

    # ---------------- snapshot for the page ----------------
    def snapshot(self):
        now = time.time()
        with self.lock:
            s = dict(self.s)
            links = {}
            for key, label, stale in (('status', 'PX4 link', 1.5), ('scan', 'Lidar', 1.0),
                                      ('/shaft/valid', 'Perception', 1.0), ('mission', 'Mission node', 1.5),
                                      ('manual_control', 'Joystick', 1.5)):
                age = now - self.t_last[key] if key in self.t_last else None
                links[label] = dict(ok=age is not None and age < stale,
                                    age=None if age is None else round(age, 1),
                                    hz=round(self._hz(key), 1))
            events = list(self.events)[:60]
            hist = list(self.hist)[-600:]
        nav = s['nav']
        mission = s['mission']
        if s['failsafe']:
            authority = 'FAILSAFE'
        elif nav == 14:
            authority = 'MISSION'
        elif nav is None:
            authority = 'NO LINK'
        elif s['arming'] == 2:
            authority = 'PILOT'
        else:
            authority = 'ON GROUND'
        return dict(
            t=round(now - self.t0, 1), authority=authority,
            nav=NAV_NAMES.get(nav, '—' if nav is None else str(nav)),
            nav_user=NAV_NAMES.get(s['nav_user'], '—'),
            arming=ARM_NAMES.get(s['arming'], '—'), failsafe=s['failsafe'], fs_flags=s['fs_flags'],
            mission=mission, mission_states=MISSION_STATES,
            depth=s['depth'], clearance=s['clearance'], roundness=s['roundness'], radius=s['radius'],
            valid=s['valid'], offset=s['offset'], cmd=s['cmd'], sector_min=s['sector_min'], budget=s['budget'],
            scan=s['scan'], lp=s['lp'], att=s['att'], mc=s['mc'],
            links=links, events=events, hist=hist)


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    html_path = os.path.join(get_package_share_directory('shaft_inspection'), 'web', 'dashboard.html')

    def http(connection, request):
        if request.path in ('/', '/index.html'):
            with open(html_path, 'rb') as f:
                body = f.read()
            return Response(200, 'OK', Headers([('Content-Type', 'text/html; charset=utf-8'),
                                               ('Cache-Control', 'no-cache'),
                                               ('Content-Length', str(len(body)))]), body)
        if request.path == '/ws':
            return None                       # proceed with the websocket handshake
        return Response(404, 'Not Found', Headers([('Content-Length', '0')]), b'')

    async def stream(ws):
        try:
            while True:
                await ws.send(json.dumps(clean(node.snapshot()), allow_nan=False, default=lambda o: None))
                await asyncio.sleep(0.1)
        except Exception:
            return

    async def run():
        async with serve(stream, node.host, node.port, process_request=http):
            node.get_logger().info(f'dashboard: http://{node.host}:{node.port}/  (read-only)')
            await asyncio.Future()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
