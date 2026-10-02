#!/usr/bin/env python3
"""
shaft_preflight -- go / no-go check of everything the mission depends on.

Run on the vehicle with the full stack up (props off for bench, or in the bore
before handing over).  Listens for ~8 s and prints PASS / WARN / FAIL.

  --in-bore   also require a valid bore fix and EKF2 horizontal position
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus

from .px4_topics import px4_qos, resolve

LISTEN_S = 8.0


class Check(Node):
    def __init__(self):
        super().__init__('shaft_preflight')
        self.scans, self.att, self.lp, self.st = [], [], [], []
        self.valid, self.clear = [], []
        q = px4_qos()
        self.create_subscription(LaserScan, '/scan', self.scans.append, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, resolve(self, '/fmu/out/vehicle_attitude', wait_s=3.0),
                                 self.att.append, q)
        self.create_subscription(VehicleLocalPosition, resolve(self, '/fmu/out/vehicle_local_position', wait_s=3.0),
                                 self.lp.append, q)
        self.create_subscription(VehicleStatus, resolve(self, '/fmu/out/vehicle_status', wait_s=3.0),
                                 self.st.append, q)
        self.create_subscription(Bool, '/shaft/valid', lambda m: self.valid.append(m.data), 10)
        self.create_subscription(Float32, '/shaft/clearance', lambda m: self.clear.append(m.data), 10)


def main():
    in_bore = '--in-bore' in sys.argv
    rclpy.init()
    n = Check()
    t0 = time.time()
    while time.time() - t0 < LISTEN_S:
        rclpy.spin_once(n, timeout_sec=0.05)
    results = []

    def res(level, what, detail):
        results.append(level)
        print(f'{level:5s} {what:28s} {detail}')

    # ---- lidar ----
    hz = len(n.scans) / LISTEN_S
    if not n.scans:
        res('FAIL', 'lidar /scan', 'no data (driver running? serial port? 460800 baud?)')
    else:
        res('PASS' if 7.0 <= hz <= 13.0 else 'WARN', 'lidar rate', f'{hz:.1f} Hz (C1 nominal 10)')
        s = n.scans[-1]
        r = np.asarray(s.ranges, dtype=float)
        finite = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        res('PASS' if r.size >= 400 else 'WARN', 'lidar samples/scan', f'{r.size}')
        close = [np.count_nonzero(np.isfinite(np.asarray(x.ranges)) & (np.asarray(x.ranges) < 0.40))
                 for x in n.scans]
        near = float(np.min(r[finite])) if np.any(finite) else float('inf')
        res('WARN' if np.median(close) > 0 else 'PASS', 'self-hits < 0.40 m',
            f'{int(np.median(close))} returns/scan, nearest {near:.2f} m '
            '-> set self_filter_radius above the airframe returns')

    # ---- PX4 link ----
    if not n.st:
        res('FAIL', 'PX4 DDS link', 'no vehicle_status (agent running? UXRCE_DDS_CFG? px4_msgs = release/1.14?)')
    else:
        res('PASS', 'PX4 DDS link', f'status {len(n.st)/LISTEN_S:.1f} Hz, attitude {len(n.att)/LISTEN_S:.0f} Hz')
        s = n.st[-1]
        res('PASS', 'arming / nav state', f'arming={s.arming_state} nav={s.nav_state}')
    if n.lp:
        m = n.lp[-1]
        res('PASS' if m.z_valid else 'FAIL', 'EKF2 height', f'z_valid={m.z_valid}')
        rng = bool(m.dist_bottom_valid and (m.dist_bottom_sensor_bitfield & m.DIST_BOTTOM_SENSOR_RANGE))
        res('PASS' if rng else 'FAIL', 'H-Flow range (dist_bottom)',
            f'valid={m.dist_bottom_valid} from_range={bool(m.dist_bottom_sensor_bitfield & 1)} '
            f'dist={m.dist_bottom:.2f} m')
        if in_bore:
            res('PASS' if m.xy_valid else 'FAIL', 'EKF2 horizontal position',
                f'xy_valid={m.xy_valid} (needs the lidar bore fix fused as EV)')
        if n.att:
            q = n.att[-1].q
            roll = math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2)))
            pitch = math.degrees(math.asin(max(-1, min(1, 2*(q[0]*q[2]-q[3]*q[1])))))
            res('PASS' if abs(roll) < 5 and abs(pitch) < 5 else 'WARN', 'attitude (on ground)',
                f'roll {roll:+.1f} pitch {pitch:+.1f} deg')

    # ---- perception ----
    if in_bore:
        frac = float(np.mean(n.valid)) if n.valid else 0.0
        res('PASS' if frac > 0.9 else 'FAIL', 'bore fix', f'valid {frac*100:.0f}% of scans')
        if n.clear:
            c = float(np.median(n.clear))
            res('PASS' if c > 0.6 else 'FAIL', 'bore clearance', f'{c:.2f} m (descent needs > 0.60)')

    verdict = 'NO-GO' if 'FAIL' in results else ('GO with warnings' if 'WARN' in results else 'GO')
    print(f'\n==> {verdict}')
    n.destroy_node()
    rclpy.shutdown()
    return 1 if 'FAIL' in results else 0


if __name__ == '__main__':
    sys.exit(main())
