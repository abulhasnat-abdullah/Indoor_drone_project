#!/usr/bin/env python3
"""
lidar_mount_check -- find lidar_yaw_offset_deg and lidar_upside_down.

Bench procedure (props OFF):
  1. Put the vehicle on the ground with open space around it.
  2. Hold a board (or stand) about 0.6-1.0 m directly IN FRONT of the nose.
     Run this tool; it reports the raw bearing of that object.
  3. Move the board to the vehicle's LEFT side.  Run again.

  offset  = -(bearing measured in step 2)
  LEFT should then read about +90 deg after the offset.  If it reads -90,
  the lidar is effectively mirrored: set lidar_upside_down: true.
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node('lidar_mount_check')
    topic = sys.argv[1] if len(sys.argv) > 1 else '/scan'
    scans = []
    node.create_subscription(LaserScan, topic, scans.append, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not scans:
        print(f'FAIL: no LaserScan on {topic} within 3 s')
        return 1

    # nearest object between 0.3 and 1.5 m, averaged over all scans
    bearings = []
    for s in scans:
        r = np.asarray(s.ranges, dtype=float)
        a = s.angle_min + s.angle_increment * np.arange(r.size)
        m = np.isfinite(r) & (r > 0.3) & (r < 1.5)
        if np.count_nonzero(m) < 3:
            continue
        k = np.argmin(np.where(m, r, np.inf))
        # centroid of returns within 10 cm of the nearest -> centre of the board
        near = m & (r < r[k] + 0.10)
        bearings.append(math.atan2(np.mean(np.sin(a[near])), np.mean(np.cos(a[near]))))
    if not bearings:
        print('FAIL: nothing found between 0.3 and 1.5 m -- place the board closer')
        return 1
    b = math.degrees(math.atan2(np.mean(np.sin(bearings)), np.mean(np.cos(bearings))))
    print(f'{len(scans)} scans, object at raw bearing {b:+.1f} deg (CCW positive, 0 = scan zero)')
    print(f'If the object is IN FRONT of the nose:  lidar_yaw_offset_deg: {-b:+.1f}')
    print('If the object is on the LEFT: with the offset applied this should be ~+90;')
    print('  ~-90 means the scan is mirrored -> lidar_upside_down: true')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
