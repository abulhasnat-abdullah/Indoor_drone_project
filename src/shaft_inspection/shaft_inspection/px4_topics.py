#!/usr/bin/env python3
"""Resolve PX4 uORB topic names that carry a message-version suffix.

PX4 1.16+ publishes versioned topic names (/fmu/out/vehicle_local_position_v1,
/fmu/out/vehicle_status_v4, ...) while the ROS message TYPE stays unversioned
(px4_msgs/msg/VehicleLocalPosition).  The suffix changes between PX4 releases,
so hard-coding it makes a node break on the next upgrade.  Resolve it at
runtime instead.
"""
import re
import time

from rclpy.clock import Clock, ClockType
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


def px4_qos(depth=5):
    """QoS matching the uXRCE-DDS client's publishers."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def resolve(node, base, default=None, wait_s=5.0, poll_s=0.25):
    """Return the live topic whose name is `base` or `base_v<N>`.

    Discovery is not instant: a node constructed before PX4's DDS client has
    announced its publishers sees an empty graph, and silently falling back to
    the unsuffixed name then subscribes to a topic nobody publishes -- a node
    that starts cleanly and simply never receives anything.  Poll briefly
    instead, and only fall back once the wait is genuinely exhausted.
    """
    pattern = re.compile(r'^' + re.escape(base) + r'(_v\d+)?$')
    deadline = time.monotonic() + max(0.0, wait_s)
    while True:
        best = None
        for name, _types in node.get_topic_names_and_types():
            if pattern.match(name):
                # Prefer the versioned spelling when both are present.
                if best is None or len(name) > len(best):
                    best = name
        if best is not None:
            return best
        if time.monotonic() >= deadline:
            break
        time.sleep(poll_s)

    fallback = default if default is not None else base
    node.get_logger().warn(
        f'no publisher found for {base} (or {base}_v<N>) after {wait_s:.0f}s; '
        f'falling back to {fallback}')
    return fallback


_SYSTEM_CLOCK = Clock(clock_type=ClockType.SYSTEM_TIME)


def px4_timestamp_us():
    """Timestamp for messages sent TO PX4, in microseconds of SYSTEM time.

    Do not use node.get_clock() here.  With use_sim_time the node clock runs on
    Gazebo /clock, but the uXRCE-DDS client time-syncs PX4 against the host's
    system clock and rewrites incoming timestamps with that offset.  A sim-time
    stamp lands thousands of seconds in the past after correction, so PX4
    reports offboard_control_signal_lost and EKF2 rejects every vision sample,
    while the topics look perfectly connected.
    """
    return int(_SYSTEM_CLOCK.now().nanoseconds / 1000)
