#!/usr/bin/env python3
"""
frontier_explorer.py

Autonomous frontier-based exploration on top of Nav2.

A "frontier" is the boundary between mapped free space and the unknown: any
free cell in the occupancy grid that touches an unknown cell. Driving to one is
the cheapest way to learn something new, because by definition the lidar will
see territory it has never seen. Explore every frontier and the map is closed.

The loop is:

    1. take the latest /map
    2. mark free cells adjacent to unknown cells
    3. cluster those cells (connected components) into candidate frontiers
    4. score each cluster by size and distance, skipping blacklisted ones
    5. send the nearest cell of the best cluster to Nav2 as a NavigateToPose
       goal (see _find_frontiers for why the centroid is the wrong point)
    6. when the goal finishes, fails, or times out -- go back to step 1

Why this instead of `m-explore-ros2`
------------------------------------
explore_lite is the usual answer and it is a fine package, but it is not
packaged for Jazzy and has to be built from source. This is one file with no
dependencies beyond what Nav2 already pulls in, it is tuned for a small indoor
space, and every knob it has is a ROS parameter you can retune without a
rebuild. On a Pi 5 the detection pass over a typical indoor map costs a couple
of milliseconds, because the heavy lifting is numpy and scipy.ndimage rather
than a Python loop over cells.

Key parameters
--------------
    min_frontier_size      cells; ignore specks of noise at the map edge
    distance_weight        higher = prefer near frontiers (thorough sweep),
                           lower  = prefer big ones (cover ground fast)
    goal_timeout           give up on a goal that is taking too long
    blacklist_radius       don't retry near a goal that already failed
    robot_radius_cells     require this much clearance around a candidate goal
"""
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from scipy import ndimage

from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

FREE, UNKNOWN = 0, -1


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('planning_period', 3.0)
        self.declare_parameter('min_frontier_size', 12)
        self.declare_parameter('free_threshold', 20)     # 0-100 occupancy
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('distance_weight', 1.5)
        self.declare_parameter('size_weight', 1.0)
        self.declare_parameter('min_goal_distance', 0.4)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('robot_radius_cells', 3)
        # How many consecutive empty passes before we declare the map closed.
        self.declare_parameter('done_after_empty_passes', 3)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('start_paused', False)

        gp = lambda n: self.get_parameter(n).value
        self.map_frame = gp('map_frame')
        self.base_frame = gp('base_frame')
        self.min_size = int(gp('min_frontier_size'))
        self.free_thr = int(gp('free_threshold'))
        self.occ_thr = int(gp('occupied_threshold'))
        self.w_dist = float(gp('distance_weight'))
        self.w_size = float(gp('size_weight'))
        self.min_goal_dist = float(gp('min_goal_distance'))
        self.goal_timeout = float(gp('goal_timeout'))
        self.blacklist_radius = float(gp('blacklist_radius'))
        self.robot_cells = int(gp('robot_radius_cells'))
        self.done_after = int(gp('done_after_empty_passes'))
        self.want_markers = bool(gp('publish_markers'))
        self.paused = bool(gp('start_paused'))

        self.map_msg = None
        self.goal_handle = None
        self.goal_active = False
        self.goal_sent_at = 0.0
        self.current_goal = None
        self.blacklist = []
        self.empty_passes = 0
        self.finished = False
        self.goals_sent = 0

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, gp('map_topic'),
                                 self._map_cb, map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 1)

        # Runtime control without restarting the node -- handy from the GUI or
        # a terminal mid-flight.
        self.create_service(Trigger, '~/pause', self._pause_cb)
        self.create_service(Trigger, '~/resume', self._resume_cb)
        self.create_service(Trigger, '~/reset', self._reset_cb)

        self.create_timer(float(gp('planning_period')), self._tick)
        self.get_logger().info(
            'frontier_explorer up. Waiting for /map and the navigate_to_pose '
            'action server...' + (' (started paused)' if self.paused else ''))

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                 rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return float(tf.transform.translation.x), float(tf.transform.translation.y)

    # ------------------------------------------------------------------
    def _tick(self):
        if self.paused or self.finished:
            return
        if self.map_msg is None:
            self.get_logger().info('no /map yet -- is slam_toolbox running?',
                                   throttle_duration_sec=10.0)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info('waiting for Nav2 navigate_to_pose action...',
                                   throttle_duration_sec=10.0)
            return

        if self.goal_active:
            if time.time() - self.goal_sent_at < self.goal_timeout:
                return
            self.get_logger().warn(
                f'goal timed out after {self.goal_timeout:.0f}s -- blacklisting it')
            self._blacklist(self.current_goal)
            self._cancel_goal()

        robot = self._robot_xy()
        if robot is None:
            self.get_logger().warn(
                f'no {self.map_frame}->{self.base_frame} TF yet',
                throttle_duration_sec=10.0)
            return

        frontiers = self._find_frontiers(self.map_msg, robot)
        if self.want_markers:
            self._publish_markers(frontiers)

        if not frontiers:
            self.empty_passes += 1
            self.get_logger().info(
                f'no reachable frontiers ({self.empty_passes}/{self.done_after})')
            if self.empty_passes >= self.done_after:
                self.finished = True
                self.get_logger().info(
                    f'EXPLORATION COMPLETE -- map closed after {self.goals_sent} '
                    f'goals. Call ~/reset to run again.')
            return

        self.empty_passes = 0
        self._send_goal(frontiers[0])

    # ------------------------------------------------------------------
    def _find_frontiers(self, grid: OccupancyGrid, robot):
        """Return candidate goals as (x, y, size, score), best first."""
        info = grid.info
        h, w = info.height, info.width
        if h == 0 or w == 0:
            return []
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)

        free = (data >= 0) & (data <= self.free_thr)
        unknown = data == UNKNOWN
        occupied = data >= self.occ_thr

        # A frontier cell is free and 4-adjacent to unknown space.
        cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
        unknown_nearby = ndimage.binary_dilation(unknown, structure=cross)
        frontier = free & unknown_nearby

        if not frontier.any():
            return []

        # Reject cells too close to an obstacle -- the drone cannot fit there
        # and Nav2 would just reject the goal.
        if self.robot_cells > 0:
            inflated = ndimage.binary_dilation(
                occupied, structure=np.ones((3, 3), dtype=bool),
                iterations=self.robot_cells)
            frontier &= ~inflated
            if not frontier.any():
                return []

        labels, n = ndimage.label(frontier, structure=np.ones((3, 3), dtype=bool))
        if n == 0:
            return []

        sizes = ndimage.sum_labels(frontier, labels, index=np.arange(1, n + 1))
        keep = np.nonzero(sizes >= self.min_size)[0] + 1
        if keep.size == 0:
            return []

        # Deliberately NOT the cluster centroid. A frontier is often an arc or
        # a ring around explored space, and its centre of mass then falls in
        # the middle of territory already mapped -- a goal that teaches the
        # robot nothing and that Nav2 reports as instantly reached. Instead,
        # take the cell of the cluster nearest the robot, which is guaranteed
        # to lie on the frontier itself and is the cheapest part to reach.
        rows, cols = np.nonzero(frontier)
        cell_labels = labels[rows, cols]
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        xs = ox + (cols + 0.5) * res
        ys = oy + (rows + 0.5) * res
        d2 = (xs - robot[0]) ** 2 + (ys - robot[1]) ** 2

        out = []
        for label in keep:
            member = cell_labels == label
            if not member.any():
                continue
            local = int(np.argmin(d2[member]))
            x = float(xs[member][local])
            y = float(ys[member][local])
            d = float(math.sqrt(d2[member][local]))
            if d < self.min_goal_dist:
                continue
            if self._is_blacklisted(x, y):
                continue
            size = float(sizes[label - 1])
            # Bigger frontiers are worth more; distance is a cost. The log
            # keeps a huge far-away frontier from dominating everything nearby.
            score = self.w_size * math.log1p(size) - self.w_dist * d
            out.append((x, y, size, score))

        out.sort(key=lambda f: f[3], reverse=True)
        return out

    # ------------------------------------------------------------------
    def _blacklist(self, xy):
        if xy:
            self.blacklist.append(xy)
            if len(self.blacklist) > 200:
                del self.blacklist[0]

    def _is_blacklisted(self, x, y):
        return any(math.dist((x, y), b) < self.blacklist_radius
                   for b in self.blacklist)

    def _send_goal(self, frontier):
        x, y, size, score = frontier
        robot = self._robot_xy() or (x, y)
        yaw = math.atan2(y - robot[1], x - robot[0])

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.current_goal = (x, y)
        self.goal_active = True
        self.goal_sent_at = time.time()
        self.goals_sent += 1
        self.get_logger().info(
            f'goal #{self.goals_sent}: ({x:.2f}, {y:.2f}) '
            f'{math.dist(robot, (x, y)):.1f} m away, {int(size)} frontier cells')

        self.nav.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'goal send failed: {e!r}')
            self.goal_active = False
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal -- blacklisting it')
            self._blacklist(self.current_goal)
            self.goal_active = False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().warn(f'goal result error: {e!r}')
            status = None
        # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
        if status != 4:
            self.get_logger().warn(
                f'goal did not succeed (status {status}) -- blacklisting it')
            self._blacklist(self.current_goal)
        else:
            self.get_logger().info('goal reached')
        self.goal_active = False
        self.goal_handle = None

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_active = False

    # ------------------------------------------------------------------
    def _publish_markers(self, frontiers):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontiers'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.18
        m.pose.orientation.w = 1.0
        for i, (x, y, _size, _score) in enumerate(frontiers[:25]):
            m.points.append(Point(x=x, y=y, z=0.1))
            # Brightest sphere is the one we are about to fly to.
            best = i == 0
            m.colors.append(ColorRGBA(r=1.0 if best else 0.2,
                                      g=0.9 if best else 0.5,
                                      b=0.1 if best else 0.9,
                                      a=1.0 if best else 0.55))
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    # ------------------------------------------------------------------
    def _pause_cb(self, request, response):
        self.paused = True
        self._cancel_goal()
        response.success = True
        response.message = 'exploration paused, current goal cancelled'
        self.get_logger().info(response.message)
        return response

    def _resume_cb(self, request, response):
        self.paused = False
        self.finished = False
        self.empty_passes = 0
        response.success = True
        response.message = 'exploration resumed'
        self.get_logger().info(response.message)
        return response

    def _reset_cb(self, request, response):
        n = len(self.blacklist)
        self.blacklist.clear()
        self.empty_passes = 0
        self.finished = False
        response.success = True
        response.message = f'cleared {n} blacklisted goals and restarted exploration'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
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
