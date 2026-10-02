#!/usr/bin/env python3
"""
Full chain: PX4 odom bridge (/odom + static lidar TF, no dynamic TF) ->
rf2o (owns odom->base_link TF) -> slam_toolbox (map->odom correction,
loop closure) -> vision_odom_bridge (feeds corrected pose back into PX4
EKF2 as external vision).

Staggered with TimerAction: slam_toolbox was previously starting before
rf2o had published odom->base_link TF or /scan was flowing, which made it
sit idle with no visible output (looks like "not launching" but is really
"launched with nothing to match against yet"). Each stage now waits for
the one before it to plausibly be alive.

--log-level debug is set on rf2o and slam_toolbox so failures/idling show
up in the terminal instead of being silent.
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('odom_bridge')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([

        # 1. PX4 -> /odom + static lidar TF + gz clock/lidar bridges.
        #    Starts immediately; everything else depends on /scan and
        #    /clock which this brings up.
        Node(
            package='odom_bridge',
            executable='odom_bridge_node',
            name='odom_bridge_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'publish_dynamic_tf': False,
                'use_sim_time': True,
            }],
        ),

        # 2. rf2o laser odometry — owns odom->base_link TF. Delayed so
        #    /scan and /clock are already flowing before it starts
        #    matching, rather than racing the gz bridge subprocesses
        #    spawned above.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rf2o_laser_odometry',
                    executable='rf2o_laser_odometry_node',
                    name='rf2o_laser_odometry',
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'debug'],
                    parameters=[{
                        'laser_scan_topic': '/scan',
                        'odom_topic': '/odom_rf2o',
                        'publish_tf': True,
                        'base_frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'init_pose_from_topic': '',
                        'freq': 20.0,
                        'use_sim_time': True,
                    }],
                )
            ]
        ),

        # 3. slam_toolbox — map->odom correction + loop closure. Delayed
        #    further so rf2o's odom->base_link TF and /scan are both
        #    confirmed alive before slam_toolbox's scan matcher starts
        #    looking for them.
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'debug'],
                    parameters=[slam_params_file, {'use_sim_time': True}],
                )
            ]
        ),

        # 4. Corrected pose -> PX4 external vision. Delayed so slam_toolbox
        #    has had a chance to publish at least one map->odom TF before
        #    this starts looking up map->base_link.
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='odom_bridge',
                    executable='vision_odom_bridge',
                    name='vision_odom_bridge',
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'debug'],
                    parameters=[{'use_sim_time': True}],
                )
            ]
        ),
    ])