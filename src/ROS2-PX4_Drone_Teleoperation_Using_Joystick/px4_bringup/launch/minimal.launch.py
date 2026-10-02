#!/usr/bin/env python3
"""
PX4 Minimal Launch File
Launches PX4 SITL (Headless), Gazebo GUI, MicroXRCE Agent, Gazebo bridges, and QGroundControl.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

# ── Updated for the hospital world and x500_gps_denied model ─────────────
# Note: If topic doesn't match, run: gz topic -l | grep -i scan
GZ_LIDAR_TOPIC = '/world/hospital/model/x500_gps_denied_0/link/link/sensor/lidar_2d_v2/scan'  


def generate_launch_description():

    # Define paths
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    qgc_dir = os.path.expanduser('~/Downloads')

    return LaunchDescription([
        # 1. Launch PX4 SITL Headless (hospital world, spawn at 0,15,0)
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL (Headless)',
                '--',
                'bash', '-c',
                f'cd {px4_dir} && HEADLESS=1 PX4_GZ_MODEL_POSE="0,15,0,0,0,0" PX4_GZ_WORLD=hospital make px4_sitl gz_x500_gps_denied; exec bash'
            ],
            output='screen',
            name='px4_sitl'
        ),

        # 2. Launch MicroXRCE Agent after 3 seconds
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE Agent',
                        '--',
                        'bash', '-c',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen',
                    name='microxrce_agent'
                )
            ]
        ),

        # 3. Launch Gazebo GUI after 11 seconds (since PX4 is headless)
        TimerAction(
            period=11.0,
            actions=[
                ExecuteProcess(
                    cmd=['gz', 'sim', '-g'],
                    output='screen',
                    name='gz_gui'
                )
            ]
        ),

        # 4. Launch QGroundControl after 6 seconds
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', f'cd {qgc_dir} && ./QGroundControl-x86_64.AppImage'],
                    output='screen',
                    name='qgroundcontrol'
                )
            ]
        ),

        # 5. Gazebo clock bridge — after 8s (ensures world is fully up)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_clock_bridge',
                    arguments=[
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                    ],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                )
            ]
        ),

        # 6. LiDAR scan bridge — after 8s
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_lidar_bridge',
                    arguments=[
                        f'{GZ_LIDAR_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                    ],
                    remappings=[
                        (GZ_LIDAR_TOPIC, '/scan'),
                    ],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                )
            ]
        ),
    ])