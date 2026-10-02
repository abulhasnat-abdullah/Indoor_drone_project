#!/usr/bin/env python3
"""Bring up the shaft-inspection stack (bridges + perception + mission + mapper).

PX4 SITL and the Micro XRCE agent are started separately by run_cmds/shaft_sim.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD = 'vshaft'
MODEL = 'x500_shaft_0'

SCAN_GZ = f'/world/{WORLD}/model/{MODEL}/link/link/sensor/lidar_2d_v2/scan'
DOWN_GZ = f'/world/{WORLD}/model/{MODEL}/link/lidar_sensor_link/sensor/lidar/scan'


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('shaft_inspection'),
                       'config', 'shaft.yaml')

    autostart = LaunchConfiguration('autostart')
    start_mode = LaunchConfiguration('start_mode')

    bridges = [
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gz_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gz_scan_bridge',
             arguments=[f'{SCAN_GZ}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
             remappings=[(SCAN_GZ, '/scan')],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gz_down_bridge',
             arguments=[f'{DOWN_GZ}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
             remappings=[(DOWN_GZ, '/down_range')],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]

    perception = Node(package='shaft_inspection', executable='shaft_perception',
                      name='shaft_perception', parameters=[cfg], output='screen')
    dashboard = Node(package='shaft_inspection', executable='shaft_dashboard',
                     name='shaft_dashboard', parameters=[{'port': 8080}], output='screen',
                     condition=IfCondition(LaunchConfiguration('dashboard')))
    mapper = Node(package='shaft_inspection', executable='shaft_mapper',
                  name='shaft_mapper', parameters=[cfg], output='screen')
    # start_mode: auto_launch (node arms and takes off) or pilot_handover (you
    # fly it, then switch to Offboard to start).  Deliberately absent from
    # shaft.yaml: a node-specific key there outranks a value passed here.
    mission = Node(package='shaft_inspection', executable='shaft_mission',
                   name='shaft_mission', parameters=[cfg, {'start_mode': start_mode}],
                   output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Start the mission node automatically'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='Serve the read-only web dashboard on :8080'),
        DeclareLaunchArgument('start_mode', default_value='auto_launch',
                              description='auto_launch | pilot_handover'),
        *bridges,
        TimerAction(period=3.0, actions=[perception, mapper, dashboard]),
        # The mission node only arms once it has a bore fix, but give perception
        # a head start so the first thing it sees is already valid.
        TimerAction(period=8.0, actions=[mission]),
    ])
