#!/bin/bash

# Terminal 1: Run odom_bridge_node
echo "Starting odom_bridge_node..."
ros2 run odom_bridge odom_bridge_node &
sleep 3

# Terminal 2: Launch rf2o_laser_odometry
echo "Launching rf2o_laser_odometry..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &
sleep 15

# Terminal 3: Launch slam_toolbox
echo "Launching slam_toolbox..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/abdazwd/px4_ros2_ws/src/odom_bridge/config/slam_toolbox_params.yaml &
sleep 15

# Terminal 4: Run vision_odom_bridge
echo "Starting vision_odom_bridge..."
ros2 run odom_bridge vision_odom_bridge &


# Keep the script running to monitor background processes
echo "All nodes started successfully. Press [CTRL+C] to stop everything."
trap "kill 0" EXIT
wait
