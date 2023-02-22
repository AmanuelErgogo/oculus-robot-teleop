#!/bin/bash

# Specify the name of the catkin workspace
workspace_name="teleop_ws"

# Specify the name of the source directory
source_dir="src"

# Set the ROS_MASTER_URI and ROS_IP environment variables
export ROS_MASTER_URI=http://192.168.0.134:11311
export ROS_IP=192.168.0.134

# Start the roscore process
roscore &

# Wait for the roscore process to start up
sleep 5

# Launch a node using roslaunch
cd ~/$workspace_name
source devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch

cd ~/$workspace_name
source devel/setup.bash
cd ../../..
./orbit.sh -p source/standalone/ros_ws/src/teleop_isaac/scripts/teleop_demo.py