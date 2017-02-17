#!/bin/bash

sudo etherwake c0:3f:d5:6b:d6:85

sleep 10


sudo ifconfig eth0 10.0.0.1

sleep 20

source /opt/ros/indigo/setup.bash
export ROS_IP=10.0.0.2
export ROS_MASTER_URI=http://10.0.0.2:11311
export ROS_PACKAGE_PATH=/opt/ros/indigo/stacks:$ROS_PACKAGE_PATH
sleep 2
# roslaunch launch_package.launch

