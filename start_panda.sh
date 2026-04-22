#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/mobseap/mobseap_ws/install/setup.bash

export ROS_DOMAIN_ID=123
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 launch lattepanda_bringup panda_bringup.launch.py