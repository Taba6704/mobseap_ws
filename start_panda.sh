#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/mobseap/mobseap_ws/install/setup.bash

export ROS_DOMAIN_ID=123
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATh=/usr/local/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH

echo "Waiting for OAK Camera..."

for i in {1..60}; do
    if oakctl list 2>/dev/null | grep -q "OAK4-D"; then
        echo "OAK Camera Detected."
        break
    fi

    echo "OAK not detected yet... attempt $i/60"
    sleep 2
done

echo "Starting LattePanda bringup..."
exec ros2 launch lattepanda_bringup panda_bringup.launch.py