#!/bin/bash
# Callable Sorting Garbage Bin - Kobuki Startup Script
# Usage:
#   source ~/kobuki_start.sh
#   Then run one of:
#     roslaunch ~/kobuki_launch/kobuki_bringup.launch    (start robot + lidar)
#     roslaunch ~/kobuki_launch/kobuki_slam.launch        (mapping mode)
#     roslaunch ~/kobuki_launch/kobuki_nav.launch         (navigation mode)

# Source ROS Melodic
source /opt/ros/melodic/setup.bash

# Source all workspace packages
for dir in ~/catkin_ws/devel_isolated/*/; do
    if [ -f "$dir/setup.bash" ]; then
        source "$dir/setup.bash"
    fi
done

# ROS environment
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

echo "========================================="
echo "  Kobuki Callable Garbage Bin - Ready"
echo "========================================="
echo ""
echo "STEP 1: Start the robot (Terminal 1):"
echo "  roslaunch ~/kobuki_launch/kobuki_bringup.launch"
echo ""
echo "STEP 2a: For MAPPING (Terminal 2):"
echo "  roslaunch ~/kobuki_launch/kobuki_slam.launch"
echo ""
echo "STEP 2b: For NAVIGATION (Terminal 2):"
echo "  roslaunch ~/kobuki_launch/kobuki_nav.launch"
echo ""
echo "NOTE: If RPLiDAR fails, unplug/replug USB and retry"
echo "========================================="
