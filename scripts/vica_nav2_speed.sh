#!/bin/bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
source /opt/ros/humble/setup.bash
source /home/ji_w/VICA-smarthandle/vica_ros2_ws/install/setup.bash
exec python3 "$(dirname "$(readlink -f "$0")")/vica_nav2_speed.py" "$@"
