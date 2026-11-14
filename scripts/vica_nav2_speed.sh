#!/bin/bash
# Nav2 최고 속도를 노드를 끄지 않고 바꾼다. 사용법·하는 일은 vica_nav2_speed.py 머리말.
#   ~/VICA-smarthandle/vica_ros2_ws/scripts/vica_nav2_speed.sh 0.9
#   ~/VICA-smarthandle/vica_ros2_ws/scripts/vica_nav2_speed.sh 0.5
# 환경은 로봇 칸들과 같게(~/.bashrc 의 humble 별칭과 같은 값). 이미 정해져 있으면 그대로 쓴다.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
source /opt/ros/humble/setup.bash
source /home/ji_w/VICA-smarthandle/vica_ros2_ws/install/setup.bash
exec python3 "$(dirname "$(readlink -f "$0")")/vica_nav2_speed.py" "$@"
