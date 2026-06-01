#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/ji_w/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[TF VICA] Publishing Host static TF"
echo "[TF VICA] base_link -> laser_frame"
echo "[TF VICA] base_link -> camera_link"

# base_link -> laser_frame
ros2 run tf2_ros static_transform_publisher \
  0.007 0.0 0.346 0 0 0 \
  base_link laser_frame &

PID_LASER=$!

# base_link -> camera_link
ros2 run tf2_ros static_transform_publisher \
  0.079 0.0 0.288 0 0 0 \
  base_link camera_link &

PID_CAMERA=$!

trap "echo '[TF VICA] Stopping static TF publishers'; kill $PID_LASER $PID_CAMERA 2>/dev/null" INT TERM

wait
