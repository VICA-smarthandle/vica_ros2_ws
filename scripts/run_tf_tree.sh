#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/ji_w/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

OUT_DIR="/home/ji_w/ros2_ws/tf_trees"
mkdir -p "$OUT_DIR"

cd "$OUT_DIR" || exit 1

echo "[TF TREE] Saving TF tree to: $OUT_DIR"
echo "[TF TREE] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

ros2 run tf2_tools view_frames

echo ""
echo "[TF TREE] Saved files:"
ls -lh "$OUT_DIR"/frames*.pdf "$OUT_DIR"/frames*.gv 2>/dev/null | tail -n 10
