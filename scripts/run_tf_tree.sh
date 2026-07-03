#!/bin/bash

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

OUT_DIR="$HOME/ros2_ws/tf_trees"
mkdir -p "$OUT_DIR"
cd "$OUT_DIR" || exit 1

echo "============================================================"
echo "[TF TREE - HOST] Saving TF tree to: $OUT_DIR"
echo "[TF TREE - HOST] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[TF TREE - HOST] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[TF TREE - HOST] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "============================================================"

echo ""
echo "[1] Refresh ROS daemon..."
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true
sleep 3

echo ""
echo "[2] Camera topics visible?"
ros2 topic list | grep camera || true

echo ""
echo "[3] TF static camera frames visible?"
timeout 8 ros2 topic echo /tf_static --once | grep -E "camera_link|camera_color|camera_depth|camera_imu|camera_gyro|camera_accel" || true

echo ""
echo "[4] TF echo test: camera_link -> camera_color_optical_frame"
timeout 8 ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame || true

echo ""
echo "[5] TF echo test: camera_link -> camera_depth_optical_frame"
timeout 8 ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame || true

echo ""
echo "[6] Keep old frames files..."
echo "[TF TREE - HOST] Existing frames*.pdf/gv/yaml files will not be removed."

echo ""
echo "[7] Run view_frames..."
ros2 run tf2_tools view_frames

echo ""
echo "[8] Saved files:"
ls -lh "$OUT_DIR"/frames* 2>/dev/null || true

LATEST_GV=$(ls -t "$OUT_DIR"/frames*.gv 2>/dev/null | head -n 1)
LATEST_PDF=$(ls -t "$OUT_DIR"/frames*.pdf 2>/dev/null | head -n 1)

echo ""
echo "[9] Check camera frames inside GV file:"
if [ -n "$LATEST_GV" ]; then
  echo "[GV] $LATEST_GV"
  grep -E "camera_link|camera_color_optical_frame|camera_depth_optical_frame|camera_imu_optical_frame" "$LATEST_GV" || true
else
  echo "[ERROR] No .gv file generated"
fi

echo ""
echo "============================================================"
echo "[TF TREE - HOST] Done"
echo "Latest PDF:"
echo "$LATEST_PDF"
echo "Open command:"
echo "xdg-open $LATEST_PDF"
echo "============================================================"
