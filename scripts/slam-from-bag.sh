#!/bin/bash
# (no set -u: ROS setup scripts reference unbound vars)
source /opt/ros/jazzy/setup.sh
LOG=/tmp/slam-final.log
: > "$LOG"
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$HOME/robot/config/slam_offline.yaml use_sim_time:=false >> "$LOG" 2>&1 &
SLAM=$!
sleep 2
ros2 bag play "${BAG:-$HOME/robot-bags/mapping-drive-20261004-stamped}" >> "$LOG" 2>&1 &
BAGPROC=$!
wait $BAGPROC
echo "[bag done]" >> "$LOG"
sleep 5
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: ${MAP_NAME:-./maps/mapping_drive_20261004}}}" >> "$LOG" 2>&1
kill $SLAM 2>/dev/null
wait $SLAM 2>/dev/null
echo "[done]" >> "$LOG"
