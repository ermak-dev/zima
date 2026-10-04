#!/bin/bash
# Offline localization check: AMCL on the saved map against the replayed
# mapping bag. Success criterion: /amcl_pose published continuously and
# map -> odom appears in TF while the bag plays.
# (no set -u: ROS setup scripts reference unbound vars)
source /opt/ros/jazzy/setup.sh
LOG=${LOG:-/tmp/amcl-replay.log}
: > "$LOG"

BAG=${BAG:-$HOME/robot-bags/mapping-drive-20261004-stamped}
MAP=${MAP:-$HOME/robot/maps/mapping_drive_20261004.yaml}
PARAMS=${PARAMS:-$HOME/robot/config/amcl_replay.yaml}

ros2 launch nav2_bringup localization_launch.py \
    map:=$MAP use_sim_time:=false autostart:=true \
    params_file:=$PARAMS >> "$LOG" 2>&1 &
LOC=$!
sleep 4

echo "[stage] bag play" >> "$LOG"
ros2 bag play "$BAG" >> "$LOG" 2>&1 &
BAGPROC=$!
wait $BAGPROC
echo "[bag done]" >> "$LOG"
kill $LOC 2>/dev/null
wait $LOC 2>/dev/null
echo "[done]" >> "$LOG"
