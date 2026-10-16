#!/bin/bash
# robot panel --slam: supervised bridge + SLAM Toolbox + web teleop.
# No Nav2, RViz or autonomous command source. The bridge owns the on-robot
# supervisor; stopping it triggers stock-controller restoration.
# No nounset: ROS setup scripts reference unset variables.
set -eo pipefail
port=8085
while (( $# )); do
    case $1 in
        --port)
            (( $# >= 2 )) || { echo 'robot mapping: --port requires a value' >&2; exit 2; }
            port=$2; shift 2 ;;
        *) echo "robot mapping: unknown option: $1" >&2; exit 2 ;;
    esac
done
if [[ ! $port =~ ^[0-9]{1,5}$ ]] || (( 10#$port < 1 || 10#$port > 65535 )); then
    echo 'robot mapping: port must be in 1..65535' >&2
    exit 2
fi
repo=$(cd "$(dirname "$(realpath "$0")")/.." && pwd)
robot_bin=${ROBOT_BIN:-$repo/bin/robot}
RUN_LOG=${ROBOT_RUN_LOG:-/tmp/robot-mapping.log}
MAP_OUT=${ROBOT_MAP_OUT:-$repo/maps/live-$(date +%Y%m%d-%H%M%S)}
banner() { printf '\n[robot mapping] %s\n' "$*"; }
die() { echo "robot mapping: $*" >&2; exit 2; }
# shellcheck source=/dev/null
source "${ROBOT_ROS_SETUP:-/opt/ros/jazzy/setup.sh}" || die 'ROS setup not found'
# shellcheck source=/dev/null
source "${ROBOT_WS_SETUP:-$HOME/robot/ros-ws/install/setup.sh}" 2>/dev/null \
    || die 'ros-ws workspace is not built; see README'
# shellcheck source=scripts/lib.sh
source "$repo/scripts/lib.sh"
[[ -x $robot_bin ]] || die "robot CLI not found: $robot_bin"
command -v ros2 >/dev/null || die 'ros2 is not in PATH'
[[ -f $repo/config/slam_online.yaml ]] || die 'SLAM config not found'
if other=$(running_ros_session); then
    die "$other is already running on this host; end it first"
fi
# Open the log before installing traps or taking ownership of the robot.
: >"$RUN_LOG"
pids=()
BRIDGE_PID=
SLAM_PID=
PANEL_PID=
session_rc=0

# shellcheck disable=SC2317 # invoked by cleanup trap
save_map() {
    banner "saving the map to $MAP_OUT.yaml"
    if mkdir -p "$(dirname "$MAP_OUT")" &&
        timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAP_OUT" \
            --ros-args -p use_sim_time:=false >>"$RUN_LOG" 2>&1; then
        banner "map saved: $MAP_OUT.yaml"
    else
        echo "robot mapping: WARNING: the map was not saved; see $RUN_LOG" >&2
    fi
}

# Stop the panel's command source BEFORE zeroing /cmd_vel. Save while SLAM
# is alive, then stop all groups (including the bridge's tee pipeline).
# Repeated signals must not interrupt this bounded teardown.
# shellcheck disable=SC2317 # invoked by traps
cleanup() {
    local status=$? restored=0 _
    trap '' INT TERM EXIT
    (( session_rc )) || session_rc=$status
    banner 'teardown'
    if [[ -n $PANEL_PID ]]; then stop_components "$PANEL_PID"; fi
    if [[ -n $BRIDGE_PID ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        timeout 5 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
            '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
    fi
    if [[ -n $SLAM_PID ]] && kill -0 "$SLAM_PID" 2>/dev/null; then save_map; fi
    stop_components "${pids[@]}"
    # Supervisor rollback normally restores the controller. As in robot run,
    # cover startup failures before heartbeat ACTIVATE with robot enable.
    if [[ -n $BRIDGE_PID ]]; then
        for _ in $(seq 1 "${ROBOT_RESTORE_TRIES:-10}"); do
            if timeout 20 "$robot_bin" status 2>/dev/null | grep -q '^enabled$'; then
                restored=1; break
            fi
            sleep 1
        done
        if (( ! restored )); then
            banner 'controller still down; restoring via robot enable'
            if timeout 120 "$robot_bin" enable >/dev/null 2>&1 &&
                timeout 20 "$robot_bin" status 2>/dev/null | grep -q '^enabled$'; then
                banner 'stock controller restored via robot enable'
            else
                echo "robot mapping: WARNING: stock controller did not return; run '$robot_bin enable' and check the robot" >&2
                session_rc=2
            fi
        fi
    fi
    banner 'done'
    exit "$session_rc"
}
trap cleanup EXIT
trap 'session_rc=130; exit 130' INT
trap 'session_rc=143; exit 143' TERM

banner "starting supervised bridge; log: $RUN_LOG"
# pipefail is essential: a crashed bridge must not be hidden by tee's exit.
# shellcheck disable=SC2016 # positional args expand in the child shell
setsid bash -o pipefail -c 'bash "$1" bridge 2>&1 | tee -a "$2"' _ "$robot_bin" "$RUN_LOG" </dev/null &
BRIDGE_PID=$!
pids+=("$BRIDGE_PID")

scan_seen=0
for _ in $(seq 1 "${ROBOT_SCAN_TRIES:-60}"); do
    kill -0 "$BRIDGE_PID" 2>/dev/null || break
    if timeout 2 ros2 topic echo /scan --once >/dev/null 2>&1; then scan_seen=1; break; fi
done
if (( ! scan_seen )); then
    echo "robot mapping: no /scan from the bridge; see $RUN_LOG" >&2
    exit 3
fi
banner 'starting SLAM Toolbox'
setsid ros2 launch slam_toolbox online_sync_launch.py \
    slam_params_file:="$repo/config/slam_online.yaml" use_sim_time:=false >>"$RUN_LOG" 2>&1 &
SLAM_PID=$!
pids+=("$SLAM_PID")
map_seen=0
for _ in $(seq 1 "${ROBOT_MAP_TRIES:-45}"); do
    if ! kill -0 "$BRIDGE_PID" 2>/dev/null || ! kill -0 "$SLAM_PID" 2>/dev/null; then break; fi
    if timeout 2 ros2 topic echo /map --once --qos-durability transient_local >/dev/null 2>&1; then
        map_seen=1; break
    fi
done
if (( ! map_seen )); then
    echo "robot mapping: no /map from SLAM; see $RUN_LOG" >&2
    exit 3
fi
banner "starting web panel on port $port; Ctrl-C ends the session and saves $MAP_OUT.yaml"
# Do not pass --slam recursively: the plain panel is just the web process.
setsid "$robot_bin" panel --port "$port" >>"$RUN_LOG" 2>&1 &
PANEL_PID=$!
pids+=("$PANEL_PID")
finished=
rc=0
wait -n -p finished "$BRIDGE_PID" "$SLAM_PID" "$PANEL_PID" || rc=$?
if [[ $finished != "$PANEL_PID" || $rc != 0 ]]; then
    echo "robot mapping: a session component exited (status $rc); see $RUN_LOG" >&2
    session_rc=3
fi
exit "$session_rc"
