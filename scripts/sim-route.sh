#!/bin/bash
# Regression check for Nav2 settings and the simulated vacuum: start the
# playground headless (`robot run --sim`, optionally --slam), drive the
# fixed four-goal route of ros2/route_check.py, stop the sim, and exit
# with the verdict (0 = every goal reached, 1 = a goal failed, 3 = the
# simulator never came up). Maps of failed goals land in /tmp/robot-route.
# usage: sim-route.sh [--slam]
# (no set -u: ROS setup scripts reference unbound vars)
set -eo pipefail
sim_args=()
for arg in "$@"; do
    case $arg in
        --slam) sim_args+=(--slam) ;;
        *) echo "sim-route: unknown option: $arg (usage: sim-route.sh [--slam])" >&2; exit 2 ;;
    esac
done
ROS_SETUP=${ROBOT_ROS_SETUP:-/opt/ros/jazzy/setup.sh}
# shellcheck source=/dev/null
source "$ROS_SETUP"

repo=$(cd "$(dirname "$(realpath "$0")")/.." && pwd)
SIM=${ROBOT_ROUTE_SIM_SCRIPT:-$repo/scripts/run-sim.sh}
CLIENT=${ROBOT_ROUTE_CLIENT:-$repo/ros2/route_check.py}
OUT=${ROBOT_ROUTE_OUT:-/tmp/robot-route}

# Goals are relative to the start pose. Under SLAM the map frame starts
# at the robot; under AMCL it is the world, so shift by the spawn point
# the sim overlay gives AMCL.
if (( ${#sim_args[@]} )); then
    offset=0,0
else
    offset=$(python3 - "$repo/config/nav2_sim.yaml" <<'PY'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))['amcl']['ros__parameters']['initial_pose']
print(f"{p['x']},{p['y']}")
PY
)
fi

console=$(mktemp /tmp/robot-route-sim.XXXXXX.log)
# Own session: the sim wrapper and its group are stopped together.
ROBOT_SIM_HEADLESS=True ROBOT_SIM_RVIZ=False \
    setsid bash "$SIM" "${sim_args[@]}" >"$console" 2>&1 &
sim_pid=$!

# shellcheck disable=SC2317  # runs from the traps below
stop_sim() {
    trap '' INT TERM EXIT
    kill -TERM -- "-$sim_pid" 2>/dev/null || kill -TERM "$sim_pid" 2>/dev/null || true
    local waited=0
    while kill -0 "$sim_pid" 2>/dev/null && (( waited < 60 )); do
        sleep 1; waited=$((waited + 1))
    done
    rm -f "$console"
}
trap 'stop_sim; exit 130' INT TERM
trap stop_sim EXIT

until grep -q 'playground up' "$console"; do
    if ! kill -0 "$sim_pid" 2>/dev/null; then
        echo 'sim-route: simulator did not come up:' >&2
        tail -5 "$console" >&2
        exit 3
    fi
    sleep 2
done
for _ in $(seq 1 40); do
    [[ $(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null) == active* ]] && break
    sleep 2
done

rc=0
# --offset=: a negative value as a separate word reads as an option.
python3 "$CLIENT" --out-dir "$OUT" --offset="$offset" || rc=$?
if (( rc )); then
    echo "sim-route: route FAILED (maps in $OUT)"
else
    echo 'sim-route: route passed'
fi
exit "$rc"
