#!/bin/bash
# Shared helpers for the session wrappers (run-rviz.sh, run-sim.sh).
# Sourced after the ROS setup; no set -u here (ROS scripts reference
# unbound vars), callers keep their own flags.

# Write a robot_state_publisher params file for the given URDF.
# A multi-line robot_description cannot go through `-p` on the command
# line, so it must travel via a params file. Prints nothing; the caller
# owns the temp file path passed in $2. <urdf_path> <params_out> <sim_time>
rsp_params_write() {
    python3 - "$1" "$2" "$3" <<'PY'
import json, sys
urdf = open(sys.argv[1]).read()
open(sys.argv[2], 'w').write(
    'robot_state_publisher:\n  ros__parameters:\n'
    '    use_sim_time: ' + sys.argv[3] + '\n'
    '    robot_description: ' + json.dumps(urdf) + '\n'
)
PY
}

# Merge Nav2 params overlays, in order, into a base file (launches take a
# single params file). Within each node, an overlay replaces whole keys of
# ros__parameters. PyYAML is in the ROS desktop install; without it a
# plain concatenation still works because overlays redefine whole
# top-level sections only. <out> <base> <overlay>...
nav2_params_merge() {
    local out=$1 base=$2
    shift 2
    python3 - "$out" "$base" "$@" <<'PY' || cat "$base" "$@" >"$out"
import sys
try:
    import yaml
except ImportError:
    sys.exit(10)
out, base, overlays = sys.argv[1], sys.argv[2], sys.argv[3:]
b = yaml.safe_load(open(base)) or {}
for overlay in overlays:
    o = yaml.safe_load(open(overlay)) or {}
    for k, v in o.items():
        if k in b and isinstance(b.get(k), dict) and isinstance(v, dict):
            for k2, v2 in v.items():
                if isinstance(v2, dict) and isinstance(b[k].get(k2), dict):
                    b[k][k2].update(v2)
                else:
                    b[k][k2] = v2
        else:
            b[k] = v
yaml.safe_dump(b, open(out, 'w'), sort_keys=False)
PY
}

# Name the ROS session already running on this host, if any; returns 1
# when there is none. Only one may run: two simulators fight over /clock
# and node names, and next to the live bridge a simulator's Nav2 /cmd_vel
# would be forwarded to the real wheels. The [x] brackets keep pgrep from
# matching a command line that carries this very pattern (a `bash -c`).
running_ros_session() {
    if pgrep -f '[p]layer_to_ros2' >/dev/null; then
        echo 'a live bridge session (player_to_ros2)'
    elif pgrep -f '[t]b3_simulation_launch' >/dev/null; then
        echo 'a simulator session (robot run --sim)'
    else
        return 1
    fi
}

# Stop a list of session components started with setsid (each $! is a
# group leader): TERM to the group, bounded reap, then KILL. Wedged
# children (ssh over dead Wi-Fi) cannot stall the teardown.
stop_components() {
    local pid waited
    for pid in "$@"; do
        # SIGTERM, not SIGINT: children backgrounded from a script inherit
        # SIG_IGN for SIGINT, so INT would not stop them.
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    for pid in "$@"; do
        waited=0
        # `wait` cannot be timed out; poll instead.
        while kill -0 "$pid" 2>/dev/null && (( waited < 15 )); do
            sleep 1; waited=$((waited + 1))
        done
        kill -KILL "$pid" 2>/dev/null || true
    done
}
