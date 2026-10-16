#!/usr/bin/env bash
# FEATURE: robot panel --slam owns a bridge + SLAM + web mapping session.
# SPEC: no Nav2/RViz; wait for scan/map; stop command source, zero velocity,
# save with SLAM alive, terminate process groups, verify controller restore.
# All external operations use local fakes; never contact the robot.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/bin"
export ROBOT_TEST_LOG=$tmp/events ROBOT_TEST_DIR=$tmp
fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
has() { grep -Fq -- "$1" "$ROBOT_TEST_LOG" || fail "missing: $1"; }
absent() { if grep -Fq -- "$1" "$ROBOT_TEST_LOG"; then fail "unexpected: $1"; fi; }
line() { grep -Fn -- "$1" "$ROBOT_TEST_LOG" | head -1 | cut -d: -f1; }
cat >"$tmp/dispatch" <<'EOF'
printf 'mapping:%s\n' "$*" >>"$ROBOT_TEST_LOG"
exit "${FAKE_DISPATCH_RC:-0}"
EOF
: >"$ROBOT_TEST_LOG"
ROBOT_MAPPING_SCRIPT="$tmp/dispatch" bash "$root/bin/robot" panel --port 8099 --slam >"$tmp/out" 2>&1 \
    || fail "CLI rejected mapping mode: $(cat "$tmp/out")"
has 'mapping:--port 8099'
: >"$ROBOT_TEST_LOG"
ROBOT_MAPPING_SCRIPT="$tmp/dispatch" bash "$root/bin/robot" panel --slam
has 'mapping:--port 8085'
if ROBOT_MAPPING_SCRIPT="$tmp/dispatch" FAKE_DISPATCH_RC=7 bash "$root/bin/robot" panel --slam; then
    fail 'CLI swallowed wrapper failure'
fi
for args in '--slam --port' '--slam --port nope' '--slam --port 0' '--slam --port 65536' '--slam extra'; do
    : >"$ROBOT_TEST_LOG"
    read -r -a argv <<<"$args"
    if ROBOT_MAPPING_SCRIPT="$tmp/dispatch" bash "$root/bin/robot" panel "${argv[@]}" >"$tmp/out" 2>&1; then
        fail "accepted invalid args: $args"
    fi
    [[ ! -s $ROBOT_TEST_LOG ]] || fail 'invalid options launched wrapper'
done
bash "$root/bin/robot" --help | grep -q -- 'panel.*--slam' || fail 'help missing mapping mode'

: >"$tmp/ros.sh"
: >"$tmp/ws.sh"
cat >"$tmp/bin/pgrep" <<'EOF'
#!/bin/bash
[[ ${FAKE_BUSY:-0} == 1 ]]
EOF
cat >"$tmp/bin/robot" <<'EOF'
#!/bin/bash
printf 'robot:%s\n' "$*" >>"$ROBOT_TEST_LOG"
case $1 in
    bridge)
        touch "$ROBOT_TEST_DIR/bridge-up"
        trap 'printf "bridge:term\n" >>"$ROBOT_TEST_LOG"; exit 0' TERM
        if [[ ${FAKE_DIES:-} == bridge ]]; then sleep 0.5; exit 4; fi
        sleep 600 & wait $! ;;
    panel)
        trap 'printf "panel:term\n" >>"$ROBOT_TEST_LOG"; exit 0' TERM
        touch "$ROBOT_TEST_DIR/panel-up"
        if [[ ${FAKE_DIES:-} == signal ]]; then kill -TERM "$PPID"; fi
        if [[ ${FAKE_DIES:-panel} == panel ]]; then sleep 0.5; exit 0; fi
        if [[ ${FAKE_DIES:-} == panel-failure ]]; then sleep 0.5; exit 4; fi
        sleep 600 & wait $! ;;
    status)
        if [[ ${FAKE_RESTORE:-enabled} == enabled || -f $ROBOT_TEST_DIR/enabled ]]; then
            printf 'enabled\n'
        else printf 'disabled\n'; fi ;;
    enable)
        [[ ${FAKE_RESTORE:-} != fail ]] || exit 1
        touch "$ROBOT_TEST_DIR/enabled" ;;
esac
EOF
cat >"$tmp/bin/ros2" <<'EOF'
#!/bin/bash
if [[ $1 == topic && $2 == echo && $3 == /scan ]]; then
    for ((i=0; i<100; i++)); do
        [[ -f $ROBOT_TEST_DIR/bridge-up ]] && break
        sleep 0.01
    done
    [[ -f $ROBOT_TEST_DIR/bridge-up ]] || exit 1
fi
printf 'ros2:%s\n' "$*" >>"$ROBOT_TEST_LOG"
case "$1 $2" in
    'topic echo')
        if [[ $3 == /scan && ${FAKE_READY:-yes} == no-scan ]]; then exit 1; fi
        if [[ $3 == /map && ${FAKE_READY:-yes} == no-map ]]; then exit 1; fi
        if [[ $3 == /map ]]; then
            for ((i=0; i<100; i++)); do [[ -f $ROBOT_TEST_DIR/slam-up ]] && exit 0; sleep 0.01; done
            exit 1
        fi ;;
    'launch slam_toolbox')
        touch "$ROBOT_TEST_DIR/slam-up"
        trap 'printf "slam:term\n" >>"$ROBOT_TEST_LOG"; rm -f "$ROBOT_TEST_DIR/slam-up"; exit 0' TERM
        if [[ ${FAKE_DIES:-} == slam ]]; then
            until [[ -f $ROBOT_TEST_DIR/panel-up ]]; do sleep 0.01; done
            sleep 0.1; exit 4
        fi
        sleep 600 & wait $! ;;
    'run nav2_map_server')
        [[ -f $ROBOT_TEST_DIR/slam-up ]] || exit 8
        printf 'map:slam-alive\n' >>"$ROBOT_TEST_LOG"
        [[ ${FAKE_SAVE_FAIL:-0} != 1 ]] ;;
esac
EOF
chmod +x "$tmp/bin/"*
run_case() {
    : >"$ROBOT_TEST_LOG"
    rm -f "$tmp/bridge-up" "$tmp/slam-up" "$tmp/panel-up" "$tmp/enabled"
    rc=0
    env PATH="$tmp/bin:$PATH" ROBOT_BIN="$tmp/bin/robot" \
        ROBOT_ROS_SETUP="$tmp/ros.sh" ROBOT_WS_SETUP="$tmp/ws.sh" \
        ROBOT_SCAN_TRIES=2 ROBOT_MAP_TRIES=2 ROBOT_RESTORE_TRIES=1 \
        ROBOT_RUN_LOG="$tmp/run.log" ROBOT_MAP_OUT="$tmp/maps/test map" \
        "$@" timeout 20 bash "$root/scripts/run-mapping.sh" --port 8099 >"$tmp/out" 2>&1 || rc=$?
    [[ $rc != 124 && $rc != 137 ]] || fail "wrapper hung: $(cat "$tmp/out")"
}
run_case
[[ $rc == 0 ]] || fail "happy path rc=$rc: $(cat "$tmp/out")"
has 'robot:bridge'
has 'ros2:launch slam_toolbox online_sync_launch.py'
has "slam_params_file:=$root/config/slam_online.yaml use_sim_time:=false"
has 'ros2:topic echo /map --once --qos-durability transient_local'
has 'robot:panel --port 8099'
has "ros2:run nav2_map_server map_saver_cli -f $tmp/maps/test map --ros-args -p use_sim_time:=false"
for step in 'robot:bridge' 'ros2:topic echo /scan' 'ros2:launch slam_toolbox' 'robot:panel' 'ros2:topic pub -1 /cmd_vel' 'map:slam-alive' 'bridge:term' 'robot:status'; do
    n=$(line "$step")
    (( n > ${prev:-0} )) || fail "incorrect order: $step"
    prev=$n
done
(( $(line 'map:slam-alive') < $(line 'slam:term') )) || fail 'saved after SLAM stopped'
absent 'nav2_bringup'; absent 'rviz'; absent 'teleop'; absent 'robot:enable'

for component in bridge slam; do
    run_case FAKE_DIES="$component"
    [[ $rc == 3 ]] || fail "$component death rc=$rc, want 3: $(cat "$tmp/out")"
    has 'robot:status'
    if [[ $component == slam ]]; then
        has 'panel:term'
        (( $(line 'panel:term') < $(line 'ros2:topic pub -1 /cmd_vel') )) || fail 'panel can publish after cleanup zero'
    fi
done
for ready in no-scan no-map; do
    run_case FAKE_READY="$ready"
    [[ $rc == 3 ]] || fail "$ready rc=$rc: $(cat "$tmp/out")"
    absent 'robot:panel'; has 'bridge:term'; has 'robot:status'
    if [[ $ready == no-scan ]]; then absent 'launch slam_toolbox'; fi
done
run_case FAKE_BUSY=1
[[ $rc == 2 ]] || fail "busy session rc=$rc"
absent 'robot:bridge'; absent 'robot:status'
run_case FAKE_RESTORE=disabled
[[ $rc == 0 ]] || fail 'fallback restore failed'
has 'robot:enable'
run_case FAKE_RESTORE=fail
[[ $rc == 2 ]] || fail 'failed restoration not reported'
has 'robot:enable'
run_case FAKE_SAVE_FAIL=1
has 'bridge:term'; has 'robot:status'
grep -q 'WARNING.*map.*not saved' "$tmp/out" || fail 'map failure not visible'
run_case FAKE_DIES=panel-failure
[[ $rc == 3 ]] || fail 'panel failure not reported'
has 'bridge:term'; has 'robot:status'
run_case FAKE_DIES=signal
[[ $rc == 143 ]] || fail "TERM exit rc=$rc, want 143"
for step in 'panel:term' 'ros2:topic pub -1 /cmd_vel' 'map:slam-alive' 'bridge:term' 'robot:status'; do
    n=$(line "$step")
    (( n > ${signal_prev:-0} )) || fail "TERM cleanup out of order: $step"
    signal_prev=$n
done
# RESULT: IMPLEMENTED — separate mapping wrapper and CLI routing.
printf 'robot mapping tests passed\n'
