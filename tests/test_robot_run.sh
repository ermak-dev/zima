#!/usr/bin/env bash
# FEATURE: `robot run` session wrapper (scripts/run-rviz.sh).
# SPEC: start the supervised bridge, wait for /scan, open RViz and teleop;
# on exit stop the robot (zero /cmd_vel before the bridge dies) and verify
# the stock controller came back. Everything runs against local fakes.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/fake-bin"
export ROBOT_TEST_LOG=$tmp/log ROBOT_TEST_DIR=$tmp
: > "$ROBOT_TEST_LOG"

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
assert_log() { grep -Fq -- "$1" "$ROBOT_TEST_LOG" || fail "expected log: $1"; }
line_of() { grep -Fn -- "$1" "$ROBOT_TEST_LOG" | head -1 | cut -d: -f1; }

cat > "$tmp/ros-setup.sh" < /dev/null
cat > "$tmp/ws-setup.sh" < /dev/null

cat > "$tmp/fake-bin/robot" <<'EOF'
#!/usr/bin/env bash
printf 'robot:%s %s\n' "$1" "${*:2}" >> "$ROBOT_TEST_LOG"
case $1 in
    bridge)
        [[ ${ROBOT_FAKE_SCAN:-1} == 1 ]] && touch "$ROBOT_TEST_DIR/scan-ready"
        trap 'printf "robot-bridge:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
        sleep 600 &
        wait $!
        ;;
    status)
        printf '%s\n' "${ROBOT_FAKE_STATUS:-enabled}"
        ;;
    enable)
        # Simulates the CLI restoring the stock controller: subsequent
        # status checks then report enabled.
        printf '%s\n' "${ROBOT_FAKE_STATUS:-enabled}" > "$ROBOT_TEST_DIR/enable-result"
        [[ ${ROBOT_FAKE_ENABLE_FAIL:-0} != 1 ]]
        ;;
esac
EOF

cat > "$tmp/fake-bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf 'ros2:%s\n' "$*" >> "$ROBOT_TEST_LOG"
if [[ $1 == run && $2 == teleop_twist_keyboard ]]; then
    trap 'printf "teleop:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
    if [[ -n ${ROBOT_FAKE_TELEOP_SECS:-} ]]; then
        sleep "$ROBOT_FAKE_TELEOP_SECS"
    else
        sleep 600 &
        wait $!
    fi
elif [[ $1 == topic && $2 == echo ]]; then
    [[ -e $ROBOT_TEST_DIR/scan-ready ]] || exit 1
fi
exit 0
EOF

cat > "$tmp/fake-bin/rviz2" <<'EOF'
#!/usr/bin/env bash
printf 'rviz2:%s\n' "$*" >> "$ROBOT_TEST_LOG"
sleep "${ROBOT_FAKE_RVIZ_SECS:-3}"
exit 0
EOF

cat > "$tmp/fake-bin/gnome-terminal" <<'EOF'
#!/usr/bin/env bash
printf 'gnome-terminal:%s\n' "$*" >> "$ROBOT_TEST_LOG"
while [[ $# -gt 0 && $1 != -- ]]; do shift; done
shift
setsid bash -c "$*" >/dev/null 2>&1 &
sleep 600 &
wait $!
EOF

chmod +x "$tmp/fake-bin/robot" "$tmp/fake-bin/ros2" "$tmp/fake-bin/rviz2" "$tmp/fake-bin/gnome-terminal"

run_session() {
    env -u DISPLAY \
        PATH="$tmp/fake-bin:$PATH" \
        ROBOT_TEST_LOG="$ROBOT_TEST_LOG" ROBOT_TEST_DIR="$tmp" \
        ROBOT_ROS_SETUP="$tmp/ros-setup.sh" ROBOT_WS_SETUP="$tmp/ws-setup.sh" \
        ROBOT_BIN="$tmp/fake-bin/robot" \
        ROBOT_RVIZ_CONFIG="$root/config/rviz_teleop.rviz" \
        ROBOT_RUN_LOG="$tmp/run.log" \
        ROBOT_SCAN_TRIES=2 ROBOT_RESTORE_TRIES=2 \
        "$@"
}

# --- console session: teleop exits, everything tears down in order ---
run_session ROBOT_FAKE_TELEOP_SECS=1 \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 \
    || fail "console session exited nonzero: $(cat "$tmp/out")"
assert_log 'robot:bridge'
assert_log 'ros2:topic echo /scan --once'
assert_log 'rviz2:-d'
assert_log 'ros2:run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4'
assert_log 'ros2:topic pub -1 /cmd_vel geometry_msgs/msg/Twist {linear: {x: 0.0}, angular: {z: 0.0}}'
assert_log 'robot:status'
assert_log 'robot-bridge:term'
zero_line=$(line_of 'ros2:topic pub -1 /cmd_vel')
term_line=$(line_of 'robot-bridge:term')
[[ -n $zero_line && -n $term_line && $zero_line -lt $term_line ]] \
    || fail 'zero /cmd_vel was not published before the bridge was stopped'
grep -q '\[robot run\] done' "$tmp/out" || fail 'session did not report teardown done'
grep -q 'lidar stream ok' "$tmp/out" || fail 'session did not confirm the lidar stream'

# --- slow scan (slow Wi-Fi): keep waiting while the bridge lives ---
# The scan window must outlast slow ssh phases (uploads, disable retries);
# only a dead bridge ends the wait early.
cat > "$tmp/fake-bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf 'ros2:%s\n' "$*" >> "$ROBOT_TEST_LOG"
if [[ $1 == run && $2 == teleop_twist_keyboard ]]; then
    trap 'printf "teleop:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
    if [[ -n ${ROBOT_FAKE_TELEOP_SECS:-} ]]; then
        sleep "$ROBOT_FAKE_TELEOP_SECS"
    else
        sleep 600 &
        wait $!
    fi
elif [[ $1 == topic && $2 == echo ]]; then
    [[ -e $ROBOT_TEST_DIR/scan-ready ]] && exit 0
    if [[ -e $ROBOT_TEST_DIR/bridge-up ]]; then
        up=$(stat -c %Y "$ROBOT_TEST_DIR/bridge-up")
        now=$(date +%s)
        if (( now - up >= ${ROBOT_FAKE_SCAN_DELAY_S:-0} )); then exit 0; fi
    fi
    # The real `ros2 topic echo --once` waits out its timeout when no
    # message arrives; emulate that so the wrapper's probe loop is paced.
    sleep 2
    exit 1
fi
exit 0
EOF
cat > "$tmp/fake-bin/robot" <<'EOF'
#!/usr/bin/env bash
printf 'robot:%s %s\n' "$1" "${*:2}" >> "$ROBOT_TEST_LOG"
case $1 in
    bridge)
        touch "$ROBOT_TEST_DIR/bridge-up"
        trap 'printf "robot-bridge:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
        sleep 600 &
        wait $!
        ;;
    status)
        printf '%s\n' "${ROBOT_FAKE_STATUS:-enabled}"
        ;;
    enable)
        [[ ${ROBOT_FAKE_ENABLE_FAIL:-0} != 1 ]]
        ;;
esac
EOF
chmod +x "$tmp/fake-bin/ros2" "$tmp/fake-bin/robot"
rm -f "$tmp/scan-ready" "$tmp/bridge-up"
: > "$ROBOT_TEST_LOG"
status=0
run_session ROBOT_SCAN_TRIES=30 ROBOT_FAKE_SCAN_DELAY_S=8 ROBOT_FAKE_TELEOP_SECS=1 \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 || status=$?
[[ $status == 0 ]] || fail "slow-scan session exited $status: $(tail -3 "$tmp/out")"
grep -q 'lidar stream ok' "$tmp/out" || fail 'slow scan not waited for'

# --- graphical session: closing RViz ends it; detached teleop is stopped ---
: > "$ROBOT_TEST_LOG"
run_session DISPLAY=:99 ROBOT_FAKE_RVIZ_SECS=2 \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 \
    || fail "graphical session exited nonzero: $(cat "$tmp/out")"
sleep 1
assert_log 'gnome-terminal:--wait'
assert_log 'speed:=0.1'
assert_log 'turn:=0.4'
assert_log 'teleop:term'
assert_log 'robot-bridge:term'
grep -q 'teleop window open' "$tmp/out" || fail 'teleop window not confirmed open'

# --- no lidar stream: abort before teleop/RViz, still roll back ---
# The fake bridge announces its own failure; the wrapper must surface the
# bridge's last words on the console, not only inside the log file.
cat > "$tmp/fake-bin/robot" <<'EOF'
#!/usr/bin/env bash
printf 'robot:%s %s\n' "$1" "${*:2}" >> "$ROBOT_TEST_LOG"
case $1 in
    bridge)
        if [[ ${ROBOT_FAKE_SCAN:-1} != 1 ]]; then
            echo 'fake-bridge: player unreachable' >&2
        fi
        [[ ${ROBOT_FAKE_SCAN:-1} == 1 ]] && touch "$ROBOT_TEST_DIR/scan-ready"
        trap 'printf "robot-bridge:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
        sleep 600 &
        wait $!
        ;;
    status)
        printf '%s\n' "${ROBOT_FAKE_STATUS:-enabled}"
        ;;
esac
EOF
chmod +x "$tmp/fake-bin/robot"
: > "$ROBOT_TEST_LOG"
rm -f "$tmp/scan-ready" "$tmp/bridge-up"
status=0
run_session ROBOT_FAKE_SCAN=0 \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 || status=$?
[[ $status == 3 ]] || fail "no-scan session exited $status, want 3"
if grep -Fq 'ros2:run teleop' "$ROBOT_TEST_LOG"; then fail 'teleop started without a lidar stream'; fi
if grep -Fq 'rviz2:' "$ROBOT_TEST_LOG"; then fail 'rviz started without a lidar stream'; fi
assert_log 'robot-bridge:term'
grep -q 'no /scan' "$tmp/out" || fail 'no-scan message missing on console'
grep -q 'fake-bridge: player unreachable' "$tmp/out" || fail 'bridge failure output not surfaced on console'
grep -q 'fake-bridge: player unreachable' "$tmp/run.log" || fail 'bridge failure output missing from the log'

# --- controller did not return on its own: wrapper must run robot enable ---
# The supervisor only rolls back after ACTIVATE; a bridge killed before
# activation leaves the controller down, so the teardown restores it
# itself before declaring failure.
cat > "$tmp/fake-bin/robot" <<'EOF'
#!/usr/bin/env bash
printf 'robot:%s %s\n' "$1" "${*:2}" >> "$ROBOT_TEST_LOG"
case $1 in
    bridge)
        [[ ${ROBOT_FAKE_SCAN:-1} == 1 ]] && touch "$ROBOT_TEST_DIR/scan-ready"
        trap 'printf "robot-bridge:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
        sleep 600 &
        wait $!
        ;;
    status)
        if [[ -e $ROBOT_TEST_DIR/enabled-by-wrapper ]]; then
            printf 'enabled\n'
        else
            printf '%s\n' "${ROBOT_FAKE_STATUS:-enabled}"
        fi
        ;;
    enable)
        printf 'enable:%s\n' >> "$ROBOT_TEST_LOG"
        if [[ ${ROBOT_FAKE_ENABLE_FAIL:-0} == 1 ]]; then
            exit 1
        fi
        touch "$ROBOT_TEST_DIR/enabled-by-wrapper"
        ;;
esac
EOF
chmod +x "$tmp/fake-bin/robot"
: > "$ROBOT_TEST_LOG"
rm -f "$tmp/scan-ready" "$tmp/enabled-by-wrapper"
touch "$tmp/scan-ready"
status=0
run_session ROBOT_FAKE_TELEOP_SECS=1 ROBOT_FAKE_STATUS=disabled \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 || status=$?
[[ $status == 0 ]] || fail "self-restore session exited $status: $(tail -3 "$tmp/out")"
grep -q 'enable:' "$ROBOT_TEST_LOG" || fail 'wrapper did not run robot enable when the controller was down'
grep -q 'stock controller restored via robot enable' "$tmp/out" || fail 'self-restore not reported'

# --- robot enable itself fails: warning and nonzero exit ---
: > "$ROBOT_TEST_LOG"
rm -f "$tmp/enabled-by-wrapper"
status=0
run_session ROBOT_FAKE_TELEOP_SECS=1 ROBOT_FAKE_STATUS=disabled ROBOT_FAKE_ENABLE_FAIL=1 \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 || status=$?
[[ $status == 2 ]] || fail "enable-failed session exited $status, want 2"
grep -q 'stock controller did not return' "$tmp/out" || fail 'missing rollback warning'

# --- refuses to start on top of an existing bridge session ---
: > "$ROBOT_TEST_LOG"
touch "$tmp/scan-ready"
# Fake a live bridge: pgrep -f matches the player_to_ros2 command line.
setsid bash -c 'exec -a player_to_ros2 sleep 600' >/dev/null 2>&1 &
stray=$!
status=0
run_session ROBOT_FAKE_TELEOP_SECS=1 \
    bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 || status=$?
kill -TERM "$stray" 2>/dev/null || true
wait "$stray" 2>/dev/null || true
[[ $status == 2 ]] || fail "bridge-running session exited $status, want 2"
if grep -Fq 'robot:bridge' "$ROBOT_TEST_LOG"; then fail 'second bridge started on top of an existing one'; fi

# --- double Ctrl-C during teardown must not abort the cleanup ---
# The teardown takes time (status polls, robot enable); a second Ctrl-C
# must not leave the wrapper before the bridge is stopped and the stock
# controller state is settled.
cat > "$tmp/fake-bin/robot" <<'EOF'
#!/usr/bin/env bash
printf 'robot:%s %s\n' "$1" "${*:2}" >> "$ROBOT_TEST_LOG"
case $1 in
    bridge)
        touch "$ROBOT_TEST_DIR/scan-ready"
        trap 'printf "robot-bridge:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
        sleep 600 &
        wait $!
        ;;
    status)
        sleep "${ROBOT_FAKE_SLOW_TEARDOWN:-0}"
        printf 'enabled\n'
        ;;
    enable) exit 0 ;;
esac
EOF
chmod +x "$tmp/fake-bin/robot"
: > "$ROBOT_TEST_LOG"; rm -f "$tmp/scan-ready" "$tmp/out"
run_session ROBOT_FAKE_TELEOP_SECS=1 ROBOT_FAKE_SLOW_TEARDOWN=2 \
    setsid bash "$root/scripts/run-rviz.sh" >"$tmp/out" 2>&1 &
subshell=$!
for _ in 1 2 3 4 5; do
    wpid=$(ps -eo pid,ppid,cmd | awk -v p="$subshell" '$2==p && /run-rviz/ {print $1; exit}')
    [[ -n ${wpid:-} ]] && break
    sleep 1
done
until [[ -f $tmp/out ]] && grep -q 'playground\|teardown\|teleop in this console' "$tmp/out"; do sleep 1; done
until grep -q 'teardown' "$tmp/out"; do sleep 1; done
# teardown now sleeps in `robot status`; hammer the group with a second INT
kill -INT -- "-$wpid" 2>/dev/null || kill -INT "$wpid" 2>/dev/null || true
wait "$subshell" 2>/dev/null || true
assert_log 'robot-bridge:term'
grep -q '\[robot run\] done' "$tmp/out" || fail 'teardown aborted by second Ctrl-C'

printf 'robot run session tests passed\n'
