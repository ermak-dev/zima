#!/usr/bin/env bash
# FEATURE: scripts/sim-route.sh — drive a fixed goal route in the headless
# playground and report pass/fail.
# SPEC: start the sim wrapper headless (with --slam when asked), wait for
# Nav2, run the route client with the right goal offset, always stop the
# sim, exit with the route's verdict; a sim that never comes up exits 3
# without driving. Fakes for the sim wrapper, ros2 and the route client.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/bin"
log=$tmp/log

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
assert_log() { grep -Fq -- "$1" "$log" || fail "expected log: $1 (log: $(tr '\n' '|' <"$log"))"; }

: > "$tmp/ros-setup.sh"

# Fake sim wrapper: records how it was started, announces the playground
# (or fails to), and runs until terminated.
cat > "$tmp/fake-sim.sh" <<'EOF'
#!/usr/bin/env bash
printf 'sim:args=%s headless=%s rviz=%s\n' "$*" "${ROBOT_SIM_HEADLESS:-}" "${ROBOT_SIM_RVIZ:-}" >> "$ROUTE_TEST_LOG"
trap 'printf "sim:term\n" >> "$ROUTE_TEST_LOG"; exit 0' TERM
if [[ ${ROUTE_TEST_SIM_DOWN:-0} == 1 ]]; then
    printf 'robot sim: simulator did not come up\n'
    exit 3
fi
printf '[robot sim] playground up\n'
sleep 600 &
wait $!
EOF

cat > "$tmp/bin/ros2" <<'EOF'
#!/usr/bin/env bash
[[ "$1 $2" == 'lifecycle get' ]] && { printf 'active [3]\n'; exit 0; }
exit 0
EOF

# Parses its arguments like ros2/route_check.py (argparse): a negative
# offset passed as a separate word must not be mistaken for an option.
cat > "$tmp/fake-client.py" <<'EOF'
import argparse, os, sys
p = argparse.ArgumentParser()
p.add_argument('--offset', default='0,0')
p.add_argument('--out-dir', default='/tmp/robot-route')
a = p.parse_args()
with open(os.environ['ROUTE_TEST_LOG'], 'a') as f:
    f.write(f"client:out={a.out_dir} offset={a.offset}\n")
sys.exit(int(os.environ.get('ROUTE_TEST_CLIENT_RC', '0')))
EOF
chmod +x "$tmp/fake-sim.sh" "$tmp/bin/ros2"

run_route() {
    : > "$log"
    env PATH="$tmp/bin:$PATH" ROUTE_TEST_LOG="$log" \
        ROBOT_ROS_SETUP="$tmp/ros-setup.sh" \
        ROBOT_ROUTE_SIM_SCRIPT="$tmp/fake-sim.sh" \
        ROBOT_ROUTE_CLIENT="$tmp/fake-client.py" \
        ROBOT_ROUTE_OUT="$tmp/out" \
        "$@" bash "$root/scripts/sim-route.sh" "${ROUTE_ARGS[@]}" >"$tmp/stdout" 2>&1
}

# --- AMCL playground: prebuilt map, goals offset to the spawn point ---
ROUTE_ARGS=()
run_route || fail "route run failed: $(cat "$tmp/stdout")"
assert_log 'sim:args= headless=True rviz=False'
assert_log "client:out=$tmp/out offset=-2.0,-0.5"
assert_log 'sim:term'
grep -q 'route passed' "$tmp/stdout" || fail 'missing pass verdict'

# --- SLAM playground: the map frame starts at the robot ---
ROUTE_ARGS=(--slam)
run_route || fail "slam route run failed: $(cat "$tmp/stdout")"
assert_log 'sim:args=--slam headless=True rviz=False'
assert_log "client:out=$tmp/out offset=0,0"
assert_log 'sim:term'

# --- a failed goal fails the run, and the sim is still stopped ---
ROUTE_ARGS=()
status=0
run_route ROUTE_TEST_CLIENT_RC=1 || status=$?
[[ $status == 1 ]] || fail "failed route exited $status, want 1"
assert_log 'sim:term'
grep -q 'route FAILED' "$tmp/stdout" || fail 'missing fail verdict'

# --- the sim never comes up: exit 3, nothing driven ---
status=0
run_route ROUTE_TEST_SIM_DOWN=1 || status=$?
[[ $status == 3 ]] || fail "sim-down run exited $status, want 3"
if grep -q '^client:' "$log"; then fail 'route driven without a simulator'; fi

# --- unknown option ---
ROUTE_ARGS=(--bogus)
status=0
run_route || status=$?
[[ $status == 2 ]] || fail "unknown option exited $status, want 2"
if grep -q '^sim:' "$log"; then fail 'sim started on a bad option'; fi

printf 'robot route tests passed\n'
