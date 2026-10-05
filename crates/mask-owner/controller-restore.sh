#!/usr/bin/env bash
# Restore the stock controller and its WatchDoge supervision.
# Runs ON the robot (invoked by robot-supervisor after the rollback).
# This is the enable half of the former bin/controller/enable with the
# ssh wrapper removed; the logic is copied verbatim and is idempotent.
set -euo pipefail

list=${ROBOT_PROCESS_LIST:-/opt/rockrobo/watchdog/ProcessList.conf}
backup=${ROBOT_BACKUP:-/mnt/data/rockrobo/ProcessList.conf.robot-backup}
entry='RoboController,setsid RoboController&,1,3,0'
marker="# ROBOT_DISABLED: $entry"
tmp=
disabled_copy=
changed=0
log() { echo "controller-restore: $*" >&2; }

cleanup() {
    local status=$?
    [[ -z $tmp ]] || rm -f "$tmp"
    if (( status && changed )) && [[ -n $disabled_copy && -f $disabled_copy ]]; then
        cp -p "$disabled_copy" "$list"
        initctl restart rrwatchdoge >&2 || true
    fi
    [[ -z $disabled_copy ]] || rm -f "$disabled_copy"
}
trap cleanup EXIT

if grep -Fqx "$entry" "$list"; then
    if [[ -e $backup ]]; then
        cmp -s "$list" "$backup" || { log 'recovery copy differs from enabled config'; exit 1; }
        initctl restart rrwatchdoge
        for _ in $(seq 1 30); do
            if pgrep -x RoboController >/dev/null; then break; fi
            sleep 1
        done
    fi
    pgrep -x RoboController >/dev/null && pgrep -x WatchDoge >/dev/null || {
        log 'entry enabled but controller/watchdog is not running'; exit 1;
    }
    rm -f "$backup"
    log 'controller enabled'
    exit 0
fi
[[ -f $backup && $(grep -Fxc "$entry" "$backup") == 1 && $(grep -Fxc "$marker" "$list") == 1 ]] || {
    log 'no valid recovery copy and disabled entry; refusing to guess'; exit 1;
}
disabled_copy=$(mktemp "${list}.disabled.XXXXXX")
cp -p "$list" "$disabled_copy"
tmp=$(mktemp "${list}.robot.XXXXXX")
cp -p "$list" "$tmp"
awk -v entry="$entry" -v marker="$marker" '{ if ($0 == marker) print entry; else print }' "$list" > "$tmp"
changed=1
mv -f "$tmp" "$list"
tmp=
initctl restart rrwatchdoge
# The old WatchDoge may take seconds to die after the restart; retry the
# readiness check instead of failing mid-respawn.
for _ in $(seq 1 30); do
    pgrep -x WatchDoge >/dev/null && break
    sleep 1
done
pgrep -x WatchDoge >/dev/null || { log 'watchdog failed to start'; exit 1; }
for _ in $(seq 1 30); do
    if pgrep -x RoboController >/dev/null; then break; fi
    sleep 1
done
pgrep -x RoboController >/dev/null || { log 'RoboController did not restart'; exit 1; }
rm -f "$backup"
changed=0
log 'controller enabled and watched'
