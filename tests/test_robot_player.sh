#!/usr/bin/env bash
# FEATURE: Direct Player odometry and bounded wheel movement without cleaning commands.
# SPEC: Reject invalid motion before contacting the robot; compile against local libplayerc.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
player=${ROBOT_PLAYER_ROOT:-$HOME/Projects/robot/ros2/player}
bin=$(mktemp)
trap 'rm -f "$bin"' EXIT
libs="$player/client_libs/libplayerc:$player/libplayerinterface:$player/libplayerwkb:$player/libplayerjpeg:$player/libplayercommon"
gcc -Wall -Wextra -Werror -I"$player/replace" -I"$player" -I"$player/client_libs/libplayerc" -I"$player/libplayerinterface" "$root/bin/controller/player.c" -o "$bin" -L"$player/client_libs/libplayerc" -L"$player/libplayerinterface" -L"$player/libplayerwkb" -L"$player/libplayerjpeg" -L"$player/libplayercommon" -lplayerc -lplayerinterface -lplayerwkb -lplayerjpeg -lplayercommon -lm -lz
export LD_LIBRARY_PATH="$libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for args in 'move sideways 1' 'move forward 0' 'move forward 4' 'move forward nan' 'move backward -1' 'move forward 1 extra' 'position extra' 'fan extra' 'stop extra'; do
    read -r -a argv <<< "$args"
    if timeout 2 "$bin" 127.0.0.1 "${argv[@]}" >/dev/null 2>&1; then
        echo "invalid Player request succeeded: $args" >&2; exit 1
    fi
done
printf 'Player validation tests passed\n'
