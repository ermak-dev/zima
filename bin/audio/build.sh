#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
native=0
testing=0
output=
while (( $# )); do
    case $1 in
        --native) native=1; shift ;;
        --testing) testing=1; shift ;;
        --output) (( $# >= 2 )) || exit 2; output=$2; shift 2 ;;
        *) printf 'usage: build.sh [--native] [--testing] [--output PATH]\n' >&2; exit 2 ;;
    esac
done
if (( testing && !native )); then
    printf 'test backend is available only for native offline tests\n' >&2
    exit 2
fi
if [[ -z $output ]]; then
    if (( native )); then output=$root/crates/target/audio/robot-audio-host
    else output=$root/crates/target/armv7-unknown-linux-musleabihf/release/robot-audio; fi
fi
mkdir -p "$(dirname "$output")"
flags=(-O2 -Wall -Wextra -Werror -std=c11)
(( testing )) && flags+=(-DROBOT_AUDIO_TESTING)
if (( native )); then
    "${CC:-cc}" "${flags[@]}" "$root/bin/audio/robot_audio.c" -o "$output" -ldl -lm
else
    command -v zig >/dev/null || { printf 'zig is required for ARM compilation\n' >&2; exit 1; }
    script=$(mktemp)
    trap 'rm -f -- "$script"' EXIT
    # glibc 2.19 requires dynamic and PLT relocations to remain adjacent.
    printf 'SECTIONS { .ARM.exidx : { *(.ARM.exidx*) } } INSERT AFTER .rel.plt;\n' > "$script"
    zig cc -target arm-linux-gnueabihf.2.19 -mcpu=cortex_a7 "${flags[@]}" \
        -fno-sanitize=all "$root/bin/audio/robot_audio.c" -Wl,-T,"$script" \
        -o "$output" -ldl -lm
fi
printf '%s\n' "$output"
