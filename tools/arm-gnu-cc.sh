#!/usr/bin/env bash
# C compiler and Rust linker for the robot's ARMv7 glibc 2.19 ABI.
set -euo pipefail
command -v zig >/dev/null || { printf 'zig is required for ARM compilation\n' >&2; exit 1; }
flags=(-target arm-linux-gnueabihf.2.19 -mcpu=cortex_a7 -fno-sanitize=all)
for arg in "$@"; do
    if [[ $arg == -c ]]; then
        exec zig cc "${flags[@]}" "$@"
    fi
done
script=$(mktemp)
trap 'rm -f -- "$script"' EXIT
# Same layout workaround as bin/audio/build.sh: glibc 2.19 needs dynamic
# and PLT relocations adjacent, without .ARM.exidx inserted between them.
printf 'SECTIONS { .ARM.exidx : { *(.ARM.exidx*) } } INSERT AFTER .rel.plt;\n' > "$script"
zig cc "${flags[@]}" "$@" -Wl,-T,"$script"
