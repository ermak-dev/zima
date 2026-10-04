#!/usr/bin/env bash
# FEATURE: Owner-side change-device-state codec and request loop on fakes.
# SPEC: Wire format, finite timeout, foreign-packet bound, transport failures.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
bin=$(mktemp)
trap 'rm -f "$bin"' EXIT
gcc -Wall -Wextra -Werror -I"$root/bin/owner" "$root/tests/owner/mask_test.c" \
    "$root/bin/owner/mask.c" -o "$bin"
"$bin"
printf 'owner mask tests passed\n'
