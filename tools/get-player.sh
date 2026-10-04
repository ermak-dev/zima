#!/usr/bin/env bash
# Vendor the Player 3.1 tree into ros2/player as test infrastructure.
#
# The tree is NOT committed (74 MB of upstream git history): this script
# clones upstream release-3.1 at the exact commit we validated against,
# applies the two modern-GCC compile fixes from player-modern-gcc.patch,
# and builds the server binary in-tree (nothing is installed). The C
# headers and libplayerc are no longer used by anything: the Rust
# player-bridge core implements the wire protocol itself.
#
# Notes:
# - HAVE_GEOS is forced off to match the validated build: the libgeos-dev
#   package currently lacks the unversioned libgeos.so symlink, which
#   breaks linking when cmake auto-detects geos.
# - Requires: a C/C++ toolchain (gcc/g++, cmake, make).
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
ref=${1:-cb1e0b13}
dest=$root/ros2/player

if [[ -x $dest/server/player && -d $dest/.git ]]; then
    echo "ros2/player already vendored and built at $dest" >&2
    exit 0
fi
mkdir -p "$(dirname "$dest")"
git clone https://github.com/playerproject/player.git "$dest"
git -C "$dest" checkout "$ref"
git -C "$dest" apply "$root/tools/player-modern-gcc.patch"

# Build in-tree: the server binary is the ground truth for the
# player-bridge integration tests. Nothing is installed system-wide.
cd "$dest"
cmake -DCMAKE_BUILD_TYPE=Release -DHAVE_GEOS:BOOL=OFF . >/dev/null
make -j"$(nproc)" player >/dev/null
test -x server/player
echo "ros2/player ready (ref $ref)"
