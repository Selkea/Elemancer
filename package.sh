#!/usr/bin/env bash
# Build a portable, shareable Windows package of Elemancer: the executable, its
# shaders, and the MSYS2 UCRT64 runtime DLLs it needs, zipped up. The recipient
# does not need MSYS2 -- they just unzip and run elemancer.exe.
#
# Run from an MSYS2 UCRT64 shell (so ldd and the DLLs are on PATH):
#   ./package.sh
#
# The exe finds its shaders next to itself (see resolveAssetDir in main.cpp), so
# the folder is relocatable.
set -euo pipefail

root="$(cd "$(dirname "$0")" && pwd)"
dist="$root/dist/Elemancer"
# A dedicated build dir with ELEMANCER_NATIVE=OFF, so the shared zip is portable
# (baseline x86-64, no -march=native that would crash an older CPU) without
# disturbing the dev build/, which stays tuned for this machine.
pkgbuild="$root/build-pkg"

# Reconfigure so the baked-in version (git describe, resolved at configure time)
# matches the current tag/commit, then build.
cmake -S "$root" -B "$pkgbuild" -G Ninja -DCMAKE_BUILD_TYPE=Release -DELEMANCER_NATIVE=OFF
cmake --build "$pkgbuild"

rm -rf "$root/dist"
mkdir -p "$dist"
cp "$pkgbuild/elemancer.exe" "$dist/"
cp -r "$root/shaders" "$dist/"

# Copy exactly the non-system DLLs the exe links, straight from ldd, so this
# stays correct if the dependencies ever change.
ldd "$pkgbuild/elemancer.exe" | awk '/ucrt64/ {print $3}' | while read -r dll; do
  cp "$dll" "$dist/"
done

( cd "$root/dist" &&
  powershell -NoProfile -Command \
    "Compress-Archive -Path Elemancer -DestinationPath Elemancer-win64.zip -Force" )

echo "Packaged: $root/dist/Elemancer-win64.zip"
ls -1 "$dist"
