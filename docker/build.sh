#!/bin/ash
set -e

ARCH=$(uname -m)

cd /working

cmake -B build.$ARCH -GNinja
cmake --build build.$ARCH

## Copy ixy executables
mkdir -p output
for f in build.$ARCH/ixy*; do cp "$f" output/$(basename "$f").$ARCH; done
