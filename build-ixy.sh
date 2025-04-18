#!/bin/bash
set -e
ARCH=$1
PLATFORM=$2

# Build
docker run --network host --platform=linux/$PLATFORM --rm -u $(id -u):$(id -g) -v $(pwd):/working -e RUNTIME_ARCH=${ARCH} ixy-builder.$ARCH $(pwd)

mkdir -p $(pwd)/output
