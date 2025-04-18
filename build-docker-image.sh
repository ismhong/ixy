#!/bin/bash
set -e
#CLEAN_BUILD="--no-cache"
ARCH=$1
PLATFORM=$2


pushd docker
DOCKER_BUILDKIT=1 docker build $CLEAN_BUILD --platform=linux/$PLATFORM --network host -t ixy-builder.$ARCH --build-arg RUNTIME_ARCH=$ARCH -f Dockerfile .
popd
