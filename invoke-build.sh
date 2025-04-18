#!/bin/bash
set -e 

ARCH_LIST="arm64v8 amd64"

for ARCH in $ARCH_LIST
do
    if [ $ARCH == "arm64v8" ]; then
        PLATFORM="arm64"
    else
        PLATFORM="amd64"
    fi

    echo "Building ixy for $ARCH"
    ./build-docker-image.sh $ARCH $PLATFORM
    ./build-ixy.sh $ARCH $PLATFORM
done
