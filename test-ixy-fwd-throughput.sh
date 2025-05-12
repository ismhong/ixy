#!/bin/bash

pkt_size=(64 128 256 512 1024 1400)
exec_time=10
pcie_addr="0000:01:00.0"

for pkt in "${pkt_size[@]}"; do
    export PKT_SIZE=${pkt}
    export EXEC_TIME=${exec_time}
    cmake "-DPKT_SIZE=${pkt}" "-DEXEC_TIME=${exec_time}" . > /dev/null 2>&1
    make > /dev/null 2>&1
    sudo ./ixy-fwd-loopback ${pcie_addr}
done
