#!/bin/bash
set -e

CPU=3
# set up DVFS to performance
echo performance > /sys/devices/system/cpu/cpu${CPU}/cpufreq/scaling_governor

# run test
PKT_SIZE=(64 128 256 512 1024 1400)
EXEC_TIME=10
PCIE_ADDR="0000:01:00.0"

CSV_FILE="output.csv"
rm -f $CSV_FILE
echo "Packet Size (bytes),Tx (Mbits/s),Rx (Mbits/s)" > $CSV_FILE

for pkt in "${PKT_SIZE[@]}"; do
    echo $pkt
    ./ixy-fwd-loopback ${PCIE_ADDR} ${pkt} ${EXEC_TIME} ${CSV_FILE}
done
