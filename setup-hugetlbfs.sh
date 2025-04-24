#!/bin/bash

if mountpoint -q /mnt/huge; then
	echo "Clear mount point first..."
    umount /mnt/huge
fi

if grep -q HugePages_Total /proc/meminfo; then
	echo "[INFO] HugePage support detected, setting 512 hugepages..."
	mkdir -p /mnt/huge
	(mount | grep /mnt/huge) > /dev/null || mount -t hugetlbfs hugetlbfs /mnt/huge
	echo '512' | sudo tee -a /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages
	cat /proc/meminfo | grep -i huge
else
	echo "[ERROR] HugePage not supported on this system!" >&2
fi
