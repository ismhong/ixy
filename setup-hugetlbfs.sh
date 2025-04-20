#!/bin/bash

if grep -q HugePages_Total /proc/meminfo; then
	echo "[INFO] HugePage support detected, setting 1024 hugepages..."
	echo '1024' | sudo tee -a /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages
	mkdir -p /mnt/huge
	(mount | grep /mnt/huge) > /dev/null || mount -t hugetlbfs hugetlbfs /mnt/huge
	cat /proc/meminfo | grep -i huge
else
	echo "[ERROR] HugePage not supported on this system!" >&2
fi
