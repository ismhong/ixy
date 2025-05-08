/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 *
 *   Copyright 2023 NXP
 *
 */

#ifndef KPG_NC_MODULE_H
#define KPG_NC_MODULE_H

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "memory.h"

#define KPG_NC_DEVICE_NAME "page_ncache"
#define KPG_NC_DEVICE_PATH "/dev/" KPG_NC_DEVICE_NAME

/* IOCTL */
#define KPG_NC_MAGIC_NUM 0xf0f0
#define KPG_NC_IOCTL_UPDATE _IOWR(KPG_NC_MAGIC_NUM, 1, size_t)

#define KNRM "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"

static void mark_kpage_ncache(uint64_t huge_page);
static void flush_tlb(void *p);

static void mark_kpage_ncache(uint64_t huge_page)
{
	int fd;

	fd = open(KPG_NC_DEVICE_PATH, O_RDONLY);
	if (fd < 0)
	{
		printf(KYEL "Error: " KNRM "Could not open: %s\n", KPG_NC_DEVICE_PATH);
		return;
	}
	printf(KCYN "%s: Huge_Page addr =" KNRM " 0x%lX\n",
		   __func__, huge_page);
	ioctl(fd, KPG_NC_IOCTL_UPDATE, (size_t)&huge_page);
	flush_tlb((void *)huge_page);
	printf(KYEL "Page should be non-cachable now\n" KNRM);
	close(fd);
}

#define CACHE_LINE_SIZE 64

static void flush_tlb(void *p)
{
	unsigned long start_addr, end_addr;
	unsigned long start_aligned, end_aligned;
	unsigned long cache_line_mask = CACHE_LINE_SIZE - 1;

	start_addr = (unsigned long)p;
	end_addr = start_addr + HUGE_PAGE_SIZE;
	start_aligned = start_addr & ~cache_line_mask;
	end_aligned = (end_addr + cache_line_mask) & ~cache_line_mask;

	for (unsigned long vaddr = start_aligned; vaddr < end_aligned; vaddr += CACHE_LINE_SIZE) {
		asm volatile("dc civac, %0" ::"r"((void*)vaddr));
	}

	asm volatile("dsb ish");
	asm volatile("isb");
}

#endif // KPG_NC_MODULE_H
