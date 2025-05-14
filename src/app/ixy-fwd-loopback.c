#include <stdio.h>
#include <stdlib.h>
#include "stats.h"
#include "log.h"
#include "memory.h"
#include "driver/device.h"

// number of packets sent simultaneously to our driver
static const int NUM_BUFS = 1024;
static const uint32_t BATCH_SIZE = 64;

// calculate a IP/TCP/UDP checksum
static uint16_t calc_ip_checksum(uint8_t* data, uint32_t len) {
	if (len % 1) error("odd-sized checksums NYI"); // we don't need that
	uint32_t cs = 0;
	for (uint32_t i = 0; i < len / 2; i++) {
		cs += ((uint16_t*)data)[i];
		if (cs > 0xFFFF) {
			cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
		}
	}
	return ~((uint16_t) cs);
}

static struct mempool* init_mempool(uint32_t pkt_size) {
	const uint8_t pkt_data[] = {
		0x6C, 0xB3, 0x11, 0x18, 0x4E, 0xAD, // dst MAC
		0x10, 0x10, 0x10, 0x10, 0x10, 0x10, // src MAC
		0x08, 0x00,							// ether type: IPv4
		0x45, 0x00,							// Version, IHL, TOS
		(pkt_size - 14) >> 8,				// ip len excluding ethernet, high byte
		(pkt_size - 14) & 0xFF,				// ip len exlucding ethernet, low byte
		0x00, 0x00, 0x00, 0x00,				// id, flags, fragmentation
		0x40, 0x11, 0x00, 0x00,				// TTL (64), protocol (UDP), checksum
		0x0A, 0x00, 0x00, 0x01,				// src ip (10.0.0.1)
		0x0A, 0x00, 0x00, 0x02,				// dst ip (10.0.0.2)
		0x00, 0x2A, 0x05, 0x39,				// src and dst ports (42 -> 1337)
		(pkt_size - 20 - 14) >> 8,			// udp len excluding ip & ethernet, high byte
		(pkt_size - 20 - 14) & 0xFF,		// udp len exlucding ip & ethernet, low byte
		0x00, 0x00,							// udp checksum, optional
		'i', 'x', 'y'						// payload
		// rest of the payload is zero-filled because mempools guarantee empty bufs
	};

	struct mempool* mempool = memory_allocate_mempool(NUM_BUFS, 0);
	struct pkt_buf* bufs[NUM_BUFS];

	// pre-fill all our packet buffers with some templates that can be modified later
	// we have to do it like this because sending is async in the hardware; we cannot re-use a buffer immediately
	for (uint32_t buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		struct pkt_buf* buf = pkt_buf_alloc(mempool);
		buf->size = pkt_size;
		memcpy(buf->data, pkt_data, sizeof(pkt_data));
		*(uint16_t*) (buf->data + 24) = calc_ip_checksum(buf->data + 14, 20);
		bufs[buf_id] = buf;
	}

	// return them all to the mempool, all future allocations will return bufs with the data set above
	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		pkt_buf_free(bufs[buf_id]);
	}

	return mempool;
}

static void dump_result_csv(uint32_t pkt_size, struct device_stats* stats_new, struct device_stats* stats_old, uint64_t nanos, char csv_file[]) {
	FILE* p = fopen(csv_file, "a");

	if (!p) {
		error("can not open csv file");
	}
	uint32_t tx_mbits = diff_mbit(stats_new->tx_bytes, stats_old->tx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos);
	uint32_t rx_mbits = diff_mbit(stats_new->rx_bytes, stats_old->rx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos);


	fprintf(p, "%u,%u,%u\n", pkt_size, tx_mbits, rx_mbits);
	fclose(p);
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		printf("Usage: %s <pci bus id>\n", argv[0]);
		return 1;
	}

	char* endptr = NULL;
	uint32_t pkt_size = strtoul(argv[2], &endptr, 10);
	uint32_t exec_time = strtoul(argv[3], &endptr, 10);

	debug("pktsize: %d, exec_time: %d", pkt_size, exec_time);
	struct ixy_device* dev = ixy_init(argv[1], 1, 1, 0, true);
	struct mempool* tx_mempool = init_mempool(pkt_size);

	uint64_t last_stats_printed = monotonic_time();
	uint64_t counter = 0;
	struct device_stats stats_old, stats;
	stats_init(&stats, dev);
	stats_init(&stats_old, dev);
	uint32_t seq_num = 0;

	// array of bufs sent out in a batch
	struct pkt_buf* tx_bufs[NUM_BUFS];
	struct pkt_buf* rx_bufs[NUM_BUFS];
	pkt_buf_alloc_batch(tx_mempool, tx_bufs, NUM_BUFS);
	for (uint32_t i = 0; i < NUM_BUFS; i++) {
		*(uint32_t*)(tx_bufs[i]->data + pkt_size - 4) = seq_num++;
	}

	uint32_t count_tx = 0;
	uint32_t count_rx = 0;
	uint32_t num_rx = 0;

	// loopback
	while(true) {
		if (count_tx - count_rx < BATCH_SIZE)
			count_tx += ixy_tx_batch(dev, 0, tx_bufs, BATCH_SIZE);
		num_rx = ixy_rx_batch(dev, 0, rx_bufs, BATCH_SIZE);
		count_rx += num_rx;

		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = monotonic_time();
			if (time - last_stats_printed > 1000000000UL * exec_time) {
				// every second
				count_rx = 0;
				count_tx = 0;
				ixy_read_stats(dev, &stats);
				dump_result_csv(pkt_size, &stats, &stats_old, time - last_stats_printed, argv[4]);
				stats_old = stats;
				last_stats_printed = time;
				return 0;
			}
		}
	}
}
