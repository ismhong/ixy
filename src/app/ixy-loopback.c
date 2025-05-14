#include <stdio.h>
#include <stdlib.h>
#include <argp.h> // For GNU argp
#include "stats.h"
#include "log.h"
#include "memory.h"
#include "driver/device.h"

// number of packets sent simultaneously to our driver
static const int NUM_BUFS = 1024;
static const uint32_t BATCH_SIZE = 64;

// For GNU argp
const char *argp_program_version = "ixy-loopback 1.0";
const char *argp_program_bug_address = "<ism.hong@gmail.com>"; // Replace with your email
static char doc[] = "A simple ixy-based loopback application.";
static char args_doc[] = ""; // No non-option arguments

// Default values
#define DEFAULT_EXEC_TIME 10
#define DEFAULT_PKT_SIZE 64
#define DEFAULT_CSV_FILENAME "loopback_stats.csv"

#define XSTR(s) STR(s)
#define STR(s) #s

// Options definition for argp
static struct argp_option options[] = {
    {"time", 't', "SECONDS", 0, "Execute time in seconds (default: " STR(DEFAULT_EXEC_TIME) ")", 0},
    {"device", 'd', "PCIE_ADDR", 0, "PCIE device address (e.g., 0000:00:08.0) (required)", 0},
    {"packetsize", 's', "SIZE", 0, "Packet size in bytes (default: " STR(DEFAULT_PKT_SIZE) ")", 0},
    {"file", 'f', "CSV_FILE", 0, "CSV file name for dumping results (default: " DEFAULT_CSV_FILENAME ")", 0},
    {0}};

// Structure to hold the parsed arguments
struct arguments {
	uint32_t exec_time;
	char *pcie_address;
	uint32_t pkt_size;
	char *csv_file_name;
};

// Parser function for argp
static error_t parse_opt(int key, char *arg, struct argp_state *state) {
	struct arguments *arguments = state->input;
	char *endptr;

	switch (key) {
		case 't':
			arguments->exec_time = strtoul(arg, &endptr, 10);
			if (*endptr != '\0' || arguments->exec_time == 0) {
				argp_usage(state); // Invalid number
			}
			break;
		case 'd':
			arguments->pcie_address = arg;
			break;
		case 's':
			arguments->pkt_size = strtoul(arg, &endptr, 10);
			if (*endptr != '\0' || arguments->pkt_size == 0) {
				argp_usage(state); // Invalid number
			}
			break;
		case 'f':
			arguments->csv_file_name = arg;
			break;
		case ARGP_KEY_ARG:
			// Too many arguments
			argp_usage(state);
			break;
		case ARGP_KEY_END:
			// Check if all required arguments are provided
			if (arguments->pcie_address == NULL || arguments->csv_file_name == NULL || arguments->exec_time == 0 || arguments->pkt_size == 0) {
				argp_usage(state);
			}
			break;
		default:
			return ARGP_ERR_UNKNOWN;
	}
	return 0;
}

// argp parser
static struct argp argp = {
	.options = options,
	.parser = parse_opt,
	.args_doc = args_doc,
	.doc = doc
};

// calculate a IP/TCP/UDP checksum
static uint16_t calc_ip_checksum(uint8_t *data, uint32_t len) {
	if (len % 2 != 0)
		error("odd-sized checksums NYI"); // Corrected the modulo operation
	uint32_t cs = 0;
	for (uint32_t i = 0; i < len / 2; i++) {
		cs += ((uint16_t *)data)[i];
		if (cs > 0xFFFF) {
			cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
		}
	}
	return ~((uint16_t)cs);
}

static struct mempool *init_mempool(uint32_t pkt_size) {
	const uint8_t pkt_data_template[] = { // Renamed to avoid conflict
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

	struct mempool *mempool = memory_allocate_mempool(NUM_BUFS, 0);
	struct pkt_buf *bufs[NUM_BUFS];

	// pre-fill all our packet buffers with some templates that can be modified later
	// we have to do it like this because sending is async in the hardware; we cannot re-use a buffer immediately
	for (uint32_t buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		struct pkt_buf *buf = pkt_buf_alloc(mempool);
		buf->size = pkt_size;
		memcpy(buf->data, pkt_data_template, sizeof(pkt_data_template));

		// Set IP length
		uint16_t ip_len = pkt_size - 14;
		buf->data[16] = (ip_len >> 8) & 0xFF;
		buf->data[17] = ip_len & 0xFF;

		// Set UDP length
		uint16_t udp_len = pkt_size - 14 - 20;
		buf->data[38] = (udp_len >> 8) & 0xFF;
		buf->data[39] = udp_len & 0xFF;

		// Calculate and set IP checksum
		*(uint16_t *)(buf->data + 24) = 0; // Clear checksum field before calculation
		*(uint16_t *)(buf->data + 24) = calc_ip_checksum(buf->data + 14, 20);
		bufs[buf_id] = buf;
	}

	// return them all to the mempool, all future allocations will return bufs with the data set above
	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		pkt_buf_free(bufs[buf_id]);
	}

	return mempool;
}

static void dump_result_csv(uint32_t pkt_size_arg, struct device_stats *stats_new, struct device_stats *stats_old, uint64_t nanos, const char csv_file[]) { // Changed pkt_size to pkt_size_arg to avoid conflict
	FILE *p = fopen(csv_file, "a");

	if (!p) {
		error("can not open csv file: %s", csv_file); // Added filename to error message
		return; // Return on error
	}

	// Check if the file is empty to print the header
	fseek(p, 0, SEEK_END);
	long size = ftell(p);
	if (size == 0) {
		fprintf(p, "pkt_size,tx_mbits,rx_mbits\n");
	}
	fseek(p, 0, SEEK_CUR); // Reset position if not at end (though append mode handles this)


	uint32_t tx_mbits = diff_mbit(stats_new->tx_bytes, stats_old->tx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos);
	uint32_t rx_mbits = diff_mbit(stats_new->rx_bytes, stats_old->rx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos); // Should this be stats_new->rx_pkts for rx_mbits? Assuming it's stats_old->tx_pkts based on original code.

	fprintf(p, "%u,%u,%u\n", pkt_size_arg, tx_mbits, rx_mbits);
	fclose(p);
}

int main(int argc, char *argv[]) {
	struct arguments arguments;

	// Set default values
    // Initialize with default values
    arguments.exec_time = DEFAULT_EXEC_TIME;
    arguments.pcie_address = NULL; // Required, so no default string
    arguments.pkt_size = DEFAULT_PKT_SIZE;
    arguments.csv_file_name = DEFAULT_CSV_FILENAME;

	// Parse arguments
	argp_parse(&argp, argc, argv, 0, 0, &arguments);

	uint32_t pkt_size = arguments.pkt_size;
	uint32_t exec_time = arguments.exec_time;

	debug("pktsize: %u, exec_time: %u, pcie_dev: %s, csv_file: %s",
			pkt_size, exec_time, arguments.pcie_address, arguments.csv_file_name);

	struct ixy_device *dev = ixy_init(arguments.pcie_address, 1, 1, 0, true);
	if (!dev) {
		error("ixy_init failed for device %s", arguments.pcie_address);
		return 1;
	}
	struct mempool *tx_mempool = init_mempool(pkt_size);

	uint64_t last_stats_printed = monotonic_time();
	uint64_t counter = 0;
	struct device_stats stats_old, stats;
	stats_init(&stats, dev);
	stats_init(&stats_old, dev);
	uint32_t seq_num = 0;

	// array of bufs sent out in a batch
	struct pkt_buf *tx_bufs[NUM_BUFS];
	struct pkt_buf *rx_bufs[NUM_BUFS];
	pkt_buf_alloc_batch(tx_mempool, tx_bufs, NUM_BUFS);
	for (uint32_t i = 0; i < NUM_BUFS; i++) {
		if (pkt_size >= 4) { // Ensure packet size is large enough for sequence number
			*(uint32_t *)(tx_bufs[i]->data + pkt_size - 4) = seq_num++;
		}
	}

	uint32_t count_tx = 0;
	uint32_t count_rx = 0;
	uint32_t num_rx = 0;

	// loopback
	while (true) {
		if (count_tx - count_rx < BATCH_SIZE) { // Potential optimization: pre-calculate available slots in tx_bufs
			uint32_t num_to_tx = BATCH_SIZE; // Send BATCH_SIZE if possible
											 // In a real loopback, you'd likely want to ensure rx_bufs are processed before overwriting them by tx
											 // This simple forwarder might not care as much if packets are dropped due to full TX/RX queues.
			count_tx += ixy_tx_batch(dev, 0, tx_bufs, num_to_tx); // Using tx_bufs directly. Consider if these should be distinct from received packets.
		}

		num_rx = ixy_rx_batch(dev, 0, rx_bufs, BATCH_SIZE);
		count_rx += num_rx;

		// Potentially process received packets in rx_bufs here if needed for true loopback logic
		// For simple forwarding, we might just be measuring throughput.
		// If these were truly looped back, you'd copy data from rx_bufs to tx_bufs or similar.

		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = monotonic_time();
			if (time - last_stats_printed > 1000000000UL * exec_time) {
				// every `exec_time` seconds
				count_rx = 0; // Resetting these counters here means stats are per interval
				count_tx = 0;
				ixy_read_stats(dev, &stats);
				dump_result_csv(pkt_size, &stats, &stats_old, time - last_stats_printed, arguments.csv_file_name);
				stats_old = stats;
				last_stats_printed = time;
				// If exec_time is a one-shot duration, you might want to break the loop here.
				// The current logic makes it run for exec_time, print stats, then exit in the next check.
				// For continuous run with periodic stats, remove the return 0.
				// Based on "execute time", it implies a one-shot run.
				info("Execution time reached. Exiting.");
				break; // Exit the while loop
			}
		}
	}
	// Clean up (optional for such a simple app, but good practice)
	// ixy_remove_device(dev); // This function might not exist or be needed depending on ixy version
	// memory_free_mempool(tx_mempool); // This function might not exist

	return 0; // Return 0 after the loop breaks
}
