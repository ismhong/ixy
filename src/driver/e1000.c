#include <linux/limits.h>
#include <linux/vfio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "device.h"
#include "e1000_type.h"
#include "e1000.h"
#include "libixy-vfio.h"
#include "log.h"
#include "memory.h"
#include "interrupts.h"
#include "pci.h"
#include "stats.h"
#include "time.h"

/* General e1000 driver settings & data structure */
#define E1000_MAX_RX_RING_SIZE 4096
#define E1000_MAX_TX_RING_SIZE 4096
#define E1000_RX_RING_SIZE 512
#define E1000_TX_RING_SIZE 512
#define E1000_PKT_BUF_ENTRY_SIZE 2048
#define E1000_MIN_MEMPOOL_ENTRIES 4096
#define E1000_TX_CLEAN_BATCH 32


// allocated for each rx queue, keeps state for the receive function
struct e1000_rx_queue {
	volatile struct e1000_rx_desc* descriptors;
	struct mempool* mempool;
	uint16_t num_entries;
	// position we are reading from
	uint16_t rx_index;
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

// allocated for each tx queue, keeps state for the transmit function
struct e1000_tx_queue {
	volatile struct e1000_tx_desc* descriptors;
	uint16_t num_entries;
	// position to clean up descriptors that where sent out by the nic
	uint16_t clean_index;
	// position to insert packets for transmission
	uint16_t tx_index;
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

static char* driver_name = "ixy-e1000";

static int e1000_read_phy_reg(struct e1000_device* dev, uint32_t reg_addr,
		uint16_t *phy_data)
{
	/* Set up Op-code, Phy Address, and register address in the MDI
	 * Control register.  The MAC will take care of interfacing with the
	 * PHY to retrieve the desired data.
	 */
	uint32_t phy_addr = 1; // 10.2.2.7 Gigabit PHY
	uint32_t i;
	uint32_t mdic = 0;

	mdic = ((reg_addr << E1000_MDIC_REG_SHIFT) |
			(phy_addr << E1000_MDIC_PHY_SHIFT) |
			(E1000_MDIC_OP_READ));

	set_reg32(dev->addr, E1000_MDIC, mdic);

	/* Poll the ready bit to see if the MDI read completed */
	for (i = 0; i < 64; i++) {
		usleep(10);
		mdic = get_reg32(dev->addr, E1000_MDIC);
		if (mdic & E1000_MDIC_READY)
			break;
	}
	if (!(mdic & E1000_MDIC_READY)) {
		error("MDI Read did not complete");
		return -1;
	}
	if (mdic & E1000_MDIC_ERROR) {
		error("MDI Error");
		return -1;
	}
	*phy_data = mdic;
	return 0;
}

static int e1000_write_phy_reg(struct e1000_device* dev, uint32_t reg_addr, uint16_t phy_data)
{
	/* Set up Op-code, Phy Address, register address, and data intended
	 * for the PHY register in the MDI Control register.  The MAC will take
	 * care of interfacing with the PHY to send the desired data.
	 */
	uint32_t phy_addr = 1; // 10.2.2.7 Gigabit PHY
	uint32_t i;
	uint32_t mdic = 0;

	mdic = (((uint32_t) phy_data) |
			(reg_addr << E1000_MDIC_REG_SHIFT) |
			(phy_addr << E1000_MDIC_PHY_SHIFT) |
			(E1000_MDIC_OP_WRITE));

	set_reg32(dev->addr, E1000_MDIC, mdic);

	/* Poll the ready bit to see if the MDI read completed */
	for (i = 0; i < 64; i++) {
		usleep(10);
		mdic = get_reg32(dev->addr, E1000_MDIC);
		if (mdic & E1000_MDIC_READY)
			break;
	}
	if (!(mdic & E1000_MDIC_READY)) {
		error("MDI Write did not complete");
		return -1;
	}

	return 0;
}

static int e1000_wait_autoneg(struct e1000_device* dev)
{
	uint16_t i;
	uint16_t phy_data;

	info("Waiting for Auto-Neg to complete.");

	/* We will wait for autoneg to complete or 4.5 seconds to expire. */
	for (i = PHY_AUTO_NEG_TIME; i > 0; i--) {
		/* Read the MII Status Register and wait for Auto-Neg
		 * Complete bit to be set.
		 */
		if (e1000_read_phy_reg(dev, PHY_STATUS, &phy_data) < 0) {
			error("PHY Read Error");
			return -1;
		}
		if (e1000_read_phy_reg(dev, PHY_STATUS, &phy_data) < 0) {
			error("PHY Read Error");
			return -1;
		}
		if (phy_data & MII_SR_AUTONEG_COMPLETE) {
			info("Auto-Neg complete.");
			return 0;
		}
		usleep(100*1000);
		fprintf(stdout, ".");
	}
	error("Auto-Neg timedout.");
	return -1;
}

static int32_t e1000_detect_gig_phy(struct e1000_device* dev)
{
	int32_t ret_val;
	uint16_t phy_id_high, phy_id_low;
	uint32_t phy_id = 0;

	/* Read the PHY ID Registers to identify which PHY is onboard. */
	ret_val = e1000_read_phy_reg(dev, PHY_ID1, &phy_id_high);
	if (ret_val) {
		return ret_val;
	}

	phy_id = (uint32_t) (phy_id_high << 16);
	usleep(20);
	ret_val = e1000_read_phy_reg(dev, PHY_ID2, &phy_id_low);
	if (ret_val) {
		return ret_val;
	}

	phy_id |= (uint32_t) (phy_id_low & PHY_REVISION_MASK);
	uint32_t phy_revision = (uint32_t) phy_id_low & ~PHY_REVISION_MASK;

	info("Got phy id(%x) revision(%x)", phy_id, phy_revision);

	return 0;
}

static int32_t e1000_phy_setup_autoneg(struct e1000_device* dev)
{
	int32_t ret_val;
	uint16_t mii_autoneg_adv_reg;
	uint16_t mii_1000t_ctrl_reg;

	/* Read the MII Auto-Neg Advertisement Register (Address 4). */
	ret_val = e1000_read_phy_reg(dev, PHY_AUTONEG_ADV, &mii_autoneg_adv_reg);
	if (ret_val) {
		return ret_val;
	}

	/* Read the MII 1000Base-T Control Register (Address 9). */
	ret_val = e1000_read_phy_reg(dev, PHY_1000T_CTRL,
			&mii_1000t_ctrl_reg);
	if (ret_val) {
		return ret_val;
	}

	/* Need to parse both autoneg_advertised and fc and set up
	 * the appropriate PHY registers.  First we will parse for
	 * autoneg_advertised software override.  Since we can advertise
	 * a plethora of combinations, we need to check each bit
	 * individually.
	 */

	/* First we clear all the 10/100 mb speed bits in the Auto-Neg
	 * Advertisement Register (Address 4) and the 1000 mb speed bits in
	 * the  1000Base-T Control Register (Address 9).
	 */
	mii_autoneg_adv_reg &= ~REG4_SPEED_MASK;
	mii_1000t_ctrl_reg &= ~REG9_SPEED_MASK;

	/*mii_autoneg_adv_reg |= NWAY_AR_10T_HD_CAPS;*/
	/*mii_autoneg_adv_reg |= NWAY_AR_10T_FD_CAPS;*/
	/*mii_autoneg_adv_reg |= NWAY_AR_100TX_HD_CAPS;*/
	/*mii_autoneg_adv_reg |= NWAY_AR_100TX_FD_CAPS;*/
	mii_1000t_ctrl_reg |= CR_1000T_FD_CAPS;

	/* Flow control (RX & TX) is completely disabled by a
	 * software over-ride.
	 */
	mii_autoneg_adv_reg |= (NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);

	ret_val = e1000_write_phy_reg(dev, PHY_AUTONEG_ADV, mii_autoneg_adv_reg);
	if (ret_val) {
		return ret_val;
	}

	debug("Auto-Neg Advertising %x", mii_autoneg_adv_reg);

	ret_val = e1000_write_phy_reg(dev, PHY_1000T_CTRL,
			mii_1000t_ctrl_reg);
	if (ret_val) {
		return ret_val;
	}

	return 0;
}

static int32_t e1000_copper_link_autoneg(struct e1000_device* dev)
{
	int32_t ret_val;
	uint16_t phy_data;

	info("Reconfiguring auto-neg advertisement params");
	ret_val = e1000_phy_setup_autoneg(dev);
	if (ret_val) {
		error("Error Setting up Auto-Negotiation");
		return ret_val;
	}
	info("Restarting Auto-Neg");

	/* Restart auto-negotiation by setting the Auto Neg Enable bit and
	 * the Auto Neg Restart bit in the PHY control register.
	 */
	ret_val = e1000_read_phy_reg(dev, PHY_CTRL, &phy_data);
	if (ret_val) {
		return ret_val;
	}

	phy_data |= (MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG);
	ret_val = e1000_write_phy_reg(dev, PHY_CTRL, phy_data);
	if (ret_val) {
		return ret_val;
	}

	ret_val = e1000_wait_autoneg(dev);
	if (ret_val) {
		error("Error while waiting for autoneg to complete");
		return ret_val;
	}

	return 0;
}

static int32_t e1000_phy_power_up(struct e1000_device* dev)
{
	int ret;
	uint16_t phy_data;
	ret = e1000_read_phy_reg(dev, PHY_CTRL, &phy_data);
	if (ret) {
		error("Read PHY CTRL error");
		return -1;
	}
	debug("PHY CTRL(%x)", phy_data);

	/*phy_data |= MII_CR_RESET;*/
	phy_data &= ~MII_CR_POWER_DOWN;
	ret = e1000_write_phy_reg(dev, PHY_CTRL, phy_data);
	if (ret) {
		error("Write PHY CTRL error");
		return -1;
	}

	ret = e1000_read_phy_reg(dev, PHY_COPPER_SPEC, &phy_data);
	if (ret) {
		error("Read PHY COPPER SPEC error");
	}
	debug("PHY COPPER(%x)", phy_data);

	phy_data &= ~PHY_COPPER_SPEC_POWER_DOWN;
	ret = e1000_write_phy_reg(dev, PHY_COPPER_SPEC, phy_data);
	if (ret) {
		error("Write PHY COPPER SPEC error");
		return -1;
	}
	usleep(10);

	return 0;
}

// see section 14.4
//  - allocate memory from hugepage to setup receive queue
//  - initialize multicast table array (MTA)
//  - setup rx control register (RCTL)
static void init_rx(struct e1000_device* dev) {
	// setup receive queue
	uint32_t ring_size_bytes = E1000_RX_RING_SIZE * sizeof(struct e1000_rx_desc);
	struct dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
	memset(mem.virt, 0, ring_size_bytes);
	struct e1000_rx_desc* rx_ring = (struct e1000_rx_desc*)mem.virt;
	struct e1000_rx_queue* queue = (struct e1000_rx_queue*)(dev->rx_queues);
	queue->num_entries = E1000_RX_RING_SIZE;
	queue->rx_index = 0;
	queue->descriptors = (struct e1000_rx_desc*) mem.virt;
	int mempool_size = E1000_RX_RING_SIZE + E1000_TX_RING_SIZE;
	queue->mempool = memory_allocate_mempool(
		mempool_size < E1000_MIN_MEMPOOL_ENTRIES ?
		E1000_MIN_MEMPOOL_ENTRIES : mempool_size, E1000_PKT_BUF_ENTRY_SIZE);

	for (int i = 0; i < queue->num_entries; i++) {
		volatile struct e1000_rx_desc *rxd = queue->descriptors + i;
		struct pkt_buf* buf = pkt_buf_alloc(queue->mempool);
		rxd->buffer_addr = (uint64_t) (buf->buf_addr_phy + offsetof(struct pkt_buf, data));
    }

	set_reg32(dev->addr, E1000_RDBAL, ((uint64_t) mem.phy) & 0xFFFFFFFFull);
	set_reg32(dev->addr, E1000_RDBAH, ((uint64_t) mem.phy) >> 32);
	set_reg32(dev->addr, E1000_RDH, 0);
	set_reg32(dev->addr, E1000_RDT, 0);
	set_reg32(dev->addr, E1000_RDLEN, sizeof(rx_ring));

	// init multicast table array
	for (int i = 0; i < 128; i++) {
		set_reg32(dev->addr, E1000_MTA + 4*i, 0);
	}

	// setup receive control register
	set_reg32(dev->addr, E1000_RCTL,
		E1000_RCTL_EN |       // enable receiver
		E1000_RCTL_BAM |      // enable broadcast
		E1000_RCTL_SZ_2048 |  // 2048-byte rx buffers
		E1000_RCTL_SECRC);    // strip CRC
}


// see section 14.5
//  - allocate memory from hugepage to setup tx queue
//  - setup tx control register (RCTL)
static void init_tx(struct e1000_device* dev) {
	uint32_t ring_size_bytes = E1000_TX_RING_SIZE * sizeof(struct e1000_tx_desc);
	struct dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
	memset(mem.virt, 0, ring_size_bytes);
	// struct e1000_tx_desc *tx_ring = (struct e1000_tx_desc*)mem.virt;
	// for (int i = 0; i < E1000_TX_RING_SIZE; i++) {
	// 	tx_ring[i].status = 0;
	// }
	struct e1000_tx_queue* queue = (struct e1000_tx_queue*)(dev->tx_queues);
	queue->num_entries = E1000_TX_RING_SIZE;
	queue->descriptors = (struct e1000_tx_desc*) mem.virt;

	debug("tx ring phy addr:  0x%lX", mem.phy);
	debug("tx ring virt addr: 0x%lX", (uintptr_t) mem.virt);

	set_reg32(dev->addr, E1000_TDBAL, (uint32_t) (mem.phy & 0xFFFFFFFFull));
#if RASPBERRY_PI5
	set_reg32(dev->addr, E1000_TDBAH, (uint32_t) (mem.phy >> 32 | 0x10));
#else
	set_reg32(dev->addr, E1000_TDBAH, (uint32_t) (mem.phy >> 32));
#endif
	set_reg32(dev->addr, E1000_TDLEN, ring_size_bytes);

	// tx queue starts out empty
	set_reg32(dev->addr, E1000_TDH, 0);
	set_reg32(dev->addr, E1000_TDT, 0);

	// close mulitiple queue
	clear_flags32(dev->addr, E1000_TCTL, E1000_TCTL_MULR);

	uint32_t txdctl = get_reg32(dev->addr, E1000_TXDCTL);
	txdctl = ((txdctl & ~E1000_TXDCTL_WTHRESH) |
			(1 << 22) | 
		    E1000_TXDCTL_FULL_TX_DESC_WB | E1000_TXDCTL_COUNT_DESC);
	txdctl |= E1000_TXDCTL_GRAN;
	set_reg32(dev->addr, E1000_TXDCTL, txdctl);

	// setup tx control register
	uint32_t tctl;
	tctl = get_reg32(dev->addr, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		 (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));
	set_reg32(dev->addr, E1000_TCTL, tctl);

	set_reg32(dev->addr, E1000_TIPG, 10 | (8<<10) | (6<<20)); // inter-pkt gap
	get_reg32(dev->addr, E1000_STATUS);
}

static void wait_for_link(const struct e1000_device* dev) {
	info("Waiting for link...");
	int32_t max_wait = 10000000; // 10 seconds in us
	uint32_t poll_interval = 100000; // 10 ms in us
	while (!(e1000_get_link_speed(&dev->ixy)) && max_wait > 0) {
		usleep(poll_interval);
		max_wait -= poll_interval;
	}
	info("Link speed is %d Mbit/s", e1000_get_link_speed(&dev->ixy));
}

uint32_t e1000_get_link_speed(const struct ixy_device* ixy) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t status = get_reg32(dev->addr, E1000_STATUS);

	if (!(status & E1000_STATUS_LU)) {
		return 0;
	}
	int  value = (status & E1000_LINKSPEED) >> 6;
	switch (value) {
		case E1000_1000Mbps:
			return 1000;
		case E1000_100Mbps:
			return 100;
		case E1000_10Mbps:
			return 10;
		default:
			return 0;
  }
  return 0;
}

struct mac_address e1000_get_mac_addr(const struct ixy_device* ixy) {
	struct mac_address mac;
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t rar_low = get_reg32(dev->addr, E1000_RAL(0));
	uint32_t rar_high = get_reg32(dev->addr, E1000_RAH(0)) & ~(1 << 31);

	mac.addr[0] = rar_low;
	mac.addr[1] = rar_low >> 8;
	mac.addr[2] = rar_low >> 16;
	mac.addr[3] = rar_low >> 24;
	mac.addr[4] = rar_high;
	mac.addr[5] = rar_high >> 8;
	return mac;
}

void e1000_set_mac_addr(struct ixy_device* ixy, struct mac_address mac) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t rar_low = mac.addr[0] + (mac.addr[1] << 8) + (mac.addr[2] << 16) + (mac.addr[3] << 24);
	uint32_t rar_high = mac.addr[4] + (mac.addr[5] << 8);

	set_reg32(dev->addr, E1000_RAL(0), rar_low);
	set_reg32(dev->addr, E1000_RAH(0), rar_high);
}

void e1000_set_promisc(struct ixy_device* ixy, bool enabled) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	if (enabled) {
		info("enabling promisc mode");
		set_flags32(dev->addr, E1000_RCTL, E1000_RCTL_MPE | E1000_RCTL_UPE);
	} else {
		info("disabling promisc mode");
		clear_flags32(dev->addr, E1000_RCTL, E1000_RCTL_MPE | E1000_RCTL_UPE);
	}
}

// read stat counters and accumulate in stats
// stats may be NULL to just reset the counters
void e1000_read_stats(struct ixy_device* ixy, struct device_stats* stats) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t rx_pkts = get_reg32(dev->addr, E1000_GPRC);
	uint32_t tx_pkts = get_reg32(dev->addr, E1000_GPTC);
	uint64_t rx_bytes = get_reg32(dev->addr, E1000_GORCL) + (((uint64_t) get_reg32(dev->addr, E1000_GORCH)) << 32);
	uint64_t tx_bytes = get_reg32(dev->addr, E1000_GOTCL) + (((uint64_t) get_reg32(dev->addr, E1000_GOTCH)) << 32);

	if (stats) {
		stats->rx_pkts += rx_pkts;
		stats->tx_pkts += tx_pkts;
		stats->rx_bytes += rx_bytes;
		stats->tx_bytes += tx_bytes;
	}
}

// advance index with wrap-around, this line is the reason why we require a power of two for the queue size
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))

// see section 3.2 and 3.3 for rx ring structure and information.
// general logic is as follows:
// - when hardware receives complete data, it update status DD in rx descriptor
//   and move the head of the rx ring.
// - when driver tries to get data from rx ring, it will read status of rx descriptor
//   to determine whether rx descriptor contains a complete data.
// - rx ring structure:
//    [ <rx-desc> <rx-desc> ... <rx-desc> ... <rx-desc>]
//         |                                     |
//        tail                                  head

uint32_t e1000_rx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);

	if (queue_id > 0) {
		error("queue id must be zero, queue_id=%d", queue_id);
		return 0;
	}

	struct e1000_rx_queue* queue = (struct e1000_rx_queue*) (dev->rx_queues);
	uint16_t rx_index = queue->rx_index;
	uint16_t last_rx_index = rx_index;
	uint32_t buf_index;

	for (buf_index = 0; buf_index < num_bufs; buf_index++) {
		volatile struct e1000_rx_desc* desc_ptr = queue->descriptors + rx_index;
		uint32_t status = desc_ptr->status;
		// if rx desc contains complete data, proceed to copy it out
		if (status & E1000_RXD_STAT_DD) {
			if (!(status & E1000_RXD_STAT_EOP)) {
				error("multi-segment packets are not supported - increase buffer size or decrease MTU");
			}

			struct e1000_rx_desc desc = *desc_ptr;
			struct pkt_buf* buf = (struct pkt_buf*) queue->virtual_addresses[rx_index];
			buf->size = desc.length;

			struct pkt_buf* new_buf = pkt_buf_alloc(queue->mempool);
			if (!new_buf) {
				error("failed to allocate new mbuf for rx");
			}

			// reset the descriptor
			desc_ptr->buffer_addr = new_buf->buf_addr_phy + offsetof(struct pkt_buf, data);
			desc_ptr->status = 0;
			queue->virtual_addresses[rx_index] = new_buf;

			//store data in output buffers
			bufs[buf_index] = buf;
			last_rx_index = rx_index;
			rx_index = wrap_ring(rx_index, queue->num_entries);
		} else {
			break;
		}
	}

	if (rx_index != last_rx_index) {
		set_reg32(dev->addr, E1000_RDT, last_rx_index);
		queue->rx_index = rx_index;
	}

	return buf_index;
}

// see section 3.4 and 3.4 for tx ring structure, general logic is as follows:
//  - hardware controls head to tx ring, driver controls tail of tx ring
//  - when data to transmit is done by hardware, it updates status DD
//    in tx descriptor and move the ring head
//  - when software sending data, it tries to clean previous done transmisions
//    [clean -> clean + batch_size] and update tx descriptors at tail
//  - tx ring structure:
//    [ <tx-desc> <tx-desc> ... <tx-desc> ... <tx-desc>]
//         |                       |             |
//       head                    clean          tail
// tx batching results in good performance gains
uint32_t e1000_tx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	if (queue_id != 0) {
		info("queue not exist");
		return 0;
	}
	struct e1000_tx_queue* queue = (struct e1000_tx_queue*)(dev->tx_queues);
	uint16_t clean_index = queue->clean_index; // next descriptor to clean up

	while (true) {
		int32_t cleanable = queue->tx_index - clean_index;
		if (cleanable < 0) { // handle wrap-around
			cleanable = queue->num_entries + cleanable;
		}
		if (cleanable < E1000_TX_CLEAN_BATCH) {
			break;
		}
		int32_t cleanup_to = clean_index + E1000_TX_CLEAN_BATCH - 1;
		if (cleanup_to >= queue->num_entries) {
			cleanup_to -= queue->num_entries;
		}
		volatile struct e1000_tx_desc* txd = queue->descriptors + cleanup_to;
		uint32_t status = txd->upper.data;
		if (status & E1000_TXD_STAT_DD) {
			int32_t i = clean_index;
			while (true) {
				volatile struct e1000_tx_desc* r = queue->descriptors + i;
				if (!(r->upper.data & E1000_TXD_STAT_DD)) {
					error("status ring: i=%d, status=%x",i, r->upper.fields.status);
					break;
				}
				struct pkt_buf* buf = queue->virtual_addresses[i];
				if (buf) {
					pkt_buf_free(buf);
				}
				if (i == cleanup_to) {
					break;
				}
				i = wrap_ring(i, queue->num_entries);
			}
			clean_index = wrap_ring(cleanup_to, queue->num_entries);
		} else {
			break;
		}
	}
	queue->clean_index = clean_index;

	uint32_t sent;
	for (sent = 0; sent < num_bufs; sent++) {
		uint32_t next_index = wrap_ring(queue->tx_index, queue->num_entries);
		if (clean_index == next_index) {
			break;
		}
		struct pkt_buf* buf = bufs[sent];
		queue->virtual_addresses[queue->tx_index] = (void*) buf;
		volatile struct e1000_tx_desc* txd = queue->descriptors + queue->tx_index;
#if RASPBERRY_PI5
		txd->buffer_addr = (0x1000000000) | ((uint64_t)buf->buf_addr_phy + offsetof(struct pkt_buf, data));
#else
		txd->buffer_addr = ((uint64_t)buf->buf_addr_phy + offsetof(struct pkt_buf, data));
#endif
		uint32_t flag = 0;
		flag |= (E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS | E1000_TXD_CMD_IFCS);
		txd->lower.data = (buf->size & 0xFFFF) | flag;
		txd->upper.data = 0;
		queue->tx_index = next_index;
	}
	set_reg32(dev->addr, E1000_TDT, queue->tx_index);
	// debug("good packets transmit count %d", get_reg32(dev->addr, E1000_GPTC));
	// debug("total packets transmit count %d", get_reg32(dev->addr, E1000_TPT));

	return sent;
}

static void reset_and_init(struct e1000_device* dev) {
	info("Resetting device %s", dev->ixy.pci_addr);

	// disable interrupts
	uint32_t control = get_reg32(dev->addr, E1000_CTL);
	set_reg32(dev->addr, E1000_IMS, 0);
	set_reg32(dev->addr, E1000_CTL, control | E1000_CTL_RST);
	wait_clear_reg32(dev->addr, E1000_CTL, E1000_CTL_RST);

	// redisable interrupts
	set_reg32(dev->addr, E1000_IMS, 0);
	get_reg32(dev->addr, E1000_STATUS);

	struct mac_address mac = e1000_get_mac_addr(&dev->ixy);
	info("Initializing device %s", dev->ixy.pci_addr);
	info("MAC address %02x:%02x:%02x:%02x:%02x:%02x", mac.addr[0], mac.addr[1], mac.addr[2], mac.addr[3], mac.addr[4], mac.addr[5]);

	// Make sure PHY power up
	e1000_phy_power_up(dev);

	// reset statistics
	e1000_read_stats(&dev->ixy, NULL);

	// init rx and tx queue
	init_tx(dev);
	init_rx(dev);

	// ask e1000 for receive interrupts.
	set_reg32(dev->addr, E1000_RDTR, 0); // interrupt after every received packet (no timer)
	set_reg32(dev->addr, E1000_RADV, 0); // interrupt after every packet (no timer)
	set_reg32(dev->addr, E1000_IMS, (1 << 7)); // RXDW -- Receiver Descriptor Write Back

	e1000_set_promisc(&dev->ixy, true);
	// set link up
	set_reg32(dev->addr, E1000_CTL, get_reg32(dev->addr, E1000_CTL) | E1000_CTL_SLU);
	wait_for_link(dev);

	info("Done resetting device %s, ctrl_reg=%x", dev->ixy.pci_addr, get_reg32(dev->addr, E1000_CTL)) ;
}

/**
 * Initializes and returns the e1000 device.
 * @param pci_addr The PCI address of the device.
 * @param rx_queues The number of receiver queues.
 * @param tx_queues The number of transmitter queues.
 * @param interrupt_timeout The interrupt timeout in milliseconds
 * 	- if set to -1 the interrupt timeout is disabled
 * 	- if set to 0 the interrupt is disabled entirely)
 * @return The initialized e1000 device.
 */
struct ixy_device* e1000_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues, int interrupt_timeout) {
  debug("init e1000");
	if (getuid()) {
		warn("Not running as root, this will probably fail");
	}
	if (rx_queues > MAX_QUEUES) {
		error("cannot configure %d rx queues: limit is %d", rx_queues, MAX_QUEUES);
	}
	if (tx_queues > MAX_QUEUES) {
		error("cannot configure %d tx queues: limit is %d", tx_queues, MAX_QUEUES);
	}

	// Allocate memory for the e1000 device that will be returned
	struct e1000_device* dev = (struct e1000_device*) malloc(sizeof(struct e1000_device));
	dev->ixy.pci_addr = strdup(pci_addr);

	dev->ixy.vfio = 0;
	dev->ixy.driver_name = driver_name;
	dev->ixy.num_rx_queues = rx_queues;
	dev->ixy.num_tx_queues = tx_queues;
	dev->ixy.rx_batch = e1000_rx_batch;
	dev->ixy.tx_batch = e1000_tx_batch;
	dev->ixy.read_stats = e1000_read_stats;
	dev->ixy.set_promisc = e1000_set_promisc;
	dev->ixy.get_link_speed = e1000_get_link_speed;
	dev->ixy.get_mac_addr = e1000_get_mac_addr;
	dev->ixy.set_mac_addr = e1000_set_mac_addr;
	dev->ixy.interrupts.interrupts_enabled = false;
	dev->ixy.interrupts.itr_rate = 0;
	dev->ixy.interrupts.timeout_ms = 0;

	debug("mapping BAR0 region via pci file...");
	dev->addr = pci_map_resource(pci_addr);
	dev->rx_queues = calloc(rx_queues, sizeof(struct e1000_rx_queue) + sizeof(void*) * E1000_MAX_RX_RING_SIZE);
	dev->tx_queues = calloc(tx_queues, sizeof(struct e1000_tx_queue) + sizeof(void*) * E1000_MAX_TX_RING_SIZE);
	reset_and_init(dev);
	read_power_state(pci_addr);
	return &dev->ixy;
}

bool is_e1000_compatible(uint16_t vendor_id, uint16_t device_id) {
	if (vendor_id != E1000_VENDOR_ID) {
		return false;
	}

	if (device_id == E1000_82540EMA_DEVICE_ID
		|| device_id == E1000_82540EPA_DEVICE_ID
		|| device_id == E1000_82545EM_DEVICE_ID
		|| device_id == E1000_82574_DEVICE_ID) { // It seems the e1000 control flow also works for e1000e
		return true;
	}

	return false;
}
