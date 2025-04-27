//
// E1000 hardware definitions: registers and DMA ring format.
// from the Intel 82540EP/EM &c manual.
// https://www.intel.com/content/dam/doc/manual/pci-pci-x-family-gbe-controllers-software-dev-manual.pdf
//
// For Intel 82574 (E1000E), it seems most registers are the same with E1000 for controlling RX/RX.
// https://docs.rs-online.com/96e8/0900766b81384733.pdf

/* Registers */
#define E1000_CTL      0x00000  /* Device Control Register - RW */
#define E1000_STATUS   0x00008  /* Device Status Register - R */
#define E1000_CTRL_EXT 0x00018	/* Extended Device Control - RW */
#define E1000_MDIC     0x00020  /* MDI Control Register - R */
#define E1000_ICR      0x000C0  /* Interrupt Cause Read - R */
#define E1000_IMS      0x000D0  /* Interrupt Mask Set - RW */
#define E1000_RCTL     0x00100  /* RX Control - RW */
#define E1000_TCTL     0x00400  /* TX Control - RW */
#define E1000_TIPG     0x00410  /* TX Inter-packet gap -RW */
#define E1000_POEMB    0x00F10  /* PHY OEM Bits -RW */
#define E1000_RDBAL    0x02800  /* RX Descriptor Base Address Low - RW */
#define E1000_RDBAH    0x02804  /* RX Descriptor Base Address High - RW */
#define E1000_RDTR     0x02820  /* RX Delay Timer */
#define E1000_RADV     0x0282C  /* RX Interrupt Absolute Delay Timer */
#define E1000_RDH      0x02810  /* RX Descriptor Head - RW */
#define E1000_RDT      0x02818  /* RX Descriptor Tail - RW */
#define E1000_RDLEN    0x02808  /* RX Descriptor Length - RW */
#define E1000_RSRPD    0x02C00  /* RX Small Packet Detect Interrupt */
#define E1000_TDBAL    0x03800  /* TX Descriptor Base Address Low - RW */
#define E1000_TDBAH    0x03804  /* TX Descriptor Base Address High - RW */
#define E1000_TDLEN    0x03808  /* TX Descriptor Length - RW */
#define E1000_TDH      0x03810  /* TX Descriptor Head - RW */
#define E1000_TDT      0x03818  /* TX Descriptor Tail - RW */
#define E1000_TXDCTL   0x03828  /* TX Descriptor Control - RW */
#define E1000_TARC0    0x03840  /* TX Arbitration Count (0) */
#define E1000_TXDCTL1  0x03928  /* TX Descriptor Control (1) - RW */
#define E1000_MTA      0x05200  /* Multicast Table Array - RW Array */
#define E1000_RA       0x05400  /* Receive Address - RW Array */
#define E1000_WUC      0x05800  /* Receive Address - RW Array */
#define E1000_GPRC     0x04074  /* Good Packets Received Count */
#define E1000_GPTC     0x04080  /* Good Packets Transmitted Count */
#define E1000_GORCL    0x04088  /* Good Octets Received Count (Low) */
#define E1000_GORCH    0x0408C  /* Good Octets Received Count (Hi) */
#define E1000_GOTCL    0x0408C  /* Good Octets Transmitted Count (Low) */
#define E1000_GOTCH    0x0408C  /* Good Octets Transmitted Count (Hi) */
#define E1000_TPT      0x040D4  /* Total Packets Transmitted */
#define E1000_TDFH     0x03410	/* TX Data FIFO Head - RW */
#define E1000_TDFT     0x03418	/* TX Data FIFO Tail - RW */
#define E1000_TDFHS    0x03420	/* TX Data FIFO Head Saved - RW */
#define E1000_TDFTS    0x03428	/* TX Data FIFO Tail Saved - RW */
#define E1000_TDFPC    0x03430	/* TX Data FIFO Packet Count - RW */
#define E1000_PBA	0x01000	/* Packet Buffer Allocation - RW */

/* Device Control */
#define E1000_CTL_FD	    0x00000001	/* Full duplex.0=half; 1=full */
#define E1000_CTL_BEM	    0x00000002	/* Endian Mode.0=little,1=big */
#define E1000_CTL_PRIOR    0x00000004	/* Priority on PCI. 0=rx,1=fair */
#define E1000_CTL_LRST     0x00000008	/* Link reset. 0=normal,1=reset */
#define E1000_CTL_TME	    0x00000010	/* Test mode. 0=normal,1=test */
#define E1000_CTL_SLE	    0x00000020	/* Serial Link on 0=dis,1=en */
#define E1000_CTL_ASDE     0x00000020	/* Auto-speed detect enable */
#define E1000_CTL_SLU	    0x00000040	/* Set link up (Force Link) */
#define E1000_CTL_ILOS     0x00000080	/* Invert Loss-Of Signal */
#define E1000_CTL_SPD_SEL  0x00000300	/* Speed Select Mask */
#define E1000_CTL_SPD_10   0x00000000	/* Force 10Mb */
#define E1000_CTL_SPD_100  0x00000100	/* Force 100Mb */
#define E1000_CTL_SPD_1000 0x00000200	/* Force 1Gb */
#define E1000_CTL_BEM32    0x00000400	/* Big Endian 32 mode */
#define E1000_CTL_FRCSPD   0x00000800	/* Force Speed */
#define E1000_CTL_FRCDPX   0x00001000	/* Force Duplex */
#define E1000_CTL_SWDPIN0  0x00040000	/* SWDPIN 0 value */
#define E1000_CTL_SWDPIN1  0x00080000	/* SWDPIN 1 value */
#define E1000_CTL_SWDPIN2  0x00100000	/* SWDPIN 2 value */
#define E1000_CTL_SWDPIN3  0x00200000	/* SWDPIN 3 value */
#define E1000_CTL_SWDPIO0  0x00400000	/* SWDPIN 0 Input or output */
#define E1000_CTL_SWDPIO1  0x00800000	/* SWDPIN 1 input or output */
#define E1000_CTL_SWDPIO2  0x01000000	/* SWDPIN 2 input or output */
#define E1000_CTL_SWDPIO3  0x02000000	/* SWDPIN 3 input or output */
#define E1000_CTL_RST	    0x04000000	/* Global reset */
#define E1000_CTL_RFCE     0x08000000	/* Receive Flow Control enable */
#define E1000_CTL_TFCE     0x10000000	/* Transmit flow control enable */
#define E1000_CTL_RTE	    0x20000000	/* Routing tag enable */
#define E1000_CTL_VME	    0x40000000	/* IEEE VLAN mode enable */
#define E1000_CTL_PHY_RST  0x80000000	/* PHY Reset */

/* Transmit Control */
#define E1000_TCTL_RST    0x00000001    /* software reset */
#define E1000_TCTL_EN     0x00000002    /* enable tx */
#define E1000_TCTL_BCE    0x00000004    /* busy check enable */
#define E1000_TCTL_PSP    0x00000008    /* pad short packets */
#define E1000_TCTL_CT     0x00000ff0    /* collision threshold */
#define E1000_TCTL_CT_SHIFT 4
#define E1000_TCTL_COLD   0x003ff000    /* collision distance */
#define E1000_TCTL_COLD_SHIFT 12
#define E1000_TCTL_SWXOFF 0x00400000    /* SW Xoff transmission */
#define E1000_TCTL_PBE    0x00800000    /* Packet Burst Enable */
#define E1000_TCTL_RTLC   0x01000000    /* Re-transmit on late collision */
#define E1000_TCTL_NRTU   0x02000000    /* No Re-transmit on underrun */
#define E1000_TCTL_MULR   0x10000000    /* Multiple request support */

/* Receive Control */
#define E1000_RCTL_RST            0x00000001    /* Software reset */
#define E1000_RCTL_EN             0x00000002    /* enable */
#define E1000_RCTL_SBP            0x00000004    /* store bad packet */
#define E1000_RCTL_UPE            0x00000008    /* unicast promiscuous enable */
#define E1000_RCTL_MPE            0x00000010    /* multicast promiscuous enab */
#define E1000_RCTL_LPE            0x00000020    /* long packet enable */
#define E1000_RCTL_LBM_NO         0x00000000    /* no loopback mode */
#define E1000_RCTL_LBM_MAC        0x00000040    /* MAC loopback mode */
#define E1000_RCTL_LBM_SLP        0x00000080    /* serial link loopback mode */
#define E1000_RCTL_LBM_TCVR       0x000000C0    /* tcvr loopback mode */
#define E1000_RCTL_DTYP_MASK      0x00000C00    /* Descriptor type mask */
#define E1000_RCTL_DTYP_PS        0x00000400    /* Packet Split descriptor */
#define E1000_RCTL_RDMTS_HALF     0x00000000    /* rx desc min threshold size */
#define E1000_RCTL_RDMTS_QUAT     0x00000100    /* rx desc min threshold size */
#define E1000_RCTL_RDMTS_EIGTH    0x00000200    /* rx desc min threshold size */
#define E1000_RCTL_MO_SHIFT       12            /* multicast offset shift */
#define E1000_RCTL_MO_0           0x00000000    /* multicast offset 11:0 */
#define E1000_RCTL_MO_1           0x00001000    /* multicast offset 12:1 */
#define E1000_RCTL_MO_2           0x00002000    /* multicast offset 13:2 */
#define E1000_RCTL_MO_3           0x00003000    /* multicast offset 15:4 */
#define E1000_RCTL_MDR            0x00004000    /* multicast desc ring 0 */
#define E1000_RCTL_BAM            0x00008000    /* broadcast enable */
#define E1000_RCTL_VME            0x40000000    /* vlan enable */
/* these buffer sizes are valid if E1000_RCTL_BSEX is 0 */
#define E1000_RCTL_SZ_2048        0x00000000    /* rx buffer size 2048 */
#define E1000_RCTL_SZ_1024        0x00010000    /* rx buffer size 1024 */
#define E1000_RCTL_SZ_512         0x00020000    /* rx buffer size 512 */
#define E1000_RCTL_SZ_256         0x00030000    /* rx buffer size 256 */
/* these buffer sizes are valid if E1000_RCTL_BSEX is 1 */
#define E1000_RCTL_SZ_16384       0x00010000    /* rx buffer size 16384 */
#define E1000_RCTL_SZ_8192        0x00020000    /* rx buffer size 8192 */
#define E1000_RCTL_SZ_4096        0x00030000    /* rx buffer size 4096 */
#define E1000_RCTL_VFE            0x00040000    /* vlan filter enable */
#define E1000_RCTL_CFIEN          0x00080000    /* canonical form enable */
#define E1000_RCTL_CFI            0x00100000    /* canonical form indicator */
#define E1000_RCTL_DPF            0x00400000    /* discard pause frames */
#define E1000_RCTL_PMCF           0x00800000    /* pass MAC control frames */
#define E1000_RCTL_BSEX           0x02000000    /* Buffer size extension */
#define E1000_RCTL_SECRC          0x04000000    /* Strip Ethernet CRC */
#define E1000_RCTL_FLXBUF_MASK    0x78000000    /* Flexible buffer size */
#define E1000_RCTL_FLXBUF_SHIFT   27            /* Flexible buffer shift */

/* Receive Descriptor Control */
#define E1000_RXDCTL_PTHRESH 0x0000003F	/* RXDCTL Prefetch Threshold */
#define E1000_RXDCTL_HTHRESH 0x00003F00	/* RXDCTL Host Threshold */
#define E1000_RXDCTL_WTHRESH 0x003F0000	/* RXDCTL Writeback Threshold */
#define E1000_RXDCTL_GRAN    0x01000000	/* RXDCTL Granularity */
#define E1000_RXDCTL_FULL_RX_DESC_WB 0x01010000	/* GRAN=1, WTHRESH=1 */

/* Transmit Descriptor Control */
#define E1000_TXDCTL_PTHRESH 0x0000003F	/* TXDCTL Prefetch Threshold */
#define E1000_TXDCTL_HTHRESH 0x00003F00	/* TXDCTL Host Threshold */
#define E1000_TXDCTL_WTHRESH 0x003F0000	/* TXDCTL Writeback Threshold */
#define E1000_TXDCTL_GRAN    0x01000000	/* TXDCTL Granularity */
#define E1000_TXDCTL_LWTHRESH 0xFE000000	/* TXDCTL Low Threshold */
#define E1000_TXDCTL_FULL_TX_DESC_WB 0x01010000	/* GRAN=1, WTHRESH=1 */
#define E1000_TXDCTL_COUNT_DESC 0x00400000 /* Enable the counting of desc.
					      still to be processed. */

/* Transmit Descriptor command definitions [E1000 3.3.3.1] */
#define E1000_TXD_DTYP_D     0x00100000	/* Data Descriptor */
#define E1000_TXD_DTYP_C     0x00000000	/* Context Descriptor */
#define E1000_TXD_POPTS_IXSM 0x01	/* Insert IP checksum */
#define E1000_TXD_POPTS_TXSM 0x02	/* Insert TCP/UDP checksum */
#define E1000_TXD_CMD_EOP    0x01000000	/* End of Packet */
#define E1000_TXD_CMD_IFCS   0x02000000	/* Insert FCS (Ethernet CRC) */
#define E1000_TXD_CMD_IC     0x04000000	/* Insert Checksum */
#define E1000_TXD_CMD_RS     0x08000000	/* Report Status */
#define E1000_TXD_CMD_RPS    0x10000000	/* Report Packet Sent */
#define E1000_TXD_CMD_DEXT   0x20000000	/* Descriptor extension (0 = legacy) */
#define E1000_TXD_CMD_VLE    0x40000000	/* Add VLAN tag */
#define E1000_TXD_CMD_IDE    0x80000000	/* Enable Tidv register */
#define E1000_TXD_STAT_DD    0x00000001	/* Descriptor Done */
#define E1000_TXD_STAT_EC    0x00000002	/* Excess Collisions */
#define E1000_TXD_STAT_LC    0x00000004	/* Late Collisions */
#define E1000_TXD_STAT_TU    0x00000008	/* Transmit underrun */
#define E1000_TXD_CMD_TCP    0x01000000	/* TCP packet */
#define E1000_TXD_CMD_IP     0x02000000	/* IP packet */
#define E1000_TXD_CMD_TSE    0x04000000	/* TCP Seg enable */
#define E1000_TXD_STAT_TC    0x00000004	/* Tx Underrun */

/* Transmit Descriptor status definitions [E1000 3.3.3.2] */
#define E1000_TXD_STAT_DD    0x00000001 /* Descriptor Done */

/* Receive Address Low & High */
#define E1000_RAL(_i) (0x05400 + ((_i) * 8))
#define E1000_RAH(_i) (0x05404 + ((_i) * 8))

/* Device Status */
#define E1000_STATUS_LU    0x2
#define E1000_LINKSPEED    0xC0    /* two bits of link speed */
#define E1000_10Mbps       0x0
#define E1000_100Mbps      0x1
#define E1000_1000Mbps     0x2
#define E1000_NoneMbps     0x3

/* MDI Control */
#define E1000_MDIC_DATA_MASK 0x0000FFFF
#define E1000_MDIC_REG_MASK  0x001F0000
#define E1000_MDIC_REG_SHIFT 16
#define E1000_MDIC_PHY_MASK  0x03E00000
#define E1000_MDIC_PHY_SHIFT 21
#define E1000_MDIC_OP_WRITE  0x04000000
#define E1000_MDIC_OP_READ   0x08000000
#define E1000_MDIC_READY     0x10000000
#define E1000_MDIC_INT_EN    0x20000000
#define E1000_MDIC_ERROR     0x40000000

/* PHY Control Register */
#define MII_CR_SPEED_SELECT_MSB		0x0040	/* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_COLL_TEST_ENABLE		0x0080	/* Collision test enable */
#define MII_CR_FULL_DUPLEX		0x0100	/* FDX =1, half duplex =0 */
#define MII_CR_RESTART_AUTO_NEG		0x0200	/* Restart auto negotiation */
#define MII_CR_ISOLATE			0x0400	/* Isolate PHY from MII */
#define MII_CR_POWER_DOWN		0x0800	/* Power down */
#define MII_CR_AUTO_NEG_EN		0x1000	/* Auto Neg Enable */
#define MII_CR_SPEED_SELECT_LSB		0x2000	/* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_LOOPBACK			0x4000	/* 0 = normal, 1 = loopback */
#define MII_CR_RESET			0x8000	/* 0 = normal, 1 = PHY reset */

/* PHY Status Register */
#define MII_SR_EXTENDED_CAPS		0x0001	/* Extended register capabilities */
#define MII_SR_JABBER_DETECT		0x0002	/* Jabber Detected */
#define MII_SR_LINK_STATUS		0x0004	/* Link Status 1 = link */
#define MII_SR_AUTONEG_CAPS		0x0008	/* Auto Neg Capable */
#define MII_SR_REMOTE_FAULT		0x0010	/* Remote Fault Detect */
#define MII_SR_AUTONEG_COMPLETE		0x0020	/* Auto Neg Complete */
#define MII_SR_PREAMBLE_SUPPRESS	0x0040	/* Preamble may be suppressed */
#define MII_SR_EXTENDED_STATUS		0x0100	/* Ext. status info in Reg 0x0F */
#define MII_SR_100T2_HD_CAPS		0x0200	/* 100T2 Half Duplex Capable */
#define MII_SR_100T2_FD_CAPS		0x0400	/* 100T2 Full Duplex Capable */
#define MII_SR_10T_HD_CAPS		0x0800	/* 10T	 Half Duplex Capable */
#define MII_SR_10T_FD_CAPS		0x1000	/* 10T	 Full Duplex Capable */
#define MII_SR_100X_HD_CAPS		0x2000	/* 100X  Half Duplex Capable */
#define MII_SR_100X_FD_CAPS		0x4000	/* 100X  Full Duplex Capable */
#define MII_SR_100T4_CAPS		0x8000	/* 100T4 Capable */

/* PHY 1000 MII Register/Bit Definitions */
/* PHY Registers defined by IEEE */
#define PHY_CTRL			0x00	/* Control Register */
#define PHY_STATUS			0x01	/* Status Regiser */
#define PHY_ID1				0x02	/* Phy Id Reg (word 1) */
#define PHY_ID2				0x03	/* Phy Id Reg (word 2) */
#define PHY_AUTONEG_ADV		0x04	/* Autoneg Advertisement */
#define PHY_LP_ABILITY			0x05	/* Link Partner Ability (Base Page) */
#define PHY_AUTONEG_EXP		0x06	/* Autoneg Expansion Reg */
#define PHY_NEXT_PAGE_TX		0x07	/* Next Page TX */
#define PHY_LP_NEXT_PAGE		0x08	/* Link Partner Next Page */
#define PHY_1000T_CTRL			0x09	/* 1000Base-T Control Reg */
#define PHY_1000T_STATUS		0x0A	/* 1000Base-T Status Reg */
#define PHY_EXT_STATUS			0x0F	/* Extended Status Reg */
#define PHY_COPPER_SPEC			0x10	/* Copper Specific Control Reg */

/* Miscellaneous PHY bit definitions. */
#define PHY_PREAMBLE			0xFFFFFFFF
#define PHY_SOF				0x01
#define PHY_OP_READ			0x02
#define PHY_OP_WRITE			0x01
#define PHY_TURNAROUND			0x02
#define PHY_PREAMBLE_SIZE		32
#define MII_CR_SPEED_1000		0x0040
#define MII_CR_SPEED_100		0x2000
#define MII_CR_SPEED_10		0x0000
#define E1000_PHY_ADDRESS		0x01
#define PHY_AUTO_NEG_TIME		90	/* 4.5 Seconds */
#define PHY_FORCE_TIME			20	/* 2.0 Seconds */
#define PHY_REVISION_MASK		0xFFFFFFF0
#define DEVICE_SPEED_MASK		0x00000300	/* Device Ctrl Reg Speed Mask */
#define REG4_SPEED_MASK		0x01E0
#define REG9_SPEED_MASK		0x0300
#define ADVERTISE_10_HALF		0x0001
#define ADVERTISE_10_FULL		0x0002
#define ADVERTISE_100_HALF		0x0004
#define ADVERTISE_100_FULL		0x0008
#define ADVERTISE_1000_HALF		0x0010
#define ADVERTISE_1000_FULL		0x0020

/*  PHY Copper Specific Control bit definitions. */
#define PHY_COPPER_SPEC_POWER_DOWN  0x04    /* Phy power down */

/* Autoneg Advertisement Register */
#define NWAY_AR_SELECTOR_FIELD		0x0001	/* indicates IEEE 802.3 CSMA/CD */
#define NWAY_AR_10T_HD_CAPS		0x0020	/* 10T	 Half Duplex Capable */
#define NWAY_AR_10T_FD_CAPS		0x0040	/* 10T	 Full Duplex Capable */
#define NWAY_AR_100TX_HD_CAPS		0x0080	/* 100TX Half Duplex Capable */
#define NWAY_AR_100TX_FD_CAPS		0x0100	/* 100TX Full Duplex Capable */
#define NWAY_AR_100T4_CAPS		0x0200	/* 100T4 Capable */
#define NWAY_AR_PAUSE			0x0400	/* Pause operation desired */
#define NWAY_AR_ASM_DIR		0x0800	/* Asymmetric Pause Direction bit */
#define NWAY_AR_REMOTE_FAULT		0x2000	/* Remote Fault detected */
#define NWAY_AR_NEXT_PAGE		0x8000	/* Next Page ability supported */

/* 1000BASE-T Control Register */
#define CR_1000T_ASYM_PAUSE		0x0080	/* Advertise asymmetric pause bit */
#define CR_1000T_HD_CAPS		0x0100	/* Advertise 1000T HD capability */
#define CR_1000T_FD_CAPS		0x0200	/* Advertise 1000T FD capability  */
#define CR_1000T_REPEATER_DTE		0x0400	/* 1=Repeater/switch device port */
						/* 0=DTE device */
#define CR_1000T_MS_VALUE		0x0800	/* 1=Configure PHY as Master */
						/* 0=Configure PHY as Slave */
#define CR_1000T_MS_ENABLE		0x1000	/* 1=Master/Slave manual config value */
						/* 0=Automatic Master/Slave config */
#define CR_1000T_TEST_MODE_NORMAL	0x0000	/* Normal Operation */
#define CR_1000T_TEST_MODE_1		0x2000	/* Transmit Waveform test */
#define CR_1000T_TEST_MODE_2		0x4000	/* Master Transmit Jitter test */
#define CR_1000T_TEST_MODE_3		0x6000	/* Slave Transmit Jitter test */
#define CR_1000T_TEST_MODE_4		0x8000	/* Transmitter Distortion test */

/* 1000BASE-T Status Register */
#define SR_1000T_IDLE_ERROR_CNT	0x00FF	/* Num idle errors since last read */
#define SR_1000T_ASYM_PAUSE_DIR	0x0100	/* LP asymmetric pause direction bit */
#define SR_1000T_LP_HD_CAPS		0x0400	/* LP is 1000T HD capable */
#define SR_1000T_LP_FD_CAPS		0x0800	/* LP is 1000T FD capable */
#define SR_1000T_REMOTE_RX_STATUS	0x1000	/* Remote receiver OK */
#define SR_1000T_LOCAL_RX_STATUS	0x2000	/* Local receiver OK */
#define SR_1000T_MS_CONFIG_RES		0x4000	/* 1=Local TX is Master, 0=Slave */
#define SR_1000T_MS_CONFIG_FAULT	0x8000	/* Master/Slave config fault */
#define SR_1000T_REMOTE_RX_STATUS_SHIFT 12
#define SR_1000T_LOCAL_RX_STATUS_SHIFT	13

struct e1000_rx_desc {
	__le64 buffer_addr; /* Address of the descriptor's data buffer */
	__le16 length;      /* Length of data DMAed into data buffer */
	__le16 csum; /* Packet checksum */
	uint8_t  status;  /* Descriptor status */
	uint8_t  errors;  /* Descriptor Errors */
	__le16 special;
};

/* Receive Descriptor bit definitions [section 3.2.3.1] */
#define E1000_RXD_STAT_DD       0x01    /* Descriptor Done */
#define E1000_RXD_STAT_EOP      0x02    /* End of Packet */

/* Transmit Descriptor */
struct e1000_tx_desc {
	uint64_t buffer_addr;	/* Address of the descriptor's data buffer */
	union {
		uint32_t data;
		struct {
			uint16_t length;	/* Data buffer length */
			uint8_t cso;	/* Checksum offset */
			uint8_t cmd;	/* Descriptor control */
		} flags;
	} lower;
	union {
		uint32_t data;
		struct {
			uint8_t status;	/* Descriptor status */
			uint8_t css;	/* Checksum start */
			uint16_t special;
		} fields;
	} upper;
};

/* Supported Device IDs */
#define E1000_VENDOR_ID 0x8086
#define E1000_82540EMA_DEVICE_ID 0x100E
#define E1000_82545EM_DEVICE_ID 0x100F
#define E1000_82540EPA_DEVICE_ID 0x1017
#define E1000_82574_DEVICE_ID 0x10D3

/* Collision related configuration parameters */
#define E1000_COLLISION_THRESHOLD	0xF
#define E1000_CT_SHIFT			4
#define E1000_COLLISION_DISTANCE        63
#define E1000_COLLISION_DISTANCE_82542  64
#define E1000_FDX_COLLISION_DISTANCE	E1000_COLLISION_DISTANCE
#define E1000_HDX_COLLISION_DISTANCE	E1000_COLLISION_DISTANCE
#define E1000_GB_HDX_COLLISION_DISTANCE 512
#define E1000_COLD_SHIFT		12

/* Additional Transmit Descriptor Control definitions */
#define E1000_TXDCTL_QUEUE_ENABLE	0x02000000 /* Ena specific Tx Queue */
