
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/delay.h>
#include <linux/ethtool.h>

//#define DEBUG_RX 1
//#define DEBUG_TX 1

/* Packet size info */
#define XAE_TRL_SIZE              4 /* Size of Ethernet trailer (FCS) */
#define XAE_MTU			          1500 /* Max MTU of an Ethernet frame */
#define XAE_JUMBO_MTU		      9000 /* Max MTU of a jumbo Eth. frame */

#define XAE_MAX_FRAME_SIZE	     (XAE_MTU + VLAN_ETH_HLEN + XAE_TRL_SIZE)
#define XAE_MAX_VLAN_FRAME_SIZE  (XAE_MTU + VLAN_ETH_HLEN + XAE_TRL_SIZE)
#define XAE_MAX_JUMBO_FRAME_SIZE (XAE_JUMBO_MTU + VLAN_ETH_HLEN + XAE_TRL_SIZE)

#define XAUIENET_NAPI_WEIGHT	  64

#define XAUIENET_RX_BUF_COUNT	  2048
#define XAUIENET_TX_BUF_COUNT	  2048

#define XAUIENET_MIN_RX_BUF_COUNT 256
#define XAUIENET_MIN_TX_BUF_COUNT 256

#define XAUIENET_MAX_RX_BUF_COUNT 4096
#define XAUIENET_MAX_TX_BUF_COUNT 4096

#define XAUIENET_PAUSE_GAP 0x7FFF

#define AXIS_IN_KERNEL 1
#include "xaui_enet.h"

#define MAC_BASE_ADDR       0xb0000000 // base address of XAUI MAC register space
#define BSI_BASE_ADDR       0x84000008 // base address of BSI registers
#define LINK_WAIT           100        // link training wait in usec - 100ms

#define FIFO_OFFSET         0x10000
#define REG_OFFSET          0x00000

#define RX_ENABLE_ADDR     (REG_OFFSET  + 0x000)
#define TX_ENABLE_ADDR     (REG_OFFSET  + 0x004)
#define FIFO_CLEAR_ADDR    (REG_OFFSET  + 0x008)
#define INT_ENABLE_ADDR    (REG_OFFSET  + 0x00C)
#define FIFO_VALID_ADDR    (REG_OFFSET  + 0x010)
#define MAX_RX_SIZE_ADDR   (REG_OFFSET  + 0x014)
#define ONLINE_ACK_ADDR    (REG_OFFSET  + 0x018)
#define INT_PENDING_ACK    (REG_OFFSET  + 0x01C)
#define RX_PEND_ADDR       (FIFO_OFFSET + 0x000)
#define TX_FREE_ADDR       (FIFO_OFFSET + 0x004)
#define RX_FREE_ADDR       (FIFO_OFFSET + 0x200)
#define TX_POST_ADDR_A     (FIFO_OFFSET + 0x240)
#define TX_POST_ADDR_B     (FIFO_OFFSET + 0x244)
#define TX_POST_ADDR_C     (FIFO_OFFSET + 0x248)
#define TX_POST_ADDR       (FIFO_OFFSET + 0x24C)

#define RX_IRQ_READY_MASK   0x1
#define TX_IRQ_READY_MASK   0x2

// Module Configuration
__u32 cfgTxCount = XAUIENET_TX_BUF_COUNT;
__u32 cfgRxCount = XAUIENET_RX_BUF_COUNT;
__u32 cfgRxSize  = XAE_MAX_JUMBO_FRAME_SIZE;

#define DRIVER_NAME "xaui_enet_drv"
#define MODULE_NAME "xaui_enet"

// Axi DMA Register structure
struct xauienet_reg  {
   __u8 * rxEnable;
   __u8 * txEnable;
   __u8 * fifoClear;
   __u8 * intEnable;
   __u8 * fifoValid;
   __u8 * maxRxSize;
   __u8 * onlineAck;
   __u8 * intPendAck;
   __u8 * rxPend;
   __u8 * txFree;
   __u8 * rxFree;
   __u8 * txPostA;
   __u8 * txPostB;
   __u8 * txPostC;
   __u8 * txPass;
};

// DMA Buffer List
struct xauienet_buffer {
   dma_addr_t  buffHandle;
   struct sk_buff * skb;
};

// Device Driver Structure
struct xauienet_device {
    phys_addr_t   baseAddr;
    __u32         baseSize;
    __u8        * virtAddr;
    __u8        * mapped_base;
    __u8        * mapped_bsi;

    const char * devName;

	struct device * device;
	struct net_device *ndev;
	struct napi_struct napi;	/* NAPI Structure */
	struct net_device_stats stats; /* Statistics for this device */

	__u32 max_frm_size;
	__u32 rxmem;

    struct tasklet_struct tx_done_tasklet;
    
	spinlock_t rx_lock;		/* Spin lock */
	spinlock_t tx_lock;		/* Spin lock */
    
    __u32 cur_rx_idx;    
    __u32 cur_tx_idx;
    __u32 last_tx_idx;
    
   __u32 idx;
   __u32 irq;
   
   __u32 ip_csum; // checksum offload

   // Buffers
   struct xauienet_buffer ** rxBuffers;
   struct xauienet_buffer ** txBuffers;

   __u32 rxCount;
   __u32 txCount;

   // Debug
   __u32 writeCount;
   __u32 readCount; 
   __u32 descDrops;
   __u32 descRetires;
   __u32 badSizeMarker;
   __u32 badFifoMarker;
   __u32 axiWriteErrors;
   __u32 axiStreamErrors;
   __u32 readSizeErrors;
   __u32 writeSizeErrors;
   __u32 overFlows;
   __u32 frags;
   __u32 csumPart;
   __u32 txFreeCount;
   __u32 txBusyCount;
   __u32 rxIrqCount;
   __u32 txIrqCount;
   __u32 txBuffErrors;
   __u32 skbAllocErrors;
   __u32 skbFreeErrors;
   
   // Register Space
   struct xauienet_reg reg;
 
};

/*
** XAUI driver statistics structure
*/

static const struct xauienet_stat {
	char name[ETH_GSTRING_LEN];
	u16 offset;
} xaui_stats[] = {
	{ "rxCnt      ", XAUI_RXCNT       },   
	{ "rxIrqCnt   ", XAUI_RXIRQCNT    },   
	{ "rxSzErrs   ", XAUI_RXSZERRS    },   
	{ "rxOvrFlow  ", XAUI_RXOVRFLOW   },   
	{ "rxFrags    ", XAUI_RXFRAGS     },   
	{ "rxDscRtire ", XAUI_RXDSCRTIRE  },   
	{ "rxDscDrop  ", XAUI_RXDSCDROP   },   
	{ "rxSkbErrs  ", XAUI_RXSKBERRS   },   
	{ "rxBadSzMrk ", XAUI_RXMADSZMRK  },   
	{ "rxBadFifMrk", XAUI_RXBADFIFMRK },   
	{ "axiStrErrs ", XAUI_AXISTRERRS  },   
	{ "axiWrErrs  ", XAUI_AXIWRERRS   },   
	{ "txCnt      ", XAUI_TXCNT       },   
	{ "txFreeCnt  ", XAUI_TXFREECNT   },   
	{ "txIrqCnt   ", XAUI_TXIRQCNT    },   
	{ "txSzErrs   ", XAUI_TXSZERRS    },   
	{ "txSkbErrs  ", XAUI_TXSKBERRS   },   
	{ "txBsyCnt   ", XAUI_TXBSYCNT    },   
	{ "txBufErrs  ", XAUI_TXBUFERRS   },   
	{ "txCsumPart ", XAUI_TXCSUMPART  },   
    { "macRxFrames", XAUI_MACRXFRAMES },
    { "macRxPause ", XAUI_MACRXPAUSE  },
    { "macRxOvrFlw", XAUI_MACRXOVRFLW },
    { "macRxCrcErr", XAUI_MACRXCRCERR },
    { "macRxDrops ", XAUI_MACRXDROPS  },    
    { "macTxFrames", XAUI_MACTXFRAMES },
    { "macTxPause ", XAUI_MACTXPAUSE  },
    { "macTxUndrRn", XAUI_MACTXUNDRRN },
    { "macTxLkNrdy", XAUI_MACTXLKNRDY },
};

/*
** XAUI MAC register structure
*/

Xaui_Registers* _Xaui_Registers = (Xaui_Registers*)0;

/*
** XAUI MAC dump strings
*/

static const char _enabled[]      = "enabled";
static const char _disabled[]     = "disabled";
static const char _asserted[]     = "asserted";
static const char _deasserted[]   = "deasserted";
static const char _up[]           = "up";
static const char _down[]         = "down";
static const char _not_synched[]  = "not synchronized";
static const char _are_synched[]  = "are synchronized";
static const char _complete[]     = "complete";
static const char _not_complete[] = "not complete";
static const char _aligned[]      = "aligned";
static const char _not_aligned[]  = "not aligned";

static const char _line_0[] =  "  Register 0000: = %08X (Reset counter is %s)\n";
static const char _line_1[] =  "  Register 0004: = %08X (Reset PHY is %s)\n";
static const char _line_2[] =  "  Register 0008: = %08X (Core parameters)\n";
static const char _line_4[] =  "  Register 0010: = %08X (Pause frame time is %d)\n";
static const char _line_5[] =  "  Register 0014: = %08X%08X (MAC address)\n";
static const char _line_6[] =  "  Register 001C: = %08X (RSSI IP)\n";
static const char _line_7[] =  "  Register 0020: = %08X (Core status)\n";
static const char _line_8[] =  "  Register 0024: = %08X (Debug status)\n";
static const char _line_9[] = "  Register 0038: = %08X (Control bits)\n";

/*
** decoded core fields...
*/

static const char _loopback[]     = "    Loopback is %s\n";            
static const char _power_down[]   = "    Power-down is %s\n";          
static const char _reset_faults[] = "    Reset faults is %s\n";        
static const char _reset_link[]   = "    Reset link is %s\n";          
static const char _test[]         = "    Test pattern is %s\n";        
static const char _test_pattern[] = "    Test pattern is set to %d\n"; 

/*
** decoded status register fields...
*/

static const char _tx_fault[] = "    Transmit (TX) fault is %s\n"; 
static const char _rx_fault[] = "    Receive (RX) fault is %s\n";  
static const char _link[]     = "    Receive (RX) Link is %s\n";   
static const char _synched[]  = "    Receive (RX) lanes are %s\n"; 

/*
** decoded debug register fields...
*/

static const char _phased[]        = "    Transmit (TX) phase alignment is %s\n";                                 
static const char _lanes_synched[] = "    Bit-list of synchronized lanes is %01X (synchronized if offset SET)\n"; 
static const char _lanes_aligned[] = "    Receive (RX) lanes are %s\n";                                           

/*
** decoded ctl fields...
*/

static const char _rxShiftCnt[]   = "    Receive (RX) header shift bytes %d\n";            
static const char _txShiftCnt[]   = "    Transmit (TX) header shift bytes %d\n";               
static const char _macFilter[]    = "    MAC filter is %s\n";        
static const char _ipCsum[]       = "    IP checksum offload is %s\n"; 
static const char _tcpCsum[]      = "    TCP checksum offload is %s\n"; 
static const char _udpCsum[]      = "    UDP checksum offload is %s\n"; 

/*
** XAUI MAC dump strings
*/

static const char _counter_0[] = "  Register 0100: = %d (decimal) (# of received (RX) frames)\n";
static const char _counter_1[] = "  Register 0104: = %d (decimal) (# of transmitted (TX) frames)\n";
static const char _counter_2[] = "  Register 0108: = %d (decimal) (# of receive (RX) pause frames)\n";
static const char _counter_3[] = "  Register 010C: = %d (decimal) (# of transmit (TX) pause frames)\n";
static const char _counter_4[] = "  Register 0110: = %d (decimal) (# of receive (RX) overflows)\n";
static const char _counter_5[] = "  Register 0114: = %d (decimal) (# of received (RX) CRC errors)\n";
static const char _counter_6[] = "  Register 0118: = %d (decimal) (# of transmit (TX) under runs)\n";
static const char _counter_7[] = "  Register 011C: = %d (decimal) (# of transmit (TX) link not ready)\n";
static const char _counter_8[] = "  Register 0120: = %d (decimal) (# of receive (RX) frame drops)\n";

/*
** ++
**
**
** --
*/

inline __u32 Xaui_ENABLE(__u32 offset, __u32 axi_register) 
  { 
  __u32 mask   = ~(~(0xFFFFFFFF << 1) << offset);  
  __u32 result = axi_register & mask; 
  
  return result | (1 << offset);
  }

/*
** ++
**
**
** --
*/

inline __u32 Xaui_DISABLE(__u32 offset, __u32 axi_register) 
 {
 __u32 mask = ~(~(0xFFFFFFFF << 1) << offset);

 return axi_register & mask;
 }

/*
** ++
**
**
** --
*/

inline __u32 Xaui_ENCODE(__u32 offset, __u32 length, __u32 value, __u32 axi_register)
 {

 __u32 mask   = ~(~(0xFFFFFFFF << length) << offset);
 __u32 result = axi_register & mask;

 return result | (value << offset);
 }
 
/*
** ++
**
**
** --
*/

inline __u32 Xaui_IS(__u32 offset, __u32 axi_register) 
 {
 return (1 << offset) & axi_register;
 }
 
/*
** ++
**
**
** --
*/

__u32 Xaui_DECODE(__u32 offset, __u32 length, __u32 axi_register) 
 {
 return (axi_register >> offset) & ((1 << length) - 1);
 }

/**
 * xauienet_mac_dump - MAC firmware register dump
 * Prints the MAC firmware registers.
 */
void xauienet_mac_dump(void)
 {
 Xaui_Counters* counters;
   
 /*
 ** print all configuration and status registers...
 */
  
 Xaui_Config* config = &(_Xaui_Registers->config);

 __u32    next;
 const char* state;
 
 next  = config->counters;
 state = Xaui_IS(XAUI_COUNTERS_RESET, next) ? _asserted : _deasserted;

 printk(_line_0, next, state);
 
 next  = config->phy;
 state = Xaui_IS(XAUI_PHY_RESET, next) ? _asserted : _deasserted;

 printk(_line_1, next, state);

 next  = config->core;
 
 printk(_line_2, next);
  
 printk(_loopback,     Xaui_IS(XAUI_CORE_LOOPBACK,     next) ? _enabled  : _disabled);
 printk(_power_down,   Xaui_IS(XAUI_CORE_POWER_DOWN,   next) ? _enabled  : _disabled);
 printk(_reset_faults, Xaui_IS(XAUI_CORE_RESET_FAULTS, next) ? _asserted : _deasserted);
 printk(_reset_link,   Xaui_IS(XAUI_CORE_RESET_LINK,   next) ? _asserted : _deasserted);
 printk(_test,         Xaui_IS(XAUI_CORE_TEST,         next) ? _enabled  : _disabled);
 printk(_test_pattern, Xaui_DECODE(XAUI_CORE_TEST_PATTERN, XAUI_CORE_TEST_PATTERN_LENGTH, next));

 next  = config->pause;
 
 printk(_line_4, next, Xaui_DECODE(XAUI_PAUSE_GAP, XAUI_PAUSE_GAP_LENGTH, next));

 printk(_line_5, config->mac[1], config->mac[0]);

 next  = config->rssiIp;
 
 printk(_line_6, next);

 next  = config->status;
 
 printk(_line_7, next);

 printk(_tx_fault, Xaui_IS(XAUI_STATUS_TX_FAULT, next) ? _asserted    : _deasserted);
 printk(_rx_fault, Xaui_IS(XAUI_STATUS_RX_FAULT, next) ? _asserted    : _deasserted);
 printk(_synched,  Xaui_IS(XAUI_STATUS_SYNCHED,  next) ? _are_synched : _not_synched);
 printk(_link,     Xaui_IS(XAUI_STATUS_RX_LINK,  next) ? _up          : _down);

 next  = config->debug;

 printk(_line_8, next);

 printk(_phased,        Xaui_IS(XAUI_DEBUG_PHASED, next) ? _complete    : _not_complete);
 printk(_lanes_synched, Xaui_DECODE(XAUI_DEBUG_SYNCHED, XAUI_DEBUG_SYNCHED_LENGTH, next));
 printk(_lanes_aligned, Xaui_IS(XAUI_DEBUG_ALIGNED,  next) ? _aligned : _not_aligned);
 
 next = config->ctl;
 
 printk(_line_9, next);
  
 printk(_rxShiftCnt, Xaui_DECODE(XAUI_CTL_RX_SHIFT, XAUI_CTL_RX_SHIFT_LENGTH, next));
 printk(_txShiftCnt, Xaui_DECODE(XAUI_CTL_TX_SHIFT, XAUI_CTL_TX_SHIFT_LENGTH, next));
 printk(_macFilter, Xaui_IS(XAUI_CTL_MAC_FILTER, next) ? _enabled : _disabled);
 printk(_ipCsum, Xaui_IS(XAUI_CTL_IP_CSUM, next) ? _enabled : _disabled);
 printk(_tcpCsum, Xaui_IS(XAUI_CTL_TCP_CSUM, next) ? _enabled : _disabled);
 printk(_udpCsum, Xaui_IS(XAUI_CTL_UDP_CSUM, next) ? _enabled : _disabled);
 
 /*
 ** Print all counters...
 */
 
 counters = &(_Xaui_Registers->counters);
 
 printk(_counter_0, counters->rx); 
 printk(_counter_1, counters->tx); 
 printk(_counter_2, counters->rxp);
 printk(_counter_3, counters->txp);
 printk(_counter_4, counters->rx_overflow); 
 printk(_counter_5, counters->rx_crc); 
 printk(_counter_6, counters->tx_underflow); 
 printk(_counter_7, counters->tx_link_not_ready); 
 printk(_counter_8, counters->rx_drops);
 
 return;
 }

/**
 * xauienet_get_ringparam - Ethtool ring parameters

 * Returns driver ring parameters to ethtool
 */
static void
xauienet_get_ringparam(struct net_device *ndev, struct ethtool_ringparam *erp)
 {
 struct xauienet_device *dev;

 dev = netdev_priv(ndev);

 memset(erp, 0, sizeof(struct ethtool_ringparam));

 erp->rx_max_pending = XAUIENET_MAX_RX_BUF_COUNT;
 erp->tx_max_pending = XAUIENET_MAX_TX_BUF_COUNT;
 erp->rx_pending = dev->rxCount;
 erp->tx_pending = dev->txCount;
 
 }

/**
 * xauienet_set_ringparam - Ethtool ring parameters

 * Sets driver ring parameters from ethtool
 */
static int
xauienet_set_ringparam(struct net_device *ndev, struct ethtool_ringparam *erp)
 {
 
 struct xauienet_device *dev;

 dev = netdev_priv(ndev);

 if (netif_running(ndev))
   {
   netdev_info(ndev, "Disable interface before changing ring parameters\n");
   return 0;
   }

 if ((erp->rx_mini_pending) || (erp->rx_jumbo_pending))
   return -EINVAL;

 if((erp->rx_pending <= XAUIENET_MAX_RX_BUF_COUNT) &&
    (erp->rx_pending >= XAUIENET_MIN_RX_BUF_COUNT))
   dev->rxCount = erp->rx_pending;
 else
   return -EINVAL;
 
 if((erp->tx_pending <= XAUIENET_MAX_TX_BUF_COUNT) &&
    (erp->tx_pending >= XAUIENET_MIN_TX_BUF_COUNT))
   dev->txCount = erp->tx_pending;
 else
   return -EINVAL;
 
 return 0;
 
 }

/**
 * xauienet_get_ethtool_strings - Ethtool statistics strings

 * Returns driver statistics strings to ethtool
 */
void xauienet_get_ethtool_strings(struct net_device *ndev, u32 stringset, u8 *data)
 {
 
 int i;
 
 switch (stringset) {
 case ETH_SS_STATS:
	 for (i = 0; i < ARRAY_SIZE(xaui_stats); i++)
		 memcpy(data + i * ETH_GSTRING_LEN,
			 xaui_stats[i].name, ETH_GSTRING_LEN);
	 break;
   }
 }

/**
 * xauienet_get_sset_count - Ethtool statistics string count

 * Returns driver statistics string count to ethtool
 */
static int xauienet_get_sset_count(struct net_device *dev, int sset)
 {
 
 switch (sset) 
   {
   case ETH_SS_STATS:
	 return ARRAY_SIZE(xaui_stats);
   default:
	 return -EOPNOTSUPP;
   }
   
 }
 
/**
 * xauienet_get_ethtool_stats - Ethtool statistics

 * Returns driver statistics to ethtool
 */
void xauienet_get_ethtool_stats(struct net_device *ndev, struct ethtool_stats *stats, u64 *data)
 {
 
 struct xauienet_device *dev;

 Xaui_Counters* counters;

 dev = netdev_priv(ndev);
  
 counters = &(_Xaui_Registers->counters);  
 
 /* driver counters */   
 data[XAUI_RXCNT      ] = dev->readCount;
 data[XAUI_RXIRQCNT   ] = dev->rxIrqCount;
 data[XAUI_RXSZERRS   ] = dev->readSizeErrors;
 data[XAUI_RXOVRFLOW  ] = dev->overFlows;
 data[XAUI_RXFRAGS    ] = dev->frags;
 data[XAUI_RXDSCRTIRE ] = dev->descRetires;
 data[XAUI_RXDSCDROP  ] = dev->descDrops;
 data[XAUI_RXSKBERRS  ] = dev->skbAllocErrors;
 data[XAUI_RXMADSZMRK ] = dev->badSizeMarker;
 data[XAUI_RXBADFIFMRK] = dev->badFifoMarker;
 data[XAUI_AXISTRERRS ] = dev->axiStreamErrors;
 data[XAUI_AXIWRERRS  ] = dev->axiWriteErrors;
 data[XAUI_TXCNT      ] = dev->writeCount;
 data[XAUI_TXFREECNT  ] = dev->txFreeCount;
 data[XAUI_TXIRQCNT   ] = dev->txIrqCount;
 data[XAUI_TXSZERRS   ] = dev->writeSizeErrors;
 data[XAUI_TXSKBERRS  ] = dev->skbFreeErrors;
 data[XAUI_TXBSYCNT   ] = dev->txBusyCount;
 data[XAUI_TXBUFERRS  ] = dev->txBuffErrors;
 data[XAUI_TXCSUMPART ] = dev->csumPart;
 
 /* firmware counters */
 data[XAUI_MACRXFRAMES] = counters->rx;
 data[XAUI_MACRXPAUSE ] = counters->rxp;
 data[XAUI_MACRXOVRFLW] = counters->rx_overflow;
 data[XAUI_MACRXCRCERR] = counters->rx_crc;
 data[XAUI_MACRXDROPS ] = counters->rx_drops;
 data[XAUI_MACTXFRAMES] = counters->tx;
 data[XAUI_MACTXPAUSE ] = counters->txp;
 data[XAUI_MACTXUNDRRN] = counters->tx_underflow;
 data[XAUI_MACTXLKNRDY] = counters->tx_link_not_ready;
 
 }
  

/**
 * xauienet_mac_up - MAC firmware device init
 * @mac:	Pointer to 64 bit mac address storage
 * @ndev:	Pointer to the net_device structure
 *
 * Takes the MAC core out of reset and initializes
 * configuration registers.
 */
void xauienet_mac_up(__u32 *mac, struct net_device *ndev)
 {

 Xaui_Registers* this = _Xaui_Registers;
 __u32 core;
 
 struct xauienet_device *dev;

 dev = netdev_priv(ndev);
 
 /* clear the core config */
 this->config.core     = 0;  
 
 /* take phy out of reset */  
 this->config.phy      = Xaui_DISABLE(XAUI_PHY_RESET, 0);  

 /* configure interface */ 
 this->config.pause    = Xaui_ENCODE(XAUI_PAUSE_GAP, XAUI_PAUSE_GAP_LENGTH, XAUIENET_PAUSE_GAP, 0); 
 this->config.mac[0]   = mac[0];
 this->config.mac[1]   = mac[1];
 
 this->config.ctl = Xaui_ENCODE(XAUI_CTL_RX_SHIFT, XAUI_CTL_RX_SHIFT_LENGTH, 0, this->config.ctl); 
 this->config.ctl = Xaui_ENCODE(XAUI_CTL_TX_SHIFT, XAUI_CTL_TX_SHIFT_LENGTH, 0, this->config.ctl); 
 this->config.ctl = Xaui_ENABLE(XAUI_CTL_MAC_FILTER, this->config.ctl);

 /* RX and TX checksums cannot be enabled/disabled separately */
 if( (ndev->features & NETIF_F_IP_CSUM) ||
     (ndev->features & NETIF_F_RXCSUM))
   {
   this->config.ctl = Xaui_ENABLE(XAUI_CTL_IP_CSUM, this->config.ctl);
   this->config.ctl = Xaui_ENABLE(XAUI_CTL_TCP_CSUM, this->config.ctl);
   this->config.ctl = Xaui_ENABLE(XAUI_CTL_UDP_CSUM, this->config.ctl);
   ndev->features |= NETIF_F_IP_CSUM | NETIF_F_RXCSUM;
   dev->ip_csum = 1;
   }
   else
   {
   this->config.ctl = Xaui_DISABLE(XAUI_CTL_IP_CSUM, this->config.ctl);
   this->config.ctl = Xaui_DISABLE(XAUI_CTL_TCP_CSUM, this->config.ctl);
   this->config.ctl = Xaui_DISABLE(XAUI_CTL_UDP_CSUM, this->config.ctl);
   ndev->features &= ~NETIF_F_IP_CSUM;
   ndev->features &= ~NETIF_F_RXCSUM;
   dev->ip_csum = 0;
   }

 while(!Xaui_IS(XAUI_STATUS_RX_LINK, this->config.status))
   {          
   /* assert reset */
   core = 0;   
   core = Xaui_ENABLE(XAUI_CORE_RESET_LINK,   core); 
   core = Xaui_ENABLE(XAUI_CORE_RESET_FAULTS, core);
   this->config.core = core;

   /* wait for link training */
   msleep(LINK_WAIT);

   /* deassert reset */
   core = 0;
   core = Xaui_DISABLE(XAUI_CORE_RESET_LINK,   core); 
   core = Xaui_DISABLE(XAUI_CORE_RESET_FAULTS, core);
   this->config.core = core;
   }

   printk(KERN_INFO "<%s>: link up (10000/FULL)\n",dev->devName);        

 }

/**
 * xauienet_mac_down - MAC firmware device reset
 * @ndev:	Pointer to the net_device structure
 *
 * Places the MAC core in reset.
 */
void xauienet_mac_down(struct net_device *ndev)
 {
 
 Xaui_Registers* this = _Xaui_Registers;

 struct xauienet_device *dev;

 dev = netdev_priv(ndev);

// printk(KERN_INFO "<%s>: rdCnt      %08d wrCnt       %08d\n",dev->devName,dev->readCount,dev->writeCount);
// printk(KERN_INFO "<%s>: rxSzErrs   %08d wrSzErrs    %08d\n",dev->devName,dev->readSizeErrors,dev->writeSizeErrors);
// printk(KERN_INFO "<%s>: rxIrqCnt   %08d txIrqCnt    %08d\n",dev->devName,dev->rxIrqCount,dev->txIrqCount);
// printk(KERN_INFO "<%s>: rxSkbErrs  %08d txSkbErrs   %08d\n",dev->devName,dev->skbAllocErrors,dev->skbFreeErrors);        
// printk(KERN_INFO "<%s>: rxDscDrop  %08d txFreeCnt   %08d\n",dev->devName,dev->descDrops,dev->txFreeCount);
// printk(KERN_INFO "<%s>: rxOvrFlow  %08d txBsyCnt    %08d\n",dev->devName,dev->overFlows,dev->txBusyCount);        
// printk(KERN_INFO "<%s>: rxDscRtire %08d txBufErrs   %08d\n",dev->devName,dev->descRetires,dev->txBuffErrors);        
// printk(KERN_INFO "<%s>: rxBadSzMrk %08d rxBadFifMrk %08d\n",dev->devName,dev->badSizeMarker,dev->badFifoMarker);        
// printk(KERN_INFO "<%s>: axiStrErrs %08d axiWrErrs   %08d\n",dev->devName,dev->axiStreamErrors,dev->axiWriteErrors);
// printk(KERN_INFO "<%s>: MAC firmware registers\n",dev->devName);
 
// xauienet_mac_dump();
  
 /* put phy into reset */  
 this->config.phy = Xaui_ENABLE(XAUI_PHY_RESET, 0);  

 printk(KERN_INFO "<%s>: link down\n",dev->devName);

 }

// Setup register pointers
void setRegisterPointers ( struct xauienet_device * dev ) {
   dev->reg.rxEnable     = dev->virtAddr + RX_ENABLE_ADDR;
   dev->reg.txEnable     = dev->virtAddr + TX_ENABLE_ADDR;
   dev->reg.fifoClear    = dev->virtAddr + FIFO_CLEAR_ADDR;
   dev->reg.intEnable    = dev->virtAddr + INT_ENABLE_ADDR;
   dev->reg.fifoValid    = dev->virtAddr + FIFO_VALID_ADDR;
   dev->reg.maxRxSize    = dev->virtAddr + MAX_RX_SIZE_ADDR;
   dev->reg.onlineAck    = dev->virtAddr + ONLINE_ACK_ADDR;
   dev->reg.intPendAck   = dev->virtAddr + INT_PENDING_ACK;
   dev->reg.rxPend       = dev->virtAddr + RX_PEND_ADDR;
   dev->reg.txFree       = dev->virtAddr + TX_FREE_ADDR;
   dev->reg.rxFree       = dev->virtAddr + RX_FREE_ADDR;
   dev->reg.txPostA      = dev->virtAddr + TX_POST_ADDR_A;
   dev->reg.txPostB      = dev->virtAddr + TX_POST_ADDR_B;
   dev->reg.txPostC      = dev->virtAddr + TX_POST_ADDR_C;
   dev->reg.txPass       = dev->virtAddr + TX_POST_ADDR;
}

/**
 * xauienet_dma_release - Release buffer descriptors
 * @ndev:	Pointer to the net_device structure
 *
 * This function is used to release the descriptors allocated in
 * xauienet_dma_init. Called when the Xaui Ethernet
 * driver stop api is called.
 */
static void xauienet_dma_bd_release(struct net_device *ndev)
{
   __u32 x;

   struct xauienet_device *dev;

   dev = netdev_priv(ndev);

   // De-Allocate TX buffers
   for (x=0; x < dev->txCount; x++) {
      if ( dev->txBuffers[x]->skb != NULL ) {
         dev_kfree_skb(dev->txBuffers[x]->skb); 
         dev->txBuffers[x]->skb = NULL;
      }         
      kfree(dev->txBuffers[x]);
   }
   if(dev->txCount)
     kfree(dev->txBuffers);

   // De-Allocate RX buffers
   for (x=0; x < dev->rxCount; x++) {
      if ( dev->rxBuffers[x]->skb != NULL ) {
         dev_kfree_skb(dev->rxBuffers[x]->skb);
         dev->rxBuffers[x]->skb = NULL;
      }
      kfree(dev->rxBuffers[x]);
   }
   if(dev->rxCount)
     kfree(dev->rxBuffers);
}

/**
 * xauienet_dma_bd_init - Setup buffer descriptors
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to initialize the Rx and Tx DMA descriptors.
 * This initializes allocates and initializes descriptors with 
 * required default values. Called when the Xaui Ethernet 
 * driver reset is called.
 */
static int xauienet_dma_bd_init(struct net_device *ndev)
{
   __u32 x;
   __u32 count;

   struct sk_buff *skb;

   struct xauienet_device *dev;
   
   dev = netdev_priv(ndev);

   printk(KERN_DEBUG "<%s> bd_init: Creating %i RX Buffers. Size=%i Bytes\n", dev->devName,dev->rxCount,dev->max_frm_size);
   if ( dev->rxCount > 0 ) {
      dev->rxBuffers = (struct xauienet_buffer **) 
         kmalloc( (sizeof(struct xauienet_buffer *) * dev->rxCount), GFP_KERNEL );
   }

   // Allocate RX buffers
   count = 0;
   for (x=0; x < dev->rxCount; x++) {
      dev->rxBuffers[x] = (struct xauienet_buffer *) 
         kmalloc( sizeof(struct xauienet_buffer), GFP_KERNEL );

	  skb = netdev_alloc_skb(ndev, dev->max_frm_size);
	  if (skb == NULL) {
        printk(KERN_ERR"<%s> bd_init: Unable to allocate rx skb.",dev->devName);
        return (-1);
	  }

      dev->rxBuffers[x]->buffHandle = dma_map_single(ndev->dev.parent,
						    skb->data,           
						    dev->max_frm_size,    
						    DMA_FROM_DEVICE);

      dev->rxBuffers[x]->skb = skb;

      iowrite32(dev->rxBuffers[x]->buffHandle,dev->reg.rxFree);
      ++count;
   }

   printk(KERN_DEBUG "<%s> bd_init: Created  %i out of %i RX Buffers. %i Bytes\n", 
         dev->devName,count,dev->rxCount,(count*dev->max_frm_size));
   dev->rxCount = count;
   dev->cur_rx_idx = 0;

   printk(KERN_DEBUG "<%s> bd_init: Creating %i TX Buffers. Size=%i Bytes\n", dev->devName,dev->txCount,dev->max_frm_size);
   if ( dev->txCount > 0 ) {
      dev->txBuffers = (struct xauienet_buffer **) 
         kmalloc( (sizeof(struct xauienet_buffer *) * dev->txCount), GFP_KERNEL );
   }

   // Allocate TX buffers
   count = 0;
   for (x=0; x < dev->txCount; x++) {
      dev->txBuffers[x] = (struct xauienet_buffer *) 
         kmalloc( sizeof(struct xauienet_buffer), GFP_KERNEL );

      dev->txBuffers[x]->skb      = NULL;

      ++count;
   }

   printk(KERN_DEBUG "<%s> bd_init: Created  %i out of %i TX Buffers. %i Bytes\n", 
         dev->devName,count,dev->txCount,(count*dev->max_frm_size));
   dev->txCount = count;
   dev->cur_tx_idx = 0;
   dev->last_tx_idx = 0;
   
   return 0;
}


/**
 * xauienet_set_mac_address - Write the MAC address
 * @ndev:	Pointer to the net_device structure
 * @address:	6 byte Address to be written as MAC address
 *
 * This function is called to initialize the MAC address of the Xaui Ethernet
 * core.
 */
static void xauienet_set_mac_address(struct net_device *ndev, void *address)
{
	if (address)
		memcpy(ndev->dev_addr, address, ETH_ALEN);
	if (!is_valid_ether_addr(ndev->dev_addr))
		eth_random_addr(ndev->dev_addr);
}

/**
 * netdev_set_mac_address - Write the MAC address (from outside the driver)
 * @ndev:	Pointer to the net_device structure
 * @p:		6 byte Address to be written as MAC address
 *
 * Return: 0 for all conditions. Presently, there is no failure case.
 *
 * This function is called to initialize the MAC address of the Xaui Ethernet
 * core. It calls the core specific xauienet_set_mac_address. This is the
 * function that goes into net_device_ops structure entry ndo_set_mac_address.
 */
static int netdev_set_mac_address(struct net_device *ndev, void *p)
{
	struct sockaddr *addr = p;
	xauienet_set_mac_address(ndev, addr->sa_data);
	return 0;
}

/**
 * xauienet_device_reset - Reset and initialize the Xaui Ethernet hardware.
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to reset and initialize the Xaui Ethernet core. This
 * is typically called during initialization. It does a reset of the Axi DMA
 * Rx/Tx channels and initializes the Axi DMA BDs. Since Axi DMA reset lines
 * are connected to Xaui Ethernet reset lines, this in turn resets the Axi
 * Ethernet core. No separate hardware reset is done for the Xaui Ethernet
 * core.
 */
static void xauienet_device_reset(struct net_device *ndev)
{
    struct xauienet_device * dev;

    dev = netdev_priv(ndev);

	dev->max_frm_size = XAE_MAX_VLAN_FRAME_SIZE;

	if ((ndev->mtu > XAE_MTU) &&
		(ndev->mtu <= XAE_JUMBO_MTU)) {
		dev->max_frm_size = ndev->mtu + VLAN_ETH_HLEN +
					XAE_TRL_SIZE;
	}

    // Set MAX RX                      
    iowrite32(dev->max_frm_size,dev->reg.maxRxSize);

    // Clear FIFOs
    iowrite32(0x1,dev->reg.fifoClear);
    iowrite32(0x0,dev->reg.fifoClear);

	if (xauienet_dma_bd_init(ndev)) {
		netdev_err(ndev, "<%s> device_reset: descriptor allocation failed\n",
			   __func__);
	}

	/* Sync default options with HW but leave receiver and
	 * transmitter disabled.
	 */
	xauienet_set_mac_address(ndev, NULL);

	netif_trans_update(ndev);
    
    dev->writeCount = 0;
    dev->readCount = 0; 
    dev->descDrops = 0;
    dev->descRetires = 0;
    dev->badSizeMarker = 0;
    dev->badFifoMarker = 0;
    dev->axiWriteErrors = 0;
    dev->axiStreamErrors = 0;
    dev->readSizeErrors = 0;
    dev->overFlows = 0;
    dev->frags = 0;
    dev->csumPart = 0;    
    dev->txFreeCount = 0;
    dev->txBusyCount = 0;
    dev->txBuffErrors = 0;
    dev->rxIrqCount = 0;
    dev->txIrqCount = 0;
    dev->skbAllocErrors = 0;
    dev->skbFreeErrors = 0;    
}

/**
 * xauienet_irq - Tx/Rx Isr.
 * @irq:	irq number
 * @dev_id:	net_device pointer
 *
 * Return: IRQ_HANDLED or IRQ_NONE
 *
 * This is the Axi DMA Tx/Rx Isr. It invokes "napi_schedule" to 
 * complete the RX BD processing. Schedules the TX done
 * tasklet to complete TX DMA.
 */
static irqreturn_t xauienet_irq(int irq, void *dev_id) {
   __u32       stat;
   irqreturn_t ret;

   struct xauienet_device * dev = (struct xauienet_device *)dev_id;

   // Read IRQ Status
   stat = ioread32(dev->reg.intPendAck);

   // Is this the source
   if ( stat != 0 ) {
   
      // Schedule rx
      if ( (stat & RX_IRQ_READY_MASK) != 0 ) {      
      
         ++dev->rxIrqCount;
      
         napi_schedule(&dev->napi);
         
        // Disable interrupt
        iowrite32(0x0,dev->reg.intEnable);         
      }
   
      // Complete tx
      if ( (stat & TX_IRQ_READY_MASK) != 0 ) {
      
        ++dev->txIrqCount;
        
        tasklet_schedule(&dev->tx_done_tasklet);
      }
      
      // Ack Interrupt
      iowrite32(0x1,dev->reg.intPendAck);

      ret = IRQ_HANDLED;
   }
   else ret = IRQ_NONE;

   return(ret);
}

/**
 * xauienet_start_xmit - Starts the transmission.
 * @skb:	sk_buff pointer that contains data to be Txed.
 * @ndev:	Pointer to net_device structure.
 *
 * Return: NETDEV_TX_OK, on success
 *	    NETDEV_TX_BUSY, if any of the descriptors are not free
 *
 * This function is invoked from upper layers to initiate transmission. The
 * function uses the next available free BDs and populates their fields to
 * start the transmission.
 */
static int xauienet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
   dma_addr_t handle;
   __u32      control = 0;
   __s32      idx;
   int        ret = 0;
    
   struct xauienet_device * dev;

   dev = netdev_priv(ndev);

   spin_lock(&dev->tx_lock);

   idx = dev->cur_tx_idx;
   
   if(dev->txBuffers[idx]->skb != NULL) {
     ++dev->txBusyCount;
     ret = NETDEV_TX_BUSY;     
     }
   else {
     if(++dev->cur_tx_idx >= dev->txCount)
       dev->cur_tx_idx = 0;   
     }
   
   spin_unlock(&dev->tx_lock);
   
   if(ret == NETDEV_TX_BUSY) {
     if (!netif_queue_stopped(ndev))
        netif_stop_queue(ndev);
    return ret;
    }  
      
   if(dev->txBuffers[idx]->skb != NULL)
     {
     printk(KERN_ERR "<%s> start_xmit: invalid tx skb storage idx %d\n",dev->devName,idx);
     ++dev->txBuffErrors;
     netif_stop_queue(ndev);
     return NETDEV_TX_BUSY;
     }
   
   dev->txBuffers[idx]->skb = skb;

   if(skb->ip_summed == CHECKSUM_PARTIAL)
     ++dev->csumPart;
   else
     control |= 0x100; // offload not required, inform firmware

#ifdef DEBUG_TX
     {
     int i;
     u32* data;
     data = (u32*)skb->data;
     
     printk(KERN_INFO "Processing tx skb 0x%x size %d\n",(u32)data,skb_headlen(skb)); 
     
     for(i=0;i<(skb_headlen(skb)/4);i++)   
       printk(KERN_INFO "Data[%d] 0x%x\n",i,data[i]);
     }
#endif  

   handle = dma_map_single(ndev->dev.parent,
						 (void*)skb->data,           
						 dev->max_frm_size,    
						 DMA_TO_DEVICE);
   
   dev->txBuffers[idx]->buffHandle = handle;
   
   iowrite32(handle,dev->reg.txPostA);   
   iowrite32(skb_headlen(skb),dev->reg.txPostB);
   iowrite32(control,dev->reg.txPostC);
                            
   ++dev->writeCount;
   ++dev->stats.tx_packets;
   dev->stats.tx_bytes += skb_headlen(skb);
    
   return NETDEV_TX_OK;
}

/**
 * xauienet_xmit_done - Completes the dma transmission.
 * Frees the sk_buff, enables interrupts, and wakes 
 * the network queue.
 * @lp:	xauienet_dev pointer 
 *
 */
static void xauienet_xmit_done(unsigned long lp)
{
   __u32 handle;
   __u32 idx;

   struct xauienet_device * dev = (struct xauienet_device *)lp;
      
   while((handle = ioread32(dev->reg.txFree)) & 0x80000000)
     {
     handle &= 0x7FFFFFFF;

     ++dev->txFreeCount;
     
     dma_unmap_single(dev->ndev->dev.parent,
  				      handle,           
  				      dev->max_frm_size, 
  				      DMA_TO_DEVICE);

     idx = dev->last_tx_idx;

     if(++dev->last_tx_idx >= dev->txCount)
       dev->last_tx_idx = 0;

     if(dev->txBuffers[idx]->buffHandle == handle)
       {
       if(dev->txBuffers[idx]->skb)
         {
         dev_kfree_skb(dev->txBuffers[idx]->skb);                           
         dev->txBuffers[idx]->skb = NULL;
         }
       else
         {
         ++dev->skbFreeErrors;
         printk(KERN_ERR "<%s> xmit_done: NULL tx skb idx %d\n",dev->devName,idx);
         }
       }
       else
         {
         ++dev->skbFreeErrors;
         printk(KERN_ERR "<%s> xmit_done: mismatch TX idx %d 0x%x 0x%x\n",dev->devName,idx,handle,dev->txBuffers[idx]->buffHandle);         
         }
     }
 
      /* Enable the interrupts again */
     iowrite32(0x1,dev->reg.intEnable);    
     
     netif_wake_queue(dev->ndev);
}

/**
 * xauienet_rx - XAUI dma firmware receive routine
 *
 * @ndev:	Pointer to net_device structure.
 *
 * This function reads the firmware fifos for a completed
 * packet dma.  It allocates a new sk_buff after the packet
 * is verified.
 * Return: bytes received or error condition.
 */
static ssize_t xauienet_rx(struct net_device *ndev, struct sk_buff **rskb) {
   dma_addr_t handle;
   __u32      status;
   __u32      size;
   __u32      frag = 0;
   __s32      idx;
   ssize_t    ret = 0;
   
   struct sk_buff *skb, *new_skb;

   struct xauienet_device * dev;

   dev = netdev_priv(ndev);
   
   *rskb = NULL;

   // Receive queue entry
   handle = ioread32(dev->reg.rxPend);
   if((handle & 0x80000000) == 0 ) {
      return 0;
   }
   handle &= 0x7FFFFFFF;

   // Find Buffer
   idx = dev->cur_rx_idx;
   if(++dev->cur_rx_idx >= dev->rxCount)
     dev->cur_rx_idx = 0;
      
   // Read size
   do { 
      size = ioread32(dev->reg.rxPend);
   } while ((size & 0x80000000) == 0);

   // Bad marker
   if ( (size & 0xFF000000) != 0xE0000000 ) {
      //printk(KERN_ERR "<%s> rx: Bad FIFO size marker 0x%.8x\n", dev->devName,size);
      ++dev->badSizeMarker;
      ret = ERR_DRIVER;
   }
   size &= 0xFFFFFF;
   ret = size;

   // Bad size
   if ( dev->max_frm_size < size ) {
      ++dev->readSizeErrors;
      //printk(KERN_ERR "<%s> rx: Bad frame size %d idx %d\n", dev->devName, size, idx);   
      ret = ERR_BUFF_OFLOW;
   }
   
   // Get status
   do { 
      status = ioread32(dev->reg.rxPend);
   } while ((status & 0x80000000) == 0);

   // Bad marker
   if ( (status & 0xF0000000) != 0xF0000000 ) {
      //printk(KERN_ERR "<%s> rx: Bad FIFO status marker 0x%.8x\n", dev->devName,size);
      ++dev->badFifoMarker;
      ret = ERR_DRIVER;
   }

   // Check for errors
   if ( (status & 0x00FF0000) != 0 ) {
      ++dev->axiStreamErrors;
      ret = ERR_AXI_STREAM;
   }

   if ( (status & 0x01000000) != 0 ) {
      ++dev->axiWriteErrors;
      ret = ERR_AXI_WRITE;
   }
   
   if ( (status & 0x02000000) != 0 ) {
      ++dev->overFlows;
      ret = ERR_DMA_OFLOW;
   }

   /* check for firmware fragmentation flag: if set checksum offload was not performed */
   if ( (status & 0x00000100) != 0 ) {
     frag = 1;
     ++dev->frags;
   }
      
   if(handle != dev->rxBuffers[idx]->buffHandle) {
     //printk(KERN_ERR "<%s> rx: Mismatch dma handle idx %d 0x%x 0x%x\n", dev->devName,idx,handle,dev->rxBuffers[idx]->buffHandle);
     ++dev->descDrops;
     return(ERR_DRIVER);     
    }
   else {     
     skb = dev->rxBuffers[idx]->skb;
     *rskb = skb;

     /* dismiss frames with error status */
     if(ret < 0)
       {
       ++dev->descRetires;
       skb_put(skb,dev->max_frm_size);
       ret = ERR_BUFF_OFLOW;
       size = 0;
       }
     else
       {
       dma_unmap_single(dev->ndev->dev.parent,
  				  handle,           
  				  dev->max_frm_size, 
  				  DMA_FROM_DEVICE);
 
       skb_put(skb, size);
       skb->protocol = eth_type_trans(skb, ndev);       
       skb->ip_summed = (dev->ip_csum && !frag) ? CHECKSUM_UNNECESSARY : CHECKSUM_NONE;
       skb->csum_level = (skb->ip_summed == CHECKSUM_UNNECESSARY) ? 1 : 0;
       }
   
     new_skb = netdev_alloc_skb(ndev, dev->max_frm_size);
     if (new_skb == NULL) {
         ++dev->skbAllocErrors;
	     dev_err(dev->device, "<%s> rx: No memory for skb\n\r",dev->devName);
	     return(ERR_DRIVER);
     }
     
     dev->rxBuffers[idx]->buffHandle = dma_map_single(ndev->dev.parent,
						   new_skb->data,           
						   dev->max_frm_size,    
						   DMA_FROM_DEVICE);

     dev->rxBuffers[idx]->skb = new_skb;

     iowrite32(dev->rxBuffers[idx]->buffHandle,dev->reg.rxFree);
   }   

   ++dev->readCount;
   ++dev->stats.rx_packets;
   dev->stats.rx_bytes += size;

   return(ret);
}

/**
 * xauienet_recv - Is called from Axi DMA Rx Isr to complete the received
 *		  BD processing.
 * @ndev:	Pointer to net_device structure.
 * @budget:	NAPI budget
 *
 * This function is invoked from the Axi DMA Rx isr(poll) to process the Rx BDs
 * It does minimal processing and invokes "netif_receive_skb" to complete
 * further processing.
 * Return: Number of BD's processed.
 */
static int xauienet_recv(struct net_device *ndev, int budget)
{
	struct sk_buff *skb;
    struct xauienet_device * dev;    
    unsigned int numrecv = 0;
    
    dev = netdev_priv(ndev);
    
    while (numrecv < budget)
      {
      ssize_t bytes;
      
      bytes = xauienet_rx(ndev,&skb);

      if(bytes > 0)
        {
	    netif_receive_skb(skb);
        ++numrecv;
        }
      else if((bytes < 0) && skb)
        {
        dev_kfree_skb(skb);
        break;
        }
      else break;
      }
      
	return numrecv;
}

/**
 * xauienet_rx_poll - Poll routine for rx packets (NAPI)
 * @napi:	napi structure pointer
 * @quota:	Max number of rx packets to be processed.
 *
 * This is the poll routine for rx part.
 * It will process the packets maximum quota value.
 *
 * Return: number of packets received
 */
static int xauienet_rx_poll(struct napi_struct *napi, int quota)
{
	int work_done = 0;
    __u32 status;

	struct xauienet_device *dev = container_of(napi,
					struct xauienet_device, napi);

	spin_lock(&dev->rx_lock);
    status = ioread32(dev->reg.fifoValid);
	while ((status & 0x1) && (work_done < quota)) {
        work_done += xauienet_recv(dev->ndev, quota - work_done);
        status = ioread32(dev->reg.fifoValid);
	}

	if (work_done < quota) {
		napi_complete(napi);
                
        /* Enable the interrupts again */
        iowrite32(0x1,dev->reg.intEnable);        
	}

	spin_unlock(&dev->rx_lock);

	return work_done;
}

/**
 * xauienet_open - Driver open routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *	    non-zero error value on failure
 *
 * This is the driver open routine. It enables AXI DMA channels.
 * It also allocates interrupt service routines, enables the interrupt lines
 * and ISR handling. Xaui Ethernet core and Axi DMA core are reset. Buffer
 * descriptors are initialized.
 */
static int xauienet_open(struct net_device *ndev)
{
	struct xauienet_device *dev = netdev_priv(ndev);

	dev_dbg(&ndev->dev, "xauienet_open()\n");

	xauienet_device_reset(ndev);

    netif_carrier_off(ndev);

    // Online bits = 1, Ack bit = 0
    iowrite32(0x1,dev->reg.onlineAck);

    // init hw
    xauienet_mac_up((__u32*)dev->mapped_bsi,ndev);    

    /* Enable tasklets for transmit complete handling */
    tasklet_init(&dev->tx_done_tasklet, xauienet_xmit_done,
		     (unsigned long) dev);

    // Enable rx and tx
    iowrite32(0x1,dev->reg.rxEnable);
    iowrite32(0x1,dev->reg.txEnable);
 
    // Request IRQ from OS.
    if (request_irq( dev->irq, xauienet_irq, 0, MODULE_NAME, (void*)dev) < 0 ) {
       printk(KERN_ERR"<%s> Init: Unable to allocate IRQ.",dev->devName);
       return (-1);
    }

    // Enable interrupt
    iowrite32(0x1,dev->reg.intPendAck);
    iowrite32(0x1,dev->reg.intEnable);

	napi_enable(&dev->napi);
    
    netif_carrier_on(ndev);
    netif_start_queue(ndev);

	return 0;
}

/**
 * xauienet_stop - Driver stop routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *
 * This is the driver stop routine. It calls phy_disconnect to stop the PHY
 * device. It also removes the interrupt handlers and disables the interrupts.
 * The Axi DMA Tx/Rx BDs are released.
 */
static int xauienet_stop(struct net_device *ndev)
{
    struct xauienet_device * dev = netdev_priv(ndev);

	dev_dbg(&ndev->dev, "xauienet_stop()\n");

    netif_stop_queue(ndev);
	napi_disable(&dev->napi);

    // Disable interrupt
    iowrite32(0x0,dev->reg.intEnable);

    // Release IRQ
    free_irq(dev->irq, dev);

    // Disable rx and tx
    iowrite32(0x0,dev->reg.rxEnable);
    iowrite32(0x0,dev->reg.txEnable);

	tasklet_kill(&dev->tx_done_tasklet);

    // Clear FIFOs
    iowrite32(0x1,dev->reg.fifoClear);
    iowrite32(0x0,dev->reg.fifoClear);

    // reset hw
    xauienet_mac_down(ndev);

    netif_carrier_off(ndev);

    // Offline bits = 0, Ack bit = 0
    iowrite32(0x0,dev->reg.onlineAck);    
    
    xauienet_dma_bd_release(ndev);

	return 0;
}

/**
 * xauienet_change_mtu - Driver change mtu routine.
 * @ndev:	Pointer to net_device structure
 * @new_mtu:	New mtu value to be applied
 *
 * Return: Always returns 0 (success).
 *
 * This is the change mtu driver routine. It checks if the Xaui Ethernet
 * hardware supports jumbo frames before changing the mtu. This can be
 * called only when the device is not up.
 */
static int xauienet_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct xauienet_device *dev = netdev_priv(ndev);

	if (netif_running(ndev))
		return -EBUSY;

	if ((new_mtu + VLAN_ETH_HLEN +
		XAE_TRL_SIZE) > dev->rxmem)
		return -EINVAL;

	if ((new_mtu > XAE_JUMBO_MTU) || (new_mtu < 64))
		return -EINVAL;

	ndev->mtu = new_mtu;

	return 0;
}

/**
 * xemacps_update_stats - Update the statistic structure entries from
 * the corresponding emacps hardware statistic registers
 * @data: Used for net_local instance pointer
 */
static void xauienet_update_stats(unsigned long data)
{
	struct xauienet_device *dev = (struct xauienet_device *)data;
	struct net_device_stats *nstat = &dev->stats;

    Xaui_Counters* counters;
    
    counters = &(_Xaui_Registers->counters);

	nstat->rx_crc_errors     = counters->rx_crc;    
	nstat->rx_fifo_errors    = counters->rx_overflow+counters->rx_drops;
    nstat->rx_errors         = dev->descDrops + dev->descRetires + dev->skbAllocErrors;
	nstat->rx_length_errors  = dev->overFlows + dev->readSizeErrors + dev->badSizeMarker;
	nstat->rx_frame_errors   = dev->axiStreamErrors + dev->axiWriteErrors + dev->badFifoMarker;

	nstat->tx_fifo_errors    = counters->tx_underflow;
    nstat->tx_carrier_errors = counters->tx_link_not_ready;
	nstat->tx_errors         = dev->txBuffErrors + dev->skbFreeErrors;
    nstat->tx_dropped        = dev->txBusyCount;

	//nstat->rx_missed_errors  = 0;
	//nstat->tx_aborted_errors = 0;
	//nstat->collisions        = 0;
}

/**
 * xauienet_get_stats - get device statistics
 * @ndev: network device
 * Return: Pointer to network device statistics
 */
static struct net_device_stats
*xauienet_get_stats(struct net_device *ndev)
{
	struct xauienet_device *dev = netdev_priv(ndev);
	struct net_device_stats *nstat = &dev->stats;

	xauienet_update_stats((unsigned long)dev);
	return nstat;
}

static const struct net_device_ops xauienet_netdev_ops = {
	.ndo_open = xauienet_open,
	.ndo_stop = xauienet_stop,
	.ndo_start_xmit = xauienet_start_xmit,
	.ndo_change_mtu	= xauienet_change_mtu,
	.ndo_set_mac_address = netdev_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
    .ndo_get_stats     = xauienet_get_stats,
};

static struct ethtool_ops xauienet_ethtool_ops = {
    .get_strings       = xauienet_get_ethtool_strings,
    .get_ethtool_stats = xauienet_get_ethtool_stats,
    .get_sset_count    = xauienet_get_sset_count,
    .get_ringparam     = xauienet_get_ringparam,
    .set_ringparam     = xauienet_set_ringparam,
};

/**
 * xauienet_probe - module probe routine
 * @pdev: platform device
 * Return: 0 success or error status
 */
static int xauienet_probe(struct platform_device *pdev) {
   int ret;

   struct net_device *ndev;

   struct xauienet_device *dev;
   
   ndev = alloc_etherdev(sizeof(*dev));
   if (!ndev)
	   return -ENOMEM;
          
   platform_set_drvdata(pdev, ndev);

   SET_NETDEV_DEV(ndev, &pdev->dev);
   ndev->flags |= IFF_MULTICAST;  /* clear multicast */
   ndev->netdev_ops = &xauienet_netdev_ops;
   ndev->ethtool_ops = &xauienet_ethtool_ops;
   
   dev = netdev_priv(ndev);
   dev->ndev = ndev;

   dev->baseAddr = pdev->resource[0].start;
   dev->baseSize = (pdev->resource[0].end - pdev->resource[0].start) + 1;
   dev->devName  = pdev->name + 9;
   dev->device   = &pdev->dev;

   // Set idx
   if ( strcmp(dev->devName,"xaui_enet_0") == 0 ) dev->idx = 3;
   else goto free_netdev;

   // Get and map register space
   request_mem_region(dev->baseAddr,dev->baseSize,MODULE_NAME);
   dev->virtAddr = (char *) ioremap(dev->baseAddr,dev->baseSize);
   setRegisterPointers(dev);

   // Configure buffers
   dev->txCount = cfgTxCount;
   dev->rxCount = cfgRxCount;
   dev->rxmem   = cfgRxSize;
   
   spin_lock_init(&dev->rx_lock);
   spin_lock_init(&dev->tx_lock);
   
   // Get IRQ from pci_dev structure. 
   dev->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);;
   printk(KERN_INFO "<%s> init: IRQ %d\n", dev->devName, dev->irq);

   /* Map the AXI space */
   dev->mapped_base = ioremap(MAC_BASE_ADDR,sizeof(Xaui_Registers));
   if (dev->mapped_base == (void *) -1)
     {
     printk(KERN_ERR "<%s> init: Failure mapping XAUI MAC register space.\n",dev->devName);
     goto free_netdev;
     }

   _Xaui_Registers = (Xaui_Registers*)dev->mapped_base;

   /* Map the BSI space */
   dev->mapped_bsi = ioremap(BSI_BASE_ADDR,32);
   if (dev->mapped_bsi == (void *) -1)
     {
     printk(KERN_ERR "<%s> init: Failure mapping XAUI mac address BSI space.\n",dev->devName);
     goto free_netdev; 
     }
      
   xauienet_set_mac_address(ndev, (void *) dev->mapped_bsi);
   
   /* Set hardware checksum options - ON by default */
   ndev->hw_features = NETIF_F_IP_CSUM | NETIF_F_RXCSUM;
   ndev->features = ndev->hw_features;
   dev->ip_csum = 1;

   netif_napi_add(ndev, &dev->napi, xauienet_rx_poll, XAUIENET_NAPI_WEIGHT);

   ret = register_netdev(dev->ndev);
   if (ret) {
	   dev_err(dev->device, "<%s> register_netdev() error (%i)\n",dev->devName, ret);
	   goto free_netdev;
   }

   return 0;

free_netdev:
   free_netdev(ndev);

   return -1;
}

/**
 * xauienet_remove - module remove routine
 * @pdev: platform device
 * Return: 0 success or error status
 */
static int xauienet_remove(struct platform_device *pdev) {

	struct net_device *ndev = platform_get_drvdata(pdev);
    struct xauienet_device *dev = netdev_priv(ndev);
    
    netif_napi_del(&dev->napi);
    unregister_netdev(dev->ndev);
    free_netdev(dev->ndev);
       
    // Release register space
    iounmap(dev->virtAddr);
    release_mem_region(dev->baseAddr,dev->baseSize);
    iounmap(dev->mapped_base);
    iounmap(dev->mapped_bsi);
   
    // Free device area
    kfree(dev);
        
	printk(KERN_INFO "<%s> remove: unregistered\n", MODULE_NAME);
    
	return 0;
}

static int xauienet_runtime_nop(struct device *dev) {
   return 0;
}

static const struct dev_pm_ops xauienet_ops = {
   .runtime_suspend = xauienet_runtime_nop,
   .runtime_resume = xauienet_runtime_nop,
};

static struct of_device_id xauienet_match[] = {
   { .compatible = MODULE_NAME, },
   { /* This is filled with module_parm */ },
   { /* Sentinel */ },
};


module_param(cfgTxCount,uint,0);
MODULE_PARM_DESC(cfgTxCount, "TX buffer count");

module_param(cfgRxCount,uint,0);
MODULE_PARM_DESC(cfgRxCount, "RX buffer count");

module_param(cfgRxSize,uint,0);
MODULE_PARM_DESC(cfgRxSize, "RX max buffer size");

MODULE_DEVICE_TABLE(of, xauienet_match);
module_param_string(of_id, xauienet_match[1].compatible, 128, 0);
MODULE_PARM_DESC(of_id, "Id of the device to be handled by driver");

static struct platform_driver xauienet_pdrv = {
   .probe  = xauienet_probe,
   .remove = xauienet_remove,
   .driver = {
      .name = DRIVER_NAME,
      .owner = THIS_MODULE,
      .pm = &xauienet_ops,
      .of_match_table = of_match_ptr(xauienet_match),
   },
};

module_platform_driver(xauienet_pdrv);

MODULE_AUTHOR("SLAC");
MODULE_DESCRIPTION("Xaui Ethernet/MAC driver");
MODULE_LICENSE("GPL v2");

