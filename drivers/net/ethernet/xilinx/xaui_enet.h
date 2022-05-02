#ifndef __XAUIENET_H__
#define __XAUIENET_H__

// Return Errors
#define ERR_DRIVER     -1
#define ERR_BUFF_OFLOW -2
#define ERR_DMA_OFLOW  -3
#define ERR_AXI_WRITE  -4
#define ERR_AXI_STREAM -5

/* @brief

Used to print driver and firmware statistics

*/

enum {
XAUI_RXCNT = 0,
XAUI_RXIRQCNT,
XAUI_RXSZERRS,
XAUI_RXOVRFLOW,
XAUI_RXFRAGS,
XAUI_RXDSCRTIRE,
XAUI_RXDSCDROP,
XAUI_RXSKBERRS,
XAUI_RXMADSZMRK,
XAUI_RXBADFIFMRK,
XAUI_AXISTRERRS,
XAUI_AXIWRERRS,
XAUI_TXCNT,
XAUI_TXFREECNT,
XAUI_TXIRQCNT,
XAUI_TXSZERRS,
XAUI_TXSKBERRS,
XAUI_TXBSYCNT,
XAUI_TXBUFERRS,
XAUI_TXCSUMPART,
XAUI_MACRXFRAMES,
XAUI_MACRXPAUSE,
XAUI_MACRXOVRFLW,
XAUI_MACRXCRCERR,
XAUI_MACRXDROPS,
XAUI_MACTXFRAMES,
XAUI_MACTXPAUSE,
XAUI_MACTXUNDRRN,
XAUI_MACTXLKNRDY};

/* @brief

Used to Reset per packet counters (see "Xaui_Counters")

*/

enum {
XAUI_COUNTERS_RESET   = 0, 
XAUI_COUNTERS_MBZ     = XAUI_COUNTERS_RESET + 1, XAUI_COUNTERS_MBZ_LENGTH = 31};

/* @brief

Used to Reset the PHY

*/

enum {
XAUI_PHY_RESET   = 0, 
XAUI_PHY_MBZ     = XAUI_PHY_RESET + 1, XAUI_PHY_MBZ_LENGTH = 31};
     
/* @brief

This is the bit field structure of the XAUI core configuration vector as defined by the 
"Configuration and Status Vectors" section of the Xilinx XAUI documentation. 
(see page 95 of XAUI v12.1 Xilinx PG053) 

*/

enum {
XAUI_CORE_LOOPBACK     = 0, 
XAUI_CORE_POWER_DOWN   = XAUI_CORE_LOOPBACK + 1,
XAUI_CORE_RESET_FAULTS = XAUI_CORE_POWER_DOWN + 1,
XAUI_CORE_RESET_LINK   = XAUI_CORE_RESET_FAULTS + 1,
XAUI_CORE_TEST         = XAUI_CORE_RESET_LINK + 1,
XAUI_CORE_TEST_PATTERN = XAUI_CORE_TEST+ 1,                                      XAUI_CORE_TEST_PATTERN_LENGTH = 2,
XAUI_CORE_MBZ          = XAUI_CORE_TEST_PATTERN + XAUI_CORE_TEST_PATTERN_LENGTH, XAUI_CORE_MBZ_LENGTH = 25};
   
/* @brief

Amount of pause time for a sent pause frame

*/

enum {
XAUI_PAUSE_GAP     = 0,                                      XAUI_PAUSE_GAP_LENGTH = 16,
XAUI_PAUSE_GAP_MBZ = XAUI_PAUSE_GAP + XAUI_PAUSE_GAP_LENGTH, XAUI_PAUSE_GAP_MBZ_LENGTH = 16};
     
/* @brief

This is the bit field structure of the XAUI core status vector as defined by the 
"Configuration and Status Vectors" section of the Xilinx XAUI documentation. 
(see page 96 of XAUI v12.1 Xilinx PG053) 

*/

enum {
XAUI_STATUS_TX_FAULT = 0,                                                        // transmit fault (reset by "Xaui_Core_State.reset_fault") 
XAUI_STATUS_RX_FAULT = XAUI_STATUS_TX_FAULT + 1,                                 // receive fault  (reset by "Xaui_Core_State.reset_fault")
XAUI_STATUS_SYNCHED  = XAUI_STATUS_RX_FAULT + 1, XAUI_STATUS_SYNCHED_LENGTH = 4, // receive synchronization with one bit per lane(see table 5-47)
XAUI_STATUS_ALIGNED  = XAUI_STATUS_SYNCHED + XAUI_STATUS_SYNCHED_LENGTH,         // All 4 lanes aligned (table 5-32)
XAUI_STATUS_RX_LINK  = XAUI_STATUS_ALIGNED + 1,                                  // Receive link/up/down (reset by "Xaui_Core_State.reset_rx_link")
XAUI_STATUS_MBZ      = XAUI_STATUS_RX_LINK + 1,  XAUI_STATUS_MBZ_LENGTH = 24};   // 32-bit padding (Must Be Zero)

/* @brief

This is the bit field structure of the XAUI core debug vector as defined by the 
"Configuration and Status Vectors" section of the Xilinx XUAI documentation. 
(see page 97 of XAUI v12.1. Xilinx PG053) 

*/

enum {
XAUI_DEBUG_PHASED  = 0,                                                     // TX phase alignment completed
XAUI_DEBUG_SYNCHED = XAUI_DEBUG_PHASED + 1,  XAUI_DEBUG_SYNCHED_LENGTH = 4, // receive synchronization with one bit per lane(see table 5-47)
XAUI_DEBUG_ALIGNED = XAUI_DEBUG_SYNCHED + XAUI_DEBUG_SYNCHED_LENGTH,        // All 4 lanes aligned (table 5-32)
XAUI_DEBUG_MBZ     = XAUI_DEBUG_ALIGNED + 1, XAUI_DEBUG_MBZ_LENGTH = 26};   // 32-bit padding (Must Be Zero)

/* @brief

This is the bit field structure of the XAUI core control vector.

*/

enum {
XAUI_CTL_RX_SHIFT    = 0, XAUI_CTL_RX_SHIFT_LENGTH = 4, // RX header shift byte count
XAUI_CTL_TX_SHIFT    = XAUI_CTL_RX_SHIFT + XAUI_CTL_RX_SHIFT_LENGTH,  XAUI_CTL_TX_SHIFT_LENGTH = 12, // TX header shift byte count
XAUI_CTL_MAC_FILTER  = XAUI_CTL_TX_SHIFT + XAUI_CTL_TX_SHIFT_LENGTH, XAUI_CTL_MAC_FILTER_LENGTH = 1,     // MAC filter enable
XAUI_CTL_IP_CSUM   = XAUI_CTL_MAC_FILTER + XAUI_CTL_MAC_FILTER_LENGTH, XAUI_CTL_IP_CSUM_LENGTH = 1,     // IP checksum
XAUI_CTL_TCP_CSUM  = XAUI_CTL_IP_CSUM + XAUI_CTL_IP_CSUM_LENGTH, XAUI_CTL_TCP_CSUM_LENGTH = 1,     // TCP checksum
XAUI_CTL_UDP_CSUM  = XAUI_CTL_TCP_CSUM + XAUI_CTL_TCP_CSUM_LENGTH, XAUI_CTL_UDP_CSUM_LENGTH = 1};     // UDP checksum

/* @brief

This is the bit field structure of the XAUI core configuration vector as defined by the 
"Configuration and Status Vectors" section of the Xilinx XAUI documentation. 
(see page 95 of XUAI v21. Xilinx PG053) 

*/

typedef struct {
  __u32 counters;    // Reset RX and TX related counters (see
  __u32 phy;         // Reset phy interface
  __u32 core;        // Core configuration;
  __u32 spare0;      // Spare bytes
  __u32 pause;       // Pause time in one (1) Nanosecond tics  
  __u32 mac[2];      // Own MAC address (high order two bytes are MBZ)
  __u32 rssiIp;      // IP address for RSSI
  __u32 status;      // Xilinx Core status values
  __u32 debug;       // Xilinx Core debug configuration
  __u32 spare[4];    // Don't care... (for alignment)    
  __u32 ctl;         // Control bits enable/disable hdr shift, csum offload, MAC filter
  __u8  hole[196];   // Don't care... (for alignment)    
} Xaui_Config;
 
/* @brief

These are the counters the firmware maintains on a per-packet basis

*/

typedef struct {
  __u32 rx;                // # of received frames
  __u32 tx;                // # of transmitted frames
  __u32 rxp;               // # of received pause frames
  __u32 txp;               // # of transmitted pause frames
  __u32 rx_overflow;       // # receive buffering errors
  __u32 rx_crc;            // # of RX CRC errors
  __u32 tx_underflow;      // # of transmit buffering errors;
  __u32 tx_link_not_ready; // # of link not ready errors;
  __u32 tx_local_fault;
  __u32 rx_local_fault;
  __u32 sync_status[4];
  __u32 alignment;
  __u32 rx_link_status;
  __u32 rx_drops;          // # of RX FIFO drops
} Xaui_Counters;
 
/* @brief

TBD.

*/

typedef struct {
  Xaui_Config   config;    // configuration parameters
  Xaui_Counters counters;  //  counters...
} Xaui_Registers;

#endif /* __XAUIENET_H */

