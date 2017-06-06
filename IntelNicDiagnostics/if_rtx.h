/******************************************************************************

Copyright (c) 2001-2016, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

3. Neither the name of the Intel Corporation nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/

#include "e1000_osdep.h"
#include "e1000_hw.h"
#include <stdio.h>



#define	ETHERTYPE_PUP		0x0200	/* PUP protocol */
#define	ETHERTYPE_IP		0x0800	/* IP protocol */
#define ETHERTYPE_ARP		0x0806	/* Addr. resolution protocol */
#define ETHERTYPE_REVARP	0x8035	/* reverse Addr. resolution protocol */
#define	ETHERTYPE_VLAN		0x8100 /* IEEE 802.1Q VLAN tagging */
#define	ETHERTYPE_LOOPBACK	0x9000	/* used to test interfaces */


#define MCLBYTES        2048            /* large enough for ether MTU */
#define MJUMPAGESIZE    4096  // PAGE_SIZE
#define MJUM2PAGESIZE   (4096 *2)  // 8192
#define MJUM9BYTES      (9 * 1024)      /* jumbo cluster 9k */
#define MJUM16BYTES     (16 * 1024)     /* jumbo cluster 16k */

#define MAX_INTS_PER_SEC	8000
#define DEFAULT_ITR		(1000000000/(MAX_INTS_PER_SEC * 256))

/* Tunables */

/*
* EM_TXD: Maximum number of Transmit Descriptors
* Valid Range: 80-256 for 82542 and 82543-based adapters
*              80-4096 for others
* Default Value: 256
*   This value is the number of transmit descriptors allocated by the driver.
*   Increasing this value allows the driver to queue more transmits. Each
*   descriptor is 16 bytes.
*   Since TDLEN should be multiple of 128bytes, the number of transmit
*   desscriptors should meet the following condition.
*      (num_tx_desc * sizeof(struct e1000_tx_desc)) % 128 == 0
*/
#define EM_MIN_TXD		80
#define EM_MAX_TXD		4096
#ifdef EM_MULTIQUEUE
#define EM_DEFAULT_TXD		4096
#else
#define EM_DEFAULT_TXD		1024
#endif

/*
* EM_RXD - Maximum number of receive Descriptors
* Valid Range: 80-256 for 82542 and 82543-based adapters
*              80-4096 for others
* Default Value: 256
*   This value is the number of receive descriptors allocated by the driver.
*   Increasing this value allows the driver to buffer more incoming packets.
*   Each descriptor is 16 bytes.  A receive buffer is also allocated for each
*   descriptor. The maximum MTU size is 16110.
*   Since TDLEN should be multiple of 128bytes, the number of transmit
*   desscriptors should meet the following condition.
*      (num_tx_desc * sizeof(struct e1000_tx_desc)) % 128 == 0
*/
#define EM_MIN_RXD		80
#define EM_MAX_RXD		4096
#ifdef EM_MULTIQUEUE
#define EM_DEFAULT_RXD		4096
#else
#define EM_DEFAULT_RXD		1024
#endif

/*
* EM_TIDV - Transmit Interrupt Delay Value
* Valid Range: 0-65535 (0=off)
* Default Value: 64
*   This value delays the generation of transmit interrupts in units of
*   1.024 microseconds. Transmit interrupt reduction can improve CPU
*   efficiency if properly tuned for specific network traffic. If the
*   system is reporting dropped transmits, this value may be set too high
*   causing the driver to run out of available transmit descriptors.
*/
#define EM_TIDV                         64

/*
* EM_TADV - Transmit Absolute Interrupt Delay Value
* (Not valid for 82542/82543/82544)
* Valid Range: 0-65535 (0=off)
* Default Value: 64
*   This value, in units of 1.024 microseconds, limits the delay in which a
*   transmit interrupt is generated. Useful only if EM_TIDV is non-zero,
*   this value ensures that an interrupt is generated after the initial
*   packet is sent on the wire within the set amount of time.  Proper tuning,
*   along with EM_TIDV, may improve traffic throughput in specific
*   network conditions.
*/
#define EM_TADV                         64

/*
* EM_RDTR - Receive Interrupt Delay Timer (Packet Timer)
* Valid Range: 0-65535 (0=off)
* Default Value: 0
*   This value delays the generation of receive interrupts in units of 1.024
*   microseconds.  Receive interrupt reduction can improve CPU efficiency if
*   properly tuned for specific network traffic. Increasing this value adds
*   extra latency to frame reception and can end up decreasing the throughput
*   of TCP traffic. If the system is reporting dropped receives, this value
*   may be set too high, causing the driver to run out of available receive
*   descriptors.
*
*   CAUTION: When setting EM_RDTR to a value other than 0, adapters
*            may hang (stop transmitting) under certain network conditions.
*            If this occurs a WATCHDOG message is logged in the system
*            event log. In addition, the controller is automatically reset,
*            restoring the network connection. To eliminate the potential
*            for the hang ensure that EM_RDTR is set to 0.
*/
#ifdef EM_MULTIQUEUE
#define EM_RDTR                         64
#else
#define EM_RDTR                         0
#endif

/*
* Receive Interrupt Absolute Delay Timer (Not valid for 82542/82543/82544)
* Valid Range: 0-65535 (0=off)
* Default Value: 64
*   This value, in units of 1.024 microseconds, limits the delay in which a
*   receive interrupt is generated. Useful only if EM_RDTR is non-zero,
*   this value ensures that an interrupt is generated after the initial
*   packet is received within the set amount of time.  Proper tuning,
*   along with EM_RDTR, may improve traffic throughput in specific network
*   conditions.
*/
#ifdef EM_MULTIQUEUE
#define EM_RADV                         128
#else
#define EM_RADV                         64
#endif

/*
* This parameter controls the max duration of transmit watchdog.
*/
#define EM_WATCHDOG                   (10 * hz)

/*
* This parameter controls when the driver calls the routine to reclaim
* transmit descriptors.
*/
#define EM_TX_CLEANUP_THRESHOLD	(adapter->num_tx_desc / 8)

/*
* This parameter controls whether or not autonegotation is enabled.
*              0 - Disable autonegotiation
*              1 - Enable  autonegotiation
*/
#define DO_AUTO_NEG                     1

/*
* This parameter control whether or not the driver will wait for
* autonegotiation to complete.
*              1 - Wait for autonegotiation to complete
*              0 - Don't wait for autonegotiation to complete
*/
#define WAIT_FOR_AUTO_NEG_DEFAULT       0

/* Tunables -- End */

#define AUTONEG_ADV_DEFAULT	(ADVERTISE_10_HALF | ADVERTISE_10_FULL | \
				ADVERTISE_100_HALF | ADVERTISE_100_FULL | \
				ADVERTISE_1000_FULL)

#define AUTO_ALL_MODES		0

/* PHY master/slave setting */
#define EM_MASTER_SLAVE		e1000_ms_hw_default

/*
* Micellaneous constants
*/
#define EM_VENDOR_ID                    0x8086
#define EM_FLASH                        0x0014 

#define EM_JUMBO_PBA                    0x00000028
#define EM_DEFAULT_PBA                  0x00000030
#define EM_SMARTSPEED_DOWNSHIFT         3
#define EM_SMARTSPEED_MAX               15
#define EM_MAX_LOOP			10

#define MAX_NUM_MULTICAST_ADDRESSES     128
#define PCI_ANY_ID                      (~0U)
#define ETHER_ALIGN                     2
#define EM_FC_PAUSE_TIME		0x0680
#define EM_EEPROM_APME			0x400;
#define EM_82544_APME			0x0004;

/*
* Driver state logic for the detection of a hung state
* in hardware.  Set TX_HUNG whenever a TX packet is used
* (data is sent) and clear it when txeof() is invoked if
* any descriptors from the ring are cleaned/reclaimed.
* Increment internal counter if no descriptors are cleaned
* and compare to TX_MAXTRIES.  When counter > TX_MAXTRIES,
* reset adapter.
*/
#define EM_TX_IDLE			0x00000000
#define EM_TX_BUSY			0x00000001
#define EM_TX_HUNG			0x80000000
#define EM_TX_MAXTRIES			10

#define PCICFG_DESC_RING_STATUS		0xe4
#define FLUSH_DESC_REQUIRED		0x100

/*
* TDBA/RDBA should be aligned on 16 byte boundary. But TDLEN/RDLEN should be
* multiple of 128 bytes. So we align TDBA/RDBA on 128 byte boundary. This will
* also optimize cache line size effect. H/W supports up to cache line size 128.
*/
#define EM_DBA_ALIGN			128

/*
* See Intel 82574 Driver Programming Interface Manual, Section 10.2.6.9
*/
#define TARC_COMPENSATION_MODE	(1 << 7)	/* Compensation Mode */
#define TARC_SPEED_MODE_BIT 	(1 << 21)	/* On PCI-E MACs only */
#define TARC_MQ_FIX		(1 << 23) | \
				(1 << 24) | \
				(1 << 25)	/* Handle errata in MQ mode */
#define TARC_ERRATA_BIT 	(1 << 26)	/* Note from errata on 82574 */

/* PCI Config defines */
#define EM_BAR_TYPE(v)		((v) & EM_BAR_TYPE_MASK)
#define EM_BAR_TYPE_MASK	0x00000001
#define EM_BAR_TYPE_MMEM	0x00000000
#define EM_BAR_TYPE_FLASH	0x0014 
#define EM_BAR_MEM_TYPE(v)	((v) & EM_BAR_MEM_TYPE_MASK)
#define EM_BAR_MEM_TYPE_MASK	0x00000006
#define EM_BAR_MEM_TYPE_32BIT	0x00000000
#define EM_BAR_MEM_TYPE_64BIT	0x00000004
#define EM_MSIX_BAR		3	/* On 82575 */

/* More backward compatibility */
#if __FreeBSD_version < 900000
#define SYSCTL_ADD_UQUAD SYSCTL_ADD_QUAD
#endif

/* Defines for printing debug information */
#define DEBUG_INIT  0
#define DEBUG_IOCTL 0
#define DEBUG_HW    0

#define INIT_DEBUGOUT(S)            if (DEBUG_INIT)  RtPrintf(S "\n")
#define INIT_DEBUGOUT1(S, A)        if (DEBUG_INIT)  RtPrintf(S "\n", A)
#define INIT_DEBUGOUT2(S, A, B)     if (DEBUG_INIT)  RtPrintf(S "\n", A, B)
#define IOCTL_DEBUGOUT(S)           if (DEBUG_IOCTL) RtPrintf(S "\n")
#define IOCTL_DEBUGOUT1(S, A)       if (DEBUG_IOCTL) RtPrintf(S "\n", A)
#define IOCTL_DEBUGOUT2(S, A, B)    if (DEBUG_IOCTL) RtPrintf(S "\n", A, B)
#define HW_DEBUGOUT(S)              if (DEBUG_HW) RtPrintf(S "\n")
#define HW_DEBUGOUT1(S, A)          if (DEBUG_HW) RtPrintf(S "\n", A)
#define HW_DEBUGOUT2(S, A, B)       if (DEBUG_HW) RtPrintf(S "\n", A, B)

#define EM_MAX_SCATTER		40
#define EM_VFTA_SIZE		128
#define EM_TSO_SIZE		(65535 + sizeof(struct ether_vlan_header))
#define EM_TSO_SEG_SIZE		4096	/* Max dma segment size */
#define EM_MSIX_MASK		0x01F00000 /* For 82574 use */
#define EM_MSIX_LINK		0x01000000 /* For 82574 use */
#define ETH_ZLEN		60
#define ETH_ADDR_LEN		6
#define CSUM_OFFLOAD		7	/* Offload bits in mbuf flag */

/*
* 82574 has a nonstandard address for EIAC
* and since its only used in MSIX, and in
* the em driver only 82574 uses MSIX we can
* solve it just using this define.
*/
#define EM_EIAC 0x000DC
/*
* 82574 only reports 3 MSI-X vectors by default;
* defines assisting with making it report 5 are
* located here.
*/
#define EM_NVM_PCIE_CTRL	0x1B
#define EM_NVM_MSIX_N_MASK	(0x7 << EM_NVM_MSIX_N_SHIFT)
#define EM_NVM_MSIX_N_SHIFT	7


#define PCI_ANY_ID                      (~0U)

typedef struct _em_vendor_info_t {
	unsigned int vendor_id;
	unsigned int device_id;
	unsigned int subvendor_id;
	unsigned int subdevice_id;
	unsigned int index;
} em_vendor_info_t;

struct em_int_delay_info {
	struct adapter *adapter;	/* Back-pointer to the adapter struct */
	int offset;			/* Register offset to read/write */
	int value;			/* Current value in usecs */
};

/* Our adapter structure */
struct adapter {
	if_t 		ifp;
	struct e1000_hw	hw;

	/* FreeBSD operating-system-specific structures. */
	struct e1000_osdep osdep;

	//struct device	*dev;  RTX_PORT
	
	void	*dev;
	struct cdev	*led_dev;

	struct resource *memory;
	struct resource *flash;
	struct resource *msix_mem;

	struct resource	*res;
	void		*tag;
	u32		linkvec;
	u32		ivars;

	//struct ifmedia	media;
	//struct callout	timer;
	int		msix;
	int		if_flags;
	int		max_frame_size;
	int		min_frame_size;
	//struct mtx	core_mtx;
	int		em_insert_vlan_header;
	u32		ims;
	bool		in_detach;

	/* Task for FAST handling */
	//struct task     link_task;
	//struct task     que_task;
	//struct taskqueue *tq;           /* private task queue */

	//eventhandler_tag vlan_attach;
	//eventhandler_tag vlan_detach;

	u16	num_vlans;
	u8	num_queues;

	/*
	* Transmit rings:
	*      Allocated at run time, an array of rings.
	*/
	struct tx_ring  *tx_rings;
	int             num_tx_desc;
	u32		txd_cmd;

	/*
	* Receive rings:
	*      Allocated at run time, an array of rings.
	*/
	struct rx_ring  *rx_rings;
	int             num_rx_desc;
	u32             rx_process_limit;
	u32		rx_mbuf_sz;

	/* Management and WOL features */
	u32		wol;
	bool		has_manage;
	bool		has_amt;

	/* Multicast array memory */
	u8		*mta;

	/*
	** Shadow VFTA table, this is needed because
	** the real vlan filter table gets cleared during
	** a soft reset and the driver needs to be able
	** to repopulate it.
	*/
	u32		shadow_vfta[EM_VFTA_SIZE];

	/* Info about the interface */
	u16		link_active;
	u16		fc;
	u16		link_speed;
	u16		link_duplex;
	u32		smartspeed;

	struct em_int_delay_info tx_int_delay;
	struct em_int_delay_info tx_abs_int_delay;
	struct em_int_delay_info rx_int_delay;
	struct em_int_delay_info rx_abs_int_delay;
	struct em_int_delay_info tx_itr;

	/* Misc stats maintained by the driver */
	unsigned long	dropped_pkts;
	unsigned long	link_irq;
	unsigned long	mbuf_defrag_failed;
	unsigned long	no_tx_dma_setup;
	unsigned long	no_tx_map_avail;
	unsigned long	rx_overruns;
	unsigned long	watchdog_events;

	struct e1000_hw_stats stats;
};


int em_attach(DRVDEV *pCardInfo);

void em_stop(DRVDEV *pCardInfo);

#define device_printf_2(dev, _Data_) if (pGlobals->displayErrorMessages){ \
								RtPrintf("%s ",pGlobals->msgPrefix); \
								RtPrintf _Data_ ; } \
								else (void)0




