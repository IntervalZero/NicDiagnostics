/******************************************************************************

  Copyright (c) 2001-2015, Intel Corporation 
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
/*$FreeBSD$*/


#ifndef _FREEBSD_OS_H_
#define _FREEBSD_OS_H_

#ifndef UNDER_RTSS  // RTX_PORT

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/mbuf.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/clock.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#endif


#include "RtPCH.h"

#include <windows.h>
#include <rtapi.h>      // RTX64 APIs that can be used in real-time or Windows applications.



#ifdef UNDER_RTSS
#include <rtssapi.h>    // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS


/******************************* PCI Definitions ******************************/
/*
* PCIM_xxx: mask to locate subfield in register
* PCIR_xxx: config register offset
* PCIC_xxx: device class
* PCIS_xxx: device subclass
* PCIP_xxx: device programming interface
* PCIV_xxx: PCI vendor ID (only required to fixup ancient devices)
* PCID_xxx: device ID
*/
#define PCIR_DEVVENDOR          0x00
#define PCIR_VENDOR             0x00
#define PCIR_DEVICE             0x02
#define PCIR_COMMAND            0x04
#define PCIM_CMD_PORTEN         0x0001
#define PCIM_CMD_MEMEN          0x0002
#define PCIM_CMD_BUSMASTEREN    0x0004
#define PCIM_CMD_MWRICEN        0x0010
#define PCIM_CMD_PERRESPEN      0x0040
#define PCIM_CMD_SERRESPEN      0x0100
#define PCIR_STATUS             0x06
#define PCIR_REVID              0x08
#define PCIR_PROGIF             0x09
#define PCIR_SUBCLASS           0x0a
#define PCIR_CLASS              0x0b
#define PCIR_CACHELNSZ          0x0c
#define PCIR_LATTIMER           0x0d
#define PCIR_HEADERTYPE         0x0e
#define PCIM_MFDEV              0x80
#define PCIR_BIST               0x0f
#define PCIR_CAP_PTR            0x34

/* config registers for header type 0 devices */
#define PCIR_MAPS       0x10
#define PCIR_SUBVEND_0  0x2c
#define PCIR_SUBDEV_0   0x2e

///****************************** PCI-X definitions *****************************/
//#define PCIXR_COMMAND   0x96
//#define PCIXR_DEVADDR   0x98
//#define PCIXM_DEVADDR_FNUM      0x0003  /* Function Number */
//#define PCIXM_DEVADDR_DNUM      0x00F8  /* Device Number */
//#define PCIXM_DEVADDR_BNUM      0xFF00  /* Bus Number */
//#define PCIXR_STATUS    0x9A
//#define PCIXM_STATUS_64BIT      0x0001  /* Active 64bit connection to device. */
//#define PCIXM_STATUS_133CAP     0x0002  /* Device is 133MHz capable */
//#define PCIXM_STATUS_SCDISC     0x0004  /* Split Completion Discarded */
//#define PCIXM_STATUS_UNEXPSC    0x0008  /* Unexpected Split Completion */
//#define PCIXM_STATUS_CMPLEXDEV  0x0010  /* Device Complexity (set == bridge) */
//#define PCIXM_STATUS_MAXMRDBC   0x0060  /* Maximum Burst Read Count */
//#define PCIXM_STATUS_MAXSPLITS  0x0380  /* Maximum Split Transactions */
//#define PCIXM_STATUS_MAXCRDS    0x1C00  /* Maximum Cumulative Read Size */
//#define PCIXM_STATUS_RCVDSCEM   0x2000  /* Received a Split Comp w/Error msg */


#define PCIY_EXPRESS	0x10

#define	EIO		5		/* Input/output error */
#define ENXIO            6 
#define	ENOMEM		12		/* Cannot allocate memory */

#if(UNDER_RTSS) //// RTX_PORT
#define ASSERT(x) if(!(x)) RtPrintf("RTX64 RtPCH NIC driver Panic: x")
#define EWARN(H, W, S)  RtPrintf(W)


__inline void DELAY(int microseconds)
{
	static LARGE_INTEGER freq = { 0, 0 };
	LARGE_INTEGER start;
	LARGE_INTEGER delay;
	LARGE_INTEGER now;
	volatile int cycles = 0;

	if (freq.QuadPart == 0)
	{
		QueryPerformanceFrequency(&freq);
	}

	delay.QuadPart = microseconds * freq.QuadPart / 1000000;

	QueryPerformanceCounter(&start);
	do
	{
		QueryPerformanceCounter(&now);
		// do busy wait here
		++cycles;
	} while (now.QuadPart - start.QuadPart < delay.QuadPart);
}
#endif

#define usec_delay(x) DELAY(x)
#define usec_delay_irq(x) usec_delay(x)
#define msec_delay(x) DELAY(1000*(x))
#define msec_delay_irq(x) DELAY(1000*(x))

/* Enable/disable debugging statements in shared code */
#define DBG		0

#define DEBUGOUT(...) \
    do { if (DBG) RtPrintf(__VA_ARGS__); } while (0)
#define DEBUGOUT1(...)			DEBUGOUT(__VA_ARGS__)
#define DEBUGOUT2(...)			DEBUGOUT(__VA_ARGS__)
#define DEBUGOUT3(...)			DEBUGOUT(__VA_ARGS__)
#define DEBUGOUT7(...)			DEBUGOUT(__VA_ARGS__)
#define DEBUGFUNC(F)			DEBUGOUT(F "\n")

#define STATIC			static
#define FALSE			0
#define TRUE			1
#define CMD_MEM_WRT_INVALIDATE	0x0010  /* BIT_4 */
#define PCI_COMMAND_REGISTER	PCIR_COMMAND

#ifndef UNDER_RTSS  // RTX_PORT
/* Mutex used in the shared code */
#define E1000_MUTEX                     struct mtx
#define E1000_MUTEX_INIT(mutex)         mtx_init((mutex), #mutex, \
                                            MTX_NETWORK_LOCK, \
					    MTX_DEF | MTX_DUPOK)
#define E1000_MUTEX_DESTROY(mutex)      mtx_destroy(mutex)
#define E1000_MUTEX_LOCK(mutex)         mtx_lock(mutex)
#define E1000_MUTEX_TRYLOCK(mutex)      mtx_trylock(mutex)
#define E1000_MUTEX_UNLOCK(mutex)       mtx_unlock(mutex)
#endif  // UNDER_RTSS  // RTX_PORT


void emInitializeCriticalLock(HFREECRITICALLOCK *pLock, char * szIndentifier);
void emDeleteCriticalLock(HFREECRITICALLOCK *pLock);
void emEnterCriticalLock(HFREECRITICALLOCK *pLock);
void emLeaveCriticalLock(HFREECRITICALLOCK *pLock);

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define __AT__ __FILE__ ":" TOSTRING(__LINE__)


#define E1000_MUTEX                     HFREECRITICALLOCK
#define E1000_MUTEX_INIT(mutex)         emInitializeCriticalLock(mutex, __AT__)
#define E1000_MUTEX_DESTROY(mutex)      emDeleteCriticalLock(mutex)
#define E1000_MUTEX_LOCK(mutex)         emEnterCriticalLock(mutex)
// unused #define E1000_MUTEX_TRYLOCK(mutex)      emEnterCriticalLock(mutex)
#define E1000_MUTEX_UNLOCK(mutex)       emLeaveCriticalLock(mutex)

typedef unsigned __int64 uint64_t;
typedef unsigned __int64 u_int16_t;
typedef unsigned __int32  uint32_t;
typedef unsigned __int16  uint16_t;
typedef unsigned __int8  uint8_t;
typedef __int64 int64_t;
typedef __int32 int32_t;
typedef __int16 int16_t;
typedef __int8 int8_t;


typedef uint64_t	u64;
typedef uint32_t	u32;
typedef uint16_t	u16;
typedef uint8_t		u8;
typedef int64_t		s64;
typedef int32_t		s32;
typedef int16_t		s16;
typedef int8_t		s8;

#define __le16		u16
#define __le32		u32
#define __le64		u64

typedef void * caddr_t;

typedef  BOOL bool;
#define  false 0
#define  true  1

#include "_bus.h"

#define __FreeBSD_version 10300000

#if __FreeBSD_version < 800000
#if defined(__i386__) || defined(__amd64__)
#define mb()	__asm volatile("mfence" ::: "memory")
#define wmb()	__asm volatile("sfence" ::: "memory")
#define rmb()	__asm volatile("lfence" ::: "memory")
#else
#define mb()
#define rmb()
#define wmb()
#endif
#endif /*__FreeBSD_version < 800000 */

//#if defined(__i386__) || defined(__amd64__)
//static __inline
//void prefetch(void *x)
//{
//	__asm volatile("prefetcht0 %0" :: "m" (*(unsigned long *)x));
//}
//#else
//#define prefetch(x)
//#endif



struct e1000_osdep
{
	bus_space_tag_t    mem_bus_space_tag;
	bus_space_handle_t mem_bus_space_handle;
	bus_space_tag_t    io_bus_space_tag;
	bus_space_handle_t io_bus_space_handle;
	bus_space_tag_t    flash_bus_space_tag;
	bus_space_handle_t flash_bus_space_handle;
	//struct device     *dev;
	void     *dev;
};




#define READ8(reg)  (*((unsigned __int8  *)(reg)))
#define READ16(reg) (*((unsigned __int16 *)(reg)))
#define READ32(reg) (*((unsigned __int32 *)(reg)))

#define WRITE8(reg, val)    (*((unsigned __int8  *)(reg)) =   ((unsigned __int8 )(val)) )
#define WRITE16(reg, val)   (*((unsigned __int16 *)(reg)) =   ((unsigned __int16)(val)) )
#define WRITE32(reg, val)   (*((unsigned __int32 *)(reg)) =   ((unsigned __int32)(val)) )


//#define READ8(reg)  RtPrintf("READ8(%p)\n",reg)
//#define READ16(reg) RtPrintf("READ16(%p)\n",reg)
//#define READ32(reg) RtPrintf("READ32(%p)\n",reg)
//
//#define WRITE8(reg, val)   RtPrintf("Write8(%p, val=%x)\n",reg, val)
//#define WRITE16(reg, val)   RtPrintf("Write16(%p, val=%x)\n",reg, val)
//#define WRITE32(reg, val)   RtPrintf("Write32(%p, val=%x)\n",reg, val)



#define bus_space_read_1(mem_bus_space_tag,	 mem_bus_space_handle,	reg)   READ8(mem_bus_space_handle + reg) 
#define bus_space_read_2(mem_bus_space_tag,	 mem_bus_space_handle,	reg)   READ16(mem_bus_space_handle + reg)  
#define bus_space_read_4(mem_bus_space_tag,	 mem_bus_space_handle,	reg)   READ32(mem_bus_space_handle + reg) 


#define bus_space_write_1(mem_bus_space_tag, mem_bus_space_handle, offset, value)  WRITE8(mem_bus_space_handle + offset, value)
#define bus_space_write_2(mem_bus_space_tag, mem_bus_space_handle, offset, value)  WRITE16(mem_bus_space_handle + offset, value)
#define bus_space_write_4(mem_bus_space_tag, mem_bus_space_handle, offset, value)  WRITE32(mem_bus_space_handle + offset, value)


#define E1000_REGISTER(hw, reg) (((hw)->mac.type >= e1000_82543) \
    ? reg : e1000_translate_register_82542(reg))

#define E1000_WRITE_FLUSH(a) E1000_READ_REG(a, E1000_STATUS)

/* Read from an absolute offset in the adapter's memory space */
#define E1000_READ_OFFSET(hw, offset) \
    bus_space_read_4(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
    ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, offset)

/* Write to an absolute offset in the adapter's memory space */
#define E1000_WRITE_OFFSET(hw, offset, value) \
    bus_space_write_4(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
    ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, offset, value)

/* Register READ/WRITE macros */

#define E1000_READ_REG(hw, reg) \
    bus_space_read_4(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg))

#define E1000_WRITE_REG(hw, reg, value) \
    bus_space_write_4(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg), value)

#define E1000_READ_REG_ARRAY(hw, reg, index) \
    bus_space_read_4(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg) + ((index)<< 2))

#define E1000_WRITE_REG_ARRAY(hw, reg, index, value) \
    bus_space_write_4(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg) + ((index)<< 2), value)

#define E1000_READ_REG_ARRAY_DWORD E1000_READ_REG_ARRAY
#define E1000_WRITE_REG_ARRAY_DWORD E1000_WRITE_REG_ARRAY

#define E1000_READ_REG_ARRAY_BYTE(hw, reg, index) \
    bus_space_read_1(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg) + index)

#define E1000_WRITE_REG_ARRAY_BYTE(hw, reg, index, value) \
    bus_space_write_1(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg) + index, value)

#define E1000_WRITE_REG_ARRAY_WORD(hw, reg, index, value) \
    bus_space_write_2(((struct e1000_osdep *)(hw)->back)->mem_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->mem_bus_space_handle, \
        E1000_REGISTER(hw, reg) + (index << 1), value)


#define E1000_WRITE_REG_IO(hw, reg, value) do {\
    bus_space_write_4(((struct e1000_osdep *)(hw)->back)->io_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->io_bus_space_handle, \
        (hw)->io_base, reg); \
    bus_space_write_4(((struct e1000_osdep *)(hw)->back)->io_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->io_bus_space_handle, \
        (hw)->io_base + 4, value); } while (0)

#define E1000_READ_FLASH_REG(hw, reg) \
    bus_space_read_4(((struct e1000_osdep *)(hw)->back)->flash_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->flash_bus_space_handle, reg)

#define E1000_READ_FLASH_REG16(hw, reg) \
    bus_space_read_2(((struct e1000_osdep *)(hw)->back)->flash_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->flash_bus_space_handle, reg)

#define E1000_WRITE_FLASH_REG(hw, reg, value) \
    bus_space_write_4(((struct e1000_osdep *)(hw)->back)->flash_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->flash_bus_space_handle, reg, value)

#define E1000_WRITE_FLASH_REG16(hw, reg, value) \
    bus_space_write_2(((struct e1000_osdep *)(hw)->back)->flash_bus_space_tag, \
        ((struct e1000_osdep *)(hw)->back)->flash_bus_space_handle, reg, value)


#define TUNABLE_INT(path, var)           

#define boolean_t BOOL


// from  "if_var.h"
typedef enum {
	IFCOUNTER_IPACKETS = 0,
	IFCOUNTER_IERRORS,
	IFCOUNTER_OPACKETS,
	IFCOUNTER_OERRORS,
	IFCOUNTER_COLLISIONS,
	IFCOUNTER_IBYTES,
	IFCOUNTER_OBYTES,
	IFCOUNTER_IMCASTS,
	IFCOUNTER_OMCASTS,
	IFCOUNTER_IQDROPS,
	IFCOUNTER_OQDROPS,
	IFCOUNTER_NOPROTO,
	IFCOUNTERS /* Array size. */
} ift_counter;


typedef struct ifnet * if_t;

typedef	void(*if_start_fn_t)(if_t);
typedef	int(*if_ioctl_fn_t)(if_t, u_long, caddr_t);
typedef	void(*if_init_fn_t)(void *);
typedef void(*if_qflush_fn_t)(if_t);
typedef int(*if_transmit_fn_t)(if_t, struct mbuf *);
typedef	uint64_t(*if_get_counter_t)(if_t, ift_counter);

struct ifnet_hw_tsomax {
	u_int	tsomaxbytes;	/* TSO total burst length limit in bytes */
	u_int	tsomaxsegcount;	/* TSO maximum segment count */
	u_int	tsomaxsegsize;	/* TSO maximum segment size in bytes */
};

/* Interface encap request types */
typedef enum {
	IFENCAP_LL = 1			/* pre-calculate link-layer header */
} ife_type;

/*
* The structure below allows to request various pre-calculated L2/L3 headers
* for different media. Requests varies by type (rtype field).
*
* IFENCAP_LL type: pre-calculates link header based on address family
*   and destination lladdr.
*
*   Input data fields:
*     buf: pointer to destination buffer
*     bufsize: buffer size
*     flags: IFENCAP_FLAG_BROADCAST if destination is broadcast
*     family: address family defined by AF_ constant.
*     lladdr: pointer to link-layer address
*     lladdr_len: length of link-layer address
*     hdata: pointer to L3 header (optional, used for ARP requests).
*   Output data fields:
*     buf: encap data is stored here
*     bufsize: resulting encap length is stored here
*     lladdr_off: offset of link-layer address from encap hdr start
*     hdata: L3 header may be altered if necessary
*/

struct if_encap_req {
	u_char		*buf;		/* Destination buffer (w) */
	size_t		bufsize;	/* size of provided buffer (r) */
	ife_type	rtype;		/* request type (r) */
	uint32_t	flags;		/* Request flags (r) */
	int		family;		/* Address family AF_* (r) */
	int		lladdr_off;	/* offset from header start (w) */
	int		lladdr_len;	/* lladdr length (r) */
	char		*lladdr;	/* link-level address pointer (r) */
	char		*hdata;		/* Upper layer header data (rw) */
};

#define	IFENCAP_FLAG_BROADCAST	0x02	/* Destination is broadcast */


/*
* Structure defining a network interface.
*
* Size ILP32:  592 (approx)
*	 LP64: 1048 (approx)
*/
struct ifnet {
	/* General book keeping of interface lists. */
	//TAILQ_ENTRY(ifnet) if_link; 	/* all struct ifnets are chained */
	//LIST_ENTRY(ifnet) if_clones;	/* interfaces of a cloner */
	//TAILQ_HEAD(, ifg_list) if_groups; /* linked list of groups per if */
	///* protected by if_addr_lock */
	//u_char	if_alloctype;		/* if_type at time of allocation */

	/* Driver and protocol specific information that remains stable. */
	void	*if_softc;		/* pointer to driver state */
	void	*if_llsoftc;		/* link layer softc */
	void	*if_l2com;		/* pointer to protocol bits */
	const char *if_dname;		/* driver name */
	int	if_dunit;		/* unit or IF_DUNIT_NONE */
	u_short	if_index;		/* numeric abbreviation for this if  */
	short	if_index_reserved;	/* spare space to grow if_index */
	///char	if_xname[IFNAMSIZ];	/* external name (name + unit) */
	char	*if_description;	/* interface description */

	/* Variable fields that are touched by the stack and drivers. */
	int	if_flags;		/* up/down, broadcast, etc. */
	int	if_drv_flags;		/* driver-managed status flags */
	int	if_capabilities;	/* interface features & capabilities */
	int	if_capenable;		/* enabled features & capabilities */
	void	*if_linkmib;		/* link-type-specific MIB data */
	size_t	if_linkmiblen;		/* length of above data */
	u_int	if_refcount;		/* reference count */

	/* These fields are shared with struct if_data. */
	uint8_t		if_type;	/* ethernet, tokenring, etc */
	uint8_t		if_addrlen;	/* media address length */
	uint8_t		if_hdrlen;	/* media header length */
	uint8_t		if_link_state;	/* current link state */
	uint32_t	if_mtu;		/* maximum transmission unit */
	uint32_t	if_metric;	/* routing metric (external only) */
	uint64_t	if_baudrate;	/* linespeed */
	uint64_t	if_hwassist;	/* HW offload capabilities, see IFCAP */
	time_t		if_epoch;	/* uptime at attach or stat reset */
	struct timeval	if_lastchange;	/* time of last administrative change */

	//struct  ifaltq if_snd;		/* output queue (includes altq) */
	//struct	task if_linktask;	/* task for link change events */

	/* Addresses of different protocol families assigned to this if. */
	//struct	rwlock if_addr_lock;	/* lock to protect address lists */
	/*
	* if_addrhead is the list of all addresses associated to
	* an interface.
	* Some code in the kernel assumes that first element
	* of the list has type AF_LINK, and contains sockaddr_dl
	* addresses which store the link-level address and the name
	* of the interface.
	* However, access to the AF_LINK address through this
	* field is deprecated. Use if_addr or ifaddr_byindex() instead.
	*/
	//  struct	ifaddrhead if_addrhead;	/* linked list of addresses per if */
	//struct	ifmultihead if_multiaddrs; /* multicast addresses configured */
	//int	if_amcount;		/* number of all-multicast requests */
	//struct	ifaddr	*if_addr;	/* pointer to link-level address */
	//const u_int8_t *if_broadcastaddr; /* linklevel broadcast bytestring */
	//struct	rwlock if_afdata_lock;
	//void	*if_afdata[AF_MAX];
	//int	if_afdata_initialized;

	/* Additional features hung off the interface. */
	u_int	if_fib;			/* interface FIB */
	struct	vnet *if_vnet;		/* pointer to network stack instance */
	struct	vnet *if_home_vnet;	/* where this ifnet originates from */
	struct  ifvlantrunk *if_vlantrunk; /* pointer to 802.1q data */
	struct	bpf_if *if_bpf;		/* packet filter structure */
	int	if_pcount;		/* number of promiscuous listeners */
	void	*if_bridge;		/* bridge glue */
	void	*if_lagg;		/* lagg glue */
	void	*if_pf_kif;		/* pf glue */
	struct	carp_if *if_carp;	/* carp interface structure */
	struct	label *if_label;	/* interface MAC label */
	struct	netmap_adapter *if_netmap; /* netmap(4) softc */

	/* Various procedures of the layer2 encapsulation and drivers. */
	int(*if_output)		/* output routine (enqueue) */
		(struct ifnet *, struct mbuf *, const struct sockaddr *,
	struct route *);
	void(*if_input)		/* input routine (from h/w driver) */
		(struct ifnet *, struct mbuf *);
	if_start_fn_t	if_start;	/* initiate output routine */
	if_ioctl_fn_t	if_ioctl;	/* ioctl routine */
	if_init_fn_t	if_init;	/* Init routine */
	int(*if_resolvemulti)	/* validate/resolve multicast */
		(struct ifnet *, struct sockaddr **, struct sockaddr *);
	if_qflush_fn_t	if_qflush;	/* flush any queue */
	if_transmit_fn_t if_transmit;   /* initiate output routine */

	void(*if_reassign)		/* reassign to vnet routine */
		(struct ifnet *, struct vnet *, char *);
	if_get_counter_t if_get_counter; /* get counter values */
	int(*if_requestencap)	/* make link header from request */
		(struct ifnet *, struct if_encap_req *);

	/* Statistics. */
	//counter_u64_t	if_counters[IFCOUNTERS];

	/* Stuff that's only temporary and doesn't belong here. */

	/*
	* Network adapter TSO limits:
	* ===========================
	*
	* If the "if_hw_tsomax" field is zero the maximum segment
	* length limit does not apply. If the "if_hw_tsomaxsegcount"
	* or the "if_hw_tsomaxsegsize" field is zero the TSO segment
	* count limit does not apply. If all three fields are zero,
	* there is no TSO limit.
	*
	* NOTE: The TSO limits should reflect the values used in the
	* BUSDMA tag a network adapter is using to load a mbuf chain
	* for transmission. The TCP/IP network stack will subtract
	* space for all linklevel and protocol level headers and
	* ensure that the full mbuf chain passed to the network
	* adapter fits within the given limits.
	*/
	u_int	if_hw_tsomax;		/* TSO maximum size in bytes */
	u_int	if_hw_tsomaxsegcount;	/* TSO maximum segment count */
	u_int	if_hw_tsomaxsegsize;	/* TSO maximum segment size in bytes */

	/*
	* Spare fields to be added before branching a stable branch, so
	* that structure can be enhanced without changing the kernel
	* binary interface.
	*/
	void	*if_pspare[4];		/* packet pacing / general use */
	int	if_ispare[4];		/* packet pacing / general use */
};



u32 pci_read_config(DRVDEV *pCardInfo, u32 offset, u32 len);
void pci_write_config(DRVDEV *pCardInfo, u32 offset, u32 data, u32 len);
int pci_find_cap(DRVDEV *pCardInfo, int capability, int *capreg);

#endif  /* _FREEBSD_OS_H_ */

