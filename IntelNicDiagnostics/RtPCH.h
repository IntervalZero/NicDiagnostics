/**--------------------------------------------------------------------------------------------------
//
// Copyright(c) 2016  IntervalZero, Inc.  All rights reserved.
//
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//-------------------------------------------------------------------------------------------------
// @file	RtPCH.h
//
// Environments:
// 	         Real-time 64-bit Subsystem (supported configurations: RTSSDebug and RTSSRelease)
//
//  @brief This solutions shows the framework required for and RTX64 Network Interface Adapter(NIC)
//         driver.
//
//
*-----------------------------------------------------------------------------------------------**/


#pragma once
//This define will deprecate all unsupported Microsoft C-runtime functions when compiled under RTSS.
//If using this define, #include <rtapi.h> should remain below all windows headers
//#define UNDER_RTSS_UNSUPPORTED_CRT_APIS

#include <SDKDDKVer.h>

// for RTTCPIP Support
//#include <winsock2.h>
//#include <ws2tcpip.h>

//#include <stdio.h>
//#include <string.h>
//#include <ctype.h>
//#include <conio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <errno.h>
#include <windows.h>
#include <rtapi.h>      // RTX64 APIs that can be used in real-time or Windows applications.
#include "rtnapi.h" 

#ifdef UNDER_RTSS
#include <rtssapi.h>    // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS



//*****************************************************************************
// COMMON PCI/PCIe DEFINES
//
#define PCI_COMMON_HDR_LENGTH (FIELD_OFFSET (PCI_COMMON_CONFIG, DeviceSpecific))
#define PCI_MAX_BUS                         255
#define PCI_MAX_DEVICES                     32
#define PCI_MAX_FUNCTION                    8
#define PCI_INVALID_VENDORID                0xFFFF
#define PCI_BAR0 0
#define PCI_BAR1 1
#define PCI_BAR5 5
#define SUPPORTED_BASECLASS		0x02
#define SUPPORTED_SUBCLASS		0x00
#define SUPPORTED_PROG_INF_REG	0x00
#define SZBUFFER_SIZE 100
#define LOCS 24
#define BUS_LOC 0
#define DEV_LOC 1
#define FUNC_LOC 2

#define SMALL_PAGE_SIZE 4096


#define NUMBER_OF_x540_QUEUES  1   // the x540 has 128 queues but since we are not using them we can set it to 1
#define INTEL_x540_REGISTERMAP_SIZE 0x20000  // 


#define MAX_NUMBER_OF_QUEUES 16


// Ethernet values
#define ETHERNET_ADDRESS_LEN   6
#define ETHERNET_ETYPE_LEN     2
#define ETHERNET_HEADER_LEN    14
#define ETHERNET_FCS_LEN       4     // Checksum length
#define ETHERNET_DATA_LEN      1500  // Maximum (standard) data (w/o HDR/FCS)
#define ETHERNET_MIN_LEN       64    // Minumum frame
#define ETHERNET_STD_LEN       1518  // Maximum (standard) frame 
// Jumbo ethernet values
#define ETHERNET_MAX_STD_JUMBO_LEN 9234  // Maximum standard jumbo frame
#define ETHERNET_MAX_JUMBO_LEN     16128 // Maximum jumbo frame (max possible receive buffer size)
#define ETHERNET_ETYPE_OFFSET		12
#define ETHERNET_VLANTAG_OFFSET		12


// Supported RX buffer Sizes
#define DRV_RXBUFFER_128   128
#define DRV_RXBUFFER_256   256
#define DRV_RXBUFFER_512   512
#define DRV_RXBUFFER_1024  1024
#define DRV_RXBUFFER_2048  2048
#define DRV_RXBUFFER_4096  4096
#define DRV_RXBUFFER_8192  8192
#define DRV_RXBUFFER_9216  9216
#define DRV_RXBUFFER_16384 16384

// Number of Transmit Descriptors
#define DEFAULT_TXD      256 
#define MAX_TXD         16384
#define MIN_TXD          80 

// Number of Receive Descriptors
#define DEFAULT_RXD      256 
#define MAX_RXD         16384 
#define MIN_RXD          80 

#define REQ_TX_DESCRIPTOR_MULTIPLE 8
#define REQ_RX_DESCRIPTOR_MULTIPLE 8



typedef LARGE_INTEGER PHYSICAL_ADDRESS, *PPHYSICAL_ADDRESS;


#define ROUNDUP(size, unit) (unit * ((size + unit - 1) / unit))

typedef struct _COMMON_DEV_STATS
{
	unsigned long Size;     // Size of structure
	unsigned long Sbrcnt;  // Receive packet count
	unsigned long Sbxcnt;  // Transmit packet count
	unsigned long Sbrecnt; // Receive error count
	unsigned long Sbxecnt; // Transmit error count
	unsigned long Sbjam;   // Jam retry count 
	unsigned long SbXnbc;	// Transmit no buffer count
} COMMON_DEV_STATS, *PCOMMON_DEV_STATS;


//-----------------------------------------------------------------------------
// Descriptor (transmit and receive) structures
//
typedef struct _IGB_64_BIT_PHYS_ADDRESS
{
	DWORD Lo32;
	DWORD Hi32;
} IGB_64_BIT_PHYS_ADDRESS, *TIGB_64_BIT_PHYS_ADDRESS;



typedef struct _RECEIVE_ADDRESS_REGISTER_PAIR
{
	volatile unsigned int Low;
	volatile unsigned int High;
} RECEIVE_ADDRESS_REGISTER_PAIR, *PRECEIVE_ADDRESS_REGISTER_PAIR;



typedef int(__cdecl* RTNDL2TXCALLBACK)(void * pContext);

typedef struct _PTP_TIME
{
	volatile unsigned __int32 sec;
	volatile unsigned __int32 nsec;
} PTP_TIME;

typedef struct _PTP_HW_TSTAMPS
{
	unsigned int	timeStampCount;
	PTP_TIME		timeStamp;
} PTP_HW_TSTAMPS;

 
typedef struct _HFREECRITICALLOCK
{
	LONG volatile	lock;  // this variable must be first in this structure
	DWORD volatile	lockOwnerThreadID;  // acquired from GetCurrentThreadId(VOID);
	DWORD volatile	lockOwnerThreadPriority;
	DWORD volatile	lockRequestorThreadPriority;  // original priority before priority inversion 
	DWORD			maxNumberOfSpins;
	char			lockSymbolic[MAX_PATH];  // string describing what the lock is used for

} HFREECRITICALLOCK;

/**--------------------------------------------------------------------------------------------------
//   \struct _RTIGB_DESCRIPTOR_EXTENSION
//   \brief  _RTIGB_DESCRIPTOR_EXTENSION is a structure used to pass additional information about a
//   		 Ethernet frame.
//
*-----------------------------------------------------------------------------------------------**/
typedef struct _RTIGB_DESCRIPTOR_EXTENSION
{
	void				*frameBuffPhysicalAddr;	/**< The physical memory address for the Ethernet frame */
	void				*frameBuffKernelAddr;	/**< The virtual memory address for the Ethernet frame */
	ULONG				frameSize;				/**< The size of the Layer 2 Ethernet frame */
	int					QueNum;					/**< The queue that the Ethernet frame is to be transmitted on
												or the queue that the frame was received on */
	RTNDL2TXCALLBACK	fp_L2TxCallBack;		/**< A function pointer to call when transmission is complete */
	RTNDL2TXCALLBACK	fp_L2RxCallBack;		/**< A function pointer to call when a frame is received */

	BOOL				bTimeStampRequired;		/**< Indicates if a time stamp is required */
	BOOL				bEnableLaunchTime;		/**< Launch time is currently not implemented */
	DWORD				launchTime;				/**< Launch time is currently not implemented */

	PTP_TIME	packetTimeStamp;				/**< The time of transmit or receive.  Time is from the NIC's hardware timer
												When this variable returns the transmit DMA time stamp for the i210 NIC
												only the 10 least significant bits are reported for seconds, while the 32 bits
												for nanoseconds are complete */
	DWORD		transmitStatus;					/**< 0 = success */ //

	// some diagnostics 
	unsigned int		frameCounter;			/**< A time provided to the transmit call and returned in the transmit callback */ //
	LARGE_INTEGER		RtnTxCallTime;			/**< A time provided to the transmit call and returned in the transmit callback */ //
	LARGE_INTEGER		TxCallbackTime;			/**< A time captured just prior to the callback and returned in the transmit callback */ //

} RTIGB_DESCRIPTOR_EXTENSION;


typedef struct _DRV_QUE_DESCRIPTORS
{
	//  Allocated Memory blocks prior to alignment
	BYTE* pTxDescriptorBufferUnAligned;
	BYTE* pRxDescriptorBufferUnAligned;

	// Aligned memory blocks
	BYTE* pbyTxDescriptors;
	BYTE* pbyRxDescriptors;

	// Transmit descriptor variables
	struct			e1000_tx_desc	*TxDescriptors;
	RTIGB_DESCRIPTOR_EXTENSION	*pTxDescriptorExtension;
	BYTE**			txFrameBuffPhysicalAddr;  // points to an array of physical addresses
	BYTE**			txFrameBuffKernelAddr;  // points to an array of pointers
	BYTE*			TxDescDMA;
	DWORD			TxIntDelay;
	DWORD			TxAbsIntDelay;
	DWORD			NumTxDescriptors;
	volatile DWORD	NextAvailTxDescriptor;
	volatile DWORD	OldestUsedTxDescriptor;
	volatile DWORD	NumTxDescriptorsAvail;

	// Receive descriptor variables
	union e1000_rx_desc_extended *RxDescriptors;
	RTIGB_DESCRIPTOR_EXTENSION	*pRxDescriptorExtension;
	BYTE**			rxFrameBuffPhysicalAddr;  // points to an array of physical addresses
	BYTE**			rxFrameBuffKernelAddr;  // points to an array of pointers
	BYTE*			RxDescDMA;
	DWORD			RxIntDelay;
	DWORD	RxAbsIntDelay;
	volatile DWORD	NumRxDescriptors;
	volatile DWORD	NextRxDescriptorToCheck;
	volatile DWORD	NumRxDescriptorsEmpty;
	volatile DWORD	NextRxDescriptorToFill;

}  DRV_QUE_DESCRIPTORS, *PDRV_QUE_DESCRIPTORS;


typedef struct _DRVDEV
{
	//
	// These are required by the stack interface
	//
	void *ndp;
	char StackDeviceName[32];

	//
	// These are device-specific, but are very common and will probably be required
	//
	ULONG Irq; // IRQ number
	ULONG BusNumber;
	ULONG FuncNumber;
	PCI_SLOT_NUMBER SlotNumber;
	BOOL LineBasedOnly;
	BOOL UsingMSI;
	BOOL PreferMsi;
	ULONG IntIdealProc;
	ULONG IntMask;
	ULONG IntPriority;
	ULONG RecvPriority;

	HANDLE hInterrupt;
	HANDLE hRecvEvent;
	HANDLE hTxCompleteEvent;
	HANDLE hCompEvent;
	HANDLE hLineStatusEvent;
	HANDLE hLineStatusThread;
	BOOL  bLinkStatus;
	ULONG ulLinkStatusPriority;
	ULONG ulLinkStatusIdealProcessor;
	HANDLE hDrvTxCompleteThread;

	volatile int RecvW;
	volatile int TransW;
	volatile int InterW;

	HFREECRITICALLOCK csRxDMARing[MAX_NUMBER_OF_QUEUES];
	HFREECRITICALLOCK csTxDMARing[MAX_NUMBER_OF_QUEUES];
	HFREECRITICALLOCK csIoctl;

	ULONG ulLatencyRecvUpdated;

	ULONG IOBase;     // IO space base address
	UCHAR *CsrBase;   // CSR memory base address of registers on the card
	PHYSICAL_ADDRESS CsrPhysical;
	UCHAR *FlashBase; // FLASH device base address

	// Has the device been brought up?
	BOOL DeviceIsUp;

	// Common network device statistics
	COMMON_DEV_STATS ComStats;

	// Mac and Phy Settings
	BYTE  MediaType;
	DWORD PhyId;
	DWORD PhyAddress;
	BOOL  PhyResetDisable; // currently always FALSE
	DWORD	PhyTxDelay;
	DWORD	PhyRxDelay;

	USHORT	PhyType;				// ACB
	USHORT	PhyRevision;			// ACB

	// MAC Configuration 
	BOOL 	AutoNeg;				// ACB
	BOOL 	AutoNegAdv;				// ACB
	BOOL 	WaitAutoNegComplete;
	BOOL 	ForcedSpeedDuplex;		// ACB
	BOOL 	SpeedDowngraded;		// ACB
	USHORT  AutoNegAdvertised;
	BOOL    AutoNegFailed;

	DWORD  LEDCtlDefault;
	DWORD  LEDCtlMode1;
	DWORD  LEDCtlMode2;

	// Status Flags
	RTND_MEDIA_CONNECT_STATE		LinkIsActive;  
	USHORT		LineSpeed;
	USHORT		FullDuplex;
	BOOL		GetLinkStatus;
	BOOL		TbiCompatibilityEnable;
	BOOL		TbiCompatibilityOn;

	// PCI Device Info
	USHORT		VendorId;
	USHORT		DeviceId;
	BYTE		RevId;
	DWORD		adapterType;

	//  Queue info
	int			numberOfNicQueues;
	int			currentRxQueue;
	int			defaultQueue;

	DWORD		MulticastFilterType;
	BYTE		DmaFairness;
	DWORD		OriginalFlowControl;
	DWORD		FlowControl;
	BOOL		ReportTxEarly;
	DWORD		TxcwRegValue;

	DRV_QUE_DESCRIPTORS	adapterQ[MAX_NUMBER_OF_QUEUES];		// the i350 has the most queues

	//RXPOLLINGCONFIG rxPollingConfig[MAX_NUMBER_OF_QUEUES];

	// Frame/MTU values
	ULONG		Mtu;
	ULONG		MaxPacket;
	ULONG		RxBufferSize;

	// TO settings
	BYTE RxChecksum;

	// Our Ethernet address
	BYTE EnetAddr[6]; // Physical address: from TCP/IP or

	// Statistic Registers Dump
	__int64 Crcerrs;         // rx crc error count
	__int64 Algnerrc;        // alignment error count (LG)
	__int64 Symerrs;         // rx symbol error count
	__int64 Rxerrc;          // RX_ERROR count (LG) - PHY detected rx errors
	__int64 Mpc;             // Missed Packet Count - packets dropped due to full rx fifo
	__int64 Scc;             // single collision count
	__int64 Ecol;            // excessive collision count
	__int64 Mcc;             // multiple collision count
	__int64 Latecol;         // late collision count
	__int64 Colc;            // Total collision count
	__int64 Tuc;             // transmit underrun count (LG)
	__int64 Dc;              // tx defer count due
	__int64 Tncrs;           // transmit no CRS count (LG)
	__int64 Sec;             // rx sequence error count
	__int64 Cexterr;         // carrier ext error count (LG)
	__int64 Rlec;            // receive length error count
	__int64 Rutec;           // receive DMA to early count (LG)
	__int64 Xonrxc;          // XON received count
	__int64 Xontxc;          // XON transmitted count
	__int64 Xoffrxc;         // XOFF received count
	__int64 Xofftxc;         // XOFF transmitted count
	__int64 Fcruc;           // unknown Flow Control packet rcved count
	__int64 Prc64;           // packets received (64 Bytes)
	__int64 Prc127;          // packets received (65-127)
	__int64 Prc255;          // packets received (128-255)
	__int64 Prc511;          // packets received (256-511)
	__int64 Prc1023;         // packets received (512-1023)
	__int64 Prc1522;         // packets received (1024+)
	__int64 Gprc;            // Good packets received count
	__int64 Bprc;            // Broadcasts pkts recvd count
	__int64 Mprc;            // Multicast  pkts recvd count
	__int64 Gptc;            // Good packets xmited count
	__int64 Gor;             // Good octets recvd
	__int64 Got;             // Good octets xmitd
	__int64 Rnbc;            // Receive no buffers count
	__int64 Ruc;             // Receive undersize count
	__int64 Rfc;             // Receive fragment count
	__int64 Roc;             // Receive oversize count
	__int64 Rjc;             // Receive jabber count
	__int64 Tor;             // Total octets recvd
	__int64 Tot;             // Total octets xmitd
	__int64 Tpr;             // Total packets received
	__int64 Tpt;             // Total packets xmitted
	__int64 Ptc64;           // packets xmitted (64B)
	__int64 Ptc127;          // packets xmitted (64-127)
	__int64 Ptc255;          // packets xmitted (128-255)
	__int64 Ptc511;          // packets xmitted (256-511)
	__int64 Ptc1023;         // packets xmitted (512-1023)
	__int64 Ptc1522;         // packets xmitted (1024-1522)
	__int64 Mptc;            // Multicast  pkts xmit count
	__int64 Bptc;            // Broadcasts pkts xmit count

	// EEPROM helper fields
	WORD	eeType;
	WORD	eeWordSize;
	WORD	eePageSize;
	WORD	eeOpcodeBits;
	WORD	eeAddressBits;
	WORD	eeDelay;
	BOOL	UseEerd;

	// Type of onboard NVM
	//NVM_TYPE NvmType;

	BOOL ZeroCopy;

	// Flash EEPROM/NVM
	UINT FlashBankSize;
	UINT FlashBaseOffset;
	//ICH8_SHADOW_RAM *EepromShadowRAM;

	BOOL			AvbEnabled;

	BOOL			appendTimeStampToPtpV2L2RxPacket;


	struct adapter *pBsdStyleAdapter;

	RtLIST_ENTRY(_DRVDEV) dlist;

} DRVDEV, *PDRVDEV;




#define DRV_MAX_PORTS 64

typedef struct _DRV_GLOBALS
{
	ULONG		nCards;
	DRVDEV		*drvCards[DRV_MAX_PORTS];
	char		* msgPrefix;
	BOOL		displayErrorMessages;
	PTP_HW_TSTAMPS txTimeStamp, rxTimeStamp;
	DWORD		reserved[256];
} DRV_GLOBALS;

// This structure is used to separate the initialization (input) parameters from
// the actual card values.
typedef struct _DRVDEV_INIT_PARAMS
{
	BYTE MacAddress[6];
	ULONG BusNumber;
	ULONG DeviceNumber;
	ULONG FuncNumber;
	DRVDEV *pCardInfo;
} DRVDEV_INIT_PARAMS, *PDRVDEV_INIT_PARAMS;


/**--------------------------------------------------------------------------------------------------
\fn	RTNDPROC int RtndInitialize(int fDisplayErrorMessages, char *DeviceName, PRTNINTERFACE pNetInterface);

\brief	RtndInitialize is called by the stack when an instance of a driver is being first
brought online.  This is the first routine in a driver that is called by the stack.

RtndInitialize will be called once and only once per instance of the driver, but
maybe called multiple times if more than one instance of the driver is needed for
more than one NIC. As such, all drivers should keep a local configuration structure
array to hold configuration information read in during the RtndInitialize function.
The driver then simply increments a local instance count variable to index the array.
RtndInitialize should store a copy of the device name and a pointer to a network
device data structure in the local configuration structure array slot along with
other configuration information, so that RtndConfigure can find the proper slot in
the local configuration array.

\param	fDisplayErrorMessages	Verbose flag. If non-zero then driver has the option to be
verbose about errors or warnings. If 0, driver should be
silent and simply return the -1 error code on catastrophic
errors.
\param [in,out]	DeviceName   	ASCII name for this device (normally rtnd0 through rtnd3).
Can be stored in a local configuration structure to help de-
reference network device pointer in RtndConfigure.
\param	pNetInterface		 	Pointer to an RTNINTERFACE defined in rtnapi.h. This
structure contains information used by the stack to bring up
the driver. Normally the driver simply fills in the unsigned
long dwMtu value with the maximum transmission unit (packet
size), not including the Ethernet header (i.e. 1500 bytes for
a standard Ethernet driver).

\return	A return value of 0 indicates Success, -1 indicates Failure.

*-----------------------------------------------------------------------------------------------**/
RTNDPROC int RtndInitialize(int fDisplayErrorMessages, char *DeviceName, PRTNINTERFACE pNetInterface);

/**--------------------------------------------------------------------------------------------------
\fn	RTNDPROC int RtndConfigure(void *ndp);

\brief	RtndConfigure is called for each instance of the driver when the driver should
configure the NIC hardware. This function should set up receive buffers/transmit
buffers, card modes, events, etc.

\param [in,out]	ndp	Device Pointer. Opaque network device identifier.

\return	A return value of 0 indicates Success, -1 indicates Failure.

\note	The driver should NOT enable interrupts or packet reception on the NIC within
RtndConfigure. The driver should wait until the RtndUpDown routine is called before
enabling interrupts or packet reception.

*-----------------------------------------------------------------------------------------------**/
RTNDPROC int RtndConfigure(void *ndp);

/**--------------------------------------------------------------------------------------------------
\fn	RTNDPROC int RtndUpDown(void *ndp, unsigned short flags, char *options);

\brief	RtndUpDown is called when the stack requires that the driver start (or stop) a
particular NIC card. Given a flag value of non-zero RtndUpDown will bring up the NIC.
An interrupt should be attached to a vector and enabled from the card; DMA should be
turned on, etc. Given a flag value of zero RtndUpDown will bring down the NIC card.
All interrupts should be disabled from the card and interrupt vector should be
released. DMA should be turned off, and the card should be placed in a safe dormant
state. Event handles should be closed, the Receive thread and possibly the Transmit
thread should be terminated, etc.

\param [in,out]	ndp	   	Network Device Pointer. Opaque network device identifier.
\param	flags		   	Flag indicating if the network device is being started or stopped. 1
means bring device up, 0 means bring device down.
\param [in,out]	options	Reserved.

\return	A return value of 0 indicates Success, -1 indicates Failure.

*-----------------------------------------------------------------------------------------------**/
RTNDPROC int RtndUpDown(void *ndp, unsigned short flags, char *options);

/**--------------------------------------------------------------------------------------------------
\fn	RTNDPROC int RtndTransmit(void *mp);

\brief	RtndTransmit is called when the stack requires that the driver transmit a packet
(pointed to by the parameter mp). The packet is in a structure format required by the
RtxTcpIp Stack, which the driver does not need to know. The driver should use the
RtnDecodePacket routine to obtain the network device (ndp), data pointer and data
length for the packet.

\param [in,out]	mp	Pointer to a Packet to transmit.

\return	A return value of 0 indicates Success, -1 indicates Failure.

*-----------------------------------------------------------------------------------------------**/
RTNDPROC int RtndTransmit(void *mp);

/**--------------------------------------------------------------------------------------------------
\fn	RTNDPROC int RtndIoctl(void *ndp, int cmd, char *addr);

\brief	RtndIoctl is called when the stack requires that the driver configure certain NIC
modes or characteristics.

\param [in,out]	ndp 	Network Device Pointer. Opaque network device identifier.
\param	cmd				Control code to execute.
\param [in,out]	addr	If non-null, the address.

\return	A return value of 0 indicates Success, -1 indicates Failure.

*-----------------------------------------------------------------------------------------------**/
RTNDPROC int RtndIoctl(void *ndp, int cmd, char *addr);

/**--------------------------------------------------------------------------------------------------
\fn	RTNDPROC VOID RtndRequest( PRTND_STATUS Status, PCHAR DeviceName, PRTND_REQUEST RtndReq );

\brief	RtndRequest function forwards a request to the underlying driver to query the
requested capabilities and/or statistics of the RTX64 converted NIC card.

\param	Status	  	The RTND_STATUS type set on return from this function which reflects
status of RtndRequest.
\param	DeviceName	Zero terminated network adapter name.
\param	RtndReq   	The RTND_REQUEST type specifying RTX64 network adapter capabilities and
statistics to request.

\return	A VOID.

\see	RTND_REQUEST

*-----------------------------------------------------------------------------------------------**/
RTNDPROC VOID  RtndRequest(PRTND_STATUS Status, PCHAR DeviceName, PRTND_REQUEST RtndReq);




void printTxDiag(DRVDEV *pCardInfo);
void printRxDiag(DRVDEV *pCardInfo);
void printPhyDiag(DRVDEV *pCardInfo);
void printStatsDiag(DRVDEV *pCardInfo);

