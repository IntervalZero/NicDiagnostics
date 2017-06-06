//////////////////////////////////////////////////////////////////
//
// PciReadRegDriver.cpp - cpp file
//
// This file was generated using the RTX64 Application Template for Visual Studio.
//
// Created: 4/28/2016 11:17:26 AM
// User: medwards
//
//////////////////////////////////////////////////////////////////

#include "PciReadReg.h"

#define VENDOR_ID_SIZE 2

// Global Variables
//

extern PCHAR vMemAddr[NUM_REGISTERS];     // pointers to virtual memory addresses returned by RtMapMemory
extern PUCHAR _baseAddress;               // base address of memory mapped space



// DeviceSearch: Searches for the first instance of a given card on the
// machine's PCI buses.  It returns the bus number when the specified card has
// been found, or returns DEVICE_NOT_FOUND (-1) if no device found.
//
int
DeviceSearch(int vendorID,                    // IN: Card Vendor ID
             int deviceID,                    // IN: Card Device ID
             PCI_SLOT_NUMBER * pSlotNumber,   // OUT: Pointer to slot number
             PPCI_COMMON_CONFIG PciData)      // OUT: PCI card information
{
    ULONG bus = 0;;             // bus number
    ULONG deviceNumber = 0;     // logical slot number for the PCI adapter
    ULONG functionNumber = 0;   // function number on the specified adapter
    ULONG bytesWritten = 0;     // return value from RtGetBusDataByOffset
    BOOLEAN bFlag = TRUE;
    int CardIndex = 0;

    pSlotNumber->u.bits.Reserved = 0;

    // All PCI cards have been searched when we are out of PCI busses. (bFlag = FALSE)
    //
    for (bus=0; bFlag; bus++)
    {
         for (deviceNumber=0; deviceNumber < PCI_MAX_DEVICES && bFlag; deviceNumber++)
         {
            pSlotNumber->u.bits.DeviceNumber = deviceNumber;

            for (functionNumber=0; functionNumber < PCI_MAX_FUNCTION; functionNumber++)
            {
                pSlotNumber->u.bits.FunctionNumber = functionNumber;

                bytesWritten = RtGetBusDataByOffset(PCIConfiguration,       // Type of bus data to be retrieved
                                                    bus,                    // Zero-based number of the bus
                                                    pSlotNumber->u.AsULONG, // Logical slot number
                                                    PciData,                // Pointer to a buffer for configuration information
                                                    0,                      // Byte offset into buffer
                                                    PCI_COMMON_HDR_LENGTH); // Length of buffer

                if (bytesWritten == 0)
                {
                    // Out of PCI buses: done.
                    bFlag = FALSE;
                    break;
                }

                if (bytesWritten == VENDOR_ID_SIZE && PciData->VendorID == PCI_INVALID_VENDORID)
                {
                    // No device at this slot number, skip to next slot.
                    break;
                }

                //
                // If device is found return, otherwise continue until all buses
                // have been searched.  Base class and sub-class can also be
                // used to find your specific device.
                //
                if ((PciData->VendorID == vendorID) && (PciData->DeviceID == deviceID))
                {
                        return bus;
                }
            } // functionNumber loop
         } // deviceNumber loop
    } // bus loop

    return DEVICE_NOT_FOUND;
}

//
// DeviceInit: initializes the card found on the machine's PCI bus.
// It returns TRUE if it succeeds otherwise FALSE is returned
//
BOOLEAN
DeviceInit(int busNumber,                  // IN: Interrupt bus number
           PCI_SLOT_NUMBER * pSlotNumber,  // IN: Pointer to slot number
           PPCI_COMMON_CONFIG PciData,   // IN: PCI card information
		   int addressRange)    
{

    LARGE_INTEGER memAddr;      // a base port address
    LARGE_INTEGER tranMemAddr;  // translated base addresses returned by RtMapMemory
    ULONG AddressSpace = 0;     // indicates memory space
    ULONG bytesWritten = 0;
    int i=0;


    //
    // Get the virtual bus addresses for each address register
    // if call fails set to NULL and continue
    //
    for (i=0; i< NUM_REGISTERS; i++)
    {
        memAddr.QuadPart = PciData->u.type0.BaseAddresses[i];

        //
        // Translate the bus relative addresses to system wide addresses.
        //
        if (!RtTranslateBusAddress(PCIBus,               // Bus interface type
                                   busNumber,            // Bus number (zero based)
                                   memAddr,              // Bus-relative address
                                   &AddressSpace,    // Specifies a port number or memory address
                                   &tranMemAddr))    // Pointer to the translated address
        {
            vMemAddr[i] = NULL;
            continue;
        }

        //
        // Map the addresses to virtual addresses the software can use
        //
        vMemAddr[i] = (PCHAR)RtMapMemory(tranMemAddr,    // Base of the physical address range to map
                                         addressRange,   // The length of address range in bytes
                                         MmNonCached);   // Whether or not to use cache
    }

    //
    // Set the command parameter so we can access the PCI device's control registers.
    //
    PciData->Command = (PCI_ENABLE_IO_SPACE | PCI_ENABLE_MEMORY_SPACE |
                    PCI_ENABLE_BUS_MASTER | PCI_ENABLE_WRITE_AND_INVALIDATE);


    bytesWritten = RtSetBusDataByOffset(PCIConfiguration,          // Type of bus data to be set
                                        busNumber,                 // Bus number (zero based)
                                        pSlotNumber->u.AsULONG, // Logical slot number
                                        PciData,                   // Pointer to buffer containing configuration information
                                        0,                         // Byte offset in the buffer
                                        PCI_COMMON_HDR_LENGTH);    // Number of bytes in the buffer

    if (bytesWritten == 0)
        return FALSE;

    Sleep(500);


    return TRUE;
}


//
// DeviceCleanup: Cleanup interrupt vector and other objects
//
void
DeviceCleanup(void)  // IN: Handle of interrupt from RtAttachInterruptVector call
{
    BOOLEAN bResult = FALSE;
    int i = 0;


    for (i=0; i< NUM_REGISTERS; i++)
    {
        if (vMemAddr[i] != NULL)
        {
            RtUnmapMemory(vMemAddr[i]);
            vMemAddr[i] = NULL;
        }
    }
 
}

void listPciDevices(void)
{
	
		PCI_SLOT_NUMBER     SlotNumber;
		PPCI_COMMON_CONFIG  PciData;
		UCHAR               buffer[PCI_COMMON_HDR_LENGTH];
		ULONG       i;                    // logical slot number for the PCI adapter
		ULONG       f;                    // function number on the specified adapter
		ULONG       bytesWritten;         // return value from HalGetBusDataByOffset
		ULONG       bus;                  // bus number
		BOOLEAN     flag;
		ULONG		Offset = 0;
		ULONG		nothingWritten = 0;
		BOOL fResult;



		PciData = (PPCI_COMMON_CONFIG)buffer;
		SlotNumber.u.bits.Reserved = 0;
		flag = TRUE;

		for (bus = 0; bus<10; bus++) {

			for (i = 0; i<PCI_MAX_DEVICES && flag; i++) {
				SlotNumber.u.bits.DeviceNumber = i;

				for (f = 0; f<PCI_MAX_FUNCTION; f++) {
					SlotNumber.u.bits.FunctionNumber = f;

					bytesWritten = RtGetBusDataByOffset(
						PCIConfiguration,
						bus,
						SlotNumber.u.AsULONG,
						PciData,
						Offset,
						PCI_COMMON_HDR_LENGTH
						);

					if (bytesWritten == nothingWritten) {
						// out of PCI buses 
						RtPrintf("%d:%d:%d (Bus:Device:Func) No Data \n", bus, i, f);
						continue;
					}



					if (PciData->VendorID == PCI_INVALID_VENDORID) {
						// function numbers can be skipped for multi-port NIC cards,
						// continue to the next function number.
						continue;
					}

					RtPrintf("%d:%d:%d (Bus:Device:Func) VendorID=%x  DeviceID=%x Intr=%d\n", bus, i, f, PciData->VendorID, PciData->DeviceID, PciData->u.type0.InterruptPin);

				}
			}
		}

	
}

