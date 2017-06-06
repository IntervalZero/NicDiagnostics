//////////////////////////////////////////////////////////////////
//
// PciReadReg.h - header file
//
// This file was generated using the RTX64 Application Template for Visual Studio.
//
// Created: 4/28/2016 11:17:26 AM 
// User: medwards
//
//////////////////////////////////////////////////////////////////

#pragma once
//This define will deprecate all unsupported Microsoft C-runtime functions when compiled under RTSS.
//If using this define, #include <rtapi.h> should remain below all windows headers
//#define UNDER_RTSS_UNSUPPORTED_CRT_APIS

#include <SDKDDKVer.h>

//#include <stdio.h>
//#include <string.h>
//#include <ctype.h>
//#include <conio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <errno.h>
#include <windows.h>
#include <tchar.h>

//#include "if_rtx.h"

#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications.

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS

  
#define DEVICE_NOT_FOUND    -1        // error return
#define NUM_REGISTERS    PCI_TYPE0_ADDRESSES    // TO DO: Modify to number of Device Registers
//
// With mapped memory you can write directly to reference pointers
//
// TO DO:
//    - Setup _baseAddress to point to the base of your mapped memory
//    - Setup    defines for commonly used locations
//
#define BASE_ADDRESS _baseAddress

#define DebugMsg(_Data_) 	{ RtPrintf("IntelNicDiag: "); \
								RtPrintf _Data_ ; } \



//
// FUNCTION PROTOTYPES
//


//
// Search devices
//
int
DeviceSearch(int vendorID,
             int deviceID,
             PCI_SLOT_NUMBER * pSlotNumber,
             PPCI_COMMON_CONFIG  PciData);

//
// Initialize device
//
BOOLEAN
DeviceInit(int busNumber,                  // IN: Interrupt bus number
PCI_SLOT_NUMBER * pSlotNumber,  // IN: Pointer to slot number
PPCI_COMMON_CONFIG PciData,   // IN: PCI card information
int addressRange);


//
// Cleanup
//
void
DeviceCleanup(void);

