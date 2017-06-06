//////////////////////////////////////////////////////////////////
//
// PciReadReg.cpp - cpp file
//
// This file was generated using the RTX64 Application Template for Visual Studio.
//
// Created: 4/28/2016 11:17:26 AM 
// User: medwards
//
//////////////////////////////////////////////////////////////////

#include "IntelNicDiag.h"
#include "e1000_osdep.h"
#include "if_rtx.h"

// This array holds pointers to the virtual memory addresses returned by RtMapMemory.
PCHAR vMemAddr[NUM_REGISTERS];

void listPciDevices(void);
                
void Usage(char *progname)
{
	RtPrintf("\nUsage: \n%s v= VendorID d= DeviceID\n\n", progname);
	exit(-1);
} // Usage



void printTxDiag(DRVDEV *pCardInfo)
{
	struct e1000_hw	*hw = &pCardInfo->pBsdStyleAdapter->hw;

	DebugMsg(("\n\n"));

	DebugMsg(("E1000_STATUS      %08X\n\n", E1000_READ_REG(hw, E1000_STATUS)));


	DebugMsg(("E1000_TDH(0)      %08X\n", E1000_READ_REG(hw, E1000_TDH(0))));
	DebugMsg(("E1000_TDH(1)      %08X\n", E1000_READ_REG(hw, E1000_TDH(1))));
	DebugMsg(("E1000_TDT(0)      %08X\n", E1000_READ_REG(hw, E1000_TDT(0))));
	DebugMsg(("E1000_TDT(1)      %08X\n", E1000_READ_REG(hw, E1000_TDT(1))));

	DebugMsg(("E1000_TDLEN(0)    %08X\n", E1000_READ_REG(hw, E1000_TDLEN(0))));
	DebugMsg(("E1000_TDLEN(1)    %08X\n", E1000_READ_REG(hw, E1000_TDLEN(1))));

	DebugMsg(("E1000_TDBAL(0)    %08X\n", E1000_READ_REG(hw, E1000_TDBAL(0))));
	DebugMsg(("E1000_TDBAL(1)    %08X\n", E1000_READ_REG(hw, E1000_TDBAL(1))));
	DebugMsg(("E1000_TDBAH(0)    %08X\n", E1000_READ_REG(hw, E1000_TDBAH(0))));
	DebugMsg(("E1000_TDBAH(1)    %08X\n", E1000_READ_REG(hw, E1000_TDBAH(1))));


	DebugMsg(("E1000_TXCTL(0)    %08X\n", E1000_READ_REG(hw, E1000_TXCTL(0))));
	DebugMsg(("E1000_TXCTL(1)    %08X\n", E1000_READ_REG(hw, E1000_TXCTL(1))));
	DebugMsg(("E1000_TXDCTL(0)   %08X\n", E1000_READ_REG(hw, E1000_TXDCTL(0))));
	DebugMsg(("E1000_TXDCTL(1)   %08X\n", E1000_READ_REG(hw, E1000_TXDCTL(1))));

	DebugMsg(("E1000_TDWBAL(0)   %08X\n", E1000_READ_REG(hw, E1000_TDWBAL(0))));
	DebugMsg(("E1000_TDWBAL(1)   %08X\n", E1000_READ_REG(hw, E1000_TDWBAL(1))));
	DebugMsg(("E1000_TDWBAH(0)   %08X\n", E1000_READ_REG(hw, E1000_TDWBAH(0))));
	DebugMsg(("E1000_TDWBAH(1)   %08X\n", E1000_READ_REG(hw, E1000_TDWBAH(1))));

	DebugMsg(("E1000_TARC(0)     %08X\n", E1000_READ_REG(hw, E1000_TARC(0))));
	DebugMsg(("E1000_TARC(1)     %08X\n", E1000_READ_REG(hw, E1000_TARC(1))));

	DebugMsg(("E1000_TXDMAC      %08X\n", E1000_READ_REG(hw, E1000_TXDMAC)));

}
void printRxDiag(DRVDEV *pCardInfo)
{
	struct e1000_hw	*hw = &pCardInfo->pBsdStyleAdapter->hw;

	DebugMsg(("\n\n"));
	DebugMsg(("E1000_RDBAL(0)      %08X\n", E1000_READ_REG(hw, E1000_RDBAL(0))));
	DebugMsg(("E1000_RDBAL(1)      %08X\n", E1000_READ_REG(hw, E1000_RDBAL(1))));
	DebugMsg(("E1000_RDBAH(0)      %08X\n", E1000_READ_REG(hw, E1000_RDBAH(0))));
	DebugMsg(("E1000_RDBAH(1)      %08X\n", E1000_READ_REG(hw, E1000_RDBAH(1))));

	DebugMsg(("E1000_RDLEN(0)    %08X\n", E1000_READ_REG(hw, E1000_RDLEN(0))));
	DebugMsg(("E1000_RDLEN(1)    %08X\n", E1000_READ_REG(hw, E1000_RDLEN(1))));

	DebugMsg(("E1000_RDH(0)    %08X\n", E1000_READ_REG(hw, E1000_RDH(0))));
	DebugMsg(("E1000_RDH(1)    %08X\n", E1000_READ_REG(hw, E1000_RDH(1))));
	DebugMsg(("E1000_RDT(0)    %08X\n", E1000_READ_REG(hw, E1000_RDT(0))));
	DebugMsg(("E1000_RDT(1)    %08X\n", E1000_READ_REG(hw, E1000_RDT(1))));


	DebugMsg(("E1000_SRRCTL(0)    %08X\n", E1000_READ_REG(hw, E1000_SRRCTL(0))));
	DebugMsg(("E1000_SRRCTL(1)    %08X\n", E1000_READ_REG(hw, E1000_SRRCTL(1))));
	DebugMsg(("E1000_RXCTL(0)   %08X\n", E1000_READ_REG(hw, E1000_RXCTL(0))));
	DebugMsg(("E1000_RXCTL(1)   %08X\n", E1000_READ_REG(hw, E1000_RXCTL(1))));

	DebugMsg(("E1000_RXDCTL(0)   %08X\n", E1000_READ_REG(hw, E1000_RXDCTL(0))));
	DebugMsg(("E1000_RXDCTL(1)   %08X\n", E1000_READ_REG(hw, E1000_RXDCTL(1))));
	DebugMsg(("E1000_RQDPC(0)   %08X\n", E1000_READ_REG(hw, E1000_RQDPC(0))));
	DebugMsg(("E1000_RQDPC(1)   %08X\n", E1000_READ_REG(hw, E1000_RQDPC(1))));



}

void printPhyDiag(DRVDEV *pCardInfo)
{
	struct e1000_hw	*hw = &pCardInfo->pBsdStyleAdapter->hw;

	struct e1000_phy_info *phy = &hw->phy;
	u16 phy_value;

	DebugMsg(("\n\n"));

	if (!phy->ops.read_reg)
		return;

	DebugMsg(("  e1000_get_phy_id  %08X\n", e1000_get_phy_id(hw)));

	phy->ops.read_reg(hw, PHY_CONTROL, &phy_value); 	DebugMsg(("  PHY_CONTROL			/* Control Register */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_STATUS, &phy_value); 	DebugMsg(("  PHY_STATUS			/* Status Register */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_ID1, &phy_value); 	DebugMsg(("  PHY_ID1				/* Phy Id Reg (word 1) */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_ID2, &phy_value); 	DebugMsg(("  PHY_ID2				/* Phy Id Reg (word 2) */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_AUTONEG_ADV, &phy_value); 	DebugMsg(("  PHY_AUTONEG_ADV		/* Autoneg Advertisement */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_LP_ABILITY, &phy_value); 	DebugMsg(("  PHY_LP_ABILITY		/* Link Partner Ability (Base Page) */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_AUTONEG_EXP, &phy_value); 	DebugMsg(("  PHY_AUTONEG_EXP		/* Autoneg Expansion Reg */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_NEXT_PAGE_TX, &phy_value); 	DebugMsg(("  PHY_NEXT_PAGE_TX	/* Next Page Tx */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_LP_NEXT_PAGE, &phy_value); 	DebugMsg(("  PHY_LP_NEXT_PAGE	/* Link Partner Next Page */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_1000T_CTRL, &phy_value); 	DebugMsg(("  PHY_1000T_CTRL		/* 1000Base-T Control Reg */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_1000T_STATUS, &phy_value); 	DebugMsg(("  PHY_1000T_STATUS	/* 1000Base-T Status Reg */  %04X\n", phy_value));
	phy->ops.read_reg(hw, PHY_EXT_STATUS, &phy_value); 	DebugMsg(("  PHY_EXT_STATUS		/* Extended Status Reg */  %04X\n", phy_value));


	hw->phy.ops.read_reg_locked(hw, HV_PM_CTRL, &phy_value);        DebugMsg(("  HV_PM_CTRL  PHY_REG(770, 17)   PCIe Power Management Control PHY %04X\n", phy_value));
	hw->phy.ops.read_reg_locked(hw, I218_ULP_CONFIG1, &phy_value);  DebugMsg(("  ULP_CONFIG1 PHY_REG(779, 16)   Ultra Low Power Configuration     %04X\n", phy_value));
	hw->phy.ops.read_reg_locked(hw, HV_OEM_BITS, &phy_value);       DebugMsg(("  OEM Bits    PHY_REG(768, 25)   OEM bits                          %04X\n", phy_value));
	hw->phy.ops.read_reg_locked(hw, I82579_LPI_CTRL, &phy_value);   DebugMsg(("  LPI_CTRL    PHY_REG(772, 20)   Low Power Idle Control            %04X\n", phy_value));


}


void printStatsDiag(DRVDEV *pCardInfo)
{
	struct e1000_hw	*hw = &pCardInfo->pBsdStyleAdapter->hw;

	DebugMsg(("\n\n"));
	DebugMsg(("E1000_CRCERRS		/* CRC Error Count - R/clr */                  %08X\n", E1000_READ_REG(hw, E1000_CRCERRS)));
	DebugMsg(("E1000_ALGNERRC		/* Alignment Error Count - R/clr */            %08X\n", E1000_READ_REG(hw, E1000_ALGNERRC)));
	DebugMsg(("E1000_SYMERRS		/* Symbol Error Count - R/clr */               %08X\n", E1000_READ_REG(hw, E1000_SYMERRS)));
	DebugMsg(("E1000_RXERRC		    /* Receive Error Count - R/clr */              %08X\n", E1000_READ_REG(hw, E1000_RXERRC)));
	DebugMsg(("E1000_MPC			/* Missed Packet Count - R/clr */              %08X\n", E1000_READ_REG(hw, E1000_MPC)));
	DebugMsg(("E1000_SCC			/* Single Collision Count - R/clr */           %08X\n", E1000_READ_REG(hw, E1000_SCC)));
	DebugMsg(("E1000_ECOL			/* Excessive Collision Count - R/clr */        %08X\n", E1000_READ_REG(hw, E1000_ECOL)));
	DebugMsg(("E1000_MCC			/* Multiple Collision Count - R/clr */         %08X\n", E1000_READ_REG(hw, E1000_MCC)));
	DebugMsg(("E1000_LATECOL		/* Late Collision Count - R/clr */             %08X\n", E1000_READ_REG(hw, E1000_LATECOL)));
	DebugMsg(("E1000_COLC			/* Collision Count - R/clr */                  %08X\n", E1000_READ_REG(hw, E1000_COLC)));
	DebugMsg(("E1000_DC			    /* Defer Count - R/clr */                      %08X\n", E1000_READ_REG(hw, E1000_DC)));
	DebugMsg(("E1000_TNCRS			/* Tx-No CRS - R/clr */                        %08X\n", E1000_READ_REG(hw, E1000_TNCRS)));
	DebugMsg(("E1000_SEC			/* Sequence Error Count - R/clr */             %08X\n", E1000_READ_REG(hw, E1000_SEC)));
	DebugMsg(("E1000_CEXTERR		/* Carrier Extension Error Count - R/clr */    %08X\n", E1000_READ_REG(hw, E1000_CEXTERR)));
	DebugMsg(("E1000_RLEC			/* Receive Length Error Count - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_RLEC)));
	DebugMsg(("E1000_XONRXC		    /* XON Rx Count - R/clr */                     %08X\n", E1000_READ_REG(hw, E1000_XONRXC)));
	DebugMsg(("E1000_XONTXC		    /* XON Tx Count - R/clr */                     %08X\n", E1000_READ_REG(hw, E1000_XONTXC)));
	DebugMsg(("E1000_XOFFRXC		/* XOFF Rx Count - R/clr */                    %08X\n", E1000_READ_REG(hw, E1000_XOFFRXC)));
	DebugMsg(("E1000_XOFFTXC		/* XOFF Tx Count - R/clr */                    %08X\n", E1000_READ_REG(hw, E1000_XOFFTXC)));
	DebugMsg(("E1000_FCRUC			/* Flow Control Rx Unsupported Count- R/clr */ %08X\n", E1000_READ_REG(hw, E1000_FCRUC)));
	DebugMsg(("E1000_PRC64			/* Packets Rx (64 bytes) - R/clr */            %08X\n", E1000_READ_REG(hw, E1000_PRC64)));
	DebugMsg(("E1000_PRC127		    /* Packets Rx (65-127 bytes) - R/clr */        %08X\n", E1000_READ_REG(hw, E1000_PRC127)));
	DebugMsg(("E1000_PRC255		    /* Packets Rx (128-255 bytes) - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_PRC255)));
	DebugMsg(("E1000_PRC511		    /* Packets Rx (255-511 bytes) - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_PRC511)));
	DebugMsg(("E1000_PRC1023		/* Packets Rx (512-1023 bytes) - R/clr */      %08X\n", E1000_READ_REG(hw, E1000_PRC1023)));
	DebugMsg(("E1000_PRC1522		/* Packets Rx (1024-1522 bytes) - R/clr */     %08X\n", E1000_READ_REG(hw, E1000_PRC1522)));
	DebugMsg(("E1000_GPRC			/* Good Packets Rx Count - R/clr */            %08X\n", E1000_READ_REG(hw, E1000_GPRC)));
	DebugMsg(("E1000_BPRC			/* Broadcast Packets Rx Count - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_BPRC)));
	DebugMsg(("E1000_MPRC			/* Multicast Packets Rx Count - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_MPRC)));
	DebugMsg(("E1000_GPTC			/* Good Packets Tx Count - R/clr */            %08X\n", E1000_READ_REG(hw, E1000_GPTC)));
	DebugMsg(("E1000_GORCL			/* Good Octets Rx Count Low - R/clr */         %08X\n", E1000_READ_REG(hw, E1000_GORCL)));
	DebugMsg(("E1000_GORCH			/* Good Octets Rx Count High - R/clr */        %08X\n", E1000_READ_REG(hw, E1000_GORCH)));
	DebugMsg(("E1000_GOTCL			/* Good Octets Tx Count Low - R/clr */         %08X\n", E1000_READ_REG(hw, E1000_GOTCL)));
	DebugMsg(("E1000_GOTCH			/* Good Octets Tx Count High - R/clr */        %08X\n", E1000_READ_REG(hw, E1000_GOTCH)));
	DebugMsg(("E1000_RNBC			/* Rx No Buffers Count - R/clr */              %08X\n", E1000_READ_REG(hw, E1000_RNBC)));
	DebugMsg(("E1000_RUC			/* Rx Undersize Count - R/clr */               %08X\n", E1000_READ_REG(hw, E1000_RUC)));
	DebugMsg(("E1000_RFC			/* Rx Fragment Count - R/clr */                %08X\n", E1000_READ_REG(hw, E1000_RFC)));
	DebugMsg(("E1000_ROC			/* Rx Oversize Count - R/clr */                %08X\n", E1000_READ_REG(hw, E1000_ROC)));
	DebugMsg(("E1000_RJC			/* Rx Jabber Count - R/clr */                  %08X\n", E1000_READ_REG(hw, E1000_RJC)));
	DebugMsg(("E1000_MGTPRC		    /* Management Packets Rx Count - R/clr */      %08X\n", E1000_READ_REG(hw, E1000_MGTPRC)));
	DebugMsg(("E1000_MGTPDC		    /* Management Packets Dropped Count - R/clr */ %08X\n", E1000_READ_REG(hw, E1000_MGTPDC)));
	DebugMsg(("E1000_MGTPTC		    /* Management Packets Tx Count - R/clr */      %08X\n", E1000_READ_REG(hw, E1000_MGTPTC)));
	DebugMsg(("E1000_TORL			/* Total Octets Rx Low - R/clr */              %08X\n", E1000_READ_REG(hw, E1000_TORL)));
	DebugMsg(("E1000_TORH			/* Total Octets Rx High - R/clr */             %08X\n", E1000_READ_REG(hw, E1000_TORH)));
	DebugMsg(("E1000_TOTL			/* Total Octets Tx Low - R/clr */              %08X\n", E1000_READ_REG(hw, E1000_TOTL)));
	DebugMsg(("E1000_TOTH			/* Total Octets Tx High - R/clr */             %08X\n", E1000_READ_REG(hw, E1000_TOTH)));
	DebugMsg(("E1000_TPR			/* Total Packets Rx - R/clr */                 %08X\n", E1000_READ_REG(hw, E1000_TPR)));
	DebugMsg(("E1000_TPT			/* Total Packets Tx - R/clr */                 %08X\n", E1000_READ_REG(hw, E1000_TPT)));
	DebugMsg(("E1000_PTC64			/* Packets Tx (64 bytes) - R/clr */            %08X\n", E1000_READ_REG(hw, E1000_PTC64)));
	DebugMsg(("E1000_PTC127		    /* Packets Tx (65-127 bytes) - R/clr */        %08X\n", E1000_READ_REG(hw, E1000_PTC127)));
	DebugMsg(("E1000_PTC255		    /* Packets Tx (128-255 bytes) - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_PTC255)));
	DebugMsg(("E1000_PTC511		    /* Packets Tx (256-511 bytes) - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_PTC511)));
	DebugMsg(("E1000_PTC1023		/* Packets Tx (512-1023 bytes) - R/clr */      %08X\n", E1000_READ_REG(hw, E1000_PTC1023)));
	DebugMsg(("E1000_PTC1522		/* Packets Tx (1024-1522 Bytes) - R/clr */     %08X\n", E1000_READ_REG(hw, E1000_PTC1522)));
	DebugMsg(("E1000_MPTC			/* Multicast Packets Tx Count - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_MPTC)));
	DebugMsg(("E1000_BPTC			/* Broadcast Packets Tx Count - R/clr */       %08X\n", E1000_READ_REG(hw, E1000_BPTC)));
	DebugMsg(("E1000_TSCTC			/* TCP Segmentation Context Tx - R/clr */      %08X\n", E1000_READ_REG(hw, E1000_TSCTC)));
	DebugMsg(("E1000_TSCTFC		    /* TCP Segmentation Context Tx Fail - R/clr */ %08X\n", E1000_READ_REG(hw, E1000_TSCTFC)));
	DebugMsg(("E1000_IAC			/* Interrupt Assertion Count */                %08X\n", E1000_READ_REG(hw, E1000_IAC)));
	DebugMsg(("E1000_ICRXPTC		/* Interrupt Cause Rx Pkt Timer Expire Count */%08X\n", E1000_READ_REG(hw, E1000_ICRXPTC)));
	DebugMsg(("E1000_ICRXATC		/* Interrupt Cause Rx Abs Timer Expire Count */%08X\n", E1000_READ_REG(hw, E1000_ICRXATC)));
	DebugMsg(("E1000_ICTXPTC		/* Interrupt Cause Tx Pkt Timer Expire Count */%08X\n", E1000_READ_REG(hw, E1000_ICTXPTC)));
	DebugMsg(("E1000_ICTXATC		/* Interrupt Cause Tx Abs Timer Expire Count */%08X\n", E1000_READ_REG(hw, E1000_ICTXATC)));
	DebugMsg(("E1000_ICTXQEC		/* Interrupt Cause Tx Queue Empty Count */     %08X\n", E1000_READ_REG(hw, E1000_ICTXQEC)));
	DebugMsg(("E1000_ICTXQMTC		/* Interrupt Cause Tx Queue Min Thresh Count */%08X\n", E1000_READ_REG(hw, E1000_ICTXQMTC)));
	DebugMsg(("E1000_ICRXDMTC		/* Interrupt Cause Rx Desc Min Thresh Count */ %08X\n", E1000_READ_REG(hw, E1000_ICRXDMTC)));
	DebugMsg(("E1000_ICRXOC		    /* Interrupt Cause Receiver Overrun Count */   %08X\n", E1000_READ_REG(hw, E1000_ICRXOC)));

}







int _tmain(int argc, _TCHAR * argv[])
{
  
    UCHAR buffer[PCI_COMMON_HDR_LENGTH];
    PPCI_COMMON_CONFIG  pciData = (PPCI_COMMON_CONFIG) buffer;
    PCI_SLOT_NUMBER     slotNumber;           // logical slot number
    ULONG busNumber;                          // interrupt bus number

	int vendor_id = -1;
	int device_id = -1;
	int bar_number = -1;
	int offset = -1;
	int size = -1;
	int iCmd;

	unsigned __int8 returnedByte;
	unsigned __int16 returnedWord;
	unsigned __int32 returnedDWORD;

	DRVDEV *pCardInfo;

	pCardInfo = (DRVDEV *)malloc(sizeof(DRVDEV));

	memset(pCardInfo, 0x00, sizeof(DRVDEV));

	//
	// Parse command line arguments
	//
	for (iCmd = 1; iCmd < argc; iCmd++) {
		switch (argv[iCmd][0]) {

		case 'l':	listPciDevices();
			goto exit;

		case 'v':	vendor_id = strtoul(argv[iCmd] + 2, NULL, 0);
			break;

		case 'd':	device_id = strtoul(argv[iCmd] + 2, NULL, 0);
			break;

		case '?':	Usage(argv[0]);
		} // switch
	} // for 
	
	if ((vendor_id == -1)
		|| (device_id == -1) )
	{
		RtPrintf("IntelNicDiag: Missing or bad parameter\n");
		Usage(argv[0]);
		goto exit;
	}

	busNumber = DeviceSearch(vendor_id, device_id, &slotNumber, pciData);

    //
    // If the device is not found exit
    //
    if (busNumber == DEVICE_NOT_FOUND)
    {
		RtPrintf("IntelNicDiag: The Specified Device was not found on this machine.\n");
    }

    //
    // Initialize device
    //
	offset = 0;
	size = 0x20000;
    if (!DeviceInit(busNumber, &slotNumber, pciData, offset+size))
    {
        DeviceCleanup();
        RtPrintf(_T("IntelNicDiag: Device initialization failed.\n"));
		goto exit;
    }

	pCardInfo->VendorId = vendor_id;
	pCardInfo->DeviceId = device_id;
	pCardInfo->RevId = pciData->RevisionID;
	pCardInfo->BusNumber = busNumber;
	pCardInfo->FuncNumber = slotNumber.u.bits.FunctionNumber;
	pCardInfo->SlotNumber = slotNumber;
	pCardInfo->CsrBase = NULL;
	pCardInfo->FlashBase = NULL;
	pCardInfo->IOBase = 0;


	if (em_attach(pCardInfo) != E1000_SUCCESS)
	{
		DebugMsg(("IntelNicDiag: em_attach errors\n"));
		return FALSE;
	}


	printTxDiag(pCardInfo);
	printRxDiag(pCardInfo);
	printPhyDiag(pCardInfo);
	printStatsDiag(pCardInfo);

    DeviceCleanup();

 exit:               
    return 0;
}
