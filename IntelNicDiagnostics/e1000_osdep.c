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


#include "e1000_api.h"
#include "pcireg.h"



// RTX_PORT
u32 pci_read_config(DRVDEV *pCardInfo, u32 offset, u32 len)
{
	u32 retValue = 0;



	if (len <= sizeof(u32))
	{
		RtGetBusDataByOffset(PCIConfiguration,
			pCardInfo->BusNumber,
			pCardInfo->SlotNumber.u.AsULONG,
			(PVOID)&retValue,
			offset,
			len
			);
	}

	return(retValue);
}


void pci_write_config(DRVDEV *pCardInfo, u32 offset, u32 data, u32 len)
{

	if (len <= sizeof(u32))
	{
		RtSetBusDataByOffset(PCIConfiguration,
			pCardInfo->BusNumber,
			pCardInfo->SlotNumber.u.AsULONG,
			(PVOID)&data,
			offset,
			len
			);
	}

	return;

}

void emInitializeCriticalLock(HFREECRITICALLOCK *pLock, char * szIndentifier)
{
	strcpy(pLock->lockSymbolic, szIndentifier);
	pLock->lock = 0;
}

void emDeleteCriticalLock(HFREECRITICALLOCK *pLock)
{

}


void emEnterCriticalLock(HFREECRITICALLOCK *pLock)
{
	DWORD spins = 0;

	if (!InterlockedCompareExchange(&(pLock->lock), 1, 0))
	{
		// got the lock right away
		pLock->lockOwnerThreadID = GetCurrentThreadId();
		pLock->lockOwnerThreadPriority = RtGetThreadPriority(GetCurrentThread());
		return;
	}
	else
	{
		int Error = 0;

		HANDLE			hThread;
		int				newPriority, originalPriority;

		hThread = GetCurrentThread();
		newPriority = originalPriority = RtGetThreadPriority(hThread);
		pLock->lockRequestorThreadPriority = originalPriority;

		while (InterlockedCompareExchange(&(pLock->lock), 1, 0))
		{
			--newPriority;
			if (newPriority <= 0)
				newPriority = 1;
			RtSetThreadPriority(hThread, newPriority);

			spins++;

			if (spins > 1000)
			{	// Dead locked but we shouldn't be.  Developer has a problem in his design.
				// OK Give up.  We expect it is impossible  to reach this point.  But given murphey's law...
				// To help early development we return here with out actually having the lock.
				//  The RtPrintf will show even if verbose mode is not set.
				RtSetThreadPriority(hThread, originalPriority);
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. Number of spins=%d\n", spins);
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. lockOwnerThreadID=%d\n", pLock->lockOwnerThreadID);
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. lockOwnerThreadPriority=%d\n", pLock->lockOwnerThreadPriority);
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. lockOwner Identifier=%s\n", pLock->lockSymbolic);
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. lockRequestor ThreadID=%d\n", GetCurrentThreadId());
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. lockRequestor starting ThreadPriority=%d\n", pLock->lockRequestorThreadPriority);
				RtPrintf("RtIgbEnterCriticalLock. Excessive time in spin lock. last number for max spins=%d\n", pLock->maxNumberOfSpins);
				return;
			}
			else if (spins > 127)
			{	// Dead locked and we are at low priority.
				// This can happen if the thread owns a mutex and is at another priority level
				// In that case RtSetThreadPriority won't call the scheduler
				// The last resort is to sleep for least amount of time.
				LARGE_INTEGER	Duration;

				RtSetThreadPriority(hThread, originalPriority);
				Duration.QuadPart = 1;  // Any nonzero value will sleep for 1 or 2 RtssHalClockTimerPeriods
				RtSleepFt(&Duration);
			}
		}

		// The while() loop stopped because we got the lock via the atomic operation
		// This is the expected path.  Fix up our priority and be done.
		RtSetThreadPriority(hThread, originalPriority);
	}
	// got the lock after some spins
	pLock->lockOwnerThreadID = GetCurrentThreadId();
	pLock->lockOwnerThreadPriority = RtGetThreadPriority(GetCurrentThread());
	if (spins > pLock->maxNumberOfSpins)
		pLock->maxNumberOfSpins = spins;
}


void emLeaveCriticalLock(HFREECRITICALLOCK *pLock)
{
	if (InterlockedCompareExchange(&(pLock->lock), 0, 1) != 1)
	{
		RtPrintf("RtIgbLeaveCriticalLock with invalid lock.  LockOwner Identifier=%s\n", pLock->lockSymbolic);
		pLock->lock = 0;
	}
}


/*
 * NOTE: the following routines using the e1000 
 * 	naming style are provided to the shared
 *	code but are OS specific
 */
void
e1000_write_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	pci_write_config(((struct e1000_osdep *)hw->back)->dev, reg, *value, 2);
}

void
e1000_read_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	*value = pci_read_config(((struct e1000_osdep *)hw->back)->dev, reg, 2);
}

void
e1000_pci_set_mwi(struct e1000_hw *hw)
{
	pci_write_config(((struct e1000_osdep *)hw->back)->dev, PCIR_COMMAND,
	    (hw->bus.pci_cmd_word | CMD_MEM_WRT_INVALIDATE), 2);
}

void
e1000_pci_clear_mwi(struct e1000_hw *hw)
{
	pci_write_config(((struct e1000_osdep *)hw->back)->dev, PCIR_COMMAND,
	    (hw->bus.pci_cmd_word & ~CMD_MEM_WRT_INVALIDATE), 2);
}






/*
* Find the requested capability and return the offset in
* configuration space via the pointer provided.  The function returns
* 0 on success and an error code otherwise.
*/
int
pci_find_cap(DRVDEV *pCardInfo, int capability, int *capreg)
{
	//struct pci_devinfo *dinfo = device_get_ivars(child);
	//pcicfgregs *cfg = &dinfo->cfg;
	DWORD status;
	unsigned __int8 ptr;


	

	/*
	* Check the CAP_LIST bit of the PCI status register first.
	*/
	status = pci_read_config(pCardInfo, PCIR_STATUS, 2);//     pci_read_config(child, PCIR_STATUS, 2);
	if (!(status & PCIM_STATUS_CAPPRESENT))
		return (ENXIO);

	/*
	* Determine the start pointer of the capabilities list.
	*/
	//switch (cfg->hdrtype & PCIM_HDRTYPE) {
	//case PCIM_HDRTYPE_NORMAL:
	//case PCIM_HDRTYPE_BRIDGE:
	//	ptr = PCIR_CAP_PTR;
	//	break;
	//case PCIM_HDRTYPE_CARDBUS:
	//	ptr = PCIR_CAP_PTR_2;
	//	break;
	//default:
	//	/* XXX: panic? */
	//	return (ENXIO);		/* no extended capabilities support */
	//}
	ptr = PCIR_CAP_PTR;
	ptr = pci_read_config(pCardInfo, ptr, 1);

	/*
	* Traverse the capabilities list.
	*/
	while (ptr != 0) {
		if (pci_read_config(pCardInfo, ptr + PCICAP_ID, 1) == capability) {
			if (capreg != NULL)
				*capreg = ptr;
			return (0);
		}
		ptr = pci_read_config(pCardInfo, ptr + PCICAP_NEXTPTR, 1);
	}

	return (-1);
}


/* Find offset of a specific capability.  Returns 0 on failure. */




/*
* Read the PCI Express capabilities
*/
int32_t
e1000_read_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	return (-1);

	// unsupported ....
	////device_t dev = ((struct e1000_osdep *)hw->back)->dev;
	//u32	offset;

	//pci_find_cap(dev, PCIY_EXPRESS, &offset);
	//*value = pci_read_config(dev, offset + reg, 2);
	//return (E1000_SUCCESS);
}

/*
 * Write the PCI Express capabilities
 */
int32_t
e1000_write_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	return (-1);
	// unsupported ....
	//device_t dev = ((struct e1000_osdep *)hw->back)->dev;
	//u32	offset;

	//pci_find_cap(dev, PCIY_EXPRESS, &offset);
	//pci_write_config(dev, offset + reg, *value, 2);
	//return (E1000_SUCCESS);
}
