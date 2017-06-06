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


#include "pcireg.h"
#include "e1000_hw.h"
#include "if_rtx.h"

//#include "e1000_osdep.h"
//#include "if_lem.h"
#include "Ether.h"

extern DRV_GLOBALS * pGlobals;
extern PCHAR vMemAddr[];

#define DebugMsg(_Data_) 	{ RtPrintf("IntelNicDiag: "); \
								RtPrintf _Data_ ; } \


#define EM_MULTIQUEUE 1
#define IFCAP_RXCSUM 0   /* can offload checksum on RX */

/*********************************************************************
*  PCI Device ID Table
*
*  Used by probe to select devices to load on
*  Last field stores an index into e1000_strings
*  Last entry must be all 0s
*
*  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
*********************************************************************/

em_vendor_info_t em_vendor_info_array[] =
{
	/* Intel(R) PRO/1000 Network Connection */
	{ 0x8086, E1000_DEV_ID_82571EB_COPPER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_FIBER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_SERDES, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_SERDES_DUAL, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_SERDES_QUAD, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_QUAD_COPPER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_QUAD_COPPER_LP, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571EB_QUAD_FIBER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82571PT_QUAD_COPPER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82572EI_COPPER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82572EI_FIBER, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82572EI_SERDES, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82572EI, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82573E, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82573E_IAMT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82573L, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82583V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_COPPER_SPT, 	PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_SERDES_SPT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_COPPER_DPT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_SERDES_DPT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_M_AMT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_AMT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_C, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IFE, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IFE_GT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IFE_G, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_M, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH8_82567V_3, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_M_AMT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_AMT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_C, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_M, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_M_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IFE, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IFE_GT, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_IFE_G, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH9_BM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82574L, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_82574LA, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH10_R_BM_LM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH10_R_BM_LF, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH10_R_BM_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH10_D_BM_LM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH10_D_BM_LF, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_ICH10_D_BM_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_M_HV_LM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_M_HV_LC, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_D_HV_DM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_D_HV_DC, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH2_LV_LM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH2_LV_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_LPT_I217_LM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_LPT_I217_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_LPTLP_I218_LM, 	PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_LPTLP_I218_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_I218_LM2, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_I218_V2, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_I218_LM3, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_I218_V3, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_LM, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_V, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_LM2, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_V2, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_LBG_I219_LM3, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_LM4, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_V4, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_LM5, PCI_ANY_ID, PCI_ANY_ID, 0 },
	{ 0x8086, E1000_DEV_ID_PCH_SPT_I219_V5, PCI_ANY_ID, PCI_ANY_ID, 0 },
	/* required last entry */
	{ 0, 0, 0, 0, 0 }
};


/*
* em_release_hw_control resets {CTRL_EXT|FWSM}:DRV_LOAD bit.
* For ASF and Pass Through versions of f/w this means that
* the driver is no longer loaded. For AMT versions of the
* f/w this means that the network i/f is closed.
*/
static void
em_release_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext, swsm;

	if (!adapter->has_manage)
		return;

	if (adapter->hw.mac.type == e1000_82573) {
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
			swsm & ~E1000_SWSM_DRV_LOAD);
		return;
	}
	/* else */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
		ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
	return;
}

/*
* em_get_hw_control sets the {CTRL_EXT|FWSM}:DRV_LOAD bit.
* For ASF and Pass Through versions of f/w this means
* that the driver is loaded. For AMT version type f/w
* this means that the network i/f is open.
*/
static void
em_get_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext, swsm;

	if (adapter->hw.mac.type == e1000_82573) {
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
			swsm | E1000_SWSM_DRV_LOAD);
		return;
	}
	/* else */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
		ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	return;
}

/*
** WOL in the newer chipset interfaces (pchlan)
** require thing to be copied into the phy
*/
static int
em_enable_phy_wakeup(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mreg, ret = 0;
	u16 preg;
	int i;

	/* copy MAC RARs to PHY RARs */
	e1000_copy_rx_addrs_to_phy_ich8lan(hw);

	/* copy MAC MTA to PHY MTA */
	for (i = 0; i < adapter->hw.mac.mta_reg_count; i++) {
		mreg = E1000_READ_REG_ARRAY(hw, E1000_MTA, i);
		e1000_write_phy_reg(hw, BM_MTA(i), (u16)(mreg & 0xFFFF));
		e1000_write_phy_reg(hw, BM_MTA(i) + 1,
			(u16)((mreg >> 16) & 0xFFFF));
	}

	/* configure PHY Rx Control register */
	e1000_read_phy_reg(&adapter->hw, BM_RCTL, &preg);
	mreg = E1000_READ_REG(hw, E1000_RCTL);
	if (mreg & E1000_RCTL_UPE)
		preg |= BM_RCTL_UPE;
	if (mreg & E1000_RCTL_MPE)
		preg |= BM_RCTL_MPE;
	preg &= ~(BM_RCTL_MO_MASK);
	if (mreg & E1000_RCTL_MO_3)
		preg |= (((mreg & E1000_RCTL_MO_3) >> E1000_RCTL_MO_SHIFT)
		<< BM_RCTL_MO_SHIFT);
	if (mreg & E1000_RCTL_BAM)
		preg |= BM_RCTL_BAM;
	if (mreg & E1000_RCTL_PMCF)
		preg |= BM_RCTL_PMCF;
	mreg = E1000_READ_REG(hw, E1000_CTRL);
	if (mreg & E1000_CTRL_RFCE)
		preg |= BM_RCTL_RFCE;
	e1000_write_phy_reg(&adapter->hw, BM_RCTL, preg);

	/* enable PHY wakeup in MAC register */
	E1000_WRITE_REG(hw, E1000_WUC,
		E1000_WUC_PHY_WAKE | E1000_WUC_PME_EN);
	E1000_WRITE_REG(hw, E1000_WUFC, adapter->wol);

	/* configure and enable PHY wakeup in PHY registers */
	e1000_write_phy_reg(&adapter->hw, BM_WUFC, adapter->wol);
	e1000_write_phy_reg(&adapter->hw, BM_WUC, E1000_WUC_PME_EN);

	/* activate PHY wakeup */
	ret = hw->phy.ops.acquire(hw);
	if (ret) {
		printf("Could not acquire PHY\n");
		return ret;
	}
	e1000_write_phy_reg_mdic(hw, IGP01E1000_PHY_PAGE_SELECT,
		(BM_WUC_ENABLE_PAGE << IGP_PAGE_SHIFT));
	ret = e1000_read_phy_reg_mdic(hw, BM_WUC_ENABLE_REG, &preg);
	if (ret) {
		printf("Could not read PHY page 769\n");
		goto out;
	}
	preg |= BM_WUC_ENABLE_BIT | BM_WUC_HOST_WU_BIT;
	ret = e1000_write_phy_reg_mdic(hw, BM_WUC_ENABLE_REG, preg);
	if (ret)
		printf("Could not set PHY Host Wakeup bit\n");
out:
	hw->phy.ops.release(hw);

	return ret;
}

static void
em_led_func(void *arg, int onoff)
{
	struct adapter	*adapter = arg;

	//EM_CORE_LOCK(adapter);  // RTX_PORT
	if (onoff) {
		e1000_setup_led(&adapter->hw);
		e1000_led_on(&adapter->hw);
	}
	else {
		e1000_led_off(&adapter->hw);
		e1000_cleanup_led(&adapter->hw);
	}
	//EM_CORE_UNLOCK(adapter);    RTX_PORT
}

/*
** Disable the L0S and L1 LINK states
*/
static void
em_disable_aspm(DRVDEV *pCardInfo)
{
	int		base, reg;
	u16		link_cap, link_ctrl;
	//device_t	dev = adapter->dev;
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;

	switch (adapter->hw.mac.type) {
	case e1000_82573:
	case e1000_82574:
	case e1000_82583:
		break;
	default:
		return;
	}
	if (pci_find_cap(pCardInfo, PCIY_EXPRESS, &base) != 0) 
			return;
	reg = base + PCIER_LINK_CAP;
	link_cap = pci_read_config(pCardInfo, reg, 2);
	if ((link_cap & PCIEM_LINK_CAP_ASPM) == 0)
		return;
	reg = base + PCIER_LINK_CTL;
	link_ctrl = pci_read_config(pCardInfo, reg, 2);
	link_ctrl &= ~PCIEM_LINK_CTL_ASPMC;
	pci_write_config(pCardInfo, reg, link_ctrl, 2);
	return;
}

/**********************************************************************
*
*  Update the board statistics counters.
*
**********************************************************************/
static void
em_update_stats_counters(struct adapter *adapter)
{

	if (adapter->hw.phy.media_type == e1000_media_type_copper ||
		(E1000_READ_REG(&adapter->hw, E1000_STATUS) & E1000_STATUS_LU)) {
		adapter->stats.symerrs += E1000_READ_REG(&adapter->hw, E1000_SYMERRS);
		adapter->stats.sec += E1000_READ_REG(&adapter->hw, E1000_SEC);
	}
	adapter->stats.crcerrs += E1000_READ_REG(&adapter->hw, E1000_CRCERRS);
	adapter->stats.mpc += E1000_READ_REG(&adapter->hw, E1000_MPC);
	adapter->stats.scc += E1000_READ_REG(&adapter->hw, E1000_SCC);
	adapter->stats.ecol += E1000_READ_REG(&adapter->hw, E1000_ECOL);

	adapter->stats.mcc += E1000_READ_REG(&adapter->hw, E1000_MCC);
	adapter->stats.latecol += E1000_READ_REG(&adapter->hw, E1000_LATECOL);
	adapter->stats.colc += E1000_READ_REG(&adapter->hw, E1000_COLC);
	adapter->stats.dc += E1000_READ_REG(&adapter->hw, E1000_DC);
	adapter->stats.rlec += E1000_READ_REG(&adapter->hw, E1000_RLEC);
	adapter->stats.xonrxc += E1000_READ_REG(&adapter->hw, E1000_XONRXC);
	adapter->stats.xontxc += E1000_READ_REG(&adapter->hw, E1000_XONTXC);
	adapter->stats.xoffrxc += E1000_READ_REG(&adapter->hw, E1000_XOFFRXC);
	adapter->stats.xofftxc += E1000_READ_REG(&adapter->hw, E1000_XOFFTXC);
	adapter->stats.fcruc += E1000_READ_REG(&adapter->hw, E1000_FCRUC);
	adapter->stats.prc64 += E1000_READ_REG(&adapter->hw, E1000_PRC64);
	adapter->stats.prc127 += E1000_READ_REG(&adapter->hw, E1000_PRC127);
	adapter->stats.prc255 += E1000_READ_REG(&adapter->hw, E1000_PRC255);
	adapter->stats.prc511 += E1000_READ_REG(&adapter->hw, E1000_PRC511);
	adapter->stats.prc1023 += E1000_READ_REG(&adapter->hw, E1000_PRC1023);
	adapter->stats.prc1522 += E1000_READ_REG(&adapter->hw, E1000_PRC1522);
	adapter->stats.gprc += E1000_READ_REG(&adapter->hw, E1000_GPRC);
	adapter->stats.bprc += E1000_READ_REG(&adapter->hw, E1000_BPRC);
	adapter->stats.mprc += E1000_READ_REG(&adapter->hw, E1000_MPRC);
	adapter->stats.gptc += E1000_READ_REG(&adapter->hw, E1000_GPTC);

	/* For the 64-bit byte counters the low dword must be read first. */
	/* Both registers clear on the read of the high dword */

	adapter->stats.gorc += E1000_READ_REG(&adapter->hw, E1000_GORCL) +
		((u64)E1000_READ_REG(&adapter->hw, E1000_GORCH) << 32);
	adapter->stats.gotc += E1000_READ_REG(&adapter->hw, E1000_GOTCL) +
		((u64)E1000_READ_REG(&adapter->hw, E1000_GOTCH) << 32);

	adapter->stats.rnbc += E1000_READ_REG(&adapter->hw, E1000_RNBC);
	adapter->stats.ruc += E1000_READ_REG(&adapter->hw, E1000_RUC);
	adapter->stats.rfc += E1000_READ_REG(&adapter->hw, E1000_RFC);
	adapter->stats.roc += E1000_READ_REG(&adapter->hw, E1000_ROC);
	adapter->stats.rjc += E1000_READ_REG(&adapter->hw, E1000_RJC);

	adapter->stats.tor += E1000_READ_REG(&adapter->hw, E1000_TORH);
	adapter->stats.tot += E1000_READ_REG(&adapter->hw, E1000_TOTH);

	adapter->stats.tpr += E1000_READ_REG(&adapter->hw, E1000_TPR);
	adapter->stats.tpt += E1000_READ_REG(&adapter->hw, E1000_TPT);
	adapter->stats.ptc64 += E1000_READ_REG(&adapter->hw, E1000_PTC64);
	adapter->stats.ptc127 += E1000_READ_REG(&adapter->hw, E1000_PTC127);
	adapter->stats.ptc255 += E1000_READ_REG(&adapter->hw, E1000_PTC255);
	adapter->stats.ptc511 += E1000_READ_REG(&adapter->hw, E1000_PTC511);
	adapter->stats.ptc1023 += E1000_READ_REG(&adapter->hw, E1000_PTC1023);
	adapter->stats.ptc1522 += E1000_READ_REG(&adapter->hw, E1000_PTC1522);
	adapter->stats.mptc += E1000_READ_REG(&adapter->hw, E1000_MPTC);
	adapter->stats.bptc += E1000_READ_REG(&adapter->hw, E1000_BPTC);

	/* Interrupt Counts */

	adapter->stats.iac += E1000_READ_REG(&adapter->hw, E1000_IAC);
	adapter->stats.icrxptc += E1000_READ_REG(&adapter->hw, E1000_ICRXPTC);
	adapter->stats.icrxatc += E1000_READ_REG(&adapter->hw, E1000_ICRXATC);
	adapter->stats.ictxptc += E1000_READ_REG(&adapter->hw, E1000_ICTXPTC);
	adapter->stats.ictxatc += E1000_READ_REG(&adapter->hw, E1000_ICTXATC);
	adapter->stats.ictxqec += E1000_READ_REG(&adapter->hw, E1000_ICTXQEC);
	adapter->stats.ictxqmtc += E1000_READ_REG(&adapter->hw, E1000_ICTXQMTC);
	adapter->stats.icrxdmtc += E1000_READ_REG(&adapter->hw, E1000_ICRXDMTC);
	adapter->stats.icrxoc += E1000_READ_REG(&adapter->hw, E1000_ICRXOC);

	if (adapter->hw.mac.type >= e1000_82543) {
		adapter->stats.algnerrc +=
			E1000_READ_REG(&adapter->hw, E1000_ALGNERRC);
		adapter->stats.rxerrc +=
			E1000_READ_REG(&adapter->hw, E1000_RXERRC);
		adapter->stats.tncrs +=
			E1000_READ_REG(&adapter->hw, E1000_TNCRS);
		adapter->stats.cexterr +=
			E1000_READ_REG(&adapter->hw, E1000_CEXTERR);
		adapter->stats.tsctc +=
			E1000_READ_REG(&adapter->hw, E1000_TSCTC);
		adapter->stats.tsctfc +=
			E1000_READ_REG(&adapter->hw, E1000_TSCTFC);
	}
}

void
em_enable_intr(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ims_mask = IMS_ENABLE_MASK;

	if (hw->mac.type == e1000_82574) {
		E1000_WRITE_REG(hw, EM_EIAC, adapter->ims);
		ims_mask |= adapter->ims;
	}
	E1000_WRITE_REG(hw, E1000_IMS, ims_mask);
}

static void
em_disable_intr(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	if (hw->mac.type == e1000_82574)
		E1000_WRITE_REG(hw, EM_EIAC, 0);
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, 0xffffffff);
}


static void
em_update_link_status(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	struct e1000_hw *hw = &adapter->hw;
	if_t ifp = adapter->ifp;
	//device_t dev = adapter->dev;
	struct tx_ring *txr = adapter->tx_rings;
	u32 link_check = 0;

	/* Get the cached link value or read phy for real */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			if (hw->mac.type == e1000_pch_spt)
				msec_delay(50);
			/* Do the work to read phy */
			e1000_check_for_link(hw);
			link_check = !hw->mac.get_link_status;
			if (link_check) /* ESB2 fix */
				e1000_cfg_on_link_up(hw);
		}
		else
			link_check = TRUE;
		break;
	case e1000_media_type_fiber:
		e1000_check_for_link(hw);
		link_check = (E1000_READ_REG(hw, E1000_STATUS) &
			E1000_STATUS_LU);
		break;
	case e1000_media_type_internal_serdes:
		e1000_check_for_link(hw);
		link_check = adapter->hw.mac.serdes_has_link;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}

	/* Now check for a transition */
	if (link_check && (adapter->link_active == 0)) {
		e1000_get_speed_and_duplex(hw, &adapter->link_speed,
			&adapter->link_duplex);
		/* Check if we must disable SPEED_MODE bit on PCI-E */
		if ((adapter->link_speed != SPEED_1000) &&
			((hw->mac.type == e1000_82571) ||
			(hw->mac.type == e1000_82572))) {
			int tarc0;
			tarc0 = E1000_READ_REG(hw, E1000_TARC(0));
			tarc0 &= ~TARC_SPEED_MODE_BIT;
			E1000_WRITE_REG(hw, E1000_TARC(0), tarc0);
		}
		adapter->link_active = 1;
		adapter->smartspeed = 0;
	}
	else if (!link_check && (adapter->link_active == 1)) {
		adapter->link_speed = 0;
		adapter->link_duplex = 0;
		adapter->link_active = 0;

	}
}

/*
** The 3 following flush routines are used as a workaround in the
** I219 client parts and only for them.
**
** em_flush_tx_ring - remove all descriptors from the tx_ring
**
** We want to clear all pending descriptors from the TX ring.
** zeroing happens when the HW reads the regs. We  assign the ring itself as
** the data of the next descriptor. We don't care about the data we are about
** to reset the HW.
*/
void
em_flush_tx_ring(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	struct e1000_hw		*hw = &adapter->hw;
	//struct tx_ring		*txr = adapter->tx_rings;
	//struct e1000_tx_desc	*txd;
	u32			tctl, txd_lower = E1000_TXD_CMD_IFCS;
	u16			size = 512;

	tctl = E1000_READ_REG(hw, E1000_TCTL);
	E1000_WRITE_REG(hw, E1000_TCTL, tctl | E1000_TCTL_EN);

	//txd = &txr->tx_base[txr->next_avail_desc++];
	//if (txr->next_avail_desc == adapter->num_tx_desc)
	//	txr->next_avail_desc = 0;

	/* Just use the ring as a dummy buffer addr */
	//txd->buffer_addr = txr->txdma.dma_paddr;
	//txd->lower.data = htole32(txd_lower | size);
	//txd->upper.data = 0;

	/* flush descriptors to memory before notifying the HW */
	//wmb();

	//E1000_WRITE_REG(hw, E1000_TDT(0), txr->next_avail_desc);
	E1000_WRITE_REG(hw, E1000_TDT(0), 0);  
	//mb();
	usec_delay(250);
}


/*
** em_flush_rx_ring - remove all descriptors from the rx_ring
**
** Mark all descriptors in the RX ring as consumed and disable the rx ring
*/
void
em_flush_rx_ring(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	struct e1000_hw	*hw = &adapter->hw;
	u32		rctl, rxdctl;

	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	E1000_WRITE_FLUSH(hw);
	usec_delay(150);

	rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
	/* zero the lower 14 bits (prefetch and host thresholds) */
	rxdctl &= 0xffffc000;
	/*
	* update thresholds: prefetch threshold to 31, host threshold to 1
	* and make sure the granularity is "descriptors" and not "cache lines"
	*/
	rxdctl |= (0x1F | (1 << 8) | E1000_RXDCTL_THRESH_UNIT_DESC);
	E1000_WRITE_REG(hw, E1000_RXDCTL(0), rxdctl);

	/* momentarily enable the RX ring for the changes to take effect */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl | E1000_RCTL_EN);
	E1000_WRITE_FLUSH(hw);
	usec_delay(150);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
}

/*
** em_flush_desc_rings - remove all descriptors from the descriptor rings
**
** In i219, the descriptor rings must be emptied before resetting the HW
** or before changing the device state to D3 during runtime (runtime PM).
**
** Failure to do this will cause the HW to enter a unit hang state which can
** only be released by PCI reset on the device
**
*/
void
em_flush_desc_rings(DRVDEV *pCardInfo)
{
	//device_t	dev = adapter->dev;
	u16		hang_state;
	u32		fext_nvm11, tdlen;
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	struct e1000_hw	*hw = &adapter->hw;

	/* First, disable MULR fix in FEXTNVM11 */
	fext_nvm11 = E1000_READ_REG(hw, E1000_FEXTNVM11);
	fext_nvm11 |= E1000_FEXTNVM11_DISABLE_MULR_FIX;
	E1000_WRITE_REG(hw, E1000_FEXTNVM11, fext_nvm11);

	/* do nothing if we're not in faulty state, or if the queue is empty */
	tdlen = E1000_READ_REG(hw, E1000_TDLEN(0));
	hang_state = pci_read_config(pCardInfo, PCICFG_DESC_RING_STATUS, 2);
	if (!(hang_state & FLUSH_DESC_REQUIRED) || !tdlen)
		return;
	em_flush_tx_ring(pCardInfo);

	/* recheck, maybe the fault is caused by the rx ring */
	hang_state = pci_read_config(pCardInfo, PCICFG_DESC_RING_STATUS, 2);
	if (hang_state & FLUSH_DESC_REQUIRED)
		em_flush_rx_ring(pCardInfo);
}


/*********************************************************************
*
*  This routine disables all traffic on the adapter by issuing a
*  global reset on the MAC and deallocates TX/RX buffers.
*
*  This routine should always be called with BOTH the CORE
*  and TX locks.
**********************************************************************/

void
em_stop(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	if_t ifp = adapter->ifp;
	struct tx_ring	*txr = adapter->tx_rings;

	//EM_CORE_LOCK_ASSERT(adapter);   RTX_PORT

	INIT_DEBUGOUT("em_stop: begin");

	em_disable_intr(adapter);
	//callout_stop(&adapter->timer);

	/* Tell the stack that the interface is no longer active */
	//if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, IFF_DRV_RUNNING);

	/* Disarm Hang Detection. */
	//for (int i = 0; i < adapter->num_queues; i++, txr++) {
	//	EM_TX_LOCK(txr);
	//	txr->busy = EM_TX_IDLE;
	//	EM_TX_UNLOCK(txr);
	//}

	/* I219 needs some special flushing to avoid hangs */
	if (adapter->hw.mac.type == e1000_pch_spt)
		em_flush_desc_rings(pCardInfo);

	e1000_reset_hw(&adapter->hw);
	E1000_WRITE_REG(&adapter->hw, E1000_WUC, 0);

	e1000_led_off(&adapter->hw);
	e1000_cleanup_led(&adapter->hw);
}


/*********************************************************************
*
*  Initialize the hardware to a configuration
*  as specified by the adapter structure.
*
**********************************************************************/
static void
em_reset(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	//device_t	dev = adapter->dev;
	if_t ifp = adapter->ifp;
	struct e1000_hw	*hw = &adapter->hw;
	u16		rx_buffer_size;
	u32		pba;

	INIT_DEBUGOUT("em_reset: begin");

	/* Set up smart power down as default off on newer adapters. */
	//if (!em_smart_pwr_down && (hw->mac.type == e1000_82571 ||
	//	hw->mac.type == e1000_82572)) {
	//	u16 phy_tmp = 0;

	//	/* Speed up time to link by disabling smart power down. */
	//	e1000_read_phy_reg(hw, IGP02E1000_PHY_POWER_MGMT, &phy_tmp);
	//	phy_tmp &= ~IGP02E1000_PM_SPD;
	//	e1000_write_phy_reg(hw, IGP02E1000_PHY_POWER_MGMT, phy_tmp);
	//}

	/*
	* Packet Buffer Allocation (PBA)
	* Writing PBA sets the receive portion of the buffer
	* the remainder is used for the transmit buffer.
	*/
	switch (hw->mac.type) {
		/* Total Packet Buffer on these is 48K */
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		pba = E1000_PBA_32K; /* 32K for Rx, 16K for Tx */
		break;
	case e1000_82573: /* 82573: Total Packet Buffer is 32K */
		pba = E1000_PBA_12K; /* 12K for Rx, 20K for Tx */
		break;
	case e1000_82574:
	case e1000_82583:
		pba = E1000_PBA_20K; /* 20K for Rx, 20K for Tx */
		break;
	case e1000_ich8lan:
		pba = E1000_PBA_8K;
		break;
	case e1000_ich9lan:
	case e1000_ich10lan:
		/* Boost Receive side for jumbo frames */
		if (adapter->hw.mac.max_frame_size > 4096)
			pba = E1000_PBA_14K;
		else
			pba = E1000_PBA_10K;
		break;
	case e1000_pchlan:
	case e1000_pch2lan:
	case e1000_pch_lpt:
	case e1000_pch_spt:
		pba = E1000_PBA_26K;
		break;
	default:
		if (adapter->hw.mac.max_frame_size > 8192)
			pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
		else
			pba = E1000_PBA_48K; /* 48K for Rx, 16K for Tx */
	}
	E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);

	/*
	* These parameters control the automatic generation (Tx) and
	* response (Rx) to Ethernet PAUSE frames.
	* - High water mark should allow for at least two frames to be
	*   received after sending an XOFF.
	* - Low water mark works best when it is very near the high water mark.
	*   This allows the receiver to restart by sending XON when it has
	*   drained a bit. Here we use an arbitrary value of 1500 which will
	*   restart after one full frame is pulled from the buffer. There
	*   could be several smaller frames in the buffer and if so they will
	*   not trigger the XON until their total number reduces the buffer
	*   by 1500.
	* - The pause time is fairly large at 1000 x 512ns = 512 usec.
	*/
	rx_buffer_size = ((E1000_READ_REG(hw, E1000_PBA) & 0xffff) << 10);
	hw->fc.high_water = rx_buffer_size -
		ROUNDUP(adapter->hw.mac.max_frame_size, 1024);
	hw->fc.low_water = hw->fc.high_water - 1500;

	if (adapter->fc) /* locally set flow control value? */
		hw->fc.requested_mode = adapter->fc;
	else
		hw->fc.requested_mode = e1000_fc_none;

	if (hw->mac.type == e1000_80003es2lan)
		hw->fc.pause_time = 0xFFFF;
	else
		hw->fc.pause_time = EM_FC_PAUSE_TIME;

	hw->fc.send_xon = TRUE;

	/* Device specific overrides/settings */
	switch (hw->mac.type) {
	case e1000_pchlan:
		/* Workaround: no TX flow ctrl for PCH */
		hw->fc.requested_mode = e1000_fc_rx_pause;
		hw->fc.pause_time = 0xFFFF; /* override */
		if (pCardInfo->Mtu > ETHERMTU)   
		{
			hw->fc.high_water = 0x3500;
			hw->fc.low_water = 0x1500;
		}
		else {
			hw->fc.high_water = 0x5000;
			hw->fc.low_water = 0x3000;
		}
		hw->fc.refresh_time = 0x1000;
		break;
	case e1000_pch2lan:
	case e1000_pch_lpt:
	case e1000_pch_spt:
		hw->fc.high_water = 0x5C20;
		hw->fc.low_water = 0x5048;
		hw->fc.pause_time = 0x0650;
		hw->fc.refresh_time = 0x0400;
		/* Jumbos need adjusted PBA */
		if (pCardInfo->Mtu > ETHERMTU)  
			E1000_WRITE_REG(hw, E1000_PBA, 12);
		else
			E1000_WRITE_REG(hw, E1000_PBA, 26);
		break;
	case e1000_ich9lan:
	case e1000_ich10lan:
		if (pCardInfo->Mtu > ETHERMTU)  
		{
			hw->fc.high_water = 0x2800;
			hw->fc.low_water = hw->fc.high_water - 8;
			break;
		}
		/* else fall thru */
	default:
		if (hw->mac.type == e1000_80003es2lan)
			hw->fc.pause_time = 0xFFFF;
		break;
	}

	/* I219 needs some special flushing to avoid hangs */
	if (hw->mac.type == e1000_pch_spt)
		em_flush_desc_rings(pCardInfo);

	/* Issue a global reset */
	e1000_reset_hw(hw);
	E1000_WRITE_REG(hw, E1000_WUC, 0);
	em_disable_aspm(pCardInfo);
	/* and a re-init */
	if (e1000_init_hw(hw) < 0) {
		DebugMsg(("Hardware Initialization Failed\n"));
		return;
	}

	E1000_WRITE_REG(hw, E1000_VET, ETHERTYPE_VLAN);
	e1000_get_phy_info(hw);
	e1000_check_for_link(hw);
	return;
}


void
pci_set_command_bit(DRVDEV *pCardInfo, uint16_t bit)
{
	uint16_t	command;

	command = (u16)pci_read_config(pCardInfo, PCIR_COMMAND, 2);
	command |= bit;
	pci_write_config(pCardInfo, PCIR_COMMAND, command, 2);
}

int
pci_enable_busmaster(DRVDEV *pCardInfo)
{
	pci_set_command_bit(pCardInfo, PCIM_CMD_BUSMASTEREN);
	return (0);
}

/*********************************************************************
*
*  Determine hardware revision.
*
**********************************************************************/
static void
em_identify_hardware(DRVDEV *pCardInfo)
{
	//device_t dev = &pCardInfo->pBsdStyleAdapter->dev;
	struct e1000_hw *hw = &pCardInfo->pBsdStyleAdapter->hw;

	/* Make sure our PCI config space has the necessary stuff set */
	pci_enable_busmaster(pCardInfo);
	hw->bus.pci_cmd_word = (u16)pci_read_config(pCardInfo, PCIR_COMMAND, 2);

	/* Save off the information about this board */
	hw->vendor_id = pCardInfo->VendorId; //pci_get_vendor(dev);
	hw->device_id = pCardInfo->DeviceId; //pci_get_device(dev);
	hw->revision_id = (u8)pci_read_config(pCardInfo, PCIR_REVID, 1);
	hw->subsystem_vendor_id = (u16)pci_read_config(pCardInfo, PCIR_SUBVEND_0, 2);
	hw->subsystem_device_id = (u16)pci_read_config(pCardInfo, PCIR_SUBDEV_0, 2);

	/* Do Shared Code Init and Setup */
	if (e1000_set_mac_type(hw)) {
		DebugMsg(("Setup init failure\n"));
		//device_printf(dev, );
		return;
	}
}



static int
em_is_valid_ether_addr(u8 *addr)
{
	char zero_addr[6] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || (!memcmp(addr, zero_addr, ETHER_ADDR_LEN))) {
		return (FALSE);
	}

	return (TRUE);
}

/*
* Bit of a misnomer, what this really means is
* to enable OS management of the system... aka
* to disable special hardware management features
*/
static void
em_init_manageability(struct adapter *adapter)
{
	/* A shared code workaround */
#define E1000_82542_MANC2H E1000_MANC2H
	if (adapter->has_manage) {
		int manc2h = E1000_READ_REG(&adapter->hw, E1000_MANC2H);
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);

		/* enable receiving management packets to the host */
		manc |= E1000_MANC_EN_MNG2HOST;
#define E1000_MNG2HOST_PORT_623 (1 << 5)
#define E1000_MNG2HOST_PORT_664 (1 << 6)
		manc2h |= E1000_MNG2HOST_PORT_623;
		manc2h |= E1000_MNG2HOST_PORT_664;
		E1000_WRITE_REG(&adapter->hw, E1000_MANC2H, manc2h);
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
* Give control back to hardware management
* controller if there is one.
*/
static void
em_release_manageability(struct adapter *adapter)
{
	if (adapter->has_manage) {
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* re-enable hardware interception of ARP */
		manc |= E1000_MANC_ARP_EN;
		manc &= ~E1000_MANC_EN_MNG2HOST;

		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*********************************************************************
*
*  Enable transmit unit.
*
**********************************************************************/
static void 
em_initialize_transmit_unit(DRVDEV *pCardInfo)
{

	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	//struct tx_ring	*txr = adapter->tx_rings;
	struct e1000_hw	*hw = &adapter->hw;
	u32	tctl, txdctl = 0, tarc, tipg = 0;
	int QueNum;

	INIT_DEBUGOUT("em_initialize_transmit_unit: begin");

	//for (int i = 0; i < adapter->num_queues; i++, txr++)
	for (QueNum = 0; QueNum < pCardInfo->numberOfNicQueues; QueNum++)
	{
		//u64 bus_addr = txr->txdma.dma_paddr;
		u64 bus_addr = (u64)pCardInfo->adapterQ[QueNum].TxDescDMA;
		/* Base and Len of TX Ring */
		E1000_WRITE_REG(hw, E1000_TDLEN(QueNum), adapter->num_tx_desc * sizeof(struct e1000_tx_desc));
		E1000_WRITE_REG(hw, E1000_TDBAH(QueNum), (u32)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_TDBAL(QueNum), (u32)bus_addr);
		/* Init the HEAD/TAIL indices */
		E1000_WRITE_REG(hw, E1000_TDT(QueNum), 0);
		E1000_WRITE_REG(hw, E1000_TDH(QueNum), 0);

		HW_DEBUGOUT2("Base = %x, Length = %x\n",
			E1000_READ_REG(&adapter->hw, E1000_TDBAL(QueNum)),
			E1000_READ_REG(&adapter->hw, E1000_TDLEN(QueNum)));

		//txr->busy = EM_TX_IDLE;
		txdctl = 0; /* clear txdctl */
		txdctl |= 0x1f; /* PTHRESH */
		txdctl |= 1 << 8; /* HTHRESH */
		txdctl |= 1 << 16;/* WTHRESH */
		txdctl |= 1 << 22; /* Reserved bit 22 must always be 1 */
		txdctl |= E1000_TXDCTL_GRAN;
		txdctl |= 1 << 25; /* LWTHRESH */

		E1000_WRITE_REG(hw, E1000_TXDCTL(QueNum), txdctl);
	}

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (adapter->hw.mac.type) {
	case e1000_80003es2lan:
		tipg = DEFAULT_82543_TIPG_IPGR1;
		tipg |= DEFAULT_80003ES2LAN_TIPG_IPGR2 <<
			E1000_TIPG_IPGR2_SHIFT;
		break;
	default:
		if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
			(adapter->hw.phy.media_type ==
			e1000_media_type_internal_serdes))
			tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
		else
			tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
		tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	}

	E1000_WRITE_REG(&adapter->hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(&adapter->hw, E1000_TIDV, adapter->tx_int_delay.value);

	if (adapter->hw.mac.type >= e1000_82540)
		E1000_WRITE_REG(&adapter->hw, E1000_TADV,
		adapter->tx_abs_int_delay.value);

	if ((adapter->hw.mac.type == e1000_82571) ||
		(adapter->hw.mac.type == e1000_82572)) {
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= TARC_SPEED_MODE_BIT;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
	}
	else if (adapter->hw.mac.type == e1000_80003es2lan) {
		/* errata: program both queues to unweighted RR */
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= 1;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(1));
		tarc |= 1;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(1), tarc);
	}
	else if (adapter->hw.mac.type == e1000_82574) {
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= TARC_ERRATA_BIT;
		if (adapter->num_queues > 1) {
			tarc |= (TARC_COMPENSATION_MODE | TARC_MQ_FIX);
			E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
			E1000_WRITE_REG(&adapter->hw, E1000_TARC(1), tarc);
		}
		else
			E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
	}

	adapter->txd_cmd = E1000_TXD_CMD_IFCS;
	if (adapter->tx_int_delay.value > 0)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	if (adapter->hw.mac.type >= e1000_82571)
		tctl |= E1000_TCTL_MULR;

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);

	if (hw->mac.type == e1000_pch_spt) {
		u32 reg;
		reg = E1000_READ_REG(hw, E1000_IOSFPC);
		reg |= E1000_RCTL_RDMTS_HEX;
		E1000_WRITE_REG(hw, E1000_IOSFPC, reg);
		reg = E1000_READ_REG(hw, E1000_TARC(0));
		reg |= E1000_TARC0_CB_MULTIQ_3_REQ;
		E1000_WRITE_REG(hw, E1000_TARC(0), reg);
	}
}


/*********************************************************************
*
*  Enable receive unit.
*
**********************************************************************/

static void
em_initialize_receive_unit(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	//struct rx_ring *rxr = adapter->rx_rings;
	if_t ifp = adapter->ifp;
	struct e1000_hw	*hw = &adapter->hw;
	u32	rctl, rxcsum, rfctl;
	int i;

	INIT_DEBUGOUT("em_initialize_receive_units: begin");

	/*
	* Make sure receives are disabled while setting
	* up the descriptor ring
	*/
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	/* Do not disable if ever enabled on this hardware */
	if ((hw->mac.type != e1000_82574) && (hw->mac.type != e1000_82583))
		E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	/* Setup the Receive Control Register */   
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
		E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* Do not store bad packets */
	rctl &= ~E1000_RCTL_SBP;

	/* Enable Long Packet receive */
	if (pCardInfo->Mtu > ETHERMTU)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Strip the CRC */
	//if (!em_disable_crc_stripping)
		rctl |= E1000_RCTL_SECRC;

	//E1000_WRITE_REG(&adapter->hw, E1000_RADV,
	//	adapter->rx_abs_int_delay.value);

	//E1000_WRITE_REG(&adapter->hw, E1000_RDTR,
	//	adapter->rx_int_delay.value);
	/*
	* Set the interrupt throttling rate. Value is calculated
	* as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
	*/
	// E1000_WRITE_REG(hw, E1000_ITR, DEFAULT_ITR);  // RTX_PORT

	/* Use extended rx descriptor formats */
	rfctl = E1000_READ_REG(hw, E1000_RFCTL);
	rfctl |= E1000_RFCTL_EXTEN;
	/*
	** When using MSIX interrupts we need to throttle
	** using the EITR register (82574 only)
	*/
	if (hw->mac.type == e1000_82574) {
		for ( i = 0; i < 4; i++)
			E1000_WRITE_REG(hw, E1000_EITR_82574(i),
			DEFAULT_ITR);
		/* Disable accelerated acknowledge */
		rfctl |= E1000_RFCTL_ACK_DIS;
	}
	E1000_WRITE_REG(hw, E1000_RFCTL, rfctl);

	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
	//if (if_getcapenable(ifp) & IFCAP_RXCSUM) {
	if ( IFCAP_RXCSUM) {
#ifdef EM_MULTIQUEUE
		rxcsum |= E1000_RXCSUM_TUOFL |
			E1000_RXCSUM_IPOFL |
			E1000_RXCSUM_PCSD;
#else
		rxcsum |= E1000_RXCSUM_TUOFL;
#endif
	}
	else
		rxcsum &= ~E1000_RXCSUM_TUOFL;

	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);

#ifndef UNDER_RTSS  // RTX_PORT  RSS is not enabled
#ifdef EM_MULTIQUEUE
#define RSSKEYLEN 10
	if (adapter->num_queues > 1) {
		uint8_t  rss_key[4 * RSSKEYLEN];
		uint32_t reta = 0;
		int i;

		/*
		* Configure RSS key
		*/
		arc4rand(rss_key, sizeof(rss_key), 0);
		for (i = 0; i < RSSKEYLEN; ++i) {
			uint32_t rssrk = 0;

			rssrk = EM_RSSRK_VAL(rss_key, i);
			E1000_WRITE_REG(hw, E1000_RSSRK(i), rssrk);
		}

		/*
		* Configure RSS redirect table in following fashion:
		* (hash & ring_cnt_mask) == rdr_table[(hash & rdr_table_mask)]
		*/
		for (i = 0; i < sizeof(reta); ++i) {
			uint32_t q;

			q = (i % adapter->num_queues) << 7;
			reta |= q << (8 * i);
		}

		for (i = 0; i < 32; ++i) {
			E1000_WRITE_REG(hw, E1000_RETA(i), reta);
		}

		E1000_WRITE_REG(hw, E1000_MRQC, E1000_MRQC_RSS_ENABLE_2Q |
			E1000_MRQC_RSS_FIELD_IPV4_TCP |
			E1000_MRQC_RSS_FIELD_IPV4 |
			E1000_MRQC_RSS_FIELD_IPV6_TCP_EX |
			E1000_MRQC_RSS_FIELD_IPV6_EX |
			E1000_MRQC_RSS_FIELD_IPV6);
	}
#endif
#endif  //UNDER_RTSS
	/*
	** XXX TEMPORARY WORKAROUND: on some systems with 82573
	** long latencies are observed, like Lenovo X60. This
	** change eliminates the problem, but since having positive
	** values in RDTR is a known source of problems on other
	** platforms another solution is being sought.
	*/
	if (hw->mac.type == e1000_82573)
		E1000_WRITE_REG(hw, E1000_RDTR, 0x20);

	for (i = 0; i < adapter->num_queues; i++) {
		/* Setup the Base and Length of the Rx Descriptor Ring */
		//u64 bus_addr = rxr->rxdma.dma_paddr;
		u64 bus_addr = (u64) pCardInfo->adapterQ[i].RxDescDMA;
		u32 rdt = adapter->num_rx_desc - 1; /* default */

		E1000_WRITE_REG(hw, E1000_RDLEN(i),
			adapter->num_rx_desc * sizeof(union e1000_rx_desc_extended));
		E1000_WRITE_REG(hw, E1000_RDBAH(i), (u32)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_RDBAL(i), (u32)bus_addr);
		/* Setup the Head and Tail Descriptor Pointers */
		E1000_WRITE_REG(hw, E1000_RDH(i), 0);
#ifdef DEV_NETMAP
		/*
		* an init() while a netmap client is active must
		* preserve the rx buffers passed to userspace.
		*/
		if (if_getcapenable(ifp) & IFCAP_NETMAP) {
			struct netmap_adapter *na = netmap_getna(adapter->ifp);
			rdt -= nm_kr_rxspace(&na->rx_rings[i]);
		}
#endif /* DEV_NETMAP */
		E1000_WRITE_REG(hw, E1000_RDT(i), rdt);
	}

	/*
	* Set PTHRESH for improved jumbo performance
	* According to 10.2.5.11 of Intel 82574 Datasheet,
	* RXDCTL(1) is written whenever RXDCTL(0) is written.
	* Only write to RXDCTL(1) if there is a need for different
	* settings.
	*/
	if (((adapter->hw.mac.type == e1000_ich9lan) ||
		(adapter->hw.mac.type == e1000_pch2lan) ||
		(adapter->hw.mac.type == e1000_ich10lan)) &&
		(pCardInfo->Mtu > ETHERMTU)) {
		u32 rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		E1000_WRITE_REG(hw, E1000_RXDCTL(0), rxdctl | 3);
	}
	else if (adapter->hw.mac.type == e1000_82574) {
		for (i = 0; i < adapter->num_queues; i++) {
			u32 rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));

			rxdctl |= 0x20; /* PTHRESH */
			rxdctl |= 4 << 8; /* HTHRESH */
			rxdctl |= 4 << 16;/* WTHRESH */
			rxdctl |= 1 << 24; /* Switch to granularity */
			E1000_WRITE_REG(hw, E1000_RXDCTL(i), rxdctl);
		}
	}

	if (adapter->hw.mac.type >= e1000_pch2lan) {
		if (pCardInfo->Mtu > ETHERMTU)
			e1000_lv_jumbo_workaround_ich8lan(hw, TRUE);
		else
			e1000_lv_jumbo_workaround_ich8lan(hw, FALSE);
	}

	/* Make sure VLAN Filters are off */
	rctl &= ~E1000_RCTL_VFE;

	rctl &= ~E1000_RCTL_SZ_256; // clear BSIZE bits
	rctl &= ~E1000_RCTL_LPE; // clear long packet bit

	if (adapter->rx_mbuf_sz <= MCLBYTES)
		rctl |= E1000_RCTL_SZ_2048 | E1000_RCTL_LPE;
	else if (adapter->rx_mbuf_sz <= MJUMPAGESIZE)
		rctl |= E1000_RCTL_SZ_4096 | E1000_RCTL_BSEX | E1000_RCTL_LPE;
	else if (adapter->rx_mbuf_sz <= MJUM2PAGESIZE)
		rctl |= E1000_RCTL_SZ_8192 | E1000_RCTL_BSEX | E1000_RCTL_LPE;
	else if (adapter->rx_mbuf_sz <= MJUM9BYTES)
		rctl |= E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX | E1000_RCTL_LPE;
	else if (adapter->rx_mbuf_sz <= MJUM16BYTES)
		rctl |= E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX | E1000_RCTL_LPE;


	/* ensure we clear use DTYPE of 00 here */
	rctl &= ~0x00000C00;
	/* Write out the settings */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	return;
}

/*********************************************************************
*  Init entry point
*
*  This routine is used in two ways. It is used by the stack as
*  init entry point in network interface structure. It is also used
*  by the driver as a hw/sw initialization routine to get to a
*  consistent state.
*
*  return 0 on success, positive on failure
**********************************************************************/

static void
em_init_locked(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;
	//if_t ifp = adapter->ifp;
	//device_t	dev = adapter->dev;

	INIT_DEBUGOUT("em_init: begin");

	//EM_CORE_LOCK_ASSERT(adapter);   RTX_PORT

	em_disable_intr(adapter);
	//callout_stop(&adapter->timer);

	/* Get the latest mac address, User can use a LAA */
	//memcpy(if_getlladdr(adapter->ifp), adapter->hw.mac.addr,	ETHER_ADDR_LEN);

	/* Put the address into the Receive Address Array */
	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);  

	/*
	* With the 82571 adapter, RAR[0] may be overwritten
	* when the other port is reset, we make a duplicate
	* in RAR[14] for that eventuality, this assures
	* the interface continues to function.
	*/
	if (adapter->hw.mac.type == e1000_82571) {
		e1000_set_laa_state_82571(&adapter->hw, TRUE);
		e1000_rar_set(&adapter->hw, adapter->hw.mac.addr,
			E1000_RAR_ENTRIES - 1);
	}



	/* Initialize the hardware */
	em_reset(pCardInfo);
	em_update_link_status(pCardInfo);

	/* Setup VLAN support, basic and offload if available */
	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERTYPE_VLAN);

	/* Set hardware offload abilities */
	//if_clearhwassist(ifp);
	//if (if_getcapenable(ifp) & IFCAP_TXCSUM)
	//	if_sethwassistbits(ifp, CSUM_TCP | CSUM_UDP, 0);
	/*
	** There have proven to be problems with TSO when not
	** at full gigabit speed, so disable the assist automatically
	** when at lower speeds.  -jfv
	*/
	//if (if_getcapenable(ifp) & IFCAP_TSO4) {
	//	if (adapter->link_speed == SPEED_1000)
	//		if_sethwassistbits(ifp, CSUM_TSO, 0);
	//}



	/* Configure for OS presence */
	em_init_manageability(adapter);

	/* Prepare transmit descriptors and buffers */
	//em_setup_transmit_structures(adapter);
	em_initialize_transmit_unit(pCardInfo);

	/* Setup Multicast table */
	//em_set_multi(adapter);  // RTX_PORT  eventually setup real multicast

	/*
	** Figure out the desired mbuf
	** pool for doing jumbos
	*/
	if (adapter->hw.mac.max_frame_size <= MCLBYTES)
		adapter->rx_mbuf_sz = MCLBYTES;
	else if (adapter->hw.mac.max_frame_size <= MJUMPAGESIZE)
		adapter->rx_mbuf_sz = MJUMPAGESIZE;
	else if (adapter->hw.mac.max_frame_size <= MJUM2PAGESIZE)
		adapter->rx_mbuf_sz = MJUM2PAGESIZE;
	else if (adapter->hw.mac.max_frame_size <= MJUM9BYTES)
		adapter->rx_mbuf_sz = MJUM9BYTES;
	else if (adapter->hw.mac.max_frame_size <= MJUM16BYTES)
		adapter->rx_mbuf_sz = MJUM16BYTES;


	/* Prepare receive descriptors and buffers */
	//if (em_setup_receive_structures(adapter)) {
	//	DebugMsg(("Could not setup receive structures\n"));
	//	em_stop(adapter);
	//	return;
	//}
	em_initialize_receive_unit(pCardInfo);

	/* Use real VLAN Filter support? */
	//if (if_getcapenable(ifp) & IFCAP_VLAN_HWTAGGING) {
	//	if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
	//		/* Use real VLAN Filter support */
	//		em_setup_vlan_hw_support(adapter);
	//	else {
	//		u32 ctrl;
	//		ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
	//		ctrl |= E1000_CTRL_VME;
	//		E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
	//	}
	//}

	/* Don't lose promiscuous settings */
	//em_set_promisc(adapter);

	/* Set the interface as ACTIVE */
	//if_setdrvflagbits(ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);

	//callout_reset(&adapter->timer, hz, em_local_timer, adapter);
	e1000_clear_hw_cntrs_base_generic(&adapter->hw);

	/* MSI/X configuration for 82574 */
	if (adapter->hw.mac.type == e1000_82574) {
		int tmp;
		tmp = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
		tmp |= E1000_CTRL_EXT_PBA_CLR;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT, tmp);
		/* Set the IVAR - interrupt vector routing. */
		E1000_WRITE_REG(&adapter->hw, E1000_IVAR, adapter->ivars);
	}

#ifdef DEVICE_POLLING
	/*
	* Only enable interrupts if we are not polling, make sure
	* they are off otherwise.
	*/
	if (if_getcapenable(ifp) & IFCAP_POLLING)
		em_disable_intr(adapter);
	else
#endif /* DEVICE_POLLING */
		em_enable_intr(adapter);

	/* AMT based hardware can now take control from firmware */
	if (adapter->has_manage && adapter->has_amt)
		em_get_hw_control(adapter);
}


static void
em_init(DRVDEV *pCardInfo)
{
	struct adapter *adapter = pCardInfo->pBsdStyleAdapter;

	
	//EM_CORE_LOCK(adapter);  // RTX_PORT
	em_init_locked(pCardInfo);
	//EM_CORE_UNLOCK(adapter);  RTX_PORT
}



/*********************************************************************
*  Device initialization routine
*
*  The attach entry point is called when the driver is being loaded.
*  This routine identifies the type of hardware, allocates all resources
*  and initializes the hardware.
*
*  return 0 on success, positive on failure
*********************************************************************/

int em_attach(DRVDEV *pCardInfo)
{
	struct adapter	*adapter;
	struct e1000_hw	*hw;
	int		error = 0;
	LARGE_INTEGER BusAddr, TranslatedAddr;
	ULONG AddressSpace;



	INIT_DEBUGOUT("em_attach: begin");


	adapter = malloc(sizeof(struct adapter));

	if (!adapter)
	{
		return EIO;
	}

	
	pCardInfo->pBsdStyleAdapter = adapter;
	hw = &adapter->hw;
	
	adapter->hw.back = &adapter->osdep;

	//adapter->osdep.mem_bus_space_handle = (bus_space_handle_t) pCardInfo->CsrBase;
	//hw->hw_addr = pCardInfo->CsrBase;

	adapter->osdep.mem_bus_space_handle = (bus_space_handle_t)vMemAddr[0];
	hw->hw_addr = vMemAddr[0];

	adapter->num_queues = pCardInfo->numberOfNicQueues;

	hw->fc.requested_mode = e1000_fc_none;
	hw->fc.current_mode = e1000_fc_none;
	adapter->fc = e1000_fc_none;

	((struct e1000_osdep *)hw->back)->dev = pCardInfo;

	pCardInfo->pBsdStyleAdapter->hw.vendor_id = pCardInfo->VendorId; //pci_get_vendor(dev);
	pCardInfo->pBsdStyleAdapter->hw.device_id = pCardInfo->DeviceId; //pci_get_device(dev);

#ifndef UNDER_RTSS  // RTX_PORT
	if (resource_disabled("em", device_get_unit(dev))) {
		device_printf(dev, "Disabled by device hint\n");
		return (ENXIO);
	}

	adapter = device_get_softc(dev);
	adapter->dev = adapter->osdep.dev = dev;
	hw = &adapter->hw;
	EM_CORE_LOCK_INIT(adapter, device_get_nameunit(dev));

	/* SYSCTL stuff */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
		SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
		OID_AUTO, "nvm", CTLTYPE_INT | CTLFLAG_RW, adapter, 0,
		em_sysctl_nvm_info, "I", "NVM Information");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
		SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
		OID_AUTO, "debug", CTLTYPE_INT | CTLFLAG_RW, adapter, 0,
		em_sysctl_debug_info, "I", "Debug Information");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
		SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
		OID_AUTO, "fc", CTLTYPE_INT | CTLFLAG_RW, adapter, 0,
		em_set_flowcntl, "I", "Flow Control");

	callout_init_mtx(&adapter->timer, &adapter->core_mtx, 0);

#endif UNDER_RTSS  // RTX_PORT



	/* Determine hardware and mac info */
	em_identify_hardware(pCardInfo);




	///* Setup PCI resources */
	//if (em_allocate_pci_resources(adapter)) {
	//	DebugMsg(("Allocation of PCI resources failed\n"));
	//	error = ENXIO;
	//	goto err_pci;
	//}

	/*
	** For ICH8 and family we need to
	** map the flash memory, and this
	** must happen after the MAC is
	** identified
	*/
	if ((hw->mac.type == e1000_ich8lan) ||
		(hw->mac.type == e1000_ich9lan) ||
		(hw->mac.type == e1000_ich10lan) ||
		(hw->mac.type == e1000_pchlan) ||
		(hw->mac.type == e1000_pch2lan) ||
		(hw->mac.type == e1000_pch_lpt))
	{
		int rid = EM_BAR_TYPE_FLASH;
		int nvm_size;

		//adapter->flash = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);

		BusAddr.QuadPart = 0;
		RtGetBusDataByOffset(PCIConfiguration,
			pCardInfo->BusNumber,
			pCardInfo->SlotNumber.u.AsULONG,
			(PVOID)&(BusAddr.QuadPart),
			EM_BAR_TYPE_FLASH,
			4
			);

		if (!BusAddr.QuadPart)
		{
			DebugMsg(("Mapping of Flash failed\n"));
			error = ENXIO;
			goto err_pci;
		}
		AddressSpace = 0;
		if (!RtTranslateBusAddress(PCIBus, pCardInfo->BusNumber, BusAddr, &AddressSpace, &TranslatedAddr))
		{
			DebugMsg(("CheckForNetworkCard, error translating bus address\n"));
			error = ENXIO;
			goto err_pci;
		}

		nvm_size =
			(((E1000_READ_REG(hw, E1000_STRAP) >> 1) & 0x1F) + 1)
			* NVM_SIZE_MULTIPLIER;
		nvm_size = nvm_size / 2;
		/* Adjust to word count */
		nvm_size /= sizeof(u16);
		//
		// Map the addresses to virtual addresses the software can use
		//
		adapter->flash = (struct resource *)RtMapMemory(TranslatedAddr,    // Base of the physical address range to map
			nvm_size,   // The length of address range in bytes
			MmNonCached);   // Whether or not to use cache


		/* This is used in the shared code */
		hw->flash_address = (u8 *)adapter->flash;
		adapter->osdep.flash_bus_space_tag = (bus_space_tag_t)NULL;  // rman_get_bustag(adapter->flash);
		adapter->osdep.flash_bus_space_handle = (bus_space_handle_t) adapter->flash;  // rman_get_bushandle(adapter->flash);
	}
	/*
	** In the new SPT device flash is not  a
	** separate BAR, rather it is also in BAR0,
	** so use the same tag and an offset handle for the
	** FLASH read/write macros in the shared code.
	*/
	else if (hw->mac.type == e1000_pch_spt) {
		adapter->osdep.flash_bus_space_tag =
			adapter->osdep.mem_bus_space_tag;
		adapter->osdep.flash_bus_space_handle =
			adapter->osdep.mem_bus_space_handle
			+ E1000_FLASH_BASE_ADDR;
	}

	/* Do Shared Code initialization */
	error = e1000_setup_init_funcs(hw, TRUE);
	if (error) {
		DebugMsg(("Setup of Shared code failed, error %d\n", error));
		error = ENXIO;
		goto err_pci;
	}

	/*
	* Setup MSI/X or MSI if PCI Express
	*/
	//adapter->msix = em_setup_msix(adapter);

	e1000_get_bus_info(hw);

#ifndef UNDER_RTSS  // RTX_PORT
	/* Set up some sysctls for the tunable interrupt delays */
	em_add_int_delay_sysctl(adapter, "rx_int_delay",
		"receive interrupt delay in usecs", &adapter->rx_int_delay,
		E1000_REGISTER(hw, E1000_RDTR), em_rx_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "tx_int_delay",
		"transmit interrupt delay in usecs", &adapter->tx_int_delay,
		E1000_REGISTER(hw, E1000_TIDV), em_tx_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "rx_abs_int_delay",
		"receive interrupt delay limit in usecs",
		&adapter->rx_abs_int_delay,
		E1000_REGISTER(hw, E1000_RADV),
		em_rx_abs_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "tx_abs_int_delay",
		"transmit interrupt delay limit in usecs",
		&adapter->tx_abs_int_delay,
		E1000_REGISTER(hw, E1000_TADV),
		em_tx_abs_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "itr",
		"interrupt delay limit in usecs/4",
		&adapter->tx_itr,
		E1000_REGISTER(hw, E1000_ITR),
		DEFAULT_ITR);

	/* Sysctl for limiting the amount of work done in the taskqueue */
	em_set_sysctl_value(adapter, "rx_processing_limit",
		"max number of rx packets to process", &adapter->rx_process_limit,
		em_rx_process_limit);

	/*
	* Validate number of transmit and receive descriptors. It
	* must not exceed hardware maximum, and must be multiple
	* of E1000_DBA_ALIGN.
	*/
	if (((em_txd * sizeof(struct e1000_tx_desc)) % EM_DBA_ALIGN) != 0 ||
		(em_txd > EM_MAX_TXD) || (em_txd < EM_MIN_TXD)) {
		device_printf(dev, "Using %d TX descriptors instead of %d!\n",
			EM_DEFAULT_TXD, em_txd);
		adapter->num_tx_desc = EM_DEFAULT_TXD;
	}
	else
		adapter->num_tx_desc = em_txd;

	if (((em_rxd * sizeof(union e1000_rx_desc_extended)) % EM_DBA_ALIGN) != 0 ||
		(em_rxd > EM_MAX_RXD) || (em_rxd < EM_MIN_RXD)) {
		device_printf(dev, "Using %d RX descriptors instead of %d!\n",
			EM_DEFAULT_RXD, em_rxd);
		adapter->num_rx_desc = EM_DEFAULT_RXD;
	}
	else
		adapter->num_rx_desc = em_rxd;
#endif UNDER_RTSS  // RTX_PORT

	hw->mac.autoneg = DO_AUTO_NEG;
	hw->phy.autoneg_wait_to_complete = FALSE;
	hw->phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = FALSE;
		hw->phy.ms_type = EM_MASTER_SLAVE;
	}

	/*
	* Set the frame limits assuming
	* standard ethernet sized frames.
	*/
	//adapter->hw.mac.max_frame_size =
	//	ETHERMTU + ETHER_HDR_LEN + ETHERNET_FCS_SIZE;
	//	RTX_PORT
	//	
	adapter->hw.mac.max_frame_size = pCardInfo->MaxPacket;


	/*
	* This controls when hardware reports transmit completion
	* status.
	*/
	hw->mac.report_tx_early = 1;

	/*
	** Get queue/ring memory
	*/
	//if (em_allocate_queues(adapter)) {
	//	error = ENOMEM;
	//	goto err_pci;
	//}

	/* Allocate multicast array memory. */
	adapter->mta = malloc(sizeof(u8) * ETH_ADDR_LEN * MAX_NUM_MULTICAST_ADDRESSES);
	if (adapter->mta == NULL)
	{
		DebugMsg(("Can not allocate multicast setup array\n"));
		error = ENOMEM;
		goto err_late;
	}

	/* Check SOL/IDER usage */
	if (e1000_check_reset_block(hw))
		DebugMsg(("PHY reset is blocked due to SOL/IDER session.\n"));

	/* Sysctl for setting Energy Efficient Ethernet */
	hw->dev_spec.ich8lan.eee_disable = TRUE;  /// eee_setting;
	//SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	//	SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	//	OID_AUTO, "eee_control", CTLTYPE_INT | CTLFLAG_RW,
	//	adapter, 0, em_sysctl_eee, "I",
	//	"Disable Energy Efficient Ethernet");


	INIT_DEBUGOUT("em_attach: end");

	return (0);

err_late:
	//em_free_transmit_structures(adapter);
	//em_free_receive_structures(adapter);
	em_release_hw_control(adapter);
	//if (adapter->ifp != (void *)NULL)
	//	if_free(adapter->ifp);
err_pci:
	//em_free_pci_resources(adapter);
	free(adapter->mta);
	//EM_CORE_LOCK_DESTROY(adapter);

	return (error);
}






