/*
 * Copyright (C) 2012 Texas Instruments
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/* Part of the code is taken from
   chromiumos/third_party/u-boot/drivers/usb/host/xhci-exynos5.c
   which supports the same DW3 usb core
 */

#include <common.h>
#include <watchdog.h>
#include <usb.h>
#include <asm/arch/psc_defs.h>
#include <asm/io.h>
#include <linux/usb/dwc3.h>
#include <linux/list.h>
#include "xhci.h"

#define KEYSTONE2_USB_SS_BASE			0x02680000
#define KEYSTONE2_USB_HOST_XHCI_BASE		(KEYSTONE2_USB_SS_BASE + 0x10000)
#define KEYSTONE2_DEV_USB_PHY_BASE		0x02620738
#define KEYSTONE2_USB_PHY_CFG_BASE		0x02630000

#define BIT(x)					(1 << x)

#define K_USB3_PHY_SSC_EN			BIT(31)
#define K_USB3_PHY_REF_USE_PAD			BIT(30)
#define K_USB3_PHY_REF_SSP_EN			BIT(29)
#define K_USB3_PHY_FSEL				0x27
#define K_USB3_PHY_FSEL_SHIFT			22
#define K_USB3_PHY_RETENABLEEN			BIT(21)
#define K_USB3_PHY_REFCLKSEL			0x2
#define K_USB3_PHY_REFCLKSEL_SHIFT		19
#define K_USB3_PHY_OTGDISABLE			BIT(15)
#define K_USB3_PHY_OTG_VBUSVLDECTSEL		BIT(16)


#define K_USB3_PHY_SSC_REF_CLK_SEL		0
#define K_USB3_PHY_SSC_REF_CLK_SEL_SHIFT	4
#define K_USB3_PHY_MPLL_MULTIPLIER		0x19
#define K_USB3_PHY_MPLL_MULTIPLIER_SHIFT	13
#define K_USB3_PHY_REF_CLKDIV2			0

#define K_USB3_PHY_CLOCK_DEFAULT \
		(K_USB3_PHY_SSC_EN |\
		 K_USB3_PHY_REF_USE_PAD |\
		 K_USB3_PHY_REF_SSP_EN |\
		 K_USB3_PHY_FSEL << K_USB3_PHY_FSEL_SHIFT |\
		 K_USB3_PHY_RETENABLEEN |\
		 K_USB3_PHY_REFCLKSEL << K_USB3_PHY_REFCLKSEL_SHIFT |\
		 K_USB3_PHY_OTGDISABLE)

#define K_USB3_PHY_PLL_DEFAULT	\
	(K_USB3_PHY_SSC_REF_CLK_SEL |\
	 K_USB3_PHY_MPLL_MULTIPLIER << K_USB3_PHY_MPLL_MULTIPLIER_SHIFT)

/* KEYSTONE2 XHCI PHY register structure */
struct xhci_phy {
	unsigned int phy_utmi;		/* ctl0 */
	unsigned int phy_pipe;		/* ctl1 */
	unsigned int phy_param_ctrl_1;	/* ctl2 */
	unsigned int phy_param_ctrl_2;	/* ctl3 */
	unsigned int phy_clock;		/* ctl4 */
	unsigned int phy_pll;		/* ctl5 */
};

struct kdwc3_irq_regs {
	u32		revision;	/* 0x000 */
	u32		_rsvd0[3];
	u32		sysconfig;	/* 0x010 */
	u32		_rsvd1[1];
	u32		irq_eoi;
	u32		_rsvd2[1];
	struct {
		u32	raw_status;
		u32	status;
		u32	enable_set;
		u32	enable_clr;
	} irqs[16];
};

struct kxhci_ctrl {
	struct kdwc3_irq_regs *usbss;
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	struct xhci_phy *phy;
	unsigned int *hcd;
};

struct kxhci_ctrl kxhci;

static void keystone_xhci_phy_set(struct xhci_phy *phy)
{
	u32 val;

	/* VBUSVLDEXTSEL has a default value of 1 in BootCfg but shouldn't.
	 * It should always be cleared because our USB PHY has an onchip VBUS
	 * analog comparator.
	 */
	val = readl(&phy->phy_clock);
	/* quit selecting the vbusvldextsel by default! */
	val &= ~K_USB3_PHY_OTG_VBUSVLDECTSEL;
	writel(val, &phy->phy_clock);

}

static void keystone_xhci_phy_unset(struct xhci_phy *phy)
{
	u32 val;

	/* Disable the PHY REFCLK clock gate */
	val = readl(&phy->phy_clock);
	val &= ~K_USB3_PHY_REF_SSP_EN;
	writel(val, &phy->phy_clock);
}

void dwc3_set_mode(unsigned int *hcd, u32 mode)
{
	clrsetbits_le32(((char *)hcd + DWC3_GCTL),
			DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG),
			DWC3_GCTL_PRTCAPDIR(mode));
}

static void dwc3_core_soft_reset(unsigned int *hcd)
{
	/* Before Resetting PHY, put Core in Reset */
	setbits_le32(((char *)hcd + DWC3_GCTL),
				DWC3_GCTL_CORESOFTRESET);
	/* Assert USB3 PHY reset */
	setbits_le32(((char *)hcd + DWC3_GUSB3PIPECTL(0)),
				DWC3_GUSB3PIPECTL_PHYSOFTRST);
	/* Assert USB2 PHY reset */
	setbits_le32(((char *)hcd + DWC3_GUSB2PHYCFG(0)),
				DWC3_GUSB2PHYCFG_PHYSOFTRST);

	mdelay(100);

	/* Clear USB3 PHY reset */
	clrbits_le32(((char *)hcd + DWC3_GUSB3PIPECTL(0)),
				DWC3_GUSB3PIPECTL_PHYSOFTRST);
	/* Clear USB2 PHY reset */
	clrbits_le32(((char *)hcd + DWC3_GUSB2PHYCFG(0)),
				DWC3_GUSB2PHYCFG_PHYSOFTRST);
	/* After PHYs are stable we can take Core out of reset state */
	clrbits_le32(((char *)hcd + DWC3_GCTL),
				DWC3_GCTL_CORESOFTRESET);
}

static int dwc3_core_init(unsigned int *hcd)
{
	struct dwc3_hwparams parms;
	unsigned long t_rst;
	u32 revision, val;
	int ret = 0;

	revision = dwc3_readl(hcd, DWC3_GSNPSID);
	/* This should read as U3 followed by revision number */
	if ((revision & DWC3_GSNPSID_MASK) != 0x55330000) {
		printf("this is not a DesignWare USB3 DRD Core: "
			"hcd=%08x, rev=%08x\n", (u32)hcd, revision);
		return -1;
	}

	/* issue device SoftReset too */
	dwc3_writel(hcd, DWC3_DCTL, DWC3_DCTL_CSFTRST);

	t_rst = get_timer(0);
	do {
		val = dwc3_readl(hcd, DWC3_DCTL);
		if (!(val & DWC3_DCTL_CSFTRST))
			break;
		WATCHDOG_RESET();
	} while (get_timer(t_rst) < 500);

	if (val & DWC3_DCTL_CSFTRST) {
		debug("Reset timed out\n");
		return -2;
	}

	dwc3_core_soft_reset(hcd);

	parms.hwparams1 = dwc3_readl(hcd, DWC3_GHWPARAMS1);

	val = dwc3_readl(hcd, DWC3_GCTL);
	val &= ~DWC3_GCTL_SCALEDOWN_MASK;
	val &= ~DWC3_GCTL_DISSCRAMBLE;
	switch (DWC3_GHWPARAMS1_EN_PWROPT(parms.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		val &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	default:
		printf("No power optimization available\n");
	}

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */

	if ((revision & DWC3_REVISION_MASK) < 0x190a)
		val |= DWC3_GCTL_U2RSTECN;

	dwc3_writel(hcd, DWC3_GCTL, val);
	return ret;
}

int xhci_hcd_init(int index, struct xhci_hccr **ret_hccr,
			struct xhci_hcor **ret_hcor)
{
	struct kdwc3_irq_regs *usbss;
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	struct xhci_phy *phy;
	unsigned int *hcd;
	int ret;
	u32 val;

	usbss = (struct kdwc3_irq_regs *)KEYSTONE2_USB_SS_BASE;
	phy = (struct xhci_phy *)KEYSTONE2_DEV_USB_PHY_BASE;

	/* Enable the PHY REFCLK clock gate with phy_ref_ssp_en = 1 */
	val = readl(&(phy->phy_clock));
	val |= K_USB3_PHY_REF_SSP_EN;
	writel(val, &phy->phy_clock);

	mdelay(100);

	/* Release USB from reset */
	ret = psc_enable_module(TCI6638_LPSC_USB);

	mdelay(100);

	/* Initialize usb phy */
	keystone_xhci_phy_set(phy);

	/* soft reset usbss */
	writel(1, &usbss->sysconfig);
	while (readl(&usbss->sysconfig) & 1)
		;

	val = readl(&usbss->revision);
	debug("usbss revision %x\n", val);

	/* Initialize usb core */
	hcd = (unsigned int *)KEYSTONE2_USB_HOST_XHCI_BASE;

	ret = dwc3_core_init(hcd);
	if (ret) {
		debug("failed to initialize core\n");
		return -1;
	}

	dwc3_set_mode(hcd, DWC3_GCTL_PRTCAP_HOST);

	/* set register addresses */
	hccr = (struct xhci_hccr *)KEYSTONE2_USB_HOST_XHCI_BASE;
	hcor = (struct xhci_hcor *)((uint32_t) hccr
				+ HC_LENGTH(readl(&hccr->cr_capbase)));

	debug("Keystone2-xhci: init hccr %08x and hcor %08x hc_length %d\n",
		(u32)hccr, (u32)hcor,
		(u32)HC_LENGTH(xhci_readl(&hccr->cr_capbase)));

	kxhci.usbss = usbss;
	kxhci.hccr = hccr;
	kxhci.hcor = hcor;
	kxhci.phy = phy;
	kxhci.hcd = hcd;

	*ret_hccr = hccr;
	*ret_hcor = hcor;

	return 0;
}

int keystone_xhci_phy_suspend(void)
{
	struct xhci_hcor *hcor;
	uint32_t *portsc_1 = NULL;
	uint32_t *portsc_2 = NULL;
	unsigned int *hcd;
	u32 val, usb2_pls, usb3_pls, event_q;
	int loop_cnt = 0;

	/* set register addresses */
	hcor = kxhci.hcor;
	hcd = kxhci.hcd;

	/* Bypass Scrambling and Set Shorter Training sequence for simulation */
	val = DWC3_GCTL_PWRDNSCALE(0x4b0) | DWC3_GCTL_PRTCAPDIR(0x2);
	dwc3_writel(hcd, DWC3_GCTL, val);

	/* GUSB2PHYCFG */
	val = dwc3_readl(hcd, DWC3_GUSB2PHYCFG(0));
	/* assert bit 6 (SusPhy) */
	val |= DWC3_GUSB2PHYCFG_SUSPHY;
	dwc3_writel(hcd, DWC3_GUSB2PHYCFG(0), val);

	/* GUSB3PIPECTL */
	val = dwc3_readl(hcd, DWC3_GUSB3PIPECTL(0));
	/* assert bit 29 to allow PHY to go to suspend when idle
	 * and cause the USB3 SS PHY to enter suspend mode
	 */
	val |= (BIT(29) | DWC3_GUSB3PIPECTL_SUSPHY);
	dwc3_writel(hcd, DWC3_GUSB3PIPECTL(0), val);

	/* Steps necessary to allow controller to suspend even when
	 * VBUS is HIGH:
	 * - Init DCFG[2:0] (DevSpd) to: 1=FS
	 * - Init GEVNTADR0 to point to an eventQ
	 * - Init GEVNTSIZ0 to 0x0100 to specify the size of the eventQ
	 * - Init DCTL::Run_nStop = 1
	 */
	dwc3_writel(hcd, DWC3_DCFG, 0x00020001);
	/* TODO: local2global( (Uint32) eventQ )? */
	dwc3_writel(hcd, DWC3_GEVNTADRLO(0), (u32)&event_q);
	dwc3_writel(hcd, DWC3_GEVNTADRHI(0), 0);
	dwc3_writel(hcd, DWC3_GEVNTSIZ(0), 0x4);
	/* Run */
	dwc3_writel(hcd, DWC3_DCTL, DWC3_DCTL_RUN_STOP);

	mdelay(100);

	/* Wait for USB2 & USB3 PORTSC::PortLinkState to indicate suspend */
	portsc_1 = (uint32_t *) (&hcor->PortRegs[0].or_portsc);
	portsc_2 = (uint32_t *) (&hcor->PortRegs[1].or_portsc);
	usb2_pls = 0;
	usb3_pls = 0;
	do {
		++loop_cnt;
		usb2_pls = (readl(portsc_1) & PORT_PLS_MASK) >> 5;
		usb3_pls = (readl(portsc_2) & PORT_PLS_MASK) >> 5;
	} while (((usb2_pls != 0x4) || (usb3_pls != 0x4)) && loop_cnt < 1000);

	if (usb2_pls != 0x4 || usb3_pls != 0x4) {
		debug("USB suspend failed - PLS USB2=%02x, USB3=%02x\n",
			usb2_pls, usb3_pls);
		return -1;
	}

	debug("USB2 and USB3 PLS - Disabled, loop_cnt=%d\n", loop_cnt);
	return 0;
}

void xhci_hcd_stop(int index)
{
	/* Disable USB */
	if (keystone_xhci_phy_suspend())
		return;

	if (psc_disable_module(TCI6638_LPSC_USB)) {
		debug("PSC disable module USB failed!\n");
		return;
	}

	/* Disable PHY */
	keystone_xhci_phy_unset(kxhci.phy);

	memset(&kxhci, 0, sizeof(struct kxhci_ctrl));

	debug("xhci_hcd_stop OK.\n");
}
