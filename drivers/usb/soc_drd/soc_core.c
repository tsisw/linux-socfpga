// SPDX-License-Identifier: GPL-2.0
/*
 * soc_core.c - SOC USB3 DRD Controller Core file
 *
 */

#include <linux/clk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset.h>
#include <linux/bitfield.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#include "soc_core.h"
#include "soc_gadget.h"
#include "soc_io.h"

#include "soc_debug.h"

#define SOC_USB_DEFAULT_AUTOSUSPEND_DELAY	5000 /* ms */

/**
 * soc_usb_get_dr_mode - Validates and sets dr_mode
 * @su: pointer to our context structure
 */
static int soc_usb_get_dr_mode(struct soc_usb *su)
{
	enum usb_dr_mode mode;
	struct device *dev = su->dev;
	unsigned int hw_mode;

	if (su->dr_mode == USB_DR_MODE_UNKNOWN)
		su->dr_mode = USB_DR_MODE_OTG;

	mode = su->dr_mode;
	hw_mode = SOC_USB_GHWPARAMS0_MODE(su->hwparams.hwparams0);

	switch (hw_mode) {
	case SOC_USB_GHWPARAMS0_MODE_GADGET:
		if (IS_ENABLED(CONFIG_USB_SOC_DRD_HOST)) {
			dev_err(dev,
				"Controller does not support host mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_PERIPHERAL;
		break;
	case SOC_USB_GHWPARAMS0_MODE_HOST:
		if (IS_ENABLED(CONFIG_USB_SOC_DRD_GADGET)) {
			dev_err(dev,
				"Controller does not support device mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_HOST;
		break;
	default:
		if (IS_ENABLED(CONFIG_USB_SOC_DRD_HOST))
			mode = USB_DR_MODE_HOST;
		else if (IS_ENABLED(CONFIG_USB_SOC_DRD_GADGET))
			mode = USB_DR_MODE_PERIPHERAL;
	}

	if (mode != su->dr_mode) {
		dev_warn(dev,
			 "Configuration mismatch. dr_mode forced to %s\n",
			 mode == USB_DR_MODE_HOST ? "host" : "gadget");

		su->dr_mode = mode;
	}

	return 0;
}

void soc_usb_enable_susphy(struct soc_usb *su, bool enable)
{
	u32 reg;

	reg = soc_usb_readl(su->regs, SOC_USB_GUSB3PIPECTL(0));
	if (enable && !su->dis_u3_susphy_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_SUSPHY;
	else
		reg &= ~SOC_USB_GUSB3PIPECTL_SUSPHY;

	soc_usb_writel(su->regs, SOC_USB_GUSB3PIPECTL(0), reg);

	reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));
	if (enable && !su->dis_u2_susphy_quirk)
		reg |= SOC_USB_GUSB2PHYCFG_SUSPHY;
	else
		reg &= ~SOC_USB_GUSB2PHYCFG_SUSPHY;

	soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);
}

void soc_usb_set_prtcap(struct soc_usb *su, u32 mode)
{
	u32 reg;

	reg = soc_usb_readl(su->regs, SOC_USB_GCTL);
	reg &= ~(SOC_USB_GCTL_PRTCAPDIR(SOC_USB_GCTL_PRTCAP_OTG));
	reg |= SOC_USB_GCTL_PRTCAPDIR(mode);
	soc_usb_writel(su->regs, SOC_USB_GCTL, reg);

	su->current_dr_role = mode;
}

static void __soc_usb_set_mode(struct work_struct *work)
{
	struct soc_usb *su = work_to_su(work);
	unsigned long flags;
	int ret;
	u32 reg;
	u32 desired_dr_role;

	mutex_lock(&su->mutex);
	spin_lock_irqsave(&su->lock, flags);
	desired_dr_role = su->desired_dr_role;
	spin_unlock_irqrestore(&su->lock, flags);

	pm_runtime_get_sync(su->dev);

	if (su->current_dr_role == SOC_USB_GCTL_PRTCAP_OTG)
		soc_usb_otg_update(su, 0);

	if (!desired_dr_role)
		goto out;

	if (desired_dr_role == su->current_dr_role)
		goto out;

	if (desired_dr_role == SOC_USB_GCTL_PRTCAP_OTG && su->edev)
		goto out;

	switch (su->current_dr_role) {
	case SOC_USB_GCTL_PRTCAP_HOST:
		soc_usb_host_exit(su);
		break;
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		soc_usb_gadget_exit(su);
		soc_usb_event_buffers_cleanup(su);
		break;
	case SOC_USB_GCTL_PRTCAP_OTG:
		soc_usb_otg_exit(su);
		spin_lock_irqsave(&su->lock, flags);
		su->desired_otg_role = SOC_USB_OTG_ROLE_IDLE;
		spin_unlock_irqrestore(&su->lock, flags);
		soc_usb_otg_update(su, 1);
		break;
	default:
		break;
	}

	/*
	 * When current_dr_role is not set, there's no role switching.
	 * Only perform GCTL.CoreSoftReset when there's DRD role switching.
	 */
	if (su->current_dr_role && desired_dr_role != SOC_USB_GCTL_PRTCAP_OTG) {
		reg = soc_usb_readl(su->regs, SOC_USB_GCTL);
		reg |= SOC_USB_GCTL_CORESOFTRESET;
		soc_usb_writel(su->regs, SOC_USB_GCTL, reg);

		/*
		 * Wait for internal clocks to synchronized. SOC USB may need at least 50ms. 
		 * To keep it consistent across different IPs, let's wait up to
		 * 100ms before clearing GCTL.CORESOFTRESET.
		 */
		msleep(100);

		reg = soc_usb_readl(su->regs, SOC_USB_GCTL);
		reg &= ~SOC_USB_GCTL_CORESOFTRESET;
		soc_usb_writel(su->regs, SOC_USB_GCTL, reg);
	}

	spin_lock_irqsave(&su->lock, flags);

	soc_usb_set_prtcap(su, desired_dr_role);

	spin_unlock_irqrestore(&su->lock, flags);

	switch (desired_dr_role) {
	case SOC_USB_GCTL_PRTCAP_HOST:
		ret = soc_usb_host_init(su);
		if (ret) {
			dev_err(su->dev, "failed to initialize host\n");
		}
		break;
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		soc_usb_core_soft_reset(su);

		soc_usb_event_buffers_setup(su);

		ret = soc_usb_gadget_init(su);
		if (ret)
			dev_err(su->dev, "failed to initialize peripheral\n");
		break;
	case SOC_USB_GCTL_PRTCAP_OTG:
		soc_usb_otg_init(su);
		soc_usb_otg_update(su, 0);
		break;
	default:
		break;
	}

out:
	pm_runtime_mark_last_busy(su->dev);
	pm_runtime_put_autosuspend(su->dev);
	mutex_unlock(&su->mutex);
}

void soc_usb_set_mode(struct soc_usb *su, u32 mode)
{
	unsigned long flags;

	if (su->dr_mode != USB_DR_MODE_OTG)
		return;

	spin_lock_irqsave(&su->lock, flags);
	su->desired_dr_role = mode;
	spin_unlock_irqrestore(&su->lock, flags);

	queue_work(system_freezable_wq, &su->drd_work);
}

/**
 * soc_usb_core_soft_reset - Issues core soft reset and PHY reset
 * @su: pointer to our context structure
 */
int soc_usb_core_soft_reset(struct soc_usb *su)
{
	u32		reg;
	int		retries = 1000;

	/*
	 * We're resetting only the device side because, if we're in host mode,
	 * XHCI driver will reset the host block. If soc_usb was configured for
	 * host-only mode, then we can return early.
	 */
	if (su->current_dr_role == SOC_USB_GCTL_PRTCAP_HOST)
		return 0;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	reg |= SOC_USB_DCTL_CSFTRST;
	reg &= ~SOC_USB_DCTL_RUN_STOP;
	soc_usb_gadget_dctl_write_safe(su, reg);

    if (SOC_USB_IP_IS(SOC_USB32))
		retries = 10;
    
	do {
		reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
		if (!(reg & SOC_USB_DCTL_CSFTRST))
			goto done;

        if (SOC_USB_IP_IS(SOC_USB32))
            msleep(20);
        else
			udelay(1);
	} while (--retries);

	dev_warn(su->dev, "SOC_USB controller soft reset failed.\n");
	return -ETIMEDOUT;

done:
	/*
	 * For USB3.2, once DCTL.CSFRST bit
	 * is cleared, we must wait at least 50ms before accessing the PHY
	 * domain (synchronization delay).
	 */
	if (SOC_USB_IP_IS(SOC_USB32))
		msleep(50);

	return 0;
}


/**
 * soc_usb_free_one_event_buffer - Frees one event buffer
 * @su: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void soc_usb_free_one_event_buffer(struct soc_usb *su,
		struct soc_usb_event_buffer *evt)
{
	dma_free_coherent(su->sysdev, evt->length, evt->buf, evt->dma);
}

/**
 * soc_usb_alloc_one_event_buffer - Allocates one event buffer structure
 * @su: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct soc_usb_event_buffer *soc_usb_alloc_one_event_buffer(struct soc_usb *su,
		unsigned int length)
{
	struct soc_usb_event_buffer	*evt;
	int ret = 0;

	evt = devm_kzalloc(su->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->su	= su;
	evt->length	= length;
	evt->cache	= devm_kzalloc(su->dev, length, GFP_KERNEL);
	if (!evt->cache)
		return ERR_PTR(-ENOMEM);

	ret = dma_set_mask(su->sysdev, DMA_BIT_MASK(64));
	ret |= dma_set_coherent_mask(su->sysdev, DMA_BIT_MASK(64));
	evt->buf	= dma_alloc_coherent(su->sysdev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf)
		return ERR_PTR(-ENOMEM);

	return evt;
}

/**
 * soc_usb_free_event_buffers - frees all allocated event buffers
 * @su: Pointer to our controller context structure
 */
static void soc_usb_free_event_buffers(struct soc_usb *su)
{
	struct soc_usb_event_buffer	*evt;

	evt = su->ev_buf;
	if (evt)
		soc_usb_free_one_event_buffer(su, evt);
}

/**
 * soc_usb_alloc_event_buffers - Allocates @num event buffers of size @length
 * @su: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, su
 * may contain some buffers allocated but not all which were requested.
 */
static int soc_usb_alloc_event_buffers(struct soc_usb *su, unsigned int length)
{
	struct soc_usb_event_buffer *evt;
	unsigned int hw_mode;

	hw_mode = SOC_USB_GHWPARAMS0_MODE(su->hwparams.hwparams0);
	if (hw_mode == SOC_USB_GHWPARAMS0_MODE_HOST) {
		su->ev_buf = NULL;
		return 0;
	}

	evt = soc_usb_alloc_one_event_buffer(su, length);
	if (IS_ERR(evt)) {
		dev_err(su->dev, "can't allocate event buffer\n");
		return PTR_ERR(evt);
	}
	su->ev_buf = evt;

	return 0;
}

/**
 * soc_usb_event_buffers_setup - setup our allocated event buffers
 * @su: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int soc_usb_event_buffers_setup(struct soc_usb *su)
{
	struct soc_usb_event_buffer	*evt;

	if (!su->ev_buf)
		return 0;

	evt = su->ev_buf;
	evt->lpos = 0;
	soc_usb_writel(su->regs, SOC_USB_GEVNTADRLO(0),
			lower_32_bits(evt->dma));
	soc_usb_writel(su->regs, SOC_USB_GEVNTADRHI(0),
			upper_32_bits(evt->dma));
	soc_usb_writel(su->regs, SOC_USB_GEVNTSIZ(0),
			SOC_USB_GEVNTSIZ_SIZE(evt->length));
	soc_usb_writel(su->regs, SOC_USB_GEVNTCOUNT(0), 0);

	return 0;
}

void soc_usb_event_buffers_cleanup(struct soc_usb *su)
{
	struct soc_usb_event_buffer	*evt;
	u32				reg;

	if (!su->ev_buf)
		return;

	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);
	if (!(reg & SOC_USB_DSTS_DEVCTRLHLT))
		return;

	evt = su->ev_buf;

	evt->lpos = 0;

	soc_usb_writel(su->regs, SOC_USB_GEVNTADRLO(0), 0);
	soc_usb_writel(su->regs, SOC_USB_GEVNTADRHI(0), 0);
	soc_usb_writel(su->regs, SOC_USB_GEVNTSIZ(0), SOC_USB_GEVNTSIZ_INTMASK
			| SOC_USB_GEVNTSIZ_SIZE(0));
	soc_usb_writel(su->regs, SOC_USB_GEVNTCOUNT(0), 0);
}

static void soc_usb_core_num_eps(struct soc_usb *su)
{
	struct soc_usb_hwparams	*parms = &su->hwparams;

	printk("###################################\n");
	printk("num ep = %d, in ep = %d\n", SOC_USB_NUM_EPS(parms), SOC_USB_NUM_IN_EPS(parms));
	printk("###################################\n");

	su->num_eps = SOC_USB_NUM_EPS(parms);
}

static void soc_usb_cache_hwparams(struct soc_usb *su)
{
	struct soc_usb_hwparams	*parms = &su->hwparams;

	parms->hwparams0 = soc_usb_readl(su->regs, SOC_USB_GHWPARAMS0);
	parms->hwparams3 = soc_usb_readl(su->regs, SOC_USB_GHWPARAMS3);
	parms->hwparams6 = soc_usb_readl(su->regs, SOC_USB_GHWPARAMS6);
	parms->hwparams7 = soc_usb_readl(su->regs, SOC_USB_GHWPARAMS7);
}

static void soc_usb_core_exit(struct soc_usb *su)
{
	soc_usb_event_buffers_cleanup(su);
}

static void soc_usb_core_setup_global_control(struct soc_usb *su)
{
	u32 reg;

	reg = soc_usb_readl(su->regs, SOC_USB_GCTL);
	reg &= ~SOC_USB_GCTL_SCALEDOWN_MASK;

	/* check if current soc_usb is on simulation board */
	if (su->hwparams.hwparams6 & SOC_USB_GHWPARAMS6_EN_FPGA) {
		dev_info(su->dev, "Running with FPGA optimizations\n");
		su->is_fpga = true;
	}

	WARN_ONCE(su->disable_scramble_quirk && !su->is_fpga,
			"disable_scramble cannot be used on non-FPGA builds\n");

	if (su->disable_scramble_quirk && su->is_fpga)
		reg |= SOC_USB_GCTL_DISSCRAMBLE;
	else
		reg &= ~SOC_USB_GCTL_DISSCRAMBLE;

	if (su->u2exit_lfps_quirk)
		reg |= SOC_USB_GCTL_U2EXIT_LFPS;

	soc_usb_writel(su->regs, SOC_USB_GCTL, reg);
}

static void soc_usb_config_threshold(struct soc_usb *su)
{
	u32 reg;
	u8 rx_thr_num;
	u8 rx_maxburst;
	u8 tx_thr_num;
	u8 tx_maxburst;

	/*
	 * Must config both number of packets and max burst settings to enable
	 * RX and/or TX threshold.
	 */
	if (!SOC_USB_IP_IS(SOC_USB) && su->dr_mode == USB_DR_MODE_HOST) {
		rx_thr_num = su->rx_thr_num_pkt_prd;
		rx_maxburst = su->rx_max_burst_prd;
		tx_thr_num = su->tx_thr_num_pkt_prd;
		tx_maxburst = su->tx_max_burst_prd;

		if (rx_thr_num && rx_maxburst) {
			reg = soc_usb_readl(su->regs, SOC_USB_GRXTHRCFG);
			reg |= SOC_USB32_RXTHRNUMPKTSEL_PRD;

			reg &= ~SOC_USB32_RXTHRNUMPKT_PRD(~0);
			reg |= SOC_USB32_RXTHRNUMPKT_PRD(rx_thr_num);

			reg &= ~SOC_USB32_MAXRXBURSTSIZE_PRD(~0);
			reg |= SOC_USB32_MAXRXBURSTSIZE_PRD(rx_maxburst);

			soc_usb_writel(su->regs, SOC_USB_GRXTHRCFG, reg);
		}

		if (tx_thr_num && tx_maxburst) {
			reg = soc_usb_readl(su->regs, SOC_USB_GTXTHRCFG);
			reg |= SOC_USB32_TXTHRNUMPKTSEL_PRD;

			reg &= ~SOC_USB32_TXTHRNUMPKT_PRD(~0);
			reg |= SOC_USB32_TXTHRNUMPKT_PRD(tx_thr_num);

			reg &= ~SOC_USB32_MAXTXBURSTSIZE_PRD(~0);
			reg |= SOC_USB32_MAXTXBURSTSIZE_PRD(tx_maxburst);

			soc_usb_writel(su->regs, SOC_USB_GTXTHRCFG, reg);
		}
	}

	rx_thr_num = su->rx_thr_num_pkt;
	rx_maxburst = su->rx_max_burst;
	tx_thr_num = su->tx_thr_num_pkt;
	tx_maxburst = su->tx_max_burst;

	if (SOC_USB_IP_IS(SOC_USB)) {
		if (rx_thr_num && rx_maxburst) {
			reg = soc_usb_readl(su->regs, SOC_USB_GRXTHRCFG);
			reg |= SOC_USB_GRXTHRCFG_PKTCNTSEL;

			reg &= ~SOC_USB_GRXTHRCFG_RXPKTCNT(~0);
			reg |= SOC_USB_GRXTHRCFG_RXPKTCNT(rx_thr_num);

			reg &= ~SOC_USB_GRXTHRCFG_MAXRXBURSTSIZE(~0);
			reg |= SOC_USB_GRXTHRCFG_MAXRXBURSTSIZE(rx_maxburst);

			soc_usb_writel(su->regs, SOC_USB_GRXTHRCFG, reg);
		}

		if (tx_thr_num && tx_maxburst) {
			reg = soc_usb_readl(su->regs, SOC_USB_GTXTHRCFG);
			reg |= SOC_USB_GTXTHRCFG_PKTCNTSEL;

			reg &= ~SOC_USB_GTXTHRCFG_TXPKTCNT(~0);
			reg |= SOC_USB_GTXTHRCFG_TXPKTCNT(tx_thr_num);

			reg &= ~SOC_USB_GTXTHRCFG_MAXTXBURSTSIZE(~0);
			reg |= SOC_USB_GTXTHRCFG_MAXTXBURSTSIZE(tx_maxburst);

			soc_usb_writel(su->regs, SOC_USB_GTXTHRCFG, reg);
		}
	} else {
		if (rx_thr_num && rx_maxburst) {
			reg = soc_usb_readl(su->regs, SOC_USB_GRXTHRCFG);
			reg |= SOC_USB32_GRXTHRCFG_PKTCNTSEL;

			reg &= ~SOC_USB32_GRXTHRCFG_RXPKTCNT(~0);
			reg |= SOC_USB32_GRXTHRCFG_RXPKTCNT(rx_thr_num);

			reg &= ~SOC_USB32_GRXTHRCFG_MAXRXBURSTSIZE(~0);
			reg |= SOC_USB32_GRXTHRCFG_MAXRXBURSTSIZE(rx_maxburst);

			soc_usb_writel(su->regs, SOC_USB_GRXTHRCFG, reg);
		}

		if (tx_thr_num && tx_maxburst) {
			reg = soc_usb_readl(su->regs, SOC_USB_GTXTHRCFG);
			reg |= SOC_USB32_GTXTHRCFG_PKTCNTSEL;

			reg &= ~SOC_USB32_GTXTHRCFG_TXPKTCNT(~0);
			reg |= SOC_USB32_GTXTHRCFG_TXPKTCNT(tx_thr_num);

			reg &= ~SOC_USB32_GTXTHRCFG_MAXTXBURSTSIZE(~0);
			reg |= SOC_USB32_GTXTHRCFG_MAXTXBURSTSIZE(tx_maxburst);

			soc_usb_writel(su->regs, SOC_USB_GTXTHRCFG, reg);
		}
	}
}

static int soc_usb_set_phy_interface_bitwidth(struct soc_usb *su)
{
	u32 reg;

	reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));

	switch (su->hsphy_mode) {
	case USBPHY_INTERFACE_MODE_UTMI:
		reg &= ~(SOC_USB_GUSB2PHYCFG_PHYIF_MASK |
		       SOC_USB_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= SOC_USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
		       SOC_USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
		break;
	case USBPHY_INTERFACE_MODE_UTMIW:
		reg &= ~(SOC_USB_GUSB2PHYCFG_PHYIF_MASK |
		       SOC_USB_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= SOC_USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
		       SOC_USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT);
		break;
	default:
		break;
	}

	soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);

	return 0;
}


static int soc_usb3_phy_intf_setup(struct soc_usb *su)
{
	u32 reg;

	soc_usb_set_phy_interface_bitwidth(su);
	
	reg = soc_usb_readl(su->regs, SOC_USB_GUSB3PIPECTL(0));
	reg &= ~SOC_USB_GUSB3PIPECTL_UX_EXIT_PX;

	if (su->dis_u3_susphy_quirk)
		reg &= ~SOC_USB_GUSB3PIPECTL_SUSPHY;

	if (su->u2ss_inp3_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_U2SSINP3OK;

	if (su->dis_rxdet_inp3_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_DISRXDETINP3;

	if (su->req_p1p2p3_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_REQP1P2P3;

	if (su->del_p1p2p3_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_DEP1P2P3_EN;

	if (su->del_phy_power_chg_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_DEPOCHANGE;

	if (su->lfps_filter_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_LFPSFILT;

	if (su->rx_detect_poll_quirk)
		reg |= SOC_USB_GUSB3PIPECTL_RX_DETOPOLL;


	if (su->dis_del_phy_power_chg_quirk)
		reg &= ~SOC_USB_GUSB3PIPECTL_DEPOCHANGE;

	soc_usb_writel(su->regs, SOC_USB_GUSB3PIPECTL(0), reg);

	reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));

	if (su->dis_u2_susphy_quirk)
		reg &= ~SOC_USB_GUSB2PHYCFG_SUSPHY;

	soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);

	return 0;

}


/**
 * soc_usb_core_init - Low-level initialization of SOC_USB Core
 * @su: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int soc_usb_core_init(struct soc_usb *su)
{
	u32			reg;
	int			ret;

	ret = soc_usb3_phy_intf_setup(su);
	if(ret)
		goto err_ret;

	ret = soc_usb_core_soft_reset(su);
	if (ret)
		goto err_ret;

	soc_usb_core_setup_global_control(su);
	soc_usb_core_num_eps(su);

	ret = soc_usb_event_buffers_setup(su);
	if (ret) {
		dev_err(su->dev, "failed to setup event buffers\n");
		goto err_ret;
	}

	/*
	 * When configured in HOST mode, after issuing U3/L2 exit controller
	 * fails to send proper CRC checksum in CRC5 feild. Because of this
	 * behaviour Transaction Error is generated, resulting in reset and
	 * re-enumeration of usb device attached. All the termsel, xcvrsel,
	 * opmode becomes 0 during end of resume. Enabling bit 10 of GUCTL1
	 * will correct this problem. This option is to support certain
	 * legacy ULPI PHYs.
	 */
	if (su->resume_hs_terminations) {
		reg = soc_usb_readl(su->regs, SOC_USB_GUCTL1);
		reg |= SOC_USB_GUCTL1_RESUME_OPMODE_HS_HOST;
		soc_usb_writel(su->regs, SOC_USB_GUCTL1, reg);
	}

	reg = soc_usb_readl(su->regs, SOC_USB_GUCTL1);
	/*
	 * Enable hardware control of sending remote wakeup
	 * in HS when the device is in the L1 state.
	 */
	reg |= SOC_USB_GUCTL1_DEV_L1_EXIT_BY_HW;
	/*
	 * Decouple USB 2.0 L1 & L2 events which will allow for
	 * gadget driver to only receive U3/L2 suspend & wakeup
	 * events and prevent the more frequent L1 LPM transitions
	 * from interrupting the driver.
	 */
	reg |= SOC_USB_GUCTL1_DEV_DECOUPLE_L1L2_EVT;

	if (su->dis_tx_ipgap_linecheck_quirk)
		reg |= SOC_USB_GUCTL1_TX_IPGAP_LINECHECK_DIS;

	soc_usb_writel(su->regs, SOC_USB_GUCTL1, reg);

	soc_usb_config_threshold(su);

	return 0;

err_ret:
	return ret;
}

static int soc_usb_core_init_mode(struct soc_usb *su)
{
	struct device *dev = su->dev;
	int ret;

	switch (su->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		soc_usb_set_prtcap(su, SOC_USB_GCTL_PRTCAP_DEVICE);

		ret = soc_usb_gadget_init(su);
		if (ret)
			return dev_err_probe(dev, ret, "failed to initialize gadget\n");
		break;
	case USB_DR_MODE_HOST:
		soc_usb_set_prtcap(su, SOC_USB_GCTL_PRTCAP_HOST);

		ret = soc_usb_host_init(su);
		if (ret)
			return dev_err_probe(dev, ret, "failed to initialize host\n");
		break;
	case USB_DR_MODE_OTG:
		INIT_WORK(&su->drd_work, __soc_usb_set_mode);
		ret = soc_usb_drd_init(su);
		if (ret)
			return dev_err_probe(dev, ret, "failed to initialize dual-role\n");
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", su->dr_mode);
		return -EINVAL;
	}

	return 0;
}

static void soc_usb_core_exit_mode(struct soc_usb *su)
{
	switch (su->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		soc_usb_gadget_exit(su);
		break;
	case USB_DR_MODE_HOST:
		soc_usb_host_exit(su);
		break;
	case USB_DR_MODE_OTG:
		soc_usb_drd_exit(su);
		break;
	default:
		/* do nothing */
		break;
	}

	/* de-assert DRVVBUS for HOST and OTG mode */
	soc_usb_set_prtcap(su, SOC_USB_GCTL_PRTCAP_DEVICE);
}

static void soc_usb_get_properties(struct soc_usb *su)
{
	struct device		*dev = su->dev;
	u8			lpm_nyet_threshold;
	u8			hird_threshold;
	u8			rx_thr_num_pkt = 0;
	u8			rx_max_burst = 0;
	u8			tx_thr_num_pkt = 0;
	u8			tx_max_burst = 0;
	u8			rx_thr_num_pkt_prd = 0;
	u8			rx_max_burst_prd = 0;
	u8			tx_thr_num_pkt_prd = 0;
	u8			tx_max_burst_prd = 0;
	u8			tx_fifo_resize_max_num;
	const char		*usb_psy_name;
	int			ret;

	/* default to highest possible threshold */
	lpm_nyet_threshold = 0xf;

	/*
	 * default to assert utmi_sleep_n and use maximum allowed HIRD
	 * threshold value of 0b1100
	 */
	hird_threshold = 12;

	/*
	 * default to a TXFIFO size large enough to fit 6 max packets.  This
	 * allows for systems with larger bus latencies to have some headroom
	 * for endpoints that have a large bMaxBurst value.
	 */
	tx_fifo_resize_max_num = 6;

	su->revision = 0;
	su->maximum_speed = usb_get_maximum_speed(dev);
	
	su->ip = SOC_USB32_IP;


	printk(">>>su->ip_name:0x%x\n", su->ip);
	su->max_ssp_rate = usb_get_maximum_ssp_rate(dev);
	su->dr_mode = usb_get_dr_mode(dev);
	su->hsphy_mode = of_usb_get_phy_mode(dev->of_node);

	su->sysdev_is_parent = device_property_read_bool(dev,
				"linux,sysdev_is_parent");
	if (su->sysdev_is_parent)
		su->sysdev = su->dev->parent;
	else
		su->sysdev = su->dev;

	su->sys_wakeup = device_may_wakeup(su->sysdev);

	ret = device_property_read_string(dev, "usb-psy-name", &usb_psy_name);
	if (ret >= 0) {
		su->usb_psy = power_supply_get_by_name(usb_psy_name);
		if (!su->usb_psy)
			dev_err(dev, "couldn't get usb power supply\n");
	}

	su->has_lpm_erratum = device_property_read_bool(dev,
				"soc,has-lpm-erratum");
	device_property_read_u8(dev, "soc,lpm-nyet-threshold",
				&lpm_nyet_threshold);
	su->is_utmi_l1_suspend = device_property_read_bool(dev,
				"soc,is-utmi-l1-suspend");
	device_property_read_u8(dev, "soc,hird-threshold",
				&hird_threshold);
	su->dis_start_transfer_quirk = device_property_read_bool(dev,
				"soc,dis-start-transfer-quirk");
	su->usb3_lpm_capable = device_property_read_bool(dev,
				"soc,usb3_lpm_capable");
	su->usb2_lpm_disable = device_property_read_bool(dev,
				"soc,usb2-lpm-disable");
	su->usb2_gadget_lpm_disable = device_property_read_bool(dev,
				"soc,usb2-gadget-lpm-disable");
	device_property_read_u8(dev, "soc,rx-thr-num-pkt",
				&rx_thr_num_pkt);
	device_property_read_u8(dev, "soc,rx-max-burst",
				&rx_max_burst);
	device_property_read_u8(dev, "soc,tx-thr-num-pkt",
				&tx_thr_num_pkt);
	device_property_read_u8(dev, "soc,tx-max-burst",
				&tx_max_burst);
	device_property_read_u8(dev, "soc,rx-thr-num-pkt-prd",
				&rx_thr_num_pkt_prd);
	device_property_read_u8(dev, "soc,rx-max-burst-prd",
				&rx_max_burst_prd);
	device_property_read_u8(dev, "soc,tx-thr-num-pkt-prd",
				&tx_thr_num_pkt_prd);
	device_property_read_u8(dev, "soc,tx-max-burst-prd",
				&tx_max_burst_prd);
	su->do_fifo_resize = device_property_read_bool(dev,
							"tx-fifo-resize");
	if (su->do_fifo_resize)
		device_property_read_u8(dev, "tx-fifo-max-num",
					&tx_fifo_resize_max_num);

	su->disable_scramble_quirk = device_property_read_bool(dev,
				"soc,disable_scramble_quirk");
	su->u2exit_lfps_quirk = device_property_read_bool(dev,
				"soc,u2exit_lfps_quirk");
	su->u2ss_inp3_quirk = device_property_read_bool(dev,
				"soc,u2ss_inp3_quirk");
	su->req_p1p2p3_quirk = device_property_read_bool(dev,
				"soc,req_p1p2p3_quirk");
	su->del_p1p2p3_quirk = device_property_read_bool(dev,
				"soc,del_p1p2p3_quirk");
	su->del_phy_power_chg_quirk = device_property_read_bool(dev,
				"soc,del_phy_power_chg_quirk");
	su->lfps_filter_quirk = device_property_read_bool(dev,
				"soc,lfps_filter_quirk");
	su->rx_detect_poll_quirk = device_property_read_bool(dev,
				"soc,rx_detect_poll_quirk");
	su->dis_u3_susphy_quirk = device_property_read_bool(dev,
				"soc,dis_u3_susphy_quirk");
	su->dis_u2_susphy_quirk = device_property_read_bool(dev,
				"soc,dis_u2_susphy_quirk");
	su->dis_u1_entry_quirk = device_property_read_bool(dev,
				"soc,dis-u1-entry-quirk");
	su->dis_u2_entry_quirk = device_property_read_bool(dev,
				"soc,dis-u2-entry-quirk");
	su->dis_rxdet_inp3_quirk = device_property_read_bool(dev,
				"soc,dis_rxdet_inp3_quirk");
	su->dis_del_phy_power_chg_quirk = device_property_read_bool(dev,
				"soc,dis-del-phy-power-chg-quirk");
	su->dis_tx_ipgap_linecheck_quirk = device_property_read_bool(dev,
				"soc,dis-tx-ipgap-linecheck-quirk");
	su->resume_hs_terminations = device_property_read_bool(dev,
				"soc,resume-hs-terminations");

	su->lpm_nyet_threshold = lpm_nyet_threshold;

	su->hird_threshold = hird_threshold;

	su->rx_thr_num_pkt = rx_thr_num_pkt;
	su->rx_max_burst = rx_max_burst;

	su->tx_thr_num_pkt = tx_thr_num_pkt;
	su->tx_max_burst = tx_max_burst;

	su->rx_thr_num_pkt_prd = rx_thr_num_pkt_prd;
	su->rx_max_burst_prd = rx_max_burst_prd;

	su->tx_thr_num_pkt_prd = tx_thr_num_pkt_prd;
	su->tx_max_burst_prd = tx_max_burst_prd;

	su->imod_interval = 0;

	su->tx_fifo_resize_max_num = tx_fifo_resize_max_num;
}

/* check whether the core supports IMOD */
bool soc_usb_has_imod(struct soc_usb *su)
{
	return 1;
}

static void soc_usb_check_params(struct soc_usb *su)
{
	struct device *dev = su->dev;
	unsigned int hwparam_gen =
		SOC_USB_GHWPARAMS3_SSPHY_IFC(su->hwparams.hwparams3);

	/* Check for proper value of imod_interval */
	if (su->imod_interval && !soc_usb_has_imod(su)) {
		dev_warn(su->dev, "Interrupt moderation not supported\n");
		su->imod_interval = 0;
	}

	/* Check the maximum_speed parameter */
	switch (su->maximum_speed) {
	case USB_SPEED_FULL:
	case USB_SPEED_HIGH:
		break;
	case USB_SPEED_SUPER:
		if (hwparam_gen == SOC_USB_GHWPARAMS3_SSPHY_IFC_DIS)
			dev_warn(dev, "UDC doesn't support Gen 1\n");
		break;
	case USB_SPEED_SUPER_PLUS:
        if ((SOC_USB_IP_IS(SOC_USB32) &&
		     hwparam_gen == SOC_USB_GHWPARAMS3_SSPHY_IFC_DIS) ||
		    (!SOC_USB_IP_IS(SOC_USB32) &&
		     hwparam_gen != SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN2))
			dev_warn(dev, "UDC doesn't support SSP\n");
		break;
	default:
		dev_err(dev, "invalid maximum_speed parameter %d\n",
			su->maximum_speed);
		fallthrough;
	case USB_SPEED_UNKNOWN:
		switch (hwparam_gen) {
		case SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN2:
			su->maximum_speed = USB_SPEED_SUPER_PLUS;
			break;
		case SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN1:
            if (SOC_USB_IP_IS(SOC_USB32))
				su->maximum_speed = USB_SPEED_SUPER_PLUS;
			else
			    su->maximum_speed = USB_SPEED_SUPER;
			break;
		case SOC_USB_GHWPARAMS3_SSPHY_IFC_DIS:
			su->maximum_speed = USB_SPEED_HIGH;
			break;
		default:
			su->maximum_speed = USB_SPEED_SUPER;
			break;
		}
		break;
	}

	/*
	 * Currently the controller does not have visibility into the HW
	 * parameter to determine the maximum number of lanes the HW supports.
	 * If the number of lanes is not specified in the device property, then
	 * set the default to support dual-lane for SOC USB and single-lane
	 * for SOC USB for super-speed-plus.
	 */
	if (su->maximum_speed == USB_SPEED_SUPER_PLUS) {
		switch (su->max_ssp_rate) {
		case USB_SSP_GEN_2x1:
			if (hwparam_gen == SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN1)
				dev_warn(dev, "UDC only supports Gen 1\n");
			break;
		case USB_SSP_GEN_1x2:
		case USB_SSP_GEN_2x2:
			if (SOC_USB_IP_IS(SOC_USB32))
				dev_warn(dev, "UDC only supports single lane\n");
			break;
		case USB_SSP_GEN_UNKNOWN:
		default:
			switch (hwparam_gen) {
			case SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN2:
                if (SOC_USB_IP_IS(SOC_USB32))
                    su->max_ssp_rate = USB_SSP_GEN_2x2;
                else
					su->max_ssp_rate = USB_SSP_GEN_2x1;
				break;
			case SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN1:
                 if (SOC_USB_IP_IS(SOC_USB32))
                    su->max_ssp_rate = USB_SSP_GEN_1x2;
				break;
			}
			break;
		}
	}
}

/* phy and clock initalization, post-silicon need to be implemented. */
static int  soc_usb_clk_phy_init(struct platform_device *pdev)
{
	/* step 1 */
	/* all controller and phy reset signales are in the reset state. */

	/* step 2 */
	/* release the present of phy. */

	/* step 3 */
	/* configure phy register  */
	/* if there are no registers that need to be configured, omit them. */

	/* step 4 */
	/* release the power of phy.  */

	/* step 5 */
	/* delay at least 500us */

	/* step 6 */
	/* release the ahb_reset of controller */

	/* step 7 */
	/* relase the vcc_reset */

	/* step 8 */
	/* release others reset signales, now start the initialization of the controller. */

	/* setp 9 */
	/* read usb3 phy pll register  */

	return 0;
}


static int soc_usb_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct resource		*res, su_res;
	void __iomem		*regs;
	struct soc_usb		*su;
	int			ret;

	soc_usb_clk_phy_init(pdev);

	su = devm_kzalloc(dev, sizeof(*su), GFP_KERNEL);
	if (!su)
		return -ENOMEM;

	su->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}

	su->xhci_resources[0].start = res->start;
	su->xhci_resources[0].end = su->xhci_resources[0].start +
					SOC_USB_XHCI_REGS_END;
	su->xhci_resources[0].flags = res->flags;
	su->xhci_resources[0].name = res->name;

	/*
	 * Request memory region but exclude xHCI regs,
	 * since it will be requested by the xhci-plat driver.
	 */
	su_res = *res;
	su_res.start += SOC_USB_GLOBALS_REGS_START;

	regs = devm_ioremap_resource(dev, &su_res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	su->regs	= regs;
	su->regs_size	= resource_size(&su_res);

	soc_usb_get_properties(su);

	platform_set_drvdata(pdev, su);
	soc_usb_cache_hwparams(su);

	if (!su->sysdev_is_parent &&
	    SOC_USB_GHWPARAMS0_AWIDTH(su->hwparams.hwparams0) == 64) {
		ret = dma_set_mask_and_coherent(su->sysdev, DMA_BIT_MASK(64));
		if (ret)
			goto err_ret;
	}

	spin_lock_init(&su->lock);
	mutex_init(&su->mutex);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, SOC_USB_DEFAULT_AUTOSUSPEND_DELAY);
	pm_runtime_enable(dev);

	pm_runtime_forbid(dev);

	ret = soc_usb_alloc_event_buffers(su, SOC_USB_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(su->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err_allow_rpm;
	}

	ret = soc_usb_get_dr_mode(su);
	if (ret)
		goto err_free_event_buffers;

	ret = soc_usb_core_init(su);
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialize core\n");
		goto err_free_event_buffers;
	}

	soc_usb_check_params(su);
	soc_usb_debugfs_init(su);

	ret = soc_usb_core_init_mode(su);
	if (ret)
		goto err_exit_debugfs;

	pm_runtime_put(dev);

	dma_set_max_seg_size(dev, UINT_MAX);

	return 0;

err_exit_debugfs:
	soc_usb_debugfs_exit(su);
	soc_usb_event_buffers_cleanup(su);
err_free_event_buffers:
	soc_usb_free_event_buffers(su);
err_allow_rpm:
	pm_runtime_allow(dev);
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);
err_ret:

	return ret;
}

static void soc_usb_remove(struct platform_device *pdev)
{
	struct soc_usb	*su = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);

	soc_usb_core_exit_mode(su);
	soc_usb_debugfs_exit(su);

	soc_usb_core_exit(su);

	pm_runtime_allow(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	/*
	 * HACK: Clear the driver data, which is currently accessed by parent
	 * glue drivers, before allowing the parent to suspend.
	 */
	platform_set_drvdata(pdev, NULL);
	pm_runtime_set_suspended(&pdev->dev);

	soc_usb_free_event_buffers(su);
}

#ifdef CONFIG_PM
static int soc_usb_core_init_for_resume(struct soc_usb *su)
{
	int ret;
	ret = soc_usb_core_init(su);
	if (ret)
		goto err_ret;

	return 0;

err_ret:

	return ret;
}

static int soc_usb_suspend_common(struct soc_usb *su, pm_message_t msg)
{
	u32 reg;

	switch (su->current_dr_role) {
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		if (pm_runtime_suspended(su->dev))
			break;
		soc_usb_gadget_suspend(su);
		synchronize_irq(su->irq_gadget);
		soc_usb_core_exit(su);
		break;
	case SOC_USB_GCTL_PRTCAP_HOST:
		if (!PMSG_IS_AUTO(msg) && !device_may_wakeup(su->dev)) {
			soc_usb_core_exit(su);
			break;
		}

		/* Let controller to suspend HSPHY before PHY driver suspends */
		if (su->dis_u2_susphy_quirk) {
			reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));
			reg |=  SOC_USB_GUSB2PHYCFG_SUSPHY;
			soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);

			/* Give some time for USB2 PHY to suspend */
			usleep_range(5000, 6000);
		}

		break;
	case SOC_USB_GCTL_PRTCAP_OTG:
		/* do nothing during runtime_suspend */
		if (PMSG_IS_AUTO(msg))
			break;

		if (su->current_otg_role == SOC_USB_OTG_ROLE_DEVICE) {
			soc_usb_gadget_suspend(su);
			synchronize_irq(su->irq_gadget);
		}

		soc_usb_otg_exit(su);
		soc_usb_core_exit(su);
		break;
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int soc_usb_resume_common(struct soc_usb *su, pm_message_t msg)
{
	int		ret;
	u32		reg;

	switch (su->current_dr_role) {
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		ret = soc_usb_core_init_for_resume(su);
		if (ret)
			return ret;

		soc_usb_set_prtcap(su, SOC_USB_GCTL_PRTCAP_DEVICE);
		soc_usb_gadget_resume(su);
		break;
	case SOC_USB_GCTL_PRTCAP_HOST:
		if (!PMSG_IS_AUTO(msg) && !device_may_wakeup(su->dev)) {
			ret = soc_usb_core_init_for_resume(su);
			if (ret)
				return ret;
			soc_usb_set_prtcap(su, SOC_USB_GCTL_PRTCAP_HOST);
			break;
		}
		/* Restore GUSB2PHYCFG bits that were modified in suspend */
		reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));
		if (su->dis_u2_susphy_quirk)
			reg &= ~SOC_USB_GUSB2PHYCFG_SUSPHY;

		soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);

		phy_pm_runtime_get_sync(su->usb2_generic_phy);
		phy_pm_runtime_get_sync(su->usb3_generic_phy);
		break;
	case SOC_USB_GCTL_PRTCAP_OTG:
		/* nothing to do on runtime_resume */
		if (PMSG_IS_AUTO(msg))
			break;

		ret = soc_usb_core_init_for_resume(su);
		if (ret)
			return ret;

		soc_usb_set_prtcap(su, su->current_dr_role);

		soc_usb_otg_init(su);
		if (su->current_otg_role == SOC_USB_OTG_ROLE_HOST) {
			soc_usb_otg_host_init(su);
		} else if (su->current_otg_role == SOC_USB_OTG_ROLE_DEVICE) {
			soc_usb_gadget_resume(su);
		}

		break;
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int soc_usb_runtime_checks(struct soc_usb *su)
{
	switch (su->current_dr_role) {
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		if (su->connected)
			return -EBUSY;
		break;
	case SOC_USB_GCTL_PRTCAP_HOST:
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int soc_usb_runtime_suspend(struct device *dev)
{
	struct soc_usb     *su = dev_get_drvdata(dev);
	int		ret;

	if (soc_usb_runtime_checks(su))
		return -EBUSY;

	ret = soc_usb_suspend_common(su, PMSG_AUTO_SUSPEND);
	if (ret)
		return ret;

	return 0;
}

static int soc_usb_runtime_resume(struct device *dev)
{
	struct soc_usb     *su = dev_get_drvdata(dev);
	int		ret;

	ret = soc_usb_resume_common(su, PMSG_AUTO_RESUME);
	if (ret)
		return ret;

	switch (su->current_dr_role) {
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		soc_usb_gadget_process_pending_events(su);
		break;
	case SOC_USB_GCTL_PRTCAP_HOST:
	default:
		/* do nothing */
		break;
	}

	pm_runtime_mark_last_busy(dev);

	return 0;
}

static int soc_usb_runtime_idle(struct device *dev)
{
	struct soc_usb     *su = dev_get_drvdata(dev);

	switch (su->current_dr_role) {
	case SOC_USB_GCTL_PRTCAP_DEVICE:
		if (soc_usb_runtime_checks(su))
			return -EBUSY;
		break;
	case SOC_USB_GCTL_PRTCAP_HOST:
	default:
		/* do nothing */
		break;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_autosuspend(dev);

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
int soc_usb_suspend(struct device *dev)
{
	struct soc_usb	*su = dev_get_drvdata(dev);
	int		ret;

	ret = soc_usb_suspend_common(su, PMSG_SUSPEND);
	if (ret)
		return ret;

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}
EXPORT_SYMBOL(soc_usb_suspend);

int soc_usb_resume(struct device *dev)
{
	struct soc_usb	*su = dev_get_drvdata(dev);
	int		ret;

	pinctrl_pm_select_default_state(dev);

	ret = soc_usb_resume_common(su, PMSG_RESUME);
	if (ret)
		return ret;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}
EXPORT_SYMBOL(soc_usb_resume);

#define soc_usb_complete NULL
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops soc_usb_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(soc_usb_suspend, soc_usb_resume)
	.complete = soc_usb_complete,
	SET_RUNTIME_PM_OPS(soc_usb_runtime_suspend, soc_usb_runtime_resume,
			soc_usb_runtime_idle)
};

#ifdef CONFIG_OF
static const struct of_device_id of_soc_usb_match[] = {
	{
		.compatible = "soc,soc_drd2"
	},
	{
		.compatible = "soc,soc_drd3"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_soc_usb_match);
#endif

#ifdef CONFIG_ACPI

#define ACPI_ID_INTEL_BSW	"808622B7"

static const struct acpi_device_id soc_usb_acpi_match[] = {
	{ ACPI_ID_INTEL_BSW, 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, soc_usb_acpi_match);
#endif

static struct platform_driver soc_usb_driver = {
	.probe		= soc_usb_probe,
	.remove_new	= soc_usb_remove,
	.driver		= {
		.name	= "soc_drd",
		.of_match_table	= of_match_ptr(of_soc_usb_match),
		.acpi_match_table = ACPI_PTR(soc_usb_acpi_match),
		.pm	= &soc_usb_dev_pm_ops,
	},
};

module_platform_driver(soc_usb_driver);

MODULE_LICENSE("GPL v2");
