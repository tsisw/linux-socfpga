// SPDX-License-Identifier: GPL-2.0
/*
 * soc_gadget_ep0.c - SOC USB3 DRD Controller Endpoint 0 Handling
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include "soc_core.h"
#include "soc_debug.h"
#include "soc_gadget.h"
#include "soc_io.h"

static void __soc_usb_ep0_do_control_status(struct soc_usb *su, struct soc_usb_ep *dep);
static void __soc_usb_ep0_do_control_data(struct soc_usb *su,
		struct soc_usb_ep *dep, struct soc_usb_request *req);
static int soc_usb_ep0_delegate_req(struct soc_usb *su,
				 struct usb_ctrlrequest *ctrl);

static void soc_usb_ep0_prepare_one_trb(struct soc_usb_ep *dep,
		dma_addr_t buf_dma, u32 len, u32 type, bool chain)
{
	struct soc_usb_trb			*trb;
	struct soc_usb			*su;

	su = dep->su;
	trb = &su->ep0_trb[dep->trb_enqueue];

	if (chain)
		dep->trb_enqueue++;

	trb->bpl = lower_32_bits(buf_dma);
	trb->bph = upper_32_bits(buf_dma);
	trb->size = len;
	trb->ctrl = type;

	trb->ctrl |= (SOC_USB_TRB_CTRL_HWO
			| SOC_USB_TRB_CTRL_ISP_IMI);

	if (chain)
		trb->ctrl |= SOC_USB_TRB_CTRL_CHN;
	else
		trb->ctrl |= (SOC_USB_TRB_CTRL_IOC
				| SOC_USB_TRB_CTRL_LST);

	trace_soc_usb_prepare_trb(dep, trb);
}

static int soc_usb_ep0_start_trans(struct soc_usb_ep *dep)
{
	struct soc_usb_gadget_ep_cmd_params params;
	struct soc_usb			*su;
	int				ret;

	if (dep->flags & SOC_USB_EP_TRANSFER_STARTED)
		return 0;

	su = dep->su;

	memset(&params, 0, sizeof(params));
	params.param0 = upper_32_bits(su->ep0_trb_addr);
	params.param1 = lower_32_bits(su->ep0_trb_addr);

	ret = soc_usb_send_gadget_ep_cmd(dep, SOC_USB_DEPCMD_STARTTRANSFER, &params);
	if (ret < 0)
		return ret;

	su->ep0_next_event = SOC_USB_EP0_COMPLETE;

	return 0;
}

static int __soc_usb_gadget_ep0_queue(struct soc_usb_ep *dep,
		struct soc_usb_request *req)
{
	struct soc_usb		*su = dep->su;

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->epnum		= dep->number;

	list_add_tail(&req->list, &dep->pending_list);

	/*
	 * Gadget driver might not be quick enough to queue a request
	 * before we get a Transfer Not Ready event on this endpoint.
	 *
	 * In that case, we will set SOC_USB_EP_PENDING_REQUEST. When that
	 * flag is set, it's telling us that as soon as Gadget queues the
	 * required request, we should kick the transfer here because the
	 * IRQ we were waiting for is long gone.
	 */
	if (dep->flags & SOC_USB_EP_PENDING_REQUEST) {
		unsigned int direction;

		direction = !!(dep->flags & SOC_USB_EP0_DIR_IN);

		if (su->ep0state != EP0_DATA_PHASE) {
			dev_WARN(su->dev, "Unexpected pending request\n");
			return 0;
		}

		__soc_usb_ep0_do_control_data(su, su->eps[direction], req);

		dep->flags &= ~(SOC_USB_EP_PENDING_REQUEST |
				SOC_USB_EP0_DIR_IN);

		return 0;
	}

	/*
	 * In case gadget driver asked us to delay the STATUS phase,
	 * handle it here.
	 */
	if (su->delayed_status) {
		unsigned int direction;

		direction = !su->ep0_expect_in;
		su->delayed_status = false;
		usb_gadget_set_state(su->gadget, USB_STATE_CONFIGURED);

		if (su->ep0state == EP0_STATUS_PHASE)
			__soc_usb_ep0_do_control_status(su, su->eps[direction]);

		return 0;
	}

	/*
	 * Unfortunately we have uncovered a limitation wrt the Data Phase.
	 *
	 * Section 9.4 says we can wait for the XferNotReady(DATA) event to
	 * come before issueing Start Transfer command, but if we do, we will
	 * miss situations where the host starts another SETUP phase instead of
	 * the DATA phase.  Such cases happen at least on TD.7.6 of the Link
	 * Layer Compliance Suite.
	 *
	 * The problem surfaces due to the fact that in case of back-to-back
	 * SETUP packets there will be no XferNotReady(DATA) generated and we
	 * will be stuck waiting for XferNotReady(DATA) forever.
	 *
	 * By looking at tables 9-13 and 9-14 of the Databook, we can see that
	 * it tells us to start Data Phase right away. It also mentions that if
	 * we receive a SETUP phase instead of the DATA phase, core will issue
	 * XferComplete for the DATA phase, before actually initiating it in
	 * the wire, with the TRB's status set to "SETUP_PENDING". Such status
	 * can only be used to print some debugging logs, as the core expects
	 * us to go through to the STATUS phase and start a CONTROL_STATUS TRB,
	 * just so it completes right away, without transferring anything and,
	 * only then, we can go back to the SETUP phase.
	 *
	 * Because of this scenario, soc usb decided to change the programming
	 * model of control transfers and support on-demand transfers only for
	 * the STATUS phase. To fix the issue we have now, we will always wait
	 * for gadget driver to queue the DATA phase's struct usb_request, then
	 * start it right away.
	 *
	 * If we're actually in a 2-stage transfer, we will wait for
	 * XferNotReady(STATUS).
	 */
	if (su->three_stage_setup) {
		unsigned int direction;

		direction = su->ep0_expect_in;
		su->ep0state = EP0_DATA_PHASE;

		__soc_usb_ep0_do_control_data(su, su->eps[direction], req);

		dep->flags &= ~SOC_USB_EP0_DIR_IN;
	}

	return 0;
}

int soc_usb_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *request,
		gfp_t gfp_flags)
{
	struct soc_usb_request		*req = to_soc_usb_request(request);
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&su->lock, flags);
	if (!dep->endpoint.desc || !su->pullups_connected || !su->connected) {
		dev_err(su->dev, "%s: can't queue to disabled endpoint\n",
				dep->name);
		ret = -ESHUTDOWN;
		goto out;
	}

	/* we share one TRB for ep0/1 */
	if (!list_empty(&dep->pending_list)) {
		ret = -EBUSY;
		goto out;
	}

	ret = __soc_usb_gadget_ep0_queue(dep, req);

out:
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

void soc_usb_ep0_stall_and_restart(struct soc_usb *su)
{
	struct soc_usb_ep		*dep;

	/* reinitialize physical ep1 */
	dep = su->eps[1];
	dep->flags = SOC_USB_EP_ENABLED;

	/* stall is always issued on EP0 */
	dep = su->eps[0];
	__soc_usb_gadget_ep_set_halt(dep, 1, false);
	dep->flags = SOC_USB_EP_ENABLED;
	su->delayed_status = false;

	if (!list_empty(&dep->pending_list)) {
		struct soc_usb_request	*req;

		req = next_request(&dep->pending_list);
		if (!su->connected)
			soc_usb_gadget_giveback(dep, req, -ESHUTDOWN);
		else
			soc_usb_gadget_giveback(dep, req, -ECONNRESET);
	}

	su->eps[0]->trb_enqueue = 0;
	su->eps[1]->trb_enqueue = 0;
	su->ep0state = EP0_SETUP_PHASE;
	soc_usb_ep0_out_start(su);
}

int __soc_usb_gadget_ep0_set_halt(struct usb_ep *ep, int value)
{
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;

	soc_usb_ep0_stall_and_restart(su);

	return 0;
}

int soc_usb_gadget_ep0_set_halt(struct usb_ep *ep, int value)
{
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;
	unsigned long			flags;
	int				ret;

	spin_lock_irqsave(&su->lock, flags);
	ret = __soc_usb_gadget_ep0_set_halt(ep, value);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

void soc_usb_ep0_out_start(struct soc_usb *su)
{
	struct soc_usb_ep			*dep;
	int				ret;
	int                             i;

	complete(&su->ep0_in_setup);

	dep = su->eps[0];
	soc_usb_ep0_prepare_one_trb(dep, su->ep0_trb_addr, 8,
			SOC_USB_TRBCTL_CONTROL_SETUP, false);
	ret = soc_usb_ep0_start_trans(dep);
	WARN_ON(ret < 0);
	for (i = 2; i < SOC_USB_ENDPOINTS_NUM; i++) {
		struct soc_usb_ep *soc_usb_ep;

		soc_usb_ep = su->eps[i];
		if (!soc_usb_ep)
			continue;

		if (!(soc_usb_ep->flags & SOC_USB_EP_DELAY_STOP))
			continue;

		soc_usb_ep->flags &= ~SOC_USB_EP_DELAY_STOP;
		if (su->connected)
			soc_usb_stop_active_transfer(soc_usb_ep, true, true);
		else
			soc_usb_remove_requests(su, soc_usb_ep, -ESHUTDOWN);
	}
}

static struct soc_usb_ep *soc_usb_wIndex_to_dep(struct soc_usb *su, __le16 wIndex_le)
{
	struct soc_usb_ep		*dep;
	u32			windex = le16_to_cpu(wIndex_le);
	u32			epnum;

	epnum = (windex & USB_ENDPOINT_NUMBER_MASK) << 1;
	if ((windex & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
		epnum |= 1;

	dep = su->eps[epnum];
	if (dep == NULL)
		return NULL;

	if (dep->flags & SOC_USB_EP_ENABLED)
		return dep;

	return NULL;
}

static void soc_usb_ep0_status_cmpl(struct usb_ep *ep, struct usb_request *req)
{
}
/*
 * ch 9.4.5
 */
static int soc_usb_ep0_handle_status(struct soc_usb *su,
		struct usb_ctrlrequest *ctrl)
{
	struct soc_usb_ep		*dep;
	u32			recip;
	u32			value;
	u32			reg;
	u16			usb_status = 0;
	__le16			*response_pkt;

	/* We don't support PTM_STATUS */
	value = le16_to_cpu(ctrl->wValue);
	if (value != 0)
		return -EINVAL;

	recip = ctrl->bRequestType & USB_RECIP_MASK;
	switch (recip) {
	case USB_RECIP_DEVICE:
		/*
		 * LTM will be set once we know how to set this in HW.
		 */
		usb_status |= su->gadget->is_selfpowered;

		if ((su->speed == SOC_USB_DSTS_SUPERSPEED) ||
		    (su->speed == SOC_USB_DSTS_SUPERSPEED_PLUS)) {
			reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
			if (reg & SOC_USB_DCTL_INITU1ENA)
				usb_status |= 1 << USB_DEV_STAT_U1_ENABLED;
			if (reg & SOC_USB_DCTL_INITU2ENA)
				usb_status |= 1 << USB_DEV_STAT_U2_ENABLED;
		} else {
			usb_status |= su->gadget->wakeup_armed <<
					USB_DEVICE_REMOTE_WAKEUP;
		}

		break;

	case USB_RECIP_INTERFACE:
		/*
		 * Function Remote Wake Capable	D0
		 * Function Remote Wakeup	D1
		 */
		return soc_usb_ep0_delegate_req(su, ctrl);

	case USB_RECIP_ENDPOINT:
		dep = soc_usb_wIndex_to_dep(su, ctrl->wIndex);
		if (!dep)
			return -EINVAL;

		if (dep->flags & SOC_USB_EP_STALL)
			usb_status = 1 << USB_ENDPOINT_HALT;
		break;
	default:
		return -EINVAL;
	}

	response_pkt = (__le16 *) su->setup_buf;
	*response_pkt = cpu_to_le16(usb_status);

	dep = su->eps[0];
	su->ep0_usb_req.dep = dep;
	su->ep0_usb_req.request.length = sizeof(*response_pkt);
	su->ep0_usb_req.request.buf = su->setup_buf;
	su->ep0_usb_req.request.complete = soc_usb_ep0_status_cmpl;

	return __soc_usb_gadget_ep0_queue(dep, &su->ep0_usb_req);
}

static int soc_usb_ep0_handle_u1(struct soc_usb *su, enum usb_device_state state,
		int set)
{
	u32 reg;

	if (state != USB_STATE_CONFIGURED)
		return -EINVAL;
	if ((su->speed != SOC_USB_DSTS_SUPERSPEED) &&
			(su->speed != SOC_USB_DSTS_SUPERSPEED_PLUS))
		return -EINVAL;
	if (set && su->dis_u1_entry_quirk)
		return -EINVAL;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	if (set)
		reg |= SOC_USB_DCTL_INITU1ENA;
	else
		reg &= ~SOC_USB_DCTL_INITU1ENA;
	soc_usb_writel(su->regs, SOC_USB_DCTL, reg);

	return 0;
}

static int soc_usb_ep0_handle_u2(struct soc_usb *su, enum usb_device_state state,
		int set)
{
	u32 reg;


	if (state != USB_STATE_CONFIGURED)
		return -EINVAL;
	if ((su->speed != SOC_USB_DSTS_SUPERSPEED) &&
			(su->speed != SOC_USB_DSTS_SUPERSPEED_PLUS))
		return -EINVAL;
	if (set && su->dis_u2_entry_quirk)
		return -EINVAL;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	if (set)
		reg |= SOC_USB_DCTL_INITU2ENA;
	else
		reg &= ~SOC_USB_DCTL_INITU2ENA;
	soc_usb_writel(su->regs, SOC_USB_DCTL, reg);

	return 0;
}

static int soc_usb_ep0_handle_test(struct soc_usb *su, enum usb_device_state state,
		u32 wIndex, int set)
{
	if ((wIndex & 0xff) != 0)
		return -EINVAL;
	if (!set)
		return -EINVAL;

	switch (wIndex >> 8) {
	case USB_TEST_J:
	case USB_TEST_K:
	case USB_TEST_SE0_NAK:
	case USB_TEST_PACKET:
	case USB_TEST_FORCE_ENABLE:
		su->test_mode_nr = wIndex >> 8;
		su->test_mode = true;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int soc_usb_ep0_handle_device(struct soc_usb *su,
		struct usb_ctrlrequest *ctrl, int set)
{
	enum usb_device_state	state;
	u32			wValue;
	u32			wIndex;
	int			ret = 0;

	wValue = le16_to_cpu(ctrl->wValue);
	wIndex = le16_to_cpu(ctrl->wIndex);
	state = su->gadget->state;

	switch (wValue) {
	case USB_DEVICE_REMOTE_WAKEUP:
		if (su->wakeup_configured)
			su->gadget->wakeup_armed = set;
		else
			ret = -EINVAL;
		break;
	/*
	 * 9.4.1 says only for SS, in AddressState only for
	 * default control pipe
	 */
	case USB_DEVICE_U1_ENABLE:
		ret = soc_usb_ep0_handle_u1(su, state, set);
		break;
	case USB_DEVICE_U2_ENABLE:
		ret = soc_usb_ep0_handle_u2(su, state, set);
		break;
	case USB_DEVICE_LTM_ENABLE:
		ret = -EINVAL;
		break;
	case USB_DEVICE_TEST_MODE:
		ret = soc_usb_ep0_handle_test(su, state, wIndex, set);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int soc_usb_ep0_handle_intf(struct soc_usb *su,
		struct usb_ctrlrequest *ctrl, int set)
{
	u32			wValue;
	int			ret = 0;

	wValue = le16_to_cpu(ctrl->wValue);

	switch (wValue) {
	case USB_INTRF_FUNC_SUSPEND:
		ret = soc_usb_ep0_delegate_req(su, ctrl);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int soc_usb_ep0_handle_endpoint(struct soc_usb *su,
		struct usb_ctrlrequest *ctrl, int set)
{
	struct soc_usb_ep		*dep;
	u32			wValue;
	int			ret;

	wValue = le16_to_cpu(ctrl->wValue);

	switch (wValue) {
	case USB_ENDPOINT_HALT:
		dep = soc_usb_wIndex_to_dep(su, ctrl->wIndex);
		if (!dep)
			return -EINVAL;

		if (set == 0 && (dep->flags & SOC_USB_EP_WEDGE))
			break;

		ret = __soc_usb_gadget_ep_set_halt(dep, set, true);
		if (ret)
			return -EINVAL;

		/* ClearFeature(Halt) may need delayed status */
		if (!set && (dep->flags & SOC_USB_EP_END_TRANSFER_PENDING))
			return USB_GADGET_DELAYED_STATUS;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int soc_usb_ep0_handle_feature(struct soc_usb *su,
		struct usb_ctrlrequest *ctrl, int set)
{
	u32			recip;
	int			ret;

	recip = ctrl->bRequestType & USB_RECIP_MASK;

	switch (recip) {
	case USB_RECIP_DEVICE:
		ret = soc_usb_ep0_handle_device(su, ctrl, set);
		break;
	case USB_RECIP_INTERFACE:
		ret = soc_usb_ep0_handle_intf(su, ctrl, set);
		break;
	case USB_RECIP_ENDPOINT:
		ret = soc_usb_ep0_handle_endpoint(su, ctrl, set);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int soc_usb_ep0_set_address(struct soc_usb *su, struct usb_ctrlrequest *ctrl)
{
	enum usb_device_state state = su->gadget->state;
	u32 addr;
	u32 reg;

	addr = le16_to_cpu(ctrl->wValue);
	if (addr > 127) {
		dev_err(su->dev, "invalid device address %d\n", addr);
		return -EINVAL;
	}

	if (state == USB_STATE_CONFIGURED) {
		dev_err(su->dev, "can't SetAddress() from Configured State\n");
		return -EINVAL;
	}

	reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
	reg &= ~(SOC_USB_DCFG_DEVADDR_MASK);
	reg |= SOC_USB_DCFG_DEVADDR(addr);
	soc_usb_writel(su->regs, SOC_USB_DCFG, reg);

	if (addr)
		usb_gadget_set_state(su->gadget, USB_STATE_ADDRESS);
	else
		usb_gadget_set_state(su->gadget, USB_STATE_DEFAULT);

	return 0;
}

static int soc_usb_ep0_delegate_req(struct soc_usb *su, struct usb_ctrlrequest *ctrl)
{
	int ret = -EINVAL;

	if (su->async_callbacks) {
		spin_unlock(&su->lock);
		ret = su->gadget_driver->setup(su->gadget, ctrl);
		spin_lock(&su->lock);
	}
	return ret;
}

static int soc_usb_ep0_set_config(struct soc_usb *su, struct usb_ctrlrequest *ctrl)
{
	enum usb_device_state state = su->gadget->state;
	u32 cfg;
	int ret;
	u32 reg;

	cfg = le16_to_cpu(ctrl->wValue);

	switch (state) {
	case USB_STATE_DEFAULT:
		return -EINVAL;

	case USB_STATE_ADDRESS:
		soc_usb_gadget_clear_tx_fifos(su);

		ret = soc_usb_ep0_delegate_req(su, ctrl);
		/* if the cfg matches and the cfg is non zero */
		if (cfg && (!ret || (ret == USB_GADGET_DELAYED_STATUS))) {

			/*
			 * only change state if set_config has already
			 * been processed. If gadget driver returns
			 * USB_GADGET_DELAYED_STATUS, we will wait
			 * to change the state on the next usb_ep_queue()
			 */
			if (ret == 0)
				usb_gadget_set_state(su->gadget,
						USB_STATE_CONFIGURED);

			/*
			 * Enable transition to U1/U2 state when
			 * nothing is pending from application.
			 */
			reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
			if (!su->dis_u1_entry_quirk)
				reg |= SOC_USB_DCTL_ACCEPTU1ENA;
			if (!su->dis_u2_entry_quirk)
				reg |= SOC_USB_DCTL_ACCEPTU2ENA;
			soc_usb_writel(su->regs, SOC_USB_DCTL, reg);
		}
		break;

	case USB_STATE_CONFIGURED:
		ret = soc_usb_ep0_delegate_req(su, ctrl);
		if (!cfg && !ret)
			usb_gadget_set_state(su->gadget,
					USB_STATE_ADDRESS);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static void soc_usb_ep0_set_sel_cmpl(struct usb_ep *ep, struct usb_request *req)
{
	struct soc_usb_ep	*dep = to_soc_usb_ep(ep);
	struct soc_usb	*su = dep->su;

	u32		param = 0;
	u32		reg;

	struct timing {
		u8	u1sel;
		u8	u1pel;
		__le16	u2sel;
		__le16	u2pel;
	} __packed timing;

	int		ret;

	memcpy(&timing, req->buf, sizeof(timing));

	su->u1sel = timing.u1sel;
	su->u1pel = timing.u1pel;
	su->u2sel = le16_to_cpu(timing.u2sel);
	su->u2pel = le16_to_cpu(timing.u2pel);

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	if (reg & SOC_USB_DCTL_INITU2ENA)
		param = su->u2pel;
	if (reg & SOC_USB_DCTL_INITU1ENA)
		param = su->u1pel;

	/*
	 * if parameter is greater than 125, a value of zero should be
	 * programmed in the register.
	 */
	if (param > 125)
		param = 0;

	/* now that we have the time, issue DGCMD Set Sel */
	ret = soc_usb_send_gadget_generic_command(su,
			SOC_USB_DGCMD_SET_PERIODIC_PAR, param);
	WARN_ON(ret < 0);
}

static int soc_usb_ep0_set_sel(struct soc_usb *su, struct usb_ctrlrequest *ctrl)
{
	struct soc_usb_ep	*dep;
	enum usb_device_state state = su->gadget->state;
	u16		wLength;

	if (state == USB_STATE_DEFAULT)
		return -EINVAL;

	wLength = le16_to_cpu(ctrl->wLength);

	if (wLength != 6) {
		dev_err(su->dev, "Set SEL should be 6 bytes, got %d\n",
				wLength);
		return -EINVAL;
	}

	/*
	 * To handle Set SEL we need to receive 6 bytes from Host. So let's
	 * queue a usb_request for 6 bytes.
	 *
	 * Remember, though, this controller can't handle non-wMaxPacketSize
	 * aligned transfers on the OUT direction, so we queue a request for
	 * wMaxPacketSize instead.
	 */
	dep = su->eps[0];
	su->ep0_usb_req.dep = dep;
	su->ep0_usb_req.request.length = dep->endpoint.maxpacket;
	su->ep0_usb_req.request.buf = su->setup_buf;
	su->ep0_usb_req.request.complete = soc_usb_ep0_set_sel_cmpl;

	return __soc_usb_gadget_ep0_queue(dep, &su->ep0_usb_req);
}

static int soc_usb_ep0_set_isoch_delay(struct soc_usb *su, struct usb_ctrlrequest *ctrl)
{
	u16		wLength;
	u16		wValue;
	u16		wIndex;

	wValue = le16_to_cpu(ctrl->wValue);
	wLength = le16_to_cpu(ctrl->wLength);
	wIndex = le16_to_cpu(ctrl->wIndex);

	if (wIndex || wLength)
		return -EINVAL;

	su->gadget->isoch_delay = wValue;

	return 0;
}

static int soc_usb_ep0_std_request(struct soc_usb *su, struct usb_ctrlrequest *ctrl)
{
	int ret;

	switch (ctrl->bRequest) {
	case USB_REQ_GET_STATUS:
		ret = soc_usb_ep0_handle_status(su, ctrl);
		break;
	case USB_REQ_CLEAR_FEATURE:
		ret = soc_usb_ep0_handle_feature(su, ctrl, 0);
		break;
	case USB_REQ_SET_FEATURE:
		ret = soc_usb_ep0_handle_feature(su, ctrl, 1);
		break;
	case USB_REQ_SET_ADDRESS:
		ret = soc_usb_ep0_set_address(su, ctrl);
		break;
	case USB_REQ_SET_CONFIGURATION:
		ret = soc_usb_ep0_set_config(su, ctrl);
		break;
	case USB_REQ_SET_SEL:
		ret = soc_usb_ep0_set_sel(su, ctrl);
		break;
	case USB_REQ_SET_ISOCH_DELAY:
		ret = soc_usb_ep0_set_isoch_delay(su, ctrl);
		break;
	default:
		ret = soc_usb_ep0_delegate_req(su, ctrl);
		break;
	}

	return ret;
}

static void soc_usb_ep0_inspect_setup(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	struct usb_ctrlrequest *ctrl = (void *) su->ep0_trb;
	int ret = -EINVAL;
	u32 len;

	if (!su->gadget_driver || !su->softconnect || !su->connected)
		goto out;

	trace_soc_usb_ctrl_req(ctrl);

	len = le16_to_cpu(ctrl->wLength);
	if (!len) {
		su->three_stage_setup = false;
		su->ep0_expect_in = false;
		su->ep0_next_event = SOC_USB_EP0_NRDY_STATUS;
	} else {
		su->three_stage_setup = true;
		su->ep0_expect_in = !!(ctrl->bRequestType & USB_DIR_IN);
		su->ep0_next_event = SOC_USB_EP0_NRDY_DATA;
	}

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
		ret = soc_usb_ep0_std_request(su, ctrl);
	else
		ret = soc_usb_ep0_delegate_req(su, ctrl);

	if (ret == USB_GADGET_DELAYED_STATUS)
		su->delayed_status = true;

out:
	if (ret < 0)
		soc_usb_ep0_stall_and_restart(su);
}

static void soc_usb_ep0_complete_data(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	struct soc_usb_request	*r;
	struct usb_request	*ur;
	struct soc_usb_trb		*trb;
	struct soc_usb_ep		*ep0;
	u32			transferred = 0;
	u32			status;
	u32			length;
	u8			epnum;

	epnum = event->endpoint_number;
	ep0 = su->eps[0];

	su->ep0_next_event = SOC_USB_EP0_NRDY_STATUS;
	trb = su->ep0_trb;
	trace_soc_usb_complete_trb(ep0, trb);

	r = next_request(&ep0->pending_list);
	if (!r)
		return;

	status = SOC_USB_TRB_SIZE_TRBSTS(trb->size);
	if (status == SOC_USB_TRBSTS_SETUP_PENDING) {
		su->setup_packet_pending = true;
		if (r)
			soc_usb_gadget_giveback(ep0, r, -ECONNRESET);

		return;
	}

	ur = &r->request;

	length = trb->size & SOC_USB_TRB_SIZE_MASK;
	transferred = ur->length - length;
	ur->actual += transferred;

	if ((IS_ALIGNED(ur->length, ep0->endpoint.maxpacket) &&
	     ur->length && ur->zero) || su->ep0_bounced) {
		trb++;
		trb->ctrl &= ~SOC_USB_TRB_CTRL_HWO;
		trace_soc_usb_complete_trb(ep0, trb);

		if (r->direction)
			su->eps[1]->trb_enqueue = 0;
		else
			su->eps[0]->trb_enqueue = 0;

		su->ep0_bounced = false;
	}

	if ((epnum & 1) && ur->actual < ur->length)
		soc_usb_ep0_stall_and_restart(su);
	else
		soc_usb_gadget_giveback(ep0, r, 0);
}

static void soc_usb_ep0_complete_status(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	struct soc_usb_request	*r;
	struct soc_usb_ep		*dep;
	struct soc_usb_trb		*trb;
	u32			status;

	dep = su->eps[0];
	trb = su->ep0_trb;

	trace_soc_usb_complete_trb(dep, trb);

	if (!list_empty(&dep->pending_list)) {
		r = next_request(&dep->pending_list);

		soc_usb_gadget_giveback(dep, r, 0);
	}

	if (su->test_mode) {
		int ret;

		ret = soc_usb_gadget_set_test_mode(su, su->test_mode_nr);
		if (ret < 0) {
			dev_err(su->dev, "invalid test #%d\n",
					su->test_mode_nr);
			soc_usb_ep0_stall_and_restart(su);
			return;
		}
	}

	status = SOC_USB_TRB_SIZE_TRBSTS(trb->size);
	if (status == SOC_USB_TRBSTS_SETUP_PENDING)
		su->setup_packet_pending = true;

	su->ep0state = EP0_SETUP_PHASE;
	soc_usb_ep0_out_start(su);
}

static void soc_usb_ep0_xfer_complete(struct soc_usb *su,
			const struct soc_usb_event_depevt *event)
{
	struct soc_usb_ep		*dep = su->eps[event->endpoint_number];

	dep->flags &= ~SOC_USB_EP_TRANSFER_STARTED;
	dep->resource_index = 0;
	su->setup_packet_pending = false;

	switch (su->ep0state) {
	case EP0_SETUP_PHASE:
		soc_usb_ep0_inspect_setup(su, event);
		break;

	case EP0_DATA_PHASE:
		soc_usb_ep0_complete_data(su, event);
		break;

	case EP0_STATUS_PHASE:
		soc_usb_ep0_complete_status(su, event);
		break;
	default:
		WARN(true, "UNKNOWN ep0state %d\n", su->ep0state);
	}
}

static void __soc_usb_ep0_do_control_data(struct soc_usb *su,
		struct soc_usb_ep *dep, struct soc_usb_request *req)
{
	unsigned int		trb_length = 0;
	int			ret;

	req->direction = !!dep->number;

	if (req->request.length == 0) {
		if (!req->direction)
			trb_length = dep->endpoint.maxpacket;

		soc_usb_ep0_prepare_one_trb(dep, su->bounce_addr, trb_length,
				SOC_USB_TRBCTL_CONTROL_DATA, false);
		ret = soc_usb_ep0_start_trans(dep);
	} else if (!IS_ALIGNED(req->request.length, dep->endpoint.maxpacket)
			&& (dep->number == 0)) {
		u32	maxpacket;
		u32	rem;

		ret = usb_gadget_map_request_by_dev(su->sysdev,
				&req->request, dep->number);
		if (ret)
			return;

		maxpacket = dep->endpoint.maxpacket;
		rem = req->request.length % maxpacket;
		su->ep0_bounced = true;

		/* prepare normal TRB */
		soc_usb_ep0_prepare_one_trb(dep, req->request.dma,
					 req->request.length,
					 SOC_USB_TRBCTL_CONTROL_DATA,
					 true);

		req->trb = &su->ep0_trb[dep->trb_enqueue - 1];

		/* Now prepare one extra TRB to align transfer size */
		soc_usb_ep0_prepare_one_trb(dep, su->bounce_addr,
					 maxpacket - rem,
					 SOC_USB_TRBCTL_CONTROL_DATA,
					 false);
		ret = soc_usb_ep0_start_trans(dep);
	} else if (IS_ALIGNED(req->request.length, dep->endpoint.maxpacket) &&
		   req->request.length && req->request.zero) {

		ret = usb_gadget_map_request_by_dev(su->sysdev,
				&req->request, dep->number);
		if (ret)
			return;

		/* prepare normal TRB */
		soc_usb_ep0_prepare_one_trb(dep, req->request.dma,
					 req->request.length,
					 SOC_USB_TRBCTL_CONTROL_DATA,
					 true);

		req->trb = &su->ep0_trb[dep->trb_enqueue - 1];

		if (!req->direction)
			trb_length = dep->endpoint.maxpacket;

		/* Now prepare one extra TRB to align transfer size */
		soc_usb_ep0_prepare_one_trb(dep, su->bounce_addr,
					 trb_length, SOC_USB_TRBCTL_CONTROL_DATA,
					 false);
		ret = soc_usb_ep0_start_trans(dep);
	} else {
		ret = usb_gadget_map_request_by_dev(su->sysdev,
				&req->request, dep->number);
		if (ret)
			return;

		soc_usb_ep0_prepare_one_trb(dep, req->request.dma,
				req->request.length, SOC_USB_TRBCTL_CONTROL_DATA,
				false);

		req->trb = &su->ep0_trb[dep->trb_enqueue];

		ret = soc_usb_ep0_start_trans(dep);
	}

	WARN_ON(ret < 0);
}

static int soc_usb_ep0_start_control_status(struct soc_usb_ep *dep)
{
	struct soc_usb		*su = dep->su;
	u32			type;

	type = su->three_stage_setup ? SOC_USB_TRBCTL_CONTROL_STATUS3
		: SOC_USB_TRBCTL_CONTROL_STATUS2;

	soc_usb_ep0_prepare_one_trb(dep, su->ep0_trb_addr, 0, type, false);
	return soc_usb_ep0_start_trans(dep);
}

static void __soc_usb_ep0_do_control_status(struct soc_usb *su, struct soc_usb_ep *dep)
{
	WARN_ON(soc_usb_ep0_start_control_status(dep));
}

static void soc_usb_ep0_do_control_status(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	struct soc_usb_ep		*dep = su->eps[event->endpoint_number];

	__soc_usb_ep0_do_control_status(su, dep);
}

void soc_usb_ep0_send_delayed_status(struct soc_usb *su)
{
	unsigned int direction = !su->ep0_expect_in;

	su->delayed_status = false;
	su->clear_stall_protocol = 0;

	if (su->ep0state != EP0_STATUS_PHASE)
		return;

	__soc_usb_ep0_do_control_status(su, su->eps[direction]);
}

void soc_usb_ep0_end_control_data(struct soc_usb *su, struct soc_usb_ep *dep)
{
	struct soc_usb_gadget_ep_cmd_params params;
	u32			cmd;
	int			ret;

	/*
	 * For status/DATA OUT stage, TRB will be queued on ep0 out
	 * endpoint for which resource index is zero. Hence allow
	 * queuing ENDXFER command for ep0 out endpoint.
	 */
	if (!dep->resource_index && dep->number)
		return;

	cmd = SOC_USB_DEPCMD_ENDTRANSFER;
	cmd |= SOC_USB_DEPCMD_CMDIOC;
	cmd |= SOC_USB_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
	ret = soc_usb_send_gadget_ep_cmd(dep, cmd, &params);
	WARN_ON_ONCE(ret);
	dep->resource_index = 0;
}

static void soc_usb_ep0_xfernotready(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	switch (event->status) {
	case DEPEVT_STATUS_CONTROL_DATA:
		if (!su->softconnect || !su->connected)
			return;
		/*
		 * We already have a DATA transfer in the controller's cache,
		 * if we receive a XferNotReady(DATA) we will ignore it, unless
		 * it's for the wrong direction.
		 *
		 * In that case, we must issue END_TRANSFER command to the Data
		 * Phase we already have started and issue SetStall on the
		 * control endpoint.
		 */
		if (su->ep0_expect_in != event->endpoint_number) {
			struct soc_usb_ep	*dep = su->eps[su->ep0_expect_in];

			dev_err(su->dev, "unexpected direction for Data Phase\n");
			soc_usb_ep0_end_control_data(su, dep);
			soc_usb_ep0_stall_and_restart(su);
			return;
		}

		break;

	case DEPEVT_STATUS_CONTROL_STATUS:
		if (su->ep0_next_event != SOC_USB_EP0_NRDY_STATUS)
			return;

		if (su->setup_packet_pending) {
			soc_usb_ep0_stall_and_restart(su);
			return;
		}

		su->ep0state = EP0_STATUS_PHASE;

		if (su->delayed_status) {
			struct soc_usb_ep *dep = su->eps[0];

			WARN_ON_ONCE(event->endpoint_number != 1);
			/*
			 * We should handle the delay STATUS phase here if the
			 * request for handling delay STATUS has been queued
			 * into the list.
			 */
			if (!list_empty(&dep->pending_list)) {
				su->delayed_status = false;
				usb_gadget_set_state(su->gadget,
						     USB_STATE_CONFIGURED);
				soc_usb_ep0_do_control_status(su, event);
			}

			return;
		}

		soc_usb_ep0_do_control_status(su, event);
	}
}

void soc_usb_ep0_interrupt(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	struct soc_usb_ep	*dep = su->eps[event->endpoint_number];
	u8		cmd;

	switch (event->endpoint_event) {
	case SOC_USB_DEPEVT_XFERCOMPLETE:
		soc_usb_ep0_xfer_complete(su, event);
		break;

	case SOC_USB_DEPEVT_XFERNOTREADY:
		soc_usb_ep0_xfernotready(su, event);
		break;

	case SOC_USB_DEPEVT_XFERINPROGRESS:
	case SOC_USB_DEPEVT_RXTXFIFOEVT:
	case SOC_USB_DEPEVT_STREAMEVT:
		break;
	case SOC_USB_DEPEVT_EPCMDCMPLT:
		cmd = DEPEVT_PARAMETER_CMD(event->parameters);

		if (cmd == SOC_USB_DEPCMD_ENDTRANSFER) {
			dep->flags &= ~SOC_USB_EP_END_TRANSFER_PENDING;
			dep->flags &= ~SOC_USB_EP_TRANSFER_STARTED;
		}
		break;
	default:
		dev_err(su->dev, "unknown endpoint event %d\n", event->endpoint_event);
		break;
	}
}
