// SPDX-License-Identifier: GPL-2.0
/*
 * soc_gadget.c - SOC USB3 DRD Controller Gadget Framework Link
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
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

#include "soc_debug.h"
#include "soc_core.h"
#include "soc_gadget.h"
#include "soc_io.h"


#define SOC_USB_ALIGN_FRAME(d, n)	(((d)->frame_number + ((d)->interval * (n))) \
					& ~((d)->interval - 1))

/**
 * soc_usb_gadget_set_test_mode - enables usb2 test modes
 * @su: pointer to our context structure
 * @mode: the mode to set (J, K SE0 NAK, Force Enable)
 *
 * Caller should take care of locking. This function will return 0 on
 * success or -EINVAL if wrong Test Selector is passed.
 */
int soc_usb_gadget_set_test_mode(struct soc_usb *su, int mode)
{
	u32		reg;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	reg &= ~SOC_USB_DCTL_TSTCTRL_MASK;

	switch (mode) {
	case USB_TEST_J:
	case USB_TEST_K:
	case USB_TEST_SE0_NAK:
	case USB_TEST_PACKET:
	case USB_TEST_FORCE_ENABLE:
		reg |= mode << 1;
		break;
	default:
		return -EINVAL;
	}

	soc_usb_gadget_dctl_write_safe(su, reg);

	return 0;
}

/**
 * soc_usb_gadget_get_link_state - gets current state of usb link
 * @su: pointer to our context structure
 *
 * Caller should take care of locking. This function will
 * return the link state on success (>= 0) or -ETIMEDOUT.
 */
int soc_usb_gadget_get_link_state(struct soc_usb *su)
{
	u32		reg;

	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);

	return SOC_USB_DSTS_USBLNKST(reg);
}

/**
 * soc_usb_gadget_set_link_state - sets usb link to a particular state
 * @su: pointer to our context structure
 * @state: the state to put link into
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -ETIMEDOUT.
 */
int soc_usb_gadget_set_link_state(struct soc_usb *su, enum soc_usb_link_state state)
{
	u32		reg;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	reg &= ~SOC_USB_DCTL_ULSTCHNGREQ_MASK;

	/* set no action before sending new link state change */
	soc_usb_writel(su->regs, SOC_USB_DCTL, reg);

	/* set requested state */
	reg |= SOC_USB_DCTL_ULSTCHNGREQ(state);
	soc_usb_writel(su->regs, SOC_USB_DCTL, reg);

	return 0;
}

static void soc_usb_ep0_reset_state(struct soc_usb *su)
{
	unsigned int	dir;

	if (su->ep0state != EP0_SETUP_PHASE) {
		dir = !!su->ep0_expect_in;
		if (su->ep0state == EP0_DATA_PHASE)
			soc_usb_ep0_end_control_data(su, su->eps[dir]);
		else
			soc_usb_ep0_end_control_data(su, su->eps[!dir]);

		su->eps[0]->trb_enqueue = 0;
		su->eps[1]->trb_enqueue = 0;

		soc_usb_ep0_stall_and_restart(su);
	}
}

/**
 * soc_usb_ep_inc_trb - increment a trb index.
 * @index: Pointer to the TRB index to increment.
 *
 * The index should never point to the link TRB. After incrementing,
 * if it is point to the link TRB, wrap around to the beginning. The
 * link TRB is always at the last TRB entry.
 */
static void soc_usb_ep_inc_trb(u8 *index)
{
	(*index)++;
	if (*index == (SOC_USB_TRB_NUM - 1))
		*index = 0;
}

/**
 * soc_usb_ep_inc_enq - increment endpoint's enqueue pointer
 * @dep: The endpoint whose enqueue pointer we're incrementing
 */
static void soc_usb_ep_inc_enq(struct soc_usb_ep *dep)
{
	soc_usb_ep_inc_trb(&dep->trb_enqueue);
}

/**
 * soc_usb_ep_inc_deq - increment endpoint's dequeue pointer
 * @dep: The endpoint whose enqueue pointer we're incrementing
 */
static void soc_usb_ep_inc_deq(struct soc_usb_ep *dep)
{
	soc_usb_ep_inc_trb(&dep->trb_dequeue);
}

static void soc_usb_gadget_del_and_unmap_request(struct soc_usb_ep *dep,
		struct soc_usb_request *req, int status)
{
	struct soc_usb			*su = dep->su;

	list_del(&req->list);
	req->remaining = 0;
	req->needs_extra_trb = false;
	req->num_trbs = 0;

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	if (req->trb)
		usb_gadget_unmap_request_by_dev(su->sysdev,
				&req->request, req->direction);

	req->trb = NULL;
	trace_soc_usb_gadget_giveback(req);

	if (dep->number > 1)
		pm_runtime_put(su->dev);
}

/**
 * soc_usb_gadget_giveback - call struct usb_request's ->complete callback
 * @dep: The endpoint to whom the request belongs to
 * @req: The request we're giving back
 * @status: completion code for the request
 *
 * Must be called with controller's lock held and interrupts disabled. This
 * function will unmap @req and call its ->complete() callback to notify upper
 * layers that it has completed.
 */
void soc_usb_gadget_giveback(struct soc_usb_ep *dep, struct soc_usb_request *req,
		int status)
{
	struct soc_usb			*su = dep->su;

	soc_usb_gadget_del_and_unmap_request(dep, req, status);
	req->status = SOC_USB_REQUEST_STATUS_COMPLETED;

	spin_unlock(&su->lock);
	usb_gadget_giveback_request(&dep->endpoint, &req->request);
	spin_lock(&su->lock);
}

/**
 * soc_usb_send_gadget_generic_command - issue a generic command for the controller
 * @su: pointer to the controller context
 * @cmd: the command to be issued
 * @param: command parameter
 *
 * Caller should take care of locking. Issue @cmd with a given @param to @su
 * and wait for its completion.
 */
int soc_usb_send_gadget_generic_command(struct soc_usb *su, unsigned int cmd,
		u32 param)
{
	u32		timeout = 500;
	int		status = 0;
	int		ret = 0;
	u32		reg;

	soc_usb_writel(su->regs, SOC_USB_DGCMDPAR, param);
	soc_usb_writel(su->regs, SOC_USB_DGCMD, cmd | SOC_USB_DGCMD_CMDACT);

	do {
		reg = soc_usb_readl(su->regs, SOC_USB_DGCMD);
		if (!(reg & SOC_USB_DGCMD_CMDACT)) {
			status = SOC_USB_DGCMD_STATUS(reg);
			if (status)
				ret = -EINVAL;
			break;
		}
	} while (--timeout);

	if (!timeout) {
		ret = -ETIMEDOUT;
		status = -ETIMEDOUT;
	}

	trace_soc_usb_gadget_generic_cmd(cmd, param, status);

	return ret;
}

static int __soc_usb_gadget_wakeup(struct soc_usb *su, bool async);

/**
 * soc_usb_send_gadget_ep_cmd - issue an endpoint command
 * @dep: the endpoint to which the command is going to be issued
 * @cmd: the command to be issued
 * @params: parameters to the command
 *
 * Caller should handle locking. This function will issue @cmd with given
 * @params to @dep and wait for its completion.
 *
 * According to the programming guide, if the link state is in L1/L2/U3,
 * then sending the Start Transfer command may not complete. The
 * programming guide suggested to bring the link state back to ON/U0 by
 * performing remote wakeup prior to sending the command. However, don't
 * initiate remote wakeup when the user/function does not send wakeup
 * request via wakeup ops. Send the command when it's allowed.
 *
 * Notes:
 * For L1 link state, issuing a command requires the clearing of
 * GUSB2PHYCFG.SUSPENDUSB2, which turns on the signal required to complete
 * the given command (usually within 50us). This should happen within the
 * command timeout set by driver. No additional step is needed.
 *
 * For L2 or U3 link state, the gadget is in USB suspend. Care should be
 * taken when sending Start Transfer command to ensure that it's done after
 * USB resume.
 */
int soc_usb_send_gadget_ep_cmd(struct soc_usb_ep *dep, unsigned int cmd,
		struct soc_usb_gadget_ep_cmd_params *params)
{
	const struct usb_endpoint_descriptor *desc = dep->endpoint.desc;
	struct soc_usb		*su = dep->su;
	u32			timeout = 5000;
	u32			saved_config = 0;
	u32			reg;

	int			cmd_status = 0;
	int			ret = -EINVAL;

	/*
	 * When operating in USB 2.0 speeds (HS/FS), if GUSB2PHYCFG.ENBLSLPM or
	 * GUSB2PHYCFG.SUSPHY is set, it must be cleared before issuing an
	 * endpoint command.
	 *
	 * Save and clear both GUSB2PHYCFG.ENBLSLPM and GUSB2PHYCFG.SUSPHY
	 * settings. Restore them after the command is completed.
	 *
	 */
	if (su->gadget->speed <= USB_SPEED_HIGH ||
	    SOC_USB_DEPCMD_CMD(cmd) == SOC_USB_DEPCMD_ENDTRANSFER) {
		reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));
		if (unlikely(reg & SOC_USB_GUSB2PHYCFG_SUSPHY)) {
			saved_config |= SOC_USB_GUSB2PHYCFG_SUSPHY;
			reg &= ~SOC_USB_GUSB2PHYCFG_SUSPHY;
		}

		if (saved_config)
			soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);
	}

	/*
	 * For some commands such as Update Transfer command, DEPCMDPARn
	 * registers are reserved. Since the driver often sends Update Transfer
	 * command, don't write to DEPCMDPARn to avoid register write delays and
	 * improve performance.
	 */
	if (SOC_USB_DEPCMD_CMD(cmd) != SOC_USB_DEPCMD_UPDATETRANSFER) {
		soc_usb_writel(dep->regs, SOC_USB_DEPCMDPAR0, params->param0);
		soc_usb_writel(dep->regs, SOC_USB_DEPCMDPAR1, params->param1);
		soc_usb_writel(dep->regs, SOC_USB_DEPCMDPAR2, params->param2);
	}

	/*
	 * if we're not relying on XferNotReady, we can make use of a special "No
	 * Response Update Transfer" command where we should clear both CmdAct
	 * and CmdIOC bits.
	 *
	 * With this, we don't need to wait for command completion and can
	 * straight away issue further commands to the endpoint.
	 *
	 * NOTICE: We're making an assumption that control endpoints will never
	 * make use of Update Transfer command. This is a safe assumption
	 * because we can never have more than one request at a time with
	 * Control Endpoints. If anybody changes that assumption, this chunk
	 * needs to be updated accordingly.
	 */
	if (SOC_USB_DEPCMD_CMD(cmd) == SOC_USB_DEPCMD_UPDATETRANSFER &&
			!usb_endpoint_xfer_isoc(desc))
		cmd &= ~(SOC_USB_DEPCMD_CMDIOC | SOC_USB_DEPCMD_CMDACT);
	else
		cmd |= SOC_USB_DEPCMD_CMDACT;

	soc_usb_writel(dep->regs, SOC_USB_DEPCMD, cmd);

	if (!(cmd & SOC_USB_DEPCMD_CMDACT) ||
		(SOC_USB_DEPCMD_CMD(cmd) == SOC_USB_DEPCMD_ENDTRANSFER &&
		!(cmd & SOC_USB_DEPCMD_CMDIOC))) {
		ret = 0;
		goto skip_status;
	}

	do {
		reg = soc_usb_readl(dep->regs, SOC_USB_DEPCMD);
		if (!(reg & SOC_USB_DEPCMD_CMDACT)) {
			cmd_status = SOC_USB_DEPCMD_STATUS(reg);

			switch (cmd_status) {
			case 0:
				ret = 0;
				break;
			case DEPEVT_TRANSFER_NO_RESOURCE:
				dev_WARN(su->dev, "No resource for %s\n",
					 dep->name);
				ret = -EINVAL;
				break;
			case DEPEVT_TRANSFER_BUS_EXPIRY:
				/*
				 * SW issues START TRANSFER command to
				 * isochronous ep with future frame interval. If
				 * future interval time has already passed when
				 * core receives the command, it will respond
				 * with an error status of 'Bus Expiry'.
				 *
				 * Instead of always returning -EINVAL, let's
				 * give a hint to the gadget driver that this is
				 * the case by returning -EAGAIN.
				 */
				ret = -EAGAIN;
				break;
			default:
				dev_WARN(su->dev, "UNKNOWN cmd status\n");
			}

			break;
		}
	} while (--timeout);

	if (timeout == 0) {
		ret = -ETIMEDOUT;
		cmd_status = -ETIMEDOUT;
	}

skip_status:
	trace_soc_usb_gadget_ep_cmd(dep, cmd, params, cmd_status);

	if (SOC_USB_DEPCMD_CMD(cmd) == SOC_USB_DEPCMD_STARTTRANSFER) {
		if (ret == 0)
			dep->flags |= SOC_USB_EP_TRANSFER_STARTED;

		if (ret != -ETIMEDOUT)
			soc_usb_gadget_ep_get_transfer_index(dep);
	}

	if (saved_config) {
		reg = soc_usb_readl(su->regs, SOC_USB_GUSB2PHYCFG(0));
		reg |= saved_config;
		soc_usb_writel(su->regs, SOC_USB_GUSB2PHYCFG(0), reg);
	}

	return ret;
}

static int soc_usb_send_clear_stall_ep_cmd(struct soc_usb_ep *dep)
{
	struct soc_usb *su = dep->su;
	struct soc_usb_gadget_ep_cmd_params params;
	u32 cmd = SOC_USB_DEPCMD_CLEARSTALL;

	/*
	 * As of core revision 2.60a the recommended programming model
	 * is to set the ClearPendIN bit when issuing a Clear Stall EP
	 * command for IN endpoints. This is to prevent an issue where
	 * some (non-compliant) hosts may not send ACK TPs for pending
	 * IN transfers due to a mishandled error condition.
	 */
	if (dep->direction &&
	    (su->gadget->speed >= USB_SPEED_SUPER))
		cmd |= SOC_USB_DEPCMD_CLEARPENDIN;

	memset(&params, 0, sizeof(params));

	return soc_usb_send_gadget_ep_cmd(dep, cmd, &params);
}

static dma_addr_t soc_usb_trb_dma_offset(struct soc_usb_ep *dep,
		struct soc_usb_trb *trb)
{
	u32		offset = (char *) trb - (char *) dep->trb_pool;

	return dep->trb_pool_dma + offset;
}

static int soc_usb_alloc_trb_pool(struct soc_usb_ep *dep)
{
	struct soc_usb		*su = dep->su;

	if (dep->trb_pool)
		return 0;

	dep->trb_pool = dma_alloc_coherent(su->sysdev,
			sizeof(struct soc_usb_trb) * SOC_USB_TRB_NUM,
			&dep->trb_pool_dma, GFP_KERNEL);
	if (!dep->trb_pool) {
		dev_err(dep->su->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	return 0;
}

static void soc_usb_free_trb_pool(struct soc_usb_ep *dep)
{
	struct soc_usb		*su = dep->su;

	dma_free_coherent(su->sysdev, sizeof(struct soc_usb_trb) * SOC_USB_TRB_NUM,
			dep->trb_pool, dep->trb_pool_dma);

	dep->trb_pool = NULL;
	dep->trb_pool_dma = 0;
}

static int soc_usb_gadget_set_xfer_resource(struct soc_usb_ep *dep)
{
	struct soc_usb_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = SOC_USB_DEPXFERCFG_NUM_XFER_RES(1);

	return soc_usb_send_gadget_ep_cmd(dep, SOC_USB_DEPCMD_SETTRANSFRESOURCE,
			&params);
}

/**
 * soc_usb_gadget_start_config - configure ep resources
 * @dep: endpoint that is being enabled
 *
 * Issue a %SOC_USB_DEPCMD_DEPSTARTCFG command to @dep. After the command's
 * completion, it will set Transfer Resource for all available endpoints.
 *
 * The assignment of transfer resources cannot perfectly follow the data book
 * due to the fact that the controller driver does not have all knowledge of the
 * configuration in advance. It is given this information piecemeal by the
 * composite gadget framework after every SET_CONFIGURATION and
 * SET_INTERFACE. Trying to follow the databook programming model in this
 * scenario can cause errors. For two reasons:
 *
 * 1) The databook says to do %SOC_USB_DEPCMD_DEPSTARTCFG for every
 * %USB_REQ_SET_CONFIGURATION and %USB_REQ_SET_INTERFACE (8.1.5). This is
 * incorrect in the scenario of multiple interfaces.
 *
 * 2) The databook does not mention doing more %SOC_USB_DEPCMD_DEPXFERCFG for new
 * endpoint on alt setting (8.1.6).
 *
 * The following simplified method is used instead:
 *
 * All hardware endpoints can be assigned a transfer resource and this setting
 * will stay persistent until either a core reset or hibernation. So whenever we
 * do a %SOC_USB_DEPCMD_DEPSTARTCFG(0) we can go ahead and do
 * %SOC_USB_DEPCMD_DEPXFERCFG for every hardware endpoint as well. We are
 * guaranteed that there are as many transfer resources as endpoints.
 *
 * This function is called for each endpoint when it is being enabled but is
 * triggered only when called for EP0-out, which always happens first, and which
 * should only happen in one of the above conditions.
 */
static int soc_usb_gadget_start_config(struct soc_usb_ep *dep)
{
	struct soc_usb_gadget_ep_cmd_params params;
	struct soc_usb		*su;
	u32			cmd;
	int			i;
	int			ret;

	if (dep->number)
		return 0;

	memset(&params, 0x00, sizeof(params));
	cmd = SOC_USB_DEPCMD_DEPSTARTCFG;
	su = dep->su;

	ret = soc_usb_send_gadget_ep_cmd(dep, cmd, &params);
	if (ret)
		return ret;

	for (i = 0; i < SOC_USB_ENDPOINTS_NUM; i++) {
		struct soc_usb_ep *dep = su->eps[i];

		if (!dep)
			continue;

		ret = soc_usb_gadget_set_xfer_resource(dep);
		if (ret)
			return ret;
	}

	return 0;
}

static int soc_usb_gadget_set_ep_config(struct soc_usb_ep *dep, unsigned int action)
{
	const struct usb_ss_ep_comp_descriptor *comp_desc;
	const struct usb_endpoint_descriptor *desc;
	struct soc_usb_gadget_ep_cmd_params params;
	struct soc_usb *su = dep->su;

	comp_desc = dep->endpoint.comp_desc;
	desc = dep->endpoint.desc;

	memset(&params, 0x00, sizeof(params));

	params.param0 = SOC_USB_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| SOC_USB_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	/* Burst size is only needed in SuperSpeed mode */
	if (su->gadget->speed >= USB_SPEED_SUPER) {
		u32 burst = dep->endpoint.maxburst;

		params.param0 |= SOC_USB_DEPCFG_BURST_SIZE(burst - 1);
	}

	params.param0 |= action;
	if (action == SOC_USB_DEPCFG_ACTION_RESTORE)
		params.param2 |= dep->saved_state;

	if (usb_endpoint_xfer_control(desc))
		params.param1 = SOC_USB_DEPCFG_XFER_COMPLETE_EN;

	if (dep->number <= 1 || usb_endpoint_xfer_isoc(desc))
		params.param1 |= SOC_USB_DEPCFG_XFER_NOT_READY_EN;

	if (usb_ss_max_streams(comp_desc) && usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= SOC_USB_DEPCFG_STREAM_CAPABLE
			| SOC_USB_DEPCFG_XFER_COMPLETE_EN
			| SOC_USB_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}

	if (!usb_endpoint_xfer_control(desc))
		params.param1 |= SOC_USB_DEPCFG_XFER_IN_PROGRESS_EN;

	/*
	 * We are doing 1:1 mapping for endpoints, meaning
	 * Physical Endpoints 2 maps to Logical Endpoint 2 and
	 * so on. We consider the direction bit as part of the physical
	 * endpoint number. So USB endpoint 0x81 is 0x03.
	 */
	params.param1 |= SOC_USB_DEPCFG_EP_NUMBER(dep->number);

	/*
	 * We must use the lower 16 TX FIFOs even though
	 * HW might have more
	 */
	if (dep->direction)
		params.param0 |= SOC_USB_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	if (desc->bInterval) {
		u8 bInterval_m1;

		/*
		 * Valid range for DEPCFG.bInterval_m1 is from 0 to 13.
		 *
		 * NOTE: The programming guide incorrectly stated bInterval_m1
		 * must be set to 0 when operating in fullspeed. Internally the
		 * controller does not have this limitation.
		 */
		bInterval_m1 = min_t(u8, desc->bInterval - 1, 13);

		if (usb_endpoint_type(desc) == USB_ENDPOINT_XFER_INT &&
		    su->gadget->speed == USB_SPEED_FULL)
			dep->interval = desc->bInterval;
		else
			dep->interval = 1 << (desc->bInterval - 1);

		params.param1 |= SOC_USB_DEPCFG_BINTERVAL_M1(bInterval_m1);
	}

	return soc_usb_send_gadget_ep_cmd(dep, SOC_USB_DEPCMD_SETEPCONFIG, &params);
}

/**
 * soc_usb_gadget_calc_tx_fifo_size - calculates the txfifo size value
 * @su: pointer to the SOC_USB context
 * @mult: multiplier to be used when calculating the fifo_size
 *
 * Calculates the size value based on the equation below:
 *
 * SOC_USB revision 280A and prior:
 * fifo_size = mult * (max_packet / mdwidth) + 1;
 *
 * SOC_USB revision 290A and onwards:
 * fifo_size = mult * ((max_packet + mdwidth)/mdwidth + 1) + 1
 *
 * The max packet size is set to 1024, as the txfifo requirements mainly apply
 * to super speed USB use cases.  However, it is safe to overestimate the fifo
 * allocations for other scenarios, i.e. high speed USB.
 */
static int soc_usb_gadget_calc_tx_fifo_size(struct soc_usb *su, int mult)
{
	int max_packet = 1024;
	int fifo_size;
	int mdwidth;

	mdwidth = soc_usb_mdwidth(su);

	/* MDWIDTH is represented in bits, we need it in bytes */
	mdwidth >>= 3;

	fifo_size = mult * ((max_packet + mdwidth) / mdwidth) + 1;
	return fifo_size;
}

/**
 * soc_usb_gadget_clear_tx_fifos - Clears txfifo allocation
 * @su: pointer to the SOC_USB context
 *
 * Iterates through all the endpoint registers and clears the previous txfifo
 * allocations.
 */
void soc_usb_gadget_clear_tx_fifos(struct soc_usb *su)
{
	struct soc_usb_ep *dep;
	int fifo_depth;
	int size;
	int num;

	if (!su->do_fifo_resize)
		return;

	/* Read ep0IN related TXFIFO size */
	dep = su->eps[1];
	size = soc_usb_readl(su->regs, SOC_USB_GTXFIFOSIZ(0));
	if (SOC_USB_IP_IS(SOC_USB))
		fifo_depth = SOC_USB_GTXFIFOSIZ_TXFDEP(size);
	else
		fifo_depth = SOC_USB32_GTXFIFOSIZ_TXFDEP(size);

	su->last_fifo_depth = fifo_depth;
	/* Clear existing TXFIFO for all IN eps except ep0 */
	for (num = 3; num < min_t(int, su->num_eps, SOC_USB_ENDPOINTS_NUM);
	     num += 2) {
		dep = su->eps[num];
		/* Don't change TXFRAMNUM on usb31 version */
		size = SOC_USB_IP_IS(SOC_USB) ? 0 :
			soc_usb_readl(su->regs, SOC_USB_GTXFIFOSIZ(num >> 1)) &
				   SOC_USB32_GTXFIFOSIZ_TXFRAMNUM;

		soc_usb_writel(su->regs, SOC_USB_GTXFIFOSIZ(num >> 1), size);
		dep->flags &= ~SOC_USB_EP_TXFIFO_RESIZED;
	}
	su->num_ep_resized = 0;
}

/*
 * soc_usb_gadget_resize_tx_fifos - reallocate fifo spaces for current use-case
 * @su: pointer to our context structure
 *
 * This function will a best effort FIFO allocation in order
 * to improve FIFO usage and throughput, while still allowing
 * us to enable as many endpoints as possible.
 *
 * Keep in mind that this operation will be highly dependent
 * on the configured size for RAM1 - which contains TxFifo -,
 * the amount of endpoints enabled on coreConsultant tool, and
 * the width of the Master Bus.
 *
 * In general, FIFO depths are represented with the following equation:
 *
 * fifo_size = mult * ((max_packet + mdwidth)/mdwidth + 1) + 1
 *
 * In conjunction with soc_usb_gadget_check_config(), this resizing logic will
 * ensure that all endpoints will have enough internal memory for one max
 * packet per endpoint.
 */
static int soc_usb_gadget_resize_tx_fifos(struct soc_usb_ep *dep)
{
	struct soc_usb *su = dep->su;
	int fifo_0_start;
	int ram1_depth;
	int fifo_size;
	int min_depth;
	int num_in_ep;
	int remaining;
	int num_fifos = 1;
	int fifo;
	int tmp;

	if (!su->do_fifo_resize)
		return 0;

	/* resize IN endpoints except ep0 */
	if (!usb_endpoint_dir_in(dep->endpoint.desc) || dep->number <= 1)
		return 0;

	/* bail if already resized */
	if (dep->flags & SOC_USB_EP_TXFIFO_RESIZED)
		return 0;

	ram1_depth = SOC_USB_RAM1_DEPTH(su->hwparams.hwparams7);

	if ((dep->endpoint.maxburst > 1 &&
	     usb_endpoint_xfer_bulk(dep->endpoint.desc)) ||
	    usb_endpoint_xfer_isoc(dep->endpoint.desc))
		num_fifos = 3;

	if (dep->endpoint.maxburst > 6 &&
	    (usb_endpoint_xfer_bulk(dep->endpoint.desc) ||
	     usb_endpoint_xfer_isoc(dep->endpoint.desc)) && SOC_USB_IP_IS(SOC_USB32))
		num_fifos = su->tx_fifo_resize_max_num;

	/* FIFO size for a single buffer */
	fifo = soc_usb_gadget_calc_tx_fifo_size(su, 1);

	/* Calculate the number of remaining EPs w/o any FIFO */
	num_in_ep = su->max_cfg_eps;
	num_in_ep -= su->num_ep_resized;

	/* Reserve at least one FIFO for the number of IN EPs */
	min_depth = num_in_ep * (fifo + 1);
	remaining = ram1_depth - min_depth - su->last_fifo_depth;
	remaining = max_t(int, 0, remaining);
	/*
	 * We've already reserved 1 FIFO per EP, so check what we can fit in
	 * addition to it.  If there is not enough remaining space, allocate
	 * all the remaining space to the EP.
	 */
	fifo_size = (num_fifos - 1) * fifo;
	if (remaining < fifo_size)
		fifo_size = remaining;

	fifo_size += fifo;
	/* Last increment according to the TX FIFO size equation */
	fifo_size++;

	/* Check if TXFIFOs start at non-zero addr */
	tmp = soc_usb_readl(su->regs, SOC_USB_GTXFIFOSIZ(0));
	fifo_0_start = SOC_USB_GTXFIFOSIZ_TXFSTADDR(tmp);

	fifo_size |= (fifo_0_start + (su->last_fifo_depth << 16));
	if (SOC_USB_IP_IS(SOC_USB))
		su->last_fifo_depth += SOC_USB_GTXFIFOSIZ_TXFDEP(fifo_size);
	else
		su->last_fifo_depth += SOC_USB32_GTXFIFOSIZ_TXFDEP(fifo_size);

	/* Check fifo size allocation doesn't exceed available RAM size. */
	if (su->last_fifo_depth >= ram1_depth) {
		dev_err(su->dev, "Fifosize(%d) > RAM size(%d) %s depth:%d\n",
			su->last_fifo_depth, ram1_depth,
			dep->endpoint.name, fifo_size);
		if (SOC_USB_IP_IS(SOC_USB))
			fifo_size = SOC_USB_GTXFIFOSIZ_TXFDEP(fifo_size);
		else
			fifo_size = SOC_USB32_GTXFIFOSIZ_TXFDEP(fifo_size);

		su->last_fifo_depth -= fifo_size;
		return -ENOMEM;
	}

	soc_usb_writel(su->regs, SOC_USB_GTXFIFOSIZ(dep->number >> 1), fifo_size);
	dep->flags |= SOC_USB_EP_TXFIFO_RESIZED;
	su->num_ep_resized++;

	return 0;
}

/**
 * __soc_usb_gadget_ep_enable - initializes a hw endpoint
 * @dep: endpoint to be initialized
 * @action: one of INIT, MODIFY or RESTORE
 *
 * Caller should take care of locking. Execute all necessary commands to
 * initialize a HW endpoint so it can be used by a gadget driver.
 */
static int __soc_usb_gadget_ep_enable(struct soc_usb_ep *dep, unsigned int action)
{
	const struct usb_endpoint_descriptor *desc = dep->endpoint.desc;
	struct soc_usb		*su = dep->su;

	u32			reg;
	int			ret;

	if (!(dep->flags & SOC_USB_EP_ENABLED)) {
		ret = soc_usb_gadget_resize_tx_fifos(dep);
		if (ret)
			return ret;

		ret = soc_usb_gadget_start_config(dep);
		if (ret)
			return ret;
	}

	ret = soc_usb_gadget_set_ep_config(dep, action);
	if (ret)
		return ret;

	if (!(dep->flags & SOC_USB_EP_ENABLED)) {
		struct soc_usb_trb	*trb_st_hw;
		struct soc_usb_trb	*trb_link;

		dep->type = usb_endpoint_type(desc);
		dep->flags |= SOC_USB_EP_ENABLED;

		reg = soc_usb_readl(su->regs, SOC_USB_DALEPENA);
		reg |= SOC_USB_DALEPENA_EP(dep->number);
		soc_usb_writel(su->regs, SOC_USB_DALEPENA, reg);

		dep->trb_dequeue = 0;
		dep->trb_enqueue = 0;

		if (usb_endpoint_xfer_control(desc))
			goto out;

		/* Initialize the TRB ring */
		memset(dep->trb_pool, 0,
		       sizeof(struct soc_usb_trb) * SOC_USB_TRB_NUM);

		/* Link TRB. The HWO bit is never reset */
		trb_st_hw = &dep->trb_pool[0];

		trb_link = &dep->trb_pool[SOC_USB_TRB_NUM - 1];
		trb_link->bpl = lower_32_bits(soc_usb_trb_dma_offset(dep, trb_st_hw));
		trb_link->bph = upper_32_bits(soc_usb_trb_dma_offset(dep, trb_st_hw));
		trb_link->ctrl |= SOC_USB_TRBCTL_LINK_TRB;
		trb_link->ctrl |= SOC_USB_TRB_CTRL_HWO;
	}

	/*
	 * Issue StartTransfer here with no-op TRB so we can always rely on No
	 * Response Update Transfer command.
	 */
	if (usb_endpoint_xfer_bulk(desc) ||
			usb_endpoint_xfer_int(desc)) {
		struct soc_usb_gadget_ep_cmd_params params;
		struct soc_usb_trb	*trb;
		dma_addr_t trb_dma;
		u32 cmd;

		memset(&params, 0, sizeof(params));
		trb = &dep->trb_pool[0];
		trb_dma = soc_usb_trb_dma_offset(dep, trb);

		params.param0 = upper_32_bits(trb_dma);
		params.param1 = lower_32_bits(trb_dma);

		cmd = SOC_USB_DEPCMD_STARTTRANSFER;

		ret = soc_usb_send_gadget_ep_cmd(dep, cmd, &params);
		if (ret < 0)
			return ret;

		if (dep->stream_capable) {
			/*
			 * For streams, at start, there maybe a race where the
			 * host primes the endpoint before the function driver
			 * queues a request to initiate a stream. In that case,
			 * the controller will not see the prime to generate the
			 * ERDY and start stream. To workaround this, issue a
			 * no-op TRB as normal, but end it immediately. As a
			 * result, when the function driver queues the request,
			 * the next START_TRANSFER command will cause the
			 * controller to generate an ERDY to initiate the
			 * stream.
			 */
			soc_usb_stop_active_transfer(dep, true, true);

			/*
			 * All stream eps will reinitiate stream on NoStream
			 * rejection until we can determine that the host can
			 * prime after the first transfer.
			 *
			 * However, if the controller is capable of
			 * TXF_FLUSH_BYPASS, then IN direction endpoints will
			 * automatically restart the stream without the driver
			 * initiation.
			 */
			if (!dep->direction)
				dep->flags |= SOC_USB_EP_FORCE_RESTART_STREAM;
		}
	}

out:
	trace_soc_usb_gadget_ep_enable(dep);

	return 0;
}

void soc_usb_remove_requests(struct soc_usb *su, struct soc_usb_ep *dep, int status)
{
	struct soc_usb_request		*req;

	soc_usb_stop_active_transfer(dep, true, false);

	/* If endxfer is delayed, avoid unmapping requests */
	if (dep->flags & SOC_USB_EP_DELAY_STOP)
		return;

	/* - giveback all requests to gadget driver */
	while (!list_empty(&dep->started_list)) {
		req = next_request(&dep->started_list);

		soc_usb_gadget_giveback(dep, req, status);
	}

	while (!list_empty(&dep->pending_list)) {
		req = next_request(&dep->pending_list);

		soc_usb_gadget_giveback(dep, req, status);
	}

	while (!list_empty(&dep->cancelled_list)) {
		req = next_request(&dep->cancelled_list);

		soc_usb_gadget_giveback(dep, req, status);
	}
}

/**
 * __soc_usb_gadget_ep_disable - disables a hw endpoint
 * @dep: the endpoint to disable
 *
 * This function undoes what __soc_usb_gadget_ep_enable did and also removes
 * requests which are currently being processed by the hardware and those which
 * are not yet scheduled.
 *
 * Caller should take care of locking.
 */
static int __soc_usb_gadget_ep_disable(struct soc_usb_ep *dep)
{
	struct soc_usb		*su = dep->su;
	u32			reg;
	u32			mask;

	trace_soc_usb_gadget_ep_disable(dep);

	/* make sure HW endpoint isn't stalled */
	if (dep->flags & SOC_USB_EP_STALL)
		__soc_usb_gadget_ep_set_halt(dep, 0, false);

	reg = soc_usb_readl(su->regs, SOC_USB_DALEPENA);
	reg &= ~SOC_USB_DALEPENA_EP(dep->number);
	soc_usb_writel(su->regs, SOC_USB_DALEPENA, reg);

	soc_usb_remove_requests(su, dep, -ESHUTDOWN);

	dep->stream_capable = false;
	dep->type = 0;
	mask = SOC_USB_EP_TXFIFO_RESIZED;
	/*
	 * soc_usb_remove_requests() can exit early if SOC_USB EP delayed stop is
	 * set.  Do not clear DEP flags, so that the end transfer command will
	 * be reattempted during the next SETUP stage.
	 */
	if (dep->flags & SOC_USB_EP_DELAY_STOP)
		mask |= (SOC_USB_EP_DELAY_STOP | SOC_USB_EP_TRANSFER_STARTED);
	dep->flags &= mask;

	/* Clear out the ep descriptors for non-ep0 */
	if (dep->number > 1) {
		dep->endpoint.comp_desc = NULL;
		dep->endpoint.desc = NULL;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */

static int soc_usb_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int soc_usb_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}

/* -------------------------------------------------------------------------- */

static int soc_usb_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct soc_usb_ep			*dep;
	struct soc_usb			*su;
	unsigned long			flags;
	int				ret;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("soc_usb: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("soc_usb: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_soc_usb_ep(ep);
	su = dep->su;

	if (dev_WARN_ONCE(su->dev, dep->flags & SOC_USB_EP_ENABLED,
					"%s is already enabled\n",
					dep->name))
		return 0;

	spin_lock_irqsave(&su->lock, flags);
	ret = __soc_usb_gadget_ep_enable(dep, SOC_USB_DEPCFG_ACTION_INIT);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static int soc_usb_gadget_ep_disable(struct usb_ep *ep)
{
	struct soc_usb_ep			*dep;
	struct soc_usb			*su;
	unsigned long			flags;
	int				ret;

	if (!ep) {
		pr_debug("soc_usb: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_soc_usb_ep(ep);
	su = dep->su;

	if (dev_WARN_ONCE(su->dev, !(dep->flags & SOC_USB_EP_ENABLED),
					"%s is already disabled\n",
					dep->name))
		return 0;

	spin_lock_irqsave(&su->lock, flags);
	ret = __soc_usb_gadget_ep_disable(dep);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static struct usb_request *soc_usb_gadget_ep_alloc_request(struct usb_ep *ep,
		gfp_t gfp_flags)
{
	struct soc_usb_request		*req;
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;

	req->direction	= dep->direction;
	req->epnum	= dep->number;
	req->dep	= dep;
	req->status	= SOC_USB_REQUEST_STATUS_UNKNOWN;

	trace_soc_usb_alloc_request(req);

	return &req->request;
}

static void soc_usb_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct soc_usb_request		*req = to_soc_usb_request(request);

	trace_soc_usb_free_request(req);
	kfree(req);
}

/**
 * soc_usb_ep_prev_trb - returns the previous TRB in the ring
 * @dep: The endpoint with the TRB ring
 * @index: The index of the current TRB in the ring
 *
 * Returns the TRB prior to the one pointed to by the index. If the
 * index is 0, we will wrap backwards, skip the link TRB, and return
 * the one just before that.
 */
static struct soc_usb_trb *soc_usb_ep_prev_trb(struct soc_usb_ep *dep, u8 index)
{
	u8 tmp = index;

	if (!tmp)
		tmp = SOC_USB_TRB_NUM - 1;

	return &dep->trb_pool[tmp - 1];
}

static u32 soc_usb_calc_trbs_left(struct soc_usb_ep *dep)
{
	u8			trbs_left;

	/*
	 * If the enqueue & dequeue are equal then the TRB ring is either full
	 * or empty. It's considered full when there are SOC_USB_TRB_NUM-1 of TRBs
	 * pending to be processed by the driver.
	 */
	if (dep->trb_enqueue == dep->trb_dequeue) {
		/*
		 * If there is any request remained in the started_list at
		 * this point, that means there is no TRB available.
		 */
		if (!list_empty(&dep->started_list))
			return 0;

		return SOC_USB_TRB_NUM - 1;
	}

	trbs_left = dep->trb_dequeue - dep->trb_enqueue;
	trbs_left &= (SOC_USB_TRB_NUM - 1);

	if (dep->trb_dequeue < dep->trb_enqueue)
		trbs_left--;

	return trbs_left;
}

/**
 * soc_usb_prepare_one_trb - setup one TRB from one request
 * @dep: endpoint for which this request is prepared
 * @req: soc_usb_request pointer
 * @trb_length: buffer size of the TRB
 * @chain: should this TRB be chained to the next?
 * @node: only for isochronous endpoints. First TRB needs different type.
 * @use_bounce_buffer: set to use bounce buffer
 * @must_interrupt: set to interrupt on TRB completion
 */
static void soc_usb_prepare_one_trb(struct soc_usb_ep *dep,
		struct soc_usb_request *req, unsigned int trb_length,
		unsigned int chain, unsigned int node, bool use_bounce_buffer,
		bool must_interrupt)
{
	struct soc_usb_trb		*trb;
	dma_addr_t		dma;
	unsigned int		stream_id = req->request.stream_id;
	unsigned int		short_not_ok = req->request.short_not_ok;
	unsigned int		no_interrupt = req->request.no_interrupt;
	unsigned int		is_last = req->request.is_last;
	struct soc_usb		*su = dep->su;
	struct usb_gadget	*gadget = su->gadget;
	enum usb_device_speed	speed = gadget->speed;

	if (use_bounce_buffer)
		dma = dep->su->bounce_addr;
	else if (req->request.num_sgs > 0)
		dma = sg_dma_address(req->start_sg);
	else
		dma = req->request.dma;

	trb = &dep->trb_pool[dep->trb_enqueue];

	if (!req->trb) {
		soc_usb_gadget_move_started_request(req);
		req->trb = trb;
		req->trb_dma = soc_usb_trb_dma_offset(dep, trb);
	}

	req->num_trbs++;

	trb->size = SOC_USB_TRB_SIZE_LENGTH(trb_length);
	trb->bpl = lower_32_bits(dma);
	trb->bph = upper_32_bits(dma);

	switch (usb_endpoint_type(dep->endpoint.desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		trb->ctrl = SOC_USB_TRBCTL_CONTROL_SETUP;
		break;

	case USB_ENDPOINT_XFER_ISOC:
		if (!node) {
			trb->ctrl = SOC_USB_TRBCTL_ISOCHRONOUS_FIRST;

			/*
			 * USB Specification 2.0 Section 5.9.2 states that: "If
			 * there is only a single transaction in the microframe,
			 * only a DATA0 data packet PID is used.  If there are
			 * two transactions per microframe, DATA1 is used for
			 * the first transaction data packet and DATA0 is used
			 * for the second transaction data packet.  If there are
			 * three transactions per microframe, DATA2 is used for
			 * the first transaction data packet, DATA1 is used for
			 * the second, and DATA0 is used for the third."
			 *
			 * IOW, we should satisfy the following cases:
			 *
			 * 1) length <= maxpacket
			 *	- DATA0
			 *
			 * 2) maxpacket < length <= (2 * maxpacket)
			 *	- DATA1, DATA0
			 *
			 * 3) (2 * maxpacket) < length <= (3 * maxpacket)
			 *	- DATA2, DATA1, DATA0
			 */
			if (speed == USB_SPEED_HIGH) {
				struct usb_ep *ep = &dep->endpoint;
				unsigned int mult = 2;
				unsigned int maxp = usb_endpoint_maxp(ep->desc);

				if (req->request.length <= (2 * maxp))
					mult--;

				if (req->request.length <= maxp)
					mult--;

				trb->size |= SOC_USB_TRB_SIZE_PCM1(mult);
			}
		} else {
			trb->ctrl = SOC_USB_TRBCTL_ISOCHRONOUS;
		}

		if (!no_interrupt && !chain)
			trb->ctrl |= SOC_USB_TRB_CTRL_ISP_IMI;
		break;

	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		trb->ctrl = SOC_USB_TRBCTL_NORMAL;
		break;
	default:
		/*
		 * This is only possible with faulty memory because we
		 * checked it already :)
		 */
		dev_WARN(su->dev, "Unknown endpoint type %d\n",
				usb_endpoint_type(dep->endpoint.desc));
	}

	/*
	 * Enable Continue on Short Packet
	 * when endpoint is not a stream capable
	 */
	if (usb_endpoint_dir_out(dep->endpoint.desc)) {
		if (!dep->stream_capable)
			trb->ctrl |= SOC_USB_TRB_CTRL_CSP;

		if (short_not_ok)
			trb->ctrl |= SOC_USB_TRB_CTRL_ISP_IMI;
	}

	if ((!no_interrupt && !chain) || must_interrupt)
		trb->ctrl |= SOC_USB_TRB_CTRL_IOC;

	if (chain)
		trb->ctrl |= SOC_USB_TRB_CTRL_CHN;
	else if (dep->stream_capable && is_last)
		trb->ctrl |= SOC_USB_TRB_CTRL_LST;

	if (usb_endpoint_xfer_bulk(dep->endpoint.desc) && dep->stream_capable)
		trb->ctrl |= SOC_USB_TRB_CTRL_SID_SOFN(stream_id);

	/*
	 * As per data book 4.2.3.2TRB Control Bit Rules section
	 *
	 * The controller autonomously checks the HWO field of a TRB to determine if the
	 * entire TRB is valid. Therefore, software must ensure that the rest of the TRB
	 * is valid before setting the HWO field to '1'. In most systems, this means that
	 * software must update the fourth DWORD of a TRB last.
	 *
	 * However there is a possibility of CPU re-ordering here which can cause
	 * controller to observe the HWO bit set prematurely.
	 * Add a write memory barrier to prevent CPU re-ordering.
	 */
	wmb();
	trb->ctrl |= SOC_USB_TRB_CTRL_HWO;

	soc_usb_ep_inc_enq(dep);

	trace_soc_usb_prepare_trb(dep, trb);
}

static bool soc_usb_needs_extra_trb(struct soc_usb_ep *dep, struct soc_usb_request *req)
{
	unsigned int maxp = usb_endpoint_maxp(dep->endpoint.desc);
	unsigned int rem = req->request.length % maxp;

	if ((req->request.length && req->request.zero && !rem &&
			!usb_endpoint_xfer_isoc(dep->endpoint.desc)) ||
			(!req->direction && rem))
		return true;

	return false;
}

/**
 * soc_usb_prepare_last_sg - prepare TRBs for the last SG entry
 * @dep: The endpoint that the request belongs to
 * @req: The request to prepare
 * @entry_length: The last SG entry size
 * @node: Indicates whether this is not the first entry (for isoc only)
 *
 * Return the number of TRBs prepared.
 */
static int soc_usb_prepare_last_sg(struct soc_usb_ep *dep,
		struct soc_usb_request *req, unsigned int entry_length,
		unsigned int node)
{
	unsigned int maxp = usb_endpoint_maxp(dep->endpoint.desc);
	unsigned int rem = req->request.length % maxp;
	unsigned int num_trbs = 1;

	if (soc_usb_needs_extra_trb(dep, req))
		num_trbs++;

	if (soc_usb_calc_trbs_left(dep) < num_trbs)
		return 0;

	req->needs_extra_trb = num_trbs > 1;

	/* Prepare a normal TRB */
	if (req->direction || req->request.length)
		soc_usb_prepare_one_trb(dep, req, entry_length,
				req->needs_extra_trb, node, false, false);

	/* Prepare extra TRBs for ZLP and MPS OUT transfer alignment */
	if ((!req->direction && !req->request.length) || req->needs_extra_trb)
		soc_usb_prepare_one_trb(dep, req,
				req->direction ? 0 : maxp - rem,
				false, 1, true, false);

	return num_trbs;
}

static int soc_usb_prepare_trbs_sg(struct soc_usb_ep *dep,
		struct soc_usb_request *req)
{
	struct scatterlist *sg = req->start_sg;
	struct scatterlist *s;
	int		i;
	unsigned int length = req->request.length;
	unsigned int remaining = req->request.num_mapped_sgs
		- req->num_queued_sgs;
	unsigned int num_trbs = req->num_trbs;
	bool needs_extra_trb = soc_usb_needs_extra_trb(dep, req);

	/*
	 * If we resume preparing the request, then get the remaining length of
	 * the request and resume where we left off.
	 */
	for_each_sg(req->request.sg, s, req->num_queued_sgs, i)
		length -= sg_dma_len(s);

	for_each_sg(sg, s, remaining, i) {
		unsigned int num_trbs_left = soc_usb_calc_trbs_left(dep);
		unsigned int trb_length;
		bool must_interrupt = false;
		bool last_sg = false;

		trb_length = min_t(unsigned int, length, sg_dma_len(s));

		length -= trb_length;

		/*
		 * IOMMU driver is coalescing the list of sgs which shares a
		 * page boundary into one and giving it to USB driver. With
		 * this the number of sgs mapped is not equal to the number of
		 * sgs passed. So mark the chain bit to false if it isthe last
		 * mapped sg.
		 */
		if ((i == remaining - 1) || !length)
			last_sg = true;

		if (!num_trbs_left)
			break;

		if (last_sg) {
			if (!soc_usb_prepare_last_sg(dep, req, trb_length, i))
				break;
		} else {
			/*
			 * Look ahead to check if we have enough TRBs for the
			 * next SG entry. If not, set interrupt on this TRB to
			 * resume preparing the next SG entry when more TRBs are
			 * free.
			 */
			if (num_trbs_left == 1 || (needs_extra_trb &&
					num_trbs_left <= 2 &&
					sg_dma_len(sg_next(s)) >= length)) {
				struct soc_usb_request *r;

				/* Check if previous requests already set IOC */
				list_for_each_entry(r, &dep->started_list, list) {
					if (r != req && !r->request.no_interrupt)
						break;

					if (r == req)
						must_interrupt = true;
				}
			}

			soc_usb_prepare_one_trb(dep, req, trb_length, 1, i, false,
					must_interrupt);
		}

		/*
		 * There can be a situation where all sgs in sglist are not
		 * queued because of insufficient trb number. To handle this
		 * case, update start_sg to next sg to be queued, so that
		 * we have free trbs we can continue queuing from where we
		 * previously stopped
		 */
		if (!last_sg)
			req->start_sg = sg_next(s);

		req->num_queued_sgs++;
		req->num_pending_sgs--;

		/*
		 * The number of pending SG entries may not correspond to the
		 * number of mapped SG entries. If all the data are queued, then
		 * don't include unused SG entries.
		 */
		if (length == 0) {
			req->num_pending_sgs = 0;
			break;
		}

		if (must_interrupt)
			break;
	}

	return req->num_trbs - num_trbs;
}

static int soc_usb_prepare_trbs_linear(struct soc_usb_ep *dep,
		struct soc_usb_request *req)
{
	return soc_usb_prepare_last_sg(dep, req, req->request.length, 0);
}

/*
 * soc_usb_prepare_trbs - setup TRBs from requests
 * @dep: endpoint for which requests are being prepared
 *
 * The function goes through the requests list and sets up TRBs for the
 * transfers. The function returns once there are no more TRBs available or
 * it runs out of requests.
 *
 * Returns the number of TRBs prepared or negative errno.
 */
static int soc_usb_prepare_trbs(struct soc_usb_ep *dep)
{
	struct soc_usb_request	*req, *n;
	int			ret = 0;

	BUILD_BUG_ON_NOT_POWER_OF_2(SOC_USB_TRB_NUM);

	/*
	 * We can get in a situation where there's a request in the started list
	 * but there weren't enough TRBs to fully kick it in the first time
	 * around, so it has been waiting for more TRBs to be freed up.
	 *
	 * In that case, we should check if we have a request with pending_sgs
	 * in the started list and prepare TRBs for that request first,
	 * otherwise we will prepare TRBs completely out of order and that will
	 * break things.
	 */
	list_for_each_entry(req, &dep->started_list, list) {
		if (req->num_pending_sgs > 0) {
			ret = soc_usb_prepare_trbs_sg(dep, req);
			if (!ret || req->num_pending_sgs)
				return ret;
		}

		if (!soc_usb_calc_trbs_left(dep))
			return ret;

		/*
		 * Don't prepare beyond a transfer. In USB, its transfer
		 * burst capability may try to read and use TRBs beyond the
		 * active transfer instead of stopping.
		 */
		if (dep->stream_capable && req->request.is_last)
			return ret;
	}

	list_for_each_entry_safe(req, n, &dep->pending_list, list) {
		struct soc_usb	*su = dep->su;

		ret = usb_gadget_map_request_by_dev(su->sysdev, &req->request,
						    dep->direction);
		if (ret)
			return ret;

		req->sg			= req->request.sg;
		req->start_sg		= req->sg;
		req->num_queued_sgs	= 0;
		req->num_pending_sgs	= req->request.num_mapped_sgs;

		if (req->num_pending_sgs > 0) {
			ret = soc_usb_prepare_trbs_sg(dep, req);
			if (req->num_pending_sgs)
				return ret;
		} else {
			ret = soc_usb_prepare_trbs_linear(dep, req);
		}

		if (!ret || !soc_usb_calc_trbs_left(dep))
			return ret;

		/*
		 * Don't prepare beyond a transfer. In SOC USB, its transfer
		 * burst capability may try to read and use TRBs beyond the
		 * active transfer instead of stopping.
		 */
		if (dep->stream_capable && req->request.is_last)
			return ret;
	}

	return ret;
}

static void soc_usb_gadget_ep_cleanup_cancelled_requests(struct soc_usb_ep *dep);

static int __soc_usb_gadget_kick_transfer(struct soc_usb_ep *dep)
{
	struct soc_usb_gadget_ep_cmd_params params;
	struct soc_usb_request		*req;
	int				starting;
	int				ret;
	u32				cmd;

	/*
	 * Note that it's normal to have no new TRBs prepared (i.e. ret == 0).
	 * This happens when we need to stop and restart a transfer such as in
	 * the case of reinitiating a stream or retrying an isoc transfer.
	 */
	ret = soc_usb_prepare_trbs(dep);
	if (ret < 0)
		return ret;

	starting = !(dep->flags & SOC_USB_EP_TRANSFER_STARTED);

	/*
	 * If there's no new TRB prepared and we don't need to restart a
	 * transfer, there's no need to update the transfer.
	 */
	if (!ret && !starting)
		return ret;

	req = next_request(&dep->started_list);
	if (!req) {
		dep->flags |= SOC_USB_EP_PENDING_REQUEST;
		return 0;
	}

	memset(&params, 0, sizeof(params));

	if (starting) {
		params.param0 = upper_32_bits(req->trb_dma);
		params.param1 = lower_32_bits(req->trb_dma);
		cmd = SOC_USB_DEPCMD_STARTTRANSFER;

		if (dep->stream_capable)
			cmd |= SOC_USB_DEPCMD_PARAM(req->request.stream_id);

		if (usb_endpoint_xfer_isoc(dep->endpoint.desc))
			cmd |= SOC_USB_DEPCMD_PARAM(dep->frame_number);
	} else {
		cmd = SOC_USB_DEPCMD_UPDATETRANSFER |
			SOC_USB_DEPCMD_PARAM(dep->resource_index);
	}

	ret = soc_usb_send_gadget_ep_cmd(dep, cmd, &params);
	if (ret < 0) {
		struct soc_usb_request *tmp;

		if (ret == -EAGAIN)
			return ret;

		soc_usb_stop_active_transfer(dep, true, true);

		list_for_each_entry_safe(req, tmp, &dep->started_list, list)
			soc_usb_gadget_move_cancelled_request(req, SOC_USB_REQUEST_STATUS_DEQUEUED);

		/* If ep isn't started, then there's no end transfer pending */
		if (!(dep->flags & SOC_USB_EP_END_TRANSFER_PENDING))
			soc_usb_gadget_ep_cleanup_cancelled_requests(dep);

		return ret;
	}

	if (dep->stream_capable && req->request.is_last)
		dep->flags |= SOC_USB_EP_WAIT_TRANSFER_COMPLETE;

	return 0;
}

static int __soc_usb_gadget_get_frame(struct soc_usb *su)
{
	u32			reg;

	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);
	return SOC_USB_DSTS_SOFFN(reg);
}

/**
 * __soc_usb_stop_active_transfer - stop the current active transfer
 * @dep: isoc endpoint
 * @force: set forcerm bit in the command
 * @interrupt: command complete interrupt after End Transfer command
 *
 * When setting force, the ForceRM bit will be set. In that case
 * the controller won't update the TRB progress on command
 * completion. It also won't clear the HWO bit in the TRB.
 * The command will also not complete immediately in that case.
 */
static int __soc_usb_stop_active_transfer(struct soc_usb_ep *dep, bool force, bool interrupt)
{
	struct soc_usb_gadget_ep_cmd_params params;
	u32 cmd;
	int ret;

	cmd = SOC_USB_DEPCMD_ENDTRANSFER;
	cmd |= force ? SOC_USB_DEPCMD_HIPRI_FORCERM : 0;
	cmd |= interrupt ? SOC_USB_DEPCMD_CMDIOC : 0;
	cmd |= SOC_USB_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
	ret = soc_usb_send_gadget_ep_cmd(dep, cmd, &params);
	/*
	 * If the End Transfer command was timed out while the device is
	 * not in SETUP phase, it's possible that an incoming Setup packet
	 * may prevent the command's completion. Let's retry when the
	 * ep0state returns to EP0_SETUP_PHASE.
	 */
	if (ret == -ETIMEDOUT && dep->su->ep0state != EP0_SETUP_PHASE) {
		dep->flags |= SOC_USB_EP_DELAY_STOP;
		return 0;
	}
	WARN_ON_ONCE(ret);
	dep->resource_index = 0;

	if (!interrupt) {
		mdelay(1);
		dep->flags &= ~SOC_USB_EP_TRANSFER_STARTED;
	} else if (!ret) {
		dep->flags |= SOC_USB_EP_END_TRANSFER_PENDING;
	}

	dep->flags &= ~SOC_USB_EP_DELAY_STOP;
	return ret;
}

static int __soc_usb_gadget_start_isoc(struct soc_usb_ep *dep)
{
	const struct usb_endpoint_descriptor *desc = dep->endpoint.desc;
	struct soc_usb *su = dep->su;
	int ret;
	int i;

	if (list_empty(&dep->pending_list) &&
	    list_empty(&dep->started_list)) {
		dep->flags |= SOC_USB_EP_PENDING_REQUEST;
		return -EAGAIN;
	}

	if (desc->bInterval <= 14 &&
	    su->gadget->speed >= USB_SPEED_HIGH) {
		u32 frame = __soc_usb_gadget_get_frame(su);
		bool rollover = frame <
				(dep->frame_number & SOC_USB_FRNUMBER_MASK);

		/*
		 * frame_number is set from XferNotReady and may be already
		 * out of date. DSTS only provides the lower 14 bit of the
		 * current frame number. So add the upper two bits of
		 * frame_number and handle a possible rollover.
		 * This will provide the correct frame_number unless more than
		 * rollover has happened since XferNotReady.
		 */

		dep->frame_number = (dep->frame_number & ~SOC_USB_FRNUMBER_MASK) |
				     frame;
		if (rollover)
			dep->frame_number += BIT(14);
	}

	for (i = 0; i < SOC_USB_ISOC_MAX_RETRIES; i++) {
		int future_interval = i + 1;

		/* Give the controller at least 500us to schedule transfers */
		if (desc->bInterval < 3)
			future_interval += 3 - desc->bInterval;

		dep->frame_number = SOC_USB_ALIGN_FRAME(dep, future_interval);

		ret = __soc_usb_gadget_kick_transfer(dep);
		if (ret != -EAGAIN)
			break;
	}

	/*
	 * After a number of unsuccessful start attempts due to bus-expiry
	 * status, issue END_TRANSFER command and retry on the next XferNotReady
	 * event.
	 */
	if (ret == -EAGAIN)
		ret = __soc_usb_stop_active_transfer(dep, false, true);

	return ret;
}

static int __soc_usb_gadget_ep_queue(struct soc_usb_ep *dep, struct soc_usb_request *req)
{
	struct soc_usb		*su = dep->su;

	if (!dep->endpoint.desc || !su->pullups_connected || !su->connected) {
		dev_dbg(su->dev, "%s: can't queue to disabled endpoint\n",
				dep->name);
		return -ESHUTDOWN;
	}

	if (WARN(req->dep != dep, "request %pK belongs to '%s'\n",
				&req->request, req->dep->name))
		return -EINVAL;

	if (WARN(req->status < SOC_USB_REQUEST_STATUS_COMPLETED,
				"%s: request %pK already in flight\n",
				dep->name, &req->request))
		return -EINVAL;

	pm_runtime_get(su->dev);

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;

	trace_soc_usb_ep_queue(req);

	list_add_tail(&req->list, &dep->pending_list);
	req->status = SOC_USB_REQUEST_STATUS_QUEUED;

	if (dep->flags & SOC_USB_EP_WAIT_TRANSFER_COMPLETE)
		return 0;

	/*
	 * Start the transfer only after the END_TRANSFER is completed
	 * and endpoint STALL is cleared.
	 */
	if ((dep->flags & SOC_USB_EP_END_TRANSFER_PENDING) ||
	    (dep->flags & SOC_USB_EP_WEDGE) ||
	    (dep->flags & SOC_USB_EP_DELAY_STOP) ||
	    (dep->flags & SOC_USB_EP_STALL)) {
		dep->flags |= SOC_USB_EP_DELAY_START;
		return 0;
	}

	/*
	 * NOTICE: Isochronous endpoints should NEVER be prestarted. We must
	 * wait for a XferNotReady event so we will know what's the current
	 * (micro-)frame number.
	 *
	 * Without this trick, we are very, very likely gonna get Bus Expiry
	 * errors which will force us issue EndTransfer command.
	 */
	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		if (!(dep->flags & SOC_USB_EP_TRANSFER_STARTED)) {
			if ((dep->flags & SOC_USB_EP_PENDING_REQUEST))
				return __soc_usb_gadget_start_isoc(dep);

			return 0;
		}
	}

	__soc_usb_gadget_kick_transfer(dep);

	return 0;
}

static int soc_usb_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct soc_usb_request		*req = to_soc_usb_request(request);
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&su->lock, flags);
	ret = __soc_usb_gadget_ep_queue(dep, req);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static void soc_usb_gadget_ep_skip_trbs(struct soc_usb_ep *dep, struct soc_usb_request *req)
{
	int i;

	/* If req->trb is not set, then the request has not started */
	if (!req->trb)
		return;

	/*
	 * If request was already started, this means we had to
	 * stop the transfer. With that we also need to ignore
	 * all TRBs used by the request, however TRBs can only
	 * be modified after completion of END_TRANSFER
	 * command. So what we do here is that we wait for
	 * END_TRANSFER completion and only after that, we jump
	 * over TRBs by clearing HWO and incrementing dequeue
	 * pointer.
	 */
	for (i = 0; i < req->num_trbs; i++) {
		struct soc_usb_trb *trb;

		trb = &dep->trb_pool[dep->trb_dequeue];
		trb->ctrl &= ~SOC_USB_TRB_CTRL_HWO;
		soc_usb_ep_inc_deq(dep);
	}

	req->num_trbs = 0;
}

static void soc_usb_gadget_ep_cleanup_cancelled_requests(struct soc_usb_ep *dep)
{
	struct soc_usb_request		*req;
	struct soc_usb			*su = dep->su;

	while (!list_empty(&dep->cancelled_list)) {
		req = next_request(&dep->cancelled_list);
		soc_usb_gadget_ep_skip_trbs(dep, req);
		switch (req->status) {
		case SOC_USB_REQUEST_STATUS_DISCONNECTED:
			soc_usb_gadget_giveback(dep, req, -ESHUTDOWN);
			break;
		case SOC_USB_REQUEST_STATUS_DEQUEUED:
			soc_usb_gadget_giveback(dep, req, -ECONNRESET);
			break;
		case SOC_USB_REQUEST_STATUS_STALLED:
			soc_usb_gadget_giveback(dep, req, -EPIPE);
			break;
		default:
			dev_err(su->dev, "request cancelled with wrong reason:%d\n", req->status);
			soc_usb_gadget_giveback(dep, req, -ECONNRESET);
			break;
		}
		/*
		 * The endpoint is disabled, let the soc_usb_remove_requests()
		 * handle the cleanup.
		 */
		if (!dep->endpoint.desc)
			break;
	}
}

static int soc_usb_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct soc_usb_request		*req = to_soc_usb_request(request);
	struct soc_usb_request		*r = NULL;

	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;

	unsigned long			flags;
	int				ret = 0;

	trace_soc_usb_ep_dequeue(req);

	spin_lock_irqsave(&su->lock, flags);

	list_for_each_entry(r, &dep->cancelled_list, list) {
		if (r == req)
			goto out;
	}

	list_for_each_entry(r, &dep->pending_list, list) {
		if (r == req) {
			/*
			 * Explicitly check for EP0/1 as dequeue for those
			 * EPs need to be handled differently.  Control EP
			 * only deals with one USB req, and giveback will
			 * occur during soc_usb_ep0_stall_and_restart().  EP0
			 * requests are never added to started_list.
			 */
			if (dep->number > 1)
				soc_usb_gadget_giveback(dep, req, -ECONNRESET);
			else
				soc_usb_ep0_reset_state(su);
			goto out;
		}
	}

	list_for_each_entry(r, &dep->started_list, list) {
		if (r == req) {
			struct soc_usb_request *t;

			/* wait until it is processed */
			soc_usb_stop_active_transfer(dep, true, true);

			/*
			 * Remove any started request if the transfer is
			 * cancelled.
			 */
			list_for_each_entry_safe(r, t, &dep->started_list, list)
				soc_usb_gadget_move_cancelled_request(r,
						SOC_USB_REQUEST_STATUS_DEQUEUED);

			dep->flags &= ~SOC_USB_EP_WAIT_TRANSFER_COMPLETE;

			goto out;
		}
	}

	dev_err(su->dev, "request %pK was not queued to %s\n",
		request, ep->name);
	ret = -EINVAL;
out:
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

int __soc_usb_gadget_ep_set_halt(struct soc_usb_ep *dep, int value, int protocol)
{
	struct soc_usb_gadget_ep_cmd_params	params;
	struct soc_usb				*su = dep->su;
	struct soc_usb_request			*req;
	struct soc_usb_request			*tmp;
	int					ret;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		dev_err(su->dev, "%s is of Isochronous type\n", dep->name);
		return -EINVAL;
	}

	memset(&params, 0x00, sizeof(params));

	if (value) {
		struct soc_usb_trb *trb;

		unsigned int transfer_in_flight;
		unsigned int started;

		if (dep->number > 1)
			trb = soc_usb_ep_prev_trb(dep, dep->trb_enqueue);
		else
			trb = &su->ep0_trb[dep->trb_enqueue];

		transfer_in_flight = trb->ctrl & SOC_USB_TRB_CTRL_HWO;
		started = !list_empty(&dep->started_list);

		if (!protocol && ((dep->direction && transfer_in_flight) ||
				(!dep->direction && started))) {
			return -EAGAIN;
		}

		ret = soc_usb_send_gadget_ep_cmd(dep, SOC_USB_DEPCMD_SETSTALL,
				&params);
		if (ret)
			dev_err(su->dev, "failed to set STALL on %s\n",
					dep->name);
		else
			dep->flags |= SOC_USB_EP_STALL;
	} else {
		/*
		 * Don't issue CLEAR_STALL command to control endpoints. The
		 * controller automatically clears the STALL when it receives
		 * the SETUP token.
		 */
		if (dep->number <= 1) {
			dep->flags &= ~(SOC_USB_EP_STALL | SOC_USB_EP_WEDGE);
			return 0;
		}

		soc_usb_stop_active_transfer(dep, true, true);

		list_for_each_entry_safe(req, tmp, &dep->started_list, list)
			soc_usb_gadget_move_cancelled_request(req, SOC_USB_REQUEST_STATUS_STALLED);

		if (dep->flags & SOC_USB_EP_END_TRANSFER_PENDING ||
		    (dep->flags & SOC_USB_EP_DELAY_STOP)) {
			dep->flags |= SOC_USB_EP_PENDING_CLEAR_STALL;
			if (protocol)
				su->clear_stall_protocol = dep->number;

			return 0;
		}

		soc_usb_gadget_ep_cleanup_cancelled_requests(dep);

		ret = soc_usb_send_clear_stall_ep_cmd(dep);
		if (ret) {
			dev_err(su->dev, "failed to clear STALL on %s\n",
					dep->name);
			return ret;
		}

		dep->flags &= ~(SOC_USB_EP_STALL | SOC_USB_EP_WEDGE);

		if ((dep->flags & SOC_USB_EP_DELAY_START) &&
		    !usb_endpoint_xfer_isoc(dep->endpoint.desc))
			__soc_usb_gadget_kick_transfer(dep);

		dep->flags &= ~SOC_USB_EP_DELAY_START;
	}

	return ret;
}

static int soc_usb_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&su->lock, flags);
	ret = __soc_usb_gadget_ep_set_halt(dep, value, false);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static int soc_usb_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct soc_usb_ep			*dep = to_soc_usb_ep(ep);
	struct soc_usb			*su = dep->su;
	unsigned long			flags;
	int				ret;

	spin_lock_irqsave(&su->lock, flags);
	dep->flags |= SOC_USB_EP_WEDGE;

	if (dep->number == 0 || dep->number == 1)
		ret = __soc_usb_gadget_ep0_set_halt(ep, 1);
	else
		ret = __soc_usb_gadget_ep_set_halt(dep, 1, false);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

/* -------------------------------------------------------------------------- */

static struct usb_endpoint_descriptor soc_usb_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static const struct usb_ep_ops soc_usb_gadget_ep0_ops = {
	.enable		= soc_usb_gadget_ep0_enable,
	.disable	= soc_usb_gadget_ep0_disable,
	.alloc_request	= soc_usb_gadget_ep_alloc_request,
	.free_request	= soc_usb_gadget_ep_free_request,
	.queue		= soc_usb_gadget_ep0_queue,
	.dequeue	= soc_usb_gadget_ep_dequeue,
	.set_halt	= soc_usb_gadget_ep0_set_halt,
	.set_wedge	= soc_usb_gadget_ep_set_wedge,
};

static const struct usb_ep_ops soc_usb_gadget_ep_ops = {
	.enable		= soc_usb_gadget_ep_enable,
	.disable	= soc_usb_gadget_ep_disable,
	.alloc_request	= soc_usb_gadget_ep_alloc_request,
	.free_request	= soc_usb_gadget_ep_free_request,
	.queue		= soc_usb_gadget_ep_queue,
	.dequeue	= soc_usb_gadget_ep_dequeue,
	.set_halt	= soc_usb_gadget_ep_set_halt,
	.set_wedge	= soc_usb_gadget_ep_set_wedge,
};

/* -------------------------------------------------------------------------- */

static void soc_usb_gadget_enable_linksts_evts(struct soc_usb *su, bool set)
{
	u32 reg;

	reg = soc_usb_readl(su->regs, SOC_USB_DEVTEN);
	if (set)
		reg |= SOC_USB_DEVTEN_ULSTCNGEN;
	else
		reg &= ~SOC_USB_DEVTEN_ULSTCNGEN;

	soc_usb_writel(su->regs, SOC_USB_DEVTEN, reg);
}

static int soc_usb_gadget_get_frame(struct usb_gadget *g)
{
	struct soc_usb		*su = gadget_to_su(g);

	return __soc_usb_gadget_get_frame(su);
}

static int __soc_usb_gadget_wakeup(struct soc_usb *su, bool async)
{
	int			retries;

	int			ret;
	u32			reg;

	u8			link_state;

	/*
	 * According to the Databook Remote wakeup request should
	 * be issued only when the device is in early suspend state.
	 *
	 * We can check that via USB Link State bits in DSTS register.
	 */
	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);

	link_state = SOC_USB_DSTS_USBLNKST(reg);

	switch (link_state) {
	case SOC_USB_LINK_STATE_RESET:
	case SOC_USB_LINK_STATE_RX_DET:	/* in HS, means Early Suspend */
	case SOC_USB_LINK_STATE_U3:	/* in HS, means SUSPEND */
	case SOC_USB_LINK_STATE_U2:	/* in HS, means Sleep (L1) */
	case SOC_USB_LINK_STATE_U1:
	case SOC_USB_LINK_STATE_RESUME:
		break;
	default:
		return -EINVAL;
	}

	if (async)
		soc_usb_gadget_enable_linksts_evts(su, true);

	ret = soc_usb_gadget_set_link_state(su, SOC_USB_LINK_STATE_RECOV);
	if (ret < 0) {
		dev_err(su->dev, "failed to put link in Recovery\n");
		soc_usb_gadget_enable_linksts_evts(su, false);
		return ret;
	}

	/*
	 * Since link status change events are enabled we will receive
	 * an U0 event when wakeup is successful. So bail out.
	 */
	if (async)
		return 0;

	/* poll until Link State changes to ON */
	retries = 20000;

	while (retries--) {
		reg = soc_usb_readl(su->regs, SOC_USB_DSTS);

		/* in HS, means ON */
		if (SOC_USB_DSTS_USBLNKST(reg) == SOC_USB_LINK_STATE_U0)
			break;
	}

	if (SOC_USB_DSTS_USBLNKST(reg) != SOC_USB_LINK_STATE_U0) {
		dev_err(su->dev, "failed to send remote wakeup\n");
		return -EINVAL;
	}

	return 0;
}

static int soc_usb_gadget_wakeup(struct usb_gadget *g)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;
	int			ret;

	if (!su->wakeup_configured) {
		dev_err(su->dev, "remote wakeup not configured\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&su->lock, flags);
	if (!su->gadget->wakeup_armed) {
		dev_err(su->dev, "not armed for remote wakeup\n");
		spin_unlock_irqrestore(&su->lock, flags);
		return -EINVAL;
	}
	ret = __soc_usb_gadget_wakeup(su, true);

	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static void soc_usb_resume_gadget(struct soc_usb *su);

static int soc_usb_gadget_func_wakeup(struct usb_gadget *g, int intf_id)
{
	struct  soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;
	int			ret;
	int			link_state;

	if (!su->wakeup_configured) {
		dev_err(su->dev, "remote wakeup not configured\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&su->lock, flags);
	/*
	 * If the link is in U3, signal for remote wakeup and wait for the
	 * link to transition to U0 before sending device notification.
	 */
	link_state = soc_usb_gadget_get_link_state(su);
	if (link_state == SOC_USB_LINK_STATE_U3) {
		ret = __soc_usb_gadget_wakeup(su, false);
		if (ret) {
			spin_unlock_irqrestore(&su->lock, flags);
			return -EINVAL;
		}
		soc_usb_resume_gadget(su);
		su->suspended = false;
		su->link_state = SOC_USB_LINK_STATE_U0;
	}

	ret = soc_usb_send_gadget_generic_command(su, SOC_USB_DGCMD_DEV_NOTIFICATION,
					       SOC_USB_DGCMDPAR_DN_FUNC_WAKE |
					       SOC_USB_DGCMDPAR_INTF_SEL(intf_id));
	if (ret)
		dev_err(su->dev, "function remote wakeup failed, ret:%d\n", ret);

	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static int soc_usb_gadget_set_remote_wakeup(struct usb_gadget *g, int set)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;

	spin_lock_irqsave(&su->lock, flags);
	su->wakeup_configured = !!set;
	spin_unlock_irqrestore(&su->lock, flags);

	return 0;
}

static int soc_usb_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;

	spin_lock_irqsave(&su->lock, flags);
	g->is_selfpowered = !!is_selfpowered;
	spin_unlock_irqrestore(&su->lock, flags);

	return 0;
}

static void soc_usb_stop_active_transfers(struct soc_usb *su)
{
	u32 epnum;

	for (epnum = 2; epnum < su->num_eps; epnum++) {
		struct soc_usb_ep *dep;

		dep = su->eps[epnum];
		if (!dep)
			continue;

		soc_usb_remove_requests(su, dep, -ESHUTDOWN);
	}
}

static void __soc_usb_gadget_set_ssp_rate(struct soc_usb *su)
{
	enum usb_ssp_rate	ssp_rate = su->gadget_ssp_rate;
	u32			reg;

    printk(">>>>ssp_rate:0x%x, max:0x%x, FUN:%s, line:%d\n", ssp_rate, su->max_ssp_rate, __FUNCTION__, __LINE__);
	if (ssp_rate == USB_SSP_GEN_UNKNOWN)
		ssp_rate = su->max_ssp_rate;

	reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
	reg &= ~SOC_USB_DCFG_SPEED_MASK;
	reg &= ~SOC_USB_DCFG_NUMLANES(~0);

	if (ssp_rate == USB_SSP_GEN_1x2)
		reg |= SOC_USB_DCFG_SUPERSPEED;
	else if (su->max_ssp_rate != USB_SSP_GEN_1x2)
		reg |= SOC_USB_DCFG_SUPERSPEED_PLUS;

	if (ssp_rate != USB_SSP_GEN_2x1 &&
	    su->max_ssp_rate != USB_SSP_GEN_2x1)
		reg |= SOC_USB_DCFG_NUMLANES(1);

	soc_usb_writel(su->regs, SOC_USB_DCFG, reg);
}


static void __soc_usb_gadget_set_speed(struct soc_usb *su)
{
	enum usb_device_speed	speed;
	u32			reg;

	speed = su->gadget_max_speed;
	if (speed == USB_SPEED_UNKNOWN || speed > su->maximum_speed)
		speed = su->maximum_speed;

    if (speed == USB_SPEED_SUPER_PLUS &&
	    SOC_USB_IP_IS(SOC_USB32)) {
		__soc_usb_gadget_set_ssp_rate(su);
		return;
	}

	reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
	reg &= ~(SOC_USB_DCFG_SPEED_MASK);

	switch (speed) {
	case USB_SPEED_FULL:
		reg |= SOC_USB_DCFG_FULLSPEED;
		break;
	case USB_SPEED_HIGH:
		reg |= SOC_USB_DCFG_HIGHSPEED;
		break;
	case USB_SPEED_SUPER:
		reg |= SOC_USB_DCFG_SUPERSPEED;
		break;
	case USB_SPEED_SUPER_PLUS:
		if (SOC_USB_IP_IS(SOC_USB))
			reg |= SOC_USB_DCFG_SUPERSPEED;
		else
			reg |= SOC_USB_DCFG_SUPERSPEED_PLUS;
		break;
	default:
		dev_err(su->dev, "invalid speed (%d)\n", speed);

		if (SOC_USB_IP_IS(SOC_USB))
			reg |= SOC_USB_DCFG_SUPERSPEED;
		else
			reg |= SOC_USB_DCFG_SUPERSPEED_PLUS;
	}

    if (SOC_USB_IP_IS(SOC_USB32) &&
	    speed > USB_SPEED_UNKNOWN &&
	    speed < USB_SPEED_SUPER_PLUS)
		reg &= ~SOC_USB_DCFG_NUMLANES(~0);

	soc_usb_writel(su->regs, SOC_USB_DCFG, reg);
}

static int soc_usb_gadget_run_stop(struct soc_usb *su, int is_on)
{
	u32			reg;
	u32			timeout = 2000;

	if (pm_runtime_suspended(su->dev))
		return 0;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	if (is_on) {
		reg &= ~SOC_USB_DCTL_KEEP_CONNECT;
		reg |= SOC_USB_DCTL_RUN_STOP;

		__soc_usb_gadget_set_speed(su);
		su->pullups_connected = true;
	} else {
		reg &= ~SOC_USB_DCTL_RUN_STOP;

		su->pullups_connected = false;
	}

	soc_usb_gadget_dctl_write_safe(su, reg);

	do {
		usleep_range(1000, 2000);
		reg = soc_usb_readl(su->regs, SOC_USB_DSTS);
		reg &= SOC_USB_DSTS_DEVCTRLHLT;
	} while (--timeout && !(!is_on ^ !reg));

	if (!timeout)
		return -ETIMEDOUT;

	return 0;
}

static void soc_usb_gadget_disable_irq(struct soc_usb *su);
static void __soc_usb_gadget_stop(struct soc_usb *su);
static int __soc_usb_gadget_start(struct soc_usb *su);

static int soc_usb_gadget_soft_disconnect(struct soc_usb *su)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&su->lock, flags);
	if (!su->pullups_connected) {
		spin_unlock_irqrestore(&su->lock, flags);
		return 0;
	}

	su->connected = false;

	/*
	 * Attempt to end pending SETUP status phase, and not wait for the
	 * function to do so.
	 */
	if (su->delayed_status)
		soc_usb_ep0_send_delayed_status(su);

	soc_usb_stop_active_transfers(su);
	spin_unlock_irqrestore(&su->lock, flags);

	/*
	 * Per databook, when we want to stop the gadget, if a control transfer
	 * is still in process, complete it and get the core into setup phase.
	 * In case the host is unresponsive to a SETUP transaction, forcefully
	 * stall the transfer, and move back to the SETUP phase, so that any
	 * pending endxfers can be executed.
	 */
	if (su->ep0state != EP0_SETUP_PHASE) {
		reinit_completion(&su->ep0_in_setup);

		ret = wait_for_completion_timeout(&su->ep0_in_setup,
				msecs_to_jiffies(SOC_USB_PULL_UP_TIMEOUT));
		if (ret == 0) {
			dev_warn(su->dev, "wait for SETUP phase timed out\n");
			spin_lock_irqsave(&su->lock, flags);
			soc_usb_ep0_reset_state(su);
			spin_unlock_irqrestore(&su->lock, flags);
		}
	}

	/*
	 * Note: if the GEVNTCOUNT indicates events in the event buffer, the
	 * driver needs to acknowledge them before the controller can halt.
	 * Simply let the interrupt handler acknowledges and handle the
	 * remaining event generated by the controller while polling for
	 * DSTS.DEVCTLHLT.
	 */
	ret = soc_usb_gadget_run_stop(su, false);

	/*
	 * Stop the gadget after controller is halted, so that if needed, the
	 * events to update EP0 state can still occur while the run/stop
	 * routine polls for the halted state.  DEVTEN is cleared as part of
	 * gadget stop.
	 */
	spin_lock_irqsave(&su->lock, flags);
	__soc_usb_gadget_stop(su);
	spin_unlock_irqrestore(&su->lock, flags);

	return ret;
}

static int soc_usb_gadget_soft_connect(struct soc_usb *su)
{
	int ret;

	/*
	 * In the  SOC USB 1.90a programming guide section
	 * 4.1.9, it specifies that for a reconnect after a
	 * device-initiated disconnect requires a core soft reset
	 * (DCTL.CSftRst) before enabling the run/stop bit.
	 */
	ret = soc_usb_core_soft_reset(su);
	if (ret)
		return ret;

	soc_usb_event_buffers_setup(su);
	__soc_usb_gadget_start(su);
	return soc_usb_gadget_run_stop(su, true);
}

static int soc_usb_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct soc_usb		*su = gadget_to_su(g);
	int			ret;

	is_on = !!is_on;

	su->softconnect = is_on;

	/*
	 * Avoid issuing a runtime resume if the device is already in the
	 * suspended state during gadget disconnect.  SOC_USB gadget was already
	 * halted/stopped during runtime suspend.
	 */
	if (!is_on) {
		pm_runtime_barrier(su->dev);
		if (pm_runtime_suspended(su->dev))
			return 0;
	}

	/*
	 * Check the return value for successful resume, or error.  For a
	 * successful resume, the SOC_USB runtime PM resume routine will handle
	 * the run stop sequence, so avoid duplicate operations here.
	 */
	ret = pm_runtime_get_sync(su->dev);
	if (!ret || ret < 0) {
		pm_runtime_put(su->dev);
		if (ret < 0)
			pm_runtime_set_suspended(su->dev);
		return ret;
	}

	if (su->pullups_connected == is_on) {
		pm_runtime_put(su->dev);
		return 0;
	}

	synchronize_irq(su->irq_gadget);

	if (!is_on)
		ret = soc_usb_gadget_soft_disconnect(su);
	else
		ret = soc_usb_gadget_soft_connect(su);

	pm_runtime_put(su->dev);

	return ret;
}

static void soc_usb_gadget_enable_irq(struct soc_usb *su)
{
	u32			reg;

	/* Enable all but Start and End of Frame IRQs */
	reg = (SOC_USB_DEVTEN_ERRTICERREN |
			SOC_USB_DEVTEN_WKUPEVTEN |
			SOC_USB_DEVTEN_CONNECTDONEEN |
			SOC_USB_DEVTEN_U3L2L1SUSPEN |
			SOC_USB_DEVTEN_USBRSTEN |
			SOC_USB_DEVTEN_DISCONNEVTEN);

	soc_usb_writel(su->regs, SOC_USB_DEVTEN, reg);
}

static void soc_usb_gadget_disable_irq(struct soc_usb *su)
{
	/* mask all interrupts */
	soc_usb_writel(su->regs, SOC_USB_DEVTEN, 0x00);
}

static irqreturn_t soc_usb_interrupt(int irq, void *_su);
static irqreturn_t soc_usb_thread_interrupt(int irq, void *_su);

/**
 * soc_usb_gadget_setup_nump - calculate and initialize NUMP field of %SOC_USB_DCFG
 * @su: pointer to our context structure
 *
 * The following looks like complex but it's actually very simple. In order to
 * calculate the number of packets we can burst at once on OUT transfers, we're
 * gonna use RxFIFO size.
 *
 * To calculate RxFIFO size we need two numbers:
 * MDWIDTH = size, in bits, of the internal memory bus
 * RAM2_DEPTH = depth, in MDWIDTH, of internal RAM2 (where RxFIFO sits)
 *
 * Given these two numbers, the formula is simple:
 *
 * RxFIFO Size = (RAM2_DEPTH * MDWIDTH / 8) - 24 - 16;
 *
 * 24 bytes is for 3x SETUP packets
 * 16 bytes is a clock domain crossing tolerance
 *
 * Given RxFIFO Size, NUMP = RxFIFOSize / 1024;
 */
static void soc_usb_gadget_setup_nump(struct soc_usb *su)
{
	u32 ram2_depth;
	u32 mdwidth;
	u32 nump;
	u32 reg;

	ram2_depth = SOC_USB_GHWPARAMS7_RAM2_DEPTH(su->hwparams.hwparams7);
	mdwidth = soc_usb_mdwidth(su);

	nump = ((ram2_depth * mdwidth / 8) - 24 - 16) / 1024;
	nump = min_t(u32, nump, 16);

	/* update NumP */
	reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
	reg &= ~SOC_USB_DCFG_NUMP_MASK;
	reg |= nump << SOC_USB_DCFG_NUMP_SHIFT;
	soc_usb_writel(su->regs, SOC_USB_DCFG, reg);
}

static int __soc_usb_gadget_start(struct soc_usb *su)
{
	struct soc_usb_ep		*dep;
	int			ret = 0;
	u32			reg;

	/*
	 * Use IMOD if enabled via su->imod_interval. Otherwise, if
	 * the core supports IMOD, disable it.
	 */
	if (su->imod_interval) {
		soc_usb_writel(su->regs, SOC_USB_DEV_IMOD(0), su->imod_interval);
		soc_usb_writel(su->regs, SOC_USB_GEVNTCOUNT(0), SOC_USB_GEVNTCOUNT_EHB);
	} else if (soc_usb_has_imod(su)) {
		soc_usb_writel(su->regs, SOC_USB_DEV_IMOD(0), 0);
	}

	/*
	 * We are telling soc_usb that we want to use DCFG.NUMP as ACK TP's NUMP
	 * field instead of letting soc_usb itself calculate that automatically.
	 *
	 * This way, we maximize the chances that we'll be able to get several
	 * bursts of data without going through any sort of endpoint throttling.
	 */
	reg = soc_usb_readl(su->regs, SOC_USB_GRXTHRCFG);
	if (SOC_USB_IP_IS(SOC_USB))
		reg &= ~SOC_USB_GRXTHRCFG_PKTCNTSEL;
	else
		reg &= ~SOC_USB32_GRXTHRCFG_PKTCNTSEL;

	soc_usb_writel(su->regs, SOC_USB_GRXTHRCFG, reg);

	soc_usb_gadget_setup_nump(su);

	/* Start with SuperSpeed Default */
	soc_usb_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = su->eps[0];
	dep->flags = 0;
	ret = __soc_usb_gadget_ep_enable(dep, SOC_USB_DEPCFG_ACTION_INIT);
	if (ret) {
		dev_err(su->dev, "failed to enable %s\n", dep->name);
		goto err0;
	}

	dep = su->eps[1];
	dep->flags = 0;
	ret = __soc_usb_gadget_ep_enable(dep, SOC_USB_DEPCFG_ACTION_INIT);
	if (ret) {
		dev_err(su->dev, "failed to enable %s\n", dep->name);
		goto err1;
	}

	/* begin to receive SETUP packets */
	su->ep0state = EP0_SETUP_PHASE;
	su->ep0_bounced = false;
	su->link_state = SOC_USB_LINK_STATE_SS_DIS;
	su->delayed_status = false;
	soc_usb_ep0_out_start(su);

	soc_usb_gadget_enable_irq(su);
	soc_usb_enable_susphy(su, true);

	return 0;

err1:
	__soc_usb_gadget_ep_disable(su->eps[0]);

err0:
	return ret;
}

static int soc_usb_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;
	int			ret;
	int			irq;

	irq = su->irq_gadget;
	ret = request_threaded_irq(irq, soc_usb_interrupt, soc_usb_thread_interrupt,
			IRQF_SHARED, "soc_usb", su->ev_buf);
	if (ret) {
		dev_err(su->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		return ret;
	}

	spin_lock_irqsave(&su->lock, flags);
	su->gadget_driver	= driver;
	spin_unlock_irqrestore(&su->lock, flags);

	if (su->sys_wakeup)
		device_wakeup_enable(su->sysdev);

	return 0;
}

static void __soc_usb_gadget_stop(struct soc_usb *su)
{
	soc_usb_gadget_disable_irq(su);
	__soc_usb_gadget_ep_disable(su->eps[0]);
	__soc_usb_gadget_ep_disable(su->eps[1]);
}

static int soc_usb_gadget_stop(struct usb_gadget *g)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;

	if (su->sys_wakeup)
		device_wakeup_disable(su->sysdev);

	spin_lock_irqsave(&su->lock, flags);
	su->gadget_driver	= NULL;
	su->max_cfg_eps = 0;
	spin_unlock_irqrestore(&su->lock, flags);

	free_irq(su->irq_gadget, su->ev_buf);

	return 0;
}

static void soc_usb_gadget_config_params(struct usb_gadget *g,
				      struct usb_dcd_config_params *params)
{
	struct soc_usb		*su = gadget_to_su(g);

	params->besl_baseline = USB_DEFAULT_BESL_UNSPECIFIED;
	params->besl_deep = USB_DEFAULT_BESL_UNSPECIFIED;

	/* Recommended BESL */
	/*
	* If the recommended BESL baseline is 0 or if the BESL deep is
	* less than 2, Microsoft's Windows 10 host usb stack will issue
	* a usb reset immediately after it receives the extended BOS
	* descriptor and the enumeration will fail. To maintain
	* compatibility with the Windows' usb stack, let's set the
	* recommended BESL baseline to 1 and clamp the BESL deep to be
	* within 2 to 15.
	*/
	params->besl_baseline = 1;
	if (su->is_utmi_l1_suspend)
		params->besl_deep =
			clamp_t(u8, su->hird_threshold, 2, 15);

	/* U1 Device exit Latency */
	if (su->dis_u1_entry_quirk)
		params->bU1devExitLat = 0;
	else
		params->bU1devExitLat = SOC_USB_DEFAULT_U1_DEV_EXIT_LAT;

	/* U2 Device exit Latency */
	if (su->dis_u2_entry_quirk)
		params->bU2DevExitLat = 0;
	else
		params->bU2DevExitLat =
				cpu_to_le16(SOC_USB_DEFAULT_U2_DEV_EXIT_LAT);
}

static void soc_usb_gadget_set_speed(struct usb_gadget *g,
				  enum usb_device_speed speed)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;

	spin_lock_irqsave(&su->lock, flags);
	su->gadget_max_speed = speed;
	spin_unlock_irqrestore(&su->lock, flags);
}

static void soc_usb_gadget_set_ssp_rate(struct usb_gadget *g,
				     enum usb_ssp_rate rate)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;

	spin_lock_irqsave(&su->lock, flags);
	su->gadget_max_speed = USB_SPEED_SUPER_PLUS;
	su->gadget_ssp_rate = rate;
	spin_unlock_irqrestore(&su->lock, flags);
}

static int soc_usb_gadget_vbus_draw(struct usb_gadget *g, unsigned int mA)
{
	struct soc_usb		*su = gadget_to_su(g);
	union power_supply_propval	val = {0};
	int				ret;

	if (su->usb2_phy)
		return usb_phy_set_power(su->usb2_phy, mA);

	if (!su->usb_psy)
		return -EOPNOTSUPP;

	val.intval = 1000 * mA;
	ret = power_supply_set_property(su->usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);

	return ret;
}

/**
 * soc_usb_gadget_check_config - ensure soc_usb can support the USB configuration
 * @g: pointer to the USB gadget
 *
 * Used to record the maximum number of endpoints being used in a USB composite
 * device. (across all configurations)  This is to be used in the calculation
 * of the TXFIFO sizes when resizing internal memory for individual endpoints.
 * It will help ensured that the resizing logic reserves enough space for at
 * least one max packet.
 */
static int soc_usb_gadget_check_config(struct usb_gadget *g)
{
	struct soc_usb *su = gadget_to_su(g);
	struct usb_ep *ep;
	int fifo_size = 0;
	int ram1_depth;
	int ep_num = 0;

	if (!su->do_fifo_resize)
		return 0;

	list_for_each_entry(ep, &g->ep_list, ep_list) {
		/* Only interested in the IN endpoints */
		if (ep->claimed && (ep->address & USB_DIR_IN))
			ep_num++;
	}

	if (ep_num <= su->max_cfg_eps)
		return 0;

	/* Update the max number of eps in the composition */
	su->max_cfg_eps = ep_num;

	fifo_size = soc_usb_gadget_calc_tx_fifo_size(su, su->max_cfg_eps);
	/* Based on the equation, increment by one for every ep */
	fifo_size += su->max_cfg_eps;

	/* Check if we can fit a single fifo per endpoint */
	ram1_depth = SOC_USB_RAM1_DEPTH(su->hwparams.hwparams7);
	if (fifo_size > ram1_depth)
		return -ENOMEM;

	return 0;
}

static void soc_usb_gadget_async_callbacks(struct usb_gadget *g, bool enable)
{
	struct soc_usb		*su = gadget_to_su(g);
	unsigned long		flags;

	spin_lock_irqsave(&su->lock, flags);
	su->async_callbacks = enable;
	spin_unlock_irqrestore(&su->lock, flags);
}

static const struct usb_gadget_ops soc_usb_gadget_ops = {
	.get_frame		= soc_usb_gadget_get_frame,
	.wakeup			= soc_usb_gadget_wakeup,
	.func_wakeup		= soc_usb_gadget_func_wakeup,
	.set_remote_wakeup	= soc_usb_gadget_set_remote_wakeup,
	.set_selfpowered	= soc_usb_gadget_set_selfpowered,
	.pullup			= soc_usb_gadget_pullup,
	.udc_start		= soc_usb_gadget_start,
	.udc_stop		= soc_usb_gadget_stop,
	.udc_set_speed		= soc_usb_gadget_set_speed,
	.udc_set_ssp_rate	= soc_usb_gadget_set_ssp_rate,
	.get_config_params	= soc_usb_gadget_config_params,
	.vbus_draw		= soc_usb_gadget_vbus_draw,
	.check_config		= soc_usb_gadget_check_config,
	.udc_async_callbacks	= soc_usb_gadget_async_callbacks,
};

/* -------------------------------------------------------------------------- */

static int soc_usb_gadget_init_control_endpoint(struct soc_usb_ep *dep)
{
	struct soc_usb *su = dep->su;

	usb_ep_set_maxpacket_limit(&dep->endpoint, 512);
	dep->endpoint.maxburst = 1;
	dep->endpoint.ops = &soc_usb_gadget_ep0_ops;
	if (!dep->direction)
		su->gadget->ep0 = &dep->endpoint;

	dep->endpoint.caps.type_control = true;

	return 0;
}

static int soc_usb_gadget_init_in_endpoint(struct soc_usb_ep *dep)
{
	struct soc_usb *su = dep->su;
	u32 mdwidth;
	int size;
	int maxpacket;

	mdwidth = soc_usb_mdwidth(su);

	/* MDWIDTH is represented in bits, we need it in bytes */
	mdwidth /= 8;

	size = soc_usb_readl(su->regs, SOC_USB_GTXFIFOSIZ(dep->number >> 1));
	if (SOC_USB_IP_IS(SOC_USB))
		size = SOC_USB_GTXFIFOSIZ_TXFDEP(size);
	else
		size = SOC_USB32_GTXFIFOSIZ_TXFDEP(size);

	maxpacket = mdwidth * ((size - 1) - 1) - mdwidth;

	/* Functionally, space for one max packet is sufficient */
	size = min_t(int, maxpacket, 1024);
	usb_ep_set_maxpacket_limit(&dep->endpoint, size);

	dep->endpoint.max_streams = 16;
	dep->endpoint.ops = &soc_usb_gadget_ep_ops;
	list_add_tail(&dep->endpoint.ep_list,
			&su->gadget->ep_list);
	dep->endpoint.caps.type_iso = true;
	dep->endpoint.caps.type_bulk = true;
	dep->endpoint.caps.type_int = true;

	return soc_usb_alloc_trb_pool(dep);
}

static int soc_usb_gadget_init_out_endpoint(struct soc_usb_ep *dep)
{
	struct soc_usb *su = dep->su;
	u32 mdwidth;
	int size;

	mdwidth = soc_usb_mdwidth(su);

	/* MDWIDTH is represented in bits, convert to bytes */
	mdwidth /= 8;

	/* All OUT endpoints share a single RxFIFO space */
	size = soc_usb_readl(su->regs, SOC_USB_GRXFIFOSIZ(0));
	if (SOC_USB_IP_IS(SOC_USB))
		size = SOC_USB_GRXFIFOSIZ_RXFDEP(size);
	else
		size = SOC_USB32_GRXFIFOSIZ_RXFDEP(size);

	/* FIFO depth is in MDWDITH bytes */
	size *= mdwidth;

	/*
	 * To meet performance requirement, a minimum recommended RxFIFO size
	 * is defined as follow:
	 * RxFIFO size >= (3 x MaxPacketSize) +
	 * (3 x 8 bytes setup packets size) + (16 bytes clock crossing margin)
	 *
	 * Then calculate the max packet limit as below.
	 */
	size -= (3 * 8) + 16;
	if (size < 0)
		size = 0;
	else
		size /= 3;

	usb_ep_set_maxpacket_limit(&dep->endpoint, size);
	dep->endpoint.max_streams = 16;
	dep->endpoint.ops = &soc_usb_gadget_ep_ops;
	list_add_tail(&dep->endpoint.ep_list,
			&su->gadget->ep_list);
	dep->endpoint.caps.type_iso = true;
	dep->endpoint.caps.type_bulk = true;
	dep->endpoint.caps.type_int = true;

	return soc_usb_alloc_trb_pool(dep);
}

static int soc_usb_gadget_init_endpoint(struct soc_usb *su, u8 epnum)
{
	struct soc_usb_ep			*dep;
	bool				direction = epnum & 1;
	int				ret;
	u8				num = epnum >> 1;

	dep = kzalloc(sizeof(*dep), GFP_KERNEL);
	if (!dep)
		return -ENOMEM;

	dep->su = su;
	dep->number = epnum;
	dep->direction = direction;
	dep->regs = su->regs + SOC_USB_DEP_BASE(epnum);
	su->eps[epnum] = dep;
	dep->combo_num = 0;
	dep->start_cmd_status = 0;

	snprintf(dep->name, sizeof(dep->name), "ep%u%s", num,
			direction ? "in" : "out");

	dep->endpoint.name = dep->name;

	if (!(dep->number > 1)) {
		dep->endpoint.desc = &soc_usb_gadget_ep0_desc;
		dep->endpoint.comp_desc = NULL;
	}

	if (num == 0)
		ret = soc_usb_gadget_init_control_endpoint(dep);
	else if (direction)
		ret = soc_usb_gadget_init_in_endpoint(dep);
	else
		ret = soc_usb_gadget_init_out_endpoint(dep);

	if (ret)
		return ret;

	dep->endpoint.caps.dir_in = direction;
	dep->endpoint.caps.dir_out = !direction;

	INIT_LIST_HEAD(&dep->pending_list);
	INIT_LIST_HEAD(&dep->started_list);
	INIT_LIST_HEAD(&dep->cancelled_list);

	soc_usb_debugfs_create_endpoint_dir(dep);

	return 0;
}

static int soc_usb_gadget_init_endpoints(struct soc_usb *su, u8 total)
{
	u8				epnum;

	INIT_LIST_HEAD(&su->gadget->ep_list);

	for (epnum = 0; epnum < total; epnum++) {
		int			ret;

		ret = soc_usb_gadget_init_endpoint(su, epnum);
		if (ret)
			return ret;
	}

	return 0;
}

static void soc_usb_gadget_free_endpoints(struct soc_usb *su)
{
	struct soc_usb_ep			*dep;
	u8				epnum;

	for (epnum = 0; epnum < SOC_USB_ENDPOINTS_NUM; epnum++) {
		dep = su->eps[epnum];
		if (!dep)
			continue;
		/*
		 * Physical endpoints 0 and 1 are special; they form the
		 * bi-directional USB endpoint 0.
		 *
		 * For those two physical endpoints, we don't allocate a TRB
		 * pool nor do we add them the endpoints list. Due to that, we
		 * shouldn't do these two operations otherwise we would end up
		 * with all sorts of bugs when removing soc_usb.ko.
		 */
		if (epnum != 0 && epnum != 1) {
			soc_usb_free_trb_pool(dep);
			list_del(&dep->endpoint.ep_list);
		}

		soc_usb_debugfs_remove_endpoint_dir(dep);
		kfree(dep);
	}
}

/* -------------------------------------------------------------------------- */

static int soc_usb_gadget_ep_reclaim_completed_trb(struct soc_usb_ep *dep,
		struct soc_usb_request *req, struct soc_usb_trb *trb,
		const struct soc_usb_event_depevt *event, int status, int chain)
{
	unsigned int		count;

	soc_usb_ep_inc_deq(dep);

	trace_soc_usb_complete_trb(dep, trb);
	req->num_trbs--;

	/*
	 * If we're in the middle of series of chained TRBs and we
	 * receive a short transfer along the way, SOC_USB will skip
	 * through all TRBs including the last TRB in the chain (the
	 * where CHN bit is zero. SOC_USB will also avoid clearing HWO
	 * bit and SW has to do it manually.
	 *
	 * We're going to do that here to avoid problems of HW trying
	 * to use bogus TRBs for transfers.
	 */
	if (chain && (trb->ctrl & SOC_USB_TRB_CTRL_HWO))
		trb->ctrl &= ~SOC_USB_TRB_CTRL_HWO;

	/*
	 * For isochronous transfers, the first TRB in a service interval must
	 * have the Isoc-First type. Track and report its interval frame number.
	 */
	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
	    (trb->ctrl & SOC_USB_TRBCTL_ISOCHRONOUS_FIRST)) {
		unsigned int frame_number;

		frame_number = SOC_USB_TRB_CTRL_GET_SID_SOFN(trb->ctrl);
		frame_number &= ~(dep->interval - 1);
		req->request.frame_number = frame_number;
	}

	/*
	 * We use bounce buffer for requests that needs extra TRB or OUT ZLP. If
	 * this TRB points to the bounce buffer address, it's a MPS alignment
	 * TRB. Don't add it to req->remaining calculation.
	 */
	if (trb->bpl == lower_32_bits(dep->su->bounce_addr) &&
	    trb->bph == upper_32_bits(dep->su->bounce_addr)) {
		trb->ctrl &= ~SOC_USB_TRB_CTRL_HWO;
		return 1;
	}

	count = trb->size & SOC_USB_TRB_SIZE_MASK;
	req->remaining += count;

	if ((trb->ctrl & SOC_USB_TRB_CTRL_HWO) && status != -ESHUTDOWN)
		return 1;

	if (event->status & DEPEVT_STATUS_SHORT && !chain)
		return 1;

	if ((trb->ctrl & SOC_USB_TRB_CTRL_ISP_IMI) &&
	    SOC_USB_TRB_SIZE_TRBSTS(trb->size) == SOC_USB_TRBSTS_MISSED_ISOC)
		return 1;

	if ((trb->ctrl & SOC_USB_TRB_CTRL_IOC) ||
	    (trb->ctrl & SOC_USB_TRB_CTRL_LST))
		return 1;

	return 0;
}

static int soc_usb_gadget_ep_reclaim_trb_sg(struct soc_usb_ep *dep,
		struct soc_usb_request *req, const struct soc_usb_event_depevt *event,
		int status)
{
	struct soc_usb_trb *trb = &dep->trb_pool[dep->trb_dequeue];
	struct scatterlist *sg = req->sg;
	struct scatterlist *s;
	unsigned int num_queued = req->num_queued_sgs;
	unsigned int i;
	int ret = 0;

	for_each_sg(sg, s, num_queued, i) {
		trb = &dep->trb_pool[dep->trb_dequeue];

		req->sg = sg_next(s);
		req->num_queued_sgs--;

		ret = soc_usb_gadget_ep_reclaim_completed_trb(dep, req,
				trb, event, status, true);
		if (ret)
			break;
	}

	return ret;
}

static int soc_usb_gadget_ep_reclaim_trb_linear(struct soc_usb_ep *dep,
		struct soc_usb_request *req, const struct soc_usb_event_depevt *event,
		int status)
{
	struct soc_usb_trb *trb = &dep->trb_pool[dep->trb_dequeue];

	return soc_usb_gadget_ep_reclaim_completed_trb(dep, req, trb,
			event, status, false);
}

static bool soc_usb_gadget_ep_request_completed(struct soc_usb_request *req)
{
	return req->num_pending_sgs == 0 && req->num_queued_sgs == 0;
}

static int soc_usb_gadget_ep_cleanup_completed_request(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event,
		struct soc_usb_request *req, int status)
{
	int request_status;
	int ret;

	if (req->request.num_mapped_sgs)
		ret = soc_usb_gadget_ep_reclaim_trb_sg(dep, req, event,
				status);
	else
		ret = soc_usb_gadget_ep_reclaim_trb_linear(dep, req, event,
				status);

	req->request.actual = req->request.length - req->remaining;

	if (!soc_usb_gadget_ep_request_completed(req))
		goto out;

	if (req->needs_extra_trb) {
		ret = soc_usb_gadget_ep_reclaim_trb_linear(dep, req, event,
				status);
		req->needs_extra_trb = false;
	}

	/*
	 * The event status only reflects the status of the TRB with IOC set.
	 * For the requests that don't set interrupt on completion, the driver
	 * needs to check and return the status of the completed TRBs associated
	 * with the request. Use the status of the last TRB of the request.
	 */
	if (req->request.no_interrupt) {
		struct soc_usb_trb *trb;

		trb = soc_usb_ep_prev_trb(dep, dep->trb_dequeue);
		switch (SOC_USB_TRB_SIZE_TRBSTS(trb->size)) {
		case SOC_USB_TRBSTS_MISSED_ISOC:
			/* Isoc endpoint only */
			request_status = -EXDEV;
			break;
		case SOC_USB_TRB_STS_XFER_IN_PROG:
			/* Applicable when End Transfer with ForceRM=0 */
		case SOC_USB_TRBSTS_SETUP_PENDING:
			/* Control endpoint only */
		case SOC_USB_TRBSTS_OK:
		default:
			request_status = 0;
			break;
		}
	} else {
		request_status = status;
	}

	soc_usb_gadget_giveback(dep, req, request_status);

out:
	return ret;
}

static void soc_usb_gadget_ep_cleanup_completed_requests(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event, int status)
{
	struct soc_usb_request	*req;

	while (!list_empty(&dep->started_list)) {
		int ret;

		req = next_request(&dep->started_list);
		ret = soc_usb_gadget_ep_cleanup_completed_request(dep, event,
				req, status);
		if (ret)
			break;
		/*
		 * The endpoint is disabled, let the soc_usb_remove_requests()
		 * handle the cleanup.
		 */
		if (!dep->endpoint.desc)
			break;
	}
}

static bool soc_usb_gadget_ep_should_continue(struct soc_usb_ep *dep)
{
	struct soc_usb_request	*req;
	struct soc_usb		*su = dep->su;

	if (!dep->endpoint.desc || !su->pullups_connected ||
	    !su->connected)
		return false;

	if (!list_empty(&dep->pending_list))
		return true;

	/*
	 * We only need to check the first entry of the started list. We can
	 * assume the completed requests are removed from the started list.
	 */
	req = next_request(&dep->started_list);
	if (!req)
		return false;

	return !soc_usb_gadget_ep_request_completed(req);
}

static void soc_usb_gadget_endpoint_frame_from_event(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event)
{
	dep->frame_number = event->parameters;
}

static bool soc_usb_gadget_endpoint_trbs_complete(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event, int status)
{
	bool			no_started_trb = true;

	soc_usb_gadget_ep_cleanup_completed_requests(dep, event, status);

	if (dep->flags & SOC_USB_EP_END_TRANSFER_PENDING)
		goto out;

	if (!dep->endpoint.desc)
		return no_started_trb;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
		list_empty(&dep->started_list) &&
		(list_empty(&dep->pending_list) || status == -EXDEV))
		soc_usb_stop_active_transfer(dep, true, true);
	else if (soc_usb_gadget_ep_should_continue(dep))
		if (__soc_usb_gadget_kick_transfer(dep) == 0)
			no_started_trb = false;

out:

	return no_started_trb;
}

static void soc_usb_gadget_endpoint_transfer_in_progress(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event)
{
	int status = 0;

	if (!dep->endpoint.desc)
		return;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc))
		soc_usb_gadget_endpoint_frame_from_event(dep, event);

	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	if (event->status & DEPEVT_STATUS_MISSED_ISOC)
		status = -EXDEV;

	soc_usb_gadget_endpoint_trbs_complete(dep, event, status);
}

static void soc_usb_gadget_endpoint_transfer_complete(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event)
{
	int status = 0;

	dep->flags &= ~SOC_USB_EP_TRANSFER_STARTED;

	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	if (soc_usb_gadget_endpoint_trbs_complete(dep, event, status))
		dep->flags &= ~SOC_USB_EP_WAIT_TRANSFER_COMPLETE;
}

static void soc_usb_gadget_endpoint_transfer_not_ready(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event)
{
	soc_usb_gadget_endpoint_frame_from_event(dep, event);

	/*
	 * The XferNotReady event is generated only once before the endpoint
	 * starts. It will be generated again when END_TRANSFER command is
	 * issued. For some controller versions, the XferNotReady event may be
	 * generated while the END_TRANSFER command is still in process. Ignore
	 * it and wait for the next XferNotReady event after the command is
	 * completed.
	 */
	if (dep->flags & SOC_USB_EP_END_TRANSFER_PENDING)
		return;

	(void) __soc_usb_gadget_start_isoc(dep);
}

static void soc_usb_gadget_endpoint_command_complete(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event)
{
	u8 cmd = DEPEVT_PARAMETER_CMD(event->parameters);

	if (cmd != SOC_USB_DEPCMD_ENDTRANSFER)
		return;

	/*
	 * The END_TRANSFER command will cause the controller to generate a
	 * NoStream Event, and it's not due to the host DP NoStream rejection.
	 * Ignore the next NoStream event.
	 */
	if (dep->stream_capable)
		dep->flags |= SOC_USB_EP_IGNORE_NEXT_NOSTREAM;

	dep->flags &= ~SOC_USB_EP_END_TRANSFER_PENDING;
	dep->flags &= ~SOC_USB_EP_TRANSFER_STARTED;
	soc_usb_gadget_ep_cleanup_cancelled_requests(dep);

	if (dep->flags & SOC_USB_EP_PENDING_CLEAR_STALL) {
		struct soc_usb *su = dep->su;

		dep->flags &= ~SOC_USB_EP_PENDING_CLEAR_STALL;
		if (soc_usb_send_clear_stall_ep_cmd(dep)) {
			struct usb_ep *ep0 = &su->eps[0]->endpoint;

			dev_err(su->dev, "failed to clear STALL on %s\n", dep->name);
			if (su->delayed_status)
				__soc_usb_gadget_ep0_set_halt(ep0, 1);
			return;
		}

		dep->flags &= ~(SOC_USB_EP_STALL | SOC_USB_EP_WEDGE);
		if (su->clear_stall_protocol == dep->number)
			soc_usb_ep0_send_delayed_status(su);
	}

	if ((dep->flags & SOC_USB_EP_DELAY_START) &&
	    !usb_endpoint_xfer_isoc(dep->endpoint.desc))
		__soc_usb_gadget_kick_transfer(dep);

	dep->flags &= ~SOC_USB_EP_DELAY_START;
}

static void soc_usb_gadget_endpoint_stream_event(struct soc_usb_ep *dep,
		const struct soc_usb_event_depevt *event)
{

	if (event->status == DEPEVT_STREAMEVT_FOUND) {
		dep->flags |= SOC_USB_EP_FIRST_STREAM_PRIMED;
		goto out;
	}

	/* Note: NoStream rejection event param value is 0 and not 0xFFFF */
	switch (event->parameters) {
	case DEPEVT_STREAM_PRIME:
		/*
		 * If the host can properly transition the endpoint state from
		 * idle to prime after a NoStream rejection, there's no need to
		 * force restarting the endpoint to reinitiate the stream. To
		 * simplify the check, assume the host follows the USB spec if
		 * it primed the endpoint more than once.
		 */
		if (dep->flags & SOC_USB_EP_FORCE_RESTART_STREAM) {
			if (dep->flags & SOC_USB_EP_FIRST_STREAM_PRIMED)
				dep->flags &= ~SOC_USB_EP_FORCE_RESTART_STREAM;
			else
				dep->flags |= SOC_USB_EP_FIRST_STREAM_PRIMED;
		}

		break;
	case DEPEVT_STREAM_NOSTREAM:
		if ((dep->flags & SOC_USB_EP_IGNORE_NEXT_NOSTREAM) ||
		    !(dep->flags & SOC_USB_EP_FORCE_RESTART_STREAM) ||
		    (!(dep->flags & SOC_USB_EP_WAIT_TRANSFER_COMPLETE)))
			break;

		dep->flags |= SOC_USB_EP_DELAY_START;
		soc_usb_stop_active_transfer(dep, true, true);
		return;
	}

out:
	dep->flags &= ~SOC_USB_EP_IGNORE_NEXT_NOSTREAM;
}

static void soc_usb_endpoint_interrupt(struct soc_usb *su,
		const struct soc_usb_event_depevt *event)
{
	struct soc_usb_ep		*dep;
	u8			epnum = event->endpoint_number;

	dep = su->eps[epnum];

	if (!(dep->flags & SOC_USB_EP_ENABLED)) {
		if ((epnum > 1) && !(dep->flags & SOC_USB_EP_TRANSFER_STARTED))
			return;

		/* Handle only EPCMDCMPLT when EP disabled */
		if ((event->endpoint_event != SOC_USB_DEPEVT_EPCMDCMPLT) &&
			!(epnum <= 1 && event->endpoint_event == SOC_USB_DEPEVT_XFERCOMPLETE))
			return;
	}

	if (epnum == 0 || epnum == 1) {
		soc_usb_ep0_interrupt(su, event);
		return;
	}

	switch (event->endpoint_event) {
	case SOC_USB_DEPEVT_XFERINPROGRESS:
		soc_usb_gadget_endpoint_transfer_in_progress(dep, event);
		break;
	case SOC_USB_DEPEVT_XFERNOTREADY:
		soc_usb_gadget_endpoint_transfer_not_ready(dep, event);
		break;
	case SOC_USB_DEPEVT_EPCMDCMPLT:
		soc_usb_gadget_endpoint_command_complete(dep, event);
		break;
	case SOC_USB_DEPEVT_XFERCOMPLETE:
		soc_usb_gadget_endpoint_transfer_complete(dep, event);
		break;
	case SOC_USB_DEPEVT_STREAMEVT:
		soc_usb_gadget_endpoint_stream_event(dep, event);
		break;
	case SOC_USB_DEPEVT_RXTXFIFOEVT:
		break;
	default:
		dev_err(su->dev, "unknown endpoint event %d\n", event->endpoint_event);
		break;
	}
}

static void soc_usb_disconnect_gadget(struct soc_usb *su)
{
	if (su->async_callbacks && su->gadget_driver->disconnect) {
		spin_unlock(&su->lock);
		su->gadget_driver->disconnect(su->gadget);
		spin_lock(&su->lock);
	}
}

static void soc_usb_suspend_gadget(struct soc_usb *su)
{
	if (su->async_callbacks && su->gadget_driver->suspend) {
		spin_unlock(&su->lock);
		su->gadget_driver->suspend(su->gadget);
		spin_lock(&su->lock);
	}
}

static void soc_usb_resume_gadget(struct soc_usb *su)
{
	if (su->async_callbacks && su->gadget_driver->resume) {
		spin_unlock(&su->lock);
		su->gadget_driver->resume(su->gadget);
		spin_lock(&su->lock);
	}
}

static void soc_usb_reset_gadget(struct soc_usb *su)
{
	if (!su->gadget_driver)
		return;

	if (su->async_callbacks && su->gadget->speed != USB_SPEED_UNKNOWN) {
		spin_unlock(&su->lock);
		usb_gadget_udc_reset(su->gadget, su->gadget_driver);
		spin_lock(&su->lock);
	}
}

void soc_usb_stop_active_transfer(struct soc_usb_ep *dep, bool force,
	bool interrupt)
{
	struct soc_usb *su = dep->su;

	/*
	 * Only issue End Transfer command to the control endpoint of a started
	 * Data Phase. Typically we should only do so in error cases such as
	 * invalid/unexpected direction as described in the control transfer
	 * flow of the programming guide.
	 */
	if (dep->number <= 1 && su->ep0state != EP0_DATA_PHASE)
		return;

	if (interrupt && (dep->flags & SOC_USB_EP_DELAY_STOP))
		return;

	if (!(dep->flags & SOC_USB_EP_TRANSFER_STARTED) ||
	    (dep->flags & SOC_USB_EP_END_TRANSFER_PENDING))
		return;

	/*
	 * If a Setup packet is received but yet to DMA out, the controller will
	 * not process the End Transfer command of any endpoint. Polling of its
	 * DEPCMD.CmdAct may block setting up TRB for Setup packet, causing a
	 * timeout. Delay issuing the End Transfer command until the Setup TRB is
	 * prepared.
	 */
	if (su->ep0state != EP0_SETUP_PHASE && !su->delayed_status) {
		dep->flags |= SOC_USB_EP_DELAY_STOP;
		return;
	}

	/*
	 * NOTICE: We are violating what the Databook says about the
	 * EndTransfer command. Ideally we would _always_ wait for the
	 * EndTransfer Command Completion IRQ, but that's causing too
	 * much trouble synchronizing between us and gadget driver.
	 *
	 * We have discussed this with the IP Provider and it was
	 * suggested to giveback all requests here.
	 *
	 * In short, what we're doing is issuing EndTransfer with
	 * CMDIOC bit set and delay kicking transfer until the
	 * EndTransfer command had completed.
	 *
	 * As of IP version 3.10a of the SOC USB IP, the controller
	 * supports a mode to work around the above limitation. The
	 * software can poll the CMDACT bit in the DEPCMD register
	 * after issuing a EndTransfer command. This mode is enabled
	 * by writing GUCTL2[14]. This polling is already done in the
	 * soc_usb_send_gadget_ep_cmd() function so if the mode is
	 * enabled, the EndTransfer command will have completed upon
	 * returning from this function.
	 *
	 * This mode is NOT available on the SOC USB IP.  In this
	 * case, if the IOC bit is not set, then delay by 1ms
	 * after issuing the EndTransfer command.  This allows for the
	 * controller to handle the command completely before SOC_USB
	 * remove requests attempts to unmap USB request buffers.
	 */

	__soc_usb_stop_active_transfer(dep, force, interrupt);
}

static void soc_usb_clear_stall_all_ep(struct soc_usb *su)
{
	u32 epnum;

	for (epnum = 1; epnum < SOC_USB_ENDPOINTS_NUM; epnum++) {
		struct soc_usb_ep *dep;
		int ret;

		dep = su->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & SOC_USB_EP_STALL))
			continue;

		dep->flags &= ~SOC_USB_EP_STALL;

		ret = soc_usb_send_clear_stall_ep_cmd(dep);
		WARN_ON_ONCE(ret);
	}
}

static void soc_usb_gadget_disconnect_interrupt(struct soc_usb *su)
{
	int			reg;

	su->suspended = false;

	soc_usb_gadget_set_link_state(su, SOC_USB_LINK_STATE_RX_DET);

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	reg &= ~SOC_USB_DCTL_INITU1ENA;
	reg &= ~SOC_USB_DCTL_INITU2ENA;
	soc_usb_gadget_dctl_write_safe(su, reg);

	su->connected = false;

	soc_usb_disconnect_gadget(su);

	su->gadget->speed = USB_SPEED_UNKNOWN;
	su->setup_packet_pending = false;
	su->gadget->wakeup_armed = false;
	soc_usb_gadget_enable_linksts_evts(su, false);
	usb_gadget_set_state(su->gadget, USB_STATE_NOTATTACHED);

	soc_usb_ep0_reset_state(su);

	/*
	 * Request PM idle to address condition where usage count is
	 * already decremented to zero, but waiting for the disconnect
	 * interrupt to set su->connected to FALSE.
	 */
	pm_request_idle(su->dev);
}

static void soc_usb_gadget_reset_interrupt(struct soc_usb *su)
{
	u32			reg;

	su->suspended = false;

	/*
	 * Ideally, soc_usb_reset_gadget() would trigger the function
	 * drivers to stop any active transfers through ep disable.
	 * However, for functions which defer ep disable, such as mass
	 * storage, we will need to rely on the call to stop active
	 * transfers here, and avoid allowing of request queuing.
	 */
	su->connected = false;

	soc_usb_reset_gadget(su);

	soc_usb_ep0_reset_state(su);

	soc_usb_stop_active_transfers(su);
	su->connected = true;

	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	reg &= ~SOC_USB_DCTL_TSTCTRL_MASK;
	soc_usb_gadget_dctl_write_safe(su, reg);
	su->test_mode = false;
	su->gadget->wakeup_armed = false;
	soc_usb_gadget_enable_linksts_evts(su, false);
	soc_usb_clear_stall_all_ep(su);

	/* Reset device address to zero */
	reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
	reg &= ~(SOC_USB_DCFG_DEVADDR_MASK);
	soc_usb_writel(su->regs, SOC_USB_DCFG, reg);
}

static void soc_usb_gadget_conndone_interrupt(struct soc_usb *su)
{
	struct soc_usb_ep		*dep;
	int			ret;
	u32			reg;
	u8			lanes = 1;
	u8			speed;

	if (!su->softconnect)
		return;

	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);
	speed = reg & SOC_USB_DSTS_CONNECTSPD;
	su->speed = speed;

    if (SOC_USB_IP_IS(SOC_USB32))
		lanes = SOC_USB_DSTS_CONNLANES(reg) + 1;

	su->gadget->ssp_rate = USB_SSP_GEN_UNKNOWN;

	/*
	 * RAMClkSel is reset to 0 after USB reset, so it must be reprogrammed
	 * each time on Connect Done.
	 *
	 * Currently we always use the reset value. If any platform
	 * wants to set this to a different value, we need to add a
	 * setting and update GCTL.RAMCLKSEL here.
	 */

	switch (speed) {
	case SOC_USB_DSTS_SUPERSPEED_PLUS:
		soc_usb_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		su->gadget->ep0->maxpacket = 512;
		su->gadget->speed = USB_SPEED_SUPER_PLUS;

		if (lanes > 1)
			su->gadget->ssp_rate = USB_SSP_GEN_2x2;
		else
			su->gadget->ssp_rate = USB_SSP_GEN_2x1;
		break;
	case SOC_USB_DSTS_SUPERSPEED:
		soc_usb_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		su->gadget->ep0->maxpacket = 512;
		su->gadget->speed = USB_SPEED_SUPER;

		if (lanes > 1) {
			su->gadget->speed = USB_SPEED_SUPER_PLUS;
			su->gadget->ssp_rate = USB_SSP_GEN_1x2;
		}
		break;
	case SOC_USB_DSTS_HIGHSPEED:
		soc_usb_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		su->gadget->ep0->maxpacket = 64;
		su->gadget->speed = USB_SPEED_HIGH;
		break;
	case SOC_USB_DSTS_FULLSPEED:
		soc_usb_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		su->gadget->ep0->maxpacket = 64;
		su->gadget->speed = USB_SPEED_FULL;
		break;
	}

	su->eps[1]->endpoint.maxpacket = su->gadget->ep0->maxpacket;

	/* Enable USB2 LPM Capability */

	if (!su->usb2_gadget_lpm_disable &&
	    (speed != SOC_USB_DSTS_SUPERSPEED) &&
	    (speed != SOC_USB_DSTS_SUPERSPEED_PLUS)) {
		reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
		reg |= SOC_USB_DCFG_LPM_CAP;
		soc_usb_writel(su->regs, SOC_USB_DCFG, reg);

		reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
		reg &= ~(SOC_USB_DCTL_HIRD_THRES_MASK | SOC_USB_DCTL_L1_HIBER_EN);

		reg |= SOC_USB_DCTL_HIRD_THRES(su->hird_threshold |
					    (su->is_utmi_l1_suspend << 4));

		if (su->has_lpm_erratum)
			reg |= SOC_USB_DCTL_NYET_THRES(su->lpm_nyet_threshold);

		soc_usb_gadget_dctl_write_safe(su, reg);
	} else {
		if (su->usb2_gadget_lpm_disable) {
			reg = soc_usb_readl(su->regs, SOC_USB_DCFG);
			reg &= ~SOC_USB_DCFG_LPM_CAP;
			soc_usb_writel(su->regs, SOC_USB_DCFG, reg);
		}

		reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
		reg &= ~SOC_USB_DCTL_HIRD_THRES_MASK;
		soc_usb_gadget_dctl_write_safe(su, reg);
	}

	dep = su->eps[0];
	ret = __soc_usb_gadget_ep_enable(dep, SOC_USB_DEPCFG_ACTION_MODIFY);
	if (ret) {
		dev_err(su->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = su->eps[1];
	ret = __soc_usb_gadget_ep_enable(dep, SOC_USB_DEPCFG_ACTION_MODIFY);
	if (ret) {
		dev_err(su->dev, "failed to enable %s\n", dep->name);
		return;
	}

	/*
	 * Configure PHY via GUSB3PIPECTLn if required.
	 *
	 * Update GTXFIFOSIZn
	 *
	 * In both cases reset values should be sufficient.
	 */
}

static void soc_usb_gadget_wakeup_interrupt(struct soc_usb *su, unsigned int evtinfo)
{
	su->suspended = false;

	/*
	 * TODO take core out of low power mode when that's
	 * implemented.
	 */

	if (su->async_callbacks && su->gadget_driver->resume) {
		spin_unlock(&su->lock);
		su->gadget_driver->resume(su->gadget);
		spin_lock(&su->lock);
	}

	su->link_state = evtinfo & SOC_USB_LINK_STATE_MASK;
}

static void soc_usb_gadget_linksts_change_interrupt(struct soc_usb *su,
		unsigned int evtinfo)
{
	enum soc_usb_link_state	next = evtinfo & SOC_USB_LINK_STATE_MASK;

	switch (next) {
	case SOC_USB_LINK_STATE_U0:
		if (su->gadget->wakeup_armed) {
			soc_usb_gadget_enable_linksts_evts(su, false);
			soc_usb_resume_gadget(su);
			su->suspended = false;
		}
		break;
	case SOC_USB_LINK_STATE_U1:
		if (su->speed == USB_SPEED_SUPER)
			soc_usb_suspend_gadget(su);
		break;
	case SOC_USB_LINK_STATE_U2:
	case SOC_USB_LINK_STATE_U3:
		soc_usb_suspend_gadget(su);
		break;
	case SOC_USB_LINK_STATE_RESUME:
		soc_usb_resume_gadget(su);
		break;
	default:
		/* do nothing */
		break;
	}

	su->link_state = next;
}

static void soc_usb_gadget_suspend_interrupt(struct soc_usb *su,
					  unsigned int evtinfo)
{
	enum soc_usb_link_state next = evtinfo & SOC_USB_LINK_STATE_MASK;

	if (!su->suspended && next == SOC_USB_LINK_STATE_U3) {
		su->suspended = true;
		soc_usb_suspend_gadget(su);
	}

	su->link_state = next;
}

static void soc_usb_gadget_interrupt(struct soc_usb *su,
		const struct soc_usb_event_devt *event)
{
	printk(">>>>event_type:0x%x\n", event->type);
	switch (event->type) {
	case SOC_USB_DEVICE_EVENT_DISCONNECT:
		soc_usb_gadget_disconnect_interrupt(su);
		break;
	case SOC_USB_DEVICE_EVENT_RESET:
		soc_usb_gadget_reset_interrupt(su);
		break;
	case SOC_USB_DEVICE_EVENT_CONNECT_DONE:
		soc_usb_gadget_conndone_interrupt(su);
		break;
	case SOC_USB_DEVICE_EVENT_WAKEUP:
		soc_usb_gadget_wakeup_interrupt(su, event->event_info);
		break;
	case SOC_USB_DEVICE_EVENT_HIBER_REQ:
		dev_WARN_ONCE(su->dev, true, "unexpected hibernation event\n");
		break;
	case SOC_USB_DEVICE_EVENT_LINK_STATUS_CHANGE:
		soc_usb_gadget_linksts_change_interrupt(su, event->event_info);
		break;
	case SOC_USB_DEVICE_EVENT_SUSPEND:
		soc_usb_gadget_suspend_interrupt(su, event->event_info);
		break;
	case SOC_USB_DEVICE_EVENT_SOF:
	case SOC_USB_DEVICE_EVENT_ERRATIC_ERROR:
	case SOC_USB_DEVICE_EVENT_CMD_CMPL:
	case SOC_USB_DEVICE_EVENT_OVERFLOW:
		break;
	default:
		dev_WARN(su->dev, "UNKNOWN IRQ %d\n", event->type);
	}
}

static void soc_usb_process_event_entry(struct soc_usb *su,
		const union soc_usb_event *event)
{
	trace_soc_usb_event(event->raw, su);

	if (!event->type.is_devspec)
		soc_usb_endpoint_interrupt(su, &event->depevt);
	else if (event->type.type == SOC_USB_EVENT_TYPE_DEV)
		soc_usb_gadget_interrupt(su, &event->devt);
	else
		dev_err(su->dev, "UNKNOWN IRQ type %d\n", event->raw);
}

static irqreturn_t soc_usb_process_event_buf(struct soc_usb_event_buffer *evt)
{
	struct soc_usb *su = evt->su;
	irqreturn_t ret = IRQ_NONE;
	int left;
	u32 reg;

	left = evt->count;

	if (!(evt->flags & SOC_USB_EVENT_PENDING))
		return IRQ_NONE;

	while (left > 0) {
		union soc_usb_event event;

		event.raw = *(u32 *) (evt->cache + evt->lpos);

		soc_usb_process_event_entry(su, &event);

		/*
		 * FIXME we wrap around correctly to the next entry as
		 * almost all entries are 4 bytes in size. There is one
		 * entry which has 12 bytes which is a regular entry
		 * followed by 8 bytes data. ATM I don't know how
		 * things are organized if we get next to the a
		 * boundary so I worry about that once we try to handle
		 * that.
		 */
		evt->lpos = (evt->lpos + 4) % evt->length;
		left -= 4;
	}

	evt->count = 0;
	ret = IRQ_HANDLED;

	/* Unmask interrupt */
	soc_usb_writel(su->regs, SOC_USB_GEVNTSIZ(0),
		    SOC_USB_GEVNTSIZ_SIZE(evt->length));

    // set hc in u0 state only
    if ((su->dis_u1_entry_quirk) && (su->dis_u2_entry_quirk)) {
        reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	    reg &= ~(SOC_USB_DCTL_INITU2ENA
						| SOC_USB_DCTL_ACCEPTU2ENA
						| SOC_USB_DCTL_INITU1ENA
						| SOC_USB_DCTL_ACCEPTU1ENA);
	    soc_usb_writel(su->regs, SOC_USB_DCTL, reg);
    }

	if (su->imod_interval) {
		soc_usb_writel(su->regs, SOC_USB_GEVNTCOUNT(0), SOC_USB_GEVNTCOUNT_EHB);
		soc_usb_writel(su->regs, SOC_USB_DEV_IMOD(0), su->imod_interval);
	}

	/* Keep the clearing of SOC_USB_EVENT_PENDING at the end */
	evt->flags &= ~SOC_USB_EVENT_PENDING;

	return ret;
}

static irqreturn_t soc_usb_thread_interrupt(int irq, void *_evt)
{
	struct soc_usb_event_buffer *evt = _evt;
	struct soc_usb *su = evt->su;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;

	local_bh_disable();
	spin_lock_irqsave(&su->lock, flags);
	ret = soc_usb_process_event_buf(evt);
	spin_unlock_irqrestore(&su->lock, flags);
	local_bh_enable();

	return ret;
}

static irqreturn_t soc_usb_check_event_buf(struct soc_usb_event_buffer *evt)
{
	struct soc_usb *su = evt->su;
	u32 amount;
	u32 count;

	if (pm_runtime_suspended(su->dev)) {
		su->pending_events = true;
		/*
		 * Trigger runtime resume. The get() function will be balanced
		 * after processing the pending events in soc_usb_process_pending
		 * events().
		 */
		pm_runtime_get(su->dev);
		disable_irq_nosync(su->irq_gadget);
		return IRQ_HANDLED;
	}

	/*
	 * With PCIe legacy interrupt, test shows that top-half irq handler can
	 * be called again after HW interrupt deassertion. Check if bottom-half
	 * irq event handler completes before caching new event to prevent
	 * losing events.
	 */
	if (evt->flags & SOC_USB_EVENT_PENDING)
		return IRQ_HANDLED;

	count = soc_usb_readl(su->regs, SOC_USB_GEVNTCOUNT(0));
	count &= SOC_USB_GEVNTCOUNT_MASK;
	if (!count)
		return IRQ_NONE;

	evt->count = count;
	evt->flags |= SOC_USB_EVENT_PENDING;

	/* Mask interrupt */
	soc_usb_writel(su->regs, SOC_USB_GEVNTSIZ(0),
		    SOC_USB_GEVNTSIZ_INTMASK | SOC_USB_GEVNTSIZ_SIZE(evt->length));

	amount = min(count, evt->length - evt->lpos);
	memcpy(evt->cache + evt->lpos, evt->buf + evt->lpos, amount);

	if (amount < count)
		memcpy(evt->cache, evt->buf, count - amount);

	soc_usb_writel(su->regs, SOC_USB_GEVNTCOUNT(0), count);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t soc_usb_interrupt(int irq, void *_evt)
{
	struct soc_usb_event_buffer	*evt = _evt;

	return soc_usb_check_event_buf(evt);
}

static int soc_usb_gadget_get_irq(struct soc_usb *su)
{
	struct platform_device *soc_usb_pdev = to_platform_device(su->dev);
	int irq;

	irq = platform_get_irq_byname_optional(soc_usb_pdev, "peripheral");
	if (irq > 0)
		goto out;

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq_byname_optional(soc_usb_pdev, "su_usb3");
	if (irq > 0)
		goto out;

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq(soc_usb_pdev, 0);

out:
	return irq;
}

static void su_gadget_release(struct device *dev)
{
	struct usb_gadget *gadget = container_of(dev, struct usb_gadget, dev);

	kfree(gadget);
}

/**
 * soc_usb_gadget_init - initializes gadget related registers
 * @su: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int soc_usb_gadget_init(struct soc_usb *su)
{
	int ret;
	int irq;
	struct device *dev;

	irq = soc_usb_gadget_get_irq(su);
	if (irq < 0) {
		ret = irq;
		goto err0;
	}

	su->irq_gadget = irq;

	su->ep0_trb = dma_alloc_coherent(su->sysdev,
					  sizeof(*su->ep0_trb) * 2,
					  &su->ep0_trb_addr, GFP_KERNEL);
	if (!su->ep0_trb) {
		dev_err(su->dev, "failed to allocate ep0 trb\n");
		ret = -ENOMEM;
		goto err0;
	}

	su->setup_buf = kzalloc(SOC_USB_EP0_SETUP_SIZE, GFP_KERNEL);
	if (!su->setup_buf) {
		ret = -ENOMEM;
		goto err1;
	}

	su->bounce = dma_alloc_coherent(su->sysdev, SOC_USB_BOUNCE_SIZE,
			&su->bounce_addr, GFP_KERNEL);
	if (!su->bounce) {
		ret = -ENOMEM;
		goto err2;
	}

	init_completion(&su->ep0_in_setup);
	su->gadget = kzalloc(sizeof(struct usb_gadget), GFP_KERNEL);
	if (!su->gadget) {
		ret = -ENOMEM;
		goto err3;
	}


	usb_initialize_gadget(su->dev, su->gadget, su_gadget_release);
	dev				= &su->gadget->dev;
	dev->platform_data		= su;
	su->gadget->ops		= &soc_usb_gadget_ops;
	su->gadget->speed		= USB_SPEED_UNKNOWN;
	su->gadget->ssp_rate		= USB_SSP_GEN_UNKNOWN;
	su->gadget->sg_supported	= true;
	su->gadget->name		= "soc_usb-gadget";
	su->gadget->lpm_capable	= !su->usb2_gadget_lpm_disable;
	su->gadget->wakeup_capable	= true;

	su->gadget->max_speed		= su->maximum_speed;
	su->gadget->max_ssp_rate	= su->max_ssp_rate;

	/*
	 * REVISIT: Here we should clear all pending IRQs to be
	 * sure we're starting from a well known location.
	 */

	ret = soc_usb_gadget_init_endpoints(su, su->num_eps);
	if (ret)
		goto err4;

	ret = usb_add_gadget(su->gadget);
	if (ret) {
		dev_err(su->dev, "failed to add gadget\n");
		goto err5;
	}

    if (SOC_USB_IP_IS(SOC_USB32) && su->maximum_speed == USB_SPEED_SUPER_PLUS)
		soc_usb_gadget_set_ssp_rate(su->gadget, su->max_ssp_rate);
	else
	    soc_usb_gadget_set_speed(su->gadget, su->maximum_speed);

	/* No system wakeup if no gadget driver bound */
	if (su->sys_wakeup)
		device_wakeup_disable(su->sysdev);

	return 0;

err5:
	soc_usb_gadget_free_endpoints(su);
err4:
	usb_put_gadget(su->gadget);
	su->gadget = NULL;
err3:
	dma_free_coherent(su->sysdev, SOC_USB_BOUNCE_SIZE, su->bounce,
			su->bounce_addr);

err2:
	kfree(su->setup_buf);

err1:
	dma_free_coherent(su->sysdev, sizeof(*su->ep0_trb) * 2,
			su->ep0_trb, su->ep0_trb_addr);

err0:
	return ret;
}

/* -------------------------------------------------------------------------- */

void soc_usb_gadget_exit(struct soc_usb *su)
{
	if (!su->gadget)
		return;

	soc_usb_enable_susphy(su, false);
	usb_del_gadget(su->gadget);
	soc_usb_gadget_free_endpoints(su);
	usb_put_gadget(su->gadget);
	dma_free_coherent(su->sysdev, SOC_USB_BOUNCE_SIZE, su->bounce,
			  su->bounce_addr);
	kfree(su->setup_buf);
	dma_free_coherent(su->sysdev, sizeof(*su->ep0_trb) * 2,
			  su->ep0_trb, su->ep0_trb_addr);
}

int soc_usb_gadget_suspend(struct soc_usb *su)
{
	unsigned long flags;
	int ret;

	ret = soc_usb_gadget_soft_disconnect(su);
	if (ret)
		goto err;

	spin_lock_irqsave(&su->lock, flags);
	if (su->gadget_driver)
		soc_usb_disconnect_gadget(su);
	spin_unlock_irqrestore(&su->lock, flags);

	return 0;

err:
	/*
	 * Attempt to reset the controller's state. Likely no
	 * communication can be established until the host
	 * performs a port reset.
	 */
	if (su->softconnect)
		soc_usb_gadget_soft_connect(su);

	return ret;
}

int soc_usb_gadget_resume(struct soc_usb *su)
{
	if (!su->gadget_driver || !su->softconnect)
		return 0;

	return soc_usb_gadget_soft_connect(su);
}

void soc_usb_gadget_process_pending_events(struct soc_usb *su)
{
	if (su->pending_events) {
		soc_usb_interrupt(su->irq_gadget, su->ev_buf);
		soc_usb_thread_interrupt(su->irq_gadget, su->ev_buf);
		pm_runtime_put(su->dev);
		su->pending_events = false;
		enable_irq(su->irq_gadget);
	}
}
