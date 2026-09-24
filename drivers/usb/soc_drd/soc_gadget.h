/* SPDX-License-Identifier: GPL-2.0 */
/*
 * soc_gadget.h - SOC USB3 DRD Gadget Header
 *
 */

#ifndef __DRIVERS_USB_SOC_DRD_GADGET_H
#define __DRIVERS_USB_SOC_DRD_GADGET_H

#include <linux/list.h>
#include <linux/usb/gadget.h>
#include "soc_io.h"

struct soc_usb;
#define to_soc_usb_ep(ep)		(container_of(ep, struct soc_usb_ep, endpoint))
#define gadget_to_su(g)	(dev_get_platdata(&g->dev))

/* DEPCFG parameter 1 */
#define SOC_USB_DEPCFG_INT_NUM(n)		(((n) & 0x1f) << 0)
#define SOC_USB_DEPCFG_XFER_COMPLETE_EN	BIT(8)
#define SOC_USB_DEPCFG_XFER_IN_PROGRESS_EN	BIT(9)
#define SOC_USB_DEPCFG_XFER_NOT_READY_EN	BIT(10)
#define SOC_USB_DEPCFG_FIFO_ERROR_EN	BIT(11)
#define SOC_USB_DEPCFG_STREAM_EVENT_EN	BIT(13)
#define SOC_USB_DEPCFG_BINTERVAL_M1(n)	(((n) & 0xff) << 16)
#define SOC_USB_DEPCFG_STREAM_CAPABLE	BIT(24)
#define SOC_USB_DEPCFG_EP_NUMBER(n)	(((n) & 0x1f) << 25)
#define SOC_USB_DEPCFG_BULK_BASED		BIT(30)
#define SOC_USB_DEPCFG_FIFO_BASED		BIT(31)

/* DEPCFG parameter 0 */
#define SOC_USB_DEPCFG_EP_TYPE(n)		(((n) & 0x3) << 1)
#define SOC_USB_DEPCFG_MAX_PACKET_SIZE(n)	(((n) & 0x7ff) << 3)
#define SOC_USB_DEPCFG_FIFO_NUMBER(n)	(((n) & 0x1f) << 17)
#define SOC_USB_DEPCFG_BURST_SIZE(n)	(((n) & 0xf) << 22)
#define SOC_USB_DEPCFG_DATA_SEQ_NUM(n)	((n) << 26)
/* This applies for core versions earlier than 1.94a */
#define SOC_USB_DEPCFG_IGN_SEQ_NUM		BIT(31)
/* These apply for core versions 1.94a and later */
#define SOC_USB_DEPCFG_ACTION_INIT		(0 << 30)
#define SOC_USB_DEPCFG_ACTION_RESTORE	BIT(30)
#define SOC_USB_DEPCFG_ACTION_MODIFY	(2 << 30)

/* DEPXFERCFG parameter 0 */
#define SOC_USB_DEPXFERCFG_NUM_XFER_RES(n)	((n) & 0xffff)

/* U1 Device exit Latency */
#define SOC_USB_DEFAULT_U1_DEV_EXIT_LAT	0x0A	/* Less then 10 microsec */

/* U2 Device exit Latency */
#define SOC_USB_DEFAULT_U2_DEV_EXIT_LAT	0x1FF	/* Less then 511 microsec */

/* Frame/Microframe Number Mask */
#define SOC_USB_FRNUMBER_MASK		0x3fff
/* -------------------------------------------------------------------------- */

#define to_soc_usb_request(r)	(container_of(r, struct soc_usb_request, request))

/**
 * next_request - gets the next request on the given list
 * @list: the request list to operate on
 *
 * Caller should take care of locking. This function return %NULL or the first
 * request available on @list.
 */
static inline struct soc_usb_request *next_request(struct list_head *list)
{
	return list_first_entry_or_null(list, struct soc_usb_request, list);
}

/**
 * soc_usb_gadget_move_started_request - move @req to the started_list
 * @req: the request to be moved
 *
 * Caller should take care of locking. This function will move @req from its
 * current list to the endpoint's started_list.
 */
static inline void soc_usb_gadget_move_started_request(struct soc_usb_request *req)
{
	struct soc_usb_ep		*dep = req->dep;

	req->status = SOC_USB_REQUEST_STATUS_STARTED;
	list_move_tail(&req->list, &dep->started_list);
}

/**
 * soc_usb_gadget_move_cancelled_request - move @req to the cancelled_list
 * @req: the request to be moved
 * @reason: cancelled reason for the soc_usb request
 *
 * Caller should take care of locking. This function will move @req from its
 * current list to the endpoint's cancelled_list.
 */
static inline void soc_usb_gadget_move_cancelled_request(struct soc_usb_request *req,
		unsigned int reason)
{
	struct soc_usb_ep		*dep = req->dep;

	req->status = reason;
	list_move_tail(&req->list, &dep->cancelled_list);
}

void soc_usb_gadget_giveback(struct soc_usb_ep *dep, struct soc_usb_request *req,
		int status);

void soc_usb_ep0_interrupt(struct soc_usb *su,
		const struct soc_usb_event_depevt *event);
void soc_usb_ep0_out_start(struct soc_usb *su);
void soc_usb_ep0_end_control_data(struct soc_usb *su, struct soc_usb_ep *dep);
void soc_usb_ep0_stall_and_restart(struct soc_usb *su);
int __soc_usb_gadget_ep0_set_halt(struct usb_ep *ep, int value);
int soc_usb_gadget_ep0_set_halt(struct usb_ep *ep, int value);
int soc_usb_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *request,
		gfp_t gfp_flags);
int __soc_usb_gadget_ep_set_halt(struct soc_usb_ep *dep, int value, int protocol);
void soc_usb_ep0_send_delayed_status(struct soc_usb *su);
void soc_usb_stop_active_transfer(struct soc_usb_ep *dep, bool force, bool interrupt);

/**
 * soc_usb_gadget_ep_get_transfer_index - Gets transfer index from HW
 * @dep: soc_usb endpoint
 *
 * Caller should take care of locking. Returns the transfer resource
 * index for a given endpoint.
 */
static inline void soc_usb_gadget_ep_get_transfer_index(struct soc_usb_ep *dep)
{
	u32			res_id;

	res_id = soc_usb_readl(dep->regs, SOC_USB_DEPCMD);
	dep->resource_index = SOC_USB_DEPCMD_GET_RSC_IDX(res_id);
}

/**
 * soc_usb_gadget_dctl_write_safe - write to DCTL safe from link state change
 * @su: pointer to our context structure
 * @value: value to write to DCTL
 *
 * Use this function when doing read-modify-write to DCTL. It will not
 * send link state change request.
 */
static inline void soc_usb_gadget_dctl_write_safe(struct soc_usb *su, u32 value)
{
	value &= ~SOC_USB_DCTL_ULSTCHNGREQ_MASK;
	soc_usb_writel(su->regs, SOC_USB_DCTL, value);
}

#endif /* __DRIVERS_USB_SOC_DRD_GADGET_H */
