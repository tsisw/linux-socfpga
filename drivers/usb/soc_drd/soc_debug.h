/* SPDX-License-Identifier: GPL-2.0 */
/*
 * soc_debug.h - SOC USB3 DRD Controller Debug Header
 *
 */

#ifndef __SOC_USB_DEBUG_H
#define __SOC_USB_DEBUG_H

#include "soc_core.h"

/**
 * soc_usb_gadget_ep_cmd_string - returns endpoint command string
 * @cmd: command code
 */
static inline const char *
soc_usb_gadget_ep_cmd_string(u8 cmd)
{
	switch (cmd) {
	case SOC_USB_DEPCMD_DEPSTARTCFG:
		return "Start New Configuration";
	case SOC_USB_DEPCMD_ENDTRANSFER:
		return "End Transfer";
	case SOC_USB_DEPCMD_UPDATETRANSFER:
		return "Update Transfer";
	case SOC_USB_DEPCMD_STARTTRANSFER:
		return "Start Transfer";
	case SOC_USB_DEPCMD_CLEARSTALL:
		return "Clear Stall";
	case SOC_USB_DEPCMD_SETSTALL:
		return "Set Stall";
	case SOC_USB_DEPCMD_GETEPSTATE:
		return "Get Endpoint State";
	case SOC_USB_DEPCMD_SETTRANSFRESOURCE:
		return "Set Endpoint Transfer Resource";
	case SOC_USB_DEPCMD_SETEPCONFIG:
		return "Set Endpoint Configuration";
	default:
		return "UNKNOWN command";
	}
}

/**
 * soc_usb_gadget_generic_cmd_string - returns generic command string
 * @cmd: command code
 */
static inline const char *
soc_usb_gadget_generic_cmd_string(u8 cmd)
{
	switch (cmd) {
	case SOC_USB_DGCMD_SET_PERIODIC_PAR:
		return "Set Periodic Parameters";
	case SOC_USB_DGCMD_SET_SCRATCHPAD_ADDR_LO:
		return "Set Scratchpad Buffer Array Address Lo";
	case SOC_USB_DGCMD_SET_SCRATCHPAD_ADDR_HI:
		return "Set Scratchpad Buffer Array Address Hi";
	case SOC_USB_DGCMD_SELECTED_FIFO_FLUSH:
		return "Selected FIFO Flush";
	case SOC_USB_DGCMD_ALL_FIFO_FLUSH:
		return "All FIFO Flush";
	case SOC_USB_DGCMD_SET_ENDPOINT_NRDY:
		return "Set Endpoint NRDY";
	case SOC_USB_DGCMD_RESTART_AFTER_DISC:
		return "Restart after disconnect";
	case SOC_USB_DGCMD_DEV_NOTIFICATION:
		return "Device Notification";
	default:
		return "UNKNOWN";
	}
}

/**
 * soc_usb_gadget_link_string - returns link name
 * @link_state: link state code
 */
static inline const char *
soc_usb_gadget_link_string(enum soc_usb_link_state link_state)
{
	switch (link_state) {
	case SOC_USB_LINK_STATE_U0:
		return "U0";
	case SOC_USB_LINK_STATE_U1:
		return "U1";
	case SOC_USB_LINK_STATE_U2:
		return "U2";
	case SOC_USB_LINK_STATE_U3:
		return "U3";
	case SOC_USB_LINK_STATE_SS_DIS:
		return "SS.Disabled";
	case SOC_USB_LINK_STATE_RX_DET:
		return "RX.Detect";
	case SOC_USB_LINK_STATE_SS_INACT:
		return "SS.Inactive";
	case SOC_USB_LINK_STATE_POLL:
		return "Polling";
	case SOC_USB_LINK_STATE_RECOV:
		return "Recovery";
	case SOC_USB_LINK_STATE_HRESET:
		return "Hot Reset";
	case SOC_USB_LINK_STATE_CMPLY:
		return "Compliance";
	case SOC_USB_LINK_STATE_LPBK:
		return "Loopback";
	case SOC_USB_LINK_STATE_RESET:
		return "Reset";
	case SOC_USB_LINK_STATE_RESUME:
		return "Resume";
	default:
		return "UNKNOWN link state";
	}
}

/**
 * soc_usb_gadget_hs_link_string - returns highspeed and below link name
 * @link_state: link state code
 */
static inline const char *
soc_usb_gadget_hs_link_string(enum soc_usb_link_state link_state)
{
	switch (link_state) {
	case SOC_USB_LINK_STATE_U0:
		return "On";
	case SOC_USB_LINK_STATE_U2:
		return "Sleep";
	case SOC_USB_LINK_STATE_U3:
		return "Suspend";
	case SOC_USB_LINK_STATE_SS_DIS:
		return "Disconnected";
	case SOC_USB_LINK_STATE_RX_DET:
		return "Early Suspend";
	case SOC_USB_LINK_STATE_RECOV:
		return "Recovery";
	case SOC_USB_LINK_STATE_RESET:
		return "Reset";
	case SOC_USB_LINK_STATE_RESUME:
		return "Resume";
	default:
		return "UNKNOWN link state";
	}
}

/**
 * soc_usb_trb_type_string - returns TRB type as a string
 * @type: the type of the TRB
 */
static inline const char *soc_usb_trb_type_string(unsigned int type)
{
	switch (type) {
	case SOC_USB_TRBCTL_NORMAL:
		return "normal";
	case SOC_USB_TRBCTL_CONTROL_SETUP:
		return "setup";
	case SOC_USB_TRBCTL_CONTROL_STATUS2:
		return "status2";
	case SOC_USB_TRBCTL_CONTROL_STATUS3:
		return "status3";
	case SOC_USB_TRBCTL_CONTROL_DATA:
		return "data";
	case SOC_USB_TRBCTL_ISOCHRONOUS_FIRST:
		return "isoc-first";
	case SOC_USB_TRBCTL_ISOCHRONOUS:
		return "isoc";
	case SOC_USB_TRBCTL_LINK_TRB:
		return "link";
	default:
		return "UNKNOWN";
	}
}

static inline const char *soc_usb_ep0_state_string(enum soc_usb_ep0_state state)
{
	switch (state) {
	case EP0_UNCONNECTED:
		return "Unconnected";
	case EP0_SETUP_PHASE:
		return "Setup Phase";
	case EP0_DATA_PHASE:
		return "Data Phase";
	case EP0_STATUS_PHASE:
		return "Status Phase";
	default:
		return "UNKNOWN";
	}
}

/**
 * soc_usb_gadget_event_string - returns event name
 * @event: the event code
 */
static inline const char *soc_usb_gadget_event_string(char *str, size_t size,
		const struct soc_usb_event_devt *event)
{
	enum soc_usb_link_state state = event->event_info & SOC_USB_LINK_STATE_MASK;

	switch (event->type) {
	case SOC_USB_DEVICE_EVENT_DISCONNECT:
		snprintf(str, size, "Disconnect: [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_RESET:
		snprintf(str, size, "Reset [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_CONNECT_DONE:
		snprintf(str, size, "Connection Done [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_LINK_STATUS_CHANGE:
		snprintf(str, size, "Link Change [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_WAKEUP:
		snprintf(str, size, "WakeUp [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_SUSPEND:
		snprintf(str, size, "Suspend [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_SOF:
		snprintf(str, size, "Start-Of-Frame [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_ERRATIC_ERROR:
		snprintf(str, size, "Erratic Error [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_CMD_CMPL:
		snprintf(str, size, "Command Complete [%s]",
				soc_usb_gadget_link_string(state));
		break;
	case SOC_USB_DEVICE_EVENT_OVERFLOW:
		snprintf(str, size, "Overflow [%s]",
				soc_usb_gadget_link_string(state));
		break;
	default:
		snprintf(str, size, "UNKNOWN");
	}

	return str;
}

/**
 * soc_usb_ep_event_string - returns event name
 * @event: then event code
 */
static inline const char *soc_usb_ep_event_string(char *str, size_t size,
		const struct soc_usb_event_depevt *event, u32 ep0state)
{
	u8 epnum = event->endpoint_number;
	size_t len;
	int status;

	len = scnprintf(str, size, "ep%d%s: ", epnum >> 1,
			(epnum & 1) ? "in" : "out");

	status = event->status;

	switch (event->endpoint_event) {
	case SOC_USB_DEPEVT_XFERCOMPLETE:
		len += scnprintf(str + len, size - len,
				"Transfer Complete (%c%c%c)",
				status & DEPEVT_STATUS_SHORT ? 'S' : 's',
				status & DEPEVT_STATUS_IOC ? 'I' : 'i',
				status & DEPEVT_STATUS_LST ? 'L' : 'l');

		if (epnum <= 1)
			scnprintf(str + len, size - len, " [%s]",
					soc_usb_ep0_state_string(ep0state));
		break;
	case SOC_USB_DEPEVT_XFERINPROGRESS:
		scnprintf(str + len, size - len,
				"Transfer In Progress [%08x] (%c%c%c)",
				event->parameters,
				status & DEPEVT_STATUS_SHORT ? 'S' : 's',
				status & DEPEVT_STATUS_IOC ? 'I' : 'i',
				status & DEPEVT_STATUS_LST ? 'M' : 'm');
		break;
	case SOC_USB_DEPEVT_XFERNOTREADY:
		len += scnprintf(str + len, size - len,
				"Transfer Not Ready [%08x]%s",
				event->parameters,
				status & DEPEVT_STATUS_TRANSFER_ACTIVE ?
				" (Active)" : " (Not Active)");

		/* Control Endpoints */
		if (epnum <= 1) {
			int phase = DEPEVT_STATUS_CONTROL_PHASE(event->status);

			switch (phase) {
			case DEPEVT_STATUS_CONTROL_DATA:
				scnprintf(str + len, size - len,
						" [Data Phase]");
				break;
			case DEPEVT_STATUS_CONTROL_STATUS:
				scnprintf(str + len, size - len,
						" [Status Phase]");
			}
		}
		break;
	case SOC_USB_DEPEVT_RXTXFIFOEVT:
		scnprintf(str + len, size - len, "FIFO");
		break;
	case SOC_USB_DEPEVT_STREAMEVT:
		status = event->status;

		switch (status) {
		case DEPEVT_STREAMEVT_FOUND:
			scnprintf(str + len, size - len, " Stream %d Found",
					event->parameters);
			break;
		case DEPEVT_STREAMEVT_NOTFOUND:
		default:
			scnprintf(str + len, size - len, " Stream Not Found");
			break;
		}

		break;
	case SOC_USB_DEPEVT_EPCMDCMPLT:
		scnprintf(str + len, size - len, "Endpoint Command Complete");
		break;
	default:
		scnprintf(str + len, size - len, "UNKNOWN");
	}

	return str;
}

/**
 * soc_usb_gadget_event_type_string - return event name
 * @event: the event code
 */
static inline const char *soc_usb_gadget_event_type_string(u8 event)
{
	switch (event) {
	case SOC_USB_DEVICE_EVENT_DISCONNECT:
		return "Disconnect";
	case SOC_USB_DEVICE_EVENT_RESET:
		return "Reset";
	case SOC_USB_DEVICE_EVENT_CONNECT_DONE:
		return "Connect Done";
	case SOC_USB_DEVICE_EVENT_LINK_STATUS_CHANGE:
		return "Link Status Change";
	case SOC_USB_DEVICE_EVENT_WAKEUP:
		return "Wake-Up";
	case SOC_USB_DEVICE_EVENT_HIBER_REQ:
		return "Hibernation";
	case SOC_USB_DEVICE_EVENT_SUSPEND:
		return "Suspend";
	case SOC_USB_DEVICE_EVENT_SOF:
		return "Start of Frame";
	case SOC_USB_DEVICE_EVENT_ERRATIC_ERROR:
		return "Erratic Error";
	case SOC_USB_DEVICE_EVENT_CMD_CMPL:
		return "Command Complete";
	case SOC_USB_DEVICE_EVENT_OVERFLOW:
		return "Overflow";
	default:
		return "UNKNOWN";
	}
}

static inline const char *soc_usb_decode_event(char *str, size_t size, u32 event,
		u32 ep0state)
{
	union soc_usb_event evt;

	memcpy(&evt, &event, sizeof(event));

	if (evt.type.is_devspec)
		return soc_usb_gadget_event_string(str, size, &evt.devt);
	else
		return soc_usb_ep_event_string(str, size, &evt.depevt, ep0state);
}

static inline const char *soc_usb_ep_cmd_status_string(int status)
{
	switch (status) {
	case -ETIMEDOUT:
		return "Timed Out";
	case 0:
		return "Successful";
	case DEPEVT_TRANSFER_NO_RESOURCE:
		return "No Resource";
	case DEPEVT_TRANSFER_BUS_EXPIRY:
		return "Bus Expiry";
	default:
		return "UNKNOWN";
	}
}

static inline const char *soc_usb_gadget_generic_cmd_status_string(int status)
{
	switch (status) {
	case -ETIMEDOUT:
		return "Timed Out";
	case 0:
		return "Successful";
	case 1:
		return "Error";
	default:
		return "UNKNOWN";
	}
}

extern ssize_t soc_usb3_consis_write(struct file *file, const char __user *buf, size_t count, loff_t *offset);

#ifdef CONFIG_PM_SLEEP
extern int soc_usb_suspend(struct device *dev);
extern int soc_usb_resume(struct device *dev);
#endif


#ifdef CONFIG_DEBUG_FS
extern void soc_usb_debugfs_create_endpoint_dir(struct soc_usb_ep *dep);
extern void soc_usb_debugfs_remove_endpoint_dir(struct soc_usb_ep *dep);
extern void soc_usb_debugfs_init(struct soc_usb *d);
extern void soc_usb_debugfs_exit(struct soc_usb *d);
#else
static inline void soc_usb_debugfs_create_endpoint_dir(struct soc_usb_ep *dep)
{  }
static inline void soc_usb_debugfs_remove_endpoint_dir(struct soc_usb_ep *dep)
{  }
static inline void soc_usb_debugfs_init(struct soc_usb *d)
{  }
static inline void soc_usb_debugfs_exit(struct soc_usb *d)
{  }
#endif
#endif /* __SOC_USB_DEBUG_H */
