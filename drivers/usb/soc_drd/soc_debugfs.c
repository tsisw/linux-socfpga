// SPDX-License-Identifier: GPL-2.0
/*
 * soc_debugfs.c - SOC USB3 DRD Controller DebugFS file
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ptrace.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <linux/usb/ch9.h>

#include "soc_core.h"
#include "soc_io.h"
#include "soc_debug.h"

#include "../host/xhci.h"
#include <linux/platform_device.h>
#include <linux/usb/ch11.h>
#include <linux/usb/ch9.h>

#define SOC_USB_LSP_MUX_UNSELECTED 0xfffff

#define dump_register(nm)				\
{							\
	.name	= __stringify(nm),			\
	.offset	= SOC_USB_ ##nm,				\
}

#define dump_ep_register_set(n)			\
	{					\
		.name = "DEPCMDPAR2("__stringify(n)")",	\
		.offset = SOC_USB_DEP_BASE(n) +	\
			SOC_USB_DEPCMDPAR2,	\
	},					\
	{					\
		.name = "DEPCMDPAR1("__stringify(n)")",	\
		.offset = SOC_USB_DEP_BASE(n) +	\
			SOC_USB_DEPCMDPAR1,	\
	},					\
	{					\
		.name = "DEPCMDPAR0("__stringify(n)")",	\
		.offset = SOC_USB_DEP_BASE(n) +	\
			SOC_USB_DEPCMDPAR0,	\
	},					\
	{					\
		.name = "DEPCMD("__stringify(n)")",	\
		.offset = SOC_USB_DEP_BASE(n) +	\
			SOC_USB_DEPCMD,		\
	}


static const struct debugfs_reg32 soc_usb_regs[] = {
	dump_register(GTXTHRCFG),
	dump_register(GRXTHRCFG),
	dump_register(GCTL),
	dump_register(GSTS),
	dump_register(GUCTL1),
	dump_register(GUCTL),
	dump_register(GHWPARAMS0),
	dump_register(GHWPARAMS3),
	dump_register(GHWPARAMS6),
	dump_register(GHWPARAMS7),

	dump_register(GUSB2PHYCFG(0)),
	dump_register(GUSB2PHYCFG(1)),
	dump_register(GUSB2PHYCFG(2)),
	dump_register(GUSB2PHYCFG(3)),
	dump_register(GUSB2PHYCFG(4)),
	dump_register(GUSB2PHYCFG(5)),
	dump_register(GUSB2PHYCFG(6)),
	dump_register(GUSB2PHYCFG(7)),
	dump_register(GUSB2PHYCFG(8)),
	dump_register(GUSB2PHYCFG(9)),
	dump_register(GUSB2PHYCFG(10)),
	dump_register(GUSB2PHYCFG(11)),
	dump_register(GUSB2PHYCFG(12)),
	dump_register(GUSB2PHYCFG(13)),
	dump_register(GUSB2PHYCFG(14)),
	dump_register(GUSB2PHYCFG(15)),

	dump_register(GUSB3PIPECTL(0)),
	dump_register(GUSB3PIPECTL(1)),
	dump_register(GUSB3PIPECTL(2)),
	dump_register(GUSB3PIPECTL(3)),
	dump_register(GUSB3PIPECTL(4)),
	dump_register(GUSB3PIPECTL(5)),
	dump_register(GUSB3PIPECTL(6)),
	dump_register(GUSB3PIPECTL(7)),
	dump_register(GUSB3PIPECTL(8)),
	dump_register(GUSB3PIPECTL(9)),
	dump_register(GUSB3PIPECTL(10)),
	dump_register(GUSB3PIPECTL(11)),
	dump_register(GUSB3PIPECTL(12)),
	dump_register(GUSB3PIPECTL(13)),
	dump_register(GUSB3PIPECTL(14)),
	dump_register(GUSB3PIPECTL(15)),

	dump_register(GTXFIFOSIZ(0)),
	dump_register(GTXFIFOSIZ(1)),
	dump_register(GTXFIFOSIZ(2)),
	dump_register(GTXFIFOSIZ(3)),
	dump_register(GTXFIFOSIZ(4)),
	dump_register(GTXFIFOSIZ(5)),
	dump_register(GTXFIFOSIZ(6)),
	dump_register(GTXFIFOSIZ(7)),
	dump_register(GTXFIFOSIZ(8)),
	dump_register(GTXFIFOSIZ(9)),
	dump_register(GTXFIFOSIZ(10)),
	dump_register(GTXFIFOSIZ(11)),
	dump_register(GTXFIFOSIZ(12)),
	dump_register(GTXFIFOSIZ(13)),
	dump_register(GTXFIFOSIZ(14)),
	dump_register(GTXFIFOSIZ(15)),
	dump_register(GTXFIFOSIZ(16)),
	dump_register(GTXFIFOSIZ(17)),
	dump_register(GTXFIFOSIZ(18)),
	dump_register(GTXFIFOSIZ(19)),
	dump_register(GTXFIFOSIZ(20)),
	dump_register(GTXFIFOSIZ(21)),
	dump_register(GTXFIFOSIZ(22)),
	dump_register(GTXFIFOSIZ(23)),
	dump_register(GTXFIFOSIZ(24)),
	dump_register(GTXFIFOSIZ(25)),
	dump_register(GTXFIFOSIZ(26)),
	dump_register(GTXFIFOSIZ(27)),
	dump_register(GTXFIFOSIZ(28)),
	dump_register(GTXFIFOSIZ(29)),
	dump_register(GTXFIFOSIZ(30)),
	dump_register(GTXFIFOSIZ(31)),

	dump_register(GRXFIFOSIZ(0)),
	dump_register(GRXFIFOSIZ(1)),
	dump_register(GRXFIFOSIZ(2)),
	dump_register(GRXFIFOSIZ(3)),
	dump_register(GRXFIFOSIZ(4)),
	dump_register(GRXFIFOSIZ(5)),
	dump_register(GRXFIFOSIZ(6)),
	dump_register(GRXFIFOSIZ(7)),
	dump_register(GRXFIFOSIZ(8)),
	dump_register(GRXFIFOSIZ(9)),
	dump_register(GRXFIFOSIZ(10)),
	dump_register(GRXFIFOSIZ(11)),
	dump_register(GRXFIFOSIZ(12)),
	dump_register(GRXFIFOSIZ(13)),
	dump_register(GRXFIFOSIZ(14)),
	dump_register(GRXFIFOSIZ(15)),
	dump_register(GRXFIFOSIZ(16)),
	dump_register(GRXFIFOSIZ(17)),
	dump_register(GRXFIFOSIZ(18)),
	dump_register(GRXFIFOSIZ(19)),
	dump_register(GRXFIFOSIZ(20)),
	dump_register(GRXFIFOSIZ(21)),
	dump_register(GRXFIFOSIZ(22)),
	dump_register(GRXFIFOSIZ(23)),
	dump_register(GRXFIFOSIZ(24)),
	dump_register(GRXFIFOSIZ(25)),
	dump_register(GRXFIFOSIZ(26)),
	dump_register(GRXFIFOSIZ(27)),
	dump_register(GRXFIFOSIZ(28)),
	dump_register(GRXFIFOSIZ(29)),
	dump_register(GRXFIFOSIZ(30)),
	dump_register(GRXFIFOSIZ(31)),

	dump_register(GEVNTADRLO(0)),
	dump_register(GEVNTADRHI(0)),
	dump_register(GEVNTSIZ(0)),
	dump_register(GEVNTCOUNT(0)),

	dump_register(DCFG),
	dump_register(DCTL),
	dump_register(DEVTEN),
	dump_register(DSTS),
	dump_register(DGCMDPAR),
	dump_register(DGCMD),
	dump_register(DALEPENA),

	dump_ep_register_set(0),
	dump_ep_register_set(1),
	dump_ep_register_set(2),
	dump_ep_register_set(3),
	dump_ep_register_set(4),
	dump_ep_register_set(5),
	dump_ep_register_set(6),
	dump_ep_register_set(7),
	dump_ep_register_set(8),
	dump_ep_register_set(9),
	dump_ep_register_set(10),
	dump_ep_register_set(11),
	dump_ep_register_set(12),
	dump_ep_register_set(13),
	dump_ep_register_set(14),
	dump_ep_register_set(15),
	dump_ep_register_set(16),
	dump_ep_register_set(17),
	dump_ep_register_set(18),
	dump_ep_register_set(19),
	dump_ep_register_set(20),
	dump_ep_register_set(21),
	dump_ep_register_set(22),
	dump_ep_register_set(23),
	dump_ep_register_set(24),
	dump_ep_register_set(25),
	dump_ep_register_set(26),
	dump_ep_register_set(27),
	dump_ep_register_set(28),
	dump_ep_register_set(29),
	dump_ep_register_set(30),
	dump_ep_register_set(31),
};

/*
 * This code is used to test consistency in host mode.
 * conduct consistency testing by entering commands at the shell command line.
 *
 * execute commands under the shell as flollow:
 * mount -t debugfs none /sys/kernel/debug
 * cd /sys/kernel/debug/usb/$USB_CONTROL/
 
 * echo 1 3 0 1 > consistency_test
 * parameter 1 represents the test item.
 * parameter 2 represents the usb bus, usb2 or usb3.
 * parameter 3 represents the test sub item of usb2, if testing USB3, the default setting is 0.
 * parameter 4 represents the usb port number.
 *
 */
static const char *soc_usb_test_string(u8 idx)
{
	switch (idx) {
	case 1:
		return "compliance mode test";
	case 2:
		return "set port suspend test";
	case 3:
		return "clear port suspend test";
	case 4:
		return "loopback mode test";
	case 5:
		return "U0 mode test";
	case 6:
		return "U1 mode test";
	case 7:
		return "U2 mode test";
	case 8:
		return "U3 mode test";
	case 9:
		return "USB2 TEST MODE test";
	case 10:
		return "RESET test";
	}
	return "UNKNOWN IDX";
}

static int soc_usb_clear_port_feature(struct usb_device *hdev,
		int port1, int feature)
{
	return usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
		USB_REQ_CLEAR_FEATURE, USB_RT_PORT, feature, port1,
		NULL, 0, 1000);
}

static int soc_usb_set_port_feature(struct usb_device *hdev,
		int port1, int feature)
{
	return usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
		USB_REQ_SET_FEATURE, USB_RT_PORT, feature, port1,
		NULL, 0, 1000);
}

static int soc_usb3_consis_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

ssize_t soc_usb3_consis_write(struct file *file, const char __user *buf, size_t count,
		loff_t *offset) 
{
	struct seq_file *s = file->private_data;
	struct soc_usb *su = s->private;
	struct usb_hcd *hcd;
	struct xhci_hcd *xhci;
	struct usb_device *hdev;
	u8 content[32];
	int idx = 0, port = 1;
	int usb_controller = 0,  mode = 0;
	int ret;

	hcd = platform_get_drvdata(su->xhci);
	if(hcd == NULL) {
		dev_err(su->dev, "get usb hcd error\n");
		return -EINVAL;
	}

	xhci = hcd_to_xhci(hcd);
	if(xhci == NULL) {
		dev_err(su->dev, "get usb xhci hcd error\n");
		return -EINVAL;
	}

	if(count > 32)
		count = 32;

	memset(content, 0x0, 32);
	if(copy_from_user(content, buf, count)){
		dev_err(su->dev, "copy from user error \n");
		return -EFAULT;
	}

	sscanf(content, "%d %d %d %d", &idx, &usb_controller, &mode, &port);
	dev_info(su->dev, "You will  test usb%d portnum=%d, function: %s!\n", usb_controller, port, soc_usb_test_string(idx));

	if(usb_controller == 2) {
		hdev = xhci->main_hcd->self.root_hub;
    } else {
		hdev = xhci->shared_hcd->self.root_hub;
    }

	if(hdev == NULL) {
		dev_err(su->dev, "get usb device error\n");
		return -EINVAL;
	} else {
		hdev->can_submit = 1;
	}

	switch(idx) {
		case 1:
			ret = soc_usb_set_port_feature(hdev, port | USB_SS_PORT_LS_COMP_MOD << 3, USB_PORT_FEAT_LINK_STATE);
			if (ret < 0) {
				dev_info(su->dev, "can't enable compliance mode %d\n", ret);
			} else {
				dev_info(su->dev, "set compliance successfully\n");
			}
			break;
		case 2:
			ret = soc_usb_set_port_feature(hdev, port, USB_PORT_FEAT_SUSPEND);
			if (ret < 0) {
				dev_info(su->dev, "can't set suspend test %d\n", ret);
			} else {
				dev_info(su->dev, "set suspend successfully\n");
			}
			break;
		case 3:
			ret = soc_usb_clear_port_feature(hdev, port, USB_PORT_FEAT_SUSPEND);
			if (ret < 0) {
				dev_info(su->dev, "can't clear suspend %d\n", ret);
			} else {
				dev_info(su->dev, "clear suspend successfully\n");
			}
			break;
		case 4:
			ret = soc_usb_set_port_feature(hdev, port | USB_SS_PORT_LS_LOOPBACK << 3, USB_PORT_FEAT_LINK_STATE);
			if (ret < 0) {
				dev_info(su->dev, "can't enable loopback mode %d\n", ret);
			} else {
				dev_info(su->dev, "set loopback successfully\n");
			}
			break;
		case 5:
			ret = soc_usb_set_port_feature(hdev, port | USB_SS_PORT_LS_U0 << 3, USB_PORT_FEAT_LINK_STATE);
			if (ret < 0) {
				dev_info(su->dev, "can't enable U0 mode %d\n", ret);
			} else {
				dev_info(su->dev, "set U0 successfully\n");
			}
			break;
		case 6:
			ret = soc_usb_set_port_feature(hdev, port | 0x10 << 8, USB_PORT_FEAT_U1_TIMEOUT);
			if (ret < 0) {
				dev_info(su->dev, "can't enable U1 mode %d\n", ret);
			} else {
				dev_info(su->dev, "set U1 successfully\n");
			}
			break;
		case 7:
			ret = soc_usb_set_port_feature(hdev, port | 0x10 << 8, USB_PORT_FEAT_U2_TIMEOUT);
			if (ret < 0) {
				dev_info(su->dev, "can't enable U2 mode %d\n", ret);
			} else {
				dev_info(su->dev, "set U2 successfully\n");
			}
			break;
		case 8:
			ret = soc_usb_set_port_feature(hdev, port | USB_SS_PORT_LS_U3 << 3, USB_PORT_FEAT_LINK_STATE);
			if (ret < 0) {
				dev_info(su->dev, "can't enable U3 mode %d\n", ret);
			} else {
				dev_info(su->dev, "set U3 successfully\n");
			}
			break;
		case 9:
			ret = usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
                              USB_REQ_SET_FEATURE, USB_RT_PORT, 21, port | (mode << 8),
                              NULL, 0, 1000);
        	if (ret < 0) {
            	dev_info(su->dev, "can't set usb2 test mode %d\n", ret);
        	} else {
            	dev_info(su->dev, "set usb2 test mode successfully\n");
        	}
			break;
		case 10:
			ret = soc_usb_set_port_feature(hdev, port , USB_PORT_FEAT_RESET);
			if (ret < 0) {
				dev_info(su->dev, "can't reset test %d\n", ret);
			} else {
				dev_info(su->dev, "set reset test successfully\n");
			}
			break;
		default:
			dev_err(su->dev, "idx = %d error\n", idx);
			
	}

	if(ret == -ESHUTDOWN) {
		dev_info(su->dev, "Maybe the usb controller has entered suspend mode.\n ");
	} else if(ret == -EPIPE) {
		dev_info(su->dev, "Maybe the port number selection error.\n ");
	} else if(ret == EHOSTUNREACH) {
		dev_info(su->dev, "Maybe the usb controller has enterd suspend mode and is preventing the submission of URBs .\n ");
	}

	return count;
}

static const struct file_operations inno_usb3_consistent_fops = {
	.open			= soc_usb3_consis_open,
	.write			= soc_usb3_consis_write,
};

static int soc_usb_testmode_show(struct seq_file *s, void *unused)
{
	struct soc_usb		*su = s->private;
	unsigned long		flags;
	u32			reg;
	int			ret;

	ret = pm_runtime_resume_and_get(su->dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&su->lock, flags);
	reg = soc_usb_readl(su->regs, SOC_USB_DCTL);
	reg &= SOC_USB_DCTL_TSTCTRL_MASK;
	reg >>= 1;
	spin_unlock_irqrestore(&su->lock, flags);

	switch (reg) {
	case 0:
		seq_puts(s, "no test\n");
		break;
	case USB_TEST_J:
		seq_puts(s, "test_j\n");
		break;
	case USB_TEST_K:
		seq_puts(s, "test_k\n");
		break;
	case USB_TEST_SE0_NAK:
		seq_puts(s, "test_se0_nak\n");
		break;
	case USB_TEST_PACKET:
		seq_puts(s, "test_packet\n");
		break;
	case USB_TEST_FORCE_ENABLE:
		seq_puts(s, "test_force_enable\n");
		break;
	default:
		seq_printf(s, "UNKNOWN %d\n", reg);
	}

	pm_runtime_put_sync(su->dev);

	return 0;
}

static int soc_usb_testmode_open(struct inode *inode, struct file *file)
{
	return single_open(file, soc_usb_testmode_show, inode->i_private);
}

static ssize_t soc_usb_testmode_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct soc_usb		*su = s->private;
	unsigned long		flags;
	u32			testmode = 0;
	char			buf[32];
	int			ret;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "test_j", 6))
		testmode = USB_TEST_J;
	else if (!strncmp(buf, "test_k", 6))
		testmode = USB_TEST_K;
	else if (!strncmp(buf, "test_se0_nak", 12))
		testmode = USB_TEST_SE0_NAK;
	else if (!strncmp(buf, "test_packet", 11))
		testmode = USB_TEST_PACKET;
	else if (!strncmp(buf, "test_force_enable", 17))
		testmode = USB_TEST_FORCE_ENABLE;
	else
		testmode = 0;

	ret = pm_runtime_resume_and_get(su->dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&su->lock, flags);
	soc_usb_gadget_set_test_mode(su, testmode);
	spin_unlock_irqrestore(&su->lock, flags);

	pm_runtime_put_sync(su->dev);

	return count;
}

static const struct file_operations soc_usb_testmode_fops = {
	.open			= soc_usb_testmode_open,
	.write			= soc_usb_testmode_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int soc_usb_link_state_show(struct seq_file *s, void *unused)
{
	struct soc_usb		*su = s->private;
	unsigned long		flags;
	enum soc_usb_link_state	state;
	u32			reg;
	u8			speed;
	int			ret;

	ret = pm_runtime_resume_and_get(su->dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&su->lock, flags);
	reg = soc_usb_readl(su->regs, SOC_USB_GSTS);
	if (SOC_USB_GSTS_CURMOD(reg) != SOC_USB_GSTS_CURMOD_DEVICE) {
		seq_puts(s, "Not available\n");
		spin_unlock_irqrestore(&su->lock, flags);
		pm_runtime_put_sync(su->dev);
		return 0;
	}

	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);
	state = SOC_USB_DSTS_USBLNKST(reg);
	speed = reg & SOC_USB_DSTS_CONNECTSPD;

	seq_printf(s, "%s\n", (speed >= SOC_USB_DSTS_SUPERSPEED) ?
		   soc_usb_gadget_link_string(state) :
		   soc_usb_gadget_hs_link_string(state));
	spin_unlock_irqrestore(&su->lock, flags);

	pm_runtime_put_sync(su->dev);

	return 0;
}

static int soc_usb_link_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, soc_usb_link_state_show, inode->i_private);
}

static ssize_t soc_usb_link_state_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct soc_usb		*su = s->private;
	unsigned long		flags;
	enum soc_usb_link_state	state = 0;
	char			buf[32];
	u32			reg;
	u8			speed;
	int			ret;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "SS.Disabled", 11))
		state = SOC_USB_LINK_STATE_SS_DIS;
	else if (!strncmp(buf, "Rx.Detect", 9))
		state = SOC_USB_LINK_STATE_RX_DET;
	else if (!strncmp(buf, "SS.Inactive", 11))
		state = SOC_USB_LINK_STATE_SS_INACT;
	else if (!strncmp(buf, "Recovery", 8))
		state = SOC_USB_LINK_STATE_RECOV;
	else if (!strncmp(buf, "Compliance", 10))
		state = SOC_USB_LINK_STATE_CMPLY;
	else if (!strncmp(buf, "Loopback", 8))
		state = SOC_USB_LINK_STATE_LPBK;
	else
		return -EINVAL;

	ret = pm_runtime_resume_and_get(su->dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&su->lock, flags);
	reg = soc_usb_readl(su->regs, SOC_USB_GSTS);
	if (SOC_USB_GSTS_CURMOD(reg) != SOC_USB_GSTS_CURMOD_DEVICE) {
		spin_unlock_irqrestore(&su->lock, flags);
		pm_runtime_put_sync(su->dev);
		return -EINVAL;
	}

	reg = soc_usb_readl(su->regs, SOC_USB_DSTS);
	speed = reg & SOC_USB_DSTS_CONNECTSPD;

	if (speed < SOC_USB_DSTS_SUPERSPEED &&
	    state != SOC_USB_LINK_STATE_RECOV) {
		spin_unlock_irqrestore(&su->lock, flags);
		pm_runtime_put_sync(su->dev);
		return -EINVAL;
	}

	soc_usb_gadget_set_link_state(su, state);
	spin_unlock_irqrestore(&su->lock, flags);

	pm_runtime_put_sync(su->dev);

	return count;
}

static const struct file_operations soc_usb_link_state_fops = {
	.open			= soc_usb_link_state_open,
	.write			= soc_usb_link_state_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

struct soc_usb_ep_file_map {
	const char name[25];
	const struct file_operations *const fops;
};

static int soc_usb_transfer_type_show(struct seq_file *s, void *unused)
{
	struct soc_usb_ep		*dep = s->private;
	struct soc_usb		*su = dep->su;
	unsigned long		flags;

	spin_lock_irqsave(&su->lock, flags);
	if (!(dep->flags & SOC_USB_EP_ENABLED) || !dep->endpoint.desc) {
		seq_puts(s, "--\n");
		goto out;
	}

	switch (usb_endpoint_type(dep->endpoint.desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		seq_puts(s, "control\n");
		break;
	case USB_ENDPOINT_XFER_ISOC:
		seq_puts(s, "isochronous\n");
		break;
	case USB_ENDPOINT_XFER_BULK:
		seq_puts(s, "bulk\n");
		break;
	case USB_ENDPOINT_XFER_INT:
		seq_puts(s, "interrupt\n");
		break;
	default:
		seq_puts(s, "--\n");
	}

out:
	spin_unlock_irqrestore(&su->lock, flags);

	return 0;
}

static int soc_usb_trb_ring_show(struct seq_file *s, void *unused)
{
	struct soc_usb_ep		*dep = s->private;
	struct soc_usb		*su = dep->su;
	unsigned long		flags;
	int			i;
	int			ret;

	ret = pm_runtime_resume_and_get(su->dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&su->lock, flags);
	if (dep->number <= 1) {
		seq_puts(s, "--\n");
		goto out;
	}

	seq_puts(s, "buffer_addr,size,type,ioc,isp_imi,csp,chn,lst,hwo\n");

	for (i = 0; i < SOC_USB_TRB_NUM; i++) {
		struct soc_usb_trb *trb = &dep->trb_pool[i];
		unsigned int type = SOC_USB_TRBCTL_TYPE(trb->ctrl);

		seq_printf(s, "%08x%08x,%d,%s,%d,%d,%d,%d,%d,%d       %c%c\n",
				trb->bph, trb->bpl, trb->size,
				soc_usb_trb_type_string(type),
				!!(trb->ctrl & SOC_USB_TRB_CTRL_IOC),
				!!(trb->ctrl & SOC_USB_TRB_CTRL_ISP_IMI),
				!!(trb->ctrl & SOC_USB_TRB_CTRL_CSP),
				!!(trb->ctrl & SOC_USB_TRB_CTRL_CHN),
				!!(trb->ctrl & SOC_USB_TRB_CTRL_LST),
				!!(trb->ctrl & SOC_USB_TRB_CTRL_HWO),
				dep->trb_enqueue == i ? 'E' : ' ',
				dep->trb_dequeue == i ? 'D' : ' ');
	}

out:
	spin_unlock_irqrestore(&su->lock, flags);

	pm_runtime_put_sync(su->dev);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(soc_usb_transfer_type);
DEFINE_SHOW_ATTRIBUTE(soc_usb_trb_ring);

static const struct soc_usb_ep_file_map soc_usb_ep_file_map[] = {
	{ "transfer_type", &soc_usb_transfer_type_fops, },
	{ "trb_ring", &soc_usb_trb_ring_fops, },
};

void soc_usb_debugfs_create_endpoint_dir(struct soc_usb_ep *dep)
{
	struct dentry		*dir;
	int			i;

	dir = debugfs_create_dir(dep->name, dep->su->debug_root);
	for (i = 0; i < ARRAY_SIZE(soc_usb_ep_file_map); i++) {
		const struct file_operations *fops = soc_usb_ep_file_map[i].fops;
		const char *name = soc_usb_ep_file_map[i].name;

		debugfs_create_file(name, 0444, dir, dep, fops);
	}
}

void soc_usb_debugfs_remove_endpoint_dir(struct soc_usb_ep *dep)
{
	debugfs_lookup_and_remove(dep->name, dep->su->debug_root);
}

uint32_t g_select_num = 1;
void soc_usb_debugfs_init(struct soc_usb *su)
{
	struct dentry		*root;

	su->regset = kzalloc(sizeof(*su->regset), GFP_KERNEL);
	if (!su->regset)
		return;

	su->dbg_lsp_select = SOC_USB_LSP_MUX_UNSELECTED;

	su->regset->regs = soc_usb_regs;
	su->regset->nregs = ARRAY_SIZE(soc_usb_regs);
	su->regset->base = su->regs - SOC_USB_GLOBALS_REGS_START;
	su->regset->dev = su->dev;

	root = debugfs_create_dir(dev_name(su->dev), usb_debug_root);
	su->debug_root = root;
	debugfs_create_regset32("regdump", 0444, root, su->regset);

	if (IS_ENABLED(CONFIG_USB_SOC_DRD_GADGET)) {
		debugfs_create_file("testmode", 0644, root, su,
				&soc_usb_testmode_fops);
		debugfs_create_file("link_state", 0644, root, su,
				    &soc_usb_link_state_fops);
	} else {
		debugfs_create_file("consis_test", 0644, root, su,
				    &inno_usb3_consistent_fops);
	}
	debugfs_create_u32("select_ep_num", 0644, root, &g_select_num);
}

void soc_usb_debugfs_exit(struct soc_usb *su)
{
	debugfs_lookup_and_remove(dev_name(su->dev), usb_debug_root);
	kfree(su->regset);
}
