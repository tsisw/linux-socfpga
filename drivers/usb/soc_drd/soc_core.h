/* SPDX-License-Identifier: GPL-2.0 */
/*
 * soc_core.h - SOC USB3 DRD Core Header
 *
 */

#ifndef __DRIVERS_USB_SOC_DRD_CORE_H
#define __DRIVERS_USB_SOC_DRD_CORE_H

#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/role.h>
#include <linux/ulpi/interface.h>

#include <linux/phy/phy.h>

#include <linux/power_supply.h>

#define SOC_USB32_IP		0x3210
#define SOC_USB31_IP		0x3110
#define SOC_USB_IP			0x2010


#define SOC_USB_MSG_MAX	500

/* Global constants */
#define SOC_USB_PULL_UP_TIMEOUT	500	/* ms */
#define SOC_USB_BOUNCE_SIZE	1024	/* size of a superspeed bulk */
#define SOC_USB_EP0_SETUP_SIZE	512
#define SOC_USB_ENDPOINTS_NUM	32
#define SOC_USB_XHCI_RESOURCES_NUM	2
#define SOC_USB_ISOC_MAX_RETRIES	5

#define SOC_USB_SCRATCHBUF_SIZE	4096	/* each buffer is assumed to be 4KiB */
#define SOC_USB_EVENT_BUFFERS_SIZE	4096
#define SOC_USB_EVENT_TYPE_MASK	0xfe

#define SOC_USB_EVENT_TYPE_DEV	0
#define SOC_USB_EVENT_TYPE_CARKIT	3
#define SOC_USB_EVENT_TYPE_I2C	4

#define SOC_USB_DEVICE_EVENT_DISCONNECT		0
#define SOC_USB_DEVICE_EVENT_RESET			1
#define SOC_USB_DEVICE_EVENT_CONNECT_DONE		2
#define SOC_USB_DEVICE_EVENT_LINK_STATUS_CHANGE	3
#define SOC_USB_DEVICE_EVENT_WAKEUP		4
#define SOC_USB_DEVICE_EVENT_HIBER_REQ		5
#define SOC_USB_DEVICE_EVENT_SUSPEND		6
#define SOC_USB_DEVICE_EVENT_SOF			7
#define SOC_USB_DEVICE_EVENT_ERRATIC_ERROR		9
#define SOC_USB_DEVICE_EVENT_CMD_CMPL		10
#define SOC_USB_DEVICE_EVENT_OVERFLOW		11

/* Controller's role while using the OTG block */
#define SOC_USB_OTG_ROLE_IDLE	0
#define SOC_USB_OTG_ROLE_HOST	1
#define SOC_USB_OTG_ROLE_DEVICE	2

#define SOC_USB_GEVNTCOUNT_MASK	0xfffc
#define SOC_USB_GEVNTCOUNT_EHB	BIT(31)
#define SOC_USB_ID_MASK	0xffff0000
#define SOC_USB_REV_MASK	0xffff
#define SOC_USB_ID(p)	(((p) & SOC_USB_ID_MASK) >> 16)

/* SOC_USB registers memory space boundries */
#define SOC_USB_XHCI_REGS_START		0x0
#define SOC_USB_XHCI_REGS_END		0x7fff
#define SOC_USB_GLOBALS_REGS_START		0xc100
#define SOC_USB_GLOBALS_REGS_END		0xc6ff
#define SOC_USB_DEVICE_REGS_START		0xc700
#define SOC_USB_DEVICE_REGS_END		0xcbff
#define SOC_USB_OTG_REGS_START		0xcc00
#define SOC_USB_OTG_REGS_END		0xccff

/* Global Registers */
#define SOC_USB_GTXTHRCFG		0xc108
#define SOC_USB_GRXTHRCFG		0xc10c
#define SOC_USB_GCTL		0xc110
#define SOC_USB_GSTS		0xc118
#define SOC_USB_GUCTL1		0xc11c
#define SOC_USB_GUCTL		0xc12c
#define SOC_USB_GHWPARAMS0		0xc140
#define SOC_USB_GHWPARAMS3		0xc14c
#define SOC_USB_GHWPARAMS6		0xc158
#define SOC_USB_GHWPARAMS7		0xc15c

#define SOC_USB_GUSB2PHYCFG(n)	(0xc200 + ((n) * 0x04))

#define SOC_USB_GUSB3PIPECTL(n)	(0xc2c0 + ((n) * 0x04))

#define SOC_USB_GTXFIFOSIZ(n)	(0xc300 + ((n) * 0x04))
#define SOC_USB_GRXFIFOSIZ(n)	(0xc380 + ((n) * 0x04))

#define SOC_USB_GEVNTADRLO(n)	(0xc400 + ((n) * 0x10))
#define SOC_USB_GEVNTADRHI(n)	(0xc404 + ((n) * 0x10))
#define SOC_USB_GEVNTSIZ(n)	(0xc408 + ((n) * 0x10))
#define SOC_USB_GEVNTCOUNT(n)	(0xc40c + ((n) * 0x10))


/* Device Registers */
#define SOC_USB_DCFG		0xc700
#define SOC_USB_DCTL		0xc704
#define SOC_USB_DEVTEN		0xc708
#define SOC_USB_DSTS		0xc70c
#define SOC_USB_DGCMDPAR		0xc710
#define SOC_USB_DGCMD		0xc714
#define SOC_USB_DALEPENA		0xc720

#define SOC_USB_DEP_BASE(n)	(0xc800 + ((n) * 0x10))
#define SOC_USB_DEPCMDPAR2		0x00
#define SOC_USB_DEPCMDPAR1		0x04
#define SOC_USB_DEPCMDPAR0		0x08
#define SOC_USB_DEPCMD		0x0c

#define SOC_USB_DEV_IMOD(n)	(0xca00 + ((n) * 0x4))

/* Bit fields */

/* Global RX Threshold Configuration Register for SOC USB30 only*/
#define SOC_USB_GRXTHRCFG_MAXRXBURSTSIZE(n) (((n) & 0x1f) << 19)
#define SOC_USB_GRXTHRCFG_RXPKTCNT(n) (((n) & 0xf) << 24)
#define SOC_USB_GRXTHRCFG_PKTCNTSEL BIT(29)

/* Global TX Threshold Configuration Register for SOC USB30 only*/
#define SOC_USB_GTXTHRCFG_MAXTXBURSTSIZE(n) (((n) & 0xff) << 16)
#define SOC_USB_GTXTHRCFG_TXPKTCNT(n) (((n) & 0xf) << 24)
#define SOC_USB_GTXTHRCFG_PKTCNTSEL BIT(29)

/* Global RX Threshold Configuration Register for SOC USB31/USB32 only */
#define SOC_USB32_GRXTHRCFG_MAXRXBURSTSIZE(n)	(((n) & 0x1f) << 16)
#define SOC_USB32_GRXTHRCFG_RXPKTCNT(n)		(((n) & 0x1f) << 21)
#define SOC_USB32_GRXTHRCFG_PKTCNTSEL		BIT(26)
#define SOC_USB32_RXTHRNUMPKTSEL_HS_PRD		BIT(15)
#define SOC_USB32_RXTHRNUMPKT_HS_PRD(n)		(((n) & 0x3) << 13)
#define SOC_USB32_RXTHRNUMPKTSEL_PRD		BIT(10)
#define SOC_USB32_RXTHRNUMPKT_PRD(n)		(((n) & 0x1f) << 5)
#define SOC_USB32_MAXRXBURSTSIZE_PRD(n)		((n) & 0x1f)

/* Global TX Threshold Configuration Register for SOC USB31/USB32 only */
#define SOC_USB32_GTXTHRCFG_MAXTXBURSTSIZE(n)	(((n) & 0x1f) << 16)
#define SOC_USB32_GTXTHRCFG_TXPKTCNT(n)		(((n) & 0x1f) << 21)
#define SOC_USB32_GTXTHRCFG_PKTCNTSEL		BIT(26)
#define SOC_USB32_TXTHRNUMPKTSEL_HS_PRD		BIT(15)
#define SOC_USB32_TXTHRNUMPKT_HS_PRD(n)		(((n) & 0x3) << 13)
#define SOC_USB32_TXTHRNUMPKTSEL_PRD		BIT(10)
#define SOC_USB32_TXTHRNUMPKT_PRD(n)		(((n) & 0x1f) << 5)
#define SOC_USB32_MAXTXBURSTSIZE_PRD(n)		((n) & 0x1f)

/* Global Configuration Register */
#define SOC_USB_GCTL_U2RSTECN	BIT(16)
#define SOC_USB_GCTL_PRTCAP(n)	(((n) & (3 << 12)) >> 12)
#define SOC_USB_GCTL_PRTCAPDIR(n)	((n) << 12)
#define SOC_USB_GCTL_PRTCAP_HOST	1
#define SOC_USB_GCTL_PRTCAP_DEVICE	2
#define SOC_USB_GCTL_PRTCAP_OTG	3

#define SOC_USB_GCTL_CORESOFTRESET		BIT(11)
#define SOC_USB_GCTL_SCALEDOWN(n)		((n) << 4)
#define SOC_USB_GCTL_SCALEDOWN_MASK	SOC_USB_GCTL_SCALEDOWN(3)
#define SOC_USB_GCTL_DISSCRAMBLE		BIT(3)
#define SOC_USB_GCTL_U2EXIT_LFPS		BIT(2)
#define SOC_USB_GCTL_DSBLCLKGTNG		BIT(0)

/* Global User Control 1 Register */
#define SOC_USB_GUCTL1_DEV_DECOUPLE_L1L2_EVT	BIT(31)
#define SOC_USB_GUCTL1_TX_IPGAP_LINECHECK_DIS	BIT(28)
#define SOC_USB_GUCTL1_DEV_L1_EXIT_BY_HW		BIT(24)
#define SOC_USB_GUCTL1_RESUME_OPMODE_HS_HOST	BIT(10)

/* Global Status Register */
#define SOC_USB_GSTS_HOST_IP	BIT(7)
#define SOC_USB_GSTS_DEVICE_IP	BIT(6)
#define SOC_USB_GSTS_BUS_ERR_ADDR_VLD	BIT(4)
#define SOC_USB_GSTS_CURMOD(n)	((n) & 0x3)
#define SOC_USB_GSTS_CURMOD_DEVICE	0
#define SOC_USB_GSTS_CURMOD_HOST	1

/* Global USB2 PHY Configuration Register */
#define SOC_USB_GUSB2PHYCFG_PHYSOFTRST	BIT(31)
#define SOC_USB_GUSB2PHYCFG_SUSPHY		BIT(6)
#define SOC_USB_GUSB2PHYCFG_PHYIF(n)	(n << 3)
#define SOC_USB_GUSB2PHYCFG_PHYIF_MASK	SOC_USB_GUSB2PHYCFG_PHYIF(1)
#define SOC_USB_GUSB2PHYCFG_USBTRDTIM(n)	(n << 10)
#define SOC_USB_GUSB2PHYCFG_USBTRDTIM_MASK	SOC_USB_GUSB2PHYCFG_USBTRDTIM(0xf)
#define USBTRDTIM_UTMI_8_BIT		9
#define USBTRDTIM_UTMI_16_BIT		5
#define UTMI_PHYIF_16_BIT		1
#define UTMI_PHYIF_8_BIT		0

/* Global USB3 PIPE Control Register */
#define SOC_USB_GUSB3PIPECTL_PHYSOFTRST	BIT(31)
#define SOC_USB_GUSB3PIPECTL_U2SSINP3OK	BIT(29)
#define SOC_USB_GUSB3PIPECTL_DISRXDETINP3	BIT(28)
#define SOC_USB_GUSB3PIPECTL_UX_EXIT_PX	BIT(27)
#define SOC_USB_GUSB3PIPECTL_REQP1P2P3	BIT(24)
#define SOC_USB_GUSB3PIPECTL_DEP1P2P3(n)	((n) << 19)
#define SOC_USB_GUSB3PIPECTL_DEP1P2P3_MASK	SOC_USB_GUSB3PIPECTL_DEP1P2P3(7)
#define SOC_USB_GUSB3PIPECTL_DEP1P2P3_EN	SOC_USB_GUSB3PIPECTL_DEP1P2P3(1)
#define SOC_USB_GUSB3PIPECTL_DEPOCHANGE	BIT(18)
#define SOC_USB_GUSB3PIPECTL_SUSPHY	BIT(17)
#define SOC_USB_GUSB3PIPECTL_LFPSFILT	BIT(9)
#define SOC_USB_GUSB3PIPECTL_RX_DETOPOLL	BIT(8)

/* Global TX Fifo Size Register */
#define SOC_USB32_GTXFIFOSIZ_TXFRAMNUM	BIT(15)		/* SOC USB only */
#define SOC_USB32_GTXFIFOSIZ_TXFDEP(n)	((n) & 0x7fff)	/* SOC USB only */
#define SOC_USB_GTXFIFOSIZ_TXFDEP(n)	((n) & 0xffff)
#define SOC_USB_GTXFIFOSIZ_TXFSTADDR(n)	((n) & 0xffff0000)

/* Global RX Fifo Size Register */
#define SOC_USB32_GRXFIFOSIZ_RXFDEP(n)	((n) & 0x7fff)	/* SOC USB only */
#define SOC_USB_GRXFIFOSIZ_RXFDEP(n)	((n) & 0xffff)

/* Global Event Size Registers */
#define SOC_USB_GEVNTSIZ_INTMASK		BIT(31)
#define SOC_USB_GEVNTSIZ_SIZE(n)		((n) & 0xffff)

/* Global HWPARAMS0 Register */
#define SOC_USB_GHWPARAMS0_MODE(n)		((n) & 0x3)
#define SOC_USB_GHWPARAMS0_MODE_GADGET	0
#define SOC_USB_GHWPARAMS0_MODE_HOST	1
#define SOC_USB_GHWPARAMS0_MODE_DRD	2
#define SOC_USB_GHWPARAMS0_MBUS_TYPE(n)	(((n) >> 3) & 0x7)
#define SOC_USB_GHWPARAMS0_SBUS_TYPE(n)	(((n) >> 6) & 0x3)
#define SOC_USB_GHWPARAMS0_MDWIDTH(n)	(((n) >> 8) & 0xff)
#define SOC_USB_GHWPARAMS0_SDWIDTH(n)	(((n) >> 16) & 0xff)
#define SOC_USB_GHWPARAMS0_AWIDTH(n)	(((n) >> 24) & 0xff)


/* Global HWPARAMS3 Register */
#define SOC_USB_GHWPARAMS3_SSPHY_IFC(n)		((n) & 3)
#define SOC_USB_GHWPARAMS3_SSPHY_IFC_DIS		0
#define SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN1		1
#define SOC_USB_GHWPARAMS3_SSPHY_IFC_GEN2		2 /* SOC USB only */
#define SOC_USB_GHWPARAMS3_HSPHY_IFC(n)		(((n) & (3 << 2)) >> 2)
#define SOC_USB_GHWPARAMS3_HSPHY_IFC_DIS		0
#define SOC_USB_GHWPARAMS3_HSPHY_IFC_UTMI		1
#define SOC_USB_GHWPARAMS3_HSPHY_IFC_ULPI		2
#define SOC_USB_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI	3


/* Global HWPARAMS6 Register */
#define SOC_USB_GHWPARAMS6_EN_FPGA			BIT(7)

/* soc usb higher only */
#define SOC_USB_GHWPARAMS6_MDWIDTH(n)		((n) & (0x3 << 8))

/* Global HWPARAMS7 Register */
#define SOC_USB_GHWPARAMS7_RAM1_DEPTH(n)	((n) & 0xffff)
#define SOC_USB_GHWPARAMS7_RAM2_DEPTH(n)	(((n) >> 16) & 0xffff)



/* Device Configuration Register */
#define SOC_USB_DCFG_NUMLANES_MASK	(0x3 << 30)
#define SOC_USB_DCFG_NUMLANES(n)	(((n) & 0x3) << 30) /* SOC USB32 only */

#define SOC_USB_DCFG_DEVADDR(addr)	((addr) << 3)
#define SOC_USB_DCFG_DEVADDR_MASK	SOC_USB_DCFG_DEVADDR(0x7f)

#define SOC_USB_DCFG_SPEED_MASK	(7 << 0)
#define SOC_USB_DCFG_SUPERSPEED_PLUS (5 << 0)  /* soc usb higher only */
#define SOC_USB_DCFG_SUPERSPEED	(4 << 0)
#define SOC_USB_DCFG_HIGHSPEED	(0 << 0)
#define SOC_USB_DCFG_FULLSPEED	BIT(0)

#define SOC_USB_DCFG_NUMP_SHIFT	17
#define SOC_USB_DCFG_NUMP(n)	(((n) >> SOC_USB_DCFG_NUMP_SHIFT) & 0x1f)
#define SOC_USB_DCFG_NUMP_MASK	(0x1f << SOC_USB_DCFG_NUMP_SHIFT)
#define SOC_USB_DCFG_LPM_CAP	BIT(22)

/* Device Control Register */
#define SOC_USB_DCTL_RUN_STOP	BIT(31)
#define SOC_USB_DCTL_CSFTRST	BIT(30)

#define SOC_USB_DCTL_HIRD_THRES_MASK	(0x1f << 24)
#define SOC_USB_DCTL_HIRD_THRES(n)	((n) << 24)


/* These apply for core versions 1.94a and later */
#define SOC_USB_DCTL_NYET_THRES(n)		(((n) & 0xf) << 20)

#define SOC_USB_DCTL_KEEP_CONNECT		BIT(19)
#define SOC_USB_DCTL_L1_HIBER_EN		BIT(18)
#define SOC_USB_DCTL_CRS			BIT(17)
#define SOC_USB_DCTL_CSS			BIT(16)

#define SOC_USB_DCTL_INITU2ENA		BIT(12)
#define SOC_USB_DCTL_ACCEPTU2ENA		BIT(11)
#define SOC_USB_DCTL_INITU1ENA		BIT(10)
#define SOC_USB_DCTL_ACCEPTU1ENA		BIT(9)
#define SOC_USB_DCTL_TSTCTRL_MASK		(0xf << 1)

#define SOC_USB_DCTL_ULSTCHNGREQ_MASK	(0x0f << 5)
#define SOC_USB_DCTL_ULSTCHNGREQ(n) (((n) << 5) & SOC_USB_DCTL_ULSTCHNGREQ_MASK)

#define SOC_USB_DCTL_ULSTCHNG_NO_ACTION	(SOC_USB_DCTL_ULSTCHNGREQ(0))
#define SOC_USB_DCTL_ULSTCHNG_SS_DISABLED	(SOC_USB_DCTL_ULSTCHNGREQ(4))
#define SOC_USB_DCTL_ULSTCHNG_RX_DETECT	(SOC_USB_DCTL_ULSTCHNGREQ(5))
#define SOC_USB_DCTL_ULSTCHNG_SS_INACTIVE	(SOC_USB_DCTL_ULSTCHNGREQ(6))
#define SOC_USB_DCTL_ULSTCHNG_RECOVERY	(SOC_USB_DCTL_ULSTCHNGREQ(8))
#define SOC_USB_DCTL_ULSTCHNG_COMPLIANCE	(SOC_USB_DCTL_ULSTCHNGREQ(10))
#define SOC_USB_DCTL_ULSTCHNG_LOOPBACK	(SOC_USB_DCTL_ULSTCHNGREQ(11))

/* Device Event Enable Register */
#define SOC_USB_DEVTEN_VNDRDEVTSTRCVEDEN	BIT(12)
#define SOC_USB_DEVTEN_ERRTICERREN		BIT(9)
#define SOC_USB_DEVTEN_SOFEN		BIT(7)
#define SOC_USB_DEVTEN_U3L2L1SUSPEN	BIT(6)
#define SOC_USB_DEVTEN_HIBERNATIONREQEVTEN	BIT(5)
#define SOC_USB_DEVTEN_WKUPEVTEN		BIT(4)
#define SOC_USB_DEVTEN_ULSTCNGEN		BIT(3)
#define SOC_USB_DEVTEN_CONNECTDONEEN	BIT(2)
#define SOC_USB_DEVTEN_USBRSTEN		BIT(1)
#define SOC_USB_DEVTEN_DISCONNEVTEN	BIT(0)

#define SOC_USB_DSTS_CONNLANES(n)		(((n) >> 30) & 0x3) /* soc usb32 only */

/* These apply for core versions 1.94a and later */
#define SOC_USB_DSTS_RSS			BIT(25)
#define SOC_USB_DSTS_SSS			BIT(24)

#define SOC_USB_DSTS_COREIDLE		BIT(23)
#define SOC_USB_DSTS_DEVCTRLHLT		BIT(22)

#define SOC_USB_DSTS_USBLNKST_MASK		(0x0f << 18)
#define SOC_USB_DSTS_USBLNKST(n)		(((n) & SOC_USB_DSTS_USBLNKST_MASK) >> 18)

#define SOC_USB_DSTS_RXFIFOEMPTY		BIT(17)

#define SOC_USB_DSTS_SOFFN_MASK		(0x3fff << 3)
#define SOC_USB_DSTS_SOFFN(n)		(((n) & SOC_USB_DSTS_SOFFN_MASK) >> 3)

#define SOC_USB_DSTS_CONNECTSPD		(7 << 0)

#define SOC_USB_DSTS_SUPERSPEED_PLUS	(5 << 0) /* soc usb higher only */
#define SOC_USB_DSTS_SUPERSPEED		(4 << 0)
#define SOC_USB_DSTS_HIGHSPEED		(0 << 0)
#define SOC_USB_DSTS_FULLSPEED		BIT(0)

/* Device Generic Command Register */
#define SOC_USB_DGCMD_SET_PERIODIC_PAR	0x02
#define SOC_USB_DGCMD_SET_SCRATCHPAD_ADDR_LO	0x04
#define SOC_USB_DGCMD_SET_SCRATCHPAD_ADDR_HI	0x05
#define SOC_USB_DGCMD_DEV_NOTIFICATION	0x07
#define SOC_USB_DGCMD_SELECTED_FIFO_FLUSH	0x09
#define SOC_USB_DGCMD_ALL_FIFO_FLUSH	0x0a
#define SOC_USB_DGCMD_SET_ENDPOINT_NRDY	0x0c
#define SOC_USB_DGCMD_RESTART_AFTER_DISC	0x11


#define SOC_USB_DGCMD_STATUS(n)		(((n) >> 12) & 0x0F)
#define SOC_USB_DGCMD_CMDACT		BIT(10)
#define SOC_USB_DGCMD_CMDIOC		BIT(8)

/* Device Generic Command Parameter Register */
#define SOC_USB_DGCMDPAR_FIFO_NUM(n)		((n) << 0)
#define SOC_USB_DGCMDPAR_RX_FIFO			(0 << 5)
#define SOC_USB_DGCMDPAR_TX_FIFO			BIT(5)
#define SOC_USB_DGCMDPAR_LOOPBACK_DIS		(0 << 0)
#define SOC_USB_DGCMDPAR_LOOPBACK_ENA		BIT(0)
#define SOC_USB_DGCMDPAR_DN_FUNC_WAKE		BIT(0)
#define SOC_USB_DGCMDPAR_INTF_SEL(n)		((n) << 4)

/* Device Endpoint Command Register */
#define SOC_USB_DEPCMD_PARAM_SHIFT		16
#define SOC_USB_DEPCMD_PARAM(x)		((x) << SOC_USB_DEPCMD_PARAM_SHIFT)
#define SOC_USB_DEPCMD_GET_RSC_IDX(x)	(((x) >> SOC_USB_DEPCMD_PARAM_SHIFT) & 0x7f)
#define SOC_USB_DEPCMD_STATUS(x)		(((x) >> 12) & 0x0F)
#define SOC_USB_DEPCMD_HIPRI_FORCERM	BIT(11)
#define SOC_USB_DEPCMD_CLEARPENDIN		BIT(11)
#define SOC_USB_DEPCMD_CMDACT		BIT(10)
#define SOC_USB_DEPCMD_CMDIOC		BIT(8)

#define SOC_USB_DEPCMD_DEPSTARTCFG		(0x09 << 0)
#define SOC_USB_DEPCMD_ENDTRANSFER		(0x08 << 0)
#define SOC_USB_DEPCMD_UPDATETRANSFER	(0x07 << 0)
#define SOC_USB_DEPCMD_STARTTRANSFER	(0x06 << 0)
#define SOC_USB_DEPCMD_CLEARSTALL		(0x05 << 0)
#define SOC_USB_DEPCMD_SETSTALL		(0x04 << 0)
#define SOC_USB_DEPCMD_GETEPSTATE		(0x03 << 0)
#define SOC_USB_DEPCMD_SETTRANSFRESOURCE	(0x02 << 0)
#define SOC_USB_DEPCMD_SETEPCONFIG		(0x01 << 0)

#define SOC_USB_DEPCMD_CMD(x)		((x) & 0xf)

/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define SOC_USB_DALEPENA_EP(n)		BIT(n)

#define SOC_USB_DEPCMD_TYPE_CONTROL	0
#define SOC_USB_DEPCMD_TYPE_ISOC		1
#define SOC_USB_DEPCMD_TYPE_BULK		2
#define SOC_USB_DEPCMD_TYPE_INTR		3

#define SOC_USB_DEV_IMOD_COUNT_SHIFT	16
#define SOC_USB_DEV_IMOD_COUNT_MASK	(0xffff << 16)
#define SOC_USB_DEV_IMOD_INTERVAL_SHIFT	0
#define SOC_USB_DEV_IMOD_INTERVAL_MASK	(0xffff << 0)

/* Structures */

struct soc_usb_trb;

/**
 * struct soc_usb_event_buffer - Software event buffer representation
 * @buf: _THE_ buffer
 * @cache: The buffer cache used in the threaded interrupt
 * @length: size of this buffer
 * @lpos: event offset
 * @count: cache of last read event count register
 * @flags: flags related to this event buffer
 * @dma: dma_addr_t
 * @su: pointer to SOC USB controller
 */
struct soc_usb_event_buffer {
	void			*buf;
	void			*cache;
	unsigned int		length;
	unsigned int		lpos;
	unsigned int		count;
	unsigned int		flags;

#define SOC_USB_EVENT_PENDING	BIT(0)

	dma_addr_t		dma;

	struct soc_usb		*su;
};

#define SOC_USB_EP_FLAG_STALLED	BIT(0)
#define SOC_USB_EP_FLAG_WEDGED	BIT(1)

#define SOC_USB_EP_DIRECTION_TX	true
#define SOC_USB_EP_DIRECTION_RX	false

#define SOC_USB_TRB_NUM		256

/**
 * struct soc_usb_ep - device side endpoint representation
 * @endpoint: usb endpoint
 * @cancelled_list: list of cancelled requests for this endpoint
 * @pending_list: list of pending requests for this endpoint
 * @started_list: list of started requests on this endpoint
 * @regs: pointer to first endpoint register
 * @trb_pool: array of transaction buffers
 * @trb_pool_dma: dma address of @trb_pool
 * @trb_enqueue: enqueue 'pointer' into TRB array
 * @trb_dequeue: dequeue 'pointer' into TRB array
 * @su: pointer to SOC USB controller
 * @saved_state: ep state saved during hibernation
 * @flags: endpoint flags (wedged, stalled, ...)
 * @number: endpoint number (1 - 15)
 * @type: set to bmAttributes & USB_ENDPOINT_XFERTYPE_MASK
 * @resource_index: Resource transfer index
 * @frame_number: set to the frame number we want this transfer to start (ISOC)
 * @interval: the interval on which the ISOC transfer is started
 * @name: a human readable name e.g. ep1out-bulk
 * @direction: true for TX, false for RX
 * @stream_capable: true when streams are enabled
 * @combo_num: the test combination BIT[15:14] of the frame number to test
 *		isochronous START TRANSFER command failure workaround
 * @start_cmd_status: the status of testing START TRANSFER command with
 *		combo_num = 'b00
 */
struct soc_usb_ep {
	struct usb_ep		endpoint;
	struct list_head	cancelled_list;
	struct list_head	pending_list;
	struct list_head	started_list;

	void __iomem		*regs;

	struct soc_usb_trb		*trb_pool;
	dma_addr_t		trb_pool_dma;
	struct soc_usb		*su;

	u32			saved_state;
	unsigned int		flags;
#define SOC_USB_EP_ENABLED			BIT(0)
#define SOC_USB_EP_STALL			BIT(1)
#define SOC_USB_EP_WEDGE			BIT(2)
#define SOC_USB_EP_TRANSFER_STARTED	BIT(3)
#define SOC_USB_EP_END_TRANSFER_PENDING	BIT(4)
#define SOC_USB_EP_PENDING_REQUEST		BIT(5)
#define SOC_USB_EP_DELAY_START		BIT(6)
#define SOC_USB_EP_WAIT_TRANSFER_COMPLETE	BIT(7)
#define SOC_USB_EP_IGNORE_NEXT_NOSTREAM	BIT(8)
#define SOC_USB_EP_FORCE_RESTART_STREAM	BIT(9)
#define SOC_USB_EP_FIRST_STREAM_PRIMED	BIT(10)
#define SOC_USB_EP_PENDING_CLEAR_STALL	BIT(11)
#define SOC_USB_EP_TXFIFO_RESIZED		BIT(12)
#define SOC_USB_EP_DELAY_STOP             BIT(13)

	/* This last one is specific to EP0 */
#define SOC_USB_EP0_DIR_IN			BIT(31)

	/*
	 * IMPORTANT: we *know* we have 256 TRBs in our @trb_pool, so we will
	 * use a u8 type here. If anybody decides to increase number of TRBs to
	 * anything larger than 256 - I can't see why people would want to do
	 * this though - then this type needs to be changed.
	 *
	 * By using u8 types we ensure that our % operator when incrementing
	 * enqueue and dequeue get optimized away by the compiler.
	 */
	u8			trb_enqueue;
	u8			trb_dequeue;

	u8			number;
	u8			type;
	u8			resource_index;
	u32			frame_number;
	u32			interval;

	char			name[20];

	unsigned		direction:1;
	unsigned		stream_capable:1;

	/* For isochronous START TRANSFER workaround only */
	u8			combo_num;
	int			start_cmd_status;
};

enum soc_usb_phy {
	SOC_USB_PHY_UNKNOWN = 0,
	SOC_USB_PHY_USB3,
	SOC_USB_PHY_USB2,
};

enum soc_usb_ep0_next {
	SOC_USB_EP0_UNKNOWN = 0,
	SOC_USB_EP0_COMPLETE,
	SOC_USB_EP0_NRDY_DATA,
	SOC_USB_EP0_NRDY_STATUS,
};

enum soc_usb_ep0_state {
	EP0_UNCONNECTED		= 0,
	EP0_SETUP_PHASE,
	EP0_DATA_PHASE,
	EP0_STATUS_PHASE,
};

enum soc_usb_link_state {
	/* In SuperSpeed */
	SOC_USB_LINK_STATE_U0		= 0x00, /* in HS, means ON */
	SOC_USB_LINK_STATE_U1		= 0x01,
	SOC_USB_LINK_STATE_U2		= 0x02, /* in HS, means SLEEP */
	SOC_USB_LINK_STATE_U3		= 0x03, /* in HS, means SUSPEND */
	SOC_USB_LINK_STATE_SS_DIS		= 0x04,
	SOC_USB_LINK_STATE_RX_DET		= 0x05, /* in HS, means Early Suspend */
	SOC_USB_LINK_STATE_SS_INACT	= 0x06,
	SOC_USB_LINK_STATE_POLL		= 0x07,
	SOC_USB_LINK_STATE_RECOV		= 0x08,
	SOC_USB_LINK_STATE_HRESET		= 0x09,
	SOC_USB_LINK_STATE_CMPLY		= 0x0a,
	SOC_USB_LINK_STATE_LPBK		= 0x0b,
	SOC_USB_LINK_STATE_RESET		= 0x0e,
	SOC_USB_LINK_STATE_RESUME		= 0x0f,
	SOC_USB_LINK_STATE_MASK		= 0x0f,
};

/* TRB Length, PCM and Status */
#define SOC_USB_TRB_SIZE_MASK	(0x00ffffff)
#define SOC_USB_TRB_SIZE_LENGTH(n)	((n) & SOC_USB_TRB_SIZE_MASK)
#define SOC_USB_TRB_SIZE_PCM1(n)	(((n) & 0x03) << 24)
#define SOC_USB_TRB_SIZE_TRBSTS(n)	(((n) & (0x0f << 28)) >> 28)

#define SOC_USB_TRBSTS_OK			0
#define SOC_USB_TRBSTS_MISSED_ISOC		1
#define SOC_USB_TRBSTS_SETUP_PENDING	2
#define SOC_USB_TRB_STS_XFER_IN_PROG	4

/* TRB Control */
#define SOC_USB_TRB_CTRL_HWO		BIT(0)
#define SOC_USB_TRB_CTRL_LST		BIT(1)
#define SOC_USB_TRB_CTRL_CHN		BIT(2)
#define SOC_USB_TRB_CTRL_CSP		BIT(3)
#define SOC_USB_TRB_CTRL_TRBCTL(n)		(((n) & 0x3f) << 4)
#define SOC_USB_TRB_CTRL_ISP_IMI		BIT(10)
#define SOC_USB_TRB_CTRL_IOC		BIT(11)
#define SOC_USB_TRB_CTRL_SID_SOFN(n)	(((n) & 0xffff) << 14)
#define SOC_USB_TRB_CTRL_GET_SID_SOFN(n)	(((n) & (0xffff << 14)) >> 14)

#define SOC_USB_TRBCTL_TYPE(n)		((n) & (0x3f << 4))
#define SOC_USB_TRBCTL_NORMAL		SOC_USB_TRB_CTRL_TRBCTL(1)
#define SOC_USB_TRBCTL_CONTROL_SETUP	SOC_USB_TRB_CTRL_TRBCTL(2)
#define SOC_USB_TRBCTL_CONTROL_STATUS2	SOC_USB_TRB_CTRL_TRBCTL(3)
#define SOC_USB_TRBCTL_CONTROL_STATUS3	SOC_USB_TRB_CTRL_TRBCTL(4)
#define SOC_USB_TRBCTL_CONTROL_DATA	SOC_USB_TRB_CTRL_TRBCTL(5)
#define SOC_USB_TRBCTL_ISOCHRONOUS_FIRST	SOC_USB_TRB_CTRL_TRBCTL(6)
#define SOC_USB_TRBCTL_ISOCHRONOUS		SOC_USB_TRB_CTRL_TRBCTL(7)
#define SOC_USB_TRBCTL_LINK_TRB		SOC_USB_TRB_CTRL_TRBCTL(8)

/**
 * struct soc_usb_trb - transfer request block (hw format)
 * @bpl: DW0-3
 * @bph: DW4-7
 * @size: DW8-B
 * @ctrl: DWC-F
 */
struct soc_usb_trb {
	u32		bpl;
	u32		bph;
	u32		size;
	u32		ctrl;
} __packed;

/**
 * struct soc_usb_hwparams - copy of HWPARAMS registers
 * @hwparams0: GHWPARAMS0
 * @hwparams3: GHWPARAMS3
 * @hwparams6: GHWPARAMS6
 * @hwparams7: GHWPARAMS7
 */
struct soc_usb_hwparams {
	u32	hwparams0;
	u32	hwparams3;
	u32	hwparams6;
	u32	hwparams7;
};

/* HWPARAMS0 */
#define SOC_USB_MODE(n)		((n) & 0x7)

/* HWPARAMS1 */
#define SOC_USB_NUM_INT(n)		(((n) & (0x3f << 15)) >> 15)

/* HWPARAMS3 */
#define SOC_USB_NUM_IN_EPS_MASK	(0x1f << 18)
#define SOC_USB_NUM_EPS_MASK	(0x3f << 12)
#define SOC_USB_NUM_EPS(p)		(((p)->hwparams3 &		\
			(SOC_USB_NUM_EPS_MASK)) >> 12)
#define SOC_USB_NUM_IN_EPS(p)	(((p)->hwparams3 &		\
			(SOC_USB_NUM_IN_EPS_MASK)) >> 18)

/* HWPARAMS7 */
#define SOC_USB_RAM1_DEPTH(n)	((n) & 0xffff)

/**
 * struct soc_usb_request - representation of a transfer request
 * @request: struct usb_request to be transferred
 * @list: a list_head used for request queueing
 * @dep: struct soc_usb_ep owning this request
 * @sg: pointer to first incomplete sg
 * @start_sg: pointer to the sg which should be queued next
 * @num_pending_sgs: counter to pending sgs
 * @num_queued_sgs: counter to the number of sgs which already got queued
 * @remaining: amount of data remaining
 * @status: internal soc_usb request status tracking
 * @epnum: endpoint number to which this request refers
 * @trb: pointer to struct soc_usb_trb
 * @trb_dma: DMA address of @trb
 * @num_trbs: number of TRBs used by this request
 * @needs_extra_trb: true when request needs one extra TRB (either due to ZLP
 *	or unaligned OUT)
 * @direction: IN or OUT direction flag
 * @mapped: true when request has been dma-mapped
 */
struct soc_usb_request {
	struct usb_request	request;
	struct list_head	list;
	struct soc_usb_ep		*dep;
	struct scatterlist	*sg;
	struct scatterlist	*start_sg;

	unsigned int		num_pending_sgs;
	unsigned int		num_queued_sgs;
	unsigned int		remaining;

	unsigned int		status;
#define SOC_USB_REQUEST_STATUS_QUEUED		0
#define SOC_USB_REQUEST_STATUS_STARTED		1
#define SOC_USB_REQUEST_STATUS_DISCONNECTED	2
#define SOC_USB_REQUEST_STATUS_DEQUEUED		3
#define SOC_USB_REQUEST_STATUS_STALLED		4
#define SOC_USB_REQUEST_STATUS_COMPLETED		5
#define SOC_USB_REQUEST_STATUS_UNKNOWN		-1

	u8			epnum;
	struct soc_usb_trb		*trb;
	dma_addr_t		trb_dma;

	unsigned int		num_trbs;

	unsigned int		needs_extra_trb:1;
	unsigned int		direction:1;
	unsigned int		mapped:1;
};

/**
 * struct soc_usb - representation of our controller
 * @drd_work: workqueue used for role swapping
 * @ep0_trb: trb which is used for the ctrl_req
 * @bounce: address of bounce buffer
 * @setup_buf: used while precessing STD USB requests
 * @ep0_trb_addr: dma address of @ep0_trb
 * @bounce_addr: dma address of @bounce
 * @ep0_usb_req: dummy req used while handling STD USB requests
 * @ep0_in_setup: one control transfer is completed and enter setup phase
 * @lock: for synchronizing
 * @mutex: for mode switching
 * @dev: pointer to our struct device
 * @sysdev: pointer to the DMA-capable device
 * @xhci: pointer to our xHCI child
 * @xhci_resources: struct resources for our @xhci child
 * @ev_buf: struct soc_usb_event_buffer pointer
 * @eps: endpoint array
 * @gadget: device side representation of the peripheral controller
 * @gadget_driver: pointer to the gadget driver
 * @bus_clk: clock for accessing the registers
 * @ref_clk: reference clock
 * @susp_clk: clock used when the SS phy is in low power (S3) state
 * @reset: reset control
 * @regs: base address for our registers
 * @regs_size: address space size
 * @irq_gadget: peripheral controller's IRQ number
 * @otg_irq: IRQ number for OTG IRQs
 * @current_otg_role: current role of operation while using the OTG block
 * @desired_otg_role: desired role of operation while using the OTG block
 * @otg_restart_host: flag that OTG controller needs to restart host
 * @u1u2: only used on revisions <1.83a for workaround
 * @maximum_speed: maximum speed requested (mainly for testing purposes)
 * @max_ssp_rate: SuperSpeed Plus maximum signaling rate and lane count
 * @gadget_max_speed: maximum gadget speed requested
 * @gadget_ssp_rate: Gadget driver's maximum supported SuperSpeed Plus signaling
 *			rate and lane count.
 * @ip: controller's ID
 * @revision: controller's version of an IP
 * @version_type: VERSIONTYPE register contents, a sub release of a revision
 * @dr_mode: requested mode of operation
 * @current_dr_role: current role of operation when in dual-role mode
 * @desired_dr_role: desired role of operation when in dual-role mode
 * @edev: extcon handle
 * @edev_nb: extcon notifier
 * @hsphy_mode: UTMI phy mode, one of following:
 *		- USBPHY_INTERFACE_MODE_UTMI
 *		- USBPHY_INTERFACE_MODE_UTMIW
 * @role_sw: usb_role_switch handle
 * @role_switch_default_mode: default operation mode of controller while
 *			usb role is USB_ROLE_NONE.
 * @usb_psy: pointer to power supply interface.
 * @usb2_phy: pointer to USB2 PHY
 * @usb3_phy: pointer to USB3 PHY
 * @usb2_generic_phy: pointer to USB2 PHY
 * @usb3_generic_phy: pointer to USB3 PHY
 * @phys_ready: flag to indicate that PHYs are ready
 * @ulpi: pointer to ulpi interface
 * @ulpi_ready: flag to indicate that ULPI is initialized
 * @u2sel: parameter from Set SEL request.
 * @u2pel: parameter from Set SEL request.
 * @u1sel: parameter from Set SEL request.
 * @u1pel: parameter from Set SEL request.
 * @num_eps: number of endpoints
 * @ep0_next_event: hold the next expected event
 * @ep0state: state of endpoint zero
 * @link_state: link state
 * @speed: device speed (super, high, full, low)
 * @hwparams: copy of hwparams registers
 * @regset: debugfs pointer to regdump file
 * @dbg_lsp_select: current debug lsp mux register selection
 * @test_mode: true when we're entering a USB test mode
 * @test_mode_nr: test feature selector
 * @lpm_nyet_threshold: LPM NYET response threshold
 * @hird_threshold: HIRD threshold
 * @rx_thr_num_pkt: USB receive packet count
 * @rx_max_burst: max USB receive burst size
 * @tx_thr_num_pkt: USB transmit packet count
 * @tx_max_burst: max USB transmit burst size
 * @rx_thr_num_pkt_prd: periodic ESS receive packet count
 * @rx_max_burst_prd: max periodic ESS receive burst size
 * @tx_thr_num_pkt_prd: periodic ESS transmit packet count
 * @tx_max_burst_prd: max periodic ESS transmit burst size
 * @tx_fifo_resize_max_num: max number of fifos allocated during txfifo resize
 * @clear_stall_protocol: endpoint number that requires a delayed status phase
 * @connected: true when we're connected to a host, false otherwise
 * @softconnect: true when gadget connect is called, false when disconnect runs
 * @delayed_status: true when gadget driver asks for delayed status
 * @ep0_bounced: true when we used bounce buffer
 * @ep0_expect_in: true when we expect a DATA IN transfer
 * @sysdev_is_parent: true when soc_usb device has a parent driver
 * @has_lpm_erratum: true when core was configured with LPM Erratum. Note that
 *			there's now way for software to detect this in runtime.
 * @is_utmi_l1_suspend: the core asserts output signal
 *	0	- utmi_sleep_n
 *	1	- utmi_l1_suspend_n
 * @is_fpga: true when we are using the FPGA board
 * @pending_events: true when we have pending IRQs to be handled
 * @do_fifo_resize: true when txfifo resizing is enabled for soc_usb endpoints
 * @pullups_connected: true when Run/Stop bit is set
 * @setup_packet_pending: true when there's a Setup Packet in FIFO. Workaround
 * @three_stage_setup: set if we perform a three phase setup
 * @dis_start_transfer_quirk: set if start_transfer failure SW workaround is
 *			not needed for SOC_USB version 1.70a-ea06 and below
 * @usb3_lpm_capable: set if hadrware supports Link Power Management
 * @usb2_lpm_disable: set to disable usb2 lpm for host
 * @usb2_gadget_lpm_disable: set to disable usb2 lpm for gadget
 * @disable_scramble_quirk: set if we enable the disable scramble quirk
 * @u2exit_lfps_quirk: set if we enable u2exit lfps quirk
 * @u2ss_inp3_quirk: set if we enable P3 OK for U2/SS Inactive quirk
 * @req_p1p2p3_quirk: set if we enable request p1p2p3 quirk
 * @del_p1p2p3_quirk: set if we enable delay p1p2p3 quirk
 * @del_phy_power_chg_quirk: set if we enable delay phy power change quirk
 * @dis_u3_susphy_quirk: set if we disable usb3 suspend phy
 * @dis_u2_susphy_quirk: set if we disable usb2 suspend phy
 * @dis_u1_entry_quirk: set if link entering into U1 state needs to be disabled.
 * @dis_u2_entry_quirk: set if link entering into U2 state needs to be disabled.
 * @dis_rxdet_inp3_quirk: set if we disable Rx.Detect in P3
 * @async_callbacks: if set, indicate that async callbacks will be used.
 * @dis_del_phy_power_chg_quirk: set if we disable delay phy power
 *			change quirk.
 * @dis_tx_ipgap_linecheck_quirk: set if we disable u2mac linestate
 *			check during HS transmit.
 * @resume_hs_terminations: Set if we enable quirk for fixing improper crc
 *			generation after resume from suspend.
 * @sys_wakeup: set if the device may do system wakeup.
 * @wakeup_configured: set if the device is configured for remote wakeup.
 * @suspended: set to track suspend event due to U3/L2.
 * @imod_interval: set the interrupt moderation interval in 250ns
 *			increments or 0 to disable.
 * @max_cfg_eps: current max number of IN eps used across all USB configs.
 * @last_fifo_depth: last fifo depth used to determine next fifo ram start
 *		     address.
 * @num_ep_resized: carries the current number endpoints which have had its tx
 *		    fifo resized.
 * @debug_root: root debugfs directory for this device to put its files in.
 */
struct soc_usb {
	struct work_struct	drd_work;
	struct soc_usb_trb		*ep0_trb;
	void			*bounce;
	u8			*setup_buf;
	dma_addr_t		ep0_trb_addr;
	dma_addr_t		bounce_addr;
	struct soc_usb_request	ep0_usb_req;
	struct completion	ep0_in_setup;

	/* device lock */
	spinlock_t		lock;

	/* mode switching lock */
	struct mutex		mutex;

	struct device		*dev;
	struct device		*sysdev;

	struct platform_device	*xhci;
	struct resource		xhci_resources[SOC_USB_XHCI_RESOURCES_NUM];

	struct soc_usb_event_buffer *ev_buf;
	struct soc_usb_ep		*eps[SOC_USB_ENDPOINTS_NUM];

	struct usb_gadget	*gadget;
	struct usb_gadget_driver *gadget_driver;

	struct clk		*bus_clk;
	struct clk		*ref_clk;
	struct clk		*susp_clk;

	struct reset_control	*reset;

	struct usb_phy		*usb2_phy;
	struct usb_phy		*usb3_phy;

	struct phy		*usb2_generic_phy;
	struct phy		*usb3_generic_phy;

	bool			phys_ready;

	struct ulpi		*ulpi;
	bool			ulpi_ready;

	void __iomem		*regs;
	size_t			regs_size;

	enum usb_dr_mode	dr_mode;
	u32			current_dr_role;
	u32			desired_dr_role;
	struct extcon_dev	*edev;
	struct notifier_block	edev_nb;
	enum usb_phy_interface	hsphy_mode;
	struct usb_role_switch	*role_sw;
	enum usb_dr_mode	role_switch_default_mode;

	struct power_supply	*usb_psy;

	u32			irq_gadget;
	u32			otg_irq;
	u32			current_otg_role;
	u32			desired_otg_role;
	bool			otg_restart_host;
	u32			u1u2;
	u32			maximum_speed;
	u32			gadget_max_speed;
	enum usb_ssp_rate	max_ssp_rate;
	enum usb_ssp_rate	gadget_ssp_rate;

	u32			ip;

	u32			revision;
	u32			version_type;

	enum soc_usb_ep0_next	ep0_next_event;
	enum soc_usb_ep0_state	ep0state;
	enum soc_usb_link_state	link_state;

	u16			u2sel;
	u16			u2pel;
	u8			u1sel;
	u8			u1pel;

	u8			speed;

	u8			num_eps;

	struct soc_usb_hwparams	hwparams;
	struct debugfs_regset32	*regset;

	u32			dbg_lsp_select;

	u8			test_mode;
	u8			test_mode_nr;
	u8			lpm_nyet_threshold;
	u8			hird_threshold;
	u8			rx_thr_num_pkt;
	u8			rx_max_burst;
	u8			tx_thr_num_pkt;
	u8			tx_max_burst;
	u8			rx_thr_num_pkt_prd;
	u8			rx_max_burst_prd;
	u8			tx_thr_num_pkt_prd;
	u8			tx_max_burst_prd;
	u8			tx_fifo_resize_max_num;
	u8			clear_stall_protocol;

	unsigned		connected:1;
	unsigned		softconnect:1;
	unsigned		delayed_status:1;
	unsigned		ep0_bounced:1;
	unsigned		ep0_expect_in:1;
	unsigned		sysdev_is_parent:1;
	unsigned		has_lpm_erratum:1;
	unsigned		is_utmi_l1_suspend:1;
	unsigned		is_fpga:1;
	unsigned		pending_events:1;
	unsigned		do_fifo_resize:1;
	unsigned		pullups_connected:1;
	unsigned		setup_packet_pending:1;
	unsigned		three_stage_setup:1;
	unsigned		dis_start_transfer_quirk:1;
	unsigned		usb3_lpm_capable:1;
	unsigned		usb2_lpm_disable:1;
	unsigned		usb2_gadget_lpm_disable:1;

	unsigned		disable_scramble_quirk:1;
	unsigned		u2exit_lfps_quirk:1;
	unsigned		u2ss_inp3_quirk:1;
	unsigned		req_p1p2p3_quirk:1;
	unsigned		del_p1p2p3_quirk:1;
	unsigned		del_phy_power_chg_quirk:1;
	unsigned		lfps_filter_quirk:1;
	unsigned		rx_detect_poll_quirk:1;

	unsigned		dis_u3_susphy_quirk:1;
	unsigned		dis_u2_susphy_quirk:1;
	unsigned		dis_u1_entry_quirk:1;
	unsigned		dis_u2_entry_quirk:1;
	unsigned		dis_rxdet_inp3_quirk:1;
	unsigned		dis_del_phy_power_chg_quirk:1;
	unsigned		dis_tx_ipgap_linecheck_quirk:1;
	unsigned		resume_hs_terminations:1;
	unsigned		async_callbacks:1;
	unsigned		sys_wakeup:1;
	unsigned		wakeup_configured:1;
	unsigned		suspended:1;

	u16			imod_interval;

	int			max_cfg_eps;
	int			last_fifo_depth;
	int			num_ep_resized;
	struct dentry		*debug_root;
};

#define INCRX_BURST_MODE 0
#define INCRX_UNDEF_LENGTH_BURST_MODE 1

#define work_to_su(w)		(container_of((w), struct soc_usb, drd_work))

/* -------------------------------------------------------------------------- */

struct soc_usb_event_type {
	u32	is_devspec:1;
	u32	type:7;
	u32	reserved8_31:24;
} __packed;

#define SOC_USB_DEPEVT_XFERCOMPLETE	0x01
#define SOC_USB_DEPEVT_XFERINPROGRESS	0x02
#define SOC_USB_DEPEVT_XFERNOTREADY	0x03
#define SOC_USB_DEPEVT_RXTXFIFOEVT		0x04
#define SOC_USB_DEPEVT_STREAMEVT		0x06
#define SOC_USB_DEPEVT_EPCMDCMPLT		0x07

/**
 * struct soc_usb_event_depevt - Device Endpoint Events
 * @one_bit: indicates this is an endpoint event (not used)
 * @endpoint_number: number of the endpoint
 * @endpoint_event: The event we have:
 *	0x00	- Reserved
 *	0x01	- XferComplete
 *	0x02	- XferInProgress
 *	0x03	- XferNotReady
 *	0x04	- RxTxFifoEvt (IN->Underrun, OUT->Overrun)
 *	0x05	- Reserved
 *	0x06	- StreamEvt
 *	0x07	- EPCmdCmplt
 * @reserved11_10: Reserved, don't use.
 * @status: Indicates the status of the event. Refer to databook for
 *	more information.
 * @parameters: Parameters of the current event. Refer to databook for
 *	more information.
 */
struct soc_usb_event_depevt {
	u32	one_bit:1;
	u32	endpoint_number:5;
	u32	endpoint_event:4;
	u32	reserved11_10:2;
	u32	status:4;

/* Within XferNotReady */
#define DEPEVT_STATUS_TRANSFER_ACTIVE	BIT(3)

/* Within XferComplete or XferInProgress */
#define DEPEVT_STATUS_BUSERR	BIT(0)
#define DEPEVT_STATUS_SHORT	BIT(1)
#define DEPEVT_STATUS_IOC	BIT(2)
#define DEPEVT_STATUS_LST	BIT(3) /* XferComplete */
#define DEPEVT_STATUS_MISSED_ISOC BIT(3) /* XferInProgress */

/* Stream event only */
#define DEPEVT_STREAMEVT_FOUND		1
#define DEPEVT_STREAMEVT_NOTFOUND	2

/* Stream event parameter */
#define DEPEVT_STREAM_PRIME		0xfffe
#define DEPEVT_STREAM_NOSTREAM		0x0

/* Control-only Status */
#define DEPEVT_STATUS_CONTROL_DATA	1
#define DEPEVT_STATUS_CONTROL_STATUS	2
#define DEPEVT_STATUS_CONTROL_PHASE(n)	((n) & 3)

/* In response to Start Transfer */
#define DEPEVT_TRANSFER_NO_RESOURCE	1
#define DEPEVT_TRANSFER_BUS_EXPIRY	2

	u32	parameters:16;

/* For Command Complete Events */
#define DEPEVT_PARAMETER_CMD(n)	(((n) & (0xf << 8)) >> 8)
} __packed;

/**
 * struct soc_usb_event_devt - Device Events
 * @one_bit: indicates this is a non-endpoint event (not used)
 * @device_event: indicates it's a device event. Should read as 0x00
 * @type: indicates the type of device event.
 *	0	- DisconnEvt
 *	1	- USBRst
 *	2	- ConnectDone
 *	3	- ULStChng
 *	4	- WkUpEvt
 *	5	- Reserved
 *	6	- Suspend (EOPF on revisions 2.10a and prior)
 *	7	- SOF
 *	8	- Reserved
 *	9	- ErrticErr
 *	10	- CmdCmplt
 *	11	- EvntOverflow
 *	12	- VndrDevTstRcved
 * @reserved15_12: Reserved, not used
 * @event_info: Information about this event
 * @reserved31_25: Reserved, not used
 */
struct soc_usb_event_devt {
	u32	one_bit:1;
	u32	device_event:7;
	u32	type:4;
	u32	reserved15_12:4;
	u32	event_info:9;
	u32	reserved31_25:7;
} __packed;

/**
 * struct soc_usb_event_gevt - Other Core Events
 * @one_bit: indicates this is a non-endpoint event (not used)
 * @device_event: indicates it's (0x03) Carkit or (0x04) I2C event.
 * @phy_port_number: self-explanatory
 * @reserved31_12: Reserved, not used.
 */
struct soc_usb_event_gevt {
	u32	one_bit:1;
	u32	device_event:7;
	u32	phy_port_number:4;
	u32	reserved31_12:20;
} __packed;

/**
 * union soc_usb_event - representation of Event Buffer contents
 * @raw: raw 32-bit event
 * @type: the type of the event
 * @depevt: Device Endpoint Event
 * @devt: Device Event
 * @gevt: Global Event
 */
union soc_usb_event {
	u32				raw;
	struct soc_usb_event_type		type;
	struct soc_usb_event_depevt	depevt;
	struct soc_usb_event_devt		devt;
	struct soc_usb_event_gevt		gevt;
};

/**
 * struct soc_usb_gadget_ep_cmd_params - representation of endpoint command
 * parameters
 * @param2: third parameter
 * @param1: second parameter
 * @param0: first parameter
 */
struct soc_usb_gadget_ep_cmd_params {
	u32	param2;
	u32	param1;
	u32	param0;
};

/*
 * SOC_USB Features to be used as Driver Data
 */

#define SOC_USB_HAS_PERIPHERAL		BIT(0)
#define SOC_USB_HAS_XHCI			BIT(1)
#define SOC_USB_HAS_OTG			BIT(3)

/* prototypes */
void soc_usb_set_prtcap(struct soc_usb *su, u32 mode);
void soc_usb_set_mode(struct soc_usb *su, u32 mode);
u32 soc_usb_core_fifo_space(struct soc_usb_ep *dep, u8 type);

#define SOC_USB_IP_IS(_ip)							\
	(su->ip == _ip##_IP)
/**
 * soc_usb_mdwidth - get MDWIDTH value in bits
 * @su: pointer to our context structure
 *
 * Return MDWIDTH configuration value in bits.
 */
static inline u32 soc_usb_mdwidth(struct soc_usb *su)
{
	u32 mdwidth;

	mdwidth = SOC_USB_GHWPARAMS0_MDWIDTH(su->hwparams.hwparams0);
    if (SOC_USB_IP_IS(SOC_USB32)) {
        mdwidth += SOC_USB_GHWPARAMS6_MDWIDTH(su->hwparams.hwparams6);
    }
	return mdwidth;
}

bool soc_usb_has_imod(struct soc_usb *su);

int soc_usb_event_buffers_setup(struct soc_usb *su);
void soc_usb_event_buffers_cleanup(struct soc_usb *su);

int soc_usb_core_soft_reset(struct soc_usb *su);
void soc_usb_enable_susphy(struct soc_usb *su, bool enable);

#if IS_ENABLED(CONFIG_USB_SOC_DRD_HOST) || IS_ENABLED(CONFIG_USB_SOC_DRD_DUAL_ROLE)
int soc_usb_host_init(struct soc_usb *su);
void soc_usb_host_exit(struct soc_usb *su);
#else
static inline int soc_usb_host_init(struct soc_usb *su)
{ return 0; }
static inline void soc_usb_host_exit(struct soc_usb *su)
{ }
#endif

#if IS_ENABLED(CONFIG_USB_SOC_DRD_GADGET) || IS_ENABLED(CONFIG_USB_SOC_DRD_DUAL_ROLE)
int soc_usb_gadget_init(struct soc_usb *su);
void soc_usb_gadget_exit(struct soc_usb *su);
int soc_usb_gadget_set_test_mode(struct soc_usb *su, int mode);
int soc_usb_gadget_get_link_state(struct soc_usb *su);
int soc_usb_gadget_set_link_state(struct soc_usb *su, enum soc_usb_link_state state);
int soc_usb_send_gadget_ep_cmd(struct soc_usb_ep *dep, unsigned int cmd,
		struct soc_usb_gadget_ep_cmd_params *params);
int soc_usb_send_gadget_generic_command(struct soc_usb *su, unsigned int cmd,
		u32 param);
void soc_usb_gadget_clear_tx_fifos(struct soc_usb *su);
void soc_usb_remove_requests(struct soc_usb *su, struct soc_usb_ep *dep, int status);
#else
static inline int soc_usb_gadget_init(struct soc_usb *su)
{ return 0; }
static inline void soc_usb_gadget_exit(struct soc_usb *su)
{ }
static inline int soc_usb_gadget_set_test_mode(struct soc_usb *su, int mode)
{ return 0; }
static inline int soc_usb_gadget_get_link_state(struct soc_usb *su)
{ return 0; }
static inline int soc_usb_gadget_set_link_state(struct soc_usb *su,
		enum soc_usb_link_state state)
{ return 0; }

static inline int soc_usb_send_gadget_ep_cmd(struct soc_usb_ep *dep, unsigned int cmd,
		struct soc_usb_gadget_ep_cmd_params *params)
{ return 0; }
static inline int soc_usb_send_gadget_generic_command(struct soc_usb *su,
		int cmd, u32 param)
{ return 0; }
static inline void soc_usb_gadget_clear_tx_fifos(struct soc_usb *su)
{ }
#endif

/*
 * not supported
 */
static inline int soc_usb_drd_init(struct soc_usb *su)
{ return 0; }
static inline void soc_usb_drd_exit(struct soc_usb *su)
{ }
static inline void soc_usb_otg_init(struct soc_usb *su)
{ }
static inline void soc_usb_otg_exit(struct soc_usb *su)
{ }
static inline void soc_usb_otg_update(struct soc_usb *su, bool ignore_idstatus)
{ }
static inline void soc_usb_otg_host_init(struct soc_usb *su)
{ }

/* power management interface */
#if IS_ENABLED(CONFIG_USB_SOC_DRD_GADGET) || IS_ENABLED(CONFIG_USB_SOC_DRD_DUAL_ROLE)
int soc_usb_gadget_suspend(struct soc_usb *su);
int soc_usb_gadget_resume(struct soc_usb *su);
void soc_usb_gadget_process_pending_events(struct soc_usb *su);
#else
static inline int soc_usb_gadget_suspend(struct soc_usb *su)
{
	return 0;
}

static inline int soc_usb_gadget_resume(struct soc_usb *su)
{
	return 0;
}

static inline void soc_usb_gadget_process_pending_events(struct soc_usb *su)
{
}
#endif /* GADGET || DUAL_ROLE */

#if IS_ENABLED(CONFIG_USB_SOC_DRD_ULPI)
int soc_usb_ulpi_init(struct soc_usb *su);
void soc_usb_ulpi_exit(struct soc_usb *su);
#else
static inline int soc_usb_ulpi_init(struct soc_usb *su)
{ return 0; }
static inline void soc_usb_ulpi_exit(struct soc_usb *su)
{ }
#endif

#endif /* __DRIVERS_USB_SOC_DRD_CORE_H */
