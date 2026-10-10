/* SPDX-License-Identifier: GPL-2.0 */
/*
 * soc_io.h - SOC USB3 DRD IO Header
 *
 */

#ifndef __DRIVERS_USB_SOC_DRD_IO_H
#define __DRIVERS_USB_SOC_DRD_IO_H

#include <linux/io.h>
#include "soc_trace.h"
#include "soc_debug.h"
#include "soc_core.h"

static inline u32 soc_usb_readl(void __iomem *base, u32 offset)
{
	u32 value;

	/*
	 * We requested the mem region starting from the Globals address
	 * space, see soc_usb_probe in core.c.
	 * However, the offsets are given starting from xHCI address space.
	 */
	value = readl(base + offset - SOC_USB_GLOBALS_REGS_START);

	/*
	 * When tracing we want to make it easy to find the correct address on
	 * documentation, so we revert it back to the proper addresses.
	 */
	trace_soc_usb_readl(base - SOC_USB_GLOBALS_REGS_START, offset, value);

	return value;
}

static inline void soc_usb_writel(void __iomem *base, u32 offset, u32 value)
{
	/*
	 * We requested the mem region starting from the Globals address
	 * space, see soc_usb_probe in core.c.
	 * However, the offsets are given starting from xHCI address space.
	 */
	writel(value, base + offset - SOC_USB_GLOBALS_REGS_START);

	/*
	 * When tracing we want to make it easy to find the correct address on
	 * documentation, so we revert it back to the proper addresses.
	 */
	trace_soc_usb_writel(base - SOC_USB_GLOBALS_REGS_START, offset, value);
}

#endif /* __DRIVERS_USB_SOC_DRD_IO_H */
