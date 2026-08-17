/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI chiplet core — API for TSI IP drivers (gpio, pvt, mbox, i2c, wdt).
 *
 * The chiplet core owns the CSR window of one chiplet and hands out a
 * regmap plus poll helpers. IP drivers never ioremap or readl/writel
 * themselves; everything goes through the core so chiplet addressing,
 * quirks, and a future IRQ path stay in one place.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#ifndef __LINUX_SOC_TSI_CHIPLET_H
#define __LINUX_SOC_TSI_CHIPLET_H

#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/types.h>

struct device;

struct tsi_chiplet {
	struct device	*dev;
	struct regmap	*regmap;
	u32		chiplet_id;
};

#if IS_REACHABLE(CONFIG_TSI_CHIPLET)

/*
 * Resolve the chiplet core from an IP driver's device (the IP node is a
 * child of the chiplet node in DT). Returns ERR_PTR(-EPROBE_DEFER) until
 * the core has probed.
 */
struct tsi_chiplet *tsi_chiplet_get(struct device *child);

/*
 * Poll a register field until (val & mask) == expect or timeout.
 *
 * No IRQ path exists through skylp-mmio-dev today, so completion is
 * always polled. IP drivers must use this helper (not open-coded loops)
 * so an IRQ mode can replace the implementation later without touching
 * them.
 */
int tsi_chiplet_poll(struct tsi_chiplet *chiplet, u32 reg, u32 mask,
		     u32 expect, u32 timeout_us);

#else /* !TSI_CHIPLET: keep COMPILE_TEST/KUnit consumers linkable */

static inline struct tsi_chiplet *tsi_chiplet_get(struct device *child)
{
	return ERR_PTR(-ENODEV);
}

static inline int tsi_chiplet_poll(struct tsi_chiplet *chiplet, u32 reg,
				   u32 mask, u32 expect, u32 timeout_us)
{
	return -ENODEV;
}

#endif /* IS_REACHABLE(CONFIG_TSI_CHIPLET) */

#endif /* __LINUX_SOC_TSI_CHIPLET_H */
