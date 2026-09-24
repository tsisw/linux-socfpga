/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TSI SkyLP integration for the soc_drd USB controller — the init
 * sequence the vendor left as an empty stub (gap G2), split into a
 * pure core so it is KUnit-testable without the USB stack.
 *
 * Fully-known facts encoded here (Confluence gap register §6c/§8b):
 *   - the Type-C lane mux lives at APB offset 0x115c4 (SkyLP
 *     0x2402_1714) and RESETS TO "NO CONNECTION"; bits 12/13 are the
 *     mode/flip bypass enables, bits [15:14] the mode, bit 16 the flip.
 *     Without this write no USB3 link can ever train.
 *   - >= 100 us must elapse after por_n release before operation
 *     (databook §5.2, Table 24).
 *
 * Deliberately DT-driven (values pending databook section mapping,
 * HW-8/HW-9): the TSAR block-control and UDI refclk-select values.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#ifndef __SOC_SKYLP_H
#define __SOC_SKYLP_H

#include <linux/types.h>

/* Lane-mux field encoding, APB reg 0x115c4 (databook §4.3, Tables 22/23). */
#define TSI_SKYLP_MUX_CTL_BYPASS	BIT(12)
#define TSI_SKYLP_MUX_FLIP_BYPASS	BIT(13)
#define TSI_SKYLP_MUX_MODE_SHIFT	14
#define TSI_SKYLP_MUX_MODE_MASK		GENMASK(15, 14)
#define TSI_SKYLP_MUX_FLIP		BIT(16)

#define TSI_SKYLP_MUX_MODE_NONE		0	/* reset: no connection */
#define TSI_SKYLP_MUX_MODE_USB		1	/* USB3.2 Gen2x2 */
#define TSI_SKYLP_MUX_MODE_USB_2DP	2	/* USB3.2 + 2-lane DP */
#define TSI_SKYLP_MUX_MODE_4DP		3	/* 4-lane DP, no USB3 */

#define TSI_SKYLP_POR_SETTLE_US		100	/* databook §5.2 minimum */

/* Register slots the sequence may touch, resolved by the binder. */
enum tsi_skylp_reg {
	TSI_SKYLP_REG_TSAR,	/* gpp tsar_control */
	TSI_SKYLP_REG_CLKSEL,	/* gpp udi_clk_sel */
	TSI_SKYLP_REG_MUX,	/* apb Type-C lane mux */
};

struct tsi_skylp_hw {
	void (*wr)(void *ctx, enum tsi_skylp_reg reg, u32 val);
	void (*delay_us)(void *ctx, unsigned int us);
	void *ctx;
};

struct tsi_skylp_init {
	bool	has_tsar;
	u32	tsar_val;	/* DT tsi,tsar-init (HW-8 pending) */
	bool	has_clksel;
	u32	clksel_val;	/* DT tsi,clksel-init (HW-4/27 pending) */
	u32	mux_mode;	/* TSI_SKYLP_MUX_MODE_* */
	bool	mux_flip;
};

u32 tsi_skylp_mux_val(u32 mode, bool flip);
int tsi_skylp_usb_init_seq(const struct tsi_skylp_hw *hw,
			   const struct tsi_skylp_init *init);

struct device;

/*
 * Probe-time binder for soc_usb: parses the "tsi,skylp-usb" node, maps
 * the named windows and runs the sequence. A node without that
 * compatible returns 0 untouched (vendor/legacy platforms).
 */
#if IS_REACHABLE(CONFIG_USB_SOC_DRD_SKYLP)
int tsi_skylp_usb_init(struct device *dev);
#else
static inline int tsi_skylp_usb_init(struct device *dev)
{
	return 0;
}
#endif

#endif /* __SOC_SKYLP_H */
