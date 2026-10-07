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
	TSI_SKYLP_REG_GCTL,	/* controller GCTL (power-down scale) */
	TSI_SKYLP_REG_GUCTL,	/* controller GUCTL (refclk period) */
	TSI_SKYLP_REG_COUNT,
};

struct tsi_skylp_hw {
	u32 (*rd)(void *ctx, enum tsi_skylp_reg reg);	/* GCTL/GUCTL only */
	void (*wr)(void *ctx, enum tsi_skylp_reg reg, u32 val);
	void (*delay_us)(void *ctx, unsigned int us);
	void *ctx;
};

/*
 * The controller's ref_clk and suspend_clk are 50 MHz on SkyLP (HW-5),
 * while the RTL defaults assume 24 MHz (GUCTL.REFCLKPER = 41 ns) and
 * ~25 MHz (GCTL.PWRDNSCALE = 1560). Field positions are those of the
 * DWC3 layout the vendor's own GCTL definitions follow, and match the
 * reset values the hardware team quoted.
 */
#define TSI_SKYLP_USB_REF_CLK_HZ	50000000UL
#define TSI_SKYLP_GCTL_PWRDNSCALE	GENMASK(31, 19)	/* ref_clk / 16 kHz */
#define TSI_SKYLP_GUCTL_REFCLKPER	GENMASK(31, 22)	/* ref_clk period, ns */

struct tsi_skylp_init {
	bool	has_tsar;
	u32	tsar_val;	/* DT tsi,tsar-init (HW-8 pending) */
	bool	has_clksel;
	u32	clksel_val;	/* DT tsi,clksel-init (HW-4/27 pending) */
	u32	mux_mode;	/* TSI_SKYLP_MUX_MODE_* */
	bool	mux_flip;
};

/* Last-written mux value, so runtime updates skip redundant writes. */
struct tsi_skylp_mux_cache {
	u32	val;
	bool	valid;
};

#define TSI_SKYLP_MUX_CACHE_INIT	{ }

u32 tsi_skylp_mux_val(u32 mode, bool flip);
int tsi_skylp_usb_init_seq(const struct tsi_skylp_hw *hw,
			   const struct tsi_skylp_init *init);
/*
 * Program GUCTL.REFCLKPER and GCTL.PWRDNSCALE for @ref_clk_hz, keeping
 * every other bit of both registers. Runs after the controller's core
 * soft reset, where dwc3 does the same. -EINVAL for a zero rate, -ERANGE
 * when a field cannot hold the value.
 */
int tsi_skylp_usb_refclk_seq(const struct tsi_skylp_hw *hw,
			     unsigned long ref_clk_hz);
/*
 * Fill @init from a firmware node (DT, ACPI _DSD or software node). Returns
 * -ENODEV unless the node is compatible with tsi,skylp-usb, -EINVAL for an
 * out-of-range tsi,mux-mode.
 */
struct fwnode_handle;
int tsi_skylp_usb_parse(const struct fwnode_handle *fw, struct tsi_skylp_init *init);

bool tsi_skylp_mux_update(struct tsi_skylp_mux_cache *cache,
			  const struct tsi_skylp_hw *hw,
			  u32 mode, bool flip);

/*
 * Translate a Type-C mode-switch state (TYPEC_STATE_* / TYPEC_DP_STATE_*)
 * into a lane-mux mode, so a port manager can drive this mux without any
 * SkyLP-specific knowledge. Leaves *mux_mode untouched and returns
 * -EINVAL for states the mux cannot represent.
 */
int tsi_skylp_mux_mode_from_typec(unsigned long typec_mode, u32 *mux_mode);

/* What the PD controller's plug lines last told us. */
struct tsi_skylp_plug_state {
	bool	flip;		/* USB_PLUG_FLIP: reversed orientation */
	bool	present;	/* USB_PLUG_EVENT: a plug is attached */
};

#define TSI_SKYLP_PLUG_FLIP_CHANGED	BIT(0)
#define TSI_SKYLP_PLUG_PRESENCE_CHANGED	BIT(1)
#define TSI_SKYLP_PLUG_READ_FAILED	BIT(2)

/*
 * Fold one poll of the plug lines into @s. @flip_raw and @event_raw are
 * gpiod_get_value() results: 0 or 1, or a negative errno for a failed
 * read, which keeps that line's last state rather than being mistaken
 * for a set line. A line the board does not route is passed as its
 * current state. Returns TSI_SKYLP_PLUG_* bits for what changed.
 */
unsigned int tsi_skylp_plug_sync(struct tsi_skylp_plug_state *s,
				 int flip_raw, int event_raw);

struct device;

/*
 * Probe-time binder for soc_usb: parses the "tsi,skylp-usb" node, maps
 * the named windows and runs the sequence. A node without that
 * compatible returns 0 untouched (vendor/legacy platforms).
 */
#if IS_REACHABLE(CONFIG_USB_SOC_DRD_SKYLP)
int tsi_skylp_usb_init(struct device *dev);
/*
 * Post-reset hook for soc_usb's core init: on a "tsi,skylp-usb" node
 * runs tsi_skylp_usb_refclk_seq() over the controller's global registers
 * (@globals is soc_usb's regs pointer, the GLOBALS window). Other nodes
 * return 0 untouched.
 */
int tsi_skylp_usb_core_init(struct device *dev, void __iomem *globals);
#else
static inline int tsi_skylp_usb_init(struct device *dev)
{
	return 0;
}

static inline int tsi_skylp_usb_core_init(struct device *dev,
					  void __iomem *globals)
{
	return 0;
}
#endif

#endif /* __SOC_SKYLP_H */
