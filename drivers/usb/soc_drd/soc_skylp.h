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
 * Deliberately DT-driven: the TSAR block-control value (still pending,
 * HW-8/HW-9) and the UDI refclk-select value (bit layout confirmed by
 * the hardware team on 2026-10-07 - see TSI_SKYLP_CLKSEL_*; which bits
 * a board sets is still its own choice). The UDI reset-release window
 * (udi_reset_cfg, bit map confirmed the same day) is optional too:
 * absent when firmware releases the resets before Linux boots.
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

/*
 * gpp udi_clk_sel fields (hardware team, 2026-10-07):
 * {29'h0, div2_clken, refclk100m_sel, refclk_sel}. div2_clken divides
 * the 50 MHz FREF by two for the controller's ref_clk pin, which
 * Innosilicon spec at 24 MHz and confirmed working at 25 MHz.
 */
#define TSI_SKYLP_CLKSEL_REFCLK_SEL	BIT(0)	/* 0 single-ended, 1 differential */
#define TSI_SKYLP_CLKSEL_REFCLK100M_SEL	BIT(1)	/* 0 50M FREF, 1 100M FREF_100M */
#define TSI_SKYLP_CLKSEL_DIV2_CLKEN	BIT(2)	/* ref_clk = FREF / 2; RESETS TO 1 */

/*
 * gpp udi_reset_cfg (hardware team, 2026-10-07): active-low reset
 * releases, set = out of reset. Layout
 * {22'h0, aud_axi_aresetn_1, aud_axi_aresetn_0, aud_rst_n,
 *  vid_axi_aresetn_1, vid_axi_aresetn_0, dptx_sys_rstn,
 *  usb_axi_aresetn, apb_presetn, ahb_hresetn, por_n}.
 */
#define TSI_SKYLP_RST_POR_N		BIT(0)
#define TSI_SKYLP_RST_AHB_HRESETN	BIT(1)
#define TSI_SKYLP_RST_APB_PRESETN	BIT(2)
#define TSI_SKYLP_RST_USB_AXI_ARESETN	BIT(3)
#define TSI_SKYLP_RST_DPTX_SYS_RSTN	BIT(4)
#define TSI_SKYLP_RST_VID_AXI_ARESETN_0	BIT(5)
#define TSI_SKYLP_RST_VID_AXI_ARESETN_1	BIT(6)
#define TSI_SKYLP_RST_AUD_RST_N		BIT(7)
#define TSI_SKYLP_RST_AUD_AXI_ARESETN_0	BIT(8)
#define TSI_SKYLP_RST_AUD_AXI_ARESETN_1	BIT(9)

/* What USB bring-up releases; dptx/vid/aud belong to their own drivers. */
#define TSI_SKYLP_RST_USB_SET		(TSI_SKYLP_RST_AHB_HRESETN |	\
					 TSI_SKYLP_RST_APB_PRESETN |	\
					 TSI_SKYLP_RST_USB_AXI_ARESETN)

/* Register slots the sequence may touch, resolved by the binder. */
enum tsi_skylp_reg {
	TSI_SKYLP_REG_TSAR,	/* gpp tsar_control */
	TSI_SKYLP_REG_CLKSEL,	/* gpp udi_clk_sel */
	TSI_SKYLP_REG_RESET,	/* gpp udi_reset_cfg */
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
 * The UDI reference is a 50 MHz FREF (HW-5); the controller's ref_clk
 * pin sees that directly, or 25 MHz when udi_clk_sel.div2_clken
 * divides it (the Innosilicon pin spec is 24 MHz, confirmed working at
 * 25 MHz, 2026-10-07). The RTL defaults assume 24 MHz
 * (GUCTL.REFCLKPER = 41 ns) and ~25 MHz (GCTL.PWRDNSCALE = 1560).
 * Field positions are those of the DWC3 layout the vendor's own GCTL
 * definitions follow, and match the reset values the hardware team
 * quoted.
 */
#define TSI_SKYLP_USB_FREF_HZ		50000000UL
#define TSI_SKYLP_USB_FREF100_HZ	100000000UL
#define TSI_SKYLP_GCTL_PWRDNSCALE	GENMASK(31, 19)	/* ref_clk / 16 kHz */
#define TSI_SKYLP_GUCTL_REFCLKPER	GENMASK(31, 22)	/* ref_clk period, ns */

struct tsi_skylp_init {
	bool	has_tsar;
	u32	tsar_val;	/* DT tsi,tsar-init (HW-8 pending) */
	bool	has_reset;	/* tsi-reset window mapped by the binder */
	bool	has_clksel;
	u32	clksel_val;	/* DT tsi,clksel-init (TSI_SKYLP_CLKSEL_*) */
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
 * Release the UDI resets USB needs: por_n, the validated >= 100 us
 * settle, then AHB/APB/USB-AXI in one read-modify-write that leaves
 * the dptx/vid/aud releases alone. Bits already set are not rewritten,
 * so a firmware-initialised part sees no writes and no delay. Needs
 * hw->rd for TSI_SKYLP_REG_RESET.
 */
void tsi_skylp_usb_reset_seq(const struct tsi_skylp_hw *hw);
/*
 * The controller ref_clk rate the board's clksel choice produces: the
 * base is FREF, or FREF_100M when an applied clksel sets
 * refclk100m_sel; div2_clken (which resets to 1, HW-4) then halves it.
 * No applied clksel means the reset value {0, 0, 1}: 25 MHz.
 */
unsigned long tsi_skylp_usb_ref_clk_hz(const struct tsi_skylp_init *init);
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
