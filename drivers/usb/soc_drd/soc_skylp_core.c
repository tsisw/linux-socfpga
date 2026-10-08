// SPDX-License-Identifier: GPL-2.0
/*
 * TSI SkyLP USB init-sequence core (gap G2).
 *
 * Pure sequence logic over injected callbacks — no MMIO, no USB stack —
 * so it is KUnit-testable on UML and reusable by both the soc_drd
 * binder (probe-time init) and the Type-C glue (runtime flip updates).
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>

#include "soc_core.h"		/* GLOBALS window offsets of GCTL/GUCTL */
#include "soc_skylp.h"

u32 tsi_skylp_mux_val(u32 mode, bool flip)
{
	u32 val = TSI_SKYLP_MUX_CTL_BYPASS | TSI_SKYLP_MUX_FLIP_BYPASS;

	val |= (mode << TSI_SKYLP_MUX_MODE_SHIFT) & TSI_SKYLP_MUX_MODE_MASK;
	if (flip)
		val |= TSI_SKYLP_MUX_FLIP;
	return val;
}
EXPORT_SYMBOL_GPL(tsi_skylp_mux_val);

int tsi_skylp_usb_init_seq(const struct tsi_skylp_hw *hw,
			   const struct tsi_skylp_init *init)
{
	if (init->mux_mode & ~(TSI_SKYLP_MUX_MODE_MASK >>
			       TSI_SKYLP_MUX_MODE_SHIFT))
		return -EINVAL;

	if (init->has_tsar) {
		hw->wr(hw->ctx, TSI_SKYLP_REG_TSAR, init->tsar_val);
		hw->delay_us(hw->ctx, TSI_SKYLP_POR_SETTLE_US);
	}
	/* Resets before the mux: its APB word sits behind apb_presetn. */
	if (init->has_reset)
		tsi_skylp_usb_reset_seq(hw);
	if (init->has_clksel)
		hw->wr(hw->ctx, TSI_SKYLP_REG_CLKSEL, init->clksel_val);

	/* Always last, and never skipped: the mux resets to "no connection". */
	hw->wr(hw->ctx, TSI_SKYLP_REG_MUX,
	       tsi_skylp_mux_val(init->mux_mode, init->mux_flip));
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_init_seq);

/* Read-modify-write one field; skip the write when it already holds @val. */
static void tsi_skylp_set_field(const struct tsi_skylp_hw *hw,
				enum tsi_skylp_reg reg, u32 mask, u32 val)
{
	u32 cur = hw->rd(hw->ctx, reg);
	u32 want = (cur & ~mask) | (val & mask);

	if (want != cur)
		hw->wr(hw->ctx, reg, want);
}

int tsi_skylp_usb_refclk_seq(const struct tsi_skylp_hw *hw,
			     unsigned long ref_clk_hz)
{
	unsigned long period_ns, scale;

	if (!ref_clk_hz)
		return -EINVAL;

	period_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, ref_clk_hz);
	scale = ref_clk_hz / 16000;	/* PWRDNSCALE: ref_clk in 16 kHz units */
	if (!period_ns || period_ns > FIELD_MAX(TSI_SKYLP_GUCTL_REFCLKPER) ||
	    !scale || scale > FIELD_MAX(TSI_SKYLP_GCTL_PWRDNSCALE))
		return -ERANGE;

	tsi_skylp_set_field(hw, TSI_SKYLP_REG_GUCTL, TSI_SKYLP_GUCTL_REFCLKPER,
			    FIELD_PREP(TSI_SKYLP_GUCTL_REFCLKPER, period_ns));
	tsi_skylp_set_field(hw, TSI_SKYLP_REG_GCTL, TSI_SKYLP_GCTL_PWRDNSCALE,
			    FIELD_PREP(TSI_SKYLP_GCTL_PWRDNSCALE, scale));
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_refclk_seq);

/*
 * Release the UDI resets USB needs (udi_reset_cfg, bit map from the
 * hardware team 2026-10-07): por_n first with the validated >= 100 us
 * settle after it, then ahb_hresetn/apb_presetn/usb_axi_aresetn in one
 * write. Read-modify-write throughout, so the dptx/vid/aud releases
 * owned by the display and audio drivers are never disturbed, and bits
 * already out of reset are not rewritten - a firmware-initialised part
 * sees no writes and no settle delay.
 */
void tsi_skylp_usb_reset_seq(const struct tsi_skylp_hw *hw)
{
	u32 cur = hw->rd(hw->ctx, TSI_SKYLP_REG_RESET);

	if (!(cur & TSI_SKYLP_RST_POR_N)) {
		cur |= TSI_SKYLP_RST_POR_N;
		hw->wr(hw->ctx, TSI_SKYLP_REG_RESET, cur);
		hw->delay_us(hw->ctx, TSI_SKYLP_POR_SETTLE_US);
	}
	if ((cur | TSI_SKYLP_RST_USB_SET) != cur)
		hw->wr(hw->ctx, TSI_SKYLP_REG_RESET,
		       cur | TSI_SKYLP_RST_USB_SET);
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_reset_seq);

/*
 * What the controller's ref_clk pin will run at once the sequence has
 * applied the board's clksel choice. div2_clken RESETS TO 1 (udi_clk_sel
 * reset {0, 0, 1}, HW-4), so the divided 25 MHz is the default; only an
 * explicit clksel write that clears the bit selects the full 50 MHz
 * FREF. A clksel the sequence will not write (no window or property)
 * leaves the divider at that reset state.
 */
unsigned long tsi_skylp_usb_ref_clk_hz(const struct tsi_skylp_init *init)
{
	/* No applied clksel leaves the reset value {0, 0, 1}. */
	u32 sel = init->has_clksel ? init->clksel_val :
				     TSI_SKYLP_CLKSEL_DIV2_CLKEN;
	unsigned long fref = (sel & TSI_SKYLP_CLKSEL_REFCLK100M_SEL) ?
			     TSI_SKYLP_USB_FREF100_HZ : TSI_SKYLP_USB_FREF_HZ;

	return (sel & TSI_SKYLP_CLKSEL_DIV2_CLKEN) ? fref / 2 : fref;
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_ref_clk_hz);

/*
 * Runtime mux update (plug/flip interrupt path): write only when the
 * value actually changes, so servicing an event never glitches the
 * lane mux under an established link. Returns true when written.
 */
bool tsi_skylp_mux_update(struct tsi_skylp_mux_cache *cache,
			  const struct tsi_skylp_hw *hw,
			  u32 mode, bool flip)
{
	u32 val = tsi_skylp_mux_val(mode, flip);

	if (cache->valid && cache->val == val)
		return false;

	hw->wr(hw->ctx, TSI_SKYLP_REG_MUX, val);
	cache->val = val;
	cache->valid = true;
	return true;
}
EXPORT_SYMBOL_GPL(tsi_skylp_mux_update);

int tsi_skylp_mux_mode_from_typec(unsigned long typec_mode, u32 *mux_mode)
{
	switch (typec_mode) {
	case TYPEC_STATE_SAFE:
		*mux_mode = TSI_SKYLP_MUX_MODE_NONE;
		return 0;
	case TYPEC_STATE_USB:
		*mux_mode = TSI_SKYLP_MUX_MODE_USB;
		return 0;
	/* Pin assignments C and E carry four DP lanes, no USB3. */
	case TYPEC_DP_STATE_C:
	case TYPEC_DP_STATE_E:
		*mux_mode = TSI_SKYLP_MUX_MODE_4DP;
		return 0;
	/* D keeps USB3 on one pair alongside two DP lanes. */
	case TYPEC_DP_STATE_D:
		*mux_mode = TSI_SKYLP_MUX_MODE_USB_2DP;
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(tsi_skylp_mux_mode_from_typec);

/* One line: a failed read keeps the last state and is reported, not applied. */
static unsigned int tsi_skylp_plug_line(bool *state, int raw, unsigned int changed)
{
	bool now;

	if (raw < 0)
		return TSI_SKYLP_PLUG_READ_FAILED;
	now = raw != 0;
	if (now == *state)
		return 0;
	*state = now;
	return changed;
}

unsigned int tsi_skylp_plug_sync(struct tsi_skylp_plug_state *s,
				 int flip_raw, int event_raw)
{
	return tsi_skylp_plug_line(&s->flip, flip_raw, TSI_SKYLP_PLUG_FLIP_CHANGED) |
	       tsi_skylp_plug_line(&s->present, event_raw,
				   TSI_SKYLP_PLUG_PRESENCE_CHANGED);
}
EXPORT_SYMBOL_GPL(tsi_skylp_plug_sync);

/* Binder: DT + MMIO plumbing over the tested sequence core. */

struct tsi_skylp_iomem {
	void __iomem *regs[TSI_SKYLP_REG_COUNT];	/* indexed by enum tsi_skylp_reg */
};

static void tsi_skylp_iomem_wr(void *ctx, enum tsi_skylp_reg reg, u32 val)
{
	struct tsi_skylp_iomem *io = ctx;

	writel(val, io->regs[reg]);
}

static u32 tsi_skylp_iomem_rd(void *ctx, enum tsi_skylp_reg reg)
{
	struct tsi_skylp_iomem *io = ctx;

	return readl(io->regs[reg]);
}

static void tsi_skylp_iomem_delay(void *ctx, unsigned int us)
{
	usleep_range(us, us + 20);
}

/*
 * Map a named window. No request_mem_region: the GPP registers (tsar,
 * clksel) sit inside the chiplet CSR window already claimed by the
 * tsi-chiplet core, and the mux word lives in the DP/PHY APB space.
 */
/*
 * DT names the extra windows (tsi-mux, tsi-tsar, tsi-clksel). ACPI _CRS
 * entries carry no names, so there they are taken by position after the
 * controller window: 1 = mux, 2 = tsar_control, 3 = udi_clk_sel, the order
 * tools/tsi/acpi/skylp-udi.asl fixes.
 */
static void __iomem *tsi_skylp_map(struct platform_device *pdev,
				   const char *name, unsigned int index)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res && !pdev->dev.of_node)
		res = platform_get_resource(pdev, IORESOURCE_MEM, index);
	if (!res)
		return NULL;	/* genuinely absent: optional windows skip */
	dev_info(&pdev->dev, "%s window %pR\n", name, res);
	/* Declared but unmappable must fail, not degrade to "absent". */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
	       ERR_PTR(-ENOMEM);
}

int tsi_skylp_usb_parse(const struct fwnode_handle *fw, struct tsi_skylp_init *init)
{
	if (!fw || !fwnode_device_is_compatible(fw, "tsi,skylp-usb"))
		return -ENODEV;

	memset(init, 0, sizeof(*init));
	init->has_tsar = !fwnode_property_read_u32(fw, "tsi,tsar-init",
						   &init->tsar_val);
	init->has_clksel = !fwnode_property_read_u32(fw, "tsi,clksel-init",
						     &init->clksel_val);
	init->mux_mode = TSI_SKYLP_MUX_MODE_USB;
	fwnode_property_read_u32(fw, "tsi,mux-mode", &init->mux_mode);
	if (init->mux_mode > TSI_SKYLP_MUX_MODE_4DP)
		return -EINVAL;
	init->mux_flip = fwnode_property_read_bool(fw, "tsi,mux-flip");
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_parse);

int tsi_skylp_usb_init(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tsi_skylp_init init;
	struct tsi_skylp_iomem *io;
	struct tsi_skylp_hw hw = {
		.rd = tsi_skylp_iomem_rd,
		.wr = tsi_skylp_iomem_wr,
		.delay_us = tsi_skylp_iomem_delay,
	};
	struct clk_bulk_data *clks;
	int ret;

	ret = tsi_skylp_usb_parse(dev_fwnode(dev), &init);
	if (ret == -ENODEV)
		return 0;	/* not a SkyLP node: vendor defaults */
	if (ret)
		return dev_err_probe(dev, ret, "bad tsi,mux-mode\n");

	dev_info(dev, "SkyLP init: mux mode %u flip %u, tsar-init %#x (%s), clksel-init %#x (%s)\n",
		 init.mux_mode, init.mux_flip,
		 init.tsar_val, init.has_tsar ? "applied" : "absent, reset value kept",
		 init.clksel_val, init.has_clksel ? "applied" : "absent, reset value kept");

	/*
	 * soc_drd itself consumes no clocks at all (gap G8) - unlike dwc3,
	 * which does a clk_bulk_get. Enable whatever the DT lists for this
	 * node before the sequence runs, and hold them for the device's
	 * lifetime. A node with no clocks property is a no-op, which is the
	 * case until the UDI clock topology is confirmed (HW-27).
	 */
	ret = devm_clk_bulk_get_all_enable(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	io = devm_kzalloc(dev, sizeof(*io), GFP_KERNEL);
	if (!io)
		return -ENOMEM;
	hw.ctx = io;

	io->regs[TSI_SKYLP_REG_MUX] = tsi_skylp_map(pdev, "tsi-mux", 1);
	if (IS_ERR(io->regs[TSI_SKYLP_REG_MUX]))
		return dev_err_probe(dev, PTR_ERR(io->regs[TSI_SKYLP_REG_MUX]),
				     "tsi-mux window map failed\n");
	if (!io->regs[TSI_SKYLP_REG_MUX])
		return dev_err_probe(dev, -EINVAL,
				     "missing tsi-mux window (mux resets to no-connection)\n");

	if (init.has_tsar) {
		io->regs[TSI_SKYLP_REG_TSAR] = tsi_skylp_map(pdev, "tsi-tsar", 2);
		if (IS_ERR(io->regs[TSI_SKYLP_REG_TSAR]))
			return dev_err_probe(dev,
					     PTR_ERR(io->regs[TSI_SKYLP_REG_TSAR]),
					     "tsi-tsar window map failed\n");
		if (!io->regs[TSI_SKYLP_REG_TSAR])
			return dev_err_probe(dev, -EINVAL,
					     "tsi,tsar-init without tsi-tsar window\n");
	}

	if (init.has_clksel) {
		io->regs[TSI_SKYLP_REG_CLKSEL] = tsi_skylp_map(pdev,
							       "tsi-clksel", 3);
		if (IS_ERR(io->regs[TSI_SKYLP_REG_CLKSEL]))
			return dev_err_probe(dev,
					     PTR_ERR(io->regs[TSI_SKYLP_REG_CLKSEL]),
					     "tsi-clksel window map failed\n");
		if (!io->regs[TSI_SKYLP_REG_CLKSEL])
			return dev_err_probe(dev, -EINVAL,
					     "tsi,clksel-init without tsi-clksel window\n");
	}

	/*
	 * Window-gated, not property-gated: a board that declares the
	 * udi_reset_cfg word wants Linux to release the USB resets; one
	 * that leaves it out had firmware do it.
	 */
	io->regs[TSI_SKYLP_REG_RESET] = tsi_skylp_map(pdev, "tsi-reset", 4);
	if (IS_ERR(io->regs[TSI_SKYLP_REG_RESET]))
		return dev_err_probe(dev, PTR_ERR(io->regs[TSI_SKYLP_REG_RESET]),
				     "tsi-reset window declared but unmappable: USB may be held in reset\n");
	init.has_reset = io->regs[TSI_SKYLP_REG_RESET] != NULL;
	if (!init.has_reset)
		dev_info(dev, "no tsi-reset window: UDI resets assumed released\n");

	ret = tsi_skylp_usb_init_seq(&hw, &init);
	if (ret)
		return ret;

	/*
	 * Read the mux back. A value other than the one written means the
	 * write did not land: wrong window, a blocked bus, or a register the
	 * hardware owns (HW-36).
	 */
	{
		u32 want = tsi_skylp_mux_val(init.mux_mode, init.mux_flip);
		u32 got = readl(io->regs[TSI_SKYLP_REG_MUX]);

		if (got == want)
			dev_info(dev, "lane mux %#010x (written and read back)\n", got);
		else
			dev_warn(dev, "lane mux wrote %#010x, reads %#010x: write did not land (HW-36?)\n",
				 want, got);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_init);

int tsi_skylp_usb_core_init(struct device *dev, void __iomem *globals)
{
	struct tsi_skylp_iomem io = { };
	struct tsi_skylp_hw hw = {
		.rd = tsi_skylp_iomem_rd,
		.wr = tsi_skylp_iomem_wr,
		.ctx = &io,
	};
	struct tsi_skylp_init init;
	unsigned long ref_clk_hz, period, scale;
	u32 gctl, guctl;
	int ret;

	ret = tsi_skylp_usb_parse(dev_fwnode(dev), &init);
	if (ret == -ENODEV)
		return 0;	/* not a SkyLP node: vendor defaults */
	if (ret)
		return ret;

	/* soc_usb's regs point at the GLOBALS window; offsets are xHCI-based. */
	io.regs[TSI_SKYLP_REG_GCTL] = globals + SOC_USB_GCTL - SOC_USB_GLOBALS_REGS_START;
	io.regs[TSI_SKYLP_REG_GUCTL] = globals + SOC_USB_GUCTL - SOC_USB_GLOBALS_REGS_START;

	gctl = readl(io.regs[TSI_SKYLP_REG_GCTL]);
	guctl = readl(io.regs[TSI_SKYLP_REG_GUCTL]);

	ref_clk_hz = tsi_skylp_usb_ref_clk_hz(&init);
	ret = tsi_skylp_usb_refclk_seq(&hw, ref_clk_hz);
	if (ret)
		return dev_err_probe(dev, ret, "refclk period / power-down scale\n");

	/*
	 * The field positions are inferred from the DWC3 layout (see
	 * soc_skylp.h); this readback is what confirms them on silicon.
	 */
	period = FIELD_GET(TSI_SKYLP_GUCTL_REFCLKPER, readl(io.regs[TSI_SKYLP_REG_GUCTL]));
	scale = FIELD_GET(TSI_SKYLP_GCTL_PWRDNSCALE, readl(io.regs[TSI_SKYLP_REG_GCTL]));
	dev_info(dev, "refclk %lu Hz: GUCTL.REFCLKPER %lu -> %lu ns, GCTL.PWRDNSCALE %lu -> %lu\n",
		 ref_clk_hz,
		 FIELD_GET(TSI_SKYLP_GUCTL_REFCLKPER, guctl), period,
		 FIELD_GET(TSI_SKYLP_GCTL_PWRDNSCALE, gctl), scale);
	if (period != DIV_ROUND_CLOSEST(NSEC_PER_SEC, ref_clk_hz) ||
	    scale != ref_clk_hz / 16000)
		dev_warn(dev, "refclk fields did not take the written values\n");
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_skylp_usb_core_init);

MODULE_DESCRIPTION("TSI SkyLP USB init-sequence core");
MODULE_LICENSE("GPL");
