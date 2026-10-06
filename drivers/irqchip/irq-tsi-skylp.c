// SPDX-License-Identifier: GPL-2.0-only
/*
 * TSI SkyLP interrupt-collector irqchip ("tsi,skylp-intc").
 *
 * Standalone driver for the TSI collector blocks that fan IP interrupt
 * sources into four destination groups. The register pattern and the
 * mask/unmask/ack discipline are the ones established (and KUnit-
 * tested) for the IO-corner collectors in drivers/pinctrl/tsi; this
 * driver serves the collectors that carry non-GPIO sources — first the
 * GPP/UDI instance (bit 0 usb_intr, bit 1 dptx_intr, video bridge and
 * audio behind them).
 *
 * One instance owns ONE destination group (tsi,dest-group, default 0).
 * The other groups may be routed to other masters (M85); the shared
 * global enable is the one register where this driver can affect what
 * those masters see — same unresolved ownership note as pinctrl-tsi.
 *
 * The parent interrupt (collector group -> GIC SPI) is still an open
 * hardware question (gap G3 / HW-1). Like the corner collectors, the
 * driver probes dormant without it: the domain exists, consumers can
 * map and request their lines, and nothing fires until the routing is
 * known and the DT gains an interrupts property.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/visibility.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "irq-tsi-skylp.h"

static u32 tsi_intc_grp_reg(struct tsi_intc *ti, u32 which)
{
	return ti->base + TSI_INTC_GRP_BASE +
	       ti->dest_grp * TSI_INTC_GRP_STRIDE + which;
}

/* Disarm for our group only; the shared global enable is untouched. */
VISIBLE_IF_KUNIT void tsi_intc_irq_mask_hw(struct tsi_intc *ti,
					   unsigned int hwirq)
{
	regmap_clear_bits(ti->regmap, tsi_intc_grp_reg(ti, TSI_INTC_GRP_EN),
			  BIT(hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_irq_mask_hw);

/*
 * Arm the source: set its bit in BOTH the shared global enable and this
 * group's routing enable. The global register is shared with the other
 * destination groups (M85 et al.) - same unarbitrated-ownership note as
 * the corner collectors.
 */
VISIBLE_IF_KUNIT void tsi_intc_irq_unmask_hw(struct tsi_intc *ti,
					     unsigned int hwirq)
{
	regmap_set_bits(ti->regmap, ti->base + TSI_INTC_GLB_EN, BIT(hwirq));
	regmap_set_bits(ti->regmap, tsi_intc_grp_reg(ti, TSI_INTC_GRP_EN),
			BIT(hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_irq_unmask_hw);

/* Write-1-clear the latched pending bit. */
VISIBLE_IF_KUNIT void tsi_intc_irq_ack_hw(struct tsi_intc *ti,
					  unsigned int hwirq)
{
	regmap_write(ti->regmap, ti->base + TSI_INTC_W1C, BIT(hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_irq_ack_hw);

/* Level-sensitive only; no polarity register exists. */
VISIBLE_IF_KUNIT int tsi_intc_irq_type_valid(unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_irq_type_valid);

VISIBLE_IF_KUNIT int tsi_intc_parse(const struct fwnode_handle *fw,
				    u32 *dest_grp, u32 *nr_sources)
{
	if (fwnode_property_read_u32(fw, "tsi,dest-group", dest_grp))
		*dest_grp = 0;
	if (*dest_grp >= TSI_INTC_NR_GROUPS)
		return -EINVAL;

	if (fwnode_property_read_u32(fw, "tsi,num-sources", nr_sources))
		*nr_sources = TSI_INTC_MAX_SOURCES;
	if (!*nr_sources || *nr_sources > TSI_INTC_MAX_SOURCES)
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_parse);

VISIBLE_IF_KUNIT int tsi_intc_drain(struct tsi_intc *ti,
				    void (*fire)(unsigned int hwirq,
						 void *cookie),
				    void *cookie)
{
	unsigned int hwirq;
	unsigned long ip;
	u32 val = 0;
	int n = 0;

	if (regmap_read(ti->regmap, tsi_intc_grp_reg(ti, TSI_INTC_GRP_IP),
			&val))
		return 0;
	ip = val & GENMASK(ti->nr_sources - 1, 0);

	/* no ack here: the flow's irq_eoi clears the latch after the child */
	for_each_set_bit(hwirq, &ip, ti->nr_sources) {
		fire(hwirq, cookie);
		n++;
	}
	return n;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_drain);

/* irq_chip plumbing over the _hw primitives */

static void tsi_intc_mask(struct irq_data *d)
{
	tsi_intc_irq_mask_hw(irq_data_get_irq_chip_data(d), irqd_to_hwirq(d));
}

static void tsi_intc_unmask(struct irq_data *d)
{
	tsi_intc_irq_unmask_hw(irq_data_get_irq_chip_data(d),
			       irqd_to_hwirq(d));
}

/*
 * The latch is sticky and level-transparent (HW-2): clearing it while the
 * child still asserts its source is undone at once, and clearing it before
 * the child runs leaves it set after the child clears the source, which
 * fires the line a second time for nothing. So the W1C is an EOI, issued
 * by handle_fasteoi_irq after the handler, and after the thread for
 * threaded handlers (IRQCHIP_EOI_THREADED).
 */
static void tsi_intc_eoi(struct irq_data *d)
{
	tsi_intc_irq_ack_hw(irq_data_get_irq_chip_data(d), irqd_to_hwirq(d));
}

VISIBLE_IF_KUNIT const irq_flow_handler_t tsi_intc_level_flow = handle_fasteoi_irq;
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_level_flow);

static int tsi_intc_set_type(struct irq_data *d, unsigned int type)
{
	int ret = tsi_intc_irq_type_valid(type);

	if (ret)
		return ret;
	irq_set_handler_locked(d, tsi_intc_level_flow);
	return 0;
}

VISIBLE_IF_KUNIT const struct irq_chip tsi_intc_chip = {
	.name		= "tsi-skylp-intc",
	.irq_mask	= tsi_intc_mask,
	.irq_unmask	= tsi_intc_unmask,
	.irq_eoi	= tsi_intc_eoi,
	.irq_set_type	= tsi_intc_set_type,
	.flags		= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_EOI_THREADED,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_intc_chip);

static int tsi_intc_domain_map(struct irq_domain *d, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	struct tsi_intc *ti = d->host_data;

	irq_set_chip_data(irq, ti);
	/*
	 * handle_bad_irq until irq_set_type() installs the level handler,
	 * same trap as the corner collectors: an unconfigured trigger
	 * complains instead of being silently serviced.
	 */
	irq_set_chip_and_handler(irq, &tsi_intc_chip, handle_bad_irq);
	irq_set_probe(irq);
	return 0;
}

static const struct irq_domain_ops tsi_intc_domain_ops = {
	.map		= tsi_intc_domain_map,
	/* two cells from DT, or hwirq + flags from an ACPI ResourceSource */
	.translate	= irq_domain_translate_twocell,
};

static void tsi_intc_fire(unsigned int hwirq, void *cookie)
{
	struct tsi_intc *ti = cookie;

	generic_handle_domain_irq(ti->domain, hwirq);
}

/* Chained on the parent (GIC) IRQ. */
static void tsi_intc_handler(struct irq_desc *desc)
{
	struct tsi_intc *ti = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);
	tsi_intc_drain(ti, tsi_intc_fire, ti);
	chained_irq_exit(chip, desc);
}

static int tsi_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tsi_intc *ti;
	void __iomem *base;
	struct regmap_config cfg = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
	};
	struct resource *res;
	int irq;

	ti = devm_kzalloc(dev, sizeof(*ti), GFP_KERNEL);
	if (!ti)
		return -ENOMEM;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	cfg.max_register = resource_size(res) - 4;
	ti->regmap = devm_regmap_init_mmio(dev, base, &cfg);
	if (IS_ERR(ti->regmap))
		return PTR_ERR(ti->regmap);
	ti->base = 0;	/* reg points at the collector origin */

	if (tsi_intc_parse(dev_fwnode(dev), &ti->dest_grp, &ti->nr_sources))
		return dev_err_probe(dev, -EINVAL,
				     "bad tsi,dest-group / tsi,num-sources\n");

	/*
	 * The domain is keyed on the device's firmware node, so an ACPI child
	 * naming this device as its Interrupt ResourceSource resolves here
	 * (and defers until this probe has run), exactly as a DT child does
	 * through interrupt-parent.
	 */
	ti->domain = irq_domain_create_linear(dev_fwnode(dev), ti->nr_sources,
					      &tsi_intc_domain_ops, ti);
	if (!ti->domain)
		return -ENOMEM;

	/*
	 * Collector-group -> GIC routing is gap G3: without an answer the
	 * firmware carries no parent interrupt (-ENXIO) and the instance
	 * stays dormant (domain up, nothing dispatches). Any other failure,
	 * a parent not yet up or a bad mapping, fails the probe instead of
	 * producing a dormant chip that deferred probe would never retry.
	 */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		irq_set_chained_handler_and_data(irq, tsi_intc_handler, ti);
	} else if (irq == -ENXIO) {
		dev_info(dev, "no parent interrupt, dormant (G3 open)\n");
	} else {
		irq_domain_remove(ti->domain);
		return dev_err_probe(dev, irq, "parent interrupt\n");
	}

	platform_set_drvdata(pdev, ti);
	return 0;
}

static const struct of_device_id tsi_intc_of_match[] = {
	{ .compatible = "tsi,skylp-intc" },
	{ }
};
MODULE_DEVICE_TABLE(of, tsi_intc_of_match);

static struct platform_driver tsi_intc_driver = {
	.probe = tsi_intc_probe,
	.driver = {
		.name = "irq-tsi-skylp",
		.of_match_table = tsi_intc_of_match,
		.suppress_bind_attrs = true,
	},
};

/*
 * Built in only (Kconfig bool): the chained handler on the parent line and
 * the irq domain stay registered for the life of the kernel, so there is
 * no unload path for them to outlive.
 */
builtin_platform_driver(tsi_intc_driver);

MODULE_DESCRIPTION("TSI SkyLP interrupt-collector irqchip");
MODULE_LICENSE("GPL");
