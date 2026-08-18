// SPDX-License-Identifier: GPL-2.0-only
/*
 * TSI chiplet core driver.
 *
 * Owns the CSR window of one chiplet: maps it, wraps it in a regmap,
 * and probes the IP blocks (gpio, pvt, mbox, i2c, wdt) declared as DT
 * children. IP drivers get register access only through this core (see
 * include/linux/soc/tsi/tsi-chiplet.h).
 *
 * On TSISIM the loads/stores terminate in vFlex via the skylp-mmio-dev
 * window; on silicon the same accesses reach flop-backed registers. The
 * core is deliberately ignorant of which one is behind it.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/tsi/tsi-chiplet.h>

static const struct regmap_config tsi_chiplet_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	/* max_register is set per instance from the DT reg size. */
};

struct tsi_chiplet *tsi_chiplet_get(struct device *child)
{
	struct tsi_chiplet *chiplet;
	struct device *parent = child->parent;

	if (!parent || !parent->of_node ||
	    !of_device_is_compatible(parent->of_node, "tsi,skylp-chiplet"))
		return ERR_PTR(-ENODEV);

	chiplet = dev_get_drvdata(parent);
	if (!chiplet)
		return ERR_PTR(-EPROBE_DEFER);

	return chiplet;
}
EXPORT_SYMBOL_GPL(tsi_chiplet_get);

int tsi_chiplet_poll(struct tsi_chiplet *chiplet, u32 reg, u32 mask,
		     u32 expect, u32 timeout_us)
{
	u32 val;

	return regmap_read_poll_timeout(chiplet->regmap, reg, val,
					(val & mask) == expect,
					0, timeout_us);
}
EXPORT_SYMBOL_GPL(tsi_chiplet_poll);

static int tsi_chiplet_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap_config regmap_cfg = tsi_chiplet_regmap_config;
	struct tsi_chiplet *chiplet;
	struct resource *res;
	void __iomem *base;
	int ret;

	chiplet = devm_kzalloc(dev, sizeof(*chiplet), GFP_KERNEL);
	if (!chiplet)
		return -ENOMEM;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (resource_size(res) < 4 || (resource_size(res) & 0x3))
		return dev_err_probe(dev, -EINVAL,
				     "invalid CSR window size\n");
	regmap_cfg.max_register = resource_size(res) - 4;
	chiplet->regmap = devm_regmap_init_mmio(dev, base, &regmap_cfg);
	if (IS_ERR(chiplet->regmap))
		return dev_err_probe(dev, PTR_ERR(chiplet->regmap),
				     "failed to init CSR regmap\n");

	ret = of_property_read_u32(dev->of_node, "tsi,chiplet-id",
				   &chiplet->chiplet_id);
	if (ret)
		return dev_err_probe(dev, ret, "missing tsi,chiplet-id\n");

	chiplet->dev = dev;
	dev_set_drvdata(dev, chiplet);

	/* Probe the IP blocks declared as children of this chiplet. */
	ret = devm_of_platform_populate(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to populate IPs\n");

	dev_info(dev, "chiplet %u: CSR window %pR\n", chiplet->chiplet_id,
		 res);

	return 0;
}

static const struct of_device_id tsi_chiplet_of_match[] = {
	{ .compatible = "tsi,skylp-chiplet" },
	{ }
};
MODULE_DEVICE_TABLE(of, tsi_chiplet_of_match);

static struct platform_driver tsi_chiplet_driver = {
	.probe = tsi_chiplet_probe,
	.driver = {
		.name = "tsi-chiplet",
		.of_match_table = tsi_chiplet_of_match,
	},
};
module_platform_driver(tsi_chiplet_driver);

MODULE_AUTHOR("Tsavorite Scalable Intelligence");
MODULE_DESCRIPTION("TSI chiplet CSR window core driver");
MODULE_LICENSE("GPL");
