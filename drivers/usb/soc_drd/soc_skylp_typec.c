// SPDX-License-Identifier: GPL-2.0
/*
 * TSI SkyLP Type-C plug/orientation glue ("tsi,skylp-typec").
 *
 * On Helix-M the PD controller (TI TPS25730D on J1) resolves the cable
 * orientation and plug state autonomously and reports them on two GPIO
 * lines (USB_PLUG_FLIP, USB_PLUG_EVENT) wired to SkyLP LP0 GPIO pads
 * (schematic 400-TSI-HM01-R1, CF802). This driver follows those lines
 * and keeps the Type-C lane mux's flip bit in sync, using the
 * KUnit-tested composition/update helpers from the SkyLP core.
 *
 * The lines are POLLED (like the vendor dptx driver polls HPD): the
 * pinctrl-tsi GPIO interrupt path is level-only and the pad routing of
 * CF802 is still an open hardware question, so edge interrupts cannot
 * be relied on yet. TODO: switch to interrupts once the routing and
 * trigger capabilities are confirmed.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "soc_skylp.h"

#define SKYLP_TYPEC_POLL_MS	100

struct skylp_typec {
	struct device *dev;
	void __iomem *mux;
	struct tsi_skylp_hw hw;
	struct tsi_skylp_mux_cache cache;
	u32 mode;
	struct gpio_desc *plug_event;
	struct gpio_desc *plug_flip;
	bool present;
	struct delayed_work work;
};

static void skylp_typec_wr(void *ctx, enum tsi_skylp_reg reg, u32 val)
{
	struct skylp_typec *st = ctx;

	writel(val, st->mux);
}

static void skylp_typec_sync(struct skylp_typec *st)
{
	bool flip = st->plug_flip ?
		    gpiod_get_value_cansleep(st->plug_flip) : false;

	if (tsi_skylp_mux_update(&st->cache, &st->hw, st->mode, flip))
		dev_info(st->dev, "lane mux set: mode %u, %s orientation\n",
			 st->mode, flip ? "flipped" : "normal");

	if (st->plug_event) {
		bool present = gpiod_get_value_cansleep(st->plug_event);

		if (present != st->present)
			dev_info(st->dev, "plug %s\n",
				 present ? "attached" : "detached");
		st->present = present;
	}
}

static void skylp_typec_poll(struct work_struct *work)
{
	struct skylp_typec *st = container_of(to_delayed_work(work),
					      struct skylp_typec, work);

	skylp_typec_sync(st);
	schedule_delayed_work(&st->work,
			      msecs_to_jiffies(SKYLP_TYPEC_POLL_MS));
}

static void skylp_typec_stop_poll(void *data)
{
	struct skylp_typec *st = data;

	cancel_delayed_work_sync(&st->work);
}

static int skylp_typec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct skylp_typec *st;
	struct resource *res;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;
	st->dev = dev;
	st->hw.wr = skylp_typec_wr;
	st->hw.ctx = st;

	/*
	 * The mux word sits inside the DP/PHY APB window the dptx driver
	 * claims, so map without requesting the region.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL, "missing mux reg\n");
	st->mux = devm_ioremap(dev, res->start, resource_size(res));
	if (!st->mux)
		return -ENOMEM;

	st->mode = TSI_SKYLP_MUX_MODE_USB;
	of_property_read_u32(dev->of_node, "tsi,mux-mode", &st->mode);
	if (st->mode > TSI_SKYLP_MUX_MODE_4DP)
		return dev_err_probe(dev, -EINVAL, "bad tsi,mux-mode %u\n",
				     st->mode);

	st->plug_event = devm_gpiod_get_optional(dev, "plug-event", GPIOD_IN);
	if (IS_ERR(st->plug_event))
		return dev_err_probe(dev, PTR_ERR(st->plug_event),
				     "plug-event gpio\n");
	st->plug_flip = devm_gpiod_get_optional(dev, "plug-flip", GPIOD_IN);
	if (IS_ERR(st->plug_flip))
		return dev_err_probe(dev, PTR_ERR(st->plug_flip),
				     "plug-flip gpio\n");

	skylp_typec_sync(st);

	if (st->plug_event || st->plug_flip) {
		INIT_DELAYED_WORK(&st->work, skylp_typec_poll);
		ret = devm_add_action_or_reset(dev, skylp_typec_stop_poll, st);
		if (ret)
			return ret;
		schedule_delayed_work(&st->work,
				      msecs_to_jiffies(SKYLP_TYPEC_POLL_MS));
	} else {
		dev_info(dev, "no plug gpios, static orientation\n");
	}

	platform_set_drvdata(pdev, st);
	return 0;
}

static const struct of_device_id skylp_typec_of_match[] = {
	{ .compatible = "tsi,skylp-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, skylp_typec_of_match);

static struct platform_driver skylp_typec_driver = {
	.probe = skylp_typec_probe,
	.driver = {
		.name = "tsi-skylp-typec",
		.of_match_table = skylp_typec_of_match,
	},
};
module_platform_driver(skylp_typec_driver);

MODULE_DESCRIPTION("TSI SkyLP Type-C plug/orientation glue");
MODULE_LICENSE("GPL");
