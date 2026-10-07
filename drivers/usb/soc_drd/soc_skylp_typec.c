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
 * trigger capabilities are confirmed. What a poll does with the two
 * samples, including a failed read, is tsi_skylp_plug_sync() in the
 * core, where it is KUnit-tested; this file only reads the lines and
 * reports.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#include "soc_skylp.h"

#define SKYLP_TYPEC_POLL_MS	100

struct skylp_typec {
	struct device *dev;
	void __iomem *mux;
	struct tsi_skylp_hw hw;
	struct tsi_skylp_mux_cache cache;
	struct mutex lock;		/* serialises mode/flip against apply */
	u32 mode;
	struct tsi_skylp_plug_state plug;
	struct gpio_desc *plug_event;
	struct gpio_desc *plug_flip;
	struct delayed_work work;
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux_dev;
};

static void skylp_typec_wr(void *ctx, enum tsi_skylp_reg reg, u32 val)
{
	struct skylp_typec *st = ctx;

	writel(val, st->mux);
}

/* Single writer for the mux word; caller holds the lock. */
static void skylp_typec_apply(struct skylp_typec *st)
{
	lockdep_assert_held(&st->lock);

	if (tsi_skylp_mux_update(&st->cache, &st->hw, st->mode, st->plug.flip))
		dev_dbg(st->dev, "lane mux set: mode %u, %s orientation\n",
			st->mode, st->plug.flip ? "flipped" : "normal");
}

/*
 * Orientation from a port manager. On Helix-M nothing calls this: the PD
 * controller's flip output stops at the board CPLD, so the mux runs at
 * the fixed orientation below. A board that routes orientation to a port
 * manager drives this path instead, with no change here.
 */
static int skylp_typec_switch_set(struct typec_switch_dev *sw,
				  enum typec_orientation orientation)
{
	struct skylp_typec *st = typec_switch_get_drvdata(sw);

	guard(mutex)(&st->lock);
	st->plug.flip = orientation == TYPEC_ORIENTATION_REVERSE;
	skylp_typec_apply(st);
	return 0;
}

/* Mode from a port manager: USB, USB+2-lane DP, or 4-lane DP. */
static int skylp_typec_mux_set(struct typec_mux_dev *mux,
			       struct typec_mux_state *state)
{
	struct skylp_typec *st = typec_mux_get_drvdata(mux);
	u32 mode;
	int ret;

	ret = tsi_skylp_mux_mode_from_typec(state->mode, &mode);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	st->mode = mode;
	skylp_typec_apply(st);
	return 0;
}

/*
 * One poll. Lines the board does not route are passed as their current
 * state so the core sees "unchanged"; a read error keeps the last state.
 */
static void skylp_typec_sync(struct skylp_typec *st)
{
	int flip_raw, event_raw;
	unsigned int ev;

	guard(mutex)(&st->lock);

	flip_raw = st->plug_flip ? gpiod_get_value_cansleep(st->plug_flip) :
				   st->plug.flip;
	event_raw = st->plug_event ? gpiod_get_value_cansleep(st->plug_event) :
				     st->plug.present;
	ev = tsi_skylp_plug_sync(&st->plug, flip_raw, event_raw);

	if (ev & TSI_SKYLP_PLUG_READ_FAILED)
		dev_warn_ratelimited(st->dev,
				     "plug gpio read failed (flip %d, event %d), keeping last state\n",
				     flip_raw, event_raw);

	if (ev & (TSI_SKYLP_PLUG_FLIP_CHANGED | TSI_SKYLP_PLUG_PRESENCE_CHANGED))
		dev_dbg(st->dev, "plug lines: flip %d, event %d\n", flip_raw, event_raw);

	skylp_typec_apply(st);

	if (ev & TSI_SKYLP_PLUG_FLIP_CHANGED)
		dev_info(st->dev, "orientation %s\n",
			 st->plug.flip ? "reversed" : "normal");
	if (ev & TSI_SKYLP_PLUG_PRESENCE_CHANGED)
		dev_info(st->dev, "plug %s\n",
			 st->plug.present ? "attached" : "detached");
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

static void skylp_typec_unreg_switch(void *data)
{
	typec_switch_unregister(data);
}

static void skylp_typec_unreg_mux(void *data)
{
	typec_mux_unregister(data);
}

/*
 * Register as a standard orientation switch and mode mux. Nothing on
 * Helix-M drives them today - there is no port manager, because the board
 * has no TCPC and its PD controller is autonomous - but this is the
 * interface a port manager uses, so a source-capable board needs no
 * change here.
 */
static int skylp_typec_register_mux(struct skylp_typec *st)
{
	struct typec_switch_desc sw_desc = {
		.fwnode = dev_fwnode(st->dev),
		.set = skylp_typec_switch_set,
		.drvdata = st,
	};
	struct typec_mux_desc mux_desc = {
		.fwnode = dev_fwnode(st->dev),
		.set = skylp_typec_mux_set,
		.drvdata = st,
	};
	int ret;

	st->sw = typec_switch_register(st->dev, &sw_desc);
	if (IS_ERR(st->sw))
		return dev_err_probe(st->dev, PTR_ERR(st->sw),
				     "failed to register orientation switch\n");
	ret = devm_add_action_or_reset(st->dev, skylp_typec_unreg_switch,
				       st->sw);
	if (ret)
		return ret;

	st->mux_dev = typec_mux_register(st->dev, &mux_desc);
	if (IS_ERR(st->mux_dev))
		return dev_err_probe(st->dev, PTR_ERR(st->mux_dev),
				     "failed to register mode mux\n");
	return devm_add_action_or_reset(st->dev, skylp_typec_unreg_mux,
					st->mux_dev);
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
	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

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
	device_property_read_u32(dev, "tsi,mux-mode", &st->mode);
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

	ret = skylp_typec_register_mux(st);
	if (ret)
		return ret;

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
