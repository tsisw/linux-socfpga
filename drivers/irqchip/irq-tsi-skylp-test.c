// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the TSI SkyLP interrupt-collector irqchip.
 *
 * The register discipline under test is the one established for the
 * corner collectors (drivers/pinctrl/tsi), re-checked here against
 * independent literals from ral.json for the GPP/UDI instance
 * (udi_intr @ 0x2400_0108, 8 sources: usb, dptx, 2x vid_bridge
 * frame-done, 2x vid_bridge underflow, 2x audio):
 *
 *  - unmask arms BOTH the shared global enable and this group's enable
 *  - mask disarms only this group's enable (never the shared global)
 *  - ack is a single W1C write of the source bit
 *  - only level trigger types are accepted
 *  - the ack is an EOI, after the child handler: the latch is sticky and
 *    level-transparent (HW-2), so draining fires without acking and the
 *    chip has no pre-handler irq_ack
 *
 * Driven against a fake regmap that records every write.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>

#include "irq-tsi-skylp.h"

#define FAKE_NREGS	128
#define FAKE_MAX_WRITES	16

/* Independent expectations: udi_intr collector, regmap origin = 0. */
#define UDI_BASE	0x108
#define UDI_W1C		(UDI_BASE + 0x04)
#define UDI_GLB_EN	(UDI_BASE + 0x0c)
#define UDI_G0_IP	(UDI_BASE + 0x14)
#define UDI_G0_EN	(UDI_BASE + 0x18)
#define UDI_G2_IP	(UDI_BASE + 0x2c)
#define UDI_G2_EN	(UDI_BASE + 0x30)

#define BIT_USB		BIT(0)
#define BIT_DPTX	BIT(1)

struct fake_regs {
	u32 regs[FAKE_NREGS];
	struct { u32 reg; u32 val; } writes[FAKE_MAX_WRITES];
	int nwrites;
};

static int fake_read(void *context, unsigned int reg, unsigned int *val)
{
	struct fake_regs *f = context;

	*val = f->regs[reg / 4];
	return 0;
}

static int fake_write(void *context, unsigned int reg, unsigned int val)
{
	struct fake_regs *f = context;

	if (f->nwrites < FAKE_MAX_WRITES) {
		f->writes[f->nwrites].reg = reg;
		f->writes[f->nwrites].val = val;
	}
	f->nwrites++;
	/* Model plain RW for enables; W1C/W1S aliases left to the tests. */
	f->regs[reg / 4] = val;
	return 0;
}

static const struct regmap_config fake_cfg = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = (FAKE_NREGS - 1) * 4,
	.reg_read = fake_read,
	.reg_write = fake_write,
};

struct intc_fixture {
	struct fake_regs *f;
	struct tsi_intc ti;
};

static struct intc_fixture *intc_fixture(struct kunit *test, u32 dest_grp)
{
	struct intc_fixture *fx;
	struct device *dev;
	struct regmap *map;

	fx = kunit_kzalloc(test, sizeof(*fx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fx);
	fx->f = kunit_kzalloc(test, sizeof(*fx->f), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fx->f);

	dev = kunit_device_register(test, "tsi-intc-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	map = devm_regmap_init(dev, NULL, fx->f, &fake_cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, map);

	fx->ti.regmap = map;
	fx->ti.base = UDI_BASE;
	fx->ti.dest_grp = dest_grp;
	fx->ti.nr_sources = 8;
	return fx;
}

/* unmask(usb): sets BIT(0) in the global enable AND in enable_g0 */
static void tsi_intc_test_unmask_arms_global_and_group(struct kunit *test)
{
	struct intc_fixture *fx = intc_fixture(test, 0);

	tsi_intc_irq_unmask_hw(&fx->ti, 0);

	KUNIT_EXPECT_EQ(test, fx->f->regs[UDI_GLB_EN / 4], BIT_USB);
	KUNIT_EXPECT_EQ(test, fx->f->regs[UDI_G0_EN / 4], BIT_USB);
}

/* mask(dptx): clears only the group enable; global enable untouched */
static void tsi_intc_test_mask_clears_group_only(struct kunit *test)
{
	struct intc_fixture *fx = intc_fixture(test, 0);

	fx->f->regs[UDI_GLB_EN / 4] = BIT_USB | BIT_DPTX;
	fx->f->regs[UDI_G0_EN / 4] = BIT_USB | BIT_DPTX;

	tsi_intc_irq_mask_hw(&fx->ti, 1);

	KUNIT_EXPECT_EQ(test, fx->f->regs[UDI_G0_EN / 4], BIT_USB);
	KUNIT_EXPECT_EQ(test, fx->f->regs[UDI_GLB_EN / 4], BIT_USB | BIT_DPTX);
}

/* ack(usb): exactly one write, to the W1C alias, of just that bit */
static void tsi_intc_test_ack_is_single_w1c_write(struct kunit *test)
{
	struct intc_fixture *fx = intc_fixture(test, 0);

	tsi_intc_irq_ack_hw(&fx->ti, 0);

	KUNIT_EXPECT_EQ(test, fx->f->nwrites, 1);
	KUNIT_EXPECT_EQ(test, fx->f->writes[0].reg, UDI_W1C);
	KUNIT_EXPECT_EQ(test, fx->f->writes[0].val, BIT_USB);
}

/* the dest-group knob moves the enable to g2's registers */
static void tsi_intc_test_dest_group_selects_registers(struct kunit *test)
{
	struct intc_fixture *fx = intc_fixture(test, 2);

	tsi_intc_irq_unmask_hw(&fx->ti, 1);

	KUNIT_EXPECT_EQ(test, fx->f->regs[UDI_G2_EN / 4], BIT_DPTX);
	KUNIT_EXPECT_EQ(test, fx->f->regs[UDI_G0_EN / 4], 0u);
}

/* collector is level-only: edge requests must be rejected */
static void tsi_intc_test_rejects_edge_types(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, tsi_intc_irq_type_valid(IRQ_TYPE_LEVEL_HIGH), 0);
	KUNIT_EXPECT_EQ(test, tsi_intc_irq_type_valid(IRQ_TYPE_LEVEL_LOW), 0);
	KUNIT_EXPECT_EQ(test, tsi_intc_irq_type_valid(IRQ_TYPE_EDGE_RISING),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, tsi_intc_irq_type_valid(IRQ_TYPE_EDGE_BOTH),
			-EINVAL);
}

struct drain_log {
	unsigned int fired[8];
	int nfired;
	struct fake_regs *f;
	struct kunit *test;
};

static void drain_fire(unsigned int hwirq, void *cookie)
{
	struct drain_log *log = cookie;

	/* nothing may be acked before the child has run */
	KUNIT_EXPECT_EQ(log->test, log->f->nwrites, 0);
	if (log->nfired < 8)
		log->fired[log->nfired] = hwirq;
	log->nfired++;
}

/* drain fires each ip_status_g bit low-to-high and writes nothing */
static void tsi_intc_test_drain_fires_in_order_without_ack(struct kunit *test)
{
	struct intc_fixture *fx = intc_fixture(test, 0);
	struct drain_log log = { .f = fx->f, .test = test };
	int n;

	fx->f->regs[UDI_G0_IP / 4] = BIT_USB | BIT_DPTX;

	n = tsi_intc_drain(&fx->ti, drain_fire, &log);

	KUNIT_EXPECT_EQ(test, n, 2);
	KUNIT_EXPECT_EQ(test, log.nfired, 2);
	KUNIT_EXPECT_EQ(test, log.fired[0], 0u);
	KUNIT_EXPECT_EQ(test, log.fired[1], 1u);
	KUNIT_EXPECT_EQ(test, fx->f->nwrites, 0);
}

/*
 * The flow must ack after the handler: fasteoi with the W1C as irq_eoi,
 * deferred past threaded handlers, and no irq_ack for a flow to call
 * before the child has cleared its source.
 */
static void tsi_intc_test_ack_is_post_handler_eoi(struct kunit *test)
{
	KUNIT_EXPECT_PTR_EQ(test, (void *)tsi_intc_level_flow,
			    (void *)handle_fasteoi_irq);
	KUNIT_EXPECT_NULL(test, tsi_intc_chip.irq_ack);
	KUNIT_EXPECT_NOT_NULL(test, tsi_intc_chip.irq_eoi);
	KUNIT_EXPECT_TRUE(test, tsi_intc_chip.flags & IRQCHIP_EOI_THREADED);
}

/* nothing pending: no writes, no fires, returns 0 */
static void tsi_intc_test_drain_idle_is_silent(struct kunit *test)
{
	struct intc_fixture *fx = intc_fixture(test, 0);
	struct drain_log log = { .f = fx->f, .test = test };

	KUNIT_EXPECT_EQ(test, tsi_intc_drain(&fx->ti, drain_fire, &log), 0);
	KUNIT_EXPECT_EQ(test, log.nfired, 0);
	KUNIT_EXPECT_EQ(test, fx->f->nwrites, 0);
}

static struct kunit_case tsi_intc_test_cases[] = {
	KUNIT_CASE(tsi_intc_test_unmask_arms_global_and_group),
	KUNIT_CASE(tsi_intc_test_mask_clears_group_only),
	KUNIT_CASE(tsi_intc_test_ack_is_single_w1c_write),
	KUNIT_CASE(tsi_intc_test_dest_group_selects_registers),
	KUNIT_CASE(tsi_intc_test_rejects_edge_types),
	KUNIT_CASE(tsi_intc_test_drain_fires_in_order_without_ack),
	KUNIT_CASE(tsi_intc_test_ack_is_post_handler_eoi),
	KUNIT_CASE(tsi_intc_test_drain_idle_is_silent),
	{}
};

static struct kunit_suite tsi_intc_test_suite = {
	.name = "tsi-skylp-intc",
	.test_cases = tsi_intc_test_cases,
};
kunit_test_suite(tsi_intc_test_suite);

MODULE_DESCRIPTION("KUnit tests for the TSI SkyLP interrupt collector");
MODULE_LICENSE("GPL");
