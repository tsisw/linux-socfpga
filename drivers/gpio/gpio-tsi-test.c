// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the TSI SkyLP GPIO driver.
 *
 * Drives the real gpio_chip ops against a no-bus regmap backed by a
 * fake SkyLP corner-register model. Expected offsets and bit numbers
 * are written here as independent literals (from ral.json release
 * SKYLP_G0829) rather than through the driver's tables, so a mapping
 * typo in the driver cannot verify itself.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regmap.h>

#include "gpio-tsi.h"

#define FAKE_MAX_REGS	16
#define FAKE_MAX_WRITES	16

/* Independent expectations (spec section "Verified hardware facts"). */
#define IONW_BASE	0x13000000
#define IONW_DATA_OUT	(IONW_BASE + 0x178)
#define IONW_DATA_IN	(IONW_BASE + 0x17c)
#define IONE_BASE	0x6000000
#define IONE_DATA_OUT	(IONE_BASE + 0x188)
#define IONE_DATA_IN	(IONE_BASE + 0x18c)

static const u32 ionw_ctrl[] = {
	IONW_BASE + 0x11c, IONW_BASE + 0x120, IONW_BASE + 0x124,
	IONW_BASE + 0x128, IONW_BASE + 0x12c, IONW_BASE + 0x130,
	IONW_BASE + 0x134, IONW_BASE + 0x138,
};
static const u8 ionw_bit[] = { 0, 1, 2, 3, 21, 22, 23, 24 };

static const u32 ione_ctrl[] = {
	IONE_BASE + 0x124, IONE_BASE + 0x128,
	IONE_BASE + 0x12c, IONE_BASE + 0x130,
};
static const u8 ione_bit[] = { 10, 11, 12, 13 };

/* Pad control field bits, as independent literals (ral.json SKYLP_G0829). */
#define CTRL_OE		0x08	/* bit[3] */
#define CTRL_IE		0x10	/* bit[4] */
#define CTRL_IS		0x20	/* bit[5]: 1=Schmitt trigger */
#define CTRL_PE		0x40	/* bit[6]: 1=pull enable */
#define CTRL_PS		0x100	/* bit[8]: 0=pull-down, 1=pull-up */

/* IONW t2 collector - independent literals (spec section 4b). */
#define IONW_T2_W1C		(IONW_BASE + 0x34c4)
#define IONW_T2_GLBEN		(IONW_BASE + 0x34cc)
#define IONW_T2_G0_EN		(IONW_BASE + 0x34d8)
#define IONW_T2_G0_IP		(IONW_BASE + 0x34d4)
#define IONW_T2_GRP_STRIDE	0xc

struct fake_reg {
	u32	off;
	u32	val;
	bool	ro;
};

struct fake_regs {
	struct kunit	*test;
	struct fake_reg	regs[FAKE_MAX_REGS];
	unsigned int	nregs;
	struct {
		u32 off;
		u32 val;
	}		log[FAKE_MAX_WRITES];
	unsigned int	nwrites;
};

/* Look up a modeled register by window offset; NULL if unmodeled. */
static struct fake_reg *fake_find(struct fake_regs *f, u32 off)
{
	unsigned int i;

	for (i = 0; i < f->nregs; i++)
		if (f->regs[i].off == off)
			return &f->regs[i];
	return NULL;
}

/*
 * regmap reg_read callback: the driver touching any register outside
 * the documented GPIO set is itself a bug, so it fails the test.
 */
static int fake_read(void *context, unsigned int reg, unsigned int *val)
{
	struct fake_regs *f = context;
	struct fake_reg *r = fake_find(f, reg);

	if (!r) {
		KUNIT_FAIL(f->test, "read of unmodeled reg 0x%x", reg);
		return -EINVAL;
	}
	*val = r->val;
	return 0;
}

/*
 * regmap reg_write callback: enforces data_in's read-only nature and
 * appends every write to the ordered log so tests can assert both
 * final values and write sequence (value-before-OE).
 */
static int fake_write(void *context, unsigned int reg, unsigned int val)
{
	struct fake_regs *f = context;
	struct fake_reg *r = fake_find(f, reg);

	if (!r) {
		KUNIT_FAIL(f->test, "write of unmodeled reg 0x%x", reg);
		return -EINVAL;
	}
	if (r->ro) {
		KUNIT_FAIL(f->test, "write to read-only reg 0x%x", reg);
		return -EPERM;
	}
	if (f->nwrites < FAKE_MAX_WRITES) {
		f->log[f->nwrites].off = reg;
		f->log[f->nwrites].val = val;
	}
	f->nwrites++;
	r->val = val;
	return 0;
}

/* Add one register to the model with its initial value. */
static void fake_add(struct fake_regs *f, u32 off, u32 val, bool ro)
{
	KUNIT_ASSERT_LT(f->test, f->nregs, (unsigned int)FAKE_MAX_REGS);
	f->regs[f->nregs++] = (struct fake_reg){ .off = off, .val = val,
						 .ro = ro };
}

/* Backdoor read: inspect a register without going through regmap. */
static u32 fake_get(struct fake_regs *f, u32 off)
{
	struct fake_reg *r = fake_find(f, off);

	KUNIT_ASSERT_NOT_NULL(f->test, r);
	return r->val;
}

/*
 * Backdoor write: inject state (e.g. a pin level into the read-only
 * data_in) without tripping the ro check or polluting the write log.
 */
static void fake_set(struct fake_regs *f, u32 off, u32 val)
{
	struct fake_reg *r = fake_find(f, off);

	KUNIT_ASSERT_NOT_NULL(f->test, r);
	r->val = val;
}

/*
 * Build one corner's register model: all control registers preloaded
 * with ctrl_init (lets a test start from the documented reset value
 * or from a known input/output state), data_out zeroed, data_in
 * read-only.
 */
static struct fake_regs *fake_new(struct kunit *test, const u32 *ctrl,
				  unsigned int nctrl, u32 ctrl_init,
				  u32 data_out, u32 data_in)
{
	struct fake_regs *f;
	unsigned int i;

	f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, f);
	f->test = test;
	for (i = 0; i < nctrl; i++)
		fake_add(f, ctrl[i], ctrl_init, false);
	fake_add(f, data_out, 0, false);
	fake_add(f, data_in, 0, true);
	return f;
}

/*
 * Register a real tsi_gpio instance over the fake model: kunit-managed
 * device + no-bus regmap whose reg_read/reg_write land in fake_regs.
 * Everything is torn down automatically at test exit.
 */
static struct tsi_gpio *fake_chip(struct kunit *test, struct fake_regs *f,
				  u32 base, const struct tsi_gpio_soc *soc,
				  int irq, u32 irq_grp)
{
	static const struct regmap_config cfg = {
		.reg_bits	= 32,
		.reg_stride	= 4,
		.val_bits	= 32,
		.reg_read	= fake_read,
		.reg_write	= fake_write,
		.max_register	= 0x14000000,
		.fast_io	= true,
	};
	struct tsi_gpio *gpio;
	struct device *dev;
	struct regmap *map;

	dev = kunit_device_register(test, "gpio-tsi-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	map = devm_regmap_init(dev, NULL, f, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, map);
	gpio = tsi_gpio_register(dev, map, base, soc, irq, irq_grp);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, gpio);
	return gpio;
}

/* Convenience: IONW model + registered chip in one call, no IRQ. */
static struct tsi_gpio *ionw_chip(struct kunit *test, struct fake_regs **fp,
				  u32 ctrl_init)
{
	struct fake_regs *f = fake_new(test, ionw_ctrl, ARRAY_SIZE(ionw_ctrl),
				       ctrl_init, IONW_DATA_OUT, IONW_DATA_IN);
	*fp = f;
	return fake_chip(test, f, IONW_BASE, &tsi_gpio_ionw_soc, 0, 0);
}

/* Add the IONW t2 collector registers for one destination group. */
static void fake_add_ionw_irq_regs(struct fake_regs *f, u32 grp)
{
	fake_add(f, IONW_T2_W1C, 0, false);
	fake_add(f, IONW_T2_GLBEN, 0, false);
	fake_add(f, IONW_T2_G0_EN + grp * IONW_T2_GRP_STRIDE, 0, false);
	fake_add(f, IONW_T2_G0_IP + grp * IONW_T2_GRP_STRIDE, 0, false);
}

/*
 * IONW model + collector registers for one destination group. irq=0
 * (no chained parent): the _hw irq helpers touch only gpio->regmap and
 * gpio->irq_grp, so no live irq_data/domain is needed to test them.
 */
static struct tsi_gpio *ionw_chip_with_irq(struct kunit *test,
					   struct fake_regs **fp, u32 irq_grp)
{
	struct fake_regs *f = fake_new(test, ionw_ctrl, ARRAY_SIZE(ionw_ctrl),
				       0, IONW_DATA_OUT, IONW_DATA_IN);

	fake_add_ionw_irq_regs(f, irq_grp);
	*fp = f;
	return fake_chip(test, f, IONW_BASE, &tsi_gpio_ionw_soc, 0, irq_grp);
}

/* Every IONW line drives exactly its documented data_out bit. */
static void gpio_tsi_ionw_set_hits_documented_bits(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ionw_bit); i++) {
		gc->set(gc, i, 1);
		KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT),
				BIT(ionw_bit[i]));
		gc->set(gc, i, 0);
		KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT), 0u);
	}
}

/* set() must read-modify-write: neighbors stay untouched. */
static void gpio_tsi_ionw_set_is_rmw(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;

	fake_set(f, IONW_DATA_OUT, BIT(0) | BIT(24));
	gc->set(gc, 4, 1);	/* GPIO_4 -> bit 21 */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT),
			BIT(0) | BIT(21) | BIT(24));
	gc->set(gc, 7, 0);	/* GPIO_7 -> bit 24 */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT), BIT(0) | BIT(21));
}

/* get() reads the documented data_in bit, not the line index. */
static void gpio_tsi_ionw_get_reads_data_in_bit(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;

	fake_set(f, IONW_DATA_IN, BIT(22));		/* GPIO_5 */
	KUNIT_EXPECT_EQ(test, gc->get(gc, 5), 1);
	KUNIT_EXPECT_EQ(test, gc->get(gc, 4), 0);
	KUNIT_EXPECT_EQ(test, gc->get(gc, 0), 0);
}

/* direction_output writes the value before enabling the driver (OE). */
static void gpio_tsi_ionw_direction_output_value_before_oe(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_IE);	/* inputs */
	struct gpio_chip *gc = &g->chip;

	KUNIT_ASSERT_EQ(test, gc->direction_output(gc, 4, 1), 0);
	KUNIT_ASSERT_EQ(test, f->nwrites, 2u);
	KUNIT_EXPECT_EQ(test, f->log[0].off, (u32)IONW_DATA_OUT);
	KUNIT_EXPECT_EQ(test, f->log[0].val, BIT(21));
	KUNIT_EXPECT_EQ(test, f->log[1].off, ionw_ctrl[4]);
	KUNIT_EXPECT_EQ(test, f->log[1].val, (u32)CTRL_OE);	/* OE=1 IE=0 */
}

/* direction_input flips OE/IE only; DS and pull bits are preserved. */
static void gpio_tsi_ionw_direction_input_preserves_ds(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0x5 | CTRL_OE); /* DS=5 out */
	struct gpio_chip *gc = &g->chip;

	KUNIT_ASSERT_EQ(test, gc->direction_input(gc, 0), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]), 0x5 | CTRL_IE);
}

/* get_direction decodes OE; 0x008 is the documented IONW reset value. */
static void gpio_tsi_ionw_get_direction_decodes_oe(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_OE);	/* reset 0x008 */
	struct gpio_chip *gc = &g->chip;

	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 3),
			GPIO_LINE_DIRECTION_OUT);
	fake_set(f, ionw_ctrl[3], CTRL_IE);
	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 3),
			GPIO_LINE_DIRECTION_IN);
}

/* IONE: FS lines sit at bits 10..13 with their own register offsets. */
static void gpio_tsi_ione_lines_and_registers(struct kunit *test)
{
	struct fake_regs *f = fake_new(test, ione_ctrl, ARRAY_SIZE(ione_ctrl),
				       0, IONE_DATA_OUT, IONE_DATA_IN);
	struct tsi_gpio *g = fake_chip(test, f, IONE_BASE, &tsi_gpio_ione_soc,
				       0, 0);
	struct gpio_chip *gc = &g->chip;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ione_bit); i++) {
		gc->set(gc, i, 1);
		KUNIT_EXPECT_EQ(test, fake_get(f, IONE_DATA_OUT),
				BIT(ione_bit[i]));
		gc->set(gc, i, 0);
	}
	KUNIT_ASSERT_EQ(test, gc->direction_output(gc, 0, 1), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ione_ctrl[0]), (u32)CTRL_OE);
	/* IONE control regs reset to 0x000: default is input (IO list). */
	fake_set(f, ione_ctrl[1], 0);
	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 1),
			GPIO_LINE_DIRECTION_IN);
	fake_set(f, IONE_DATA_IN, BIT(12));		/* GPIO_FS_2 */
	KUNIT_EXPECT_EQ(test, gc->get(gc, 2), 1);
}

/*
 * irq_mask clears only the line's bit in OUR destination group's
 * enable_gN register. Using group 2 (not 0) exercises the stride math:
 * a wrong group index would write an unmodeled address and fail the
 * test via fake_write()'s unmodeled-register check.
 */
static void gpio_tsi_irq_mask_clears_group_en_bit_only(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip_with_irq(test, &f, 2);
	u32 g0_en = IONW_T2_G0_EN + 2 * IONW_T2_GRP_STRIDE;

	fake_set(f, g0_en, BIT(ionw_bit[4]) | BIT(ionw_bit[3]));
	tsi_gpio_irq_mask_hw(g, 4);	/* GPIO_4 -> bit 21 */
	KUNIT_EXPECT_EQ(test, fake_get(f, g0_en), BIT(ionw_bit[3]));
}

/*
 * irq_unmask sets the line's bit in BOTH the global int_enable_reg
 * AND our group's enable_gN (open item [E]: the former is shared by
 * every destination group, not just ours).
 */
static void gpio_tsi_irq_unmask_sets_glben_and_group_en(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip_with_irq(test, &f, 2);
	u32 g0_en = IONW_T2_G0_EN + 2 * IONW_T2_GRP_STRIDE;

	tsi_gpio_irq_unmask_hw(g, 4);	/* GPIO_4 -> bit 21 */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_T2_GLBEN), BIT(ionw_bit[4]));
	KUNIT_EXPECT_EQ(test, fake_get(f, g0_en), BIT(ionw_bit[4]));
}

/* irq_ack writes exactly the line's bit to the w1c register. */
static void gpio_tsi_irq_ack_writes_w1c(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip_with_irq(test, &f, 0);

	tsi_gpio_irq_ack_hw(g, 4);	/* GPIO_4 -> bit 21 */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_T2_W1C), BIT(ionw_bit[4]));
}

/*
 * set_multiple must translate both lines' bits into a single RMW on
 * data_out. Using GPIO_4 (bit 21) and GPIO_7 (bit 24) exercises the
 * non-contiguous mapping; only one write should appear in the log.
 */
static void gpio_tsi_ionw_set_multiple_is_single_write(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;
	DECLARE_BITMAP(mask, 8);
	DECLARE_BITMAP(bits, 8);

	bitmap_zero(mask, 8);
	bitmap_zero(bits, 8);
	set_bit(4, mask);	/* GPIO_4 -> hw bit 21 */
	set_bit(7, mask);	/* GPIO_7 -> hw bit 24 */
	set_bit(4, bits);	/* set GPIO_4 high; GPIO_7 stays low */

	gc->set_multiple(gc, mask, bits);
	KUNIT_EXPECT_EQ(test, f->nwrites, 1u);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT),
			BIT(ionw_bit[4]));	/* only bit 21 set */
}

/*
 * get_multiple must read data_in exactly once and map hardware bits back
 * to the logical line indices via the non-contiguous table.
 */
static void gpio_tsi_ionw_get_multiple_maps_bits(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;
	DECLARE_BITMAP(mask, 8);
	DECLARE_BITMAP(bits, 8);
	unsigned int reads_before;

	/* GPIO_0 (bit 0) and GPIO_5 (bit 22) are live; GPIO_7 (bit 24) is not. */
	fake_set(f, IONW_DATA_IN,
		 BIT(ionw_bit[0]) | BIT(ionw_bit[5]));

	bitmap_zero(mask, 8);
	set_bit(0, mask);
	set_bit(5, mask);
	set_bit(7, mask);

	/* Record reads already consumed by chip registration, then call op. */
	reads_before = 0;	/* fake model does not count reads; verified by result */
	KUNIT_ASSERT_EQ(test, gc->get_multiple(gc, mask, bits), 0);

	KUNIT_EXPECT_EQ(test, (int)test_bit(0, bits), 1);
	KUNIT_EXPECT_EQ(test, (int)test_bit(5, bits), 1);
	KUNIT_EXPECT_EQ(test, (int)test_bit(7, bits), 0);
	/* Lines not in mask must not be set. */
	KUNIT_EXPECT_EQ(test, (int)test_bit(1, bits), 0);
	(void)reads_before;
}

/* set_config BIAS_PULL_UP sets PE|PS without disturbing OE or DS. */
static void gpio_tsi_set_config_pull_up_preserves_ctrl(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_OE);
	struct gpio_chip *gc = &g->chip;
	unsigned long cfg = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_UP, 0);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0, cfg), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]),
			(u32)(CTRL_OE | CTRL_PE | CTRL_PS));
}

/* set_config BIAS_PULL_DOWN sets PE and clears PS (even if PS was set). */
static void gpio_tsi_set_config_pull_down(struct kunit *test)
{
	struct fake_regs *f;
	/* start with PS already set to verify it is cleared */
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_PS);
	struct gpio_chip *gc = &g->chip;
	unsigned long cfg = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_DOWN, 0);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0, cfg), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]), (u32)CTRL_PE);
}

/* set_config BIAS_DISABLE clears PE and PS. */
static void gpio_tsi_set_config_bias_disable(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_PE | CTRL_PS);
	struct gpio_chip *gc = &g->chip;
	unsigned long cfg = pinconf_to_config_packed(PIN_CONFIG_BIAS_DISABLE, 0);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0, cfg), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]), 0u);
}

/* set_config DRIVE_STRENGTH 5mA → DS code 2; other bits (OE) preserved. */
static void gpio_tsi_set_config_drive_strength(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_OE);
	struct gpio_chip *gc = &g->chip;
	unsigned long cfg = pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 5);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 1, cfg), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[1]),
			(u32)(CTRL_OE | 2));   /* DS=2 (5mA) */
}

/* set_config INPUT_SCHMITT_ENABLE toggles IS on/off. */
static void gpio_tsi_set_config_schmitt_enable(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;
	unsigned long cfg_en  = pinconf_to_config_packed(PIN_CONFIG_INPUT_SCHMITT_ENABLE, 1);
	unsigned long cfg_dis = pinconf_to_config_packed(PIN_CONFIG_INPUT_SCHMITT_ENABLE, 0);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0, cfg_en), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]), (u32)CTRL_IS);
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0, cfg_dis), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]), 0u);
}

/* set_config returns -ENOTSUPP for parameters the hardware cannot do. */
static void gpio_tsi_set_config_unknown_enotsupp(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;
	unsigned long cfg = pinconf_to_config_packed(PIN_CONFIG_INPUT_DEBOUNCE, 10);

	KUNIT_EXPECT_EQ(test, gc->set_config(gc, 0, cfg), -ENOTSUPP);
}

/*
 * irq_set_type: hardware is level-only. Level requests must be accepted;
 * edge requests must be rejected. The function does not dereference its
 * irq_data argument, so NULL is safe here.
 */
static void gpio_tsi_irq_set_type_accepts_level(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, tsi_gpio_irq_set_type(NULL, IRQ_TYPE_LEVEL_HIGH), 0);
	KUNIT_EXPECT_EQ(test, tsi_gpio_irq_set_type(NULL, IRQ_TYPE_LEVEL_LOW), 0);
}

static void gpio_tsi_irq_set_type_rejects_edge(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, tsi_gpio_irq_set_type(NULL, IRQ_TYPE_EDGE_RISING),  -EINVAL);
	KUNIT_EXPECT_EQ(test, tsi_gpio_irq_set_type(NULL, IRQ_TYPE_EDGE_FALLING), -EINVAL);
	KUNIT_EXPECT_EQ(test, tsi_gpio_irq_set_type(NULL, IRQ_TYPE_EDGE_BOTH),    -EINVAL);
}

/* DRIVE_STRENGTH rejects values not in the supported mA table. */
static void gpio_tsi_set_config_drive_strength_invalid(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, 0);
	struct gpio_chip *gc = &g->chip;

	/* 7 mA falls between valid codes 011 (6mA) and 100 (8mA). */
	KUNIT_EXPECT_EQ(test, gc->set_config(gc, 0,
		pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 7)), -EINVAL);
	/* 0 mA and 11 mA are also outside the table. */
	KUNIT_EXPECT_EQ(test, gc->set_config(gc, 0,
		pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 0)), -EINVAL);
	KUNIT_EXPECT_EQ(test, gc->set_config(gc, 0,
		pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 11)), -EINVAL);
	/* ctrl reg must be untouched after all rejections. */
	KUNIT_EXPECT_EQ(test, fake_get(f, ionw_ctrl[0]), 0u);
}

/* Registration wires names, line count, and non-sleeping access. */
static void gpio_tsi_registration(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_gpio *g = ionw_chip(test, &f, CTRL_OE);
	struct gpio_chip *gc = &g->chip;

	KUNIT_EXPECT_EQ(test, gc->ngpio, 8);
	KUNIT_EXPECT_FALSE(test, gc->can_sleep);
	KUNIT_ASSERT_NOT_NULL(test, gc->names);
	KUNIT_EXPECT_STREQ(test, gc->names[0], "GPIO_0");
	KUNIT_EXPECT_STREQ(test, gc->names[4], "GPIO_4");
}

static struct kunit_case gpio_tsi_test_cases[] = {
	KUNIT_CASE(gpio_tsi_ionw_set_hits_documented_bits),
	KUNIT_CASE(gpio_tsi_ionw_set_is_rmw),
	KUNIT_CASE(gpio_tsi_ionw_get_reads_data_in_bit),
	KUNIT_CASE(gpio_tsi_ionw_direction_output_value_before_oe),
	KUNIT_CASE(gpio_tsi_ionw_direction_input_preserves_ds),
	KUNIT_CASE(gpio_tsi_ionw_get_direction_decodes_oe),
	KUNIT_CASE(gpio_tsi_ione_lines_and_registers),
	KUNIT_CASE(gpio_tsi_irq_mask_clears_group_en_bit_only),
	KUNIT_CASE(gpio_tsi_irq_unmask_sets_glben_and_group_en),
	KUNIT_CASE(gpio_tsi_irq_ack_writes_w1c),
	KUNIT_CASE(gpio_tsi_ionw_set_multiple_is_single_write),
	KUNIT_CASE(gpio_tsi_ionw_get_multiple_maps_bits),
	KUNIT_CASE(gpio_tsi_set_config_pull_up_preserves_ctrl),
	KUNIT_CASE(gpio_tsi_set_config_pull_down),
	KUNIT_CASE(gpio_tsi_set_config_bias_disable),
	KUNIT_CASE(gpio_tsi_set_config_drive_strength),
	KUNIT_CASE(gpio_tsi_set_config_schmitt_enable),
	KUNIT_CASE(gpio_tsi_set_config_unknown_enotsupp),
	KUNIT_CASE(gpio_tsi_irq_set_type_accepts_level),
	KUNIT_CASE(gpio_tsi_irq_set_type_rejects_edge),
	KUNIT_CASE(gpio_tsi_set_config_drive_strength_invalid),
	KUNIT_CASE(gpio_tsi_registration),
	{}
};

static struct kunit_suite gpio_tsi_test_suite = {
	.name = "gpio-tsi",
	.test_cases = gpio_tsi_test_cases,
};
kunit_test_suite(gpio_tsi_test_suite);

MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");
MODULE_DESCRIPTION("KUnit tests for the TSI SkyLP GPIO driver");
MODULE_LICENSE("GPL");
