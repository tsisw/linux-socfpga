// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the TSI SkyLP pinctrl/GPIO driver.
 *
 * Two kinds of test live here:
 *
 *  - Table integrity. The pad and group tables in the driver are
 *    generated from ral.json, so the expected offsets, data bits and
 *    iomode encodings are repeated here as independent literals rather
 *    than read back through the driver's own tables. A generator bug or
 *    a bad hand-edit then shows up as a failure instead of verifying
 *    itself.
 *
 *  - Behaviour, driven through the real pinctrl/gpio_chip ops against a
 *    fake register model: pad-to-bit mapping, group-wide muxing, OE/IE
 *    direction encoding, write ordering and pin configuration.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>

#include "pinctrl-tsi.h"

#define FAKE_MAX_REGS	64
#define FAKE_MAX_WRITES	16

/* Corner bases: RAL address minus the 0x2000_0000 CSR window origin. */
#define IONW_BASE	0x13000000
#define IONE_BASE	0x6000000
#define IOSE_BASE	0x9000000
#define IOSW_BASE	0x19000000

/* Independent expectations (ral.json SKYLP_G0829). */
#define IONW_DATA_OUT	(IONW_BASE + 0x178)
#define IONW_DATA_IN	(IONW_BASE + 0x17c)
#define IONW_SPI_IOMODE	(IONW_BASE + 0x18c)
#define IONW_PCIE_IOMODE (IONW_BASE + 0x188)
#define IONW_PRST_IOMODE (IONW_BASE + 0x180)

#define IONE_DATA_OUT	(IONE_BASE + 0x188)
#define IONE_DATA_IN	(IONE_BASE + 0x18c)
#define IONE_SMB_IOMODE	(IONE_BASE + 0x16c)
#define IONE_UART0_IOMODE (IONE_BASE + 0x178)

/* Pad control field bits, as independent literals. */
#define CTRL_DS		0x07	/* bits[2:0] */
#define CTRL_OE		0x08	/* bit[3] */
#define CTRL_IE		0x10	/* bit[4] */
#define CTRL_IS		0x20	/* bit[5]: 1 = Schmitt trigger */
#define CTRL_PE		0x40	/* bit[6]: 1 = pull enable */
#define CTRL_PS		0x100	/* bit[8]: 0 = pull-down, 1 = pull-up */

/*
 * Interrupt collector offsets, as independent literals. IONW uses the
 * t2 collector, IONE intr_regs_8, IOSE intr_regs, IOSW intr_regs_2.
 * ip_status/enable repeat every GRP_STRIDE bytes per destination group.
 */
#define GRP_STRIDE		0xc
#define IONW_T2_W1C		(IONW_BASE + 0x34c4)
#define IONW_T2_GLBEN		(IONW_BASE + 0x34cc)
#define IONW_T2_IP_G0		(IONW_BASE + 0x34d4)
#define IONW_T2_EN_G0		(IONW_BASE + 0x34d8)
#define IOSE_W1C		(IOSE_BASE + 0x2044)
#define IOSE_GLBEN		(IOSE_BASE + 0x204c)
#define IOSE_EN_G0		(IOSE_BASE + 0x2058)

/* DS[2:0] code -> nominal mA, as independent literals. */
static const u8 tsi_test_ds_ma[] = { 3, 4, 5, 6, 8, 10 };

/* Every corner, for the exhaustive per-pin sweeps. */
struct tsi_corner_ref {
	const struct tsi_pinctrl_soc	*soc;
	u32				base;
};

/*
 * Pad control offsets and pin indices used by the table-integrity
 * tests, as test-local literals from ral.json SKYLP_G0829. These are
 * deliberately NOT the TSI_SKYLP_* header defines: the driver tables
 * are built from that header, so asserting against it would only prove
 * the driver equals itself. Keeping a second, independently transcribed
 * copy here is what lets a bad header edit fail the suite.
 */
#define RAL_IONW_GPIO0_CTRL	0x11c
#define RAL_IONW_GPIO4_CTRL	0x12c
#define RAL_IONW_GPIO7_CTRL	0x138
#define RAL_IONW_SPI_IO0_CTRL	0x144
#define RAL_IONW_SPI_SCS_CTRL	0x13c
#define RAL_IONW_PRST2_CTRL	0x10c
#define RAL_IONE_I2C_SMB_SCL0_CTRL	0x134
#define RAL_IONE_GPIO_FS0_CTRL	0x124
#define RAL_IONE_GPIO_FS3_CTRL	0x130
#define RAL_IONE_UART_RTS_1_CTRL	0x11c
#define RAL_IONE_UART_CTS_1_CTRL	0x120
#define RAL_IOSE_PRST0_CTRL	0x120
#define RAL_IOSE_PRST1_CTRL	0x124
#define RAL_IOSW_I3C_SCL0_CTRL	0x14c
#define RAL_IOSW_I3C_SDA2_CTRL	0x160
#define RAL_IOSW_I2S_SCK_CTRL	0x164
#define RAL_IOSW_I2S_WS_CTRL	0x16c
#define RAL_IOSW_I2S_SDI0_CTRL	0x168
#define RAL_IOSW_I2S_SDO1_CTRL	0x178
#define RAL_IOSW_I2S_SDI1_CTRL	0x174

/* GPIO line (== data bit) of pads named in behavioural tests. */
#define IONE_PIN_UART_RTS_1	14
#define IONE_PIN_UART_RX_0	15

/* iomode values that select GPIO, per RAL field descriptions. */
#define IONW_SPI_GPIO		5
#define IONW_PCIE_GPIO		1
#define IONW_PERST01_GPIO	2
#define IONE_SMB_GPIO		2
#define IONE_UART_GPIO		1
#define IOSW_I2S_GPIO		1

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

static struct fake_reg *fake_find(struct fake_regs *f, u32 off)
{
	unsigned int i;

	for (i = 0; i < f->nregs; i++)
		if (f->regs[i].off == off)
			return &f->regs[i];
	return NULL;
}

/* Touching a register outside the modeled set is itself a driver bug. */
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

static void fake_add(struct fake_regs *f, u32 off, u32 val, bool ro)
{
	KUNIT_ASSERT_LT(f->test, f->nregs, (unsigned int)FAKE_MAX_REGS);
	f->regs[f->nregs++] = (struct fake_reg){ .off = off, .val = val,
						 .ro = ro };
}

static u32 fake_get(struct fake_regs *f, u32 off)
{
	struct fake_reg *r = fake_find(f, off);

	KUNIT_ASSERT_NOT_NULL(f->test, r);
	return r->val;
}

static void fake_set(struct fake_regs *f, u32 off, u32 val)
{
	struct fake_reg *r = fake_find(f, off);

	KUNIT_ASSERT_NOT_NULL(f->test, r);
	r->val = val;
}

/*
 * Model every register one corner's driver instance can touch: each
 * pad's control register (from the driver's own table - the offsets
 * themselves are checked by the table-integrity tests), the data pair,
 * and every group's iomode register.
 */
static struct fake_regs *fake_new(struct kunit *test, u32 base,
				  const struct tsi_pinctrl_soc *soc,
				  u32 ctrl_init)
{
	struct fake_regs *f;
	unsigned int i;

	f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, f);
	f->test = test;

	for (i = 0; i < soc->npins; i++)
		fake_add(f, base + soc->pins[i].ctrl_off, ctrl_init, false);

	fake_add(f, base + soc->data_out_off, 0, false);
	fake_add(f, base + soc->data_in_off, 0, true);

	for (i = 0; i < soc->ngroups; i++)
		if (soc->groups[i].iomode_off)
			fake_add(f, base + soc->groups[i].iomode_off, 0, false);

	/* Interrupt collector: ack, global enable, and all four groups. */
	fake_add(f, base + soc->irq_w1c_off, 0, false);
	fake_add(f, base + soc->irq_glben_off, 0, false);
	for (i = 0; i < 4; i++) {
		fake_add(f, base + soc->irq_grp0_ip_off + i * GRP_STRIDE,
			 0, false);
		fake_add(f, base + soc->irq_grp0_en_off + i * GRP_STRIDE,
			 0, false);
	}

	return f;
}

/* Register a real driver instance (pinctrl + gpiochip) over the model. */
static struct tsi_pinctrl *fake_corner_irq(struct kunit *test,
					  struct fake_regs **fp, u32 base,
					  const struct tsi_pinctrl_soc *soc,
					  u32 ctrl_init, int irq, u32 irq_grp)
{
	static const struct regmap_config cfg = {
		.reg_bits	= 32,
		.reg_stride	= 4,
		.val_bits	= 32,
		.reg_read	= fake_read,
		.reg_write	= fake_write,
		.max_register	= 0x1a000000,
		.fast_io	= true,
	};
	struct fake_regs *f = fake_new(test, base, soc, ctrl_init);
	struct tsi_pinctrl *tp;
	struct device *dev;
	struct regmap *map;

	/*
	 * Name the device after the corner: kunit_device_register() creates
	 * a driver of this name, so a test registering several corners at
	 * once needs a distinct name per instance.
	 */
	dev = kunit_device_register(test, soc->label);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	map = devm_regmap_init(dev, NULL, f, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, map);
	tp = tsi_pinctrl_register(dev, map, base, soc, irq, irq_grp);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, tp);

	*fp = f;
	return tp;
}

/*
 * Most tests want the dormant (no parent IRQ) instance: the irqchip's
 * register-level helpers are exercised directly, which is why they are
 * split out of the irq_chip callbacks in the first place. Standing up a
 * live irq domain would need a real parent interrupt.
 */
static struct tsi_pinctrl *fake_corner(struct kunit *test,
				       struct fake_regs **fp, u32 base,
				       const struct tsi_pinctrl_soc *soc,
				       u32 ctrl_init)
{
	return fake_corner_irq(test, fp, base, soc, ctrl_init, 0, 0);
}

/*
 * Resolve a group or function NAME to the selector index the pinctrl
 * core would pass in. Done through the driver's own enumeration ops, so
 * a test drives exactly the path the core does rather than indexing the
 * tables behind its back.
 */
static int grp_sel(struct kunit *test, struct tsi_pinctrl *tp, const char *name)
{
	int i, n = tsi_pinctrl_ops.get_groups_count(tp->pctl);

	for (i = 0; i < n; i++)
		if (!strcmp(tsi_pinctrl_ops.get_group_name(tp->pctl, i), name))
			return i;

	KUNIT_FAIL(test, "no group named %s", name);
	return -1;
}

static int func_sel(struct kunit *test, struct tsi_pinctrl *tp,
		    const char *name)
{
	int i, n = tsi_pinmux_ops.get_functions_count(tp->pctl);

	for (i = 0; i < n; i++)
		if (!strcmp(tsi_pinmux_ops.get_function_name(tp->pctl, i), name))
			return i;

	KUNIT_FAIL(test, "no function named %s", name);
	return -1;
}

/* set_mux() as the core calls it: by function and group selector. */
static int do_set_mux(struct kunit *test, struct tsi_pinctrl *tp,
		      const char *func, const char *group)
{
	int f = func_sel(test, tp, func);
	int g = grp_sel(test, tp, group);

	KUNIT_ASSERT_GE(test, f, 0);
	KUNIT_ASSERT_GE(test, g, 0);
	return tsi_pinmux_ops.set_mux(tp->pctl, f, g);
}

/* ---------------------- table integrity (generated) ------------------ */

/*
 * Pad count per corner is the width of that corner's gpio_data
 * register: IONW [25:0], IONE [24:0], IOSE [1:0], IOSW [11:0].
 */
static void tsi_pin_counts(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, tsi_skylp_ionw_soc.npins, 26u);
	KUNIT_EXPECT_EQ(test, tsi_skylp_ione_soc.npins, 25u);
	KUNIT_EXPECT_EQ(test, tsi_skylp_iose_soc.npins, 2u);
	KUNIT_EXPECT_EQ(test, tsi_skylp_iosw_soc.npins, 12u);
}

/*
 * Spot-check pads whose data bit is deliberately NOT their pad-address
 * order: IONW GPIO_4..7 are bits 21..24 even though their control
 * registers (0x12c..0x138) sit right after GPIO_0..3 (0x11c..0x128).
 */
static void tsi_ionw_pad_offsets(struct kunit *test)
{
	const struct tsi_pinctrl_soc *s = &tsi_skylp_ionw_soc;

	KUNIT_EXPECT_STREQ(test, s->pins[0].name, "GPIO_0");
	KUNIT_EXPECT_EQ(test, s->pins[0].ctrl_off, RAL_IONW_GPIO0_CTRL);
	KUNIT_EXPECT_STREQ(test, s->pins[21].name, "GPIO_4");
	KUNIT_EXPECT_EQ(test, s->pins[21].ctrl_off, RAL_IONW_GPIO4_CTRL);
	KUNIT_EXPECT_STREQ(test, s->pins[24].name, "GPIO_7");
	KUNIT_EXPECT_EQ(test, s->pins[24].ctrl_off, RAL_IONW_GPIO7_CTRL);
	/* SPI block occupies bits 9..20, not 4..15. */
	KUNIT_EXPECT_STREQ(test, s->pins[9].name, "SPI_IO0");
	KUNIT_EXPECT_EQ(test, s->pins[9].ctrl_off, RAL_IONW_SPI_IO0_CTRL);
	KUNIT_EXPECT_STREQ(test, s->pins[20].name, "SPI_SCS");
	KUNIT_EXPECT_EQ(test, s->pins[20].ctrl_off, RAL_IONW_SPI_SCS_CTRL);
	/* PERST_2 is bit 25, above the GPIO_4..7 block. */
	KUNIT_EXPECT_STREQ(test, s->pins[25].name, "TLP_PERST_2");
	KUNIT_EXPECT_EQ(test, s->pins[25].ctrl_off, RAL_IONW_PRST2_CTRL);
	KUNIT_EXPECT_EQ(test, s->data_out_off, 0x178u);
	KUNIT_EXPECT_EQ(test, s->data_in_off, 0x17cu);
}

/* IONE: the fail-safe GPIO pads are bits 10..13, not 0..3. */
static void tsi_ione_pad_offsets(struct kunit *test)
{
	const struct tsi_pinctrl_soc *s = &tsi_skylp_ione_soc;

	KUNIT_EXPECT_STREQ(test, s->pins[0].name, "I2C_SMB_SCL_0");
	KUNIT_EXPECT_EQ(test, s->pins[0].ctrl_off, RAL_IONE_I2C_SMB_SCL0_CTRL);
	KUNIT_EXPECT_STREQ(test, s->pins[10].name, "GPIO_FS_0");
	KUNIT_EXPECT_EQ(test, s->pins[10].ctrl_off, RAL_IONE_GPIO_FS0_CTRL);
	KUNIT_EXPECT_STREQ(test, s->pins[13].name, "GPIO_FS_3");
	KUNIT_EXPECT_EQ(test, s->pins[13].ctrl_off, RAL_IONE_GPIO_FS3_CTRL);
	/* UART_RTS_1 is bit 14, split from the rest of UART1 (19,20,24). */
	KUNIT_EXPECT_STREQ(test, s->pins[14].name, "UART_RTS_1");
	KUNIT_EXPECT_EQ(test, s->pins[14].ctrl_off, RAL_IONE_UART_RTS_1_CTRL);
	KUNIT_EXPECT_STREQ(test, s->pins[24].name, "UART_CTS_1");
	KUNIT_EXPECT_EQ(test, s->pins[24].ctrl_off, RAL_IONE_UART_CTS_1_CTRL);
	KUNIT_EXPECT_EQ(test, s->data_out_off, 0x188u);
}

/* Every pad must be named and have a real control register. */
static void tsi_all_pads_populated(struct kunit *test)
{
	const struct tsi_pinctrl_soc *socs[] = {
		&tsi_skylp_ionw_soc, &tsi_skylp_ione_soc,
		&tsi_skylp_iose_soc, &tsi_skylp_iosw_soc,
	};
	unsigned int s, i;

	for (s = 0; s < ARRAY_SIZE(socs); s++) {
		for (i = 0; i < socs[s]->npins; i++) {
			KUNIT_ASSERT_NOT_NULL(test, socs[s]->pins[i].name);
			KUNIT_EXPECT_NE_MSG(test, socs[s]->pins[i].ctrl_off, 0,
					    "%s pin %u has no control reg",
					    socs[s]->label, i);
		}
	}
}

/*
 * A pad may belong to at most one group: tsi_pinctrl_group_of_pin()
 * returns the first match, so an overlap would silently mux the wrong
 * register. Also checks every group pin is a valid pad index.
 */
static void tsi_groups_partition_pins(struct kunit *test)
{
	const struct tsi_pinctrl_soc *socs[] = {
		&tsi_skylp_ionw_soc, &tsi_skylp_ione_soc,
		&tsi_skylp_iose_soc, &tsi_skylp_iosw_soc,
	};
	unsigned int s, g, i;

	for (s = 0; s < ARRAY_SIZE(socs); s++) {
		const struct tsi_pinctrl_soc *soc = socs[s];
		unsigned long seen = 0;

		KUNIT_ASSERT_LE(test, soc->npins, BITS_PER_LONG);

		for (g = 0; g < soc->ngroups; g++) {
			const struct tsi_group *grp = &soc->groups[g];

			for (i = 0; i < grp->npins; i++) {
				unsigned int pin = grp->pins[i];

				KUNIT_ASSERT_LT_MSG(test, pin, soc->npins,
						    "%s/%s pin %u out of range",
						    soc->label, grp->name, pin);
				KUNIT_EXPECT_FALSE_MSG(test, seen & BIT(pin),
						       "%s pin %u in two groups",
						       soc->label, pin);
				seen |= BIT(pin);
			}
		}
	}
}

/*
 * Every muxable group must offer exactly one GPIO option, otherwise
 * gpio_request_enable() has nothing to select (or an ambiguous choice).
 */
static void tsi_groups_have_one_gpio_opt(struct kunit *test)
{
	const struct tsi_pinctrl_soc *socs[] = {
		&tsi_skylp_ionw_soc, &tsi_skylp_ione_soc,
		&tsi_skylp_iose_soc, &tsi_skylp_iosw_soc,
	};
	unsigned int s, g, o;

	for (s = 0; s < ARRAY_SIZE(socs); s++) {
		for (g = 0; g < socs[s]->ngroups; g++) {
			const struct tsi_group *grp = &socs[s]->groups[g];
			unsigned int ngpio = 0;

			if (!grp->iomode_off) {
				/* Dedicated group: no mux options at all. */
				KUNIT_EXPECT_EQ(test, grp->nopts, 0u);
				continue;
			}

			KUNIT_EXPECT_NE(test, grp->width, 0);
			for (o = 0; o < grp->nopts; o++) {
				if (!strcmp(grp->opts[o].func, "gpio"))
					ngpio++;
				/* Value must fit the iomode field. */
				KUNIT_EXPECT_LT_MSG(test, grp->opts[o].val,
						    1 << grp->width,
						    "%s/%s opt %s overflows",
						    socs[s]->label, grp->name,
						    grp->opts[o].func);
			}
			KUNIT_EXPECT_EQ_MSG(test, ngpio, 1u,
					    "%s/%s has %u gpio options",
					    socs[s]->label, grp->name, ngpio);
		}
	}
}

/* iomode GPIO encodings, as independent literals from the RAL text. */
static void tsi_iomode_encodings(struct kunit *test)
{
	const struct tsi_group *g;

	/* IONW spi group: 3-bit field at 0x18c, GPIO == 0b101. */
	g = &tsi_skylp_ionw_soc.groups[4];
	KUNIT_EXPECT_STREQ(test, g->name, "spi");
	KUNIT_EXPECT_EQ(test, g->iomode_off, 0x18cu);
	KUNIT_EXPECT_EQ(test, g->width, 3);
	KUNIT_EXPECT_EQ(test, g->npins, 12u);
	KUNIT_EXPECT_STREQ(test, g->opts[g->nopts - 1].func, "gpio");
	KUNIT_EXPECT_EQ(test, g->opts[g->nopts - 1].val, IONW_SPI_GPIO);
	/* Reset default 0b010 must still be reachable as "qspi". */
	KUNIT_EXPECT_STREQ(test, g->opts[2].func, "qspi");
	KUNIT_EXPECT_EQ(test, g->opts[2].val, 2);

	/* IONE smb group: 2-bit field at 0x16c, GPIO == 0b10. */
	g = &tsi_skylp_ione_soc.groups[1];
	KUNIT_EXPECT_STREQ(test, g->name, "smb");
	KUNIT_EXPECT_EQ(test, g->iomode_off, 0x16cu);
	KUNIT_EXPECT_EQ(test, g->width, 2);
	KUNIT_EXPECT_EQ(test, g->npins, 8u);
	KUNIT_EXPECT_EQ(test, g->opts[g->nopts - 1].val, IONE_SMB_GPIO);
}

/* ------------------------- mux behaviour ---------------------------- */

/* A dedicated pad needs no mux write: GPIO is its only function. */
static void tsi_gpio_enable_dedicated_is_noop(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	f->nwrites = 0;
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_gpio_enable(tp, 0), 0);
	KUNIT_EXPECT_EQ(test, f->nwrites, 0u);
}

/* A muxed pad writes its group's iomode field with the GPIO value. */
static void tsi_gpio_enable_muxed_writes_iomode(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	/* Start from the hardware reset default (QSPI == 0b010). */
	fake_set(f, IONW_SPI_IOMODE, 2);
	f->nwrites = 0;

	/* SPI_IO0 is pin 9. */
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_gpio_enable(tp, 9), 0);
	KUNIT_EXPECT_EQ(test, f->nwrites, 1u);
	KUNIT_EXPECT_EQ(test, f->log[0].off, (u32)IONW_SPI_IOMODE);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_SPI_IOMODE), IONW_SPI_GPIO);
}

/*
 * Muxing is group-wide: requesting one SPI pad as GPIO moves all 12,
 * and requesting a second pad of the same group writes nothing new.
 * This is a hardware property, asserted so it cannot regress silently.
 */
static void tsi_gpio_enable_is_group_wide(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);
	const struct tsi_group *g9 = tsi_pinctrl_group_of_pin(tp, 9);
	const struct tsi_group *g20 = tsi_pinctrl_group_of_pin(tp, 20);

	/* SPI_IO0 (9) and SPI_SCS (20) share one iomode register. */
	KUNIT_ASSERT_NOT_NULL(test, g9);
	KUNIT_EXPECT_PTR_EQ(test, g9, g20);

	KUNIT_EXPECT_EQ(test, tsi_pinctrl_gpio_enable(tp, 9), 0);
	f->nwrites = 0;
	/* Already GPIO: regmap_update_bits elides the redundant write. */
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_gpio_enable(tp, 20), 0);
	KUNIT_EXPECT_EQ(test, f->nwrites, 0u);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_SPI_IOMODE), IONW_SPI_GPIO);
}

/* A narrow iomode field must not disturb neighbouring bits. */
static void tsi_mux_set_preserves_other_bits(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONE_BASE,
					     &tsi_skylp_ione_soc, CTRL_OE);
	const struct tsi_group *smb = tsi_pinctrl_group_of_pin(tp, 0);

	KUNIT_ASSERT_NOT_NULL(test, smb);
	/* Junk in bits above the 2-bit field must survive. */
	fake_set(f, IONE_SMB_IOMODE, 0xf0);
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_mux_set(tp, smb, IONE_SMB_GPIO), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONE_SMB_IOMODE),
			0xf0 | IONE_SMB_GPIO);
}

/* UART0 and UART1 are separate groups with separate iomode registers. */
static void tsi_uart_groups_are_independent(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONE_BASE,
					     &tsi_skylp_ione_soc, CTRL_OE);
	const struct tsi_group *u0 = tsi_pinctrl_group_of_pin(tp,
							IONE_PIN_UART_RX_0);
	const struct tsi_group *u1 =
		tsi_pinctrl_group_of_pin(tp, IONE_PIN_UART_RTS_1);

	KUNIT_ASSERT_NOT_NULL(test, u0);
	KUNIT_ASSERT_NOT_NULL(test, u1);
	KUNIT_EXPECT_PTR_NE(test, u0, u1);
	KUNIT_EXPECT_STREQ(test, u0->name, "uart0");
	KUNIT_EXPECT_STREQ(test, u1->name, "uart1");

	/* Moving UART0 to GPIO must leave UART1's register alone. */
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_gpio_enable(tp, IONE_PIN_UART_RX_0),
			0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONE_UART0_IOMODE), IONE_UART_GPIO);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONE_BASE + 0x17c), 0u);
}

/*
 * set_mux() driving a group to a PERIPHERAL function - the path a DT
 * pinmux node takes. Every other mux test moves a group towards GPIO,
 * so without this the five non-GPIO spi_iomode encodings are only ever
 * asserted as table data, never actually written.
 */
static void tsi_set_mux_selects_peripheral(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "qspi", "spi"), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_SPI_IOMODE), 2u);

	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "ospi", "spi"), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_SPI_IOMODE), 0u);

	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "armspi_spi", "spi"), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_SPI_IOMODE), 4u);

	/* and back to GPIO through the same entry point */
	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "gpio", "spi"), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_SPI_IOMODE), IONW_SPI_GPIO);
}

/* A three-value field: MDIO is a real alternative, not just GPIO. */
static void tsi_set_mux_selects_mdio(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "mdio", "perst01"), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_PRST_IOMODE), 1u);
	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "perst", "perst01"), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_PRST_IOMODE), 0u);
}

/*
 * A function that exists on the corner but not on the addressed group
 * must be refused, and must not write anything: the encodings are
 * per-group, so accepting it would program a value that means something
 * else entirely in that field.
 */
static void tsi_set_mux_rejects_foreign_function(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	f->nwrites = 0;
	/* "qspi" is an IONW function, but only the spi group offers it. */
	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "qspi", "pcie"), -EINVAL);
	KUNIT_EXPECT_EQ(test, f->nwrites, 0u);
}

/*
 * A dedicated group has no iomode register: selecting gpio on it is a
 * no-op that still succeeds (so a DT node may name it explicitly), and
 * anything else is refused rather than silently ignored.
 */
static void tsi_set_mux_on_dedicated_group(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	f->nwrites = 0;
	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "gpio", "gpio"), 0);
	KUNIT_EXPECT_EQ(test, f->nwrites, 0u);

	KUNIT_EXPECT_EQ(test, do_set_mux(test, tp, "qspi", "gpio"), -EINVAL);
	KUNIT_EXPECT_EQ(test, f->nwrites, 0u);
}

/*
 * The function list is derived from the groups' options at probe rather
 * than tabled, so check the derivation itself: the distinct names, and
 * that a function maps back to every group offering it.
 */
static void tsi_function_list_is_derived(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);
	const char * const *groups;
	unsigned int ngroups;
	int sel;

	/*
	 * IONW: gpio, pcie, perst, mdio, ospi, armspi_qspi, qspi,
	 * armspi_dspi, armspi_spi.
	 */
	KUNIT_EXPECT_EQ(test, tsi_pinmux_ops.get_functions_count(tp->pctl), 9);

	/* gpio is offered by all five groups (the dedicated one included). */
	sel = func_sel(test, tp, "gpio");
	KUNIT_ASSERT_GE(test, sel, 0);
	KUNIT_ASSERT_EQ(test, tsi_pinmux_ops.get_function_groups(tp->pctl, sel,
							&groups, &ngroups), 0);
	KUNIT_EXPECT_EQ(test, ngroups, 5u);

	/* perst spans two groups; qspi belongs to spi alone. */
	sel = func_sel(test, tp, "perst");
	KUNIT_ASSERT_GE(test, sel, 0);
	KUNIT_ASSERT_EQ(test, tsi_pinmux_ops.get_function_groups(tp->pctl, sel,
							&groups, &ngroups), 0);
	KUNIT_EXPECT_EQ(test, ngroups, 2u);

	sel = func_sel(test, tp, "qspi");
	KUNIT_ASSERT_GE(test, sel, 0);
	KUNIT_ASSERT_EQ(test, tsi_pinmux_ops.get_function_groups(tp->pctl, sel,
							&groups, &ngroups), 0);
	KUNIT_EXPECT_EQ(test, ngroups, 1u);
	KUNIT_EXPECT_STREQ(test, groups[0], "spi");
}

/*
 * The mux is group-wide, so the core must arbitrate: strict makes it
 * refuse a GPIO claim on a pad a peripheral owns, instead of letting
 * gpio_request_enable() drag the whole group to GPIO under a running
 * driver. Asserted as a flag because the enforcement lives in the
 * pinctrl core, not here.
 */
static void tsi_pinmux_is_strict(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, tsi_pinmux_ops.strict);
}

/*
 * Decoding a field value back to a function is what debugfs reports and
 * what makes an unexpected hardware state legible. Undefined encodings
 * must come back NULL rather than falling through to a neighbour.
 */
static void tsi_func_of_val_decodes(struct kunit *test)
{
	const struct tsi_group *spi = &tsi_skylp_ionw_soc.groups[4];
	const struct tsi_group *gpio = &tsi_skylp_ionw_soc.groups[0];

	KUNIT_ASSERT_STREQ(test, spi->name, "spi");
	KUNIT_EXPECT_STREQ(test, tsi_pinctrl_func_of_val(spi, 0), "ospi");
	KUNIT_EXPECT_STREQ(test, tsi_pinctrl_func_of_val(spi, 2), "qspi");
	KUNIT_EXPECT_STREQ(test, tsi_pinctrl_func_of_val(spi, IONW_SPI_GPIO),
			   "gpio");
	/* spi_iomode 110 and 111 are undefined in the RAL. */
	KUNIT_EXPECT_NULL(test, tsi_pinctrl_func_of_val(spi, 6));
	KUNIT_EXPECT_NULL(test, tsi_pinctrl_func_of_val(spi, 7));

	/* A dedicated group has no field, and is always gpio. */
	KUNIT_ASSERT_STREQ(test, gpio->name, "gpio");
	KUNIT_EXPECT_STREQ(test, tsi_pinctrl_func_of_val(gpio, 0), "gpio");
}

/* ------------------------- gpio_chip behaviour ----------------------- */

/* set() drives the pad's documented data_out bit, which is the pin. */
static void tsi_gpio_set_hits_data_bit(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);
	struct gpio_chip *gc = &tp->chip;

	gc->set(gc, 21, 1);		/* GPIO_4 -> bit 21 */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT), BIT(21));
	gc->set(gc, 24, 1);		/* GPIO_7 -> bit 24 */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT), BIT(21) | BIT(24));
	/* RMW: clearing one leaves the other set. */
	gc->set(gc, 21, 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT), BIT(24));
}

/* direction_output writes the level before raising OE (no stale drive). */
static void tsi_gpio_direction_output_value_before_oe(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONW_BASE + 0x12c;	/* GPIO_4 control */

	f->nwrites = 0;
	KUNIT_ASSERT_EQ(test, gc->direction_output(gc, 21, 1), 0);
	KUNIT_ASSERT_EQ(test, f->nwrites, 2u);
	KUNIT_EXPECT_EQ(test, f->log[0].off, (u32)IONW_DATA_OUT);
	KUNIT_EXPECT_EQ(test, f->log[1].off, ctrl);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_OE | CTRL_IE),
			(u32)CTRL_OE);
}

static void tsi_gpio_get_reads_data_in(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_IE);
	struct gpio_chip *gc = &tp->chip;

	fake_set(f, IONW_DATA_IN, BIT(23));	/* GPIO_6 */
	KUNIT_EXPECT_EQ(test, gc->get(gc, 23), 1);
	KUNIT_EXPECT_EQ(test, gc->get(gc, 22), 0);
}

static void tsi_gpio_get_direction_decodes_oe(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONE_BASE,
					     &tsi_skylp_ione_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONE_BASE + 0x124;	/* GPIO_FS_0 control */

	fake_set(f, ctrl, CTRL_OE);
	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 10),
			GPIO_LINE_DIRECTION_OUT);
	fake_set(f, ctrl, CTRL_IE);
	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 10),
			GPIO_LINE_DIRECTION_IN);
}

/*
 * set_multiple is one write, pin == bit means no remapping, and it is a
 * read-modify-write on a register shared by the whole corner. Lines
 * outside the mask are preloaded here so that losing them - or widening
 * the mask to the full word - fails; starting from a zeroed register
 * would hide both.
 */
static void tsi_gpio_set_multiple_single_write(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IOSW_BASE,
					     &tsi_skylp_iosw_soc, CTRL_OE);
	struct gpio_chip *gc = &tp->chip;
	unsigned long mask = BIT(6) | BIT(9);
	unsigned long bits = BIT(6);

	/* BIT(0) and BIT(11) are untouched by the mask; BIT(9) must clear. */
	fake_set(f, IOSW_BASE + 0x184, BIT(0) | BIT(9) | BIT(11));
	f->nwrites = 0;
	gc->set_multiple(gc, &mask, &bits);
	KUNIT_EXPECT_EQ(test, f->nwrites, 1u);
	KUNIT_EXPECT_EQ(test, fake_get(f, IOSW_BASE + 0x184),
			BIT(0) | BIT(6) | BIT(11));
}

static void tsi_gpio_get_multiple_maps_bits(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IOSW_BASE,
					     &tsi_skylp_iosw_soc, CTRL_IE);
	struct gpio_chip *gc = &tp->chip;
	unsigned long mask = BIT(0) | BIT(11);
	unsigned long bits = 0;

	fake_set(f, IOSW_BASE + 0x188, BIT(11) | BIT(5));
	KUNIT_ASSERT_EQ(test, gc->get_multiple(gc, &mask, &bits), 0);
	/* Only masked lines reported; bit 5 was not requested. */
	KUNIT_EXPECT_EQ(test, bits, BIT(11));
}

/* ----------------------------- irqchip ------------------------------ */

/*
 * mask clears the line's bit in OUR destination group's enable register
 * and nothing else. Group 2 is used rather than 0 so a wrong stride
 * writes an unmodeled address and fails via fake_write().
 */
static void tsi_irq_mask_clears_group_en_bit_only(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner_irq(test, &f, IONW_BASE,
						 &tsi_skylp_ionw_soc,
						 CTRL_OE, 0, 2);
	u32 en_g2 = IONW_T2_EN_G0 + 2 * GRP_STRIDE;

	/* GPIO_4 is line 21; GPIO_3 (line 3) must be left alone. */
	fake_set(f, en_g2, BIT(21) | BIT(3));
	tsi_pinctrl_irq_mask_hw(tp, 21);
	KUNIT_EXPECT_EQ(test, fake_get(f, en_g2), BIT(3));
}

/*
 * unmask arms the line in BOTH the global per-source enable and the
 * group enable. The global one is shared with the other destination
 * groups, which is the documented open item, so it is asserted rather
 * than left implicit.
 */
static void tsi_irq_unmask_sets_glben_and_group_en(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner_irq(test, &f, IONW_BASE,
						 &tsi_skylp_ionw_soc,
						 CTRL_OE, 0, 2);
	u32 en_g2 = IONW_T2_EN_G0 + 2 * GRP_STRIDE;

	tsi_pinctrl_irq_unmask_hw(tp, 21);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_T2_GLBEN), BIT(21));
	KUNIT_EXPECT_EQ(test, fake_get(f, en_g2), BIT(21));
	/* Other groups' enables must not be touched. */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_T2_EN_G0), 0u);
}

/* ack writes exactly the line's bit to the write-1-clear register. */
static void tsi_irq_ack_writes_w1c(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	f->nwrites = 0;
	tsi_pinctrl_irq_ack_hw(tp, 24);		/* GPIO_7 -> bit 24 */
	KUNIT_ASSERT_EQ(test, f->nwrites, 1u);
	KUNIT_EXPECT_EQ(test, f->log[0].off, (u32)IONW_T2_W1C);
	KUNIT_EXPECT_EQ(test, f->log[0].val, BIT(24));
}

/*
 * IOSE shares its collector with MDIO at bit 0, so its GPIO sources sit
 * at bits 1..2 while the pins are still 0..1. If the shift were dropped
 * the driver would mask or ack the MDIO interrupt instead - this is the
 * one corner where collector bit != pin.
 */
static void tsi_iose_irq_bit_is_shifted(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IOSE_BASE,
					     &tsi_skylp_iose_soc, CTRL_OE);

	KUNIT_EXPECT_EQ(test, tsi_skylp_iose_soc.irq_bit_shift, 1);

	/* TLQ_PERST_0 is pin 0 but collector bit 1. */
	tsi_pinctrl_irq_unmask_hw(tp, 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IOSE_GLBEN), BIT(1));
	KUNIT_EXPECT_EQ(test, fake_get(f, IOSE_EN_G0), BIT(1));

	/* TLQ_PERST_1 is pin 1 but collector bit 2. */
	tsi_pinctrl_irq_unmask_hw(tp, 1);
	KUNIT_EXPECT_EQ(test, fake_get(f, IOSE_GLBEN), BIT(1) | BIT(2));

	/* mdio_intr at bit 0 must never be armed by GPIO code. */
	KUNIT_EXPECT_EQ(test, fake_get(f, IOSE_GLBEN) & BIT(0), 0u);

	f->nwrites = 0;
	tsi_pinctrl_irq_ack_hw(tp, 0);
	KUNIT_ASSERT_EQ(test, f->nwrites, 1u);
	KUNIT_EXPECT_EQ(test, f->log[0].val, BIT(1));
}

/* Corners whose collector is GPIO-only must not shift. */
static void tsi_gpio_only_collectors_do_not_shift(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, tsi_skylp_ionw_soc.irq_bit_shift, 0);
	KUNIT_EXPECT_EQ(test, tsi_skylp_ione_soc.irq_bit_shift, 0);
	KUNIT_EXPECT_EQ(test, tsi_skylp_iosw_soc.irq_bit_shift, 0);
}

/*
 * The collector is level-only and has no polarity control. Mixed
 * edge+level values must be rejected too: a bitmask test against
 * IRQ_TYPE_LEVEL_MASK would wrongly accept them.
 */
static void tsi_irq_set_type_accepts_level(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			tsi_pinctrl_irq_type_valid(IRQ_TYPE_LEVEL_HIGH), 0);
	KUNIT_EXPECT_EQ(test,
			tsi_pinctrl_irq_type_valid(IRQ_TYPE_LEVEL_LOW), 0);
}

static void tsi_irq_set_type_rejects_edge_and_mixed(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			tsi_pinctrl_irq_type_valid(IRQ_TYPE_EDGE_RISING),
			-EINVAL);
	KUNIT_EXPECT_EQ(test,
			tsi_pinctrl_irq_type_valid(IRQ_TYPE_EDGE_BOTH),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_irq_type_valid(IRQ_TYPE_NONE),
			-EINVAL);
	/* Mixed edge+level: has a level bit set, but is still invalid. */
	KUNIT_EXPECT_EQ(test,
			tsi_pinctrl_irq_type_valid(IRQ_TYPE_LEVEL_HIGH |
						 IRQ_TYPE_EDGE_RISING),
			-EINVAL);
}

/*
 * Without an "interrupts" property the corner registers as a plain GPIO
 * controller: no irq domain is created, so nothing can request a line
 * as an interrupt. That dormancy is deliberate while the
 * collector-to-GIC wiring and M85 ownership are unconfirmed.
 */
static void tsi_irq_dormant_without_parent(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);

	KUNIT_EXPECT_NULL(test, tp->chip.irq.domain);
	KUNIT_EXPECT_NULL(test, tp->chip.irq.parent_handler);
}

/* ---------------------------- pinconf ------------------------------- */

static void tsi_pinconf_bias(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONW_BASE + 0x11c;	/* GPIO_0 */

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_UP, 1)),
			0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_PE | CTRL_PS),
			(u32)(CTRL_PE | CTRL_PS));

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_DOWN, 1)),
			0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_PE | CTRL_PS),
			(u32)CTRL_PE);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_BIAS_DISABLE, 0)),
			0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_PE | CTRL_PS), 0u);
}

static void tsi_pinconf_drive_strength(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONW_BASE + 0x11c;

	/* 10 mA is DS code 5 (table 3/4/5/6/8/10). */
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 10)),
			0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & CTRL_DS, 5u);

	/* A value not in the table must be rejected, not rounded. */
	KUNIT_EXPECT_NE(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 7)),
			0);
}

static void tsi_pinconf_schmitt(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONW_BASE + 0x11c;

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_INPUT_SCHMITT_ENABLE,
						 1)), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & CTRL_IS, (u32)CTRL_IS);
}

/* pinconf_get helper: returns the op's status, argument via @arg. */
static int pinconf_query(struct tsi_pinctrl *tp, unsigned int pin,
			 enum pin_config_param param, u32 *arg)
{
	unsigned long cfg = pinconf_to_config_packed(param, 0);
	int ret = tsi_pinconf_ops.pin_config_get(tp->pctl, pin, &cfg);

	if (!ret && arg)
		*arg = pinconf_to_config_argument(cfg);
	return ret;
}

/*
 * Bias readback is a QUERY, not a value: pinconf expects -EINVAL when
 * the pad is not in the state being asked about. Reporting "pull-up = 0"
 * instead would read as success and is the easy way to get this wrong.
 */
static void tsi_pinconf_get_bias_is_a_query(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 arg = 0;

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_UP, 1)), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_PULL_UP,
					    &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 1u);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_PULL_DOWN,
					    NULL), -EINVAL);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_DISABLE,
					    NULL), -EINVAL);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_DOWN, 1)), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_PULL_DOWN,
					    NULL), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_PULL_UP,
					    NULL), -EINVAL);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_BIAS_DISABLE, 0)), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_DISABLE,
					    NULL), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_BIAS_PULL_UP,
					    NULL), -EINVAL);
}

/* Drive strength reads back in mA, not as the raw DS code. */
static void tsi_pinconf_get_drive_and_schmitt(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;
	u32 arg = 0;

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 8)), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_BASE + 0x11c) & CTRL_DS, 4u);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_DRIVE_STRENGTH,
					    &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 8u);

	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0,
			PIN_CONFIG_INPUT_SCHMITT_ENABLE, &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 0u);
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_INPUT_SCHMITT_ENABLE,
						 1)), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0,
			PIN_CONFIG_INPUT_SCHMITT_ENABLE, &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 1u);
}

/*
 * DS codes 6 and 7 are reserved and carry no documented current, so a
 * pad found in that state must report an error. Folding them back into
 * the table (a stray modulo, say) would invent a plausible 3 mA or 4 mA
 * for a pad firmware or the M85 left in an undefined state.
 */
static void tsi_pinconf_get_rejects_reserved_ds(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	u32 ctrl = IONW_BASE + 0x11c;	/* GPIO_0 */
	u32 arg = 0;

	/* every documented code still reads back its own current */
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tsi_test_ds_ma); i++) {
		fake_set(f, ctrl, i);
		KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0,
				PIN_CONFIG_DRIVE_STRENGTH, &arg), 0);
		KUNIT_EXPECT_EQ_MSG(test, arg, tsi_test_ds_ma[i],
				    "DS code %u", i);
	}

	fake_set(f, ctrl, 6);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_DRIVE_STRENGTH,
					    NULL), -EINVAL);
	fake_set(f, ctrl, 7);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_DRIVE_STRENGTH,
					    NULL), -EINVAL);
}

/*
 * OUTPUT_ENABLE and INPUT_ENABLE are implemented but were never
 * exercised. Both write the OE/IE pair together, so each one also
 * clears the other - the same rule direction_input/output follow.
 */
static void tsi_pinconf_direction_params(struct kunit *test)
{
	struct fake_regs *f;
	/*
	 * Start with IE already set, so switching to output has to actively
	 * CLEAR it. Starting from a cleared register would let a mask that
	 * covers only OE pass while leaving the pad both driving and
	 * receiving.
	 */
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_IE);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONW_BASE + 0x11c;	/* GPIO_0 */
	u32 arg = 0;

	KUNIT_ASSERT_EQ(test, fake_get(f, ctrl) & CTRL_IE, (u32)CTRL_IE);

	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 1)), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_OE | CTRL_IE),
			(u32)CTRL_OE);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_OUTPUT_ENABLE,
					    &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 1u);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_INPUT_ENABLE,
					    &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 0u);
	/* gpiolib must agree about the direction it can now report. */
	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 0),
			GPIO_LINE_DIRECTION_OUT);

	/* Back to input: OE must be cleared as IE is set. */
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1)), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_OE | CTRL_IE),
			(u32)CTRL_IE);
	KUNIT_EXPECT_EQ(test, gc->get_direction(gc, 0),
			GPIO_LINE_DIRECTION_IN);

	/* output-enable = 0 means "receive", i.e. the IE side of the pair. */
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 1)), 0);
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 0)), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_OE | CTRL_IE),
			(u32)CTRL_IE);
}

/*
 * PIN_CONFIG_OUTPUT is what a DT hog's output-high/output-low becomes:
 * drive the pad to a defined level at configuration time. Same
 * no-stale-drive rule as direction_output(): the level must reach
 * data_out BEFORE OE rises, and only the pad's own bit may move.
 */
static void tsi_pinconf_output_drives_level(struct kunit *test)
{
	struct fake_regs *f;
	/* start receiving, so OUTPUT must actively clear IE too */
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_IE);
	struct gpio_chip *gc = &tp->chip;
	u32 ctrl = IONW_BASE + 0x11c;	/* GPIO_0 */
	u32 arg = 0;

	/* neighbour bits preloaded: the data_out write must be an RMW */
	fake_set(f, IONW_DATA_OUT, BIT(5) | BIT(24));

	f->nwrites = 0;
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_OUTPUT, 1)), 0);
	KUNIT_ASSERT_EQ(test, f->nwrites, 2u);
	/* order: level first, OE second */
	KUNIT_EXPECT_EQ(test, f->log[0].off, (u32)IONW_DATA_OUT);
	KUNIT_EXPECT_EQ(test, f->log[1].off, ctrl);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT),
			BIT(0) | BIT(5) | BIT(24));
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_OE | CTRL_IE),
			(u32)CTRL_OE);

	/* output-low: clears only this pad's bit, pad keeps driving */
	KUNIT_ASSERT_EQ(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_OUTPUT, 0)), 0);
	KUNIT_EXPECT_EQ(test, fake_get(f, IONW_DATA_OUT), BIT(5) | BIT(24));
	KUNIT_EXPECT_EQ(test, fake_get(f, ctrl) & (CTRL_OE | CTRL_IE),
			(u32)CTRL_OE);

	/* readback reports the driven level... */
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_OUTPUT, &arg), 0);
	KUNIT_EXPECT_EQ(test, arg, 0u);

	/* ...and is a query: a non-driving pad has no output level */
	KUNIT_ASSERT_EQ(test, gc->direction_input(gc, 0), 0);
	KUNIT_EXPECT_EQ(test, pinconf_query(tp, 0, PIN_CONFIG_OUTPUT, NULL),
			-EINVAL);
}

/* An unsupported parameter must be refused, not quietly dropped. */
static void tsi_pinconf_rejects_unsupported_param(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, 0);
	struct gpio_chip *gc = &tp->chip;

	f->nwrites = 0;
	KUNIT_EXPECT_NE(test, gc->set_config(gc, 0,
			pinconf_to_config_packed(PIN_CONFIG_SLEW_RATE, 1)), 0);
	KUNIT_EXPECT_EQ(test, f->nwrites, 0u);
	KUNIT_EXPECT_NE(test, pinconf_query(tp, 0, PIN_CONFIG_SLEW_RATE,
					    NULL), 0);
}

/*
 * Configuration is per pad in hardware even though muxing is not, so a
 * group_set has to touch every pad's own control register. IONE's led
 * group is two pads (LED_0 = line 21, LED_1 = line 22).
 */
static void tsi_pinconf_group_set_touches_every_pad(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONE_BASE,
					     &tsi_skylp_ione_soc, 0);
	unsigned long cfgs[] = {
		pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 6),
		pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_UP, 1),
	};
	int g = grp_sel(test, tp, "led");
	u32 led0 = IONE_BASE + 0x15c, led1 = IONE_BASE + 0x160;

	KUNIT_ASSERT_GE(test, g, 0);
	KUNIT_ASSERT_EQ(test, tsi_pinconf_ops.pin_config_group_set(tp->pctl, g,
					cfgs, ARRAY_SIZE(cfgs)), 0);

	/* 6 mA is DS code 3 in the 3/4/5/6/8/10 table. */
	KUNIT_EXPECT_EQ(test, fake_get(f, led0) & CTRL_DS, 3u);
	KUNIT_EXPECT_EQ(test, fake_get(f, led1) & CTRL_DS, 3u);
	KUNIT_EXPECT_EQ(test, fake_get(f, led0) & (CTRL_PE | CTRL_PS),
			(u32)(CTRL_PE | CTRL_PS));
	KUNIT_EXPECT_EQ(test, fake_get(f, led1) & (CTRL_PE | CTRL_PS),
			(u32)(CTRL_PE | CTRL_PS));

	/* A pad outside the group must be untouched. */
	KUNIT_EXPECT_EQ(test, fake_get(f, IONE_BASE + 0x164) & CTRL_DS, 0u);
}

/* ------------- exhaustive per-pin sweeps (all 65 pads) --------------- */

/*
 * The spot-check tests above pick the pads whose mapping is
 * counter-intuitive. These sweeps take the other approach and touch
 * EVERY pad of EVERY corner, which is what catches a single bad table
 * row - a wrong ctrl_off, a duplicated entry, an off-by-one - on a pad
 * nobody thought to name in a test.
 */

static const struct tsi_corner_ref tsi_all_corners[] = {
	{ &tsi_skylp_ionw_soc, IONW_BASE },
	{ &tsi_skylp_ione_soc, IONE_BASE },
	{ &tsi_skylp_iose_soc, IOSE_BASE },
	{ &tsi_skylp_iosw_soc, IOSW_BASE },
};

/*
 * Two pads sharing a control register would be a generator bug that
 * every other test misses: each pad would still configure "correctly",
 * just also clobbering its twin.
 */
static void tsi_pad_ctrl_offsets_are_unique(struct kunit *test)
{
	unsigned int c, i, j;

	for (c = 0; c < ARRAY_SIZE(tsi_all_corners); c++) {
		const struct tsi_pinctrl_soc *soc = tsi_all_corners[c].soc;

		for (i = 0; i < soc->npins; i++) {
			/* the data pair must not alias a pad either */
			KUNIT_EXPECT_NE_MSG(test, soc->pins[i].ctrl_off,
					    soc->data_out_off,
					    "%s pin %u aliases data_out",
					    soc->label, i);
			KUNIT_EXPECT_NE_MSG(test, soc->pins[i].ctrl_off,
					    soc->data_in_off,
					    "%s pin %u aliases data_in",
					    soc->label, i);
			for (j = i + 1; j < soc->npins; j++)
				KUNIT_EXPECT_NE_MSG(test,
					soc->pins[i].ctrl_off,
					soc->pins[j].ctrl_off,
					"%s pins %u and %u share ctrl +0x%03x",
					soc->label, i, j,
					soc->pins[i].ctrl_off);
		}
	}
}

/*
 * Drive every pad as an output with an alternating pattern, then read
 * data_out ONCE: a pad wired to the wrong bit shows up as a mismatch in
 * the whole word rather than needing a test per pad. Each pad's own
 * control register is checked for OE too, so a wrong ctrl_off is caught
 * in the same pass.
 */
static void tsi_every_pin_drives_its_own_bit(struct kunit *test)
{
	unsigned int c, i;

	for (c = 0; c < ARRAY_SIZE(tsi_all_corners); c++) {
		const struct tsi_pinctrl_soc *soc = tsi_all_corners[c].soc;
		u32 base = tsi_all_corners[c].base;
		struct fake_regs *f;
		/*
		 * Start every pad receiving, so switching to output has to
		 * clear IE on each one. Starting from a zeroed register
		 * would let a mask covering only OE pass on all 65 pads.
		 */
		struct tsi_pinctrl *tp = fake_corner_irq(test, &f, base, soc,
							 CTRL_IE, 0, 0);
		struct gpio_chip *gc = &tp->chip;
		u32 expect = 0;

		for (i = 0; i < soc->npins; i++) {
			int val = i % 2;

			KUNIT_ASSERT_EQ_MSG(test,
				gc->direction_output(gc, i, val), 0,
				"%s pin %u", soc->label, i);
			if (val)
				expect |= BIT(i);

			/* the pad itself must now be driving, not receiving */
			KUNIT_EXPECT_EQ_MSG(test,
				fake_get(f, base + soc->pins[i].ctrl_off) &
					(CTRL_OE | CTRL_IE), (u32)CTRL_OE,
				"%s pin %u (%s) ctrl +0x%03x", soc->label, i,
				soc->pins[i].name, soc->pins[i].ctrl_off);
		}

		KUNIT_EXPECT_EQ_MSG(test, fake_get(f, base + soc->data_out_off),
				    expect, "%s data_out pattern", soc->label);

		/* and clearing every line empties the register exactly */
		for (i = 0; i < soc->npins; i++)
			gc->set(gc, i, 0);
		KUNIT_EXPECT_EQ_MSG(test, fake_get(f, base + soc->data_out_off),
				    0u, "%s data_out not fully cleared",
				    soc->label);
	}
}

/*
 * Same idea for the input path: put every pad in input mode, inject one
 * known word into the read-only data_in, and check each pad reports its
 * own bit rather than a neighbour's.
 */
static void tsi_every_pin_samples_its_own_bit(struct kunit *test)
{
	unsigned int c, i;

	for (c = 0; c < ARRAY_SIZE(tsi_all_corners); c++) {
		const struct tsi_pinctrl_soc *soc = tsi_all_corners[c].soc;
		u32 base = tsi_all_corners[c].base;
		struct fake_regs *f;
		struct tsi_pinctrl *tp = fake_corner_irq(test, &f, base, soc,
							 CTRL_OE, 0, 0);
		struct gpio_chip *gc = &tp->chip;
		unsigned long mask = 0, bits = 0;
		u32 pattern = 0;

		for (i = 0; i < soc->npins; i++) {
			KUNIT_ASSERT_EQ_MSG(test,
				gc->direction_input(gc, i), 0,
				"%s pin %u", soc->label, i);
			KUNIT_EXPECT_EQ_MSG(test,
				fake_get(f, base + soc->pins[i].ctrl_off) &
					(CTRL_OE | CTRL_IE), (u32)CTRL_IE,
				"%s pin %u (%s)", soc->label, i,
				soc->pins[i].name);
			/* every third line high - not the output pattern */
			if (i % 3 == 0)
				pattern |= BIT(i);
			mask |= BIT(i);
		}

		fake_set(f, base + soc->data_in_off, pattern);

		for (i = 0; i < soc->npins; i++)
			KUNIT_EXPECT_EQ_MSG(test, gc->get(gc, i),
					    !!(pattern & BIT(i)),
					    "%s pin %u (%s)", soc->label, i,
					    soc->pins[i].name);

		/* get_multiple must agree with the per-line reads */
		KUNIT_ASSERT_EQ(test, gc->get_multiple(gc, &mask, &bits), 0);
		KUNIT_EXPECT_EQ_MSG(test, (u32)bits, pattern,
				    "%s get_multiple", soc->label);
	}
}

/*
 * Every pad, through the interrupt collector. Each line is armed alone
 * from a cleared collector, so the assertion is exact-equality on the
 * whole register: a pad that targets a neighbour's bit fails, and so
 * does one that forgets the IOSE shift.
 */
static void tsi_every_pin_irq_targets_its_own_bit(struct kunit *test)
{
	unsigned int c, i;

	for (c = 0; c < ARRAY_SIZE(tsi_all_corners); c++) {
		const struct tsi_pinctrl_soc *soc = tsi_all_corners[c].soc;
		u32 base = tsi_all_corners[c].base;
		struct fake_regs *f;
		/* group 3 also exercises the ip/en stride on every corner */
		struct tsi_pinctrl *tp = fake_corner_irq(test, &f, base, soc,
							 CTRL_OE, 0, 3);
		u32 glben = base + soc->irq_glben_off;
		u32 en = base + soc->irq_grp0_en_off + 3 * GRP_STRIDE;
		u32 w1c = base + soc->irq_w1c_off;

		for (i = 0; i < soc->npins; i++) {
			u32 bit = BIT(i + soc->irq_bit_shift);

			fake_set(f, glben, 0);
			fake_set(f, en, 0);

			tsi_pinctrl_irq_unmask_hw(tp, i);
			KUNIT_EXPECT_EQ_MSG(test, fake_get(f, glben), bit,
					    "%s pin %u (%s) glben",
					    soc->label, i, soc->pins[i].name);
			KUNIT_EXPECT_EQ_MSG(test, fake_get(f, en), bit,
					    "%s pin %u (%s) enable_g3",
					    soc->label, i, soc->pins[i].name);

			tsi_pinctrl_irq_mask_hw(tp, i);
			KUNIT_EXPECT_EQ_MSG(test, fake_get(f, en), 0u,
					    "%s pin %u mask left a bit set",
					    soc->label, i);

			f->nwrites = 0;
			tsi_pinctrl_irq_ack_hw(tp, i);
			KUNIT_ASSERT_EQ(test, f->nwrites, 1u);
			KUNIT_EXPECT_EQ_MSG(test, f->log[0].off, w1c,
					    "%s pin %u acked wrong register",
					    soc->label, i);
			KUNIT_EXPECT_EQ_MSG(test, f->log[0].val, bit,
					    "%s pin %u (%s) ack bit",
					    soc->label, i, soc->pins[i].name);
		}
	}
}

/*
 * Pad configuration on every pad, with a DIFFERENT drive strength per
 * pad so the readback proves isolation: if two pads shared a register
 * the second write would clobber the first and the mismatch surfaces.
 * Bias alternates for the same reason.
 */
static void tsi_every_pin_pinconf_is_isolated(struct kunit *test)
{
	unsigned int c, i;

	for (c = 0; c < ARRAY_SIZE(tsi_all_corners); c++) {
		const struct tsi_pinctrl_soc *soc = tsi_all_corners[c].soc;
		u32 base = tsi_all_corners[c].base;
		struct fake_regs *f;
		struct tsi_pinctrl *tp = fake_corner_irq(test, &f, base, soc,
							 0, 0, 0);
		struct gpio_chip *gc = &tp->chip;

		for (i = 0; i < soc->npins; i++) {
			u8 ma = tsi_test_ds_ma[i % ARRAY_SIZE(tsi_test_ds_ma)];
			bool up = i % 2;

			KUNIT_ASSERT_EQ_MSG(test, gc->set_config(gc, i,
				pinconf_to_config_packed(
					PIN_CONFIG_DRIVE_STRENGTH, ma)), 0,
				"%s pin %u ds %umA", soc->label, i, ma);
			KUNIT_ASSERT_EQ_MSG(test, gc->set_config(gc, i,
				pinconf_to_config_packed(up ?
					PIN_CONFIG_BIAS_PULL_UP :
					PIN_CONFIG_BIAS_PULL_DOWN, 1)), 0,
				"%s pin %u bias", soc->label, i);
			if (i % 5 == 0)
				KUNIT_ASSERT_EQ_MSG(test, gc->set_config(gc, i,
					pinconf_to_config_packed(
					  PIN_CONFIG_INPUT_SCHMITT_ENABLE, 1)),
					0, "%s pin %u schmitt", soc->label, i);
		}

		/* now read every pad back and demand its own settings */
		for (i = 0; i < soc->npins; i++) {
			u8 ma = tsi_test_ds_ma[i % ARRAY_SIZE(tsi_test_ds_ma)];
			bool up = i % 2;
			u32 arg = 0;

			KUNIT_ASSERT_EQ_MSG(test, pinconf_query(tp, i,
				PIN_CONFIG_DRIVE_STRENGTH, &arg), 0,
				"%s pin %u (%s)", soc->label, i,
				soc->pins[i].name);
			KUNIT_EXPECT_EQ_MSG(test, arg, (u32)ma,
				"%s pin %u (%s) ds readback", soc->label, i,
				soc->pins[i].name);

			KUNIT_EXPECT_EQ_MSG(test, pinconf_query(tp, i,
				up ? PIN_CONFIG_BIAS_PULL_UP :
				     PIN_CONFIG_BIAS_PULL_DOWN, NULL), 0,
				"%s pin %u (%s) bias readback", soc->label, i,
				soc->pins[i].name);
			/* and the opposite bias must NOT read back */
			KUNIT_EXPECT_EQ_MSG(test, pinconf_query(tp, i,
				up ? PIN_CONFIG_BIAS_PULL_DOWN :
				     PIN_CONFIG_BIAS_PULL_UP, NULL), -EINVAL,
				"%s pin %u wrong bias reported", soc->label, i);

			KUNIT_EXPECT_EQ_MSG(test, pinconf_query(tp, i,
				PIN_CONFIG_INPUT_SCHMITT_ENABLE, &arg), 0,
				"%s pin %u schmitt readback", soc->label, i);
			KUNIT_EXPECT_EQ_MSG(test, arg, (u32)(i % 5 == 0),
				"%s pin %u (%s) schmitt", soc->label, i,
				soc->pins[i].name);
		}
	}
}

/*
 * The window a DT node must declare, as independent literals. Each span
 * lands one word past the collector's enable_g3 - the highest register
 * any corner touches - which is what a DT author has to cover. These
 * numbers are also what the binding documents, so a table change that
 * moves the collector shows up here and in review rather than as a
 * board that writes outside its declared window.
 */
static void tsi_reg_span_covers_every_register(struct kunit *test)
{
	unsigned int c, i;

	KUNIT_EXPECT_EQ(test, tsi_pinctrl_reg_span(&tsi_skylp_ionw_soc),
			0x3500u);
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_reg_span(&tsi_skylp_ione_soc),
			0xc380u);
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_reg_span(&tsi_skylp_iose_soc),
			0x2080u);
	KUNIT_EXPECT_EQ(test, tsi_pinctrl_reg_span(&tsi_skylp_iosw_soc),
			0x9100u);

	/* and it must actually bound every offset in every table */
	for (c = 0; c < ARRAY_SIZE(tsi_all_corners); c++) {
		const struct tsi_pinctrl_soc *soc = tsi_all_corners[c].soc;
		u32 span = tsi_pinctrl_reg_span(soc);

		for (i = 0; i < soc->npins; i++)
			KUNIT_EXPECT_LT_MSG(test, soc->pins[i].ctrl_off, span,
					    "%s pin %u (%s) outside span",
					    soc->label, i, soc->pins[i].name);

		for (i = 0; i < soc->ngroups; i++)
			KUNIT_EXPECT_LT_MSG(test, soc->groups[i].iomode_off,
					    span, "%s group %s iomode outside",
					    soc->label, soc->groups[i].name);

		KUNIT_EXPECT_LT(test, soc->data_out_off, span);
		KUNIT_EXPECT_LT(test, soc->data_in_off, span);
		KUNIT_EXPECT_LT(test, soc->irq_w1c_off, span);
		KUNIT_EXPECT_LT(test, soc->irq_glben_off, span);
		/* the furthest group, g3, is the one that sets the bound */
		KUNIT_EXPECT_LT(test, soc->irq_grp0_en_off +
				3 * GRP_STRIDE, span);
		KUNIT_EXPECT_LT(test, soc->irq_grp0_ip_off +
				3 * GRP_STRIDE, span);
	}
}

/*
 * IOSE and IOSW pad offsets against independent literals, completing
 * what tsi_ionw_pad_offsets/tsi_ione_pad_offsets do for the other two:
 * before this their tables were only checked for "named and non-zero".
 */
static void tsi_iose_iosw_pad_offsets(struct kunit *test)
{
	const struct tsi_pinctrl_soc *e = &tsi_skylp_iose_soc;
	const struct tsi_pinctrl_soc *w = &tsi_skylp_iosw_soc;

	KUNIT_EXPECT_STREQ(test, e->pins[0].name, "TLQ_PERST_0");
	KUNIT_EXPECT_EQ(test, e->pins[0].ctrl_off, RAL_IOSE_PRST0_CTRL);
	KUNIT_EXPECT_STREQ(test, e->pins[1].name, "TLQ_PERST_1");
	KUNIT_EXPECT_EQ(test, e->pins[1].ctrl_off, RAL_IOSE_PRST1_CTRL);
	KUNIT_EXPECT_EQ(test, e->data_out_off, 0x128u);
	KUNIT_EXPECT_EQ(test, e->data_in_off, 0x12cu);

	/* I3C occupies 0..5, I2S 6..11 - and SDI/SDO are interleaved. */
	KUNIT_EXPECT_STREQ(test, w->pins[0].name, "I2C_I3C_SCL_0");
	KUNIT_EXPECT_EQ(test, w->pins[0].ctrl_off, RAL_IOSW_I3C_SCL0_CTRL);
	KUNIT_EXPECT_STREQ(test, w->pins[5].name, "I2C_I3C_SDA_2");
	KUNIT_EXPECT_EQ(test, w->pins[5].ctrl_off, RAL_IOSW_I3C_SDA2_CTRL);
	KUNIT_EXPECT_STREQ(test, w->pins[6].name, "I2S_SCK");
	KUNIT_EXPECT_EQ(test, w->pins[6].ctrl_off, RAL_IOSW_I2S_SCK_CTRL);
	/* WS is +0x16c while SDI_0 is +0x168 - not pad order */
	KUNIT_EXPECT_STREQ(test, w->pins[7].name, "I2S_WS");
	KUNIT_EXPECT_EQ(test, w->pins[7].ctrl_off, RAL_IOSW_I2S_WS_CTRL);
	KUNIT_EXPECT_STREQ(test, w->pins[8].name, "I2S_SDI_0");
	KUNIT_EXPECT_EQ(test, w->pins[8].ctrl_off, RAL_IOSW_I2S_SDI0_CTRL);
	KUNIT_EXPECT_STREQ(test, w->pins[10].name, "I2S_SDO_1");
	KUNIT_EXPECT_EQ(test, w->pins[10].ctrl_off, RAL_IOSW_I2S_SDO1_CTRL);
	KUNIT_EXPECT_STREQ(test, w->pins[11].name, "I2S_SDI_1");
	KUNIT_EXPECT_EQ(test, w->pins[11].ctrl_off, RAL_IOSW_I2S_SDI1_CTRL);
	KUNIT_EXPECT_EQ(test, w->data_out_off, 0x184u);
	KUNIT_EXPECT_EQ(test, w->data_in_off, 0x188u);
}

/*
 * gpiolib must receive the pad names: they come from gpio_chip.names,
 * which is a separate array from the pinctrl pin descriptors, so
 * populating only the latter would leave every line unnamed in
 * gpioinfo. Checked against the non-contiguous IONW mapping so a
 * name array built in the wrong order also fails.
 */
static void tsi_gpio_line_names_are_set(struct kunit *test)
{
	struct fake_regs *f;
	struct tsi_pinctrl *tp = fake_corner(test, &f, IONW_BASE,
					     &tsi_skylp_ionw_soc, CTRL_OE);
	struct gpio_chip *gc = &tp->chip;

	KUNIT_ASSERT_NOT_NULL(test, gc->names);
	KUNIT_EXPECT_STREQ(test, gc->names[0], "GPIO_0");
	KUNIT_EXPECT_STREQ(test, gc->names[9], "SPI_IO0");
	/* GPIO_4 is line 21, not line 4. */
	KUNIT_EXPECT_STREQ(test, gc->names[21], "GPIO_4");
	KUNIT_EXPECT_STREQ(test, gc->names[25], "TLP_PERST_2");
}

/* Every corner names all of its lines, not just IONW. */
static void tsi_all_corners_name_every_line(struct kunit *test)
{
	const struct tsi_pinctrl_soc *socs[] = {
		&tsi_skylp_ionw_soc, &tsi_skylp_ione_soc,
		&tsi_skylp_iose_soc, &tsi_skylp_iosw_soc,
	};
	const u32 bases[] = { IONW_BASE, IONE_BASE, IOSE_BASE, IOSW_BASE };
	unsigned int s, i;

	for (s = 0; s < ARRAY_SIZE(socs); s++) {
		struct fake_regs *f;
		struct tsi_pinctrl *tp = fake_corner(test, &f, bases[s],
						     socs[s], CTRL_OE);

		KUNIT_ASSERT_NOT_NULL(test, tp->chip.names);
		for (i = 0; i < socs[s]->npins; i++)
			KUNIT_EXPECT_STREQ_MSG(test, tp->chip.names[i],
					       socs[s]->pins[i].name,
					       "%s line %u misnamed",
					       socs[s]->label, i);
	}
}

/* Every corner must register: pins, groups and a non-empty function list. */
static void tsi_all_corners_register(struct kunit *test)
{
	const struct tsi_pinctrl_soc *socs[] = {
		&tsi_skylp_ionw_soc, &tsi_skylp_ione_soc,
		&tsi_skylp_iose_soc, &tsi_skylp_iosw_soc,
	};
	const u32 bases[] = { IONW_BASE, IONE_BASE, IOSE_BASE, IOSW_BASE };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(socs); i++) {
		struct fake_regs *f;
		struct tsi_pinctrl *tp = fake_corner(test, &f, bases[i],
						     socs[i], CTRL_OE);

		KUNIT_EXPECT_EQ(test, tp->chip.ngpio, socs[i]->npins);
		KUNIT_EXPECT_NE_MSG(test, tp->nfuncs, 0,
				    "%s built no functions", socs[i]->label);
	}
}

static struct kunit_case tsi_pinctrl_test_cases[] = {
	/* generated-table integrity */
	KUNIT_CASE(tsi_pin_counts),
	KUNIT_CASE(tsi_ionw_pad_offsets),
	KUNIT_CASE(tsi_ione_pad_offsets),
	KUNIT_CASE(tsi_all_pads_populated),
	KUNIT_CASE(tsi_groups_partition_pins),
	KUNIT_CASE(tsi_groups_have_one_gpio_opt),
	KUNIT_CASE(tsi_iomode_encodings),
	/* mux */
	KUNIT_CASE(tsi_gpio_enable_dedicated_is_noop),
	KUNIT_CASE(tsi_gpio_enable_muxed_writes_iomode),
	KUNIT_CASE(tsi_gpio_enable_is_group_wide),
	KUNIT_CASE(tsi_mux_set_preserves_other_bits),
	KUNIT_CASE(tsi_uart_groups_are_independent),
	KUNIT_CASE(tsi_set_mux_selects_peripheral),
	KUNIT_CASE(tsi_set_mux_selects_mdio),
	KUNIT_CASE(tsi_set_mux_rejects_foreign_function),
	KUNIT_CASE(tsi_set_mux_on_dedicated_group),
	KUNIT_CASE(tsi_function_list_is_derived),
	KUNIT_CASE(tsi_pinmux_is_strict),
	KUNIT_CASE(tsi_func_of_val_decodes),
	/* gpio_chip */
	KUNIT_CASE(tsi_gpio_set_hits_data_bit),
	KUNIT_CASE(tsi_gpio_direction_output_value_before_oe),
	KUNIT_CASE(tsi_gpio_get_reads_data_in),
	KUNIT_CASE(tsi_gpio_get_direction_decodes_oe),
	KUNIT_CASE(tsi_gpio_set_multiple_single_write),
	KUNIT_CASE(tsi_gpio_get_multiple_maps_bits),
	/* irqchip */
	KUNIT_CASE(tsi_irq_mask_clears_group_en_bit_only),
	KUNIT_CASE(tsi_irq_unmask_sets_glben_and_group_en),
	KUNIT_CASE(tsi_irq_ack_writes_w1c),
	KUNIT_CASE(tsi_iose_irq_bit_is_shifted),
	KUNIT_CASE(tsi_gpio_only_collectors_do_not_shift),
	KUNIT_CASE(tsi_irq_set_type_accepts_level),
	KUNIT_CASE(tsi_irq_set_type_rejects_edge_and_mixed),
	KUNIT_CASE(tsi_irq_dormant_without_parent),
	/* pinconf */
	KUNIT_CASE(tsi_pinconf_bias),
	KUNIT_CASE(tsi_pinconf_drive_strength),
	KUNIT_CASE(tsi_pinconf_schmitt),
	KUNIT_CASE(tsi_pinconf_get_bias_is_a_query),
	KUNIT_CASE(tsi_pinconf_get_drive_and_schmitt),
	KUNIT_CASE(tsi_pinconf_get_rejects_reserved_ds),
	KUNIT_CASE(tsi_pinconf_direction_params),
	KUNIT_CASE(tsi_pinconf_output_drives_level),
	KUNIT_CASE(tsi_pinconf_rejects_unsupported_param),
	KUNIT_CASE(tsi_pinconf_group_set_touches_every_pad),
	/* exhaustive per-pin sweeps */
	KUNIT_CASE(tsi_pad_ctrl_offsets_are_unique),
	KUNIT_CASE(tsi_iose_iosw_pad_offsets),
	KUNIT_CASE(tsi_reg_span_covers_every_register),
	KUNIT_CASE(tsi_every_pin_drives_its_own_bit),
	KUNIT_CASE(tsi_every_pin_samples_its_own_bit),
	KUNIT_CASE(tsi_every_pin_irq_targets_its_own_bit),
	KUNIT_CASE(tsi_every_pin_pinconf_is_isolated),
	/* probe */
	KUNIT_CASE(tsi_gpio_line_names_are_set),
	KUNIT_CASE(tsi_all_corners_name_every_line),
	KUNIT_CASE(tsi_all_corners_register),
	{}
};

static struct kunit_suite tsi_pinctrl_test_suite = {
	.name = "pinctrl-tsi",
	.test_cases = tsi_pinctrl_test_cases,
};
kunit_test_suite(tsi_pinctrl_test_suite);

MODULE_DESCRIPTION("KUnit tests for the TSI pinctrl driver");
MODULE_LICENSE("GPL");
