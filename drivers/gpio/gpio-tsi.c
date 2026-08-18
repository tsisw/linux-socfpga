// SPDX-License-Identifier: GPL-2.0-only
/*
 * TSI SkyLP GPIO controller driver (gpiolib adapter over the
 * tsi-chiplet core).
 *
 * One instance drives the dedicated GPIO pads of one SkyLP IO corner:
 *   IONW (CF802): GPIO_0..7,    control +0x11c.., data 0x178/0x17c
 *   IONE (CF803): GPIO_FS_0..3, control +0x124.., data 0x188/0x18c
 * Register layout comes from the generated tsi-skylp-regs.h (ral.json
 * release SKYLP_G0829); the data-register bit of a line is NOT its
 * line index (GPIO_4..7 sit at bits 21..24), so each line carries an
 * explicit {ctrl_off, bit} pair.
 *
 * Direction is OE/IE in the per-pad control register. On output the
 * data_out bit is written before OE is raised so the pad never drives
 * a stale level. All accesses are single-register RMWs through the
 * chiplet regmap, whose locking makes them atomic; no driver lock.
 *
 * Interrupt support is present but DORMANT BY DEFAULT: pin events latch
 * in the corner's interrupt collector (t2 on IONW, intr_regs_8 on IONE),
 * fan out to four maskable destination groups g0..g3. Groups G2 and G3
 * are wired to the A520 GIC (confirmed RTL team 2026-08-16):
 *   IONW  G2=SPI34  G3=SPI35
 *   IONE  G2=SPI82  G3=SPI83
 * Hardware sensitivity is level-only (confirmed same date); no
 * polarity register exists so active-high vs active-low cannot be
 * programmed — handle_level_irq is wired unconditionally and
 * irq_set_type rejects edge requests.
 * The irqchip below is wired up ONLY if the platform device has an
 * "interrupts" property; with none (the case in tsi-soc.dtsi today),
 * the driver behaves exactly as before: polling only, nothing touches
 * the collector. Open item [E] (see below) is the remaining blocker
 * before "interrupts" should be added to a production DT node.
 *
 * Open hardware items tracked in the driver kit (Confluence page
 * 1821245459): [E] M85 co-ownership — no set/clear register exists
 * for data_out, so cross-master RMW cannot be made safe from Linux;
 * the interrupt path sharpens this further, since int_enable_reg is
 * a SINGLE GLOBAL per-source enable shared by all four destination
 * groups (unlike enable_gN, which is per-group). irq_unmask() below
 * writes int_enable_reg, so an A520 consumer unmasking its line can
 * affect whatever the M85 receives on a different group from the
 * same source — this needs an explicit ownership ruling, not just a
 * group assignment.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/visibility.h>
#include <linux/bitfield.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/tsi/tsi-chiplet.h>
#include <linux/soc/tsi/tsi-skylp-regs.h>

#include "gpio-tsi.h"

#define TSI_GPIO_LINE(_name, _ctrl, _bit) \
	{ .name = (_name), .ctrl_off = (_ctrl), .bit = (_bit) }

static const struct tsi_gpio_line tsi_gpio_ionw_lines[] = {
	TSI_GPIO_LINE("GPIO_0", TSI_SKYLP_IONW_GPIO0_CTRL,
		      TSI_SKYLP_IONW_GPIO0_BIT),
	TSI_GPIO_LINE("GPIO_1", TSI_SKYLP_IONW_GPIO1_CTRL,
		      TSI_SKYLP_IONW_GPIO1_BIT),
	TSI_GPIO_LINE("GPIO_2", TSI_SKYLP_IONW_GPIO2_CTRL,
		      TSI_SKYLP_IONW_GPIO2_BIT),
	TSI_GPIO_LINE("GPIO_3", TSI_SKYLP_IONW_GPIO3_CTRL,
		      TSI_SKYLP_IONW_GPIO3_BIT),
	TSI_GPIO_LINE("GPIO_4", TSI_SKYLP_IONW_GPIO4_CTRL,
		      TSI_SKYLP_IONW_GPIO4_BIT),
	TSI_GPIO_LINE("GPIO_5", TSI_SKYLP_IONW_GPIO5_CTRL,
		      TSI_SKYLP_IONW_GPIO5_BIT),
	TSI_GPIO_LINE("GPIO_6", TSI_SKYLP_IONW_GPIO6_CTRL,
		      TSI_SKYLP_IONW_GPIO6_BIT),
	TSI_GPIO_LINE("GPIO_7", TSI_SKYLP_IONW_GPIO7_CTRL,
		      TSI_SKYLP_IONW_GPIO7_BIT),
};

VISIBLE_IF_KUNIT const struct tsi_gpio_soc tsi_gpio_ionw_soc = {
	.lines		= tsi_gpio_ionw_lines,
	.nlines		= ARRAY_SIZE(tsi_gpio_ionw_lines),
	.data_out_off	= TSI_SKYLP_IONW_GPIO_DATA_OUT,
	.data_in_off	= TSI_SKYLP_IONW_GPIO_DATA_IN,
	.irq_w1c_off	= TSI_SKYLP_IONW_T2_INTR_W1C,
	.irq_glben_off	= TSI_SKYLP_IONW_T2_INTR_EN,
	.irq_grp0_ip_off = TSI_SKYLP_IONW_T2_G0_IP,
	.irq_grp0_en_off = TSI_SKYLP_IONW_T2_G0_EN,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_ionw_soc);

static const struct tsi_gpio_line tsi_gpio_ione_lines[] = {
	TSI_GPIO_LINE("GPIO_FS_0", TSI_SKYLP_IONE_GPIO_FS0_CTRL,
		      TSI_SKYLP_IONE_GPIO_FS0_BIT),
	TSI_GPIO_LINE("GPIO_FS_1", TSI_SKYLP_IONE_GPIO_FS1_CTRL,
		      TSI_SKYLP_IONE_GPIO_FS1_BIT),
	TSI_GPIO_LINE("GPIO_FS_2", TSI_SKYLP_IONE_GPIO_FS2_CTRL,
		      TSI_SKYLP_IONE_GPIO_FS2_BIT),
	TSI_GPIO_LINE("GPIO_FS_3", TSI_SKYLP_IONE_GPIO_FS3_CTRL,
		      TSI_SKYLP_IONE_GPIO_FS3_BIT),
};

VISIBLE_IF_KUNIT const struct tsi_gpio_soc tsi_gpio_ione_soc = {
	.lines		= tsi_gpio_ione_lines,
	.nlines		= ARRAY_SIZE(tsi_gpio_ione_lines),
	.data_out_off	= TSI_SKYLP_IONE_GPIO_DATA_OUT,
	.data_in_off	= TSI_SKYLP_IONE_GPIO_DATA_IN,
	.irq_w1c_off	= TSI_SKYLP_IONE_INTR8_INTR_W1C,
	.irq_glben_off	= TSI_SKYLP_IONE_INTR8_INTR_EN,
	.irq_grp0_ip_off = TSI_SKYLP_IONE_INTR8_G0_IP,
	.irq_grp0_en_off = TSI_SKYLP_IONE_INTR8_G0_EN,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_ione_soc);

/* CSR-window offset of a line's per-pad control register. */
static u32 tsi_gpio_ctrl_reg(struct tsi_gpio *gpio, unsigned int off)
{
	return gpio->base + gpio->soc->lines[off].ctrl_off;
}

/*
 * data_out/data_in mask of a line. The bit is NOT the line index:
 * IONW GPIO_4..7 sit at bits 21..24, IONE GPIO_FS_0..3 at 10..13.
 */
static u32 tsi_gpio_line_mask(struct tsi_gpio *gpio, unsigned int off)
{
	return BIT(gpio->soc->lines[off].bit);
}

/*
 * tsi_gpio_get_direction() - decode IN/OUT from the pad's OE bit.
 *
 * There is no direction register on SkyLP; OE[3] of the per-pad
 * control register decides whether the pad drives. OE is active-high
 * (confirmed from ral.json: IONW reset 0x008 = OE set).
 */
static int tsi_gpio_get_direction(struct gpio_chip *chip, unsigned int off)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	u32 ctrl;
	int ret, dir;

	ret = regmap_read(gpio->regmap, tsi_gpio_ctrl_reg(gpio, off), &ctrl);
	if (ret) {
		dev_dbg(gpio->chip.parent,
			"get_direction(%s): regmap_read failed: %d\n",
			gpio->soc->lines[off].name, ret);
		return ret;
	}

	dir = (ctrl & TSI_SKYLP_GPIO_CTRL_OE) ? GPIO_LINE_DIRECTION_OUT
					      : GPIO_LINE_DIRECTION_IN;
	dev_dbg(gpio->chip.parent, "get_direction(%s): ctrl=0x%03x -> %s\n",
		gpio->soc->lines[off].name, ctrl,
		dir == GPIO_LINE_DIRECTION_OUT ? "out" : "in");
	return dir;
}

/*
 * tsi_gpio_direction_input() - stop driving, enable the receiver.
 *
 * Clears OE and sets IE in one atomic RMW so the pad's other control
 * bits (drive strength, pulls) survive; a non-RMW write here would
 * silently reset them.
 */
static int tsi_gpio_direction_input(struct gpio_chip *chip, unsigned int off)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	int ret;

	dev_dbg(gpio->chip.parent, "direction_input(%s): clear OE, set IE\n",
		gpio->soc->lines[off].name);

	ret = regmap_update_bits(gpio->regmap, tsi_gpio_ctrl_reg(gpio, off),
				 TSI_SKYLP_GPIO_CTRL_OE |
				 TSI_SKYLP_GPIO_CTRL_IE,
				 TSI_SKYLP_GPIO_CTRL_IE);
	if (ret)
		dev_dbg(gpio->chip.parent,
			"direction_input(%s): regmap_update_bits failed: %d\n",
			gpio->soc->lines[off].name, ret);
	return ret;
}

/*
 * tsi_gpio_set() - set/clear one line's bit in the shared data_out.
 *
 * data_out is one register for all 26/25 pads of the corner and has
 * no set/clear companion, so this must be an RMW. The chiplet regmap
 * lock makes it atomic against other Linux threads; it cannot protect
 * against the M85 writing the same register (open item [E]).
 */
static int tsi_gpio_set(struct gpio_chip *chip, unsigned int off, int value)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	u32 mask = tsi_gpio_line_mask(gpio, off);
	int ret;

	dev_dbg(gpio->chip.parent, "set(%s): value=%d (data_out bit %u)\n",
		gpio->soc->lines[off].name, value, gpio->soc->lines[off].bit);

	ret = regmap_update_bits(gpio->regmap,
				 gpio->base + gpio->soc->data_out_off,
				 mask, value ? mask : 0);
	if (ret)
		dev_dbg(gpio->chip.parent,
			"set(%s): regmap_update_bits failed: %d\n",
			gpio->soc->lines[off].name, ret);
	return ret;
}

/*
 * tsi_gpio_direction_output() - drive the line at the given level.
 *
 * Ordering is a hardware constraint: the data_out bit is written
 * before OE is raised, otherwise the pad would briefly drive whatever
 * stale level data_out last held. The KUnit suite asserts this order.
 */
static int tsi_gpio_direction_output(struct gpio_chip *chip, unsigned int off,
				     int value)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	int ret;

	dev_dbg(gpio->chip.parent,
		"direction_output(%s): value=%d, set data_out before OE\n",
		gpio->soc->lines[off].name, value);

	/* Latch the level before raising OE: no stale-drive glitch. */
	ret = tsi_gpio_set(chip, off, value);
	if (ret)
		return ret;

	ret = regmap_update_bits(gpio->regmap, tsi_gpio_ctrl_reg(gpio, off),
				 TSI_SKYLP_GPIO_CTRL_OE |
				 TSI_SKYLP_GPIO_CTRL_IE,
				 TSI_SKYLP_GPIO_CTRL_OE);
	if (ret)
		dev_dbg(gpio->chip.parent,
			"direction_output(%s): OE update failed: %d\n",
			gpio->soc->lines[off].name, ret);
	return ret;
}

/*
 * tsi_gpio_get() - read the live pad level from data_in.
 *
 * data_in is read-only and reflects the receiver, so this is valid
 * for inputs and for reading back a driven output (when IE is set).
 */
static int tsi_gpio_get(struct gpio_chip *chip, unsigned int off)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	u32 val;
	int ret, level;

	ret = regmap_read(gpio->regmap, gpio->base + gpio->soc->data_in_off,
			  &val);
	if (ret) {
		dev_dbg(gpio->chip.parent, "get(%s): regmap_read failed: %d\n",
			gpio->soc->lines[off].name, ret);
		return ret;
	}

	level = !!(val & tsi_gpio_line_mask(gpio, off));
	dev_dbg(gpio->chip.parent, "get(%s): data_in=0x%x -> %d\n",
		gpio->soc->lines[off].name, val, level);
	return level;
}

/*
 * tsi_gpio_get_multiple() - read all requested lines in one regmap call.
 *
 * Reads data_in once and extracts the logical bit for each requested line,
 * accounting for the non-contiguous bit mapping (GPIO_4..7 at bits 21..24).
 */
static int tsi_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask,
				  unsigned long *bits)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	unsigned int i;
	u32 val;
	int ret;

	ret = regmap_read(gpio->regmap, gpio->base + gpio->soc->data_in_off,
			  &val);
	if (ret) {
		dev_dbg(gpio->chip.parent,
			"get_multiple: regmap_read failed: %d\n", ret);
		return ret;
	}

	dev_dbg(gpio->chip.parent, "get_multiple: data_in=0x%x\n", val);

	bitmap_zero(bits, chip->ngpio);
	for_each_set_bit(i, mask, chip->ngpio) {
		if (val & tsi_gpio_line_mask(gpio, i))
			set_bit(i, bits);
	}
	return 0;
}

/*
 * tsi_gpio_set_multiple() - drive multiple lines in one RMW.
 *
 * Translates the logical mask/bits into hardware mask/bits (applying the
 * non-contiguous bit mapping), then issues a single regmap_update_bits on
 * data_out instead of one call per line.
 */
static int tsi_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				  unsigned long *bits)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	unsigned int i;
	u32 hw_mask = 0, hw_bits = 0;

	for_each_set_bit(i, mask, chip->ngpio) {
		u32 m = tsi_gpio_line_mask(gpio, i);

		hw_mask |= m;
		if (test_bit(i, bits))
			hw_bits |= m;
	}

	dev_dbg(gpio->chip.parent,
		"set_multiple: hw_mask=0x%x hw_bits=0x%x\n", hw_mask, hw_bits);

	return regmap_update_bits(gpio->regmap,
				  gpio->base + gpio->soc->data_out_off,
				  hw_mask, hw_bits);
}

/*
 * 6.12 gpiolib's gpio_chip.set/.set_multiple are void-returning (this
 * driver was written against a gpiolib where they return int); these
 * thin wrappers are the compat shim so tsi_gpio_set()/tsi_gpio_set_multiple()
 * keep their int-returning signatures for internal callers (e.g.
 * tsi_gpio_direction_output()) while satisfying this kernel's callback type.
 */
static void tsi_gpio_set_void(struct gpio_chip *chip, unsigned int off, int value)
{
	tsi_gpio_set(chip, off, value);
}

static void tsi_gpio_set_multiple_void(struct gpio_chip *chip, unsigned long *mask,
					unsigned long *bits)
{
	tsi_gpio_set_multiple(chip, mask, bits);
}

/*
 * DS[2:0] nominal milliamps for PIN_CONFIG_DRIVE_STRENGTH (codes 0..5;
 * Samsung LN04LPP PBNT/PBFS 1.8V, rounded to nearest integer mA —
 * datasheet gives 2.5/3.7/5.0/6.2/7.5/10.0; codes 6/7 reserved).
 */
static const u8 tsi_gpio_ds_ma[] = { 3, 4, 5, 6, 8, 10 };

/*
 * tsi_gpio_set_config() - apply a single PIN_CONFIG_* to one pad.
 *
 * All settings live in the per-pad control register; each is a
 * single-field RMW that preserves all other control bits.
 * Supported: BIAS_PULL_UP, BIAS_PULL_DOWN, BIAS_DISABLE,
 * DRIVE_STRENGTH (3/4/5/6/8/10 mA only), INPUT_SCHMITT_ENABLE.
 * Note: OE=1 disables pulls in hardware regardless of PE/PS; callers
 * setting a pull on an output are not rejected, but the pad will not
 * pull until direction is switched to input.
 */
static int tsi_gpio_set_config(struct gpio_chip *chip, unsigned int off,
			       unsigned long config)
{
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	unsigned long param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	u32 mask, val;
	unsigned int i;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		mask = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		val  = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		mask = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		val  = TSI_SKYLP_GPIO_CTRL_PE;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		mask = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		val  = 0;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		for (i = 0; i < ARRAY_SIZE(tsi_gpio_ds_ma); i++) {
			if (tsi_gpio_ds_ma[i] == arg)
				break;
		}
		if (i == ARRAY_SIZE(tsi_gpio_ds_ma))
			return -EINVAL;
		mask = TSI_SKYLP_GPIO_CTRL_DS;
		val  = FIELD_PREP(TSI_SKYLP_GPIO_CTRL_DS, i);
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		mask = TSI_SKYLP_GPIO_CTRL_IS;
		val  = arg ? TSI_SKYLP_GPIO_CTRL_IS : 0;
		break;
	default:
		return -ENOTSUPP;
	}

	dev_dbg(gpio->chip.parent,
		"set_config(%s): param=%lu mask=0x%03x val=0x%03x\n",
		gpio->soc->lines[off].name, param, mask, val);
	return regmap_update_bits(gpio->regmap,
				  tsi_gpio_ctrl_reg(gpio, off), mask, val);
}

/* CSR-window offset of the group-N destination-enable register. */
static u32 tsi_gpio_irq_grp_en_reg(struct tsi_gpio *gpio)
{
	return gpio->base + gpio->soc->irq_grp0_en_off +
	       gpio->irq_grp * TSI_SKYLP_INTR_GRP_STRIDE;
}

/* CSR-window offset of the group-N masked-pending register. */
static u32 tsi_gpio_irq_grp_ip_reg(struct tsi_gpio *gpio)
{
	return gpio->base + gpio->soc->irq_grp0_ip_off +
	       gpio->irq_grp * TSI_SKYLP_INTR_GRP_STRIDE;
}

/*
 * tsi_gpio_irq_mask_hw() - clear the line's bit in our destination
 * group's enable register (enable_gN).
 *
 * Split from the irq_chip callback so it can be unit tested directly
 * against a fake regmap without a live irq_data/domain.
 */
VISIBLE_IF_KUNIT void tsi_gpio_irq_mask_hw(struct tsi_gpio *gpio, unsigned int hwirq)
{
	regmap_clear_bits(gpio->regmap, tsi_gpio_irq_grp_en_reg(gpio),
			  tsi_gpio_line_mask(gpio, hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_irq_mask_hw);

/*
 * tsi_gpio_irq_unmask_hw() - arm the line: set its bit in BOTH the
 * global per-source enable (int_enable_reg) and our destination
 * group's enable (enable_gN).
 *
 * int_enable_reg is shared by every destination group, not just ours
 * (see the open item [E] note at the top of this file) — setting it
 * is the one piece of this driver that reaches outside the group the
 * A520 was assigned and can observably affect the M85's view of the
 * same source until that ownership question is answered.
 */
VISIBLE_IF_KUNIT void tsi_gpio_irq_unmask_hw(struct tsi_gpio *gpio, unsigned int hwirq)
{
	u32 mask = tsi_gpio_line_mask(gpio, hwirq);

	regmap_set_bits(gpio->regmap,
			gpio->base + gpio->soc->irq_glben_off, mask);
	regmap_set_bits(gpio->regmap, tsi_gpio_irq_grp_en_reg(gpio), mask);
}
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_irq_unmask_hw);

/* tsi_gpio_irq_ack_hw() - write-1-clear the latched pending bit. */
VISIBLE_IF_KUNIT void tsi_gpio_irq_ack_hw(struct tsi_gpio *gpio, unsigned int hwirq)
{
	regmap_write(gpio->regmap, gpio->base + gpio->soc->irq_w1c_off,
		     tsi_gpio_line_mask(gpio, hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_irq_ack_hw);

static void tsi_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	unsigned int hwirq = irqd_to_hwirq(d);

	tsi_gpio_irq_mask_hw(gpio, hwirq);
	gpiochip_disable_irq(chip, hwirq);
}

static void tsi_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	unsigned int hwirq = irqd_to_hwirq(d);

	gpiochip_enable_irq(chip, hwirq);
	tsi_gpio_irq_unmask_hw(gpio, hwirq);
}

static void tsi_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct tsi_gpio *gpio = gpiochip_get_data(chip);

	tsi_gpio_irq_ack_hw(gpio, irqd_to_hwirq(d));
}

/*
 * tsi_gpio_irq_set_type() - hardware is level-sensitive only (confirmed
 * from RTL team 2026-08-16); no polarity register exists, so active-high
 * vs active-low cannot be programmed. handle_level_irq is wired at
 * registration unconditionally. Edge requests are rejected.
 */
VISIBLE_IF_KUNIT int tsi_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_irq_set_type);

static const struct irq_chip tsi_gpio_irq_chip = {
	.name		= "tsi-gpio",
	.irq_mask	= tsi_gpio_irq_mask,
	.irq_unmask	= tsi_gpio_irq_unmask,
	.irq_ack	= tsi_gpio_irq_ack,
	.irq_set_type	= tsi_gpio_irq_set_type,
	.flags		= IRQCHIP_IMMUTABLE | IRQCHIP_MASK_ON_SUSPEND,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

/*
 * tsi_gpio_irq_handler() - chained on the parent (GIC) IRQ. Reads the
 * masked-pending register for our destination group and dispatches
 * every set line; the collector's own bit numbering is the same as
 * the data-register bit, so tsi_gpio_line_mask() applies unchanged.
 */
static void tsi_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct tsi_gpio *gpio = gpiochip_get_data(chip);
	struct irq_chip *irqc = irq_desc_get_chip(desc);
	unsigned int i;
	u32 pending;
	int ret;

	chained_irq_enter(irqc, desc);

	ret = regmap_read(gpio->regmap, tsi_gpio_irq_grp_ip_reg(gpio),
			  &pending);
	if (ret) {
		dev_dbg(gpio->chip.parent,
			"irq_handler: ip_status_g%u read failed: %d\n",
			gpio->irq_grp, ret);
		goto out;
	}
	dev_dbg(gpio->chip.parent, "irq_handler: ip_status_g%u=0x%x\n",
		gpio->irq_grp, pending);

	for (i = 0; i < gpio->soc->nlines; i++) {
		if (pending & tsi_gpio_line_mask(gpio, i))
			generic_handle_domain_irq(chip->irq.domain, i);
	}

out:
	chained_irq_exit(irqc, desc);
}

/**
 * tsi_gpio_register() - build and register the gpio_chip for one corner.
 * @dev: device to hang devm allocations and the gpiochip on
 * @regmap: register access, normally the chiplet CSR regmap
 * @base: corner block offset within the CSR window (DT child "reg")
 * @soc: per-corner line table and data register offsets
 * @irq: parent IRQ to chain the collector on, or <= 0 for none (the
 *	default today: no DT node sets "interrupts", so this stays
 *	polling-only until open items A/B/C/E are resolved)
 * @irq_grp: destination group (0..3) whose ip_status/enable this
 *	instance reads and writes; caller-validated, ignored if irq <= 0
 *
 * Split out of probe so the KUnit test can drive the real chip ops
 * against a fake regmap without a tsi-chiplet parent device.
 *
 * Return: the driver instance, or an ERR_PTR on failure.
 */
VISIBLE_IF_KUNIT struct tsi_gpio *tsi_gpio_register(struct device *dev,
						    struct regmap *regmap,
						    u32 base,
						    const struct tsi_gpio_soc *soc,
						    int irq, u32 irq_grp)
{
	struct tsi_gpio *gpio;
	const char **names;
	unsigned int i;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return ERR_PTR(-ENOMEM);

	names = devm_kcalloc(dev, soc->nlines, sizeof(*names), GFP_KERNEL);
	if (!names)
		return ERR_PTR(-ENOMEM);
	for (i = 0; i < soc->nlines; i++)
		names[i] = soc->lines[i].name;

	gpio->regmap = regmap;
	gpio->base = base;
	gpio->soc = soc;
	gpio->irq_grp = irq_grp;

	gpio->chip.label = dev_name(dev);
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = soc->nlines;
	gpio->chip.names = names;
	gpio->chip.can_sleep = regmap_might_sleep(regmap);
	gpio->chip.get_direction = tsi_gpio_get_direction;
	gpio->chip.direction_input = tsi_gpio_direction_input;
	gpio->chip.direction_output = tsi_gpio_direction_output;
	gpio->chip.get = tsi_gpio_get;
	gpio->chip.get_multiple = tsi_gpio_get_multiple;
	gpio->chip.set = tsi_gpio_set_void;
	gpio->chip.set_multiple = tsi_gpio_set_multiple_void;
	gpio->chip.set_config = tsi_gpio_set_config;

	if (irq > 0) {
		struct gpio_irq_chip *girq = &gpio->chip.irq;

		dev_dbg(dev, "irqchip: chaining on parent irq %d, group %u\n",
			irq, irq_grp);
		gpio_irq_chip_set_chip(girq, &tsi_gpio_irq_chip);
		girq->parent_handler = tsi_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents)
			return ERR_PTR(-ENOMEM);
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		/* [C] fixed until HW sensitivity is confirmed. */
		girq->handler = handle_level_irq;
	}

	ret = devm_gpiochip_add_data(dev, &gpio->chip, gpio);
	if (ret) {
		dev_dbg(dev, "gpiochip_add_data failed: %d\n", ret);
		return ERR_PTR(ret);
	}

	dev_info(dev, "registered %u lines at CSR window offset 0x%x (%s..%s)\n",
		 soc->nlines, base, soc->lines[0].name,
		 soc->lines[soc->nlines - 1].name);
	return gpio;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_gpio_register);

/*
 * tsi_gpio_probe() - bind one corner instance declared as a chiplet
 * DT child. Resolves the corner table from the compatible, the regmap
 * from the tsi-chiplet parent, and the corner base from the child's
 * "reg". Defers until the chiplet core has probed.
 *
 * The IRQ path is entirely optional: platform_get_irq_optional()
 * returns <= 0 unless the DT node has an "interrupts" property, which
 * no shipped tsi-soc.dtsi node does today, so probing stays identical
 * to before this was added until a node is deliberately updated (see
 * the file header for what must be confirmed first).
 */
static int tsi_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct tsi_gpio_soc *soc;
	struct tsi_chiplet *chiplet;
	struct tsi_gpio *gpio;
	u32 base, irq_grp = 0;
	int irq, ret;

	dev_dbg(dev, "probe: compatible=%s\n",
		dev_of_node(dev) ? dev_of_node(dev)->name : "<none>");

	soc = device_get_match_data(dev);
	if (!soc)
		return -ENODEV;

	chiplet = tsi_chiplet_get(dev);
	if (IS_ERR(chiplet))
		return dev_err_probe(dev, PTR_ERR(chiplet),
				     "no tsi-chiplet parent\n");

	/* Corner base within the chiplet CSR window (child "reg"). */
	ret = of_property_read_u32(dev->of_node, "reg", &base);
	if (ret)
		return dev_err_probe(dev, ret, "missing reg\n");

	dev_dbg(dev, "probe: chiplet regmap ready, corner base=0x%x, nlines=%u\n",
		base, soc->nlines);

	/* G2 and G3 are wired to the A520 GIC; default 0 is safe (polling). */
	device_property_read_u32(dev, "tsi,irq-dest-group", &irq_grp);
	if (irq_grp > 3)
		return dev_err_probe(dev, -EINVAL,
				     "tsi,irq-dest-group must be 0..3\n");

	irq = platform_get_irq_optional(pdev, 0);
	if (irq == -ENXIO)
		irq = 0;
	else if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get optional IRQ\n");

	if (irq > 0)
		dev_dbg(dev, "probe: interrupts property present, irq=%d\n",
			irq);

	gpio = tsi_gpio_register(dev, chiplet->regmap, base, soc, irq,
				 irq_grp);
	if (IS_ERR(gpio))
		dev_dbg(dev, "probe: tsi_gpio_register failed: %ld\n",
			PTR_ERR(gpio));
	return PTR_ERR_OR_ZERO(gpio);
}

static const struct of_device_id tsi_gpio_of_match[] = {
	{ .compatible = "tsi,skylp-gpio-ionw", .data = &tsi_gpio_ionw_soc },
	{ .compatible = "tsi,skylp-gpio-ione", .data = &tsi_gpio_ione_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, tsi_gpio_of_match);

static struct platform_driver tsi_gpio_driver = {
	.probe = tsi_gpio_probe,
	.driver = {
		.name = "tsi-gpio",
		.of_match_table = tsi_gpio_of_match,
	},
};
module_platform_driver(tsi_gpio_driver);

MODULE_AUTHOR("Tsavorite Scalable Intelligence");
MODULE_DESCRIPTION("TSI SkyLP GPIO controller driver");
MODULE_LICENSE("GPL");
