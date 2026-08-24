// SPDX-License-Identifier: GPL-2.0-only
/*
 * TSI SkyLP IO-corner pinctrl + GPIO driver.
 *
 * One instance drives one SkyLP IO corner. A corner owns a set of pads;
 * each pad has a <pad>_control register (electrical config: direction,
 * bias, drive strength, schmitt) and, if it is GPIO-capable, one bit in
 * the corner's shared gpio_data_out/gpio_data_in pair.
 *
 * Most pads are multiplexed: an iomode register selects which on-chip
 * function owns a whole group of pads (SPI/QSPI/OSPI, I2C/SMB, I3C,
 * UART, I2S, PCIe PERST/MDIO, LED, PWM) or GPIO. Muxing is therefore
 * per-group, never per-pad - see tsi_pinctrl_mux_set().
 *
 * Register layout, pad->bit assignment and iomode encodings are taken
 * from ral.json release SKYLP_G0829; the pad tables below are generated
 * from it (see the GENERATED banner) because the data bit of a pad is
 * not its index: IONW GPIO_4..7 are bits 21..24, its SPI pads 9..20.
 *
 * Scope: this driver serves the A520 (Linux) view of the corners. The
 * M85 management core runs its own OS and may own some pads; nothing
 * here arbitrates against it, so DT must only expose pads assigned to
 * Linux.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/visibility.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/soc/tsi/tsi-chiplet.h>
#include <linux/soc/tsi/tsi-skylp-regs.h>

#include "pinctrl-tsi.h"

#define TSI_FUNC_GPIO		"gpio"

#define TSI_PAD(_n, _off)	{ .name = (_n), .ctrl_off = (_off) }

#define TSI_GROUP_MUX(_name, _pins, _off, _lsb, _width, _opts)	\
	{							\
		.name = (_name),				\
		.pins = (_pins),				\
		.npins = ARRAY_SIZE(_pins),			\
		.iomode_off = (_off),				\
		.lsb = (_lsb),					\
		.width = (_width),				\
		.opts = (_opts),				\
		.nopts = ARRAY_SIZE(_opts),			\
	}

/* A dedicated group: GPIO only, no iomode register. */
#define TSI_GROUP_GPIO(_name, _pins)				\
	{							\
		.name = (_name),				\
		.pins = (_pins),				\
		.npins = ARRAY_SIZE(_pins),			\
	}

/*
 * GENERATED from ral.json release SKYLP_G0829. Pin number == the pad's
 * bit in gpio_data_out/gpio_data_in. Do not hand-edit; regenerate when
 * the RAL release changes.
 *
 * Pads that have a <pad>_control register but no data bit are NOT
 * GPIO-capable and are deliberately absent: IONE pll_fout, IOSE
 * strap0..6 (Chip_ID/CHIP_PRIM/PKG_PRIM), IOSW soc_jtag_*, soc_cjtag_*
 * and tpi_*. The IO list marks some of those "GPIO: Yes", but the RAL
 * data registers have no bits for them, so they cannot be driven or
 * sampled as GPIO. They remain configurable only through their control
 * register, which this driver does not expose (open item).
 */

/* ---------------- IONW (CF802), corner base 0x13000000 ---------------- */

static const struct tsi_pin tsi_ionw_pins[] = {
	TSI_PAD("GPIO_0",	0x11c),
	TSI_PAD("GPIO_1",	0x120),
	TSI_PAD("GPIO_2",	0x124),
	TSI_PAD("GPIO_3",	0x128),
	TSI_PAD("PCLKREQ_IO",	0x118),
	TSI_PAD("PWAKE_IO",	0x114),
	TSI_PAD("TLP_PERST_0",	0x104),
	TSI_PAD("TLP_PERST_1",	0x108),
	TSI_PAD("TLP_PERST_3",	0x110),
	TSI_PAD("SPI_IO0",	0x144),
	TSI_PAD("SPI_IO1",	0x148),
	TSI_PAD("SPI_IO2",	0x14c),
	TSI_PAD("SPI_IO3",	0x150),
	TSI_PAD("SPI_IO4",	0x154),
	TSI_PAD("SPI_IO5",	0x158),
	TSI_PAD("SPI_IO6",	0x15c),
	TSI_PAD("SPI_IO7",	0x160),
	TSI_PAD("SPI_IO8",	0x164),
	TSI_PAD("SPI_IO9",	0x168),
	TSI_PAD("SPI_SCLK",	0x140),
	TSI_PAD("SPI_SCS",	0x13c),
	TSI_PAD("GPIO_4",	0x12c),
	TSI_PAD("GPIO_5",	0x130),
	TSI_PAD("GPIO_6",	0x134),
	TSI_PAD("GPIO_7",	0x138),
	TSI_PAD("TLP_PERST_2",	0x10c),
};

static const unsigned int tsi_ionw_gpio_pins[] = { 0, 1, 2, 3, 21, 22, 23, 24 };
static const unsigned int tsi_ionw_pcie_pins[] = { 4, 5 };
static const unsigned int tsi_ionw_perst01_pins[] = { 6, 7 };
static const unsigned int tsi_ionw_perst23_pins[] = { 8, 25 };
static const unsigned int tsi_ionw_spi_pins[] = {
	9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
};

/* pcie_iomode[0]: 0 = PCIE CLKREQ/WAKE, 1 = GPIO */
static const struct tsi_mux_opt tsi_ionw_pcie_opts[] = {
	{ "pcie", 0 }, { TSI_FUNC_GPIO, 1 },
};

/* prst_iomode[1:0]: 00 = PERST, 01 = MDIO, 10 = GPIO */
static const struct tsi_mux_opt tsi_ionw_perst01_opts[] = {
	{ "perst", 0 }, { "mdio", 1 }, { TSI_FUNC_GPIO, 2 },
};

/* prst_iomode_1[0]: 0 = PERST, 1 = GPIO */
static const struct tsi_mux_opt tsi_ionw_perst23_opts[] = {
	{ "perst", 0 }, { TSI_FUNC_GPIO, 1 },
};

/*
 * spi_iomode[2:0]: 000 = OSPI + 2 CS, 001 = ARM_SPI + QSPI + 2 CS,
 * 010 = QSPI x2 (reset default), 011 = ARM_SPI + DSPI x2,
 * 100 = ARM_SPI + SPI x2, 101 = GPIO. 110/111 undefined in RAL.
 */
static const struct tsi_mux_opt tsi_ionw_spi_opts[] = {
	{ "ospi",		0 },
	{ "armspi_qspi",	1 },
	{ "qspi",		2 },
	{ "armspi_dspi",	3 },
	{ "armspi_spi",		4 },
	{ TSI_FUNC_GPIO,	5 },
};

static const struct tsi_group tsi_ionw_groups[] = {
	TSI_GROUP_GPIO("gpio", tsi_ionw_gpio_pins),
	TSI_GROUP_MUX("pcie", tsi_ionw_pcie_pins,
		      0x188, 0, 1, tsi_ionw_pcie_opts),
	TSI_GROUP_MUX("perst01", tsi_ionw_perst01_pins,
		      0x180, 0, 2, tsi_ionw_perst01_opts),
	TSI_GROUP_MUX("perst23", tsi_ionw_perst23_pins,
		      0x184, 0, 1, tsi_ionw_perst23_opts),
	TSI_GROUP_MUX("spi", tsi_ionw_spi_pins,
		      0x18c, 0, 3, tsi_ionw_spi_opts),
};

VISIBLE_IF_KUNIT const struct tsi_pinctrl_soc tsi_skylp_ionw_soc = {
	.label		= "ionw",
	.pins		= tsi_ionw_pins,
	.npins		= ARRAY_SIZE(tsi_ionw_pins),
	.groups		= tsi_ionw_groups,
	.ngroups	= ARRAY_SIZE(tsi_ionw_groups),
	.data_out_off	= 0x178,
	.data_in_off	= 0x17c,
	/* t2 collector: 26 gpio_intr sources, nothing else. */
	.irq_w1c_off	= 0x34c4,
	.irq_glben_off	= 0x34cc,
	.irq_grp0_ip_off = 0x34d4,
	.irq_grp0_en_off = 0x34d8,
	.irq_bit_shift	= 0,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_skylp_ionw_soc);

/* ---------------- IONE (CF803), corner base 0x6000000 ----------------- */

static const struct tsi_pin tsi_ione_pins[] = {
	TSI_PAD("I2C_SMB_SCL_0",	0x134),
	TSI_PAD("I2C_SMB_SDA_0",	0x138),
	TSI_PAD("I2C_SMB_SCL_1",	0x13c),
	TSI_PAD("I2C_SMB_SDA_1",	0x140),
	TSI_PAD("I2C_SMB_SCL_2",	0x144),
	TSI_PAD("I2C_SMB_SDA_2",	0x148),
	TSI_PAD("I2C_SMB_SCL_3",	0x14c),
	TSI_PAD("I2C_SMB_SDA_3",	0x150),
	TSI_PAD("I2C_I3C_SCL_3",	0x154),
	TSI_PAD("I2C_I3C_SDA_3",	0x158),
	TSI_PAD("GPIO_FS_0",		0x124),
	TSI_PAD("GPIO_FS_1",		0x128),
	TSI_PAD("GPIO_FS_2",		0x12c),
	TSI_PAD("GPIO_FS_3",		0x130),
	TSI_PAD("UART_RTS_1",		0x11c),
	TSI_PAD("UART_RX_0",		0x104),
	TSI_PAD("UART_CTS_0",		0x108),
	TSI_PAD("UART_TX_0",		0x10c),
	TSI_PAD("UART_RTS_0",		0x110),
	TSI_PAD("UART_RX_1",		0x114),
	TSI_PAD("UART_TX_1",		0x118),
	TSI_PAD("LED_0",		0x15c),
	TSI_PAD("LED_1",		0x160),
	TSI_PAD("FAN_PWM",		0x164),
	TSI_PAD("UART_CTS_1",		0x120),
};

static const unsigned int tsi_ione_gpio_fs_pins[] = { 10, 11, 12, 13 };
static const unsigned int tsi_ione_smb_pins[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
static const unsigned int tsi_ione_i3c_pins[] = { 8, 9 };
static const unsigned int tsi_ione_uart0_pins[] = { 15, 16, 17, 18 };
static const unsigned int tsi_ione_uart1_pins[] = { 14, 19, 20, 24 };
static const unsigned int tsi_ione_led_pins[] = { 21, 22 };
static const unsigned int tsi_ione_pwm_pins[] = { 23 };

/* smb_iomode[1:0]: 00 = SMB, 01 = I2C, 10 = GPIO, 11 = reserved */
static const struct tsi_mux_opt tsi_ione_smb_opts[] = {
	{ "smb", 0 }, { "i2c", 1 }, { TSI_FUNC_GPIO, 2 },
};

/* i3c_iomode[0]: 0 = I3C, 1 = GPIO */
static const struct tsi_mux_opt tsi_ione_i3c_opts[] = {
	{ "i3c", 0 }, { TSI_FUNC_GPIO, 1 },
};

/* uart_[01]_iomode[0]: 0 = UART, 1 = GPIO */
static const struct tsi_mux_opt tsi_ione_uart_opts[] = {
	{ "uart", 0 }, { TSI_FUNC_GPIO, 1 },
};

/* led_iomode[0]: 0 = LED, 1 = GPIO */
static const struct tsi_mux_opt tsi_ione_led_opts[] = {
	{ "led", 0 }, { TSI_FUNC_GPIO, 1 },
};

/* pwm_iomode[0]: 0 = PWM, 1 = GPIO */
static const struct tsi_mux_opt tsi_ione_pwm_opts[] = {
	{ "pwm", 0 }, { TSI_FUNC_GPIO, 1 },
};

/*
 * smb_iomode1 (0x170) selects SMB_ALERT vs GPIO with INVERTED polarity
 * (0 = GPIO, 1 = SMB) relative to every other iomode register. It is
 * not modelled: the RAL gpio_data_out bit map has no SMB_ALERT pad, so
 * there is no line to expose. Do not copy the polarity of the groups
 * above if that pad is added later.
 */

static const struct tsi_group tsi_ione_groups[] = {
	TSI_GROUP_GPIO("gpio_fs", tsi_ione_gpio_fs_pins),
	TSI_GROUP_MUX("smb", tsi_ione_smb_pins,
		      0x16c, 0, 2, tsi_ione_smb_opts),
	TSI_GROUP_MUX("i3c", tsi_ione_i3c_pins,
		      0x174, 0, 1, tsi_ione_i3c_opts),
	TSI_GROUP_MUX("uart0", tsi_ione_uart0_pins,
		      0x178, 0, 1, tsi_ione_uart_opts),
	TSI_GROUP_MUX("uart1", tsi_ione_uart1_pins,
		      0x17c, 0, 1, tsi_ione_uart_opts),
	TSI_GROUP_MUX("led", tsi_ione_led_pins,
		      0x180, 0, 1, tsi_ione_led_opts),
	TSI_GROUP_MUX("pwm", tsi_ione_pwm_pins,
		      0x184, 0, 1, tsi_ione_pwm_opts),
};

VISIBLE_IF_KUNIT const struct tsi_pinctrl_soc tsi_skylp_ione_soc = {
	.label		= "ione",
	.pins		= tsi_ione_pins,
	.npins		= ARRAY_SIZE(tsi_ione_pins),
	.groups		= tsi_ione_groups,
	.ngroups	= ARRAY_SIZE(tsi_ione_groups),
	.data_out_off	= 0x188,
	.data_in_off	= 0x18c,
	/* intr_regs_8 collector: 25 gpio_intr sources, nothing else. */
	.irq_w1c_off	= 0xc344,
	.irq_glben_off	= 0xc34c,
	.irq_grp0_ip_off = 0xc354,
	.irq_grp0_en_off = 0xc358,
	.irq_bit_shift	= 0,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_skylp_ione_soc);

/* ---------------- IOSE (CF801), corner base 0x9000000 ----------------- */

static const struct tsi_pin tsi_iose_pins[] = {
	TSI_PAD("TLQ_PERST_0",	0x120),
	TSI_PAD("TLQ_PERST_1",	0x124),
};

static const unsigned int tsi_iose_perst_pins[] = { 0, 1 };

/* prst_iomode[1:0]: 00 = PCIE PERST, 01 = MDIO, 10 = GPIO */
static const struct tsi_mux_opt tsi_iose_perst_opts[] = {
	{ "perst", 0 }, { "mdio", 1 }, { TSI_FUNC_GPIO, 2 },
};

static const struct tsi_group tsi_iose_groups[] = {
	TSI_GROUP_MUX("perst", tsi_iose_perst_pins,
		      0x130, 0, 2, tsi_iose_perst_opts),
};

VISIBLE_IF_KUNIT const struct tsi_pinctrl_soc tsi_skylp_iose_soc = {
	.label		= "iose",
	.pins		= tsi_iose_pins,
	.npins		= ARRAY_SIZE(tsi_iose_pins),
	.groups		= tsi_iose_groups,
	.ngroups	= ARRAY_SIZE(tsi_iose_groups),
	.data_out_off	= 0x128,
	.data_in_off	= 0x12c,
	/*
	 * intr_regs collector: bit 0 is mdio_intr, so the two gpio_intr
	 * sources are bits 1 and 2. Touching bit 0 here would disturb the
	 * MDIO interrupt, hence the shift.
	 */
	.irq_w1c_off	= 0x2044,
	.irq_glben_off	= 0x204c,
	.irq_grp0_ip_off = 0x2054,
	.irq_grp0_en_off = 0x2058,
	.irq_bit_shift	= 1,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_skylp_iose_soc);

/* ---------------- IOSW (CF800), corner base 0x19000000 ---------------- */

static const struct tsi_pin tsi_iosw_pins[] = {
	TSI_PAD("I2C_I3C_SCL_0",	0x14c),
	TSI_PAD("I2C_I3C_SDA_0",	0x150),
	TSI_PAD("I2C_I3C_SCL_1",	0x154),
	TSI_PAD("I2C_I3C_SDA_1",	0x158),
	TSI_PAD("I2C_I3C_SCL_2",	0x15c),
	TSI_PAD("I2C_I3C_SDA_2",	0x160),
	TSI_PAD("I2S_SCK",		0x164),
	TSI_PAD("I2S_WS",		0x16c),
	TSI_PAD("I2S_SDI_0",		0x168),
	TSI_PAD("I2S_SDO_0",		0x170),
	TSI_PAD("I2S_SDO_1",		0x178),
	TSI_PAD("I2S_SDI_1",		0x174),
};

static const unsigned int tsi_iosw_i3c_pins[] = { 0, 1, 2, 3, 4, 5 };
static const unsigned int tsi_iosw_i2s_pins[] = { 6, 7, 8, 9, 10, 11 };

/* i3c_iomode[0]: 0 = I3C, 1 = GPIO */
static const struct tsi_mux_opt tsi_iosw_i3c_opts[] = {
	{ "i3c", 0 }, { TSI_FUNC_GPIO, 1 },
};

/* i2s_iomode[0]: 0 = I2S, 1 = GPIO */
static const struct tsi_mux_opt tsi_iosw_i2s_opts[] = {
	{ "i2s", 0 }, { TSI_FUNC_GPIO, 1 },
};

static const struct tsi_group tsi_iosw_groups[] = {
	TSI_GROUP_MUX("i3c", tsi_iosw_i3c_pins,
		      0x180, 0, 1, tsi_iosw_i3c_opts),
	TSI_GROUP_MUX("i2s", tsi_iosw_i2s_pins,
		      0x17c, 0, 1, tsi_iosw_i2s_opts),
};

VISIBLE_IF_KUNIT const struct tsi_pinctrl_soc tsi_skylp_iosw_soc = {
	.label		= "iosw",
	.pins		= tsi_iosw_pins,
	.npins		= ARRAY_SIZE(tsi_iosw_pins),
	.groups		= tsi_iosw_groups,
	.ngroups	= ARRAY_SIZE(tsi_iosw_groups),
	.data_out_off	= 0x184,
	.data_in_off	= 0x188,
	/* intr_regs_2 collector: 12 gpio_intr sources, nothing else. */
	.irq_w1c_off	= 0x90c4,
	.irq_glben_off	= 0x90cc,
	.irq_grp0_ip_off = 0x90d4,
	.irq_grp0_en_off = 0x90d8,
	.irq_bit_shift	= 0,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_skylp_iosw_soc);

/* ------------------------- register helpers -------------------------- */

/*
 * CSR-window address of a pad's control register. Pad offsets in the
 * tables are corner-relative, so the corner base has to be added on
 * every access; nothing in this driver holds an absolute address.
 */
static u32 tsi_ctrl_reg(struct tsi_pinctrl *tp, unsigned int pin)
{
	return tp->base + tp->soc->pins[pin].ctrl_off;
}

/*
 * Size of the corner-relative window this driver actually touches: one
 * past the highest register in the corner's tables.
 *
 * Register offsets are properties of the IP, so they live in the tables
 * and are keyed off "compatible" rather than described in DT. But the
 * DT node still declares how big its window is, and nothing otherwise
 * checks that the declaration covers the tables - the parent chiplet
 * regmap spans the whole chiplet, so a short window would be written
 * straight through without complaint. Computing the requirement lets
 * probe reject that mismatch instead of trusting it.
 */
VISIBLE_IF_KUNIT u32 tsi_pinctrl_reg_span(const struct tsi_pinctrl_soc *soc)
{
	u32 hi = 0;
	unsigned int i;

	for (i = 0; i < soc->npins; i++)
		hi = max_t(u32, hi, soc->pins[i].ctrl_off);

	for (i = 0; i < soc->ngroups; i++)
		hi = max_t(u32, hi, soc->groups[i].iomode_off);

	hi = max_t(u32, hi, soc->data_out_off);
	hi = max_t(u32, hi, soc->data_in_off);
	hi = max_t(u32, hi, soc->irq_w1c_off);
	hi = max_t(u32, hi, soc->irq_glben_off);

	/* ip_status/enable repeat per destination group, g0..g3. */
	hi = max_t(u32, hi, soc->irq_grp0_ip_off +
			    3 * TSI_SKYLP_INTR_GRP_STRIDE);
	hi = max_t(u32, hi, soc->irq_grp0_en_off +
			    3 * TSI_SKYLP_INTR_GRP_STRIDE);

	return hi + 4;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_reg_span);

/* ---------------------------- pinctrl ops ---------------------------- */

/*
 * Group enumeration for the pinctrl core. Groups are fixed per corner
 * (they mirror the hardware's iomode registers, which cannot be
 * reconfigured), so all three hooks are plain lookups into the corner
 * table and the selector is an index into it.
 */
static int tsi_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	return tp->soc->ngroups;
}

/* Name of group @selector, as used in a DT "groups" property. */
static const char *tsi_get_group_name(struct pinctrl_dev *pctldev,
				      unsigned int selector)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	return tp->soc->groups[selector].name;
}

/*
 * Pads belonging to group @selector. The pin numbers handed back are
 * data-register bits, which is what the rest of the driver and the
 * gpio_chip both index by.
 */
static int tsi_get_group_pins(struct pinctrl_dev *pctldev,
			      unsigned int selector,
			      const unsigned int **pins,
			      unsigned int *npins)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	*pins = tp->soc->groups[selector].pins;
	*npins = tp->soc->groups[selector].npins;
	return 0;
}

/*
 * Defined further down, with the pinconf helpers: the debugfs line
 * decodes the pad's electrical state as well as its function, so it
 * needs the drive-strength table.
 */
static void tsi_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			     unsigned int pin);

VISIBLE_IF_KUNIT const struct pinctrl_ops tsi_pinctrl_ops = {
	.get_groups_count	= tsi_get_groups_count,
	.get_group_name		= tsi_get_group_name,
	.get_group_pins		= tsi_get_group_pins,
	.pin_dbg_show		= tsi_pin_dbg_show,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinconf_generic_dt_free_map,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_ops);

/* ----------------------------- pinmux ops ---------------------------- */

/*
 * Write a group's iomode field. Every pad in the group switches at once;
 * the hardware has no per-pad function select. Callers that only intend
 * to move one line must accept that its siblings move too, which is why
 * DT should describe a whole group as one function.
 */
VISIBLE_IF_KUNIT int tsi_pinctrl_mux_set(struct tsi_pinctrl *tp, const struct tsi_group *grp,
			u8 val)
{
	u32 mask;

	if (!grp->iomode_off)
		return -ENOTSUPP;

	mask = GENMASK(grp->lsb + grp->width - 1, grp->lsb);

	dev_dbg(tp->dev, "mux %s/%s: iomode +0x%03x <- %u (mask 0x%x)\n",
		tp->soc->label, grp->name, grp->iomode_off, val, mask);

	return regmap_update_bits(tp->regmap, tp->base + grp->iomode_off,
				  mask, (u32)val << grp->lsb);
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_mux_set);

/*
 * Find the group a pad belongs to. Returns the first match, which is
 * unambiguous only because a pad appears in exactly one group; the
 * KUnit suite asserts that partitioning for every corner.
 */
VISIBLE_IF_KUNIT const struct tsi_group *tsi_pinctrl_group_of_pin(struct tsi_pinctrl *tp,
						 unsigned int pin)
{
	unsigned int g, i;

	for (g = 0; g < tp->soc->ngroups; g++) {
		const struct tsi_group *grp = &tp->soc->groups[g];

		for (i = 0; i < grp->npins; i++)
			if (grp->pins[i] == pin)
				return grp;
	}
	return NULL;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_group_of_pin);

/*
 * Function enumeration for the pinctrl core. Unlike groups, the
 * function list is not a static table: it is derived at probe from the
 * groups' mux options by tsi_build_funcs(), so these read tp->funcs
 * rather than the corner description.
 */
static int tsi_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	return tp->nfuncs;
}

/* Name of function @selector, as used in a DT "function" property. */
static const char *tsi_get_function_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	return tp->funcs[selector].name;
}

/*
 * Groups that can be muxed to function @selector. Most functions are
 * offered by exactly one group because each iomode register covers one
 * peripheral, but "gpio" is offered by every group that has a GPIO
 * mode plus the dedicated ones.
 */
static int tsi_get_function_groups(struct pinctrl_dev *pctldev,
				   unsigned int selector,
				   const char * const **groups,
				   unsigned int * const ngroups)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	*groups = tp->funcs[selector].groups;
	*ngroups = tp->funcs[selector].ngroups;
	return 0;
}

/*
 * Point a group at one of its functions, which is how a DT pinmux node
 * takes effect. The function has to be one the group actually offers:
 * the iomode encodings differ per group, so there is no chip-wide
 * function-to-value mapping to fall back on.
 */
static int tsi_set_mux(struct pinctrl_dev *pctldev, unsigned int func_sel,
		       unsigned int grp_sel)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);
	const struct tsi_group *grp = &tp->soc->groups[grp_sel];
	const char *func = tp->funcs[func_sel].name;
	unsigned int i;

	/*
	 * A dedicated group has no iomode register: the only function it
	 * can offer is GPIO, and selecting that is a no-op rather than an
	 * error so a DT node may name it explicitly.
	 */
	if (!grp->iomode_off)
		return strcmp(func, TSI_FUNC_GPIO) ? -EINVAL : 0;

	for (i = 0; i < grp->nopts; i++)
		if (!strcmp(grp->opts[i].func, func))
			return tsi_pinctrl_mux_set(tp, grp, grp->opts[i].val);

	return -EINVAL;
}

/*
 * Switch the group owning @pin to GPIO. Invoked by the pinctrl core from
 * gpiochip_generic_request(), so a consumer doing gpiod_get() on a muxed
 * pad gets the mux moved for it.
 */
VISIBLE_IF_KUNIT int tsi_pinctrl_gpio_enable(struct tsi_pinctrl *tp, unsigned int pin)
{
	const struct tsi_group *grp = tsi_pinctrl_group_of_pin(tp, pin);
	unsigned int i;

	if (!grp)
		return -EINVAL;

	/* Dedicated pad: already GPIO, nothing to switch. */
	if (!grp->iomode_off)
		return 0;

	for (i = 0; i < grp->nopts; i++) {
		if (strcmp(grp->opts[i].func, TSI_FUNC_GPIO))
			continue;

		if (grp->npins > 1)
			dev_dbg(tp->dev,
				"gpio_request_enable(%s): moving all %u pads of group %s to GPIO\n",
				tp->soc->pins[pin].name, grp->npins,
				grp->name);

		return tsi_pinctrl_mux_set(tp, grp, grp->opts[i].val);
	}

	return -ENOTSUPP;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_gpio_enable);

/*
 * pinmux_ops hook behind gpiochip_generic_request(). Keeping the work
 * in tsi_pinctrl_gpio_enable() lets the KUnit suite drive it without a
 * pinctrl_dev or a gpio range.
 */
static int tsi_gpio_request_enable(struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned int pin)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);

	return tsi_pinctrl_gpio_enable(tp, pin);
}

VISIBLE_IF_KUNIT const struct pinmux_ops tsi_pinmux_ops = {
	.get_functions_count	= tsi_get_functions_count,
	.get_function_name	= tsi_get_function_name,
	.get_function_groups	= tsi_get_function_groups,
	.set_mux		= tsi_set_mux,
	.gpio_request_enable	= tsi_gpio_request_enable,
	/*
	 * Strict, because the mux is group-wide. Without this the core
	 * lets a GPIO be claimed on a pad a peripheral already owns, and
	 * gpio_request_enable() would then move the ENTIRE group to GPIO -
	 * silently pulling the other eleven SPI pads (or all eight I2C/SMB
	 * pads) out from under a running driver. Strict makes the core
	 * refuse that request loudly instead, in either direction: no GPIO
	 * on a muxed-away pad, no muxing away a pad held as GPIO.
	 *
	 * The cost is that a DT node cannot both select function "gpio" on
	 * a group and have a consumer gpiod_get() one of its pads: the pin
	 * would already have a mux owner. That combination is redundant
	 * here anyway, since gpio_request_enable() moves the mux for the
	 * consumer without any DT pinmux node at all.
	 */
	.strict			= true,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_pinmux_ops);

/* ----------------------------- pinconf ops --------------------------- */

/*
 * DS[2:0] nominal milliamps (codes 0..5; Samsung LN04LPP PBNT/PBFS
 * 1.8V, rounded from 2.5/3.7/5.0/6.2/7.5/10.0). Codes 6/7 reserved.
 */
static const u8 tsi_ds_ma[] = { 3, 4, 5, 6, 8, 10 };

/*
 * Decode an iomode field value back into the function it selects, or
 * NULL when the group defines nothing for that value. The RAL leaves
 * some encodings undefined (spi_iomode 110 and 111), and firmware or the
 * M85 may have left one there, so callers must handle NULL rather than
 * assume the field only ever holds values this driver knows about.
 */
VISIBLE_IF_KUNIT const char *tsi_pinctrl_func_of_val(const struct tsi_group *grp,
						     u8 val)
{
	unsigned int i;

	/* A dedicated group has no field to decode; it is always GPIO. */
	if (!grp->iomode_off)
		return TSI_FUNC_GPIO;

	for (i = 0; i < grp->nopts; i++)
		if (grp->opts[i].val == val)
			return grp->opts[i].func;

	return NULL;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_func_of_val);

/*
 * One debugfs line per pin, under pinctrl/<dev>/pins. Worth the code on
 * this driver in particular: a pin number is a data-register bit rather
 * than a position in the pad map, the function is owned by the whole
 * group, and the pad's electrical state lives in a different register
 * again. Showing all three together is what makes a bring-up mismatch
 * obvious instead of a register hunt.
 */
static void tsi_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			     unsigned int pin)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);
	const struct tsi_group *grp = tsi_pinctrl_group_of_pin(tp, pin);
	const char *func;
	u32 ctrl, mode, ds;
	int ret;

	seq_printf(s, "%s ctrl +0x%03x", tp->soc->label,
		   tp->soc->pins[pin].ctrl_off);

	if (!grp) {
		seq_puts(s, " (in no group)");
		return;
	}

	seq_printf(s, " group %s", grp->name);

	if (!grp->iomode_off) {
		seq_puts(s, " func gpio (dedicated)");
	} else {
		ret = regmap_read(tp->regmap, tp->base + grp->iomode_off,
				  &mode);
		if (ret) {
			seq_printf(s, " iomode <err %d>", ret);
		} else {
			mode = (mode >> grp->lsb) & (BIT(grp->width) - 1);
			func = tsi_pinctrl_func_of_val(grp, mode);
			seq_printf(s, " func %s (iomode +0x%03x = %u)",
				   func ?: "<undefined>", grp->iomode_off,
				   mode);
		}
	}

	ret = regmap_read(tp->regmap, tsi_ctrl_reg(tp, pin), &ctrl);
	if (ret) {
		seq_printf(s, " ctrl <err %d>", ret);
		return;
	}

	if (ctrl & TSI_SKYLP_GPIO_CTRL_OE)
		seq_puts(s, " OE");
	if (ctrl & TSI_SKYLP_GPIO_CTRL_IE)
		seq_puts(s, " IE");
	if (!(ctrl & (TSI_SKYLP_GPIO_CTRL_OE | TSI_SKYLP_GPIO_CTRL_IE)))
		seq_puts(s, " hi-z");

	ds = FIELD_GET(TSI_SKYLP_GPIO_CTRL_DS, ctrl);
	if (ds < ARRAY_SIZE(tsi_ds_ma))
		seq_printf(s, " ds %umA", tsi_ds_ma[ds]);
	else
		seq_printf(s, " ds reserved(%u)", ds);

	if (ctrl & TSI_SKYLP_GPIO_CTRL_PE)
		seq_printf(s, " pull-%s",
			   ctrl & TSI_SKYLP_GPIO_CTRL_PS ? "up" : "down");
	if (ctrl & TSI_SKYLP_GPIO_CTRL_IS)
		seq_puts(s, " schmitt");
}

/*
 * Read back one PIN_CONFIG_* from a pad's control register.
 *
 * The bias parameters are queries rather than values: pinconf expects
 * -EINVAL when the pad is not in the state being asked about, so
 * BIAS_PULL_UP on a pulled-down pad is an error, not "0".
 */
static int tsi_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			   unsigned long *config)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);
	unsigned long param = pinconf_to_config_param(*config);
	u32 ctrl, arg, i;
	int ret;

	ret = regmap_read(tp->regmap, tsi_ctrl_reg(tp, pin), &ctrl);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (ctrl & TSI_SKYLP_GPIO_CTRL_PE)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(ctrl & TSI_SKYLP_GPIO_CTRL_PE) ||
		    !(ctrl & TSI_SKYLP_GPIO_CTRL_PS))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!(ctrl & TSI_SKYLP_GPIO_CTRL_PE) ||
		    (ctrl & TSI_SKYLP_GPIO_CTRL_PS))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		/*
		 * DS codes 6 and 7 are reserved and have no documented
		 * current. Report that rather than folding them back into
		 * the table: a pad left at a reserved code by firmware or
		 * by the M85 would otherwise read back as a perfectly
		 * ordinary drive strength.
		 */
		i = FIELD_GET(TSI_SKYLP_GPIO_CTRL_DS, ctrl);
		if (i >= ARRAY_SIZE(tsi_ds_ma))
			return -EINVAL;
		arg = tsi_ds_ma[i];
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		arg = !!(ctrl & TSI_SKYLP_GPIO_CTRL_IS);
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = !!(ctrl & TSI_SKYLP_GPIO_CTRL_OE);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = !!(ctrl & TSI_SKYLP_GPIO_CTRL_IE);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

/*
 * Apply one PIN_CONFIG_* to one pad. Every setting is a single-field RMW
 * of the pad's control register, so unrelated bits survive.
 *
 * Note: OE=1 disables the pad's pull in hardware regardless of PE/PS.
 * Setting a bias on an output is accepted but has no effect until the
 * pad is switched to input.
 */
static int tsi_pinconf_set_one(struct tsi_pinctrl *tp, unsigned int pin,
			       unsigned long config)
{
	unsigned long param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	u32 mask, val;
	unsigned int i;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		mask = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		val = mask;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		mask = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		val = TSI_SKYLP_GPIO_CTRL_PE;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		mask = TSI_SKYLP_GPIO_CTRL_PE | TSI_SKYLP_GPIO_CTRL_PS;
		val = 0;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		for (i = 0; i < ARRAY_SIZE(tsi_ds_ma); i++)
			if (tsi_ds_ma[i] == arg)
				break;
		if (i == ARRAY_SIZE(tsi_ds_ma))
			return -EINVAL;
		mask = TSI_SKYLP_GPIO_CTRL_DS;
		val = FIELD_PREP(TSI_SKYLP_GPIO_CTRL_DS, i);
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		mask = TSI_SKYLP_GPIO_CTRL_IS;
		val = arg ? TSI_SKYLP_GPIO_CTRL_IS : 0;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		mask = TSI_SKYLP_GPIO_CTRL_OE | TSI_SKYLP_GPIO_CTRL_IE;
		val = arg ? TSI_SKYLP_GPIO_CTRL_OE : TSI_SKYLP_GPIO_CTRL_IE;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		mask = TSI_SKYLP_GPIO_CTRL_OE | TSI_SKYLP_GPIO_CTRL_IE;
		val = arg ? TSI_SKYLP_GPIO_CTRL_IE : TSI_SKYLP_GPIO_CTRL_OE;
		break;
	default:
		return -ENOTSUPP;
	}

	dev_dbg(tp->dev, "pinconf %s: param=%lu mask=0x%03x val=0x%03x\n",
		tp->soc->pins[pin].name, param, mask, val);

	return regmap_update_bits(tp->regmap, tsi_ctrl_reg(tp, pin),
				  mask, val);
}

/*
 * Apply a list of settings to one pad, stopping at the first failure.
 * Earlier settings are left in place: the control register is only
 * written a field at a time, so there is no all-or-nothing state to
 * roll back to.
 */
static int tsi_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			   unsigned long *configs, unsigned int nconfigs)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;
	int ret;

	for (i = 0; i < nconfigs; i++) {
		ret = tsi_pinconf_set_one(tp, pin, configs[i]);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Apply settings to every pad of a group. Unlike muxing, pin
 * configuration really is per-pad in hardware, so this is a loop over
 * individual control registers rather than one group-wide write.
 */
static int tsi_pinconf_group_set(struct pinctrl_dev *pctldev,
				 unsigned int selector,
				 unsigned long *configs,
				 unsigned int nconfigs)
{
	struct tsi_pinctrl *tp = pinctrl_dev_get_drvdata(pctldev);
	const struct tsi_group *grp = &tp->soc->groups[selector];
	unsigned int i;
	int ret;

	for (i = 0; i < grp->npins; i++) {
		ret = tsi_pinconf_set(pctldev, grp->pins[i], configs, nconfigs);
		if (ret)
			return ret;
	}
	return 0;
}

VISIBLE_IF_KUNIT const struct pinconf_ops tsi_pinconf_ops = {
	.is_generic		= true,
	.pin_config_get		= tsi_pinconf_get,
	.pin_config_set		= tsi_pinconf_set,
	.pin_config_group_set	= tsi_pinconf_group_set,
};
EXPORT_SYMBOL_IF_KUNIT(tsi_pinconf_ops);

/* ------------------------------- irqchip ----------------------------- */

/*
 * Pin events latch in one of the corner's interrupt collectors. Each
 * corner has several: on IONW t0 carries MDIO/ARM_SPI sources and t1
 * the DSPI ones, while t2 holds nothing but the 26 gpio_intr_* sources;
 * on IONE intr_regs_0..7 are i2c/smb/i3c/uart/thermal and intr_regs_8
 * is GPIO-only; on IOSW intr_regs_2 is GPIO-only. Only the GPIO-only
 * collector is described in the corner tables, so the peripheral
 * interrupts of a muxed-away pad are none of this driver's business.
 *
 * IOSE is the exception: its single collector puts mdio_intr at bit 0
 * and the two GPIO sources at bits 1..2, which is what irq_bit_shift
 * exists for. Getting that wrong would silently mask MDIO.
 *
 * A collector fans out to four maskable destination groups (g0..g3);
 * one of them is meant to reach the A520 GIC. Which group, the SPI
 * INTID and the sensitivity are still unconfirmed against the SoC
 * interrupt map, and the IO list documents GPIO alerts as controlled by
 * the M85, so the whole path stays dormant unless the DT node declares
 * "interrupts".
 */

/* Collector bit of a line. NOT always the pin - see irq_bit_shift. */
static u32 tsi_irq_bit(struct tsi_pinctrl *tp, unsigned int hwirq)
{
	return BIT(hwirq + tp->soc->irq_bit_shift);
}

/* Destination-enable register of the group this instance was given. */
static u32 tsi_irq_grp_en_reg(struct tsi_pinctrl *tp)
{
	return tp->base + tp->soc->irq_grp0_en_off +
	       tp->irq_grp * TSI_SKYLP_INTR_GRP_STRIDE;
}

/* Masked-pending register of the group this instance was given. */
static u32 tsi_irq_grp_ip_reg(struct tsi_pinctrl *tp)
{
	return tp->base + tp->soc->irq_grp0_ip_off +
	       tp->irq_grp * TSI_SKYLP_INTR_GRP_STRIDE;
}

/*
 * Clear the line's bit in our destination group's enable register.
 * Split from the irq_chip callback so it can be tested directly against
 * a fake regmap without a live irq_data/domain.
 */
VISIBLE_IF_KUNIT void tsi_pinctrl_irq_mask_hw(struct tsi_pinctrl *tp,
					      unsigned int hwirq)
{
	regmap_clear_bits(tp->regmap, tsi_irq_grp_en_reg(tp),
			  tsi_irq_bit(tp, hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_irq_mask_hw);

/*
 * Arm the line: set its bit in BOTH the global per-source enable
 * (int_enable_reg) and our destination group's enable.
 *
 * int_enable_reg is shared by all four destination groups, so this is
 * the one operation that reaches outside the group the A520 owns and
 * can affect what the M85 sees for the same source. That ownership
 * question is still open; it is not arbitrated here.
 */
VISIBLE_IF_KUNIT void tsi_pinctrl_irq_unmask_hw(struct tsi_pinctrl *tp,
						unsigned int hwirq)
{
	u32 mask = tsi_irq_bit(tp, hwirq);

	regmap_set_bits(tp->regmap, tp->base + tp->soc->irq_glben_off, mask);
	regmap_set_bits(tp->regmap, tsi_irq_grp_en_reg(tp), mask);
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_irq_unmask_hw);

/* Write-1-clear the latched pending bit. */
VISIBLE_IF_KUNIT void tsi_pinctrl_irq_ack_hw(struct tsi_pinctrl *tp,
					     unsigned int hwirq)
{
	regmap_write(tp->regmap, tp->base + tp->soc->irq_w1c_off,
		     tsi_irq_bit(tp, hwirq));
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_irq_ack_hw);

/*
 * irq_chip callbacks. Each pairs the register write with the gpiolib
 * bookkeeping that an IRQCHIP_IMMUTABLE chip owes the core:
 * gpiochip_disable_irq()/gpiochip_enable_irq() are what stop the line
 * from being handed out as a plain GPIO while it is an interrupt.
 * Ordering matters - release the line only after masking, and claim it
 * before unmasking, so no window exists where the line is armed but
 * not accounted for.
 */
static void tsi_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	unsigned int hwirq = irqd_to_hwirq(d);

	tsi_pinctrl_irq_mask_hw(tp, hwirq);
	gpiochip_disable_irq(chip, hwirq);
}

/* Also touches the collector's shared global enable - see _hw(). */
static void tsi_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	unsigned int hwirq = irqd_to_hwirq(d);

	gpiochip_enable_irq(chip, hwirq);
	tsi_pinctrl_irq_unmask_hw(tp, hwirq);
}

/* Level-triggered, so the core acks before the handler runs. */
static void tsi_irq_ack(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);

	tsi_pinctrl_irq_ack_hw(tp, irqd_to_hwirq(d));
}

/*
 * The collector is level-sensitive only and has no polarity register,
 * so active-high vs active-low cannot be programmed; handle_level_irq
 * is wired unconditionally at registration. Anything that is not
 * exactly a supported level type is rejected rather than silently
 * accepted, so a mixed edge+level request cannot look like success.
 */
VISIBLE_IF_KUNIT int tsi_pinctrl_irq_set_type(struct irq_data *d,
					      unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_irq_set_type);

static const struct irq_chip tsi_irq_chip = {
	.name		= "tsi-pinctrl",
	.irq_mask	= tsi_irq_mask,
	.irq_unmask	= tsi_irq_unmask,
	.irq_ack	= tsi_irq_ack,
	.irq_set_type	= tsi_pinctrl_irq_set_type,
	.flags		= IRQCHIP_IMMUTABLE | IRQCHIP_MASK_ON_SUSPEND,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

/*
 * Chained on the parent (GIC) IRQ: read the masked-pending register for
 * our destination group and dispatch every line that is set. Only the
 * corner's own GPIO sources are scanned, so a foreign bit sharing the
 * collector (IOSE's mdio_intr) is never dispatched as a GPIO.
 */
static void tsi_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	struct irq_chip *irqc = irq_desc_get_chip(desc);
	unsigned int i;
	u32 pending;
	int ret;

	chained_irq_enter(irqc, desc);

	ret = regmap_read(tp->regmap, tsi_irq_grp_ip_reg(tp), &pending);
	if (ret) {
		dev_dbg(tp->dev, "irq: ip_status_g%u read failed: %d\n",
			tp->irq_grp, ret);
		goto out;
	}

	dev_dbg(tp->dev, "irq: ip_status_g%u=0x%x\n", tp->irq_grp, pending);

	for (i = 0; i < tp->soc->npins; i++) {
		if (pending & tsi_irq_bit(tp, i))
			generic_handle_domain_irq(chip->irq.domain, i);
	}

out:
	chained_irq_exit(irqc, desc);
}

/* ------------------------------ gpio_chip ---------------------------- */

/*
 * There is no direction register on SkyLP: OE of the pad's control
 * register decides whether it drives. OE is active-high (ral.json IONW
 * reset 0x008 = OE set).
 */
static int tsi_gpio_get_direction(struct gpio_chip *chip, unsigned int off)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	u32 ctrl;
	int ret;

	ret = regmap_read(tp->regmap, tsi_ctrl_reg(tp, off), &ctrl);
	if (ret)
		return ret;

	return (ctrl & TSI_SKYLP_GPIO_CTRL_OE) ? GPIO_LINE_DIRECTION_OUT :
						 GPIO_LINE_DIRECTION_IN;
}

/*
 * Stop driving and enable the receiver. OE and IE are cleared and set
 * together so the pad never sits with both asserted, which would drive
 * and sample at the same time.
 */
static int tsi_gpio_direction_input(struct gpio_chip *chip, unsigned int off)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);

	return regmap_update_bits(tp->regmap, tsi_ctrl_reg(tp, off),
				  TSI_SKYLP_GPIO_CTRL_OE |
				  TSI_SKYLP_GPIO_CTRL_IE,
				  TSI_SKYLP_GPIO_CTRL_IE);
}

/*
 * data_out is one register for the whole corner and has no set/clear
 * companion, so this is an RMW. The chiplet regmap lock makes it atomic
 * against other Linux threads; it cannot protect against the M85
 * writing the same register.
 */
static int tsi_gpio_set_line(struct tsi_pinctrl *tp, unsigned int off,
			     int value)
{
	u32 mask = BIT(off);

	return regmap_update_bits(tp->regmap, tp->base + tp->soc->data_out_off,
				  mask, value ? mask : 0);
}

/*
 * Latch the level before raising OE so the pad never briefly drives the
 * stale contents of data_out.
 */
static int tsi_gpio_direction_output(struct gpio_chip *chip, unsigned int off,
				     int value)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	int ret;

	ret = tsi_gpio_set_line(tp, off, value);
	if (ret)
		return ret;

	return regmap_update_bits(tp->regmap, tsi_ctrl_reg(tp, off),
				  TSI_SKYLP_GPIO_CTRL_OE |
				  TSI_SKYLP_GPIO_CTRL_IE,
				  TSI_SKYLP_GPIO_CTRL_OE);
}

/*
 * Sample the pad. data_in is the receiver output, so this reads the
 * real pin level rather than what was last driven - valid for inputs
 * and, when IE is set, as readback on an output.
 */
static int tsi_gpio_get(struct gpio_chip *chip, unsigned int off)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	u32 val;
	int ret;

	ret = regmap_read(tp->regmap, tp->base + tp->soc->data_in_off, &val);
	if (ret)
		return ret;

	return !!(val & BIT(off));
}

/*
 * Sample several pads in one read. Because a pin number *is* its
 * data-register bit, the hardware word can be masked and returned
 * directly with no per-line remapping.
 */
static int tsi_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask,
				 unsigned long *bits)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	u32 val;
	int ret;

	ret = regmap_read(tp->regmap, tp->base + tp->soc->data_in_off, &val);
	if (ret)
		return ret;

	/* pin == data bit, so the hardware word needs no remapping. */
	*bits = val & *mask;
	return 0;
}

/* 6.12 gpiolib expects void here; the error is reported through dev_dbg. */
static void tsi_gpio_set(struct gpio_chip *chip, unsigned int off, int value)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	int ret;

	ret = tsi_gpio_set_line(tp, off, value);
	if (ret)
		dev_dbg(tp->dev, "set(%s): failed: %d\n",
			tp->soc->pins[off].name, ret);
}

/*
 * Drive several pads in one read-modify-write, which is both cheaper
 * and less exposed than a loop: the shared data_out register is only
 * touched once. Void-returning like .set on this kernel, so a failure
 * can only be reported through the log.
 */
static void tsi_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				  unsigned long *bits)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);
	int ret;

	ret = regmap_update_bits(tp->regmap,
				 tp->base + tp->soc->data_out_off,
				 (u32)*mask, (u32)(*bits & *mask));
	if (ret)
		dev_dbg(tp->dev, "set_multiple: failed: %d\n", ret);
}

/*
 * Tell the pinctrl core which pins this gpio_chip's lines correspond
 * to. Without a range, gpiochip_generic_request() has nothing to
 * translate and muxing on gpiod_get() would never happen. Called from
 * gpiochip_add(), which is why the pinctrl device is registered first.
 */
static int tsi_gpio_add_pin_ranges(struct gpio_chip *chip)
{
	struct tsi_pinctrl *tp = gpiochip_get_data(chip);

	/* pin == gpio offset, so one 1:1 range covers the corner. */
	return gpiochip_add_pin_range(chip, dev_name(tp->dev), 0, 0,
				      tp->soc->npins);
}

/* ------------------------------- probe ------------------------------- */

/*
 * Build the function list from the groups' mux options: a function is
 * every distinct name any group offers, and it maps back to each group
 * offering it. Deriving it avoids a second table that could disagree
 * with the first.
 */
static int tsi_build_funcs(struct tsi_pinctrl *tp)
{
	const struct tsi_pinctrl_soc *soc = tp->soc;
	unsigned int g, o, f, nmax = 0;

	for (g = 0; g < soc->ngroups; g++)
		nmax += max(soc->groups[g].nopts, 1U);

	tp->funcs = devm_kcalloc(tp->dev, nmax, sizeof(*tp->funcs),
				 GFP_KERNEL);
	if (!tp->funcs)
		return -ENOMEM;

	for (g = 0; g < soc->ngroups; g++) {
		const struct tsi_group *grp = &soc->groups[g];
		unsigned int nopts = grp->nopts;

		for (o = 0; o < max(nopts, 1U); o++) {
			/* A dedicated group offers GPIO only. */
			const char *name = nopts ? grp->opts[o].func :
						   TSI_FUNC_GPIO;

			for (f = 0; f < tp->nfuncs; f++)
				if (!strcmp(tp->funcs[f].name, name))
					break;

			if (f == tp->nfuncs) {
				tp->funcs[f].name = name;
				tp->funcs[f].groups =
					devm_kcalloc(tp->dev, soc->ngroups,
						     sizeof(char *),
						     GFP_KERNEL);
				if (!tp->funcs[f].groups)
					return -ENOMEM;
				tp->nfuncs++;
			}

			tp->funcs[f].groups[tp->funcs[f].ngroups++] =
				grp->name;
		}
	}

	return 0;
}

/*
 * Build and register one corner: a pinctrl device and a gpio_chip over
 * the same register block.
 *
 * Split from probe() so the KUnit suite can stand an instance up over a
 * fake regmap without a platform device or a chiplet parent. @irq of 0
 * means no parent interrupt, which leaves the irqchip unregistered;
 * @irq_grp is only meaningful alongside a real @irq.
 *
 * Everything allocated here is devm-managed and torn down with @dev.
 */
VISIBLE_IF_KUNIT struct tsi_pinctrl *tsi_pinctrl_register(struct device *dev,
					 struct regmap *regmap, u32 base,
					 const struct tsi_pinctrl_soc *soc,
					 int irq, u32 irq_grp)
{
	struct pinctrl_desc *desc;
	struct tsi_pinctrl *tp;
	const char **names;
	unsigned int i;
	int ret;

	tp = devm_kzalloc(dev, sizeof(*tp), GFP_KERNEL);
	if (!tp)
		return ERR_PTR(-ENOMEM);

	tp->dev = dev;
	tp->regmap = regmap;
	tp->base = base;
	tp->soc = soc;
	tp->irq_grp = irq_grp;

	tp->pin_desc = devm_kcalloc(dev, soc->npins, sizeof(*tp->pin_desc),
				    GFP_KERNEL);
	if (!tp->pin_desc)
		return ERR_PTR(-ENOMEM);

	/*
	 * gpiolib takes line names from gpio_chip.names, not from the
	 * pinctrl pin descriptors, so the pad names have to be handed over
	 * separately or every line shows up unnamed in gpioinfo. A "reg"
	 * array is needed rather than the pin_desc names because the field
	 * is a const char *const *.
	 *
	 * gpiochip_add_data() applies these first and then lets a DT
	 * "gpio-line-names" property override them, so a board can still
	 * label lines by their function instead of by pad name.
	 */
	names = devm_kcalloc(dev, soc->npins, sizeof(*names), GFP_KERNEL);
	if (!names)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < soc->npins; i++) {
		tp->pin_desc[i].number = i;
		tp->pin_desc[i].name = soc->pins[i].name;
		names[i] = soc->pins[i].name;
	}

	ret = tsi_build_funcs(tp);
	if (ret)
		return ERR_PTR(ret);

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	desc->name = dev_name(dev);
	desc->pins = tp->pin_desc;
	desc->npins = soc->npins;
	desc->pctlops = &tsi_pinctrl_ops;
	desc->pmxops = &tsi_pinmux_ops;
	desc->confops = &tsi_pinconf_ops;
	desc->owner = THIS_MODULE;

	/*
	 * pinctrl first: gpiochip_add() runs .add_pin_ranges, which needs
	 * this pinctrl device to already be findable by name.
	 */
	tp->pctl = devm_pinctrl_register(dev, desc, tp);
	if (IS_ERR(tp->pctl))
		return ERR_CAST(tp->pctl);

	tp->chip.label = dev_name(dev);
	tp->chip.parent = dev;
	tp->chip.owner = THIS_MODULE;
	tp->chip.base = -1;
	tp->chip.ngpio = soc->npins;
	tp->chip.names = names;
	tp->chip.can_sleep = regmap_might_sleep(regmap);
	tp->chip.request = gpiochip_generic_request;
	tp->chip.free = gpiochip_generic_free;
	tp->chip.set_config = gpiochip_generic_config;
	tp->chip.add_pin_ranges = tsi_gpio_add_pin_ranges;
	tp->chip.get_direction = tsi_gpio_get_direction;
	tp->chip.direction_input = tsi_gpio_direction_input;
	tp->chip.direction_output = tsi_gpio_direction_output;
	tp->chip.get = tsi_gpio_get;
	tp->chip.get_multiple = tsi_gpio_get_multiple;
	tp->chip.set = tsi_gpio_set;
	tp->chip.set_multiple = tsi_gpio_set_multiple;

	/*
	 * Dormant unless the DT node gave us a parent interrupt: see the
	 * irqchip section for the unresolved collector-to-GIC wiring and
	 * M85 ownership questions.
	 */
	if (irq > 0) {
		struct gpio_irq_chip *girq = &tp->chip.irq;

		dev_dbg(dev, "irqchip: chaining on parent irq %d, group %u\n",
			irq, irq_grp);
		gpio_irq_chip_set_chip(girq, &tsi_irq_chip);
		girq->parent_handler = tsi_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents)
			return ERR_PTR(-ENOMEM);
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
	}

	ret = devm_gpiochip_add_data(dev, &tp->chip, tp);
	if (ret)
		return ERR_PTR(ret);

	dev_dbg(dev, "%s: %u pads, %u groups, %u functions\n",
		soc->label, soc->npins, soc->ngroups, tp->nfuncs);

	return tp;
}
EXPORT_SYMBOL_IF_KUNIT(tsi_pinctrl_register);

/*
 * Resolve everything that comes from the platform - which corner this
 * is, the chiplet regmap, the corner's offset in the CSR window, and
 * the optional interrupt - then hand off to tsi_pinctrl_register().
 * Register access belongs to the chiplet core, so this driver never
 * ioremaps anything itself.
 */
static int tsi_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct tsi_pinctrl_soc *soc;
	struct tsi_chiplet *chiplet;
	struct tsi_pinctrl *tp;
	u32 base, size, span, irq_grp = 0;
	int irq, ret;

	soc = device_get_match_data(dev);
	if (!soc)
		return -ENODEV;

	chiplet = tsi_chiplet_get(dev);
	if (IS_ERR(chiplet))
		return dev_err_probe(dev, PTR_ERR(chiplet),
				     "no tsi-chiplet parent\n");

	/*
	 * Corner window within the chiplet CSR window: "reg" is
	 * <offset size>, both relative to the parent's "ranges". Read it
	 * as two explicit cells rather than one u32 so that a parent using
	 * #address-cells/#size-cells other than 1 is rejected instead of
	 * silently contributing its high word as the offset.
	 */
	ret = of_property_count_u32_elems(dev->of_node, "reg");
	if (ret != 2)
		return dev_err_probe(dev, -EINVAL,
				     "reg must be <offset size>; got %d cells (parent #address-cells and #size-cells must both be 1)\n",
				     ret);

	ret = of_property_read_u32_index(dev->of_node, "reg", 0, &base);
	if (ret)
		return dev_err_probe(dev, ret, "missing reg offset\n");

	ret = of_property_read_u32_index(dev->of_node, "reg", 1, &size);
	if (ret)
		return dev_err_probe(dev, ret, "missing reg size\n");

	if (base & 3)
		return dev_err_probe(dev, -EINVAL,
				     "reg offset 0x%x is not 4-byte aligned\n",
				     base);

	/*
	 * Refuse a window that does not cover the registers this corner's
	 * tables address. The parent regmap would let the writes through,
	 * so an undersized node would otherwise fail as corrupted
	 * neighbouring state rather than as a probe error.
	 */
	span = tsi_pinctrl_reg_span(soc);
	if (size < span)
		return dev_err_probe(dev, -EINVAL,
				     "reg size 0x%x too small for %s; needs at least 0x%x\n",
				     size, soc->label, span);

	/* Which collector destination group reaches the GIC; 0 is safe. */
	device_property_read_u32(dev, "tsi,irq-dest-group", &irq_grp);
	if (irq_grp > 3)
		return dev_err_probe(dev, -EINVAL,
				     "tsi,irq-dest-group must be 0..3\n");

	/*
	 * The IRQ is optional, but only a genuinely absent "interrupts"
	 * property (-ENXIO) may fall back to polling. Any other error, in
	 * particular -EPROBE_DEFER while the parent interrupt controller
	 * is still probing, has to propagate: swallowing it would leave
	 * the irqchip permanently unwired.
	 */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq == -ENXIO)
		irq = 0;
	else if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get optional IRQ\n");

	tp = tsi_pinctrl_register(dev, chiplet->regmap, base, soc, irq,
				  irq_grp);
	if (IS_ERR(tp))
		return dev_err_probe(dev, PTR_ERR(tp),
				     "failed to register %s\n", soc->label);

	return 0;
}

static const struct of_device_id tsi_pinctrl_of_match[] = {
	{ .compatible = "tsi,skylp-ionw-pinctrl", .data = &tsi_skylp_ionw_soc },
	{ .compatible = "tsi,skylp-ione-pinctrl", .data = &tsi_skylp_ione_soc },
	{ .compatible = "tsi,skylp-iose-pinctrl", .data = &tsi_skylp_iose_soc },
	{ .compatible = "tsi,skylp-iosw-pinctrl", .data = &tsi_skylp_iosw_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, tsi_pinctrl_of_match);

static struct platform_driver tsi_pinctrl_driver = {
	.probe = tsi_pinctrl_probe,
	.driver = {
		.name = "pinctrl-tsi",
		.of_match_table = tsi_pinctrl_of_match,
	},
};
module_platform_driver(tsi_pinctrl_driver);

MODULE_DESCRIPTION("TSI chiplet IO-corner pinctrl and GPIO driver");
MODULE_AUTHOR("Tsavorite Scalable Intelligence");
MODULE_LICENSE("GPL");
