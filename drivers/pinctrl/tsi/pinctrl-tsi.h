/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI SkyLP pinctrl/GPIO driver - shared definitions for the driver and
 * its KUnit test (pinctrl-tsi-test.c).
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#ifndef __PINCTRL_TSI_H
#define __PINCTRL_TSI_H

#include <linux/gpio/driver.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/types.h>

struct device;
struct regmap;

/*
 * One GPIO-capable pad. The pin number is the pad's bit in the corner's
 * shared gpio_data_out/gpio_data_in registers, which is why no explicit
 * bit member is needed: pin == data bit == gpiolib offset. The bits are
 * contiguous from 0 on all four corners (verified against ral.json
 * SKYLP_G0829), but they are NOT in pad-address order - IONW GPIO_4..7
 * sit at bits 21..24 while its SPI pads occupy 9..20.
 */
struct tsi_pin {
	const char	*name;
	u16		ctrl_off;	/* corner-relative <pad>_control */
};

/* One selectable function on a group, and the iomode value choosing it. */
struct tsi_mux_opt {
	const char	*func;
	u8		val;
};

/*
 * A set of pads that share one iomode (function-select) register.
 *
 * Muxing on SkyLP is per-group, not per-pad: one iomode field switches
 * every pad in the group at once. Groups with iomode_off == 0 are
 * dedicated GPIO pads with no alternate function and no mux register.
 */
struct tsi_group {
	const char			*name;
	const unsigned int		*pins;
	unsigned int			npins;
	u16				iomode_off;
	u8				lsb;
	u8				width;
	const struct tsi_mux_opt	*opts;
	unsigned int			nopts;
};

/* Per-corner description (IONW/IONE/IOSE/IOSW). */
struct tsi_pinctrl_soc {
	const char			*label;
	const struct tsi_pin		*pins;
	unsigned int			npins;
	const struct tsi_group		*groups;
	unsigned int			ngroups;
	u16				data_out_off;
	u16				data_in_off;

	/*
	 * GPIO interrupt collector. Each corner has several collectors;
	 * only one of them carries the gpio_intr_* sources (the others
	 * belong to the corner's peripheral IPs - see the driver's irq
	 * section). ip_status/enable repeat per destination group with
	 * TSI_SKYLP_INTR_GRP_STRIDE between groups.
	 */
	u16				irq_w1c_off;	   /* ack, write-1-clear */
	u16				irq_glben_off;	   /* global per-source enable */
	u16				irq_grp0_ip_off;   /* masked pending, group 0 */
	u16				irq_grp0_en_off;   /* destination mask, group 0 */
	/*
	 * Collector bit of a line is (pin + irq_bit_shift). Zero on every
	 * corner whose collector holds nothing but GPIO, but IOSE shares
	 * its collector with MDIO at bit 0, so its GPIO sources start at
	 * bit 1.
	 */
	u8				irq_bit_shift;
};

/* Function name -> the groups that can be muxed to it (built at probe). */
struct tsi_func {
	const char	*name;
	const char	**groups;
	unsigned int	ngroups;
};

/* One probed corner: its pinctrl device and gpio_chip share a regmap. */
struct tsi_pinctrl {
	struct device			*dev;
	struct pinctrl_dev		*pctl;
	struct gpio_chip		chip;
	struct regmap			*regmap;	/* owned by the chiplet core */
	u32				base;		/* corner base in CSR window */
	const struct tsi_pinctrl_soc	*soc;
	struct pinctrl_pin_desc		*pin_desc;	/* npins entries */
	struct tsi_func			*funcs;		/* derived at probe */
	unsigned int			nfuncs;
	u32				irq_grp;	/* destination group 0..3 */
};

/*
 * Interfaces exposed to the KUnit suite only. The register-level
 * helpers below are deliberately split out of their pinctrl/irq_chip
 * callbacks so a test can drive them against a fake regmap without a
 * live pinctrl_dev, irq_data or irq domain.
 */
#if IS_ENABLED(CONFIG_KUNIT)
/* Stand up a corner over an arbitrary regmap; irq 0 leaves it dormant. */
struct tsi_pinctrl *tsi_pinctrl_register(struct device *dev,
					 struct regmap *regmap, u32 base,
					 const struct tsi_pinctrl_soc *soc,
					 int irq, u32 irq_grp);

/*
 * The registered ops tables. Tests drive the real callbacks through
 * these rather than reimplementing the lookups: a pinctrl_dev is opaque
 * outside the pinctrl core, but every callback only resolves it with
 * the public pinctrl_dev_get_drvdata(), so passing tp->pctl straight
 * back in works.
 */
extern const struct pinctrl_ops tsi_pinctrl_ops;
extern const struct pinmux_ops tsi_pinmux_ops;
extern const struct pinconf_ops tsi_pinconf_ops;

/* Corner descriptions, so tests can assert the generated tables. */
extern const struct tsi_pinctrl_soc tsi_skylp_ionw_soc;
extern const struct tsi_pinctrl_soc tsi_skylp_ione_soc;
extern const struct tsi_pinctrl_soc tsi_skylp_iose_soc;
extern const struct tsi_pinctrl_soc tsi_skylp_iosw_soc;

/* Write a group's iomode field; affects every pad in the group. */
int tsi_pinctrl_mux_set(struct tsi_pinctrl *tp, const struct tsi_group *grp,
			u8 val);
/* Decode a field value to its function name, or NULL if undefined. */
const char *tsi_pinctrl_func_of_val(const struct tsi_group *grp, u8 val);
/* Smallest corner-relative window that covers this corner's tables. */
u32 tsi_pinctrl_reg_span(const struct tsi_pinctrl_soc *soc);
/* The group a pad belongs to, or NULL if the pin is out of range. */
const struct tsi_group *tsi_pinctrl_group_of_pin(struct tsi_pinctrl *tp,
						 unsigned int pin);
/* Move the group owning @pin to its GPIO function, if it has one. */
int tsi_pinctrl_gpio_enable(struct tsi_pinctrl *tp, unsigned int pin);

/* Collector writes. @hwirq is a pin; the bit may be shifted (IOSE). */
void tsi_pinctrl_irq_mask_hw(struct tsi_pinctrl *tp, unsigned int hwirq);
void tsi_pinctrl_irq_unmask_hw(struct tsi_pinctrl *tp, unsigned int hwirq);
void tsi_pinctrl_irq_ack_hw(struct tsi_pinctrl *tp, unsigned int hwirq);
/* d is unused; safe to call with NULL from tests. */
int tsi_pinctrl_irq_set_type(struct irq_data *d, unsigned int type);
#endif

#endif /* __PINCTRL_TSI_H */
