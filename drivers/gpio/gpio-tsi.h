/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI GPIO controller driver - shared definitions for the driver and
 * its KUnit test (gpio-tsi-test.c).
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#ifndef __GPIO_TSI_H
#define __GPIO_TSI_H

#include <linux/gpio/driver.h>
#include <linux/types.h>

struct device;
struct regmap;

/* One GPIO-capable pad: its control register and shared-data bit. */
struct tsi_gpio_line {
	const char	*name;
	u16		ctrl_off;	/* corner-relative control reg */
	u8		bit;		/* bit in gpio_data_out/in */
};

struct tsi_gpio_soc {
	const struct tsi_gpio_line	*lines;
	unsigned int			nlines;
	u16				data_out_off;
	u16				data_in_off;
	/*
	 * Interrupt collector (t2 on IONW, intr_regs_8 on IONE). Only
	 * consumed when the platform device has an "interrupts" property;
	 * see the irqchip block in gpio-tsi.c for the open items (A/B/C/E)
	 * this depends on.
	 */
	u16				irq_w1c_off;	/* ack */
	u16				irq_glben_off;	/* global per-source enable */
	u16				irq_grp0_ip_off;  /* masked pending, group 0 */
	u16				irq_grp0_en_off;  /* destination mask, group 0 */
};

struct tsi_gpio {
	struct gpio_chip		chip;
	struct regmap			*regmap;
	u32				base;	/* corner base in CSR window */
	const struct tsi_gpio_soc	*soc;
	u32				irq_grp;  /* destination group 0..3 */
};

#if IS_ENABLED(CONFIG_KUNIT)
struct tsi_gpio *tsi_gpio_register(struct device *dev, struct regmap *regmap,
				   u32 base, const struct tsi_gpio_soc *soc,
				   int irq, u32 irq_grp);
extern const struct tsi_gpio_soc tsi_gpio_ionw_soc;
extern const struct tsi_gpio_soc tsi_gpio_ione_soc;

void tsi_gpio_irq_mask_hw(struct tsi_gpio *gpio, unsigned int hwirq);
void tsi_gpio_irq_unmask_hw(struct tsi_gpio *gpio, unsigned int hwirq);
void tsi_gpio_irq_ack_hw(struct tsi_gpio *gpio, unsigned int hwirq);
/* d is unused; safe to call with NULL from tests. */
int tsi_gpio_irq_set_type(struct irq_data *d, unsigned int type);
#endif

#endif /* __GPIO_TSI_H */
