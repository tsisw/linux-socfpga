/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI SkyLP interrupt collector ("tsi,skylp-intc") — shared declarations.
 *
 * One collector fans several IP interrupt sources into four destination
 * groups (g0..g3); each group can be routed to a different master (A520
 * GIC, M85, ...). This driver owns exactly one destination group of one
 * collector instance and exposes the sources as a linear irq_domain.
 *
 * Register layout, relative to the collector's first register (all
 * collectors on SkyLP share it — GPP/UDI, GPP/GPU, IONE, IOSW differ
 * only in base address and source count):
 *
 *   0x00  interrupt_reg      RW1C  latched pending, all sources
 *   0x04  interrupt_reg_w1c  W1C   ack alias
 *   0x08  interrupt_reg_w1s  W1S   set alias (test aid)
 *   0x0c  int_enable         RW    global per-source enable (shared by
 *                                  all four groups!)
 *   0x10 + g * 0x0c:
 *     +0  unmasked_status_g  RO
 *     +4  ip_status_g        RO    pending AND enabled for group g
 *     +8  enable_g           RW    routing enable for group g
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#ifndef __IRQ_TSI_SKYLP_H
#define __IRQ_TSI_SKYLP_H

#include <linux/regmap.h>
#include <linux/types.h>
/* decls below are VISIBLE_IF_KUNIT, defined in irq-tsi-skylp.c */

struct irq_domain;

#define TSI_INTC_PENDING	0x00
#define TSI_INTC_W1C		0x04
#define TSI_INTC_W1S		0x08
#define TSI_INTC_GLB_EN		0x0c
#define TSI_INTC_GRP_BASE	0x10
#define TSI_INTC_GRP_STRIDE	0x0c
#define TSI_INTC_GRP_UNMASKED	0x0
#define TSI_INTC_GRP_IP		0x4
#define TSI_INTC_GRP_EN		0x8

#define TSI_INTC_MAX_SOURCES	32
#define TSI_INTC_NR_GROUPS	4

struct tsi_intc {
	struct device		*dev;		/* for diagnostics only */
	struct regmap		*regmap;
	u32			base;		/* collector origin in regmap */
	u32			dest_grp;	/* g0..g3 this instance owns */
	u32			nr_sources;
	struct irq_domain	*domain;
};

/* VISIBLE_IF_KUNIT is keyed on CONFIG_KUNIT, so the prototypes must be too. */
#if IS_ENABLED(CONFIG_KUNIT)
void tsi_intc_irq_mask_hw(struct tsi_intc *ti, unsigned int hwirq);
void tsi_intc_irq_unmask_hw(struct tsi_intc *ti, unsigned int hwirq);
void tsi_intc_irq_ack_hw(struct tsi_intc *ti, unsigned int hwirq);
int tsi_intc_irq_type_valid(unsigned int type);
/*
 * Read tsi,dest-group (default 0) and tsi,num-sources (default all 32) from
 * any firmware node - DT, ACPI _DSD or software node - and range-check them.
 */
int tsi_intc_parse(const struct fwnode_handle *fw, u32 *dest_grp,
		   u32 *nr_sources);
/*
 * Drain this group's pending sources: invoke fire(hwirq, cookie) for each
 * set bit in ip_status_g, low to high. The drain does not ack. The latch
 * is sticky and level-transparent (HW-2), so it may only be cleared once
 * the child has cleared its own source; that W1C is the chip's irq_eoi,
 * issued by the fasteoi flow after the child handler (or its thread) ran.
 * Returns the number of sources fired.
 */
int tsi_intc_drain(struct tsi_intc *ti,
		   void (*fire)(unsigned int hwirq, void *cookie),
		   void *cookie);
extern const struct irq_chip tsi_intc_chip;
extern const irq_flow_handler_t tsi_intc_level_flow;
#endif

#endif /* __IRQ_TSI_SKYLP_H */
