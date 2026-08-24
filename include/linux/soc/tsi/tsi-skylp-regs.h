/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SkyLP chiplet register map - GPIO subset.
 *
 * GENERATED from ral.json, release SKYLP_G0829, on 2026-08-09. Do not hand-edit;
 * regenerate with the RAL header generator (tsi-drivers staging,
 * work item 1.3). Offsets under TSI_SKYLP_*_BASE are relative to
 * the chiplet CSR window, which maps SkyLP CSR space starting at
 * 0x2000_0000 (window offset = RAL address - 0x2000_0000).
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#ifndef __LINUX_SOC_TSI_SKYLP_REGS_H
#define __LINUX_SOC_TSI_SKYLP_REGS_H

#include <linux/bits.h>

/* <pad>_control, field control[8:0] (same layout on every pad) */
#define TSI_SKYLP_GPIO_CTRL_DS		GENMASK(2, 0)
#define TSI_SKYLP_GPIO_CTRL_OE		BIT(3)
#define TSI_SKYLP_GPIO_CTRL_IE		BIT(4)
#define TSI_SKYLP_GPIO_CTRL_IS		BIT(5)
#define TSI_SKYLP_GPIO_CTRL_PE		BIT(6)
#define TSI_SKYLP_GPIO_CTRL_POE		BIT(7)
#define TSI_SKYLP_GPIO_CTRL_PS		BIT(8)

/* IONW corner (CF802), ionw_mgtclk_regs; offsets corner-relative */
#define TSI_SKYLP_IONW_BASE		0x13000000
#define TSI_SKYLP_IONW_GPIO0_CTRL	0x11c
#define TSI_SKYLP_IONW_GPIO1_CTRL	0x120
#define TSI_SKYLP_IONW_GPIO2_CTRL	0x124
#define TSI_SKYLP_IONW_GPIO3_CTRL	0x128
#define TSI_SKYLP_IONW_GPIO4_CTRL	0x12c
#define TSI_SKYLP_IONW_GPIO5_CTRL	0x130
#define TSI_SKYLP_IONW_GPIO6_CTRL	0x134
#define TSI_SKYLP_IONW_GPIO7_CTRL	0x138
#define TSI_SKYLP_IONW_GPIO0_BIT		0x0
#define TSI_SKYLP_IONW_GPIO1_BIT		0x1
#define TSI_SKYLP_IONW_GPIO2_BIT		0x2
#define TSI_SKYLP_IONW_GPIO3_BIT		0x3
#define TSI_SKYLP_IONW_GPIO4_BIT		0x15
#define TSI_SKYLP_IONW_GPIO5_BIT		0x16
#define TSI_SKYLP_IONW_GPIO6_BIT		0x17
#define TSI_SKYLP_IONW_GPIO7_BIT		0x18
#define TSI_SKYLP_IONW_GPIO_DATA_OUT	0x178
#define TSI_SKYLP_IONW_GPIO_DATA_IN	0x17c

/* IONE corner (CF803), ione_mgtclk_regs; offsets corner-relative */
#define TSI_SKYLP_IONE_BASE		0x6000000
#define TSI_SKYLP_IONE_GPIO_FS0_CTRL	0x124
#define TSI_SKYLP_IONE_GPIO_FS1_CTRL	0x128
#define TSI_SKYLP_IONE_GPIO_FS2_CTRL	0x12c
#define TSI_SKYLP_IONE_GPIO_FS3_CTRL	0x130
#define TSI_SKYLP_IONE_GPIO_FS0_BIT	0xa
#define TSI_SKYLP_IONE_GPIO_FS1_BIT	0xb
#define TSI_SKYLP_IONE_GPIO_FS2_BIT	0xc
#define TSI_SKYLP_IONE_GPIO_FS3_BIT	0xd
#define TSI_SKYLP_IONE_GPIO_DATA_OUT	0x188
#define TSI_SKYLP_IONE_GPIO_DATA_IN	0x18c

/*
 * Interrupt collectors (per-source R/W1C latched). pinctrl-tsi
 * carries its own per-corner collector offsets and arms them only
 * when a DT node declares "interrupts": the IO list documents GPIO
 * alerts/interrupts as owned by the M85, and the group-to-GIC
 * wiring, SPI INTID, and edge/level sensitivity are not in the CSR
 * spec (open items A/B/C).
 */
#define TSI_SKYLP_IONW_T2_INTR		0x34c0
#define TSI_SKYLP_IONW_T2_INTR_W1C	0x34c4
#define TSI_SKYLP_IONW_T2_INTR_W1S	0x34c8
#define TSI_SKYLP_IONW_T2_INTR_EN	0x34cc
#define TSI_SKYLP_IONW_T2_G0_STATUS	0x34d0
#define TSI_SKYLP_IONW_T2_G0_IP		0x34d4
#define TSI_SKYLP_IONW_T2_G0_EN		0x34d8
#define TSI_SKYLP_IONE_INTR8_INTR		0xc340
#define TSI_SKYLP_IONE_INTR8_INTR_W1C	0xc344
#define TSI_SKYLP_IONE_INTR8_INTR_W1S	0xc348
#define TSI_SKYLP_IONE_INTR8_INTR_EN	0xc34c
#define TSI_SKYLP_IONE_INTR8_G0_STATUS	0xc350
#define TSI_SKYLP_IONE_INTR8_G0_IP		0xc354
#define TSI_SKYLP_IONE_INTR8_G0_EN		0xc358
#define TSI_SKYLP_INTR_GRP_STRIDE	0xc

#endif /* __LINUX_SOC_TSI_SKYLP_REGS_H */
