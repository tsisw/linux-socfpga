/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Register definitions for the Synopsys DWC_i2c "Advanced I2C/SMBus
 * Controller and Target Device" as instantiated on TSI SkyLP.
 *
 * This is NOT the DW_apb_i2c that drivers/i2c/busses/i2c-designware-* drives.
 * DWC_i2c v1.01a is a later, separate IP: the register file is split into a
 * programmable-base two-block layout, and IC_CTRL's fields sit at different
 * bit positions from DW_apb_i2c's IC_CON. A few of the deltas, for orientation
 * when reading this next to i2c-designware-core.h:
 *
 *   register        DW_apb_i2c   DWC_i2c v1.01a
 *   IC_CON/IC_CTRL  0x00         0x24   SPEED b2:1 -> b5:4
 *                                       TX_EMPTY_CTRL b8 -> b11
 *                                       MASTER_MODE+SLAVE_DISABLE -> IC_OP_MODE b0
 *   IC_TAR          0x04         0x28
 *   IC_DATA_CMD     0x10         0x78
 *   IC_ENABLE       0x6c         0x04   (moved into the operational block)
 *   IC_STATUS       0x70         0xac
 *   IC_TXFLR/RXFLR  0x74/0x78    0xb0/0xb4
 *   IC_COMP_TYPE    0xfc         0xc8   (same value, 0x44570140 -- so the
 *                                        magic alone does NOT tell the two
 *                                        generations apart; IC_HCI_VERSION does)
 *
 * Also note DWC_i2c has ONE IC_SCL_HCNT/IC_SCL_LCNT pair selected by
 * IC_CTRL.SPEED, where DW_apb_i2c has SS/FS/HS triplets.
 *
 * Offsets and reset values are from the RTL-generated RAL
 * (skylp/flex_register_macros.h, skylp/flex_ral_reset_defaults_ione.c) and
 * cross-checked against DWC_i2c_databook.pdf v1.01a-lca00 and the SCU
 * testbench C tests (i2c0_fast_mode_bringup.c).
 */

#ifndef __I2C_DW_ADVANCED_REGS_H__
#define __I2C_DW_ADVANCED_REGS_H__

/* Operational block -- fixed at offset 0 in the register file. */
#define DWA_IC_HCI_VERSION		0x00
#define DWA_IC_ENABLE			0x04
#define DWA_IC_RESET_CTRL		0x08
#define DWA_IC_CAPABILITIES		0x0c
#define DWA_IC_I2C_CAPABILITIES		0x10
#define DWA_IC_I2C_BLOCK_OFFSET		0x14

/*
 * I2C block -- relative to the value programmed into IC_I2C_BLOCK_OFFSET.
 * That register is writable and reads 0x20 out of reset; RAL's published
 * absolute offsets assume 0x20, and the driver programs it explicitly.
 */
#define DWA_I2C_BLOCK_DEFAULT_OFFSET	0x20

#define DWA_IC_I2C_CORE_HEADER		0x00
#define DWA_IC_CTRL			0x04
#define DWA_IC_TAR			0x08
#define DWA_IC_DAR			0x0c
#define DWA_IC_HS_CADDR			0x10
#define DWA_IC_UFM_TBUF_CNT		0x20
#define DWA_IC_SCL_HCNT			0x24
#define DWA_IC_SCL_LCNT			0x28
#define DWA_IC_HS_SCL_HCNT		0x2c
#define DWA_IC_HS_SCL_LCNT		0x30
#define DWA_IC_SDA_HOLD			0x34
#define DWA_IC_SPKLEN			0x3c
#define DWA_IC_HS_SPKLEN		0x40
#define DWA_IC_REG_TIMEOUT_RST		0x50
#define DWA_IC_DATA_CMD			0x58
#define DWA_IC_RX_TL			0x5c
#define DWA_IC_TX_TL			0x60
#define DWA_IC_INTR_STAT		0x74
#define DWA_IC_INTR_MASK		0x78
#define DWA_IC_INTR_RAW_STAT		0x7c
#define DWA_IC_INTR_CLR			0x80
#define DWA_IC_ENABLE_STATUS		0x84
#define DWA_IC_TX_TRMNT_SOURCE		0x88
#define DWA_IC_STATUS			0x8c
#define DWA_IC_TXFLR			0x90
#define DWA_IC_RXFLR			0x94
#define DWA_IC_COMP_VERSION		0xa4
#define DWA_IC_COMP_TYPE		0xa8

/* IC_CTRL fields (databook Table 7-14). */
#define DWA_IC_CTRL_OP_MODE		BIT(0)	/* 1 = controller, 0 = target */
#define DWA_IC_CTRL_SPEED_SHIFT		4
#define DWA_IC_CTRL_SPEED_MASK		GENMASK(5, 4)
#define DWA_IC_CTRL_SPEED_STANDARD	(1 << DWA_IC_CTRL_SPEED_SHIFT)
#define DWA_IC_CTRL_SPEED_FAST		(2 << DWA_IC_CTRL_SPEED_SHIFT)
#define DWA_IC_CTRL_SPEED_HIGH		(3 << DWA_IC_CTRL_SPEED_SHIFT)
#define DWA_IC_CTRL_10BITADDR_TGT	BIT(8)
#define DWA_IC_CTRL_10BITADDR_CTRLR	BIT(9)
#define DWA_IC_CTRL_STOP_DET_IFADDRESSED	BIT(10)
#define DWA_IC_CTRL_TX_EMPTY_CTRL	BIT(11)
#define DWA_IC_CTRL_RX_FIFO_FULL_HLD	BIT(12)
#define DWA_IC_CTRL_STOP_DET_IF_ACTIVE	BIT(13)

/* IC_DATA_CMD -- same bit positions as DW_apb_i2c. */
#define DWA_IC_DATA_CMD_DAT_MASK	GENMASK(7, 0)
#define DWA_IC_DATA_CMD_CMD		BIT(8)	/* 1 = read, 0 = write */
#define DWA_IC_DATA_CMD_STOP		BIT(9)
#define DWA_IC_DATA_CMD_RESTART		BIT(10)

/* IC_STATUS -- same bit positions as DW_apb_i2c. */
#define DWA_IC_STATUS_ACTIVITY		BIT(0)
#define DWA_IC_STATUS_TFNF		BIT(1)
#define DWA_IC_STATUS_TFE		BIT(2)
#define DWA_IC_STATUS_RFNE		BIT(3)
#define DWA_IC_STATUS_RFF		BIT(4)
#define DWA_IC_STATUS_CTRLR_ACTIVITY	BIT(5)

/* IC_TX_TRMNT_SOURCE. Bits 0 and 3 match DW_apb_i2c's IC_TX_ABRT_SOURCE. */
#define DWA_TRMNT_7B_ADDR_NOACK		BIT(0)
#define DWA_TRMNT_10ADDR1_NOACK		BIT(1)
#define DWA_TRMNT_10ADDR2_NOACK		BIT(2)
#define DWA_TRMNT_TXDATA_NOACK		BIT(3)
#define DWA_TRMNT_GCALL_NOACK		BIT(4)
#define DWA_TRMNT_CTRLR_DIS		BIT(11)
#define DWA_TRMNT_ARB_LOST		BIT(12)
#define DWA_TRMNT_ADDR_NOACK_MASK \
	(DWA_TRMNT_7B_ADDR_NOACK | DWA_TRMNT_10ADDR1_NOACK | \
	 DWA_TRMNT_10ADDR2_NOACK)

#define DWA_IC_ENABLE_EN		BIT(0)

/* Expected identity. */
#define DWA_IC_COMP_TYPE_VALUE		0x44570140

#endif /* __I2C_DW_ADVANCED_REGS_H__ */
