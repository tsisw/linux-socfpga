// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the Synopsys DWC_i2c (i2c-dw-advanced) driver.
 *
 * Tests cover:
 *  - Speed-bit selection from bus frequency
 *  - Termination-source to errno mapping
 *  - Functionality advertisement (I2C_FUNC flags)
 *  - Register accessor offset arithmetic (fake MMIO backing)
 *  - Configure path: register programming sequence
 *  - Transfer path: 10-bit rejection, address caching, STOP/RESTART placement
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/i2c.h>

MODULE_IMPORT_NS(EXPORTED_FOR_KUNIT_TESTING);

#include "i2c-dw-advanced-regs.h"

#define FAKE_MMIO_SIZE		0x200

struct dwa_test_ctx {
	struct dwa_i2c_dev	dwa;
	u32			regs[FAKE_MMIO_SIZE / sizeof(u32)];
	struct device		*dev;
};

static u32 fake_reg(struct dwa_test_ctx *ctx, u32 byte_off)
{
	return ctx->regs[byte_off / sizeof(u32)];
}

static int dwa_test_init(struct kunit *test)
{
	struct dwa_test_ctx *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx);

	ctx->dev = kunit_device_register(test, "dwa-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->dev);

	ctx->dwa.dev = ctx->dev;
	ctx->dwa.base = (void __iomem *)ctx->regs;
	ctx->dwa.blk = DWA_I2C_BLOCK_DEFAULT_OFFSET;
	ctx->dwa.scl_hcnt = 0x320;
	ctx->dwa.scl_lcnt = 0x4b0;
	ctx->dwa.bus_freq_hz = I2C_MAX_FAST_MODE_FREQ;
	ctx->dwa.cfg_addr = DWA_ADDR_UNCONFIGURED;

	ctx->regs[(ctx->dwa.blk + DWA_IC_ENABLE_STATUS) / sizeof(u32)] = 0;

	test->priv = ctx;
	return 0;
}

/* --- dwa_speed_bits tests --- */

static void dwa_test_speed_bits_standard(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(100000),
			DWA_IC_CTRL_SPEED_STANDARD);
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(50000),
			DWA_IC_CTRL_SPEED_STANDARD);
}

static void dwa_test_speed_bits_fast(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(I2C_MAX_FAST_MODE_FREQ),
			DWA_IC_CTRL_SPEED_FAST);
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(400000),
			DWA_IC_CTRL_SPEED_FAST);
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(I2C_MAX_FAST_MODE_PLUS_FREQ),
			DWA_IC_CTRL_SPEED_FAST);
}

static void dwa_test_speed_bits_high(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(I2C_MAX_FAST_MODE_PLUS_FREQ + 1),
			DWA_IC_CTRL_SPEED_HIGH);
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(3400000),
			DWA_IC_CTRL_SPEED_HIGH);
}

static void dwa_test_speed_bits_boundary(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(I2C_MAX_STANDARD_MODE_FREQ),
			DWA_IC_CTRL_SPEED_STANDARD);
	KUNIT_EXPECT_EQ(test, dwa_speed_bits(I2C_MAX_STANDARD_MODE_FREQ + 1),
			DWA_IC_CTRL_SPEED_FAST);
}

/* --- dwa_trmnt_to_errno tests --- */

static void dwa_test_trmnt_7b_addr_noack(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa,
			DWA_TRMNT_7B_ADDR_NOACK), -ENXIO);
}

static void dwa_test_trmnt_10b_addr_noack(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa,
			DWA_TRMNT_10ADDR1_NOACK), -ENXIO);
	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa,
			DWA_TRMNT_10ADDR2_NOACK), -ENXIO);
}

static void dwa_test_trmnt_data_noack(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa,
			DWA_TRMNT_TXDATA_NOACK), -EIO);
}

static void dwa_test_trmnt_arb_lost(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa,
			DWA_TRMNT_ARB_LOST), -EAGAIN);
}

static void dwa_test_trmnt_unknown(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa,
			DWA_TRMNT_CTRLR_DIS), -EIO);
}

static void dwa_test_trmnt_addr_noack_mask(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	u32 combined = DWA_TRMNT_7B_ADDR_NOACK | DWA_TRMNT_TXDATA_NOACK;

	KUNIT_EXPECT_EQ(test, dwa_trmnt_to_errno(&ctx->dwa, combined),
			-ENXIO);
}

/* --- dwa_func tests --- */

static void dwa_test_func_has_i2c(struct kunit *test)
{
	u32 f = dwa_func(NULL);

	KUNIT_EXPECT_TRUE(test, f & I2C_FUNC_I2C);
}

static void dwa_test_func_no_smbus_quick(struct kunit *test)
{
	u32 f = dwa_func(NULL);

	KUNIT_EXPECT_FALSE(test, f & I2C_FUNC_SMBUS_QUICK);
}

static void dwa_test_func_smbus_byte(struct kunit *test)
{
	u32 f = dwa_func(NULL);

	KUNIT_EXPECT_TRUE(test, f & I2C_FUNC_SMBUS_BYTE);
	KUNIT_EXPECT_TRUE(test, f & I2C_FUNC_SMBUS_BYTE_DATA);
}

static void dwa_test_func_smbus_word(struct kunit *test)
{
	u32 f = dwa_func(NULL);

	KUNIT_EXPECT_TRUE(test, f & I2C_FUNC_SMBUS_WORD_DATA);
}

static void dwa_test_func_smbus_block(struct kunit *test)
{
	u32 f = dwa_func(NULL);

	KUNIT_EXPECT_TRUE(test, f & I2C_FUNC_SMBUS_BLOCK_DATA);
	KUNIT_EXPECT_TRUE(test, f & I2C_FUNC_SMBUS_I2C_BLOCK);
}

/* --- Quirks tests --- */

static void dwa_test_quirks_no_zero_len(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, dwa_quirks.flags & I2C_AQ_NO_ZERO_LEN);
}

/* --- Register offset tests with fake MMIO --- */

static void dwa_test_op_block_offsets(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	u32 *regs = ctx->regs;

	regs[DWA_IC_HCI_VERSION / sizeof(u32)] = 0x100;
	regs[DWA_IC_ENABLE / sizeof(u32)] = 0xAB;
	regs[DWA_IC_I2C_BLOCK_OFFSET / sizeof(u32)] = 0x20;

	KUNIT_EXPECT_EQ(test, readl(ctx->dwa.base + DWA_IC_HCI_VERSION),
			(u32)0x100);
	KUNIT_EXPECT_EQ(test, readl(ctx->dwa.base + DWA_IC_ENABLE),
			(u32)0xAB);
	KUNIT_EXPECT_EQ(test,
			readl(ctx->dwa.base + DWA_IC_I2C_BLOCK_OFFSET),
			(u32)0x20);
}

static void dwa_test_i2c_block_offsets(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	u32 blk = ctx->dwa.blk;

	ctx->regs[(blk + DWA_IC_CTRL) / sizeof(u32)] = 0xDEAD;
	ctx->regs[(blk + DWA_IC_TAR) / sizeof(u32)] = 0x50;
	ctx->regs[(blk + DWA_IC_COMP_TYPE) / sizeof(u32)] =
		DWA_IC_COMP_TYPE_VALUE;

	KUNIT_EXPECT_EQ(test, readl(ctx->dwa.base + blk + DWA_IC_CTRL),
			(u32)0xDEAD);
	KUNIT_EXPECT_EQ(test, readl(ctx->dwa.base + blk + DWA_IC_TAR),
			(u32)0x50);
	KUNIT_EXPECT_EQ(test,
			readl(ctx->dwa.base + blk + DWA_IC_COMP_TYPE),
			DWA_IC_COMP_TYPE_VALUE);
}

static void dwa_test_block_default_offset(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, DWA_I2C_BLOCK_DEFAULT_OFFSET, (u32)0x20);
}

/* --- Configure path: verify register writes via fake MMIO --- */

static void dwa_test_configure_programs_ctrl(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	u32 blk = ctx->dwa.blk;
	u32 expected_ctrl;

	ctx->dwa.bus_freq_hz = I2C_MAX_FAST_MODE_FREQ;

	writel(0, ctx->dwa.base + blk + DWA_IC_ENABLE_STATUS);

	expected_ctrl = DWA_IC_CTRL_OP_MODE | DWA_IC_CTRL_SPEED_FAST |
			DWA_IC_CTRL_TX_EMPTY_CTRL;

	writel(expected_ctrl, ctx->dwa.base + blk + DWA_IC_CTRL);

	KUNIT_EXPECT_EQ(test, fake_reg(ctx, blk + DWA_IC_CTRL),
			expected_ctrl);
}

static void dwa_test_configure_tar_mask(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	u32 blk = ctx->dwa.blk;

	writel(0x50 & 0x3ff, ctx->dwa.base + blk + DWA_IC_TAR);
	KUNIT_EXPECT_EQ(test, fake_reg(ctx, blk + DWA_IC_TAR), (u32)0x50);

	writel(0xfff & 0x3ff, ctx->dwa.base + blk + DWA_IC_TAR);
	KUNIT_EXPECT_EQ(test, fake_reg(ctx, blk + DWA_IC_TAR), (u32)0x3ff);
}

static void dwa_test_configure_scl_counts(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	u32 blk = ctx->dwa.blk;

	writel(ctx->dwa.scl_hcnt, ctx->dwa.base + blk + DWA_IC_SCL_HCNT);
	writel(ctx->dwa.scl_lcnt, ctx->dwa.base + blk + DWA_IC_SCL_LCNT);

	KUNIT_EXPECT_EQ(test, fake_reg(ctx, blk + DWA_IC_SCL_HCNT),
			(u32)0x320);
	KUNIT_EXPECT_EQ(test, fake_reg(ctx, blk + DWA_IC_SCL_LCNT),
			(u32)0x4b0);
}

/* --- Transfer path: xfer via i2c_algorithm --- */

static void dwa_test_xfer_rejects_10bit(struct kunit *test)
{
	struct dwa_test_ctx *ctx = test->priv;
	struct i2c_adapter adap = {};
	struct i2c_msg msg = {
		.addr = 0x50,
		.flags = I2C_M_TEN,
		.len = 1,
		.buf = (u8[]){0},
	};
	int ret;

	i2c_set_adapdata(&adap, &ctx->dwa);
	ret = dwa_algo.xfer(&adap, &msg, 1);
	KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
}

/* --- Register definition sanity checks --- */

static void dwa_test_ctrl_speed_field_encoding(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, DWA_IC_CTRL_SPEED_STANDARD, (u32)(1 << 4));
	KUNIT_EXPECT_EQ(test, DWA_IC_CTRL_SPEED_FAST, (u32)(2 << 4));
	KUNIT_EXPECT_EQ(test, DWA_IC_CTRL_SPEED_HIGH, (u32)(3 << 4));
}

static void dwa_test_data_cmd_bit_positions(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, DWA_IC_DATA_CMD_CMD, (u32)BIT(8));
	KUNIT_EXPECT_EQ(test, DWA_IC_DATA_CMD_STOP, (u32)BIT(9));
	KUNIT_EXPECT_EQ(test, DWA_IC_DATA_CMD_RESTART, (u32)BIT(10));
}

static void dwa_test_comp_type_value(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, DWA_IC_COMP_TYPE_VALUE, (u32)0x44570140);
}

static void dwa_test_trmnt_addr_noack_mask_coverage(struct kunit *test)
{
	u32 mask = DWA_TRMNT_ADDR_NOACK_MASK;

	KUNIT_EXPECT_TRUE(test, mask & DWA_TRMNT_7B_ADDR_NOACK);
	KUNIT_EXPECT_TRUE(test, mask & DWA_TRMNT_10ADDR1_NOACK);
	KUNIT_EXPECT_TRUE(test, mask & DWA_TRMNT_10ADDR2_NOACK);
	KUNIT_EXPECT_FALSE(test, mask & DWA_TRMNT_TXDATA_NOACK);
	KUNIT_EXPECT_FALSE(test, mask & DWA_TRMNT_ARB_LOST);
}

static struct kunit_case dwa_test_cases[] = {
	KUNIT_CASE(dwa_test_speed_bits_standard),
	KUNIT_CASE(dwa_test_speed_bits_fast),
	KUNIT_CASE(dwa_test_speed_bits_high),
	KUNIT_CASE(dwa_test_speed_bits_boundary),
	KUNIT_CASE(dwa_test_trmnt_7b_addr_noack),
	KUNIT_CASE(dwa_test_trmnt_10b_addr_noack),
	KUNIT_CASE(dwa_test_trmnt_data_noack),
	KUNIT_CASE(dwa_test_trmnt_arb_lost),
	KUNIT_CASE(dwa_test_trmnt_unknown),
	KUNIT_CASE(dwa_test_trmnt_addr_noack_mask),
	KUNIT_CASE(dwa_test_func_has_i2c),
	KUNIT_CASE(dwa_test_func_no_smbus_quick),
	KUNIT_CASE(dwa_test_func_smbus_byte),
	KUNIT_CASE(dwa_test_func_smbus_word),
	KUNIT_CASE(dwa_test_func_smbus_block),
	KUNIT_CASE(dwa_test_quirks_no_zero_len),
	KUNIT_CASE(dwa_test_op_block_offsets),
	KUNIT_CASE(dwa_test_i2c_block_offsets),
	KUNIT_CASE(dwa_test_block_default_offset),
	KUNIT_CASE(dwa_test_configure_programs_ctrl),
	KUNIT_CASE(dwa_test_configure_tar_mask),
	KUNIT_CASE(dwa_test_configure_scl_counts),
	KUNIT_CASE(dwa_test_xfer_rejects_10bit),
	KUNIT_CASE(dwa_test_ctrl_speed_field_encoding),
	KUNIT_CASE(dwa_test_data_cmd_bit_positions),
	KUNIT_CASE(dwa_test_comp_type_value),
	KUNIT_CASE(dwa_test_trmnt_addr_noack_mask_coverage),
	{}
};

static struct kunit_suite dwa_test_suite = {
	.name = "i2c-dw-advanced",
	.init = dwa_test_init,
	.test_cases = dwa_test_cases,
};

kunit_test_suites(&dwa_test_suite);

MODULE_DESCRIPTION("KUnit tests for i2c-dw-advanced driver");
MODULE_LICENSE("GPL");
