// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the TSI SkyLP USB init-sequence core.
 *
 * Independent expectations, taken from the databook analysis in the
 * Confluence gap register (gap G2, §6c):
 *
 *  - the Type-C lane-mux value is bit12|bit13 bypass enables, mode in
 *    [15:14], flip in bit 16; USB mode = 1, so USB/no-flip = 0x7000,
 *    USB/flip = 0x17000, 4-lane DP/no-flip = 0xf000
 *  - the sequence order is TSAR block release, >= 100 us settle,
 *    refclk select, and the mux write LAST (mux routes lanes that must
 *    already be clocked)
 *  - steps whose values the hardware team has not supplied yet (TSAR,
 *    clksel) are skipped, but the mux write always happens: the mux
 *    resets to "no connection" and skipping it can never be right
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/test.h>
#include <linux/property.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>

#include "soc_skylp.h"

#define LOG_MAX	8

enum op_kind { OP_WR, OP_DELAY };

struct op_log {
	struct {
		enum op_kind kind;
		enum tsi_skylp_reg reg;
		u32 val;		/* written value, or delay in us */
	} ops[LOG_MAX];
	int nops;
};

static void log_wr(void *ctx, enum tsi_skylp_reg reg, u32 val)
{
	struct op_log *log = ctx;

	if (log->nops < LOG_MAX) {
		log->ops[log->nops].kind = OP_WR;
		log->ops[log->nops].reg = reg;
		log->ops[log->nops].val = val;
	}
	log->nops++;
}

static void log_delay(void *ctx, unsigned int us)
{
	struct op_log *log = ctx;

	if (log->nops < LOG_MAX) {
		log->ops[log->nops].kind = OP_DELAY;
		log->ops[log->nops].val = us;
	}
	log->nops++;
}

static struct tsi_skylp_hw test_hw(struct op_log *log)
{
	return (struct tsi_skylp_hw){
		.wr = log_wr,
		.delay_us = log_delay,
		.ctx = log,
	};
}

/* USB mode, no flip: both bypass enables + mode 1 = 0x7000 */
static void skylp_test_mux_val_usb(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_val(TSI_SKYLP_MUX_MODE_USB, false),
			0x7000u);
}

/* flip only adds bit 16 */
static void skylp_test_mux_val_usb_flipped(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_val(TSI_SKYLP_MUX_MODE_USB, true),
			0x17000u);
}

/* 4-lane DP is mode 3 */
static void skylp_test_mux_val_4dp(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_val(TSI_SKYLP_MUX_MODE_4DP, false),
			0xf000u);
}

/* full sequence: tsar -> settle >= 100us -> clksel -> mux, mux LAST */
static void skylp_test_seq_full_order(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);
	struct tsi_skylp_init init = {
		.has_tsar = true,	.tsar_val = 0xa5,
		.has_clksel = true,	.clksel_val = 0x1,
		.mux_mode = TSI_SKYLP_MUX_MODE_USB,
		.mux_flip = false,
	};

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_init_seq(&hw, &init), 0);
	KUNIT_ASSERT_EQ(test, log.nops, 4);

	KUNIT_EXPECT_EQ(test, log.ops[0].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[0].reg, TSI_SKYLP_REG_TSAR);
	KUNIT_EXPECT_EQ(test, log.ops[0].val, 0xa5u);

	KUNIT_EXPECT_EQ(test, log.ops[1].kind, OP_DELAY);
	KUNIT_EXPECT_GE(test, log.ops[1].val, 100u);

	KUNIT_EXPECT_EQ(test, log.ops[2].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[2].reg, TSI_SKYLP_REG_CLKSEL);
	KUNIT_EXPECT_EQ(test, log.ops[2].val, 0x1u);

	KUNIT_EXPECT_EQ(test, log.ops[3].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[3].reg, TSI_SKYLP_REG_MUX);
	KUNIT_EXPECT_EQ(test, log.ops[3].val, 0x7000u);
}

/* no tsar/clksel values yet: no writes for them, no settle delay,
 * but the mandatory mux write still lands (with flip this time)
 */
static void skylp_test_seq_mux_only(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);
	struct tsi_skylp_init init = {
		.mux_mode = TSI_SKYLP_MUX_MODE_USB,
		.mux_flip = true,
	};

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_init_seq(&hw, &init), 0);
	KUNIT_ASSERT_EQ(test, log.nops, 1);
	KUNIT_EXPECT_EQ(test, log.ops[0].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[0].reg, TSI_SKYLP_REG_MUX);
	KUNIT_EXPECT_EQ(test, log.ops[0].val, 0x17000u);
}

/* tsar without clksel: settle still separates tsar from the mux write */
static void skylp_test_seq_tsar_only_keeps_settle(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);
	struct tsi_skylp_init init = {
		.has_tsar = true,	.tsar_val = 0x1,
		.mux_mode = TSI_SKYLP_MUX_MODE_USB_2DP,
		.mux_flip = false,
	};

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_init_seq(&hw, &init), 0);
	KUNIT_ASSERT_EQ(test, log.nops, 3);
	KUNIT_EXPECT_EQ(test, log.ops[0].reg, TSI_SKYLP_REG_TSAR);
	KUNIT_EXPECT_EQ(test, log.ops[1].kind, OP_DELAY);
	KUNIT_EXPECT_EQ(test, log.ops[2].reg, TSI_SKYLP_REG_MUX);
	KUNIT_EXPECT_EQ(test, log.ops[2].val, 0xb000u);
}

/* mode out of the 2-bit field: reject before touching hardware */
static void skylp_test_seq_rejects_bad_mode(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);
	struct tsi_skylp_init init = {
		.mux_mode = 4,
	};

	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_init_seq(&hw, &init), -EINVAL);
	KUNIT_EXPECT_EQ(test, log.nops, 0);
}

/* first update always writes; repeating the same state writes nothing;
 * a flip change writes exactly the new value (glue interrupt path)
 */
static void skylp_test_mux_update_writes_only_on_change(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);
	struct tsi_skylp_mux_cache cache = TSI_SKYLP_MUX_CACHE_INIT;

	KUNIT_EXPECT_TRUE(test, tsi_skylp_mux_update(&cache, &hw,
						     TSI_SKYLP_MUX_MODE_USB,
						     false));
	KUNIT_ASSERT_EQ(test, log.nops, 1);
	KUNIT_EXPECT_EQ(test, log.ops[0].reg, TSI_SKYLP_REG_MUX);
	KUNIT_EXPECT_EQ(test, log.ops[0].val, 0x7000u);

	KUNIT_EXPECT_FALSE(test, tsi_skylp_mux_update(&cache, &hw,
						      TSI_SKYLP_MUX_MODE_USB,
						      false));
	KUNIT_EXPECT_EQ(test, log.nops, 1);

	KUNIT_EXPECT_TRUE(test, tsi_skylp_mux_update(&cache, &hw,
						     TSI_SKYLP_MUX_MODE_USB,
						     true));
	KUNIT_ASSERT_EQ(test, log.nops, 2);
	KUNIT_EXPECT_EQ(test, log.ops[1].val, 0x17000u);
}

/*
 * The four mux modes map exactly onto the Type-C states the kernel's
 * mode-switch passes down, so a port manager can drive this mux with no
 * SkyLP-specific knowledge. DP pin assignments: C and E are 4-lane DP,
 * D is 2-lane DP alongside USB3.
 */
static void skylp_test_typec_state_safe_is_no_connection(struct kunit *test)
{
	u32 mode = 0xff;

	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_mode_from_typec(TYPEC_STATE_SAFE, &mode),
			0);
	KUNIT_EXPECT_EQ(test, mode, (u32)TSI_SKYLP_MUX_MODE_NONE);
}

static void skylp_test_typec_state_usb(struct kunit *test)
{
	u32 mode = 0xff;

	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_mode_from_typec(TYPEC_STATE_USB, &mode),
			0);
	KUNIT_EXPECT_EQ(test, mode, (u32)TSI_SKYLP_MUX_MODE_USB);
}

static void skylp_test_typec_dp_four_lane(struct kunit *test)
{
	u32 mode = 0xff;

	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_mode_from_typec(TYPEC_DP_STATE_C, &mode),
			0);
	KUNIT_EXPECT_EQ(test, mode, (u32)TSI_SKYLP_MUX_MODE_4DP);

	mode = 0xff;
	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_mode_from_typec(TYPEC_DP_STATE_E, &mode),
			0);
	KUNIT_EXPECT_EQ(test, mode, (u32)TSI_SKYLP_MUX_MODE_4DP);
}

/* pin assignment D keeps USB3 alive alongside two DP lanes */
static void skylp_test_typec_dp_two_lane_keeps_usb(struct kunit *test)
{
	u32 mode = 0xff;

	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_mode_from_typec(TYPEC_DP_STATE_D, &mode),
			0);
	KUNIT_EXPECT_EQ(test, mode, (u32)TSI_SKYLP_MUX_MODE_USB_2DP);
}

/* an unsupported alt mode must be refused, not silently mapped */
static void skylp_test_typec_unknown_mode_rejected(struct kunit *test)
{
	u32 mode = 0xff;

	KUNIT_EXPECT_EQ(test,
			tsi_skylp_mux_mode_from_typec(TYPEC_DP_STATE_A, &mode),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, mode, 0xffu);
}

/* the USB node as the SkyLP SSDT describes it, plus the pending init words */
static const char *const fw_usb_compat[] = { "tsi,skylp-usb", "soc,soc_drd3" };

static const struct property_entry fw_usb_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("compatible", fw_usb_compat),
	PROPERTY_ENTRY_U32("tsi,mux-mode", TSI_SKYLP_MUX_MODE_USB_2DP),
	PROPERTY_ENTRY_BOOL("tsi,mux-flip"),
	PROPERTY_ENTRY_U32("tsi,tsar-init", 0x11),
	{ }
};

static const struct property_entry fw_usb_minimal_props[] = {
	PROPERTY_ENTRY_STRING("compatible", "tsi,skylp-usb"),
	{ }
};

static const struct property_entry fw_usb_other_props[] = {
	PROPERTY_ENTRY_STRING("compatible", "soc,soc_drd3"),
	{ }
};

static const struct property_entry fw_usb_badmode_props[] = {
	PROPERTY_ENTRY_STRING("compatible", "tsi,skylp-usb"),
	PROPERTY_ENTRY_U32("tsi,mux-mode", 7),
	{ }
};

static void skylp_test_usb_parse_fwnode(struct kunit *test)
{
	struct tsi_skylp_init init;
	struct fwnode_handle *fw;

	fw = fwnode_create_software_node(fw_usb_props, NULL);
	KUNIT_ASSERT_FALSE(test, IS_ERR_OR_NULL(fw));
	memset(&init, 0xa5, sizeof(init));
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_parse(fw, &init), 0);
	KUNIT_EXPECT_TRUE(test, init.has_tsar);
	KUNIT_EXPECT_EQ(test, init.tsar_val, 0x11u);
	KUNIT_EXPECT_FALSE(test, init.has_clksel);
	KUNIT_EXPECT_EQ(test, init.mux_mode, (u32)TSI_SKYLP_MUX_MODE_USB_2DP);
	KUNIT_EXPECT_TRUE(test, init.mux_flip);
	fwnode_remove_software_node(fw);

	/* nothing but compatible: USB lanes, no flip, no init words */
	fw = fwnode_create_software_node(fw_usb_minimal_props, NULL);
	KUNIT_ASSERT_FALSE(test, IS_ERR_OR_NULL(fw));
	memset(&init, 0xa5, sizeof(init));
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_parse(fw, &init), 0);
	KUNIT_EXPECT_FALSE(test, init.has_tsar);
	KUNIT_EXPECT_FALSE(test, init.has_clksel);
	KUNIT_EXPECT_EQ(test, init.mux_mode, (u32)TSI_SKYLP_MUX_MODE_USB);
	KUNIT_EXPECT_FALSE(test, init.mux_flip);
	fwnode_remove_software_node(fw);

	/* a plain vendor node is not ours */
	fw = fwnode_create_software_node(fw_usb_other_props, NULL);
	KUNIT_ASSERT_FALSE(test, IS_ERR_OR_NULL(fw));
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_parse(fw, &init), -ENODEV);
	fwnode_remove_software_node(fw);

	fw = fwnode_create_software_node(fw_usb_badmode_props, NULL);
	KUNIT_ASSERT_FALSE(test, IS_ERR_OR_NULL(fw));
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_parse(fw, &init), -EINVAL);
	fwnode_remove_software_node(fw);

	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_parse(NULL, &init), -ENODEV);
}

static struct kunit_case skylp_test_cases[] = {
	KUNIT_CASE(skylp_test_mux_val_usb),
	KUNIT_CASE(skylp_test_mux_val_usb_flipped),
	KUNIT_CASE(skylp_test_mux_val_4dp),
	KUNIT_CASE(skylp_test_seq_full_order),
	KUNIT_CASE(skylp_test_seq_mux_only),
	KUNIT_CASE(skylp_test_seq_tsar_only_keeps_settle),
	KUNIT_CASE(skylp_test_seq_rejects_bad_mode),
	KUNIT_CASE(skylp_test_mux_update_writes_only_on_change),
	KUNIT_CASE(skylp_test_typec_state_safe_is_no_connection),
	KUNIT_CASE(skylp_test_typec_state_usb),
	KUNIT_CASE(skylp_test_typec_dp_four_lane),
	KUNIT_CASE(skylp_test_typec_dp_two_lane_keeps_usb),
	KUNIT_CASE(skylp_test_typec_unknown_mode_rejected),
	KUNIT_CASE(skylp_test_usb_parse_fwnode),
	{}
};

static struct kunit_suite skylp_test_suite = {
	.name = "soc-drd-skylp",
	.test_cases = skylp_test_cases,
};
kunit_test_suite(skylp_test_suite);

MODULE_DESCRIPTION("KUnit tests for the TSI SkyLP USB init-sequence core");
MODULE_LICENSE("GPL");
