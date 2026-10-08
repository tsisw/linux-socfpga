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
	u32 regs[TSI_SKYLP_REG_COUNT];	/* what a read returns; writes land here */
};

static u32 log_rd(void *ctx, enum tsi_skylp_reg reg)
{
	struct op_log *log = ctx;

	return log->regs[reg];
}

static void log_wr(void *ctx, enum tsi_skylp_reg reg, u32 val)
{
	struct op_log *log = ctx;

	if (log->nops < LOG_MAX) {
		log->ops[log->nops].kind = OP_WR;
		log->ops[log->nops].reg = reg;
		log->ops[log->nops].val = val;
	}
	log->nops++;
	log->regs[reg] = val;
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
		.rd = log_rd,
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
		.has_reset = true,
		.has_clksel = true,	.clksel_val = 0x1,
		.mux_mode = TSI_SKYLP_MUX_MODE_USB,
		.mux_flip = false,
	};

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_init_seq(&hw, &init), 0);
	KUNIT_ASSERT_EQ(test, log.nops, 7);

	KUNIT_EXPECT_EQ(test, log.ops[0].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[0].reg, TSI_SKYLP_REG_TSAR);
	KUNIT_EXPECT_EQ(test, log.ops[0].val, 0xa5u);

	KUNIT_EXPECT_EQ(test, log.ops[1].kind, OP_DELAY);
	KUNIT_EXPECT_GE(test, log.ops[1].val, 100u);

	/* resets next: por_n, settle, then the three USB bus releases */
	KUNIT_EXPECT_EQ(test, log.ops[2].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[2].reg, TSI_SKYLP_REG_RESET);
	KUNIT_EXPECT_EQ(test, log.ops[2].val, 0x1u);
	KUNIT_EXPECT_EQ(test, log.ops[3].kind, OP_DELAY);
	KUNIT_EXPECT_GE(test, log.ops[3].val, 100u);
	KUNIT_EXPECT_EQ(test, log.ops[4].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[4].reg, TSI_SKYLP_REG_RESET);
	KUNIT_EXPECT_EQ(test, log.ops[4].val, 0xfu);

	KUNIT_EXPECT_EQ(test, log.ops[5].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[5].reg, TSI_SKYLP_REG_CLKSEL);
	KUNIT_EXPECT_EQ(test, log.ops[5].val, 0x1u);

	/* and the mux write stays dead last: it sits behind apb_presetn */
	KUNIT_EXPECT_EQ(test, log.ops[6].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[6].reg, TSI_SKYLP_REG_MUX);
	KUNIT_EXPECT_EQ(test, log.ops[6].val, 0x7000u);
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

/*
 * udi_reset_cfg bit map per the hardware team (2026-10-07):
 * {22'h0, aud_axi_aresetn_1, aud_axi_aresetn_0, aud_rst_n,
 *  vid_axi_aresetn_1, vid_axi_aresetn_0, dptx_sys_rstn,
 *  usb_axi_aresetn, apb_presetn, ahb_hresetn, por_n} - active-low
 * releases, so set = out of reset. USB needs por_n (bit 0), then the
 * validated >= 100 us settle, then bits 1..3 in one write - without
 * touching the DP/video/audio releases other drivers own.
 */
static void skylp_test_reset_seq_por_then_buses(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	/* dptx (bit 4) already released by the display driver: survives */
	log.regs[TSI_SKYLP_REG_RESET] = 0x10;

	tsi_skylp_usb_reset_seq(&hw);
	KUNIT_ASSERT_EQ(test, log.nops, 3);
	KUNIT_EXPECT_EQ(test, log.ops[0].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[0].reg, TSI_SKYLP_REG_RESET);
	KUNIT_EXPECT_EQ(test, log.ops[0].val, 0x11u);
	KUNIT_EXPECT_EQ(test, log.ops[1].kind, OP_DELAY);
	KUNIT_EXPECT_GE(test, log.ops[1].val, 100u);
	KUNIT_EXPECT_EQ(test, log.ops[2].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[2].val, 0x1fu);
}

/* a part firmware already brought up sees no writes and no settle */
static void skylp_test_reset_seq_noop_when_released(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	log.regs[TSI_SKYLP_REG_RESET] = 0xf;
	tsi_skylp_usb_reset_seq(&hw);
	KUNIT_EXPECT_EQ(test, log.nops, 0);
}

/* por_n already out: the settle is long past, one bus-release write */
static void skylp_test_reset_seq_skips_settle_when_por_out(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	log.regs[TSI_SKYLP_REG_RESET] = 0x1;
	tsi_skylp_usb_reset_seq(&hw);
	KUNIT_ASSERT_EQ(test, log.nops, 1);
	KUNIT_EXPECT_EQ(test, log.ops[0].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[0].val, 0xfu);
}

/*
 * The controller's ref_clk pin is the 50 MHz FREF, or 25 MHz when
 * udi_clk_sel.div2_clken (bit 2, hardware team 2026-10-07) divides it:
 * Innosilicon spec the pin at 24 MHz and confirmed 25 MHz operation.
 */
static void skylp_test_ref_clk_follows_div2(struct kunit *test)
{
	struct tsi_skylp_init init = {};

	/* div2_clken RESETS TO 1 (HW-4): an untouched board divides */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 25000000UL);
	init.has_clksel = true;
	init.clksel_val = 0x4;	/* div2_clken kept set */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 25000000UL);
	/* an explicit clksel that CLEARS div2 gives the undivided base */
	init.clksel_val = 0x1;	/* refclk_sel only, div2 cleared */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 50000000UL);
	init.has_clksel = false;
	init.clksel_val = 0x1;	/* value without a window is never written */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 25000000UL);
}

/* refclk100m_sel (bit 1) swaps the base to FREF_100M before div2 */
static void skylp_test_ref_clk_follows_100m_base(struct kunit *test)
{
	struct tsi_skylp_init init = { .has_clksel = true };

	init.clksel_val = 0x2;	/* 100 MHz base, div2 cleared */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 100000000UL);
	init.clksel_val = 0x6;	/* 100 MHz base, halved */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 50000000UL);
	init.clksel_val = 0x7;	/* differential 100 MHz, halved */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_ref_clk_hz(&init), 50000000UL);
}

/* 100 MHz: period 10 ns, scale 100 MHz / 16 kHz = 6250 - both encodable */
static void skylp_test_refclk_seq_programs_100mhz(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 100000000UL), 0);
	KUNIT_EXPECT_EQ(test, log.regs[TSI_SKYLP_REG_GUCTL], 10u << 22);
	KUNIT_EXPECT_EQ(test, log.regs[TSI_SKYLP_REG_GCTL], 6250u << 19);
}

/* 25 MHz: period 40 ns, scale 25 MHz / 16 kHz = 1562 */
static void skylp_test_refclk_seq_programs_25mhz(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 25000000UL), 0);
	KUNIT_EXPECT_EQ(test, log.regs[TSI_SKYLP_REG_GUCTL], 40u << 22);
	KUNIT_EXPECT_EQ(test, log.regs[TSI_SKYLP_REG_GCTL], 1562u << 19);
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

/* The flip line drives orientation; a steady line reports no change. */
static void skylp_test_plug_sync_follows_flip_line(struct kunit *test)
{
	struct tsi_skylp_plug_state s = { };

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 1, 0),
			(unsigned int)TSI_SKYLP_PLUG_FLIP_CHANGED);
	KUNIT_EXPECT_TRUE(test, s.flip);

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 1, 0), 0u);
	KUNIT_EXPECT_TRUE(test, s.flip);

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 0, 0),
			(unsigned int)TSI_SKYLP_PLUG_FLIP_CHANGED);
	KUNIT_EXPECT_FALSE(test, s.flip);
}

/*
 * A failed GPIO read is a negative errno. Treated as a boolean that is
 * "set", which is the bug this guards against: one I2C timeout must not
 * flip the lane mux and drop the link. Each line keeps its last state.
 */
static void skylp_test_plug_sync_read_error_keeps_last_state(struct kunit *test)
{
	struct tsi_skylp_plug_state s = { .flip = false, .present = true };

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, -EIO, 1),
			(unsigned int)TSI_SKYLP_PLUG_READ_FAILED);
	KUNIT_EXPECT_FALSE(test, s.flip);
	KUNIT_EXPECT_TRUE(test, s.present);

	s.flip = true;
	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, -ETIMEDOUT, -EIO),
			(unsigned int)TSI_SKYLP_PLUG_READ_FAILED);
	KUNIT_EXPECT_TRUE(test, s.flip);
	KUNIT_EXPECT_TRUE(test, s.present);

	/* a good read on one line still lands while the other fails */
	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 0, -EIO),
			(unsigned int)(TSI_SKYLP_PLUG_FLIP_CHANGED |
				       TSI_SKYLP_PLUG_READ_FAILED));
	KUNIT_EXPECT_FALSE(test, s.flip);
	KUNIT_EXPECT_TRUE(test, s.present);
}

/* Attach and detach are each reported once, not on every poll. */
static void skylp_test_plug_sync_reports_presence_edges_once(struct kunit *test)
{
	struct tsi_skylp_plug_state s = { };

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 0, 1),
			(unsigned int)TSI_SKYLP_PLUG_PRESENCE_CHANGED);
	KUNIT_EXPECT_TRUE(test, s.present);

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 0, 1), 0u);
	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 0, 1), 0u);

	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 0, 0),
			(unsigned int)TSI_SKYLP_PLUG_PRESENCE_CHANGED);
	KUNIT_EXPECT_FALSE(test, s.present);

	/* both lines moving in one poll report both */
	KUNIT_EXPECT_EQ(test, tsi_skylp_plug_sync(&s, 1, 1),
			(unsigned int)(TSI_SKYLP_PLUG_FLIP_CHANGED |
				       TSI_SKYLP_PLUG_PRESENCE_CHANGED));
}

/*
 * 50 MHz: period 20 ns into GUCTL[31:22], scale 50 MHz / 16 kHz = 3125
 * into GCTL[31:19]. The RTL reset values (41 ns, 1560) are what is
 * replaced; every other bit of both registers is kept.
 */
static void skylp_test_refclk_seq_programs_50mhz(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);
	const u32 gctl_rest = 0x00012004;	/* PRTCAP, U2RSTECN, U2EXIT_LFPS */
	const u32 guctl_rest = 0x0000c0ab;

	log.regs[TSI_SKYLP_REG_GCTL] = (1560u << 19) | gctl_rest;
	log.regs[TSI_SKYLP_REG_GUCTL] = (41u << 22) | guctl_rest;

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 50000000UL), 0);
	KUNIT_ASSERT_EQ(test, log.nops, 2);
	KUNIT_EXPECT_EQ(test, log.ops[0].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.ops[1].kind, OP_WR);
	KUNIT_EXPECT_EQ(test, log.regs[TSI_SKYLP_REG_GUCTL], (20u << 22) | guctl_rest);
	KUNIT_EXPECT_EQ(test, log.regs[TSI_SKYLP_REG_GCTL], (3125u << 19) | gctl_rest);
}

/* Already programmed: nothing is written, so a resume re-run is free. */
static void skylp_test_refclk_seq_skips_when_current(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	log.regs[TSI_SKYLP_REG_GCTL] = 3125u << 19;
	log.regs[TSI_SKYLP_REG_GUCTL] = 20u << 22;

	KUNIT_ASSERT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 50000000UL), 0);
	KUNIT_EXPECT_EQ(test, log.nops, 0);
}

/* A rate the fields cannot encode is refused before anything is written. */
static void skylp_test_refclk_seq_rejects_unencodable_rate(struct kunit *test)
{
	struct op_log log = {};
	struct tsi_skylp_hw hw = test_hw(&log);

	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 0), -EINVAL);
	/* 200 MHz: scale 12500 does not fit 13 bits */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 200000000UL), -ERANGE);
	/* 500 kHz: period 2000 ns does not fit 10 bits */
	KUNIT_EXPECT_EQ(test, tsi_skylp_usb_refclk_seq(&hw, 500000UL), -ERANGE);
	KUNIT_EXPECT_EQ(test, log.nops, 0);
}

static struct kunit_case skylp_test_cases[] = {
	KUNIT_CASE(skylp_test_mux_val_usb),
	KUNIT_CASE(skylp_test_mux_val_usb_flipped),
	KUNIT_CASE(skylp_test_mux_val_4dp),
	KUNIT_CASE(skylp_test_seq_full_order),
	KUNIT_CASE(skylp_test_seq_mux_only),
	KUNIT_CASE(skylp_test_seq_tsar_only_keeps_settle),
	KUNIT_CASE(skylp_test_seq_rejects_bad_mode),
	KUNIT_CASE(skylp_test_refclk_seq_programs_50mhz),
	KUNIT_CASE(skylp_test_refclk_seq_skips_when_current),
	KUNIT_CASE(skylp_test_refclk_seq_rejects_unencodable_rate),
	KUNIT_CASE(skylp_test_reset_seq_por_then_buses),
	KUNIT_CASE(skylp_test_reset_seq_noop_when_released),
	KUNIT_CASE(skylp_test_reset_seq_skips_settle_when_por_out),
	KUNIT_CASE(skylp_test_ref_clk_follows_div2),
	KUNIT_CASE(skylp_test_ref_clk_follows_100m_base),
	KUNIT_CASE(skylp_test_refclk_seq_programs_100mhz),
	KUNIT_CASE(skylp_test_refclk_seq_programs_25mhz),
	KUNIT_CASE(skylp_test_mux_update_writes_only_on_change),
	KUNIT_CASE(skylp_test_typec_state_safe_is_no_connection),
	KUNIT_CASE(skylp_test_typec_state_usb),
	KUNIT_CASE(skylp_test_typec_dp_four_lane),
	KUNIT_CASE(skylp_test_typec_dp_two_lane_keeps_usb),
	KUNIT_CASE(skylp_test_typec_unknown_mode_rejected),
	KUNIT_CASE(skylp_test_plug_sync_follows_flip_line),
	KUNIT_CASE(skylp_test_plug_sync_read_error_keeps_last_state),
	KUNIT_CASE(skylp_test_plug_sync_reports_presence_edges_once),
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
