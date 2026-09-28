// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the TSI SkyLP video bridge programming core.
 *
 * Copyright (C) 2026 Tsavorite Scalable Intelligence
 */
#include <kunit/test.h>
#include <linux/bitfield.h>
#include <linux/property.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modes.h>

#include "tsi_skylp_vb.h"

#define FAKE_REGS	(TSI_VB_WINDOW_SIZE / 4)
#define FAKE_LOG	64

struct fake_vb {
	u32 regs[FAKE_REGS];
	struct { u32 off, val; } log[FAKE_LOG];
	unsigned int nlog;
};

static u32 fake_rd(void *ctx, u32 off)
{
	struct fake_vb *f = ctx;

	return f->regs[off / 4];
}

static void fake_wr(void *ctx, u32 off, u32 val)
{
	struct fake_vb *f = ctx;

	f->regs[off / 4] = val;
	if (f->nlog < FAKE_LOG) {
		f->log[f->nlog].off = off;
		f->log[f->nlog].val = val;
		f->nlog++;
	}
}

static struct fake_vb *fake_init(struct kunit *test, struct tsi_vb_hw *hw,
				 unsigned int ch)
{
	struct fake_vb *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	hw->rd = fake_rd;
	hw->wr = fake_wr;
	hw->ctx = f;
	hw->ch = ch;
	return f;
}

/* Index of the last write to @off, or -1 */
static int last_write(const struct fake_vb *f, u32 off)
{
	int i;

	for (i = f->nlog - 1; i >= 0; i--)
		if (f->log[i].off == off)
			return i;
	return -1;
}

static void set_mode(struct drm_display_mode *m, u16 h, u16 hss, u16 hse,
		     u16 ht, u16 v, u16 vss, u16 vse, u16 vt, u32 flags)
{
	memset(m, 0, sizeof(*m));
	m->hdisplay = h;
	m->hsync_start = hss;
	m->hsync_end = hse;
	m->htotal = ht;
	m->vdisplay = v;
	m->vsync_start = vss;
	m->vsync_end = vse;
	m->vtotal = vt;
	m->flags = flags;
	drm_mode_set_crtcinfo(m, 0);
}

/* 720p60 is the bridge's reset programming, so it doubles as a units check */
static void vb_timing_720p_matches_reset_values(struct kunit *test)
{
	struct drm_display_mode m;
	struct tsi_vb_timing t;

	set_mode(&m, 1280, 1390, 1430, 1650, 720, 725, 730, 750,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	KUNIT_ASSERT_EQ(test, tsi_vb_timing_from_mode(&m, &t), 0);
	KUNIT_EXPECT_EQ(test, t.hactive, 0x500);	/* H_SIZE reset */
	KUNIT_EXPECT_EQ(test, t.hfp, 0x6e);		/* HFP reset */
	KUNIT_EXPECT_EQ(test, t.hsw, 0x28);		/* HSW reset */
	KUNIT_EXPECT_EQ(test, t.hbp, 220);
	KUNIT_EXPECT_EQ(test, t.vactive, 720);
	KUNIT_EXPECT_EQ(test, t.vfp, 5);		/* VFP reset */
	KUNIT_EXPECT_EQ(test, t.vsw, 5);
	KUNIT_EXPECT_EQ(test, t.vbp, 20);
	KUNIT_EXPECT_TRUE(test, t.hsync_pos);
	KUNIT_EXPECT_TRUE(test, t.vsync_pos);
}

static void vb_timing_negative_sync(struct kunit *test)
{
	struct drm_display_mode m;
	struct tsi_vb_timing t;

	/* VESA 640x480@60: both syncs negative */
	set_mode(&m, 640, 656, 752, 800, 480, 490, 492, 525,
		 DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	KUNIT_ASSERT_EQ(test, tsi_vb_timing_from_mode(&m, &t), 0);
	KUNIT_EXPECT_EQ(test, t.hfp, 16);
	KUNIT_EXPECT_EQ(test, t.hsw, 96);
	KUNIT_EXPECT_EQ(test, t.hbp, 48);
	KUNIT_EXPECT_EQ(test, t.vfp, 10);
	KUNIT_EXPECT_EQ(test, t.vsw, 2);
	KUNIT_EXPECT_EQ(test, t.vbp, 33);
	KUNIT_EXPECT_FALSE(test, t.hsync_pos);
	KUNIT_EXPECT_FALSE(test, t.vsync_pos);
}

static void vb_timing_rejects_interlace(struct kunit *test)
{
	struct drm_display_mode m;
	struct tsi_vb_timing t;

	set_mode(&m, 1920, 2008, 2052, 2200, 1080, 1084, 1094, 1125,
		 DRM_MODE_FLAG_INTERLACE);
	KUNIT_EXPECT_EQ(test, tsi_vb_timing_from_mode(&m, &t), -EINVAL);
}

/* VFP, VBP and VSW are 8-bit fields */
static void vb_timing_rejects_vporch_overflow(struct kunit *test)
{
	struct drm_display_mode m;
	struct tsi_vb_timing t;

	set_mode(&m, 640, 656, 752, 800, 480, 490, 492, 480 + 10 + 2 + 256, 0);
	KUNIT_EXPECT_EQ(test, tsi_vb_timing_from_mode(&m, &t), -EINVAL);
	set_mode(&m, 640, 656, 752, 800, 480, 480 + 256, 480 + 258, 480 + 300, 0);
	KUNIT_EXPECT_EQ(test, tsi_vb_timing_from_mode(&m, &t), -EINVAL);
}

static void vb_timing_rejects_zero_sync(struct kunit *test)
{
	struct drm_display_mode m;
	struct tsi_vb_timing t;

	set_mode(&m, 640, 656, 656, 800, 480, 490, 492, 525, 0);
	KUNIT_EXPECT_EQ(test, tsi_vb_timing_from_mode(&m, &t), -EINVAL);
	set_mode(&m, 640, 656, 752, 800, 480, 490, 490, 525, 0);
	KUNIT_EXPECT_EQ(test, tsi_vb_timing_from_mode(&m, &t), -EINVAL);
}

static void vb_format_codes(struct kunit *test)
{
	u32 code = 0xff;

	KUNIT_EXPECT_EQ(test, tsi_vb_format_code(DRM_FORMAT_XRGB8888, &code), 0);
	KUNIT_EXPECT_EQ(test, code, TSI_VB_FMT_ARGB8888);
	code = 0xff;
	KUNIT_EXPECT_EQ(test, tsi_vb_format_code(DRM_FORMAT_ARGB8888, &code), 0);
	KUNIT_EXPECT_EQ(test, code, TSI_VB_FMT_ARGB8888);
	code = 0xff;
	KUNIT_EXPECT_EQ(test, tsi_vb_format_code(DRM_FORMAT_RGB565, &code), -EINVAL);
	KUNIT_EXPECT_EQ(test, code, 0xff);
}

static void vb_check_scanout_limits(struct kunit *test)
{
	const u32 fmt = DRM_FORMAT_XRGB8888;

	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0x80000000ULL, 7680, 1080, fmt), 0);
	/* highest byte the bridge can address */
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0xff00000000ULL, 7680, 1080, fmt), 0);
	/* beyond 40 bits */
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0x10000000000ULL, 7680, 1080, fmt), -ERANGE);
	/* ends past 40 bits */
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0xffffff0000ULL, 7680, 1080, fmt), -ERANGE);
	/* straddles a 4 GiB boundary: the shared high byte cannot follow */
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0x1ffff0000ULL, 7680, 1080, fmt), -ERANGE);
	/* pitch is a 16-bit field */
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0x80000000ULL, 0x10000, 16, fmt), -EINVAL);
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0x80000000ULL, 0, 16, fmt), -EINVAL);
	KUNIT_EXPECT_EQ(test, tsi_vb_check_scanout(0x80000000ULL, 7680, 1080,
						   DRM_FORMAT_RGB565), -EINVAL);
}

static void vb_set_timing_programs_channel_and_dp_mode(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 1);
	const u32 cb = TSI_VB_CH_STRIDE;	/* channel 1 */
	struct tsi_vb_timing t = {
		.hactive = 1920, .hfp = 88, .hsw = 44, .hbp = 148,
		.vactive = 1080, .vfp = 4, .vsw = 5, .vbp = 36,
		.hsync_pos = true, .vsync_pos = false,
	};

	f->regs[TSI_VB_COMMON_TIMING_MODE / 4] = TSI_VB_TIMING_MODE_HDMI;
	tsi_vb_set_timing(&hw, &t);

	KUNIT_EXPECT_EQ(test, f->regs[(cb + TSI_VB_SIZE) / 4], (1080u << 16) | 1920);
	KUNIT_EXPECT_EQ(test, f->regs[(cb + TSI_VB_HPORCH) / 4], (148u << 16) | 88);
	KUNIT_EXPECT_EQ(test, f->regs[(cb + TSI_VB_HSYNC) / 4], 44u);
	KUNIT_EXPECT_EQ(test, f->regs[(cb + TSI_VB_VTIMING) / 4],
			TSI_VB_VTIMING_HPOL_POS | (5u << 16) | (36u << 8) | 4);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_COMMON_TIMING_MODE / 4], 0u);
	/* channel 0 untouched */
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_SIZE / 4], 0u);
}

static void vb_first_scanout_uses_slot0(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);
	struct tsi_vb_state st = { };
	int fmt_w, addr_w, high_w;

	f->regs[TSI_VB_COMMON_AXI / 4] = FIELD_PREP(TSI_VB_AXI_ARQOS, 0xa);
	tsi_vb_set_scanout(&hw, &st, 0x12345678c0ULL, 7680, TSI_VB_FMT_ARGB8888);

	KUNIT_EXPECT_EQ(test, st.slot, 0u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_BASE_ADDR(0) / 4], 0x345678c0u);
	/* high byte set, QoS preserved */
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_COMMON_AXI / 4], 0xa12u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_LINE_STRIDE / 4], 7680u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_FORMAT / 4], 0u);

	/* the buffer id selects the new buffer, so it must be written last */
	fmt_w = last_write(f, TSI_VB_FORMAT);
	addr_w = last_write(f, TSI_VB_BASE_ADDR(0));
	high_w = last_write(f, TSI_VB_COMMON_AXI);
	KUNIT_ASSERT_GE(test, fmt_w, 0);
	KUNIT_EXPECT_GT(test, fmt_w, addr_w);
	KUNIT_EXPECT_GT(test, fmt_w, high_w);
	KUNIT_EXPECT_EQ(test, (unsigned int)fmt_w, f->nlog - 1);
}

static void vb_flip_alternates_slots(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);
	struct tsi_vb_state st = { };

	tsi_vb_set_scanout(&hw, &st, 0x80000000ULL, 7680, TSI_VB_FMT_ARGB8888);
	tsi_vb_start(&hw, &st);

	tsi_vb_set_scanout(&hw, &st, 0x81000000ULL, 7680, TSI_VB_FMT_ARGB8888);
	KUNIT_EXPECT_EQ(test, st.slot, 1u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_BASE_ADDR(1) / 4], 0x81000000u);
	/* the slot being scanned out is not rewritten */
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_BASE_ADDR(0) / 4], 0x80000000u);
	KUNIT_EXPECT_EQ(test, FIELD_GET(TSI_VB_FORMAT_BUFID,
					f->regs[TSI_VB_FORMAT / 4]), 1u);

	tsi_vb_set_scanout(&hw, &st, 0x82000000ULL, 7680, TSI_VB_FMT_ARGB8888);
	KUNIT_EXPECT_EQ(test, st.slot, 0u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_BASE_ADDR(0) / 4], 0x82000000u);
	KUNIT_EXPECT_EQ(test, FIELD_GET(TSI_VB_FORMAT_BUFID,
					f->regs[TSI_VB_FORMAT / 4]), 0u);
}

static void vb_start_configures_then_enables(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);
	struct tsi_vb_state st = { };
	u32 ctrl;

	f->regs[TSI_VB_CTRL / 4] = TSI_VB_CTRL_ADDR_AUTO_SWITCH |
				   TSI_VB_CTRL_FIFO_RST_LINE |
				   FIELD_PREP(TSI_VB_CTRL_OUTSTANDING, 0x7) |
				   FIELD_PREP(TSI_VB_CTRL_ARID, 0x3);
	tsi_vb_start(&hw, &st);

	ctrl = f->regs[TSI_VB_CTRL / 4];
	KUNIT_EXPECT_TRUE(test, st.started);
	KUNIT_EXPECT_FALSE(test, ctrl & TSI_VB_CTRL_ADDR_AUTO_SWITCH);
	KUNIT_EXPECT_FALSE(test, ctrl & TSI_VB_CTRL_FIFO_RST_LINE);
	KUNIT_EXPECT_TRUE(test, ctrl & TSI_VB_CTRL_UNDERFLOW_MASK);
	KUNIT_EXPECT_EQ(test, FIELD_GET(TSI_VB_CTRL_OUTSTANDING, ctrl), 0x7u);
	KUNIT_EXPECT_EQ(test, FIELD_GET(TSI_VB_CTRL_ARID, ctrl), 0x3u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_ENABLE / 4],
			TSI_VB_ENABLE_DMA_READ | TSI_VB_ENABLE_VIDEO_GEN);
	KUNIT_EXPECT_GT(test, last_write(f, TSI_VB_ENABLE), last_write(f, TSI_VB_CTRL));
}

static void vb_stop_disables_and_resets_slot(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);
	struct tsi_vb_state st = { };

	tsi_vb_set_scanout(&hw, &st, 0x80000000ULL, 7680, TSI_VB_FMT_ARGB8888);
	tsi_vb_start(&hw, &st);
	tsi_vb_set_scanout(&hw, &st, 0x81000000ULL, 7680, TSI_VB_FMT_ARGB8888);
	tsi_vb_stop(&hw, &st);

	KUNIT_EXPECT_FALSE(test, st.started);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_ENABLE / 4], 0u);
	KUNIT_EXPECT_TRUE(test, f->regs[TSI_VB_CTRL / 4] & TSI_VB_CTRL_FRAME_DONE_MASK);

	/* after a stop the next scanout starts again at slot 0 */
	tsi_vb_set_scanout(&hw, &st, 0x83000000ULL, 7680, TSI_VB_FMT_ARGB8888);
	KUNIT_EXPECT_EQ(test, st.slot, 0u);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_BASE_ADDR(0) / 4], 0x83000000u);
}

static void vb_frame_done_irq_toggles_mask_only(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);
	const u32 other = TSI_VB_CTRL_UNDERFLOW_MASK |
			  FIELD_PREP(TSI_VB_CTRL_OUTSTANDING, 0x5);

	f->regs[TSI_VB_CTRL / 4] = other | TSI_VB_CTRL_FRAME_DONE_MASK;
	tsi_vb_frame_done_irq(&hw, true);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_CTRL / 4], other);
	tsi_vb_frame_done_irq(&hw, false);
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_CTRL / 4],
			other | TSI_VB_CTRL_FRAME_DONE_MASK);
}

static void vb_irq_ack_pulses_clear_bits(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);
	const u32 base = FIELD_PREP(TSI_VB_CTRL_OUTSTANDING, 0x5);
	int i, set_at = -1;

	f->regs[TSI_VB_CTRL / 4] = base;
	f->regs[TSI_VB_STATUS / 4] = TSI_VB_STATUS_FRAME_DONE |
				     TSI_VB_STATUS_UNDERFLOW;
	KUNIT_EXPECT_EQ(test, tsi_vb_irq_ack(&hw),
			TSI_VB_STATUS_FRAME_DONE | TSI_VB_STATUS_UNDERFLOW);

	for (i = 0; i < f->nlog; i++)
		if (f->log[i].off == TSI_VB_CTRL &&
		    (f->log[i].val & TSI_VB_CTRL_FRAME_DONE_CLR) &&
		    (f->log[i].val & TSI_VB_CTRL_UNDERFLOW_CLR))
			set_at = i;
	KUNIT_ASSERT_GE(test, set_at, 0);
	KUNIT_EXPECT_EQ(test, f->log[set_at].val & ~(TSI_VB_CTRL_FRAME_DONE_CLR |
						   TSI_VB_CTRL_UNDERFLOW_CLR), base);
	/* the clear bits are released again, leaving the rest as it was */
	KUNIT_EXPECT_EQ(test, f->regs[TSI_VB_CTRL / 4], base);
	KUNIT_EXPECT_GT(test, last_write(f, TSI_VB_CTRL), set_at);
}

static void vb_irq_ack_idle_writes_nothing(struct kunit *test)
{
	struct tsi_vb_hw hw;
	struct fake_vb *f = fake_init(test, &hw, 0);

	KUNIT_EXPECT_EQ(test, tsi_vb_irq_ack(&hw), 0u);
	KUNIT_EXPECT_EQ(test, f->nlog, 0u);
}

/*
 * The firmware graph exactly as the SkyLP SSDT (tools/tsi/acpi/skylp-udi.asl)
 * and the DT binding example describe it: bridge port@0/endpoint@0 whose
 * remote-endpoint is the DP controller's port@0/endpoint@0.
 */
static const struct software_node fw_dp = { .name = "DP00" };

static const struct property_entry fw_dp_port_props[] = {
	PROPERTY_ENTRY_U32("port", 0),
	{ }
};

static const struct software_node fw_dp_port = {
	.name = "port@0", .parent = &fw_dp, .properties = fw_dp_port_props,
};

static const struct property_entry fw_dp_ep_props[] = {
	PROPERTY_ENTRY_U32("reg", 0),
	{ }
};

static const struct software_node fw_dp_ep = {
	.name = "endpoint@0", .parent = &fw_dp_port, .properties = fw_dp_ep_props,
};

static const struct software_node fw_vb = { .name = "VBR0" };

static const struct software_node fw_vb_port = {
	.name = "port@0", .parent = &fw_vb, .properties = fw_dp_port_props,
};

static const struct property_entry fw_vb_ep_props[] = {
	PROPERTY_ENTRY_U32("reg", 0),
	PROPERTY_ENTRY_REF("remote-endpoint", &fw_dp_ep),
	{ }
};

static const struct software_node fw_vb_ep = {
	.name = "endpoint@0", .parent = &fw_vb_port, .properties = fw_vb_ep_props,
};

/* a bridge whose port has no endpoint: nothing to bind to */
static const struct software_node fw_vb_lonely = { .name = "VBR1" };

static const struct software_node fw_vb_lonely_port = {
	.name = "port@0", .parent = &fw_vb_lonely, .properties = fw_dp_port_props,
};

static const struct software_node *fw_nodes[] = {
	&fw_dp, &fw_dp_port, &fw_dp_ep, &fw_vb, &fw_vb_port, &fw_vb_ep,
	&fw_vb_lonely, &fw_vb_lonely_port, NULL,
};

static void fw_nodes_unregister(void *unused)
{
	software_node_unregister_node_group(fw_nodes);
}

static void vb_fw_remote_encoder_follows_graph(struct kunit *test)
{
	struct fwnode_handle *remote;

	KUNIT_ASSERT_EQ(test, software_node_register_node_group(fw_nodes), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fw_nodes_unregister, NULL), 0);

	remote = tsi_skylp_remote_encoder(software_node_fwnode(&fw_vb));
	KUNIT_ASSERT_NOT_NULL(test, remote);
	KUNIT_EXPECT_PTR_EQ(test, remote, software_node_fwnode(&fw_dp));
	fwnode_handle_put(remote);

	KUNIT_EXPECT_NULL(test, tsi_skylp_remote_encoder(software_node_fwnode(&fw_vb_lonely)));
	KUNIT_EXPECT_NULL(test, tsi_skylp_remote_encoder(NULL));
}

static struct kunit_case tsi_vb_cases[] = {
	KUNIT_CASE(vb_fw_remote_encoder_follows_graph),
	KUNIT_CASE(vb_timing_720p_matches_reset_values),
	KUNIT_CASE(vb_timing_negative_sync),
	KUNIT_CASE(vb_timing_rejects_interlace),
	KUNIT_CASE(vb_timing_rejects_vporch_overflow),
	KUNIT_CASE(vb_timing_rejects_zero_sync),
	KUNIT_CASE(vb_format_codes),
	KUNIT_CASE(vb_check_scanout_limits),
	KUNIT_CASE(vb_set_timing_programs_channel_and_dp_mode),
	KUNIT_CASE(vb_first_scanout_uses_slot0),
	KUNIT_CASE(vb_flip_alternates_slots),
	KUNIT_CASE(vb_start_configures_then_enables),
	KUNIT_CASE(vb_stop_disables_and_resets_slot),
	KUNIT_CASE(vb_frame_done_irq_toggles_mask_only),
	KUNIT_CASE(vb_irq_ack_pulses_clear_bits),
	KUNIT_CASE(vb_irq_ack_idle_writes_nothing),
	{}
};

static struct kunit_suite tsi_vb_suite = {
	.name = "tsi-skylp-video-bridge",
	.test_cases = tsi_vb_cases,
};

kunit_test_suite(tsi_vb_suite);

MODULE_DESCRIPTION("KUnit tests for the TSI SkyLP video bridge core");
MODULE_LICENSE("GPL");
