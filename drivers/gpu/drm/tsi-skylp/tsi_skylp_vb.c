// SPDX-License-Identifier: GPL-2.0-only
/*
 * TSI SkyLP video bridge programming core.
 *
 * Encodes DRM modes and framebuffers into video bridge register values and
 * owns the order in which they are written. All register access goes
 * through struct tsi_vb_hw so the sequences can be tested without hardware.
 *
 * Behaviour the IP-XACT map does not state, and which this file therefore
 * assumes (each is a question for Innosilicon):
 *
 *  - With ADDR_AUTO_SWITCH clear, the channel reads DMA_BASE_ADDR[FRAME_BUFID]
 *    and latches FRAME_BUFID at a frame boundary. Page flips write the idle
 *    slot and then switch FRAME_BUFID, so a mid-frame write cannot tear.
 *  - The *_CLEAR bits in CTRL are level controls: set to clear the flag,
 *    then released. They are pulsed.
 *  - The *_MASK bits in CTRL suppress the interrupt output when set.
 *  - VIDEO_FORMAT 0 ("ARGB888") is the DRM_FORMAT_XRGB8888 memory layout,
 *    a little-endian 32-bit word with blue in the low byte.
 *  - The bridge does not carry into ADDR_HIGH_8BIT as it walks a buffer, so
 *    a buffer must not straddle a 4 GiB boundary.
 *
 * Copyright (C) 2026 Tsavorite Scalable Intelligence
 */
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modes.h>

#include "tsi_skylp_vb.h"

#define TSI_VB_CTRL_ALL_CLR	(TSI_VB_CTRL_UNDERFLOW_CLR | \
				 TSI_VB_CTRL_FRAME_DONE_CLR | \
				 TSI_VB_CTRL_ID_ERR_CLR | \
				 TSI_VB_CTRL_RESP_ERR_CLR)

static u32 ch_rd(const struct tsi_vb_hw *hw, u32 reg)
{
	return hw->rd(hw->ctx, hw->ch * TSI_VB_CH_STRIDE + reg);
}

static void ch_wr(const struct tsi_vb_hw *hw, u32 reg, u32 val)
{
	hw->wr(hw->ctx, hw->ch * TSI_VB_CH_STRIDE + reg, val);
}

/**
 * tsi_vb_timing_from_mode - derive bridge timing from a display mode
 * @mode: mode with crtc_* fields filled in (an adjusted mode)
 * @t: result
 *
 * Return: 0, or -EINVAL if the bridge cannot generate @mode: interlaced,
 * double-scanned or pixel-repeated modes, a zero-width sync pulse, or a
 * vertical porch or sync wider than its 8-bit field.
 */
int tsi_vb_timing_from_mode(const struct drm_display_mode *mode,
			    struct tsi_vb_timing *t)
{
	int hfp, hsw, hbp, vfp, vsw, vbp;

	if (mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN |
			   DRM_MODE_FLAG_DBLCLK))
		return -EINVAL;
	if (!mode->crtc_hdisplay || !mode->crtc_vdisplay)
		return -EINVAL;

	hfp = mode->crtc_hsync_start - mode->crtc_hdisplay;
	hsw = mode->crtc_hsync_end - mode->crtc_hsync_start;
	hbp = mode->crtc_htotal - mode->crtc_hsync_end;
	vfp = mode->crtc_vsync_start - mode->crtc_vdisplay;
	vsw = mode->crtc_vsync_end - mode->crtc_vsync_start;
	vbp = mode->crtc_vtotal - mode->crtc_vsync_end;

	if (hfp < 0 || hsw < 1 || hbp < 0 || vfp < 0 || vsw < 1 || vbp < 0)
		return -EINVAL;
	if (hfp > U16_MAX || hsw > U16_MAX || hbp > U16_MAX)
		return -EINVAL;
	if (vfp > U8_MAX || vsw > U8_MAX || vbp > U8_MAX)
		return -EINVAL;

	t->hactive = mode->crtc_hdisplay;
	t->hfp = hfp;
	t->hsw = hsw;
	t->hbp = hbp;
	t->vactive = mode->crtc_vdisplay;
	t->vfp = vfp;
	t->vsw = vsw;
	t->vbp = vbp;
	t->hsync_pos = mode->flags & DRM_MODE_FLAG_PHSYNC;
	t->vsync_pos = mode->flags & DRM_MODE_FLAG_PVSYNC;
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_vb_timing_from_mode);

/**
 * tsi_vb_format_code - map a DRM fourcc to a VIDEO_FORMAT code
 * @fourcc: DRM_FORMAT_*
 * @code: result, untouched on error
 *
 * Only the 32-bit layout is mapped. The bridge ignores alpha, so ARGB8888
 * scans out as XRGB8888. The packed 24-bit code is left unmapped until its
 * byte order is confirmed.
 *
 * Return: 0, or -EINVAL for an unsupported format.
 */
int tsi_vb_format_code(u32 fourcc, u32 *code)
{
	switch (fourcc) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		*code = TSI_VB_FMT_ARGB8888;
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(tsi_vb_format_code);

/**
 * tsi_vb_check_scanout - can the bridge scan out this buffer?
 * @addr: bus address of the first visible pixel
 * @pitch: line stride in bytes
 * @height: visible lines
 * @fourcc: DRM_FORMAT_*
 *
 * Return: 0; -EINVAL for an unsupported format or a stride outside the
 * 16-bit LINE_STRIDE field; -ERANGE if the buffer reaches past 40 bits or
 * crosses a 4 GiB boundary (ADDR_HIGH_8BIT is one byte shared by the whole
 * scan).
 */
int tsi_vb_check_scanout(u64 addr, u32 pitch, u32 height, u32 fourcc)
{
	u32 code;
	u64 end;

	if (tsi_vb_format_code(fourcc, &code))
		return -EINVAL;
	if (!pitch || pitch > TSI_VB_LINE_STRIDE_MAX || !height)
		return -EINVAL;

	end = addr + (u64)pitch * height - 1;
	if (end < addr || end >> TSI_VB_DMA_ADDR_BITS)
		return -ERANGE;
	if (upper_32_bits(addr) != upper_32_bits(end))
		return -ERANGE;
	return 0;
}
EXPORT_SYMBOL_GPL(tsi_vb_check_scanout);

/**
 * tsi_vb_set_timing - program active size, porches, syncs and DP mode
 * @hw: channel access
 * @t: timing from tsi_vb_timing_from_mode()
 *
 * Writes the channel's timing registers and selects DP rather than HDMI
 * timing in the shared block. Does not start the channel.
 */
void tsi_vb_set_timing(const struct tsi_vb_hw *hw, const struct tsi_vb_timing *t)
{
	u32 vt;

	ch_wr(hw, TSI_VB_SIZE, FIELD_PREP(TSI_VB_SIZE_H, t->hactive) |
			       FIELD_PREP(TSI_VB_SIZE_V, t->vactive));
	ch_wr(hw, TSI_VB_HPORCH, FIELD_PREP(TSI_VB_HPORCH_HFP, t->hfp) |
				 FIELD_PREP(TSI_VB_HPORCH_HBP, t->hbp));
	ch_wr(hw, TSI_VB_HSYNC, FIELD_PREP(TSI_VB_HSYNC_HSW, t->hsw));

	vt = FIELD_PREP(TSI_VB_VTIMING_VFP, t->vfp) |
	     FIELD_PREP(TSI_VB_VTIMING_VBP, t->vbp) |
	     FIELD_PREP(TSI_VB_VTIMING_VSW, t->vsw);
	if (t->vsync_pos)
		vt |= TSI_VB_VTIMING_VPOL_POS;
	if (t->hsync_pos)
		vt |= TSI_VB_VTIMING_HPOL_POS;
	ch_wr(hw, TSI_VB_VTIMING, vt);

	hw->wr(hw->ctx, TSI_VB_COMMON_TIMING_MODE,
	       hw->rd(hw->ctx, TSI_VB_COMMON_TIMING_MODE) & ~TSI_VB_TIMING_MODE_HDMI);
}
EXPORT_SYMBOL_GPL(tsi_vb_set_timing);

/**
 * tsi_vb_set_scanout - point the channel at a framebuffer
 * @hw: channel access
 * @st: scanout state
 * @addr: bus address, already accepted by tsi_vb_check_scanout()
 * @pitch: line stride in bytes
 * @code: VIDEO_FORMAT code from tsi_vb_format_code()
 *
 * Before tsi_vb_start() this programs slot 0. Once running it programs the
 * slot that is not being scanned out, then switches FRAME_BUFID to it as
 * the final write, so the change takes effect as one step.
 */
void tsi_vb_set_scanout(const struct tsi_vb_hw *hw, struct tsi_vb_state *st,
			u64 addr, u32 pitch, u32 code)
{
	unsigned int slot = st->started ? st->slot ^ 1 : 0;
	u32 axi;

	ch_wr(hw, TSI_VB_BASE_ADDR(slot), lower_32_bits(addr));

	axi = hw->rd(hw->ctx, TSI_VB_COMMON_AXI) & ~TSI_VB_AXI_ADDR_HIGH;
	axi |= FIELD_PREP(TSI_VB_AXI_ADDR_HIGH, upper_32_bits(addr));
	hw->wr(hw->ctx, TSI_VB_COMMON_AXI, axi);

	ch_wr(hw, TSI_VB_LINE_STRIDE, pitch);

	/* 1 pixel per clock, MAP_ORDER at its reset value */
	ch_wr(hw, TSI_VB_FORMAT, FIELD_PREP(TSI_VB_FORMAT_BUFID, slot) |
				 FIELD_PREP(TSI_VB_FORMAT_CODE, code));
	st->slot = slot;
}
EXPORT_SYMBOL_GPL(tsi_vb_set_scanout);

/**
 * tsi_vb_start - start DMA and video generation
 * @hw: channel access
 * @st: scanout state
 *
 * Selects manual buffer switching and end-of-frame FIFO reset, masks the
 * underflow interrupt (underflow is still latched in STATUS and reported
 * at frame done), and keeps the AXI outstanding limit and ARID as found.
 * The frame-done mask is owned by tsi_vb_frame_done_irq().
 */
void tsi_vb_start(const struct tsi_vb_hw *hw, struct tsi_vb_state *st)
{
	u32 ctrl = ch_rd(hw, TSI_VB_CTRL);

	ctrl &= ~(TSI_VB_CTRL_ADDR_AUTO_SWITCH | TSI_VB_CTRL_FIFO_RST_LINE |
		  TSI_VB_CTRL_ALL_CLR);
	ctrl |= TSI_VB_CTRL_UNDERFLOW_MASK;
	ch_wr(hw, TSI_VB_CTRL, ctrl);

	ch_wr(hw, TSI_VB_ENABLE, TSI_VB_ENABLE_DMA_READ | TSI_VB_ENABLE_VIDEO_GEN);
	st->started = true;
}
EXPORT_SYMBOL_GPL(tsi_vb_start);

/**
 * tsi_vb_stop - stop the channel and mask its interrupts
 * @hw: channel access
 * @st: scanout state; the next tsi_vb_set_scanout() programs slot 0
 */
void tsi_vb_stop(const struct tsi_vb_hw *hw, struct tsi_vb_state *st)
{
	u32 ctrl;

	ch_wr(hw, TSI_VB_ENABLE, 0);

	ctrl = ch_rd(hw, TSI_VB_CTRL) & ~TSI_VB_CTRL_ALL_CLR;
	ctrl |= TSI_VB_CTRL_FRAME_DONE_MASK | TSI_VB_CTRL_UNDERFLOW_MASK;
	ch_wr(hw, TSI_VB_CTRL, ctrl);

	st->started = false;
	st->slot = 0;
}
EXPORT_SYMBOL_GPL(tsi_vb_stop);

/**
 * tsi_vb_frame_done_irq - unmask or mask the frame-done interrupt
 * @hw: channel access
 * @enable: true to unmask
 */
void tsi_vb_frame_done_irq(const struct tsi_vb_hw *hw, bool enable)
{
	u32 ctrl = ch_rd(hw, TSI_VB_CTRL) & ~TSI_VB_CTRL_ALL_CLR;

	if (enable)
		ctrl &= ~TSI_VB_CTRL_FRAME_DONE_MASK;
	else
		ctrl |= TSI_VB_CTRL_FRAME_DONE_MASK;
	ch_wr(hw, TSI_VB_CTRL, ctrl);
}
EXPORT_SYMBOL_GPL(tsi_vb_frame_done_irq);

/**
 * tsi_vb_irq_ack - read and clear the channel's status flags
 * @hw: channel access
 *
 * Return: the TSI_VB_STATUS_* flags that were set. Nothing is written when
 * none are, so a shared or spurious interrupt leaves the hardware alone.
 */
u32 tsi_vb_irq_ack(const struct tsi_vb_hw *hw)
{
	u32 status = ch_rd(hw, TSI_VB_STATUS) & TSI_VB_STATUS_ALL;
	u32 ctrl, clr = 0;

	if (!status)
		return 0;

	if (status & TSI_VB_STATUS_UNDERFLOW)
		clr |= TSI_VB_CTRL_UNDERFLOW_CLR;
	if (status & TSI_VB_STATUS_FRAME_DONE)
		clr |= TSI_VB_CTRL_FRAME_DONE_CLR;
	if (status & TSI_VB_STATUS_ID_ERR)
		clr |= TSI_VB_CTRL_ID_ERR_CLR;
	if (status & TSI_VB_STATUS_RESP_ERR)
		clr |= TSI_VB_CTRL_RESP_ERR_CLR;

	ctrl = ch_rd(hw, TSI_VB_CTRL) & ~TSI_VB_CTRL_ALL_CLR;
	ch_wr(hw, TSI_VB_CTRL, ctrl | clr);
	ch_wr(hw, TSI_VB_CTRL, ctrl);
	return status;
}
EXPORT_SYMBOL_GPL(tsi_vb_irq_ack);

struct fwnode_handle *tsi_skylp_remote_encoder(const struct fwnode_handle *node)
{
	struct fwnode_handle *ep, *remote;

	if (!node)
		return NULL;
	ep = fwnode_graph_get_endpoint_by_id(node, 0, 0, 0);
	if (!ep)
		return NULL;
	remote = fwnode_graph_get_remote_port_parent(ep);
	fwnode_handle_put(ep);
	return remote;
}
EXPORT_SYMBOL_GPL(tsi_skylp_remote_encoder);

MODULE_DESCRIPTION("TSI SkyLP video bridge programming core");
MODULE_LICENSE("GPL");
