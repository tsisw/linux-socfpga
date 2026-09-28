/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI SkyLP video bridge: register layout and programming core.
 *
 * The Innosilicon UDI subsystem carries two video bridges. Each one reads
 * pixels from memory over AXI and feeds them, with timing, to the DP
 * controller. A bridge holds four channels at a 0x80 stride plus a block
 * shared by all four at +0x200. Offsets below are relative to one bridge;
 * bridge 0 sits at subsystem APB +0x400 (0x2401_0550 on SkyLP).
 *
 * Field layout is taken from the vendor IP-XACT APB register map
 * (inno_typec_apb_registers.xml, 20260907 drop). The reset values decode
 * to CEA 1280x720p60 with a 5120-byte stride, which is what fixes the units:
 * timings are pixels and lines, stride is bytes.
 *
 * This file carries no DRM object state so the register encoding can be
 * unit tested on its own.
 *
 * Copyright (C) 2026 Tsavorite Scalable Intelligence
 */
#ifndef _TSI_SKYLP_VB_H_
#define _TSI_SKYLP_VB_H_

#include <linux/bits.h>
#include <linux/types.h>

struct drm_display_mode;
struct fwnode_handle;

#define TSI_VB_NUM_CHANNELS		4
#define TSI_VB_CH_STRIDE		0x80

/* Per-channel registers, relative to the channel base */
#define TSI_VB_BASE_ADDR(n)		(0x00 + 4 * (n))	/* n = 0..7 */
#define TSI_VB_NUM_BASE_ADDR		8
#define TSI_VB_SIZE			0x20
#define   TSI_VB_SIZE_H			GENMASK(15, 0)
#define   TSI_VB_SIZE_V			GENMASK(31, 16)
#define TSI_VB_FORMAT			0x24
#define   TSI_VB_FORMAT_BUFID		GENMASK(3, 0)
#define   TSI_VB_FORMAT_CODE		GENMASK(7, 4)
#define   TSI_VB_FORMAT_4PIXEL		BIT(8)
#define   TSI_VB_FORMAT_MAP_ORDER	BIT(9)
#define TSI_VB_HPORCH			0x28
#define   TSI_VB_HPORCH_HFP		GENMASK(15, 0)
#define   TSI_VB_HPORCH_HBP		GENMASK(31, 16)
#define TSI_VB_LINE_STRIDE		0x2c
#define   TSI_VB_LINE_STRIDE_MAX	0xffff
#define TSI_VB_HSYNC			0x30
#define   TSI_VB_HSYNC_HSW		GENMASK(15, 0)
#define   TSI_VB_HSYNC_DSC_BYTES	GENMASK(31, 16)
#define TSI_VB_VTIMING			0x34
#define   TSI_VB_VTIMING_VFP		GENMASK(7, 0)
#define   TSI_VB_VTIMING_VBP		GENMASK(15, 8)
#define   TSI_VB_VTIMING_VSW		GENMASK(23, 16)
#define   TSI_VB_VTIMING_VOFST		GENMASK(27, 24)
#define   TSI_VB_VTIMING_VPOL_POS	BIT(28)
#define   TSI_VB_VTIMING_HPOL_POS	BIT(29)
#define TSI_VB_CTRL			0x38
#define   TSI_VB_CTRL_FIFO_RST_LINE	BIT(0)
#define   TSI_VB_CTRL_ADDR_AUTO_SWITCH	BIT(1)
#define   TSI_VB_CTRL_OUTSTANDING	GENMASK(7, 4)
#define   TSI_VB_CTRL_UNDERFLOW_CLR	BIT(8)
#define   TSI_VB_CTRL_FRAME_DONE_CLR	BIT(9)
#define   TSI_VB_CTRL_ARID		GENMASK(13, 10)
#define   TSI_VB_CTRL_ID_ERR_CLR	BIT(14)
#define   TSI_VB_CTRL_RESP_ERR_CLR	BIT(15)
#define   TSI_VB_CTRL_UNDERFLOW_MASK	BIT(16)
#define   TSI_VB_CTRL_FRAME_DONE_MASK	BIT(17)
#define TSI_VB_ENABLE			0x3c
#define   TSI_VB_ENABLE_DMA_READ	BIT(0)
#define   TSI_VB_ENABLE_VIDEO_GEN	BIT(1)
#define TSI_VB_STATUS			0x40
#define   TSI_VB_STATUS_UNDERFLOW	BIT(0)
#define   TSI_VB_STATUS_FRAME_DONE	BIT(1)
#define   TSI_VB_STATUS_ID_ERR		BIT(2)
#define   TSI_VB_STATUS_RESP_ERR	BIT(3)
#define   TSI_VB_STATUS_ALL		GENMASK(3, 0)

/* Registers shared by the four channels, relative to the bridge base */
#define TSI_VB_COMMON_AXI		0x200
#define   TSI_VB_AXI_ADDR_HIGH		GENMASK(7, 0)
#define   TSI_VB_AXI_ARQOS		GENMASK(11, 8)
#define TSI_VB_COMMON_AXI_ATTR		0x204
#define TSI_VB_COMMON_TIMING_MODE	0x208
#define   TSI_VB_TIMING_MODE_HDMI	BIT(0)	/* clear selects DP */

#define TSI_VB_WINDOW_SIZE		0x20c

/* VIDEO_FORMAT codes */
#define TSI_VB_FMT_ARGB8888		0
#define TSI_VB_FMT_RGB888		1

/* The bridge addresses 40 bits: 32 per base register plus a shared byte */
#define TSI_VB_DMA_ADDR_BITS		40

struct tsi_vb_timing {
	u16 hactive, hfp, hsw, hbp;
	u16 vactive;
	u8 vfp, vsw, vbp;
	bool hsync_pos, vsync_pos;
};

/*
 * Register access for one channel. @rd and @wr take offsets relative to the
 * bridge base; the core adds the channel offset itself.
 */
struct tsi_vb_hw {
	u32 (*rd)(void *ctx, u32 off);
	void (*wr)(void *ctx, u32 off, u32 val);
	void *ctx;
	unsigned int ch;
};

/* Scanout bookkeeping owned by the core. Callers serialise access. */
struct tsi_vb_state {
	bool started;
	unsigned int slot;
};

int tsi_vb_timing_from_mode(const struct drm_display_mode *mode,
			    struct tsi_vb_timing *t);
int tsi_vb_format_code(u32 fourcc, u32 *code);
int tsi_vb_check_scanout(u64 addr, u32 pitch, u32 height, u32 fourcc);

void tsi_vb_set_timing(const struct tsi_vb_hw *hw,
		       const struct tsi_vb_timing *t);
void tsi_vb_set_scanout(const struct tsi_vb_hw *hw, struct tsi_vb_state *st,
			u64 addr, u32 pitch, u32 code);
void tsi_vb_start(const struct tsi_vb_hw *hw, struct tsi_vb_state *st);
void tsi_vb_stop(const struct tsi_vb_hw *hw, struct tsi_vb_state *st);
void tsi_vb_frame_done_irq(const struct tsi_vb_hw *hw, bool enable);
u32 tsi_vb_irq_ack(const struct tsi_vb_hw *hw);

/*
 * Firmware description: the encoder this bridge feeds is the parent of the
 * remote end of port 0, endpoint 0. Works for devicetree, ACPI _DSD graphs
 * and software nodes alike. The caller puts the returned handle.
 */
struct fwnode_handle *tsi_skylp_remote_encoder(const struct fwnode_handle *node);

#endif /* _TSI_SKYLP_VB_H_ */
