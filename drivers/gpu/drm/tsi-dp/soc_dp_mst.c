#include <linux/phy/phy.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>

#include "soc_dp_dri.h"
#include "soc_dp_mst.h"
#include "soc_dp_reg_ops.h"

#define SOC_DP_MAX_DPCD_TRANSACTION_BYTES 16

struct soc_dp_mst_conn_state {
	struct drm_connector_state base;
	int  pbn;
	uint32_t color_format;
	uint32_t bpp;
};

#define to_soc_dp_mst_conn_state(x) \
	container_of(x, struct soc_dp_mst_conn_state, base)

struct soc_dp_mst_conn {
	struct drm_connector base;
	struct drm_dp_mst_topology_mgr *mgr;
	struct drm_dp_mst_port *port;
};

#define to_soc_dp_mst_conn(c) \
	container_of(c, struct soc_dp_mst_conn, base)

static void soc_dp_mst_set_mode(struct soc_dp_dev *dp, bool enable)
{
	soc_dp_reg_write_range(dp, SOC_DPTX_ENABLE_MST, enable ? 1 : 0);
}

void soc_dp_mst_trigger_act(struct soc_dp_dev *dp)
{
	soc_dp_reg_write_range(dp, SOC_DPTX_INITIATE_MST_ACT, 1);
}

#define SOC_DPTX_SLOT_STRIDE \
	(SOC_DPTX_REG_OFFSET(SOC_DPTX_STREAM_ID_SLOT1_0) - \
	 SOC_DPTX_REG_OFFSET(SOC_DPTX_STREAM_ID_SLOT0_0))

#define __SOC_DPTX_SLOT_N(off, hi, lo, n) \
	(off) + (n) * SOC_DPTX_SLOT_STRIDE, (hi), (lo)

#define SOC_DPTX_SLOT_N(base, n) \
	__SOC_DPTX_SLOT_N(base, n)

static void _soc_dp_mst_flush_vcpi(struct soc_dp_dev *dp, const uint8_t *slots)
{
	int i;

	for (i = 0; i < 8; i++) {
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_0, i), slots[i * 8 + 0]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_1, i), slots[i * 8 + 1]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_2, i), slots[i * 8 + 2]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_3, i), slots[i * 8 + 3]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_4, i), slots[i * 8 + 4]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_5, i), slots[i * 8 + 5]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_6, i), slots[i * 8 + 6]);
		soc_dp_reg_write_range(dp,
			SOC_DPTX_SLOT_N(SOC_DPTX_STREAM_ID_SLOT0_7, i), slots[i * 8 + 7]);
	}
}

static int soc_dp_get_stream_id(struct soc_dp_dev *dp, struct drm_crtc *crtc)
{
	int crtc_idx;
	int stream_id = -1;
	struct device_node *ep;

	if (!crtc)
		return -EINVAL;

	crtc_idx = drm_crtc_index(crtc);

	if (crtc_idx >= 0 && crtc_idx < 32 && dp->crtc_stream_map[crtc_idx] >= 0)
		return dp->crtc_stream_map[crtc_idx];

	if (!dp->dev->of_node)
		goto fallback;

	for_each_endpoint_of_node(dp->dev->of_node, ep) {
		struct device_node *remote_port = of_graph_get_remote_port(ep);
		struct device_node *remote_node = of_graph_get_remote_port_parent(ep);
		bool match = false;

		if (crtc->port && (remote_port == crtc->port || remote_node == crtc->port))
			match = true;
		else if (crtc->dev && crtc->dev->dev && crtc->dev->dev->of_node &&
				(remote_port == crtc->dev->dev->of_node || remote_node == crtc->dev->dev->of_node))
			match = true;

		of_node_put(remote_port);
		of_node_put(remote_node);

		if (match) {
			struct device_node *local_port = of_get_parent(ep);

			if (local_port) {
				if (of_property_read_u32(local_port, "reg", &stream_id) < 0)
					dev_warn(dp->dev, "Port node missing 'reg' property\n");
				of_node_put(local_port);
			}
			of_node_put(ep);
			break;
		}
	}

fallback:
	if (stream_id >= 0 && stream_id < dp->max_mst_streams) {
		if (crtc_idx >= 0 && crtc_idx < 32)
			dp->crtc_stream_map[crtc_idx] = stream_id;
		return stream_id;
	}

	dev_err(dp->dev, "No OF graph mapping for CRTC %d\n", crtc_idx);
	return -EINVAL;
}

/*
 * soc_dp_mst_sync_slots - Synchronize hardware slot registers with DRM payload state
 * @dp: DP device structure
 *
 * Reads the consolidated payload allocations from the DRM topology manager,
 * maps each active VCPI to its corresponding hardware stream ID, and flashes
 * the full 64-timeslot table configuration to the hardware.
 */
static void soc_dp_mst_sync_slots(struct soc_dp_dev *dp, struct drm_atomic_state *state)
{
	int j;
	uint8_t slots[64] = {0};
	struct drm_dp_mst_topology_mgr *mgr = &dp->mst_mgr;
	struct drm_dp_mst_topology_state *mst_state;
	struct drm_dp_mst_atomic_payload *payload;

	mst_state = drm_atomic_get_new_mst_topology_state(state, mgr);
	if (!mst_state)
		return;

	list_for_each_entry(payload, &mst_state->payloads, next) {
		int stream_id = -1;
		struct drm_dp_mst_port *port = payload->port;
		struct drm_crtc *crtc = NULL;

		if (!port || !payload->time_slots)
			continue;

		if (port->connector && port->connector->state)
			crtc = port->connector->state->crtc;

		if (crtc)
			stream_id = soc_dp_get_stream_id(dp, crtc);

		if (stream_id >= 0 && stream_id < dp->max_mst_streams) {
			uint8_t hw_stream_val = stream_id + 1;
			int start = payload->vc_start_slot;
			int num = payload->time_slots;

			for (j = 0; j < num; j++) {
				if ((start + j) < 64)
					slots[start + j] = hw_stream_val;
			}
		}
	}

	_soc_dp_mst_flush_vcpi(dp, slots);
}

void soc_dp_mst_stream_enable(struct soc_dp_dev *dp, uint8_t stream_id)
{
	if (unlikely(stream_id >= dp->max_mst_streams)) {
		dev_err(dp->dev, "Stream enable invalid stream_id %u\n", stream_id);
		return;
	}

	dev_info(dp->dev, "Enabling Video Stream %u\n", stream_id);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VIDEO_STREAM_ENABLE, 1);
}

void soc_dp_mst_stream_disable(struct soc_dp_dev *dp, uint8_t stream_id)
{
	if (unlikely(stream_id >= dp->max_mst_streams)) {
		dev_err(dp->dev, "Stream disable invalid stream_id %u\n", stream_id);
		return;
	}

	dev_info(dp->dev, "Disabling Video Stream %u\n", stream_id);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VIDEO_STREAM_ENABLE, 0);
}

/*
 * soc_dp_div64 - Wrapper for 64-bit division to handle ACTIVATE_DO_DIV
 * @n: Pointer to dividend (will be updated to quotient)
 * @base: Divisor
 * Return: Remainder
 */
static uint32_t soc_dp_div64(uint64_t *n, uint32_t base)
{
#ifdef CONFIG_SOC_DP_ACTIVATE_DO_DIV
	return do_div(*n, base);
#else
	uint32_t rem = *n % base;
	*n = *n / base;
	return rem;
#endif
}

void soc_dp_mst_set_stream_msa(struct soc_dp_dev *dp, uint8_t stream_id,
		const struct drm_display_mode *mode,
		uint32_t color_format, enum soc_dp_link_rate rate,
		enum soc_dp_lane_count lanes)
{
	/*
	 * Per-stream MSA configuration pipeline:
	 *   1. Derive bpp and MISC0 from color_format
	 *   2. Compute HBlank interval (hb_num)
	 *   3. Compute Transfer Unit (tu = pixel_clk * bpp * 800 / lanes / rate)
	 *   4. Determine FIFO read threshold from TU and blanking size
	 *   5. Write all timing/polarity/MSA/link-layer registers via stream API
	 */
	uint64_t hb_num, temp_tu;
	uint32_t link_rate_khz;
	uint32_t pixel_clk_khz;
	uint32_t bpp, misc0, den;
	uint32_t tu, tu_frac, tu_int;
	uint32_t hsync_len, rd_thres;

	if (unlikely(stream_id >= dp->max_mst_streams)) {
		dev_err(dp->dev, "Set stream MSA invalid stream_id %u\n", stream_id);
		return;
	}

	// 1. Prepare basic parameters
	pixel_clk_khz = mode->clock;
	if (!pixel_clk_khz)
		pixel_clk_khz = 1; // Prevent division by zero

	// Get BPP
	bpp = soc_dp_get_bpp(color_format);

	// Calculate MISC0
	// bit0: 0 (Sync Clock)
	// bits1-7: Color Format (000=RGB, 001=YCbCr422, 010=YCbCr444)
	// bits5-7: BPC (001=8bpc, 010=10bpc, etc)
	switch (color_format) {
	case SOC_VIDEO_RGB_6BIT:      misc0 = 0x00; break;
	case SOC_VIDEO_RGB_8BIT:      misc0 = 0x20; break;
	case SOC_VIDEO_RGB_10BIT:     misc0 = 0x40; break;
	case SOC_VIDEO_RGB_12BIT:     misc0 = 0x60; break;
	case SOC_VIDEO_RGB_16BIT:     misc0 = 0x80; break;
	case SOC_VIDEO_YUV422_8BIT:   misc0 = 0x22; break;
	case SOC_VIDEO_YUV422_10BIT:  misc0 = 0x42; break;
	case SOC_VIDEO_YUV422_12BIT:  misc0 = 0x62; break;
	case SOC_VIDEO_YUV422_16BIT:  misc0 = 0x82; break;
	case SOC_VIDEO_YUV444_8BIT:   misc0 = 0x24; break;
	case SOC_VIDEO_YUV444_10BIT:  misc0 = 0x44; break;
	case SOC_VIDEO_YUV444_12BIT:  misc0 = 0x64; break;
	case SOC_VIDEO_YUV444_16BIT:  misc0 = 0x84; break;
	default:                      misc0 = 0x20; break;
	}

	// 2. Calculate HBlank Interval (hb_num)
	// hb_num = hblank * (LinkRate_kHz / 10000) / 4 / (PixelClock_kHz / 1000)
	// Optimized Formula: hb_num = hblank * LinkRate_kHz / (40 * PixelClock_kHz)
	link_rate_khz = rate; // e.g., 1620000

	hb_num = (uint64_t)(mode->htotal - mode->hdisplay) * link_rate_khz;
	den = 40 * pixel_clk_khz;

	hb_num += (den / 2);
	soc_dp_div64(&hb_num, den);

	// 3. Calculate TU (Transfer Unit)
	// tu = (PixelClock_kHz/1000) * bpp * 640 / (8 * lanes * (LinkRate_kHz/10000))
	// Optimized Formula: tu = PixelClock_kHz * bpp * 800 / (lanes * LinkRate_kHz)
	temp_tu = (uint64_t)pixel_clk_khz * bpp * 800;
	den = lanes * link_rate_khz;

	// Note: TU is typically floored or carefully rounded.
	// Adding rounding here to be safe.
	temp_tu += (den / 2);

	soc_dp_div64(&temp_tu, den);
	tu = temp_tu;
	tu_frac = tu % 10;
	tu_int  = tu / 10;

	// 4. Calculate FIFO read threshold
	if (tu_int < 6)
		rd_thres = 32;
	else if ((mode->htotal - mode->hdisplay) < 80)
		rd_thres = 12;
	else
		rd_thres = 16;

	dev_info(dp->dev, "MSA: stream %u, %ux%u, Rate:%d kHz, Lanes:%d, BPP:%u, TU:%u.%u\n",
			stream_id, mode->hdisplay, mode->vdisplay, rate, lanes, bpp, tu_int, tu_frac);

	// 5. Video mapping format
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VIDEO_MAPPING, color_format);

	// Polarity configuration
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_HSYNC_IN_POLARITY, 1);
	else
		soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_HSYNC_IN_POLARITY, 0);

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VSYNC_IN_POLARITY, 1);
	else
		soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VSYNC_IN_POLARITY, 0);

	// Basic timing
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_HACTIVE, mode->hdisplay);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VACTIVE, mode->vdisplay);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_HBLANK, mode->htotal - mode->hdisplay);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VBLANK, mode->vtotal - mode->vdisplay);

	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_HSTART, mode->htotal - mode->hsync_start);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_VSTART, mode->vtotal - mode->vsync_start);

	hsync_len = mode->hsync_end - mode->hsync_start;

	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_H_SYNC_WIDTH, hsync_len);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_V_SYNC_WIDTH, mode->vsync_end - mode->vsync_start);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_H_FRONT_PORCH, mode->hsync_start - mode->hdisplay);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_V_FRONT_PORCH, mode->vsync_start - mode->vdisplay);

	// MSA and MISC
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_MISC0, misc0);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_MISC1, 0);

	// Link layer parameters
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_HBLANK_INTERVAL, (uint32_t)hb_num);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_AVERAGE_BYTES_PER_TU, tu_int);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_AVERAGE_BYTES_PER_TU_FRAC, tu_frac);
	soc_dp_stream_reg_write_range(dp, stream_id, SOC_DPTX_INIT_THRESHOLD, rd_thres);
}

static enum drm_connector_status
soc_dp_mst_conn_detect(struct drm_connector *connector, bool force)
{
	int ret;
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);
	struct drm_modeset_acquire_ctx ctx;

	drm_modeset_acquire_init(&ctx, 0);
retry:
	ret = drm_dp_mst_detect_port(connector, &ctx,
			mst_conn->mgr, mst_conn->port);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
		goto retry;
	}
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret < 0 ? connector_status_disconnected : ret;
}

static int soc_dp_mst_conn_detect_ctx(struct drm_connector *connector,
		struct drm_modeset_acquire_ctx *ctx, bool force)
{
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);

	return drm_dp_mst_detect_port(connector, ctx, mst_conn->mgr, mst_conn->port);
}

static int soc_dp_mst_conn_get_modes(struct drm_connector *connector)
{
	int count;
	struct edid *edid;
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);

	edid = drm_dp_mst_get_edid(connector, mst_conn->mgr, mst_conn->port);
	drm_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);
	kfree(edid);

	return count;
}

static struct drm_encoder *
soc_dp_mst_atomic_best_encoder(struct drm_connector *connector,
		struct drm_atomic_state *state)
{
	int stream_id;
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);
	struct soc_dp_dev *dp = container_of(mst_conn->mgr, struct soc_dp_dev, mst_mgr);
	struct drm_connector_state *connector_state;

	connector_state = drm_atomic_get_new_connector_state(state, connector);
	if (!connector_state || !connector_state->crtc)
		return NULL;

	stream_id = soc_dp_get_stream_id(dp, connector_state->crtc);
	if (stream_id < 0 || stream_id >= dp->max_mst_streams)
		return NULL;

	return &dp->mst_encoders[stream_id].base;
}

static int soc_dp_mst_conn_atomic_check(struct drm_connector *connector,
		struct drm_atomic_state *state)
{
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);
	struct drm_connector_state *old_conn_state =
		drm_atomic_get_old_connector_state(state, connector);
	struct drm_connector_state *new_conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_crtc_state *new_crtc_state = NULL;

	if (new_conn_state->crtc) {
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_conn_state->crtc);
		if (!new_crtc_state)
			return -EINVAL;
	}

	if (mst_conn->port->pdt == DP_PEER_DEVICE_MST_BRANCHING)
		return -EINVAL;

	if (old_conn_state->crtc) {
		struct soc_dp_mst_conn_state *old_soc_state =
			to_soc_dp_mst_conn_state(old_conn_state);

		if (old_soc_state->pbn > 0
				&& (!new_conn_state->crtc || !new_crtc_state->enable)) {
			int ret = drm_dp_atomic_release_time_slots(state,
					mst_conn->mgr, mst_conn->port);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static void soc_dp_mst_conn_destroy(struct drm_connector *connector)
{
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);

	if (mst_conn->port)
		drm_dp_mst_put_port_malloc(mst_conn->port);

	drm_connector_cleanup(connector);
	kfree(mst_conn);
}

static void soc_dp_mst_conn_reset(struct drm_connector *connector)
{
	struct soc_dp_mst_conn_state *state;

	if (connector->state) {
		__drm_atomic_helper_connector_destroy_state(connector->state);
		kfree(to_soc_dp_mst_conn_state(connector->state));
	}
	connector->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		__drm_atomic_helper_connector_reset(connector, &state->base);
}

static struct drm_connector_state *
soc_dp_mst_conn_atomic_duplicate_state(struct drm_connector *connector)
{
	struct soc_dp_mst_conn_state *state, *old_state;

	if (WARN_ON(!connector->state))
		return NULL;

	old_state = to_soc_dp_mst_conn_state(connector->state);
	state = kmemdup(old_state, sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_connector_duplicate_state(connector, &state->base);
	return &state->base;
}

static void soc_dp_mst_conn_atomic_destroy_state(struct drm_connector *connector,
		struct drm_connector_state *state)
{
	struct soc_dp_mst_conn_state *soc_state = to_soc_dp_mst_conn_state(state);

	__drm_atomic_helper_connector_destroy_state(state);
	kfree(soc_state);
}

static const struct drm_connector_funcs soc_dp_mst_connector_funcs = {
	.fill_modes     = drm_helper_probe_single_connector_modes,
	.destroy        = soc_dp_mst_conn_destroy,
	.detect         = soc_dp_mst_conn_detect,
	.reset          = soc_dp_mst_conn_reset,
	.atomic_duplicate_state = soc_dp_mst_conn_atomic_duplicate_state,
	.atomic_destroy_state   = soc_dp_mst_conn_atomic_destroy_state,
};

static const struct drm_connector_helper_funcs soc_dp_mst_conn_helper_funcs = {
	.detect_ctx     = soc_dp_mst_conn_detect_ctx,
	.get_modes      = soc_dp_mst_conn_get_modes,
	.atomic_check   = soc_dp_mst_conn_atomic_check,
	.atomic_best_encoder    = soc_dp_mst_atomic_best_encoder,
};

static const struct drm_encoder_funcs soc_dp_mst_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct drm_connector *
soc_dp_mst_add_connector(struct drm_dp_mst_topology_mgr *mgr,
		struct drm_dp_mst_port *port, const char *path)
{
	int i, ret;
	struct soc_dp_mst_conn *mst_conn;
	struct drm_connector *connector;
	struct soc_dp_dev *dp = container_of(mgr, struct soc_dp_dev, mst_mgr);

	mst_conn = kzalloc(sizeof(*mst_conn), GFP_KERNEL);
	if (!mst_conn)
		return NULL;

	connector = &mst_conn->base;

	mst_conn->mgr  = mgr;
	mst_conn->port = port;

	drm_dp_mst_get_port_malloc(port);

	ret = drm_connector_init(mgr->dev, connector,
			&soc_dp_mst_connector_funcs, DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		drm_dp_mst_put_port_malloc(port);
		kfree(mst_conn);
		return NULL;
	}

	drm_connector_helper_add(connector, &soc_dp_mst_conn_helper_funcs);

	for (i = 0; i < SOC_MAX_CRTC_NUM; i++)
		drm_connector_attach_encoder(connector, &dp->mst_encoders[i].base);

	drm_atomic_helper_connector_reset(connector);
	return connector;
}

static const struct drm_dp_mst_topology_cbs soc_dp_mst_topology_cbs = {
	.add_connector = soc_dp_mst_add_connector,
};

static enum drm_mode_status soc_dp_mst_encoder_mode_valid(struct drm_encoder *crtc,
		const struct drm_display_mode *mode)
{
	/* FPGA DEBUG */
	// if (mode->hdisplay > 3840 || mode->vdisplay > 2160)
	if (mode->hdisplay > 640 || mode->vdisplay > 480)
		return MODE_BAD_HVALUE;

	return MODE_OK;
}

static void soc_dp_mst_encoder_atomic_enable(struct drm_encoder *encoder,
		struct drm_atomic_state *state)
{
	int stream_id, ret;
	struct soc_dp_dev *dp;
	struct drm_connector *connector =
		drm_atomic_get_new_connector_for_encoder(state, encoder);
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct soc_dp_mst_conn_state *soc_conn_state;
	struct soc_dp_mst_conn *mst_conn;
	struct drm_display_mode *adjusted_mode;
	struct drm_dp_mst_topology_state *mst_state;
	struct drm_dp_mst_atomic_payload *payload;

	if (!connector)
		return;

	mst_conn    = to_soc_dp_mst_conn(connector);
	dp          = container_of(mst_conn->mgr, struct soc_dp_dev, mst_mgr);
	conn_state  = drm_atomic_get_new_connector_state(state, connector);
	soc_conn_state = to_soc_dp_mst_conn_state(conn_state);

	if (!conn_state->crtc)
		return;

	stream_id = soc_dp_get_stream_id(dp, conn_state->crtc);
	if (stream_id < 0)
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (!crtc_state)
		return;

	adjusted_mode = &crtc_state->adjusted_mode;

	if (soc_conn_state->pbn > 0) {
		mst_state = drm_atomic_get_new_mst_topology_state(state, &dp->mst_mgr);
		if (!mst_state)
			return;

		mutex_lock(&dp->mode_lock);

		if (dp->mode != SOC_DP_MODE_MST || !dp->link_initialized) {
			mutex_unlock(&dp->mode_lock);
			return;
		}

		dev_info(dp->dev, "Mode Set %ux%u (PCLK: %u kHz)\n",
				adjusted_mode->hdisplay, adjusted_mode->vdisplay, adjusted_mode->clock);

		list_for_each_entry(payload, &mst_state->payloads, next) {
			if (payload->time_slots > 0) {
				dev_info(dp->dev, "    vcpi:%d slots:%d-%d\n",
					payload->vcpi, payload->vc_start_slot,
					payload->vc_start_slot + payload->time_slots - 1);
			}
		}

		ret = clk_set_rate(dp->pixel_clks[stream_id],
				(unsigned long)adjusted_mode->clock * 1000);
		if (ret) {
			dev_err(dp->dev, "Failed to set pixel_clks[%d] to %lu Hz, err=%d\n",
					stream_id, (unsigned long)adjusted_mode->clock * 1000, ret);
			mutex_unlock(&dp->mode_lock);
			return;
		}

		soc_dp_mst_set_stream_msa(dp, stream_id, adjusted_mode,
			soc_conn_state->color_format,
			dp->active_rate, dp->active_lanes);
		msleep(20);

		soc_dp_mst_stream_enable(dp, stream_id);

		soc_dp_mst_sync_slots(dp, state);
		soc_dp_mst_trigger_act(dp);

		mutex_unlock(&dp->mode_lock);

		if (drm_dp_check_act_status(&dp->mst_mgr) < 0)
			dev_err(dp->dev, "MST payload ACT timeout!\n");

		drm_dp_add_payload_part2(&dp->mst_mgr,
			drm_atomic_get_mst_payload_state(mst_state, mst_conn->port));
		msleep(20);
	}
}

static void soc_dp_mst_encoder_atomic_disable(struct drm_encoder *encoder,
		struct drm_atomic_state *state)
{
	int i, stream_id;
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state;
	struct soc_dp_mst_conn *mst_conn;
	struct soc_dp_dev *dp;
	struct drm_connector *saved_conn = NULL;
	struct drm_connector_state *saved_conn_state = NULL;
	struct drm_connector *iter;
	struct drm_connector_state *iter_state;
	struct drm_dp_mst_topology_state *mst_state;
	struct drm_dp_mst_atomic_payload *payload;

	connector = drm_atomic_get_old_connector_for_encoder(state, encoder);
	if (!connector) {
		for_each_old_connector_in_state(state, iter, iter_state, i) {
			if (iter_state->best_encoder == encoder) {
				saved_conn = iter;
				saved_conn_state = iter_state;
				break;
			}
		}
		connector = saved_conn;
	}

	if (connector) {
		mst_conn = to_soc_dp_mst_conn(connector);
		dp = container_of(mst_conn->mgr, struct soc_dp_dev, mst_mgr);

		mutex_lock(&dp->mode_lock);

		if (saved_conn_state)
			old_conn_state = saved_conn_state;
		else
			old_conn_state = drm_atomic_get_old_connector_state(
				state, connector);

		stream_id = soc_dp_get_stream_id(dp, old_conn_state->crtc);

		mst_state = drm_atomic_get_new_mst_topology_state(state, &dp->mst_mgr);
		if (!mst_state) {
			mutex_unlock(&dp->mode_lock);
			return;
		}
		payload = drm_atomic_get_mst_payload_state(mst_state, mst_conn->port);

		mutex_unlock(&dp->mode_lock);
		drm_dp_remove_payload_part1(&dp->mst_mgr, mst_state, payload);
		mutex_lock(&dp->mode_lock);

		if (dp->mode != SOC_DP_MODE_MST) {
			mutex_unlock(&dp->mode_lock);
			return;
		}

		soc_dp_mst_sync_slots(dp, state);
		soc_dp_mst_trigger_act(dp);

		mutex_unlock(&dp->mode_lock);

		if (drm_dp_check_act_status(&dp->mst_mgr) < 0)
			dev_err(dp->dev, "MST payload teardown ACT timeout!\n");

		drm_dp_remove_payload_part2(&dp->mst_mgr, mst_state,
			drm_atomic_get_mst_payload_state(mst_state, mst_conn->port), payload);

		mutex_lock(&dp->mode_lock);
		soc_dp_mst_stream_disable(dp, stream_id);
		mutex_unlock(&dp->mode_lock);

		msleep(20);
	}
}

static int soc_dp_mst_encoder_atomic_check(struct drm_encoder *encoder,
		struct drm_crtc_state *crtc_state, struct drm_connector_state *conn_state)
{
	int stream_id;
	int slots = 0;
	uint32_t bpc, color_format, bpp;
	struct soc_dp_dev *dp;
	struct drm_dp_mst_topology_state *mst_state;

	struct drm_atomic_state *state = crtc_state->state;
	struct drm_connector *connector = conn_state->connector;
	struct soc_dp_mst_conn *mst_conn = to_soc_dp_mst_conn(connector);
	struct soc_dp_mst_conn_state *soc_new_state =
		to_soc_dp_mst_conn_state(conn_state);

	dp = container_of(mst_conn->mgr, struct soc_dp_dev, mst_mgr);

	if (!crtc_state->enable) {
		soc_new_state->pbn = 0;
		return 0;
	}

	if (!drm_atomic_crtc_needs_modeset(crtc_state))
		return 0;

	stream_id = soc_dp_get_stream_id(dp, conn_state->crtc);
	if (stream_id < 0 || stream_id >= dp->max_mst_streams) {
		dev_err(connector->dev->dev, "Invalid stream_id for CRTC\n");
		return -EINVAL;
	}

	soc_new_state->pbn = 0;

	mutex_lock(&dp->mode_lock);
	if (soc_dp_compute_color_spec(dp,
		&crtc_state->adjusted_mode, conn_state, &bpc, &color_format)) {
		mutex_unlock(&dp->mode_lock);
		dev_err(connector->dev->dev, "No valid color spec for mode %s\n",
			crtc_state->adjusted_mode.name);
		return -EINVAL;
	}
	mutex_unlock(&dp->mode_lock);

	bpp = soc_dp_get_bpp(color_format);

	soc_new_state->color_format = color_format;
	soc_new_state->bpp = bpp;

	soc_new_state->pbn = drm_dp_calc_pbn_mode(
		crtc_state->adjusted_mode.clock, bpp << 4);

	mst_state = drm_atomic_get_mst_topology_state(state, &dp->mst_mgr);
	if (!IS_ERR(mst_state) && !mst_state->pbn_div.full)
		mst_state->pbn_div = drm_dp_get_vc_payload_bw(&dp->mst_mgr,
			dp->active_rate / 10, dp->active_lanes);

	slots = drm_dp_atomic_find_time_slots(state,
			mst_conn->mgr, mst_conn->port,
			soc_new_state->pbn);

	if (slots < 0) {
		dev_err(connector->dev->dev, "Failed to find VCPI slots for %s, err=%d\n",
				connector->name, slots);
		return slots;
	}

	return drm_dp_mst_atomic_check(state);
}

static const struct drm_encoder_helper_funcs soc_dp_mst_encoder_helper_funcs = {
	.mode_valid     = soc_dp_mst_encoder_mode_valid,
	.atomic_check   = soc_dp_mst_encoder_atomic_check,
	.atomic_enable  = soc_dp_mst_encoder_atomic_enable,
	.atomic_disable = soc_dp_mst_encoder_atomic_disable,
};

int soc_dp_mst_create(struct soc_dp_dev *dp)
{
	/*
	 * Reset CRTC→stream mapping, init MST topology manager with DPCD
	 * helpers, then create per-CRTC DPMST encoders with identical
	 * possible_crtcs mask derived from SST encoder.
	 */
	int i, ret;

	memset(dp->crtc_stream_map, -1, sizeof(dp->crtc_stream_map));
	dp->mst_mgr.cbs = &soc_dp_mst_topology_cbs;

	ret = drm_dp_mst_topology_mgr_init(&dp->mst_mgr, dp->drm,
			&dp->aux, SOC_DP_MAX_DPCD_TRANSACTION_BYTES,
			SOC_MAX_CRTC_NUM, dp->connector.base.id);
	if (ret) {
		dev_err(dp->dev, "MST mgr init failed, err=%d\n", ret);
		return ret;
	}

	for (i = 0; i < SOC_MAX_CRTC_NUM; i++) {
		dp->mst_encoders[i].dp = dp;
		drm_encoder_init(dp->drm, &dp->mst_encoders[i].base,
				&soc_dp_mst_encoder_funcs, DRM_MODE_ENCODER_DPMST, NULL);
		drm_encoder_helper_add(&dp->mst_encoders[i].base,
				&soc_dp_mst_encoder_helper_funcs);

		dp->mst_encoders[i].base.possible_crtcs = dp->encoder.possible_crtcs;
	}

	return 0;
}

void soc_dp_mst_destroy(struct soc_dp_dev *dp)
{
	/* Destroy topology manager, then cleanup all MST encoders */
	int i;

	drm_dp_mst_topology_mgr_destroy(&dp->mst_mgr);

	for (i = 0; i < SOC_MAX_CRTC_NUM; i++)
		drm_encoder_cleanup(&dp->mst_encoders[i].base);
}

int soc_dp_mst_init(struct soc_dp_dev *dp)
{
	int i, ret;
	uint32_t num_streams = 0;

	ret = clk_set_rate(dp->pixel_clks[0], 25175 * 1000);
	if (ret) {
		dev_err(dp->dev, "Failed to set pixel clock for MST init, err=%d\n", ret);
		return ret;
	}

	dp->active_pixel_clk_khz = 0;
	dp->active_bpp = 8;

	ret = soc_dp_establish_link(dp);
	if (ret) {
		dev_err(dp->dev, "MST link training failed during init, err=%d\n", ret);
		return ret;
	}

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_reg_read_range(dp, SOC_DPTX_NUM_STREAMS, &num_streams);
#else
	num_streams = SOC_MAX_CRTC_NUM;
#endif
	if (num_streams == 0 || num_streams > SOC_MAX_CRTC_NUM) {
		dev_warn(dp->dev, "Invalid HW max streams (%u), fallback to %u\n",
				num_streams, SOC_MAX_CRTC_NUM);
		dp->max_mst_streams = SOC_MAX_CRTC_NUM;
	} else {
		dp->max_mst_streams = num_streams;
	}
	dev_info(dp->dev, "MST HW max streams %u\n", dp->max_mst_streams);

	for (i = 1; i < dp->max_mst_streams; i++) {
		char name[16];

		snprintf(name, sizeof(name), "pixel-%d", i);
		dp->pixel_clks[i] = clk_get(dp->dev, name);
		if (IS_ERR(dp->pixel_clks[i])) {
			ret = PTR_ERR(dp->pixel_clks[i]);
			dp->pixel_clks[i] = NULL;
			dev_err(dp->dev, "Failed to get %s clock: %d\n", name, ret);
			goto err_clocks;
		}
	}

	soc_dp_mst_set_mode(dp, true);

	ret = drm_dp_mst_topology_mgr_set_mst(&dp->mst_mgr, true);
	if (ret) {
		soc_dp_mst_set_mode(dp, false);
		dev_err(dp->dev, "Failed to set MST topology manager, err=%d\n", ret);
		goto err_clocks;
	}

	return 0;

err_clocks:
	while (--i >= 1) {
		clk_put(dp->pixel_clks[i]);
		dp->pixel_clks[i] = NULL;
	}

	phy_power_off(dp->phy);
	return ret;
}

void soc_dp_mst_fini(struct soc_dp_dev *dp)
{
	int i;

	drm_dp_mst_topology_mgr_set_mst(&dp->mst_mgr, false);
	soc_dp_mst_set_mode(dp, false);

	for (i = 1; i < dp->max_mst_streams; i++) {
		if (dp->pixel_clks[i]) {
			clk_put(dp->pixel_clks[i]);
			dp->pixel_clks[i] = NULL;
		}
	}

	phy_power_off(dp->phy);
}

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
int soc_dp_mst_default_topology_build(struct soc_dp_virtual_rx *rx)
{
	int ret;
	struct soc_dp_mst_dev *root;
	struct soc_dp_mst_dev *hubDeep1, *hubDeep2, *hubDeep3, *hubDeep4, *hubDeep5;
	struct soc_dp_mst_dev *det_ep, *det_hub, *test_plug_ep;

	root = soc_dp_mst_branch_create(NULL, 0, 4, "root-hub", 4, 54);
	if (!root)
		return -ENOMEM;

	hubDeep1 = soc_dp_mst_branch_create(root, 1, 1, "hub-Deep1", 4, 54);
	if (hubDeep1) {
		hubDeep2 = soc_dp_mst_branch_create(hubDeep1, 1, 1, "hub-Deep2", 4, 54);
		if (hubDeep2) {
			hubDeep3 = soc_dp_mst_branch_create(hubDeep2, 1, 1, "hub-Deep3", 4, 54);
			if (hubDeep3) {
				hubDeep4 = soc_dp_mst_branch_create(hubDeep3, 1, 1, "hub-Deep4", 4, 54);
				if (hubDeep4) {
					hubDeep5 = soc_dp_mst_branch_create(hubDeep4, 1, 2, "hub-Deep5", 4, 54);
					if (hubDeep5) {
						soc_dp_mst_endpoint_create(hubDeep5, 1, "ep-DeepA", NULL, 0, 4, 54);
						soc_dp_mst_endpoint_create(hubDeep5, 2, "ep-DeepB", NULL, 0, 4, 54);
					}
				}
			}
		}
	}

	soc_dp_mst_endpoint_create(root, 2, "ep-Direct", NULL, 0, 4, 81);

	ret = soc_dp_mst_add_subtree(rx, root, 0);
	if (ret < 0)
		return ret;

	det_ep = soc_dp_mst_endpoint_create(NULL, 0, "ep-detached-4K", NULL, 0, 4, 54);
	if (det_ep)
		soc_dp_mst_add_subtree(rx, det_ep, 0);

	det_hub = soc_dp_mst_branch_create(NULL, 0, 2, "hub-detached-mini", 4, 54);
	if (det_hub) {
		soc_dp_mst_endpoint_create(det_hub, 1, "ep-det-1080p", NULL, 0, 2, 27);
		soc_dp_mst_add_subtree(rx, det_hub, 0);
	}

	test_plug_ep = soc_dp_mst_endpoint_create(NULL, 0, "ep-auto-plugged", NULL, 0, 2, 27);
	if (test_plug_ep)
		soc_dp_mst_add_subtree(rx, test_plug_ep, root->ports[3].port_id);

	return 0;
}
#endif
