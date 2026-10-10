#ifndef __SOC_DP_DRI_H__
#define __SOC_DP_DRI_H__

#include <linux/io.h>

#ifdef CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED
#include <linux/workqueue.h>
#endif

#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dp_mst_helper.h>

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
#include "soc_dp_virtual_rx.h"
#endif

#define SOC_MAX_CRTC_NUM 4

enum soc_dp_mode {
	SOC_DP_MODE_NONE = 0,
	SOC_DP_MODE_SST,
	SOC_DP_MODE_MST,
};

enum soc_video_format {
	SOC_VIDEO_RGB_6BIT = 0,
	SOC_VIDEO_RGB_8BIT = 1,
	SOC_VIDEO_RGB_10BIT = 2,
	SOC_VIDEO_RGB_12BIT = 3,
	SOC_VIDEO_RGB_16BIT = 4,
	SOC_VIDEO_YUV444_8BIT = 5,
	SOC_VIDEO_YUV444_10BIT = 6,
	SOC_VIDEO_YUV444_12BIT = 7,
	SOC_VIDEO_YUV444_16BIT = 8,
	SOC_VIDEO_YUV422_8BIT = 9,
	SOC_VIDEO_YUV422_10BIT = 10,
	SOC_VIDEO_YUV422_12BIT = 11,
	SOC_VIDEO_YUV422_16BIT = 12,
};

enum soc_dp_link_rate {
	SOC_DP_LINK_RATE_1_62 = 1620000,
	SOC_DP_LINK_RATE_2_70 = 2700000,
	SOC_DP_LINK_RATE_5_40 = 5400000,
	SOC_DP_LINK_RATE_8_10 = 8100000,
};

enum soc_dp_lane_count {
	SOC_DP_LANE_1 = 1,
	SOC_DP_LANE_2 = 2,
	SOC_DP_LANE_4 = 4,
};

/*
 * TSI: one entry of the link-training priority table (soc_dp_dri.c).
 * The struct is named here so the KUnit table-invariant tests can see
 * the entries; the table itself is only exposed in KUnit builds.
 */
struct soc_dp_link_config {
	enum soc_dp_link_rate rate;
	enum soc_dp_lane_count lanes;
};

#if IS_ENABLED(CONFIG_KUNIT)
extern const struct soc_dp_link_config soc_dp_link_priority_table[];
extern const unsigned int soc_dp_link_priority_table_len;
#endif

struct soc_dp_mst_encoder {
	struct drm_encoder base;
	struct soc_dp_dev *dp;
};

struct soc_dp_dev {
	struct device *dev;

	struct drm_device *drm;
	struct drm_encoder encoder;
	struct drm_connector connector;

	struct proc_dir_entry *proc_irq;
	enum drm_connector_status connector_status;

	struct phy *phy;
	struct clk *pixel_clks[SOC_MAX_CRTC_NUM];

	/* Hardware registers */
	void __iomem *regs;
	struct mutex mode_lock;

	/* Link state */
	bool link_initialized;
	uint8_t dpcd[DP_RECEIVER_CAP_SIZE];

	struct {
		uint8_t revision;
		uint8_t enhanced_framing;
		uint32_t max_rate;
		uint32_t max_num_lanes;
	} link;

	/* Cache for active link and clock context used during retraining. */
	uint32_t active_rate;
	uint32_t active_lanes;
	uint32_t active_pixel_clk_khz;
	uint32_t active_bpp;
	bool sink_irq_pending;

	/* HPD handling */
#ifdef CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED
	struct delayed_work hpd_work;
#else
	int irq;
#endif

	struct drm_dp_aux aux;

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	struct soc_dp_virtual_rx rx;
#endif

	enum soc_dp_mode mode;
	bool only_sst;

	uint8_t max_mst_streams;
	int8_t crtc_stream_map[32];
	struct drm_dp_mst_topology_mgr mst_mgr;
	struct soc_dp_mst_encoder mst_encoders[SOC_MAX_CRTC_NUM];
};

static inline struct soc_dp_dev *encoder_to_dp(struct drm_encoder *encoder)
{
	return container_of(encoder, struct soc_dp_dev, encoder);
}

static inline struct soc_dp_dev *mst_encoder_to_dp(struct drm_encoder *encoder)
{
	struct soc_dp_mst_encoder *mst_enc =
		container_of(encoder, struct soc_dp_mst_encoder, base);
	return mst_enc->dp;
}

static inline struct soc_dp_dev *aux_to_dp(struct drm_dp_aux *aux)
{
	return container_of(aux, struct soc_dp_dev, aux);
}

static inline struct soc_dp_dev *connector_to_dp(struct drm_connector *connector)
{
	return container_of(connector, struct soc_dp_dev, connector);
}

/**
 * soc_dp_establish_link - Iterate link config priority table and train link
 * @dp: DP device structure
 *
 * Selects the highest-priority link configuration (rate + lanes) that
 * meets the required bandwidth and is within sink capabilities, then
 * performs link training. On success caches active rate/lanes in @dp.
 *
 * Return: 0 on success, negative errno on failure
 */
int soc_dp_establish_link(struct soc_dp_dev *dp);

/**
 * soc_dp_get_bpp - Look up bits-per-pixel for a color format
 * @format: SOC video format index (enum soc_video_format)
 *
 * Return: bpp value, or 24 (RGB888 default) for out-of-range index
 */
int soc_dp_get_bpp(uint32_t format);

/**
 * soc_dp_compute_color_spec - Determine optimal color format and bpc
 * @dp: DP device structure for link capability info
 * @mode: Target display mode
 * @conn_state: Connector state carrying display_info and user bpc preference
 * @out_bpc: Output parameter for selected bits-per-color
 * @out_color_format: Output parameter for selected SOC video format
 *
 * Evaluates YUV444, YUV422, and RGB444 formats against sink capabilities
 * and available link bandwidth. Falls back from higher to lower bpc
 * until the required bandwidth fits within the link capacity.
 *
 * Return: 0 on success, -EINVAL if no valid configuration is found
 */
int soc_dp_compute_color_spec(struct soc_dp_dev *dp,
		const struct drm_display_mode *mode,
		struct drm_connector_state *conn_state,
		uint32_t *out_bpc, uint32_t *out_color_format);

/**
 * soc_dp_get_sink_irq - Read the Sink IRQ event status from hardware
 * @dp: DP device structure
 *
 * Return: true if a sink IRQ is pending, false otherwise
 */
bool soc_dp_get_sink_irq(struct soc_dp_dev *dp);

/**
 * soc_dp_clean_sink_irq - Clear the Sink IRQ event status in hardware
 * @dp: DP device structure
 */
void soc_dp_clean_sink_irq(struct soc_dp_dev *dp);

#endif
