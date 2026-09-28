#include <linux/module.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-dp.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>

#include "soc_dp_dri.h"
#include "soc_dp_mst.h"
#include "soc_dp_reg_ops.h"

#define HPD_POLL_INTERVAL_MS 80

#define SOC_DP_SWING_MAX  2
#define SOC_DP_PREEMP_MAX 2

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
#include <linux/proc_fs.h>
#endif

#ifdef CONFIG_SOC_DP_ACTIVATE_DO_DIV
#include <linux/math64.h>
#endif

/* FPGA DEBUG */
static const struct soc_dp_link_config {
	enum soc_dp_link_rate rate;
	enum soc_dp_lane_count lanes;
} soc_dp_link_priority_table[] = {
	/* --- Tier 1: Ultra High Bandwidth (> 17 Gbps) --- */
	// {SOC_DP_LINK_RATE_5_40, SOC_DP_LANE_4}, /* 21.6 Gbps */

	/* --- Tier 2: High Bandwidth (~10 Gbps) --- */
	// {SOC_DP_LINK_RATE_2_70, SOC_DP_LANE_4}, /* 10.8 Gbps */
	// {SOC_DP_LINK_RATE_5_40, SOC_DP_LANE_2}, /* 10.8 Gbps */

	/* --- Tier 3: Medium Bandwidth (~5-6 Gbps) --- */
	// {SOC_DP_LINK_RATE_1_62, SOC_DP_LANE_4}, /* 6.48 Gbps */
	// {SOC_DP_LINK_RATE_2_70, SOC_DP_LANE_2}, /* 5.40 Gbps */
	// {SOC_DP_LINK_RATE_5_40, SOC_DP_LANE_1}, /* 5.40 Gbps */

	/* --- Tier 4: Low Bandwidth (< 4 Gbps) --- */
	{SOC_DP_LINK_RATE_1_62, SOC_DP_LANE_2}, /* 3.24 Gbps */
	// {SOC_DP_LINK_RATE_2_70, SOC_DP_LANE_1}, /* 2.70 Gbps */
	{SOC_DP_LINK_RATE_1_62, SOC_DP_LANE_1}, /* 1.62 Gbps */
};

static const struct soc_format_info {
	uint8_t bpp; /* Bits Per Pixel */
} format_info_table[] = {
	[SOC_VIDEO_RGB_6BIT]      = { .bpp = 18 },
	[SOC_VIDEO_RGB_8BIT]      = { .bpp = 24 },
	[SOC_VIDEO_RGB_10BIT]     = { .bpp = 30 },
	[SOC_VIDEO_RGB_12BIT]     = { .bpp = 36 },
	[SOC_VIDEO_RGB_16BIT]     = { .bpp = 48 },

	[SOC_VIDEO_YUV444_8BIT]   = { .bpp = 24 },
	[SOC_VIDEO_YUV444_10BIT]  = { .bpp = 30 },
	[SOC_VIDEO_YUV444_12BIT]  = { .bpp = 36 },
	[SOC_VIDEO_YUV444_16BIT]  = { .bpp = 48 },

	[SOC_VIDEO_YUV422_8BIT]   = { .bpp = 16 },
	[SOC_VIDEO_YUV422_10BIT]  = { .bpp = 20 },
	[SOC_VIDEO_YUV422_12BIT]  = { .bpp = 24 },
	[SOC_VIDEO_YUV422_16BIT]  = { .bpp = 32 },
};

int soc_dp_get_bpp(uint32_t format)
{
	/* Out-of-range guard: return RGB888 bpp as safe default */
	if (format >= ARRAY_SIZE(format_info_table)) {
		pr_warn("Invalid color format index %u, defaulting to RGB888\n", format);
		return 24;
	}

	return format_info_table[format].bpp;
}

static enum soc_video_format soc_dp_get_video_format(uint32_t drm_color_format, uint32_t bpc)
{
	switch (drm_color_format) {
	case DRM_COLOR_FORMAT_YCBCR422:
		if (bpc >= 16)
			return SOC_VIDEO_YUV422_16BIT;
		if (bpc >= 12)
			return SOC_VIDEO_YUV422_12BIT;
		if (bpc >= 10)
			return SOC_VIDEO_YUV422_10BIT;
		return SOC_VIDEO_YUV422_8BIT;

	case DRM_COLOR_FORMAT_YCBCR444:
		if (bpc >= 16)
			return SOC_VIDEO_YUV444_16BIT;
		if (bpc >= 12)
			return SOC_VIDEO_YUV444_12BIT;
		if (bpc >= 10)
			return SOC_VIDEO_YUV444_10BIT;
		return SOC_VIDEO_YUV444_8BIT;

	case DRM_COLOR_FORMAT_RGB444:
	default:
		if (bpc >= 16)
			return SOC_VIDEO_RGB_16BIT;
		if (bpc >= 12)
			return SOC_VIDEO_RGB_12BIT;
		if (bpc >= 10)
			return SOC_VIDEO_RGB_10BIT;
		if (bpc == 6)
			return SOC_VIDEO_RGB_6BIT;
		return SOC_VIDEO_RGB_8BIT;
	}

	return SOC_VIDEO_RGB_8BIT;
}

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
static ssize_t soc_dp_hw_aux_transfer(struct drm_dp_aux *aux,
		struct drm_dp_aux_msg *msg)
{
	int ret, i;
	unsigned long timeout;
	struct soc_dp_dev *dp = container_of(aux, struct soc_dp_dev, aux);
	uint32_t val, status, err_code;
	uint32_t data[4] = {0};
	uint8_t *buf = msg->buffer;

	/* Determine transfer direction (I2C Read: 0x1, Native Read: 0x9) */
	bool is_read = (msg->request & DP_AUX_I2C_READ) ||
		((msg->request & DP_AUX_NATIVE_READ) == DP_AUX_NATIVE_READ);

	/* 1. Reject transfers if the cable is disconnected */
	if (dp->connector_status != connector_status_connected)
		return -EIO;

	/* 2. Validate payload size against standard DP AUX maximum */
	if (msg->size > 16)
		return -EINVAL;

	ret = phy_power_on(dp->phy);
	if (ret)
		return ret;

	/* 3. Pack byte buffer into 32-bit little-endian hardware registers */
	if (!is_read && msg->size > 0) {
		for (i = 0; i < msg->size; i++)
			data[i / 4] |= buf[i] << ((i % 4) * 8);

		soc_dp_reg_write_range(dp, SOC_DPTX_AUX_DATA1, data[0]);
		soc_dp_reg_write_range(dp, SOC_DPTX_AUX_DATA2, data[1]);
		soc_dp_reg_write_range(dp, SOC_DPTX_AUX_DATA3, data[2]);
		soc_dp_reg_write_range(dp, SOC_DPTX_AUX_DATA4, data[3]);
	}

	/*
	 * 4. Configure AUX transaction address and command type
	 * Note: Linux DRM request flags exactly match the hardware's 4-bit command
	 * definition (including MOT bit), allowing direct register assignment.
	 */
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_ADDR, msg->address);
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_CMD_TYPE, msg->request & 0xF);

	/* 5. Configure transaction length and I2C primitive flags */
	if (msg->size == 0) {
		/* Zero-sized payload triggers I2C bare address packet (START/STOP) */
		soc_dp_reg_write_range(dp, SOC_DPTX_I2C_ADDR_ONLY, 1);
		soc_dp_reg_write_range(dp, SOC_DPTX_AUX_LENGTH, 0);
	} else {
		/* Standard payload transaction (hardware expects length minus 1) */
		soc_dp_reg_write_range(dp, SOC_DPTX_I2C_ADDR_ONLY, 0);
		soc_dp_reg_write_range(dp, SOC_DPTX_AUX_LENGTH, msg->size - 1);
	}

	/* 6. Trigger hardware transaction execution */
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_START, 0);
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_START, 1);

	/* 7. Wait for hardware completion or timeout */
	timeout = jiffies + msecs_to_jiffies(200);
	ret = -ETIMEDOUT;

	while (1) {
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_REPLY_EVENT_INT_STA, &val);
		if (val) {
			ret = 0;
			break;
		}

		/* Hardware timeout bit acts as a fail-safe mechanism */
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_TIMEOUT, &val);
		if (val || time_after(jiffies, timeout))
			break;

		usleep_range(100, 110);
	}

	/* Clear interrupt status (Write 1 to clear) */
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_REPLY_EVENT_INT_STA, 1);

	if (ret) {
		phy_power_off(dp->phy);
		dev_err(dp->dev, "AUX transfer timeout\n");
		return ret;
	}

	/* 8. Parse AUX response status (ACK/NACK/DEFER) */
	soc_dp_reg_read_range(dp, SOC_DPTX_AUX_STATUS, &status);
	msg->reply = status;

	switch (status) {
	case DP_AUX_NATIVE_REPLY_ACK:
		break;
	case DP_AUX_NATIVE_REPLY_NACK:
		/* Return 0 bytes transferred on NACK to trigger upper-layer retry */
		phy_power_off(dp->phy);
		return 0;
	case DP_AUX_NATIVE_REPLY_DEFER:
		phy_power_off(dp->phy);
		return -EBUSY;
	default:
		/* Evaluate hardware protocol errors (Fail-fast before status check) */
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_REPLY_ERR, &val);
		if (val) {
			soc_dp_reg_read_range(dp, SOC_DPTX_AUX_REPLY_ERR_CODE, &err_code);
			dev_err(dp->dev, "AUX physical/protocol error, code: 0x%x\n", err_code);
		}
		phy_power_off(dp->phy);
		return -EIO;
	}

	/* 9. Unpack 32-bit hardware registers into byte buffer for read requests */
	if (is_read && msg->size > 0) {
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_DATA1, &data[0]);
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_DATA2, &data[1]);
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_DATA3, &data[2]);
		soc_dp_reg_read_range(dp, SOC_DPTX_AUX_DATA4, &data[3]);

		for (i = 0; i < msg->size; i++)
			buf[i] = (data[i / 4] >> ((i % 4) * 8)) & 0xFF;
	}

	phy_power_off(dp->phy);
	return msg->size;
}
#endif

static ssize_t soc_dp_aux_transfer(struct drm_dp_aux *aux,
		struct drm_dp_aux_msg *msg)
{
#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	return soc_dp_virtual_rx_aux_transfer(aux, msg);
#else
	return soc_dp_hw_aux_transfer(aux, msg);
#endif
}

/*
 * Read the Sink's DPCD capability information.
 * Note: EDID is parsed separately. This function focuses solely on
 * Link Layer capabilities (Rate, Lanes, etc.).
 */
static int soc_dp_read_sink_caps(struct soc_dp_dev *dp)
{
	ssize_t ret;
	uint8_t max_bw;

	ret = phy_power_on(dp->phy);
	if (ret)
		return ret;

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	dp->dpcd[DP_DPCD_REV]               = 0x14; /* DP 1.4 */
	dp->dpcd[DP_MAX_LINK_RATE]          = DP_LINK_BW_5_4;
	dp->dpcd[DP_MAX_LANE_COUNT]         = 4 | DP_TPS3_SUPPORTED | DP_ENHANCED_FRAME_CAP;
	dp->dpcd[DP_MAX_DOWNSPREAD]         = DP_MAX_DOWNSPREAD_0_5;
	dp->dpcd[DP_NORP]                   = 0x01;
	dp->dpcd[DP_DOWNSTREAMPORT_PRESENT] = 0x00;
	dp->dpcd[DP_MAIN_LINK_CHANNEL_CODING] = DP_CAP_ANSI_8B10B;
	dp->dpcd[DP_DOWN_STREAM_PORT_COUNT] = 0x00;
	dp->dpcd[DP_RECEIVE_PORT_0_CAP_0]   = DP_LOCAL_EDID_PRESENT;
#else
	/* 1. Read DPCD Receiver Capability fields (0x00000 - 0x0000F) */
	ret = drm_dp_dpcd_read(&dp->aux, DP_DPCD_REV, dp->dpcd, DP_RECEIVER_CAP_SIZE);
	if (ret != DP_RECEIVER_CAP_SIZE) {
		phy_power_off(dp->phy);
		dev_err(dp->dev, "Failed to read DPCD, err=%zd\n", ret);
		return ret;
	}
#endif

	/* 2. Parse DP Revision */
	dp->link.revision = dp->dpcd[DP_DPCD_REV];

	/*
	 * 3. Parse and determine Link Rate. Get the maximum link rate
	 * supported by the Sink. Note: During link training, we usually
	 * start from min(Sink_Max, Source_Max).
	 */
	max_bw = dp->dpcd[DP_MAX_LINK_RATE];
	switch (max_bw) {
	case DP_LINK_BW_1_62:
		dp->link.max_rate = SOC_DP_LINK_RATE_1_62;
		break;
	case DP_LINK_BW_2_7:
		dp->link.max_rate = SOC_DP_LINK_RATE_2_70;
		break;
	case DP_LINK_BW_5_4:
		dp->link.max_rate = SOC_DP_LINK_RATE_5_40;
		break;
	case DP_LINK_BW_8_1:
		dp->link.max_rate = SOC_DP_LINK_RATE_8_10;
		break;
	default:
		dev_warn(dp->dev, "Unknown DPCD Max Rate: 0x%x, defaulting to 1.62G\n", max_bw);
		dp->link.max_rate = SOC_DP_LINK_RATE_1_62;
		break;
	}

	/* 4. Parse and determine Lane Count */
	dp->link.max_num_lanes = dp->dpcd[DP_MAX_LANE_COUNT] & DP_MAX_LANE_COUNT_MASK;

	/* 5. Check for Enhanced Framing support */
	dp->link.enhanced_framing =
		(dp->dpcd[DP_MAX_LANE_COUNT] & DP_ENHANCED_FRAME_CAP);

	phy_power_off(dp->phy);

	dev_info(dp->dev, "DPCD: Rev %x.%x, MaxRate %u kHz, MaxLanes %u, EnhFrame %u\n",
			dp->link.revision >> 4, dp->link.revision & 0xF,
			dp->link.max_rate, dp->link.max_num_lanes, dp->link.enhanced_framing);

	return 0;
}

/*
 * Check Hot Plug Detect (HPD) Status
 */
static enum drm_connector_status soc_dp_detect_hpd(struct soc_dp_dev *dp)
{
#if defined(CONFIG_SOC_DP_DRIVER_QEMU) || defined(CONFIG_SOC_DP_HPD_BYPASS)
	return dp->connector_status;
#else
	uint32_t plug_event, unplug_event;
	uint32_t hpd_real_level;

	enum drm_connector_status connector_status = dp->connector_status;

	soc_dp_reg_read_range(dp, SOC_DPTX_HOT_PLUG_EVENT, &plug_event);
	soc_dp_reg_read_range(dp, SOC_DPTX_HOT_UNPLUG_EVENT, &unplug_event);
	soc_dp_reg_read_range(dp, SOC_DPTX_HPD_IN_STATUS, &hpd_real_level);

	if (plug_event && !unplug_event) {
		connector_status = connector_status_connected;
	} else if (unplug_event && !plug_event) {
		connector_status = connector_status_disconnected;
	} else {
		if (hpd_real_level)
			connector_status = connector_status_connected;
		else
			connector_status = connector_status_disconnected;
	}

	return connector_status;
#endif
}

/*
 * Clean Hot Plug Detect (HPD) Status
 */
static void soc_dp_clean_hpd(struct soc_dp_dev *dp)
{
#if defined(CONFIG_SOC_DP_DRIVER_QEMU) || defined(CONFIG_SOC_DP_HPD_BYPASS)
	return;
#else
	uint32_t plug_event, unplug_event;

	soc_dp_reg_read_range(dp, SOC_DPTX_HOT_PLUG_EVENT, &plug_event);
	soc_dp_reg_read_range(dp, SOC_DPTX_HOT_UNPLUG_EVENT, &unplug_event);

	if (plug_event)
		soc_dp_reg_only_write_range(dp, SOC_DPTX_HOT_PLUG_EVENT, 0x1);

	if (unplug_event)
		soc_dp_reg_only_write_range(dp, SOC_DPTX_HOT_UNPLUG_EVENT, 0x1);
#endif
}

/*
 * Read the Sink IRQ event status from hardware.
 * Returns true if a sink IRQ is pending, false otherwise.
 */
bool soc_dp_get_sink_irq(struct soc_dp_dev *dp)
{
#if   defined(CONFIG_SOC_DP_DRIVER_QEMU)
	return false;
#elif defined(CONFIG_SOC_DP_HPD_BYPASS)
	return true;
#else
	uint32_t sink_irq = 0;

	soc_dp_reg_read_range(dp, SOC_DPTX_SINK_IRQ_EVENT, &sink_irq);
	return sink_irq != 0;
#endif
}

/*
 * Clear the Sink IRQ event status in hardware.
 */
void soc_dp_clean_sink_irq(struct soc_dp_dev *dp)
{
#if defined(CONFIG_SOC_DP_DRIVER_QEMU) || defined(CONFIG_SOC_DP_HPD_BYPASS)
	return;
#else
	uint32_t sink_irq = 0;

	soc_dp_reg_read_range(dp, SOC_DPTX_SINK_IRQ_EVENT, &sink_irq);
	if (sink_irq)
		soc_dp_reg_only_write_range(dp, SOC_DPTX_SINK_IRQ_EVENT, 0x1);
#endif
}

static int soc_dp_set_training_pattern(struct soc_dp_dev *dp, uint8_t pattern)
{
	int ret;
	uint32_t tps_sel = 0;
	uint8_t dpcd_pattern = pattern;

	if (pattern != DP_TRAINING_PATTERN_DISABLE)
		dpcd_pattern |= DP_LINK_SCRAMBLING_DISABLE;

	/* Configure PHY Pattern */
	switch (pattern) {
	case DP_TRAINING_PATTERN_DISABLE:
		tps_sel = 0;
		soc_dp_reg_write_range(dp, SOC_DPTX_SCRAMBLER_DISABLE, 0);
		break;
	case DP_TRAINING_PATTERN_1:
		tps_sel = 1;
		soc_dp_reg_write_range(dp, SOC_DPTX_SCRAMBLER_DISABLE, 1);
		break;
	case DP_TRAINING_PATTERN_2:
		tps_sel = 2;
		soc_dp_reg_write_range(dp, SOC_DPTX_SCRAMBLER_DISABLE, 1);
		break;
	case DP_TRAINING_PATTERN_3:
		tps_sel = 3;
		soc_dp_reg_write_range(dp, SOC_DPTX_SCRAMBLER_DISABLE, 1);
		break;
	default:
		dev_err(dp->dev, "Unsupported training pattern: 0x%x\n", pattern);
		return -EINVAL;
	}

	soc_dp_reg_write_range(dp, SOC_DPTX_TPS_SEL, tps_sel);

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	/* Configure DPCD Pattern */
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET, dpcd_pattern);
	if (ret < 0) {
		dev_err(dp->dev, "Failed to set DPCD training pattern, err=%d\n", ret);
		return ret;
	}
#else
	ret = 0;
	return ret;
#endif

	return 0;
}

static int soc_dp_link_train_clock_recovery(struct soc_dp_dev *dp, enum soc_dp_link_rate rate, enum soc_dp_lane_count lanes)
{
	int i, ret;
	int retries = 0;
	uint8_t link_status[DP_LINK_STATUS_SIZE];
	uint8_t training_set[4] = {0};
	union phy_configure_opts phy_cfg = {0};

	phy_cfg.dp.lanes = lanes;
	phy_cfg.dp.set_voltages = 1;

	phy_configure(dp->phy, &phy_cfg);

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	ret = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET,
			training_set, lanes);
	if (ret < 0)
		return ret;
#endif

	ret = soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_1);
	if (ret < 0) {
		soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
		return ret;
	}

	while (retries < 8) {
#ifndef CONFIG_SOC_DP_DRIVER_QEMU
		drm_dp_link_train_clock_recovery_delay(&dp->aux, dp->dpcd);

		ret = drm_dp_dpcd_read_link_status(&dp->aux, link_status);
		if (ret < 0) {
			soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
			return ret;
		}

		if (drm_dp_clock_recovery_ok(link_status, lanes))
#else
		if (1)
#endif
			return 0;

		/* Update settings based on Sink request */
		for (i = 0; i < lanes; i++) {
			uint8_t v = drm_dp_get_adjust_request_voltage(link_status, i);
			uint8_t p = drm_dp_get_adjust_request_pre_emphasis(link_status, i);

			if (v >= SOC_DP_SWING_MAX) {
				v = SOC_DP_SWING_MAX;
				v |= DP_TRAIN_MAX_SWING_REACHED;
			}

			if (p >= SOC_DP_PREEMP_MAX) {
				p = SOC_DP_PREEMP_MAX;
				v |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;
			}

			training_set[i] = v | (p << DP_TRAIN_PRE_EMPHASIS_SHIFT);

			// Update PHY Config
			phy_cfg.dp.voltage[i] = v & DP_TRAIN_VOLTAGE_SWING_MASK;
			phy_cfg.dp.pre[i] = p & DP_TRAIN_PRE_EMPHASIS_MASK;
		}

		ret = phy_configure(dp->phy, &phy_cfg);
		if (ret)
			return ret;

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
		ret = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET,
				training_set, lanes);
		if (ret < 0) {
			soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
			return ret;
		}
#else
		(void)training_set;
#endif

		retries++;
	}

	dev_err(dp->dev, "Link Training Clock Recovery Failed\n");
	soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
	return -ETIMEDOUT;
}

static int soc_dp_link_train_channel_eq(struct soc_dp_dev *dp, enum soc_dp_link_rate rate, enum soc_dp_lane_count lanes)
{
	int i, ret;
	int retries = 0;
	uint8_t link_status[DP_LINK_STATUS_SIZE];
	uint8_t training_set[4] = {0};
	uint8_t training_pattern = DP_TRAINING_PATTERN_2;
	union phy_configure_opts phy_cfg = {0};

	phy_cfg.dp.lanes = lanes;
	phy_cfg.dp.set_voltages = 1;

	if (dp->dpcd[DP_MAX_LANE_COUNT] & DP_TPS3_SUPPORTED) {
		training_pattern = DP_TRAINING_PATTERN_3;
		dev_info(dp->dev, "Link Training: Using TPS3\n");
	} else {
		dev_info(dp->dev, "Link Training: Using TPS2\n");
	}

	ret = soc_dp_set_training_pattern(dp, training_pattern);
	if (ret < 0) {
		soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
		return ret;
	}

	while (retries < 8) {
#ifndef CONFIG_SOC_DP_DRIVER_QEMU
		drm_dp_link_train_channel_eq_delay(&dp->aux, dp->dpcd);

		ret = drm_dp_dpcd_read_link_status(&dp->aux, link_status);
		if (ret < 0) {
			soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
			return ret;
		}

		if (drm_dp_channel_eq_ok(link_status, lanes)) {
#else
		if (1) {
#endif
			soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
			return 0;
		}

		/* Update settings based on Sink request */
		for (i = 0; i < lanes; i++) {
			uint8_t v = drm_dp_get_adjust_request_voltage(link_status, i);
			uint8_t p = drm_dp_get_adjust_request_pre_emphasis(link_status, i);

			if (v >= SOC_DP_SWING_MAX) {
				v = SOC_DP_SWING_MAX;
				v |= DP_TRAIN_MAX_SWING_REACHED;
			}

			if (p >= SOC_DP_PREEMP_MAX) {
				p = SOC_DP_PREEMP_MAX;
				v |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;
			}

			training_set[i] = v | (p << DP_TRAIN_PRE_EMPHASIS_SHIFT);

			// Update PHY Config
			phy_cfg.dp.voltage[i] = v & DP_TRAIN_VOLTAGE_SWING_MASK;
			phy_cfg.dp.pre[i] = p & DP_TRAIN_PRE_EMPHASIS_MASK;
		}

		ret = phy_configure(dp->phy, &phy_cfg);
		if (ret)
			return ret;

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
		ret = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET,
				training_set, lanes);
		if (ret < 0) {
			soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
			return ret;
		}
#else
		(void)training_set;
#endif

		retries++;
	}

	dev_err(dp->dev, "Link Training Channel EQ Failed\n");
	soc_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
	return -ETIMEDOUT;
}

/*
 * Main Link Training Function
 */
static int soc_dp_link_train(struct soc_dp_dev *dp, enum soc_dp_link_rate rate, enum soc_dp_lane_count lanes)
{
	int ret;
	uint8_t link_config[2];
	uint8_t bw_code;

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	/* Set Power to D0 (Wake up Sink) */
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, DP_SET_POWER_D0);
	if (ret < 0)
		dev_warn(dp->dev, "Failed to power up sink\n");

	msleep(20);

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, DP_SET_POWER_D0);
	if (ret < 0)
		dev_warn(dp->dev, "Failed to power up sink\n");

	msleep(100);

	/* Downspread Control (PHY SSC is disabled, explicitly set to 0) */
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_DOWNSPREAD_CTRL, 0);
	if (ret < 0)
		dev_warn(dp->dev, "Failed to disable downspread\n");

	/* Main Link Channel Coding (ANSI 8B/10B) */
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_MAIN_LINK_CHANNEL_CODING_SET, DP_SET_ANSI_8B10B);
	if (ret < 0)
		dev_warn(dp->dev, "Failed to set channel coding\n");
#endif

	/* Map Link Rate Enum to DPCD Bandwidth Code */
	switch (rate) {
	case SOC_DP_LINK_RATE_1_62:
		bw_code = DP_LINK_BW_1_62;
		break;
	case SOC_DP_LINK_RATE_2_70:
		bw_code = DP_LINK_BW_2_7;
		break;
	case SOC_DP_LINK_RATE_5_40:
		bw_code = DP_LINK_BW_5_4;
		break;
	case SOC_DP_LINK_RATE_8_10:
		bw_code = DP_LINK_BW_8_1;
		break;
	default:
		bw_code = DP_LINK_BW_1_62;
		break;
	}

	/* Configure DPCD Link Rate and Lane Count */
	link_config[0] = bw_code;
	link_config[1] = lanes;
	if (dp->link.enhanced_framing)
		link_config[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	ret = drm_dp_dpcd_write(&dp->aux, DP_LINK_BW_SET, link_config, 2);
	if (ret < 0) {
		dev_err(dp->dev, "Failed to configure DPCD\n");
		return ret;
	}
#endif

	ret = soc_dp_link_train_clock_recovery(dp, rate, lanes);
	if (ret)
		return ret;

	ret = soc_dp_link_train_channel_eq(dp, rate, lanes);
	if (ret)
		return ret;

	msleep(20);
	return 0;
}

static void soc_dp_set_msa(struct soc_dp_dev *dp, const struct drm_display_mode *mode,
		uint32_t color_format, enum soc_dp_link_rate rate, enum soc_dp_lane_count lanes)
{
	soc_dp_mst_set_stream_msa(dp, 0, mode, color_format, rate, lanes);
}

static void soc_dp_enable(struct soc_dp_dev *dp)
{
	soc_dp_mst_stream_enable(dp, 0);
}

static void soc_dp_disable(struct soc_dp_dev *dp)
{
	soc_dp_mst_stream_disable(dp, 0);
}

/* Calculate required bandwidth in kbps (Pixel Clock * Bits Per Pixel) */
static inline uint32_t soc_dp_calc_required_bw(uint32_t pixel_clk_khz, uint32_t bpp)
{
	return pixel_clk_khz * bpp;
}

/* Calculate available link capacity in kbps (taking 8b/10b overhead into account) */
static inline uint32_t soc_dp_calc_link_capacity(enum soc_dp_link_rate rate, enum soc_dp_lane_count lanes)
{
	return (rate * lanes * 8) / 10;
}

static enum drm_connector_status
soc_dp_conn_detect(struct drm_connector *connector, bool force)
{
	struct soc_dp_dev *dp = connector_to_dp(connector);
	enum drm_connector_status status;

	if (dp->mode != SOC_DP_MODE_SST)
		return connector_status_disconnected;

	mutex_lock(&dp->mode_lock);

	status = dp->connector_status;

	if (force && status == connector_status_connected)
		soc_dp_read_sink_caps(dp);

	mutex_unlock(&dp->mode_lock);

	return status;
}

static int soc_dp_conn_detect_ctx(struct drm_connector *connector,
		struct drm_modeset_acquire_ctx *ctx, bool force)
{
	struct soc_dp_dev *dp = connector_to_dp(connector);

	if (dp->mode != SOC_DP_MODE_SST)
		return connector_status_disconnected;

	return dp->connector_status;
}

static int soc_dp_conn_get_modes(struct drm_connector *connector)
{
	int count;
	struct edid *edid;
	struct soc_dp_dev *dp = connector_to_dp(connector);

	if (dp->mode == SOC_DP_MODE_SST) {
		if (phy_power_on(dp->phy))
			return 0;
		edid = drm_get_edid(connector, &dp->aux.ddc);
		phy_power_off(dp->phy);

		drm_connector_update_edid_property(connector, edid);
		count = drm_add_edid_modes(connector, edid);
		kfree(edid);
		return count;
	}

	return 0;
}

static const struct drm_connector_funcs soc_dp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.detect = soc_dp_conn_detect,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs soc_dp_conn_helper_funcs = {
	.detect_ctx	= soc_dp_conn_detect_ctx,
	.get_modes	= soc_dp_conn_get_modes,
};

static int soc_dp_connector_create(struct soc_dp_dev *dp)
{
	int ret;

	ret = drm_connector_init(dp->drm, &dp->connector,
			&soc_dp_connector_funcs, DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		dev_err(dp->dev, "Connector init failed, err=%d\n", ret);
		return ret;
	}

	drm_connector_helper_add(&dp->connector, &soc_dp_conn_helper_funcs);
	drm_atomic_helper_connector_reset(&dp->connector);
	drm_connector_attach_max_bpc_property(&dp->connector, 8, 16);
	drm_connector_attach_encoder(&dp->connector, &dp->encoder);

	return 0;
}

static inline void soc_dp_connector_destroy(struct soc_dp_dev *dp)
{
	drm_connector_cleanup(&dp->connector);
}

static const struct drm_encoder_funcs soc_dp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static enum drm_mode_status soc_dp_encoder_mode_valid(struct drm_encoder *crtc,
		const struct drm_display_mode *mode)
{
	/* FPGA DEBUG */
	// if (mode->hdisplay > 3840 || mode->vdisplay > 2160)
	if (mode->hdisplay > 640 || mode->vdisplay > 480)
		return MODE_BAD_HVALUE;

	return MODE_OK;
}

int soc_dp_compute_color_spec(struct soc_dp_dev *dp,
		const struct drm_display_mode *mode,
		struct drm_connector_state *conn_state,
		uint32_t *out_bpc, uint32_t *out_color_format)
{
	/*
	 * Implementation: prefer RGB444 at highest available bpc.
	 * Step down bpc (10→8→6) until bandwidth fits. If RGB fails,
	 * fall back to YCbCr422 if supported by sink.
	 */
	struct drm_display_info *info = &conn_state->connector->display_info;
	uint32_t bpc, max_allowed_bpc;
	uint32_t supported_formats = info->color_formats;
	uint32_t max_bw, req_bw, bpp;
	uint32_t color_format, target_drm_format;

	/* FPGA DEBUG */
	info->bpc = 8;

	max_allowed_bpc = info->bpc ? info->bpc : 8;
	if (conn_state->max_requested_bpc > 0)
		max_allowed_bpc = min_t(uint32_t, max_allowed_bpc, conn_state->max_requested_bpc);
	else if (conn_state->max_bpc > 0)
		max_allowed_bpc = min_t(uint32_t, max_allowed_bpc, conn_state->max_bpc);

	if (dp->link_initialized)
		max_bw = soc_dp_calc_link_capacity(dp->active_rate, dp->active_lanes);
	else
		max_bw = soc_dp_calc_link_capacity(dp->link.max_rate, dp->link.max_num_lanes);

	bpc = max_allowed_bpc;
	target_drm_format = DRM_COLOR_FORMAT_RGB444;
	while (bpc >= 6) {
		color_format = soc_dp_get_video_format(target_drm_format, bpc);
		bpp = soc_dp_get_bpp(color_format);
		req_bw = mode->clock * bpp;

		if (req_bw <= max_bw)
			goto format_found;

		bpc -= 2;
	}

	if (supported_formats & DRM_COLOR_FORMAT_YCBCR422) {
		bpc = max_allowed_bpc;

		target_drm_format = DRM_COLOR_FORMAT_YCBCR422;

		while (bpc >= 8) {
			color_format = soc_dp_get_video_format(target_drm_format, bpc);
			bpp = soc_dp_get_bpp(color_format);
			req_bw = mode->clock * bpp;

			if (req_bw <= max_bw)
				goto format_found;

			bpc -= 2;
		}
	}

	return -EINVAL;

format_found:
	*out_bpc = bpc;
	*out_color_format = color_format;
	return 0;
}

static int soc_dp_encoder_atomic_check(struct drm_encoder *encoder,
		struct drm_crtc_state *crtc_state, struct drm_connector_state *conn_state)
{
	int ret;
	uint32_t bpc, color_format;
	struct soc_dp_dev *dp = encoder_to_dp(encoder);

	mutex_lock(&dp->mode_lock);

	if (dp->mode == SOC_DP_MODE_MST && crtc_state->active) {
		mutex_unlock(&dp->mode_lock);
		return -EINVAL;
	}

	if (dp->mode == SOC_DP_MODE_SST && crtc_state->active) {
		struct drm_display_mode *mode = &crtc_state->adjusted_mode;

		ret = soc_dp_compute_color_spec(dp, mode, conn_state, &bpc, &color_format);
		if (ret) {
			mutex_unlock(&dp->mode_lock);
			return ret;
		}
	}

	mutex_unlock(&dp->mode_lock);
	return 0;
}

static void soc_dp_encoder_atomic_disable(struct drm_encoder *encoder,
		struct drm_atomic_state *state)
{
	struct soc_dp_dev *dp = encoder_to_dp(encoder);

	mutex_lock(&dp->mode_lock);

	soc_dp_disable(dp);

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	if (dp->connector_status == connector_status_connected) {
		drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, DP_SET_POWER_D3);
		msleep(20);
	}
#endif

	if (dp->link_initialized) {
		phy_power_off(dp->phy);
		dp->link_initialized = false;
	}

	mutex_unlock(&dp->mode_lock);
}

int soc_dp_establish_link(struct soc_dp_dev *dp)
{
	/*
	 * Walk the link-config priority table (highest bandwidth first).
	 * For each config: check sink limits, bandwidth sufficiency,
	 * configure PHY, power on, then run link training.
	 * First successful train wins; cache rate/lanes in @dp.
	 */
	int i, ret;
	const struct soc_dp_link_config *cfg;
	union phy_configure_opts phy_opts = {0};
	uint32_t req_bw = soc_dp_calc_required_bw(dp->active_pixel_clk_khz, dp->active_bpp);

	dp->link_initialized = false;

	for (i = 0; i < ARRAY_SIZE(soc_dp_link_priority_table); i++) {
		uint32_t capacity;

		cfg = &soc_dp_link_priority_table[i];

		if (cfg->rate > dp->link.max_rate || cfg->lanes > dp->link.max_num_lanes)
			continue;

		capacity = soc_dp_calc_link_capacity(cfg->rate, cfg->lanes);
		if (capacity < req_bw)
			continue;

		dev_info(dp->dev, "Attempting Config: R=%d, L=%d (Cap: %u > Req: %u)\n",
				cfg->rate, cfg->lanes, capacity, req_bw);

		phy_opts.dp.lanes = cfg->lanes;
		phy_opts.dp.link_rate = cfg->rate / 1000;
		phy_opts.dp.set_lanes = 1;
		phy_opts.dp.set_rate = 1;

		if (phy_configure(dp->phy, &phy_opts))
			continue;

		if (dp->mode == SOC_DP_MODE_SST) {
			ret = clk_set_rate(dp->pixel_clks[0],
					(unsigned long)dp->active_pixel_clk_khz * 1000);
			if (ret) {
				dev_err(dp->dev,
					"Failed to set pixel clock rate: %d\n", ret);
				continue;
			}
		}

		if (phy_power_on(dp->phy))
			continue;

		if (soc_dp_link_train(dp, cfg->rate, cfg->lanes) == 0) {
			dp->link_initialized = true;
			dp->active_rate = cfg->rate;
			dp->active_lanes = cfg->lanes;
			dev_info(dp->dev, "Training successful for R:%d L:%d\n",
					cfg->rate, cfg->lanes);
			break;
		}

		phy_power_off(dp->phy);
		dev_warn(dp->dev, "Training failed for R:%d L:%d. Downgrading...\n",
				cfg->rate, cfg->lanes);
	}

	if (!dp->link_initialized) {
		dev_err(dp->dev, "Critical Failure - No valid link config found\n");
		return -EIO;
	}

	return 0;
}

/*
 * Handle Sink IRQ (Short HPD) by reading DPCD link status
 * and retraining the link if degradation or loss is detected.
 */
static void soc_dp_handle_sink_irq(struct soc_dp_dev *dp)
{
#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	int i, ret;
	uint32_t old_rate, old_lanes;
	bool handled = false;
	bool link_loss = false;
	bool trigger_hpd = false;
	uint8_t esi[16] = {0}, ack[2] = {0};
	uint8_t link_status[DP_LINK_STATUS_SIZE];

	mutex_lock(&dp->mode_lock);

	if (!dp->link_initialized) {
		mutex_unlock(&dp->mode_lock);
		return;
	}

	/* 1. Read DPCD status (merge MST UP_REQ and physical layer EQ handling) */
	if (dp->mode == SOC_DP_MODE_MST) {
		ret = drm_dp_dpcd_read(&dp->aux, DP_SINK_COUNT_ESI, esi, sizeof(esi));
		if (ret != sizeof(esi)) {
			mutex_unlock(&dp->mode_lock);
			dev_err(dp->dev, "Failed to read DPCD ESI\n");
			return;
		}

		ret = drm_dp_mst_hpd_irq_handle_event(&dp->mst_mgr, esi, ack, &handled);
		if (handled) {
			drm_dp_dpcd_writeb(&dp->aux, DP_SINK_COUNT_ESI + 1, ack[1]);
			drm_dp_mst_hpd_irq_send_new_request(&dp->mst_mgr);
		}

		if (esi[12] & DP_LINK_STATUS_UPDATED) {
			if (!drm_dp_channel_eq_ok(&esi[10], dp->active_lanes))
				link_loss = true;
			drm_dp_dpcd_writeb(&dp->aux, DP_LANE_ALIGN_STATUS_UPDATED_ESI, DP_LINK_STATUS_UPDATED);
		}
	} else {
		ret = drm_dp_dpcd_read(&dp->aux, DP_LANE0_1_STATUS, link_status, DP_LINK_STATUS_SIZE);
		if (ret != DP_LINK_STATUS_SIZE) {
			mutex_unlock(&dp->mode_lock);
			dev_err(dp->dev, "Failed to read DPCD on SINK IRQ\n");
			return;
		}

		if (link_status[2] & DP_LINK_STATUS_UPDATED) {
			if (!drm_dp_channel_eq_ok(link_status, dp->active_lanes))
				link_loss = true;
			drm_dp_dpcd_writeb(&dp->aux, DP_LANE_ALIGN_STATUS_UPDATED, DP_LINK_STATUS_UPDATED);
		}
	}

	/* 2. Link retraining and state decision */
	if (link_loss) {
		dev_warn(dp->dev, "Link loss detected! Retraining...\n");

		old_rate = dp->active_rate;
		old_lanes = dp->active_lanes;

		if (dp->mode == SOC_DP_MODE_SST) {
			soc_dp_disable(dp);
		} else {
			for (i = 0; i < dp->max_mst_streams; i++)
				soc_dp_mst_stream_disable(dp, i);
		}

		phy_power_off(dp->phy);

		if (soc_dp_establish_link(dp) == 0) {
			/* Check for link downgrade */
			if (dp->active_rate == old_rate && dp->active_lanes == old_lanes) {
				if (dp->mode == SOC_DP_MODE_SST) {
					dev_info(dp->dev, "Retraining successful, recovering SST stream\n");
					soc_dp_enable(dp);
				} else {
					dev_warn(dp->dev, "MST link lost, triggering HPD to rebuild topology\n");
					trigger_hpd = true;
				}
			} else {
				dev_warn(dp->dev, "Link downgraded (Rate:%u Lanes:%u), triggering HPD\n",
						dp->active_rate, dp->active_lanes);
				trigger_hpd = true;
			}
		} else {
			dev_err(dp->dev, "Retraining failed, triggering HPD\n");
			trigger_hpd = true;
		}
	}

	mutex_unlock(&dp->mode_lock);

	/* 3. Trigger uevent outside lock to avoid deadlock with atomic check */
	if (trigger_hpd)
		drm_kms_helper_hotplug_event(dp->drm);
#endif
}

/*
 * soc_dp_encoder_atomic_enable - Configure link and setup video timing.
 * Caches active mode properties and delegates link establishment
 * to soc_dp_establish_link for potential retraining scenarios.
 */
static void soc_dp_encoder_atomic_enable(struct drm_encoder *encoder,
		struct drm_atomic_state *state)
{
	uint32_t bpc, color_format;
	struct soc_dp_dev *dp = encoder_to_dp(encoder);
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_display_mode *adjusted_mode;

	connector = drm_atomic_get_new_connector_for_encoder(state, encoder);
	if (WARN_ON(!connector))
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;

	adjusted_mode = &crtc_state->adjusted_mode;

	mutex_lock(&dp->mode_lock);

	soc_dp_compute_color_spec(dp, adjusted_mode, conn_state, &bpc, &color_format);

	/* Cache active mode properties for potential retraining scenarios. */
	dp->active_pixel_clk_khz = adjusted_mode->clock;
	dp->active_bpp = soc_dp_get_bpp(color_format);

	dev_info(dp->dev, "Mode Set %ux%u (PCLK: %u kHz)\n",
			adjusted_mode->hdisplay, adjusted_mode->vdisplay, adjusted_mode->clock);

	if (soc_dp_establish_link(dp) < 0) {
		mutex_unlock(&dp->mode_lock);
		return;
	}

	soc_dp_set_msa(dp, adjusted_mode, color_format, dp->active_rate, dp->active_lanes);
	msleep(20);

	soc_dp_enable(dp);
	mutex_unlock(&dp->mode_lock);
}

static const struct drm_encoder_helper_funcs soc_dp_encoder_helper_funcs = {
	.mode_valid     = soc_dp_encoder_mode_valid,
	.atomic_check   = soc_dp_encoder_atomic_check,
	.atomic_enable  = soc_dp_encoder_atomic_enable,
	.atomic_disable = soc_dp_encoder_atomic_disable,
};

static int soc_dp_mode_switch(struct soc_dp_dev *dp, enum soc_dp_mode new_mode)
{
	int ret = 0;

	if (dp->mode == new_mode)
		return 0;

	if (dp->mode == SOC_DP_MODE_MST)
		soc_dp_mst_fini(dp);

	dp->mode = new_mode;

	if (dp->mode == SOC_DP_MODE_MST)
		ret = soc_dp_mst_init(dp);

	if (ret)
		dp->mode = SOC_DP_MODE_NONE;

	dev_info(dp->dev, "Mode switch -> %s\n",
			dp->mode == SOC_DP_MODE_SST ? "SST" :
			dp->mode == SOC_DP_MODE_MST ? "MST" : "NONE");

	return ret;
}

static void soc_dp_mode_update(struct soc_dp_dev *dp)
{
	enum soc_dp_mode new_mode = SOC_DP_MODE_NONE;

	mutex_lock(&dp->mode_lock);

	if (dp->connector_status == connector_status_connected) {
		if (!soc_dp_read_sink_caps(dp)) {
			if (drm_dp_read_mst_cap(&dp->aux, dp->dpcd))
				new_mode = SOC_DP_MODE_MST;
			else
				new_mode = SOC_DP_MODE_SST;
		} else {
			dev_warn(dp->dev, "DPCD read failed, defaulting to SST\n");
			new_mode = SOC_DP_MODE_SST;
		}
	}

	if (dp->only_sst && new_mode == SOC_DP_MODE_MST)
		new_mode = SOC_DP_MODE_SST;

	soc_dp_mode_switch(dp, new_mode);

	mutex_unlock(&dp->mode_lock);
}

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
static ssize_t soc_dp_irq_proc_write(struct file *filp,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret;
	char cmd;
	char buf[32] = {0};
	int id = -1, port_id = 0;
	struct soc_dp_dev *dp = pde_data(file_inode(filp));

	if (count >= sizeof(buf))
		count = sizeof(buf) - 1;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (sscanf(buf, "%c %d %d", &cmd, &id, &port_id) < 1)
		return -EINVAL;

	switch (cmd) {
	case 'd':
		dp->connector_status = connector_status_disconnected;
		soc_dp_mode_update(dp);
		drm_kms_helper_hotplug_event(dp->drm);
		break;

	case 's':
		if (dp->mode != SOC_DP_MODE_NONE) {
			dp->connector_status = connector_status_disconnected;
			soc_dp_mode_update(dp);
			drm_kms_helper_hotplug_event(dp->drm);
			msleep(1000);
		}
		soc_dp_virtual_rx_set_mst_cap(&dp->rx, false);
		dp->connector_status = connector_status_connected;
		soc_dp_mode_update(dp);
		drm_kms_helper_hotplug_event(dp->drm);
		break;

	case 'm':
		if (dp->mode != SOC_DP_MODE_NONE) {
			dp->connector_status = connector_status_disconnected;
			soc_dp_mode_update(dp);
			drm_kms_helper_hotplug_event(dp->drm);
			msleep(1000);
		}
		soc_dp_virtual_rx_set_mst_cap(&dp->rx, true);
		dp->connector_status = connector_status_connected;
		soc_dp_mode_update(dp);
		drm_kms_helper_hotplug_event(dp->drm);
		break;

	case 'p':
			soc_dp_mst_topology_dump_all(&dp->rx, 0 /* pbn_div */);
			break;

	case 'u':
			if (id <= 0) {
				dev_err(dp->dev, "Invalid device ID\n");
				break;
			}

			ret = soc_dp_mst_unplug_device(&dp->rx, id);
			if (ret < 0) {
				dev_err(dp->dev, "Unplug device %d failed, err=%d\n", id, ret);
			} else {
				soc_dp_mst_topology_dump_all(&dp->rx, 0 /* pbn_div */);
			}
			break;
	case 'a':
			if (id <= 0) {
				dev_err(dp->dev, "Invalid device ID\n");
				break;
			}
			if (port_id <= 0) {
				dev_err(dp->dev, "Invalid target port ID\n");
				break;
			}

			ret = soc_dp_mst_plug_device(&dp->rx, id, port_id);
			if (ret < 0) {
				dev_err(dp->dev, "Attach device %d to port %d failed, err=%d\n",
						id, port_id, ret);
			} else {
				soc_dp_mst_topology_dump_all(&dp->rx, 0 /* pbn_div */);
			}
			break;
	case 'c':
			if (id <= 0) {
				dev_err(dp->dev, "Invalid root ID to clone\n");
				break;
			}
			if (port_id < 0)
				port_id = 0;

			ret = soc_dp_mst_clone_subtree(&dp->rx, id, port_id);
			if (ret < 0) {
				dev_err(dp->dev, "Clone subtree %d failed, err=%d\n", id, ret);
			} else {
				soc_dp_mst_topology_dump_all(&dp->rx, 0 /* pbn_div */);
			}
			break;
	case 'r':
			if (id <= 0) {
				dev_err(dp->dev, "Invalid root ID to delete\n");
				break;
			}

			ret = soc_dp_mst_delete_subtree(&dp->rx, id);
			if (ret < 0) {
				dev_err(dp->dev, "Delete subtree %d failed, err=%d\n", id, ret);
			} else {
				soc_dp_mst_topology_dump_all(&dp->rx, 0 /* pbn_div */);
			}
			break;
	default:
		pr_info("DP debug interface\n");
		pr_info("  d   Disconnect\n");
		pr_info("  s   Connect (SST)\n");
		pr_info("  m   Connect (MST)\n");
		pr_info("  h   Help\n");
		if (dp->mode == SOC_DP_MODE_MST) {
			pr_info("MST operations:\n");
			pr_info("  p              Dump topology\n");
			pr_info("  u <id>         Unplug device\n");
			pr_info("  a <id> <port>  Plug device to port\n");
			pr_info("  c <id> [port]  Clone subtree\n");
			pr_info("  r <id>         Delete subtree\n");
		}
	}

	return count;
}

static const struct proc_ops soc_dp_irq_proc_ops = {
	.proc_write = soc_dp_irq_proc_write,
};

static void soc_dp_proc_irq_debug_init(struct soc_dp_dev *dp)
{
	char proc_name[32];

	snprintf(proc_name, sizeof(proc_name), "soc-dp-%s",
			dev_name(dp->dev));
	dp->proc_irq = proc_create_data(proc_name,
			0200, NULL, &soc_dp_irq_proc_ops, dp);
}

static void soc_dp_proc_irq_debug_exit(struct soc_dp_dev *dp)
{
	if (dp->proc_irq)
		proc_remove(dp->proc_irq);
	dp->proc_irq = NULL;
}

static void soc_dp_rx_up_req_cb(void *data)
{
	int ret;
	ssize_t n;
	uint8_t esi[16] = {0}, ack[2] = {0};
	bool handled = false;
	struct soc_dp_dev *dp = data;

	if (dp->mode != SOC_DP_MODE_MST)
		return;

	n = drm_dp_dpcd_read(&dp->aux, DP_SINK_COUNT_ESI, esi, sizeof(esi));
	if (n != sizeof(esi)) {
		dev_warn(dp->dev, "ESI read failed, n=%zd\n", n);
		return;
	}

	ret = drm_dp_mst_hpd_irq_handle_event(&dp->mst_mgr, esi, ack, &handled);
	if (ret)
		dev_warn(dp->dev, "MST HPD IRQ failed, err=%d\n", ret);
	if (handled) {
		drm_dp_dpcd_writeb(&dp->aux, DP_SINK_COUNT_ESI + 1, ack[1]);
		drm_dp_mst_hpd_irq_send_new_request(&dp->mst_mgr);
	}
}
#endif

#ifdef CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED
static void soc_dp_hpd_poll_work(struct work_struct *work)
{
	bool sink_irq_detected;
	struct soc_dp_dev *dp = container_of(work, struct soc_dp_dev, hpd_work.work);
	enum drm_connector_status old_status, new_status;

	mutex_lock(&dp->mode_lock);

	old_status = dp->connector_status;
	new_status = soc_dp_detect_hpd(dp);
	dp->connector_status = new_status;
	soc_dp_clean_hpd(dp);

	sink_irq_detected = soc_dp_get_sink_irq(dp);
	soc_dp_clean_sink_irq(dp);

	mutex_unlock(&dp->mode_lock);

	if (sink_irq_detected
			&& new_status == connector_status_connected)
		soc_dp_handle_sink_irq(dp);

	if (new_status != old_status) {
		soc_dp_mode_update(dp);
		drm_kms_helper_hotplug_event(dp->drm);
	}

	schedule_delayed_work(&dp->hpd_work, msecs_to_jiffies(HPD_POLL_INTERVAL_MS));
}
#else
static irqreturn_t soc_dp_irq_handler(int irq, void *data)
{
	struct soc_dp_dev *dp = data;
	enum drm_connector_status old_status, new_status;

	old_status = dp->connector_status;
	new_status = soc_dp_detect_hpd(dp);
	dp->connector_status = new_status;

	soc_dp_clean_hpd(dp);

	if (soc_dp_get_sink_irq(dp)
			&& new_status == connector_status_connected)
		dp->sink_irq_pending = true;

	soc_dp_clean_sink_irq(dp);

	if (new_status != old_status || dp->sink_irq_pending)
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

static irqreturn_t soc_dp_hotplug_event_handler(int irq, void *data)
{
	struct soc_dp_dev *dp = data;

	mutex_lock(&dp->mode_lock);
	dp->connector_status = soc_dp_detect_hpd(dp);
	mutex_unlock(&dp->mode_lock);

	if (dp->sink_irq_pending) {
		dp->sink_irq_pending = false;
		soc_dp_handle_sink_irq(dp);
	}

	soc_dp_mode_update(dp);

	drm_kms_helper_hotplug_event(dp->drm);
	return IRQ_HANDLED;
}
#endif

static int soc_dp_dev_init(struct soc_dp_dev *dp)
{
	int ret;

	// Reset Controller
	soc_dp_reg_write_range(dp, SOC_DPTX_CONTROLLER_RESET, 0x1);
	soc_dp_reg_write_range(dp, SOC_DPTX_HDCP_RESET, 0x1);
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_RESET, 0x1);
	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_RESET, 0x1);
	mdelay(5);

	// Clear Video Reset
	soc_dp_reg_write_range(dp, SOC_DPTX_CONTROLLER_RESET, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_HDCP_RESET, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_RESET, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_RESET, 0x0);
	mdelay(2);

	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_REPLY_EVENT_INT_STA, 1);

	soc_dp_reg_write_range(dp, SOC_DPTX_DEFAULT_FAST_LINK_TRAIN_EN, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SCRAMBLER_DISABLE, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SCALE_DOWN_MODE, 0x0);

	// Unmask Interrupts
	soc_dp_reg_write_range(dp, SOC_DPTX_AUX_REPLY_EVENT_INT_STA_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_HDCP_INT_STA_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_ILLEGAL_AUX_CMD_INT_STA_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_TYPE_C_EVENT_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_DSC_EVENT_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SDP_INT_STA_S3_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SDP_INT_STA_S2_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SDP_INT_STA_S1_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SDP_INT_STA_S0_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_FIFO_OVERFLOW_INT_STA_S3_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_FIFO_OVERFLOW_INT_STA_S2_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_FIFO_OVERFLOW_INT_STA_S1_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_FIFO_OVERFLOW_INT_STA_S0_MSK, 0x0);
#if defined(CONFIG_SOC_DP_HPD_BYPASS) || defined(CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED)
	soc_dp_reg_write_range(dp, SOC_DPTX_HPD_INT_STA_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_HOT_PLUG_EVENT_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_HOT_UNPLUG_EVENT_MSK, 0x0);
	soc_dp_reg_write_range(dp, SOC_DPTX_SINK_IRQ_EVENT_MSK, 0x0);
#else
	soc_dp_reg_write_range(dp, SOC_DPTX_HPD_INT_STA_MSK, 0x1);
	soc_dp_reg_write_range(dp, SOC_DPTX_HOT_PLUG_EVENT_MSK, 0x1);
	soc_dp_reg_write_range(dp, SOC_DPTX_HOT_UNPLUG_EVENT_MSK, 0x1);
	soc_dp_reg_write_range(dp, SOC_DPTX_SINK_IRQ_EVENT_MSK, 0x1);
#endif
	soc_dp_reg_write_range(dp, SOC_DPTX_SINK_UNPLUG_ERROR_EVENT_MSK, 0x0);
	mdelay(2);

	soc_dp_reg_write_range(dp, SOC_DPTX_VIDEO_STREAM_ENABLE, 0);
	soc_dp_reg_write_range(dp, SOC_DPTX_REG_VID_CLK_SEL, 0);
	soc_dp_reg_write_range(dp, SOC_DPTX_VID_BIST_EN, 0);

	// Initial PHY Config
	ret = phy_init(dp->phy);
	if (ret) {
		dev_err(dp->dev, "Failed to init PHY\n");
		return ret;
	}

	ret = phy_set_mode(dp->phy, PHY_MODE_DP);
	if (ret) {
		dev_err(dp->dev, "Failed to set PHY mode, err=%d\n", ret);
		phy_exit(dp->phy);
		return ret;
	}

	// Update connector status using hardware detection interface
#if defined(CONFIG_SOC_DP_DRIVER_QEMU) || defined(CONFIG_SOC_DP_HPD_BYPASS)
	soc_dp_reg_write_range(dp, SOC_DPTX_FORCE_HPD, 0x1);
	mdelay(5);
	dp->connector_status = connector_status_connected;
#else
	dp->connector_status = soc_dp_detect_hpd(dp);
#endif

#ifdef CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED
	dev_info(dp->dev, "Starting HPD Polling Thread...\n");
	schedule_delayed_work(&dp->hpd_work, msecs_to_jiffies(HPD_POLL_INTERVAL_MS));
#else
	ret = request_threaded_irq(dp->irq, soc_dp_irq_handler,
			soc_dp_hotplug_event_handler, 0, dev_name(dp->dev), dp);
	if (ret) {
		dev_err(dp->dev, "Failure requesting irq %d, err=%d\n", dp->irq, ret);
		phy_exit(dp->phy);
		return ret;
	}
#endif

	return 0;
}

static int soc_dp_bind(struct device *dev, struct device *master, void *data)
{
	int ret;
	struct soc_dp_dev *dp = dev_get_drvdata(dev);
	struct drm_device *drm = (struct drm_device *)data;
#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	struct soc_dp_virtual_rx_init_data rx_init;
#endif

	dp->drm = drm;
	dp->aux.drm_dev = drm;
	dp->mode = SOC_DP_MODE_NONE;
	mutex_init(&dp->mode_lock);

	/* Init Encoder */
	ret = drm_encoder_init(drm, &dp->encoder,
			&soc_dp_encoder_funcs, DRM_MODE_ENCODER_TMDS, NULL);
	if (ret) {
		dev_err(dev, "Encoder init failed, err=%d\n", ret);
		return ret;
	}
	drm_encoder_helper_add(&dp->encoder, &soc_dp_encoder_helper_funcs);

	dp->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	dp->aux.name = "soc-dp-aux";
	dp->aux.dev = dev;
	dp->aux.transfer = soc_dp_aux_transfer;

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	rx_init.up_req_cb      = soc_dp_rx_up_req_cb;
	rx_init.up_req_cb_data = dp;
	rx_init.topology_build = soc_dp_mst_default_topology_build;
	ret = soc_dp_virtual_rx_init(&dp->rx, &rx_init);
	if (ret) {
		dev_err(dev, "Virtual RX init failed, err=%d\n", ret);
		goto err_encoder;
	}
#endif

	ret = drm_dp_aux_register(&dp->aux);
	if (ret) {
		dev_err(dev, "AUX registration failed, err=%d\n", ret);
		goto err_rx;
	}

	/* Init Connector */
	ret = soc_dp_connector_create(dp);
	if (ret) {
		dev_err(dev, "Connector create failed, err=%d\n", ret);
		goto err_aux;
	}

	ret = soc_dp_mst_create(dp);
	if (ret) {
		dev_err(dev, "MST create failed, err=%d\n", ret);
		goto err_connector;
	}

	ret = soc_dp_dev_init(dp);
	if (ret) {
		dev_err(dev, "Device init failed, err=%d\n", ret);
		goto err_mst;
	}

	soc_dp_mode_update(dp);
	drm_kms_helper_hotplug_event(dp->drm);

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_proc_irq_debug_init(dp);
#endif

	return 0;

err_mst:
	soc_dp_mst_destroy(dp);
err_connector:
	soc_dp_connector_destroy(dp);
err_aux:
	drm_dp_aux_unregister(&dp->aux);
err_rx:
#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_virtual_rx_fini(&dp->rx);
err_encoder:
#endif
	drm_encoder_cleanup(&dp->encoder);
	return ret;
}

static void soc_dp_unbind(struct device *dev, struct device *master, void *data)
{
	struct soc_dp_dev *dp = dev_get_drvdata(dev);

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_proc_irq_debug_exit(dp);
#endif

#ifdef CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED
	cancel_delayed_work_sync(&dp->hpd_work);
#else
	free_irq(dp->irq, dp);
#endif

	soc_dp_mode_switch(dp, SOC_DP_MODE_NONE);

	if (dp->phy)
		phy_exit(dp->phy);

	drm_dp_aux_unregister(&dp->aux);

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_virtual_rx_fini(&dp->rx);
#endif

	soc_dp_mst_destroy(dp);
	soc_dp_connector_destroy(dp);

	drm_encoder_cleanup(&dp->encoder);
}

static const struct component_ops soc_dp_ops = {
	.bind   = soc_dp_bind,
	.unbind = soc_dp_unbind,
};

static int soc_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct soc_dp_dev *dp;
	struct resource *res;

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dev = dev;
	dp->proc_irq = NULL;
	dp->max_mst_streams = 1;
	dp->connector_status = connector_status_connected;
	dp->only_sst = device_property_read_bool(dev, "only-sst");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Failed to obtain dp resource\n");
		return -EINVAL;
	}

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	dp->regs = devm_ioremap(dev, res->start, resource_size(res));
#else
	dp->regs = devm_kmalloc(dev, resource_size(res), GFP_KERNEL);
#endif
	if (!dp->regs)
		return -ENOMEM;

#ifdef CONFIG_SOC_DP_HOT_PLUG_THREAD_ENABLED
	INIT_DELAYED_WORK(&dp->hpd_work, soc_dp_hpd_poll_work);
#else
	dp->irq = platform_get_irq(pdev, 0);
	if (dp->irq < 0) {
		dev_err(dev, "Failed to get IRQ\n");
		return dp->irq;
	}
#endif

	/*
	 * TSI: under ACPI the PHY and pixel clocks come from lookups the PHY
	 * driver registers in its probe; "not found" then means "not yet".
	 */
	dp->phy = devm_phy_get(dev, "phy");
	if (IS_ERR(dp->phy)) {
		if (!dev->of_node && PTR_ERR(dp->phy) == -ENODEV)
			return -EPROBE_DEFER;
		dev_err(dev, "Failed to get PHY\n");
		return PTR_ERR(dp->phy);
	}

	dp->pixel_clks[0] = devm_clk_get(dev, "pixel-0");
	if (IS_ERR(dp->pixel_clks[0])) {
		if (!dev->of_node && PTR_ERR(dp->pixel_clks[0]) == -ENOENT)
			return -EPROBE_DEFER;
		return dev_err_probe(dev, PTR_ERR(dp->pixel_clks[0]),
			"Failed to get pixel-0 clock\n");
	}

	platform_set_drvdata(pdev, dp);

	return component_add(dev, &soc_dp_ops);
}

static void soc_dp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &soc_dp_ops);
}

static const struct of_device_id soc_dp_match[] = {
	{ .compatible = "soc,dp" },
	{}
};
MODULE_DEVICE_TABLE(of, soc_dp_match);

static struct platform_driver soc_dp_driver = {
	.probe  = soc_dp_probe,
	.remove = soc_dp_remove,
	.driver = {
		.name = "soc_dp",
		.of_match_table = soc_dp_match,
	},
};

module_platform_driver(soc_dp_driver);
MODULE_LICENSE("GPL");
