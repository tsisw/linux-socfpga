#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/minmax.h>
#include <linux/bitops.h>
#include <linux/random.h>
#include <linux/kthread.h>

#include <drm/drm_edid.h>

#include "soc_dp_virtual_rx.h"
#include "soc_dp_dri.h"

#define DDC_SEGMENT_ADDR 0x30
#define MST_REPLY_MAX_LEN 1024

enum soc_dp_mst_find_type {
	SOC_DP_MST_FIND_NONE = 0,
	SOC_DP_MST_FIND_DEVICE,
	SOC_DP_MST_FIND_PORT,
};

struct soc_dp_mst_find_result {
	enum soc_dp_mst_find_type type;
	union {
		struct soc_dp_mst_dev *dev;
		struct soc_dp_mst_port *port;
	};
	struct soc_dp_mst_dev *owner;
};

enum dpcd_access {
	DPCD_RO = 0,
	DPCD_RW = 1,
	DPCD_WO = 2,
	DPCD_FN = 3,
};

typedef ssize_t (*dpcd_handler_t)(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len, bool write, unsigned int base);

struct dpcd_range {
	unsigned int base;
	unsigned int size;
	enum dpcd_access access;
	uint8_t *store;
	dpcd_handler_t handler;
	const char *name;
};

static const uint8_t raw_edid[] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x05, 0xe3, 0x01, 0xb2, 0xc4, 0x07, 0x00, 0x00,
	0x23, 0x1f, 0x01, 0x03, 0x80, 0x3e, 0x22, 0x78, 0x2e, 0xe6, 0x15, 0xac, 0x50, 0x45, 0x9f, 0x26,
	0x0e, 0x50, 0x54, 0xbf, 0xef, 0x00, 0xd1, 0xc0, 0x81, 0x80, 0x31, 0x68, 0x31, 0x7c, 0x45, 0x68,
	0x45, 0x7c, 0x61, 0x68, 0x61, 0x7c, 0x4d, 0xd0, 0x00, 0xa0, 0xf0, 0x70, 0x3e, 0x80, 0x30, 0x40,
	0x35, 0x00, 0x6d, 0x55, 0x1a, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0xff, 0x00, 0x50, 0x44, 0x52,
	0x4d, 0x39, 0x4a, 0x41, 0x30, 0x30, 0x31, 0x39, 0x38, 0x38, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x55,
	0x32, 0x38, 0x47, 0x32, 0x47, 0x34, 0x52, 0x34, 0x0a, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfd,
	0x00, 0x30, 0x78, 0x1e, 0xff, 0x3c, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x72,
	0x02, 0x03, 0x4d, 0xf1, 0x4d, 0x90, 0x04, 0x03, 0x1f, 0x13, 0x01, 0x12, 0x5d, 0x5e, 0x5f, 0x60,
	0x61, 0x3f, 0x23, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00, 0x6d, 0x03, 0x0c, 0x00, 0x10, 0x00,
	0x38, 0x3c, 0x20, 0x00, 0x60, 0x01, 0x02, 0x03, 0x67, 0xd8, 0x5d, 0xc4, 0x01, 0x78, 0x80, 0x03,
	0xe3, 0x05, 0xe3, 0x01, 0xe3, 0x0f, 0x00, 0x0c, 0xe6, 0x06, 0x07, 0x01, 0x62, 0x62, 0x00, 0x6d,
	0x1a, 0x00, 0x00, 0x02, 0x01, 0x30, 0x78, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0xc2, 0x00,
	0xa0, 0xa0, 0xa0, 0x55, 0x50, 0x30, 0x20, 0x35, 0x00, 0x6d, 0x55, 0x21, 0x00, 0x00, 0x1e, 0x4d,
	0x6c, 0x80, 0xa0, 0x70, 0x70, 0x3e, 0x80, 0x30, 0x20, 0x3a, 0x00, 0x6d, 0x55, 0x21, 0x00, 0x00,
	0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
};

static const uint8_t raw_dpcd_endpoint[] = {
	[DP_DPCD_REV]                = DP_DPCD_REV_14,
	[DP_MAX_LINK_RATE]           = DP_LINK_BW_5_4,
	[DP_MAX_LANE_COUNT]          = 4 | DP_TPS3_SUPPORTED | DP_ENHANCED_FRAME_CAP,
	[DP_MAX_DOWNSPREAD]          = DP_MAX_DOWNSPREAD_0_5,
	[DP_NORP]                    = 0x01,
	[DP_DOWNSTREAMPORT_PRESENT]  = 0x00,
	[DP_MAIN_LINK_CHANNEL_CODING] = DP_CAP_ANSI_8B10B,
	[DP_DOWN_STREAM_PORT_COUNT]  = 0x00,
	[DP_RECEIVE_PORT_0_CAP_0]    = DP_LOCAL_EDID_PRESENT,
	[DP_MSTM_CAP]                = 0x00,
	[DP_DSC_SUPPORT]             = DP_DSC_DECOMPRESSION_IS_SUPPORTED,
	[DP_FEC_CAPABILITY]          = DP_FEC_CAPABLE |
		DP_FEC_UNCORR_BLK_ERROR_COUNT_CAP | DP_FEC_CORR_BLK_ERROR_COUNT_CAP,
};

static const uint8_t raw_dpcd_branch[] = {
	[DP_DPCD_REV]                = DP_DPCD_REV_14,
	[DP_MAX_LINK_RATE]           = DP_LINK_BW_5_4,
	[DP_MAX_LANE_COUNT]          = 4 | DP_TPS3_SUPPORTED | DP_ENHANCED_FRAME_CAP,
	[DP_MAX_DOWNSPREAD]          = DP_MAX_DOWNSPREAD_0_5,
	[DP_NORP]                    = 0x01,
	[DP_DOWNSTREAMPORT_PRESENT]  = DP_DWN_STRM_PORT_PRESENT | DP_DWN_STRM_PORT_TYPE_DP,
	[DP_MAIN_LINK_CHANNEL_CODING] = DP_CAP_ANSI_8B10B,
	[DP_DOWN_STREAM_PORT_COUNT]  = 0x00,
	[DP_RECEIVE_PORT_0_CAP_0]    = 0x00,
	[DP_MSTM_CAP]                = DP_MST_CAP,
	[DP_DSC_SUPPORT]             = DP_DSC_DECOMPRESSION_IS_SUPPORTED | 0x2,
	[DP_FEC_CAPABILITY]          = DP_FEC_CAPABLE |
		DP_FEC_UNCORR_BLK_ERROR_COUNT_CAP | DP_FEC_CORR_BLK_ERROR_COUNT_CAP,
};

static uint8_t gbps_to_dpcd_bw(uint8_t dgbps)
{
	switch (dgbps) {
	case 16: return DP_LINK_BW_1_62;
	case 27: return DP_LINK_BW_2_7;
	case 54: return DP_LINK_BW_5_4;
	case 81: return DP_LINK_BW_8_1;
	default: return DP_LINK_BW_5_4;
	}
}

static ssize_t aux_handle_mst(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len, bool write, unsigned int base);
static ssize_t aux_handle_esi(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len, bool write, unsigned int base);
static ssize_t dpcd_sink_count_handler(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len, bool write, unsigned int base);
static void soc_dp_mst_dev_free(struct soc_dp_mst_dev *dev);
static struct soc_dp_mst_find_result
soc_dp_mst_find_by_id(struct soc_dp_virtual_rx *rx, int id);

static int mst_count_endpoints(struct soc_dp_mst_dev *dev)
{
	int i, count = 0;

	if (!dev)
		return 0;
	if (dev->type == SOC_DP_MST_ENDPOINT)
		return 1;
	if (dev->ports) {
		for (i = 0; i < dev->num_ports; i++) {
			if (dev->ports[i].connected && dev->ports[i].device)
				count += mst_count_endpoints(dev->ports[i].device);
		}
	}

	return count;
}

static void mst_update_sink_count_locked(struct soc_dp_virtual_rx *rx)
{
	int count = mst_count_endpoints(rx->mst_root);

	rx->esi[0] = count & 0xff;
	rx->sink_cnt[0] = count & 0xff;
}

static void mst_reset_payloads_recursive(struct soc_dp_mst_dev *dev);
static bool soc_dp_mst_is_in_active_tree(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *dev);
static int __soc_dp_mst_notify_conn_stat_locked(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *root, int device_id, bool plugged);

static int __soc_dp_mst_plug_device_no_lock(struct soc_dp_virtual_rx *rx, int device_id, int target_port_id);
static int __soc_dp_mst_unplug_device_no_lock(struct soc_dp_virtual_rx *rx, int device_id);
static int __soc_dp_mst_add_subtree_no_lock(struct soc_dp_virtual_rx *rx, struct soc_dp_mst_dev *tree_root, int target_port_id);

static const struct dpcd_range dpcd_ranges_template[] = {
	{ DP_DPCD_REV,                    DP_RX_CAPS_SIZE,       DPCD_RO, NULL, NULL,                    "RX_CAPS_RO"   },
	{ DP_GUID,                        DP_GUID_SIZE,          DPCD_RW, NULL, NULL,                    "RX_GUID"      },
	{ DP_RX_CAPS_TAIL_BASE,           DP_RX_CAPS_TAIL_SIZE,  DPCD_RO, NULL, NULL,                    "RX_CAPS_TAIL" },
	{ DP_LINK_BW_SET,                 DPCD_PAGE_SIZE,        DPCD_RW, NULL, NULL,                    "LINK_CFG"     },
	{ DP_PAYLOAD_ALLOCATE_SET,        DP_PAYLOAD_ALLOC_SIZE, DPCD_RW, NULL, NULL,                    "PAYLOAD_ALLOC"},
	{ DP_PAYLOAD_TABLE_UPDATE_STATUS, 1,                     DPCD_RW, NULL, NULL,                    "PAYLOAD_STAT" },
	{ DP_SINK_COUNT,                  DPCD_PAGE_SIZE,        DPCD_FN, NULL, dpcd_sink_count_handler, "SINK_COUNT"   },
	{ DP_SIDEBAND_MSG_DOWN_REQ_BASE,  DPCD_MST_WINDOW_SIZE,  DPCD_FN, NULL, aux_handle_mst,          "MST_DOWN"     },
	{ DP_SIDEBAND_MSG_UP_REP_BASE,    DPCD_MST_WINDOW_SIZE,  DPCD_FN, NULL, aux_handle_mst,          "MST_UP"       },
	{ DP_SIDEBAND_MSG_DOWN_REP_BASE,  DPCD_MST_WINDOW_SIZE,  DPCD_FN, NULL, aux_handle_mst,          "MST_DOWN_REP" },
	{ DP_SIDEBAND_MSG_UP_REQ_BASE,    DPCD_MST_WINDOW_SIZE,  DPCD_FN, NULL, aux_handle_mst,          "MST_UP_REQ"   },
	{ DP_SINK_COUNT_ESI & ~0x1ff,     DPCD_PAGE_SIZE,        DPCD_FN, NULL, aux_handle_esi,          "ESI"          },
};

static const struct dpcd_range *dpcd_find_range(struct soc_dp_virtual_rx *rx, unsigned int addr)
{
	size_t i;
	struct dpcd_range *ranges = (struct dpcd_range *)rx->ranges;

	if (!ranges)
		return NULL;

	for (i = 0; i < rx->num_ranges; i++) {
		if (addr >= ranges[i].base && addr < ranges[i].base + ranges[i].size)
			return &ranges[i];
	}

	return NULL;
}

static ssize_t dpcd_simple_access(const struct dpcd_range *r, unsigned int offset,
		void *buf, size_t len, bool write)
{
	len = min_t(size_t, len, r->size - offset);

	if (write) {
		if (r->access == DPCD_RW)
			memcpy(r->store + offset, buf, len);
	} else {
		if (r->access == DPCD_WO)
			memset(buf, 0, len);
		else
			memcpy(buf, r->store + offset, len);
	}

	return len;
}

static ssize_t dpcd_sink_count_handler(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len, bool write, unsigned int base)
{
	unsigned int idx;
	const struct dpcd_range *r = dpcd_find_range(rx, DP_SINK_COUNT);

	if (!write)
		return dpcd_simple_access(r, offset, buf, len, false);

	len = min_t(size_t, len, r->size - offset);
	if (offset == 0 && len >= 1)
		r->store[0] = ((uint8_t *)buf)[0];
	if (offset <= 1 && offset + len > 1) {
		idx = (offset <= 1) ? 1 - offset : 0;
		r->store[DP_DEVICE_SERVICE_IRQ_VECTOR - DP_SINK_COUNT] &= ~((uint8_t *)buf)[idx];
	}

	return len;
}

static void dpcd_init_defaults(struct soc_dp_virtual_rx *rx)
{
	rx->rx_caps[DP_DPCD_REV]                 = DP_DPCD_REV_14;
	rx->rx_caps[DP_MAX_LINK_RATE]            = DP_LINK_BW_5_4;
	rx->rx_caps[DP_MAX_LANE_COUNT]           = 4 | DP_TPS3_SUPPORTED | DP_ENHANCED_FRAME_CAP;
	rx->rx_caps[DP_MAX_DOWNSPREAD]           = DP_MAX_DOWNSPREAD_0_5;
	rx->rx_caps[DP_NORP]                     = 0x01;
	rx->rx_caps[DP_DOWNSTREAMPORT_PRESENT]   = DP_DWN_STRM_PORT_PRESENT | DP_DWN_STRM_PORT_TYPE_DP;
	rx->rx_caps[DP_MAIN_LINK_CHANNEL_CODING] = DP_CAP_ANSI_8B10B;
	rx->rx_caps[DP_DOWN_STREAM_PORT_COUNT]   = 0x00;
	rx->rx_caps[DP_RECEIVE_PORT_0_CAP_0]     = BIT(0) | DP_LOCAL_EDID_PRESENT;
	rx->rx_caps[DP_MSTM_CAP]                 = DP_MST_CAP;
	rx->rx_caps[DP_RECEIVE_PORT_0_BUFFER_SIZE] = 1 | DP_ASSOCIATED_TO_PRECEDING_PORT;
	rx->sink_cnt[0]                          = 1;
}

static ssize_t aux_handle_native(struct soc_dp_virtual_rx *rx, struct drm_dp_aux_msg *msg)
{
	uint8_t val;
	bool is_write = (msg->request == DP_AUX_NATIVE_WRITE);
	const struct dpcd_range *r;

	if (is_write && msg->address == DP_PAYLOAD_TABLE_UPDATE_STATUS) {
		if (msg->size > 0 && msg->buffer) {
			val = ((uint8_t *)msg->buffer)[0];
			if (val & DP_PAYLOAD_TABLE_UPDATED)
				rx->payload_status = DP_PAYLOAD_TABLE_UPDATED;
			else
				rx->payload_status = val;
		}
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return msg->size;
	}

	if (!is_write && msg->address == DP_PAYLOAD_TABLE_UPDATE_STATUS) {
		if (rx->payload_status & DP_PAYLOAD_TABLE_UPDATED &&
				!(rx->payload_status & DP_PAYLOAD_ACT_HANDLED)) {
			uint8_t vc_id = rx->payload_alloc[0];
			uint8_t start = rx->payload_alloc[1];
			uint8_t num   = rx->payload_alloc[2];
			bool valid = true;

			if (num > 0) {
				if (start + num > DP_PAYLOAD_SLOT_MAX)
					valid = false;
				if (vc_id == 0)
					valid = false;
			}

			if (valid)
				rx->payload_status |= DP_PAYLOAD_ACT_HANDLED;
		}
		if (msg->size > 0 && msg->buffer)
			*(uint8_t *)msg->buffer = rx->payload_status;
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return 1;
	}
	r = dpcd_find_range(rx, msg->address);

	if (!r) {
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
		return -EIO;
	}

	if (r->access == DPCD_FN)
		return r->handler(rx, msg->address - r->base, msg->buffer, msg->size, is_write, r->base);

	msg->reply = DP_AUX_NATIVE_REPLY_ACK;
	return dpcd_simple_access(r, msg->address - r->base, msg->buffer, msg->size, is_write);
}

static ssize_t aux_handle_ddc(struct soc_dp_virtual_rx *rx, struct drm_dp_aux_msg *msg)
{
	size_t i;
	bool is_read = (msg->request & DP_AUX_I2C_READ) == DP_AUX_I2C_READ;
	unsigned int segment_base;

	if (!is_read) {
		if (msg->buffer && msg->size > 0) {
			const uint8_t *buf = msg->buffer;

			if (msg->address == DDC_ADDR)
				rx->edid_offset = buf[0];
			else if (msg->address == DDC_SEGMENT_ADDR)
				rx->edid_segment = buf[0];
		}
		msg->reply = DP_AUX_I2C_REPLY_ACK;
		return msg->size;
	}

	if (msg->size == 0) {
		msg->reply = DP_AUX_I2C_REPLY_ACK;
		return 0;
	}

	if (msg->buffer) {
		uint8_t *out = msg->buffer;

		segment_base = (unsigned int)rx->edid_segment * 256;

		for (i = 0; i < msg->size; i++) {
			if (segment_base + rx->edid_offset < rx->local_edid_size)
				out[i] = rx->local_edid[segment_base + rx->edid_offset];
			else
				out[i] = 0x00;

			rx->edid_offset = (rx->edid_offset + 1) & 0xff;
		}
	} else {
		msg->reply = DP_AUX_I2C_REPLY_NACK;
		return -EIO;
	}

	msg->reply = DP_AUX_I2C_REPLY_ACK;
	return msg->size;
}

static inline struct soc_dp_virtual_rx *aux_to_rx(struct drm_dp_aux *aux)
{
	return &container_of(aux, struct soc_dp_dev, aux)->rx;
}

ssize_t soc_dp_virtual_rx_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	ssize_t ret = -EIO;
	struct soc_dp_virtual_rx *rx = aux_to_rx(aux);
	uint8_t base_req = msg->request & ~DP_AUX_I2C_MOT;

	mutex_lock(&rx->lock);

	if (base_req == DP_AUX_NATIVE_WRITE || base_req == DP_AUX_NATIVE_READ) {
		ret = aux_handle_native(rx, msg);
		goto out;
	}

	if (base_req == DP_AUX_I2C_WRITE ||
			base_req == DP_AUX_I2C_WRITE_STATUS_UPDATE || base_req == DP_AUX_I2C_READ) {
		if (msg->address == DDC_ADDR || msg->address == DDC_SEGMENT_ADDR) {
			ret = aux_handle_ddc(rx, msg);
			goto out;
		}

		msg->reply = DP_AUX_I2C_REPLY_NACK;
		ret = -EIO;
		goto out;
	}

	msg->reply = DP_AUX_NATIVE_REPLY_NACK;

out:
	mutex_unlock(&rx->lock);
	return ret;
}

static uint16_t soc_dp_mst_calc_pbn(uint8_t lane_count, uint8_t link_rate_dgbps)
{
	uint16_t pbn_per_lane;

	switch (link_rate_dgbps) {
	case 16:
		pbn_per_lane = 192; break;
	case 27:
		pbn_per_lane = 320; break;
	case 54:
		pbn_per_lane = 640; break;
	case 81:
		pbn_per_lane = 960; break;
	default:
		pbn_per_lane = 640; break;
	}

	return pbn_per_lane * (lane_count & 0xf);
}

static struct soc_dp_mst_dev *
mst_dev_alloc(enum soc_dp_mst_dev_type type, struct soc_dp_mst_dev *parent,
		int parent_port_number, const struct soc_dp_mst_dev_cfg *cfg)
{
	int i;
	const uint8_t *dpcd_template = (type == SOC_DP_MST_BRANCH) ?
		raw_dpcd_branch : raw_dpcd_endpoint;
	size_t dpcd_sz = (type == SOC_DP_MST_BRANCH) ?
		sizeof(raw_dpcd_branch) : sizeof(raw_dpcd_endpoint);
	struct soc_dp_mst_dev *dev;

	if (cfg->num_ports > DP_MAX_PORTS) {
		pr_err("Dev alloc failed, num_ports %d exceeds DP limit %d\n",
				cfg->num_ports, DP_MAX_PORTS);
		return NULL;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->type       = type;
	dev->name       = cfg->name;
	dev->num_ports  = cfg->num_ports;
	dev->parent     = parent;
	dev->parent_port_number = parent_port_number;
	dev->edid_size  = cfg->edid_size;
	if (cfg->edid && cfg->edid_size > 0) {
		dev->edid = kmemdup(cfg->edid, cfg->edid_size, GFP_KERNEL);
		if (!dev->edid) {
			kfree(dev);
			return NULL;
		}
	} else {
		dev->edid = NULL;
	}

	dev->dpcd_size       = dpcd_sz;
	dev->lane_count      = cfg->lane_count;
	dev->link_rate_dgbps = cfg->link_rate_dgbps;
	dev->link_full_pbn   = soc_dp_mst_calc_pbn(cfg->lane_count, cfg->link_rate_dgbps);
	dev->dpcd = kmemdup(dpcd_template, dpcd_sz, GFP_KERNEL);
	if (!dev->dpcd) {
		kfree(dev->edid);
		kfree(dev);
		return NULL;
	}

	dev->dpcd[DP_MAX_LINK_RATE]  = gbps_to_dpcd_bw(cfg->link_rate_dgbps);
	dev->dpcd[DP_MAX_LANE_COUNT] = (cfg->lane_count & 0xf)
		| DP_TPS3_SUPPORTED | DP_ENHANCED_FRAME_CAP;
	if (type == SOC_DP_MST_BRANCH)
		dev->dpcd[DP_DOWN_STREAM_PORT_COUNT] = cfg->num_ports & 0xff;

	get_random_bytes(dev->guid, DP_GUID_SIZE);

	if (dev->dpcd_size > DP_GUID + DP_GUID_SIZE)
		memcpy(dev->dpcd + DP_GUID, dev->guid, DP_GUID_SIZE);

	if (cfg->num_ports > 0) {
		dev->ports = kcalloc(cfg->num_ports, sizeof(*dev->ports), GFP_KERNEL);
		if (!dev->ports) {
			kfree(dev->dpcd);
			kfree(dev->edid);
			kfree(dev);
			return NULL;
		}
		for (i = 0; i < cfg->num_ports; i++) {
			dev->ports[i].port_number = i + 1;
			dev->ports[i].full_pbn = 0;
			dev->ports[i].avail_pbn = 0;
		}
	}

	if (parent) {
		int array_idx;

		if (parent_port_number == 0) {
			pr_err("Dev alloc failed, attempted to bind device '%s' to input port 0 of '%s'\n",
					cfg->name ? cfg->name : "unnamed", parent->name ? parent->name : "hub");
			kfree(dev->ports);
			kfree(dev->dpcd);
			kfree(dev->edid);
			kfree(dev);
			return NULL;
		}

		array_idx = parent_port_number - 1;
		if (array_idx < 0 || array_idx >= parent->num_ports) {
			pr_err("Dev alloc failed, parent_port_number %d out of bounds (max %d)\n",
					parent_port_number, parent->num_ports);
			kfree(dev->ports);
			kfree(dev->dpcd);
			kfree(dev->edid);
			kfree(dev);
			return NULL;
		}

		parent->ports[array_idx].connected = true;
		parent->ports[array_idx].device    = dev;

		parent->ports[array_idx].full_pbn  = dev->link_full_pbn;
		parent->ports[array_idx].avail_pbn = dev->link_full_pbn;
	}

	return dev;
}

struct soc_dp_mst_dev *soc_dp_mst_branch_create(struct soc_dp_mst_dev *parent,
		int parent_port_number, int num_downstream_ports,
		const char *name, uint8_t lane_count, uint8_t link_rate_dgbps)
{
	struct soc_dp_mst_dev_cfg cfg = {
		.name		= name,
		.num_ports	= num_downstream_ports,
		.lane_count	= lane_count,
		.link_rate_dgbps	= link_rate_dgbps,
	};

	return mst_dev_alloc(SOC_DP_MST_BRANCH, parent, parent_port_number, &cfg);
}

struct soc_dp_mst_dev *soc_dp_mst_endpoint_create(struct soc_dp_mst_dev *parent,
		int parent_port_number, const char *name,
		uint8_t *edid, size_t edid_size,
		uint8_t lane_count, uint8_t link_rate_dgbps)
{
	struct soc_dp_mst_dev_cfg cfg = {
		.name		= name,
		.num_ports	= 0,
		.lane_count	= lane_count,
		.link_rate_dgbps	= link_rate_dgbps,
		.edid		= edid,
		.edid_size	= edid_size,
	};

	return mst_dev_alloc(SOC_DP_MST_ENDPOINT, parent, parent_port_number, &cfg);
}

static int __soc_dp_mst_add_subtree_no_lock(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *tree_root, int target_port_id)
{
	int ret;
	int assigned_dev_id;
	bool is_root = false;
	struct soc_dp_mst_detached *det;

	soc_dp_mst_assign_ids(tree_root, &rx->mst_id_counter);
	assigned_dev_id = tree_root->id;

	if (!rx->mst_root) {
		rx->mst_root = tree_root;
		is_root = true;
	} else {
		det = kzalloc(sizeof(*det), GFP_KERNEL);
		if (!det) {
			soc_dp_mst_dev_free(tree_root);
			return -ENOMEM;
		}
		det->root = tree_root;
		det->next = rx->detached_subtrees;
		rx->detached_subtrees = det;
	}

	if (!is_root && target_port_id != 0) {
		ret = __soc_dp_mst_plug_device_no_lock(rx, assigned_dev_id, target_port_id);
		if (ret < 0) {
			rx->detached_subtrees = det->next;
			kfree(det);
			soc_dp_mst_dev_free(tree_root);
			pr_warn("Subtree %d auto-plug to port %d failed, err=%d, freed\n",
				assigned_dev_id, target_port_id, ret);
			return ret;
		}
	}

	return 0;
}

int soc_dp_mst_add_subtree(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *tree_root, int target_port_id)
{
	int ret;

	if (!rx || !tree_root)
		return -EINVAL;

	mutex_lock(&rx->lock);
	ret = __soc_dp_mst_add_subtree_no_lock(rx, tree_root, target_port_id);
	if (ret == 0)
		mst_update_sink_count_locked(rx);
	mutex_unlock(&rx->lock);

	return ret;
}

static struct soc_dp_mst_dev *
soc_dp_mst_subtree_clone_recursive(const struct soc_dp_mst_dev *src,
		struct soc_dp_mst_dev *parent, int parent_port_number)
{
	int i;
	struct soc_dp_mst_dev *dst;

	if (!src)
		return NULL;

	dst = kzalloc(sizeof(*dst), GFP_KERNEL);
	if (!dst)
		return NULL;

	dst->type            = src->type;
	dst->name            = src->name;
	dst->num_ports       = src->num_ports;
	dst->parent          = parent;
	dst->parent_port_number = parent_port_number;
	dst->lane_count      = src->lane_count;
	dst->link_rate_dgbps = src->link_rate_dgbps;
	dst->link_full_pbn   = src->link_full_pbn;
	dst->dpcd_size       = src->dpcd_size;
	dst->edid_size       = src->edid_size;
	if (src->edid && src->edid_size > 0) {
		dst->edid = kmemdup(src->edid, src->edid_size, GFP_KERNEL);
		if (!dst->edid)
			goto err_free;
	} else {
		dst->edid = NULL;
	}

	get_random_bytes(dst->guid, DP_GUID_SIZE);

	if (src->dpcd && src->dpcd_size > 0) {
		dst->dpcd = kmemdup(src->dpcd, src->dpcd_size, GFP_KERNEL);
		if (!dst->dpcd)
			goto err_edid;

		if (dst->dpcd_size > DP_GUID + DP_GUID_SIZE)
			memcpy(dst->dpcd + DP_GUID, dst->guid, DP_GUID_SIZE);
	}

	if (src->num_ports > 0) {
		dst->ports = kcalloc(src->num_ports, sizeof(*dst->ports), GFP_KERNEL);
		if (!dst->ports)
			goto err_dpcd;

		for (i = 0; i < src->num_ports; i++) {
			dst->ports[i].port_number  = src->ports[i].port_number;
			dst->ports[i].full_pbn  = src->ports[i].full_pbn;
			dst->ports[i].avail_pbn = src->ports[i].full_pbn;

			if (src->ports[i].connected && src->ports[i].device) {
				dst->ports[i].connected = true;
				dst->ports[i].device = soc_dp_mst_subtree_clone_recursive(
					src->ports[i].device, dst,
					dst->ports[i].port_number);
				if (!dst->ports[i].device)
					goto err_ports;
			}
		}
	}

	return dst;

err_ports:

	while (--i >= 0) {
		if (dst->ports[i].device)
			soc_dp_mst_dev_free(dst->ports[i].device);
	}
	kfree(dst->ports);
err_dpcd:
	kfree(dst->dpcd);
err_edid:
	kfree(dst->edid);
err_free:
	kfree(dst);
	return NULL;
}

static int __soc_dp_mst_clone_subtree_no_lock(struct soc_dp_virtual_rx *rx,
		int root_device_id, int target_port_id)
{
	int ret;
	struct soc_dp_mst_find_result result;
	struct soc_dp_mst_dev *src_dev;
	struct soc_dp_mst_dev *cloned_root;
	struct soc_dp_mst_find_result port_res;

	result = soc_dp_mst_find_by_id(rx, root_device_id);
	if (result.type != SOC_DP_MST_FIND_DEVICE)
		return -ENOENT;
	src_dev = result.dev;

	if (target_port_id != 0) {
		port_res = soc_dp_mst_find_by_id(rx, target_port_id);
		if (port_res.type != SOC_DP_MST_FIND_PORT)
			return -EINVAL;
		if (!port_res.owner || port_res.owner->type != SOC_DP_MST_BRANCH)
			return -EINVAL;
		if (port_res.port->connected || port_res.port->device)
			return -EBUSY;
	}

	cloned_root = soc_dp_mst_subtree_clone_recursive(src_dev, NULL, 0);
	if (!cloned_root)
		return -ENOMEM;

	ret = __soc_dp_mst_add_subtree_no_lock(rx, cloned_root, target_port_id);
	if (ret < 0)
		return ret;

	return cloned_root->id;
}

int soc_dp_mst_clone_subtree(struct soc_dp_virtual_rx *rx, int root_device_id, int target_port_id)
{
	int ret;

	if (!rx)
		return -EINVAL;

	mutex_lock(&rx->lock);
	ret = __soc_dp_mst_clone_subtree_no_lock(rx, root_device_id, target_port_id);
	if (ret >= 0)
		mst_update_sink_count_locked(rx);
	mutex_unlock(&rx->lock);

	return ret;
}

static int __soc_dp_mst_delete_subtree_no_lock(struct soc_dp_virtual_rx *rx, int root_device_id)
{
	int ret;
	int port_number;
	bool in_active;
	uint16_t consumed_pbn;

	struct soc_dp_mst_find_result result;
	struct soc_dp_mst_dev *dev;
	struct soc_dp_mst_dev *parent;
	struct soc_dp_mst_dev *up_node;
	struct soc_dp_mst_port *up_port;
	struct soc_dp_mst_detached *det, *prev;

	result = soc_dp_mst_find_by_id(rx, root_device_id);
	if (result.type == SOC_DP_MST_FIND_NONE)
		return -ENOENT;
	if (result.type == SOC_DP_MST_FIND_PORT)
		return -EINVAL;
	dev = result.dev;

	if (dev == rx->mst_root)
		return -EPERM;

	if (dev->parent) {
		parent = dev->parent;
		port_number = dev->parent_port_number - 1;

		in_active = soc_dp_mst_is_in_active_tree(rx, dev);
		consumed_pbn = parent->ports[port_number].full_pbn - parent->ports[port_number].avail_pbn;

		if (in_active) {
			ret = __soc_dp_mst_notify_conn_stat_locked(rx, rx->mst_root, root_device_id, false);
			if (ret < 0)
				pr_warn_ratelimited("CSN queue full, dropped delete notify for device %d (topology change proceeds)\n",
						root_device_id);
		}

		parent->ports[port_number].connected = false;
		parent->ports[port_number].device = NULL;

		dev->parent = NULL;
		dev->parent_port_number = -1;

		up_node = parent;
		while (up_node && up_node->parent) {
			up_port = &up_node->parent->ports[up_node->parent_port_number - 1];
			up_port->avail_pbn += consumed_pbn;
			up_node = up_node->parent;
		}

		parent->ports[port_number].full_pbn = 0;
		parent->ports[port_number].avail_pbn = 0;
		memset(parent->ports[port_number].payloads, 0,
				sizeof(parent->ports[port_number].payloads));
		mst_reset_payloads_recursive(dev);

	} else {
		prev = NULL;
		for (det = rx->detached_subtrees; det; prev = det, det = det->next) {
			if (det->root == dev)
				break;
		}

		if (det) {
			if (prev)
				prev->next = det->next;
			else
				rx->detached_subtrees = det->next;
			kfree(det);
		}
	}

	soc_dp_mst_dev_free(dev);
	return 0;
}

int soc_dp_mst_delete_subtree(struct soc_dp_virtual_rx *rx, int root_device_id)
{
	int ret;

	if (!rx || !rx->mst_root)
		return -EINVAL;

	mutex_lock(&rx->lock);
	ret = __soc_dp_mst_delete_subtree_no_lock(rx, root_device_id);
	if (ret == 0)
		mst_update_sink_count_locked(rx);
	mutex_unlock(&rx->lock);

	return ret;
}

static void soc_dp_mst_dev_free(struct soc_dp_mst_dev *dev)
{
	int i;

	if (!dev)
		return;

	if (dev->ports) {
		for (i = 0; i < dev->num_ports; i++)
			soc_dp_mst_dev_free(dev->ports[i].device);
		kfree(dev->ports);
	}

	kfree(dev->dpcd);
	kfree(dev->edid);
	kfree(dev);
}

#define DP_TOPOLOGY_DUMP_BUF_SIZE 256
void soc_dp_mst_topology_dump(const struct soc_dp_mst_dev *dev, const char *prefix, int pbn_div)
{
	int i, j, last, pos;
	char *child_prefix = NULL;
	char *dev_prefix = NULL;
	char *payload_str = NULL;
	int effective_div = (pbn_div > 0) ? pbn_div : 54;
	const char *marker = (prefix && prefix[0] != '\0') ? "\xE2\x94\x94-- " : "";

	if (!dev)
		return;

	if (dev->type == SOC_DP_MST_BRANCH) {
		pr_info("%s%s[Branch Device ID:%d]: %s (%d ports) [Up Link: %d lane%s, %d.%d Gbps]\n",
			prefix, marker, dev->id, dev->name ? dev->name : "hub", dev->num_ports,
			dev->lane_count, dev->lane_count > 1 ? "s" : "", dev->link_rate_dgbps / 10,
			dev->link_rate_dgbps == 16 ? 62 : dev->link_rate_dgbps % 10);
	} else {
		pr_info("%s%s[Endpoint Device ID:%d]: %s [Up Link: %d lane%s, %d.%d Gbps]\n",
			prefix, marker, dev->id, dev->name ? dev->name : "sink",
			dev->lane_count, dev->lane_count > 1 ? "s" : "", dev->link_rate_dgbps / 10,
			dev->link_rate_dgbps == 16 ? 62 : dev->link_rate_dgbps % 10);
	}

	if (!dev->ports)
		return;

	child_prefix = kzalloc(DP_TOPOLOGY_DUMP_BUF_SIZE, GFP_KERNEL);
	dev_prefix   = kzalloc(DP_TOPOLOGY_DUMP_BUF_SIZE, GFP_KERNEL);
	payload_str  = kzalloc(DP_TOPOLOGY_DUMP_BUF_SIZE, GFP_KERNEL);

	if (!child_prefix || !dev_prefix || !payload_str) {
		pr_err("OOM during topology dump\n");
		goto out;
	}

	for (i = 0; i < dev->num_ports; i++) {
		const struct soc_dp_mst_port *p = &dev->ports[i];

		last = (i == dev->num_ports - 1);

		snprintf(child_prefix, DP_TOPOLOGY_DUMP_BUF_SIZE, "%s    ", prefix);

		if (p->connected) {
			pos = 0;
			payload_str[0] = '\0';
			for (j = 0; j < DP_PAYLOAD_SLOT_MAX; j++) {
				if (p->payloads[j].vcpi > 0) {
					int slots = (p->payloads[j].pbn + effective_div - 1) / effective_div;
					int remain = DP_TOPOLOGY_DUMP_BUF_SIZE - pos;

					if (remain > 0) {
						pos += scnprintf(payload_str + pos, remain,
								" | VCPI:%d PBN:%d (%d slots)",
								p->payloads[j].vcpi, p->payloads[j].pbn, slots);
					}
				}
			}

			pr_info("%s%s-- [Port %d ID:%d]: [Connected] -> PBN: %d/%d avail%s\n",
				child_prefix, last ? "\xE2\x94\x94" : "\xE2\x94\x9C",
				p->port_number, p->port_id, p->avail_pbn, p->full_pbn, payload_str);

		} else {
			pr_info("%s%s-- [Port %d ID:%d]: [Dangling/Empty]\n",
				child_prefix, last ? "\xE2\x94\x94" : "\xE2\x94\x9C", p->port_number, p->port_id);
		}

		if (p->connected && p->device) {
			snprintf(dev_prefix, DP_TOPOLOGY_DUMP_BUF_SIZE, "%s%s   ",
					child_prefix, last ? " " : "\xE2\x94\x82");
			soc_dp_mst_topology_dump(p->device, dev_prefix, pbn_div);
		}
	}

out:
	kfree(child_prefix);
	kfree(dev_prefix);
	kfree(payload_str);
}

void soc_dp_mst_topology_dump_all(struct soc_dp_virtual_rx *rx, int pbn_div)
{
	struct soc_dp_mst_detached *det;

	if (!(rx->rx_caps[DP_MSTM_CAP] & DP_MST_CAP)) {
		pr_info("Topology dump only available in MST mode\n");
		return;
	}

	mutex_lock(&rx->lock);

	soc_dp_mst_topology_dump(rx->mst_root, "", pbn_div);

	for (det = rx->detached_subtrees; det; det = det->next) {
		pr_info("\n");
		soc_dp_mst_topology_dump(det->root, "", pbn_div);
	}

	mutex_unlock(&rx->lock);
}

static uint8_t soc_dp_msg_header_crc4(const uint8_t *data, size_t num_nibbles)
{
	uint8_t bitmask = 0x80;
	uint8_t bitshift = 7;
	uint8_t array_index = 0;
	uint8_t remainder = 0;
	int number_of_bits = num_nibbles * 4;

	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		remainder |= (data[array_index] & bitmask) >> bitshift;
		bitmask >>= 1;
		bitshift--;
		if (bitmask == 0) {
			bitmask = 0x80;
			bitshift = 7;
			array_index++;
		}
		if ((remainder & 0x10) == 0x10)
			remainder ^= 0x13;
	}

	number_of_bits = 4;
	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		if ((remainder & 0x10) != 0)
			remainder ^= 0x13;
	}

	return remainder & 0xf;
}

static uint8_t soc_dp_msg_data_crc4(const uint8_t *data, uint8_t number_of_bytes)
{
	uint8_t bitmask = 0x80;
	uint8_t bitshift = 7;
	uint8_t array_index = 0;
	uint16_t remainder = 0;
	int number_of_bits = number_of_bytes * 8;

	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		remainder |= (data[array_index] & bitmask) >> bitshift;
		bitmask >>= 1;
		bitshift--;
		if (bitmask == 0) {
			bitmask = 0x80;
			bitshift = 7;
			array_index++;
		}
		if ((remainder & 0x100) == 0x100)
			remainder ^= 0xd5;
	}

	number_of_bits = 8;
	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		if ((remainder & 0x100) != 0)
			remainder ^= 0xd5;
	}

	return remainder & 0xff;
}

static int mst_hdr_parse(const uint8_t *buf, size_t len, struct soc_dp_mst_hdr *hdr)
{
	uint8_t idx;
	int hdr_size, i;

	if (len < 4)
		return -EINVAL;

	if (buf[0] == 0)
		return -EINVAL;

	memset(hdr, 0, sizeof(*hdr));
	hdr->lct = (buf[0] & 0xf0) >> 4;
	hdr->lcr = buf[0] & 0xf;

	hdr_size = 3;
	hdr_size += (hdr->lct / 2);
	if (len < (size_t)hdr_size)
		return -EINVAL;

	if (soc_dp_msg_header_crc4(buf, (hdr_size * 2) - 1)
			!= (buf[hdr_size - 1] & 0xf))
		return -EIO;

	idx = 1;
	for (i = 0; i < (hdr->lct / 2); i++)
		hdr->rad[i] = buf[idx++];

	hdr->broadcast = (buf[idx] >> 7) & 0x1;
	hdr->path_msg  = (buf[idx] >> 6) & 0x1;
	hdr->msg_len   = buf[idx] & 0x3f;
	idx++;

	hdr->somt  = (buf[idx] >> 7) & 0x1;
	hdr->eomt  = (buf[idx] >> 6) & 0x1;
	hdr->seqno = (buf[idx] >> 4) & 0x1;
	idx++;

	if (len > (size_t)hdr_size)
		hdr->msg_type = buf[hdr_size] & 0x7f;

	return 0;
}

static struct soc_dp_mst_dev *mst_route(struct soc_dp_mst_dev *root,
		const struct soc_dp_mst_hdr *hdr)
{
	struct soc_dp_mst_dev *dev = root;
	int i, port, idx;

	if (hdr->broadcast)
		return root;

	for (i = 0; i < hdr->lcr; i++) {
		port = (i % 2 == 0) ? (hdr->rad[i / 2] >> 4) & 0xf : hdr->rad[i / 2] & 0xf;
		idx = port - 1;
		if (!dev || dev->type != SOC_DP_MST_BRANCH || idx < 0
				|| idx >= dev->num_ports || !dev->ports[idx].connected)
			return NULL;
		dev = dev->ports[idx].device;
	}

	return dev;
}

static int mst_build_reply_hdr(uint8_t *hdr_buf, uint8_t lct, uint8_t lcr, const uint8_t *rad,
		uint8_t msg_len, uint8_t seqno, bool somt, bool eomt, bool path_msg)
{
	int i;
	int rad_bytes = lct / 2;
	int idx = 0;
	uint8_t crc4;

	hdr_buf[idx++] = ((lct & 0xf) << 4) | (lcr & 0xf);

	for (i = 0; i < rad_bytes; i++)
		hdr_buf[idx++] = rad ? rad[i] : 0;

	hdr_buf[idx++] = (0 << 7) | (path_msg << 6) | (msg_len & 0x3f);

	hdr_buf[idx++] = (somt << 7) | (eomt << 6) | ((seqno & 0x1) << 4);

	crc4 = soc_dp_msg_header_crc4(hdr_buf, (idx * 2) - 1);
	hdr_buf[idx - 1] |= (crc4 & 0xf);

	return idx;
}

static uint8_t mst_req_port(uint8_t msg_type, const uint8_t *body, size_t len)
{
	if (len < 2)
		return 0;

	return (body[1] >> 4) & 0xf;
}

static uint8_t *mst_put_u16(uint8_t *p, uint16_t val)
{
	*p++ = val >> 8;
	*p++ = val & 0xff;
	return p;
}

static const uint8_t zero_guid[DP_GUID_SIZE] = {0};

static uint8_t *mst_build_nak(uint8_t *p, uint8_t reason)
{
	memcpy(p, zero_guid, DP_GUID_SIZE); p += DP_GUID_SIZE;
	*p++ = reason;
	*p++ = 0x00;
	return p;
}

static uint8_t *mst_handle_link_address(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t body_len)
{
	int i;

	memcpy(p, dev ? dev->guid : zero_guid, DP_GUID_SIZE); p += DP_GUID_SIZE;

	if (!dev || dev->type != SOC_DP_MST_BRANCH) {
		*p++ = 0x00;
		return p;
	}

	*p++ = (dev->num_ports + 1) & 0xf;
	*p++ = 0x80 | (0 << 4) | 0;
	*p++ = DP_PORT_CAP_MCS;

	for (i = 0; i < dev->num_ports; i++) {
		struct soc_dp_mst_port *port = &dev->ports[i];
		uint8_t peer_type, caps_byte;
		bool is_branch;

		if (!port->connected) {
			*p++ = (0 << 7) | (port->port_number & 0xf);
			*p++ = 0x00;
			*p++ = 0x00;
			memset(p, 0, DP_GUID_SIZE); p += DP_GUID_SIZE;
			*p++ = 0x00;
			continue;
		}

		is_branch = (port->device->type == SOC_DP_MST_BRANCH);
		peer_type = is_branch ? DP_PEER_DEVICE_MST_BRANCHING : DP_PEER_DEVICE_SST_SINK;

		*p++ = ((peer_type & 0x7) << 4) | (port->port_number & 0xf);

		caps_byte = DP_PORT_CAP_DDPS;
		if (is_branch)
			caps_byte |= DP_PORT_CAP_MCS;
		else
			caps_byte |= DP_PORT_CAP_LDPS;
		*p++ = caps_byte;

		*p++ = DP_DPCD_REV_14;

		memcpy(p, port->device->guid, DP_GUID_SIZE); p += DP_GUID_SIZE;

		if (is_branch)
			*p++ = 0x00;
		else
			*p++ = (DP_LINK_ADDR_SST_SDP_COUNT << 4) | DP_LINK_ADDR_SST_SDP_SINKS;
	}

	return p;
}

static uint8_t *mst_handle_enum_path_resources(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t body_len)
{
	int idx;
	uint8_t port_number = (body_len >= 2) ? (body[1] >> 4) & 0xf : 0;
	uint16_t full_pbn = 0, avail_pbn = 0;
	struct soc_dp_mst_port *port = NULL;

	if (port_number > 0) {
		idx = port_number - 1;
		if (dev && dev->ports && idx < dev->num_ports)
			port = &dev->ports[idx];
	}

	if (port) {
		full_pbn  = port->full_pbn;
		avail_pbn = port->avail_pbn;
	}

	*p++ = (port_number & 0xf) << 4;
	p = mst_put_u16(p, full_pbn);
	p = mst_put_u16(p, avail_pbn);
	return p;
}

static uint8_t *mst_handle_remote_dpcd_read(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t body_len)
{
	uint8_t port = mst_req_port(req_type, body, body_len);
	uint8_t num_bytes = 0;
	uint32_t offset = 0;

	if (body_len >= 5) {
		offset    = ((body[1] & 0xf) << 16) | (body[2] << 8) | body[3];
		num_bytes = body[4];
	}

	*p++ = port & 0xf;

	if (dev && dev->dpcd && dev->dpcd_size > 0) {
		if (num_bytes > 0 && offset < dev->dpcd_size) {
			uint8_t copy_len = min_t(size_t, num_bytes, dev->dpcd_size - offset);
			*p++ = copy_len;
			memcpy(p, dev->dpcd + offset, copy_len);
			p += copy_len;
		} else {
			*p++ = 0x00;
		}
	} else {
		const uint8_t *dpcd = raw_dpcd_endpoint;
		size_t dpcd_sz = sizeof(raw_dpcd_endpoint);

		if (num_bytes > 0 && offset < dpcd_sz) {
			uint8_t copy_len = min_t(size_t, num_bytes, dpcd_sz - offset);
			*p++ = copy_len;
			memcpy(p, dpcd + offset, copy_len);
			p += copy_len;
		} else {
			*p++ = 0x00;
		}
	}

	return p;
}

static uint8_t *mst_handle_remote_dpcd_write(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	uint8_t port_number;
	uint8_t num_bytes;
	uint8_t copy_len;
	uint32_t offset;

	port_number = mst_req_port(req_type, body, len);
	*p++ = port_number & 0xf;

	if (dev && dev->dpcd && dev->dpcd_size > 0 && len >= 5) {
		offset    = ((body[1] & 0xf) << 16) | (body[2] << 8) | body[3];
		num_bytes = body[4];

		if (len < 5 + num_bytes)
			num_bytes = len - 5;

		if (num_bytes > 0 && offset < dev->dpcd_size) {
			copy_len = min_t(size_t, num_bytes, dev->dpcd_size - offset);
			memcpy(dev->dpcd + offset, &body[5], copy_len);

				if (offset < DP_GUID + DP_GUID_SIZE && offset + copy_len > DP_GUID) {
					unsigned int guid_start
						= max_t(unsigned int, offset, DP_GUID);
					unsigned int guid_end
						= min_t(unsigned int, offset + copy_len, DP_GUID + DP_GUID_SIZE);
					memcpy(dev->guid + (guid_start - DP_GUID),
							&body[5 + (guid_start - offset)], guid_end - guid_start);
				}
		}
	}

	return p;
}

static uint8_t *mst_handle_remote_i2c_read(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	int i, pos = 2;
	uint8_t port = mst_req_port(req_type, body, len);
	uint8_t nt = (len >= 2) ? (body[1] & 0x3) : 0;

	uint8_t hw_offset = dev->edid_offset;
	uint8_t hw_segment = dev->edid_segment;

	uint8_t req_bytes;
	size_t edid_sz = dev->edid ? dev->edid_size : sizeof(raw_edid);
	const uint8_t *edid = dev->edid ? dev->edid : raw_edid;
	unsigned int segment_base;

	if (!dev || dev->type != SOC_DP_MST_ENDPOINT)
		return mst_build_nak(p, DP_NAK_WRITE_FAILURE);

	for (i = 0; i < nt && pos + 1 < (int)len; i++) {
		uint8_t addr = body[pos++] & 0x7f;
		uint8_t wlen = body[pos++];

		if (pos + wlen > (int)len)
			break;

		if (addr == DDC_ADDR && wlen >= 1)
			hw_offset = body[pos];
		else if (addr == DDC_SEGMENT_ADDR && wlen >= 1)
			hw_segment = body[pos];

		pos += wlen;
		if (pos < (int)len)
			pos++;
	}

	if (pos < (int)len)
		pos++;
	req_bytes = (pos < (int)len) ? body[pos] : 0;

	*p++ = port & 0xf;
	*p++ = req_bytes;

	if (req_bytes > 0) {
		segment_base = (unsigned int)hw_segment * 256;

		for (i = 0; i < req_bytes; i++) {
			if (segment_base + hw_offset < edid_sz)
				*p++ = edid[segment_base + hw_offset];
			else
				*p++ = 0x00;

			hw_offset = (hw_offset + 1) & 0xff;
		}

		dev->edid_offset = hw_offset;
		dev->edid_segment = hw_segment;
	}

	return p;
}

static uint8_t *mst_handle_remote_i2c_write(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	if (!dev || dev->type != SOC_DP_MST_ENDPOINT)
		return mst_build_nak(p, DP_NAK_WRITE_FAILURE);
	*p++ = mst_req_port(req_type, body, len) & 0xf;
	return p;
}

static uint8_t *mst_handle_allocate_payload(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	int i, idx = -1;
	uint8_t port_number;
	uint8_t vcpi;
	uint8_t number_sdp_streams;
	uint16_t allocated_pbn = 0;
	uint16_t req_pbn, old_pbn = 0;
	size_t expected_len;
	struct soc_dp_mst_port *port;
	struct soc_dp_mst_dev *node;

	if (len < 5)
		return mst_build_nak(p, DP_NAK_WRITE_FAILURE);

	port_number           = (body[1] >> 4) & 0xf;
	number_sdp_streams = body[1] & 0xf;
	vcpi               = body[2] & 0x7f;
	req_pbn            = (body[3] << 8) | body[4];

	expected_len = 5 + (number_sdp_streams + 1) / 2;
	if (len < expected_len)
		return mst_build_nak(p, DP_NAK_WRITE_FAILURE);

	port = NULL;
	if (port_number > 0) {
		int port_idx = port_number - 1;

		if (dev && dev->ports && port_idx < dev->num_ports)
			port = &dev->ports[port_idx];
	}

	if (port && vcpi > 0) {
		for (i = 0; i < DP_PAYLOAD_SLOT_MAX; i++) {
			if (port->payloads[i].vcpi == vcpi) {
				old_pbn = port->payloads[i].pbn;
				idx = i;
				break;
			}
			if (port->payloads[i].vcpi == 0 && idx == -1)
				idx = i;
		}

		if (req_pbn == 0) {
			if (old_pbn > 0 && idx != -1) {
				port->avail_pbn += old_pbn;
				port->payloads[idx].vcpi = 0;
				port->payloads[idx].pbn = 0;

				node = dev;
				while (node->parent) {
					node->parent->ports[node->parent_port_number - 1].avail_pbn += old_pbn;
					node = node->parent;
				}
			}
		} else {
			bool cascade_ok = (idx != -1);

			if (cascade_ok && port->avail_pbn + old_pbn < req_pbn)
				cascade_ok = false;

			node = dev;
			while (node->parent && cascade_ok) {
				struct soc_dp_mst_port *up_port =
					&node->parent->ports[node->parent_port_number - 1];
				if (up_port->avail_pbn + old_pbn < req_pbn) {
					cascade_ok = false;
					break;
				}
				node = node->parent;
			}

			if (cascade_ok) {
				port->avail_pbn = (port->avail_pbn + old_pbn) - req_pbn;
				port->payloads[idx].vcpi = vcpi;
				port->payloads[idx].pbn = req_pbn;
				allocated_pbn = req_pbn;

				node = dev;
				while (node->parent) {
					struct soc_dp_mst_port *up_port =
						&node->parent->ports[node->parent_port_number - 1];
					up_port->avail_pbn =
						(up_port->avail_pbn + old_pbn) - req_pbn;
					node = node->parent;
				}
			} else {
				allocated_pbn = 0;
			}
		}
	}

	*p++ = port_number << 4;
	*p++ = vcpi;
	return mst_put_u16(p, allocated_pbn);
}

static uint8_t *mst_handle_query_payload(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	int i, p_idx;
	uint8_t port_number = (len >= 2) ? (body[1] >> 4) & 0xf : 0;
	uint8_t vcpi = (len >= 3) ? (body[2] & 0x7f) : 0;
	uint16_t allocated_pbn = 0;
	struct soc_dp_mst_port *port = NULL;

	if (port_number > 0) {
		p_idx = port_number - 1;
		if (dev && dev->ports && p_idx < dev->num_ports)
			port = &dev->ports[p_idx];
	}

	if (port && vcpi > 0) {
		for (i = 0; i < DP_PAYLOAD_SLOT_MAX; i++) {
			if (port->payloads[i].vcpi == vcpi) {
				allocated_pbn = port->payloads[i].pbn;
				break;
			}
		}
	}

	*p++ = port_number << 4;
	*p++ = vcpi;

	return mst_put_u16(p, allocated_pbn);
}

static uint8_t *mst_handle_power_phy(struct soc_dp_mst_dev *dev, uint8_t req_type,
		uint8_t *p, struct soc_dp_virtual_rx *rx, const uint8_t *body, size_t len)
{
	*p++ = (mst_req_port(req_type, body, len) & 0xf) << 4;
	return p;
}

static void mst_reset_payloads_recursive(struct soc_dp_mst_dev *dev)
{
	int i, j;

	if (!dev || !dev->ports)
		return;

	for (i = 0; i < dev->num_ports; i++) {
		dev->ports[i].avail_pbn = dev->ports[i].full_pbn;
		for (j = 0; j < DP_PAYLOAD_SLOT_MAX; j++) {
			dev->ports[i].payloads[j].vcpi = 0;
			dev->ports[i].payloads[j].pbn  = 0;
		}
		if (dev->ports[i].device)
			mst_reset_payloads_recursive(dev->ports[i].device);
	}
}

static uint8_t *mst_handle_clear_payload_id_table(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	if (rx->mst_root)
		mst_reset_payloads_recursive(rx->mst_root);

	return p;
}

static uint8_t *mst_handle_query_stream_enc_status(struct soc_dp_mst_dev *dev,
		uint8_t req_type, uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len)
{
	uint8_t stream_id = (len >= 2) ? body[1] : 0;

	*p++ = 0;
	*p++ = 0;
	*p++ = stream_id;
	return p;
}

typedef uint8_t *(*mst_msg_handler_fn)(struct soc_dp_mst_dev *dev, uint8_t req_type,
		uint8_t *p, struct soc_dp_virtual_rx *rx,
		const uint8_t *body, size_t len);

static mst_msg_handler_fn mst_lookup_handler(uint8_t msg_type)
{
	switch (msg_type) {
	case DP_LINK_ADDRESS: return mst_handle_link_address;
	case DP_ENUM_PATH_RESOURCES: return mst_handle_enum_path_resources;
	case DP_REMOTE_DPCD_READ: return mst_handle_remote_dpcd_read;
	case DP_REMOTE_DPCD_WRITE: return mst_handle_remote_dpcd_write;
	case DP_REMOTE_I2C_READ: return mst_handle_remote_i2c_read;
	case DP_REMOTE_I2C_WRITE: return mst_handle_remote_i2c_write;
	case DP_CLEAR_PAYLOAD_ID_TABLE: return mst_handle_clear_payload_id_table;
	case DP_POWER_UP_PHY:
	case DP_POWER_DOWN_PHY: return mst_handle_power_phy;
	case DP_QUERY_PAYLOAD: return mst_handle_query_payload;
	case DP_ALLOCATE_PAYLOAD: return mst_handle_allocate_payload;
	case DP_QUERY_STREAM_ENC_STATUS: return mst_handle_query_stream_enc_status;
	default: return NULL;
	}
}

static void mst_deliver_next_chunk(struct soc_dp_virtual_rx *rx)
{
	bool eomt;
	bool somt = (rx->mst_pending_sent == 0);
	bool path_msg = rx->mst_pending_body[10];
	int hdr_len;
	uint8_t chunk[128];
	uint8_t *dst = chunk;
	uint8_t lct = rx->mst_pending_body[0];
	uint8_t lcr;
	size_t data_len;
	const uint8_t *rad = &rx->mst_pending_body[1];
	size_t remaining = rx->mst_pending_body_len - rx->mst_pending_sent;

	lcr = (lct > 0) ? lct - 1 : 0;

	hdr_len = 3 + (lct / 2);

	data_len = min_t(size_t, remaining, DP_SIDEBAND_CHUNK_MAX - 1 - hdr_len);
	eomt = (rx->mst_pending_sent + data_len >= rx->mst_pending_body_len);

	hdr_len = mst_build_reply_hdr(dst, lct, lcr, rad,
			data_len + 1, rx->mst_pending_seqno, somt, eomt, path_msg);
	dst += hdr_len;

	memcpy(dst, rx->mst_pending_body + 16 + rx->mst_pending_sent, data_len);
	dst += data_len;

	*dst++ = soc_dp_msg_data_crc4(chunk + hdr_len, data_len);

	memcpy(rx->mst_down_rep, chunk, min_t(size_t, dst - chunk, sizeof(rx->mst_down_rep)));
	rx->mst_down_rep_len = dst - chunk;
	rx->mst_pending_sent += data_len;

	rx->esi[1] |= DP_DOWN_REP_MSG_RDY;
	schedule_work(&rx->up_req_work);

	if (eomt) {
		kfree(rx->mst_pending_body);
		rx->mst_pending_body = NULL;
		rx->mst_pending_body_len = 0;
		rx->mst_pending_sent = 0;
	}
}

static ssize_t aux_handle_mst_write(struct soc_dp_virtual_rx *rx, const uint8_t *data, size_t len)
{
	bool nak = false;
	int hdr_size, body_len;
	uint8_t port_number;
	uint8_t msg_type;
	uint8_t *reply_buf, *p;
	uint8_t received_crc, calc_crc;
	size_t chunk_payload_len;
	struct soc_dp_mst_hdr hdr;
	struct soc_dp_mst_dev *dev;

	if (mst_hdr_parse(data, len, &hdr) < 0)
		return -EIO;

	hdr_size = 3 + hdr.lct / 2;

	if (len < (size_t)hdr_size + 1) {
		pr_err("DOWN_REQ chunk too small (%zu)\n", len);
		return -EIO;
	}

	chunk_payload_len = len - hdr_size - 1;

	received_crc = data[len - 1];
	calc_crc = soc_dp_msg_data_crc4(data + hdr_size, chunk_payload_len);
	if (received_crc != calc_crc) {
		pr_err("DOWN_REQ chunk CRC mismatch (recv 0x%02x, calc 0x%02x)\n",
				received_crc, calc_crc);
		return -EIO;
	}

	if (hdr.somt) {
		rx->down_req_len = 0;
		rx->down_req_seqno = hdr.seqno;
	}

	if (!hdr.somt && rx->down_req_len == 0) {
		pr_err("DOWN_REQ orphan chunk, dropping\n");
		return -EIO;
	}

	if (!hdr.somt && hdr.seqno != rx->down_req_seqno) {
		pr_err("DOWN_REQ seqno mismatch, aborting assembly\n");
		rx->down_req_len = 0;
		return -EIO;
	}

	if (rx->down_req_len + chunk_payload_len > sizeof(rx->down_req_buf)) {
		pr_err("DOWN_REQ buffer overflow, dropping transaction\n");
		rx->down_req_len = 0;
		return -EIO;
	}

	if (chunk_payload_len > 0) {
		memcpy(rx->down_req_buf + rx->down_req_len,
				data + hdr_size, chunk_payload_len);
		rx->down_req_len += chunk_payload_len;
	}

	if (!hdr.eomt)
		return len;

	if (rx->down_req_len == 0)
		return len;

	if (rx->down_req_buf[0] & 0x80) {
		rx->down_req_len = 0;
		return len;
	}

	msg_type = rx->down_req_buf[0] & 0x7f;

	dev = mst_route(rx->mst_root, &hdr);

	if (msg_type >= DP_REMOTE_DPCD_READ
			&& msg_type <= DP_REMOTE_I2C_WRITE) {
		port_number = mst_req_port(msg_type,
				rx->down_req_buf, rx->down_req_len);
		if (port_number == 0) {
			dev = NULL;
			nak = true;
		} else if (dev && dev->ports) {
			int idx = port_number - 1;

			if (idx < dev->num_ports && dev->ports[idx].connected) {
				dev = dev->ports[idx].device;
			} else {
				dev = NULL;
				nak = true;
			}
		}
	}

	rx->mst_down_rep_len = 0;

	reply_buf = kzalloc(MST_REPLY_MAX_LEN, GFP_KERNEL);
	if (!reply_buf)
		return -ENOMEM;

	if ((msg_type == DP_REMOTE_I2C_READ || msg_type == DP_REMOTE_I2C_WRITE)
			&& (!dev || dev->type != SOC_DP_MST_ENDPOINT))
		nak = true;

	reply_buf[0] = (nak ? DP_SIDEBAND_REPLY_NAK : DP_SIDEBAND_REPLY_ACK) << 7
		| (msg_type & 0x7f);

	if (nak) {
		uint8_t reason = DP_NAK_WRITE_FAILURE;

		if (msg_type == DP_REMOTE_I2C_READ || msg_type == DP_REMOTE_I2C_WRITE)
			reason = DP_NAK_I2C_NAK;
		p = mst_build_nak(&reply_buf[1], reason);
	} else {
		mst_msg_handler_fn handler = mst_lookup_handler(msg_type);

		if (!handler) {
			p = mst_build_nak(&reply_buf[1], DP_NAK_BAD_PARAM);
			reply_buf[0] = (DP_SIDEBAND_REPLY_NAK << 7) | (msg_type & 0x7f);
		} else {
			p = handler(dev, msg_type, &reply_buf[1],
					rx, rx->down_req_buf, rx->down_req_len);
		}
	}
	body_len = p - reply_buf;

	kfree(rx->mst_pending_body);

	rx->mst_pending_body = kmalloc(body_len + 16, GFP_KERNEL);
	if (!rx->mst_pending_body) {
		kfree(reply_buf);
		rx->mst_pending_body_len = 0;
		rx->mst_pending_sent = 0;
		return -ENOMEM;
	}
	memset(rx->mst_pending_body, 0, 16);

	rx->mst_pending_body[0] = hdr.lct;
	rx->mst_pending_body[9] = hdr.lcr;
	rx->mst_pending_body[10] = hdr.path_msg;
	memcpy(&rx->mst_pending_body[1], hdr.rad, DP_RAD_SIZE);
	memcpy(&rx->mst_pending_body[16], reply_buf, body_len);
	kfree(reply_buf);

	rx->mst_pending_body_len = body_len;
	rx->mst_pending_sent = 0;
	rx->mst_pending_seqno = hdr.seqno;

	mst_deliver_next_chunk(rx);
	rx->down_req_len = 0;
	return len;
}

static ssize_t aux_handle_mst(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len,
		bool write, unsigned int base)
{
	size_t q_len, payload_len;
	uint8_t received_crc, calc_crc;

	if (write && buf && len > 0) {
		if (base == DP_SIDEBAND_MSG_UP_REP_BASE) {
			struct soc_dp_mst_hdr hdr;
			int hdr_size;

			if (mst_hdr_parse((uint8_t *)buf, len, &hdr) < 0)
				return -EIO;

			hdr_size = 3 + hdr.lct / 2;
			if (len < (size_t)hdr_size + 1)
				return -EIO;

			payload_len = len - hdr_size - 1;
			received_crc = ((uint8_t *)buf)[len - 1];
			calc_crc = soc_dp_msg_data_crc4((uint8_t *)buf + hdr_size, payload_len);
			if (received_crc != calc_crc) {
				pr_err("UP_REP chunk CRC mismatch (recv 0x%02x, calc 0x%02x)\n",
						received_crc, calc_crc);
				return -EIO;
			}

			return len;
		}

		if (base == DP_SIDEBAND_MSG_DOWN_REQ_BASE)
			return aux_handle_mst_write(rx, buf, len);

		return -EIO;
	}

	if (!write && buf) {
		size_t data;

		if (base == DP_SIDEBAND_MSG_UP_REQ_BASE) {
			data = 0;
			if (rx->up_req_head != rx->up_req_tail) {
				q_len = rx->up_req_q[rx->up_req_head].len;
				if (offset < q_len) {
					data = min_t(size_t, len, q_len - offset);
					memcpy(buf, rx->up_req_q[rx->up_req_head].msg + offset, data);
				}
			}
			if (data < len)
				memset((uint8_t *)buf + data, 0, len - data);
			return len;
		}

		if (base == DP_SIDEBAND_MSG_DOWN_REP_BASE) {
			data = 0;
			if (offset < rx->mst_down_rep_len) {
				data = min_t(size_t, len, rx->mst_down_rep_len - offset);
				memcpy(buf, rx->mst_down_rep + offset, data);
			}
			if (data < len)
				memset((uint8_t *)buf + data, 0, len - data);
			return len;
		}

		return -EIO;
	}

	return -EIO;
}

static ssize_t aux_handle_esi(struct soc_dp_virtual_rx *rx,
		unsigned int offset, void *buf, size_t len,
		bool write, unsigned int base)
{
	static const unsigned int esi_start = DP_SINK_COUNT_ESI -
						(DP_SINK_COUNT_ESI & ~0x1ff);
	unsigned int idx;
	size_t n, i;

	if (offset < esi_start) {
		size_t pad = min_t(size_t, len, esi_start - offset);

		if (!write)
			memset(buf, 0, pad);

		if (len <= pad)
			return len;

		buf = (uint8_t *)buf + pad;
		offset += pad;
		len -= pad;
	}

	idx = offset - esi_start;
	n = min_t(size_t, len, sizeof(rx->esi) - idx);

	if (write) {
		const uint8_t *src = buf;

		for (i = 0; i < n; i++) {
			uint8_t clear_bits = src[i];

			rx->esi[idx + i] &= ~clear_bits;

			if (idx + i == 1 && (clear_bits & DP_UP_REQ_MSG_RDY)) {
				if (rx->up_req_head != rx->up_req_tail)
					rx->up_req_head = (rx->up_req_head + 1) % DP_UP_REQ_Q_DEPTH;

				if (rx->up_req_head != rx->up_req_tail) {
					rx->esi[1] |= DP_UP_REQ_MSG_RDY;
					schedule_work(&rx->up_req_work);
				}
			}

			if (idx + i == 1 && (clear_bits & DP_DOWN_REP_MSG_RDY)) {
				rx->mst_down_rep_len = 0;
				if (rx->mst_pending_body
						&& rx->mst_pending_sent < rx->mst_pending_body_len)
					mst_deliver_next_chunk(rx);
			}
		}
	} else {
		memcpy(buf, rx->esi + idx, n);
	}

	return n;
}

static int soc_dp_virtual_rx_up_req_thread_fn(void *data)
{
	struct soc_dp_virtual_rx *rx = data;

	if (rx->up_req_cb)
		rx->up_req_cb(rx->up_req_cb_data);

	return 0;
}

static void soc_dp_virtual_rx_up_req_work_fn(struct work_struct *work)
{
	struct soc_dp_virtual_rx *rx
		= container_of(work, struct soc_dp_virtual_rx, up_req_work);
	struct task_struct *task;

	task = kthread_run(soc_dp_virtual_rx_up_req_thread_fn, rx,
			   "soc_dp_up_req");
	if (IS_ERR(task))
		pr_err("%s: kthread_run failed: %ld\n", __func__, PTR_ERR(task));
}

void soc_dp_virtual_rx_set_mst_cap(struct soc_dp_virtual_rx *rx, bool enable)
{
	mutex_lock(&rx->lock);

	if (enable && rx->mst_root && rx->mst_root->type == SOC_DP_MST_ENDPOINT) {
		pr_err("Cannot enable MST mode, root device is an endpoint, no root hub present\n");
		mutex_unlock(&rx->lock);
		return;
	}

	rx->rx_caps[DP_MSTM_CAP] = enable ? BIT(0) : 0x00;
	mutex_unlock(&rx->lock);
}

void soc_dp_mst_assign_ids(struct soc_dp_mst_dev *node, int *id_counter)
{
	int i;

	if (!node)
		return;

	node->id = ++(*id_counter);

	if (node->ports) {
		for (i = 0; i < node->num_ports; i++) {
			node->ports[i].port_id = ++(*id_counter);
			if (node->ports[i].device)
				soc_dp_mst_assign_ids(node->ports[i].device, id_counter);
		}
	}
}

int soc_dp_virtual_rx_init(struct soc_dp_virtual_rx *rx,
		const struct soc_dp_virtual_rx_init_data *init)
{
	struct dpcd_range *ranges;

	mutex_init(&rx->lock);
	dpcd_init_defaults(rx);

	rx->num_ranges = ARRAY_SIZE(dpcd_ranges_template);
	rx->ranges = kmemdup(dpcd_ranges_template, sizeof(dpcd_ranges_template), GFP_KERNEL);
	if (!rx->ranges)
		return -ENOMEM;

	ranges = (struct dpcd_range *)rx->ranges;
	ranges[0].store = rx->rx_caps;
	ranges[1].store = rx->rx_caps + DP_RX_CAPS_SIZE;
	ranges[2].store = rx->rx_caps + DP_RX_CAPS_TAIL_BASE;
	ranges[3].store = rx->link_cfg;
	ranges[4].store = rx->payload_alloc;
	ranges[5].store = &rx->payload_status;
	ranges[6].store = rx->sink_cnt;

	INIT_WORK(&rx->up_req_work, soc_dp_virtual_rx_up_req_work_fn);

	rx->up_req_cb = init->up_req_cb;
	rx->up_req_cb_data = init->up_req_cb_data;
	rx->local_edid = raw_edid;
	rx->local_edid_size = sizeof(raw_edid);

	rx->up_req_head = 0;
	rx->up_req_tail = 0;
	rx->down_req_len = 0;
	rx->down_req_seqno = 0;

	rx->detached_subtrees = NULL;

	rx->mst_id_counter = 0;

	if (init->topology_build) {
		int build_ret = init->topology_build(rx);

		if (build_ret < 0) {
			soc_dp_virtual_rx_fini(rx);
			return build_ret;
		}

		if (!rx->mst_root) {
			pr_err("Topology build finished, but no root tree was registered\n");
			soc_dp_virtual_rx_fini(rx);
			return -ENODEV;
		}
		mst_update_sink_count_locked(rx);

		if (rx->mst_root->type == SOC_DP_MST_ENDPOINT)
			soc_dp_virtual_rx_set_mst_cap(rx, false);
	}

	return 0;
}

static struct soc_dp_mst_dev *
soc_dp_mst_find_dev_in_tree(struct soc_dp_mst_dev *node, int id)
{
	int i;
	struct soc_dp_mst_dev *found;

	if (!node)
		return NULL;
	if (node->id == id)
		return node;

	if (node->ports) {
		for (i = 0; i < node->num_ports; i++) {
			if (node->ports[i].device) {
				found = soc_dp_mst_find_dev_in_tree(
					node->ports[i].device, id);
				if (found)
					return found;
			}
		}
	}

	return NULL;
}

static struct soc_dp_mst_dev *soc_dp_mst_find_dev_anywhere(struct soc_dp_virtual_rx *rx, int id)
{
	struct soc_dp_mst_detached *det;
	struct soc_dp_mst_dev *found;

	found = soc_dp_mst_find_dev_in_tree(rx->mst_root, id);
	if (found)
		return found;

	for (det = rx->detached_subtrees; det; det = det->next) {
		found = soc_dp_mst_find_dev_in_tree(det->root, id);
		if (found)
			return found;
	}

	return NULL;
}

static void soc_dp_mst_find_in_tree(struct soc_dp_mst_dev *node, int id,
		struct soc_dp_mst_find_result *result)
{
	int i;

	if (!node || result->type != SOC_DP_MST_FIND_NONE)
		return;

	if (node->id == id) {
		result->type = SOC_DP_MST_FIND_DEVICE;
		result->dev = node;
		return;
	}

	if (node->ports) {
		for (i = 0; i < node->num_ports; i++) {
			if (node->ports[i].port_id == id) {
				result->type = SOC_DP_MST_FIND_PORT;
				result->port = &node->ports[i];
				result->owner = node;
				return;
			}
			if (node->ports[i].device)
				soc_dp_mst_find_in_tree(node->ports[i].device, id, result);
			if (result->type != SOC_DP_MST_FIND_NONE)
				return;
		}
	}
}

static struct soc_dp_mst_find_result soc_dp_mst_find_by_id(struct soc_dp_virtual_rx *rx, int id)
{
	struct soc_dp_mst_detached *det;
	struct soc_dp_mst_find_result result = { .type = SOC_DP_MST_FIND_NONE };

	if (!rx)
		return result;

	soc_dp_mst_find_in_tree(rx->mst_root, id, &result);
	if (result.type != SOC_DP_MST_FIND_NONE)
		return result;

	for (det = rx->detached_subtrees; det; det = det->next) {
		soc_dp_mst_find_in_tree(det->root, id, &result);
		if (result.type != SOC_DP_MST_FIND_NONE)
			return result;
	}

	return result;
}

static int __soc_dp_mst_notify_conn_stat_locked(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *root, int device_id, bool plugged)
{
	uint8_t hdr_buf[4];
	uint8_t msg[256];
	uint8_t data_crc;
	uint8_t body[DP_CONNSTAT_BODY_SIZE];
	int next_tail, hdr_len, body_len, msg_len;
	struct soc_dp_mst_dev *dev;
	struct soc_dp_mst_dev *parent;

	dev = soc_dp_mst_find_dev_anywhere(rx, device_id);
	if (!dev)
		return -ENOENT;

	parent = dev->parent;
	if (!parent)
		return -EINVAL;

	memset(body, 0, DP_CONNSTAT_BODY_SIZE);
	body_len = 0;

	body[body_len++] = DP_CONNECTION_STATUS_NOTIFY & 0x7f;
	body[body_len++] = (dev->parent_port_number & 0xf) << 4;

	memcpy(&body[body_len], parent->guid, DP_GUID_SIZE);
	body_len += DP_GUID_SIZE;

	if (plugged) {
		uint8_t pdt, flags;

		pdt = (dev->type == SOC_DP_MST_BRANCH) ?
			DP_PEER_DEVICE_MST_BRANCHING : DP_PEER_DEVICE_SST_SINK;

		flags = (pdt & 0x7);
		flags |= BIT(5);

		if (dev->type == SOC_DP_MST_BRANCH || dev->type == SOC_DP_MST_ENDPOINT)
			flags |= BIT(4);

		body[body_len] = flags;
	} else {
		body[body_len] = DP_PEER_DEVICE_NONE;
	}
	body_len++;

	data_crc = soc_dp_msg_data_crc4(body, body_len);

	hdr_buf[0] = (1 << 4) | 6;
	hdr_buf[1] = (1U << 7) | ((body_len + 1) & 0x3f);
	hdr_buf[2] = (1 << 7) | (1 << 6) | 0;
	hdr_buf[2] |= soc_dp_msg_header_crc4(hdr_buf, 5) & 0xf;
	hdr_len = 3;

	msg_len = 0;
	memcpy(msg + msg_len, hdr_buf, hdr_len);
	msg_len += hdr_len;
	memcpy(msg + msg_len, body, body_len);
	msg_len += body_len;
	msg[msg_len++] = data_crc;

	next_tail = (rx->up_req_tail + 1) % DP_UP_REQ_Q_DEPTH;
	if (next_tail != rx->up_req_head) {
		memcpy(rx->up_req_q[rx->up_req_tail].msg, msg,
				min_t(size_t, msg_len, 256));
		rx->up_req_q[rx->up_req_tail].len = msg_len;
		rx->up_req_tail = next_tail;

		rx->esi[1] |= DP_UP_REQ_MSG_RDY;
		schedule_work(&rx->up_req_work);
		return 0;
	}

	pr_warn("UP_REQ queue full, dropping hotplug notification\n");
	return -EBUSY;
}

static bool soc_dp_mst_is_in_active_tree(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *dev)
{
	struct soc_dp_mst_dev *node = dev;

	while (node->parent)
		node = node->parent;

	return (node == rx->mst_root);
}

static int __soc_dp_mst_unplug_device_no_lock(struct soc_dp_virtual_rx *rx, int device_id)
{
	int ret;
	int port_number;
	bool in_active;
	uint16_t consumed_pbn;

	struct soc_dp_mst_find_result result;
	struct soc_dp_mst_dev *dev;
	struct soc_dp_mst_dev *parent;
	struct soc_dp_mst_dev *up_node;
	struct soc_dp_mst_port *up_port;
	struct soc_dp_mst_detached *det;
	struct soc_dp_mst_detached *prev_det;

	det = kzalloc(sizeof(*det), GFP_KERNEL);
	if (!det)
		return -ENOMEM;

	result = soc_dp_mst_find_by_id(rx, device_id);
	if (result.type == SOC_DP_MST_FIND_NONE) {
		kfree(det);
		pr_err("Device %d not found for unplug\n", device_id);
		return -ENOENT;
	}
	if (result.type == SOC_DP_MST_FIND_PORT) {
		kfree(det);
		pr_err("ID %d is a port, not a device\n", device_id);
		return -EINVAL;
	}
	dev = result.dev;

	if (dev == rx->mst_root) {
		kfree(det);
		pr_err("Root node cannot be removed\n");
		return -EPERM;
	}

	parent = dev->parent;
	port_number = dev->parent_port_number - 1;

	if (!parent || port_number < 0 || port_number >= parent->num_ports) {
		kfree(det);
		pr_err("Device %d has no valid parent\n", device_id);
		return -EINVAL;
	}

	for (prev_det = rx->detached_subtrees; prev_det; prev_det = prev_det->next) {
		if (prev_det->root == dev) {
			kfree(det);
			pr_err("Device %d already detached\n", device_id);
			return -EALREADY;
		}
	}

	in_active = soc_dp_mst_is_in_active_tree(rx, dev);
	consumed_pbn = parent->ports[port_number].full_pbn - parent->ports[port_number].avail_pbn;

	if (in_active) {
		ret = __soc_dp_mst_notify_conn_stat_locked(rx, rx->mst_root, device_id, false);
		if (ret < 0)
			pr_warn_ratelimited("CSN queue full, dropped unplug notify for device %d (topology change proceeds)\n",
					device_id);
	}

	parent->ports[port_number].connected = false;
	parent->ports[port_number].device = NULL;

	dev->parent = NULL;
	dev->parent_port_number = -1;

	up_node = parent;
	while (up_node && up_node->parent) {
		up_port = &up_node->parent->ports[up_node->parent_port_number - 1];
		up_port->avail_pbn += consumed_pbn;
		up_node = up_node->parent;
	}

	parent->ports[port_number].full_pbn = 0;
	parent->ports[port_number].avail_pbn = 0;
	memset(parent->ports[port_number].payloads, 0,
			sizeof(parent->ports[port_number].payloads));
	mst_reset_payloads_recursive(dev);

	det->root = dev;
	det->next = rx->detached_subtrees;
	rx->detached_subtrees = det;

	return 0;
}

int soc_dp_mst_unplug_device(struct soc_dp_virtual_rx *rx, int device_id)
{
	int ret;

	if (!rx || !rx->mst_root) {
		pr_err("RX or root not initialized\n");
		return -EINVAL;
	}

	if (!(rx->rx_caps[DP_MSTM_CAP] & DP_MST_CAP)) {
		pr_err("Not in MST mode\n");
		return -EINVAL;
	}

	mutex_lock(&rx->lock);
	ret = __soc_dp_mst_unplug_device_no_lock(rx, device_id);
	if (ret == 0)
		mst_update_sink_count_locked(rx);
	mutex_unlock(&rx->lock);

	return ret;
}

static int __soc_dp_mst_plug_device_no_lock(struct soc_dp_virtual_rx *rx,
		int device_id, int target_port_id)
{
	bool in_active;
	struct soc_dp_mst_find_result dev_result;
	struct soc_dp_mst_find_result port_result;
	struct soc_dp_mst_detached *det, *prev;
	struct soc_dp_mst_dev *dev;
	struct soc_dp_mst_dev *target_owner;
	struct soc_dp_mst_port *target_port;

	dev_result = soc_dp_mst_find_by_id(rx, device_id);
	if (dev_result.type == SOC_DP_MST_FIND_NONE) {
		pr_err("Device %d not found for plug\n", device_id);
		return -ENOENT;
	}
	if (dev_result.type == SOC_DP_MST_FIND_PORT) {
		pr_err("ID %d is a port, not a device\n", device_id);
		return -EINVAL;
	}
	dev = dev_result.dev;

	if (dev == rx->mst_root) {
		pr_err("Root node cannot be used as plug device\n");
		return -EPERM;
	}
	if (dev->parent) {
		pr_err("Device %d has a parent, not detached\n", device_id);
		return -EINVAL;
	}

	prev = NULL;
	for (det = rx->detached_subtrees; det; prev = det, det = det->next) {
		if (det->root == dev)
			break;
	}
	if (!det) {
		pr_err("Device %d not in detached list\n", device_id);
		return -ENOENT;
	}

	port_result = soc_dp_mst_find_by_id(rx, target_port_id);
	if (port_result.type == SOC_DP_MST_FIND_NONE) {
		pr_err("Port %d not found\n", target_port_id);
		return -ENOENT;
	}
	if (port_result.type == SOC_DP_MST_FIND_DEVICE) {
		pr_err("ID %d is a device, not a port\n", target_port_id);
		return -EINVAL;
	}
	target_port = port_result.port;
	target_owner = port_result.owner;

	if (!target_owner || target_owner->type != SOC_DP_MST_BRANCH) {
		pr_err("Port %d not on a branch device\n", target_port_id);
		return -EINVAL;
	}

	in_active = soc_dp_mst_is_in_active_tree(rx, target_owner);

	if (target_port->connected || target_port->device) {
		pr_err("Target port %d is occupied\n", target_port_id);
		return -EBUSY;
	}

	dev->parent = target_owner;
	dev->parent_port_number = target_port->port_number;
	target_port->connected = true;
	target_port->device = dev;
	target_port->full_pbn = dev->link_full_pbn;
	target_port->avail_pbn = dev->link_full_pbn;
	memset(target_port->payloads, 0, sizeof(target_port->payloads));

	if (prev)
		prev->next = det->next;
	else
		rx->detached_subtrees = det->next;
	kfree(det);

	if (in_active) {
		int notify_ret;

		notify_ret = __soc_dp_mst_notify_conn_stat_locked(
			rx, rx->mst_root, device_id, true);
		if (notify_ret < 0)
			pr_warn_ratelimited("CSN queue full, dropped plug notify for device %d (topology change proceeds)\n",
					device_id);
	}

	return 0;
}

int soc_dp_mst_plug_device(struct soc_dp_virtual_rx *rx, int device_id, int target_port_id)
{
	int ret;

	if (!rx || !rx->mst_root) {
		pr_err("RX or root not initialized\n");
		return -EINVAL;
	}

	if (!(rx->rx_caps[DP_MSTM_CAP] & DP_MST_CAP)) {
		pr_err("Not in MST mode\n");
		return -EINVAL;
	}

	mutex_lock(&rx->lock);
	ret = __soc_dp_mst_plug_device_no_lock(rx, device_id, target_port_id);
	if (ret == 0)
		mst_update_sink_count_locked(rx);
	mutex_unlock(&rx->lock);

	return ret;
}

void soc_dp_virtual_rx_fini(struct soc_dp_virtual_rx *rx)
{
	struct soc_dp_mst_detached *det, *next;

	cancel_work_sync(&rx->up_req_work);

	soc_dp_mst_dev_free(rx->mst_root);
	rx->mst_root = NULL;

	for (det = rx->detached_subtrees; det; det = next) {
		next = det->next;
		soc_dp_mst_dev_free(det->root);
		kfree(det);
	}
	rx->detached_subtrees = NULL;

	kfree(rx->mst_pending_body);
	rx->mst_pending_body = NULL;
	kfree(rx->ranges);
	rx->ranges = NULL;
	mutex_destroy(&rx->lock);
}
