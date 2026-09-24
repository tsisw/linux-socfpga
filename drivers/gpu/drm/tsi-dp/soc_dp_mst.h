#ifndef __SOC_DP_MST_H__
#define __SOC_DP_MST_H__

#include <drm/drm_connector.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/display/drm_dp_mst_helper.h>

#include "soc_dp_dri.h"

/**
 * soc_dp_mst_create - Initialize MST topology manager and encoders
 * @dp: DP device structure
 *
 * Resets CRTC-to-stream mapping, registers MST topology manager,
 * and creates per-CRTC MST encoders with DPMST encoder type.
 *
 * Return: 0 on success, negative errno on topology manager init failure
 */
int soc_dp_mst_create(struct soc_dp_dev *dp);

/**
 * soc_dp_mst_destroy - Tear down MST topology manager and encoders
 * @dp: DP device structure
 *
 * Destroys the MST topology manager and cleans up all MST encoder
 * registrations. Must be called after soc_dp_mst_fini().
 */
void soc_dp_mst_destroy(struct soc_dp_dev *dp);

/**
 * soc_dp_mst_init - Power on PHY, train link, and set MST mode
 * @dp: DP device structure
 *
 * Configures PHY, powers on, trains the main link, reads hardware
 * maximum stream count, enables MST mode in hardware, and sets
 * the topology manager into MST mode.
 *
 * Return: 0 on success, negative errno on PHY or link training failure
 */
int soc_dp_mst_init(struct soc_dp_dev *dp);

/**
 * soc_dp_mst_fini - Disable MST mode and power off PHY
 * @dp: DP device structure
 *
 * Disables MST mode in hardware, sets topology manager to SST mode,
 * and powers off the PHY. The reverse of soc_dp_mst_init().
 */
void soc_dp_mst_fini(struct soc_dp_dev *dp);

#ifdef CONFIG_SOC_DP_DRIVER_QEMU
/**
 * soc_dp_mst_default_topology_build - Construct a default MST topology tree
 * @rx: Virtual RX device for QEMU mode
 *
 * Creates a multi-level topology with root hub, daisy-chain branches,
 * endpoint devices, and optional detached subtree for testing MST
 * add/remove operations.
 *
 * Return: 0 on success, -ENOMEM on allocation failure
 */
int soc_dp_mst_default_topology_build(struct soc_dp_virtual_rx *rx);
#endif

/**
 * soc_dp_mst_set_stream_msa - Configure per-stream MSA and timing registers
 * @dp: DP device structure
 * @stream_id: MST stream index (0-based)
 * @mode: Display mode with timing parameters
 * @color_format: SOC video format index
 * @rate: Link rate in kHz
 * @lanes: Lane count
 *
 * Computes TU, HBlank interval, FIFO read threshold and writes all
 * per-stream MSA, polarity, timing, and link-layer registers via
 * soc_dp_stream_reg_write_range().
 */
void soc_dp_mst_set_stream_msa(struct soc_dp_dev *dp, uint8_t stream_id,
		const struct drm_display_mode *mode,
		uint32_t color_format, enum soc_dp_link_rate rate,
		enum soc_dp_lane_count lanes);

/**
 * soc_dp_mst_trigger_act - Send ACT (Allocation Change Trigger) sequence
 * @dp: DP device structure
 */
void soc_dp_mst_trigger_act(struct soc_dp_dev *dp);

/**
 * soc_dp_mst_stream_enable - Enable a specific MST video stream
 * @dp: DP device structure
 * @stream_id: Stream index (0-based)
 */
void soc_dp_mst_stream_enable(struct soc_dp_dev *dp, uint8_t stream_id);

/**
 * soc_dp_mst_stream_disable - Disable a specific MST video stream
 * @dp: DP device structure
 * @stream_id: Stream index (0-based)
 */
void soc_dp_mst_stream_disable(struct soc_dp_dev *dp, uint8_t stream_id);

#endif
