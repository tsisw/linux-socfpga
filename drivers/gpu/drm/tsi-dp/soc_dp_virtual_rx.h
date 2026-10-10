#ifndef __SOC_DP_AUX_H__
#define __SOC_DP_AUX_H__

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dp_helper.h>

#define DPCD_PAGE_SIZE           0x100
#define DPCD_MST_WINDOW_SIZE     0x200

#define DP_RX_CAPS_SIZE          0x30
#define DP_RX_CAPS_TAIL_BASE     0x40
#define DP_RX_CAPS_TAIL_SIZE     0xC0
#define DP_PAYLOAD_ALLOC_SIZE    3

#define DP_GUID_SIZE             16
#define DP_PAYLOAD_SLOT_MAX      64
#define DP_MAX_PORTS             15
#define DP_SIDEBAND_CHUNK_MAX    48
#define DP_UP_REQ_Q_DEPTH        8
#define DP_RAD_SIZE              8
#define DP_MST_HDR_BASE          3
#define DP_CONNSTAT_BODY_SIZE    24
#define DP_ESI_SIZE              16

#define DP_PORT_CAP_DDPS         BIT(6)
#define DP_PORT_CAP_MCS          BIT(7)
#define DP_PORT_CAP_LDPS         BIT(5)

#define DP_LINK_ADDR_SST_SDP_COUNT   6
#define DP_LINK_ADDR_SST_SDP_SINKS   14

enum soc_dp_mst_dev_type {
	SOC_DP_MST_BRANCH = 0,
	SOC_DP_MST_ENDPOINT,
};

struct soc_dp_vcpi_alloc {
	uint8_t	vcpi;
	uint16_t pbn;
};

struct soc_dp_mst_port {
	int port_number;
	int port_id;
	bool connected;
	struct soc_dp_mst_dev *device;

	uint16_t full_pbn;
	uint16_t avail_pbn;
	struct soc_dp_vcpi_alloc payloads[DP_PAYLOAD_SLOT_MAX];
};

struct soc_dp_mst_dev {
	enum soc_dp_mst_dev_type type;
	int id;
	const char *name;

	int num_ports;
	struct soc_dp_mst_port *ports;

	struct soc_dp_mst_dev *parent;
	int parent_port_number;

	uint8_t guid[DP_GUID_SIZE];

	uint8_t	*dpcd;
	size_t  dpcd_size;

	uint8_t	*edid;
	size_t  edid_size;
	uint8_t edid_offset;
	uint8_t edid_segment;

	uint8_t  lane_count;
	uint8_t  link_rate_dgbps;
	uint16_t link_full_pbn;
};

struct soc_dp_mst_hdr {
	uint8_t	lct;
	uint8_t	lcr;
	uint8_t	rad[DP_RAD_SIZE];
	uint8_t	broadcast;
	uint8_t	path_msg;
	uint8_t	msg_len;
	uint8_t	seqno;
	uint8_t	somt;
	uint8_t	eomt;
	uint8_t	msg_type;
};

struct soc_dp_virtual_rx;

typedef void (*soc_dp_rx_cb_t)(void *data);

typedef int (*soc_dp_mst_topology_build_fn)(struct soc_dp_virtual_rx *rx);

struct soc_dp_virtual_rx_init_data {
	soc_dp_rx_cb_t up_req_cb;
	void *up_req_cb_data;
	soc_dp_mst_topology_build_fn topology_build;
};

struct soc_dp_virtual_rx {
	struct mutex lock;
	int mst_id_counter;

	uint8_t rx_caps[DPCD_PAGE_SIZE];
	uint8_t link_cfg[DPCD_PAGE_SIZE];
	uint8_t sink_cnt[DPCD_PAGE_SIZE];
	uint8_t payload_alloc[DP_PAYLOAD_ALLOC_SIZE];
	uint8_t payload_status;
	uint8_t esi[DP_ESI_SIZE];

	uint8_t edid_offset;

	const uint8_t *local_edid;
	size_t local_edid_size;
	uint8_t edid_segment;

	void *ranges;
	size_t num_ranges;

	uint8_t mst_down_rep[256];
	size_t  mst_down_rep_len;

	uint8_t	down_req_buf[512];
	size_t  down_req_len;
	uint8_t	down_req_seqno;

	struct {
		uint8_t	msg[256];
		size_t  len;
	} up_req_q[DP_UP_REQ_Q_DEPTH];
	int	up_req_head;
	int	up_req_tail;

	struct soc_dp_mst_dev *mst_root;

	struct soc_dp_mst_detached {
		struct soc_dp_mst_dev *root;
		struct soc_dp_mst_detached *next;
	} *detached_subtrees;

	uint8_t *mst_pending_body;
	size_t   mst_pending_body_len;
	size_t   mst_pending_sent;
	uint8_t  mst_pending_seqno;

	soc_dp_rx_cb_t up_req_cb;
	void *up_req_cb_data;

	struct work_struct up_req_work;
};

struct soc_dp_mst_dev_cfg {
	const char *name;
	int num_ports;
	uint8_t lane_count;
	uint8_t link_rate_dgbps;
	uint8_t *edid;
	size_t edid_size;
};

/**
 * soc_dp_virtual_rx_init - Initialize virtual DP RX for QEMU simulation
 * @rx: Virtual RX device structure
 * @init: Initialization data including up-request callback and topology builder
 *
 * Allocates and configures DPCD storage, registers the up-request
 * work handler, and sets default EDID.
 *
 * Return: 0 on success, -ENOMEM on allocation failure
 */
int soc_dp_virtual_rx_init(struct soc_dp_virtual_rx *rx,
		const struct soc_dp_virtual_rx_init_data *init);

/**
 * soc_dp_virtual_rx_fini - Release resources used by virtual DP RX
 * @rx: Virtual RX device structure
 *
 * Frees all allocated topology subtrees, DPCD ranges, and pending
 * sideband message buffers.
 */
void soc_dp_virtual_rx_fini(struct soc_dp_virtual_rx *rx);

/**
 * soc_dp_virtual_rx_aux_transfer - Handle AUX channel transactions for virtual RX
 * @aux: DP AUX channel structure
 * @msg: AUX message containing request and reply buffer
 *
 * Dispatches AUX reads/writes to the appropriate virtual DPCD range
 * or sideband message handler. Simulates sink-side AUX behavior
 * for QEMU-based testing.
 *
 * Return: Number of bytes transferred, or negative errno on failure
 */
ssize_t soc_dp_virtual_rx_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg);

/**
 * soc_dp_virtual_rx_set_mst_cap - Enable or disable MST capability in virtual RX
 * @rx: Virtual RX device structure
 * @enable: true to set MST-capable DPCD bit, false to clear it
 */
void soc_dp_virtual_rx_set_mst_cap(struct soc_dp_virtual_rx *rx, bool enable);

/**
 * soc_dp_mst_branch_create - Allocate and initialize an MST branch device
 * @parent: Parent device in the topology (NULL for root)
 * @parent_port_number: Port number on the parent this branch connects to
 * @num_downstream_ports: Number of downstream ports on this branch
 * @name: Human-readable device name
 * @lane_count: Maximum lane count this branch supports
 * @link_rate_dgbps: Maximum link rate in deca-Gbps (e.g. 54 = 5.4 Gbps)
 *
 * Return: Pointer to new branch device, or NULL on allocation failure
 */
struct soc_dp_mst_dev *
soc_dp_mst_branch_create(struct soc_dp_mst_dev *parent,
		int parent_port_number, int num_downstream_ports,
		const char *name, uint8_t lane_count, uint8_t link_rate_dgbps);

/**
 * soc_dp_mst_endpoint_create - Allocate and initialize an MST endpoint device
 * @parent: Parent branch device in the topology
 * @parent_port_number: Port number on the parent this endpoint connects to
 * @name: Human-readable device name
 * @edid: EDID block data for this sink
 * @edid_size: Size of @edid in bytes
 * @lane_count: Maximum lane count this endpoint supports
 * @link_rate_dgbps: Maximum link rate in deca-Gbps
 *
 * Return: Pointer to new endpoint device, or NULL on allocation failure
 */
struct soc_dp_mst_dev *
soc_dp_mst_endpoint_create(struct soc_dp_mst_dev *parent,
		int parent_port_number, const char *name,
		uint8_t *edid, size_t edid_size,
		uint8_t lane_count, uint8_t link_rate_dgbps);

/**
 * soc_dp_mst_add_subtree - Attach a topology subtree to the virtual RX topology
 * @rx: Virtual RX device structure
 * @tree_root: Root of the subtree to attach
 * @target_port_id: Port ID on the existing topology where the subtree attaches
 *
 * Inserts @tree_root into the topology rooted at @rx->mst_root at the
 * port identified by @target_port_id. Updates sink count after insertion.
 *
 * Return: 0 on success, -EINVAL if arguments are NULL
 */
int soc_dp_mst_add_subtree(struct soc_dp_virtual_rx *rx,
		struct soc_dp_mst_dev *tree_root, int target_port_id);

/**
 * soc_dp_mst_assign_ids - Assign unique device IDs to an MST topology subtree
 * @node: Root of the subtree to assign IDs to
 * @id_counter: Pointer to shared counter, incremented for each device
 *
 * Assigns a unique integer ID to every node in the subtree via
 * depth-first traversal.
 */
void soc_dp_mst_assign_ids(struct soc_dp_mst_dev *node, int *id_counter);

/**
 * soc_dp_mst_clone_subtree - Deep-copy a subtree and attach it to another port
 * @rx: Virtual RX device structure
 * @root_device_id: Device ID of the subtree root to clone
 * @target_port_id: Port ID where the cloned subtree will be attached
 *
 * Performs a recursive clone of the subtree rooted at @root_device_id
 * and attaches the copy at @target_port_id.
 *
 * Return: 0 on success, negative errno on failure
 */
int soc_dp_mst_clone_subtree(struct soc_dp_virtual_rx *rx, int root_device_id, int target_port_id);

/**
 * soc_dp_mst_delete_subtree - Remove and free a subtree from the topology
 * @rx: Virtual RX device structure
 * @root_device_id: Device ID of the subtree root to delete
 *
 * Detaches the subtree rooted at @root_device_id from the topology
 * and frees all nodes in that subtree. If the detached subtree is
 * the root itself, uses the detached_subtrees list.
 *
 * Return: 0 on success, negative errno if device not found
 */
int soc_dp_mst_delete_subtree(struct soc_dp_virtual_rx *rx, int root_device_id);

/**
 * soc_dp_mst_topology_dump - Print a topology subtree to kernel log
 * @dev: Root of the subtree to dump
 * @prefix: Indentation prefix string for tree formatting
 * @pbn_div: PBN divisor for bandwidth display
 *
 * Recursively prints device type, name, ID, port PBN, and EDID
 * information for each node in the subtree.
 */
void soc_dp_mst_topology_dump(const struct soc_dp_mst_dev *dev, const char *prefix, int pbn_div);

/**
 * soc_dp_mst_topology_dump_all - Print the full virtual RX topology
 * @rx: Virtual RX device structure
 * @pbn_div: PBN divisor for bandwidth display
 *
 * Convenience wrapper that dumps the main topology and any detached
 * subtrees via soc_dp_mst_topology_dump().
 */
void soc_dp_mst_topology_dump_all(struct soc_dp_virtual_rx *rx, int pbn_div);

/**
 * soc_dp_mst_unplug_device - Simulate hot-unplug of an MST device
 * @rx: Virtual RX device structure
 * @device_id: Device ID to unplug
 *
 * Removes the device from the topology and triggers the up-request
 * callback so the source driver processes the unplug event.
 *
 * Return: 0 on success, negative errno if device not found
 */
int soc_dp_mst_unplug_device(struct soc_dp_virtual_rx *rx, int device_id);

/**
 * soc_dp_mst_plug_device - Simulate hot-plug of a previously detached MST device
 * @rx: Virtual RX device structure
 * @device_id: Device ID of the detached subtree to re-plug
 * @target_port_id: Port ID where the device should be re-attached
 *
 * Moves a detached subtree back into the active topology at
 * @target_port_id and triggers the up-request callback.
 *
 * Return: 0 on success, negative errno if device not found or port invalid
 */
int soc_dp_mst_plug_device(struct soc_dp_virtual_rx *rx, int device_id, int target_port_id);

#endif
