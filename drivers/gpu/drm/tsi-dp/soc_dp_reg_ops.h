#ifndef __SOC_DP_REG_OPS_H__
#define __SOC_DP_REG_OPS_H__

#include "soc_dp_reg.h"

struct soc_dp_dev;

#define SOC_DPTX_STREAM_OFFSET 0x10000

#define __SOC_DPTX_REG_OFFSET(o, h, l) (o)
#define SOC_DPTX_REG_OFFSET(base) __SOC_DPTX_REG_OFFSET(base)

/**
 * soc_dp_reg_write - Read-modify-write a masked field in a DP register
 * @dp: DP device structure
 * @offset: Register byte offset within MMIO region
 * @bit_wide: Bit width of the register (must be 32)
 * @mask: Bitmask for the target field
 * @val: Value to write into the masked field
 */
int soc_dp_reg_write(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t bit_wide, uint32_t mask, uint32_t val);

/**
 * soc_dp_reg_write_range - Write a value to a bit range within a DP register
 * @dp: DP device structure
 * @offset: Register byte offset within MMIO region
 * @high: High bit position of the target field (inclusive)
 * @low: Low bit position of the target field
 * @val: Value to write (shifted into position by @low)
 *
 * Convenience wrapper around soc_dp_reg_write() that derives the mask
 * from @high and @low.
 */
int soc_dp_reg_write_range(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val);

/**
 * soc_dp_reg_only_write_range - Write a value to a bit range (direct write)
 * @dp: DP device structure
 * @offset: Register byte offset within MMIO region
 * @high: High bit position of the target field (inclusive)
 * @low: Low bit position of the target field
 * @val: Value to write (shifted into position by @low)
 *
 * Direct register write without read-modify-write. Used only for
 * non-QEMU, non-bypass HPD register access.
 */
int soc_dp_reg_only_write_range(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val);

/**
 * soc_dp_reg_read - Read a masked field from a DP register
 * @dp: DP device structure
 * @offset: Register byte offset within MMIO region
 * @bit_wide: Bit width of the register (must be 32)
 * @mask: Bitmask for the target field
 * @val: Output parameter for the masked register value
 */
int soc_dp_reg_read(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t bit_wide, uint32_t mask, uint32_t *val);

/**
 * soc_dp_reg_read_range - Read a bit range from a DP register
 * @dp: DP device structure
 * @offset: Register byte offset within MMIO region
 * @high: High bit position of the target field (inclusive)
 * @low: Low bit position of the target field
 * @val: Output parameter for the field value (shifted down by @low)
 *
 * Convenience wrapper around soc_dp_reg_read() that derives the mask
 * from @high and @low.
 */
int soc_dp_reg_read_range(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t *val);

/**
 * _soc_dp_stream_reg_write_range - Write to a per-stream DP register range
 * @dp: DP device structure
 * @stream_id: MST stream index (0-based, offset applied automatically)
 * @offset: Base register byte offset (stream offset added internally)
 * @high: High bit position of the target field (inclusive)
 * @low: Low bit position of the target field
 * @val: Value to write (shifted into position by @low)
 *
 * Low-level helper; prefer soc_dp_stream_reg_write_range() macro.
 */
void _soc_dp_stream_reg_write_range(struct soc_dp_dev *dp, uint8_t stream_id,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val);

/**
 * _soc_dp_stream_reg_read_range - Read from a per-stream DP register range
 * @dp: DP device structure
 * @stream_id: MST stream index (0-based, offset applied automatically)
 * @offset: Base register byte offset (stream offset added internally)
 * @high: High bit position of the target field (inclusive)
 * @low: Low bit position of the target field
 * @val: Output parameter for the field value (shifted down by @low)
 *
 * Low-level helper; prefer soc_dp_stream_reg_read_range() macro.
 */
void _soc_dp_stream_reg_read_range(struct soc_dp_dev *dp, uint8_t stream_id,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t *val);

/**
 * soc_dp_stream_reg_write_range - Per-stream register write macro
 * @dp: DP device structure
 * @stream_id: MST stream index (0-based)
 * @reg_macro: Register field macro from soc_dp_reg.h
 * @val: Value to write
 *
 * Macro wrapper that expands @reg_macro into offset/high/low and
 * delegates to _soc_dp_stream_reg_write_range().
 */
#define soc_dp_stream_reg_write_range(dp, stream_id, reg_macro, val) \
	_soc_dp_stream_reg_write_range(dp, stream_id, reg_macro, val)

/**
 * soc_dp_stream_reg_read_range - Per-stream register read macro
 * @dp: DP device structure
 * @stream_id: MST stream index (0-based)
 * @reg_macro: Register field macro from soc_dp_reg.h
 * @val_ptr: Pointer to variable receiving the value
 *
 * Macro wrapper that expands @reg_macro into offset/high/low and
 * delegates to _soc_dp_stream_reg_read_range().
 */
#define soc_dp_stream_reg_read_range(dp, stream_id, reg_macro, val_ptr) \
	_soc_dp_stream_reg_read_range(dp, stream_id, reg_macro, val_ptr)

#endif
