#include <linux/io.h>

#include "soc_dp_reg_ops.h"
#include "soc_dp_dri.h"

int soc_dp_reg_write(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t bit_wide, uint32_t mask, uint32_t val)
{
	uint32_t reg_val;

	reg_val = (uint32_t)readl(dp->regs + offset);
	reg_val &= ~mask;
	reg_val |= val & mask;
	writel(reg_val, dp->regs + offset);

	return 0;
}

int soc_dp_reg_write_range(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val)
{
	uint32_t mask;

	mask = (uint32_t)(((((uint64_t)1) << (high - low + 1)) - 1) << low);
	return soc_dp_reg_write(dp, offset, 32, mask, (val << low) & mask);
}

int soc_dp_reg_only_write_range(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val)
{
	uint32_t mask;

	mask = (uint32_t)(((((uint64_t)1) << (high - low + 1)) - 1) << low);
	writel((val << low) & mask, dp->regs + offset);
	return 0;
}

int soc_dp_reg_read(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t bit_wide, uint32_t mask, uint32_t *val)
{
	*val = ((uint32_t)readl(dp->regs + offset)) & mask;
	return 0;
}

int soc_dp_reg_read_range(struct soc_dp_dev *dp,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t *val)
{
	int ret;
	uint32_t mask;

	mask = (uint32_t)(((((uint64_t)1) << (high - low + 1)) - 1) << low);
	ret = soc_dp_reg_read(dp, offset, 32, mask, val);
	*val = *val >> low;

	return ret;
}

void _soc_dp_stream_reg_write_range(struct soc_dp_dev *dp, uint8_t stream_id,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val)
{
	if (unlikely(stream_id >= dp->max_mst_streams)) {
		dev_err(dp->dev, "Write out of bounds stream_id %u (max %u)\n",
			stream_id, dp->max_mst_streams);
		return;
	}

	soc_dp_reg_write_range(dp, offset + (stream_id * SOC_DPTX_STREAM_OFFSET),
		high, low, val);
}

void _soc_dp_stream_reg_read_range(struct soc_dp_dev *dp, uint8_t stream_id,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t *val)
{
	if (unlikely(stream_id >= dp->max_mst_streams)) {
		dev_err(dp->dev, "Read out of bounds stream_id %u (max %u)\n",
			stream_id, dp->max_mst_streams);
		*val = 0;
		return;
	}

	soc_dp_reg_read_range(dp, offset + (stream_id * SOC_DPTX_STREAM_OFFSET),
		high, low, val);
}
