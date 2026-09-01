// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DWC_i2c ("Advanced I2C/SMBus Controller") driver for TSI SkyLP.
 *
 * Why this is not a variant of i2c-designware-platdrv
 * ---------------------------------------------------
 * The existing i2c-designware-* driver targets DW_apb_i2c. SkyLP instantiates
 * DWC_i2c v1.01a, a later and separate Synopsys IP. Every register offset
 * differs, and three differences are semantic rather than positional, so a
 * regmap offset-translation on top of the existing driver cannot express them:
 *
 *   1. IC_CON's SPEED field moved from bits 2:1 to IC_CTRL bits 5:4, and the
 *      upper control bits all shifted by three.
 *   2. MASTER_MODE (b0) + SLAVE_DISABLE (b6) collapsed into one IC_OP_MODE bit.
 *   3. The SS/FS/HS SCL count triplets collapsed into a single
 *      IC_SCL_HCNT/IC_SCL_LCNT pair selected by IC_CTRL.SPEED.
 *
 * What IS reused is the algorithm: the transfer loop below follows
 * i2c-designware-master.c's structure (fill IC_DATA_CMD, drain IC_RXFLR,
 * decode the abort/termination source on failure), with the register layer
 * replaced.
 *
 * Polled, not interrupt-driven
 * ----------------------------
 * The GIC routing for this controller is not yet established (it is one of the
 * open interrupt fields in the SkyLP chiplet layout), and the DWC_i2c v1.01a
 * interrupt bit positions are not established from the RTL-generated RAL
 * either. Rather than guess, this driver polls IC_STATUS/IC_TXFLR/IC_RXFLR --
 * which is also all the vFlex behavioural model implements. Interrupt support
 * lands together with the routing.
 *
 * Pin muxing belongs to pinctrl, not here
 * ---------------------------------------
 * The SCL/SDA pads live in the IONE IO corner, whose pad and iomode registers
 * are owned by pinctrl-tsi (tsi,skylp-ione-pinctrl). Muxing them is a
 * "groups = \"smb\"; function = \"i2c\"" pin-configuration node referenced from
 * this controller's pinctrl-0; the pinctrl core applies that default state
 * before probe runs, so there is nothing to do here.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "i2c-dw-advanced-regs.h"

#define DWA_DRV_NAME		"i2c-dw-advanced"

/*
 * Fast-mode SCL counts. These are the values the RTL DV bring-up
 * (i2c0_fast_mode_bringup.c) programs and the silicon is validated against.
 *
 * They are used verbatim rather than computed from the input clock because
 * the IONE MGTCLK rate is not yet established. Computing from a guessed clock
 * would silently produce a wrong bus speed; using the known-good constants and
 * saying so does not. Device tree can override via tsi,scl-hcnt / tsi,scl-lcnt
 * once the clock is known.
 */
#define DWA_DEFAULT_SCL_HCNT	0x320
#define DWA_DEFAULT_SCL_LCNT	0x4b0
#define DWA_DEFAULT_SDA_HOLD	0x40
#define DWA_DEFAULT_SPKLEN	0x1

/*
 * Poll bounds. Generous: under tsisim every register access is a socket
 * round trip to the behavioural model, not a bus cycle.
 */
#define DWA_ENABLE_TIMEOUT_US	100000
#define DWA_XFER_TIMEOUT_US	500000
#define DWA_POLL_INTERVAL_US	10

struct dwa_i2c_dev {
	struct device		*dev;
	void __iomem		*base;		/* controller register file */
	struct i2c_adapter	adap;
	u32			blk;		/* IC_I2C_BLOCK_OFFSET value */
	u32			scl_hcnt;
	u32			scl_lcnt;
	u32			bus_freq_hz;
	/*
	 * Target address the controller is currently programmed for, or
	 * DWA_ADDR_UNCONFIGURED. Kept across transfers so a repeated transfer
	 * to the same device skips the disable/program/enable cycle: that
	 * cycle is 11 register accesses, and under tsisim every access is a
	 * socket round trip to the behavioural model. For a 1 Hz sensor poll
	 * it was 40% of all traffic.
	 *
	 * Invalidated on any error, so a failed transfer always reprograms
	 * from scratch rather than inheriting whatever state it left behind.
	 */
	u16			cfg_addr;
};

#define DWA_ADDR_UNCONFIGURED	0xffff

/* Operational-block accessors: fixed at offset 0. */
static inline u32 dwa_op_read(struct dwa_i2c_dev *d, u32 off)
{
	return readl(d->base + off);
}

static inline void dwa_op_write(struct dwa_i2c_dev *d, u32 off, u32 val)
{
	writel(val, d->base + off);
}

/* I2C-block accessors: relative to the programmable block base. */
static inline u32 dwa_read(struct dwa_i2c_dev *d, u32 off)
{
	return readl(d->base + d->blk + off);
}

static inline void dwa_write(struct dwa_i2c_dev *d, u32 off, u32 val)
{
	writel(val, d->base + d->blk + off);
}

static int dwa_wait_enable_state(struct dwa_i2c_dev *d, bool want_enabled)
{
	u32 val;

	return readl_poll_timeout(d->base + d->blk + DWA_IC_ENABLE_STATUS,
				  val,
				  !!(val & DWA_IC_ENABLE_EN) == want_enabled,
				  DWA_POLL_INTERVAL_US, DWA_ENABLE_TIMEOUT_US);
}

static int dwa_disable(struct dwa_i2c_dev *d)
{
	dwa_op_write(d, DWA_IC_ENABLE, 0);
	return dwa_wait_enable_state(d, false);
}

static int dwa_enable(struct dwa_i2c_dev *d)
{
	dwa_op_write(d, DWA_IC_ENABLE, DWA_IC_ENABLE_EN);
	return dwa_wait_enable_state(d, true);
}

static u32 dwa_speed_bits(u32 bus_freq_hz)
{
	if (bus_freq_hz > I2C_MAX_FAST_MODE_PLUS_FREQ)
		return DWA_IC_CTRL_SPEED_HIGH;
	if (bus_freq_hz > I2C_MAX_STANDARD_MODE_FREQ)
		return DWA_IC_CTRL_SPEED_FAST;
	return DWA_IC_CTRL_SPEED_STANDARD;
}

/*
 * Bring the controller up in controller mode at the configured speed and
 * target address. Mirrors the DV bring-up order: disable, program, enable.
 * Every I2C-block register is write-protected while enabled.
 */
static int dwa_configure(struct dwa_i2c_dev *d, u16 target_addr)
{
	u32 ctrl;
	int ret;

	ret = dwa_disable(d);
	if (ret) {
		dev_err(d->dev, "controller stuck enabled\n");
		return ret;
	}

	ctrl = DWA_IC_CTRL_OP_MODE | dwa_speed_bits(d->bus_freq_hz) |
	       DWA_IC_CTRL_TX_EMPTY_CTRL;
	dwa_write(d, DWA_IC_CTRL, ctrl);
	dwa_write(d, DWA_IC_TAR, target_addr & 0x3ff);
	dwa_write(d, DWA_IC_SCL_HCNT, d->scl_hcnt);
	dwa_write(d, DWA_IC_SCL_LCNT, d->scl_lcnt);
	dwa_write(d, DWA_IC_SDA_HOLD, DWA_DEFAULT_SDA_HOLD);
	dwa_write(d, DWA_IC_SPKLEN, DWA_DEFAULT_SPKLEN);

	/* Polled driver: keep every source masked. */
	dwa_write(d, DWA_IC_INTR_MASK, 0);

	return dwa_enable(d);
}

/* Map a termination cause onto an errno the I2C core understands. */
static int dwa_trmnt_to_errno(struct dwa_i2c_dev *d, u32 trmnt)
{
	if (trmnt & DWA_TRMNT_ADDR_NOACK_MASK) {
		dev_dbg(d->dev, "address NACK (IC_TX_TRMNT_SOURCE 0x%08x)\n",
			trmnt);
		return -ENXIO;
	}
	if (trmnt & DWA_TRMNT_TXDATA_NOACK) {
		dev_dbg(d->dev, "data NACK (IC_TX_TRMNT_SOURCE 0x%08x)\n",
			trmnt);
		return -EIO;
	}
	if (trmnt & DWA_TRMNT_ARB_LOST) {
		dev_dbg(d->dev, "arbitration lost\n");
		return -EAGAIN;
	}
	dev_dbg(d->dev, "transfer terminated, IC_TX_TRMNT_SOURCE 0x%08x\n",
		trmnt);
	return -EIO;
}

/* Wait for the controller to go idle with the TX FIFO drained. */
static int dwa_wait_idle(struct dwa_i2c_dev *d)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(d->base + d->blk + DWA_IC_STATUS, status,
				 !(status & (DWA_IC_STATUS_ACTIVITY |
					     DWA_IC_STATUS_CTRLR_ACTIVITY)) &&
				 (status & DWA_IC_STATUS_TFE),
				 DWA_POLL_INTERVAL_US, DWA_XFER_TIMEOUT_US);
	if (ret)
		dev_err(d->dev, "timeout waiting for idle (IC_STATUS 0x%08x)\n",
			readl(d->base + d->blk + DWA_IC_STATUS));
	return ret;
}

/* Issue one IC_DATA_CMD and wait for it to retire. */
static int dwa_issue(struct dwa_i2c_dev *d, u32 cmd)
{
	u32 trmnt;
	int ret;

	dwa_write(d, DWA_IC_DATA_CMD, cmd);

	ret = dwa_wait_idle(d);
	if (ret)
		return ret;

	trmnt = dwa_read(d, DWA_IC_TX_TRMNT_SOURCE);
	if (trmnt)
		return dwa_trmnt_to_errno(d, trmnt);

	return 0;
}

static int dwa_read_byte(struct dwa_i2c_dev *d, u8 *out)
{
	u32 rxflr;
	int ret;

	ret = readl_poll_timeout(d->base + d->blk + DWA_IC_RXFLR, rxflr,
				 rxflr > 0, DWA_POLL_INTERVAL_US,
				 DWA_XFER_TIMEOUT_US);
	if (ret) {
		dev_err(d->dev, "timeout waiting for RX data\n");
		return ret;
	}

	*out = dwa_read(d, DWA_IC_DATA_CMD) & DWA_IC_DATA_CMD_DAT_MASK;
	return 0;
}

static int dwa_xfer_msg(struct dwa_i2c_dev *d, struct i2c_msg *msg, bool last)
{
	bool is_read = !!(msg->flags & I2C_M_RD);
	int ret;
	u16 i;

	for (i = 0; i < msg->len; i++) {
		u32 cmd = is_read ? DWA_IC_DATA_CMD_CMD : msg->buf[i];

		/* STOP goes on the final byte of the final message only. */
		if (last && i == msg->len - 1)
			cmd |= DWA_IC_DATA_CMD_STOP;

		ret = dwa_issue(d, cmd);
		if (ret)
			return ret;

		if (is_read) {
			ret = dwa_read_byte(d, &msg->buf[i]);
			if (ret)
				return ret;
		}
	}
	return 0;
}

static int dwa_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct dwa_i2c_dev *d = i2c_get_adapdata(adap);
	int ret = 0;
	int i;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_TEN) {
			dev_dbg(d->dev, "10-bit addressing not supported\n");
			return -EOPNOTSUPP;
		}

		/*
		 * IC_TAR is write-protected while enabled, so a change of
		 * target needs a disable/program/enable cycle. Both messages
		 * within one transfer and successive transfers to the same
		 * device skip it -- see cfg_addr.
		 */
		if (msgs[i].addr != d->cfg_addr) {
			ret = dwa_configure(d, msgs[i].addr);
			if (ret)
				goto out_err;
			d->cfg_addr = msgs[i].addr;
		}

		ret = dwa_xfer_msg(d, &msgs[i], i == num - 1);
		if (ret)
			goto out_err;
	}

	return num;

out_err:
	/*
	 * Force a full reprogram next time. Disabling the controller is also
	 * what clears any half-finished transaction and the FIFOs, so the next
	 * dwa_configure() doubles as the recovery path.
	 */
	d->cfg_addr = DWA_ADDR_UNCONFIGURED;
	return ret;
}

static u32 dwa_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm dwa_algo = {
	.xfer = dwa_xfer,
	.functionality = dwa_func,
};

static int dwa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwa_i2c_dev *d;
	u32 comp_type, hci_version;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->dev = dev;
	d->cfg_addr = DWA_ADDR_UNCONFIGURED;

	d->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);

	if (device_property_read_u32(dev, "clock-frequency", &d->bus_freq_hz))
		d->bus_freq_hz = I2C_MAX_FAST_MODE_FREQ;

	if (device_property_read_u32(dev, "tsi,scl-hcnt", &d->scl_hcnt))
		d->scl_hcnt = DWA_DEFAULT_SCL_HCNT;
	if (device_property_read_u32(dev, "tsi,scl-lcnt", &d->scl_lcnt))
		d->scl_lcnt = DWA_DEFAULT_SCL_LCNT;

	/*
	 * Establish the I2C block base before any I2C-block access. It reads
	 * 0x20 out of reset, but the DV bring-up writes it explicitly and so
	 * does this driver -- the register is writable, so a prior user could
	 * have moved it.
	 */
	dwa_op_write(d, DWA_IC_I2C_BLOCK_OFFSET, DWA_I2C_BLOCK_DEFAULT_OFFSET);
	d->blk = dwa_op_read(d, DWA_IC_I2C_BLOCK_OFFSET) & 0xff;
	if (d->blk != DWA_I2C_BLOCK_DEFAULT_OFFSET) {
		dev_err(dev, "IC_I2C_BLOCK_OFFSET reads 0x%02x, expected 0x%02x\n",
			d->blk, DWA_I2C_BLOCK_DEFAULT_OFFSET);
		return -ENODEV;
	}

	ret = dwa_disable(d);
	if (ret) {
		dev_err(dev, "controller does not respond to IC_ENABLE\n");
		return ret;
	}

	comp_type = dwa_read(d, DWA_IC_COMP_TYPE);
	if (comp_type != DWA_IC_COMP_TYPE_VALUE) {
		dev_err(dev, "bad IC_COMP_TYPE 0x%08x (expected 0x%08x)\n",
			comp_type, DWA_IC_COMP_TYPE_VALUE);
		return -ENODEV;
	}

	/*
	 * IC_COMP_TYPE is identical on DW_apb_i2c, so it cannot tell the two
	 * generations apart. IC_HCI_VERSION exists only on DWC_i2c and reads
	 * 0x100 here; log it so a mis-targeted driver is obvious in dmesg.
	 */
	hci_version = dwa_op_read(d, DWA_IC_HCI_VERSION);
	dev_info(dev, "DWC_i2c HCI version 0x%08x, COMP_VERSION 0x%08x\n",
		 hci_version, dwa_read(d, DWA_IC_COMP_VERSION));

	i2c_set_adapdata(&d->adap, d);
	strscpy(d->adap.name, "TSI SkyLP DWC_i2c adapter", sizeof(d->adap.name));
	d->adap.owner = THIS_MODULE;
	d->adap.algo = &dwa_algo;
	d->adap.dev.parent = dev;
	d->adap.dev.of_node = dev->of_node;

	platform_set_drvdata(pdev, d);

	ret = devm_i2c_add_adapter(dev, &d->adap);
	if (ret)
		return ret;

	dev_info(dev, "%u Hz, SCL hcnt/lcnt %u/%u, polled\n",
		 d->bus_freq_hz, d->scl_hcnt, d->scl_lcnt);
	return 0;
}

static const struct of_device_id dwa_of_match[] = {
	{ .compatible = "tsi,skylp-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwa_of_match);

static struct platform_driver dwa_driver = {
	.probe = dwa_probe,
	.driver = {
		.name = DWA_DRV_NAME,
		.of_match_table = dwa_of_match,
	},
};
module_platform_driver(dwa_driver);

MODULE_AUTHOR("Sanjay R Mehta <smehta@tsavoritesi.com>");
MODULE_DESCRIPTION("Synopsys DWC_i2c (Advanced I2C/SMBus) driver for TSI SkyLP");
MODULE_LICENSE("GPL");
