// SPDX-License-Identifier: GPL-2.0-only
/*
 * TSI SkyLP display controller: DRM master, CRTC and primary plane.
 *
 * The video bridge in the Innosilicon UDI subsystem scans a framebuffer out
 * to the DP controller. The DP encoder and connector live in a separate
 * component driver (soc,dp); this driver is the component master that
 * creates the drm_device and binds it, in the shape of drivers/gpu/drm/arm
 * (hdlcd). One node drives one channel of one bridge, SST only.
 *
 * Commit order matters here. The bridge must not start DMA before it has a
 * buffer address, and the pixel clock it runs on is set by the DP encoder
 * during link training. So the device uses the runtime-PM commit tail:
 *
 *	CRTC atomic_enable	program timing, stay stopped
 *	encoder atomic_enable	train link, set pixel clock, enable stream
 *	plane atomic_update	program the buffer, then start the channel
 *
 * The device is described by devicetree or by ACPI (PRP0001 with the same
 * compatible and property names, an _DSD graph for the port, and the
 * frame-done line as an Extended Interrupt whose ResourceSource is the UDI
 * collector). Everything here goes through fwnode so both work; only
 * crtc.port, which DRM types as a device_node, is set on devicetree, and the
 * encoder's possible_crtcs is filled in after bind when the vendor driver
 * could not derive it from an OF graph.
 *
 * Page flips after that are one idle-slot write and a buffer-id switch
 * (see tsi_skylp_vb.c). The frame-done interrupt is the vblank. Without an
 * interrupt in the devicetree (routing unconfirmed, or a model without the
 * bridge) the CRTC runs with no_vblank and events complete at commit time.
 *
 * Copyright (C) 2026 Tsavorite Scalable Intelligence
 */
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_of.h>
#include <drm/drm_plane.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "tsi_skylp_vb.h"

#define TSI_SKYLP_MAX_WIDTH	4096
#define TSI_SKYLP_MAX_HEIGHT	4096

struct tsi_skylp {
	struct drm_device drm;
	struct drm_crtc crtc;
	struct drm_plane plane;

	void __iomem *regs;
	struct tsi_vb_hw hw;
	struct tsi_vb_state st;
	/* serialises CTRL read-modify-write between commit and interrupt */
	spinlock_t lock;
	int irq;
	bool rmem;

	/* written only by the IRQ handler; read by debugfs scanout_stats */
	u64 frames;
	u64 underflows;
	u64 axi_errors;
};

static inline struct tsi_skylp *to_tsi(struct drm_device *drm)
{
	return container_of(drm, struct tsi_skylp, drm);
}

static u32 tsi_skylp_rd(void *ctx, u32 off)
{
	struct tsi_skylp *p = ctx;

	return readl(p->regs + off);
}

static void tsi_skylp_wr(void *ctx, u32 off, u32 val)
{
	struct tsi_skylp *p = ctx;

	writel(val, p->regs + off);
}

static const u32 tsi_skylp_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

/* ---------------------------------------------------------------- plane */

static int tsi_skylp_plane_atomic_check(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs;
	int ret;

	if (!ps->crtc)
		return 0;

	cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	ret = drm_atomic_helper_check_plane_state(ps, cs, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret || !ps->visible)
		return ret;

	return tsi_vb_check_scanout(&to_tsi(plane->dev)->st,
				    drm_fb_dma_get_gem_addr(ps->fb, ps, 0),
				    ps->fb->pitches[0], drm_rect_height(&ps->dst),
				    ps->fb->format->format);
}

static void tsi_skylp_plane_atomic_update(struct drm_plane *plane,
					  struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct tsi_skylp *p = to_tsi(plane->dev);
	dma_addr_t addr;
	unsigned long flags;
	unsigned int slot;
	bool first;
	u32 code;

	if (!ps->fb || !ps->visible)
		return;
	if (WARN_ON(tsi_vb_format_code(ps->fb->format->format, &code)))
		return;
	addr = drm_fb_dma_get_gem_addr(ps->fb, ps, 0);

	spin_lock_irqsave(&p->lock, flags);
	tsi_vb_set_scanout(&p->hw, &p->st, addr, ps->fb->pitches[0], code);
	slot = p->st.slot;
	first = !p->st.started;
	if (first)
		tsi_vb_start(&p->hw, &p->st);
	spin_unlock_irqrestore(&p->lock, flags);

	/* logged outside the lock: nothing printed with interrupts off */
	drm_dbg_kms(&p->drm, "scanout slot %u: %pad, pitch %u, %p4cc%s\n",
		    slot, &addr, ps->fb->pitches[0], &ps->fb->format->format,
		    first ? ", channel started" : "");
}

static const struct drm_plane_helper_funcs tsi_skylp_plane_helper_funcs = {
	.atomic_check	= tsi_skylp_plane_atomic_check,
	.atomic_update	= tsi_skylp_plane_atomic_update,
};

static const struct drm_plane_funcs tsi_skylp_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

/* ----------------------------------------------------------------- crtc */

static enum drm_mode_status
tsi_skylp_crtc_mode_valid(struct drm_crtc *crtc,
			  const struct drm_display_mode *mode)
{
	struct drm_display_mode m = *mode;
	struct tsi_vb_timing t;

	drm_mode_set_crtcinfo(&m, 0);
	return tsi_vb_timing_from_mode(&m, &t) ? MODE_BAD : MODE_OK;
}

static int tsi_skylp_crtc_atomic_check(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct tsi_vb_timing t;
	int ret;

	/*
	 * The bridge has no background colour: no scanout without a plane.
	 * no_vblank is already set by the helpers from drm_dev_has_vblank(),
	 * which here means "the frame-done interrupt is wired".
	 */
	ret = drm_atomic_helper_check_crtc_primary_plane(cs);
	if (ret)
		return ret;

	if (cs->enable && tsi_vb_timing_from_mode(&cs->adjusted_mode, &t))
		return -EINVAL;
	return 0;
}

static void tsi_skylp_crtc_atomic_enable(struct drm_crtc *crtc,
					 struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct tsi_skylp *p = to_tsi(crtc->dev);
	struct tsi_vb_timing t;

	if (WARN_ON(tsi_vb_timing_from_mode(&cs->adjusted_mode, &t)))
		return;

	drm_dbg_kms(&p->drm, "timing %ux%u, h %u/%u/%u, v %u/%u/%u (fp/sync/bp), sync %c/%c\n",
		    t.hactive, t.vactive, t.hfp, t.hsw, t.hbp, t.vfp, t.vsw, t.vbp,
		    t.hsync_pos ? '+' : '-', t.vsync_pos ? '+' : '-');

	/* the plane update starts the channel once it has a buffer */
	tsi_vb_set_timing(&p->hw, &t);
	if (p->irq)
		drm_crtc_vblank_on(crtc);
}

static void tsi_skylp_crtc_atomic_disable(struct drm_crtc *crtc,
					  struct drm_atomic_state *state)
{
	struct tsi_skylp *p = to_tsi(crtc->dev);
	unsigned long flags;

	if (p->irq)
		drm_crtc_vblank_off(crtc);

	spin_lock_irqsave(&p->lock, flags);
	tsi_vb_stop(&p->hw, &p->st);
	spin_unlock_irqrestore(&p->lock, flags);

	/* planes of an inactive CRTC are not flushed, so complete here */
	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void tsi_skylp_crtc_atomic_flush(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_pending_vblank_event *event = cs->event;

	/* with no_vblank the helpers fake the event after hw_done */
	if (!event || cs->no_vblank)
		return;

	cs->event = NULL;
	spin_lock_irq(&crtc->dev->event_lock);
	if (drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irq(&crtc->dev->event_lock);
}

static int tsi_skylp_enable_vblank(struct drm_crtc *crtc)
{
	struct tsi_skylp *p = to_tsi(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	tsi_vb_frame_done_irq(&p->hw, true);
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

static void tsi_skylp_disable_vblank(struct drm_crtc *crtc)
{
	struct tsi_skylp *p = to_tsi(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	tsi_vb_frame_done_irq(&p->hw, false);
	spin_unlock_irqrestore(&p->lock, flags);
}

static const struct drm_crtc_helper_funcs tsi_skylp_crtc_helper_funcs = {
	.mode_valid	= tsi_skylp_crtc_mode_valid,
	.atomic_check	= tsi_skylp_crtc_atomic_check,
	.atomic_flush	= tsi_skylp_crtc_atomic_flush,
	.atomic_enable	= tsi_skylp_crtc_atomic_enable,
	.atomic_disable	= tsi_skylp_crtc_atomic_disable,
};

static const struct drm_crtc_funcs tsi_skylp_crtc_funcs = {
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.enable_vblank		= tsi_skylp_enable_vblank,
	.disable_vblank		= tsi_skylp_disable_vblank,
};

static irqreturn_t tsi_skylp_irq(int irq, void *arg)
{
	struct tsi_skylp *p = arg;
	struct device *dev = p->drm.dev;
	u32 status;

	spin_lock(&p->lock);
	status = tsi_vb_irq_ack(&p->hw);
	spin_unlock(&p->lock);

	if (!status)
		return IRQ_NONE;

	if (status & TSI_VB_STATUS_UNDERFLOW) {
		WRITE_ONCE(p->underflows, p->underflows + 1);
		dev_warn_ratelimited(dev, "scanout FIFO underflow\n");
	}
	if (status & (TSI_VB_STATUS_ID_ERR | TSI_VB_STATUS_RESP_ERR)) {
		WRITE_ONCE(p->axi_errors, p->axi_errors + 1);
		dev_err_ratelimited(dev, "scanout AXI read error, status %#x\n",
				    status);
	}
	if (status & TSI_VB_STATUS_FRAME_DONE) {
		if (!p->frames)
			dev_info(dev, "first frame done: scanout DMA is running\n");
		WRITE_ONCE(p->frames, p->frames + 1);
		drm_crtc_handle_vblank(&p->crtc);
	}

	return IRQ_HANDLED;
}

static int tsi_skylp_stats_show(struct seq_file *m, void *unused)
{
	struct drm_debugfs_entry *entry = m->private;
	struct tsi_skylp *p = to_tsi(entry->dev);

	seq_printf(m, "frames_done\t%llu\nunderflows\t%llu\naxi_errors\t%llu\n",
		   READ_ONCE(p->frames), READ_ONCE(p->underflows),
		   READ_ONCE(p->axi_errors));
	seq_printf(m, "started\t%d\nslot\t%u\nirq\t%d\n",
		   READ_ONCE(p->st.started), READ_ONCE(p->st.slot), p->irq);
	return 0;
}

/* --------------------------------------------------------------- device */

static const struct drm_mode_config_funcs tsi_skylp_mode_config_funcs = {
	.fb_create	= drm_gem_fb_create,
	.atomic_check	= drm_atomic_helper_check,
	.atomic_commit	= drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs tsi_skylp_mode_config_helpers = {
	/* enables before planes: see the ordering note at the top */
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

DEFINE_DRM_GEM_DMA_FOPS(tsi_skylp_fops);

static const struct drm_driver tsi_skylp_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	DRM_GEM_DMA_DRIVER_OPS,
	.fops			= &tsi_skylp_fops,
	.name			= "tsi-skylp",
	.desc			= "TSI SkyLP display controller",
	.date			= "20260928",
	.major			= 1,
	.minor			= 0,
};

static int tsi_skylp_hw_init(struct tsi_skylp *p, struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *clks;
	int ret;

	p->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->regs))
		return PTR_ERR(p->regs);

	ret = devm_clk_bulk_get_all_enable(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(TSI_VB_DMA_ADDR_BITS));
	if (ret)
		return dev_err_probe(dev, ret, "no usable DMA mask\n");

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to claim memory-region\n");
	p->rmem = !ret;

	/*
	 * -ENXIO is the only "there is none": a malformed mapping or a
	 * parent that is not up yet must not silently bind without vblank.
	 */
	p->irq = platform_get_irq_optional(pdev, 0);
	if (p->irq == -ENXIO) {
		dev_info(dev, "no frame-done interrupt, running without vblank\n");
		p->irq = 0;
	} else if (p->irq < 0) {
		ret = dev_err_probe(dev, p->irq, "frame-done interrupt\n");
		goto err_rmem;
	}

	p->hw.rd = tsi_skylp_rd;
	p->hw.wr = tsi_skylp_wr;
	p->hw.ctx = p;
	p->hw.ch = 0;
	spin_lock_init(&p->lock);

	/* quiesce whatever a bootloader left running */
	tsi_vb_stop(&p->hw, &p->st);
	tsi_vb_irq_ack(&p->hw);

	dev_info(dev, "regs %pR, %u-bit DMA, buffers from %s, frame-done irq %d\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0), TSI_VB_DMA_ADDR_BITS,
		 p->rmem ? "the reserved memory-region" : "the default DMA pool",
		 p->irq);
	return 0;

err_rmem:
	if (p->rmem)
		of_reserved_mem_device_release(dev);
	return ret;
}

static int tsi_skylp_kms_init(struct tsi_skylp *p)
{
	struct drm_device *drm = &p->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 1;
	drm->mode_config.min_height = 1;
	drm->mode_config.max_width = TSI_SKYLP_MAX_WIDTH;
	drm->mode_config.max_height = TSI_SKYLP_MAX_HEIGHT;
	drm->mode_config.funcs = &tsi_skylp_mode_config_funcs;
	drm->mode_config.helper_private = &tsi_skylp_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &p->plane, 0, &tsi_skylp_plane_funcs,
				       tsi_skylp_formats,
				       ARRAY_SIZE(tsi_skylp_formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&p->plane, &tsi_skylp_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &p->crtc, &p->plane, NULL,
					&tsi_skylp_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&p->crtc, &tsi_skylp_crtc_helper_funcs);
	return 0;
}

static int tsi_skylp_bind(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tsi_skylp *p;
	struct drm_device *drm;
	struct drm_encoder *enc;
	int ret;

	p = devm_drm_dev_alloc(dev, &tsi_skylp_driver, struct tsi_skylp, drm);
	if (IS_ERR(p))
		return PTR_ERR(p);
	drm = &p->drm;

	ret = tsi_skylp_hw_init(p, pdev);
	if (ret)
		return ret;

	ret = tsi_skylp_kms_init(p);
	if (ret)
		goto err_rmem;

	/* on devicetree the encoder resolves possible_crtcs against this port */
	if (dev->of_node) {
		p->crtc.port = of_graph_get_port_by_id(dev->of_node, 0);
		if (!p->crtc.port) {
			ret = dev_err_probe(dev, -EINVAL, "no output port\n");
			goto err_rmem;
		}
	}

	ret = component_bind_all(dev, drm);
	if (ret) {
		dev_err_probe(dev, ret, "failed to bind the DP encoder\n");
		goto err_port;
	}

	/*
	 * Under ACPI the vendor encoder finds no OF graph and leaves
	 * possible_crtcs at 0, which drm_dev_register() rejects. There is
	 * exactly one CRTC here, so the answer is not in doubt.
	 */
	drm_for_each_encoder(enc, drm)
		if (!enc->possible_crtcs)
			enc->possible_crtcs = drm_crtc_mask(&p->crtc);

	if (p->irq) {
		ret = drm_vblank_init(drm, 1);
		if (ret)
			goto err_unbind;
		ret = devm_request_irq(dev, p->irq, tsi_skylp_irq, 0,
				       dev_name(dev), p);
		if (ret) {
			dev_err_probe(dev, ret, "failed to request irq\n");
			goto err_unbind;
		}
	}

	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);
	drm_debugfs_add_file(drm, "scanout_stats", tsi_skylp_stats_show, NULL);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_poll;

	dev_set_drvdata(dev, drm);
	drm_fbdev_dma_setup(drm, 32);
	return 0;

err_poll:
	drm_kms_helper_poll_fini(drm);
err_unbind:
	component_unbind_all(dev, drm);
err_port:
	of_node_put(p->crtc.port);
	p->crtc.port = NULL;
err_rmem:
	if (p->rmem)
		of_reserved_mem_device_release(dev);
	return ret;
}

static void tsi_skylp_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct tsi_skylp *p = to_tsi(drm);

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	/* disable through the encoder while it is still bound */
	drm_atomic_helper_shutdown(drm);
	component_unbind_all(dev, drm);
	of_node_put(p->crtc.port);
	p->crtc.port = NULL;
	if (p->rmem)
		of_reserved_mem_device_release(dev);
	dev_set_drvdata(dev, NULL);
}

static const struct component_master_ops tsi_skylp_master_ops = {
	.bind	= tsi_skylp_bind,
	.unbind	= tsi_skylp_unbind,
};

static int tsi_skylp_compare_fwnode(struct device *dev, void *data)
{
	return dev_fwnode(dev) == data;
}

static void tsi_skylp_release_fwnode(struct device *dev, void *data)
{
	fwnode_handle_put(data);
}

static int tsi_skylp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct fwnode_handle *remote;

	/* one output port, one endpoint: the DP encoder */
	remote = tsi_skylp_remote_encoder(dev_fwnode(dev));
	if (!remote)
		return dev_err_probe(dev, -ENODEV, "output port has no remote\n");

	component_match_add_release(dev, &match, tsi_skylp_release_fwnode,
				    tsi_skylp_compare_fwnode, remote);
	if (IS_ERR(match))
		return PTR_ERR(match);

	return component_master_add_with_match(dev, &tsi_skylp_master_ops, match);
}

static void tsi_skylp_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &tsi_skylp_master_ops);
}

static void tsi_skylp_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	if (drm)
		drm_atomic_helper_shutdown(drm);
}

static const struct of_device_id tsi_skylp_of_match[] = {
	{ .compatible = "tsi,skylp-video-bridge" },
	{ }
};
MODULE_DEVICE_TABLE(of, tsi_skylp_of_match);

static struct platform_driver tsi_skylp_platform_driver = {
	.probe		= tsi_skylp_probe,
	.remove		= tsi_skylp_remove,
	.shutdown	= tsi_skylp_shutdown,
	.driver	= {
		.name		= "tsi-skylp-display",
		.of_match_table	= tsi_skylp_of_match,
	},
};
module_platform_driver(tsi_skylp_platform_driver);

MODULE_DESCRIPTION("TSI SkyLP display controller (video bridge scanout)");
MODULE_LICENSE("GPL");
