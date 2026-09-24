// SPDX-License-Identifier: GPL-2.0
/*
 * soc_host.c - SOC USB3 DRD Controller Host Glue
 *
 */

#include <linux/irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "../host/xhci-plat.h"
#include "soc_core.h"

static void soc_usb_xhci_plat_start(struct usb_hcd *hcd)
{
	struct platform_device *pdev;
	struct soc_usb *su;

	if (!usb_hcd_is_primary_hcd(hcd))
		return;

	pdev = to_platform_device(hcd->self.controller);
	su = dev_get_drvdata(pdev->dev.parent);

	soc_usb_enable_susphy(su, true);
}

static const struct xhci_plat_priv soc_usb_xhci_plat_quirk = {
	.plat_start = soc_usb_xhci_plat_start,
};

static void soc_usb_host_fill_xhci_irq_res(struct soc_usb *su,
					int irq, char *name)
{
	struct platform_device *pdev = to_platform_device(su->dev);
	struct device_node *np = dev_of_node(&pdev->dev);

	su->xhci_resources[1].start = irq;
	su->xhci_resources[1].end = irq;
	su->xhci_resources[1].flags = IORESOURCE_IRQ | irq_get_trigger_type(irq);
	if (!name && np)
		su->xhci_resources[1].name = of_node_full_name(pdev->dev.of_node);
	else
		su->xhci_resources[1].name = name;
}

static int soc_usb_host_get_irq(struct soc_usb *su)
{
	struct platform_device	*soc_usb_pdev = to_platform_device(su->dev);
	int irq;

	irq = platform_get_irq_byname_optional(soc_usb_pdev, "host");
	if (irq > 0) {
		soc_usb_host_fill_xhci_irq_res(su, irq, "host");
		goto out;
	}

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq_byname_optional(soc_usb_pdev, "su_usb3");
	if (irq > 0) {
		soc_usb_host_fill_xhci_irq_res(su, irq, "su_usb3");
		goto out;
	}

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq(soc_usb_pdev, 0);
	if (irq > 0)
		soc_usb_host_fill_xhci_irq_res(su, irq, NULL);

out:
	return irq;
}

int soc_usb_host_init(struct soc_usb *su)
{
	struct property_entry	props[5];
	struct platform_device	*xhci;
	int			ret, irq;
	int			prop_idx = 0;

	irq = soc_usb_host_get_irq(su);
	if (irq < 0)
		return irq;

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(su->dev, "couldn't allocate xHCI device\n");
		return -ENOMEM;
	}

	xhci->dev.parent	= su->dev;

	su->xhci = xhci;

	ret = platform_device_add_resources(xhci, su->xhci_resources,
						SOC_USB_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(su->dev, "couldn't add resources to xHCI device\n");
		goto err;
	}

	memset(props, 0, sizeof(struct property_entry) * ARRAY_SIZE(props));

	props[prop_idx++] = PROPERTY_ENTRY_BOOL("xhci-sg-trb-cache-size-quirk");

	if (su->usb3_lpm_capable)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("usb3-lpm-capable");

	if (su->usb2_lpm_disable)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("usb2-lpm-disable");

	if (prop_idx) {
		ret = device_create_managed_software_node(&xhci->dev, props, NULL);
		if (ret) {
			dev_err(su->dev, "failed to add properties to xHCI\n");
			goto err;
		}
	}

	ret = platform_device_add_data(xhci, &soc_usb_xhci_plat_quirk,
				       sizeof(struct xhci_plat_priv));
	if (ret)
		goto err;

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(su->dev, "failed to register xHCI device\n");
		goto err;
	}

	if (su->sys_wakeup) {
		/* Restore wakeup setting if switched from device */
		device_wakeup_enable(su->sysdev);

		/* Pass on wakeup setting to the new xhci platform device */
		device_init_wakeup(&xhci->dev, true);
	}

	return 0;
err:
	platform_device_put(xhci);
	return ret;
}

void soc_usb_host_exit(struct soc_usb *su)
{
	if (su->sys_wakeup)
		device_init_wakeup(&su->xhci->dev, false);

	soc_usb_enable_susphy(su, false);
	platform_device_unregister(su->xhci);
	su->xhci = NULL;
}
