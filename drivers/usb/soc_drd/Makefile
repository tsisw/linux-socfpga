# SPDX-License-Identifier: GPL-2.0
# define_trace.h needs to know how to find our header
CFLAGS_soc_trace.o				:= -I$(src)

obj-$(CONFIG_USB_SOC_DRD)			+= soc_usb.o

soc_usb-y					:= soc_core.o

ifneq ($(CONFIG_TRACING),)
	soc_usb-y				+= soc_trace.o
endif

ifneq ($(or $(CONFIG_USB_SOC_DRD_HOST),$(CONFIG_USB_SOC_DRD_DUAL_ROLE)),)
	soc_usb-y				+= soc_host.o
endif

ifneq ($(or $(CONFIG_USB_SOC_DRD_GADGET),$(CONFIG_USB_SOC_DRD_DUAL_ROLE)),)
	soc_usb-y				+= soc_gadget.o soc_gadget_ep0.o
endif

ifneq ($(CONFIG_DEBUG_FS),)
	soc_usb-y				+= soc_debugfs.o
endif

obj-$(CONFIG_USB_SOC_DRD_SKYLP)			+= soc_skylp_core.o
obj-$(CONFIG_USB_SOC_DRD_SKYLP_TYPEC)		+= soc_skylp_typec.o
obj-$(CONFIG_USB_SOC_DRD_SKYLP_KUNIT_TEST)	+= soc_skylp-test.o
