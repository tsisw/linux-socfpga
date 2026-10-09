soc_dp_drm-y := soc_dp_dri.o soc_dp_mst.o soc_dp_reg_ops.o
soc_dp_drm-$(CONFIG_SOC_DP_DRIVER_QEMU) += soc_dp_virtual_rx.o
# Linked into the module (vc4 pattern): the tables it validates are not
# exported across a module boundary.
soc_dp_drm-$(CONFIG_DRM_SOC_DP_KUNIT_TEST) += soc_dp_dri_test.o
obj-$(CONFIG_DRM_SOC_DP) += soc_dp_drm.o

obj-$(CONFIG_DRM_SOC_DP_PHY) += soc_dp_phy.o
