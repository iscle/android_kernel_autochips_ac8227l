#
# Makefile for misc devices that really don't fit anywhere else.

define FILTER_PROJECT_DRV
ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/imgsensor/$(1)),)
platform_drv += $(1)
endif
endef

obj-y				+= kd_sensorlist.o
obj-y				+= camera_hw/
# DTS2016122308780 yangyale ywx429347 20161222 begin
obj-$(CONFIG_HUAWEI_DSM)	+= mtk_camera_dsm.o
# DTS2016122308780 yangyale ywx429347 20161222 end
$(foreach drv,$(subst $\",,$(CONFIG_CUSTOM_KERNEL_IMGSENSOR)),$(eval $(call FILTER_PROJECT_DRV,$(drv))))
custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(platform_drv)))
obj-y += $(custom_kernel_imgsensor)
$(info imgsensor drv by platform $(platform_drv))
