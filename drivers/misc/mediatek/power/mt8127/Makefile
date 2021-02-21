obj-$(CONFIG_MTK_PMIC)          += pmic_mt6323.o pmic_chr_type_det.o
# DTS2016111800622 yanfei/mwx333388 20161118 begin
obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o
ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_fan5405.o fan5405.o tbl_helper_func.o
else
ifeq ($(CONFIG_MTK_BQ24158_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24158.o bq24158.o tbl_helper_func.o
else
ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24296.o bq24296.o tbl_helper_func.o
else
ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24196.o bq24196.o tbl_helper_func.o
else
     obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
endif
endif
endif
endif
# DTS2016111800622 yanfei/mwx333388 20161118 end

