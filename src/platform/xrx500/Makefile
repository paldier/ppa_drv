ifneq ($(CONFIG_PPA_ACCEL),)
obj-m	+= ltq_pae_hal.o
endif
ifeq ($(CONFIG_PPA_TMU_HAL),m)
obj-m	+= ltq_tmu_hal_drv.o
ltq_tmu_hal_drv-objs	+= ltq_tmu_hal_proc.o ltq_tmu_hal.o
endif

ccflags-y := -Werror

ifeq ($(CONFIG_PPA_MPE_HAL),m)
obj-m	+= ltq_mpe_hal_drv.o
ltq_mpe_hal_drv-objs	+= ltq_mpe_hal_proc.o ltq_mpe_tmpl_buf.o ltq_mpe_hal.o
endif
obj-$(CONFIG_LTQ_PPA_ETH_BOND)  += ltq_eth_bond_dp.o
