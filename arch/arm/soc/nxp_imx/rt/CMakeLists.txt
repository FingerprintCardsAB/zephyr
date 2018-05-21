#
# Copyright (c) 2017, NXP
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_sources(
  soc.c
  )

zephyr_sources_ifdef(CONFIG_ARM_MPU_IMX_RT arm_mpu_regions.c)

zephyr_sources_ifdef(CONFIG_CODE_HYPERFLASH
  evkbimxrt1050_hyper_config.c
  evkbimxrt1050_sdram_ini_dcd.c
  fsl_flexspi_nor_boot.c
  )

