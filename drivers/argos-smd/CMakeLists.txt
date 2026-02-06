# Copyright (c) 2025 Arribada Initiative
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${ZEPHYR_ARGOS_SMD_DRIVER_ZEPHYR_MODULE_DIR}/include)

# Shared CRC utilities (required by DFU)
zephyr_library_sources_ifdef(CONFIG_ARGOS_SMD_DFU argos_crc.c)

# UART transport
zephyr_library_sources_ifdef(CONFIG_ARGOS_SMD_UART argos_smd.c)
zephyr_library_sources_ifdef(CONFIG_ARGOS_SMD_UART
    $<$<BOOL:${CONFIG_ARGOS_SMD_DFU}>:argos_dfu.c>
)

# SPI transport
zephyr_library_sources_ifdef(CONFIG_ARGOS_SMD_SPI argos_smd_spi.c)
zephyr_library_sources_ifdef(CONFIG_ARGOS_SMD_SPI
    $<$<BOOL:${CONFIG_ARGOS_SMD_DFU}>:argos_smd_dfu_spi.c>
)
