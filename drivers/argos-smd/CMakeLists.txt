zephyr_library()
zephyr_library_include_directories(.)

# UART-based Argos SMD driver
zephyr_library_sources_ifdef(CONFIG_ARGOS_SMD argos_smd.c)

# UART-based DFU support
zephyr_library_sources_ifdef(CONFIG_ARGOS_DFU argos_dfu.c)

# SPI-based Kineis driver and DFU
zephyr_library_sources_ifdef(CONFIG_KINEIS_SPI kineis_spi.c)
zephyr_library_sources_ifdef(CONFIG_KINEIS_DFU_SPI kineis_dfu_spi.c)
