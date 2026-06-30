# Add argos-smd directory if driver is enabled (UART or SPI)
if(CONFIG_ARGOS_SMD)
    add_subdirectory(argos-smd)
endif()