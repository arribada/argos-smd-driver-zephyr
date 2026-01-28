# Add argos-smd directory if UART or SPI driver is enabled
if(CONFIG_ARGOS_SMD OR CONFIG_KINEIS_SPI)
    add_subdirectory(argos-smd)
endif()