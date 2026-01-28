# Copyright (c) 2025 Arribada Initiative
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(argos_ota_test)

# Generate firmware array from external file if specified
# Support both CMake variable (FIRMWARE_FILE) and Kconfig variable (CONFIG_ARGOS_OTA_FIRMWARE_FILE)
if(DEFINED FIRMWARE_FILE)
    set(FW_INPUT_FILE "${FIRMWARE_FILE}")
elseif(DEFINED CONFIG_ARGOS_OTA_FIRMWARE_FILE AND NOT "${CONFIG_ARGOS_OTA_FIRMWARE_FILE}" STREQUAL "")
    set(FW_INPUT_FILE "${CONFIG_ARGOS_OTA_FIRMWARE_FILE}")
endif()

if(DEFINED FW_INPUT_FILE AND NOT "${FW_INPUT_FILE}" STREQUAL "")
    set(FW_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/firmware_image.h")

    if(NOT EXISTS "${FW_INPUT_FILE}")
        message(FATAL_ERROR "Firmware file not found: ${FW_INPUT_FILE}")
    endif()

    # Run Python script to convert firmware to C array
    add_custom_command(
        OUTPUT ${FW_OUTPUT_FILE}
        COMMAND ${PYTHON_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/fw_to_array.py
                ${FW_INPUT_FILE}
                ${FW_OUTPUT_FILE}
        DEPENDS ${FW_INPUT_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/fw_to_array.py
        COMMENT "Generating firmware array from ${FW_INPUT_FILE}"
        VERBATIM
    )

    # Create custom target to ensure the header is generated
    add_custom_target(generate_fw_image DEPENDS ${FW_OUTPUT_FILE})

    # Add dependency to app
    add_dependencies(app generate_fw_image)

    # Add include directory for generated header
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    # Define macro to use external firmware
    target_compile_definitions(app PRIVATE USE_EXTERNAL_FIRMWARE=1)

    message(STATUS "Using external firmware: ${FW_INPUT_FILE}")
endif()

target_sources(app PRIVATE src/main.c)
