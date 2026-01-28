# Copyright (c) 2025 Arribada Initiative
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kineis_spi_dfu_test)

# Add external firmware file if provided
if(DEFINED FIRMWARE_FILE)
    if(EXISTS ${FIRMWARE_FILE})
        message(STATUS "Using external firmware: ${FIRMWARE_FILE}")

        # Read the binary file and generate a C array
        file(READ ${FIRMWARE_FILE} FIRMWARE_HEX HEX)
        string(LENGTH ${FIRMWARE_HEX} FIRMWARE_HEX_LEN)
        math(EXPR FIRMWARE_SIZE "${FIRMWARE_HEX_LEN} / 2")

        # Generate header file with firmware data
        set(FIRMWARE_HEADER "${CMAKE_BINARY_DIR}/generated/firmware_image.h")
        file(WRITE ${FIRMWARE_HEADER} "/* Auto-generated firmware image */\n")
        file(APPEND ${FIRMWARE_HEADER} "#ifndef FIRMWARE_IMAGE_H\n#define FIRMWARE_IMAGE_H\n\n")
        file(APPEND ${FIRMWARE_HEADER} "#include <stdint.h>\n\n")
        file(APPEND ${FIRMWARE_HEADER} "#define FIRMWARE_IMAGE_SIZE ${FIRMWARE_SIZE}\n\n")
        file(APPEND ${FIRMWARE_HEADER} "static const uint8_t firmware_image[${FIRMWARE_SIZE}] = {\n    ")

        string(REGEX REPLACE "([0-9a-fA-F][0-9a-fA-F])" "0x\\1, " FIRMWARE_C_ARRAY ${FIRMWARE_HEX})
        string(REGEX REPLACE ", $" "" FIRMWARE_C_ARRAY ${FIRMWARE_C_ARRAY})

        # Add line breaks every 16 bytes
        string(REGEX REPLACE "((0x[0-9a-fA-F][0-9a-fA-F], ){16})" "\\1\n    " FIRMWARE_C_ARRAY ${FIRMWARE_C_ARRAY})

        file(APPEND ${FIRMWARE_HEADER} "${FIRMWARE_C_ARRAY}\n};\n\n")
        file(APPEND ${FIRMWARE_HEADER} "#endif /* FIRMWARE_IMAGE_H */\n")

        target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/generated)
        target_compile_definitions(app PRIVATE USE_EXTERNAL_FIRMWARE=1)
    else()
        message(FATAL_ERROR "Firmware file not found: ${FIRMWARE_FILE}")
    endif()
endif()

target_sources(app PRIVATE src/main.c)
