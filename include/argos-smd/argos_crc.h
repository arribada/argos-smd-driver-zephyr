/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARGOS_CRC_H
#define ARGOS_CRC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file argos_crc.h
 * @brief Shared CRC utilities for Argos SMD drivers
 *
 * This header provides common CRC functions used by both
 * UART DFU and SPI DFU implementations.
 */

/**
 * @brief Calculate CRC32 checksum
 *
 * Uses standard polynomial 0x04C11DB7 (reflected form 0xEDB88320).
 * Compatible with standard CRC-32 (Ethernet, ZIP, etc.)
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC32 checksum value
 */
uint32_t argos_dfu_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARGOS_CRC_H */
