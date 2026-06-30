/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared CRC utilities for Argos SMD drivers
 */

#include <argos-smd/argos_crc.h>

/*
 * CRC32 calculation using STM32 polynomial (MSB-first, non-reflected)
 *
 * Polynomial: 0x04C11DB7 (CRC-32/MPEG-2)
 * Initial value: 0xFFFFFFFF
 * No final XOR, no reflection
 *
 * This matches the STM32WL hardware CRC unit configuration.
 */
uint32_t argos_dfu_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)data[i] << 24;
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80000000) {
				crc = (crc << 1) ^ 0x04C11DB7;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}
