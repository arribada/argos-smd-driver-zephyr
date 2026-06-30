/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Functional tests for Argos SMD SPI driver.
 * Uses a Protocol A+ emulator backend to test the full driver stack.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/ztest.h>
#include <string.h>

#include <argos-smd/argos_smd_spi.h>

static const struct device *dev = DEVICE_DT_GET_ONE(arribada_argos_smd_spi);

ZTEST(argos_smd_spi, test_device_ready)
{
	zassert_true(device_is_ready(dev), "SPI device not ready");
}

ZTEST(argos_smd_spi, test_ping)
{
	zassert_ok(argos_spi_ping(dev), "Ping failed");
}

ZTEST(argos_smd_spi, test_get_version)
{
	char version[32];
	size_t len = sizeof(version);

	zassert_ok(argos_spi_get_version(dev, version, &len),
		   "Get version failed");
	zassert_true(len > 0, "Version length is 0");
	zassert_equal(strcmp(version, "1.0.0-test"), 0,
		      "Version mismatch: got '%s'", version);
}

ZTEST(argos_smd_spi, test_get_sn)
{
	char sn[32];
	size_t len = sizeof(sn);

	zassert_ok(argos_spi_get_sn(dev, sn, &len), "Get SN failed");
	zassert_true(len > 0, "SN length is 0");
	zassert_equal(strcmp(sn, "SN12345678"), 0,
		      "SN mismatch: got '%s'", sn);
}

ZTEST(argos_smd_spi, test_get_id)
{
	uint8_t id[4];
	size_t len = sizeof(id);

	zassert_ok(argos_spi_get_id(dev, id, &len), "Get ID failed");
	zassert_equal(len, 4, "ID length mismatch");
	zassert_equal(id[0], 0xDE);
	zassert_equal(id[1], 0xAD);
	zassert_equal(id[2], 0xBE);
	zassert_equal(id[3], 0xEF);
}

ZTEST(argos_smd_spi, test_get_addr)
{
	uint8_t addr[4];
	size_t len = sizeof(addr);

	zassert_ok(argos_spi_get_addr(dev, addr, &len), "Get addr failed");
	zassert_equal(len, 4, "Addr length mismatch");
	zassert_equal(addr[0], 0x01);
	zassert_equal(addr[1], 0x02);
	zassert_equal(addr[2], 0x03);
	zassert_equal(addr[3], 0x04);
}

ZTEST(argos_smd_spi, test_set_get_id)
{
	uint8_t new_id[] = {0xCA, 0xFE, 0xBA, 0xBE};
	uint8_t read_id[4];
	size_t len;

	zassert_ok(argos_spi_set_id(dev, new_id, sizeof(new_id)),
		   "Set ID failed");

	len = sizeof(read_id);
	zassert_ok(argos_spi_get_id(dev, read_id, &len),
		   "Get ID after set failed");
	zassert_equal(len, 4, "ID length mismatch");
	zassert_equal(memcmp(read_id, new_id, 4), 0,
		      "ID data mismatch after set");
}

ZTEST(argos_smd_spi, test_set_get_addr)
{
	uint8_t new_addr[] = {0xAA, 0xBB, 0xCC, 0xDD};
	uint8_t read_addr[4];
	size_t len;

	zassert_ok(argos_spi_set_addr(dev, new_addr, sizeof(new_addr)),
		   "Set addr failed");

	len = sizeof(read_addr);
	zassert_ok(argos_spi_get_addr(dev, read_addr, &len),
		   "Get addr after set failed");
	zassert_equal(len, 4, "Addr length mismatch");
	zassert_equal(memcmp(read_addr, new_addr, 4), 0,
		      "Addr data mismatch after set");
}

ZTEST_SUITE(argos_smd_spi, NULL, NULL, NULL, NULL, NULL);
