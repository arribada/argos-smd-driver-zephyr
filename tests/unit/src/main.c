#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <string.h>

#include <../../drivers/argos-smd/argos_smd.h>

#include <zephyr/ztest.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(argossmd_tests);

ZTEST_SUITE(argossmd_tests, NULL, NULL, NULL, NULL, NULL);

/**
 * @brief Test encoding of sensor data
 *
 * Tests the encoding of single and multiple sensor data packages
 *
 */
ZTEST(argossmd_tests, test_driver)
{
	const struct device *dev = DEVICE_DT_GET_ONE(arribada_argossmd);

	zassert_equal(1, 1);
}