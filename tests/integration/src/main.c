#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/ztest.h>

#include <argos-smd/argos_smd.h>

const struct device *dev = DEVICE_DT_GET_ONE(arribada_argossmd);

#define RESPONSE_BUFFER_SIZE 128
K_MSGQ_DEFINE(response_msgq, RESPONSE_BUFFER_SIZE, 10, 1);

void read_callback(const char *response, void *user_data)
{
	ARG_UNUSED(user_data);

	char out[RESPONSE_BUFFER_SIZE];
	strcpy(out, response);
	k_msgq_put(&response_msgq, out, K_NO_WAIT);
}

ZTEST(argos_smd, test_write)
{
	char response[RESPONSE_BUFFER_SIZE];

	zassert_ok(argos_set_address(dev, "00000000"));
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+OK", response), 0, "%s", response);

	zassert_ok(argos_read_address(dev));
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+ADDR=00000000", response), 0, "%s", response);
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+OK", response), 0, "%s", response);

	zassert_ok(argos_set_address(dev, "abcdef01"));
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+OK", response), 0, "%s", response);

	zassert_ok(argos_read_address(dev));
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+ADDR=abcdef01", response), 0);
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+OK", response), 0, "%s", response);
}

ZTEST(argos_smd, test_read)
{
	char response[RESPONSE_BUFFER_SIZE];

	char msg[9] = "FFFFFFFF";
	zassert_ok(argos_send_payload(dev, msg));

	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+OK", response), 0, "%s", response);
	k_msgq_get(&response_msgq, response, K_FOREVER);
	zassert_equal(strcmp("+TX=0,FFFFFFFF", response), 0, "%s", response);
}

void *setup(void)
{
	zassert_true(device_is_ready(dev));
	argos_smd_set_callback(dev, read_callback, NULL);

	/* Enable wakeup pin for all tests */
	int ret = argos_smd_wakeup_enable(dev);
	if (ret == 0) {
		printk("Wakeup pin enabled for tests\n");
	} else if (ret == -ENOTSUP) {
		printk("No wakeup pin configured\n");
	}

	return NULL;
}

void teardown(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Disable wakeup pin after all tests */
	int ret = argos_smd_wakeup_disable(dev);
	if (ret == 0) {
		printk("Wakeup pin disabled after tests\n");
	}
}

ZTEST_SUITE(argos_smd, NULL, setup, NULL, NULL, teardown);

// Required for the ACM CDC UART to start and connected to
// before twister starts looking at it
int uart_delay()
{
	k_busy_wait(3000000); // 5s delay;
	return 0;
}

SYS_INIT_NAMED(uart_delay, uart_delay, APPLICATION, 95);
