#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/ztest.h>

#include <argos-smd/argos_smd.h>

const struct device *dev = DEVICE_DT_GET_ONE(arribada_argossmd);

void array_to_string(uint8_t *buf, char *str, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		str[i] = (char)buf[i];
	}
	str[len-1] = '\0'; // Remove renew line and null-terminate the string
}

char response[ARGOS_SMD_BUF_SIZE];

void read_callback(uint8_t *buf, size_t len, void *user_data)
{
	array_to_string(buf, response, len);
	TC_PRINT("%s", response);
}


ZTEST(argos_smd, test_write)
{
	zassert_ok(argos_set_address(dev, "00000000"));
	zassert_ok(argos_read_address(dev));
  zassert_equal(strncmp("+ADDR=000000", response, strlen("+ADDR=000000")), 0); 

	zassert_ok(argos_set_address(dev, "abcdef01"));
	zassert_ok(argos_read_address(dev));
  zassert_equal(strncmp("+ADDR=abcdef01", response, strlen("+ADDR=abcdef01")), 0); 
}


ZTEST(argos_smd, test_read)
{
    char msg[9] = "FFFFFFFF";
    zassert_ok(argos_send_payload(dev, msg));
    char expected[] = "+OK";
    zassert_equal(strncmp(expected, response, strlen(expected)), 0); 
}

void *setup(void)
{
	zassert_true(device_is_ready(dev));
	argos_smd_set_callback(dev, read_callback, NULL);

	return NULL;
}

ZTEST_SUITE(argos_smd, NULL, setup, NULL, NULL, NULL);

// Required for the ACM CDC UART to start and connected to
// before twister starts looking at it
int uart_delay()
{
    k_busy_wait(3000000); // 3s delay;
    return 0;
}

SYS_INIT_NAMED(uart_delay, uart_delay, APPLICATION, 95);
