/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART-frame wake test: for SLEEP / STOP / STANDBY, force the mode with the wake
 * line (WKUP3/PB3) held LOW, then send a UART frame WITHOUT touching the wake
 * pin and see whether the module wakes and answers. Shallow modes (SLEEP/STOP)
 * keep the UART alive and should wake on RX activity; deep modes (STANDBY) power
 * the UART off and need the wake pin.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(uart_wake, LOG_LEVEL_INF);

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#define MAXB 200
static uint8_t rxbuf[MAXB];
static volatile int rxlen;

static void uart_cb(const struct device *dev, void *ud)
{
	ARG_UNUSED(ud);
	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		uint8_t c;
		if (uart_fifo_read(dev, &c, 1) == 1 && rxlen < MAXB) {
			rxbuf[rxlen++] = c;
		}
	}
}

static void send_str(const char *s)
{
	for (const char *p = s; *p; p++) {
		uart_poll_out(uart, (unsigned char)*p);
	}
}

/* Send a string, wait `ms`, return bytes received. */
static int cmd(const char *s, int ms)
{
	rxlen = 0;
	send_str(s);
	k_msleep(ms);
	return rxlen;
}

static void nrst_pulse(void)
{
	gpio_pin_set_raw(gpio0, 7, 0);
	k_msleep(20);
	gpio_pin_set_raw(gpio0, 7, 1);
	k_msleep(600);
}

static void test_mode(int bits, const char *name)
{
	char buf[32];

	LOG_INF("---------- %s (LPM=0x%02X) ----------", name, bits);

	/* Fresh module, wake line LOW the whole time. */
	gpio_pin_set_raw(gpio0, 26, 0);    /* WKUP3 LOW */
	nrst_pulse();

	if (cmd("AT+FW=?\r\n", 800) == 0) {
		LOG_WRN("  module not responsive at start, skipping");
		return;
	}

	/* Force the mode (wake line already LOW). */
	snprintf(buf, sizeof(buf), "AT+LPM=0x%X,0x%X\r\n", bits, bits);
	int n = cmd(buf, 1500);
	LOG_INF("  forced %s (set resp: %d bytes)", name, n);

	k_msleep(4000);    /* dwell in the mode, wake pin still LOW */

	/* Try to wake with a UART frame ONLY (wake pin stays LOW). */
	n = cmd("AT+FW=?\r\n", 1500);
	if (n > 0) {
		int j;
		for (j = 0; j < n && j < (int)sizeof(buf) - 1; j++) {
			buf[j] = (rxbuf[j] >= 0x20 && rxbuf[j] < 0x7f) ? rxbuf[j] : '.';
		}
		buf[j] = '\0';
		LOG_INF("  >>> UART-frame wake (NO pin): YES, woke -> %d bytes [%s]", n, buf);
	} else {
		LOG_INF("  >>> UART-frame wake (NO pin): NO response (needs the wake pin)");
	}

	/* Restore: raise wake pin to recover, clear LPM. */
	gpio_pin_set_raw(gpio0, 26, 1);
	k_msleep(700);
	cmd("AT+LPM=0x0,0x0\r\n", 500);
}

int main(void)
{
	struct uart_config cfg = {
		.baudrate = 9600, .parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1, .data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	LOG_INF("===== UART-frame wake test (no wake pin) =====");
	if (!device_is_ready(uart) || !device_is_ready(gpio0)) {
		LOG_ERR("not ready");
		return -ENODEV;
	}
	gpio_pin_configure(gpio0, 26, GPIO_OUTPUT_LOW);   /* wake line, start LOW */
	gpio_pin_configure(gpio0, 7, GPIO_OUTPUT_HIGH);   /* NRST released */
	uart_configure(uart, &cfg);
	uart_irq_callback_set(uart, uart_cb);
	uart_irq_rx_enable(uart);

	test_mode(0x01, "SLEEP");
	test_mode(0x02, "STOP");
	test_mode(0x04, "STANDBY");

	LOG_INF("===== test done =====");
	return 0;
}
