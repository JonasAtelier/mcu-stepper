/*
 * Host-side check of the TMC2209 driver on ESP-IDF: the fake chip and the
 * cases are ../tmc2209_cases.h; this file puts the chip on the UART driver's
 * calls, with the echo of a single wire, and adds what only ESP-IDF has.
 * esp-tmc2209-stepper had no tests.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "stepper.h"
#include "stepper_priv.h"
#include "tmc2209.h"

#define UART_PORT	1
#define EN_GPIO		21

#define RET(err)	stepper_esp_err(err)
#define LOG_PREFIX(level)	level " "

static bool line_echo = true, line_fail, installed = true;
static uint8_t line[256];	/* what waits in the RX ring buffer */
static size_t line_len;
static int level = -1;
static int uart_deletes;

static tmc2209_config_t base_cfg(void)
{
	tmc2209_config_t c = {
		.uart = UART_PORT,
		.en_gpio = GPIO_NUM_NC,
		.rsense_mohm = 110.0f,
	};

	return c;
}

static void cfg_en(tmc2209_config_t *c)
{
	c->en_gpio = EN_GPIO;
}

static int en_level(void)
{
	return level;
}

#include "../tmc2209_cases.h"

static void line_put(const uint8_t *b, size_t n)
{
	assert(line_len + n <= sizeof(line));
	memcpy(line + line_len, b, n);
	line_len += n;
}

bool uart_is_driver_installed(uart_port_t port)
{
	return installed && port == UART_PORT;
}

esp_err_t uart_flush_input(uart_port_t port)
{
	assert(port == UART_PORT);
	line_len = 0;
	return ESP_OK;
}

/* The bytes go out; on one wire they come straight back; the chip hears
 * them, and its reply follows them onto the line. */
int uart_write_bytes(uart_port_t port, const void *src, size_t size)
{
	const uint8_t *b = src;

	assert(port == UART_PORT);
	if (line_fail)
		return 0;
	for (size_t i = 0; i < size; i++) {
		uint8_t reply[TMC2209_REPLY_LEN];
		size_t n;

		if (line_echo)
			line_put(&b[i], 1);
		n = chip_feed(b[i], reply);
		if (n)
			line_put(reply, n);
	}
	return (int)size;
}

esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t ticks)
{
	(void)ticks;
	assert(port == UART_PORT);
	return ESP_OK;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
		    TickType_t ticks)
{
	size_t n = length < line_len ? length : line_len;

	(void)ticks;
	assert(port == UART_PORT);
	memcpy(buf, line, n);
	memmove(line, line + n, line_len - n);
	line_len -= n;
	return (int)n;
}

/* The app owns the UART: the driver must never delete it. */
esp_err_t uart_driver_delete(uart_port_t port);
esp_err_t uart_driver_delete(uart_port_t port)
{
	(void)port;
	uart_deletes++;
	return ESP_OK;
}

esp_err_t gpio_config(const gpio_config_t *c)
{
	assert(c->pin_bit_mask == 1ULL << EN_GPIO &&
	       c->mode == GPIO_MODE_OUTPUT);
	return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t g, uint32_t l)
{
	assert(g == EN_GPIO);
	level = (int)l;
	return ESP_OK;
}

/* A port whose driver the app never installed is refused, before anything
 * is sent; the UART is never deleted, on failure or on del. */
static void test_app_owns_uart(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h = NULL;

	chip_reset();
	installed = false;
	allocs = frees = 0;
	assert(tmc2209_init(&c, &h) == ESP_ERR_INVALID_ARG);
	assert(!h && allocs == frees && chip_reads == 0);
	installed = true;

	chip_silent = true;
	assert(tmc2209_init(&c, &h) == ESP_ERR_TIMEOUT);
	chip_silent = false;
	h = open_ok(&c);
	tmc2209_del(h);
	assert(uart_deletes == 0);
}

/* The original's error codes, through the map. */
static void test_esp_codes(void)
{
	assert(RET(-EBADMSG) == ESP_ERR_INVALID_CRC);
	assert(RET(-ENODEV) == ESP_ERR_NOT_FOUND);
	assert(RET(-EOPNOTSUPP) == ESP_ERR_NOT_SUPPORTED);
	assert(RET(-EIO) == ESP_FAIL);
}

int main(void)
{
	run_tmc2209_cases();
	test_app_owns_uart();
	test_esp_codes();
	printf("all tmc2209 checks passed\n");
	return 0;
}
