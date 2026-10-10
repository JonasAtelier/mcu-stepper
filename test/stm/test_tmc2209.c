/*
 * Host-side check of the TMC2209 driver on the STM32 HAL: the fake chip and
 * the cases are ../tmc2209_cases.h; this file puts the chip on blocking
 * HAL_UART_Transmit()/Receive(), with a single wire modelled down to the
 * one-byte data register, and checks that no echo ever overran it.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "stepper.h"
#include "stepper_priv.h"
#include "tmc2209.h"

#define RET(err)	(err)
#define LOG_PREFIX(level)	""

static UART_HandleTypeDef huart;
static GPIO_TypeDef en_port;
static bool line_echo = true, line_fail;
static int level = -1;

/*
 * What the receiver holds: rdr is the data register, then the bytes still
 * on their way -- the chip's reply, which arrives after the echo and is read
 * as it comes. An echoed byte that finds the register full is lost: ORE.
 */
static uint8_t line[256];
static size_t line_len;
static int overruns;

static tmc2209_config_t base_cfg(void)
{
	tmc2209_config_t c = {
		.uart = &huart,
		.rsense_mohm = 110.0f,
	};

	return c;
}

static void cfg_en(tmc2209_config_t *c)
{
	c->en_port = &en_port;
	c->en_pin = GPIO_PIN_15;
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

int stub_uart_get_flag(UART_HandleTypeDef *h, uint32_t flag)
{
	assert(h == &huart && flag == UART_FLAG_RXNE);
	return line_len > 0;
}

void stub_uart_clear_ore(UART_HandleTypeDef *h)
{
	assert(h == &huart);
}

void stub_uart_flush(UART_HandleTypeDef *h)
{
	assert(h == &huart && line_len);
	memmove(line, line + 1, --line_len);
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *data,
				    uint16_t size, uint32_t t)
{
	(void)t;
	assert(h == &huart);
	if (line_fail)
		return HAL_ERROR;
	for (uint16_t i = 0; i < size; i++) {
		uint8_t reply[TMC2209_REPLY_LEN];
		size_t n;

		if (line_echo) {
			if (line_len)
				overruns++;	/* the echo is lost */
			else
				line_put(&data[i], 1);
		}
		n = chip_feed(data[i], reply);
		if (n)
			line_put(reply, n);
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *data,
				   uint16_t size, uint32_t t)
{
	(void)t;
	assert(h == &huart);
	if (line_len < size)
		return HAL_TIMEOUT;
	memcpy(data, line, size);
	memmove(line, line + size, line_len - size);
	line_len -= size;
	return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
	assert(port == &en_port && pin == GPIO_PIN_15);
	level = state == GPIO_PIN_SET;
}

static void test_null_handle(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h = NULL;

	chip_reset();
	c.uart = NULL;
	allocs = frees = 0;
	assert(tmc2209_init(&c, &h) == -EINVAL);
	assert(!h && allocs == 0 && chip_reads == 0);
}

int main(void)
{
	run_tmc2209_cases();
	test_null_handle();
	/* Every echo was taken before the next byte went out. */
	assert(overruns == 0);
	printf("all tmc2209 checks passed\n");
	return 0;
}
