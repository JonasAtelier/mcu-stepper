/*
 * Host-side check of the TMC5240 driver on ESP-IDF against this library's
 * cases: the fake chip and the cases are ../tmc5240_cases.h; this file
 * feeds the chip from spi_device_polling_transmit() and adds what only
 * ESP-IDF has. esp-tmc5240-stepper's own test is esp/test_tmc5240.c.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "stepper.h"
#include "stepper_priv.h"
#include "tmc5240.h"

#define CS_GPIO		5
#define EN_GPIO		9

#define RET(err)	stepper_esp_err(err)

static bool bus_fail;
static int level = -1;
static int added;
static spi_device_interface_config_t devcfg;

static tmc5240_config_t base_cfg(void)
{
	tmc5240_config_t c = {
		.host = SPI2_HOST,
		.cs_gpio = CS_GPIO,
		.en_gpio = GPIO_NUM_NC,
		.rref_kohm = 12.0f,
		.run_current_a = 0.8f,
		.microsteps = 256,
	};

	return c;
}

static void cfg_en(tmc5240_config_t *c)
{
	c->en_gpio = EN_GPIO;
}

static int en_level(void)
{
	return level;
}

#include "../tmc5240_cases.h"

esp_err_t spi_bus_add_device(spi_host_device_t host,
			     const spi_device_interface_config_t *cfg,
			     spi_device_handle_t *out)
{
	assert(host == SPI2_HOST);
	devcfg = *cfg;
	added++;
	*out = (spi_device_handle_t)(void *)&devcfg;
	return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t dev)
{
	assert(dev == (spi_device_handle_t)(void *)&devcfg);
	added--;
	return ESP_OK;
}

/* Five bytes: past the four of tx_data, so through the buffers. */
esp_err_t spi_device_polling_transmit(spi_device_handle_t dev,
				      spi_transaction_t *t)
{
	assert(dev == (spi_device_handle_t)(void *)&devcfg);
	assert(t->length == 40 && t->flags == 0);
	if (bus_fail)
		return ESP_ERR_TIMEOUT;
	chip_frame(t->tx_buffer, t->rx_buffer);
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

/* Mode 3, the 4 MHz default or spi_hz, on the CS GPIO; removed again. */
static void test_device(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;

	chip_reset();
	h = open_ok(&c);
	assert(added == 1 && devcfg.mode == 3 &&
	       devcfg.clock_speed_hz == 4000000 &&
	       devcfg.spics_io_num == CS_GPIO);
	tmc5240_del(h);
	assert(added == 0);

	chip_reset();
	c.spi_hz = 8000000;
	h = open_ok(&c);
	assert(devcfg.clock_speed_hz == 8000000);
	tmc5240_del(h);

	/* A failed init removes what it added. */
	chip_reset();
	regs[TMC5240_INP_OUT] = 0;
	assert(tmc5240_init(&c, &h) == ESP_ERR_NOT_FOUND);
	assert(added == 0);
}

/* The original's codes, through the map. */
static void test_esp_codes(void)
{
	assert(RET(-EBUSY) == ESP_ERR_INVALID_STATE);
	assert(RET(-ENODATA) == ESP_ERR_INVALID_STATE);
	assert(RET(-ENODEV) == ESP_ERR_NOT_FOUND);
	assert(RET(-ETIMEDOUT) == ESP_ERR_TIMEOUT);
}

int main(void)
{
	run_tmc5240_cases();
	test_device();
	test_esp_codes();
	printf("all tmc5240 checks passed\n");
	return 0;
}
