/*
 * Host-side check of the TMC5240 driver on the STM32 HAL: the fake chip and
 * the cases are ../tmc5240_cases.h; this file feeds the chip from
 * HAL_SPI_TransmitReceive() under a software chip select, and adds what
 * only the HAL has: the handle's mode check and the chip select itself.
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
#include "tmc5240.h"

#define RET(err)	(err)

static SPI_HandleTypeDef hspi = {
	.Init = {
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_HIGH,
		.CLKPhase = SPI_PHASE_2EDGE,
	},
};
static GPIO_TypeDef cs_port, en_port;
static bool bus_fail;
static bool cs_low;
static int level = -1;
static int cs_frames;		/* frames that came with CS low */

static tmc5240_config_t base_cfg(void)
{
	tmc5240_config_t c = {
		.spi = &hspi,
		.cs_port = &cs_port,
		.cs_pin = GPIO_PIN_4,
		.rref_kohm = 12.0f,
		.run_current_a = 0.8f,
		.microsteps = 256,
	};

	return c;
}

static void cfg_en(tmc5240_config_t *c)
{
	c->en_port = &en_port;
	c->en_pin = GPIO_PIN_15;
}

static int en_level(void)
{
	return level;
}

#include "../tmc5240_cases.h"

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
	if (port == &en_port) {
		assert(pin == GPIO_PIN_15);
		level = state == GPIO_PIN_SET;
		return;
	}
	assert(port == &cs_port && pin == GPIO_PIN_4);
	/* Every fall has its rise: one frame per chip select. */
	assert(cs_low == (state == GPIO_PIN_SET) || state == GPIO_PIN_SET);
	cs_low = state == GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,
					  const uint8_t *tx, uint8_t *rx,
					  uint16_t size, uint32_t t)
{
	(void)t;
	assert(h == &hspi && size == 5 && cs_low);
	if (bus_fail)
		return HAL_TIMEOUT;
	cs_frames++;
	chip_frame(tx, rx);
	return HAL_OK;
}

/* A handle not in mode 3 with 8-bit frames is refused before the bus. */
static void test_handle_checks(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h = NULL;

	chip_reset();
	allocs = 0;
	hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	assert(tmc5240_init(&c, &h) == -EINVAL);
	hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	assert(tmc5240_init(&c, &h) == -EINVAL);
	hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi.Init.DataSize = SPI_DATASIZE_16BIT;
	assert(tmc5240_init(&c, &h) == -EINVAL);
	hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	c.spi = NULL;
	assert(tmc5240_init(&c, &h) == -EINVAL);
	c = base_cfg();
	c.cs_port = NULL;
	assert(tmc5240_init(&c, &h) == -EINVAL);
	assert(!h && allocs == 0 && frames == 0);
}

/* Every frame went under its own chip select, and it ends high. */
static void test_chip_select(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;
	uint32_t v;

	chip_reset();
	h = open_ok(&c);
	cs_frames = frames = 0;
	assert(tmc5240_read(h, TMC5240_XACTUAL, &v) == 0);
	assert(cs_frames == 2 && frames == 2 && !cs_low);
	/* High again after a failure. */
	bus_fail = true;
	assert(tmc5240_read(h, TMC5240_XACTUAL, &v) == -ETIMEDOUT);
	assert(!cs_low);
	bus_fail = false;
	tmc5240_del(h);
}

int main(void)
{
	run_tmc5240_cases();
	test_handle_checks();
	test_chip_select();
	printf("all tmc5240 checks passed\n");
	return 0;
}
