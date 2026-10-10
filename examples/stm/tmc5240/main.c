/*
 * TMC5240: let the chip's ramp generator move a motor back and forth, and
 * watch.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * SPI bus are the app's -- here hspi1, chip select and DRV_ENN as CubeMX
 * generates them (MX_SPI1_Init(), MX_GPIO_Init()). The handle must be in
 * mode 3 (CPOL high, CPHA second edge) with 8-bit frames, at 10 MHz or less;
 * chip select and DRV_ENN outputs, idle high. Needs HAL_SPI_MODULE_ENABLED
 * and STEPPER_TMC5240 = y; printing the floats needs -u _printf_float.
 *
 * No lock on the HAL: call one TMC5240 from one context only.
 */
#include <errno.h>
#include <stdio.h>

#include CMSIS_device_header

#include "stepper.h"
#include "stepper_log.h"
#include "tmc5240.h"

#define CS_PORT		GPIOA		/* the app's choices, set up as outputs */
#define CS_PIN		GPIO_PIN_4
#define EN_PORT		GPIOB
#define EN_PIN		GPIO_PIN_1

#define REV		(200 * 256)	/* microsteps in one turn */

extern SPI_HandleTypeDef hspi1;		/* from the app's MX_SPI1_Init() */

void stepper_task(void)
{
	const tmc5240_config_t cfg = {
		.spi = &hspi1,
		.cs_port = CS_PORT,
		.cs_pin = CS_PIN,
		.en_port = EN_PORT,
		.en_pin = EN_PIN,
		.rref_kohm = 12.0f,	/* the resistor fitted from IREF to GND */
		.run_current_a = 0.8f,	/* RMS; start low */
		.hold_current_a = 0.3f,
		.microsteps = 256,
		.stealthchop = true,
	};
	/* One rev/s, reached in half a second: a plain trapezoid. */
	const tmc5240_ramp_t ramp = {
		.vmax = 1.0f * REV,
		.amax = 2.0f * REV,
		.dmax = 2.0f * REV,
		.vstop = 100.0f,
		.tzerowait_ms = 50,
	};
	tmc5240_handle_t m;

	/* -ENODEV: no VERSION 0x40, usually wiring or the SPI mode. */
	if (tmc5240_init(&cfg, &m) || tmc5240_set_ramp(m, &ramp))
		return;

	for (int i = 0;; i++) {
		struct stepper_status st;

		tmc5240_move_to(m, (i % 2) ? 0 : 4 * REV);
		/* The chip runs the ramp; this only watches. */
		while (tmc5240_wait(m, 100) == -ETIMEDOUT) {
			int32_t x;

			if (!tmc5240_position(m, &x))
				printf("x=%ld\n", (long)x);
		}
		if (!stepper_read_status(tmc5240_stepper(m), &st))
			stepper_log_status(tmc5240_stepper(m), &st);
		HAL_Delay(500);
	}
}
