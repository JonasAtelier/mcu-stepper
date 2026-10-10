/*
 * TMC2209: spin a stepper from the chip's own pulse generator, no STEP pin,
 * and print what it reports.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * UART are the app's -- here huart2 and the ENN pin as CubeMX generates them
 * (MX_USART2_UART_Init(), MX_GPIO_Init()), at any baud the chip follows,
 * 115200 for one. Wiring is the usual single wire: TX through 1 kOhm to
 * PDN_UART, RX straight to it; the driver takes its own echo off the line a
 * byte at a time. Needs HAL_UART_MODULE_ENABLED and STEPPER_TMC2209 = y.
 */
#include CMSIS_device_header

#include "stepper.h"
#include "stepper_log.h"
#include "tmc2209.h"

#define EN_PORT		GPIOB		/* the app's choice, set up as an output */
#define EN_PIN		GPIO_PIN_0

#define MICROSTEPS	16
#define ONE_REV_S	(200 * MICROSTEPS)	/* a 1.8 degree motor */

extern UART_HandleTypeDef huart2;	/* from the app's MX_USART2_UART_Init() */

void stepper_task(void)
{
	const tmc2209_config_t cfg = {
		.uart = &huart2,
		.address = 0,		/* MS1 and MS2 both low */
		.en_port = EN_PORT,
		.en_pin = EN_PIN,
		.rsense_mohm = 110.0f,	/* check your board */
		.run_current_ma = 600,	/* RMS; start below the motor's rating */
		.hold_current_ma = 200,
		.microsteps = MICROSTEPS,
		.interpolate = true,
	};
	tmc2209_handle_t drv;
	struct stepper_dev *motor;

	/* -ETIMEDOUT: nothing answers; check the wire and the address. */
	if (tmc2209_init(&cfg, &drv))
		return;
	motor = tmc2209_stepper(drv);
	stepper_enable(motor, true);

	for (int dir = 1;; dir = -dir) {
		tmc2209_move(drv, dir * (float)ONE_REV_S);
		for (int i = 0; i < 10; i++) {
			struct stepper_status st;

			HAL_Delay(200);
			if (stepper_read_status(motor, &st))
				continue;
			stepper_log_status(motor, &st);
			if (st.overtemp || st.short_to_gnd || st.short_to_supply) {
				tmc2209_stop(drv);
				stepper_enable(motor, false);
				return;
			}
		}
	}
}
