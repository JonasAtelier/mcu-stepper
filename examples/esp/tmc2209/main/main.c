/*
 * Spin a stepper with no STEP pin at all, using the TMC2209's own pulse
 * generator, and print what the driver reports back.
 *
 * Wiring: MCU TX through a 1 kOhm resistor to PDN_UART, MCU RX straight to
 * PDN_UART. That is the standard single-wire arrangement. The UART is the
 * app's: install and configure it, then hand the driver the port.
 */
#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepper.h"
#include "stepper_log.h"
#include "tmc2209.h"

#define UART_PORT   UART_NUM_1
#define TX_GPIO     GPIO_NUM_17
#define RX_GPIO     GPIO_NUM_16
#define EN_GPIO     GPIO_NUM_4
#define BAUD        115200
#define UART_BUF    256

#define MICROSTEPS  16
#define FULL_STEPS  200                         /* a 1.8 degree motor */
#define ONE_REV_S   (FULL_STEPS * MICROSTEPS)   /* microsteps in one turn */

static const char *TAG = "tmc2209-example";

static void uart_setup(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF, UART_BUF, 0,
                                        NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_GPIO, RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void)
{
    tmc2209_config_t cfg = {
        .uart            = UART_PORT,
        .address         = 0,          /* MS1 and MS2 both low */
        .en_gpio         = EN_GPIO,
        .rsense_mohm     = 110.0f,     /* check your board */
        .run_current_ma  = 600,        /* RMS. Start below the motor's rating */
        .hold_current_ma = 200,
        .microsteps      = MICROSTEPS,
        .interpolate     = true,
        .spreadcycle     = false,      /* StealthChop: quiet */
    };
    tmc2209_handle_t drv;
    struct stepper_dev *motor;

    uart_setup();
    ESP_ERROR_CHECK(tmc2209_init(&cfg, &drv));
    motor = tmc2209_stepper(drv);      /* the same chip, as any stepper */
    ESP_ERROR_CHECK(tmc2209_enable(drv, true));

    while (1) {
        struct stepper_status st;
        uint16_t load;
        int dir;

        for (dir = 1; dir >= -1; dir -= 2) {
            /* One revolution per second, straight from the chip. */
            ESP_ERROR_CHECK(tmc2209_move(drv, dir * (float)ONE_REV_S));

            for (int i = 0; i < 10; i++) {
                vTaskDelay(pdMS_TO_TICKS(200));

                if (stepper_read_status(motor, &st))
                    continue;

                if (st.overtemp || st.short_to_gnd || st.short_to_supply) {
                    ESP_LOGE(TAG, "driver fault, stopping");
                    tmc2209_stop(drv);
                    stepper_enable(motor, false);
                    return;
                }

                if (st.was_reset)
                    ESP_LOGW(TAG, "the driver restarted - settings were lost");

                stepper_log_status(motor, &st);
                if (tmc2209_stallguard(drv, &load) == ESP_OK)
                    ESP_LOGI(TAG, "load %4u", load);
            }
        }

        ESP_ERROR_CHECK(tmc2209_stop(drv));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
