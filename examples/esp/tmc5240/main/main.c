/*
 * TMC5240: move a motor back and forth, printing what the chip reports.
 *
 * Wiring: an SPI bus of your choice plus CS, and DRV_ENN to a GPIO (or tie it
 * low and pass -1). Set RREF_KOHM to the resistor actually fitted from IREF to
 * GND -- the driver cannot measure it, and every current below scales with it.
 */
#include <stdio.h>

#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stepper.h"
#include "stepper_log.h"
#include "tmc5240.h"

#define PIN_SCK    12
#define PIN_MOSI   11
#define PIN_MISO   13
#define PIN_CS     10
#define PIN_ENN    9

#define RREF_KOHM  12.0f
#define FULLSTEPS  200      /* per revolution, from the motor's label */
#define USTEPS     256

/* One revolution, in the microsteps the ramp generator counts in. */
#define REV        (FULLSTEPS * USTEPS)

static const char *TAG = "example";

void app_main(void)
{
    spi_bus_config_t bus = {
        .sclk_io_num = PIN_SCK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    tmc5240_config_t cfg = {
        .host = SPI2_HOST,
        .cs_gpio = PIN_CS,
        .en_gpio = PIN_ENN,
        .rref_kohm = RREF_KOHM,
        .run_current_a = 0.8f,     /* RMS. Start low; a hot motor is a wrong one. */
        .hold_current_a = 0.3f,
        .microsteps = USTEPS,
        .stealthchop = true,
    };

    tmc5240_handle_t m;
    ESP_ERROR_CHECK(tmc5240_init(&cfg, &m));
    ESP_LOGI(TAG, "run current came out at %.3f A RMS", tmc5240_run_current(m));

    /* The same chip, as any stepper: the flags every chip here shares. */
    struct stepper_status common;
    if (stepper_read_status(tmc5240_stepper(m), &common) == 0)
        stepper_log_status(tmc5240_stepper(m), &common);

    /* One rev/s, reaching it in half a second. A plain trapezoid: leave v1 and
     * v2 at zero and the EightPoint segments stay out of the way. */
    tmc5240_ramp_t ramp = {
        .vmax = 1.0f * REV,
        .amax = 2.0f * REV,
        .dmax = 2.0f * REV,
        .vstop = 100.0f,
        .tzerowait_ms = 50,
    };
    ESP_ERROR_CHECK(tmc5240_set_ramp(m, &ramp));

    for (int i = 0;; i++) {
        int32_t target = (i % 2) ? 0 : 4 * REV;

        ESP_ERROR_CHECK(tmc5240_move_to(m, target));

        /* The chip runs the ramp on its own; this loop only watches. */
        while (tmc5240_wait(m, 100) == ESP_ERR_TIMEOUT) {
            tmc5240_status_t st;

            if (tmc5240_status(m, &st) != ESP_OK)
                continue;

            printf("x=%-9ld v=%-9.0f sg=%-4u cs=%-3u%s%s\n",
                   (long)st.position, st.velocity, st.sg_result, st.cs_actual,
                   st.overtemp_warn ? " OTPW" : "",
                   st.stallguard ? " STALL" : "");

            if (st.overtemp) {
                ESP_LOGE(TAG, "driver shut down on overtemperature");
                tmc5240_del(m);
                return;
            }
        }

        float volts = 0.0f, degc = 0.0f;
        tmc5240_supply_voltage(m, &volts);
        tmc5240_temperature(m, &degc);
        ESP_LOGI(TAG, "reached %ld, VS %.1f V, %.0f C", (long)target,
                 volts, degc);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
