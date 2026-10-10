/*
 * tmc2209 — the Trinamic/ADI TMC2209 stepper driver, as a stepper chip
 * driver, for ESP-IDF and the STM32 HAL.
 *
 * The cheap, quiet one. StealthChop makes a stepper close to silent, and the
 * single-wire UART lets you set current, microstepping and even move the motor
 * without a step pulse anywhere. StallGuard detects a stall, which is how you
 * home an axis with no switch.
 *
 * Two ways to move it:
 *   - STEP/DIR, as with any stepper driver. Generate the pulses yourself, with
 *     LEDC, MCPWM or RMT, or a timer — this driver deliberately does not.
 *   - tmc2209_move(), which hands a velocity to the chip's own pulse generator
 *     and needs no STEP pin at all. Slower to react, but nothing to generate.
 *
 * If you want a ramp generator in hardware — a target position and hardware
 * acceleration — that is its bigger sibling, the TMC5240.
 *
 * The UART is the app's, as every bus is in the mcu-* libraries: on ESP-IDF a
 * port it has installed and configured (uart_driver_install(),
 * uart_param_config(), uart_set_pin()), on the STM32 HAL a handle it has
 * HAL_UART_Init()ed. Baud rate and pins are set there. The ENN pin is the
 * driver's: one output per chip.
 *
 * tmc2209_stepper() hands out the struct stepper_dev inside the handle, for
 * stepper_enable(), stepper_set_current() and stepper_read_status(). Its
 * status has no stall flag: the TMC2209 reports a stall on its DIAG pin.
 *
 * Every register address, bit position and formula here is quoted from the
 * datasheet in docs/tmc2209.pdf (rev 1.08). Section 4 is the UART protocol,
 * section 5 the register map, section 9 the current formula.
 *
 * On ESP-IDF every call returns an esp_err_t, as esp-tmc2209-stepper did; on
 * the STM32 HAL, 0 or a negative errno.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#else
#include CMSIS_device_header
#endif

#include "stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc2209_t *tmc2209_handle_t;

typedef struct {
#ifdef ESP_PLATFORM
    uart_port_t uart;         /* the app has installed and configured it */
#else
    UART_HandleTypeDef *uart; /* the app has HAL_UART_Init()ed it */
#endif

    /* Which chip on the bus, 0 to 3, strapped on the MS1 and MS2 pins.
     * Several TMC2209s can share one wire if each has its own address. */
    uint8_t     address;

    /* Nearly every breakout wires one wire for both directions: MCU TX through
     * 1 kOhm to PDN_UART, MCU RX straight to it. The MCU therefore hears its
     * own transmission, and this driver swallows that echo. Set this only if
     * TX and RX really are separate nets. */
    bool        two_wire;

#ifdef ESP_PLATFORM
    gpio_num_t  en_gpio;      /* ENN, active low. GPIO_NUM_NC if tied to GND */
#else
    /* ENN, active low: an output the app has set up. en_port NULL if tied
     * to GND. */
    GPIO_TypeDef *en_port;
    uint16_t    en_pin;
#endif

    /* Sense resistors actually fitted, in milliohms. 110 on almost every
     * BigTreeTech-style breakout. Nothing can measure this for you: get it
     * wrong and every current below is wrong by the same ratio. */
    float       rsense_mohm;

    uint16_t    run_current_ma;   /* RMS, not peak. 0 selects 500 */
    uint16_t    hold_current_ma;  /* 0 selects half of run */

    uint16_t    microsteps;       /* 256,128,64,32,16,8,4,2,1. 0 selects 16 */
    bool        interpolate;      /* smooth coarse microsteps up to 256 */

    bool        spreadcycle;      /* false is StealthChop: quiet, less torque */
    bool        invert_direction;

    /* The clock the chip counts in. 0 means its internal oscillator, which
     * this driver then assumes is its 12 MHz nominal. That oscillator is not
     * trimmed, so every tmc2209_move() velocity can be several per cent out
     * with nothing reporting an error. */
    uint32_t    clock_hz;
} tmc2209_config_t;

/* DRV_STATUS and GSTAT, decoded. Datasheet section 5.5.3 and register 0x01. */
typedef struct {
    bool standstill;        /* no step for 2^20 clocks */
    bool stealthchop;       /* which chopper is running right now */
    bool overtemp_warning;  /* otpw: getting hot */
    bool overtemp;          /* ot: shut down until it cools */
    bool short_to_gnd;      /* s2ga/s2gb - driver disabled */
    bool short_low_side;    /* s2vsa/s2vsb - driver disabled */
    bool open_load;         /* ola/olb. Only meaningful turning slowly */
    bool undervoltage_cp;   /* charge pump undervoltage */
    bool was_reset;         /* the chip restarted and lost its settings */
    uint8_t current_scale;  /* CS_ACTUAL, 0-31: what it is really driving */
    uint32_t drv_status;    /* the raw register, for the per-phase bits */
} tmc2209_status_t;

#ifdef ESP_PLATFORM
/* Take the app's UART, prove the chip answers, and apply the configuration.
 * ESP_ERR_INVALID_ARG if the port's driver is not installed. Fails with
 * ESP_ERR_TIMEOUT if nothing replies, ESP_ERR_INVALID_CRC if a reply does
 * not check out, and ESP_ERR_NOT_FOUND if replies come but writes do not
 * land. */
esp_err_t tmc2209_init(const tmc2209_config_t *cfg, tmc2209_handle_t *out);

/* Turn the output stage on or off with the ENN pin. Off means the motor
 * freewheels and holds nothing. Needs en_gpio. */
esp_err_t tmc2209_enable(tmc2209_handle_t dev, bool enable);

/* Change the run and hold currents, RMS milliamps, while running. hold_ma of
 * 0 means half of run. */
esp_err_t tmc2209_set_current(tmc2209_handle_t dev, uint16_t run_ma,
                              uint16_t hold_ma);

/* Move at a constant velocity in microsteps per second, using the chip's own
 * pulse generator — no STEP pin needed. Negative runs the other way, 0 stops
 * and hands control back to the STEP input. */
esp_err_t tmc2209_move(tmc2209_handle_t dev, float usteps_per_s);

/* Same as tmc2209_move(dev, 0). */
esp_err_t tmc2209_stop(tmc2209_handle_t dev);

/* Read the driver's state. Also clears GSTAT, so was_reset reports each
 * restart exactly once. */
esp_err_t tmc2209_read_status(tmc2209_handle_t dev, tmc2209_status_t *out);

/* StallGuard load: 0 to 1023, and it falls as the motor loads up. Read it
 * while moving at the speed you intend to home at, and pick a threshold from
 * what you see. */
esp_err_t tmc2209_stallguard(tmc2209_handle_t dev, uint16_t *load);

/* Arm sensorless stall detection. DIAG goes high when the load drops to
 * threshold * 2, but only above min_usteps_per_s — StallGuard is meaningless
 * at low speed, so this floor matters. threshold 0 disarms it. */
esp_err_t tmc2209_set_stall(tmc2209_handle_t dev, uint8_t threshold,
                            float min_usteps_per_s);

/* Raw register access, for everything this API does not expose. */
esp_err_t tmc2209_read(tmc2209_handle_t dev, uint8_t reg, uint32_t *value);
esp_err_t tmc2209_write(tmc2209_handle_t dev, uint8_t reg, uint32_t value);

/* Leaves the stage off and frees the handle. The UART is the app's and is
 * left installed. */
esp_err_t tmc2209_del(tmc2209_handle_t dev);
#else
/* Take the app's UART handle, prove the chip answers, and apply the
 * configuration. -EINVAL for a NULL handle or a bad config. -ETIMEDOUT if
 * nothing replies, -EBADMSG if a reply does not check out, -ENODEV if
 * replies come but writes do not land. */
int tmc2209_init(const tmc2209_config_t *cfg, tmc2209_handle_t *out);

/* Turn the output stage on or off with the ENN pin. -EOPNOTSUPP without
 * en_port. */
int tmc2209_enable(tmc2209_handle_t dev, bool enable);

/* Change the run and hold currents, RMS milliamps, while running. hold_ma of
 * 0 means half of run. */
int tmc2209_set_current(tmc2209_handle_t dev, uint16_t run_ma,
                        uint16_t hold_ma);

/* Move at a constant velocity in microsteps per second, using the chip's own
 * pulse generator. Negative runs the other way, 0 stops and hands control
 * back to the STEP input. -EINVAL past the register's range. */
int tmc2209_move(tmc2209_handle_t dev, float usteps_per_s);

/* Same as tmc2209_move(dev, 0). */
int tmc2209_stop(tmc2209_handle_t dev);

/* Read the driver's state. Also clears GSTAT, so was_reset reports each
 * restart exactly once. */
int tmc2209_read_status(tmc2209_handle_t dev, tmc2209_status_t *out);

/* StallGuard load: 0 to 1023, falling as the motor loads up. */
int tmc2209_stallguard(tmc2209_handle_t dev, uint16_t *load);

/* Arm sensorless stall detection above min_usteps_per_s; threshold 0
 * disarms it. */
int tmc2209_set_stall(tmc2209_handle_t dev, uint8_t threshold,
                      float min_usteps_per_s);

/* Raw register access, for everything this API does not expose. */
int tmc2209_read(tmc2209_handle_t dev, uint8_t reg, uint32_t *value);
int tmc2209_write(tmc2209_handle_t dev, uint8_t reg, uint32_t value);

/* Leaves the stage off and frees the handle. The UART is the app's. */
int tmc2209_del(tmc2209_handle_t dev);
#endif

/* The handle as a struct stepper_dev; NULL for a NULL handle. Valid until
 * tmc2209_del(). */
struct stepper_dev *tmc2209_stepper(tmc2209_handle_t dev);

#ifdef __cplusplus
}
#endif
