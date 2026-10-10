/*
 * tmc5240 -- the ADI/Trinamic TMC5240 stepper driver, as a stepper chip
 * driver, for ESP-IDF and the STM32 HAL.
 *
 * The TMC5240 is not a step/direction driver with a serial port bolted on: it
 * contains the ramp generator. You give it a target position and it does the
 * acceleration itself, in hardware, while your CPU does something else. This
 * driver is shaped around that -- there is no step pulse anywhere in it.
 *
 * SPI only. The chip also speaks single-wire UART; that is not implemented.
 *
 * Every register address and constant is quoted from the datasheet in
 * docs/tmc5240.pdf. Velocity and acceleration are microsteps per second and
 * per second squared, at whatever microstep resolution you configured -- see
 * the unit warning in docs/usage.md.
 *
 * tmc5240_stepper() hands out the struct stepper_dev inside the handle, for
 * stepper_enable(), stepper_set_current() and stepper_read_status().
 *
 * On ESP-IDF every call returns an esp_err_t, as esp-tmc5240-stepper did,
 * and a FreeRTOS mutex keeps tasks from splitting a read chain. On the STM32
 * HAL a call returns 0 or a negative errno, and there is no lock: one caller
 * per device.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#else
#include CMSIS_device_header
#endif

#include "stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc5240_t *tmc5240_handle_t;

typedef struct {
#ifdef ESP_PLATFORM
    spi_host_device_t host;         /* an SPI bus you have already initialised */
    gpio_num_t        cs_gpio;
    gpio_num_t        en_gpio;      /* DRV_ENN, active low; -1 if tied to GND */
    int               spi_hz;       /* 0 selects 4 MHz. The chip tops out at 10 MHz. */
#else
    /* An SPI handle the app has HAL_SPI_Init()ed in mode 3 (CPOL high, CPHA
     * second edge) with 8-bit frames, at 10 MHz or less; the clock is the
     * handle's. Chip select is software: an output the app has set up, idle
     * high. */
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef      *cs_port;
    uint16_t          cs_pin;
    /* DRV_ENN, active low: an output the app has set up. en_port NULL if
     * tied to GND. */
    GPIO_TypeDef      *en_port;
    uint16_t          en_pin;
#endif

    /* Clock the ramp generator counts in. 0 means the internal oscillator,
     * which this driver then assumes is its 12.5 MHz nominal. That oscillator
     * is specified 11.9 to 13.2 MHz, so every velocity you ask for can be off
     * by 5% with nothing reporting an error. Feed CLK from the MCU and put the
     * real frequency here when speed has to be right. */
    int               clock_hz;

    /* Reference resistor from IREF to GND, 12 to 60 kOhm. It sets full-scale
     * current, and there is no way to measure it -- if this does not match the
     * resistor actually fitted, every current below is wrong by the ratio. */
    float             rref_kohm;

    float             run_current_a;   /* RMS. Quantised to 1/32 of range. */
    float             hold_current_a;  /* RMS. 0 selects half of run. */

    int               microsteps;      /* 256, 128, 64, 32, 16, 8, 4, 2 or 1 */
    bool              stealthchop;     /* quiet below TPWMTHRS, less torque */
    bool              invert_direction;
} tmc5240_config_t;

/*
 * A ramp, in microsteps per second and per second squared.
 *
 * The TMC5240's EightPoint generator can use three acceleration and three
 * deceleration segments so the ramp follows the motor's torque curve. Leave
 * v1 and v2 at 0 and you get a plain trapezoid using amax/dmax only, which is
 * what most applications want.
 *
 *   v >= v2 > v1:   amax/dmax apply above v2, a2/d2 between v1 and v2,
 *                   a1/d1 below v1.
 */
typedef struct {
    float vmax;         /* cruise velocity */
    float amax, dmax;   /* acceleration and deceleration above v2 */
    float vstart;       /* velocity stepped to instantly at start; 0 is fine */
    float vstop;        /* velocity the ramp stops from; must be > vstart */
    float v1, a1, d1;   /* 0 disables this segment */
    float v2, a2, d2;   /* 0 disables this segment */
    uint16_t tzerowait_ms; /* dwell at zero before reversing */
} tmc5240_ramp_t;

typedef struct {
    int32_t  position;        /* XACTUAL, microsteps */
    float    velocity;        /* VACTUAL, microsteps/s, signed */

    bool     position_reached;
    bool     velocity_reached;
    bool     standstill;
    bool     stopped_left, stopped_right;   /* reference switch active */

    bool     stallguard;      /* StallGuard threshold crossed */
    uint16_t sg_result;       /* 0 = maximum load; falls as load rises */
    uint8_t  cs_actual;       /* current scale in use, after CoolStep */

    bool     overtemp_warn;   /* otpw -- back off now */
    bool     overtemp;        /* ot -- driver has shut down */
    bool     open_load_a, open_load_b;
    bool     short_gnd_a, short_gnd_b;
    bool     short_vs_a, short_vs_b;
    bool     stealth;         /* StealthChop2 currently active */
} tmc5240_status_t;

#ifdef ESP_PLATFORM
/*
 * Probe the chip, apply the configuration and leave the motor stopped at
 * position 0 with the driver stage enabled.
 *
 * An ESP reboot does not reset the chip, so the motor may still be turning.
 * Init stops the ramp before it configures anything or enables the stage: at
 * once while en_gpio holds the stage off. With DRV_ENN tied low the stage is
 * live, so it decelerates with the default AMAX of 51200 microsteps/s^2 and
 * can block for up to about 2 * v / 51200 seconds. ESP_ERR_TIMEOUT if the
 * ramp does not stop, with the stage left off where en_gpio allows.
 *
 * ESP_ERR_NOT_FOUND if INP_OUT.VERSION does not read 0x40 -- usually miswired
 * SPI, the wrong CS, or a bus running faster than 10 MHz.
 * ESP_ERR_INVALID_ARG for a current the chosen rref cannot reach, an rref
 * outside 12..60 kOhm, or a microstep count that is not a power of two.
 */
esp_err_t tmc5240_init(const tmc5240_config_t *cfg, tmc5240_handle_t *out);
esp_err_t tmc5240_del(tmc5240_handle_t dev);

/* Replace the ramp. Safe to call while moving; the generator picks it up.
 * After run() or stop(), the new vmax waits for the next move_to, so the
 * velocity run() set, or stop()'s 0, is left alone. */
esp_err_t tmc5240_set_ramp(tmc5240_handle_t dev, const tmc5240_ramp_t *ramp);

/* Move to an absolute position and return immediately. The chip runs the ramp,
 * at the ramp's vmax whatever run() or stop() last set.
 * Poll tmc5240_status() or block in tmc5240_wait(). */
esp_err_t tmc5240_move_to(tmc5240_handle_t dev, int32_t position);

/* Relative to the current *target*, not the current position, so calling it
 * twice in quick succession queues both moves rather than losing the first.
 * After run(), stop() or stop_hard() there is no target, so it is relative to
 * the current position. */
esp_err_t tmc5240_move_by(tmc5240_handle_t dev, int32_t delta);

/* Turn at a constant velocity, sign selecting direction, until told otherwise.
 * There is no target position in this mode. */
esp_err_t tmc5240_run(tmc5240_handle_t dev, float velocity);

/* Decelerate to standstill using the ramp, and hold. */
esp_err_t tmc5240_stop(tmc5240_handle_t dev);

/* Cut the ramp and stop as fast as the mechanics allow. Loses position sync
 * with the load if the motor slips -- that is the trade you are making.
 * AMAX stays at its maximum until the next move_to, move_by, run, stop or
 * set_ramp puts the ramp's value back. In StealthChop at speed, the datasheet
 * warns an abrupt stop raises coil current for several tens of ms, and can
 * trip short detection; its recovery is to clear the error flags and restart
 * from zero velocity. */
esp_err_t tmc5240_stop_hard(tmc5240_handle_t dev);

/* Block until the target position is reached. ESP_ERR_TIMEOUT if it is not,
 * which is also what a stalled or unpowered motor looks like. */
esp_err_t tmc5240_wait(tmc5240_handle_t dev, uint32_t timeout_ms);

esp_err_t tmc5240_position(tmc5240_handle_t dev, int32_t *position);
esp_err_t tmc5240_velocity(tmc5240_handle_t dev, float *velocity);

/* Redefine the current position without moving. Refused while the motor is
 * turning, where it would corrupt the ramp in progress. */
esp_err_t tmc5240_set_position(tmc5240_handle_t dev, int32_t position);

esp_err_t tmc5240_status(tmc5240_handle_t dev, tmc5240_status_t *out);

/* Change current at runtime. hold_a of 0 selects half of run_a. */
esp_err_t tmc5240_set_current(tmc5240_handle_t dev, float run_a, float hold_a);

/* What the current actually came out as. CS quantises to 1/32 of the selected
 * range, so this is not what you asked for, and at low currents the gap is
 * worth knowing. */
float tmc5240_run_current(tmc5240_handle_t dev);

/* On-chip diagnostics. Both read ESP_ERR_INVALID_STATE if INP_OUT.ADC_ERR is
 * set, because the chip is telling you its ADC is not to be trusted. */
esp_err_t tmc5240_supply_voltage(tmc5240_handle_t dev, float *volts);
esp_err_t tmc5240_temperature(tmc5240_handle_t dev, float *celsius);

/*
 * StallGuard4: detect a stall from the motor's own back-EMF, no switch needed.
 *
 * threshold is 0..255; higher trips earlier. min_velocity is the speed below
 * which detection is disabled -- StallGuard means nothing near standstill, so
 * this is not optional. Pass 0 to threshold to turn detection off.
 *
 * Tuning is empirical and load-dependent. Watch status.sg_result while the
 * motor runs loaded; it falls towards 0 as the motor approaches a stall.
 */
esp_err_t tmc5240_stallguard(tmc5240_handle_t dev, uint8_t threshold,
                             float min_velocity);

/* Escape hatch for the registers this driver does not model -- MSLUT, the
 * encoder block, DcStep, the virtual stop switches. Writing RAMPMODE, VMAX,
 * AMAX or XTARGET here goes behind the motion calls' back: they remember
 * whether a move is in progress and whether stop_hard() still holds AMAX. */
esp_err_t tmc5240_read(tmc5240_handle_t dev, uint8_t reg, uint32_t *value);
esp_err_t tmc5240_write(tmc5240_handle_t dev, uint8_t reg, uint32_t value);

/* The eight status bits the chip returns with every datagram, from the most
 * recent transfer. See TMC5240_SPI_* in src/tmc5240_regs.h. */
uint8_t tmc5240_spi_status(tmc5240_handle_t dev);
#else
/*
 * Probe the chip, apply the configuration and leave the motor stopped at
 * position 0 with the driver stage enabled, as on ESP-IDF. -EINVAL for a NULL
 * handle or chip select port, or a handle not in mode 3 with 8-bit frames,
 * before the bus is touched; for a current the chosen rref cannot reach, an
 * rref outside 12..60 kOhm, or a microstep count that is not a power of two.
 * -ENODEV if INP_OUT.VERSION does not read 0x40. -ETIMEDOUT if a motor still
 * turning from before the reset does not stop.
 */
int tmc5240_init(const tmc5240_config_t *cfg, tmc5240_handle_t *out);
int tmc5240_del(tmc5240_handle_t dev);

/* The motion, reading and diagnostic calls, each as on ESP-IDF above. */
int tmc5240_set_ramp(tmc5240_handle_t dev, const tmc5240_ramp_t *ramp);
int tmc5240_move_to(tmc5240_handle_t dev, int32_t position);
int tmc5240_move_by(tmc5240_handle_t dev, int32_t delta);
int tmc5240_run(tmc5240_handle_t dev, float velocity);
int tmc5240_stop(tmc5240_handle_t dev);
int tmc5240_stop_hard(tmc5240_handle_t dev);

/* -ETIMEDOUT if the target is not reached in time. */
int tmc5240_wait(tmc5240_handle_t dev, uint32_t timeout_ms);

int tmc5240_position(tmc5240_handle_t dev, int32_t *position);
int tmc5240_velocity(tmc5240_handle_t dev, float *velocity);

/* -EBUSY while the motor is turning. */
int tmc5240_set_position(tmc5240_handle_t dev, int32_t position);

int tmc5240_status(tmc5240_handle_t dev, tmc5240_status_t *out);
int tmc5240_set_current(tmc5240_handle_t dev, float run_a, float hold_a);
float tmc5240_run_current(tmc5240_handle_t dev);

/* -ENODATA if INP_OUT.ADC_ERR is set. */
int tmc5240_supply_voltage(tmc5240_handle_t dev, float *volts);
int tmc5240_temperature(tmc5240_handle_t dev, float *celsius);

int tmc5240_stallguard(tmc5240_handle_t dev, uint8_t threshold,
                       float min_velocity);
int tmc5240_read(tmc5240_handle_t dev, uint8_t reg, uint32_t *value);
int tmc5240_write(tmc5240_handle_t dev, uint8_t reg, uint32_t value);
uint8_t tmc5240_spi_status(tmc5240_handle_t dev);
#endif

/* The handle as a struct stepper_dev; NULL for a NULL handle. Valid until
 * tmc5240_del(). */
struct stepper_dev *tmc5240_stepper(tmc5240_handle_t dev);

#ifdef __cplusplus
}
#endif
