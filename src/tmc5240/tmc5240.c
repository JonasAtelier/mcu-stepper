/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "stepper_priv.h"

#if CONFIG_STEPPER_TMC5240

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "stepper_log.h"
#include "tmc5240.h"
#include "tmc5240_math.h"
#include "tmc5240_priv.h"
#include "tmc5240_regs.h"

#define DEFAULT_SPI_HZ  4000000     /* the chip allows 10 MHz; wiring often does not */
#define WAIT_POLL_MS    5
#define SPI_TIMEOUT_MS  10          /* HAL only; five bytes take microseconds */

/*
 * One write, or one whole read chain, at a time. On ESP-IDF a FreeRTOS mutex,
 * as esp-tmc5240-stepper had; on the bare-metal HAL there is no other task to
 * hold off, so one caller per device is the rule (docs/usage.md).
 */
#ifdef ESP_PLATFORM
#define LOCK(d)     xSemaphoreTake((d)->lock, portMAX_DELAY)
#define UNLOCK(d)   xSemaphoreGive((d)->lock)
#else
#define LOCK(d)     ((void)(d))
#define UNLOCK(d)   ((void)(d))
#endif

/* ---------------------------------------------------------------- SPI ---- */

/*
 * Every access is exactly 40 bits: one address byte then four data bytes,
 * MSB first, SPI mode 3. Bit 39 is write_not_read, so 0x80 is OR'd into the
 * address to write. The eight bits shifted back over the address byte are
 * SPI_STATUS (Table 3), which is why every transfer updates it.
 */
static int xfer(tmc5240_handle_t d, uint8_t addr, uint32_t out, uint32_t *in)
{
    uint8_t tx[5], rx[5];
    int err;

    tx[0] = addr;
    tx[1] = (uint8_t)(out >> 24);
    tx[2] = (uint8_t)(out >> 16);
    tx[3] = (uint8_t)(out >> 8);
    tx[4] = (uint8_t)out;

    err = stepper_bus_exchange(&d->bus, tx, rx, sizeof(tx));
    if (err)
        return err;

    d->spi_status = rx[0];
    if (in)
        *in = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) |
              ((uint32_t)rx[3] << 8) | rx[4];
    return 0;
}

static int write_reg(tmc5240_handle_t d, uint8_t reg, uint32_t value)
{
    int err;

    LOCK(d);
    err = xfer(d, reg | TMC5240_WRITE, value, NULL);
    UNLOCK(d);
    return err;
}

/*
 * Reads are pipelined: a datagram returns the data for the address sent in the
 * *previous* datagram, not its own. So n registers take n + 1 datagrams: the
 * first answer is thrown away, and the last address goes out a second time to
 * collect its own data. Getting this wrong reads plausible values that are one
 * access stale.
 *
 * The chain runs under the lock. Another task's datagram landing inside it
 * would be answered with this chain's data, and this chain with its echo.
 */
static int read_chain(tmc5240_handle_t d, const uint8_t *reg,
                      uint32_t *value, int n)
{
    int err;
    int i;

    LOCK(d);
    err = xfer(d, reg[0], 0, NULL);
    for (i = 0; i < n && !err; i++)
        err = xfer(d, reg[i + 1 < n ? i + 1 : i], 0, &value[i]);
    UNLOCK(d);
    return err;
}

static int read_reg(tmc5240_handle_t d, uint8_t reg, uint32_t *value)
{
    return read_chain(d, &reg, value, 1);
}

/* ------------------------------------------------------------- ramp ------ */

static int put_scaled(tmc5240_handle_t d, uint8_t reg, double raw, int bits)
{
    if (!tmc5240_fits(raw, bits))
        return -EINVAL;
    return write_reg(d, reg, tmc5240_fit(raw, bits));
}

static double v_raw(tmc5240_handle_t d, float hz)
{
    return tmc5240_hz_to_v(fabs((double)hz), d->fclk);
}

static double a_raw(tmc5240_handle_t d, float hz_s)
{
    return tmc5240_hzs_to_a(fabs((double)hz_s), d->fclk);
}

static int put_v(tmc5240_handle_t d, uint8_t reg, float hz, int bits)
{
    return put_scaled(d, reg, v_raw(d, hz), bits);
}

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

static int set_ramp(tmc5240_handle_t d, const tmc5240_ramp_t *r)
{
    double vstop;
    size_t i;

    if (!(r->vmax > 0.0f) || !(r->amax > 0.0f) || !(r->dmax > 0.0f))
        return -EINVAL;

    /* VSTOP of 0 hangs the ramp generator in positioning mode -- it never
     * declares the move finished. 10 is the datasheet's own suggestion and is
     * slow enough to be invisible. */
    vstop = v_raw(d, r->vstop);
    if (vstop < 10.0)
        vstop = 10.0;

    /* Every field is converted and range-checked before the first write, so
     * a rejected ramp leaves the chip, vmax_reg and amax_reg all on the old
     * one rather than half of each. */
    const struct { uint8_t reg; int bits; double raw; } f[] = {
        { TMC5240_VSTART, TMC5240_VSTART_BITS, v_raw(d, r->vstart) },
        { TMC5240_VSTOP,  TMC5240_VSTART_BITS, vstop },
        { TMC5240_V1,     TMC5240_V1_BITS,     v_raw(d, r->v1) },
        { TMC5240_V2,     TMC5240_V1_BITS,     v_raw(d, r->v2) },
        { TMC5240_VMAX,   TMC5240_V_BITS,      v_raw(d, r->vmax) },
        { TMC5240_A1,     TMC5240_A_BITS,      a_raw(d, r->a1) },
        { TMC5240_A2,     TMC5240_A_BITS,      a_raw(d, r->a2) },
        { TMC5240_AMAX,   TMC5240_A_BITS,      a_raw(d, r->amax) },
        { TMC5240_D1,     TMC5240_A_BITS,      a_raw(d, r->d1) },
        { TMC5240_D2,     TMC5240_A_BITS,      a_raw(d, r->d2) },
        { TMC5240_DMAX,   TMC5240_A_BITS,      a_raw(d, r->dmax) },
        /* TZEROWAIT counts in 512 clock ticks. */
        { TMC5240_TZEROWAIT, 16, (double)r->tzerowait_ms / 1000.0 * d->fclk / 512.0 },
    };

    for (i = 0; i < sizeof(f) / sizeof(f[0]); i++)
        if (!tmc5240_fits(f[i].raw, f[i].bits))
            return -EINVAL;
    for (i = 0; i < sizeof(f) / sizeof(f[0]); i++) {
        /* In velocity mode VMAX is run()'s speed or stop()'s 0, not the
         * ramp's. move_to() writes vmax_reg when it switches over. */
        if (f[i].reg == TMC5240_VMAX && !d->positioning)
            continue;
        TRY(write_reg(d, f[i].reg, tmc5240_fit(f[i].raw, f[i].bits)));
    }

    d->vmax_reg = tmc5240_fit(tmc5240_hz_to_v(r->vmax, d->fclk), TMC5240_V_BITS);
    d->amax_reg = tmc5240_fit(tmc5240_hzs_to_a(r->amax, d->fclk), TMC5240_A_BITS);
    d->amax_borrowed = false;
    return 0;
}

/* ---------------------------------------------------------- current ------ */

static int apply_current(tmc5240_handle_t d, float run_a, float hold_a)
{
    int range;
    double full;
    uint8_t irun, ihold;

    if (!(run_a > 0.0f))
        return -EINVAL;

    range = tmc5240_pick_range((double)run_a, d->rref);
    if (range < 0) {
        stepper_err(&d->stepper, "%.2f A RMS is out of reach with a %.1f kOhm reference",
                    (double)run_a, d->rref);
        return -EINVAL;
    }
    full = tmc5240_range_rms(range, d->rref);

    if (!(hold_a > 0.0f))
        hold_a = run_a * 0.5f;

    irun  = tmc5240_cs((double)run_a, full);
    ihold = tmc5240_cs((double)hold_a, full);

    TRY(write_reg(d, TMC5240_DRV_CONF, (uint32_t)range));
    /* GLOBAL_SCALER 0 means full scale. The range is already the smallest one
     * that fits, so the extra attenuation is not needed and the datasheet
     * warns against values below 128. */
    TRY(write_reg(d, TMC5240_GLOBAL_SCALER, 0));
    TRY(write_reg(d, TMC5240_IHOLD_IRUN,
                  ((uint32_t)ihold << TMC5240_IHOLD_SHIFT) |
                  ((uint32_t)irun << TMC5240_IRUN_SHIFT) |
                  (6u << TMC5240_IHOLDDELAY_SHIFT) |
                  (4u << TMC5240_IRUNDELAY_SHIFT)));

    d->range_rms = full;
    d->run_rms   = (float)tmc5240_cs_rms(irun, full);

    if (fabs((double)d->run_rms - run_a) > 0.05 * run_a)
        stepper_warn(&d->stepper, "asked %.3f A RMS, current scale gives %.3f A",
                     (double)run_a, (double)d->run_rms);
    return 0;
}

/*
 * Poll RAMPSTAT until a flag sets. A zero timeout still gets one look.
 *
 * Time is measured in ticks, not assumed per poll (stepper_expired()): at
 * ESP-IDF's default 100 Hz tick, pdMS_TO_TICKS(5) is 0 and vTaskDelay(0) only
 * yields, so a poll can take microseconds. Each poll sleeps at least one tick.
 * The first tick is already partly gone when the count starts, so the timeout
 * rounds up to whole ticks and adds one; that guarantees timeout_ms of real
 * time. The count saturates, so UINT32_MAX still means wait forever.
 */
static int wait_flag(tmc5240_handle_t d, uint32_t flag, uint32_t timeout_ms)
{
    uint32_t start = stepper_tick_now();
    uint32_t stat;

    for (;;) {
        TRY(read_reg(d, TMC5240_RAMPSTAT, &stat));
        if (stat & flag)
            return 0;
        if (stepper_expired(start, timeout_ms))
            return -ETIMEDOUT;

        stepper_poll_wait(WAIT_POLL_MS);
    }
}

/* stop_hard() leaves AMAX at its maximum, because the ramp keeps using AMAX
 * until the motor is still. Every call that starts new motion gives it back. */
static int restore_amax(tmc5240_handle_t d)
{
    if (!d->amax_borrowed)
        return 0;
    TRY(write_reg(d, TMC5240_AMAX, d->amax_reg));
    d->amax_borrowed = false;
    return 0;
}

/* ------------------------------------------------------------ motion ----- */

static int move_to(tmc5240_handle_t d, int32_t position)
{
    TRY(restore_amax(d));

    /* Positioning cruises at whatever VMAX holds, and run() and stop() leave
     * their own velocity there. Target and velocity go in before the mode
     * switch, so the ramp never starts on stale values. */
    TRY(write_reg(d, TMC5240_XTARGET, (uint32_t)position));
    TRY(write_reg(d, TMC5240_VMAX, d->vmax_reg));
    TRY(write_reg(d, TMC5240_RAMPMODE, TMC5240_MODE_POSITION));
    d->positioning = true;
    return 0;
}

static int stop(tmc5240_handle_t d)
{
    /* Velocity mode with VMAX 0 decelerates using AMAX, then holds. */
    TRY(restore_amax(d));
    d->positioning = false;
    TRY(write_reg(d, TMC5240_VMAX, 0));
    return write_reg(d, TMC5240_RAMPMODE, TMC5240_MODE_VELPOS);
}

static int stop_hard(tmc5240_handle_t d)
{
    /* Borrow the acceleration register for the whole deceleration, which
     * outlasts this call. restore_amax() gives it back on the next move. */
    TRY(write_reg(d, TMC5240_AMAX, (1u << TMC5240_A_BITS) - 1));
    d->amax_borrowed = true;
    d->positioning = false;
    TRY(write_reg(d, TMC5240_VMAX, 0));
    return write_reg(d, TMC5240_RAMPMODE, TMC5240_MODE_VELPOS);
}

static int read_velocity(tmc5240_handle_t d, float *velocity)
{
    uint32_t raw;

    TRY(read_reg(d, TMC5240_VACTUAL, &raw));
    *velocity = (float)tmc5240_v_to_hz(tmc5240_sext(raw, TMC5240_VACTUAL_BITS),
                                       d->fclk);
    return 0;
}

/* ------------------------------------------------- the stepper_dev ops -- */

static tmc5240_handle_t of(struct stepper_dev *stepper)
{
    return stepper->priv;
}

static int op_enable(struct stepper_dev *stepper, bool on)
{
    tmc5240_handle_t d = of(stepper);

    if (!stepper_pin_wired(&d->en))
        return -EOPNOTSUPP;
    /* DRV_ENN is active low. */
    stepper_pin_set(&d->en, !on);
    return 0;
}

static int op_set_current(struct stepper_dev *stepper, float run_a,
                          float hold_a)
{
    return apply_current(of(stepper), run_a, hold_a);
}

/*
 * DRV_STATUS and GSTAT in one chain. GSTAT's flags are write 1 to clear
 * (p.92), so the ones that latched are written back and was_reset reports
 * each restart once -- as the TMC2209 does.
 */
static int op_read_status(struct stepper_dev *stepper,
                          struct stepper_status *status)
{
    static const uint8_t reg[] = { TMC5240_DRVSTATUS, TMC5240_GSTAT };
    tmc5240_handle_t d = of(stepper);
    uint32_t val[2], drv, gstat;

    TRY(read_chain(d, reg, val, 2));
    drv   = val[0];
    gstat = val[1];

    status->standstill       = !!(drv & TMC5240_STST);
    status->stealthchop      = !!(drv & TMC5240_STEALTH);
    status->overtemp_warning = !!(drv & TMC5240_OTPW);
    status->overtemp         = !!(drv & TMC5240_OT);
    status->short_to_gnd     = !!(drv & (TMC5240_S2GA | TMC5240_S2GB));
    status->short_to_supply  = !!(drv & (TMC5240_S2VSA | TMC5240_S2VSB));
    status->open_load        = !!(drv & (TMC5240_OLA | TMC5240_OLB));
    status->stall            = !!(drv & TMC5240_STALLGUARD);
    status->was_reset        = !!(gstat & TMC5240_RESET_FLAG);
    status->current_scale    = (uint8_t)((drv >> TMC5240_CS_ACTUAL_SHIFT) &
                                         TMC5240_CS_ACTUAL_MASK);

    if (gstat & (TMC5240_RESET_FLAG | TMC5240_DRV_ERR))
        TRY(write_reg(d, TMC5240_GSTAT, gstat));
    return 0;
}

static const struct stepper_ops tmc5240_ops = {
    .enable = op_enable,
    .set_current = op_set_current,
    .read_status = op_read_status,
};

/* ------------------------------------------------------------- init ------ */

static const tmc5240_ramp_t DEFAULT_RAMP = {
    .vmax = 51200.0f,   /* one rev/s on a 200 fullstep motor at 256 microsteps */
    .amax = 51200.0f,   /* reached in a second */
    .dmax = 51200.0f,
    .vstart = 0.0f,
    .vstop  = 0.0f,     /* set_ramp raises this to the minimum the chip needs */
};

/* Who logs before there is a handle to log as. */
static const struct stepper_dev unbound = { .name = "tmc5240" };

/* Free what init made, in reverse. The bus only if it was attached. */
static void release(tmc5240_handle_t d)
{
    if (d->bus.ops)
        stepper_bus_detach(&d->bus);
#ifdef ESP_PLATFORM
    vSemaphoreDelete(d->lock);
#endif
    free(d);
}

static int init(const tmc5240_config_t *cfg, tmc5240_handle_t *out)
{
    tmc5240_handle_t d;
    uint32_t inp_out, chopconf, gconf, gstat, stop_amax;
    bool en_wired;
    float v;
    int mres;
    int err;

    if (!cfg || !out)
        return -EINVAL;

    mres = tmc5240_mres(cfg->microsteps);
    if (mres < 0) {
        stepper_err(&unbound, "microsteps must be a power of two from 1 to 256, not %d",
                    cfg->microsteps);
        return -EINVAL;
    }
    if (!(cfg->rref_kohm >= TMC5240_RREF_MIN_KOHM &&
          cfg->rref_kohm <= TMC5240_RREF_MAX_KOHM)) {
        stepper_err(&unbound, "rref_kohm must be 12 to 60, not %.1f",
                    (double)cfg->rref_kohm);
        return -EINVAL;
    }
#ifndef ESP_PLATFORM
    /* "The SPI interface uses SPI MODE 3" (p.25): clock idle high, data
     * taken on the second edge, in bytes. The handle is the app's, so
     * refuse a wrong one before the bus is touched. */
    if (!cfg->spi || !cfg->cs_port ||
        cfg->spi->Init.CLKPolarity != SPI_POLARITY_HIGH ||
        cfg->spi->Init.CLKPhase != SPI_PHASE_2EDGE ||
        cfg->spi->Init.DataSize != SPI_DATASIZE_8BIT)
        return -EINVAL;
#endif

    d = calloc(1, sizeof(*d));
    if (!d)
        return -ENOMEM;
#ifdef ESP_PLATFORM
    d->lock = xSemaphoreCreateMutex();
    if (!d->lock) {
        free(d);
        return -ENOMEM;
    }
#endif

    d->stepper.name = "tmc5240";
#ifdef ESP_PLATFORM
    d->en.gpio    = cfg->en_gpio;
#else
    d->en.port    = cfg->en_port;
    d->en.pin     = cfg->en_pin;
#endif
    en_wired      = stepper_pin_wired(&d->en);
    d->rref       = cfg->rref_kohm;
    d->microsteps = cfg->microsteps;
    d->fclk       = cfg->clock_hz > 0 ? (double)cfg->clock_hz : TMC5240_FCLK_INTERNAL;

    /* Hold the driver stage off until it is configured, so a stale IHOLD_IRUN
     * from a previous run cannot put current through the motor. */
    if (en_wired) {
        err = stepper_pin_output(&d->en, true);
        if (err)
            goto fail;
    }

#ifdef ESP_PLATFORM
    /* "The SPI interface uses SPI MODE 3." */
    err = stepper_spi_attach(&d->bus, cfg->host, cfg->cs_gpio,
                             cfg->spi_hz > 0 ? (uint32_t)cfg->spi_hz : DEFAULT_SPI_HZ,
                             3);
#else
    err = stepper_spi_attach(&d->bus, cfg->spi, cfg->cs_port, cfg->cs_pin,
                             SPI_TIMEOUT_MS);
#endif
    if (err)
        goto fail;

    err = read_reg(d, TMC5240_INP_OUT, &inp_out);
    if (err)
        goto fail;

    if (((inp_out >> TMC5240_VERSION_SHIFT) & TMC5240_VERSION_MASK) !=
        TMC5240_VERSION_FIRST) {
        stepper_err(&d->stepper, "no TMC5240 answering: INP_OUT=0x%08lx",
                    (unsigned long)inp_out);
        err = -ENODEV;
        goto fail;
    }

    /* The chip knows whether it is running on an external clock. It does not
     * know that clock's frequency, and neither do we unless told -- so say so
     * rather than silently scaling every velocity by the wrong number. */
    if ((inp_out & TMC5240_EXT_CLK) && cfg->clock_hz <= 0)
        stepper_warn(&d->stepper, "external clock in use but clock_hz is 0; "
                                  "velocities assume %.1f MHz and will be wrong",
                     TMC5240_FCLK_INTERNAL / 1e6);
    if (!(inp_out & TMC5240_EXT_CLK) && cfg->clock_hz > 0)
        stepper_warn(&d->stepper, "clock_hz given but the chip is on its internal oscillator");

    /* Read GSTAT, as esp-tmc5240-stepper did. Its flags clear only when
     * written back with 1 (p.92); stepper_read_status() does that. */
    err = read_reg(d, TMC5240_GSTAT, &gstat);
    if (err)
        goto fail;

    /* An MCU reset does not reset the TMC5240, so it may still be turning,
     * and HOLD would keep that velocity. Ramp it to standstill before any
     * current, chopper or StealthChop setting changes under it. With an
     * enable pin the stage is off and the motor only coasts, so the ramp can
     * stop at maximum AMAX; tied low, the stage is live and an abrupt stop
     * risks overcurrent, so it uses the default ramp. The wait allows twice
     * what that AMAX needs, so a timeout means the ramp is not stopping. */
    err = set_ramp(d, &DEFAULT_RAMP);
    stop_amax = en_wired ? (1u << TMC5240_A_BITS) - 1 : d->amax_reg;
    if (!err)
        err = en_wired ? stop_hard(d) : stop(d);
    if (!err)
        err = read_velocity(d, &v);
    if (!err) {
        err = wait_flag(d, TMC5240_VZERO,
                        (uint32_t)(2000.0 * fabs((double)v) /
                                   tmc5240_a_to_hzs((int32_t)stop_amax, d->fclk)) +
                        WAIT_POLL_MS);
        if (err == -ETIMEDOUT)
            stepper_err(&d->stepper, "motor still turning at %.0f microsteps/s%s",
                        (double)v, en_wired ? "; driver stage left off" : "");
    }
    if (!err)
        err = restore_amax(d);
    if (err)
        goto fail;

    err = apply_current(d, cfg->run_current_a, cfg->hold_current_a);
    if (err)
        goto fail;

    /* SpreadCycle defaults that work on most motors: TOFF 5, TBL 2, and the
     * hysteresis left mid-scale. Tune HSTRT/HEND on a bench if the motor
     * whines at low speed. */
    chopconf = (5u << TMC5240_TOFF_SHIFT) |
               (4u << TMC5240_HSTRT_SHIFT) |
               (0u << TMC5240_HEND_SHIFT) |
               (2u << TMC5240_TBL_SHIFT) |
               ((uint32_t)mres << TMC5240_MRES_SHIFT) |
               TMC5240_INTPOL;
    err = write_reg(d, TMC5240_CHOPCONF, chopconf);
    if (err)
        goto fail;

    gconf = 0;
    if (cfg->stealthchop)
        gconf |= TMC5240_EN_PWM_MODE;
    if (cfg->invert_direction)
        gconf |= TMC5240_SHAFT;
    err = write_reg(d, TMC5240_GCONF, gconf);
    if (err)
        goto fail;

    err = write_reg(d, TMC5240_TPOWERDOWN, 10);
    if (!err)
        err = write_reg(d, TMC5240_TPWMTHRS, 0);
    if (!err)
        err = write_reg(d, TMC5240_SWMODE, 0);   /* no reference switches */
    if (!err)
        err = write_reg(d, TMC5240_RAMPMODE, TMC5240_MODE_HOLD);
    if (!err)
        err = write_reg(d, TMC5240_XACTUAL, 0);
    if (!err)
        err = write_reg(d, TMC5240_XTARGET, 0);
    if (err)
        goto fail;

    if (en_wired)
        stepper_pin_set(&d->en, false);

    stepper_info(&d->stepper, "TMC5240 rev %lu, %d microsteps, %.3f A RMS run current",
                 (unsigned long)((inp_out >> 16) & 0x7), cfg->microsteps,
                 (double)d->run_rms);

    d->stepper.ops = &tmc5240_ops;
    d->stepper.priv = d;
    d->stepper.microsteps = (uint16_t)cfg->microsteps;
    *out = d;
    return 0;

fail:
    release(d);
    return err;
}

/* ------------------------------------------------------ the handle API -- */

stepper_handle_ret tmc5240_init(const tmc5240_config_t *cfg, tmc5240_handle_t *out)
{
    return STEPPER_HANDLE_RET(init(cfg, out));
}

stepper_handle_ret tmc5240_del(tmc5240_handle_t d)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);

    if (stepper_pin_wired(&d->en))
        stepper_pin_set(&d->en, true);
    release(d);
    return STEPPER_HANDLE_RET(0);
}

stepper_handle_ret tmc5240_write(tmc5240_handle_t d, uint8_t reg, uint32_t value)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(write_reg(d, reg, value));
}

stepper_handle_ret tmc5240_read(tmc5240_handle_t d, uint8_t reg, uint32_t *value)
{
    if (!d || !value)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(read_reg(d, reg, value));
}

uint8_t tmc5240_spi_status(tmc5240_handle_t d)
{
    return d ? d->spi_status : 0;
}

stepper_handle_ret tmc5240_set_ramp(tmc5240_handle_t d, const tmc5240_ramp_t *r)
{
    if (!d || !r)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(set_ramp(d, r));
}

stepper_handle_ret tmc5240_set_current(tmc5240_handle_t d, float run_a, float hold_a)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(apply_current(d, run_a, hold_a));
}

float tmc5240_run_current(tmc5240_handle_t d)
{
    return d ? d->run_rms : 0.0f;
}

stepper_handle_ret tmc5240_move_to(tmc5240_handle_t d, int32_t position)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(move_to(d, position));
}

stepper_handle_ret tmc5240_move_by(tmc5240_handle_t d, int32_t delta)
{
    uint32_t base;
    int err;

    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);

    /* Outside positioning, XTARGET is wherever the last move left it, not
     * where the motor is. The sum is unsigned so it wraps the way the
     * 32-bit position registers do, instead of overflowing an int32_t. */
    err = read_reg(d, d->positioning ? TMC5240_XTARGET : TMC5240_XACTUAL, &base);
    if (!err)
        err = move_to(d, (int32_t)(base + (uint32_t)delta));
    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc5240_run(tmc5240_handle_t d, float velocity)
{
    int err;

    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);

    err = restore_amax(d);
    if (!err) {
        d->positioning = false;
        err = put_v(d, TMC5240_VMAX, velocity, TMC5240_V_BITS);
    }
    if (!err)
        err = write_reg(d, TMC5240_RAMPMODE,
                        velocity < 0 ? TMC5240_MODE_VELNEG : TMC5240_MODE_VELPOS);
    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc5240_stop(tmc5240_handle_t d)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(stop(d));
}

stepper_handle_ret tmc5240_stop_hard(tmc5240_handle_t d)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(stop_hard(d));
}

stepper_handle_ret tmc5240_wait(tmc5240_handle_t d, uint32_t timeout_ms)
{
    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(wait_flag(d, TMC5240_POSITION_REACHED, timeout_ms));
}

/* ------------------------------------------------------------ reading ---- */

stepper_handle_ret tmc5240_position(tmc5240_handle_t d, int32_t *position)
{
    uint32_t raw;
    int err;

    if (!d || !position)
        return STEPPER_HANDLE_RET(-EINVAL);
    err = read_reg(d, TMC5240_XACTUAL, &raw);
    if (!err)
        *position = (int32_t)raw;
    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc5240_set_position(tmc5240_handle_t d, int32_t position)
{
    uint32_t stat;
    int err;

    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);

    /* XACTUAL is the ramp generator's own counter. Rewriting it mid-move
     * makes the generator chase a target it has already passed. */
    err = read_reg(d, TMC5240_RAMPSTAT, &stat);
    if (!err && !(stat & TMC5240_VZERO))
        err = -EBUSY;
    if (!err)
        err = write_reg(d, TMC5240_RAMPMODE, TMC5240_MODE_HOLD);
    if (!err)
        err = write_reg(d, TMC5240_XACTUAL, (uint32_t)position);
    if (!err)
        err = write_reg(d, TMC5240_XTARGET, (uint32_t)position);
    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc5240_velocity(tmc5240_handle_t d, float *velocity)
{
    if (!d || !velocity)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(read_velocity(d, velocity));
}

stepper_handle_ret tmc5240_status(tmc5240_handle_t d, tmc5240_status_t *out)
{
    static const uint8_t reg[] = {
        TMC5240_DRVSTATUS, TMC5240_RAMPSTAT, TMC5240_XACTUAL, TMC5240_VACTUAL,
    };
    uint32_t val[4], drv, ramp, x, v;
    int err;

    if (!d || !out)
        return STEPPER_HANDLE_RET(-EINVAL);

    /* One chain of five datagrams rather than four reads of two. */
    err = read_chain(d, reg, val, 4);
    if (err)
        return STEPPER_HANDLE_RET(err);
    drv  = val[0];
    ramp = val[1];
    x    = val[2];
    v    = val[3];

    memset(out, 0, sizeof(*out));
    out->position = (int32_t)x;
    out->velocity = (float)tmc5240_v_to_hz(tmc5240_sext(v, TMC5240_VACTUAL_BITS),
                                           d->fclk);

    out->position_reached = !!(ramp & TMC5240_POSITION_REACHED);
    out->velocity_reached = !!(ramp & TMC5240_VELOCITY_REACHED);
    out->stopped_left     = !!(ramp & TMC5240_STATUS_STOP_L);
    out->stopped_right    = !!(ramp & TMC5240_STATUS_STOP_R);

    out->standstill    = !!(drv & TMC5240_STST);
    out->stallguard    = !!(drv & TMC5240_STALLGUARD);
    out->sg_result     = (uint16_t)(drv & TMC5240_SG_RESULT_MASK);
    out->cs_actual     = (uint8_t)((drv >> TMC5240_CS_ACTUAL_SHIFT) &
                                   TMC5240_CS_ACTUAL_MASK);
    out->overtemp_warn = !!(drv & TMC5240_OTPW);
    out->overtemp      = !!(drv & TMC5240_OT);
    out->open_load_a   = !!(drv & TMC5240_OLA);
    out->open_load_b   = !!(drv & TMC5240_OLB);
    out->short_gnd_a   = !!(drv & TMC5240_S2GA);
    out->short_gnd_b   = !!(drv & TMC5240_S2GB);
    out->short_vs_a    = !!(drv & TMC5240_S2VSA);
    out->short_vs_b    = !!(drv & TMC5240_S2VSB);
    out->stealth       = !!(drv & TMC5240_STEALTH);
    return STEPPER_HANDLE_RET(0);
}

/* -------------------------------------------------------- diagnostics ---- */

/* The chip says its ADC is not to be trusted: -ENODATA, no reading. */
static int adc_ok(tmc5240_handle_t d)
{
    uint32_t inp_out;

    TRY(read_reg(d, TMC5240_INP_OUT, &inp_out));
    return (inp_out & TMC5240_ADC_ERR) ? -ENODATA : 0;
}

stepper_handle_ret tmc5240_supply_voltage(tmc5240_handle_t d, float *volts)
{
    uint32_t raw;
    int err;

    if (!d || !volts)
        return STEPPER_HANDLE_RET(-EINVAL);
    err = adc_ok(d);
    if (!err)
        err = read_reg(d, TMC5240_ADC_VSUPPLY_AIN, &raw);
    if (!err)
        *volts = tmc5240_vsupply_v(raw);
    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc5240_temperature(tmc5240_handle_t d, float *celsius)
{
    uint32_t raw;
    int err;

    if (!d || !celsius)
        return STEPPER_HANDLE_RET(-EINVAL);
    err = adc_ok(d);
    if (!err)
        err = read_reg(d, TMC5240_ADC_TEMP, &raw);
    if (!err)
        *celsius = tmc5240_temp_c(raw);
    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc5240_stallguard(tmc5240_handle_t d, uint8_t threshold,
                                      float min_velocity)
{
    double tcool;
    int err;

    if (!d)
        return STEPPER_HANDLE_RET(-EINVAL);

    err = write_reg(d, TMC5240_SG4_THRS, threshold);
    if (err)
        return STEPPER_HANDLE_RET(err);

    if (threshold == 0 || !(min_velocity > 0.0f))
        return STEPPER_HANDLE_RET(write_reg(d, TMC5240_TCOOLTHRS, 0));

    /* StallGuard runs while TSTEP <= TCOOLTHRS, and TSTEP falls as velocity
     * rises -- so the threshold register holds the step period of the slowest
     * velocity at which detection should be trusted. */
    tcool = tmc5240_v_to_tstep(tmc5240_hz_to_v((double)min_velocity, d->fclk),
                               d->microsteps);
    return STEPPER_HANDLE_RET(put_scaled(d, TMC5240_TCOOLTHRS, tcool, 20));
}

struct stepper_dev *tmc5240_stepper(tmc5240_handle_t d)
{
    return d ? &d->stepper : NULL;
}

#endif /* CONFIG_STEPPER_TMC5240 */
