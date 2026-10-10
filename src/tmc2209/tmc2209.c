/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "stepper_priv.h"

#if CONFIG_STEPPER_TMC2209

#include <math.h>
#include <stdlib.h>

#include "stepper_log.h"
#include "tmc2209.h"
#include "tmc2209_math.h"
#include "tmc2209_regs.h"

#define DEFAULT_CLOCK_HZ        12000000    /* internal oscillator, nominal */
#define DEFAULT_RUN_MA          500
#define DEFAULT_MICROSTEPS      16
#define REPLY_TIMEOUT_MS        20

struct tmc2209_t {
    struct stepper_dev stepper; /* the category view; priv points back here */
    struct stepper_bus bus;
    struct stepper_pin en;      /* ENN, active low */
    uint8_t     address;
    bool        two_wire;
    float       rsense_mohm;
    uint32_t    clock_hz;
    uint8_t     mres;       /* one step pulse advances 2^mres 1/256 microsteps */
};

/*
 * On the usual single wire the MCU hears everything it sends, so a datagram
 * goes out through exchange(), which takes that echo back off the line and
 * drops it, so the reply that follows starts clean. The echo is not compared
 * with what was sent. With two_wire there is no echo: transfer() sends.
 */
static int send(struct tmc2209_t *d, const uint8_t *dg, size_t len)
{
    uint8_t echo[TMC2209_WRITE_LEN];

    if (d->two_wire)
        return d->bus.ops->transfer(&d->bus, dg[0], dg + 1, len - 1,
                                    NULL, 0);
    return stepper_bus_exchange(&d->bus, dg, echo, len);
}

static int reg_write(struct tmc2209_t *d, uint8_t reg, uint32_t value)
{
    uint8_t dg[TMC2209_WRITE_LEN];

    dg[0] = TMC2209_SYNC;
    dg[1] = d->address;
    dg[2] = (uint8_t)(reg | TMC2209_WRITE);
    dg[3] = (uint8_t)(value >> 24);     /* data bytes go high to low */
    dg[4] = (uint8_t)(value >> 16);
    dg[5] = (uint8_t)(value >> 8);
    dg[6] = (uint8_t)value;
    dg[7] = tmc2209_crc8(dg, 7);

    return send(d, dg, sizeof dg);
}

static int reg_read(struct tmc2209_t *d, uint8_t reg, uint32_t *value)
{
    uint8_t req[TMC2209_READ_REQ_LEN];
    uint8_t reply[TMC2209_REPLY_LEN];
    int err;

    req[0] = TMC2209_SYNC;
    req[1] = d->address;
    req[2] = reg;                       /* bit 7 clear means read */
    req[3] = tmc2209_crc8(req, 3);

    if (d->two_wire) {
        err = d->bus.ops->transfer(&d->bus, req[0], req + 1, sizeof req - 1,
                                   reply, sizeof reply);
    } else {
        err = send(d, req, sizeof req);
        if (!err)
            err = stepper_bus_receive(&d->bus, reply, sizeof reply);
    }
    if (err)
        return err;

    /* A reply is addressed to the master, carries the register we asked for,
     * and checks out. Anything else means noise or the wrong chip answered. */
    if (reply[0] != TMC2209_SYNC || reply[1] != TMC2209_MASTER_ADDR ||
        reply[2] != reg || reply[7] != tmc2209_crc8(reply, 7))
        return -EBADMSG;

    *value = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
             ((uint32_t)reply[5] << 8) | reply[6];

    return 0;
}

/* Read, change some bits, write back — for the registers where the reset
 * defaults hold settings we have no business clearing. */
static int update(struct tmc2209_t *d, uint8_t reg, uint32_t clear,
                  uint32_t set)
{
    uint32_t v;
    int err;

    err = reg_read(d, reg, &v);
    if (err)
        return err;

    return reg_write(d, reg, (v & ~clear) | set);
}

static int set_current(struct tmc2209_t *d, uint16_t run_ma, uint16_t hold_ma)
{
    uint8_t irun;
    uint8_t ihold;
    bool vsense;
    bool hold_vsense;
    int err;

    if (run_ma == 0)
        return -EINVAL;

    if (hold_ma == 0)
        hold_ma = (uint16_t)(run_ma / 2);

    irun = tmc2209_current_to_cs(d->rsense_mohm, run_ma, &vsense);

    /* Both currents share one vsense bit, so the hold current has to be
     * scaled on whichever range the run current chose. */
    ihold = tmc2209_current_to_cs(d->rsense_mohm, hold_ma, &hold_vsense);
    ihold = tmc2209_cs_on_range(ihold, hold_vsense, vsense);

    stepper_info(&d->stepper,
                 "run %u mA (IRUN=%u), hold %u mA (IHOLD=%u), vsense=%d",
                 tmc2209_cs_to_current(d->rsense_mohm, irun, vsense), irun,
                 tmc2209_cs_to_current(d->rsense_mohm, ihold, vsense), ihold,
                 vsense);

    err = update(d, TMC2209_CHOPCONF, TMC2209_CHOP_VSENSE,
                 vsense ? TMC2209_CHOP_VSENSE : 0);
    if (err)
        return err;

    /* IHOLDDELAY 10: fade the current down over a moment rather than dropping
     * it in one step, which the motor feels as a jerk. */
    return reg_write(d, TMC2209_IHOLD_IRUN,
                     ((uint32_t)10 << TMC2209_IHOLDDELAY_SHIFT) |
                     ((uint32_t)irun << TMC2209_IRUN_SHIFT) |
                     ((uint32_t)ihold << TMC2209_IHOLD_SHIFT));
}

static int move(struct tmc2209_t *d, float usteps_per_s)
{
    int32_t vactual;

    if (!tmc2209_vactual(usteps_per_s, d->clock_hz, &vactual))
        return -EINVAL;

    /* VACTUAL is 24 bits, two's complement. */
    return reg_write(d, TMC2209_VACTUAL, (uint32_t)vactual & 0x00FFFFFFu);
}

static int enable(struct tmc2209_t *d, bool on)
{
    if (!stepper_pin_wired(&d->en))
        return -EOPNOTSUPP;

    /* ENN is active low. */
    stepper_pin_set(&d->en, !on);

    return 0;
}

static int read_status(struct tmc2209_t *d, tmc2209_status_t *out)
{
    uint32_t drv;
    uint32_t gstat;
    int err;

    err = reg_read(d, TMC2209_DRV_STATUS, &drv);
    if (err)
        return err;

    err = reg_read(d, TMC2209_GSTAT, &gstat);
    if (err)
        return err;

    out->drv_status = drv;
    out->standstill = (drv & TMC2209_DRV_STST) != 0;
    out->stealthchop = (drv & TMC2209_DRV_STEALTH) != 0;
    out->overtemp_warning = (drv & TMC2209_DRV_OTPW) != 0;
    out->overtemp = (drv & TMC2209_DRV_OT) != 0;
    out->short_to_gnd = (drv & (TMC2209_DRV_S2GA | TMC2209_DRV_S2GB)) != 0;
    out->short_low_side = (drv & (TMC2209_DRV_S2VSA | TMC2209_DRV_S2VSB)) != 0;
    out->open_load = (drv & (TMC2209_DRV_OLA | TMC2209_DRV_OLB)) != 0;
    out->current_scale = (uint8_t)((drv & TMC2209_DRV_CS_ACTUAL_MASK) >>
                                   TMC2209_DRV_CS_ACTUAL_SHIFT);
    out->undervoltage_cp = (gstat & TMC2209_GSTAT_UV_CP) != 0;
    out->was_reset = (gstat & TMC2209_GSTAT_RESET) != 0;

    /* GSTAT's latched bits clear by writing 1 back, so each reset is reported
     * exactly once. uv_cp is not latched and needs no clearing. */
    if (gstat & (TMC2209_GSTAT_RESET | TMC2209_GSTAT_DRV_ERR))
        reg_write(d, TMC2209_GSTAT, gstat);

    return 0;
}

/* The chip has no ID register. IFCNT counts successful writes, so writing
 * something harmless and watching the counter move proves both directions of
 * the link and that the CRC is agreed. Register 0x02, 5.1 p.24. */
static int verify_link(struct tmc2209_t *d)
{
    uint32_t before;
    uint32_t after;
    int err;

    err = reg_read(d, TMC2209_IFCNT, &before);
    if (err)
        return err;

    /* SENDDELAY 2 - three times eight bit times before the reply. The
     * datasheet asks for at least this on a shared bus (4.1.2 p.19), and it
     * costs nothing on a private one. */
    err = reg_write(d, TMC2209_SLAVECONF,
                    (uint32_t)2 << TMC2209_SLAVECONF_SENDDELAY_SHIFT);
    if (err)
        return err;

    err = reg_read(d, TMC2209_IFCNT, &after);
    if (err)
        return err;

    /* The counter wraps 255 to 0, so compare the difference, not the values. */
    if ((uint8_t)(after - before) != 1) {
        stepper_err(&d->stepper, "IFCNT did not advance (%u then %u): writes "
                                 "are not landing",
                    (unsigned)before, (unsigned)after);
        return -ENODEV;
    }

    return 0;
}

static int apply_config(struct tmc2209_t *d, const tmc2209_config_t *cfg)
{
    uint32_t gconf_set;
    uint32_t gconf_clear;
    int err;

    /* pdn_disable is not optional: without it PDN_UART keeps its standstill
     * current function and the UART cannot use the pin. mstep_reg_select hands
     * microstepping to the register instead of the MS1/MS2 straps, which are
     * doing double duty as the chip address. I_scale_analog off makes the
     * current purely digital rather than also scaled by the VREF trimmer. */
    gconf_set = TMC2209_GCONF_PDN_DISABLE | TMC2209_GCONF_MSTEP_REG_SELECT;
    gconf_clear = TMC2209_GCONF_I_SCALE_ANALOG | TMC2209_GCONF_EN_SPREADCYCLE |
                  TMC2209_GCONF_SHAFT;

    if (cfg->spreadcycle)
        gconf_set |= TMC2209_GCONF_EN_SPREADCYCLE;

    if (cfg->invert_direction)
        gconf_set |= TMC2209_GCONF_SHAFT;

    err = update(d, TMC2209_GCONF, gconf_clear, gconf_set);
    if (err)
        return err;

    /* TOFF must be non-zero or the bridges stay off. 3 is the StealthChop
     * default and works with any blank time. */
    err = update(d, TMC2209_CHOPCONF,
                 TMC2209_CHOP_MRES_MASK | TMC2209_CHOP_INTPOL |
                 TMC2209_CHOP_TOFF_MASK,
                 ((uint32_t)d->mres << TMC2209_CHOP_MRES_SHIFT) |
                 (cfg->interpolate ? TMC2209_CHOP_INTPOL : 0) | 3u);
    if (err)
        return err;

    /* Hold off the standstill current reduction so short pauses do not keep
     * dropping and restoring the current: 20 * 2^18 clocks (5.2 p.28), about
     * 0.44 s on the 12 MHz oscillator. */
    err = reg_write(d, TMC2209_TPOWERDOWN, 20);
    if (err)
        return err;

    return set_current(d, cfg->run_current_ma, cfg->hold_current_ma);
}

/* ------------------------------------------------- the stepper_dev ops -- */

static struct tmc2209_t *of(struct stepper_dev *stepper)
{
    return stepper->priv;
}

static int op_enable(struct stepper_dev *stepper, bool on)
{
    return enable(of(stepper), on);
}

/* Amperes to the milliamps the chip's own API takes, refusing what does
 * not fit a uint16_t; NaN fails every comparison and is refused too. */
static int op_set_current(struct stepper_dev *stepper, float run_a,
                          float hold_a)
{
    if (!(run_a > 0.0f && run_a <= 65.535f) ||
        !(hold_a >= 0.0f && hold_a <= 65.535f))
        return -EINVAL;

    return set_current(of(stepper), (uint16_t)lroundf(run_a * 1000.0f),
                       (uint16_t)lroundf(hold_a * 1000.0f));
}

static int op_read_status(struct stepper_dev *stepper,
                          struct stepper_status *status)
{
    tmc2209_status_t st;
    int err;

    err = read_status(of(stepper), &st);
    if (err)
        return err;

    status->standstill = st.standstill;
    status->stealthchop = st.stealthchop;
    status->overtemp_warning = st.overtemp_warning;
    status->overtemp = st.overtemp;
    status->short_to_gnd = st.short_to_gnd;
    status->short_to_supply = st.short_low_side;
    status->open_load = st.open_load;
    status->stall = false;      /* on the DIAG pin only, not in a register */
    status->was_reset = st.was_reset;
    status->current_scale = st.current_scale;
    return 0;
}

static const struct stepper_ops tmc2209_ops = {
    .enable = op_enable,
    .set_current = op_set_current,
    .read_status = op_read_status,
};

/* ------------------------------------------------------ the handle API -- */

stepper_handle_ret tmc2209_init(const tmc2209_config_t *cfg,
                                tmc2209_handle_t *out)
{
    tmc2209_config_t use;
    struct tmc2209_t *d;
    int mres;
    int err;

    if (cfg == NULL || out == NULL || cfg->address > TMC2209_MAX_SLAVE_ADDR)
        return STEPPER_HANDLE_RET(-EINVAL);
#ifndef ESP_PLATFORM
    if (cfg->uart == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
#endif

    use = *cfg;
    if (use.clock_hz == 0)
        use.clock_hz = DEFAULT_CLOCK_HZ;
    if (use.run_current_ma == 0)
        use.run_current_ma = DEFAULT_RUN_MA;
    if (use.microsteps == 0)
        use.microsteps = DEFAULT_MICROSTEPS;
    if (use.rsense_mohm <= 0.0f)
        return STEPPER_HANDLE_RET(-EINVAL);

    d = calloc(1, sizeof *d);
    if (d == NULL)
        return STEPPER_HANDLE_RET(-ENOMEM);

    mres = tmc2209_mres(use.microsteps);
    if (mres < 0) {
        free(d);
        return STEPPER_HANDLE_RET(-EINVAL);
    }

    d->stepper.name = "tmc2209";
    d->mres = (uint8_t)mres;
    d->address = use.address;
    d->two_wire = use.two_wire;
#ifdef ESP_PLATFORM
    d->en.gpio = use.en_gpio;
#else
    d->en.port = use.en_port;
    d->en.pin = use.en_pin;
#endif
    d->rsense_mohm = use.rsense_mohm;
    d->clock_hz = use.clock_hz;

    if (stepper_pin_wired(&d->en)) {
        /* Start with the output stage off — nothing should move until asked. */
        err = stepper_pin_output(&d->en, true);
        if (err)
            goto fail;
    }

    /* The UART is the app's, installed and configured. */
    err = stepper_uart_attach(&d->bus, use.uart, REPLY_TIMEOUT_MS);
    if (err)
        goto fail;

    /* The chip takes a moment after power-up before it answers. */
    stepper_sleep_ms(10);

    err = verify_link(d);
    if (err)
        goto fail_uart;

    err = apply_config(d, &use);
    if (err)
        goto fail_uart;

    d->stepper.ops = &tmc2209_ops;
    d->stepper.priv = d;
    d->stepper.microsteps = use.microsteps;
    *out = d;

    return STEPPER_HANDLE_RET(0);

fail_uart:
    stepper_bus_detach(&d->bus);
fail:
    free(d);

    return STEPPER_HANDLE_RET(err);
}

stepper_handle_ret tmc2209_enable(tmc2209_handle_t dev, bool enable_stage)
{
    if (dev == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(enable(dev, enable_stage));
}

stepper_handle_ret tmc2209_set_current(tmc2209_handle_t dev, uint16_t run_ma,
                                       uint16_t hold_ma)
{
    if (dev == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(set_current(dev, run_ma, hold_ma));
}

stepper_handle_ret tmc2209_move(tmc2209_handle_t dev, float usteps_per_s)
{
    if (dev == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(move(dev, usteps_per_s));
}

stepper_handle_ret tmc2209_stop(tmc2209_handle_t dev)
{
    return tmc2209_move(dev, 0.0f);
}

stepper_handle_ret tmc2209_read_status(tmc2209_handle_t dev,
                                       tmc2209_status_t *out)
{
    if (dev == NULL || out == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(read_status(dev, out));
}

stepper_handle_ret tmc2209_stallguard(tmc2209_handle_t dev, uint16_t *load)
{
    uint32_t v;
    int err;

    if (dev == NULL || load == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);

    err = reg_read(dev, TMC2209_SG_RESULT, &v);
    if (err)
        return STEPPER_HANDLE_RET(err);

    *load = (uint16_t)(v & TMC2209_SG_RESULT_MASK);

    return STEPPER_HANDLE_RET(0);
}

stepper_handle_ret tmc2209_set_stall(tmc2209_handle_t dev, uint8_t threshold,
                                     float min_usteps_per_s)
{
    uint32_t tcoolthrs;
    int err;

    if (dev == NULL || min_usteps_per_s <= 0.0f)
        return STEPPER_HANDLE_RET(-EINVAL);

    err = reg_write(dev, TMC2209_SGTHRS, threshold);
    if (err)
        return STEPPER_HANDLE_RET(err);

    tcoolthrs = tmc2209_tcoolthrs(dev->clock_hz, min_usteps_per_s, dev->mres);

    return STEPPER_HANDLE_RET(reg_write(dev, TMC2209_TCOOLTHRS,
                                        threshold ? tcoolthrs : 0));
}

stepper_handle_ret tmc2209_read(tmc2209_handle_t dev, uint8_t reg,
                                uint32_t *value)
{
    if (dev == NULL || value == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(reg_read(dev, reg, value));
}

stepper_handle_ret tmc2209_write(tmc2209_handle_t dev, uint8_t reg,
                                 uint32_t value)
{
    if (dev == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);
    return STEPPER_HANDLE_RET(reg_write(dev, reg, value));
}

stepper_handle_ret tmc2209_del(tmc2209_handle_t dev)
{
    if (dev == NULL)
        return STEPPER_HANDLE_RET(-EINVAL);

    /* Leave the motor free rather than holding torque into whatever is next. */
    if (stepper_pin_wired(&dev->en))
        stepper_pin_set(&dev->en, true);

    /* The UART is the app's: detached, not deleted. */
    stepper_bus_detach(&dev->bus);
    free(dev);

    return STEPPER_HANDLE_RET(0);
}

struct stepper_dev *tmc2209_stepper(tmc2209_handle_t dev)
{
    return dev ? &dev->stepper : NULL;
}

#endif /* CONFIG_STEPPER_TMC2209 */
