/*
 * Host-side check of the driver against a fake TMC5240. No hardware, no ESP
 * toolchain -- see test/Makefile.
 *
 * The fake models the one part of the SPI protocol that is easy to get wrong
 * and impossible to notice: a datagram returns the data belonging to the
 * *previous* datagram's address, not its own. A driver that reads with a
 * single transfer passes every casual smoke test and returns values that are
 * one access stale forever, so the read pipeline is checked first and hardest.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "tmc5240.h"
#include "tmc5240_math.h"
#include "tmc5240_regs.h"

/* ------------------------------------------------------- the fake chip --- */

static uint32_t regs[128];
static int      pending_read = -1;    /* address whose data is owed next time */
static uint32_t pending_write_echo;
static bool     pending_is_write;
static uint8_t  fake_status = 0x00;
static int      transfers;
static bool     lock_held;
static int      lock_takes;
static int      unlocked_transfers;   /* datagrams another task could split */
static int      coast_reads;          /* RAMPSTAT polls until a stopping ramp is still; 0 never */
static int      en_level;             /* last level driven on DRV_ENN, -1 if never */
static bool     enabled_while_moving;
static bool     configured_while_moving;
static uint32_t stop_amax;            /* AMAX when the coasting ramp reached standstill */
static uint32_t amax_at_enable;

static void chip_reset(void)
{
    memset(regs, 0, sizeof(regs));
    regs[TMC5240_INP_OUT] = (uint32_t)TMC5240_VERSION_FIRST << TMC5240_VERSION_SHIFT;
    regs[TMC5240_RAMPSTAT] = TMC5240_VZERO;     /* at power-on the ramp is still */
    pending_read = -1;
    pending_is_write = false;
    pending_write_echo = 0;
    transfers = 0;
    coast_reads = 0;
    en_level = -1;
    enabled_while_moving = false;
    configured_while_moving = false;
    stop_amax = 0;
    amax_at_enable = 0;
}

esp_err_t spi_bus_add_device(spi_host_device_t host,
                             const spi_device_interface_config_t *cfg,
                             spi_device_handle_t *out)
{
    (void)host;
    /* The datasheet is explicit: mode 3, and 10 MHz is the ceiling. */
    assert(cfg->mode == 3);
    assert(cfg->clock_speed_hz > 0 && cfg->clock_speed_hz <= 10000000);
    *out = (spi_device_handle_t)(void *)regs;
    return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t dev) { (void)dev; return ESP_OK; }

esp_err_t spi_device_polling_transmit(spi_device_handle_t dev, spi_transaction_t *t)
{
    const uint8_t *tx = t->tx_buffer;
    uint8_t *rx = t->rx_buffer;
    uint8_t addr = tx[0] & 0x7F;
    bool write = (tx[0] & TMC5240_WRITE) != 0;
    uint32_t data = ((uint32_t)tx[1] << 24) | ((uint32_t)tx[2] << 16) |
                    ((uint32_t)tx[3] << 8) | tx[4];
    uint32_t reply;

    (void)dev;
    assert(t->length == 40);           /* "A minimum of 40 SCK clock cycles" */
    transfers++;
    if (!lock_held)
        unlocked_transfers++;

    if (pending_is_write)
        reply = pending_write_echo;    /* a write echoes back what was sent */
    else if (pending_read >= 0)
        reply = regs[pending_read];
    else
        reply = 0xDEADBEEF;            /* nothing owed: junk, as on a real chip */

    if (write) {
        /* Current, chopper and StealthChop belong to a motor at standstill. */
        if (regs[TMC5240_VACTUAL] != 0 &&
            (addr == TMC5240_DRV_CONF || addr == TMC5240_IHOLD_IRUN ||
             addr == TMC5240_CHOPCONF || addr == TMC5240_GCONF))
            configured_while_moving = true;
        regs[addr] = data;
        pending_is_write = true;
        pending_write_echo = data;
        pending_read = -1;
    } else {
        pending_is_write = false;
        pending_read = addr;

        /* Velocity mode with VMAX 0 is a ramp down; model it as reaching
         * standstill after coast_reads polls of RAMPSTAT. */
        if (addr == TMC5240_RAMPSTAT && regs[TMC5240_VMAX] == 0 &&
            (regs[TMC5240_RAMPMODE] == TMC5240_MODE_VELPOS ||
             regs[TMC5240_RAMPMODE] == TMC5240_MODE_VELNEG) &&
            coast_reads > 0 && --coast_reads == 0) {
            regs[TMC5240_VACTUAL] = 0;
            regs[TMC5240_RAMPSTAT] |= TMC5240_VZERO;
            stop_amax = regs[TMC5240_AMAX];
        }
    }

    rx[0] = fake_status;
    rx[1] = (uint8_t)(reply >> 24);
    rx[2] = (uint8_t)(reply >> 16);
    rx[3] = (uint8_t)(reply >> 8);
    rx[4] = (uint8_t)reply;
    return ESP_OK;
}

esp_err_t gpio_config(const gpio_config_t *c) { (void)c; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t g, uint32_t l)
{
    (void)g;
    en_level = (int)l;
    if (l == 0 && regs[TMC5240_VACTUAL] != 0)
        enabled_while_moving = true;
    if (l == 0)
        amax_at_enable = regs[TMC5240_AMAX];
    return ESP_OK;
}

/* Time passes only in vTaskDelay, so a delay of 0 ticks is a busy spin. */
unsigned int      stub_tick_hz = 1000;
static TickType_t tick_count;
static int        zero_delays;
static TickType_t reached_at;         /* tick at which a move finishes; 0 never */

void vTaskDelay(TickType_t t)
{
    if (t == 0)
        zero_delays++;
    tick_count += t;
    if (reached_at && tick_count >= reached_at)
        regs[TMC5240_RAMPSTAT] |= TMC5240_POSITION_REACHED;
}

TickType_t xTaskGetTickCount(void) { return tick_count; }

/* One task, so a take while held can only be the driver locking twice --
 * which on the real, non-recursive mutex deadlocks forever. */
SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    static int m;
    return (SemaphoreHandle_t)(void *)&m;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t)
{
    (void)s; (void)t;
    assert(!lock_held);
    lock_held = true;
    lock_takes++;
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t s)
{
    (void)s;
    assert(lock_held);
    lock_held = false;
    return pdTRUE;
}

void vSemaphoreDelete(SemaphoreHandle_t s) { (void)s; }

/* ------------------------------------------------------------ helpers ---- */

static tmc5240_config_t base_cfg(void)
{
    tmc5240_config_t c = {
        .host = SPI2_HOST,
        .cs_gpio = 5,
        .en_gpio = -1,
        .rref_kohm = 12.0f,
        .run_current_a = 0.8f,
        .microsteps = 256,
    };
    return c;
}

static tmc5240_handle_t open_ok(void)
{
    tmc5240_config_t c = base_cfg();
    tmc5240_handle_t d = NULL;

    chip_reset();
    assert(tmc5240_init(&c, &d) == ESP_OK);
    assert(d);
    return d;
}

/* ------------------------------------------------- the read pipeline ----- */

static void test_read_pipeline(void)
{
    tmc5240_handle_t d = open_ok();
    uint32_t v;

    /* Park a value where nothing else writes it, then make sure the value
     * that comes back is this one and not whatever the previous access left
     * in the pipe. */
    regs[TMC5240_XLATCH] = 0x0BADC0DE;
    regs[TMC5240_MSCNT]  = 0x00001234;

    assert(tmc5240_read(d, TMC5240_XLATCH, &v) == ESP_OK);
    assert(v == 0x0BADC0DE);

    /* Two different registers back to back: a stale pipeline would return
     * XLATCH here. */
    assert(tmc5240_read(d, TMC5240_MSCNT, &v) == ESP_OK);
    assert(v == 0x00001234);
    assert(tmc5240_read(d, TMC5240_XLATCH, &v) == ESP_OK);
    assert(v == 0x0BADC0DE);

    /* A read immediately after a write must not pick up the write echo. */
    assert(tmc5240_write(d, TMC5240_XTARGET, 0x11111111) == ESP_OK);
    assert(tmc5240_read(d, TMC5240_MSCNT, &v) == ESP_OK);
    assert(v == 0x00001234);

    /* Each read costs exactly two datagrams. */
    transfers = 0;
    assert(tmc5240_read(d, TMC5240_MSCNT, &v) == ESP_OK);
    assert(transfers == 2);

    tmc5240_del(d);
}

static void test_lock(void)
{
    tmc5240_handle_t d = open_ok();
    uint32_t v;

    /* Another task's datagram between the two halves of a read would take
     * this read's data, and this read would get its echo. Both halves have to
     * sit inside one hold of the lock; so does every write. */
    lock_takes = 0;
    unlocked_transfers = 0;
    assert(tmc5240_read(d, TMC5240_MSCNT, &v) == ESP_OK);
    assert(lock_takes == 1 && unlocked_transfers == 0 && !lock_held);
    assert(tmc5240_write(d, TMC5240_XTARGET, 1) == ESP_OK);
    assert(lock_takes == 2 && unlocked_transfers == 0 && !lock_held);

    tmc5240_del(d);
}

/* ------------------------------------------------------------- probe ----- */

static void test_probe(void)
{
    tmc5240_config_t c = base_cfg();
    tmc5240_handle_t d = NULL;

    /* Wrong VERSION is a miswired bus, not a working chip. */
    chip_reset();
    regs[TMC5240_INP_OUT] = 0x00000000;
    assert(tmc5240_init(&c, &d) == ESP_ERR_NOT_FOUND);

    chip_reset();
    regs[TMC5240_INP_OUT] = 0x41u << TMC5240_VERSION_SHIFT;
    assert(tmc5240_init(&c, &d) == ESP_ERR_NOT_FOUND);

    /* And the driver must leave the stage disabled rather than half-set-up. */
    d = open_ok();
    tmc5240_del(d);
}

static void turning_chip(void)
{
    chip_reset();
    regs[TMC5240_VACTUAL]  = 100000;
    regs[TMC5240_VMAX]     = 100000;
    regs[TMC5240_RAMPMODE] = TMC5240_MODE_VELPOS;
    regs[TMC5240_RAMPSTAT] = 0;
}

static void test_init_stops_turning_motor(void)
{
    tmc5240_config_t c = base_cfg();
    tmc5240_handle_t d = NULL;
    uint32_t amax = tmc5240_fit(tmc5240_hzs_to_a(51200.0, TMC5240_FCLK_INTERNAL),
                                TMC5240_A_BITS);

    /* An ESP reboot does not reset the TMC5240, so it can still be turning,
     * and HOLD keeps that velocity. Init must ramp it down before it
     * configures anything or lets the driver stage on. With the stage off the
     * motor only coasts, so the ramp may stop at maximum AMAX -- but the
     * ramp's own AMAX is back first. */
    c.en_gpio = 9;
    turning_chip();
    coast_reads = 3;
    assert(tmc5240_init(&c, &d) == ESP_OK);
    assert(en_level == 0 && !enabled_while_moving && !configured_while_moving);
    assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_HOLD);
    assert(stop_amax == (1u << TMC5240_A_BITS) - 1);
    assert(amax_at_enable == amax);
    tmc5240_del(d);

    /* DRV_ENN tied low keeps the stage powered, where an abrupt stop is the
     * overcurrent case: the default ramp's AMAX, not the maximum. */
    c.en_gpio = -1;
    turning_chip();
    coast_reads = 3;
    assert(tmc5240_init(&c, &d) == ESP_OK);
    assert(!configured_while_moving);
    assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_HOLD);
    assert(stop_amax == amax);
    tmc5240_del(d);

    /* A ramp that never stops is an error, with the stage left off. */
    c.en_gpio = 9;
    turning_chip();
    assert(tmc5240_init(&c, &d) == ESP_ERR_TIMEOUT);
    assert(en_level == 1);
}

static void test_bad_config(void)
{
    tmc5240_config_t c;
    tmc5240_handle_t d = NULL;

    chip_reset();

    c = base_cfg(); c.microsteps = 3;
    assert(tmc5240_init(&c, &d) == ESP_ERR_INVALID_ARG);
    c = base_cfg(); c.microsteps = 0;
    assert(tmc5240_init(&c, &d) == ESP_ERR_INVALID_ARG);
    c = base_cfg(); c.rref_kohm = 5.0f;
    assert(tmc5240_init(&c, &d) == ESP_ERR_INVALID_ARG);
    c = base_cfg(); c.rref_kohm = 100.0f;
    assert(tmc5240_init(&c, &d) == ESP_ERR_INVALID_ARG);

    /* 5 A RMS is past every range on a legal reference resistor. */
    c = base_cfg(); c.run_current_a = 5.0f;
    assert(tmc5240_init(&c, &d) == ESP_ERR_INVALID_ARG);
    c = base_cfg(); c.run_current_a = 0.0f;
    assert(tmc5240_init(&c, &d) == ESP_ERR_INVALID_ARG);

    assert(tmc5240_init(NULL, &d) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_init(&c, NULL) == ESP_ERR_INVALID_ARG);
}

/* ----------------------------------------------------------- current ----- */

static void test_current(void)
{
    tmc5240_handle_t d = open_ok();
    uint32_t ihold_irun;
    double full;

    /* 0.8 A on a 12k reference needs the 2 A range (range 0 tops out at
     * 0.69 A RMS), and DRV_CONF must say so. */
    assert(regs[TMC5240_DRV_CONF] == 1);
    full = tmc5240_range_rms(1, 12.0);

    ihold_irun = regs[TMC5240_IHOLD_IRUN];
    uint8_t irun  = (ihold_irun >> TMC5240_IRUN_SHIFT) & 0x1F;
    uint8_t ihold = (ihold_irun >> TMC5240_IHOLD_SHIFT) & 0x1F;

    assert(irun == tmc5240_cs(0.8, full));
    /* Hold defaults to half of run. */
    assert(ihold == tmc5240_cs(0.4, full));
    assert(fabs(tmc5240_run_current(d) - tmc5240_cs_rms(irun, full)) < 1e-6);

    /* GLOBAL_SCALER 0 is full scale -- never a value in the forbidden 1..31. */
    assert(regs[TMC5240_GLOBAL_SCALER] == 0);

    /* A lower current should drop to the smallest range that still fits. */
    assert(tmc5240_set_current(d, 0.5f, 0.1f) == ESP_OK);
    assert(regs[TMC5240_DRV_CONF] == 0);

    assert(tmc5240_set_current(d, 5.0f, 0.0f) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_set_current(d, -1.0f, 0.0f) == ESP_ERR_INVALID_ARG);

    tmc5240_del(d);
}

/* --------------------------------------------------------- microsteps ---- */

static void test_microsteps(void)
{
    static const int cases[] = { 256, 128, 64, 32, 16, 8, 4, 2, 1 };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        tmc5240_config_t c = base_cfg();
        tmc5240_handle_t d = NULL;

        c.microsteps = cases[i];
        chip_reset();
        assert(tmc5240_init(&c, &d) == ESP_OK);

        uint32_t mres = (regs[TMC5240_CHOPCONF] >> TMC5240_MRES_SHIFT) & 0xF;
        assert((int)mres == tmc5240_mres(cases[i]));
        assert(regs[TMC5240_CHOPCONF] & TMC5240_INTPOL);
        /* TOFF must never be 0 -- that switches the driver stage off. */
        assert((regs[TMC5240_CHOPCONF] & 0xF) != 0);

        tmc5240_del(d);
    }
}

/* --------------------------------------------------------- direction ----- */

static void test_direction(void)
{
    tmc5240_config_t c = base_cfg();
    tmc5240_handle_t d = NULL;

    c.invert_direction = true;
    chip_reset();
    assert(tmc5240_init(&c, &d) == ESP_OK);

    /* shaft is GCONF bit 4 (datasheet GCONF table); bit 3 is reserved. A
     * literal, not the macro, so a wrong macro cannot agree with itself. */
    assert(regs[TMC5240_GCONF] == 0x10);

    tmc5240_del(d);
}

/* -------------------------------------------------------------- ramp ----- */

static void test_ramp(void)
{
    tmc5240_handle_t d = open_ok();
    tmc5240_ramp_t r = { .vmax = 51200.0f, .amax = 51200.0f, .dmax = 51200.0f };

    /* Positioning, so set_ramp() writes VMAX; see test_ramp_velocity_mode. */
    assert(tmc5240_move_to(d, 0) == ESP_OK);
    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    assert(regs[TMC5240_VMAX] ==
           tmc5240_fit(tmc5240_hz_to_v(51200.0, TMC5240_FCLK_INTERNAL),
                       TMC5240_V_BITS));
    assert(regs[TMC5240_AMAX] ==
           tmc5240_fit(tmc5240_hzs_to_a(51200.0, TMC5240_FCLK_INTERNAL),
                       TMC5240_A_BITS));

    /* VSTOP of 0 stalls the ramp generator forever; it must be floored. */
    assert(regs[TMC5240_VSTOP] >= 10);

    /* A velocity past the 23-bit register is refused, not silently clamped
     * into a motor command nobody asked for. */
    r.vmax = 1e12f;
    assert(tmc5240_set_ramp(d, &r) == ESP_ERR_INVALID_ARG);
    r.vmax = 51200.0f;
    r.amax = 1e12f;
    assert(tmc5240_set_ramp(d, &r) == ESP_ERR_INVALID_ARG);

    /* Zero acceleration would never start the motor. */
    r.amax = 0.0f;
    assert(tmc5240_set_ramp(d, &r) == ESP_ERR_INVALID_ARG);
    r.amax = 51200.0f; r.dmax = 0.0f;
    assert(tmc5240_set_ramp(d, &r) == ESP_ERR_INVALID_ARG);
    r.dmax = 51200.0f; r.vmax = 0.0f;
    assert(tmc5240_set_ramp(d, &r) == ESP_ERR_INVALID_ARG);

    /* An external clock scales every ramp register. */
    tmc5240_del(d);
    tmc5240_config_t c = base_cfg();
    c.clock_hz = 16000000;
    chip_reset();
    regs[TMC5240_INP_OUT] |= TMC5240_EXT_CLK;
    assert(tmc5240_init(&c, &d) == ESP_OK);
    r.vmax = 51200.0f; r.amax = 51200.0f; r.dmax = 51200.0f;
    assert(tmc5240_move_to(d, 0) == ESP_OK);
    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    assert(regs[TMC5240_VMAX] ==
           tmc5240_fit(tmc5240_hz_to_v(51200.0, 16e6), TMC5240_V_BITS));
    assert(regs[TMC5240_VMAX] !=
           tmc5240_fit(tmc5240_hz_to_v(51200.0, TMC5240_FCLK_INTERNAL),
                       TMC5240_V_BITS));
    tmc5240_del(d);
}

static void test_ramp_velocity_mode(void)
{
    tmc5240_handle_t d = open_ok();
    tmc5240_ramp_t r = { .vmax = 20000.0f, .amax = 51200.0f, .dmax = 51200.0f };
    uint32_t run_v;

    /* In velocity mode VMAX is the speed itself. A new ramp after stop()
     * must not set the motor off again at cruise velocity... */
    assert(tmc5240_stop(d) == ESP_OK);
    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    assert(regs[TMC5240_VMAX] == 0);
    assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_VELPOS);

    /* ...nor change the speed run() asked for. */
    assert(tmc5240_run(d, 1000.0f) == ESP_OK);
    run_v = regs[TMC5240_VMAX];
    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    assert(regs[TMC5240_VMAX] == run_v);

    /* Mid-move, the new cruise velocity applies at once. */
    assert(tmc5240_move_to(d, 0) == ESP_OK);
    r.vmax = 30000.0f;
    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    assert(regs[TMC5240_VMAX] ==
           tmc5240_fit(tmc5240_hz_to_v(30000.0, TMC5240_FCLK_INTERNAL),
                       TMC5240_V_BITS));

    tmc5240_del(d);
}

static void test_ramp_all_or_nothing(void)
{
    tmc5240_handle_t d = open_ok();
    tmc5240_ramp_t r = { .vmax = 20000.0f, .amax = 20000.0f, .dmax = 20000.0f,
                         .tzerowait_ms = 3000 };
    uint32_t before[128];

    /* 3 s of TZEROWAIT is past its 16 bits, and it is the last field
     * converted. A rejected ramp must leave the chip on the old one, not on
     * new velocities and accelerations with the old dwell. */
    memcpy(before, regs, sizeof(regs));
    assert(tmc5240_set_ramp(d, &r) == ESP_ERR_INVALID_ARG);
    assert(memcmp(before, regs, sizeof(regs)) == 0);

    tmc5240_del(d);
}

/* ------------------------------------------------------------ motion ----- */

static void test_motion(void)
{
    tmc5240_handle_t d = open_ok();
    int32_t pos;
    float v;

    assert(tmc5240_move_to(d, -12345) == ESP_OK);
    assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_POSITION);
    assert((int32_t)regs[TMC5240_XTARGET] == -12345);

    /* Relative moves stack on the target, so two in a row do not lose one. */
    assert(tmc5240_move_by(d, 45) == ESP_OK);
    assert((int32_t)regs[TMC5240_XTARGET] == -12300);
    assert(tmc5240_move_by(d, -300) == ESP_OK);
    assert((int32_t)regs[TMC5240_XTARGET] == -12600);

    /* Velocity mode: the sign picks the ramp mode, the magnitude VMAX. */
    assert(tmc5240_run(d, 1000.0f) == ESP_OK);
    assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_VELPOS);
    assert(tmc5240_run(d, -1000.0f) == ESP_OK);
    assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_VELNEG);
    assert(regs[TMC5240_VMAX] ==
           tmc5240_fit(tmc5240_hz_to_v(1000.0, TMC5240_FCLK_INTERNAL),
                       TMC5240_V_BITS));

    assert(tmc5240_stop(d) == ESP_OK);
    assert(regs[TMC5240_VMAX] == 0);

    /* XACTUAL is signed and comes straight back. */
    regs[TMC5240_XACTUAL] = (uint32_t)(int32_t)-99;
    assert(tmc5240_position(d, &pos) == ESP_OK);
    assert(pos == -99);

    /* VACTUAL is a signed 24-bit field inside a 32-bit read. */
    regs[TMC5240_VACTUAL] = 0x00FFFFFF;                 /* -1 */
    assert(tmc5240_velocity(d, &v) == ESP_OK);
    assert(v < 0.0f);
    assert(fabs(v - tmc5240_v_to_hz(-1, TMC5240_FCLK_INTERNAL)) < 1e-6);

    regs[TMC5240_VACTUAL] = 0x00800000;                 /* most negative */
    assert(tmc5240_velocity(d, &v) == ESP_OK);
    assert(v < 0.0f);

    tmc5240_del(d);
}

static void test_move_after_stop(void)
{
    tmc5240_handle_t d = open_ok();
    tmc5240_ramp_t r = { .vmax = 20000.0f, .amax = 51200.0f, .dmax = 51200.0f };
    uint32_t vmax = tmc5240_fit(tmc5240_hz_to_v(20000.0, TMC5240_FCLK_INTERNAL),
                                TMC5240_V_BITS);

    /* Positioning cruises at whatever is in VMAX. stop() leaves 0 there, and
     * a move that inherits it never starts. */
    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    assert(tmc5240_stop(d) == ESP_OK);
    assert(tmc5240_move_to(d, 1000) == ESP_OK);
    assert(regs[TMC5240_VMAX] == vmax);

    /* Nor may run()'s speed leak into the next move. */
    assert(tmc5240_run(d, 1000.0f) == ESP_OK);
    assert(tmc5240_move_to(d, 0) == ESP_OK);
    assert(regs[TMC5240_VMAX] == vmax);

    tmc5240_del(d);
}

static void test_stop_hard(void)
{
    tmc5240_handle_t d = open_ok();
    tmc5240_ramp_t r = { .vmax = 51200.0f, .amax = 51200.0f, .dmax = 51200.0f };
    uint32_t amax;

    assert(tmc5240_set_ramp(d, &r) == ESP_OK);
    amax = regs[TMC5240_AMAX];

    /* Velocity mode decelerates with whatever AMAX holds for the whole stop,
     * so the borrowed maximum has to stay there after the call returns. */
    assert(tmc5240_stop_hard(d) == ESP_OK);
    assert(regs[TMC5240_VMAX] == 0);
    assert(regs[TMC5240_AMAX] == (1u << TMC5240_A_BITS) - 1);

    /* The next motion call gives the configured ramp back. */
    assert(tmc5240_move_to(d, 0) == ESP_OK);
    assert(regs[TMC5240_AMAX] == amax);
    assert(tmc5240_stop_hard(d) == ESP_OK);
    assert(tmc5240_run(d, 1000.0f) == ESP_OK);
    assert(regs[TMC5240_AMAX] == amax);
    assert(tmc5240_stop_hard(d) == ESP_OK);
    assert(tmc5240_stop(d) == ESP_OK);
    assert(regs[TMC5240_AMAX] == amax);

    tmc5240_del(d);
}

static void test_move_by_after_run(void)
{
    tmc5240_handle_t d = open_ok();

    /* After run() and stop() XTARGET still says 0, but the motor is at
     * 100000. Adding to the stale target would drive 99,900 steps back. */
    regs[TMC5240_XACTUAL] = 100000;
    assert(tmc5240_run(d, 1000.0f) == ESP_OK);
    assert(tmc5240_stop(d) == ESP_OK);
    assert(tmc5240_move_by(d, 100) == ESP_OK);
    assert((int32_t)regs[TMC5240_XTARGET] == 100100);

    /* XTARGET is 32-bit two's complement and the chip wraps it, so the
     * driver's sum has to wrap too rather than overflow. */
    assert(tmc5240_move_to(d, INT32_MAX) == ESP_OK);
    assert(tmc5240_move_by(d, 1) == ESP_OK);
    assert((int32_t)regs[TMC5240_XTARGET] == INT32_MIN);

    tmc5240_del(d);
}

static void test_set_position(void)
{
    tmc5240_handle_t d = open_ok();

    /* Rewriting the counter mid-move corrupts the ramp, so it is refused. */
    regs[TMC5240_RAMPSTAT] = 0;                          /* vzero clear = moving */
    assert(tmc5240_set_position(d, 500) == ESP_ERR_INVALID_STATE);

    regs[TMC5240_RAMPSTAT] = TMC5240_VZERO;
    assert(tmc5240_set_position(d, 500) == ESP_OK);
    assert((int32_t)regs[TMC5240_XACTUAL] == 500);
    /* Target has to follow, or the chip immediately drives back to the old one. */
    assert((int32_t)regs[TMC5240_XTARGET] == 500);

    tmc5240_del(d);
}

static void test_wait(void)
{
    tmc5240_handle_t d = open_ok();

    regs[TMC5240_RAMPSTAT] = 0;
    assert(tmc5240_wait(d, 20) == ESP_ERR_TIMEOUT);

    regs[TMC5240_RAMPSTAT] = TMC5240_POSITION_REACHED;
    assert(tmc5240_wait(d, 20) == ESP_OK);
    /* A zero timeout still gets one look, so an already-finished move passes. */
    assert(tmc5240_wait(d, 0) == ESP_OK);

    tmc5240_del(d);
}

static void test_wait_tick_rate(void)
{
    tmc5240_handle_t d = open_ok();
    TickType_t start;

    /* At 100 Hz, pdMS_TO_TICKS(5) is 0 and vTaskDelay(0) only yields.
     * Counting 5 ms per poll would then spin through a 100 ms timeout in
     * microseconds. It must sleep, and since the first tick is only partly
     * waited, give up only after 11 ticks. */
    stub_tick_hz = 100;
    regs[TMC5240_RAMPSTAT] = 0;
    zero_delays = 0;
    start = xTaskGetTickCount();
    assert(tmc5240_wait(d, 100) == ESP_ERR_TIMEOUT);
    assert(zero_delays == 0);
    assert(xTaskGetTickCount() - start >= 11);

    /* 13 ms is two ticks rounded up, three with the partial one. A move that
     * finishes at the third tick is still inside the timeout. */
    regs[TMC5240_RAMPSTAT] = 0;
    reached_at = xTaskGetTickCount() + 3;
    assert(tmc5240_wait(d, 13) == ESP_OK);
    reached_at = 0;
    stub_tick_hz = 1000;

    /* UINT32_MAX is the usual "wait forever". Its tick count overflows
     * TickType_t, and must saturate rather than wrap to a zero timeout. */
    regs[TMC5240_RAMPSTAT] = 0;
    reached_at = xTaskGetTickCount() + 50;
    assert(tmc5240_wait(d, UINT32_MAX) == ESP_OK);
    reached_at = 0;

    tmc5240_del(d);
}

/* ------------------------------------------------------------ status ----- */

static void test_status(void)
{
    tmc5240_handle_t d = open_ok();
    tmc5240_status_t st;

    regs[TMC5240_DRVSTATUS] = TMC5240_STST | TMC5240_OTPW | TMC5240_OLB |
                              TMC5240_STALLGUARD | TMC5240_S2VSA |
                              (17u << TMC5240_CS_ACTUAL_SHIFT) | 123u;
    regs[TMC5240_RAMPSTAT] = TMC5240_POSITION_REACHED | TMC5240_STATUS_STOP_L;
    regs[TMC5240_XACTUAL] = (uint32_t)(int32_t)-7;
    regs[TMC5240_VACTUAL] = 1000;       /* the last slot of the read chain */

    assert(tmc5240_status(d, &st) == ESP_OK);
    assert(st.standstill && st.overtemp_warn && st.open_load_b);
    assert(st.stallguard && st.short_vs_a);
    assert(!st.overtemp && !st.open_load_a && !st.short_gnd_a && !st.short_vs_b);
    assert(st.sg_result == 123);
    assert(st.cs_actual == 17);
    assert(st.position_reached && st.stopped_left && !st.stopped_right);
    assert(st.position == -7);
    assert(fabs(st.velocity - tmc5240_v_to_hz(1000, TMC5240_FCLK_INTERNAL)) < 1e-3);

    /* Every field is rewritten each call, never left over from the last one. */
    regs[TMC5240_DRVSTATUS] = 0;
    regs[TMC5240_RAMPSTAT] = 0;
    assert(tmc5240_status(d, &st) == ESP_OK);
    assert(!st.standstill && !st.overtemp_warn && !st.stallguard);
    assert(st.sg_result == 0 && st.cs_actual == 0);

    /* Four registers chained through the pipeline: each datagram asks for
     * the next one, so five datagrams, all inside one hold of the lock. */
    transfers = 0;
    lock_takes = 0;
    assert(tmc5240_status(d, &st) == ESP_OK);
    assert(transfers == 5 && lock_takes == 1);

    tmc5240_del(d);
}

static void test_diagnostics(void)
{
    tmc5240_handle_t d = open_ok();
    float x;

    regs[TMC5240_ADC_VSUPPLY_AIN] = 2467;
    assert(tmc5240_supply_voltage(d, &x) == ESP_OK);
    assert(fabs(x - 24.01) < 0.01);

    regs[TMC5240_ADC_TEMP] = 2038 + 77;
    assert(tmc5240_temperature(d, &x) == ESP_OK);
    assert(fabs(x - 10.0) < 0.05);

    /* When the chip says its ADC is broken, neither reading is offered. */
    regs[TMC5240_INP_OUT] |= TMC5240_ADC_ERR;
    assert(tmc5240_supply_voltage(d, &x) == ESP_ERR_INVALID_STATE);
    assert(tmc5240_temperature(d, &x) == ESP_ERR_INVALID_STATE);

    tmc5240_del(d);
}

static void test_stallguard(void)
{
    tmc5240_handle_t d = open_ok();

    assert(tmc5240_stallguard(d, 40, 1000.0f) == ESP_OK);
    assert(regs[TMC5240_SG4_THRS] == 40);
    assert(regs[TMC5240_TCOOLTHRS] > 0);

    /* Threshold 0 means off, and must also disable the velocity window
     * rather than leave StallGuard armed with no threshold. */
    assert(tmc5240_stallguard(d, 0, 1000.0f) == ESP_OK);
    assert(regs[TMC5240_SG4_THRS] == 0);
    assert(regs[TMC5240_TCOOLTHRS] == 0);

    tmc5240_del(d);
}

static void test_null_arguments(void)
{
    tmc5240_status_t st;
    int32_t i32;
    float f;
    uint32_t u32;

    assert(tmc5240_set_ramp(NULL, NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_move_to(NULL, 0) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_move_by(NULL, 0) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_run(NULL, 0) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_stop(NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_stop_hard(NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_wait(NULL, 0) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_position(NULL, &i32) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_velocity(NULL, &f) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_set_position(NULL, 0) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_status(NULL, &st) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_set_current(NULL, 1.0f, 0.0f) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_supply_voltage(NULL, &f) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_temperature(NULL, &f) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_stallguard(NULL, 1, 1.0f) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_read(NULL, 0, &u32) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_write(NULL, 0, 0) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_del(NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_run_current(NULL) == 0.0f);
    assert(tmc5240_spi_status(NULL) == 0);

    tmc5240_handle_t d = open_ok();
    assert(tmc5240_position(d, NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_velocity(d, NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_status(d, NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_read(d, 0, NULL) == ESP_ERR_INVALID_ARG);
    assert(tmc5240_set_ramp(d, NULL) == ESP_ERR_INVALID_ARG);
    tmc5240_del(d);
}

static void test_spi_status(void)
{
    tmc5240_handle_t d;

    fake_status = TMC5240_SPI_STANDSTILL | TMC5240_SPI_RESET_FLAG;
    d = open_ok();
    assert(tmc5240_spi_status(d) == (TMC5240_SPI_STANDSTILL | TMC5240_SPI_RESET_FLAG));
    tmc5240_del(d);
    fake_status = 0;
}

int main(void)
{
    test_read_pipeline();
    test_lock();
    test_probe();
    test_init_stops_turning_motor();
    test_bad_config();
    test_current();
    test_microsteps();
    test_direction();
    test_ramp();
    test_ramp_velocity_mode();
    test_ramp_all_or_nothing();
    test_motion();
    test_move_after_stop();
    test_stop_hard();
    test_move_by_after_run();
    test_set_position();
    test_wait();
    test_wait_tick_rate();
    test_status();
    test_diagnostics();
    test_stallguard();
    test_null_arguments();
    test_spi_status();
    printf("all tmc5240 driver checks passed\n");
    return 0;
}
