/*
 * Pure register <-> physical-unit conversions, split out so they can be
 * compiled and asserted on the host (see test/test_tmc5240_math.c) without
 * a toolchain. Every constant here is quoted from docs/tmc5240.pdf.
 *
 * This is where a TMC5240 driver silently goes wrong. The ramp registers are
 * not in Hz, they are in units of fCLK, and the chip does not know what fCLK
 * you gave it. Feed the wrong clock and the motor runs at the wrong speed
 * with no error anywhere.
 */
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Internal oscillator, "0: The internal oscillator is used for generating the
 * clock-signal (12.5 MHz)" (INP_OUT.EXT_CLK). The EC table gives 11.9 typ 12.5
 * max 13.2 MHz -- a +/-5% speed error if you trust it. Clock the chip
 * externally when velocity has to be right. */
#define TMC5240_FCLK_INTERNAL   12500000.0

/* Ramp register widths, from each register's entry (p.105-115). Note these
 * are wider than the TMC5130/5160 equivalents: acceleration is 18 bits here,
 * not 16. */
#define TMC5240_V_BITS          23      /* VMAX, p.109 */
#define TMC5240_V1_BITS         20      /* V1 p.107, V2 p.114 */
#define TMC5240_VSTART_BITS     18      /* VSTART p.106, VSTOP p.112 */
#define TMC5240_A_BITS          18      /* AMAX, A1, A2, DMAX, D1, D2 */
#define TMC5240_VACTUAL_BITS    24      /* signed, p.105 */

#define TMC5240_TWO_P24         16777216.0        /* 2^24 */
#define TMC5240_TWO_P42         4398046511104.0   /* 2^42 */

/* Two's complement sign extension of the low `bits` of v. */
static inline int32_t tmc5240_sext(uint32_t v, int bits)
{
    uint32_t sign = 1u << (bits - 1);
    uint32_t msk  = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1);

    v &= msk;
    return (int32_t)((v ^ sign) - sign);
}

/*
 * Register values have to land in a field of a known width, and the inputs
 * reach here unscreened: a zero or negative fCLK makes the division produce
 * infinity, and an ambitious velocity produces a number far past the field.
 * Screen it once, here.
 *
 * Written as a rejection so that NaN, which compares false against
 * everything, falls into it too.
 */
static inline bool tmc5240_fits(double x, int bits)
{
    return x >= 0.0 && x <= (double)((1u << bits) - 1u);
}

/* Clamp into the field. Out-of-range is a caller error the driver reports as
 * ESP_ERR_INVALID_ARG; this only decides what gets written if it does not. */
static inline uint32_t tmc5240_fit(double x, int bits)
{
    uint32_t max = (1u << bits) - 1u;

    if (!(x >= 0.0))            /* catches NaN */
        return 0;
    if (x >= (double)max)
        return max;
    return (uint32_t)(x + 0.5);
}

/* ---- ramp generator, Table 19 "Ramp Generator Parameters vs. Units" (p.49) ----
 *
 *   v[Hz]   = v[TMC5240] * fCLK / 2^24
 *   a[Hz/s] = a[TMC5240] * fCLK^2 / 2^42
 *
 * "Hz" here means microsteps per second at the sequencer's 1/256 resolution.
 *
 * 2^42 is the TMC5240 datasheet's. The TMC5130/5160 datasheets divide by
 * 2^41 -- the (512 * 256) * 2^24 form -- and esp-tmc5240-stepper did too,
 * so every acceleration it wrote was half what was asked. Either the TMC5240
 * really changed, or its datasheet is wrong; this library follows the
 * datasheet until a bench run settles it (docs/chips.md).
 */
static inline double tmc5240_hz_to_v(double hz, double fclk)
{
    return hz * TMC5240_TWO_P24 / fclk;
}
static inline double tmc5240_v_to_hz(int32_t v, double fclk)
{
    return (double)v * fclk / TMC5240_TWO_P24;
}
static inline double tmc5240_hzs_to_a(double hz_s, double fclk)
{
    return hz_s * TMC5240_TWO_P42 / (fclk * fclk);
}
static inline double tmc5240_a_to_hzs(int32_t a, double fclk)
{
    return (double)a * fclk * fclk / TMC5240_TWO_P42;
}

/* Microsteps consumed accelerating from standstill to v: rs = v^2 / a / 2^8
 * (Table 19).
 * Both arguments are register units, and so is the result. Useful for
 * checking a move is long enough to ever reach VMAX. */
static inline double tmc5240_ramp_usteps(double v_reg, double a_reg)
{
    return (v_reg * v_reg) / a_reg / 256.0;
}

/* Velocity thresholds (TPWMTHRS, TCOOLTHRS, THIGH) are expressed as a step
 * period, not a velocity: TSTEP = 2^24 / (v[TMC5240] * 256 / USC), Table 19.
 * Higher velocity means a *smaller* TSTEP, so the comparisons invert. */
static inline double tmc5240_v_to_tstep(double v_reg, int usteps)
{
    return TMC5240_TWO_P24 / (v_reg * 256.0 / (double)usteps);
}

/* ---- motor current ----
 *
 *   IFS = KIFS(A x kOhm) / RREF(kOhm),          a *peak* current (p.45)
 *   I_rms = IFS / sqrt(2) * (CS + 1)/32 * GLOBALSCALER/256
 *
 * KIFS is selected by DRV_CONF.CURRENT_RANGE (Table 16). Ranges 2 and 3 are
 * the same 3 A; the driver never picks 3.
 */
#define TMC5240_RREF_MIN_KOHM   12.0
#define TMC5240_RREF_MAX_KOHM   60.0

static inline double tmc5240_kifs(int range)
{
    static const double k[4] = { 11.75, 24.0, 36.0, 36.0 };

    return (range >= 0 && range < 4) ? k[range] : 0.0;
}

/* Full-scale RMS current for a range, with CS = 31 and GLOBALSCALER = 256.
 * Reproduces Table 17. */
static inline double tmc5240_range_rms(int range, double rref_kohm)
{
    if (!(rref_kohm >= TMC5240_RREF_MIN_KOHM && rref_kohm <= TMC5240_RREF_MAX_KOHM))
        return 0.0;
    return tmc5240_kifs(range) / rref_kohm / sqrt(2.0);
}

/* Smallest range that still covers want_rms, for the best current resolution.
 * "Select the lowest fitting range for best current precision." Returns -1 if
 * even the 3 A range cannot reach it. */
static inline int tmc5240_pick_range(double want_rms, double rref_kohm)
{
    for (int r = 0; r <= 2; r++) {
        double full = tmc5240_range_rms(r, rref_kohm);

        if (full > 0.0 && want_rms <= full)
            return r;
    }
    return -1;
}

/* CS for a target current inside a chosen range. CS counts in 1/32 steps of
 * full scale, so 0 means 1/32, not off. */
static inline uint8_t tmc5240_cs(double want_rms, double full_rms)
{
    double cs;

    if (!(full_rms > 0.0) || !(want_rms > 0.0))
        return 0;
    cs = want_rms / full_rms * 32.0 - 1.0;
    if (cs < 0.0)
        return 0;
    if (cs > 31.0)
        return 31;
    return (uint8_t)(cs + 0.5);
}

/* What that CS actually delivers -- the value to report back, because the
 * quantisation is coarse enough to matter at low currents. */
static inline double tmc5240_cs_rms(uint8_t cs, double full_rms)
{
    return full_rms * ((double)cs + 1.0) / 32.0;
}

/* ---- on-chip ADC: ADC_VSUPPLY_AIN and ADC_TEMP entries, p.128-129 ---- */
#define TMC5240_VSUPPLY_LSB_V   9.732e-3
#define TMC5240_AIN_LSB_V       305.2e-6
#define TMC5240_TEMP_OFFSET     2038.0
#define TMC5240_TEMP_DIV        7.7

static inline float tmc5240_vsupply_v(uint32_t raw)
{
    return (float)((raw & 0x1FFFu) * TMC5240_VSUPPLY_LSB_V);
}
static inline float tmc5240_ain_v(uint32_t raw)
{
    return (float)(((raw >> 16) & 0x1FFFu) * TMC5240_AIN_LSB_V);
}
static inline float tmc5240_temp_c(uint32_t raw)
{
    return (float)(((double)(raw & 0x1FFFu) - TMC5240_TEMP_OFFSET) / TMC5240_TEMP_DIV);
}

/* ---- microstep resolution ----
 * CHOPCONF.MRES: 0 = 256 microsteps, each step halves, 8 = fullstep. */
static inline int tmc5240_mres(int usteps)
{
    for (int m = 0; m <= 8; m++)
        if (usteps == (256 >> m))
            return m;
    return -1;
}
static inline int tmc5240_usteps(int mres)
{
    return (mres >= 0 && mres <= 8) ? (256 >> mres) : -1;
}
