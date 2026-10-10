/*
 * Host-side check of the register conversions. No ESP toolchain needed:
 *
 *     cc -o /tmp/t test/test_tmc5240_math.c -lm && /tmp/t
 *
 * Two kinds of check live here. The datasheet section holds values copied
 * straight out of docs/tmc5240.pdf, so the constants are provably the right
 * ones. The edge sections hold the inputs that silently produce a wrong
 * number rather than an obviously broken one: sign boundaries, field widths,
 * and the points where a value stops fitting its register.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../src/tmc5240/tmc5240_math.h"
#include "../src/tmc5240/tmc5240_regs.h"

#define NEAR(a, b)   (fabs((double)(a) - (double)(b)) < 1e-6 * fabs(b) + 1e-9)
#define ROUGH(a, b)  (fabs((double)(a) - (double)(b)) < 5e-3 * fabs(b) + 1e-6)
/* Table 17 prints 2 decimals, so compare against its rounding, not tighter. */
#define TABLE(a, b)  (fabs((double)(a) - (double)(b)) <= 5e-3 + 1e-9)

/* ---------------- values quoted from the datasheet ---------------- */

static void test_datasheet(void)
{
    /* Table 17, RREF = 12k: the max RMS current of each range. */
    assert(TABLE(tmc5240_range_rms(2, 12.0), 2.12));
    assert(TABLE(tmc5240_range_rms(1, 12.0), 1.41));
    assert(TABLE(tmc5240_range_rms(0, 12.0), 0.69));

    /* Table 17, further down the resistor column. */
    assert(TABLE(tmc5240_range_rms(2, 18.0), 1.41));
    assert(TABLE(tmc5240_range_rms(1, 27.0), 0.63));
    assert(TABLE(tmc5240_range_rms(0, 47.0), 0.18));
    assert(TABLE(tmc5240_range_rms(2, 56.0), 0.45));

    /* Range 3 is the same 3 A as range 2 (Table 16). */
    assert(tmc5240_kifs(3) == tmc5240_kifs(2));

    /* Table 19: v[Hz] = v[TMC5240] * fCLK / 2^24. */
    assert(NEAR(tmc5240_v_to_hz(1, 12.5e6), 12.5e6 / 16777216.0));
    assert(NEAR(tmc5240_hz_to_v(12.5e6 / 16777216.0, 12.5e6), 1.0));

    /* Table 19: a[Hz/s] = a[TMC5240] * fCLK^2 / 2^42. */
    assert(NEAR(tmc5240_a_to_hzs(1, 12.5e6), 12.5e6 * 12.5e6 / 4398046511104.0));
    assert(ROUGH(tmc5240_a_to_hzs(1, 12.5e6), 35.527));

    /* Table 19: rs = v^2 / a / 2^8. */
    assert(NEAR(tmc5240_ramp_usteps(1024.0, 4.0), 1024.0));

    /* Table 19: TSTEP = 2^24 / (v * 256 / USC). */
    assert(NEAR(tmc5240_v_to_tstep(16777216.0, 256), 1.0));
    /* At 256 microsteps the USC term cancels and TSTEP is just 2^24 / v. */
    assert(NEAR(tmc5240_v_to_tstep(137439.0, 256), 16777216.0 / 137439.0));
    /* Halving the microstep resolution halves the step period. */
    assert(NEAR(tmc5240_v_to_tstep(137439.0, 128),
                tmc5240_v_to_tstep(137439.0, 256) / 2.0));

    /* ADC scaling, from the ADC_VSUPPLY_AIN and ADC_TEMP register tables. */
    assert(ROUGH(tmc5240_vsupply_v(2467), 24.01));
    assert(ROUGH(tmc5240_ain_v(1000u << 16), 0.3052));
    assert(NEAR(tmc5240_temp_c(2038), 0.0));
    assert(ROUGH(tmc5240_temp_c(2038 + 77), 10.0));

    /* INP_OUT.VERSION, "0x40 = first version of the IC". */
    /* Guarded here so the regs header and the datasheet cannot drift apart. */
    assert(TMC5240_VERSION_FIRST == 0x40);
}

/* ---------------- a real motor, end to end ---------------- */

static void test_worked_example(void)
{
    /* 200 fullstep/rev motor at 256 microsteps, asked for 2 rev/s.
     * 2 * 200 * 256 = 102400 microsteps/s. */
    double hz = 2.0 * 200.0 * 256.0;
    double v  = tmc5240_hz_to_v(hz, TMC5240_FCLK_INTERNAL);

    assert(tmc5240_fits(v, TMC5240_V_BITS));
    assert(ROUGH(tmc5240_v_to_hz((int32_t)tmc5240_fit(v, TMC5240_V_BITS),
                                 TMC5240_FCLK_INTERNAL), hz));

    /* Reach it in half a second: 204800 microsteps/s^2. */
    double a = tmc5240_hzs_to_a(hz / 0.5, TMC5240_FCLK_INTERNAL);
    assert(tmc5240_fits(a, TMC5240_A_BITS));

    /* And confirm the ramp fits a 1 rev move: 51200 microsteps. */
    assert(tmc5240_ramp_usteps(v, a) < 51200.0);

    /* 0.8 A RMS on a 12k reference: range 0 tops out at 0.69, so range 1. */
    int r = tmc5240_pick_range(0.8, 12.0);
    assert(r == 1);
    double full = tmc5240_range_rms(r, 12.0);
    uint8_t cs = tmc5240_cs(0.8, full);
    /* Quantisation is 1/32 of full scale, so never worse than ~1.5%. */
    assert(fabs(tmc5240_cs_rms(cs, full) - 0.8) <= full / 64.0 + 1e-9);
}

/* ---------------- sign extension ---------------- */

static void test_sext(void)
{
    assert(tmc5240_sext(0xFFFFFF, 24) == -1);
    assert(tmc5240_sext(0x7FFFFF, 24) == 8388607);
    assert(tmc5240_sext(0x800000, 24) == -8388608);
    assert(tmc5240_sext(0, 24) == 0);
    /* VACTUAL arrives inside a 32-bit read with the top byte as status junk. */
    assert(tmc5240_sext(0xAB800000, 24) == -8388608);
}

/* ---------------- field widths ---------------- */

static void test_fit(void)
{
    assert(tmc5240_fits(0.0, TMC5240_V_BITS));
    assert(tmc5240_fits(8388607.0, TMC5240_V_BITS));
    assert(!tmc5240_fits(8388608.0, TMC5240_V_BITS));
    assert(!tmc5240_fits(-1.0, TMC5240_V_BITS));
    assert(!tmc5240_fits(NAN, TMC5240_V_BITS));
    assert(!tmc5240_fits(INFINITY, TMC5240_V_BITS));

    /* Acceleration is 18 bits on this part, not the 16 of a TMC5160. */
    assert(tmc5240_fits(262143.0, TMC5240_A_BITS));
    assert(!tmc5240_fits(262144.0, TMC5240_A_BITS));

    /* Clamping, including the values that would otherwise be undefined. */
    assert(tmc5240_fit(8388608.0, TMC5240_V_BITS) == 8388607);
    assert(tmc5240_fit(NAN, TMC5240_V_BITS) == 0);
    assert(tmc5240_fit(-INFINITY, TMC5240_V_BITS) == 0);
    assert(tmc5240_fit(INFINITY, TMC5240_V_BITS) == 8388607);
    assert(tmc5240_fit(41.4, TMC5240_A_BITS) == 41);
    assert(tmc5240_fit(41.6, TMC5240_A_BITS) == 42);
}

/* ---------------- inputs that should be refused ---------------- */

static void test_bad_inputs(void)
{
    /* RREF outside the datasheet's 12k..60k range is not a current setting. */
    assert(tmc5240_range_rms(2, 11.9) == 0.0);
    assert(tmc5240_range_rms(2, 60.1) == 0.0);
    assert(tmc5240_pick_range(1.0, 100.0) == -1);

    /* No range reaches 5 A on any legal resistor. */
    assert(tmc5240_pick_range(5.0, 12.0) == -1);

    /* A request below one CS step still programs the smallest non-zero
     * current, never "off" -- CS 0 means 1/32 of full scale. */
    assert(tmc5240_cs(1e-9, 2.12) == 0);
    assert(tmc5240_cs_rms(0, 2.12) > 0.0);

    /* Nonsense arguments must not divide by zero into a register. */
    assert(tmc5240_cs(0.5, 0.0) == 0);
    assert(tmc5240_cs(-1.0, 2.12) == 0);
    assert(tmc5240_kifs(-1) == 0.0);
    assert(tmc5240_kifs(4) == 0.0);

    /* A zero clock makes every ramp conversion infinite, which must not
     * survive into a register. */
    assert(!tmc5240_fits(tmc5240_hz_to_v(1000.0, 0.0), TMC5240_V_BITS));
}

/* ---------------- microstep resolution ---------------- */

static void test_mres(void)
{
    assert(tmc5240_mres(256) == 0);
    assert(tmc5240_mres(128) == 1);
    assert(tmc5240_mres(16) == 4);
    assert(tmc5240_mres(1) == 8);
    assert(tmc5240_mres(0) == -1);
    assert(tmc5240_mres(3) == -1);
    assert(tmc5240_mres(512) == -1);

    for (int m = 0; m <= 8; m++)
        assert(tmc5240_mres(tmc5240_usteps(m)) == m);
    assert(tmc5240_usteps(9) == -1);
}

int main(void)
{
    test_datasheet();
    test_worked_example();
    test_sext();
    test_fit();
    test_bad_inputs();
    test_mres();
    printf("all tmc5240 math checks passed\n");
    return 0;
}
