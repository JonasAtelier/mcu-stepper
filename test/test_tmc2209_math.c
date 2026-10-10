/*
 * Host-side check of src/tmc2209/tmc2209_math.h: the CRC against an
 * independent formulation, the current formula of section 9 against
 * numbers worked by hand, and the edges where a value stops fitting its
 * register. esp-tmc2209-stepper had no tests.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../src/tmc2209/tmc2209_math.h"

#define NEAR(a, b, tol)	(fabs((double)(a) - (double)(b)) <= (tol))

/* The same CRC said the textbook way: reflect each byte, then a plain
 * MSB-first CRC-8 with polynomial 0x07 and init 0. */
static uint8_t crc_reference(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		uint8_t b = data[i], r = 0;

		for (int k = 0; k < 8; k++)
			r |= (uint8_t)(((b >> k) & 1) << (7 - k));
		crc ^= r;
		for (int k = 0; k < 8; k++)
			crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07
						     : crc << 1);
	}
	return crc;
}

static void test_crc(void)
{
	uint8_t dg[7] = { 0x05, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 };
	uint32_t seed = 1;

	assert(tmc2209_crc8(dg, 0) == 0);
	assert(tmc2209_crc8(dg, 7) == crc_reference(dg, 7));

	/* Many datagrams, every byte position exercised. */
	for (int n = 0; n < 10000; n++) {
		for (int i = 0; i < 7; i++) {
			seed = seed * 1103515245u + 12345u;
			dg[i] = (uint8_t)(seed >> 16);
		}
		assert(tmc2209_crc8(dg, 7) == crc_reference(dg, 7));
		assert(tmc2209_crc8(dg, 3) == crc_reference(dg, 3));
	}

	/* One flipped bit changes the CRC. */
	{
		uint8_t a[8] = { 0x05, 0x01, 0xA2, 0x00, 0x00, 0x05, 0xDC, 0 };
		uint8_t c;

		a[7] = tmc2209_crc8(a, 7);
		c = a[7];
		a[4] ^= 0x01;
		assert(tmc2209_crc8(a, 7) != c);
	}
}

/* I_RMS = (CS + 1) / 32 * VFS / (Rsense + 20 mOhm) / sqrt(2). */
static double irms_ma(uint8_t cs, bool vsense, double rsense_mohm)
{
	double vfs = vsense ? 180.0 : 325.0;

	return (cs + 1) / 32.0 * vfs / (rsense_mohm + 20.0) / sqrt(2.0) *
	       1000.0;
}

static void test_current(void)
{
	bool vsense;
	uint8_t cs;

	/* Full scale on the high range, 110 mOhm: 32/32 * 325 / 130 / sqrt 2
	 * = 1.768 A. */
	cs = tmc2209_current_to_cs(110.0f, 1768, &vsense);
	assert(cs == 31 && !vsense);
	assert(NEAR(tmc2209_cs_to_current(110.0f, 31, false), 1767, 1));

	/* Under half scale takes the low range, for resolution. */
	cs = tmc2209_current_to_cs(110.0f, 500, &vsense);
	assert(vsense);
	assert(NEAR(irms_ma(cs, true, 110.0), 500, irms_ma(0, true, 110.0)));
	assert(NEAR(tmc2209_cs_to_current(110.0f, cs, true),
		    irms_ma(cs, true, 110.0), 1));

	/* Every current on the way up lands within half a step. */
	for (unsigned ma = 100; ma <= 1700; ma += 7) {
		double step;

		cs = tmc2209_current_to_cs(110.0f, (uint16_t)ma, &vsense);
		step = irms_ma(0, vsense, 110.0);
		assert(NEAR(irms_ma(cs, vsense, 110.0), ma, step / 2 + 1));
	}

	/* Past full scale clamps to 31; nothing at all to 0, not off. */
	assert(tmc2209_current_to_cs(110.0f, 5000, &vsense) == 31 && !vsense);
	assert(tmc2209_current_to_cs(110.0f, 0, &vsense) == 0 && vsense);

	/* The hold current moved onto the run current's range keeps its
	 * current within a step. */
	cs = tmc2209_current_to_cs(110.0f, 600, &vsense);
	assert(vsense);
	{
		uint8_t high = tmc2209_cs_on_range(cs, true, false);

		assert(NEAR(irms_ma(high, false, 110.0), irms_ma(cs, true, 110.0),
			    irms_ma(0, false, 110.0)));
		assert(tmc2209_cs_on_range(cs, true, true) == cs);
		assert(tmc2209_cs_on_range(31, false, true) == 31);
		assert(tmc2209_cs_on_range(0, true, false) == 0);
	}
}

static void test_mres(void)
{
	static const uint16_t steps[] = { 256, 128, 64, 32, 16, 8, 4, 2, 1 };

	for (int m = 0; m <= 8; m++)
		assert(tmc2209_mres(steps[m]) == m);
	assert(tmc2209_mres(0) == -1 && tmc2209_mres(3) == -1 &&
	       tmc2209_mres(512) == -1);
}

static void test_vactual(void)
{
	int32_t v;

	/* v * 2^24 / fCLK. */
	assert(tmc2209_vactual(1000.0f, 12000000, &v) && v == 1398);
	assert(tmc2209_vactual(-1000.0f, 12000000, &v) && v == -1398);
	assert(tmc2209_vactual(0.0f, 12000000, &v) && v == 0);

	/* +-(2^23)-1 at 12 MHz is about 5.96 million microsteps/s. */
	assert(tmc2209_vactual(5.9e6f, 12000000, &v) && v > 8200000);
	v = 7;
	assert(!tmc2209_vactual(6.0e6f, 12000000, &v) && v == 7);
	assert(!tmc2209_vactual(-6.0e6f, 12000000, &v) && v == 7);
}

static void test_tcoolthrs(void)
{
	/* fCLK / (v * 2^MRES): a period in 1/256 microsteps. */
	assert(tmc2209_tcoolthrs(12000000, 1000.0f, 4) == 750);
	assert(tmc2209_tcoolthrs(12000000, 1000.0f, 0) == 12000);
	/* Faster is smaller. */
	assert(tmc2209_tcoolthrs(12000000, 2000.0f, 4) == 375);
	/* Slower than 20 bits can say clamps. */
	assert(tmc2209_tcoolthrs(12000000, 0.5f, 4) == 0xFFFFF);
	/* Clamped before the cast: a period of 7.5e8, one past 2^32 (7.5e11,
	 * where the cast was undefined), and NaN all give the 20-bit maximum. */
	assert(tmc2209_tcoolthrs(12000000, 0.001f, 4) == 0xFFFFF);
	assert(tmc2209_tcoolthrs(12000000, 1e-6f, 4) == 0xFFFFF);
	assert(tmc2209_tcoolthrs(12000000, NAN, 4) == 0xFFFFF);
	/* Right at the edge: just under the maximum truncates, at it clamps. */
	assert(tmc2209_tcoolthrs(1048575, 1.0f, 0) == 0xFFFFF);
	assert(tmc2209_tcoolthrs(1048574, 1.0f, 0) == 1048574);
}

int main(void)
{
	test_crc();
	test_current();
	test_mres();
	test_vactual();
	test_tcoolthrs();
	printf("all tmc2209 math checks passed\n");
	return 0;
}
