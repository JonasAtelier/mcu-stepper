/*
 * A fake TMC2209 at datagram level, and the driver's cases against it,
 * shared by esp/test_tmc2209.c and stm/test_tmc2209.c. The fake parses what
 * the MCU sends byte by byte -- sync, slave address, register, data, CRC
 * (section 4) -- keeps a register file with the GSTAT and IFCNT behaviour of
 * section 5.1, and answers a read with a reply datagram. Each platform file
 * puts it on its own UART fakes, single wire (echo) or two, and provides:
 *
 *   RET(err)               what the handle API returns for a negative errno
 *   base_cfg()             a config on that platform's UART, ENN tied
 *   cfg_en(cfg)            the same with ENN wired
 *   en_level()             the last level driven on ENN, -1 if never
 *   line_echo              true: the MCU hears what it sends
 *   line_fail              true: the UART driver refuses to send
 *   LOG_PREFIX             what the platform's log puts before "stepper: "
 *   stub_log_last          the last log line
 */

#include "tmc2209_math.h"
#include "tmc2209_regs.h"

/* ------------------------------------------------------- the fake chip --- */

static uint32_t regs[128];
static uint8_t chip_addr;	/* the MS1/MS2 strap */
static uint8_t ifcnt;		/* successful writes, mod 256 */
static bool chip_silent;	/* never answers a read */
static bool chip_bad_crc;	/* answers with a broken CRC */
static bool chip_deaf;		/* answers reads, but writes do not land */
static uint8_t dg[8];		/* the datagram being received */
static size_t dg_len;
static int chip_writes, chip_reads, chip_rejected;
static uint8_t last_write[8];	/* the last whole write datagram */

static void chip_reset(void)
{
	memset(regs, 0, sizeof(regs));
	/* Reset defaults, 5.1 and 5.5.1: I_scale_analog and multistep_filt
	 * set in GCONF; CHOPCONF 0x10000053 (TOFF 3, HSTRT 5, intpol); GSTAT
	 * reset flag set after power-up. */
	regs[TMC2209_GCONF] = 0x00000101;
	regs[TMC2209_CHOPCONF] = 0x10000053;
	regs[TMC2209_GSTAT] = TMC2209_GSTAT_RESET;
	chip_addr = 0;
	ifcnt = 0;
	chip_silent = chip_bad_crc = chip_deaf = false;
	dg_len = 0;
	chip_writes = chip_reads = chip_rejected = 0;
}

/*
 * One byte off the line into the chip. Returns the reply's length, 0 if
 * none, with the reply in out.
 */
static size_t chip_feed(uint8_t byte, uint8_t *out)
{
	size_t want;
	uint8_t reg;

	if (dg_len == 0 && (byte & 0x0F) != TMC2209_SYNC)
		return 0;	/* not a sync nibble: wait for one */
	dg[dg_len++] = byte;
	if (dg_len < 3)
		return 0;
	want = (dg[2] & TMC2209_WRITE) ? TMC2209_WRITE_LEN : TMC2209_READ_REQ_LEN;
	if (dg_len < want)
		return 0;
	dg_len = 0;

	if (dg[1] != chip_addr || dg[want - 1] != tmc2209_crc8(dg, want - 1)) {
		chip_rejected++;
		return 0;
	}
	reg = dg[2] & 0x7F;

	if (want == TMC2209_WRITE_LEN) {
		uint32_t v = ((uint32_t)dg[3] << 24) | ((uint32_t)dg[4] << 16) |
			     ((uint32_t)dg[5] << 8) | dg[6];

		memcpy(last_write, dg, sizeof(last_write));
		chip_writes++;
		if (chip_deaf)
			return 0;
		if (reg == TMC2209_GSTAT)	/* write 1 to clear; uv_cp is not latched */
			regs[reg] &= ~(v & (TMC2209_GSTAT_RESET | TMC2209_GSTAT_DRV_ERR));
		else
			regs[reg] = v;
		ifcnt++;
		return 0;
	}

	chip_reads++;
	if (chip_silent)
		return 0;
	{
		uint32_t v = reg == TMC2209_IFCNT ? ifcnt : regs[reg];

		out[0] = TMC2209_SYNC;
		out[1] = TMC2209_MASTER_ADDR;
		out[2] = reg;
		out[3] = (uint8_t)(v >> 24);
		out[4] = (uint8_t)(v >> 16);
		out[5] = (uint8_t)(v >> 8);
		out[6] = (uint8_t)v;
		out[7] = tmc2209_crc8(out, 7);
		if (chip_bad_crc)
			out[7] ^= 0x01;
	}
	return TMC2209_REPLY_LEN;
}

/* ------------------------------------------------------ allocations ------ */

void *__real_calloc(size_t n, size_t size);
void __real_free(void *p);
void *__wrap_calloc(size_t n, size_t size);
void __wrap_free(void *p);

static int allocs, frees;
static bool fail_calloc;

void *__wrap_calloc(size_t n, size_t size)
{
	if (fail_calloc)
		return NULL;
	allocs++;
	return __real_calloc(n, size);
}

void __wrap_free(void *p)
{
	if (p)
		frees++;
	__real_free(p);
}

/* ------------------------------------------------------------ helpers ---- */

#define CFG_RSENSE	110.0f

static tmc2209_handle_t open_ok(tmc2209_config_t *c)
{
	tmc2209_handle_t h = NULL;

	assert(tmc2209_init(c, &h) == RET(0));
	assert(h);
	return h;
}

/* ------------------------------------------------------------- cases ---- */

static void test_init(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	struct stepper_dev *s;
	uint32_t chop;

	chip_reset();
	line_echo = true;
	c.run_current_ma = 800;
	c.interpolate = true;
	h = open_ok(&c);

	/* GCONF, read-modify-write: pdn_disable and mstep_reg_select on,
	 * I_scale_analog off, the reset default multistep_filt kept. */
	assert(regs[TMC2209_GCONF] == (TMC2209_GCONF_PDN_DISABLE |
				       TMC2209_GCONF_MSTEP_REG_SELECT | 0x100));
	/* CHOPCONF: MRES 4 for the default 16 microsteps, intpol, TOFF 3;
	 * HSTRT, outside the mask, kept. */
	chop = regs[TMC2209_CHOPCONF];
	assert(((chop & TMC2209_CHOP_MRES_MASK) >> TMC2209_CHOP_MRES_SHIFT) == 4);
	assert((chop & TMC2209_CHOP_INTPOL) && (chop & TMC2209_CHOP_TOFF_MASK) == 3);
	assert((chop & 0x70) == 0x50);
	assert(regs[TMC2209_TPOWERDOWN] == 20);
	/* SENDDELAY 2: bits 11..8. */
	assert(regs[TMC2209_SLAVECONF] == 0x200);
	/* IHOLDDELAY 10 over the two current scales. */
	assert((regs[TMC2209_IHOLD_IRUN] >> TMC2209_IHOLDDELAY_SHIFT) == 10);

	/* The stage stays off: init drives ENN high and nothing more. */
	tmc2209_del(h);
	chip_reset();
	cfg_en(&c);
	h = open_ok(&c);
	assert(en_level() == 1);

	/* The category view. */
	s = tmc2209_stepper(h);
	assert(s && !strcmp(s->name, "tmc2209") && s->ops && s->priv == h);
	assert(s->microsteps == 16);
	assert(tmc2209_stepper(NULL) == NULL);
	tmc2209_del(h);
	assert(en_level() == 1);
	assert(allocs == frees);
}

static void test_options(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;

	chip_reset();
	line_echo = true;
	c.spreadcycle = true;
	c.invert_direction = true;
	c.microsteps = 256;
	h = open_ok(&c);
	assert(regs[TMC2209_GCONF] & TMC2209_GCONF_EN_SPREADCYCLE);
	assert(regs[TMC2209_GCONF] & TMC2209_GCONF_SHAFT);
	assert(!(regs[TMC2209_GCONF] & TMC2209_GCONF_I_SCALE_ANALOG));
	assert((regs[TMC2209_CHOPCONF] & TMC2209_CHOP_MRES_MASK) == 0);
	assert(!(regs[TMC2209_CHOPCONF] & TMC2209_CHOP_INTPOL));
	assert(tmc2209_stepper(h)->microsteps == 256);
	tmc2209_del(h);

	/* A chip strapped to address 2 is the one addressed. */
	chip_reset();
	chip_addr = 2;
	c = base_cfg();
	c.address = 2;
	h = open_ok(&c);
	assert(last_write[1] == 2 && chip_rejected == 0);
	tmc2209_del(h);
}

static void test_two_wire(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	uint32_t v;

	/* Separate TX and RX nets: no echo to take off the line. */
	chip_reset();
	line_echo = false;
	c.two_wire = true;
	h = open_ok(&c);
	regs[0x41] = 0x123;
	assert(tmc2209_read(h, 0x41, &v) == RET(0) && v == 0x123);
	tmc2209_del(h);
	line_echo = true;
}

static void test_datagrams(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	uint32_t v;

	chip_reset();
	line_echo = true;
	h = open_ok(&c);

	/* A write: sync, address, register with bit 7, data high to low, CRC
	 * over the seven before it. */
	assert(tmc2209_write(h, 0x22, 0x00123456) == RET(0));
	assert(last_write[0] == 0x05 && last_write[1] == 0 &&
	       last_write[2] == 0xA2 && last_write[3] == 0x00 &&
	       last_write[4] == 0x12 && last_write[5] == 0x34 &&
	       last_write[6] == 0x56);
	assert(last_write[7] == tmc2209_crc8(last_write, 7));

	/* A read comes back whole, after the echo of the request. */
	regs[0x6A] = 0xCAFEF00D;
	assert(tmc2209_read(h, 0x6A, &v) == RET(0) && v == 0xCAFEF00D);
	/* Back to back, never one stale. */
	regs[0x6B] = 0x11;
	assert(tmc2209_read(h, 0x6B, &v) == RET(0) && v == 0x11);
	assert(tmc2209_read(h, 0x6A, &v) == RET(0) && v == 0xCAFEF00D);

	/* A reply that does not check out is refused, *value left alone. */
	chip_bad_crc = true;
	v = 7;
	assert(tmc2209_read(h, 0x6A, &v) == RET(-EBADMSG) && v == 7);
	chip_bad_crc = false;
	/* No reply is a timeout. */
	chip_silent = true;
	assert(tmc2209_read(h, 0x6A, &v) == RET(-ETIMEDOUT));
	chip_silent = false;
	/* The line refusing to send is an I/O error. */
	line_fail = true;
	assert(tmc2209_write(h, 0x22, 0) == RET(-EIO));
	line_fail = false;
	assert(tmc2209_read(h, 0x6A, &v) == RET(0));
	assert(chip_rejected == 0);
	tmc2209_del(h);
}

static void test_init_failures(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h = NULL;

	/* Nothing answers. ENN, if wired, was set high first and stays so. */
	chip_reset();
	chip_silent = true;
	cfg_en(&c);
	allocs = frees = 0;
	assert(tmc2209_init(&c, &h) == RET(-ETIMEDOUT));
	assert(!h && allocs == 1 && frees == 1 && en_level() == 1);

	/* Replies that never check out. */
	chip_reset();
	chip_bad_crc = true;
	assert(tmc2209_init(&c, &h) == RET(-EBADMSG));

	/* Replies come but writes do not land: IFCNT stands still. */
	chip_reset();
	chip_deaf = true;
	assert(tmc2209_init(&c, &h) == RET(-ENODEV));
	assert(strstr(stub_log_last, "stepper: tmc2209: IFCNT did not advance"));

	/* IFCNT wraps 255 to 0; that is still one write. */
	chip_reset();
	ifcnt = 255;
	h = open_ok(&c);
	tmc2209_del(h);
	assert(allocs == frees);
}

static void test_bad_config(void)
{
	tmc2209_config_t c;
	tmc2209_handle_t h = NULL;

	chip_reset();
	allocs = frees = 0;

	c = base_cfg(); c.address = 4;
	assert(tmc2209_init(&c, &h) == RET(-EINVAL));
	c = base_cfg(); c.rsense_mohm = 0.0f;
	assert(tmc2209_init(&c, &h) == RET(-EINVAL));
	c = base_cfg(); c.microsteps = 3;
	assert(tmc2209_init(&c, &h) == RET(-EINVAL));
	assert(tmc2209_init(NULL, &h) == RET(-EINVAL));
	c = base_cfg();
	assert(tmc2209_init(&c, NULL) == RET(-EINVAL));
	assert(!h && allocs == frees && chip_writes == 0 && chip_reads == 0);

	fail_calloc = true;
	assert(tmc2209_init(&c, &h) == RET(-ENOMEM));
	fail_calloc = false;
}

static void test_current(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	bool vsense, hold_vsense;
	uint8_t irun, ihold;

	chip_reset();
	line_echo = true;
	c.run_current_ma = 1200;
	h = open_ok(&c);

	/* 1.2 A on 110 mOhm needs the high range; hold defaults to half,
	 * which alone would take the low one, so it is scaled onto the
	 * high. */
	irun = tmc2209_current_to_cs(CFG_RSENSE, 1200, &vsense);
	ihold = tmc2209_current_to_cs(CFG_RSENSE, 600, &hold_vsense);
	assert(hold_vsense);
	ihold = tmc2209_cs_on_range(ihold, hold_vsense, vsense);
	assert(!vsense);
	assert(((regs[TMC2209_IHOLD_IRUN] >> TMC2209_IRUN_SHIFT) & 0x1F) == irun);
	assert(((regs[TMC2209_IHOLD_IRUN] >> TMC2209_IHOLD_SHIFT) & 0x1F) == ihold);
	assert(!(regs[TMC2209_CHOPCONF] & TMC2209_CHOP_VSENSE));

	/* A small current takes the low range, and says so in CHOPCONF. */
	assert(tmc2209_set_current(h, 200, 100) == RET(0));
	irun = tmc2209_current_to_cs(CFG_RSENSE, 200, &vsense);
	assert(vsense && (regs[TMC2209_CHOPCONF] & TMC2209_CHOP_VSENSE));
	assert(((regs[TMC2209_IHOLD_IRUN] >> TMC2209_IRUN_SHIFT) & 0x1F) == irun);
	assert(strstr(stub_log_last, "stepper: tmc2209: run "));

	/* Through the category: amperes, the same registers. */
	assert(stepper_set_current(tmc2209_stepper(h), 1.2f, 0.0f) == 0);
	irun = tmc2209_current_to_cs(CFG_RSENSE, 1200, &vsense);
	assert(((regs[TMC2209_IHOLD_IRUN] >> TMC2209_IRUN_SHIFT) & 0x1F) == irun);
	assert(!(regs[TMC2209_CHOPCONF] & TMC2209_CHOP_VSENSE));
	assert(stepper_set_current(tmc2209_stepper(h), 0.0f, 0.0f) == -EINVAL);
	assert(stepper_set_current(tmc2209_stepper(h), -1.0f, 0.0f) == -EINVAL);
	assert(stepper_set_current(tmc2209_stepper(h), NAN, 0.0f) == -EINVAL);
	assert(stepper_set_current(tmc2209_stepper(h), 70.0f, 0.0f) == -EINVAL);
	assert(stepper_set_current(tmc2209_stepper(h), 1.0f, -0.1f) == -EINVAL);
	assert(stepper_set_current(tmc2209_stepper(h), 0.0001f, 0.0f) ==
	       -EINVAL);

	assert(tmc2209_set_current(h, 0, 0) == RET(-EINVAL));
	tmc2209_del(h);
}

static void test_move(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;

	chip_reset();
	line_echo = true;
	h = open_ok(&c);

	/* VACTUAL = v * 2^24 / fCLK, at the 12 MHz default. */
	assert(tmc2209_move(h, 1000.0f) == RET(0));
	assert(regs[TMC2209_VACTUAL] == 1398);
	/* 24-bit two's complement the other way. */
	assert(tmc2209_move(h, -1000.0f) == RET(0));
	assert(regs[TMC2209_VACTUAL] == (0x01000000u - 1398));
	/* Past +-(2^23)-1 is refused, VACTUAL untouched. */
	assert(tmc2209_move(h, 1e7f) == RET(-EINVAL));
	assert(regs[TMC2209_VACTUAL] == (0x01000000u - 1398));
	assert(tmc2209_stop(h) == RET(0) && regs[TMC2209_VACTUAL] == 0);
	tmc2209_del(h);

	/* An external clock scales it. */
	chip_reset();
	c.clock_hz = 16000000;
	h = open_ok(&c);
	assert(tmc2209_move(h, 1000.0f) == RET(0));
	assert(regs[TMC2209_VACTUAL] == 1049);
	tmc2209_del(h);
}

static void test_enable(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;

	/* Tied: nothing to drive. */
	chip_reset();
	line_echo = true;
	h = open_ok(&c);
	assert(tmc2209_enable(h, true) == RET(-EOPNOTSUPP));
	assert(stepper_enable(tmc2209_stepper(h), true) == -EOPNOTSUPP);
	tmc2209_del(h);

	/* Wired: ENN is active low. */
	chip_reset();
	cfg_en(&c);
	h = open_ok(&c);
	assert(tmc2209_enable(h, true) == RET(0) && en_level() == 0);
	assert(tmc2209_enable(h, false) == RET(0) && en_level() == 1);
	assert(stepper_enable(tmc2209_stepper(h), true) == 0 &&
	       en_level() == 0);
	/* del leaves the motor free. */
	tmc2209_del(h);
	assert(en_level() == 1);
}

static void test_status(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	tmc2209_status_t st;
	struct stepper_status s;

	chip_reset();
	line_echo = true;
	h = open_ok(&c);
	regs[TMC2209_DRV_STATUS] = TMC2209_DRV_STST | TMC2209_DRV_OTPW |
				   TMC2209_DRV_S2VSB | TMC2209_DRV_OLA |
				   (17u << TMC2209_DRV_CS_ACTUAL_SHIFT);
	regs[TMC2209_GSTAT] = TMC2209_GSTAT_RESET | TMC2209_GSTAT_UV_CP;

	assert(tmc2209_read_status(h, &st) == RET(0));
	assert(st.standstill && st.overtemp_warning && st.short_low_side &&
	       st.open_load && st.undervoltage_cp && st.was_reset);
	assert(!st.stealthchop && !st.overtemp && !st.short_to_gnd);
	assert(st.current_scale == 17 && st.drv_status == regs[TMC2209_DRV_STATUS]);
	/* The reset flag was written back and is gone; uv_cp is not latched. */
	assert(regs[TMC2209_GSTAT] == TMC2209_GSTAT_UV_CP);
	assert(tmc2209_read_status(h, &st) == RET(0) && !st.was_reset);

	/* Through the category. No stall flag: that is the DIAG pin. */
	regs[TMC2209_DRV_STATUS] = TMC2209_DRV_STEALTH | TMC2209_DRV_S2GA |
				   TMC2209_DRV_OT |
				   (31u << TMC2209_DRV_CS_ACTUAL_SHIFT);
	regs[TMC2209_GSTAT] = TMC2209_GSTAT_RESET;
	assert(stepper_read_status(tmc2209_stepper(h), &s) == 0);
	assert(s.stealthchop && s.short_to_gnd && s.overtemp && s.was_reset);
	assert(!s.standstill && !s.short_to_supply && !s.open_load &&
	       !s.stall && !s.overtemp_warning && s.current_scale == 31);
	regs[TMC2209_DRV_STATUS] = TMC2209_DRV_S2VSA;
	assert(stepper_read_status(tmc2209_stepper(h), &s) == 0);
	assert(s.short_to_supply && !s.was_reset);

	/* A failed read leaves the status alone. */
	chip_silent = true;
	s.current_scale = 99;
	assert(stepper_read_status(tmc2209_stepper(h), &s) == -ETIMEDOUT);
	assert(s.current_scale == 99);
	chip_silent = false;
	tmc2209_del(h);
}

static void test_stall(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	uint16_t load;

	chip_reset();
	line_echo = true;
	h = open_ok(&c);

	/* SG_RESULT is ten bits. */
	regs[TMC2209_SG_RESULT] = 0xFFFFF000u | 0x2A5;
	assert(tmc2209_stallguard(h, &load) == RET(0) && load == 0x2A5);

	/* TCOOLTHRS: fCLK / (v * 2^MRES) at 16 microsteps (MRES 4). */
	assert(tmc2209_set_stall(h, 40, 1000.0f) == RET(0));
	assert(regs[TMC2209_SGTHRS] == 40);
	assert(regs[TMC2209_TCOOLTHRS] == 12000000u / (1000u * 16u));
	/* Slower than the register can say: clamped to its 20 bits. */
	assert(tmc2209_set_stall(h, 40, 0.5f) == RET(0));
	assert(regs[TMC2209_TCOOLTHRS] == 0xFFFFF);
	/* Threshold 0 disarms, and zeroes the window too. */
	assert(tmc2209_set_stall(h, 0, 1000.0f) == RET(0));
	assert(regs[TMC2209_SGTHRS] == 0 && regs[TMC2209_TCOOLTHRS] == 0);
	assert(tmc2209_set_stall(h, 40, 0.0f) == RET(-EINVAL));
	tmc2209_del(h);
}

static void test_null_arguments(void)
{
	tmc2209_config_t c = base_cfg();
	tmc2209_handle_t h;
	tmc2209_status_t st;
	uint32_t v;
	uint16_t u;

	assert(tmc2209_enable(NULL, true) == RET(-EINVAL));
	assert(tmc2209_set_current(NULL, 500, 0) == RET(-EINVAL));
	assert(tmc2209_move(NULL, 0.0f) == RET(-EINVAL));
	assert(tmc2209_stop(NULL) == RET(-EINVAL));
	assert(tmc2209_read_status(NULL, &st) == RET(-EINVAL));
	assert(tmc2209_stallguard(NULL, &u) == RET(-EINVAL));
	assert(tmc2209_set_stall(NULL, 1, 1.0f) == RET(-EINVAL));
	assert(tmc2209_read(NULL, 0, &v) == RET(-EINVAL));
	assert(tmc2209_write(NULL, 0, 0) == RET(-EINVAL));
	assert(tmc2209_del(NULL) == RET(-EINVAL));

	chip_reset();
	line_echo = true;
	h = open_ok(&c);
	assert(tmc2209_read_status(h, NULL) == RET(-EINVAL));
	assert(tmc2209_stallguard(h, NULL) == RET(-EINVAL));
	assert(tmc2209_read(h, 0, NULL) == RET(-EINVAL));
	tmc2209_del(h);
}

static void run_tmc2209_cases(void)
{
	test_init();
	test_options();
	test_two_wire();
	test_datagrams();
	test_init_failures();
	test_bad_config();
	test_current();
	test_move();
	test_enable();
	test_status();
	test_stall();
	test_null_arguments();
}
