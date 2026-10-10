/*
 * A fake TMC5240 at frame level, and this library's cases against it,
 * shared by esp/test_tmc5240_shared.c and stm/test_tmc5240.c. The fake
 * answers each 40-bit frame with SPI_STATUS and the data owed for the
 * *previous* frame's address, as the chip does (Table 3 and the read
 * pipeline above it), and models a ramp that coasts to a stop. Each
 * platform file feeds it from its own SPI calls and provides:
 *
 *   RET(err)               what the handle API returns for a negative errno
 *   base_cfg()             a config on that platform's SPI, DRV_ENN tied
 *   cfg_en(cfg)            the same with DRV_ENN wired
 *   en_level()             the last level driven on DRV_ENN, -1 if never
 *   bus_fail               true: the SPI transfer fails with a timeout
 *
 * esp-tmc5240-stepper's own test, esp/test_tmc5240.c, covers the driver's
 * motion and its lock on ESP-IDF; these cases cover what this library adds
 * -- the stepper_dev view, the 2^42 acceleration, the errno codes -- and
 * the same driver on the HAL.
 */

#include "tmc5240_math.h"
#include "tmc5240_regs.h"

/* ------------------------------------------------------- the fake chip --- */

static uint32_t regs[128];
static int pending_read = -1;	/* address whose data is owed next time */
static uint32_t pending_echo;
static bool pending_is_write;
static uint8_t fake_status;
static int frames;
static int coast_reads;		/* RAMPSTAT polls until a stopping ramp is
				 * still; 0 never */

static void chip_reset(void)
{
	memset(regs, 0, sizeof(regs));
	regs[TMC5240_INP_OUT] = (uint32_t)TMC5240_VERSION_FIRST <<
				TMC5240_VERSION_SHIFT;
	regs[TMC5240_RAMPSTAT] = TMC5240_VZERO;
	regs[TMC5240_GSTAT] = TMC5240_RESET_FLAG;	/* after power-up */
	pending_read = -1;
	pending_is_write = false;
	fake_status = 0;
	frames = 0;
	coast_reads = 0;
}

static void chip_frame(const uint8_t *tx, uint8_t *rx)
{
	uint8_t addr = tx[0] & 0x7F;
	bool write = (tx[0] & TMC5240_WRITE) != 0;
	uint32_t data = ((uint32_t)tx[1] << 24) | ((uint32_t)tx[2] << 16) |
			((uint32_t)tx[3] << 8) | tx[4];
	uint32_t reply;

	frames++;
	if (pending_is_write)
		reply = pending_echo;
	else if (pending_read >= 0)
		reply = regs[pending_read];
	else
		reply = 0xDEADBEEF;

	if (write) {
		if (addr == TMC5240_GSTAT)	/* write 1 to clear, p.92 */
			regs[addr] &= ~data;
		else
			regs[addr] = data;
		pending_is_write = true;
		pending_echo = data;
		pending_read = -1;
	} else {
		pending_is_write = false;
		pending_read = addr;
		if (addr == TMC5240_RAMPSTAT && regs[TMC5240_VMAX] == 0 &&
		    regs[TMC5240_RAMPMODE] == TMC5240_MODE_VELPOS &&
		    coast_reads > 0 && --coast_reads == 0) {
			regs[TMC5240_VACTUAL] = 0;
			regs[TMC5240_RAMPSTAT] |= TMC5240_VZERO;
		}
	}

	rx[0] = fake_status;
	rx[1] = (uint8_t)(reply >> 24);
	rx[2] = (uint8_t)(reply >> 16);
	rx[3] = (uint8_t)(reply >> 8);
	rx[4] = (uint8_t)reply;
}

/* ------------------------------------------------------ allocations ------ */

void *__real_calloc(size_t n, size_t size);
void __real_free(void *p);
void *__wrap_calloc(size_t n, size_t size);
void __wrap_free(void *p);

static int allocs, frees;

void *__wrap_calloc(size_t n, size_t size)
{
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

static tmc5240_handle_t open_ok(tmc5240_config_t *c)
{
	tmc5240_handle_t h = NULL;

	assert(tmc5240_init(c, &h) == RET(0));
	assert(h);
	return h;
}

/* ------------------------------------------------------------- cases ---- */

static void test_init(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;
	struct stepper_dev *s;
	uint32_t v;

	chip_reset();
	allocs = frees = 0;
	h = open_ok(&c);
	assert(regs[TMC5240_RAMPMODE] == TMC5240_MODE_HOLD);
	assert(((regs[TMC5240_CHOPCONF] >> TMC5240_MRES_SHIFT) & 0xF) == 0);

	/* One read is two frames: the request, then the answer. */
	regs[TMC5240_MSCNT] = 0x1234;
	frames = 0;
	assert(tmc5240_read(h, TMC5240_MSCNT, &v) == RET(0) && v == 0x1234);
	assert(frames == 2);

	/* The category view. */
	s = tmc5240_stepper(h);
	assert(s && !strcmp(s->name, "tmc5240") && s->ops && s->priv == h);
	assert(s->microsteps == 256);
	assert(tmc5240_stepper(NULL) == NULL);
	tmc5240_del(h);
	assert(allocs == frees);

	/* DRV_ENN wired: held high through init, low once configured, high
	 * again at del. */
	chip_reset();
	cfg_en(&c);
	h = open_ok(&c);
	assert(en_level() == 0);
	tmc5240_del(h);
	assert(en_level() == 1);
}

static void test_init_failures(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h = NULL;

	/* Something answers, but not VERSION 0x40. Nothing leaks. */
	chip_reset();
	regs[TMC5240_INP_OUT] = 0x41u << TMC5240_VERSION_SHIFT;
	allocs = frees = 0;
	assert(tmc5240_init(&c, &h) == RET(-ENODEV));
	assert(!h && allocs == 1 && frees == 1);

	/* The bus failing. */
	chip_reset();
	bus_fail = true;
	assert(tmc5240_init(&c, &h) == RET(-ETIMEDOUT));
	bus_fail = false;

	/* A config refused before anything is allocated. */
	allocs = frees = 0;
	c.microsteps = 3;
	assert(tmc5240_init(&c, &h) == RET(-EINVAL));
	c = base_cfg();
	c.rref_kohm = 5.0f;
	assert(tmc5240_init(&c, &h) == RET(-EINVAL));
	c = base_cfg();
	assert(tmc5240_init(NULL, &h) == RET(-EINVAL));
	assert(tmc5240_init(&c, NULL) == RET(-EINVAL));
	assert(allocs == 0 && !h);

	/* A current past every range is refused after the probe; nothing
	 * leaks. */
	chip_reset();
	c.run_current_a = 5.0f;
	assert(tmc5240_init(&c, &h) == RET(-EINVAL));
	assert(allocs == frees);
}

static void test_init_stops_turning_motor(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h = NULL;

	/* An MCU reset does not reset the chip; init ramps it down first. */
	chip_reset();
	cfg_en(&c);
	regs[TMC5240_VACTUAL] = 100000;
	regs[TMC5240_VMAX] = 100000;
	regs[TMC5240_RAMPMODE] = TMC5240_MODE_VELPOS;
	regs[TMC5240_RAMPSTAT] = 0;
	coast_reads = 3;
	h = open_ok(&c);
	assert(regs[TMC5240_VACTUAL] == 0 && en_level() == 0);
	tmc5240_del(h);

	/* A ramp that never stops: a timeout, with the stage left off. */
	chip_reset();
	regs[TMC5240_VACTUAL] = 100000;
	regs[TMC5240_RAMPMODE] = TMC5240_MODE_VELPOS;
	regs[TMC5240_RAMPSTAT] = 0;
	assert(tmc5240_init(&c, &h) == RET(-ETIMEDOUT));
	assert(en_level() == 1);
}

/*
 * The one intended change from esp-tmc5240-stepper: Table 19 divides by
 * 2^42, not 2^41. Literal numbers, so a conversion that drifts back cannot
 * agree with itself.
 */
static void test_acceleration_scale(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;
	tmc5240_ramp_t r = { .vmax = 51200.0f, .amax = 51200.0f,
			     .dmax = 25600.0f };

	chip_reset();
	h = open_ok(&c);
	assert(tmc5240_set_ramp(h, &r) == RET(0));
	/* 51200 * 2^42 / 12.5e6^2 = 1441.2; 2^41 would give 721. */
	assert(regs[TMC5240_AMAX] == 1441);
	assert(regs[TMC5240_DMAX] == 721);
	/* And back: one AMAX count is 12.5e6^2 / 2^42 = 35.53 Hz/s. */
	assert(fabs(tmc5240_a_to_hzs(1, 12.5e6) - 35.527) < 0.001);
	tmc5240_del(h);
}

static void test_enable(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;

	chip_reset();
	h = open_ok(&c);
	assert(stepper_enable(tmc5240_stepper(h), true) == -EOPNOTSUPP);
	tmc5240_del(h);

	chip_reset();
	cfg_en(&c);
	h = open_ok(&c);
	assert(stepper_enable(tmc5240_stepper(h), false) == 0 &&
	       en_level() == 1);
	assert(stepper_enable(tmc5240_stepper(h), true) == 0 &&
	       en_level() == 0);
	tmc5240_del(h);
}

static void test_current(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;
	double full;

	chip_reset();
	h = open_ok(&c);

	/* Through the category: the same registers tmc5240_set_current()
	 * writes. 0.5 A on 12k fits range 0. */
	assert(stepper_set_current(tmc5240_stepper(h), 0.5f, 0.0f) == 0);
	full = tmc5240_range_rms(0, 12.0);
	assert(regs[TMC5240_DRV_CONF] == 0);
	assert(((regs[TMC5240_IHOLD_IRUN] >> TMC5240_IRUN_SHIFT) & 0x1F) ==
	       tmc5240_cs(0.5, full));
	assert(((regs[TMC5240_IHOLD_IRUN] >> TMC5240_IHOLD_SHIFT) & 0x1F) ==
	       tmc5240_cs(0.25, full));
	assert(stepper_set_current(tmc5240_stepper(h), 5.0f, 0.0f) == -EINVAL);
	assert(stepper_set_current(tmc5240_stepper(h), NAN, 0.0f) == -EINVAL);
	tmc5240_del(h);
}

static void test_status(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;
	struct stepper_status s;

	chip_reset();
	h = open_ok(&c);

	/* GSTAT still holds the power-up reset: reported once, cleared by
	 * writing it back. */
	regs[TMC5240_DRVSTATUS] = TMC5240_STST | TMC5240_STALLGUARD |
				  TMC5240_S2VSB | TMC5240_OLA |
				  TMC5240_OTPW | (9u << TMC5240_CS_ACTUAL_SHIFT);
	frames = 0;
	assert(stepper_read_status(tmc5240_stepper(h), &s) == 0);
	assert(s.standstill && s.stall && s.short_to_supply && s.open_load &&
	       s.overtemp_warning && s.was_reset && s.current_scale == 9);
	assert(!s.stealthchop && !s.overtemp && !s.short_to_gnd);
	/* DRV_STATUS and GSTAT in one chain, then the write-back. */
	assert(frames == 4);
	assert(regs[TMC5240_GSTAT] == 0);
	assert(stepper_read_status(tmc5240_stepper(h), &s) == 0 &&
	       !s.was_reset);

	regs[TMC5240_DRVSTATUS] = TMC5240_STEALTH | TMC5240_S2GA | TMC5240_OT;
	assert(stepper_read_status(tmc5240_stepper(h), &s) == 0);
	assert(s.stealthchop && s.short_to_gnd && s.overtemp && !s.stall &&
	       !s.standstill && !s.short_to_supply && s.current_scale == 0);

	/* A failed read leaves the status alone. */
	bus_fail = true;
	s.current_scale = 99;
	assert(stepper_read_status(tmc5240_stepper(h), &s) == -ETIMEDOUT);
	assert(s.current_scale == 99);
	bus_fail = false;
	tmc5240_del(h);
}

/* The original's ESP_ERR_INVALID_STATE cases, as errno. */
static void test_refusals(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;
	float x;

	chip_reset();
	h = open_ok(&c);

	/* Moving: XACTUAL is not rewritten. */
	regs[TMC5240_RAMPSTAT] = 0;
	assert(tmc5240_set_position(h, 500) == RET(-EBUSY));
	assert(regs[TMC5240_XACTUAL] == 0);
	regs[TMC5240_RAMPSTAT] = TMC5240_VZERO;
	assert(tmc5240_set_position(h, 500) == RET(0));
	assert(regs[TMC5240_XACTUAL] == 500 && regs[TMC5240_XTARGET] == 500);

	/* The chip says its ADC is broken: no reading. */
	regs[TMC5240_INP_OUT] |= TMC5240_ADC_ERR;
	x = 1.0f;
	assert(tmc5240_supply_voltage(h, &x) == RET(-ENODATA) && x == 1.0f);
	assert(tmc5240_temperature(h, &x) == RET(-ENODATA));

	/* A move that never finishes. */
	regs[TMC5240_RAMPSTAT] = 0;
	assert(tmc5240_wait(h, 20) == RET(-ETIMEDOUT));
	regs[TMC5240_RAMPSTAT] = TMC5240_POSITION_REACHED;
	assert(tmc5240_wait(h, 0) == RET(0));
	tmc5240_del(h);
}

static void test_spi_status(void)
{
	tmc5240_config_t c = base_cfg();
	tmc5240_handle_t h;

	chip_reset();
	fake_status = TMC5240_SPI_STANDSTILL | TMC5240_SPI_RESET_FLAG;
	h = open_ok(&c);
	assert(tmc5240_spi_status(h) ==
	       (TMC5240_SPI_STANDSTILL | TMC5240_SPI_RESET_FLAG));
	tmc5240_del(h);
	fake_status = 0;
}

static void run_tmc5240_cases(void)
{
	test_init();
	test_init_failures();
	test_init_stops_turning_motor();
	test_acceleration_scale();
	test_enable();
	test_current();
	test_status();
	test_refusals();
	test_spi_status();
}
