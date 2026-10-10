/*
 * The core's cases -- stepper_enable(), stepper_set_current(),
 * stepper_read_status() and stepper_log_status() -- shared by
 * esp/test_core.c and stm/test_core.c. Each includes this after providing:
 *
 *   LOG_PREFIX             what the platform's log puts before "stepper: "
 *   stub_log_last          the last log line
 *
 * What each chip's ops do lives in its driver, so each test_<chip>.c checks
 * that against the real driver.
 */

static int fake_ret;
static int fake_enables, fake_currents, fake_reads;
static bool fake_enabled;
static float fake_run, fake_hold;

static int fake_enable(struct stepper_dev *stepper, bool enable)
{
	(void)stepper;
	fake_enables++;
	if (fake_ret)
		return fake_ret;
	fake_enabled = enable;
	return 0;
}

static int fake_set_current(struct stepper_dev *stepper, float run_a,
			    float hold_a)
{
	(void)stepper;
	fake_currents++;
	fake_run = run_a;
	fake_hold = hold_a;
	return fake_ret;
}

static int fake_read_status(struct stepper_dev *stepper,
			    struct stepper_status *status)
{
	(void)stepper;
	fake_reads++;
	/* Scribble first, as a driver failing half way through might. */
	memset(status, 0xA5, sizeof(*status));
	if (fake_ret)
		return fake_ret;
	memset(status, 0, sizeof(*status));
	status->standstill = true;
	status->overtemp_warning = true;
	status->current_scale = 17;
	return 0;
}

static const struct stepper_ops fake_ops = {
	.enable = fake_enable,
	.set_current = fake_set_current,
	.read_status = fake_read_status,
};

static struct stepper_dev fake_dev(void)
{
	struct stepper_dev stepper = {
		.name = "fake",
		.ops = &fake_ops,
		.microsteps = 16,
	};

	fake_ret = 0;
	fake_enables = fake_currents = fake_reads = 0;
	fake_enabled = false;
	return stepper;
}

static void test_dispatch(void)
{
	struct stepper_dev stepper = fake_dev();
	struct stepper_status st;

	assert(stepper_enable(&stepper, true) == 0 && fake_enabled);
	assert(stepper_enable(&stepper, false) == 0 && !fake_enabled);
	assert(fake_enables == 2);

	/* Amperes go through as they are; the chip decides about hold 0. */
	assert(stepper_set_current(&stepper, 1.2f, 0.0f) == 0);
	assert(fake_currents == 1 && fake_run == 1.2f && fake_hold == 0.0f);

	assert(stepper_read_status(&stepper, &st) == 0 && fake_reads == 1);
	assert(st.standstill && st.overtemp_warning && !st.overtemp &&
	       st.current_scale == 17);
}

static void test_errors(void)
{
	struct stepper_dev stepper = fake_dev();
	struct stepper_status st, before;

	/* Whatever the chip says comes back unchanged... */
	fake_ret = -EOPNOTSUPP;
	assert(stepper_enable(&stepper, true) == -EOPNOTSUPP);
	fake_ret = -ETIMEDOUT;
	assert(stepper_set_current(&stepper, 1.0f, 0.5f) == -ETIMEDOUT);

	/* ...and a failed read leaves *status alone, whatever the driver
	 * scribbled on its way to failing. */
	memset(&st, 0x3C, sizeof(st));
	before = st;
	fake_ret = -EBADMSG;
	assert(stepper_read_status(&stepper, &st) == -EBADMSG);
	assert(memcmp(&st, &before, sizeof(st)) == 0);

	/* Nothing to call is the caller's mistake, and nothing is called. */
	fake_enables = fake_currents = fake_reads = 0;
	assert(stepper_enable(NULL, true) == -EINVAL);
	assert(stepper_set_current(NULL, 1.0f, 0.0f) == -EINVAL);
	assert(stepper_read_status(NULL, &st) == -EINVAL);
	assert(stepper_read_status(&stepper, NULL) == -EINVAL);
	stepper.ops = NULL;
	assert(stepper_enable(&stepper, true) == -EINVAL);
	assert(stepper_set_current(&stepper, 1.0f, 0.0f) == -EINVAL);
	assert(stepper_read_status(&stepper, &st) == -EINVAL);
	assert(fake_enables + fake_currents + fake_reads == 0);

	/* An ops table without the op is refused the same way. */
	{
		static const struct stepper_ops none = { 0 };

		stepper.ops = &none;
		assert(stepper_enable(&stepper, true) == -EINVAL);
		assert(stepper_set_current(&stepper, 1.0f, 0.0f) == -EINVAL);
		assert(stepper_read_status(&stepper, &st) == -EINVAL);
	}
}

static void test_log(void)
{
	struct stepper_dev stepper = fake_dev();
	struct stepper_status st = { 0 };

	stepper_err(&stepper, "no answer from %d (%d)", 3, -110);
	assert(!strcmp(stub_log_last, LOG_PREFIX("E")
		       "stepper: fake: no answer from 3 (-110)"));
	stepper_info(&stepper, "i");
	assert(!strcmp(stub_log_last, LOG_PREFIX("I") "stepper: fake: i"));

	/* Nothing set: a dash, and the current scale. */
	stepper_log_status(&stepper, &st);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I") "stepper: fake: - cs 0"));

	st.standstill = st.overtemp_warning = true;
	st.current_scale = 17;
	stepper_log_status(&stepper, &st);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I")
		       "stepper: fake: stst otpw cs 17"));

	/* Every flag at once still fits. */
	st = (struct stepper_status){ true, true, true, true, true, true,
				      true, true, true, 31 };
	stepper_log_status(&stepper, &st);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I")
		       "stepper: fake: stst stealth otpw ot s2g s2vs ol "
		       "stall reset cs 31"));
}

static void run_core_cases(void)
{
	test_dispatch();
	test_errors();
	test_log();
}
