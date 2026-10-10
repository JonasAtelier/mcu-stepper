#include <string.h>

#include "stepper_log.h"

/* Append " name" to buf when on; buf is big enough for every flag at once. */
static void stepper_flag(char *buf, bool on, const char *name)
{
	if (!on)
		return;
	strcat(buf, " ");
	strcat(buf, name);
}

void stepper_log_status(const struct stepper_dev *stepper,
			const struct stepper_status *status)
{
	/* " stst stealth otpw ot s2g s2vs ol stall reset" and its NUL */
	char flags[48] = "";

	stepper_flag(flags, status->standstill, "stst");
	stepper_flag(flags, status->stealthchop, "stealth");
	stepper_flag(flags, status->overtemp_warning, "otpw");
	stepper_flag(flags, status->overtemp, "ot");
	stepper_flag(flags, status->short_to_gnd, "s2g");
	stepper_flag(flags, status->short_to_supply, "s2vs");
	stepper_flag(flags, status->open_load, "ol");
	stepper_flag(flags, status->stall, "stall");
	stepper_flag(flags, status->was_reset, "reset");
	stepper_info(stepper, "%s cs %u", flags[0] ? flags + 1 : "-",
		     (unsigned)status->current_scale);
}
