/*
 * Host-side check of the core on the STM32 HAL: the cases are
 * ../core_cases.h; this file supplies the log prefix.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "stepper.h"
#include "stepper_log.h"

/* The HAL's lines are bare printf(), no level letter. */
#define LOG_PREFIX(level)	""

#include "../core_cases.h"

/* With CONFIG_STEPPER_DEBUG on, as test/Makefile builds it, dbg prints. */
static void test_dbg(void)
{
	struct stepper_dev stepper = { .name = "fake" };

	stepper_dbg(&stepper, "d %d", 1);
	assert(!strcmp(stub_log_last, "stepper: fake: d 1"));
}

int main(void)
{
	run_core_cases();
	test_dbg();
	printf("all core checks passed\n");
	return 0;
}
