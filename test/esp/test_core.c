/*
 * Host-side check of the core on ESP-IDF: the cases are ../core_cases.h;
 * this file supplies the log prefix.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "stepper.h"
#include "stepper_log.h"

/* The ESP-IDF stub log puts the level letter first. */
#define LOG_PREFIX(level)	level " "

#include "../core_cases.h"

int main(void)
{
	run_core_cases();
	printf("all core checks passed\n");
	return 0;
}
