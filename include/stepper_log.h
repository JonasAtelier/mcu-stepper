/*
 * Kernel dev_err()-style logging for stepper drivers and their users.
 *
 * ESP-IDF: everything goes out under the one ESP-IDF tag "stepper", prefixed
 * with the device's name, so one esp_log_level_set("stepper", ...) controls
 * every chip and a line still says which chip it came from. There is
 * deliberately no level machinery of our own; stepper_dbg() is also subject
 * to CONFIG_LOG_MAXIMUM_LEVEL, and CONFIG_STEPPER_DEBUG is not used.
 *
 * STM32 HAL: every line goes out through printf() as "stepper: <name>: ...",
 * so a line still says which chip it came from. Where printf() ends up is
 * the app's business: with newlib, retarget _write() to a UART. There is
 * deliberately no level machinery; stepper_dbg() alone is compiled out
 * unless CONFIG_STEPPER_DEBUG is 1 (stm-forge: STEPPER_DEBUG = y).
 */
#ifndef STEPPER_LOG_H
#define STEPPER_LOG_H

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <stdio.h>
#endif

#include "stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP_PLATFORM
#define STEPPER_LOG_TAG	"stepper"

#define stepper_err(stepper, fmt, ...) \
	ESP_LOGE(STEPPER_LOG_TAG, "%s: " fmt, (stepper)->name, ##__VA_ARGS__)
#define stepper_warn(stepper, fmt, ...) \
	ESP_LOGW(STEPPER_LOG_TAG, "%s: " fmt, (stepper)->name, ##__VA_ARGS__)
#define stepper_info(stepper, fmt, ...) \
	ESP_LOGI(STEPPER_LOG_TAG, "%s: " fmt, (stepper)->name, ##__VA_ARGS__)
#define stepper_dbg(stepper, fmt, ...) \
	ESP_LOGD(STEPPER_LOG_TAG, "%s: " fmt, (stepper)->name, ##__VA_ARGS__)

#else /* STM32 HAL */
#define stepper_err(stepper, fmt, ...) \
	printf("stepper: %s: " fmt "\n", (stepper)->name, ##__VA_ARGS__)
#define stepper_warn(stepper, fmt, ...) \
	printf("stepper: %s: " fmt "\n", (stepper)->name, ##__VA_ARGS__)
#define stepper_info(stepper, fmt, ...) \
	printf("stepper: %s: " fmt "\n", (stepper)->name, ##__VA_ARGS__)

#if CONFIG_STEPPER_DEBUG
#define stepper_dbg(stepper, fmt, ...) \
	printf("stepper: %s: " fmt "\n", (stepper)->name, ##__VA_ARGS__)
#else
/* Still type-checked, so a debug line cannot rot while it is off. */
#define stepper_dbg(stepper, fmt, ...) do { \
	if (0) \
		printf("stepper: %s: " fmt "\n", (stepper)->name, \
		       ##__VA_ARGS__); \
} while (0)
#endif
#endif /* ESP_PLATFORM */

/*
 * One info line: the flags that are set, by their datasheet names, and the
 * current scale, e.g. "stst otpw cs 17".
 */
void stepper_log_status(const struct stepper_dev *stepper,
			const struct stepper_status *status);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_LOG_H */
