#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#include "esp_err.h"

/* Host stub for the ESP-IDF gpio driver: only what the drivers use. */

typedef int gpio_num_t;

#define GPIO_NUM_NC	(-1)

typedef enum {
	GPIO_MODE_DISABLE = 0,
	GPIO_MODE_INPUT = 1,
	GPIO_MODE_OUTPUT = 2,
} gpio_mode_t;

typedef struct {
	uint64_t pin_bit_mask;
	gpio_mode_t mode;
	int pull_up_en;
	int pull_down_en;
	int intr_type;
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_level(gpio_num_t gpio, uint32_t level);

#endif
