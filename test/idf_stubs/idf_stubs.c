/* The stub definitions every test binary links. */
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

char stub_log_last[256];

/*
 * ESP-IDF's default CONFIG_FREERTOS_HZ. Weak, because esp-tmc5240-stepper's
 * own test defines it, at 1000.
 */
__attribute__((weak)) unsigned int stub_tick_hz = 100;

void stub_log(char level, const char *tag, const char *fmt, ...)
{
	va_list ap;
	int n;

	n = snprintf(stub_log_last, sizeof(stub_log_last), "%c %s: ", level,
		     tag);
	va_start(ap, fmt);
	vsnprintf(stub_log_last + n, sizeof(stub_log_last) - n, fmt, ap);
	va_end(ap);
	fprintf(stderr, "%s\n", stub_log_last);
}

/*
 * A scheduler of one task: time passes only in vTaskDelay(), and a mutex
 * taken twice can only be the code locking itself out, which on the real,
 * non-recursive mutex deadlocks forever. Weak, so a test that wants its own
 * clock or lock -- esp-tmc5240-stepper's does -- defines them.
 */
TickType_t stub_ticks;
static int stub_mutex;
static bool stub_mutex_held;

__attribute__((weak)) void vTaskDelay(TickType_t ticks)
{
	stub_ticks += ticks;
}

__attribute__((weak)) TickType_t xTaskGetTickCount(void)
{
	return stub_ticks;
}

__attribute__((weak)) SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
	return (SemaphoreHandle_t)(void *)&stub_mutex;
}

__attribute__((weak)) BaseType_t xSemaphoreTake(SemaphoreHandle_t sem,
						TickType_t ticks)
{
	(void)ticks;
	assert(sem == (SemaphoreHandle_t)(void *)&stub_mutex);
	assert(!stub_mutex_held);
	stub_mutex_held = true;
	return pdTRUE;
}

__attribute__((weak)) BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
	assert(sem == (SemaphoreHandle_t)(void *)&stub_mutex);
	assert(stub_mutex_held);
	stub_mutex_held = false;
	return pdTRUE;
}

__attribute__((weak)) void vSemaphoreDelete(SemaphoreHandle_t sem)
{
	assert(sem == (SemaphoreHandle_t)(void *)&stub_mutex);
	assert(!stub_mutex_held);
}

/*
 * src/stepper_port_esp.c is in every binary and calls the bus and GPIO ones.
 * A test with a fake chip defines the ones it needs, which win over these
 * weak ones; a test that reaches one it did not define has a bug, so stop it
 * there.
 */
__attribute__((weak)) esp_err_t
i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
			  const i2c_device_config_t *config,
			  i2c_master_dev_handle_t *out_handle)
{
	(void)bus;
	(void)config;
	(void)out_handle;
	abort();
}

__attribute__((weak)) esp_err_t
i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
	(void)device;
	abort();
}

__attribute__((weak)) esp_err_t
i2c_master_transmit(i2c_master_dev_handle_t device, const uint8_t *data,
		    size_t length, int timeout_ms)
{
	(void)device;
	(void)data;
	(void)length;
	(void)timeout_ms;
	abort();
}

__attribute__((weak)) esp_err_t
i2c_master_transmit_receive(i2c_master_dev_handle_t device,
			    const uint8_t *tx, size_t tx_length, uint8_t *rx,
			    size_t rx_length, int timeout_ms)
{
	(void)device;
	(void)tx;
	(void)tx_length;
	(void)rx;
	(void)rx_length;
	(void)timeout_ms;
	abort();
}

__attribute__((weak)) esp_err_t
spi_bus_add_device(spi_host_device_t host,
		   const spi_device_interface_config_t *config,
		   spi_device_handle_t *out_handle)
{
	(void)host;
	(void)config;
	(void)out_handle;
	abort();
}

__attribute__((weak)) esp_err_t
spi_bus_remove_device(spi_device_handle_t device)
{
	(void)device;
	abort();
}

__attribute__((weak)) esp_err_t
spi_device_polling_transmit(spi_device_handle_t device,
			    spi_transaction_t *transfer)
{
	(void)device;
	(void)transfer;
	abort();
}

__attribute__((weak)) bool
uart_is_driver_installed(uart_port_t uart_num)
{
	(void)uart_num;
	abort();
}

__attribute__((weak)) esp_err_t
uart_flush_input(uart_port_t uart_num)
{
	(void)uart_num;
	abort();
}

__attribute__((weak)) int
uart_write_bytes(uart_port_t uart_num, const void *src, size_t size)
{
	(void)uart_num;
	(void)src;
	(void)size;
	abort();
}

__attribute__((weak)) esp_err_t
uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait)
{
	(void)uart_num;
	(void)ticks_to_wait;
	abort();
}

__attribute__((weak)) int
uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length,
		TickType_t ticks_to_wait)
{
	(void)uart_num;
	(void)buf;
	(void)length;
	(void)ticks_to_wait;
	abort();
}

__attribute__((weak)) esp_err_t
gpio_config(const gpio_config_t *config)
{
	(void)config;
	abort();
}

__attribute__((weak)) esp_err_t
gpio_set_level(gpio_num_t gpio, uint32_t level)
{
	(void)gpio;
	(void)level;
	abort();
}
