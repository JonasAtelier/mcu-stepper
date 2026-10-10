/*
 * Host-side check of src/stepper_port_esp.c against test/idf_stubs: the I2C
 * ops, which no stepper chip uses yet; the SPI ops the TMC5240 uses; the UART
 * ops the TMC2209 uses, echo included; the enable pin; and the waits, at
 * ESP-IDF's default 100 Hz tick.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stepper_priv.h"

#define UART_PORT 1

extern TickType_t stub_ticks;

static esp_err_t flush_err, done_err;
static bool installed;
static char calls[16];		/* F flush, W write, D wait done, R read */
static uint8_t out[40];		/* what the last write sent */
static size_t out_len;
static uint8_t in[64];		/* what is waiting to be read */
static size_t in_len;
static uint8_t reply[32];	/* what the device sends after a write */
static size_t reply_len;
static bool echo;		/* the line gives every byte written back */
static TickType_t waited;	/* ticks_to_wait of the last read */
static TickType_t done_waited;	/* ticks_to_wait of the last wait done */
static int write_short, read_error;

static void reset(void)
{
	installed = true;
	calls[0] = '\0';
	out_len = in_len = reply_len = 0;
	echo = false;
	write_short = read_error = 0;
	flush_err = done_err = ESP_OK;
}

static void called(char c)
{
	size_t n = strlen(calls);

	assert(n + 1 < sizeof(calls));
	calls[n] = c;
	calls[n + 1] = '\0';
}

bool uart_is_driver_installed(uart_port_t port)
{
	return installed && port == UART_PORT;
}

esp_err_t uart_flush_input(uart_port_t port)
{
	assert(port == UART_PORT);
	called('F');
	if (flush_err != ESP_OK)
		return flush_err;
	in_len = 0;
	return ESP_OK;
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size)
{
	assert(port == UART_PORT && size <= sizeof(out));
	called('W');
	if (write_short)
		return 0;
	memcpy(out, src, size);
	out_len = size;
	/* On a single wire the bytes come straight back, then the device
	 * answers after them: queue both. */
	if (echo) {
		memcpy(in + in_len, src, size);
		in_len += size;
	}
	memcpy(in + in_len, reply, reply_len);
	in_len += reply_len;
	return (int)size;
}

esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t ticks_to_wait)
{
	assert(port == UART_PORT);
	called('D');
	done_waited = ticks_to_wait;
	return done_err;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
		    TickType_t ticks_to_wait)
{
	size_t n = length < in_len ? length : in_len;

	assert(port == UART_PORT);
	called('R');
	waited = ticks_to_wait;
	if (read_error)
		return -1;
	memcpy(buf, in, n);
	memmove(in, in + n, in_len - n);
	in_len -= n;
	return (int)n;
}

/* The SPI device: what was added, the last transaction, the next answer. */
static spi_device_interface_config_t spi_cfg;
static int spi_added;
static esp_err_t spi_err;
static uint8_t spi_out[40];	/* what the last transaction clocked out */
static size_t spi_bits;
static bool spi_inline;		/* the last transaction used tx_data/rx_data */
static uint8_t spi_in[40];	/* what the device clocks back */

esp_err_t spi_bus_add_device(spi_host_device_t host,
			     const spi_device_interface_config_t *config,
			     spi_device_handle_t *out_handle)
{
	assert(host == SPI2_HOST);
	spi_cfg = *config;
	spi_added++;
	*out_handle = (spi_device_handle_t)(void *)&spi_cfg;
	return spi_err;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t device)
{
	assert(device == (spi_device_handle_t)(void *)&spi_cfg);
	spi_added--;
	return ESP_OK;
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t device,
				      spi_transaction_t *transfer)
{
	size_t n = transfer->length / 8;

	assert(device == (spi_device_handle_t)(void *)&spi_cfg);
	assert(transfer->length % 8 == 0 && n <= sizeof(spi_out));
	assert(transfer->rxlength == 0 && "rx as long as tx");
	if (spi_err != ESP_OK)
		return spi_err;
	spi_inline = transfer->flags != 0;
	if (spi_inline) {
		/* Up to four bytes in the transaction itself, both ways. */
		assert(transfer->flags ==
		       (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA) && n <= 4);
		memcpy(spi_out, transfer->tx_data, n);
		memcpy(transfer->rx_data, spi_in, n);
	} else {
		memcpy(spi_out, transfer->tx_buffer, n);
		memcpy(transfer->rx_buffer, spi_in, n);
	}
	spi_bits = transfer->length;
	return ESP_OK;
}

/* The I2C device: what was added, the last write, the next answer. */
static i2c_device_config_t i2c_cfg;
static int i2c_added;
static esp_err_t i2c_err;
static uint8_t i2c_out[40];
static size_t i2c_out_len, i2c_rx_len;

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *config,
				    i2c_master_dev_handle_t *out_handle)
{
	(void)bus;
	i2c_cfg = *config;
	i2c_added++;
	*out_handle = (i2c_master_dev_handle_t)(void *)&i2c_cfg;
	return i2c_err;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
	assert(device == (i2c_master_dev_handle_t)(void *)&i2c_cfg);
	i2c_added--;
	return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
			      const uint8_t *data, size_t length,
			      int timeout_ms)
{
	(void)device;
	(void)timeout_ms;
	memcpy(i2c_out, data, length);
	i2c_out_len = length;
	i2c_rx_len = 0;
	return i2c_err;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
				      const uint8_t *tx, size_t tx_length,
				      uint8_t *rx, size_t rx_length,
				      int timeout_ms)
{
	(void)device;
	(void)timeout_ms;
	memcpy(i2c_out, tx, tx_length);
	i2c_out_len = tx_length;
	i2c_rx_len = rx_length;
	memcpy(rx, reply, rx_length);
	return i2c_err;
}

/* The enable pin: its config and the last level driven. */
static gpio_config_t gpio_cfg;
static esp_err_t gpio_err;
static int level = -1;

esp_err_t gpio_config(const gpio_config_t *config)
{
	gpio_cfg = *config;
	return gpio_err;
}

esp_err_t gpio_set_level(gpio_num_t gpio, uint32_t lvl)
{
	assert(gpio == 21);
	level = (int)lvl;
	return ESP_OK;
}

static void test_i2c(void)
{
	static const uint8_t args[2] = { 0x11, 0x22 };
	struct stepper_bus bus = { 0 };
	uint8_t rx[2] = { 0 };

	/* A refused add leaves no ops: nothing to detach. */
	i2c_err = ESP_ERR_NO_MEM;
	assert(stepper_i2c_attach(&bus, NULL, 0x2A, 0, 10) == -ENOMEM);
	assert(!bus.ops);
	i2c_added = 0;
	i2c_err = ESP_OK;

	assert(stepper_i2c_attach(&bus, NULL, 0x2A, 0, 10) == 0);
	assert(bus.ops->kind == STEPPER_BUS_I2C && i2c_added == 1);
	assert(i2c_cfg.device_address == 0x2A &&
	       i2c_cfg.scl_speed_hz == 400000);

	reset();
	memcpy(reply, "\x12\x34", 2);
	assert(bus.ops->transfer(&bus, 0x0F, NULL, 0, rx, 2) == 0);
	assert(i2c_out_len == 1 && i2c_out[0] == 0x0F && i2c_rx_len == 2);
	assert(rx[0] == 0x12 && rx[1] == 0x34);
	assert(bus.ops->transfer(&bus, 0x20, args, 2, NULL, 0) == 0);
	assert(i2c_out_len == 3 && i2c_out[0] == 0x20 && i2c_out[2] == 0x22);
	assert(bus.ops->transfer(&bus, 0x20, args, 2, rx, 1) == -EINVAL);

	/* Neither exchange() nor receive() on I2C. */
	assert(stepper_bus_exchange(&bus, args, rx, 1) == -EOPNOTSUPP);
	assert(stepper_bus_receive(&bus, rx, 1) == -EOPNOTSUPP);

	stepper_bus_detach(&bus);
	assert(i2c_added == 0);
}

static void test_spi(void)
{
	static const uint8_t args[2] = { 0x11, 0x22 };
	uint8_t many[STEPPER_BUS_MAX_LEN] = { 0 };
	struct stepper_bus bus = { 0 };
	uint8_t rx[3] = { 0 };

	/* A refused add leaves no ops: nothing to detach. */
	spi_err = ESP_ERR_INVALID_ARG;
	assert(stepper_spi_attach(&bus, SPI2_HOST, 5, 1000000, 3) == -EINVAL);
	assert(!bus.ops && spi_added == 1);
	spi_added = 0;
	spi_err = ESP_OK;

	assert(stepper_spi_attach(&bus, SPI2_HOST, 5, 4000000, 3) == 0);
	assert(bus.ops->kind == STEPPER_BUS_SPI && spi_added == 1);
	assert(spi_cfg.mode == 3 && spi_cfg.clock_speed_hz == 4000000 &&
	       spi_cfg.spics_io_num == 5);

	/* A read: cmd, then zeros, and what came back after cmd. */
	memcpy(spi_in, "\xFF\x12\x34\x56", 4);
	memset(spi_out, 0xAA, sizeof(spi_out));
	assert(bus.ops->transfer(&bus, 0x8D, NULL, 0, rx, 3) == 0);
	assert(spi_bits == 4 * 8);
	assert(spi_out[0] == 0x8D && spi_out[1] == 0 && spi_out[2] == 0 &&
	       spi_out[3] == 0);
	assert(rx[0] == 0x12 && rx[1] == 0x34 && rx[2] == 0x56);

	/* A write: cmd and tx, nothing kept. */
	assert(bus.ops->transfer(&bus, 0x0D, args, 2, NULL, 0) == 0);
	assert(spi_bits == 3 * 8);
	assert(spi_out[0] == 0x0D && spi_out[1] == 0x11 && spi_out[2] == 0x22);

	/* Too long, counting tx and rx together; a failure is mapped. */
	assert(bus.ops->transfer(&bus, 0x0D, many, sizeof(many), rx, 1) ==
	       -EINVAL);
	rx[0] = 0x99;
	spi_err = ESP_ERR_TIMEOUT;
	assert(bus.ops->transfer(&bus, 0x8D, NULL, 0, rx, 1) == -ETIMEDOUT);
	assert(rx[0] == 0x99 && "rx untouched on failure");
	spi_err = ESP_OK;

	/* exchange(): one frame, rx[i] clocked in while tx[i] goes out. */
	{
		static const uint8_t two[2] = { 0xFF, 0xFC };
		static const uint8_t five[5] = { 0xA1, 1, 2, 3, 4 };
		uint8_t back[5] = { 0 };
		uint8_t big[2 + STEPPER_BUS_MAX_LEN] = { 0 };

		memcpy(spi_in, "\x9A\xBC", 2);
		assert(stepper_bus_exchange(&bus, two, back, 2) == 0);
		assert(spi_inline && spi_bits == 16);
		assert(spi_out[0] == 0xFF && spi_out[1] == 0xFC);
		assert(back[0] == 0x9A && back[1] == 0xBC);

		/* The TMC5240's five bytes go through buffers. */
		memcpy(spi_in, "\x09\xDE\xAD\xBE\xEF", 5);
		assert(stepper_bus_exchange(&bus, five, back, 5) == 0);
		assert(!spi_inline && spi_bits == 40);
		assert(memcmp(spi_out, five, 5) == 0);
		assert(memcmp(back, "\x09\xDE\xAD\xBE\xEF", 5) == 0);

		assert(stepper_bus_exchange(&bus, two, back, 0) == -EINVAL);
		assert(stepper_bus_exchange(&bus, big, big, sizeof(big)) ==
		       -EINVAL);
		back[0] = 0x55;
		spi_err = ESP_ERR_TIMEOUT;
		assert(stepper_bus_exchange(&bus, five, back, 5) ==
		       -ETIMEDOUT);
		assert(back[0] == 0x55 && "rx untouched on failure");
		spi_err = ESP_OK;
	}

	/* No receive() on SPI. */
	assert(stepper_bus_receive(&bus, rx, 1) == -EOPNOTSUPP);
	stepper_bus_detach(&bus);
	assert(spi_added == 0);
}

static void test_uart(void)
{
	static const uint8_t args[2] = { 0x11, 0x22 };
	uint8_t many[STEPPER_BUS_MAX_LEN + 1] = { 0 };
	struct stepper_bus bus;
	uint8_t rx[2] = { 0 };

	/* The driver is the app's to install; without it, refuse. */
	reset();
	installed = false;
	assert(stepper_uart_attach(&bus, UART_PORT, 100) == -EINVAL);
	installed = true;
	assert(stepper_uart_attach(&bus, UART_PORT, 100) == 0);
	assert(bus.ops->kind == STEPPER_BUS_UART);

	/* A stale byte is dropped, then cmd and args go out, then the reply. */
	in[0] = 0xEE;
	in_len = 1;
	memcpy(reply, "\x12\x34", 2);
	reply_len = 2;
	assert(bus.ops->transfer(&bus, 0x13, args, 2, rx, 2) == 0);
	assert(strcmp(calls, "FWR") == 0);
	assert(out_len == 3 && out[0] == 0x13 && out[1] == 0x11 &&
	       out[2] == 0x22);
	assert(rx[0] == 0x12 && rx[1] == 0x34);
	/* 100 ms at the stub's 100 Hz: 10 ticks, plus one for the first. */
	assert(waited == 11);

	/* A command with no reply does not read. */
	reset();
	assert(bus.ops->transfer(&bus, 0x80, NULL, 0, NULL, 0) == 0);
	assert(strcmp(calls, "FW") == 0);

	/* A reply that stops short is a timeout. */
	reset();
	reply[0] = 0x12;
	reply_len = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -ETIMEDOUT);

	/* Driver failures. */
	reset();
	read_error = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EIO);
	reset();
	write_short = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EIO);
	assert(strcmp(calls, "FW") == 0);
	reset();
	flush_err = ESP_ERR_INVALID_ARG;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EINVAL);
	assert(strcmp(calls, "F") == 0);
	reset();
	assert(bus.ops->transfer(&bus, 0x13, many, sizeof(many), NULL, 0) ==
	       -EINVAL);
	assert(calls[0] == '\0');

	stepper_bus_detach(&bus);
}

/*
 * The TMC2209's single wire: exchange() flushes, sends, waits for the last
 * bit to leave and reads back as many bytes as went out; receive() then
 * takes the reply that follows, nothing flushed in between.
 */
static void test_uart_echo(void)
{
	static const uint8_t req[4] = { 0x05, 0x00, 0x02, 0x3F };
	uint8_t echo_back[4] = { 0 }, answer[8] = { 0 };
	uint8_t big[2 + STEPPER_BUS_MAX_LEN] = { 0 };
	struct stepper_bus bus;

	reset();
	assert(stepper_uart_attach(&bus, UART_PORT, 20) == 0);

	in[0] = 0xEE;
	in_len = 1;
	echo = true;
	memcpy(reply, "\x05\xFF\x02\x00\x00\x00\x07\x99", 8);
	reply_len = 8;
	assert(stepper_bus_exchange(&bus, req, echo_back, 4) == 0);
	assert(strcmp(calls, "FWDR") == 0);
	assert(memcmp(echo_back, req, 4) == 0);
	/* 20 ms at 100 Hz: 2 ticks rounded up, plus one. */
	assert(done_waited == 3 && waited == 3);
	assert(stepper_bus_receive(&bus, answer, 8) == 0);
	assert(strcmp(calls, "FWDRR") == 0);
	assert(memcmp(answer, reply, 8) == 0 && waited == 3);

	/* A short echo, or a short answer, is a timeout. */
	reset();
	assert(stepper_bus_exchange(&bus, req, echo_back, 4) == -ETIMEDOUT);
	reset();
	echo = true;
	assert(stepper_bus_exchange(&bus, req, echo_back, 4) == 0);
	assert(stepper_bus_receive(&bus, answer, 8) == -ETIMEDOUT);

	/* The last bit never leaving is a timeout, before any read. */
	reset();
	done_err = ESP_ERR_TIMEOUT;
	assert(stepper_bus_exchange(&bus, req, echo_back, 4) == -ETIMEDOUT);
	assert(strcmp(calls, "FWD") == 0);

	/* Driver failures; nothing, or too much, refused first. */
	reset();
	write_short = 1;
	assert(stepper_bus_exchange(&bus, req, echo_back, 4) == -EIO);
	reset();
	read_error = 1;
	echo = true;
	assert(stepper_bus_exchange(&bus, req, echo_back, 4) == -EIO);
	assert(stepper_bus_receive(&bus, answer, 8) == -EIO);
	reset();
	assert(stepper_bus_exchange(&bus, req, echo_back, 0) == -EINVAL);
	assert(stepper_bus_exchange(&bus, big, big, sizeof(big)) == -EINVAL);
	assert(calls[0] == '\0');

	stepper_bus_detach(&bus);
}

static void test_pin(void)
{
	struct stepper_pin tied = { .gpio = GPIO_NUM_NC };
	struct stepper_pin en = { .gpio = 21 };

	assert(!stepper_pin_wired(&tied) && stepper_pin_wired(&en));

	/* Made an output, as both original drivers did, and driven. */
	assert(stepper_pin_output(&en, true) == 0);
	assert(gpio_cfg.pin_bit_mask == 1ULL << 21 &&
	       gpio_cfg.mode == GPIO_MODE_OUTPUT && level == 1);
	stepper_pin_set(&en, false);
	assert(level == 0);

	/* A refused config is mapped and drives nothing. */
	level = -1;
	gpio_err = ESP_ERR_INVALID_ARG;
	assert(stepper_pin_output(&en, true) == -EINVAL && level == -1);
	gpio_err = ESP_OK;
}

static void test_waits(void)
{
	uint32_t start;

	/* At 100 Hz. 0 is no wait at all. */
	stub_ticks = 500;
	assert(stepper_expired(stepper_tick_now(), 0));

	/* 15 ms rounds up to 2 ticks, plus 1 for the partial first. */
	start = stepper_tick_now();
	stub_ticks = start + 2;
	assert(!stepper_expired(start, 15));
	stub_ticks = start + 3;
	assert(stepper_expired(start, 15));

	/* UINT32_MAX saturates to portMAX_DELAY rather than wrap to 0. */
	start = stepper_tick_now();
	stub_ticks = start + 1000000;
	assert(!stepper_expired(start, UINT32_MAX));

	/* A poll sleeps at least one tick: pdMS_TO_TICKS(5) is 0 here. */
	stub_ticks = 0;
	stepper_poll_wait(5);
	assert(stub_ticks == 1);
	stepper_poll_wait(30);
	assert(stub_ticks == 4);

	/* The TMC2209's settle wait is esp-tmc2209-stepper's
	 * vTaskDelay(pdMS_TO_TICKS(ms)), truncation and all. */
	stepper_sleep_ms(10);
	assert(stub_ticks == 5);
	stepper_sleep_ms(5);
	assert(stub_ticks == 5);
}

int main(void)
{
	test_i2c();
	test_spi();
	test_uart();
	test_uart_echo();
	test_pin();
	test_waits();
	printf("test_port_esp: all passed\n");
	return 0;
}
