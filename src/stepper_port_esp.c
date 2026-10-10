/*
 * stepper_priv.h on ESP-IDF. The CMakeLists lists this file; stm-forge
 * compiles every .c in src/, so outside ESP-IDF it is empty.
 */
#ifdef ESP_PLATFORM

#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stepper.h"
#include "stepper_priv.h"

/* 400 kHz when a caller says 0: fast mode. */
#define DEFAULT_SCL_HZ		400000

/*
 * Ticks for at least ms milliseconds, for the bus timeouts. Two things make
 * a plain pdMS_TO_TICKS(ms) wait less than asked:
 *
 *   - pdMS_TO_TICKS() truncates. At IDF's default 100 Hz tick, 2 ms is 0
 *     ticks and 15 ms is 1, so round up instead.
 *   - A wait of n ticks counts tick interrupts, and the first one may be
 *     about to fire, so n ticks can be as little as n - 1 full periods.
 *     Add one.
 *
 * So 20 ms at 100 Hz is 3 ticks, 20-30 ms: long, never short.
 */
static TickType_t stepper_ticks(uint32_t ms)
{
	return (TickType_t)(((uint64_t)ms * configTICK_RATE_HZ + 999) / 1000) +
	       1;
}

/* ------------------------------------------------------------- waits -- */

uint32_t stepper_tick_now(void)
{
	return (uint32_t)xTaskGetTickCount();
}

/*
 * The TMC5240 original's deadline, unchanged: limit_ms rounds up to whole
 * ticks plus one for the partial first, so limit_ms of real time has surely
 * gone; 0 is no wait at all, after the one look the caller already took; a
 * count past the tick type saturates, so UINT32_MAX still waits forever.
 */
bool stepper_expired(uint32_t start, uint32_t limit_ms)
{
	uint64_t ticks = ((uint64_t)limit_ms * configTICK_RATE_HZ + 999) / 1000
			 + 1;
	TickType_t limit = !limit_ms ? 0 :
			   ticks > portMAX_DELAY ? portMAX_DELAY :
			   (TickType_t)ticks;

	return (TickType_t)(xTaskGetTickCount() - (TickType_t)start) >= limit;
}

/*
 * Between two polls: at least one tick. At the default 100 Hz,
 * pdMS_TO_TICKS() of a few milliseconds is 0, and vTaskDelay(0) only
 * yields, so a poll loop would spin.
 */
void stepper_poll_wait(uint32_t ms)
{
	TickType_t ticks = pdMS_TO_TICKS(ms);

	vTaskDelay(ticks ? ticks : 1);
}

/*
 * vTaskDelay(pdMS_TO_TICKS(ms)), as esp-tmc2209-stepper waited. It
 * truncates: at 100 Hz, 10 ms is one tick, which can end at once. Kept as
 * the original had it; see docs/development.md.
 */
void stepper_sleep_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

/* --------------------------------------------------------------- pin -- */

bool stepper_pin_wired(const struct stepper_pin *pin)
{
	return pin->gpio >= 0;
}

int stepper_pin_output(const struct stepper_pin *pin, bool high)
{
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << pin->gpio,
		.mode = GPIO_MODE_OUTPUT,
	};
	int ret;

	ret = stepper_errno(gpio_config(&io));
	if (ret)
		return ret;
	gpio_set_level(pin->gpio, high ? 1 : 0);
	return 0;
}

void stepper_pin_set(const struct stepper_pin *pin, bool high)
{
	gpio_set_level(pin->gpio, high ? 1 : 0);
}

/* --------------------------------------------------------------- I2C -- */

static int stepper_i2c_transfer(struct stepper_bus *bus, uint8_t cmd,
				const uint8_t *tx, size_t tx_len, uint8_t *rx,
				size_t rx_len)
{
	uint8_t out[1 + STEPPER_BUS_MAX_LEN];

	if ((tx_len && rx_len) || tx_len > STEPPER_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (rx_len)
		return stepper_errno(i2c_master_transmit_receive(
			bus->i2c, out, 1, rx, rx_len, (int)bus->timeout_ms));
	if (tx_len)
		memcpy(out + 1, tx, tx_len);
	return stepper_errno(i2c_master_transmit(bus->i2c, out, 1 + tx_len,
						 (int)bus->timeout_ms));
}

static void stepper_i2c_detach(struct stepper_bus *bus)
{
	i2c_master_bus_rm_device(bus->i2c);
}

static const struct stepper_bus_ops stepper_i2c_ops = {
	.kind = STEPPER_BUS_I2C,
	.transfer = stepper_i2c_transfer,
	.detach = stepper_i2c_detach,
};

int stepper_i2c_attach(struct stepper_bus *bus, stepper_i2c_bus handle,
		       uint8_t address, uint32_t scl_hz, uint32_t timeout_ms)
{
	i2c_device_config_t devcfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address,
		.scl_speed_hz = scl_hz ? scl_hz : DEFAULT_SCL_HZ,
	};

	int ret;

	ret = stepper_errno(i2c_master_bus_add_device(handle, &devcfg,
						      &bus->i2c));
	if (ret)
		return ret;
	/* Set only now: a bus with ops is one that has something to detach. */
	bus->ops = &stepper_i2c_ops;
	bus->timeout_ms = timeout_ms;
	return 0;
}

/* --------------------------------------------------------------- SPI -- */

/*
 * One full-duplex transaction, so CS stays low across cmd, tx and rx: clock
 * out cmd and tx followed by zeros, and keep what came back after them. The
 * buffers live on the stack, word-aligned: IDF bounces non-DMA-capable or
 * unaligned buffers through a copy of its own.
 */
static int stepper_spi_transfer(struct stepper_bus *bus, uint8_t cmd,
				const uint8_t *tx, size_t tx_len, uint8_t *rx,
				size_t rx_len)
{
	uint8_t out[1 + STEPPER_BUS_MAX_LEN] __attribute__((aligned(4))) = {
		0
	};
	uint8_t in[1 + STEPPER_BUS_MAX_LEN] __attribute__((aligned(4)));
	spi_transaction_t t = { 0 };
	int ret;

	if (tx_len + rx_len > STEPPER_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	t.length = (1 + tx_len + rx_len) * 8;
	t.tx_buffer = out;
	t.rx_buffer = in;
	ret = stepper_errno(spi_device_polling_transmit(bus->spi, &t));
	if (!ret && rx_len)
		memcpy(rx, in + 1 + tx_len, rx_len);
	return ret;
}

/*
 * One frame, full duplex: rx[i] is what came back while tx[i] went out.
 * Polled, not queued: for a few bytes the interrupt and task switch of a
 * queued transfer cost more than the transfer itself. A frame of up to four
 * bytes travels in the transaction itself (tx_data, rx_data); a longer one,
 * like the TMC5240's five, through word-aligned buffers on the stack.
 */
static int stepper_spi_exchange(struct stepper_bus *bus, const uint8_t *tx,
				uint8_t *rx, size_t len)
{
	uint8_t out[1 + STEPPER_BUS_MAX_LEN] __attribute__((aligned(4)));
	uint8_t in[1 + STEPPER_BUS_MAX_LEN] __attribute__((aligned(4)));
	spi_transaction_t t = { 0 };
	bool inline_data = len <= sizeof(t.tx_data);
	int ret;

	if (!len || len > sizeof(out))
		return -EINVAL;

	t.length = len * 8;
	if (inline_data) {
		t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
		memcpy(t.tx_data, tx, len);
	} else {
		memcpy(out, tx, len);
		t.tx_buffer = out;
		t.rx_buffer = in;
	}
	ret = stepper_errno(spi_device_polling_transmit(bus->spi, &t));
	if (!ret)
		memcpy(rx, inline_data ? t.rx_data : in, len);
	return ret;
}

static void stepper_spi_detach(struct stepper_bus *bus)
{
	spi_bus_remove_device(bus->spi);
}

static const struct stepper_bus_ops stepper_spi_ops = {
	.kind = STEPPER_BUS_SPI,
	.transfer = stepper_spi_transfer,
	.exchange = stepper_spi_exchange,
	.detach = stepper_spi_detach,
};

int stepper_spi_attach(struct stepper_bus *bus, spi_host_device_t host,
		       int cs_pin, uint32_t clock_hz, uint8_t mode)
{
	spi_device_interface_config_t devcfg = {
		.mode = mode,
		.clock_speed_hz = (int)clock_hz,
		.spics_io_num = cs_pin,
		.queue_size = 1,
	};

	int ret;

	ret = stepper_errno(spi_bus_add_device(host, &devcfg, &bus->spi));
	if (ret)
		return ret;
	bus->ops = &stepper_spi_ops;
	bus->timeout_ms = 0;	/* spi_master transfers take none */
	return 0;
}

/* -------------------------------------------------------------- UART -- */

/*
 * Drop whatever is waiting -- a late reply to an earlier command would read
 * as the start of this one -- then send, then wait for the whole reply.
 * uart_read_bytes() returns early only with everything; a short count means
 * the timeout ran out.
 */
static int stepper_uart_transfer(struct stepper_bus *bus, uint8_t cmd,
				 const uint8_t *tx, size_t tx_len,
				 uint8_t *rx, size_t rx_len)
{
	uint8_t out[1 + STEPPER_BUS_MAX_LEN];
	int n, ret;

	if (tx_len > STEPPER_BUS_MAX_LEN || rx_len > INT32_MAX)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	ret = stepper_errno(uart_flush_input(bus->uart));
	if (ret)
		return ret;
	n = uart_write_bytes(bus->uart, out, 1 + tx_len);
	if (n != (int)(1 + tx_len))
		return -EIO;
	if (!rx_len)
		return 0;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)rx_len,
			    stepper_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < rx_len ? -ETIMEDOUT : 0;
}

/*
 * The UART driver keeps everything the line delivers in its RX ring buffer,
 * so the echo of a whole datagram waits there however fast it came: flush,
 * send, wait for the last bit to leave, then read back as many bytes as
 * went out -- what esp-tmc2209-stepper did before every reply.
 */
static int stepper_uart_exchange(struct stepper_bus *bus, const uint8_t *tx,
				 uint8_t *rx, size_t len)
{
	int n, ret;

	if (!len || len > 1 + STEPPER_BUS_MAX_LEN)
		return -EINVAL;

	ret = stepper_errno(uart_flush_input(bus->uart));
	if (ret)
		return ret;
	n = uart_write_bytes(bus->uart, tx, len);
	if (n != (int)len)
		return -EIO;
	ret = stepper_errno(uart_wait_tx_done(bus->uart,
					      stepper_ticks(bus->timeout_ms)));
	if (ret)
		return ret;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)len,
			    stepper_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < len ? -ETIMEDOUT : 0;
}

static int stepper_uart_receive(struct stepper_bus *bus, uint8_t *rx,
				size_t rx_len)
{
	int n;

	if (rx_len > INT32_MAX)
		return -EINVAL;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)rx_len,
			    stepper_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < rx_len ? -ETIMEDOUT : 0;
}

/* The driver is the app's: nothing was added, nothing to remove. */
static void stepper_uart_detach(struct stepper_bus *bus)
{
	(void)bus;
}

static const struct stepper_bus_ops stepper_uart_ops = {
	.kind = STEPPER_BUS_UART,
	.transfer = stepper_uart_transfer,
	.exchange = stepper_uart_exchange,
	.receive = stepper_uart_receive,
	.detach = stepper_uart_detach,
};

int stepper_uart_attach(struct stepper_bus *bus, uart_port_t port,
			uint32_t timeout_ms)
{
	if (!uart_is_driver_installed(port))
		return -EINVAL;
	bus->ops = &stepper_uart_ops;
	bus->timeout_ms = timeout_ms;
	bus->uart = port;
	return 0;
}

#endif /* ESP_PLATFORM */
