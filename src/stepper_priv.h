/*
 * Shared by the chip drivers inside this library. Not public API.
 *
 * Two layers sit under the chips, the way regmap sits under a kernel driver:
 *
 *   - The bus kind is chosen at run time. A chip holds a struct stepper_bus
 *     and reaches it only through bus->ops, so a driver can sit on I2C, SPI
 *     or a UART. Every category library offers all three kinds, used or not:
 *     the TMC2209 is on a UART, the TMC5240 on SPI.
 *   - The platform is chosen at link time. stepper_port_esp.c and
 *     stepper_port_stm.c each define the ops tables, the attach functions,
 *     the enable pin and the waits below for their platform; the other one
 *     compiles to nothing.
 *
 * ESP_PLATFORM, which ESP-IDF defines for every component, and the HAL's own
 * module switches are defined or not, never 0 or 1, so they are the one place
 * #ifdef is right; this library's CONFIG_STEPPER_* options are always tested
 * with #if.
 */
#ifndef STEPPER_PRIV_H
#define STEPPER_PRIV_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The most bytes after cmd: tx on I2C and UART, tx and rx together on SPI. */
#define STEPPER_BUS_MAX_LEN	31

struct stepper_bus;

enum stepper_bus_kind {
	STEPPER_BUS_I2C,
	STEPPER_BUS_SPI,
	STEPPER_BUS_UART,
};

struct stepper_bus_ops {
	enum stepper_bus_kind kind;
	/*
	 * One transaction: cmd, then tx_len bytes of tx, then rx_len bytes
	 * into rx. cmd is the register address, with any SPI read bit already
	 * set by the chip, and the first byte of a datagram on a UART.
	 *
	 *   I2C:  rx_len ? cmd, repeated start, read : cmd and tx, one write;
	 *         a bare cmd is a one-byte write. Sending tx and then reading
	 *         is -EINVAL.
	 *   SPI:  CS low, cmd, tx, rx, CS high -- high again even when the
	 *         transfer failed.
	 *   UART: drop stale input, send cmd and tx, then wait for rx_len
	 *         bytes; fewer within the timeout is -ETIMEDOUT. For a line
	 *         where the MCU does not hear itself; see exchange().
	 *
	 * On SPI more than STEPPER_BUS_MAX_LEN bytes after cmd is -EINVAL; on
	 * I2C and UART only tx is bounded so, and rx on the HAL by its 16-bit
	 * count. 0 or a negative errno.
	 */
	int (*transfer)(struct stepper_bus *bus, uint8_t cmd,
			const uint8_t *tx, size_t tx_len, uint8_t *rx,
			size_t rx_len);
	/*
	 * len bytes of tx out while len bytes come back into rx, rx[i] the
	 * one that answered tx[i]. More than 1 + STEPPER_BUS_MAX_LEN bytes, or
	 * none, is -EINVAL. NULL on I2C.
	 *
	 *   SPI:  one frame, full duplex: CS low, the bytes at the same clocks,
	 *         CS high -- high again even when the transfer failed. For a
	 *         part whose answer to one frame comes back in the next.
	 *   UART: drop stale input, send tx, and collect what the line gave
	 *         back meanwhile. On a single wire the MCU hears its own
	 *         transmission; this is how a chip takes that echo off the
	 *         line, so that the answer which follows starts clean. Fewer
	 *         than len bytes back within the timeout is -ETIMEDOUT.
	 */
	int (*exchange)(struct stepper_bus *bus, const uint8_t *tx,
			uint8_t *rx, size_t len);
	/*
	 * rx_len bytes, with nothing sent and nothing dropped first: the
	 * answer that follows an exchange(). Fewer within the timeout is
	 * -ETIMEDOUT; rx is then undefined. NULL on I2C and SPI.
	 */
	int (*receive)(struct stepper_bus *bus, uint8_t *rx, size_t rx_len);
	/* Undo the attach. The bus itself is the app's and is left alone. */
	void (*detach)(struct stepper_bus *bus);
};

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"

/* The I2C argument of every I2C attach on this platform. */
typedef i2c_master_bus_handle_t stepper_i2c_bus;

/*
 * One device on one bus, as its driver holds it. STEPPER_HAVE_BUS says the
 * platform has at least one bus kind, so the struct exists.
 */
#define STEPPER_HAVE_BUS
struct stepper_bus {
	const struct stepper_bus_ops *ops;
	uint32_t timeout_ms;
	union {
		i2c_master_dev_handle_t i2c;
		spi_device_handle_t spi;
		uart_port_t uart;
	};
};

/* An output pin the driver owns: the chip's enable. */
struct stepper_pin {
	gpio_num_t gpio;	/* negative: not wired */
};

/*
 * esp_err_t -> negative errno, at the bus boundary and nowhere else.
 * ESP_ERR_NOT_FOUND is -ENXIO (nothing at that address), not -ENODEV
 * (something answered but it is the wrong chip).
 */
static inline int stepper_errno(esp_err_t err)
{
	switch (err) {
	case ESP_OK:
		return 0;
	case ESP_ERR_INVALID_ARG:
		return -EINVAL;
	case ESP_ERR_TIMEOUT:
		return -ETIMEDOUT;
	case ESP_ERR_NOT_FOUND:
		return -ENXIO;
	case ESP_ERR_INVALID_SIZE:
		return -EMSGSIZE;
	case ESP_ERR_NO_MEM:
		return -ENOMEM;
	case ESP_ERR_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	default:
		return -EIO;
	}
}

/*
 * negative errno -> esp_err_t, for the chips' handle APIs, which return
 * esp_err_t on ESP-IDF as they always did. The codes the original drivers
 * returned come back as they were: -ENODEV and -ENXIO as
 * ESP_ERR_NOT_FOUND, -EBADMSG as ESP_ERR_INVALID_CRC, -EBUSY and -ENODATA
 * as ESP_ERR_INVALID_STATE. The bus layer's own mapping is lossy in this
 * direction: an IDF code with no errno of its own came up as -EIO and goes
 * back as ESP_FAIL.
 */
static inline esp_err_t stepper_esp_err(int err)
{
	switch (err) {
	case 0:
		return ESP_OK;
	case -EINVAL:
		return ESP_ERR_INVALID_ARG;
	case -ENOMEM:
		return ESP_ERR_NO_MEM;
	case -ETIMEDOUT:
		return ESP_ERR_TIMEOUT;
	case -EOPNOTSUPP:
		return ESP_ERR_NOT_SUPPORTED;
	case -ENODEV:
	case -ENXIO:
		return ESP_ERR_NOT_FOUND;
	case -EMSGSIZE:
		return ESP_ERR_INVALID_SIZE;
	case -EBADMSG:
		return ESP_ERR_INVALID_CRC;
	case -EBUSY:
	case -ENODATA:
		return ESP_ERR_INVALID_STATE;
	default:
		return ESP_FAIL;
	}
}

#else /* STM32 HAL */
/*
 * Undefined, the include below fails with only
 * `#include expects "FILENAME" or <FILENAME>`. Every library source that
 * needs the HAL includes this header before any chip header, so one check
 * here names the cause on every build route. The HAL module checks sit in
 * its #else: without the family header no module looks enabled, and each of
 * them would blame a module instead.
 */
#ifndef CMSIS_device_header
#error "stepper: CMSIS_device_header is not defined; define it as the STM32 family header, \"stm32f4xx.h\" or \"stm32f7xx.h\" (CMake: set(MCU_DEVICE_HEADER stm32f4xx.h))"
#else
#include CMSIS_device_header

#if !defined(HAL_UART_MODULE_ENABLED) && CONFIG_STEPPER_TMC2209
#error "the TMC2209 driver needs the HAL UART module: #define HAL_UART_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_uart.c in the build"
#endif
#if !defined(HAL_SPI_MODULE_ENABLED) && CONFIG_STEPPER_TMC5240
#error "the TMC5240 driver needs the HAL SPI module: #define HAL_SPI_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_spi.c in the build"
#endif
#endif /* CMSIS_device_header */

#ifdef HAL_I2C_MODULE_ENABLED
typedef I2C_HandleTypeDef *stepper_i2c_bus;
#endif

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
#define STEPPER_HAVE_BUS
struct stepper_bus {
	const struct stepper_bus_ops *ops;
	uint32_t timeout_ms;
	union {
#ifdef HAL_I2C_MODULE_ENABLED
		struct {
			I2C_HandleTypeDef *handle;
			uint8_t address;	/* 7-bit */
		} i2c;
#endif
#ifdef HAL_SPI_MODULE_ENABLED
		struct {
			SPI_HandleTypeDef *handle;
			GPIO_TypeDef *cs_port;	/* app's output, idle high */
			uint16_t cs_pin;
		} spi;
#endif
#ifdef HAL_UART_MODULE_ENABLED
		UART_HandleTypeDef *uart;
#endif
	};
};
#endif

/* An output pin the app has set up: the chip's enable. */
struct stepper_pin {
	GPIO_TypeDef *port;	/* NULL: not wired */
	uint16_t pin;
};

/*
 * HAL_StatusTypeDef -> negative errno, at the bus boundary and nowhere else.
 * HAL_BUSY -- the bus or the handle in use, or a handle never initialised --
 * is -EAGAIN, not -EBUSY, which the TMC5240 keeps for "refused while
 * moving". A NACK in the middle of a transfer is HAL_ERROR like any other
 * fault, so -EIO.
 */
static inline int stepper_hal_errno(HAL_StatusTypeDef status)
{
	switch (status) {
	case HAL_OK:
		return 0;
	case HAL_TIMEOUT:
		return -ETIMEDOUT;
	case HAL_BUSY:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

#endif /* STM32 HAL */

/*
 * The chips' handle APIs return esp_err_t on ESP-IDF, as the original
 * drivers did, and 0 or a negative errno on the HAL. The drivers work in
 * errno and convert once, where each public handle function returns.
 */
#ifdef ESP_PLATFORM
typedef esp_err_t stepper_handle_ret;
#define STEPPER_HANDLE_RET(err)	stepper_esp_err(err)
#else
typedef int stepper_handle_ret;
#define STEPPER_HANDLE_RET(err)	(err)
#endif

#ifdef STEPPER_HAVE_BUS
/* bus->ops->exchange(), or -EOPNOTSUPP on a kind that has none. */
static inline int stepper_bus_exchange(struct stepper_bus *bus,
				       const uint8_t *tx, uint8_t *rx,
				       size_t len)
{
	if (!bus->ops->exchange)
		return -EOPNOTSUPP;
	return bus->ops->exchange(bus, tx, rx, len);
}

/* bus->ops->receive(), or -EOPNOTSUPP on a kind that has none. */
static inline int stepper_bus_receive(struct stepper_bus *bus, uint8_t *rx,
				      size_t rx_len)
{
	if (!bus->ops->receive)
		return -EOPNOTSUPP;
	return bus->ops->receive(bus, rx, rx_len);
}

static inline void stepper_bus_detach(struct stepper_bus *bus)
{
	bus->ops->detach(bus);
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
/*
 * Bind bus to the I2C device at address. On ESP-IDF the driver adds the
 * device at scl_hz, 0 for 400 kHz; on the HAL the clock is the handle's and
 * scl_hz is ignored. Undo with stepper_bus_detach(). Every attach leaves
 * bus->ops NULL when it fails, so a bus with ops is one with something to
 * detach. No chip here is on I2C yet.
 */
int stepper_i2c_attach(struct stepper_bus *bus, stepper_i2c_bus handle,
		       uint8_t address, uint32_t scl_hz, uint32_t timeout_ms);
#endif

#ifdef ESP_PLATFORM
/*
 * Add the SPI device on host's cs_pin, in SPI mode 0-3 at clock_hz. The
 * host is the app's, already spi_bus_initialize()d.
 */
int stepper_spi_attach(struct stepper_bus *bus, spi_host_device_t host,
		       int cs_pin, uint32_t clock_hz, uint8_t mode);

/* port's driver is the app's, already installed and configured. */
int stepper_uart_attach(struct stepper_bus *bus, uart_port_t port,
			uint32_t timeout_ms);
#else
#ifdef HAL_SPI_MODULE_ENABLED
/*
 * Bind bus to the SPI device behind cs_port/cs_pin, and deselect it, so the
 * first transfer starts on a falling edge. Mode, clock and frame are the
 * handle's; the chip select is an output the app has set up.
 */
int stepper_spi_attach(struct stepper_bus *bus, SPI_HandleTypeDef *handle,
		       GPIO_TypeDef *cs_port, uint16_t cs_pin,
		       uint32_t timeout_ms);
#endif
#ifdef HAL_UART_MODULE_ENABLED
/* handle is the app's, already HAL_UART_Init()ed. */
int stepper_uart_attach(struct stepper_bus *bus, UART_HandleTypeDef *handle,
			uint32_t timeout_ms);
#endif
#endif

/*
 * The enable pin. stepper_pin_wired() says whether there is one. On ESP-IDF
 * stepper_pin_output() makes the GPIO an output (gpio_config()), as both
 * original drivers did, and drives it; on the HAL the app has set it up and
 * this only drives it. Then stepper_pin_set() drives it high or low.
 */
bool stepper_pin_wired(const struct stepper_pin *pin);
int stepper_pin_output(const struct stepper_pin *pin, bool high);
void stepper_pin_set(const struct stepper_pin *pin, bool high);

/*
 * Polling a status bit until a deadline, the way the TMC5240 original did:
 * stepper_tick_now() starts the clock, stepper_expired() says whether
 * limit_ms has surely passed -- at once for 0, never for UINT32_MAX -- and
 * stepper_poll_wait() pauses between two polls.
 */
uint32_t stepper_tick_now(void);
bool stepper_expired(uint32_t start, uint32_t limit_ms);
void stepper_poll_wait(uint32_t ms);

/* The TMC2209's settle wait after power-up; see the port for its length. */
void stepper_sleep_ms(uint32_t ms);

#endif /* STEPPER_PRIV_H */
