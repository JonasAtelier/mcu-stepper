# API usage

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Handles and the category view

Each chip keeps the handle API its original driver had: `tmc2209_init()` /
`tmc5240_init()` fill a handle, every chip call takes it, and
`tmc2209_del()` / `tmc5240_del()` free it. The handle holds a
`struct stepper_dev`, and `tmc2209_stepper(h)` / `tmc5240_stepper(h)` hand
that out for the calls every chip shares:

| Call | Does |
|---|---|
| `stepper_enable(s, on)` | the enable pin, active low on both chips; `-EOPNOTSUPP` when it is tied |
| `stepper_set_current(s, run_a, hold_a)` | run and hold current, **RMS amperes**; hold 0 is half of run |
| `stepper_read_status(s, &st)` | the flags both chips report, decoded; clears the latched reset flag |
| `s->microsteps` | per full step, as init set it; read only |
| `stepper_log_status(s, &st)` | one info line: the flags that are set, and the current scale |

The `struct stepper_dev *` is valid from a successful init to the del; it is
never probed or removed on its own. A flag a chip does not have reads false:
the TMC2209 has no stall flag (it reports a stall on its DIAG pin). The
TMC2209's own API takes milliamps; `stepper_set_current()` takes amperes and
rounds to the milliamp.

Motion is per chip: `tmc2209_move()` for a velocity from the TMC2209's pulse
generator, `tmc5240_move_to()`, `_move_by()`, `_run()`, `_stop()` and
`_wait()` for the TMC5240's ramp. Each header documents its calls.

## Buses are the app's

As in every mcu-* library, the app creates and configures the bus; the
driver only uses it.

- **TMC2209 on ESP-IDF:** a `uart_port_t` whose driver the app has
  installed (`uart_driver_install()`), configured (`uart_param_config()`)
  and pinned (`uart_set_pin()`). The baud rate is the app's. Init refuses a
  port with no driver installed (`ESP_ERR_INVALID_ARG`); del leaves it
  installed. The RX buffer must hold a reply and its echo — 12 bytes — which
  any buffer the driver accepts does.
- **TMC2209 on the HAL:** a `UART_HandleTypeDef *` the app has
  `HAL_UART_Init()`ed as an ordinary full-duplex 8N1 USART. Blocking calls
  only; no interrupt or DMA receive may be running on that handle.
- **TMC5240 on ESP-IDF:** an SPI host the app has `spi_bus_initialize()`d and
  a CS GPIO; the driver adds its device in mode 3 at `spi_hz` (0: 4 MHz) and
  removes it at del, as esp-tmc5240-stepper did.
- **TMC5240 on the HAL:** an `SPI_HandleTypeDef *` in mode 3 with 8-bit
  frames, and a chip select as a GPIO port and pin, an output idle high.
  Init checks the mode and frame size and refuses a wrong handle with
  `-EINVAL` before it touches the bus.

**The enable pin is the driver's.** It is one output per chip, not a shared
bus, so on ESP-IDF the driver configures it itself with `gpio_config()` and
drives it, as both original drivers did. On the HAL the app sets it up as an
output and the driver only drives it. Both drivers hold the stage off
(ENN high) through init. The TMC2209 leaves it off — call `tmc2209_enable()`
or `stepper_enable()`; the TMC5240 turns it on once configured. Both turn it
off at del.

## Current

Both chips set current in RMS. The TMC2209's comes from its sense resistors
(`rsense_mohm`), the TMC5240's from the resistor on IREF (`rref_kohm`);
nothing can measure either, and if the value is wrong every current is wrong
by the same ratio. Each chip quantises to 1/32 of a range and picks the
finest range that fits; both log what the chip will actually drive. Steppers
are rated in **peak** amps: a motor rated 1.5 A peak wants about 1.06 A RMS.
See [the chip reference](chips.md) for the formulas.

## Timing and threads

**ESP-IDF.** The TMC2209 waits 10 ms after taking the UART, with
`vTaskDelay(pdMS_TO_TICKS(10))` as esp-tmc2209-stepper did — at the default
100 Hz tick that is one tick, anything up to 10 ms. Its UART timeouts are
20 ms, rounded up to whole ticks plus one: never short. The TMC5240 polls
RAMP_STAT every 5 ms, at least one tick, and its timeouts round up to whole
ticks plus one; `UINT32_MAX` waits forever.

**HAL.** Waits are `HAL_Delay()`, which already adds one tick, so never
short. A wait of `timeout_ms` gives up only when strictly more than that has
passed on `HAL_GetTick()`. Call from thread context, never from an interrupt
at or above SysTick's priority.

**Locking.** On ESP-IDF a TMC5240 handle can be shared between tasks: every
register access runs under its mutex, held across every datagram of a read
chain, so a read is never answered with another task's data. A motion
command is several writes and is not atomic as a whole: command motion from
one task. On the HAL there is no lock: call each TMC5240 from one context.
The TMC2209 has no lock on either platform: one task per chip, or wrap it in
your own mutex; several chips on one wire share the UART, so they share that
mutex too.

## Logging

On ESP-IDF everything goes under the log tag `stepper`, prefixed with the
chip name, and `stepper_dbg()` is `ESP_LOGD`. On the HAL every line goes
through `printf()` as `stepper: <chip>: ...`. Your own code can use the same
`stepper_err()`, `stepper_warn()`, `stepper_info()` and `stepper_dbg()` from
`stepper_log.h`. `stepper_dbg()` is compiled out unless
`CONFIG_STEPPER_DEBUG` is 1 (stm-forge: `STEPPER_DEBUG = y`).

## Errors

`stepper_*` calls return 0 or a negative errno on both platforms. The chip
calls return `esp_err_t` on ESP-IDF, as the original drivers did, and errno
on the HAL:

| Meaning | HAL | ESP-IDF |
|---|---|---|
| bad argument or config, NULL handle, value past its register | `-EINVAL` | `ESP_ERR_INVALID_ARG` |
| no memory for the handle | `-ENOMEM` | `ESP_ERR_NO_MEM` |
| TMC2209: no reply, or no echo of what was sent | `-ETIMEDOUT` | `ESP_ERR_TIMEOUT` |
| TMC2209: a reply that is not ours or fails its CRC | `-EBADMSG` | `ESP_ERR_INVALID_CRC` |
| TMC2209: replies come but writes do not land (IFCNT) | `-ENODEV` | `ESP_ERR_NOT_FOUND` |
| TMC5240: INP_OUT.VERSION is not 0x40 | `-ENODEV` | `ESP_ERR_NOT_FOUND` |
| TMC5240: a ramp or move that does not finish in time | `-ETIMEDOUT` | `ESP_ERR_TIMEOUT` |
| TMC5240: `set_position()` while moving | `-EBUSY` | `ESP_ERR_INVALID_STATE` |
| TMC5240: the chip flags its ADC (INP_OUT.ADC_ERR) | `-ENODATA` | `ESP_ERR_INVALID_STATE` |
| enable pin tied | `-EOPNOTSUPP` | `ESP_ERR_NOT_SUPPORTED` |
| bus busy (HAL_BUSY) | `-EAGAIN` | — |
| other bus failure | `-EIO` | `ESP_FAIL` |

On ESP-IDF a bus error passes through errno on its way up, and the map is
lossy in that direction: an IDF code with no errno of its own (for example
`ESP_ERR_INVALID_STATE` from an SPI host never initialised) comes back as
`ESP_FAIL`. `strerror(-ret)` names any errno.

## Changes from esp-tmc2209-stepper

The ESP-IDF API changes in one place, deliberately: **the driver no longer
installs or configures the UART**. Every other mcu-* library takes a bus the
app owns, and esp-tmc2209-stepper had no users to keep.

Removed from `tmc2209_config_t`:

| Removed | Now |
|---|---|
| `tx_gpio` | `uart_set_pin()` in the app |
| `rx_gpio` | `uart_set_pin()` in the app |
| `baud` (0 selected 115200) | `uart_param_config()` in the app |

`uart` stays, and now names a port the app has installed. Nothing was
renamed. Behaviour that follows:

- `tmc2209_init()` returns `ESP_ERR_INVALID_ARG` for a port with no driver
  installed, instead of installing one; it never calls
  `uart_driver_install()`, `uart_param_config()` or `uart_set_pin()`.
- `tmc2209_del()` and a failed init no longer call `uart_driver_delete()`.
- A single-wire echo that does not come back in full is now
  `ESP_ERR_TIMEOUT`; the original read whatever came and went on. When
  nothing comes back at all, init fails with `ESP_ERR_TIMEOUT` as before;
  a write that gets no echo now fails where it used to pass unnoticed.
- The UART timeouts are 20 ms rounded up to whole ticks plus one (3 ticks
  at 100 Hz), not `pdMS_TO_TICKS(20)` (2 ticks).
- Log lines go under the tag `stepper` as `tmc2209: ...`, not under
  `tmc2209`.
- **Upstream bug fixed:** `tmc2209_set_stall()` converted the TCOOLTHRS
  period from float to `uint32_t` before clamping it to 20 bits. For a
  very small `min_usteps_per_s` the period passes 2^32, and a NaN passes the
  `<= 0` check, and the cast is undefined for both. It now clamps while
  still a float: both give the 20-bit maximum, 0xFFFFF. Every value that
  was in range before comes out the same.

Added: `tmc2209_stepper()` and the `stepper_*` calls, and the STM32 port.
`tmc2209_status_t` and every function signature are unchanged.

## Changes from esp-tmc5240-stepper

The ESP-IDF header is token-for-token the original's, plus
`tmc5240_stepper()` and the `stepper.h` it includes. One behaviour change is
intended:

- **Acceleration is scaled by 2^42, not 2^41.** The TMC5240 datasheet's
  Table 19 (p.49) gives `a[Hz/s] = a[TMC5240] * fCLK^2 / 2^42`; the
  original used 2^41, the older TMC5130/5160 formula. Every AMAX, A1, A2,
  DMAX, D1 and D2 now holds twice the register value it did for the same
  `tmc5240_ramp_t`, and the original's own math test changed with it (two
  lines of `test/test_tmc5240_math.c`). If the original was right about
  the silicon, ramps now take half as long as asked. It has not been
  measured; [the chip reference](chips.md#acceleration-scale) says how.

Otherwise:

- Log lines go under the tag `stepper` as `tmc5240: ...`, not under
  `tmc5240`.
- Errors pass through errno inside the driver, so an SPI error with no errno
  of its own (see [Errors](#errors)) comes back as `ESP_FAIL` rather than
  the original IDF code. The driver's own codes are unchanged.
- `TMC5240_AACTUAL` (0x31) and `TMC5240_AACTUAL_BITS` are gone from the
  private headers: neither is in the datasheet, and nothing used them.
