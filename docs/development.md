# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Repository layout

```
include/                    public API, the only directory apps include
  stepper.h                 struct stepper_dev, stepper_enable/set_current/read_status
  stepper_log.h             stepper_err/warn/info/dbg, stepper_log_status()
  tmc2209.h tmc5240.h       one header per chip: its handle API and tmcXXXX_stepper()
src/                        private to the library
  stepper.c                 the core: checks, then the chip's op
  stepper_log.c             stepper_log_status()
  stepper_priv.h            bus ops, attach functions, enable pin, waits, errno maps
  stepper_port_esp.c        the bus layer, pin and waits on ESP-IDF; empty elsewhere
  stepper_port_stm.c        the same on the STM32 HAL; empty under ESP-IDF
  tmc2209/                  tmc2209.c  tmc2209_regs.h  tmc2209_math.h
  tmc5240/                  tmc5240.c  tmc5240_regs.h  tmc5240_math.h  tmc5240_priv.h
test/                       host tests, `make -C test test`
  test_<chip>_math.c        maths tests: no bus and no platform API
  core_cases.h              core cases, shared by esp/ and stm/
  tmc2209_cases.h           a TMC2209 at datagram level and its cases, shared
  tmc5240_cases.h           a TMC5240 at frame level and its cases, shared
  esp/                      suites on idf_stubs/, through stepper_port_esp.c,
                            esp-tmc5240-stepper's own test_tmc5240.c among them
  stm/                      suites on hal_stubs/, through stepper_port_stm.c
  idf_stubs/  hal_stubs/    the slice of ESP-IDF and of the HAL the code calls
examples/
  esp/<chip>/               ESP-IDF projects: tmc2209, tmc5240
  stm/<chip>/               STM32 snippets: tmc2209, tmc5240
docs/                       the five guides; datasheet PDFs here are gitignored
assets/                     README and wiring figures
CMakeLists.txt              ESP-IDF component; elsewhere a static library for add_subdirectory(); stm-forge globs src/**/*.c instead
                            the static library's options are Kconfig's bool entries, found by the type line directly under each `config`
Kconfig                     STEPPER and STEPPER_<CHIP>, for ESP-IDF and both forges' menus
idf_component.yml           ESP-IDF component manifest
```

Naming inside `src/<chip>/`, the same in every category library:

| File | Holds |
|---|---|
| `<chip>.c` | the driver: init, del, the ops, the chip's calls; may split into `<chip>_<part>.c` |
| `<chip>_regs.h` | register map: addresses, bit fields, values, each with its datasheet citation |
| `<chip>_proto.h` | serial protocol: command bytes, frames, checksums, for parts without registers |
| `<chip>_math.h` | pure conversions, compiled and asserted on the host by `test/<chip>_math` tests |
| `<chip>_priv.h` | driver state, only if needed |

A chip has `_regs.h` or `_proto.h`, never both; `_math.h` and `_priv.h` only
when it needs them. The TMC2209 has registers behind its UART datagrams, so
`tmc2209_regs.h` holds the datagram framing as well, and its CRC lives in
`tmc2209_math.h` with the other pure functions. The chip drivers keep the
originals' four-space style; the library's own files use tabs.

## Handles first

The other category libraries are probe-first: the app owns a zeroed
`struct <cat>_dev`, a chip's probe binds it, and the chip's extras take that
`<cat>_dev`. Here the chips' calls already had public names taking a handle
(`tmc5240_move_to(h, ...)`), and the ESP-IDF headers had to stay as they
were. So the handle stays the device: `struct tmc2209_t` and
`struct tmc5240_t` each embed a `struct stepper_dev`, init fills it and
points `priv` back at the handle, and `tmc2209_stepper()` /
`tmc5240_stepper()` hand it out. There is no second probe path and no
renamed call. The common part is what both chips really share — enable,
run/hold current, status — and motion stays per chip.

## Platform ports

The drivers in `src/` are the same C on both platforms; only the bus, the
enable pin and the waits under them differ, in `src/stepper_port_esp.c`
(ESP-IDF `i2c_master`, `spi_master`, UART driver, GPIO, FreeRTOS ticks) and
`src/stepper_port_stm.c` (HAL). Each port is empty on the other platform,
chosen by `ESP_PLATFORM`. Configs take the platform's own handles: on
ESP-IDF the originals', and on the HAL a `UART_HandleTypeDef *`, an
`SPI_HandleTypeDef *` with a CS port and pin, and the enable pin as a port
and pin.

The two things the drivers still do per platform, behind `#ifdef
ESP_PLATFORM`:

- **Return codes.** The handle calls return `esp_err_t` on ESP-IDF and errno
  on the HAL. The drivers work in errno; `STEPPER_HANDLE_RET()` converts once
  where each public call returns (`stepper_esp_err()`, in
  `src/stepper_priv.h`).
- **The TMC5240's lock.** A FreeRTOS mutex on ESP-IDF, as the original had
  and its test asserts; nothing on the bare-metal HAL.

The waits keep each original's behaviour on ESP-IDF exactly:
`stepper_expired()` is the TMC5240's deadline (whole ticks plus one,
saturating, 0 for one look) and `stepper_poll_wait()` its at-least-one-tick
poll. `stepper_sleep_ms()` is the TMC2209's settle wait, kept as its
truncating `vTaskDelay(pdMS_TO_TICKS(ms))` — at 100 Hz, 10 ms is one tick,
which may end at once. It is a candidate to round up like the bus
timeouts; it was left as the original had it.

## Bus abstraction

Under the chips sits a regmap-style ops table, `struct stepper_bus_ops` in
`src/stepper_priv.h`: each chip holds one `struct stepper_bus` and talks to
it only through its ops. The kind — I2C, SPI or UART — is picked at run time
by which attach init calls. The TMC2209 is on a UART, the TMC5240 on SPI;
I2C is there because every category library offers all three kinds, and
`test/{esp,stm}/test_port_*` checks it.

- `transfer(cmd, tx, rx)` is the register access, as in the other category
  libraries. The TMC2209 uses it on two wires.
- `exchange(tx, rx, len)` is len bytes out while len come back, `rx[i]`
  answering `tx[i]`. On SPI it is one full-duplex frame under one chip
  select, from mcu-encoder: the TMC5240's 40-bit datagram, whose answer
  belongs to the previous one. On a UART it is new here: send, and collect
  what the line gives back meanwhile — the TMC2209's echo. NULL on I2C.
- `receive(rx, n)` is n bytes with nothing sent and nothing dropped: the
  reply that follows the echo. UART only.

`stepper_bus_exchange()` and `stepper_bus_receive()` say `-EOPNOTSUPP` on a
kind that has neither.

### The TMC2209's echo

On one wire the MCU receives every byte it sends. On ESP-IDF that is easy:
the UART driver keeps all of it in its RX ring buffer, so `exchange()`
flushes, writes, waits for the last bit to leave and reads back as many
bytes as it wrote — what esp-tmc2209-stepper did.

On the HAL, blocking `HAL_UART_Transmit()` of a whole datagram never reads
the receiver while it sends. The data register holds one byte, so the second
echoed byte overruns the first (ORE), the rest are lost, and the reply after
them comes up misaligned. `exchange()` therefore sends one byte with
`HAL_UART_Transmit()` and takes its echo with `HAL_UART_Receive()` before the
next goes out: only one byte is ever in flight. `test/stm/test_port_stm.c`
models the data register and checks it: an 8-byte datagram through
`exchange()` never overruns, the same datagram as one `transfer()` does.

The alternative was a half-duplex USART (`HAL_HalfDuplex_Init()`, HDSEL),
deaf while it sends, with `HAL_HalfDuplex_EnableTransmitter()` /
`EnableReceiver()` around each datagram. It was not used: it makes the
app's handle setup part of the driver's contract, the driver cannot check
from a handle that it was done, it needs a different op anyway, and the
plain full-duplex wiring the breakouts already use works without it.

## Adding a chip

A driver is an init, a del, and three ops:

1. `include/foo.h`: the chip's handle API, with the platform's handle types
   behind `#ifdef ESP_PLATFORM`, and `struct stepper_dev *foo_stepper(foo_handle_t)`.
2. `src/foo/foo.c`, in a folder of its own with its private headers
   (`foo_regs.h`, ...). Include siblings and `stepper_priv.h` by plain name,
   never with `../`. Wrap the whole driver in `#if CONFIG_STEPPER_FOO` (never
   `#ifdef`); the HAL module check goes in `src/stepper_priv.h`. Keep a
   `struct stepper_bus` and a `struct stepper_pin` in the handle and talk to
   the chip only through the bus ops and the pin helpers. Give it a
   `static const struct stepper_ops foo_ops = { .enable, .set_current, .read_status };`.
3. In init: set `stepper.name` first so failures log with it, refuse a bad
   config before allocating, allocate, hold the stage off, attach the bus,
   check the chip answers, configure, and only on success set
   `stepper.ops`, `.priv` and `.microsteps`. Undo everything on failure.
   Work in errno and return `STEPPER_HANDLE_RET(err)` from the public calls.
4. Add `CONFIG_STEPPER_FOO` to the no-chip checks in `CMakeLists.txt` (its
   ESP-IDF part; the `add_subdirectory()` part reads `Kconfig`) and
   `src/stepper_port_stm.c` and a HAL module `#error` to
   `src/stepper_priv.h`, a `src/foo/foo.c` line to `CMakeLists.txt`, a
   `config STEPPER_FOO` bool to `Kconfig`, `# STEPPER_FOO is not set` to
   both forges' defconfigs, and `test/esp/test_foo.c` and
   `test/stm/test_foo.c` (and `test/test_foo_math.c` for its maths header)
   targets to `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL or ESP-IDF needed. `test/esp/` runs through
`src/stepper_port_esp.c` and `test/idf_stubs/`, `test/stm/` through
`src/stepper_port_stm.c` and `test/hal_stubs/`. Nothing is checked against
real silicon.

- **esp-tmc5240-stepper's own tests** (fe3530d) run as
  `test/esp/test_tmc5240.c`, its `test_driver.c` unchanged, and
  `test/test_tmc5240_math.c`, its `test_math.c` with two kinds of change:
  the include paths (`../src/tmc5240/`), and the one intended deviation, the
  Table 19 check at 2^42 — `12.5e6 * 12.5e6 / 4398046511104.0` and 35.527
  where it had 2^41's `2199023255552.0` and 71.054. Its stubs are merged
  into `test/idf_stubs/`; the stub tick rate and the FreeRTOS calls there
  are weak, so the original test's own definitions win.
- esp-tmc2209-stepper had no tests; all the TMC2209's are new.
- **This library's own cases** are written once, in `test/*_cases.h`,
  against a fake that knows nothing of the driver; `test/esp/` and
  `test/stm/` feed that fake from their own platform's calls and add the
  checks only that platform has.

| Binary | Checks |
|---|---|
| `test_tmc2209_math` | the CRC against the textbook CRC-8 over 10,000 random datagrams; the section 9 current formula — full scale, the range switch, every current from 0.1 to 1.7 A within half a step, clamping, the hold CS moved between ranges; MRES; VACTUAL scale and its ±(2^23)−1 edge; TCOOLTHRS and its 20-bit clamp |
| `test_tmc5240_math` | esp-tmc5240-stepper's: Table 17 currents, Table 19 conversions (at 2^42), sign extension, field widths and clamping, refused inputs, MRES |
| `esp/test_tmc5240` | esp-tmc5240-stepper's: the read pipeline, the mutex around every datagram chain, probe, stopping a motor still turning at init, config refusals, current, microsteps, direction, ramps all-or-nothing, motion, stop_hard's AMAX, move_by wrapping, set_position, wait at 100 and 1000 Hz ticks, status, ADC, StallGuard, NULLs, SPI_STATUS |
| `esp/test_core`, `stm/test_core` | `stepper_*` dispatch, errors passed through, a failed read leaving the status alone, NULL and missing ops refused without a call, log prefix and `stepper_log_status()` with no flag, some, and all |
| `esp/test_tmc2209`, `stm/test_tmc2209` | a fake TMC2209 parsing datagrams, with GSTAT write-1-to-clear and IFCNT: init's GCONF and CHOPCONF read-modify-writes keeping other bits, TPOWERDOWN, SENDDELAY, IHOLDDELAY, the stage left off; spreadCycle, shaft, MRES, address straps; two wires; a write datagram byte for byte, reads back to back, bad CRC, silence, a refused send; init with no answer, bad CRCs, writes not landing (IFCNT), IFCNT wrapping, config refusals, no memory, nothing leaked; currents through both APIs and both ranges, amperes refused out of range or NaN; VACTUAL both ways, out of range, an external clock; ENN; status and the category status, reset reported once; SG_RESULT, TCOOLTHRS, disarm; NULLs. ESP-IDF: a port with no driver installed refused, the UART never deleted. HAL: a NULL handle refused, and no echo ever overrunning the data register |
| `esp/test_tmc5240_shared`, `stm/test_tmc5240` | a fake TMC5240 at frame level: init, two frames a read, the category view; wrong VERSION, a bus failure, config refusals before allocation, a current out of reach, nothing leaked; a turning motor ramped down, a ramp that never stops; **AMAX and DMAX at 2^42 as literal register values**; enable, current and status through the category, GSTAT cleared once, a failed read leaving the status alone; `-EBUSY`, `-ENODATA` and `-ETIMEDOUT` (and their IDF codes); SPI_STATUS. ESP-IDF: mode 3 at 4 MHz or `spi_hz`, the device removed again, also after a failed init. HAL: a handle not in mode 3 or not 8-bit, or a NULL handle or CS port, refused before anything; a chip select around every frame, high again after a failure |
| `esp/test_port_esp`, `stm/test_port_stm` | the I2C, SPI and UART ops of each port. I2C: attach, read, write, a bare command, tx-then-rx refused, failures mapped, no `exchange()` or `receive()`. SPI: `transfer()` as in mcu-encoder; `exchange()` of two bytes and of the TMC5240's five, bounds, failures with rx untouched, CS high again; no `receive()`. UART: `transfer()` as in mcu-encoder; `exchange()` — on ESP-IDF flush, write, wait for the last bit, read the echo, with the 20 ms timeout at 3 ticks; on the HAL a byte out and its echo in, eight times, never overrunning, against one whole transmit that does — short echoes and failures mapped, bounds; `receive()` with nothing flushed first. The enable pin, and the waits: 0, rounding, wrap, `UINT32_MAX`, at least one tick, `HAL_Delay()`'s extra tick, the TMC2209's truncating settle wait |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic` with
UBSan (`-fsanitize=undefined -fno-sanitize-recover=all`), and `-Wl,--wrap`
(GNU ld) to count allocations and capture `printf()`. esp-tmc5240-stepper's
tests built under `-std=c11`; the `stepper_*` log macros need gnu11's
`, ##__VA_ARGS__`.
