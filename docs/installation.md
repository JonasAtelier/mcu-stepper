# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-stepper`; the forge package key, C API and
configuration options are `stepper`. The clone command below names the
ESP-IDF component directory `stepper`. It replaces two components,
`esp-tmc2209-stepper` and `esp-tmc5240-stepper` (forge keys `tmc2209` and
`tmc5240`): a project that `REQUIRES` either now `REQUIRES stepper`.

## With forge

stm-forge must be at `ee921c3` or later ("build: libraries may keep sources
in src/ subfolders"); older ones only compile `src/*.c`, so the chip drivers
in `src/<chip>/` are never built and the link fails with undefined
references to the chip functions. esp-forge is unaffected: the component's
CMakeLists lists its sources.

With [esp-forge](https://github.com/JonasAtelier/esp-forge) or
[stm-forge](https://github.com/JonasAtelier/stm-forge). The catalog line in
`profiles/pkgversion` is the same for both:

```make
stepper_GIT=https://github.com/JonasAtelier/mcu-stepper@stable
```

and a target turns the library and its chips on in `configs/<target>_defconfig`:

```make
PKG_stepper         = y
STEPPER_TMC2209     = y
# STEPPER_TMC5240 is not set
# STEPPER_DEBUG is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_STEPPER_<CHIP>=y`
in `sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_STEPPER_<CHIP>=1`.
`STEPPER_DEBUG = y` turns on `stepper_dbg()` lines on the HAL; on ESP-IDF
the log level decides.

## ESP-IDF

Clone into your project's `components/` directory. A local component is
named after its directory, and that name is what you `REQUIRES`:

```sh
git clone https://github.com/JonasAtelier/mcu-stepper components/stepper
```

```cmake
idf_component_register(SRCS "main.c" REQUIRES stepper driver)
```

Enable the chips under `idf.py menuconfig` → **Component config → Stepper
driver subsystem**, or put `CONFIG_STEPPER_<CHIP>=y` in your
`sdkconfig.defaults` as each example in `examples/esp/` does. With the
subsystem on and no chip picked, the build warns. ESP-IDF 5.2 or newer is
required.

## STM32 HAL

Copy `include/` and `src/` into an STM32 project, compile every `.c` under
`src/`, chip folders included, add `include/` and `src/` to the include path
(`src/` only needs to reach this library's own sources), and define what the
forge would: `CMSIS_device_header` as your family header (`"stm32f4xx.h"` or
`"stm32f7xx.h"`), `USE_HAL_DRIVER`, and `CONFIG_STEPPER_<CHIP>=1` for each
chip you want.

**No chip driver is built by default.** The core is always built and each
driver is opt-in: a chip whose option is absent or 0 compiles to nothing.
With the library on and no chip picked, the ESP-IDF CMakeLists warns and
`src/stepper_port_stm.c` emits a `#warning`.

**The HAL modules are yours to enable.** Your app's `stm32f4xx_hal_conf.h`
(or `stm32f7xx_hal_conf.h`) must define the module each chip uses, and its
`.c` must be in the build:

| Option | Needs in hal_conf | HAL source |
|---|---|---|
| `STEPPER_TMC2209` | `HAL_UART_MODULE_ENABLED` | `stm32f{4,7}xx_hal_uart.c` |
| `STEPPER_TMC5240` | `HAL_SPI_MODULE_ENABLED` | `stm32f{4,7}xx_hal_spi.c` |

A chip whose module is off stops the build with an `#error` in
`src/stepper_priv.h` that says which define is missing. The bus layer's I2C
code sits under `HAL_I2C_MODULE_ENABLED` and builds to nothing without it.
Both drivers drive their enable pin with `HAL_GPIO_WritePin()`.

**printf() is the log.** Lines go out through `printf()`, so retarget
newlib's `_write()` to a UART. stm-forge links newlib-nano
(`--specs=nano.specs`), whose `printf()` drops `%f` unless the app links with
`-u _printf_float`: add `target_link_options(firmware PRIVATE -u _printf_float)`
to your app's `CMakeLists.txt`, or the current lines print blanks where the
numbers go.
