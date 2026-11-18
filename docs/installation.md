# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-stepper`; the forge package key, C API and
configuration options are `stepper`. The clone command below names the
ESP-IDF component directory `stepper`. It replaces two components,
`esp-tmc2209-stepper` and `esp-tmc5240-stepper` (forge keys `tmc2209` and
`tmc5240`): a project that `REQUIRES` either now `REQUIRES stepper`.

Three step-by-step guides come first: [ESP-IDF](#esp-idf-step-by-step),
[STM32 with CMake](#with-cmake-a-cubemx-cmake-project) and
[STM32 without CMake](#without-cmake-stm32cubeide-a-makefile). None
needs a forge; if you build with esp-forge or stm-forge, see
[With forge](#with-forge). The steps use the TMC2209; for the TMC5240,
change the chip name. The reference follows the guides.

## ESP-IDF, step by step

**You need** ESP-IDF 5.2 or newer, with `idf.py` working in your terminal, and
an ESP-IDF project: a folder with a `CMakeLists.txt` and a `main/` folder.
`idf.py create-project my-project` makes one.

ESP-IDF calls a library a **component**: a folder with its own
`CMakeLists.txt` that the build finds and compiles. This library is the
component `stepper`.

1. **Get the library.** Pick one way.

   - **Component manager**, the simplest: the first build downloads the
     library into `managed_components/stepper`. Create
     `main/idf_component.yml`:

     ```yaml
     dependencies:
       stepper:
         git: https://github.com/JonasAtelier/mcu-stepper.git
         version: stable
     ```

   - **Clone**, if you want to edit the library or build offline. In the
     project folder:

     ```sh
     git clone https://github.com/JonasAtelier/mcu-stepper components/stepper
     ```

     Keep the folder name `stepper`: ESP-IDF names a component after its
     folder.

   ```
   my-project/
   ├── CMakeLists.txt
   ├── sdkconfig.defaults       step 2
   ├── main/
   │   ├── CMakeLists.txt       step 3
   │   ├── idf_component.yml    step 1, component manager
   │   └── my-project.c         step 4
   └── components/
       └── stepper/             step 1, clone
   ```

2. **Turn your chip on.** No chip driver is built until you do. Create
   `sdkconfig.defaults` in the project folder, a text file of settings you
   choose, with one line per chip:

   ```ini
   CONFIG_STEPPER_TMC2209=y
   ```

   The other chip is `CONFIG_STEPPER_TMC5240`. ESP-IDF copies these lines
   into `sdkconfig`, the project's full settings file, only when it creates
   that file: **if `sdkconfig` already exists, delete it** so your line is
   read. `idf.py menuconfig` → **Component config → Stepper driver
   subsystem** does the same from a menu.

3. **Let your code see the library.** A new project's `main/CMakeLists.txt`
   has no `REQUIRES` and already sees every component. If yours has a
   `REQUIRES` list, add `stepper` and `driver` to it:

   ```cmake
   idf_component_register(SRCS "main.c" REQUIRES stepper driver)
   ```

4. **Write the smallest test.** Copy
   [examples/esp/tmc2209/main/main.c](../examples/esp/tmc2209/main/main.c)
   over your main source file and set `TX_GPIO`, `RX_GPIO` and `EN_GPIO` to
   your wiring ([hardware](hardware.md) shows the single-wire UART), and
   `rsense_mohm` and the currents to your board and motor.
   [API usage](usage.md) explains the calls.

5. **Build.**

   ```sh
   idf.py set-target esp32    # your chip: esp32s3, esp32c3, ...
   idf.py build
   ```

   A good build ends with `Project build complete.` Flash it as any ESP-IDF
   app and open the serial monitor: with the example's 110 mOhm and
   600/200 mA, the driver's first line ends in
   `stepper: tmc2209: run 611 mA (IRUN=19), hold 214 mA (IHOLD=6), vsense=1`.

**If it fails**

| The build prints | Why | Fix |
|---|---|---|
| `Failed to resolve component 'stepper' required by component 'main': unknown name.` | The clone is not in a folder named `stepper` | Rename it to `components/stepper` |
| `fatal error: stepper.h: No such file or directory` | `main/CMakeLists.txt` has a `REQUIRES` list without `stepper` | Step 3 |
| `stepper: CONFIG_STEPPER is on but no chip driver is selected` (a CMake warning) | No chip turned on | Step 2 |
| ``undefined reference to `tmc2209_init'`` | That chip is not turned on, often because `sdkconfig` existed before you edited `sdkconfig.defaults` | Delete `sdkconfig`, build again |

## STM32, step by step

**You need** a project for an STM32F4 or F7 that uses ST's **HAL**, the
vendor library behind `HAL_Init()`, `HAL_UART_Init()` and friends, with the
UART (or SPI) your chip is wired to already set up. STM32CubeMX makes such a
project; with **Toolchain / IDE: CMake** it makes a CMake one, the main case
below.

Two terms. A **HAL module** is the part of the HAL for one peripheral: I2C,
SPI, UART. It is compiled only when it is switched on in **hal_conf**, the
file `Core/Inc/stm32f4xx_hal_conf.h` (`stm32f7xx_hal_conf.h` on an F7).
CubeMX switches a module on when you enable that peripheral.

### With CMake (a CubeMX CMake project)

1. **Get the library.** In the project folder, next to the top-level
   `CMakeLists.txt`:

   ```sh
   git clone https://github.com/JonasAtelier/mcu-stepper
   ```

   ```
   my-board/
   ├── CMakeLists.txt                  step 2
   ├── cmake/stm32cubemx/              made by CubeMX
   ├── Core/
   │   ├── Inc/stm32f4xx_hal_conf.h    hal_conf, step 3
   │   └── Src/main.c                  step 4
   ├── Drivers/                        the HAL, copied in by CubeMX
   └── mcu-stepper/                    step 1
   ```

2. **Add it to the build.** In the top `CMakeLists.txt`, after the line
   `add_subdirectory(cmake/stm32cubemx)`, add:

   ```cmake
   set(CONFIG_STEPPER_TMC2209 ON)      # your chip; one line per chip
   set(MCU_HAL_TARGET stm32cubemx)
   set(MCU_DEVICE_HEADER stm32f4xx.h)  # stm32f7xx.h on an F7
   add_subdirectory(mcu-stepper)
   ```

   and add `stepper` to the link line CubeMX already wrote, so it reads:

   ```cmake
   target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx stepper)
   ```

   Do not add `PRIVATE`: CubeMX's line has no such keyword, and CMake refuses
   to mix the two forms on one target.

   `stm32cubemx` is a CMake *target*, a named bundle of settings, that CubeMX
   makes to carry the HAL's include folders and defines; `MCU_HAL_TARGET`
   lends those to the library, which works with any CubeMX version.
   `MCU_DEVICE_HEADER` is your chip family's main header.

3. **Check the HAL module.** The TMC2209 talks UART, so hal_conf must have
   `#define HAL_UART_MODULE_ENABLED`; enabling USART2 (or any U(S)ART) in
   asynchronous mode in CubeMX does that. Set PB0, the chip's ENN in this
   test, as a GPIO output too. The [HAL module table](#stm32-hal) lists what
   every chip needs.

4. **Write the smallest test.** It is
   [examples/stm/tmc2209/main.c](../examples/stm/tmc2209/main.c) fitted into
   CubeMX's `Core/Src/main.c`. Paste each part between the `USER CODE BEGIN`
   and `END` comments named, so a regenerate keeps it.

   After `/* USER CODE BEGIN Includes */`:

   ```c
   #include "stepper.h"
   #include "stepper_log.h"
   #include "tmc2209.h"
   ```

   After `/* USER CODE BEGIN 2 */`, which comes after
   `MX_USART2_UART_Init()`:

   ```c
     /* As in examples/stm/tmc2209/main.c: ENN on PB0, 16 microsteps. */
     const tmc2209_config_t cfg = {
       .uart = &huart2,
       .address = 0,           /* MS1 and MS2 both low */
       .en_port = GPIOB,
       .en_pin = GPIO_PIN_0,
       .rsense_mohm = 110.0f,  /* check your board */
       .run_current_ma = 600,  /* RMS; start below the motor's rating */
       .hold_current_ma = 200,
       .microsteps = 16,
       .interpolate = true,
     };
     tmc2209_handle_t drv;
     struct stepper_dev *motor;
     int dir = 1;

     /* -ETIMEDOUT: nothing answers; check the wire and the address. */
     if (tmc2209_init(&cfg, &drv))
       Error_Handler();
     motor = tmc2209_stepper(drv);
     stepper_enable(motor, true);
   ```

   After `/* USER CODE BEGIN 3 */`, inside the `while (1)` loop:

   ```c
       /* One turn a second (200 full steps x 16) for two seconds, then
        * back the other way. */
       tmc2209_move(drv, dir * (float)(200 * 16));
       dir = -dir;
       for (int i = 0; i < 10; i++) {
         struct stepper_status st;

         HAL_Delay(200);
         if (stepper_read_status(motor, &st))
           continue;
         stepper_log_status(motor, &st);
         if (st.overtemp || st.short_to_gnd || st.short_to_supply) {
           tmc2209_stop(drv);
           stepper_enable(motor, false);
           Error_Handler();
         }
       }
   ```

   The lines go out through `printf()`: send that to a UART other than the
   chip's by writing newlib's `_write()`. [API usage](usage.md) explains the
   calls.

5. **Build** as before, for example `cmake --build` with your build folder.
   A good build shows a `Linking C executable` line and no error. On the
   UART, the driver's first line reads
   `stepper: tmc2209: run 611 mA (IRUN=19), hold 214 mA (IHOLD=6), vsense=1`.

**If it fails**

| You see | Why | Fix |
|---|---|---|
| `stepper: no chip driver is selected; set(CONFIG_STEPPER_<CHIP> ON) before add_subdirectory()` (a CMake warning), then ``undefined reference to `tmc2209_init'`` | No chip turned on, or turned on after `add_subdirectory(mcu-stepper)` | Step 2 |
| `#error "the TMC2209 driver needs the HAL UART module: ...` | The HAL module is off | Enable the peripheral in CubeMX and regenerate |
| `#error "stepper: CMSIS_device_header is not defined; ...` | `MCU_DEVICE_HEADER` is missing | Step 2 |
| `fatal error: stm32f4xx.h: No such file or directory` | `MCU_HAL_TARGET` is missing | Step 2 |
| `Target "stm32cubmx" not found.` (a CMake error) | `MCU_HAL_TARGET` is misspelt | Step 2 |
| `The plain signature for target_link_libraries has already been used with the target` | `PRIVATE` added to the link line | Step 2 |

### Without CMake (STM32CubeIDE, a Makefile)

1. Copy `include/` and `src/` of this repository into your project, for
   example to `Middlewares/mcu-stepper/`, keeping the chip folders inside
   `src/`.
2. Add every `.c` file under `src/` to the build, chip folders included.
3. Add two include paths, `Middlewares/mcu-stepper/include` and
   `Middlewares/mcu-stepper/src`, and two defines:
   `CMSIS_device_header="stm32f4xx.h"`, quotes included, and
   `CONFIG_STEPPER_TMC2209=1`, one per chip. CubeMX already defines
   `USE_HAL_DRIVER` and your device, such as `STM32F405xx`.
4. Steps 3 to 5 of the CMake guide apply as they are.

Where these settings usually live: in STM32CubeIDE, **Project → Properties →
C/C++ Build → Settings → Tool Settings → MCU GCC Compiler** (Include paths,
Preprocessor); in a CubeMX Makefile, `C_SOURCES`, `C_INCLUDES` and `C_DEFS`.
The build errors are the ones in the table above; the fix is the matching
define or include path instead of a CMake line. One more:
`#include expects "FILENAME" or <FILENAME>` means `CMSIS_device_header`
lost its quotes.

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

Setup: [ESP-IDF, step by step](#esp-idf-step-by-step). For reference:

- In `main/idf_component.yml`, `version` is a git ref; `stable` is the
  released one.
- A clone in `components/` is a local component, named after its folder;
  that name is what other components `REQUIRES`. The examples in
  `examples/esp/` need no name: their `main` lists no `REQUIRES`, and ESP-IDF
  gives `main` every component, so they build from any checkout name.
- With the subsystem on and no chip picked, the build warns.
- ESP-IDF 5.2 or newer is required.

## STM32 HAL

Setup: [With CMake](#with-cmake-a-cubemx-cmake-project) or
[Without CMake](#without-cmake-stm32cubeide-a-makefile). The
`add_subdirectory()` route builds a static library `stepper`. For reference:

- Each `CONFIG_STEPPER_<CHIP>` (and `CONFIG_STEPPER_DEBUG`) that is true
  reaches the compiler as `CONFIG_STEPPER_<CHIP>=1`, for the library and
  your app.
- `MCU_HAL_TARGET` is a target that carries your `USE_HAL_DRIVER`, device
  define (`STM32F405xx`, ...) and the include dirs of the HAL, CMSIS and your
  `stm32f4xx_hal_conf.h`. The library copies only those include dirs and
  defines, never the target's sources, so this works with any CubeMX
  version. `MCU_DEVICE_HEADER` becomes `CMSIS_device_header`, which CubeMX
  does not define. Every `mcu-*` library reads these two, so set them once.
  Leave them unset if your project already provides both.
- The CPU flags (`-mcpu`, `-mfloat-abi`, ...) must reach the library too;
  CubeMX's toolchain file sets them for every target.
- Without CMake, `src/` only needs to be on the include path of this
  library's own sources; the defines are what stm-forge passes.

**No chip driver is built by default.** The core is always built and each
driver is opt-in: a chip whose option is absent or 0 compiles to nothing.
With the library on and no chip picked, the CMakeLists warns (ESP-IDF and
`add_subdirectory()`) and `src/stepper_port_stm.c` emits a `#warning`.

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
to your app's `CMakeLists.txt` (in a CubeMX project the target is
`${CMAKE_PROJECT_NAME}`), or the current lines print blanks where the
numbers go.
