# mcu-stepper

Set motor current and read driver status for a TMC2209 over UART or a TMC5240 over SPI. The TMC2209 accepts STEP/DIR or a velocity command. The TMC5240 generates position and velocity ramps on the chip.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL. Hardware testing is pending. The host tests use simulated devices.

## Quick start

Wire the TMC2209 single-wire UART as shown in the wiring guide. Check the enable pin, sense resistor and motor current settings before powering the motor. The example commands motion.

In a terminal with ESP-IDF activated, get the example and select the target for an ESP32. Change the target if your board uses another ESP32 chip.

```sh
git clone https://github.com/JonasAtelier/mcu-stepper.git stepper
cd stepper/examples/esp/tmc2209
idf.py set-target esp32
```

Check the [wiring guide](docs/hardware.md) and edit the settings in [main.c](examples/esp/tmc2209/main/main.c) to match your board. Connect the board by USB, then build, upload and open the serial monitor.

```sh
idf.py build flash monitor
```

The example spins the motor using the chip’s pulse generator and reports driver status. The TMC5240 acceleration scale still needs a bench check. [Acceleration scale](docs/chips.md#acceleration-scale)

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Start from an example for your chip. The STM32 examples fit into a project with board setup already in place. [ESP-IDF examples](examples/esp/) · [STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE)

<p align="center">
  <img src="assets/ramp.svg" width="100%" alt="A target position entering a TMC5240, which runs the whole ramp itself">
</p>
