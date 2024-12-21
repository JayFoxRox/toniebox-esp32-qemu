# toniebox-esp32-qemu

Hacks for https://github.com/espressif/qemu/tree/1311d7b25a6eba3d990b1bbf0ef32dc8a90d54ef to add some ESP32S3 features to support the Toniebox.
For the original QEMU README, see qemu-README.rst

**Warning: Because this is very hacky and unfinished, I might force-push from time to time.**


## Building

Follow the steps in https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/esp32s3/README.md


## Running and debugging

See emulate.sh for how to run this.
Because I'm lazy, this also rebuilds before each run.

You also need to run debug.sh at the same time, as the emulation is paused until a debugger connects.

Both files must be tweaked for your directory structure.


## Status

- When supplied with a flashdump, the emulation successfully goes into the user code.
- The expected serial output is produced on the QEMU serial output.
- The log shows that the LED controller is hooked up to the Toniebox LED GPIOs.

TODO:

- Add more debug logging.
- Emulate the LIS3DH (accelerometer).
- Emulate the ear buttons.
- Expose the current LED color.
- Figure out why there's no I2S data.
- Figure out why there's no SDMMC data.
- Emulate the TRF7962A (RFID).
- Emulate the DAC3100 / capture I2S.


## Useful links

Pinout: https://tonies-wiki.revvox.de/docs/wiki/esp32/pinout/
Some firmware information: https://github.com/JayFoxRox/toniebox-hacking/wiki
