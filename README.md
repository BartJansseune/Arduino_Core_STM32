# Arduino core support for STM32L082-based boards

This repository is MCCI's version of [Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32), adapted for the STM32L082. The BSP here targets the STM32L082 as used in the Murata CMWX1ZZABZ LoRa&reg; module, as further in the MCCI Catena&reg; LoRaWAN&reg; technology boards. Several of these boards are closely compatible with the Adafruit Feather M0 LoRa. See [Supported Boards and CPUs](#supported-boards-and-cpus) for more info.

For general information, please check the stm32duino [README.md](https://github.com/stm32duino/Arduino_Core_STM32#arduino-core-support-for-stm32-based-boards), especially the [Getting Started](https://github.com/stm32duino/Arduino_Core_STM32#getting-started) section.

[![GitHub release](https://img.shields.io/github/release/mcci-catena/Arduino_Core_STM32.svg)](https://github.com/mcci-catena/Arduino_Core_STM32/releases/latest) [![GitHub commits](https://img.shields.io/github/commits-since/mcci-catena/Arduino_Core_STM32/latest.svg)](https://github.com/mcci-catena/Arduino_Core_STM32/compare/v3.0.5...master)

**Contents:**
<!-- markdownlint-disable MD033 -->
<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
<!-- TOC depthFrom:2 updateOnSave:true -->

- [Getting Started](#getting-started)
- [Features](#features)
- [Supported Boards and CPUs](#supported-boards-and-cpus)
	- [Catena 461x Series](#catena-461x-series)
		- [Sensors on Catena 4612, 4617, and 4618](#sensors-on-catena-4612-4617-and-4618)
	- [Catena 4630 Features](#catena-4630-features)
	- [Catena 480x Features](#catena-480x-features)
- [Troubleshooting](#troubleshooting)
- [Installing a Development Copy of this BSP](#installing-a-development-copy-of-this-bsp)
- [Release History](#release-history)
- [Notes and Acknowledgements](#notes-and-acknowledgements)
- [Support Open-Source Software, Hardware, and Community IoT](#support-open-source-software-hardware-and-community-iot)

<!-- /TOC -->
<!-- markdownlint-restore -->
<!-- Due to a bug in Markdown TOC, the table is formatted incorrectly if tab indentation is set other than 4. Due to another bug, this comment must be *after* the TOC entry. -->

## Getting Started

This repository is available as a package usable with the [Arduino Boards Manager](https://www.arduino.cc/en/guide/cores).  Add this URL to _"Additional Boards Manager URLs"_ under `File>Preferences`:

`https://github.com/mcci-catena/arduino-boards/raw/master/BoardManagerFiles/package_mcci_index.json`

For full instructions on using the "**Boards Manager**", see  [Installing the MCCI Catena BSP](https://github.com/mcci-catena/arduino-boards#installing-the-mcci-catena-bsp).

## Features

The Arduino IDE allows you to select the following items.

- **LoRaWAN Region**: when using the Arduino LMIC, you can select North America, Europe, Australia, Asia-923, Japan, Korea, or India as your target region.
- **Optimization**: choose smallest, fast, faster, fastest, or debug.
- **Serial interface**: Select "USB Serial", "Generic Serial" or "No Serial".
- **System clock**: Select "32 MHz (most power)", "24 MHz", "16 MHz", "4.194 MHz (no USB)" or "2.097 MHz (no USB, least power)". If "2.097 MHz (no USB, least power)" or "4.194 MHz (no USB)" is selected, then you cannot use "USB Serial" for serial interface.
- **Upload method**: select Mass Storage, DFU, or STLink. DFU works well, but requires use of the boot jumper to select DFU mode, and a little driver wrangling on Windows.

## Supported Boards and CPUs

| Board | CPU/SOC/Module | Section | Comment |
|:-----:|:--------------:|:-------:|---------|
| MCCI Catena 4551 | Murata CMWX1ZZABZ module, STM32L082 | [461x](#catena-461x-series) | Obsolete |
| MCCI Catena 4610 | Murata CMWX1ZZABZ module, STM32L082 | [461x](#catena-461x-series) | LiPo battery |
| MCCI Catena 4611 | Murata CMWX1ZZABZ module, STM32L082 | [461x](#catena-461x-series) | Special order only |
| MCCI Catena 4612 | Murata CMWX1ZZABZ module, STM32L082 | [461x](#catena-461x-series) | Primary battery, BME280 |
| MCCI Catena 4617 | Murata CMWX1ZZABZ module, STM32L082 | [461x](#catena-461x-series) | Primary battery, HS3001 |
| MCCI Catena 4618 | Murata CMWX1ZZABZ module, STM32L082 | [461x](#catena-461x-series) | Primary battery, SHT31-DIS-F |
| MCCI Catena 4630 | Murata CMWX1ZZABZ module, STM32L082 | [4630](#catena-4630-features) | LiPo battery,ZMOD4410, PMS7003 |
| MCCI Catena 4801 | Murata CMWX1ZZABZ module, STM32L082 | [480x](#catena-480x-features) | Primary battery, Modbus |
| MCCI Catena 4802 | Murata CMWX1ZZABZ module, STM32L082 | [480x](#catena-480x-features) | Primary battery, Modbus, SHT31-DIS-F, I2C Expander |

### Catena 461x Series

| Feature | 4551 | 4610 | 4611 | 4612 / 4617 / 4618 |
|---------|------|------|------|------|
| TCXO control | Always on (power consumption issue) | Controlled by code | Controlled by code | Controlled by code | Controlled by code | Controlled by code |
| Battery type | Primary (non-rechargeable) reference is 2x AAA cells | Secondary (LiPo rechargeable), compatible with Adafruit | Feather batteries | Primary (non-rechargeable) reference is 2x AAA cells | Primary (non-rechargeable) reference is 2x AAA cells |
| System voltage | 3.3V | 3.3V | 3.3V | 2.2V to 3.3V, depending on whether boost regulator is enabled. |
| Regulator control | EN pin on Feather disables VDD altogether | No boost regulator; EN disables VDD altogether. | EN pin on JP2-3 shuts down boost regulator | EN output from CPU controls boost regulator. 4612/4617/4618 normally run with boost regulator off for lower power. |
| High-side switch for external sensors | No switch, screw terminal power is from VDD (3.3V) | High-side switch allows software to turn off power to the external sensor screw terminals | High-side switch allows software to turn off power to the external sensor screw terminals | High-side switch allows software to turn off power to the external sensor screw terminals |
| Feather electrical compatibility  | Good, except for different battery system | Very good | Very good | Good physical compatibility but the varying VDD may be an issue |
| Feather physical compatibility | Yes | Yes | Yes | Yes |
| USB | Supported | Supported | Supported | Supported | Supported | Supported |
| Sensors | BME280, Si1123 | BME280, Si1113 | BME280, Si1113 | See [Catena 4612/7/8 Sensors](#sensors-on-catena-4612-4617-and-4618), below |
| Screw terminals for external sensors | 2x4 pin | 2x4 pin | 2x4 pin | 2x4 pin | 2x4 pin | 2x4 pin |
| Boot mode enable | Jumper | Dedicated switch | Jumper | Jumper |

#### Sensors on Catena 4612, 4617, and 4618

All three boards incorporate a Silicon Labs Si1133 ambient light sensor. They differ in the environmental sensor.

Board | Sensor | Temperature | Humidity | Barometric Pressure | Comments
:----:|:------:|:-----------:|:--------:|:-------------------:|:----------
4612  | BME280 | Yes | Yes | Yes | Not suitable for outdoor applications or high-humidity environments.
4617  | HS3001 | Yes | Yes | **No** | Suitable for general-purpose use.
4618  | SHT31-DIS-F | Yes | Yes | **No** | Sensor protected by an IP66 PTFE membrane, best for outdoor deployments.

### Catena 4630 Features

The 4630 is a Feather-compatible board designed for field air-quality monitoring.

| Feature | 4630 |
|---------|------|
| TCXO Control | Controlled by code |
| Battery type | LiPo with on-board charger |
| System voltage | 3.3V, plus programmable 5V regulator for PMS7003 PM2.5 sensor support. |
| Regulator control | No boost regulator for 3.3V; EN disables VDD altogether. D12 controls 5V boost regulator for PMS7003. |
| High-side switch for power for external sensors | No |
| Screw terminals for external sensors | None |
| Feather electrical compatibility | Yes |
| Feather physical compatibility | Yes |
| USB | Yes, DFU download, runtime data, and charging |
| Sensors | ZMOD4410 VOC sensor; BME280 temperature/pressure/humidity |
| External interfaces | TTL Serial, dedicated cable for connecting to PMS7003 PM2.5 sensor |

### Catena 480x Features

The 480x is a dedicated board for remote Modbus applications, using the Murata module.

| Feature | 4801 | 4802 |
|---------|------|------|
| TCXO Control | Controlled by code | Controlled by code |
| Battery type | Primary battery, boost regulator (but can tolerate up to 3.7V battery) | Primary battery, buck regulator (but can tolerate up to 5V battery) |
| System voltage | 2.2V to 3.3V, depending on whether boost regulator is enabled. | 3.6V to 5V |
| Regulator control | EN output from CPU controls boost regulator, and 4801 normally runs with boost regulator off for lower power. | Always enabled if +VIN present.
| High-side switch for power for external sensors | Yes | Yes |
| Screw terminals for external sensors | 1x4 pin | 2x4 pin |
| Feather physical compatibility | No | No |
| USB | Not supported | Not supported |
| Sensors | none | SHT31 |
| External interfaces | Modbus, TTL serial | Modbus, TTL serial, I2C Expander |

## Troubleshooting

If you have any issue, you may [file an issue on GitHub](https://github.com/mcci-catena/Arduino_Core_STM32/issues/new).  You may also submit a support request on the [MCCI support forum](http://portal.mcci.com).

## Installing a Development Copy of this BSP

If you want to develop and test changes to this package, we suggest the following.

1. Install the current release from the standard location using the Arduino `Tools>Boards>Boards Manager...` menu. This installs tools and so forth.
2. Select a board supported by this package.
3. Create an empty sketch with Arduino `File>New`, and check that you can build.
4. Close the Arduino IDE.
5. Clone this repo to a convenient spot on your computer.
6. Open a command window and go to the Arduino board installation directory. It will be one of the following.

   | System | Location |
   |:------:|:---------|
   | Windows 7 through 10 | <code>c:\Users\\<em><strong>username</strong></em>\AppData\Local\Arduino15</code>. If you're using git bash, you can also use `~/AppData/Local/Arduino15` |
   | macOS | <code>~/Library/Arduino15</code> |
   | Linux | <code>~/.arduino15</code> |

7. From this location, change directory to `packages/mcci/stm32`.
8. Look at the directory contents with `ls`. You'll see a directory named like a version number, for example `1.0.8`.
9. Move the directory you just found away from the `stm32` directory. It's convenient to have it around, so we suggest that you not delete it.
10. Create a symbolic link to the sandbox you created in step (3) above. On macOS and Linux, this is done using the `ln -s` command. On Windows 10, this is done with the `mklink /d` command. (On older versions of Windows, unless you're very experienced, you might want to use a Linux VM in VirtualBox -- we can't advise on the best way to do this.)
11. Open the Arduino IDE.
12. Use the menu `File>New` to create an empty sketch (or reuse the sketch from step 3).
13. Make sure the board you want to develop for is selected in `Tools>Boards`.
14. Make changes and build.

Remember to restart the IDE whenever you change `platform.txt`, `boards.txt` or `programmers.txt`.

## Release History

- [v3.0.5](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v3.0.5) Patch release. Don't use `<algorithm>` to implement `min()` and `max()` ([#184](https://github.com/mcci-catena/Arduino_Core_STM32/issues/184)). Prevent hangs if USB is enabled but D+/D- float to high/high ([#189](https://github.com/mcci-catena/Arduino_Core_STM32/issues/189), [#190](https://github.com/mcci-catena/Arduino_Core_STM32/issues/190)) -- thanks to Mohammed Mayyan ([@mmayyan](https://github.com/mhmayyan)) for help in finding this.

- [v3.0.4](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v3.0.4) Patch release. Fix typo in `tools/linux/stm32l0-upload` that broke DFU on Linux.

- [v3.0.3](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v3.0.3) Patch release. Fix typo in `tools/macosx/stm32l0-upload` that broke DFU on macOS.

- [v3.0.2](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v3.0.2) Patch release. Update `mccibootloader_image` tool to v0.4.0 ([#171](https://github.com/mcci-catena/Arduino_Core_STM32/issues/171)) to fix problem with ELF files with unusual number of program headers.

- [v3.0.1](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v3.0.1) Patch release. Make "download with bootloader" the default choice in the IDE ([#158](https://github.com/mcci-catena/Arduino_Core_STM32/issues/158)). Windows "download with bootloader was broken for DFU ([#164](https://github.com/mcci-catena/Arduino_Core_STM32/issues/164)) and STLink ([#167](https://github.com/mcci-catena/Arduino_Core_STM32/issues/167)).

- [v3.0.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v3.0.0) Switch to true semantic versioning [#150](https://github.com/mcci-catena/Arduino_Core_STM32/issues/150). Add support for MCCI trusted bootloader [#148](https://github.com/mcci-catena/Arduino_Core_STM32/issues/148), [#149](https://github.com/mcci-catena/Arduino_Core_STM32/issues/149), [#151](https://github.com/mcci-catena/Arduino_Core_STM32/issues/151), [#153](https://github.com/mcci-catena/Arduino_Core_STM32/issues/153). Enable interrupts on entry to reset vector for consistency with ST bootloader.

- [v2.8.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.8.0) Added support for new board: 4802, [#143](https://github.com/mcci-catena/Arduino_Core_STM32/issues/143).

- [v2.7.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.7.0) adds UI support for LoRaWAN network and subband selection, [#22](https://github.com/mcci-catena/Arduino_Core_STM32/issues/22).  Note, though, that the UI options must be implemented by your target LoRaWAN stack in order to be effective.

- [v2.6.1](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.6.1) fixed numerous compile warnings. Also fixed [#78](https://github.com/mcci-catena/Arduino_Core_STM32/issues/78), which resulted in occasional reuse of the entire input ring buffer.

- [v2.6.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.6.0) adds support for KR-920 (Korea 920 MHz) in the region selection menu ([#117](https://github.com/mcci-catena/Arduino_Core_STM32/issues/117)). Adds official library.properties fields for linking pre-compiled object and adds additional pins definitions for SX1276 radios ([#128](https://github.com/mcci-catena/Arduino_Core_STM32/issues/128)) (thanks to Kent Williams). Add source-level debugging support ([#122](https://github.com/mcci-catena/Arduino_Core_STM32/issues/122)). Adjust USB `Vbus` check values for Catena 461x platform. Define AU915 as well as AU921 for LMIC migration ([#131](https://github.com/mcci-catena/Arduino_Core_STM32/issues/131)). Fix link "`warning: changing start of section .bss by 4 bytes`" ([#129](https://github.com/mcci-catena/Arduino_Core_STM32/issues/129)). Update issue template.

- [v2.5.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.5.0) enables the crystal controlled LSE clock, and uses it to run the RTC. Code was substantially refactored to move common code to common directories. Bug fixes.

- [v2.4.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.4.0) Added support for new boards: 4617, 4618 and 4630. Serial interface configuration in `boards.template` has been updated with new option to use both USB Serial and Hardware Serial at same time.

- [v2.3.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.3.0) corrects the handling of clocking in the various SLEEP/STOP/SSTANDBY modes. It also ensures that `.RamFunc` code is properly placed in RAM (initialzed when statics are initialized).

- [v2.2.1](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.2.1) fixed the version number in `platform.txt`, and sets GPIO to reset state when de-initializing SPI, I2C and UARTs.

- [v2.2.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.2.0) increases the serial buffer size to 256 and adds low-power changes for the Catena 4801.

- [v2.1.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.1.0) fixed the device ID register offset, and improved sleep mode power.

- [v2.0.0](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v2.0.0) is a major release. It improves wakeup support and power management, and simplifies some of the menus. New boards: 4610 and 4801. The template system was enhanced. USB manufacturer string is no longer "unknown". Changed hardware serial default baud rate to 115,200 from 9600. Moved USB clock control to USB module. Enhanced begin()/end() for USB and I2C to start/stop clocks. (This is a major bump beause the baud rate change may be a breaking change.)

- [v1.1.2](https://github.com/mcci-catena/Arduino_Core_STM32/releases/tag/v1.1.2) added 4611, 4612 support, and templating for generating boards.txt.

## Notes and Acknowledgements

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. ChaeHee Won, Sungjoon Park, and Terry Moore of MCCI maintain this BSP.

LoRa is a registered trademark of the LoRa Alliance. LoRaWAN is a registered trademark of the LoRa Alliance.

All other trademarks are the property of their respective owners.

## Support Open-Source Software, Hardware, and Community IoT

Everyone at MCCI invests time and resources providing this open-source code and open-source hardware. MCCI is also the principal corporate sponsor of [The Things Network New York](https://thethings.nyc) and [Ithaca](https://ttni.tech). Please support our work by purchasing products from MCCI! Visit our on-line store at [store.mcci.com](https://store.mcci.com).
