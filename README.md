# TinyUF2 Bootloader

[![Build Status](https://github.com/adafruit/tinyuf2/workflows/Build/badge.svg)](https://github.com/adafruit/tinyuf2/actions)[![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

This repo is cross-platform UF2 Bootloader projects for MCUs based on [TinyUSB](https://github.com/hathach/tinyusb)

```
.
├── apps              # Useful applications such as self-update, erase firmware
├── lib               # Sources from 3rd party such as tinyusb, mcu drivers ...
├── ports             # Port/family specific sources
│   ├── espressif
│   │   └── boards/   # Board specific sources
│   │   └── Makefile  # Makefile for this port
│   └── mimxrt10xx         
├── src               # Cross-platform bootloader sources files
```

## Features

Supported features are

- Double tap to enter DFU, reboot to DFU and quick reboot from application
- DFU with MassStorage (MSC)
- Self update with uf2 file
- Indicator: LED, RGB, TFT
- Debug log with uart/swd

Not all features are implemented for all MCUs, following is supported MCUs and its feature

| MCU         | MSC  | Double Reset | Self-update | Write Protection | Neopixel | TFT  |
| :---------- | :--: | :----------: | :---------: | :--------------: | :------: | :--: |
| ESP32 S2/S3 |  ✔   |   Need RC    |      ✔      |                  |    ✔     |  ✔   |
| K32L2       |  ✔   |      ✔       |             |                  |          |      |
| LPC55       |  ✔   |      ✔       |             |                  |    ✔     |      |
| iMXRT       |  ✔   |      ✔       |      ✔      |                  |    ✔     |      |
| STM32F3     |  ✔   |      ✔       |      ✔      |                  |    ✔     |      |
| STM32F4     |  ✔   |      ✔       |      ✔      |        ✔         |    ✔     |      |

## Build and Flash

Following is generic compiling information. Each port may require extra set-up and slight different process e.g esp32s2 require setup IDF.

### Compile

To build this for a specific board, we need to change current directory to its port folder

```
$ cd ports/stm32l4_no_otg
```

Firstly we need to get all of submodule dependecy for our board e.g mcu driver with `get-deps` target. You only need to do this once for each mcu family

```
make BOARD=generic_l433 get-deps
```

Then compile with `all` target:

```
make BOARD=generic_l433 all
```

### Flash

`flash` target will use the default on-board debugger (jlink/cmsisdap/stlink/dfu) to flash the binary, please install those support software in advance. Some board use bootloader/DFU via serial which is required to pass to make command

```
$ make BOARD=generic_l433 flash
```

If you use an external debugger, there is `flash-jlink`, `flash-stlink`, `flash-pyocd` which are mostly like to work out of the box for most of the supported board.

### Debug

To compile for debugging add `DEBUG=1`, this will mostly change the compiler optimization

```
$ make BOARD=generic_l433 DEBUG=1 all
```
Debug with Log on rtt:

```
$ make BOARD=generic_l433 LOG=2 DEBUG=1 LOGGER=rtt all
```

#### Log

Should you have an issue running example and/or submitting an bug report. You could enable TinyUSB built-in debug logging with optional `LOG=`. 
- **LOG=1** will print message from bootloader and error if any from TinyUSB stack.
- **LOG=2** and **LOG=3** will print more information with TinyUSB stack events

```
$ make BOARD=generic_l433 LOG=1 all
```

#### Logger

By default log message is printed via on-board UART which is slow and take lots of CPU time comparing to USB speed. If your board support on-board/external debugger, it would be more efficient to use it for logging. There are 2 protocols: 

- `LOGGER=rtt`: use [Segger RTT protocol](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/)
  - Cons: requires jlink as the debugger.
  - Pros: work with most if not all MCUs
  - Software viewer is JLink RTT Viewer/Client/Logger which is bundled with JLink driver package.
- `LOGGER=swo`: Use dedicated SWO pin of ARM Cortex SWD debug header.
  - Cons: only work with ARM Cortex MCUs minus M0
  - Pros: should be compatible with more debugger that support SWO.
  - Software viewer should be provided along with your debugger driver.

```
$ make BOARD=generic_l433 LOG=2 LOGGER=rtt all
$ make BOARD=generic_l433 LOG=2 LOGGER=swo all
```

# STM32L433 TinyUF2 Bootloader encryption
First block where bootloader checks[256 bytes] is information block. Reset vector is placed after
```C
#define ECDSA_SIGN_LEN (64)

typedef struct __attribute__((packed)) {
  uint32_t fw_len;
  uint8_t fw_signature[ECDSA_SIGN_LEN];
  uint32_t fw_version[2];          // two bytes
  uint32_t date_of_compilation[6]; // year YYYY, month MM, day DD, hour HH, min
                                   // MM, sec SS
} fw_controll_sector_t;
```
Each UF2 block when encryped has UF2 set flag 0x10000 meaning encryption enabled.
Each block is encrypted with AES-CCM so also integrity verified. 
Nonce is constant and held in the same place as the secret.
Nonce for each UF2 block is increased by block number | fw_version | date_of_compilation.

Signature of FW is every code byte SHA256 signed by ECDSA256 held in bootloader.

If FW signature is not ok the FW is beeing deleted. 

If signature matches the FW is run.

## TinyUF2 Bootloader signature

Bootloader is signed and signature(with public key) is placed at constant address[TO BE DETERMINED]. 
Signature is calculated over SHA256 calculated like below:
hash=SHA256(Bootloader_code1+signature_and_public_key_zeroed_bytes+Bootloader_code2+MCU_UID[12bytes])
