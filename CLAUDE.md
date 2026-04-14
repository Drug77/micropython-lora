# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MicroPython firmware for the **LilyGO T3S3 v1.1** (ESP32-S3) board. Implements a secure LoRa radio link using the **LR1121** transceiver — payloads are AES-128 CBC encrypted. Two-unit car alarm system: car unit (TX) has LD2410B radar and detects human presence; handheld unit (RX) displays received alarms. OLED display, battery monitoring, deep sleep with pin-wakeup, WebREPL OTA.

## Flashing the Firmware

```bash
# Erase flash (use correct COM port)
python -m esptool --port COM15 erase_flash

# Flash MicroPython firmware
python -m esptool --port COM15 --baud 460800 write_flash 0 ESP32_GENERIC_S3-FLASH_4M-20241129-v1.24.1.bin
```

Firmware binaries are included in the repo root (`ESP32_GENERIC_S3-*.bin`).

## Deploying Code to Device

Use **mpremote** or **Thonny IDE** to copy files to the device filesystem.

```bash
# Car unit (TX, has radar)
mpremote cp main.py lr1121.py crypto.py oled.py battery.py logging.py ssd1306.py ld2410b.py boot.py webrepl_cfg.py secret.key :

# Handheld unit (RX, no radar hardware)
mpremote cp main.py lr1121.py crypto.py oled.py battery.py logging.py ssd1306.py boot.py webrepl_cfg.py secret.key :
```

## OTA Update (WebREPL)

1. Hold BOOT button (GPIO 0) for **2 seconds** at power-on → LED blinks 6× → stays solid
2. Device connects to Wi-Fi and starts WebREPL on port 8266
3. Upload files via browser (`webrepl.html`) or CLI:
   ```bash
   python webrepl_cli.py -p ota12345 ws://DEVICE_IP:8266/ file.py :/file.py
   ```
4. Reset device when done — boots normally

Password is in `webrepl_cfg.py` (`PASS = "ota12345"`). Change before deploying.

## Architecture

### Module Responsibilities

| File | Role |
|------|------|
| `boot.py` | OTA gate: hold GPIO 0 for 2 s at power-on → WebREPL mode; otherwise loads `main.py` |
| `main.py` | Application entry point: Wi-Fi/NTP sync, radar/radio init, TX/RX main loop, deep sleep |
| `lr1121.py` | Full SPI driver for the Semtech LR1121 chip — init, TX, RX, signal quality |
| `ld2410b.py` | Driver for HLK-LD2410B 24 GHz FMCW radar via UART1 (GPIO 9/10, 256000 baud) |
| `crypto.py` | `AESCryptoManager` — AES-128 CBC encrypt/decrypt JSON payloads; key in `secret.key` |
| `oled.py` | `OLEDDisplay` wrapper around `ssd1306.py` — status screens, progress bar, RX info box |
| `battery.py` | `BatteryMonitor` — reads LiPo voltage via ADC (GPIO 1), estimates percentage and charge state |
| `logging.py` | Custom MicroPython logging with ANSI colour output and optional file handler |
| `ssd1306.py` | Low-level SSD1306 I2C driver (third-party) |
| `webrepl_cfg.py` | WebREPL password (read by `webrepl.start()` in OTA mode) |

### Hardware Pin Map (T3S3 v1.1)

| Signal | GPIO |
|--------|------|
| SPI SCK / MISO / MOSI / NSS | 5 / 3 / 6 / 7 |
| LR1121 BUSY / RST / DIO9 | 34 / 8 / 36 |
| I2C SDA / SCL (OLED) | 18 / 17 |
| Trigger button / OTA gate | 0 |
| LED | 37 |
| Wake pin (ext1) | 8 |
| Battery ADC | 1 |
| LD2410B radar TX / RX | 9 / 10 (UART1) |
| ATGM336H GPS TX / RX (reserved) | 16 / 15 (UART2) |

### Radio Configuration

`lr1121.init_radio()` configures the chip for maximum range:
- **Frequency:** 868 MHz
- **Modulation:** LoRa SF12 / BW125 kHz / CR 4/8 / LDRO enabled
- **TX power:** +22 dBm (HP PA, VBAT supply)
- **Sync word:** 0x12 (private network)

### Encryption

Both TX and RX units must share the same `secret.key` (16-byte AES key). On first boot the key is auto-generated and saved. Copy the key from the TX unit to the RX unit manually.

### Data Flow (TX mode with radar)

1. Poll `LD2410B.read_frame()` every 20 ms (non-blocking)
2. If `target_state > 0` (person detected) and cooldown elapsed (10 s):
   - Read battery → build JSON `{msg, t, c, v, b, ch, ts, dd}` (ts=radar state, dd=distance cm)
   - `AESCryptoManager.encrypt_json()` → `IV (16 B) + AES-CBC ciphertext`
   - `LR1121.transmit_payload()` with progress callback for OLED bar
3. If radar not available (`radar is None`): falls back to periodic TX every 3 s

### Deep Sleep / Wakeup

- Deep sleep is entered after an RX timeout in RX mode.
- Car unit (TX + radar) does **not** deep sleep — radar must scan continuously.
- `esp32.wake_on_ext1` on GPIO 8 wakes the CPU when the radio signals a packet.
- On wakeup, `machine.wake_description()` identifies which pin fired; `need_radio_init` is set `False`.

### Archived / Variant Files

Files named `main.*.py` (e.g. `main.old.py`, `main.oled.py`) are historical snapshots — do not deploy them; only `main.py` is the active firmware.
