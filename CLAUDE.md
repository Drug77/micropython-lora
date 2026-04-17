# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MicroPython firmware for the **LilyGO T3S3 v1.1** (ESP32-S3) board. Implements a secure LoRa radio link using the **LR1121** transceiver — payloads are AES-128 CBC encrypted. Two-unit car alarm system: car unit (TX) has LD2410B radar and detects human presence; handheld unit (RX) is the fob/controller. OLED display with pixel-art icons, battery monitoring, deep sleep with ext0+ext1 wakeup, WebREPL OTA.

## Flashing the Firmware

```bash
python -m esptool --port COM15 erase_flash
python -m esptool --port COM15 --baud 460800 write_flash 0 ESP32_GENERIC_S3-FLASH_4M-20241129-v1.24.1.bin
```

Firmware binaries are included in the repo root (`ESP32_GENERIC_S3-*.bin`).

## Deploying Code to Device

Both TX and RX units receive the same file set. Mode is set via `config.json`.

```bash
mpremote cp main.py lr1121.py crypto.py oled.py battery.py logging.py ssd1306.py ld2410b.py boot.py webrepl_cfg.py secret.key :
```

Then create config.json on device:
```bash
# TX (car):
mpremote exec "f=open('config.json','w'); f.write('{\"mode\":\"TX\",\"wifi_ssid\":\"...\",\"wifi_pass\":\"...\",\"timezone\":2}'); f.close()"
# RX (fob):
mpremote exec "f=open('config.json','w'); f.write('{\"mode\":\"RX\",\"wifi_ssid\":\"...\",\"wifi_pass\":\"...\",\"timezone\":2}'); f.close()"
```

## OTA Update (WebREPL)

1. On **cold boot only** (not deep sleep wake): hold BOOT button for 2 s → LED blinks 6× → WebREPL
2. Deep sleep wake skips OTA check entirely (boot.py guards with `DEEPSLEEP_RESET`)
3. Upload files, then reset device

## Architecture

### Module Responsibilities

| File | Role |
|------|------|
| `boot.py` | OTA gate: hold GPIO 0 for 2 s at cold boot → WebREPL; skips check on deep sleep wake |
| `main.py` | Entry point: button detection, WiFi (RX only), radio/radar init, TX/RX loops, deep sleep |
| `lr1121.py` | SPI driver for LR1121 — init, TX, RX, continuous RX for deep sleep |
| `ld2410b.py` | LD2410B 24 GHz radar driver via UART1 (GPIO 9/10, 256000 baud) |
| `crypto.py` | AES-128 CBC encrypt/decrypt JSON; key in `secret.key` |
| `oled.py` | SSD1306 OLED wrapper — header with batteries/WiFi/lock, splash, alerts, status screens |
| `battery.py` | LiPo ADC (GPIO 1) → voltage, percentage, charge state |
| `logging.py` | ANSI colour logging for MicroPython |
| `ssd1306.py` | Low-level SSD1306 I2C driver (third-party) |

### Hardware Pin Map (T3S3 v1.1)

| Signal | GPIO |
|--------|------|
| SPI SCK / MISO / MOSI / NSS | 5 / 3 / 6 / 7 |
| LR1121 BUSY / RST / DIO9 | 34 / 8 / 36 |
| I2C SDA / SCL (OLED) | 18 / 17 |
| BOOT button (ext0 wake + OTA gate) | 0 |
| LED | 37 |
| Wake pin (ext1) / Radar OUT | 8 |
| Vibration sensor (ext1) | 11 |
| Battery ADC | 1 |
| LD2410B radar TX / RX | 9 / 10 (UART1) |

### Button Behavior

**Short press** (quick tap) vs **Long press** (hold 2 seconds).

| Mode | Short Press | Long Press (2s) |
|------|-------------|-----------------|
| **RX (fob)** | Send `"Arm"` toggle to TX | Send `"Ping"` status request to TX |
| **TX sleeping** | Wake → show status 5s → sleep | Wake → send `"SOS"` to RX → sleep |
| **TX active** | Ignored | Send `"SOS"` to RX → sleep |

Button wake detection: ext0 (GPIO 0, active LOW). Checked immediately in `main()` before heavy init.

### WiFi & Time Sync

- **RX only** connects to WiFi on cold boot, syncs NTP, then disconnects.
- **TX** never uses WiFi. Gets time from RX via LoRa packets (RTC sync on every received packet with valid timestamp).

### Packet Types

| Direction | `msg` | Purpose |
|-----------|-------|---------|
| RX → TX | `"Arm"` | Toggle arm/disarm |
| RX → TX | `"Ping"` | Request status |
| TX → RX | `"Armed"` / `"Disarmed"` | Arm state confirmation |
| TX → RX | `"Status"` | Status response to Ping |
| TX → RX | `"Heartbeat"` | Periodic keepalive |
| TX → RX | `"Alarm!"` | Radar detection |
| TX → RX | `"Vibration!"` | Vibration sensor |
| TX → RX | `"SOS"` | Emergency (long press TX) |
| TX → RX | `"Radar Error"` | Radar offline |

Payload structure: `{msg, t, c, v, b, ch, ts, dd, armed}`

### Deep Sleep / Wakeup

Both modes use ext0 (button, GPIO 0) + ext1 (sensors/radio) + optional timer.

- **Before sleep:** `radio.prepare_for_deepsleep()` → continuous RX for both modes. Armed state persisted to RTC memory.
- **ext0 wake (button):** boot.py skips OTA. main.py detects short/long press immediately.
- **ext1 wake:** GPIO 8 (radio DIO9 / radar OUT) + GPIO 11 (vibration, TX only). On TX, tries LoRa receive first; if no packet → sensor alarm.
- **Timer wake (RX):** periodic ping every `RX_SLEEP_INTERVAL_S` (300s).
- **TX disarmed + sensor wake:** ignores alarm, goes back to sleep.

### OLED Display

128×64 SSD1306, 8×8 font. Header bar (13px, inverted) with:
- Title (7 chars max)
- Lock icon (armed/disarmed)
- Antenna icon
- WiFi icon
- Two mini battery icons with percentage (local + remote device)

Splash screen on cold boot: mode in 2× large text, version below.

### Encryption

Both units share `secret.key` (16-byte AES key, auto-generated on first boot). Copy from TX to RX manually.

### Radio Configuration

LoRa SF12 / BW125 kHz / CR 4/8 / 868 MHz / +22 dBm / LDRO enabled / Sync word 0x12

### Archived Files

`main.*.py` are historical snapshots — only `main.py` is active firmware.
