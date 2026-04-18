# LoRa Car Alarm v2.3

Двухблочная система охраны автомобиля на базе LilyGO T3S3 v1.1 (ESP32-S3) + LR1121.

Блок в машине (TX) следит за салоном через радар LD2410B и отправляет зашифрованную тревогу по LoRa. Ручной блок (RX) получает пакет и выводит на OLED тип угрозы, расстояние, батарейки, время.

---

## Схема системы

```
┌─────────────────────────────┐          LoRa 868 MHz          ┌────────────────────────┐
│        БЛОК В МАШИНЕ (TX)   │  ──────── AES-128 ──────────►  │   РУЧНОЙ БЛОК (RX)     │
│                             │  ◄─────── AES-128 ──────────   │                        │
│  LD2410B ──► ESP32-S3       │    Heartbeat / Alarm / SOS     │  ESP32-S3              │
│  (радар)     (LR1121)       │    Arm / Ping / SOS_ACK        │  (LR1121)    OLED      │
│  Вибрация    OLED  Battery  │                                │  Кнопка     Battery    │
└─────────────────────────────┘                                └────────────────────────┘
```

---

## Железо

| Компонент | Назначение |
|-----------|------------|
| LilyGO T3S3 v1.1 (×2) | основная плата (ESP32-S3 + LR1121 + OLED SSD1306 + LiPo) |
| HLK-LD2410B | 24 ГГц FMCW-радар, детекция человека в салоне (только TX) |
| Датчик вибрации | SW-420 или аналог, GPIO 11 (только TX) |

---

## Подключение радара LD2410B

### Распиновка

```
LD2410B         T3S3 v1.1 (ESP32-S3)
───────────────────────────────────────
TX      ──►     GPIO 9   (UART1 RX)
RX      ◄──     GPIO 10  (UART1 TX)
OUT     ──►     GPIO 8   (ext1 wakeup)
VCC     ──►     3.3V
GND     ──►     GND
```

> **Важно:** TX радара → RX контроллера (GPIO 9), RX радара → TX контроллера (GPIO 10). Перекрёстное подключение!

### Параметры UART

| Параметр | Значение |
|----------|----------|
| UART ID | 1 |
| Baud rate | 256000 |
| Format | 8N1 |
| TX pin (ESP32→radar) | GPIO 10 |
| RX pin (radar→ESP32) | GPIO 9 |

### Пин OUT радара

GPIO 8 используется для двух функций:
- **Radio DIO9** — прерывание от LR1121 при получении пакета
- **Radar OUT** — HIGH когда радар детектирует присутствие человека

Оба сигнала подключены к одному GPIO через ext1 wakeup. При пробуждении из deep sleep firmware определяет источник (радио пакет или детекция радара).

### Проверка работы радара

После подключения радара и загрузки прошивки, при холодном старте TX модуля:

1. На экране появится "RADAR Проверка.." с прогрессбаром
2. Если радар отвечает в течение 3с → "Radar OK" в логах
3. Если нет ответа → `radar = None`, TX стартует в режиме DISARMED
4. При попытке поставить на охрану без радара → "Radar Error" на обоих модулях

### Конфигурация радара в коде

```python
# main.py — константы
RADAR_UART_ID  = 1       # UART1
RADAR_TX_PIN   = 9       # GPIO 9 (вход от радара)
RADAR_RX_PIN   = 10      # GPIO 10 (выход к радару)
RADAR_CHECK_MS = 3000    # таймаут инициализации (3с)
RADAR_POLL_MS  = 20      # интервал опроса read_frame() (20мс)
RADAR_OLED_MS  = 1000    # интервал обновления OLED при патрулировании
ALARM_COOLDOWN_S = 10    # минимум между тревогами (10с)
```

### Состояния радара (target_state)

| Значение | Описание | На OLED |
|----------|----------|---------|
| 0 | Никого нет | Чисто |
| 1 | Движущийся объект | Движение |
| 2 | Неподвижный объект | Стоит |
| 3 | Оба типа | Дв+Ст |
| -1 | Радар недоступен | НетРад |
| -2 | Вибрация (датчик) | Вибр! |

### Подключение датчика вибрации

```
SW-420          T3S3 v1.1
───────────────────────────
DO      ──►     GPIO 11  (ext1 wakeup)
VCC     ──►     3.3V
GND     ──►     GND
```

При вибрации GPIO 11 → HIGH → ext1 wakeup → пакет "Vibration!" на RX.

---

## Полная распиновка T3S3 v1.1

| Сигнал | GPIO | Примечание |
|--------|------|------------|
| SPI SCK | 5 | LR1121 radio |
| SPI MISO | 3 | LR1121 radio |
| SPI MOSI | 6 | LR1121 radio |
| SPI NSS | 7 | LR1121 radio |
| LR1121 BUSY | 34 | Radio busy |
| LR1121 RST | 8 | Radio reset / Radar OUT / ext1 wake |
| LR1121 DIO9 | 36 | Radio IRQ |
| I2C SDA | 18 | OLED SSD1306 |
| I2C SCL | 17 | OLED SSD1306 |
| BOOT button | 0 | ext0 wake + OTA gate |
| LED | 37 | Статусный |
| Battery ADC | 1 | Напряжение LiPo |
| LD2410B TX→RX | 9 | UART1, radar data in |
| LD2410B RX←TX | 10 | UART1, radar data out |
| Vibration | 11 | ext1 wake (TX only) |

---

## Установка прошивки

### Первый раз (через USB)

```bash
# 1. Стереть flash
python -m esptool --port COM15 erase_flash

# 2. Залить MicroPython
python -m esptool --port COM15 --baud 460800 write_flash 0 ESP32_GENERIC_S3-FLASH_4M-20241129-v1.24.1.bin

# 3. Залить файлы (одинаковый набор на оба блока)
mpremote connect COM15 cp main.py lr1121.py crypto.py oled.py battery.py logging.py ssd1306.py ld2410b.py boot.py webrepl_cfg.py font_cyr.py lang.py :
```

### Настройка config.json

```bash
# TX (машина):
mpremote connect COM7 exec "f=open('config.json','w'); f.write('{\"mode\":\"TX\",\"wifi_ssid\":\"...\",\"wifi_pass\":\"...\",\"timezone\":2,\"sleep\":false,\"lang\":\"ru\"}'); f.close()"

# RX (брелок):
mpremote connect COM14 exec "f=open('config.json','w'); f.write('{\"mode\":\"RX\",\"wifi_ssid\":\"...\",\"wifi_pass\":\"...\",\"timezone\":2,\"sleep\":false,\"lang\":\"ru\"}'); f.close()"
```

### Ключ шифрования

Оба блока должны иметь одинаковый `secret.key` (16 байт AES):

```bash
# Скопировать с первого блока
mpremote connect COM7 cp :secret.key .
# Залить на второй
mpremote connect COM14 cp secret.key :
```

---

## Типы пакетов

| Направление | `msg` | Описание |
|-------------|-------|----------|
| RX → TX | `Arm` | Постановка/снятие с охраны |
| RX → TX | `Ping` | Запрос статуса |
| RX → TX | `SOS_ACK` | Подтверждение просмотра SOS |
| TX → RX | `Armed` / `Disarmed` | Подтверждение охраны |
| TX → RX | `Status` | Ответ на Ping |
| TX → RX | `Heartbeat` | Периодический пинг (60с) |
| TX → RX | `Alarm!` | Тревога (радар) |
| TX → RX | `Vibration!` | Тревога (вибрация) |
| TX → RX | `SOS` | Экстренный сигнал |
| TX → RX | `Radar Error` | Радар недоступен |

### Структура пакета

```json
{
  "msg": "Heartbeat",
  "t": 1744123456,
  "c": 42,
  "v": 3.87,
  "b": 72,
  "ch": false,
  "ts": 0,
  "dd": 0,
  "armed": true
}
```

Все пакеты зашифрованы: `[IV 16 байт][AES-128-CBC шифротекст]`.

---

## Радиопараметры LoRa

| Параметр | Значение |
|----------|----------|
| Частота | 868 МГц |
| Spreading Factor | SF12 |
| Bandwidth | 125 кГц |
| Coding Rate | 4/8 |
| Мощность TX | +22 дБм |
| LDRO | Включён |
| Sync Word | 0x12 |
| Time-on-Air (~128 байт) | ~8 секунд |

---

## Управление через Claude Code

```
/micropython              — прошить firmware
/micropython chip         — информация о чипе
/micropython version      — версия MicroPython
/micropython memory       — RAM и диск
/micropython deploy       — залить код на устройство
/micropython logs         — boot-лог (25 сек)
```

---

## OTA обновление (WebREPL)

1. При холодном старте удерживай BOOT 2с → LED мигнёт 6× → WebREPL активен
2. Подключись через `ws://IP:8266`, пароль: `ota12345`
3. Загрузи файлы, нажми RESET
