"""
LoRa Car Alarm — единая прошивка для TX (машина) и RX (брелок).

Режим работы определяется файлом config.py → MODE = "TX" | "RX".
Один и тот же набор файлов заливается на оба устройства.

TX (машина): радар LD2410B + датчик вибрации → шифрованные тревоги по LoRa.
RX (брелок): приём тревог, пинг машины для запроса статуса.

Плата: LilyGO T3S3 v1.1 (ESP32-S3 + LR1121 + SSD1306 OLED + LiPo).
"""

FIRMWARE_VERSION = "2.0.0"

# =============================================================================
# Импорты
# =============================================================================
from battery import BatteryMonitor
from crypto import AESCryptoManager
import esp32
from machine import Pin, SPI, I2C
import machine
import logging
import micropython
import network
import ntptime
import time
import ujson

from lr1121 import LR1121, LR1121_SPI_BAUDRATE
from oled import OLEDDisplay

# --- Конфигурация устройства (config.json) ---
def _load_config(path="config.json"):
    """Загрузить config.json. Если не найден — вернуть defaults."""
    try:
        with open(path, 'r') as f:
            return ujson.loads(f.read())
    except (OSError, ValueError):
        return {}

_cfg = _load_config()
MODE            = _cfg.get("mode", "TX")
WIFI_SSID       = _cfg.get("wifi_ssid", "YOUR_WIFI_NAME")
WIFI_PASS       = _cfg.get("wifi_pass", "")
TIMEZONE_OFFSET = _cfg.get("timezone", 2)

# --- Драйвер радара (только TX) ---
if MODE == "TX":
    from ld2410b import LD2410B
else:
    LD2410B = None

micropython.alloc_emergency_exception_buf(256)

# =============================================================================
# Логирование
# =============================================================================
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("main")

# =============================================================================
# Аппаратные пины (T3S3 v1.1)
# =============================================================================
I2C_SDA, I2C_SCL = 18, 17
SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN = 5, 3, 6, 7
BUSY_PIN, RST_PIN, DIO9_PIN = 34, 8, 36
TRIGGER_PIN = 0       # Кнопка BOOT (GPIO 0, внешний pull-up)
LED_PIN     = 37      # Встроенный светодиод
WAKE_PIN    = 8       # ext1 wakeup: radio DIO9 (RX) / radar OUT (TX)
SHOCK_PIN   = 11      # ext1 wakeup: датчик вибрации (TX)
BATTERY_ADC = 1       # АЦП батареи

# --- LD2410B Radar (UART1, только TX) ---
RADAR_UART_ID = 1
RADAR_TX_PIN  = 9
RADAR_RX_PIN  = 10

# =============================================================================
# Константы времени
# =============================================================================
ALARM_COOLDOWN_S     = 10     # Минимум секунд между повторными тревогами
RADAR_POLL_MS        = 20     # Период опроса радара (мс)
RADAR_OLED_MS        = 1000   # Обновление OLED при ожидании (мс)
RADAR_CHECK_MS       = 3000   # Время проверки наличия радара при загрузке (мс)

TX_IDLE_SLEEP_S      = 120    # Секунд без цели → TX засыпает
TX_HEARTBEAT_S       = 60     # Интервал heartbeat от TX (с)
TX_LISTEN_INTERVAL_S = 30     # Как часто TX слушает пинги от RX (с)
TX_LISTEN_WINDOW_MS  = 10000  # Длина окна прослушивания (мс, ≥ airtime SF12)

RX_LISTEN_TIMEOUT_MS = 30000  # RX слушает эфир до deep sleep (мс)
RX_PING_TIMEOUT_MS   = 20000  # Сколько RX ждёт ответ на пинг (мс)
RX_SLEEP_INTERVAL_S  = 300    # Периодическое пробуждение RX (с, 0 = выкл)

# =============================================================================
# Глобальные счётчики
# =============================================================================
tx_counter = 0
rx_counter = 0
lost_counter = 0
expected_c = None

# =============================================================================
# Утилиты
# =============================================================================

def go_to_deepsleep(oled=None, radio=None, sleep_ms=0):
    """Перевести ESP32 в deep sleep с настроенными источниками пробуждения.

    TX: GPIO 8 (radar OUT) + GPIO 11 (вибрация), radio в standby.
    RX: GPIO 8 (radio DIO9), radio в continuous RX.

    Args:
        oled:     OLEDDisplay — выключить экран перед сном.
        radio:    LR1121 — перевести в нужный режим.
        sleep_ms: int — таймер пробуждения (мс). 0 = без таймера (только ext1).
    """
    # Экран — выключить для экономии
    if oled and hasattr(oled, 'display') and oled.display:
        oled.display.poweroff()

    # Радио — режим зависит от роли
    if radio:
        if MODE == "TX":
            radio.standby()  # TX не слушает эфир во сне
        else:
            radio.prepare_for_deepsleep()  # RX: continuous RX для DIO9

    # Wake-пины (GPIO 0–21 only)
    wake_pins = [Pin(WAKE_PIN, Pin.IN, Pin.PULL_DOWN)]  # GPIO 8
    if MODE == "TX":
        wake_pins.append(Pin(SHOCK_PIN, Pin.IN, Pin.PULL_DOWN))  # GPIO 11

    esp32.wake_on_ext1(pins=tuple(wake_pins), level=esp32.WAKEUP_ANY_HIGH)

    time.sleep_ms(100)  # Дать логгеру вывести текст
    if sleep_ms > 0:
        machine.deepsleep(sleep_ms)  # Timer + ext1
    else:
        machine.deepsleep()  # Только ext1


def format_date_time(timestamp=None):
    """Форматировать Unix-время в читаемую дату и время.

    Args:
        timestamp: int — Unix-время. None = текущее время.

    Returns:
        tuple: (дата_строка, время_строка). Пример: ('Fr 27.02.2026', '12:30:44')
    """
    if timestamp is None:
        timestamp = time.time()
    local_time = time.localtime(timestamp + (TIMEZONE_OFFSET * 3600))
    days = ["Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"]
    wd = days[local_time[6]]
    date_str = f"{wd} {local_time[2]:02d}.{local_time[1]:02d}.{local_time[0]}"
    time_str = f"{local_time[3]:02d}:{local_time[4]:02d}:{local_time[5]:02d}"
    return date_str, time_str


def connect_wifi_and_sync(oled):
    """Подключиться к Wi-Fi и синхронизировать время по NTP.

    Три попытки подключения. После синхронизации Wi-Fi полностью отключается,
    чтобы не мешать deep sleep (известный баг ESP32).

    Args:
        oled: OLEDDisplay — для отображения прогресса на экране.
    """
    if WIFI_SSID == "YOUR_WIFI_NAME":
        logger.warning("Default Wi-Fi credentials — skipping")
        oled.show_status("WIFI SKIP", "Default config", progress=100)
        time.sleep(1)
        return

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    time.sleep_ms(200)
    if wlan.isconnected():
        wlan.disconnect()
        time.sleep_ms(200)

    for attempt in range(1, 4):
        logger.info("Wi-Fi attempt %d/3...", attempt)
        oled.show_status("WIFI", f"Attempt {attempt}/3", WIFI_SSID[:15], progress=attempt * 33)
        try:
            wlan.connect(WIFI_SSID, WIFI_PASS)
        except OSError as e:
            logger.error("Wi-Fi error on attempt %d: %s", attempt, e)
        for _ in range(50):
            if wlan.isconnected():
                break
            time.sleep_ms(100)
        if wlan.isconnected():
            break

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        logger.info("Wi-Fi connected: %s", ip)
        oled.show_status("WIFI OK", ip, progress=90)
        time.sleep(1)
        try:
            ntptime.settime()
            d, t = format_date_time()
            logger.info("NTP synced: %s %s", d, t)
        except Exception as e:
            logger.warning("NTP failed: %s", e)
    else:
        logger.warning("Wi-Fi failed — offline mode")
        oled.show_status("WIFI FAIL", "Offline", progress=100)
        time.sleep(1)

    # Полностью выключить Wi-Fi (иначе deep sleep может крашиться)
    if wlan.isconnected():
        wlan.disconnect()
    wlan.active(False)
    logger.debug("Wi-Fi disabled")


def blink_ok(led):
    """Однократное мигание LED — операция успешна."""
    led.value(1); time.sleep_ms(100); led.value(0)


def blink_fail(led):
    """Тройное быстрое мигание LED — ошибка."""
    for _ in range(3):
        led.value(1); time.sleep_ms(50); led.value(0); time.sleep_ms(50)


class TriggerFlag:
    """Флаг аппаратного прерывания кнопки с debounce через micropython.schedule.

    Кнопка вызывает ISR → schedule → устанавливает флаг.
    Основной цикл проверяет is_triggered() и сбрасывает флаг.
    """
    def __init__(self):
        self.value = False

    def _set_true_scheduled(self, _arg):
        self.value = True

    def isr(self, _pin):
        """ISR-обработчик прерывания кнопки."""
        micropython.schedule(self._set_true_scheduled, 0)

    def is_triggered(self):
        """Проверить и сбросить флаг. Возвращает True один раз за нажатие."""
        if self.value:
            self.value = False
            return True
        return False


# =============================================================================
# Коммуникация
# =============================================================================

def send_packet(radio, crypto, payload, oled=None, led=None):
    """Зашифровать и отправить payload по LoRa.

    Args:
        radio:   LR1121 — радиомодуль.
        crypto:  AESCryptoManager — шифрование.
        payload: dict — данные для отправки (будут сериализованы в JSON).
        oled:    OLEDDisplay — для progress bar (опционально).
        led:     Pin — LED для индикации результата (опционально).

    Returns:
        bool: True если пакет отправлен успешно.
    """
    global tx_counter
    encrypted = crypto.encrypt_json(payload)
    if not encrypted:
        logger.error("Encryption failed for msg=%s", payload.get("msg"))
        if led:
            blink_fail(led)
        return False

    logger.info("TX %d bytes: msg=%s", len(encrypted), payload.get("msg"))
    ok = radio.transmit_payload(encrypted, timeout_ms=10000)
    if ok:
        tx_counter += 1
        logger.info("TX_DONE #%d", tx_counter)
        if led:
            blink_ok(led)
    else:
        logger.error("TX timeout")
        if led:
            blink_fail(led)
    return ok


def receive_packet(radio, crypto, timeout_ms):
    """Принять и расшифровать пакет из эфира.

    Args:
        radio:      LR1121 — радиомодуль.
        crypto:     AESCryptoManager — дешифрование.
        timeout_ms: int — таймаут ожидания (мс).

    Returns:
        dict | None: расшифрованный payload или None при таймауте/ошибке.
    """
    raw = radio.receive_payload(timeout_ms=timeout_ms, max_len=255)
    if raw is None:
        return None
    logger.info("RX %d bytes", len(raw))
    obj = crypto.decrypt_json(raw)
    if obj is None:
        logger.error("Decryption failed")
    return obj


def build_status_payload(bat_monitor, radar=None, armed=True):
    """Собрать payload со статусом устройства.

    Args:
        bat_monitor: BatteryMonitor — данные батареи.
        radar:       LD2410B | None — радар (для ts/dd).
        armed:       bool — состояние охраны.

    Returns:
        dict: payload без поля 'msg' (добавляется вызывающим кодом).
    """
    v, pct, is_chrg = bat_monitor.get_status()
    ts_val = 0
    dd_val = 0
    if radar:
        ts_val = radar.target_state
        dd_val = radar.detection_distance
    return {
        "t": time.time(), "c": tx_counter,
        "v": v, "b": pct, "ch": is_chrg,
        "ts": ts_val, "dd": dd_val, "armed": armed,
    }


# =============================================================================
# TX Main Loop (машина)
# =============================================================================

def tx_main_loop(radio, crypto, oled, bat_monitor, radar, led, flag):
    """Главный цикл TX-блока (машина).

    Логика:
    - ARMED: радар патрулирует, тревоги → LoRa, idle → deep sleep.
    - DISARMED: без радара, периодический heartbeat, ожидание re-arm.
    - Каждые TX_LISTEN_INTERVAL_S секунд: слушаем пинги от RX.

    Args:
        radio:       LR1121 — радиомодуль.
        crypto:      AESCryptoManager — шифрование.
        oled:        OLEDDisplay — экран.
        bat_monitor: BatteryMonitor — заряд батареи.
        radar:       LD2410B | None — радар (None = не подключён).
        led:         Pin — светодиод.
        flag:        TriggerFlag — флаг кнопки BOOT.
    """
    global tx_counter

    armed = True
    last_alarm_time = 0
    last_detection_time = time.time()
    last_heartbeat_time = time.time()
    last_listen_time = time.time()
    radar_oled_timer = time.ticks_ms()

    logger.info("TX loop started. Armed=%s, Radar=%s", armed, radar is not None)

    while True:
        now = time.time()

        # ── Кнопка BOOT: ARM / DISARM ───────────────────────────────────
        if flag.is_triggered():
            armed = not armed
            state = "ARMED" if armed else "DISARMED"
            logger.warning("Button: %s", state)
            oled.show_status(state, "Car alarm", f"v{FIRMWARE_VERSION}", progress=100 if armed else 0)
            # Уведомить RX о смене режима
            p = build_status_payload(bat_monitor, radar, armed)
            p["msg"] = state
            send_packet(radio, crypto, p, led=led)
            last_detection_time = now  # Сброс idle-таймера
            time.sleep(2)
            continue

        # ── Окно прослушивания (пинги от RX) ────────────────────────────
        if (now - last_listen_time) >= TX_LISTEN_INTERVAL_S:
            logger.debug("Listen window (%dms)...", TX_LISTEN_WINDOW_MS)
            oled.show_status("LISTENING", "For RX ping...", progress=0, antenna=True)
            msg = receive_packet(radio, crypto, TX_LISTEN_WINDOW_MS)
            if msg and msg.get("msg") == "Ping":
                logger.info("Ping from RX! Sending status...")
                p = build_status_payload(bat_monitor, radar, armed)
                p["msg"] = "Status"
                send_packet(radio, crypto, p, oled=oled, led=led)
            last_listen_time = time.time()
            continue

        # ── DISARMED: только heartbeat ──────────────────────────────────
        if not armed:
            if (now - last_heartbeat_time) >= TX_HEARTBEAT_S:
                p = build_status_payload(bat_monitor, radar, armed)
                p["msg"] = "Heartbeat"
                oled.show_status("DISARMED", "Heartbeat", f"Pkt#{tx_counter}", progress=0)
                send_packet(radio, crypto, p, led=led)
                last_heartbeat_time = time.time()
            else:
                v, pct, chrg = bat_monitor.get_status()
                oled.show_tx_disarmed(pct, chrg, tx_counter)
            time.sleep_ms(500)
            continue

        # ── ARMED: радар не подключён → ошибка + deep sleep ─────────────
        if radar is None:
            p = build_status_payload(bat_monitor, None, armed)
            p["msg"] = "Radar Error"
            p["ts"] = -1
            oled.show_status("TX ERROR", "Radar offline!", f"Pkt#{tx_counter}", "Sleeping...")
            send_packet(radio, crypto, p, led=led)
            time.sleep(2)
            logger.info("No radar — deep sleep (wake: radar OUT + vibration)")
            oled.show_status("DEEP SLEEP", "No radar", "Wake: sensor")
            go_to_deepsleep(oled=oled, radio=radio)

        # ── ARMED: радар активен — патрулирование ────────────────────────
        idle_elapsed = now - last_detection_time

        # Heartbeat по таймеру
        if (now - last_heartbeat_time) >= TX_HEARTBEAT_S:
            p = build_status_payload(bat_monitor, radar, armed)
            p["msg"] = "Heartbeat"
            send_packet(radio, crypto, p, led=led)
            last_heartbeat_time = time.time()

        # Idle timeout → deep sleep
        if idle_elapsed >= TX_IDLE_SLEEP_S:
            logger.info("Radar idle %ds — deep sleep", TX_IDLE_SLEEP_S)
            oled.show_status("DEEP SLEEP", "Radar idle", f"{TX_IDLE_SLEEP_S}s", "Wake: sensor")
            go_to_deepsleep(oled=oled, radio=radio)

        # Опрос радара
        ts_map = {0: "Clear", 1: "Moving", 2: "Still", 3: "Mov+Stat"}

        if not radar.read_frame():
            if time.ticks_diff(time.ticks_ms(), radar_oled_timer) >= RADAR_OLED_MS:
                state = ts_map.get(radar.target_state, "?")
                v, pct, chrg = bat_monitor.get_status()
                oled.show_tx_armed(f"{state} {radar.detection_distance}cm",
                                   pct, chrg, idle_elapsed, TX_IDLE_SLEEP_S, tx_counter)
                radar_oled_timer = time.ticks_ms()
            time.sleep_ms(RADAR_POLL_MS)
            continue

        if radar.target_state == 0:
            if time.ticks_diff(time.ticks_ms(), radar_oled_timer) >= RADAR_OLED_MS:
                v, pct, chrg = bat_monitor.get_status()
                oled.show_tx_armed(f"Clear {radar.detection_distance}cm",
                                   pct, chrg, idle_elapsed, TX_IDLE_SLEEP_S, tx_counter)
                radar_oled_timer = time.ticks_ms()
            time.sleep_ms(RADAR_POLL_MS)
            continue

        # ── Обнаружен человек! ──────────────────────────────────────────
        last_detection_time = time.time()
        ts_map = {1: "Moving", 2: "Still", 3: "Mov+Stat"}
        state_str = ts_map.get(radar.target_state, "?")
        logger.warning("RADAR: %s at %dcm", state_str, radar.detection_distance)

        # Кулдаун
        if (now - last_alarm_time) < ALARM_COOLDOWN_S:
            remaining = ALARM_COOLDOWN_S - (now - last_alarm_time)
            oled.show_status("COOLDOWN", state_str,
                             f"{radar.detection_distance}cm",
                             f"Wait {int(remaining)}s", progress=0, antenna=True)
            time.sleep_ms(RADAR_POLL_MS)
            continue

        # Отправка тревоги
        p = build_status_payload(bat_monitor, radar, armed)
        p["msg"] = "Alarm!"

        size_est = 128
        expected_toa = radio.get_time_on_air_ms(size_est)
        oled.show_status("TX ALARM!", f"{state_str} {radar.detection_distance}cm",
                         f"Pkt#{tx_counter}", f"ToA:{expected_toa/1000:.1f}s",
                         progress=0, antenna=True)

        last_pct = 0
        def tx_progress(elapsed_ms):
            nonlocal last_pct
            pct = min(int((elapsed_ms / expected_toa) * 100), 100)
            if pct - last_pct >= 2:
                oled.update_progress_only(54, pct)
                last_pct = pct

        encrypted = crypto.encrypt_json(p)
        if encrypted:
            t_start = time.ticks_ms()
            ok = radio.transmit_payload(encrypted, timeout_ms=10000, progress_cb=tx_progress)
            actual_toa = time.ticks_diff(time.ticks_ms(), t_start)
            if ok:
                tx_counter += 1
                last_alarm_time = time.time()
                logger.info("TX Alarm OK (airtime %dms)", actual_toa)
                oled.show_status("TX OK", f"{state_str} {radar.detection_distance}cm",
                                 f"Air:{actual_toa/1000:.1f}s", progress=100, antenna=True)
                blink_ok(led)
                time.sleep(2)
            else:
                logger.error("TX Alarm TIMEOUT")
                oled.show_status("TX FAIL", "Radio timeout", progress=0)
                blink_fail(led)
                time.sleep_ms(500)
        else:
            logger.error("Encryption failed for alarm")
            blink_fail(led)


# =============================================================================
# RX Main Loop (брелок)
# =============================================================================

def rx_main_loop(radio, crypto, oled, bat_monitor, led, flag, wakeup_source):
    """Главный цикл RX-блока (брелок).

    Логика:
    - Слушаем эфир RX_LISTEN_TIMEOUT_MS мс.
    - Кнопка BOOT: отправить пинг машине, ждать ответ.
    - Таймаут → deep sleep (timer RX_SLEEP_INTERVAL_S + ext1 DIO9).

    Args:
        radio:          LR1121 — радиомодуль.
        crypto:         AESCryptoManager — дешифрование.
        oled:           OLEDDisplay — экран.
        bat_monitor:    BatteryMonitor — заряд батареи.
        led:            Pin — светодиод.
        flag:           TriggerFlag — флаг кнопки BOOT.
        wakeup_source:  str | None — источник пробуждения (для первого цикла).
    """
    global rx_counter, lost_counter, expected_c

    # Если проснулись по таймеру — сразу пингуем машину
    if wakeup_source == "timer":
        _rx_send_ping(radio, crypto, oled, bat_monitor, led)

    display_idle = False

    while True:
        # Кнопка BOOT → принудительный пинг машины
        if flag.is_triggered():
            _rx_send_ping(radio, crypto, oled, bat_monitor, led)
            display_idle = False
            continue

        # Показать статус ожидания
        if not display_idle:
            my_v, my_pct, my_chrg = bat_monitor.get_status()
            oled.show_rx_idle(my_pct, my_chrg, rx_counter, lost_counter)
            display_idle = True

        # Ожидание пакета
        msg = receive_packet(radio, crypto, RX_LISTEN_TIMEOUT_MS)

        if msg is not None:
            display_idle = False
            _rx_handle_packet(msg, radio, oled, bat_monitor, led)
            continue

        # Таймаут — deep sleep
        logger.info("RX timeout — deep sleep")
        oled.show_status("DEEP SLEEP", "Zzzz...")
        sleep_ms = RX_SLEEP_INTERVAL_S * 1000 if RX_SLEEP_INTERVAL_S > 0 else 0
        go_to_deepsleep(oled=oled, radio=radio, sleep_ms=sleep_ms)


def _rx_send_ping(radio, crypto, oled, bat_monitor, led):
    """Отправить пинг машине и дождаться ответа со статусом.

    Args:
        radio, crypto, oled, bat_monitor, led — как в rx_main_loop.
    """
    logger.info("Sending Ping to car...")
    oled.show_status("PING", "Requesting", "car status...", progress=0, antenna=True)

    v, pct, is_chrg = bat_monitor.get_status()
    p = {
        "msg": "Ping", "t": time.time(), "c": tx_counter,
        "v": v, "b": pct, "ch": is_chrg, "ts": 0, "dd": 0,
    }
    ok = send_packet(radio, crypto, p, led=led)
    if not ok:
        oled.show_status("PING FAIL", "TX error", progress=0)
        time.sleep(2)
        return

    # Ждём ответ
    oled.show_status("WAITING", "For response...", f"{RX_PING_TIMEOUT_MS//1000}s timeout",
                     progress=50, antenna=True)
    resp = receive_packet(radio, crypto, RX_PING_TIMEOUT_MS)

    if resp and resp.get("msg") in ("Status", "Heartbeat", "Armed", "Disarmed"):
        _rx_display_status(resp, radio, oled, bat_monitor)
        blink_ok(led)
    else:
        logger.warning("No response from car")
        oled.show_status("NO ANSWER", "Car offline?", "Try again", progress=0)
        blink_fail(led)
    time.sleep(3)


def _rx_handle_packet(msg, radio, oled, bat_monitor, led):
    """Обработать полученный пакет на RX.

    Типы пакетов: Alarm!, Vibration!, Heartbeat, Status, Armed, Disarmed, Radar Error.

    Args:
        msg:         dict — расшифрованный payload.
        radio:       LR1121 — для RSSI/SNR.
        oled:        OLEDDisplay — экран.
        bat_monitor: BatteryMonitor — локальная батарея.
        led:         Pin — светодиод.
    """
    global rx_counter, lost_counter, expected_c

    msg_type = msg.get("msg", "?")
    c_val    = msg.get("c", 0)
    ts_val   = msg.get("ts", 0)
    dd_val   = msg.get("dd", 0)
    is_armed = msg.get("armed", True)

    ts_map = {-2: "Vibr!", -1: "NoRadar", 0: "None", 1: "Moving", 2: "Still", 3: "Both"}
    ts_str = ts_map.get(ts_val, "?")

    logger.info("RX: '%s' pkt#%d ts=%s dd=%d armed=%s", msg_type, c_val, ts_str, dd_val, is_armed)

    # Счётчик потерь
    rx_counter += 1
    if expected_c is not None:
        if c_val > expected_c:
            lost_counter += (c_val - expected_c)
        elif c_val < expected_c:
            lost_counter = 0
            rx_counter = 1
    expected_c = c_val + 1

    rssi = getattr(radio, 'last_rssi', 0)
    snr  = getattr(radio, 'last_snr', 0)

    # Информативные пакеты (Heartbeat, Status, Armed, Disarmed)
    if msg_type in ("Heartbeat", "Status", "Armed", "Disarmed"):
        _rx_display_status(msg, radio, oled, bat_monitor)
        blink_ok(led)
        time.sleep(3)
        return

    # Тревожные пакеты (Alarm!, Vibration!, Radar Error)
    t_val = msg.get("t", 0)
    _, time_str = format_date_time(t_val)
    v     = msg.get("v", 0)
    pct   = msg.get("b", 0)
    chrg  = msg.get("ch", False)
    my_v, my_pct, my_chrg = bat_monitor.get_status()

    logger.warning("ALARM: '%s' | Car:%.1fV %d%% | Signal:R%.0f S%.1f",
                   msg_type, v, pct, rssi, snr)

    oled.show_rx_alarm(
        signal_str   = f"R:{rssi:.0f} S:{snr:.1f}",
        car_pct      = pct,
        car_charging = bool(chrg),
        my_pct       = my_pct,
        my_charging  = my_chrg,
        radar_str    = f"{ts_str} {dd_val}cm",
        stats_str    = f"{time_str[:5]} RX:{rx_counter} L:{lost_counter}",
    )
    blink_ok(led)
    time.sleep(5)


def _rx_display_status(msg, radio, oled, bat_monitor):
    """Отобразить информационный пакет (статус/heartbeat) на OLED RX.

    Args:
        msg:         dict — payload со статусом машины.
        radio:       LR1121 — для RSSI/SNR.
        oled:        OLEDDisplay — экран.
        bat_monitor: BatteryMonitor — локальная батарея.
    """
    msg_type = msg.get("msg", "?")
    is_armed = msg.get("armed", True)
    car_v    = msg.get("v", 0)
    car_pct  = msg.get("b", 0)
    car_chrg = msg.get("ch", False)
    ts_val   = msg.get("ts", 0)
    dd_val   = msg.get("dd", 0)
    rssi     = getattr(radio, 'last_rssi', 0)
    snr      = getattr(radio, 'last_snr', 0)
    my_v, my_pct, my_chrg = bat_monitor.get_status()

    arm_str = "ARMED" if is_armed else "DISARM"
    ts_map  = {-2: "Vibr!", -1: "NoRdr", 0: "Clear", 1: "Mov", 2: "Stat", 3: "Both"}
    ts_str  = ts_map.get(ts_val, "?")

    oled.show_status(
        f"CAR {arm_str}",
        f"R:{rssi:.0f} S:{snr:.1f}",
        f"Car:{car_pct}%{'+'if car_chrg else ''} {ts_str} {dd_val}cm",
        f"Me:{my_pct}%{'+'if my_chrg else ''} {msg_type[:8]}",
        progress=100 if is_armed else 0, antenna=True,
    )


# =============================================================================
# Точка входа
# =============================================================================

def main():
    """Инициализация системы и запуск главного цикла в зависимости от MODE."""
    global tx_counter

    need_radio_init = True
    wakeup_source = None  # "vibration" | "radar_out" | "radio" | "timer"

    # --- Boot ---
    logger.info("=== LoRa Car Alarm v%s ===", FIRMWARE_VERSION)
    logger.info("=== MODE: %s | Board: T3S3 v1.1 ===", MODE)

    bat_monitor = BatteryMonitor(adc_pin=BATTERY_ADC)
    led = Pin(LED_PIN, Pin.OUT, value=0)

    flag = TriggerFlag()
    trigger = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_UP)
    trigger.irq(flag.isr, trigger=Pin.IRQ_FALLING)

    # OLED
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    oled = OLEDDisplay(i2c)
    oled.show_status("BOOTING", f"v{FIRMWARE_VERSION}", f"Mode: {MODE}", progress=10)

    # --- Радар (только TX) ---
    radar = None
    if MODE == "TX" and LD2410B is not None:
        try:
            radar = LD2410B(uart_id=RADAR_UART_ID, tx_pin=RADAR_TX_PIN, rx_pin=RADAR_RX_PIN)
            logger.info("LD2410B UART opened. Probing for %dms...", RADAR_CHECK_MS)
            oled.show_status("RADAR", "Checking...", progress=20)
            radar_ok = False
            check_start = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), check_start) < RADAR_CHECK_MS:
                if radar.read_frame():
                    radar_ok = True
                    break
                time.sleep_ms(50)
            if radar_ok:
                logger.info("LD2410B radar OK")
            else:
                logger.error("LD2410B not responding — disabled")
                radar = None
        except Exception as e:
            logger.error("Radar init failed: %s", e)

    # --- Wakeup analysis ---
    reset_cause = machine.reset_cause()
    wake_reason = machine.wake_reason()
    logger.debug("reset_cause=%d wake_reason=%d", reset_cause, wake_reason)

    if reset_cause == machine.DEEPSLEEP_RESET:
        if wake_reason == machine.TIMER_WAKE:
            wakeup_source = "timer"
            logger.info("WOKE UP: timer (periodic)")
            oled.show_status("WAKE", "Timer", progress=100)
            time.sleep(1)

        elif wake_reason == machine.EXT1_WAKE:
            logger.warning("WOKE UP: ext1 sensor")
            oled.show_status("WAKE", "Sensor!", progress=100)

            wake_pins = ()
            if hasattr(machine, 'wake_description'):
                wake_pins = machine.wake_description()
                logger.info("Wake pins: %s", str(wake_pins))

            # GPIO 11 = вибрация (однозначно)
            if wake_pins and SHOCK_PIN in wake_pins:
                wakeup_source = "vibration"
                logger.warning("Vibration sensor triggered!")
            # GPIO 8: radar OUT (TX) или radio DIO9 (RX)
            elif wake_pins and WAKE_PIN in wake_pins:
                if MODE == "TX":
                    wakeup_source = "radar_out"
                    logger.warning("Radar OUT triggered!")
                else:
                    wakeup_source = "radio"
                    need_radio_init = False
                    logger.info("Radio DIO9 triggered (packet)")
            else:
                # wake_description() недоступен
                if MODE == "TX":
                    wakeup_source = "radar_out"
                else:
                    wakeup_source = "radio"
                    need_radio_init = False
                logger.warning("Unknown wake pin, assuming %s", wakeup_source)

            time.sleep(1)
        else:
            logger.info("WOKE UP: other reason (%d)", wake_reason)
    else:
        logger.info("=== COLD BOOT ===")

    # --- Wi-Fi & NTP ---
    connect_wifi_and_sync(oled)

    # --- Радио ---
    logger.debug("Configuring SPI...")
    spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0,
              sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN),
                   rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))

    if need_radio_init:
        logger.info("Initializing LR1121...")
        radio.init_radio()
    else:
        logger.info("Skipping radio init (wakeup)")

    # --- Крипто ---
    crypto = AESCryptoManager(key_path="secret.key")

    # --- System Ready ---
    oled.show_status(f"{MODE} READY", f"v{FIRMWARE_VERSION}",
                     "Press BOOT", "for action", antenna=True)
    time.sleep(2)

    # --- Wakeup alarm dispatch (TX) ---
    if MODE == "TX" and wakeup_source in ("vibration", "radar_out"):
        p = build_status_payload(bat_monitor, radar, True)
        if wakeup_source == "vibration":
            p["msg"] = "Vibration!"
            p["ts"] = -2
            oled.show_status("TX VIBR!", "Shock alarm", f"Pkt#{tx_counter}", progress=0, antenna=True)
            logger.warning("Sending vibration alarm")
        else:
            # Попробовать прочитать UART для деталей
            if radar:
                check_start = time.ticks_ms()
                while time.ticks_diff(time.ticks_ms(), check_start) < 1000:
                    if radar.read_frame():
                        p["ts"] = radar.target_state
                        p["dd"] = radar.detection_distance
                        break
                    time.sleep_ms(50)
            p["msg"] = "Alarm!"
            oled.show_status("TX ALARM!", "Radar wake", f"Pkt#{tx_counter}", progress=0, antenna=True)
            logger.warning("Sending radar wake alarm")
        send_packet(radio, crypto, p, led=led)
        time.sleep(2)

    # --- Главный цикл ---
    if MODE == "TX":
        tx_main_loop(radio, crypto, oled, bat_monitor, radar, led, flag)
    else:
        rx_main_loop(radio, crypto, oled, bat_monitor, led, flag, wakeup_source)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.exception("FATAL: %s", str(e))
        try:
            i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
            oled = OLEDDisplay(i2c)
            oled.show_status("CRASHED", "See console", str(e)[:15])
        except:
            pass
