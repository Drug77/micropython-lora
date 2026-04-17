"""
LoRa Car Alarm v2.1.0 — единая прошивка для TX (машина) и RX (брелок).

Режим: config.json → "mode": "TX" | "RX".
TX: радар + вибрация → тревоги по LoRa, arm/disarm от RX.
RX: пульт управления, WiFi/NTP, arm/disarm, ping, приём тревог.
Кнопка: short press + long press (2с).

Плата: LilyGO T3S3 v1.1 (ESP32-S3 + LR1121 + SSD1306 OLED + LiPo).
"""

FIRMWARE_VERSION = "2.1.0"

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

from lang import set_lang, t
from lr1121 import LR1121, LR1121_SPI_BAUDRATE
from oled import OLEDDisplay

# --- Конфигурация устройства (config.json) ---
def _load_config(path="config.json"):
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
SLEEP_ENABLED   = _cfg.get("sleep", True)
set_lang(_cfg.get("lang", "ru"))

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
TRIGGER_PIN = 0
LED_PIN     = 37
WAKE_PIN    = 8       # ext1: radio DIO9 / radar OUT
SHOCK_PIN   = 11      # ext1: датчик вибрации (TX)
BATTERY_ADC = 1

RADAR_UART_ID = 1
RADAR_TX_PIN  = 9
RADAR_RX_PIN  = 10

# =============================================================================
# Константы времени
# =============================================================================
ALARM_COOLDOWN_S     = 10
RADAR_POLL_MS        = 20
RADAR_OLED_MS        = 1000
RADAR_CHECK_MS       = 3000
LONG_PRESS_MS        = 2000   # Порог long press (мс)

TX_IDLE_SLEEP_S      = 120
TX_HEARTBEAT_S       = 60
TX_LISTEN_INTERVAL_S = 5
TX_LISTEN_WINDOW_MS  = 20000

RX_LISTEN_TIMEOUT_MS = 30000
RX_PING_TIMEOUT_MS   = 40000
RX_SLEEP_INTERVAL_S  = 300
RX_LISTEN_CHUNK_MS   = 10000  # Чанк приёма (>ToA пакета ~8с при SF12)

# =============================================================================
# Глобальные счётчики и состояние
# =============================================================================
tx_counter = 0
rx_counter = 0
lost_counter = 0
expected_c = None
remote_bat_pct = 0
armed = True

# =============================================================================
# Персистентное состояние (RTC memory, выживает deep sleep)
# =============================================================================

def _save_state():
    """Сохранить armed + remote_bat_pct в RTC memory."""
    machine.RTC().memory(bytes([1 if armed else 0, min(remote_bat_pct, 255)]))

def _load_state():
    """Загрузить состояние из RTC memory."""
    global armed, remote_bat_pct
    try:
        mem = machine.RTC().memory()
        if mem and len(mem) >= 2:
            armed = (mem[0] == 1)
            remote_bat_pct = mem[1]
            return
    except:
        pass
    armed = True
    remote_bat_pct = 0

def _update_remote_battery(msg):
    """Обновить кэш батареи удалённого устройства из пакета."""
    global remote_bat_pct
    if "b" in msg:
        remote_bat_pct = msg["b"]
        _save_state()

def _sync_time_from_packet(msg):
    """Синхронизировать RTC из timestamp пакета."""
    t_val = msg.get("t", 0)
    if t_val > 946684800:  # После 2000-01-01
        try:
            tm = time.gmtime(t_val)
            machine.RTC().datetime((tm[0], tm[1], tm[2], tm[6], tm[3], tm[4], tm[5], 0))
            logger.debug("RTC synced from packet: %d", t_val)
        except Exception as e:
            logger.debug("RTC sync failed: %s", e)

# =============================================================================
# Deep Sleep
# =============================================================================

def go_to_deepsleep(oled=None, radio=None, sleep_ms=0):
    """Deep sleep с ext0 (кнопка) + ext1 (sensor/radio).

    Оба режима: radio в continuous RX для пробуждения от LoRa.
    """
    _save_state()

    if oled and hasattr(oled, 'display') and oled.display:
        oled.display.poweroff()

    if radio:
        radio.prepare_for_deepsleep()

    # ext0: кнопка BOOT (GPIO 0, active LOW)
    if hasattr(esp32, 'wake_on_ext0'):
        esp32.wake_on_ext0(pin=Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_UP),
                           level=esp32.WAKEUP_ALL_LOW)

    # ext1: sensor/radio pins
    wake_pins = [Pin(WAKE_PIN, Pin.IN, Pin.PULL_DOWN)]
    if MODE == "TX":
        wake_pins.append(Pin(SHOCK_PIN, Pin.IN, Pin.PULL_DOWN))
    esp32.wake_on_ext1(pins=tuple(wake_pins), level=esp32.WAKEUP_ANY_HIGH)

    logger.info("Deep sleep (timer=%dms)", sleep_ms)
    time.sleep_ms(100)
    if sleep_ms > 0:
        machine.deepsleep(sleep_ms)
    else:
        machine.deepsleep()

# =============================================================================
# Утилиты
# =============================================================================

def format_date_time(timestamp=None):
    if timestamp is None:
        timestamp = time.time()
    local_time = time.localtime(timestamp + (TIMEZONE_OFFSET * 3600))
    wd = t("days")[local_time[6]]
    date_str = f"{wd} {local_time[2]:02d}.{local_time[1]:02d}.{local_time[0]}"
    time_str = f"{local_time[3]:02d}:{local_time[4]:02d}:{local_time[5]:02d}"
    return date_str, time_str


def connect_wifi_and_sync(oled):
    """Wi-Fi + NTP sync. Только для RX при холодном старте."""
    if WIFI_SSID == "YOUR_WIFI_NAME":
        logger.warning("Default Wi-Fi — skipping")
        oled.show_status("WIFI SKIP", t("default_cfg"), progress=100)
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
            logger.error("Wi-Fi error: %s", e)
        for _ in range(50):
            if wlan.isconnected():
                break
            time.sleep_ms(100)
        if wlan.isconnected():
            break

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        logger.info("Wi-Fi OK: %s", ip)
        oled.show_status("WIFI OK", ip, progress=90)
        time.sleep(1)
        try:
            ntptime.settime()
            d_s, t_s = format_date_time()
            logger.info("NTP synced: %s %s", d_s, t_s)
        except Exception as e:
            logger.warning("NTP failed: %s", e)
    else:
        logger.warning("Wi-Fi failed")
        oled.show_status("WIFI FAIL", t("offline"), progress=100)
        time.sleep(1)

    if wlan.isconnected():
        wlan.disconnect()
    wlan.active(False)


def blink_tx_ok(led):
    """TX завершён — погасить (был включён во время передачи)."""
    led.value(0)

def blink_rx_ok(led):
    """Пакет получен — 2 короткие вспышки."""
    for _ in range(2):
        led.value(1); time.sleep_ms(80)
        led.value(0); time.sleep_ms(80)

def blink_fail(led):
    """Ошибка — 3 быстрых мигания."""
    for _ in range(3):
        led.value(1); time.sleep_ms(50); led.value(0); time.sleep_ms(50)


class TriggerFlag:
    """Кнопка с определением short/long press через ISR + schedule.

    ISR захватывает pin.value() синхронно, schedule обрабатывает.
    FALLING: записывает время. RISING: вычисляет длительность.
    check() неблокирующий.
    """
    def __init__(self):
        self._result = None
        self._press_time = 0

    def _handle(self, val):
        now = time.ticks_ms()
        if val == 0:
            # Кнопка нажата
            self._press_time = now
        else:
            # Кнопка отпущена
            if self._press_time > 0:
                duration = time.ticks_diff(now, self._press_time)
                self._result = "long" if duration >= LONG_PRESS_MS else "short"
                self._press_time = 0

    def isr(self, pin):
        val = pin.value()  # Захват значения в ISR контексте
        try:
            micropython.schedule(self._handle, val)
        except RuntimeError:
            pass  # Schedule queue full — пропустить

    def check(self, pin=None):
        """Неблокирующая проверка. Returns: 'short', 'long', или None."""
        r = self._result
        if r is not None:
            self._result = None
            logger.info("Button: %s", r)
        return r


# =============================================================================
# Коммуникация
# =============================================================================

def send_packet(radio, crypto, payload, led=None):
    """Зашифровать и отправить payload по LoRa."""
    global tx_counter
    encrypted = crypto.encrypt_json(payload)
    if not encrypted:
        logger.error("Encrypt fail: msg=%s", payload.get("msg"))
        if led: blink_fail(led)
        return False

    logger.info("TX %d bytes: msg=%s", len(encrypted), payload.get("msg"))
    if led: led.value(1)  # LED ON во время передачи
    ok = radio.transmit_payload(encrypted, timeout_ms=10000)
    if ok:
        tx_counter += 1
        logger.info("TX_DONE #%d", tx_counter)
        if led: blink_tx_ok(led)
    else:
        logger.error("TX timeout")
        if led: blink_fail(led)
    return ok


def receive_packet(radio, crypto, timeout_ms):
    """Принять и расшифровать пакет."""
    raw = radio.receive_payload(timeout_ms=timeout_ms, max_len=255)
    if raw is None:
        return None
    logger.info("RX %d bytes", len(raw))
    obj = crypto.decrypt_json(raw)
    if obj is None:
        logger.error("Decrypt failed")
    return obj


def build_status_payload(bat_monitor, radar=None):
    """Собрать payload со статусом устройства (без поля msg)."""
    v, pct, is_chrg = bat_monitor.get_status()
    ts_val, dd_val = 0, 0
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

def tx_main_loop(radio, crypto, oled, bat_monitor, radar, led, flag, trigger):
    """TX main loop: patrol, listen for RX commands, SOS on long press."""
    global tx_counter, armed

    last_alarm_time = 0
    last_detection_time = time.time()
    last_heartbeat_time = time.time()
    last_listen_time = time.time()
    radar_oled_timer = time.ticks_ms()

    logger.info("TX loop: Armed=%s Radar=%s", armed, radar is not None)

    while True:
        now = time.time()

        # ── Кнопка: short=игнор, long=SOS ──────────────────────────
        btn = flag.check(trigger)
        if btn == "short":
            logger.info("Short press ignored (TX active)")
        elif btn == "long":
            logger.warning("SOS button!")
            p = build_status_payload(bat_monitor, radar)
            p["msg"] = "SOS"
            v, pct, chrg = bat_monitor.get_status()
            oled.show_sos_sent(pct, remote_bat=remote_bat_pct)
            send_packet(radio, crypto, p, led=led)
            time.sleep(3)
            if SLEEP_ENABLED:
                go_to_deepsleep(oled=oled, radio=radio)

        # ── DISARMED: always listen + periodic heartbeat ─────────────
        if not armed:
            # Heartbeat check
            if (now - last_heartbeat_time) >= TX_HEARTBEAT_S:
                p = build_status_payload(bat_monitor, radar)
                p["msg"] = "Heartbeat"
                send_packet(radio, crypto, p, led=led)
                last_heartbeat_time = time.time()

            # Always listen (no gaps)
            v, pct, chrg = bat_monitor.get_status()
            oled.show_tx_disarmed(pct, chrg, tx_counter, remote_bat=remote_bat_pct)
            msg = receive_packet(radio, crypto, TX_LISTEN_WINDOW_MS)
            if msg:
                _tx_handle_rx_command(msg, radio, crypto, oled, bat_monitor, radar, led)
            continue

        # ── ARMED: periodic listen window ───────────────────────────
        if (now - last_listen_time) >= TX_LISTEN_INTERVAL_S:
            logger.debug("Listen window %dms", TX_LISTEN_WINDOW_MS)
            v, pct, chrg = bat_monitor.get_status()
            oled.show_status(t("listen"), t("rx_cmd"), progress=0,
                             bat1=pct, bat2=remote_bat_pct, locked=armed)
            msg = receive_packet(radio, crypto, TX_LISTEN_WINDOW_MS)
            if msg:
                _tx_handle_rx_command(msg, radio, crypto, oled, bat_monitor, radar, led)
            last_listen_time = time.time()
            continue

        # ── ARMED без радара → ошибка + sleep или disarm ────────────
        if radar is None:
            p = build_status_payload(bat_monitor)
            p["msg"] = "Radar Error"
            p["ts"] = -1
            oled.show_status("CAR ERR", t("no_radar"), t("sleeping"))
            send_packet(radio, crypto, p, led=led)
            time.sleep(2)
            if SLEEP_ENABLED:
                go_to_deepsleep(oled=oled, radio=radio)
            else:
                armed = False
                _save_state()
                logger.info("No radar + no sleep → disarmed")
            continue

        # ── ARMED + радар: патрулирование ───────────────────────────
        idle_elapsed = now - last_detection_time

        if (now - last_heartbeat_time) >= TX_HEARTBEAT_S:
            p = build_status_payload(bat_monitor, radar)
            p["msg"] = "Heartbeat"
            send_packet(radio, crypto, p, led=led)
            last_heartbeat_time = time.time()

        if idle_elapsed >= TX_IDLE_SLEEP_S:
            logger.info("Idle %ds — sleep", TX_IDLE_SLEEP_S)
            oled.show_status(t("sleep"), t("radar_idle"), f"{TX_IDLE_SLEEP_S}s")
            if SLEEP_ENABLED:
                go_to_deepsleep(oled=oled, radio=radio)
            else:
                last_detection_time = time.time()  # Reset idle timer

        ts_map = {0: t("clear"), 1: t("moving"), 2: t("still"), 3: t("movstat")}

        if not radar.read_frame():
            if time.ticks_diff(time.ticks_ms(), radar_oled_timer) >= RADAR_OLED_MS:
                state = ts_map.get(radar.target_state, "?")
                v, pct, chrg = bat_monitor.get_status()
                oled.show_tx_armed(state, f"{radar.detection_distance}cm",
                                   pct, chrg, idle_elapsed, TX_IDLE_SLEEP_S,
                                   tx_counter, remote_bat=remote_bat_pct)
                radar_oled_timer = time.ticks_ms()
            time.sleep_ms(RADAR_POLL_MS)
            continue

        if radar.target_state == 0:
            if time.ticks_diff(time.ticks_ms(), radar_oled_timer) >= RADAR_OLED_MS:
                v, pct, chrg = bat_monitor.get_status()
                oled.show_tx_armed(t("clear"), f"{radar.detection_distance}cm",
                                   pct, chrg, idle_elapsed, TX_IDLE_SLEEP_S,
                                   tx_counter, remote_bat=remote_bat_pct)
                radar_oled_timer = time.ticks_ms()
            time.sleep_ms(RADAR_POLL_MS)
            continue

        # ── Обнаружен человек! ──────────────────────────────────────
        last_detection_time = time.time()
        det_map = {1: t("moving"), 2: t("still"), 3: t("movstat")}
        state_str = det_map.get(radar.target_state, "?")
        logger.warning("RADAR: %s at %dcm", state_str, radar.detection_distance)

        if (now - last_alarm_time) < ALARM_COOLDOWN_S:
            time.sleep_ms(RADAR_POLL_MS)
            continue

        p = build_status_payload(bat_monitor, radar)
        p["msg"] = "Alarm!"
        size_est = 128
        expected_toa = radio.get_time_on_air_ms(size_est)
        oled.show_status("TX ALARM!", f"{state_str} {radar.detection_distance}cm",
                         f"Pkt#{tx_counter}", f"ToA:{expected_toa/1000:.1f}s",
                         progress=0, )

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
                logger.info("Alarm OK (%dms)", actual_toa)
                blink_rx_ok(led)
                time.sleep(2)
            else:
                logger.error("Alarm TX timeout")
                blink_fail(led)
                time.sleep_ms(500)


def _tx_handle_rx_command(msg, radio, crypto, oled, bat_monitor, radar, led):
    """Обработать команду от RX в listen window."""
    global armed
    msg_type = msg.get("msg")

    _sync_time_from_packet(msg)
    _update_remote_battery(msg)

    if msg_type == "Arm":
        armed = not armed
        _save_state()
        state = "Armed" if armed else "Disarmed"
        logger.warning("RX command: %s", state)
        # Задержка: дать RX время перейти из TX в RX mode
        time.sleep(2)
        p = build_status_payload(bat_monitor, radar)
        p["msg"] = state
        send_packet(radio, crypto, p, led=led)
        v, pct, chrg = bat_monitor.get_status()
        oled.show_status(state, t("from_fob"), progress=100 if armed else 0,
                         bat1=pct, bat2=remote_bat_pct, locked=armed)
        time.sleep(2)

    elif msg_type == "Ping":
        logger.info("Ping from RX")
        p = build_status_payload(bat_monitor, radar)
        p["msg"] = "Status"
        send_packet(radio, crypto, p, led=led)
        blink_rx_ok(led)


# =============================================================================
# RX Main Loop (брелок)
# =============================================================================

def rx_main_loop(radio, crypto, oled, bat_monitor, led, flag, trigger, wakeup_source):
    """RX main loop: listen for TX, arm/ping on button, deep sleep on timeout."""
    global rx_counter, lost_counter, expected_c

    # Wake dispatch: button or timer
    if wakeup_source == "button_short":
        _rx_send_arm(radio, crypto, oled, bat_monitor, led)
    elif wakeup_source == "button_long":
        _rx_send_ping(radio, crypto, oled, bat_monitor, led)
    elif wakeup_source == "timer":
        _rx_send_ping(radio, crypto, oled, bat_monitor, led)

    # Main listen loop with chunked receive for button responsiveness
    listen_start = time.ticks_ms()
    last_heartbeat_ts = 0  # timestamp последнего пульса

    while True:
        # Check button
        btn = flag.check(trigger)
        if btn == "short":
            _rx_send_arm(radio, crypto, oled, bat_monitor, led)
            listen_start = time.ticks_ms()
            continue
        elif btn == "long":
            _rx_send_ping(radio, crypto, oled, bat_monitor, led)
            listen_start = time.ticks_ms()
            continue

        # Show idle status
        my_v, my_pct, my_chrg = bat_monitor.get_status()
        d_str, t_str = format_date_time()
        # Heartbeat: "HH:MM:SS (XXс)"
        hb_s = ""
        if last_heartbeat_ts > 0:
            _, hb_time = format_date_time(last_heartbeat_ts)
            ago = int(time.time() - last_heartbeat_ts)
            nxt = max(0, TX_HEARTBEAT_S - ago)
            hb_s = f"{hb_time} ({nxt}\u0441)"
        lock_state = armed if last_heartbeat_ts > 0 else None
        oled.show_rx_idle(my_pct, my_chrg, rx_counter, lost_counter,
                          remote_bat=remote_bat_pct, locked=lock_state,
                          date_str=d_str, time_str=t_str, hb_str=hb_s)

        # Listen in 1s slices within a chunk (for countdown accuracy)
        chunk_start = time.ticks_ms()
        got_packet = False
        while time.ticks_diff(time.ticks_ms(), chunk_start) < RX_LISTEN_CHUNK_MS:
            elapsed = time.ticks_diff(time.ticks_ms(), listen_start)
            if elapsed >= RX_LISTEN_TIMEOUT_MS:
                break
            msg = receive_packet(radio, crypto, 1000)
            if msg is not None:
                _rx_handle_packet(msg, radio, oled, bat_monitor, led, flag)
                last_heartbeat_ts = time.time()
                listen_start = time.ticks_ms()
                got_packet = True
                break
            # Update countdown on screen every second
            if last_heartbeat_ts > 0:
                ago = int(time.time() - last_heartbeat_ts)
                nxt = max(0, TX_HEARTBEAT_S - ago)
                _, hb_time = format_date_time(last_heartbeat_ts)
                hb_s = f"{hb_time} ({nxt}\u0441)"
                oled.show_rx_idle(my_pct, my_chrg, rx_counter, lost_counter,
                                  remote_bat=remote_bat_pct, locked=armed,
                                  date_str=d_str, time_str=t_str, hb_str=hb_s)

        # Timeout check
        elapsed = time.ticks_diff(time.ticks_ms(), listen_start)
        if elapsed >= RX_LISTEN_TIMEOUT_MS:
            if SLEEP_ENABLED:
                logger.info("RX timeout — sleep")
                oled.show_status(t("sleep"), t("zzzz"))
                sleep_ms = RX_SLEEP_INTERVAL_S * 1000 if RX_SLEEP_INTERVAL_S > 0 else 0
                go_to_deepsleep(oled=oled, radio=radio, sleep_ms=sleep_ms)
                return
            else:
                logger.debug("RX timeout — restart (sleep disabled)")
                listen_start = time.ticks_ms()


def _rx_send_arm(radio, crypto, oled, bat_monitor, led):
    """Отправить toggle arm/disarm на TX, дождаться подтверждения."""
    logger.info("Sending Arm toggle...")
    v, pct, chrg = bat_monitor.get_status()
    oled.show_status(t("arm"), t("sending"), progress=0,                      bat1=pct, bat2=remote_bat_pct)

    p = {"msg": "Arm", "t": time.time(), "c": tx_counter,
         "v": v, "b": pct, "ch": chrg, "ts": 0, "dd": 0}
    ok = send_packet(radio, crypto, p, led=led)
    if not ok:
        oled.show_status("ARM FAIL", t("tx_error"), progress=0)
        time.sleep(2)
        return

    oled.show_status(t("wait"), t("confirm"), f"{RX_PING_TIMEOUT_MS//1000}s",
                     progress=50, bat1=pct, bat2=remote_bat_pct)
    # Retry до получения Armed/Disarmed (TX может отправить другие пакеты первыми)
    t_wait = time.ticks_ms()
    confirmed = False
    while time.ticks_diff(time.ticks_ms(), t_wait) < RX_PING_TIMEOUT_MS:
        remaining = RX_PING_TIMEOUT_MS - time.ticks_diff(time.ticks_ms(), t_wait)
        if remaining <= 0:
            break
        resp = receive_packet(radio, crypto, min(remaining, 10000))
        if resp:
            _update_remote_battery(resp)
            _sync_time_from_packet(resp)
            if resp.get("msg") in ("Armed", "Disarmed"):
                global armed
                armed = resp.get("armed", True)
                _rx_display_status(resp, radio, oled, bat_monitor)
                blink_rx_ok(led)
                confirmed = True
                break
            else:
                logger.info("Got '%s' while waiting for arm confirm", resp.get("msg"))
    if not confirmed:
        logger.warning("No arm confirmation")
        oled.show_status(t("no_answer"), t("car_offline"), progress=0)
        blink_fail(led)
    time.sleep(3)


def _rx_send_ping(radio, crypto, oled, bat_monitor, led):
    """Отправить ping на TX, дождаться статуса."""
    logger.info("Sending Ping...")
    v, pct, chrg = bat_monitor.get_status()
    oled.show_status(t("ping"), t("requesting"), progress=0,                      bat1=pct, bat2=remote_bat_pct)

    p = {"msg": "Ping", "t": time.time(), "c": tx_counter,
         "v": v, "b": pct, "ch": chrg, "ts": 0, "dd": 0}
    ok = send_packet(radio, crypto, p, led=led)
    if not ok:
        oled.show_status("PING FAIL", t("tx_error"), progress=0)
        time.sleep(2)
        return

    oled.show_status(t("wait"), t("response"), f"{RX_PING_TIMEOUT_MS//1000}s",
                     progress=50, )
    t_wait = time.ticks_ms()
    confirmed = False
    while time.ticks_diff(time.ticks_ms(), t_wait) < RX_PING_TIMEOUT_MS:
        remaining = RX_PING_TIMEOUT_MS - time.ticks_diff(time.ticks_ms(), t_wait)
        if remaining <= 0:
            break
        resp = receive_packet(radio, crypto, min(remaining, 10000))
        if resp:
            _update_remote_battery(resp)
            _sync_time_from_packet(resp)
            if resp.get("msg") in ("Status", "Heartbeat", "Armed", "Disarmed"):
                _rx_display_status(resp, radio, oled, bat_monitor)
                blink_rx_ok(led)
                confirmed = True
                break
    if not confirmed:
        logger.warning("No ping response")
        oled.show_status(t("no_answer"), t("car_offline"), progress=0)
        blink_fail(led)
    time.sleep(3)


def _rx_handle_packet(msg, radio, oled, bat_monitor, led, flag):
    """Обработать пакет на RX: alarm, heartbeat, status, SOS."""
    global rx_counter, lost_counter, expected_c, armed

    msg_type = msg.get("msg", "?")
    c_val    = msg.get("c", 0)
    ts_val   = msg.get("ts", 0)
    dd_val   = msg.get("dd", 0)
    is_armed = msg.get("armed", True)

    ts_map = {-2: t("vibr"), -1: t("no_rdr"), 0: t("none"), 1: t("moving"), 2: t("still"), 3: t("both")}
    ts_str = ts_map.get(ts_val, "?")

    logger.info("RX: '%s' #%d ts=%s dd=%d armed=%s", msg_type, c_val, ts_str, dd_val, is_armed)

    # Счётчик потерь
    rx_counter += 1
    if expected_c is not None:
        if c_val > expected_c:
            lost_counter += (c_val - expected_c)
        elif c_val < expected_c:
            lost_counter = 0
            rx_counter = 1
    expected_c = c_val + 1

    armed = is_armed
    _update_remote_battery(msg)
    _sync_time_from_packet(msg)

    rssi = getattr(radio, 'last_rssi', 0)
    snr  = getattr(radio, 'last_snr', 0)

    # SOS — экстренный сигнал (остаётся пока не нажмут кнопку)
    if msg_type == "SOS":
        my_v, my_pct, my_chrg = bat_monitor.get_status()
        car_pct = msg.get("b", 0)
        oled.show_rx_alarm(
            signal_str=f"R:{rssi:.0f} S:{snr:.1f}",
            car_pct=car_pct, car_charging=bool(msg.get("ch", False)),
            my_pct=my_pct, my_charging=my_chrg,
            radar_str=t("emergency"), stats_str=t("sos_car"),
            is_sos=True,
        )
        # Мигаем + ждём нажатия кнопки
        btn_pin = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_UP)
        while True:
            led.value(1); time.sleep_ms(200)
            led.value(0); time.sleep_ms(200)
            if flag.check() is not None:
                break
        return

    # Информативные пакеты
    if msg_type in ("Heartbeat", "Status", "Armed", "Disarmed"):
        _rx_display_status(msg, radio, oled, bat_monitor)
        blink_rx_ok(led)
        time.sleep(10 if msg_type == "Heartbeat" else 3)
        return

    # Тревожные пакеты
    t_val = msg.get("t", 0)
    _, time_str = format_date_time(t_val)
    car_pct = msg.get("b", 0)
    car_chrg = msg.get("ch", False)
    my_v, my_pct, my_chrg = bat_monitor.get_status()

    logger.warning("ALARM: '%s' | Car:%d%% | R:%.0f S:%.1f", msg_type, car_pct, rssi, snr)

    oled.show_rx_alarm(
        signal_str=f"R:{rssi:.0f} S:{snr:.1f}",
        car_pct=car_pct, car_charging=bool(car_chrg),
        my_pct=my_pct, my_charging=my_chrg,
        radar_str=f"{ts_str} {dd_val}cm",
        stats_str=f"{time_str[:5]} RX:{rx_counter} L:{lost_counter}",
    )
    blink_rx_ok(led)
    time.sleep(5)


def _rx_display_status(msg, radio, oled, bat_monitor):
    """Отобразить информационный пакет на OLED."""
    msg_type = msg.get("msg", "?")
    is_armed = msg.get("armed", True)
    car_pct  = msg.get("b", 0)
    ts_val   = msg.get("ts", 0)
    dd_val   = msg.get("dd", 0)
    rssi     = getattr(radio, 'last_rssi', 0)
    snr      = getattr(radio, 'last_snr', 0)
    my_v, my_pct, my_chrg = bat_monitor.get_status()

    ts_map = {-2: t("vibr"), -1: t("no_rdr"), 0: t("clear"), 1: t("moving"), 2: t("still"), 3: t("both")}
    ts_str = ts_map.get(ts_val, "?")

    oled.show_rx_box(
        title=t("msg_" + msg_type)[:7] if msg_type else "?",
        arm_str=t('armed') if is_armed else t('disarmed'),
        radar_str=ts_str,
        signal_str=f"R:{rssi:.0f} S:{snr:.1f}",
        distance=f"{dd_val}cm",
        stats_str=f"{t('rx_lbl')}:{rx_counter} {t('lost_lbl')}:{lost_counter}",
        bat1=my_pct, bat2=car_pct,
        locked=is_armed,
    )


# =============================================================================
# Точка входа
# =============================================================================

def main():
    global tx_counter, armed, remote_bat_pct

    # === Раннее определение wake source (до тяжёлой инициализации) ===
    reset_cause = machine.reset_cause()
    is_deepsleep = (reset_cause == machine.DEEPSLEEP_RESET)
    wakeup_source = None
    button_long = False
    need_radio_init = True

    led = Pin(LED_PIN, Pin.OUT, value=0)

    # Загрузить persisted state
    _load_state()

    logger.info("=== LoRa Alarm v%s | %s ===", FIRMWARE_VERSION, MODE)

    # --- Определение источника пробуждения ---
    if is_deepsleep:
        wake_reason = machine.wake_reason()
        logger.debug("Deep sleep wake: reason=%d", wake_reason)

        EXT0_VAL = getattr(machine, 'EXT0_WAKE', getattr(machine, 'PIN_WAKE', 2))

        if wake_reason == EXT0_VAL:
            # Кнопка wake — определить short/long СРАЗУ (до ISR setup)
            wakeup_source = "button"
            btn_pin = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_UP)
            led.value(1)
            start = time.ticks_ms()
            while btn_pin.value() == 0:
                if time.ticks_diff(time.ticks_ms(), start) >= LONG_PRESS_MS:
                    button_long = True
                    for _ in range(3):
                        led.value(0); time.sleep_ms(50)
                        led.value(1); time.sleep_ms(50)
                    break
                time.sleep_ms(50)
            led.value(0)
            logger.info("Button wake: %s", "long" if button_long else "short")

        elif wake_reason == machine.TIMER_WAKE:
            wakeup_source = "timer"
            logger.info("Timer wake")

        elif wake_reason == machine.EXT1_WAKE:
            logger.warning("Ext1 wake (sensor/radio)")
            wake_pins = ()
            if hasattr(machine, 'wake_description'):
                wake_pins = machine.wake_description()
                logger.info("Wake pins: %s", str(wake_pins))

            if wake_pins and SHOCK_PIN in wake_pins:
                wakeup_source = "vibration"
            else:
                wakeup_source = "radio_or_sensor"
                need_radio_init = False
        else:
            logger.info("Other wake: %d", wake_reason)
    else:
        logger.info("=== COLD BOOT ===")

    # === Инициализация hardware ===
    bat_monitor = BatteryMonitor(adc_pin=BATTERY_ADC)

    flag = TriggerFlag()
    trigger = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_UP)
    trigger.irq(flag.isr, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    oled = OLEDDisplay(i2c)

    display_name = t("car") if MODE == "TX" else t("you")

    # Cold boot: splash screen
    if not is_deepsleep:
        oled.show_splash(display_name, FIRMWARE_VERSION)
        time.sleep(2)

    # === TX SHORT BUTTON WAKE: quick status → sleep ===
    if wakeup_source == "button" and not button_long and MODE == "TX":
        v, pct, chrg = bat_monitor.get_status()
        _, t_str = format_date_time()
        oled.show_brief_status(t("car"), pct, chrg, armed, t_str,
                               remote_bat=remote_bat_pct)
        # Quick radio init for continuous RX in next sleep
        spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0,
                  sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
        radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN),
                       rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))
        radio.init_radio()
        time.sleep(3)
        if SLEEP_ENABLED:
            go_to_deepsleep(oled=oled, radio=radio)
            return

    # === Radar (TX only, skip for button wake) ===
    radar = None
    if MODE == "TX" and LD2410B is not None and wakeup_source != "button":
        try:
            radar = LD2410B(uart_id=RADAR_UART_ID, tx_pin=RADAR_TX_PIN, rx_pin=RADAR_RX_PIN)
            logger.info("Probing radar %dms...", RADAR_CHECK_MS)
            oled.show_status("RADAR", t("checking"), progress=20)
            radar_ok = False
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < RADAR_CHECK_MS:
                if radar.read_frame():
                    radar_ok = True
                    break
                time.sleep_ms(50)
            if radar_ok:
                logger.info("Radar OK")
            else:
                logger.error("Radar not responding")
                radar = None
        except Exception as e:
            logger.error("Radar fail: %s", e)

    # TX cold boot without radar → start disarmed (don't sleep immediately)
    if MODE == "TX" and radar is None and not is_deepsleep:
        armed = False
        _save_state()
        logger.info("No radar — starting DISARMED")

    # === WiFi (RX cold boot only) ===
    if MODE == "RX" and not is_deepsleep:
        connect_wifi_and_sync(oled)

    # === Radio ===
    spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0,
              sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN),
                   rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))
    if need_radio_init:
        logger.info("Radio init...")
        radio.init_radio()
    else:
        logger.info("Radio: skip init (continuous RX)")

    crypto = AESCryptoManager(key_path="secret.key")

    # === Wake dispatch ===

    # TX long button: SOS
    if wakeup_source == "button" and button_long and MODE == "TX":
        v, pct, chrg = bat_monitor.get_status()
        p = build_status_payload(bat_monitor, radar)
        p["msg"] = "SOS"
        oled.show_sos_sent(pct, remote_bat=remote_bat_pct)
        send_packet(radio, crypto, p, led=led)
        time.sleep(3)
        if SLEEP_ENABLED:
            go_to_deepsleep(oled=oled, radio=radio)
            return

    # TX ext1: try LoRa packet first, then sensor alarm
    if MODE == "TX" and wakeup_source == "radio_or_sensor":
        pkt = receive_packet(radio, crypto, 1000)
        if pkt:
            _tx_handle_rx_command(pkt, radio, crypto, oled, bat_monitor, radar, led)
            # After command: continue to main loop (handles armed/disarmed)
        else:
            # No LoRa → sensor wake
            if armed:
                wakeup_source = "radar_out"
            else:
                # Disarmed: ignore sensor, go back to sleep
                logger.info("Disarmed, ignoring sensor → sleep")
                if SLEEP_ENABLED:
                    go_to_deepsleep(oled=oled, radio=radio)
                    return

    # TX vibration alarm
    if MODE == "TX" and wakeup_source == "vibration" and armed:
        p = build_status_payload(bat_monitor, radar)
        p["msg"] = "Vibration!"
        p["ts"] = -2
        oled.show_status(t("vibr"), t("shock_alarm"))
        send_packet(radio, crypto, p, led=led)
        time.sleep(2)

    # TX radar wake alarm
    if MODE == "TX" and wakeup_source == "radar_out" and armed:
        p = build_status_payload(bat_monitor, radar)
        if radar:
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < 1000:
                if radar.read_frame():
                    p["ts"] = radar.target_state
                    p["dd"] = radar.detection_distance
                    break
                time.sleep_ms(50)
        p["msg"] = "Alarm!"
        oled.show_status(t("alarm"), t("radar_wake"))
        send_packet(radio, crypto, p, led=led)
        time.sleep(2)

    # TX/RX disarmed sensor wake → ignore, sleep
    if MODE == "TX" and wakeup_source in ("vibration", "radar_out") and not armed:
        logger.info("Disarmed, ignoring sensor → sleep")
        v, pct, chrg = bat_monitor.get_status()
        oled.show_brief_status(t("car"), pct, chrg, armed, "", remote_bat=remote_bat_pct)
        time.sleep(2)
        if SLEEP_ENABLED:
            go_to_deepsleep(oled=oled, radio=radio)
            return

    # RX button wake dispatch
    rx_wake_type = None
    if MODE == "RX" and wakeup_source == "button":
        rx_wake_type = "button_long" if button_long else "button_short"
    elif MODE == "RX" and wakeup_source == "timer":
        rx_wake_type = "timer"
    elif MODE == "RX" and wakeup_source == "radio_or_sensor":
        # Radio wake: packet waiting — handle in main loop
        rx_wake_type = "radio"

    # === Ready ===
    v, pct, chrg = bat_monitor.get_status()
    oled.show_status(f"{display_name} OK", f"v{FIRMWARE_VERSION}", progress=100,
                     bat1=pct, bat2=remote_bat_pct, locked=armed)
    time.sleep(1)

    # === Main loop ===
    if MODE == "TX":
        tx_main_loop(radio, crypto, oled, bat_monitor, radar, led, flag, trigger)
    else:
        rx_main_loop(radio, crypto, oled, bat_monitor, led, flag, trigger, rx_wake_type)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import sys
        sys.print_exception(e)
        logger.exception("FATAL: %s", str(e))
        try:
            i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
            oled = OLEDDisplay(i2c)
            oled.show_status("CRASHED", t("see_console"), str(e)[:15])
        except:
            pass
