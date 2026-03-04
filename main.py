# main.py
from crypto import AESCryptoManager
import esp32
from machine import Pin, SPI, I2C
import machine
import logging
import micropython
import network
import ntptime
import time

from lr1121 import LR1121, LR1121_SPI_BAUDRATE
from oled import OLEDDisplay


micropython.alloc_emergency_exception_buf(256)

# ==============================================================================
# Инициализация логгеров
# ==============================================================================
logging.basicConfig(level=logging.DEBUG) 
logger = logging.getLogger(__name__)
log_oled = logging.getLogger("OLED")
log_crypto = logging.getLogger("AES")

# ==============================================================================
# Настройки Wi-Fi и Времени
# ==============================================================================
WIFI_SSID = "Fold5"
WIFI_PASS = "159632478"
TIMEZONE_OFFSET = 2  

I2C_SDA, I2C_SCL = 18, 17
DISPLAY_ADDR = 0x3C
OLED_WIDTH, OLED_HEIGHT = 128, 64

SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN = 5, 3, 6, 7
BUSY_PIN, RST_PIN, DIO9_PIN = 34, 8, 36
TRIGGER_PIN, LED_PIN = 0, 37
SHOCK_PIN_NUM = 11   # Пин вибродатчика (пример)

# ==============================================================================
# Глобальные переменные для статистики сети
# ==============================================================================
tx_counter = 0
rx_counter = 0
lost_counter = 0
expected_c = None

def go_to_deepsleep():
    # Указываем, что ESP32 должна проснуться, если на ЛЮБОМ из этих пинов появится "1" (WAKEUP_ANY_HIGH)
    esp32.wake_on_ext1(pins=(DIO9_PIN), level=esp32.WAKEUP_ANY_HIGH)
    time.sleep_ms(100) # Даем логгеру время вывести текст в консоль
    machine.deepsleep()

# ==============================================================================
# Форматирование времени
# ==============================================================================
def format_date_time(timestamp=None):
    """Возвращает кортеж: (Дата, Время). Пример: ('Fr 27.02.2026', '12:30:44')"""
    if timestamp is None:
        timestamp = time.time()
    
    local_time = time.localtime(timestamp + (TIMEZONE_OFFSET * 3600))
    days = ["Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"]
    wd = days[local_time[6]]
    
    date_str = f"{wd} {local_time[2]:02d}.{local_time[1]:02d}.{local_time[0]}"
    time_str = f"{local_time[3]:02d}:{local_time[4]:02d}:{local_time[5]:02d}"
    return date_str, time_str

# ==============================================================================
# Подключение к Wi-Fi и синхронизация времени
# ==============================================================================
def connect_wifi_and_sync(oled):
    # Проверка на то, что данные Wi-Fi были изменены
    if WIFI_SSID == "YOUR_WIFI_NAME":
        logger.warning("❌ Default Wi-Fi credentials detected. Skipping Wi-Fi.")
        oled.show_status("WIFI SKIPPED", "Default config", progress=100)
        time.sleep(1.5)
        return

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    # Небольшая пауза для инициализации радиомодуля Wi-Fi
    time.sleep_ms(200)
    
    if wlan.isconnected(): wlan.disconnect(); time.sleep_ms(200)
    
    if not wlan.isconnected():
        for attempt in range(1, 4):
            logger.info("Wi-Fi attempt %d/3...", attempt)
            oled.show_status("WIFI INIT", f"Attempt {attempt}/3", WIFI_SSID[:15], progress=attempt*33)
            
            try:
                wlan.connect(WIFI_SSID, WIFI_PASS)
            except OSError as e:
                # Перехват "Wifi Internal Error", чтобы скрипт не крашился
                logger.error("Wi-Fi internal error on attempt %d: %s", attempt, str(e))
            
            # Ждем до 5 секунд
            for _ in range(50):
                if wlan.isconnected():
                    break
                time.sleep_ms(100)
                
            if wlan.isconnected():
                break

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        logger.info("✅ Wi-Fi connected! IP: %s", ip)
        oled.show_status("WIFI OK", "IP Address:", ip, progress=90)
        time.sleep(1)
        
        try:
            logger.info("Syncing time via NTP...")
            oled.show_status("NTP SYNC", "Fetching time...", progress=95)
            ntptime.settime()
            # Распаковываем кортеж из двух значений: дата и время
            d_str, t_str = format_date_time()
            logger.info("✅ Time synchronized: %s %s", d_str, t_str)
        except Exception as e:
            logger.warning("❌ NTP Sync failed: %s", e)
    else:
        logger.warning("❌ Wi-Fi connection failed. Using un-synced RTC.")
        oled.show_status("WIFI FAILED", "Working offline", progress=100)
        time.sleep(1.5)

# ==============================================================================
# LED & Trigger helpers
# ==============================================================================
def blink_ok(led: Pin):
    led.value(1); time.sleep_ms(100); led.value(0)

def blink_fail(led: Pin):
    for _ in range(3):
        led.value(1); time.sleep_ms(50); led.value(0); time.sleep_ms(50)

class TriggerFlag:
    def __init__(self):
        self.value = False
    def _set_true_scheduled(self, _arg):
        self.value = True
    def isr(self, _pin):
        micropython.schedule(self._set_true_scheduled, 0)

# ==============================================================================
# Main
# ==============================================================================
def main():
    global tx_counter, rx_counter, lost_counter, expected_c

    need_radio_init = True

    # Проверяем, почему мы включились
    wake_reason = machine.wake_reason()
    
    if wake_reason == machine.EXT1_WAKE:
        logger.warning("🚨 WOKE UP FROM ALARM / SENSOR / RADIO!")
        
        # Узнаем, какой именно пин нас разбудил
        # wake_description() вернет кортеж пинов
        wake_pins = machine.wake_description()
        
        if DIO9_PIN in wake_pins:
            need_radio_init = False
            logger.info("Радиомодуль принял команду из эфира!")
            # Идем читать буфер LR1121
        elif SHOCK_PIN_NUM in wake_pins:
            need_radio_init = False

            logger.info("Датчик движения сработал!")
            # Отправляем тревогу по радио
        elif SHOCK_PIN_NUM in wake_pins:
            need_radio_init = False
            logger.info("Зафиксирован удар/вибрация!")
            # Отправляем тревогу по радио
    else:
        logger.info("=== SYSTEM COLD BOOT (Power On) ===")
        # Обычное включение, инициализируем Wi-Fi и т.д.

    logger.info("=== SYSTEM BOOTING ===")
    led = Pin(LED_PIN, Pin.OUT, value=0)

    flag = TriggerFlag()
    trigger = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_DOWN)
    trigger.irq(flag.isr, trigger=Pin.IRQ_RISING)

    # 1. Запуск экрана
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    oled = OLEDDisplay(i2c)
    oled.show_status("BOOTING", "System Start...", progress=10)
    
    # 2. Подключение к сети и синхронизация времени
    connect_wifi_and_sync(oled)

    # 3. Инициализация Радио
    logger.debug("Configuring SPI for LR1121...")
    spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN), rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))
    
    if need_radio_init:
        logger.info("Initializing LR1121 Radio...")
        radio.init_radio()
    else:
        logger.info("Skipping Radio configuration because of wakeup.")
    
    # 4. Инициализация Крипто
    crypto = AESCryptoManager(key_path="secret.key")
    
    mode_tx = True
    display_idle = False

    logger.info("System Ready. Starting in TX mode.")
    oled.show_status("SYSTEM READY", "Mode: TX", "Press button", "to switch mode", antenna=True)
    time.sleep(2)

    while True:
        # Смена режима по кнопке
        if flag.value:
            flag.value = False
            mode_tx = not mode_tx
            display_idle = False
            state_str = "TX" if mode_tx else "RX"
            
            logger.warning("🔄 Mode switched via hardware button -> %s", state_str)
            oled.show_status(f"MODE: {state_str}", "Ready...", progress=0, antenna=True)
            blink_ok(led)
            time.sleep_ms(500)

        if mode_tx:
            tx_counter += 1
            payload = {"msg": "Alarm!", "t": time.time(), "c": tx_counter}
            
            oled.show_status("TRANSMITTER", "Encrypting...", f"Pkt: #{tx_counter}", progress=0)
            encrypted_data = crypto.encrypt_json(payload)

            if encrypted_data:
                size = len(encrypted_data)
                
                # Запрашиваем у драйвера расчетное время в эфире
                expected_toa = radio.get_time_on_air_ms(size)
                logger.info("Initiating TX (Packet #%d). Expected ToA: %d ms", tx_counter, expected_toa)
                
                oled.show_status("TRANSMITTER", f"Sending #{tx_counter}", f"ToA: {expected_toa/1000:.1f}s", progress=0, antenna=True)
                
                # Функция обратного вызова для плавного заполнения бара
                last_pct = 0
                def tx_progress(elapsed_ms):
                    nonlocal last_pct
                    pct = int((elapsed_ms / expected_toa) * 100)
                    if pct > 100: pct = 100
                    # Обновляем экран только если процент изменился (экономия ресурсов)
                    if pct - last_pct >= 2:
                        oled.update_progress_only(54, pct)
                        last_pct = pct

                t_start = time.ticks_ms()
                # Передаем наш callback в драйвер!
                ok = radio.transmit_payload(encrypted_data, timeout_ms=10000, progress_cb=tx_progress)
                t_end = time.ticks_ms()
                
                if ok:
                    actual_toa = time.ticks_diff(t_end, t_start)
                    logger.info("✅ TX Success (Airtime: %d ms)", actual_toa)
                    oled.show_status("TX SUCCESS", f"Sent: {size} bytes", f"Time: {actual_toa/1000:.1f}s", progress=100, antenna=True)
                    blink_ok(led)
                else:
                    oled.show_status("TX ERROR", "Radio timeout", progress=0)
                    blink_fail(led)
            else:
                oled.show_status("TX ERROR", "Crypto failed!", progress=0)
                blink_fail(led)

            time.sleep_ms(3000)

        else:
            # РЕЖИМ ПРИЕМА (RX)
            if not display_idle:
                oled.show_status("RECEIVER", "Listening...", f"RX:{rx_counter} L:{lost_counter}", progress=0, antenna=True)
                display_idle = True
            
            raw_data = radio.receive_payload(timeout_ms=30000, max_len=255)

            if raw_data is not None:
                display_idle = False
                rx_size = len(raw_data)
                logger.info("📡 RX Event: Captured %d bytes.", rx_size)
                
                oled.show_status("RX DATA!", f"Size: {rx_size} bytes", "Decrypting...", progress=50, antenna=True)
                
                decrypted_obj = crypto.decrypt_json(raw_data)
                
                if decrypted_obj is not None:
                    msg = decrypted_obj.get("msg", "Unknown")
                    t_val = decrypted_obj.get("t", 0)
                    c_val = decrypted_obj.get("c", 0) # Читаем номер пакета
                    
                    # Логика подсчета потерь пакетов
                    rx_counter += 1
                    if expected_c is not None:
                        if c_val > expected_c:
                            lost_counter += (c_val - expected_c)
                        elif c_val < expected_c:
                            # Отправитель был перезагружен (счетчик сбросился)
                            lost_counter = 0 
                            rx_counter = 1
                    expected_c = c_val + 1
                    
                    rssi = getattr(radio, 'last_rssi', 0)
                    snr = getattr(radio, 'last_snr', 0)
                    
                    date_str, time_str = format_date_time(t_val)
                    
                    # Формируем компактные строки (до 16 символов)
                    sig_str = f"R:{rssi} S:{snr}"
                    # Пример: "12:30 RX:5 L:0"
                    stats_str = f"{time_str[:5]} RX:{rx_counter} L:{lost_counter}" 
                    
                    logger.info("✅ Decoded: '%s' | Packet: #%d", msg, c_val)
                    logger.info("📶 Signal: %s dBm, SNR: %s dB | Lost Total: %d", rssi, snr, lost_counter)
                    
                    # Выводим супер-информативное окно
                    oled.show_rx_box("SECURE RX OK", msg, sig_str, date_str, stats_str)
                    
                    blink_ok(led)
                    
                    logger.info("🛌 Good night!")
                    oled.show_status("DEEP SLEEP", "Going to sleep!")
                    go_to_deepsleep()

                else:
                    oled.show_status("RX ERROR", "Decryption fail", progress=0)
                    blink_fail(led)
                    time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.exception("🔥 FATAL ERROR: %s", str(e))
        try:
            i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
            oled = OLEDDisplay(i2c)
            oled.show_status("CRASHED", "See Console", str(e)[:15])
        except: pass