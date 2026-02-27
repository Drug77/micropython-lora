# main.py
import time
import os
import ujson
import network
import ntptime
from machine import Pin, SPI, I2C
import micropython
import logging

import ucryptolib 
from ssd1306 import SSD1306_I2C

from lr1121 import (
    LR1121, LR1121_SPI_BAUDRATE, LR1121_OP_CLR_ERROR,
    LR1121_OP_SET_REG, LR1121_OP_SET_STDBY, LR1121_OP_CALIBRATE,
    STDBY_XOSC, CALIB_ALL_MASK, TCXO_VOLTAGE_1_8V,
)

micropython.alloc_emergency_exception_buf(256)

# ==============================================================================
# Инициализация логгеров
# ==============================================================================
logging.basicConfig(level=logging.DEBUG) 
log_main = logging.getLogger("MAIN")
log_oled = logging.getLogger("OLED")
log_crypto = logging.getLogger("AES")

# ==============================================================================
# Настройки Wi-Fi и Времени
# ==============================================================================
WIFI_SSID = "Fold5"
WIFI_PASS = "159632478"
TIMEZONE_OFFSET = 2  # Смещение часового пояса (в часах), например +2 для Киева/Кишинева

# ==============================================================================
# Пины и настройки дисплея
# ==============================================================================
I2C_SDA = 18
I2C_SCL = 17
DISPLAY_ADDR = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64

# ==============================================================================
# Пины радио LR1121 и периферии
# ==============================================================================
SCK_PIN  = 5
MISO_PIN = 3
MOSI_PIN = 6
NSS_PIN  = 7
BUSY_PIN = 34
RST_PIN  = 8
DIO9_PIN = 36

TRIGGER_PIN = 0
LED_PIN     = 37

# ==============================================================================
# Форматирование времени
# ==============================================================================
def format_time(timestamp=None):
    """Форматирует Unix timestamp в 'ДД.ММ ЧЧ:ММ:СС'"""
    if timestamp is None:
        timestamp = time.time()
    
    # Добавляем смещение часового пояса (в секундах)
    local_time = time.localtime(timestamp + (TIMEZONE_OFFSET * 3600))
    # local_time = (year, month, mday, hour, minute, second, weekday, yearday)
    return f"{local_time[2]:02d}.{local_time[1]:02d} {local_time[3]:02d}:{local_time[4]:02d}:{local_time[5]:02d}"

# ==============================================================================
# Продвинутый класс для OLED экрана
# ==============================================================================
class OLEDDisplay:
    def __init__(self, i2c_bus):
        log_oled.debug("Initializing OLED on I2C (addr: 0x%02X)", DISPLAY_ADDR)
        try:
            self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_bus, addr=DISPLAY_ADDR)
            self.clear()
            log_oled.info("✅ Display initialized.")
        except Exception as e:
            log_oled.error("❌ Display Init failed: %s", str(e))
            self.display = None

    def clear(self):
        if self.display:
            self.display.fill(0)
            self.display.show()

    def draw_header(self, title, sub_title="", show_antenna=False):
        if not self.display: return
        
        # Если есть подзаголовок (качество сигнала), делаем хэдер шире
        h_height = 20 if sub_title else 13
        self.display.fill_rect(0, 0, OLED_WIDTH, h_height, 1)
        
        # Основной заголовок (черный на белом)
        self.display.text(title, 2, 2, 0)
        
        # Подзаголовок (черный на белом)
        if sub_title:
            self.display.text(sub_title, 2, 11, 0)
        
        # Антенна
        if show_antenna:
            ax, ay = 110, 2
            self.display.pixel(ax+4, ay, 0)
            self.display.hline(ax+3, ay+1, 3, 0)
            self.display.hline(ax+2, ay+2, 5, 0)
            self.display.hline(ax+1, ay+3, 7, 0)
            self.display.vline(ax+4, ay+4, 6, 0)
            self.display.pixel(ax+4, ay+10, 0)

    def draw_progress_bar(self, y, percent):
        if not self.display: return
        width = OLED_WIDTH - 4
        height = 8
        x = 2
        self.display.rect(x, y, width, height, 1)
        fill_width = int((width - 4) * (percent / 100.0))
        if fill_width > 0:
            self.display.fill_rect(x + 2, y + 2, fill_width, height - 4, 1)

    def show_status(self, title, line1="", line2="", line3="", progress=None, antenna=False):
        if not self.display: return
        self.display.fill(0)
        self.draw_header(title, show_antenna=antenna)
        self.display.text(line1, 2, 18, 1)
        self.display.text(line2, 2, 30, 1)
        self.display.text(line3, 2, 42, 1)
        if progress is not None:
            self.draw_progress_bar(54, progress)
        self.display.show()

    def show_message_box(self, title, message, signal_str="", time_str=""):
        if not self.display: return
        self.display.fill(0)
        
        # Рисуем хэдер (с сигналом, если он передан)
        self.draw_header(title, sub_title=signal_str)
        
        # Рамка для сообщения начинается под хэдером
        box_y = 22 if signal_str else 15
        box_h = OLED_HEIGHT - box_y
        self.display.rect(0, box_y, OLED_WIDTH, box_h, 1)
        
        # Разбиваем сообщение и выводим
        chars_per_line = 15
        msg_lines = [message[i:i+chars_per_line] for i in range(0, len(message), chars_per_line)]
        
        for i, line in enumerate(msg_lines[:2]): 
            self.display.text(line, 4, box_y + 4 + (i * 12), 1)
            
        # Выводим аккуратную дату в самом низу экрана
        if time_str:
            self.display.text(time_str[:15], 4, OLED_HEIGHT - 10, 1)
            
        self.display.show()

# ==============================================================================
# Класс для симметричного шифрования (AES-128 CBC)
# ==============================================================================
class AESCryptoManager:
    def __init__(self, key_path="secret.key"):
        self.key = None
        self.block_size = 16
        self._load_or_generate_key(key_path)

    def _load_or_generate_key(self, key_path):
        try:
            with open(key_path, 'rb') as f:
                self.key = f.read(16)
            log_crypto.info("✅ Key loaded from %s", key_path)
        except OSError:
            log_crypto.warning("No key found. Generating new 16-byte key...")
            self.key = os.urandom(16)
            with open(key_path, 'wb') as f:
                f.write(self.key)
            log_crypto.info("✅ New key generated and saved to %s", key_path)

    def _pad(self, data: bytes) -> bytes:
        pad_len = self.block_size - (len(data) % self.block_size)
        return data + bytes([pad_len] * pad_len)

    def _unpad(self, data: bytes) -> bytes:
        return data[:-data[-1]]

    def encrypt_json(self, data_dict) -> bytes:
        if not self.key: return None
        raw_bytes = ujson.dumps(data_dict).encode("utf-8")
        iv = os.urandom(self.block_size)
        cipher = ucryptolib.aes(self.key, 2, iv)
        return iv + cipher.encrypt(self._pad(raw_bytes))

    def decrypt_json(self, payload: bytes):
        if not self.key or len(payload) <= self.block_size: return None
        iv, enc = payload[:self.block_size], payload[self.block_size:]
        try:
            cipher = ucryptolib.aes(self.key, 2, iv)
            decrypted_raw = self._unpad(cipher.decrypt(enc))
            return ujson.loads(decrypted_raw.decode("utf-8"))
        except Exception as e:
            log_crypto.error("❌ Decryption failed: %s", str(e))
            return None

# ==============================================================================
# Подключение к Wi-Fi и синхронизация времени
# ==============================================================================
# ==============================================================================
# Подключение к Wi-Fi и синхронизация времени
# ==============================================================================
def connect_wifi_and_sync(oled):
    # Проверка на то, что данные Wi-Fi были изменены
    if WIFI_SSID == "YOUR_WIFI_NAME":
        log_main.warning("❌ Default Wi-Fi credentials detected. Skipping Wi-Fi.")
        oled.show_status("WIFI SKIPPED", "Default config", progress=100)
        time.sleep(1.5)
        return

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    # Небольшая пауза для инициализации радиомодуля Wi-Fi
    time.sleep_ms(200)
    
    # Сброс старых подключений (помогает от зависаний)
    if wlan.isconnected():
        wlan.disconnect()
        time.sleep_ms(200)
    
    if not wlan.isconnected():
        for attempt in range(1, 4):
            log_main.info("Wi-Fi attempt %d/3...", attempt)
            oled.show_status("WIFI INIT", f"Attempt {attempt}/3", WIFI_SSID[:15], progress=attempt*33)
            
            try:
                wlan.connect(WIFI_SSID, WIFI_PASS)
            except OSError as e:
                # Перехват "Wifi Internal Error", чтобы скрипт не крашился
                log_main.error("Wi-Fi internal error on attempt %d: %s", attempt, str(e))
            
            # Ждем до 5 секунд
            for _ in range(50):
                if wlan.isconnected():
                    break
                time.sleep_ms(100)
                
            if wlan.isconnected():
                break

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        log_main.info("✅ Wi-Fi connected! IP: %s", ip)
        oled.show_status("WIFI OK", "IP Address:", ip, progress=90)
        time.sleep(1)
        
        try:
            log_main.info("Syncing time via NTP...")
            oled.show_status("NTP SYNC", "Fetching time...", progress=95)
            ntptime.settime()
            log_main.info("✅ Time synchronized: %s", format_time())
        except Exception as e:
            log_main.warning("❌ NTP Sync failed: %s", e)
    else:
        log_main.warning("❌ Wi-Fi connection failed. Using un-synced RTC.")
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
    log_main.info("=== SYSTEM BOOTING ===")
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
    log_main.debug("Configuring SPI for LR1121...")
    spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN), rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))
    
    log_main.info("Initializing LR1121 Radio...")
    radio.init_radio()
    
    # 4. Инициализация Крипто
    crypto = AESCryptoManager(key_path="secret.key")
    
    mode_tx = True
    display_idle = False

    log_main.info("System Ready. Starting in TX mode.")
    oled.show_status("SYSTEM READY", "Mode: TX", "Press button", "to switch mode", antenna=True)
    time.sleep(2)

    while True:
        # Смена режима по кнопке
        if flag.value:
            flag.value = False
            mode_tx = not mode_tx
            display_idle = False
            state_str = "TX" if mode_tx else "RX"
            
            log_main.warning("🔄 Mode switched via hardware button -> %s", state_str)
            oled.show_status(f"MODE: {state_str}", "Ready...", progress=0, antenna=True)
            blink_ok(led)
            time.sleep_ms(500)

        if mode_tx:
            # Отправляем актуальный timestamp вместо ticks_ms()
            payload = {"msg": "Alarm!", "t": time.time()}
            log_main.info("Initiating Transmission. Time: %s", format_time(payload["t"]))
            
            oled.show_status("TRANSMITTER", "Encrypting...", f"Data: {payload['msg']}", progress=30)
            encrypted_data = crypto.encrypt_json(payload)

            if encrypted_data:
                size = len(encrypted_data)
                oled.show_status("TRANSMITTER", "Sending...", f"Size: {size}b AES", progress=70, antenna=True)
                
                t_start = time.ticks_ms()
                ok = radio.transmit_payload(encrypted_data, timeout_ms=10000)
                t_end = time.ticks_ms()
                
                if ok:
                    log_main.info("✅ TX Success (Airtime: %d ms)", time.ticks_diff(t_end, t_start))
                    oled.show_status("TX SUCCESS", f"Sent: {size} bytes", "Secure Channel", progress=100, antenna=True)
                    blink_ok(led)
                else:
                    log_main.error("❌ TX Failed. Radio timeout or error.")
                    oled.show_status("TX ERROR", "Radio timeout", "Check antenna!", progress=0)
                    blink_fail(led)
            else:
                oled.show_status("TX ERROR", "Crypto failed!", progress=0)
                blink_fail(led)

            time.sleep_ms(3000)

        else:
            # РЕЖИМ ПРИЕМА (RX)
            if not display_idle:
                log_main.info("🎧 Listening for incoming packets...")
                oled.show_status("RECEIVER", "Listening...", "Awaiting data", progress=0, antenna=True)
                display_idle = True
            
            raw_data = radio.receive_payload(timeout_ms=30000, max_len=255)

            if raw_data is not None:
                display_idle = False
                rx_size = len(raw_data)
                log_main.info("📡 RX Event: Captured %d bytes.", rx_size)
                
                oled.show_status("RX DATA!", f"Size: {rx_size} bytes", "Decrypting...", progress=50, antenna=True)
                
                decrypted_obj = crypto.decrypt_json(raw_data)
                
                if decrypted_obj is not None:
                    msg = decrypted_obj.get("msg", "Unknown")
                    t_val = decrypted_obj.get("t", 0)
                    
                    # Читаем уровень сигнала из драйвера
                    rssi = getattr(radio, 'last_rssi', 0)
                    snr = getattr(radio, 'last_snr', 0)
                    
                    time_str = format_time(t_val)
                    sig_str = f"R:{rssi} S:{snr}"
                    
                    log_main.info("✅ Decoded: '%s' | Sent at: %s", msg, time_str)
                    log_main.info("📶 Signal: %s dBm, SNR: %s dB", rssi, snr)
                    
                    # Выводим на экран: Заголовок, Сообщение (по центру), Сигнал (в хэдере), Время (внизу окна)
                    oled.show_message_box("SECURE RX OK", msg, signal_str=sig_str, time_str=time_str)
                    
                    blink_ok(led)
                    time.sleep(4) # Даем время прочитать сообщение на экране
                else:
                    log_main.warning("❌ RX Payload rejected (Decryption failed).")
                    oled.show_status("RX ERROR", "Decryption fail", "Wrong secret key", progress=0)
                    blink_fail(led)
                    time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        log_main.exception("🔥 FATAL ERROR: %s", str(e), exc_info=True)
        try:
            i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
            oled = OLEDDisplay(i2c)
            oled.show_message_box("CRASHED", str(e)[:30])
        except: pass