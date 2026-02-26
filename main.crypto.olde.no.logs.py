# main.py
import time
import os
import ujson
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

logger = logging.getLogger("MAIN")
logging.basicConfig(level=logging.DEBUG)

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
# Продвинутый класс для OLED экрана
# ==============================================================================
class OLEDDisplay:
    def __init__(self, i2c_bus):
        try:
            self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_bus, addr=DISPLAY_ADDR)
            self.clear()
            logger.info("✅ OLED display initialized.")
        except Exception as e:
            logger.error(f"❌ OLED Init failed: {e}")
            self.display = None

    def clear(self):
        if self.display:
            self.display.fill(0)
            self.display.show()

    def draw_header(self, title, show_antenna=False):
        """Рисует красивый заголовок сверху экрана"""
        if not self.display: return
        
        self.display.fill_rect(0, 0, OLED_WIDTH, 14, 1) # Белый фон
        self.display.text(title, 2, 3, 0)               # Черный текст
        
        if show_antenna:
            # Рисуем значок антенны справа (пиксель-арт)
            ax, ay = 110, 2
            self.display.pixel(ax+4, ay, 0)
            self.display.hline(ax+3, ay+1, 3, 0)
            self.display.hline(ax+2, ay+2, 5, 0)
            self.display.hline(ax+1, ay+3, 7, 0)
            self.display.vline(ax+4, ay+4, 6, 0)
            self.display.pixel(ax+4, ay+10, 0) # Точка внизу антенны

    def draw_progress_bar(self, y, percent):
        """Рисует полосу загрузки (0-100%)"""
        if not self.display: return
        
        width = OLED_WIDTH - 4
        height = 8
        x = 2
        # Рамка прогресс-бара
        self.display.rect(x, y, width, height, 1)
        # Заливка прогресс-бара
        fill_width = int((width - 4) * (percent / 100.0))
        if fill_width > 0:
            self.display.fill_rect(x + 2, y + 2, fill_width, height - 4, 1)

    def show_status(self, title, line1="", line2="", line3="", progress=None, antenna=False):
        """Универсальный метод вывода информации с возможным прогресс-баром"""
        if not self.display: return
        
        self.display.fill(0)
        self.draw_header(title, show_antenna=antenna)
        
        self.display.text(line1, 2, 18, 1)
        self.display.text(line2, 2, 30, 1)
        self.display.text(line3, 2, 42, 1)
        
        if progress is not None:
            self.draw_progress_bar(54, progress)
            
        self.display.show()

    def show_message_box(self, title, message):
        """Рисует текст в рамочке по центру экрана (хорошо для результата)"""
        if not self.display: return
        
        self.display.fill(0)
        self.draw_header(title)
        
        # Рисуем рамку для сообщения
        self.display.rect(0, 16, OLED_WIDTH, 48, 1)
        self.display.text(message[:15], 4, 20, 1) # Обрезаем, если длинно
        if len(message) > 15:
            self.display.text(message[15:30], 4, 32, 1)
            
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
        except OSError:
            self.key = os.urandom(16)
            with open(key_path, 'wb') as f:
                f.write(self.key)

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
            return ujson.loads(self._unpad(cipher.decrypt(enc)).decode("utf-8"))
        except:
            return None

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
    led = Pin(LED_PIN, Pin.OUT, value=0)

    flag = TriggerFlag()
    trigger = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_DOWN)
    trigger.irq(flag.isr, trigger=Pin.IRQ_RISING)

    # Запуск экрана
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    oled = OLEDDisplay(i2c)
    
    # Красивая анимация загрузки
    for i in range(0, 101, 25):
        oled.show_status("BOOTING", "Init LR1121...", "Loading AES...", progress=i)
        time.sleep_ms(200)

    spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN), rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))
    
    radio.init_radio()
    crypto = AESCryptoManager(key_path="secret.key")
    
    mode_tx = True
    display_idle = False

    oled.show_status("SYSTEM READY", "Mode: TX", "Press button", "to switch mode", antenna=True)
    time.sleep(2)

    while True:
        if flag.value:
            flag.value = False
            mode_tx = not mode_tx
            display_idle = False
            state_str = "TRANSMITTER" if mode_tx else "RECEIVER"
            oled.show_status(f"MODE: {state_str}", "Ready...", progress=0, antenna=True)
            blink_ok(led)
            time.sleep_ms(500)

        if mode_tx:
            payload = {"msg": "Alarm!", "t": time.ticks_ms()}
            
            # Анимация подготовки
            oled.show_status("TRANSMITTER", "Encrypting...", f"Data: {payload['msg']}", progress=30)
            encrypted_data = crypto.encrypt_json(payload)

            if encrypted_data:
                size = len(encrypted_data)
                oled.show_status("TRANSMITTER", "Sending...", f"Size: {size}b AES", progress=70, antenna=True)
                
                ok = radio.transmit_payload(encrypted_data, timeout_ms=3000)
                
                if ok:
                    oled.show_status("TX SUCCESS", f"Sent: {size} bytes", "Secure Channel", progress=100, antenna=True)
                    blink_ok(led)
                else:
                    oled.show_status("TX ERROR", "Radio timeout", "Check antenna!", progress=0)
                    blink_fail(led)
            else:
                oled.show_status("TX ERROR", "Crypto failed!", progress=0)
                blink_fail(led)

            time.sleep_ms(3000)

        else:
            if not display_idle:
                oled.show_status("RECEIVER", "Listening...", "Awaiting data", progress=0, antenna=True)
                display_idle = True
            
            raw_data = radio.receive_payload(timeout_ms=5000, max_len=255)

            if raw_data is not None:
                display_idle = False
                oled.show_status("RX DATA!", f"Size: {len(raw_data)} bytes", "Decrypting...", progress=50, antenna=True)
                
                decrypted_obj = crypto.decrypt_json(raw_data)
                
                if decrypted_obj is not None:
                    msg = decrypted_obj.get("msg", "Unknown")
                    # Выводим красивое окно с сообщением
                    oled.show_message_box("SECURE RX OK", msg)
                    blink_ok(led)
                    time.sleep(3)
                else:
                    oled.show_status("RX ERROR", "Decryption fail", "Wrong secret.key?", progress=0)
                    blink_fail(led)
                    time.sleep(2)

if __name__ == "__main__":
    main()