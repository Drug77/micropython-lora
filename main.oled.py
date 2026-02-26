# main.py
import time
import os
import ujson
from machine import Pin, SPI, I2C
import micropython
import logging

# Встроенная библиотека для AES
import ucryptolib 
# Драйвер дисплея (файл ssd1306.py должен быть на устройстве)
from ssd1306 import SSD1306_I2C

from lr1121 import (
    LR1121,
    LR1121_SPI_BAUDRATE,
    LR1121_OP_CLR_ERROR,
    LR1121_OP_SET_REG,
    LR1121_OP_SET_STDBY,
    LR1121_OP_CALIBRATE,
    STDBY_XOSC,
    CALIB_ALL_MASK,
    TCXO_VOLTAGE_1_8V,
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
# Класс для OLED экрана
# ==============================================================================
class OLEDDisplay:
    def __init__(self, i2c_bus):
        """Инициализация OLED дисплея"""
        try:
            self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_bus, addr=DISPLAY_ADDR)
            self.clear()
            logger.info("✅ OLED display initialized.")
        except Exception as e:
            logger.error(f"❌ OLED Init failed: {e}")
            self.display = None

    def clear(self):
        """Очистка экрана"""
        if self.display:
            self.display.fill(0)
            self.display.show()

    def show_status(self, title, line1="", line2="", line3=""):
        """
        Удобный метод для вывода статуса в 4 строки.
        Стандартный шрифт MicroPython 8x8 пикселей.
        В 128x64 помещается 16 символов по ширине и 8 строк по высоте.
        """
        if not self.display:
            return
            
        self.display.fill(0)
        # Заголовок (с инверсией цвета для красоты - рисуем белый прямоугольник)
        self.display.fill_rect(0, 0, OLED_WIDTH, 12, 1)
        self.display.text(title, 2, 2, 0) # Черный текст на белом фоне
        
        # Данные
        self.display.text(line1, 0, 16, 1)
        self.display.text(line2, 0, 28, 1)
        self.display.text(line3, 0, 40, 1)
        
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
                if len(self.key) != 16:
                    raise ValueError("Key must be exactly 16 bytes")
            logger.info("✅ AES Key loaded.")
        except OSError:
            logger.warning("No secret.key found. Generating new...")
            self.key = os.urandom(16)
            with open(key_path, 'wb') as f:
                f.write(self.key)
            logger.info("✅ New AES Key generated.")

    def _pad(self, data: bytes) -> bytes:
        pad_len = self.block_size - (len(data) % self.block_size)
        return data + bytes([pad_len] * pad_len)

    def _unpad(self, data: bytes) -> bytes:
        pad_len = data[-1]
        return data[:-pad_len]

    def encrypt_json(self, data_dict) -> bytes:
        if not self.key: return None
        raw_bytes = ujson.dumps(data_dict).encode("utf-8")
        padded_data = self._pad(raw_bytes)
        iv = os.urandom(self.block_size)
        cipher = ucryptolib.aes(self.key, 2, iv)
        return iv + cipher.encrypt(padded_data)

    def decrypt_json(self, payload: bytes):
        if not self.key or len(payload) <= self.block_size: return None
        iv = payload[:self.block_size]
        encrypted_data = payload[self.block_size:]
        try:
            cipher = ucryptolib.aes(self.key, 2, iv)
            decrypted_raw = self._unpad(cipher.decrypt(encrypted_data))
            return ujson.loads(decrypted_raw.decode("utf-8"))
        except Exception:
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

    # Инициализация кнопки/триггера
    flag = TriggerFlag()
    trigger = Pin(TRIGGER_PIN, Pin.IN, Pin.PULL_DOWN)
    trigger.irq(flag.isr, trigger=Pin.IRQ_RISING)

    # Инициализация I2C и экрана
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    oled = OLEDDisplay(i2c)
    oled.show_status("SYSTEM INIT", "Booting LR1121...", "Loading AES...")

    # Инициализация SPI и Радио
    spi = SPI(1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    radio = LR1121(spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN), rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN))
    
    radio.init_radio()
    crypto = AESCryptoManager(key_path="secret.key")
    
    logger.info("Radio ready. Pin0 toggles TX/RX mode.")
    oled.show_status("SYSTEM READY", "Mode: TX", "Press button", "to switch mode")
    time.sleep(2)

    mode_tx = True
    display_idle = False  # Флаг, чтобы не перерисовывать экран приема каждую секунду

    while True:
        # Обработка переключения режимов
        if flag.value:
            flag.value = False
            mode_tx = not mode_tx
            display_idle = False # Сбрасываем флаг ожидания
            state_str = "TRANSMITTER" if mode_tx else "RECEIVER"
            logger.info("Mode switched to %s", state_str)
            oled.show_status(f"MODE: {state_str}", "Ready...")
            blink_ok(led)
            time.sleep_ms(500)

        if mode_tx:
            payload = {"msg": "Alarm!", "t": time.ticks_ms()}
            logger.debug("TX (Plain): %s", payload)
            
            oled.show_status("TRANSMITTER", "Encrypting...", f"Data: {payload['msg']}")
            encrypted_data = crypto.encrypt_json(payload)

            if encrypted_data:
                size = len(encrypted_data)
                oled.show_status("TRANSMITTER", "Sending over LoRa...", f"Payload: {size} bytes")
                
                # Отправка по радио
                ok = radio.transmit_payload(encrypted_data, timeout_ms=3000)
                
                if ok:
                    logger.info("TX success.")
                    oled.show_status("TX SUCCESS", f"Sent: {size} bytes", "AES-128 Encrypted", "Waiting for next..")
                    blink_ok(led)
                else:
                    logger.warning("TX failed.")
                    oled.show_status("TX ERROR", "Radio timeout", "or cmd error!")
                    blink_fail(led)
            else:
                oled.show_status("TX ERROR", "Encryption failed!")
                blink_fail(led)

            time.sleep_ms(3000) # Пауза перед следующей отправкой

        else: # РЕЖИМ RX
            if not display_idle:
                oled.show_status("RECEIVER", "Listening...", "Awaiting AES data")
                display_idle = True # Чтобы не моргать экраном в цикле
            
            # Прием данных (висим тут до 5 секунд, потом цикл повторяется)
            # Я уменьшил таймаут до 5 сек, чтобы кнопка переключения срабатывала быстрее
            raw_data = radio.receive_payload(timeout_ms=5000, max_len=255)

            if raw_data is not None:
                display_idle = False # Данные пришли, обновим экран
                size = len(raw_data)
                oled.show_status("RX DATA!", f"Size: {size} bytes", "Decrypting...")
                
                decrypted_obj = crypto.decrypt_json(raw_data)
                
                if decrypted_obj is not None:
                    msg = decrypted_obj.get("msg", "Unknown")
                    logger.info("✅ Decrypted: %s", decrypted_obj)
                    oled.show_status("RX SUCCESS", f"Msg: {msg}", "Status: Verified", "AES-128 OK")
                    blink_ok(led)
                    time.sleep(2) # Задержим сообщение на экране, чтобы можно было прочитать
                else:
                    logger.warning("❌ Decryption failed!")
                    oled.show_status("RX ERROR", "Decryption failed!", "Wrong key?", "Or corrupt data")
                    blink_fail(led)
                    time.sleep(2)

if __name__ == "__main__":
    main()