# main.py
import time
import os
import ujson
from machine import Pin, SPI
import micropython
import logging

# Встроенная библиотека для AES
import ucryptolib 

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
# Пины
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
# Класс для симметричного шифрования (AES-128 CBC)
# ==============================================================================
class AESCryptoManager:
    def __init__(self, key_path="secret.key"):
        self.key = None
        self.block_size = 16
        self._load_or_generate_key(key_path)

    def _load_or_generate_key(self, key_path):
        """Загружает 16-байтный ключ или создает новый, если файла нет."""
        try:
            with open(key_path, 'rb') as f:
                self.key = f.read(16)
                if len(self.key) != 16:
                    raise ValueError("Key must be exactly 16 bytes for AES-128.")
            logger.info("✅ AES Key loaded successfully.")
        except OSError:
            logger.warning("No secret.key found. Generating a new 16-byte key...")
            self.key = os.urandom(16)
            with open(key_path, 'wb') as f:
                f.write(self.key)
            logger.info(f"✅ New AES Key saved to {key_path}. COPY IT TO OTHER DEVICES!")

    def _pad(self, data: bytes) -> bytes:
        """Дополняет данные до размера блока (PKCS#7 padding)."""
        pad_len = self.block_size - (len(data) % self.block_size)
        return data + bytes([pad_len] * pad_len)

    def _unpad(self, data: bytes) -> bytes:
        """Убирает дополнение из расшифрованных данных."""
        pad_len = data[-1]
        return data[:-pad_len]

    def encrypt_json(self, data_dict) -> bytes:
        """Сериализует dict в JSON и шифрует AES-128 CBC."""
        if not self.key:
            return None
        
        raw_bytes = ujson.dumps(data_dict).encode("utf-8")
        padded_data = self._pad(raw_bytes)
        
        # Генерируем случайный вектор инициализации (IV) для каждого сообщения
        iv = os.urandom(self.block_size)
        
        # Инициализируем шифратор: 2 = ucryptolib.MODE_CBC
        cipher = ucryptolib.aes(self.key, 2, iv)
        encrypted_data = cipher.encrypt(padded_data)
        
        # Отправляем IV вместе с шифротекстом
        return iv + encrypted_data

    def decrypt_json(self, payload: bytes):
        """Извлекает IV, дешифрует AES-128 CBC и парсит JSON."""
        if not self.key or len(payload) <= self.block_size:
            logger.error("Payload too short or missing key.")
            return None
        
        # Извлекаем первые 16 байт как IV, остальное - шифротекст
        iv = payload[:self.block_size]
        encrypted_data = payload[self.block_size:]
        
        try:
            cipher = ucryptolib.aes(self.key, 2, iv)
            decrypted_padded = cipher.decrypt(encrypted_data)
            decrypted_raw = self._unpad(decrypted_padded)
            return ujson.loads(decrypted_raw.decode("utf-8"))
        except Exception as e:
            logger.error(f"AES Decryption/Parsing failed: {e}")
            return None

# ==============================================================================
# LED & Trigger helpers
# ==============================================================================
def blink_ok(led: Pin):
    led.value(1)
    time.sleep_ms(120)
    led.value(0)
    time.sleep_ms(120)

def blink_fail(led: Pin):
    for _ in range(2):
        led.value(1)
        time.sleep_ms(90)
        led.value(0)
        time.sleep_ms(90)

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

    spi = SPI(
        1,
        baudrate=LR1121_SPI_BAUDRATE,
        polarity=0,
        phase=0,
        sck=Pin(SCK_PIN),
        mosi=Pin(MOSI_PIN),
        miso=Pin(MISO_PIN),
    )

    radio = LR1121(
        spi_bus=spi,
        nss_pin=Pin(NSS_PIN),
        busy_pin=Pin(BUSY_PIN),
        rst_pin=Pin(RST_PIN),
        dio9_pin=Pin(DIO9_PIN),
    )

    radio.init_radio()
    
    # Инициализация AES
    crypto = AESCryptoManager(key_path="secret.key")
    
    logger.info("Radio ready. Pin0 toggles TX/RX mode.")

    mode_tx = True

    while True:
        if flag.value:
            flag.value = False
            mode_tx = not mode_tx
            logger.info("Pin0 trigger -> mode switched to %s", "TX" if mode_tx else "RX")
            blink_ok(led)

        if mode_tx:
            payload = {"message": "Alarm!", "time": time.ticks_ms()}
            logger.debug("TX (Plain): %s", payload)

            # 1. Шифруем
            encrypted_data = crypto.encrypt_json(payload)

            if encrypted_data:
                logger.debug("TX (Encrypted, %d bytes)", len(encrypted_data))
                
                # 2. Отправляем сырые байты (убедись, что в lr1121.py ты используешь transmit_payload)
                ok = radio.transmit_payload(encrypted_data, timeout_ms=3000)
                
                if ok:
                    logger.info("TX success.")
                    blink_ok(led)
                else:
                    logger.warning("TX failed at radio level.")
                    blink_fail(led)
            else:
                logger.warning("TX skipped due to encryption error.")
                blink_fail(led)

            time.sleep_ms(300)

        else:
            logger.debug("RX: waiting for AES encrypted data...")
            
            # Принимаем сырые байты (в lr1121.py используй receive_payload)
            raw_data = radio.receive_payload(timeout_ms=15000, max_len=255)

            if raw_data is not None:
                logger.info("RX success. Received %d encrypted bytes.", len(raw_data))
                
                # Расшифровываем
                decrypted_obj = crypto.decrypt_json(raw_data)
                
                if decrypted_obj is not None:
                    logger.info("✅ RX Decrypted JSON: %s", decrypted_obj)
                    blink_ok(led)
                else:
                    logger.warning("❌ RX Decryption failed (wrong key or corrupted data?).")
                    blink_fail(led)
            else:
                logger.debug("RX: no data / timeout.")
                blink_fail(led)

            time.sleep_ms(200)

if __name__ == "__main__":
    main()