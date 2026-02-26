# main.py
import time
from machine import Pin, SPI
import micropython
import logging

from lr1121 import (
    LR1121,
    LR1121_SPI_BAUDRATE,
    # opcodes
    LR1121_OP_CLR_ERROR,
    LR1121_OP_SET_REG,
    LR1121_OP_SET_STDBY,
    LR1121_OP_CALIBRATE,
    # constants
    STDBY_XOSC,
    CALIB_ALL_MASK,
    TCXO_VOLTAGE_1_8V,
)

micropython.alloc_emergency_exception_buf(256)

# ==============================================================================
# Logging
# ==============================================================================
logger = logging.getLogger("MAIN")
logging.basicConfig(level=logging.DEBUG)

# ==============================================================================
# Pins
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
# LED helpers
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

# ==============================================================================
# Trigger flag without global + ISR-safe (schedule)
# ==============================================================================
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
    logger.info("Radio ready. Pin0 toggles TX/RX mode.")

    mode_tx = True

    while True:
        if flag.value:
            flag.value = False
            mode_tx = not mode_tx
            logger.info("Pin0 trigger -> mode switched to %s", "TX" if mode_tx else "RX")
            blink_ok(led)

        if mode_tx:
            payload = {"message": "Hello from alarm module!", "time": time.time()}
            logger.debug("TX: %s", payload)

            ok = radio.transmit_json(payload, timeout_ms=3000, chunk_size=200)
            if ok:
                logger.info("TX success.")
                blink_ok(led)
            else:
                logger.warning("TX failed.")
                blink_fail(led)

            time.sleep_ms(300)

        else:
            logger.debug("RX: waiting JSON...")
            # assumes lr1121.py has receive_json(timeout_ms=..., max_len=...)
            obj = radio.receive_json(timeout_ms=15000, max_len=255)

            if obj is not None:
                logger.info("RX JSON: %s", obj)
                blink_ok(led)
            else:
                logger.debug("RX: no JSON / failed.")
                blink_fail(led)

            time.sleep_ms(200)

if __name__ == "__main__":
    main()
