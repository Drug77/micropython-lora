import time
import logging
from machine import Pin, I2C, SPI
from lr1121 import LR1121
from oled import OLEDDisplay
from button import ButtonHandler
from config import I2C_SDA, I2C_SCL, RADIO_CS_PIN, RADIO_BUSY_PIN, RADIO_RST_PIN, RADIO_DIO_PIN

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("Main")

# Initialize I2C bus
logger.info("Initializing I2C bus")
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
spi = SPI(1, baudrate=5000000, polarity=0, phase=0)

oled = OLEDDisplay(i2c)
lora = LR1121(spi, RADIO_CS_PIN, RADIO_BUSY_PIN, RADIO_RST_PIN, RADIO_DIO_PIN)
button = ButtonHandler()

# Reset LoRa module
logger.info("Resetting LoRa module")
lora.reset()

def handle_button_press():
    """Handles button press events."""
    logger.info("Button pressed! Sending LoRa packet.")
    lora.send_packet("Hello, LoRa!")

def main():
    """Main loop."""
    button.set_callback(handle_button_press)
    
    while True:
        lora.receive_packet()
        oled.update_display()
        time.sleep(0.5)

if __name__ == "__main__":
    logger.info("Starting LoRa MicroPython Project.")
    main()
