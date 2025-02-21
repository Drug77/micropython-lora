import logging
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from config import OLED_WIDTH, OLED_HEIGHT, DISPLAY_ADDR, I2C_SDA, I2C_SCL


logger = logging.getLogger("OLED")

class OLEDDisplay:
    def __init__(self, i2c):
        """Initialize the OLED display"""
        self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=DISPLAY_ADDR)
        self.clear()
        logger.info("OLED display initialized.")
    
    def clear(self):
        """Clear the OLED display"""
        self.display.fill(0)
        self.display.show()
        logger.debug("OLED display cleared.")
    
    def update_display(self, message="LoRa Ready"):
        """Update the OLED display with a message"""
        self.clear()
        self.display.text(message, 0, 0)
        self.display.text(message, 0, 8)
        self.display.show()
        logger.info(f"OLED display updated: '{message}'.")
        
if __name__ == "__main__":
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
    oled = OLEDDisplay(i2c)
    oled.update_display("This is a Test.")

