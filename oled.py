import logging
import time
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from config import OLED_WIDTH, OLED_HEIGHT, DISPLAY_ADDR, I2C_SDA, I2C_SCL

logger = logging.getLogger("OLED")

class OLEDDisplay:
    def __init__(self, i2c):
        """Initialize the OLED display"""
        self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c, addr=DISPLAY_ADDR)
        self.clear()
        self.font_size = 1  # Default font size
        logger.info("OLED display initialized.")
    
    def clear(self):
        """Clear the OLED display"""
        self.display.fill(0)
        self.display.show()
        logger.debug("OLED display cleared.")
    
    def set_font_size(self, size):
        """Set the font size (only supports basic scaling)"""
        if size not in [1, 2, 3]:
            raise ValueError("Font size must be 1, 2, or 3")
        self.font_size = size
        logger.info(f"Font size set to {size}")
    
    def wrap_text(self, text):
        """Wrap text to fit within OLED width"""
        max_chars_per_line = OLED_WIDTH // (8 * self.font_size)  # Approximate char limit per line
        wrapped_lines = []
        words = text.split()
        current_line = ""

        for word in words:
            if len(current_line) + len(word) <= max_chars_per_line:
                current_line += word + " "
            else:
                wrapped_lines.append(current_line.strip())
                current_line = word + " "
        
        if current_line:
            wrapped_lines.append(current_line.strip())

        max_lines = OLED_HEIGHT // (8 * self.font_size)  # Max lines based on font size
        if len(wrapped_lines) > max_lines:
            raise ValueError("Text is too long to fit on the display!")

        return wrapped_lines
    
    def update_display(self, message, center=False):
        """Update the OLED display with a message, with optional centering"""
        self.clear()
        try:
            lines = self.wrap_text(message)
        except ValueError as e:
            logger.error(str(e))
            raise

        y = 0
        for line in lines:
            x = 0
            if center:
                text_width = len(line) * 8 * self.font_size
                x = (OLED_WIDTH - text_width) // 2  # Center horizontally
            
            # Render text multiple times for larger font effect
            for i in range(self.font_size):
                for j in range(self.font_size):
                    self.display.text(line, x + i, y + j)

            y += 8 * self.font_size  # Move to next row based on font size
        
        self.display.show()
        logger.info(f"OLED display updated: '{message}'.")

if __name__ == "__main__":
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
    oled = OLEDDisplay(i2c)
    
    # Test default text
    oled.update_display("Default Text")
    time.sleep(2)
    
    # Test wrapping
    oled.update_display("This is a longer message that should wrap correctly.", center=False)
    time.sleep(2)

    # Test centering
    oled.update_display("Centered!", center=True)
    time.sleep(2)

    # Test font sizes
    for size in [1, 2, 3]:
        oled.set_font_size(size)
        oled.update_display(f"Size {size}", center=True)
        time.sleep(2)

    # Test error case (text too long)
    try:
        oled.update_display("This message is way too long to fit on the screen, it should raise an error.", center=False)
    except ValueError as e:
        print("Error caught:", str(e))
