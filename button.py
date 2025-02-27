import logging
import time
from machine import Pin
from config import BUTTON_PIN

logger = logging.getLogger("BUTTON")

class ButtonHandler:
    def __init__(self):
        """Initialize button handler"""
        self.button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)  # Button configured with pull-up
        self.callback = None
        self.button.irq(trigger=Pin.IRQ_FALLING, handler=self._button_pressed)  # Detect falling edge (button press)
        logger.info("Button handler initialized")
    
    def _button_pressed(self, pin):
        """Internal callback for button press"""
        if self.callback:
            logger.info("Button press detected.")
            self.callback()  # Call user-defined callback

    def set_callback(self, callback):
        """Set callback function for button press"""
        self.callback = callback
        logger.debug("Button callback set.")

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)  # Set logging level for testing

    def test_callback():
        """Test callback function"""
        print("Button was pressed!")

    logger.info("Testing ButtonHandler...")

    button_handler = ButtonHandler()
    button_handler.set_callback(test_callback)

    try:
        while True:
            time.sleep(1)  # Keep the script running to detect button presses
    except KeyboardInterrupt:
        logger.info("Test interrupted, exiting...")
