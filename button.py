import logging
from machine import Pin
from config import BUTTON_PIN


logger = logging.getLogger(__name__)

class ButtonHandler:
    def __init__(self):
        """Initialize button handler"""
        self.button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
        self.callback = None
        self.button.irq(trigger=Pin.IRQ_FALLING, handler=self._button_pressed)
        logger.info("Button handler initialized")
    
    def _button_pressed(self, pin):
        """Internal callback for button press"""
        if self.callback:
            logger.info("Button press detected.")
            self.callback()
    
    def set_callback(self, callback):
        """Set callback function for button press"""
        self.callback = callback
        logger.debug("Button callback set.")
