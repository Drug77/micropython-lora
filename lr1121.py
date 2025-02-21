import machine
import time
import micropython
import logging


# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class LR1121:
    def __init__(self, spi, cs_pin, busy_pin, reset_pin, dio_pin):
        """
        Initialize the LR1121 LoRa transceiver.
        :param spi: The SPI bus object
        :param cs_pin: Chip select pin (NSS)
        :param busy_pin: Busy pin
        :param reset_pin: Reset pin
        :param dio_pin: DIO pin
        """
        self.spi = spi
        self.cs = machine.Pin(cs_pin, machine.Pin.OUT, value=1)
        self.busy = machine.Pin(busy_pin, machine.Pin.IN)
        self.reset = machine.Pin(reset_pin, machine.Pin.OUT, value=1)
        self.dio_pin = dio_pin
        
        logger.info("LR1121 initialized.")

    def reset_device(self):
        """Perform a hardware reset of the LR1121."""
        logger.info("Resetting LR1121...")
        self.reset.value(0)
        time.sleep(0.01)
        self.reset.value(1)
        time.sleep(0.01)
        logger.debug("Reset complete.")

    def wait_busy(self):
        """Wait until the LR1121 is ready to accept a command."""
        logger.debug("Waiting for BUSY to clear...")
        while self.busy.value():
            time.sleep(0.001)
        logger.debug("BUSY cleared.")

    def send_command(self, command, data=[]):
        """
        Send a command to the LR1121 over SPI.
        :param command: Command byte
        :param data: List of data bytes
        """
        self.wait_busy()
        self.cs.value(0)
        self.spi.write(bytes([command] + data))
        self.cs.value(1)
        logger.debug("Sent command 0x%02X with data %s", command, data)
        self.wait_busy()

    def receive_data(self, length):
        """
        Receive data from the LR1121.
        :param length: Number of bytes to read
        :return: Received data as bytes
        """
        self.wait_busy()
        self.cs.value(0)
        self.spi.write(bytes([0x01, 0x07, length]))
        response = self.spi.read(length)
        self.cs.value(1)
        logger.info("Received data: %s", response)
        return response

    def send_packet(self, data):
        """Send a packet over LoRa."""
        logger.info("Sending packet: %s", data)
        self.send_command(0x01, [0x0E] + list(data))
        logger.debug("Packet sent.")

    def receive_packet(self, length):
        """Receive a packet over LoRa."""
        logger.info("Receiving packet...")
        return self.receive_data(length)

    def set_max_range_mode(self):
        """Configure settings for maximum range and lowest speed."""
        logger.info("Configuring for max range and lowest speed.")
        self.set_rf_frequency(868000000)  # Set frequency to 868MHz
        self.set_lora_modulation(12, 125, 8)  # SF=12, BW=125kHz, CR=4/8
        self.set_output_power(22)  # Max output power
        self.set_sync_word(0x34)  # Public LoRaWAN sync word
        self.enable_crc(True)
        logger.info("Max range mode configured.")

    def set_modulation(self, modulation_type):
        """Change the modulation type."""
        modulation_map = {
            "FSK": 0x01,
            "LoRa": 0x02,
            "GFSK": 0x03
        }
        if modulation_type not in modulation_map:
            logger.error("Invalid modulation type: %s", modulation_type)
            return
        self.send_command(0x01, [0x89, modulation_map[modulation_type]])
        logger.info("Modulation set to %s", modulation_type)

# Example usage
spi = machine.SPI(1, baudrate=5000000, polarity=0, phase=0)
lr1121 = LR1121(spi, cs_pin=5, busy_pin=34, reset_pin=23, dio_pin=33)

if __name__ == "__main__":
    lr1121.reset_device()
    lr1121.set_max_range_mode()
    lr1121.set_modulation("LoRa")
    lr1121.send_packet(b"Hello LoRa!")
    received_data = lr1121.receive_packet(64)
    logger.info("Received: %s", received_data)
