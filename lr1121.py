import machine
import time
import logging
from config import RADIO_CS_PIN, RADIO_BUSY_PIN, RADIO_RST_PIN, RADIO_DIO_PIN, LED_PIN


# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("RADIO")
logger.setLevel(logging.DEBUG)

class LR1121:
    def __init__(self, spi, cs_pin, busy_pin, reset_pin, dio_pin, led_pin):
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
        self.dio_pin = machine.Pin(dio_pin, machine.Pin.IN)
        self.led_pin = machine.Pin(led_pin, machine.Pin.OUT, value=0)

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
        # Enable onboard led to indicate radio is busy 
        self.led_pin.value(1)
        while self.busy.value():
            time.sleep(0.001)
        # Disable onboard led to indicate radio is done 
        self.led_pin.value(0)
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

    def set_rf_frequency(self, frequency):
        """Set the RF frequency."""
        freq_bytes = frequency.to_bytes(4, "big")
        self.send_command(0x86, list(freq_bytes))
        logger.info("RF frequency set to %d Hz", frequency)

    def set_lora_modulation(self, spreading_factor, bandwidth, coding_rate):
        """Set LoRa modulation parameters."""
        self.send_command(0x8A, [spreading_factor, bandwidth, coding_rate])
        logger.info("LoRa modulation set: SF=%d, BW=%dkHz, CR=%d", spreading_factor, bandwidth, coding_rate)

    def set_output_power(self, power):
        """Set output power (max 22 dBm)."""
        if power > 22:
            power = 22
        self.send_command(0x8E, [power])
        logger.info("Output power set to %d dBm", power)

    def set_sync_word(self, sync_word):
        """Set sync word for LoRa (Public: 0x34, Private: 0x12)."""
        self.send_command(0x8C, [sync_word])
        logger.info("Sync word set to 0x%02X", sync_word)

    def enable_crc(self, enable):
        """Enable or disable CRC."""
        self.send_command(0x8B, [1 if enable else 0])
        logger.info("CRC %s", "enabled" if enable else "disabled")

    def set_max_range_mode(self):
        """Configure settings for maximum range and lowest speed."""
        logger.info("Configuring for max range and lowest speed.")
        self.set_rf_frequency(868000000)  # Set frequency to 868MHz
        self.set_lora_modulation(12, 125, 8)  # SF=12, BW=125kHz, CR=4/8
        self.set_output_power(22)  # Max output power
        self.set_sync_word(0x34)  # Public LoRaWAN sync word
        self.enable_crc(True)
        logger.info("Max range mode configured.")
        
    def set_2_4ghz_mode(self):
        """Configure settings for 2.4 GHz LoRa mode."""
        logger.info("Configuring for 2.4 GHz LoRa mode.")
        # Set frequency to 2450 MHz (2.45 GHz)
        self.set_rf_frequency(2450000000)
        # LoRa Modulation Parameters (Spreading Factor, Bandwidth, Coding Rate)
        # SF=12 (Maximum range), BW=125 kHz (as per config), CR=4/8
        self.set_lora_modulation(12, 125, 6)
        # Set output power to max allowed for 2.4 GHz (13 dBm)
        self.set_output_power(13)
        # Use an appropriate sync word (default LoRaWAN or custom for private networks)
        self.set_sync_word(0xAB)  # Example sync word for 2.4 GHz LoRa
        # Enable CRC for data integrity
        self.enable_crc(False)
        logger.info("2.4 GHz LoRa mode configured with 2450 MHz, 125 kHz BW, 13 dBm power.")
        
    def set_868mhz_mode(self):
        """Configure settings for 868 MHz LoRa mode."""
        logger.info("Configuring for 868 MHz LoRa mode.")
        # Set frequency to 868 MHz (European ISM Band)
        self.set_rf_frequency(868000000)
        # LoRa Modulation Parameters (Spreading Factor, Bandwidth, Coding Rate)
        # SF=12 (Maximum range), BW=125 kHz, CR=4/8 (for better reception)
        self.set_lora_modulation(12, 125, 6)
        # Set output power to max allowed for Sub-1 GHz (22 dBm)
        self.set_output_power(22)
        # Use the LoRaWAN public sync word (or a custom one for private networks)
        self.set_sync_word(0xAB)  # Standard LoRaWAN sync word
        # Enable CRC for data integrity
        self.enable_crc(False)
        logger.info("868 MHz LoRa mode configured with 125 kHz BW, 22 dBm power.")

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
        self.send_command(0x89, [modulation_map[modulation_type]])
        logger.info("Modulation set to %s", modulation_type)


# Example usage
spi = machine.SPI(1, baudrate=5000000, polarity=0, phase=0)
lr1121 = LR1121(spi, cs_pin=RADIO_CS_PIN, busy_pin=RADIO_BUSY_PIN, reset_pin=RADIO_RST_PIN, dio_pin=RADIO_DIO_PIN, led_pin=LED_PIN)

if __name__ == "__main__":
    lr1121.reset_device()
    lr1121.set_2_4ghz_mode()
    lr1121.set_modulation("LoRa")
    while True:
        lr1121.send_packet(b"Hello LoRa!")
        received_data = lr1121.receive_packet(64)
        logger.info("Received: %s", received_data)
        time.sleep(10)
