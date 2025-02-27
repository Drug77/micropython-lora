"""
lr1121.py - MicroPython driver for the LR1121 radio transceiver module

This library provides basic functions for initializing and communicating with the LR1121.
It is adapted from the Arduino Radiolib LR11x0 code and tailored specifically for the LR1121.

Author: Your Name
Date: YYYY-MM-DD
License: MIT
"""

import machine
import time
import logging

# Configure module logger
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("LR1121")
logger.setLevel(logging.DEBUG)

class LR1121:
    """
    MicroPython driver for the LR1121 module.

    This driver uses the SPI interface to send commands and data to the LR1121.
    It provides basic methods for initialization, transmission, reception and reading the RSSI.
    """

    # Command constants (a few examples from the LR11x0 datasheet)
    _CMD_SET_PACKET_TYPE      = 0x020E  # Set packet type (LoRa, GFSK, etc.)
    _CMD_SET_RF_FREQUENCY     = 0x020B  # Set radio frequency
    _CMD_SET_MODULATION_PARAMS= 0x020F  # Set modulation parameters (for LoRa)
    _CMD_SET_PACKET_PARAMS    = 0x0210  # Set packet parameters (for LoRa)
    _CMD_SET_LORA_SYNC_WORD   = 0x022B  # Set LoRa sync word
    _CMD_WRITE_BUFFER         = 0x0109  # Write payload to the buffer
    _CMD_SET_TX               = 0x020A  # Start transmission
    _CMD_SET_RX               = 0x0209  # Start reception
    _CMD_READ_BUFFER          = 0x010A  # Read received payload
    _CMD_GET_RSSI_INST        = 0x0205  # Get instantaneous RSSI
    _CMD_GET_VERSION          = 0x0101  # Command to get version info
    _CMD_GET_RANDOM_NUMBER    = 0x0120
    _CMD_MODE_STANDBY = 0x011C
    _DEVICE_LR1121 = 0x03

    # Packet types
    _PACKET_TYPE_LORA = 0x02
    _BUSY_MAX_DELAY_MS = 300
    _REBOOT_MAX_DELAY_MS = 300

    def __init__(self, spi, cs_pin, reset_pin, irq_pin=None, busy_pin=None, tcxo_voltage=0):
        """
        Initialize an instance of LR1121.

        :param spi: An initialized machine.SPI object.
        :param cs_pin: Chip select (CS) pin (machine.Pin instance).
        :param reset_pin: Reset pin (machine.Pin instance).
        :param irq_pin: (Optional) IRQ pin (machine.Pin instance) for interrupt handling.
        :param tcxo_voltage: Optional TCXO voltage in volts. Set to 0 if not used.
        """
        self.spi = spi
        self.cs = cs_pin
        self.reset_pin = reset_pin
        self.irq_pin = irq_pin
        self.busy_pin = busy_pin
        self.tcxo_voltage = tcxo_voltage
        self.busy_pin.irq(self.busy_handler, machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING)
        self.is_busy = False

        # Internal configuration state
        self.frequency = None
        self.bandwidth = None
        self.spreading_factor = None
        self.coding_rate = None

        logger.info("LR1121 instance created.")

    def busy_handler(self, busy_pin):
        self.is_busy = busy_pin.value() == 1
        
    def close(self):
        """
        Properly shuts down the LR1121 module by:
        - Disabling the IRQ pin interrupt.
        - Deinitializing the SPI interface to free hardware resources.
        
        This should be called when the module is no longer needed to prevent 
        conflicts with other SPI devices or redundant power consumption.
        """
        logger.info("Closing LR1121 module.")
        # Disable IRQ if it was configured
        self.irq_pin.irq(None)
        # Deinitialize SPI
        self.spi.deinit()
        
    def reset(self):
        """
        Reset the LR1121 module and wait until it is ready.
        """
        logger.info("Resetting LR1121 module.")
        self.reset_pin.value(0)
        time.sleep_ms(10)
        self.reset_pin.value(1)
        
        logger.debug("Waiting for BUSY signal to go low...")
        deadline = time.ticks_add(time.ticks_ms(), self._REBOOT_MAX_DELAY_MS)
        while self.is_busy:
            if time.ticks_diff(deadline, time.ticks_ms()) < 0:
                logger.error("Timeout waiting for BUSY to go low.")
                raise ValueError("Timeout waiting for BUSY to go low.")
            time.sleep_ms(10)
        logger.debug("BUSY signal is low, reset complete.")
    
    def spi_command(self, cmd: int, write: bool, data: bytes = b"", read_length: int = 0, pre_read_length: int = 0) -> bytes:
        """
        Send an SPI command to the LR1121 device.

        The function handles both read and write SPI transactions. 
        - For write operations, it sends the command and data in a single transaction.
        - For read operations, it performs a two-step process:
        1. Sends the command and optionally reads preliminary bytes.
        2. Waits for BUSY to go low, then reads the expected response.

        :param cmd: 16-bit command to send.
        :param write: True for a write operation, False for a read operation.
        :param data: Data bytes to send (default: empty bytes for read commands).
        :param read_length: Number of bytes to read (for read operations).
        :param pre_read_length: Number of preliminary bytes to read before actual data read.
        :return: Received response bytes (for read operations) or an empty bytes object for write operations.
        """
        # Construct the command bytes (16-bit for standard, 24-bit if data is appended)
        cmd_bytes = cmd.to_bytes(2, 'big') + data

        if data:
            logger.debug(f"Sending command '%s' with data: '%s'.", f"0x{cmd:04X}", " ".join(f"0x{b:02X}" for b in data))
        else:
            logger.debug(f"Sending command '0x{cmd:04X}'.")

        # Step 1: Send command and optionally read preliminary bytes
        self.cs.value(0)  # CS Low (begin transaction)

        prelim = None
        if pre_read_length:
            prelim = bytearray(pre_read_length)
            self.spi.write_readinto(cmd_bytes, prelim)
            logger.debug("Preliminary response: '%s'.", " ".join(f"0x{b:02X}" for b in prelim))
        else:
            self.spi.write(cmd_bytes)

        self.cs.value(1)  # CS High (end transaction)
        
        # Log BUSY pin state
        busy_state = "HIGH" if self.busy_pin.value() == 1 else "LOW"
        logger.debug(f"BUSY pin after command: '{busy_state}'.")

        if write:
            return prelim if prelim else b""  # Write operation is complete, return empty bytes

        # Step 2: Wait for BUSY to go LOW before reading the response
        deadline = time.ticks_add(time.ticks_ms(), self._BUSY_MAX_DELAY_MS)
        while self.busy_pin.value() == 1:
            if time.ticks_diff(deadline, time.ticks_ms()) < 0:
                logger.error("Timeout waiting for BUSY to go low after SPI command.")
                raise TimeoutError("Timeout waiting for BUSY to go low after SPI command.")
            time.sleep_ms(10)

        logger.debug("BUSY pin is LOW, proceeding with read.")

        # Step 3: Read the response
        self.cs.value(0)  # CS Low again for reading response
        response = self.spi.read(read_length)
        self.cs.value(1)  # CS High (end transaction)

        logger.debug("Response: '%s'.", " ".join(f"0x{b:02X}" for b in response))
        return response
    
    def is_stat1_successful(self, stat1):
        """
        Check if Stat1 response indicates a successful command execution.

        :param stat1: The Stat1 byte received from LR1121.
        :return: True if the command was successful (CMD_OK or CMD_DAT), False otherwise.
        """
        command_status = (stat1 >> 1) & 0b111  # Extract bits 3:1
        interrupt_status = stat1 & 0b1         # Extract bit 0

        # Decode command status
        command_status_map = {
            0: "CMD_FAIL - Command could not be executed",
            1: "CMD_PERR - Invalid opcode/arguments, possible DIO interrupt",
            2: "CMD_OK - Command executed successfully",
            3: "CMD_DAT - Command executed, data is being transmitted",
        }
        
        command_status_str = command_status_map.get(command_status, "RFU - Reserved for future use")

        # Decode interrupt status
        interrupt_status_str = "No interrupt active" if interrupt_status == 0 else "At least one interrupt active"

        # Logging debug details
        logger.debug(f"Stat1 Raw Value: 0x{stat1:02X} (Binary: {stat1:08b}).")
        logger.debug(f"Extracted Command Status: {command_status} ({command_status_str}).")
        logger.debug(f"Extracted Interrupt Status: {interrupt_status} ({interrupt_status_str}).")

        # Log overall success or failure
        if command_status in (2, 3):  # CMD_OK (2) or CMD_DAT (3)
            logger.info(f"Stat1: SUCCESS - {command_status_str}.")
            return True
        else:
            logger.warning(f"Stat1: FAILURE - {command_status_str}.")
            return False

    def begin(self, frequency, bandwidth, spreading_factor, coding_rate,
              preamble_length=8, sync_word=0x34):
        """
        Initialize the LR1121 module with LoRa settings.

        This method resets the module and configures it for LoRa operation with the
        provided parameters.

        :param frequency: Operating frequency in MHz.
        :param bandwidth: LoRa bandwidth in kHz (valid: 62.5, 125.0, 250.0, 500.0).
        :param spreading_factor: Spreading factor (integer 5–12).
        :param coding_rate: Coding rate denominator (integer, e.g. 5 for 4/5, 8 for 4/8).
        :param preamble_length: Preamble length in symbols.
        :param sync_word: Sync word (1 byte). Typically 0x34 for public networks.
        """
        logger.info("Beginning LR1121 initialization...")
        logger.info(" Frequency: %s MHz, Bandwidth: %s kHz, SF: %d, CR: 4/%d.", frequency, bandwidth, spreading_factor, coding_rate)

        self.reset()
    
        stat1, *rest = self.get_version()
        is_ok = self.is_stat1_successful(stat1)
        if not is_ok:
             raise ValueError("GetVersion command failed.")
        
        stat1,*rest = self.standby()
        is_ok = self.is_stat1_successful(stat1)
        if not is_ok:
             raise ValueError("SetStandby command failed.")
        return

        # Set packet type to LoRa
        self._send_command(self._CMD_SET_PACKET_TYPE, data=bytes([self._PACKET_TYPE_LORA]))
        logger.debug("Packet type set to LoRa.")

        # Set radio frequency (command takes 4-byte frequency in Hz)
        freq_hz = int(frequency * 1e6)
        freq_data = freq_hz.to_bytes(4, 'big')
        self._send_command(self._CMD_SET_RF_FREQUENCY, data=freq_data)
        self.frequency = frequency
        logger.debug("Frequency set to %d Hz", freq_hz)

        # Set modulation parameters (LoRa)
        # Format: [SF, BW_reg, (CR - 4), LDRO]
        # For simplicity, LDRO (low data rate optimization) is set to 0 (automatic)
        bw_reg = self._bandwidth_to_reg(bandwidth)
        mod_params = bytes([spreading_factor, bw_reg, coding_rate - 4, 0])
        self._send_command(self._CMD_SET_MODULATION_PARAMS, data=mod_params)
        self.spreading_factor = spreading_factor
        self.bandwidth = bandwidth
        self.coding_rate = coding_rate
        logger.debug("Modulation parameters set: SF=%d, BW_reg=0x%02X, CR=4/%d",
                      spreading_factor, bw_reg, coding_rate)

        # Set packet parameters (LoRa)
        # Format: [Preamble (2 bytes), Header (0=explicit), Payload length (0 for variable), CRC (1=enabled), Invert IQ (0=standard)]
        pkt_params = preamble_length.to_bytes(2, 'big') + bytes([0x00, 0x00, 0x01, 0x00])
        self._send_command(self._CMD_SET_PACKET_PARAMS, data=pkt_params)
        logger.debug("Packet parameters set: Preamble=%d symbols", preamble_length)

        # Set LoRa sync word
        self._send_command(self._CMD_SET_LORA_SYNC_WORD, data=bytes([sync_word]))
        logger.debug("Sync word set to 0x%02X", sync_word)

        logger.info("LR1121 initialization complete.")

    def _bandwidth_to_reg(self, bw):
        """
        Convert LoRa bandwidth in kHz to register value.
        
        Valid conversions (approximate):
          62.5 kHz -> 0x03
          125.0 kHz -> 0x04
          250.0 kHz -> 0x05
          500.0 kHz -> 0x06

        :param bw: Bandwidth in kHz.
        :return: Corresponding register value.
        :raises ValueError: If the bandwidth is not supported.
        """
        if abs(bw - 62.5) < 0.1:
            return 0x03
        elif abs(bw - 125.0) < 0.1:
            return 0x04
        elif abs(bw - 250.0) < 0.1:
            return 0x05
        elif abs(bw - 500.0) < 0.1:
            return 0x06
        else:
            logger.error("Invalid bandwidth: %s kHz", bw)
            raise ValueError("Invalid bandwidth value")

    def transmit(self, data):
        """
        Transmit a payload via LR1121 in LoRa mode.

        This method writes the payload to the radio buffer and initiates transmission.
        Note: This is a blocking call that waits for the transmission to complete.

        :param data: A bytes object containing the payload.
        """
        logger.info("Transmitting data: %s", data)
        # Write data to TX buffer
        self._send_command(self._CMD_WRITE_BUFFER, data=data)
        logger.debug("Data written to TX buffer.")

        # Start transmission with no timeout (0x000000 means "no timeout")
        tx_timeout = (0).to_bytes(3, 'big')
        self._send_command(self._CMD_SET_TX, data=tx_timeout)
        logger.info("Transmission started.")

        # In a real implementation you would wait for an IRQ or poll a status register.
        # Here we simply wait a fixed time.
        time.sleep_ms(100)
        logger.info("Transmission complete.")

    def receive(self, timeout_ms=1000):
        """
        Receive a payload via LR1121 in LoRa mode.

        This method initiates reception with the specified timeout.
        It then waits (polling the IRQ pin if available) for the packet to be received.
        If a packet is received, it is read from the RX buffer and returned.

        :param timeout_ms: Timeout in milliseconds.
        :return: Received payload as bytes, or None if timeout occurred.
        """
        logger.info("Starting reception (timeout %d ms)...", timeout_ms)
        # Set reception timeout (3-byte value). A value of 0xFFFFFF means continuous mode.
        timeout_val = int(timeout_ms)
        timeout_bytes = timeout_val.to_bytes(3, 'big')
        self._send_command(self._CMD_SET_RX, data=timeout_bytes)
        logger.debug("RX mode set with timeout.")

        # Wait for packet reception (using IRQ pin if provided)
        start = time.ticks_ms()
        while True:
            if self.irq_pin and self.irq_pin.value() == 1:
                logger.debug("IRQ triggered: packet received.")
                break
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                logger.warning("Receive timeout after %d ms", timeout_ms)
                return None
            time.sleep_ms(10)

        # Read the received data from RX buffer (assume maximum length 255 bytes)
        received = self._send_command(self._CMD_READ_BUFFER, read_length=255)
        logger.info("Received data: %s", received)
        return received

    def get_rssi(self):
        """
        Retrieve the RSSI (Received Signal Strength Indicator) of the last received packet.

        :return: RSSI in dBm (float) or None if reading fails.
        """
        logger.info("Getting RSSI...")
        response = self._send_command(self._CMD_GET_RSSI_INST, read_length=1)
        if response:
            # The LR1121 returns an 8-bit value; see datasheet for conversion (here, divided by -2)
            raw = int.from_bytes(response, 'big')
            rssi = raw / -2.0
            logger.info("RSSI: %.2f dBm", rssi)
            return rssi
        else:
            logger.error("Failed to get RSSI.")
            return None
        
    def standby(self):
        logger.info("Calling SetStandby mode.")
        response = self.spi_command(self._CMD_MODE_STANDBY, True, b"\x00", 0, 3)                
        if response is None or len(response) < 3:
            logger.error("Invalid SetStandby response length.")
            raise ValueError("Invalid SetStandby response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2]
        logger.debug(f"Standby command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"Standby command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"Standby command 'IrqStatus': '0x{irq_status:02X}'.")
        
        return (stat1, stat2, irq_status)
        
    def get_version(self):
        """
        Query the LR1121 for its version information.

        The response is expected to be 4 bytes:
          - Byte 0: Hardware revision
          - Byte 1: Device ID
          - Byte 2: Firmware major version
          - Byte 3: Firmware minor version

        :return: A tuple (hardware, device, fw_major, fw_minor) if successful, or None if no valid response.
        """
        logger.info("Querying GetVersion info.")
        response = self.spi_command(self._CMD_GET_VERSION, False, b"", 5, 2)                
        if response is None or len(response) < 5:
            logger.error("Invalid GetVersion response length.")
            raise ValueError("Invalid GetVersion response length.")
        
        stat1 = response[0]
        hw_version = response[1]
        device = response[2]
        fw_major = response[3]
        fw_minor = response[4]
        
        logger.debug(f"GetVersion command 'Stat1': '0x{stat1:02X}'.")
        logger.info(f"Hardware version: '{hw_version}'.")
        logger.info(f"Device: '{device}'.")
        logger.info(f"Firmware: '{fw_major}.{fw_minor}'.")
        
        return (stat1, hw_version, device, fw_major, fw_minor)
    
    def get_random_number(self):
        """
        Get a 32-bit random number from the LR1121.

        According to the LR1121 datasheet (Section 3.7.7),
        the GetRandomNumber command is sent as two bytes: 0x01, 0x20.
        The response is 5 bytes:
            Byte 0: Status byte (Stat1)
            Byte 1-4: Random number (MSB first)

        :return: A 32-bit unsigned integer with the random number, or None if the response is invalid.
        """
        logger.info("Requesting random number.")
        # Send command 0x0120 with no extra data and expect 5 bytes in response.
        response = self.spi_command(self._CMD_GET_RANDOM_NUMBER, False, b"", 5, 2)
        if response is None or len(response) < 5:
            logger.error("Invalid random number response.")
            return None

        stat1 = response[0]
        logger.debug(f"GetRandomNumber command 'Stat1': '0x{stat1:02X}'.")
        
        # The following 4 bytes are the random number.
        rand_bytes = response[1:5]
        rand_val = int.from_bytes(rand_bytes, 'big')
        logger.info(f"Random number: '{rand_val}'.")
        
        return rand_val

# ===== Example usage =====
if __name__ == "__main__":
    from config import RADIO_CS_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, RADIO_DIO_PIN, RADIO_MOSI_PIN, RADIO_MISO_PIN, RADIO_SCK_PIN
    
    try:
        # Example hardware configuration (adjust pins and SPI settings to your board)
        cs = machine.Pin(RADIO_CS_PIN, machine.Pin.OUT, value=1)
        rst = machine.Pin(RADIO_RST_PIN, machine.Pin.OUT)
        busy = machine.Pin(RADIO_BUSY_PIN, machine.Pin.IN)
        irq = machine.Pin(RADIO_DIO_PIN, machine.Pin.IN)
        mosi = machine.Pin(RADIO_MOSI_PIN, machine.Pin.OUT)
        miso = machine.Pin(RADIO_MISO_PIN, machine.Pin.IN)
        sck = machine.Pin(RADIO_SCK_PIN, machine.Pin.OUT)
        spi = machine.SPI(1, baudrate=1000000, polarity=0, phase=0, sck=sck, miso=miso, mosi=mosi)
        spi.init()

        # Create an LR1121 instance
        lr = LR1121(spi, cs, rst, irq_pin=irq, busy_pin=busy)

        # Initialize the module for LoRa operation:
        # Frequency: 915 MHz, Bandwidth: 125 kHz, SF: 7, CR: 4/5, default preamble, public sync word (0x34)
        lr.begin(frequency=868, bandwidth=125.0, spreading_factor=12, coding_rate=8)

        # # Transmit a simple message
        # message = b'Hello LR1121!'
        # lr.transmit(message)

        # # Try to receive a message (with 1000 ms timeout)
        # received = lr.receive(timeout_ms=1000)
        # if received:
        #     logger.info("Message received: %s", received)
        # else:
        #     logger.info("No message received.")

        # # Read and print the RSSI
        # rssi = lr.get_rssi()
        # if rssi is not None:
        #     logger.info("Current RSSI: %.2f dBm", rssi)
            
        #lr.get_random_number()
    finally:
        lr.close()
