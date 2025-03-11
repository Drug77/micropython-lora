import machine
import time
import logging


logger = logging.getLogger("LR1121")

class LR1121:
    """
    MicroPython driver for the LR1121 module.

    This driver uses the SPI interface to send commands and data to the LR1121.
    It provides basic methods for initialization, transmission, reception and reading the RSSI.
    """

    # System Configuration Commands
    _CMD_CALIBRATE                  = 0x010F   # Calibrate RC oscillators, PLL, ADC, etc.
    _CMD_MODE_STANDBY               = 0x011C   # Enter standby mode (RC/XOSC)
    _CMD_GET_VERSION                = 0x0101   # Get chip hardware/firmware version

    # Error Handling Commands
    _CMD_CLEAR_ERRORS               = 0x010E   # Clear all error flags (PLL/ADC/calibration errors)
    _CMD_GET_ERRORS                 = 0x0100   # Get pending error status

    # Interrupt Configuration Commands
    _CMD_CLEAR_IRQ                  = 0x0114   # Clear interrupt flags
    _CMD_SET_DIO_IRQ_PARAMS         = 0x0113   # Configure DIO pin interrupt mapping

    # Radio Configuration Commands
    _CMD_SET_LORA_SYNC_WORD         = 0x022B   # Set LoRa® sync word (public/private network)
    _CMD_SET_MODULATION_PARAMS      = 0x020F   # Set SF/BW/CR for LoRa or BR/Fdev for FSK
    _CMD_SET_PA_CONFIG              = 0x020E   # Configure PA selection and supply (LP/HP/HF)
    _CMD_SET_PACKET_PARAMS          = 0x0210   # Set packet structure (preamble/header/CRC)
    _CMD_SET_PACKET_TYPE            = 0x020E   # Select modem (LoRa®/FSK/LR-FHSS)
    _CMD_SET_RF_FREQUENCY           = 0x0208   # Set operating frequency (150MHz-2.5GHz)
    _CMD_SET_TX_PARAMS              = 0x020D   # Set TX power level and ramp time

    # Radio Operation Commands
    _CMD_SET_RX                     = 0x0209   # Start reception (with timeout configuration)
    _CMD_SET_TX                     = 0x020A   # Start transmission
    _CMD_SET_RX_TX_FALLBACK_MODE    = 0x0213   # Set post-TX/RX fallback mode (standby/FS)

    # Data Buffer Commands
    _CMD_READ_BUFFER                = 0x010A   # Read RX buffer content
    _CMD_WRITE_BUFFER               = 0x0109   # Write payload to TX buffer

    # System Utilities
    _CMD_GET_RANDOM_NUMBER          = 0x0120   # Generate 32-bit random number

    # Radio Status Commands
    _CMD_GET_RSSI_INST              = 0x0205   # Get instantaneous RSSI measurement
    
    # Standby Mode Options
    _STANDBY_MODE_INTERNAL_RC_OSCILLATOR = 0x00  # Selects internal RC oscillator (Standby RC mode)

    # Fallback Mode Options
    _FALLBACK_MODE_STANDBY_RC            = 0x01  # Standby RC mode (default value)

    # IRQ Configuration Constants
    _CLEAR_IRQ_ALL                       = 0xFFFFFFFF  # Clear all interrupts
    _IRQ_PARAMS_DIO9_NONE                = 0x00000000  # Disable all interrupts on DIO9
    _IRQ_PARAMS_DIO11_NONE               = 0x00000000  # Disable all interrupts on DIO11

    # Calibration Parameters
    _CALIBRATE_ALL                       = 0x3F  # Calibrate all blocks (LF_RC, HF_RC, PLL, ADC, IMG, PLL_TX)

    # Packet Type Constants
    PACKET_TYPE_NONE                     = 0x00  # No packet type selected
    PACKET_TYPE_GFSK                     = 0x01  # (G)FSK modulation
    PACKET_TYPE_LORA                     = 0x02  # LoRa modulation
    PACKET_TYPE_LR_FHSS                  = 0x04  # LR-FHSS modulation
    PACKET_TYPE_RFU                      = 0x03  # Reserved for future use

    # LoRa Coding Rate Constants
    LORA_CR_4_5                          = 0x01  # Short interleaver CR=4/5
    LORA_CR_4_6                          = 0x02  # Short interleaver CR=4/6
    LORA_CR_4_7                          = 0x03  # Short interleaver CR=4/7
    LORA_CR_4_8                          = 0x04  # Short interleaver CR=4/8
    LORA_CR_LI_4_5                       = 0x05  # Long interleaver CR=4/5
    LORA_CR_LI_4_6                       = 0x06  # Long interleaver CR=4/6
    LORA_CR_LI_4_8                       = 0x07  # Long interleaver CR=4/8

    # Hardware Constants
    _DEVICE_LR1121                       = 0x03  # LR1121 Device ID

    # Timing Constants
    _BUSY_MAX_DELAY_MS                   = 80   # Maximum delay before BUSY timeout
    _REBOOT_MAX_DELAY_MS                 = 300  # Maximum delay before reboot timeout


    def __init__(self, spi, cs_pin, reset_pin, irq_pin=None, busy_pin=None, tcxo_voltage=0):
        """
        Initialize an instance of LR1121.

        :param spi: An initialized machine.SPI object.
        :param cs_pin: Chip select (CS) pin (machine.Pin instance).
        :param reset_pin: Reset pin (machine.Pin instance).
        """
        self.spi = spi
        self.cs = cs_pin
        self.reset_pin = reset_pin
        self.busy_pin = busy_pin
        self.tcxo_voltage = tcxo_voltage
        self.busy_pin.irq(self.busy_handler, machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING)
        self.is_busy = False

        # Internal configuration state
        self.frequency = None
        self.bandwidth = None
        self.spreading_factor = None
        self.coding_rate = None

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
        self.busy_pin.irq(None)
        # Deinitialize SPI
        self.spi.deinit()
        
    def reset(self):
        """
        Reset the LR1121 module and wait until it is ready.
        """
        logger.info("Resetting LR1121 module...")
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
                raise ValueError("Timeout waiting for BUSY to go low after SPI command.")
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
            logger.debug(f"Stat1: SUCCESS - {command_status_str}.")
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
        logger.info("Frequency: %s MHz, Bandwidth: %s kHz, SF: %d, CR: 4/%d.", frequency, bandwidth, spreading_factor, coding_rate)

        self.reset()
    
        stat1, *rest = self.get_version()
        is_ok = self.is_stat1_successful(stat1)
        if not is_ok:
             raise ValueError("GetVersion command failed.")
        
        stat1, *rest = self.standby()
        if self.is_stat1_successful(stat1):
            logger.info("Standby mode successfull.")
        else:
             raise ValueError("SetStandby command failed.")
    
        stat1, *rest = self.set_rx_tx_fallback_mode()
        if self.is_stat1_successful(stat1):
            logger.info("Fallabck mode set successfully.")
        else:
             raise ValueError("SetRxTxFallbackMode command failed.")
    
        stat1, *rest = self.clear_irq()
        if self.is_stat1_successful(stat1):
            logger.info("Cleared interrupt signals successfully.")
        else:
             raise ValueError("ClearIrq command failed.")
         
        stat1, *rest = self.set_dio_irq_params(self._IRQ_PARAMS_DIO9_NONE, self._IRQ_PARAMS_DIO11_NONE)
        if self.is_stat1_successful(stat1):
            logger.info("All interrupts disabled successfully.")
        else:
            logger.error("Failed to disable all interrupts.")
            
        stat1, *rest = self.calibrate(self._CALIBRATE_ALL)
        if self.is_stat1_successful(stat1):
            logger.info("Calibration successful.")
        else:
            logger.error("Calibration failed.")
            
        # Get current errors
        stat1, stat2, error_stat = self.get_errors()
        if self.is_stat1_successful(stat1):
            logger.info(f"Current error status: 0x{error_stat:04X}")
        else:
            logger.error("Failed to retrieve errors.")

        # Clear all errors
        stat1, *rest = self.clear_errors()
        if self.is_stat1_successful(stat1):
            logger.info("All errors cleared.")
        else:
            logger.error("Failed to clear errors.")
            
        # Set radio frequency (convert MHz to Hz)
        frequency_mhz = 868
        frequency_hz = int(frequency_mhz * 1e6)
        stat1, *_ = self.set_frequency(frequency_hz)
        if not self.is_stat1_successful(stat1):
            raise RuntimeError("Failed to set frequency.")
            
        # Example usage of set_packet_type
        # Set packet type to LoRa
        stat1, *rest = self.set_packet_type(lr.PACKET_TYPE_LORA)
        if self.is_stat1_successful(stat1):
            logger.info("Packet type set to LoRa.")
        else:
            logger.error("Failed to set packet type.")
            
        # Maximum range configuration for LR1121
        stat1, *rest = self.set_modulation_params(
            sf=12,                      # Highest spreading factor (SF12)
            bw_khz=62.5,                # Narrowest bandwidth (62.5 kHz)
            cr_code=self.LORA_CR_4_8,   # Strongest error correction (4/8 coding rate)
            ldro=1                      # Enable Low Data Rate Optimization
        )
        if self.is_stat1_successful(stat1):
            logger.info("Modulation parameters set successfuly.")
        else:
            logger.error("Failed to set modulation parameters.")
            
        # Set packet parameters (max preamble, CRC enabled)
        stat1, *_ = self.set_packet_params(preamble_length=65535, crc_en=1)
        if not self.is_stat1_successful(stat1):
            raise RuntimeError("Failed to set packet params.")
        
        # Configure PA for high power
        stat1, *_ = self.set_pa_config(pa_sel=1, pa_duty_cycle=7)
        if not self.is_stat1_successful(stat1):
            raise RuntimeError("PA config failed.")
        
        # Set TX power to maximum
        stat1, *_ = self.set_tx_params(power=22)
        if not self.is_stat1_successful(stat1):
            raise RuntimeError("TX params config failed.")
        
        logger.info("LR1121 configured for max power and range.")
        return

        # Set LoRa sync word
        self._send_command(self._CMD_SET_LORA_SYNC_WORD, data=bytes([sync_word]))
        logger.debug("Sync word set to 0x%02X", sync_word)


    def _bandwidth_to_reg(self, bw: float) -> int:
        """
        Convert bandwidth in kHz to BWL register value.

        :param bw: Bandwidth in kHz (62.5, 125, 250, 500).
        :return: Register value (0x03, 0x04, 0x05, 0x06).
        :raises ValueError: For invalid bandwidth.
        """
        bw_map = {
            62.5: 0x03,
            125.0: 0x04,
            250.0: 0x05,
            500.0: 0x06
        }
        
        # Allow slight floating point tolerance
        for valid_bw, reg in bw_map.items():
            if abs(bw - valid_bw) < 0.1:
                return reg
    
        raise ValueError(f"Invalid LoRa bandwidth '{bw:.1f}' kHz. Valid values: 62.5, 125, 250, 500.")

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
        logger.info("Calling SetStandby mode command...")
        response = self.spi_command(self._CMD_MODE_STANDBY, True, self._STANDBY_MODE_INTERNAL_RC_OSCILLATOR.to_bytes(1, "big"), 0, 3)                
        if response is None or len(response) < 3:
            logger.error("Invalid SetStandby response length.")
            raise ValueError("Invalid SetStandby response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2]
        logger.debug(f"SetStandby command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"SetStandby command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"SetStandby command 'IrqStatus': '0x{irq_status:02X}'.")
        
        return (stat1, stat2, irq_status)
    
    def set_rx_tx_fallback_mode(self):
        logger.info("Calling SetRxTxFallbackMode command...")
        response = self.spi_command(self._CMD_SET_RX_TX_FALLBACK_MODE, True, self._FALLBACK_MODE_STANDBY_RC.to_bytes(1, "big"), 0, 3)                
        if response is None or len(response) < 3:
            logger.error("Invalid SetRxTxFallbackMode response length.")
            raise ValueError("Invalid SetRxTxFallbackMode response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2]
        logger.debug(f"SetRxTxFallbackMode command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"SetRxTxFallbackMode command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"SetRxTxFallbackMode command 'IrqStatus': '0x{irq_status:02X}'.")
        
        return (stat1, stat2, irq_status)
    
    def clear_irq(self):
        logger.info("Calling ClearIrq command...")
        response = self.spi_command(self._CMD_CLEAR_IRQ, True, self._CLEAR_IRQ_ALL.to_bytes(4, "big"), 0, 6)                
        if response is None or len(response) < 6:
            logger.error("Invalid ClearIrq response length.")
            raise ValueError("Invalid ClearIrq response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2:6]
        irq_hex = " ".join(f"{byte:02X}" for byte in irq_status)
        logger.debug(f"ClearIrq command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"ClearIrq command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"ClearIrq command 'IrqStatus': '{irq_hex}'.")
        
        return (stat1, stat2, irq_status)
    
    def set_dio_irq_params(self, irq1_to_enable, irq2_to_enable):
        """
        Configure which interrupt signals should be activated on the DIO9 and/or DIO11 interrupt pins.

        :param irq1_to_enable: 32-bit bitmask for enabling interrupts on DIO9.
        :param irq2_to_enable: 32-bit bitmask for enabling interrupts on DIO11.
        :return: A tuple (stat1, stat2, irq_status) if successful, or None if the response is invalid.
        """
        logger.info("Calling SetDioIrqParams command...")
        
        # Prepare the data bytes: Irq1ToEnable (4 bytes) + Irq2ToEnable (4 bytes)
        data = irq1_to_enable.to_bytes(4, 'big') + irq2_to_enable.to_bytes(4, 'big')
        
        # Send the command and expect 3 bytes in response (Stat1, Stat2, IrqStatus)
        response = self.spi_command(self._CMD_SET_DIO_IRQ_PARAMS, True, data, 0, 10)
        
        if response is None or len(response) < 10:
            logger.error("Invalid SetDioIrqParams response length.")
            raise ValueError("Invalid SetDioIrqParams response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2:10]
        irq_hex = " ".join(f"{byte:02X}" for byte in irq_status)
        logger.debug(f"SetDioIrqParams command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"SetDioIrqParams command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"SetDioIrqParams command 'IrqStatus': '{irq_hex}'.")
        
        return (stat1, stat2, irq_status)
    
    def calibrate(self, calib_params):
        """
        Calibrate the specified blocks of the LR1121.

        :param calib_params: 1-byte bitmask specifying which blocks to calibrate.
            - Bit 0: LF_RC calibration
            - Bit 1: HF_RC calibration
            - Bit 2: PLL calibration
            - Bit 3: ADC calibration
            - Bit 4: IMG calibration
            - Bit 5: PLL_TX calibration
            - Bits 6-7: RFU (Reserved for Future Use)
        :return: A tuple (stat1, stat2, irq_status) if successful, or None if the response is invalid.
        """
        logger.info("Calling Calibrate command...")
        
        # Prepare the data byte: CalibParams (1 byte)
        data = calib_params.to_bytes(1, 'big')
        
        # Send the command and expect 3 bytes in response (Stat1, Stat2, IrqStatus)
        response = self.spi_command(self._CMD_CALIBRATE, True, data, 0, 3)
        
        if response is None or len(response) < 3:
            logger.error("Invalid Calibrate response length.")
            raise ValueError("Invalid Calibrate response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2]
        
        logger.debug(f"Calibrate command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"Calibrate command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"Calibrate command 'IrqStatus': '0x{irq_status:02X}'.")
        
        return (stat1, stat2, irq_status)
    
    def get_errors(self):
        """
        Retrieve the pending errors that occurred since the last ClearErrors or circuit startup.

        :return: A tuple (stat1, stat2, error_stat) if successful.
        """
        logger.info("Calling GetErrors command...")
        
        # Send the command and read the response
        response = self.spi_command(self._CMD_GET_ERRORS, False, b"", 3, 2)
        
        if response is None or len(response) < 3:
            logger.error("Invalid GetErrors response length.")
            raise ValueError("Invalid GetErrors response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        error_stat = response[2]
        
        logger.debug(f"GetErrors command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"GetErrors command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"GetErrors command 'ErrorStat': '0x{error_stat:04X}'.")
        
        # Log detailed error information if errors are present
        if error_stat != 0:
            self._log_error_details(error_stat)
        
        return (stat1, stat2, error_stat)

    def clear_errors(self):
        """
        Clear all error flags in the LR1121.

        :return: A tuple (stat1, stat2) if successful.
        """
        logger.info("Calling ClearErrors command.")
        
        # Send the command and read the response
        response = self.spi_command(self._CMD_CLEAR_ERRORS, True, b"", 2, 2)
        
        if response is None or len(response) < 2:
            logger.error("Invalid ClearErrors response length.")
            raise ValueError("Invalid ClearErrors response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        
        logger.debug(f"ClearErrors command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"ClearErrors command 'Stat2': '0x{stat2:02X}'.")
        
        return (stat1, stat2)

    def _log_error_details(self, error_stat):
        """
        Log detailed information about the errors in the ErrorStat byte.

        :param error_stat: The 16-bit error status byte.
        """
        error_messages = {
            0: "LF_RC_CALIB_ERR: Calibration of low frequency RC was not done.",
            1: "HF_RC_CALIB_ERR: Calibration of high frequency RC was not done.",
            2: "ADC_CALIB_ERR: Calibration of ADC was not done.",
            3: "PLL_CALIB_ERR: Calibration of maximum and minimum frequencies was not done.",
            4: "IMG_CALIB_ERR: Calibration of the image rejection was not done.",
            5: "HF_XOSC_START_ERR: High frequency XOSC did not start correctly.",
            6: "LF_XOSC_START_ERR: Low frequency XOSC did not start correctly.",
            7: "PLL_LOCK_ERR: The PLL did not lock.",
            8: "RX_ADC_OFFSET_ERR: Calibration of ADC offset was not done.",
        }
        
        logger.debug("Error details:")
        for bit, message in error_messages.items():
            if error_stat & (1 << bit):
                logger.debug(f"  - {message}")
                
    def set_frequency(self, frequency_hz: int) -> tuple:
        """
        Set the radio frequency for transmission and reception.

        :param frequency_hz: Frequency in Hertz (Hz).
        :return: Tuple (stat1, stat2, irq_status) from LR1121 response.
        :raises ValueError: If the response is invalid.
        """
        logger.info("Setting frequency to %d Hz...", frequency_hz)
        
        # Convert frequency to 4-byte big-endian
        data = frequency_hz.to_bytes(4, 'big')
        
        # Send command and read response (6 bytes: Stat1, Stat2, 4-byte IrqStatus)
        response = self.spi_command(
            cmd=self._CMD_SET_RF_FREQUENCY,
            write=True,
            data=data,
            read_length=0,
            pre_read_length=6
        )
        
        if len(response) < 6:
            logger.error("Invalid response length for SetRfFrequency.")
            raise ValueError("Invalid SetRfFrequency response.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2:6]
        
        logger.debug("SetFrequency Stat1: 0x%02X, Stat2: 0x%02X", stat1, stat2)
        return (stat1, stat2, irq_status)
                
    def set_packet_type(self, packet_type):
        """
        Set the packet type for the LR1121 modem.

        :param packet_type: The packet type to set (use class constants: PACKET_TYPE_*).
        :return: A tuple (stat1, stat2, irq_status) if successful.
        """
        logger.info("Calling SetPacketType command...")
        
        # Validate the packet type
        valid_packet_types = [
            self.PACKET_TYPE_NONE,
            self.PACKET_TYPE_GFSK,
            self.PACKET_TYPE_LORA,
            self.PACKET_TYPE_LR_FHSS,
            self.PACKET_TYPE_RFU,
        ]
        if packet_type not in valid_packet_types:
            logger.error(f"Invalid packet type: '0x{packet_type:02X}'.")
            raise ValueError(f"Invalid packet type: '0x{packet_type:02X}'.")
        
        # Pack the packet type into a single byte
        data = packet_type.to_bytes(1, 'big')
        
        # Send the command and read the response
        response = self.spi_command(self._CMD_SET_PACKET_TYPE, True, data, 0, 3)
        
        if response is None or len(response) < 3:
            logger.error("Invalid SetPacketType response length.")
            raise ValueError("Invalid SetPacketType response length.")
        
        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2]
        
        logger.debug(f"SetPacketType command 'Stat1': '0x{stat1:02X}'.")
        logger.debug(f"SetPacketType command 'Stat2': '0x{stat2:02X}'.")
        logger.debug(f"SetPacketType command 'IrqStatus': '0x{irq_status:02X}'.")
        
        return (stat1, stat2, irq_status)
    
    def set_modulation_params(self, sf: int, bw_khz: float, cr_code: int, ldro: int) -> tuple:
        """
        Configure LoRa modulation parameters.

        :param sf: Spreading factor (5-12).
        :param bw_khz: Bandwidth in kHz (62.5, 125, 250, 500).
        :param cr_code: Coding rate code (0x01 to 0x07, see LORA_CR_* constants).
        :param ldro: Low Data Rate Optimization (0: disabled, 1: enabled).
        :return: Tuple (stat1, stat2, irq_status) from LR1121 response.
        :raises ValueError: If parameters are invalid.
        """
        logger.info("Configuring LoRa modulation parameters: SF=%d, BW=%.1f kHz, CR=0x%02X, LDRO=%d.", 
                    sf, bw_khz, cr_code, ldro)

        # Validate spreading factor
        if not (5 <= sf <= 12):
            raise ValueError(f"Invalid spreading factor '{sf}'. Must be 5-12.")

        # Convert bandwidth to register value
        try:
            bw_reg = self._bandwidth_to_reg(bw_khz)
        except ValueError as e:
            logger.error("Bandwidth validation failed: '%s'.", str(e))
            raise

        # Validate coding rate
        valid_cr_codes = [self.LORA_CR_4_5, self.LORA_CR_4_6, self.LORA_CR_4_7, self.LORA_CR_4_8,
                        self.LORA_CR_LI_4_5, self.LORA_CR_LI_4_6, self.LORA_CR_LI_4_8]
        if cr_code not in valid_cr_codes:
            raise ValueError(f"Invalid CR code '0x{cr_code:02X}'. Use LORA_CR_* constants.")

        # Validate LDRO
        if ldro not in (0, 1):
            raise ValueError("LDRO must be 0 (disabled) or 1 (enabled).")

        # Construct command data: [SF, BWL, CR, LDRO]
        data = bytes([sf, bw_reg, cr_code, ldro])
        logger.debug("Modulation params data: '%s'.", " ".join(f"0x{b:02X}" for b in data))

        # Send command (0x020F) with 6-byte response (Stat1, Stat2, 4-byte IrqStatus)
        response = self.spi_command(
            cmd=self._CMD_SET_MODULATION_PARAMS,
            write=True,
            data=data,
            read_length=0,
            pre_read_length=6  # Expect 6 bytes: Stat1, Stat2, IrqStatus[4]
        )

        # Parse response
        if len(response) < 6:
            logger.error("Invalid response length '%d' for SetModulationParams.", len(response))
            raise ValueError("Incomplete response for SetModulationParams.")

        stat1 = response[0]
        stat2 = response[1]
        irq_status = response[2:6]  # 4 bytes (32-bit IrqStatus)
        
        logger.debug(f"SetModulationParams command 'Stat1': '0x{stat1:02X}'.")

        return (stat1, stat2, irq_status)
    
    def set_packet_params(self, preamble_length=65535, header_type=0x00, payload_len=0, crc_en=1, invert_iq=0):
        """
        Configure LoRa packet parameters for maximum range.
        
        :param preamble_length: Preamble length in symbols (default 65535 for max).
        :param header_type: 0x00 (explicit header), 0x01 (implicit).
        :param payload_len: 0 (variable length) or fixed length.
        :param crc_en: 1 (CRC enabled), 0 (disabled).
        :param invert_iq: 0 (standard), 1 (inverted).
        :return: Tuple (stat1, stat2, irq_status) from LR1121.
        """
        logger.info("Setting LoRa packet params: Preamble=%d, CRC=%d", preamble_length, crc_en)
        
        # Pack data: [Preamble(2B), HeaderType, PayloadLen, CRC, InvertIQ]
        data = (
            preamble_length.to_bytes(2, 'big') +
            bytes([header_type, payload_len, crc_en, invert_iq])
        )
        
        # Send command and read response (Stat1, Stat2, 4-byte IrqStatus)
        response = self.spi_command(
            cmd=self._CMD_SET_PACKET_PARAMS,
            write=True,
            data=data,
            read_length=0,
            pre_read_length=8  # Expect 8 bytes: Stat1, Stat2, 4-byte IrqStatus
        )
        
        if len(response) < 8:
            raise ValueError("Invalid SetPacketParams response.")
        
        stat1, stat2 = response[0], response[1]
        irq_status = response[2:6]
        
        logger.debug(f"SetPacketParams command 'Stat1': '0x{stat1:02X}'.")
        
        return (stat1, stat2, irq_status)
    
    def set_pa_config(self, pa_sel=1, reg_pa_supply=0, pa_duty_cycle=7, pa_hp_sel=7):
        """
        Configure PA for maximum power output (HP PA with duty cycle 7).
        
        :param pa_sel: 0 (LP PA), 1 (HP PA), 2 (HF PA).
        :param reg_pa_supply: 0 (VBAT), 1 (VREG).
        :param pa_duty_cycle: 0-7 (higher = more power).
        :param pa_hp_sel: 0-7 (HP PA size, 7 = max).
        :return: Tuple (stat1, stat2, irq_status).
        """
        logger.info("Configuring PA...")
        
        # Pack data: [PaSel, RegPaSupply, PaDutyCycle, PaHpSel]
        data = bytes([pa_sel, reg_pa_supply, pa_duty_cycle, pa_hp_sel])
        
        response = self.spi_command(
            cmd=self._CMD_SET_PA_CONFIG,
            write=True,
            data=data,
            read_length=0,
            pre_read_length=6  # Stat1, Stat2, 1-byte IrqStatus
        )
        
        if len(response) < 6:
            raise ValueError("Invalid SetPaConfig response.")
        
        stat1, stat2, irq = response[0], response[1], response[2:6]
        return (stat1, stat2, irq)
    
    def set_tx_params(self, power=22, ramp_time=0):
        """
        Set TX power and ramp time for maximum output.
        
        :param power: Output power in dBm (HP PA: up to 22 dBm).
        :param ramp_time: 0 (fastest) to 7 (slowest).
        :return: Tuple (stat1, stat2, irq_status).
        """
        logger.info("Setting TX power: %d dBm, ramp=%d.", power, ramp_time)
        
        # Power is clipped to 0-22 dBm for HP PA
        power = max(0, min(power, 22))
        data = bytes([power, ramp_time])
        
        response = self.spi_command(
            cmd=self._CMD_SET_TX_PARAMS,
            write=True,
            data=data,
            read_length=0,
            pre_read_length=4  # Stat1, Stat2, 1-byte IrqStatus
        )
        
        if len(response) < 4:
            raise ValueError("Invalid SetTxParams response.")
        
        stat1, stat2, irq = response[0], response[1], response[2:4]
        return (stat1, stat2, irq)
        
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
        logger.info("Calling GetVersion info command...")
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
        logger.info("Calling random number command...")
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
    
    # Configure module logger
    logging.basicConfig(level=logging.INFO)
    
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
