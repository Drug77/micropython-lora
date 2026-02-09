import time
import struct
from machine import Pin, SPI
import ujson
import micropython
import logging


micropython.alloc_emergency_exception_buf(256)

logger = logging.getLogger("LR1121")

# ==============================================================================
# LR1121 Constants and Opcodes (Semtech LR1121 UM Rev 1.2)
# ==============================================================================

LR1121_SPI_BAUDRATE = 2_000_000

# ---- System
LR1121_OP_GET_STATUS     = 0x0100  # GetStatus (returns Stat1, Stat2, IrqStatus[31:0])
LR1121_OP_SET_TCXO       = 0x0117  # SetTcxoMode
LR1121_OP_CALIBRATE      = 0x010F  # Calibrate
LR1121_OP_GET_ERROR      = 0x010D  # GetErrors
LR1121_OP_CLR_ERROR      = 0x010E  # ClearErrors
LR1121_OP_SET_REG        = 0x0110  # SetRegMode
LR1121_OP_SET_STDBY      = 0x011C  # SetStandby
LR1121_OP_SET_DIO_IRQ    = 0x0113  # SetDioIrqParams
LR1121_OP_CLR_IRQ        = 0x0114  # ClearIrqStatus
LR1121_OP_WRITE_BUFFER8  = 0x0109  # WriteBuffer8

# ---- Radio
LR1121_OP_SET_FREQ       = 0x0206  # SetRfFrequency
LR1121_OP_SET_TX         = 0x020A  # SetTx

# ---- LoRa/GFSK/LR-FHSS config (minimal LoRa TX)
LR1121_OP_SET_PACKET_TYPE        = 0x020E  # SetPacketType
LR1121_OP_SET_MODULATION_PARAMS  = 0x020F  # SetModulationParams
LR1121_OP_SET_PACKET_PARAMS      = 0x0210  # SetPacketParams
LR1121_OP_SET_TX_PARAMS          = 0x0211  # SetTxParams
LR1121_OP_SET_PA_CONFIG          = 0x0215  # SetPaConfig
LR1121_OP_SET_LORA_SYNC_WORD     = 0x022B  # SetLoRaSyncWord

# ---- Packet types
PACKET_TYPE_LORA = 0x02

# ---- Standby modes
STDBY_RC   = 0x00
STDBY_XOSC = 0x01

# ---- Calibration mask (all blocks)
CALIB_ALL_MASK = 0x3F

# ---- TCXO voltage codes
TCXO_VOLTAGE_1_8V = 0x02
TCXO_VOLTAGE_3_3V = 0x07

# ---- IRQ bits (commonly used mapping)
IRQ_TX_DONE   = 1 << 2
IRQ_TIMEOUT   = 1 << 10
IRQ_CMD_ERROR = 1 << 22
IRQ_ERROR     = 1 << 23

IRQ_ALL = 0xFFFFFFFF

# ==============================================================================
# LR1121 Driver Class
# ==============================================================================
class LR1121:
    def __init__(self, spi_bus, nss_pin, busy_pin, rst_pin, dio9_pin):
        self.spi = spi_bus
        self.nss = nss_pin
        self.busy = busy_pin
        self.rst = rst_pin
        self.dio9 = dio9_pin
        self.dio9_triggered = False

        self.nss.init(Pin.OUT, value=1)
        self.rst.init(Pin.OUT, value=1)
        self.busy.init(Pin.IN)

        self.dio9.init(Pin.IN, Pin.PULL_DOWN)
        self.dio9.irq(self._dio9_irq, trigger=Pin.IRQ_RISING)

        logger.info("LR1121 driver Initialized.")


    def _dio9_irq(self, _pin):
        self.dio9_triggered = True

    def _dio9_clear_trigger(self):
        self.dio9_triggered = False

    def _wait_busy(self, timeout_ms=2000):
        start_time = time.ticks_ms()
        while self.busy.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                raise OSError("Hardware Timeout: BUSY stuck HIGH.")

    def send_command(self, opcode, data=b''):
        self._wait_busy()
        self.nss.value(0)
        self.spi.write(struct.pack('>H', opcode))
        if data:
            self.spi.write(data)
        self.nss.value(1)
        # next call will wait BUSY

    def read_command(self, opcode, read_len):
        """
        Classic: send opcode (write phase), wait busy, then read read_len bytes
        """
        self._wait_busy()
        self.nss.value(0)
        self.spi.write(struct.pack('>H', opcode))
        self.nss.value(1)

        self._wait_busy()

        self.nss.value(0)
        resp = self.spi.read(read_len)
        self.nss.value(1)
        return resp

    # --------------------------------------------------------------------------
    # Basics
    # --------------------------------------------------------------------------
    def hardware_reset(self):
        logger.warning("Resetting...")
        self.rst.value(0)
        time.sleep_ms(2)
        self.rst.value(1)
        time.sleep_ms(10)
        self._wait_busy()
        logger.info("Reset Complete.")

    def get_status(self):
        # Stat1, Stat2, Irq[31:24], Irq[23:16], Irq[15:8], Irq[7:0]
        resp = self.read_command(LR1121_OP_GET_STATUS, 6)
        stat1, stat2 = resp[0], resp[1]
        irq = int.from_bytes(resp[2:6], "big")
        logger.debug(f"GetStatus: Stat1=0b{stat1:08b} Stat2=0b{stat2:08b} IrqStatus=0b{irq:08b}.")
        return stat1, stat2, irq

    def clear_irq(self, mask=IRQ_ALL):
        payload = mask.to_bytes(4, "big")
        self.send_command(LR1121_OP_CLR_IRQ, payload)
        logger.debug("ClearIrq command sent.")

    def set_dio_irq_params(self, dio9_mask, dio11_mask=0):
        payload = dio9_mask.to_bytes(4, "big") + dio11_mask.to_bytes(4, "big")
        self.send_command(LR1121_OP_SET_DIO_IRQ, payload)
        logger.debug("SetDioIrqParams command sent.")

    def get_errors(self):
        resp = self.read_command(LR1121_OP_GET_ERROR, 4)
        stat1 = resp[0]
        err = (resp[2] << 8) | resp[3]
        logger.debug(f"GetErrors: Stat1=0b{stat1:08b} Errors=0b{err:016b}.")
        return err

    def configure_tcxo(self, voltage=TCXO_VOLTAGE_1_8V, delay_ms=10):
        logger.debug(f"Configuring TCXO: {delay_ms}ms delay, Voltage Code {voltage}.")
        steps = int(delay_ms * 32.76)  # per UM (~30.52us steps)
        delay_bytes = struct.pack('>I', steps)[1:]  # 24-bit
        payload = struct.pack('B', voltage) + delay_bytes
        self.send_command(LR1121_OP_SET_TCXO, payload)
        logger.debug("SetTcxoMode command sent.")

    def set_regulator_ldo(self):
        self.send_command(LR1121_OP_SET_REG, b"\x00")
        logger.debug("SetRegMode LDO command sent.")

    def standby(self, mode=STDBY_RC):
        self.send_command(LR1121_OP_SET_STDBY, bytes([mode]))
        logger.debug("SetStandby command sent.")

    def set_rf_frequency(self, freq_hz):
        logger.info(f"Setting Frequency to {freq_hz} Hz.")
        payload = struct.pack('>I', freq_hz)
        self.send_command(LR1121_OP_SET_FREQ, payload)
        logger.debug("SetRfFrequency command sent.")
    
    def calibrate_system(self):
        logger.info("Starting Calibration...")
        self.send_command(LR1121_OP_CALIBRATE, struct.pack('B', CALIB_ALL_MASK))
        self._wait_busy(timeout_ms=2000)

        errors = self.get_errors()
        if errors == 0:
            logger.info("Calibration Successful.")
        else:
            logger.error(f"Calibration Failed! Error Code: 0b{errors:016b}")
        return errors
    
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
        logger.debug(f"Stat1 Raw Value: 0b{stat1:08b}.")
        logger.debug(f"Extracted Command Status: {command_status} ({command_status_str}).")
        logger.debug(f"Extracted Interrupt Status: {interrupt_status} ({interrupt_status_str}).")

        # Log overall success or failure
        if command_status in (2, 3):  # CMD_OK (2) or CMD_DAT (3)
            logger.debug(f"Stat1: SUCCESS - {command_status_str}.")
            return True
        else:
            logger.warning(f"Stat1: FAILURE - {command_status_str}.")
            return False
        
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
        
        logger.info("Error details:")
        for bit, message in error_messages.items():
            if error_stat & (1 << bit):
                logger.info(f"  - {message}")

    # --------------------------------------------------------------------------
    # LoRa minimal setup (needed before TX, otherwise CMD_ERROR)
    # --------------------------------------------------------------------------
    def lora_setup_minimal(self):
        # Put to Standby XOSC (stable clock) after TCXO
        self.standby(STDBY_XOSC)
        time.sleep_ms(5)

        # Packet type = LoRa
        self.send_command(LR1121_OP_SET_PACKET_TYPE, bytes([PACKET_TYPE_LORA]))

        # Mod params (example: SF7, BW125, CR 4/5, LDRO=0)
        # Bandwidth encoding depends on UM table; for many Semtech radios:
        # 62.5=0x03, 125=0x04, 250=0x05, 500=0x06
        sf = 7
        bw = 0x03   # Narrowest bandwidth (62.5 kHz)
        cr = 0x04   # Strongest error correction (4/8 coding rate)
        ldro = 1
        self.send_command(LR1121_OP_SET_MODULATION_PARAMS, bytes([sf, bw, cr, ldro]))

        # Packet params:
        # preamble_len (2B), header_type(0 explicit), payload_len(0 variable),
        # crc_en(1), invert_iq(0)
        preamble = 12
        header_type = 0x00
        payload_len = 0x00
        crc_en = 0x01
        invert_iq = 0x00
        self.send_command(
            LR1121_OP_SET_PACKET_PARAMS,
            preamble.to_bytes(2, "big") + bytes([header_type, payload_len, crc_en, invert_iq])
        )

        # PA config (example: HP PA)
        # pa_sel=1, reg_pa_supply=0 (VBAT), duty=7, hp_sel=7
        self.send_command(LR1121_OP_SET_PA_CONFIG, bytes([1, 0, 7, 7]))

        # TX params: power (dBm), ramp_time
        # power 14 is safe start; tune later
        self.send_command(LR1121_OP_SET_TX_PARAMS, bytes([14, 0]))

        # Sync word (0x34 public, or your own)
        self.send_command(LR1121_OP_SET_LORA_SYNC_WORD, bytes([0x34]))

        # IRQ: enable only what we need on DIO9
        self.clear_irq()
        self.set_dio_irq_params(IRQ_TX_DONE | IRQ_TIMEOUT | IRQ_CMD_ERROR | IRQ_ERROR, 0)
        self.clear_irq()

    # --------------------------------------------------------------------------
    # TX helpers
    # --------------------------------------------------------------------------
    def _write_buffer8(self, payload: bytes):
        # WriteBuffer8: opcode + data bytes
        self.send_command(LR1121_OP_WRITE_BUFFER8, payload)

    def transmit_payload(self, payload: bytes, timeout_ms=15000) -> bool:
        """
        Real radio-level success:
        - DIO9 IRQ wait
        - confirm TX_DONE via GetStatus IRQ flags
        """
        self._dio9_clear_trigger()
        self.clear_irq()

        # 1) load payload
        self._write_buffer8(payload)

        # 2) start TX, timeout parameter is 24-bit
        # 0x000000 = no radio timeout (host controls timeout)
        self.send_command(LR1121_OP_SET_TX, b"\x00\x00\x00")

        # 3) wait for DIO9 edge (or host timeout)
        t0 = time.ticks_ms()
        while not self.dio9_triggered:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                print("⏱ Timeout waiting for DIO9.")
                break
            time.sleep_ms(10)

        self._dio9_clear_trigger()

        # 4) read IRQ flags (source of truth)
        _, _, irq = self.get_status()

        # 5) clear IRQ after read
        if irq & IRQ_TX_DONE:
            print(f"✅ TX_DONE. IRQ={irq:08b}.")
            return True

        if irq & IRQ_CMD_ERROR:
            print(f"❌ CMD_ERROR. IRQ=0x{irq:08X}")
            return False

        if irq & IRQ_TIMEOUT:
            print(f"❌ TIMEOUT. IRQ=0x{irq:08X}")
            return False

        if irq & IRQ_ERROR:
            print(f"❌ ERROR. IRQ=0x{irq:08X}")
            return False

        print(f"❌ TX failed/unknown. IRQ=0x{irq:08X}")
        return False

    def transmit_json(self, obj, timeout_ms=15000, chunk_size=200) -> bool:
        """
        Sends JSON over LoRa as UTF-8 bytes.
        If JSON is larger than chunk_size, splits into multiple packets with a small header.

        Header format (very simple):
        b'J' + total_chunks(1B) + chunk_index(1B) + payload_bytes
        """
        raw = ujson.dumps(obj).encode("utf-8")

        if len(raw) <= chunk_size:
            print("SEND JSON (single chunk)")
            ok = self.transmit_payload(raw, timeout_ms=timeout_ms)
            if ok:
                print("✅ JSON TX success (1/1)")
            else:
                print("❌ JSON TX failed (1/1)")
            return ok

        # chunked
        chunks = []
        i = 0
        while i < len(raw):
            chunks.append(raw[i:i + chunk_size])
            i += chunk_size

        total = len(chunks)
        for idx, part in enumerate(chunks, start=1):
            # header: 'J' + total + idx
            frame = b"J" + bytes([total & 0xFF, idx & 0xFF]) + part
            print(f"SEND JSON chunk {idx}/{total} ({len(frame)} bytes)")
            ok = self.transmit_payload(frame, timeout_ms=timeout_ms)
            if ok:
                print(f"✅ JSON TX success on chunk {idx}/{total}")
            else:
                print(f"❌ JSON TX failed on chunk {idx}/{total}")
                return False

        print("✅ JSON transmit finished successfully.")
        return True


# ==============================================================================
# CLI Test
# ==============================================================================
if __name__ == "__main__":
    # Configure module logger
    logging.basicConfig(level=logging.INFO)

    # Pins
    SCK_PIN  = 5
    MISO_PIN = 3
    MOSI_PIN = 6
    NSS_PIN  = 7
    BUSY_PIN = 34
    RST_PIN  = 8
    DIO9_PIN = 36

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

    # 1) reset
    radio.hardware_reset()

    # 2) clear boot errors
    radio.send_command(LR1121_OP_CLR_ERROR)

    # 3) regulator
    radio.set_regulator_ldo()

    # 4) TCXO mode
    radio.configure_tcxo(voltage=TCXO_VOLTAGE_1_8V, delay_ms=10)

    # 5) set frequency (868 MHz)
    radio.set_rf_frequency(868_000_000)

    # 6) calibrate
    err = radio.calibrate_system()

    # 7) final status
    err = radio.get_errors()
    if err == 0:
        print("System Ready. HF Clock is stable.")
    else:
        print("System Faulted.")
        radio._log_error_details(err)

    # 8) IMPORTANT: configure LoRa before TX (otherwise CMD_ERROR)
    radio.lora_setup_minimal()

    # 9) Send JSON
    payload = {
        "type": "ping",
        "ts_ms": time.ticks_ms(),
        "msg": "Hello from LR1121",
    }

    ok = radio.transmit_json(payload, timeout_ms=100, chunk_size=200)
    if not ok:
        print("❌ JSON transmit failed.")
    else:
        print("✅ JSON transmit OK.")
