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
LR1121_OP_GET_STATUS = 0x0100  # GetStatus
LR1121_OP_SET_TCXO = 0x0117  # SetTcxoMode
LR1121_OP_CALIBRATE = 0x010F  # Calibrate
LR1121_OP_GET_ERROR = 0x010D  # GetErrors
LR1121_OP_CLR_ERROR = 0x010E  # ClearErrors
LR1121_OP_SET_REG = 0x0110  # SetRegMode
LR1121_OP_SET_STDBY = 0x011C  # SetStandby
LR1121_OP_SET_DIO_IRQ = 0x0113  # SetDioIrqParams
LR1121_OP_CLR_IRQ = 0x0114  # ClearIrqStatus
LR1121_OP_WRITE_BUFFER8 = 0x0109  # WriteBuffer8
LR1121_OP_READ_BUFFER8 = 0x010A  # ReadBuffer8

# ---- Radio
LR1121_OP_SET_FREQ = 0x0206  # SetRfFrequency
LR1121_OP_SET_TX = 0x020A  # SetTx
LR1121_OP_SET_RX = 0x0209  # SetRx
LR1121_OP_GET_RX_BUFFER_STATUS = 0x0203  # GetRxBufferStatus
LR1121_OP_GET_PACKET_STATUS = 0x021B  # <--- ДОБАВЛЯЕМ ЭТУ СТРОКУ (GetPacketStatus)

# ---- LoRa/GFSK/LR-FHSS config
LR1121_OP_SET_PACKET_TYPE = 0x020E  # SetPacketType
LR1121_OP_SET_MODULATION_PARAMS = 0x020F  # SetModulationParams
LR1121_OP_SET_PACKET_PARAMS = 0x0210  # SetPacketParams
LR1121_OP_SET_TX_PARAMS = 0x0211  # SetTxParams
LR1121_OP_SET_PA_CONFIG = 0x0215  # SetPaConfig
LR1121_OP_SET_LORA_SYNC_WORD = 0x022B  # SetLoRaSyncWord

# ---- Packet types
PACKET_TYPE_LORA = 0x02

# ---- Standby modes
STDBY_RC = 0x00
STDBY_XOSC = 0x01

# ---- Calibration mask
CALIB_ALL_MASK = 0x3F

# ---- TCXO voltage codes
TCXO_VOLTAGE_1_8V = 0x02
TCXO_VOLTAGE_3_3V = 0x07

# ---- IRQ bits
IRQ_TX_DONE = 1 << 2
IRQ_TIMEOUT = 1 << 10
IRQ_CMD_ERROR = 1 << 22
IRQ_ERROR = 1 << 23
IRQ_RX_DONE = 1 << 3
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

        self.last_rssi = 0
        self.last_snr = 0

        logger.info("LR1121 Driver Initialized.")

    def _dio9_irq(self, _pin):
        self.dio9_triggered = True

    def _dio9_clear_trigger(self):
        self.dio9_triggered = False

    def _wait_busy(self, timeout_ms=2000):
        start_time = time.ticks_ms()
        while self.busy.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                logger.critical("⏱ Hardware Timeout: BUSY pin stuck HIGH.")
                raise OSError("Hardware Timeout: BUSY stuck HIGH.")

    def send_command(self, opcode, data=b""):
        self._wait_busy()
        self.nss.value(0)
        self.spi.write(struct.pack(">H", opcode))
        if data:
            self.spi.write(data)
        self.nss.value(1)

    def read_command(self, opcode, read_len):
        self._wait_busy()
        self.nss.value(0)
        self.spi.write(struct.pack(">H", opcode))
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
        logger.info("Performing hardware reset...")
        self.rst.value(0)
        time.sleep_ms(10)
        self.rst.value(1)
        time.sleep_ms(300)
        self._wait_busy()
        logger.debug("Hardware reset complete.")

    def get_status(self):
        resp = self.read_command(LR1121_OP_GET_STATUS, 6)
        stat1, stat2 = resp[0], resp[1]
        irq = int.from_bytes(resp[2:6], "big")
        logger.debug("GetStatus: Stat1=0x%02X Stat2=0x%02X IrqStatus=0x%08X", stat1, stat2, irq)
        return stat1, stat2, irq

    def clear_irq(self, mask=IRQ_ALL):
        payload = mask.to_bytes(4, "big")
        self.send_command(LR1121_OP_CLR_IRQ, payload)
        logger.debug("Cleared IRQ flags: 0x%08X", mask)

    def set_dio_irq_params(self, dio9_mask, dio11_mask=0):
        payload = dio9_mask.to_bytes(4, "big") + dio11_mask.to_bytes(4, "big")
        self.send_command(LR1121_OP_SET_DIO_IRQ, payload)
        logger.debug("Set DIO IRQ Params: DIO9=0x%08X, DIO11=0x%08X", dio9_mask, dio11_mask)

    def get_errors(self):
        resp = self.read_command(LR1121_OP_GET_ERROR, 3)
        err = (resp[1] << 8) | resp[2]
        if err != 0:
            logger.warning("GetErrors returned fault: 0x%04X", err)
        return err

    def configure_tcxo(self, voltage=TCXO_VOLTAGE_1_8V, delay_ms=10):
        logger.debug("Configuring TCXO: %dms delay, Voltage Code 0x%02X.", delay_ms, voltage)
        steps = int(delay_ms * 32.76)
        delay_bytes = struct.pack(">I", steps)[1:]
        payload = struct.pack("B", voltage) + delay_bytes
        self.send_command(LR1121_OP_SET_TCXO, payload)

    def set_regulator_ldo(self):
        self.send_command(LR1121_OP_SET_REG, b"\x00")
        logger.debug("Regulator mode set to LDO.")

    def standby(self, mode=STDBY_RC):
        self.send_command(LR1121_OP_SET_STDBY, bytes([mode]))
        logger.debug("Entered Standby Mode 0x%02X.", mode)

    def set_rf_frequency(self, freq_hz):
        logger.info("Setting RF Frequency to %d Hz.", freq_hz)
        payload = struct.pack(">I", freq_hz)
        self.send_command(LR1121_OP_SET_FREQ, payload)

    def calibrate_system(self):
        logger.info("Starting System Calibration...")
        self.send_command(LR1121_OP_CALIBRATE, struct.pack("B", CALIB_ALL_MASK))
        self._wait_busy(timeout_ms=2000)

        errors = self.get_errors()
        if errors == 0:
            logger.info("✅ Calibration Successful.")
        else:
            logger.error("❌ Calibration Failed! Error Code: 0x%04X", errors)
        return errors

    def _log_error_details(self, error_stat):
        error_messages = {
            0: "LF_RC_CALIB_ERR: Low frequency RC calib fail",
            1: "HF_RC_CALIB_ERR: High frequency RC calib fail",
            2: "ADC_CALIB_ERR: ADC calib fail",
            3: "PLL_CALIB_ERR: PLL calib fail",
            4: "IMG_CALIB_ERR: Image rejection calib fail",
            5: "HF_XOSC_START_ERR: HF XOSC start fail",
            6: "LF_XOSC_START_ERR: LF XOSC start fail",
            7: "PLL_LOCK_ERR: PLL failed to lock",
            8: "RX_ADC_OFFSET_ERR: RX ADC offset calib fail",
        }
        for bit, message in error_messages.items():
            if error_stat & (1 << bit):
                logger.error("  - %s", message)

    # --------------------------------------------------------------------------
    # Init Radio
    # --------------------------------------------------------------------------
    def init_radio(self):
        self.hardware_reset()
        self.send_command(LR1121_OP_CLR_ERROR)
        self.set_regulator_ldo()
        self.configure_tcxo(voltage=TCXO_VOLTAGE_1_8V, delay_ms=10)
        self.set_rf_frequency(868_000_000)
        
        err = self.calibrate_system()
        err = self.get_errors()
        if err == 0:
            logger.info("✅ System Ready. HF Clock is stable.")
        else:
            logger.error("❌ System Faulted on Init.")
            self._log_error_details(err)

        self.standby(STDBY_XOSC)
        time.sleep_ms(5)

        logger.debug("Applying LoRa Modulation parameters...")
        self.send_command(LR1121_OP_SET_PACKET_TYPE, bytes([PACKET_TYPE_LORA]))

        # SF7, BW125, CR 4/5, LDRO=0
        sf, bw, cr, ldro = 7, 0x03, 0x04, 1
        self.send_command(LR1121_OP_SET_MODULATION_PARAMS, bytes([sf, bw, cr, ldro]))

        # Dummy packet params (updated dynamically in transmit_payload)
        preamble, header_type, payload_len, crc_en, invert_iq = 12, 0x00, 255, 0x01, 0x00
        self.send_command(
            LR1121_OP_SET_PACKET_PARAMS,
            preamble.to_bytes(2, "big") + bytes([header_type, payload_len, crc_en, invert_iq]),
        )

        self.send_command(LR1121_OP_SET_PA_CONFIG, bytes([1, 0, 7, 7]))
        self.send_command(LR1121_OP_SET_TX_PARAMS, bytes([14, 0]))
        self.send_command(LR1121_OP_SET_LORA_SYNC_WORD, bytes([0x12]))

        self.clear_irq()
        self.set_dio_irq_params(IRQ_TX_DONE | IRQ_TIMEOUT | IRQ_CMD_ERROR | IRQ_ERROR, 0)
        self.clear_irq()
        logger.info("LoRa Configuration applied successfully.")

    # --------------------------------------------------------------------------
    # TX Helpers
    # --------------------------------------------------------------------------
    def _write_buffer8(self, payload: bytes):
        self.send_command(LR1121_OP_WRITE_BUFFER8, payload)

    def transmit_payload(self, payload: bytes, timeout_ms=15000) -> bool:
        self._dio9_clear_trigger()
        self.clear_irq()

        payload_len = len(payload)
        logger.debug("Preparing to transmit %d bytes. Hex: %s", payload_len, payload.hex())

        # Update packet parameters for current payload length
        preamble, header_type, crc_en, invert_iq = 12, 0x00, 0x01, 0x00
        self.send_command(
            LR1121_OP_SET_PACKET_PARAMS,
            preamble.to_bytes(2, "big") + bytes([header_type, payload_len, crc_en, invert_iq]),
        )

        self._write_buffer8(payload)
        self.send_command(LR1121_OP_SET_TX, b"\x00\x00\x00")
        self._wait_busy()

        t0 = time.ticks_ms()
        while not self.dio9_triggered:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                logger.error("⏱ Timeout waiting for DIO9 (TX DONE).")
                break
            time.sleep_ms(10)

        self._dio9_clear_trigger()
        _, _, irq = self.get_status()

        if irq & IRQ_TX_DONE:
            logger.info("✅ Radio TX_DONE. IRQ=0x%08X", irq)
            return True
        if irq & IRQ_CMD_ERROR:
            logger.error("❌ Radio CMD_ERROR. IRQ=0x%08X", irq)
            return False
        if irq & IRQ_TIMEOUT:
            logger.error("❌ Radio TIMEOUT. IRQ=0x%08X", irq)
            return False
        if irq & IRQ_ERROR:
            logger.error("❌ Radio ERROR. IRQ=0x%08X", irq)
            return False

        logger.error("❌ TX failed or unknown state. IRQ=0x%08X", irq)
        return False

    def transmit_json(self, obj, timeout_ms=15000, chunk_size=200) -> bool:
        raw = ujson.dumps(obj).encode("utf-8")
        if len(raw) <= chunk_size:
            logger.debug("Sending unfragmented JSON...")
            return self.transmit_payload(raw, timeout_ms=timeout_ms)

        # Chunked sending logic
        chunks = [raw[i : i + chunk_size] for i in range(0, len(raw), chunk_size)]
        total = len(chunks)
        
        for idx, part in enumerate(chunks, start=1):
            frame = b"J" + bytes([total & 0xFF, idx & 0xFF]) + part
            logger.debug("Sending JSON chunk %d/%d (%d bytes)", idx, total, len(frame))
            if not self.transmit_payload(frame, timeout_ms=timeout_ms):
                logger.error("❌ Failed to send JSON chunk %d/%d", idx, total)
                return False

        logger.info("✅ Multi-chunk JSON transmit finished successfully.")
        return True

    # --------------------------------------------------------------------------
    # RX Helpers
    # --------------------------------------------------------------------------
    def _read_buffer8(self, length: int, offset: int = 0) -> bytes:
        if length <= 0: return b""

        self._wait_busy()
        self.nss.value(0)
        self.spi.write(struct.pack(">H", LR1121_OP_READ_BUFFER8))
        self.spi.write(bytes([(offset & 0xFF), (length & 0xFF)]))
        self.nss.value(1)

        self._wait_busy()
        self.nss.value(0)
        resp = self.spi.read(length + 1)
        self.nss.value(1)

        if not resp: return b""
        
        stat1, data = resp[0], resp[1:]
        logger.debug("ReadBuffer8: offset=%d len=%d stat1=0x%02X", offset, length, stat1)
        return data

    def receive_payload(self, timeout_ms: int = 15000, max_len: int = 255) -> bytes | None:
        self._dio9_clear_trigger()
        self.clear_irq()

        self.set_dio_irq_params(IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_CMD_ERROR | IRQ_ERROR, 0)
        self.clear_irq()

        self.send_command(LR1121_OP_SET_RX, b"\xff\xff\xff")

        t0 = time.ticks_ms()
        while not self.dio9_triggered:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                logger.debug("RX host-timeout (normal scanning behavior).")
                break
            time.sleep_ms(5)

        self._dio9_clear_trigger()
        _, _, irq = self.get_status()

        if irq & IRQ_RX_DONE:
            payload_len, start = self.get_rx_buffer_status()

            # 👇 --- НОВЫЙ КОД --- 👇
            rssi, snr, sig_rssi = self.get_packet_status()
            
            if payload_len <= 0:
                logger.warning("RX_DONE triggered but payload length is 0.")
                return None
            if payload_len > max_len:
                logger.warning("RX payload too large (%d), clamping to %d", payload_len, max_len)
                payload_len = max_len

            raw = self._read_buffer8(payload_len, offset=start)
            
            # Красиво логируем вместе с уровнем сигнала!
            logger.info("✅ Radio RX_DONE. Fetched %d bytes.", len(raw))
            logger.info("📊 Signal Quality: RSSI=%.1f dBm | SNR=%.2f dB", rssi, snr)
            
            return raw

        if irq & IRQ_TIMEOUT:
            logger.warning("❌ Radio RX TIMEOUT. IRQ=0x%08X", irq)
            return None
        if irq & IRQ_CMD_ERROR:
            logger.error("❌ Radio RX CMD_ERROR. IRQ=0x%08X", irq)
            return None
        if irq & IRQ_ERROR:
            logger.error("❌ Radio RX ERROR. IRQ=0x%08X", irq)
            return None

        return None

    def receive_json(self, timeout_ms: int = 15000, max_len: int = 255):
        data = self.receive_payload(timeout_ms=timeout_ms, max_len=max_len)
        if not data: return None

        try:
            obj = ujson.loads(data.decode("utf-8"))
            logger.info("✅ Raw JSON successfully parsed.")
            return obj
        except Exception as e:
            logger.error("❌ JSON parse failed: %s. Raw=%s", str(e), data.hex())
            return None
        
    def get_rx_buffer_status(self):
        resp = self.read_command(LR1121_OP_GET_RX_BUFFER_STATUS, 3)
        stat1, payload_len, start_offset = resp[0], resp[1], resp[2]
        logger.debug("GetRxBufferStatus: Stat1=0x%02X len=%d offset=%d", stat1, payload_len, start_offset)
        return payload_len, start_offset
    
    def get_packet_status(self):
        """
        Команда 0x021B (GetPacketStatus). 
        Для LoRa возвращает 4 байта: [Stat1, RssiPkt, SnrPkt, SignalRssiPkt]
        """
        resp = self.read_command(LR1121_OP_GET_PACKET_STATUS, 4)
        stat1 = resp[0]
        
        # RssiPkt: Средний уровень сигнала за время приема пакета
        rssi_pkt = -resp[1] / 2.0
        
        # SnrPkt: Отношение сигнал/шум. Signed 8-bit, шаг 0.25 dB
        snr_raw = resp[2]
        if snr_raw > 127:
            snr_raw -= 256
        snr_pkt = snr_raw / 4.0
        
        # SignalRssiPkt: Уровень именно полезного LoRa-сигнала
        signal_rssi_pkt = -resp[3] / 2.0
        
        logger.debug("GetPacketStatus: Stat1=0x%02X, RSSI=%.1f dBm, SNR=%.2f dB", stat1, rssi_pkt, snr_pkt)
        
        # Сохраняем в объекте, чтобы можно было прочитать из main.py
        self.last_rssi = rssi_pkt
        self.last_snr = snr_pkt
        
        return rssi_pkt, snr_pkt, signal_rssi_pkt


# ==============================================================================
# CLI Test
# ==============================================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    log_cli = logging.getLogger("CLI")

    # Pins
    SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN = 5, 3, 6, 7
    BUSY_PIN, RST_PIN, DIO9_PIN = 34, 8, 36

    spi = SPI(
        1, baudrate=LR1121_SPI_BAUDRATE, polarity=0, phase=0,
        sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN)
    )

    radio = LR1121(
        spi_bus=spi, nss_pin=Pin(NSS_PIN), busy_pin=Pin(BUSY_PIN),
        rst_pin=Pin(RST_PIN), dio9_pin=Pin(DIO9_PIN)
    )

    radio.init_radio()

    payload = {"type": "ping", "ts_ms": time.ticks_ms(), "msg": "Hello from LR1121"}
    log_cli.info("Sending test JSON payload: %s", payload)

    if radio.transmit_json(payload, timeout_ms=3000):
        log_cli.info("✅ JSON transmit OK.")
    else:
        log_cli.error("❌ JSON transmit failed.")

    log_cli.info("--- Switching to RX MODE ---")
    rx_obj = radio.receive_json(timeout_ms=10000)
    
    if rx_obj:
        log_cli.info("✅ JSON received: %s", rx_obj)
    else:
        log_cli.warning("❌ No JSON received within timeout.")