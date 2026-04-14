import time
from machine import UART
import struct

class LD2410B:
    """
    Высокопроизводительный неблокирующий драйвер для радарного датчика FMCW HLK-LD2410B.
    Реализует парсинг UART-потока на скорости 256000 бод с обработкой Little Endian.
    """
    
    # Константы заголовков фрейма отчета (Telemetry Report Frames)
    REPORT_HEAD = b'\xf4\xf3\xf2\xf1'
    REPORT_TAIL = b'\xf8\xf7\xf6\xf5'
    
    # Идентификаторы типов данных
    TARGET_BASIC_INFO = 0x02
    ENGINEERING_MODE = 0x01
    
    def __init__(self, uart_id=1, tx_pin=9, rx_pin=10, baudrate=256000):
        """
        Инициализация аппаратного интерфейса UART. 
        Настройка портов и скорости передачи 256000 бод (8N1).
        """
        # В зависимости от платформы (ESP32/RP2040) синтаксис UART может незначительно отличаться
        self.uart = UART(uart_id, baudrate=baudrate, tx=tx_pin, rx=rx_pin)
        self.uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)
        self.buffer = bytearray()
        
        # Переменные хранения состояния цели (Target Basic Info)
        self.target_state = 0
        self.moving_distance = 0
        self.moving_energy = 0
        self.stationary_distance = 0
        self.stationary_energy = 0
        self.detection_distance = 0
        
        # Матрицы энергий для Инженерного режима (Engineering Mode Data)
        self.max_moving_gate = 0
        self.max_stationary_gate = 0
        self.gate_moving_energy = [0] * 9
        self.gate_stationary_energy = [0] * 9
        
        # Данные опциональных датчиков
        self.light_sensor_val = 0
        self.out_pin_state = 0
        
    def read_frame(self):
        """
        Метод конечного автомата для неблокирующего чтения и парсинга UART буфера.
        Возвращает True, если был успешно валидирован и извлечен полный фрейм телеметрии.
        """
        if self.uart.any():
            data = self.uart.read()
            if data:
                self.buffer.extend(data)
                
        # Стратегия предотвращения деградации памяти (Garbage Collector Overhead).
        # Если контроллер завис и не читал данные, срезаем старый мусор, оставляя последние байты.
        if len(self.buffer) > 1024:
            self.buffer = self.buffer[-512:]
            
        # Поиск паттерна синхронизации (F4 F3 F2 F1)
        head_idx = self.buffer.find(self.REPORT_HEAD)
        if head_idx == -1:
            return False
            
        # Сброс фрагментированных данных до актуального заголовка
        if head_idx > 0:
            self.buffer = self.buffer[head_idx:]
            
        # Проверка наличия минимально необходимого количества байт 
        # (Заголовок:4 + Длина:2 + Базовая нагрузка:10 + Хвост:4) = 20 байт
        if len(self.buffer) < 20:
            return False
            
        # Вычисление заявленной длины блока полезной нагрузки (Little Endian архитектура)
        data_len = self.buffer[4] | (self.buffer[5] << 8)
        total_frame_len = 4 + 2 + data_len + 4 
        
        # Ожидание накопления оставшихся байт фрейма
        if len(self.buffer) < total_frame_len:
            return False
            
        # Проверка целостности терминирующей последовательности (F8 F7 F6 F5)
        tail_idx = 4 + 2 + data_len
        tail_bytes = bytes(self.buffer[tail_idx : tail_idx+4])
        
        if tail_bytes!= self.REPORT_TAIL:
            # Обнаружена рассинхронизация или коллизия. 
            # Удаляем ложный заголовок и перезапускаем поиск.
            self.buffer = self.buffer[4:]
            return False
            
        # Изоляция подтвержденного блока полезной нагрузки
        payload = self.buffer[6 : tail_idx]
        
        # Усечение буфера (удаление распарсенного фрейма)
        self.buffer = self.buffer[total_frame_len:]
        
        # Декодирование полезной нагрузки
        self._parse_payload(payload)
        return True

    def _parse_payload(self, payload):
        """
        Внутренний метод извлечения переменных из валидированной полезной нагрузки.
        """
        if len(payload) < 2:
            return
            
        data_type = payload[0]
        intra_head = payload[1]
        
        # Проверка маркера начала семантического блока (0xAA)
        if intra_head!= 0xAA:
            return
            
        if data_type == self.TARGET_BASIC_INFO or data_type == self.ENGINEERING_MODE:
            # Извлечение базовой телеметрии, присутствующей в обоих режимах
            # Индексы смещены на 2 байта от начала payload (Type + Intra-head)
            self.target_state = payload[2]

            # Декодирование расстояний путем побитового сдвига (Little Endian)
            self.moving_distance = payload[3] | (payload[4] << 8)
            self.moving_energy = payload[5]
            
            self.stationary_distance = payload[6] | (payload[7] << 8)
            self.stationary_energy = payload[8]
            
            self.detection_distance = payload[9] | (payload[10] << 8)
            
            # Расширенный парсинг для инженерного режима
            if data_type == self.ENGINEERING_MODE and len(payload) >= 31:
                self.max_moving_gate = payload[11]
                self.max_stationary_gate = payload[12]
                
                # Массив энергии движения по 9 гейтам
                for i in range(9):
                    self.gate_moving_energy[i] = payload[13 + i]
                    
                # Массив энергии статики по 9 гейтам
                for i in range(9):
                    self.gate_stationary_energy[i] = payload[22 + i]
            
            # Опциональный парсинг датчика освещенности (учитывая переменный размер payload)
            # Ищем маркер конца семантического блока 0x55 0x00 в последних двух байтах
            if len(payload) >= 15 and payload[-2] == 0x55 and payload[-1] == 0x00:
                self.light_sensor_val = payload[-4]
                self.out_pin_state = payload[-3]
                
    def print_telemetry(self):
        """
        Форматированный вывод текущего состояния радара.
        """
        state_map = {0: "Clear", 1: "Moving", 2: "Stationary", 3: "Mov & Stat"}
        status = state_map.get(self.target_state, "Unknown")
        
        print(f"[{status}] Det Dist: {self.detection_distance}cm | "
              f"Mov: {self.moving_distance}cm (E:{self.moving_energy}) | "
              f"Stat: {self.stationary_distance}cm (E:{self.stationary_energy}) | "
              f"Light: {self.light_sensor_val}")

# Пример интеграции в основной цикл микроконтроллера:
if __name__ == '__main__':
    # Подключение к UART2 на пинах 17(TX) и 16(RX), характерно для ESP32
    radar = LD2410B(uart_id=1, tx_pin=9, rx_pin=10, baudrate=256000)
    print("FMCW Радар HLK-LD2410B инициализирован. Запуск сбора телеметрии...")
    
    while True:
        if radar.read_frame():
            radar.print_telemetry()
        # Имитация выполнения других задач микроконтроллером
        time.sleep_ms(20)