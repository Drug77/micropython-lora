import time
from machine import Pin, ADC
import logging

log_bat = logging.getLogger(__name__)

class BatteryMonitor:
    def __init__(self, adc_pin=1):
        """
        На LilyGO T3S3 v1.2 пин измерения батареи - GPIO 1.
        """
        self.adc = ADC(Pin(adc_pin))
        # ATTN_11DB позволяет измерять напряжение до ~3.1V на пине
        self.adc.atten(ADC.ATTN_11DB)
        
        # Делитель напряжения (100k / 100k) делит напряжение на 2. 
        # Если напряжение занижается/завышается, можно слегка подкрутить этот множитель (например, 2.05 или 1.95)
        self.multiplier = 2.0 
        
        # Переменные для определения зарядки
        self.last_voltage = self.get_voltage()
        self.last_check_time = time.time()

    def get_voltage(self):
        """Возвращает напряжение аккумулятора в Вольтах"""
        # Метод read_uv() возвращает микровольты с учетом заводской калибровки ESP32-S3!
        # Это в 10 раз точнее, чем читать сырые значения 0-4095.
        uv = self.adc.read_uv()
        voltage = (uv / 1_000_000.0) * self.multiplier
        return round(voltage, 2)

    def get_percentage(self, voltage=None):
        """Переводит напряжение в проценты (очень примерная LiPo кривая)"""
        if voltage is None:
            voltage = self.get_voltage()
            
        if voltage >= 4.15: return 100
        if voltage >= 4.00: return 80
        if voltage >= 3.90: return 60
        if voltage >= 3.75: return 40
        if voltage >= 3.60: return 20
        if voltage <= 3.30: return 0
        
        # Линейная интерполяция для остатка
        return int((voltage - 3.3) / (4.15 - 3.3) * 100)

    def get_status(self):
        """Возвращает: Напряжение, Проценты и Статус зарядки (True/False)"""
        current_v = self.get_voltage()
        percent = self.get_percentage(current_v)
        
        is_charging = False
        
        # Если напряжение выше 4.25В - плата 100% питается от USB
        if current_v > 4.25:
            is_charging = True
            percent = 100
        else:
            # Сравниваем с прошлым замером. Если выросло на 0.02V - заряжаемся
            if (current_v - self.last_voltage) >= 0.02:
                is_charging = True
            # Если напряжение падает или стоит на месте - не заряжаемся
            elif current_v < self.last_voltage:
                is_charging = False

        # Обновляем историю
        self.last_voltage = current_v
        
        log_bat.debug("Bat: %.2fV, %d%%, Charging: %s", current_v, percent, is_charging)
        return current_v, percent, is_charging