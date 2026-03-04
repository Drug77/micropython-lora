import logging
from ssd1306 import SSD1306_I2C


OLED_WIDTH, OLED_HEIGHT = 128, 64
DISPLAY_ADDR = 0x3C

logger = logging.getLogger(__name__)

# ==============================================================================
# Продвинутый класс для OLED экрана
# ==============================================================================
class OLEDDisplay:
    def __init__(self, i2c_bus):
        logger.debug("Initializing OLED on I2C (addr: 0x%02X)", DISPLAY_ADDR)
        try:
            self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_bus, addr=DISPLAY_ADDR)
            self.clear()
            logger.info("✅ Display initialized.")
        except Exception as e:
            logger.error("❌ Display Init failed: %s", str(e))
            self.display = None

    def clear(self):
        if self.display:
            self.display.fill(0)
            self.display.show()

    def draw_header(self, title, sub_title="", show_antenna=False):
        if not self.display: return
        h_height = 20 if sub_title else 13
        self.display.fill_rect(0, 0, OLED_WIDTH, h_height, 1)
        self.display.text(title, 2, 2, 0)
        
        if sub_title:
            self.display.text(sub_title, 2, 11, 0)
            
        if show_antenna:
            ax, ay = 110, 2
            self.display.pixel(ax+4, ay, 0)
            self.display.hline(ax+3, ay+1, 3, 0)
            self.display.hline(ax+2, ay+2, 5, 0)
            self.display.hline(ax+1, ay+3, 7, 0)
            self.display.vline(ax+4, ay+4, 6, 0)
            self.display.pixel(ax+4, ay+10, 0)

    def draw_progress_bar(self, y, percent):
        if not self.display: return
        width = OLED_WIDTH - 4
        height = 8
        x = 2
        self.display.rect(x, y, width, height, 1)
        fill_width = int((width - 4) * (percent / 100.0))
        if fill_width > 0:
            self.display.fill_rect(x + 2, y + 2, fill_width, height - 4, 1)

    def update_progress_only(self, y, percent):
        """Перерисовывает только шкалу прогресса без моргания экрана"""
        if not self.display: return
        width = OLED_WIDTH - 4
        height = 8
        x = 2
        self.display.fill_rect(x, y, width, height, 0) # Очистка старой полосы
        self.display.rect(x, y, width, height, 1)      # Рамка
        fill_width = int((width - 4) * (percent / 100.0))
        if fill_width > 0:
            self.display.fill_rect(x + 2, y + 2, fill_width, height - 4, 1)
        self.display.show()

    def show_status(self, title, line1="", line2="", line3="", progress=None, antenna=False):
        if not self.display: return
        self.display.fill(0)
        self.draw_header(title, show_antenna=antenna)
        self.display.text(line1, 2, 18, 1)
        self.display.text(line2, 2, 30, 1)
        self.display.text(line3, 2, 42, 1)
        if progress is not None:
            self.draw_progress_bar(54, progress)
        self.display.show()

    def show_rx_box(self, title, message, signal_str, date_str, stats_str):
        """Специальное компактное окно для отображения пакета со статистикой"""
        if not self.display: return
        self.display.fill(0)
        
        self.draw_header(title, sub_title=signal_str)
        box_y = 22
        box_h = OLED_HEIGHT - box_y
        self.display.rect(0, box_y, OLED_WIDTH, box_h, 1)
        
        # Вывод самого сообщения
        self.display.text(message[:15], 4, box_y + 4, 1)
        
        # Вывод даты и статистики (ровно по 16 символов макс)
        self.display.text(date_str, 4, box_y + 16, 1)
        self.display.text(stats_str, 4, box_y + 26, 1)
            
        self.display.show()