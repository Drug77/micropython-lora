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

    def _draw_battery_icon(self, x, y, label, percent, is_charging):
        """
        Battery widget: label (3 chars) + icon + percentage.
        Occupies ~55 px wide, 19 px tall.

        Pixel layout (relative to x, y):
          [x]         label text (24 px)
          [x+26, y]   battery body rect  26×9 px
          [x+52, y+3] nub (positive terminal) 3×3 px
          [x+26, y+11] percentage text
        """
        if not self.display:
            return
        self.display.text(label[:3], x, y, 1)

        bx, by, bw, bh = x + 26, y, 26, 9
        self.display.rect(bx, by, bw, bh, 1)
        self.display.fill_rect(bx + bw, by + 3, 3, 3, 1)   # nub

        fill = max(0, int((bw - 4) * percent / 100))
        if fill > 0:
            self.display.fill_rect(bx + 2, by + 2, fill, bh - 4, 1)

        pct_str = f"{percent}%" + ("+" if is_charging else "")
        self.display.text(pct_str, bx, y + 11, 1)

    def show_rx_alarm(self, signal_str, car_pct, car_charging,
                      my_pct, my_charging, radar_str, stats_str):
        """
        Full alarm screen with dual battery indicators (128×64).

        y= 0..12  inverted header "ALARM RX" + antenna icon
        y=14..21  signal quality (R/SNR)
        y=23..31  battery bodies side-by-side (separator at x=63)
        y=34..41  battery percentage labels
        y=42..49  radar detection string
        y=51..59  stats (time, rx count, lost)
        """
        if not self.display:
            return
        self.display.fill(0)

        self.draw_header("ALARM RX", show_antenna=True)

        # Signal quality row
        self.display.text(signal_str[:16], 0, 14, 1)

        # Vertical separator between the two battery widgets
        self.display.vline(63, 22, 20, 1)

        # Left: CAR battery (TX unit, data from payload)
        self._draw_battery_icon(0, 23, "CAR", car_pct, car_charging)

        # Right: MY battery (this RX unit)
        self._draw_battery_icon(65, 23, "YOU", my_pct, my_charging)

        # Radar detection
        self.display.text(radar_str[:16], 0, 42, 1)

        # Stats line (time + rx/lost counters)
        self.display.text(stats_str[:16], 0, 52, 1)

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