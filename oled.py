import framebuf
import logging
from ssd1306 import SSD1306_I2C


OLED_WIDTH, OLED_HEIGHT = 128, 64
DISPLAY_ADDR = 0x3C

logger = logging.getLogger(__name__)


class OLEDDisplay:
    def __init__(self, i2c_bus):
        logger.debug("Initializing OLED on I2C (addr: 0x%02X)", DISPLAY_ADDR)
        try:
            self.display = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_bus, addr=DISPLAY_ADDR)
            self.clear()
            logger.info("Display initialized.")
        except Exception as e:
            logger.error("Display Init failed: %s", str(e))
            self.display = None

    def clear(self):
        if self.display:
            self.display.fill(0)
            self.display.show()

    # ==================================================================
    # Pixel-art icons
    # ==================================================================

    def _draw_mini_battery(self, x, y, percent, color=0):
        """Mini battery icon 12x7: 10x7 body + 2x3 nub.

        Designed for inverted header (color=0 = black on white).
        Fill level: 0-6px interior width.
        """
        d = self.display
        d.rect(x, y, 10, 7, color)                    # body outline
        d.fill_rect(x + 10, y + 2, 2, 3, color)       # positive terminal nub
        fill = max(0, int(6 * percent / 100))
        if fill > 0:
            d.fill_rect(x + 2, y + 2, fill, 3, color) # charge fill

    def _draw_wifi_icon(self, x, y, color=0):
        """WiFi signal icon 7x7. Simplified arcs + dot."""
        d = self.display
        # Outer arc
        d.pixel(x + 3, y, color)
        d.pixel(x + 1, y + 1, color)
        d.pixel(x + 5, y + 1, color)
        d.pixel(x, y + 2, color)
        d.pixel(x + 6, y + 2, color)
        # Middle arc
        d.pixel(x + 3, y + 2, color)
        d.pixel(x + 2, y + 3, color)
        d.pixel(x + 4, y + 3, color)
        # Inner arc
        d.pixel(x + 3, y + 4, color)
        # Dot
        d.pixel(x + 3, y + 6, color)

    def _draw_lock_icon(self, x, y, locked=True, color=1):
        """Padlock icon 7x9. locked=True: closed shackle. locked=False: open."""
        d = self.display
        # Shackle (top arc)
        if locked:
            d.vline(x + 1, y, 4, color)
            d.vline(x + 5, y, 4, color)
            d.hline(x + 2, y, 2, color)
        else:
            d.vline(x + 1, y, 4, color)
            d.vline(x + 5, y - 2, 4, color)
            d.hline(x + 2, y, 2, color)
        # Body (rectangle)
        d.fill_rect(x, y + 3, 7, 6, color)
        # Keyhole (inverted color)
        kc = 0 if color == 1 else 1
        d.pixel(x + 3, y + 5, kc)
        d.vline(x + 3, y + 6, 2, kc)

    def _draw_alert_icon(self, x, y, color=1):
        """Alert triangle icon 9x8 with '!' inside."""
        d = self.display
        # Triangle outline
        d.pixel(x + 4, y, color)
        d.hline(x + 3, y + 1, 3, color)
        d.pixel(x + 2, y + 2, color)
        d.pixel(x + 6, y + 2, color)
        d.pixel(x + 1, y + 3, color)
        d.pixel(x + 7, y + 3, color)
        d.pixel(x + 1, y + 4, color)
        d.pixel(x + 7, y + 4, color)
        d.pixel(x, y + 5, color)
        d.pixel(x + 8, y + 5, color)
        d.pixel(x, y + 6, color)
        d.pixel(x + 8, y + 6, color)
        d.hline(x, y + 7, 9, color)
        # Exclamation mark inside
        d.vline(x + 4, y + 3, 3, color)
        d.pixel(x + 4, y + 6, color)

    def _draw_antenna_icon(self, x, y, color=0):
        """Antenna/radio icon for header (inverted colors by default)."""
        d = self.display
        d.pixel(x + 4, y, color)
        d.hline(x + 3, y + 1, 3, color)
        d.hline(x + 2, y + 2, 5, color)
        d.hline(x + 1, y + 3, 7, color)
        d.vline(x + 4, y + 4, 6, color)
        d.pixel(x + 4, y + 10, color)

    # ==================================================================
    # Large text rendering (2x scale)
    # ==================================================================

    def _draw_large_text(self, x, y, text, color=1, scale=2):
        """Render text at scale*8 px per character using framebuf scaling."""
        if not self.display:
            return
        char_buf = bytearray(8)
        fb = framebuf.FrameBuffer(char_buf, 8, 8, framebuf.MONO_VLSB)
        for i, ch in enumerate(text):
            fb.fill(0)
            fb.text(ch, 0, 0, 1)
            for py in range(8):
                for px in range(8):
                    if fb.pixel(px, py):
                        self.display.fill_rect(
                            x + i * 8 * scale + px * scale,
                            y + py * scale,
                            scale, scale, color)

    # ==================================================================
    # Header with status bar
    # ==================================================================

    def draw_header(self, title, bat1_pct=None, bat2_pct=None, wifi=False,
                    show_antenna=False, locked=None):
        """Draw inverted header bar with optional status icons.

        Layout (128x13px, white background):
        [TITLE]  [lock] [antenna] [wifi] [bat1 XX] [bat2 XX]
        """
        if not self.display:
            return
        h_height = 13
        self.display.fill_rect(0, 0, OLED_WIDTH, h_height, 1)
        self.display.text(title[:8], 2, 3, 0)

        rx = 126  # Right edge cursor (build icons right-to-left)

        # Battery 2 (remote device)
        if bat2_pct is not None:
            pct_str = str(bat2_pct)
            tw = len(pct_str) * 8
            rx -= tw
            self.display.text(pct_str, rx, 3, 0)
            rx -= 13  # battery icon width + gap
            self._draw_mini_battery(rx, 3, bat2_pct, color=0)
            rx -= 2  # gap

        # Battery 1 (local device)
        if bat1_pct is not None:
            pct_str = str(bat1_pct)
            tw = len(pct_str) * 8
            rx -= tw
            self.display.text(pct_str, rx, 3, 0)
            rx -= 13
            self._draw_mini_battery(rx, 3, bat1_pct, color=0)
            rx -= 2

        # WiFi icon
        if wifi:
            rx -= 8
            self._draw_wifi_icon(rx, 3, color=0)
            rx -= 2

        # Antenna icon
        if show_antenna:
            rx -= 9
            self._draw_antenna_icon(rx, 1, color=0)
            rx -= 2

        # Lock icon (in header, inverted)
        if locked is not None:
            rx -= 8
            # Lock body in header: inverted colors
            self._draw_lock_header(rx, 2, locked)

    def _draw_lock_header(self, x, y, locked):
        """Compact lock for header (inverted: color=0 on white bg)."""
        d = self.display
        c = 0  # black on white header
        # Shackle
        if locked:
            d.vline(x + 1, y, 3, c)
            d.vline(x + 5, y, 3, c)
            d.hline(x + 2, y, 2, c)
        else:
            d.vline(x + 1, y, 3, c)
            d.vline(x + 5, y - 1, 3, c)
            d.hline(x + 2, y, 2, c)
        # Body
        d.rect(x, y + 2, 7, 5, c)
        # Keyhole
        d.pixel(x + 3, y + 4, c)
        d.vline(x + 3, y + 5, 1, c)

    # ==================================================================
    # Progress bars
    # ==================================================================

    def draw_progress_bar(self, y, percent):
        if not self.display:
            return
        width = OLED_WIDTH - 4
        height = 8
        x = 2
        self.display.rect(x, y, width, height, 1)
        fill_width = int((width - 4) * (percent / 100.0))
        if fill_width > 0:
            self.display.fill_rect(x + 2, y + 2, fill_width, height - 4, 1)

    def update_progress_only(self, y, percent):
        """Redraw only the progress bar without full screen refresh."""
        if not self.display:
            return
        width = OLED_WIDTH - 4
        height = 8
        x = 2
        self.display.fill_rect(x, y, width, height, 0)
        self.display.rect(x, y, width, height, 1)
        fill_width = int((width - 4) * (percent / 100.0))
        if fill_width > 0:
            self.display.fill_rect(x + 2, y + 2, fill_width, height - 4, 1)
        self.display.show()

    # ==================================================================
    # Splash screen (cold boot)
    # ==================================================================

    def show_splash(self, mode, version):
        """Boot splash: mode in large text centered, version small below."""
        if not self.display:
            return
        self.display.fill(0)
        # Mode in 2x scale (16px per char height)
        text_w = len(mode) * 16
        x = (OLED_WIDTH - text_w) // 2
        y = 12
        self._draw_large_text(x, y, mode, color=1, scale=2)
        # Version small centered below
        ver_str = f"v{version}"
        vx = (OLED_WIDTH - len(ver_str) * 8) // 2
        self.display.text(ver_str, vx, 44, 1)
        self.display.show()

    # ==================================================================
    # Status screens
    # ==================================================================

    def show_status(self, title, line1="", line2="", line3="", progress=None,
                    antenna=False, bat1=None, bat2=None, wifi=False, locked=None):
        """General status screen with header + up to 3 text lines + progress bar."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header(title, bat1_pct=bat1, bat2_pct=bat2, wifi=wifi,
                         show_antenna=antenna, locked=locked)
        self.display.text(line1[:16], 2, 16, 1)
        self.display.text(line2[:16], 2, 28, 1)
        self.display.text(line3[:16], 2, 40, 1)
        if progress is not None:
            self.draw_progress_bar(54, progress)
        self.display.show()

    def show_tx_armed(self, radar_str, bat_pct, charging, idle_s, idle_max, pkt,
                      remote_bat=None):
        """TX armed patrol screen."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header("CAR ARM", bat1_pct=bat_pct, bat2_pct=remote_bat,
                         show_antenna=True, locked=True)
        self.display.text(radar_str[:16], 2, 16, 1)
        bat_str = f"Bat:{bat_pct}%{'+'if charging else ''}"
        self.display.text(bat_str, 2, 28, 1)
        self.display.text(f"Pkt:{pkt}", 2, 40, 1)
        pct = min(int(idle_s / idle_max * 100), 100) if idle_max > 0 else 0
        self.draw_progress_bar(54, pct)
        idle_lbl = f"{int(idle_s)}s/{idle_max}s"
        self.display.text(idle_lbl, 70, 40, 1)
        self.display.show()

    def show_tx_disarmed(self, bat_pct, charging, pkt, remote_bat=None):
        """TX disarmed minimal screen."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header("CAR", bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=False)
        # Lock icon large in center
        self._draw_lock_icon(52, 18, locked=False, color=1)
        self.display.text("DISARMED", 32, 34, 1)
        bat_str = f"Bat:{bat_pct}% Pkt:{pkt}"
        self.display.text(bat_str, 2, 52, 1)
        self.display.show()

    def show_rx_idle(self, bat_pct, charging, rx_cnt, lost_cnt, remote_bat=None,
                     locked=None, date_str="", time_str=""):
        """RX idle — waiting for packets."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header("YOU", bat1_pct=bat_pct, bat2_pct=remote_bat,
                         show_antenna=True, locked=locked)
        if date_str:
            self.display.text(date_str[:16], 2, 16, 1)
        if time_str:
            self.display.text(time_str[:16], 2, 26, 1)
        self.display.text("Listening...", 20, 40, 1)
        self.display.text(f"RX:{rx_cnt} L:{lost_cnt}", 2, 52, 1)
        self.display.show()

    def show_rx_alarm(self, signal_str, car_pct, car_charging,
                      my_pct, my_charging, radar_str, stats_str, is_sos=False):
        """RX alarm screen with signal info."""
        if not self.display:
            return
        self.display.fill(0)
        title = "SOS!" if is_sos else "ALARM!"
        self.draw_header(title, bat1_pct=my_pct, bat2_pct=car_pct,
                         show_antenna=True)
        # Alert icon
        self._draw_alert_icon(2, 16, color=1)
        self.display.text(signal_str[:13], 14, 16, 1)
        self.display.text(radar_str[:16], 2, 28, 1)
        self.display.text(stats_str[:16], 2, 40, 1)
        # Car battery detail
        car_str = f"Car:{car_pct}%{'+'if car_charging else ''}"
        self.display.text(car_str, 2, 52, 1)
        self.display.show()

    def show_rx_box(self, title, message, signal_str, date_str, stats_str,
                    bat1=None, bat2=None, locked=None):
        """Compact box for displaying received packet info."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header(title, bat1_pct=bat1, bat2_pct=bat2,
                         show_antenna=True, locked=locked)
        box_y = 15
        box_h = OLED_HEIGHT - box_y
        self.display.rect(0, box_y, OLED_WIDTH, box_h, 1)
        self.display.text(message[:15], 4, box_y + 3, 1)
        self.display.text(signal_str[:15], 4, box_y + 14, 1)
        self.display.text(date_str[:15], 4, box_y + 25, 1)
        self.display.text(stats_str[:15], 4, box_y + 36, 1)
        self.display.show()

    def show_brief_status(self, mode, bat_pct, charging, armed, time_str="",
                          remote_bat=None):
        """Brief status screen for button wake (TX shows status then sleeps)."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header(mode, bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=armed)
        self._draw_lock_icon(52, 18, locked=armed, color=1)
        state = "ARMED" if armed else "DISARMED"
        sx = (OLED_WIDTH - len(state) * 8) // 2
        self.display.text(state, sx, 34, 1)
        if time_str:
            tx = (OLED_WIDTH - len(time_str) * 8) // 2
            self.display.text(time_str, tx, 46, 1)
        bat_str = f"Bat:{bat_pct}%{'+'if charging else ''}"
        bx = (OLED_WIDTH - len(bat_str) * 8) // 2
        self.display.text(bat_str, bx, 56, 1)
        self.display.show()

    def show_sos_sent(self, bat_pct, remote_bat=None):
        """SOS sent confirmation screen."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header("SOS", bat1_pct=bat_pct, bat2_pct=remote_bat)
        self._draw_alert_icon(56, 16, color=1)
        self.display.text("SOS SENT!", 28, 30, 1)
        self.display.text("Signal sent", 20, 44, 1)
        self.display.show()
