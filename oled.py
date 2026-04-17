import framebuf
import logging
from ssd1306 import SSD1306_I2C
from font_cyr import char_data
from lang import t


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

    def _text(self, s, x, y, color=1):
        """Text with Cyrillic support. ASCII uses built-in, Cyrillic uses font_cyr."""
        if not self.display:
            return
        for i, ch in enumerate(s):
            cx = x + i * 8
            if cx >= OLED_WIDTH:
                break
            data = char_data(ch)
            if data is not None:
                for row in range(8):
                    b = data[row]
                    for col in range(8):
                        if b & (0x80 >> col):
                            self.display.pixel(cx + col, y + row, color)
            else:
                self.display.text(ch, cx, y, color)

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
        """Padlock icon 9x14. locked=True: closed shackle. locked=False: open."""
        d = self.display
        # Shackle (top arc) — 2px thick
        if locked:
            d.vline(x + 1, y, 6, color)
            d.vline(x + 2, y, 6, color)
            d.vline(x + 6, y, 6, color)
            d.vline(x + 7, y, 6, color)
            d.hline(x + 2, y, 5, color)
            d.hline(x + 2, y + 1, 5, color)
        else:
            d.vline(x + 1, y + 1, 5, color)
            d.vline(x + 2, y + 1, 5, color)
            d.vline(x + 6, y - 2, 5, color)
            d.vline(x + 7, y - 2, 5, color)
            d.hline(x + 2, y + 1, 5, color)
            d.hline(x + 2, y + 2, 5, color)
        # Body (rectangle)
        d.fill_rect(x, y + 5, 9, 9, color)
        # Keyhole (inverted color)
        kc = 0 if color == 1 else 1
        d.pixel(x + 4, y + 8, kc)
        d.vline(x + 4, y + 9, 3, kc)

    def _draw_heart_icon(self, x, y, color=1):
        """Heart icon 9x8."""
        d = self.display
        d.hline(x + 1, y, 3, color)
        d.hline(x + 5, y, 3, color)
        d.hline(x, y + 1, 9, color)
        d.hline(x, y + 2, 9, color)
        d.hline(x + 1, y + 3, 7, color)
        d.hline(x + 1, y + 4, 7, color)
        d.hline(x + 2, y + 5, 5, color)
        d.hline(x + 3, y + 6, 3, color)
        d.pixel(x + 4, y + 7, color)

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
                    locked=None):
        """Draw inverted header bar with optional status icons.

        Layout (128x13px, white background):
        [TITLE]  [lock] [antenna] [wifi] [bat1 XX] [bat2 XX]
        """
        if not self.display:
            return
        h_height = 13
        self.display.fill_rect(0, 0, OLED_WIDTH, h_height, 1)
        self._text(title[:8], 2, 3, 0)

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

        # Lock icon — right after title, with 3px gap
        if locked is not None:
            lock_x = len(title[:8]) * 8 + 5
            if lock_x + 9 <= rx:  # fits before batteries
                self._draw_lock_header(lock_x, 1, locked)

    def _draw_lock_header(self, x, y, locked):
        """Lock for header 9x11 (inverted: color=0 on white bg)."""
        d = self.display
        c = 0  # black on white header
        # Shackle — 2px thick lines
        if locked:
            d.vline(x + 1, y, 5, c)
            d.vline(x + 2, y, 5, c)
            d.vline(x + 6, y, 5, c)
            d.vline(x + 7, y, 5, c)
            d.hline(x + 2, y, 5, c)
        else:
            d.vline(x + 1, y + 1, 4, c)
            d.vline(x + 2, y + 1, 4, c)
            d.vline(x + 6, y - 1, 4, c)
            d.vline(x + 7, y - 1, 4, c)
            d.hline(x + 2, y + 1, 5, c)
        # Body
        d.fill_rect(x, y + 4, 9, 7, c)
        # Keyhole (white on black body)
        d.pixel(x + 4, y + 7, 1)
        d.vline(x + 4, y + 8, 2, 1)

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
                         locked=locked)
        self._text(line1[:16], 2, 16, 1)
        self._text(line2[:16], 2, 28, 1)
        self._text(line3[:16], 2, 40, 1)
        if progress is not None:
            self.draw_progress_bar(54, progress)
        self.display.show()

    def show_tx_armed(self, radar_state, distance, bat_pct, charging,
                      idle_s, idle_max, pkt, remote_bat=None):
        """TX armed — table layout like rx_box."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        self.draw_header(t("car"), bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=True)

        tbl_y = 14
        tbl_h = 38
        mid_x = 56

        # Outer border + divider
        d.rect(0, tbl_y, OLED_WIDTH, tbl_h, 1)
        d.vline(mid_x, tbl_y, tbl_h, 1)

        # Left top: ОХРАНА (inverted)
        cell_h = tbl_h // 2
        d.fill_rect(1, tbl_y + 1, mid_x - 1, cell_h - 1, 1)
        arm = t("armed")
        self._text(arm[:7], 3, tbl_y + 5, 0)

        # Left bottom: radar state
        d.hline(0, tbl_y + cell_h, mid_x + 1, 1)
        self._text(radar_state[:7], 3, tbl_y + cell_h + 5, 1)

        # Right: distance large
        right_w = OLED_WIDTH - mid_x - 1
        dist_w = len(distance) * 16
        dist_x = mid_x + 1 + (right_w - dist_w) // 2
        dist_y = tbl_y + (tbl_h - 16) // 2
        self._draw_large_text(dist_x, dist_y, distance, color=1, scale=2)

        # Bottom: idle progress
        pct = min(int(idle_s / idle_max * 100), 100) if idle_max > 0 else 0
        idle_lbl = f"{t('pkt')}:{pkt} {int(idle_s)}s/{idle_max}s"
        d.text(idle_lbl, 2, 55, 1)
        d.show()

    def show_tx_disarmed(self, bat_pct, charging, pkt, remote_bat=None):
        """TX disarmed — table layout with lock icon."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        self.draw_header(t("car"), bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=False)

        tbl_y = 14
        tbl_h = 38
        mid_x = 56

        # Outer border + divider
        d.rect(0, tbl_y, OLED_WIDTH, tbl_h, 1)
        d.vline(mid_x, tbl_y, tbl_h, 1)

        # Left: СНЯТО (inverted) full height
        d.fill_rect(1, tbl_y + 1, mid_x - 1, tbl_h - 2, 1)
        ds = t("disarmed")
        dsx = (mid_x - len(ds) * 8) // 2
        self._text(ds[:7], dsx, tbl_y + (tbl_h - 8) // 2, 0)

        # Right: lock icon centered
        self._draw_lock_icon(mid_x + (OLED_WIDTH - mid_x - 9) // 2,
                             tbl_y + (tbl_h - 14) // 2,
                             locked=False, color=1)

        # Bottom: centered
        bat_str = f"{t('bat')}:{bat_pct}%{'+'if charging else ''} {t('pkt')}:{pkt}"
        bx = (OLED_WIDTH - len(bat_str) * 8) // 2
        self._text(bat_str, bx, 55, 1)
        d.show()

    def show_rx_idle(self, bat_pct, charging, rx_cnt, lost_cnt, remote_bat=None,
                     locked=None, date_str="", time_str="", hb_str=""):
        """RX idle — waiting for packets."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header(t("you"), bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=locked)
        if date_str:
            dx = (OLED_WIDTH - len(date_str[:16]) * 8) // 2
            self._text(date_str[:16], dx, 16, 1)
        if time_str:
            tx = (OLED_WIDTH - len(time_str[:16]) * 8) // 2
            self._text(time_str[:16], tx, 26, 1)
        # Heartbeat box: stripe bg + border + outlined text
        box_y = 35
        box_h = 16
        # Light gray bg: every 3rd pixel lit (diagonal pattern)
        for py in range(box_y + 1, box_y + box_h - 1):
            for px in range(1, OLED_WIDTH - 1):
                if (px + py) % 3 == 0:
                    self.display.pixel(px, py, 1)
        # Border
        self.display.rect(0, box_y, OLED_WIDTH, box_h, 1)
        # White outline + bold black text
        ty = box_y + 4
        if hb_str:
            tx = 16
            # White outline: draw text+heart shifted in 8 directions
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if dx or dy:
                        self._draw_heart_icon(4+dx, ty+dy, 1)
                        self._text(hb_str, tx+dx, ty+dy, 1)
            # Black bold on top
            self._draw_heart_icon(4, ty, 0)
            self._draw_heart_icon(5, ty, 0)
            self._text(hb_str, tx, ty, 0)
            self._text(hb_str, tx+1, ty, 0)
        else:
            nl = t("no_link")
            # Center accounting for bold (+1px width)
            tw = len(nl) * 8 + 1
            nx = (OLED_WIDTH - tw) // 2
            # White outline: same as heartbeat
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if dx or dy:
                        self._text(nl, nx+dx, ty+dy, 1)
                        self._text(nl, nx+dx+1, ty+dy, 1)
            # Black bold centered
            self._text(nl, nx, ty, 0)
            self._text(nl, nx+1, ty, 0)
        self._text(f"{t('rx_lbl')}:{rx_cnt} {t('lost_lbl')}:{lost_cnt}", 2, 55, 1)
        self.display.show()

    def show_rx_alarm(self, signal_str, car_pct, car_charging,
                      my_pct, my_charging, radar_str, stats_str, is_sos=False):
        """RX alarm screen with signal info."""
        if not self.display:
            return
        self.display.fill(0)
        title = "SOS!" if is_sos else "ALARM!"
        self.draw_header(title, bat1_pct=my_pct, bat2_pct=car_pct,
)
        # Alert icon
        self._draw_alert_icon(2, 16, color=1)
        self.display.text(signal_str[:13], 14, 16, 1)
        self.display.text(radar_str[:16], 2, 28, 1)
        self.display.text(stats_str[:16], 2, 40, 1)
        # Car battery detail
        car_str = f"{t('car')}:{car_pct}%{'+'if car_charging else ''}"
        self.display.text(car_str, 2, 52, 1)
        self.display.show()

    def show_rx_box(self, title, arm_str, radar_str, signal_str, distance,
                    stats_str, bat1=None, bat2=None, locked=None):
        """Table layout: left = arm+radar, right = distance large."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        self.draw_header(title, bat1_pct=bat1, bat2_pct=bat2, locked=locked)

        # Table area: y=14..51
        tbl_y = 14
        tbl_h = 38
        mid_x = 56  # vertical divider

        # Outer table border
        d.rect(0, tbl_y, OLED_WIDTH, tbl_h, 1)
        # Vertical divider
        d.vline(mid_x, tbl_y, tbl_h, 1)

        # Left top cell: arm_str (inverted bg, centered)
        cell_h = tbl_h // 2
        d.fill_rect(1, tbl_y + 1, mid_x - 1, cell_h - 1, 1)
        ax = (mid_x - len(arm_str[:7]) * 8) // 2
        self._text(arm_str[:7], ax, tbl_y + 5, 0)
        # Horizontal divider
        d.hline(0, tbl_y + cell_h, mid_x + 1, 1)
        # Left bottom cell: radar_str (centered)
        rx = (mid_x - len(radar_str[:7]) * 8) // 2
        self._text(radar_str[:7], rx, tbl_y + cell_h + 5, 1)

        # Right cell: distance large, centered vertically
        right_w = OLED_WIDTH - mid_x - 1
        dist_w = len(distance) * 16
        dist_x = mid_x + 1 + (right_w - dist_w) // 2
        dist_y = tbl_y + (tbl_h - 16) // 2
        self._draw_large_text(dist_x, dist_y, distance, color=1, scale=2)

        # Bottom line: signal + stats
        d.text(signal_str[:8], 2, 55, 1)
        self._text(stats_str[:8], 66, 55, 1)
        d.show()

    def show_brief_status(self, mode, bat_pct, charging, armed, time_str="",
                          remote_bat=None):
        """Brief status screen for button wake (TX shows status then sleeps)."""
        if not self.display:
            return
        self.display.fill(0)
        self.draw_header(mode, bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=armed)
        self._draw_lock_icon(52, 18, locked=armed, color=1)
        state = t("armed") if armed else t("disarmed")
        sx = (OLED_WIDTH - len(state) * 8) // 2
        self._text(state, sx, 34, 1)
        if time_str:
            tx = (OLED_WIDTH - len(time_str) * 8) // 2
            self.display.text(time_str, tx, 46, 1)
        bat_str = f"{t('bat')}:{bat_pct}%{'+'if charging else ''}"
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
        self._text(t("sos_sent"), 28, 30, 1)
        self._text(t("signal_sent"), 20, 44, 1)
        self.display.show()
