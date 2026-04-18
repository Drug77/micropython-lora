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

    def _draw_person_icon(self, x, y, color=0):
        """Person icon 7x9 for header."""
        d = self.display
        # Head (round)
        d.hline(x + 2, y, 3, color)
        d.hline(x + 1, y + 1, 5, color)
        d.hline(x + 2, y + 2, 3, color)
        # Body
        d.vline(x + 3, y + 3, 3, color)
        # Arms
        d.hline(x, y + 4, 7, color)
        # Legs
        d.pixel(x + 1, y + 7, color)
        d.pixel(x + 2, y + 6, color)
        d.pixel(x + 4, y + 6, color)
        d.pixel(x + 5, y + 7, color)

    def _draw_car_icon(self, x, y, color=0):
        """Car icon 11x7 for header."""
        d = self.display
        # Roof
        d.hline(x + 2, y, 7, color)
        # Windshield + body
        d.hline(x + 1, y + 1, 9, color)
        d.hline(x, y + 2, 11, color)
        d.hline(x, y + 3, 11, color)
        d.hline(x, y + 4, 11, color)
        # Wheels
        d.fill_rect(x + 1, y + 5, 3, 2, color)
        d.fill_rect(x + 7, y + 5, 3, 2, color)

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
        """Render text at scale*8 px per character. Supports ASCII + Cyrillic."""
        if not self.display:
            return
        char_buf_arr = bytearray(8)
        fb = framebuf.FrameBuffer(char_buf_arr, 8, 8, framebuf.MONO_VLSB)
        for i, ch in enumerate(text):
            data = char_data(ch)
            if data is not None:
                # Cyrillic: row format, MSB=leftmost
                for py in range(8):
                    byte = data[py]
                    for px in range(8):
                        if byte & (0x80 >> px):
                            self.display.fill_rect(
                                x + i * 8 * scale + px * scale,
                                y + py * scale,
                                scale, scale, color)
            else:
                # ASCII: built-in framebuf
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
        """Draw inverted header bar with person/car icons at batteries.

        Layout (128x13px, white background):
        [TITLE]  [wifi] [icon1 bat1 XX] [icon2 bat2 XX]
        """
        if not self.display:
            return
        h_height = 13
        self.display.fill_rect(0, 0, OLED_WIDTH, h_height, 1)
        if title == "\x01":
            # Heart icon instead of text
            self._draw_heart_icon(3, 3, 0)
        else:
            self._text(title[:8], 2, 3, 0)

        # Determine which icon goes with which battery
        # bat2 = remote device, bat1 = local
        is_car = (title == t("car"))

        rx = 126  # Right edge cursor (build icons right-to-left)

        # Battery 2 (remote device) + icon
        if bat2_pct is not None:
            pct_str = str(bat2_pct)
            tw = len(pct_str) * 8
            rx -= tw
            self.display.text(pct_str, rx, 3, 0)
            rx -= 13
            self._draw_mini_battery(rx, 3, bat2_pct, color=0)
            rx -= 1
            if is_car:
                rx -= 8
                self._draw_person_icon(rx, 2, color=0)
            else:
                rx -= 12
                self._draw_car_icon(rx, 3, color=0)
            rx -= 1

        # Battery 1 (local device) + icon
        if bat1_pct is not None:
            pct_str = str(bat1_pct)
            tw = len(pct_str) * 8
            rx -= tw
            self.display.text(pct_str, rx, 3, 0)
            rx -= 13
            self._draw_mini_battery(rx, 3, bat1_pct, color=0)
            rx -= 1
            if is_car:
                rx -= 12
                self._draw_car_icon(rx, 3, color=0)
            else:
                rx -= 8
                self._draw_person_icon(rx, 2, color=0)
            rx -= 1

        # WiFi icon
        if wifi:
            rx -= 8
            self._draw_wifi_icon(rx, 3, color=0)
            rx -= 2

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

    def show_send_progress(self, title, status, percent, toa_str,
                           bat1=None, bat2=None, locked=None, header=None):
        """Sending screen: table layout with prominent progress bar."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        h = header if header is not None else t("you")
        self.draw_header(h, bat1_pct=bat1, bat2_pct=bat2, locked=locked)

        tbl_y = 14
        top_h = 20
        mid_x = 56

        # Outer border
        d.rect(0, tbl_y, OLED_WIDTH, 50, 1)

        # Top section: left inverted title, right status+toa
        d.vline(mid_x, tbl_y, top_h + 1, 1)
        d.hline(0, tbl_y + top_h, OLED_WIDTH, 1)

        # Left: title inverted centered
        d.fill_rect(1, tbl_y + 1, mid_x - 1, top_h - 1, 1)
        tx = (mid_x - len(title) * 8) // 2
        self._text(title[:7], max(tx, 2), tbl_y + 6, 0)

        # Right: status + toa
        self._text(status[:8], mid_x + 3, tbl_y + 2, 1)
        self._text(toa_str[:8], mid_x + 3, tbl_y + 12, 1)

        # Bottom: progress bar
        self._draw_send_bar(d, percent, "")
        d.show()

    def update_send_bar(self, percent, elapsed_str):
        """Quick update: redraw only progress section."""
        if not self.display:
            return
        d = self.display
        d.fill_rect(1, 35, OLED_WIDTH - 2, 27, 0)
        self._draw_send_bar(d, percent, elapsed_str)
        d.show()

    def _draw_send_bar(self, d, percent, elapsed_str):
        """Draw progress bar section for send screen."""
        label_y = 36
        bar_y = 46
        bar_h = 16
        bar_x = 4
        bar_w = OLED_WIDTH - 8
        if elapsed_str:
            self._text(elapsed_str, 4, label_y, 1)
        pct_str = f"{percent}%"
        d.text(pct_str, OLED_WIDTH - len(pct_str) * 8 - 4, label_y, 1)
        d.rect(bar_x, bar_y, bar_w, bar_h, 1)
        fill_w = int((bar_w - 4) * (percent / 100.0))
        if fill_w > 0:
            d.fill_rect(bar_x + 2, bar_y + 2, fill_w, bar_h - 4, 1)

    def show_tx_armed(self, radar_state, distance, bat_pct, charging,
                      idle_s, idle_max, pkt, remote_bat=None,
                      hb_pct=0):
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

        # Bottom: heartbeat progress bar
        bar_x, bar_y, bar_w, bar_h = 2, 54, OLED_WIDTH - 4, 8
        d.rect(bar_x, bar_y, bar_w, bar_h, 1)
        fill_w = int((bar_w - 4) * (hb_pct / 100.0))
        if fill_w > 0:
            d.fill_rect(bar_x + 2, bar_y + 2, fill_w, bar_h - 4, 1)
        d.show()

    def show_tx_disarmed(self, bat_pct, charging, pkt, remote_bat=None,
                         hb_pct=0, date_str="", time_str="",
                         rx_cnt=0, lost_cnt=0):
        """TX disarmed — full table layout."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        self.draw_header(t("car"), bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=False)

        tbl_y = 14
        mid_x = 56

        # Outer border
        d.rect(0, tbl_y, OLED_WIDTH, 50, 1)

        # ── Row 1 (18px): СНЯТО | date+time ──
        row1_h = 18
        d.vline(mid_x, tbl_y, row1_h + 1, 1)
        d.hline(0, tbl_y + row1_h, OLED_WIDTH, 1)

        # Left: СНЯТО inverted
        d.fill_rect(1, tbl_y + 1, mid_x - 1, row1_h - 1, 1)
        ds = t("disarmed")
        dsx = (mid_x - len(ds) * 8) // 2
        self._text(ds[:7], max(dsx, 2), tbl_y + 5, 0)

        # Right: date + time (or dashes)
        dt = date_str[:9] if date_str else "--.--.--"
        tm = time_str[:8] if time_str else "--:--:--"
        self._text(dt, mid_x + 3, tbl_y + 1, 1)
        self._text(tm, mid_x + 3, tbl_y + 10, 1)

        # ── Row 2 (12px): pkt | rx/lost ──
        row2_y = tbl_y + row1_h
        row2_h = 12
        d.vline(mid_x, row2_y, row2_h + 1, 1)
        d.hline(0, row2_y + row2_h, OLED_WIDTH, 1)

        self._text(f"{t('pkt')}:{pkt}", 3, row2_y + 2, 1)
        self._text(f"{t('rx_lbl')}:{rx_cnt} {t('lost_lbl')}:{lost_cnt}", mid_x + 3, row2_y + 2, 1)

        # ── Row 3 (18px): heartbeat progress bar (100→0) ──
        bar_x, bar_y = 4, tbl_y + row1_h + row2_h + 4
        bar_w, bar_h = OLED_WIDTH - 8, 10
        d.rect(bar_x, bar_y, bar_w, bar_h, 1)
        remaining = max(0, 100 - hb_pct)
        fill_w = int((bar_w - 4) * (remaining / 100.0))
        if fill_w > 0:
            d.fill_rect(bar_x + 2, bar_y + 2, fill_w, bar_h - 4, 1)
        d.show()

    def show_rx_idle(self, bat_pct, charging, rx_cnt, lost_cnt, remote_bat=None,
                     locked=None, date_str="", time_str="", hb_str="",
                     hb_pct=0):
        """RX idle — table layout: state|datetime, heart|stats, progress bar."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        self.draw_header(t("you"), bat1_pct=bat_pct, bat2_pct=remote_bat,
                         locked=locked)

        tbl_y = 14
        mid_x = 56
        row1_h = 18
        row2_h = 12

        # Outer border
        d.rect(0, tbl_y, OLED_WIDTH, 50, 1)

        # ── Row 1: Armed state | Date+Time (18px) ──
        d.vline(mid_x, tbl_y, row1_h + 1, 1)
        d.hline(0, tbl_y + row1_h, OLED_WIDTH, 1)

        if locked == "error":
            # Error: inverted left cell
            d.fill_rect(1, tbl_y + 1, mid_x - 1, row1_h - 1, 1)
            er = t("no_rdr")
            ex = (mid_x - len(er) * 8) // 2
            self._text(er, max(ex, 2), tbl_y + 5, 0)
        elif locked is True:
            # Armed: inverted left cell
            d.fill_rect(1, tbl_y + 1, mid_x - 1, row1_h - 1, 1)
            arm = t("armed")
            ax = (mid_x - len(arm) * 8) // 2
            self._text(arm, max(ax, 2), tbl_y + 5, 0)
        elif locked is False:
            # Disarmed
            ds = t("disarmed")
            dsx = (mid_x - len(ds) * 8) // 2
            self._text(ds, max(dsx, 2), tbl_y + 5, 1)
        else:
            # Offline: inverted
            d.fill_rect(1, tbl_y + 1, mid_x - 1, row1_h - 1, 1)
            ol = t("offline_box")
            ox = (mid_x - len(ol) * 8) // 2
            self._text(ol, max(ox, 2), tbl_y + 5, 0)

        # Right: date + time
        if date_str:
            self._text(date_str[:9], mid_x + 3, tbl_y + 1, 1)
        if time_str:
            self._text(time_str[:8], mid_x + 3, tbl_y + 10, 1)

        # ── Row 2: Heart+countdown | Stats (12px) ──
        row2_y = tbl_y + row1_h
        d.vline(mid_x, row2_y, row2_h + 1, 1)
        d.hline(0, row2_y + row2_h, OLED_WIDTH, 1)

        if hb_str:
            self._draw_heart_icon(3, row2_y + 2, 1)
            self._text(hb_str[:5], 14, row2_y + 2, 1)
        else:
            self._text("--", 3, row2_y + 2, 1)

        # Stats right
        stats = f"{t('rx_lbl')}:{rx_cnt} {t('lost_lbl')}:{lost_cnt}"
        self._text(stats[:9], mid_x + 3, row2_y + 2, 1)

        # ── Row 3: Heartbeat progress bar (18px) ──
        bar_section_y = row2_y + row2_h
        # Dithered bg
        for py in range(bar_section_y + 1, 62):
            for px in range(1, OLED_WIDTH - 1):
                if (px + py) % 4 == 0:
                    d.pixel(px, py, 1)

        # Progress bar (full width, no percent label)
        pb_y = bar_section_y + 3
        pb_h = 12
        pb_x = 4
        pb_w = OLED_WIDTH - 8
        d.fill_rect(pb_x - 1, pb_y - 1, pb_w + 2, pb_h + 2, 0)
        d.rect(pb_x, pb_y, pb_w, pb_h, 1)
        remaining = max(0, 100 - hb_pct)
        fill_w = int((pb_w - 4) * (remaining / 100.0))
        if fill_w > 0:
            d.fill_rect(pb_x + 2, pb_y + 2, fill_w, pb_h - 4, 1)

        d.show()

    def show_rx_alarm(self, signal_str, car_pct, car_charging,
                      my_pct, my_charging, radar_str, stats_str,
                      is_sos=False, date_str="", time_str=""):
        """RX alarm/SOS screen — table layout."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        title = "SOS!" if is_sos else "ALARM!"
        self.draw_header(title, bat1_pct=my_pct, bat2_pct=car_pct)

        tbl_y = 14
        mid_x = 56

        if is_sos:
            # ── SOS: bell icon centered + info table below ──
            cx = 64
            # Bell icon (same as show_sos_sent)
            d.fill_rect(cx - 3, 15, 6, 3, 1)
            d.fill_rect(cx - 5, 18, 10, 2, 1)
            d.fill_rect(cx - 7, 20, 14, 4, 1)
            d.fill_rect(cx - 8, 24, 16, 2, 1)
            d.fill_rect(cx - 1, 27, 2, 2, 1)
            # Sound waves
            d.pixel(cx - 10, 21, 1); d.pixel(cx - 11, 22, 1)
            d.pixel(cx + 9, 21, 1); d.pixel(cx + 10, 22, 1)
            d.pixel(cx - 1, 14, 1); d.pixel(cx, 14, 1)

            # Table below bell: y=30..63
            tbl_y2 = 30
            d.rect(0, tbl_y2, OLED_WIDTH, 34, 1)

            # Row 1 (12px): SOS! inverted | datetime
            d.hline(0, tbl_y2 + 12, OLED_WIDTH, 1)
            mid = 36
            d.vline(mid, tbl_y2, 12, 1)
            d.fill_rect(1, tbl_y2 + 1, mid - 1, 10, 1)
            self._text("SOS!", 2, tbl_y2 + 2, 0)
            if date_str:
                self._text(date_str[:11], mid + 3, tbl_y2 + 2, 1)

            # Row 2 (10px): signal + stats full width
            r2y = tbl_y2 + 12
            d.hline(0, r2y + 10, OLED_WIDTH, 1)
            self._text(f"{signal_str} {stats_str}"[:15], 3, r2y + 2, 1)

            # Row 3 (10px): car battery — inverted for contrast
            r3y = r2y + 10
            d.fill_rect(1, r3y + 1, OLED_WIDTH - 2, 34 - 22 - 2, 1)
            car_s = f"{t('car')}:{car_pct}%"
            cx = (OLED_WIDTH - len(car_s) * 8) // 2
            self._text(car_s, cx, r3y + 2, 0)
        else:
            # ── ALARM: table with radar info ──
            d.rect(0, tbl_y, OLED_WIDTH, 50, 1)

            row1_h = 20
            d.vline(mid_x, tbl_y, row1_h + 1, 1)
            d.hline(0, tbl_y + row1_h, OLED_WIDTH, 1)

            # Left: inverted + alert icon
            d.fill_rect(1, tbl_y + 1, mid_x - 1, row1_h - 1, 1)
            self._draw_alert_icon((mid_x - 9) // 2, tbl_y + 2, 0)
            self._text("ALARM", 3, tbl_y + 11, 0)

            # Right: radar + distance
            self._text(radar_str[:9], mid_x + 3, tbl_y + 2, 1)
            d.text(signal_str[:9], mid_x + 3, tbl_y + 11, 1)

            # Row 2
            row2_y = tbl_y + row1_h
            d.hline(0, row2_y + 12, OLED_WIDTH, 1)
            self._text(stats_str[:16], 3, row2_y + 2, 1)

            # Bottom
            car_s = f"{t('car')}:{car_pct}%{'+'if car_charging else ''}"
            self._text(car_s, 3, row2_y + 16, 1)

        d.show()

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

    def show_sos_sent(self, bat_pct, remote_bat=None, text=None):
        """SOS sent/ack — bell icon centered + text below."""
        if not self.display:
            return
        d = self.display
        d.fill(0)
        self.draw_header("SOS", bat1_pct=bat_pct, bat2_pct=remote_bat)

        # Bell/alarm icon 3x scale, centered (y=16..40, ~24px tall)
        cx = 64  # center x
        # Bell dome (rounded top)
        d.fill_rect(cx - 3, 16, 6, 3, 1)
        d.fill_rect(cx - 6, 19, 12, 3, 1)
        d.fill_rect(cx - 8, 22, 16, 3, 1)
        d.fill_rect(cx - 9, 25, 18, 6, 1)
        # Bell rim (wider bottom)
        d.fill_rect(cx - 11, 31, 22, 3, 1)
        # Clapper (dot below)
        d.fill_rect(cx - 2, 35, 4, 3, 1)
        # Sound waves left
        d.pixel(cx - 13, 26, 1)
        d.pixel(cx - 14, 27, 1)
        d.pixel(cx - 14, 28, 1)
        d.pixel(cx - 13, 29, 1)
        # Sound waves right
        d.pixel(cx + 12, 26, 1)
        d.pixel(cx + 13, 27, 1)
        d.pixel(cx + 13, 28, 1)
        d.pixel(cx + 12, 29, 1)
        # Handle on top
        d.fill_rect(cx - 1, 14, 2, 3, 1)

        # Text centered below icon
        sos = text if text else t("sos_sent")
        sx = (OLED_WIDTH - len(sos) * 8) // 2
        self._text(sos, sx, 42, 1)
        self._text(sos, sx + 1, 42, 1)  # bold

        d.show()
