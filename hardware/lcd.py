"""DFRobot I2C LCD Module v1.2 driver (PCF8574 backpack, address 0x20).

20x4 character display showing robot status:
  Row 0: "SipServe" (static)
  Row 1: Current state (dynamic)
  Row 2: Subphase / detail (dynamic)
  Row 3: "Table: <numbers>" (dynamic)
"""

LCD_ADDR = 0x20
LCD_COLS = 20
LCD_ROWS = 4


class SipServeLCD:
    def __init__(self, bus_number=1, address=LCD_ADDR, cols=LCD_COLS, rows=LCD_ROWS):
        self._available = False
        self._last_state = None
        self._last_subphase = None
        self._last_queue = None
        self._cols = cols
        try:
            from RPLCD.i2c import CharLCD
            self._lcd = CharLCD(
                i2c_expander="PCF8574",
                address=address,
                port=bus_number,
                cols=cols,
                rows=rows,
            )
            self._available = True
            self._lcd.clear()
            self._write_row(0, "SipServe")
        except OSError as e:
            print(f"LCD init failed ({e}) — running without display")

    def _write_row(self, row, text):
        if not self._available:
            return
        try:
            padded = text.center(self._cols)[:self._cols]
            self._lcd.cursor_pos = (row, 0)
            self._lcd.write_string(padded)
        except OSError:
            self._available = False
            print("LCD disconnected — disabling display")

    def update_state(self, state_name, target=None):
        display = f"{state_name} {target}" if target else state_name
        if display == self._last_state:
            return
        self._last_state = display
        self._write_row(1, display)

    def update_subphase(self, subphase: str | None):
        """Row 2 — current sub-phase / detail. Pass empty string or None to clear."""
        text = subphase or ""
        if text == self._last_subphase:
            return
        self._last_subphase = text
        self._write_row(2, text)

    def update_queue(self, table_numbers):
        key = tuple(table_numbers)
        if key == self._last_queue:
            return
        self._last_queue = key
        nums = " ".join(str(n) for n in table_numbers)
        self._write_row(3, f"Table: {nums}" if nums else "Table:")

    def clear(self):
        if not self._available:
            return
        try:
            self._lcd.clear()
            self._lcd.backlight_enabled = False
        except OSError:
            pass
