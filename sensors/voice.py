"""DF2301QG offline voice recognition sensor driver (I2C, address 0x64).

The sensor sits behind the TCA9548A mux — select the correct mux channel
before calling any method, then deselect when done.

Usage:
    source venv4sipserve/bin/activate
    python sensors/voice.py

Key command IDs (returned by get_command_id()):
    0   — no command / idle
    1   — wake word detected
    See DF2301QG datasheet for the full command ID table.
"""

import smbus2
import time

MUX_ADDR = 0x70
VOICE_ADDR = 0x64

# Registers
_REG_COMMAND_ID = 0x02   # read: last recognized command ID (1 byte)
_REG_MUTE       = 0x03   # write: 0x00 = unmute, 0x01 = mute
_REG_VOLUME     = 0x04   # write: 0–7

# How long to wait after issuing a write command (seconds)
_WRITE_DELAY = 0.05


class VoiceSensor:
    def __init__(self, bus: smbus2.SMBus, mux_channel: int,
                 mux_addr: int = MUX_ADDR, addr: int = VOICE_ADDR):
        """
        Args:
            bus:         open SMBus instance (bus 1)
            mux_channel: TCA9548A channel (0–7) the sensor is wired to
            mux_addr:    TCA9548A I2C address (default 0x70)
            addr:        DF2301QG I2C address (default 0x64)
        """
        self.bus = bus
        self.mux_channel = mux_channel
        self.mux_addr = mux_addr
        self.addr = addr

    # ------------------------------------------------------------------
    # Mux helpers
    # ------------------------------------------------------------------

    def _select(self):
        """Enable the mux channel this sensor is on."""
        self.bus.write_byte(self.mux_addr, 1 << self.mux_channel)

    def _deselect(self):
        """Disable all mux channels."""
        self.bus.write_byte(self.mux_addr, 0x00)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_command_id(self) -> int | None:
        """Return the last recognized voice command ID, or None on error.

        Returns 0 when the sensor is idle (no new command).
        Call this in a loop; the sensor clears the register after each read.
        """
        try:
            self._select()
            data = self.bus.read_i2c_block_data(self.addr, _REG_COMMAND_ID, 1)
            return data[0]
        except OSError:
            return None
        finally:
            self._deselect()

    def set_mute(self, mute: bool):
        """Mute (True) or unmute (False) the onboard speaker."""
        try:
            self._select()
            self.bus.write_byte_data(self.addr, _REG_MUTE, 0x01 if mute else 0x00)
            time.sleep(_WRITE_DELAY)
        except OSError:
            pass
        finally:
            self._deselect()

    def set_volume(self, level: int):
        """Set speaker volume. level must be 0–7."""
        level = max(0, min(7, level))
        try:
            self._select()
            self.bus.write_byte_data(self.addr, _REG_VOLUME, level)
            time.sleep(_WRITE_DELAY)
        except OSError:
            pass
        finally:
            self._deselect()


# ------------------------------------------------------------------
# Smoke test — run directly to verify the sensor responds
# ------------------------------------------------------------------

if __name__ == "__main__":
    MUX_CHANNEL = 5  # change to whichever channel you wired the sensor to

    with smbus2.SMBus(1) as bus:
        sensor = VoiceSensor(bus, mux_channel=MUX_CHANNEL)
        sensor.set_volume(4)
        print(f"VoiceSensor on mux channel {MUX_CHANNEL} — listening (Ctrl+C to quit)")
        try:
            while True:
                cmd = sensor.get_command_id()
                if cmd:
                    print(f"Command ID: {cmd}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nDone.")
