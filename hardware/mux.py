"""TCA9548A I2C multiplexer driver (address 0x70)."""

import smbus2

MUX_ADDR = 0x70

# Sensor channel assignments
SENSOR_CHANNELS = {
    "left":        0,
    "front_left":  1,
    "front":       2,
    "front_right": 3,
    "right":       7,
}


class Mux:
    def __init__(self, bus: smbus2.SMBus, addr: int = MUX_ADDR):
        self.bus = bus
        self.addr = addr
        self._active = None

    def select(self, channel: int):
        """Enable a single channel (0-7). Disables all others."""
        if channel < 0 or channel > 7:
            raise ValueError(f"Channel must be 0-7, got {channel}")
        if self._active != channel:
            self.bus.write_byte(self.addr, 1 << channel)
            self._active = channel

    def disable(self):
        """Disable all channels."""
        self.bus.write_byte(self.addr, 0x00)
        self._active = None

    def invalidate(self):
        """Force the next select() to re-send even if the channel matches cache.

        Needed when another caller may have written the mux register while we
        weren't holding the I2C lock — our cached `_active` would then be wrong.
        """
        self._active = None
