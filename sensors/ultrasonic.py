"""TK50 I2C ultrasonic sensor driver (address 0x57).

Each sensor shares address 0x57; the TCA9548A mux selects which one to read.
Protocol: write 0x01 to trigger, wait 150ms, then read 3 bytes.
Distance (µm) = (byte[0] << 16) | (byte[1] << 8) | byte[2]
"""

import smbus2
import time

from hardware.mux import Mux

SENSOR_ADDR = 0x57
_READ_LEN = 3

# Sensor layout: (number, name, mux channel) ordered 1–5 left to right
SENSOR_LAYOUT = [
    (1, "left",        0),
    (2, "front-left",  1),
    (3, "front",       2),
    (4, "front-right", 3),
    (5, "right",       7),
]

OBSTACLE_MM = 305  # 12 inches


class UltrasonicSensor:
    def __init__(self, bus: smbus2.SMBus, addr: int = SENSOR_ADDR):
        self.bus = bus
        self.addr = addr

    def read_mm(self, settle: float = 0.01) -> int | None:
        """
        Returns distance in mm, or None on read error or out-of-range.
        Valid range: ~20 mm – 4500 mm.
        settle: seconds to wait before triggering (mux switching delay).
        """
        try:
            time.sleep(settle)                    # let mux channel settle
            self.bus.write_byte(self.addr, 0x01)  # trigger measurement
            time.sleep(0.15)                      # TK50 needs ~150ms
            msg = smbus2.i2c_msg.read(self.addr, _READ_LEN)
            self.bus.i2c_rdwr(msg)
            data = list(msg)
            um = (data[0] << 16) | (data[1] << 8) | data[2]
            dist = um // 1000                     # micrometers → millimeters
            if dist < 20 or dist > 4500:
                return None
            return dist
        except OSError:
            return None

    def trigger(self, settle: float = 0.01):
        """Trigger a measurement. Call read_result() after 150ms."""
        try:
            time.sleep(settle)
            self.bus.write_byte(self.addr, 0x01)
        except OSError:
            pass

    def read_result(self, settle: float = 0.01) -> int | None:
        """Read the result of a previously triggered measurement."""
        try:
            time.sleep(settle)
            msg = smbus2.i2c_msg.read(self.addr, _READ_LEN)
            self.bus.i2c_rdwr(msg)
            data = list(msg)
            um = (data[0] << 16) | (data[1] << 8) | data[2]
            dist = um // 1000
            if dist < 20 or dist > 4500:
                return None
            return dist
        except OSError:
            return None


class UltrasonicArray:
    """Manages all 5 ultrasonic sensors through the TCA9548A mux."""

    def __init__(self, bus: smbus2.SMBus):
        self.mux = Mux(bus)
        self.sensor = UltrasonicSensor(bus)

    def poll(self) -> list[tuple[int, str, int | None]]:
        """Read all sensors. Returns list of (num, name, distance_mm)."""
        results = []
        for num, name, channel in SENSOR_LAYOUT:
            self.mux.select(channel)
            dist = self.sensor.read_mm()
            results.append((num, name, dist))
        self.mux.disable()
        return results

    def close(self):
        self.mux.disable()
