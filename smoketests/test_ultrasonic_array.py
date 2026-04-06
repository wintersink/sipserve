"""
Ultrasonic sensor array test.

Polls all 5 ultrasonic sensors through the TCA9548A multiplexer every 2 seconds
and prints which sensors detect an obstacle (< 305 mm / 12 in).

Sensor layout (numbered 1–5, left to right):
  1 = left        (mux channel 0)
  2 = front-left  (mux channel 1)
  3 = front       (mux channel 2)
  4 = front-right (mux channel 3)
  5 = right       (mux channel 7)
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import smbus2
from sensors.ultrasonic import UltrasonicArray, OBSTACLE_MM

POLL_INTERVAL = 2.0     # seconds between sweeps

bus = smbus2.SMBus(1)
array = UltrasonicArray(bus)


if __name__ == "__main__":
    print("Ultrasonic array test — polling every 2s")
    print(f"Obstacle threshold: {OBSTACLE_MM} mm ({OBSTACLE_MM / 25.4:.1f} in)")
    print("Ctrl+C to quit\n")

    try:
        while True:
            results = array.poll()

            # Print all readings
            parts = []
            for num, name, dist in results:
                if dist is None:
                    parts.append(f"[{num}] {name}: ERR")
                else:
                    parts.append(f"[{num}] {name}: {dist}mm")
            print("  ".join(parts))

            # Report obstacles
            obstacles = [(num, name, dist) for num, name, dist in results
                         if dist is not None and dist < OBSTACLE_MM]
            if obstacles:
                tags = [f"[{num}] {name} ({dist}mm)" for num, name, dist in obstacles]
                print(f"  >> OBSTACLE: {', '.join(tags)}")

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        array.close()
        print("\nDone.")
