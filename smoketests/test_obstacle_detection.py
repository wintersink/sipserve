"""
Obstacle detection test.

Drives forward at 5% duty cycle continuously. Stops if any sensor detects an
obstacle within 8 inches (203 mm), resumes once the path is clear.

Sensors: left, front_left, front, front_right, right (via TCA9548A mux).
Ctrl+C to quit.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import smbus2
from hardware.motor import Motor, SLOW_SPEED
from hardware.mux import Mux, SENSOR_CHANNELS
from sensors.ultrasonic import UltrasonicSensor

OBSTACLE_MM   = 203      # 8 inches in mm (203 mm) with some margin
POLL_INTERVAL = 0.05     # seconds between sensor sweeps

bus    = smbus2.SMBus(1)
motor  = Motor(bus)
mux    = Mux(bus)
sensor = UltrasonicSensor(bus)


def read_all_sensors():
    """Read all 5 ultrasonic sensors. Returns dict of name -> distance_mm (None on error)."""
    readings = {}
    for name, channel in SENSOR_CHANNELS.items():
        mux.select(channel)
        readings[name] = sensor.read_mm()
    mux.disable()
    return readings


def print_readings(readings):
    parts = []
    for name, dist in readings.items():
        if dist is None:
            parts.append(f"{name}: ERR")
        else:
            parts.append(f"{name}: {dist}mm")
    print("  ".join(parts))


def check_obstacle(readings):
    """Return list of sensor names that detect an obstacle within threshold."""
    return [name for name, dist in readings.items()
            if dist is not None and dist < OBSTACLE_MM]


if __name__ == "__main__":
    print("Initialising motor parameters...")
    motor.set_motor_parameter()
    motor.stop()

    print(f"Obstacle threshold: {OBSTACLE_MM} mm ({OBSTACLE_MM / 25.4:.1f} in)")
    print("Driving forward — Ctrl+C to quit\n")

    driving = True
    motor.forward(SLOW_SPEED)

    try:
        while True:
            readings = read_all_sensors()
            print_readings(readings)

            triggered = check_obstacle(readings)

            if driving and triggered:
                motor.stop()
                driving = False
                print(f"  >> OBSTACLE: {', '.join(triggered)} — stopped!")
            elif not driving and not triggered:
                motor.forward(SLOW_SPEED)
                driving = True
                print("  >> Clear — resuming forward.")

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        mux.disable()
        print("\nMotors stopped. Mux disabled.")
