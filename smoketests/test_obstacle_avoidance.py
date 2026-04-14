"""
Obstacle avoidance test.

Drives forward at 10% duty cycle and steers around obstacles.

Behavior:
  - front obstacle        → stop, reverse briefly, turn away from closer side
  - front-left obstacle   → turn right
  - front-right obstacle  → turn left
  - left only             → slow down (stay course)
  - right only            → slow down (stay course)
  - clear                 → full speed forward

Sensors: left, front_left, front, front_right, right (via TCA9548A mux).
Ctrl+C to quit.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import smbus2
from hardware.motor import Motor, FULL_SPEED, SLOW_SPEED, TURN_SPEED
from hardware.mux import Mux, SENSOR_CHANNELS
from sensors.ultrasonic import UltrasonicSensor

REVERSE_TIME  = 0.5      # seconds to reverse when front is blocked
TURN_TIME     = 0.5      # seconds to turn when front is blocked
OBSTACLE_MM   = 800      # detection threshold in mm
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


def print_readings(readings, state):
    parts = []
    for name, dist in readings.items():
        if dist is None:
            parts.append(f"{name}: ERR")
        else:
            parts.append(f"{name}: {dist}mm")
    print(f"[{state}]  " + "  ".join(parts))


def is_triggered(dist):
    """Return True if a sensor reading counts as an obstacle."""
    return dist is not None and dist < OBSTACLE_MM


if __name__ == "__main__":
    print("Initialising motor parameters...")
    motor.set_motor_parameter()
    motor.stop()

    print(f"Obstacle threshold: {OBSTACLE_MM} mm")
    print("Starting obstacle avoidance — Ctrl+C to quit\n")

    try:
        while True:
            readings = read_all_sensors()

            front       = is_triggered(readings.get("front"))
            front_left  = is_triggered(readings.get("front_left"))
            front_right = is_triggered(readings.get("front_right"))
            left        = is_triggered(readings.get("left"))
            right       = is_triggered(readings.get("right"))

            if front or (front_left and front_right):
                # Front blocked — stop, reverse, then turn away from closer side
                state = "FRONT BLOCKED" if front else "BOTH SIDES"
                print_readings(readings, state)
                motor.stop()
                motor.reverse()
                time.sleep(REVERSE_TIME)
                motor.stop()
                fl = readings.get("front_left")  or 9999
                fr = readings.get("front_right") or 9999
                if fl <= fr:
                    motor.turn_right()
                else:
                    motor.turn_left()
                time.sleep(TURN_TIME)
                motor.stop()

            elif front_left:
                print_readings(readings, "TURN RIGHT")
                motor.turn_right()
                time.sleep(POLL_INTERVAL)

            elif front_right:
                print_readings(readings, "TURN LEFT")
                motor.turn_left()
                time.sleep(POLL_INTERVAL)

            elif left or right:
                print_readings(readings, "SLOW")
                motor.forward(SLOW_SPEED)
                time.sleep(POLL_INTERVAL)

            else:
                print_readings(readings, "FORWARD")
                motor.forward(FULL_SPEED)
                time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        mux.disable()
        print("\nMotors stopped. Mux disabled.")
