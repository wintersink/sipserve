"""
Obstacle avoidance test.

Drives forward at 5% duty cycle and steers around obstacles.

Behavior:
  - front obstacle        → stop, reverse briefly, turn away from closest side
  - front-left obstacle   → turn right
  - front-right obstacle  → turn left
  - left only             → slow down (stay course)
  - right only            → slow down (stay course)
  - clear                 → full speed forward

Sensors: left, front_left, front, front_right, right (via TCA9548A mux).
Obstacle threshold: 1000 mm.
Ctrl+C to quit.
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import IIC
import smbus2
from hardware.mux import Mux, SENSOR_CHANNELS
from sensors.ultrasonic import UltrasonicSensor

SPEED = 720            # ~10% duty cycle (PWM range 0–7200)
SLOW_SPEED = 180       # ~2.5% duty cycle — used when side sensors trigger
TURN_SPEED = 360       # speed for turning away from obstacles
REVERSE_TIME = 0.5     # seconds to reverse when front is blocked
TURN_TIME = 0.5        # seconds to turn when front is blocked
OBSTACLE_MM = 203      # detection threshold in mm
POLL_INTERVAL = 0.05   # seconds between sensor sweeps

bus = smbus2.SMBus(1)
mux = Mux(bus)
sensor = UltrasonicSensor(bus)


def stop_motors():
    IIC.control_pwm(0, 0, 0, 0)


def drive_forward(speed=SPEED):
    IIC.control_pwm(-speed, 0, -speed, 0)


def reverse():
    IIC.control_pwm(SPEED, 0, SPEED, 0)


def turn_left():
    IIC.control_pwm(-TURN_SPEED, 0, TURN_SPEED, 0)


def turn_right():
    IIC.control_pwm(TURN_SPEED, 0, -TURN_SPEED, 0)


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
    IIC.set_motor_parameter()
    stop_motors()

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

            if front:
                # Front blocked — stop, reverse, then turn away from closest side
                print_readings(readings, "FRONT BLOCKED")
                stop_motors()
                reverse()
                time.sleep(REVERSE_TIME)
                stop_motors()
                # Turn away from whichever side has the closer obstacle
                fl = readings.get("front_left") or 9999
                fr = readings.get("front_right") or 9999
                if fl <= fr:
                    turn_right()
                else:
                    turn_left()
                time.sleep(TURN_TIME)
                stop_motors()

            elif front_left and front_right:
                # Both front sides blocked — reverse and turn
                print_readings(readings, "BOTH SIDES")
                stop_motors()
                reverse()
                time.sleep(REVERSE_TIME)
                stop_motors()
                fl = readings.get("front_left") or 9999
                fr = readings.get("front_right") or 9999
                if fl <= fr:
                    turn_right()
                else:
                    turn_left()
                time.sleep(TURN_TIME)
                stop_motors()

            elif front_left:
                # Obstacle on front-left — steer right
                print_readings(readings, "TURN RIGHT")
                turn_right()
                time.sleep(POLL_INTERVAL)

            elif front_right:
                # Obstacle on front-right — steer left
                print_readings(readings, "TURN LEFT")
                turn_left()
                time.sleep(POLL_INTERVAL)

            elif left or right:
                # Side sensor triggered — slow down but keep going
                print_readings(readings, "SLOW")
                drive_forward(SLOW_SPEED)
                time.sleep(POLL_INTERVAL)

            else:
                # All clear — full speed
                print_readings(readings, "FORWARD")
                drive_forward()
                time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        mux.disable()
        print("\nMotors stopped. Mux disabled.")
