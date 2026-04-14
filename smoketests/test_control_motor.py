"""
Interactive motor control test.

Commands:
  f  — forward for 2 seconds
  b  — reverse for 2 seconds
  l  — turn left for 2 seconds
  r  — turn right for 2 seconds
  s  — stop motors
  q  — quit

Motors: M1 = right tread, M3 = left tread (M2/M4 unused).
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import smbus2
from hardware.motor import Motor, FULL_SPEED

DURATION = 2.0    # seconds each command runs

bus = smbus2.SMBus(1)
motor = Motor(bus)


def stop():
    print("Stopping...")
    motor.stop()


def forward():
    print("Forward...")
    motor.forward()
    time.sleep(DURATION)
    stop()


def reverse():
    print("Reverse...")
    motor.reverse()
    time.sleep(DURATION)
    stop()


def turn_left():
    print("Turning left...")
    motor.turn_left()
    time.sleep(DURATION)
    stop()


def turn_right():
    print("Turning right...")
    motor.turn_right()
    time.sleep(DURATION)
    stop()


COMMANDS = {
    'f': forward,
    'b': reverse,
    'l': turn_left,
    'r': turn_right,
    's': stop,
}

if __name__ == "__main__":
    print("Initialising motor parameters...")
    motor.set_motor_parameter()
    motor.stop()
    print("Ready.  Commands: f=forward  b=reverse  l=left  r=right  s=stop  q=quit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == 'q':
                break
            action = COMMANDS.get(cmd)
            if action:
                action()
                print("Done.")
            else:
                print("Unknown command. Use f / b / l / r / s / q")
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        print("\nMotors stopped.")
