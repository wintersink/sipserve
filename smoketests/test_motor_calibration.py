"""
Time-based motor calibration.

Drives at FULL_SPEED / TURN_SPEED for a fixed duration and asks you to measure
the result. Computes mm/sec and deg/sec you should plug into hardware/motor.py.

Commands:
  d  — drive forward for 3 seconds at FULL_SPEED
  r  — rotate right for 2 seconds at TURN_SPEED
  q  — quit

Repeat each test a few times with different durations and average the results.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import smbus2
from hardware.motor import (
    Motor,
    FULL_SPEED,
    TURN_SPEED,
    MM_PER_SEC_AT_FULL,
    DEG_PER_SEC_AT_TURN,
)

DRIVE_DURATION_S  = 5.0
ROTATE_DURATION_S = 5.0

bus = smbus2.SMBus(1)
motor = Motor(bus)


def drive_test():
    print(f"\nDriving forward at FULL_SPEED ({FULL_SPEED}) for {DRIVE_DURATION_S}s...")
    print("Clear the path. Starting in 2s...")
    time.sleep(2)
    motor.forward(FULL_SPEED)
    time.sleep(DRIVE_DURATION_S)
    motor.stop()
    print("Stopped. Measure the distance travelled (mm).")
    raw = input("  Distance (mm): ").strip()
    try:
        mm = float(raw)
    except ValueError:
        print("  Skipped.")
        return
    mm_per_sec = mm / DRIVE_DURATION_S
    print(f"  → {mm_per_sec:.1f} mm/sec  (current constant: {MM_PER_SEC_AT_FULL})")
    print(f"  Set MM_PER_SEC_AT_FULL = {mm_per_sec:.1f} in hardware/motor.py")


def rotate_test():
    print(f"\nRotating right at TURN_SPEED ({TURN_SPEED}) for {ROTATE_DURATION_S}s...")
    print("Mark the starting heading. Starting in 2s...")
    time.sleep(2)
    motor.turn_right(TURN_SPEED)
    time.sleep(ROTATE_DURATION_S)
    motor.stop()
    print("Stopped. Estimate the angle rotated (degrees).")
    raw = input("  Angle (degrees): ").strip()
    try:
        deg = float(raw)
    except ValueError:
        print("  Skipped.")
        return
    deg_per_sec = deg / ROTATE_DURATION_S
    print(f"  → {deg_per_sec:.1f} deg/sec  (current constant: {DEG_PER_SEC_AT_TURN})")
    print(f"  Set DEG_PER_SEC_AT_TURN = {deg_per_sec:.1f} in hardware/motor.py")


COMMANDS = {
    'd': drive_test,
    'r': rotate_test,
}

if __name__ == "__main__":
    print("Initialising motor parameters...")
    motor.set_motor_parameter()
    motor.stop()
    print("Current calibration:")
    print(f"  MM_PER_SEC_AT_FULL  = {MM_PER_SEC_AT_FULL}")
    print(f"  DEG_PER_SEC_AT_TURN = {DEG_PER_SEC_AT_TURN}")
    print("\nCommands: d=drive  r=rotate  q=quit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == 'q':
                break
            action = COMMANDS.get(cmd)
            if action:
                action()
            else:
                print("Unknown command. Use d / r / q")
    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        print("\nMotors stopped.")
