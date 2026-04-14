"""
Time-based motor calibration.

Drives at FULL_SPEED / SMOOTH_TURN_SPEED for a fixed duration and asks you
to measure the result. Computes the per-PWM-unit calibration primitives
you should plug into hardware/motor.py.

Commands:
  d  — drive forward at FULL_SPEED
  r  — rotate right at SMOOTH_TURN_SPEED
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
    SMOOTH_TURN_SPEED,
    MM_PER_SEC_PER_PWM,
    DEG_PER_SEC_PER_PWM,
    MM_PER_SEC_AT_FULL,
    DEG_PER_SEC_AT_SMOOTH_TURN,
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
    new_per_pwm = mm_per_sec / FULL_SPEED
    print(f"  → {mm_per_sec:.1f} mm/sec at FULL_SPEED "
          f"(current derived: {MM_PER_SEC_AT_FULL:.1f})")
    print(f"  Set MM_PER_SEC_PER_PWM = {new_per_pwm:.4f} in hardware/motor.py "
          f"(current: {MM_PER_SEC_PER_PWM})")


def rotate_test():
    print(f"\nRotating right at SMOOTH_TURN_SPEED ({SMOOTH_TURN_SPEED}) "
          f"for {ROTATE_DURATION_S}s...")
    print("Mark the starting heading. Starting in 2s...")
    time.sleep(2)
    motor.turn_right(SMOOTH_TURN_SPEED)
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
    new_per_pwm = deg_per_sec / SMOOTH_TURN_SPEED
    print(f"  → {deg_per_sec:.1f} deg/sec at SMOOTH_TURN_SPEED "
          f"(current derived: {DEG_PER_SEC_AT_SMOOTH_TURN:.1f})")
    print(f"  Set DEG_PER_SEC_PER_PWM = {new_per_pwm:.4f} in hardware/motor.py "
          f"(current: {DEG_PER_SEC_PER_PWM:.4f})")


COMMANDS = {
    'd': drive_test,
    'r': rotate_test,
}

if __name__ == "__main__":
    print("Initialising motor parameters...")
    motor.set_motor_parameter()
    motor.stop()
    print("Current calibration:")
    print(f"  MM_PER_SEC_PER_PWM         = {MM_PER_SEC_PER_PWM}")
    print(f"  DEG_PER_SEC_PER_PWM        = {DEG_PER_SEC_PER_PWM:.4f}")
    print(f"  MM_PER_SEC_AT_FULL         = {MM_PER_SEC_AT_FULL:.1f}")
    print(f"  DEG_PER_SEC_AT_SMOOTH_TURN = {DEG_PER_SEC_AT_SMOOTH_TURN:.1f}")
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
