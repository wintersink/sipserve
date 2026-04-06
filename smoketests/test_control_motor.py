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
Positive speed = forward, negative = reverse.
"""

import sys
import os
import time

# Allow importing IIC from the same directory
sys.path.insert(0, os.path.dirname(__file__))
import sipserve.smoketests.IIC as IIC

SPEED = 720       # ~10% duty cycle (PWM range 0–7200)
DURATION = 2.0    # seconds each command runs

def stop():
    print("Stopping...")
    IIC.control_pwm(0, 0, 0, 0)

def forward():
    print("Forward...")
    IIC.control_pwm(-SPEED, 0, -SPEED, 0)
    time.sleep(DURATION)
    stop()

def reverse():
    print("Reverse...")
    IIC.control_pwm(SPEED, 0, SPEED, 0)
    time.sleep(DURATION)
    stop()

def turn_left():
    print("Turning left...")
    # Left tread backward, right tread forward
    IIC.control_pwm(-SPEED, 0, SPEED, 0)
    time.sleep(DURATION)
    stop()

def turn_right():
    print("Turning right...")
    # Left tread forward, right tread backward
    IIC.control_pwm(SPEED, 0, -SPEED, 0)
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
    IIC.set_motor_parameter()
    IIC.control_pwm(0, 0, 0, 0)
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
