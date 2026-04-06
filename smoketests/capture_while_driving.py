"""
capture_while_driving.py — Drive around avoiding obstacles while auto-capturing
training images from the OAK-D-Lite.

Combines obstacle avoidance from test_obstacle_avoidance.py with frame capture
from capture_dataset.py.  The robot drives autonomously and saves a frame every
CAPTURE_INTERVAL seconds.

Controls (press in terminal, then Enter):
    q  — quit (stops motors and exits)

Images are saved to:  dataset/images/YYYY-MM-DD_HH-MM-SS_NNN.jpg

Usage:
    source venv4sipserve/bin/activate
    python smoketests/capture_while_driving.py
"""

import sys
import os
import time
import threading

import cv2
import numpy as np
import depthai as dai
import smbus2

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import IIC
from hardware.mux import Mux, SENSOR_CHANNELS
from sensors.ultrasonic import UltrasonicSensor

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SAVE_DIR        = os.path.join(os.path.dirname(__file__), "..", "dataset", "images")
RGB_W, RGB_H    = 640, 480
CAPTURE_INTERVAL = 2.0   # seconds between auto-captures

SPEED           = 720    # ~10% duty cycle forward
SLOW_SPEED      = 180    # ~2.5% duty cycle when side sensors trigger
TURN_SPEED      = 360    # turning speed
REVERSE_TIME    = 0.5    # seconds to reverse when front is blocked
TURN_TIME       = 0.5    # seconds to turn when front is blocked
OBSTACLE_MM     = 800    # detection threshold in mm
POLL_INTERVAL   = 0.05   # seconds between sensor sweeps

os.makedirs(SAVE_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Camera pipeline — RGB only
# ---------------------------------------------------------------------------
def build_pipeline():
    pipeline = dai.Pipeline()
    camRgb   = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb_out  = camRgb.requestOutput((RGB_W, RGB_H))
    queues   = {"rgb": rgb_out.createOutputQueue()}
    return pipeline, queues

# ---------------------------------------------------------------------------
# Non-blocking stdin reader
# ---------------------------------------------------------------------------
_cmd_lock = threading.Lock()
_pending  = []

def _input_thread():
    while True:
        try:
            line = input()
        except EOFError:
            break
        with _cmd_lock:
            _pending.append(line.strip().lower())

# ---------------------------------------------------------------------------
# Frame save helper
# ---------------------------------------------------------------------------
_frame_count = 0

def save_frame(frame):
    global _frame_count
    _frame_count += 1
    ts   = time.strftime("%Y-%m-%d_%H-%M-%S")
    name = f"{ts}_{_frame_count:04d}.jpg"
    path = os.path.join(SAVE_DIR, name)
    cv2.imwrite(path, frame)
    print(f"[SAVED]  {path}")
    return path

# ---------------------------------------------------------------------------
# Motor helpers
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# Sensor helpers
# ---------------------------------------------------------------------------
bus    = smbus2.SMBus(1)
mux    = Mux(bus)
sensor = UltrasonicSensor(bus)

def read_all_sensors():
    readings = {}
    for name, channel in SENSOR_CHANNELS.items():
        mux.select(channel)
        readings[name] = sensor.read_mm()
    mux.disable()
    return readings

def is_triggered(dist):
    return dist is not None and dist < OBSTACLE_MM

def print_readings(readings, state):
    parts = []
    for name, dist in readings.items():
        if dist is None:
            parts.append(f"{name}: ERR")
        else:
            parts.append(f"{name}: {dist}mm")
    print(f"[{state}]  " + "  ".join(parts))

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Initialising motor parameters...")
    IIC.set_motor_parameter()
    stop_motors()

    print("Building camera pipeline...")
    pipeline, queues = build_pipeline()

    inp_thread = threading.Thread(target=_input_thread, daemon=True)
    inp_thread.start()

    last_capture_t = 0.0

    print(f"\nDataset folder: {os.path.abspath(SAVE_DIR)}")
    print(f"Obstacle threshold: {OBSTACLE_MM} mm")
    print(f"Capturing a frame every {CAPTURE_INTERVAL:.0f}s while driving")
    print("─" * 50)
    print("  q + Enter  →  quit")
    print("─" * 50)

    try:
        with pipeline:
            pipeline.start()
            print("Pipeline running — robot is driving.\n")

            while pipeline.isRunning():
                # --- Grab latest camera frame ---
                rgb_msg = queues["rgb"].get()
                frame   = rgb_msg.getCvFrame()
                if frame is None:
                    continue
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                frame = cv2.resize(frame, (RGB_W, RGB_H))

                # --- Auto-capture on interval ---
                now = time.time()
                if (now - last_capture_t) >= CAPTURE_INTERVAL:
                    save_frame(frame)
                    last_capture_t = now

                # --- Check for quit command ---
                with _cmd_lock:
                    cmds = list(_pending)
                    _pending.clear()
                for cmd in cmds:
                    if cmd == "q":
                        raise KeyboardInterrupt

                # --- Read sensors and drive ---
                readings    = read_all_sensors()
                front       = is_triggered(readings.get("front"))
                front_left  = is_triggered(readings.get("front_left"))
                front_right = is_triggered(readings.get("front_right"))
                left        = is_triggered(readings.get("left"))
                right       = is_triggered(readings.get("right"))

                if front:
                    print_readings(readings, "FRONT BLOCKED")
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

                elif front_left and front_right:
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
                    print_readings(readings, "TURN RIGHT")
                    turn_right()
                    time.sleep(POLL_INTERVAL)

                elif front_right:
                    print_readings(readings, "TURN LEFT")
                    turn_left()
                    time.sleep(POLL_INTERVAL)

                elif left or right:
                    print_readings(readings, "SLOW")
                    drive_forward(SLOW_SPEED)
                    time.sleep(POLL_INTERVAL)

                else:
                    print_readings(readings, "FORWARD")
                    drive_forward()
                    time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        mux.disable()
        print(f"\n[DONE]  {_frame_count} frames saved to {os.path.abspath(SAVE_DIR)}")
        print("Motors stopped. Mux disabled.")
