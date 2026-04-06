#!/usr/bin/env python3
"""
Delivery test: navigate to a specified AprilTag and stop 10 inches away, centered.

Tag ID mapping:
  0 = Home
  1 = Table 1
  2 = Table 2
  3 = Table 3
  4 = Table 4

Usage:
    source venv4sipserve/bin/activate
    python smoketests/test_delivery.py <destination>

Destinations: home, table1, table2, table3, table4
"""

import sys
import os
import time

import depthai as dai

# Allow imports from project root (sensors/) and smoketests/ (IIC)
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                        # for IIC
sys.path.insert(0, os.path.join(_HERE, ".."))    # for sensors.apriltag

import IIC
from sensors.apriltag import build_pipeline, send_roi_configs, MM_PER_INCH
from config import TAG_IDS

# ── Tuning ────────────────────────────────────────────────────────────────────
STOP_DIST_INCHES = 10.0   # stop this close to the tag
CENTER_X_MM      = 60     # acceptable lateral offset (mm) to consider centered
APPROACH_SPEED   = 500    # PWM magnitude for driving forward
TURN_SPEED       = 400    # PWM magnitude for in-place centering
SEARCH_SPEED     = 350    # PWM magnitude for search spin


# ── Motor helpers ─────────────────────────────────────────────────────────────
# M1 = right tread, M3 = left tread. Negative PWM = forward.

def stop():
    IIC.control_pwm(0, 0, 0, 0)

def forward():
    IIC.control_pwm(-APPROACH_SPEED, 0, -APPROACH_SPEED, 0)

def turn_left():
    IIC.control_pwm(-TURN_SPEED, 0, TURN_SPEED, 0)

def turn_right():
    IIC.control_pwm(TURN_SPEED, 0, -TURN_SPEED, 0)

def search_spin():
    """Rotate right slowly to scan for the target tag."""
    IIC.control_pwm(SEARCH_SPEED, 0, -SEARCH_SPEED, 0)


# ── Delivery loop ─────────────────────────────────────────────────────────────

def deliver(target_key: str) -> None:
    target_id = TAG_IDS[target_key]
    print(f"Navigating to {target_key!r} (tag ID {target_id}) — Ctrl+C to abort.")

    IIC.set_motor_parameter()
    stop()

    pipeline, q = build_pipeline()
    prev_target_tag = None

    with pipeline:
        pipeline.start()
        try:
            while pipeline.isRunning():
                # ── AprilTag detections ───────────────────────────────────────
                tag_msg = q["tags"].get()
                all_tags = list(tag_msg.aprilTags)
                target_tags = [t for t in all_tags if t.id == target_id]

                # Send ROI for the target tag so depth can be calculated
                if target_tags:
                    send_roi_configs([target_tags[0]], q["spatial_cfg"])

                # ── Spatial data (one frame behind the ROI we just sent) ──────
                spatial_msg = q["spatial"].get()
                spatial_locations = spatial_msg.getSpatialLocations()

                dist_inches = None
                x_mm = None
                if prev_target_tag is not None and spatial_locations:
                    loc = spatial_locations[0]
                    z_mm = loc.spatialCoordinates.z
                    if z_mm > 0:
                        dist_inches = z_mm / MM_PER_INCH
                        x_mm = loc.spatialCoordinates.x

                prev_target_tag = target_tags[0] if target_tags else None

                # ── Control logic ─────────────────────────────────────────────
                if dist_inches is None:
                    search_spin()
                    print(f"  Searching for tag {target_id}...")
                    continue

                print(f"  Tag {target_id}: {dist_inches:.1f} in  x={x_mm:+.0f} mm")

                if dist_inches <= STOP_DIST_INCHES:
                    stop()
                    print(f"\nArrived at {target_key!r}! Stopped {dist_inches:.1f} inches away.")
                    break

                if abs(x_mm) > CENTER_X_MM:
                    if x_mm > 0:
                        turn_right()
                        print("  Centering — turning right")
                    else:
                        turn_left()
                        print("  Centering — turning left")
                else:
                    forward()
                    print("  Approaching...")

        except KeyboardInterrupt:
            print("\nAborted by user.")
        finally:
            stop()
            pipeline.stop()
            print("Motors stopped.")


def prompt_destination() -> str:
    print("Select a destination:")
    options = list(TAG_IDS.keys())
    for i, name in enumerate(options, 1):
        print(f"  {i}. {name}")
    while True:
        raw = input("Enter name or number: ").strip().lower().replace(" ", "")
        if raw in TAG_IDS:
            return raw
        if raw.isdigit() and 1 <= int(raw) <= len(options):
            return options[int(raw) - 1]
        print(f"  Invalid — enter a name ({', '.join(options)}) or number 1–{len(options)}.")


def main():
    if len(sys.argv) >= 2:
        dest = sys.argv[1].lower().replace(" ", "")
        if dest not in TAG_IDS:
            print(f"Unknown destination: {sys.argv[1]!r}")
            print("Valid destinations:", "  ".join(TAG_IDS.keys()))
            sys.exit(1)
    else:
        dest = prompt_destination()

    deliver(dest)


if __name__ == "__main__":
    main()
