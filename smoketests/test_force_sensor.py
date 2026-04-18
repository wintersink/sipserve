"""Drink-tray FSR smoke test — read the force-sensitive resistor under the
tray via an ADS1115 ADC and report how many cups are on the tray.

Wiring: FSR → voltage divider → ADS1115 channel 0, ADS1115 on I2C bus 1.

Usage:
    python smoketests/test_force_sensor.py            # live polling
    python smoketests/test_force_sensor.py --calibrate # print raw values to help set thresholds
"""

import argparse
import time
import Adafruit_ADS1x15

# ── ADC setup ────────────────────────────────────────────────────────────────
# busnum=1 is required on Pi 5 — the legacy library's platform detection
# doesn't recognize the Pi 5 and raises "Could not determine default I2C bus"
# if you let it auto-detect.
ADC = Adafruit_ADS1x15.ADS1115(busnum=1)
CHANNEL = 0
GAIN = 1

# ── Tray load thresholds (raw ADC counts) ────────────────────────────────────
# TODO: Run with --calibrate at each load level and fill in real values.
#       Place the tray with 0, 1, 2, 3, 4 cups and note the raw readings.
#       Set each threshold midway between adjacent load levels.
EMPTY_TRAY_MAX   = 1000   # below this → no tray / sensor error
ONE_CUP_MAX      = 2000   # empty tray to 1 cup
TWO_CUP_MAX      = 3000   # 1 cup to 2 cups
THREE_CUP_MAX    = 4000   # 2 cups to 3 cups
FOUR_CUP_MAX     = 5000   # 3 cups to 4 cups

TRAY_LEVELS = [
    (EMPTY_TRAY_MAX,  "Empty tray"),
    (ONE_CUP_MAX,     "1 cup"),
    (TWO_CUP_MAX,     "2 cups"),
    (THREE_CUP_MAX,   "3 cups"),
    (FOUR_CUP_MAX,    "4 cups"),
]


def classify(raw_value: int) -> str:
    for threshold, label in TRAY_LEVELS:
        if raw_value < threshold:
            return label
    return "4+ cups (saturated)"


def calibrate():
    """Print raw ADC values continuously so you can record readings at each
    load level and update the thresholds above."""
    print("Calibration mode — press Ctrl+C to stop")
    print("Place tray with 0, 1, 2, 3, 4 cups and note the raw values.\n")
    try:
        while True:
            raw = ADC.read_adc(CHANNEL, gain=GAIN)
            print(f"  raw = {raw}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nCalibration stopped.")


def poll():
    """Continuously read the FSR and print the classified tray load."""
    print("Tray load monitor — press Ctrl+C to stop\n")
    try:
        while True:
            raw = ADC.read_adc(CHANNEL, gain=GAIN)
            level = classify(raw)
            print(f"  raw = {raw:>5d}  →  {level}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drink-tray FSR smoke test")
    parser.add_argument("--calibrate", action="store_true",
                        help="Print raw ADC values for threshold calibration")
    args = parser.parse_args()
    if args.calibrate:
        calibrate()
    else:
        poll()
