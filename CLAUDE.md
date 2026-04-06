# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SipServe is an autonomous beverage delivery robot using a Raspberry Pi 5 with differential drive (tank treads), 5 ultrasonic sensors, an OakLite camera, and a Yahboom 4-channel Motor Encoder Driver board. It communicates with motors over I2C (address `0x26`) and GPIO/PWM via `pigpio`.

SSH access: user `testbot`, hostname `sipserve-johann`.

## Running the Code

```bash
# Activate the virtual environment (uses system site-packages)
source venv4sipserve/bin/activate

# Run main motor control demo
python main.py

# Run smoke tests (require hardware to be connected)
python smoketests/test_i2c.py   # Check I2C device presence at 0x26
python smoketests/IIC.py        # Full Yahboom motor driver test
```

No build step required — pure Python project.

## Architecture

```
main.py / control/          ← Application / navigation logic (control/ is a stub)
hardware/motor.py           ← Motor abstraction (pigpio PWM + direction pin)
smoketests/IIC.py           ← Comprehensive Yahboom I2C driver (encoder read/write)
sensors/apriltag.py         ← OAK-D-Lite AprilTag 36h11 detection + stereo depth distance (inches)
sensors/ultrasonic.py       ← Ultrasonic sensor integration (stub)
config.py                   ← Stub — intended for shared configuration constants
```

**Two motor control approaches exist in parallel:**
- `main.py` — direct `smbus2` I2C writes to the Yahboom board (register `0x01`, speeds 0–127)
- `hardware/motor.py` — `pigpio`-based PWM on individual GPIO pins, speed range −1.0 to 1.0

`smoketests/IIC.py` is the most complete driver and supports motor type configuration, encoder reads (total and real-time), deadzone tuning, and PWM control — it should be the reference implementation when building out the `control/` layer.

## Key Hardware Constants

| Item | Value |
|------|-------|
| I2C bus | 1 |
| Yahboom motor driver address | `0x26` |
| Motor control register | `0x01` |
| I2C pins | SDA = Pin 3, SCL = Pin 5 |

## Dependencies

- `smbus2` — I2C communication
- `pigpio` — GPIO/PWM control (daemon must be running: `sudo pigpiod`)
- `depthai` — Luxonis OAK-D-Lite camera pipeline (v3 API)
- `opencv-python-headless` — Image processing and display
