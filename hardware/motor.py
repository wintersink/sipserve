"""Yahboom 4-channel Motor Encoder Driver (I2C address 0x26).

Wraps the Yahboom IIC protocol with a clean class-based API.

M1 = right tread, M3 = left tread (M2/M4 unused).
On this robot, negative PWM drives forward (wiring convention).

PWM range: -7200 to 7200 (0 to ~100% duty cycle).
"""

import smbus2
import struct
import time

# ── I2C constants ─────────────────────────────────────────────────────────────
MOTOR_ADDR           = 0x26
MOTOR_TYPE_REG       = 0x01
MOTOR_DEADZONE_REG   = 0x02
MOTOR_PULSELINE_REG  = 0x03
MOTOR_PULSEPHASE_REG = 0x04
WHEEL_DIA_REG        = 0x05
SPEED_CONTROL_REG    = 0x06
PWM_CONTROL_REG      = 0x07

# ── Default speeds (PWM units out of 7200) ────────────────────────────────────
FULL_SPEED = 720    # ~10% duty cycle
SLOW_SPEED = 360    # ~5%  duty cycle
TURN_SPEED = 360    # in-place rotation

# ── Robot geometry ────────────────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 55.0     # 520 motor wheels
TRACK_WIDTH_MM    = 154.0    # distance between tread centers
GEAR_RATIO        = 45       # 45:1
ENCODER_PPR       = 11       # informational — encoders aren't wired/reliable

# ── Time-based movement calibration ───────────────────────────────────────────
# Measured empirically — see smoketests/test_motor_calibration.py
# These are at FULL_SPEED / TURN_SPEED respectively.
MM_PER_SEC_AT_FULL  = 288.0   # measured at FULL_SPEED (720 PWM)
DEG_PER_SEC_AT_TURN = 84.0    # measured at TURN_SPEED (360 PWM)


class Motor:
    """Yahboom 4-channel motor driver (I2C)."""

    def __init__(self, bus: smbus2.SMBus, addr: int = MOTOR_ADDR):
        self.bus = bus
        self.addr = addr

    # ── Raw control ───────────────────────────────────────────────────────────

    def control_pwm(self, m1: int, m2: int, m3: int, m4: int) -> None:
        """Send raw signed-int16 PWM values to each motor (-7200..7200)."""
        data = [
            (m1 >> 8) & 0xFF, m1 & 0xFF,
            (m2 >> 8) & 0xFF, m2 & 0xFF,
            (m3 >> 8) & 0xFF, m3 & 0xFF,
            (m4 >> 8) & 0xFF, m4 & 0xFF,
        ]
        self.bus.write_i2c_block_data(self.addr, PWM_CONTROL_REG, data)

    # ── One-time configuration ────────────────────────────────────────────────

    def _write(self, reg: int, data: list) -> None:
        self.bus.write_i2c_block_data(self.addr, reg, data)

    def set_motor_type(self, t: int) -> None:
        self._write(MOTOR_TYPE_REG, [t])

    def set_motor_deadzone(self, d: int) -> None:
        self._write(MOTOR_DEADZONE_REG, [(d >> 8) & 0xFF, d & 0xFF])

    def set_pulse_line(self, p: int) -> None:
        self._write(MOTOR_PULSELINE_REG, [(p >> 8) & 0xFF, p & 0xFF])

    def set_pulse_phase(self, p: int) -> None:
        self._write(MOTOR_PULSEPHASE_REG, [(p >> 8) & 0xFF, p & 0xFF])

    def set_wheel_diameter(self, mm: float) -> None:
        self._write(WHEEL_DIA_REG, list(struct.pack('<f', mm)))

    def set_motor_parameter(self) -> None:
        """Configure for the SipServe motors (520-type, 55mm wheels, 45:1 gearbox)."""
        self.set_motor_type(1);                          time.sleep(0.1)
        self.set_pulse_phase(GEAR_RATIO);                time.sleep(0.1)
        self.set_pulse_line(ENCODER_PPR);                time.sleep(0.1)
        self.set_wheel_diameter(WHEEL_DIAMETER_MM);      time.sleep(0.1)
        self.set_motor_deadzone(1600);                   time.sleep(0.1)

    # ── High-level movement ───────────────────────────────────────────────────
    # Negative PWM = forward on M1 (right) and M3 (left) — wiring convention.

    def stop(self) -> None:
        self.control_pwm(0, 0, 0, 0)

    def forward(self, speed: int = FULL_SPEED) -> None:
        self.control_pwm(-speed, 0, -speed, 0)

    def reverse(self, speed: int = FULL_SPEED) -> None:
        self.control_pwm(speed, 0, speed, 0)

    def turn_left(self, speed: int = TURN_SPEED) -> None:
        # left tread backward, right tread forward
        self.control_pwm(-speed, 0, speed, 0)

    def turn_right(self, speed: int = TURN_SPEED) -> None:
        # left tread forward, right tread backward
        self.control_pwm(speed, 0, -speed, 0)

    # ── Distance / angle movement (time-based, blocking) ──────────────────────
    # Encoders aren't reliable on this robot, so we dead-reckon with calibrated
    # mm/sec and deg/sec values. AprilTag vision corrects drift during nav.

    def move_distance(self, mm: float, speed: int = FULL_SPEED) -> None:
        """Drive straight for *mm* millimeters. Negative = reverse."""
        duration = abs(mm) / MM_PER_SEC_AT_FULL
        if duration <= 0:
            return
        if mm >= 0:
            self.forward(speed)
        else:
            self.reverse(speed)
        try:
            time.sleep(duration)
        finally:
            self.stop()

    def rotate(self, degrees: float, speed: int = TURN_SPEED) -> None:
        """Rotate in place by *degrees*. Positive = right (clockwise from above)."""
        duration = abs(degrees) / DEG_PER_SEC_AT_TURN
        if duration <= 0:
            return
        if degrees >= 0:
            self.turn_right(speed)
        else:
            self.turn_left(speed)
        try:
            time.sleep(duration)
        finally:
            self.stop()
