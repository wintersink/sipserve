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

# ── Default drive speed (PWM units out of 7200) ───────────────────────────────
FULL_SPEED = 1440    # 720 is ~10% duty cycle — straight-line cruise
SLOW_SPEED = 360    # ~5%  duty cycle

# ── Rotation-style speeds ─────────────────────────────────────────────────────
# Two distinct rotation styles, each with its own speed knob so they can be
# tuned independently. There is intentionally no generic "TURN_SPEED" — pick
# the style you want and edit the matching constant.
#
# BURST_TURN_SPEED — used by Motor.burst_rotate() for short rotation pulses
#   that alternate with still pauses so the AprilTag detector gets sharp
#   frames between motions. Tune this to control how far per burst the robot
#   sweeps; pair with BURST_ROTATE_S for the time-per-pulse.
#
# SMOOTH_TURN_SPEED — used by Motor.smooth_turn() AND by the bare
#   turn_left / turn_right / rotate primitives. Continuous (no-pause)
#   rotation, e.g. spinning to find a clear path. Slower than
#   BURST_TURN_SPEED so the camera retains a chance to catch tags during
#   uninterrupted rotation. Tune this for scan / clear-path / general
#   in-place rotation behavior.
BURST_TURN_SPEED  = 720   # per-pulse speed for Motor.burst_rotate
SMOOTH_TURN_SPEED = 270   # continuous-rotation speed for everything else

# ── Burst-rotate timing ───────────────────────────────────────────────────────
# Pulse width / pause width for Motor.burst_rotate. Affects WHEN the motors
# stop, not how fast they spin (that's BURST_TURN_SPEED above).
BURST_ROTATE_S = 0.3   # rotate this long per burst
BURST_PAUSE_S  = 0.2   # pause after each burst to let the camera settle

# ── Robot geometry ────────────────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 55.0     # 520 motor wheels
TRACK_WIDTH_MM    = 154.0    # distance between tread centers
GEAR_RATIO        = 45       # 45:1
ENCODER_PPR       = 11       # informational — encoders aren't wired/reliable

# ── Time-based movement calibration ───────────────────────────────────────────
# Measured empirically — see smoketests/test_motor_calibration.py
# Per-PWM-unit rates are the calibration primitives; the at-FULL_SPEED and
# at-SMOOTH_TURN_SPEED constants are derived so changing those speed knobs
# automatically rescales the dead-reckoning math (assumes linearity, which
# holds well enough in the operating range we use).
#
# Original calibration points:
#   straight-line: 288.0 mm/sec @ FULL_SPEED=720  → 0.4    mm/sec/PWM
#   in-place spin: 84.0  deg/sec @ 360 PWM        → 0.2333 deg/sec/PWM
MM_PER_SEC_PER_PWM  = 0.4
DEG_PER_SEC_PER_PWM = 84.0 / 360.0   # ≈ 0.2333

MM_PER_SEC_AT_FULL          = MM_PER_SEC_PER_PWM  * FULL_SPEED         # 288.0
DEG_PER_SEC_AT_SMOOTH_TURN  = DEG_PER_SEC_PER_PWM * SMOOTH_TURN_SPEED  # ≈ 63.0
DEG_PER_SEC_AT_BURST_TURN   = DEG_PER_SEC_PER_PWM * BURST_TURN_SPEED   # ≈ 168.0


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

    def turn_left(self, speed: int = SMOOTH_TURN_SPEED) -> None:
        # left tread backward, right tread forward
        self.control_pwm(-speed, 0, speed, 0)

    def turn_right(self, speed: int = SMOOTH_TURN_SPEED) -> None:
        # left tread forward, right tread backward
        self.control_pwm(speed, 0, -speed, 0)

    # ── Burst / smooth turns ──────────────────────────────────────────────────

    def smooth_turn(self, direction: str, speed: int = SMOOTH_TURN_SPEED) -> None:
        """Continuous (non-bursted) rotation at a gentler speed.

        Use when rotating until a condition is met (e.g. clear path found)
        without introducing still-frame pauses. Caller is responsible for
        calling stop() when done.
        """
        if direction == "right":
            self.turn_right(speed)
        elif direction == "left":
            self.turn_left(speed)
        else:
            raise ValueError(f"direction must be 'left' or 'right', got {direction!r}")

    def burst_rotate(self, direction: str,
                     speed: int = BURST_TURN_SPEED,
                     burst_s: float = BURST_ROTATE_S,
                     pause_s: float = BURST_PAUSE_S) -> None:
        """Rotate in a short burst, then stop and pause.

        Defaults to BURST_TURN_SPEED so this rotation style stays tunable
        independently of SMOOTH_TURN_SPEED (which drives the bare
        turn_left / turn_right primitives this method calls into).
        The pause between bursts leaves still frames for the AprilTag
        detector — motion blur destroys tag detection reliability.
        """
        if direction == "right":
            self.turn_right(speed)
        elif direction == "left":
            self.turn_left(speed)
        else:
            raise ValueError(f"direction must be 'left' or 'right', got {direction!r}")
        time.sleep(burst_s)
        self.stop()
        time.sleep(pause_s)

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

    def rotate(self, degrees: float, speed: int = SMOOTH_TURN_SPEED) -> None:
        """Rotate in place by *degrees*. Positive = right (clockwise from above).

        Duration is derived from DEG_PER_SEC_AT_SMOOTH_TURN, which is
        SMOOTH_TURN_SPEED × DEG_PER_SEC_PER_PWM. If you call this with a
        non-default speed, the dead-reckoned duration will be off in
        proportion to (speed / SMOOTH_TURN_SPEED).
        """
        duration = abs(degrees) / DEG_PER_SEC_AT_SMOOTH_TURN
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
