"""
Obstacle avoidance controller for SipServe.

Behaviour:
  - Drive forward unless any front-facing sensor reads < OBSTACLE_MM (12 inches / 305 mm).
  - On obstacle: stop, pick the side with more space, and turn until the front is clear.
  - If the front doesn't clear after MAX_TURN_S seconds: back up, re-assess, try opposite turn.
  - If still blocked: execute a 180° escape turn.

Run directly:
    source venv4sipserve/bin/activate
    python -m control.avoid
"""

import smbus2
import time
import sys

from hardware.mux import Mux, SENSOR_CHANNELS
from sensors.ultrasonic import UltrasonicSensor

# ── Hardware constants ────────────────────────────────────────────────────────
MOTOR_ADDR      = 0x26
PWM_CONTROL_REG = 0x07   # 8 bytes: 2 bytes big-endian int16 per motor (M1–M4)

# ── Tuning ────────────────────────────────────────────────────────────────────
OBSTACLE_MM   = 305   # 12 inches
DRIVE_SPEED   = 385   # forward / backward, range 0–1000
TURN_SPEED    = 330   # in-place rotation speed
LOOP_HZ       = 10    # sensor-poll rate while driving forward
MAX_TURN_S    = 2.0   # seconds to turn before giving up and backing up
BACKUP_S      = 0.6   # seconds to reverse before re-assessing
WAIT_CLEAR_S  = 2.0   # seconds to wait for obstacle to move before rotating

LEFT  = "left"
RIGHT = "right"


# ── Motor helpers ─────────────────────────────────────────────────────────────

def _pack(speed: int) -> list[int]:
    """Pack a signed int16 speed value as two big-endian bytes."""
    w = speed & 0xFFFF
    return [(w >> 8) & 0xFF, w & 0xFF]


def _send(bus: smbus2.SMBus, m1: int, m3: int) -> None:
    """Write M1 (left track) and M3 (right track); M2/M4 unused."""
    data = _pack(m1) + [0, 0] + _pack(m3) + [0, 0]
    bus.write_i2c_block_data(MOTOR_ADDR, PWM_CONTROL_REG, data)


def cmd_forward(bus):  _send(bus,  DRIVE_SPEED,  DRIVE_SPEED)
def cmd_backward(bus): _send(bus, -DRIVE_SPEED, -DRIVE_SPEED)
def cmd_turn_right(bus): _send(bus,  TURN_SPEED, -TURN_SPEED)
def cmd_turn_left(bus):  _send(bus, -TURN_SPEED,  TURN_SPEED)
def cmd_stop(bus):       _send(bus, 0, 0)


# ── Sensor helpers ────────────────────────────────────────────────────────────

def read_all(mux: Mux, sensor: UltrasonicSensor) -> dict[str, int | None]:
    """Return {name: distance_mm} for every sensor. None on read failure."""
    readings = {}
    for name, ch in SENSOR_CHANNELS.items():
        mux.select(ch)
        readings[name] = sensor.read_mm()
    return readings


def _safe(dist: int | None) -> int:
    """Return distance or a large value when the sensor fails."""
    return dist if dist is not None else 9999


def path_blocked(r: dict) -> bool:
    return (
        _safe(r["front"])       < OBSTACLE_MM
        or _safe(r["front_left"])  < OBSTACLE_MM
        or _safe(r["front_right"]) < OBSTACLE_MM
    )


def best_turn_direction(r: dict) -> str:
    """Choose the side with more clearance."""
    return RIGHT if _safe(r["right"]) >= _safe(r["left"]) else LEFT


# ── Avoidance logic ───────────────────────────────────────────────────────────

def _apply_turn(bus, direction: str) -> None:
    if direction == RIGHT:
        cmd_turn_right(bus)
    else:
        cmd_turn_left(bus)


def _wait_for_obstacle_to_clear(
    mux: Mux,
    sensor: UltrasonicSensor,
    timeout_s: float,
) -> bool:
    """
    Poll sensors for up to *timeout_s* seconds waiting for the path to clear.
    Returns True if the path clears within the timeout, False otherwise.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        time.sleep(0.1)
        r = read_all(mux, sensor)
        if not path_blocked(r):
            return True
    return False


def _turn_until_clear(
    bus: smbus2.SMBus,
    mux: Mux,
    sensor: UltrasonicSensor,
    direction: str,
    timeout_s: float,
) -> bool:
    """
    Turn in *direction* until the front clears or *timeout_s* elapses.
    Returns True if the front is now clear.
    """
    _apply_turn(bus, direction)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        time.sleep(0.1)
        r = read_all(mux, sensor)
        if not path_blocked(r):
            cmd_stop(bus)
            return True
    cmd_stop(bus)
    return False


def _log(r: dict) -> None:
    parts = " | ".join(
        f"{k}={_safe(r[k]):4d}" for k in ("front", "front_left", "front_right", "left", "right")
    )
    print(f"  [{parts}] mm")


# ── Main loop ─────────────────────────────────────────────────────────────────

def run(duration_s: float = 5.0) -> None:
    bus    = smbus2.SMBus(1)
    mux    = Mux(bus)
    sensor = UltrasonicSensor(bus)

    end_time = time.monotonic() + duration_s
    print(f"Obstacle avoidance running for {duration_s}s — Ctrl+C to stop early.")
    try:
        while time.monotonic() < end_time:
            r = read_all(mux, sensor)
            _log(r)

            # ── Clear path: keep driving forward ──────────────────────────────
            if not path_blocked(r):
                cmd_forward(bus)
                time.sleep(1 / LOOP_HZ)
                continue

            # ── Obstacle detected ─────────────────────────────────────────────
            cmd_stop(bus)
            print("  Obstacle! Waiting for it to clear…")
            if _wait_for_obstacle_to_clear(mux, sensor, WAIT_CLEAR_S):
                print("  Obstacle cleared — resuming forward.")
                continue

            # ── Still blocked after waiting: rotate to find a clear path ──────
            print("  Still blocked — rotating to find clear path…")
            r = read_all(mux, sensor)
            direction = best_turn_direction(r)
            print(f"  Turning {direction}…")
            cleared = _turn_until_clear(bus, mux, sensor, direction, MAX_TURN_S)

            if cleared:
                print("  Path clear — resuming forward.")
                continue

            # ── First turn timed out: back up and try opposite side ───────────
            print("  Still blocked — backing up…")
            cmd_backward(bus)
            time.sleep(BACKUP_S)
            cmd_stop(bus)
            time.sleep(0.15)

            r = read_all(mux, sensor)
            direction = best_turn_direction(r)
            print(f"  Retrying — turning {direction}…")
            cleared = _turn_until_clear(bus, mux, sensor, direction, MAX_TURN_S)

            if cleared:
                print("  Path clear — resuming forward.")
                continue

            # ── Still stuck: escape with a longer turn ────────────────────────
            print("  Escape turn…")
            _turn_until_clear(bus, mux, sensor, direction, MAX_TURN_S * 2)
            print("  Resuming forward.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        cmd_stop(bus)
        mux.disable()
        bus.close()


if __name__ == "__main__":
    run()
