#!/usr/bin/env python3
"""
Delivery round-trip: clear startup path, wait for destination request, navigate
to AprilTag, pause, then return home.

The robot starts by spinning until the forward path is clear and no AprilTag is
visible.  It then waits for an operator to enter a destination.  All 5 ultrasonic
sensors are used for obstacle avoidance.  The OAK-D-Lite camera is used to find
and approach destination AprilTags.

Usage:
    source venv4sipserve/bin/activate
    python smoketests/test_delivery_run.py

Destinations (prompted interactively): table1, table2, table3, table4
"""

import enum
import math
import os
import sys
import threading
import time
from collections import deque

import depthai as dai
import smbus2

# ── Path setup (same pattern as other smoketests) ────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, ".."))

import IIC
from sensors.apriltag import (send_roi_configs,
                              TAG_INPUT_W, TAG_INPUT_H,
                              DEPTH_MIN_MM, DEPTH_MAX_MM)
from sensors.ultrasonic import UltrasonicSensor
from hardware.mux import Mux, SENSOR_CHANNELS
from config import TAG_IDS
from hardware.lcd import SipServeLCD
from sensors.voice import VoiceSensor

IMU_RATE_HZ = 100

# ── Tuning ────────────────────────────────────────────────────────────────────

# Speeds (PWM magnitude — sent as negative for forward, positive for backward)
ROAM_SPEED     = 800   # cruising while searching for tag
CRUISE_SPEED   = 500   # driving toward a visible tag, far away
APPROACH_SPEED = 250   # close to tag, fine control
TURN_SPEED     = 300   # centering corrections during NAVIGATE
AVOID_TURN     = 300   # turning away from obstacles
BACKUP_SPEED   = 300   # reversing away from an obstacle

# Distance thresholds (mm)
OBSTACLE_MM      = 300    # ~12 inches — obstacle trigger (ultrasonic + depth)
SLOW_DIST_MM     = 508    # 20 inches — start slowing down near tag
STOP_DIST_MM     = 254    # 10 inches — stop at destination
CENTER_X_MM      = 60     # lateral offset tolerance for "centered"

# Ramp zone: linear speed interpolation between these two distances
RAMP_FAR_MM  = 508    # above this → CRUISE_SPEED
RAMP_NEAR_MM = 300    # below this → APPROACH_SPEED

# Lock-on: when tag is lost, hold heading via gyro at reduced speed
LOCKON_SPEED       = 300   # forward speed while holding heading (tag lost)
HEADING_TOL_DEG    = 5.0   # heading error tolerance before correcting
HEADING_TURN_SPEED = 200   # gentle turn to correct heading drift

# Timing
TAG_LOST_TIMEOUT_S = 3.0   # seconds without tag before LOST state
TAG_LOST_FORWARD_GRACE_S = 1.5  # coast forward this long on brief tag loss
SEARCH_SPIN_TIMEOUT_S = 10.0  # SEARCHING: 10s with no lock → 360° spin
SPIN_SWEEP_DEG     = 360.0  # how far to spin when scanning for a tag
DELIVER_PAUSE_S    = 6.0   # seconds to pause at destination for delivery
AVOID_TIMEOUT_S    = 4.0   # max seconds turning during obstacle avoidance
BACKUP_S           = 0.5   # seconds to reverse if turn times out


# ── State machine ─────────────────────────────────────────────────────────────

# ── Voice command IDs (DF2301QG custom training) ─────────────────────────────

VCMD_WAKE_WORD      = 2
VCMD_QUEUE_TABLE1   = 5
VCMD_QUEUE_TABLE2   = 6
VCMD_QUEUE_TABLE3   = 7
VCMD_QUEUE_TABLE4   = 8
VCMD_CLEAR_ORDERS   = 9
VCMD_START_DELIVERY = 10

VCMD_QUEUE_MAP = {
    VCMD_QUEUE_TABLE1: "table1",
    VCMD_QUEUE_TABLE2: "table2",
    VCMD_QUEUE_TABLE3: "table3",
    VCMD_QUEUE_TABLE4: "table4",
}


class State(enum.Enum):
    INITIALIZING = "INITIALIZING"  # rotate until home tag found
    ORDERING     = "ORDERING"      # wait for wake word, then queue tables
    SEARCHING    = "SEARCHING"     # clear path, search for queued tags
    LOCKED       = "LOCKED"        # approaching a found tag
    LOST         = "LOST"          # lost sight of tag, re-finding
    DELIVERING   = "DELIVERING"    # paused at tag, delivery in progress
    RETURNING    = "RETURNING"     # heading home after all deliveries


# ── Motor helpers ─────────────────────────────────────────────────────────────
# Negative PWM = forward on this robot (wiring is inverted).
# M1 = right tread, M3 = left tread.

def stop():
    IIC.control_pwm(0, 0, 0, 0)

def forward(speed=ROAM_SPEED):
    IIC.control_pwm(-speed, 0, -speed, 0)

def backward(speed=BACKUP_SPEED):
    IIC.control_pwm(speed, 0, speed, 0)

def turn_left(speed=TURN_SPEED):
    IIC.control_pwm(-speed, 0, speed, 0)

def turn_right(speed=TURN_SPEED):
    IIC.control_pwm(speed, 0, -speed, 0)


# Proportional steering constant: how aggressively to steer per mm of offset.
# At STEER_GAIN=1.0 and x_mm=200, the slower tread runs at speed - 200.
STEER_GAIN = 1.5


def arc_drive(speed: int, x_mm: float):
    """Drive forward while curving toward the tag.

    speed: base forward PWM magnitude
    x_mm:  lateral offset — positive = tag is to the right
    """
    steer = int(STEER_GAIN * x_mm)
    # Clamp so neither tread reverses (no in-place spin during arc)
    steer = max(-speed, min(speed, steer))
    left  = speed + steer   # speed up left tread to turn right
    right = speed - steer   # slow down right tread to turn right
    # Negative PWM = forward
    IIC.control_pwm(-right, 0, -left, 0)


# ── Ultrasonic background poller ──────────────────────────────────────────────

def _safe(dist: int | None) -> int:
    return dist if dist is not None else 9999


class UltrasonicPoller:
    """Continuously polls all 5 ultrasonic sensors in a background thread.

    The main loop calls .snapshot() to get the latest readings without
    blocking, so the camera loop runs at full speed.
    """

    def __init__(self, bus: smbus2.SMBus, i2c_lock: threading.Lock | None = None):
        self._mux = Mux(bus)
        self._sensor = UltrasonicSensor(bus)
        self._i2c_lock = i2c_lock
        self._lock = threading.Lock()
        self._readings: dict[str, int | None] = {name: None for name in SENSOR_CHANNELS}
        self._ready = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def wait_ready(self, timeout: float = 3.0):
        """Block until a full sensor sweep completes with at least one valid reading."""
        print("  Waiting for ultrasonic sensors…")
        self._ready.clear()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._ready.wait(timeout=0.5)
            snap = self.snapshot()
            if any(v is not None for v in snap.values()):
                print(f"  Sensors ready: {snap}")
                return
            self._ready.clear()
        print(f"  Sensor timeout — readings: {self.snapshot()}")

    def _poll_loop(self):
        """Read all 5 sensors in a continuous loop (~0.8 s per cycle).

        When an i2c_lock is provided, only hold it during actual I2C ops
        (trigger and read), NOT during the 150ms sensor measurement wait.
        This lets the voice poller access the mux between measurements.
        """
        while self._running:
            fresh = {}
            for name, ch in SENSOR_CHANNELS.items():
                if not self._running:
                    return
                if self._i2c_lock:
                    with self._i2c_lock:
                        self._mux.select(ch)
                        self._sensor.trigger()
                    time.sleep(0.15)
                    with self._i2c_lock:
                        # Voice poller may have changed the mux channel
                        # during the 150ms wait — invalidate cache and
                        # force a re-select.
                        self._mux._active = None
                        self._mux.select(ch)
                        fresh[name] = self._sensor.read_result()
                else:
                    self._mux.select(ch)
                    fresh[name] = self._sensor.read_mm()
            with self._lock:
                self._readings.update(fresh)
            self._ready.set()

    def snapshot(self) -> dict[str, int | None]:
        """Return the most recent readings (non-blocking)."""
        with self._lock:
            return dict(self._readings)

    def front_blocked(self) -> bool:
        snap = self.snapshot()
        return any(_safe(snap.get(k)) < OBSTACLE_MM
                   for k in ("front", "front_left", "front_right"))

    def min_front_dist(self) -> int:
        """Return the minimum distance (mm) across front-facing sensors."""
        snap = self.snapshot()
        return min(_safe(snap.get(k)) for k in ("front", "front_left", "front_right"))

    def best_turn_dir(self) -> str:
        """Return 'left' or 'right' based on which side has more clearance."""
        snap = self.snapshot()
        return "right" if _safe(snap.get("right")) >= _safe(snap.get("left")) else "left"

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._mux.disable()


# ── Voice background poller ──────────────────────────────────────────────────

VOICE_MUX_CHANNEL = 5

class VoicePoller:
    """Polls the DF2301QG voice sensor in a background thread."""

    def __init__(self, bus: smbus2.SMBus, i2c_lock: threading.Lock | None = None):
        self._sensor = VoiceSensor(bus, mux_channel=VOICE_MUX_CHANNEL,
                                   i2c_lock=i2c_lock)
        self._commands: deque[int] = deque(maxlen=16)
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            cmd = self._sensor.get_command_id()
            if cmd and cmd != 0:
                with self._lock:
                    self._commands.append(cmd)
            time.sleep(0.1)

    def get_commands(self) -> list[int]:
        """Drain and return all pending voice commands."""
        with self._lock:
            cmds = list(self._commands)
            self._commands.clear()
        return cmds

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)


# ── Pipeline builder (lightweight: AprilTag + depth + IMU, no YOLO) ───────────

def build_delivery_pipeline():
    """Build a lean OAK-D-Lite pipeline for delivery navigation.

    Includes AprilTag detection, stereo depth, and IMU — but no YOLO or
    FeatureTracker, so there's no shave contention on the OAK-D-Lite.
    """
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    tag_out = camRgb.requestOutput((TAG_INPUT_W, TAG_INPUT_H))

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    monoLeft.requestOutput((640, 400)).link(stereo.left)
    monoRight.requestOutput((640, 400)).link(stereo.right)

    aprilTag = pipeline.create(dai.node.AprilTag)
    aprilTag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_36H11)
    tag_out.link(aprilTag.inputImage)

    spatialCalc = pipeline.create(dai.node.SpatialLocationCalculator)
    stereo.depth.link(spatialCalc.inputDepth)
    spatialCalc.inputConfig.setWaitForMessage(False)
    default_cfg = dai.SpatialLocationCalculatorConfigData()
    default_cfg.depthThresholds.lowerThreshold = DEPTH_MIN_MM
    default_cfg.depthThresholds.upperThreshold = DEPTH_MAX_MM
    default_cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
    default_cfg.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
    spatialCalc.initialConfig.addROI(default_cfg)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, IMU_RATE_HZ)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_RATE_HZ)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    queues = {
        "tags": aprilTag.out.createOutputQueue(),
        "spatial": spatialCalc.out.createOutputQueue(),
        "spatial_cfg": spatialCalc.inputConfig.createInputQueue(),
        "depth": stereo.depth.createOutputQueue(),
        "imu": imu.out.createOutputQueue(),
    }
    return pipeline, queues


# ── Gyro tracker ──────────────────────────────────────────────────────────────

class GyroTracker:
    """Integrates BMI270 raw gyroscope z-axis to track heading changes."""

    def __init__(self):
        self.heading_deg = 0.0
        self._prev_ts = None

    def update(self, imu_queue):
        while True:
            msg = imu_queue.tryGet()
            if msg is None:
                break
            for packet in msg.packets:
                gyro = packet.gyroscope
                ts = gyro.getTimestampDevice().total_seconds()
                if self._prev_ts is not None:
                    dt = ts - self._prev_ts
                    if 0 < dt < 0.5:
                        self.heading_deg += math.degrees(gyro.z) * dt
                self._prev_ts = ts
        return self.heading_deg

    def reset(self):
        self.heading_deg = 0.0
        self._prev_ts = None


# ── Tag helper ────────────────────────────────────────────────────────────────

class TagTracker:
    """Tracks target AprilTag(s) across frames.

    Handles the one-frame-behind spatial data: we send an ROI for the tag
    we see now, but the spatial result we read back corresponds to the ROI
    we sent in the *previous* frame.

    Accepts a set of target IDs so SEARCH can match any queued tag.
    """

    def __init__(self):
        self._prev_tag = None  # tag object from previous frame (or None)
        self.tag_px = None     # current-frame tag center x in pixels (or None)
        self.any_tag_visible = False  # were ANY tags visible this frame?
        self.visible_ids: set[int] = set()  # IDs of all tags seen this frame
        self.visible_centers: dict[int, float] = {}  # id → pixel x of center

    def update(self, queues, target_ids) -> tuple[float | None, float | None, int | None]:
        """Read one camera frame and return (dist_mm, x_mm, found_id).

        target_ids: a set (or single int) of tag IDs to match.
        Returns (None, None, None) if no target tag is visible or depth is
        not yet available.  self.tag_px is set to the tag's pixel center x
        on the current frame (available even when depth is not).
        """
        if isinstance(target_ids, int):
            target_ids = {target_ids}

        tag_msg = queues["tags"].get()
        all_tags = list(tag_msg.aprilTags)
        self.any_tag_visible = bool(all_tags)
        self.visible_ids = {t.id for t in all_tags}
        self.visible_centers = {
            t.id: (t.topLeft.x + t.topRight.x +
                   t.bottomLeft.x + t.bottomRight.x) / 4.0
            for t in all_tags
        }
        target_tags = [t for t in all_tags if t.id in target_ids]

        # Tag pixel position — available THIS frame, no one-frame lag
        found_id = None
        if target_tags:
            tag = target_tags[0]
            found_id = tag.id
            self.tag_px = (tag.topLeft.x + tag.topRight.x +
                           tag.bottomLeft.x + tag.bottomRight.x) / 4.0
            send_roi_configs([tag], queues["spatial_cfg"])
        else:
            self.tag_px = None

        # Read spatial data (corresponds to *previous* frame's ROI)
        spatial_msg = queues["spatial"].get()
        spatial_locs = spatial_msg.getSpatialLocations()

        dist_mm = None
        x_mm = None
        if self._prev_tag is not None and spatial_locs:
            loc = spatial_locs[0]
            z = loc.spatialCoordinates.z
            if z > 0:
                dist_mm = z
                x_mm = loc.spatialCoordinates.x

        self._prev_tag = target_tags[0] if target_tags else None
        return dist_mm, x_mm, found_id


# Camera center x in pixels — offset from this = how far off-center the tag is
_CAM_CENTER_PX = TAG_INPUT_W / 2.0
# Rough scale: convert pixel offset to approximate mm offset for arc_drive.
# At ~1m distance, 1 pixel ≈ 1.5 mm lateral offset (rough FOV estimate).
PX_TO_MM = 1.5


def pixel_x_to_offset(tag_px: float) -> float:
    """Convert tag pixel x to a lateral mm-like offset for steering."""
    return (tag_px - _CAM_CENTER_PX) * PX_TO_MM


# Horizontal FOV of the OAK-D-Lite RGB camera (IMX214).  If the tag output
# is cropped/scaled, tune this value to match the effective HFOV.
CAM_HFOV_DEG = 69.0
# Drive toward the target as long as it stays within this FOV window centered
# on the camera centerline.  Outside this window → stop and rotate to center.
TARGET_FOV_DEG = 30.0
FOV_HALF_PX = (TARGET_FOV_DEG / CAM_HFOV_DEG) * (TAG_INPUT_W / 2.0)


def _tag_in_fov(tag_px: float | None, half_px: float = FOV_HALF_PX) -> bool:
    """True if the tag's pixel-x is within the configured FOV window."""
    if tag_px is None:
        return False
    return abs(tag_px - _CAM_CENTER_PX) <= half_px


# Pixel band around camera center used to detect a wrong tag crossing through.
CENTER_BAND_PX = 40.0


def _wrong_tag_centered(tracker, target_id: int,
                        band_px: float = CENTER_BAND_PX) -> bool:
    """True if a NON-target tag is currently near the camera center."""
    for tid, px in tracker.visible_centers.items():
        if tid == target_id:
            continue
        if abs(px - _CAM_CENTER_PX) < band_px:
            return True
    return False


def _flip_dir(direction: str) -> str:
    return "left" if direction == "right" else "right"


def _bearing_to_dir(bearing: float | None, default: str = "right") -> str:
    """Pick initial rotate direction from last-known bearing sign."""
    if bearing is None or bearing == 0:
        return default
    return "right" if bearing > 0 else "left"


def _rotate_in(direction: str, speed: int):
    if direction == "right":
        turn_right(speed)
    else:
        turn_left(speed)


def _spin_search(gyro, queues, tracker, target_ids: set[int],
                 sweep_deg: float = SPIN_SWEEP_DEG) -> bool:
    """Spin in place up to *sweep_deg* scanning for any tag in target_ids.
    Returns True and stops the moment a target is seen, else finishes the
    sweep and returns False.  Uses gyro to integrate absolute yaw change.
    """
    gyro.reset()
    gyro.update(queues["imu"])
    turn_right(TURN_SPEED)
    while abs(gyro.update(queues["imu"])) < sweep_deg:
        dist_mm, _, _ = tracker.update(queues, target_ids)
        if dist_mm is not None and _tag_in_fov(tracker.tag_px):
            stop()
            return True
        time.sleep(0.05)
    stop()
    return False


# ── Speed helpers ─────────────────────────────────────────────────────────────

def compute_speed(dist_mm: float) -> int:
    """Linear ramp from CRUISE_SPEED down to APPROACH_SPEED."""
    if dist_mm >= RAMP_FAR_MM:
        return CRUISE_SPEED
    if dist_mm <= RAMP_NEAR_MM:
        return APPROACH_SPEED
    ratio = (dist_mm - RAMP_NEAR_MM) / (RAMP_FAR_MM - RAMP_NEAR_MM)
    return int(APPROACH_SPEED + ratio * (CRUISE_SPEED - APPROACH_SPEED))


TURN_AWAY_TIMEOUT_S = 6.0  # max seconds to turn away from arrived tag


# Fraction of the depth frame (centered) sampled for forward obstacle detection.
_DEPTH_ROI_FRAC = 0.4


def _depth_front_blocked(depth_queue, threshold_mm: int = OBSTACLE_MM) -> bool:
    """Read the latest stereo-depth frame; return True if the center ROI has
    any valid pixel closer than `threshold_mm`.

    Drains all queued depth frames so we always act on the freshest one.
    Returns False if no frame is available yet.
    """
    msg = None
    while True:
        m = depth_queue.tryGet()
        if m is None:
            break
        msg = m
    if msg is None:
        return False
    frame = msg.getFrame()
    if frame is None or frame.size == 0:
        return False
    h, w = frame.shape[:2]
    y1 = int(h * (0.5 - _DEPTH_ROI_FRAC / 2))
    y2 = int(h * (0.5 + _DEPTH_ROI_FRAC / 2))
    x1 = int(w * (0.5 - _DEPTH_ROI_FRAC / 2))
    x2 = int(w * (0.5 + _DEPTH_ROI_FRAC / 2))
    roi = frame[y1:y2, x1:x2]
    # Filter out zeros (invalid) and absurd values (> 10 m = noise/saturation)
    valid = roi[(roi > 0) & (roi < 10000)]
    if valid.size == 0:
        return False
    return int(valid.min()) < threshold_mm


def _front_blocked(sonar, depth_queue, threshold_mm: int = OBSTACLE_MM) -> bool:
    """Unified obstacle check: True if ultrasonic OR depth reads < threshold."""
    return sonar.front_blocked() or _depth_front_blocked(depth_queue, threshold_mm)


def _rotate_to_reacquire_tag(sonar, queues, tracker, target_id: int,
                             tag_bearing: float | None = None,
                             prefer_dir: str | None = None,
                             timeout: float = AVOID_TIMEOUT_S) -> str:
    """Rotate in place (biased toward the tag) until the front is clear AND
    the target tag is visible inside the drive-toward FOV window.

    Direction priority (highest first):
        1. tag_bearing side, if that side has clearance > OBSTACLE_MM
        2. prefer_dir (sticky from a previous rotation), if that side has
           clearance > OBSTACLE_MM — prevents ping-ponging back into the
           obstacle we just cleared
        3. The more-open side (fallback)

    Returns the direction used ("left" or "right").  Feed this back in as
    prefer_dir on the next call to stay committed.
    """
    snap = sonar.snapshot()
    left_clear = _safe(snap.get("left"))
    right_clear = _safe(snap.get("right"))
    direction = "right" if right_clear >= left_clear else "left"
    if prefer_dir in ("left", "right"):
        prefer_clear = right_clear if prefer_dir == "right" else left_clear
        if prefer_clear > OBSTACLE_MM:
            direction = prefer_dir
    if tag_bearing is not None and tag_bearing != 0:
        want = "right" if tag_bearing > 0 else "left"
        want_clear = right_clear if want == "right" else left_clear
        if want_clear > OBSTACLE_MM:
            direction = want

    print(f"  LOCKED: rotating {direction} — seeking clear path + tag in FOV")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if direction == "right":
            turn_right(AVOID_TURN)
        else:
            turn_left(AVOID_TURN)
        time.sleep(0.15)
        # tracker.update() consumes one tag+spatial frame; also drains depth
        _, _, found_id = tracker.update(queues, {target_id})
        tag_ok = (found_id == target_id) and _tag_in_fov(tracker.tag_px)
        path_clear = not _front_blocked(sonar, queues["depth"])
        if path_clear and tag_ok:
            stop()
            print("  LOCKED: clear path + tag in FOV — resuming approach")
            return direction
    stop()
    print("  LOCKED: rotate-reacquire timed out")
    return direction


def _rotate_clear(sonar, queues, gyro,
                  tag_bearing: float | None = None,
                  prefer_dir: str | None = None,
                  timeout: float = AVOID_TIMEOUT_S) -> str:
    """Rotate to clear an obstacle.

    Direction priority (highest first):
        1. tag_bearing side, if that side has clearance > OBSTACLE_MM —
           keeps the target tag in view during avoidance
        2. prefer_dir (sticky from a previous rotation), if that side has
           clearance > OBSTACLE_MM — prevents ping-ponging back into the
           obstacle we just cleared
        3. The more-open side (fallback)

    After clearing, nudges forward briefly to get past the obstacle.
    Falls back to a short backup if rotating alone doesn't clear the path.
    Returns the direction used ("left" or "right").  Feed this back in as
    prefer_dir on the next call to stay committed.
    """
    snap = sonar.snapshot()
    left_clear = _safe(snap.get("left"))
    right_clear = _safe(snap.get("right"))

    direction = "right" if right_clear >= left_clear else "left"

    if prefer_dir in ("left", "right"):
        prefer_clear = right_clear if prefer_dir == "right" else left_clear
        if prefer_clear > OBSTACLE_MM:
            direction = prefer_dir

    if tag_bearing is not None and tag_bearing != 0:
        want_right = tag_bearing > 0
        want_side_clear = right_clear if want_right else left_clear
        if want_side_clear > OBSTACLE_MM:
            direction = "right" if want_right else "left"

    print(f"  Rotating {direction} to clear obstacle "
          f"(bearing={tag_bearing}, prefer={prefer_dir})…")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if direction == "right":
            turn_right(AVOID_TURN)
        else:
            turn_left(AVOID_TURN)
        time.sleep(0.15)
        gyro.update(queues["imu"])
        queues["tags"].tryGet()
        queues["spatial"].tryGet()
        if not _front_blocked(sonar, queues["depth"]):
            stop()
            # Nudge forward slowly past the obstacle
            forward(APPROACH_SPEED)
            time.sleep(0.3)
            stop()
            return direction
    stop()
    print("  Backing up…")
    backward(BACKUP_SPEED)
    time.sleep(BACKUP_S)
    stop()
    return direction


def _turn_away(arrived_tag_id: int, queues: dict):
    """Spin slowly until the arrived-at tag is no longer visible."""
    print(f"  Turning away from tag {arrived_tag_id}…")
    turn_right(AVOID_TURN)
    deadline = time.monotonic() + TURN_AWAY_TIMEOUT_S
    while time.monotonic() < deadline:
        tag_msg = queues["tags"].get()
        # Drain spatial queue to keep it in sync
        queues["spatial"].get()
        visible_ids = {t.id for t in tag_msg.aprilTags}
        if arrived_tag_id not in visible_ids:
            break
    stop()
    print("  Tag out of view — ready to clear path")


# ── Main delivery loop ───────────────────────────────────────────────────────

def _queue_table_numbers(q) -> list[int]:
    """Extract table numbers from queue names for LCD display."""
    nums = []
    for name in q:
        num = "".join(c for c in name if c.isdigit())
        if num:
            nums.append(int(num))
    return nums


def deliver() -> None:
    # Hardware init
    IIC.set_motor_parameter()
    stop()
    bus = smbus2.SMBus(1)
    lcd = SipServeLCD(bus_number=1)
    i2c_lock = threading.Lock()
    sonar = UltrasonicPoller(bus, i2c_lock=i2c_lock)
    voice = VoicePoller(bus, i2c_lock=i2c_lock)
    tracker = TagTracker()
    gyro = GyroTracker()

    # Reverse lookup: tag ID → table name (for removing delivered tables)
    tag_id_to_name = {v: k for k, v in TAG_IDS.items() if k != "home"}

    pipeline, queues = build_delivery_pipeline()

    with pipeline:
        pipeline.start()

        # Wait for sensors before moving — don't drive blind
        sonar.wait_ready()
        # Prime the camera: first frame has no spatial data (one-frame-behind),
        # so read and discard it while stationary.
        queues["tags"].get()
        queues["spatial"].get()

        state = State.INITIALIZING
        lcd.update_state("INITIALIZING")
        target_id = TAG_IDS["home"]
        target_label = None
        last_dist_mm = None
        last_tag_bearing: float | None = None  # last known x_mm for locked tag
        delivery_queue: deque[str] = deque()
        wake_word_heard = False
        lost_entry_time: float | None = None
        # Home-bearing memory for RETURNING's "move toward last-seen home" fallback
        home_last_bearing: float | None = None
        # Scan-rotate direction for LOST and RETURNING-lost.
        search_rotate_dir: str | None = None
        # Sticky direction committed to during obstacle/tag avoidance — we stay
        # with it across consecutive rotations so the robot doesn't ping-pong
        # back into the obstacle it just cleared.  Cleared on LOCKED entry,
        # DELIVERING, and ORDERING.
        last_avoid_dir: str | None = None
        # Once we lock onto a specific delivery tag, stay committed to it.
        # SEARCHING will only accept this tag even if other queued tags are
        # also visible.  Cleared after DELIVERING pops it from the queue.
        committed_target_id: int | None = None
        # SEARCHING start time — if no lock within SEARCH_SPIN_TIMEOUT_S the
        # robot does a 360° in-place spin to scan around for queued tags.
        search_start_time: float | None = None
        # LOST wrong-tag flip: rising-edge detection + "flip at most once per
        # LOST episode" so the robot doesn't ping-pong between wrong tags.
        wrong_centered_prev: bool = False
        lost_flipped: bool = False

        try:
            while pipeline.isRunning():
                gyro.update(queues["imu"])

                # ── Voice commands — only processed during ORDERING ───────
                if state == State.ORDERING:
                    started_delivery = False
                    for cmd in voice.get_commands():
                        if cmd == VCMD_WAKE_WORD:
                            wake_word_heard = True
                            print("  Voice: wake word — ready for orders")
                            continue
                        if not wake_word_heard:
                            continue
                        table_name = VCMD_QUEUE_MAP.get(cmd)
                        if table_name:
                            if table_name not in delivery_queue:
                                delivery_queue.append(table_name)
                            lcd.update_queue(_queue_table_numbers(delivery_queue))
                            print(f"  Voice: queued {table_name} — {', '.join(delivery_queue)}")
                        elif cmd == VCMD_CLEAR_ORDERS:
                            delivery_queue.clear()
                            lcd.update_queue([])
                            print("  Voice: orders cleared")
                        elif cmd == VCMD_START_DELIVERY and delivery_queue:
                            tracker = TagTracker()
                            gyro.reset()
                            last_dist_mm = None
                            while queues["tags"].tryGet() is not None:
                                queues["spatial"].tryGet()
                            sonar.wait_ready()
                            wake_word_heard = False  # must re-wake after deliveries
                            target_label = None
                            state = State.SEARCHING
                            lcd.update_state("SEARCHING")
                            print(f"  Voice: START — delivering to {', '.join(delivery_queue)}")
                            started_delivery = True
                            break
                    if started_delivery:
                        continue

                # ── INITIALIZING: rotate in place until home tag is seen ──
                if state == State.INITIALIZING:
                    tag_msg = queues["tags"].tryGet()
                    queues["spatial"].tryGet()  # keep spatial in sync
                    home_visible = bool(tag_msg) and any(
                        t.id == TAG_IDS["home"] for t in tag_msg.aprilTags)
                    if home_visible:
                        stop()
                        print("Home tag found — say wake word to begin orders.")
                        state = State.ORDERING
                        lcd.update_state("ORDERING")
                    else:
                        turn_right(AVOID_TURN)
                        time.sleep(0.1)

                # ── ORDERING: stationary, wait for voice input ─────────────
                elif state == State.ORDERING:
                    stop()
                    queues["tags"].tryGet()
                    queues["spatial"].tryGet()
                    time.sleep(0.05)

                # ── SEARCHING: avoid obstacles, scan for any queued tag ───
                elif state == State.SEARCHING:
                    # Obstacle first — always safe before moving
                    if _front_blocked(sonar, queues["depth"]):
                        stop()
                        last_avoid_dir = _rotate_clear(
                            sonar, queues, gyro,
                            prefer_dir=last_avoid_dir)
                        continue

                    # If we've already committed to a specific delivery tag
                    # (previously locked and lost), only that tag can re-lock us.
                    if committed_target_id is not None:
                        search_ids = {committed_target_id}
                    else:
                        search_ids = {TAG_IDS[n] for n in delivery_queue}

                    # Start the 10s search-timeout clock on first tick in SEARCHING
                    if search_start_time is None:
                        search_start_time = time.monotonic()

                    # 10s with no lock → 360° in-place spin to scan for a tag
                    if time.monotonic() - search_start_time > SEARCH_SPIN_TIMEOUT_S:
                        stop()
                        print("  SEARCH: no lock in 10s — 360° scan spin")
                        found = _spin_search(gyro, queues, tracker, search_ids)
                        search_start_time = time.monotonic()
                        if found:
                            # Promote to LOCKED immediately on next tick
                            continue
                        continue

                    dist_mm, x_mm, found_id = tracker.update(queues, search_ids)

                    if dist_mm is not None and _tag_in_fov(tracker.tag_px):
                        target_id = found_id
                        target_label = tag_id_to_name.get(target_id, f"tag{target_id}")
                        last_dist_mm = dist_mm
                        last_tag_bearing = x_mm
                        lost_entry_time = None
                        last_avoid_dir = None
                        committed_target_id = target_id
                        search_start_time = None
                        state = State.LOCKED
                        lcd.update_state("LOCKED", target_label)
                        print(f"  {target_label} (tag {target_id}) locked at {dist_mm:.0f}mm")
                        continue

                    # No queued tag near center — keep roaming forward
                    forward(ROAM_SPEED)

                # ── LOCKED: center + approach with speed ramp ─────────────
                elif state == State.LOCKED:
                    # Obstacle priority: wait, then rotate to reacquire a clear
                    # path WHILE keeping the target tag in FOV.
                    if _front_blocked(sonar, queues["depth"]):
                        stop()
                        last_avoid_dir = _rotate_to_reacquire_tag(
                            sonar, queues, tracker, target_id,
                            tag_bearing=last_tag_bearing,
                            prefer_dir=last_avoid_dir)
                        continue

                    dist_mm, x_mm, _ = tracker.update(queues, {target_id})

                    # Tag visible anywhere → reset lost-grace timer
                    if dist_mm is not None or tracker.tag_px is not None:
                        lost_entry_time = None

                    # ── Tag fully lost: coast forward briefly, then go LOST ─
                    if dist_mm is None and tracker.tag_px is None:
                        if lost_entry_time is None:
                            lost_entry_time = time.monotonic()
                        elapsed = time.monotonic() - lost_entry_time
                        if elapsed < TAG_LOST_FORWARD_GRACE_S:
                            speed = (APPROACH_SPEED
                                     if last_dist_mm is not None and last_dist_mm < SLOW_DIST_MM
                                     else CRUISE_SPEED)
                            bearing = last_tag_bearing if last_tag_bearing is not None else 0.0
                            arc_drive(speed, bearing)
                            print(f"  LOCK tag lost {elapsed:.1f}s — coasting forward")
                            continue
                        # Enter LOST rotating OPPOSITE the side the tag was
                        # last on — covers the over-shoot case where the robot
                        # arced past the tag and needs to swing back.
                        bearing_dir = _bearing_to_dir(last_tag_bearing, "right")
                        search_rotate_dir = _flip_dir(bearing_dir)
                        wrong_centered_prev = False
                        lost_flipped = False
                        state = State.LOST
                        lcd.update_state("LOST", target_label)
                        print(f"  LOCKED → LOST ({target_label}) — rotating {search_rotate_dir}")
                        continue

                    # ── Tag seen in pixels but no depth: pixel-steer forward ─
                    if dist_mm is None:
                        px_offset = pixel_x_to_offset(tracker.tag_px)
                        last_tag_bearing = px_offset
                        speed = (APPROACH_SPEED
                                 if last_dist_mm is not None and last_dist_mm < SLOW_DIST_MM
                                 else CRUISE_SPEED)
                        arc_drive(speed, px_offset)
                        continue

                    last_dist_mm = dist_mm
                    last_tag_bearing = x_mm

                    # Stop at destination
                    if dist_mm <= STOP_DIST_MM:
                        stop()
                        last_avoid_dir = None
                        state = State.DELIVERING
                        lcd.update_state("DELIVERING", target_label)
                        print(f"  Arrived at {target_label} ({dist_mm:.0f}mm)")
                        continue

                    # Drive forward while steering toward the tag, regardless
                    # of where it is in the camera frame.  arc_drive clamps
                    # the steer so one tread idles at full offset — a tight
                    # forward arc — instead of spinning in place.
                    speed = compute_speed(dist_mm)
                    if sonar.min_front_dist() < SLOW_DIST_MM:
                        speed = min(speed, APPROACH_SPEED)
                    arc_drive(speed, x_mm)
                    print(f"  LOCK dist={dist_mm:.0f}mm x={x_mm:+.0f}mm speed={speed}")

                # ── LOST: rotate to reacquire target; flip dir on wrong-center
                elif state == State.LOST:
                    if _front_blocked(sonar, queues["depth"]):
                        stop()
                        last_avoid_dir = _rotate_clear(
                            sonar, queues, gyro,
                            tag_bearing=last_tag_bearing,
                            prefer_dir=last_avoid_dir)
                        continue

                    dist_mm, x_mm, _ = tracker.update(queues, {target_id})
                    if (dist_mm is not None or tracker.tag_px is not None) \
                            and _tag_in_fov(tracker.tag_px):
                        lost_entry_time = None
                        search_rotate_dir = None
                        last_avoid_dir = None
                        state = State.LOCKED
                        lcd.update_state("LOCKED", target_label)
                        print(f"  LOST → LOCKED ({target_label})")
                        continue

                    if lost_entry_time is None:
                        lost_entry_time = time.monotonic()
                    if time.monotonic() - lost_entry_time > TAG_LOST_TIMEOUT_S:
                        lost_entry_time = None
                        search_rotate_dir = None
                        wrong_centered_prev = False
                        lost_flipped = False
                        search_start_time = None
                        state = State.SEARCHING
                        lcd.update_state("SEARCHING")
                        print("  LOST → SEARCHING (timeout)")
                        continue

                    # Rotate in place to search for the target.  Direction was
                    # seeded on LOCKED → LOST entry (opposite the bearing side).
                    if search_rotate_dir is None:
                        bearing_dir = _bearing_to_dir(last_tag_bearing, "right")
                        search_rotate_dir = _flip_dir(bearing_dir)

                    # If a non-target tag crosses the camera center, flip
                    # direction — but only once per LOST episode so we don't
                    # ping-pong between multiple wrong tags.
                    wrong_now = _wrong_tag_centered(tracker, target_id)
                    if wrong_now and not wrong_centered_prev and not lost_flipped:
                        search_rotate_dir = _flip_dir(search_rotate_dir)
                        lost_flipped = True
                        print(f"  LOST: wrong tag centered — flipping to {search_rotate_dir}")
                    wrong_centered_prev = wrong_now

                    _rotate_in(search_rotate_dir, TURN_SPEED)
                    time.sleep(0.1)

                # ── DELIVERING: 6s pause, unqueue, route to next ──────────
                elif state == State.DELIVERING:
                    stop()
                    delivered_name = tag_id_to_name.get(target_id)
                    print(f"\n  Delivering to {delivered_name} — pausing {DELIVER_PAUSE_S:.0f}s")
                    time.sleep(DELIVER_PAUSE_S)
                    if delivered_name and delivered_name in delivery_queue:
                        delivery_queue.remove(delivered_name)
                    committed_target_id = None
                    lcd.update_queue(_queue_table_numbers(delivery_queue))

                    _turn_away(target_id, queues)
                    tracker = TagTracker()
                    last_dist_mm = None
                    last_tag_bearing = None
                    sonar.wait_ready()

                    if delivery_queue:
                        target_label = None
                        state = State.SEARCHING
                        lcd.update_state("SEARCHING")
                        print(f"  Remaining: {', '.join(delivery_queue)}")
                    else:
                        target_label = "home"
                        target_id = TAG_IDS["home"]
                        home_last_bearing = None
                        search_rotate_dir = None
                        state = State.RETURNING
                        lcd.update_state("RETURNING", target_label)
                        print("  All tables delivered — heading home…")

                # ── RETURNING: navigate to home, back to ORDERING ─────────
                elif state == State.RETURNING:
                    # Obstacle first — rotate to keep line-of-sight to home
                    if _front_blocked(sonar, queues["depth"]):
                        stop()
                        last_avoid_dir = _rotate_to_reacquire_tag(
                            sonar, queues, tracker, TAG_IDS["home"],
                            tag_bearing=home_last_bearing,
                            prefer_dir=last_avoid_dir)
                        continue

                    home_id = TAG_IDS["home"]
                    dist_mm, x_mm, _ = tracker.update(queues, {home_id})

                    # Arrived at home
                    if dist_mm is not None and dist_mm <= STOP_DIST_MM:
                        stop()
                        print("\n  Returned home! Say wake word for new orders.")
                        wake_word_heard = False
                        target_label = None
                        home_last_bearing = None
                        lcd.update_queue([])
                        state = State.ORDERING
                        lcd.update_state("ORDERING")
                        continue

                    # Home visible with depth — normal approach, reset scan state
                    if dist_mm is not None:
                        home_last_bearing = x_mm
                        search_rotate_dir = None
                        if not _tag_in_fov(tracker.tag_px):
                            if tracker.tag_px is not None and tracker.tag_px > _CAM_CENTER_PX:
                                turn_right(TURN_SPEED)
                            else:
                                turn_left(TURN_SPEED)
                            print(f"  RETURN out-of-FOV (px={tracker.tag_px}) — rotating")
                            continue
                        speed = compute_speed(dist_mm)
                        if sonar.min_front_dist() < SLOW_DIST_MM:
                            speed = min(speed, APPROACH_SPEED)
                        arc_drive(speed, x_mm)
                        print(f"  RETURN dist={dist_mm:.0f}mm x={x_mm:+.0f}mm speed={speed}")
                        continue

                    # Home visible in pixels but no depth yet — pixel steer,
                    # gated by the drive-toward FOV window.
                    if tracker.tag_px is not None:
                        px_offset = pixel_x_to_offset(tracker.tag_px)
                        home_last_bearing = px_offset
                        search_rotate_dir = None
                        if not _tag_in_fov(tracker.tag_px):
                            if px_offset > 0:
                                turn_right(TURN_SPEED)
                            else:
                                turn_left(TURN_SPEED)
                            print(f"  RETURN out-of-FOV (px={tracker.tag_px:.0f}) — rotating")
                            continue
                        arc_drive(CRUISE_SPEED, px_offset)
                        continue

                    # Home NOT visible — rotate in place to scan, biased toward
                    # last-seen home.  Commit to that direction for the whole
                    # scan; flipping on a non-home tag just ping-pongs back
                    # into whatever we already rotated away from.
                    if search_rotate_dir is None:
                        search_rotate_dir = _bearing_to_dir(home_last_bearing, "right")

                    _rotate_in(search_rotate_dir, TURN_SPEED)
                    time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nAborted by user.")
        finally:
            stop()
            voice.stop()
            sonar.stop()
            lcd.clear()
            pipeline.stop()
            bus.close()
            print("Motors stopped, cleanup done.")


def main():
    deliver()


if __name__ == "__main__":
    main()
