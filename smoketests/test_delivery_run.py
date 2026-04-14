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
import numpy as np
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

# ── Tuning ────────────────────────────────────────────────────────────────────

IMU_RATE_HZ = 100

# Speeds (PWM magnitude — sent as negative for forward, positive for backward).
# NOTE: this file uses raw IIC.control_pwm with a ±1000-ish range. The newer
# hardware/motor.Motor class uses ±7200. Don't mix values between the two.
ROAM_SPEED     = 800   # cruising while searching for tag
CRUISE_SPEED   = 500   # driving toward a visible tag, far away
APPROACH_SPEED = 250   # close to tag, fine control
TURN_SPEED     = 300   # centering corrections during NAVIGATE
SMOOTH_TURN_SPEED = 270   # 10% slower than TURN_SPEED — used only for the
                          # continuous "find clear path / find home" rotations
                          # so the camera has better chance to detect tags
                          # during a non-bursted rotation.
AVOID_TURN     = 300   # turning away from obstacles
BACKUP_SPEED   = 300   # reversing away from an obstacle

# Distance thresholds (mm)
OBSTACLE_MM      = 203    # ~8 inches — obstacle trigger (ultrasonic + depth)
CLEAR_PATH_MM    = 610    # ~24 inches (2 ft) — center ultrasonic must read this
                          # far before SEARCHING begins forward motion
DEPTH_CLEAR_PATH_MM = 1219  # 48 inches (4 ft) — depth-based escape for the
                            # clear-path rotation; a depth ROI this clear is
                            # sufficient evidence that forward motion is safe.
SLOW_DIST_MM     = 508    # 20 inches — proximity ramp near obstacles
STOP_DIST_MM     = 305    # 12 inches — front ultrasonic arrival threshold
CENTER_TOL_PX    = 30     # pixel tolerance for "tag centered" in LOCKED

# Timing
SEARCH_SPIN_TIMEOUT_S = 10.0  # SEARCHING: 10s with no lock → 360° spin
POST_SPIN_SCAN_S      = 5.0   # After a failed spin, roam forward for this
                              # long before triggering another spin. Short
                              # forward scan lets the robot change position
                              # in case the target tag is behind an occluder
                              # from the current spot.
SPIN_SWEEP_DEG     = 360.0  # how far to spin when scanning for a tag
DELIVER_PAUSE_S    = 6.0   # seconds to pause at destination for delivery
AVOID_TIMEOUT_S    = 4.0   # max seconds turning during obstacle avoidance
BACKUP_S           = 0.5   # seconds to reverse if turn times out

# SEARCHING clear-path rotation: max time to wait for 2 ft of center clearance
# before giving up and proceeding (prevents infinite spin in a tight corridor).
SEARCH_CLEAR_PATH_TIMEOUT_S = 8.0

# Burst-rotate timing: short turn bursts with pauses to avoid motion blur
# on the AprilTag detector (camera needs a still frame to detect reliably).
SEARCH_ROTATE_BURST_S = 0.3   # rotate this long per burst
SEARCH_PAUSE_S        = 0.2   # pause after each burst to let camera settle


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
    VCMD_QUEUE_TABLE1: "Table 1",
    VCMD_QUEUE_TABLE2: "Table 2",
    VCMD_QUEUE_TABLE3: "Table 3",
    VCMD_QUEUE_TABLE4: "Table 4",
}


class State(enum.Enum):
    INITIALIZING = "INITIALIZING"  # rotate until home tag found
    ORDERING     = "ORDERING"      # wait for wake word, then queue tables
    SEARCHING    = "SEARCHING"     # clear path, search for queued tags
    LOCKED       = "LOCKED"        # approaching a found tag
    DELIVERING   = "DELIVERING"    # paused at tag, delivery in progress
    RETURNING    = "RETURNING"     # heading home after all deliveries


# ── Motor helpers ─────────────────────────────────────────────────────────────
# Negative PWM = forward on this robot (wiring is inverted).
# M1 = right tread, M3 = left tread.

# Hard safety floor: if any front-facing ultrasonic reads < OBSTACLE_MM (8"),
# forward() refuses to move. `stop()`, `backward()`, and turns are always
# allowed so the FSM can still back away or rotate to clear.
# `deliver()` registers the live UltrasonicPoller via set_safety_sonar().
_safety_sonar = None
_safety_tripped = False   # rising-edge flag for logging

def set_safety_sonar(sonar) -> None:
    """Register an UltrasonicPoller as the hard-stop safety source."""
    global _safety_sonar
    _safety_sonar = sonar

def _safety_halt() -> bool:
    """True if forward motion must be blocked. Logs once per rising edge."""
    global _safety_tripped
    tripped = _safety_sonar is not None and _safety_sonar.front_blocked()
    if tripped and not _safety_tripped:
        print("  [SAFETY] Front ultrasonic <8\" — forward halted")
        _safety_tripped = True
    elif not tripped and _safety_tripped:
        _safety_tripped = False
    return tripped

def stop():
    IIC.control_pwm(0, 0, 0, 0)

def forward(speed=ROAM_SPEED):
    if _safety_halt():
        IIC.control_pwm(0, 0, 0, 0)
        return
    IIC.control_pwm(-speed, 0, -speed, 0)

def backward(speed=BACKUP_SPEED):
    IIC.control_pwm(speed, 0, speed, 0)

def turn_left(speed=TURN_SPEED):
    IIC.control_pwm(-speed, 0, speed, 0)

def turn_right(speed=TURN_SPEED):
    IIC.control_pwm(speed, 0, -speed, 0)


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
                        self._mux.invalidate()
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
        # Cache of the most recent non-empty update() result. The main loop
        # ticks MUCH faster than the camera produces frames, so `tryGet()`
        # returns None on most ticks. Without caching, the FSM would see
        # "tag lost" every no-frame tick and flip state constantly.
        self._cached_dist: float | None = None
        self._cached_x: float | None = None
        self._cached_found: int | None = None

    def update(self, queues, target_ids) -> tuple[float | None, float | None, int | None]:
        """Read one camera frame and return (dist_mm, x_mm, found_id).

        target_ids: a set (or single int) of tag IDs to match.
        When no new camera frame is available this tick, returns the last
        frame's cached result and leaves instance state (tag_px, visible_*)
        unchanged — so the FSM sees tag state update at camera rate (~30 Hz),
        not main-loop rate (kHz). Prevents "tag lost" false positives.
        Returns (None, None, None) when a frame DID arrive but no target
        tag was in it, or when no frame has ever arrived yet.
        """
        if isinstance(target_ids, int):
            target_ids = {target_ids}

        # Non-blocking: on a camera hiccup, return "no new data" sentinels so
        # the FSM keeps ticking instead of freezing on a blocking .get().
        tag_msg = queues["tags"].tryGet()
        if tag_msg is None:
            # No new frame — preserve instance state, return cached result.
            return self._cached_dist, self._cached_x, self._cached_found

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

        # Spatial data corresponds to the *previous* frame's ROI. Only trust it
        # if this frame's target is the same tag id we sent ROI for last frame;
        # otherwise depth could be attributed to the wrong tag.
        spatial_msg = queues["spatial"].tryGet()
        spatial_locs = spatial_msg.getSpatialLocations() if spatial_msg else []

        dist_mm = None
        x_mm = None
        if (self._prev_tag is not None and spatial_locs
                and found_id is not None and found_id == self._prev_tag.id):
            loc = spatial_locs[0]
            z = loc.spatialCoordinates.z
            if z > 0:
                dist_mm = z
                x_mm = loc.spatialCoordinates.x

        self._prev_tag = target_tags[0] if target_tags else None
        # Cache so no-frame ticks can return a consistent result.
        self._cached_dist = dist_mm
        self._cached_x = x_mm
        self._cached_found = found_id
        return dist_mm, x_mm, found_id


# Camera center x in pixels — offset from this = how far off-center the tag is
_CAM_CENTER_PX = TAG_INPUT_W / 2.0

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


def _burst_rotate(direction: str, speed: int):
    """Rotate in a short burst then pause to let the camera settle.

    This avoids motion blur on the AprilTag detector — the camera needs
    a still frame to detect tags reliably.
    """
    _rotate_in(direction, speed)
    time.sleep(SEARCH_ROTATE_BURST_S)
    stop()
    time.sleep(SEARCH_PAUSE_S)


def _spin_search(gyro, queues, tracker, target_ids: set[int],
                 sweep_deg: float = SPIN_SWEEP_DEG) -> bool:
    """Spin in place up to *sweep_deg* scanning for any tag in target_ids.
    Uses burst-rotate pattern to avoid motion blur on the AprilTag detector.
    Returns True and stops the moment a target is seen INSIDE the drive-toward
    FOV window (_tag_in_fov). Stopping on an edge-of-frame detection leads to
    over-rotation cascades later when LOCKED commits to an approach with the
    tag already near the periphery of the camera.
    Returns False if the full sweep completes without a centered target.
    Uses gyro to integrate absolute yaw change.
    """
    gyro.reset()
    gyro.update(queues["imu"])
    while abs(gyro.update(queues["imu"])) < sweep_deg:
        # Burst: rotate briefly
        turn_right(TURN_SPEED)
        time.sleep(SEARCH_ROTATE_BURST_S)
        stop()
        # Pause: let camera settle and grab a frame
        time.sleep(SEARCH_PAUSE_S)
        gyro.update(queues["imu"])
        # Check for target tag during the still moment — require it to be in
        # the drive-toward FOV window, not merely visible at the edge.
        dist_mm, _, found_id = tracker.update(queues, target_ids)
        if (found_id is not None and found_id in target_ids
                and _tag_in_fov(tracker.tag_px)):
            stop()
            return True
    stop()
    return False


# ── Speed helpers ─────────────────────────────────────────────────────────────

def _proximity_speed(near_mm: int, max_speed: int) -> int:
    """Scale a forward speed down as an obstacle gets closer.

    Returns `max_speed` when clear (>= SLOW_DIST_MM), `APPROACH_SPEED` when
    at the OBSTACLE_MM floor, and a linear interpolation in between.
    Used for SEARCHING roaming where the thing in front is unknown.
    """
    if near_mm >= SLOW_DIST_MM:
        return max_speed
    if near_mm <= OBSTACLE_MM:
        return APPROACH_SPEED
    ratio = (near_mm - OBSTACLE_MM) / (SLOW_DIST_MM - OBSTACLE_MM)
    return int(APPROACH_SPEED + ratio * (max_speed - APPROACH_SPEED))


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


def _depth_front_roi(depth_queue):
    """Grab the newest depth frame's center ROI. Returns a 1-D numpy array of
    valid pixels (in mm), or an empty array if no data. Drains the queue so
    we always act on the freshest frame."""
    msg = None
    while True:
        m = depth_queue.tryGet()
        if m is None:
            break
        msg = m
    if msg is None:
        return np.empty(0, dtype=np.int32)
    frame = msg.getFrame()
    if frame is None or frame.size == 0:
        return np.empty(0, dtype=np.int32)
    h, w = frame.shape[:2]
    y1 = int(h * (0.5 - _DEPTH_ROI_FRAC / 2))
    y2 = int(h * (0.5 + _DEPTH_ROI_FRAC / 2))
    x1 = int(w * (0.5 - _DEPTH_ROI_FRAC / 2))
    x2 = int(w * (0.5 + _DEPTH_ROI_FRAC / 2))
    roi = frame[y1:y2, x1:x2]
    return roi[(roi > 0) & (roi < 10000)]


def _depth_front_min_mm(depth_queue) -> int:
    """Nearest valid mm in the center depth ROI (99999 if no valid frame).

    Sensitive — used for obstacle detection and proximity slowdown where we
    want any close pixel to count. For clear-path checks use
    `_depth_front_clear_mm` instead, which tolerates a few noisy pixels.
    """
    valid = _depth_front_roi(depth_queue)
    return int(valid.min()) if valid.size else 99999


def _depth_front_clear_mm(depth_queue, percentile: float = 10.0) -> int:
    """Robust "nearest real obstacle" in the depth ROI — the Nth-percentile
    distance instead of the absolute minimum, so occasional noise pixels
    (stereo artifacts, low-texture dropouts, floor edges) don't disqualify
    an otherwise-clear path. Returns 99999 when no usable data.

    Default percentile=10: up to 10% of valid ROI pixels can be noise before
    the clear-path check is defeated.
    """
    valid = _depth_front_roi(depth_queue)
    # Require at least a reasonable fraction of the ROI to be valid — if the
    # whole depth frame is black (shadow, reflection, out-of-range) we can't
    # make claims about clearance.
    if valid.size < 100:
        return 99999
    return int(np.percentile(valid, percentile))


def _front_blocked(sonar, depth_queue, threshold_mm: int = OBSTACLE_MM) -> bool:
    """Unified obstacle check: True if ultrasonic OR depth reads < threshold."""
    return sonar.front_blocked() or _depth_front_blocked(depth_queue, threshold_mm)


def _rotate_clear(sonar, queues, gyro,
                  tag_bearing: float | None = None,
                  prefer_dir: str | None = None,
                  timeout: float = AVOID_TIMEOUT_S) -> str:
    """Rotate to clear an obstacle.

    Uses burst-rotate pattern to avoid motion blur on the AprilTag detector.

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
        # Burst: rotate briefly
        _rotate_in(direction, AVOID_TURN)
        time.sleep(SEARCH_ROTATE_BURST_S)
        stop()
        # Pause: let camera settle and check
        time.sleep(SEARCH_PAUSE_S)
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


POST_DELIVER_FORWARD_S = 2.0  # max forward-drive time after 180° turn-away


def _turn_until_front_clear(sonar, queues, gyro, tracker, search_ids):
    """Reorient after a delivery.

    1. Rotate 180° from the delivered heading (burst-rotate right, gyro-tracked).
    2. After the 180°, check the new forward view:
         - If front is NOT clear → fall back to burst-rotating right until
           no front obstacle is detected.
         - If front IS clear AND a queued AprilTag is visible in FOV → stop,
           let the next SEARCHING tick pick up the lock.
         - If front IS clear and no queued tag is visible → drive forward for
           up to POST_DELIVER_FORWARD_S, bailing early on a tag sighting or
           a new front obstacle.
    """
    print("  Turning 180° away from delivered tag…")
    gyro.reset()
    gyro.update(queues["imu"])
    while abs(gyro.update(queues["imu"])) < 180.0:
        _burst_rotate("right", TURN_SPEED)
        gyro.update(queues["imu"])
    stop()

    tracker.update(queues, search_ids)
    tag_in_view = _tag_in_fov(tracker.tag_px)

    if _front_blocked(sonar, queues["depth"]):
        print("  Front still blocked after 180° — burst-rotating to clear")
        deadline = time.monotonic() + TURN_AWAY_TIMEOUT_S
        while time.monotonic() < deadline:
            if not _front_blocked(sonar, queues["depth"]):
                stop()
                print("  Front clear — ready to resume")
                return
            _burst_rotate("right", AVOID_TURN)
        stop()
        print("  Turn-away timed out — front still blocked")
        return

    if tag_in_view:
        print("  Queued tag visible after 180° — holding for SEARCHING lock")
        return

    print(f"  Front clear — driving forward up to {POST_DELIVER_FORWARD_S:.1f}s")
    deadline = time.monotonic() + POST_DELIVER_FORWARD_S
    while time.monotonic() < deadline:
        if _front_blocked(sonar, queues["depth"]):
            break
        tracker.update(queues, search_ids)
        if _tag_in_fov(tracker.tag_px):
            break
        forward(APPROACH_SPEED)
        time.sleep(0.1)
    stop()


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
    set_safety_sonar(sonar)  # hard-stop forward() if front <8"
    voice = VoicePoller(bus, i2c_lock=i2c_lock)
    tracker = TagTracker()
    gyro = GyroTracker()

    # Reverse lookup: tag ID → table name (for removing delivered tables)
    tag_id_to_name = {v: k for k, v in TAG_IDS.items() if k != "Home"}

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
        target_id = TAG_IDS["Home"]
        target_label = None
        delivery_queue: deque[str] = deque()
        wake_word_heard = False
        # Home-bearing memory for RETURNING's "move toward last-seen home" fallback
        home_last_bearing: float | None = None
        # Scan-rotate direction for RETURNING when home is not yet visible.
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
        # Has the robot already done a 360° spin in this SEARCHING episode?
        # If yes, subsequent spins trigger after POST_SPIN_SCAN_S (shorter)
        # instead of SEARCH_SPIN_TIMEOUT_S. Reset on every fresh SEARCHING
        # entry and on successful tag lock-on.
        did_spin: bool = False
        # SEARCHING: robot starts facing home — must rotate until the center
        # ultrasonic reads >= 2 ft before driving forward.
        search_path_clear: bool = False
        # Start time of the clear-path rotation phase, for timeout fallback.
        clear_path_start: float | None = None

        try:
            while pipeline.isRunning():
                gyro.update(queues["imu"])

                # ── Voice commands — only processed during ORDERING ───────
                # Wake word gates table-queuing only. CLEAR and START work at
                # any time so the operator can always cancel or launch.
                if state == State.ORDERING:
                    started_delivery = False
                    for cmd in voice.get_commands():
                        if cmd == VCMD_WAKE_WORD:
                            wake_word_heard = True
                            print("  Voice: wake word — ready for orders")
                            continue
                        table_name = VCMD_QUEUE_MAP.get(cmd)
                        if table_name:
                            if not wake_word_heard:
                                continue
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
                            # Drain each queue independently so they don't desync
                            while queues["tags"].tryGet() is not None:
                                pass
                            while queues["spatial"].tryGet() is not None:
                                pass
                            sonar.wait_ready()
                            wake_word_heard = False  # must re-wake after deliveries
                            target_label = None
                            search_path_clear = False
                            clear_path_start = None
                            search_start_time = None
                            did_spin = False
                            state = State.SEARCHING
                            lcd.update_state("SEARCHING")
                            print(f"  Voice: START — delivering to {', '.join(delivery_queue)}")
                            started_delivery = True
                            break
                    if started_delivery:
                        continue

                # ── INITIALIZING: burst-rotate until home tag is seen ─────
                if state == State.INITIALIZING:
                    tag_msg = queues["tags"].tryGet()
                    queues["spatial"].tryGet()  # keep spatial in sync
                    home_visible = bool(tag_msg) and any(
                        t.id == TAG_IDS["Home"] for t in tag_msg.aprilTags)
                    if home_visible:
                        stop()
                        print("Home tag found — say wake word to begin orders.")
                        state = State.ORDERING
                        lcd.update_state("ORDERING")
                    else:
                        _burst_rotate("right", AVOID_TURN)

                # ── ORDERING: stationary, wait for voice input ─────────────
                elif state == State.ORDERING:
                    stop()
                    queues["tags"].tryGet()
                    queues["spatial"].tryGet()
                    time.sleep(0.05)

                # ── SEARCHING: avoid obstacles, scan for any queued tag ───
                elif state == State.SEARCHING:
                    # After starting from ORDERING or finishing a delivery the
                    # robot is facing the home/delivered tag.  Rotate until the
                    # center ultrasonic reads >= 2 ft of clear space — but if a
                    # queued delivery tag is spotted during the rotation, abandon
                    # the clear-path search and lock on immediately.
                    if not search_path_clear:
                        if clear_path_start is None:
                            clear_path_start = time.monotonic()
                        # Check if a queued tag is visible during the rotation
                        if committed_target_id is not None:
                            rotate_search_ids = {committed_target_id}
                        else:
                            rotate_search_ids = {TAG_IDS[n] for n in delivery_queue}
                        dist_mm, x_mm, found_id = tracker.update(queues, rotate_search_ids)
                        if dist_mm is not None and _tag_in_fov(tracker.tag_px):
                            search_path_clear = True
                            clear_path_start = None
                            target_id = found_id
                            target_label = tag_id_to_name.get(target_id, f"tag{target_id}")
                            last_avoid_dir = None
                            committed_target_id = target_id
                            search_start_time = None
                            did_spin = False
                            state = State.LOCKED
                            lcd.update_state("LOCKED", target_label)
                            print(f"  SEARCH (rotating): spotted {target_label} at {dist_mm:.0f}mm — locking on")
                            continue

                        # Timeout fallback — don't spin forever in a tight space.
                        # Hand off to _rotate_clear so we actively resolve the
                        # near obstacle instead of declaring the path clear and
                        # driving into whatever is in front.
                        if time.monotonic() - clear_path_start > SEARCH_CLEAR_PATH_TIMEOUT_S:
                            print(f"  SEARCH: clear-path rotation timed out "
                                  f"(>{SEARCH_CLEAR_PATH_TIMEOUT_S:.0f}s) — handing off to _rotate_clear")
                            last_avoid_dir = _rotate_clear(
                                sonar, queues, gyro,
                                prefer_dir=last_avoid_dir)
                            search_path_clear = True
                            clear_path_start = None
                            continue

                        # Clear-path escape: EITHER center ultrasonic reads
                        # ≥ 2 ft, OR depth ROI reports ≥ 4 ft of clearance.
                        # Depth catches the "far-open scene" case the narrow
                        # ultrasonic beam can miss; ultrasonic catches close
                        # low-texture / glass obstacles depth misses.
                        # Depth check uses 10th-percentile (robust) so a few
                        # noise pixels don't defeat an otherwise-clear frame.
                        snap = sonar.snapshot()
                        center_dist = _safe(snap.get("front"))
                        near_depth = _depth_front_clear_mm(queues["depth"])
                        ultrasonic_clear = center_dist >= CLEAR_PATH_MM
                        depth_clear = (near_depth < 99999
                                       and near_depth >= DEPTH_CLEAR_PATH_MM)
                        if ultrasonic_clear or depth_clear:
                            search_path_clear = True
                            clear_path_start = None
                            reason = "ultrasonic" if ultrasonic_clear else "depth"
                            print(f"  SEARCH: clear path found via {reason} "
                                  f"(front={center_dist}mm depth={near_depth}mm)")
                            # Fall through next tick — avoid double tracker.update
                            continue
                        # Smooth continuous rotation (not burst): this phase is
                        # pivoting away from a tag we're sitting on top of — we
                        # want to find clearance fast, not accumulate still-frame
                        # tag detections. The tag-spotted escape path above still
                        # runs every tick so we can lock on mid-rotation.
                        turn_right(SMOOTH_TURN_SPEED)
                        continue

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

                    # Start the search-timeout clock on first tick in SEARCHING
                    if search_start_time is None:
                        search_start_time = time.monotonic()

                    # Timeout before next 360° spin:
                    #   - SEARCH_SPIN_TIMEOUT_S (10s) on the first spin of this
                    #     SEARCHING episode, while roaming forward from entry.
                    #   - POST_SPIN_SCAN_S (5s) between subsequent spins, so
                    #     after a failed spin the robot moves forward briefly
                    #     (scanning as it goes) before trying another spin.
                    #   If a tag enters the FOV during either phase, the
                    #   tracker.update check below locks on and abandons the
                    #   timer entirely.
                    current_spin_timeout = (POST_SPIN_SCAN_S if did_spin
                                            else SEARCH_SPIN_TIMEOUT_S)
                    if time.monotonic() - search_start_time > current_spin_timeout:
                        stop()
                        print(f"  SEARCH: no lock in {current_spin_timeout:.0f}s"
                              f" — 360° scan spin")
                        found = _spin_search(gyro, queues, tracker, search_ids)
                        search_start_time = time.monotonic()
                        did_spin = True
                        if found:
                            # Promote to LOCKED immediately on next tick
                            continue
                        continue

                    dist_mm, x_mm, found_id = tracker.update(queues, search_ids)

                    if dist_mm is not None and _tag_in_fov(tracker.tag_px):
                        target_id = found_id
                        target_label = tag_id_to_name.get(target_id, f"tag{target_id}")
                        last_avoid_dir = None
                        committed_target_id = target_id
                        search_start_time = None
                        did_spin = False
                        state = State.LOCKED
                        lcd.update_state("LOCKED", target_label)
                        print(f"  {target_label} (tag {target_id}) locked at {dist_mm:.0f}mm")
                        continue

                    # No queued tag near center — keep roaming forward.
                    # Speed ramps down proportionally to the nearest obstacle
                    # (ultrasonic OR depth), so the robot decelerates smoothly
                    # as things get closer rather than flipping binary.
                    near_mm = min(sonar.min_front_dist(),
                                  _depth_front_min_mm(queues["depth"]))
                    forward(_proximity_speed(near_mm, ROAM_SPEED))

                # ── LOCKED: center tag in frame, then drive forward ───────
                # Arrival fires when the front center ultrasonic reads
                # ≤ STOP_DIST_MM (12"). The target tag itself is what trips
                # the ultrasonic, so no separate tag-distance check is needed.
                elif state == State.LOCKED:
                    snap = sonar.snapshot()
                    front_mm = _safe(snap.get("front"))
                    if front_mm <= STOP_DIST_MM:
                        stop()
                        last_avoid_dir = None
                        state = State.DELIVERING
                        lcd.update_state("DELIVERING", target_label)
                        print(f"  Arrived at {target_label} "
                              f"(front ultrasonic {front_mm}mm)")
                        continue

                    tracker.update(queues, {target_id})
                    tag_px = tracker.tag_px

                    if tag_px is not None:
                        offset_px = tag_px - _CAM_CENTER_PX
                        if abs(offset_px) > CENTER_TOL_PX:
                            direction = "right" if offset_px > 0 else "left"
                            _burst_rotate(direction, TURN_SPEED)
                            continue

                    # Tag centered, OR no fresh tag data this tick — drive forward.
                    # Once centered we commit to the heading; a brief loss of
                    # the tag shouldn't stop forward progress toward arrival.
                    forward(APPROACH_SPEED)

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

                    # Next phase's search targets: remaining queue, or Home.
                    if delivery_queue:
                        post_deliver_ids = {TAG_IDS[n] for n in delivery_queue}
                    else:
                        post_deliver_ids = {TAG_IDS["Home"]}
                    _turn_until_front_clear(sonar, queues, gyro, tracker,
                                            post_deliver_ids)
                    tracker = TagTracker()
                    sonar.wait_ready()

                    if delivery_queue:
                        target_label = None
                        search_path_clear = False
                        clear_path_start = None
                        search_start_time = None
                        did_spin = False
                        state = State.SEARCHING
                        lcd.update_state("SEARCHING")
                        print(f"  Remaining: {', '.join(delivery_queue)}")
                    else:
                        target_label = "Home"
                        target_id = TAG_IDS["Home"]
                        home_last_bearing = None
                        search_rotate_dir = None
                        state = State.RETURNING
                        lcd.update_state("RETURNING", target_label)
                        print("  All tables delivered — heading home…")

                # ── RETURNING: find home tag, center, drive forward ───────
                # Same logic as LOCKED but with the home tag as target:
                #   1. If home not visible → burst-rotate to search.
                #   2. If home visible but off-center → burst-rotate to center.
                #   3. If home centered → forward.
                #   4. Arrival when front center ultrasonic ≤ STOP_DIST_MM.
                elif state == State.RETURNING:
                    snap = sonar.snapshot()
                    front_mm = _safe(snap.get("front"))
                    if front_mm <= STOP_DIST_MM:
                        stop()
                        print("\n  Returned home! Say wake word for new orders.")
                        wake_word_heard = False
                        target_label = None
                        home_last_bearing = None
                        search_rotate_dir = None
                        lcd.update_queue([])
                        state = State.ORDERING
                        lcd.update_state("ORDERING")
                        continue

                    tracker.update(queues, {TAG_IDS["Home"]})
                    tag_px = tracker.tag_px

                    if tag_px is None:
                        # Home not visible — burst-rotate to search.
                        if search_rotate_dir is None:
                            search_rotate_dir = _bearing_to_dir(home_last_bearing, "right")
                        _burst_rotate(search_rotate_dir, TURN_SPEED)
                        continue

                    home_last_bearing = tag_px - _CAM_CENTER_PX
                    offset_px = tag_px - _CAM_CENTER_PX
                    if abs(offset_px) > CENTER_TOL_PX:
                        direction = "right" if offset_px > 0 else "left"
                        search_rotate_dir = direction
                        _burst_rotate(direction, TURN_SPEED)
                        continue

                    # Home centered — drive forward until ultrasonic trips.
                    search_rotate_dir = None
                    forward(APPROACH_SPEED)

        except KeyboardInterrupt:
            print("\nAborted by user.")
        finally:
            # `with pipeline:` tears the pipeline down on exit — don't call
            # pipeline.stop() here, it's redundant and can log warnings.
            stop()
            voice.stop()
            sonar.stop()
            lcd.clear()
            bus.close()
            print("Motors stopped, cleanup done.")


def main():
    deliver()


if __name__ == "__main__":
    main()

