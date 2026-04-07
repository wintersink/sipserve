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
OBSTACLE_MM      = 400    # ~16 inches — ultrasonic trigger during SEARCH
APPROACH_DIST_MM = 762    # 30 inches — switch to APPROACH speed
STOP_DIST_MM     = 330    # 12 inches — stop at destination
CENTER_X_MM      = 60     # lateral offset tolerance for "centered"

# Tag-locked obstacle thresholds (mm) — when NAVIGATE/APPROACH, tag takes priority
OBSTACLE_SLOW_MM  = 508   # 20 inches — slow down while still approaching tag
OBSTACLE_AVOID_MM = 152   # 6 inches — too close, navigate around obstacle
OBSTACLE_HARD_MM  = 76    # 3 inches — emergency stop, must avoid

# Ramp zone: linear speed interpolation between these two distances
RAMP_FAR_MM  = 1000   # above this → CRUISE_SPEED
RAMP_NEAR_MM = 400    # below this → APPROACH_SPEED

# Lock-on: when tag is lost, hold heading via gyro at reduced speed
LOCKON_SPEED       = 300   # forward speed while holding heading (tag lost)
HEADING_TOL_DEG    = 5.0   # heading error tolerance before correcting
HEADING_TURN_SPEED = 200   # gentle turn to correct heading drift

# Timing
TAG_LOST_TIMEOUT_S = 3.0   # seconds without tag before falling back to SEARCH
ARRIVE_PAUSE_S     = 3.0   # seconds to pause at destination
AVOID_TIMEOUT_S    = 4.0   # max seconds turning during AVOID
BACKUP_S           = 0.5   # seconds to reverse if turn times out


# ── State machine ─────────────────────────────────────────────────────────────

class State(enum.Enum):
    STARTUP        = "STARTUP"        # spin until path clear and no tag visible
    IDLE           = "IDLE"           # wait for operator to enter destination
    SEARCH         = "SEARCH"         # drive forward, scan camera for target tag
    NAVIGATE       = "NAVIGATE"       # steer toward visible tag (far)
    APPROACH       = "APPROACH"       # slow fine approach (close)
    AVOID          = "AVOID"          # turn away from obstacle
    ARRIVED        = "ARRIVED"        # pause at destination
    RETURN_STARTUP = "RETURN_STARTUP" # clear path before heading home
    DONE           = "DONE"           # delivery complete


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

    def __init__(self, bus: smbus2.SMBus):
        self._mux = Mux(bus)
        self._sensor = UltrasonicSensor(bus)
        self._lock = threading.Lock()
        self._readings: dict[str, int | None] = {name: None for name in SENSOR_CHANNELS}
        self._ready = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def wait_ready(self, timeout: float = 3.0):
        """Block until the first full sensor sweep completes."""
        print("  Waiting for ultrasonic sensors…")
        self._ready.wait(timeout=timeout)
        print(f"  Sensors ready: {self.snapshot()}")

    def _poll_loop(self):
        """Read all 5 sensors in a continuous loop (~0.8 s per cycle)."""
        while self._running:
            fresh = {}
            for name, ch in SENSOR_CHANNELS.items():
                if not self._running:
                    return
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
    """Tracks a single target AprilTag across frames.

    Handles the one-frame-behind spatial data: we send an ROI for the tag
    we see now, but the spatial result we read back corresponds to the ROI
    we sent in the *previous* frame.
    """

    def __init__(self):
        self._prev_tag = None  # tag object from previous frame (or None)
        self.tag_px = None     # current-frame tag center x in pixels (or None)

    def update(self, queues, target_id) -> tuple[float | None, float | None]:
        """Read one camera frame and return (dist_mm, x_mm) for the target.

        Returns (None, None) if the target tag is not visible or depth is
        not yet available.  self.tag_px is set to the tag's pixel center x
        on the current frame (available even when depth is not).
        """
        tag_msg = queues["tags"].get()
        all_tags = list(tag_msg.aprilTags)
        target_tags = [t for t in all_tags if t.id == target_id]

        # Tag pixel position — available THIS frame, no one-frame lag
        if target_tags:
            tag = target_tags[0]
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
        return dist_mm, x_mm


# Camera center x in pixels — offset from this = how far off-center the tag is
_CAM_CENTER_PX = TAG_INPUT_W / 2.0
# Rough scale: convert pixel offset to approximate mm offset for arc_drive.
# At ~1m distance, 1 pixel ≈ 1.5 mm lateral offset (rough FOV estimate).
PX_TO_MM = 1.5


def pixel_x_to_offset(tag_px: float) -> float:
    """Convert tag pixel x to a lateral mm-like offset for steering."""
    return (tag_px - _CAM_CENTER_PX) * PX_TO_MM


# ── Speed / heading helpers ───────────────────────────────────────────────────

def hold_heading(gyro: GyroTracker, locked_heading: float, speed: int = LOCKON_SPEED):
    """Drive forward while correcting heading drift via gyro."""
    error = locked_heading - gyro.heading_deg
    # Normalize to (-180, 180]
    error = (error + 180) % 360 - 180

    if abs(error) < HEADING_TOL_DEG:
        forward(speed)
    elif error > 0:
        turn_left(HEADING_TURN_SPEED)
    else:
        turn_right(HEADING_TURN_SPEED)


def compute_speed(dist_mm: float) -> int:
    """Linear ramp from CRUISE_SPEED down to APPROACH_SPEED."""
    if dist_mm >= RAMP_FAR_MM:
        return CRUISE_SPEED
    if dist_mm <= RAMP_NEAR_MM:
        return APPROACH_SPEED
    ratio = (dist_mm - RAMP_NEAR_MM) / (RAMP_FAR_MM - RAMP_NEAR_MM)
    return int(APPROACH_SPEED + ratio * (CRUISE_SPEED - APPROACH_SPEED))


TURN_AWAY_TIMEOUT_S = 6.0  # max seconds to turn away from arrived tag


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

def deliver() -> None:
    # Hardware init
    IIC.set_motor_parameter()
    stop()
    bus = smbus2.SMBus(1)
    sonar = UltrasonicPoller(bus)
    tracker = TagTracker()
    gyro = GyroTracker()

    pipeline, queues = build_delivery_pipeline()

    with pipeline:
        pipeline.start()

        # Wait for sensors before moving — don't drive blind
        sonar.wait_ready()
        # Prime the camera: first frame has no spatial data (one-frame-behind),
        # so read and discard it while stationary.
        queues["tags"].get()
        queues["spatial"].get()

        state = State.STARTUP
        target_id = TAG_IDS["home"]
        return_home = False
        tag_last_seen = time.monotonic()
        locked_heading = None
        last_dist_mm = None

        try:
            while pipeline.isRunning():
                gyro.update(queues["imu"])

                # ── Obstacle check ────────────────────────────────────────
                if state == State.SEARCH:
                    # Standard avoidance during search (no tag lock)
                    if sonar.front_blocked():
                        state = State.AVOID
                        stop()
                        print("  !! Obstacle detected — avoiding")
                elif state in (State.NAVIGATE, State.APPROACH):
                    # Tag-locked: prioritize approach, only avoid at 6 inches
                    front_dist = sonar.min_front_dist()
                    if front_dist < OBSTACLE_HARD_MM:
                        state = State.AVOID
                        stop()
                        print(f"  !! Emergency — obstacle at {front_dist}mm, avoiding")
                    elif front_dist < OBSTACLE_AVOID_MM:
                        state = State.AVOID
                        stop()
                        print(f"  !! Obstacle at {front_dist}mm during approach — avoiding")

                # ── STARTUP: spin until front is clear and no tag is visible ──
                if state == State.STARTUP:
                    front_clear = not sonar.front_blocked()
                    tag_msg = queues["tags"].tryGet()
                    queues["spatial"].tryGet()   # drain spatial to stay in sync
                    tag_visible = bool(tag_msg and tag_msg.aprilTags)

                    if front_clear and not tag_visible:
                        stop()
                        print("Startup clear — enter destination to begin delivery.")
                        state = State.IDLE
                    else:
                        direction = sonar.best_turn_dir()
                        if direction == "right":
                            turn_right(AVOID_TURN)
                        else:
                            turn_left(AVOID_TURN)
                        time.sleep(0.1)

                # ── IDLE: wait for operator input ─────────────────────────────
                elif state == State.IDLE:
                    stop()
                    dest = prompt_destination()
                    target_id = TAG_IDS[dest]
                    return_home = False
                    tracker = TagTracker()
                    gyro.reset()
                    locked_heading = None
                    last_dist_mm = None
                    tag_last_seen = time.monotonic()
                    # Drain stale camera frames accumulated during blocking input
                    while queues["tags"].tryGet() is not None:
                        queues["spatial"].tryGet()
                    state = State.SEARCH
                    print(f"Heading to {dest!r} (tag {target_id})…")

                # ── SEARCH: drive forward, scan for target tag ────────────────
                elif state == State.SEARCH:
                    dist_mm, x_mm = tracker.update(queues, target_id)
                    if dist_mm is not None:
                        tag_last_seen = time.monotonic()
                        locked_heading = gyro.heading_deg
                        last_dist_mm = dist_mm
                        if dist_mm <= APPROACH_DIST_MM:
                            state = State.APPROACH
                        else:
                            state = State.NAVIGATE
                        print(f"  Tag {target_id} found at {dist_mm:.0f} mm → {state.value}")
                    else:
                        forward(ROAM_SPEED)

                # ── NAVIGATE: steer toward tag, course-correct ────────────────
                elif state == State.NAVIGATE:
                    dist_mm, x_mm = tracker.update(queues, target_id)

                    # Slow down if obstacle is within 20 inches
                    obstacle_near = sonar.min_front_dist() < OBSTACLE_SLOW_MM

                    if dist_mm is None:
                        # Tag visible in pixels but no depth yet — steer by pixel
                        if tracker.tag_px is not None:
                            tag_last_seen = time.monotonic()
                            locked_heading = gyro.heading_deg
                            px_offset = pixel_x_to_offset(tracker.tag_px)
                            nav_speed = APPROACH_SPEED if obstacle_near else CRUISE_SPEED
                            arc_drive(nav_speed, px_offset)
                            continue
                        lost_s = time.monotonic() - tag_last_seen
                        if lost_s > TAG_LOST_TIMEOUT_S:
                            print("  Tag lost — searching")
                            locked_heading = None
                            state = State.SEARCH
                        elif locked_heading is not None:
                            hold_heading(gyro, locked_heading)
                        continue

                    tag_last_seen = time.monotonic()
                    locked_heading = gyro.heading_deg
                    last_dist_mm = dist_mm
                    print(f"  NAV  dist={dist_mm:.0f}mm  x={x_mm:+.0f}mm")

                    if dist_mm <= STOP_DIST_MM:
                        stop()
                        state = State.ARRIVED
                    elif dist_mm <= APPROACH_DIST_MM:
                        state = State.APPROACH
                    else:
                        speed = compute_speed(dist_mm)
                        if obstacle_near:
                            speed = min(speed, APPROACH_SPEED)
                        arc_drive(speed, x_mm)

                # ── APPROACH: slow drive, fine centering ──────────────────────
                elif state == State.APPROACH:
                    dist_mm, x_mm = tracker.update(queues, target_id)

                    if dist_mm is None:
                        # Tag visible in pixels but no depth — steer by pixel
                        if tracker.tag_px is not None:
                            tag_last_seen = time.monotonic()
                            locked_heading = gyro.heading_deg
                            px_offset = pixel_x_to_offset(tracker.tag_px)
                            arc_drive(APPROACH_SPEED, px_offset)
                            continue
                        lost_s = time.monotonic() - tag_last_seen
                        if last_dist_mm is not None and last_dist_mm < 500:
                            if lost_s > TAG_LOST_TIMEOUT_S:
                                stop()
                                print("  Tag lost at close range — arriving by proximity")
                                state = State.ARRIVED
                            else:
                                hold_heading(gyro, locked_heading, APPROACH_SPEED)
                        elif lost_s > TAG_LOST_TIMEOUT_S:
                            print("  Tag lost during approach — searching")
                            locked_heading = None
                            state = State.SEARCH
                        elif locked_heading is not None:
                            hold_heading(gyro, locked_heading, APPROACH_SPEED)
                        continue

                    tag_last_seen = time.monotonic()
                    locked_heading = gyro.heading_deg
                    last_dist_mm = dist_mm
                    print(f"  APPR dist={dist_mm:.0f}mm  x={x_mm:+.0f}mm")

                    if dist_mm <= STOP_DIST_MM:
                        stop()
                        state = State.ARRIVED
                    else:
                        arc_drive(APPROACH_SPEED, x_mm)

                # ── AVOID: turn away from obstacle until path is clear ─────────
                elif state == State.AVOID:
                    direction = sonar.best_turn_dir()
                    deadline = time.monotonic() + AVOID_TIMEOUT_S
                    cleared = False

                    while time.monotonic() < deadline:
                        if direction == "right":
                            turn_right(AVOID_TURN)
                        else:
                            turn_left(AVOID_TURN)
                        time.sleep(0.15)
                        gyro.update(queues["imu"])
                        if not sonar.front_blocked():
                            cleared = True
                            break

                    if not cleared:
                        print("  Backing up…")
                        backward(BACKUP_SPEED)
                        time.sleep(BACKUP_S)
                    stop()

                    # Check if we can see the tag now
                    dist_mm, x_mm = tracker.update(queues, target_id)
                    if dist_mm is not None:
                        tag_last_seen = time.monotonic()
                        state = State.NAVIGATE
                        print("  Obstacle cleared — tag visible, navigating")
                    else:
                        state = State.SEARCH
                        print("  Obstacle cleared — searching")

                # ── ARRIVED: pause, then clear path for return leg ────────────
                elif state == State.ARRIVED:
                    stop()
                    arrived_tag_id = target_id
                    if not return_home:
                        print(f"\n  Arrived at destination (tag {arrived_tag_id})!")
                        print(f"  Pausing {ARRIVE_PAUSE_S:.0f}s…")
                        time.sleep(ARRIVE_PAUSE_S)
                        _turn_away(arrived_tag_id, queues)
                        return_home = True
                        tracker = TagTracker()
                        locked_heading = None
                        state = State.RETURN_STARTUP
                        print("  Clearing path before heading home…")
                    else:
                        print("\n  Returned home! Delivery complete.")
                        time.sleep(ARRIVE_PAUSE_S)
                        state = State.DONE

                # ── RETURN_STARTUP: spin until path clear and no tag visible ──
                elif state == State.RETURN_STARTUP:
                    front_clear = not sonar.front_blocked()
                    tag_msg = queues["tags"].tryGet()
                    queues["spatial"].tryGet()   # drain spatial to stay in sync
                    tag_visible = bool(tag_msg and tag_msg.aprilTags)

                    if front_clear and not tag_visible:
                        stop()
                        target_id = TAG_IDS["home"]
                        tracker = TagTracker()
                        gyro.reset()
                        locked_heading = None
                        last_dist_mm = None
                        tag_last_seen = time.monotonic()
                        state = State.SEARCH
                        print("  Path clear — searching for home tag…")
                    else:
                        direction = sonar.best_turn_dir()
                        if direction == "right":
                            turn_right(AVOID_TURN)
                        else:
                            turn_left(AVOID_TURN)
                        time.sleep(0.1)

                # ── DONE ──────────────────────────────────────────────────────
                elif state == State.DONE:
                    break

        except KeyboardInterrupt:
            print("\nAborted by user.")
        finally:
            stop()
            sonar.stop()
            pipeline.stop()
            bus.close()
            print("Motors stopped, cleanup done.")


# ── CLI ───────────────────────────────────────────────────────────────────────

def prompt_destination() -> str:
    options = [k for k in TAG_IDS if k != "home"]
    print("Select a destination:")
    for i, name in enumerate(options, 1):
        print(f"  {i}. {name}")
    while True:
        raw = input("Enter name or number: ").strip().lower().replace(" ", "")
        if raw in TAG_IDS and raw != "home":
            return raw
        if raw.isdigit() and 1 <= int(raw) <= len(options):
            return options[int(raw) - 1]
        print(f"  Invalid — choose from: {', '.join(options)}")


def main():
    deliver()


if __name__ == "__main__":
    main()
