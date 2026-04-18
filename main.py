"""
SipServe — autonomous beverage delivery state machine.

States:
  INITIALIZING     — rotate until home tag is visible
  AWAITING_ORDERS  — stationary; wake word + table commands queue orders
  SEARCHING        — rotate in place scanning for the first queued station
  LOCKED_ON        — drive toward the tag; front ultrasonics for obstacle halt
  LOST             — tag dropped mid-approach; gentle scan to reacquire
  DELIVERED        — paused at destination; 10s then pop queue
  RETURNING        — same LOCKED_ON/LOST pattern but target = home

Voice commands (DF2301QG, same as test_delivery_run.py):
  WAKE (2) — start interaction
  QUEUE_TABLE1..4 (5..8) — add to queue
  CLEAR_ORDERS (9) — wipe queue
  START_DELIVERY (10) — begin delivery
"""

import enum
import threading
import time
from collections import deque

import depthai as dai
import smbus2

from config import TAG_IDS
from hardware.motor import (
    Motor,
    FULL_SPEED,
    SLOW_SPEED,
    SMOOTH_TURN_SPEED,
    BURST_ROTATE_S,
    DEG_PER_SEC_AT_BURST_TURN,
)
from hardware.mux import Mux, SENSOR_CHANNELS
from hardware.lcd import SipServeLCD
from sensors.apriltag import (
    send_roi_configs,
    TAG_INPUT_W, TAG_INPUT_H,
    DEPTH_MIN_MM, DEPTH_MAX_MM,
)
from sensors.ultrasonic import UltrasonicSensor
from sensors.voice import VoiceSensor

# ── Constants ─────────────────────────────────────────────────────────────────

MM_PER_INCH = 25.4

# Voice command IDs
VCMD_WAKE_WORD       = 2
VCMD_QUEUE_TABLE1    = 5
VCMD_QUEUE_TABLE2    = 6
VCMD_QUEUE_TABLE3    = 7
VCMD_QUEUE_TABLE4    = 8
VCMD_CLEAR_ORDERS    = 9
VCMD_START_DELIVERY  = 10
VOICE_MUX_CHANNEL    = 5

VCMD_QUEUE_MAP = {
    VCMD_QUEUE_TABLE1: "Table 1",
    VCMD_QUEUE_TABLE2: "Table 2",
    VCMD_QUEUE_TABLE3: "Table 3",
    VCMD_QUEUE_TABLE4: "Table 4",
}

HOME_KEY = "Home"

# Arrival / timing (inches + seconds)
ARRIVAL_DIST_IN       = 12.0    # tag closer than this → arrived
OBSTACLE_STOP_IN      = 12.0    # front ultrasonic threshold for obstacle + arrival confirm
SIDE_SLOW_IN          = 16.0    # left/right triggered → drive slower
DELIVERY_PAUSE_S      = 10.0    # pause at destination
LOST_TIMEOUT_S        = 1.0     # LOCKED_ON → LOST after this many seconds of no-tag
SEARCH_ROTATE_BURST_S = 0.3     # rotate this long then pause to scan
SEARCH_PAUSE_S        = 0.2     # pause after each burst to let camera settle
TICK_S                = 0.1     # main loop tick interval

# SEARCHING alternates between a roam phase (reactive obstacle avoidance,
# same movement logic as smoketests/test_obstacle_avoidance) and a scan
# phase (burst-rotate for SCAN_DEGREES). Each tick checks for the queued
# tag first, so LOCKED_ON can be entered from either phase.
ROAM_DURATION         = 10.0     # seconds per roam phase
SCAN_DEGREES          = 960.0    # degrees to burst-rotate per scan phase
SEARCH_OBSTACLE_MM    = 400     # roam-phase obstacle trigger (mm)
ROAM_REVERSE_S        = 0.5     # reverse this long when front is blocked
ROAM_TURN_S           = 0.5     # turn this long when front is blocked

# LOCKED_ON centering sub-phase — on fresh entry, slow-rotate to center the
# tag tightly in frame before beginning forward approach. Tighter tolerance
# than _drive_toward's bang-bang (±60 px) so approach starts well-aimed.
CENTER_TURN_SPEED     = SMOOTH_TURN_SPEED   # PWM — same as smooth turn; lower
                                            # values fall below the motor deadzone
                                            # (1600) and produce no rotation.
CENTER_PX_TOL         = 30      # pixels — considered centered once |dx| < this
# Pulse-and-pause centering: each tick rotates briefly, stops, then sleeps
# so the camera grabs a still frame before the next decision. Tames the
# overshoot caused by the motor running continuously across ticks.
CENTER_PULSE_S        = 0.40    # rotate this long per centering pulse (~4° each)
CENTER_PAUSE_S        = 0.15    # then pause for camera to settle

# Cautious-approach margin (inches): when the front ultrasonic reads closer
# than the tag, leave this much clearance before the obstacle when creeping
# forward. Lets the robot reach arrival distance even when the tag's
# mounting surface (or something near it) trips the ultrasonic short.
APPROACH_SAFETY_IN    = 1.0
APPROACH_STEER_PX     = 60      # pixels — bang-bang steer threshold during approach
CENTER_NUDGE_IN       = 2.0     # inches to nudge forward when tag lost during centering
CENTER_NUDGE_MAX      = 5       # max nudges before giving up to LOST

TAG_ID_TO_STATION = {tid: name for name, tid in TAG_IDS.items()}


class State(enum.Enum):
    INITIALIZING    = "INITIALIZING"
    AWAITING_ORDERS = "AWAITING_ORDERS"
    SEARCHING       = "SEARCHING"
    LOCKED_ON       = "LOCKED_ON"
    LOST            = "LOST"
    DELIVERED       = "DELIVERED"
    RETURNING       = "RETURNING"


# LCD state display: (label, include_target)
STATE_DISPLAY = {
    State.INITIALIZING:    ("Initializing",   False),
    State.AWAITING_ORDERS: ("Awaiting Orders", False),
    State.SEARCHING:       ("Searching",       False),
    State.LOCKED_ON:       ("Locked On",       True),
    State.LOST:            ("Lost",            True),
    State.DELIVERED:       ("Delivered To",    True),
    State.RETURNING:       ("Returning",       False),
}


def _table_number(name: str) -> str:
    """'Table 1' → '1'; 'Home' → ''."""
    parts = name.split()
    return parts[-1] if parts and parts[-1].isdigit() else ""


# ── Ultrasonic background poller ──────────────────────────────────────────────

def _mm_to_in(mm):
    return None if mm is None else mm / MM_PER_INCH


def _safe_mm(d):
    return d if d is not None else 99999


class UltrasonicPoller:
    """Polls all 5 ultrasonic sensors continuously; snapshot() is non-blocking."""

    def __init__(self, bus: smbus2.SMBus, i2c_lock: threading.Lock):
        self._mux = Mux(bus)
        self._sensor = UltrasonicSensor(bus)
        self._lock = i2c_lock
        self._state_lock = threading.Lock()
        self._readings = {name: None for name in SENSOR_CHANNELS}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            fresh = {}
            for name, ch in SENSOR_CHANNELS.items():
                if self._stop.is_set():
                    return
                with self._lock:
                    self._mux.select(ch)
                    self._sensor.trigger()
                time.sleep(0.15)
                with self._lock:
                    self._mux._active = None  # force re-select after lock gap
                    self._mux.select(ch)
                    fresh[name] = self._sensor.read_result()
            with self._state_lock:
                self._readings.update(fresh)

    def snapshot(self) -> dict:
        with self._state_lock:
            return dict(self._readings)

    def front_min_in(self) -> float:
        snap = self.snapshot()
        vals = [_safe_mm(snap.get(k)) for k in ("front", "front_left", "front_right")]
        return min(vals) / MM_PER_INCH

    def sides_min_in(self) -> float:
        snap = self.snapshot()
        vals = [_safe_mm(snap.get(k)) for k in ("left", "right")]
        return min(vals) / MM_PER_INCH

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        with self._lock:
            self._mux.disable()


# ── Voice background poller ───────────────────────────────────────────────────

class VoicePoller:
    """Polls DF2301QG voice recognition in a background thread."""

    def __init__(self, bus: smbus2.SMBus, i2c_lock: threading.Lock):
        self._sensor = VoiceSensor(bus, mux_channel=VOICE_MUX_CHANNEL, i2c_lock=i2c_lock)
        self._cmds: deque = deque(maxlen=32)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            cmd = self._sensor.get_command_id()
            if cmd:
                with self._lock:
                    self._cmds.append(cmd)
            time.sleep(0.1)

    def drain(self) -> list:
        with self._lock:
            out = list(self._cmds)
            self._cmds.clear()
        return out

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


# ── Camera pipeline + tag watcher ─────────────────────────────────────────────

def build_pipeline():
    """Lean AprilTag + stereo-depth pipeline (no IMU / YOLO)."""
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    tag_out = camRgb.requestOutput((TAG_INPUT_W, TAG_INPUT_H))

    monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
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

    queues = {
        "tags":        aprilTag.out.createOutputQueue(),
        "spatial":     spatialCalc.out.createOutputQueue(),
        "spatial_cfg": spatialCalc.inputConfig.createInputQueue(),
    }
    return pipeline, queues


class TagWatcher:
    """Runs the camera pipeline in a thread. Exposes latest visible-tag info.

    For each visible tag we record:
        {'px': float, 'dist_mm': float|None, 'last_seen': monotonic_s}

    Also emits 'detected'/'lost' events for AprilTagWatcher-style notifications.
    """

    def __init__(self):
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self._visible: dict[int, dict] = {}
        self._events: deque = deque(maxlen=64)
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    # ── Queries used by the FSM ───────────────────────────────────────────────

    def get_tag(self, tag_id: int) -> dict | None:
        with self._state_lock:
            info = self._visible.get(tag_id)
            return dict(info) if info else None

    def any_visible(self, tag_ids: set[int]) -> int | None:
        with self._state_lock:
            for tid in tag_ids:
                if tid in self._visible:
                    return tid
        return None

    def pop_events(self) -> list:
        with self._state_lock:
            out = list(self._events)
            self._events.clear()
        return out

    # ── Thread ────────────────────────────────────────────────────────────────

    def _run(self):
        try:
            pipeline, q = build_pipeline()
        except Exception as e:
            print(f"\n[camera] pipeline build failed: {e}")
            return

        prev_tag = None  # for one-frame-lag spatial correlation
        prev_ids: set[int] = set()

        try:
            with pipeline:
                pipeline.start()
                print("\n[camera] AprilTag detector running.")
                while not self._stop.is_set():
                    tag_msg = q["tags"].tryGet()
                    if tag_msg is None:
                        time.sleep(0.02)
                        continue
                    tags = list(tag_msg.aprilTags)

                    # Compute pixel center for each visible tag
                    centers = {
                        t.id: (t.topLeft.x + t.topRight.x +
                               t.bottomLeft.x + t.bottomRight.x) / 4.0
                        for t in tags
                    }
                    current_ids = set(centers.keys())

                    # Send ROI for the first tag so we can get its depth next frame
                    if tags:
                        send_roi_configs([tags[0]], q["spatial_cfg"])

                    # Read spatial locations (corresponds to previous frame's ROI)
                    spatial_msg = q["spatial"].tryGet()
                    dist_mm = None
                    if spatial_msg and prev_tag is not None:
                        locs = spatial_msg.getSpatialLocations()
                        if locs and locs[0].spatialCoordinates.z > 0:
                            dist_mm = locs[0].spatialCoordinates.z

                    now = time.monotonic()
                    new_visible = {}
                    for tid, px in centers.items():
                        d = dist_mm if (prev_tag is not None and tid == prev_tag.id) else None
                        new_visible[tid] = {"px": px, "dist_mm": d, "last_seen": now}

                    with self._state_lock:
                        # Emit enter/leave events
                        for tid in current_ids - prev_ids:
                            self._events.append(("detected", tid))
                        for tid in prev_ids - current_ids:
                            self._events.append(("lost", tid))
                        self._visible = new_visible

                    prev_tag = tags[0] if tags else None
                    prev_ids = current_ids

        except Exception as e:
            print(f"\n[camera] stopped: {e}")


# ── SipServe FSM ──────────────────────────────────────────────────────────────

class SipServe:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.i2c_lock = threading.Lock()
        # Share the lock with Motor so its bus writes can't collide with the
        # ultrasonic mux poller or voice poller on bus 1.
        self.motor = Motor(self.bus, i2c_lock=self.i2c_lock)
        self.motor.set_motor_parameter()
        self.motor.stop()

        self.lcd = SipServeLCD()
        self.sonar = UltrasonicPoller(self.bus, self.i2c_lock)
        self.voice = VoicePoller(self.bus, self.i2c_lock)
        self.tags = TagWatcher()
        self.tags.start()

        self.queue: deque[str] = deque()   # station names
        self.state = State.INITIALIZING
        self._delivered_at = 0.0           # monotonic timestamp
        self._lost_since = 0.0             # monotonic timestamp when tag disappeared
        self._search_next_burst = 0.0      # when to kick another rotate burst
        # SEARCHING phase state: "roam" | "scan" | None (= needs init on entry).
        # Set to None on every entry into SEARCHING so _tick_searching resets
        # timers and accumulated scan angle.
        self._search_phase: str | None = None
        self._search_phase_start: float = 0.0
        self._search_scan_deg: float = 0.0
        # LOCKED_ON has a one-shot centering sub-phase — False on fresh entry,
        # latches True once the tag is tightly centered, then _drive_toward
        # handles the approach with its own bang-bang steering.
        self._locked_centered: bool = False
        self._center_nudges: int = 0
        # INITIALIZING also runs a centering pass on the home tag before
        # transitioning to AWAITING_ORDERS, so the robot starts each
        # ordering cycle facing home squarely.
        self._init_centered: bool = False
        # RETURNING toggles between approach mode (same logic as LOCKED_ON)
        # and search mode (roam/scan cycle) when the home tag drops out
        # past LOST_TIMEOUT_S. Reacquiring home flips it back to approach.
        self._return_searching: bool = False
        self._update_lcd()

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self):
        self.motor.stop()
        self.tags.stop()
        self.sonar.stop()
        self.voice.stop()
        self.lcd.clear()
        print("\nMotors stopped. Camera stopped. Sensors stopped. LCD cleared.")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_state(self, new):
        if new != self.state:
            print(f"\n[state] {self.state.value} → {new.value}")
            self.motor.stop()
            self.state = new
            if new in (State.SEARCHING, State.LOST, State.RETURNING):
                # Reset the roam ↔ scan cycle on entry so each search-style
                # state starts in a known phase (SEARCHING re-runs its
                # initial 180°; LOST and RETURNING start fresh in roam).
                self._search_phase = None
            if new in (State.LOCKED_ON, State.RETURNING):
                # Force a fresh slow-centering pass every time we start a
                # new approach (LOCKED_ON or RETURNING). RETURNING starts
                # in approach mode and only flips to search if it loses
                # home for longer than LOST_TIMEOUT_S.
                self._locked_centered = False
                self._center_nudges = 0
                self._lost_since = 0
            if new == State.RETURNING:
                self._return_searching = False
            if new == State.INITIALIZING:
                # Re-run home centering on every fresh INITIALIZING entry.
                self._init_centered = False
            self._update_lcd()

    def _current_target_name(self) -> str | None:
        """Station name used for display based on current state."""
        if self.state == State.RETURNING:
            return HOME_KEY
        if self.state in (State.SEARCHING, State.LOCKED_ON, State.LOST, State.DELIVERED):
            return self.queue[0] if self.queue else None
        return None

    def _update_lcd(self):
        label, include_target = STATE_DISPLAY[self.state]
        target = self._current_target_name() if include_target else None
        self.lcd.update_state(label, target)
        nums = [_table_number(n) for n in self.queue]
        nums = [n for n in nums if n]
        self.lcd.update_queue(nums)
        # Subphase row gets cleared on every state transition; live ticks
        # set it via _set_subphase() below.
        self.lcd.update_subphase(None)

    def _set_subphase(self, text: str | None):
        """Update LCD row 2 with the current sub-phase / decision branch."""
        self.lcd.update_subphase(text)

    def _current_target_id(self) -> int | None:
        """Tag ID of the first queued station (or None if queue is empty)."""
        if not self.queue:
            return None
        return TAG_IDS.get(self.queue[0])

    def _drain_tag_events(self):
        """Drain the per-frame detected/lost events from TagWatcher.

        The FSM already handles tag presence via get_tag() + LOST_TIMEOUT_S,
        so we just clear the deque — no per-frame printing.
        """
        self.tags.pop_events()

    def _handle_voice_in_ordering(self):
        for cmd in self.voice.drain():
            if cmd == VCMD_WAKE_WORD:
                print("  Voice: wake word — ready for orders")
            elif cmd in VCMD_QUEUE_MAP:
                name = VCMD_QUEUE_MAP[cmd]
                if name not in self.queue:
                    self.queue.append(name)
                    self._update_lcd()
                print(f"  Voice: queued {name} — {', '.join(self.queue) or 'empty'}")
            elif cmd == VCMD_CLEAR_ORDERS:
                self.queue.clear()
                self._update_lcd()
                print("  Voice: orders cleared")
            elif cmd == VCMD_START_DELIVERY:
                if self.queue:
                    print(f"  Voice: starting delivery — {', '.join(self.queue)}")
                    self._set_state(State.SEARCHING)
                    return
                else:
                    print("  Voice: start requested but queue is empty")

    def _arrived(self, tag_id: int) -> bool:
        """Arrival check. Prefers stereo depth; falls back to ultrasonic when
        the tag is visible but depth is unavailable (one-frame-lag misses)."""
        info = self.tags.get_tag(tag_id)
        if not info:
            return False
        dist_mm = info.get("dist_mm")
        if dist_mm is not None:
            return (dist_mm / MM_PER_INCH) < ARRIVAL_DIST_IN
        # Fallback: tag visible (px present) AND front ultrasonic close enough.
        if info.get("px") is not None and self.sonar.front_min_in() < ARRIVAL_DIST_IN:
            return True
        return False

    def _drive_toward(self, tag_id: int):
        """One-tick steer-and-drive toward a visible tag. Caller ensures tag is visible."""
        front_in = self.sonar.front_min_in()
        info = self.tags.get_tag(tag_id)
        tag_in = (info["dist_mm"] / MM_PER_INCH) if info and info["dist_mm"] else None
        px = info.get("px") if info else None
        tag_visible = px is not None

        # Steering takes priority — turning in place is always safe (no
        # forward motion). If the tag is way off-axis, re-center before
        # any obstacle/approach decision so we don't freeze pointing at
        # something other than the tag.  Use pulse-and-pause so the motor
        # doesn't keep turning during the camera-frame lag and overshoot.
        if tag_visible:
            dx = px - TAG_INPUT_W / 2.0
            if abs(dx) > APPROACH_STEER_PX:
                direction = "RIGHT" if dx > 0 else "LEFT"
                print(f"    drive: steer {direction} pulse  dx={dx:.0f}px")
                if dx > 0:
                    self.motor.turn_right(SMOOTH_TURN_SPEED)
                else:
                    self.motor.turn_left(SMOOTH_TURN_SPEED)
                time.sleep(CENTER_PULSE_S)
                self.motor.stop()
                time.sleep(CENTER_PAUSE_S)
                return

        # Obstacle / cautious-approach logic. Cases when front_in < OBSTACLE_STOP_IN:
        #   1. tag_in known, obstacle clearly closer than tag (front < tag - 2):
        #      check if there's room to creep forward to arrival without
        #      hitting the obstacle (cautious approach). If not, halt.
        #   2. tag_in known, ultrasonic ~= tag → fall through and drive.
        #   3. tag_in unknown but tag visible (px) → assume sonar is reading
        #      the tag itself; drive forward slowly.
        #   4. tag fully invisible AND front close → real unknown obstacle, halt.
        if front_in < OBSTACLE_STOP_IN:
            if tag_in is not None and front_in < tag_in - 2:
                want_in = tag_in - ARRIVAL_DIST_IN
                safe_in = front_in - APPROACH_SAFETY_IN
                if 0 < want_in <= safe_in:
                    # We can reach arrival distance without colliding —
                    # creep forward by want_in inches, then re-evaluate.
                    print(f"    drive: CAUTIOUS approach  want={want_in:.1f}\" safe={safe_in:.1f}\"  "
                          f"(front={front_in:.1f}\" tag={tag_in:.1f}\")")
                    self._set_subphase(f"cautious +{want_in:.1f}\"")
                    self.motor.move_distance(want_in * MM_PER_INCH, SLOW_SPEED)
                    return
                print(f"    drive: HALT obstacle  front={front_in:.1f}\" tag={tag_in:.1f}\"  "
                      f"(want={want_in:.1f}\" > safe={safe_in:.1f}\")")
                self.motor.stop()
                return
            if tag_in is None and not tag_visible:
                print(f"    drive: HALT obstacle  front={front_in:.1f}\" tag=unknown")
                self.motor.stop()
                return
            # else: ultrasonic likely seeing the tag — fall through and drive.

        speed = SLOW_SPEED if self.sonar.sides_min_in() < SIDE_SLOW_IN else FULL_SPEED
        print(f"    drive: FORWARD speed={speed}  front={front_in:.1f}\" tag={tag_in}")
        self.motor.forward(speed)

    # ── Centering helper ──────────────────────────────────────────────────────

    def _center_pulse_on(self, tag_id: int, label: str = "center") -> bool | None:
        """One pulse of pulse-and-pause centering on tag_id.

        Returns:
          True  — already centered (|dx| < CENTER_PX_TOL); motor stopped.
          False — pulsed in a direction; not yet centered.
          None  — tag not visible (no fresh px); no pulse done.

        Each pulse runs the motor for CENTER_PULSE_S then stops and pauses
        CENTER_PAUSE_S so the next camera frame is captured while still.
        Tames overshoot caused by continuous motor commands persisting
        across the main loop's TICK_S.
        """
        info = self.tags.get_tag(tag_id)
        px = info.get("px") if info else None
        if px is None:
            return None
        dx = px - TAG_INPUT_W / 2.0
        if abs(dx) < CENTER_PX_TOL:
            self.motor.stop()
            return True
        direction = "RIGHT" if dx > 0 else "LEFT"
        print(f"  {label}: pulse {direction}  dx={dx:.0f}px")
        self._set_subphase(f"center {direction[0]} dx={dx:.0f}")
        if dx > 0:
            self.motor.turn_right(CENTER_TURN_SPEED)
        else:
            self.motor.turn_left(CENTER_TURN_SPEED)
        time.sleep(CENTER_PULSE_S)
        self.motor.stop()
        time.sleep(CENTER_PAUSE_S)
        return False

    # ── Per-state ticks ───────────────────────────────────────────────────────

    def _tick_initializing(self):
        home_id = TAG_IDS[HOME_KEY]
        info = self.tags.get_tag(home_id)
        if info:
            # Home found — center on it before handing off to AWAITING_ORDERS.
            if not self._init_centered:
                result = self._center_pulse_on(home_id, "INIT")
                if result is True:
                    self._init_centered = True
                    print("  INIT: centered on home — ready for orders")
                # result False → pulsed, wait next tick. result None → no
                # px this tick, hold still and wait for next frame.
                if result is None:
                    self.motor.stop()
                return
            print("  Home tag found. Say wake word, queue tables, then 'start delivery'.")
            self._set_state(State.AWAITING_ORDERS)
            return
        # Home not visible — gentle rotation bursts to scan
        now = time.monotonic()
        if now >= self._search_next_burst:
            self.motor.turn_right(SMOOTH_TURN_SPEED)
            self._search_next_burst = now + SEARCH_ROTATE_BURST_S + SEARCH_PAUSE_S
        elif now >= self._search_next_burst - SEARCH_PAUSE_S:
            self.motor.stop()

    def _tick_awaiting_orders(self):
        self.motor.stop()
        self._handle_voice_in_ordering()

    def _roam_tick(self):
        """One tick of reactive obstacle-avoidance roaming.

        Mirrors the movement logic in smoketests/test_obstacle_avoidance:
          - front blocked OR both front sides → stop, reverse, turn away
          - front_left only  → turn right
          - front_right only → turn left
          - left/right only  → forward slow
          - clear            → forward full
        The reverse+turn branch is blocking (matches the reference script);
        the other branches are non-blocking so tag pickup remains responsive.
        """
        snap = self.sonar.snapshot()
        def trig(name):
            d = snap.get(name)
            return d is not None and d < SEARCH_OBSTACLE_MM

        front = trig("front")
        fl = trig("front_left")
        fr = trig("front_right")
        left = trig("left")
        right = trig("right")

        if front or (fl and fr):
            self.motor.stop()
            self.motor.reverse()
            time.sleep(ROAM_REVERSE_S)
            self.motor.stop()
            fl_mm = snap.get("front_left") if snap.get("front_left") is not None else 9999
            fr_mm = snap.get("front_right") if snap.get("front_right") is not None else 9999
            if fl_mm <= fr_mm:
                self.motor.turn_right()
            else:
                self.motor.turn_left()
            time.sleep(ROAM_TURN_S)
            self.motor.stop()
        elif fl:
            self.motor.turn_right()
        elif fr:
            self.motor.turn_left()
        elif left or right:
            self.motor.forward(SLOW_SPEED)
        else:
            self.motor.forward(FULL_SPEED)

    def _search_step(self, label: str):
        """One tick of the roam ↔ scan cycle. Caller handles the
        'tag visible' branch and any state-specific arrival logic.

        On first call (or after a state-transition reset) starts in roam.
        Cycles: roam (ROAM_DURATION s) → scan (SCAN_DEGREES °) → roam → …
        Used by SEARCHING (after its initial 180° turn), LOST, and RETURNING
        when their target tag isn't visible — so each state actually
        relocates instead of spinning in place forever.
        """
        if self._search_phase is None or self._search_phase == "initial_rotate":
            self._search_phase = "roam"
            self._search_phase_start = time.monotonic()
            self._search_scan_deg = 0.0

        if self._search_phase == "roam":
            elapsed = time.monotonic() - self._search_phase_start
            if elapsed >= ROAM_DURATION:
                print(f"  {label}: roam done ({elapsed:.1f}s) → scan")
                self.motor.stop()
                self._search_phase = "scan"
                self._search_scan_deg = 0.0
                return
            self._set_subphase(f"roam {elapsed:.0f}/{ROAM_DURATION:.0f}s")
            self._roam_tick()
            return

        # scan
        if self._search_scan_deg >= SCAN_DEGREES:
            print(f"  {label}: scan done → roam")
            self.motor.stop()
            self._search_phase = "roam"
            self._search_phase_start = time.monotonic()
            return
        print(f"  {label}: scan  {self._search_scan_deg:.0f}°/{SCAN_DEGREES:.0f}°")
        self._set_subphase(f"scan {self._search_scan_deg:.0f}/{SCAN_DEGREES:.0f}")
        self.motor.burst_rotate("right")
        self._search_scan_deg += BURST_ROTATE_S * DEG_PER_SEC_AT_BURST_TURN

    def _tick_searching(self):
        target = self._current_target_id()
        if target is None:
            print("  SEARCH: no target → RETURNING")
            self._set_state(State.RETURNING)
            return
        if self.tags.get_tag(target):
            self._lost_since = 0
            print(f"  SEARCH: tag {target} found → LOCKED_ON")
            self._set_state(State.LOCKED_ON)
            return

        if self._search_phase is None:
            self._search_phase = "initial_rotate"
            self._search_phase_start = time.monotonic()
            self._search_scan_deg = 0.0

        if self._search_phase == "initial_rotate":
            print("  SEARCH: initial 180° turn-away")
            self._set_subphase("init 180 turn")
            self.motor.rotate(180)
            self._search_phase = "roam"
            self._search_phase_start = time.monotonic()
            return

        # Roam ↔ scan cycle for the rest of SEARCHING.
        self._search_step("SEARCH")

    def _approach_target(self, target_id: int, label: str) -> str:
        """Shared approach logic: arrival → lost-debounce → center → drive.

        Used by both LOCKED_ON (delivery) and RETURNING (home) so home
        gets the same flicker tolerance, centering pulses, and nudges.

        Returns one of:
          "arrived"  — tag close enough; caller handles state transition
          "lost"     — tag absent past LOST_TIMEOUT_S and nudges exhausted;
                       caller hands off to whatever search behavior fits
          "working"  — keep ticking; motor commands already issued
        """
        if self._arrived(target_id):
            return "arrived"

        info = self.tags.get_tag(target_id)
        if not info:
            front_in = self.sonar.front_min_in()
            # Close-range arrival fallback: if we already centered on the
            # tag and the front ultrasonic reads inside the arrival
            # distance, the most likely reason the tag dropped out of view
            # is that we drove right up to it (it left the camera's
            # vertical FOV). Declare arrived rather than restarting search.
            if self._locked_centered and front_in < ARRIVAL_DIST_IN:
                print(f"  {label}: tag lost while centered, front={front_in:.1f}\" → arrived")
                return "arrived"
            elapsed = (time.monotonic() - self._lost_since) if self._lost_since else 0
            if self._lost_since == 0:
                self._lost_since = time.monotonic()
            print(f"  {label}: tag missing  absent={elapsed:.2f}s/{LOST_TIMEOUT_S}s"
                  f"  centered={self._locked_centered}  nudges={self._center_nudges}"
                  f"  front={front_in:.1f}\"")
            self._set_subphase(f"missing {elapsed:.1f}/{LOST_TIMEOUT_S:.0f}s")
            if time.monotonic() - self._lost_since >= LOST_TIMEOUT_S:
                if (not self._locked_centered
                        and self._center_nudges < CENTER_NUDGE_MAX
                        and front_in > OBSTACLE_STOP_IN):
                    self._center_nudges += 1
                    print(f"  {label}: centering nudge {self._center_nudges}/{CENTER_NUDGE_MAX} "
                          f"({CENTER_NUDGE_IN:.0f}\")")
                    self._set_subphase(f"nudge {self._center_nudges}/{CENTER_NUDGE_MAX}")
                    self.motor.move_distance(CENTER_NUDGE_IN * MM_PER_INCH,
                                             SLOW_SPEED)
                    self._lost_since = 0
                    return "working"
                return "lost"
            return "working"

        # Tag visible — reset the absence timer.
        self._lost_since = 0
        dist_mm = info.get("dist_mm")

        if not self._locked_centered:
            result = self._center_pulse_on(target_id, label)
            if result is True:
                self._locked_centered = True
                print(f"  {label}: centered — beginning approach")
                self._set_subphase("centered, approach")
                # fall through to _drive_toward
            elif result is False:
                return "working"
            else:
                print(f"  {label}: centering but px=None, dist={dist_mm} — falling through")
                self._set_subphase("center px=None")

        dist_str = f"{dist_mm/MM_PER_INCH:.0f}\"" if dist_mm else "?"
        self._set_subphase(f"approach d={dist_str}")
        self._drive_toward(target_id)
        return "working"

    def _tick_locked_on(self):
        target = self._current_target_id()
        if target is None:
            print("  LOCKED: no target in queue → RETURNING")
            self._set_state(State.RETURNING)
            return
        result = self._approach_target(target, "LOCKED")
        if result == "arrived":
            name = self.queue[0]
            self.motor.stop()
            print(f"\n  >> Delivered {name}!  Pausing {DELIVERY_PAUSE_S:.0f}s…")
            self._delivered_at = time.monotonic()
            self._set_state(State.DELIVERED)
        elif result == "lost":
            print("  LOCKED: giving up → LOST")
            self._set_state(State.LOST)

    def _tick_lost(self):
        target = self._current_target_id()
        if target is None:
            print("  LOST: no target → RETURNING")
            self._set_state(State.RETURNING)
            return
        if self.tags.get_tag(target):
            print("  LOST: tag reacquired → LOCKED_ON")
            self._set_state(State.LOCKED_ON)
            return
        # Use the same roam ↔ scan cycle as SEARCHING so we actually move
        # to a new viewpoint instead of spinning in place forever.
        self._search_step("LOST")

    def _tick_delivered(self):
        self.motor.stop()
        if time.monotonic() - self._delivered_at >= DELIVERY_PAUSE_S:
            if self.queue:
                self.queue.popleft()
                self._update_lcd()
            if self.queue:
                print(f"  Next: {self.queue[0]}  (remaining: {', '.join(self.queue)})")
                self._set_state(State.SEARCHING)
            else:
                print("  Queue empty — heading home.")
                self._set_state(State.RETURNING)

    def _tick_returning(self):
        home_id = TAG_IDS[HOME_KEY]

        # Search mode — entered after _approach_target returned "lost".
        # Stay here until the home tag becomes visible again, then flip
        # back to approach mode with a fresh centering pass.
        if self._return_searching:
            if self.tags.get_tag(home_id):
                print("  RETURN: home tag reacquired → resuming approach")
                self._return_searching = False
                self._locked_centered = False
                self._center_nudges = 0
                self._lost_since = 0
                self._search_phase = None
                # Fall through to approach this same tick.
            else:
                self._search_step("RETURN")
                return

        # Approach mode — same delivery-style logic as LOCKED_ON.
        result = self._approach_target(home_id, "RETURN")
        if result == "arrived":
            front_in = self.sonar.front_min_in()
            self.motor.stop()
            print(f"\n  RETURN: arrived home  front={front_in:.1f}\"")
            print("  Returned home. Say wake word to place new orders.")
            self._set_state(State.AWAITING_ORDERS)
        elif result == "lost":
            print("  RETURN: home tag lost past timeout → roam/scan")
            self._return_searching = True
            self._search_phase = None

    # ── Run loop ──────────────────────────────────────────────────────────────

    TICKS = {
        State.INITIALIZING:    "_tick_initializing",
        State.AWAITING_ORDERS: "_tick_awaiting_orders",
        State.SEARCHING:       "_tick_searching",
        State.LOCKED_ON:       "_tick_locked_on",
        State.LOST:            "_tick_lost",
        State.DELIVERED:       "_tick_delivered",
        State.RETURNING:       "_tick_returning",
    }

    def run(self):
        print(f"[state] starting in {self.state.value}")
        try:
            while True:
                self._drain_tag_events()
                tick = getattr(self, self.TICKS[self.state])
                tick()
                time.sleep(TICK_S)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            self.shutdown()


if __name__ == "__main__":
    SipServe().run()
