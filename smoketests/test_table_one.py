"""
Table-one delivery test.

Navigates the robot from AprilTag 0 (start) to AprilTag 1 (table one)
using every available OAK-D-Lite feature.

Tracking (tag 1 visible):
  TRACK   — proportional steering toward tag 1 (always driving forward)
  SLOW    — within SLOW_MM, reduce speed
  YIELD   — person detected in path, stop and wait for them to move
  ARRIVED — within ARRIVE_MM, stop

Search (tag 1 NOT visible):
  SCAN    — spin in place using IMU for precise heading
  DEPTH   — read stereo depth map, split into sectors, pick clearest
            (sectors containing tag 0 are penalised)
  ADVANCE — drive toward chosen heading for up to ADVANCE_TIME seconds
  AVOID   — obstacle detected (ultrasonic OR NN), stop/reverse/rescan
  STUCK   — feature tracker detects no visual motion despite motors
            running, reverse and rescan

Sensors used:
  OAK-D-Lite: AprilTag, stereo depth, IMU, MobileNet-SSD spatial
              detection, object tracker, feature tracker
  Ultrasonic:  5-channel via TCA9548A mux (front, front_left, front_right)

Ctrl+C to quit.
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import smbus2
import IIC
from sensors.oak_pipeline import (
    build_enhanced_pipeline,
    send_roi_configs,
    GyroTracker,
    compute_feature_displacement,
    get_tracked_obstacles,
    TAG_INPUT_W,
    DEPTH_MIN_MM,
    DEPTH_MAX_MM,
    MM_PER_INCH,
)
from hardware.mux import Mux, SENSOR_CHANNELS
from sensors.ultrasonic import UltrasonicSensor

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
TARGET_TAG = 1
AVOID_TAG = 0

SPEED = 720                # ~10% duty cycle
SLOW_SPEED = 360
TURN_SPEED = 360
SEARCH_SPEED = 300

ARRIVE_MM = 400            # stop distance (~16 in)
SLOW_MM = 1000             # start slowing
POLL_INTERVAL = 0.05

# Search
SCAN_TIME = 2.0            # seconds to spin only when all paths are blocked
ADVANCE_TIME = 10.0        # seconds driving toward clearest path
OBSTACLE_MM = 500          # ultrasonic threshold
REVERSE_TIME = 0.4

# Depth-based navigation
NUM_SECTORS = 5

# Person awareness
PERSON_STOP_MM = 1000      # stop if person closer than this
PERSON_YIELD_SEC = 5.0     # max seconds to wait for person to move

# Stuck detection (feature tracker)
STUCK_DISP_PX = 1.5        # avg feature displacement below this = not moving
STUCK_FRAMES = 30          # consecutive low-displacement frames before stuck
STUCK_REVERSE_TIME = 0.8


# ---------------------------------------------------------------------------
# Motor primitives
# ---------------------------------------------------------------------------
def stop_motors():
    IIC.control_pwm(0, 0, 0, 0)


def drive_forward(speed=SPEED):
    IIC.control_pwm(-speed, 0, -speed, 0)


def reverse(speed=SPEED):
    IIC.control_pwm(speed, 0, speed, 0)


def spin_cw(speed=SEARCH_SPEED):
    IIC.control_pwm(-speed, 0, speed, 0)


def spin_ccw(speed=SEARCH_SPEED):
    IIC.control_pwm(speed, 0, -speed, 0)


def drive_toward(offset, speed=SPEED):
    """Drive forward with proportional steering.

    offset: normalised horizontal offset (-0.5 = far left, +0.5 = far right).
    Both tracks always move forward; the inner track slows proportionally.
    """
    steer = min(abs(offset) * 3.0, 0.95)
    fast = int(speed)
    slow = int(speed * (1.0 - steer))
    if offset > 0:
        IIC.control_pwm(-fast, 0, -slow, 0)
    else:
        IIC.control_pwm(-slow, 0, -fast, 0)


# ---------------------------------------------------------------------------
# IMU-based turning (gyroscope integration)
# ---------------------------------------------------------------------------
# Shared GyroTracker instance — created once at startup
gyro = GyroTracker()


def turn_degrees(deg, imu_queue, timeout=5.0, tag_queue=None, target_id=None):
    """Turn *deg* degrees using integrated gyroscope heading.

    Positive = clockwise, negative = counter-clockwise.

    If tag_queue and target_id are provided, polls the tag queue each tick
    and returns early as soon as the target tag appears.

    Returns (found: bool, tags: list) — found=True means early exit on tag.
    """
    # Drain any stale gyro data, then reset so we measure from zero
    gyro.update(imu_queue)
    gyro.reset()

    (spin_cw if deg > 0 else spin_ccw)(TURN_SPEED)

    t0 = time.time()
    while time.time() - t0 < timeout:
        turned = gyro.update(imu_queue)
        # gyro.z is positive for CCW; our convention: positive deg = CW
        # BMI270 z-axis: positive = CCW when viewed from above
        # So for CW turn (deg > 0), turned will go negative
        if deg > 0 and turned <= -deg:
            break
        if deg < 0 and turned >= -deg:
            break
        if tag_queue is not None and target_id is not None:
            msg = tag_queue.tryGet()
            if msg is not None:
                tags = list(msg.aprilTags)
                if find_tag(tags, target_id) is not None:
                    stop_motors()
                    print(f"[TURN]  Tag {target_id} spotted mid-turn — stopping")
                    return True, tags
        time.sleep(0.01)

    stop_motors()
    return False, []


# Sector index -> degrees to turn from dead centre
SECTOR_DEG = [-60, -30, 0, 30, 60]


def turn_to_sector(sector, imu_queue, tag_queue=None):
    """Turn to face a depth sector using gyroscope.

    Returns (found: bool, tags: list) forwarded from turn_degrees.
    """
    deg = SECTOR_DEG[sector]
    if deg != 0:
        return turn_degrees(deg, imu_queue, tag_queue=tag_queue, target_id=TARGET_TAG)
    return False, []


# ---------------------------------------------------------------------------
# Tag helpers
# ---------------------------------------------------------------------------
def find_tag(tags, tag_id):
    for tag in tags:
        if tag.id == tag_id:
            return tag
    return None


def tag_centre_x_norm(tag):
    cx = (tag.topLeft.x + tag.topRight.x + tag.bottomLeft.x + tag.bottomRight.x) / 4.0
    return cx / TAG_INPUT_W


# ---------------------------------------------------------------------------
# Ultrasonic helpers
# ---------------------------------------------------------------------------
def read_all_sensors(mux, sensor):
    """Read all 5 ultrasonic sensors."""
    readings = {}
    for name, channel in SENSOR_CHANNELS.items():
        mux.select(channel)
        readings[name] = sensor.read_mm()
    mux.disable()
    return readings


# ---------------------------------------------------------------------------
# Obstacle sensing — front stop vs side steer
# ---------------------------------------------------------------------------
# Actions returned by check_obstacles()
OBS_CLEAR      = "clear"
OBS_STOP       = "stop"       # front/front_left/front_right or depth blocked
OBS_STEER_LEFT  = "steer_left"   # obstacle on RIGHT → nudge left
OBS_STEER_RIGHT = "steer_right"  # obstacle on LEFT  → nudge right

SIDE_STEER_FACTOR = 0.45   # inner track at 45% speed during side correction
CENTER_TOL = 0.08          # normalised offset tolerance — tag counts as "centred"
FACE_SPEED = 200           # slow spin speed used when centering on a tag


def depth_centre_mm(q):
    """Median depth of the centre 40% of the depth frame. Returns 9999 if no data."""
    msg = q["depth"].tryGet()
    if msg is None:
        return 9999
    depth = msg.getFrame()
    _, w = depth.shape[:2]
    strip = depth[:, int(w * 0.3):int(w * 0.7)].flatten()
    valid = strip[(strip > DEPTH_MIN_MM) & (strip < DEPTH_MAX_MM)]
    return float(np.median(valid)) if len(valid) > 0 else 9999


def check_obstacles(mux, sensor, q):
    """Read all sensors and return (action, reason).

    Front sensors (ultrasonic + depth + NN) → OBS_STOP
    Side sensors only → OBS_STEER_LEFT / OBS_STEER_RIGHT
    Nothing close    → OBS_CLEAR
    """
    readings = read_all_sensors(mux, sensor)

    def near(name):
        d = readings.get(name)
        return d is not None and d < OBSTACLE_MM

    front_blocked = near("front") or near("front_left") or near("front_right")

    # Depth and NN also count as front obstacles
    if not front_blocked:
        if depth_centre_mm(q) < OBSTACLE_MM:
            front_blocked = True
        else:
            tracked = get_tracked_obstacles(q)
            for _tid, _label, x_mm, z_mm, _status in tracked:
                if z_mm > 0 and z_mm < OBSTACLE_MM and abs(x_mm) < 400:
                    front_blocked = True
                    break

    if front_blocked:
        sides = ""
        if near("left"):
            sides += "L"
        if near("right"):
            sides += "R"
        reason = f"front blocked (sides:{sides or '-'})"
        return OBS_STOP, reason

    # Front is clear — check sides for steering correction only
    left_near  = near("left")
    right_near = near("right")

    if left_near and not right_near:
        return OBS_STEER_RIGHT, f"left {readings['left']}mm"
    if right_near and not left_near:
        return OBS_STEER_LEFT, f"right {readings['right']}mm"
    if left_near and right_near:
        # Narrow gap — keep going straight, let front sensors decide
        return OBS_CLEAR, "narrow"

    return OBS_CLEAR, ""


def drive_with_correction(action, speed=SPEED):
    """Drive forward, steering away from whichever side has an obstacle."""
    fast = int(speed)
    slow = int(speed * (1.0 - SIDE_STEER_FACTOR))
    if action == OBS_STEER_LEFT:
        # Obstacle on right → slow right track → arc left
        IIC.control_pwm(-fast, 0, -slow, 0)
    elif action == OBS_STEER_RIGHT:
        # Obstacle on left → slow left track → arc right
        IIC.control_pwm(-slow, 0, -fast, 0)
    else:
        drive_forward(speed)


def path_is_blocked(mux, sensor, q):
    """Convenience wrapper returning (blocked: bool, reason: str)."""
    action, reason = check_obstacles(mux, sensor, q)
    return action == OBS_STOP, reason


def face_tag(tag_id, q, timeout=4.0):
    """Rotate in place until tag_id is centred in the camera frame.

    The camera is mounted forward-centre, so centred means the tag's
    horizontal offset is within CENTER_TOL of the frame midpoint.

    Returns True if centred, False if the tag is lost or timeout expires.
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = q["tags"].tryGet()
        if msg is None:
            time.sleep(0.02)
            continue
        tags = list(msg.aprilTags)
        tag = find_tag(tags, tag_id)
        if tag is None:
            stop_motors()
            return False
        offset = tag_centre_x_norm(tag) - 0.5
        if abs(offset) < CENTER_TOL:
            stop_motors()
            return True
        # Spin toward the tag: offset > 0 means tag is right of centre → CW
        (spin_cw if offset > 0 else spin_ccw)(FACE_SPEED)
        time.sleep(0.01)
    stop_motors()
    return False


def person_ahead(q):
    """Check if a person is close and roughly ahead.  Returns z_mm or None."""
    tracked = get_tracked_obstacles(q)
    for _tid, label_name, x_mm, z_mm, _status in tracked:
        if label_name == "person" and z_mm > 0 and z_mm < PERSON_STOP_MM and abs(x_mm) < 500:
            return z_mm
    return None


# ---------------------------------------------------------------------------
# Camera frame helper
# ---------------------------------------------------------------------------
def poll_camera(q):
    tag_msg = q["tags"].get()
    tags = list(tag_msg.aprilTags)
    if tags:
        send_roi_configs(tags, q["spatial_cfg"])
    spatial_msg = q["spatial"].get()
    spatial_locations = spatial_msg.getSpatialLocations()
    return tags, spatial_locations


def get_target_depth(prev_tags, spatial_locations):
    for i, pt in enumerate(prev_tags):
        if pt.id == TARGET_TAG and i < len(spatial_locations):
            loc = spatial_locations[i]
            if loc.spatialCoordinates.z > 0:
                return loc.spatialCoordinates.z
    return None


# ---------------------------------------------------------------------------
# Depth-sector analysis
# ---------------------------------------------------------------------------
def find_clearest_sector(q, tags=None):
    """Read depth frame, split into sectors, return (best_index, medians).

    Penalises sectors containing AVOID_TAG.
    """
    depth_frame = q["depth"].get()
    depth = depth_frame.getFrame()

    _, w = depth.shape[:2]
    sector_w = w // NUM_SECTORS
    medians = []

    for s in range(NUM_SECTORS):
        c0 = s * sector_w
        c1 = c0 + sector_w if s < NUM_SECTORS - 1 else w
        strip = depth[:, c0:c1].flatten()
        valid = strip[(strip > DEPTH_MIN_MM) & (strip < DEPTH_MAX_MM)]
        medians.append(float(np.median(valid)) if len(valid) > 0 else 0.0)

    if tags:
        avoid = find_tag(tags, AVOID_TAG)
        if avoid is not None:
            acx = tag_centre_x_norm(avoid)
            a_sec = min(int(acx * NUM_SECTORS), NUM_SECTORS - 1)
            medians[a_sec] = 0.0
            if a_sec > 0:
                medians[a_sec - 1] *= 0.3
            if a_sec < NUM_SECTORS - 1:
                medians[a_sec + 1] *= 0.3
            print(f"[DEPTH]  Tag {AVOID_TAG} in sector {a_sec} — penalised")

    best = int(np.argmax(medians))
    return best, medians


# ---------------------------------------------------------------------------
# Search: depth-first navigation — read depth, face clearest path, go
# ---------------------------------------------------------------------------
SECTOR_NAMES = ["FAR-L", "LEFT", "CENTRE", "RIGHT", "FAR-R"]


def search_for_tag(pipeline, q, mux, sensor):
    """Depth-guided search: read the depth map, turn to face the clearest
    path, drive until blocked or time expires, then re-evaluate.

    No blind spinning — movement is always toward the most open space.
    A brief spin only happens when stuck or every SPIN_EVERY_N cycles to
    check directions not in the current field of view.

    Returns (tags, prev_tags, spatial_locations) from the frame where
    TARGET_TAG was found so the caller can resume tracking.
    """
    cycle = 0
    prev_tags = []
    prev_features = {}

    while pipeline.isRunning():
        cycle += 1

        # --- Step 0: if facing tag 0, turn away first ---
        # The depth map only sees the camera's FOV. If tag 0 is ahead,
        # the clear path is behind us — invisible until we turn around.
        tags, spatial_locations = poll_camera(q)
        if find_tag(tags, TARGET_TAG) is not None:
            stop_motors()
            print(f"[SEARCH]  Tag {TARGET_TAG} found before moving!")
            face_tag(TARGET_TAG, q)
            return tags, prev_tags, spatial_locations

        avoid = find_tag(tags, AVOID_TAG)
        if avoid is not None:
            acx = tag_centre_x_norm(avoid)
            # Tag 0 is in frame — turn away from it so the depth map
            # can see the direction we actually want to go
            if acx < 0.5:
                # tag 0 is on the left → turn right (away)
                turn_deg = 90 + int((0.5 - acx) * 180)  # 90°–180°
            else:
                # tag 0 is on the right → turn left (away)
                turn_deg = -(90 + int((acx - 0.5) * 180))
            print(f"[SEARCH]  Tag {AVOID_TAG} visible (x={acx:.2f}) — turning {turn_deg}° away")
            found_mid, mid_tags = turn_degrees(
                turn_deg, q["imu"], tag_queue=q["tags"], target_id=TARGET_TAG
            )
            if found_mid:
                print(f"[SEARCH]  Tag {TARGET_TAG} found mid-turn!")
                face_tag(TARGET_TAG, q)
                return mid_tags, prev_tags, []
            # Re-read camera after turning
            tags, spatial_locations = poll_camera(q)
            if find_tag(tags, TARGET_TAG) is not None:
                stop_motors()
                print(f"[SEARCH]  Tag {TARGET_TAG} found after turning!")
                face_tag(TARGET_TAG, q)
                return tags, prev_tags, spatial_locations
            prev_tags = tags

            # After turning away from tag 0, check if center is now clear.
            # If so, commit to advancing immediately rather than re-running
            # sector analysis which might steer back toward a suboptimal angle.
            centre_depth = depth_centre_mm(q)
            if centre_depth > OBSTACLE_MM:
                print(f"[SEARCH]  Centre clear ({centre_depth:.0f} mm) after turn — advancing directly")
                advance_start = time.time()
                while time.time() - advance_start < ADVANCE_TIME and pipeline.isRunning():
                    action, reason = check_obstacles(mux, sensor, q)
                    if action == OBS_STOP:
                        print(f"[AVOID]  {reason} — reversing...")
                        stop_motors()
                        reverse()
                        time.sleep(REVERSE_TIME)
                        stop_motors()
                        break
                    drive_with_correction(action, SPEED)
                    tags, spatial_locations = poll_camera(q)
                    if find_tag(tags, TARGET_TAG) is not None:
                        stop_motors()
                        print(f"[ADVANCE]  Tag {TARGET_TAG} found!")
                        face_tag(TARGET_TAG, q)
                        return tags, prev_tags, spatial_locations
                    avoid2 = find_tag(tags, AVOID_TAG)
                    if avoid2 is not None and 0.2 < tag_centre_x_norm(avoid2) < 0.8:
                        print(f"[ADVANCE]  Tag {AVOID_TAG} ahead — re-evaluating...")
                        stop_motors()
                        break
                    prev_tags = tags
                    time.sleep(POLL_INTERVAL)
                stop_motors()
                continue  # re-evaluate from top of loop

        # --- Step 1: read depth to find clearest direction from here ---

        best, medians = find_clearest_sector(q, tags=tags)
        best_depth = medians[best]
        med_str = "  ".join(
            f"{SECTOR_NAMES[i]}:{medians[i]:.0f}" for i in range(NUM_SECTORS)
        )
        print(f"[DEPTH]  {med_str}  -> {SECTOR_NAMES[best]} ({best_depth:.0f} mm)")
        prev_tags = tags

        # If all paths are shallow, do a brief spin to look for a way out
        if best_depth < OBSTACLE_MM:
            direction = 1 if cycle % 2 == 0 else -1
            print(f"[SEARCH]  All paths blocked — spinning to find opening...")
            (spin_cw if direction > 0 else spin_ccw)(SEARCH_SPEED)
            t0 = time.time()
            while time.time() - t0 < SCAN_TIME and pipeline.isRunning():
                tags, spatial_locations = poll_camera(q)
                if find_tag(tags, TARGET_TAG) is not None:
                    stop_motors()
                    print(f"[SEARCH]  Tag {TARGET_TAG} found while spinning!")
                    face_tag(TARGET_TAG, q)
                    return tags, prev_tags, spatial_locations
                prev_tags = tags
                time.sleep(POLL_INTERVAL)
            stop_motors()
            continue  # re-evaluate depth after spin

        # --- Step 2: turn to face the clearest sector ---
        found_mid, mid_tags = turn_to_sector(best, q["imu"], tag_queue=q["tags"])
        if found_mid:
            print(f"[SEARCH]  Tag {TARGET_TAG} spotted mid-sector-turn!")
            face_tag(TARGET_TAG, q)
            return mid_tags, prev_tags, []

        # --- Step 3: drive toward it, re-evaluating depth every few seconds ---
        print(f"[ADVANCE]  Heading {SECTOR_NAMES[best]}, depth {best_depth:.0f} mm...")
        advance_start = time.time()
        stuck_count = 0

        while time.time() - advance_start < ADVANCE_TIME and pipeline.isRunning():
            # Re-read depth every 2s to steer toward best open path
            elapsed = time.time() - advance_start
            if elapsed > 0 and elapsed % 2.0 < POLL_INTERVAL:
                new_best, new_medians = find_clearest_sector(q, tags=prev_tags)
                new_depth = new_medians[new_best]
                if new_best != best:
                    print(f"[DEPTH]  Redirecting: {SECTOR_NAMES[best]} -> {SECTOR_NAMES[new_best]} ({new_depth:.0f} mm)")
                    stop_motors()
                    found_mid, mid_tags = turn_to_sector(new_best, q["imu"], tag_queue=q["tags"])
                    if found_mid:
                        print(f"[ADVANCE]  Tag {TARGET_TAG} spotted mid-redirect-turn!")
                        face_tag(TARGET_TAG, q)
                        return mid_tags, prev_tags, []
                    best = new_best
                # If path has closed up ahead, bail and re-evaluate
                if new_depth < OBSTACLE_MM:
                    print(f"[DEPTH]  Path closing — re-evaluating...")
                    stop_motors()
                    break

            # Combined obstacle check (ultrasonic + depth + NN)
            action, reason = check_obstacles(mux, sensor, q)
            if action == OBS_STOP:
                print(f"[AVOID]  {reason} — reversing...")
                stop_motors()
                reverse()
                time.sleep(REVERSE_TIME)
                stop_motors()
                break

            drive_with_correction(action, SPEED)

            # Check camera for target tag
            tags, spatial_locations = poll_camera(q)

            if find_tag(tags, TARGET_TAG) is not None:
                stop_motors()
                print(f"[ADVANCE]  Tag {TARGET_TAG} found!")
                face_tag(TARGET_TAG, q)
                return tags, prev_tags, spatial_locations

            # Abort if heading back toward tag 0
            avoid = find_tag(tags, AVOID_TAG)
            if avoid is not None and 0.2 < tag_centre_x_norm(avoid) < 0.8:
                print(f"[ADVANCE]  Tag {AVOID_TAG} ahead — wrong way, re-evaluating...")
                stop_motors()
                break

            # Stuck detection via feature tracker
            disp, prev_features = compute_feature_displacement(
                prev_features, q["features"]
            )
            if disp < STUCK_DISP_PX:
                stuck_count += 1
            else:
                stuck_count = 0

            if stuck_count >= STUCK_FRAMES:
                print(f"[STUCK]  Not moving — reversing and re-evaluating...")
                stop_motors()
                reverse()
                time.sleep(STUCK_REVERSE_TIME)
                stop_motors()
                stuck_count = 0
                break

            prev_tags = tags
            time.sleep(POLL_INTERVAL)

        stop_motors()

    return [], [], []


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Initialising motor parameters...")
    IIC.set_motor_parameter()
    stop_motors()

    bus = smbus2.SMBus(1)
    mux = Mux(bus)
    us_sensor = UltrasonicSensor(bus)

    print(f"Target: AprilTag {TARGET_TAG}")
    print(f"Arrive threshold: {ARRIVE_MM} mm ({ARRIVE_MM / MM_PER_INCH:.0f} in)")
    print("Building enhanced camera pipeline...")

    pipeline, q = build_enhanced_pipeline()

    with pipeline:
        pipeline.start()
        print("Pipeline started.\n")

        # --- Depth calibration ---
        # Position the robot facing the clearest available path, then press
        # Enter.  The robot captures N depth frames and learns what "clear"
        # looks like in this environment, then sets OBSTACLE_MM accordingly.
        print("=" * 60)
        print("CALIBRATION")
        print("Position the robot facing the clearest available path,")
        print("then press Enter to capture the depth reference.")
        print("=" * 60)
        input()
        print("Capturing depth reference — hold still...")

        CALIBRATION_FRAMES = 15
        sector_samples = [[] for _ in range(NUM_SECTORS)]
        for _ in range(CALIBRATION_FRAMES):
            _, medians = find_clearest_sector(q)
            for s in range(NUM_SECTORS):
                if medians[s] > 0:
                    sector_samples[s].append(medians[s])
            time.sleep(0.1)

        calibrated = []
        for s in range(NUM_SECTORS):
            vals = sector_samples[s]
            calibrated.append(sum(vals) / len(vals) if vals else 0.0)

        # Clearest calibrated sector = reference for "open corridor"
        clear_ref_mm = max(calibrated)
        # Treat anything below 50% of the reference as blocked.
        # This is a module-level assignment — check_obstacles / depth_centre_mm
        # will pick it up on their next call.
        OBSTACLE_MM = clear_ref_mm * 0.50

        print(f"\nCalibrated sector depths (mm):")
        for i in range(NUM_SECTORS):
            print(f"  {SECTOR_NAMES[i]:8s}: {calibrated[i]:.0f} mm")
        print(f"\nClear-path reference : {clear_ref_mm:.0f} mm")
        print(f"Obstacle threshold   : {OBSTACLE_MM:.0f} mm  (50% of reference)")
        print("=" * 60)
        print("Navigating to tag 1.  Ctrl+C to quit\n")

        prev_tags = []
        prev_features = {}
        stuck_count = 0

        try:
            while pipeline.isRunning():
                tags, spatial_locations = poll_camera(q)
                target_z_mm = get_target_depth(prev_tags, spatial_locations)
                target = find_tag(tags, TARGET_TAG)

                if target is not None:
                    cx_norm = tag_centre_x_norm(target)
                    offset = cx_norm - 0.5

                    # --- Arrived ---
                    if target_z_mm is not None and target_z_mm < ARRIVE_MM:
                        stop_motors()
                        print(f"[ARRIVED]  Tag {TARGET_TAG} at {target_z_mm:.0f} mm — done!")
                        break

                    # --- Person in path: yield ---
                    pz = person_ahead(q)
                    if pz is not None:
                        stop_motors()
                        print(f"[YIELD]  Person at {pz:.0f} mm — waiting...")
                        yield_start = time.time()
                        while time.time() - yield_start < PERSON_YIELD_SEC:
                            pz = person_ahead(q)
                            if pz is None:
                                print("[YIELD]  Path clear, resuming")
                                break
                            time.sleep(0.2)
                        else:
                            # Person didn't move — try steering around
                            print("[YIELD]  Person still there — steering around")
                            turn_degrees(45, q["imu"], tag_queue=q["tags"], target_id=TARGET_TAG)
                        prev_tags = tags
                        continue

                    # --- Obstacle check while tracking ---
                    obs_action, obs_reason = check_obstacles(mux, us_sensor, q)
                    if obs_action == OBS_STOP:
                        stop_motors()
                        print(f"[TRACK]  Obstacle ({obs_reason}) — stopping, waiting...")
                        # Wait briefly for path to clear (e.g. person walking by)
                        time.sleep(0.5)
                        prev_tags = tags
                        continue

                    # --- Proportional tracking with side-obstacle correction ---
                    if target_z_mm is not None and target_z_mm < SLOW_MM:
                        speed = SLOW_SPEED
                    else:
                        speed = SPEED
                    if obs_action in (OBS_STEER_LEFT, OBS_STEER_RIGHT):
                        # Side obstacle takes priority over tag offset steering
                        drive_with_correction(obs_action, speed)
                    else:
                        drive_toward(offset, speed)
                    dist_str = f"{target_z_mm:.0f} mm" if target_z_mm else "pending"
                    print(f"[TRACK]  offset={offset:+.2f}  dist={dist_str}  speed={speed}")

                else:
                    # --- Tag not visible — search ---
                    stop_motors()
                    tags, prev_tags, spatial_locations = search_for_tag(
                        pipeline, q, mux, us_sensor
                    )
                    target = find_tag(tags, TARGET_TAG)
                    if target is None:
                        break  # pipeline stopped

                prev_tags = tags
                time.sleep(POLL_INTERVAL)

        except KeyboardInterrupt:
            pass
        finally:
            stop_motors()
            mux.disable()
            pipeline.stop()
            print("\nMotors stopped. Mux disabled. Pipeline closed.")
