"""
Enhanced OAK-D-Lite pipeline with all available hardware features.

Nodes:
  - RGB camera  -> AprilTag 36h11 detector
  - RGB camera  -> YOLOv6-nano detection network (auto-downloaded from Luxonis Hub)
  - Detections  -> Object tracker with persistent IDs
  - Mono pair   -> StereoDepth -> depth map
  - Depth       -> SpatialLocationCalculator (per-tag distance)
  - Left mono   -> FeatureTracker (visual odometry)
  - IMU         -> raw gyroscope (yaw integration) + raw accelerometer

The YOLOv6-nano model is auto-downloaded from Luxonis Hub on first run
(requires internet once, then cached).  If the download fails, the pipeline
still builds — the ``detections`` and ``tracklets`` queues will be absent.
"""

import math
import depthai as dai
from sensors.apriltag import tag_to_roi, send_roi_configs  # noqa: F401 — re-export

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MM_PER_INCH = 25.4
TAG_INPUT_W = 640
TAG_INPUT_H = 480
MONO_W = 640
MONO_H = 400
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 10000
IMU_RATE_HZ = 100

# YOLOv6-nano COCO labels (80 classes)
LABEL_MAP = None  # populated at runtime from model via getClasses()

# COCO label names the robot should treat as obstacles (matched by string)
OBSTACLE_NAMES = {"person", "chair", "couch", "potted plant", "dining table",
                  "dog", "cat", "bicycle", "suitcase"}

# Luxonis Hub model slug — auto-downloaded by depthai v3
# Two slugs to try — 512x384 may have a different compiled shave count
YOLO_MODEL_SLUGS = [
    "luxonis/yolov10-nano:coco-512x288",
    "luxonis/yolov6-nano:r2-coco-512x384",  # fallback
    "luxonis/yolov6-nano:r2-coco-512x288",  # fallback
]


# ---------------------------------------------------------------------------
# Pipeline builder
# ---------------------------------------------------------------------------
def build_enhanced_pipeline():
    """Build the full OAK-D-Lite pipeline.

    Returns (pipeline, queues).

    Queues always present:
        tags, spatial, spatial_cfg, depth, imu, features

    Queues present when YOLO model loads successfully:
        detections, tracklets

    Construction order matters for CMX resource allocation: the YOLO
    detection network is created before StereoDepth so it claims CMX
    slices [0-7] first, pushing StereoDepth to higher slots.
    """
    global LABEL_MAP

    pipeline = dai.Pipeline()

    # --- RGB camera ---
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    tag_out = camRgb.requestOutput((TAG_INPUT_W, TAG_INPUT_H))

    # --- YOLOv6-nano: created FIRST so it wins CMX [0-7] before StereoDepth ---
    _setup_yolo(pipeline, camRgb)

    # --- Mono cameras ---
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # --- Stereo depth (created after YOLO, so it gets CMX [8+]) ---
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setRectification(True)
    monoLeft.requestOutput((MONO_W, MONO_H)).link(stereo.left)
    monoRight.requestOutput((MONO_W, MONO_H)).link(stereo.right)

    # --- AprilTag detector ---
    aprilTag = pipeline.create(dai.node.AprilTag)
    aprilTag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_36H11)
    tag_out.link(aprilTag.inputImage)

    # --- Spatial location calculator (per-tag depth) ---
    spatialCalc = pipeline.create(dai.node.SpatialLocationCalculator)
    stereo.depth.link(spatialCalc.inputDepth)
    spatialCalc.inputConfig.setWaitForMessage(False)
    default_cfg = dai.SpatialLocationCalculatorConfigData()
    default_cfg.depthThresholds.lowerThreshold = DEPTH_MIN_MM
    default_cfg.depthThresholds.upperThreshold = DEPTH_MAX_MM
    default_cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
    default_cfg.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
    spatialCalc.initialConfig.addROI(default_cfg)

    # --- IMU (BMI270: raw gyro + accel only, no rotation vector) ---
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, IMU_RATE_HZ)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_RATE_HZ)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # --- Feature tracker (visual odometry on left mono) ---
    featureTracker = pipeline.create(dai.node.FeatureTracker)
    monoLeft.requestOutput((MONO_W, MONO_H)).link(featureTracker.inputImage)

    queues = {
        "tags": aprilTag.out.createOutputQueue(),
        "spatial": spatialCalc.out.createOutputQueue(),
        "spatial_cfg": spatialCalc.inputConfig.createInputQueue(),
        "depth": stereo.depth.createOutputQueue(),
        "imu": imu.out.createOutputQueue(),
        "features": featureTracker.outputFeatures.createOutputQueue(),
    }

    # Add YOLO queues if setup succeeded
    if _YOLO_NODE is not None:
        queues["detections"] = _YOLO_NODE["detections"]
        queues["tracklets"] = _YOLO_NODE["tracklets"]

    return pipeline, queues


# Holds YOLO output queues between _setup_yolo() and build_enhanced_pipeline()
_YOLO_NODE = None


def _setup_yolo(pipeline, camRgb):
    """Create YOLO + tracker nodes early in the pipeline build.

    Stores output queues in the module-level _YOLO_NODE dict.
    Tries both model slugs; silently skips if both fail.
    """
    global LABEL_MAP, _YOLO_NODE
    _YOLO_NODE = None

    for slug in YOLO_MODEL_SLUGS:
        try:
            print(f"[PIPELINE] Loading {slug} from Luxonis Hub...")
            detNet = pipeline.create(dai.node.DetectionNetwork).build(
                camRgb, dai.NNModelDescription(slug)
            )
            LABEL_MAP = detNet.getClasses()
            print(f"[PIPELINE] YOLO loaded — {len(LABEL_MAP) if LABEL_MAP else '?'} classes")

            _build_obstacle_index()

            tracker = pipeline.create(dai.node.ObjectTracker)
            tracker.setDetectionLabelsToTrack(list(OBSTACLE_INDEX.keys()))
            tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
            tracker.setTrackerIdAssignmentPolicy(
                dai.TrackerIdAssignmentPolicy.SMALLEST_ID
            )
            detNet.passthrough.link(tracker.inputTrackerFrame)
            detNet.passthrough.link(tracker.inputDetectionFrame)
            detNet.out.link(tracker.inputDetections)

            _YOLO_NODE = {
                "detections": detNet.out.createOutputQueue(),
                "tracklets": tracker.out.createOutputQueue(),
            }
            print(f"[PIPELINE] YOLOv6-nano + Object Tracker enabled ({slug})")
            return
        except Exception as e:
            print(f"[PIPELINE] {slug} failed: {e}")

    print("[PIPELINE] YOLO unavailable — running without object detection")


# ---------------------------------------------------------------------------
# Obstacle label index (built at runtime from model classes)
# ---------------------------------------------------------------------------
OBSTACLE_INDEX = {}  # {label_int: class_name} for obstacle classes


def _build_obstacle_index():
    """Populate OBSTACLE_INDEX from LABEL_MAP + OBSTACLE_NAMES."""
    global OBSTACLE_INDEX
    OBSTACLE_INDEX = {}
    if LABEL_MAP is None:
        return
    for i, name in enumerate(LABEL_MAP):
        if name in OBSTACLE_NAMES:
            OBSTACLE_INDEX[i] = name


# ---------------------------------------------------------------------------
# IMU helpers — gyroscope yaw integration
# ---------------------------------------------------------------------------
# The OAK-D-Lite BMI270 only supports GYROSCOPE_RAW and ACCELEROMETER_RAW.
# We integrate the gyro z-axis (yaw rate in rad/s) to get heading change.
# This drifts over time but is accurate enough for turns of a few seconds.


class GyroTracker:
    """Integrates BMI270 raw gyroscope z-axis to track heading changes."""

    def __init__(self):
        self.heading_deg = 0.0      # accumulated heading since construction
        self._prev_ts = None        # previous packet timestamp (seconds)

    def update(self, imu_queue):
        """Drain all available gyro packets and integrate.

        Returns current accumulated heading in degrees.
        """
        while True:
            msg = imu_queue.tryGet()
            if msg is None:
                break
            for packet in msg.packets:
                gyro = packet.gyroscope
                # gyro.z = yaw rate in rad/s (positive = CCW when viewed from above)
                ts = packet.gyroscope.getTimestampDevice().total_seconds()
                if self._prev_ts is not None:
                    dt = ts - self._prev_ts
                    if 0 < dt < 0.5:  # ignore stale/huge gaps
                        # Convert rad/s -> deg/s, integrate
                        self.heading_deg += math.degrees(gyro.z) * dt
                self._prev_ts = ts
        return self.heading_deg

    def reset(self):
        """Reset accumulated heading to zero."""
        self.heading_deg = 0.0
        self._prev_ts = None


def normalize_angle(deg):
    """Normalize angle to (-180, 180]."""
    deg = deg % 360
    if deg > 180:
        deg -= 360
    return deg


def angle_diff(target, current):
    """Shortest signed angle from *current* to *target*, in degrees."""
    return normalize_angle(target - current)


# ---------------------------------------------------------------------------
# Feature tracker helpers
# ---------------------------------------------------------------------------
def compute_feature_displacement(prev_features, feature_queue):
    """Compute average pixel displacement of tracked features between frames.

    prev_features: dict {feature_id: (x, y)} from the previous frame.
    feature_queue: depthai output queue for FeatureTracker.

    Returns (avg_displacement_px, new_features_dict).
    """
    msg = feature_queue.tryGet()
    if msg is None:
        return 0.0, prev_features

    curr = {}
    for f in msg.trackedFeatures:
        curr[f.id] = (f.position.x, f.position.y)

    if not prev_features or not curr:
        return 0.0, curr

    displacements = []
    for fid, (cx, cy) in curr.items():
        if fid in prev_features:
            px, py = prev_features[fid]
            displacements.append(math.sqrt((cx - px) ** 2 + (cy - py) ** 2))

    avg = sum(displacements) / len(displacements) if displacements else 0.0
    return avg, curr


# ---------------------------------------------------------------------------
# Spatial detection helpers
# ---------------------------------------------------------------------------
def get_obstacle_detections(queues):
    """Read spatial detections of obstacles.

    Returns list of (label_name, x_mm, z_mm, confidence).
    Returns [] if NN is not available or no detections this frame.
    """
    if "detections" not in queues:
        return []
    msg = queues["detections"].tryGet()
    if msg is None:
        return []
    obstacles = []
    for det in msg.detections:
        if det.label in OBSTACLE_INDEX:
            obstacles.append((
                OBSTACLE_INDEX[det.label],
                det.spatialCoordinates.x,
                det.spatialCoordinates.z,
                det.confidence,
            ))
    return obstacles


def get_tracked_obstacles(queues):
    """Read tracked obstacles with persistent IDs.

    Returns list of (tracker_id, label_name, x_mm, z_mm, status_str).
    Returns [] if tracker is not available.
    """
    if "tracklets" not in queues:
        return []
    msg = queues["tracklets"].tryGet()
    if msg is None:
        return []
    tracked = []
    for t in msg.tracklets:
        if t.label in OBSTACLE_INDEX and t.status != dai.Tracklet.TrackingStatus.LOST:
            tracked.append((
                t.id,
                OBSTACLE_INDEX[t.label],
                t.spatialCoordinates.x,
                t.spatialCoordinates.z,
                t.status.name,
            ))
    return tracked
