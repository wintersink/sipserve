#!/usr/bin/env python3
"""AprilTag detection with distance estimation using the OAK-D-Lite stereo camera.

Detects 36h11 AprilTags, reports each tag's ID and distance in inches using
the stereo depth from the OAK-D-Lite. Console output only — no GUI.

Usage:
    source venv4sipserve/bin/activate
    python sensors/apriltag.py
"""

import depthai as dai

MM_PER_INCH = 25.4

# AprilTag detection resolution (fed to the AprilTag node)
TAG_INPUT_W = 640
TAG_INPUT_H = 480

# Mono camera resolution for stereo depth
MONO_W = 640
MONO_H = 400

# Depth thresholds (mm)
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 10000


def build_pipeline():
    """Build and return a depthai v3 pipeline for AprilTag + depth."""
    pipeline = dai.Pipeline()

    # --- RGB camera (for AprilTag detection + preview) ---
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgbOut = camRgb.requestOutput((TAG_INPUT_W, TAG_INPUT_H))

    # --- Stereo pair (mono left + right) ---
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # --- Stereo depth ---
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    monoLeft.requestOutput((MONO_W, MONO_H)).link(stereo.left)
    monoRight.requestOutput((MONO_W, MONO_H)).link(stereo.right)

    # --- AprilTag detector ---
    aprilTag = pipeline.create(dai.node.AprilTag)
    aprilTag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_36H11)
    rgbOut.link(aprilTag.inputImage)

    # --- Spatial location calculator (depth-based distance per ROI) ---
    spatialCalc = pipeline.create(dai.node.SpatialLocationCalculator)
    stereo.depth.link(spatialCalc.inputDepth)
    spatialCalc.inputConfig.setWaitForMessage(False)

    # Default ROI (overwritten at runtime for each detected tag)
    default_cfg = dai.SpatialLocationCalculatorConfigData()
    default_cfg.depthThresholds.lowerThreshold = DEPTH_MIN_MM
    default_cfg.depthThresholds.upperThreshold = DEPTH_MAX_MM
    default_cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
    default_cfg.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
    spatialCalc.initialConfig.addROI(default_cfg)

    # --- Output / input queues ---
    queues = {
        "tags": aprilTag.out.createOutputQueue(),
        "spatial": spatialCalc.out.createOutputQueue(),
        "spatial_cfg": spatialCalc.inputConfig.createInputQueue(),
        "depth": stereo.depth.createOutputQueue(),
    }

    return pipeline, queues


def tag_to_roi(tag):
    """Convert tag corner pixels to a normalized ROI for the depth map."""
    min_x = min(tag.topLeft.x, tag.bottomLeft.x) / TAG_INPUT_W
    max_x = max(tag.topRight.x, tag.bottomRight.x) / TAG_INPUT_W
    min_y = min(tag.topLeft.y, tag.topRight.y) / TAG_INPUT_H
    max_y = max(tag.bottomLeft.y, tag.bottomRight.y) / TAG_INPUT_H

    # Clamp to [0, 1]
    min_x = max(0.001, min(0.999, min_x))
    max_x = max(0.001, min(0.999, max_x))
    min_y = max(0.001, min(0.999, min_y))
    max_y = max(0.001, min(0.999, max_y))

    return min_x, min_y, max_x, max_y


def send_roi_configs(tags, spatial_cfg_queue):
    """Send one SpatialLocationCalculator config per detected tag."""
    cfg = dai.SpatialLocationCalculatorConfig()
    for tag in tags:
        min_x, min_y, max_x, max_y = tag_to_roi(tag)
        roi_cfg = dai.SpatialLocationCalculatorConfigData()
        roi_cfg.depthThresholds.lowerThreshold = DEPTH_MIN_MM
        roi_cfg.depthThresholds.upperThreshold = DEPTH_MAX_MM
        roi_cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        roi_cfg.roi = dai.Rect(dai.Point2f(min_x, min_y), dai.Point2f(max_x, max_y))
        cfg.addROI(roi_cfg)
    spatial_cfg_queue.send(cfg)


def main():
    pipeline, q = build_pipeline()

    with pipeline:
        pipeline.start()
        print("AprilTag detector running — press Ctrl+C to quit")

        prev_tags = []

        try:
            while pipeline.isRunning():
                # --- Get AprilTag detections ---
                tag_msg = q["tags"].get()
                tags = list(tag_msg.aprilTags)

                # --- Send ROI configs for current tags ---
                if tags:
                    send_roi_configs(tags, q["spatial_cfg"])

                # --- Read spatial data (corresponds to previous frame's ROIs) ---
                spatial_msg = q["spatial"].get()
                spatial_locations = spatial_msg.getSpatialLocations()

                # Match spatial results to the previous frame's tags
                distances = {}
                for i, loc in enumerate(spatial_locations):
                    z_mm = loc.spatialCoordinates.z
                    if z_mm > 0 and i < len(prev_tags):
                        distances[i] = z_mm / MM_PER_INCH

                # --- Report detections ---
                for i, tag in enumerate(tags):
                    dist = distances.get(i)
                    if dist is not None:
                        print(f"  Tag {tag.id}: {dist:.1f} inches")
                    else:
                        print(f"  Tag {tag.id}: detected (distance pending)")

                prev_tags = tags
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            pipeline.stop()


if __name__ == "__main__":
    main()
