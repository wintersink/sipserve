"""
stream_camera.py — Live MJPEG browser stream from OAK-D-Lite.

Starts an HTTP server on port 8080.  Open this URL on any device on the
same network:

    http://sipserve-johann:8080

Shows the RGB feed (with AprilTag overlays) and depth colourmap side-by-side.

Ctrl+C to stop.
"""

import sys
import os
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

PORT = 8080
RGB_W, RGB_H = 640, 480
MONO_W, MONO_H = 640, 400
DEPTH_MAX_MM = 6000   # beyond this = far background, shown as same colour

YOLO_SLUG = "luxonis/yolov10-nano:coco-512x288"

# Colour per detection category (BGR)
COL_TABLE  = (0,   255, 255)  # yellow  — dining table
COL_OBS    = (0,   165, 255)  # orange  — other obstacle classes
COL_OTHER  = (255, 200,   0)  # cyan    — everything else

OBSTACLE_NAMES = {"person", "chair", "couch", "potted plant", "dining table",
                  "dog", "cat", "bicycle", "suitcase"}

# Populated from model at runtime
_label_map = []

# ---------------------------------------------------------------------------
# Shared state between camera thread and HTTP handler threads
# ---------------------------------------------------------------------------
_lock = threading.Lock()
_latest_jpeg = b""


# ---------------------------------------------------------------------------
# Pipeline  — YOLO created FIRST so it wins CMX [0-7] before StereoDepth
# ---------------------------------------------------------------------------
def build_pipeline():
    global _label_map
    pipeline = dai.Pipeline()

    # RGB camera
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb_out = camRgb.requestOutput((RGB_W, RGB_H))

    # --- YOLOv6-nano (must be before StereoDepth for CMX allocation) ---
    yolo_queue = None
    try:
        print(f"[STREAM]  Loading {YOLO_SLUG} ...")
        detNet = pipeline.create(dai.node.DetectionNetwork).build(
            camRgb, dai.NNModelDescription(YOLO_SLUG)
        )
        _label_map = detNet.getClasses() or []
        print(f"[STREAM]  YOLO loaded — {len(_label_map)} classes")
        yolo_queue = detNet.out.createOutputQueue(maxSize=1, blocking=False)
    except Exception as e:
        print(f"[STREAM]  YOLO unavailable ({e}) — detections disabled")

    # Mono pair + stereo depth (after YOLO so StereoDepth gets CMX [8+])
    monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setRectification(True)
    monoLeft.requestOutput((MONO_W, MONO_H)).link(stereo.left)
    monoRight.requestOutput((MONO_W, MONO_H)).link(stereo.right)

    # AprilTag detector
    aprilTag = pipeline.create(dai.node.AprilTag)
    aprilTag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_36H11)
    camRgb.requestOutput((RGB_W, RGB_H)).link(aprilTag.inputImage)

    queues = {
        "rgb":   rgb_out.createOutputQueue(),
        "depth": stereo.depth.createOutputQueue(maxSize=1, blocking=False),
        "tags":  aprilTag.out.createOutputQueue(maxSize=1, blocking=False),
        "yolo":  yolo_queue,   # None if model failed to load
    }
    return pipeline, queues


# ---------------------------------------------------------------------------
# Camera thread — grabs frames, builds composite JPEG, stores in _latest_jpeg
# ---------------------------------------------------------------------------
def camera_loop(pipeline, queues):
    global _latest_jpeg

    with pipeline:
        pipeline.start()
        print(f"[STREAM]  Pipeline running — open http://sipserve-johann:{PORT}")

        depth_color  = np.zeros((RGB_H, RGB_W, 3), dtype=np.uint8)
        tags_cache   = []
        dets_cache   = []

        while pipeline.isRunning():
            # --- RGB (blocking — drives our frame rate) ---
            rgb_msg = queues["rgb"].get()
            rgb = rgb_msg.getCvFrame()
            if rgb is None:
                continue
            # Ensure BGR 3-channel (getCvFrame may return grayscale or NV12)
            if rgb.ndim == 2:
                rgb = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
            elif rgb.shape[2] == 4:
                rgb = cv2.cvtColor(rgb, cv2.COLOR_BGRA2BGR)
            rgb = cv2.resize(rgb, (RGB_W, RGB_H))

            # --- Depth (non-blocking, keep last good frame) ---
            depth_msg = queues["depth"].tryGet()
            if depth_msg is not None:
                raw = depth_msg.getFrame().astype(np.float32)
                clipped = np.clip(raw, 0, DEPTH_MAX_MM)
                scaled  = (clipped / DEPTH_MAX_MM * 255).astype(np.uint8)
                coloured = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
                depth_color = cv2.resize(coloured, (RGB_W, RGB_H))

                # Overlay distance at depth frame centre
                cy, cx = raw.shape[0] // 2, raw.shape[1] // 2
                centre_strip = raw[cy - 20:cy + 20, cx - 20:cx + 20].flatten()
                valid = centre_strip[(centre_strip > 100) & (centre_strip < DEPTH_MAX_MM)]
                if len(valid):
                    dist_mm = int(np.median(valid))
                    cv2.putText(depth_color, f"{dist_mm} mm",
                                (RGB_W // 2 - 40, RGB_H // 2 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # --- YOLO detections (non-blocking, keep last set) ---
            if queues["yolo"] is not None:
                det_msg = queues["yolo"].tryGet()
                if det_msg is not None:
                    dets_cache = det_msg.detections

            for det in dets_cache:
                label = _label_map[det.label] if det.label < len(_label_map) else str(det.label)
                conf  = int(det.confidence * 100)
                x1 = int(det.xmin * RGB_W)
                y1 = int(det.ymin * RGB_H)
                x2 = int(det.xmax * RGB_W)
                y2 = int(det.ymax * RGB_H)

                if label == "dining table":
                    colour = COL_TABLE
                    thickness = 3
                elif label in OBSTACLE_NAMES:
                    colour = COL_OBS
                    thickness = 2
                else:
                    colour = COL_OTHER
                    thickness = 1

                cv2.rectangle(rgb, (x1, y1), (x2, y2), colour, thickness)
                text = f"{label} {conf}%"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(rgb, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour, -1)
                cv2.putText(rgb, text, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

            # --- AprilTags (non-blocking, keep last set) ---
            tag_msg = queues["tags"].tryGet()
            if tag_msg is not None:
                tags_cache = list(tag_msg.aprilTags)

            for tag in tags_cache:
                pts = np.array([
                    [tag.topLeft.x,     tag.topLeft.y],
                    [tag.topRight.x,    tag.topRight.y],
                    [tag.bottomRight.x, tag.bottomRight.y],
                    [tag.bottomLeft.x,  tag.bottomLeft.y],
                ], dtype=np.int32)
                cv2.polylines(rgb, [pts], True, (0, 255, 0), 2)
                cx = int(sum(p[0] for p in pts) / 4)
                cy = int(sum(p[1] for p in pts) / 4)
                cv2.putText(rgb, f"ID {tag.id}", (cx - 24, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Draw vertical centreline on RGB to show camera centre
            cv2.line(rgb, (RGB_W // 2, 0), (RGB_W // 2, RGB_H), (0, 180, 255), 1)

            # --- Labels ---
            cv2.putText(rgb,         "RGB  |  YOLO  |  AprilTags", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(depth_color, "Depth  (TURBO)",              (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # --- Composite side-by-side ---
            composite = np.hstack([rgb, depth_color])
            ok, buf = cv2.imencode(".jpg", composite,
                                   [cv2.IMWRITE_JPEG_QUALITY, 72])
            if ok:
                with _lock:
                    _latest_jpeg = buf.tobytes()


# ---------------------------------------------------------------------------
# HTTP handler — serves the HTML page and the MJPEG stream
# ---------------------------------------------------------------------------
class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silence per-request log noise

    def do_GET(self):
        if self.path == "/stream":
            self._serve_stream()
        else:
            self._serve_page()

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    jpeg = _latest_jpeg
                if jpeg:
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                time.sleep(0.033)   # ~30 fps cap
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _serve_page(self):
        html = f"""\
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <title>SipServe Camera</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      background: #0e0e0e; color: #e8e8e8;
      font-family: system-ui, sans-serif;
      display: flex; flex-direction: column;
      align-items: center; justify-content: center;
      min-height: 100vh; gap: 14px; padding: 20px;
    }}
    h1  {{ font-size: 1.2rem; letter-spacing: .05em; color: #7ec8e3; }}
    img {{ max-width: 100%; border-radius: 6px;
           border: 1px solid #333; display: block; }}
    p   {{ font-size: .8rem; color: #666; }}
  </style>
</head>
<body>
  <h1>SipServe — OAK-D-Lite Live View</h1>
  <img src="/stream" alt="camera stream"/>
  <p>
    Left: RGB &nbsp;—&nbsp;
    <span style="color:#ffff00">■</span> dining table &nbsp;
    <span style="color:#ffa500">■</span> obstacle &nbsp;
    <span style="color:#00c8ff">■</span> other &nbsp;
    <span style="color:#00ff00">■</span> AprilTag
    &nbsp;|&nbsp;
    Right: Stereo depth (TURBO, blue=near, red=far)
  </p>
</body>
</html>
""".encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Building camera pipeline...")
    pipeline, queues = build_pipeline()

    cam_thread = threading.Thread(
        target=camera_loop, args=(pipeline, queues), daemon=True
    )
    cam_thread.start()

    server = HTTPServer(("0.0.0.0", PORT), StreamHandler)
    print(f"[STREAM]  Serving on http://sipserve-johann:{PORT}  — Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    print("[STREAM]  Stopped.")
