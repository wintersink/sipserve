"""
capture_dataset.py — Capture training images from the OAK-D-Lite.

Saves RGB frames to a dataset folder.  Use this to build a labelling
dataset for Roboflow or any other annotation tool.

Controls (press in terminal, then Enter):
    s  — save current frame
    a  — auto-capture mode: save one frame every AUTO_INTERVAL seconds
    q  — quit

Images are saved to:  dataset/images/YYYY-MM-DD_HH-MM-SS_NNN.jpg

Usage:
    source venv4sipserve/bin/activate
    python smoketests/capture_dataset.py
"""

import sys
import os
import time
import threading
import select

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SAVE_DIR       = os.path.join(os.path.dirname(__file__), "..", "dataset", "images")
RGB_W, RGB_H   = 640, 480
AUTO_INTERVAL  = 2.0   # seconds between auto-captures

os.makedirs(SAVE_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Pipeline — RGB only, no YOLO (faster startup, no resource conflicts)
# ---------------------------------------------------------------------------
def build_pipeline():
    pipeline = dai.Pipeline()
    camRgb   = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb_out  = camRgb.requestOutput((RGB_W, RGB_H))
    queues   = {"rgb": rgb_out.createOutputQueue()}
    return pipeline, queues


# ---------------------------------------------------------------------------
# Non-blocking stdin reader (runs in its own thread)
# ---------------------------------------------------------------------------
_cmd_lock  = threading.Lock()
_pending   = []

def _input_thread():
    while True:
        try:
            line = input()
        except EOFError:
            break
        with _cmd_lock:
            _pending.append(line.strip().lower())

# ---------------------------------------------------------------------------
# Save helper
# ---------------------------------------------------------------------------
_frame_count = 0

def save_frame(frame):
    global _frame_count
    _frame_count += 1
    ts   = time.strftime("%Y-%m-%d_%H-%M-%S")
    name = f"{ts}_{_frame_count:04d}.jpg"
    path = os.path.join(SAVE_DIR, name)
    cv2.imwrite(path, frame)
    print(f"[SAVED]  {path}")
    return path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Building pipeline...")
    pipeline, queues = build_pipeline()

    inp_thread = threading.Thread(target=_input_thread, daemon=True)
    inp_thread.start()

    auto_mode    = False
    last_auto_t  = 0.0

    print(f"\nDataset folder: {os.path.abspath(SAVE_DIR)}")
    print("─" * 50)
    print("  s + Enter  →  save one frame")
    print("  a + Enter  →  toggle auto-capture every "
          f"{AUTO_INTERVAL:.0f}s")
    print("  q + Enter  →  quit")
    print("─" * 50)

    with pipeline:
        pipeline.start()
        print("Pipeline running — point camera at your subject.\n")

        while pipeline.isRunning():
            # Grab latest frame
            rgb_msg = queues["rgb"].get()
            frame   = rgb_msg.getCvFrame()
            if frame is None:
                continue
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            frame = cv2.resize(frame, (RGB_W, RGB_H))

            # Process keyboard commands
            with _cmd_lock:
                cmds = list(_pending)
                _pending.clear()

            for cmd in cmds:
                if cmd == "s":
                    save_frame(frame)
                elif cmd == "a":
                    auto_mode = not auto_mode
                    print(f"[AUTO]  {'ON' if auto_mode else 'OFF'} "
                          f"(every {AUTO_INTERVAL:.0f}s)")
                elif cmd == "q":
                    print(f"\n[DONE]  {_frame_count} frames saved to "
                          f"{os.path.abspath(SAVE_DIR)}")
                    pipeline.stop()
                    sys.exit(0)

            # Auto-capture
            if auto_mode and (time.time() - last_auto_t) >= AUTO_INTERVAL:
                save_frame(frame)
                last_auto_t = time.time()

            time.sleep(0.03)

    print(f"\n[DONE]  {_frame_count} frames saved to {os.path.abspath(SAVE_DIR)}")
