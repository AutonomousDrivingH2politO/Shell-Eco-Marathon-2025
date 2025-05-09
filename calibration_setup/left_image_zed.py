#!/usr/bin/env python3
"""
Capture a single image from the *left* sensor of a ZED 2i and save it.

Requires:
  • ZED SDK ≥ 4.1 with its Python bindings installed
  • OpenCV-Python (pip install opencv-python)

Typical run:
    $ python capture_left.py  # -> left_20250509_153722.png
"""
from datetime import datetime
import os
import cv2
import pyzed.sl as sl

def capture_left(output_dir: str = ".",  # where to save
                 resolution: sl.RESOLUTION = sl.RESOLUTION.HD720,
                 fps: int = 30) -> str | None:
    """Grab one frame from the left camera and save it.  
    Returns the path of the saved file or None on failure."""
    zed = sl.Camera()
    init = sl.InitParameters(camera_resolution=resolution,
                             camera_fps=fps)
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("⚠️  Could not open ZED camera")
        return None

    runtime = sl.RuntimeParameters()
    left_mat = sl.Mat()

    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:       # new frame available
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)       # left image only  :contentReference[oaicite:0]{index=0}
        img = left_mat.get_data()[:, :, :3]              # BGRA → BGR (drop alpha)  :contentReference[oaicite:1]{index=1}

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(output_dir, f"left_{ts}.png")
        if cv2.imwrite(filename, img):
            print(f"✅ Image saved → {filename}")
            zed.close()
            return filename
        else:
            print("⚠️  Failed to write image")
    else:
        print("⚠️  No frame grabbed")

    zed.close()
    return None


if __name__ == "__main__":
    capture_left()
