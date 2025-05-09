#!/usr/bin/env python3
"""
record_left.py  –  Record video from the *left* camera of a ZED 2i.

Usage examples
--------------
$ python record_left.py                 # left_20250509_154200.mp4, default 15 fps
$ python record_left.py -o garden.mp4   # custom file name
$ python record_left.py -d 60           # record 60 s then stop
$ python record_left.py -r HD720 -f 60  # 60 fps @ 1280×720

Dependencies
------------
  • ZED SDK ≥ 4.1 with its Python API  (pip install pyzed)
  • OpenCV-Python                       (pip install opencv-python)
"""
from __future__ import annotations
import argparse
import sys
from datetime import datetime, timedelta
import cv2
import pyzed.sl as sl


def parse_args() -> argparse.Namespace:
    res_choices = {r.name: r for r in sl.RESOLUTION}
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="output .mp4/.avi file (default: left_<timestamp>.mp4)")
    parser.add_argument("-r", "--resolution", choices=res_choices.keys(),
                        default="HD720", help="ZED resolution (default: HD720)")
    parser.add_argument("-f", "--fps", type=int, default=15,
                        help="capture FPS (default: 15)")
    parser.add_argument("-d", "--duration", type=int, default=None,
                        help="max seconds to record; Ctrl-C always stops")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # --- Open ZED ------------------------------------------------------------
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION[args.resolution],
                             camera_fps=args.fps)
    zed = sl.Camera()
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        sys.exit("❌  Could not open ZED camera")
    cam_info = zed.get_camera_information()
    img_size = cam_info.camera_configuration.resolution
    width, height = img_size.width, img_size.height

    # --- OpenCV video writer -------------------------------------------------
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = args.output or f"left_{ts}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # or "XVID" for .avi
    writer = cv2.VideoWriter(outfile, fourcc, args.fps, (width, height))

    if not writer.isOpened():
        zed.close()
        sys.exit("❌  Could not create VideoWriter")

    print(f"► Recording to {outfile}  —  Ctrl-C to stop")
    end_time = datetime.now() + timedelta(seconds=args.duration) if args.duration else None
    runtime = sl.RuntimeParameters()
    left = sl.Mat()

    try:
        while True:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(left, sl.VIEW.LEFT)  # left image only
                frame = left.get_data()[:, :, :3]       # BGRA ➜ BGR (drop alpha)
                writer.write(frame)

            if end_time and datetime.now() >= end_time:
                print("⏱️  Duration reached, stopping.")
                break
    except KeyboardInterrupt:
        print("\n⏹️  Stopped by user.")

    # --- Cleanup -------------------------------------------------------------
    writer.release()
    zed.close()
    print(f"✅ Saved {outfile}")


if __name__ == "__main__":
    main()
