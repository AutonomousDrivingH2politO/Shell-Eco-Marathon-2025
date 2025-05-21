#!/usr/bin/env python3
"""
record_dual.py  –  Record video simultaneously from the left and right cameras of a ZED 2i.

Usage examples
--------------
$ python record_dual.py                     # left_20250509_154200.mp4 & right_20250509_154200.mp4 at 15 fps
$ python record_dual.py -p garden            # garden_left_20250509_154200.mp4 & garden_right_20250509_154200.mp4
$ python record_dual.py -d 60               # record 60 s then stop
$ python record_dual.py -r HD720 -f 60      # 60 fps @ 1280×720
"""
from __future__ import annotations
import argparse
import sys
from datetime import datetime, timedelta
import cv2
import pyzed.sl as sl


def parse_args() -> argparse.Namespace:
    res_choices = {r.name: r for r in sl.RESOLUTION}
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "-p", "--prefix",
        type=str,
        default=None,
        help="base name prefix for output files (default: timestamp)"
    )
    parser.add_argument(
        "-r", "--resolution",
        choices=res_choices.keys(),
        default="HD720",
        help="ZED resolution (default: HD720)"
    )
    parser.add_argument(
        "-f", "--fps",
        type=int,
        default=15,
        help="capture FPS (default: 15)"
    )
    parser.add_argument(
        "-d", "--duration",
        type=int,
        default=None,
        help="max seconds to record; Ctrl-C always stops"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # --- Open ZED ------------------------------------------------------------
    init = sl.InitParameters(
        camera_resolution=sl.RESOLUTION[args.resolution],
        camera_fps=args.fps
    )
    zed = sl.Camera()
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        sys.exit("❌  Could not open ZED camera")

    cam_info = zed.get_camera_information()
    img_size = cam_info.camera_configuration.resolution
    width, height = img_size.width, img_size.height

    # --- Prepare output filenames -------------------------------------------
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = args.prefix or ts
    left_out = f"{base}_left_{ts}.mp4"
    right_out = f"{base}_right_{ts}.mp4"

    # --- OpenCV video writers -----------------------------------------------
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer_left = cv2.VideoWriter(left_out, fourcc, args.fps, (width, height))
    writer_right = cv2.VideoWriter(right_out, fourcc, args.fps, (width, height))

    if not writer_left.isOpened() or not writer_right.isOpened():
        zed.close()
        sys.exit("❌  Could not create VideoWriter for one or both streams")

    print(
        f"► Recording to:\n"
        f"   Left:  {left_out}\n"
        f"   Right: {right_out}\n"
        "   — Ctrl-C to stop"
    )

    end_time = (
        datetime.now() + timedelta(seconds=args.duration)
        if args.duration else None
    )
    runtime = sl.RuntimeParameters()
    left_mat = sl.Mat()
    right_mat = sl.Mat()

    try:
        while True:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left and right images
                zed.retrieve_image(left_mat, sl.VIEW.LEFT)
                zed.retrieve_image(right_mat, sl.VIEW.RIGHT)

                # Convert BGRA to BGR (drop alpha)
                frame_left = left_mat.get_data()[:, :, :3]
                frame_right = right_mat.get_data()[:, :, :3]

                # Write frames
                writer_left.write(frame_left)
                writer_right.write(frame_right)

            if end_time and datetime.now() >= end_time:
                print("⏱️  Duration reached, stopping.")
                break
    except KeyboardInterrupt:
        print("\n⏹️  Stopped by user.")

    # --- Cleanup -------------------------------------------------------------
    writer_left.release()
    writer_right.release()
    zed.close()
    print(f"✅ Saved:\n   {left_out}\n   {right_out}")


if __name__ == "__main__":
    main()
