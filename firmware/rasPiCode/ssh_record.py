#!/usr/bin/env python3
"""
Sub-Terra Rover - record from an SSH session (no ESP32 needed)

  python3 ssh_record.py            # Enter starts, Enter again stops
  python3 ssh_record.py 300        # record exactly 300 s (5 min), then stop
  python3 ssh_record.py --size 1920x1080 60

Saves to the USB drive (/mnt/rover/recordings) if mounted, else ./recordings.
When it stops it prints the numbers for test 2:
  duration, file size, MB per minute, GB per hour, and the throttle flags.

Stop the rover service first if it is running (only one program can use the camera):
  sudo systemctl stop rover_pi
"""

import argparse
import os
import subprocess
import sys
import time

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput

LOCAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
USB_MOUNT = "/mnt/rover"
BITRATE = 5_000_000            # same as rover_pi.py


def rec_dir():
    """Return the recordings folder: USB drive if mounted, else ./recordings."""
    if os.path.ismount(USB_MOUNT):
        d = os.path.join(USB_MOUNT, "recordings")
    else:
        d = LOCAL_DIR
    os.makedirs(d, exist_ok=True)
    return d


def throttled():
    """Return the Pi's `vcgencmd get_throttled` flags (0x0 = power OK)."""
    try:
        return subprocess.run(["vcgencmd", "get_throttled"],
                              capture_output=True, text=True).stdout.strip()
    except FileNotFoundError:
        return "vcgencmd not found"


def main():
    """Record one clip (until Enter, or for a fixed time), then print duration,
    size, data rate, projected size for a 2-mile run and throttle flags.
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", nargs="?", type=float,
                    help="record this long, then stop (default: until Enter)")
    ap.add_argument("--size", default="1280x720", help="WIDTHxHEIGHT (default 1280x720)")
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.lower().split("x"))

    d = rec_dir()
    where = "USB drive" if d.startswith(USB_MOUNT) else "SD card (no USB drive mounted)"

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (w, h)}))

    if a.seconds is None:
        input(f"Saving to {d}  [{where}]\nPress Enter to START recording...")

    path = os.path.join(d, time.strftime("clip_%Y%m%d_%H%M%S.mp4"))
    cam.start_recording(H264Encoder(bitrate=BITRATE), FfmpegOutput(path))
    t0 = time.time()
    print(f"REC ON  {w}x{h}  -> {path}")

    try:
        if a.seconds is None:
            input("Recording... press Enter to STOP ")
        else:
            while time.time() - t0 < a.seconds:
                left = a.seconds - (time.time() - t0)
                print(f"\r  {time.time() - t0:5.0f} s recorded, {left:5.0f} s left  ", end="", flush=True)
                time.sleep(1)
            print()
    except KeyboardInterrupt:
        print("\nStopped with Ctrl+C")
    finally:
        cam.stop_recording()
        cam.close()
        os.sync()

    dur = time.time() - t0
    size_mb = os.path.getsize(path) / 1e6 if os.path.exists(path) else 0.0
    mb_min = size_mb / (dur / 60) if dur > 0 else 0.0

    print("\n=== Recording results ===")
    print(f"file      : {path}")
    print(f"duration  : {dur:.1f} s")
    print(f"size      : {size_mb:.1f} MB")
    print(f"rate      : {mb_min:.1f} MB/min  =  {mb_min * 60 / 1000:.2f} GB/hour")
    print(f"2-mile run: ~{mb_min * 60 * 1.8 / 1000:.1f} GB  (1.8 h at 0.5 m/s)")
    print(f"throttle  : {throttled()}   (0x0 = power OK)")


if __name__ == "__main__":
    main()
