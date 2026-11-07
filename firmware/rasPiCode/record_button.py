#!/usr/bin/env python3
"""
Sub-Terra Rover - start/stop recording from the ESP32 button.

The ESP32 sends "BTN" over USB on each button press. This script toggles
recording and replies "REC_ON" / "REC_OFF".

Clips are saved to a USB flash drive labelled ROVER if one is mounted
(/mnt/rover or /media/<user>/ROVER), so you can unplug it and open the
videos on any laptop. Without the drive they go to ./recordings instead.

USB drive setup (once):
  1. Format the drive as exFAT (or FAT32) on your laptop, volume label ROVER
  2. On the Pi:
       sudo mkdir -p /mnt/rover
       echo 'LABEL=ROVER /mnt/rover exfat defaults,nofail,uid=1000,gid=1000 0 0' | sudo tee -a /etc/fstab
       sudo mount -a
     (use vfat instead of exfat for a FAT32 drive)
  3. Before unplugging: stop recording, then `sudo umount /mnt/rover`
     or shut the Pi down.

Setup on the Pi (once):
  sudo apt install -y python3-picamera2 python3-serial ffmpeg
  sudo usermod -aG dialout $USER     # then log out and back in

Run:
  python3 record_button.py
"""

import os
import time

import serial
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput

PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
BAUD = 115200

LOCAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
USB_MOUNTS = ["/mnt/rover", os.path.join("/media", os.environ.get("USER", "pi"), "ROVER")]
VIDEO_SIZE = (1280, 720)
VIDEO_BITRATE = 5_000_000


def pick_rec_dir():
    """USB drive if mounted, otherwise the local folder."""
    for m in USB_MOUNTS:
        if os.path.ismount(m):
            d = os.path.join(m, "recordings")
            try:
                os.makedirs(d, exist_ok=True)
                return d
            except OSError as e:
                print(f"Can't write to {m}: {e}")
    os.makedirs(LOCAL_DIR, exist_ok=True)
    return LOCAL_DIR


def log_block(rec_dir, line):
    """Append a BLOCK line from the ESP32 to blockages.csv next to the clips.
    ESP32 format: BLOCK,esp_ms,distance_m,height_m,width_m,offset_m,height_capped"""
    path = os.path.join(rec_dir, "blockages.csv")
    new = not os.path.exists(path)
    with open(path, "a") as f:
        if new:
            f.write("pi_time,esp_ms,distance_m,height_m,width_m,offset_m,height_capped\n")
        f.write(time.strftime("%Y-%m-%d %H:%M:%S,") + line[len("BLOCK,"):] + "\n")
    os.sync()
    print(f"Blockage logged: {line}")


def open_serial():
    """Open the ESP32 USB serial port without toggling DTR/RTS (which would
    reset the ESP32).
    """
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = 0.1
    ser.dtr = False              # don't reset the ESP32 when opening
    ser.rts = False
    ser.open()
    return ser


def main():
    """Toggle recording on each "BTN" line from the ESP32 and reply REC_ON /
    REC_OFF; log BLOCK lines. Saves any open clip on exit.
    """
    rec_dir = pick_rec_dir()
    where = "USB drive" if rec_dir != LOCAL_DIR else "Pi SD card (no ROVER drive found)"
    print(f"Saving clips to {rec_dir}  [{where}]")

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": VIDEO_SIZE}))

    ser = open_serial()
    recording = False
    path = None
    started = 0.0

    print("Waiting for button presses (Ctrl+C to quit)...")
    try:
        while True:
            line = ser.readline().decode(errors="replace").strip()

            if line.startswith("BLOCK,"):            # blockage dimensions from the tower
                log_block(rec_dir, line)
                continue
            if line != "BTN":
                continue

            if not recording:
                path = os.path.join(rec_dir, time.strftime("clip_%Y%m%d_%H%M%S.mp4"))
                cam.start_recording(H264Encoder(bitrate=VIDEO_BITRATE), FfmpegOutput(path))
                recording = True
                started = time.time()
                ser.write(b"REC_ON\n")
                print(f"REC ON  -> {path}")
            else:
                cam.stop_recording()
                os.sync()                    # flush to the drive so it's safe to unplug
                recording = False
                ser.write(b"REC_OFF\n")
                print(f"REC OFF ({time.time() - started:.1f} s) saved {path}")

    except KeyboardInterrupt:
        print("\nQuitting")
    finally:
        if recording:
            cam.stop_recording()
            os.sync()
            print(f"Saved {path}")
        cam.close()
        ser.close()


if __name__ == "__main__":
    main()
