#!/usr/bin/env python3
"""
Sub-Terra Rover - Pi side of the mission (ESP32 runs the mission, Pi records).

Talks to the ESP32 over USB, one text line per message:
  Pi  -> ESP32 : READY (heartbeat, once a second), REC_ON, REC_OFF
  ESP32 -> Pi  : START, STOP, SHUTDOWN, BLOCK,..., STATE,..., FAULT,..., LOG,...

  START    -> start recording on the USB drive, reply REC_ON
              (no drive mounted -> reply REC_FAIL,no_usb and don't record)
  STOP     -> save the clip, reply REC_OFF
  SHUTDOWN -> save the clip, shut the Pi down cleanly
  BLOCK,.. -> appended to blockages.csv
  STATE / FAULT / LOG -> printed and appended to rover_log.txt

Clips are recorded ONLY to the USB drive labelled ROVER (/mnt/rover).
The drive is checked at every START. Logs go to the drive too, or to
./recordings while no drive is mounted.

Keys typed in this terminal are forwarded to the ESP32 (simulated tilt):
  1 = tilt left 5 deg, 2 = tilt right 5 deg, 0 = level
(only when run by hand in a terminal / SSH, not as the boot service)

If the ESP32 is unplugged or resets, any recording is saved and the script
keeps retrying the port.

Setup (once):
  sudo apt install -y python3-picamera2 python3-serial ffmpeg
  sudo usermod -aG dialout $USER        # then log out and back in
  (USB drive: see record_button.py for the /etc/fstab line)

Run by hand:   python3 rover_pi.py
Run at boot:   see rover_pi.service
"""

import os
import select
import subprocess
import sys
import termios
import time
import tty

import serial
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput

PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
BAUD = 115200
HEARTBEAT_S = 1.0
RECONNECT_S = 2.0

LOCAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
USB_MOUNTS = ["/mnt/rover", os.path.join("/media", os.environ.get("USER", "pi"), "ROVER")]
VIDEO_SIZE = (1280, 720)
VIDEO_BITRATE = 5_000_000


# ---------------------------------------------------------------------------
# Storage
# ---------------------------------------------------------------------------
def usb_dir():
    """recordings folder on the USB drive, or None if no writable drive is mounted."""
    for m in USB_MOUNTS:
        if os.path.ismount(m):
            d = os.path.join(m, "recordings")
            try:
                os.makedirs(d, exist_ok=True)
                return d
            except OSError as e:
                print(f"Can't write to {m}: {e}")
    return None


def pick_rec_dir():
    """Where logs go: USB drive if mounted, otherwise the local folder."""
    d = usb_dir()
    if d:
        return d
    os.makedirs(LOCAL_DIR, exist_ok=True)
    return LOCAL_DIR


KEYS = "120"


def read_key():
    """One key from the terminal without waiting, or None."""
    if not sys.stdin.isatty():
        return None
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def append(path, text):
    """Append text to a file, creating it if needed."""
    with open(path, "a") as f:
        f.write(text)


def log(rec_dir, msg):
    """Print a timestamped message and append it to rover_log.txt in rec_dir."""
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
    print(line, flush=True)
    append(os.path.join(rec_dir, "rover_log.txt"), line + "\n")


def log_block(rec_dir, line):
    """BLOCK,esp_ms,distance_m,height_m,width_m,offset_m,height_capped"""
    path = os.path.join(rec_dir, "blockages.csv")
    if not os.path.exists(path):
        append(path, "pi_time,esp_ms,distance_m,height_m,width_m,offset_m,height_capped\n")
    append(path, time.strftime("%Y-%m-%d %H:%M:%S,") + line[len("BLOCK,"):] + "\n")
    os.sync()
    log(rec_dir, f"Blockage logged: {line}")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
class Recorder:
    """Wraps Picamera2 for start/stop MP4 recording of one clip at a time."""
    def __init__(self):
        """Configure the camera for VIDEO_SIZE video; nothing is recording yet."""
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_video_configuration(main={"size": VIDEO_SIZE}))
        self.recording = False
        self.path = None
        self.started = 0.0

    def start(self, rec_dir):
        """Start a new timestamped clip in rec_dir. No-op if already recording."""
        if self.recording:
            return
        self.path = os.path.join(rec_dir, time.strftime("clip_%Y%m%d_%H%M%S.mp4"))
        self.cam.start_recording(H264Encoder(bitrate=VIDEO_BITRATE), FfmpegOutput(self.path))
        self.recording = True
        self.started = time.time()

    def stop(self):
        """Returns (path, seconds) of the saved clip, or None."""
        if not self.recording:
            return None
        self.cam.stop_recording()
        os.sync()
        self.recording = False
        return self.path, time.time() - self.started

    def close(self):
        """Stop any recording and release the camera."""
        self.stop()
        self.cam.close()


# ---------------------------------------------------------------------------
# Serial
# ---------------------------------------------------------------------------
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


def send(ser, msg):
    """Send one newline-terminated text message to the ESP32."""
    ser.write((msg + "\n").encode())


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def session(ser, rec, rec_dir):
    """Handle one connection to the ESP32. Returns 'shutdown' or raises on disconnect."""
    last_beat = 0.0

    while True:
        now = time.time()
        if now - last_beat >= HEARTBEAT_S:
            send(ser, "READY")
            last_beat = now

        k = read_key()
        if k and k in KEYS:
            send(ser, f"KEY,{k}")

        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue

        if line == "START":
            d = usb_dir()
            if d is None:
                send(ser, "REC_FAIL,no_usb")
                log(pick_rec_dir(), "START refused: USB drive ROVER not mounted")
                continue
            rec_dir = d
            rec.start(rec_dir)
            send(ser, "REC_ON")
            log(rec_dir, f"REC ON  -> {rec.path}")

        elif line == "STOP":
            saved = rec.stop()
            send(ser, "REC_OFF")
            if saved:
                log(rec_dir, f"REC OFF ({saved[1]:.1f} s) saved {saved[0]}")

        elif line == "SHUTDOWN":
            saved = rec.stop()
            if saved:
                log(rec_dir, f"REC OFF ({saved[1]:.1f} s) saved {saved[0]}")
            log(rec_dir, "SHUTDOWN requested by the rover button")
            return "shutdown"

        elif line.startswith("BLOCK,"):
            log_block(rec_dir, line)

        elif line.startswith(("FAULT,", "LOG,")) or line.startswith("STATE,") and line.count(",") > 1:
            log(rec_dir, line)          # faults, logs and state changes with a reason

        elif line.startswith("STATE,"):
            pass                        # periodic status: ignore (too chatty to log)

        else:
            print(f"? {line}", flush=True)


def main():
    """Connect to the ESP32 (retrying forever), run sessions, save clips on
    disconnect, and shut the Pi down when the rover asks for it.
    """
    rec = Recorder()
    rec_dir = pick_rec_dir()
    log(rec_dir, f"rover_pi started, USB drive: {usb_dir() or 'NOT MOUNTED'}")

    term_saved = None
    if sys.stdin.isatty():                          # single-key input, Ctrl+C still works
        term_saved = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print("Keys: 1 = tilt left, 2 = tilt right, 0 = level", flush=True)

    try:
        while True:
            try:
                ser = open_serial()
            except (serial.SerialException, OSError):
                time.sleep(RECONNECT_S)             # ESP32 not plugged in yet
                continue

            log(rec_dir, "ESP32 connected")
            try:
                result = session(ser, rec, rec_dir)
            except (serial.SerialException, OSError) as e:
                saved = rec.stop()
                if saved:
                    log(rec_dir, f"ESP32 lost - clip saved {saved[0]}")
                log(rec_dir, f"ESP32 disconnected ({e}), retrying")
                result = None
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

            if result == "shutdown":
                rec.close()
                subprocess.run(["sudo", "-n", "shutdown", "-h", "now"])
                return
            time.sleep(RECONNECT_S)

    except KeyboardInterrupt:
        print("\nQuitting")
    finally:
        if term_saved:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term_saved)
        try:
            rec.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
