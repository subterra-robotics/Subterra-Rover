#!/usr/bin/env python3
"""
Sub-Terra Rover - Pi camera test
Raspberry Pi 3B+, Camera Module 3 (IMX708) or OV5647, Raspberry Pi OS Bookworm.

Modes:
  python3 camera_test.py record [seconds]   record an MP4 clip into ./recordings
  python3 camera_test.py serve              host recordings, open http://<pi-ip>:8000 on your PC
  python3 camera_test.py live               live MJPEG stream, open http://<pi-ip>:8000 on your PC

One-time setup on the Pi:
  sudo apt update
  sudo apt install -y python3-picamera2 ffmpeg
  rpicam-hello --list-cameras        # confirm the camera is detected
"""

import io
import os
import socket
import sys
import time
import html
from http import server
from socketserver import ThreadingMixIn
from threading import Condition

PORT = 8000
REC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")

# 720p30 is comfortable for the 3B+ hardware H.264 encoder.
VIDEO_SIZE = (1280, 720)
VIDEO_BITRATE = 5_000_000
LIVE_SIZE = (640, 480)            # smaller for the live stream over Wi-Fi


def pi_ip():
    """Best-guess LAN IP so we can print a clickable address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))     # no packets are sent, just picks the route
        return s.getsockname()[0]
    except OSError:
        return "<pi-ip>"
    finally:
        s.close()


# --------------------------------------------------------------------------
# record
# --------------------------------------------------------------------------
def record(seconds):
    """Record a `seconds`-long 720p H.264 clip into REC_DIR as MP4 and print its size."""
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import FfmpegOutput

    os.makedirs(REC_DIR, exist_ok=True)
    path = os.path.join(REC_DIR, time.strftime("clip_%Y%m%d_%H%M%S.mp4"))

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": VIDEO_SIZE}))

    encoder = H264Encoder(bitrate=VIDEO_BITRATE)
    output = FfmpegOutput(path)        # ffmpeg wraps the H.264 stream into an MP4

    print(f"Recording {seconds} s -> {path}")
    cam.start_recording(encoder, output)
    try:
        time.sleep(seconds)
    except KeyboardInterrupt:
        print("Stopped early")
    finally:
        cam.stop_recording()
        cam.close()

    size_mb = os.path.getsize(path) / 1e6 if os.path.exists(path) else 0
    print(f"Saved {path} ({size_mb:.1f} MB)")
    print("Run 'python3 camera_test.py serve' to view it from your PC.")


# --------------------------------------------------------------------------
# serve recordings
# --------------------------------------------------------------------------
class RecordingsHandler(server.SimpleHTTPRequestHandler):
    """HTTP handler that lists every clip in REC_DIR on "/" with an inline
    player and download link, and serves the .mp4 files themselves.
    """
    def __init__(self, *args, **kwargs):
        """Serve files out of REC_DIR instead of the working directory."""
        super().__init__(*args, directory=REC_DIR, **kwargs)

    def do_GET(self):
        """Build the clip index page for "/", otherwise serve the requested file."""
        if self.path not in ("/", "/index.html"):
            return super().do_GET()       # serve the .mp4 files themselves

        clips = sorted((f for f in os.listdir(REC_DIR) if f.endswith(".mp4")),
                       reverse=True)
        items = "".join(
            f'<h3>{html.escape(c)}</h3>'
            f'<video src="{html.escape(c)}" controls width="720"></video>'
            f'<p><a href="{html.escape(c)}" download>Download</a></p>'
            for c in clips
        ) or "<p>No recordings yet. Run: python3 camera_test.py record</p>"

        page = (f"<!doctype html><html><head><meta charset='utf-8'>"
                f"<title>Rover recordings</title></head>"
                f"<body style='font-family:sans-serif;background:#111;color:#eee'>"
                f"<h1>Sub-Terra Rover recordings</h1>{items}</body></html>")
        data = page.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class ThreadedServer(ThreadingMixIn, server.HTTPServer):
    """HTTPServer that handles each request in its own daemon thread."""
    allow_reuse_address = True
    daemon_threads = True


def serve():
    """Serve REC_DIR over HTTP on PORT until Ctrl+C."""
    os.makedirs(REC_DIR, exist_ok=True)
    print(f"Serving recordings at http://{pi_ip()}:{PORT}  (Ctrl+C to stop)")
    with ThreadedServer(("", PORT), RecordingsHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


# --------------------------------------------------------------------------
# live MJPEG stream
# --------------------------------------------------------------------------
class StreamingOutput(io.BufferedIOBase):
    """Holds the latest JPEG frame; the HTTP handler waits on it."""
    def __init__(self):
        """Start with no frame and a condition for waiting readers."""
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        """Called by the MJPEG encoder: store the new frame and wake all readers."""
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


stream_output = StreamingOutput()

LIVE_PAGE = (b"<!doctype html><html><head><title>Rover live</title></head>"
             b"<body style='margin:0;background:#000;text-align:center'>"
             b"<img src='/stream.mjpg' style='max-width:100%;height:auto'>"
             b"</body></html>")


class LiveHandler(server.BaseHTTPRequestHandler):
    """HTTP handler for the live view: "/" is a page with one <img>,
    "/stream.mjpg" is a multipart MJPEG stream of the latest frames.
    """
    def do_GET(self):
        """Serve the live page or stream frames until the browser disconnects."""
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(LIVE_PAGE)))
            self.end_headers()
            self.wfile.write(LIVE_PAGE)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with stream_output.condition:
                        stream_output.condition.wait()
                        frame = stream_output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                       # browser tab closed
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        """Silence the default per-request logging."""
        pass                               # keep the terminal quiet


def live():
    """Start the camera with an MJPEG encoder and serve the live stream on PORT."""
    from picamera2 import Picamera2
    from picamera2.encoders import MJPEGEncoder
    from picamera2.outputs import FileOutput

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": LIVE_SIZE}))
    cam.start_recording(MJPEGEncoder(), FileOutput(stream_output))

    print(f"Live stream at http://{pi_ip()}:{PORT}  (Ctrl+C to stop)")
    try:
        with ThreadedServer(("", PORT), LiveHandler) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop_recording()
        cam.close()


# --------------------------------------------------------------------------
def main():
    """Dispatch to record / serve / live from the command line."""
    if len(sys.argv) < 2 or sys.argv[1] not in ("record", "serve", "live"):
        print(__doc__)
        sys.exit(1)

    mode = sys.argv[1]
    if mode == "record":
        seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 10
        record(seconds)
    elif mode == "serve":
        serve()
    else:
        live()


if __name__ == "__main__":
    main()
