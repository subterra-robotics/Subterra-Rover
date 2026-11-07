#!/usr/bin/env python3
"""
Sub-Terra Rover - Wi-Fi range test (decision data for the presentation)

The Pi makes its own hotspot. You walk away with a laptop/phone connected to
it; the Pi logs, once a second, the laptop's signal strength as the Pi hears
it, ping time and packet loss. The link is symmetric, so it doesn't matter
which end moves - leave the Pi at the "pipe entrance" and walk the laptop.

Usage (on the Pi, over Ethernet SSH or a keyboard):
  sudo ROVER_WIFI_PASS=... python3 wifi_test.py hotspot   # start the hotspot
  sudo python3 wifi_test.py hotspot --txpower 5   # weaker signal (dBm) so it drops within a hallway
  python3 wifi_test.py log                     # log until Ctrl+C
  sudo python3 wifi_test.py stop               # hotspot off, back to normal Wi-Fi

While logging, type a note + Enter to mark a spot, e.g.
  10m          25m corner          stairwell door
Easiest: SSH into the Pi FROM the walking laptop over the hotspot
(ssh yunus@10.42.0.1) and type marks as you go. When the link drops the SSH
session freezes, but logging keeps running - reconnect when you walk back.

Output: rover USB drive (/mnt/rover/recordings) if mounted, else ./recordings
        wifi_test_YYYYmmdd_HHMMSS.csv
"""

import argparse
import csv
import os
import re
import select
import signal
import subprocess
import sys
import time

SSID = "RoverLink"
PASSWORD = os.environ.get("ROVER_WIFI_PASS", "")   # set before running hotspot, 8+ characters
CON_NAME = "rover-hotspot"
IFACE = "wlan0"
PI_IP = "10.42.0.1"              # NetworkManager's hotspot address

LOCAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
USB_DIR = "/mnt/rover/recordings"


def run(cmd, check=False):
    """Run a command and capture its output. With check=True, exit on failure."""
    r = subprocess.run(cmd, capture_output=True, text=True)
    if check and r.returncode != 0:
        print(f"FAILED: {' '.join(cmd)}\n{r.stderr.strip()}")
        sys.exit(1)
    return r


def say(msg):
    """print that never kills the script if the SSH terminal went away"""
    try:
        print(msg, flush=True)
    except (BrokenPipeError, OSError):
        pass


# ---------------------------------------------------------------------------
# hotspot / stop
# ---------------------------------------------------------------------------
def hotspot(txpower):
    """Start a 2.4 GHz NetworkManager hotspot on wlan0, optionally at a fixed
    TX power (dBm) to shorten the range for indoor testing. Needs sudo.
    """
    if os.geteuid() != 0:
        sys.exit("Run with sudo:  sudo python3 wifi_test.py hotspot")
    if len(PASSWORD) < 8:
        sys.exit("Set the hotspot password first (8+ characters):\n"
                 "  sudo ROVER_WIFI_PASS='yourpassword' python3 wifi_test.py hotspot")

    if run(["iw", "dev", IFACE, "info"]).returncode != 0:
        sys.exit(f"No {IFACE}. Wi-Fi may be disabled: remove 'dtoverlay=disable-wifi' "
                 "from /boot/firmware/config.txt and reboot.")

    run(["rfkill", "unblock", "wifi"])
    run(["nmcli", "radio", "wifi", "on"])
    run(["nmcli", "connection", "delete", CON_NAME])           # clean start, ignore errors

    run(["nmcli", "device", "wifi", "hotspot", "ifname", IFACE, "con-name", CON_NAME,
         "ssid", SSID, "password", PASSWORD, "band", "bg", "channel", "6"], check=True)
    run(["nmcli", "connection", "modify", CON_NAME, "802-11-wireless.powersave", "2"])

    if txpower is not None:
        r = run(["iw", "dev", IFACE, "set", "txpower", "fixed", str(int(txpower * 100))])
        if r.returncode != 0:
            print(f"Note: this Wi-Fi chip refused a fixed TX power ({r.stderr.strip()}).")
            print("      Test at full power and use corners/walls to make the signal drop.")
        else:
            print(f"TX power set to {txpower} dBm")

    print(f"\nHotspot ON   SSID: {SSID}   password: (ROVER_WIFI_PASS)   2.4 GHz ch 6")
    print(f"Pi address: {PI_IP}   (ssh yunus@{PI_IP} from the laptop)")
    print("Next: connect the laptop to the hotspot, then  python3 wifi_test.py log")


def stop():
    """Take the hotspot down and delete its connection. Needs sudo."""
    if os.geteuid() != 0:
        sys.exit("Run with sudo:  sudo python3 wifi_test.py stop")
    run(["nmcli", "connection", "down", CON_NAME])
    run(["nmcli", "connection", "delete", CON_NAME])
    print("Hotspot off. NetworkManager will reconnect to your normal Wi-Fi if it knows one.")


# ---------------------------------------------------------------------------
# log
# ---------------------------------------------------------------------------
def station():
    """(mac, signal_dbm, tx_bitrate_mbps) of the first connected client, or None"""
    out = run(["iw", "dev", IFACE, "station", "dump"]).stdout
    m_mac = re.search(r"Station ([0-9a-f:]{17})", out)
    if not m_mac:
        return None
    sig = re.search(r"signal:\s*(-?\d+)", out)
    rate = re.search(r"tx bitrate:\s*([\d.]+)", out)
    return (m_mac.group(1),
            int(sig.group(1)) if sig else None,
            float(rate.group(1)) if rate else None)


def client_ip(mac):
    """Look up a connected client's IP from the neighbour table by MAC."""
    out = run(["ip", "neigh", "show", "dev", IFACE]).stdout
    for line in out.splitlines():
        if mac.lower() in line.lower():
            return line.split()[0]
    return None


def ping(ip):
    """round-trip ms, or None if lost"""
    r = run(["ping", "-c", "1", "-W", "1", ip])
    m = re.search(r"time=([\d.]+)", r.stdout)
    return float(m.group(1)) if m else None


def log():
    """Once a second, log the client's signal, bitrate, ping and rolling packet
    loss to a CSV (USB drive if mounted). Lines typed into the terminal are
    saved as location marks. Keeps running if the SSH session drops.
    """
    signal.signal(signal.SIGHUP, signal.SIG_IGN)      # keep running if the SSH session dies

    out_dir = USB_DIR if os.path.ismount("/mnt/rover") else LOCAL_DIR
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, time.strftime("wifi_test_%Y%m%d_%H%M%S.csv"))

    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["time", "elapsed_s", "connected", "signal_dbm", "tx_bitrate_mbps",
                "ping_ms", "loss_pct_last10", "mark"])

    say(f"Logging to {path}")
    say("Type a mark + Enter (e.g. 10m, corner). Ctrl+C to stop.\n")

    t0 = time.time()
    recent = []                 # last 10 ping results (True = reply)
    stdin_ok = sys.stdin.isatty()
    ip = None

    try:
        while True:
            loop_start = time.time()

            mark = ""
            if stdin_ok:
                try:
                    if select.select([sys.stdin], [], [], 0)[0]:
                        line = sys.stdin.readline()
                        if line == "":
                            stdin_ok = False      # terminal gone
                        else:
                            mark = line.strip()
                except (OSError, ValueError):
                    stdin_ok = False

            st = station()
            if st:
                mac, sig, rate = st
                ip = client_ip(mac) or ip
                rtt = ping(ip) if ip else None
            else:
                sig = rate = rtt = None

            recent.append(rtt is not None)
            recent = recent[-10:]
            loss = 100.0 * recent.count(False) / len(recent)

            w.writerow([time.strftime("%H:%M:%S"), f"{time.time() - t0:.0f}",
                        int(st is not None),
                        "" if sig is None else sig,
                        "" if rate is None else rate,
                        "" if rtt is None else f"{rtt:.1f}",
                        f"{loss:.0f}", mark])
            f.flush()
            os.fsync(f.fileno())

            say(f"{time.strftime('%H:%M:%S')}  "
                f"{'CONNECTED ' if st else 'NO CLIENT '}"
                f"signal={'--' if sig is None else f'{sig} dBm':>8}  "
                f"ping={'lost' if rtt is None else f'{rtt:.0f} ms':>7}  "
                f"loss={loss:3.0f}%"
                + (f"   << {mark}" if mark else ""))

            time.sleep(max(0.0, 1.0 - (time.time() - loop_start)))
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        say(f"\nSaved {path}")


# ---------------------------------------------------------------------------
def main():
    """Parse the hotspot / log / stop subcommand and run it."""
    ap = argparse.ArgumentParser(description="Rover Wi-Fi range test")
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("hotspot", help="start the Pi hotspot (sudo)")
    h.add_argument("--txpower", type=float, help="fixed TX power in dBm, e.g. 5")
    sub.add_parser("log", help="log signal / ping / loss once a second")
    sub.add_parser("stop", help="turn the hotspot off (sudo)")
    a = ap.parse_args()

    if a.cmd == "hotspot":
        hotspot(a.txpower)
    elif a.cmd == "log":
        log()
    else:
        stop()


if __name__ == "__main__":
    main()
