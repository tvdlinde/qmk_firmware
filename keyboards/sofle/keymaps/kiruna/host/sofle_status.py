#!/usr/bin/env python3
"""Feed the Sofle (kiruna keymap) OLED status screens over Raw HID.

Once a second, sends one 32-byte packet to the left half: time, date, volume
and battery for the left screen; load average, CPU, disk busy and memory for
the right screen (forwarded by the firmware over the split link). Packet layout
must match the ST_* enum in ../keymap.c.

While the screen is locked (swaylock running) nothing is sent, so the firmware
marks the data stale and blanks the OLEDs -- spares them from burn-in overnight.

    sofle_status.py           run the feeder (reconnects on unplug/replug)
    sofle_status.py --print   print the values instead of sending (no keyboard needed)
"""

import os
import re
import subprocess
import sys
import time

import psutil

VID, PID = 0xFC32, 0x0287          # keyboards/sofle/{info.json,rev1/keyboard.json}
USAGE_PAGE, USAGE = 0xFF60, 0x61   # QMK Raw HID interface
MAGIC = 0x5A
FLAG_MUTED, FLAG_CHARGING, FLAG_NO_BAT = 0x01, 0x02, 0x04


def root_disk():
    """Whole-disk name (e.g. 'nvme0n1') backing /, as psutil names it."""
    dev = os.stat("/").st_dev
    path = os.path.realpath(f"/sys/dev/block/{os.major(dev)}:{os.minor(dev)}")
    if os.path.exists(os.path.join(path, "partition")):
        path = os.path.dirname(path)
    return os.path.basename(path)


def volume():
    """(percent, muted) of the default PipeWire sink."""
    try:
        out = subprocess.run(["wpctl", "get-volume", "@DEFAULT_AUDIO_SINK@"],
                             capture_output=True, text=True, timeout=1).stdout
        m = re.search(r"Volume:\s*([\d.]+)", out)
        return (round(float(m.group(1)) * 100) if m else 0), "MUTED" in out
    except (OSError, subprocess.TimeoutExpired):
        return 0, False


def screen_locked():
    """Same test the sway config's swayidle uses: is a swaylock process running?"""
    return subprocess.run(["pgrep", "-x", "swaylock"], stdout=subprocess.DEVNULL).returncode == 0


class Sampler:
    def __init__(self):
        self.disk = root_disk()
        self.last_busy = self._busy_ms()
        self.last_t = time.monotonic()
        psutil.cpu_percent()  # prime: first call compares against boot

    def _busy_ms(self):
        c = psutil.disk_io_counters(perdisk=True).get(self.disk)
        return c.busy_time if c else 0

    def packet(self):
        now = time.localtime()
        vol, muted = volume()
        bat = psutil.sensors_battery()

        busy, t = self._busy_ms(), time.monotonic()
        disk = (busy - self.last_busy) / ((t - self.last_t) * 1000) * 100
        self.last_busy, self.last_t = busy, t

        flags = (FLAG_MUTED if muted else 0)
        if bat is None:
            flags |= FLAG_NO_BAT
        elif bat.power_plugged:
            flags |= FLAG_CHARGING
        load = min(round(os.getloadavg()[0] * 100), 0xFFFF)

        def pct(x):
            return max(0, min(255, round(x)))

        data = [
            MAGIC, now.tm_hour, now.tm_min, now.tm_mday, now.tm_mon,
            pct(vol), flags, pct(bat.percent) if bat else 0,
            load & 0xFF, load >> 8,
            pct(psutil.cpu_percent()), pct(min(disk, 100)),
            pct(psutil.virtual_memory().percent),
        ]
        return bytes(data + [0] * (32 - len(data)))


def describe(p):
    flags = p[6]
    return (f"{p[1]:02}:{p[2]:02} {p[3]:02}/{p[4]:02}  "
            f"vol {'mute' if flags & FLAG_MUTED else f'{p[5]}%'}  "
            f"bat {'AC' if flags & FLAG_NO_BAT else f'{p[7]}%'}{'+' if flags & FLAG_CHARGING else ''}  |  "
            f"load {(p[8] | p[9] << 8) / 100:.2f}  cpu {p[10]}%  disk {p[11]}%  mem {p[12]}%")


def open_keyboard():
    import hid
    for d in hid.enumerate(VID, PID):
        if d["usage_page"] == USAGE_PAGE and d["usage"] == USAGE:
            dev = hid.device()
            dev.open_path(d["path"])
            return dev
    return None


def sleep_to_next_second():
    time.sleep(1 - time.time() % 1 + 0.02)


def main():
    sampler = Sampler()
    if "--print" in sys.argv:
        while True:
            sleep_to_next_second()
            packet = sampler.packet()
            print("locked (paused)" if screen_locked() else describe(packet), flush=True)

    dev, warned = None, False
    while True:
        sleep_to_next_second()
        packet = sampler.packet()  # sample every second so CPU/disk deltas stay 1 s wide
        if screen_locked():
            continue
        if dev is None:
            try:
                dev = open_keyboard()
            except OSError as e:
                if not warned:
                    print(f"cannot open keyboard: {e} (udev rule installed?)", file=sys.stderr, flush=True)
                    warned = True
            if dev is None:
                continue
            print("connected", flush=True)
            warned = False
        try:
            dev.write(b"\x00" + packet)  # leading report ID 0
        except (OSError, ValueError):
            print("disconnected", flush=True)
            dev.close()
            dev = None


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
