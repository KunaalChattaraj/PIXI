#!/usr/bin/env python3
"""
Live channel monitor for the Quantron QGP-1800 gamepad — RC-transmitter
style readout. Shows the CURRENT value of every axis and button,
continuously refreshing in place in the terminal (like checking a
transmitter's channels before binding to a flight controller).

Unlike the event-driven test script (which only prints when something
changes), this reads current device state on a timer and redraws the
whole dashboard every tick — so you always see live values for every
channel at once, including ones you're not currently touching.

Install:
    pip3 install evdev

Usage:
    python3 joystick_live_dashboard.py

Ctrl+C to quit.
"""

import sys
import time
import shutil

try:
    import evdev
    from evdev import ecodes
except ImportError:
    print("evdev not installed. Run: pip3 install evdev")
    sys.exit(1)


DEVICE_NAME_HINT = "microntek"
AXIS_MIN, AXIS_MAX, AXIS_CENTER = 0, 255, 128
DEADZONE_RAW = 15
REFRESH_HZ = 20  # how often to redraw the dashboard

# Channel labels, RC-style (CH1..CH4 = sticks, CH5/CH6 = D-pad)
AXIS_CHANNELS = [
    (ecodes.ABS_X,  "CH1 (Left Stick X / Roll)"),
    (ecodes.ABS_Y,  "CH2 (Left Stick Y / Pitch)"),
    (ecodes.ABS_Z,  "CH3 (Right Stick X / Yaw)"),
    (ecodes.ABS_RZ, "CH4 (Right Stick Y / Throttle)"),
]

# Set True for any channel whose direction feels backwards on your hardware.
# Confirmed: throttle (ABS_RZ) reads reversed on the QGP-1800 -> inverted here.
AXIS_INVERT = {
    ecodes.ABS_X:  False,
    ecodes.ABS_Y:  False,
    ecodes.ABS_Z:  False,
    ecodes.ABS_RZ: True,   # throttle fix
}
HAT_CHANNELS = [
    (ecodes.ABS_HAT0X, "CH5 (D-pad X)"),
    (ecodes.ABS_HAT0Y, "CH6 (D-pad Y)"),
]

BUTTON_LABELS = {
    288: "TRIGGER", 289: "THUMB",  290: "THUMB2", 291: "TOP",
    292: "TOP2",    293: "PINKIE", 294: "BASE",   295: "BASE2",
    296: "BASE3",   297: "BASE4",  298: "BASE5",  299: "BASE6",
}


def normalize_stick(raw_value: int, axis_code: int = None) -> float:
    centered = raw_value - AXIS_CENTER
    if abs(centered) <= DEADZONE_RAW:
        return 0.0
    span = AXIS_MAX - AXIS_CENTER
    norm = max(-1.0, min(1.0, centered / span))
    if axis_code is not None and AXIS_INVERT.get(axis_code, False):
        norm = -norm
    return norm


def find_gamepad():
    for path in evdev.list_devices():
        d = evdev.InputDevice(path)
        if DEVICE_NAME_HINT in d.name.lower():
            return d
    print(f"No device matching '{DEVICE_NAME_HINT}' found. Devices seen:")
    for path in evdev.list_devices():
        d = evdev.InputDevice(path)
        print(f"  {d.path}: {d.name}")
    return None


def make_bar(value: float, width: int = 20) -> str:
    """Simple text bar for -1.0..+1.0, centered at middle."""
    mid = width // 2
    pos = int(mid + value * mid)
    pos = max(0, min(width - 1, pos))
    bar = ["-"] * width
    bar[mid] = "|"
    bar[pos] = "#"
    return "".join(bar)


def main():
    dev = find_gamepad()
    if dev is None:
        sys.exit(1)

    print(f"Watching: {dev.path} ({dev.name!r})")
    print("Move sticks / D-pad / press buttons. Ctrl+C to quit.\n")
    time.sleep(1)

    # Current state, updated non-blockingly, redrawn on a timer.
    axis_state = {code: AXIS_CENTER for code, _ in AXIS_CHANNELS}
    hat_state = {code: 0 for code, _ in HAT_CHANNELS}
    button_state = {code: 0 for code in BUTTON_LABELS}

    dev.grab = None  # not grabbing exclusively; leave device usable elsewhere too

    try:
        # Make reads non-blocking so we can redraw on our own timer
        import fcntl
        import os
        fl = fcntl.fcntl(dev.fd, fcntl.F_GETFL)
        fcntl.fcntl(dev.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        last_draw = 0
        while True:
            # Drain any pending events (non-blocking)
            try:
                for event in dev.read():
                    if event.type == ecodes.EV_ABS:
                        if event.code in axis_state:
                            axis_state[event.code] = event.value
                        elif event.code in hat_state:
                            hat_state[event.code] = event.value
                    elif event.type == ecodes.EV_KEY:
                        if event.code in button_state:
                            button_state[event.code] = event.value
            except BlockingIOError:
                pass  # no new events right now, that's fine

            now = time.time()
            if now - last_draw >= 1.0 / REFRESH_HZ:
                draw_dashboard(axis_state, hat_state, button_state)
                last_draw = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nExiting.")


def draw_dashboard(axis_state, hat_state, button_state):
    cols = shutil.get_terminal_size((80, 20)).columns
    lines = []
    lines.append("=" * min(cols, 60))
    lines.append("LIVE CHANNEL MONITOR — Quantron QGP-1800")
    lines.append("=" * min(cols, 60))

    for code, label in AXIS_CHANNELS:
        raw = axis_state[code]
        norm = normalize_stick(raw, code)
        bar = make_bar(norm)
        lines.append(f"{label:32s} raw={raw:3d}  norm={norm:+.2f}  [{bar}]")

    for code, label in HAT_CHANNELS:
        val = hat_state[code]
        lines.append(f"{label:32s} value={val:+d}")

    lines.append("-" * min(cols, 60))
    btn_line = ""
    for code, label in BUTTON_LABELS.items():
        state = "ON " if button_state[code] else "off"
        btn_line += f"{label}:{state}  "
    lines.append(btn_line)
    lines.append("-" * min(cols, 60))
    lines.append("Ctrl+C to quit")

    # Move cursor to top-left and redraw, avoids scrolling spam
    print("\033[H\033[J", end="")  # clear screen
    print("\n".join(lines))


if __name__ == "__main__":
    main()