#!/usr/bin/env python3
"""
Isolated joystick test using evdev — tuned specifically for the
Quantron QGP-1800 (DragonRise/Microntek chipset), based on values
confirmed via `evtest` on the Jetson:

    lsusb:  ID 0079:0006 DragonRise Inc. PC TWIN SHOCK Gamepad
    device: /dev/input/by-id/usb-Microntek_USB_Joystick-event-joystick

Confirmed axis map (raw evdev, 0-255, center=128, flat/deadzone=15):
    ABS_X  (code 0)  -> Left stick, horizontal
    ABS_Y  (code 1)  -> Left stick, vertical
    ABS_Z  (code 2)  -> Right stick, horizontal
    ABS_RZ (code 5)  -> Right stick, vertical
    ABS_HAT0X (code 16) -> D-pad horizontal, range -1/0/1
    ABS_HAT0Y (code 17) -> D-pad vertical,   range -1/0/1

Confirmed buttons (12 total, EV_KEY):
    288 BTN_TRIGGER   289 BTN_THUMB    290 BTN_THUMB2   291 BTN_TOP
    292 BTN_TOP2      293 BTN_PINKIE   294 BTN_BASE     295 BTN_BASE2
    296 BTN_BASE3     297 BTN_BASE4    298 BTN_BASE5    299 BTN_BASE6

This script auto-locates the device by name ("Microntek") so you don't
have to pick an index every run, normalizes stick axes to -1.0..+1.0
(useful for feeding straight into a setpoint), and applies the
hardware's own deadzone (15/255 ~= 0.06) so idle sticks read as a
clean 0.0 instead of jittering.

Install (on Jetson):
    pip3 install evdev

Permissions (only needed once):
    sudo usermod -aG input $USER      # then log out/in
    (or just run with sudo for now)

Usage:
    python3 test_joystick_evdev.py
"""

import sys

try:
    import evdev
    from evdev import ecodes
except ImportError:
    print("evdev not installed. Run: pip3 install evdev")
    sys.exit(1)


DEVICE_NAME_HINT = "microntek"  # matches the QGP-1800's reported name

# Raw hardware range, confirmed via evtest.
AXIS_MIN, AXIS_MAX, AXIS_CENTER = 0, 255, 128
DEADZONE_RAW = 15  # matches the "Flat" value evtest reported

AXIS_LABELS = {
    ecodes.ABS_X:  "LEFT_STICK_X",
    ecodes.ABS_Y:  "LEFT_STICK_Y",
    ecodes.ABS_Z:  "RIGHT_STICK_X",
    ecodes.ABS_RZ: "RIGHT_STICK_Y",
    ecodes.ABS_HAT0X: "DPAD_X",
    ecodes.ABS_HAT0Y: "DPAD_Y",
}

BUTTON_LABELS = {
    288: "TRIGGER",
    289: "THUMB",
    290: "THUMB2",
    291: "TOP",
    292: "TOP2",
    293: "PINKIE",
    294: "BASE",
    295: "BASE2",
    296: "BASE3",
    297: "BASE4",
    298: "BASE5",
    299: "BASE6",
}


def normalize_stick(raw_value: int) -> float:
    """Map raw 0-255 (center 128) to -1.0..+1.0, with deadzone clamped to 0."""
    centered = raw_value - AXIS_CENTER
    if abs(centered) <= DEADZONE_RAW:
        return 0.0
    span = AXIS_MAX - AXIS_CENTER  # 127
    return max(-1.0, min(1.0, centered / span))


def find_gamepad():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    if not devices:
        print("No input devices found. Check `ls /dev/input/event*` and `lsusb`.")
        return None

    for d in devices:
        if DEVICE_NAME_HINT in d.name.lower():
            print(f"Found QGP-1800 at {d.path} ({d.name!r})")
            return d

    print(f"Couldn't find a device matching '{DEVICE_NAME_HINT}'. Devices seen:")
    for d in devices:
        print(f"  {d.path}: {d.name}")
    return None


def main():
    dev = find_gamepad()
    if dev is None:
        sys.exit(1)

    print("\nMove the sticks / D-pad / press buttons. Ctrl+C to quit.\n")

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_ABS:
                label = AXIS_LABELS.get(event.code, f"ABS_{event.code}")
                if event.code in (ecodes.ABS_X, ecodes.ABS_Y,
                                   ecodes.ABS_Z, ecodes.ABS_RZ):
                    norm = normalize_stick(event.value)
                    print(f"{label:15s} raw={event.value:3d}  norm={norm:+.3f}")
                else:  # D-pad hats: already -1/0/1
                    print(f"{label:15s} value={event.value:+d}")
            elif event.type == ecodes.EV_KEY:
                label = BUTTON_LABELS.get(event.code, f"KEY_{event.code}")
                state = "DOWN" if event.value == 1 else ("UP" if event.value == 0 else "HOLD")
                print(f"BUTTON {label:10s} {state}")
    except KeyboardInterrupt:
        print("\nExiting.")
    except PermissionError:
        print("\nPermission denied reading the device.")
        print("Fix: sudo usermod -aG input $USER   (then log out/in)")
        print("Or run this script with sudo for now.")


if __name__ == "__main__":
    main()
