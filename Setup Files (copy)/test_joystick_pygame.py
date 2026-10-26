#!/usr/bin/env python3
"""
Isolated joystick test using pygame — tuned for the Quantron QGP-1800
(DragonRise/Microntek chipset), confirmed on the Jetson via evtest/jstest:

    6 axes:   X, Y, Z, RZ, Hat0X, Hat0Y   (jstest driver auto-normalizes
              raw 0-255 to the -32767..+32767 range; pygame further
              normalizes that to -1.0..+1.0)
    12 buttons: Trigger, ThumbBtn, ThumbBtn2, TopBtn, TopBtn2, PinkieBtn,
              BaseBtn, BaseBtn2, BaseBtn3, BaseBtn4, BaseBtn5, BaseBtn6

NOTE: pygame's axis *order* is assigned by the OS joystick driver, not
guaranteed to exactly match evdev's ABS_X/Y/Z/RZ order. Run this once
and confirm which axis index moves for "left stick" vs "right stick" —
print it out here, then cross-check against the evdev script's labels
before wiring up final controls. If they match your expectation from
evtest, great; if not, just relabel here rather than assuming.

Install (on Jetson):
    pip3 install pygame

Usage:
    python3 test_joystick_pygame.py
"""

import pygame
import sys

# Fill these in once you've confirmed the mapping against evtest output.
# Defaults below are a reasonable starting guess based on evdev order
# (ABS_X=0, ABS_Y=1, ABS_Z=2, ABS_RZ=3 in pygame's own axis indexing —
# pygame typically drops the hat axes from get_numaxes() and reports
# D-pad separately via get_hat()).
AXIS_LABELS = {
    0: "LEFT_STICK_X",
    1: "LEFT_STICK_Y",
    2: "RIGHT_STICK_X",
    3: "RIGHT_STICK_Y",
}

BUTTON_LABELS = {
    0: "TRIGGER",
    1: "THUMB",
    2: "THUMB2",
    3: "TOP",
    4: "TOP2",
    5: "PINKIE",
    6: "BASE",
    7: "BASE2",
    8: "BASE3",
    9: "BASE4",
    10: "BASE5",
    11: "BASE6",
}

DEADZONE = 0.08  # roughly matches the hardware's 15/255 flat zone


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Checks to try:")
        print("  - lsusb   (is the QGP-1800 / DragonRise device listed?)")
        print("  - ls /dev/input/js*")
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using: {js.get_name()}  "
          f"axes={js.get_numaxes()} buttons={js.get_numbuttons()} hats={js.get_numhats()}")
    print("\nMove sticks / D-pad / press buttons. Values print only on change. Ctrl+C to quit.\n")

    last_axes = [0.0] * js.get_numaxes()
    last_buttons = [0] * js.get_numbuttons()
    last_hats = [(0, 0)] * js.get_numhats()

    clock = pygame.time.Clock()
    try:
        while True:
            pygame.event.pump()

            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                if abs(val) < DEADZONE:
                    val = 0.0
                if abs(val - last_axes[i]) > 0.03:
                    label = AXIS_LABELS.get(i, f"AXIS_{i}")
                    print(f"{label:15s} (index {i}) = {val:+.3f}")
                    last_axes[i] = val

            for i in range(js.get_numbuttons()):
                val = js.get_button(i)
                if val != last_buttons[i]:
                    label = BUTTON_LABELS.get(i, f"BUTTON_{i}")
                    print(f"{label:10s} (index {i}) = {'DOWN' if val else 'UP'}")
                    last_buttons[i] = val

            for i in range(js.get_numhats()):
                val = js.get_hat(i)
                if val != last_hats[i]:
                    print(f"DPAD (hat {i}) = {val}")
                    last_hats[i] = val

            clock.tick(60)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        pygame.joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()
