"""
Level 3 — Keyboard Control
==========================
Fly the drone with WASD keyboard controls.

What you'll learn:
    - Starting manual (keyboard) control
    - Hold modes: "current" vs "origin"
    - Sensitivity and momentum settings
    - Stopping manual control safely

Controls:
    W = Forward        S = Backward
    A = Left           D = Right
    Q = Rotate Left    E = Rotate Right
    R = Up             F = Down
    SPACE = Land and stop
    Ctrl + C = Emergency stop

Hold modes:
    "current" — release keys → hold at current position (free flight)
    "origin"  — release keys → snap back to launch point (spring mode)
"""

from litewing import LiteWing
import time

with LiteWing("192.168.43.42") as drone:
    drone.connect()
    drone.target_height = 0.3

    # ── Manual control settings ──────────────────────────

    # How fast the drone responds to key input
    drone.sensitivity = 0.4  # Default: 0.2 (m/s per key)

    # What happens when you release all keys?
    drone.hold_mode = "current"  # "current" = stay here, "origin" = go back

    print("Manual Control Settings:")
    print(f"  Sensitivity: {drone.sensitivity}")
    print(f"  Hold mode:   {drone.hold_mode}")
    print()
    print("Controls: WASD to move, SPACE or Q to land")
    print()

    # ── Start manual control ─────────────────────────────
    # This starts a background thread that handles:
    #   arm → takeoff → WASD loop → land
    # Note: connect/disconnect are now handled on outside.
    drone.start_manual_control()

    # Wait for manual control to finish (user presses SPACE/Q to land)
    # We must wait for _flight_active (not just _manual_active) because
    # the landing sequence runs AFTER _manual_active goes False.
    try:
        while drone._flight_active or drone._manual_active:
            time.sleep(0.1)
    except KeyboardInterrupt:
        drone.stop_manual_control()

    print("Manual control ended.")
    print("Done!")

# ── For GUI integration ──────────────────────────────
# If building a GUI (tkinter, PyQt, etc.), you can send key events
# programmatically instead of using the terminal:
#
#   drone.set_key("w", True)    # simulate W pressed
#   drone.set_key("w", False)   # simulate W released
#
#   drone.on_key_press(my_callback)    # register press handler
#   drone.on_key_release(my_callback)  # register release handler
#
# This lets you build custom controllers (buttons, joysticks, etc.)
# that drive the same manual control loop.