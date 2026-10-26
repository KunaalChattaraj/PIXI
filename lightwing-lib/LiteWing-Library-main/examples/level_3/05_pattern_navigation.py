"""
Level 3 — Pattern Navigation
=============================
Fly geometric shapes using the built-in shape methods.

What you'll learn:
    - drone.square()    — fly a square path
    - drone.triangle()  — fly an equilateral triangle
    - drone.circle()    — fly a smooth circle (auto-calculated waypoints)
    - drone.pentagon()  — fly a regular pentagon
    - face_direction    — whether the drone nose tracks the flight direction

All shapes are centered on the drone's current hover position and
the drone returns to its start point after each shape.

face_direction=True  → nose always points where you're going
face_direction=False → drone keeps its initial heading throughout

Coordinate system (from drone's starting perspective):
    +X = forward
    -X = backward
    +Y = left
    -Y = right
"""

from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.4              # hover at 40 cm

drone.connect()
drone.arm()
drone.takeoff()

# ── 1. Square ──────────────────────────────────────────
print("\\n--- Square (0.6m sides, 10s, nose forward) ---")
drone.square(
    length=0.6,         # side length in metres
    duration=10,        # total time for the full square
    face_direction=True # drone nose tracks each side direction
)
drone.hover(3)

# ── 2. Triangle ────────────────────────────────────────
print("\\n--- Triangle (0.6m sides, 8s, nose forward) ---")
drone.triangle(
    length=0.6,
    duration=8,
    face_direction=False
)
drone.hover(3)

# ── 3. Circle ──────────────────────────────────────────
# Waypoint count is auto-calculated for smooth arc (≤ 0.05m spacing).
# diameter=1.0m → circumference≈3.14m → ~60 waypoints
print("\\n--- Circle (1.0m diameter, 15s, tangent heading) ---")
drone.circle(
    diameter=1.0,
    duration=8,
    face_direction=True # nose follows the tangent (circular arc direction)
)
drone.hover(3)

# ── 4. Pentagon ────────────────────────────────────────
print("\\n--- Pentagon (0.6m sides, 12s, nose forward) ---")
drone.pentagon(
    length=0.6,
    duration=12,
    face_direction=True
)
drone.hover(2)

# ── Done ───────────────────────────────────────────────
drone.land()
drone.disconnect()

print("\\nDone! All shapes completed.")
