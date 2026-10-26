"""
Level 3 — Waypoint Navigation
================================
Fly to absolute positions and follow multi-point paths.

What you'll learn:
    - fly_to(x, y) for absolute position navigation
    - fly_path() for multi-waypoint routes
    - Coordinate system: X = forward/backward, Y = left/right
    - Speed control during waypoint flight

Coordinate system (from drone's starting perspective):
    +X = forward
    -X = backward
    +Y = left
    -Y = right
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")
drone.target_height = 0.3

drone.connect()
time.sleep(2)

print(f"Battery: {drone.battery:.2f}V")

drone.start_logging("my_flight_log.csv")

drone.arm()
drone.takeoff()

# ── Method 1: fly_to (absolute positions) ────────────
print("\n--- fly_to: Triangle pattern ---")

# Fly forward
drone.fly_to(0.6, 0.0, speed=0.25)    # +X = forward
print(f"  Position: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

# Fly forward-left (corner)
drone.fly_to(0.6, 0.6, speed=0.25)    # +X forward, +Y left
print(f"  Position: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

# Move Diagonal Right-Backward (back to origin)
drone.fly_to(0.0, 0.0, speed=0.25)
print(f"  Position: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

drone.hover(2)

# ── Method 2: fly_path (waypoint list) ───────────────
print("\n--- fly_path: Square pattern ---")

# Square pattern movements: Forward → Left → Backward → Right
#
#                        +X (Forward)
#                             ^
#         (0.3, 0.3)          |
#              * <----------- * (0.3, 0.0)
#              |              |
#              |              |
#              v              ^
# +Y (Left) <--*------------> O (0.0, 0.0) Origin
#         (0.0, 0.3)          |
#                             |
#                             v
#                            -X (Backward)
#
square = [
    (0.3,  0.0),   # Move Forward
    (0.3,  0.3),   # Move Left (to Forward-Left corner)
    (0.0,  0.3),   # Move Backward (to Left corner)
    (0.0,  0.0),   # Move Right (back to origin)
]

drone.fly_path(square, speed=0.3)
print(f"  Final: ({drone.position[0]:.2f}, {drone.position[1]:.2f})")

drone.land()
drone.stop_logging()
drone.disconnect()

print("\nDone! Check 'waypoint_flight.csv' for the full trajectory.")
