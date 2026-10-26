"""
Level 2 — Live Sensor Visualizations (GUI)
===========================================

This script combines multiple live GUI tools:

Available modes:
  1 - Full Sensor Dashboard (all data)
  2 - Height Plot (filtered vs raw ToF)
  3 - IMU Plot (orientation + gyro)
  4 - Position Trail (XY movement)

What you'll learn:
  - How to visualize real-time drone sensor data
  - Differences between raw vs filtered signals
  - How IMU responds to motion
  - How optical flow estimates position
  - How to select and run different GUI tools

Instructions:
  - Run the script
  - Enter a number (1–4) to choose a visualization
  - Close the window or press Ctrl+C to stop
"""

from litewing import LiteWing
from litewing.gui import (
    live_dashboard,
    live_height_plot,
    live_imu_plot,
    live_position_plot,
)

# ── Setup ────────────────────────────────────────────
drone = LiteWing("192.168.43.42")

# ── Menu ─────────────────────────────────────────────
print("\nSelect visualization mode:")
print("1 - Full Sensor Dashboard")
print("2 - Height Plot (ToF)")
print("3 - IMU Plot")
print("4 - Position Trail")

choice = input("Enter choice (1-4): ").strip()

# ── Run selected GUI ─────────────────────────────────
if choice == "1":
    print("Opening Full Dashboard...")
    live_dashboard(drone)

elif choice == "2":
    print("Opening Height Plot...")
    live_height_plot(drone)

elif choice == "3":
    print("Opening IMU Plot...")
    live_imu_plot(drone)

elif choice == "4":
    print("Opening Position Trail...")
    live_position_plot(drone)

else:
    print("Invalid choice. Please run again and select 1–4.")