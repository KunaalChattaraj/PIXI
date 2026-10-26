"""
Level 1 — Read IMU Data
========================
Connect to the drone and read Inertial Measurement Unit (IMU) data:
  - Roll, Pitch, Yaw  (orientation in degrees)
  - Gyroscope X, Y, Z (rotation speed in deg/s)

What you'll learn:
    - How the drone knows its orientation
    - What gyroscope readings look like
    - How tilting the drone changes roll/pitch/yaw
"""

import time
from litewing import LiteWing

drone = LiteWing("192.168.43.42")

# Connect (no motors!)
drone.connect()

# Wait for sensor data to arrive
time.sleep(2)

# Read IMU data
sensors = drone.read_sensors()

print("IMU Sensor Readings:")
print(f"  Roll:  {sensors.roll:7.2f}°   (tilt left/right)")
print(f"  Pitch: {sensors.pitch:7.2f}°   (tilt forward/back)")
print(f"  Yaw:   {sensors.yaw:7.2f}°   (rotation)")
print()
print(f"  Gyro X: {sensors.gyro_x:8.2f} °/s")
print(f"  Gyro Y: {sensors.gyro_y:8.2f} °/s")
print(f"  Gyro Z: {sensors.gyro_z:8.2f} °/s")

# Continuous reading for 10 seconds
print("\nLive IMU readings (10 seconds):")
print("Try tilting the drone gently to see values change!")
print(f"  {'#':>3}  {'Roll°':>7}  {'Pitch°':>7}  {'Yaw°':>7}  {'GyroX':>8}  {'GyroY':>8}  {'GyroZ':>8}")
print(f"  {'---':>3}  {'-------':>7}  {'-------':>7}  {'-------':>7}  {'--------':>8}  {'--------':>8}  {'--------':>8}")

for i in range(20):
    sensors = drone.read_sensors()
    print(
        f"  {i+1:3d}  "
        f"{sensors.roll:7.2f}  "
        f"{sensors.pitch:7.2f}  "
        f"{sensors.yaw:7.2f}  "
        f"{sensors.gyro_x:8.2f}  "
        f"{sensors.gyro_y:8.2f}  "
        f"{sensors.gyro_z:8.2f}"
    )
    time.sleep(0.5)

# Disconnect
drone.disconnect()
print("\nDone!")
