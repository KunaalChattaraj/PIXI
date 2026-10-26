# Level 1 — Sensor Data

Read and understand the drone's sensors without taking off.

| # | Script | What it teaches |
|---|--------|-----------------|
| 01 | `01_battery_voltage.py` | Connect to drone and read live battery voltage |
| 02 | `02_height_data.py` | Read raw ToF and Kalman-filtered height data |
| 03 | `03_position_velocity.py` | Read optical flow-based position and velocity |
| 04 | `04_imu_data.py` | Read orientation (roll/pitch/yaw) and gyroscope data |
| 05 | `05_all_sensors.py` | Complete overview of all available drone sensors |

## 💡 Note

Level 1 examples are **read-only**. They connect to the drone to stream telemetry but do not arm the motors or attempt flight. This is the safest way to verify your connection and sensor health.
