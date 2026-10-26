# LiteWing Quick Reference

A simple lookup table of every function and property you'll use.

---

## 🔌 Connection

| Function | What it does | When to use |
|---|---|---|
| `drone = LiteWing("192.168.43.42")` | Create a drone controller | Always - first line of every script |
| `drone.connect()` | Connect to the drone, start sensor data | Before reading sensors or flying |
| `drone.disconnect()` | Disconnect safely | End of every script |

**Example:**
```python
drone = LiteWing("192.168.43.42")
drone.connect()
print(f"Battery: {drone.battery:.2f}V")
drone.disconnect()
```

---

## 🛫 Flight Commands

| Function | What it does | When to use |
|---|---|---|
| `drone.arm()` | Prepare drone for flight | After `connect()`, before `takeoff()` |
| `drone.takeoff(height, duration)` | Take off and hover *(blocking)* | After `arm()` |
| `drone.change_height(delta, min_h, max_h)` | Change hover height safely | During flight |
| `drone.hover(seconds)` | Hover in place for N seconds | Between maneuvers |
| `drone.land(duration)` | Descend and stop motors | When done flying |
| `drone.emergency_stop()` | **Kill all motors immediately** | Emergency only! Drone will fall |

**Typical flow:**
```python
drone.connect()
drone.arm()
drone.takeoff(0.3, 0.1)     # drone lifts off 30cm within 0.1s duration
drone.hover(5)              # hover for 5 seconds
drone.land()                # come back down
drone.disconnect()
```

> See: `level_2/02_basic_flight.py`

---

## 📋 Status Properties

| Property | Type | What it tells you |
|---|---|---|
| `drone.is_connected` | `bool` | Connected to drone? |
| `drone.is_flying` | `bool` | Currently in flight? |
| `drone.flight_phase` | `str` | Current state: `IDLE`, `TAKEOFF`, `HOVERING`, `LANDING`, etc. |
---

## ⚡ Safety Keys

| Key | Action | Effect |
|---|---|---|
| `Ctrl+C` | Emergency stop | Immediately cuts all motors - drone falls |
| `Space` | Safe landing | Triggers `drone.land()` - controlled descent |

---

---

## 💡 LED Control

| Function | What it does | When to use |
|---|---|---|
| `drone.set_led_color(r, g, b)` | Set all LEDs to a color | Visual status (green=go, red=stop) |
| `drone.set_led(index, r, g, b)` | Set one LED (0-3) to a color | Individual LED patterns |
| `drone.blink_leds(on_ms, off_ms)` | Blink LEDs | Warnings, attention |
| `drone.clear_leds()` | Turn off all LEDs | When done |

The drone has **4 LEDs** numbered 0-3. Colors use RGB values 0-255:
```python
# All LEDs same color
drone.set_led_color(255, 0, 0)    # All red
drone.set_led_color(0, 255, 0)    # All green

# Individual LEDs
drone.set_led(0, 255, 0, 0)      # LED 0 = red
drone.set_led(1, 0, 255, 0)      # LED 1 = green
drone.set_led(2, 0, 0, 255)      # LED 2 = blue
drone.set_led(3, 255, 255, 0)    # LED 3 = yellow
```

> See: `level_2/01_led_control.py`

## 🏃 Movement Commands

All movement commands are **blocking** - they return when the drone arrives.

| Function | What it does | When to use |
|---|---|---|
| `drone.pitch_forward(distance, speed)` | Pitch forward | push the drone forward by distance, speed in m/s |
| `drone.pitch_backward(distance, speed)` | Pitch backward | same as forward |
| `drone.roll_left(distance, speed)` | Roll left | same |
| `drone.roll_right(distance, speed)` | Roll right | same |
| `drone.rotate_left(degrees, speed)` | Rotate left (counter-clockwise) | Rotate the drone by degrees |
| `drone.rotate_right(degrees, speed)` | Rotate right (clockwise) | Rotate the drone by degrees |

- `distance` defaults to `0.5m` (= `drone.maneuver_distance`)
- `speed` defaults to `0.2 m/s` (= max velocity setpoint)

**Example:**
```python
drone.arm()
drone.takeoff(0.3, 0.1)
drone.hover(1)
drone.pitch_forward(0.3)               # 30cm forward at 0.2 m/s
drone.pitch_forward(0.3, speed=0.7)    # same distance, faster
drone.roll_right(0.3)                  # 30cm to the right
drone.land()
```

> See: `level_3/02_movement_commands.py`

---

## 🗺️ Waypoint Navigation

| Function | What it does | When to use |
|---|---|---|
| `drone.fly_to(x, y, z=None, yaw=None, speed=None, threshold=None)` | Fly to 3D position with optional rotation | Navigate to a specific coordinate/heading |
| `drone.fly_path(waypoints, speed=None, threshold=None)` | Fly through flexible waypoints: `(x,y)`, `(x,y,z)`, or `(x,y,z,yaw)` | Fly a custom 3D shape or route |

**Coordinate system** (from drone's starting point):
- `+X` = forward, `-X` = backward
- `+Y` = left, `-Y` = right

**Example:**
```python
drone.arm()
drone.takeoff(0.3, 0.1)
drone.hover(1)
# Fly a triangle
drone.fly_to(0.3, 0.0)       # forward
drone.fly_to(0.0, -0.3)      # right
drone.fly_to(0.0, 0.0)       # back to start
drone.land()

# Or use fly_path for a square
drone.arm()
drone.takeoff(0.3, 0.1)
drone.hover(1)
drone.fly_path([
    (0.3, 0.0),  # Pitch Forward
    (0.3, -0.3), # Roll Right
    (0.0, -0.3), # Pitch Backward
    (0.0, 0.0),  # Roll Right (orgin)
], speed=0.3)
drone.land()
drone.disconnect()
```

> See: `level_3/03_waypoint_navigation.py`

---

## ⭐ Shape Flight

| Function | What it does | When to use |
|---|---|---|
| `drone.square(length, duration, face_direction)` | Fly a square path | Geometric patterns |
| `drone.triangle(length, duration, face_direction)` | Fly an equilateral triangle | Geometric patterns |
| `drone.circle(diameter, duration, face_direction)` | Fly a smooth circle | Smooth circular arcs |
| `drone.pentagon(length, duration, face_direction)` | Fly a regular pentagon | Geometric patterns |

- `face_direction=True` (default): drone nose faces direction of travel
- `face_direction=False`: drone keeps its initial heading
- All shapes reset position to origin before starting

**Example:**
```python
drone.square(length=0.6, duration=10, face_direction=True)  # 60cm square in 10s
drone.circle(diameter=1.0, duration=8, face_direction=True) # 1m diameter circle in 8s
drone.triangle(length=0.5, duration=8, face_direction=False)  # fixed heading
```

> See: `level_3/05_shape_flight.py`

---



## 📡 Sensor Reading

| Property | Returns | What it tells you |
|---|---|---|
| `drone.battery` | `float` (volts) | Battery voltage (3.0V = empty, 4.2V = full) |
| `drone.height` | `float` (meters) | How high the drone is |
| `drone.position` | `(x, y)` tuple | Estimated position from launch point |
| `drone.velocity` | `(vx, vy)` tuple | How fast the drone is moving |
| `drone.read_sensors()` | `SensorData` object | All sensor data in one snapshot |
| `sensors.roll` | `float` (degrees) | Tilt left/right |
| `sensors.pitch` | `float` (degrees) | Tilt forward/back |
| `sensors.yaw` | `float` (degrees) | Rotation heading |
| `sensors.acc_x/y/z` | `float` (g) | Acceleration on each axis |
| `sensors.gyro_x/y/z` | `float` (deg/s) | Angular rate on each axis |

**Example:**
```python
print(f"Battery: {drone.battery:.2f}V")
print(f"Height:  {drone.height:.2f}m")
print(f"Position: {drone.position}")

sensors = drone.read_sensors()
print(f"Roll: {sensors.roll:.1f}°  Pitch: {sensors.pitch:.1f}°  Yaw: {sensors.yaw:.1f}°")
```

---

## 📊 Data Logging

| Function | What it does | When to use |
|---|---|---|
| `drone.start_logging("filename.csv")` | Start recording flight data | Before `arm()` |
| `drone.stop_logging()` | Stop recording, close CSV | After `land()` |

**Logged columns:** Timestamp, Position X/Y, Height, Velocity X/Y, PID Corrections

> See: `level_2/04_data_logging.py`

---

## 📈 Live Visualization (GUI)

| Function | What it shows | When to use |
|---|---|---|
| `drone.start_dashboard()` | 4-panel: height, attitude, velocity, battery | Full flight monitoring |
| `drone.start_height_plot()` | Filtered + raw height | Height sensor testing |
| `drone.start_imu_plot()` | Roll, pitch, yaw angles | IMU testing |
| `drone.start_position_plot()` | 2D XY position trail | Position hold testing |
| `drone.stop_plot()` | Stops the background plot | When visualization is done |

Requires `matplotlib`. Runs in the background (non-blocking) so your flight code continues.

```python
# Setup plot before takeoff
plot = drone.start_dashboard()

drone.arm()
drone.takeoff()
# ... fly ...
drone.land()

# Stop when done
drone.stop_plot() 
```

> See: `level_1/with_gui/` examples

---

## 🎮 Manual (Keyboard) Control

| Function | What it does | When to use |
|---|---|---|
| `drone.start_manual_control()` | Full automated flight with WASD QERF control: connect → takeoff → keyboard loop → land | Interactive flight sessions |
| `drone.stop_manual_control()` | Land and end manual mode | Programmatic stop (or press SPACE/Ctrl+C) |


**Controls:** `W`=Forward, `S`=Backward, `A`=Left, `D`=Right, `Q`=Rotate Left `E`=Rotate Right `R`=Up `F`=Down `SPACE`=Land

### Manual Control Settings
| Property | Default | What it does |
|---|---|---|
| `drone.sensitivity` | `0.2` | Speed per key press (m/s) |
| `drone.hold_mode` | `"current"` | `"current"` = stay put, `"origin"` = snap back |

> See: `level_3/04_keyboard_control.py`

---
## 🕹️ Raw Control (Without LiteWing Flight Positioning Module)

| Function | What it does | When to use |
|---|---|---|
| `drone.send_control(roll, pitch, yawrate, thrust)` | Send raw motor commands directly | Sensorless flight, testing motors |

- `roll` / `pitch`: tilt angles in degrees (−30 to +30)
- `yawrate`: spin rate in deg/s (−200 to +200)
- `thrust`: motor power (0–33500, ~28000 = hover)

> ⚠️ Start thrust low (~15000) and increase slowly. Use `emergency_stop()` if needed.

```python
drone.connect()
drone.send_control(thrust=0)    # Must send a zero-thrust setpoint first to unlock the commander
time.sleep(0.5)
drone.send_control(thrust=28500)     # gentle lift
time.sleep(2)
drone.disconnect()
```

---

## ⚙️ Tunable Parameters

### Flight Settings
| Property | Default | What to change it for |
|---|---|---|
| `drone.target_height` | `0.3` | Fly higher or lower |
| `drone.default_takeoff_duration` | `0.1` | How long the takeoff takes (seconds) |
| `drone.default_landing_duration` | `2.0` | How long the landing takes (seconds) |
| `drone.default_flight_speed` | `0.7` | Default speed for `fly_to()` (m/s) |
| `drone.max_flight_speed` | `2` | Safety clamp for flight speed (m/s) |
| `drone.maneuver_distance` | `0.5` | Default move distance |
| `drone.max_correction` | `0.7` | Speed cap for PID corrections |
| `drone.descent_rate` | `0.25` | Landing descent speed (m/s) |
| `drone.debug_mode` | `False` | Set `True` to test without motors |
| `drone.enable_sensor_check` | `True` | Set `False` to skip ToF/flow check on `arm()` |

### PID Tuning
| Property | Default | Effect of increasing |
|---|---|---|
| `drone.position_pid.kp` | `1.0` | Stronger push toward target |
| `drone.position_pid.ki` | `0.03` | Corrects persistent drift |
| `drone.position_pid.kd` | `0.0` | Dampens oscillation |
| `drone.velocity_pid.kp` | `0.7` | More speed damping |
| `drone.velocity_pid.ki` | `0.01` | Corrects velocity bias |
| `drone.velocity_pid.kd` | `0.0` | Smooths rapid changes |

> See: `level_2/03_tuning_config.py`

### Drift Correction
| Property | Default | What it does |
|---|---|---|
| `drone.hover_trim_pitch` | `0.0` | Forward/backward drift correction (hover mode). Positive = nudge forward. |
| `drone.hover_trim_roll` | `0.0` | Left/right drift correction (hover mode). Positive = nudge right. |
| `drone.raw_trim_roll` | `0.0` | Roll trim for raw control mode (degrees). |
| `drone.raw_trim_pitch` | `0.0` | Pitch trim for raw control mode (degrees). |

### Firmware (Height Controller)
| Property | Default | What it does |
|---|---|---|
| `drone.thrust_base` | `24000` | Base motor power (increase if heavy) |
| `drone.z_position_kp` | `1.6` | Height correction strength |
| `drone.z_velocity_kp` | `15.0` | Vertical speed damping |
| `drone.apply_firmware_params()` | - | Send these values to the drone |



### Raw Control Safety
| Property | Default | What it does |
|---|---|---|
| `drone.max_thrust` | `35000` | Safety cap for `send_control()` thrust |
| `drone.raw_trim_roll` | `0.0` | Roll trim for raw control (degrees) |
| `drone.raw_trim_pitch` | `0.0` | Pitch trim for raw control (degrees) |

