# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [0.1.4] - 2026-04-06

### Added
- `drone.change_height(delta, min_h, max_h)` — adjust hover altitude in-flight with optional safety floor/ceiling capping.
- `drone.rotate_left(degrees, speed)` / `drone.rotate_right(degrees, speed)` — dedicated yaw rotation with firmware (high-level commander) and library (active yawrate streaming) modes.
- Arrow key support (`up`, `down`, `left`, `right`) in manual control key map.
- Flight safety guard in manual control — warns if movement keys are pressed while not in flight.
- Height adjustment feedback during manual control — logs climbing/descending messages when using R/F keys.
- `MAX_THRUST = 55000` constant added to config defaults.
- New example: `level_2/03_live_sensor_visualization.py`.
- New example: `level_3/01_position_hold_analysis.py`.
- Level 1 example README.

### Changed
- GUI theme overhauled: replaced dark theme (`_apply_dark_theme`) with lightweight theme (`_apply_lightweight_theme`) — light background, stronger accent colors, dashed gridlines.
- GUI rendering performance: removed `constrained_layout`, added `tight_layout()` + `subplots_adjust()`, explicit `blit=False` for reliable animation.
- Hover height safety range changed from hardcoded `0.1m` floor to `0.15m–2.0m` range.
- Convergence wait reduced from 2s to 1s after position reset.
- Log messages simplified: user-facing text (e.g., "Resetting position to origin…").
- `set_key()` now triggers `on_key_press`/`on_key_release` callbacks on state change (previously only updated internal map).
- `import math` moved to module-level (removed per-function duplication).
- Example reorganization:
  - Removed `level_1/with_gui/` subfolder (4 old GUI examples deleted).
  - Removed `level_1/01_basic_flight.py` (moved to level 2).
  - Renamed `level_3/04_manual_control.py` → `04_keyboard_control.py`.
  - Renamed `level_3/05_shape_flight.py` → `05_pattern_navigation.py`.
  - Swapped `level_1/04_all_sensors.py` ↔ `05_imu_data.py` numbering.
  - Replaced `level_2/03_tuning_config.py` with `03_live_sensor_visualization.py`.
  - Replaced `level_3/01_position_hold.py` with `01_position_hold_analysis.py`.
- Updated QUICK_REFERENCE.md and API_REFERENCE.md with new methods and corrections.

### Fixed
- **Direction coordinates swapped** in `pitch_forward`/`pitch_backward`/`roll_left`/`roll_right` — were passing `(dy, dx)` instead of `(dx, dy)` to `_execute_movement`.
- Event chain not bridged from app UI — `set_key()` key press/release callbacks were not firing.
- Stale telemetry log blocks causing missed sensor updates.
- `target_height` now explicitly cast to `float` before rounding (prevents type errors).
- Added debug logging to `land()` for troubleshooting flight state issues.

## [0.1.3] - 2026-03-20

### Added
- Shape flight methods: `drone.square(length, duration, face_direction)`, `drone.triangle()`, `drone.circle()`, `drone.pentagon()`.
- Circle auto-calculates waypoints for smooth arcs (≤ 0.05m spacing) and sends `go_to` commands directly for continuous motion.
- Polygons use edge-walking geometry (corner-start, full side length, clean yaw angles).
- Space key safe landing: pressing **Space** during flight triggers `drone.land()` (vs `Ctrl+C` = emergency stop).
- New example: `examples/level_3/05_shape_flight.py`.
- New test classes: `TestShapeFunctions`, `TestConfigDefaults`, `TestConfigModifiable` in `tests/test_litewing.py`.

### Changed
- Unified takeoff/landing duration config: both firmware and library modes now use `DEFAULT_TAKEOFF_DURATION` and `DEFAULT_LANDING_DURATION` (removed duplicate `TAKEOFF_TIME`/`LANDING_TIME`).
- `takeoff(height, duration)` — first param is **height**, second is **duration**. Previously had inconsistent configs per mode.
- `emergency_stop()` no longer clears LEDs — preserves error notification state.
- `disconnect()` no longer clears LEDs — only detaches the LED controller.
- Position reset before each shape/circle pattern (current hover position becomes origin).
- Yaw normalized to 0°–360° range in shape methods.

### Fixed
- Takeoff duration not applying correctly when passed as argument (`drone.takeoff(duration=1)`).
- Square shape was centered (halved coordinates) instead of corner-start with full side length.
- Circle timing was incorrect — `fly_to()` added 0.5s overhead per waypoint; now uses direct `go_to` for continuous smooth motion.

## [0.1.2] - 2026-03-17

### Changed
- Decoupled connection management from `start_manual_control()`. It now requires an external connection (`drone.connect()`).
- Manual control examples updated to use `with LiteWing() as drone:` and `drone.connect()`.
- `run_manual_control` now automatically calls `drone.arm()` if the drone is not already armed.

## [0.1.1] - 2026-03-12

### Changed
- Version is now auto-synced from `pyproject.toml` via `importlib.metadata` — single source of truth.
- Signal handler (`Ctrl+C` emergency stop) is now opt-in via `install_signal_handler` parameter.
- `install.bat` now works correctly when run from any directory (`pushd`/`popd`).

### Fixed
- Version mismatch between `pyproject.toml` and `__init__.py`.
- README manual install section no longer uses editable mode (`-e`).

## [0.1.0] - 2025-02-17

### Added
- Initial release of the LiteWing Python library.
- `LiteWing` main class with connection, flight, sensor, and LED control.
- Cascaded PID position hold controller using optical flow.
- Manual (keyboard/joystick) control with two hold modes: `"current"` and `"origin"`.
- `PIDConfig` class for tuning position and velocity PID gains.
- `SensorData` snapshot class for reading height, velocity, position, and battery.
- `LEDController` for NeoPixel LED color, blinking, and clearing.
- `FlightLogger` for CSV flight data recording.
- Firmware Z-axis parameter tuning (`thrust_base`, `z_position_kp`, `z_velocity_kp`).
- Waypoint navigation (`fly_to`, `fly_path`).
- Context manager support for safe emergency stops.
- Three tiered example scripts (beginner, intermediate, advanced).
- Full API reference documentation.
