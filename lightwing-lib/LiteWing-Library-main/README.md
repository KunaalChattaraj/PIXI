# LiteWing-Library

**Beginner-friendly Python library for LiteWing drone control.**

Using this library, you will be able to fly your drone using simple lines of Python code. For example, you can ask it to take off and hold its position at a particular height, then land automatically, or even move in a particular direction for a fixed distance and then land.

## Documentation
Read Project Documentation: [LiteWing Python Library](https://circuitdigest.com/articles/litewing-drone-python-library-documentation)

## Installation

### Quick Install (Windows)
Double-click **`install.bat`** - it checks your Python version and installs everything automatically.

### Quick Install (macOS / Linux)
Open a terminal in the `litewing-library` folder and run:
```bash
chmod +x install.sh
./install.sh
```

### Manual Install
```bash
# Navigate to the litewing-library folder
cd litewing-library

# Install the library
pip install .
```

### Requirements
- **Python 3.11** (mandatory)
  - Download: [Python 3.11](https://www.python.org/downloads/release/python-3119/)
  - Check **"Add Python to PATH"** during installation!
- [cflib](https://github.com/Circuit-Digest/crazyflie-lib-python) (installed automatically)
- [matplotlib](https://matplotlib.org/) (installed automatically)


## API Reference

See [API_REFERENCE.md](litewing/API_REFERENCE.md) for the full list of every function, property, and configurable parameter.

## Project Structure

```
litewing-library/
├── pyproject.toml          # Package metadata & build config
├── README.md               # This file
├── LICENSE                 # MIT License
├── CHANGELOG.md            # Version history
├── QUICK_REFERENCE.md      # Cheat-sheet for common patterns
├── litewing/               # The library package
│   ├── __init__.py         # Public exports: LiteWing, SensorData, PIDConfig
│   ├── litewing.py         # Main LiteWing class
│   ├── config.py           # All default constants
│   ├── pid.py              # PID controller
│   ├── sensors.py          # SensorData snapshot class
│   ├── position_hold.py    # Position hold controller
│   ├── manual_control.py   # Joystick/keyboard control
│   ├── leds.py             # NeoPixel LED control
│   ├── logger.py           # CSV flight data logger
│   ├── gui.py              # Live GUI / plot windows
│   ├── _connection.py      # Internal: cflib management
│   ├── _crtp.py            # Internal: CRTP packets
│   ├── _position.py        # Internal: dead reckoning
│   ├── _flight_engine.py   # Internal: flight state machine
│   ├── _plot_runner.py     # Internal: background plot thread
│   ├── _safety.py          # Internal: link/battery checks
│   └── API_REFERENCE.md    # Full API docs
├── examples/               # Example scripts (by level)
│   ├── level_0/            # Quick-start scripts
│   ├── level_1/            # Sensor reading - no flying
│   │   ├── 01_battery_voltage.py
│   │   ├── 02_height_data.py
│   │   ├── 03_position_velocity.py
│   │   ├── 04_imu_data.py
│   │   └── 05_all_sensors.py
│   ├── level_2/            # Basic flight control
│   │   ├── 01_led_control.py
│   │   ├── 02_basic_flight.py
│   │   ├── 03_live_sensor_visualization.py
│   │   └── 04_data_logging.py
│   └── level_3/            # waypoints, advanced
│       ├── 01_position_hold_analysis.py
│       ├── 02_movement_commands.py
│       ├── 03_waypoint_navigation.py
│       ├── 04_keyboard_control.py
│       └── 05_pattern_navigation.py
```

## License

MIT - see [LICENSE](LICENSE) for details.
