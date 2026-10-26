"""
LiteWing - Beginner-Friendly Drone Library
=============================================
Author: Dharagesh

Control your LiteWing drone with simple, educational Python code.

Quick Start:
    from litewing import LiteWing

    with LiteWing("192.168.43.42") as drone:
        drone.arm()
        drone.takeoff()
        drone.pitch_forward(0.5, speed=0.2)
        drone.land()

The LiteWing class is your "remote control." It handles all the low-level
communication, but keeps drone engineering concepts visible so you can learn
about sensors, PID tuning, position hold, and flight control.
"""

import time
import atexit
import signal
import threading
import math
from typing import List, Tuple, Union

from .config import defaults
from .pid import PIDConfig
from .sensors import SensorData, _SensorState
from .leds import LEDController
from .logger import FlightLogger
from ._position import PositionEngine
from .position_hold import PositionHoldController
from . import _flight_engine
from . import manual_control as _manual_control


class _PositionConfigProxy:
    """Provides a config.defaults-compatible interface mapping to LiteWing instance properties."""
    def __init__(self, drone):
        self._drone = drone
    
    @property
    def SENSOR_PERIOD_MS(self): return self._drone.sensor_update_rate
    @property
    def USE_HEIGHT_SCALING(self): return self._drone.use_height_scaling
    @property
    def OPTICAL_FLOW_SCALE(self): return self._drone.optical_flow_scale
    @property
    def VELOCITY_SMOOTHING_ALPHA(self): return self._drone.velocity_smoothing
    @property
    def VELOCITY_THRESHOLD(self): return self._drone.velocity_threshold
    @property
    def DRIFT_COMPENSATION_RATE(self): return self._drone.drift_compensation_rate
    @property
    def MAX_POSITION_ERROR(self): return self._drone.max_position_error
    @property
    def PERIODIC_RESET_INTERVAL(self): return self._drone.position_reset_interval
    @property
    def CONTROL_UPDATE_RATE(self): return self._drone.control_update_rate
    @property
    def MAX_CORRECTION(self): return self._drone.max_correction
    @property
    def DEG_TO_RAD(self): return defaults.DEG_TO_RAD


class LiteWing:
    """
    Main drone controller — your "remote control."

    Use this class to connect, fly, read sensors, and tune the drone.
    Supports context manager (with statement) for safe auto-disconnects.

    Args:
        ip: The IP address of the drone (default: "192.168.43.42").
    """

    def __init__(self, ip=None, install_signal_handler=True):
        # === Connection ===
        self._ip = ip or defaults.DRONE_IP
        self._port = defaults.DRONE_PORT
        self._scf = None
        self._flight_active = False
        self._flight_phase = "IDLE"
        self._cmd_height = 0.0  # actual commanded height (for logging)
        self._flight_thread = None
        self._manual_active = False
        self._manual_thread = None
        self._logger_fn = print  # Default output function

        # === Exposed Configuration (Tier 1–3) ===

        # Flight parameters
        self.target_height = defaults.TARGET_HEIGHT
        self.descent_rate = defaults.DESCENT_RATE
        self.hover_duration = defaults.HOVER_DURATION
        self.enable_takeoff_ramp = defaults.ENABLE_TAKEOFF_RAMP

        # Trim corrections
        self.hover_trim_pitch = defaults.TRIM_VX
        self.hover_trim_roll = defaults.TRIM_VY
        self.raw_trim_roll = 0.0    # Raw-mode roll trim (degrees)
        self.raw_trim_pitch = 0.0   # Raw-mode pitch trim (degrees)

        # Debug & safety
        self.debug_mode = defaults.DEBUG_MODE
        self.enable_height_sensor_safety = defaults.ENABLE_HEIGHT_SENSOR_SAFETY
        self.enable_csv_logging = defaults.ENABLE_CSV_LOGGING
        self.enable_sensor_check = True  # Check ToF / flow sensors on connect
        self.max_thrust = 35000  # Safety cap for raw thrust (0-65535)
        self._height_sensor_min_change = defaults.HEIGHT_SENSOR_MIN_CHANGE

        # PID controllers (VISIBLE — learners tune these!)
        self.position_pid = PIDConfig(
            kp=defaults.POSITION_KP,
            ki=defaults.POSITION_KI,
            kd=defaults.POSITION_KD,
        )
        self.velocity_pid = PIDConfig(
            kp=defaults.VELOCITY_KP,
            ki=defaults.VELOCITY_KI,
            kd=defaults.VELOCITY_KD,
        )

        # Position hold settings
        self.max_correction = defaults.MAX_CORRECTION
        self.velocity_threshold = defaults.VELOCITY_THRESHOLD
        self.drift_compensation_rate = defaults.DRIFT_COMPENSATION_RATE
        self.position_reset_interval = defaults.PERIODIC_RESET_INTERVAL
        self.max_position_error = defaults.MAX_POSITION_ERROR
        self.optical_flow_scale = defaults.OPTICAL_FLOW_SCALE
        self.use_height_scaling = defaults.USE_HEIGHT_SCALING
        self.velocity_smoothing = defaults.VELOCITY_SMOOTHING_ALPHA

        # Approach damping settings
        self.approach_damping_distance = defaults.APPROACH_DAMPING_DISTANCE
        self.approach_damping_velocity = defaults.APPROACH_DAMPING_VELOCITY
        self.approach_damping_factor = defaults.APPROACH_DAMPING_FACTOR

        # Joystick / manual control
        self.sensitivity = defaults.JOYSTICK_SENSITIVITY
        self.hold_mode = defaults.JOYSTICK_HOLD_MODE
        self.commander_mode = defaults.COMMANDER_MODE
        self.momentum_compensation_time = defaults.MOMENTUM_COMPENSATION_TIME
        self.settling_duration = defaults.SETTLING_DURATION
        self.settling_correction_factor = defaults.SETTLING_CORRECTION_FACTOR
        self._manual_keys = {
            "w": False, "s": False, "a": False, "d": False,
            "q": False, "e": False, "r": False, "f": False,
            "up": False, "down": False, "left": False, "right": False
        }

        # Firmware parameters
        self.enable_firmware_params = defaults.ENABLE_FIRMWARE_PARAMS
        self.thrust_base = defaults.FW_THRUST_BASE
        self.z_position_kp = defaults.FW_Z_POS_KP
        self.z_velocity_kp = defaults.FW_Z_VEL_KP

        # Control loop timing
        self.sensor_update_rate = defaults.SENSOR_PERIOD_MS
        self.control_update_rate = defaults.CONTROL_UPDATE_RATE

        # Waypoint parameters
        self.waypoint_timeout = defaults.WAYPOINT_TIMEOUT
        self.waypoint_threshold = defaults.MANEUVER_THRESHOLD
        self.waypoint_stabilization_time = defaults.WAYPOINT_STABILIZATION_TIME
        self.maneuver_distance = defaults.MANEUVER_DISTANCE

        # Position hold mode (firmware or library)
        self.position_hold_mode = defaults.POSITION_HOLD_MODE

        # Firmware flight defaults
        self.default_takeoff_duration = defaults.DEFAULT_TAKEOFF_DURATION
        self.default_landing_duration = defaults.DEFAULT_LANDING_DURATION
        self.default_flight_speed = defaults.DEFAULT_FLIGHT_SPEED
        self.max_flight_speed = defaults.MAX_FLIGHT_SPEED

        # === Internal state (hidden from learners) ===
        self._position_cfg_proxy = _PositionConfigProxy(self)
        self._sensors = _SensorState()
        self._position_engine = PositionEngine(cfg=self._position_cfg_proxy)
        self._position_hold = PositionHoldController(
            self.position_pid, self.velocity_pid, cfg=self._position_cfg_proxy
        )
        self._leds = LEDController()
        self._flight_logger = FlightLogger()
        self._hover_duration = defaults.HOVER_DURATION

        # Callbacks for events
        self._on_key_press_cb = None
        self._on_key_release_cb = None

        # === System-wide Ctrl+C emergency stop ===
        # This ensures pressing Ctrl+C ALWAYS kills motors immediately,
        # even if the student forgot to add error handling.
        # Set install_signal_handler=False if embedding in a larger app.
        if install_signal_handler:
            def _ctrl_c_handler(sig, frame):
                print("\n Ctrl+C detected — EMERGENCY STOP!")
                self.emergency_stop()
                raise SystemExit(1)

            signal.signal(signal.SIGINT, _ctrl_c_handler)

            # Space key safe landing (runs in a background thread)
            self._land_key_thread = None
            self._land_key_active = False
            self._start_land_key_listener()

    # === Context Manager (with statement) ===

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    # === Space Key Safe Landing ===

    def _start_land_key_listener(self):
        """
        Start a background thread that listens for the space key.
        When pressed, triggers a safe landing (not emergency stop).
        """
        import threading

        if self._land_key_active:
            return

        self._land_key_active = True

        def _listener():
            try:
                import msvcrt  # Windows only
                while self._land_key_active:
                    if msvcrt.kbhit():
                        key = msvcrt.getch()
                        if key == b' ':  # space key
                            if self._flight_active:
                                print("\n [SPACE] Safe landing triggered!")
                                # Run land() in a separate thread to avoid blocking
                                threading.Thread(
                                    target=self._safe_land_from_key,
                                    daemon=True
                                ).start()
                    time.sleep(0.05)
            except ImportError:
                # Non-Windows: try termios-based approach
                try:
                    import sys, tty, termios, select
                    fd = sys.stdin.fileno()
                    old = termios.tcgetattr(fd)
                    try:
                        tty.setcbreak(fd)
                        while self._land_key_active:
                            if select.select([sys.stdin], [], [], 0.05)[0]:
                                ch = sys.stdin.read(1)
                                if ch == ' ' and self._flight_active:
                                    print("\n [SPACE] Safe landing triggered!")
                                    threading.Thread(
                                        target=self._safe_land_from_key,
                                        daemon=True
                                    ).start()
                    finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old)
                except Exception:
                    pass  # no keyboard available (e.g. headless)

        self._land_key_thread = threading.Thread(
            target=_listener, daemon=True, name="LiteWing-LandKey"
        )
        self._land_key_thread.start()

    def _safe_land_from_key(self):
        """Called when space key is pressed — performs a normal land()."""
        try:
            self.land()
        except Exception as e:
            if self._logger_fn:
                self._logger_fn(f"Land-key error: {e}")
            self.emergency_stop()

    # === Connection ===

    def connect(self):
        """
        Connect to the drone and start reading sensor data.

        After calling this, you can read battery, height, position, etc.
        No motors are started — the drone stays on the ground.

        Usage:
            drone.connect()
            print(drone.battery)
            drone.disconnect()

        Or use the context manager:
            with LiteWing("192.168.43.42") as drone:
                drone.connect()
                print(drone.battery)
        """
        if self._scf is not None:
            if self._logger_fn:
                self._logger_fn("Already connected!")
            return

        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from ._connection import setup_sensor_logging, setup_debug_wrappers

        cflib.crtp.init_drivers()
        uri = f"udp://{self._ip}:{self._port}"

        self._flight_phase = "CONNECTING"
        if self._logger_fn:
            self._logger_fn(f"Connecting to {uri}...")

        # Auto-retry with port recovery (up to 3 attempts)
        last_error = None
        for attempt in range(3):
            cf = Crazyflie(rw_cache="./cache")
            setup_debug_wrappers(cf, logger_fn=self._logger_fn)
            self._cf_instance = cf
            self._sync_cf = SyncCrazyflie(uri, cf=cf)

            try:
                self._sync_cf.open_link()
                last_error = None
                break  # Success!
            except Exception as e:
                last_error = e
                self._cf_instance = None
                self._sync_cf = None

                # Check if it's a port conflict (WinError 10048)
                if "10048" in str(e) or "Address already in use" in str(e):
                    if self._logger_fn:
                        self._logger_fn(
                            f"Port 2399 busy (previous session). "
                            f"Recovering... (attempt {attempt + 1}/3)"
                        )
                    self._free_port_2399()
                    time.sleep(2)
                else:
                    # Not a port conflict — fail immediately
                    self._flight_phase = "IDLE"
                    raise ConnectionError(
                        f"Failed to connect to {uri}: {e}"
                    ) from e

        if last_error is not None:
            self._flight_phase = "IDLE"
            raise ConnectionError(
                f"Failed to connect to {uri} after 3 attempts. "
                f"Try closing all other Python scripts and wait a few seconds.\n"
                f"Error: {last_error}"
            )

        self._scf = self._sync_cf

        # Register cleanup so port is ALWAYS freed, even on crash
        atexit.register(self._atexit_cleanup)

        # Attach LEDs
        self._leds.attach(cf)

        # Start sensor logging
        self._log_motion, self._log_battery, self._log_imu, self._log_thrust = setup_sensor_logging(
            cf,
            motion_callback=self._motion_callback,
            battery_callback=self._battery_callback,
            imu_callback=self._imu_callback,
            sensor_period_ms=self.sensor_update_rate,
            logger=self._logger_fn,
        )

        # Apply firmware parameters if enabled
        if self.enable_firmware_params:
            from ._connection import apply_firmware_parameters
            apply_firmware_parameters(
                cf, self.thrust_base, self.z_position_kp, self.z_velocity_kp,
                logger=self._logger_fn,
            )

        # Reset position tracking
        self._position_engine.reset()
        self._position_hold.reset()

        # === Sensor health check ===
        # Read firmware params to verify external sensors initialized OK
        self._sensor_health = self._check_sensor_health(cf)

        self._flight_phase = "CONNECTED"
        if self._logger_fn:
            self._logger_fn("Connected! Sensor data streaming.")

    def disconnect(self):
        """
        Disconnect from the drone and stop sensor logging.

        Safe to call even if not connected.
        """
        from ._connection import stop_logging_configs

        # Stop background plot if running
        try:
            self.stop_plot()
        except Exception:
            pass

        # Stop motors if flying
        if self._flight_active:
            self._flight_active = False
            if self._scf and not self.debug_mode:
                try:
                    if self.position_hold_mode == "firmware":
                        self._scf.cf.high_level_commander.stop()
                    else:
                        self._scf.cf.commander.send_setpoint(0, 0, 0, 0)
                except Exception:
                    pass

        # Stop logging (suppress errors during teardown)
        try:
            log_m = getattr(self, '_log_motion', None)
            log_b = getattr(self, '_log_battery', None)
            log_i = getattr(self, '_log_imu', None)
            log_t = getattr(self, '_log_thrust', None)
            stop_logging_configs(log_m, log_b, log_i, log_t)
        except Exception:
            pass
        try:
            self._flight_logger.stop(logger=self._logger_fn)
        except Exception:
            pass

        # Detach LEDs (do NOT clear — preserves error notification state)
        try:
            self._leds.detach()
        except Exception:
            pass

        # Close link
        if self._scf is not None:
            try:
                sync = getattr(self, '_sync_cf', None)
                if sync:
                    sync.close_link()
            except Exception:
                pass
            self._scf = None
            self._sync_cf = None
            self._cf_instance = None

        self._flight_phase = "IDLE"
        if self._logger_fn:
            self._logger_fn("Disconnected.")

    def _atexit_cleanup(self):
        """Called automatically when Python exits — ensures port is freed."""
        try:
            if self._scf is not None:
                self.disconnect()
        except Exception:
            pass

    @staticmethod
    def _free_port_2399():
        """
        Kill any zombie Python process holding UDP port 2399.
        This happens when a previous script crashed without disconnecting.
        """
        import subprocess
        import os

        try:
            # Find process using port 2399
            result = subprocess.run(
                ["netstat", "-ano", "-p", "UDP"],
                capture_output=True, text=True, timeout=5
            )
            for line in result.stdout.splitlines():
                if ":2399 " in line:
                    parts = line.split()
                    pid = int(parts[-1])
                    # Don't kill ourselves!
                    if pid != os.getpid() and pid != 0:
                        try:
                            subprocess.run(
                                ["taskkill", "/F", "/PID", str(pid)],
                                capture_output=True, timeout=5
                            )
                        except Exception:
                            pass
        except Exception:
            pass

    def _check_sensor_health(self, cf):
        """
        Check if external sensors (ToF, optical flow) initialized OK.

        Reads firmware params that the drone sets during boot:
            deck.bcZRanger2  → 1 if VL53L1x ToF sensor initialized
            deck.bcFlow2     → 1 if PMW3901 optical flow initialized

        Returns:
            dict: {"tof": bool, "flow": bool} — True if sensor is OK.
        """
        health = {"tof": False, "flow": False}

        try:
            tof_ok = int(cf.param.get_value("deck.bcZRanger2"))
            flow_ok = int(cf.param.get_value("deck.bcFlow2"))

            health["tof"] = bool(tof_ok)
            health["flow"] = bool(flow_ok)

            if self._logger_fn:
                # ToF status
                if health["tof"]:
                    self._logger_fn("  [OK] ToF sensor (VL53L1x) detected")
                else:
                    self._logger_fn(
                        "  [FAIL] ToF sensor (VL53L1x) NOT detected!\n"
                        "         -> Check I2C wiring to the ToF module"
                    )

                # Optical flow status
                if health["flow"]:
                    self._logger_fn("  [OK] Optical flow (PMW3901) detected")
                else:
                    self._logger_fn(
                        "  [FAIL] Optical flow (PMW3901) NOT detected!\n"
                        "         -> Check SPI wiring to the flow sensor"
                    )
        except Exception as e:
            if self._logger_fn:
                self._logger_fn(
                    f"  [WARN] Could not read sensor status: {e}"
                )

        return health

    @property
    def is_connected(self):
        """True if the drone is currently connected."""
        return self._scf is not None

    @property
    def is_flying(self):
        """True if the drone is currently in flight."""
        return self._flight_active

    @property
    def flight_phase(self):
        """Current phase: IDLE, CONNECTING, TAKEOFF, HOVERING, LANDING, etc."""
        return self._flight_phase

    @property
    def debug_print_mode(self):
        """If True, prints every command sent to the drone."""
        return defaults.DEBUG_PRINT_MODE

    @debug_print_mode.setter
    def debug_print_mode(self, value):
        defaults.DEBUG_PRINT_MODE = bool(value)

    # === Sensor Access ===

    def read_sensors(self):
        """
        Get a snapshot of all current sensor readings.

        Returns:
            SensorData: Object with height, vx, vy, x, y, battery, etc.
        """
        return self._sensors.snapshot(self._position_engine)

    @property
    def battery(self):
        """Current battery voltage in volts."""
        return self._sensors.battery_voltage

    @property
    def height(self):
        """Current estimated height in meters."""
        return self._sensors.height

    @property
    def position(self):
        """Current estimated (x, y) position in meters."""
        return (self._position_engine.x, self._position_engine.y)

    @property
    def velocity(self):
        """Current estimated (vx, vy) velocity in m/s."""
        return (self._position_engine.vx, self._position_engine.vy)

    # === Flight Commands (Tier 1) ===

    def arm(self):
        """
        Arm the drone — prepare it for flight.

        This must be called before takeoff. Arming initializes the
        flight commander so motors can spin.
        """
        if self._scf is None:
            raise RuntimeError("Not connected! Call connect() first.")

        # Block flight if sensors are missing (safety for students)
        if self.enable_sensor_check:
            health = getattr(self, '_sensor_health', {})
            missing = []
            if not health.get('tof', False):
                missing.append('ToF sensor (VL53L1x)')
            if not health.get('flow', False):
                missing.append('Optical flow (PMW3901)')
            if missing:
                msg = (
                    "Cannot arm — sensors not detected:\n"
                    + "\n".join(f"  - {s}" for s in missing)
                    + "\n\nCheck wiring and power cycle the drone."
                    + "\nTo skip this check: drone.enable_sensor_check = False"
                )
                raise RuntimeError(msg)

        cf = self._cf_instance
        if not self.debug_mode:
            if self.position_hold_mode == "firmware":
                # Firmware mode: match Position.py exactly
                # DO NOT send send_setpoint — it activates Port 0x07 commander
                # which conflicts with Port 0x08 high-level commander
                cf.param.set_value("commander.enHighLevel", "1")
                time.sleep(0.1)

                if self._logger_fn:
                    self._logger_fn("Resetting position to origin...")
                cf.param.set_value("kalman.resetEstimation", "1")
                time.sleep(0.1)
                cf.param.set_value("kalman.resetEstimation", "0")
                time.sleep(1)  # Wait for position to converge
            else:
                # Library mode: unlock commander with zero setpoint
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.1)
                cf.param.set_value("commander.enHighLevel", "1")
                time.sleep(0.5)

        self._flight_active = True
        self._position_engine.reset()
        self._position_hold.reset()

        if self._logger_fn:
            self._logger_fn(f"Drone armed and ready!")

    def takeoff(self, height=None, duration=None):
        """
        Take off and hover at the specified height.

        This is a BLOCKING call — it won't return until the drone is
        hovering at the target height.

        Args:
            height: Target height in meters (default: drone.target_height).
            duration: Time in seconds for takeoff (default: drone.default_takeoff_duration).
        """
        if self._scf is None:
            raise RuntimeError("Not connected! Call connect() first.")
        if not self._flight_active:
            raise RuntimeError("Not armed! Call arm() first.")

        if height is not None:
            self.target_height = height

        dur = duration if duration is not None else self.default_takeoff_duration
        cf = self._cf_instance
        self._flight_phase = "TAKEOFF"
        if self._logger_fn:
            self._logger_fn(f"Taking off to {self.target_height}m in {dur}s...")

        # Start CSV logging if enabled
        if self.enable_csv_logging and not self._flight_logger.is_logging:
            self._flight_logger.start(logger=self._logger_fn)

        # ── FIRMWARE MODE ─────────────────────────────────────────
        if self.position_hold_mode == "firmware":
            # Use high-level commander takeoff (Port 0x08)
            # yaw=None → firmware sets useCurrentYaw=True automatically
            cf.high_level_commander.takeoff(self.target_height, dur, yaw=None)
            self._cmd_height = self.target_height

            # Wait for takeoff trajectory to complete
            t0 = time.time()
            while time.time() - t0 < dur and self._flight_active:
                self._log_csv_if_active()
                time.sleep(0.1)

            # Post-takeoff stabilisation (firmware holds position)
            self._flight_phase = "HOVERING"
            t0 = time.time()
            while time.time() - t0 < 1.0 and self._flight_active:
                self._log_csv_if_active()
                time.sleep(0.1)
            return

        # ── LIBRARY MODE (legacy Port 0x07) ───────────────────────
        takeoff_start = time.time()
        takeoff_height_start = self._sensors.height

        while (time.time() - takeoff_start < dur and
               self._flight_active):
            if not self.debug_mode:
                if (self._sensors.sensor_data_ready and
                        self._sensors.height > 0.04):
                    mvx, mvy = self._position_hold.calculate_corrections(
                        self._position_engine.x, self._position_engine.y,
                        self._position_engine.vx, self._position_engine.vy,
                        self._sensors.height, True, current_yaw=self._sensors.yaw
                    )
                else:
                    mvx, mvy = 0.0, 0.0

                total_vx = self.hover_trim_pitch + mvx
                total_vy = self.hover_trim_roll + mvy

                if self.enable_takeoff_ramp:
                    elapsed = time.time() - takeoff_start
                    progress = min(1.0, elapsed / dur)
                    cmd_height = takeoff_height_start + (
                        self.target_height - takeoff_height_start
                    ) * progress
                else:
                    cmd_height = self.target_height

                cf.commander.send_hover_setpoint(total_vx, total_vy, 0, cmd_height)
                self._cmd_height = cmd_height

            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

        # Post-takeoff safety check
        if self.enable_height_sensor_safety and not self.debug_mode:
            height_change = self._sensors.height - takeoff_height_start
            if height_change < self._height_sensor_min_change:
                cf.commander.send_setpoint(0, 0, 0, 0)
                self._flight_active = False
                raise RuntimeError(
                    f"EMERGENCY: Height sensor failure! "
                    f"Height stuck at {self._sensors.height:.3f}m"
                )

        # Stabilize at hover height
        self._flight_phase = "HOVERING"
        stab_start = time.time()
        while (time.time() - stab_start < 2.0 and self._flight_active):
            if not self.debug_mode:
                if self._sensors.sensor_data_ready:
                    mvx, mvy = self._position_hold.calculate_corrections(
                        self._position_engine.x, self._position_engine.y,
                        self._position_engine.vx, self._position_engine.vy,
                        self._sensors.height, True, current_yaw=self._sensors.yaw
                    )
                else:
                    mvx, mvy = 0.0, 0.0
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    0, self.target_height
                )
                self._cmd_height = self.target_height
            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

    def land(self, duration=None):
        """
        Land the drone safely.

        Args:
            duration: Time in seconds for landing (default: drone.default_landing_duration).
        """
        if self._logger_fn and self.debug_print_mode:
            self._logger_fn(f"[DEBUG] land() entered. scf={self._scf is not None}, flight_active={self._flight_active}")

        if self._scf is None or not self._flight_active:
            if self._logger_fn and self.debug_print_mode:
                self._logger_fn("[DEBUG] land() ABORTED - not connected or flight not active")
            self._flight_active = False
            return

        dur = duration if duration is not None else self.default_landing_duration
        cf = self._cf_instance
        self._flight_phase = "LANDING"
        if self._logger_fn:
            self._logger_fn(f"Landing over {dur}s...")

        # ── FIRMWARE MODE ─────────────────────────────────────────
        if self.position_hold_mode == "firmware":
            cf.high_level_commander.land(0.0, dur, yaw=None)

            t0 = time.time()
            while time.time() - t0 < dur + 0.5 and self._flight_active:
                self._log_csv_if_active()
                time.sleep(0.1)

            if not self.debug_mode:
                cf.high_level_commander.stop()

            self._flight_active = False
            self._flight_phase = "IDLE"
            self._flight_logger.stop(logger=self._logger_fn)
            if self._logger_fn:
                self._logger_fn("Landed!")
            return

        # ── LIBRARY MODE (legacy Port 0x07) ───────────────────────
        land_start = time.time()
        current_land_height = self.target_height
        dt = 0.02

        while (current_land_height > 0.02 and
               time.time() - land_start < dur and
               self._flight_active):
            current_land_height -= self.descent_rate * dt
            current_land_height = max(current_land_height, 0.0)
            self._cmd_height = current_land_height

            if self._sensors.sensor_data_ready and current_land_height > 0.03:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True
                )
            else:
                mvx, mvy = 0.0, 0.0

            total_vx = self.hover_trim_pitch + mvx
            total_vy = self.hover_trim_roll + mvy

            if not self.debug_mode:
                cf.commander.send_hover_setpoint(
                    total_vx, total_vy, 0, current_land_height
                )
            self._log_csv_if_active()
            time.sleep(dt)

        settle_start = time.time()
        while time.time() - settle_start < 0.3 and self._flight_active:
            if not self.debug_mode:
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch, self.hover_trim_roll, 0, 0
                )
            time.sleep(0.02)

        if not self.debug_mode:
            cf.commander.send_setpoint(0, 0, 0, 0)

        self._flight_active = False
        self._flight_phase = "IDLE"
        self._flight_logger.stop(logger=self._logger_fn)

        if self._logger_fn:
            self._logger_fn("Landed!")

    def change_height(self, delta, min_h=None, max_h=None):
        """
        Increase or decrease the hover height while in flight.

        Args:
            delta: Change in meters (positive = up, negative = down).
            min_h: Optional safety floor in meters.
            max_h: Optional safety ceiling in meters.
        """
        if delta is None:
            return

        if self._scf is None or not self._flight_active:
            if self._logger_fn:
                self._logger_fn("Cannot change height — not in flight!")
            return

        current_h = self._sensors.height
        new_h = current_h + float(delta)

        # Apply safety capping if requested
        if min_h is not None:
            new_h = max(new_h, float(min_h))
        if max_h is not None:
            new_h = min(new_h, float(max_h))

        dur = 1.0  # Transition speed
        move_delta = new_h - current_h

        # Terminal feedback (consistent with other movement blocks)
        if self._logger_fn:
            verb = "Climbing" if delta > 0 else "Descending"
            self._logger_fn(f"{verb} to {new_h:.2f}m...")

        if self._logger_fn and self.debug_print_mode:
            self._logger_fn(f"[DEBUG] height change: {delta}m -> target: {new_h:.2f}m")

        # Update the property so future hover/maneuver loops use it
        self.target_height = new_h

        if self.position_hold_mode == "firmware":
            # Move straight up/down using relative coordinates (delta Z)
            self._cf_instance.high_level_commander.go_to(
                0, 0, round(move_delta, 3), 0, dur, relative=True
            )
            # Blocking wait for the height change trajectory to complete
            time.sleep(dur + 0.2)
        else:
            # Library mode: The hover loop already uses self.target_height
            time.sleep(dur)

    def emergency_stop(self):
        """
        EMERGENCY STOP — immediately cuts all motors.

        Use only in emergencies! The drone will fall from whatever height
        it's at. For normal stops, use land().
        """
        self._flight_active = False
        self._manual_active = False
        if self._scf and not self.debug_mode:
            try:
                self._cf_instance.commander.send_setpoint(0, 0, 0, 0)
            except Exception:
                pass
        # NOTE: Do NOT clear LEDs here — preserves error notification state

    # === Raw Control (no sensors needed) ===

    def send_control(self, roll=0.0, pitch=0.0, yawrate=0.0, thrust=0):
        """
        Send raw flight commands directly to motors.

        No sensors required! This bypasses height/position hold entirely.
        YOU control throttle, tilt, and rotation manually.

        Args:
            roll:    Tilt left/right in degrees (-30 to +30).
            pitch:   Tilt forward/back in degrees (-30 to +30).
            yawrate: Spin rate in degrees/sec (-200 to +200).
            thrust:  Motor power (0 to 65535). ~20000 = hover for light drone.

        Warning:
            Too much thrust will flip the drone! Start low (~15000) and
            increase slowly. Use emergency_stop() or Ctrl+C if needed.
        """
        if self._scf is None:
            raise RuntimeError("Not connected! Call connect() first.")

        # Apply trim corrections
        roll = float(roll) + self.raw_trim_roll
        pitch = float(pitch) + self.raw_trim_pitch

        # Clamp for safety
        thrust = max(0, min(int(thrust), self.max_thrust))
        roll = max(-30, min(30, roll))
        pitch = max(-30, min(30, pitch))
        yawrate = max(-200, min(200, float(yawrate)))

        if not self.debug_mode:
            self._cf_instance.commander.send_setpoint(
                roll, pitch, yawrate, thrust
            )

    def hover(self, seconds):
        """
        Hover in place for the specified duration.

        In firmware mode, the high-level commander holds position autonomously.
        In library mode, actively streams hold setpoints.

        Args:
            seconds: How long to hover in seconds.
        """
        if self._scf is None or not self._flight_active:
            time.sleep(seconds)
            return

        cf = self._cf_instance
        start = time.time()
        _low_bat_warned = False

        while (time.time() - start < seconds and self._flight_active):
            # Battery safety check (applies to both modes)
            voltage = self._sensors.battery_voltage
            if voltage > 0:
                if voltage < defaults.CRITICAL_BATTERY_THRESHOLD:
                    if self._logger_fn:
                        self._logger_fn(
                            f"CRITICAL: Battery {voltage:.2f}V! Auto-landing!"
                        )
                    try:
                        self._leds.set_color(255, 0, 0)
                    except Exception:
                        pass
                    break
                elif (voltage < defaults.LOW_BATTERY_THRESHOLD
                      and not _low_bat_warned):
                    _low_bat_warned = True
                    if self._logger_fn:
                        self._logger_fn(
                            f"WARNING: Low battery ({voltage:.2f}V)! "
                            f"Land soon and charge/replace battery."
                        )
                    try:
                        self._leds.set_color(255, 0, 0)
                        self._leds.blink(200, 200)
                    except Exception:
                        pass

            if self.position_hold_mode == "firmware":
                # Firmware holds position autonomously — no commands needed
                pass
            else:
                # Library mode: stream hover setpoints via Port 0x07
                if not self.debug_mode:
                    if self._sensors.sensor_data_ready:
                        mvx, mvy = self._position_hold.calculate_corrections(
                            self._position_engine.x, self._position_engine.y,
                            self._position_engine.vx, self._position_engine.vy,
                            self._sensors.height, True, current_yaw=self._sensors.yaw
                        )
                    else:
                        mvx, mvy = 0.0, 0.0
                    cf.commander.send_hover_setpoint(
                        self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                        0, self.target_height
                    )
                    self._cmd_height = self.target_height

            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

    def _log_csv_if_active(self):
        """Log a CSV row if flight logging is active."""
        if self._flight_logger.is_logging:
            corr_vx = self._position_hold.correction_vx
            corr_vy = self._position_hold.correction_vy
            self._flight_logger.log_row(
                self._position_engine.x, self._position_engine.y,
                self._sensors.height, self._sensors.range_height,
                self._position_engine.vx, self._position_engine.vy,
                correction_vx=corr_vx,
                correction_vy=corr_vy,
                battery=self._sensors.battery_voltage,
                roll=self._sensors.roll,
                pitch=self._sensors.pitch,
                yaw=self._sensors.yaw,
                gyro_x=self._sensors.gyro_x,
                gyro_y=self._sensors.gyro_y,
                gyro_z=self._sensors.gyro_z,
                flight_phase=self._flight_phase,
                target_height=self._cmd_height,
                cmd_vx=self.hover_trim_pitch + corr_vx,
                cmd_vy=self.hover_trim_roll + corr_vy,
                cmd_thrust=self._sensors.thrust,
            )

    # === Movement Commands (Tier 1) ===

    def pitch_forward(self, distance=None, speed=0.2):
        """
        Pitch forward by the specified distance.

        Args:
            distance: Distance in meters (default: drone.maneuver_distance).
            speed: Maximum velocity setpoint (m/s).
        """
        if distance is None:
            distance = self.maneuver_distance
        self._execute_movement(distance, 0.0, speed)

    def pitch_backward(self, distance=None, speed=0.2):
        """
        Pitch backward by the specified distance.

        Args:
            distance: Distance in meters (default: drone.maneuver_distance).
            speed: Maximum velocity setpoint (m/s).
        """
        if distance is None:
            distance = self.maneuver_distance
        self._execute_movement(-distance, 0.0, speed)

    def roll_left(self, distance=None, speed=0.2):
        """
        Roll left by the specified distance.

        Args:
            distance: Distance in meters (default: drone.maneuver_distance).
            speed: Maximum velocity setpoint (m/s).
        """
        if distance is None:
            distance = self.maneuver_distance
        self._execute_movement(0.0, distance, speed)

    def roll_right(self, distance=None, speed=0.2):
        """
        Roll right by the specified distance.

        Args:
            distance: Distance in meters (default: drone.maneuver_distance).
            speed: Maximum velocity setpoint (m/s).
        """
        if distance is None:
            distance = self.maneuver_distance
        self._execute_movement(0.0, -distance, speed)

    def rotate_left(self, degrees=90, speed=90):
        """
        Rotate left (counter-clockwise) by the specified angle.

        Args:
            degrees: Degrees to rotate (default: 90).
            speed: Rotation speed in degrees per second (default: 90).
        """
        self._execute_rotation(degrees, speed)

    def rotate_right(self, degrees=90, speed=90):
        """
        Rotate right (clockwise) by the specified angle.

        Args:
            degrees: Degrees to rotate (default: 90).
            speed: Rotation speed in degrees per second (default: 90).
        """
        self._execute_rotation(-degrees, speed)

    def _execute_rotation(self, diff_deg, speed):
        """
        Execute a relative rotation (yaw change).
        """
        if self._scf is None or not self._flight_active:
            if self._logger_fn:
                self._logger_fn("Cannot rotate — not in flight!")
            return

        cf = self._cf_instance
        target_yaw = self._sensors.yaw + diff_deg

        if self.position_hold_mode == "firmware":
            dur = max(abs(diff_deg) / speed, 0.5)
            yaw_rad = math.radians(diff_deg)
            if self._logger_fn:
                self._logger_fn(f"Rotating by {diff_deg}° in {dur:.2f}s")
            
            # Use relative go_to with zero translation to rotate in place
            cf.high_level_commander.go_to(0, 0, 0, round(yaw_rad, 4), round(dur, 2), relative=True)
            
            t0 = time.time()
            while time.time() - t0 < dur + 0.2 and self._flight_active:
                self._log_csv_if_active()
                time.sleep(0.1)
            return

        # Library mode: active yawrate streaming with position hold
        if self._logger_fn:
            self._logger_fn(f"Rotating to target yaw: {target_yaw:.1f}°")

        kp_yaw = 2.0
        start_time = time.time()
        timeout = max(abs(diff_deg) / 10.0 + 2.0, 5.0)

        while self._flight_active and (time.time() - start_time < timeout):
            current_yaw = self._sensors.yaw
            error = target_yaw - current_yaw
            # Normalize error to [-180, 180]
            while error > 180: error -= 360
            while error < -180: error += 360

            if abs(error) < 2.0: # threshold
                break

            yawrate = error * kp_yaw
            yawrate = max(-speed, min(speed, yawrate))

            if not self.debug_mode and self._sensors.sensor_data_ready:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True, current_yaw=current_yaw
                )
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    round(yawrate, 2), self.target_height
                )
            
            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

        # Stabilize
        stab_start = time.time()
        while time.time() - stab_start < 0.5 and self._flight_active:
            if not self.debug_mode and self._sensors.sensor_data_ready:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True
                )
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    0, self.target_height
                )
            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

    def _execute_movement(self, dx, dy, speed):
        """
        Execute a relative movement.

        In firmware mode, uses high_level_commander go_to (relative).
        In library mode, uses PID + send_hover_setpoint.
        """
        if self._scf is None or not self._flight_active:
            if self._logger_fn:
                self._logger_fn("Cannot move — not in flight!")
            return

        cf = self._cf_instance

        if self.position_hold_mode == "firmware":
            distance = math.sqrt(dx**2 + dy**2)
            spd = min(speed, self.max_flight_speed)
            dur = max(distance / spd, 0.5)

            if self._logger_fn:
                self._logger_fn(
                    f"Moving by ({dx:.2f}, {dy:.2f}) in {dur:.1f}s"
                )

            cf.high_level_commander.go_to(round(dx, 2), round(dy, 2), 0, 0, round(dur, 2), relative=True)
            t0 = time.time()
            while time.time() - t0 < dur + 0.5 and self._flight_active:
                self._log_csv_if_active()
                time.sleep(0.1)
            return

        # Library mode
        target_x = self._position_engine.x + dx
        target_y = self._position_engine.y + dy
        self._position_hold.set_target(target_x, target_y)

        if self._logger_fn:
            self._logger_fn(
                f"Moving to ({target_x:.2f}, {target_y:.2f}) "
                f"at {speed:.1f} m/s"
            )

        move_max_correction = min(speed, self.max_correction)

        start = time.time()
        while (time.time() - start < self.waypoint_timeout and
               self._flight_active):
            dist = ((self._position_engine.x - target_x) ** 2 +
                    (self._position_engine.y - target_y) ** 2) ** 0.5
            if dist < self.waypoint_threshold:
                break

            if not self.debug_mode and self._sensors.sensor_data_ready:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True,
                    max_correction=move_max_correction,
                )
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    0, self.target_height
                )

            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

        # Stabilize at target
        stab_start = time.time()
        while (time.time() - stab_start < self.waypoint_stabilization_time and
               self._flight_active):
            if not self.debug_mode and self._sensors.sensor_data_ready:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True,
                )
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    0, self.target_height
                )
            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

    # === Position Hold Control (Tier 3) ===

    def enable_position_hold(self):
        """Activate optical flow-based position hold."""
        self._position_hold.enabled = True
        if self._logger_fn:
            self._logger_fn("Position hold: ENABLED")

    def disable_position_hold(self):
        """Disable position hold — switch to raw velocity mode."""
        self._position_hold.enabled = False
        if self._logger_fn:
            self._logger_fn("Position hold: DISABLED (raw velocity mode)")

    def reset_position(self):
        """
        Reset the estimated position to (0, 0).

        Useful when drift accumulates or you want to establish a new
        "home" position.
        """
        self._position_engine.reset()
        self._position_hold.reset()
        if self._logger_fn:
            self._logger_fn("Position reset to origin (0, 0)")

    # === Advanced Flight (Tier 3) ===

    def fly_to(self, x, y, z=None, yaw=None, speed=None, threshold=None):
        """
        Fly to an absolute position.

        In firmware mode, uses high_level_commander go_to.
        In library mode, uses PID + send_hover_setpoint.

        Args:
            x: Target X position in meters.
            y: Target Y position in meters.
            z: Target Z (height) in meters (default: target_height).
            yaw: Target yaw in degrees.
            speed: Flight speed in m/s (default: default_flight_speed).
            threshold: How close is "close enough" (meters).
        """
        if self._scf is None or not self._flight_active:
            if self._logger_fn:
                self._logger_fn("Cannot fly_to — not in flight!")
            return
        if threshold is None:
            threshold = self.waypoint_threshold
        if z is None:
            z = self.target_height
        if speed is None:
            speed = self.default_flight_speed

        cf = self._cf_instance

        if self.position_hold_mode == "firmware":
            # Get current position from EKF
            cur_x = self._sensors.kalman_x
            cur_y = self._sensors.kalman_y
            cur_z = self._sensors.kalman_z
            cur_yaw = self._sensors.yaw  # in degrees

            dx = x - cur_x
            dy = y - cur_y
            dz = z - cur_z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            spd = min(speed, self.max_flight_speed)
            lin_dur = distance / spd

            if yaw is not None:
                target_yaw_deg = yaw
                yaw_str = f", yaw={yaw}°"
            else:
                target_yaw_deg = 0.0
                yaw_str = ""

            # Calculate yaw duration (assuming 90 deg/sec comfortable rotation)
            yaw_diff = abs(target_yaw_deg - cur_yaw) % 360
            shortest_yaw_diff = min(yaw_diff, 360 - yaw_diff)
            yaw_dur = shortest_yaw_diff / 90.0
            
            target_yaw_rad = math.radians(target_yaw_deg)

            dur = max(lin_dur, yaw_dur, 0.5)

            if self._logger_fn:
                self._logger_fn(
                    f"Flying to ({x:.2f}, {y:.2f}, {z:.2f}{yaw_str}) in {dur:.1f}s"
                )

            cf.high_level_commander.go_to(round(x, 2), round(y, 2), round(z, 2), round(target_yaw_rad, 4), round(dur, 2), relative=False)

            t0 = time.time()
            while time.time() - t0 < dur + 0.5 and self._flight_active:
                self._log_csv_if_active()
                time.sleep(0.1)
            return

        # Library mode
        self._position_hold.set_target(x, y)

        if self._logger_fn:
            self._logger_fn(
                f"Flying to ({x:.2f}, {y:.2f}) at {speed:.1f} m/s"
            )

        move_max_correction = min(speed, self.max_correction)

        start = time.time()
        while (time.time() - start < self.waypoint_timeout and
               self._flight_active):
            dist = ((self._position_engine.x - x) ** 2 +
                    (self._position_engine.y - y) ** 2) ** 0.5
            if dist < threshold:
                break

            if not self.debug_mode and self._sensors.sensor_data_ready:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True,
                    max_correction=move_max_correction,
                )
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    0, self.target_height
                )

            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

        # Stabilize at target
        stab_start = time.time()
        while (time.time() - stab_start < self.waypoint_stabilization_time and
               self._flight_active):
            if not self.debug_mode and self._sensors.sensor_data_ready:
                mvx, mvy = self._position_hold.calculate_corrections(
                    self._position_engine.x, self._position_engine.y,
                    self._position_engine.vx, self._position_engine.vy,
                    self._sensors.height, True,
                )
                cf.commander.send_hover_setpoint(
                    self.hover_trim_pitch + mvx, self.hover_trim_roll + mvy,
                    0, self.target_height
                )
            self._log_csv_if_active()
            time.sleep(self.control_update_rate)

    def fly_path(self, waypoints, speed=None, threshold=None):
        """
        Fly through a sequence of waypoints.

        Accepts flexible tuples: (x,y), (x,y,z), or (x,y,z,yaw).

        Args:
            waypoints: List of tuples.
            speed: Flight speed in m/s.
            threshold: How close is "close enough" (meters).
        """
        if threshold is None:
            threshold = self.waypoint_threshold
        if speed is None:
            speed = self.default_flight_speed

        for i, wp in enumerate(waypoints):
            if not self._flight_active:
                break
            x, y = wp[0], wp[1]
            z = wp[2] if len(wp) > 2 else None
            yaw = wp[3] if len(wp) > 3 else None
            if self._logger_fn:
                self._logger_fn(f"Waypoint {i + 1}/{len(waypoints)}")
            self.fly_to(x, y, z=z, yaw=yaw, speed=speed, threshold=threshold)

    # === Shape Paths (Tier 3) ===

    def _fly_shape_path(self, offsets, duration, face_direction):
        """
        Internal helper — fly through a list of (x, y) offsets from the
        drone's current position, then return to the starting point.

        Args:
            offsets:        List of (x, y) tuples relative to start position.
            duration:       Total time budget for the whole shape (seconds).
            face_direction: If True, rotate drone nose toward the next waypoint
                            before/during each leg. If False, keep current yaw.
        """
        import math

        if self._scf is None or not self._flight_active:
            if self._logger_fn:
                self._logger_fn("Cannot fly shape — not in flight!")
            return

        cf = self._cf_instance

        # ── Reset EKF so current hover position becomes origin (0,0) ──
        if not self.debug_mode and self.position_hold_mode == "firmware":
            if self._logger_fn:
                self._logger_fn("Resetting — current position becomes origin...")
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(0.2)  # wait for EKF to re-converge

            # Set yaw to 0° (face forward) before starting shape
            if self._logger_fn:
                self._logger_fn("Setting yaw=0° (face forward) before shape...")
            cf.high_level_commander.go_to(0, 0, self.target_height, 0, 1.0, relative=False)
            time.sleep(1.5)

        # After EKF reset, origin is always (0, 0)
        origin_x = 0.0
        origin_y = 0.0

        # Build absolute waypoint list: all offsets + return-to-origin
        abs_waypoints = [(origin_x + dx, origin_y + dy) for dx, dy in offsets]
        abs_waypoints.append((origin_x, origin_y))  # final: return to start

        # Speed calculation — spread the time budget across all legs evenly
        # fly_to() in firmware mode waits (dur + 0.5s) per call,
        # so we subtract that overhead to hit the requested total duration.
        n_legs = len(abs_waypoints)
        FLY_TO_OVERHEAD = 0.5  # seconds of stabilization in fly_to()
        time_per_leg = duration / n_legs
        effective_fly_time = max(time_per_leg - FLY_TO_OVERHEAD, 0.3)

        # Iterate through waypoints
        prev_x, prev_y = origin_x, origin_y
        for i, (tx, ty) in enumerate(abs_waypoints):
            if not self._flight_active:
                break

            dx = tx - prev_x
            dy = ty - prev_y
            seg_len = math.sqrt(dx * dx + dy * dy)

            # Yaw toward the next point's direction (normalized to 0°–360°)
            if face_direction and seg_len > 0.001:
                yaw_deg = math.degrees(math.atan2(dy, dx)) % 360.0
            else:
                yaw_deg = None

            # Speed = distance / effective flight time (excluding overhead)
            speed = seg_len / effective_fly_time if effective_fly_time > 0 else self.default_flight_speed
            speed = max(0.05, min(speed, self.max_flight_speed))

            if self._logger_fn:
                yaw_str = f", yaw={yaw_deg:.0f}°" if yaw_deg is not None else ""
                self._logger_fn(
                    f"  Shape leg {i + 1}/{n_legs}: "
                    f"({tx:.2f}, {ty:.2f}){yaw_str} at {speed:.2f} m/s"
                )

            self.fly_to(tx, ty, yaw=yaw_deg, speed=speed)

            prev_x, prev_y = tx, ty

    def square(self, length, duration, face_direction=True):
        """
        Fly a square path starting from the current position.

        Path: origin → forward → forward-left → left → back to origin.
        Each side is exactly `length` meters. 4 legs total.

        Args:
            length:         Side length of the square in meters.
            duration:       Total time to complete the full square (seconds).
            face_direction: If True, drone nose faces the direction of travel
                            at each corner turn. If False, heading is unchanged.

        Example:
            drone.square(0.6, 10)           # 0.6m square, 10 seconds
            drone.square(0.8, 12, False)    # 0.8m square, fixed heading
        """
        L = length
        # Forward → Left → Back → Origin (4 equal-length sides)
        #   (0,0) → (L,0) → (L,L) → (0,L) → (0,0)
        offsets = [
            (L, 0.0),      # forward
            (L, L),         # forward-left corner
            (0.0, L),       # left
        ]                   # return to (0,0) is added by _fly_shape_path
        if self._logger_fn:
            self._logger_fn(
                f"Square: side={length}m, duration={duration}s, "
                f"face_direction={face_direction}"
            )
        self._fly_shape_path(offsets, duration, face_direction)

    def triangle(self, length, duration, face_direction=True):
        """
        Fly an equilateral triangle path starting from the current position.

        Path: origin → forward → forward-left → back to origin.
        Each side is exactly `length` meters. 3 legs total.

        Args:
            length:         Side length of the triangle in meters.
            duration:       Total time to complete the full triangle (seconds).
            face_direction: If True, drone nose faces the direction of travel.
                            If False, heading is unchanged.

        Example:
            drone.triangle(0.6, 8)          # 0.6m triangle, 8 seconds
            drone.triangle(0.5, 10, False)  # fixed heading
        """
        import math
        L = length
        # Equilateral triangle: first side along +X (forward), then 120° turn
        #   (0,0) → (L,0) → (L/2, L*√3/2) → (0,0)
        # offsets = [
        #     (L, 0.0),                                       # forward
        #     (L / 2.0, L * math.sqrt(3.0) / 2.0),            # forward-left
        # ]  # return to (0,0) added by _fly_shape_path

        # Left hand triangle
        offsets = [
            (L, 0.0),                                       # forward
            (0.0, L),                                       # left-back
        ]  # return to (0,0) added by _fly_shape_path
        if self._logger_fn:
            self._logger_fn(
                f"Triangle: side={length}m, duration={duration}s, "
                f"face_direction={face_direction}"
            )
        self._fly_shape_path(offsets, duration, face_direction)

    def circle(self, diameter, duration, face_direction=True):
        """
        Fly a smooth circular path centered on the current position.

        The number of waypoints is automatically calculated so the arc
        spacing is ≤ 0.05 m, giving a smooth circle regardless of size.

        Unlike polygon shapes, circle sends go_to commands directly
        (not through fly_to) for continuous smooth motion without
        stabilization pauses between waypoints.

        Args:
            diameter:       Diameter of the circle in meters.
            duration:       Total time to complete one full loop (seconds).
            face_direction: If True, drone nose faces the tangent direction
                            (always pointing the way it's flying).
                            If False, heading is unchanged.

        Example:
            drone.circle(1.0, 15)           # 1m diameter circle, 15 seconds
            drone.circle(0.5, 8, False)     # no yaw rotation
        """
        import math
        radius = diameter / 2.0
        circumference = math.pi * diameter

        # Dynamic waypoint count for smooth arc (≤ 0.05 m per step, min 12)
        n = max(12, int(math.ceil(circumference / 0.05)))

        if self._logger_fn:
            self._logger_fn(
                f"Circle: diameter={diameter}m, duration={duration}s, "
                f"n_waypoints={n}, face_direction={face_direction}"
            )

        if self._scf is None or not self._flight_active:
            if self._logger_fn:
                self._logger_fn("Cannot fly circle — not in flight!")
            return

        cf = self._cf_instance

        # ── Reset EKF so current hover position becomes origin (0,0) ──
        if not self.debug_mode and self.position_hold_mode == "firmware":
            if self._logger_fn:
                self._logger_fn("Resetting — current position becomes origin...")
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(0.2)

        # ── Smooth lead-in: origin → circle edge → rotate to tangent ──
        # Step 1: Face forward at origin (yaw=0°)
        # Step 2: Fly straight out to (radius, 0) — the circle start point
        # Step 3: Rotate yaw to 90° (tangent direction at angle=0)
        # This prevents the jerk from jumping directly into circular motion.
        if not self.debug_mode:
            if self._logger_fn:
                self._logger_fn("Lead-in: moving to circle edge...")
            # Face forward at origin
            cf.high_level_commander.go_to(0, 0, self.target_height, 0, 1.0, relative=False)
            time.sleep(1.0)
            # Fly to circle edge (radius, 0)
            cf.high_level_commander.go_to(radius, 0, self.target_height, 0, 2.0, relative=False)
            time.sleep(2.5)
            # Rotate to tangent direction (90° = π/2 rad) at circle start
            if face_direction:
                if self._logger_fn:
                    self._logger_fn("Lead-in: rotating to tangent yaw (90°)...")
                cf.high_level_commander.go_to(radius, 0, self.target_height, round(math.pi / 2.0, 2), 1.5, relative=False)
                time.sleep(2.0)

        # ── Trace the circle by sending go_to directly (like Position.py) ──
        # Each waypoint gets an equal time slice — no stabilization pauses,
        # so the drone moves continuously along the arc.
        time_per_wp = duration / n
        height = self.target_height

        for i in range(1, n + 1):
            if not self._flight_active:
                break

            angle = (2.0 * math.pi * i) / n
            x = round(radius * math.cos(angle), 2)
            y = round(radius * math.sin(angle), 2)

            # Tangent direction: angle + 90° (normalized to 0°–360°)
            if face_direction:
                yaw_rad = round(angle + math.pi / 2.0, 2)
            else:
                yaw_rad = 0.0

            if self._logger_fn:
                yaw_deg = math.degrees(yaw_rad) % 360.0
                self._logger_fn(
                    f"  Circle wp {i}/{n}: ({x}, {y}), "
                    f"yaw={yaw_deg:.0f}°, dur={time_per_wp:.2f}s"
                )

            if not self.debug_mode:
                cf.high_level_commander.go_to(
                    x, y, height, yaw_rad,
                    round(time_per_wp, 2), relative=False
                )

            self._log_csv_if_active()
            time.sleep(time_per_wp)

        if self._logger_fn:
            self._logger_fn("Circle complete!")

    def pentagon(self, length, duration, face_direction=True):
        """
        Fly a regular pentagon path starting from the current position.

        The drone walks along 5 equal-length sides and returns to origin.
        Each side is exactly `length` meters. 5 legs total.

        Args:
            length:         Side length of the pentagon in meters.
            duration:       Total time to complete the full pentagon (seconds).
            face_direction: If True, drone nose faces the direction of travel.
                            If False, heading is unchanged.

        Example:
            drone.pentagon(0.6, 12)         # 0.6m pentagon, 12 seconds
            drone.pentagon(0.8, 15, False)  # fixed heading
        """
        import math
        L = length
        n_sides = 5
        ext_angle = 2.0 * math.pi / n_sides  # 72° exterior angle

        # Walk the edges using turtle-graphics:
        # Start at origin heading 0° (+X), turn left 72° at each vertex
        heading = 0.0
        cx, cy = 0.0, 0.0
        offsets = []
        for _ in range(n_sides - 1):  # 4 intermediate vertices
            cx += L * math.cos(heading)
            cy += L * math.sin(heading)
            offsets.append((round(cx, 4), round(cy, 4)))
            heading += ext_angle
        # The 5th side (back to origin) is added automatically by _fly_shape_path

        if self._logger_fn:
            self._logger_fn(
                f"Pentagon: side={length}m, duration={duration}s, "
                f"face_direction={face_direction}"
            )
        self._fly_shape_path(offsets, duration, face_direction)

    # === Complete Flight Execution ===

    def fly(self, maneuver_fn=None, hover_duration=None):
        """
        Execute a complete flight: connect → takeoff → hover/maneuver → land.

        This is the main entry point for scripted flights. It handles
        connection, takeoff, your maneuver code, and safe landing.

        Args:
            maneuver_fn: Optional function(drone, cf, has_pos_hold) to execute
                         during the hover phase. If None, hovers for
                         hover_duration seconds.
            hover_duration: How long to hover in seconds (default: drone.hover_duration).
        """
        if hover_duration is not None:
            self._hover_duration = hover_duration
        _flight_engine.run_flight_sequence(self, maneuver_fn)

    # === Manual Control (Tier 2–3) ===

    def start_manual_control(self):
        """
        Start joystick/keyboard control mode.

        Takes off and enters a real-time control loop where the drone
        responds to keyboard inputs (WASD) while maintaining position hold.

        Hold mode determines behavior when keys are released:
            "current" — hold at current position (Flying Mode)
            "origin"  — snap back to launch point (Spring Mode)

        Call stop_manual_control() or press emergency stop to end.
        """
        if self._manual_active:
            if self._logger_fn:
                self._logger_fn("Manual control already active!")
            return

        self._manual_active = True
        self._manual_thread = threading.Thread(
            target=_manual_control.run_manual_control,
            args=(self,),
            daemon=True,
        )
        self._manual_thread.start()

    def stop_manual_control(self):
        """Stop manual control mode and land the drone."""
        self._manual_active = False
        if self._manual_thread:
            self._manual_thread.join(timeout=10)
            self._manual_thread = None

    def set_key(self, key, pressed):
        """
        Set the state of a manual control key and trigger callbacks.

        Args:
            key: Key name ("w", "a", "s", "d").
            pressed: True if pressed, False if released.
        """
        if key in self._manual_keys:
            old_state = self._manual_keys[key]
            self._manual_keys[key] = pressed
            
            # Trigger callbacks only on state change to avoid repeats
            if pressed and not old_state and self._on_key_press_cb:
                self._on_key_press_cb(key)
            elif not pressed and old_state and self._on_key_release_cb:
                self._on_key_release_cb(key)

    def on_key_press(self, callback):
        """Register a callback for key press events."""
        self._on_key_press_cb = callback

    def on_key_release(self, callback):
        """Register a callback for key release events."""
        self._on_key_release_cb = callback

    # === Firmware Parameters (Tier 3) ===

    def apply_firmware_params(self):
        """
        Send current firmware parameters to the drone's onboard controller.

        Must be called while connected. Parameters include:
            drone.thrust_base    — base motor thrust
            drone.z_position_kp  — height position gain
            drone.z_velocity_kp  — vertical velocity damping
        """
        if self._scf:
            from ._connection import apply_firmware_parameters
            apply_firmware_parameters(
                self._scf.cf,
                self.thrust_base, self.z_position_kp, self.z_velocity_kp,
                logger=self._logger_fn,
            )
        else:
            if self._logger_fn:
                self._logger_fn("Cannot apply firmware params: not connected")

    # === LED Control ===

    def set_led_color(self, r, g, b):
        """
        Set all LEDs to a solid RGB color.

        Args:
            r: Red (0–255).
            g: Green (0–255).
            b: Blue (0–255).
        """
        self._leds.set_color(r, g, b, logger=self._logger_fn)

    def set_led(self, index, r, g, b):
        """
        Set a single LED to a specific RGB color.

        The drone has 4 LEDs numbered 0–3.

        Args:
            index: LED number (0–3).
            r: Red (0–255).
            g: Green (0–255).
            b: Blue (0–255).

        Example:
            drone.set_led(0, 255, 0, 0)  # LED 0 = red
            drone.set_led(1, 0, 255, 0)  # LED 1 = green
        """
        self._leds.set_pixel(index, r, g, b, logger=self._logger_fn)

    def blink_leds(self, on_ms=500, off_ms=500):
        """
        Start blinking the LEDs.

        Args:
            on_ms: Duration LEDs stay ON (milliseconds).
            off_ms: Duration LEDs stay OFF (milliseconds).
        """
        self._leds.blink(on_ms, off_ms, logger=self._logger_fn)

    def clear_leds(self):
        """Turn off all LEDs."""
        self._leds.clear(logger=self._logger_fn)

    # === Data Logging ===

    def start_logging(self, filename=None):
        """
        Start recording flight data to a CSV file.

        Args:
            filename: Optional custom filename.
        """
        self._flight_logger.start(filename, logger=self._logger_fn)

    def stop_logging(self):
        """Stop recording and close the CSV file."""
        self._flight_logger.stop(logger=self._logger_fn)

    # === Live Plots (Non-Blocking) ===

    def start_dashboard(self, max_points=200, update_ms=100):
        """
        Start a full sensor dashboard in the background (non-blocking).

        The plot runs in a separate process — your script keeps running!

        Returns:
            BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.

        Example::

            plot = drone.start_dashboard()
            drone.arm()
            drone.takeoff()
            drone.hover(5)
            drone.land()
            plot.stop()
        """
        from .gui import start_live_dashboard
        self._background_plot = start_live_dashboard(self, max_points, update_ms)
        return self._background_plot

    def start_height_plot(self, max_points=200, update_ms=100):
        """
        Start a live height plot in the background (non-blocking).

        Returns:
            BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.
        """
        from .gui import start_live_height_plot
        self._background_plot = start_live_height_plot(self, max_points, update_ms)
        return self._background_plot

    def start_imu_plot(self, max_points=200, update_ms=100):
        """
        Start a live IMU plot in the background (non-blocking).

        Returns:
            BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.
        """
        from .gui import start_live_imu_plot
        self._background_plot = start_live_imu_plot(self, max_points, update_ms)
        return self._background_plot

    def start_position_plot(self, max_points=500, update_ms=100):
        """
        Start a live XY position trail plot in the background (non-blocking).

        Returns:
            BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.
        """
        from .gui import start_live_position_plot
        self._background_plot = start_live_position_plot(self, max_points, update_ms)
        return self._background_plot

    def stop_plot(self):
        """Stop the currently running background plot, if any."""
        plot = getattr(self, '_background_plot', None)
        if plot is not None:
            plot.stop()
            self._background_plot = None

    # === Internal Callbacks (hidden from learners) ===

    def _motion_callback(self, timestamp, data, logconf):
        """Internal: called when new motion sensor data arrives."""
        self._sensors.last_sensor_heartbeat = time.time()
        self._sensors.sensor_data_ready = True

        # Extract sensor values
        delta_x = data.get("motion.deltaX", 0)
        delta_y = data.get("motion.deltaY", 0)
        z_estimate = data.get("stateEstimate.z", self._sensors.height)
        z_range = data.get("range.zrange", 0)

        # Update height
        self._sensors.height = z_estimate
        self._sensors.range_height = z_range / 1000.0  # mm → meters

        # Use range if available and reasonable, otherwise state estimate
        altitude_for_calc = z_estimate
        if z_range > 0:
            altitude_for_calc = z_range / 1000.0

        # Store raw firmware values BEFORE any axis remapping (for diagnostics)
        self._sensors.raw_delta_x = delta_x   # literal motion.deltaX from firmware
        self._sensors.raw_delta_y = delta_y   # literal motion.deltaY from firmware

        # Extract EKF position estimates (firmware mode uses these)
        self._sensors.kalman_x = data.get("kalman.stateX", self._sensors.kalman_x)
        self._sensors.kalman_y = data.get("kalman.stateY", self._sensors.kalman_y)
        self._sensors.kalman_z = data.get("kalman.stateZ", self._sensors.kalman_z)

        # Swap axes: sensor deltaY → drone forward (X), sensor deltaX → drone lateral (Y)
        # This aligns the optical flow frame with the drone's body frame at the source,
        # so all downstream values (velocity, position, corrections) are in body frame.
        self._position_engine.update_from_sensor(delta_y, delta_x, altitude_for_calc, yaw=self._sensors.yaw)

    def _battery_callback(self, timestamp, data, logconf):
        """Internal: called when new battery data arrives."""
        voltage = data.get("pm.vbat", 0.0)
        if voltage > 0:
            self._sensors.battery_voltage = voltage
            self._sensors.battery_data_ready = True

    def _imu_callback(self, timestamp, data, logconf):
        """Internal: called when new IMU data arrives."""
        self._sensors.roll = data.get("stabilizer.roll", self._sensors.roll)
        self._sensors.pitch = data.get("stabilizer.pitch", self._sensors.pitch)
        self._sensors.yaw = data.get("stabilizer.yaw", self._sensors.yaw)
        self._sensors.gyro_x = data.get("gyro.x", self._sensors.gyro_x)
        self._sensors.gyro_y = data.get("gyro.y", self._sensors.gyro_y)
        self._sensors.gyro_z = data.get("gyro.z", self._sensors.gyro_z)
        self._sensors.thrust = data.get("stabilizer.thrust", self._sensors.thrust)

    def set_logger(self, fn):
        """
        Set a custom output function for log messages.

        Args:
            fn: A callable that takes a string, e.g. print or your_logger.
        """
        self._logger_fn = fn
