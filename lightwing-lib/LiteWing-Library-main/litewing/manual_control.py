"""
LiteWing Manual Control
========================
Author: Dharagesh

Keyboard/joystick-based real-time drone control with position hold.

There are two hold modes:
    "current"  — Hold at Current Position (Flying Mode)
                 When keys are released, the drone holds at whatever
                 position it reached. Like driving a car.

    "origin"   — Hold at Origin (Spring Mode)
                 When keys are released, the drone snaps back to launch.
                 Like a rubber band attached to the starting point.
"""

import time
import math
from .config import defaults
from ._safety import check_link_safety
from ._connection import (
    setup_sensor_logging, apply_firmware_parameters, stop_logging_configs
)


def run_manual_control(drone):
    """
    Execute manual (joystick) control flight.

    Assumes the drone is already connected via drone.connect().
    Takes off, then enters a control loop where the drone responds
    to key inputs while maintaining position hold.
    """
    if not drone.is_connected:
        raise RuntimeError("Drone not connected! Call drone.connect() before starting manual control.")

    # Arm the drone if not already active (handles commander/highlevel init)
    if not drone._flight_active:
        drone.arm()

    cf = drone._cf_instance
    has_pos_hold = getattr(drone, '_log_motion', None) is not None

    try:
        # Initial comandante/param check (already done in arm(), but ensure here for safety)
        if not drone.debug_mode:
            if drone.position_hold_mode != "firmware":
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.1)

        # === TAKEOFF ===
        drone._flight_phase = "MANUAL_TAKEOFF"
        if drone._logger_fn:
            drone._logger_fn(f"Manual control: taking off to {drone.target_height}m...")

        if drone.enable_csv_logging and not drone._flight_logger.is_logging:
            drone._flight_logger.start(logger=drone._logger_fn)

        takeoff_start = time.time()
        takeoff_height_start = drone._sensors.height

        while (time.time() - takeoff_start < drone.default_takeoff_duration and
               drone._manual_active):
            if not check_link_safety(cf, drone._sensors.sensor_data_ready and drone.position_hold_mode != "firmware",
                                    drone._sensors.last_sensor_heartbeat,
                                    drone.debug_mode, drone._logger_fn):
                drone._manual_active = False
                break
            if not drone.debug_mode:
                if drone.enable_takeoff_ramp:
                    # Smooth height ramp
                    elapsed = time.time() - takeoff_start
                    progress = min(1.0, elapsed / drone.default_takeoff_duration)
                    cmd_height = round(takeoff_height_start + (
                        drone.target_height - takeoff_height_start
                    ) * progress, 2)
                else:
                    # No ramp — go straight to target height
                    cmd_height = drone.target_height

                if drone.position_hold_mode == "firmware" and drone.commander_mode == "position":
                    # Position mode: lock XY at origin while ramping Z
                    cf.commander.send_position_setpoint(0.0, 0.0, cmd_height, 0.0)
                else:
                    if (has_pos_hold and drone._sensors.sensor_data_ready and
                            drone._sensors.height > 0.05 and 
                            drone.position_hold_mode != "firmware"):
                        mvx, mvy = drone._position_hold.calculate_corrections(
                            drone._position_engine.x, drone._position_engine.y,
                            drone._position_engine.vx, drone._position_engine.vy,
                            drone._sensors.height, True, current_yaw=drone._sensors.yaw
                        )
                    else:
                        mvx, mvy = 0.0, 0.0
                    total_vx = drone.hover_trim_pitch + mvx
                    total_vy = drone.hover_trim_roll + mvy
                    cf.commander.send_hover_setpoint(
                        total_vx, total_vy, 0, cmd_height
                    )
            _log_csv_row(drone)
            time.sleep(0.01)

        # === STABILIZE ===
        drone._flight_phase = "MANUAL_STABILIZING"
        stab_start = time.time()
        while (time.time() - stab_start < 3.0 and drone._manual_active):
            if not check_link_safety(cf, drone._sensors.sensor_data_ready and drone.position_hold_mode != "firmware",
                                    drone._sensors.last_sensor_heartbeat,
                                    drone.debug_mode, drone._logger_fn):
                drone._manual_active = False
                break
            if not drone.debug_mode:
                if drone.position_hold_mode == "firmware" and drone.commander_mode == "position":
                    cf.commander.send_position_setpoint(0.0, 0.0, drone.target_height, 0.0)
                else:
                    if (has_pos_hold and drone._sensors.sensor_data_ready and 
                            drone.position_hold_mode != "firmware"):
                        mvx, mvy = drone._position_hold.calculate_corrections(
                            drone._position_engine.x, drone._position_engine.y,
                            drone._position_engine.vx, drone._position_engine.vy,
                            drone._sensors.height, True
                        )
                    else:
                        mvx, mvy = 0.0, 0.0
                    total_vx = drone.hover_trim_pitch + mvx
                    total_vy = drone.hover_trim_roll + mvy
                    cf.commander.send_hover_setpoint(
                        total_vx, total_vy, 0, drone.target_height
                    )
            time.sleep(drone.control_update_rate)

        # === MANUAL CONTROL LOOP ===
        drone._flight_phase = "MANUAL_CONTROL"
        if drone._logger_fn:
            drone._logger_fn("Manual control: WASD=Move, Q/E=Yaw, R/F=Up/Down, SPACE=Land")

        # Start keyboard listener thread (Windows: msvcrt)
        import threading as _threading
        _key_timeout = 0.15  # seconds before key auto-releases

        def _keyboard_listener():
            """Poll for keyboard input and update _manual_keys."""
            try:
                import msvcrt
            except ImportError:
                if drone._logger_fn:
                    drone._logger_fn(
                        "WARNING: msvcrt not available, "
                        "keyboard control disabled"
                    )
                return

            key_timers = {}
            while drone._manual_active:
                if msvcrt.kbhit():
                    ch = msvcrt.getch()
                    ch_str = ch.decode("utf-8", errors="ignore").lower()
                    if ch_str == " " or ch == b'\x1b' or ch == b'\x03':
                        # Quit / land / Ctrl+C
                        if ch == b'\x03':
                            print("\n Ctrl+C detected — EMERGENCY STOP!")
                            drone.emergency_stop()
                        drone._manual_active = False
                        break
                    if ch_str in drone._manual_keys:
                        drone._manual_keys[ch_str] = True
                        key_timers[ch_str] = time.time()

                # Auto-release keys after timeout
                now = time.time()
                for k in list(key_timers):
                    if now - key_timers[k] > _key_timeout:
                        drone._manual_keys[k] = False
                        del key_timers[k]

                time.sleep(0.02)

        kb_thread = _threading.Thread(
            target=_keyboard_listener, daemon=True
        )
        kb_thread.start()

        last_loop_time = time.time()

        # Position-mode navigation state (for commander_mode == "position")
        nav_x = drone._sensors.kalman_x if drone._sensors.sensor_data_ready else 0.0
        nav_y = drone._sensors.kalman_y if drone._sensors.sensor_data_ready else 0.0
        nav_yaw = drone._sensors.yaw if drone._sensors.sensor_data_ready else 0.0
        _last_h_log = 0.0

        while drone._manual_active:
            if not check_link_safety(cf, drone._sensors.sensor_data_ready and drone.position_hold_mode != "firmware",
                                    drone._sensors.last_sensor_heartbeat,
                                    drone.debug_mode, drone._logger_fn):
                drone._manual_active = False
                break
            
            # Flight safety check for manual movement
            if not drone._flight_active:
                # If key pressed, warn user
                if any(drone._manual_keys.get(k) for k in ["w", "a", "s", "d", "q", "e", "r", "f", "up", "down", "left", "right"]):
                    if drone._logger_fn and time.time() - _last_h_log > 1.0:
                        drone._logger_fn("Cannot command — not in flight!")
                        _last_h_log = time.time()
                time.sleep(0.1)
                continue

            current_time = time.time()
            dt = current_time - last_loop_time
            last_loop_time = current_time
            if dt > 0.1:
                dt = 0.1

            # Get joystick input from key states
            # Drone body frame: +X = forward (pitch), +Y = left (roll)
            # send_hover_setpoint(vx, vy): vx → pitch, vy → roll
            joystick_vx = 0.0  # forward / backward
            joystick_vy = 0.0  # left / right
            joystick_yaw = 0.0 # yaw rotation rate (deg/s)

            if drone._manual_keys.get("w", False):
                joystick_vx += drone.sensitivity   # W = forward (+X)
            if drone._manual_keys.get("s", False):
                joystick_vx -= drone.sensitivity   # S = backward (-X)
            if drone._manual_keys.get("a", False):
                joystick_vy += drone.sensitivity   # A = left (+Y)
            if drone._manual_keys.get("d", False):
                joystick_vy -= drone.sensitivity   # D = right (-Y)
            
            if drone._manual_keys.get("q", False):
                joystick_yaw += 90.0               # Q = yaw left (+deg/s)
            if drone._manual_keys.get("e", False):
                joystick_yaw -= 90.0               # E = yaw right (-deg/s)
            
            # Height Adjustment (R = Up, F = Down) with safety caps and feedback
            if drone._manual_keys.get("r", False):
                drone.target_height += 0.5 * dt    # Rate: 0.5 m/s
                if drone._logger_fn and time.time() - _last_h_log > 2.0:
                    drone._logger_fn(f"Climbing to {drone.target_height:.2f}m...")
                    _last_h_log = time.time()
            if drone._manual_keys.get("f", False):
                drone.target_height -= 0.5 * dt
                if drone._logger_fn and time.time() - _last_h_log > 2.0:
                    drone._logger_fn(f"Descending to {drone.target_height:.2f}m...")
                    _last_h_log = time.time()

            # Enforce Blockly-standard safety caps (0.15m - 1.0m)
            if drone.target_height < 0.15:
                drone.target_height = 0.15
            elif drone.target_height > 2.0:
                drone.target_height = 2.0

            # Check if user is actively providing input
            joystick_active = (abs(joystick_vx) > 0.001 or 
                               abs(joystick_vy) > 0.001 or 
                               abs(joystick_yaw) > 0.1 or 
                               drone._manual_keys.get("r", False) or 
                               drone._manual_keys.get("f", False))

            joystick_vx = round(joystick_vx, 2)
            joystick_vy = round(joystick_vy, 2)
            joystick_yaw = round(joystick_yaw, 2)
            drone.target_height = round(float(drone.target_height), 2)

            if drone.position_hold_mode == "firmware":
                if drone.commander_mode == "position":
                    # === POSITION MODE: accumulate target coordinates ===
                    if joystick_active:
                        nav_x += joystick_vx * dt
                        nav_y += joystick_vy * dt
                        nav_yaw += joystick_yaw * dt
                    elif drone.hold_mode == "origin":
                        # Snap back to takeoff origin when idle
                        nav_x, nav_y, nav_yaw = 0.0, 0.0, 0.0
                    # else: hold_mode == "current" — nav stays locked at last position

                    if not drone.debug_mode:
                        cf.commander.send_position_setpoint(
                            round(nav_x, 2), round(nav_y, 2),
                            drone.target_height, round(nav_yaw, 2)
                        )
                else:
                    # === HOVER MODE: send velocity commands ===
                    if joystick_active or drone.hold_mode != "origin":
                        if not drone.debug_mode:
                            cf.commander.send_hover_setpoint(
                                joystick_vx, joystick_vy, joystick_yaw, drone.target_height
                            )
                    else:
                        # Sticks released AND hold_mode == "origin" -> lock onto takeoff spot
                        if not drone.debug_mode:
                            cf.commander.send_position_setpoint(
                                0.0, 0.0, drone.target_height, 0.0
                            )
            else:
                # Update target position based on hold mode (Library PID)
                if has_pos_hold and drone._sensors.sensor_data_ready:
                    if drone.hold_mode == "current":
                        # Target moves with joystick input
                        drone._position_hold.target_x += joystick_vx * dt
                        drone._position_hold.target_y += joystick_vy * dt
                        # Clamp target
                        dx = drone._position_hold.target_x - drone._position_engine.x
                        dy = drone._position_hold.target_y - drone._position_engine.y
                        if abs(dx) > drone.max_position_error:
                            drone._position_hold.target_x = (
                                drone._position_engine.x +
                                math.copysign(drone.max_position_error, dx)
                            )
                        if abs(dy) > drone.max_position_error:
                            drone._position_hold.target_y = (
                                drone._position_engine.y +
                                math.copysign(drone.max_position_error, dy)
                            )
                    else:
                        # Origin mode — target is always (0, 0)
                        drone._position_hold.target_x = 0.0
                        drone._position_hold.target_y = 0.0

                    # Calculate PID corrections
                    mvx, mvy = drone._position_hold.calculate_corrections(
                        drone._position_engine.x, drone._position_engine.y,
                        drone._position_engine.vx, drone._position_engine.vy,
                        drone._sensors.height, True
                    )
                else:
                    mvx, mvy = 0.0, 0.0

                # Combine feedforward (joystick) + feedback (PID)
                total_vx = round(drone.hover_trim_pitch + mvx + joystick_vx, 2)
                total_vy = round(drone.hover_trim_roll + mvy + joystick_vy, 2)

                if not drone.debug_mode:
                    cf.commander.send_hover_setpoint(
                        total_vx, total_vy, joystick_yaw, drone.target_height
                    )

            _log_csv_row(drone)
            time.sleep(drone.control_update_rate)

        # === LANDING ===
        drone._flight_phase = "MANUAL_LANDING"
        if drone._logger_fn:
            drone._logger_fn("Manual control: landing...")
        land_start = time.time()
        current_land_height = drone.target_height
        dt = 0.02  # 50Hz control loop

        # Capture the landing position so we lock onto it during descent
        if drone.position_hold_mode == "firmware":
            if drone.commander_mode == "position":
                # Use the tracked nav position (already precise)
                land_x, land_y, land_yaw = round(nav_x, 2), round(nav_y, 2), round(nav_yaw, 2)
                if drone.hold_mode == "origin":
                    land_x, land_y, land_yaw = 0.0, 0.0, 0.0
            elif drone.hold_mode == "origin":
                land_x, land_y, land_yaw = 0.0, 0.0, 0.0
            else:
                land_x = round(drone._sensors.kalman_x, 2)
                land_y = round(drone._sensors.kalman_y, 2)
                land_yaw = round(drone._sensors.yaw, 2)

        while (current_land_height > 0.10 and drone._flight_active):
            current_land_height -= drone.descent_rate * dt
            current_land_height = max(current_land_height, 0.0)

            if not drone.debug_mode:
                if drone.position_hold_mode == "firmware":
                    # Use position setpoint to lock XY and descend Z
                    cf.commander.send_position_setpoint(
                        land_x, land_y, round(current_land_height, 2), land_yaw
                    )
                else:
                    # Keep position hold active during descent (Library PID)
                    if (has_pos_hold and drone._sensors.sensor_data_ready and
                            current_land_height > 0.03):
                        mvx, mvy = drone._position_hold.calculate_corrections(
                            drone._position_engine.x, drone._position_engine.y,
                            drone._position_engine.vx, drone._position_engine.vy,
                            drone._sensors.height, True
                        )
                    else:
                        mvx, mvy = 0.0, 0.0

                    total_vx = round(drone.hover_trim_pitch + mvx, 2)
                    total_vy = round(drone.hover_trim_roll + mvy, 2)
                    cf.commander.send_hover_setpoint(
                        total_vx, total_vy, 0, round(current_land_height, 2)
                    )
            _log_csv_row(drone)
            time.sleep(dt)

        # Brief settle at ground level before killing motors
        settle_start = time.time()
        while time.time() - settle_start < 0.3 and drone._flight_active:
            if not drone.debug_mode:
                if drone.position_hold_mode == "firmware":
                    cf.commander.send_position_setpoint(
                        land_x, land_y, 0.0, land_yaw
                    )
                else:
                    cf.commander.send_hover_setpoint(
                        round(drone.hover_trim_pitch, 2), round(drone.hover_trim_roll, 2), 0, 0
                    )
            time.sleep(0.02)

        if not drone.debug_mode:
            if drone.position_hold_mode == "firmware":
                cf.high_level_commander.stop()
            else:
                cf.commander.send_setpoint(0, 0, 0, 0)

        drone._flight_phase = "MANUAL_COMPLETE"
        if drone._logger_fn:
            drone._logger_fn("Manual control complete!")

    except Exception as e:
        drone._flight_phase = "MANUAL_ERROR"
        if drone._logger_fn:
            drone._logger_fn(f"Manual control error: {str(e)}")
        try:
            if not drone.debug_mode:
                if drone.position_hold_mode == "firmware":
                    cf.high_level_commander.stop()
                else:
                    cf.commander.send_setpoint(0, 0, 0, 0)
        except Exception:
            pass
    finally:
        drone._flight_logger.stop(logger=drone._logger_fn)
        # Teardown (logging/LEDs/disconnect) is now handled by the user/disconnect()
        drone._flight_active = False
        drone._manual_active = False



def _log_csv_row(drone):
    """Log current state to CSV if logging is active."""
    drone._flight_logger.log_row(
        drone._position_engine.x,
        drone._position_engine.y,
        drone._sensors.height,
        drone._sensors.range_height,
        drone._position_engine.vx,
        drone._position_engine.vy,
        drone._position_hold.correction_vx,
        drone._position_hold.correction_vy,
        battery=drone._sensors.battery_voltage,
        roll=drone._sensors.roll,
        pitch=drone._sensors.pitch,
        yaw=drone._sensors.yaw,
        gyro_x=drone._sensors.gyro_x,
        gyro_y=drone._sensors.gyro_y,
        gyro_z=drone._sensors.gyro_z,
        flight_phase=drone._flight_phase,
        target_height=drone._cmd_height,
        cmd_thrust=drone._sensors.thrust,
    )
