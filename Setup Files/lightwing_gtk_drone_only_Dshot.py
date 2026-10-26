#!/usr/bin/env python3
"""
LiteWing Drone Control — GTK (no video)
========================================
Pure drone control: connect, arm, joystick/keyboard flight, telemetry,
E-STOP. No camera, no GStreamer, no video pipeline of any kind - this is
the phase-2 baseline with all video code removed.

Threads/contexts:
  1. GTK/GLib main loop      - GUI, gamepad IOChannel events, timers
  2. cflib background thread - telemetry + connection callbacks
                                (marshaled to GUI via GLib.idle_add)
  3. persistent_stream thread - sends a setpoint every 100ms (keepalive)

Safety features:
  - E-STOP button + spacebar: sends stop-setpoint immediately (motors cut)
  - Two-step arm: Start -> "CONFIRM ARM?" within 3s -> armed
  - Battery voltage monitoring with low-voltage warning
  - persistent_stream hardened: link errors surface in the log, never die silently

Usage:
  python3 lightwing_gtk_drone_only.py --drone udp://192.168.0.12:2390
  (optional - default below)
"""

import os
import fcntl

import time
import math
import argparse
import threading
from datetime import datetime

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, Gdk

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

try:
    import evdev
    from evdev import ecodes as ev_ecodes
    GAMEPAD_AVAILABLE = True
except ImportError:
    GAMEPAD_AVAILABLE = False

cflib.crtp.init_drivers()

# ── Config - defaults, overridable from the command line ────────────────────
# DRONE_URI = "udp://192.168.0.12:2390"
# DRONE_URI = "udp://192.168.0.07:2390" #Caustom Drone(made by us)
DRONE_URI = "udp://192.168.43.42:2390" #Caustom Drone(made by us)

# Manual control tuning
ROLL_PITCH_STEP = 2.0
MAX_TILT = 15.0
THRUST_STEP = 2000
THRUST_MIN = 0
# cflib's send_setpoint() thrust field is a 16-bit unsigned int (0-65535 /
# 0xFFFF). 60000 leaves a safety margin under that hard ceiling - do not
# raise this above 65535 or send_setpoint() will throw on every call once
# the stick crosses that value.
THRUST_MAX = 60000

# Safety / robustness tuning
BATTERY_LOW_V = 3.3               # warn below this (per-cell LiPo sag threshold)
ARM_CONFIRM_TIMEOUT_S = 3         # Start -> Confirm window before reverting
LOG_DIR = os.path.expanduser("~/litewing_logs")

# ── Positioning Module - "all sensor data" telemetry ─────────────────────────
# Every CRTP variable the firmware exposes from the optical-flow/ToF module
# plus the Kalman estimator's internal state, confirmed directly from the
# firmware source (LOG_ADD calls in flowdeck_v1v2.c, range.c, stabilizer.c,
# estimator_kalman.c) - not guessed from docs.
#
# Split into several LogConfig BLOCKS rather than one giant one, because a
# CRTP log block has a payload size limit (~26 bytes) - cramming every float
# into a single config would silently fail or get rejected. Each tuple is
# (crtp_variable_name, cflib_type, display_label). Each block is independent:
# if one variable or block isn't available on a given firmware build, only
# that piece is skipped (see DroneLink._start_extra_telemetry) - the rest
# keeps working.
EXTRA_TELEMETRY_BLOCKS = [
    # Motor outputs: the 0-65535 number the firmware sends to each motor
    # (motors.c LOG group "pwm"). Works on brushed and DShot firmware.
    ("Motors", [
        ("pwm.m1_pwm", "uint32_t", "M1 ratio"),
        ("pwm.m2_pwm", "uint32_t", "M2 ratio"),
        ("pwm.m3_pwm", "uint32_t", "M3 ratio"),
        ("pwm.m4_pwm", "uint32_t", "M4 ratio"),
    ]),
    # DShot frames the RMT driver refused - should stay 0. Only exists on
    # the DShot firmware. Kept in its own block on purpose: cflib rejects a
    # whole block if one variable is missing, so on older firmware only this
    # line is skipped and the M1-M4 values above still show.
    ("DShot", [
        ("pwm.dshotErr", "uint32_t", "DShot errors"),
    ]),
    ("PosVel", [
        ("stateEstimate.x", "float", "Pos X (m)"),
        ("stateEstimate.y", "float", "Pos Y (m)"),
        ("stateEstimate.vx", "float", "Vel X (m/s)"),
        ("stateEstimate.vy", "float", "Vel Y (m/s)"),
        ("stateEstimate.vz", "float", "Vel Z (m/s)"),
    ]),
    ("FlowRange", [
        ("range.zrange", "uint16_t", "Raw ToF range (mm)"),
        ("motion.deltaX", "int16_t", "Flow deltaX (raw)"),
        ("motion.deltaY", "int16_t", "Flow deltaY (raw)"),
        ("motion.squal", "uint8_t", "Flow quality (squal)"),
        ("motion.shutter", "uint16_t", "Flow shutter"),
        ("motion.outlierCount", "uint8_t", "Flow outliers"),
        ("motion.motion", "uint8_t", "Flow motion flag"),
        ("motion.maxRaw", "uint8_t", "Flow maxRaw"),
        ("motion.minRaw", "uint8_t", "Flow minRaw"),
        ("motion.Rawsum", "uint8_t", "Flow rawSum"),
        ("motion.std", "float", "Flow std"),
    ]),
    ("KalmanCore", [
        ("kalman.stateX", "float", "Kalman X"),
        ("kalman.stateY", "float", "Kalman Y"),
        ("kalman.stateZ", "float", "Kalman Z"),
        ("kalman.statePX", "float", "Kalman VX"),
        ("kalman.statePY", "float", "Kalman VY"),
        ("kalman.statePZ", "float", "Kalman VZ"),
    ]),
    ("KalmanVar1", [
        ("kalman.stateD0", "float", "Kalman D0"),
        ("kalman.stateD1", "float", "Kalman D1"),
        ("kalman.stateD2", "float", "Kalman D2"),
        ("kalman.varX", "float", "Kalman varX"),
        ("kalman.varY", "float", "Kalman varY"),
        ("kalman.varZ", "float", "Kalman varZ"),
    ]),
    ("KalmanVar2", [
        ("kalman.varPX", "float", "Kalman varVX"),
        ("kalman.varPY", "float", "Kalman varVY"),
        ("kalman.varPZ", "float", "Kalman varVZ"),
        ("kalman.varD0", "float", "Kalman varD0"),
        ("kalman.varD1", "float", "Kalman varD1"),
        ("kalman.varD2", "float", "Kalman varD2"),
    ]),
    ("KalmanQuat", [
        ("kalman.q0", "float", "Quat q0"),
        ("kalman.q1", "float", "Quat q1"),
        ("kalman.q2", "float", "Quat q2"),
        ("kalman.q3", "float", "Quat q3"),
        ("kalman.inFlight", "uint8_t", "In-flight flag"),
    ]),
    ("KalmanRates", [
        ("kalman.rtUpdate", "float", "Rate: update"),
        ("kalman.rtPred", "float", "Rate: predict"),
        ("kalman.rtBaro", "float", "Rate: baro"),
        ("kalman.rtFinal", "float", "Rate: finalize"),
        ("kalman.rtApnd", "float", "Rate: appended"),
        ("kalman.rtRej", "float", "Rate: rejected"),
    ]),
    ("KalmanStatesGrp", [
        ("kalman_states.ox", "float", "KS offset X"),
        ("kalman_states.oy", "float", "KS offset Y"),
        ("kalman_states.vx", "float", "KS vel X"),
        ("kalman_states.vy", "float", "KS vel Y"),
    ]),
]

# ── Gamepad (Quantron QGP-1800) tuning ───────────────────────────────────────
# Confirmed via evtest/jstest on the Jetson: DragonRise/Microntek chipset,
# ABS_X/Y/Z/RZ range 0-255 (center 128, flat/deadzone 15), 12 buttons.
GAMEPAD_DEVICE_NAME_HINT = "microntek"
GAMEPAD_AXIS_MIN, GAMEPAD_AXIS_MAX, GAMEPAD_AXIS_CENTER = 0, 255, 128
GAMEPAD_DEADZONE_RAW = 15

# Roll/pitch/yaw use the normal symmetric -1..+1 mapping. Flip any of these
# if that channel reads backwards on your hardware.
GAMEPAD_AXIS_INVERT = {
    "ABS_X": False,   # CH1 - roll
    "ABS_Y": False,   # CH2 - pitch
    "ABS_Z": False,   # CH3 - yaw
}

# Throttle (CH4 / ABS_RZ) uses a DIFFERENT, asymmetric mapping - confirmed
# working via rc_drone_isolated_test.py:
#   stick at CENTER or above -> THRUST_MIN (idle, dead zone, no response)
#   stick pushed DOWN        -> ramps linearly up to THRUST_MAX
# Only the downward half of the stick's travel does anything, by design.
# If the live half ends up on the wrong physical side once tested, flip
# this one flag - nothing else needs to change.
GAMEPAD_THROTTLE_DOWN_IS_RAW_LOW = True

GAMEPAD_YAW_RATE_MAX = 200  # deg/s, matches typical cflib yaw-rate setpoint scale

# Button codes confirmed via evtest (EV_KEY codes, not indices):
#   294 BASE  295 BASE2  296 BASE3  297 BASE4  298 BASE5  299 BASE6
# BASE3 = kill switch (disable CH1-4 response). BASE4 = enable flight
# (allow CH1-4 response). Both are MOMENTARY on the hardware - the button
# reports pressed then immediately released - so the app latches the
# enable/disable state itself rather than treating them as held switches.
GAMEPAD_BTN_DISABLE_FLIGHT = 296  # BASE3
GAMEPAD_BTN_ENABLE_FLIGHT = 297   # BASE4

# ── CSS styling ──────────────────────────────────────────────────────────────
CSS = b"""
* { font-family: monospace; }
window { background-color: #0a0a0a; }
#title_label { color: #00ff88; font-size: 16px; font-weight: bold; padding: 8px 0px 2px 0px; }
#section_label { color: #00ff88; font-size: 11px; font-weight: bold; padding: 4px 0px; }
#status_ok { color: #00ff88; font-size: 11px; font-weight: bold; }
#status_bad { color: #ff4444; font-size: 11px; font-weight: bold; }
#status_off { color: #334433; font-size: 11px; }
#readout { color: #00ff88; font-size: 13px; font-weight: bold; }
#readout_warn { color: #ff4444; font-size: 13px; font-weight: bold; }
#log_view { background-color: #0a0a0a; color: #66cc88; font-family: monospace; font-size: 10px; }
button { font-family: monospace; }
#control_panel { background-color: rgba(10, 10, 10, 0.92); border-left: 1px solid #1a3a1a; }
#estop_btn { background-color: #3a0808; color: #ff4444; border: 2px solid #cc2222;
             font-size: 14px; font-weight: bold; padding: 10px 0px; }
#arm_confirm { background-color: #3a2a08; color: #ffcc00; border: 1px solid #cc9900;
               font-weight: bold; }
#led_off { color: #334433; font-size: 11px; font-weight: bold; }
#led_on { color: #00ff88; font-size: 11px; font-weight: bold; background-color: #0a2a1a;
          border: 1px solid #00aa55; padding: 2px 4px; }
"""


class AttitudeIndicator(Gtk.DrawingArea):
    """Artificial horizon (attitude indicator), pure GTK/Cairo widget,
    redraws on the GUI thread at telemetry rate (~10Hz).

    Color convention:
      green  = moving references (horizon, ladder, roll arc)
      yellow = fixed references (top pointer, center chevron)
    """

    def __init__(self, size=280):
        super().__init__()
        self._attitude = (0.0, 0.0)  # atomic tuple (roll, pitch)
        self.set_size_request(size, size)
        self.connect("draw", self._on_draw)

    def set_attitude(self, roll_deg, pitch_deg):
        self._attitude = (roll_deg, pitch_deg)
        self.queue_draw()

    def _on_draw(self, widget, cr):
        roll_deg, pitch_deg = self._attitude

        w = widget.get_allocated_width()
        h = widget.get_allocated_height()
        cx, cy = w / 2, h / 2
        radius = min(w, h) / 2 - 42  # leave room for arc ticks + labels

        cr.set_source_rgb(0.05, 0.05, 0.05)
        cr.paint()

        # ---- Horizon ball (clipped to circle; rotates with roll,
        #      shifts with pitch) ----
        cr.save()
        cr.arc(cx, cy, radius, 0, 2 * math.pi)
        cr.clip()

        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(roll_deg))

        pixels_per_degree = radius / 45.0
        # Clamp displayed pitch so extreme telemetry pins at the ladder
        # edge instead of drawing meaninglessly off-scale.
        display_pitch = max(-60.0, min(60.0, pitch_deg))
        pitch_offset = display_pitch * pixels_per_degree
        cr.translate(0, pitch_offset)

        big = radius * 3  # oversized so rotation never reveals corners
        cr.set_source_rgb(0.25, 0.55, 0.85)   # sky
        cr.rectangle(-big, -big, 2 * big, big)
        cr.fill()
        cr.set_source_rgb(0.30, 0.55, 0.15)   # ground
        cr.rectangle(-big, 0, 2 * big, big)
        cr.fill()

        cr.set_source_rgb(1, 1, 1)            # horizon line
        cr.set_line_width(2)
        cr.move_to(-big, 0)
        cr.line_to(big, 0)
        cr.stroke()

        cr.set_line_width(1.5)                # pitch ladder
        for deg in range(-60, 61, 10):
            if deg == 0:
                continue
            y = -deg * pixels_per_degree
            half = 26 if deg % 20 == 0 else 14
            cr.move_to(-half, y)
            cr.line_to(half, y)
            cr.stroke()

        cr.restore()  # undo roll/pitch transform
        cr.restore()  # undo clip

        # ---- Roll arc: ticks every 10deg to 40, labels every 20 with
        #      minus sign on the left side; rotates with roll ----
        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(roll_deg))
        cr.set_source_rgb(1, 1, 1)
        cr.select_font_face("monospace", 0, 1)
        cr.set_font_size(11)
        for mag in (0, 10, 20, 30, 40):
            for sign in ((1,) if mag == 0 else (-1, 1)):
                theta = math.radians(sign * mag)
                is_labeled = mag % 20 == 0
                tick_len = 11 if is_labeled else 6
                x1, y1 = radius * math.sin(theta), -radius * math.cos(theta)
                x2 = (radius + tick_len) * math.sin(theta)
                y2 = -(radius + tick_len) * math.cos(theta)
                cr.set_line_width(2)
                cr.move_to(x1, y1)
                cr.line_to(x2, y2)
                cr.stroke()

                if is_labeled:
                    label = f"-{mag}" if sign < 0 and mag != 0 else str(mag)
                    lx = (radius + tick_len + 12) * math.sin(theta)
                    ly = -(radius + tick_len + 12) * math.cos(theta)
                    cr.save()
                    cr.translate(lx, ly)
                    cr.rotate(theta)
                    extents = cr.text_extents(label)
                    cr.move_to(-extents.width / 2, extents.height / 2)
                    cr.show_text(label)
                    cr.restore()
        cr.restore()

        # ---- Fixed pointer (yellow - never rotates) ----
        cr.set_source_rgb(1.0, 0.82, 0.05)
        px, py = cx, cy - radius - 4
        cr.move_to(px - 6, py - 10)
        cr.line_to(px + 6, py - 10)
        cr.line_to(px, py)
        cr.close_path()
        cr.fill()

        # ---- Fixed center chevron (yellow - never moves) ----
        cr.set_source_rgb(1.0, 0.82, 0.05)
        cr.set_line_width(3)
        cr.move_to(cx - 26, cy + 9)
        cr.line_to(cx - 5, cy + 2)
        cr.line_to(cx, cy + 6)
        cr.line_to(cx + 5, cy + 2)
        cr.line_to(cx + 26, cy + 9)
        cr.stroke()
        cr.arc(cx, cy, 3, 0, 2 * math.pi)
        cr.fill()

        # ---- Bezel ring ----
        cr.set_source_rgb(0.6, 0.6, 0.6)
        cr.set_line_width(3)
        cr.arc(cx, cy, radius, 0, 2 * math.pi)
        cr.stroke()

        return False


class DroneLink:
    """Owns ONE drone, end to end: Crazyflie connection, telemetry,
    keepalive thread, arm state, and its own Connect/Start/Stop/E-STOP
    buttons. Kept as its own class (not inlined into the app) so a
    second drone can be added back later without restructuring.
    """

    def __init__(self, app, name, uri):
        self.app = app
        self.name = name
        self.uri = uri

        self.cf = None
        self.connected = False
        self.armed = False
        self.log_conf = None
        self.stop_stream_evt = threading.Event()
        self.raw_attitude = {"roll": 0.0, "pitch": 0.0}
        self.calib_offset = {"roll": 0.0, "pitch": 0.0}

        # "All sensor data" telemetry (Positioning Module + Kalman debug) -
        # see EXTRA_TELEMETRY_BLOCKS. Several LogConfig blocks run
        # concurrently; extra_data collects their values by display label,
        # extra_dirty is a cheap flag set from cflib's background thread and
        # checked by a GLib timer (see _extra_display_tick) so the widget
        # only actually redraws a few times a second, not once per variable.
        self.extra_data = {}
        self.extra_log_confs = []
        self._extra_dirty = False
        self._extra_tick_id = None

        # Two-step arm state (Start -> "CONFIRM ARM?" -> armed)
        self._arm_pending = False
        self._arm_confirm_timeout_id = None

        # Widgets - attached after _build_ui constructs them
        self.status_lbl = None
        self.roll_lbl = None
        self.pitch_lbl = None
        self.height_lbl = None
        self.battery_lbl = None
        self.extra_view = None
        self.connect_btn = None
        self.start_btn = None
        self.stop_btn = None
        self.estop_btn = None

    def attach_widgets(self, status_lbl, roll_lbl, pitch_lbl, height_lbl, battery_lbl,
                        extra_view, connect_btn, start_btn, stop_btn, estop_btn):
        self.status_lbl = status_lbl
        self.roll_lbl = roll_lbl
        self.pitch_lbl = pitch_lbl
        self.height_lbl = height_lbl
        self.battery_lbl = battery_lbl
        self.extra_view = extra_view
        self.connect_btn = connect_btn
        self.start_btn = start_btn
        self.stop_btn = stop_btn
        self.estop_btn = estop_btn

    def make_crazyflie(self):
        cf = Crazyflie()
        cf.connected.add_callback(self._on_connected)
        cf.connection_failed.add_callback(self._on_connection_failed)
        cf.disconnected.add_callback(self._on_disconnected)
        return cf

    def do_connect(self):
        if self.connected:
            self.app.gui_log(f"[{self.name}] Already connected.")
            return
        self.cf = self.make_crazyflie()
        self.app.gui_log(f"[{self.name}] Connecting to {self.uri} ...")
        if self.connect_btn is not None:
            self.connect_btn.set_sensitive(False)
            self.connect_btn.set_label("Connecting...")
        self.cf.open_link(self.uri)

    # ── Two-step arm: Start -> "CONFIRM ARM?" (3s window) -> armed ───────
    def do_start(self):
        if not self.connected:
            self.app.gui_log(f"[{self.name}] Not connected yet - click Connect first.")
            return

        if not self._arm_pending:
            self._arm_pending = True
            if self.start_btn is not None:
                self.start_btn.set_label("CONFIRM ARM?")
                self.start_btn.set_name("arm_confirm")
            self._arm_confirm_timeout_id = GLib.timeout_add_seconds(
                ARM_CONFIRM_TIMEOUT_S, self._arm_confirm_expired)
            self.app.gui_log(f"[{self.name}] Arm requested - click again within "
                              f"{ARM_CONFIRM_TIMEOUT_S}s to confirm.")
            return

        # Second click within the window: actually arm.
        self._cancel_arm_confirm()
        self.app.gui_log(f"[{self.name}] Motors ARMED - streaming real setpoint.")
        self.armed = True
        if self.start_btn is not None:
            self.start_btn.set_sensitive(False)
        if self.stop_btn is not None:
            self.stop_btn.set_sensitive(True)

    def _arm_confirm_expired(self):
        if self._arm_pending:
            self.app.gui_log(f"[{self.name}] Arm request timed out - not armed.")
            self._arm_confirm_timeout_id = None
            self._cancel_arm_confirm()
        return False  # one-shot

    def _cancel_arm_confirm(self):
        self._arm_pending = False
        if self._arm_confirm_timeout_id is not None:
            GLib.source_remove(self._arm_confirm_timeout_id)
            self._arm_confirm_timeout_id = None
        if self.start_btn is not None:
            self.start_btn.set_label("Start")
            self.start_btn.set_name("")

    def do_stop(self):
        self.app.gui_log(f"[{self.name}] Motors STOPPED - streaming zero setpoint (still connected).")
        self.armed = False
        if self.start_btn is not None:
            self.start_btn.set_sensitive(True)
        if self.stop_btn is not None:
            self.stop_btn.set_sensitive(False)

    def estop(self):
        """Immediate motor cut - disarms, cancels any pending arm, sends
        the firmware's stop-setpoint burst."""
        self.armed = False
        self._cancel_arm_confirm()
        if self.start_btn is not None:
            self.start_btn.set_sensitive(self.connected)
        if self.stop_btn is not None:
            self.stop_btn.set_sensitive(False)
        return self.send_stop_setpoint_burst() if self.connected else False

    def calibrate(self):
        self.calib_offset["roll"] = self.raw_attitude["roll"]
        self.calib_offset["pitch"] = self.raw_attitude["pitch"]
        self.app.gui_log(
            f"[{self.name}] Display calibrated - offset stored "
            f"(raw roll={self.raw_attitude['roll']:.2f}, "
            f"raw pitch={self.raw_attitude['pitch']:.2f}). Display-only."
        )

    def persistent_stream(self):
        """Setpoint keepalive, 10Hz. Hardened so a link error can never
        make it die silently: failures surface in the log and the loop
        keeps trying while connected."""
        consecutive_errors = 0
        while self.connected and not self.stop_stream_evt.is_set():
            try:
                if self.armed:
                    sp = self.app.setpoint
                    self.cf.commander.send_setpoint(
                        sp["roll"], sp["pitch"], sp["yaw"], sp["thrust"])
                else:
                    self.cf.commander.send_setpoint(0, 0, 0, 0)
                if consecutive_errors:
                    self.app.gui_log(f"[{self.name} STREAM] Link recovered - setpoints flowing again.")
                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors == 1:
                    self.app.gui_log(f"[{self.name} STREAM ERROR] send_setpoint failed: {e} - retrying...")
                elif consecutive_errors == 10:
                    self.app.gui_log(f"[{self.name} STREAM ERROR] 10 consecutive send failures - "
                                      f"link likely down; firmware failsafe should cut motors.")
            time.sleep(0.1)
        self.app.gui_log(f"[{self.name} STREAM] Setpoint stream stopped.")

    # ── cflib callbacks (fire on cflib's own background thread) ──────────
    def _on_connected(self, link_uri):
        def _apply():
            self.connected = True
            self.app.gui_log(f"[OK] {self.name} connected: {link_uri}")
            if self.status_lbl is not None:
                self.status_lbl.set_text("Connected")
                self.status_lbl.set_name("status_ok")
            if self.connect_btn is not None:
                self.connect_btn.set_sensitive(False)
                self.connect_btn.set_label("Connected")
            if self.start_btn is not None:
                self.start_btn.set_sensitive(True)
            self.start_telemetry()
            self.stop_stream_evt.clear()
            threading.Thread(target=self.persistent_stream, daemon=True).start()
            return False
        GLib.idle_add(_apply)

    def _on_connection_failed(self, link_uri, msg):
        def _apply():
            self.connected = False
            self.app.gui_log(f"[FAIL] {self.name}: {msg}")
            if self.status_lbl is not None:
                self.status_lbl.set_text("Connection failed")
                self.status_lbl.set_name("status_bad")
            if self.connect_btn is not None:
                self.connect_btn.set_sensitive(True)
                self.connect_btn.set_label("Connect")
            if self.start_btn is not None:
                self.start_btn.set_sensitive(False)
            return False
        GLib.idle_add(_apply)

    def _on_disconnected(self, link_uri):
        def _apply():
            self.connected = False
            self.armed = False
            self._cancel_arm_confirm()
            self.stop_stream_evt.set()
            self.app.gui_log(f"[INFO] {self.name} disconnected: {link_uri}")
            if self.status_lbl is not None:
                self.status_lbl.set_text("Disconnected")
                self.status_lbl.set_name("status_bad")
            if self.connect_btn is not None:
                self.connect_btn.set_sensitive(True)
                self.connect_btn.set_label("Connect")
            if self.start_btn is not None:
                self.start_btn.set_sensitive(False)
            if self.stop_btn is not None:
                self.stop_btn.set_sensitive(False)
            self.stop_telemetry()
            return False
        GLib.idle_add(_apply)

    def start_telemetry(self):
        log_conf = LogConfig(name=f'AttitudeLog_{self.name}', period_in_ms=100)
        try:
            log_conf.add_variable('stabilizer.roll', 'float')
            log_conf.add_variable('stabilizer.pitch', 'float')
            log_conf.add_variable('pm.vbat', 'float')
        except (KeyError, AttributeError) as e:
            self.app.gui_log(f"[LOG ERROR] {self.name}: couldn't find expected variables: {e}")
            return

        # Height (Flight Positioning Module) - added separately from the
        # roll/pitch/battery block above so a drone without the module
        # still gets full roll/pitch/battery telemetry; only height itself
        # falls back to "--" if stateEstimate.z isn't in this drone's TOC.
        height_available = True
        try:
            log_conf.add_variable('stateEstimate.z', 'float')
        except (KeyError, AttributeError) as e:
            height_available = False
            self.app.gui_log(f"[WARN] {self.name}: stateEstimate.z unavailable ({e}) - "
                              f"height will show '--'. Check Positioning Module firmware.")

        def log_data_cb(timestamp, data, logconf):
            roll = data.get('stabilizer.roll', 0.0)
            pitch = data.get('stabilizer.pitch', 0.0)
            vbat = data.get('pm.vbat', 0.0)
            self.raw_attitude["roll"] = roll
            self.raw_attitude["pitch"] = pitch
            self._set_attitude_display(roll - self.calib_offset["roll"],
                                        pitch - self.calib_offset["pitch"])
            self._set_battery_display(vbat)
            if height_available:
                self._set_height_display(data.get('stateEstimate.z', 0.0))

        def log_error_cb(logconf, msg):
            self.app.gui_log(f"[LOG ERROR] {self.name}: {msg}")

        log_conf.data_received_cb.add_callback(log_data_cb)
        log_conf.error_cb.add_callback(log_error_cb)
        try:
            self.cf.log.add_config(log_conf)
            log_conf.start()
        except (KeyError, AttributeError) as e:
            self.app.gui_log(f"[WARN] {self.name}: battery variable unavailable ({e}) - "
                              f"retrying telemetry without it.")
            self._start_telemetry_no_battery()
            return
        self.log_conf = log_conf
        suffix = "/height" if height_available else ""
        self.app.gui_log(f"[{self.name}] Telemetry started - roll/pitch/battery{suffix} streaming.")
        self._start_extra_telemetry()

    def _start_telemetry_no_battery(self):
        log_conf = LogConfig(name=f'AttitudeLogNB_{self.name}', period_in_ms=100)
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')

        height_available = True
        try:
            log_conf.add_variable('stateEstimate.z', 'float')
        except (KeyError, AttributeError) as e:
            height_available = False
            self.app.gui_log(f"[WARN] {self.name}: stateEstimate.z unavailable ({e}) - "
                              f"height will show '--'.")

        def log_data_cb(timestamp, data, logconf):
            roll = data.get('stabilizer.roll', 0.0)
            pitch = data.get('stabilizer.pitch', 0.0)
            self.raw_attitude["roll"] = roll
            self.raw_attitude["pitch"] = pitch
            self._set_attitude_display(roll - self.calib_offset["roll"],
                                        pitch - self.calib_offset["pitch"])
            if height_available:
                self._set_height_display(data.get('stateEstimate.z', 0.0))

        log_conf.data_received_cb.add_callback(log_data_cb)
        log_conf.error_cb.add_callback(lambda lc, m: self.app.gui_log(f"[LOG ERROR] {self.name}: {m}"))
        self.cf.log.add_config(log_conf)
        log_conf.start()
        self.log_conf = log_conf
        self.app.gui_log(f"[{self.name}] Telemetry started - roll/pitch streaming (no battery var).")
        self._start_extra_telemetry()

    def stop_telemetry(self):
        if self.log_conf is not None:
            try:
                self.log_conf.stop()
            except Exception:
                pass
            self.log_conf = None

        for lc in self.extra_log_confs:
            try:
                lc.stop()
            except Exception:
                pass
        self.extra_log_confs = []
        self.extra_data = {}
        if self._extra_tick_id is not None:
            GLib.source_remove(self._extra_tick_id)
            self._extra_tick_id = None

        def _clear():
            if self.roll_lbl is not None:
                self.roll_lbl.set_text("Roll: --")
            if self.pitch_lbl is not None:
                self.pitch_lbl.set_text("Pitch: --")
            if self.height_lbl is not None:
                self.height_lbl.set_text("Height: --")
            if self.battery_lbl is not None:
                self.battery_lbl.set_text("Battery: --")
                self.battery_lbl.set_name("readout")
            if self.extra_view is not None:
                self.extra_view.get_buffer().set_text("(not connected)")
            self.app.attitude_indicator.set_attitude(0.0, 0.0)
            return False
        GLib.idle_add(_clear)

    def _start_extra_telemetry(self):
        """Starts every block in EXTRA_TELEMETRY_BLOCKS independently - one
        LogConfig per block, each wrapped so a variable or whole block
        missing from this firmware build just gets skipped (logged once)
        rather than taking down telemetry entirely."""
        self.extra_data = {}
        self.extra_log_confs = []

        for block_name, variables in EXTRA_TELEMETRY_BLOCKS:
            lc = LogConfig(name=f'{block_name}_{self.name}', period_in_ms=200)
            added = []
            for var, vtype, label in variables:
                try:
                    lc.add_variable(var, vtype)
                    added.append((var, label))
                except (KeyError, AttributeError) as e:
                    self.app.gui_log(f"[WARN] {self.name}: {var} unavailable ({e}) - skipped.")
            if not added:
                continue  # nothing in this block existed on this firmware

            def make_cb(added=added):
                def cb(timestamp, data, logconf):
                    for var, label in added:
                        if var in data:
                            self.extra_data[label] = data[var]
                    self._extra_dirty = True
                return cb

            lc.data_received_cb.add_callback(make_cb())
            lc.error_cb.add_callback(
                lambda logconf, msg: self.app.gui_log(f"[LOG ERROR] {self.name} {logconf.name}: {msg}"))
            try:
                self.cf.log.add_config(lc)
                lc.start()
                self.extra_log_confs.append(lc)
            except (KeyError, AttributeError) as e:
                self.app.gui_log(f"[WARN] {self.name}: couldn't start block '{block_name}' ({e}) - skipped.")

        if self.extra_log_confs:
            self.app.gui_log(f"[{self.name}] Extra sensor telemetry started - "
                              f"{len(self.extra_log_confs)}/{len(EXTRA_TELEMETRY_BLOCKS)} blocks active.")
            self._extra_tick_id = GLib.timeout_add(200, self._extra_display_tick)

    def _extra_display_tick(self):
        """10Hz-ish redraw throttle - only actually rewrites the widget if
        something changed since the last tick, no matter how many of the
        underlying blocks fired in between."""
        if self._extra_dirty:
            self._extra_dirty = False
            self._render_extra_display()
        return True  # keep the timer running

    def _render_extra_display(self):
        if self.extra_view is None:
            return
        lines = []
        for _, variables in EXTRA_TELEMETRY_BLOCKS:
            for _, _, label in variables:
                if label in self.extra_data:
                    value = self.extra_data[label]
                    if isinstance(value, float):
                        lines.append(f"{label}: {value:.4f}")
                    else:
                        lines.append(f"{label}: {value}")
        text = "\n".join(lines) if lines else "(waiting for data...)"
        self.extra_view.get_buffer().set_text(text)

    def _set_attitude_display(self, roll, pitch):
        def _apply():
            if self.roll_lbl is not None:
                self.roll_lbl.set_text(f"Roll: {roll:.2f}°")
            if self.pitch_lbl is not None:
                self.pitch_lbl.set_text(f"Pitch: {pitch:.2f}°")
            self.app.attitude_indicator.set_attitude(roll, pitch)
            return False
        GLib.idle_add(_apply)

    def _set_height_display(self, height_m):
        def _apply():
            if self.height_lbl is not None:
                self.height_lbl.set_text(f"Height: {height_m:.3f} m")
            return False
        GLib.idle_add(_apply)

    def _set_battery_display(self, vbat):
        def _apply():
            if self.battery_lbl is not None:
                self.battery_lbl.set_text(f"Battery: {vbat:.2f} V")
                self.battery_lbl.set_name(
                    "readout_warn" if vbat < BATTERY_LOW_V else "readout")
            return False
        GLib.idle_add(_apply)

    def send_stop_setpoint_burst(self, times=3):
        """Used by E-STOP - a few sends in case one is dropped."""
        if self.connected and self.cf is not None:
            try:
                for _ in range(times):
                    self.cf.commander.send_stop_setpoint()
                    time.sleep(0.01)
                return True
            except Exception as e:
                self.app.gui_log(f"[{self.name}] EMERGENCY STOP send failed: {e}")
                return False
        return False

    def close_link(self):
        self.stop_stream_evt.set()
        if self.cf is not None:
            try:
                self.cf.close_link()
            except Exception:
                pass


class DroneControlApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="LiteWing Drone Control")
        self.set_default_size(420, 760)

        # ── Drone/cflib state ─────────────────────────────────────────────
        # Single drone. Kept as a list (self.drones) rather than a bare
        # self.drone reference in do_emergency_stop/_on_destroy, so a
        # second DroneLink can be added back in later with minimal churn.
        self.drone = DroneLink(self, "drone", DRONE_URI)
        self.drones = [self.drone]
        self.setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0, "thrust": THRUST_MIN}

        self.attitude_indicator = AttitudeIndicator()

        # ── Gamepad state ─────────────────────────────────────────────────
        # Latched flags, since BASE3/BASE4 are momentary on the hardware -
        # they report pressed-then-released instantly, so the app has to
        # remember "flight enabled" itself rather than reading button state
        # directly. Starts False: gamepad axes are ignored until BASE4 is
        # pressed once, even if the sticks are already off-center.
        self.gamepad_flight_enabled = False
        self.gamepad_dev = None
        self._gamepad_channel = None
        self._gamepad_watch_id = None
        self._gamepad_btn_prev = {}  # for edge (rising-edge) detection on momentary buttons
        self._gamepad_last_block_reason = None  # rate-limits the "input ignored" debug log
        self._gamepad_first_event_seen = False  # confirms hardware events are actually arriving
        self._gamepad_display_dirty = False     # throttled-redraw flag, see _gamepad_display_tick

        # ── Persistent file log ───────────────────────────────────────────
        self._log_file = None
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            log_path = os.path.join(
                LOG_DIR, f"flight_{datetime.now():%Y%m%d_%H%M%S}.log")
            self._log_file = open(log_path, "a", buffering=1)  # line-buffered
            print(f"[INFO] Logging to {log_path}")
        except OSError as e:
            print(f"[WARN] Could not open log file: {e} - continuing without file log")

        css_provider = Gtk.CssProvider()
        css_provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), css_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self._build_ui()
        self.connect("destroy", self._on_destroy)
        # Keyboard flight controls (see _on_key_press for bindings)
        self.connect("key-press-event", self._on_key_press)
        # Gamepad flight controls (CH1-4 sticks + BASE3/BASE4 enable-kill)
        self._init_gamepad()

    # ======================================================================
    # UI
    # ======================================================================
    def _build_ui(self):
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        self.add(scroll)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.set_name("control_panel")
        left.set_margin_start(14)
        left.set_margin_end(14)
        left.set_margin_top(14)
        left.set_margin_bottom(14)

        title = Gtk.Label(label="◈ LiteWing Drone Control")
        title.set_name("title_label")
        left.pack_start(title, False, False, 0)

        self.uri_lbl = Gtk.Label(label=f"Target: {DRONE_URI}")
        self.uri_lbl.set_xalign(0)
        left.pack_start(self.uri_lbl, False, False, 0)

        self.status_lbl = Gtk.Label(label="Not connected")
        self.status_lbl.set_name("status_bad")
        self.status_lbl.set_xalign(0)
        left.pack_start(self.status_lbl, False, False, 4)

        # E-STOP: prominent, always available (also bound to spacebar)
        self.estop_btn = Gtk.Button(label="■ EMERGENCY STOP (Space)")
        self.estop_btn.set_name("estop_btn")
        self.estop_btn.connect("clicked", lambda *_: self.do_emergency_stop())
        left.pack_start(self.estop_btn, False, False, 6)

        self.connect_btn = Gtk.Button(label="Connect")
        self.connect_btn.connect("clicked", lambda *_: self.drone.do_connect())
        left.pack_start(self.connect_btn, False, False, 4)

        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.start_btn = Gtk.Button(label="Start")
        self.start_btn.set_sensitive(False)
        self.start_btn.connect("clicked", lambda *_: self.drone.do_start())
        self.stop_btn = Gtk.Button(label="Stop")
        self.stop_btn.set_sensitive(False)
        self.stop_btn.connect("clicked", lambda *_: self.drone.do_stop())
        btn_row.pack_start(self.start_btn, True, True, 0)
        btn_row.pack_start(self.stop_btn, True, True, 0)
        left.pack_start(btn_row, False, False, 6)

        # Attitude indicator - the side-panel artificial horizon
        indicator_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        indicator_box.set_halign(Gtk.Align.CENTER)
        indicator_box.pack_start(self.attitude_indicator, False, False, 0)
        left.pack_start(indicator_box, False, False, 6)

        # Telemetry
        telem_label = Gtk.Label(label="LIVE ATTITUDE")
        telem_label.set_name("section_label")
        telem_label.set_xalign(0)
        left.pack_start(telem_label, False, False, 10)

        # Roll / Pitch / Height side by side in one row - height on the
        # right, per request, so it's easy to eyeball all three at once
        # while testing whether the Positioning Module's height telemetry
        # is actually arriving.
        attitude_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)

        self.roll_lbl = Gtk.Label(label="Roll: --")
        self.roll_lbl.set_name("readout")
        self.roll_lbl.set_xalign(0)
        attitude_row.pack_start(self.roll_lbl, False, False, 0)

        self.pitch_lbl = Gtk.Label(label="Pitch: --")
        self.pitch_lbl.set_name("readout")
        self.pitch_lbl.set_xalign(0)
        attitude_row.pack_start(self.pitch_lbl, False, False, 0)

        self.height_lbl = Gtk.Label(label="Height: --")
        self.height_lbl.set_name("readout")
        self.height_lbl.set_xalign(0)
        attitude_row.pack_start(self.height_lbl, False, False, 0)

        left.pack_start(attitude_row, False, False, 0)

        self.battery_lbl = Gtk.Label(label="Battery: --")
        self.battery_lbl.set_name("readout")
        self.battery_lbl.set_xalign(0)
        left.pack_start(self.battery_lbl, False, False, 0)

        calib_btn = Gtk.Button(label="Calibrate (zero display)")
        calib_btn.connect("clicked", lambda *_: self.drone.calibrate())
        left.pack_start(calib_btn, False, False, 4)

        # All Positioning Module + Kalman estimator telemetry - every
        # variable in EXTRA_TELEMETRY_BLOCKS, rendered as one scrolling
        # monospace readout rather than ~35 individual labels.
        extra_label = Gtk.Label(label="ALL SENSOR DATA (Positioning Module)")
        extra_label.set_name("section_label")
        extra_label.set_xalign(0)
        left.pack_start(extra_label, False, False, 10)

        self.extra_view = Gtk.TextView()
        self.extra_view.set_name("log_view")
        self.extra_view.set_editable(False)
        self.extra_view.set_wrap_mode(Gtk.WrapMode.NONE)
        self.extra_view.set_size_request(-1, 340)
        self.extra_view.get_buffer().set_text("(not connected)")
        left.pack_start(self.extra_view, False, False, 4)

        self.drone.attach_widgets(self.status_lbl, self.roll_lbl, self.pitch_lbl,
                                   self.height_lbl, self.battery_lbl, self.extra_view,
                                   self.connect_btn, self.start_btn, self.stop_btn, self.estop_btn)

        # Manual control
        manual_label = Gtk.Label(label="MANUAL CONTROL")
        manual_label.set_name("section_label")
        manual_label.set_xalign(0)
        left.pack_start(manual_label, False, False, 10)

        keys_hint = Gtk.Label(label="Keys: arrows=roll/pitch  W/S=thrust  Space=E-STOP")
        keys_hint.set_name("status_off")
        keys_hint.set_xalign(0)
        left.pack_start(keys_hint, False, False, 0)

        gamepad_hint = Gtk.Label(label="Gamepad: BASE4=enable  BASE3=disable  (sticks=CH1-4)")
        gamepad_hint.set_name("status_off")
        gamepad_hint.set_xalign(0)
        left.pack_start(gamepad_hint, False, False, 0)

        self.gamepad_status_lbl = Gtk.Label(label="Gamepad flight: DISABLED")
        self.gamepad_status_lbl.set_name("status_bad")
        self.gamepad_status_lbl.set_xalign(0)
        left.pack_start(self.gamepad_status_lbl, False, False, 4)

        led_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.base3_led = Gtk.Label(label=" BASE3 (kill) ")
        self.base3_led.set_name("led_off")
        self.base4_led = Gtk.Label(label=" BASE4 (enable) ")
        self.base4_led.set_name("led_off")
        led_row.pack_start(self.base3_led, False, False, 0)
        led_row.pack_start(self.base4_led, False, False, 0)
        left.pack_start(led_row, False, False, 2)

        self.thrust_lbl = Gtk.Label(label=f"{THRUST_MIN}")
        self._add_adjust_row(left, "Throttle", self.thrust_lbl,
                              lambda: self.adjust_thrust(-THRUST_STEP),
                              lambda: self.adjust_thrust(THRUST_STEP))

        self.pitch_cmd_lbl = Gtk.Label(label="0.0°")
        self._add_adjust_row(left, "Pitch", self.pitch_cmd_lbl,
                              lambda: self.adjust_pitch(-ROLL_PITCH_STEP),
                              lambda: self.adjust_pitch(ROLL_PITCH_STEP))

        self.roll_cmd_lbl = Gtk.Label(label="0.0°")
        self._add_adjust_row(left, "Roll", self.roll_cmd_lbl,
                              lambda: self.adjust_roll(-ROLL_PITCH_STEP),
                              lambda: self.adjust_roll(ROLL_PITCH_STEP))

        center_btn = Gtk.Button(label="Center (level)")
        center_btn.connect("clicked", lambda *_: self.center_attitude())
        left.pack_start(center_btn, False, False, 4)

        # Log
        log_label = Gtk.Label(label="LOG")
        log_label.set_name("section_label")
        log_label.set_xalign(0)
        left.pack_start(log_label, False, False, 10)

        self.log_view = Gtk.TextView()
        self.log_view.set_name("log_view")
        self.log_view.set_editable(False)
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD)
        self.log_view.set_size_request(-1, 220)
        left.pack_start(self.log_view, False, False, 4)

        scroll.add(left)
        self.show_all()

    def _add_adjust_row(self, parent, label_text, value_label, on_minus, on_plus):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        lbl = Gtk.Label(label=f"{label_text}:")
        lbl.set_xalign(0)
        lbl.set_size_request(70, -1)
        minus_btn = Gtk.Button(label="-")
        minus_btn.connect("clicked", lambda *_: on_minus())
        plus_btn = Gtk.Button(label="+")
        plus_btn.connect("clicked", lambda *_: on_plus())
        value_label.set_name("readout")
        row.pack_start(lbl, False, False, 0)
        row.pack_start(minus_btn, False, False, 0)
        row.pack_start(value_label, True, True, 0)
        row.pack_start(plus_btn, False, False, 0)
        parent.pack_start(row, False, False, 2)

    # ======================================================================
    # Keyboard flight controls
    # ======================================================================
    def _on_key_press(self, widget, event):
        key = event.keyval
        if key == Gdk.KEY_space:
            self.do_emergency_stop()
            return True
        if key == Gdk.KEY_Left:
            self.adjust_roll(-ROLL_PITCH_STEP)
            return True
        if key == Gdk.KEY_Right:
            self.adjust_roll(ROLL_PITCH_STEP)
            return True
        if key == Gdk.KEY_Up:
            self.adjust_pitch(ROLL_PITCH_STEP)
            return True
        if key == Gdk.KEY_Down:
            self.adjust_pitch(-ROLL_PITCH_STEP)
            return True
        if key in (Gdk.KEY_w, Gdk.KEY_W):
            self.adjust_thrust(THRUST_STEP)
            return True
        if key in (Gdk.KEY_s, Gdk.KEY_S):
            self.adjust_thrust(-THRUST_STEP)
            return True
        if key in (Gdk.KEY_c, Gdk.KEY_C):
            self.center_attitude()
            return True
        return False  # let other keys through

    # ======================================================================
    # Gamepad flight controls (Quantron QGP-1800, evdev)
    # ======================================================================
    # Uses GLib.IOChannel.unix_new(fd) + channel.add_watch() instead of
    # calling GLib.io_add_watch(fd, ...) directly on a raw integer fd. The
    # direct-fd form is NOT reliably supported across PyGObject versions -
    # on some setups it silently never fires the callback at all. Wrapping
    # the fd in an explicit GLib.IOChannel is the documented, portable way
    # to watch a raw fd from GLib's main loop in Python.
    #
    # CH1 (ABS_X)  -> roll    CH2 (ABS_Y)  -> pitch
    # CH3 (ABS_Z)  -> yaw     CH4 (ABS_RZ) -> throttle
    # BASE3 (momentary) -> disable flight (stop taking CH1-4)
    # BASE4 (momentary) -> enable flight  (start taking CH1-4)
    def _init_gamepad(self):
        if not GAMEPAD_AVAILABLE:
            self.gui_log("[GAMEPAD] evdev not installed - gamepad control disabled. "
                         "Run: pip3 install evdev")
            return

        dev = self._find_gamepad_device()
        if dev is None:
            self.gui_log(f"[GAMEPAD] No device matching "
                         f"'{GAMEPAD_DEVICE_NAME_HINT}' found - gamepad control disabled.")
            return

        # Non-blocking as a second safety net: even if the IOChannel watch
        # ever fires spuriously with no data ready, dev.read() won't stall
        # the GTK main loop - it raises BlockingIOError instead.
        try:
            flags = fcntl.fcntl(dev.fd, fcntl.F_GETFL)
            fcntl.fcntl(dev.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        except Exception as e:
            self.gui_log(f"[GAMEPAD] Warning: could not set fd non-blocking ({e}).")

        self.gamepad_dev = dev
        self._gamepad_channel = GLib.IOChannel.unix_new(dev.fd)
        self._gamepad_watch_id = self._gamepad_channel.add_watch(
            GLib.IOCondition.IN, self._on_gamepad_event)
        # Redraw the roll/pitch/yaw/thrust label at most 10x/second, no
        # matter how many raw stick events arrive.
        GLib.timeout_add(100, self._gamepad_display_tick)
        self.gui_log(f"[GAMEPAD] Watching {dev.path} ({dev.name!r}) via IOChannel. "
                     f"Press BASE4 to enable flight control, BASE3 to disable.")

    def _find_gamepad_device(self):
        try:
            for path in evdev.list_devices():
                d = evdev.InputDevice(path)
                if GAMEPAD_DEVICE_NAME_HINT in d.name.lower():
                    return d
        except Exception as e:
            self.gui_log(f"[GAMEPAD] Error scanning input devices: {e}")
        return None

    def _normalize_gamepad_axis(self, raw_value, axis_name):
        """Raw 0-255 (center 128) -> -1.0..+1.0, deadzone clamped to 0,
        with per-channel inversion applied (see GAMEPAD_AXIS_INVERT).
        Used for roll/pitch/yaw - throttle uses _compute_gamepad_throttle
        instead (asymmetric mapping, see below)."""
        centered = raw_value - GAMEPAD_AXIS_CENTER
        if abs(centered) <= GAMEPAD_DEADZONE_RAW:
            norm = 0.0
        else:
            span = GAMEPAD_AXIS_MAX - GAMEPAD_AXIS_CENTER
            norm = max(-1.0, min(1.0, centered / span))
        if GAMEPAD_AXIS_INVERT.get(axis_name, False):
            norm = -norm
        return norm

    def _compute_gamepad_throttle(self, raw_value):
        """
        Confirmed-working throttle mapping:
          stick at CENTER or above -> THRUST_MIN (idle, dead zone)
          stick pushed DOWN        -> ramps linearly up to THRUST_MAX

        Only the downward half of the stick's travel does anything by
        design. If the live half ends up on the wrong physical side,
        flip GAMEPAD_THROTTLE_DOWN_IS_RAW_LOW - nothing else needs to change.
        """
        center = GAMEPAD_AXIS_CENTER

        if GAMEPAD_THROTTLE_DOWN_IS_RAW_LOW:
            if raw_value >= center:
                return THRUST_MIN
            travel = center - raw_value
            max_travel = center - GAMEPAD_AXIS_MIN
        else:
            if raw_value <= center:
                return THRUST_MIN
            travel = raw_value - center
            max_travel = GAMEPAD_AXIS_MAX - center

        if max_travel <= 0:
            return THRUST_MIN

        frac = max(0.0, min(1.0, travel / max_travel))
        thrust = THRUST_MIN + frac * (THRUST_MAX - THRUST_MIN)
        # Defensive hard clamp: send_setpoint()'s thrust field is a 16-bit
        # unsigned int (0-65535).
        return int(max(0, min(65535, thrust)))

    def _on_gamepad_event(self, source, condition):
        """Called by GLib because the IOChannel has data ready - runs on
        the GTK main thread, safe to touch self.setpoint / widgets
        directly here. Must return True to keep being watched."""
        try:
            for event in self.gamepad_dev.read():
                if not self._gamepad_first_event_seen:
                    self._gamepad_first_event_seen = True
                    self.gui_log("[GAMEPAD] First event received from device - "
                                 "hardware link confirmed working.")
                if event.type == ev_ecodes.EV_ABS:
                    self._apply_gamepad_axis(event.code, event.value)
                elif event.type == ev_ecodes.EV_KEY:
                    self._handle_gamepad_button(event.code, event.value)
        except BlockingIOError:
            pass  # no more events queued right now, nothing to do
        except OSError as e:
            self.gui_log(f"[GAMEPAD] Read error ({e}) - device may have been unplugged.")
            self.gamepad_flight_enabled = False
            return False  # stop watching this fd
        return True

    def _apply_gamepad_axis(self, axis_code, raw_value):
        # Gated behind BOTH the existing arm state AND the gamepad's own
        # latched flight-enable flag - CH1-4 never reach the setpoint
        # unless the drone is armed (two-step-arm safety) and BASE4 has
        # been pressed (gamepad-specific kill switch).
        if not self.gamepad_flight_enabled or not self.drone.armed:
            # Rate-limited debug log: fires once per reason, not once per
            # axis event (which would flood the log while the stick moves).
            reason = []
            if not self.gamepad_flight_enabled:
                reason.append("gamepad_flight_enabled=False (press BASE4)")
            if not self.drone.armed:
                reason.append("armed=False (Start -> CONFIRM ARM?)")
            reason_str = " AND ".join(reason)
            if reason_str != self._gamepad_last_block_reason:
                self.gui_log(f"[GAMEPAD] Stick input received but ignored - {reason_str}")
                self._gamepad_last_block_reason = reason_str
            return
        if self._gamepad_last_block_reason is not None:
            self.gui_log("[GAMEPAD] Gates open - stick input now active.")
            self._gamepad_last_block_reason = None

        if axis_code == ev_ecodes.ABS_X:
            norm = self._normalize_gamepad_axis(raw_value, "ABS_X")
            self.setpoint["roll"] = norm * MAX_TILT
        elif axis_code == ev_ecodes.ABS_Y:
            norm = self._normalize_gamepad_axis(raw_value, "ABS_Y")
            self.setpoint["pitch"] = norm * MAX_TILT
        elif axis_code == ev_ecodes.ABS_Z:
            norm = self._normalize_gamepad_axis(raw_value, "ABS_Z")
            self.setpoint["yaw"] = norm * GAMEPAD_YAW_RATE_MAX
        elif axis_code == ev_ecodes.ABS_RZ:
            self.setpoint["thrust"] = self._compute_gamepad_throttle(raw_value)
        else:
            return  # D-pad or other axis - not used for flight control here

        # Don't redraw the GTK label on every single event - the gamepad
        # can report dozens of these per second while a stick is moving.
        # Just mark the data as changed; a separate 10Hz timer
        # (_gamepad_display_tick) does the actual drawing.
        self._gamepad_display_dirty = True

    def _handle_gamepad_button(self, code, value):
        # Rising-edge only (0 -> 1): BASE3/BASE4 are momentary on the
        # hardware (they report DOWN then immediately UP), so this is
        # what turns a one-shot press into a persisted flag flip.
        prev = self._gamepad_btn_prev.get(code, 0)
        self._gamepad_btn_prev[code] = value
        pressed_edge = (value == 1 and prev == 0)

        if not pressed_edge:
            return

        # Flash indicator - fires on every press regardless of the latch
        # state, so you can visually confirm the button is being read even
        # if you press it twice in a row (e.g. BASE4, BASE4 again).
        if code == GAMEPAD_BTN_DISABLE_FLIGHT:
            self._flash_led(self.base3_led)
        elif code == GAMEPAD_BTN_ENABLE_FLIGHT:
            self._flash_led(self.base4_led)

        if code == GAMEPAD_BTN_DISABLE_FLIGHT:
            self.gamepad_flight_enabled = False
            self.gui_log("[GAMEPAD] BASE3 pressed - flight control DISABLED "
                         "(CH1-4 ignored until BASE4 is pressed).")
            self.gamepad_status_lbl.set_text("Gamepad flight: DISABLED")
            self.gamepad_status_lbl.set_name("status_bad")
        elif code == GAMEPAD_BTN_ENABLE_FLIGHT:
            self.gamepad_flight_enabled = True
            self.gui_log("[GAMEPAD] BASE4 pressed - flight control ENABLED "
                         "(CH1-4 now active).")
            self.gamepad_status_lbl.set_text("Gamepad flight: ENABLED")
            self.gamepad_status_lbl.set_name("status_ok")

    def _flash_led(self, led_widget, duration_ms=250):
        """Lights led_widget on immediately, then reverts it after
        duration_ms - a visual confirmation that a button press was read,
        independent of whatever the persistent enable/disable state does."""
        led_widget.set_name("led_on")
        GLib.timeout_add(duration_ms, self._reset_led, led_widget)

    def _reset_led(self, led_widget):
        led_widget.set_name("led_off")
        return False  # one-shot timer

    def _gamepad_display_tick(self):
        """Runs on a 100ms GLib timer (10Hz), regardless of how fast
        gamepad events arrive. Only actually redraws if something changed
        since the last tick."""
        if self._gamepad_display_dirty:
            self._update_cmd_display()
            self._gamepad_display_dirty = False
        return True  # keep the timer running

    def _stop_gamepad(self):
        if self._gamepad_watch_id is not None:
            GLib.source_remove(self._gamepad_watch_id)
            self._gamepad_watch_id = None
        self._gamepad_channel = None  # let it be garbage collected; fd itself
                                       # is owned/closed by gamepad_dev below
        if self.gamepad_dev is not None:
            try:
                self.gamepad_dev.close()
            except Exception:
                pass
            self.gamepad_dev = None

    # ======================================================================
    # Thread-safe GUI helpers
    # ======================================================================
    def gui_log(self, msg):
        stamped = f"{datetime.now():%H:%M:%S}  {msg}"
        # Mirror to the persistent file log (thread-safe enough for a
        # line-buffered append-only text file)
        if self._log_file is not None:
            try:
                self._log_file.write(stamped + "\n")
            except OSError:
                pass

        def _append():
            buf = self.log_view.get_buffer()
            buf.insert(buf.get_end_iter(), stamped + "\n")
            self.log_view.scroll_to_iter(buf.get_end_iter(), 0, False, 0, 0)
            return False
        GLib.idle_add(_append)

    def _update_cmd_display(self):
        self.thrust_lbl.set_text(f"{self.setpoint['thrust']}")
        self.pitch_cmd_lbl.set_text(f"{self.setpoint['pitch']:.1f}°")
        self.roll_cmd_lbl.set_text(f"{self.setpoint['roll']:.1f}°")

    # ======================================================================
    # cflib / drone logic
    # ======================================================================
    def do_emergency_stop(self, drones=None):
        """Immediate motor cut - stronger than Stop. Sends the firmware's
        stop-setpoint, disarms, cancels any pending arm, resets the
        setpoint, and disables gamepad flight. Safe to press at any time,
        connected or not.

        Kept parameterized over `drones` (defaulting to self.drones, a
        one-item list right now) so a second drone can be reintroduced
        later without touching this method again."""
        targets = drones if drones is not None else self.drones
        is_global = drones is None

        if is_global:
            self.gamepad_flight_enabled = False
            self.gamepad_status_lbl.set_text("Gamepad flight: DISABLED")
            self.gamepad_status_lbl.set_name("status_bad")
            self.setpoint["roll"] = 0.0
            self.setpoint["pitch"] = 0.0
            self.setpoint["thrust"] = THRUST_MIN
            self._update_cmd_display()

        any_sent = False
        for d in targets:
            if d.estop():
                any_sent = True

        names = ", ".join(d.name for d in targets)
        if any_sent:
            self.gui_log(f"*** EMERGENCY STOP ({names}) - stop-setpoint sent, motors cut. ***")
        else:
            self.gui_log(f"*** EMERGENCY STOP pressed ({names}) - not connected, nothing sent. ***")

    def adjust_roll(self, delta):
        self.setpoint["roll"] = max(-MAX_TILT, min(MAX_TILT, self.setpoint["roll"] + delta))
        self._update_cmd_display()

    def adjust_pitch(self, delta):
        self.setpoint["pitch"] = max(-MAX_TILT, min(MAX_TILT, self.setpoint["pitch"] + delta))
        self._update_cmd_display()

    def adjust_thrust(self, delta):
        self.setpoint["thrust"] = max(THRUST_MIN, min(THRUST_MAX, self.setpoint["thrust"] + delta))
        self._update_cmd_display()

    def center_attitude(self):
        self.setpoint["roll"] = 0.0
        self.setpoint["pitch"] = 0.0
        self.gui_log("Roll/pitch centered to level.")
        self._update_cmd_display()

    # ======================================================================
    # Shutdown
    # ======================================================================
    def _on_destroy(self, *args):
        self.gamepad_flight_enabled = False
        self._stop_gamepad()
        for d in self.drones:
            d.armed = False
            d._cancel_arm_confirm()
            d.stop_stream_evt.set()
        time.sleep(0.2)
        for d in self.drones:
            d.stop_telemetry()
            d.close_link()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
        Gtk.main_quit()


def main():
    global DRONE_URI
    parser = argparse.ArgumentParser(description="LiteWing drone control (no video)")
    parser.add_argument("--drone", default=DRONE_URI,
                        help=f"Drone link URI (default: {DRONE_URI})")
    args = parser.parse_args()
    DRONE_URI = args.drone

    app = DroneControlApp()
    Gtk.main()


if __name__ == "__main__":
    main()