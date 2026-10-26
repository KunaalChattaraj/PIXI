#!/usr/bin/env python3
"""
LiteWing Motor Test — GTK
=========================
Bench tool: spin ONE motor at a time (or all four) over the normal WiFi/UDP
link, to check wiring, motor order and spin direction. Same structure as
lightwing_gtk_drone_only.py (GTK window, DroneLink class, cflib callbacks
marshaled with GLib.idle_add, keepalive thread, E-STOP).

How it works
------------
The firmware has a manual motor override in power_distribution_stock.c:

    motorPowerSet.enable   (uint8)    1 = ignore the mixer, use m1..m4 below
    motorPowerSet.m1..m4   (uint16)   0..65535 for each motor

This app only writes those parameters. It does not fly the drone: no
stabilisation is involved, the motor simply gets the number you set.

    stabilizer.stop        (uint8)    1 = firmware emergency stop (all motors 0)

is used for E-STOP, because the normal stop-setpoint does NOT stop the
override.

SAFETY — read before use
------------------------
  * REMOVE ALL PROPELLERS. Fix the frame to the bench.
  * The override has NO watchdog in the firmware. If WiFi drops while a
    motor is running, the motor keeps running until power is removed.
    Keep the battery connector within reach.
  * Throttle is capped (TEST_PERCENT_MAX) and every run stops by itself
    after the chosen number of seconds.
  * After every stop the app checks the firmware's own telemetry and logs
    "Stop confirmed" or "STOP NOT CONFIRMED". Read that line.
  * Keep the drone level and the right way up. If it is upside down the
    firmware's tumble protection stops all motors (press 'Reset E-STOP').

Threads/contexts:
  1. GTK/GLib main loop       - GUI, timers, all parameter writes
  2. cflib background thread  - telemetry + connection callbacks
                                 (marshaled to GUI via GLib.idle_add)
  3. persistent_stream thread - zero setpoint every 100ms (link keepalive)

Usage:
  python3 litewing_motor_test_gtk.py --drone udp://192.168.43.42:2390
"""

import os
import time
import argparse
import threading
from datetime import datetime

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, Gdk

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

cflib.crtp.init_drivers()

# ── Config - defaults, overridable from the command line ────────────────────
DRONE_URI = "udp://192.168.43.42:2390"  # Custom Drone (made by us)

NBR_OF_MOTORS = 4
# Default LiteWing pins (main/Kconfig.projbuild). Only a hint for the bench -
# change the text if your board is wired differently.
MOTOR_PIN_HINT = "M1=GPIO5   M2=GPIO6   M3=GPIO3   M4=GPIO4"

# Test throttle, in percent of full scale (0-65535).
TEST_PERCENT_DEFAULT = 10
TEST_PERCENT_MIN = 1
TEST_PERCENT_MAX = 30   # hard cap for bench testing - raise only on purpose
TEST_PERCENT_STEP = 1

# Every run stops by itself after this many seconds.
RUN_SECONDS_DEFAULT = 2
RUN_SECONDS_MIN = 1
RUN_SECONDS_MAX = 10

# How long to wait for cflib to finish reading the parameter list before
# warning in the log (the motor buttons stay disabled until it is done).
PARAM_WAIT_WARN_S = 20

# After a stop is sent, wait this long, then check in the telemetry that the
# firmware really has all four motors at 0.
STOP_CONFIRM_MS = 1000
# After a run starts, wait this long, then check in the telemetry that the
# firmware is really driving the chosen motor.
RUN_CONFIRM_MS = 700
# During a run: warn if no telemetry has arrived for this long (link down?).
TELEMETRY_STALE_S = 1.5

BATTERY_LOW_V = 3.3
LOG_DIR = os.path.expanduser("~/litewing_logs")

# Firmware parameter names (power_distribution_stock.c / stabilizer.c)
PARAM_ENABLE = "motorPowerSet.enable"
PARAM_MOTORS = ["motorPowerSet.m1", "motorPowerSet.m2",
                "motorPowerSet.m3", "motorPowerSet.m4"]
PARAM_FW_STOP = "stabilizer.stop"

# Same conversion as motorsConv16ToDshot() in the DShot motors.c - used for
# display only, the firmware does the real conversion.
DSHOT_THROTTLE_MIN = 48
DSHOT_THROTTLE_MAX = 2047


def percent_to_ratio(percent):
    """Percent of full scale -> the 0..65535 number the firmware expects."""
    return int(round(max(0.0, min(100.0, percent)) / 100.0 * 65535))


def ratio_to_dshot(ratio):
    """0..65535 -> value inside the DShot frame (0 = stop, else 48..2047)."""
    if ratio <= 0:
        return 0
    return DSHOT_THROTTLE_MIN + (ratio * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) // 65535


# ── CSS styling ──────────────────────────────────────────────────────────────
CSS = b"""
* { font-family: monospace; }
window { background-color: #0a0a0a; }
#title_label { color: #00ff88; font-size: 16px; font-weight: bold; padding: 8px 0px 2px 0px; }
#section_label { color: #00ff88; font-size: 11px; font-weight: bold; padding: 4px 0px; }
#status_ok { color: #00ff88; font-size: 11px; font-weight: bold; }
#status_bad { color: #ff4444; font-size: 11px; font-weight: bold; }
#status_off { color: #557755; font-size: 11px; }
#plain { color: #cfd8cf; font-size: 11px; }
#readout { color: #00ff88; font-size: 13px; font-weight: bold; }
#readout_warn { color: #ff4444; font-size: 13px; font-weight: bold; }
#motor_out { color: #00ff88; font-size: 11px; font-weight: bold; }
#motor_out_live { color: #ffcc00; font-size: 11px; font-weight: bold; }
#log_view, #log_view text { background-color: #101410; color: #66cc88; font-family: monospace; font-size: 10px; }
button { font-family: monospace; }
checkbutton { color: #ffcc00; font-size: 11px; font-weight: bold; }
#control_panel { background-color: rgba(10, 10, 10, 0.92); }
#estop_btn { background-image: none; background-color: #3a0808; color: #ff4444; border: 2px solid #cc2222;
             font-size: 14px; font-weight: bold; padding: 10px 0px; }
#motor_btn { font-size: 13px; font-weight: bold; padding: 10px 0px; }
#all_btn { font-size: 13px; font-weight: bold; padding: 8px 0px; }
"""


class DroneLink:
    """Owns ONE drone, end to end: Crazyflie connection, telemetry,
    keepalive thread and the motor-override parameter writes."""

    def __init__(self, app, name, uri):
        self.app = app
        self.name = name
        self.uri = uri

        self.cf = None
        self.connected = False
        # True once cflib has the parameter list AND the override
        # parameters were found in it. Motor buttons need this.
        self.params_ready = False
        self.fw_stop_available = False
        self.estop_latched = False
        self.log_confs = []
        # True if the firmware gave us pwm.m1_pwm..m4_pwm (needed for the
        # "did it really start / stop" checks).
        self.motor_telemetry = False
        self.stop_stream_evt = threading.Event()

        self._param_wait_ticks = 0
        self._param_wait_id = None

    # ── Connection ───────────────────────────────────────────────────────
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
        self.app.connect_btn.set_sensitive(False)
        self.app.connect_btn.set_label("Connecting...")
        self.cf.open_link(self.uri)

    def persistent_stream(self):
        """Zero setpoint, 10Hz. Not needed for the override itself - it
        keeps the link traffic identical to the flight app, and guarantees
        the normal (non-override) path is commanding zero thrust."""
        consecutive_errors = 0
        while self.connected and not self.stop_stream_evt.is_set():
            try:
                self.cf.commander.send_setpoint(0, 0, 0, 0)
                if consecutive_errors:
                    self.app.gui_log(f"[{self.name} STREAM] Link recovered.")
                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors == 1:
                    self.app.gui_log(f"[{self.name} STREAM ERROR] send_setpoint failed: {e} - retrying...")
                elif consecutive_errors == 10:
                    self.app.gui_log(f"[{self.name} STREAM ERROR] 10 consecutive send failures - link "
                                      f"likely down. The override has NO watchdog: if a motor is "
                                      f"still running, disconnect the battery.")
            time.sleep(0.1)
        self.app.gui_log(f"[{self.name} STREAM] Keepalive stopped.")

    # ── cflib callbacks (fire on cflib's own background thread) ──────────
    def _on_connected(self, link_uri):
        def _apply():
            self.connected = True
            self.params_ready = False
            self.estop_latched = False
            self.app.gui_log(f"[OK] {self.name} connected: {link_uri}")
            self.app.set_status("Connected - reading parameters...", ok=True)
            self.app.connect_btn.set_sensitive(False)
            self.app.connect_btn.set_label("Connected")
            self.start_telemetry()
            self.stop_stream_evt.clear()
            threading.Thread(target=self.persistent_stream, daemon=True).start()
            # cflib needs the full parameter list before set_value() can be
            # used without blocking. Poll for that from the GUI thread.
            self._param_wait_ticks = 0
            self._param_wait_id = GLib.timeout_add(200, self._wait_params_tick)
            self.app.refresh_controls()
            return False
        GLib.idle_add(_apply)

    def _on_connection_failed(self, link_uri, msg):
        def _apply():
            self.connected = False
            self.params_ready = False
            self.app.gui_log(f"[FAIL] {self.name}: {msg}")
            self.app.set_status("Connection failed", ok=False)
            self.app.connect_btn.set_sensitive(True)
            self.app.connect_btn.set_label("Connect")
            self.app.refresh_controls()
            return False
        GLib.idle_add(_apply)

    def _on_disconnected(self, link_uri):
        def _apply():
            was_running = self.app.is_running()
            self.connected = False
            self.params_ready = False
            self.stop_stream_evt.set()
            self._cancel_param_wait()
            self.app.gui_log(f"[INFO] {self.name} disconnected: {link_uri}")
            if was_running:
                self.app.gui_log("*** LINK LOST WHILE A MOTOR TEST WAS RUNNING - the stop command "
                                 "could not be sent. DISCONNECT THE BATTERY NOW. ***")
            self.app.set_status("Disconnected", ok=False)
            self.app.connect_btn.set_sensitive(True)
            self.app.connect_btn.set_label("Connect")
            self.stop_telemetry()
            self.app.on_link_lost()
            return False
        GLib.idle_add(_apply)

    # ── Parameter readiness ──────────────────────────────────────────────
    def _cflib_params_usable(self):
        """Newer cflib: set_value() blocks until every parameter has been
        read once (param._initialized). Older cflib has no such gate and
        is usable as soon as the link reports connected."""
        gate = getattr(self.cf.param, "_initialized", None)
        if gate is not None:
            return gate.is_set()
        return True

    def _wait_params_tick(self):
        if not self.connected:
            self._param_wait_id = None
            return False
        if self._cflib_params_usable():
            self._param_wait_id = None
            self._on_params_usable()
            return False
        self._param_wait_ticks += 1
        if self._param_wait_ticks == int(PARAM_WAIT_WARN_S / 0.2):
            self.app.gui_log(f"[WARN] {self.name}: still reading parameters after "
                              f"{PARAM_WAIT_WARN_S}s - motor buttons stay disabled until this finishes. "
                              f"Check the WiFi link.")
        return True

    def _cancel_param_wait(self):
        if self._param_wait_id is not None:
            GLib.source_remove(self._param_wait_id)
            self._param_wait_id = None

    def _has_param(self, name):
        try:
            return self.cf.param.toc.get_element_by_complete_name(name) is not None
        except Exception:
            return False

    def _on_params_usable(self):
        missing = [p for p in [PARAM_ENABLE] + PARAM_MOTORS if not self._has_param(p)]
        if missing:
            self.app.gui_log(f"[ERROR] {self.name}: firmware has no {', '.join(missing)} - "
                              f"motor test is not possible with this firmware.")
            self.app.set_status("Connected - override parameters missing", ok=False)
            self.app.refresh_controls()
            return

        self.fw_stop_available = self._has_param(PARAM_FW_STOP)
        if not self.fw_stop_available:
            self.app.gui_log(f"[WARN] {self.name}: {PARAM_FW_STOP} not found - E-STOP will only "
                              f"zero the override.")

        try:
            self.cf.param.add_update_callback(group="motorPowerSet", name="enable",
                                              cb=self._on_enable_update)
        except Exception as e:
            self.app.gui_log(f"[WARN] {self.name}: could not watch {PARAM_ENABLE} ({e}).")

        # Known-safe starting point, in case an earlier session left the
        # override switched on.
        self.params_ready = True
        self.stop_motor_override()
        self.app.set_status("Connected - ready", ok=True)
        self.app.gui_log(f"[{self.name}] Parameters ready - override forced OFF, all motors 0.")
        self.app.refresh_controls()

    def _on_enable_update(self, name, value):
        """Firmware's own answer after a write to motorPowerSet.enable."""
        try:
            on = int(float(value)) != 0
        except (TypeError, ValueError):
            return

        def _apply():
            self.app.set_override_display(on)
            return False
        GLib.idle_add(_apply)

    # ── Parameter writes (GUI thread only) ───────────────────────────────
    def _set_param(self, name, value):
        """Returns True if the write was handed to cflib. The value is sent
        as a string: every cflib version accepts that."""
        if not (self.connected and self.params_ready and self.cf is not None):
            return False
        try:
            self.cf.param.set_value(name, str(int(value)))
            return True
        except Exception as e:
            self.app.gui_log(f"[{self.name}] PARAM WRITE FAILED {name}={value}: {e}")
            return False

    def set_motor_override(self, ratios):
        """ratios: list of four 0..65535 values. Motors going to 0 are
        written first, so switching from one motor to another never has
        both spinning on purpose. Enable is written last."""
        if self.estop_latched:
            return False
        order = sorted(range(NBR_OF_MOTORS), key=lambda i: ratios[i] != 0)
        ok = True
        for i in order:
            ok = self._set_param(PARAM_MOTORS[i], ratios[i]) and ok
        if not ok:
            self.stop_motor_override()
            return False
        return self._set_param(PARAM_ENABLE, 1)

    def stop_motor_override(self):
        """All four values to 0, then the override off."""
        ok = True
        for name in PARAM_MOTORS:
            ok = self._set_param(name, 0) and ok
        ok = self._set_param(PARAM_ENABLE, 0) and ok
        return ok

    def estop(self):
        """Firmware emergency stop first (one packet, cuts all motors in
        the stabilizer loop), then clear the override. Latches until
        reset_estop()."""
        if not self.connected:
            return False
        sent = False
        if self.fw_stop_available:
            sent = self._set_param(PARAM_FW_STOP, 1)
        sent = self.stop_motor_override() or sent
        self.estop_latched = True
        return sent

    def reset_estop(self):
        ok = self.stop_motor_override()
        if self.fw_stop_available:
            ok = self._set_param(PARAM_FW_STOP, 0) and ok
        if ok:
            self.estop_latched = False
        return ok

    # ── Telemetry ────────────────────────────────────────────────────────
    def start_telemetry(self):
        """Two independent blocks, so a firmware without one of the
        variables still shows the rest:
          MotorLog - what the firmware is sending to each motor + battery
          DShotLog - pwm.dshotErr (only exists on the DShot firmware)"""
        self.log_confs = []
        self.motor_telemetry = False

        def motor_cb(timestamp, data, logconf):
            ratios = [data.get(f"pwm.m{i + 1}_pwm", 0) for i in range(NBR_OF_MOTORS)]
            vbat = data.get("pm.vbat")
            GLib.idle_add(self.app.set_motor_display, ratios, vbat)

        def dshot_cb(timestamp, data, logconf):
            GLib.idle_add(self.app.set_dshot_err_display, data.get("pwm.dshotErr", 0))

        def error_cb(logconf, msg):
            self.app.gui_log(f"[LOG ERROR] {self.name} {logconf.name}: {msg}")

        motor_vars = [(f"pwm.m{i + 1}_pwm", "uint32_t") for i in range(NBR_OF_MOTORS)]
        blocks = [
            ("MotorLog", 100, motor_vars + [("pm.vbat", "float")], motor_cb),
            ("MotorLogNoBat", 100, motor_vars, motor_cb),   # fallback if pm.vbat is missing
            ("DShotLog", 500, [("pwm.dshotErr", "uint32_t")], dshot_cb),
        ]

        motor_log_started = False
        for block_name, period_ms, variables, cb in blocks:
            if block_name == "MotorLogNoBat" and motor_log_started:
                continue
            lc = LogConfig(name=f"{block_name}_{self.name}", period_in_ms=period_ms)
            try:
                for var, vtype in variables:
                    lc.add_variable(var, vtype)
                lc.data_received_cb.add_callback(cb)
                lc.error_cb.add_callback(error_cb)
                self.cf.log.add_config(lc)
                lc.start()
            except (KeyError, AttributeError) as e:
                self.app.gui_log(f"[WARN] {self.name}: telemetry block '{block_name}' unavailable ({e}) - skipped.")
                continue
            self.log_confs.append(lc)
            if block_name == "MotorLog":
                motor_log_started = True
            if block_name in ("MotorLog", "MotorLogNoBat"):
                self.motor_telemetry = True

        if not self.motor_telemetry:
            self.app.gui_log(f"[WARN] {self.name}: no motor telemetry (pwm.m1_pwm..m4_pwm) - the app "
                              f"cannot confirm that a motor started or stopped. Watch the motors.")

        if self.log_confs:
            self.app.gui_log(f"[{self.name}] Telemetry started - {len(self.log_confs)} block(s).")

    def stop_telemetry(self):
        for lc in self.log_confs:
            try:
                lc.stop()
            except Exception:
                pass
        self.log_confs = []
        self.motor_telemetry = False
        GLib.idle_add(self.app.clear_displays)

    def close_link(self):
        self.stop_stream_evt.set()
        self._cancel_param_wait()
        if self.cf is not None:
            try:
                self.cf.close_link()
            except Exception:
                pass


class MotorTestApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="LiteWing Motor Test")
        self.set_default_size(940, 720)

        self.drone = DroneLink(self, "drone", DRONE_URI)

        # ── Test state ────────────────────────────────────────────────────
        self.test_percent = TEST_PERCENT_DEFAULT
        self.run_seconds = RUN_SECONDS_DEFAULT
        self._running_motors = None   # list of motor indices while a run is active
        self._run_timer_id = None
        self._run_seq = 0             # counts runs, so a late check knows its run is over

        # ── What the firmware last reported (MotorLog telemetry) ───────────
        self._last_ratios = [0] * NBR_OF_MOTORS
        self._last_telemetry_t = None   # time.monotonic() of the last packet
        self._stale_warned = False

        # ── Persistent file log ───────────────────────────────────────────
        self._log_file = None
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            log_path = os.path.join(
                LOG_DIR, f"motortest_{datetime.now():%Y%m%d_%H%M%S}.log")
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
        self.connect("key-press-event", self._on_key_press)
        self.refresh_controls()
        GLib.timeout_add(500, self._link_watch_tick)

    # ======================================================================
    # UI
    # ======================================================================
    def _build_ui(self):
        # Two columns: controls on the left, log on the right. The log is
        # always visible, because the stop confirmation is printed there.
        root = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        self.add(root)

        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.set_size_request(450, -1)
        root.pack_start(scroll, False, False, 0)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.set_name("control_panel")
        left.set_margin_start(14)
        left.set_margin_end(14)
        left.set_margin_top(14)
        left.set_margin_bottom(14)

        title = Gtk.Label(label="◈ LiteWing Motor Test")
        title.set_name("title_label")
        left.pack_start(title, False, False, 0)

        self.uri_lbl = Gtk.Label(label=f"Target: {DRONE_URI}")
        self.uri_lbl.set_name("plain")
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

        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.connect_btn = Gtk.Button(label="Connect")
        self.connect_btn.connect("clicked", lambda *_: self.drone.do_connect())
        self.reset_estop_btn = Gtk.Button(label="Reset E-STOP")
        self.reset_estop_btn.connect("clicked", lambda *_: self.do_reset_estop())
        btn_row.pack_start(self.connect_btn, True, True, 0)
        btn_row.pack_start(self.reset_estop_btn, True, True, 0)
        left.pack_start(btn_row, False, False, 4)

        # ── Safety gate ───────────────────────────────────────────────────
        self.props_check = Gtk.CheckButton(label="Propellers are REMOVED (tick to unlock the motor buttons)")
        self.props_check.connect("toggled", lambda *_: self._on_props_toggled())
        left.pack_start(self.props_check, False, False, 6)

        # ── Test settings ─────────────────────────────────────────────────
        settings_label = Gtk.Label(label="TEST SETTINGS")
        settings_label.set_name("section_label")
        settings_label.set_xalign(0)
        left.pack_start(settings_label, False, False, 6)

        self.percent_lbl = Gtk.Label(label="")
        self._add_adjust_row(left, "Throttle", self.percent_lbl,
                              lambda: self.adjust_percent(-TEST_PERCENT_STEP),
                              lambda: self.adjust_percent(TEST_PERCENT_STEP))

        self.seconds_lbl = Gtk.Label(label="")
        self._add_adjust_row(left, "Run time", self.seconds_lbl,
                              lambda: self.adjust_seconds(-1),
                              lambda: self.adjust_seconds(1))

        cap_hint = Gtk.Label(label=f"Throttle cap {TEST_PERCENT_MAX}%  -  each run stops by itself")
        cap_hint.set_name("status_off")
        cap_hint.set_xalign(0)
        left.pack_start(cap_hint, False, False, 0)

        # ── Motor buttons ─────────────────────────────────────────────────
        motors_label = Gtk.Label(label="RUN ONE MOTOR")
        motors_label.set_name("section_label")
        motors_label.set_xalign(0)
        left.pack_start(motors_label, False, False, 8)

        grid = Gtk.Grid()
        grid.set_column_spacing(8)
        grid.set_row_spacing(8)
        grid.set_column_homogeneous(True)
        self.motor_btns = []
        self.motor_out_lbls = []
        for i in range(NBR_OF_MOTORS):
            btn = Gtk.Button(label=f"Motor {i + 1}")
            btn.set_name("motor_btn")
            btn.connect("clicked", lambda _w, idx=i: self.run_test([idx]))
            # Live value reported by the firmware for this motor, right
            # under its button.
            out = Gtk.Label(label="--")
            out.set_name("motor_out")
            cell = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            cell.pack_start(btn, False, False, 0)
            cell.pack_start(out, False, False, 0)
            grid.attach(cell, i % 2, i // 2, 1, 1)
            self.motor_btns.append(btn)
            self.motor_out_lbls.append(out)
        left.pack_start(grid, False, False, 0)

        pin_hint = Gtk.Label(label=MOTOR_PIN_HINT)
        pin_hint.set_name("status_off")
        pin_hint.set_xalign(0)
        left.pack_start(pin_hint, False, False, 0)

        # ── All four together / Stop ──────────────────────────────────────
        all_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        all_row.set_homogeneous(True)
        self.all_btn = Gtk.Button(label="All 4 motors")
        self.all_btn.set_name("all_btn")
        self.all_btn.connect("clicked", lambda *_: self.run_test(list(range(NBR_OF_MOTORS))))
        all_row.pack_start(self.all_btn, True, True, 0)

        self.stop_btn = Gtk.Button(label="Stop (S / Esc)")
        self.stop_btn.set_name("all_btn")
        self.stop_btn.connect("clicked", lambda *_: self.stop_test("Stop pressed"))
        all_row.pack_start(self.stop_btn, True, True, 0)
        left.pack_start(all_row, False, False, 8)

        self.run_status_lbl = Gtk.Label(label="Idle")
        self.run_status_lbl.set_name("status_off")
        self.run_status_lbl.set_xalign(0)
        left.pack_start(self.run_status_lbl, False, False, 2)

        scroll.add(left)

        # ── Right column: firmware read-outs + log ────────────────────────
        right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        right.set_margin_start(6)
        right.set_margin_end(14)
        right.set_margin_top(14)
        right.set_margin_bottom(14)

        telem_label = Gtk.Label(label="FROM FIRMWARE")
        telem_label.set_name("section_label")
        telem_label.set_xalign(0)
        right.pack_start(telem_label, False, False, 0)

        fw_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=14)
        self.override_lbl = Gtk.Label(label="Override: --")
        self.override_lbl.set_name("readout")
        self.override_lbl.set_xalign(0)
        fw_row.pack_start(self.override_lbl, False, False, 0)

        self.dshot_err_lbl = Gtk.Label(label="DShot err: --")
        self.dshot_err_lbl.set_name("readout")
        self.dshot_err_lbl.set_xalign(0)
        fw_row.pack_start(self.dshot_err_lbl, False, False, 0)

        self.battery_lbl = Gtk.Label(label="Batt: --")
        self.battery_lbl.set_name("readout")
        self.battery_lbl.set_xalign(0)
        fw_row.pack_start(self.battery_lbl, False, False, 0)
        right.pack_start(fw_row, False, False, 0)

        log_label = Gtk.Label(label="LOG")
        log_label.set_name("section_label")
        log_label.set_xalign(0)
        right.pack_start(log_label, False, False, 0)

        self.log_view = Gtk.TextView()
        self.log_view.set_name("log_view")
        self.log_view.set_editable(False)
        self.log_view.set_cursor_visible(False)
        self.log_view.set_can_focus(False)   # keeps Space / S / Esc for the stop keys
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD)
        log_scroll = Gtk.ScrolledWindow()
        log_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        log_scroll.add(self.log_view)
        right.pack_start(log_scroll, True, True, 0)
        root.pack_start(right, True, True, 0)
        self._update_settings_display()
        self.show_all()

    def _add_adjust_row(self, parent, label_text, value_label, on_minus, on_plus):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        lbl = Gtk.Label(label=f"{label_text}:")
        lbl.set_name("plain")
        lbl.set_xalign(0)
        lbl.set_size_request(80, -1)
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
    # Keyboard: only STOP actions are bound to keys, never "run"
    # ======================================================================
    def _on_key_press(self, widget, event):
        key = event.keyval
        if key == Gdk.KEY_space:
            self.do_emergency_stop()
            return True
        if key in (Gdk.KEY_Escape, Gdk.KEY_s, Gdk.KEY_S):
            self.stop_test("Stop key pressed")
            return True
        return False

    # ======================================================================
    # Thread-safe GUI helpers
    # ======================================================================
    def gui_log(self, msg):
        stamped = f"{datetime.now():%H:%M:%S}  {msg}"
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

    def set_status(self, text, ok):
        self.status_lbl.set_text(text)
        self.status_lbl.set_name("status_ok" if ok else "status_bad")

    def set_motor_display(self, ratios, vbat):
        """One MotorLog packet from the firmware (GUI thread)."""
        self._last_ratios = list(ratios)
        self._last_telemetry_t = time.monotonic()
        if self._stale_warned:
            self._stale_warned = False
            self.gui_log("[LINK] Telemetry is back.")
        for i, ratio in enumerate(ratios):
            percent = ratio / 65535.0 * 100.0
            self.motor_out_lbls[i].set_text(
                f"{ratio} · {percent:.1f}% · DShot {ratio_to_dshot(ratio)}")
            self.motor_out_lbls[i].set_name("motor_out_live" if ratio > 0 else "motor_out")
        if vbat is not None:
            self.battery_lbl.set_text(f"Batt: {vbat:.2f} V")
            self.battery_lbl.set_name("readout_warn" if vbat < BATTERY_LOW_V else "readout")
        return False

    def set_dshot_err_display(self, count):
        self.dshot_err_lbl.set_text(f"DShot err: {count}")
        self.dshot_err_lbl.set_name("readout_warn" if count else "readout")
        return False

    def set_override_display(self, on):
        self.override_lbl.set_text(f"Override: {'ON' if on else 'OFF'}")

    def clear_displays(self):
        self._last_ratios = [0] * NBR_OF_MOTORS
        self._last_telemetry_t = None
        for lbl in self.motor_out_lbls:
            lbl.set_text("--")
            lbl.set_name("motor_out")
        self.override_lbl.set_text("Override: --")
        self.dshot_err_lbl.set_text("DShot err: --")
        self.dshot_err_lbl.set_name("readout")
        self.battery_lbl.set_text("Batt: --")
        self.battery_lbl.set_name("readout")
        return False

    def _update_settings_display(self):
        ratio = percent_to_ratio(self.test_percent)
        self.percent_lbl.set_text(f"{self.test_percent}%  (ratio {ratio}, DShot {ratio_to_dshot(ratio)})")
        self.seconds_lbl.set_text(f"{self.run_seconds} s")

    def refresh_controls(self):
        """One place that decides which buttons are clickable."""
        can_run = (self.drone.connected and self.drone.params_ready
                   and self.props_check.get_active() and not self.drone.estop_latched)
        for btn in self.motor_btns:
            btn.set_sensitive(can_run)
        self.all_btn.set_sensitive(can_run)
        self.stop_btn.set_sensitive(self.drone.connected and self.drone.params_ready)
        # Always available once ready: the firmware can also latch its own
        # emergency stop (tumble protection), which the app cannot see.
        self.reset_estop_btn.set_sensitive(self.drone.connected and self.drone.params_ready)

    # ======================================================================
    # Test logic
    # ======================================================================
    def is_running(self):
        return self._running_motors is not None

    def _on_props_toggled(self):
        if not self.props_check.get_active() and self.is_running():
            self.stop_test("Safety box un-ticked")
        self.refresh_controls()

    def adjust_percent(self, delta):
        self.test_percent = max(TEST_PERCENT_MIN, min(TEST_PERCENT_MAX, self.test_percent + delta))
        self._update_settings_display()
        # Changing the throttle during a run applies at once - handy for
        # finding the lowest value at which a motor starts.
        if self.is_running():
            self._send_running_values()

    def adjust_seconds(self, delta):
        self.run_seconds = max(RUN_SECONDS_MIN, min(RUN_SECONDS_MAX, self.run_seconds + delta))
        self._update_settings_display()

    def _send_running_values(self):
        ratio = percent_to_ratio(self.test_percent)
        ratios = [ratio if i in self._running_motors else 0 for i in range(NBR_OF_MOTORS)]
        if not self.drone.set_motor_override(ratios):
            self._finish_run()
            self.gui_log("[TEST] Could not send the motor values - test aborted.")
            return False
        names = "+".join(f"M{i + 1}" for i in self._running_motors)
        self._set_run_status(f"RUNNING {names} at {self.test_percent}% for {self.run_seconds}s", "status_ok")
        return True

    def run_test(self, motors):
        if not (self.drone.connected and self.drone.params_ready):
            self.gui_log("[TEST] Not ready - connect first and wait for 'ready'.")
            return
        if self.drone.estop_latched:
            self.gui_log("[TEST] E-STOP is latched - press 'Reset E-STOP' first.")
            return
        if not self.props_check.get_active():
            self.gui_log("[TEST] Tick 'Propellers are REMOVED' first.")
            return

        self._cancel_run_timer()
        self._running_motors = list(motors)
        if not self._send_running_values():
            return
        ratio = percent_to_ratio(self.test_percent)
        names = "+".join(f"M{i + 1}" for i in motors)
        self.gui_log(f"[TEST] {names} -> {self.test_percent}% (ratio {ratio}, DShot "
                     f"{ratio_to_dshot(ratio)}) for {self.run_seconds}s")
        self._run_timer_id = GLib.timeout_add(self.run_seconds * 1000, self._run_expired)
        self._run_seq += 1
        GLib.timeout_add(RUN_CONFIRM_MS, self._verify_running, self._run_seq, time.monotonic())

    def _verify_running(self, seq, t_start):
        """Did the firmware really start driving the chosen motor(s)?"""
        if seq != self._run_seq or not self.is_running() or not self.drone.connected:
            return False  # that run is already over
        if not self.drone.motor_telemetry:
            return False
        if self._last_telemetry_t is None or self._last_telemetry_t <= t_start:
            self.gui_log("[WARN] No telemetry since the run started - cannot confirm the motor "
                         "command arrived. Check the WiFi link.")
        elif all(self._last_ratios[i] == 0 for i in self._running_motors):
            self.gui_log("[WARN] Firmware reports 0 for the motor under test - it is NOT driving it. "
                         "Most likely the firmware emergency stop is active (drone tilted or upside "
                         "down = tumble protection). Put the drone level and press 'Reset E-STOP'.")
        return False  # one-shot

    def _run_expired(self):
        self._run_timer_id = None
        self.stop_test("run time over")
        return False  # one-shot

    def _cancel_run_timer(self):
        if self._run_timer_id is not None:
            GLib.source_remove(self._run_timer_id)
            self._run_timer_id = None

    def _set_run_status(self, text, style):
        self.run_status_lbl.set_text(text)
        self.run_status_lbl.set_name(style)

    def _finish_run(self):
        self._cancel_run_timer()
        self._running_motors = None
        self._set_run_status("Idle", "status_off")

    def stop_test(self, reason):
        """Motors to 0 and override off. Safe to call at any time."""
        self._finish_run()
        if not (self.drone.connected and self.drone.params_ready):
            return
        if self.drone.stop_motor_override():
            self.gui_log(f"[TEST] Stopped ({reason}) - stop sent, checking with the firmware...")
            self._schedule_stop_check()
        else:
            self.gui_log(f"*** [TEST] Stopped ({reason}) - STOP COULD NOT BE SENT. If a motor is "
                         f"still spinning, DISCONNECT THE BATTERY. ***")
            self._set_run_status("STOP NOT SENT - check the motors", "status_bad")

    # ── "Did the motors really stop?" - answered by the firmware, not by us ──
    def _schedule_stop_check(self, attempt=1):
        if not self.drone.motor_telemetry:
            self.gui_log("[WARN] No motor telemetry - stop cannot be confirmed. Look at the motors.")
            return
        GLib.timeout_add(STOP_CONFIRM_MS, self._verify_stopped, time.monotonic(), attempt)

    def _verify_stopped(self, t_stop, attempt):
        if not self.drone.connected or self.is_running():
            return False  # link-loss path logs its own warning / a new run has started
        if self._last_telemetry_t is None or self._last_telemetry_t <= t_stop:
            self.gui_log("*** STOP NOT CONFIRMED - no telemetry from the drone since the stop "
                         "command. If a motor is still spinning, DISCONNECT THE BATTERY. ***")
            self._set_run_status("STOP NOT CONFIRMED - check the motors", "status_bad")
        elif any(r > 0 for r in self._last_ratios):
            if attempt == 1:
                self.gui_log("[WARN] Firmware still reports motor output after the stop - sending stop again.")
                self.drone.stop_motor_override()
                self._schedule_stop_check(attempt=2)
            else:
                self.gui_log("*** STOP NOT CONFIRMED - firmware still reports motor output "
                             f"{self._last_ratios}. DISCONNECT THE BATTERY. ***")
                self._set_run_status("STOP NOT CONFIRMED - disconnect the battery", "status_bad")
        else:
            self.gui_log("[TEST] Stop confirmed by firmware - all four motors at 0.")
            self._set_run_status("Idle - stop confirmed by firmware", "status_ok")
        return False  # one-shot

    def _link_watch_tick(self):
        """Every 500ms: while a motor is running, telemetry must keep
        arriving. If it stops, the stop command may not get through either."""
        if (self.drone.connected and self.is_running() and not self._stale_warned
                and self._last_telemetry_t is not None
                and time.monotonic() - self._last_telemetry_t > TELEMETRY_STALE_S):
            self._stale_warned = True
            self.gui_log(f"*** NO TELEMETRY for {TELEMETRY_STALE_S}s WHILE A MOTOR IS RUNNING - the "
                         f"WiFi link may be down. Be ready to DISCONNECT THE BATTERY. ***")
        return True  # keep the timer running

    def on_link_lost(self):
        self._finish_run()
        self.refresh_controls()

    def do_emergency_stop(self):
        """Stronger than Stop: firmware emergency stop + override cleared.
        Latches - the motor buttons stay locked until 'Reset E-STOP'."""
        self._finish_run()
        if self.drone.estop():
            self.gui_log("*** EMERGENCY STOP - firmware stop sent, override cleared. "
                         "Press 'Reset E-STOP' to test again. ***")
            self.set_status("E-STOP latched", ok=False)
            self._schedule_stop_check()
        else:
            self.gui_log("*** EMERGENCY STOP pressed - not connected or nothing could be sent. ***")
        self.refresh_controls()

    def do_reset_estop(self):
        if self.drone.reset_estop():
            self.gui_log("[E-STOP] Reset - firmware stop cleared, override OFF, all motors 0.")
            self.set_status("Connected - ready", ok=True)
        else:
            self.gui_log("[E-STOP] Reset could not be sent - still latched.")
        self.refresh_controls()

    # ======================================================================
    # Shutdown
    # ======================================================================
    def _on_destroy(self, *args):
        # Leave the drone in the safe state before closing the link.
        self._finish_run()
        if self.drone.connected and self.drone.params_ready:
            self.drone.stop_motor_override()
            time.sleep(0.3)   # give cflib time to put the writes on the air
        self.drone.stop_stream_evt.set()
        time.sleep(0.2)
        self.drone.stop_telemetry()
        self.drone.close_link()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
        Gtk.main_quit()


def main():
    global DRONE_URI
    parser = argparse.ArgumentParser(description="LiteWing motor bench test (motorPowerSet override)")
    parser.add_argument("--drone", default=DRONE_URI,
                        help=f"Drone link URI (default: {DRONE_URI})")
    args = parser.parse_args()
    DRONE_URI = args.drone

    app = MotorTestApp()
    Gtk.main()


if __name__ == "__main__":
    main()
