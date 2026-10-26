#!/usr/bin/env python3
"""
LiteWing Gamepad Flight - GTK
=============================
Fly the drone from the Quantron QGP-1800 gamepad over the normal WiFi/UDP
link, with two flight modes (ANGLE and ACRO) and a real ARM / DISARM.
Same structure as the other LiteWing GTK tools (GTK window, link class,
cflib callbacks marshaled with GLib.idle_add, setpoint stream thread).

Gamepad layout (edit the CONFIG block below to change it)
---------------------------------------------------------
  Left stick  X (CH1, ABS_X)  -> YAW
  Left stick  Y (CH2, ABS_Y)  -> THROTTLE   (only the upper half is used:
                                             centre or below = 0 %,
                                             fully up        = 100 %)
  Right stick X (CH3, ABS_Z)  -> ROLL
  Right stick Y (CH4, ABS_RZ) -> PITCH

  TOP2   -> ARM      (motors turn at idle, also with the throttle stick centred)
  BASE   -> DISARM   (motors stop)
  PINKIE -> ANGLE mode (self-levelling: stick = tilt angle)
  BASE2  -> ACRO  mode (stick = rotation speed)

The buttons are momentary on the hardware. The app remembers (latches) the
state: one press of TOP2 keeps the drone ARMED until BASE is pressed, one
press of PINKIE keeps ANGLE mode until BASE2 is pressed.

What the app sends
------------------
  Setpoints, 50 times per second:   roll, pitch, yaw-rate, thrust
  Parameters (each one is confirmed by the firmware before the app uses it):
    powerDist.armed            1 = armed, 0 = disarmed   (NEW - needs the
                               changed power_distribution_stock.c)
    powerDist.idleThrust       idle motor value while armed
    flightmode.stabModeRoll    1 = angle, 0 = rate (acro)
    flightmode.stabModePitch   1 = angle, 0 = rate (acro)
    stabilizer.stop            1 = firmware emergency stop

SAFETY - read before use
------------------------
  * First tests WITHOUT propellers. Check on the screen that every stick
    moves the right bar in the right direction before you fit propellers.
  * ARM is refused while the throttle stick is above centre.
  * The firmware disarms by itself when it gets no command for 1 second
    (WiFi lost, app closed). It does not arm again by itself.
  * The app disarms when the gamepad is unplugged or telemetry stops.
  * Space = EMERGENCY STOP, Esc = DISARM.

Usage:
  python3 litewing_gamepad_flight_gtk.py --drone udp://192.168.43.42:2390
  python3 litewing_gamepad_flight_gtk.py --no-drone     (gamepad + screen only)
"""

import os
import time
import fcntl
import argparse
import threading
from datetime import datetime

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, Gdk

try:
    import evdev
    GAMEPAD_AVAILABLE = True
except ImportError:
    evdev = None
    GAMEPAD_AVAILABLE = False

# ============================================================================
# CONFIG
# ============================================================================
DRONE_URI = "udp://192.168.43.42:2390"  # Custom Drone (made by us)

# ── Gamepad ────────────────────────────────────────────────────────────────
GAMEPAD_DEVICE_NAME_HINT = "microntek"
GAMEPAD_AXIS_MIN, GAMEPAD_AXIS_MAX, GAMEPAD_AXIS_CENTER = 0, 255, 128
GAMEPAD_DEADZONE_RAW = 15

# Linux input event codes (same numbers as evdev.ecodes).
EV_KEY, EV_ABS = 1, 3
ABS_X, ABS_Y, ABS_Z, ABS_RZ = 0, 1, 2, 5

# Which stick axis does what.
AXIS_YAW = ABS_X        # CH1  left stick X
AXIS_THROTTLE = ABS_Y   # CH2  left stick Y
AXIS_ROLL = ABS_Z       # CH3  right stick X
AXIS_PITCH = ABS_RZ     # CH4  right stick Y

# Stick directions. If a bar on the screen moves the wrong way, flip the
# flag for that axis - nothing else needs to change.
THROTTLE_UP_IS_RAW_LOW = True       # stick pushed UP gives the small raw number
PITCH_FORWARD_IS_RAW_LOW = True     # stick pushed FORWARD gives the small raw number
ROLL_RIGHT_IS_RAW_HIGH = True       # stick pushed RIGHT gives the big raw number
YAW_RIGHT_IS_RAW_HIGH = True        # stick pushed RIGHT gives the big raw number

# Button codes (EV_KEY codes from the live channel monitor).
BUTTON_LABELS = {
    288: "TRIGGER", 289: "THUMB",  290: "THUMB2", 291: "TOP",
    292: "TOP2",    293: "PINKIE", 294: "BASE",   295: "BASE2",
    296: "BASE3",   297: "BASE4",  298: "BASE5",  299: "BASE6",
}
BTN_ARM = 292       # TOP2
BTN_ANGLE = 293     # PINKIE
BTN_DISARM = 294    # BASE
BTN_ACRO = 295      # BASE2
BUTTON_FUNCTIONS = {BTN_ARM: "ARM", BTN_DISARM: "DISARM", BTN_ANGLE: "ANGLE", BTN_ACRO: "ACRO"}

# ── Throttle ───────────────────────────────────────────────────────────────
# Idle: how fast the motors turn when ARMED with the throttle at 0 %.
# esp-fc default (motorIdle 550) is 5.5 %.
IDLE_PERCENT = 5.5
# Stick fully up gives this much of full motor power. Keep it low for the
# first flights, raise it when the drone flies well.
THROTTLE_LIMIT_PERCENT = 100
# ARM is refused when the throttle stick is above this.
ARM_MAX_THROTTLE_PERCENT = 2.0

# ── Flight modes ───────────────────────────────────────────────────────────
# ANGLE mode: full roll/pitch stick = this tilt angle.   (esp-fc default: 55)
ANGLE_LIMIT_DEG = 30.0
# ACRO mode: roll/pitch rotation speed, same curve as esp-fc "actual rates":
#   near the centre the stick gives CENTER deg/s per full stick,
#   at full stick it gives MAX deg/s.                    (esp-fc: 200 / 400)
ACRO_CENTER_DPS = 100.0
ACRO_MAX_DPS = 200.0
# Yaw is always a rotation speed, in both modes.         (esp-fc: 300 / 360)
YAW_CENTER_DPS = 100.0
YAW_MAX_DPS = 200.0

# ── Link ───────────────────────────────────────────────────────────────────
STREAM_HZ = 50                 # setpoints per second
TELEMETRY_STALE_S = 1.5        # no telemetry for this long = link problem
CONFIRM_TIMEOUT_S = 1.0        # how long to wait for the firmware to confirm
CONFIRM_RETRIES = 3
PARAM_WAIT_WARN_S = 20

LOG_DIR = os.path.expanduser("~/litewing_logs")

# Firmware parameter names
P_ARMED = "powerDist.armed"
P_IDLE = "powerDist.idleThrust"
P_MODE_ROLL = "flightmode.stabModeRoll"
P_MODE_PITCH = "flightmode.stabModePitch"
P_FW_STOP = "stabilizer.stop"
FW_MODE_VALUE = {"ANGLE": 1, "ACRO": 0}     # firmware: 1 = angle, 0 = rate

FULL_SCALE = 65535
FW_MAX_THRUST = 60000          # the firmware never accepts more than this
IDLE_THRUST = int(round(IDLE_PERCENT / 100.0 * FULL_SCALE))
MAX_THRUST = min(FW_MAX_THRUST, int(round(THROTTLE_LIMIT_PERCENT / 100.0 * FULL_SCALE)))


# ============================================================================
# Pure helpers (no GUI, no drone) - easy to test
# ============================================================================
def normalize_axis(raw_value):
    """Raw 0..255 (centre 128) -> -1.0..+1.0. Inside the deadzone it is
    exactly 0, and it starts from 0 right outside it (no jump)."""
    centered = raw_value - GAMEPAD_AXIS_CENTER
    if abs(centered) <= GAMEPAD_DEADZONE_RAW:
        return 0.0
    span = (GAMEPAD_AXIS_MAX - GAMEPAD_AXIS_CENTER) - GAMEPAD_DEADZONE_RAW
    travel = abs(centered) - GAMEPAD_DEADZONE_RAW
    norm = max(0.0, min(1.0, travel / span))
    return norm if centered > 0 else -norm


def actual_rate(stick, center_dps, max_dps):
    """esp-fc / Betaflight "actual rates" with expo 0.
    stick -1..+1 -> rotation speed in deg/s."""
    stick = max(-1.0, min(1.0, stick))
    extra = max(0.0, max_dps - center_dps)
    return stick * center_dps + extra * abs(stick) * stick


def throttle_to_thrust(throttle_percent):
    """Throttle 0..100 % -> thrust number for send_setpoint().
    0 % sends 0: the armed firmware then turns all motors at idle."""
    if throttle_percent < 0.5:
        return 0
    frac = min(100.0, throttle_percent) / 100.0
    return int(round(IDLE_THRUST + frac * (MAX_THRUST - IDLE_THRUST)))


def compute_setpoint(arm_state, fw_mode, sticks):
    """Returns (roll, pitch, yawrate, thrust) exactly as given to cflib.
    sticks: dict with roll, pitch, yaw (-1..+1) and throttle_pct (0..100).
    Not ARMED -> everything 0. Mode not confirmed -> roll/pitch 0."""
    if arm_state != "ARMED":
        return (0.0, 0.0, 0.0, 0)
    thrust = throttle_to_thrust(sticks["throttle_pct"])
    yaw = actual_rate(sticks["yaw"], YAW_CENTER_DPS, YAW_MAX_DPS)
    if fw_mode == "ANGLE":
        roll = sticks["roll"] * ANGLE_LIMIT_DEG
        pitch = sticks["pitch"] * ANGLE_LIMIT_DEG
    elif fw_mode == "ACRO":
        roll = actual_rate(sticks["roll"], ACRO_CENTER_DPS, ACRO_MAX_DPS)
        pitch = actual_rate(sticks["pitch"], ACRO_CENTER_DPS, ACRO_MAX_DPS)
    else:
        roll = pitch = 0.0
    return (roll, pitch, yaw, thrust)


# ── CSS styling ──────────────────────────────────────────────────────────────
CSS = b"""
* { font-family: monospace; }
window { background-color: #0a0a0a; }
#title_label { color: #00ff88; font-size: 16px; font-weight: bold; padding: 4px 0px 2px 0px; }
#section_label { color: #00ff88; font-size: 11px; font-weight: bold; padding: 2px 0px; }
#status_ok { color: #00ff88; font-size: 11px; font-weight: bold; }
#status_bad { color: #ff4444; font-size: 11px; font-weight: bold; }
#status_off { color: #557755; font-size: 11px; }
#plain { color: #cfd8cf; font-size: 11px; }
#readout { color: #00ff88; font-size: 12px; font-weight: bold; }
#throttle_big { color: #ffcc00; font-size: 26px; font-weight: bold; }
#log_view, #log_view text { background-color: #101410; color: #66cc88; font-family: monospace; font-size: 10px; }
button { font-family: monospace; background-image: none; background-color: #1a1f1a; color: #cfd8cf;
         border: 1px solid #2c3a2c; padding: 6px 0px; text-shadow: none; }
button:disabled { color: #4a5a4a; }
#estop_btn { background-image: none; background-color: #3a0808; color: #ff4444; border: 2px solid #cc2222;
             font-size: 13px; font-weight: bold; padding: 6px 0px; }
.tab { background-image: none; background-color: #1a1f1a; color: #557755; border: 1px solid #2c3a2c;
       font-size: 13px; font-weight: bold; padding: 8px 0px; }
.tab-armed { background-color: #7a1010; color: #ffffff; border: 1px solid #ff4444; }
.tab-safe { background-color: #0f4a2a; color: #ffffff; border: 1px solid #00ff88; }
.tab-mode { background-color: #0d3f66; color: #ffffff; border: 1px solid #44aaff; }
.tab-wait { background-color: #5a4a00; color: #ffffff; border: 1px solid #ffcc00; }
.btnbox { background-color: #1a1f1a; color: #557755; border: 1px solid #2c3a2c; border-radius: 4px;
          font-size: 10px; font-weight: bold; padding: 5px 2px; }
.btnbox-on { background-color: #ffcc00; color: #000000; border: 1px solid #ffffff; }
"""


# ============================================================================
# Small drawing widgets
# ============================================================================
class StickView(Gtk.DrawingArea):
    """One stick: a square with a dot. x = right, y = up, both -1..+1."""

    def __init__(self, title, x_label, y_label, size=160):
        super().__init__()
        self.title, self.x_label, self.y_label = title, x_label, y_label
        self.x = 0.0
        self.y = 0.0
        self.set_size_request(size, size + 18)
        self.connect("draw", self._on_draw)

    def set_position(self, x, y):
        if (x, y) != (self.x, self.y):
            self.x, self.y = x, y
            self.queue_draw()

    def _on_draw(self, widget, cr):
        w = self.get_allocated_width()
        h = self.get_allocated_height() - 18
        side = min(w, h) - 4
        left = (w - side) / 2.0
        top = 2.0
        # box
        cr.set_source_rgb(0.07, 0.09, 0.07)
        cr.rectangle(left, top, side, side)
        cr.fill()
        cr.set_source_rgb(0.17, 0.23, 0.17)
        cr.set_line_width(1)
        cr.rectangle(left + 0.5, top + 0.5, side - 1, side - 1)
        cr.stroke()
        # cross
        cx, cy = left + side / 2.0, top + side / 2.0
        cr.move_to(left, cy)
        cr.line_to(left + side, cy)
        cr.move_to(cx, top)
        cr.line_to(cx, top + side)
        cr.stroke()
        # dot
        r = side / 2.0 - 8
        px = cx + max(-1.0, min(1.0, self.x)) * r
        py = cy - max(-1.0, min(1.0, self.y)) * r
        moved = abs(self.x) > 0.001 or abs(self.y) > 0.001
        cr.set_source_rgb(1.0, 0.8, 0.0) if moved else cr.set_source_rgb(0.0, 1.0, 0.53)
        cr.arc(px, py, 7, 0, 6.2832)
        cr.fill()
        # labels: name inside the box, the two numbers below it
        cr.select_font_face("monospace")
        cr.set_font_size(10)
        cr.set_source_rgb(0.33, 0.47, 0.33)
        cr.move_to(left + 5, top + 13)
        cr.show_text(self.title)
        cr.set_source_rgb(0.81, 0.85, 0.81)
        cr.move_to(left, top + side + 13)
        cr.show_text(f"{self.x_label} {self.x:+.2f}  {self.y_label} {self.y:+.2f}")
        return False


class ThrottleBar(Gtk.DrawingArea):
    """Vertical bar 0..100 %."""

    def __init__(self, width=46, height=160):
        super().__init__()
        self.percent = 0.0
        self.set_size_request(width, height + 18)
        self.connect("draw", self._on_draw)

    def set_percent(self, percent):
        percent = max(0.0, min(100.0, percent))
        if abs(percent - self.percent) > 0.05:
            self.percent = percent
            self.queue_draw()

    def _on_draw(self, widget, cr):
        w = self.get_allocated_width()
        h = self.get_allocated_height() - 18
        cr.set_source_rgb(0.07, 0.09, 0.07)
        cr.rectangle(2, 2, w - 4, h - 4)
        cr.fill()
        fill_h = (h - 8) * self.percent / 100.0
        cr.set_source_rgb(1.0, 0.8, 0.0)
        cr.rectangle(4, 4 + (h - 8) - fill_h, w - 8, fill_h)
        cr.fill()
        cr.set_source_rgb(0.17, 0.23, 0.17)
        cr.set_line_width(1)
        cr.rectangle(2.5, 2.5, w - 5, h - 5)
        cr.stroke()
        cr.set_source_rgb(0.81, 0.85, 0.81)
        cr.select_font_face("monospace")
        cr.set_font_size(10)
        cr.move_to(2, h + 13)
        cr.show_text("THR")
        return False


# ============================================================================
# Links: the real drone (cflib) and a stand-in for --no-drone
# ============================================================================
class DroneLink:
    """Owns the Crazyflie connection. Every callback into the app is
    marshaled to the GTK thread with GLib.idle_add."""

    def __init__(self, app, uri):
        self.app = app
        self.uri = uri
        self.name = "drone"
        self.cf = None
        self.connected = False
        self.ready = False          # parameter list read, writes possible
        self.has_arm = False        # firmware has powerDist.armed
        self.log_confs = []
        self._param_wait_ticks = 0

    # ── Connection ───────────────────────────────────────────────────────
    def do_connect(self):
        if self.connected:
            self.app.gui_log("[LINK] Already connected.")
            return
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        cflib.crtp.init_drivers()
        self.cf = Crazyflie()
        self.cf.connected.add_callback(self._on_connected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.app.gui_log(f"[LINK] Connecting to {self.uri} ...")
        self.cf.open_link(self.uri)

    def _on_connected(self, link_uri):
        def _apply():
            self.connected = True
            self.ready = False
            self.app.gui_log(f"[OK] Connected: {link_uri}")
            self._start_telemetry()
            self._param_wait_ticks = 0
            GLib.timeout_add(200, self._wait_params_tick)
            self.app.on_link_connected()
            return False
        GLib.idle_add(_apply)

    def _on_connection_failed(self, link_uri, msg):
        def _apply():
            self.connected = False
            self.ready = False
            self.app.on_link_lost(f"connection failed: {msg}")
            return False
        GLib.idle_add(_apply)

    def _on_disconnected(self, link_uri):
        def _apply():
            self.connected = False
            self.ready = False
            self._stop_telemetry()
            self.app.on_link_lost("disconnected")
            return False
        GLib.idle_add(_apply)

    # ── Parameters ───────────────────────────────────────────────────────
    def _params_usable(self):
        gate = getattr(self.cf.param, "_initialized", None)
        return gate.is_set() if gate is not None else True

    def _wait_params_tick(self):
        if not self.connected:
            return False
        if self._params_usable():
            self._on_params_usable()
            return False
        self._param_wait_ticks += 1
        if self._param_wait_ticks == int(PARAM_WAIT_WARN_S / 0.2):
            self.app.gui_log(f"[WARN] Still reading parameters after {PARAM_WAIT_WARN_S}s - check the WiFi link.")
        return True

    def has_param(self, name):
        try:
            return self.cf.param.toc.get_element_by_complete_name(name) is not None
        except Exception:
            return False

    def _on_params_usable(self):
        self.has_arm = self.has_param(P_ARMED)
        for complete in (P_ARMED, P_IDLE, P_MODE_ROLL, P_MODE_PITCH, P_FW_STOP):
            if not self.has_param(complete):
                continue
            group, name = complete.split(".", 1)
            try:
                self.cf.param.add_update_callback(group=group, name=name, cb=self._on_param_update)
            except Exception as e:
                self.app.gui_log(f"[WARN] Could not watch {complete} ({e}).")
        self.ready = True
        self.app.on_link_ready()

    def _on_param_update(self, name, value):
        try:
            number = int(float(value))
        except (TypeError, ValueError):
            return
        GLib.idle_add(self.app.on_param_value, name, number)

    def write_param(self, name, value):
        """True if the write was handed to cflib. Sent as a string: every
        cflib version accepts that."""
        if not (self.connected and self.ready and self.cf is not None):
            return False
        if not self.has_param(name):
            return False
        try:
            self.cf.param.set_value(name, str(int(value)))
            return True
        except Exception as e:
            self.app.gui_log(f"[LINK] PARAM WRITE FAILED {name}={value}: {e}")
            return False

    def request_param(self, name):
        """Ask the firmware for the current value (answer comes through
        _on_param_update)."""
        if not (self.connected and self.ready and self.cf is not None):
            return
        try:
            self.cf.param.request_param_update(name)
        except Exception:
            pass

    # ── Setpoints (called from the stream thread) ────────────────────────
    def send_setpoint(self, roll, pitch, yawrate, thrust):
        self.cf.commander.send_setpoint(roll, pitch, yawrate, int(thrust))

    # ── Telemetry ────────────────────────────────────────────────────────
    def _start_telemetry(self):
        from cflib.crazyflie.log import LogConfig
        self.log_confs = []

        def fly_cb(timestamp, data, logconf):
            GLib.idle_add(self.app.on_telemetry, dict(data))

        def error_cb(logconf, msg):
            self.app.gui_log(f"[LOG ERROR] {logconf.name}: {msg}")

        motors = [(f"pwm.m{i}_pwm", "uint32_t") for i in (1, 2, 3, 4)]
        armed = [("powerDist.armed", "uint8_t")]
        vbat = [("pm.vbat", "float")]
        attitude = [("stabilizer.roll", "float"), ("stabilizer.pitch", "float"), ("stabilizer.yaw", "float")]

        def try_block(block_name, variables):
            lc = LogConfig(name=block_name, period_in_ms=100)
            try:
                for var, vtype in variables:
                    lc.add_variable(var, vtype)
                lc.data_received_cb.add_callback(fly_cb)
                lc.error_cb.add_callback(error_cb)
                self.cf.log.add_config(lc)
                lc.start()
            except (KeyError, AttributeError):
                return False
            self.log_confs.append(lc)
            return True

        # First combination that the firmware accepts wins.
        for variables in (motors + armed + vbat, motors + armed, motors + vbat, motors):
            if try_block("Fly", variables):
                break
        else:
            self.app.gui_log("[WARN] No motor telemetry available from this firmware.")
        if not try_block("Att", attitude):
            self.app.gui_log("[WARN] No attitude telemetry available from this firmware.")

    def _stop_telemetry(self):
        for lc in self.log_confs:
            try:
                lc.stop()
            except Exception:
                pass
        self.log_confs = []

    def close(self):
        self._stop_telemetry()
        if self.cf is not None:
            try:
                self.cf.close_link()
            except Exception:
                pass


class SimLink:
    """--no-drone: answers like the firmware would, so the gamepad and the
    screen can be tried without a drone. Nothing is sent anywhere."""

    def __init__(self, app, uri):
        self.app = app
        self.uri = "no drone (simulation)"
        self.name = "sim"
        self.connected = False
        self.ready = False
        self.has_arm = True
        self._params = {P_ARMED: 0, P_IDLE: 0, P_MODE_ROLL: 1, P_MODE_PITCH: 1, P_FW_STOP: 0}
        self._setpoint = (0.0, 0.0, 0.0, 0)

    def do_connect(self):
        if self.connected:
            return
        self.app.gui_log("[SIM] No drone: simulated link.")

        def _up():
            self.connected = True
            self.ready = True
            self.app.on_link_connected()
            self.app.on_link_ready()
            GLib.timeout_add(100, self._telemetry_tick)
            return False
        GLib.timeout_add(300, _up)

    def has_param(self, name):
        return name in self._params

    def write_param(self, name, value):
        if not self.ready or name not in self._params:
            return False
        self._params[name] = int(value)
        GLib.timeout_add(30, lambda: self.app.on_param_value(name, int(value)) and False)
        return True

    def request_param(self, name):
        if self.ready and name in self._params:
            value = self._params[name]
            GLib.timeout_add(30, lambda: self.app.on_param_value(name, value) and False)

    def send_setpoint(self, roll, pitch, yawrate, thrust):
        self._setpoint = (roll, pitch, yawrate, int(thrust))

    def _telemetry_tick(self):
        if not self.connected:
            return False
        thrust = self._setpoint[3]
        if self._params[P_FW_STOP] or not self._params[P_ARMED]:
            motor = 0
        else:
            motor = max(self._params[P_IDLE], thrust)
        self.app.on_telemetry({"pwm.m1_pwm": motor, "pwm.m2_pwm": motor, "pwm.m3_pwm": motor,
                               "pwm.m4_pwm": motor, "powerDist.armed": self._params[P_ARMED],
                               "stabilizer.roll": 0.0, "stabilizer.pitch": 0.0, "stabilizer.yaw": 0.0})
        return True

    def close(self):
        self.connected = False
        self.ready = False


# ============================================================================
# The application
# ============================================================================
class FlightApp(Gtk.Window):

    def __init__(self, no_drone=False):
        super().__init__(title="LiteWing Gamepad Flight")
        self.set_default_size(1020, 740)

        link_class = SimLink if no_drone else DroneLink
        self.link = link_class(self, DRONE_URI)

        # ── Flight state ──────────────────────────────────────────────────
        # arm_state: DISARMED -> ARMING -> ARMED -> DISARMING -> DISARMED
        self.arm_state = "DISARMED"
        self.mode_want = "ANGLE"      # what the pilot selected
        self.mode_fw = None           # what the firmware has confirmed (None = not confirmed)
        self.estop_latched = False
        self._fw_values = {}          # last value the firmware reported for each parameter
        self._arm_request_t = 0.0
        self._arm_tries = 0
        self._disarm_tries = 0
        self._mode_tries = 0
        self._mode_request_t = 0.0
        self._fw_disarmed_count = 0
        self._last_fix_disarm_t = 0.0
        self._estop_reset_t = 0.0
        self._watch_count = 0

        # ── Sticks ────────────────────────────────────────────────────────
        self.raw_axis = {AXIS_YAW: GAMEPAD_AXIS_CENTER, AXIS_THROTTLE: GAMEPAD_AXIS_CENTER,
                         AXIS_ROLL: GAMEPAD_AXIS_CENTER, AXIS_PITCH: GAMEPAD_AXIS_CENTER}
        self.sticks = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0, "throttle_pct": 0.0}
        self.setpoint = (0.0, 0.0, 0.0, 0)     # read by the stream thread
        self.button_state = {code: 0 for code in BUTTON_LABELS}
        self._button_lit_until = {code: 0.0 for code in BUTTON_LABELS}

        # ── Gamepad device ────────────────────────────────────────────────
        self.gamepad_dev = None
        self._gamepad_note = None

        # ── Telemetry ─────────────────────────────────────────────────────
        self.telemetry = {}
        self._last_telemetry_t = None
        self._stream_stop = threading.Event()

        # ── Persistent file log ───────────────────────────────────────────
        self._log_file = None
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            log_path = os.path.join(LOG_DIR, f"gamepadflight_{datetime.now():%Y%m%d_%H%M%S}.log")
            self._log_file = open(log_path, "a", buffering=1)
            print(f"[INFO] Logging to {log_path}")
        except OSError as e:
            print(f"[WARN] Could not open log file: {e} - continuing without file log")

        css_provider = Gtk.CssProvider()
        css_provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        self._build_ui()
        self.connect("destroy", self._on_destroy)
        self.connect("key-press-event", self._on_key_press)

        self.gui_log(f"[CONFIG] idle {IDLE_PERCENT}% (thrust {IDLE_THRUST}), throttle limit "
                     f"{THROTTLE_LIMIT_PERCENT}% (thrust {MAX_THRUST}), angle {ANGLE_LIMIT_DEG:.0f} deg, "
                     f"acro {ACRO_CENTER_DPS:.0f}/{ACRO_MAX_DPS:.0f} deg/s, yaw {YAW_CENTER_DPS:.0f}/{YAW_MAX_DPS:.0f} deg/s")
        self._gamepad_open()
        GLib.timeout_add(2000, self._gamepad_rescan_tick)
        GLib.timeout_add(50, self._display_tick)
        GLib.timeout_add(200, self._watch_tick)
        self.refresh()

    # ======================================================================
    # UI
    # ======================================================================
    def _build_ui(self):
        root = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        self.add(root)

        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.set_size_request(560, -1)
        root.pack_start(scroll, False, False, 0)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        for setter in (left.set_margin_start, left.set_margin_end, left.set_margin_top, left.set_margin_bottom):
            setter(12)
        scroll.add(left)

        title = Gtk.Label(label="◈ LiteWing Gamepad Flight")
        title.set_name("title_label")
        left.pack_start(title, False, False, 0)

        self.status_lbl = self._label(left, "Not connected", "status_bad")
        self.gamepad_lbl = self._label(left, "Gamepad: searching...", "status_off")

        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.connect_btn = Gtk.Button(label="Connect")
        self.connect_btn.connect("clicked", lambda *_: self.do_connect())
        self.estop_btn = Gtk.Button(label="■ EMERGENCY STOP (Space)")
        self.estop_btn.set_name("estop_btn")
        self.estop_btn.connect("clicked", lambda *_: self.do_emergency_stop())
        self.reset_estop_btn = Gtk.Button(label="Reset E-STOP")
        self.reset_estop_btn.connect("clicked", lambda *_: self.do_reset_estop())
        row.pack_start(self.connect_btn, True, True, 0)
        row.pack_start(self.estop_btn, True, True, 0)
        row.pack_start(self.reset_estop_btn, True, True, 0)
        left.pack_start(row, False, False, 2)

        # ── State tabs ────────────────────────────────────────────────────
        self._section(left, "MOTORS   (TOP2 = arm, BASE = disarm)")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_homogeneous(True)
        self.tab_disarmed = self._tab(row, "DISARMED", lambda: self.request_disarm("screen"))
        self.tab_armed = self._tab(row, "ARMED", lambda: self.request_arm("screen"))
        left.pack_start(row, False, False, 0)

        self._section(left, "FLIGHT MODE   (PINKIE = angle, BASE2 = acro)")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_homogeneous(True)
        self.tab_angle = self._tab(row, "ANGLE", lambda: self.request_mode("ANGLE", "screen"))
        self.tab_acro = self._tab(row, "ACRO", lambda: self.request_mode("ACRO", "screen"))
        left.pack_start(row, False, False, 0)

        # ── Sticks + throttle ─────────────────────────────────────────────
        self._section(left, "STICKS")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.left_stick = StickView("LEFT STICK", "yaw", "thr")
        self.throttle_bar = ThrottleBar()
        thr_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        thr_title = Gtk.Label(label="THROTTLE")
        thr_title.set_name("section_label")
        self.throttle_lbl = Gtk.Label(label="0%")
        self.throttle_lbl.set_name("throttle_big")
        self.thrust_lbl = Gtk.Label(label="thrust 0")
        self.thrust_lbl.set_name("plain")
        thr_box.pack_start(thr_title, False, False, 0)
        thr_box.pack_start(self.throttle_lbl, False, False, 6)
        thr_box.pack_start(self.thrust_lbl, False, False, 0)
        self.right_stick = StickView("RIGHT STICK", "roll", "pitch")
        row.pack_start(self.left_stick, False, False, 0)
        row.pack_start(self.throttle_bar, False, False, 0)
        row.pack_start(thr_box, True, True, 0)
        row.pack_start(self.right_stick, False, False, 0)
        left.pack_start(row, False, False, 0)

        self._section(left, "SENDING TO DRONE")
        self.sending_lbl = self._label(left, "roll 0   pitch 0   yaw 0   thrust 0", "readout")

        # ── Gamepad buttons ───────────────────────────────────────────────
        self._section(left, "GAMEPAD BUTTONS   (lights up when pressed)")
        grid = Gtk.Grid()
        grid.set_column_spacing(6)
        grid.set_row_spacing(6)
        grid.set_column_homogeneous(True)
        self.button_boxes = {}
        for index, (code, name) in enumerate(BUTTON_LABELS.items()):
            text = name if code not in BUTTON_FUNCTIONS else f"{name}\n{BUTTON_FUNCTIONS[code]}"
            box = Gtk.Label(label=text)
            box.set_justify(Gtk.Justification.CENTER)
            box.get_style_context().add_class("btnbox")
            grid.attach(box, index % 6, index // 6, 1, 1)
            self.button_boxes[code] = box
        left.pack_start(grid, False, False, 0)

        # ── Right column: firmware read-outs + log ────────────────────────
        right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        right.set_margin_start(4)
        right.set_margin_end(12)
        right.set_margin_top(12)
        right.set_margin_bottom(12)
        self._section(right, "FROM FIRMWARE")
        self.fw_state_lbl = self._label(right, "armed --   mode --", "readout")
        self.fw_motor_lbl = self._label(right, "motors  M1 --  M2 --  M3 --  M4 --", "readout")
        self.fw_att_lbl = self._label(right, "roll --  pitch --  yaw --   batt --", "readout")
        self._section(right, "LOG")
        self.log_view = Gtk.TextView()
        self.log_view.set_name("log_view")
        self.log_view.set_editable(False)
        self.log_view.set_cursor_visible(False)
        self.log_view.set_can_focus(False)
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD)
        log_scroll = Gtk.ScrolledWindow()
        log_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        log_scroll.add(self.log_view)
        right.pack_start(log_scroll, True, True, 0)
        root.pack_start(right, True, True, 0)

        self.show_all()

    def _label(self, parent, text, name):
        lbl = Gtk.Label(label=text)
        lbl.set_name(name)
        lbl.set_xalign(0)
        parent.pack_start(lbl, False, False, 0)
        return lbl

    def _section(self, parent, text):
        lbl = Gtk.Label(label=text)
        lbl.set_name("section_label")
        lbl.set_xalign(0)
        parent.pack_start(lbl, False, False, 3)

    def _tab(self, parent, text, on_click):
        btn = Gtk.Button(label=text)
        btn.get_style_context().add_class("tab")
        btn.set_can_focus(False)      # keeps Space for E-STOP
        btn.connect("clicked", lambda *_: on_click())
        parent.pack_start(btn, True, True, 0)
        return btn

    @staticmethod
    def _set_tab(btn, text, style):
        """style: None (not active) or one of tab-armed / tab-safe / tab-mode / tab-wait."""
        ctx = btn.get_style_context()
        for cls in ("tab-armed", "tab-safe", "tab-mode", "tab-wait"):
            ctx.remove_class(cls)
        if style:
            ctx.add_class(style)
        btn.set_label(text)

    # ======================================================================
    # Log / status helpers
    # ======================================================================
    def gui_log(self, msg):
        stamped = f"{datetime.now():%H:%M:%S}  {msg}"
        if self._log_file is not None:
            try:
                self._log_file.write(stamped + "\n")
            except (OSError, ValueError):
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

    def refresh(self):
        """One place that draws the state tabs and the button states."""
        if self.arm_state == "ARMED":
            self._set_tab(self.tab_armed, "ARMED", "tab-armed")
            self._set_tab(self.tab_disarmed, "DISARMED", None)
        elif self.arm_state == "ARMING":
            self._set_tab(self.tab_armed, "ARMING...", "tab-wait")
            self._set_tab(self.tab_disarmed, "DISARMED", None)
        elif self.arm_state == "DISARMING":
            self._set_tab(self.tab_armed, "ARMED", None)
            self._set_tab(self.tab_disarmed, "DISARMING...", "tab-wait")
        else:
            self._set_tab(self.tab_armed, "ARMED", None)
            self._set_tab(self.tab_disarmed, "DISARMED", "tab-safe")

        for mode, btn in (("ANGLE", self.tab_angle), ("ACRO", self.tab_acro)):
            if self.mode_want != mode:
                self._set_tab(btn, mode, None)
            elif self.mode_fw == mode:
                self._set_tab(btn, mode, "tab-mode")
            else:
                self._set_tab(btn, f"{mode} (waiting)" if self.link.ready else f"{mode} (not connected)", "tab-wait")

        self.connect_btn.set_sensitive(not self.link.connected)
        self.connect_btn.set_label("Connected" if self.link.connected else "Connect")
        self.reset_estop_btn.set_sensitive(self.link.ready and self.estop_latched)
        self._update_setpoint()

    # ======================================================================
    # Keyboard: only STOP actions are bound to keys
    # ======================================================================
    def _on_key_press(self, widget, event):
        if event.keyval == Gdk.KEY_space:
            self.do_emergency_stop()
            return True
        if event.keyval == Gdk.KEY_Escape:
            self.request_disarm("Esc key")
            return True
        return False

    # ======================================================================
    # Link events (GTK thread)
    # ======================================================================
    def do_connect(self):
        self.connect_btn.set_sensitive(False)
        self.connect_btn.set_label("Connecting...")
        self.link.do_connect()

    def on_link_connected(self):
        self.set_status("Connected - reading parameters...", ok=True)
        self._last_telemetry_t = None
        self._stream_stop.clear()
        threading.Thread(target=self._stream_loop, daemon=True).start()
        self.refresh()

    def on_link_ready(self):
        missing = [p for p in (P_MODE_ROLL, P_MODE_PITCH, P_IDLE) if not self.link.has_param(p)]
        if missing:
            self.gui_log(f"[ERROR] Firmware has no {', '.join(missing)} - this app cannot fly it.")
            self.set_status("Connected - parameters missing", ok=False)
            self.refresh()
            return
        if not self.link.has_arm:
            self.gui_log(f"[ERROR] Firmware has no {P_ARMED}. ARM is disabled. Flash the firmware with the "
                         f"changed power_distribution_stock.c, then connect again.")
            self.set_status("Connected - firmware without ARM (see log)", ok=False)
        else:
            self.set_status("Connected - ready", ok=True)
            # Known-safe starting point.
            self.link.write_param(P_ARMED, 0)
            self.link.write_param(P_IDLE, IDLE_THRUST)
            self.link.request_param(P_FW_STOP)
        self.arm_state = "DISARMED"
        self.gui_log("[LINK] Parameters ready - drone forced DISARMED.")
        self.request_mode(self.mode_want, "connect")
        self.refresh()

    def on_link_lost(self, reason):
        was_flying = self.arm_state in ("ARMED", "ARMING")
        self._stream_stop.set()
        self.arm_state = "DISARMED"
        self.mode_fw = None
        self._fw_values = {}
        self.telemetry = {}
        self._last_telemetry_t = None
        self.gui_log(f"[LINK] Lost ({reason}).")
        if was_flying:
            self.gui_log("*** LINK LOST WHILE ARMED - the firmware disarms by itself after 1 second "
                         "without commands. ***")
        self.set_status("Not connected", ok=False)
        self.refresh()

    def on_param_value(self, name, value):
        """The firmware's own answer for one parameter."""
        self._fw_values[name] = value
        if name in (P_MODE_ROLL, P_MODE_PITCH):
            self._check_mode_confirmed()
        elif name == P_ARMED:
            self._on_fw_armed(value)
        elif name == P_FW_STOP:
            self._on_fw_stop(value)
        return False

    def _on_fw_stop(self, value):
        """The firmware reported its emergency-stop flag. The firmware also
        sets this flag by itself when the drone flips over (tumble)."""
        if value and not self.estop_latched:
            if time.monotonic() - self._estop_reset_t < 0.6:
                return                      # old answer from before the reset
            self.estop_latched = True
            self.gui_log("*** [FW] Firmware emergency stop is ACTIVE (pressed earlier, or the drone "
                         "flipped over). Motors are off. Press 'Reset E-STOP' before arming. ***")
            if self.arm_state != "DISARMED":
                self.request_disarm("firmware emergency stop")
            self.refresh()
        elif not value and self.estop_latched and self.link.ready:
            # E-STOP is latched here but the firmware does not have it: send again.
            self.link.write_param(P_FW_STOP, 1)
            self.link.write_param(P_ARMED, 0)

    def on_telemetry(self, data):
        self.telemetry.update(data)
        self._last_telemetry_t = time.monotonic()
        if "powerDist.armed" in data:
            self._on_fw_armed(int(data["powerDist.armed"]))
        return False

    # ======================================================================
    # ARM / DISARM
    # ======================================================================
    def request_arm(self, source):
        if self.arm_state in ("ARMED", "ARMING"):
            return
        reasons = []
        if not (self.link.connected and self.link.ready):
            reasons.append("not connected")
        elif not self.link.has_arm:
            reasons.append(f"firmware has no {P_ARMED}")
        if self.estop_latched:
            reasons.append("E-STOP is latched (press 'Reset E-STOP')")
        if self.mode_fw is None:
            reasons.append("flight mode not confirmed by the firmware")
        if self.sticks["throttle_pct"] > ARM_MAX_THROTTLE_PERCENT:
            reasons.append(f"throttle stick is at {self.sticks['throttle_pct']:.0f}% - bring it to centre")
        if reasons:
            self.gui_log(f"[ARM] Refused ({source}): " + "; ".join(reasons) + ".")
            return
        self.arm_state = "ARMING"
        self._arm_tries = 1
        self._arm_request_t = time.monotonic()
        self._fw_disarmed_count = 0
        self.link.write_param(P_IDLE, IDLE_THRUST)
        self.link.write_param(P_ARMED, 1)
        self.gui_log(f"[ARM] Requested ({source}) - waiting for the firmware...")
        GLib.timeout_add(int(CONFIRM_TIMEOUT_S * 1000), self._arm_timeout, self._arm_request_t)
        self.refresh()

    def _arm_timeout(self, request_t):
        if self.arm_state != "ARMING" or request_t != self._arm_request_t:
            return False
        if self._arm_tries < CONFIRM_RETRIES:
            self._arm_tries += 1
            self._arm_request_t = time.monotonic()
            self.link.write_param(P_IDLE, IDLE_THRUST)
            self.link.write_param(P_ARMED, 1)
            GLib.timeout_add(int(CONFIRM_TIMEOUT_S * 1000), self._arm_timeout, self._arm_request_t)
            return False
        self.gui_log("*** [ARM] The firmware did not confirm - NOT armed. ***")
        self.request_disarm("arm not confirmed")
        return False

    def request_disarm(self, reason):
        """Motors off. Safe to call at any time."""
        if self.arm_state == "DISARMED":
            if self.link.ready and self.link.has_arm:
                self.link.write_param(P_ARMED, 0)      # already disarmed: repeat it, no fuss
            return
        self.arm_state = "DISARMING"      # the stream sends zeros from now on
        self._disarm_tries = 1
        self._update_setpoint()
        sent = self.link.write_param(P_ARMED, 0)
        self.gui_log(f"[DISARM] {reason}" + ("" if sent else " - COULD NOT BE SENT"))
        GLib.timeout_add(400, self._disarm_check)
        self.refresh()

    def _disarm_check(self):
        if self.arm_state != "DISARMING":
            return False
        fresh = (self._last_telemetry_t is not None
                 and time.monotonic() - self._last_telemetry_t < 0.5)
        motors = [int(self.telemetry.get(f"pwm.m{i}_pwm", 0)) for i in (1, 2, 3, 4)]
        fw_armed = self._fw_values.get(P_ARMED)
        if not self.link.connected or (fresh and fw_armed == 0 and not any(motors)):
            self.arm_state = "DISARMED"
            if self.link.connected:
                self.gui_log("[DISARM] Confirmed by firmware - all four motors at 0.")
            self.refresh()
            return False
        if self._disarm_tries < CONFIRM_RETRIES + 1:
            self._disarm_tries += 1
            self.link.write_param(P_ARMED, 0)
            self.link.request_param(P_ARMED)
            return True       # check again in 400 ms
        self.arm_state = "DISARMED"
        self.gui_log("*** DISARM NOT CONFIRMED - " + ("no telemetry from the drone. " if not fresh else
                     f"firmware still reports armed={fw_armed}, motors {motors}. ")
                     + "The firmware stops the motors 1 second after the last command; if a motor "
                     "still turns, DISCONNECT THE BATTERY. ***")
        self.refresh()
        return False

    def _on_fw_armed(self, value):
        """The firmware reported its arm flag (parameter answer or telemetry)."""
        self._fw_values[P_ARMED] = value
        if self.arm_state == "ARMING" and value == 1:
            self.arm_state = "ARMED"
            self._fw_disarmed_count = 0
            self.gui_log(f"[ARM] ARMED - motors at idle ({IDLE_PERCENT}%). Mode {self.mode_fw}.")
            self.refresh()
        elif self.arm_state == "ARMED" and value == 0:
            # Two reports in a row, so one late packet cannot disarm us.
            self._fw_disarmed_count += 1
            if self._fw_disarmed_count >= 2:
                self.arm_state = "DISARMED"
                self.gui_log("*** The firmware reports DISARMED (link timeout or restart) - press ARM again. ***")
                self.refresh()
        elif self.arm_state == "ARMED":
            self._fw_disarmed_count = 0
        elif self.arm_state == "DISARMED" and value == 1 and self.link.ready:
            # Armed in the firmware but not here (app restarted, lost answer).
            now = time.monotonic()
            if now - self._last_fix_disarm_t > 1.0:
                self._last_fix_disarm_t = now
                self.gui_log("[WARN] Firmware is ARMED but the app is not - sending disarm.")
                self.link.write_param(P_ARMED, 0)

    # ======================================================================
    # E-STOP
    # ======================================================================
    def do_emergency_stop(self):
        self.estop_latched = True
        self.arm_state = "DISARMING" if self.arm_state != "DISARMED" else "DISARMED"
        self._update_setpoint()
        if self.link.connected and self.link.ready:
            self.link.write_param(P_FW_STOP, 1)
            self.link.write_param(P_ARMED, 0)
            self.gui_log("*** EMERGENCY STOP - firmware stop sent, disarmed. Press 'Reset E-STOP' to fly again. ***")
            if self.arm_state == "DISARMING":
                self._disarm_tries = 1
                GLib.timeout_add(400, self._disarm_check)
        else:
            self.arm_state = "DISARMED"
            self.gui_log("*** EMERGENCY STOP pressed - not connected, nothing could be sent. ***")
        self.refresh()

    def do_reset_estop(self):
        if not self.link.ready:
            return
        self.link.write_param(P_ARMED, 0)
        if self.link.write_param(P_FW_STOP, 0):
            self.estop_latched = False
            self._estop_reset_t = time.monotonic()
            self.gui_log("[E-STOP] Reset - firmware stop cleared, still DISARMED.")
        else:
            self.gui_log("[E-STOP] Reset could not be sent - still latched.")
        self.refresh()

    # ======================================================================
    # Flight mode
    # ======================================================================
    def request_mode(self, mode, source):
        if mode == self.mode_want and self.mode_fw == mode:
            return
        self.mode_want = mode
        self.mode_fw = None            # roll/pitch stay 0 until the firmware confirms
        self._fw_values.pop(P_MODE_ROLL, None)     # only a new answer counts
        self._fw_values.pop(P_MODE_PITCH, None)
        self._update_setpoint()
        if self.link.connected and self.link.ready:
            self._mode_tries = 1
            self._send_mode()
            self.gui_log(f"[MODE] {mode} requested ({source}) - waiting for the firmware...")
        else:
            self.gui_log(f"[MODE] {mode} selected ({source}) - will be sent when connected.")
        self.refresh()

    def _send_mode(self):
        value = FW_MODE_VALUE[self.mode_want]
        self._mode_request_t = time.monotonic()
        self.link.write_param(P_MODE_ROLL, value)
        self.link.write_param(P_MODE_PITCH, value)
        GLib.timeout_add(int(CONFIRM_TIMEOUT_S * 1000), self._mode_timeout, self._mode_request_t)

    def _mode_timeout(self, request_t):
        if self.mode_fw is not None or request_t != self._mode_request_t or not self.link.ready:
            return False
        if self._mode_tries < CONFIRM_RETRIES:
            self._mode_tries += 1
            self._send_mode()
            self.link.request_param(P_MODE_ROLL)
            self.link.request_param(P_MODE_PITCH)
            return False
        self.gui_log(f"*** [MODE] The firmware did not confirm {self.mode_want}. Roll and pitch stay at 0 - "
                     f"press the mode button again. ***")
        return False

    def _check_mode_confirmed(self):
        want = FW_MODE_VALUE[self.mode_want]
        confirmed = (self._fw_values.get(P_MODE_ROLL) == want and self._fw_values.get(P_MODE_PITCH) == want)
        if confirmed and self.mode_fw != self.mode_want:
            self.mode_fw = self.mode_want
            self.gui_log(f"[MODE] {self.mode_fw} confirmed by firmware.")
            self.refresh()
        elif not confirmed and self.mode_fw is not None:
            self.mode_fw = None
            self.refresh()

    # ======================================================================
    # Sticks -> setpoint
    # ======================================================================
    def _update_sticks(self):
        yaw = normalize_axis(self.raw_axis[AXIS_YAW])
        thr = normalize_axis(self.raw_axis[AXIS_THROTTLE])
        roll = normalize_axis(self.raw_axis[AXIS_ROLL])
        pitch = normalize_axis(self.raw_axis[AXIS_PITCH])
        self.sticks["yaw"] = yaw if YAW_RIGHT_IS_RAW_HIGH else -yaw
        self.sticks["roll"] = roll if ROLL_RIGHT_IS_RAW_HIGH else -roll
        self.sticks["pitch"] = -pitch if PITCH_FORWARD_IS_RAW_LOW else pitch
        self.sticks["throttle"] = -thr if THROTTLE_UP_IS_RAW_LOW else thr
        self.sticks["throttle_pct"] = max(0.0, self.sticks["throttle"]) * 100.0
        self._update_setpoint()

    def _update_setpoint(self):
        # One tuple, replaced in one step: the stream thread always reads a
        # complete setpoint.
        self.setpoint = compute_setpoint(self.arm_state, self.mode_fw, self.sticks)

    def _stream_loop(self):
        """50 Hz. Not armed -> zeros (keeps the link alive and the motors off)."""
        period = 1.0 / STREAM_HZ
        errors = 0
        while self.link.connected and not self._stream_stop.is_set():
            roll, pitch, yaw, thrust = self.setpoint
            try:
                self.link.send_setpoint(roll, pitch, yaw, thrust)
                if errors:
                    self.gui_log("[STREAM] Link recovered.")
                errors = 0
            except Exception as e:
                errors += 1
                if errors == 1:
                    self.gui_log(f"[STREAM ERROR] send_setpoint failed: {e} - retrying...")
            time.sleep(period)

    # ======================================================================
    # Gamepad
    # ======================================================================
    def _gamepad_open(self):
        if self.gamepad_dev is not None:
            return
        if not GAMEPAD_AVAILABLE:
            self._gamepad_status("Gamepad: python 'evdev' is not installed (pip3 install evdev)", False)
            return
        dev = None
        try:
            for path in evdev.list_devices():
                candidate = evdev.InputDevice(path)
                if GAMEPAD_DEVICE_NAME_HINT in candidate.name.lower():
                    dev = candidate
                    break
                candidate.close()
        except Exception as e:
            self._gamepad_status(f"Gamepad: cannot scan input devices ({e})", False)
            return
        if dev is None:
            self._gamepad_status(f"Gamepad: no '{GAMEPAD_DEVICE_NAME_HINT}' device found - plug it in", False)
            return
        try:
            flags = fcntl.fcntl(dev.fd, fcntl.F_GETFL)
            fcntl.fcntl(dev.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        except Exception as e:
            self.gui_log(f"[GAMEPAD] Warning: could not set non-blocking ({e}).")
        # Real stick positions right now (not "centre" until the first move).
        for code in self.raw_axis:
            try:
                self.raw_axis[code] = dev.absinfo(code).value
            except Exception:
                pass
        self.gamepad_dev = dev
        # IN = new events. HUP / ERR = the gamepad was unplugged.
        GLib.io_add_watch(dev.fd, GLib.PRIORITY_DEFAULT,
                          GLib.IOCondition.IN | GLib.IOCondition.HUP | GLib.IOCondition.ERR,
                          self._on_gamepad_io)
        self._gamepad_status(f"Gamepad: {dev.name} ({dev.path})", True)
        self.gui_log(f"[GAMEPAD] Using {dev.path} ({dev.name!r}).")
        self._update_sticks()

    def _gamepad_status(self, text, ok):
        if text != self._gamepad_note:
            self._gamepad_note = text
            self.gamepad_lbl.set_text(text)
            self.gamepad_lbl.set_name("status_ok" if ok else "status_bad")

    def _gamepad_rescan_tick(self):
        if self.gamepad_dev is None:
            self._gamepad_open()
        return True

    def _on_gamepad_io(self, fd, condition):
        dev = self.gamepad_dev
        if dev is None:
            return False
        try:
            for event in dev.read():
                if event.type == EV_ABS:
                    self.on_gamepad_axis(event.code, event.value)
                elif event.type == EV_KEY:
                    self.on_gamepad_button(event.code, event.value)
        except BlockingIOError:
            pass
        except OSError as e:
            self._on_gamepad_lost(e)
            return False
        if condition & (GLib.IOCondition.HUP | GLib.IOCondition.ERR):
            self._on_gamepad_lost("device removed")
            return False
        return True

    def _on_gamepad_lost(self, error):
        self.gui_log(f"*** [GAMEPAD] Lost ({error}) - disarming. ***")
        try:
            self.gamepad_dev.close()
        except Exception:
            pass
        self.gamepad_dev = None
        for code in self.button_state:
            self.button_state[code] = 0
        for code in self.raw_axis:
            self.raw_axis[code] = GAMEPAD_AXIS_CENTER
        self._update_sticks()
        self._gamepad_status("Gamepad: LOST - plug it in again", False)
        self.request_disarm("gamepad lost")

    def on_gamepad_axis(self, code, raw_value):
        if code in self.raw_axis:
            self.raw_axis[code] = raw_value
            self._update_sticks()

    def on_gamepad_button(self, code, value):
        if code not in self.button_state:
            return
        was_down = self.button_state[code]
        self.button_state[code] = 1 if value else 0
        if value and not was_down:
            # The buttons are momentary: keep the box lit long enough to see.
            self._button_lit_until[code] = time.monotonic() + 0.4
            if code == BTN_ARM:
                self.request_arm("TOP2")
            elif code == BTN_DISARM:
                self.request_disarm("BASE button")
            elif code == BTN_ANGLE:
                self.request_mode("ANGLE", "PINKIE")
            elif code == BTN_ACRO:
                self.request_mode("ACRO", "BASE2")

    # ======================================================================
    # Timers
    # ======================================================================
    def _display_tick(self):
        """20 times per second: draw sticks, throttle, numbers, button boxes."""
        s = self.sticks
        self.left_stick.set_position(s["yaw"], s["throttle"])
        self.right_stick.set_position(s["roll"], s["pitch"])
        self.throttle_bar.set_percent(s["throttle_pct"])
        self.throttle_lbl.set_text(f"{s['throttle_pct']:.0f}%")

        roll, pitch, yaw, thrust = self.setpoint
        self.thrust_lbl.set_text(f"thrust {thrust}  ({thrust / FULL_SCALE * 100:.0f}% of full)")
        unit = "deg/s" if self.mode_fw == "ACRO" else "deg"
        self.sending_lbl.set_text(f"roll {roll:+6.1f} {unit}   pitch {pitch:+6.1f} {unit}   "
                                  f"yaw {yaw:+6.1f} deg/s   thrust {thrust}")

        now = time.monotonic()
        for code, box in self.button_boxes.items():
            lit = bool(self.button_state[code]) or now < self._button_lit_until[code]
            ctx = box.get_style_context()
            if lit and not ctx.has_class("btnbox-on"):
                ctx.add_class("btnbox-on")
            elif not lit and ctx.has_class("btnbox-on"):
                ctx.remove_class("btnbox-on")

        t = self.telemetry
        if self.link.connected and t:
            armed = t.get("powerDist.armed", self._fw_values.get(P_ARMED))
            mode_r, mode_p = self._fw_values.get(P_MODE_ROLL), self._fw_values.get(P_MODE_PITCH)
            if mode_r is None or mode_p is None:
                mode_txt = "--"
            elif mode_r == mode_p:
                mode_txt = "ANGLE" if mode_r == 1 else "ACRO"
            else:
                mode_txt = "MIXED"
            self.fw_state_lbl.set_text(f"armed {'--' if armed is None else int(armed)}   mode {mode_txt}")
            motors = "  ".join(f"M{i} {int(t.get(f'pwm.m{i}_pwm', 0)) / FULL_SCALE * 100:4.1f}%" for i in (1, 2, 3, 4))
            self.fw_motor_lbl.set_text(f"motors  {motors}")
            batt = f"{t['pm.vbat']:.2f} V" if "pm.vbat" in t else "--"
            if "stabilizer.roll" in t:
                self.fw_att_lbl.set_text(f"roll {t['stabilizer.roll']:+6.1f}  pitch {t['stabilizer.pitch']:+6.1f}  "
                                         f"yaw {t['stabilizer.yaw']:+6.1f}   batt {batt}")
            else:
                self.fw_att_lbl.set_text(f"roll --  pitch --  yaw --   batt {batt}")
        else:
            self.fw_state_lbl.set_text("armed --   mode --")
            self.fw_motor_lbl.set_text("motors  M1 --  M2 --  M3 --  M4 --")
            self.fw_att_lbl.set_text("roll --  pitch --  yaw --   batt --")
        return True

    def _watch_tick(self):
        """Telemetry must keep arriving while armed. Once per second also ask
        the firmware for its emergency-stop flag (it can set that by itself)."""
        self._watch_count += 1
        if self._watch_count % 5 == 0 and self.link.ready:
            self.link.request_param(P_FW_STOP)
        if (self.link.connected and self.arm_state in ("ARMED", "ARMING")
                and self._last_telemetry_t is not None
                and time.monotonic() - self._last_telemetry_t > TELEMETRY_STALE_S):
            self.gui_log(f"*** NO TELEMETRY for {TELEMETRY_STALE_S}s WHILE ARMED - disarming. ***")
            self.request_disarm("telemetry lost")
        return True

    # ======================================================================
    # Shutdown
    # ======================================================================
    def _on_destroy(self, *args):
        # Leave the drone disarmed before closing the link.
        self.arm_state = "DISARMED"
        self._update_setpoint()
        if self.link.connected and self.link.ready:
            self.link.write_param(P_ARMED, 0)
            time.sleep(0.3)   # give cflib time to put the write on the air
        self._stream_stop.set()
        time.sleep(0.1)
        self.link.close()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
            self._log_file = None
        Gtk.main_quit()


def main():
    global DRONE_URI
    parser = argparse.ArgumentParser(description="LiteWing gamepad flight (angle / acro, arm / disarm)")
    parser.add_argument("--drone", default=DRONE_URI, help=f"Drone link URI (default: {DRONE_URI})")
    parser.add_argument("--no-drone", action="store_true",
                        help="Do not connect to a drone: try the gamepad and the screen only")
    args = parser.parse_args()
    DRONE_URI = args.drone

    app = FlightApp(no_drone=args.no_drone)
    if args.no_drone:
        app.do_connect()
    Gtk.main()


if __name__ == "__main__":
    main()
