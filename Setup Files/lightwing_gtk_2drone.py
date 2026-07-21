#Present working code.

#!/usr/bin/env python3
"""
LiteWing Drone Control + ESP32-CAM Video — GTK + GStreamer
============================================================
Drone control via cflib, video via xvimagesink rendered directly into a GTK
DrawingArea through set_window_handle(xid). GStreamer hands frames straight
to the X server; no per-frame Python conversion, no appsink.

The tilt reference (horizon, pitch ladder, roll arc) is drawn INSIDE the
GStreamer pipeline via cairooverlay - baked into the video pixels before
they reach xvimagesink, which keeps it compatible with Xv's hardware
overlay (GTK-widget-layer transparency conflicts with it).

Threads/contexts:
  1. GTK/GLib main loop      - GUI, GStreamer bus messages, watchdog timer
  2. cflib background thread - telemetry + connection callbacks
                                (marshaled to GUI via GLib.idle_add)
  3. persistent_stream thread - sends a setpoint every 100ms (keepalive)
  4. GStreamer streaming thread - pipeline + per-frame cairooverlay draw

Safety features:
  - E-STOP button + spacebar: sends stop-setpoint immediately (motors cut)
  - Two-step arm: Start -> "CONFIRM ARM?" within 3s -> armed
  - Battery voltage monitoring with low-voltage warning
  - persistent_stream hardened: link errors surface in the log, never die silently
  - Video watchdog: detects a stalled camera feed, auto-reconnects with backoff

Usage:
  python3 lightwing_gtk_zero_latency.py \
      --drone udp://10.114.33.101 \
      --cam   http://10.114.33.110:81/stream
  (both optional - defaults below)
"""

import os
os.environ['GDK_BACKEND'] = 'x11'   # required for get_xid() / video overlay embedding

import fcntl

import time
import math
import argparse
import threading
from datetime import datetime

import cairo
import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gtk, Gst, GstVideo, GLib, Gdk

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

try:
    import evdev
    from evdev import ecodes as ev_ecodes
    GAMEPAD_AVAILABLE = True
except ImportError:
    GAMEPAD_AVAILABLE = False

Gst.init(None)
cflib.crtp.init_drivers()

# ── Config - defaults, overridable from the command line ────────────────────
# DRONE_URI = "udp://10.114.33.101"
# CAM_URL = "http://10.114.33.110:81/stream"  # port 81 = ESP32-CAM MJPEG stream port
DRONE_URI = "udp://192.168.0.11:2390"
CAM_URL = "http://192.168.0.110:81/stream"  # port 81 = ESP32-CAM MJPEG stream port
                                              # (port 80 serves the control page only)
# Second real drone + its own ESP32-CAM. MIRROR MODE: drone2 receives the
# exact same setpoint as drone1 every 100ms (same self.setpoint dict, same
# self.armed flag) - one joystick flies both identically. UPDATE these to
# your real second drone/camera's IPs. The other 7 grid tiles intentionally
# have no URL/URI and never attempt a connection at all.
DRONE_URI_2 = "udp://192.168.0.12:2390"
CAM_URL_2 = "http://192.168.0.111:81/stream"

# Camera tile index each real drone's telemetry/HUD feeds. drone1 -> tile 0
# (CAM1), drone2 -> tile 1 (CAM2). Set as tile_index on each DroneLink
# instance below (see DroneVideoApp.__init__), not a single global anymore -
# each real drone now owns its own tile.

# Manual control tuning
ROLL_PITCH_STEP = 2.0
MAX_TILT = 15.0
THRUST_STEP = 2000
THRUST_MIN = 100
# cflib's send_setpoint() thrust field is a 16-bit unsigned int (0-65535 /
# 0xFFFF). 60000 leaves a safety margin under that hard ceiling - do not
# raise this above 65535 or send_setpoint() will throw on every call once
# the stick crosses that value (this was the actual cause of "motors not
# spinning": out-of-range thrust crashed the setpoint stream, tripping the
# firmware failsafe, not a gamepad-reading problem).
THRUST_MAX = 60000

# Safety / robustness tuning
BATTERY_LOW_V = 3.3               # warn below this (per-cell LiPo sag threshold)
ARM_CONFIRM_TIMEOUT_S = 3         # Start -> Confirm window before reverting
VIDEO_WATCHDOG_PERIOD_MS = 2000   # how often to check that frames are flowing
VIDEO_RECONNECT_MAX_DELAY_S = 8   # backoff cap - retries continue forever
LOG_DIR = os.path.expanduser("~/litewing_logs")

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
#video_area { background-color: #050505; }
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
    """Artificial horizon (attitude indicator) as a GTK panel widget.

    Deliberately a GUI widget, NOT a cairooverlay in the video pipeline:
    a per-frame Python draw callback inside the pipeline couples video
    throughput to the Python GIL - the moment the drone connects (cflib
    thread + setpoint thread + telemetry GUI updates all competing for the
    GIL), frames stall, the ESP32-CAM's HTTP server drops the slow reader,
    and the feed freezes. As a panel widget this redraws on the GUI thread
    at telemetry rate (~10Hz) instead, and the video pipeline stays pure C
    end-to-end - completely immune to Python-side load.

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


class VideoTiltOverlayPainter:
    """Pipeline-embedded tilt reference, drawn directly onto video frames
    via GStreamer's cairooverlay.

    The naive version of this (recompute the full HUD - trig, multiple
    strokes, text glyph outlines - on every single video frame) is what
    caused freezing/disconnects: cflib's thread, the setpoint thread, and
    telemetry GUI updates all compete for the same GIL, and a heavy
    per-frame Python callback loses that fight often enough to stall
    frames, which the ESP32-CAM's HTTP server responds to by dropping the
    connection.

    Fix: the expensive drawing only happens once per REGEN_INTERVAL
    (matched to the 100ms/10Hz telemetry rate) onto a cached ARGB32
    surface. Every other video frame - which can be 2-3x more frequent
    than telemetry - just blits that cached surface, a single cheap
    C-level cairo call instead of dozens of Python-level trig/stroke/text
    calls. This cuts actual GIL-holding work per frame substantially
    without changing what's on screen, since the HUD only needs to look
    as fresh as the telemetry driving it anyway."""

    REGEN_INTERVAL_S = 0.1  # matches the 100ms telemetry period

    def __init__(self):
        self._attitude = (0.0, 0.0)
        self._cache_surface = None
        self._cache_dims = None
        self._last_regen_time = 0.0

    def set_attitude(self, roll_deg, pitch_deg):
        self._attitude = (roll_deg, pitch_deg)

    def _stroke(self, cr, x1, y1, x2, y2, width=2.0, color=(0.0, 1.0, 0.53)):
        cr.set_source_rgba(0, 0, 0, 0.55)
        cr.set_line_width(width + 2.5)
        cr.move_to(x1, y1)
        cr.line_to(x2, y2)
        cr.stroke()
        cr.set_source_rgb(*color)
        cr.set_line_width(width)
        cr.move_to(x1, y1)
        cr.line_to(x2, y2)
        cr.stroke()

    def _show_outlined_text(self, cr, x, y, text, rotation, font_size):
        cr.save()
        cr.translate(x, y)
        cr.rotate(rotation)
        cr.select_font_face("monospace", 0, 1)
        cr.set_font_size(font_size)
        extents = cr.text_extents(text)
        cr.move_to(-extents.width / 2, extents.height / 2)
        cr.text_path(text)
        cr.set_source_rgba(0, 0, 0, 0.75)
        cr.set_line_width(max(2.5, font_size * 0.3))
        cr.stroke_preserve()
        cr.set_source_rgb(0.0, 1.0, 0.53)
        cr.fill()
        cr.restore()

    def paint(self, cr, frame_width, frame_height):
        """Called every video frame. Only regenerates the cached surface
        at REGEN_INTERVAL_S; every other call just blits the existing
        cache - a single cheap C-level composite instead of the full
        trig/stroke/text drawing pass."""
        now = time.monotonic()
        dims = (frame_width, frame_height)
        needs_regen = (
            self._cache_surface is None
            or dims != self._cache_dims
            or (now - self._last_regen_time) >= self.REGEN_INTERVAL_S
        )
        if needs_regen:
            surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, frame_width, frame_height)
            scr = cairo.Context(surface)
            self._draw_marks(scr, frame_width, frame_height)
            self._cache_surface = surface
            self._cache_dims = dims
            self._last_regen_time = now

        cr.save()
        cr.set_source_surface(self._cache_surface, 0, 0)
        cr.paint()
        cr.restore()

    def _draw_marks(self, cr, frame_width, frame_height):
        """The actual drawing pass - only runs at REGEN_INTERVAL_S via
        paint() above, not on every video frame."""
        roll_deg, pitch_deg = self._attitude
        cx, cy = frame_width / 2, frame_height / 2
        radius = min(frame_width, frame_height) * 0.16
        scale = radius / 90.0

        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(roll_deg))
        pixels_per_degree = radius / 45.0
        display_pitch = max(-60.0, min(60.0, pitch_deg))
        pitch_offset = display_pitch * pixels_per_degree

        cr.save()
        cr.translate(0, pitch_offset)
        big = radius * 1.6
        self._stroke(cr, -big, 0, big, 0, width=2.0 * scale)
        for deg in range(-60, 61, 10):
            if deg == 0:
                continue
            y = -deg * pixels_per_degree
            half = radius * 0.28 if deg % 20 == 0 else radius * 0.16
            self._stroke(cr, -half, y, half, y, width=1.3 * scale)
        cr.restore()
        cr.restore()

        label_reach = 60
        max_arc_radius = cy - label_reach
        arc_radius = max(radius * 1.3, min(radius * 2.6, max_arc_radius))
        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(roll_deg))
        for mag in (0, 10, 20, 30, 40):
            for sign in ((1,) if mag == 0 else (-1, 1)):
                theta = math.radians(sign * mag)
                is_labeled = mag % 20 == 0
                tick_len = (11 if is_labeled else 6) * scale
                x1, y1 = arc_radius * math.sin(theta), -arc_radius * math.cos(theta)
                x2 = (arc_radius + tick_len) * math.sin(theta)
                y2 = -(arc_radius + tick_len) * math.cos(theta)
                self._stroke(cr, x1, y1, x2, y2, width=1.8 * scale)
                if is_labeled:
                    label_offset = max(15, 11 * scale)
                    lx = (arc_radius + tick_len + label_offset) * math.sin(theta)
                    ly = -(arc_radius + tick_len + label_offset) * math.cos(theta)
                    label_font_size = max(13, 10 * scale)
                    label_text = f"-{mag}" if sign < 0 and mag != 0 else str(mag)
                    self._show_outlined_text(cr, lx, ly, label_text, theta, label_font_size)
        cr.restore()

        px, py = cx, cy - arc_radius - 4 * scale
        pw, ph = 6 * scale, 10 * scale

        def _pointer_path(pad):
            cr.move_to(px - pw - pad, py - ph - pad)
            cr.line_to(px + pw + pad, py - ph - pad)
            cr.line_to(px, py + pad)
            cr.close_path()

        _pointer_path(1.5 * scale)
        cr.set_source_rgba(0, 0, 0, 0.6)
        cr.fill()
        _pointer_path(0)
        cr.set_source_rgb(1.0, 0.82, 0.05)
        cr.fill()

        cw = 2.2 * scale
        chevron_color = (1.0, 0.82, 0.05)
        self._stroke(cr, cx - 22 * scale, cy + 7 * scale, cx - 4 * scale, cy + 1 * scale, width=cw, color=chevron_color)
        self._stroke(cr, cx - 4 * scale, cy + 1 * scale, cx, cy + 5 * scale, width=cw, color=chevron_color)
        self._stroke(cr, cx, cy + 5 * scale, cx + 4 * scale, cy + 1 * scale, width=cw, color=chevron_color)
        self._stroke(cr, cx + 4 * scale, cy + 1 * scale, cx + 22 * scale, cy + 7 * scale, width=cw, color=chevron_color)
        cr.set_source_rgba(0, 0, 0, 0.6)
        cr.arc(cx, cy, 2.5 * scale, 0, 2 * math.pi)
        cr.fill()
        cr.set_source_rgb(*chevron_color)
        cr.arc(cx, cy, 1.8 * scale, 0, 2 * math.pi)
        cr.fill()


class CameraTile:
    """One cell in the 3x3 camera grid.

    If cam_url is None, this tile NEVER attempts a GStreamer pipeline - it
    just draws a static "NO CAMERA" placeholder once and stays idle
    forever. No wasted reconnect attempts, no phantom errors for the 7
    empty slots.

    If cam_url is set, this tile owns its own xvimagesink pipeline,
    watchdog, and reconnect-with-backoff logic - fully independent of
    every other tile, so one camera stalling or erroring never affects
    the other live camera (or any of the empty ones).
    """

    def __init__(self, app, index, cam_url):
        self.app = app  # DroneVideoApp - for gui_log() only now; HUD is per-tile
        self.index = index
        self.cam_url = cam_url
        self.is_live = cam_url is not None

        self.pipeline = None
        self.running = False
        self.tilt_overlay = VideoTiltOverlayPainter() if self.is_live else None
        self._overlay_frame_size = (640, 480)

        # Per-tile HUD state - independent per camera, not a single global
        # flag anymore. Each live tile gets its own dropdown to control
        # this (see the Gtk.MenuButton/Popover built below).
        self.video_hud_enabled = True if self.is_live else False

        self._frame_count = 0
        self._last_frame_count = -1
        self._reconnect_tries = 0
        self._watchdog_id = None
        self._reconnect_timeout_id = None
        self._user_stopped = False

        # ---- widgets ----
        self.box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        header = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
        label_text = (f"CAM{index+1}: {cam_url.split('/')[2]}" if self.is_live
                      else f"CAM{index+1}: --")
        self.status_lbl = Gtk.Label(label=label_text)
        self.status_lbl.set_name("status_off")
        self.status_lbl.set_xalign(0)
        header.pack_start(self.status_lbl, True, True, 2)

        # Menu dropdown - EVERY tile gets one now, live or not, since drone
        # control (this tile's panel) is independent of whether a camera
        # is actually attached. The panel itself decides real vs dummy.
        self.hud_menu_btn = Gtk.MenuButton(label="Menu ▾")
        self.hud_menu_btn.set_name("hud_dropdown_btn")

        popover = Gtk.Popover()
        popover_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        panel_btn = Gtk.Button(label="☰ Open Control Panel")
        panel_btn.set_relief(Gtk.ReliefStyle.NONE)
        panel_btn.get_child().set_xalign(0)
        panel_btn.connect("clicked", self._on_open_control_panel_clicked)
        popover_box.pack_start(panel_btn, False, False, 0)

        popover_box.show_all()
        popover.add(popover_box)
        self._popover = popover  # kept so we can popdown() after clicking
        self.hud_menu_btn.set_popover(popover)
        header.pack_start(self.hud_menu_btn, False, False, 2)

        self.box.pack_start(header, False, False, 2)

        self.drawing_area = Gtk.DrawingArea()
        self.drawing_area.set_name("video_area")
        self.box.pack_start(self.drawing_area, True, True, 0)

        if self.is_live:
            # Connect BEFORE show_all() realizes it, same reasoning as the
            # original single-camera version.
            self.drawing_area.connect("realize", lambda w: GLib.idle_add(self.start))
        else:
            self.drawing_area.connect("draw", self._draw_placeholder)

    def _draw_placeholder(self, widget, cr):
        w = widget.get_allocated_width()
        h = widget.get_allocated_height()
        cr.set_source_rgb(0.03, 0.03, 0.03)
        cr.paint()
        cr.set_source_rgb(0.22, 0.22, 0.22)
        cr.select_font_face("monospace", 0, 0)
        cr.set_font_size(12)
        text = "NO CAMERA"
        extents = cr.text_extents(text)
        cr.move_to(w / 2 - extents.width / 2, h / 2 + extents.height / 2)
        cr.show_text(text)
        return False

    def build_pipeline(self):
        if self.video_hud_enabled:
            pipe_str = (
                f'souphttpsrc location="{self.cam_url}" is-live=true do-timestamp=true '
                f'! multipartdemux '
                f'! image/jpeg '
                f'! jpegdec '
                f'! videoconvert '
                f'! video/x-raw,format=BGRx '
                f'! cairooverlay name=tiltoverlay '
                f'! videoconvert '
                f'! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
                f'! xvimagesink name=sink sync=false'
            )
        else:
            pipe_str = (
                f'souphttpsrc location="{self.cam_url}" is-live=true do-timestamp=true '
                f'! multipartdemux '
                f'! image/jpeg '
                f'! jpegdec '
                f'! videoconvert '
                f'! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
                f'! xvimagesink name=sink sync=false'
            )
        print(f"[INFO] CAM{self.index+1} pipeline (HUD={self.video_hud_enabled}):\n{pipe_str}\n")
        pipeline = Gst.parse_launch(pipe_str)

        if self.video_hud_enabled:
            tiltoverlay = pipeline.get_by_name('tiltoverlay')
            tiltoverlay.connect('caps-changed', self._on_overlay_caps_changed)
            tiltoverlay.connect('draw', self._on_overlay_draw)

        sink = pipeline.get_by_name('sink')
        gdk_window = self.drawing_area.get_window()
        if gdk_window is None:
            raise RuntimeError(f"CAM{self.index+1} drawing area not realized yet")
        sink.set_window_handle(gdk_window.get_xid())

        sink_pad = sink.get_static_pad('sink')
        sink_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_frame_probe)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        return pipeline

    def _on_frame_probe(self, pad, info):
        self._frame_count += 1
        return Gst.PadProbeReturn.OK

    def _on_overlay_caps_changed(self, overlay, caps):
        struct = caps.get_structure(0)
        w = struct.get_value('width')
        h = struct.get_value('height')
        if w and h:
            self._overlay_frame_size = (w, h)

    def _on_overlay_draw(self, overlay, cr, timestamp, duration):
        w, h = self._overlay_frame_size
        self.tilt_overlay.paint(cr, w, h)

    def start(self):
        if not self.is_live or self._user_stopped:
            return False
        self._frame_count = 0  # reset on every (re)start - see camera_server.py
                                # bug writeup for why this matters
        try:
            self.pipeline = self.build_pipeline()
            self.pipeline.set_state(Gst.State.PLAYING)
            self.running = True
            self.status_lbl.set_text(f"CAM{self.index+1}: {self.cam_url.split('/')[2]}  LIVE")
            self.status_lbl.set_name("status_ok")
            self._start_watchdog()
        except Exception as e:
            self.status_lbl.set_text(f"CAM{self.index+1} failed: {e}")
            self.status_lbl.set_name("status_bad")
        return False  # one-shot idle callback

    def _start_watchdog(self):
        if self._watchdog_id is None:
            self._last_frame_count = -1
            self._watchdog_id = GLib.timeout_add(
                VIDEO_WATCHDOG_PERIOD_MS, self._watchdog_tick)

    def _stop_watchdog(self):
        if self._watchdog_id is not None:
            GLib.source_remove(self._watchdog_id)
            self._watchdog_id = None

    def _watchdog_tick(self):
        if self._user_stopped or not self.running:
            return True
        if self._frame_count == self._last_frame_count:
            self.app.gui_log(f"[CAM{self.index+1}] Feed stalled - reconnecting...")
            self._attempt_reconnect()
        else:
            if self._reconnect_tries:
                self.app.gui_log(f"[CAM{self.index+1}] Feed recovered.")
            self._reconnect_tries = 0
        self._last_frame_count = self._frame_count
        return True

    def _attempt_reconnect(self):
        if self._user_stopped or self._reconnect_timeout_id is not None:
            return
        self.stop()
        self._reconnect_tries += 1
        delay_s = min(2 ** (self._reconnect_tries - 1), VIDEO_RECONNECT_MAX_DELAY_S)
        self.status_lbl.set_text(f"CAM{self.index+1}: reconnecting in {delay_s}s...")
        self.status_lbl.set_name("status_bad")
        self._reconnect_timeout_id = GLib.timeout_add_seconds(delay_s, self._reconnect_fire)

    def _reconnect_fire(self):
        self._reconnect_timeout_id = None
        if not self._user_stopped:
            self.start()
        return False  # one-shot

    def stop(self):
        self._stop_watchdog()
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.running = False

    def toggle(self):
        """Called from the app-level Start/Stop Camera button."""
        if not self.is_live:
            return
        if self._user_stopped:
            self._user_stopped = False
            self._reconnect_tries = 0
            self.status_lbl.set_text(f"CAM{self.index+1}: connecting...")
            self.status_lbl.set_name("status_off")
            GLib.idle_add(self.start)
        else:
            self._user_stopped = True
            if self._reconnect_timeout_id is not None:
                GLib.source_remove(self._reconnect_timeout_id)
                self._reconnect_timeout_id = None
            self.stop()
            self.status_lbl.set_text(f"CAM{self.index+1}: stopped by user")
            self.status_lbl.set_name("status_off")

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[CAM{self.index+1} ERROR] {err.message}\n[DEBUG] {debug}")
            self.app.gui_log(f"[CAM{self.index+1} ERROR] {err.message}")
            self.stop()
            self.status_lbl.set_text(f"CAM{self.index+1} error: {err.message}")
            self.status_lbl.set_name("status_bad")
            self._attempt_reconnect()
        elif t == Gst.MessageType.EOS:
            self.app.gui_log(f"[CAM{self.index+1}] Stream ended.")
            self.stop()
            self._attempt_reconnect()

    def _on_open_control_panel_clicked(self, button):
        self._popover.popdown()
        self.app.show_panel_for_tile(self.index)

    def set_attitude(self, roll, pitch):
        if self.tilt_overlay is not None:
            self.tilt_overlay.set_attitude(roll, pitch)

    def shutdown(self):
        self._user_stopped = True
        if self._reconnect_timeout_id is not None:
            GLib.source_remove(self._reconnect_timeout_id)
            self._reconnect_timeout_id = None
        self.stop()


class DroneLink:
    """Owns ONE drone, end to end: Crazyflie connection, telemetry,
    keepalive thread, AND now its own arm state + its own Connect/Start/
    Stop/E-STOP buttons. Two of these exist (drone1, drone2), each fully
    independent.

    What's genuinely per-drone and lives here: the link, connection
    state, ARM state (this was shared/mirrored before - now it isn't),
    telemetry values, which CameraTile's HUD this drone feeds, and this
    drone's own panel widgets.

    What's still shared across both, deliberately: `app.setpoint`. There
    is only one physical joystick, so there is only one commanded
    roll/pitch/yaw/thrust at any instant - but each drone's
    persistent_stream loop now checks ITS OWN `self.armed` before
    sending that setpoint for real (sending a zero setpoint instead if
    not armed). That's the whole mechanism behind "joystick moves
    whichever drone(s) are currently armed": nothing in the gamepad
    input code needs to know which drones exist or are armed - each
    drone's own stream loop makes that call independently, every 100ms.
    """

    def __init__(self, app, name, uri, tile_index):
        self.app = app
        self.name = name              # "drone1" / "drone2" - used in log lines
        self.uri = uri
        self.tile_index = tile_index  # which CameraTile's HUD this drone feeds

        self.cf = None
        self.connected = False
        self.armed = False
        self.log_conf = None
        self.stop_stream_evt = threading.Event()
        self.raw_attitude = {"roll": 0.0, "pitch": 0.0}
        self.calib_offset = {"roll": 0.0, "pitch": 0.0}

        # Two-step arm state (Start -> "CONFIRM ARM?" -> armed), now
        # per-drone instead of app-global.
        self._arm_pending = False
        self._arm_confirm_timeout_id = None

        # Widgets - attached after _build_ui constructs this drone's
        # panel (see DroneVideoApp._build_drone_subpanel).
        self.status_lbl = None
        self.roll_lbl = None
        self.pitch_lbl = None
        self.battery_lbl = None
        self.connect_btn = None
        self.start_btn = None
        self.stop_btn = None
        self.estop_btn = None

    def attach_widgets(self, status_lbl, roll_lbl, pitch_lbl, battery_lbl,
                        connect_btn, start_btn, stop_btn, estop_btn):
        self.status_lbl = status_lbl
        self.roll_lbl = roll_lbl
        self.pitch_lbl = pitch_lbl
        self.battery_lbl = battery_lbl
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

    # ── Connect (this drone only) ─────────────────────────────────────
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

    # ── Two-step arm (this drone only): Start -> "CONFIRM ARM?" -> armed
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

        # Second click within the window: actually arm THIS drone. Its
        # persistent_stream loop starts sending the real (shared)
        # setpoint on its very next 100ms tick - the other drone is
        # completely unaffected unless it's independently armed too.
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
        """Immediate motor cut for THIS drone only - disarms, cancels any
        pending arm, sends the firmware's stop-setpoint burst. Does NOT
        touch the shared setpoint dict or the other drone - see
        DroneVideoApp.do_emergency_stop for the "kill everything"
        version (spacebar)."""
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
        """Setpoint keepalive for THIS drone, 10Hz. Reads the app's
        SHARED setpoint values (one joystick), but gates on THIS drone's
        OWN armed flag - that's the entire mechanism for "joystick moves
        whichever drone is armed"."""
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

        def log_data_cb(timestamp, data, logconf):
            roll = data.get('stabilizer.roll', 0.0)
            pitch = data.get('stabilizer.pitch', 0.0)
            vbat = data.get('pm.vbat', 0.0)
            self.raw_attitude["roll"] = roll
            self.raw_attitude["pitch"] = pitch
            self._set_attitude_display(roll - self.calib_offset["roll"],
                                        pitch - self.calib_offset["pitch"])
            self._set_battery_display(vbat)

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
        self.app.gui_log(f"[{self.name}] Telemetry started - roll/pitch/battery streaming.")

    def _start_telemetry_no_battery(self):
        log_conf = LogConfig(name=f'AttitudeLogNB_{self.name}', period_in_ms=100)
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')

        def log_data_cb(timestamp, data, logconf):
            roll = data.get('stabilizer.roll', 0.0)
            pitch = data.get('stabilizer.pitch', 0.0)
            self.raw_attitude["roll"] = roll
            self.raw_attitude["pitch"] = pitch
            self._set_attitude_display(roll - self.calib_offset["roll"],
                                        pitch - self.calib_offset["pitch"])

        log_conf.data_received_cb.add_callback(log_data_cb)
        log_conf.error_cb.add_callback(lambda lc, m: self.app.gui_log(f"[LOG ERROR] {self.name}: {m}"))
        self.cf.log.add_config(log_conf)
        log_conf.start()
        self.log_conf = log_conf
        self.app.gui_log(f"[{self.name}] Telemetry started - roll/pitch streaming (no battery var).")

    def stop_telemetry(self):
        if self.log_conf is not None:
            try:
                self.log_conf.stop()
            except Exception:
                pass
            self.log_conf = None

        def _clear():
            if self.roll_lbl is not None:
                self.roll_lbl.set_text("Roll: --")
            if self.pitch_lbl is not None:
                self.pitch_lbl.set_text("Pitch: --")
            if self.battery_lbl is not None:
                self.battery_lbl.set_text("Battery: --")
                self.battery_lbl.set_name("readout")
            self.app.camera_tiles[self.tile_index].set_attitude(0.0, 0.0)
            if self.tile_index == 0:
                self.app.attitude_indicator.set_attitude(0.0, 0.0)
            return False
        GLib.idle_add(_clear)

    def _set_attitude_display(self, roll, pitch):
        def _apply():
            if self.roll_lbl is not None:
                self.roll_lbl.set_text(f"Roll: {roll:.2f}°")
            if self.pitch_lbl is not None:
                self.pitch_lbl.set_text(f"Pitch: {pitch:.2f}°")
            self.app.camera_tiles[self.tile_index].set_attitude(roll, pitch)
            # The side-panel AttitudeIndicator widget only has room to show
            # one drone's attitude - drone1 (tile 0) keeps driving it.
            # Both drones' HUD overlays on their own camera tiles, and
            # both drones' text readouts in their own panel tabs, are
            # always independently live.
            if self.tile_index == 0:
                self.app.attitude_indicator.set_attitude(roll, pitch)
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


class DroneVideoApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="LiteWing Control + Video (GTK, zero-latency)")
        self.set_default_size(1200, 760)

        # ── Drone/cflib state ─────────────────────────────────────────────
        # Two INDEPENDENT links, each with its own connect/arm state (see
        # DroneLink). Only the setpoint dict is shared - one joystick, one
        # commanded roll/pitch/yaw/thrust - but each drone's own `armed`
        # flag decides whether it actually receives that setpoint or a
        # zero one. drone1 feeds camera tile 0 (CAM1), drone2 feeds tile 1
        # (CAM2) - see DroneLink's tile_index.
        self.drone1 = DroneLink(self, "drone1", DRONE_URI, tile_index=0)
        self.drone2 = DroneLink(self, "drone2", DRONE_URI_2, tile_index=1)
        self.drones = [self.drone1, self.drone2]
        self.setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0, "thrust": THRUST_MIN}

        # ── Video/pipeline state ──────────────────────────────────────────
        # 9-tile grid; only the tiles with a real cam_url actually build a
        # pipeline (see _build_ui). Populated there since tiles need
        # widgets built first. HUD is a per-tile setting now (each live
        # tile has its own dropdown), not a single global flag.
        self.camera_tiles = []
        # Always-on, safe: redraws on the GUI thread, never touches any
        # video pipeline. Independent of the camera grid entirely.
        self.attitude_indicator = AttitudeIndicator()

        # ── Panel-tab tracking ──────────────────────────────────────────────
        # Which tile's panel is currently open, so show_panel_for_tile can
        # tell "same tile clicked again -> close" apart from "different
        # tile clicked -> switch tabs but stay open" even though tiles 0
        # and 1 both live inside the same outer "real" panel page.
        self._last_panel_tile_index = None

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
        overlay = Gtk.Overlay()
        self.add(overlay)

        # ---------------- Base layer: 3x3 camera grid ----------------
        # CAM1 (slot 0): real camera + drone1, own tab in the shared panel.
        # CAM2 (slot 1): real camera + drone2, own SEPARATE tab in the
        # same panel. Each tile's "Open Control Panel" opens the panel and
        # switches straight to that drone's tab.
        # CAM3-CAM9: still fully dummy - no camera, no drone.
        cam_urls_for_grid = [CAM_URL, CAM_URL_2] + [None] * 7

        video_grid = Gtk.Grid()
        video_grid.set_row_homogeneous(True)
        video_grid.set_column_homogeneous(True)
        video_grid.set_row_spacing(2)
        video_grid.set_column_spacing(2)

        for i, url in enumerate(cam_urls_for_grid):
            tile = CameraTile(self, i, url)
            self.camera_tiles.append(tile)
            row, col = divmod(i, 3)
            video_grid.attach(tile.box, col, row, 1, 1)

        overlay.add(video_grid)

        # ---------------- Overlay: slide-out control panel ----------------
        self.control_revealer = Gtk.Revealer()
        self.control_revealer.set_transition_type(Gtk.RevealerTransitionType.SLIDE_LEFT)
        self.control_revealer.set_transition_duration(200)
        self.control_revealer.set_halign(Gtk.Align.END)
        self.control_revealer.set_valign(Gtk.Align.FILL)
        self.control_revealer.set_reveal_child(False)

        panel_scroll = Gtk.ScrolledWindow()
        panel_scroll.set_size_request(340, -1)
        panel_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.set_name("control_panel")
        left.set_margin_start(10)
        left.set_margin_end(10)
        left.set_margin_top(44)
        left.set_margin_bottom(10)

        title = Gtk.Label(label="◈ LiteWing Drone Control")
        title.set_name("title_label")
        left.pack_start(title, False, False, 0)

        indep_hint = Gtk.Label(
            label="One joystick/keyboard. Each drone is armed independently "
                  "in its own tab below - the stick moves whichever drone(s) "
                  "are currently armed.")
        indep_hint.set_name("status_off")
        indep_hint.set_xalign(0)
        indep_hint.set_line_wrap(True)
        left.pack_start(indep_hint, False, False, 2)

        # GLOBAL E-STOP - kills BOTH drones regardless of which tab is
        # open, always available, also bound to spacebar. Each drone tab
        # additionally has its OWN local E-STOP that kills just that one.
        self.estop_btn = Gtk.Button(label="■ EMERGENCY STOP BOTH (Space)")
        self.estop_btn.set_name("estop_btn")
        self.estop_btn.connect("clicked", lambda *_: self.do_emergency_stop())
        left.pack_start(self.estop_btn, False, False, 6)

        # ---- Per-drone tab switcher ----
        tab_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
        tab1_btn = Gtk.Button(label="Drone 1")
        tab1_btn.connect("clicked", lambda *_: self._switch_drone_tab(0))
        tab2_btn = Gtk.Button(label="Drone 2")
        tab2_btn.connect("clicked", lambda *_: self._switch_drone_tab(1))
        tab_row.pack_start(tab1_btn, True, True, 0)
        tab_row.pack_start(tab2_btn, True, True, 0)
        left.pack_start(tab_row, False, False, 4)

        self.drone_stack = Gtk.Stack()
        self.drone_stack.set_transition_type(Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        drone1_tab = self._build_drone_subpanel(self.drone1, "DRONE 1")
        drone2_tab = self._build_drone_subpanel(self.drone2, "DRONE 2")
        self.drone_stack.add_named(drone1_tab, "drone0")
        self.drone_stack.add_named(drone2_tab, "drone1")
        left.pack_start(self.drone_stack, False, False, 4)

        # Camera control - independent of the drone connection, toggles
        # BOTH live cameras together
        self.cam_btn = Gtk.Button(label="Stop Cameras")
        self.cam_btn.connect("clicked", lambda *_: self.toggle_cameras())
        left.pack_start(self.cam_btn, False, False, 4)

        # ---- Manual control - SHARED: one physical stick/keyboard, one
        # setpoint dict. Which drone(s) actually move depends on each
        # drone's own armed state (set in its own tab above), not on
        # anything here.
        manual_label = Gtk.Label(label="MANUAL CONTROL (shared setpoint)")
        manual_label.set_name("section_label")
        manual_label.set_xalign(0)
        left.pack_start(manual_label, False, False, 10)

        keys_hint = Gtk.Label(label="Keys: arrows=roll/pitch  W/S=thrust  Space=E-STOP BOTH")
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

        # Log - shared, one history for the whole app
        log_label = Gtk.Label(label="LOG")
        log_label.set_name("section_label")
        log_label.set_xalign(0)
        left.pack_start(log_label, False, False, 10)

        self.log_view = Gtk.TextView()
        self.log_view.set_name("log_view")
        self.log_view.set_editable(False)
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD)
        self.log_view.set_size_request(-1, 160)
        left.pack_start(self.log_view, False, False, 4)

        panel_scroll.add(left)

        # ---- Outer panel stack: "real" = the shared panel built above
        # (title/E-STOP/drone tabs/manual control/log), used by tiles 0
        # and 1. Tiles 2-8 = lightweight dummy placeholders.
        self.panel_stack = Gtk.Stack()
        self.panel_stack.add_named(panel_scroll, "real")

        self.dummy_drone_uris = [f"udp://192.168.0.{20 + i}" for i in range(2, 9)]
        for i, dummy_uri in zip(range(2, 9), self.dummy_drone_uris):
            dummy_panel = self._build_dummy_panel(i, dummy_uri)
            self.panel_stack.add_named(dummy_panel, f"panel{i}")

        self.control_revealer.add(self.panel_stack)
        overlay.add_overlay(self.control_revealer)

        self.show_all()
        self.control_revealer.set_reveal_child(False)

    def _build_drone_subpanel(self, drone, heading):
        """One drone's own tab: target URI, status, Connect/Start/Stop,
        its own local E-STOP, telemetry readouts, and a calibrate button.
        Fully independent of the other drone's tab - none of these
        widgets or callbacks touch the other DroneLink."""
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)

        heading_lbl = Gtk.Label(label=f"— {heading} —")
        heading_lbl.set_name("section_label")
        heading_lbl.set_xalign(0)
        box.pack_start(heading_lbl, False, False, 4)

        uri_lbl = Gtk.Label(label=f"Target: {drone.uri}")
        uri_lbl.set_xalign(0)
        box.pack_start(uri_lbl, False, False, 0)

        status_lbl = Gtk.Label(label="Not connected")
        status_lbl.set_name("status_bad")
        status_lbl.set_xalign(0)
        box.pack_start(status_lbl, False, False, 4)

        connect_btn = Gtk.Button(label="Connect")
        connect_btn.connect("clicked", lambda *_: drone.do_connect())
        box.pack_start(connect_btn, False, False, 4)

        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        start_btn = Gtk.Button(label="Start")
        start_btn.set_sensitive(False)
        start_btn.connect("clicked", lambda *_: drone.do_start())
        stop_btn = Gtk.Button(label="Stop")
        stop_btn.set_sensitive(False)
        stop_btn.connect("clicked", lambda *_: drone.do_stop())
        btn_row.pack_start(start_btn, True, True, 0)
        btn_row.pack_start(stop_btn, True, True, 0)
        box.pack_start(btn_row, False, False, 4)

        estop_btn = Gtk.Button(label=f"■ E-STOP {heading}")
        estop_btn.set_name("estop_btn")
        estop_btn.connect("clicked", lambda *_: self.do_emergency_stop(drones=[drone]))
        box.pack_start(estop_btn, False, False, 4)

        telem_label = Gtk.Label(label="LIVE ATTITUDE")
        telem_label.set_name("section_label")
        telem_label.set_xalign(0)
        box.pack_start(telem_label, False, False, 8)

        roll_lbl = Gtk.Label(label="Roll: --")
        roll_lbl.set_name("readout")
        roll_lbl.set_xalign(0)
        box.pack_start(roll_lbl, False, False, 0)

        pitch_lbl = Gtk.Label(label="Pitch: --")
        pitch_lbl.set_name("readout")
        pitch_lbl.set_xalign(0)
        box.pack_start(pitch_lbl, False, False, 0)

        battery_lbl = Gtk.Label(label="Battery: --")
        battery_lbl.set_name("readout")
        battery_lbl.set_xalign(0)
        box.pack_start(battery_lbl, False, False, 0)

        calib_btn = Gtk.Button(label="Calibrate (zero display)")
        calib_btn.connect("clicked", lambda *_: drone.calibrate())
        box.pack_start(calib_btn, False, False, 4)

        drone.attach_widgets(status_lbl, roll_lbl, pitch_lbl, battery_lbl,
                              connect_btn, start_btn, stop_btn, estop_btn)
        return box

    def _switch_drone_tab(self, tile_index):
        self.drone_stack.set_visible_child_name(f"drone{tile_index}")

    def _build_dummy_panel(self, index, dummy_uri):
        """A lightweight, non-functional placeholder panel for a camera
        tile that doesn't have real drone hardware yet. Shows the same
        overall format as the real panel (title, target IP, status) but
        every control is disabled - nothing here can crash or interfere
        with the real drone."""
        scroll = Gtk.ScrolledWindow()
        scroll.set_size_request(340, -1)
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        box.set_name("control_panel")
        box.set_margin_top(36)
        box.set_margin_start(12)
        box.set_margin_end(12)
        box.set_margin_bottom(12)

        title = Gtk.Label(label=f"◆ LiteWing Drone Control - CAM{index+1}")
        title.set_name("title_label")
        title.set_xalign(0)
        box.pack_start(title, False, False, 0)

        uri_lbl = Gtk.Label(label=f"Target: {dummy_uri}")
        uri_lbl.set_name("readout")
        uri_lbl.set_xalign(0)
        box.pack_start(uri_lbl, False, False, 0)

        status_lbl = Gtk.Label(label="Not connected (dummy slot - no hardware yet)")
        status_lbl.set_name("status_off")
        status_lbl.set_xalign(0)
        box.pack_start(status_lbl, False, False, 6)

        note = Gtk.Label(
            label="This is a placeholder for a future drone/camera pair.\n"
                  "Wire up real hardware at this IP to activate it -\n"
                  "same menu format as CAM1, just not connected yet.")
        note.set_name("status_off")
        note.set_xalign(0)
        note.set_line_wrap(True)
        box.pack_start(note, False, False, 8)

        for label_text in ("Connect", "Start", "Stop", "EMERGENCY STOP"):
            btn = Gtk.Button(label=label_text)
            btn.set_sensitive(False)  # purely visual - dummy slots do nothing
            box.pack_start(btn, False, False, 2)

        scroll.add(box)
        return scroll

    def show_panel_for_tile(self, index):
        """Switches the control-panel revealer to the given tile's panel.
        Tiles 0 and 1 (drone1/drone2) both live inside the same outer
        "real" panel page, but open to DIFFERENT inner drone_stack tabs -
        each drone gets its own independent Connect/Start/Stop/telemetry
        tab now. Tiles 2-8 open their own dummy placeholder page.
        Clicking the same tile's menu again while its panel is already
        open closes the revealer instead of doing nothing."""
        if index in (0, 1):
            target_panel = "real"
            self.drone_stack.set_visible_child_name(f"drone{index}")
        else:
            target_panel = f"panel{index}"

        currently_showing = self.control_revealer.get_reveal_child()
        same_tile_as_before = (self._last_panel_tile_index == index)

        if currently_showing and same_tile_as_before:
            self.control_revealer.set_reveal_child(False)
            self._last_panel_tile_index = None
        else:
            self.panel_stack.set_visible_child_name(target_panel)
            self.control_revealer.set_reveal_child(True)
            self._last_panel_tile_index = index

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
        return False  # let other keys through (e.g. panel interaction)

    # ======================================================================
    # Gamepad flight controls (Quantron QGP-1800, evdev)
    # ======================================================================
    # Same non-blocking philosophy as everything else in this app, but using
    # GLib.IOChannel.unix_new(fd) + channel.add_watch() instead of calling
    # GLib.io_add_watch(fd, ...) directly on a raw integer fd. The direct-fd
    # form is NOT reliably supported across PyGObject versions - on some
    # setups it silently never fires the callback at all (no error, no
    # crash, callback just never runs), which is exactly the symptom seen
    # in earlier testing: stick input printed fine in isolated evdev tests,
    # but nothing arrived once this was wired into the GTK app. Wrapping
    # the fd in an explicit GLib.IOChannel is the documented, portable way
    # to watch a raw fd from GLib's main loop in Python, so this is the
    # one thing changed from the earlier attempt.
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
        # the GTK main loop - it raises BlockingIOError instead, which is
        # already handled in _on_gamepad_event below.
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
        # matter how many raw stick events arrive - see _apply_gamepad_axis
        # for why this is separated from the event handler itself.
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
        Confirmed-working throttle mapping (ported from the isolated RC
        test, rc_drone_isolated_test.py):
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
        # unsigned int (0-65535). No matter what THRUST_MIN/MAX get set to,
        # this guarantees an out-of-range value can never reach cflib and
        # crash the setpoint stream (see THRUST_MAX comment above for what
        # happens when it does).
        return int(max(0, min(65535, thrust)))

    def _on_gamepad_event(self, source, condition):
        """Called by GLib because the IOChannel has data ready - runs on
        the GTK main thread, safe to touch self.setpoint / widgets
        directly here, same as any other GTK callback. Must return True
        to keep being watched."""
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
        # Gated behind BOTH the gamepad's own latched flight-enable flag
        # AND at least one drone being armed - CH1-4 never update the
        # shared setpoint unless BASE4 has been pressed and SOMETHING is
        # armed to actually receive it. Which drone(s) move is decided
        # independently by each DroneLink's own `armed` flag in its own
        # persistent_stream loop, not here.
        any_armed = any(d.armed for d in self.drones)
        if not self.gamepad_flight_enabled or not any_armed:
            # Rate-limited debug log: fires once per reason, not once per
            # axis event (which would flood the log while the stick moves).
            reason = []
            if not self.gamepad_flight_enabled:
                reason.append("gamepad_flight_enabled=False (press BASE4)")
            if not any_armed:
                reason.append("no drone armed (arm at least one via its tab)")
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
        # can report dozens of these per second while a stick is moving,
        # and each redraw is real work on the same thread GStreamer's video
        # pipeline needs free time on (same class of issue as the overlay-
        # draw GIL contention described in the module docstring / §4 of the
        # architecture doc). Just mark the data as changed; a separate
        # 10Hz timer (_gamepad_display_tick) does the actual drawing.
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
        since the last tick - this is the throttle that keeps stick
        movement from flooding the GTK thread with label redraws."""
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
    # cflib / drone logic — each drone is now independent (see DroneLink:
    # do_connect/do_start/do_stop/estop/calibrate all live there and are
    # wired directly to that drone's own tab widgets). This class only
    # keeps the GLOBAL E-STOP (kills both, used by the top-of-panel button
    # and spacebar) and the shared setpoint adjusters (one joystick).
    # ======================================================================
    def do_emergency_stop(self, drones=None):
        """Immediate motor cut. With no argument (spacebar / the top
        "EMERGENCY STOP BOTH" button), kills every connected drone AND
        resets the shared setpoint + disables gamepad flight globally -
        the "something is wrong, stop everything now" button.

        Called with drones=[one_drone] (from that drone's own tab), it
        kills ONLY that drone - the shared setpoint dict is left alone,
        since the other drone may still be armed and flying on it."""
        targets = drones if drones is not None else self.drones
        is_global = drones is None

        if is_global:
            self.gamepad_flight_enabled = False
            self.gamepad_status_lbl.set_text("Gamepad flight: DISABLED")
            self.gamepad_status_lbl.set_name("status_bad")
            # Reset commanded values so a re-arm doesn't jump straight
            # back - safe here because a global stop means nothing should
            # still be reading this setpoint as "real" a moment later.
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
    # Camera grid controls
    # ======================================================================
    def toggle_cameras(self):
        """Toggles every LIVE tile together. Empty placeholder tiles are
        no-ops (CameraTile.toggle() checks is_live internally)."""
        any_stopped = any(t._user_stopped for t in self.camera_tiles if t.is_live)
        for tile in self.camera_tiles:
            tile.toggle()
        if any_stopped:
            self.cam_btn.set_label("Stop Cameras")
            self.gui_log("[VIDEO] Cameras started by user.")
        else:
            self.cam_btn.set_label("Start Cameras")
            self.gui_log("[VIDEO] Cameras stopped by user.")

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
        for tile in self.camera_tiles:
            tile.shutdown()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
        Gtk.main_quit()


def main():
    global DRONE_URI, DRONE_URI_2, CAM_URL, CAM_URL_2
    parser = argparse.ArgumentParser(description="LiteWing 2-drone mirror control + 3x3 camera grid GUI")
    parser.add_argument("--drone", default=DRONE_URI,
                        help=f"Drone 1 link URI (default: {DRONE_URI})")
    parser.add_argument("--drone2", default=DRONE_URI_2,
                        help=f"Drone 2 link URI (default: {DRONE_URI_2})")
    parser.add_argument("--cam", default=CAM_URL,
                        help=f"ESP32-CAM #1 stream URL (default: {CAM_URL})")
    parser.add_argument("--cam2", default=CAM_URL_2,
                        help=f"ESP32-CAM #2 stream URL (default: {CAM_URL_2})")
    args = parser.parse_args()
    DRONE_URI = args.drone
    DRONE_URI_2 = args.drone2
    CAM_URL = args.cam
    CAM_URL_2 = args.cam2

    app = DroneVideoApp()
    Gtk.main()


if __name__ == "__main__":
    main()