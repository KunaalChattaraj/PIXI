#!/usr/bin/env python3
"""
LiteWing Drone Control + ESP32-CAM Video — GTK + GStreamer
============================================================
Drone control via cflib, video via xvimagesink rendered directly into a GTK
DrawingArea through set_window_handle(xid) - same pattern as your working
single_cam_GUI.py / dual_cam.py. GStreamer hands frames straight to the X
server; no per-frame Python conversion, no appsink, no image library
round-trip.

GTK's main loop is itself a GLib main loop, so GStreamer's bus messages are
handled natively inside it - no separate thread needed just to pump an
event loop for GStreamer.

cflib's own background thread (started internally by open_link()) still
needs marshaling into the GUI thread for connect/disconnect/telemetry
callbacks - done via GLib.idle_add().
"""

import os
os.environ['GDK_BACKEND'] = 'x11'   # required for get_xid() / video overlay embedding

import time
import math
import threading

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gtk, Gst, GstVideo, GLib, Gdk

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

Gst.init(None)
cflib.crtp.init_drivers()

# ── Config - edit these ──────────────────────────────────────────────────────
DRONE_URI = "udp://10.114.33.101"          # your drone's current IP
CAM_URL = "http://10.114.33.110:81/stream"  # your ESP32-CAM's stream URL - port 81 is the
                                              # standard MJPEG stream port for the ESP32-CAM
                                              # CameraWebServer firmware (port 80 serves the
                                              # control page only, no /stream route there)

# Manual control tuning - same conservative defaults as before
ROLL_PITCH_STEP = 2.0
MAX_TILT = 15.0
THRUST_STEP = 2000
THRUST_MIN = 10000
THRUST_MAX = 100000

# ── CSS styling (matches the look of your camera viewers) ───────────────────
CSS = b"""
* { font-family: monospace; }
window { background-color: #0a0a0a; }
#title_label { color: #00ff88; font-size: 16px; font-weight: bold; padding: 8px 0px 2px 0px; }
#section_label { color: #00ff88; font-size: 11px; font-weight: bold; padding: 4px 0px; }
#status_ok { color: #00ff88; font-size: 11px; font-weight: bold; }
#status_bad { color: #ff4444; font-size: 11px; font-weight: bold; }
#status_off { color: #334433; font-size: 11px; }
#readout { color: #00ff88; font-size: 13px; font-weight: bold; }
#video_area { background-color: #050505; }
#log_view { background-color: #0a0a0a; color: #66cc88; font-family: monospace; font-size: 10px; }
button { font-family: monospace; }
#control_panel { background-color: rgba(10, 10, 10, 0.92); border-left: 1px solid #1a3a1a; }
#menu_toggle { background-color: rgba(10, 10, 10, 0.75); color: #00ff88; border: 1px solid #00aa55;
               font-size: 16px; font-weight: bold; padding: 6px 12px; }
"""


class AttitudeIndicator(Gtk.DrawingArea):
    """Artificial horizon (attitude indicator), same widget verified in the
    standalone test - roll rotates the horizon ball and bank-angle arc
    together, pitch shifts the horizon vertically, and the center chevron
    + top pointer stay fixed as visual references. Call set_attitude(roll,
    pitch) with live degrees to update it; it repaints via queue_draw()."""

    def __init__(self):
        super().__init__()
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.set_size_request(280, 280)
        self.connect("draw", self._on_draw)

    def set_attitude(self, roll_deg, pitch_deg):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.queue_draw()

    def _draw_roll_arc(self, cr, radius):
        cr.set_source_rgb(1, 1, 1)
        cr.select_font_face("monospace", 0, 1)
        cr.set_font_size(10)

        marks = [0, 10, 20, 30, 45, 60]
        for mag in marks:
            for sign in ((1,) if mag == 0 else (-1, 1)):
                theta = math.radians(sign * mag)
                tick_len = 10 if mag % 20 == 0 or mag == 0 else 6

                x1, y1 = radius * math.sin(theta), -radius * math.cos(theta)
                x2, y2 = (radius + tick_len) * math.sin(theta), -(radius + tick_len) * math.cos(theta)
                cr.set_line_width(2)
                cr.move_to(x1, y1)
                cr.line_to(x2, y2)
                cr.stroke()

                if mag == 0 or mag % 10 == 0:
                    label = str(mag)
                    lx = (radius + tick_len + 10) * math.sin(theta)
                    ly = -(radius + tick_len + 10) * math.cos(theta)
                    cr.save()
                    cr.translate(lx, ly)
                    cr.rotate(theta)
                    extents = cr.text_extents(label)
                    cr.move_to(-extents.width / 2, extents.height / 2)
                    cr.show_text(label)
                    cr.restore()

    def _on_draw(self, widget, cr):
        w = widget.get_allocated_width()
        h = widget.get_allocated_height()
        cx, cy = w / 2, h / 2
        radius = min(w, h) / 2 - 38

        cr.set_source_rgb(0.05, 0.05, 0.05)
        cr.paint()

        cr.save()
        cr.arc(cx, cy, radius, 0, 2 * math.pi)
        cr.clip()

        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(self.roll_deg))

        pixels_per_degree = radius / 45.0
        pitch_offset = self.pitch_deg * pixels_per_degree

        cr.save()
        cr.translate(0, pitch_offset)

        big = radius * 3
        cr.set_source_rgb(0.25, 0.55, 0.85)
        cr.rectangle(-big, -big, 2 * big, big)
        cr.fill()

        cr.set_source_rgb(0.30, 0.55, 0.15)
        cr.rectangle(-big, 0, 2 * big, big)
        cr.fill()

        cr.set_source_rgb(1, 1, 1)
        cr.set_line_width(2)
        cr.move_to(-big, 0)
        cr.line_to(big, 0)
        cr.stroke()

        cr.set_line_width(1.5)
        for deg in range(-40, 41, 10):
            if deg == 0:
                continue
            y = -deg * pixels_per_degree
            line_half_width = 26 if deg % 20 == 0 else 14
            cr.move_to(-line_half_width, y)
            cr.line_to(line_half_width, y)
            cr.stroke()

        cr.restore()  # undo pitch translate
        cr.restore()  # undo roll rotate
        cr.restore()  # undo circular clip

        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(self.roll_deg))
        self._draw_roll_arc(cr, radius)
        cr.restore()

        cr.set_source_rgb(1, 0.15, 0.15)
        cr.set_line_width(2)
        px, py = cx, cy - radius - 4
        cr.move_to(px - 6, py - 10)
        cr.line_to(px + 6, py - 10)
        cr.line_to(px, py)
        cr.close_path()
        cr.fill()

        cr.set_source_rgb(1, 0.15, 0.15)
        cr.set_line_width(3)
        cr.move_to(cx - 34, cy + 12)
        cr.line_to(cx - 7, cy + 2)
        cr.line_to(cx, cy + 8)
        cr.line_to(cx + 7, cy + 2)
        cr.line_to(cx + 34, cy + 12)
        cr.stroke()
        cr.arc(cx, cy, 3, 0, 2 * math.pi)
        cr.fill()

        cr.set_source_rgb(0.6, 0.6, 0.6)
        cr.set_line_width(3)
        cr.arc(cx, cy, radius, 0, 2 * math.pi)
        cr.stroke()

        return False


class DroneVideoApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="LiteWing Control + Video (GTK, zero-latency)")
        self.set_default_size(1200, 760)

        # ── Drone/cflib state ──────────────────────────────────────────────
        self.cf = None
        self.connected = False
        self.log_conf = None
        self.stop_stream_evt = threading.Event()
        self.armed = False
        self.setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0, "thrust": THRUST_MIN}
        self.raw_attitude = {"roll": 0.0, "pitch": 0.0}
        self.calib_offset = {"roll": 0.0, "pitch": 0.0}

        # ── Video/pipeline state ──────────────────────────────────────────
        self.video_pipeline = None
        self.video_running = False

        css_provider = Gtk.CssProvider()
        css_provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), css_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self._build_ui()
        self.connect("destroy", self._on_destroy)

    # ======================================================================
    # UI
    # ======================================================================
    def _build_ui(self):
        # Gtk.Overlay lets the video fill the entire window as the base
        # layer, with the control panel and its toggle button floating on
        # top of it instead of taking up permanent screen space.
        overlay = Gtk.Overlay()
        self.add(overlay)

        # ---------------- Base layer: full-window video ----------------
        video_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        self.video_status_lbl = Gtk.Label(label=f"CAM: {CAM_URL.split('/')[2]}  //  connecting...")
        self.video_status_lbl.set_name("status_off")
        self.video_status_lbl.set_xalign(0)
        video_box.pack_start(self.video_status_lbl, False, False, 4)

        self.video_area = Gtk.DrawingArea()
        self.video_area.set_name("video_area")
        # Connect BEFORE show_all() - show_all() realizes widgets immediately,
        # firing "realize" right then. Connecting after show_all() means the
        # signal has already fired and this handler would never trigger.
        self.video_area.connect("realize", self._on_video_area_realize)
        video_box.pack_start(self.video_area, True, True, 0)

        overlay.add(video_box)  # base/main child - fills the whole window

        # ---------------- Overlay: slide-out control panel ----------------
        # (added BEFORE the toggle button, so the button always stacks on
        # top of it for both rendering and click-handling - otherwise, once
        # open, the panel intercepts clicks meant for the toggle button and
        # it can never be closed again.)
        self.control_revealer = Gtk.Revealer()
        self.control_revealer.set_transition_type(Gtk.RevealerTransitionType.SLIDE_LEFT)
        self.control_revealer.set_transition_duration(200)
        self.control_revealer.set_halign(Gtk.Align.END)
        self.control_revealer.set_valign(Gtk.Align.FILL)
        self.control_revealer.set_reveal_child(False)  # start hidden - video-first view

        panel_scroll = Gtk.ScrolledWindow()
        panel_scroll.set_size_request(340, -1)
        panel_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.set_name("control_panel")
        left.set_margin_start(10)
        left.set_margin_end(10)
        left.set_margin_top(44)  # clear the toggle button, now top-right
        left.set_margin_bottom(10)

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

        self.connect_btn = Gtk.Button(label="Connect")
        self.connect_btn.connect("clicked", lambda *_: self.do_connect())
        left.pack_start(self.connect_btn, False, False, 4)

        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.start_btn = Gtk.Button(label="Start")
        self.start_btn.set_sensitive(False)
        self.start_btn.connect("clicked", lambda *_: self.do_start())
        self.stop_btn = Gtk.Button(label="Stop")
        self.stop_btn.set_sensitive(False)
        self.stop_btn.connect("clicked", lambda *_: self.do_stop())
        btn_row.pack_start(self.start_btn, True, True, 0)
        btn_row.pack_start(self.stop_btn, True, True, 0)
        left.pack_start(btn_row, False, False, 6)

        # Telemetry
        telem_label = Gtk.Label(label="LIVE ATTITUDE")
        telem_label.set_name("section_label")
        telem_label.set_xalign(0)
        left.pack_start(telem_label, False, False, 10)

        self.attitude_indicator = AttitudeIndicator()
        indicator_wrap = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        indicator_wrap.set_halign(Gtk.Align.CENTER)
        indicator_wrap.pack_start(self.attitude_indicator, False, False, 0)
        left.pack_start(indicator_wrap, False, False, 4)

        self.roll_lbl = Gtk.Label(label="Roll: --")
        self.roll_lbl.set_name("readout")
        self.roll_lbl.set_xalign(0)
        left.pack_start(self.roll_lbl, False, False, 0)

        self.pitch_lbl = Gtk.Label(label="Pitch: --")
        self.pitch_lbl.set_name("readout")
        self.pitch_lbl.set_xalign(0)
        left.pack_start(self.pitch_lbl, False, False, 0)

        calib_btn = Gtk.Button(label="Calibrate (zero display)")
        calib_btn.connect("clicked", lambda *_: self.do_calibrate())
        left.pack_start(calib_btn, False, False, 4)

        # Manual control
        manual_label = Gtk.Label(label="MANUAL CONTROL")
        manual_label.set_name("section_label")
        manual_label.set_xalign(0)
        left.pack_start(manual_label, False, False, 10)

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
        self.log_view.set_size_request(-1, 160)
        left.pack_start(self.log_view, False, False, 4)

        panel_scroll.add(left)
        self.control_revealer.add(panel_scroll)
        overlay.add_overlay(self.control_revealer)

        # ---------------- Overlay: toggle button ----------------
        # Added LAST (after the revealer) - keeps it topmost over the panel
        # in both z-order and click-handling, so it stays clickable no
        # matter whether the panel is open or closed.
        self.menu_toggle_btn = Gtk.Button(label="☰")
        self.menu_toggle_btn.set_name("menu_toggle")
        self.menu_toggle_btn.set_halign(Gtk.Align.END)
        self.menu_toggle_btn.set_valign(Gtk.Align.START)
        self.menu_toggle_btn.set_margin_end(8)
        self.menu_toggle_btn.set_margin_top(8)
        self.menu_toggle_btn.connect("clicked", lambda *_: self._toggle_control_panel())
        overlay.add_overlay(self.menu_toggle_btn)

        self.show_all()
        # Keep the panel hidden after show_all() forces every widget visible
        self.control_revealer.set_reveal_child(False)

    def _toggle_control_panel(self):
        showing = self.control_revealer.get_reveal_child()
        self.control_revealer.set_reveal_child(not showing)

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
    # Thread-safe GUI helpers - GLib.idle_add() marshals cflib's background-
    # thread callbacks onto the GTK main thread before touching any widget.
    # ======================================================================
    def gui_log(self, msg):
        def _append():
            buf = self.log_view.get_buffer()
            buf.insert(buf.get_end_iter(), msg + "\n")
            self.log_view.scroll_to_iter(buf.get_end_iter(), 0, False, 0, 0)
            return False
        GLib.idle_add(_append)

    def _set_attitude_display(self, roll, pitch):
        def _apply():
            self.roll_lbl.set_text(f"Roll: {roll:.2f}°")
            self.pitch_lbl.set_text(f"Pitch: {pitch:.2f}°")
            self.attitude_indicator.set_attitude(roll, pitch)
            return False
        GLib.idle_add(_apply)

    def _update_cmd_display(self):
        self.thrust_lbl.set_text(f"{self.setpoint['thrust']}")
        self.pitch_cmd_lbl.set_text(f"{self.setpoint['pitch']:.1f}°")
        self.roll_cmd_lbl.set_text(f"{self.setpoint['roll']:.1f}°")

    # ======================================================================
    # cflib / drone logic
    # ======================================================================
    def make_crazyflie(self):
        new_cf = Crazyflie()
        new_cf.connected.add_callback(self._on_connected)
        new_cf.connection_failed.add_callback(self._on_connection_failed)
        new_cf.disconnected.add_callback(self._on_disconnected)
        return new_cf

    def do_connect(self):
        if self.connected:
            self.gui_log("Already connected.")
            return
        self.cf = self.make_crazyflie()
        self.gui_log("Connecting to drone...")
        self.connect_btn.set_sensitive(False)
        self.connect_btn.set_label("Connecting...")
        self.cf.open_link(DRONE_URI)

    def do_start(self):
        if not self.connected:
            self.gui_log("Not connected yet - click Connect first.")
            return
        self.gui_log("Motors ARMED - streaming real setpoint.")
        self.armed = True
        self.start_btn.set_sensitive(False)
        self.stop_btn.set_sensitive(True)

    def do_stop(self):
        self.gui_log("Motors STOPPED - streaming zero setpoint (still connected).")
        self.armed = False
        self.start_btn.set_sensitive(True)
        self.stop_btn.set_sensitive(False)

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

    def do_calibrate(self):
        self.calib_offset["roll"] = self.raw_attitude["roll"]
        self.calib_offset["pitch"] = self.raw_attitude["pitch"]
        self.gui_log(
            f"Display calibrated - offset stored (raw roll={self.raw_attitude['roll']:.2f}, "
            f"raw pitch={self.raw_attitude['pitch']:.2f}). This only affects the GUI display."
        )

    def start_telemetry(self):
        log_conf = LogConfig(name='AttitudeLog', period_in_ms=100)
        try:
            log_conf.add_variable('stabilizer.roll', 'float')
            log_conf.add_variable('stabilizer.pitch', 'float')
        except (KeyError, AttributeError) as e:
            self.gui_log(f"[LOG ERROR] Couldn't find expected variables: {e}")
            return

        def log_data_cb(timestamp, data, logconf):
            roll = data.get('stabilizer.roll', 0.0)
            pitch = data.get('stabilizer.pitch', 0.0)
            self.raw_attitude["roll"] = roll
            self.raw_attitude["pitch"] = pitch
            self._set_attitude_display(roll - self.calib_offset["roll"],
                                        pitch - self.calib_offset["pitch"])

        def log_error_cb(logconf, msg):
            self.gui_log(f"[LOG ERROR] {msg}")

        log_conf.data_received_cb.add_callback(log_data_cb)
        log_conf.error_cb.add_callback(log_error_cb)
        self.cf.log.add_config(log_conf)
        log_conf.start()
        self.log_conf = log_conf
        self.gui_log("Telemetry started - live roll/pitch now streaming.")

    def stop_telemetry(self):
        if self.log_conf is not None:
            try:
                self.log_conf.stop()
            except Exception:
                pass
            self.log_conf = None

        def _clear():
            self.roll_lbl.set_text("Roll: --")
            self.pitch_lbl.set_text("Pitch: --")
            self.attitude_indicator.set_attitude(0.0, 0.0)
            return False
        GLib.idle_add(_clear)

    def persistent_stream(self):
        while self.connected and not self.stop_stream_evt.is_set():
            if self.armed:
                self.cf.commander.send_setpoint(
                    self.setpoint["roll"], self.setpoint["pitch"],
                    self.setpoint["yaw"], self.setpoint["thrust"]
                )
            else:
                self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

    # ── cflib callbacks (fire on cflib's own background thread) ──────────
    def _on_connected(self, link_uri):
        def _apply():
            self.connected = True
            self.gui_log(f"[OK] Connected: {link_uri}")
            self.status_lbl.set_text("Connected")
            self.status_lbl.set_name("status_ok")
            self.connect_btn.set_sensitive(False)
            self.connect_btn.set_label("Connected")
            self.start_btn.set_sensitive(True)
            self.start_telemetry()
            self.stop_stream_evt.clear()
            threading.Thread(target=self.persistent_stream, daemon=True).start()
            return False
        GLib.idle_add(_apply)

    def _on_connection_failed(self, link_uri, msg):
        def _apply():
            self.connected = False
            self.gui_log(f"[FAIL] {msg}")
            self.status_lbl.set_text("Connection failed")
            self.status_lbl.set_name("status_bad")
            self.connect_btn.set_sensitive(True)
            self.connect_btn.set_label("Connect")
            self.start_btn.set_sensitive(False)
            return False
        GLib.idle_add(_apply)

    def _on_disconnected(self, link_uri):
        def _apply():
            self.connected = False
            self.armed = False
            self.stop_stream_evt.set()
            self.gui_log(f"[INFO] Disconnected: {link_uri}")
            self.status_lbl.set_text("Disconnected")
            self.status_lbl.set_name("status_bad")
            self.connect_btn.set_sensitive(True)
            self.connect_btn.set_label("Connect")
            self.start_btn.set_sensitive(False)
            self.stop_btn.set_sensitive(False)
            self.stop_telemetry()
            return False
        GLib.idle_add(_apply)

    # ======================================================================
    # Video (GStreamer, xvimagesink overlay - same pattern as single_cam_GUI.py)
    # ======================================================================
    def _build_video_pipeline(self):
        pipe_str = (
            f'souphttpsrc location="{CAM_URL}" is-live=true do-timestamp=true '
            f'! multipartdemux '
            f'! image/jpeg '
            f'! jpegdec '
            f'! videoconvert '
            f'! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
            f'! xvimagesink name=sink sync=false'
        )
        print(f"[INFO] Video pipeline:\n{pipe_str}\n")

        pipeline = Gst.parse_launch(pipe_str)

        sink = pipeline.get_by_name('sink')
        gdk_window = self.video_area.get_window()
        if gdk_window is None:
            raise RuntimeError("video_area not realized yet - no X window to embed into")
        xid = gdk_window.get_xid()
        sink.set_window_handle(xid)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_video_bus_message)

        return pipeline

    def _on_video_area_realize(self, widget):
        # Widget is realized (its GdkWindow exists) but may not be mapped/
        # sized on screen yet - one more idle_add hop, same as the pattern
        # your own single_cam_GUI.py uses after a button click, ensures
        # we're past that point too before grabbing the XID.
        GLib.idle_add(self._start_video_delayed)

    def _start_video_delayed(self):
        try:
            self.video_pipeline = self._build_video_pipeline()
            self.video_pipeline.set_state(Gst.State.PLAYING)
            self.video_running = True
            self.video_status_lbl.set_text(f"CAM: {CAM_URL.split('/')[2]}  //  LIVE")
            self.video_status_lbl.set_name("status_ok")
        except Exception as e:
            self.video_status_lbl.set_text(f"Video failed: {e}")
            self.video_status_lbl.set_name("status_bad")
        return False  # don't repeat - this is a one-shot idle_add call

    def _stop_video(self):
        if self.video_pipeline:
            self.video_pipeline.set_state(Gst.State.NULL)
            self.video_pipeline = None
        self.video_running = False

    def _on_video_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[VIDEO ERROR] {err.message}\n[DEBUG] {debug}")
            self._stop_video()
            self.video_status_lbl.set_text(f"Video error: {err.message}")
            self.video_status_lbl.set_name("status_bad")
        elif t == Gst.MessageType.EOS:
            self._stop_video()
            self.video_status_lbl.set_text("Video stream ended")
            self.video_status_lbl.set_name("status_bad")

    # ======================================================================
    # Shutdown
    # ======================================================================
    def _on_destroy(self, *args):
        self.armed = False
        self.stop_stream_evt.set()
        time.sleep(0.2)
        self.stop_telemetry()
        if self.cf is not None:
            try:
                self.cf.close_link()
            except Exception:
                pass
        self._stop_video()
        Gtk.main_quit()


def main():
    app = DroneVideoApp()
    Gtk.main()


if __name__ == "__main__":
    main()