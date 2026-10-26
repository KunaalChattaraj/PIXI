#!/usr/bin/env python3
"""
LiteWing Multi-Camera Viewer - GTK + GStreamer
==============================================
Camera-only version: all drone control (cflib, gamepad, arming, telemetry,
control panel) has been removed. What remains is the 3x3 ESP32-CAM grid.

Video goes through xvimagesink rendered directly into a GTK DrawingArea via
set_window_handle(xid). GStreamer hands frames straight to the X server;
no per-frame Python conversion, no appsink.

Each live tile owns its own pipeline, frame watchdog and reconnect-with-
backoff logic, fully independent of the other tiles. Tiles with no URL
just show a static "NO CAMERA" placeholder and never open a connection.

Usage:
  python3 litewing_camera_grid.py \
      --cam  http://192.168.0.110:81/stream \
      --cam2 http://192.168.0.111:81/stream
  (both optional - defaults below)
"""

import os
os.environ['GDK_BACKEND'] = 'x11'   # required for get_xid() / video overlay embedding

import argparse
from datetime import datetime

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gtk, Gst, GstVideo, GLib, Gdk  # noqa: F401 (GstVideo needed for set_window_handle)

Gst.init(None)

# ── Config - defaults, overridable from the command line ────────────────────
CAM_URL = "http://192.168.0.110:81/stream"    # port 81 = ESP32-CAM MJPEG stream port
                                              # (port 80 serves the control page only)
CAM_URL_2 = "http://192.168.0.111:81/stream"  # second physical ESP32-CAM
# The other 7 tiles in the 3x3 grid have no URL and never attempt a connection.

VIDEO_WATCHDOG_PERIOD_MS = 2000   # how often to check that frames are flowing
VIDEO_RECONNECT_MAX_DELAY_S = 8   # backoff cap - retries continue forever
LOG_DIR = os.path.expanduser("~/litewing_logs")

# ── CSS styling ──────────────────────────────────────────────────────────────
CSS = b"""
* { font-family: monospace; }
window { background-color: #0a0a0a; }
#title_label { color: #00ff88; font-size: 14px; font-weight: bold; padding: 4px 8px; }
#status_ok { color: #00ff88; font-size: 11px; font-weight: bold; }
#status_bad { color: #ff4444; font-size: 11px; font-weight: bold; }
#status_off { color: #334433; font-size: 11px; }
#log_line { color: #66cc88; font-size: 10px; padding: 2px 8px; }
#video_area { background-color: #050505; }
button { font-family: monospace; }
"""


class CameraTile:
    """One cell in the 3x3 camera grid.

    If cam_url is None, this tile NEVER attempts a GStreamer pipeline - it
    just draws a static "NO CAMERA" placeholder and stays idle.

    If cam_url is set, this tile owns its own xvimagesink pipeline,
    watchdog, and reconnect-with-backoff logic - fully independent of
    every other tile.
    """

    def __init__(self, app, index, cam_url):
        self.app = app  # for gui_log() only
        self.index = index
        self.cam_url = cam_url
        self.is_live = cam_url is not None

        self.pipeline = None
        self.running = False

        self._frame_count = 0
        self._last_frame_count = -1
        self._reconnect_tries = 0
        self._watchdog_id = None
        self._reconnect_timeout_id = None
        self._user_stopped = False

        # ---- widgets ----
        self.box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        label_text = (f"CAM{index+1}: {cam_url.split('/')[2]}" if self.is_live
                      else f"CAM{index+1}: --")
        self.status_lbl = Gtk.Label(label=label_text)
        self.status_lbl.set_name("status_off")
        self.status_lbl.set_xalign(0)
        self.box.pack_start(self.status_lbl, False, False, 2)

        self.drawing_area = Gtk.DrawingArea()
        self.drawing_area.set_name("video_area")
        self.box.pack_start(self.drawing_area, True, True, 0)

        if self.is_live:
            # Connect BEFORE show_all() realizes it - the pipeline needs the
            # window's XID, which only exists after realize.
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
        pipe_str = (
            f'souphttpsrc location="{self.cam_url}" is-live=true do-timestamp=true '
            f'! multipartdemux '
            f'! image/jpeg '
            f'! jpegdec '
            f'! videoconvert '
            f'! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
            f'! xvimagesink name=sink sync=false'
        )
        print(f"[INFO] CAM{self.index+1} pipeline:\n{pipe_str}\n")
        pipeline = Gst.parse_launch(pipe_str)

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

    def start(self):
        if not self.is_live or self._user_stopped:
            return False
        self._frame_count = 0  # reset on every (re)start
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
        """Called from the app-level Start/Stop Cameras button."""
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

    def shutdown(self):
        self._user_stopped = True
        if self._reconnect_timeout_id is not None:
            GLib.source_remove(self._reconnect_timeout_id)
            self._reconnect_timeout_id = None
        self.stop()


class CameraGridApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="LiteWing Camera Grid")
        self.set_default_size(1200, 760)

        self.camera_tiles = []

        # ── Persistent file log ───────────────────────────────────────────
        self._log_file = None
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            log_path = os.path.join(
                LOG_DIR, f"camera_{datetime.now():%Y%m%d_%H%M%S}.log")
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

    # ======================================================================
    # UI
    # ======================================================================
    def _build_ui(self):
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        self.add(root)

        # ---- Top bar: title + Start/Stop Cameras ----
        top_bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        title = Gtk.Label(label="◈ LiteWing Camera Grid")
        title.set_name("title_label")
        title.set_xalign(0)
        top_bar.pack_start(title, True, True, 0)

        self.cam_btn = Gtk.Button(label="Stop Cameras")
        self.cam_btn.connect("clicked", lambda *_: self.toggle_cameras())
        top_bar.pack_end(self.cam_btn, False, False, 6)
        root.pack_start(top_bar, False, False, 2)

        # ---- 3x3 camera grid ----
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

        root.pack_start(video_grid, True, True, 0)

        # ---- Bottom status line: latest log message ----
        self.log_line = Gtk.Label(label="")
        self.log_line.set_name("log_line")
        self.log_line.set_xalign(0)
        self.log_line.set_ellipsize(3)  # Pango.EllipsizeMode.END
        root.pack_start(self.log_line, False, False, 2)

        self.show_all()

    # ======================================================================
    # Logging (safe to call from any thread)
    # ======================================================================
    def gui_log(self, msg):
        stamped = f"{datetime.now():%H:%M:%S}  {msg}"
        print(stamped)
        if self._log_file is not None:
            try:
                self._log_file.write(stamped + "\n")
            except OSError:
                pass

        def _apply():
            self.log_line.set_text(stamped)
            return False
        GLib.idle_add(_apply)

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
        for tile in self.camera_tiles:
            tile.shutdown()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
        Gtk.main_quit()


def main():
    global CAM_URL, CAM_URL_2
    parser = argparse.ArgumentParser(description="LiteWing 3x3 ESP32-CAM grid viewer")
    parser.add_argument("--cam", default=CAM_URL,
                        help=f"ESP32-CAM #1 stream URL (default: {CAM_URL})")
    parser.add_argument("--cam2", default=CAM_URL_2,
                        help=f"ESP32-CAM #2 stream URL (default: {CAM_URL_2})")
    args = parser.parse_args()
    CAM_URL = args.cam
    CAM_URL_2 = args.cam2

    CameraGridApp()
    Gtk.main()


if __name__ == "__main__":
    main()