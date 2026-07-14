#!/usr/bin/env python3
"""
Single ESP32-CAM Viewer — GTK + GStreamer
Step 1: Single camera working perfectly
Author: Kunal Chattaraj
"""

import os
os.environ['GDK_BACKEND'] = 'x11'   # Force X11 — needed for GStreamer XID

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import Gtk, Gst, GstVideo, GLib, Gdk
import sys

# ── Initialize GStreamer ───────────────────────────────────────────────────────
Gst.init(None)

# ── ESP32 CAM IP — Change this to your actual IP ──────────────────────────────
#CAM1_URL = "http://10.129.231.194:81/stream"
CAM1_URL = "http://192.168.0.100:81/stream"


# ── CSS Styling ───────────────────────────────────────────────────────────────
CSS = b"""
* { font-family: monospace; }
window { background-color: #0a0a0a; }
#title_label { color: #00ff88; font-size: 20px; font-weight: bold; padding: 12px 0px 2px 0px; }
#subtitle_label { color: #334433; font-size: 11px; padding-bottom: 10px; }
#cam_frame { background-color: #111111; border: 1px solid #1a3a1a; padding: 2px; }
#cam_frame_active { background-color: #111111; border: 2px solid #00ff88; padding: 2px; }
#video_area { background-color: #050505; }
#cam_label { color: #00ff88; font-size: 12px; font-weight: bold; padding: 5px 10px; background-color: #0a0a0a; }
#status_live { color: #00ff88; font-size: 11px; padding: 5px 10px; background-color: #0a0a0a; }
#status_off { color: #334433; font-size: 11px; padding: 5px 10px; background-color: #0a0a0a; }
#btn_start { background-color: #0d1f14; color: #00ff88; border: 1px solid #00aa55; padding: 10px 40px; font-family: monospace; font-size: 13px; font-weight: bold; }
#btn_stop { background-color: #1f0d0d; color: #ff4444; border: 1px solid #aa2222; padding: 10px 40px; font-family: monospace; font-size: 13px; font-weight: bold; }
#bottom_bar { background-color: #050505; padding: 6px 14px; margin-top: 8px; }
#bottom_status { color: #334433; font-size: 10px; }
"""


class SingleCamApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="ESP32-CAM VIEWER")
        self.set_default_size(680, 620)
        self.set_resizable(False)

        # Pipeline state
        self.pipeline = None
        self.is_running = False

        # Apply CSS
        css_provider = Gtk.CssProvider()
        css_provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            css_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self._build_ui()
        self.connect("destroy", self._on_destroy)

    # ── Build UI ───────────────────────────────────────────────────────────────
    def _build_ui(self):
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(main_box)

        # Title
        title = Gtk.Label(label="◈ ESP32-CAM VIEWER")
        title.set_name("title_label")
        main_box.pack_start(title, False, False, 0)

        subtitle = Gtk.Label(label="JETSON ORIN NANO  //  SINGLE CAM MODE")
        subtitle.set_name("subtitle_label")
        main_box.pack_start(subtitle, False, False, 0)

        # Camera frame box
        self.cam_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.cam_box.set_name("cam_frame")

        # Top bar — label + status
        top_bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        cam_label = Gtk.Label(label="◉ CAM-01")
        cam_label.set_name("cam_label")
        cam_label.set_xalign(0)

        self.status_lbl = Gtk.Label(label="● OFFLINE")
        self.status_lbl.set_name("status_off")
        self.status_lbl.set_xalign(1)

        top_bar.pack_start(cam_label, True, True, 0)
        top_bar.pack_start(self.status_lbl, True, True, 0)
        self.cam_box.pack_start(top_bar, False, False, 0)

        # Video drawing area
        self.video_area = Gtk.DrawingArea()
        self.video_area.set_name("video_area")
        self.video_area.set_size_request(640, 480)
        self.video_area.set_double_buffered(False)
        self.cam_box.pack_start(self.video_area, True, True, 0)

        main_box.pack_start(self.cam_box, True, True, 0)

        # Buttons
        btn_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=16)
        btn_box.set_halign(Gtk.Align.CENTER)
        btn_box.set_margin_top(12)
        btn_box.set_margin_bottom(8)

        self.btn_start = Gtk.Button(label="▶  START CAM")
        self.btn_start.set_name("btn_start")
        self.btn_start.connect("clicked", self._on_start)
        btn_box.pack_start(self.btn_start, False, False, 0)

        self.btn_stop = Gtk.Button(label="■  STOP")
        self.btn_stop.set_name("btn_stop")
        self.btn_stop.connect("clicked", self._on_stop)
        btn_box.pack_start(self.btn_stop, False, False, 0)

        main_box.pack_start(btn_box, False, False, 0)

        # Bottom status bar
        bottom = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        bottom.set_name("bottom_bar")

        self.bottom_lbl = Gtk.Label(label="READY  //  PRESS START TO BEGIN")
        self.bottom_lbl.set_name("bottom_status")
        self.bottom_lbl.set_xalign(0)
        bottom.pack_start(self.bottom_lbl, True, True, 0)

        ip_lbl = Gtk.Label(label=f"IP: {CAM1_URL.split('/')[2]}")
        ip_lbl.set_name("bottom_status")
        ip_lbl.set_xalign(1)
        bottom.pack_start(ip_lbl, True, True, 0)

        main_box.pack_end(bottom, False, False, 0)

        self.show_all()

    # ── GStreamer Pipeline ─────────────────────────────────────────────────────
    def _start_pipeline(self):
        """
        Pipeline identical to your working single-cam command:
        souphttpsrc ! multipartdemux ! image/jpeg ! jpegdec ! videoconvert ! xvimagesink
        """
        pipe_str = (
            f'souphttpsrc location="{CAM1_URL}" is-live=true '
            f'! multipartdemux '
            f'! image/jpeg '
            f'! jpegdec '
            f'! videoconvert '
            f'! xvimagesink name=sink sync=false'
        )

        print(f"[INFO] Starting pipeline:\n{pipe_str}\n")

        self.pipeline = Gst.parse_launch(pipe_str)

        # Embed video into GTK drawing area
        sink = self.pipeline.get_by_name('sink')
        xid  = self.video_area.get_window().get_xid()
        sink.set_window_handle(xid)

        # Watch for errors on bus
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.is_running = True

    def _stop_pipeline(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.is_running = False

    # ── Bus Message Handler ────────────────────────────────────────────────────
    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[ERROR] {err.message}")
            print(f"[DEBUG] {debug}")
            self._stop_pipeline()
            self._set_offline()
            self.bottom_lbl.set_text(f"ERROR: {err.message}")
        elif t == Gst.MessageType.EOS:
            print("[INFO] End of stream")
            self._stop_pipeline()
            self._set_offline()
            self.bottom_lbl.set_text("STREAM ENDED")

    # ── Button Handlers ────────────────────────────────────────────────────────
    def _on_start(self, btn):
        if self.is_running:
            return
        self.bottom_lbl.set_text("CONNECTING...")
        # Use idle_add so GTK window is fully realized before getting XID
        GLib.idle_add(self._start_delayed)

    def _start_delayed(self):
        try:
            self._start_pipeline()
            self._set_live()
            self.bottom_lbl.set_text(f"LIVE  //  {CAM1_URL.split('/')[2]}")
        except Exception as e:
            print(f"[ERROR] {e}")
            self.bottom_lbl.set_text(f"FAILED: {e}")
        return False  # Don't repeat

    def _on_stop(self, btn):
        self._stop_pipeline()
        self._set_offline()
        self.bottom_lbl.set_text("STOPPED  //  PRESS START TO BEGIN")

    # ── UI State Helpers ───────────────────────────────────────────────────────
    def _set_live(self):
        self.status_lbl.set_text("● LIVE")
        self.status_lbl.set_name("status_live")
        self.cam_box.set_name("cam_frame_active")

    def _set_offline(self):
        self.status_lbl.set_text("● OFFLINE")
        self.status_lbl.set_name("status_off")
        self.cam_box.set_name("cam_frame")

    def _on_destroy(self, *args):
        self._stop_pipeline()
        Gtk.main_quit()


# ── Main ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = SingleCamApp()
    Gtk.main()
