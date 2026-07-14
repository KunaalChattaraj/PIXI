#!/usr/bin/env python3
"""
Single ESP32-CAM Viewer — GTK + GStreamer
Step 1: Single camera working perfectly
Step 2: Continuously scan the feed for a QR code; show + stop on detect
Author: Kunal Chattaraj
"""

import os
os.environ['GDK_BACKEND'] = 'x11'   # Force X11 — needed for GStreamer XID

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('GstApp', '1.0')

from gi.repository import Gtk, Gst, GstVideo, GstApp, GLib, Gdk
import sys

import numpy as np
import cv2
import pyzbar.pyzbar as pyzbar

# ── Initialize GStreamer ───────────────────────────────────────────────────────
Gst.init(None)

# ── ESP32 CAM IP — Change this to your actual IP ──────────────────────────────
#CAM1_URL = "http://10.129.231.194:81/stream"
CAM1_URL = "http://192.168.0.100:81/stream"

# ── How often to check the feed for a QR code, in milliseconds ───────────────
QR_SCAN_INTERVAL_MS = 250

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
#qr_result { color: #55aaff; font-size: 12px; padding: 4px 10px; }
"""


class SingleCamApp(Gtk.Window):

    def __init__(self):
        super().__init__(title="ESP32-CAM VIEWER")
        self.set_default_size(680, 660)
        self.set_resizable(False)

        # Pipeline state
        self.pipeline = None
        self.is_running = False
        self.capture_sink = None

        # QR scanning state
        self.qr_scan_id = None      # GLib timeout source id, or None if not scanning
        self.qr_found = False
        self._tick_count = 0
        self._qr_detector = cv2.QRCodeDetector()

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

        # QR result line
        self.qr_lbl = Gtk.Label(label="QR: waiting for camera...")
        self.qr_lbl.set_name("qr_result")
        self.qr_lbl.set_xalign(0)
        self.qr_lbl.set_line_wrap(True)
        main_box.pack_start(self.qr_lbl, False, False, 0)

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
        Display branch: souphttpsrc ! multipartdemux ! jpegdec ! videoconvert
                         ! tee -> queue ! xvimagesink            (what you see)
                                -> queue ! videoconvert ! appsink (what the QR scanner reads)

        The tee lets us repeatedly sample frames for QR decoding without
        touching or slowing down the live view. The appsink branch is capped
        at 1 buffer with drop=True, so it always holds just the newest frame
        and never builds up a backlog — each scan tick just grabs whatever's
        currently there.
        """
        pipe_str = (
            f'souphttpsrc location="{CAM1_URL}" is-live=true do-timestamp=true '
            f'! multipartdemux '
            f'! image/jpeg '
            f'! jpegdec '
            f'! videoconvert '
            f'! tee name=t '
            f't. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
            f'     ! xvimagesink name=sink sync=false '
            f't. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
            f'     ! videoconvert ! video/x-raw,format=BGR '
            f'     ! appsink name=capture_sink emit-signals=false sync=false max-buffers=1 drop=true'
        )

        print(f"[INFO] Starting pipeline:\n{pipe_str}\n")

        self.pipeline = Gst.parse_launch(pipe_str)

        # Embed video into GTK drawing area
        sink = self.pipeline.get_by_name('sink')
        xid  = self.video_area.get_window().get_xid()
        sink.set_window_handle(xid)

        self.capture_sink = self.pipeline.get_by_name('capture_sink')

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
        self.capture_sink = None
        self.is_running = False
        self._stop_qr_scan()

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
            self._start_qr_scan()
        except Exception as e:
            print(f"[ERROR] {e}")
            self.bottom_lbl.set_text(f"FAILED: {e}")
        return False  # Don't repeat

    def _on_stop(self, btn):
        self._stop_pipeline()
        self._set_offline()
        self.bottom_lbl.set_text("STOPPED  //  PRESS START TO BEGIN")
        self.qr_lbl.set_text("QR: waiting for camera...")

    # ── Continuous QR scanning ─────────────────────────────────────────────────
    def _start_qr_scan(self):
        """Begin polling the appsink for a QR code, a few times a second."""
        self.qr_found = False
        self._tick_count = 0
        self.qr_lbl.set_text("QR: scanning...")
        if self.qr_scan_id is None:
            self.qr_scan_id = GLib.timeout_add(QR_SCAN_INTERVAL_MS, self._check_qr_tick)

    def _stop_qr_scan(self):
        if self.qr_scan_id is not None:
            GLib.source_remove(self.qr_scan_id)
            self.qr_scan_id = None

    def _check_qr_tick(self):
        """
        Runs on the GTK main thread every QR_SCAN_INTERVAL_MS (a GLib timeout,
        not a per-video-frame callback on GStreamer's thread — same reasoning
        as the overlay-throttling elsewhere in this app: don't do decode work
        faster than needed, and don't do it on a thread that's fighting for
        the GIL with the video pipeline).
        """
        if not self.is_running or self.capture_sink is None or self.qr_found:
            return False  # stop the timeout

        # Non-blocking grab of whatever frame is currently sitting in the sink.
        sample = self.capture_sink.try_pull_sample(0)
        if sample is None:
            return True  # nothing new yet, keep polling

        img = self._sample_to_bgr_ndarray(sample)
        if img is None:
            return True

        self._tick_count += 1

        data = self._decode_qr(img)

        if data:
            print(f"[QR] Decoded: {data}")
            self.qr_found = True
            self.qr_lbl.set_text(f"QR DETECTED: {data}")
            self.bottom_lbl.set_text("QR DECODED  //  SCAN STOPPED")
            self.qr_scan_id = None
            return False  # stop scanning — we're done

        # Every ~2s, log that we ARE getting frames but no QR yet — helps
        # tell "not detecting" apart from "not even receiving frames".
        if self._tick_count % 8 == 0:
            h, w = img.shape[:2]
            print(f"[QR] scanning... last frame {w}x{h}, no code found yet")

        return True  # keep scanning

    def _decode_qr(self, img_bgr):
        """
        Try pyzbar first — it (zbar) tends to be much more tolerant of
        skew/angle/lighting than OpenCV's built-in QRCodeDetector. Fall back
        to cv2 only if pyzbar isn't available or finds nothing.
        """
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        try:
            results = pyzbar.decode(gray)
        except Exception as e:
            print(f"[WARN] pyzbar decode failed ({e}); falling back to cv2 only")
            results = []

        if results:
            return results[0].data.decode('utf-8', errors='replace')

        # Fallback: cv2's own detector
        data, points, _ = self._qr_detector.detectAndDecode(img_bgr)
        return data if data else None

    # ── Frame conversion helper ────────────────────────────────────────────────
    def _sample_to_bgr_ndarray(self, sample):
        """Convert a GstSample (video/x-raw, format=BGR) into a numpy array."""
        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct = caps.get_structure(0)
        width = struct.get_value('width')
        height = struct.get_value('height')

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            arr = np.frombuffer(mapinfo.data, dtype=np.uint8)
            expected = width * height * 3
            if arr.size < expected:
                return None
            arr = arr[:expected].reshape((height, width, 3))
            return arr.copy()  # copy out before unmapping
        finally:
            buf.unmap(mapinfo)

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