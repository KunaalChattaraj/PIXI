#!/usr/bin/env python3
"""
LiteWing Camera Server
=======================
Standalone process. Owns the ESP32-CAM connection, decodes the MJPEG
stream, and republishes raw frames over a local shared-memory socket via
GStreamer's shmsink element - zero-copy on the same machine, so the GUI
process reading from it (lightwing_gtk_with_gamepad.py) keeps the same
"no per-frame Python" advantage the original single-process pipeline had.

WHY THIS EXISTS: splitting camera capture into its own process isolates
it from anything happening in the GUI process (gamepad reads, cflib
telemetry, GTK widget updates). If video lag persists even after ruling
out GUI-thread contention, this removes that thread from the picture
entirely - camera decode now runs on its own process, with its own GIL,
competing with nothing.

Pipeline:
    souphttpsrc (ESP32 MJPEG) -> multipartdemux -> jpegdec -> videoconvert
        -> shmsink (local socket, zero-copy handoff to the GUI process)

This process owns the ESP32 reconnect/backoff logic (same pattern as the
GUI's original watchdog) since it's the only thing actually talking HTTP
to the camera now.

Usage:
    python3 camera_server.py --cam http://10.114.33.110:81/stream \\
                              --socket /tmp/litewing_video.sock

Run this BEFORE starting the GUI (or restart the GUI's camera after this
comes up) - the GUI's shmsrc will keep retrying via its own watchdog if
this isn't up yet, but starting this first avoids the initial wait.
"""

import sys
import signal
import argparse

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

DEFAULT_CAM_URL = "http://10.114.33.110:81/stream"
DEFAULT_SOCKET_PATH = "/tmp/litewing_video.sock"

WATCHDOG_PERIOD_MS = 2000       # how often to check that frames are flowing
STALL_TICKS_BEFORE_RECONNECT = 3  # require 3 consecutive empty checks (~6s)
                                   # before treating it as a real stall, not
                                   # just normal ESP32 frame-rate jitter
RECONNECT_MAX_DELAY_S = 8       # backoff cap - retries continue forever


class CameraServer:
    def __init__(self, cam_url, socket_path):
        self.cam_url = cam_url
        self.socket_path = socket_path

        self.pipeline = None
        self.loop = GLib.MainLoop()

        self._frame_count = 0
        self._last_frame_count = -1
        self._consecutive_stalls = 0
        self._watchdog_id = None
        self._reconnect_tries = 0
        self._reconnect_timeout_id = None

    def build_pipeline(self):
        pipe_str = (
            f'souphttpsrc location="{self.cam_url}" is-live=true do-timestamp=true '
            f'! multipartdemux '
            f'! image/jpeg '
            f'! jpegdec name=decoder '
            f'! videoconvert '
            f'! queue name=preshm leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 '
            f'! shmsink socket-path="{self.socket_path}" '
            f'wait-for-connection=false sync=false shm-size=50000000'
        )
        print(f"[INFO] Camera server pipeline:\n{pipe_str}\n")
        pipeline = Gst.parse_launch(pipe_str)

        # CRITICAL FIX: block shmsink's ALLOCATION query from propagating
        # upstream. By default, a sink can offer its own buffer pool
        # during allocation negotiation, and upstream elements
        # (videoconvert, even jpegdec) may adopt it for zero-copy
        # efficiency. shmsink's pool lives in the shared-memory segment -
        # if no shmsrc reader is attached to drain it, requesting a new
        # buffer from that pool can BLOCK indefinitely. That blocking
        # then propagates all the way back through videoconvert and
        # jpegdec to souphttpsrc, stalling frame delivery even though the
        # ESP32 connection itself is completely healthy (confirmed via a
        # standalone gst-launch-1.0 test with fakesink - steady ~13fps,
        # zero drops). Dropping the ALLOCATION query here forces
        # everything upstream of this point to use ordinary system
        # memory instead, completely decoupling decode from whether a
        # shm reader happens to be connected.
        preshm_queue = pipeline.get_by_name('preshm')
        if preshm_queue is not None:
            src_pad = preshm_queue.get_static_pad('src')
            src_pad.add_probe(Gst.PadProbeType.QUERY_DOWNSTREAM, self._block_allocation_query)

        # IMPORTANT: the frame-count probe sits right after jpegdec, NOT on
        # shmsink. Probing shmsink measures "is the local shared-memory
        # handoff to a reader succeeding" - which depends on whether the
        # GUI process happens to be connected at that instant. If no
        # reader is attached yet (or briefly drops), shmsink's internal
        # buffer can back up, and a probe on ITS pad would then report
        # "stalled" - triggering this watchdog to tear down and rebuild
        # the ENTIRE pipeline, including the perfectly-fine ESP32 HTTP
        # connection. That was the actual bug: local shm delivery hiccups
        # were being misread as "camera is dead," causing a self-inflicted
        # reconnect loop that kept destroying the shm socket out from
        # under the GUI. Probing right after decode measures the thing we
        # actually care about here - is the ESP32 stream itself alive -
        # independent of whether a local reader happens to be attached.
        decoder = pipeline.get_by_name('decoder')
        if decoder is not None:
            src_pad = decoder.get_static_pad('src')
            src_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_frame_probe)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        return pipeline

    def _block_allocation_query(self, pad, info):
        query = info.get_query()
        if query.type == Gst.QueryType.ALLOCATION:
            return Gst.PadProbeReturn.DROP  # force system-memory allocation upstream
        return Gst.PadProbeReturn.OK

    def _on_frame_probe(self, pad, info):
        self._frame_count += 1
        return Gst.PadProbeReturn.OK

    def start(self):
        # Reset the frame counter on every (re)start, including reconnects.
        # Without this, the counter kept accumulating across pipeline
        # rebuilds, so the first watchdog check after any reconnect would
        # always compare a stale nonzero count against the -1 sentinel and
        # falsely report "frames flowing" even if nothing new had arrived
        # yet - only the FOLLOWING check would catch a genuine stall. That
        # produced exactly the false-OK-then-real-stall oscillation seen
        # in testing, regardless of whether the underlying stream was
        # actually healthy. Resetting to 0 here makes the very first
        # post-reconnect check a real one: 0 vs -1 correctly acts as a
        # one-tick grace period for the fresh connection to receive its
        # first frame, and every check after that is comparing real counts.
        self._frame_count = 0
        self._consecutive_stalls = 0
        try:
            self.pipeline = self.build_pipeline()
            self.pipeline.set_state(Gst.State.PLAYING)
            print(f"[OK] Streaming {self.cam_url} -> {self.socket_path}")
            self._start_watchdog()
        except Exception as e:
            print(f"[FAIL] Could not start pipeline: {e}")
            self._schedule_reconnect()

    def stop(self):
        self._stop_watchdog()
        if self._reconnect_timeout_id is not None:
            GLib.source_remove(self._reconnect_timeout_id)
            self._reconnect_timeout_id = None
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

    # ── Watchdog: same stall-detection + backoff pattern as the GUI's
    #    original single-process watchdog, just living here now since this
    #    process owns the real ESP32 connection. ─────────────────────────
    def _start_watchdog(self):
        if self._watchdog_id is None:
            self._last_frame_count = -1
            self._watchdog_id = GLib.timeout_add(WATCHDOG_PERIOD_MS, self._watchdog_tick)

    def _stop_watchdog(self):
        if self._watchdog_id is not None:
            GLib.source_remove(self._watchdog_id)
            self._watchdog_id = None

    def _watchdog_tick(self):
        print(f"[DEBUG] watchdog tick: frame_count={self._frame_count}  "
              f"last_frame_count={self._last_frame_count}  "
              f"consecutive_stalls={self._consecutive_stalls}")
        if self._frame_count == self._last_frame_count:
            self._consecutive_stalls += 1
            if self._consecutive_stalls < STALL_TICKS_BEFORE_RECONNECT:
                print(f"[DEBUG] No new frames this tick ({self._consecutive_stalls}/"
                      f"{STALL_TICKS_BEFORE_RECONNECT}) - waiting to see if it's just jitter.")
                return True  # don't reconnect yet, give it a few more chances
            print(f"[WARN] No new frames for {self._consecutive_stalls} consecutive "
                  f"checks - treating as a real stall.")
            self._stop_watchdog()
            self._schedule_reconnect()
            return False  # this watchdog instance is done; a fresh one
                          # starts once the reconnect succeeds
        self._consecutive_stalls = 0
        self._last_frame_count = self._frame_count
        if self._reconnect_tries:
            print("[OK] Frames flowing again.")
        self._reconnect_tries = 0  # progress resets the backoff
        return True

    def _schedule_reconnect(self):
        if self._reconnect_timeout_id is not None:
            return  # already scheduled
        self._reconnect_tries += 1
        delay_s = min(2 ** (self._reconnect_tries - 1), RECONNECT_MAX_DELAY_S)
        print(f"[INFO] Reconnecting (attempt {self._reconnect_tries}) in {delay_s}s...")
        self._reconnect_timeout_id = GLib.timeout_add_seconds(delay_s, self._reconnect_fire)

    def _reconnect_fire(self):
        self._reconnect_timeout_id = None
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.start()
        return False  # one-shot

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[ERROR] {err} ({debug})")
            self._stop_watchdog()
            self._schedule_reconnect()
        elif t == Gst.MessageType.EOS:
            print("[INFO] Stream ended - reconnecting...")
            self._stop_watchdog()
            self._schedule_reconnect()

    def run(self):
        Gst.init(None)
        self.start()

        def _on_sigint(sig, frame):
            print("\n[INFO] Shutting down camera server...")
            self.stop()
            self.loop.quit()

        signal.signal(signal.SIGINT, _on_sigint)

        try:
            self.loop.run()
        except KeyboardInterrupt:
            self.stop()


def main():
    parser = argparse.ArgumentParser(description="LiteWing camera server (ESP32-CAM -> shared memory)")
    parser.add_argument("--cam", default=DEFAULT_CAM_URL,
                        help=f"ESP32-CAM MJPEG stream URL (default: {DEFAULT_CAM_URL})")
    parser.add_argument("--socket", default=DEFAULT_SOCKET_PATH,
                        help=f"Shared-memory socket path to publish frames on "
                             f"(default: {DEFAULT_SOCKET_PATH})")
    args = parser.parse_args()

    server = CameraServer(args.cam, args.socket)
    server.run()


if __name__ == "__main__":
    main()