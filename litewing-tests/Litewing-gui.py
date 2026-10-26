#!/usr/bin/env python3
"""
LiteWing GUI flight controller with immediate abort.  (v2)

Flight runs on a worker thread; the GUI thread stays responsive so
ABORT is always clickable. Spacebar is bound to the same action.

ABORT  -> emergency_stop(): motors cut instantly, DRONE FALLS.
LAND   -> land(): controlled descent. Use this when there is time.

NOTE: litewing installs a SIGINT handler inside connect()/arm(), which
Python forbids off the main thread. We neutralise that below, so
Ctrl+C in the terminal is NOT a kill switch here — use ABORT/SPACE.
"""

import signal
import threading
import time
import tkinter as tk
from tkinter import ttk

# --- must run BEFORE importing litewing ---
_real_signal = signal.signal


def _safe_signal(sig, handler):
    """Allow signal registration only on the main thread; no-op elsewhere."""
    if threading.current_thread() is threading.main_thread():
        return _real_signal(sig, handler)
    return None


signal.signal = _safe_signal

from litewing import LiteWing  # noqa: E402

DRONE_IP = "192.168.0.12"
TARGET_HEIGHT = 0.3
HOVER_SECONDS = 5.0


class FlightGUI:
    def __init__(self, root):
        self.root = root
        self.drone = None
        self.worker = None
        self.aborted = threading.Event()

        root.title("LiteWing Flight Control")
        root.geometry("480x440")

        # --- ABORT: biggest thing on screen, always enabled ---
        self.abort_btn = tk.Button(
            root, text="ABORT  (SPACE)", command=self.abort,
            bg="#c0392b", fg="white", font=("Sans", 22, "bold"),
            activebackground="#e74c3c", height=3,
        )
        self.abort_btn.pack(fill="x", padx=10, pady=10)

        root.bind("<space>", lambda e: self.abort())
        root.bind("<Escape>", lambda e: self.abort())

        self.status = tk.Label(root, text="Not connected",
                               font=("Sans", 13, "bold"))
        self.status.pack(pady=4)

        self.telemetry = tk.Label(root, text="—", font=("Monospace", 11),
                                  justify="left")
        self.telemetry.pack(pady=4)

        btns = ttk.Frame(root)
        btns.pack(pady=8)

        self.connect_btn = ttk.Button(btns, text="Connect",
                                      command=self.connect)
        self.connect_btn.grid(row=0, column=0, padx=4)

        self.dry_btn = ttk.Button(btns, text="Dry run (no motors)",
                                  command=lambda: self.start_flight(True),
                                  state="disabled")
        self.dry_btn.grid(row=0, column=1, padx=4)

        self.fly_btn = ttk.Button(btns, text="FLY",
                                  command=lambda: self.start_flight(False),
                                  state="disabled")
        self.fly_btn.grid(row=0, column=2, padx=4)

        self.land_btn = ttk.Button(btns, text="Land", command=self.land,
                                   state="disabled")
        self.land_btn.grid(row=0, column=3, padx=4)

        self.log = tk.Text(root, height=8, font=("Monospace", 9))
        self.log.pack(fill="both", expand=True, padx=10, pady=8)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.abort_btn.focus_set()

    # ---------- helpers (safe to call from any thread) ----------

    def say(self, msg):
        self.root.after(0, lambda: (self.log.insert("end", msg + "\n"),
                                    self.log.see("end")))

    def set_status(self, msg, color="black"):
        self.root.after(0, lambda: self.status.config(text=msg, fg=color))

    # ---------- connection ----------

    def connect(self):
        self.connect_btn.config(state="disabled")
        self.set_status("Connecting...", "orange")

        def _work():
            try:
                self.drone = LiteWing(DRONE_IP)
                self.drone.connect()
                time.sleep(2)
                self.say(f"Battery: {self.drone.battery:.2f}V")
                self.set_status("Connected", "green")
                self.root.after(0, lambda: (
                    self.dry_btn.config(state="normal"),
                    self.fly_btn.config(state="normal"),
                    self.land_btn.config(state="normal"),
                ))
                self.root.after(0, self.poll_telemetry)
            except Exception as e:
                self.say(f"Connect failed: {e}")
                self.set_status("Connect failed", "red")
                self.root.after(0,
                                lambda: self.connect_btn.config(state="normal"))

        threading.Thread(target=_work, daemon=True).start()

    def poll_telemetry(self):
        """Runs on the GUI thread via after() — never blocks."""
        if self.drone is None:
            return
        try:
            s = self.drone.read_sensors()
            self.telemetry.config(
                text=(f"Bat {s.battery:5.2f}V   H {s.height:6.3f}m\n"
                      f"Pos ({s.x:6.3f}, {s.y:6.3f})m\n"
                      f"RPY ({s.roll:6.1f}, {s.pitch:6.1f}, {s.yaw:6.1f})")
            )
            if s.battery < 3.3:
                self.set_status("BATTERY CRITICAL — LAND NOW", "red")
        except Exception:
            pass
        self.root.after(200, self.poll_telemetry)

    # ---------- flight ----------

    def start_flight(self, dry):
        if self.worker and self.worker.is_alive():
            self.say("Flight already running.")
            return
        self.aborted.clear()

        def _work():
            try:
                self.drone.debug_mode = dry
                self.drone.target_height = TARGET_HEIGHT
                self.drone.hover_duration = HOVER_SECONDS

                self.set_status("DRY RUN" if dry else "ARMING", "orange")
                self.drone.arm()
                if self.aborted.is_set():
                    return

                self.say("Taking off...")
                self.set_status("TAKEOFF", "orange")
                self.drone.takeoff()
                if self.aborted.is_set():
                    return

                self.say("Hovering...")
                self.set_status("HOVERING", "green")
                self.drone.hover(HOVER_SECONDS)
                if self.aborted.is_set():
                    return

                self.say("Landing...")
                self.set_status("LANDING", "orange")
                self.drone.land()
                self.set_status("Landed", "green")
                self.say("Flight complete.")
            except Exception as e:
                self.say(f"Flight error: {e}")
                self.set_status("ERROR", "red")

        self.worker = threading.Thread(target=_work, daemon=True)
        self.worker.start()

    def land(self):
        def _work():
            try:
                self.say("Manual land requested.")
                self.drone.land()
                self.set_status("Landed", "green")
            except Exception as e:
                self.say(f"Land failed: {e}")
        threading.Thread(target=_work, daemon=True).start()

    # ---------- abort ----------

    def abort(self):
        """Called on the GUI thread. Must never block."""
        self.aborted.set()
        self.set_status("*** ABORTED ***", "red")
        self.say("ABORT — motors cut.")
        if self.drone is None:
            return

        def _kill():
            for _ in range(3):          # packet-loss insurance
                try:
                    self.drone.emergency_stop()
                except Exception as e:
                    self.say(f"abort send failed: {e}")
                time.sleep(0.02)

        threading.Thread(target=_kill, daemon=True).start()

    def on_close(self):
        self.abort()
        time.sleep(0.2)
        try:
            if self.drone:
                self.drone.disconnect()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    FlightGUI(root)
    root.mainloop()