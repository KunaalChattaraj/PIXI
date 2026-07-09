#!/usr/bin/env python3
"""
Isolated RC test: Quantron QGP-1800 gamepad -> cflib -> drone motors.
NO GTK, NO camera, NO video pipeline.

THROTTLE MAPPING (custom, per user requirement):
  Stick at CENTER or above  -> thrust = THRUST_MIN (idle, locked, no response)
  Stick pushed DOWN         -> thrust ramps linearly up to THRUST_MAX at full
                                down travel

Only HALF the stick's physical travel does anything - center-to-top is a
dead zone that always reads minimum thrust; center-to-bottom is the live
half that ramps 10000 -> 100000. This is intentional per your spec, not a
bug - if it feels backwards (dead zone on the wrong physical side), flip
THROTTLE_DOWN_IS_RAW_LOW below - that's the only thing you should need to
touch.

Flow:
  1. Connect to the drone over cflib.
  2. Type ARM + Enter in the terminal to allow motor spin-up.
  3. Press BASE4 on the gamepad to enable stick control (latched).
  4. Move sticks - CH1-4 map to roll/pitch/yaw/throttle.
  5. Press BASE3 to disable stick control again (latched).
  6. Ctrl+C at any time -> sends stop-setpoint 3x, closes the link, exits.

Usage:
    python3 rc_drone_isolated_test.py --drone udp://10.114.33.101

Install:
    pip3 install evdev
"""

import sys
import time
import threading
import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie

try:
    import evdev
    from evdev import ecodes as ev_ecodes
except ImportError:
    print("evdev not installed. Run: pip3 install evdev")
    sys.exit(1)


# ── Config (mirrors the main app's constants) ────────────────────────────────
DEFAULT_DRONE_URI = "udp://10.114.33.101"

MAX_TILT = 15.0
THRUST_MIN = 100
THRUST_MAX = 50000
YAW_RATE_MAX = 200

GAMEPAD_DEVICE_NAME_HINT = "microntek"
GAMEPAD_AXIS_MIN, GAMEPAD_AXIS_MAX, GAMEPAD_AXIS_CENTER = 0, 255, 128
GAMEPAD_DEADZONE_RAW = 15

# Roll/pitch/yaw keep the normal symmetric -1..+1 mapping.
GAMEPAD_AXIS_INVERT = {
    "ABS_X": False,   # CH1 roll
    "ABS_Y": False,   # CH2 pitch
    "ABS_Z": False,   # CH3 yaw
}

# Throttle is the special case: only ONE side of the stick's travel from
# center does anything. Set to True if "raw value decreasing toward 0"
# is the physical direction you push the stick DOWN. If the dead zone
# ends up on the wrong side when you test this, just flip this one flag.
THROTTLE_DOWN_IS_RAW_LOW = True

GAMEPAD_BTN_DISABLE_FLIGHT = 296  # BASE3
GAMEPAD_BTN_ENABLE_FLIGHT = 297   # BASE4


class IsolatedRCTest:
    def __init__(self, drone_uri):
        self.drone_uri = drone_uri

        self.connected = False
        self.armed = False               # set True by typing ARM in terminal
        self.flight_enabled = False      # latched by BASE4/BASE3
        self.stop_evt = threading.Event()

        self.setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": THRUST_MIN}

        self.cf = Crazyflie()
        self.cf.connected.add_callback(self._on_connected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.disconnected.add_callback(self._on_disconnected)

        self.gamepad_dev = None
        self._btn_prev = {}

    # ── cflib connection ──────────────────────────────────────────────────
    def _on_connected(self, uri):
        print(f"[OK] Connected: {uri}")
        self.connected = True

    def _on_connection_failed(self, uri, msg):
        print(f"[FAIL] Connection failed: {msg}")
        self.connected = False

    def _on_disconnected(self, uri):
        print(f"[INFO] Disconnected: {uri}")
        self.connected = False
        self.armed = False
        self.flight_enabled = False

    def connect(self, timeout_s=10):
        print(f"Connecting to {self.drone_uri} ...")
        self.cf.open_link(self.drone_uri)
        waited = 0.0
        while not self.connected and waited < timeout_s:
            time.sleep(0.2)
            waited += 0.2
        if not self.connected:
            print("[FAIL] Could not connect within timeout. "
                  "Check the URI and that the drone is powered on.")
            return False
        return True

    # ── setpoint keepalive thread (same 100ms pattern as persistent_stream) ─
    def persistent_stream(self):
        consecutive_errors = 0
        while self.connected and not self.stop_evt.is_set():
            try:
                if self.armed and self.flight_enabled:
                    self.cf.commander.send_setpoint(
                        self.setpoint["roll"], self.setpoint["pitch"],
                        self.setpoint["yaw"], int(self.setpoint["thrust"])
                    )
                else:
                    self.cf.commander.send_setpoint(0, 0, 0, 0)
                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors == 1:
                    print(f"[STREAM ERROR] send_setpoint failed: {e}")
            time.sleep(0.1)
        print("[STREAM] Stopped.")

    # ── gamepad: roll/pitch/yaw (symmetric mapping) ─────────────────────────
    def find_gamepad(self):
        for path in evdev.list_devices():
            d = evdev.InputDevice(path)
            if GAMEPAD_DEVICE_NAME_HINT in d.name.lower():
                return d
        return None

    def normalize_symmetric(self, raw_value, axis_name):
        """Standard -1.0..+1.0 mapping, used for roll/pitch/yaw."""
        centered = raw_value - GAMEPAD_AXIS_CENTER
        if abs(centered) <= GAMEPAD_DEADZONE_RAW:
            norm = 0.0
        else:
            span = GAMEPAD_AXIS_MAX - GAMEPAD_AXIS_CENTER
            norm = max(-1.0, min(1.0, centered / span))
        if GAMEPAD_AXIS_INVERT.get(axis_name, False):
            norm = -norm
        return norm

    def compute_throttle(self, raw_value):
        """
        Custom throttle mapping:
          center or above center -> THRUST_MIN (dead zone, no response)
          below center (down)    -> ramps linearly to THRUST_MAX

        'Below center' here means physically pushed toward THROTTLE_DOWN_IS_RAW_LOW
        direction. If raw is on the "up"/dead-zone side, clamp straight to
        THRUST_MIN. If raw is on the "down"/live side, compute how far
        toward the full-travel end it is (0.0 at center, 1.0 at the
        extreme) and scale THRUST_MIN..THRUST_MAX by that fraction.
        """
        center = GAMEPAD_AXIS_CENTER

        if THROTTLE_DOWN_IS_RAW_LOW:
            # "Down" = raw decreasing from center toward 0.
            if raw_value >= center:
                return THRUST_MIN  # dead zone: at or above center
            travel = center - raw_value          # 0 .. center
            max_travel = center - GAMEPAD_AXIS_MIN  # = center (raw 0 is the extreme)
        else:
            # "Down" = raw increasing from center toward 255.
            if raw_value <= center:
                return THRUST_MIN  # dead zone: at or below center
            travel = raw_value - center            # 0 .. (255-center)
            max_travel = GAMEPAD_AXIS_MAX - center  # raw 255 is the extreme

        if max_travel <= 0:
            return THRUST_MIN  # guard against divide-by-zero on odd hardware ranges

        frac = max(0.0, min(1.0, travel / max_travel))  # 0.0 at center .. 1.0 at extreme
        return THRUST_MIN + frac * (THRUST_MAX - THRUST_MIN)

    def apply_axis(self, code, raw_value):
        if code == ev_ecodes.ABS_X:
            norm = self.normalize_symmetric(raw_value, "ABS_X")
            self.setpoint["roll"] = norm * MAX_TILT
        elif code == ev_ecodes.ABS_Y:
            norm = self.normalize_symmetric(raw_value, "ABS_Y")
            self.setpoint["pitch"] = norm * MAX_TILT
        elif code == ev_ecodes.ABS_Z:
            norm = self.normalize_symmetric(raw_value, "ABS_Z")
            self.setpoint["yaw"] = norm * YAW_RATE_MAX
        elif code == ev_ecodes.ABS_RZ:
            self.setpoint["thrust"] = self.compute_throttle(raw_value)

    def handle_button(self, code, value):
        prev = self._btn_prev.get(code, 0)
        self._btn_prev[code] = value
        if not (value == 1 and prev == 0):
            return  # only act on rising edge (momentary press)

        if code == GAMEPAD_BTN_DISABLE_FLIGHT:
            self.flight_enabled = False
            print("\n[GAMEPAD] BASE3 - flight control DISABLED")
        elif code == GAMEPAD_BTN_ENABLE_FLIGHT:
            self.flight_enabled = True
            print("\n[GAMEPAD] BASE4 - flight control ENABLED")

    def gamepad_loop(self):
        self.gamepad_dev = self.find_gamepad()
        if self.gamepad_dev is None:
            print(f"[FAIL] No gamepad matching '{GAMEPAD_DEVICE_NAME_HINT}' found.")
            print("Devices seen:")
            for path in evdev.list_devices():
                print(f"  {evdev.InputDevice(path).name}")
            return
        print(f"Watching gamepad: {self.gamepad_dev.path} ({self.gamepad_dev.name!r})")

        last_print = 0
        for event in self.gamepad_dev.read_loop():
            if self.stop_evt.is_set():
                break
            if event.type == ev_ecodes.EV_ABS:
                self.apply_axis(event.code, event.value)
            elif event.type == ev_ecodes.EV_KEY:
                self.handle_button(event.code, event.value)

            now = time.time()
            if now - last_print > 0.1:  # throttle terminal spam to 10Hz
                self._print_status()
                last_print = now

    def _print_status(self):
        sp = self.setpoint
        state = "ARMED+ENABLED" if (self.armed and self.flight_enabled) else \
                ("ARMED,gamepad-locked" if self.armed else "NOT ARMED")
        line = (f"\r[{state:20s}] roll={sp['roll']:+6.1f}  pitch={sp['pitch']:+6.1f}  "
                f"yaw={sp['yaw']:+6.1f}  thrust={int(sp['thrust']):6d}   ")
        print(line, end="", flush=True)

    # ── terminal ARM command, on its own thread so it doesn't block gamepad reads ─
    def arm_prompt_loop(self):
        print("\nType ARM + Enter to allow motor spin-up. Type STOP + Enter to disarm.")
        while not self.stop_evt.is_set():
            try:
                cmd = input().strip().upper()
            except EOFError:
                break
            if cmd == "ARM":
                if not self.connected:
                    print("[WARN] Not connected yet - can't arm.")
                    continue
                self.armed = True
                print("[ARMED] Motors will now respond once BASE4 is also pressed.")
            elif cmd == "STOP":
                self.armed = False
                self.flight_enabled = False
                print("[DISARMED] Motors stopped.")

    def emergency_stop(self):
        self.armed = False
        self.flight_enabled = False
        try:
            for _ in range(3):
                self.cf.commander.send_stop_setpoint()
                time.sleep(0.01)
            print("\n[E-STOP] Stop-setpoint sent 3x.")
        except Exception as e:
            print(f"\n[E-STOP] send failed: {e}")

    def run(self):
        cflib.crtp.init_drivers()
        if not self.connect():
            return

        threading.Thread(target=self.persistent_stream, daemon=True).start()
        threading.Thread(target=self.arm_prompt_loop, daemon=True).start()

        try:
            self.gamepad_loop()  # blocks here, this is fine (no GTK loop to protect)
        except KeyboardInterrupt:
            pass
        finally:
            print("\nShutting down...")
            self.emergency_stop()
            self.stop_evt.set()
            time.sleep(0.3)
            try:
                self.cf.close_link()
            except Exception:
                pass
            print("Done.")


def main():
    parser = argparse.ArgumentParser(description="Isolated RC-to-drone test (no GTK, no camera)")
    parser.add_argument("--drone", default=DEFAULT_DRONE_URI,
                        help=f"Drone URI (default: {DEFAULT_DRONE_URI})")
    args = parser.parse_args()

    test = IsolatedRCTest(args.drone)
    test.run()


if __name__ == "__main__":
    main()