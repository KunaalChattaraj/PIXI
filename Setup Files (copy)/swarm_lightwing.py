"""
Two-drone swarm test for LiteWing, built directly on the pattern in
hellow_lightwing.py -- just one Crazyflie() + one link per drone, run
concurrently on separate threads.

Requires the patched udpdriver.py (fixes: honors the URI's IP instead of
a hardcoded address, binds each socket to an OS-assigned local port
instead of a fixed one) copied into your cflib install at:
    <site-packages>/cflib/crtp/udpdriver.py

Each drone gets:
  - its own Crazyflie() instance (cflib does not support one instance
    talking to two links)
  - its own thread, since open_link()/send/close are effectively
    blocking/serial operations per link
  - its own tiny keepalive loop, same idea as `persistent_stream` in the
    architecture doc -- a background 10Hz setpoint sender so the link
    doesn't go idle mid-flight
"""

import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

# One URI per drone -- these must be DIFFERENT IPs. This is exactly what
# the old udpdriver.py could not do (it silently forced every link to
# 192.168.0.12 regardless of what you passed here).
DRONE_URIS = [
    "udp://192.168.0.11:2390",
    "udp://192.168.0.12:2390",
]

THRUST_TEST = 10000   # minimum spin-up thrust, same value as hellow_lightwing.py
HOVER_SECONDS = 1.0


class DroneWorker(threading.Thread):
    """Owns one Crazyflie link end-to-end: connect, unlock, brief thrust
    test, keepalive, controlled shutdown. Mirrors hellow_lightwing.py's
    sequence but wrapped so N of these can run in parallel threads."""

    def __init__(self, uri, name):
        super().__init__(name=name, daemon=True)
        self.uri = uri
        self.cf = Crazyflie()
        self._stop_keepalive = threading.Event()
        self._link_ok = threading.Event()
        # The keepalive thread repeats THIS setpoint, not a hardcoded
        # zero -- otherwise it fights the main sequence's real commands
        # (this was the bug: keepalive was stomping thrust with zero
        # within ~100ms of it being set, which is why motors cut almost
        # immediately).
        self._lock = threading.Lock()
        self._setpoint = (0.0, 0.0, 0, 0)

    def _set_setpoint(self, roll, pitch, yaw, thrust):
        """Update the setpoint that both this thread and the keepalive
        thread will send. Sends it once immediately, then the keepalive
        loop keeps re-sending the same value until it's changed again."""
        with self._lock:
            self._setpoint = (roll, pitch, yaw, thrust)
        self.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

    def _keepalive(self):
        """Background 10Hz setpoint sender -- same role as
        `persistent_stream` in the architecture doc. Repeats the current
        commanded setpoint so the link doesn't go idle, WITHOUT
        overriding whatever the main sequence just commanded."""
        while not self._stop_keepalive.is_set():
            try:
                if not self._link_ok.is_set():
                    time.sleep(0.1)
                    continue
                with self._lock:
                    roll, pitch, yaw, thrust = self._setpoint
                self.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            except Exception as e:
                print(f"[{self.name}] keepalive send failed: {e}")
            time.sleep(0.1)

    def run(self):
        print(f"[{self.name}] connecting to {self.uri} ...")
        self.cf.open_link(self.uri)
        self._link_ok.set()

        keepalive_thread = threading.Thread(target=self._keepalive, daemon=True)
        keepalive_thread.start()

        try:
            time.sleep(1.0)  # settle, same as hellow_lightwing.py

            print(f"[{self.name}] unlocking safety with zero setpoint...")
            self._set_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

            print(f"[{self.name}] spinning motors at minimum thrust...")
            self._set_setpoint(0.0, 0.0, 0, THRUST_TEST)
            time.sleep(HOVER_SECONDS)  # keepalive now holds THRUST_TEST for this whole sleep

            print(f"[{self.name}] stopping motors...")
            self._set_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

        finally:
            self._link_ok.clear()
            self._stop_keepalive.set()
            keepalive_thread.join(timeout=1.0)
            self.cf.close_link()
            print(f"[{self.name}] link closed.")


def main():
    cflib.crtp.init_drivers()

    workers = [
        DroneWorker(uri, name=f"drone{i+1}")
        for i, uri in enumerate(DRONE_URIS)
    ]

    print("Starting swarm test on:", ", ".join(DRONE_URIS))
    for w in workers:
        w.start()

    for w in workers:
        w.join()

    print("Swarm test complete.")


if __name__ == "__main__":
    main()