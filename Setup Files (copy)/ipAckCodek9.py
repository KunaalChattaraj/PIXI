import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie

# Use the IP the drone itself reported at boot (from its serial monitor log)
DRONE_URI = "udp://192.168.43.42"

# Initialize CRTP drivers
cflib.crtp.init_drivers()

# Shared state between the callbacks (which run on a background thread)
# and the main script (which needs to wait for them)
connected_event = threading.Event()
connection_result = {"success": False, "reason": None}


def on_connected(link_uri):
    """Fires ONLY when the drone has actually responded and the
    connection handshake genuinely completed."""
    print(f"[OK] Real connection confirmed: {link_uri}")
    connection_result["success"] = True
    connected_event.set()


def on_connection_failed(link_uri, msg):
    """Fires when cflib gives up waiting for a response."""
    print(f"[FAIL] Connection failed: {msg}")
    connection_result["success"] = False
    connection_result["reason"] = msg
    connected_event.set()


def on_disconnected(link_uri):
    print(f"[INFO] Disconnected from {link_uri}")


def clean_exit(cf, code):
    """Close the link and give its background teardown thread time to
    finish before the process exits, avoiding the noisy interrupted
    traceback we saw before."""
    cf.close_link()
    time.sleep(0.5)
    raise SystemExit(code)


cf = Crazyflie()
cf.connected.add_callback(on_connected)
cf.connection_failed.add_callback(on_connection_failed)
cf.disconnected.add_callback(on_disconnected)

print("Connecting to drone...")
cf.open_link(DRONE_URI)

# Block here until we get a REAL answer (or time out) -
# this replaces the old "assume success and keep going" behavior
print("Waiting for drone to actually respond (up to 5 seconds)...")
got_response = connected_event.wait(timeout=5.0)

if not got_response:
    print("No response at all within 5 seconds. Drone unreachable at this address. Aborting.")
    clean_exit(cf, 1)

if not connection_result["success"]:
    print(f"Drone explicitly rejected/failed the connection: {connection_result['reason']}")
    clean_exit(cf, 1)

# --- Only reachable if the drone genuinely confirmed the connection ---

print("Confirmed real connection. Waiting for stability...")
time.sleep(1.0)

print("Sending zero setpoint to unlock safety...")
cf.commander.send_setpoint(0, 0, 0, 0)
time.sleep(0.1)

# Flight parameters
roll = 0.0
pitch = 0.0
yaw = 0
thrust = 10000  # Thrust value is 10000 minimum and 60000 maximum

print("Starting motors at minimum speed...")
cf.commander.send_setpoint(roll, pitch, yaw, thrust)
time.sleep(1)

print("Stopping motors...")
cf.commander.send_setpoint(0, 0, 0, 0)
time.sleep(0.1)

clean_exit(cf, 0)
print("Test complete")