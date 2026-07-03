import time
import threading
import tkinter as tk
from tkinter import scrolledtext
import cflib.crtp
from cflib.crazyflie import Crazyflie

# Update this to match your drone's current IP
DRONE_URI = "udp://10.228.47.101"

cflib.crtp.init_drivers()

state = {
    "cf": None,
    "connected": False,
    "motors_running": False,
}
stop_stream = threading.Event()


def log(msg):
    """Thread-safe way to write into the GUI log box from a background thread."""
    log_box.after(0, lambda: (log_box.insert(tk.END, msg + "\n"), log_box.see(tk.END)))


def make_crazyflie():
    """A fresh Crazyflie() must be created for every connection attempt -
    cflib starts an internal background thread on open_link(), and a
    Python thread object can only ever be started once, even after it
    finishes. Reusing one instance across multiple Connect clicks causes
    'threads can only be started once'."""
    new_cf = Crazyflie()
    new_cf.connected.add_callback(on_connected)
    new_cf.connection_failed.add_callback(on_connection_failed)
    new_cf.disconnected.add_callback(on_disconnected)
    return new_cf


def on_connected(link_uri):
    state["connected"] = True
    log(f"[OK] Connected: {link_uri}")
    status_var.set("Connected")
    status_label.config(fg="#2e7d32")
    connect_btn.config(state=tk.DISABLED, text="Connected")
    start_btn.config(state=tk.NORMAL)


def on_connection_failed(link_uri, msg):
    state["connected"] = False
    log(f"[FAIL] {msg}")
    status_var.set("Connection failed")
    status_label.config(fg="#c62828")
    connect_btn.config(state=tk.NORMAL, text="Connect")
    start_btn.config(state=tk.DISABLED)


def on_disconnected(link_uri):
    state["connected"] = False
    log(f"[INFO] Disconnected: {link_uri}")
    status_var.set("Disconnected")
    status_label.config(fg="#c62828")
    connect_btn.config(state=tk.NORMAL, text="Connect")
    start_btn.config(state=tk.DISABLED)
    stop_btn.config(state=tk.DISABLED)


def do_connect():
    if state["connected"]:
        log("Already connected.")
        return
    state["cf"] = make_crazyflie()
    log("Connecting to drone...")
    connect_btn.config(state=tk.DISABLED, text="Connecting...")
    state["cf"].open_link(DRONE_URI)


def stream_setpoints():
    """Runs continuously in a background thread while motors are
    'started'. Keeps resending the current setpoint - the firmware's
    safety watchdog disarms the motors if it stops hearing from us, so
    a single one-shot command is not enough to keep them running."""
    cf = state["cf"]
    roll, pitch, yaw, thrust = 0.0, 0.0, 0, 10000
    while not stop_stream.is_set():
        if state["connected"]:
            cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        time.sleep(0.1)
    # Always end with a zero setpoint so motors don't keep spinning
    if state["connected"]:
        cf.commander.send_setpoint(0, 0, 0, 0)


def do_start():
    if not state["connected"]:
        log("Not connected yet - click Connect first.")
        return
    if state["motors_running"]:
        log("Already running.")
        return
    log("Sending zero setpoint to unlock safety...")
    state["cf"].commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)
    log("Starting motors, streaming setpoints continuously...")
    stop_stream.clear()
    state["motors_running"] = True
    threading.Thread(target=stream_setpoints, daemon=True).start()
    start_btn.config(state=tk.DISABLED)
    stop_btn.config(state=tk.NORMAL)


def do_stop():
    if not state["motors_running"]:
        log("Nothing running to stop.")
        return
    log("Stopping motors (connection stays open)...")
    stop_stream.set()
    state["motors_running"] = False
    start_btn.config(state=tk.NORMAL)
    stop_btn.config(state=tk.DISABLED)


def on_close():
    stop_stream.set()
    time.sleep(0.2)
    if state["cf"] is not None:
        try:
            state["cf"].close_link()
        except Exception:
            pass
    root.destroy()


# --- GUI layout ---
root = tk.Tk()
root.title("LiteWing Control Panel")
root.geometry("420x440")

title_label = tk.Label(root, text="LiteWing Drone Control", font=("Segoe UI", 14, "bold"))
title_label.pack(pady=8)

uri_label = tk.Label(root, text=f"Target: {DRONE_URI}", font=("Segoe UI", 9), fg="#555555")
uri_label.pack()

status_var = tk.StringVar(value="Not connected")
status_label = tk.Label(root, textvariable=status_var, font=("Segoe UI", 10, "bold"), fg="#c62828")
status_label.pack(pady=4)

connect_btn = tk.Button(root, text="Connect", width=14, height=1, bg="#1976D2", fg="white",
                         font=("Segoe UI", 10, "bold"), command=do_connect)
connect_btn.pack(pady=6)

btn_frame = tk.Frame(root)
btn_frame.pack(pady=8)

start_btn = tk.Button(btn_frame, text="Start", width=12, height=2, bg="#4CAF50", fg="white",
                       font=("Segoe UI", 11, "bold"), command=do_start, state=tk.DISABLED)
start_btn.grid(row=0, column=0, padx=10)

stop_btn = tk.Button(btn_frame, text="Stop", width=12, height=2, bg="#E53935", fg="white",
                      font=("Segoe UI", 11, "bold"), command=do_stop, state=tk.DISABLED)
stop_btn.grid(row=0, column=1, padx=10)

log_box = scrolledtext.ScrolledText(root, width=48, height=14, font=("Consolas", 9))
log_box.pack(pady=10, padx=10)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()