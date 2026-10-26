"""
LiteWing GUI - Live Sensor Plotting
=====================================
Author: Dharagesh

Built-in matplotlib dashboards for visualizing drone sensor data.

Usage:
    from litewing import LiteWing
    from litewing.gui import live_dashboard

    live_dashboard(LiteWing("192.168.43.42"))

Available functions:
    live_dashboard(drone)      — Full 4-panel dashboard
    live_height_plot(drone)    — Height: filtered vs raw
    live_imu_plot(drone)       — IMU: roll, pitch, yaw
    live_position_plot(drone)  — XY position trail (dead reckoning)
"""

import time
import threading
from collections import deque

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib import style
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def _check_matplotlib():
    """Raise helpful error if matplotlib is not installed."""
    if not HAS_MATPLOTLIB:
        raise ImportError(
            "matplotlib is required for GUI features.\n"
            "Install it with:  pip install matplotlib"
        )


def _ensure_connected(drone):
    """Connect if not already connected, wait for data."""
    if not drone.is_connected:
        drone.connect()
        time.sleep(2)  # Wait for sensor data to start flowing


def _apply_lightweight_theme():
    """Apply a lightweight and responsive modern theme to matplotlib."""
    plt.rcParams.update({
        "figure.facecolor": "#f8f9fa",
        "axes.facecolor": "#ffffff",
        "axes.edgecolor": "#dee2e6",
        "axes.labelcolor": "#495057",
        "text.color": "#212529",
        "xtick.color": "#adb5bd",
        "ytick.color": "#adb5bd",
        "grid.color": "#e9ecef",
        "grid.alpha": 0.8,
        "grid.linestyle": "--",
        "lines.linewidth": 2.0,
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.titleweight": "bold",
        "figure.titlesize": 14,
        "figure.titleweight": "bold",
    })


# ─── Color palette ───────────────────────────────────────────────────
COLORS = {
    "cyan":    "#1098ad",
    "green":   "#37b24d",
    "yellow":  "#f59f00",
    "red":     "#f03e3e",
    "mauve":   "#845ef7",
    "blue":    "#339af0",
    "peach":   "#f76707",
    "teal":    "#20c997",
    "pink":    "#d6336c",
    "dark":    "#212529",
}


class _DataCollector:
    """Background thread that polls sensor data into deques."""

    def __init__(self, drone, max_points=200, interval_ms=100):
        self.drone = drone
        self.interval = interval_ms / 1000.0
        self.max_points = max_points
        self._running = False
        self._thread = None

        # Data buffers
        self.timestamps = deque(maxlen=max_points)
        self.height = deque(maxlen=max_points)
        self.range_height = deque(maxlen=max_points)
        self.roll = deque(maxlen=max_points)
        self.pitch = deque(maxlen=max_points)
        self.yaw = deque(maxlen=max_points)
        self.vx = deque(maxlen=max_points)
        self.vy = deque(maxlen=max_points)
        self.battery = deque(maxlen=max_points)
        self.gyro_x = deque(maxlen=max_points)
        self.gyro_y = deque(maxlen=max_points)
        self.gyro_z = deque(maxlen=max_points)
        self.x = deque(maxlen=max_points)
        self.y = deque(maxlen=max_points)
        self._start_time = time.time()

    def start(self):
        self._running = True
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._collect_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _collect_loop(self):
        while self._running:
            try:
                s = self.drone.read_sensors()
                t = time.time() - self._start_time
                self.timestamps.append(t)
                self.height.append(s.height)
                self.range_height.append(s.range_height)
                self.roll.append(s.roll)
                self.pitch.append(s.pitch)
                self.yaw.append(s.yaw)
                self.vx.append(s.vx)
                self.vy.append(s.vy)
                self.battery.append(s.battery)
                self.gyro_x.append(s.gyro_x)
                self.gyro_y.append(s.gyro_y)
                self.gyro_z.append(s.gyro_z)
                self.x.append(s.x)
                self.y.append(s.y)
            except Exception:
                pass
            time.sleep(self.interval)


# ─────────────────────────────────────────────────────────────────────
#  PUBLIC FUNCTIONS
# ─────────────────────────────────────────────────────────────────────

def live_dashboard(drone, max_points=200, update_ms=100):
    """
    Open a full sensor dashboard with 4 live-updating subplots:
      - Height (filtered + raw)
      - Attitude (roll, pitch, yaw)
      - Velocity (vx, vy)
      - Battery voltage

    Args:
        drone: LiteWing instance (will auto-connect if needed).
        max_points: Number of data points visible on screen.
        update_ms: Plot refresh interval in milliseconds.
    """
    _check_matplotlib()
    _apply_lightweight_theme()
    _ensure_connected(drone)

    collector = _DataCollector(drone, max_points=max_points, interval_ms=update_ms)
    collector.start()

    # Fast static layout (constrained_layout kills animation performance)
    fig = plt.figure(figsize=(13, 7))
    fig.suptitle("LiteWing - Live Sensor Dashboard", color=COLORS["dark"])
    gs = fig.add_gridspec(2, 3, wspace=0.3, hspace=0.4)

    ax_h = fig.add_subplot(gs[0, 0])
    ax_imu = fig.add_subplot(gs[0, 1])
    ax_vel = fig.add_subplot(gs[1, 0])
    ax_bat = fig.add_subplot(gs[1, 1])
    ax_pos = fig.add_subplot(gs[:, 2])

    # Height subplot
    line_hf, = ax_h.plot([], [], color=COLORS["cyan"], label="Filtered")
    line_hr, = ax_h.plot([], [], color=COLORS["peach"], label="Raw ToF", alpha=0.7)
    ax_h.set_title("Height")
    ax_h.set_ylabel("meters")
    ax_h.legend(loc="upper right", fontsize=8)
    ax_h.grid(True)

    # IMU subplot
    line_r, = ax_imu.plot([], [], color=COLORS["red"], label="Roll")
    line_p, = ax_imu.plot([], [], color=COLORS["green"], label="Pitch")
    line_y, = ax_imu.plot([], [], color=COLORS["yellow"], label="Yaw")
    ax_imu.set_title("Attitude (IMU)")
    ax_imu.set_ylabel("degrees")
    ax_imu.legend(loc="upper right", fontsize=8)
    ax_imu.grid(True)

    # Velocity subplot
    line_vx, = ax_vel.plot([], [], color=COLORS["blue"], label="VX")
    line_vy, = ax_vel.plot([], [], color=COLORS["mauve"], label="VY")
    ax_vel.set_title("Velocity (Optical Flow)")
    ax_vel.set_ylabel("m/s")
    ax_vel.set_xlabel("time (s)")
    ax_vel.legend(loc="upper right", fontsize=8)
    ax_vel.grid(True)

    # Battery subplot
    line_bat, = ax_bat.plot([], [], color=COLORS["green"], linewidth=2.5)
    ax_bat.set_title("Battery")
    ax_bat.set_ylabel("volts")
    ax_bat.set_xlabel("time (s)")
    ax_bat.grid(True)

    # Position subplot
    trail_line, = ax_pos.plot([], [], color=COLORS["cyan"], linewidth=1.5, alpha=0.6)
    current_dot, = ax_pos.plot([], [], 'o', color=COLORS["green"], markersize=10, zorder=5)
    start_dot, = ax_pos.plot([], [], 's', color=COLORS["red"], markersize=10,
                         label="Start", zorder=5)
    ax_pos.set_title("Position Trail")
    ax_pos.set_xlabel("← Right / Left → (m)")
    ax_pos.set_ylabel("← Backward / Forward → (m)")
    ax_pos.legend(loc="upper right", fontsize=8)
    ax_pos.grid(True)
    coord_text = ax_pos.text(
        0.02, 0.98, "", transform=ax_pos.transAxes,
        fontsize=9, verticalalignment='top',
        color=COLORS["dark"],
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffffff', edgecolor='#dee2e6', alpha=0.9)
    )

    def update(frame):
        t = list(collector.timestamps)
        if len(t) < 2:
            return

        # Height
        line_hf.set_data(t, list(collector.height))
        line_hr.set_data(t, list(collector.range_height))
        ax_h.set_xlim(t[0], t[-1])
        h_all = list(collector.height) + list(collector.range_height)
        if h_all:
            ax_h.set_ylim(min(h_all) - 0.05, max(h_all) + 0.05)

        # IMU
        line_r.set_data(t, list(collector.roll))
        line_p.set_data(t, list(collector.pitch))
        line_y.set_data(t, list(collector.yaw))
        ax_imu.set_xlim(t[0], t[-1])
        imu_all = list(collector.roll) + list(collector.pitch) + list(collector.yaw)
        if imu_all:
            margin = max(abs(min(imu_all)), abs(max(imu_all)), 5) * 0.2
            ax_imu.set_ylim(min(imu_all) - margin, max(imu_all) + margin)

        # Velocity
        line_vx.set_data(t, list(collector.vx))
        line_vy.set_data(t, list(collector.vy))
        ax_vel.set_xlim(t[0], t[-1])
        v_all = list(collector.vx) + list(collector.vy)
        if v_all:
            margin = max(abs(min(v_all)), abs(max(v_all)), 0.01) * 0.3
            ax_vel.set_ylim(min(v_all) - margin, max(v_all) + margin)

        # Battery
        line_bat.set_data(t, list(collector.battery))
        ax_bat.set_xlim(t[0], t[-1])
        bat = list(collector.battery)
        if bat:
            bat_f = [v for v in bat if v > 0]
            if bat_f:
                ax_bat.set_ylim(min(bat_f) - 0.1, max(bat_f) + 0.1)

        # Position
        x_screen = [-v for v in collector.y]
        y_screen = list(collector.x)
        if len(x_screen) >= 2:
            trail_line.set_data(x_screen, y_screen)
            current_dot.set_data([x_screen[-1]], [y_screen[-1]])
            start_dot.set_data([x_screen[0]], [y_screen[0]])
            
            x_min, x_max = min(x_screen), max(x_screen)
            y_min, y_max = min(y_screen), max(y_screen)
            x_range = x_max - x_min
            y_range = y_max - y_min
            half = max(x_range, y_range, 0.1) / 2 + 0.05
            cx = (x_max + x_min) / 2
            cy = (y_max + y_min) / 2
            ax_pos.set_xlim(cx - half, cx + half)
            ax_pos.set_ylim(cy - half, cy + half)
            
            coord_text.set_text(f"Forward (X): {collector.x[-1]:.3f} m\nLeft (Y): {collector.y[-1]:.3f} m")

    fig.tight_layout()
    fig.subplots_adjust(top=0.92)
    ani = animation.FuncAnimation(fig, update, interval=update_ms, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        collector.stop()
        drone.disconnect()


def live_height_plot(drone, max_points=200, update_ms=100):
    """
    Open a single live plot showing height data:
      - Kalman-filtered height (stateEstimate.z)
      - Raw ToF laser reading (range.zrange)

    Args:
        drone: LiteWing instance (will auto-connect if needed).
        max_points: Number of data points visible on screen.
        update_ms: Plot refresh interval in milliseconds.
    """
    _check_matplotlib()
    _apply_lightweight_theme()
    _ensure_connected(drone)

    collector = _DataCollector(drone, max_points=max_points, interval_ms=update_ms)
    collector.start()

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle("LiteWing - Live Height", color=COLORS["dark"])

    line_filt, = ax.plot([], [], color=COLORS["cyan"], linewidth=2, label="Filtered (Kalman)")
    line_raw,  = ax.plot([], [], color=COLORS["peach"], linewidth=1.5, alpha=0.7, label="Raw ToF Laser")
    ax.set_ylabel("Height (meters)")
    ax.set_xlabel("Time (seconds)")
    ax.legend(loc="upper right")
    ax.grid(True)

    def update(frame):
        t = list(collector.timestamps)
        if len(t) < 2:
            return
        line_filt.set_data(t, list(collector.height))
        line_raw.set_data(t, list(collector.range_height))
        ax.set_xlim(t[0], t[-1])
        h_all = list(collector.height) + list(collector.range_height)
        if h_all:
            ax.set_ylim(min(h_all) - 0.05, max(h_all) + 0.05)

    fig.tight_layout()
    fig.subplots_adjust(top=0.9)
    ani = animation.FuncAnimation(fig, update, interval=update_ms, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        collector.stop()
        drone.disconnect()


def live_imu_plot(drone, max_points=200, update_ms=100):
    """
    Open a live plot showing IMU attitude data:
      - Roll (tilt left/right)
      - Pitch (tilt forward/back)
      - Yaw (rotation)

    Args:
        drone: LiteWing instance (will auto-connect if needed).
        max_points: Number of data points visible on screen.
        update_ms: Plot refresh interval in milliseconds.
    """
    _check_matplotlib()
    _apply_lightweight_theme()
    _ensure_connected(drone)

    collector = _DataCollector(drone, max_points=max_points, interval_ms=update_ms)
    collector.start()

    fig, (ax_att, ax_gyro) = plt.subplots(2, 1, figsize=(10, 7))
    fig.suptitle("LiteWing - Live IMU Data", color=COLORS["dark"])

    # Attitude
    line_r, = ax_att.plot([], [], color=COLORS["red"], label="Roll")
    line_p, = ax_att.plot([], [], color=COLORS["green"], label="Pitch")
    line_y, = ax_att.plot([], [], color=COLORS["yellow"], label="Yaw")
    ax_att.set_title("Orientation")
    ax_att.set_ylabel("degrees")
    ax_att.legend(loc="upper right", fontsize=8)
    ax_att.grid(True)

    # Gyroscope
    line_gx, = ax_gyro.plot([], [], color=COLORS["blue"], label="Gyro X")
    line_gy, = ax_gyro.plot([], [], color=COLORS["mauve"], label="Gyro Y")
    line_gz, = ax_gyro.plot([], [], color=COLORS["teal"], label="Gyro Z")
    ax_gyro.set_title("Gyroscope")
    ax_gyro.set_ylabel("°/s")
    ax_gyro.set_xlabel("Time (seconds)")
    ax_gyro.legend(loc="upper right", fontsize=8)
    ax_gyro.grid(True)

    def update(frame):
        t = list(collector.timestamps)
        if len(t) < 2:
            return

        # Attitude
        line_r.set_data(t, list(collector.roll))
        line_p.set_data(t, list(collector.pitch))
        line_y.set_data(t, list(collector.yaw))
        ax_att.set_xlim(t[0], t[-1])
        att = list(collector.roll) + list(collector.pitch) + list(collector.yaw)
        if att:
            margin = max(abs(min(att)), abs(max(att)), 5) * 0.2
            ax_att.set_ylim(min(att) - margin, max(att) + margin)

        # Gyroscope
        line_gx.set_data(t, list(collector.gyro_x))
        line_gy.set_data(t, list(collector.gyro_y))
        line_gz.set_data(t, list(collector.gyro_z))
        ax_gyro.set_xlim(t[0], t[-1])
        gyro = list(collector.gyro_x) + list(collector.gyro_y) + list(collector.gyro_z)
        if gyro:
            margin = max(abs(min(gyro)), abs(max(gyro)), 1) * 0.3
            ax_gyro.set_ylim(min(gyro) - margin, max(gyro) + margin)

    fig.tight_layout()
    fig.subplots_adjust(top=0.9, hspace=0.35)
    ani = animation.FuncAnimation(fig, update, interval=update_ms, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        collector.stop()
        drone.disconnect()


def live_position_plot(drone, max_points=500, update_ms=100):
    """
    Open a live 2D XY position plot showing the drone's movement trail.
    Uses dead reckoning from the PMW3901 optical flow sensor.

    The plot shows:
      - Movement trail (fading older positions)
      - Current position (bright dot)
      - Start position marker

    Args:
        drone: LiteWing instance (will auto-connect if needed).
        max_points: Number of trail points to keep.
        update_ms: Plot refresh interval in milliseconds.
    """
    _check_matplotlib()
    _apply_lightweight_theme()
    _ensure_connected(drone)

    collector = _DataCollector(drone, max_points=max_points, interval_ms=update_ms)
    collector.start()

    fig, ax = plt.subplots(figsize=(8, 6))
    fig.suptitle("LiteWing - Position Trail (Dead Reckoning)", color=COLORS["dark"])

    # Trail line
    trail_line, = ax.plot([], [], color=COLORS["cyan"], linewidth=1.5, alpha=0.6)
    # Current position dot
    current_dot, = ax.plot([], [], 'o', color=COLORS["green"], markersize=10, zorder=5)
    # Start marker
    start_dot, = ax.plot([], [], 's', color=COLORS["red"], markersize=10,
                         label="Start", zorder=5)

    ax.set_xlabel("← Right / Left → (meters)")
    ax.set_ylabel("← Backward / Forward → (meters)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True)

    # Text annotation for coordinates
    coord_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes,
        fontsize=10, verticalalignment='top',
        color=COLORS["dark"],
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffffff', edgecolor='#dee2e6', alpha=0.9)
    )

    def update(frame):
        # Map drone coordinates to Screen/Map view:
        # Screen Y (vertical) = Drone X (Forward/Backward)
        # Screen X (horizontal) = -Drone Y (negate so +Left moves dot Left)
        x_screen = [-v for v in collector.y]
        y_screen = list(collector.x)
        if len(x_screen) < 2:
            return

        # Trail
        trail_line.set_data(x_screen, y_screen)

        # Current position
        current_dot.set_data([x_screen[-1]], [y_screen[-1]])

        # Start position
        start_dot.set_data([x_screen[0]], [y_screen[0]])

        # Equal scaling — compute square bounds manually
        x_min, x_max = min(x_screen), max(x_screen)
        y_min, y_max = min(y_screen), max(y_screen)
        x_range = x_max - x_min
        y_range = y_max - y_min
        half = max(x_range, y_range, 0.1) / 2 + 0.05
        cx = (x_max + x_min) / 2
        cy = (y_max + y_min) / 2
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

        # Coordinate text (Still show logical Drone X/Y for clarity)
        coord_text.set_text(f"Forward (X): {collector.x[-1]:.3f} m\nLeft (Y): {collector.y[-1]:.3f} m")

    fig.tight_layout()
    fig.subplots_adjust(top=0.92)
    ani = animation.FuncAnimation(fig, update, interval=update_ms, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        collector.stop()
        drone.disconnect()


# ─────────────────────────────────────────────────────────────────────
#  NON-BLOCKING (BACKGROUND) PLOT FUNCTIONS
# ─────────────────────────────────────────────────────────────────────

class BackgroundPlot:
    """
    A live plot running in a separate process — does NOT block your script.

    Uses ``subprocess.Popen`` to launch a dedicated plot runner, avoiding
    the Windows multiprocessing ``spawn`` re-execution issue. Sensor data
    is streamed to the subprocess via stdin as JSON lines.

    Created by the ``start_live_*`` functions. Call ``.stop()`` when done.

    Example::

        plot = start_live_dashboard(drone)
        # ... fly the drone normally ...
        plot.stop()
    """

    def __init__(self, drone, plot_type, max_points=200, update_ms=100):
        self._drone = drone
        self._plot_type = plot_type
        self._max_points = max_points
        self._update_ms = update_ms
        self._stop = False
        self._feeder_thread = None
        self._process = None

    def start(self):
        """Launch the feeder thread and plot subprocess."""
        import sys
        import subprocess

        # Launch plot runner as a completely separate process
        self._process = subprocess.Popen(
            [sys.executable, "-m", "litewing._plot_runner",
             self._plot_type, str(self._max_points), str(self._update_ms)],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Feeder thread: reads sensors in main process, writes JSON to stdin
        self._feeder_thread = threading.Thread(
            target=self._feed_loop, daemon=True
        )
        self._feeder_thread.start()
        return self

    def stop(self, save_path=None):
        """Stop the plot and clean up. Optionally save the plot before closing."""
        if save_path and self._process:
            import json
            try:
                line = json.dumps({"command": "save", "path": save_path}) + "\n"
                self._process.stdin.write(line.encode("utf-8"))
                self._process.stdin.flush()
                time.sleep(0.5)  # give it a moment to process the command
            except Exception:
                pass

        self._stop = True
        if self._process:
            try:
                self._process.stdin.close()
            except Exception:
                pass
            try:
                self._process.wait(timeout=3)
            except Exception:
                try:
                    self._process.terminate()
                except Exception:
                    pass

    @property
    def is_running(self):
        """True if the plot process is still alive."""
        return self._process is not None and self._process.poll() is None

    def _feed_loop(self):
        """Continuously read sensors and write JSON lines to subprocess stdin."""
        import json
        interval = self._update_ms / 1000.0
        while not self._stop:
            try:
                s = self._drone.read_sensors()
                data = {
                    "height": s.height,
                    "range_height": s.range_height,
                    "roll": s.roll,
                    "pitch": s.pitch,
                    "yaw": s.yaw,
                    "vx": s.vx,
                    "vy": s.vy,
                    "battery": s.battery,
                    "gyro_x": s.gyro_x,
                    "gyro_y": s.gyro_y,
                    "gyro_z": s.gyro_z,
                    "x": s.x,
                    "y": s.y,
                    "time": time.time(),
                }
                line = json.dumps(data) + "\n"
                self._process.stdin.write(line.encode("utf-8"))
                self._process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                break  # Subprocess closed — stop feeding
            except Exception:
                pass
            time.sleep(interval)


# ── Public non-blocking launchers ────────────────────────────────────

def start_live_dashboard(drone, max_points=200, update_ms=100):
    """
    Start a full sensor dashboard in the background (non-blocking).

    Returns:
        BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.

    Example::

        plot = start_live_dashboard(drone)
        drone.arm()
        drone.takeoff()
        drone.hover(5)
        drone.land()
        plot.stop()
    """
    _ensure_connected(drone)
    bp = BackgroundPlot(drone, "dashboard", max_points, update_ms)
    return bp.start()


def start_live_height_plot(drone, max_points=200, update_ms=100):
    """
    Start a live height plot in the background (non-blocking).

    Returns:
        BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.
    """
    _ensure_connected(drone)
    bp = BackgroundPlot(drone, "height", max_points, update_ms)
    return bp.start()


def start_live_imu_plot(drone, max_points=200, update_ms=100):
    """
    Start a live IMU plot in the background (non-blocking).

    Returns:
        BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.
    """
    _ensure_connected(drone)
    bp = BackgroundPlot(drone, "imu", max_points, update_ms)
    return bp.start()


def start_live_position_plot(drone, max_points=500, update_ms=100):
    """
    Start a live XY position trail plot in the background (non-blocking).

    Returns:
        BackgroundPlot: Handle with ``.stop()`` and ``.is_running``.
    """
    _ensure_connected(drone)
    bp = BackgroundPlot(drone, "position", max_points, update_ms)
    return bp.start()

