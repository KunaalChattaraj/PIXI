"""
LiteWing Internal - Plot Runner
=================================
Author: Dharagesh

Standalone subprocess that receives sensor data via stdin (JSON lines)
and renders live matplotlib plots.

This module is launched by BackgroundPlot via subprocess.Popen.
It does NOT import the user's script, avoiding Windows multiprocessing
re-execution issues.

Usage (internal only):
    python -m litewing._plot_runner <plot_type> <max_points> <update_ms>
"""

import sys
import json
import time
from collections import deque

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except ImportError:
    print("matplotlib is required for GUI features.\n"
          "Install it with:  pip install matplotlib", file=sys.stderr)
    sys.exit(1)


def _apply_dark_theme():
    plt.rcParams.update({
        "figure.facecolor": "#1e1e2e",
        "axes.facecolor": "#2a2a3d",
        "axes.edgecolor": "#444466",
        "axes.labelcolor": "#cdd6f4",
        "text.color": "#cdd6f4",
        "xtick.color": "#a6adc8",
        "ytick.color": "#a6adc8",
        "grid.color": "#3a3a5c",
        "grid.alpha": 0.5,
        "lines.linewidth": 1.8,
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.titleweight": "bold",
        "figure.titlesize": 14,
        "figure.titleweight": "bold",
    })


COLORS = {
    "cyan": "#89dceb", "green": "#a6e3a1", "yellow": "#f9e2af",
    "red": "#f38ba8", "mauve": "#cba6f7", "blue": "#89b4fa",
    "peach": "#fab387", "teal": "#94e2d5", "pink": "#f5c2e7",
}


def run_plot(plot_type, max_points, update_ms):
    """Main entry point: read JSON lines from stdin and plot live data."""
    _apply_dark_theme()

    # Data buffers
    timestamps = deque(maxlen=max_points)
    buf = {k: deque(maxlen=max_points) for k in [
        "height", "range_height", "roll", "pitch", "yaw",
        "vx", "vy", "battery", "gyro_x", "gyro_y", "gyro_z", "x", "y",
    ]}
    start_time = [None]
    _stdin_closed = [False]
    _save_paths = []

    def _drain_stdin():
        """Non-blocking read of all available JSON lines from stdin."""
        if _stdin_closed[0]:
            return
        import select
        import os
        # On Windows, select doesn't work on stdin. Use a buffered approach.
        while True:
            try:
                line = _pending_lines.pop(0)
            except IndexError:
                break
            try:
                data = json.loads(line)
                if start_time[0] is None:
                    start_time[0] = data["time"]
                timestamps.append(data["time"] - start_time[0])
                for key in buf:
                    buf[key].append(data.get(key, 0.0))
            except (json.JSONDecodeError, KeyError):
                pass

    # Buffer for lines read by the reader thread
    _pending_lines = []
    _lock = __import__("threading").Lock()

    def _reader_thread():
        """Background thread that reads stdin line by line."""
        try:
            for line in sys.stdin:
                line = line.strip()
                if line:
                    with _lock:
                        _pending_lines.append(line)
        except (ValueError, OSError):
            pass
        _stdin_closed[0] = True

    # Start reader thread
    import threading
    reader = threading.Thread(target=_reader_thread, daemon=True)
    reader.start()

    def _drain():
        """Drain pending lines under lock."""
        with _lock:
            lines = _pending_lines[:]
            _pending_lines.clear()
        for line in lines:
            try:
                data = json.loads(line)
                if data.get("command") == "save":
                    if "path" in data:
                        _save_paths.append(data["path"])
                    continue
                
                if start_time[0] is None:
                    start_time[0] = data["time"]
                timestamps.append(data["time"] - start_time[0])
                for key in buf:
                    buf[key].append(data.get(key, 0.0))
            except (json.JSONDecodeError, KeyError):
                pass

    # ── Build figure ─────────────────────────────────────────

    if plot_type == "dashboard":
        fig = plt.figure(figsize=(15, 8))
        fig.suptitle("LiteWing — Live Dashboard (Background)", color=COLORS["cyan"])
        gs = fig.add_gridspec(2, 3, wspace=0.3, hspace=0.4)
        ax_h = fig.add_subplot(gs[0, 0])
        ax_imu = fig.add_subplot(gs[0, 1])
        ax_vel = fig.add_subplot(gs[1, 0])
        ax_bat = fig.add_subplot(gs[1, 1])
        ax_pos = fig.add_subplot(gs[:, 2])

        line_hf, = ax_h.plot([], [], color=COLORS["cyan"], label="Filtered")
        line_hr, = ax_h.plot([], [], color=COLORS["peach"], label="Raw ToF", alpha=0.7)
        ax_h.set_title("Height"); ax_h.set_ylabel("meters")
        ax_h.legend(loc="upper right", fontsize=8); ax_h.grid(True)

        line_r, = ax_imu.plot([], [], color=COLORS["red"], label="Roll")
        line_p, = ax_imu.plot([], [], color=COLORS["green"], label="Pitch")
        line_y, = ax_imu.plot([], [], color=COLORS["yellow"], label="Yaw")
        ax_imu.set_title("Attitude (IMU)"); ax_imu.set_ylabel("degrees")
        ax_imu.legend(loc="upper right", fontsize=8); ax_imu.grid(True)

        line_vx, = ax_vel.plot([], [], color=COLORS["blue"], label="VX")
        line_vy, = ax_vel.plot([], [], color=COLORS["mauve"], label="VY")
        ax_vel.set_title("Velocity"); ax_vel.set_ylabel("m/s")
        ax_vel.set_xlabel("time (s)")
        ax_vel.legend(loc="upper right", fontsize=8); ax_vel.grid(True)

        line_bat, = ax_bat.plot([], [], color=COLORS["green"], linewidth=2.5)
        ax_bat.set_title("Battery"); ax_bat.set_ylabel("volts")
        ax_bat.set_xlabel("time (s)"); ax_bat.grid(True)

        trail_line, = ax_pos.plot([], [], color=COLORS["cyan"], linewidth=1.5, alpha=0.6)
        current_dot, = ax_pos.plot([], [], 'o', color=COLORS["green"], markersize=10, zorder=5)
        start_dot, = ax_pos.plot([], [], 's', color=COLORS["red"], markersize=10, label="Start", zorder=5)
        ax_pos.set_title("Position Trail")
        ax_pos.set_xlabel("← Right / Left → (m)")
        ax_pos.set_ylabel("← Backward / Forward → (m)")
        ax_pos.legend(loc="upper right", fontsize=8); ax_pos.grid(True)
        coord_text = ax_pos.text(
            0.02, 0.98, "", transform=ax_pos.transAxes, fontsize=9,
            verticalalignment='top', color=COLORS["green"],
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#2a2a3d', alpha=0.8)
        )

        def update(frame):
            _drain()
            if _stdin_closed[0] and len(timestamps) == 0:
                plt.close("all")
                return
            t = list(timestamps)
            if len(t) < 2:
                return
            line_hf.set_data(t, list(buf["height"]))
            line_hr.set_data(t, list(buf["range_height"]))
            ax_h.set_xlim(t[0], t[-1])
            h_all = list(buf["height"]) + list(buf["range_height"])
            if h_all:
                ax_h.set_ylim(min(h_all) - 0.05, max(h_all) + 0.05)

            line_r.set_data(t, list(buf["roll"]))
            line_p.set_data(t, list(buf["pitch"]))
            line_y.set_data(t, list(buf["yaw"]))
            ax_imu.set_xlim(t[0], t[-1])
            imu = list(buf["roll"]) + list(buf["pitch"]) + list(buf["yaw"])
            if imu:
                m = max(abs(min(imu)), abs(max(imu)), 5) * 0.2
                ax_imu.set_ylim(min(imu) - m, max(imu) + m)

            line_vx.set_data(t, list(buf["vx"]))
            line_vy.set_data(t, list(buf["vy"]))
            ax_vel.set_xlim(t[0], t[-1])
            v = list(buf["vx"]) + list(buf["vy"])
            if v:
                m = max(abs(min(v)), abs(max(v)), 0.01) * 0.3
                ax_vel.set_ylim(min(v) - m, max(v) + m)

            line_bat.set_data(t, list(buf["battery"]))
            ax_bat.set_xlim(t[0], t[-1])
            bat_f = [v for v in buf["battery"] if v > 0]
            if bat_f:
                ax_bat.set_ylim(min(bat_f) - 0.1, max(bat_f) + 0.1)

            # Position
            x_screen = [-v for v in buf["y"]]
            y_screen = list(buf["x"])
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
                coord_text.set_text(f"Forward (X): {buf['x'][-1]:.3f} m\nLeft (Y): {buf['y'][-1]:.3f} m")

    elif plot_type == "height":
        fig, ax = plt.subplots(figsize=(10, 5))
        fig.suptitle("LiteWing — Live Height (Background)", color=COLORS["cyan"])
        line_filt, = ax.plot([], [], color=COLORS["cyan"], linewidth=2, label="Filtered (Kalman)")
        line_raw, = ax.plot([], [], color=COLORS["peach"], linewidth=1.5, alpha=0.7, label="Raw ToF")
        ax.set_ylabel("Height (meters)"); ax.set_xlabel("Time (seconds)")
        ax.legend(loc="upper right"); ax.grid(True)

        def update(frame):
            _drain()
            if _stdin_closed[0] and len(timestamps) == 0:
                plt.close("all")
                return
            t = list(timestamps)
            if len(t) < 2:
                return
            line_filt.set_data(t, list(buf["height"]))
            line_raw.set_data(t, list(buf["range_height"]))
            ax.set_xlim(t[0], t[-1])
            h_all = list(buf["height"]) + list(buf["range_height"])
            if h_all:
                ax.set_ylim(min(h_all) - 0.05, max(h_all) + 0.05)

    elif plot_type == "imu":
        fig, (ax_att, ax_gyro) = plt.subplots(2, 1, figsize=(10, 7))
        fig.suptitle("LiteWing — Live IMU (Background)", color=COLORS["cyan"])
        fig.subplots_adjust(hspace=0.35)

        line_r, = ax_att.plot([], [], color=COLORS["red"], label="Roll")
        line_p, = ax_att.plot([], [], color=COLORS["green"], label="Pitch")
        line_y, = ax_att.plot([], [], color=COLORS["yellow"], label="Yaw")
        ax_att.set_title("Orientation"); ax_att.set_ylabel("degrees")
        ax_att.legend(loc="upper right", fontsize=8); ax_att.grid(True)

        line_gx, = ax_gyro.plot([], [], color=COLORS["blue"], label="Gyro X")
        line_gy, = ax_gyro.plot([], [], color=COLORS["mauve"], label="Gyro Y")
        line_gz, = ax_gyro.plot([], [], color=COLORS["teal"], label="Gyro Z")
        ax_gyro.set_title("Gyroscope"); ax_gyro.set_ylabel("°/s")
        ax_gyro.set_xlabel("Time (seconds)")
        ax_gyro.legend(loc="upper right", fontsize=8); ax_gyro.grid(True)

        def update(frame):
            _drain()
            if _stdin_closed[0] and len(timestamps) == 0:
                plt.close("all")
                return
            t = list(timestamps)
            if len(t) < 2:
                return
            line_r.set_data(t, list(buf["roll"]))
            line_p.set_data(t, list(buf["pitch"]))
            line_y.set_data(t, list(buf["yaw"]))
            ax_att.set_xlim(t[0], t[-1])
            att = list(buf["roll"]) + list(buf["pitch"]) + list(buf["yaw"])
            if att:
                m = max(abs(min(att)), abs(max(att)), 5) * 0.2
                ax_att.set_ylim(min(att) - m, max(att) + m)

            line_gx.set_data(t, list(buf["gyro_x"]))
            line_gy.set_data(t, list(buf["gyro_y"]))
            line_gz.set_data(t, list(buf["gyro_z"]))
            ax_gyro.set_xlim(t[0], t[-1])
            gyro = list(buf["gyro_x"]) + list(buf["gyro_y"]) + list(buf["gyro_z"])
            if gyro:
                m = max(abs(min(gyro)), abs(max(gyro)), 1) * 0.3
                ax_gyro.set_ylim(min(gyro) - m, max(gyro) + m)

    elif plot_type == "position":
        fig, ax = plt.subplots(figsize=(8, 8))
        fig.suptitle("LiteWing — Position Trail (Background)", color=COLORS["cyan"])
        trail_line, = ax.plot([], [], color=COLORS["cyan"], linewidth=1.5, alpha=0.6)
        current_dot, = ax.plot([], [], 'o', color=COLORS["green"], markersize=10, zorder=5)
        start_dot, = ax.plot([], [], 's', color=COLORS["red"], markersize=10,
                             label="Start", zorder=5)
        ax.set_xlabel("← Right / Left → (meters)")
        ax.set_ylabel("← Backward / Forward → (meters)")
        ax.legend(loc="upper right", fontsize=9); ax.grid(True)
        coord_text = ax.text(
            0.02, 0.98, "", transform=ax.transAxes, fontsize=10,
            verticalalignment='top', color=COLORS["green"],
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#2a2a3d', alpha=0.8)
        )

        def update(frame):
            _drain()
            if _stdin_closed[0] and len(timestamps) == 0:
                plt.close("all")
                return
            # Map drone coordinates to Screen/Map view:
            # Screen Y (vertical) = Drone X (Forward/Backward)
            # Screen X (horizontal) = -Drone Y (negate so +Left moves dot Left)
            x_screen = [-v for v in buf["y"]]
            y_screen = list(buf["x"])
            if len(x_screen) < 2:
                return
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
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
            coord_text.set_text(f"Forward (X): {buf['x'][-1]:.3f} m\nLeft (Y): {buf['y'][-1]:.3f} m")
    else:
        print(f"Unknown plot type: {plot_type}", file=sys.stderr)
        sys.exit(1)

    # Auto-close when stdin is closed (main process stopped feeding)
    def _check_stdin_closed():
        if _stdin_closed[0] and len(_pending_lines) == 0:
            for path in _save_paths:
                try:
                    fig.savefig(path)
                except Exception as e:
                    print(f"Failed to save plot: {e}", file=sys.stderr)
            plt.close("all")

    timer = fig.canvas.new_timer(interval=1000)
    timer.add_callback(_check_stdin_closed)
    timer.start()

    ani = animation.FuncAnimation(fig, update, interval=update_ms, cache_frame_data=False)

    try:
        plt.show()
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python -m litewing._plot_runner <plot_type> <max_points> <update_ms>",
              file=sys.stderr)
        sys.exit(1)

    plot_type = sys.argv[1]
    max_points = int(sys.argv[2])
    update_ms = int(sys.argv[3])
    run_plot(plot_type, max_points, update_ms)
