#!/usr/bin/env python3
"""
Attitude Indicator (Artificial Horizon) - standalone test
============================================================
Isolated test of the HUD widget you asked for - the green/blue horizon
that tilts and shifts based on roll/pitch, with a fixed center reference
marker (like the red chevron in Mission Planner's HUD). Not wired into the
real drone/video app yet - this is purely to verify the visual behavior.

How it works (the actual technique):
  - Roll  -> rotate the sky/ground drawing around the center.
  - Pitch -> shift the sky/ground drawing up/down before rotating.
  - The center reference marker is drawn AFTER undoing that rotation/shift,
    so it never moves - only the horizon moves relative to it. That's what
    creates the "aircraft attitude" illusion.

Use the sliders to manually drive roll (-180..180) and pitch (-90..90) and
confirm the marker moves the way you expect before we wire this to real
telemetry.
"""

import math
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk


class AttitudeIndicator(Gtk.DrawingArea):
    """Self-contained artificial horizon widget. Call set_attitude(roll, pitch)
    to update it - roll/pitch in degrees, standard aviation convention:
    positive roll = right wing down, positive pitch = nose up."""

    def __init__(self):
        super().__init__()
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.set_size_request(360, 360)
        self.connect("draw", self._on_draw)

    def set_attitude(self, roll_deg, pitch_deg):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.queue_draw()  # schedule a repaint

    def _draw_roll_arc(self, cr, radius):
        """Draws bank-angle tick marks + labels around the top of the ball,
        symmetric both sides (unsigned degree labels), like Mission
        Planner's roll scale. Called inside a context already translated
        to center and rotated by roll, so ticks are placed at fixed angles
        from vertical (0 = top/12 o'clock) and rotate along with the ball."""
        cr.set_source_rgb(1, 1, 1)
        cr.select_font_face("monospace", 0, 1)  # slant=0 normal, weight=1 bold
        cr.set_font_size(11)

        marks = [0, 10, 20, 30, 45, 60]
        for mag in marks:
            for sign in ((1,) if mag == 0 else (-1, 1)):
                theta = math.radians(sign * mag)  # angle from vertical
                tick_len = 12 if mag % 20 == 0 or mag == 0 else 7

                # Tick mark: radial line just outside the ball's rim
                x1, y1 = radius * math.sin(theta), -radius * math.cos(theta)
                x2, y2 = (radius + tick_len) * math.sin(theta), -(radius + tick_len) * math.cos(theta)
                cr.set_line_width(2)
                cr.move_to(x1, y1)
                cr.line_to(x2, y2)
                cr.stroke()

                # Label, positioned further out, rotated to follow the arc
                if mag == 0 or mag % 10 == 0:
                    label = str(mag)
                    lx = (radius + tick_len + 12) * math.sin(theta)
                    ly = -(radius + tick_len + 12) * math.cos(theta)
                    cr.save()
                    cr.translate(lx, ly)
                    cr.rotate(theta)
                    extents = cr.text_extents(label)
                    cr.move_to(-extents.width / 2, extents.height / 2)
                    cr.show_text(label)
                    cr.restore()

    def _on_draw(self, widget, cr):
        w = widget.get_allocated_width()
        h = widget.get_allocated_height()
        cx, cy = w / 2, h / 2
        radius = min(w, h) / 2 - 46  # leaves room around the ball for the roll-arc ticks/labels

        # ---- Outer bezel ----
        cr.set_source_rgb(0.05, 0.05, 0.05)
        cr.paint()

        cr.save()
        cr.arc(cx, cy, radius, 0, 2 * math.pi)
        cr.clip()

        # ---- Everything below rotates together with roll ----
        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(self.roll_deg))

        pixels_per_degree = radius / 45.0  # +-45 deg pitch spans the radius
        pitch_offset = self.pitch_deg * pixels_per_degree

        # ---- Horizon ball (also shifts with pitch, clipped to the circle) ----
        cr.save()
        cr.translate(0, pitch_offset)

        big = radius * 3  # oversized so rotation never reveals empty corners

        # Sky (top half)
        cr.set_source_rgb(0.25, 0.55, 0.85)
        cr.rectangle(-big, -big, 2 * big, big)
        cr.fill()

        # Ground (bottom half)
        cr.set_source_rgb(0.30, 0.55, 0.15)
        cr.rectangle(-big, 0, 2 * big, big)
        cr.fill()

        # Horizon line
        cr.set_source_rgb(1, 1, 1)
        cr.set_line_width(2)
        cr.move_to(-big, 0)
        cr.line_to(big, 0)
        cr.stroke()

        # Pitch ladder - simple reference lines every 10 degrees
        cr.set_source_rgb(1, 1, 1)
        cr.set_line_width(1.5)
        for deg in range(-40, 41, 10):
            if deg == 0:
                continue
            y = -deg * pixels_per_degree
            line_half_width = 30 if deg % 20 == 0 else 16
            cr.move_to(-line_half_width, y)
            cr.line_to(line_half_width, y)
            cr.stroke()

        cr.restore()  # undo pitch translate - horizon/ladder done

        cr.restore()  # undo roll rotate/translate

        cr.restore()  # undo circular clip

        # ---- Roll (bank angle) arc - rotates with roll, sits around the
        # ball's rim, NOT clipped to the ball so labels can extend past it ----
        cr.save()
        cr.translate(cx, cy)
        cr.rotate(-math.radians(self.roll_deg))
        self._draw_roll_arc(cr, radius)
        cr.restore()

        # ---- Fixed roll pointer (never rotates - always reads current bank) ----
        cr.set_source_rgb(1, 0.15, 0.15)
        cr.set_line_width(2)
        px, py = cx, cy - radius - 4
        cr.move_to(px - 7, py - 12)
        cr.line_to(px + 7, py - 12)
        cr.line_to(px, py)
        cr.close_path()
        cr.fill()

        # ---- Fixed center reference marker (never moves) ----
        cr.set_source_rgb(1, 0.15, 0.15)
        cr.set_line_width(3)
        # Chevron / wing marker, like Mission Planner's red angle bracket
        cr.move_to(cx - 40, cy + 14)
        cr.line_to(cx - 8, cy + 2)
        cr.line_to(cx, cy + 10)
        cr.line_to(cx + 8, cy + 2)
        cr.line_to(cx + 40, cy + 14)
        cr.stroke()
        # Center dot
        cr.arc(cx, cy, 3, 0, 2 * math.pi)
        cr.fill()

        # ---- Bezel ring ----
        cr.set_source_rgb(0.6, 0.6, 0.6)
        cr.set_line_width(3)
        cr.arc(cx, cy, radius, 0, 2 * math.pi)
        cr.stroke()

        return False


class TestWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title="Attitude Indicator - Standalone Test")
        self.set_default_size(420, 520)

        main = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        main.set_margin_start(16)
        main.set_margin_end(16)
        main.set_margin_top(16)
        main.set_margin_bottom(16)
        self.add(main)

        self.indicator = AttitudeIndicator()
        main.pack_start(self.indicator, False, False, 0)

        # Roll slider
        main.pack_start(Gtk.Label(label="Roll (degrees)"), False, False, 0)
        self.roll_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, -180, 180, 1)
        self.roll_scale.set_value(0)
        self.roll_scale.connect("value-changed", self._on_change)
        main.pack_start(self.roll_scale, False, False, 0)

        # Pitch slider
        main.pack_start(Gtk.Label(label="Pitch (degrees)"), False, False, 0)
        self.pitch_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, -90, 90, 1)
        self.pitch_scale.set_value(0)
        self.pitch_scale.connect("value-changed", self._on_change)
        main.pack_start(self.pitch_scale, False, False, 0)

        self.readout = Gtk.Label(label="Roll: 0.0°   Pitch: 0.0°")
        main.pack_start(self.readout, False, False, 0)

        self.connect("destroy", Gtk.main_quit)
        self.show_all()

    def _on_change(self, *_):
        roll = self.roll_scale.get_value()
        pitch = self.pitch_scale.get_value()
        self.indicator.set_attitude(roll, pitch)
        self.readout.set_text(f"Roll: {roll:.1f}°   Pitch: {pitch:.1f}°")


if __name__ == "__main__":
    win = TestWindow()
    Gtk.main()