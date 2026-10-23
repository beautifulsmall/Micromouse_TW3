"""
TW3_sensor_cal.py — Wall Sensor Calibration GUI
================================================
Connects directly to the sensor sub-MCU USB port at 115200 baud.
Run standalone — independent of the main TW3 GUI.

Sub-MCU USB commands (ASCII chars):
  r  — start raw CSV stream at 20 Hz
  q  — stop streaming
  s  — single shot
  R  — re-detect sensor ICs
  h  — help

Raw CSV line format (12 values) — side beams cross, so ch0=left wall, ch3=right wall:
  left_px,  left_sig,  left_mm,
  fr_px,    fr_sig,    fr_mm,
  fl_px,    fl_sig,    fl_mm,
  right_px, right_sig, right_mm

Calibration workflow (per channel) — DIRECT PERPENDICULAR method:
  1. Connect to sub-MCU USB port, click Stream
  2. FIX the mouse square (yaw 0). Put a flat wall PARALLEL to the mouse's
     forward axis on the side this beam reads (side beams cross: ch0=left
     wall, ch3=right wall). Front beams: wall square ahead.
  3. Slide the wall PERPENDICULAR (toward/away) on a ruler/rail. At each
     10 mm mark, enter the PERPENDICULAR distance from the MOUSE CENTRE to
     the wall and click Capture. The distance box auto-steps +10 mm.
  4. Capture ~10 points from the near limit (~35-40 mm) out to where the
     signal drops (~140 mm).
  5. Click "Interpolate LUT" — the pixel->perpendicular table is built by
     direct interpolation (no formula fit; dense 10 mm points capture the
     true curve).
  6. "Send LUT" (W commands) writes it live, or "Generate C Array" to paste.
  The LUT stores PERPENDICULAR-from-centre mm (matches walls_evaluate pass-
  through). NOTE: applied to the MAP, the observer needs the trig below to
  place the spot (how far AHEAD of centre it sits) using the beam angle and
  the mouse yaw pose -- see spot_forward().
"""

import sys
import json
import os
import re
import math
import time
import bisect
from collections import deque
from datetime import date

import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QGroupBox, QTabWidget, QScrollArea, QSplitter, QFrame,
    QStatusBar, QSizePolicy, QTableWidget, QTableWidgetItem,
    QRadioButton, QButtonGroup, QSpinBox, QDoubleSpinBox,
    QTextEdit, QHeaderView, QProgressBar
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QColor, QPainter, QPen, QBrush, QPalette

import pyqtgraph as pg
import serial
import serial.tools.list_ports

# =============================================================================
# Constants
# =============================================================================

BAUD_RATE   = 115200   # sensor sub-MCU USB
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "sensor_cal_config.json")

# 40-field raw CSV layout: 10 values per channel × 4 channels
# [frame, x_int, x_dec, y_int, y_dec, sum_u, sum_l, sig_x, sig_y, mm]
CH_FIELDS  = 10
F_FRAME    = 0
F_X_INT    = 1   # pixel position integer (calibration x-input)
F_X_DEC    = 2
F_Y_INT    = 3
F_Y_DEC    = 4
F_SUM_U    = 5
F_SUM_L    = 6
F_SIG_X    = 7   # peak signal X (threshold gate)
F_SIG_Y    = 8
F_MM       = 9   # computed distance mm

# channel index → (name, formula, x_var, threshold, A, B)
#   formula : 'log'   → mm = A·log(x) + B
#             'power' → mm = A·x^B
#   x_var   : 'inv'    → x = 255 − Maxpos_X_int
#             'direct' → x = Maxpos_X_int
# NOTE: side beams cross, so each side sensor reads the OPPOSITE wall:
#   ch0 is the right-MOUNTED sensor (IC0) but reads the LEFT wall  -> "Left"
#   ch3 is the left-MOUNTED  sensor (IC2) but reads the RIGHT wall -> "Right"
# Names below are by WALL MEASURED. Calibration formula/threshold stay with the
# physical sensor (the channel index), so they are unchanged by the rename.
CHANNEL_DEFAULTS = [
    ("Left",    "log",   "inv",    170, -27.8160, 205.5980),
    ("Front-R", "power", "direct", 120, 16294.0,  -1.1420),
    ("Front-L", "log",   "direct", 170, -27.8160, 205.5980),
    ("Right",   "log",   "inv",    170, -34.2192, 233.8327),
]

CHANNEL_COLORS = ["#00d4ff", "#ff922b", "#ffd43b", "#51cf66"]  # cyan, orange, yellow, green

# C variable names for each channel (matches firmware measure_and_send()).
# ch0 (right-mounted IC0) reads the left wall -> left_mm; ch3 -> right_mm.
CHANNEL_CVAR = ["left_mm", "front_right_mm", "front_left_mm", "right_mm"]

# Comment labels for generated code
CHANNEL_CLABEL = ["Left wall", "Front-right", "Front-left", "Right wall"]

# Sensor IC index for each channel (0=right IC, 1=front IC, 2=left IC)
CHANNEL_IC = [0, 1, 1, 2]

# =============================================================================
# Laser mounting geometry
# -----------------------------------------------------------------------------
# Body frame:  x = forward (mm, +ahead),  y = left (mm, +to the mouse's left).
# angle = beam direction in degrees, measured CCW from the forward (+x) axis:
#   0deg = straight ahead, +90deg = straight left, -90deg = straight right.
#
# The LUT now stores the PERPENDICULAR wall distance from the mouse centre
# DIRECTLY (measured by ruler, cal'd square at yaw 0) -- ruler-accurate, no
# baked geometry. walls_evaluate passes it straight through.
#   front lasers -> forward distance to the front wall  (xc)
#   side  lasers -> lateral distance to a side wall (|yc|)
# beam_range_from_wall() / spot_forward() give the INVERSE geometry the map
# observer needs: perp -> along-beam range -> where the spot really sits
# (forward of centre), with the mouse yaw pose folded in.
#
# The side beams CROSS: each side sensor reads the OPPOSITE wall. ch0 is the
# right-mounted sensor (y = -15) but angled +72.8deg (forward-left) so it reads
# the LEFT wall; ch3 is left-mounted (y = +15) at -71deg (forward-right) and
# reads the RIGHT wall. Channels are therefore named by the wall they measure.
#
# Angles below are the MEASURED, fixed per-laser values (TW3). Positions: front
# lasers 28 mm forward, 18 mm apart (+/-9 mm); side lasers 40 mm forward, 30 mm
# apart (+/-15 mm). Front lasers angle inward (beams cross ahead); side lasers
# angle forward of straight-out so each crosses to the opposite wall.
# =============================================================================
LASER_GEOMETRY_DEFAULTS = [
    #            x fwd   y left   angle    kind
    {"x": 40.0, "y": -15.0, "angle":  72.8, "kind": "side"},   # 0 Left  (right-mounted, beam crosses to left wall)
    {"x": 28.0, "y":  -9.0, "angle":   8.4, "kind": "front"},  # 1 Front-R
    {"x": 28.0, "y":   9.0, "angle":  -9.6, "kind": "front"},  # 2 Front-L
    {"x": 40.0, "y":  15.0, "angle": -71.0, "kind": "side"},   # 3 Right (left-mounted, beam crosses to right wall)
]

def wall_distance_from_beam(geom: dict, beam_mm: float) -> float:
    """Perpendicular wall distance from the mouse centre, given a direct
    along-beam range (mm) and one channel's mounting geometry.

    Contact point in the body frame:
        xc = x_off + r * cos(angle)
        yc = y_off + r * sin(angle)
    Front lasers return the forward distance (xc) to the front wall;
    side lasers return the lateral distance (|yc|) to their side wall.
    """
    phi = math.radians(geom["angle"])
    xc  = geom["x"] + beam_mm * math.cos(phi)
    yc  = geom["y"] + beam_mm * math.sin(phi)
    return xc if geom.get("kind") == "front" else abs(yc)

def beam_range_from_wall(geom: dict, perp_mm: float) -> float:
    """Inverse of wall_distance_from_beam (yaw 0): the along-beam range r that
    puts the spot at perpendicular wall distance perp_mm. Pure sensor geometry
    the map needs to un-bake the perpendicular LUT back to a beam range."""
    phi = math.radians(geom["angle"])
    if geom.get("kind") == "front":
        c = math.cos(phi)
        return (perp_mm - geom["x"]) / c if abs(c) > 1e-6 else 0.0
    s = math.sin(phi)
    if abs(s) < 1e-6:
        return 0.0
    r1 = (perp_mm - geom["y"]) / s   # opposite-wall crossing roots
    r2 = (-perp_mm - geom["y"]) / s
    return r1 if r1 > 0 else r2

def spot_forward(geom: dict, perp_mm: float, yaw_deg: float = 0.0):
    """MAP TRIG (bert): where the laser spot actually sits on the wall given a
    perpendicular reading and the mouse yaw. The beam rotates with yaw, so the
    spot moves forward/back along the wall. Returns (along_beam_range_mm,
    spot_forward_mm) -- spot_forward = how far AHEAD of the mouse centre the
    spot is. NOTE: this is the near-axial form (recovers r at yaw 0, then
    rotates the beam by yaw); a fully yaw-correct map placement re-solves r
    against the wall line at the live pose -- the firmware observer's job."""
    r = beam_range_from_wall(geom, perp_mm)
    phi = math.radians(geom["angle"] + yaw_deg)
    return r, geom["x"] + r * math.cos(phi)

# =============================================================================
# Formula helpers
# =============================================================================

def x_from_px(x_var: str, px: int) -> float:
    """Convert raw pixel position to formula x value."""
    return float(255 - px) if x_var == "inv" else float(px)

def compute_mm(formula: str, x_var: str, A: float, B: float,
               threshold: int, px: int, sig: int) -> float:
    """Apply calibration formula. Returns 255.0 if sig < threshold or x <= 0."""
    if sig < threshold:
        return 255.0
    x = x_from_px(x_var, px)
    if x <= 0:
        return 255.0
    try:
        if formula == "log":
            return A * math.log(x) + B
        else:
            return A * (x ** B)
    except (ValueError, ZeroDivisionError):
        return 255.0

def fit_curve(formula: str, x_var: str,
              known_mm: list, px_vals: list) -> tuple:
    """
    Fit calibration curve to (known_mm, px) pairs.
    Returns (A, B, r2) or (None, None, None) on failure.
    """
    xs = np.array([x_from_px(x_var, p) for p in px_vals], dtype=float)
    ys = np.array(known_mm, dtype=float)

    valid = (xs > 0) & (ys > 0) & np.isfinite(xs) & np.isfinite(ys)
    xs, ys = xs[valid], ys[valid]
    if len(xs) < 2:
        return None, None, None

    try:
        if formula == "log":
            # y = A·log(x) + B  →  [log(x), 1] · [A, B]ᵀ = y
            X = np.column_stack([np.log(xs), np.ones_like(xs)])
            coeffs, _, _, _ = np.linalg.lstsq(X, ys, rcond=None)
            A, B = float(coeffs[0]), float(coeffs[1])
        else:
            # y = A·x^B  →  log(y) = log(A) + B·log(x)
            log_x = np.log(xs)
            log_y = np.log(ys)
            X = np.column_stack([np.ones_like(log_x), log_x])
            coeffs, _, _, _ = np.linalg.lstsq(X, log_y, rcond=None)
            A = float(math.exp(coeffs[0]))
            B = float(coeffs[1])

        # R²
        if formula == "log":
            y_pred = A * np.log(xs) + B
        else:
            y_pred = A * (xs ** B)
        ss_res = float(np.sum((ys - y_pred) ** 2))
        ss_tot = float(np.sum((ys - ys.mean()) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        return A, B, r2
    except Exception:
        return None, None, None

def generate_code(ch_idx: int, formula: str, x_var: str,
                  threshold: int, A: float, B: float) -> str:
    """Generate the C++ snippet for this channel to paste into measure_and_send()."""
    name   = CHANNEL_CLABEL[ch_idx]
    cvar   = CHANNEL_CVAR[ch_idx]
    today  = date.today().isoformat()
    lines  = [f"    // {name} — calibrated {today}"]

    if formula == "log":
        x_desc = "(255 - Maxpos_X_int)" if x_var == "inv" else "Maxpos_X_int"
        lines.append(f"    // formula: {A:.4f} * log({x_desc}) + {B:.4f}  sig_threshold: {threshold}")
        lines.append(    "    {")
        if x_var == "inv":
            lines.append(f"        uint32_t x = 255 - Maxpos_X_int;")
            lines.append(f"        {cvar} = (Maxval_X < {threshold}) ? 255.0f : ({A:.4f}f * log((float)x) + {B:.4f}f);")
        else:
            lines.append(f"        {cvar} = (Maxval_X < {threshold}) ? 255.0f : ({A:.4f}f * log((float)Maxpos_X_int) + {B:.4f}f);")
        lines.append(    "    }")
    else:
        x_desc = "(255 - Maxpos_X_int)" if x_var == "inv" else "Maxpos_X_int"
        lines.append(f"    // formula: {A:.4f} * {x_desc}^{B:.4f}  sig_threshold: {threshold}")
        lines.append(    "    {")
        if x_var == "inv":
            lines.append(f"        uint32_t x = 255 - Maxpos_X_int;")
            lines.append(f"        {cvar} = (Maxval_X < {threshold}) ? 255.0f : ({A:.4f}f * pow((float)x, {B:.4f}f));")
        else:
            lines.append(f"        {cvar} = (Maxval_X < {threshold}) ? 255.0f : ({A:.4f}f * pow((float)Maxpos_X_int, {B:.4f}f));")
        lines.append(    "    }")
    return "\n".join(lines)

# =============================================================================
# Config persistence
# =============================================================================

def load_config() -> dict:
    try:
        with open(CONFIG_FILE) as f:
            return json.load(f)
    except Exception:
        return {}

def save_config(cfg: dict):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)

# =============================================================================
# Serial reader thread
# =============================================================================

class SensorReader(QThread):
    """
    Connects to sensor sub-MCU USB serial.
    Sends 'r' to start raw CSV stream; parses lines into 12-field tuples.
    Emits data_ready(list[int]) with [px0, sig0, mm0, px1, sig1, mm1, ...] × 4.
    """
    data_ready     = pyqtSignal(list)   # 40 ints: [frame,x_int,...,mm] × 4 channels
    ic_status      = pyqtSignal(list)   # 3 bools: sensor IC present flags (inferred)
    loop_us_ready  = pyqtSignal(int)    # firmware measurement cycle time in µs
    connection_lost = pyqtSignal()
    stats_ready    = pyqtSignal(int)    # lines/s
    port_ready     = pyqtSignal()       # serial port is open — safe to send commands
    raw_line       = pyqtSignal(str)    # every line received (for debug log)
    format_warn    = pyqtSignal(str)    # "dist" or "3ch" when not in 12-field raw mode

    def __init__(self, port: str):
        super().__init__()
        self.port       = port
        self._running   = False
        self._streaming = False
        self.ser        = None

    def run(self):
        self._running = True
        try:
            self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.1)
        except Exception:
            self.connection_lost.emit()
            return

        # Teensy native USB CDC needs a moment to stabilise after the port
        # is opened before it can reliably receive characters.
        time.sleep(0.05)
        self.ser.reset_input_buffer()   # discard any stale bytes
        self.port_ready.emit()          # UI can now update — 'r' sent ~600 ms later

        buf        = ""
        line_count = 0
        stats_t    = time.monotonic()
        retry_t    = time.monotonic()   # tracks when to auto-retry 'r'

        while self._running:
            try:
                chunk = self.ser.read(256).decode("ascii", errors="ignore")
            except Exception:
                self.connection_lost.emit()
                break

            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                self.raw_line.emit(line)   # every received line → Raw Log tab
                if line.startswith("#"):
                    continue
                parts = [p.strip() for p in line.split(",")]
                try:
                    iparts = [int(p) for p in parts if p]
                except ValueError:
                    continue

                n = len(iparts)
                if n >= 40:
                    # Current firmware: 40 sensor fields + optional loop_us (field 41)
                    self.data_ready.emit(iparts[:40])
                    mm = [iparts[ch * CH_FIELDS + F_MM] for ch in range(4)]
                    ic = [mm[0] != 0, mm[1] != 0 or mm[2] != 0, mm[3] != 0]
                    self.ic_status.emit(ic)
                    if n >= 41:
                        self.loop_us_ready.emit(iparts[40])
                    line_count += 1
                    retry_t = time.monotonic()
                elif n == 12:
                    # Legacy 12-field: [px, sig, mm] × 4 — pad to 40 fields
                    padded = []
                    for ch in range(4):
                        px, sg, mm = iparts[ch*3], iparts[ch*3+1], iparts[ch*3+2]
                        padded += [0, px, 0, 0, 0, 0, 0, sg, 0, mm]
                    self.data_ready.emit(padded)
                    mm = [iparts[2], iparts[5], iparts[8], iparts[11]]
                    ic = [mm[0] != 0, mm[1] != 0 or mm[2] != 0, mm[3] != 0]
                    self.ic_status.emit(ic)
                    line_count += 1
                    retry_t = time.monotonic()
                elif n == 4:
                    v = iparts
                    padded = []
                    for mm in v:
                        padded += [0, 0, 0, 0, 0, 0, 0, 0, 0, mm]
                    self.data_ready.emit(padded)
                    ic = [v[0] != 0, v[1] != 0 or v[2] != 0, v[3] != 0]
                    self.ic_status.emit(ic)
                    self.format_warn.emit("dist")
                    line_count += 1
                    retry_t = time.monotonic()

            now = time.monotonic()

            # Auto-retry 'r' every 3 s while streaming mode is active but silent
            if self._streaming and (now - retry_t >= 3.0):
                try:
                    self.ser.write(b"r")
                except Exception:
                    pass
                retry_t = now

            if now - stats_t >= 1.0:
                self.stats_ready.emit(line_count)
                line_count = 0
                stats_t    = now

    def send_cmd(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
                if cmd.startswith("r"):
                    self._streaming = True
                elif cmd.startswith("q"):
                    self._streaming = False
            except Exception:
                pass

    def stop(self):
        self._running = False
        if self.ser:
            try:
                self.ser.write(b"q")
            except Exception:
                pass
            self.ser.close()

# =============================================================================
# ChannelTile — live reading display for one channel
# =============================================================================

class ChannelTile(QWidget):
    """
    Live display for one channel — shows all S15366 output registers plus computed mm.
    Receives a slice of the 40-field data list (10 values: CH_FIELDS per channel).
    """

    def __init__(self, ch_idx: int, send_cmd_fn):
        super().__init__()
        self.ch_idx    = ch_idx
        self._send_cmd = send_cmd_fn
        self._laser_on = True
        name  = CHANNEL_DEFAULTS[ch_idx][0]
        color = CHANNEL_COLORS[ch_idx]

        self.setStyleSheet(f"background: #252525; border: 1px solid {color}55; border-radius: 4px;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(2)

        # Title row + laser toggle
        title_row = QHBoxLayout()
        title = QLabel(name)
        title.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 11px; border: none;")
        title_row.addWidget(title)
        title_row.addStretch()
        self._laser_btn = QPushButton("● Laser ON")
        self._laser_btn.setFixedHeight(18)
        self._laser_btn.setFixedWidth(80)
        self._laser_btn.setCheckable(True)
        self._laser_btn.setChecked(True)
        self._laser_btn.setStyleSheet(
            "QPushButton { background: #2d7a2d; color: white; border-radius: 3px; font-size: 9px; border: none; }"
            "QPushButton:hover { background: #3a9e3a; }"
            "QPushButton:!checked { background: #3a3a3a; color: #666; }"
            "QPushButton:!checked:hover { background: #4a4a4a; }"
        )
        self._laser_btn.clicked.connect(self._toggle_laser)
        title_row.addWidget(self._laser_btn)
        layout.addLayout(title_row)

        # Integration time for THIS channel (sub-MCU 'i<ch><us>').
        # The close-range saturation test: bring a target in until Maxval_X
        # pegs and Maxpos_X wanders, then wind this down and watch Maxval come
        # off the rail while Maxpos settles. All four lasers keep firing, so
        # the interlaced cadence is untouched and the other three channels
        # stay as a reference. If Maxpos SHIFTS with integration time, the LUT
        # is exposure-dependent and needs re-calibrating per exposure, not
        # just extending lower.
        integ_row = QHBoxLayout()
        integ_lbl = QLabel("integ µs")
        integ_lbl.setFixedWidth(50)
        integ_lbl.setStyleSheet("color: #777; font-size: 9px; border: none;")
        integ_row.addWidget(integ_lbl)
        self._integ = QSpinBox()
        self._integ.setRange(1, 2047)
        # mirrors the sub-MCU boot default: side 300, front 400
        self._integ.setValue(400 if ch_idx in (1, 2) else 300)
        self._integ.setSingleStep(25)
        self._integ.setKeyboardTracking(False)   # fire on Enter/step, not per digit
        self._integ.setFixedHeight(18)
        self._integ.setStyleSheet(
            "QSpinBox { background: #1a1a1a; color: #ddd; border: 1px solid #333;"
            " border-radius: 2px; font-size: 9px; }"
        )
        self._integ.valueChanged.connect(self._set_integ)
        integ_row.addWidget(self._integ)
        for label, val in (("½", 200), ("400", 400)):
            b = QPushButton(label)
            b.setFixedHeight(18)
            b.setFixedWidth(30)
            b.setStyleSheet(
                "QPushButton { background: #3a3a3a; color: #bbb; border-radius: 3px;"
                " font-size: 9px; border: none; }"
                "QPushButton:hover { background: #4a4a4a; }"
            )
            b.clicked.connect(lambda _=False, v=val: self._integ.setValue(v))
            integ_row.addWidget(b)
        layout.addLayout(integ_row)

        def _bar_row(label_text, bar_max=255, bar_color="#888"):
            row = QHBoxLayout()
            lbl = QLabel(label_text)
            lbl.setFixedWidth(50)
            lbl.setStyleSheet("color: #777; font-size: 9px; border: none;")
            bar = QProgressBar()
            bar.setRange(0, bar_max)
            bar.setTextVisible(False)
            bar.setFixedHeight(8)
            bar.setStyleSheet(
                f"QProgressBar {{ border: 1px solid #333; border-radius: 2px; background: #1a1a1a; }}"
                f"QProgressBar::chunk {{ background: {bar_color}; border-radius: 1px; }}"
            )
            val = QLabel("---")
            val.setFixedWidth(36)
            val.setStyleSheet("color: #aaa; font-family: Courier; font-size: 9px; border: none;")
            row.addWidget(lbl)
            row.addWidget(bar)
            row.addWidget(val)
            return row, bar, val

        def _rgba(hx, alpha):
            """Convert '#RRGGBB' + alpha (0-255) to 'rgba(r,g,b,a)' for Qt stylesheets."""
            return f"rgba({int(hx[1:3],16)},{int(hx[3:5],16)},{int(hx[5:7],16)},{alpha})"

        r, self._x_bar, self._x_val   = _bar_row("x_pos:", bar_color=_rgba(color, 180))
        layout.addLayout(r)
        r, self._y_bar, self._y_val   = _bar_row("y_pos:", bar_color=_rgba(color, 110))
        layout.addLayout(r)
        r, self._sx_bar, self._sx_val = _bar_row("sig_x:", bar_color="#e67e22")
        layout.addLayout(r)
        r, self._sy_bar, self._sy_val = _bar_row("sig_y:", bar_color="#d35400")
        layout.addLayout(r)

        # OUT_XY_SUM — 16-bit split across sum_u / sum_l
        sum_row = QHBoxLayout()
        sum_lbl = QLabel("xy_sum:")
        sum_lbl.setFixedWidth(50)
        sum_lbl.setStyleSheet("color: #777; font-size: 9px; border: none;")
        self._sum_val = QLabel("---")
        self._sum_val.setStyleSheet("color: #aaa; font-family: Courier; font-size: 9px; border: none;")
        sum_row.addWidget(sum_lbl)
        sum_row.addWidget(self._sum_val)
        sum_row.addStretch()
        layout.addLayout(sum_row)

        # frame count
        frame_row = QHBoxLayout()
        frame_lbl = QLabel("frame:")
        frame_lbl.setFixedWidth(50)
        frame_lbl.setStyleSheet("color: #777; font-size: 9px; border: none;")
        self._frame_val = QLabel("---")
        self._frame_val.setStyleSheet("color: #555; font-family: Courier; font-size: 9px; border: none;")
        frame_row.addWidget(frame_lbl)
        frame_row.addWidget(self._frame_val)
        frame_row.addStretch()
        layout.addLayout(frame_row)

        # Distance (large)
        self._mm_label = QLabel("--- mm")
        self._mm_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._mm_label.setFont(QFont("Courier", 13, QFont.Weight.Bold))
        self._mm_label.setStyleSheet(f"color: {color}; border: none;")
        layout.addWidget(self._mm_label)

        # Computed perpendicular wall distance (from beam range + mounting geometry)
        self._wall_label = QLabel("wall: --- mm")
        self._wall_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._wall_label.setStyleSheet("color: #888; font-size: 9px; border: none;")
        layout.addWidget(self._wall_label)

    def set_wall(self, mm):
        """Update the computed wall-distance line. Pass None when out of range."""
        if mm is None:
            self._wall_label.setText("wall: --- mm")
        else:
            self._wall_label.setText(f"wall: {mm:.0f} mm")

    def _set_integ(self, us: int):
        """Push this channel's integration time to the sub-MCU."""
        self._send_cmd(f"i{self.ch_idx}{int(us)}")

    def set_integ_display(self, us: int):
        """Reflect a value reported by the sub-MCU without echoing it back."""
        self._integ.blockSignals(True)
        self._integ.setValue(int(us))
        self._integ.blockSignals(False)

    def _toggle_laser(self):
        self._laser_on = self._laser_btn.isChecked()
        if self._laser_on:
            self._laser_btn.setText("● Laser ON")
        else:
            self._laser_btn.setText("○ Laser OFF")
        self._send_cmd(f"L{self.ch_idx}{1 if self._laser_on else 0}")

    def set_readings(self, ch_data: list):
        """
        ch_data: 10-element slice from the 40-field CSV for this channel:
          [frame, x_int, x_dec, y_int, y_dec, sum_u, sum_l, sig_x, sig_y, mm]
        """
        frame, x_int, x_dec, y_int, y_dec, sum_u, sum_l, sig_x, sig_y, mm = ch_data

        self._frame_val.setText(str(frame))

        self._x_bar.setValue(min(x_int, 255))
        self._x_val.setText(f"{x_int}.{x_dec:02d}" if x_dec else str(x_int))

        self._y_bar.setValue(min(y_int, 255))
        self._y_val.setText(f"{y_int}.{y_dec:02d}" if y_dec else str(y_int))

        self._sx_bar.setValue(min(sig_x, 255))
        self._sx_val.setText(str(sig_x))

        self._sy_bar.setValue(min(sig_y, 255))
        self._sy_val.setText(str(sig_y))

        xy_sum = (sum_u << 8) | sum_l
        self._sum_val.setText(str(xy_sum))

        if mm == 0:
            self._mm_label.setText("IC ERR")
            self._mm_label.setStyleSheet("color: #e03131; border: none; font-size: 12px;")
        elif mm == 255:
            self._mm_label.setText("OUT OF RANGE")
            self._mm_label.setStyleSheet("color: #555; border: none; font-size: 10px;")
        else:
            self._mm_label.setText(f"{mm} mm")
            color = CHANNEL_COLORS[self.ch_idx]
            self._mm_label.setStyleSheet(f"color: {color}; border: none; font-size: 13px;")

# =============================================================================
# TopDownView — schematic of mouse + wall distances
# =============================================================================

class TopDownView(QWidget):
    """
    QPainter-based top-down view of the mouse with four coloured sensor beams.
    Forward = up. Mouse body drawn at centre. Walls drawn at measured distances.
    """

    def __init__(self):
        super().__init__()
        self.setMinimumSize(230, 260)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._beam = [255, 255, 255, 255]   # direct along-beam ranges [R, FR, FL, L]
        self._geom = [dict(g) for g in LASER_GEOMETRY_DEFAULTS]

    def set_geometry(self, geom: list):
        self._geom = geom
        self.repaint()

    def set_readings(self, beam_ranges: list, geom: list = None):
        self._beam = list(beam_ranges)
        if geom is not None:
            self._geom = geom
        self.repaint()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h   = self.width(), self.height()
        cx, cy = w // 2, int(h * 0.55)  # mouse centre — shifted down so front has more room
        scale  = min(w, h) / 320.0      # px per mm

        # Background
        p.fillRect(0, 0, w, h, QColor("#1a1a1a"))

        # Grid lines every 50 mm
        p.setPen(QPen(QColor("#2a2a2a"), 1))
        for mm in range(0, 300, 50):
            d = int(mm * scale)
            p.drawLine(cx - d, 0, cx - d, h)
            p.drawLine(cx + d, 0, cx + d, h)
            p.drawLine(0, cy - d, w, cy - d)
            p.drawLine(0, cy + d, w, cy + d)

        # Scale bar (50 mm)
        bar_px = int(50 * scale)
        bar_x  = 8
        bar_y  = h - 14
        p.setPen(QPen(QColor("#555"), 1))
        p.drawLine(bar_x, bar_y, bar_x + bar_px, bar_y)
        p.setPen(QPen(QColor("#888"), 1))
        p.setFont(QFont("Arial", 7))
        p.drawText(bar_x, bar_y - 2, "50 mm")

        # Mouse body: 70 mm wide × 90 mm long
        mw = int(35 * scale)   # half-width
        ml = int(45 * scale)   # half-length
        mouse_rect_x = cx - mw
        mouse_rect_y = cy - ml
        p.setBrush(QBrush(QColor("#333")))
        p.setPen(QPen(QColor("#666"), 1))
        p.drawRect(mouse_rect_x, mouse_rect_y, 2 * mw, 2 * ml)

        # Forward arrow
        p.setPen(QPen(QColor("#555"), 1))
        p.drawLine(cx, cy - ml, cx, cy - ml - 12)

        # "FWD" label
        p.setFont(QFont("Arial", 7))
        p.setPen(QPen(QColor("#555"), 1))
        p.drawText(cx + 3, cy - ml - 4, "FWD")

        # Sensor positions & beam directions come from the mounting geometry.
        # Body frame (x fwd, y left) -> image coords (right=+x, down=+y, fwd=up):
        #   image_right = -y_left ,  image_down = -x_forward
        #   beam dir    = (-sin(angle), -cos(angle))
        max_range_mm = 220  # maximum length to draw a beam with no wall hit

        for i in range(4):
            g     = self._geom[i]
            color = QColor(CHANNEL_COLORS[i])
            r     = self._beam[i]
            phi   = math.radians(g["angle"])

            # Sensor point (body -> image)
            spx = cx + int(-g["y"] * scale)
            spy = cy + int(-g["x"] * scale)

            # Beam unit direction (body -> image)
            dx = -math.sin(phi)
            dy = -math.cos(phi)

            has_wall = 0 < r < 255
            beam_mm  = beam_range_from_wall(g, r) if has_wall else max_range_mm

            bpx = spx + int(dx * beam_mm * scale)
            bpy = spy + int(dy * beam_mm * scale)

            # Beam line
            pen = QPen(color, 1, Qt.PenStyle.SolidLine if has_wall else Qt.PenStyle.DashLine)
            pen.setColor(QColor(color.red(), color.green(), color.blue(), 140))
            p.setPen(pen)
            p.drawLine(spx, spy, bpx, bpy)

            if has_wall:
                # Wall tick: front laser -> horizontal wall, side laser -> vertical wall
                p.setPen(QPen(color, 2))
                if g.get("kind") == "front":
                    p.drawLine(bpx - 12, bpy, bpx + 12, bpy)
                else:
                    p.drawLine(bpx, bpy - 12, bpx, bpy + 12)

                # LUT now stores perpendicular-from-centre directly.
                wd = r
                p.setFont(QFont("Arial", 8))
                p.setPen(QPen(color, 1))
                p.drawText(bpx + 6, bpy + 4, f"{wd:.0f}")

            # Sensor dot
            p.setBrush(QBrush(color))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(spx - 3, spy - 3, 6, 6)

        p.end()

# =============================================================================
# ChannelCalWidget — calibration workspace for one channel
# =============================================================================

class ChannelCalWidget(QWidget):
    """Per-channel LUT calibration: capture real measurements, interpolate, send to firmware."""

    lut_updated = pyqtSignal(int, list)   # ch_idx, lut[256]

    def __init__(self, ch_idx: int, send_cmd_fn, parent=None):
        super().__init__(parent)
        self.ch_idx   = ch_idx
        self._send_fn = send_cmd_fn
        name  = CHANNEL_DEFAULTS[ch_idx][0]
        color = CHANNEL_COLORS[ch_idx]

        # LUT: direct (0→255) for Right / Front-R, inverted (255→0) for Front-L / Left
        if ch_idx in (1, 2, 3):   # Front-R, Front-L, Left all inverted
            self._lut = list(range(255, -1, -1))
        else:
            self._lut = list(range(256))   # Right: direct

        self._data_px  = []
        self._data_mm  = []
        self._live_px  = 0
        self._live_sig = 0

        # Bulk-send state
        self._send_queue = []
        self._send_timer = QTimer()
        self._send_timer.setInterval(8)   # 8 ms between W commands ≈ 32 kbps
        self._send_timer.timeout.connect(self._send_next)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        # ── Left: scatter plot ────────────────────────────────────────────────
        self._plot = pg.PlotWidget()
        self._plot.setBackground("#1e1e1e")
        self._plot.setTitle(f"{name} — x_pos vs distance", size="9pt")
        self._plot.setLabel("left",   "Distance (mm)")
        self._plot.setLabel("bottom", "x_pos (0–255)")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setXRange(0, 255)
        self._plot.setYRange(0, 255)
        self._plot.addLegend(offset=(10, 10))

        self._lut_line = self._plot.plot(list(range(256)), self._lut,
                                          pen=pg.mkPen(color=color, width=1),
                                          name="LUT")
        self._scatter  = self._plot.plot([], [], pen=None,
                                         symbol="o", symbolSize=9,
                                         symbolBrush=color, symbolPen="w",
                                         name="Captured")
        self._live_dot = self._plot.plot([], [], pen=None,
                                         symbol="+", symbolSize=14,
                                         symbolBrush="#ffffff", symbolPen="#ffffff",
                                         name="Live")
        layout.addWidget(self._plot, stretch=3)

        # ── Right: controls ───────────────────────────────────────────────────
        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidgetResizable(True)
        ctrl_scroll.setMinimumWidth(290)
        ctrl_scroll.setMaximumWidth(350)

        ctrl_w = QWidget()
        ctrl_l = QVBoxLayout(ctrl_w)
        ctrl_l.setSpacing(8)
        ctrl_l.setContentsMargins(4, 4, 4, 4)

        # ── Data capture ──────────────────────────────────────────────────────
        cap_grp = QGroupBox("Capture points  (perpendicular mm, every 10 mm)")
        cap_l = QVBoxLayout(cap_grp)

        dist_row = QHBoxLayout()
        dist_row.addWidget(QLabel("Perp mm (centre→wall):"))
        self._dist_spin = QSpinBox()
        self._dist_spin.setRange(1, 254)
        self._dist_spin.setValue(50)
        self._dist_spin.setSingleStep(10)
        self._dist_spin.setFixedWidth(65)
        dist_row.addWidget(self._dist_spin)
        dist_row.addStretch()
        self._sig_lbl = QLabel("sig: ---")
        self._sig_lbl.setFont(QFont("Courier", 10))
        dist_row.addWidget(self._sig_lbl)
        cap_l.addLayout(dist_row)

        btn_row = QHBoxLayout()
        self._btn_capture = QPushButton("Capture")
        self._btn_capture.setStyleSheet(
            "QPushButton { background: #2d7a2d; color: white; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background: #3a9e3a; }")
        self._btn_capture.clicked.connect(self._capture)
        self._btn_clear = QPushButton("Clear All")
        self._btn_clear.setStyleSheet(
            "QPushButton { background: #5a2020; color: white; border-radius: 4px; }"
            "QPushButton:hover { background: #7a2a2a; }")
        self._btn_clear.clicked.connect(self._clear_data)
        btn_row.addWidget(self._btn_capture)
        btn_row.addWidget(self._btn_clear)
        btn_row.addStretch()
        cap_l.addLayout(btn_row)

        self._cap_status = QLabel("Fix mouse square; slide wall perpendicular; capture the "
                              "centre→wall mm at each 10 mm mark")
        self._cap_status.setStyleSheet("color: #888; font-size: 10px;")
        self._cap_status.setWordWrap(True)
        cap_l.addWidget(self._cap_status)
        ctrl_l.addWidget(cap_grp)

        # ── Data table ────────────────────────────────────────────────────────
        tbl_grp = QGroupBox("Captured points")
        tbl_l = QVBoxLayout(tbl_grp)
        self._table = QTableWidget(0, 3)
        self._table.setHorizontalHeaderLabels(["x_pos", "Known mm", "LUT mm"])
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._table.setFixedHeight(155)
        self._table.setStyleSheet("font-size: 10px;")
        tbl_l.addWidget(self._table)
        ctrl_l.addWidget(tbl_grp)

        # ── Interpolate ───────────────────────────────────────────────────────
        interp_grp = QGroupBox("Build LUT")
        interp_l = QVBoxLayout(interp_grp)
        btn_interp = QPushButton("Interpolate LUT from captured points")
        btn_interp.setStyleSheet(
            "QPushButton { background: #2d5a8e; color: white; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background: #3a72b5; }")
        btn_interp.clicked.connect(self._interpolate)
        interp_l.addWidget(btn_interp)
        self._interp_status = QLabel("")
        self._interp_status.setStyleSheet("color: #888; font-size: 10px;")
        interp_l.addWidget(self._interp_status)
        ctrl_l.addWidget(interp_grp)

        # ── Send to firmware ──────────────────────────────────────────────────
        send_grp = QGroupBox("Send LUT to firmware")
        send_l = QVBoxLayout(send_grp)
        self._btn_send = QPushButton("Send 256 entries  (W commands)")
        self._btn_send.setStyleSheet(
            "QPushButton { background: #5a3a8e; color: white; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background: #7a52b5; }")
        self._btn_send.clicked.connect(self._start_send_lut)
        send_l.addWidget(self._btn_send)
        self._send_bar = QProgressBar()
        self._send_bar.setRange(0, 256)
        self._send_bar.setValue(0)
        self._send_bar.setVisible(False)
        self._send_bar.setFixedHeight(10)
        send_l.addWidget(self._send_bar)
        ctrl_l.addWidget(send_grp)

        # ── C array output ────────────────────────────────────────────────────
        code_grp = QGroupBox("C array  (paste into firmware)")
        code_l = QVBoxLayout(code_grp)
        self._code_edit = QTextEdit()
        self._code_edit.setFont(QFont("Courier", 9))
        self._code_edit.setReadOnly(True)
        self._code_edit.setFixedHeight(130)
        self._code_edit.setStyleSheet("background: #141414; color: #51cf66;")
        code_l.addWidget(self._code_edit)
        btn_gen = QPushButton("Generate C Array")
        btn_gen.setFixedWidth(130)
        btn_gen.clicked.connect(self._gen_c_array)
        code_l.addWidget(btn_gen)
        ctrl_l.addWidget(code_grp)

        ctrl_l.addStretch()
        ctrl_scroll.setWidget(ctrl_w)
        layout.addWidget(ctrl_scroll, stretch=0)

        self._gen_c_array()

    # ── Live update ───────────────────────────────────────────────────────────

    def update_live(self, px: int, sig: int):
        self._live_px  = px
        self._live_sig = sig
        if sig < 60:
            self._sig_lbl.setText(f"sig: {sig} ✗")
            self._sig_lbl.setStyleSheet("color: #e03131;")
        else:
            self._sig_lbl.setText(f"sig: {sig} ✓")
            self._sig_lbl.setStyleSheet("color: #51cf66;")
        mm = self._lut[px] if 0 <= px < 256 else 0
        self._live_dot.setData([px], [mm]) if px > 0 else self._live_dot.setData([], [])

    # ── Capture ───────────────────────────────────────────────────────────────

    def _capture(self):
        if self._live_px == 0 and self._live_sig == 0:
            self._cap_status.setText("No live data — connect and stream first")
            self._cap_status.setStyleSheet("color: #e03131;")
            return
        px = self._live_px
        mm = self._dist_spin.value()
        self._data_px.append(px)
        self._data_mm.append(mm)
        self._refresh_table()
        self._scatter.setData(self._data_px, self._data_mm)
        n = len(self._data_px)
        self._cap_status.setText(f"{n} point{'s' if n != 1 else ''} captured  (x_pos={px})")
        self._cap_status.setStyleSheet("color: #51cf66;")
        # Auto-advance to the next known distance (10 mm step), clamped to range max
        self._dist_spin.setValue(min(mm + 10, self._dist_spin.maximum()))

    def _clear_data(self):
        self._data_px.clear()
        self._data_mm.clear()
        self._table.setRowCount(0)
        self._scatter.setData([], [])
        self._cap_status.setText("Data cleared")
        self._cap_status.setStyleSheet("color: #888;")

    def _refresh_table(self):
        self._table.setRowCount(len(self._data_px))
        for row, (px, mm) in enumerate(zip(self._data_px, self._data_mm)):
            lut_mm = self._lut[px] if 0 <= px < 256 else 255
            for col, val in enumerate([px, mm, lut_mm]):
                item = QTableWidgetItem(str(val))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table.setItem(row, col, item)

    # ── Interpolate ───────────────────────────────────────────────────────────

    def _interpolate(self):
        n = len(self._data_px)
        if n < 2:
            self._interp_status.setText("Need ≥ 2 captured points")
            self._interp_status.setStyleSheet("color: #e03131;")
            return

        pts = sorted(zip(self._data_px, self._data_mm))
        pxs = [p[0] for p in pts]
        mms = [p[1] for p in pts]

        for i in range(256):
            if i < pxs[0] or i > pxs[-1]:
                self._lut[i] = 255   # outside calibrated range
            else:
                j = bisect.bisect_right(pxs, i) - 1
                if j >= len(pxs) - 1 or pxs[j] == i:
                    self._lut[i] = max(1, min(254, round(mms[j])))
                else:
                    t = (i - pxs[j]) / (pxs[j + 1] - pxs[j])
                    mm = mms[j] + t * (mms[j + 1] - mms[j])
                    self._lut[i] = max(1, min(254, round(mm)))

        self._lut_line.setData(list(range(256)), self._lut)
        self._refresh_table()
        self._gen_c_array()
        self._interp_status.setText(
            f"LUT built from {n} points  (x_pos {pxs[0]}–{pxs[-1]})")
        self._interp_status.setStyleSheet("color: #51cf66;")
        self.lut_updated.emit(self.ch_idx, self._lut[:])

    # ── Send LUT to firmware ──────────────────────────────────────────────────

    def _start_send_lut(self):
        ch = self.ch_idx
        self._send_queue = [f"W{ch},{px},{self._lut[px]}" for px in range(256)]
        self._send_bar.setValue(0)
        self._send_bar.setVisible(True)
        self._btn_send.setEnabled(False)
        self._send_timer.start()

    def _send_next(self):
        if not self._send_queue:
            self._send_timer.stop()
            self._send_bar.setValue(256)
            self._send_bar.setVisible(False)
            self._btn_send.setEnabled(True)
            self._btn_send.setText("✓ LUT sent")
            return
        self._send_fn(self._send_queue.pop(0))
        self._send_bar.setValue(256 - len(self._send_queue))

    # ── C array ───────────────────────────────────────────────────────────────

    def _gen_c_array(self):
        names = ["lut_left", "lut_front_r", "lut_front_l", "lut_right"]
        lines = [f"uint8_t {names[self.ch_idx]}[256] = {{"]
        for i in range(0, 256, 16):
            row = self._lut[i:i + 16]
            lines.append("    " + ", ".join(f"{v:3d}" for v in row) + ",")
        lines.append("};")
        self._code_edit.setPlainText("\n".join(lines))

    # ── Persistence ───────────────────────────────────────────────────────────

    def get_params(self) -> dict:
        return {"lut": self._lut[:], "data_px": self._data_px[:], "data_mm": self._data_mm[:]}

    def load_params(self, d: dict):
        if "lut" in d and len(d["lut"]) == 256:
            self._lut = list(d["lut"])
        self._data_px = list(d.get("data_px", []))
        self._data_mm = list(d.get("data_mm", []))
        self._lut_line.setData(list(range(256)), self._lut)
        self._scatter.setData(self._data_px, self._data_mm)
        self._refresh_table()
        self._gen_c_array()

# =============================================================================
# WallParamsWidget — reference distances for wall_sensors.ino on main MCU
# =============================================================================

class WallParamsWidget(QWidget):
    """
    Lwall / Rwall / WALL_mm reference values used in wall_sensors.ino.
    These are NOT sent to the sensor sub-MCU — they go in the main MCU firmware.
    """

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        info = QLabel(
            "These reference values belong in <b>wall_sensors.ino</b> on the main MCU.\n"
            "They are not sent to the sensor sub-MCU.\n\n"
            "• <b>Lwall / Rwall</b>: expected distance (mm) to each wall when the mouse\n"
            "  is centred in a cell. Used as the reference in wall_tracking_error.\n\n"
            "• <b>WALL_mm</b>: maximum distance at which a side wall is considered present.\n\n"
            "• <b>FRONT_WALL_mm</b>: maximum front distance to consider a front wall present."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color: #aaa; font-size: 10px;")
        layout.addWidget(info)

        grp = QGroupBox("Reference values")
        form = QGridLayout(grp)
        form.setSpacing(8)

        def spin_row(label, default, min_val, max_val, row):
            lbl = QLabel(label)
            sp  = QSpinBox()
            sp.setRange(min_val, max_val)
            sp.setValue(default)
            sp.setSuffix(" mm")
            sp.setFixedWidth(90)
            form.addWidget(lbl, row, 0)
            form.addWidget(sp,  row, 1)
            return sp

        self._lwall = spin_row("Lwall (left reference dist):",  81,  30, 200, 0)
        self._rwall = spin_row("Rwall (right reference dist):", 84,  30, 200, 1)
        self._wall  = spin_row("WALL_mm (side wall threshold):", 120, 50, 254, 2)
        self._front = spin_row("FRONT_WALL_mm:",               180,  50, 254, 3)
        layout.addWidget(grp)

        gen_btn = QPushButton("Generate Code")
        gen_btn.setFixedWidth(130)
        gen_btn.clicked.connect(self._gen_code)
        layout.addWidget(gen_btn)

        self._code = QTextEdit()
        self._code.setFont(QFont("Courier", 9))
        self._code.setReadOnly(True)
        self._code.setStyleSheet("background: #141414; color: #51cf66;")
        self._code.setFixedHeight(120)
        layout.addWidget(self._code)
        layout.addStretch()

        self._gen_code()

    def _gen_code(self):
        lw = self._lwall.value()
        rw = self._rwall.value()
        wm = self._wall.value()
        fw = self._front.value()
        today = date.today().isoformat()
        code = (
            f"// wall_sensors.ino reference values — updated {today}\n"
            f"#define Lwall  {lw}    // left wall centring reference (mm)\n"
            f"#define Rwall  {rw}    // right wall centring reference (mm)\n"
            f"\n"
            f"// config.h thresholds\n"
            f"#define WALL_mm        {wm}\n"
            f"#define FRONT_WALL_mm  {fw}\n"
        )
        self._code.setPlainText(code)

    def get_values(self) -> dict:
        return {
            "lwall": self._lwall.value(), "rwall": self._rwall.value(),
            "wall_mm": self._wall.value(), "front_wall_mm": self._front.value(),
        }

    def load_values(self, d: dict):
        if "lwall"        in d: self._lwall.setValue(d["lwall"])
        if "rwall"        in d: self._rwall.setValue(d["rwall"])
        if "wall_mm"      in d: self._wall.setValue(d["wall_mm"])
        if "front_wall_mm" in d: self._front.setValue(d["front_wall_mm"])
        self._gen_code()

# =============================================================================
# SensorParamsWidget — tunable S15366 parameters sent over USB
# =============================================================================

class ChannelSensorParams(QWidget):
    """Sensor registers for ONE channel, to sit on that channel's own tab.

    Two scopes are mixed here and the distinction is hardware, not cosmetic:

      per-CHANNEL  integration time ('i<ch><us>'), laser enable ('L<ch><0/1>')
      per-IC       gain / ADC / calc_area / threshold ('g','a','c','t' + <ic>)

    CHANNEL_IC is [0, 1, 1, 2] -- Front-R and Front-L are BOTH on IC1, so the
    per-IC controls on those two tabs are one physical register set. They are
    shown on both and mirrored (see _wire_ic_mirrors in the main window) so the
    two tabs can never display different values for the same register.
    """

    IC_FIELDS = ("gain_bin", "gain_ro", "gain_ca",
                 "adc_lo", "adc_hi", "calc_area", "thresh")

    def __init__(self, ch: int, send_cmd):
        super().__init__()
        self.ch = ch
        self.ic = CHANNEL_IC[ch]
        self._send = send_cmd
        self.ic_widgets = {}

        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.setContentsMargins(10, 8, 10, 10)

        def row(label, widget, btn, parent_layout):
            h = QHBoxLayout()
            l = QLabel(label)
            l.setFixedWidth(150)
            l.setStyleSheet("color: #888; font-size: 10px;")
            h.addWidget(l)
            h.addWidget(widget)
            if btn is not None:
                h.addWidget(btn)
            parent_layout.addLayout(h)

        def apply_btn(fn):
            b = QPushButton("Set")
            b.setFixedWidth(36)
            b.setFixedHeight(22)
            b.setStyleSheet("font-size: 10px;")
            b.clicked.connect(fn)
            return b

        # ── per-channel ───────────────────────────────────────────────────────
        chan_box = QGroupBox(f"This channel only  —  {CHANNEL_DEFAULTS[ch][0]}")
        chan_box.setStyleSheet("QGroupBox { color: #aaa; font-size: 10px; "
                               "font-weight: bold; border: 1px solid #333; "
                               "border-radius: 4px; margin-top: 6px; padding-top: 8px; }"
                               "QGroupBox::title { subcontrol-origin: margin; left: 8px; }")
        chan_l = QVBoxLayout(chan_box)

        self.integ = QSpinBox()
        self.integ.setRange(1, 2047)
        self.integ.setValue(400 if ch in (1, 2) else 300)
        self.integ.setSingleStep(25)
        self.integ.setToolTip(
            f"MST LOW duration in µs for this channel only (sub-MCU 'i{ch}<us>').\n"
            "Wind down with a close target to take the spot out of saturation.")
        row("integration (µs):", self.integ,
            apply_btn(lambda: self._send(f"i{self.ch}{self.integ.value()}")), chan_l)

        self.laser = QPushButton("Laser ON")
        self.laser.setCheckable(True)
        self.laser.setChecked(True)
        self.laser.setFixedHeight(22)
        self.laser.setStyleSheet(
            "QPushButton { background: #2d7a2d; color: white; border-radius: 3px; font-size: 10px; }"
            "QPushButton:!checked { background: #3a3a3a; color: #777; }")
        self.laser.clicked.connect(self._toggle_laser)
        row("laser:", self.laser, None, chan_l)
        layout.addWidget(chan_box)

        # ── per-IC ────────────────────────────────────────────────────────────
        shared = [CHANNEL_DEFAULTS[c][0] for c in range(4)
                  if CHANNEL_IC[c] == self.ic and c != ch]
        title = f"IC{self.ic} registers"
        if shared:
            title += f"  —  SHARED with {', '.join(shared)}"
        ic_box = QGroupBox(title)
        ic_box.setStyleSheet(
            "QGroupBox { color: %s; font-size: 10px; font-weight: bold; "
            "border: 1px solid %s; border-radius: 4px; margin-top: 6px; padding-top: 8px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 8px; }"
            % (("#d0a050", "#5a4520") if shared else ("#aaa", "#333")))
        ic_l = QVBoxLayout(ic_box)
        if shared:
            warn = QLabel(f"One physical register set. Changing these also "
                          f"changes {', '.join(shared)}.")
            warn.setWordWrap(True)
            warn.setStyleSheet("color: #d0a050; font-size: 9px; border: none;")
            ic_l.addWidget(warn)

        BINNING_OPTS = ["0 — 1× (off)", "1 — 2×", "2 — 4×", "3 — 8×"]
        CALC_AREA_OPTS = ["0 — 5 px", "1 — 9 px", "2 — 19 px"]
        ic = self.ic

        def gain_byte():
            return ((self.ic_widgets["gain_ca"].value() << 6) |
                    (self.ic_widgets["gain_ro"].value() << 2) |
                    self.ic_widgets["gain_bin"].currentIndex())

        def adc_byte():
            return ((self.ic_widgets["adc_hi"].value() << 4) |
                    self.ic_widgets["adc_lo"].value())

        # Defaults MIRROR the sub-MCU's compiled gain_val/threshold_val, so a
        # freshly-opened GUI shows what the hardware actually booted with
        # rather than a stale guess. IC0 (left) and IC2 (right): binning 0,
        # threshold 20. IC1 (front pair): binning 3, threshold 60.
        w = QComboBox(); w.addItems(BINNING_OPTS)
        w.setCurrentIndex(3 if ic == 1 else 0)
        w.setToolTip(f"gain[{ic}] bits 0-1 — binning")
        self.ic_widgets["gain_bin"] = w
        row("gain_bin (bits 0-1):", w, apply_btn(lambda: self._send(f"g{ic}{gain_byte()}")), ic_l)

        w = QSpinBox(); w.setRange(0, 15); w.setValue(0)
        w.setToolTip(f"gain[{ic}] bits 2-5 — readout gain")
        self.ic_widgets["gain_ro"] = w
        row("gain_ro (bits 2-5):", w, apply_btn(lambda: self._send(f"g{ic}{gain_byte()}")), ic_l)

        w = QSpinBox(); w.setRange(0, 3); w.setValue(0)
        w.setToolTip(f"gain[{ic}] bits 6-7 — charge amplifier")
        self.ic_widgets["gain_ca"] = w
        row("gain_ca (bits 6-7):", w, apply_btn(lambda: self._send(f"g{ic}{gain_byte()}")), ic_l)

        w = QSpinBox(); w.setRange(0, 15); w.setValue(0x5)
        w.setPrefix("0x"); w.setDisplayIntegerBase(16)
        w.setToolTip(f"ADC[{ic}] bits 0-3 — lower voltage limit")
        self.ic_widgets["adc_lo"] = w
        row("adc_lo (bits 0-3):", w, apply_btn(lambda: self._send(f"a{ic}{adc_byte()}")), ic_l)

        w = QSpinBox(); w.setRange(0, 15); w.setValue(0xF)
        w.setPrefix("0x"); w.setDisplayIntegerBase(16)
        w.setToolTip(f"ADC[{ic}] bits 4-7 — upper voltage limit")
        self.ic_widgets["adc_hi"] = w
        row("adc_hi (bits 4-7):", w, apply_btn(lambda: self._send(f"a{ic}{adc_byte()}")), ic_l)

        w = QComboBox(); w.addItems(CALC_AREA_OPTS)
        w.setCurrentIndex(1 if ic == 1 else 0)
        w.setToolTip(f"calc_area_val[{ic}] — spot detection area")
        self.ic_widgets["calc_area"] = w
        row("calc_area:", w,
            apply_btn(lambda: self._send(f"c{ic}{self.ic_widgets['calc_area'].currentIndex()}")), ic_l)

        w = QSpinBox(); w.setRange(0, 255); w.setValue(60 if ic == 1 else 20)
        w.setToolTip(f"threshold_val[{ic}] — min peak signal for a valid spot.\n"
                     "Lowering this is what lets the laser run at short "
                     "integration times.")
        self.ic_widgets["thresh"] = w
        row("threshold:", w,
            apply_btn(lambda: self._send(f"t{ic}{self.ic_widgets['thresh'].value()}")), ic_l)

        layout.addWidget(ic_box)

        btn = QPushButton("Apply this channel to firmware")
        btn.setToolTip("Integration + this channel's IC registers, in one go")
        btn.clicked.connect(self.apply_all)
        layout.addWidget(btn)
        layout.addStretch()

    def _toggle_laser(self):
        on = self.laser.isChecked()
        self.laser.setText("Laser ON" if on else "Laser OFF")
        self._send(f"L{self.ch}{1 if on else 0}")

    def apply_all(self):
        ic = self.ic
        self._send(f"i{self.ch}{self.integ.value()}")
        self._send(f"g{ic}{(self.ic_widgets['gain_ca'].value() << 6) | (self.ic_widgets['gain_ro'].value() << 2) | self.ic_widgets['gain_bin'].currentIndex()}")
        self._send(f"a{ic}{(self.ic_widgets['adc_hi'].value() << 4) | self.ic_widgets['adc_lo'].value()}")
        self._send(f"c{ic}{self.ic_widgets['calc_area'].currentIndex()}")
        self._send(f"t{ic}{self.ic_widgets['thresh'].value()}")

    def set_integ_display(self, us: int):
        self.integ.blockSignals(True)
        self.integ.setValue(int(us))
        self.integ.blockSignals(False)


class SensorParamsWidget(QWidget):
    """
    Controls for all configurable S15366 parameters.
    Sends firmware parameter commands (i, b, g, a, c, t, x, n) with '\n' terminator.
    """

    def __init__(self, send_cmd_fn):
        super().__init__()
        self._send = send_cmd_fn
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.setSpacing(6)
        layout.setContentsMargins(10, 10, 10, 10)

        def section(text):
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #aaa; font-weight: bold; font-size: 10px; margin-top: 6px;")
            layout.addWidget(lbl)

        def row(label, widget, btn):
            h = QHBoxLayout()
            l = QLabel(label)
            l.setFixedWidth(130)
            l.setStyleSheet("color: #888; font-size: 10px;")
            h.addWidget(l)
            h.addWidget(widget)
            h.addWidget(btn)
            layout.addLayout(h)

        def apply_btn(cmd_fn):
            b = QPushButton("Set")
            b.setFixedWidth(36)
            b.setFixedHeight(22)
            b.setStyleSheet("font-size: 10px;")
            b.clicked.connect(cmd_fn)
            return b

        # ── Global params ──────────────────────────────────────────────────────
        section("Global")

        # Group setters. Each channel also has its own spinbox on its tile
        # ('i<ch><us>'); these two just set the pair in one go and then ask the
        # sub-MCU to report, so the per-channel tiles refresh to match.
        self._integ_side = QSpinBox()
        self._integ_side.setRange(1, 2047)
        self._integ_side.setValue(300)
        self._integ_side.setToolTip(
            "MST LOW duration in µs for Right & Left (ch0 + ch3).\n"
            "Per-channel override lives on each channel tile.")
        row("integ_side (R/L):", self._integ_side,
            apply_btn(lambda: (self._send(f"is{self._integ_side.value()}"),
                               self._send("i"))))

        self._integ_front = QSpinBox()
        self._integ_front.setRange(1, 2047)
        self._integ_front.setValue(400)
        self._integ_front.setToolTip(
            "MST LOW duration in µs for Front-R & Front-L (ch1 + ch2).\n"
            "Per-channel override lives on each channel tile.")
        row("integ_front (FR/FL):", self._integ_front,
            apply_btn(lambda: (self._send(f"if{self._integ_front.value()}"),
                               self._send("i"))))

        readback = QPushButton("Read back integration from sub-MCU")
        readback.setFixedHeight(22)
        readback.setStyleSheet("font-size: 10px;")
        readback.setToolTip("Sends bare 'i' -- the reply updates the integration "
                            "box on every channel tab and tile")
        readback.clicked.connect(lambda: self._send("i"))
        layout.addWidget(readback)

        self._blank = QSpinBox()
        self._blank.setRange(0, 255)
        self._blank.setValue(0)
        self._blank.setToolTip("blanking_val — blanking level")
        row("blanking_val:", self._blank,
            apply_btn(lambda: self._send(f"b{self._blank.value()}")))

        self._xstart = QSpinBox()
        self._xstart.setRange(0, 254)
        self._xstart.setValue(0)
        self._xstart.setToolTip("Xpixel_start_val — X readout start pixel")
        row("Xpixel_start:", self._xstart,
            apply_btn(lambda: self._send(f"x{self._xstart.value()}")))

        self._npix = QSpinBox()
        self._npix.setRange(1, 255)
        self._npix.setValue(255)
        self._npix.setToolTip("readout_pixels — number of pixels to read")
        row("readout_pixels:", self._npix,
            apply_btn(lambda: self._send(f"n{self._npix.value()}")))

        # Per-IC registers (gain / ADC / calc_area / threshold) now live on
        # each CHANNEL tab -- see ChannelSensorParams. They are per-IC, and
        # CHANNEL_IC = [0,1,1,2] means Front-R and Front-L share IC1, so the
        # two front tabs mirror one physical register set. Only genuinely
        # global settings stay here: blanking / Xpixel_start / readout_pixels
        # are applied with apply_params(3), i.e. to all three ICs at once.

        # ── Apply all button ───────────────────────────────────────────────────
        layout.addSpacing(8)
        btn_all = QPushButton("Apply global params to firmware")
        btn_all.setToolTip("Integration pairs + blanking / Xpixel_start / readout_pixels.\n"
                           "Gain, ADC, calc_area and threshold are per-IC and live on\n"
                           "the channel tabs — apply them there.")
        btn_all.clicked.connect(self._apply_all)
        layout.addWidget(btn_all)

    def _apply_all(self):
        """Global settings only. The per-IC registers moved to the channel
        tabs, so they are applied from there ('Apply this channel')."""
        self._send(f"is{self._integ_side.value()}")
        self._send(f"if{self._integ_front.value()}")
        self._send(f"b{self._blank.value()}")
        self._send(f"x{self._xstart.value()}")
        self._send(f"n{self._npix.value()}")
        self._send("i")          # report back, refreshing the channel tabs


# =============================================================================
# LaserGeometryWidget — per-laser mounting offset + angle (GUI-side, for
# converting the direct beam range into a perpendicular wall distance)
# =============================================================================

class LaserGeometryWidget(QWidget):
    """Editable mounting geometry for each laser. The calibrated distance is the
    direct along-beam range; this widget converts it to the perpendicular wall
    distance from the mouse centre and shows both live."""

    def __init__(self, on_change=None):
        super().__init__()
        self._on_change = on_change
        self._kind = [g["kind"] for g in LASER_GEOMETRY_DEFAULTS]
        self._x, self._y, self._ang = [], [], []
        self._beam_lbl, self._wall_lbl = [], []

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        intro = QLabel(
            "Body frame: x = forward, y = left (mm); angle = degrees CCW from "
            "forward (0° ahead, +90° left, −90° right).\n"
            "Calibrated distance is the direct range along the beam. Wall "
            "distance is computed here — front lasers report forward distance, "
            "side lasers report lateral distance."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet("color: #888; font-size: 10px;")
        layout.addWidget(intro)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        for c, htext in enumerate(
                ["Laser", "x fwd (mm)", "y left (mm)", "angle (°)", "beam r", "wall"]):
            h = QLabel(htext)
            h.setStyleSheet("color: #aaa; font-weight: bold; font-size: 10px;")
            grid.addWidget(h, 0, c)

        def mkspin(val, lo, hi):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(1)
            s.setSingleStep(1.0)
            s.setValue(val)
            s.setFixedWidth(78)
            return s

        for i, g in enumerate(LASER_GEOMETRY_DEFAULTS):
            name = QLabel(CHANNEL_DEFAULTS[i][0])
            name.setStyleSheet(f"color: {CHANNEL_COLORS[i]}; font-weight: bold; font-size: 10px;")
            grid.addWidget(name, i + 1, 0)

            sx = mkspin(g["x"],    -200.0, 200.0)
            sy = mkspin(g["y"],    -200.0, 200.0)
            sa = mkspin(g["angle"], -180.0, 180.0)
            self._x.append(sx); self._y.append(sy); self._ang.append(sa)
            grid.addWidget(sx, i + 1, 1)
            grid.addWidget(sy, i + 1, 2)
            grid.addWidget(sa, i + 1, 3)
            for s in (sx, sy, sa):
                s.valueChanged.connect(self._changed)

            bl = QLabel("---")
            bl.setFixedWidth(54)
            bl.setStyleSheet("color: #888; font-family: Courier; font-size: 10px;")
            wl = QLabel("---")
            wl.setFixedWidth(54)
            wl.setStyleSheet(f"color: {CHANNEL_COLORS[i]}; font-family: Courier; font-size: 10px;")
            self._beam_lbl.append(bl); self._wall_lbl.append(wl)
            grid.addWidget(bl, i + 1, 4)
            grid.addWidget(wl, i + 1, 5)

        layout.addLayout(grid)

        note = QLabel(
            "Defaults: front lasers 28 mm forward, 18 mm apart (±9 mm), "
            "angled inward so beams cross ahead. Side beams cross — each "
            "side sensor reads the OPPOSITE wall: ‘Left’ is the right-mounted "
            "sensor at +72.8°, ‘Right’ is the left-mounted sensor at −71° "
            "(both 40 mm forward, ±15 mm). Angles are measured, fixed values."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #666; font-size: 9px; margin-top: 6px;")
        layout.addWidget(note)

    def _changed(self):
        if self._on_change:
            self._on_change()

    def geometry(self) -> list:
        return [
            {"x": self._x[i].value(), "y": self._y[i].value(),
             "angle": self._ang[i].value(), "kind": self._kind[i]}
            for i in range(4)
        ]

    def update_live(self, beam_ranges: list):
        geo = self.geometry()
        for i in range(4):
            r = beam_ranges[i]
            if 0 < r < 255:
                self._beam_lbl[i].setText(f"{r:.0f}")
                self._wall_lbl[i].setText(f"{r:.0f}")   # r is perpendicular
            else:
                self._beam_lbl[i].setText("---")
                self._wall_lbl[i].setText("---")

    def get_values(self) -> dict:
        return {"geometry": [
            {"x": self._x[i].value(), "y": self._y[i].value(), "angle": self._ang[i].value()}
            for i in range(4)
        ]}

    def load_values(self, d: dict):
        for i, gi in enumerate(d.get("geometry", [])):
            if i < 4:
                if "x"     in gi: self._x[i].setValue(gi["x"])
                if "y"     in gi: self._y[i].setValue(gi["y"])
                if "angle" in gi: self._ang[i].setValue(gi["angle"])

# =============================================================================
# Main window
# =============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TW3 Sensor Calibration")
        self.resize(1300, 820)

        self.cfg     = load_config()
        self.reader  = None
        self._latest = [0] * 12  # last raw packet

        self._build_ui()
        self._refresh_ports()
        self._load_state()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        pg.setConfigOption("background", "#1e1e1e")
        pg.setConfigOption("foreground", "#cccccc")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(4)

        # ── Connection bar ────────────────────────────────────────────────────
        conn = QHBoxLayout()
        conn.addWidget(QLabel("Sensor sub-MCU port:"))

        self._port_combo = QComboBox()
        self._port_combo.setFixedWidth(150)
        conn.addWidget(self._port_combo)

        btn_refresh = QPushButton("⟳")
        btn_refresh.setFixedWidth(28)
        btn_refresh.clicked.connect(self._refresh_ports)
        conn.addWidget(btn_refresh)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setFixedWidth(90)
        self._btn_connect.clicked.connect(self._toggle_connect)
        conn.addWidget(self._btn_connect)

        self._conn_lbl = QLabel("● Disconnected")
        self._conn_lbl.setStyleSheet("color: #ff6b6b;")
        conn.addWidget(self._conn_lbl)

        conn.addSpacing(16)

        self._btn_stream = QPushButton("▶ Stream")
        self._btn_stream.setFixedWidth(80)
        self._btn_stream.setEnabled(False)
        self._btn_stream.clicked.connect(lambda: self._send_cmd("r"))
        conn.addWidget(self._btn_stream)

        self._btn_stop = QPushButton("■ Stop")
        self._btn_stop.setFixedWidth(70)
        self._btn_stop.setEnabled(False)
        self._btn_stop.clicked.connect(lambda: self._send_cmd("q"))
        conn.addWidget(self._btn_stop)

        conn.addSpacing(16)
        self._rate_lbl = QLabel("--- lines/s")
        self._rate_lbl.setStyleSheet("color: #888; font-family: Courier;")
        conn.addWidget(self._rate_lbl)

        conn.addStretch()
        root.addLayout(conn)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        root.addWidget(sep)

        # ── Main splitter ─────────────────────────────────────────────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # ── Left: dashboard ───────────────────────────────────────────────────
        dash = QWidget()
        dash_layout = QVBoxLayout(dash)
        dash_layout.setSpacing(6)

        # IC status indicators
        ic_row = QHBoxLayout()
        ic_row.addWidget(QLabel("Sensor ICs:"))
        # IC labels are by mount side (hardware), not wall measured — IC0 is
        # right-mounted (reads left wall), IC2 left-mounted (reads right wall).
        ic_names = ["IC0 (R-mount)", "IC1 (Front)", "IC2 (L-mount)"]
        self._ic_labels = []
        for nm in ic_names:
            lbl = QLabel(f"● {nm}")
            lbl.setStyleSheet("color: #555; font-size: 10px;")
            lbl.setFont(QFont("Courier", 10))
            self._ic_labels.append(lbl)
            ic_row.addWidget(lbl)
        ic_row.addStretch()
        self._fps_lbl = QLabel("--- Hz")
        self._fps_lbl.setStyleSheet("color: #777; font-family: Courier; font-size: 10px;")
        self._fps_lbl.setToolTip("Frame rate — time to read all active sensors once")
        ic_row.addWidget(self._fps_lbl)
        dash_layout.addLayout(ic_row)

        # Channel tiles (2×2 grid)
        tiles_grid = QGridLayout()
        tiles_grid.setSpacing(4)
        self._tiles = [ChannelTile(i, self._send_cmd) for i in range(4)]
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for tile, (r, c) in zip(self._tiles, positions):
            tiles_grid.addWidget(tile, r, c)
        dash_layout.addLayout(tiles_grid)

        # Top-down view
        self._topdown = TopDownView()
        dash_layout.addWidget(self._topdown, stretch=1)

        splitter.addWidget(dash)

        # ── Right: calibration tabs ───────────────────────────────────────────
        self._tabs = QTabWidget()

        self._cal_widgets = []
        self._chan_params = []
        for i in range(4):
            w = ChannelCalWidget(i, self._send_cmd)
            params = ChannelSensorParams(i, self._send_cmd)
            self._chan_params.append(params)
            # Calibration on top, sensor registers underneath, one tab per
            # channel -- so a channel's LUT sweep and the gain/integration that
            # produced it are never on different screens.
            page = QWidget()
            page_l = QVBoxLayout(page)
            page_l.setContentsMargins(0, 0, 0, 0)
            page_l.addWidget(w)
            page_l.addWidget(params)
            scroll = QScrollArea()
            scroll.setWidget(page)
            scroll.setWidgetResizable(True)
            self._tabs.addTab(scroll, CHANNEL_DEFAULTS[i][0])
            self._cal_widgets.append(w)
        self._wire_ic_mirrors()

        wall_params = WallParamsWidget()
        self._wall_params = wall_params
        self._tabs.addTab(wall_params, "Wall Params")

        # ── Sensor IC params tab ──────────────────────────────────────────────
        self._sensor_params = SensorParamsWidget(self._send_cmd)
        self._tabs.addTab(self._sensor_params, "Sensor Params")

        # ── Laser geometry tab ────────────────────────────────────────────────
        self._geometry = LaserGeometryWidget(on_change=self._on_geometry_changed)
        self._tabs.addTab(self._geometry, "Laser Geometry")
        self._topdown.set_geometry(self._geometry.geometry())

        # ── Raw log tab ───────────────────────────────────────────────────────
        from PyQt6.QtWidgets import QPlainTextEdit
        raw_widget = QWidget()
        raw_layout = QVBoxLayout(raw_widget)
        raw_layout.addWidget(QLabel(
            "Raw bytes received from sub-MCU USB port.\n"
            "If empty after streaming: wrong port, or sub-MCU not running.\n"
            "Expected format (40 fields): [frame,x_int,x_dec,y_int,y_dec,sum_u,sum_l,sig_x,sig_y,mm] × 4 channels"
        ))
        self._raw_log = QPlainTextEdit()
        self._raw_log.setReadOnly(True)
        self._raw_log.setFont(QFont("Courier", 9))
        self._raw_log.setStyleSheet("background: #0d0d0d; color: #51cf66;")
        raw_layout.addWidget(self._raw_log)
        btn_clear_log = QPushButton("Clear Log")
        btn_clear_log.setFixedWidth(90)
        btn_clear_log.clicked.connect(self._raw_log.clear)
        raw_layout.addWidget(btn_clear_log)
        self._tabs.addTab(raw_widget, "Raw Log")

        # ── Help tab ──────────────────────────────────────────────────────────
        help_widget = QWidget()
        help_layout = QVBoxLayout(help_widget)
        help_layout.setContentsMargins(12, 12, 12, 12)
        help_layout.setSpacing(0)
        help_text = QTextEdit()
        help_text.setReadOnly(True)
        help_text.setStyleSheet("background: #1e1e1e; color: #cccccc; border: none;")
        help_text.setFont(QFont("Segoe UI", 10))
        help_text.setHtml("""
<style>
  body  { background:#1e1e1e; color:#cccccc; font-family:'Segoe UI',sans-serif; font-size:10pt; }
  h2    { color:#ffd43b; margin-top:18px; margin-bottom:4px; }
  h3    { color:#74c0fc; margin-top:14px; margin-bottom:3px; }
  p     { margin:4px 0 8px 0; }
  li    { margin:3px 0; }
  code  { background:#2d2d2d; color:#51cf66; padding:1px 4px; border-radius:3px; font-family:Courier,monospace; }
  .note { color:#ffa94d; }
  .warn { color:#e03131; }
  table { border-collapse:collapse; margin:6px 0; }
  td,th { padding:3px 10px; border:1px solid #444; }
  th    { background:#2a2a2a; color:#74c0fc; }
</style>

<h2>Sensor Calibration Procedure — TW3 Micromouse</h2>

<h3>Overview</h3>
<p>Each wall sensor (Right, Front-R, Front-L, Left) uses a 256-entry lookup table (LUT) to convert the S15366-256 image sensor's <code>x_pos</code> output (0–255) directly into a wall distance in mm. The LUT starts as a linear ramp and is populated from real physical measurements taken at known distances.</p>

<h3>Hardware Setup</h3>
<ul>
  <li>Connect the sensor sub-MCU (Teensy 4.0) to this PC via USB.</li>
  <li>Select the correct COM port in the top-left dropdown and click <b>Connect</b>.</li>
  <li>Place the mouse in a corridor or against a flat wall for the sensor being calibrated.</li>
  <li>Use a known flat surface (a maze wall section works best) and measure distances with calipers.</li>
</ul>

<h3>Step 1 — Check Signal Quality</h3>
<ul>
  <li>Click <b>Start Stream</b>. The live dashboard should show all four channels updating.</li>
  <li>On the channel tab you are calibrating, watch the <b>sig</b> indicator (top-right of the capture panel).</li>
  <li>A green <b>✓</b> means signal is strong enough. Red <b>✗</b> means the laser is too weak or the wall is too far / too close — check laser power (Sensor Params → gain) and the physical setup.</li>
  <li class="note">The NJL5820R laser disc must be 0.5–1.5 mm from the sensor IC. Closer causes over-counting.</li>
</ul>

<h3>Step 2 — Capture Calibration Points</h3>
<ul>
  <li>Position the mouse so the target wall is at a known distance (start at ~30 mm, work out in 10 mm steps).</li>
  <li>Wait for the <code>x_pos</code> value on the live plot to stabilise.</li>
  <li>Enter the known distance (mm) in the <b>Known dist</b> field.</li>
  <li>Click <b>Capture</b>. The point is added to the table and scatter plot.</li>
  <li>Repeat for each 10 mm increment across the full working range of that sensor (typically 20–200 mm).</li>
  <li>Aim for <b>at least 8–10 points</b> spread across the range. More points near close distances (where the curve is steepest) give better accuracy.</li>
  <li class="warn">Do not move the mouse while capturing a point — wait for a steady reading first.</li>
</ul>

<h3>Step 3 — Interpolate the LUT</h3>
<ul>
  <li>Once you have captured points across the full range, click <b>Interpolate LUT from captured points</b>.</li>
  <li>The firmware will receive a linearly interpolated value for every x_pos between your measured points.</li>
  <li>x_pos values outside your calibrated range will be set to <b>255</b> (out of range).</li>
  <li>The LUT curve on the plot will update to show the interpolated result.</li>
</ul>

<h3>Step 4 — Send LUT to Firmware</h3>
<ul>
  <li>Click <b>Send 256 entries</b>. This sends 256 <code>W&lt;ch&gt;,&lt;px&gt;,&lt;mm&gt;</code> commands to the sub-MCU at 8 ms intervals (~2 seconds total).</li>
  <li>Watch the progress bar. When it completes, the button shows <b>✓ LUT sent</b>.</li>
  <li>The LUT is held in RAM — it resets to the compiled default on power-cycle. Use <b>Generate C Array</b> to make it permanent.</li>
</ul>

<h3>Step 5 — Make it Permanent</h3>
<ul>
  <li>Click <b>Generate C Array</b> to produce a <code>uint8_t lut_xxx[256] = { ... };</code> block.</li>
  <li>Copy the array and paste it into <code>TW3_sensor_sub_mcu.ino</code>, replacing the existing table for that channel.</li>
  <li>Reflash the sub-MCU to make the calibration survive power cycles.</li>
</ul>

<h3>Calibration Order (Recommended)</h3>
<table>
  <tr><th>Tab</th><th>Sensor (mount)</th><th>Wall measured</th><th>x_pos direction</th></tr>
  <tr><td>Left</td><td>IC0 / MST[0] (right-mounted)</td><td>Left wall (beam crosses)</td><td>Direct (0 = close)</td></tr>
  <tr><td>Front-R</td><td>IC1 / MST[1]</td><td>Front wall</td><td>Inverted (255 = close)</td></tr>
  <tr><td>Front-L</td><td>IC1 / MST[1]</td><td>Front wall</td><td>Inverted (255 = close)</td></tr>
  <tr><td>Right</td><td>IC2 / MST[2] (left-mounted)</td><td>Right wall (beam crosses)</td><td>Inverted (255 = close)</td></tr>
</table>
<p><b>Side beams cross.</b> Each side sensor reads the opposite wall, so tabs are named by the wall measured, not the mounting side.</p>

<h3>Laser Enable / Disable</h3>
<p>Use the <b>● Laser ON</b> toggle on each channel tile to disable individual lasers while calibrating another channel. This prevents cross-talk when measuring front sensors with side lasers active.</p>

<h3>Sensor Params Tab</h3>
<p>Each IC has independent gain, ADC threshold and integration settings. Adjust these if signal is consistently weak or saturated:</p>
<ul>
  <li><b>Charge Amp (bits 6–7)</b> — main sensitivity; raise if signal is weak.</li>
  <li><b>Readout Gain (bits 2–5)</b> — fine gain adjust.</li>
  <li><b>ADC Hi/Lo nibbles</b> — voltage window for the ADC; defaults (Hi=0xF, Lo=0x5) work for most conditions.</li>
</ul>

<h3>Tips</h3>
<ul>
  <li>Calibrate in the same lighting conditions you will use for racing — ambient light affects the sensors.</li>
  <li>If a channel reads 255 at all distances, check that the laser is enabled and the IC was detected (check Raw Log for IC detection messages on boot).</li>
  <li>Calibration data is saved automatically when you close the GUI and reloaded on next launch.</li>
</ul>
""")
        help_layout.addWidget(help_text)
        self._tabs.addTab(help_widget, "Help")

        splitter.addWidget(self._tabs)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)

        root.addWidget(splitter)

        self.setStatusBar(QStatusBar())

    # ── Serial connection ─────────────────────────────────────────────────────

    def _refresh_ports(self):
        self._port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_combo.addItems(ports)
        last = self.cfg.get("sensor_port", "")
        if last in ports:
            self._port_combo.setCurrentText(last)

    def _toggle_connect(self):
        if self.reader and self.reader.isRunning():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_combo.currentText()
        if not port:
            return
        self.reader = SensorReader(port)
        self.reader.data_ready.connect(self._on_data)
        self.reader.ic_status.connect(self._on_ic_status)
        self.reader.loop_us_ready.connect(self._on_loop_us)
        self.reader.connection_lost.connect(self._on_conn_lost)
        self.reader.stats_ready.connect(self._on_stats)
        self.reader.port_ready.connect(self._on_port_ready)
        self.reader.raw_line.connect(self._on_raw_line)
        self.reader.format_warn.connect(self._on_format_warn)
        self.reader.start()
        self._btn_connect.setText("Disconnect")
        self._conn_lbl.setText("\u25cf Opening\u2026")
        self._conn_lbl.setStyleSheet("color: #ffd43b;")
        for b in [self._btn_stream, self._btn_stop]:
            b.setEnabled(True)
        self.cfg["sensor_port"] = port
        save_config(self.cfg)

    def _disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader.wait()
            self.reader = None
        self._btn_connect.setText("Connect")
        self._conn_lbl.setText("\u25cf Disconnected")
        self._conn_lbl.setStyleSheet("color: #ff6b6b;")
        for b in [self._btn_stream, self._btn_stop]:
            b.setEnabled(False)
        self.statusBar().showMessage("Disconnected")

    def _on_conn_lost(self):
        self._disconnect()
        self.statusBar().showMessage("Connection lost")

    def _send_cmd(self, cmd: str):
        if self.reader:
            self.reader.send_cmd(cmd)

    # Data handlers

    def _on_data(self, vals: list):
        """
        vals: 40-field list [frame,x_int,x_dec,y_int,y_dec,sum_u,sum_l,sig_x,sig_y,mm] x4 channels.
        Legacy 12-field data is already padded to 40 by the parser.
        """
        self._latest = vals
        ch    = [vals[i * CH_FIELDS:(i + 1) * CH_FIELDS] for i in range(4)]
        pxs   = [c[F_X_INT] for c in ch]
        sigs  = [c[F_SIG_X] for c in ch]
        dists = [c[F_MM]    for c in ch]

        # dists are the DIRECT along-beam ranges; derive perpendicular wall
        # distances from the per-laser mounting geometry.
        geom = self._geometry.geometry()

        for i, tile in enumerate(self._tiles):
            tile.set_readings(ch[i])
            r = dists[i]
            tile.set_wall(r if 0 < r < 255 else None)   # r is perpendicular

        for i, w in enumerate(self._cal_widgets):
            w.update_live(pxs[i], sigs[i])

        self._geometry.update_live(dists)
        self._topdown.set_readings(dists, geom)

    def _on_loop_us(self, loop_us: int):
        """Update frame rate display from firmware-reported measurement cycle time."""
        if loop_us > 0:
            hz = 1_000_000 / loop_us
            self._fps_lbl.setText(f"{hz:.1f} Hz  ({loop_us} µs)")

    def _on_ic_status(self, present: list):
        colors = ["#51cf66" if ok else "#555" for ok in present]
        for lbl, color in zip(self._ic_labels, colors):
            lbl.setStyleSheet(f"color: {color}; font-size: 10px;")

    def _on_port_ready(self):
        """Serial port is open in the reader thread -- send 'r' after a short delay
        so Teensy native USB CDC has time to be ready for RX."""
        self._conn_lbl.setText("Connected")
        self._conn_lbl.setStyleSheet("color: #51cf66;")
        self.statusBar().showMessage(
            f"Connected to {self._port_combo.currentText()} @ {BAUD_RATE} baud")
        QTimer.singleShot(600, lambda: self._send_cmd("r"))

    def _wire_ic_mirrors(self):
        """Channels sharing an IC share one physical register set (CHANNEL_IC
        is [0,1,1,2], so Front-R and Front-L are both IC1). Mirror the per-IC
        widgets between them so the two tabs can never show different values
        for the same register. Display only -- the mirrored copy is updated
        with signals blocked, so it neither re-sends nor bounces back."""
        for ic in range(3):
            group = [p for p in self._chan_params if p.ic == ic]
            if len(group) < 2:
                continue
            for field in ChannelSensorParams.IC_FIELDS:
                widgets = [p.ic_widgets[field] for p in group]
                for src in widgets:
                    peers = [w for w in widgets if w is not src]

                    def mirror(_val=None, _src=src, _peers=peers):
                        for w in _peers:
                            w.blockSignals(True)
                            if isinstance(w, QComboBox):
                                w.setCurrentIndex(_src.currentIndex())
                            else:
                                w.setValue(_src.value())
                            w.blockSignals(False)

                    if isinstance(src, QComboBox):
                        src.currentIndexChanged.connect(mirror)
                    else:
                        src.valueChanged.connect(mirror)

    _INTEG_RE = re.compile(r"ch([0-3])\([a-z_]+\)=(\d+)")

    def _on_raw_line(self, line: str):
        """Append every received line to the raw log (last 50 lines)."""
        # Sub-MCU integration readback ('i' with no argument, or the 'h' help
        # dump) -- reflect what is actually live so the spinboxes cannot drift
        # away from the hardware after a sub-MCU reboot or an 'is'/'if' group
        # set. Display only; set_integ_display does not echo a command back.
        if line.startswith("#") and "integ" in line:
            for ch, us in self._INTEG_RE.findall(line):
                ch, us = int(ch), int(us)
                self._tiles[ch].set_integ_display(us)
                if ch < len(self._chan_params):
                    self._chan_params[ch].set_integ_display(us)
        self._raw_log.appendPlainText(line)
        doc = self._raw_log.document()
        while doc.blockCount() > 50:
            cursor = self._raw_log.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _on_stats(self, lps: int):
        color = "#51cf66" if lps > 5 else "#ffd43b" if lps > 0 else "#ff6b6b"
        self._rate_lbl.setText(f"{lps} lines/s")
        self._rate_lbl.setStyleSheet(f"color: {color}; font-family: Courier;")

    def _on_format_warn(self, fmt: str):
        if fmt == "dist":
            self.statusBar().showMessage(
                "Sub-MCU is in DIST mode (4 values). Send 'r' for raw mode "
                "(12 values with px/sig needed for calibration).", 8000)
        elif fmt == "3ch":
            self.statusBar().showMessage(
                "Sub-MCU sending 3 values (legacy format). "
                "Flash latest firmware for full px/sig/mm output.", 8000)

    # State save / load

    def _on_geometry_changed(self):
        """Live-update the top-down view when mounting geometry is edited."""
        self._topdown.set_geometry(self._geometry.geometry())

    def _save_state(self):
        state = {
            "channel_params": [w.get_params() for w in self._cal_widgets],
            "wall_params":    self._wall_params.get_values(),
            "laser_geometry": self._geometry.get_values(),
        }
        self.cfg["cal_state"] = state
        save_config(self.cfg)

    def _load_state(self):
        state = self.cfg.get("cal_state", {})
        ch_list = state.get("channel_params", [])
        for i, d in enumerate(ch_list):
            if i < len(self._cal_widgets):
                self._cal_widgets[i].load_params(d)
        wp = state.get("wall_params", {})
        if wp:
            self._wall_params.load_values(wp)
        lg = state.get("laser_geometry", {})
        if lg:
            self._geometry.load_values(lg)
            self._topdown.set_geometry(self._geometry.geometry())

    def closeEvent(self, event):
        self._save_state()
        self._disconnect()
        event.accept()

# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window,          QColor(30, 30, 30))
    palette.setColor(QPalette.ColorRole.WindowText,      QColor(204, 204, 204))
    palette.setColor(QPalette.ColorRole.Base,            QColor(40, 40, 40))
    palette.setColor(QPalette.ColorRole.AlternateBase,   QColor(50, 50, 50))
    palette.setColor(QPalette.ColorRole.ToolTipBase,     QColor(30, 30, 30))
    palette.setColor(QPalette.ColorRole.ToolTipText,     QColor(204, 204, 204))
    palette.setColor(QPalette.ColorRole.Text,            QColor(204, 204, 204))
    palette.setColor(QPalette.ColorRole.Button,          QColor(55, 55, 55))
    palette.setColor(QPalette.ColorRole.ButtonText,      QColor(204, 204, 204))
    palette.setColor(QPalette.ColorRole.Highlight,       QColor(0, 120, 215))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())
