"""ADC constants, theme colors, and port configuration."""

# ADC constants
FS = 250                # Sample rate (Hz)
VREF = 4.5              # ADS1299 reference voltage (V)
GAIN = 24               # PGA gain
LSB_UV = (VREF / (2**23) / GAIN) * 1e6  # uV per LSB ≈ 0.02235 uV

# Default port configuration (7 ports, 42 devices, 336 channels)
DEFAULT_PORT_ARGS = (
    "0,0,Port1,8 0,1,Port2,7 3,0,Port3,5 3,1,Port4,5 "
    "4,0,Port5,5 4,1,Port6,5 5,0,Port7,7"
)

# Physical head layout for electrode grid (port_name, row, col)
PORT_ORDER = [
    ("Port6", 0, 0),   # Left
    ("Port7", 0, 1),   # Center-left
    ("Port1", 0, 2),   # Midline
    ("Port2", 0, 3),   # Center-right
    ("Port3", 0, 4),   # Right
    ("Port4", 1, 1),   # Posterior left
    ("Port5", 1, 3),   # Posterior right
]

# Theme colors
DARK_BG = "#0d0d0d"
SURFACE = "#1a1a1a"
SURFACE_RAISED = "#2a2a2a"
PLOT_BG = "#0a0a0a"
BORDER = "#333333"
BORDER_FOCUS = "#00ff88"
TEXT_PRIMARY = "#e0e0e0"
TEXT_SECONDARY = "#999999"
TEXT_MUTED = "#666666"
ACCENT = "#00ff88"
ACCENT_DIM = "#00aa55"
ACCENT_HOVER = "#00cc66"

# Status colors
STATUS_SUCCESS = "#00E676"
STATUS_GOOD = "#4CAF50"
STATUS_WARNING = "#FFC107"
STATUS_CAUTION = "#FF9800"
STATUS_ERROR = "#F44336"
STATUS_INFO = "#2196F3"

# Phase background colors
PHASE_COLORS = {
    "Prep":    "#1a1a2e",
    "Go":      "#0a2e0a",
    "Execute": "#0a2e0a",
    "End":     "#2e0a0a",
    "Rest":    SURFACE,
    "ITI":     DARK_BG,
}

# Channel neon colors (for signal plots)
NEON_COLORS = [
    "#00ffff",  # Cyan
    "#ff00ff",  # Magenta
    "#00ff00",  # Lime
    "#ffff00",  # Yellow
    "#ff6600",  # Orange
    "#ff0066",  # Hot pink
    "#6600ff",  # Purple
    "#00ffaa",  # Mint
]

# Port identity colors
PORT_COLORS = [
    "#ff6b6b",  # Coral red
    "#4ecdc4",  # Teal
    "#ffe66d",  # Yellow
    "#95e1d3",  # Mint
    "#f38181",  # Salmon
    "#aa96da",  # Lavender
    "#fcbad3",  # Pink
]

# Signal quality thresholds
SQ_EXCELLENT = 5_000     # < 5 kΩ
SQ_GOOD = 20_000         # < 20 kΩ
SQ_MARGINAL = 50_000     # < 50 kΩ
SQ_POOR = 100_000        # < 100 kΩ

FONT_FAMILY = "'Segoe UI', 'Inter', 'Helvetica Neue', Arial, sans-serif"

# Gestures
GESTURES = ["Open", "Close", "Pinch", "KeyGrip", "Rotate"]

# Video placeholder gesture colors
GESTURE_COLORS = {
    "Open":    "#2a6a2a",
    "Close":   "#6a2a2a",
    "Pinch":   "#2a2a6a",
    "KeyGrip": "#6a6a2a",
    "Rotate":  "#4a2a6a",
    "Rest":    "#3a3a3a",
}

# Gesture demonstration videos (relative to client/ directory)
import os as _os
_VIDEO_DIR = _os.path.join(
    _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))),
    "videos",
)
GESTURE_VIDEOS = {
    "KeyGrip": _os.path.join(_VIDEO_DIR, "IMG_5480.mp4"),
    "Close":   _os.path.join(_VIDEO_DIR, "IMG_5481.mp4"),
    "Pinch":   _os.path.join(_VIDEO_DIR, "IMG_5482.mp4"),
    "Open":    _os.path.join(_VIDEO_DIR, "IMG_5483.mp4"),
    "Rotate":  _os.path.join(_VIDEO_DIR, "Rotate.mp4"),
}
