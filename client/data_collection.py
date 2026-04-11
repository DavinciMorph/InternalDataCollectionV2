#!/usr/bin/env uv run script
# /// script
# dependencies = [
#   "PyQt5",
#   "opencv-python",
# ]
# ///
"""
BCI Gesture Paradigm — Stimulus Presentation & Event Logger
============================================================

Standalone stimulus presentation application for collecting labeled EEG data
during hand gesture experiments. Writes timestamped CSV event logs that are
later aligned offline with EEG recordings via wall_clock timestamps.

Usage:
    uv run gesture_paradigm.py
"""

import csv
import ctypes
import hashlib
import json
import os
import random
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from enum import Enum, auto
from queue import Queue, Empty
from typing import Optional

import cv2
import numpy as np

from PyQt5.QtCore import (
    Qt, QTimer, pyqtSignal, QSize, QPropertyAnimation, QEasingCurve,
)
from PyQt5.QtGui import (
    QFont, QColor, QPalette, QFontMetrics, QPainter, QKeySequence,
    QIntValidator, QDoubleValidator, QRegExpValidator, QImage, QPixmap,
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QFormLayout, QGridLayout,
    QLabel, QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox,
    QPushButton, QCheckBox, QRadioButton, QButtonGroup,
    QFrame, QProgressBar, QFileDialog, QMessageBox, QSizePolicy,
    QSpacerItem, QGraphicsOpacityEffect, QScrollArea,
)

# ---------------------------------------------------------------------------
# Windows high-resolution timer
# ---------------------------------------------------------------------------
if sys.platform == "win32":
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

# ---------------------------------------------------------------------------
# Theme constants
# ---------------------------------------------------------------------------
DARK_BG       = "#0d0d0d"
SURFACE       = "#1a1a1a"
SURFACE_RAISED = "#2a2a2a"
BORDER        = "#333333"
BORDER_FOCUS  = "#00ff88"
TEXT_PRIMARY   = "#e0e0e0"
TEXT_SECONDARY = "#999999"
TEXT_MUTED     = "#666666"
ACCENT        = "#00ff88"
ACCENT_DIM    = "#00aa55"

# Phase background colors
PHASE_COLORS = {
    "Prep":    "#1a1a2e",
    "Go":      "#0a2e0a",
    "Execute": "#0a2e0a",
    "End":     "#2e0a0a",
    "Rest":    SURFACE,
    "ITI":     DARK_BG,
}

# Video placeholder gesture colors
GESTURE_COLORS = {
    "Open":    "#2a6a2a",
    "Close":   "#6a2a2a",
    "Pinch":   "#2a2a6a",
    "KeyGrip": "#6a6a2a",
    "Rotate":  "#4a2a6a",
    "Rest":    "#3a3a3a",
}

FONT_FAMILY = "'Segoe UI', 'Inter', 'Helvetica Neue', Arial, sans-serif"

GESTURES = ["Open", "Close", "Pinch", "KeyGrip", "Rotate"]

# Gesture demonstration videos (relative to this script)
_VIDEO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gestures")
GESTURE_VIDEOS = {
    "KeyGrip": os.path.join(_VIDEO_DIR, "IMG_5480.mp4"),
    "Close":   os.path.join(_VIDEO_DIR, "close.mp4"),
    "Pinch":   os.path.join(_VIDEO_DIR, "IMG_5482.mp4"),
    "Open":    os.path.join(_VIDEO_DIR, "IMG_5483.mp4"),
    "Rotate":  os.path.join(_VIDEO_DIR, "Rotate.mp4"),
}

APP_STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_PRIMARY};
    font-family: {FONT_FAMILY};
    font-size: 14px;
}}
QLabel {{
    color: {TEXT_PRIMARY};
    background: transparent;
}}
QLineEdit {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    font-size: 14px;
}}
QLineEdit:focus {{
    border-color: {BORDER_FOCUS};
}}
QSpinBox, QDoubleSpinBox {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 14px;
}}
QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {BORDER_FOCUS};
}}
QComboBox {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 14px;
}}
QComboBox QAbstractItemView {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    selection-background-color: {ACCENT_DIM};
}}
QCheckBox {{
    color: {TEXT_PRIMARY};
    spacing: 8px;
    font-size: 14px;
}}
QCheckBox::indicator {{
    width: 18px;
    height: 18px;
    border: 1px solid {BORDER};
    border-radius: 3px;
    background: {SURFACE_RAISED};
}}
QCheckBox::indicator:checked {{
    background: {ACCENT_DIM};
    border-color: {ACCENT};
}}
QPushButton {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 8px 20px;
    font-size: 14px;
    font-weight: bold;
}}
QPushButton:hover {{
    background-color: #3a3a3a;
    border-color: #555555;
}}
QPushButton:pressed {{
    background-color: #1a1a1a;
}}
QProgressBar {{
    background-color: {SURFACE};
    border: none;
    border-radius: 2px;
    max-height: 4px;
}}
QProgressBar::chunk {{
    background-color: {ACCENT_DIM};
    border-radius: 2px;
}}
QRadioButton {{
    color: {TEXT_PRIMARY};
    font-size: 14px;
    spacing: 8px;
}}
"""


# ---------------------------------------------------------------------------
# Methodology definitions
# ---------------------------------------------------------------------------
@dataclass
class MethodologyDef:
    id: int
    name: str
    modality: str          # "physical" or "imagery"
    has_video: bool
    phases: list           # [(phase_name, duration_ms), ...]
    rest_phases: list      # phases for rest trial
    instruction_text: str
    prep_subtext: str
    execute_prefix: str    # "Perform:" or "Imagine:"
    gestures: list = field(default_factory=lambda: list(GESTURES))


METHODOLOGIES = {
    1: MethodologyDef(
        id=1,
        name="M1: Physical Execution (Cued)",
        modality="physical",
        has_video=False,
        phases=[("Prep", 2000), ("Go", 500), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "PHYSICAL EXECUTION WITH CUES\n\n"
            "You will see the name of a hand gesture on screen.\n\n"
            "1. PREPARE phase (blue): Get ready, but do NOT move yet.\n"
            "2. GO phase (green): Begin performing the gesture with your "
            "designated hand. Hold the position.\n"
            "3. EXECUTE phase: Continue holding the gesture.\n"
            "4. STOP phase (red): Relax your hand immediately.\n"
            "5. REST phase: Keep still and relax until the next trial.\n\n"
            "The five gestures are: Open (spread fingers), Close (make a fist), "
            "Pinch (thumb to index finger), Key Grip (thumb against side "
            "of index finger), and Rotate (rotate wrist).\n\n"
            "Try to keep the rest of your body still throughout."
        ),
        prep_subtext="Prepare to perform this gesture",
        execute_prefix="Perform:",
    ),
    2: MethodologyDef(
        id=2,
        name="M2: Motor Imagery — Close (Video)",
        modality="imagery",
        has_video=True,
        phases=[("Prep", 2000), ("Go", 500), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "MOTOR IMAGERY — CLOSE GESTURE WITH VIDEO\n\n"
            "Only one gesture is used in this paradigm: CLOSE (make a fist).\n\n"
            "1. PREPARE phase (blue): Watch the demonstration video of the "
            "Close gesture. Do NOT move yet.\n"
            "2. GO phase (green): A brief GO cue appears.\n"
            "3. EXECUTE phase: The video plays again. Vividly IMAGINE "
            "performing the Close gesture along with the video. Do NOT "
            "actually move your hand.\n"
            "4. STOP phase (red): Stop imagining.\n"
            "5. REST phase: Relax your mind until the next trial.\n\n"
            "Imagine the movement as vividly as possible — feel the fingers "
            "curling into a fist — but keep your hand completely still."
        ),
        prep_subtext="Watch the demonstration",
        execute_prefix="Imagine:",
        gestures=["Close"],
    ),
    3: MethodologyDef(
        id=3,
        name="M3: Physical Execution (No Cue)",
        modality="physical",
        has_video=False,
        phases=[("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "PHYSICAL EXECUTION — NO PREPARATION CUE\n\n"
            "The gesture name will appear immediately.\n\n"
            "1. When you see the gesture name (green), perform it right away "
            "and hold the position.\n"
            "2. STOP phase (red): Relax your hand immediately.\n"
            "3. REST phase: Keep still and relax.\n\n"
            "There is no preparation phase — react as quickly as you can.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="",
        execute_prefix="Perform:",
    ),
    4: MethodologyDef(
        id=4,
        name="M4: Motor Imagery (No Cue)",
        modality="imagery",
        has_video=False,
        phases=[("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "MOTOR IMAGERY — NO PREPARATION CUE\n\n"
            "The gesture name will appear immediately.\n\n"
            "1. When you see the gesture name (green), begin IMAGINING the "
            "gesture. Do NOT move your hand.\n"
            "2. STOP phase (red): Stop imagining.\n"
            "3. REST phase: Relax your mind.\n\n"
            "React as quickly as you can when the gesture appears.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="",
        execute_prefix="Imagine:",
    ),
    5: MethodologyDef(
        id=5,
        name="M5: Physical + Video Demo",
        modality="physical",
        has_video=True,
        phases=[("Prep", 2000), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "PHYSICAL EXECUTION WITH VIDEO DEMONSTRATION\n\n"
            "You will see a video demonstration of each gesture.\n\n"
            "1. PREPARE phase (blue): Watch the demonstration video.\n"
            "2. EXECUTE phase (green): Perform the gesture as shown in the "
            "video. Hold the position.\n"
            "3. STOP phase (red): Relax your hand.\n"
            "4. REST phase: Keep still and relax.\n\n"
            "Match the gesture shown in the video as closely as possible.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="Watch the demonstration",
        execute_prefix="Perform:",
    ),
    6: MethodologyDef(
        id=6,
        name="M6: Imagery + Video Demo",
        modality="imagery",
        has_video=True,
        phases=[("Prep", 2000), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "MOTOR IMAGERY WITH VIDEO DEMONSTRATION\n\n"
            "You will see a video demonstration of each gesture.\n\n"
            "1. PREPARE phase (blue): Watch the demonstration video.\n"
            "2. EXECUTE phase (green): IMAGINE performing the gesture as "
            "shown. Do NOT actually move.\n"
            "3. STOP phase (red): Stop imagining.\n"
            "4. REST phase: Relax your mind.\n\n"
            "Watch the video carefully, then imagine the movement vividly.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="Watch the demonstration",
        execute_prefix="Imagine:",
    ),
}


# ---------------------------------------------------------------------------
# Session timing helpers
# ---------------------------------------------------------------------------
SESSION_DURATION_S = 1200    # 20 minutes
BLOCK_DURATION_S = 120       # 2 minutes
BREAK_DURATION_S = 30        # 30 seconds between blocks
DEFAULT_ITI_S = 0.5


def compute_session_defaults(methodology_id: int, iti: float = DEFAULT_ITI_S):
    """Compute trials_per_gesture and block_count for a ~20-min session."""
    m = METHODOLOGIES[methodology_id]
    # Trial cycle = gesture phases + rest phases + 2 ITIs
    gesture_time = sum(d / 1000 for _, d in m.phases)
    rest_time = sum(d / 1000 for _, d in m.rest_phases)
    cycle_time = gesture_time + rest_time + 2 * iti

    # How many trials fit in one 2-min block
    trials_per_block = max(1, int(BLOCK_DURATION_S / cycle_time))

    # How many blocks fit in 20 min (including 30s breaks)
    # N_blocks * block_time + (N_blocks - 1) * break = session_time
    block_active_time = trials_per_block * cycle_time
    block_count = max(1, round(
        (SESSION_DURATION_S + BREAK_DURATION_S)
        / (block_active_time + BREAK_DURATION_S)
    ))

    # Total trials, rounded to nearest multiple of 4
    total_trials = trials_per_block * block_count
    n = len(METHODOLOGIES[methodology_id].gestures)
    total_trials = max(n, (total_trials // n) * n)
    trials_per_gesture = total_trials // n

    return trials_per_gesture, block_count


# ---------------------------------------------------------------------------
# Session configuration
# ---------------------------------------------------------------------------
@dataclass
class SessionConfig:
    methodology_id: int = 1
    participant_id: str = ""
    session_id: str = ""
    hand: str = "Right"
    trials_per_gesture: int = 16
    iti_duration: float = 0.5
    block_count: int = 8
    break_duration: int = 30
    audio_cues: bool = False
    output_dir: str = ""
    latin_square_num: int = 1
    randomization_seed: int = 0
    eeg_host: str = "192.168.137.248"
    eeg_port: int = 8888

    def methodology(self) -> MethodologyDef:
        return METHODOLOGIES[self.methodology_id]


# ---------------------------------------------------------------------------
# Experiment state
# ---------------------------------------------------------------------------
class ExperimentState(Enum):
    IDLE = auto()
    ITI = auto()
    PREP = auto()
    GO = auto()
    EXECUTE = auto()
    END = auto()
    REST = auto()
    BLOCK_BREAK = auto()
    PAUSED = auto()
    FINISHED = auto()


# ---------------------------------------------------------------------------
# Trial sequencer
# ---------------------------------------------------------------------------
class TrialSequencer:
    """Generates one shuffled session with equal gesture counts, split into blocks.

    Gestures are balanced across the whole session (not per-block) and
    randomly dispersed. Blocks are equal-sized time chunks.
    """

    def __init__(self, config: SessionConfig):
        self.config = config
        self.blocks: list[list[dict]] = []
        self._total_active = 0
        self._generate()

    def _generate(self):
        seed_raw = f"{self.config.participant_id}_{self.config.session_id}"
        seed = int(hashlib.sha256(seed_raw.encode()).hexdigest()[:8], 16)
        rng = random.Random(seed)

        # Build full gesture list — equal counts, shuffled across session
        total_per_gesture = self.config.trials_per_gesture
        all_gestures = []
        for g in self.config.methodology().gestures:
            all_gestures.extend([g] * total_per_gesture)
        rng.shuffle(all_gestures)

        self._total_active = len(all_gestures)

        # Interleave rest after each gesture
        all_trials = []
        for g in all_gestures:
            all_trials.append({"gesture": g, "is_rest": False})
            all_trials.append({"gesture": "Rest", "is_rest": True})

        # Split into blocks (pairs stay together: gesture + its rest)
        num_blocks = self.config.block_count
        pairs_total = len(all_gestures)
        pairs_per_block = max(1, pairs_total // num_blocks)

        self.blocks = []
        idx = 0
        for b in range(num_blocks):
            if b == num_blocks - 1:
                # Last block gets remainder
                block = all_trials[idx:]
            else:
                count = pairs_per_block * 2  # 2 entries per pair
                block = all_trials[idx:idx + count]
                idx += count
            if block:
                self.blocks.append(block)

    def total_active_trials(self) -> int:
        return self._total_active

    def total_trials_per_block(self) -> int:
        if not self.blocks:
            return 0
        # Return count of gesture (non-rest) trials in first block
        return sum(1 for t in self.blocks[0] if not t["is_rest"])

    def flat_trial_count(self) -> int:
        return sum(len(b) for b in self.blocks)


# ---------------------------------------------------------------------------
# CSV Logger
# ---------------------------------------------------------------------------
CSV_COLUMNS = [
    "trial_index", "block_index", "gesture_label", "modality",
    "phase", "phase_onset_time", "wall_clock_time", "methodology_id",
]


class CSVLogger:
    """Writes timestamped CSV event log with per-event flush."""

    def __init__(self, config: SessionConfig):
        self.config = config
        self.filepath = ""
        self.json_filepath = ""
        self._file = None
        self._writer = None
        self._row_count = 0

    def open(self):
        os.makedirs(self.config.output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        basename = (
            f"{self.config.participant_id}_{self.config.session_id}"
            f"_M{self.config.methodology_id}_{ts}"
        )
        self.filepath = os.path.join(self.config.output_dir, basename + ".csv")
        self.json_filepath = os.path.join(self.config.output_dir, basename + ".json")

        # Ensure unique filename
        counter = 1
        while os.path.exists(self.filepath):
            self.filepath = os.path.join(
                self.config.output_dir, f"{basename}_{counter}.csv"
            )
            self.json_filepath = os.path.join(
                self.config.output_dir, f"{basename}_{counter}.json"
            )
            counter += 1

        self._file = open(self.filepath, "w", newline="", buffering=1)
        self._writer = csv.writer(self._file)

        # Write metadata comment block
        m = self.config.methodology()
        self._file.write(f"# participant_id={self.config.participant_id}\n")
        self._file.write(f"# session_id={self.config.session_id}\n")
        self._file.write(f"# methodology_id={self.config.methodology_id}\n")
        self._file.write(f"# methodology_name={m.name}\n")
        self._file.write(f"# modality={m.modality}\n")
        self._file.write(f"# hand={self.config.hand}\n")
        self._file.write(f"# trials_per_gesture={self.config.trials_per_gesture}\n")
        self._file.write(f"# iti={self.config.iti_duration}\n")
        self._file.write(f"# blocks={self.config.block_count}\n")
        self._file.write(f"# randomization_seed={self.config.randomization_seed}\n")
        self._file.write(f"# software_version=1.0\n")
        screen = QApplication.primaryScreen()
        if screen:
            sz = screen.size()
            self._file.write(f"# screen_resolution={sz.width()}x{sz.height()}\n")
        self._file.write(f"# start_time={datetime.now().isoformat()}\n")
        self._file.write(f"# perf_counter_t0={time.perf_counter()}\n")

        # Header
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()

    def log_event(self, trial_index: int, block_index: int,
                  gesture_label: str, modality: str, phase: str):
        now_perf = time.perf_counter()
        now_wall = datetime.now().isoformat()
        row = [
            trial_index, block_index, gesture_label, modality,
            phase, f"{now_perf:.9f}", now_wall, self.config.methodology_id,
        ]
        self._writer.writerow(row)
        self._file.flush()
        self._row_count += 1

    def log_special(self, trial_index: int, block_index: int,
                    gesture_label: str, modality: str, phase: str):
        """Log special events like Paused/Resumed."""
        self.log_event(trial_index, block_index, gesture_label, modality, phase)

    def write_metadata_json(self, session_completed: bool, total_trials: int,
                            duration_s: float):
        meta = {
            "paradigm_version": "1.0",
            "participant_id": self.config.participant_id,
            "session_id": self.config.session_id,
            "methodology_id": self.config.methodology_id,
            "modality": self.config.methodology().modality,
            "hand": self.config.hand,
            "trials_per_gesture": self.config.trials_per_gesture,
            "iti_duration": self.config.iti_duration,
            "block_count": self.config.block_count,
            "break_duration": self.config.break_duration,
            "audio_cues": self.config.audio_cues,
            "randomization_seed": self.config.randomization_seed,
            "latin_square_num": self.config.latin_square_num,
            "session_completed": session_completed,
            "total_trials": total_trials,
            "total_csv_rows": self._row_count,
            "duration_s": round(duration_s, 2),
            "data_file": os.path.basename(self.filepath),
            "start_time": datetime.now().isoformat(),
        }
        with open(self.json_filepath, "w") as f:
            json.dump(meta, f, indent=2)

    @property
    def row_count(self) -> int:
        return self._row_count

    def close(self):
        if self._file and not self._file.closed:
            self._file.flush()
            try:
                os.fsync(self._file.fileno())
            except OSError:
                pass
            self._file.close()


# ---------------------------------------------------------------------------
# Audio cues (optional, Windows-only via winsound)
# ---------------------------------------------------------------------------
def play_beep(frequency: int, duration_ms: int):
    """Play a beep in a background thread (non-blocking)."""
    if sys.platform != "win32":
        return
    try:
        import winsound
        t = threading.Thread(
            target=winsound.Beep, args=(frequency, duration_ms), daemon=True
        )
        t.start()
    except Exception:
        pass


def play_go_beep():
    play_beep(800, 100)


def play_stop_beep():
    play_beep(400, 100)


# ---------------------------------------------------------------------------
# EEG Stream Recorder — connects to Pi, writes CSV with perf_counter
# ---------------------------------------------------------------------------
class EEGRecorderThread(threading.Thread):
    """Background thread: connects to EEG TCP stream, writes CSV.

    Each CSV row: timestamp, sample_number, ch1..chN, perf_counter
    Markers live exclusively in the Event CSV; offline sync uses the shared
    perf_counter clock axis.
    """

    def __init__(self, config: SessionConfig):
        super().__init__(name="eeg-recorder", daemon=True)
        self.config = config
        self.running = False
        self._buffer = b""
        self._total_written = 0
        self.filepath = ""
        self.connected = False
        self.error_msg = ""

    @property
    def total_written(self) -> int:
        return self._total_written

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        self.error_msg = ""
        sock = None
        f = None

        try:
            # Connect
            ip = self.config.eeg_host
            if ip.endswith(".local"):
                ip = socket.gethostbyname(ip)

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            print(f"[eeg] Connecting to {ip}:{self.config.eeg_port}...")
            sock.connect((ip, self.config.eeg_port))
            sock.settimeout(1.0)
            self.connected = True
            print("[eeg] Connected")

            # Receive metadata
            self._buffer = b""
            line = self._recv_line(sock)
            metadata = json.loads(line)
            print(f"[eeg] Metadata: num_channels={metadata.get('num_channels')}, "
                  f"sample_rate={metadata.get('sample_rate')}")

            sample_struct = struct.Struct(metadata["sample_struct"])
            sample_size = sample_struct.size
            header_struct = struct.Struct("<II")
            port_configs = metadata.get("port_config", [])

            # Build channel names
            channel_names = []
            for pc in port_configs:
                port_name = pc["name"]
                for d in range(pc["num_devices"]):
                    for c in range(8):
                        channel_names.append(f"{port_name}_dev{d+1}_ch{c+1}")
            num_channels = len(channel_names)

            # Open CSV file
            os.makedirs(self.config.output_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            basename = (
                f"{self.config.participant_id}_{self.config.session_id}"
                f"_M{self.config.methodology_id}_eeg_{ts}"
            )
            self.filepath = os.path.join(self.config.output_dir, basename + ".csv")
            counter = 1
            while os.path.exists(self.filepath):
                self.filepath = os.path.join(
                    self.config.output_dir, f"{basename}_{counter}.csv"
                )
                counter += 1

            f = open(self.filepath, "w", buffering=65536)

            # Write header
            header = (
                "timestamp,sample_number,"
                + ",".join(channel_names)
                + ",perf_counter\n"
            )
            f.write(header)

            print(f"[eeg] Recording {num_channels} channels to {self.filepath}")

            # Main receive loop
            while self.running:
                try:
                    header_data = self._recv_exact(sock, 8)
                    payload_size, sample_count = header_struct.unpack(header_data)
                    raw = self._recv_exact(sock, payload_size)

                    for i in range(sample_count):
                        offset = i * sample_size
                        unpacked = sample_struct.unpack_from(raw, offset)
                        ts_val = unpacked[0]
                        sn = unpacked[1]
                        channels = unpacked[2:]

                        # Build row
                        parts = [f"{ts_val:.6f}", str(sn)]
                        for ch in range(num_channels):
                            parts.append(str(channels[ch]) if ch < len(channels) else "0")
                        parts.append(f"{time.perf_counter():.6f}")

                        f.write(",".join(parts) + "\n")
                        self._total_written += 1

                except socket.timeout:
                    continue

        except Exception as e:
            self.error_msg = f"{type(e).__name__}: {e}"
            print(f"[eeg] Error: {self.error_msg}")
        finally:
            self.connected = False
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            print(f"[eeg] Recorder stopped: {self._total_written} samples written")

    def _recv_exact(self, sock, n):
        while len(self._buffer) < n:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                if not self.running:
                    raise ConnectionError("Stopped")
                continue
            if not chunk:
                raise ConnectionError("Connection closed")
            self._buffer += chunk
        data = self._buffer[:n]
        self._buffer = self._buffer[n:]
        return data

    def _recv_line(self, sock):
        while True:
            pos = self._buffer.find(b"\n")
            if pos != -1:
                line = self._buffer[:pos]
                self._buffer = self._buffer[pos + 1:]
                return line.decode("utf-8")
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Connection closed")
            self._buffer += chunk


# ---------------------------------------------------------------------------
# ConfigScreen
# ---------------------------------------------------------------------------
class ConfigScreen(QWidget):
    start_requested = pyqtSignal(SessionConfig)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        # Scroll area so content is accessible on small screens
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(16, 16, 16, 16)
        scroll_layout.addStretch(1)

        # Card
        card = QFrame()
        card.setMaximumWidth(650)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        card.setStyleSheet(
            f"QFrame {{ background: {SURFACE}; border: 1px solid {BORDER}; "
            f"border-radius: 8px; padding: 24px; }}"
        )
        card_layout = QVBoxLayout(card)
        card_layout.setSpacing(12)

        # Title
        title = QLabel("BCI Gesture Paradigm")
        title.setStyleSheet("font-size: 28px; font-weight: bold; border: none;")
        title.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(title)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {BORDER}; border: none; max-height: 1px; "
                          f"background: {BORDER};")
        card_layout.addWidget(sep)

        # --- Participant section ---
        sec1 = QLabel("Participant")
        sec1.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {ACCENT}; border: none;"
        )
        card_layout.addWidget(sec1)

        form1 = QFormLayout()
        form1.setSpacing(8)
        form1.setLabelAlignment(Qt.AlignRight)

        self.methodology_combo = QComboBox()
        for mid, mdef in METHODOLOGIES.items():
            self.methodology_combo.addItem(mdef.name, mid)
        form1.addRow("Methodology:", self.methodology_combo)

        self.participant_edit = QLineEdit()
        self.participant_edit.setPlaceholderText("e.g. P001")
        form1.addRow("Participant ID:", self.participant_edit)

        self.session_edit = QLineEdit()
        self.session_edit.setText(datetime.now().strftime("S_%Y%m%d_%H%M"))
        form1.addRow("Session ID:", self.session_edit)

        hand_widget = QWidget()
        hand_layout = QHBoxLayout(hand_widget)
        hand_layout.setContentsMargins(0, 0, 0, 0)
        self.hand_group = QButtonGroup(self)
        self.hand_left = QRadioButton("Left")
        self.hand_right = QRadioButton("Right")
        self.hand_right.setChecked(True)
        self.hand_group.addButton(self.hand_left, 0)
        self.hand_group.addButton(self.hand_right, 1)
        hand_layout.addWidget(self.hand_left)
        hand_layout.addWidget(self.hand_right)
        hand_layout.addStretch()
        form1.addRow("Hand:", hand_widget)

        card_layout.addLayout(form1)

        # --- Timing section ---
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet(f"color: {BORDER}; border: none; max-height: 1px; "
                           f"background: {BORDER};")
        card_layout.addWidget(sep2)

        sec2 = QLabel("Timing & Blocks")
        sec2.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {ACCENT}; border: none;"
        )
        card_layout.addWidget(sec2)

        form2 = QFormLayout()
        form2.setSpacing(8)
        form2.setLabelAlignment(Qt.AlignRight)

        self.trials_spin = QSpinBox()
        self.trials_spin.setRange(1, 100)
        self.trials_spin.setValue(16)
        form2.addRow("Trials per Gesture:", self.trials_spin)

        self.iti_spin = QDoubleSpinBox()
        self.iti_spin.setRange(0.0, 5.0)
        self.iti_spin.setSingleStep(0.1)
        self.iti_spin.setDecimals(1)
        self.iti_spin.setValue(0.5)
        self.iti_spin.setSuffix(" s")
        form2.addRow("ITI Duration:", self.iti_spin)

        self.blocks_spin = QSpinBox()
        self.blocks_spin.setRange(1, 20)
        self.blocks_spin.setValue(8)
        form2.addRow("Block Count:", self.blocks_spin)

        self.break_spin = QSpinBox()
        self.break_spin.setRange(5, 300)
        self.break_spin.setValue(30)
        self.break_spin.setSuffix(" s")
        form2.addRow("Break Duration:", self.break_spin)

        # Auto-compute when methodology changes
        self.methodology_combo.currentIndexChanged.connect(self._update_timing_defaults)

        self.audio_check = QCheckBox("Enable GO/STOP beeps")
        form2.addRow("Audio Cues:", self.audio_check)

        card_layout.addLayout(form2)

        # --- EEG Connection section ---
        sep_eeg = QFrame()
        sep_eeg.setFrameShape(QFrame.HLine)
        sep_eeg.setStyleSheet(f"color: {BORDER}; border: none; max-height: 1px; "
                              f"background: {BORDER};")
        card_layout.addWidget(sep_eeg)

        sec_eeg = QLabel("EEG Connection")
        sec_eeg.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {ACCENT}; border: none;"
        )
        card_layout.addWidget(sec_eeg)

        form_eeg = QFormLayout()
        form_eeg.setSpacing(8)
        form_eeg.setLabelAlignment(Qt.AlignRight)

        self.host_edit = QLineEdit()
        self.host_edit.setText("192.168.137.248")
        self.host_edit.setPlaceholderText("Pi IP address")
        form_eeg.addRow("Host:", self.host_edit)

        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(8888)
        form_eeg.addRow("Port:", self.port_spin)

        card_layout.addLayout(form_eeg)

        # --- Output section ---
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.HLine)
        sep3.setStyleSheet(f"color: {BORDER}; border: none; max-height: 1px; "
                           f"background: {BORDER};")
        card_layout.addWidget(sep3)

        sec3 = QLabel("Output")
        sec3.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {ACCENT}; border: none;"
        )
        card_layout.addWidget(sec3)

        form3 = QFormLayout()
        form3.setSpacing(8)
        form3.setLabelAlignment(Qt.AlignRight)

        dir_widget = QWidget()
        dir_layout = QHBoxLayout(dir_widget)
        dir_layout.setContentsMargins(0, 0, 0, 0)
        self.output_edit = QLineEdit()
        default_dir = os.path.join(os.path.dirname(__file__), "experiment_data")
        self.output_edit.setText(os.path.abspath(default_dir))
        browse_btn = QPushButton("Browse")
        browse_btn.setFixedWidth(80)
        browse_btn.clicked.connect(self._browse_output)
        dir_layout.addWidget(self.output_edit, 1)
        dir_layout.addWidget(browse_btn)
        form3.addRow("Output Dir:", dir_widget)

        self.latin_spin = QSpinBox()
        self.latin_spin.setRange(1, 100)
        self.latin_spin.setValue(1)
        self.latin_spin.setToolTip("Latin square participant number (not yet functional)")
        form3.addRow("Latin Square #:", self.latin_spin)

        card_layout.addLayout(form3)

        # Start button
        card_layout.addSpacing(8)
        self.start_btn = QPushButton("Start Session")
        self.start_btn.setFixedHeight(48)
        self.start_btn.setStyleSheet(
            f"QPushButton {{ background-color: {ACCENT_DIM}; color: white; "
            f"font-size: 16px; font-weight: bold; border-radius: 6px; border: none; }}"
            f"QPushButton:hover {{ background-color: #00cc66; }}"
            f"QPushButton:pressed {{ background-color: #008844; }}"
        )
        self.start_btn.clicked.connect(self._on_start)
        card_layout.addWidget(self.start_btn)

        # Center card
        h_layout = QHBoxLayout()
        h_layout.addStretch(1)
        h_layout.addWidget(card)
        h_layout.addStretch(1)
        scroll_layout.addLayout(h_layout)
        scroll_layout.addStretch(1)

        scroll.setWidget(scroll_content)
        outer.addWidget(scroll)

    def _update_timing_defaults(self):
        mid = self.methodology_combo.currentData()
        if mid is not None:
            tpg, bc = compute_session_defaults(mid, self.iti_spin.value())
            self.trials_spin.setValue(tpg)
            self.blocks_spin.setValue(bc)

    def _browse_output(self):
        d = QFileDialog.getExistingDirectory(self, "Select Output Directory",
                                             self.output_edit.text())
        if d:
            self.output_edit.setText(d)

    def _on_start(self):
        pid = self.participant_edit.text().strip()
        if not pid:
            QMessageBox.warning(self, "Validation Error",
                                "Participant ID cannot be empty.")
            return
        sid = self.session_edit.text().strip()
        if not sid:
            QMessageBox.warning(self, "Validation Error",
                                "Session ID cannot be empty.")
            return
        out_dir = self.output_edit.text().strip()
        if not out_dir:
            QMessageBox.warning(self, "Validation Error",
                                "Output directory cannot be empty.")
            return

        mid = self.methodology_combo.currentData()
        seed_raw = f"{pid}_{sid}_{mid}"
        seed = int(hashlib.sha256(seed_raw.encode()).hexdigest()[:8], 16)

        config = SessionConfig(
            methodology_id=mid,
            participant_id=pid,
            session_id=sid,
            hand="Left" if self.hand_left.isChecked() else "Right",
            trials_per_gesture=self.trials_spin.value(),
            iti_duration=self.iti_spin.value(),
            block_count=self.blocks_spin.value(),
            break_duration=self.break_spin.value(),
            audio_cues=self.audio_check.isChecked(),
            output_dir=out_dir,
            latin_square_num=self.latin_spin.value(),
            randomization_seed=seed,
            eeg_host=self.host_edit.text().strip(),
            eeg_port=self.port_spin.value(),
        )
        self.start_requested.emit(config)


# ---------------------------------------------------------------------------
# InstructionScreen
# ---------------------------------------------------------------------------
class InstructionScreen(QWidget):
    begin_practice = pyqtSignal()
    skip_practice = pyqtSignal()
    back_to_config = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(16, 16, 16, 16)
        scroll_layout.addStretch(1)

        card = QFrame()
        card.setMaximumWidth(800)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        card.setStyleSheet(
            f"QFrame {{ background: {SURFACE}; border: 1px solid {BORDER}; "
            f"border-radius: 8px; padding: 32px; }}"
        )
        card_layout = QVBoxLayout(card)
        card_layout.setSpacing(16)

        self.title_label = QLabel("Instructions")
        self.title_label.setStyleSheet(
            f"font-size: 28px; font-weight: bold; color: {ACCENT}; border: none;"
        )
        self.title_label.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(self.title_label)

        self.info_label = QLabel()
        self.info_label.setStyleSheet(
            f"font-size: 14px; color: {TEXT_SECONDARY}; border: none;"
        )
        self.info_label.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(self.info_label)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {BORDER}; border: none; max-height: 1px; "
                          f"background: {BORDER};")
        card_layout.addWidget(sep)

        self.instruction_label = QLabel()
        self.instruction_label.setWordWrap(True)
        self.instruction_label.setStyleSheet(
            f"font-size: 15px; line-height: 1.6; border: none;"
        )
        self.instruction_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        card_layout.addWidget(self.instruction_label)

        card_layout.addSpacing(16)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        back_btn = QPushButton("Back to Config")
        back_btn.clicked.connect(self.back_to_config.emit)
        btn_layout.addWidget(back_btn)

        btn_layout.addStretch()

        skip_btn = QPushButton("Skip Practice & Start")
        skip_btn.clicked.connect(self.skip_practice.emit)
        btn_layout.addWidget(skip_btn)

        practice_btn = QPushButton("Begin Practice")
        practice_btn.setStyleSheet(
            f"QPushButton {{ background-color: {ACCENT_DIM}; color: white; "
            f"font-weight: bold; border: none; border-radius: 6px; }}"
            f"QPushButton:hover {{ background-color: #00cc66; }}"
        )
        practice_btn.clicked.connect(self.begin_practice.emit)
        btn_layout.addWidget(practice_btn)

        card_layout.addLayout(btn_layout)

        h = QHBoxLayout()
        h.addStretch(1)
        h.addWidget(card)
        h.addStretch(1)
        scroll_layout.addLayout(h)
        scroll_layout.addStretch(1)

        scroll.setWidget(scroll_content)
        outer.addWidget(scroll)

    def set_config(self, config: SessionConfig):
        m = config.methodology()
        self.title_label.setText(m.name)
        total_active = (
            len(m.gestures) * config.trials_per_gesture * config.block_count
        )
        self.info_label.setText(
            f"{total_active} gesture trials across {config.block_count} block(s)  "
            f"|  Hand: {config.hand}  |  Modality: {m.modality}"
        )
        self.instruction_label.setText(m.instruction_text)


# ---------------------------------------------------------------------------
# ExperimentScreen
# ---------------------------------------------------------------------------
class ExperimentScreen(QWidget):
    """Main trial display — shows cue text, progress, phase colors."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._phase_color = QColor(DARK_BG)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Top progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFixedHeight(4)
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)

        # Header bar
        header = QWidget()
        header.setFixedHeight(32)
        header.setStyleSheet(f"background: #111111;")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(16, 0, 16, 0)

        self.block_label = QLabel("Block 1 / 1")
        self.block_label.setStyleSheet(
            f"font-size: 11px; color: {TEXT_MUTED}; background: transparent;"
        )
        header_layout.addWidget(self.block_label)

        self.practice_banner = QLabel("PRACTICE MODE")
        self.practice_banner.setStyleSheet(
            "font-size: 12px; font-weight: bold; color: #ffaa00; "
            "background: transparent;"
        )
        self.practice_banner.setAlignment(Qt.AlignCenter)
        self.practice_banner.hide()
        header_layout.addWidget(self.practice_banner, 1)

        self.trial_label = QLabel("Trial 0 / 0")
        self.trial_label.setStyleSheet(
            f"font-size: 11px; color: {TEXT_MUTED}; background: transparent;"
        )
        self.trial_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        header_layout.addWidget(self.trial_label)

        layout.addWidget(header)

        # Cue area (central)
        self.cue_area = QWidget()
        self.cue_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        cue_layout = QVBoxLayout(self.cue_area)
        cue_layout.setAlignment(Qt.AlignCenter)
        cue_layout.setSpacing(0)

        self.phase_indicator = QLabel("")
        self.phase_indicator.setFixedHeight(24)
        self.phase_indicator.setStyleSheet(
            f"font-size: 12px; color: {TEXT_MUTED}; background: transparent;"
        )
        self.phase_indicator.setAlignment(Qt.AlignCenter)
        cue_layout.addWidget(self.phase_indicator)

        # Fixed-height slot for GO label — always occupies space
        self.go_stop_label = QLabel("")
        self.go_stop_label.setFixedHeight(120)
        self.go_stop_label.setStyleSheet(
            "font-size: 96px; font-weight: 900; color: transparent; "
            "background: transparent;"
        )
        self.go_stop_label.setAlignment(Qt.AlignCenter)
        cue_layout.addWidget(self.go_stop_label)

        # Fixed-height slot for gesture cue — always occupies space
        self.cue_label = QLabel("")
        self.cue_label.setFixedHeight(100)
        self.cue_label.setStyleSheet(
            "font-size: 72px; font-weight: bold; color: white; "
            "background: transparent;"
        )
        self.cue_label.setAlignment(Qt.AlignCenter)
        cue_layout.addWidget(self.cue_label)

        self.subtext_label = QLabel("")
        self.subtext_label.setFixedHeight(36)
        self.subtext_label.setStyleSheet(
            f"font-size: 22px; color: #aaaaaa; background: transparent;"
        )
        self.subtext_label.setAlignment(Qt.AlignCenter)
        cue_layout.addWidget(self.subtext_label)

        # Video label (OpenCV frames rendered here)
        self.video_label = QLabel("")
        self.video_label.setFixedSize(960, 720)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background: black; border-radius: 8px;")
        self.video_label.hide()
        cue_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)

        self._video_cap = None
        self._video_loop = True
        self._video_timer = QTimer(self)
        self._video_timer.timeout.connect(self._next_video_frame)

        layout.addWidget(self.cue_area, 1)

        # Footer
        footer = QWidget()
        footer.setFixedHeight(24)
        footer.setStyleSheet("background: #111111;")
        footer_layout = QHBoxLayout(footer)
        footer_layout.setContentsMargins(16, 0, 16, 0)

        self.phase_time_label = QLabel("")
        self.phase_time_label.setStyleSheet(
            f"font-size: 10px; color: {TEXT_MUTED}; background: transparent;"
        )
        footer_layout.addWidget(self.phase_time_label)
        footer_layout.addStretch()

        layout.addWidget(footer)

    def set_phase(self, phase: str, gesture: str = "", subtext: str = "",
                  show_video: bool = False, go_stop_text: str = "",
                  go_stop_color: str = "", video_duration_ms: int = 0):
        color = PHASE_COLORS.get(phase, DARK_BG)
        self.cue_area.setStyleSheet(f"background: {color};")

        self.phase_indicator.setText(f"[ {phase.upper()} ]")
        self.cue_label.setText(gesture.upper() if gesture else "")
        self.cue_label.setStyleSheet(
            "font-size: 72px; font-weight: bold; color: white; "
            "background: transparent;"
        )
        self.subtext_label.setText(subtext)

        # GO label — always keeps its fixed height, just make text
        # visible or transparent so nothing shifts
        if go_stop_text:
            self.go_stop_label.setText(go_stop_text)
            font_size = 96 if len(go_stop_text) <= 4 else 56
            self.go_stop_label.setStyleSheet(
                f"font-size: {font_size}px; font-weight: 900; color: {go_stop_color}; "
                f"background: transparent;"
            )
        else:
            self.go_stop_label.setText("")
            self.go_stop_label.setStyleSheet(
                "font-size: 96px; font-weight: 900; color: transparent; "
                "background: transparent;"
            )

        if show_video and gesture:
            video_path = GESTURE_VIDEOS.get(gesture)
            if video_path and os.path.isfile(video_path):
                self._start_video(video_path, duration_ms=video_duration_ms or None)
            else:
                self._stop_video()
                self.video_label.setStyleSheet(
                    f"font-size: 36px; font-weight: bold; color: white; "
                    f"background: {GESTURE_COLORS.get(gesture, '#3a3a3a')}; border-radius: 8px;"
                )
                self.video_label.setText(f"[Video]\n{gesture}")
                self.video_label.show()
        else:
            self._stop_video()

    def _start_video(self, path, duration_ms=None):
        self._stop_video()
        self._video_cap = cv2.VideoCapture(path)
        native_fps = self._video_cap.get(cv2.CAP_PROP_FPS) or 30
        frame_count = int(self._video_cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

        if duration_ms and frame_count > 0:
            # Stretch playback so the clip fills duration_ms in exactly one
            # pass: slow down (or speed up) by adjusting the per-frame interval.
            # Disable looping — freeze on the last frame if we hit EOF early.
            interval_ms = max(1, int(round(duration_ms / frame_count)))
            self._video_loop = False
        else:
            interval_ms = max(1, int(round(1000 / native_fps)))
            self._video_loop = True

        self.video_label.setText("")
        self.video_label.setStyleSheet("background: black; border-radius: 8px;")
        self.video_label.show()
        self._video_timer.start(interval_ms)

    def _stop_video(self):
        self._video_timer.stop()
        if self._video_cap:
            self._video_cap.release()
            self._video_cap = None
        self.video_label.hide()

    def _next_video_frame(self):
        if not self._video_cap:
            return
        ret, frame = self._video_cap.read()
        if not ret:
            if self._video_loop:
                self._video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self._video_cap.read()
                if not ret:
                    return
            else:
                # Stretched playback: hit EOF early — freeze on the last
                # rendered frame by stopping the timer (label keeps its
                # current pixmap until _stop_video is called).
                self._video_timer.stop()
                return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(pixmap)

    def set_iti(self):
        self._stop_video()
        self.cue_area.setStyleSheet(f"background: {DARK_BG};")
        self.phase_indicator.setText("")
        self.cue_label.setText("+")
        self.cue_label.setStyleSheet(
            f"font-size: 48px; color: {TEXT_MUTED}; background: transparent;"
        )
        self.subtext_label.setText("")
        self.go_stop_label.setText("")
        self.go_stop_label.setStyleSheet(
            "font-size: 96px; font-weight: 900; color: transparent; "
            "background: transparent;"
        )

    def set_progress(self, current: int, total: int):
        self.progress_bar.setMaximum(total)
        self.progress_bar.setValue(current)

    def set_trial_info(self, trial: int, total_trials: int,
                       block: int, total_blocks: int):
        self.trial_label.setText(f"Trial {trial} / {total_trials}")
        self.block_label.setText(f"Block {block} / {total_blocks}")

    def set_practice(self, is_practice: bool):
        self.practice_banner.setVisible(is_practice)


# ---------------------------------------------------------------------------
# BreakScreen
# ---------------------------------------------------------------------------
class BreakScreen(QWidget):
    resume_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._remaining = 0
        self._min_break = 10  # minimum seconds before resume enabled
        self._elapsed = 0
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._tick)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(16)

        self.title_label = QLabel("Block Complete")
        self.title_label.setStyleSheet(
            f"font-size: 36px; font-weight: bold; color: {ACCENT};"
        )
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)

        self.info_label = QLabel("Take a break and relax")
        self.info_label.setStyleSheet(
            f"font-size: 18px; color: {TEXT_SECONDARY};"
        )
        self.info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.info_label)

        layout.addSpacing(24)

        self.countdown_label = QLabel("60")
        self.countdown_label.setStyleSheet(
            "font-size: 120px; font-weight: bold; color: white;"
        )
        self.countdown_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.countdown_label)

        self.seconds_label = QLabel("seconds remaining")
        self.seconds_label.setStyleSheet(
            f"font-size: 14px; color: {TEXT_MUTED};"
        )
        self.seconds_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.seconds_label)

        layout.addSpacing(24)

        self.resume_btn = QPushButton("Resume Early")
        self.resume_btn.setFixedSize(200, 48)
        self.resume_btn.setStyleSheet(
            f"QPushButton {{ background-color: {SURFACE_RAISED}; color: {TEXT_SECONDARY}; "
            f"font-size: 16px; font-weight: bold; border-radius: 6px; border: none; }}"
            f"QPushButton:hover {{ background-color: #3a3a3a; }}"
            f"QPushButton:disabled {{ color: {TEXT_MUTED}; background: #1a1a1a; }}"
        )
        self.resume_btn.setEnabled(False)
        self.resume_btn.clicked.connect(self._on_resume)
        layout.addWidget(self.resume_btn, alignment=Qt.AlignCenter)

    def start_break(self, duration: int, block_num: int, total_blocks: int):
        self._remaining = duration
        self._elapsed = 0
        self.title_label.setText(f"Block {block_num} / {total_blocks} Complete")
        self.countdown_label.setText(str(duration))
        self.resume_btn.setEnabled(False)
        self._timer.start()

    def _tick(self):
        self._elapsed += 1
        self._remaining -= 1
        self.countdown_label.setText(str(max(0, self._remaining)))

        if self._elapsed >= self._min_break:
            self.resume_btn.setEnabled(True)

        if self._remaining <= 0:
            self._timer.stop()
            self._on_resume()

    def _on_resume(self):
        self._timer.stop()
        self.resume_requested.emit()


# ---------------------------------------------------------------------------
# PauseScreen (overlay)
# ---------------------------------------------------------------------------
class PauseScreen(QWidget):
    resume_requested = pyqtSignal()
    end_session_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background: rgba(0, 0, 0, 200);")
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(16)

        paused = QLabel("PAUSED")
        paused.setStyleSheet(
            "font-size: 64px; font-weight: bold; color: #ffaa00; "
            "background: transparent;"
        )
        paused.setAlignment(Qt.AlignCenter)
        layout.addWidget(paused)

        self.info_label = QLabel("Press Space or click Resume to continue")
        self.info_label.setStyleSheet(
            f"font-size: 16px; color: {TEXT_SECONDARY}; background: transparent;"
        )
        self.info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.info_label)

        self.position_label = QLabel("")
        self.position_label.setStyleSheet(
            f"font-size: 14px; color: {TEXT_MUTED}; background: transparent;"
        )
        self.position_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.position_label)

        layout.addSpacing(24)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(16)
        btn_layout.setAlignment(Qt.AlignCenter)

        end_btn = QPushButton("End Session")
        end_btn.setFixedSize(160, 44)
        end_btn.setStyleSheet(
            f"QPushButton {{ background-color: #5a1a1a; color: white; "
            f"font-weight: bold; border: none; border-radius: 6px; }}"
            f"QPushButton:hover {{ background-color: #7a2a2a; }}"
        )
        end_btn.clicked.connect(self.end_session_requested.emit)
        btn_layout.addWidget(end_btn)

        resume_btn = QPushButton("Resume")
        resume_btn.setFixedSize(160, 44)
        resume_btn.setStyleSheet(
            f"QPushButton {{ background-color: {ACCENT_DIM}; color: white; "
            f"font-weight: bold; border: none; border-radius: 6px; }}"
            f"QPushButton:hover {{ background-color: #00cc66; }}"
        )
        resume_btn.clicked.connect(self.resume_requested.emit)
        btn_layout.addWidget(resume_btn)

        layout.addLayout(btn_layout)

    def set_position(self, trial: int, total: int, block: int, total_blocks: int):
        self.position_label.setText(
            f"Trial {trial} / {total}  |  Block {block} / {total_blocks}"
        )


# ---------------------------------------------------------------------------
# SummaryScreen
# ---------------------------------------------------------------------------
class SummaryScreen(QWidget):
    new_session = pyqtSignal()
    exit_app = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(16, 16, 16, 16)
        scroll_layout.addStretch(1)

        card = QFrame()
        card.setMaximumWidth(600)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        card.setStyleSheet(
            f"QFrame {{ background: {SURFACE}; border: 1px solid {BORDER}; "
            f"border-radius: 8px; padding: 32px; }}"
        )
        card_layout = QVBoxLayout(card)
        card_layout.setSpacing(16)

        title = QLabel("SESSION COMPLETE")
        title.setStyleSheet(
            f"font-size: 28px; font-weight: bold; color: {ACCENT}; border: none;"
        )
        title.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(title)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {BORDER}; border: none; max-height: 1px; "
                          f"background: {BORDER};")
        card_layout.addWidget(sep)

        self.stats_label = QLabel("")
        self.stats_label.setWordWrap(True)
        self.stats_label.setStyleSheet(
            f"font-size: 15px; color: {TEXT_PRIMARY}; border: none;"
        )
        card_layout.addWidget(self.stats_label)

        file_label = QLabel("Data saved to:")
        file_label.setStyleSheet(
            f"font-size: 13px; color: {TEXT_SECONDARY}; border: none;"
        )
        card_layout.addWidget(file_label)

        self.file_edit = QLineEdit()
        self.file_edit.setReadOnly(True)
        self.file_edit.setStyleSheet(
            f"font-size: 12px; background: {DARK_BG}; color: {ACCENT}; "
            f"border: 1px solid {BORDER}; padding: 6px;"
        )
        card_layout.addWidget(self.file_edit)

        card_layout.addSpacing(16)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        new_btn = QPushButton("New Session")
        new_btn.setFixedHeight(44)
        new_btn.setStyleSheet(
            f"QPushButton {{ background-color: {ACCENT_DIM}; color: white; "
            f"font-weight: bold; border: none; border-radius: 6px; }}"
            f"QPushButton:hover {{ background-color: #00cc66; }}"
        )
        new_btn.clicked.connect(self.new_session.emit)
        btn_layout.addWidget(new_btn)

        exit_btn = QPushButton("Exit")
        exit_btn.setFixedHeight(44)
        exit_btn.clicked.connect(self.exit_app.emit)
        btn_layout.addWidget(exit_btn)

        card_layout.addLayout(btn_layout)

        h = QHBoxLayout()
        h.addStretch(1)
        h.addWidget(card)
        h.addStretch(1)
        scroll_layout.addLayout(h)
        scroll_layout.addStretch(1)

        scroll.setWidget(scroll_content)
        outer.addWidget(scroll)

    def set_results(self, trials_completed: int, total_trials: int,
                    duration_s: float, csv_path: str, interrupted: int,
                    eeg_csv_path: str = "", eeg_samples: int = 0):
        mins = int(duration_s) // 60
        secs = int(duration_s) % 60
        lines = [
            f"Trials completed: {trials_completed} / {total_trials}",
            f"Session duration: {mins}m {secs}s",
        ]
        if eeg_samples > 0:
            lines.append(f"EEG samples recorded: {eeg_samples:,}")
        if interrupted > 0:
            lines.append(f"Interrupted trials: {interrupted}")
        self.stats_label.setText("\n".join(lines))

        paths = []
        if csv_path:
            paths.append(f"Events: {csv_path}")
        if eeg_csv_path:
            paths.append(f"EEG: {eeg_csv_path}")
        self.file_edit.setText("\n".join(paths))


# ---------------------------------------------------------------------------
# ExperimentEngine — trial sequencing + timing state machine
# ---------------------------------------------------------------------------
class ExperimentEngine:
    """Drives the experiment through phases using QTimer.singleShot."""

    def __init__(self, app: "ExperimentApp"):
        self.app = app
        self.config: Optional[SessionConfig] = None
        self.sequencer: Optional[TrialSequencer] = None
        self.logger: Optional[CSVLogger] = None
        self.eeg_recorder: Optional[EEGRecorderThread] = None
        self.methodology: Optional[MethodologyDef] = None

        self.state = ExperimentState.IDLE
        self.is_practice = False

        # Indices
        self.block_index = 0        # 0-based
        self.trial_in_block = 0     # 0-based index into block trial list
        self.active_trial_count = 0 # 1-based counter of gesture (non-rest) trials
        self.global_trial_index = 0 # 1-based for CSV

        # Timing
        self.phase_start_time = 0.0
        self.session_start_time = 0.0
        self.interrupted_count = 0

        # Phase sequence for current trial
        self._current_phases: list = []
        self._phase_index = 0

        # Pause state
        self._paused_remaining_ms = 0
        self._paused_phase = ""
        self._active_timer: Optional[int] = None

    def start_session(self, config: SessionConfig, practice: bool = False):
        self.config = config
        self.methodology = config.methodology()
        self.is_practice = practice

        if practice:
            # Practice: 1 trial per gesture, 1 block
            practice_config = SessionConfig(
                methodology_id=config.methodology_id,
                participant_id=config.participant_id,
                session_id=config.session_id,
                hand=config.hand,
                trials_per_gesture=1,
                iti_duration=config.iti_duration,
                block_count=1,
                break_duration=config.break_duration,
                audio_cues=config.audio_cues,
                output_dir=config.output_dir,
                latin_square_num=config.latin_square_num,
                randomization_seed=config.randomization_seed,
            )
            self.sequencer = TrialSequencer(practice_config)
        else:
            self.sequencer = TrialSequencer(config)
            self.logger = CSVLogger(config)
            self.logger.open()
            # Start EEG recorder
            self.eeg_recorder = EEGRecorderThread(config)
            self.eeg_recorder.start()

        self.block_index = 0
        self.trial_in_block = 0
        self.active_trial_count = 0
        self.global_trial_index = 0
        self.interrupted_count = 0
        self.session_start_time = time.perf_counter()
        self.state = ExperimentState.IDLE

        self._start_block()

    def _start_block(self):
        self.trial_in_block = 0
        self._advance_trial()

    def _advance_trial(self):
        block = self.sequencer.blocks[self.block_index]

        if self.trial_in_block >= len(block):
            # Block complete
            self.block_index += 1
            if self.block_index >= len(self.sequencer.blocks):
                self._finish()
                return
            # Show break
            self.state = ExperimentState.BLOCK_BREAK
            self.app.show_break(
                self.config.break_duration,
                self.block_index,  # just completed (1-based display)
                len(self.sequencer.blocks),
            )
            return

        trial = block[self.trial_in_block]
        is_rest = trial["is_rest"]
        gesture = trial["gesture"]

        if not is_rest:
            self.active_trial_count += 1
            self.global_trial_index += 1

        # Determine phase sequence
        if is_rest:
            self._current_phases = list(self.methodology.rest_phases)
        else:
            self._current_phases = list(self.methodology.phases)

        self._phase_index = 0

        # Update UI counters
        if self.is_practice:
            total_active = len(self.methodology.gestures)
        else:
            total_active = self.sequencer.total_trials_per_block()
        self.app.experiment_screen.set_trial_info(
            self.active_trial_count,
            total_active * (1 if self.is_practice else len(self.sequencer.blocks)),
            self.block_index + 1,
            len(self.sequencer.blocks),
        )
        self.app.experiment_screen.set_progress(
            self.active_trial_count,
            total_active * (1 if self.is_practice else len(self.sequencer.blocks)),
        )

        # Start ITI
        self._start_iti()

    def _start_iti(self):
        iti_ms = int(self.config.iti_duration * 1000)
        if iti_ms <= 0:
            self._start_phase()
            return

        self.state = ExperimentState.ITI
        self.app.experiment_screen.set_iti()
        self.phase_start_time = time.perf_counter()

        self._active_timer = QTimer.singleShot(iti_ms, self._start_phase)

    def _start_phase(self):
        if self._phase_index >= len(self._current_phases):
            # Trial complete, advance
            self.trial_in_block += 1
            self._advance_trial()
            return

        phase_name, duration_ms = self._current_phases[self._phase_index]
        block = self.sequencer.blocks[self.block_index]
        trial = block[self.trial_in_block]
        gesture = trial["gesture"]
        is_rest = trial["is_rest"]

        self.state = ExperimentState[phase_name.upper()]
        self.phase_start_time = time.perf_counter()

        # Log to CSV
        if self.logger and not self.is_practice:
            ti = self.global_trial_index
            self.logger.log_event(
                trial_index=ti,
                block_index=self.block_index + 1,
                gesture_label=gesture,
                modality=self.methodology.modality,
                phase=phase_name,
            )

        # Update display
        go_stop_text = ""
        go_stop_color = ""
        subtext = ""

        if phase_name == "Prep":
            subtext = self.methodology.prep_subtext
        elif phase_name == "Go":
            go_stop_text = "GO"
            go_stop_color = "#00ff44"
            # Go phase: only "GO" signal, no gesture name
            gesture_display = ""
            subtext = ""
            if self.config.audio_cues:
                play_go_beep()
        elif phase_name == "Execute":
            subtext = f"{self.methodology.execute_prefix} {gesture}"
        elif phase_name == "End":
            # End phase: red background with STOP
            go_stop_text = "STOP"
            go_stop_color = "#ff3333"
            gesture_display = ""
            subtext = ""
            if self.config.audio_cues:
                play_stop_beep()
        elif phase_name == "Rest":
            subtext = "Relax"

        # Determine what gesture text to show
        if phase_name in ("Go", "End", "Rest"):
            gesture_display = ""
        else:
            gesture_display = gesture

        show_video = self.methodology.has_video and phase_name in ("Prep", "Execute")

        # Stretch the demo video to fill the Execute window in a single pass
        # (no looping). Prep keeps native playback speed.
        stretched_duration_ms = duration_ms if (show_video and phase_name == "Execute") else 0

        self.app.experiment_screen.set_phase(
            phase=phase_name,
            gesture=gesture_display,
            subtext=subtext,
            show_video=show_video,
            go_stop_text=go_stop_text,
            go_stop_color=go_stop_color,
            video_duration_ms=stretched_duration_ms,
        )

        self._phase_index += 1
        self._active_timer = QTimer.singleShot(duration_ms, self._start_phase)

    def pause(self):
        if self.state in (ExperimentState.PAUSED, ExperimentState.IDLE,
                          ExperimentState.FINISHED, ExperimentState.BLOCK_BREAK):
            return

        elapsed = time.perf_counter() - self.phase_start_time
        if self._phase_index > 0 and self._phase_index <= len(self._current_phases):
            _, duration_ms = self._current_phases[self._phase_index - 1]
            remaining = duration_ms - int(elapsed * 1000)
            self._paused_remaining_ms = max(0, remaining)
        else:
            self._paused_remaining_ms = 0

        self._paused_phase = self.state.name
        self.state = ExperimentState.PAUSED

        # Log pause event
        if self.logger and not self.is_practice:
            block = self.sequencer.blocks[self.block_index]
            trial = block[self.trial_in_block]
            self.logger.log_special(
                trial_index=self.global_trial_index,
                block_index=self.block_index + 1,
                gesture_label=trial["gesture"],
                modality=self.methodology.modality,
                phase="Paused",
            )
            self.interrupted_count += 1

        self.app.show_pause()

    def resume(self):
        if self.state != ExperimentState.PAUSED:
            return

        # Log resume
        if self.logger and not self.is_practice:
            block = self.sequencer.blocks[self.block_index]
            trial = block[self.trial_in_block]
            self.logger.log_special(
                trial_index=self.global_trial_index,
                block_index=self.block_index + 1,
                gesture_label=trial["gesture"],
                modality=self.methodology.modality,
                phase="Resumed",
            )

        self.state = ExperimentState[self._paused_phase]
        self.phase_start_time = time.perf_counter()

        self.app.hide_pause()

        if self._paused_remaining_ms > 0:
            self._active_timer = QTimer.singleShot(
                self._paused_remaining_ms, self._start_phase
            )
        else:
            self._start_phase()

    def resume_from_break(self):
        self.trial_in_block = 0
        self._start_block()

    def _stop_eeg_recorder(self):
        if self.eeg_recorder:
            self.eeg_recorder.stop()
            self.eeg_recorder.join(timeout=3.0)
            self.eeg_recorder = None

    def _finish(self):
        self.state = ExperimentState.FINISHED
        duration = time.perf_counter() - self.session_start_time

        if self.is_practice:
            self.app.on_practice_complete()
        else:
            eeg_path = self.eeg_recorder.filepath if self.eeg_recorder else ""
            eeg_samples = self.eeg_recorder.total_written if self.eeg_recorder else 0
            self._stop_eeg_recorder()
            if self.logger:
                self.logger.write_metadata_json(
                    session_completed=True,
                    total_trials=self.active_trial_count,
                    duration_s=duration,
                )
                self.logger.close()
            self.app.show_summary(
                trials_completed=self.active_trial_count,
                total_trials=self.sequencer.total_active_trials(),
                duration_s=duration,
                csv_path=self.logger.filepath if self.logger else "",
                interrupted=self.interrupted_count,
                eeg_csv_path=eeg_path,
                eeg_samples=eeg_samples,
            )

    def end_early(self):
        """End session from pause screen."""
        duration = time.perf_counter() - self.session_start_time
        self.state = ExperimentState.FINISHED

        eeg_path = self.eeg_recorder.filepath if self.eeg_recorder else ""
        eeg_samples = self.eeg_recorder.total_written if self.eeg_recorder else 0

        if not self.is_practice:
            self._stop_eeg_recorder()
            if self.logger:
                self.logger.write_metadata_json(
                    session_completed=False,
                    total_trials=self.active_trial_count,
                    duration_s=duration,
                )
                self.logger.close()

        if self.is_practice:
            self.app.on_practice_complete()
        else:
            total = (self.sequencer.total_active_trials()
                     if self.sequencer else self.active_trial_count)
            self.app.show_summary(
                trials_completed=self.active_trial_count,
                total_trials=total,
                duration_s=duration,
                csv_path=self.logger.filepath if self.logger else "",
                interrupted=self.interrupted_count,
                eeg_csv_path=eeg_path,
                eeg_samples=eeg_samples,
            )


# ---------------------------------------------------------------------------
# ReadyScreen — "Press Spacebar when ready"
# ---------------------------------------------------------------------------
class ReadyScreen(QWidget):
    start_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(24)

        title = QLabel("Ready to Begin")
        title.setStyleSheet(
            f"font-size: 48px; font-weight: bold; color: {ACCENT};"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        prompt = QLabel("Press Spacebar or click Start when ready")
        prompt.setStyleSheet(
            f"font-size: 24px; color: {TEXT_SECONDARY};"
        )
        prompt.setAlignment(Qt.AlignCenter)
        layout.addWidget(prompt)

        self.start_btn = QPushButton("Start")
        self.start_btn.setFixedSize(200, 56)
        self.start_btn.setStyleSheet(
            f"QPushButton {{ background-color: {ACCENT_DIM}; color: white; "
            f"font-size: 20px; font-weight: bold; border-radius: 8px; border: none; }}"
            f"QPushButton:hover {{ background-color: #00cc66; }}"
            f"QPushButton:pressed {{ background-color: #008844; }}"
        )
        self.start_btn.clicked.connect(self.start_requested.emit)
        layout.addWidget(self.start_btn, alignment=Qt.AlignCenter)


# ---------------------------------------------------------------------------
# Main Application Window
# ---------------------------------------------------------------------------
PAGE_CONFIG = 0
PAGE_INSTRUCTION = 1
PAGE_READY = 2
PAGE_EXPERIMENT = 3
PAGE_BREAK = 4
PAGE_SUMMARY = 5


class ExperimentApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BCI Gesture Paradigm")
        self.resize(900, 700)

        self.config: Optional[SessionConfig] = None
        self.engine = ExperimentEngine(self)

        # Central stacked widget
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # Screens
        self.config_screen = ConfigScreen()
        self.instruction_screen = InstructionScreen()
        self.ready_screen = ReadyScreen()
        self.experiment_screen = ExperimentScreen()
        self.break_screen = BreakScreen()
        self.summary_screen = SummaryScreen()

        self.stack.addWidget(self.config_screen)      # 0
        self.stack.addWidget(self.instruction_screen)  # 1
        self.stack.addWidget(self.ready_screen)         # 2
        self.stack.addWidget(self.experiment_screen)    # 3
        self.stack.addWidget(self.break_screen)         # 4
        self.stack.addWidget(self.summary_screen)       # 5

        # Pause overlay (sits on top of experiment screen)
        self.pause_screen = PauseScreen(self.experiment_screen)
        self.pause_screen.hide()

        # Connect signals
        self.config_screen.start_requested.connect(self._on_config_done)
        self.instruction_screen.begin_practice.connect(self._on_begin_practice)
        self.instruction_screen.skip_practice.connect(self._on_skip_practice)
        self.instruction_screen.back_to_config.connect(self._on_back_to_config)
        self.break_screen.resume_requested.connect(self._on_break_resume)
        self.pause_screen.resume_requested.connect(self._on_pause_resume)
        self.pause_screen.end_session_requested.connect(self._on_pause_end)
        self.ready_screen.start_requested.connect(self._start_real_experiment)
        self.summary_screen.new_session.connect(self._on_new_session)
        self.summary_screen.exit_app.connect(self.close)

    def _on_config_done(self, config: SessionConfig):
        self.config = config
        self.instruction_screen.set_config(config)
        self.stack.setCurrentIndex(PAGE_INSTRUCTION)

    def _on_begin_practice(self):
        self.stack.setCurrentIndex(PAGE_EXPERIMENT)
        self.experiment_screen.set_practice(True)
        self.showFullScreen()
        self.engine.start_session(self.config, practice=True)

    def _on_skip_practice(self):
        self._show_ready_screen()

    def _show_ready_screen(self):
        self.showFullScreen()
        self.stack.setCurrentIndex(PAGE_READY)

    def _start_real_experiment(self):
        self.stack.setCurrentIndex(PAGE_EXPERIMENT)
        self.experiment_screen.set_practice(False)
        self.engine.start_session(self.config, practice=False)

    def _on_back_to_config(self):
        self.stack.setCurrentIndex(PAGE_CONFIG)

    def on_practice_complete(self):
        self.showNormal()
        self.resize(900, 700)
        msg = QMessageBox(self)
        msg.setWindowTitle("Practice Complete")
        msg.setText("Practice session is complete.")
        msg.setInformativeText("What would you like to do?")
        begin_btn = msg.addButton("Begin Experiment", QMessageBox.AcceptRole)
        repeat_btn = msg.addButton("Repeat Practice", QMessageBox.ActionRole)
        back_btn = msg.addButton("Back to Config", QMessageBox.RejectRole)
        msg.exec_()
        clicked = msg.clickedButton()
        if clicked is begin_btn:
            self._show_ready_screen()
        elif clicked is repeat_btn:
            self._on_begin_practice()
        else:
            self.stack.setCurrentIndex(PAGE_CONFIG)

    def show_break(self, duration: int, block_just_done: int, total_blocks: int):
        self.stack.setCurrentIndex(PAGE_BREAK)
        self.break_screen.start_break(duration, block_just_done, total_blocks)

    def _on_break_resume(self):
        self.stack.setCurrentIndex(PAGE_EXPERIMENT)
        self.engine.resume_from_break()

    def show_pause(self):
        self.pause_screen.set_position(
            self.engine.active_trial_count,
            self.engine.sequencer.total_active_trials() if self.engine.sequencer else 0,
            self.engine.block_index + 1,
            len(self.engine.sequencer.blocks) if self.engine.sequencer else 0,
        )
        self.pause_screen.setGeometry(self.experiment_screen.rect())
        self.pause_screen.raise_()
        self.pause_screen.show()

    def hide_pause(self):
        self.pause_screen.hide()

    def _on_pause_resume(self):
        self.engine.resume()

    def _on_pause_end(self):
        self.hide_pause()
        self.engine.end_early()

    def show_summary(self, trials_completed: int, total_trials: int,
                     duration_s: float, csv_path: str, interrupted: int,
                     eeg_csv_path: str = "", eeg_samples: int = 0):
        self.showNormal()
        self.resize(900, 700)
        self.summary_screen.set_results(
            trials_completed, total_trials, duration_s, csv_path, interrupted,
            eeg_csv_path, eeg_samples,
        )
        self.stack.setCurrentIndex(PAGE_SUMMARY)

    def _on_new_session(self):
        self.stack.setCurrentIndex(PAGE_CONFIG)

    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_Escape:
            if self.engine.state not in (
                ExperimentState.IDLE, ExperimentState.FINISHED,
                ExperimentState.PAUSED, ExperimentState.BLOCK_BREAK,
            ):
                self.engine.pause()
                return

        if key == Qt.Key_Space:
            if self.stack.currentIndex() == PAGE_READY:
                self._start_real_experiment()
                return
            if self.engine.state == ExperimentState.PAUSED:
                self.engine.resume()
                return

        super().keyPressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Keep pause overlay sized to experiment screen
        if self.pause_screen.isVisible():
            self.pause_screen.setGeometry(self.experiment_screen.rect())

    def closeEvent(self, event):
        if self.engine.state not in (
            ExperimentState.IDLE, ExperimentState.FINISHED
        ):
            reply = QMessageBox.question(
                self, "Confirm Exit",
                "An experiment is in progress. Data recorded so far is saved.\n\n"
                "Are you sure you want to exit?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                event.ignore()
                return

            # Clean up EEG recorder and logger
            self.engine._stop_eeg_recorder()
            if self.engine.logger:
                duration = time.perf_counter() - self.engine.session_start_time
                self.engine.logger.write_metadata_json(
                    session_completed=False,
                    total_trials=self.engine.active_trial_count,
                    duration_s=duration,
                )
                self.engine.logger.close()

        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_STYLESHEET)

    window = ExperimentApp()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()