"""CSVLogger — timestamped event log for experiment trials.

Ported from data_collection.py. Writes one CSV row per phase transition
(Prep, Go, Execute, End, Rest, Paused, Resumed) with perf_counter and
wall-clock timestamps for offline alignment with EEG data.
"""
import csv
import json
import os
import sys
import threading
import time
from datetime import datetime
from typing import Optional

from PyQt5.QtWidgets import QApplication

from eeg_app.core.types import SessionConfig


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
