"""ExperimentEngine -- trial sequencing and timing state machine.

Ported from data_collection.py. In the unified app the engine does NOT
create its own EEGRecorderThread (the SampleBus + CSVWriterThread handle
EEG recording). Instead the engine communicates state changes via Qt
signals so the UI layer can react without tight coupling.
"""
import time
from enum import Enum, auto
from typing import Optional

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from eeg_app.core.constants import GESTURES
from eeg_app.core.types import SessionConfig
from eeg_app.experiment.methodology import MethodologyDef, METHODOLOGIES
from eeg_app.experiment.sequencer import TrialSequencer
from eeg_app.recording.event_logger import CSVLogger, play_go_beep, play_stop_beep


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


class ExperimentEngine(QObject):
    """Drives the experiment through phases using QTimer.singleShot.

    Emits signals for all UI updates instead of referencing the app
    directly.

    Signals
    -------
    phase_changed(phase, gesture, subtext, show_video, go_stop_text, go_stop_color)
        Fired when the current trial phase changes.
    iti_started()
        Fired when an inter-trial interval begins.
    trial_info_changed(trial, total_trials, block, total_blocks)
        Fired when trial/block counters change.
    progress_changed(current, total)
        Fired when the overall progress changes.
    break_started(duration, block_just_done, total_blocks)
        Fired when a between-block break starts.
    pause_requested(trial, total_trials, block, total_blocks)
        Fired when the engine enters the PAUSED state.
    resume_requested()
        Fired when the engine leaves the PAUSED state.
    practice_completed()
        Fired when a practice run finishes.
    session_finished(trials_completed, total_trials, duration_s, csv_path,
                     interrupted, eeg_csv_path, eeg_samples)
        Fired when a real (non-practice) session finishes.
    """

    # UI-facing signals
    phase_changed = pyqtSignal(str, str, str, bool, str, str)
    iti_started = pyqtSignal()
    trial_info_changed = pyqtSignal(int, int, int, int)
    progress_changed = pyqtSignal(int, int)
    break_started = pyqtSignal(int, int, int)
    pause_requested = pyqtSignal(int, int, int, int)
    resume_requested = pyqtSignal()
    practice_completed = pyqtSignal()
    session_finished = pyqtSignal(int, int, float, str, int, str, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.config: Optional[SessionConfig] = None
        self.sequencer: Optional[TrialSequencer] = None
        self.logger: Optional[CSVLogger] = None
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

        # External references (set by app when recording starts)
        self._eeg_csv_path = ""
        self._eeg_sample_count_fn = None  # callable returning int

    def set_eeg_info(self, csv_path: str, sample_count_fn=None):
        """Called by the app to tell the engine where EEG data is saved."""
        self._eeg_csv_path = csv_path
        self._eeg_sample_count_fn = sample_count_fn

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
            self.break_started.emit(
                self.config.break_duration,
                self.block_index,  # just completed (1-based display)
                len(self.sequencer.blocks),
            )
            return

        trial = block[self.trial_in_block]
        is_rest = trial["is_rest"]

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
            total_active = len(GESTURES)
        else:
            total_active = self.sequencer.total_trials_per_block()
        total_display = total_active * (
            1 if self.is_practice else len(self.sequencer.blocks)
        )
        self.trial_info_changed.emit(
            self.active_trial_count,
            total_display,
            self.block_index + 1,
            len(self.sequencer.blocks),
        )
        self.progress_changed.emit(self.active_trial_count, total_display)

        # Start ITI
        self._start_iti()

    def _start_iti(self):
        iti_ms = int(self.config.iti_duration * 1000)
        if iti_ms <= 0:
            self._start_phase()
            return

        self.state = ExperimentState.ITI
        self.iti_started.emit()
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

        # Determine display parameters
        go_stop_text = ""
        go_stop_color = ""
        subtext = ""

        if phase_name == "Prep":
            subtext = self.methodology.prep_subtext
        elif phase_name == "Go":
            go_stop_text = "GO"
            go_stop_color = "#00ff44"
            if self.config.audio_cues:
                play_go_beep()
        elif phase_name == "Execute":
            subtext = f"{self.methodology.execute_prefix} {gesture}"
        elif phase_name == "End":
            go_stop_text = "STOP"
            go_stop_color = "#ff3333"
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

        self.phase_changed.emit(
            phase_name,
            gesture_display,
            subtext,
            show_video,
            go_stop_text,
            go_stop_color,
        )

        self._phase_index += 1
        self._active_timer = QTimer.singleShot(duration_ms, self._start_phase)

    def pause(self):
        if self.state in (ExperimentState.PAUSED, ExperimentState.IDLE,
                          ExperimentState.FINISHED):
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

        self.pause_requested.emit(
            self.active_trial_count,
            self.sequencer.total_active_trials() if self.sequencer else 0,
            self.block_index + 1,
            len(self.sequencer.blocks) if self.sequencer else 0,
        )

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

        self.resume_requested.emit()

        if self._paused_remaining_ms > 0:
            self._active_timer = QTimer.singleShot(
                self._paused_remaining_ms, self._start_phase
            )
        else:
            self._start_phase()

    def resume_from_break(self):
        self.trial_in_block = 0
        self._start_block()

    def _finish(self):
        self.state = ExperimentState.FINISHED
        duration = time.perf_counter() - self.session_start_time

        if self.is_practice:
            self.practice_completed.emit()
        else:
            eeg_path = self._eeg_csv_path
            eeg_samples = (
                self._eeg_sample_count_fn()
                if self._eeg_sample_count_fn else 0
            )
            if self.logger:
                self.logger.write_metadata_json(
                    session_completed=True,
                    total_trials=self.active_trial_count,
                    duration_s=duration,
                )
                self.logger.close()
            self.session_finished.emit(
                self.active_trial_count,
                self.sequencer.total_active_trials(),
                duration,
                self.logger.filepath if self.logger else "",
                self.interrupted_count,
                eeg_path,
                eeg_samples,
            )

    def end_early(self):
        """End session from pause screen."""
        duration = time.perf_counter() - self.session_start_time
        self.state = ExperimentState.FINISHED

        eeg_path = self._eeg_csv_path
        eeg_samples = (
            self._eeg_sample_count_fn()
            if self._eeg_sample_count_fn else 0
        )

        if not self.is_practice:
            if self.logger:
                self.logger.write_metadata_json(
                    session_completed=False,
                    total_trials=self.active_trial_count,
                    duration_s=duration,
                )
                self.logger.close()

        if self.is_practice:
            self.practice_completed.emit()
        else:
            total = (self.sequencer.total_active_trials()
                     if self.sequencer else self.active_trial_count)
            self.session_finished.emit(
                self.active_trial_count,
                total,
                duration,
                self.logger.filepath if self.logger else "",
                self.interrupted_count,
                eeg_path,
                eeg_samples,
            )
