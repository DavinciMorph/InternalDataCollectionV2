"""Stages 4-5: Experiment display screens.

Ported from data_collection.py. Contains:
- ExperimentScreen: phase-colored backgrounds, gesture cues, GO/STOP labels,
  progress bar, block/trial counters
- BreakScreen: countdown timer between blocks
- PauseScreen: overlay with resume/end options
- ReadyScreen: pre-experiment "press spacebar" prompt
- InstructionScreen: methodology instructions + practice option

Spacebar pauses at ANY time during trials/breaks/any phase. Spacebar
again resumes. Esc also pauses.
"""
import os
from typing import Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QProgressBar, QFrame,
    QScrollArea, QSizePolicy,
)

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

from eeg_app.core.constants import (
    DARK_BG, SURFACE, SURFACE_RAISED, BORDER,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_MUTED, ACCENT, ACCENT_DIM,
    PHASE_COLORS, GESTURE_COLORS, GESTURE_VIDEOS, GESTURES,
)
from eeg_app.core.types import SessionConfig
from eeg_app.experiment.methodology import METHODOLOGIES


# ---------------------------------------------------------------------------
# ExperimentScreen
# ---------------------------------------------------------------------------
class ExperimentScreen(QWidget):
    """Main trial display -- shows cue text, progress, phase colors."""

    def __init__(self, parent=None):
        super().__init__(parent)
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
        header.setStyleSheet("background: #111111;")
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

        # Fixed-height slot for GO label -- always occupies space
        self.go_stop_label = QLabel("")
        self.go_stop_label.setFixedHeight(120)
        self.go_stop_label.setStyleSheet(
            "font-size: 96px; font-weight: 900; color: transparent; "
            "background: transparent;"
        )
        self.go_stop_label.setAlignment(Qt.AlignCenter)
        cue_layout.addWidget(self.go_stop_label)

        # Fixed-height slot for gesture cue -- always occupies space
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
            "font-size: 22px; color: #aaaaaa; background: transparent;"
        )
        self.subtext_label.setAlignment(Qt.AlignCenter)
        cue_layout.addWidget(self.subtext_label)

        # Video label (OpenCV frames rendered here)
        self.video_label = QLabel("")
        self.video_label.setFixedSize(480, 360)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background: black; border-radius: 8px;")
        self.video_label.hide()
        cue_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)

        self._video_cap = None
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
                  go_stop_color: str = ""):
        color = PHASE_COLORS.get(phase, DARK_BG)
        self.cue_area.setStyleSheet(f"background: {color};")

        self.phase_indicator.setText(f"[ {phase.upper()} ]")
        self.cue_label.setText(gesture.upper() if gesture else "")
        self.cue_label.setStyleSheet(
            "font-size: 72px; font-weight: bold; color: white; "
            "background: transparent;"
        )
        self.subtext_label.setText(subtext)

        # GO label -- always keeps its fixed height, just make text
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

        if show_video and gesture and _HAS_CV2:
            video_path = GESTURE_VIDEOS.get(gesture)
            if video_path and os.path.isfile(video_path):
                self._start_video(video_path)
            else:
                self._stop_video()
                self.video_label.setStyleSheet(
                    f"font-size: 36px; font-weight: bold; color: white; "
                    f"background: {GESTURE_COLORS.get(gesture, '#3a3a3a')}; "
                    f"border-radius: 8px;"
                )
                self.video_label.setText(f"[Video]\n{gesture}")
                self.video_label.show()
        elif show_video and gesture:
            # No OpenCV -- show placeholder
            self._stop_video()
            self.video_label.setStyleSheet(
                f"font-size: 36px; font-weight: bold; color: white; "
                f"background: {GESTURE_COLORS.get(gesture, '#3a3a3a')}; "
                f"border-radius: 8px;"
            )
            self.video_label.setText(f"[Video]\n{gesture}")
            self.video_label.show()
        else:
            self._stop_video()

    def _start_video(self, path):
        self._stop_video()
        self._video_cap = cv2.VideoCapture(path)
        fps = self._video_cap.get(cv2.CAP_PROP_FPS) or 30
        self.video_label.setText("")
        self.video_label.setStyleSheet("background: black; border-radius: 8px;")
        self.video_label.show()
        self._video_timer.start(int(1000 / fps))

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
            self._video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._video_cap.read()
            if not ret:
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
        self.setStyleSheet("background: rgba(0, 0, 0, 200);")
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

        eeg_note = QLabel("EEG recording continues during pause")
        eeg_note.setStyleSheet(
            "font-size: 12px; color: #666666; background: transparent;"
        )
        eeg_note.setAlignment(Qt.AlignCenter)
        layout.addWidget(eeg_note)

        layout.addSpacing(24)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(16)
        btn_layout.setAlignment(Qt.AlignCenter)

        end_btn = QPushButton("End Session")
        end_btn.setFixedSize(160, 44)
        end_btn.setStyleSheet(
            "QPushButton { background-color: #5a1a1a; color: white; "
            "font-weight: bold; border: none; border-radius: 6px; }"
            "QPushButton:hover { background-color: #7a2a2a; }"
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
# ReadyScreen -- "Press Spacebar when ready"
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
        sep.setStyleSheet(
            f"color: {BORDER}; border: none; max-height: 1px; background: {BORDER};"
        )
        card_layout.addWidget(sep)

        self.instruction_label = QLabel()
        self.instruction_label.setWordWrap(True)
        self.instruction_label.setStyleSheet(
            "font-size: 15px; line-height: 1.6; border: none;"
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
            len(GESTURES) * config.trials_per_gesture * config.block_count
        )
        self.info_label.setText(
            f"{total_active} gesture trials across {config.block_count} block(s)  "
            f"|  Hand: {config.hand}  |  Modality: {m.modality}"
        )
        self.instruction_label.setText(m.instruction_text)
