"""Stage 6: Session Summary panel.

Displayed after an experiment session completes (or is ended early).
Shows trial counts, duration, EEG samples recorded, signal quality
metrics, file paths, and navigation buttons.
"""
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QFrame,
    QScrollArea, QSizePolicy,
)

from eeg_app.core.constants import (
    DARK_BG, SURFACE, BORDER,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_MUTED, ACCENT, ACCENT_DIM,
)


class SummaryPanel(QWidget):
    """Session summary -- Stage 6 in the wizard flow.

    Signals
    -------
    new_session()
        Go back to config (Stage 3) for a new session with same participant.
    new_participant()
        Go back to electrode setup (Stage 2) for a new participant.
    quit_requested()
        Close the application.
    """
    new_session = pyqtSignal()
    new_participant = pyqtSignal()
    quit_requested = pyqtSignal()

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
        sep.setStyleSheet(
            f"color: {BORDER}; border: none; max-height: 1px; background: {BORDER};"
        )
        card_layout.addWidget(sep)

        self.stats_label = QLabel("")
        self.stats_label.setWordWrap(True)
        self.stats_label.setStyleSheet(
            f"font-size: 15px; color: {TEXT_PRIMARY}; border: none;"
        )
        card_layout.addWidget(self.stats_label)

        # Signal quality summary
        self._sq_label = QLabel("")
        self._sq_label.setWordWrap(True)
        self._sq_label.setStyleSheet(
            f"font-size: 13px; color: {TEXT_SECONDARY}; border: none;"
        )
        card_layout.addWidget(self._sq_label)

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

        new_session_btn = QPushButton("New Session")
        new_session_btn.setFixedHeight(44)
        new_session_btn.setStyleSheet(
            f"QPushButton {{ background-color: {ACCENT_DIM}; color: white; "
            f"font-weight: bold; border: none; border-radius: 6px; }}"
            f"QPushButton:hover {{ background-color: #00cc66; }}"
        )
        new_session_btn.clicked.connect(self.new_session.emit)
        btn_layout.addWidget(new_session_btn)

        new_participant_btn = QPushButton("New Participant")
        new_participant_btn.setFixedHeight(44)
        new_participant_btn.clicked.connect(self.new_participant.emit)
        btn_layout.addWidget(new_participant_btn)

        quit_btn = QPushButton("Quit")
        quit_btn.setFixedHeight(44)
        quit_btn.clicked.connect(self.quit_requested.emit)
        btn_layout.addWidget(quit_btn)

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

    def set_signal_quality(self, pct: float, usable: int, total: int, railed: int):
        """Set signal quality summary metrics."""
        self._sq_label.setText(
            f"Signal quality: {pct:.0f}%  |  "
            f"{usable}/{total} channels usable  |  {railed} railed"
        )
