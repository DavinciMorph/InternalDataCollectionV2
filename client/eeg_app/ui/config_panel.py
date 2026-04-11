"""Stage 3: Session Configuration panel.

Simplified config screen with participant/session info, methodology
selection, and advanced timing settings. Replaces the standalone
ConfigScreen from data_collection.py.
"""
import hashlib
import os
from datetime import datetime

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox,
    QPushButton, QCheckBox, QRadioButton, QButtonGroup,
    QFrame, QScrollArea, QSizePolicy, QMessageBox, QFileDialog,
)

from eeg_app.core.constants import (
    DARK_BG, SURFACE, SURFACE_RAISED, BORDER, BORDER_FOCUS,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_MUTED, ACCENT, ACCENT_DIM,
    GESTURES,
)
from eeg_app.core.types import SessionConfig
from eeg_app.experiment.methodology import (
    METHODOLOGIES, compute_session_defaults, DEFAULT_ITI_S,
)


class ConfigPanel(QWidget):
    """Session configuration — Stage 3 in the wizard flow.

    Signals
    -------
    start_requested(SessionConfig)
        Emitted when the user clicks "Start Session".
    back_requested()
        Emitted when the user clicks "Back" to return to electrode setup.
    """
    start_requested = pyqtSignal(object)  # SessionConfig
    back_requested = pyqtSignal()

    def __init__(self, app_config, parent=None):
        super().__init__(parent)
        self._app_config = app_config
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        # Top nav bar
        nav = QHBoxLayout()
        nav.setContentsMargins(12, 8, 12, 8)

        back_btn = QPushButton("<  Back to Electrodes")
        back_btn.setStyleSheet(
            f"color: {TEXT_SECONDARY}; border: none; font-size: 13px; "
            f"background: transparent;"
        )
        back_btn.setCursor(Qt.PointingHandCursor)
        back_btn.clicked.connect(self.back_requested.emit)
        nav.addWidget(back_btn)

        nav.addStretch()

        title = QLabel("SESSION CONFIG")
        title.setStyleSheet(
            f"color: {TEXT_PRIMARY}; font-size: 18px; font-weight: bold;"
        )
        nav.addWidget(title)

        nav.addStretch()

        # SQ indicator placeholder (can be connected externally)
        self._sq_label = QLabel("SQ: ---%")
        self._sq_label.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 13px; padding: 4px 10px; "
            f"border: 1px solid {BORDER}; border-radius: 10px;"
        )
        nav.addWidget(self._sq_label)

        outer.addLayout(nav)

        # Scroll area
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
        card_title = QLabel("BCI Gesture Paradigm")
        card_title.setStyleSheet("font-size: 28px; font-weight: bold; border: none;")
        card_title.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(card_title)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(
            f"color: {BORDER}; border: none; max-height: 1px; background: {BORDER};"
        )
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

        self.participant_edit = QLineEdit()
        self.participant_edit.setPlaceholderText("e.g. P001")
        if self._app_config.last_participant_id:
            self.participant_edit.setText(self._app_config.last_participant_id)
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
        default_hand = getattr(self._app_config, 'default_hand', 'Right')
        if default_hand == "Left":
            self.hand_left.setChecked(True)
        else:
            self.hand_right.setChecked(True)
        self.hand_group.addButton(self.hand_left, 0)
        self.hand_group.addButton(self.hand_right, 1)
        hand_layout.addWidget(self.hand_left)
        hand_layout.addWidget(self.hand_right)
        hand_layout.addStretch()
        form1.addRow("Hand:", hand_widget)

        self.methodology_combo = QComboBox()
        for mid, mdef in METHODOLOGIES.items():
            self.methodology_combo.addItem(mdef.name, mid)
        # Set default from config
        default_mid = getattr(self._app_config, 'default_methodology', 1)
        idx = list(METHODOLOGIES.keys()).index(default_mid) if default_mid in METHODOLOGIES else 0
        self.methodology_combo.setCurrentIndex(idx)
        form1.addRow("Methodology:", self.methodology_combo)

        card_layout.addLayout(form1)

        # Estimated duration label
        self._duration_label = QLabel("")
        self._duration_label.setStyleSheet(
            f"font-size: 13px; color: {TEXT_SECONDARY}; border: none;"
        )
        self._duration_label.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(self._duration_label)

        # --- Advanced timing section (collapsed by default) ---
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet(
            f"color: {BORDER}; border: none; max-height: 1px; background: {BORDER};"
        )
        card_layout.addWidget(sep2)

        self._advanced_toggle = QPushButton("Advanced Timing  v")
        self._advanced_toggle.setStyleSheet(
            f"color: {TEXT_SECONDARY}; border: none; font-size: 13px; "
            f"background: transparent; text-align: left; padding: 4px 0;"
        )
        self._advanced_toggle.setCursor(Qt.PointingHandCursor)
        self._advanced_toggle.clicked.connect(self._toggle_advanced)
        card_layout.addWidget(self._advanced_toggle)

        self._advanced_frame = QWidget()
        advanced_layout = QFormLayout(self._advanced_frame)
        advanced_layout.setSpacing(8)
        advanced_layout.setLabelAlignment(Qt.AlignRight)

        self.trials_spin = QSpinBox()
        self.trials_spin.setRange(1, 100)
        self.trials_spin.setValue(16)
        advanced_layout.addRow("Trials per Gesture:", self.trials_spin)

        self.blocks_spin = QSpinBox()
        self.blocks_spin.setRange(1, 20)
        self.blocks_spin.setValue(8)
        advanced_layout.addRow("Block Count:", self.blocks_spin)

        self.break_spin = QSpinBox()
        self.break_spin.setRange(5, 300)
        self.break_spin.setValue(30)
        self.break_spin.setSuffix(" s")
        advanced_layout.addRow("Break Duration:", self.break_spin)

        self.iti_spin = QDoubleSpinBox()
        self.iti_spin.setRange(0.0, 5.0)
        self.iti_spin.setSingleStep(0.1)
        self.iti_spin.setDecimals(1)
        self.iti_spin.setValue(0.5)
        self.iti_spin.setSuffix(" s")
        advanced_layout.addRow("ITI Duration:", self.iti_spin)

        self.audio_check = QCheckBox("Enable GO/STOP beeps")
        advanced_layout.addRow("Audio Cues:", self.audio_check)

        # Output directory
        dir_widget = QWidget()
        dir_layout = QHBoxLayout(dir_widget)
        dir_layout.setContentsMargins(0, 0, 0, 0)
        self.output_edit = QLineEdit()
        default_dir = getattr(self._app_config, 'output_dir', 'experiment_data')
        if not os.path.isabs(default_dir):
            default_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__)
                ))),
                default_dir,
            )
        self.output_edit.setText(os.path.abspath(default_dir))
        browse_btn = QPushButton("Browse")
        browse_btn.setFixedWidth(80)
        browse_btn.clicked.connect(self._browse_output)
        dir_layout.addWidget(self.output_edit, 1)
        dir_layout.addWidget(browse_btn)
        advanced_layout.addRow("Output Dir:", dir_widget)

        self._advanced_frame.hide()
        card_layout.addWidget(self._advanced_frame)

        # Auto-compute when methodology changes
        self.methodology_combo.currentIndexChanged.connect(
            self._update_timing_defaults
        )
        self._update_timing_defaults()

        # Start button
        card_layout.addSpacing(8)
        self.start_btn = QPushButton("Start Session  \u2192")
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
        outer.addWidget(scroll, 1)

    def _toggle_advanced(self):
        visible = self._advanced_frame.isVisible()
        self._advanced_frame.setVisible(not visible)
        arrow = "^" if not visible else "v"
        self._advanced_toggle.setText(f"Advanced Timing  {arrow}")

    def _update_timing_defaults(self):
        mid = self.methodology_combo.currentData()
        if mid is not None:
            tpg, bc = compute_session_defaults(mid, self.iti_spin.value())
            self.trials_spin.setValue(tpg)
            self.blocks_spin.setValue(bc)
            self._update_duration_estimate()

    def _update_duration_estimate(self):
        mid = self.methodology_combo.currentData()
        if mid is None:
            return
        m = METHODOLOGIES[mid]
        gesture_time = sum(d / 1000 for _, d in m.phases)
        rest_time = sum(d / 1000 for _, d in m.rest_phases)
        iti = self.iti_spin.value()
        cycle_time = gesture_time + rest_time + 2 * iti

        total_trials = self.trials_spin.value() * len(GESTURES)
        num_blocks = self.blocks_spin.value()
        break_dur = self.break_spin.value()

        total_s = total_trials * cycle_time + max(0, num_blocks - 1) * break_dur
        mins = int(total_s) // 60
        secs = int(total_s) % 60
        self._duration_label.setText(
            f"Estimated duration: ~{mins}m {secs}s  "
            f"({total_trials} trials, {num_blocks} blocks)"
        )

    def _browse_output(self):
        d = QFileDialog.getExistingDirectory(
            self, "Select Output Directory", self.output_edit.text()
        )
        if d:
            self.output_edit.setText(d)

    def update_sq(self, pct: float, color: str):
        """Update the signal quality indicator in the header."""
        self._sq_label.setText(f"SQ: {pct:.0f}%")
        self._sq_label.setStyleSheet(
            f"color: {color}; font-size: 13px; padding: 4px 10px; "
            f"border: 1px solid {color}; border-radius: 10px;"
        )

    def _on_start(self):
        pid = self.participant_edit.text().strip()
        if not pid:
            QMessageBox.warning(
                self, "Validation Error", "Participant ID cannot be empty."
            )
            return
        sid = self.session_edit.text().strip()
        if not sid:
            QMessageBox.warning(
                self, "Validation Error", "Session ID cannot be empty."
            )
            return
        out_dir = self.output_edit.text().strip()
        if not out_dir:
            QMessageBox.warning(
                self, "Validation Error", "Output directory cannot be empty."
            )
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
            latin_square_num=1,
            randomization_seed=seed,
        )
        self.start_requested.emit(config)
