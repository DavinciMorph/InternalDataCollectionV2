"""Stage 2: Electrode Setup — signal preview + quality grid."""
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSplitter, QFrame, QCheckBox, QComboBox, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
import pyqtgraph as pg

pg.setConfigOptions(useOpenGL=False, antialias=False)

from eeg_app.core.types import StreamMetadata, PortConfig
from eeg_app.core.config import AppConfig
from eeg_app.core.constants import (
    FS, LSB_UV, NEON_COLORS, PORT_COLORS, PORT_ORDER,
    DARK_BG, SURFACE, SURFACE_RAISED, BORDER, PLOT_BG,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_MUTED, ACCENT, ACCENT_DIM,
    STATUS_SUCCESS, STATUS_WARNING, STATUS_ERROR,
)
from eeg_app.processing.filters import FilterBank
from eeg_app.processing.signal_quality import SignalQualityTracker


class SignalPanel(QWidget):
    """Stage 2: Electrode setup with signal quality grid + 8-channel plots.

    Left panel: signal quality grid (color-coded per-channel quality)
    Right panel: 8-channel pyqtgraph plots for selected port/device
    """
    proceed_clicked = pyqtSignal()

    def __init__(self, metadata: StreamMetadata, config: AppConfig, parent=None):
        super().__init__(parent)
        self.metadata = metadata
        self.config = config
        self.port_configs = metadata.port_config
        self.num_channels = metadata.num_channels
        self.sample_rate = metadata.sample_rate or FS

        # Data state
        self.window_seconds = config.window_seconds
        self.window_size = int(self.window_seconds * self.sample_rate)
        self._raw_buffer = deque(maxlen=self.window_size)
        self._sample_count = 0

        # Selected port/device for plot display
        self._selected_port_idx = 0
        self._selected_device_idx = 0

        # Filters (always on: 60Hz notch, 50Hz LPF, 1Hz HPF)
        self._filters = FilterBank(num_channels=8, fs=self.sample_rate)
        self._cmf_on = False

        # Signal quality tracker
        self._sq_tracker = SignalQualityTracker(self.num_channels)
        self._sq_quality = []
        self._sq_pct = 0.0
        self._sq_usable = 0
        self._sq_railed = 0

        # Y-axis range
        self._y_range = config.y_range_uv

        self._build_ui()

        # Update timer for plots (50 Hz)
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plots)
        self._plot_timer.start(20)

        # SQ update timer (5 Hz)
        self._sq_timer = QTimer(self)
        self._sq_timer.timeout.connect(self._update_sq)
        self._sq_timer.start(200)

        # Enable proceed button after 10 seconds of data
        self._enable_timer = QTimer(self)
        self._enable_timer.setSingleShot(True)
        self._enable_timer.timeout.connect(lambda: self._proceed_btn.setEnabled(True))
        self._enable_timer.start(10000)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Top nav bar
        nav = QHBoxLayout()
        nav.setContentsMargins(12, 8, 12, 8)

        title = QLabel("ELECTRODE SETUP")
        title.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 18px; font-weight: bold;")
        nav.addWidget(title)
        nav.addStretch()

        # SQ indicator
        self._sq_label = QLabel("SQ: ---%")
        self._sq_label.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 13px; padding: 4px 10px; "
            f"border: 1px solid {BORDER}; border-radius: 10px;"
        )
        nav.addWidget(self._sq_label)

        self._proceed_btn = QPushButton("Signals Look Good  →")
        self._proceed_btn.setEnabled(False)
        self._proceed_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_DIM};
                color: #0d0d0d;
                border: none;
                border-radius: 6px;
                padding: 8px 20px;
                font-weight: bold;
                font-size: 14px;
            }}
            QPushButton:hover {{ background-color: {ACCENT}; }}
            QPushButton:disabled {{ background-color: #1a1a1a; color: {TEXT_MUTED}; }}
        """)
        self._proceed_btn.clicked.connect(self.proceed_clicked.emit)
        nav.addWidget(self._proceed_btn)

        layout.addLayout(nav)

        # Main content: splitter with SQ grid + signal plots
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("QSplitter::handle { background: #333; width: 2px; }")

        # Left panel: quality grid
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(12, 8, 4, 8)

        # Port selector buttons
        port_btn_layout = QHBoxLayout()
        self._port_buttons: list[QPushButton] = []
        for i, pc in enumerate(self.port_configs):
            btn = QPushButton(pc.name)
            color = PORT_COLORS[i % len(PORT_COLORS)]
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {SURFACE_RAISED};
                    color: {color};
                    border: 1px solid {color};
                    border-radius: 4px;
                    padding: 4px 8px;
                    font-size: 11px;
                    font-weight: bold;
                }}
                QPushButton:checked {{
                    background-color: {color};
                    color: #0d0d0d;
                }}
            """)
            btn.setCheckable(True)
            btn.setChecked(i == 0)
            btn.clicked.connect(lambda checked, idx=i: self._on_port_selected(idx))
            port_btn_layout.addWidget(btn)
            self._port_buttons.append(btn)
        left_layout.addLayout(port_btn_layout)

        # Device selector buttons
        self._device_btn_layout = QHBoxLayout()
        self._device_buttons: list[QPushButton] = []
        left_layout.addLayout(self._device_btn_layout)
        self._rebuild_device_buttons()

        # Quality grid placeholder — simple per-channel quality labels
        self._quality_grid_frame = QFrame()
        self._quality_grid_frame.setStyleSheet(f"background: {SURFACE}; border-radius: 6px;")
        self._quality_grid_layout = QVBoxLayout(self._quality_grid_frame)
        self._quality_grid_layout.setContentsMargins(8, 8, 8, 8)

        self._quality_labels: list[QLabel] = []
        for ch in range(8):
            lbl = QLabel(f"Ch {ch+1}: ---")
            lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
            self._quality_grid_layout.addWidget(lbl)
            self._quality_labels.append(lbl)

        left_layout.addWidget(self._quality_grid_frame, 1)

        # Quality summary bar
        self._summary_label = QLabel("--- / --- channels usable")
        self._summary_label.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 13px;")
        self._summary_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self._summary_label)

        # CMF toggle
        self._cmf_check = QCheckBox("Common Mode Filter")
        self._cmf_check.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 12px;")
        self._cmf_check.toggled.connect(lambda v: setattr(self, '_cmf_on', v))
        left_layout.addWidget(self._cmf_check)

        splitter.addWidget(left)

        # Right panel: 8-channel plots
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 8, 12, 8)

        # Y-range selector
        y_layout = QHBoxLayout()
        y_label = QLabel("Y Range:")
        y_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        y_layout.addWidget(y_label)
        self._y_combo = QComboBox()
        self._y_combo.addItems(["50 uV", "100 uV", "200 uV", "500 uV", "1000 uV"])
        self._y_combo.setCurrentText(f"{self._y_range} uV")
        self._y_combo.currentTextChanged.connect(self._on_y_range_changed)
        y_layout.addWidget(self._y_combo)
        y_layout.addStretch()
        right_layout.addLayout(y_layout)

        # 8-channel plot widgets
        self._plots: list[pg.PlotWidget] = []
        self._curves: list[pg.PlotDataItem] = []
        self._ch_labels: list[QLabel] = []

        for ch in range(8):
            row = QHBoxLayout()
            row.setSpacing(4)

            lbl = QLabel(f"Ch{ch+1}")
            lbl.setFixedWidth(35)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            color = NEON_COLORS[ch % len(NEON_COLORS)]
            lbl.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold;")
            row.addWidget(lbl)
            self._ch_labels.append(lbl)

            pw = pg.PlotWidget()
            pw.setBackground(PLOT_BG)
            pw.setMouseEnabled(x=False, y=False)
            pw.hideButtons()
            pw.getPlotItem().hideAxis("bottom")
            pw.getPlotItem().getAxis("left").setWidth(45)
            pw.getPlotItem().getAxis("left").setStyle(tickLength=-5)
            pw.getPlotItem().getAxis("left").setPen(pg.mkPen(color="#444"))
            pw.getPlotItem().getAxis("left").setTextPen(pg.mkPen(color="#666"))
            pw.setYRange(-self._y_range, self._y_range)

            curve = pw.plot(pen=pg.mkPen(color=color, width=1))
            self._plots.append(pw)
            self._curves.append(curve)
            row.addWidget(pw, 1)

            right_layout.addLayout(row)

        # Show time axis on last plot
        self._plots[-1].getPlotItem().showAxis("bottom")
        self._plots[-1].getPlotItem().getAxis("bottom").setLabel("Time", units="s")

        splitter.addWidget(right)
        splitter.setSizes([350, 650])

        layout.addWidget(splitter, 1)

    def _rebuild_device_buttons(self):
        """Rebuild device buttons for current port."""
        # Clear existing
        for btn in self._device_buttons:
            btn.deleteLater()
        self._device_buttons.clear()

        if self._selected_port_idx >= len(self.port_configs):
            return

        pc = self.port_configs[self._selected_port_idx]
        if pc.num_devices <= 1:
            return

        for d in range(pc.num_devices):
            btn = QPushButton(f"Dev {d+1}")
            btn.setCheckable(True)
            btn.setChecked(d == self._selected_device_idx)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {SURFACE_RAISED};
                    color: {TEXT_SECONDARY};
                    border: 1px solid {BORDER};
                    border-radius: 3px;
                    padding: 2px 6px;
                    font-size: 10px;
                }}
                QPushButton:checked {{
                    background-color: {ACCENT_DIM};
                    color: #0d0d0d;
                    border-color: {ACCENT};
                }}
            """)
            btn.clicked.connect(lambda checked, idx=d: self._on_device_selected(idx))
            self._device_btn_layout.addWidget(btn)
            self._device_buttons.append(btn)

    def _on_port_selected(self, idx: int):
        self._selected_port_idx = idx
        self._selected_device_idx = 0
        # Update button states
        for i, btn in enumerate(self._port_buttons):
            btn.setChecked(i == idx)
        self._rebuild_device_buttons()
        self._filters.reset(8)

    def _on_device_selected(self, idx: int):
        self._selected_device_idx = idx
        for i, btn in enumerate(self._device_buttons):
            btn.setChecked(i == idx)
        self._filters.reset(8)

    def _on_y_range_changed(self, text: str):
        try:
            val = int(text.replace(" uV", ""))
            self._y_range = val
            for pw in self._plots:
                pw.setYRange(-val, val)
        except ValueError:
            pass

    def on_sample(self, sample: dict):
        """Called by SampleBus on main thread for each incoming sample."""
        self._raw_buffer.append(sample["channels"])
        self._sample_count += 1
        self._sq_tracker.push_sample(sample["channels"])

    def _get_selected_channels(self) -> tuple:
        """Get channel offset and count for the selected port/device."""
        offset = 0
        for i, pc in enumerate(self.port_configs):
            if i == self._selected_port_idx:
                offset += self._selected_device_idx * 8
                return offset, 8
            offset += pc.num_devices * 8
        return 0, 8

    def _update_plots(self):
        """Update the 8-channel plots (called at 50 Hz)."""
        if len(self._raw_buffer) < 2:
            return

        ch_offset, num_ch = self._get_selected_channels()
        total_ch = self.num_channels

        # Extract raw data for selected device
        raw = np.array(list(self._raw_buffer), dtype=np.float64)
        n_samples = raw.shape[0]

        # Extract 8 channels for selected device
        device_data = np.zeros((n_samples, 8))
        for c in range(8):
            idx = ch_offset + c
            if idx < total_ch:
                device_data[:, c] = raw[:, idx] * LSB_UV

        # Apply filters (always on)
        filtered = self._filters.apply(device_data, cmf=self._cmf_on)

        # Time axis
        t = np.arange(n_samples) / self.sample_rate

        # Update curves
        for c in range(8):
            self._curves[c].setData(t, filtered[:, c])

    def _update_sq(self):
        """Update signal quality display (called at 5 Hz)."""
        quality, pct, usable, railed = self._sq_tracker.compute_quality()
        self._sq_quality = quality
        self._sq_pct = pct
        self._sq_usable = usable
        self._sq_railed = railed

        # Update SQ label
        color = STATUS_SUCCESS if pct > 70 else (STATUS_WARNING if pct > 40 else STATUS_ERROR)
        self._sq_label.setText(f"SQ: {pct:.0f}%")
        self._sq_label.setStyleSheet(
            f"color: {color}; font-size: 13px; padding: 4px 10px; "
            f"border: 1px solid {color}; border-radius: 10px;"
        )

        # Update summary
        self._summary_label.setText(
            f"{usable}/{self.num_channels} channels usable ({pct:.0f}%)  |  "
            f"{railed} railed"
        )

        # Update per-channel quality labels (for selected device)
        ch_offset, _ = self._get_selected_channels()
        for c in range(8):
            idx = ch_offset + c
            if idx < len(quality):
                q = quality[idx]
                qcolors = {
                    "good": STATUS_SUCCESS,
                    "marginal": STATUS_WARNING,
                    "bad": STATUS_ERROR,
                    "railed": STATUS_ERROR,
                    "unknown": TEXT_MUTED,
                }
                qcolor = qcolors.get(q, TEXT_MUTED)
                self._quality_labels[c].setText(f"Ch {c+1}: {q}")
                self._quality_labels[c].setStyleSheet(f"color: {qcolor}; font-size: 12px;")
