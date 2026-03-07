#!/usr/bin/env uv run script
# /// script
# dependencies = [
#   "numpy",
#   "PyQt5",
#   "pyqtgraph",
#   "scipy",
# ]
# ///
"""
ADS1299 Signal Visualizer (Server-Timed, Binary TCP)
====================================================

Streams binary LZ4-compressed EEG data from Controller.py and displays
8 channels for a selected port + device combination.

Port/device selection via top-row buttons. Only one device's 8 plots
are visible at a time, filling the full window.

Port configuration is auto-detected from the server metadata.
Use --spi-ports to manually override if needed.

Usage:
    uv run simpleviz.py --host <pi-ip>
    uv run simpleviz.py --host <pi-ip> --spi-ports "Port1,2 Port2,3"
"""

import sys
import os
import socket
import json
import argparse
import time
import struct
import traceback
import threading
from collections import deque
from datetime import datetime
from queue import Queue, Empty
from dataclasses import dataclass
from typing import List, Optional
import numpy as np
from scipy import signal as sig

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
# OpenGL disabled — it causes crashes on many GPU drivers
pg.setConfigOptions(useOpenGL=False, antialias=False)

# ============================================================================
# ADC CONSTANTS
# ============================================================================
FS = 250                # Sample rate (Hz)
VREF = 4.5              # ADS1299 reference voltage (V)
GAIN = 24               # PGA gain
LSB_UV = (VREF / (2**23) / GAIN) * 1e6  # uV per LSB = 0.02235 uV

# ============================================================================
# DARK NEON THEME
# ============================================================================
DARK_BG = "#0d0d0d"
PLOT_BG = "#0a0a0a"
TEXT_COLOR = "#e0e0e0"
ACCENT_COLOR = "#00ff88"

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

PORT_COLORS = [
    "#ff6b6b",  # Coral red
    "#4ecdc4",  # Teal
    "#ffe66d",  # Yellow
    "#95e1d3",  # Mint
    "#f38181",  # Salmon
    "#aa96da",  # Lavender
    "#fcbad3",  # Pink
    "#a8d8ea",  # Light blue
]

STYLESHEET = f"""
QMainWindow {{
    background-color: {DARK_BG};
}}
QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_COLOR};
    font-family: 'Segoe UI', 'Arial', sans-serif;
}}
QLabel {{
    color: {TEXT_COLOR};
    font-size: 11pt;
}}
QPushButton {{
    background-color: #2a2a2a;
    color: {TEXT_COLOR};
    border: 1px solid #444;
    border-radius: 4px;
    padding: 6px 14px;
    font-size: 10pt;
}}
QPushButton:hover {{
    background-color: #3a3a3a;
    border-color: {ACCENT_COLOR};
}}
"""


# ============================================================================
# PORT CONFIGURATION
# ============================================================================


@dataclass
class SPIPortConfig:
    name: str
    num_devices: int

    @property
    def num_channels(self) -> int:
        return self.num_devices * 8


def parse_spi_ports(ports_string: str) -> List[SPIPortConfig]:
    ports = []
    for entry in ports_string.split():
        parts = entry.split(',')
        if len(parts) != 2:
            raise ValueError(f"Invalid port format '{entry}'. Expected 'Name,NumDevices'")
        ports.append(SPIPortConfig(name=parts[0], num_devices=int(parts[1])))
    return ports


# ============================================================================
# CLIENT-SIDE CSV WRITER (async, non-blocking)
# ============================================================================

class CSVWriterThread:
    """Async CSV writer that runs in a dedicated thread.

    Consumes raw (pre-filter) sample dicts from a queue and writes them
    to a timestamped CSV file in the same format as the server's CSVWriter:
        timestamp,sample_number,Port1_dev1_ch1,...,PortN_devM_ch8

    Non-blocking: the network thread does a non-blocking put(); if the
    CSV queue is full, the sample is silently dropped (GUI/network never stalls).
    """

    def __init__(self, port_configs, csv_dir="."):
        """Initialize the writer. Does NOT start writing until start() is called.

        Args:
            port_configs: list of dicts with 'name' and 'num_devices' keys
                          (from server metadata port_config)
            csv_dir: directory to write the CSV file into
        """
        self._port_configs = port_configs
        self._csv_dir = csv_dir
        self._queue = Queue(maxsize=10000)
        self._running = False
        self._thread = None
        self._total_written = 0
        self._filename = None

        # Build channel names matching server format: PortN_devM_chK
        self._channel_names = []
        for pc in port_configs:
            port_name = pc['name']
            num_devices = pc['num_devices']
            for d in range(num_devices):
                for c in range(8):
                    self._channel_names.append(
                        f"{port_name}_dev{d+1}_ch{c+1}"
                    )
        self._num_channels = len(self._channel_names)

    @property
    def filename(self):
        return self._filename

    @property
    def total_written(self):
        return self._total_written

    def start(self):
        """Open the CSV file and start the writer thread."""
        if self._running:
            return

        ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self._filename = os.path.join(self._csv_dir, f"eeg_data_{ts}.csv")

        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="csv-writer", daemon=True
        )
        self._thread.start()
        print(f"[csv] Writing {self._num_channels} channels to {self._filename}")

    def push(self, sample):
        """Non-blocking enqueue. Drops sample if queue is full."""
        if not self._running:
            return
        try:
            self._queue.put_nowait(sample)
        except Exception:
            pass  # Queue full — drop silently, never block network thread

    def stop(self):
        """Signal the writer to drain remaining samples, flush, and close."""
        if not self._running:
            return
        self._running = False
        # Put a sentinel to unblock the writer if it's waiting
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        print(f"[csv] Writer stopped: {self._total_written} samples written")

    def _run(self):
        """Writer thread main loop."""
        f = None
        try:
            f = open(self._filename, 'w', buffering=65536)  # 64KB buffer

            # Write header
            header = "timestamp,sample_number," + ",".join(self._channel_names) + "\n"
            f.write(header)

            while True:
                # Block up to 100ms waiting for samples (matches server flush timeout)
                try:
                    sample = self._queue.get(timeout=0.1)
                except Empty:
                    if not self._running:
                        break
                    continue

                if sample is None:
                    # Sentinel — drain remaining and exit
                    break

                self._write_sample(f, sample)
                self._total_written += 1

                # Drain burst: write up to 500 more without blocking
                for _ in range(500):
                    try:
                        sample = self._queue.get_nowait()
                    except Empty:
                        break
                    if sample is None:
                        break
                    self._write_sample(f, sample)
                    self._total_written += 1

            # Drain anything remaining after stop signal
            while True:
                try:
                    sample = self._queue.get_nowait()
                except Empty:
                    break
                if sample is None:
                    continue
                self._write_sample(f, sample)
                self._total_written += 1

        except Exception as e:
            print(f"[csv] Writer error: {e}")
            traceback.print_exc()
        finally:
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass

    def _write_sample(self, f, sample):
        """Format and write a single sample row.

        Format matches server: %.6f,sample_number,ch1,ch2,...,chN
        Channels are raw int32 values (no float conversion).
        """
        channels = sample['channels']
        num_ch = min(len(channels), self._num_channels)

        # Build row: timestamp (6 decimal places), sample_number, then channels
        parts = [f"{sample['timestamp']:.6f}", str(sample['sample_number'])]
        for i in range(num_ch):
            parts.append(str(channels[i]))
        # Pad with zeros if fewer channels than expected
        for i in range(num_ch, self._num_channels):
            parts.append('0')

        f.write(','.join(parts))
        f.write('\n')


# ============================================================================
# MAIN VISUALIZER
# ============================================================================

class SignalVisualizer(QtWidgets.QMainWindow):
    """Real-time EEG signal visualizer with port/device selector and 8-channel view."""

    # Desired left-to-right display order for port buttons.
    # Ports not listed here will be appended at the end in their natural order.
    DISPLAY_ORDER = ["Port4", "Port3", "Port2", "Port1", "Port7", "Port6", "Port5"]

    def __init__(self, host, port, spi_ports, window_seconds=5.0,
                 csv_enabled=True, csv_dir='.'):
        super().__init__()

        self.host = host
        self.port = port
        self.spi_ports = spi_ports
        self.window_seconds = window_seconds
        self.csv_enabled = csv_enabled
        self.csv_dir = csv_dir
        self.sample_rate = 250
        self.max_window_seconds = 30  # Slider max
        self.max_samples = int(self.sample_rate * self.max_window_seconds)
        self.gain = GAIN
        self.lsb_uv = (VREF / (2**23) / self.gain) * 1e6

        self.notch60_on = True
        self.notch50_on = False
        self.notch70_on = False
        self.notch80_on = False
        self.notch120_on = False
        self.lpf_on = False
        self.hpf_on = False
        self.cmf_on = False
        self.y_autoscale = False
        self.y_fixed_range = 200  # uV

        # 60 Hz notch filter coefficients
        self.notch60_b, self.notch60_a = sig.iirnotch(
            60.0, Q=30.0, fs=self.sample_rate
        )
        self.notch60_zi_template = sig.lfilter_zi(self.notch60_b, self.notch60_a)

        # 50 Hz notch filter coefficients
        self.notch50_b, self.notch50_a = sig.iirnotch(
            50.0, Q=30.0, fs=self.sample_rate
        )
        self.notch50_zi_template = sig.lfilter_zi(self.notch50_b, self.notch50_a)

        # 70 Hz notch filter coefficients
        self.notch70_b, self.notch70_a = sig.iirnotch(
            70.0, Q=30.0, fs=self.sample_rate
        )
        self.notch70_zi_template = sig.lfilter_zi(self.notch70_b, self.notch70_a)

        # 80 Hz notch filter coefficients
        self.notch80_b, self.notch80_a = sig.iirnotch(
            80.0, Q=30.0, fs=self.sample_rate
        )
        self.notch80_zi_template = sig.lfilter_zi(self.notch80_b, self.notch80_a)

        # 120 Hz notch filter coefficients
        self.notch120_b, self.notch120_a = sig.iirnotch(
            120.0, Q=30.0, fs=self.sample_rate
        )
        self.notch120_zi_template = sig.lfilter_zi(self.notch120_b, self.notch120_a)

        # 50 Hz low-pass filter coefficients (10th-order Butterworth, SOS form)
        self.lpf_sos = sig.butter(
            10, 50.0, btype='low', fs=self.sample_rate, output='sos'
        )
        self.lpf_zi_template = sig.sosfilt_zi(self.lpf_sos)

        # 1 Hz high-pass filter coefficients (10th-order Butterworth, SOS form)
        self.hpf_sos = sig.butter(
            10, 1.0, btype='high', fs=self.sample_rate, output='sos'
        )
        self.hpf_zi_template = sig.sosfilt_zi(self.hpf_sos)

        # Thread-safe queue for cross-thread sample transfer
        self.sample_queue = Queue(maxsize=5000)

        # CSV writer (created when server config arrives)
        self.csv_writer = None

        self._init_data_state()

        # Port/device selection state
        # _selected_port_display_idx: index into _display_order_indices (which port button is active)
        # _selected_device_idx: device index (0-based) within the selected port
        self._selected_port_display_idx = None
        self._selected_device_idx = None

        # Computed mapping from display order to data order
        # _display_order_indices[i] = index into self.spi_ports for the i-th displayed port
        self._display_order_indices = []
        # _port_ch_offsets[orig_idx] = channel offset in data buffer for spi_ports[orig_idx]
        self._port_ch_offsets = []

        # UI element lists (populated in setup_ui / _rebuild_selectors)
        self._port_buttons: List[QtWidgets.QPushButton] = []
        self._port_button_colors: List[str] = []  # color for each port button (display order)
        self._port_stuck_state: List[bool] = []  # per port button stuck state
        self._device_buttons: List[QtWidgets.QPushButton] = []
        self._device_stuck_state: List[bool] = []

        # 8 persistent plot widgets + curves (created once in setup_ui)
        self._plot_widgets: List[pg.PlotWidget] = []
        self._plot_curves: List[pg.PlotDataItem] = []
        self._ch_labels: List[QtWidgets.QLabel] = []

        # Smoothed Y-axis ranges for autoscale (per plot)
        self._y_smooth_min = [0.0] * 8
        self._y_smooth_max = [0.0] * 8

        self.setup_ui()

        self.network_thread = NetworkThread(self, host, port)
        self.network_thread.config_received.connect(self._on_config_received)
        self.network_thread.start()

        # 20ms timer (~50 Hz) — more stable than 16ms
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(20)

    def _init_data_state(self):
        """Initialize (or reset) ring buffers, filters, and playback state."""
        self.total_channels = sum(p.num_channels for p in self.spi_ports)

        buf_size = self.max_samples * 2
        ch = max(self.total_channels, 1)  # avoid 0-dim arrays
        self.time_buf = np.zeros((ch, buf_size))
        self.data_buf = np.zeros((ch, buf_size))
        self.buf_idx = 0
        self.buf_count = 0

        self.pending_samples = deque()
        self.buffer_delay = 0.25
        self.playback_wall_start = None
        self.playback_server_start = None

        self.start_time = None
        self.sample_count = 0
        self.frame_count = 0

        self.notch60_states = [
            self.notch60_zi_template.copy() for _ in range(self.total_channels)
        ]
        self.notch50_states = [
            self.notch50_zi_template.copy() for _ in range(self.total_channels)
        ]
        self.notch70_states = [
            self.notch70_zi_template.copy() for _ in range(self.total_channels)
        ]
        self.notch80_states = [
            self.notch80_zi_template.copy() for _ in range(self.total_channels)
        ]
        self.notch120_states = [
            self.notch120_zi_template.copy() for _ in range(self.total_channels)
        ]
        self.lpf_states = [
            self.lpf_zi_template.copy() for _ in range(self.total_channels)
        ]
        self.hpf_states = [
            self.hpf_zi_template.copy() for _ in range(self.total_channels)
        ]

        # Stuck channel detection
        self.stuck_threshold = 50  # 50 consecutive identical = 200ms at 250Hz
        self.stuck_last_value = np.full(ch, np.nan)
        self.stuck_count = np.zeros(ch, dtype=int)
        self.stuck_channels = {}  # ch_idx -> (port_name, dev_idx, ch_in_dev, stuck_value)

    def _on_config_received(self, port_configs):
        """Rebuild UI when server sends its actual port configuration."""
        new_spi_ports = [
            SPIPortConfig(name=pc['name'], num_devices=pc['num_devices'])
            for pc in port_configs
        ]

        # Skip rebuild if config hasn't changed
        if (len(new_spi_ports) == len(self.spi_ports) and
            all(a.name == b.name and a.num_devices == b.num_devices
                for a, b in zip(new_spi_ports, self.spi_ports))):
            return

        print(f"[viz] Server config: {[(p.name, p.num_devices) for p in new_spi_ports]}")
        self.spi_ports = new_spi_ports

        # Drain sample queue — old data has wrong channel mapping
        while not self.sample_queue.empty():
            try:
                self.sample_queue.get_nowait()
            except Empty:
                break

        self._init_data_state()
        self._rebuild_selectors()

        # Start CSV writer with the new port config
        self._start_csv_writer(port_configs)

    def _start_csv_writer(self, port_configs):
        """Start (or restart) the CSV writer with the given port configuration."""
        # Stop any existing writer (e.g. on reconnect with new config)
        if self.csv_writer is not None:
            self.csv_writer.stop()
            self.csv_writer = None

        if not self.csv_enabled:
            return

        self.csv_writer = CSVWriterThread(port_configs, csv_dir=self.csv_dir)
        self.csv_writer.start()

        # Tell network thread about the new writer
        self.network_thread.csv_writer = self.csv_writer

    # ------------------------------------------------------------------
    # UI SETUP
    # ------------------------------------------------------------------

    def setup_ui(self):
        self.setWindowTitle('ADS1299 Signal Visualizer')
        self.setStyleSheet(STYLESHEET)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # --- Top bar: status + notch toggle ---
        top_bar = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel('Connecting...')
        self.status.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 11pt; padding: 5px;"
        )
        top_bar.addWidget(self.status, stretch=1)

        self.notch50_btn = QtWidgets.QPushButton('50Hz Notch: OFF')
        self.notch50_btn.setStyleSheet(
            'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
        )
        self.notch50_btn.clicked.connect(self.toggle_notch50)
        top_bar.addWidget(self.notch50_btn)

        self.notch60_btn = QtWidgets.QPushButton('60Hz Notch: ON')
        self.notch60_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;'
        )
        self.notch60_btn.clicked.connect(self.toggle_notch60)
        top_bar.addWidget(self.notch60_btn)

        self.notch120_btn = QtWidgets.QPushButton('120Hz Notch: OFF')
        self.notch120_btn.setStyleSheet(
            'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
        )
        self.notch120_btn.clicked.connect(self.toggle_notch120)
        top_bar.addWidget(self.notch120_btn)

        self.lpf_btn = QtWidgets.QPushButton('50Hz LPF: OFF')
        self.lpf_btn.setStyleSheet(
            'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
        )
        self.lpf_btn.clicked.connect(self.toggle_lpf)
        top_bar.addWidget(self.lpf_btn)

        self.hpf_btn = QtWidgets.QPushButton('1Hz HPF: OFF')
        self.hpf_btn.setStyleSheet(
            'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
        )
        self.hpf_btn.clicked.connect(self.toggle_hpf)
        top_bar.addWidget(self.hpf_btn)

        self.yscale_btn = QtWidgets.QPushButton('Y: \u00b15000')
        self.yscale_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;'
        )
        self.yscale_btn.clicked.connect(self.toggle_yscale)
        top_bar.addWidget(self.yscale_btn)

        self.yrange_input = QtWidgets.QLineEdit('200')
        self.yrange_input.setFixedWidth(60)
        self.yrange_input.setStyleSheet(
            'background-color: #2a2a2a; color: #e0e0e0; border: 1px solid #444; '
            'border-radius: 4px; padding: 4px; font-size: 10pt;'
        )
        self.yrange_input.returnPressed.connect(self._apply_yrange)
        top_bar.addWidget(self.yrange_input)

        self.cmf_btn = QtWidgets.QPushButton('CMF: OFF')
        self.cmf_btn.setStyleSheet(
            'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
        )
        self.cmf_btn.clicked.connect(self.toggle_cmf)
        top_bar.addWidget(self.cmf_btn)

        gain_label = QtWidgets.QLabel('Gain:')
        gain_label.setStyleSheet(f"color: {TEXT_COLOR}; font-size: 10pt;")
        top_bar.addWidget(gain_label)

        self.gain_combo = QtWidgets.QComboBox()
        self.gain_combo.addItems(['1', '2', '4', '6', '8', '12', '24'])
        self.gain_combo.setCurrentText(str(self.gain))
        self.gain_combo.setStyleSheet(
            'background-color: #2a2a2a; color: #e0e0e0; border: 1px solid #444; '
            'border-radius: 4px; padding: 4px 8px; font-size: 10pt; '
            'selection-background-color: #00aa55;'
        )
        self.gain_combo.setFixedWidth(60)
        self.gain_combo.currentTextChanged.connect(self._on_gain_changed)
        top_bar.addWidget(self.gain_combo)

        layout.addLayout(top_bar)

        # --- Time window slider ---
        time_bar = QtWidgets.QHBoxLayout()
        time_bar.setSpacing(4)

        time_label = QtWidgets.QLabel('Window:')
        time_label.setStyleSheet(f"color: {TEXT_COLOR}; font-size: 10pt;")
        time_label.setFixedWidth(55)
        time_bar.addWidget(time_label)

        self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.time_slider.setMinimum(1)    # 1 second
        self.time_slider.setMaximum(30)   # 30 seconds
        self.time_slider.setValue(int(self.window_seconds))
        self.time_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.time_slider.setTickInterval(5)
        self.time_slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                background: #2a2a2a;
                height: 6px;
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {ACCENT_COLOR};
                width: 14px;
                margin: -5px 0;
                border-radius: 7px;
            }}
            QSlider::sub-page:horizontal {{
                background: #00aa55;
                border-radius: 3px;
            }}
        """)
        self.time_slider.valueChanged.connect(self._on_time_window_changed)
        time_bar.addWidget(self.time_slider, stretch=1)

        self.time_value_label = QtWidgets.QLabel(f'{int(self.window_seconds)}s')
        self.time_value_label.setStyleSheet(f"color: {ACCENT_COLOR}; font-size: 10pt; font-weight: bold;")
        self.time_value_label.setFixedWidth(30)
        time_bar.addWidget(self.time_value_label)

        layout.addLayout(time_bar)

        # --- Stuck channel warning banner (hidden by default) ---
        self.stuck_banner = QtWidgets.QLabel('')
        self.stuck_banner.setStyleSheet(
            'background-color: #440000; color: #ff4444; font-size: 11pt; '
            'font-weight: bold; padding: 8px; border: 2px solid #ff0000; '
            'border-radius: 4px;'
        )
        self.stuck_banner.setVisible(False)
        self.stuck_banner.setWordWrap(True)
        layout.addWidget(self.stuck_banner)

        # --- Port buttons row ---
        self._port_buttons_container = QtWidgets.QWidget()
        self._port_buttons_layout = QtWidgets.QHBoxLayout()
        self._port_buttons_layout.setSpacing(4)
        self._port_buttons_layout.setContentsMargins(0, 0, 0, 0)
        self._port_buttons_container.setLayout(self._port_buttons_layout)
        layout.addWidget(self._port_buttons_container)

        # --- Device buttons row ---
        self._device_buttons_container = QtWidgets.QWidget()
        self._device_buttons_layout = QtWidgets.QHBoxLayout()
        self._device_buttons_layout.setSpacing(4)
        self._device_buttons_layout.setContentsMargins(0, 0, 0, 0)
        self._device_buttons_container.setLayout(self._device_buttons_layout)
        self._device_buttons_container.setVisible(False)
        layout.addWidget(self._device_buttons_container)

        # --- 8 channel plots (always exist, data source changes) ---
        plots_widget = QtWidgets.QWidget()
        plots_layout = QtWidgets.QVBoxLayout()
        plots_layout.setContentsMargins(0, 2, 0, 0)
        plots_layout.setSpacing(1)
        plots_widget.setLayout(plots_layout)

        for i in range(8):
            color = NEON_COLORS[i % len(NEON_COLORS)]

            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            row.setContentsMargins(0, 0, 0, 0)

            label = QtWidgets.QLabel(f"Ch{i+1}")
            label.setFixedWidth(36)
            label.setStyleSheet(
                f"color: {color}; font-weight: bold; font-size: 9pt;"
            )
            row.addWidget(label)
            self._ch_labels.append(label)

            pw = pg.PlotWidget()
            pw.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Expanding,
            )
            pw.setBackground(PLOT_BG)
            pw.showGrid(x=True, y=True, alpha=0.2)
            pw.getAxis('left').setPen(pg.mkPen(color='#555'))
            pw.getAxis('left').setTextPen(pg.mkPen(color='#888'))
            pw.getAxis('left').setWidth(50)
            # Hide bottom axis on channels 1-7; only channel 8 shows time axis
            if i < 7:
                pw.getAxis('bottom').setHeight(0)
                pw.getAxis('bottom').setStyle(showValues=False)
            else:
                pw.getAxis('bottom').setPen(pg.mkPen(color='#555'))
                pw.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
            pw.setXRange(0, self.window_seconds)
            pw.setYRange(-self.y_fixed_range, self.y_fixed_range)
            pw.setClipToView(True)
            pw.setDownsampling(mode='peak')

            curve = pw.plot(pen=pg.mkPen(color=color, width=1))
            self._plot_widgets.append(pw)
            self._plot_curves.append(curve)

            row.addWidget(pw, stretch=1)
            plots_layout.addLayout(row, stretch=1)

        layout.addWidget(plots_widget, stretch=1)

        # Build port buttons for initial config (may be empty)
        self._build_selectors()

    # ------------------------------------------------------------------
    # PORT/DEVICE SELECTOR MANAGEMENT
    # ------------------------------------------------------------------

    def _compute_display_order(self):
        """Compute display order indices and channel offsets for current spi_ports."""
        # Channel offsets in data buffer (original order)
        self._port_ch_offsets = []
        ch_offset = 0
        for pc in self.spi_ports:
            self._port_ch_offsets.append(ch_offset)
            ch_offset += pc.num_channels

        # Build name -> original index lookup
        port_lookup = {}
        for idx, pc in enumerate(self.spi_ports):
            port_lookup[pc.name] = idx

        # Display order: DISPLAY_ORDER first, then any remaining
        ordered_orig_indices = []
        for name in self.DISPLAY_ORDER:
            if name in port_lookup:
                ordered_orig_indices.append(port_lookup[name])
        for idx, pc in enumerate(self.spi_ports):
            if idx not in ordered_orig_indices:
                ordered_orig_indices.append(idx)

        self._display_order_indices = ordered_orig_indices

    def _build_selectors(self):
        """Create port buttons for the current spi_ports config."""
        self._compute_display_order()
        self._port_buttons.clear()
        self._port_button_colors.clear()
        self._port_stuck_state.clear()

        for display_idx, orig_idx in enumerate(self._display_order_indices):
            pc = self.spi_ports[orig_idx]
            color = PORT_COLORS[orig_idx % len(PORT_COLORS)]
            self._port_button_colors.append(color)
            self._port_stuck_state.append(False)

            btn = QtWidgets.QPushButton(pc.name)
            btn.setStyleSheet(self._port_btn_style(color, active=False))
            btn.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            btn.setMinimumHeight(36)
            btn.clicked.connect(
                lambda checked, di=display_idx: self._select_port(di)
            )
            self._port_buttons.append(btn)
            self._port_buttons_layout.addWidget(btn)

        # Auto-select the first port if any exist
        if self._display_order_indices:
            self._select_port(0)

    def _rebuild_selectors(self):
        """Remove old port/device buttons and recreate for updated config."""
        # Clear port buttons
        for btn in self._port_buttons:
            self._port_buttons_layout.removeWidget(btn)
            btn.setParent(None)
            btn.deleteLater()
        self._port_buttons.clear()
        self._port_button_colors.clear()
        self._port_stuck_state.clear()

        # Clear device buttons
        self._clear_device_buttons()
        self._device_buttons_container.setVisible(False)

        # Reset selection
        self._selected_port_display_idx = None
        self._selected_device_idx = None

        # Rebuild
        self._build_selectors()

        # Update status
        total_dev = sum(p.num_devices for p in self.spi_ports)
        total_ch = sum(p.num_channels for p in self.spi_ports)
        self.status.setText(
            f"Configured: {len(self.spi_ports)} ports, "
            f"{total_dev} devices, {total_ch} channels"
        )

    def _clear_device_buttons(self):
        """Remove all device buttons from the device row."""
        for btn in self._device_buttons:
            self._device_buttons_layout.removeWidget(btn)
            btn.setParent(None)
            btn.deleteLater()
        self._device_buttons.clear()
        self._device_stuck_state.clear()

    def _select_port(self, display_idx):
        """Handle port button click. Highlight the selected port, update device buttons."""
        if display_idx == self._selected_port_display_idx:
            return  # already selected

        # Update button styles: deactivate old, activate new
        if self._selected_port_display_idx is not None:
            old_idx = self._selected_port_display_idx
            old_color = self._port_button_colors[old_idx]
            if self._port_stuck_state[old_idx]:
                self._port_buttons[old_idx].setStyleSheet(
                    self._port_btn_stuck_style(active=False)
                )
            else:
                self._port_buttons[old_idx].setStyleSheet(
                    self._port_btn_style(old_color, active=False)
                )

        self._selected_port_display_idx = display_idx
        new_color = self._port_button_colors[display_idx]
        if self._port_stuck_state[display_idx]:
            self._port_buttons[display_idx].setStyleSheet(
                self._port_btn_stuck_style(active=True)
            )
        else:
            self._port_buttons[display_idx].setStyleSheet(
                self._port_btn_style(new_color, active=True)
            )

        # Rebuild device buttons for this port
        self._update_device_buttons()

    def _update_device_buttons(self):
        """Rebuild device buttons row for the currently selected port."""
        self._clear_device_buttons()

        if self._selected_port_display_idx is None:
            self._device_buttons_container.setVisible(False)
            return

        orig_idx = self._display_order_indices[self._selected_port_display_idx]
        pc = self.spi_ports[orig_idx]
        color = self._port_button_colors[self._selected_port_display_idx]

        for d in range(pc.num_devices):
            self._device_stuck_state.append(False)
            btn = QtWidgets.QPushButton(f"Dev {d+1}")
            btn.setStyleSheet(self._dev_btn_style(color, active=False))
            btn.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            btn.setMinimumHeight(32)
            btn.clicked.connect(
                lambda checked, di=d: self._select_device(di)
            )
            self._device_buttons.append(btn)
            self._device_buttons_layout.addWidget(btn)

        self._device_buttons_container.setVisible(True)

        # Auto-select device 0
        self._selected_device_idx = None
        self._select_device(0)

    def _select_device(self, dev_idx):
        """Handle device button click. Highlight the selected device, update plots."""
        if self._selected_port_display_idx is None:
            return

        color = self._port_button_colors[self._selected_port_display_idx]

        # Deactivate old device button
        if self._selected_device_idx is not None and self._selected_device_idx < len(self._device_buttons):
            old_idx = self._selected_device_idx
            if self._device_stuck_state[old_idx]:
                self._device_buttons[old_idx].setStyleSheet(
                    self._dev_btn_stuck_style()
                )
            else:
                self._device_buttons[old_idx].setStyleSheet(
                    self._dev_btn_style(color, active=False)
                )

        self._selected_device_idx = dev_idx

        # Activate new device button
        if dev_idx < len(self._device_buttons):
            if self._device_stuck_state[dev_idx]:
                self._device_buttons[dev_idx].setStyleSheet(
                    self._dev_btn_stuck_style()
                )
            else:
                self._device_buttons[dev_idx].setStyleSheet(
                    self._dev_btn_style(color, active=True)
                )

        # Clear plot data immediately so stale traces don't linger
        for curve in self._plot_curves:
            curve.setData([], [])

    # ------------------------------------------------------------------
    # BUTTON STYLE HELPERS
    # ------------------------------------------------------------------

    def _port_btn_style(self, color, active):
        if active:
            return f"""
                QPushButton {{
                    background-color: {color};
                    color: #0d0d0d;
                    border: 2px solid {color};
                    border-radius: 6px;
                    padding: 6px 10px;
                    font-size: 11pt;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: {color};
                }}
            """
        else:
            return f"""
                QPushButton {{
                    background-color: #1a1a1a;
                    color: {color};
                    border: 2px solid {color};
                    border-radius: 6px;
                    padding: 6px 10px;
                    font-size: 11pt;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: #2a2a2a;
                }}
            """

    def _port_btn_stuck_style(self, active):
        if active:
            return """
                QPushButton {
                    background-color: #880000;
                    color: #ff4444;
                    border: 2px solid #ff0000;
                    border-radius: 6px;
                    padding: 6px 10px;
                    font-size: 11pt;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #990000;
                }
            """
        else:
            return """
                QPushButton {
                    background-color: #440000;
                    color: #ff4444;
                    border: 2px solid #ff0000;
                    border-radius: 6px;
                    padding: 6px 10px;
                    font-size: 11pt;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #550000;
                }
            """

    def _dev_btn_style(self, color, active):
        if active:
            return f"""
                QPushButton {{
                    background-color: {color};
                    color: #0d0d0d;
                    border: 1px solid {color};
                    border-radius: 4px;
                    padding: 4px 8px;
                    font-size: 10pt;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: {color};
                }}
            """
        else:
            return f"""
                QPushButton {{
                    background-color: #1a1a1a;
                    color: {color};
                    border: 1px solid {color};
                    border-radius: 4px;
                    padding: 4px 8px;
                    font-size: 10pt;
                }}
                QPushButton:hover {{
                    background-color: #2a2a2a;
                }}
            """

    def _dev_btn_stuck_style(self):
        return """
            QPushButton {
                background-color: #440000;
                color: #ff4444;
                border: 2px solid #ff0000;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 10pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #550000;
            }
        """

    # ------------------------------------------------------------------
    # STUCK CHANNEL INDICATOR UPDATES
    # ------------------------------------------------------------------

    def _update_stuck_indicators(self, stuck_channels):
        """Update port and device button styling based on stuck channel set.

        Port button turns red if ANY channel in that port is stuck.
        Device button turns red if ANY of its 8 channels are stuck.
        Only calls setStyleSheet when state actually changes.
        """
        stuck_set = set(stuck_channels.keys())

        # Update port buttons
        for display_idx, orig_idx in enumerate(self._display_order_indices):
            pc = self.spi_ports[orig_idx]
            port_ch_start = self._port_ch_offsets[orig_idx]
            port_ch_end = port_ch_start + pc.num_channels

            port_stuck = any(
                ch_idx in stuck_set
                for ch_idx in range(port_ch_start, port_ch_end)
            )
            was_stuck = self._port_stuck_state[display_idx]
            if port_stuck == was_stuck:
                continue
            self._port_stuck_state[display_idx] = port_stuck

            is_active = (display_idx == self._selected_port_display_idx)
            color = self._port_button_colors[display_idx]
            if port_stuck:
                self._port_buttons[display_idx].setStyleSheet(
                    self._port_btn_stuck_style(active=is_active)
                )
            else:
                self._port_buttons[display_idx].setStyleSheet(
                    self._port_btn_style(color, active=is_active)
                )

        # Update device buttons (only for currently selected port)
        if self._selected_port_display_idx is not None and self._device_buttons:
            orig_idx = self._display_order_indices[self._selected_port_display_idx]
            port_ch_start = self._port_ch_offsets[orig_idx]
            color = self._port_button_colors[self._selected_port_display_idx]

            for d in range(len(self._device_buttons)):
                dev_ch_start = port_ch_start + d * 8
                dev_stuck = any(
                    ch_idx in stuck_set
                    for ch_idx in range(dev_ch_start, dev_ch_start + 8)
                )
                if d >= len(self._device_stuck_state):
                    break
                was_stuck = self._device_stuck_state[d]
                if dev_stuck == was_stuck:
                    continue
                self._device_stuck_state[d] = dev_stuck

                is_active = (d == self._selected_device_idx)
                if dev_stuck:
                    self._device_buttons[d].setStyleSheet(
                        self._dev_btn_stuck_style()
                    )
                else:
                    self._device_buttons[d].setStyleSheet(
                        self._dev_btn_style(color, active=is_active)
                    )

    # ------------------------------------------------------------------
    # DATA PROCESSING (unchanged from original)
    # ------------------------------------------------------------------

    def process_batch(self, samples):
        """Process multiple samples at once with vectorized notch filtering.

        Instead of calling lfilter 224x50 = 11,200 times (once per channel per
        sample), this calls it 224 times total, each processing the full batch.
        """
        n = len(samples)
        if n == 0:
            return
        num_ch = self.total_channels
        if num_ch == 0:
            return

        if self.start_time is None:
            self.start_time = time.time()

        # Build numpy arrays from sample dicts and convert to microvolts
        timestamps = np.array([s['timestamp'] for s in samples])
        raw = np.zeros((n, num_ch))
        for i, s in enumerate(samples):
            ch = s['channels']
            ch_count = min(len(ch), num_ch)
            raw[i, :ch_count] = ch[:ch_count]
        raw *= self.lsb_uv  # Convert ADC counts to microvolts

        # Vectorized filters: one lfilter call per channel, n samples each
        if self.notch50_on:
            for c in range(num_ch):
                raw[:, c], self.notch50_states[c] = sig.lfilter(
                    self.notch50_b, self.notch50_a, raw[:, c],
                    zi=self.notch50_states[c]
                )
        if self.notch60_on:
            for c in range(num_ch):
                raw[:, c], self.notch60_states[c] = sig.lfilter(
                    self.notch60_b, self.notch60_a, raw[:, c],
                    zi=self.notch60_states[c]
                )
        if self.notch70_on:
            for c in range(num_ch):
                raw[:, c], self.notch70_states[c] = sig.lfilter(
                    self.notch70_b, self.notch70_a, raw[:, c],
                    zi=self.notch70_states[c]
                )
        if self.notch80_on:
            for c in range(num_ch):
                raw[:, c], self.notch80_states[c] = sig.lfilter(
                    self.notch80_b, self.notch80_a, raw[:, c],
                    zi=self.notch80_states[c]
                )
        if self.notch120_on:
            for c in range(num_ch):
                raw[:, c], self.notch120_states[c] = sig.lfilter(
                    self.notch120_b, self.notch120_a, raw[:, c],
                    zi=self.notch120_states[c]
                )
        if self.lpf_on:
            for c in range(num_ch):
                raw[:, c], self.lpf_states[c] = sig.sosfilt(
                    self.lpf_sos, raw[:, c],
                    zi=self.lpf_states[c]
                )
        if self.hpf_on:
            for c in range(num_ch):
                raw[:, c], self.hpf_states[c] = sig.sosfilt(
                    self.hpf_sos, raw[:, c],
                    zi=self.hpf_states[c]
                )
        if self.cmf_on:
            ch_offset = 0
            for port in self.spi_ports:
                for dev in range(port.num_devices):
                    dev_start = ch_offset + dev * 8
                    dev_end = dev_start + 8
                    if dev_end <= num_ch:
                        dev_mean = raw[:, dev_start:dev_end].mean(
                            axis=1, keepdims=True
                        )
                        raw[:, dev_start:dev_end] -= dev_mean
                ch_offset += port.num_channels

        # Write batch to ring buffer, handling wrap-around
        idx = self.buf_idx
        buf_cap = self.max_samples * 2
        space = buf_cap - idx

        if n <= space:
            # Entire batch fits without rollover
            self.time_buf[:num_ch, idx:idx + n] = timestamps  # broadcasts
            self.data_buf[:num_ch, idx:idx + n] = raw.T
            self.buf_idx += n
        else:
            # Write what fits, rollover, write the rest
            if space > 0:
                self.time_buf[:num_ch, idx:idx + space] = timestamps[:space]
                self.data_buf[:num_ch, idx:idx + space] = raw[:space].T
            self.time_buf[:, :self.max_samples] = (
                self.time_buf[:, self.max_samples:]
            )
            self.data_buf[:, :self.max_samples] = (
                self.data_buf[:, self.max_samples:]
            )
            self.buf_idx = self.max_samples
            remaining = n - space
            if remaining > 0:
                idx2 = self.buf_idx
                self.time_buf[:num_ch, idx2:idx2 + remaining] = (
                    timestamps[space:]
                )
                self.data_buf[:num_ch, idx2:idx2 + remaining] = (
                    raw[space:].T
                )
                self.buf_idx += remaining

        self.buf_count = min(self.buf_count + n, self.max_samples)
        self.sample_count += n

        # Stuck channel detection (vectorized across all channels)
        batch_min = raw.min(axis=0)
        batch_max = raw.max(axis=0)
        constant_mask = (batch_min == batch_max)  # channels with all-same values

        same_as_prev = constant_mask & (raw[0, :] == self.stuck_last_value)
        self.stuck_count[same_as_prev] += n

        new_constant = constant_mask & ~same_as_prev
        self.stuck_count[new_constant] = n

        self.stuck_count[~constant_mask] = 0

        self.stuck_last_value = raw[-1, :].copy()

        # Rebuild stuck_channels dict ~1/sec (every 250 samples)
        if self.sample_count % 250 < n:
            self.stuck_channels = {}
            stuck_set = set(np.where(self.stuck_count >= self.stuck_threshold)[0])
            if stuck_set:
                ch_offset = 0
                for port in self.spi_ports:
                    for dev_idx in range(port.num_devices):
                        for ch_in_dev in range(8):
                            ch_idx = ch_offset + dev_idx * 8 + ch_in_dev
                            if ch_idx in stuck_set:
                                self.stuck_channels[ch_idx] = (
                                    port.name, dev_idx, ch_in_dev,
                                    float(self.stuck_last_value[ch_idx])
                                )
                    ch_offset += port.num_channels

        if self.sample_count % 250 < n:  # crossed a 250-sample boundary
            elapsed = time.time() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            port_summary = " | ".join(
                f"{p.name}({p.num_channels}ch)" for p in self.spi_ports
            )
            csv_info = ""
            if self.csv_writer is not None:
                csv_info = f" | CSV: {self.csv_writer.total_written}"
            self.status.setText(
                f'{port_summary} | Rate: {rate:.1f} Hz | '
                f'Samples: {self.sample_count}{csv_info}'
            )

    # ------------------------------------------------------------------
    # PLOT UPDATE
    # ------------------------------------------------------------------

    def update_plots(self):
        """Buffered playback: absorb WiFi jitter, release samples at steady rate."""
        try:
            # Drain thread-safe queue into main-thread deque
            for _ in range(500):
                try:
                    self.pending_samples.append(self.sample_queue.get_nowait())
                except Empty:
                    break

            # Initialize playback clock on first sample
            if self.pending_samples and self.playback_wall_start is None:
                self.playback_wall_start = time.time()
                self.playback_server_start = self.pending_samples[0]['timestamp']

            if self.playback_wall_start is None:
                return

            # Collect all due samples into a batch, then process at once
            playback_time = (
                self.playback_server_start
                + (time.time() - self.playback_wall_start)
                - self.buffer_delay
            )
            count_before = self.sample_count
            batch = []
            while self.pending_samples and len(batch) < 250:
                if self.pending_samples[0]['timestamp'] <= playback_time:
                    batch.append(self.pending_samples.popleft())
                else:
                    break
            if batch:
                self.process_batch(batch)

            if self.sample_count == count_before or self.buf_count == 0:
                return

            self.frame_count += 1
            rescale_y = (self.frame_count % 3 == 0)

            # Compute X range from channel 0 timestamps
            start = self.buf_idx - self.buf_count
            end = self.buf_idx
            t0 = self.time_buf[0, start:end]
            if len(t0) == 0:
                return
            latest = t0[-1]
            if latest <= self.window_seconds:
                x_min, x_max = 0, self.window_seconds
            else:
                x_min, x_max = latest - self.window_seconds, latest

            # Update the 8 plots for the currently selected port + device
            self._update_visible_plots(x_min, x_max, rescale_y)

            # Update stuck channel banner
            if self.stuck_channels:
                groups = {}
                for ch_idx, (port_name, dev_idx, ch_in_dev, val) in self.stuck_channels.items():
                    key = (port_name, dev_idx)
                    if key not in groups:
                        groups[key] = []
                    groups[key].append((ch_in_dev, val))

                parts = []
                for (port_name, dev_idx), channels in sorted(groups.items()):
                    chs = sorted(channels, key=lambda x: x[0])
                    val = chs[0][1]
                    ch_nums = [c[0] + 1 for c in chs]
                    parts.append(f"{port_name}/Dev{dev_idx+1} Ch{ch_nums} = {val:.0f}")

                self.stuck_banner.setText(
                    f"STUCK CHANNELS ({len(self.stuck_channels)}): {' | '.join(parts)}"
                )
                self.stuck_banner.setVisible(True)

                if rescale_y:
                    self._update_stuck_indicators(self.stuck_channels)
            else:
                self.stuck_banner.setVisible(False)
                if rescale_y:
                    self._update_stuck_indicators({})

        except Exception as e:
            print(f"[viz] update error: {e}")
            traceback.print_exc()

    def _update_visible_plots(self, x_min, x_max, rescale_y):
        """Update the 8 plot widgets with data for the selected port + device."""
        if self._selected_port_display_idx is None or self._selected_device_idx is None:
            return

        orig_idx = self._display_order_indices[self._selected_port_display_idx]
        port_ch_start = self._port_ch_offsets[orig_idx]
        dev_ch_start = port_ch_start + self._selected_device_idx * 8

        buf_start = self.buf_idx - self.buf_count
        buf_end = self.buf_idx
        if buf_start < 0 or buf_end <= buf_start:
            return

        for i in range(8):
            ch_idx = dev_ch_start + i
            if ch_idx >= self.total_channels:
                self._plot_curves[i].setData([], [])
                continue

            t = self.time_buf[ch_idx, buf_start:buf_end]
            d = self.data_buf[ch_idx, buf_start:buf_end]
            self._plot_curves[i].setData(t, d)

            self._plot_widgets[i].setXRange(x_min, x_max, padding=0)

            if rescale_y and len(d) > 0:
                if self.y_autoscale:
                    y_min, y_max = float(d.min()), float(d.max())
                    margin = max((y_max - y_min) * 0.1, 10)  # uV
                    target_min = y_min - margin
                    target_max = y_max + margin
                    alpha = 0.4
                    self._y_smooth_min[i] += alpha * (target_min - self._y_smooth_min[i])
                    self._y_smooth_max[i] += alpha * (target_max - self._y_smooth_max[i])
                    self._plot_widgets[i].setYRange(
                        self._y_smooth_min[i], self._y_smooth_max[i], padding=0
                    )
                else:
                    self._plot_widgets[i].setYRange(
                        -self.y_fixed_range, self.y_fixed_range, padding=0
                    )

    # ------------------------------------------------------------------
    # NOTCH FILTER TOGGLE
    # ------------------------------------------------------------------

    def _toggle_filter(self, attr, btn, label):
        """Generic toggle helper for notch/lpf buttons."""
        state = not getattr(self, attr)
        setattr(self, attr, state)
        if state:
            btn.setText(f'{label}: ON')
            btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;'
            )
            # Reset filter states on enable
            zi_attr = attr.replace('_on', '_zi_template')
            states_attr = attr.replace('_on', '_states')
            setattr(self, states_attr, [
                getattr(self, zi_attr).copy()
                for _ in range(self.total_channels)
            ])
        else:
            btn.setText(f'{label}: OFF')
            btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
            )

    def toggle_notch50(self):
        self._toggle_filter('notch50_on', self.notch50_btn, '50Hz Notch')

    def toggle_notch60(self):
        self._toggle_filter('notch60_on', self.notch60_btn, '60Hz Notch')

    def toggle_notch70(self):
        self._toggle_filter('notch70_on', self.notch70_btn, '70Hz Notch')

    def toggle_notch80(self):
        self._toggle_filter('notch80_on', self.notch80_btn, '80Hz Notch')

    def toggle_notch120(self):
        self._toggle_filter('notch120_on', self.notch120_btn, '120Hz Notch')

    def toggle_lpf(self):
        self._toggle_filter('lpf_on', self.lpf_btn, '50Hz LPF')

    def toggle_hpf(self):
        self._toggle_filter('hpf_on', self.hpf_btn, '1Hz HPF')

    def toggle_cmf(self):
        self.cmf_on = not self.cmf_on
        if self.cmf_on:
            self.cmf_btn.setText('CMF: ON')
            self.cmf_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;'
            )
        else:
            self.cmf_btn.setText('CMF: OFF')
            self.cmf_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
            )

    def _on_gain_changed(self, text):
        try:
            self.gain = int(text)
            self.lsb_uv = (VREF / (2**23) / self.gain) * 1e6
        except ValueError:
            pass

    def _on_time_window_changed(self, value):
        """Update the display time window from slider."""
        max_allowed = self.max_samples / self.sample_rate
        self.window_seconds = min(float(value), max_allowed)
        self.time_value_label.setText(f'{int(self.window_seconds)}s')

    def toggle_yscale(self):
        self.y_autoscale = not self.y_autoscale
        if self.y_autoscale:
            self.yscale_btn.setText('Y: Auto')
            self.yscale_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
            )
        else:
            self.yscale_btn.setText(f'Y: \u00b1{int(self.y_fixed_range)}')
            self.yscale_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;'
            )
            for pw in self._plot_widgets:
                pw.setYRange(-self.y_fixed_range, self.y_fixed_range, padding=0)

    def _apply_yrange(self):
        try:
            val = int(self.yrange_input.text())
            if val > 0:
                self.y_fixed_range = val
                if not self.y_autoscale:
                    self.yscale_btn.setText(f'Y: \u00b1{val}')
                    for pw in self._plot_widgets:
                        pw.setYRange(-val, val, padding=0)
        except ValueError:
            pass

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_N:
            self.toggle_notch60()
        elif event.key() == QtCore.Qt.Key_L:
            self.toggle_lpf()
        elif event.key() == QtCore.Qt.Key_H:
            self.toggle_hpf()
        elif event.key() == QtCore.Qt.Key_Y:
            self.toggle_yscale()
        elif event.key() == QtCore.Qt.Key_C:
            self.toggle_cmf()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.network_thread.stop()
        self.network_thread.wait()
        if self.csv_writer is not None:
            self.csv_writer.stop()
            self.csv_writer = None
        event.accept()


# ============================================================================
# NETWORK THREAD
# ============================================================================

class NetworkThread(QtCore.QThread):
    """Receives binary LZ4 frames from Controller.py with auto-reconnect."""

    # Emitted when server sends port_config in metadata
    config_received = QtCore.pyqtSignal(list)

    def __init__(self, visualizer, host, port):
        super().__init__()
        self.visualizer = visualizer
        self.host = host
        self.port = port
        self.running = True
        self.buffer = b''
        self.csv_writer = None  # Set by SignalVisualizer when config arrives

    def run(self):
        ip = self.host
        if ip.endswith('.local'):
            ip = socket.gethostbyname(ip)
            print(f"Resolved {self.host} to {ip}")

        attempt = 0
        while self.running:
            attempt += 1
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                print(f"[net] Connecting to {ip}:{self.port} (attempt #{attempt})...")
                sock.connect((ip, self.port))
                sock.settimeout(1.0)
                print(f"[net] Connected")

                self.buffer = b''
                line = self._recv_line(sock)
                metadata = json.loads(line)
                print(f"[net] Metadata: {metadata}")
                attempt = 0

                # Auto-configure ports from server metadata
                if 'port_config' in metadata:
                    self.config_received.emit(metadata['port_config'])

                sample_struct = struct.Struct(metadata['sample_struct'])
                sample_size = sample_struct.size
                header_struct = struct.Struct('<II')

                while self.running:
                    try:
                        header_data = self._recv_exact(sock, 8)
                        payload_size, sample_count = header_struct.unpack(header_data)
                        raw = self._recv_exact(sock, payload_size)
                        for i in range(sample_count):
                            offset = i * sample_size
                            unpacked = sample_struct.unpack_from(raw, offset)
                            sample = {
                                'timestamp': unpacked[0],
                                'sample_number': unpacked[1],
                                'channels': list(unpacked[2:]),
                            }

                            # Feed raw sample to CSV writer (non-blocking)
                            csv_w = self.csv_writer
                            if csv_w is not None:
                                csv_w.push(sample)

                            try:
                                self.visualizer.sample_queue.put_nowait(sample)
                            except Exception:
                                # Queue full — drop oldest
                                try:
                                    self.visualizer.sample_queue.get_nowait()
                                except Empty:
                                    pass
                                try:
                                    self.visualizer.sample_queue.put_nowait(sample)
                                except Exception:
                                    pass
                    except socket.timeout:
                        continue

            except Exception as e:
                if self.running:
                    print(f"[net] Error: {type(e).__name__}: {e} \u2014 reconnecting in 2s")
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass

            if self.running:
                time.sleep(2)

    def _recv_exact(self, sock, n):
        while len(self.buffer) < n:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                if not self.running:
                    raise
                continue
            if not chunk:
                raise ConnectionError("Connection closed")
            self.buffer += chunk
        data = self.buffer[:n]
        self.buffer = self.buffer[n:]
        return data

    def _recv_line(self, sock):
        while True:
            pos = self.buffer.find(b'\n')
            if pos != -1:
                line = self.buffer[:pos]
                self.buffer = self.buffer[pos + 1:]
                return line.decode('utf-8')
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Connection closed")
            self.buffer += chunk

    def stop(self):
        self.running = False


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='ADS1299 signal visualizer with port/device selector',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Port configuration is auto-detected from the server.
Use --spi-ports to override if needed.

Examples:
  uv run simpleviz.py --host 192.168.1.100
  uv run simpleviz.py --host 192.168.1.100 --spi-ports "Port1,3 Port2,5"

SPI Port Format: "Name,NumDevices Name2,NumDevices2 ..."
        """
    )
    parser.add_argument('--host', type=str, required=True,
                        help='Server IP address')
    parser.add_argument('--port', type=int, default=8888,
                        help='Server port (default: 8888)')
    parser.add_argument('--window', type=float, default=5.0,
                        help='Time window in seconds (default: 5.0)')
    parser.add_argument('--spi-ports', type=str, default=None,
                        help='Override port config (default: auto-detect from server)')
    parser.add_argument('--no-csv', action='store_true', default=False,
                        help='Disable client-side CSV recording')
    parser.add_argument('--csv-dir', type=str, default='.',
                        help='Directory for CSV output (default: current directory)')

    args = parser.parse_args()

    # If --spi-ports given, use it; otherwise start empty and auto-detect
    if args.spi_ports:
        try:
            spi_ports = parse_spi_ports(args.spi_ports)
        except ValueError as e:
            print(f"Error parsing SPI ports: {e}")
            sys.exit(1)
    else:
        spi_ports = []

    app = QtWidgets.QApplication(sys.argv)

    viz = SignalVisualizer(
        args.host,
        args.port,
        spi_ports,
        window_seconds=args.window,
        csv_enabled=not args.no_csv,
        csv_dir=args.csv_dir,
    )
    viz.showMaximized()

    print(f"\nADS1299 Signal Visualizer")
    print(f"{'='*50}")
    print(f"Server: {args.host}:{args.port}")
    if spi_ports:
        total_devices = sum(p.num_devices for p in spi_ports)
        total_channels = sum(p.num_channels for p in spi_ports)
        print(f"Ports: {len(spi_ports)} | Devices: {total_devices} | "
              f"Channels: {total_channels}")
        ch_offset = 0
        for p in spi_ports:
            print(f"  {p.name}: {p.num_devices} dev, {p.num_channels} ch "
                  f"(ch {ch_offset+1}-{ch_offset+p.num_channels})")
            ch_offset += p.num_channels
    else:
        print("Port config: auto-detect from server")
    csv_status = f"CSV: OFF" if args.no_csv else f"CSV: ON (dir: {os.path.abspath(args.csv_dir)})"
    print(f"{csv_status}")
    print(f"{'='*50}\n")

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
