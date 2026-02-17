#!/usr/bin/env uv run script
# /// script
# dependencies = [
#   "numpy",
#   "PyQt5",
#   "pyqtgraph",
#   "scipy",
#   "lz4",
# ]
# ///
"""
ADS1299 Signal Visualizer (Server-Timed, Binary LZ4)
====================================================

Streams binary LZ4-compressed EEG data from Controller.py and displays
channels organized by SPI port in side-by-side columns.

Port configuration is auto-detected from the server metadata.
Use --spi-ports to manually override if needed.

Usage:
    uv run simpleviz.py --host <pi-ip>
    uv run simpleviz.py --host <pi-ip> --spi-ports "Port1,2 Port2,3"
"""

import sys
import socket
import json
import argparse
import time
import struct
import traceback
import lz4.frame
from collections import deque
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
QScrollArea {{
    border: none;
    background-color: {DARK_BG};
}}
QScrollBar:vertical {{
    background-color: #1a1a1a;
    width: 12px;
    border-radius: 6px;
}}
QScrollBar::handle:vertical {{
    background-color: #444;
    border-radius: 6px;
    min-height: 30px;
}}
QScrollBar::handle:vertical:hover {{
    background-color: {ACCENT_COLOR};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QScrollBar:horizontal {{
    background-color: #1a1a1a;
    height: 12px;
    border-radius: 6px;
}}
QScrollBar::handle:horizontal {{
    background-color: #444;
    border-radius: 6px;
    min-width: 30px;
}}
QScrollBar::handle:horizontal:hover {{
    background-color: {ACCENT_COLOR};
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
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
# PORT COLUMN — button on top, plots drop down below
# ============================================================================

class PortColumn(QtWidgets.QWidget):
    """A single port column: port header, device buttons, and channel plots."""

    def __init__(self, port_config, start_ch_idx, color, parent_viz):
        super().__init__()
        self.port_config = port_config
        self.start_ch_idx = start_ch_idx
        self.color = color
        self.parent_viz = parent_viz
        self.active_device: Optional[int] = None  # Currently shown device (0-indexed), None = collapsed

        # Per-device state: plots/curves created lazily
        self.num_devices = port_config.num_devices
        self.device_plots: List[List[pg.PlotWidget]] = [[] for _ in range(self.num_devices)]
        self.device_curves: List[list] = [[] for _ in range(self.num_devices)]
        self.device_created: List[bool] = [False] * self.num_devices
        self.device_containers: List[QtWidgets.QWidget] = []
        self.device_buttons: List[QtWidgets.QPushButton] = []

        self.setup_ui()

    def setup_ui(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(2, 0, 2, 0)
        layout.setSpacing(2)

        # Port header (non-toggle, just a label)
        header = QtWidgets.QLabel(self.port_config.name)
        header.setAlignment(QtCore.Qt.AlignCenter)
        header.setStyleSheet(f"""
            QLabel {{
                background-color: #1a1a1a;
                color: {self.color};
                border: 2px solid {self.color};
                border-radius: 6px;
                padding: 8px 4px;
                font-size: 11pt;
                font-weight: bold;
            }}
        """)
        layout.addWidget(header)

        # Device buttons row
        dev_row = QtWidgets.QHBoxLayout()
        dev_row.setSpacing(3)
        for d in range(self.num_devices):
            btn = QtWidgets.QPushButton(f"Dev {d+1}")
            btn.setStyleSheet(self._dev_btn_style(active=False))
            btn.clicked.connect(lambda checked, idx=d: self.select_device(idx))
            dev_row.addWidget(btn)
            self.device_buttons.append(btn)
        layout.addLayout(dev_row)

        # One container per device (only one visible at a time)
        for d in range(self.num_devices):
            container = QtWidgets.QWidget()
            container_layout = QtWidgets.QVBoxLayout()
            container_layout.setContentsMargins(2, 4, 2, 4)
            container_layout.setSpacing(2)
            container.setLayout(container_layout)
            container.setVisible(False)
            self.device_containers.append(container)
            layout.addWidget(container)

        self.setLayout(layout)

    def _create_device_plots(self, dev_idx):
        """Create 8 PlotWidgets for a device on first expand."""
        if self.device_created[dev_idx]:
            return
        container_layout = self.device_containers[dev_idx].layout()
        for i in range(8):
            color = NEON_COLORS[i % len(NEON_COLORS)]

            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            row.setContentsMargins(0, 0, 0, 0)

            label = QtWidgets.QLabel(f"Ch{i+1}")
            label.setFixedWidth(35)
            label.setStyleSheet(
                f"color: {color}; font-weight: bold; font-size: 9pt;"
            )
            row.addWidget(label)

            pw = pg.PlotWidget()
            pw.setMinimumHeight(70)
            pw.setMaximumHeight(90)
            pw.setBackground(PLOT_BG)
            pw.showGrid(x=True, y=True, alpha=0.2)
            pw.getAxis('left').setPen(pg.mkPen(color='#555'))
            pw.getAxis('bottom').setPen(pg.mkPen(color='#555'))
            pw.getAxis('left').setTextPen(pg.mkPen(color='#888'))
            pw.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
            pw.setXRange(0, self.parent_viz.window_seconds)
            pw.setYRange(-500000, 500000)
            pw.setClipToView(True)
            pw.setDownsampling(mode='peak')

            curve = pw.plot(pen=pg.mkPen(color=color, width=1))
            self.device_plots[dev_idx].append(pw)
            self.device_curves[dev_idx].append(curve)

            row.addWidget(pw)
            container_layout.addLayout(row)

        self.device_created[dev_idx] = True

    def _dev_btn_style(self, active):
        if active:
            return f"""
                QPushButton {{
                    background-color: {self.color};
                    color: #0d0d0d;
                    border: 1px solid {self.color};
                    border-radius: 4px;
                    padding: 4px 2px;
                    font-size: 9pt;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: {self.color};
                }}
            """
        else:
            return f"""
                QPushButton {{
                    background-color: #1a1a1a;
                    color: {self.color};
                    border: 1px solid {self.color};
                    border-radius: 4px;
                    padding: 4px 2px;
                    font-size: 9pt;
                }}
                QPushButton:hover {{
                    background-color: #2a2a2a;
                }}
            """

    def select_device(self, dev_idx):
        """Toggle a device's plots. Only one device shown at a time per port."""
        if self.active_device == dev_idx:
            # Clicking active device hides it
            self.device_containers[dev_idx].setVisible(False)
            self.device_buttons[dev_idx].setStyleSheet(self._dev_btn_style(False))
            self.active_device = None
            return

        # Hide previously active device
        if self.active_device is not None:
            self.device_containers[self.active_device].setVisible(False)
            self.device_buttons[self.active_device].setStyleSheet(
                self._dev_btn_style(False)
            )

        # Show new device
        self._create_device_plots(dev_idx)
        self.device_containers[dev_idx].setVisible(True)
        self.device_buttons[dev_idx].setStyleSheet(self._dev_btn_style(True))
        self.active_device = dev_idx

    def update_plots(self, x_min, x_max, rescale_y):
        if self.active_device is None:
            return
        dev = self.active_device
        if not self.device_created[dev]:
            return

        viz = self.parent_viz
        start = viz.buf_idx - viz.buf_count
        end = viz.buf_idx
        if start < 0 or end <= start:
            return

        # Channel offset for this device within all_channels
        dev_ch_start = self.start_ch_idx + dev * 8

        for i, curve in enumerate(self.device_curves[dev]):
            ch_idx = dev_ch_start + i
            if ch_idx >= viz.total_channels:
                break

            t = viz.time_buf[ch_idx, start:end]
            d = viz.data_buf[ch_idx, start:end]
            curve.setData(t, d)

            self.device_plots[dev][i].setXRange(x_min, x_max, padding=0)

            if rescale_y and len(d) > 0:
                y_min, y_max = float(d.min()), float(d.max())
                margin = max((y_max - y_min) * 0.1, 10000)
                self.device_plots[dev][i].setYRange(
                    y_min - margin, y_max + margin, padding=0
                )


# ============================================================================
# MAIN VISUALIZER
# ============================================================================

class SignalVisualizer(QtWidgets.QMainWindow):
    """Real-time EEG signal visualizer with side-by-side port columns."""

    def __init__(self, host, port, spi_ports, window_seconds=5.0):
        super().__init__()

        self.host = host
        self.port = port
        self.spi_ports = spi_ports
        self.window_seconds = window_seconds
        self.sample_rate = 250
        self.max_samples = int(self.sample_rate * window_seconds)
        self.notch_on = True

        # 60 Hz notch filter coefficients (constant)
        self.notch_b, self.notch_a = sig.iirnotch(
            60.0, Q=30.0, fs=self.sample_rate
        )
        self.notch_zi_template = sig.lfilter_zi(self.notch_b, self.notch_a)

        # Thread-safe queue for cross-thread sample transfer
        self.sample_queue = Queue(maxsize=5000)

        self._init_data_state()

        self.port_columns: List[PortColumn] = []
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
        self.buffer_delay = 1.5
        self.playback_wall_start = None
        self.playback_server_start = None

        self.start_time = None
        self.sample_count = 0
        self.frame_count = 0

        self.notch_states = [
            self.notch_zi_template.copy() for _ in range(self.total_channels)
        ]

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
        self._rebuild_port_columns()

    def _build_port_columns(self):
        """Create PortColumn widgets for current spi_ports config."""
        ch_offset = 0
        for idx, pc in enumerate(self.spi_ports):
            color = PORT_COLORS[idx % len(PORT_COLORS)]
            col = PortColumn(pc, ch_offset, color, self)
            self.port_columns.append(col)
            self.columns_layout.addWidget(col, stretch=1,
                                          alignment=QtCore.Qt.AlignTop)
            ch_offset += pc.num_channels

    def _rebuild_port_columns(self):
        """Remove old port columns and create new ones for updated config."""
        for col in self.port_columns:
            self.columns_layout.removeWidget(col)
            col.setParent(None)
            col.deleteLater()
        self.port_columns.clear()
        self._build_port_columns()

        total_dev = sum(p.num_devices for p in self.spi_ports)
        total_ch = sum(p.num_channels for p in self.spi_ports)
        self.status.setText(
            f"Configured: {len(self.spi_ports)} ports, "
            f"{total_dev} devices, {total_ch} channels"
        )

    def setup_ui(self):
        self.setWindowTitle('ADS1299 Signal Visualizer')
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet(STYLESHEET)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        # Top bar: status + notch toggle
        top_bar = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel('Connecting...')
        self.status.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 11pt; padding: 5px;"
        )
        top_bar.addWidget(self.status, stretch=1)

        self.notch_btn = QtWidgets.QPushButton('60Hz Notch: ON')
        self.notch_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;'
        )
        self.notch_btn.clicked.connect(self.toggle_notch)
        top_bar.addWidget(self.notch_btn)
        layout.addLayout(top_bar)

        # Port columns — side by side, each with button + plots below
        columns_widget = QtWidgets.QWidget()
        self.columns_layout = QtWidgets.QHBoxLayout()
        self.columns_layout.setSpacing(6)
        self.columns_layout.setContentsMargins(0, 0, 0, 0)

        self._build_port_columns()

        columns_widget.setLayout(self.columns_layout)

        # Wrap in a scroll area so expanded plots can use full height
        columns_scroll = QtWidgets.QScrollArea()
        columns_scroll.setWidgetResizable(True)
        columns_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        columns_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        columns_scroll.setWidget(columns_widget)
        layout.addWidget(columns_scroll, stretch=1)

    def process_batch(self, samples):
        """Process multiple samples at once with vectorized notch filtering.

        Instead of calling lfilter 224×50 = 11,200 times (once per channel per
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

        # Build numpy arrays from sample dicts
        timestamps = np.array([s['timestamp'] for s in samples])
        raw = np.zeros((n, num_ch))
        for i, s in enumerate(samples):
            ch = s['channels']
            ch_count = min(len(ch), num_ch)
            raw[i, :ch_count] = ch[:ch_count]

        # Vectorized notch filter: one lfilter call per channel, n samples each
        if self.notch_on:
            for c in range(num_ch):
                raw[:, c], self.notch_states[c] = sig.lfilter(
                    self.notch_b, self.notch_a, raw[:, c],
                    zi=self.notch_states[c]
                )

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

        if self.sample_count % 250 < n:  # crossed a 250-sample boundary
            elapsed = time.time() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            port_summary = " | ".join(
                f"{p.name}({p.num_channels}ch)" for p in self.spi_ports
            )
            self.status.setText(
                f'{port_summary} | Rate: {rate:.1f} Hz | '
                f'Samples: {self.sample_count}'
            )

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
            rescale_y = (self.frame_count % 15 == 0)

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

            # Update only expanded columns
            for col in self.port_columns:
                col.update_plots(x_min, x_max, rescale_y)

        except Exception as e:
            print(f"[viz] update error: {e}")
            traceback.print_exc()

    def toggle_notch(self):
        self.notch_on = not self.notch_on
        if self.notch_on:
            self.notch_btn.setText('60Hz Notch: ON')
            self.notch_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;'
            )
            self.notch_states = [
                self.notch_zi_template.copy()
                for _ in range(self.total_channels)
            ]
        else:
            self.notch_btn.setText('60Hz Notch: OFF')
            self.notch_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;'
            )

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_N:
            self.toggle_notch()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.network_thread.stop()
        self.network_thread.wait()
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
                        compressed_size, sample_count = header_struct.unpack(header_data)
                        compressed_data = self._recv_exact(sock, compressed_size)

                        raw = lz4.frame.decompress(compressed_data)
                        for i in range(sample_count):
                            offset = i * sample_size
                            unpacked = sample_struct.unpack_from(raw, offset)
                            sample = {
                                'timestamp': unpacked[0],
                                'sample_number': unpacked[1],
                                'channels': list(unpacked[2:]),
                            }
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
        description='ADS1299 signal visualizer with per-port columns',
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
    )
    viz.show()

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
    print(f"{'='*50}\n")

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
