#!/usr/bin/env uv run script
# /// script
# dependencies = [
#   "numpy",
#   "PyQt5",
# ]
# ///
"""
ADS1299 Impedance Check Client
===============================

Connects to the Pi via TCP (same binary protocol as simpleviz.py),
receives raw ADC samples, computes impedance via single-bin DFT at
the LOFF excitation frequency (fDR/4 = 62.5 Hz at 250 SPS), and
displays a color-coded grid of all channels.

Usage:
    uv run impedance_check.py --host 192.168.1.175
    uv run impedance_check.py --host 172.20.10.7

The Pi must be running: sudo ./ads1299_acquire --check-impedance
"""

import sys
import json
import struct
import socket
import time
import argparse
import numpy as np
from collections import deque
from queue import Queue, Empty
from dataclasses import dataclass
from typing import List, Optional

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QRectF, pyqtSignal, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QFontMetrics
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QApplication, QGridLayout, QToolTip, QSizePolicy,
)

# ============================================================================
# CONSTANTS
# ============================================================================
VREF = 4.5
GAIN_IMPEDANCE = 1  # Impedance mode uses gain=1
LSB_UV = (VREF / (2**23) / GAIN_IMPEDANCE) * 1e6  # uV per LSB at gain=1
EXCITATION_UA = 6.0  # 6 µA excitation current
EXCITATION_HZ = 62.5  # fDR/4 at 250 SPS
SAMPLE_RATE = 250
DFT_WINDOW = 250  # samples for DFT (1 sec)

IMPEDANCE_SCALE = 1.0

# Impedance thresholds (kOhm)
THRESH_EXCELLENT = 5.0
THRESH_GOOD = 20.0
THRESH_MARGINAL = 50.0
THRESH_POOR = 100.0

# Minimum DFT magnitude (ADC codes) — below this = disconnected
MIN_EXCITATION_CODE = 100.0

# Colors
DARK_BG = "#0d0d0d"
COLOR_EXCELLENT = "#00E676"
COLOR_GOOD = "#4CAF50"
COLOR_MARGINAL = "#FFC107"
COLOR_POOR = "#FF9800"
COLOR_BAD = "#F44336"
COLOR_NODATA = "#333333"
TEXT_COLOR = "#e0e0e0"

# Port layout matching physical head position
PORT_ORDER = [
    # (port_name, grid_row, grid_col)
    ("Port6", 0, 0),   # Left
    ("Port7", 0, 1),   # Center-left
    ("Port1", 0, 2),   # Midline
    ("Port2", 0, 3),   # Center-right
    ("Port3", 0, 4),   # Right
    ("Port4", 1, 1),   # Posterior left
    ("Port5", 1, 3),   # Posterior right
]


def classify_impedance(z_kohm):
    """Return (status_str, color) for an impedance value."""
    if z_kohm < 0 or z_kohm > 1e6:
        return "bad", COLOR_BAD
    if z_kohm < THRESH_EXCELLENT:
        return "excellent", COLOR_EXCELLENT
    if z_kohm < THRESH_GOOD:
        return "good", COLOR_GOOD
    if z_kohm < THRESH_MARGINAL:
        return "marginal", COLOR_MARGINAL
    if z_kohm < THRESH_POOR:
        return "poor", COLOR_POOR
    return "bad", COLOR_BAD


def compute_impedance_from_samples(channel_samples):
    """
    Compute impedance from a buffer of ADC code samples using single-bin DFT
    at the excitation frequency (fDR/4). Uses the mod-4 trick — no trig needed.

    Returns (impedance_kohm, voltage_rms_uv, is_disconnected)
    """
    n = len(channel_samples)
    if n < 4:
        return 1e6, 0.0, True

    samples = np.array(channel_samples, dtype=np.float64)

    # DC removal
    dc_mean = np.mean(samples)
    x = samples - dc_mean

    # Single-bin DFT at fDR/4 using mod-4 arithmetic
    indices = np.arange(n) % 4
    sum_real = np.sum(x[indices == 0]) - np.sum(x[indices == 2])
    sum_imag = np.sum(x[indices == 1]) - np.sum(x[indices == 3])

    magnitude = np.sqrt(sum_real**2 + sum_imag**2)
    rms_code = np.sqrt(2) * magnitude / n

    if rms_code < MIN_EXCITATION_CODE:
        return 1e6, 0.0, True  # Disconnected

    rms_uv = rms_code * LSB_UV
    v_rms = rms_uv * 1e-6  # Convert to volts
    z_ohm = v_rms / (EXCITATION_UA * 1e-6) * IMPEDANCE_SCALE
    z_kohm = z_ohm / 1000.0

    return z_kohm, rms_uv, False


# ============================================================================
# NETWORK THREAD (same binary protocol as simpleviz.py)
# ============================================================================
class NetworkThread(QtCore.QThread):
    config_received = pyqtSignal(list)
    connected = pyqtSignal()
    disconnected = pyqtSignal()

    def __init__(self, host, port, sample_queue):
        super().__init__()
        self.host = host
        self.port = port
        self.sample_queue = sample_queue
        self.running = True
        self.buffer = b''

    def run(self):
        attempt = 0
        while self.running:
            attempt += 1
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                print(f"[net] Connecting to {self.host}:{self.port} (attempt #{attempt})...")
                sock.connect((self.host, self.port))
                sock.settimeout(1.0)
                print(f"[net] Connected")
                self.connected.emit()
                attempt = 0

                self.buffer = b''
                line = self._recv_line(sock)
                metadata = json.loads(line)
                print(f"[net] Metadata: {json.dumps(metadata, indent=2)}")

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
                            try:
                                self.sample_queue.put_nowait(sample)
                            except Exception:
                                try:
                                    self.sample_queue.get_nowait()
                                except Empty:
                                    pass
                                try:
                                    self.sample_queue.put_nowait(sample)
                                except Empty:
                                    pass
                    except socket.timeout:
                        continue

            except Exception as e:
                if self.running:
                    print(f"[net] {type(e).__name__}: {e} — reconnecting in 3s")
                    self.disconnected.emit()
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass

            if self.running:
                time.sleep(3)

    def _recv_line(self, sock):
        while b'\n' not in self.buffer:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Connection closed")
            self.buffer += chunk
        idx = self.buffer.index(b'\n')
        line = self.buffer[:idx].decode('utf-8')
        self.buffer = self.buffer[idx + 1:]
        return line

    def _recv_exact(self, sock, n):
        while len(self.buffer) < n:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                if not self.running:
                    raise ConnectionError("Stopped")
                continue
            if not chunk:
                raise ConnectionError("Connection closed")
            self.buffer += chunk
        data = self.buffer[:n]
        self.buffer = self.buffer[n:]
        return data

    def stop(self):
        self.running = False


# ============================================================================
# PORT CARD WIDGET
# ============================================================================
class PortCard(QWidget):
    HEADER_H = 22

    def __init__(self, port_name, num_devices, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.num_devices = num_devices
        self.num_channels = 8
        self.impedance = [[None] * 8 for _ in range(num_devices)]
        self.setMouseTracking(True)
        self._hover_dev = -1
        self._hover_ch = -1

        # Dev mode: 1 device = big cells with text. Multi-device = compact dots.
        self.dev_mode = (num_devices == 1)
        if self.dev_mode:
            self.cell_w = 90
            self.cell_h = 50
            self.cell_spacing = 4
            w = 8 * (self.cell_w + self.cell_spacing) + self.cell_spacing
            h = self.HEADER_H + self.cell_h + self.cell_spacing * 2
        else:
            self.dot_size = 14
            self.dot_spacing = 3
            w = 8 * (self.dot_size + self.dot_spacing) + 30 + self.dot_spacing
            h = self.HEADER_H + num_devices * (self.dot_size + self.dot_spacing) + self.dot_spacing
        self.setMinimumSize(w, h)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_impedance(self, device_idx, channel_idx, z_kohm):
        if 0 <= device_idx < self.num_devices and 0 <= channel_idx < 8:
            self.impedance[device_idx][channel_idx] = z_kohm

    def _cell_rect(self, dev, ch):
        if self.dev_mode:
            x = self.cell_spacing + ch * (self.cell_w + self.cell_spacing)
            y = self.HEADER_H + self.cell_spacing
            return QRectF(x, y, self.cell_w, self.cell_h)
        else:
            x = 28 + ch * (self.dot_size + self.dot_spacing)
            y = self.HEADER_H + dev * (self.dot_size + self.dot_spacing)
            return QRectF(x, y, self.dot_size, self.dot_size)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#1a1a1a"))

        # Header
        p.setPen(QColor(TEXT_COLOR))
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.drawText(4, 15, self.port_name)

        for d in range(self.num_devices):
            if not self.dev_mode:
                y = self.HEADER_H + d * (self.dot_size + self.dot_spacing) + self.dot_size - 2
                p.setPen(QColor("#888888"))
                p.setFont(QFont("Segoe UI", 7))
                p.drawText(2, y, f"D{d+1}")

            for c in range(8):
                rect = self._cell_rect(d, c)
                z = self.impedance[d][c]
                if z is None:
                    color = QColor(COLOR_NODATA)
                    status_text = "N/A"
                    z_text = ""
                else:
                    status, col = classify_impedance(z)
                    color = QColor(col)
                    if z >= 1e6:
                        z_text = "OPEN"
                    elif z >= 100:
                        z_text = f"{z:.0f} kΩ"
                    elif z >= 1:
                        z_text = f"{z:.1f} kΩ"
                    else:
                        z_text = f"{z*1000:.0f} Ω"

                is_hover = (d == self._hover_dev and c == self._hover_ch)
                if is_hover:
                    p.setPen(QPen(QColor("#ffffff"), 2))
                else:
                    p.setPen(Qt.NoPen)
                p.setBrush(QBrush(color))
                p.drawRoundedRect(rect, 5 if self.dev_mode else 3, 5 if self.dev_mode else 3)

                # Draw text inside cells in dev mode
                if self.dev_mode:
                    # Channel label
                    p.setPen(QColor("#000000") if color.lightness() > 128 else QColor("#ffffff"))
                    p.setFont(QFont("Segoe UI", 7, QFont.Bold))
                    p.drawText(rect.adjusted(4, 2, 0, 0), Qt.AlignLeft | Qt.AlignTop, f"Ch{c+1}")
                    # Impedance value
                    p.setFont(QFont("Segoe UI", 10, QFont.Bold))
                    p.drawText(rect, Qt.AlignCenter, z_text)

        p.end()

    def mouseMoveEvent(self, event):
        old_dev, old_ch = self._hover_dev, self._hover_ch
        self._hover_dev = -1
        self._hover_ch = -1
        for d in range(self.num_devices):
            for c in range(8):
                if self._cell_rect(d, c).contains(event.pos().x(), event.pos().y()):
                    self._hover_dev = d
                    self._hover_ch = c
                    z = self.impedance[d][c]
                    if z is not None:
                        status, _ = classify_impedance(z)
                        tip = f"{self.port_name} / D{d+1} / Ch{c+1}\n{z:.1f} kΩ — {status}"
                    else:
                        tip = f"{self.port_name} / D{d+1} / Ch{c+1}\nNo data"
                    QToolTip.showText(event.globalPos(), tip)
        if (self._hover_dev, self._hover_ch) != (old_dev, old_ch):
            self.update()

    def leaveEvent(self, event):
        self._hover_dev = -1
        self._hover_ch = -1
        self.update()


# ============================================================================
# MAIN WINDOW
# ============================================================================
class ImpedanceWindow(QMainWindow):
    def __init__(self, host, port):
        super().__init__()
        self.setWindowTitle("ADS1299 Impedance Check")
        self.setStyleSheet(f"background-color: {DARK_BG}; color: {TEXT_COLOR};")
        self.resize(900, 500)

        self.sample_queue = Queue(maxsize=2000)
        self.port_cards = {}
        self.port_config = []
        self.channel_buffers = {}  # (port_idx, dev, ch) -> deque of samples
        self.total_channels = 0

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)

        # Summary bar
        self.summary_label = QLabel("Waiting for connection...")
        self.summary_label.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self.summary_label.setAlignment(Qt.AlignCenter)
        self.summary_label.setStyleSheet(f"color: {TEXT_COLOR}; padding: 4px;")
        layout.addWidget(self.summary_label)

        # Port grid
        self.grid_widget = QWidget()
        self.grid_layout = QGridLayout(self.grid_widget)
        self.grid_layout.setSpacing(8)
        layout.addWidget(self.grid_widget, stretch=1)

        # Legend
        legend = QHBoxLayout()
        legend.addStretch()
        for label, color in [("< 5k", COLOR_EXCELLENT), ("5-20k", COLOR_GOOD),
                              ("20-50k", COLOR_MARGINAL), ("50-100k", COLOR_POOR),
                              ("> 100k", COLOR_BAD), ("N/A", COLOR_NODATA)]:
            dot = QLabel("●")
            dot.setFont(QFont("Segoe UI", 10))
            dot.setStyleSheet(f"color: {color};")
            legend.addWidget(dot)
            lbl = QLabel(label)
            lbl.setFont(QFont("Segoe UI", 8))
            lbl.setStyleSheet(f"color: #888;")
            legend.addWidget(lbl)
            legend.addSpacing(8)
        legend.addStretch()
        legend_w = QWidget()
        legend_w.setLayout(legend)
        layout.addWidget(legend_w)

        # Status bar
        self.status_label = QLabel("Disconnected")
        self.status_label.setFont(QFont("Segoe UI", 8))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #666;")
        layout.addWidget(self.status_label)

        # Network thread
        self.net_thread = NetworkThread(host, port, self.sample_queue)
        self.net_thread.config_received.connect(self._on_config)
        self.net_thread.connected.connect(lambda: self.status_label.setText("Connected"))
        self.net_thread.disconnected.connect(lambda: self.status_label.setText("Disconnected — reconnecting..."))
        self.net_thread.start()

        # Update timer (process samples + recompute impedance)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._process_samples)
        self.update_timer.start(200)  # 5 Hz UI update

        self._sample_count = 0

    def _on_config(self, port_config):
        """Called when server sends port_config metadata."""
        self.port_config = port_config
        self.channel_buffers = {}

        # Clear existing cards
        while self.grid_layout.count():
            item = self.grid_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        # Build port name -> config mapping
        port_map = {}
        ch_offset = 0
        for i, pc in enumerate(port_config):
            name = pc['name']
            n_dev = pc['num_devices']
            port_map[name] = (i, n_dev, ch_offset)
            # Initialize channel buffers
            for d in range(n_dev):
                for c in range(8):
                    self.channel_buffers[(i, d, c)] = deque(maxlen=DFT_WINDOW)
            ch_offset += n_dev * 8

        self.total_channels = ch_offset

        # Create port cards in physical head layout
        for port_name, row, col in PORT_ORDER:
            if port_name in port_map:
                idx, n_dev, _ = port_map[port_name]
                card = PortCard(port_name, n_dev)
                self.port_cards[port_name] = (card, idx)
                self.grid_layout.addWidget(card, row, col)

        self.summary_label.setText(f"Configured: {self.total_channels} channels across {len(port_config)} ports")

    def _process_samples(self):
        """Drain sample queue, fill buffers, recompute impedance."""
        count = 0
        while not self.sample_queue.empty() and count < 500:
            try:
                sample = self.sample_queue.get_nowait()
            except Empty:
                break
            count += 1
            self._sample_count += 1

            channels = sample['channels']
            ch_idx = 0
            for p_idx, pc in enumerate(self.port_config):
                n_dev = pc['num_devices']
                for d in range(n_dev):
                    for c in range(8):
                        if ch_idx < len(channels):
                            key = (p_idx, d, c)
                            if key in self.channel_buffers:
                                self.channel_buffers[key].append(channels[ch_idx])
                        ch_idx += 1

        if count == 0:
            return

        # Recompute impedance every DFT_WINDOW samples
        if self._sample_count % DFT_WINDOW < count or self._sample_count < DFT_WINDOW * 2:
            self._recompute_impedance()

    def _recompute_impedance(self):
        """Compute impedance for all channels and update cards."""
        counts = {"excellent": 0, "good": 0, "marginal": 0, "poor": 0, "bad": 0}
        total = 0

        debug_parts = []
        for port_name, (card, p_idx) in self.port_cards.items():
            pc = self.port_config[p_idx]
            n_dev = pc['num_devices']
            for d in range(n_dev):
                for c in range(8):
                    buf = self.channel_buffers.get((p_idx, d, c))
                    if buf and len(buf) >= 16:
                        z_kohm, rms_uv, disconnected = compute_impedance_from_samples(list(buf))
                        rms_code = rms_uv / LSB_UV if LSB_UV > 0 else 0
                        card.update_impedance(d, c, z_kohm)
                        status, _ = classify_impedance(z_kohm)
                        counts[status] = counts.get(status, 0) + 1
                        debug_parts.append(f"Ch{c+1}:{rms_code:.0f}c/{z_kohm:.1f}k")
                    else:
                        card.update_impedance(d, c, None)
                        debug_parts.append(f"Ch{c+1}:--")
                    total += 1
            card.update()
        if debug_parts:
            print(f"[impedance] {' | '.join(debug_parts)}")

        # Update summary
        usable = counts["excellent"] + counts["good"]
        pct = (usable / total * 100) if total > 0 else 0
        color = COLOR_EXCELLENT if pct > 80 else (COLOR_MARGINAL if pct > 50 else COLOR_BAD)
        self.summary_label.setText(
            f"<span style='color:{color}'>{pct:.0f}% Ready</span> — "
            f"{usable}/{total} usable | "
            f"<span style='color:{COLOR_MARGINAL}'>{counts['marginal']} marginal</span> | "
            f"<span style='color:{COLOR_POOR}'>{counts['poor']} poor</span> | "
            f"<span style='color:{COLOR_BAD}'>{counts['bad']} bad</span>"
        )
        self.status_label.setText(f"Samples: {self._sample_count}")

    def closeEvent(self, event):
        self.net_thread.stop()
        self.net_thread.wait(2000)
        event.accept()


# ============================================================================
# MAIN
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="ADS1299 Impedance Check Client")
    parser.add_argument("--host", default="192.168.1.175", help="Pi IP address")
    parser.add_argument("--port", type=int, default=8888, help="TCP port (default: 8888)")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = ImpedanceWindow(args.host, args.port)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
