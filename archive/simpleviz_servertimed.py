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
8 channels for a selected device with 60Hz notch filtering.

Usage:
    uv run simpleviz_servertimed.py --host <pi-ip> --device 1
"""

import sys
import socket
import json
import csv
import argparse
import time
import struct
import lz4.frame
from collections import deque
import numpy as np
from scipy import signal as sig

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
pg.setConfigOptions(useOpenGL=True, antialias=False)


class TestSignalVisualizer(QtWidgets.QMainWindow):
    """Real-time EEG signal visualizer with buffered playback."""

    def __init__(self, host, port, window_seconds=5.0, device=1):
        super().__init__()

        self.host = host
        self.port = port
        self.window_seconds = window_seconds
        self.device = device
        self.dev_offset = (device - 1) * 8
        self.num_channels = 8
        self.sample_rate = 250
        self.max_samples = int(self.sample_rate * window_seconds)

        # Ring buffers (2x size for zero-copy slicing)
        buf_size = self.max_samples * 2
        self.time_buf = np.zeros((self.num_channels, buf_size))
        self.data_buf = np.zeros((self.num_channels, buf_size))
        self.buf_idx = 0
        self.buf_count = 0

        # Buffered playback to absorb WiFi jitter
        self.sample_queue = deque()
        self.pending_samples = deque()
        self.buffer_delay = 1.5
        self.playback_wall_start = None
        self.playback_server_start = None

        self.start_time = None
        self.sample_count = 0
        self.frame_count = 0
        self.last_ch1 = 0

        # CSV logging
        self.csv_file = open('CLIENTCh1.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'sample_number', 'ch1_value'])
        print(f"[csv] Logging Dev{self.device} Ch1 to CLIENTCh1.csv")

        # 60 Hz notch filter
        self.notch_b, self.notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sample_rate)
        self.notch_zi_template = sig.lfilter_zi(self.notch_b, self.notch_a)
        self.notch_states = [self.notch_zi_template.copy() for _ in range(self.num_channels)]
        self.notch_on = True

        self.setup_ui()

        self.network_thread = NetworkThread(self, host, port)
        self.network_thread.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(16)

    def setup_ui(self):
        self.setWindowTitle(f'ADS1299 Device {self.device} (Server-Timed)')
        self.setGeometry(100, 100, 1200, 800)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        top_bar = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel('Connecting...')
        top_bar.addWidget(self.status)
        top_bar.addStretch()
        self.notch_btn = QtWidgets.QPushButton('60Hz Notch: ON')
        self.notch_btn.setCheckable(True)
        self.notch_btn.setChecked(True)
        self.notch_btn.setStyleSheet('background-color: #00aa55; color: white; padding: 4px 12px;')
        self.notch_btn.clicked.connect(self.toggle_notch)
        top_bar.addWidget(self.notch_btn)
        layout.addLayout(top_bar)

        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        self.plots = []
        self.curves = []
        colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63', '#9C27B0', '#00BCD4', '#FFEB3B', '#795548']

        for i in range(self.num_channels):
            plot = self.plot_widget.addPlot(row=i, col=0)
            plot.setLabel('left', f'Dev{self.device} Ch{i+1}')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setXRange(0, self.window_seconds)
            plot.setYRange(-500000, 500000)
            plot.setClipToView(True)
            plot.setDownsampling(mode='peak')
            curve = plot.plot(pen=pg.mkPen(color=colors[i % len(colors)], width=1))
            self.plots.append(plot)
            self.curves.append(curve)

        for i in range(1, self.num_channels):
            self.plots[i].setXLink(self.plots[0])

    def process_sample(self, sample):
        timestamp = sample['timestamp']

        if self.start_time is None:
            self.start_time = time.time()

        all_channels = sample['channels']
        if len(all_channels) < self.dev_offset + 8:
            return
        channels = all_channels[self.dev_offset:self.dev_offset + 8]

        self.csv_writer.writerow([timestamp, sample['sample_number'], channels[0]])
        self.last_ch1 = channels[0]

        idx = self.buf_idx
        for i in range(self.num_channels):
            val = float(channels[i])
            if self.notch_on:
                filtered, self.notch_states[i] = sig.lfilter(
                    self.notch_b, self.notch_a, [val], zi=self.notch_states[i]
                )
                val = filtered[0]
            self.time_buf[i, idx] = timestamp
            self.data_buf[i, idx] = val

        self.buf_idx += 1
        if self.buf_count < self.max_samples:
            self.buf_count += 1

        if self.buf_idx >= self.max_samples * 2:
            self.time_buf[:, :self.max_samples] = self.time_buf[:, self.max_samples:]
            self.data_buf[:, :self.max_samples] = self.data_buf[:, self.max_samples:]
            self.buf_idx = self.max_samples

        self.sample_count += 1

        if self.sample_count % 250 == 0:
            elapsed = time.time() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            self.status.setText(
                f'Streaming from {self.host}:{self.port} | '
                f'Samples: {self.sample_count} | '
                f'Rate: {rate:.1f} Hz | '
                f'Dev{self.device} Ch1: {self.last_ch1:+8d}'
            )

    def update_plots(self):
        """Buffered playback: absorb WiFi jitter, release samples at steady rate."""
        # Drain network queue into holding buffer
        while True:
            try:
                self.pending_samples.append(self.sample_queue.popleft())
            except IndexError:
                break

        # Initialize playback clock on first sample
        if self.pending_samples and self.playback_wall_start is None:
            self.playback_wall_start = time.time()
            self.playback_server_start = self.pending_samples[0]['timestamp']

        if self.playback_wall_start is None:
            return

        # Release samples whose server timestamp is due
        playback_time = self.playback_server_start + (time.time() - self.playback_wall_start) - self.buffer_delay
        count_before = self.sample_count
        for _ in range(50):
            if not self.pending_samples:
                break
            if self.pending_samples[0]['timestamp'] <= playback_time:
                self.process_sample(self.pending_samples.popleft())
            else:
                break

        if self.sample_count == count_before or self.buf_count == 0:
            return

        self.frame_count += 1
        rescale_y = (self.frame_count % 15 == 0)

        start = self.buf_idx - self.buf_count
        end = self.buf_idx

        self.plot_widget.setUpdatesEnabled(False)

        for i in range(self.num_channels):
            t = self.time_buf[i, start:end]
            d = self.data_buf[i, start:end]
            self.curves[i].setData(t, d)

            if rescale_y:
                y_min, y_max = d.min(), d.max()
                margin = max((y_max - y_min) * 0.1, 10000)
                self.plots[i].setYRange(y_min - margin, y_max + margin, padding=0)

        t0 = self.time_buf[0, start:end]
        latest = t0[-1]
        if latest <= self.window_seconds:
            x_min, x_max = 0, self.window_seconds
        else:
            x_min, x_max = latest - self.window_seconds, latest
        self.plots[0].setXRange(x_min, x_max, padding=0)

        self.plot_widget.setUpdatesEnabled(True)
        self.plot_widget.viewport().update()

    def toggle_notch(self):
        self.notch_on = not self.notch_on
        if self.notch_on:
            self.notch_btn.setText('60Hz Notch: ON')
            self.notch_btn.setStyleSheet('background-color: #00aa55; color: white; padding: 4px 12px;')
            self.notch_states = [self.notch_zi_template.copy() for _ in range(self.num_channels)]
        else:
            self.notch_btn.setText('60Hz Notch: OFF')
            self.notch_btn.setStyleSheet('background-color: #2a2a2a; color: #aaaaaa; padding: 4px 12px;')

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_N:
            self.toggle_notch()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.network_thread.stop()
        self.network_thread.wait()
        self.csv_file.close()
        print(f"[csv] CLIENTCh1.csv closed ({self.sample_count} rows)")
        event.accept()


class NetworkThread(QtCore.QThread):
    """Receives binary LZ4 frames from Controller.py with auto-reconnect."""

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
                            self.visualizer.sample_queue.append({
                                'timestamp': unpacked[0],
                                'sample_number': unpacked[1],
                                'channels': list(unpacked[2:]),
                            })
                    except socket.timeout:
                        continue

            except Exception as e:
                if self.running:
                    print(f"[net] Error: {type(e).__name__}: {e} — reconnecting in 2s")
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


def main():
    parser = argparse.ArgumentParser(description='ADS1299 signal visualizer (server-timed)')
    parser.add_argument('--host', type=str, required=True, help='Server IP address')
    parser.add_argument('--port', type=int, default=8888, help='Server port (default: 8888)')
    parser.add_argument('--window', type=float, default=5.0, help='Time window in seconds (default: 5.0)')
    parser.add_argument('--device', type=int, default=1, help='ADS1299 device number to display (default: 1)')

    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv)

    viz = TestSignalVisualizer(
        args.host,
        args.port,
        window_seconds=args.window,
        device=args.device,
    )
    viz.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
