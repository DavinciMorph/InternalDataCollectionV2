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
CSV Data Viewer
===============

Scroll through recorded ADS1299 CSV data with a slider.
Supports multi-channel CSV files (e.g. all_ports_ch1_data.csv).
Device selector buttons show one device's graph at a time.

Usage:
    uv run csvviewer.py --file all_ports_ch1_data.csv
    uv run csvviewer.py --file all_ports_ch1_data.csv --window 10
"""

import sys
import argparse
import numpy as np
from scipy import signal as sig

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


class CSVViewer(QtWidgets.QMainWindow):
    def __init__(self, filepath, window_seconds=5.0):
        super().__init__()

        # Read header for channel names
        with open(filepath, 'r') as f:
            header = f.readline().strip().split(',')
        self.channel_names = header[2:]

        # Load data
        data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
        self.timestamps = data[:, 0]
        self.samples = data[:, 1].astype(int)
        self.channel_data = data[:, 2:]
        self.num_channels = self.channel_data.shape[1]
        self.total = len(self.timestamps)
        self.sps = 250
        self.window_size = int(window_seconds * self.sps)
        self.selected_channel = 0

        # Pre-compute 60 Hz notch-filtered signals for all channels
        notch_b, notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sps)
        self.filtered_data = np.zeros_like(self.channel_data)
        for i in range(self.num_channels):
            self.filtered_data[:, i] = sig.filtfilt(notch_b, notch_a, self.channel_data[:, i])
        self.notch_on = False

        self.setWindowTitle(f'CSV Viewer — {filepath}')
        self.setGeometry(100, 100, 1200, 800)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # Top bar
        top_bar = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel()
        top_bar.addWidget(self.status)
        top_bar.addStretch()
        self.notch_btn = QtWidgets.QPushButton('60Hz Notch: OFF')
        self.notch_btn.setCheckable(True)
        self.notch_btn.setChecked(False)
        self.notch_btn.setStyleSheet('background-color: #2a2a2a; color: #aaaaaa; padding: 4px 12px;')
        self.notch_btn.clicked.connect(self.toggle_notch)
        top_bar.addWidget(self.notch_btn)
        layout.addLayout(top_bar)

        # Device selector buttons
        colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63',
                  '#9C27B0', '#00BCD4', '#FFEB3B', '#795548',
                  '#F44336', '#8BC34A', '#3F51B5', '#FF5722',
                  '#607D8B', '#CDDC39']
        self.colors = colors

        btn_bar = QtWidgets.QHBoxLayout()
        self.device_btns = []
        for i in range(self.num_channels):
            name = self.channel_names[i] if i < len(self.channel_names) else f'Ch{i+1}'
            btn = QtWidgets.QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, idx=i: self.select_channel(idx))
            self.device_btns.append(btn)
            btn_bar.addWidget(btn)
        layout.addLayout(btn_bar)
        self._style_device_buttons()

        # Single plot
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setClipToView(True)
        self.plot_widget.setDownsampling(mode='peak')
        self.curve = self.plot_widget.plot(
            pen=pg.mkPen(color=colors[0], width=2))
        layout.addWidget(self.plot_widget)

        # Bottom controls
        bottom = QtWidgets.QHBoxLayout()

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, self.total - self.window_size))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.update_plot)
        bottom.addWidget(self.slider, stretch=1)

        bottom.addWidget(QtWidgets.QLabel('Window:'))
        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(0.5, self.timestamps[-1])
        self.window_spin.setValue(window_seconds)
        self.window_spin.setSuffix(' s')
        self.window_spin.setSingleStep(1.0)
        self.window_spin.valueChanged.connect(self.change_window)
        bottom.addWidget(self.window_spin)

        layout.addLayout(bottom)

        self.update_plot()

    def _style_device_buttons(self):
        for i, btn in enumerate(self.device_btns):
            color = self.colors[i % len(self.colors)]
            if i == self.selected_channel:
                btn.setChecked(True)
                btn.setStyleSheet(
                    f'background-color: {color}; color: white; padding: 4px 8px; font-weight: bold;')
            else:
                btn.setChecked(False)
                btn.setStyleSheet(
                    'background-color: #2a2a2a; color: #aaaaaa; padding: 4px 8px;')

    def select_channel(self, idx):
        self.selected_channel = idx
        self._style_device_buttons()
        color = self.colors[idx % len(self.colors)]
        self.curve.setPen(pg.mkPen(color=color, width=2))
        name = self.channel_names[idx] if idx < len(self.channel_names) else f'Ch{idx+1}'
        self.plot_widget.setLabel('left', name)
        self.update_plot()

    def change_window(self, val):
        self.window_size = max(1, int(val * self.sps))
        self.slider.setMaximum(max(0, self.total - self.window_size))
        self.update_plot()

    def update_plot(self):
        start = self.slider.value()
        end = min(start + self.window_size, self.total)

        t = self.timestamps[start:end]
        source = self.filtered_data if self.notch_on else self.channel_data
        v = source[start:end, self.selected_channel]

        self.curve.setData(t, v)
        self.plot_widget.setXRange(t[0], t[-1], padding=0)

        y_min, y_max = v.min(), v.max()
        margin = max((y_max - y_min) * 0.1, 1000)
        self.plot_widget.setYRange(y_min - margin, y_max + margin, padding=0)

        name = self.channel_names[self.selected_channel] if self.selected_channel < len(self.channel_names) else f'Ch{self.selected_channel+1}'
        self.status.setText(
            f'{name}  |  '
            f't = {t[0]:.3f} – {t[-1]:.3f} s  |  '
            f'samples {self.samples[start]}–{self.samples[end-1]}  |  '
            f'{end - start} pts shown  |  '
            f'range [{y_min:.0f}, {y_max:.0f}]'
        )

    def toggle_notch(self):
        self.notch_on = not self.notch_on
        if self.notch_on:
            self.notch_btn.setText('60Hz Notch: ON')
            self.notch_btn.setStyleSheet('background-color: #00aa55; color: white; padding: 4px 12px;')
        else:
            self.notch_btn.setText('60Hz Notch: OFF')
            self.notch_btn.setStyleSheet('background-color: #2a2a2a; color: #aaaaaa; padding: 4px 12px;')
        self.update_plot()

    def keyPressEvent(self, event):
        step = max(1, self.window_size // 4)
        key = event.key()
        if key == QtCore.Qt.Key_N:
            self.toggle_notch()
        elif key == QtCore.Qt.Key_Right:
            self.slider.setValue(min(self.slider.value() + step, self.slider.maximum()))
        elif key == QtCore.Qt.Key_Left:
            self.slider.setValue(max(self.slider.value() - step, 0))
        elif key == QtCore.Qt.Key_Home:
            self.slider.setValue(0)
        elif key == QtCore.Qt.Key_End:
            self.slider.setValue(self.slider.maximum())
        else:
            super().keyPressEvent(event)


def main():
    parser = argparse.ArgumentParser(description='CSV Data Viewer')
    parser.add_argument('--file', type=str, default='all_ports_ch1_data.csv', help='CSV file to view')
    parser.add_argument('--window', type=float, default=5.0, help='Visible window in seconds (default: 5.0)')
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    viewer = CSVViewer(args.file, window_seconds=args.window)
    viewer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
