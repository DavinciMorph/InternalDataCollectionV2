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
Supports both all_channels_data.csv (224 ch) and all_ports_ch1_data.csv (7 ch).

For 224-channel files (Port{N}_dev{M}_ch{K} naming), shows a hierarchical
Port / Device / Channel selector. For other files, shows flat channel buttons.

Usage:
    uv run csvviewer.py --file all_channels_data.csv
    uv run csvviewer.py --file all_channels_data.csv --window 10
"""

import sys
import os
import csv
import argparse
import re
import numpy as np
from scipy import signal as sig

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


PORT_COLORS = {
    'Port1': '#2196F3', 'Port2': '#4CAF50', 'Port3': '#FF9800', 'Port4': '#E91E63',
    'Port5': '#9C27B0', 'Port6': '#00BCD4', 'Port7': '#FFEB3B',
}
FLAT_COLORS = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63',
               '#9C27B0', '#00BCD4', '#FFEB3B', '#795548',
               '#F44336', '#8BC34A', '#3F51B5', '#FF5722',
               '#607D8B', '#CDDC39']
INACTIVE_STYLE = 'background-color: #2a2a2a; color: #aaaaaa; padding: 4px 8px;'


def parse_hierarchy(channel_names):
    """Detect Port{N}_dev{M}_ch{K} naming and return hierarchy mappings.

    Returns a dict with:
        'ports':        sorted list of all port names
        'all_devs':     sorted list of all unique device names (union across ports)
        'all_chs':      sorted list of all unique channel names (union across all)
        'port_devs':    {port: [sorted devices present under this port]}
        'port_dev_chs': {(port, dev): [sorted channels present under this combo]}
    Returns None if channel names don't match hierarchical pattern.
    """
    pat = re.compile(r'^(Port\d+)_(dev\d+)_(ch\d+)$')
    ports_set, devs_set, chs_set = set(), set(), set()
    port_devs_map = {}       # port -> set of devs
    port_dev_chs_map = {}    # (port, dev) -> set of chs
    for name in channel_names:
        m = pat.match(name)
        if not m:
            return None  # not hierarchical
        p, d, c = m.group(1), m.group(2), m.group(3)
        ports_set.add(p)
        devs_set.add(d)
        chs_set.add(c)
        port_devs_map.setdefault(p, set()).add(d)
        port_dev_chs_map.setdefault((p, d), set()).add(c)

    sort_key = lambda s: int(re.search(r'\d+', s).group())
    return {
        'ports':        sorted(ports_set, key=sort_key),
        'all_devs':     sorted(devs_set, key=sort_key),
        'all_chs':      sorted(chs_set, key=sort_key),
        'port_devs':    {p: sorted(ds, key=sort_key) for p, ds in port_devs_map.items()},
        'port_dev_chs': {k: sorted(cs, key=sort_key) for k, cs in port_dev_chs_map.items()},
    }


class CSVViewer(QtWidgets.QMainWindow):
    def __init__(self, filepath, window_seconds=5.0):
        super().__init__()

        # Load data, skipping truncated rows
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            expected_cols = len(header)
            rows = [row for row in reader if len(row) == expected_cols]
        self.channel_names = header[2:]

        data = np.array(rows, dtype=np.float64)
        self.timestamps = data[:, 0]
        self.samples = data[:, 1].astype(int)
        self.channel_data = data[:, 2:]
        self.num_channels = self.channel_data.shape[1]
        self.total = len(self.timestamps)
        self.sps = 250
        self.window_size = int(window_seconds * self.sps)

        # Build channel name -> column index lookup
        self.ch_index = {name: i for i, name in enumerate(self.channel_names)}

        # Detect hierarchical naming
        self.hierarchy = parse_hierarchy(self.channel_names)

        # Initial selection
        if self.hierarchy:
            self.sel_port = self.hierarchy['ports'][0]
            self.sel_dev = self.hierarchy['port_devs'][self.sel_port][0]
            self.sel_ch = self.hierarchy['port_dev_chs'][(self.sel_port, self.sel_dev)][0]
        self.selected_channel = 0

        # Pre-compute 60 Hz notch-filtered signals for all channels
        notch_b, notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sps)
        self.filtered_data = np.zeros_like(self.channel_data)
        for i in range(self.num_channels):
            self.filtered_data[:, i] = sig.filtfilt(notch_b, notch_a, self.channel_data[:, i])
        self.notch_on = False

        self.setWindowTitle(f'CSV Viewer — {os.path.basename(filepath)}')
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

        # Channel selector
        if self.hierarchy:
            self._build_hierarchical_selector(layout)
        else:
            self._build_flat_selector(layout)

        # Single plot
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setClipToView(True)
        self.plot_widget.setDownsampling(mode='peak')
        self.curve = self.plot_widget.plot(
            pen=pg.mkPen(color=self._current_color(), width=2))
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

    # --- Hierarchical selector (Port / Device / Channel rows) ---

    def _build_hierarchical_selector(self, layout):
        h = self.hierarchy

        # Port row — all ports are always visible
        port_bar = QtWidgets.QHBoxLayout()
        port_bar.addWidget(QtWidgets.QLabel('Port:'))
        self.port_btns = {}
        for p in h['ports']:
            btn = QtWidgets.QPushButton(p)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, name=p: self._pick_port(name))
            self.port_btns[p] = btn
            port_bar.addWidget(btn)
        port_bar.addStretch()
        layout.addLayout(port_bar)

        # Device row — create buttons for all unique devices, visibility controlled per port
        dev_bar = QtWidgets.QHBoxLayout()
        dev_bar.addWidget(QtWidgets.QLabel('Device:'))
        self.dev_btns = {}
        for d in h['all_devs']:
            btn = QtWidgets.QPushButton(d)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, name=d: self._pick_dev(name))
            self.dev_btns[d] = btn
            dev_bar.addWidget(btn)
        dev_bar.addStretch()
        layout.addLayout(dev_bar)

        # Channel row — create buttons for all unique channels, visibility controlled per port+dev
        ch_bar = QtWidgets.QHBoxLayout()
        ch_bar.addWidget(QtWidgets.QLabel('Channel:'))
        self.ch_btns = {}
        for c in h['all_chs']:
            btn = QtWidgets.QPushButton(c)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, name=c: self._pick_ch(name))
            self.ch_btns[c] = btn
            ch_bar.addWidget(btn)
        ch_bar.addStretch()
        layout.addLayout(ch_bar)

        self._update_visible_buttons()
        self._style_hierarchy_buttons()

    def _pick_port(self, name):
        self.sel_port = name
        self._apply_hierarchy_selection()

    def _pick_dev(self, name):
        self.sel_dev = name
        self._apply_hierarchy_selection()

    def _pick_ch(self, name):
        self.sel_ch = name
        self._apply_hierarchy_selection()

    def _apply_hierarchy_selection(self):
        col_name = f'{self.sel_port}_{self.sel_dev}_{self.sel_ch}'
        if col_name in self.ch_index:
            self.selected_channel = self.ch_index[col_name]
        self._style_hierarchy_buttons()
        self.curve.setPen(pg.mkPen(color=self._current_color(), width=2))
        self.plot_widget.setLabel('left', self._current_name())
        self.update_plot()

    def _style_hierarchy_buttons(self):
        color = PORT_COLORS.get(self.sel_port, '#2196F3')
        active_style = f'background-color: {color}; color: white; padding: 4px 8px; font-weight: bold;'
        for name, btn in self.port_btns.items():
            btn.setChecked(name == self.sel_port)
            btn.setStyleSheet(active_style if name == self.sel_port else INACTIVE_STYLE)
        for name, btn in self.dev_btns.items():
            btn.setChecked(name == self.sel_dev)
            btn.setStyleSheet(active_style if name == self.sel_dev else INACTIVE_STYLE)
        for name, btn in self.ch_btns.items():
            btn.setChecked(name == self.sel_ch)
            btn.setStyleSheet(active_style if name == self.sel_ch else INACTIVE_STYLE)

    # --- Flat selector (original behavior for non-hierarchical CSVs) ---

    def _build_flat_selector(self, layout):
        btn_bar = QtWidgets.QHBoxLayout()
        self.device_btns = []
        for i in range(self.num_channels):
            name = self.channel_names[i] if i < len(self.channel_names) else f'Ch{i+1}'
            btn = QtWidgets.QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, idx=i: self._pick_flat(idx))
            self.device_btns.append(btn)
            btn_bar.addWidget(btn)
        layout.addLayout(btn_bar)
        self._style_flat_buttons()

    def _pick_flat(self, idx):
        self.selected_channel = idx
        self._style_flat_buttons()
        self.curve.setPen(pg.mkPen(color=self._current_color(), width=2))
        self.plot_widget.setLabel('left', self._current_name())
        self.update_plot()

    def _style_flat_buttons(self):
        for i, btn in enumerate(self.device_btns):
            color = FLAT_COLORS[i % len(FLAT_COLORS)]
            if i == self.selected_channel:
                btn.setChecked(True)
                btn.setStyleSheet(
                    f'background-color: {color}; color: white; padding: 4px 8px; font-weight: bold;')
            else:
                btn.setChecked(False)
                btn.setStyleSheet(INACTIVE_STYLE)

    # --- Common helpers ---

    def _current_name(self):
        if self.selected_channel < len(self.channel_names):
            return self.channel_names[self.selected_channel]
        return f'Ch{self.selected_channel+1}'

    def _current_color(self):
        if self.hierarchy:
            return PORT_COLORS.get(self.sel_port, '#2196F3')
        return FLAT_COLORS[self.selected_channel % len(FLAT_COLORS)]

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

        self.status.setText(
            f'{self._current_name()}  |  '
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
    default_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'all_channels_data.csv')
    parser.add_argument('--file', type=str, default=default_csv, help='CSV file to view')
    parser.add_argument('--window', type=float, default=5.0, help='Visible window in seconds (default: 5.0)')
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    viewer = CSVViewer(args.file, window_seconds=args.window)
    viewer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
