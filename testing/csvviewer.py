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
        'chan_indices':  list of column indices (into header[2:]) that are actual channels
    Returns None if no channel names match hierarchical pattern.
    """
    pat = re.compile(r'^(Port\d+)_(dev\d+)_(ch\d+)$')
    ports_set, devs_set, chs_set = set(), set(), set()
    port_devs_map = {}       # port -> set of devs
    port_dev_chs_map = {}    # (port, dev) -> set of chs
    chan_indices = []         # column indices that are actual channels
    for i, name in enumerate(channel_names):
        m = pat.match(name)
        if not m:
            continue  # skip non-channel columns (e.g. perf_counter)
        p, d, c = m.group(1), m.group(2), m.group(3)
        ports_set.add(p)
        devs_set.add(d)
        chs_set.add(c)
        port_devs_map.setdefault(p, set()).add(d)
        port_dev_chs_map.setdefault((p, d), set()).add(c)
        chan_indices.append(i)

    if not chan_indices:
        return None  # no hierarchical channels found

    sort_key = lambda s: int(re.search(r'\d+', s).group())
    return {
        'ports':        sorted(ports_set, key=sort_key),
        'all_devs':     sorted(devs_set, key=sort_key),
        'all_chs':      sorted(chs_set, key=sort_key),
        'port_devs':    {p: sorted(ds, key=sort_key) for p, ds in port_devs_map.items()},
        'port_dev_chs': {k: sorted(cs, key=sort_key) for k, cs in port_dev_chs_map.items()},
        'chan_indices':  chan_indices,
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
        all_channel_names = header[2:]

        data = np.array(rows, dtype=np.float64)
        self.timestamps = data[:, 0]
        self.samples = data[:, 1].astype(int)

        # Detect hierarchical naming
        self.hierarchy = parse_hierarchy(all_channel_names)

        # If hierarchy detected with extra non-channel columns, filter them out
        if self.hierarchy and len(self.hierarchy['chan_indices']) < len(all_channel_names):
            idx = self.hierarchy['chan_indices']
            self.channel_names = [all_channel_names[i] for i in idx]
            self.channel_data = data[:, 2:][:, idx]
        else:
            self.channel_names = all_channel_names
            self.channel_data = data[:, 2:]

        self.num_channels = self.channel_data.shape[1]
        self.total = len(self.timestamps)
        self.sps = 250
        self.window_size = int(window_seconds * self.sps)

        # Build channel name -> column index lookup
        self.ch_index = {name: i for i, name in enumerate(self.channel_names)}

        # Initial selection
        if self.hierarchy:
            self.sel_port = self.hierarchy['ports'][0]
            self.sel_dev = self.hierarchy['port_devs'][self.sel_port][0]
            self.sel_ch = self.hierarchy['port_dev_chs'][(self.sel_port, self.sel_dev)][0]
        self.selected_channel = 0

        # Filter state (applied on-the-fly per selected channel)
        self.notch_on = False
        self.hpf_on = False
        self.lpf_on = False
        self._filter_cache = None  # (channel_idx, notch, hpf, lpf, data)

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
        self.hpf_btn = QtWidgets.QPushButton('1Hz HPF: OFF')
        self.hpf_btn.setCheckable(True)
        self.hpf_btn.setChecked(False)
        self.hpf_btn.setStyleSheet('background-color: #2a2a2a; color: #aaaaaa; padding: 4px 12px;')
        self.hpf_btn.clicked.connect(self.toggle_hpf)
        top_bar.addWidget(self.hpf_btn)

        self.lpf_btn = QtWidgets.QPushButton('50Hz LPF: OFF')
        self.lpf_btn.setCheckable(True)
        self.lpf_btn.setChecked(False)
        self.lpf_btn.setStyleSheet('background-color: #2a2a2a; color: #aaaaaa; padding: 4px 12px;')
        self.lpf_btn.clicked.connect(self.toggle_lpf)
        top_bar.addWidget(self.lpf_btn)

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

        # Plots in a vertical splitter (time-domain + PSD)
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setClipToView(True)
        self.plot_widget.setDownsampling(mode='peak')
        self.curve = self.plot_widget.plot(
            pen=pg.mkPen(color=self._current_color(), width=2))
        self.splitter.addWidget(self.plot_widget)

        # PSD plot (hidden until generated)
        self.psd_widget = pg.PlotWidget()
        self.psd_widget.showGrid(x=True, y=True, alpha=0.3)
        self.psd_widget.setLabel('left', 'Power (µV²/Hz)')
        self.psd_widget.setLabel('bottom', 'Frequency (Hz)')
        self.psd_widget.setLogMode(x=False, y=True)
        self.psd_60hz_line = pg.InfiniteLine(
            pos=60, angle=90, pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine))
        self.psd_widget.addItem(self.psd_60hz_line)
        self.psd_curve = self.psd_widget.plot(
            pen=pg.mkPen(color=self._current_color(), width=2))
        self.psd_widget.hide()
        self.splitter.addWidget(self.psd_widget)

        layout.addWidget(self.splitter)

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

        # PSD controls row
        psd_bar = QtWidgets.QHBoxLayout()
        psd_bar.addWidget(QtWidgets.QLabel('PSD:'))

        psd_bar.addWidget(QtWidgets.QLabel('Start'))
        self.start_spin = QtWidgets.QDoubleSpinBox()
        self.start_spin.setRange(0.0, self.timestamps[-1])
        self.start_spin.setValue(0.0)
        self.start_spin.setSuffix(' s')
        self.start_spin.setSingleStep(1.0)
        self.start_spin.setDecimals(2)
        psd_bar.addWidget(self.start_spin)

        psd_bar.addWidget(QtWidgets.QLabel('End'))
        self.end_spin = QtWidgets.QDoubleSpinBox()
        self.end_spin.setRange(0.0, self.timestamps[-1])
        self.end_spin.setValue(self.timestamps[-1])
        self.end_spin.setSuffix(' s')
        self.end_spin.setSingleStep(1.0)
        self.end_spin.setDecimals(2)
        psd_bar.addWidget(self.end_spin)

        self.psd_btn = QtWidgets.QPushButton('Generate PSD')
        self.psd_btn.setStyleSheet('background-color: #6a1b9a; color: white; padding: 4px 12px;')
        self.psd_btn.clicked.connect(self.generate_psd)
        psd_bar.addWidget(self.psd_btn)

        self.export_btn = QtWidgets.QPushButton('Export')
        self.export_btn.setStyleSheet('background-color: #1565c0; color: white; padding: 4px 12px;')
        self.export_btn.clicked.connect(self.export_psd)
        psd_bar.addWidget(self.export_btn)

        psd_bar.addStretch()

        psd_bar.addWidget(QtWidgets.QLabel('Y-axis:'))
        self.yaxis_spin = QtWidgets.QDoubleSpinBox()
        self.yaxis_spin.setRange(0, 1e9)
        self.yaxis_spin.setValue(0)
        self.yaxis_spin.setDecimals(0)
        self.yaxis_spin.setSpecialValueText('Auto')
        self.yaxis_spin.setSingleStep(100)
        self.yaxis_spin.valueChanged.connect(self.update_plot)
        psd_bar.addWidget(self.yaxis_spin)

        layout.addLayout(psd_bar)

        # SNR controls row
        snr_bar = QtWidgets.QHBoxLayout()
        snr_bar.addWidget(QtWidgets.QLabel('SNR:'))

        snr_bar.addWidget(QtWidgets.QLabel('Signal'))
        t0 = self.timestamps[0]
        t_end = self.timestamps[-1]

        self.snr_sig_start = QtWidgets.QDoubleSpinBox()
        self.snr_sig_start.setRange(t0, t_end)
        self.snr_sig_start.setValue(t0)
        self.snr_sig_start.setSuffix(' s')
        self.snr_sig_start.setSingleStep(0.5)
        self.snr_sig_start.setDecimals(2)
        snr_bar.addWidget(self.snr_sig_start)

        snr_bar.addWidget(QtWidgets.QLabel('-'))
        self.snr_sig_end = QtWidgets.QDoubleSpinBox()
        self.snr_sig_end.setRange(t0, t_end)
        self.snr_sig_end.setValue(min(t0 + 5.0, t_end))
        self.snr_sig_end.setSuffix(' s')
        self.snr_sig_end.setSingleStep(0.5)
        self.snr_sig_end.setDecimals(2)
        snr_bar.addWidget(self.snr_sig_end)

        snr_bar.addWidget(QtWidgets.QLabel('  Noise'))
        self.snr_noise_start = QtWidgets.QDoubleSpinBox()
        self.snr_noise_start.setRange(t0, t_end)
        self.snr_noise_start.setValue(t0)
        self.snr_noise_start.setSuffix(' s')
        self.snr_noise_start.setSingleStep(0.5)
        self.snr_noise_start.setDecimals(2)
        snr_bar.addWidget(self.snr_noise_start)

        snr_bar.addWidget(QtWidgets.QLabel('-'))
        self.snr_noise_end = QtWidgets.QDoubleSpinBox()
        self.snr_noise_end.setRange(t0, t_end)
        self.snr_noise_end.setValue(min(t0 + 5.0, t_end))
        self.snr_noise_end.setSuffix(' s')
        self.snr_noise_end.setSingleStep(0.5)
        self.snr_noise_end.setDecimals(2)
        snr_bar.addWidget(self.snr_noise_end)

        self.snr_btn = QtWidgets.QPushButton('Compute SNR')
        self.snr_btn.setStyleSheet('background-color: #00695c; color: white; padding: 4px 12px;')
        self.snr_btn.clicked.connect(self.compute_snr)
        snr_bar.addWidget(self.snr_btn)

        self.snr_label = QtWidgets.QLabel('')
        self.snr_label.setStyleSheet('color: #00e676; font-weight: bold; padding: 0 8px;')
        snr_bar.addWidget(self.snr_label)

        snr_bar.addStretch()
        layout.addLayout(snr_bar)

        # Store last PSD data for export
        self._last_psd = None  # (t_start, t_end, channel_name, timestamps, segment_uv, freqs, psd)

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
        h = self.hierarchy
        available_devs = h['port_devs'].get(self.sel_port, [])
        # If current device not available under new port, select the first one
        if self.sel_dev not in available_devs:
            self.sel_dev = available_devs[0] if available_devs else self.sel_dev
        # Cascade: ensure channel is valid for new port+device
        available_chs = h['port_dev_chs'].get((self.sel_port, self.sel_dev), [])
        if self.sel_ch not in available_chs:
            self.sel_ch = available_chs[0] if available_chs else self.sel_ch
        self._update_visible_buttons()
        self._apply_hierarchy_selection()

    def _pick_dev(self, name):
        self.sel_dev = name
        h = self.hierarchy
        # Ensure channel is valid for new port+device combo
        available_chs = h['port_dev_chs'].get((self.sel_port, self.sel_dev), [])
        if self.sel_ch not in available_chs:
            self.sel_ch = available_chs[0] if available_chs else self.sel_ch
        self._update_visible_buttons()
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

    def _update_visible_buttons(self):
        """Show only the device buttons valid for the selected port,
        and only the channel buttons valid for the selected port+device."""
        h = self.hierarchy
        visible_devs = set(h['port_devs'].get(self.sel_port, []))
        visible_chs = set(h['port_dev_chs'].get((self.sel_port, self.sel_dev), []))
        for name, btn in self.dev_btns.items():
            btn.setVisible(name in visible_devs)
        for name, btn in self.ch_btns.items():
            btn.setVisible(name in visible_chs)

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

    def _get_filtered_data(self):
        """Return filtered data for the selected channel, using cache."""
        cache_key = (self.selected_channel, self.notch_on, self.hpf_on, self.lpf_on)
        if self._filter_cache is not None and self._filter_cache[0] == cache_key:
            return self._filter_cache[1]

        data = self.channel_data[:, self.selected_channel].copy()
        data = data / 24.0  # remove 24x PGA gain

        if self.hpf_on:
            hpf_sos = sig.butter(10, 1.0, btype='high', fs=self.sps, output='sos')
            data = sig.sosfiltfilt(hpf_sos, data)

        if self.lpf_on:
            lpf_sos = sig.butter(10, 50.0, btype='low', fs=self.sps, output='sos')
            data = sig.sosfiltfilt(lpf_sos, data)

        if self.notch_on:
            notch_b, notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sps)
            data = sig.filtfilt(notch_b, notch_a, data)

        self._filter_cache = (cache_key, data)
        return data

    def update_plot(self):
        start = self.slider.value()
        end = min(start + self.window_size, self.total)

        t = self.timestamps[start:end]
        v = self._get_filtered_data()[start:end]

        self.curve.setData(t, v)
        self.plot_widget.setXRange(t[0], t[-1], padding=0)

        y_lock = self.yaxis_spin.value()
        if y_lock > 0:
            y_center = float(v.mean())
            y_min, y_max = y_center - y_lock, y_center + y_lock
            self.plot_widget.setYRange(y_min, y_max, padding=0)
        else:
            y_min, y_max = float(v.min()), float(v.max())
            margin = max((y_max - y_min) * 0.1, 1000)
            y_min, y_max = y_min - margin, y_max + margin
            self.plot_widget.setYRange(y_min, y_max, padding=0)

        self.status.setText(
            f'{self._current_name()}  |  '
            f't = {t[0]:.3f} – {t[-1]:.3f} s  |  '
            f'samples {self.samples[start]}–{self.samples[end-1]}  |  '
            f'{end - start} pts shown  |  '
            f'range [{y_min:.0f}, {y_max:.0f}]'
        )

    def generate_psd(self):
        LSB_UV = 4.5 / (8388608 * 24) * 1e6  # raw ADC to µV

        t_start = self.start_spin.value()
        t_end = self.end_spin.value()
        if t_end <= t_start:
            return

        i_start = int(np.searchsorted(self.timestamps, t_start))
        i_end = int(np.searchsorted(self.timestamps, t_end))
        if i_end - i_start < 4:
            return

        segment = self.channel_data[i_start:i_end, self.selected_channel].copy()
        segment = segment * LSB_UV  # convert to µV
        segment -= segment.mean()  # remove DC

        # Apply active filters
        if self.hpf_on:
            hpf_sos = sig.butter(10, 1.0, btype='high', fs=self.sps, output='sos')
            segment = sig.sosfiltfilt(hpf_sos, segment)

        if self.lpf_on:
            lpf_sos = sig.butter(10, 50.0, btype='low', fs=self.sps, output='sos')
            segment = sig.sosfiltfilt(lpf_sos, segment)

        if self.notch_on:
            notch_b, notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sps)
            segment = sig.filtfilt(notch_b, notch_a, segment)

        nperseg = min(1024, len(segment))
        freqs, psd = sig.welch(segment, fs=self.sps, nperseg=nperseg,
                               window='hann', noverlap=nperseg // 2)

        # Filter out DC bin (freq=0) since log(0) is undefined
        mask = freqs > 0
        freqs = freqs[mask]
        psd = psd[mask]

        # Store segment timestamps and µV data for export
        seg_timestamps = self.timestamps[i_start:i_end]
        self._last_psd = (t_start, t_end, self._current_name(),
                          seg_timestamps, segment, freqs, psd)

        self.psd_curve.setData(freqs, psd)
        self.psd_curve.setPen(pg.mkPen(color=self._current_color(), width=2))
        self.psd_widget.setTitle(
            f'PSD: {self._current_name()} [{t_start:.1f}s – {t_end:.1f}s]')
        self.psd_widget.show()

    def export_psd(self):
        if self._last_psd is None:
            return
        t_start, t_end, ch_name, seg_ts, seg_uv, freqs, psd = self._last_psd

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, 'Export PSD Data', f'psd_{ch_name}_{t_start:.0f}s-{t_end:.0f}s.csv',
            'CSV Files (*.csv)')
        if not path:
            return

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            # Time-domain segment
            writer.writerow(['Time-domain segment'])
            writer.writerow(['timestamp_s', f'{ch_name}_uV'])
            for t, v in zip(seg_ts, seg_uv):
                writer.writerow([f'{t:.6f}', f'{v:.6f}'])
            writer.writerow([])
            # PSD
            writer.writerow(['PSD (Welch)'])
            writer.writerow(['frequency_Hz', 'power_uV2_per_Hz'])
            for freq, power in zip(freqs, psd):
                writer.writerow([f'{freq:.4f}', f'{power:.6e}'])

    def compute_snr(self):
        LSB_UV = 4.5 / (8388608 * 24) * 1e6

        sig_start = self.snr_sig_start.value()
        sig_end = self.snr_sig_end.value()
        noise_start = self.snr_noise_start.value()
        noise_end = self.snr_noise_end.value()

        if sig_end <= sig_start or noise_end <= noise_start:
            self.snr_label.setText('Invalid range')
            return

        # Extract signal segment
        i0 = int(np.searchsorted(self.timestamps, sig_start))
        i1 = int(np.searchsorted(self.timestamps, sig_end))
        if i1 - i0 < 2:
            self.snr_label.setText('Signal too short')
            return
        signal_seg = self.channel_data[i0:i1, self.selected_channel] * LSB_UV

        # Extract noise segment
        j0 = int(np.searchsorted(self.timestamps, noise_start))
        j1 = int(np.searchsorted(self.timestamps, noise_end))
        if j1 - j0 < 2:
            self.snr_label.setText('Noise too short')
            return
        noise_seg = self.channel_data[j0:j1, self.selected_channel] * LSB_UV

        # Apply active filters
        for segment in [signal_seg, noise_seg]:
            pass  # filters applied below on copies

        signal_seg = signal_seg.copy()
        noise_seg = noise_seg.copy()

        if self.hpf_on:
            hpf_sos = sig.butter(10, 1.0, btype='high', fs=self.sps, output='sos')
            signal_seg = sig.sosfiltfilt(hpf_sos, signal_seg)
            noise_seg = sig.sosfiltfilt(hpf_sos, noise_seg)
        if self.lpf_on:
            lpf_sos = sig.butter(10, 50.0, btype='low', fs=self.sps, output='sos')
            signal_seg = sig.sosfiltfilt(lpf_sos, signal_seg)
            noise_seg = sig.sosfiltfilt(lpf_sos, noise_seg)
        if self.notch_on:
            notch_b, notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sps)
            signal_seg = sig.filtfilt(notch_b, notch_a, signal_seg)
            noise_seg = sig.filtfilt(notch_b, notch_a, noise_seg)

        # Remove DC from both
        signal_seg -= signal_seg.mean()
        noise_seg -= noise_seg.mean()

        rms_signal = np.sqrt(np.mean(signal_seg ** 2))
        rms_noise = np.sqrt(np.mean(noise_seg ** 2))

        if rms_noise < 1e-12:
            self.snr_label.setText('Noise RMS ~ 0')
            return

        snr_db = 20 * np.log10(rms_signal / rms_noise)

        self.snr_label.setText(
            f'{self._current_name()}:  SNR = {snr_db:.1f} dB  |  '
            f'Signal RMS = {rms_signal:.2f} uV  |  Noise RMS = {rms_noise:.2f} uV')

    def _toggle_filter(self, attr, btn, on_label, off_label):
        setattr(self, attr, not getattr(self, attr))
        on = getattr(self, attr)
        btn.setText(on_label if on else off_label)
        btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 4px 12px;' if on
            else 'background-color: #2a2a2a; color: #aaaaaa; padding: 4px 12px;')
        self._filter_cache = None
        self.update_plot()

    def toggle_hpf(self):
        self._toggle_filter('hpf_on', self.hpf_btn, '1Hz HPF: ON', '1Hz HPF: OFF')

    def toggle_lpf(self):
        self._toggle_filter('lpf_on', self.lpf_btn, '50Hz LPF: ON', '50Hz LPF: OFF')

    def toggle_notch(self):
        self._toggle_filter('notch_on', self.notch_btn, '60Hz Notch: ON', '60Hz Notch: OFF')

    def keyPressEvent(self, event):
        step = max(1, self.window_size // 4)
        key = event.key()
        if key == QtCore.Qt.Key_P:
            self.generate_psd()
        elif key == QtCore.Qt.Key_N:
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
