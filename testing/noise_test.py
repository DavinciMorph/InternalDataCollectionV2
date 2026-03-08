#!/usr/bin/env uv run script
# /// script
# dependencies = [
#   "numpy",
#   "scipy",
#   "lz4",
#   "PyQt5",
#   "pyqtgraph",
# ]
# ///
"""
ADS1299 Noise Characterization Test Tool
=========================================

Live view + record + analyze for noise floor characterization.
Connects to the C++ acquisition server, shows 8 channels for a
selected port/device, records ALL channels to CSV, then runs
noise analysis on the selected device.

Usage:
    uv run noise_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1
    uv run noise_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1 \
                         --output noise_test_B01_shorted_INT.csv --duration 120
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

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

pg.setConfigOptions(useOpenGL=False, antialias=False)

# ============================================================================
# CONSTANTS
# ============================================================================

FS = 250                # Sample rate (Hz)
VREF = 4.5              # ADS1299 reference voltage (V)
GAIN = 24               # PGA gain
LSB_UV = (VREF / (2**23) / GAIN) * 1e6  # µV per LSB = 0.02235 µV

DEFAULT_DURATION = 120  # Default recording duration (seconds)
DEFAULT_SETTLE_DISCARD = 500  # Samples to discard from analysis (2s settling)

# ============================================================================
# THEME
# ============================================================================

DARK_BG = "#0d0d0d"
PLOT_BG = "#0a0a0a"
TEXT_COLOR = "#e0e0e0"
ACCENT_COLOR = "#00ff88"
RECORD_COLOR = "#ff4444"

NEON_COLORS = [
    "#00ffff", "#ff00ff", "#00ff00", "#ffff00",
    "#ff6600", "#ff0066", "#6600ff", "#00ffaa",
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
QTableWidget {{
    background-color: #1a1a1a;
    color: {TEXT_COLOR};
    gridline-color: #333;
    border: 1px solid #333;
    font-size: 10pt;
}}
QTableWidget::item {{
    padding: 4px 8px;
}}
QHeaderView::section {{
    background-color: #2a2a2a;
    color: {TEXT_COLOR};
    border: 1px solid #333;
    padding: 4px 8px;
    font-weight: bold;
}}
"""


# ============================================================================
# PORT CONFIGURATION (auto-detected from server)
# ============================================================================

@dataclass
class SPIPortConfig:
    name: str
    num_devices: int

    @property
    def num_channels(self) -> int:
        return self.num_devices * 8


# ============================================================================
# CSV WRITER THREAD (records ALL channels)
# ============================================================================

class CSVRecorderThread:
    """Records all channels to CSV during the recording window."""

    def __init__(self, filepath, channel_names):
        self._filepath = filepath
        self._channel_names = channel_names
        self._num_channels = len(channel_names)
        self._queue = Queue(maxsize=50000)
        self._running = False
        self._thread = None
        self._total_written = 0

    @property
    def total_written(self):
        return self._total_written

    def start(self):
        if self._running:
            return
        self._running = True
        self._total_written = 0
        self._thread = threading.Thread(target=self._run, name="csv-recorder", daemon=True)
        self._thread.start()

    def push(self, sample):
        if not self._running:
            return
        try:
            self._queue.put_nowait(sample)
        except Exception:
            pass

    def stop(self):
        if not self._running:
            return
        self._running = False
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=10.0)
            self._thread = None

    def _run(self):
        f = None
        try:
            f = open(self._filepath, 'w', buffering=65536)
            header = "timestamp,sample_number," + ",".join(self._channel_names) + "\n"
            f.write(header)

            while True:
                try:
                    sample = self._queue.get(timeout=0.1)
                except Empty:
                    if not self._running:
                        break
                    continue

                if sample is None:
                    break

                self._write_sample(f, sample)
                self._total_written += 1

                # Drain burst
                for _ in range(500):
                    try:
                        sample = self._queue.get_nowait()
                    except Empty:
                        break
                    if sample is None:
                        break
                    self._write_sample(f, sample)
                    self._total_written += 1

            # Drain remaining
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
            print(f"[csv] Recorder error: {e}")
            traceback.print_exc()
        finally:
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass

    def _write_sample(self, f, sample):
        channels = sample['channels']
        num_ch = min(len(channels), self._num_channels)
        parts = [f"{sample['timestamp']:.6f}", str(sample['sample_number'])]
        for i in range(num_ch):
            parts.append(str(channels[i]))
        for i in range(num_ch, self._num_channels):
            parts.append('0')
        f.write(','.join(parts))
        f.write('\n')


# ============================================================================
# NETWORK THREAD (from simpleviz.py pattern)
# ============================================================================

class NetworkThread(QtCore.QThread):
    """Receives binary LZ4 frames from C++ acquisition server with auto-reconnect."""

    config_received = QtCore.pyqtSignal(list)
    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()

    def __init__(self, host, port, sample_queue):
        super().__init__()
        self.host = host
        self.port = port
        self.sample_queue = sample_queue
        self.running = True
        self.buffer = b''
        self.csv_recorder = None  # Set externally during recording

    def run(self):
        import lz4.frame

        ip = self.host
        if ip.endswith('.local'):
            ip = socket.gethostbyname(ip)

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
                self.connected.emit()

                self.buffer = b''
                line = self._recv_line(sock)
                metadata = json.loads(line)
                print(f"[net] Metadata: {metadata}")
                attempt = 0

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

                            # Feed to CSV recorder if active
                            csv_r = self.csv_recorder
                            if csv_r is not None:
                                csv_r.push(sample)

                            try:
                                self.sample_queue.put_nowait(sample)
                            except Exception:
                                try:
                                    self.sample_queue.get_nowait()
                                except Empty:
                                    pass
                                try:
                                    self.sample_queue.put_nowait(sample)
                                except Exception:
                                    pass
                    except socket.timeout:
                        continue

            except Exception as e:
                if self.running:
                    print(f"[net] Error: {type(e).__name__}: {e} — reconnecting in 2s")
                    self.disconnected.emit()
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
# NOISE ANALYSIS
# ============================================================================

def run_noise_analysis(csv_path, port_filter, device_filter, settle_discard=DEFAULT_SETTLE_DISCARD):
    """Analyze a recorded CSV file for the specified port/device.

    Returns:
        results: list of dicts per channel with noise metrics + PSD arrays
        summary: dict with mean values across channels
    """
    import csv as csv_mod

    print(f"\n{'='*60}")
    print(f"  NOISE ANALYSIS: {os.path.basename(csv_path)}")
    print(f"  Device: {port_filter} Dev{device_filter}")
    print(f"{'='*60}")

    with open(csv_path, 'r') as f:
        reader = csv_mod.reader(f)
        header = next(reader)
        rows = list(reader)

    n_samples = len(rows)
    print(f"  Total samples: {n_samples}")
    print(f"  Settling discard: {settle_discard} samples ({settle_discard/FS:.1f}s)")

    if n_samples <= settle_discard:
        print("  ERROR: Not enough samples for analysis")
        return [], {}

    # Find the 8 channels for this port/device
    # Column names: PortN_devM_chK
    prefix = f"{port_filter}_dev{device_filter}_"
    ch_indices = []
    ch_names = []
    for i, name in enumerate(header):
        if name.startswith(prefix):
            ch_indices.append(i)
            ch_names.append(name)

    if len(ch_indices) != 8:
        print(f"  ERROR: Expected 8 channels matching '{prefix}*', found {len(ch_indices)}: {ch_names}")
        return [], {}

    # Identify timestamp and sample_number columns
    ts_col = header.index('timestamp')
    sn_col = header.index('sample_number')

    # Write device-only CSV (ALL samples, not just post-settle)
    base, ext = os.path.splitext(csv_path)
    device_csv_path = f"{base}_{port_filter}_dev{device_filter}{ext}"
    try:
        with open(device_csv_path, 'w', buffering=65536) as df:
            df.write("timestamp,sample_number," + ",".join(ch_names) + "\n")
            for row in rows:
                if len(row) < max(ch_indices) + 1:
                    continue
                parts = [row[ts_col], row[sn_col]]
                for col_idx in ch_indices:
                    parts.append(row[col_idx])
                df.write(','.join(parts))
                df.write('\n')
        print(f"  Device CSV: {device_csv_path} ({n_samples} samples, 8 channels)")
    except Exception as e:
        print(f"  WARNING: Failed to write device CSV: {e}")

    # Extract data for analysis channels (skip settling period)
    data = np.zeros((n_samples - settle_discard, 8))
    for row_idx in range(settle_discard, n_samples):
        row = rows[row_idx]
        if len(row) < max(ch_indices) + 1:
            continue
        for ch_i, col_idx in enumerate(ch_indices):
            try:
                data[row_idx - settle_discard, ch_i] = int(row[col_idx])
            except (ValueError, IndexError):
                pass

    n_analysis = data.shape[0]
    duration = n_analysis / FS
    print(f"  Analysis samples: {n_analysis} ({duration:.1f}s)")
    print(f"  ADC conversion: LSB = {LSB_UV:.5f} uV (VREF={VREF}V, gain={GAIN})")

    # Convert to microvolts
    data_uv = data * LSB_UV

    # Design notch filters for 60 Hz, 70 Hz, and harmonics (applied to analysis data)
    notch_freqs = [60.0, 70.0, 120.0]  # 60 Hz + 70 Hz + first harmonic
    notch_filters = []
    for f0 in notch_freqs:
        if f0 < FS / 2:  # only if below Nyquist
            b, a = sig.iirnotch(f0, Q=30.0, fs=FS)
            notch_filters.append((b, a))

    results = []
    for ch_i in range(8):
        ch_raw = data_uv[:, ch_i]

        # Remove DC offset
        ch_raw_dc = ch_raw - np.mean(ch_raw)

        # --- 60 Hz power measured on RAW data (before notch) ---
        nperseg = min(1024, n_analysis)
        freqs_raw, psd_raw = sig.welch(ch_raw_dc, fs=FS, nperseg=nperseg,
                                       window='hann', noverlap=nperseg // 2)
        hz60_mask = (freqs_raw >= 59) & (freqs_raw <= 61)
        if np.any(hz60_mask):
            freq_res = freqs_raw[1] - freqs_raw[0] if len(freqs_raw) > 1 else 1.0
            power_60hz = np.sqrt(np.sum(psd_raw[hz60_mask]) * freq_res)
        else:
            power_60hz = 0.0

        # --- Apply 60 Hz notch filter (+ harmonics) for all other metrics ---
        ch_filtered = ch_raw_dc.copy()
        for b, a in notch_filters:
            ch_filtered = sig.filtfilt(b, a, ch_filtered)

        # RMS noise (uV) — on filtered data
        rms = np.sqrt(np.mean(ch_filtered**2))

        # Peak-to-peak (uV) — on filtered data
        p2p = np.ptp(ch_filtered)

        # Welch PSD on filtered data (uV^2/Hz)
        freqs, psd = sig.welch(ch_filtered, fs=FS, nperseg=nperseg, window='hann',
                               noverlap=nperseg // 2)

        # Noise spectral density (uV/sqrt(Hz)) — mean across 1-100 Hz band
        band_mask = (freqs >= 1) & (freqs <= 100)
        if np.any(band_mask):
            nsd = np.sqrt(np.mean(psd[band_mask]))
        else:
            nsd = 0.0

        # SNR: signal power vs noise power (on filtered PSD)
        # Signal = 1 Hz component (look for peak near 1 Hz in PSD)
        hz1_mask = (freqs >= 0.5) & (freqs <= 1.5)
        hz60_mask_filt = (freqs >= 59) & (freqs <= 61)
        if np.any(hz1_mask):
            signal_power = np.max(psd[hz1_mask])
            # Noise = median of PSD outside signal band
            noise_mask = (freqs >= 2) & (freqs <= 100) & ~hz60_mask_filt
            if np.any(noise_mask):
                noise_power = np.median(psd[noise_mask])
                if noise_power > 0:
                    snr = 10 * np.log10(signal_power / noise_power)
                else:
                    snr = float('inf')
            else:
                snr = float('nan')
        else:
            snr = float('nan')

        # Find dominant frequency peaks (on filtered PSD)
        # Skip DC (index 0), look for local maxima in PSD
        psd_search = psd.copy()
        psd_search[0] = 0  # ignore DC
        top_peaks = _find_dominant_peaks(freqs, psd_search, n_peaks=5)

        results.append({
            'channel': ch_names[ch_i],
            'ch_num': ch_i + 1,
            'rms_uv': rms,
            'p2p_uv': p2p,
            'snr_db': snr,
            'nsd_uv_rthz': nsd,
            'power_60hz_uv': power_60hz,
            'psd_freqs': freqs,
            'psd_power': psd,
            'dominant_peaks': top_peaks,  # list of (freq_hz, power_uv2hz)
        })

    # Print noise metrics table
    print(f"\n  {'Channel':<12} {'RMS(uV)':>10} {'P2P(uV)':>10} {'SNR(dB)':>10} "
          f"{'NSD(uV/rHz)':>12} {'60Hz(uV)':>10}")
    print(f"  {'-'*12} {'-'*10} {'-'*10} {'-'*10} {'-'*12} {'-'*10}")
    for r in results:
        snr_str = f"{r['snr_db']:.1f}" if np.isfinite(r['snr_db']) else "N/A"
        print(f"  Ch{r['ch_num']:<9} {r['rms_uv']:>10.3f} {r['p2p_uv']:>10.3f} "
              f"{snr_str:>10} {r['nsd_uv_rthz']:>12.4f} {r['power_60hz_uv']:>10.3f}")

    # Print dominant frequencies table
    print(f"\n  --- Dominant Frequencies (top 5 peaks per channel) ---")
    print(f"  {'Channel':<10}", end="")
    for i in range(5):
        print(f" {'Peak'+str(i+1)+' (Hz)':>12} {'Power':>10}", end="")
    print()
    print(f"  {'-'*10}", end="")
    for i in range(5):
        print(f" {'-'*12} {'-'*10}", end="")
    print()
    for r in results:
        print(f"  Ch{r['ch_num']:<7}", end="")
        for freq, power in r['dominant_peaks']:
            print(f" {freq:>12.1f} {power:>10.2f}", end="")
        # Pad if fewer than 5 peaks
        for _ in range(5 - len(r['dominant_peaks'])):
            print(f" {'--':>12} {'--':>10}", end="")
        print()

    # Cross-channel dominant frequency consensus
    all_peak_freqs = []
    for r in results:
        for freq, power in r['dominant_peaks'][:3]:  # top 3 per channel
            all_peak_freqs.append(freq)
    if all_peak_freqs:
        # Bin nearby frequencies (within 0.5 Hz) and count
        freq_bins = _bin_frequencies(all_peak_freqs, tolerance=0.5)
        print(f"\n  --- Frequency Consensus (across all 8 channels) ---")
        print(f"  {'Frequency (Hz)':>15} {'Channels':>10} {'Occurrence':>12}")
        print(f"  {'-'*15} {'-'*10} {'-'*12}")
        for freq, count in freq_bins[:8]:
            print(f"  {freq:>15.1f} {count:>10}/8 {'<< DOMINANT' if count >= 6 else ''}")

    # Summary
    mean_rms = np.mean([r['rms_uv'] for r in results])
    mean_p2p = np.mean([r['p2p_uv'] for r in results])
    finite_snrs = [r['snr_db'] for r in results if np.isfinite(r['snr_db'])]
    mean_snr = np.mean(finite_snrs) if finite_snrs else float('nan')
    mean_nsd = np.mean([r['nsd_uv_rthz'] for r in results])
    mean_60hz = np.mean([r['power_60hz_uv'] for r in results])

    summary = {
        'mean_rms_uv': mean_rms,
        'mean_p2p_uv': mean_p2p,
        'mean_snr_db': mean_snr,
        'mean_nsd_uv_rthz': mean_nsd,
        'mean_60hz_uv': mean_60hz,
    }

    snr_str = f"{mean_snr:.1f}" if np.isfinite(mean_snr) else "N/A"
    print(f"\n  Mean RMS: {mean_rms:.3f} uV | Mean P2P: {mean_p2p:.3f} uV | "
          f"Mean SNR: {snr_str} dB | Mean NSD: {mean_nsd:.4f} uV/rHz")
    print(f"{'='*60}\n")

    return results, summary


def _find_dominant_peaks(freqs, psd, n_peaks=5):
    """Find the top N frequency peaks in a PSD.

    Returns list of (frequency_hz, power_uv2_per_hz) sorted by power descending.
    Uses scipy.signal.find_peaks with prominence to avoid picking noise ripples.
    """
    if len(psd) < 3:
        return []

    # Find peaks with minimum prominence (10% of max PSD value)
    min_prominence = np.max(psd) * 0.01
    peak_indices, properties = sig.find_peaks(psd, prominence=min_prominence)

    if len(peak_indices) == 0:
        # Fallback: just pick the N highest PSD bins (excluding DC)
        sorted_idx = np.argsort(psd)[::-1]
        peaks = []
        for idx in sorted_idx[:n_peaks]:
            if freqs[idx] > 0.1:  # skip DC
                peaks.append((float(freqs[idx]), float(psd[idx])))
        return peaks

    # Sort peaks by PSD power descending
    peak_powers = psd[peak_indices]
    sorted_order = np.argsort(peak_powers)[::-1]

    peaks = []
    for i in sorted_order[:n_peaks]:
        idx = peak_indices[i]
        peaks.append((float(freqs[idx]), float(psd[idx])))

    return peaks


def _bin_frequencies(freq_list, tolerance=0.5):
    """Group nearby frequencies and count occurrences.

    Returns list of (representative_freq, count) sorted by count descending.
    """
    if not freq_list:
        return []

    sorted_freqs = sorted(freq_list)
    bins = []  # list of (sum_freq, count)
    current_sum = sorted_freqs[0]
    current_count = 1

    for f in sorted_freqs[1:]:
        if f - (current_sum / current_count) <= tolerance:
            current_sum += f
            current_count += 1
        else:
            bins.append((current_sum / current_count, current_count))
            current_sum = f
            current_count = 1
    bins.append((current_sum / current_count, current_count))

    # Sort by count descending, then by frequency
    bins.sort(key=lambda x: (-x[1], x[0]))
    return bins


# ============================================================================
# MAIN WINDOW
# ============================================================================

class NoiseTestWindow(QtWidgets.QMainWindow):
    """Noise characterization test GUI."""

    def __init__(self, host, port, port_filter, device_filter,
                 output_path, duration, server_port):
        super().__init__()

        self.host = host
        self.server_port = server_port
        self.port_filter = port_filter
        self.device_filter = device_filter
        self.output_path = output_path
        self.duration = duration

        self.sample_rate = FS
        self.window_seconds = 5.0
        self.max_samples = int(self.sample_rate * self.window_seconds)

        # Network state
        self.spi_ports: List[SPIPortConfig] = []
        self.port_configs_raw = []  # Raw dicts from server
        self.total_channels = 0
        self.is_connected = False

        # Data buffers
        self.sample_queue = Queue(maxsize=5000)
        self.buf_size = self.max_samples * 2
        self.time_buf = np.zeros((8, self.buf_size))
        self.data_buf = np.zeros((8, self.buf_size))
        self.buf_idx = 0
        self.buf_count = 0

        # Notch filter
        self.notch_on = True
        self.notch_b, self.notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sample_rate)
        self.notch_zi_template = sig.lfilter_zi(self.notch_b, self.notch_a)
        self.notch_states = [self.notch_zi_template.copy() for _ in range(8)]

        # Recording state
        self.recording = False
        self.csv_recorder = None
        self.record_start_time = None
        self.record_sample_count = 0

        # Channel mapping (set when config arrives)
        self._ch_offset = 0  # Offset into full channel list for selected port/device
        self._view_channels = 8

        # Frame counter
        self.frame_count = 0
        self.sample_count = 0

        self._setup_ui()
        self._start_network()

        # Update timer (50 Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(20)

        # Recording countdown timer (1 Hz)
        self.record_timer = QtCore.QTimer()
        self.record_timer.timeout.connect(self._update_recording)
        self.record_timer.setInterval(1000)

    def _setup_ui(self):
        self.setWindowTitle(f'Noise Test: {self.port_filter} Dev{self.device_filter}')
        self.setStyleSheet(STYLESHEET)
        self.resize(1200, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # --- Top bar ---
        top_bar = QtWidgets.QHBoxLayout()

        self.title_label = QtWidgets.QLabel(
            f"Noise Test: {self.port_filter} Dev{self.device_filter}")
        self.title_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 13pt; font-weight: bold;")
        top_bar.addWidget(self.title_label)

        top_bar.addStretch()

        self.status_label = QtWidgets.QLabel("Connecting...")
        self.status_label.setStyleSheet("color: #ff8800; font-size: 11pt;")
        top_bar.addWidget(self.status_label)

        layout.addLayout(top_bar)

        # --- Control bar ---
        ctrl_bar = QtWidgets.QHBoxLayout()

        self.record_btn = QtWidgets.QPushButton(f"Start Recording ({self.duration}s)")
        self.record_btn.setStyleSheet(
            f"background-color: #006633; color: white; font-size: 12pt; "
            f"font-weight: bold; padding: 10px 20px; border-radius: 6px;")
        self.record_btn.clicked.connect(self._toggle_recording)
        self.record_btn.setEnabled(False)
        ctrl_bar.addWidget(self.record_btn)

        self.sample_label = QtWidgets.QLabel("Samples: 0")
        self.sample_label.setStyleSheet("font-size: 11pt;")
        ctrl_bar.addWidget(self.sample_label)

        ctrl_bar.addStretch()

        self.notch_btn = QtWidgets.QPushButton('60Hz Notch: ON')
        self.notch_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;')
        self.notch_btn.clicked.connect(self._toggle_notch)
        ctrl_bar.addWidget(self.notch_btn)

        layout.addLayout(ctrl_bar)

        # --- 8-channel plots ---
        self._plot_widgets: List[pg.PlotWidget] = []
        self._plot_curves: List[pg.PlotDataItem] = []

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
                f"color: {color}; font-weight: bold; font-size: 9pt;")
            row.addWidget(label)

            pw = pg.PlotWidget()
            pw.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Expanding)
            pw.setBackground(PLOT_BG)
            pw.showGrid(x=True, y=True, alpha=0.2)
            pw.getAxis('left').setPen(pg.mkPen(color='#555'))
            pw.getAxis('left').setTextPen(pg.mkPen(color='#888'))
            pw.getAxis('left').setWidth(50)
            if i < 7:
                pw.getAxis('bottom').setHeight(0)
                pw.getAxis('bottom').setStyle(showValues=False)
            else:
                pw.getAxis('bottom').setPen(pg.mkPen(color='#555'))
                pw.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
            pw.setXRange(0, self.window_seconds)
            pw.setYRange(-500000, 500000)
            pw.setClipToView(True)
            pw.setDownsampling(mode='peak')

            curve = pw.plot(pen=pg.mkPen(color=color, width=1))
            self._plot_widgets.append(pw)
            self._plot_curves.append(curve)

            row.addWidget(pw, stretch=1)
            plots_layout.addLayout(row, stretch=1)

        layout.addWidget(plots_widget, stretch=1)

        # --- Results table (hidden until analysis completes) ---
        self.results_label = QtWidgets.QLabel("RESULTS")
        self.results_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 12pt; font-weight: bold; padding: 4px;")
        self.results_label.setVisible(False)
        layout.addWidget(self.results_label)

        self.results_table = QtWidgets.QTableWidget()
        self.results_table.setColumnCount(6)
        self.results_table.setHorizontalHeaderLabels(
            ["Channel", "RMS (uV)", "P2P (uV)", "SNR (dB)", "NSD (uV/rHz)", "60Hz (uV)"])
        self.results_table.horizontalHeader().setStretchLastSection(True)
        self.results_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.results_table.setVisible(False)
        self.results_table.setMaximumHeight(280)
        layout.addWidget(self.results_table)

        # --- PSD plot (hidden until analysis completes) ---
        self.psd_label = QtWidgets.QLabel("POWER SPECTRAL DENSITY")
        self.psd_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 12pt; font-weight: bold; padding: 4px;")
        self.psd_label.setVisible(False)
        layout.addWidget(self.psd_label)

        self.psd_plot = pg.PlotWidget()
        self.psd_plot.setBackground(PLOT_BG)
        self.psd_plot.showGrid(x=True, y=True, alpha=0.3)
        self.psd_plot.setLabel('bottom', 'Frequency', units='Hz')
        self.psd_plot.setLabel('left', 'Power', units='uV\u00b2/Hz')
        self.psd_plot.setLogMode(x=False, y=True)
        self.psd_plot.getAxis('left').setPen(pg.mkPen(color='#888'))
        self.psd_plot.getAxis('left').setTextPen(pg.mkPen(color='#888'))
        self.psd_plot.getAxis('bottom').setPen(pg.mkPen(color='#888'))
        self.psd_plot.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
        self.psd_plot.addLegend(offset=(60, 10), labelTextColor='#ccc',
                                labelTextSize='9pt')
        self.psd_plot.setVisible(False)
        self.psd_plot.setMinimumHeight(250)
        layout.addWidget(self.psd_plot, stretch=1)

        # --- Dominant frequencies table (hidden until analysis completes) ---
        self.freq_label = QtWidgets.QLabel("DOMINANT FREQUENCIES")
        self.freq_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 12pt; font-weight: bold; padding: 4px;")
        self.freq_label.setVisible(False)
        layout.addWidget(self.freq_label)

        self.freq_table = QtWidgets.QTableWidget()
        self.freq_table.setColumnCount(6)
        self.freq_table.setHorizontalHeaderLabels(
            ["Channel", "Peak 1 (Hz)", "Peak 2 (Hz)", "Peak 3 (Hz)",
             "Peak 4 (Hz)", "Peak 5 (Hz)"])
        self.freq_table.horizontalHeader().setStretchLastSection(True)
        self.freq_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.freq_table.setVisible(False)
        self.freq_table.setMaximumHeight(280)
        layout.addWidget(self.freq_table)

        self.summary_label = QtWidgets.QLabel("")
        self.summary_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 11pt; padding: 4px;")
        self.summary_label.setVisible(False)
        layout.addWidget(self.summary_label)

    def _start_network(self):
        self.net_thread = NetworkThread(self.host, self.server_port, self.sample_queue)
        self.net_thread.config_received.connect(self._on_config)
        self.net_thread.connected.connect(self._on_connected)
        self.net_thread.disconnected.connect(self._on_disconnected)
        self.net_thread.start()

    def _on_connected(self):
        self.is_connected = True
        self.status_label.setText("Connected")
        self.status_label.setStyleSheet(f"color: {ACCENT_COLOR}; font-size: 11pt;")

    def _on_disconnected(self):
        self.is_connected = False
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: #ff4444; font-size: 11pt;")
        self.record_btn.setEnabled(False)

    def _on_config(self, port_configs):
        """Called when server sends port configuration."""
        self.port_configs_raw = port_configs
        self.spi_ports = [
            SPIPortConfig(name=pc['name'], num_devices=pc['num_devices'])
            for pc in port_configs
        ]
        self.total_channels = sum(p.num_channels for p in self.spi_ports)

        # Find channel offset for the selected port/device
        ch_offset = 0
        found = False
        for pc in self.spi_ports:
            if pc.name == self.port_filter:
                ch_offset += (self.device_filter - 1) * 8
                found = True
                break
            ch_offset += pc.num_channels

        if not found:
            available = [pc.name for pc in self.spi_ports]
            print(f"[WARN] Port '{self.port_filter}' not found in server config. "
                  f"Available: {available}")
            self.status_label.setText(f"Port {self.port_filter} not found!")
            self.status_label.setStyleSheet("color: #ff4444; font-size: 11pt;")
            return

        self._ch_offset = ch_offset

        # Build channel names for CSV
        self._channel_names = []
        for pc in port_configs:
            port_name = pc['name']
            num_devices = pc['num_devices']
            for d in range(num_devices):
                for c in range(8):
                    self._channel_names.append(f"{port_name}_dev{d+1}_ch{c+1}")

        total_dev = sum(p.num_devices for p in self.spi_ports)
        self.status_label.setText(
            f"Connected | {len(self.spi_ports)} ports, {total_dev} dev, "
            f"{self.total_channels} ch | View: {self.port_filter} Dev{self.device_filter}")
        self.status_label.setStyleSheet(f"color: {ACCENT_COLOR}; font-size: 11pt;")

        self.record_btn.setEnabled(True)
        print(f"[cfg] Viewing channels {ch_offset}-{ch_offset+7} "
              f"({self.port_filter} Dev{self.device_filter})")

    # ------------------------------------------------------------------
    # RECORDING
    # ------------------------------------------------------------------

    def _toggle_recording(self):
        if self.recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        if self.recording or not self._channel_names:
            return

        filepath = self.output_path
        if not filepath:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"noise_test_{ts}.csv"

        self.csv_recorder = CSVRecorderThread(filepath, self._channel_names)
        self.csv_recorder.start()
        self.net_thread.csv_recorder = self.csv_recorder

        self.recording = True
        self.record_start_time = time.time()
        self.record_sample_count = 0
        self._csv_filepath = filepath

        self.record_btn.setText(f"Recording... {self.duration}s remaining")
        self.record_btn.setStyleSheet(
            f"background-color: #880000; color: white; font-size: 12pt; "
            f"font-weight: bold; padding: 10px 20px; border-radius: 6px;")

        self.record_timer.start()
        print(f"[rec] Started recording to {filepath}")

    def _stop_recording(self):
        if not self.recording:
            return

        self.recording = False
        self.record_timer.stop()
        self.net_thread.csv_recorder = None

        if self.csv_recorder is not None:
            self.csv_recorder.stop()
            written = self.csv_recorder.total_written
            print(f"[rec] Stopped: {written} samples written to {self._csv_filepath}")
            self.csv_recorder = None

        self.record_btn.setText(f"Start Recording ({self.duration}s)")
        self.record_btn.setStyleSheet(
            f"background-color: #006633; color: white; font-size: 12pt; "
            f"font-weight: bold; padding: 10px 20px; border-radius: 6px;")

        # Run analysis
        self._run_analysis()

    def _update_recording(self):
        """Called every 1s during recording."""
        if not self.recording:
            return

        elapsed = time.time() - self.record_start_time
        remaining = max(0, self.duration - elapsed)
        written = self.csv_recorder.total_written if self.csv_recorder else 0

        if remaining <= 0:
            self._stop_recording()
            return

        target = int(self.duration * FS)
        self.record_btn.setText(
            f"Recording... {int(remaining)}s remaining | {written}/{target}")

    def _run_analysis(self):
        """Run noise analysis on the recorded CSV."""
        if not os.path.exists(self._csv_filepath):
            print(f"[analysis] CSV file not found: {self._csv_filepath}")
            return

        try:
            results, summary = run_noise_analysis(
                self._csv_filepath, self.port_filter, self.device_filter)
        except Exception as e:
            print(f"[analysis] Error: {e}")
            traceback.print_exc()
            return

        if not results:
            return

        # Show results in GUI
        self.results_label.setVisible(True)
        self.results_label.setText(
            f"RESULTS — {self.port_filter} Dev{self.device_filter} "
            f"({os.path.basename(self._csv_filepath)})")

        self.results_table.setRowCount(len(results))
        for row, r in enumerate(results):
            self.results_table.setItem(row, 0,
                QtWidgets.QTableWidgetItem(f"Ch{r['ch_num']}"))
            self.results_table.setItem(row, 1,
                QtWidgets.QTableWidgetItem(f"{r['rms_uv']:.3f}"))
            self.results_table.setItem(row, 2,
                QtWidgets.QTableWidgetItem(f"{r['p2p_uv']:.3f}"))
            snr_str = f"{r['snr_db']:.1f}" if np.isfinite(r['snr_db']) else "N/A"
            self.results_table.setItem(row, 3,
                QtWidgets.QTableWidgetItem(snr_str))
            self.results_table.setItem(row, 4,
                QtWidgets.QTableWidgetItem(f"{r['nsd_uv_rthz']:.4f}"))
            self.results_table.setItem(row, 5,
                QtWidgets.QTableWidgetItem(f"{r['power_60hz_uv']:.3f}"))
        self.results_table.setVisible(True)

        # --- PSD plot: all 8 channels overlaid ---
        self.psd_plot.clear()
        if hasattr(self.psd_plot, 'legend') and self.psd_plot.legend is not None:
            self.psd_plot.legend.clear()
        for r in results:
            freqs = r.get('psd_freqs')
            psd_power = r.get('psd_power')
            if freqs is not None and psd_power is not None:
                color = NEON_COLORS[(r['ch_num'] - 1) % len(NEON_COLORS)]
                self.psd_plot.plot(freqs, psd_power,
                                  pen=pg.mkPen(color=color, width=1.5),
                                  name=f"Ch{r['ch_num']}")
        # Mark 60 Hz with a vertical line
        sixty_line = pg.InfiniteLine(pos=60, angle=90,
                                     pen=pg.mkPen(color='#ff4444', width=1,
                                                  style=QtCore.Qt.DashLine))
        self.psd_plot.addItem(sixty_line)
        self.psd_plot.setXRange(0, FS / 2, padding=0)
        self.psd_label.setVisible(True)
        self.psd_plot.setVisible(True)

        # --- Dominant frequencies table ---
        self.freq_table.setRowCount(len(results))
        for row, r in enumerate(results):
            self.freq_table.setItem(row, 0,
                QtWidgets.QTableWidgetItem(f"Ch{r['ch_num']}"))
            peaks = r.get('dominant_peaks', [])
            for col_i in range(5):
                if col_i < len(peaks):
                    freq, power = peaks[col_i]
                    cell_text = f"{freq:.1f}  ({power:.1f})"
                else:
                    cell_text = "--"
                self.freq_table.setItem(row, col_i + 1,
                    QtWidgets.QTableWidgetItem(cell_text))
        self.freq_label.setVisible(True)
        self.freq_table.setVisible(True)

        snr_str = (f"{summary['mean_snr_db']:.1f}"
                   if np.isfinite(summary['mean_snr_db']) else "N/A")
        self.summary_label.setText(
            f"Mean RMS: {summary['mean_rms_uv']:.3f} uV | "
            f"Mean P2P: {summary['mean_p2p_uv']:.3f} uV | "
            f"Mean SNR: {snr_str} dB | "
            f"Mean NSD: {summary['mean_nsd_uv_rthz']:.4f} uV/rHz")
        self.summary_label.setVisible(True)

    # ------------------------------------------------------------------
    # LIVE PLOT UPDATE
    # ------------------------------------------------------------------

    def _update(self):
        """20ms timer: drain sample queue, update plots."""
        try:
            # Drain queue
            batch_raw = []
            for _ in range(500):
                try:
                    batch_raw.append(self.sample_queue.get_nowait())
                except Empty:
                    break

            if not batch_raw:
                return

            n = len(batch_raw)
            self.sample_count += n

            # Extract the 8 view channels from each sample
            raw = np.zeros((n, 8))
            timestamps = np.zeros(n)
            for i, s in enumerate(batch_raw):
                timestamps[i] = s['timestamp']
                ch = s['channels']
                for c in range(8):
                    idx = self._ch_offset + c
                    if idx < len(ch):
                        raw[i, c] = ch[idx]

            # Apply notch filter for display
            if self.notch_on:
                for c in range(8):
                    raw[:, c], self.notch_states[c] = sig.lfilter(
                        self.notch_b, self.notch_a, raw[:, c],
                        zi=self.notch_states[c])

            # Write to ring buffer
            idx = self.buf_idx
            buf_cap = self.buf_size
            space = buf_cap - idx

            if n <= space:
                self.time_buf[:, idx:idx + n] = timestamps
                self.data_buf[:, idx:idx + n] = raw.T
                self.buf_idx += n
            else:
                if space > 0:
                    self.time_buf[:, idx:idx + space] = timestamps[:space]
                    self.data_buf[:, idx:idx + space] = raw[:space].T
                self.time_buf[:, :self.max_samples] = self.time_buf[:, self.max_samples:]
                self.data_buf[:, :self.max_samples] = self.data_buf[:, self.max_samples:]
                self.buf_idx = self.max_samples
                remaining = n - space
                if remaining > 0:
                    idx2 = self.buf_idx
                    self.time_buf[:, idx2:idx2 + remaining] = timestamps[space:]
                    self.data_buf[:, idx2:idx2 + remaining] = raw[space:].T
                    self.buf_idx += remaining

            self.buf_count = min(self.buf_count + n, self.max_samples)

            # Update status
            if self.sample_count % 250 < n:
                self.sample_label.setText(f"Samples: {self.sample_count}")

            # Update plots
            if self.buf_count == 0:
                return

            self.frame_count += 1
            rescale_y = (self.frame_count % 15 == 0)

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

            for i in range(8):
                t = self.time_buf[i, start:end]
                d = self.data_buf[i, start:end]
                self._plot_curves[i].setData(t, d)
                self._plot_widgets[i].setXRange(x_min, x_max, padding=0)

                if rescale_y and len(d) > 0:
                    y_min, y_max = float(d.min()), float(d.max())
                    margin = max((y_max - y_min) * 0.1, 10000)
                    self._plot_widgets[i].setYRange(
                        y_min - margin, y_max + margin, padding=0)

        except Exception as e:
            print(f"[update] Error: {e}")
            traceback.print_exc()

    # ------------------------------------------------------------------
    # NOTCH FILTER
    # ------------------------------------------------------------------

    def _toggle_notch(self):
        self.notch_on = not self.notch_on
        if self.notch_on:
            self.notch_btn.setText('60Hz Notch: ON')
            self.notch_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;')
            self.notch_states = [self.notch_zi_template.copy() for _ in range(8)]
        else:
            self.notch_btn.setText('60Hz Notch: OFF')
            self.notch_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;')

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_N:
            self._toggle_notch()
        elif event.key() == QtCore.Qt.Key_R:
            self._toggle_recording()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        if self.recording:
            self._stop_recording()
        self.net_thread.stop()
        self.net_thread.wait(3000)
        event.accept()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='ADS1299 Noise Characterization Test Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  uv run noise_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1
  uv run noise_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1 \\
                       --output noise_test_B01.csv --duration 120

Keyboard shortcuts:
  R  - Start/stop recording
  N  - Toggle 60 Hz notch filter
        """)
    parser.add_argument('--host', type=str, required=True,
                        help='Server IP address')
    parser.add_argument('--port', type=int, default=8888,
                        help='Server port (default: 8888)')
    parser.add_argument('--port-filter', type=str, required=True,
                        help='Port to view/analyze (e.g. Port1)')
    parser.add_argument('--device-filter', type=int, required=True,
                        help='Device number to view/analyze (1-based)')
    parser.add_argument('--output', type=str, default=None,
                        help='CSV output filename (default: noise_test_TIMESTAMP.csv)')
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION,
                        help=f'Recording duration in seconds (default: {DEFAULT_DURATION})')

    args = parser.parse_args()

    print(f"\nADS1299 Noise Characterization Test")
    print(f"{'='*50}")
    print(f"Server:  {args.host}:{args.port}")
    print(f"View:    {args.port_filter} Dev{args.device_filter}")
    print(f"Record:  {args.duration}s ({args.duration * FS} samples)")
    print(f"Output:  {args.output or 'auto-generated'}")
    print(f"ADC:     LSB = {LSB_UV:.5f} uV (VREF={VREF}V, gain={GAIN})")
    print(f"{'='*50}\n")

    app = QtWidgets.QApplication(sys.argv)

    window = NoiseTestWindow(
        host=args.host,
        port=args.port,
        port_filter=args.port_filter,
        device_filter=args.device_filter,
        output_path=args.output,
        duration=args.duration,
        server_port=args.port,
    )
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
