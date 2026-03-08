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
ADS1299 Eye Blink Test Tool
============================

Live view + record + analyze for eye blink characterization.
Connects to the C++ acquisition server, shows 8 channels for a
selected port/device with live filtering, records ALL channels to CSV,
then runs blink detection and epoch analysis on the selected device.

Usage:
    uv run blink_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1
    uv run blink_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1 \
                         --duration 120 --threshold 4.0 --detect-ch 2
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
LSB_UV = (VREF / (2**23) / GAIN) * 1e6  # uV per LSB = 0.02235 uV

DEFAULT_DURATION = 60           # Shorter blink sessions (seconds)
DEFAULT_SETTLE_DISCARD = 250    # 1s settling (samples)
OUTPUT_DIR = "eye_blinks"       # Relative to script dir

# Blink detection parameters
BLINK_BANDPASS = [1.0, 10.0]    # Hz, detection band
BLINK_THRESHOLD_STD = 3.5       # std devs above mean
BLINK_MIN_DISTANCE_S = 0.5     # 500ms between blinks
BLINK_EPOCH_PRE_S = 0.2        # 200ms before peak (50 samples)
BLINK_EPOCH_POST_S = 0.5       # 500ms after peak (125 samples)
BLINK_MIN_AMPLITUDE_UV = 30.0  # Minimum to count as blink

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
# NETWORK THREAD (from simpleviz.py / noise_test.py pattern)
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
# BLINK DETECTION & ANALYSIS (offline, on recorded CSV)
# ============================================================================

def run_blink_analysis(csv_path, port_filter, device_filter, output_dir,
                       threshold_std=BLINK_THRESHOLD_STD,
                       detect_ch=None,
                       settle_discard=DEFAULT_SETTLE_DISCARD):
    """Analyze a recorded CSV for eye blinks on the specified port/device.

    Returns:
        results: dict with all blink analysis results
    """
    import csv as csv_mod

    print(f"\n{'='*60}")
    print(f"  BLINK ANALYSIS: {os.path.basename(csv_path)}")
    print(f"  Device: {port_filter} Dev{device_filter}")
    print(f"{'='*60}")
    print(f"  Detection params:")
    print(f"    Bandpass: {BLINK_BANDPASS[0]}-{BLINK_BANDPASS[1]} Hz")
    print(f"    Threshold: {threshold_std} std")
    print(f"    Min distance: {BLINK_MIN_DISTANCE_S}s ({int(BLINK_MIN_DISTANCE_S * FS)} samples)")
    print(f"    Epoch window: -{BLINK_EPOCH_PRE_S}s to +{BLINK_EPOCH_POST_S}s")
    print(f"    Min amplitude: {BLINK_MIN_AMPLITUDE_UV} uV")
    if detect_ch is not None:
        print(f"    Detect channel: Ch{detect_ch} (user-specified)")
    else:
        print(f"    Detect channel: auto (highest variance)")

    # Load CSV
    with open(csv_path, 'r') as f:
        reader = csv_mod.reader(f)
        header = next(reader)
        rows = list(reader)

    n_samples = len(rows)
    print(f"\n  Total samples: {n_samples}")
    print(f"  Settling discard: {settle_discard} samples ({settle_discard/FS:.1f}s)")

    if n_samples <= settle_discard:
        print("  ERROR: Not enough samples for analysis")
        return None

    # Find the 8 channels for this port/device
    prefix = f"{port_filter}_dev{device_filter}_"
    ch_indices = []
    ch_names = []
    for i, name in enumerate(header):
        if name.startswith(prefix):
            ch_indices.append(i)
            ch_names.append(name)

    if len(ch_indices) != 8:
        print(f"  ERROR: Expected 8 channels matching '{prefix}*', found {len(ch_indices)}: {ch_names}")
        return None

    ts_col = header.index('timestamp')
    sn_col = header.index('sample_number')

    # Write device-only CSV (ALL samples)
    base, ext = os.path.splitext(csv_path)
    device_csv_path = os.path.join(output_dir,
        os.path.basename(base) + f"_{port_filter}_dev{device_filter}{ext}")
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

    # Extract data (skip settling period)
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

    # Step 1: Aggressive powerline removal (zero-phase)
    # Cascade: 60 Hz (x2 passes for deep null) + 120 Hz harmonic
    notch_freqs = [60.0, 60.0, 120.0]  # Two 60 Hz passes for ~60 dB rejection
    notch_filters = []
    for f0 in notch_freqs:
        if f0 < FS / 2:
            b, a = sig.iirnotch(f0, Q=30.0, fs=FS)
            notch_filters.append((b, a))

    data_notched = data_uv.copy()
    for ch_i in range(8):
        for b, a in notch_filters:
            data_notched[:, ch_i] = sig.filtfilt(b, a, data_notched[:, ch_i])

    # Report 60 Hz power before/after notch
    nperseg = min(1024, n_analysis)
    print(f"\n  60 Hz power (per channel, before/after notch):")
    for ch_i in range(8):
        _, psd_raw = sig.welch(data_uv[:, ch_i] - np.mean(data_uv[:, ch_i]),
                               fs=FS, nperseg=nperseg)
        _, psd_notched = sig.welch(data_notched[:, ch_i] - np.mean(data_notched[:, ch_i]),
                                   fs=FS, nperseg=nperseg)
        freqs_w, _ = sig.welch(data_uv[:, ch_i], fs=FS, nperseg=nperseg)
        hz60_mask = (freqs_w >= 59) & (freqs_w <= 61)
        freq_res = freqs_w[1] - freqs_w[0] if len(freqs_w) > 1 else 1.0
        p60_before = np.sqrt(np.sum(psd_raw[hz60_mask]) * freq_res) if np.any(hz60_mask) else 0
        p60_after = np.sqrt(np.sum(psd_notched[hz60_mask]) * freq_res) if np.any(hz60_mask) else 0
        reject_db = 20 * np.log10(p60_before / p60_after) if p60_after > 0 else float('inf')
        print(f"    Ch{ch_i+1}: {p60_before:.1f} -> {p60_after:.1f} uV RMS ({reject_db:.0f} dB rejection)")

    # Step 1b: Common Mode Filter — subtract device mean from each channel
    # Removes any signal common to all 8 channels (residual mains, reference drift)
    # Skip Ch8 if it looks like a bias/reference (std < 100 uV)
    ch_stds = np.std(data_notched, axis=0)
    cmf_mask = ch_stds > 100.0  # Only include "alive" channels in CMF
    n_cmf_ch = int(np.sum(cmf_mask))
    if n_cmf_ch >= 2:
        cmf_mean = np.mean(data_notched[:, cmf_mask], axis=1, keepdims=True)
        data_notched = data_notched - cmf_mean
        print(f"\n  CMF applied: subtracted mean of {n_cmf_ch} active channels "
              f"(excluded: {[f'Ch{i+1}' for i in range(8) if not cmf_mask[i]]})")
        # Report residual RMS after CMF
        print(f"  Residual RMS after notch+CMF:")
        for ch_i in range(8):
            rms = np.sqrt(np.mean(data_notched[:, ch_i]**2))
            print(f"    Ch{ch_i+1}: {rms:.1f} uV{' (ref/bias)' if not cmf_mask[ch_i] else ''}")
    else:
        print(f"\n  CMF skipped: only {n_cmf_ch} active channels (need >= 2)")

    # Step 2: Bandpass 1-10 Hz for detection only (4th order Butterworth)
    bp_sos = sig.butter(4, BLINK_BANDPASS, btype='bandpass', fs=FS, output='sos')
    data_bp = np.zeros_like(data_notched)
    for ch_i in range(8):
        data_bp[:, ch_i] = sig.sosfiltfilt(bp_sos, data_notched[:, ch_i])

    # Step 3: Auto-detect best channel (highest variance in bandpassed data)
    variances = np.var(data_bp, axis=0)
    if detect_ch is not None:
        best_ch = detect_ch - 1  # Convert 1-based to 0-based
        if best_ch < 0 or best_ch >= 8:
            print(f"  WARNING: --detect-ch {detect_ch} out of range, using auto-detect")
            best_ch = int(np.argmax(variances))
    else:
        best_ch = int(np.argmax(variances))

    print(f"\n  Channel variances (bandpassed):")
    for ch_i in range(8):
        marker = " << SELECTED" if ch_i == best_ch else ""
        print(f"    Ch{ch_i+1}: {variances[ch_i]:.2f} uV^2{marker}")

    # Step 4: Detect blinks on best channel
    detect_signal = data_bp[:, best_ch]
    envelope = np.abs(detect_signal)
    threshold = np.mean(envelope) + threshold_std * np.std(envelope)
    min_distance_samples = int(BLINK_MIN_DISTANCE_S * FS)

    print(f"\n  Detection (Ch{best_ch+1}):")
    print(f"    Envelope mean: {np.mean(envelope):.2f} uV")
    print(f"    Envelope std: {np.std(envelope):.2f} uV")
    print(f"    Threshold: {threshold:.2f} uV ({threshold_std} std)")

    # Find peaks
    raw_peaks, raw_props = sig.find_peaks(envelope, height=threshold,
                                           distance=min_distance_samples)
    print(f"    Raw peaks found: {len(raw_peaks)}")

    # Filter by minimum amplitude (measured in notched data, not bandpassed)
    valid_peaks = []
    valid_amplitudes = []
    epoch_pre = int(BLINK_EPOCH_PRE_S * FS)    # 50 samples
    epoch_post = int(BLINK_EPOCH_POST_S * FS)   # 125 samples

    for peak_idx in raw_peaks:
        # Check bounds for epoch extraction
        if peak_idx - epoch_pre < 0 or peak_idx + epoch_post >= n_analysis:
            continue
        # Measure amplitude on notched data (peak-to-trough in epoch window)
        epoch = data_notched[peak_idx - epoch_pre:peak_idx + epoch_post + 1, best_ch]
        amplitude = np.ptp(epoch)
        if amplitude >= BLINK_MIN_AMPLITUDE_UV:
            valid_peaks.append(peak_idx)
            valid_amplitudes.append(amplitude)

    valid_peaks = np.array(valid_peaks)
    valid_amplitudes = np.array(valid_amplitudes)
    print(f"    Valid peaks (>= {BLINK_MIN_AMPLITUDE_UV} uV): {len(valid_peaks)}")

    if len(valid_peaks) == 0:
        print("\n  NO BLINKS DETECTED — try lowering --threshold or check electrode placement")
        return {
            'n_blinks': 0,
            'detect_ch': best_ch,
            'variances': variances,
            'threshold': threshold,
            'ch_names': ch_names,
        }

    # Step 5: Extract epochs from notched data (all 8 channels)
    epoch_len = epoch_pre + epoch_post + 1  # 176 samples
    epochs = np.zeros((len(valid_peaks), epoch_len, 8))
    for i, peak_idx in enumerate(valid_peaks):
        epoch_slice = data_notched[peak_idx - epoch_pre:peak_idx + epoch_post + 1, :]
        # Baseline-correct: subtract mean of pre-stimulus period (-200ms to 0ms)
        baseline = np.mean(epoch_slice[:epoch_pre, :], axis=0, keepdims=True)
        epochs[i] = epoch_slice - baseline

    # Step 6: Grand average
    grand_avg = np.mean(epochs, axis=0)  # (epoch_len, 8)

    # Step 7: Per-channel peak-to-trough amplitude
    ch_amplitudes = np.zeros(8)
    for ch_i in range(8):
        ch_amplitudes[ch_i] = np.ptp(grand_avg[:, ch_i])

    # Step 8: Inter-blink interval statistics
    if len(valid_peaks) > 1:
        ibis = np.diff(valid_peaks) / FS  # seconds
        ibi_stats = {
            'mean': float(np.mean(ibis)),
            'std': float(np.std(ibis)),
            'min': float(np.min(ibis)),
            'max': float(np.max(ibis)),
            'cv': float(np.std(ibis) / np.mean(ibis)) if np.mean(ibis) > 0 else 0.0,
        }
    else:
        ibi_stats = {'mean': 0, 'std': 0, 'min': 0, 'max': 0, 'cv': 0}

    blink_rate = len(valid_peaks) / duration * 60  # blinks per minute
    mean_amplitude = float(np.mean(valid_amplitudes))

    # Print results
    print(f"\n  {'='*50}")
    print(f"  BLINK DETECTION RESULTS")
    print(f"  {'='*50}")
    print(f"  Blinks detected: {len(valid_peaks)}")
    print(f"  Blink rate: {blink_rate:.1f} /min")
    print(f"  Duration: {duration:.1f}s")
    print(f"  Mean amplitude (detect ch): {mean_amplitude:.1f} uV")
    print(f"  Amplitude SD: {float(np.std(valid_amplitudes)):.1f} uV")

    if len(valid_peaks) > 1:
        print(f"\n  Inter-blink intervals:")
        print(f"    Mean: {ibi_stats['mean']:.3f}s")
        print(f"    Std:  {ibi_stats['std']:.3f}s")
        print(f"    Min:  {ibi_stats['min']:.3f}s")
        print(f"    Max:  {ibi_stats['max']:.3f}s")
        print(f"    CV:   {ibi_stats['cv']:.3f}")

    print(f"\n  Per-channel amplitudes (grand average peak-to-trough):")
    print(f"  {'Channel':<12} {'Amplitude (uV)':>15}")
    print(f"  {'-'*12} {'-'*15}")
    for ch_i in range(8):
        marker = " << detect" if ch_i == best_ch else ""
        print(f"  Ch{ch_i+1:<9} {ch_amplitudes[ch_i]:>15.1f}{marker}")

    # Build time axis for epochs (ms)
    epoch_time_ms = np.arange(-epoch_pre, epoch_post + 1) / FS * 1000

    # Write epoch CSV
    epoch_csv_path = os.path.join(output_dir,
        os.path.basename(base) + "_epochs.csv")
    try:
        with open(epoch_csv_path, 'w') as ef:
            epoch_ch_names = [f"ch{i+1}_uV" for i in range(8)]
            ef.write("blink_number,time_ms," + ",".join(epoch_ch_names) + "\n")
            for blink_i in range(len(valid_peaks)):
                for t_i in range(epoch_len):
                    parts = [str(blink_i + 1), f"{epoch_time_ms[t_i]:.1f}"]
                    for ch_i in range(8):
                        parts.append(f"{epochs[blink_i, t_i, ch_i]:.2f}")
                    ef.write(','.join(parts) + '\n')
        print(f"\n  Epoch CSV: {epoch_csv_path}")
    except Exception as e:
        print(f"  WARNING: Failed to write epoch CSV: {e}")

    # Write summary text
    summary_path = os.path.join(output_dir,
        os.path.basename(base) + "_summary.txt")
    try:
        with open(summary_path, 'w') as sf:
            sf.write(f"Eye Blink Analysis Summary\n")
            sf.write(f"{'='*50}\n")
            sf.write(f"Source: {os.path.basename(csv_path)}\n")
            sf.write(f"Device: {port_filter} Dev{device_filter}\n")
            sf.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            sf.write(f"Recording duration: {duration:.1f}s\n")
            sf.write(f"Analysis samples: {n_analysis}\n")
            sf.write(f"Settle discard: {settle_discard} samples\n\n")
            sf.write(f"Detection Parameters\n")
            sf.write(f"{'-'*30}\n")
            sf.write(f"Bandpass: {BLINK_BANDPASS[0]}-{BLINK_BANDPASS[1]} Hz\n")
            sf.write(f"Threshold: {threshold_std} std ({threshold:.2f} uV)\n")
            sf.write(f"Min distance: {BLINK_MIN_DISTANCE_S}s\n")
            sf.write(f"Min amplitude: {BLINK_MIN_AMPLITUDE_UV} uV\n")
            sf.write(f"Detection channel: Ch{best_ch+1} (variance: {variances[best_ch]:.2f})\n\n")
            sf.write(f"Results\n")
            sf.write(f"{'-'*30}\n")
            sf.write(f"Blinks detected: {len(valid_peaks)}\n")
            sf.write(f"Blink rate: {blink_rate:.1f} /min\n")
            sf.write(f"Mean amplitude: {mean_amplitude:.1f} uV\n")
            sf.write(f"Amplitude SD: {float(np.std(valid_amplitudes)):.1f} uV\n\n")
            if len(valid_peaks) > 1:
                sf.write(f"Inter-blink Intervals\n")
                sf.write(f"{'-'*30}\n")
                sf.write(f"Mean: {ibi_stats['mean']:.3f}s\n")
                sf.write(f"Std:  {ibi_stats['std']:.3f}s\n")
                sf.write(f"Min:  {ibi_stats['min']:.3f}s\n")
                sf.write(f"Max:  {ibi_stats['max']:.3f}s\n")
                sf.write(f"CV:   {ibi_stats['cv']:.3f}\n\n")
            sf.write(f"Per-Channel Amplitudes (grand average peak-to-trough)\n")
            sf.write(f"{'-'*30}\n")
            for ch_i in range(8):
                marker = " (detection channel)" if ch_i == best_ch else ""
                sf.write(f"Ch{ch_i+1}: {ch_amplitudes[ch_i]:.1f} uV{marker}\n")
        print(f"  Summary: {summary_path}")
    except Exception as e:
        print(f"  WARNING: Failed to write summary: {e}")

    print(f"\n  Output files:")
    print(f"    {csv_path}")
    print(f"    {device_csv_path}")
    print(f"    {epoch_csv_path}")
    print(f"    {summary_path}")
    print(f"{'='*60}\n")

    return {
        'n_blinks': len(valid_peaks),
        'blink_rate': blink_rate,
        'mean_amplitude': mean_amplitude,
        'amplitude_std': float(np.std(valid_amplitudes)),
        'detect_ch': best_ch,
        'variances': variances,
        'threshold': threshold,
        'ch_names': ch_names,
        'ch_amplitudes': ch_amplitudes,
        'ibi_stats': ibi_stats,
        'epochs': epochs,
        'grand_avg': grand_avg,
        'epoch_time_ms': epoch_time_ms,
        'valid_peaks': valid_peaks,
        'valid_amplitudes': valid_amplitudes,
        'duration': duration,
    }


# ============================================================================
# MAIN WINDOW
# ============================================================================

class BlinkTestWindow(QtWidgets.QMainWindow):
    """Eye blink test GUI with live filtering and offline analysis."""

    def __init__(self, host, port, port_filter, device_filter,
                 output_dir, duration, server_port, threshold_std, detect_ch):
        super().__init__()

        self.host = host
        self.server_port = server_port
        self.port_filter = port_filter
        self.device_filter = device_filter
        self.output_dir = output_dir
        self.duration = duration
        self.threshold_std = threshold_std
        self.detect_ch = detect_ch

        self.sample_rate = FS
        self.window_seconds = 5.0
        self.max_samples = int(self.sample_rate * self.window_seconds)

        # Network state
        self.spi_ports: List[SPIPortConfig] = []
        self.port_configs_raw = []
        self.total_channels = 0
        self.is_connected = False

        # Data buffers
        self.sample_queue = Queue(maxsize=5000)
        self.buf_size = self.max_samples * 2
        self.time_buf = np.zeros((8, self.buf_size))
        self.data_buf = np.zeros((8, self.buf_size))
        self.buf_idx = 0
        self.buf_count = 0

        # --- 60 Hz notch filter ---
        self.notch_on = True
        self.notch_b, self.notch_a = sig.iirnotch(60.0, Q=30.0, fs=self.sample_rate)
        self.notch_zi_template = sig.lfilter_zi(self.notch_b, self.notch_a)
        self.notch_states = [self.notch_zi_template.copy() for _ in range(8)]

        # --- 1 Hz HPF (10th order Butterworth, SOS) ---
        self.hpf_on = True
        self.hpf_sos = sig.butter(10, 1.0, btype='high', fs=self.sample_rate, output='sos')
        self.hpf_zi_template = sig.sosfilt_zi(self.hpf_sos)
        self.hpf_states = [self.hpf_zi_template.copy() for _ in range(8)]

        # --- 50 Hz LPF (10th order Butterworth, SOS) ---
        self.lpf_on = True
        self.lpf_sos = sig.butter(10, 50.0, btype='low', fs=self.sample_rate, output='sos')
        self.lpf_zi_template = sig.sosfilt_zi(self.lpf_sos)
        self.lpf_states = [self.lpf_zi_template.copy() for _ in range(8)]

        # Y-axis control
        self.y_autoscale = False
        self.y_fixed_range = 5000

        # Recording state
        self.recording = False
        self.csv_recorder = None
        self.record_start_time = None
        self.record_sample_count = 0
        self._csv_filepath = None

        # Channel mapping (set when config arrives)
        self._ch_offset = 0
        self._view_channels = 8
        self._channel_names = []

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
        self.setWindowTitle(f'Blink Test: {self.port_filter} Dev{self.device_filter}')
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
            f"Blink Test: {self.port_filter} Dev{self.device_filter}")
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

        # Filter toggle buttons
        self.notch_btn = QtWidgets.QPushButton('60Hz: ON')
        self.notch_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;')
        self.notch_btn.clicked.connect(self._toggle_notch)
        ctrl_bar.addWidget(self.notch_btn)

        self.hpf_btn = QtWidgets.QPushButton('1Hz HPF: ON')
        self.hpf_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;')
        self.hpf_btn.clicked.connect(self._toggle_hpf)
        ctrl_bar.addWidget(self.hpf_btn)

        self.lpf_btn = QtWidgets.QPushButton('50Hz LPF: ON')
        self.lpf_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;')
        self.lpf_btn.clicked.connect(self._toggle_lpf)
        ctrl_bar.addWidget(self.lpf_btn)

        # Y-scale toggle
        self.yscale_btn = QtWidgets.QPushButton(f'Y: \u00b1{int(self.y_fixed_range)}')
        self.yscale_btn.setStyleSheet(
            'background-color: #00aa55; color: white; padding: 6px 14px;')
        self.yscale_btn.clicked.connect(self._toggle_yscale)
        ctrl_bar.addWidget(self.yscale_btn)

        self.yrange_input = QtWidgets.QLineEdit(str(int(self.y_fixed_range)))
        self.yrange_input.setFixedWidth(60)
        self.yrange_input.setStyleSheet(
            'background-color: #2a2a2a; color: #e0e0e0; border: 1px solid #444; '
            'border-radius: 4px; padding: 4px; font-size: 10pt;')
        self.yrange_input.returnPressed.connect(self._apply_yrange)
        self.yrange_input.editingFinished.connect(self._apply_yrange)
        ctrl_bar.addWidget(self.yrange_input)

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
            pw.setYRange(-self.y_fixed_range, self.y_fixed_range)
            pw.setClipToView(True)
            pw.setDownsampling(mode='peak')

            curve = pw.plot(pen=pg.mkPen(color=color, width=1))
            self._plot_widgets.append(pw)
            self._plot_curves.append(curve)

            row.addWidget(pw, stretch=1)
            plots_layout.addLayout(row, stretch=1)

        layout.addWidget(plots_widget, stretch=1)

        # --- Results panel (hidden until analysis completes) ---
        self._results_container = QtWidgets.QWidget()
        results_layout = QtWidgets.QVBoxLayout(self._results_container)
        results_layout.setContentsMargins(0, 4, 0, 0)
        results_layout.setSpacing(4)

        self.results_header = QtWidgets.QLabel("BLINK DETECTION RESULTS")
        self.results_header.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 12pt; font-weight: bold; padding: 4px;")
        results_layout.addWidget(self.results_header)

        self.summary_label = QtWidgets.QLabel("")
        self.summary_label.setStyleSheet(
            f"color: {TEXT_COLOR}; font-size: 11pt; padding: 4px;")
        self.summary_label.setWordWrap(True)
        results_layout.addWidget(self.summary_label)

        # Per-channel amplitude table
        self.amp_table = QtWidgets.QTableWidget()
        self.amp_table.setColumnCount(3)
        self.amp_table.setHorizontalHeaderLabels(
            ["Channel", "Amplitude (uV)", "Role"])
        self.amp_table.horizontalHeader().setStretchLastSection(True)
        self.amp_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.amp_table.setMaximumHeight(250)
        results_layout.addWidget(self.amp_table)

        # Epoch overlay plot
        self.epoch_label = QtWidgets.QLabel("EPOCH OVERLAY")
        self.epoch_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 12pt; font-weight: bold; padding: 4px;")
        results_layout.addWidget(self.epoch_label)

        self.epoch_plot = pg.PlotWidget()
        self.epoch_plot.setBackground(PLOT_BG)
        self.epoch_plot.showGrid(x=True, y=True, alpha=0.3)
        self.epoch_plot.setLabel('bottom', 'Time (ms)')
        self.epoch_plot.setLabel('left', 'Amplitude (uV)')
        self.epoch_plot.getAxis('left').setPen(pg.mkPen(color='#888'))
        self.epoch_plot.getAxis('left').setTextPen(pg.mkPen(color='#888'))
        self.epoch_plot.getAxis('bottom').setPen(pg.mkPen(color='#888'))
        self.epoch_plot.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
        self.epoch_plot.setMinimumHeight(220)
        results_layout.addWidget(self.epoch_plot, stretch=1)

        # Per-channel amplitude bar chart
        self.bar_label = QtWidgets.QLabel("PER-CHANNEL BLINK AMPLITUDE")
        self.bar_label.setStyleSheet(
            f"color: {ACCENT_COLOR}; font-size: 12pt; font-weight: bold; padding: 4px;")
        results_layout.addWidget(self.bar_label)

        self.bar_plot = pg.PlotWidget()
        self.bar_plot.setBackground(PLOT_BG)
        self.bar_plot.showGrid(x=False, y=True, alpha=0.3)
        self.bar_plot.setLabel('bottom', 'Channel')
        self.bar_plot.setLabel('left', 'Amplitude (uV)')
        self.bar_plot.getAxis('left').setPen(pg.mkPen(color='#888'))
        self.bar_plot.getAxis('left').setTextPen(pg.mkPen(color='#888'))
        self.bar_plot.getAxis('bottom').setPen(pg.mkPen(color='#888'))
        self.bar_plot.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
        self.bar_plot.setMinimumHeight(180)
        results_layout.addWidget(self.bar_plot, stretch=1)

        self._results_container.setVisible(False)
        layout.addWidget(self._results_container, stretch=1)

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

        os.makedirs(self.output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(self.output_dir, f"blink_{ts}.csv")

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

        # Run blink analysis
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
        """Run blink analysis on the recorded CSV."""
        if not self._csv_filepath or not os.path.exists(self._csv_filepath):
            print(f"[analysis] CSV file not found: {self._csv_filepath}")
            return

        try:
            results = run_blink_analysis(
                self._csv_filepath,
                self.port_filter,
                self.device_filter,
                self.output_dir,
                threshold_std=self.threshold_std,
                detect_ch=self.detect_ch,
            )
        except Exception as e:
            print(f"[analysis] Error: {e}")
            traceback.print_exc()
            return

        if results is None:
            return

        self._show_blink_results(results)

    def _show_blink_results(self, results):
        """Populate the results panel with blink analysis data."""
        n_blinks = results['n_blinks']
        detect_ch = results['detect_ch']

        if n_blinks == 0:
            self.summary_label.setText(
                f"No blinks detected (threshold: {results['threshold']:.1f} uV). "
                f"Try lowering --threshold or check electrode placement.")
            self._results_container.setVisible(True)
            return

        # Summary line
        blink_rate = results['blink_rate']
        mean_amp = results['mean_amplitude']
        ibi = results['ibi_stats']
        ibi_text = f"{ibi['mean']:.2f}s" if ibi['mean'] > 0 else "N/A"
        self.summary_label.setText(
            f"Summary: {n_blinks} blinks ({blink_rate:.1f}/min) | "
            f"Mean amplitude: {mean_amp:.1f} uV (SD: {results['amplitude_std']:.1f}) | "
            f"IBI: {ibi_text}")

        # Per-channel amplitude table
        ch_amplitudes = results['ch_amplitudes']
        self.amp_table.setRowCount(8)
        for ch_i in range(8):
            self.amp_table.setItem(ch_i, 0,
                QtWidgets.QTableWidgetItem(f"Ch{ch_i+1}"))
            self.amp_table.setItem(ch_i, 1,
                QtWidgets.QTableWidgetItem(f"{ch_amplitudes[ch_i]:.1f}"))
            role = "Detection channel" if ch_i == detect_ch else ""
            self.amp_table.setItem(ch_i, 2,
                QtWidgets.QTableWidgetItem(role))

        # Epoch overlay plot
        self.epoch_plot.clear()
        epochs = results['epochs']
        grand_avg = results['grand_avg']
        epoch_time_ms = results['epoch_time_ms']

        # Plot individual epochs (thin, semi-transparent) for detection channel
        for blink_i in range(min(epochs.shape[0], 100)):  # Cap at 100 traces
            self.epoch_plot.plot(
                epoch_time_ms,
                epochs[blink_i, :, detect_ch],
                pen=pg.mkPen(color=(100, 100, 100, 60), width=1))

        # Plot grand average (bold)
        self.epoch_plot.plot(
            epoch_time_ms,
            grand_avg[:, detect_ch],
            pen=pg.mkPen(color=ACCENT_COLOR, width=3),
            name=f"Grand Avg (Ch{detect_ch+1}, n={n_blinks})")

        # Zero line
        self.epoch_plot.addItem(pg.InfiniteLine(
            pos=0, angle=90,
            pen=pg.mkPen(color='#ff4444', width=1, style=QtCore.Qt.DashLine)))
        self.epoch_plot.addItem(pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen(color='#555', width=1, style=QtCore.Qt.DashLine)))

        # Add legend
        legend = self.epoch_plot.addLegend(offset=(60, 10), labelTextColor='#ccc',
                                            labelTextSize='9pt')

        # Per-channel amplitude bar chart
        self.bar_plot.clear()
        x = np.arange(8)
        colors = [NEON_COLORS[i % len(NEON_COLORS)] for i in range(8)]
        brushes = [pg.mkBrush(c) for c in colors]

        bar = pg.BarGraphItem(
            x=x, height=ch_amplitudes, width=0.6,
            brushes=brushes,
            pens=[pg.mkPen(color='#fff', width=1) for _ in range(8)])
        self.bar_plot.addItem(bar)

        # X-axis labels
        ax = self.bar_plot.getAxis('bottom')
        ax.setTicks([[(i, f"Ch{i+1}") for i in range(8)]])

        # Highlight detection channel
        detect_line = pg.InfiniteLine(
            pos=detect_ch, angle=90,
            pen=pg.mkPen(color=ACCENT_COLOR, width=2, style=QtCore.Qt.DashLine))
        self.bar_plot.addItem(detect_line)

        self._results_container.setVisible(True)

    # ------------------------------------------------------------------
    # LIVE PLOT UPDATE
    # ------------------------------------------------------------------

    def _update(self):
        """20ms timer: drain sample queue, apply filters, update plots."""
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

            # Apply filter chain: notch -> HPF -> LPF
            if self.notch_on:
                for c in range(8):
                    raw[:, c], self.notch_states[c] = sig.lfilter(
                        self.notch_b, self.notch_a, raw[:, c],
                        zi=self.notch_states[c])

            if self.hpf_on:
                for c in range(8):
                    raw[:, c], self.hpf_states[c] = sig.sosfilt(
                        self.hpf_sos, raw[:, c],
                        zi=self.hpf_states[c])

            if self.lpf_on:
                for c in range(8):
                    raw[:, c], self.lpf_states[c] = sig.sosfilt(
                        self.lpf_sos, raw[:, c],
                        zi=self.lpf_states[c])

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
                    if self.y_autoscale:
                        y_min, y_max = float(d.min()), float(d.max())
                        margin = max((y_max - y_min) * 0.1, 100)
                        self._plot_widgets[i].setYRange(
                            y_min - margin, y_max + margin, padding=0)
                    else:
                        self._plot_widgets[i].setYRange(
                            -self.y_fixed_range, self.y_fixed_range, padding=0)

        except Exception as e:
            print(f"[update] Error: {e}")
            traceback.print_exc()

    # ------------------------------------------------------------------
    # FILTER TOGGLES
    # ------------------------------------------------------------------

    def _toggle_notch(self):
        self.notch_on = not self.notch_on
        if self.notch_on:
            self.notch_btn.setText('60Hz: ON')
            self.notch_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;')
            self.notch_states = [self.notch_zi_template.copy() for _ in range(8)]
        else:
            self.notch_btn.setText('60Hz: OFF')
            self.notch_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;')

    def _toggle_hpf(self):
        self.hpf_on = not self.hpf_on
        if self.hpf_on:
            self.hpf_btn.setText('1Hz HPF: ON')
            self.hpf_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;')
            self.hpf_states = [self.hpf_zi_template.copy() for _ in range(8)]
        else:
            self.hpf_btn.setText('1Hz HPF: OFF')
            self.hpf_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;')

    def _toggle_lpf(self):
        self.lpf_on = not self.lpf_on
        if self.lpf_on:
            self.lpf_btn.setText('50Hz LPF: ON')
            self.lpf_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;')
            self.lpf_states = [self.lpf_zi_template.copy() for _ in range(8)]
        else:
            self.lpf_btn.setText('50Hz LPF: OFF')
            self.lpf_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;')

    def _toggle_yscale(self):
        self.y_autoscale = not self.y_autoscale
        if self.y_autoscale:
            self.yscale_btn.setText('Y: Auto')
            self.yscale_btn.setStyleSheet(
                'background-color: #2a2a2a; color: #aaa; padding: 6px 14px;')
        else:
            self.yscale_btn.setText(f'Y: \u00b1{int(self.y_fixed_range)}')
            self.yscale_btn.setStyleSheet(
                'background-color: #00aa55; color: white; padding: 6px 14px;')
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
        # Don't intercept shortcuts while typing in the Y-range input
        if self.yrange_input.hasFocus():
            super().keyPressEvent(event)
            return
        if event.key() == QtCore.Qt.Key_R:
            self._toggle_recording()
        elif event.key() == QtCore.Qt.Key_N:
            self._toggle_notch()
        elif event.key() == QtCore.Qt.Key_H:
            self._toggle_hpf()
        elif event.key() == QtCore.Qt.Key_L:
            self._toggle_lpf()
        elif event.key() == QtCore.Qt.Key_Y:
            self._toggle_yscale()
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
        description='ADS1299 Eye Blink Test Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  uv run blink_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1
  uv run blink_test.py --host 192.168.1.99 --port-filter Port1 --device-filter 1 \\
                       --duration 120 --threshold 4.0 --detect-ch 2

Keyboard shortcuts:
  R  - Start/stop recording
  N  - Toggle 60 Hz notch filter
  H  - Toggle 1 Hz high-pass filter
  L  - Toggle 50 Hz low-pass filter
  Y  - Toggle Y-axis scale (fixed / auto)
        """)
    parser.add_argument('--host', type=str, required=True,
                        help='Server IP address')
    parser.add_argument('--port', type=int, default=8888,
                        help='Server port (default: 8888)')
    parser.add_argument('--port-filter', type=str, required=True,
                        help='Port to view/analyze (e.g. Port1)')
    parser.add_argument('--device-filter', type=int, required=True,
                        help='Device number to view/analyze (1-based)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help=f'Output directory (default: testing/{OUTPUT_DIR})')
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION,
                        help=f'Recording duration in seconds (default: {DEFAULT_DURATION})')
    parser.add_argument('--threshold', type=float, default=BLINK_THRESHOLD_STD,
                        help=f'Blink detection threshold in std devs (default: {BLINK_THRESHOLD_STD})')
    parser.add_argument('--detect-ch', type=int, default=None,
                        help='Channel for blink detection (1-8, default: auto)')

    args = parser.parse_args()

    # Resolve output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, OUTPUT_DIR)
    os.makedirs(output_dir, exist_ok=True)

    print(f"\nADS1299 Eye Blink Test")
    print(f"{'='*50}")
    print(f"Server:    {args.host}:{args.port}")
    print(f"View:      {args.port_filter} Dev{args.device_filter}")
    print(f"Record:    {args.duration}s ({args.duration * FS} samples)")
    print(f"Output:    {output_dir}")
    print(f"Threshold: {args.threshold} std")
    detect_str = f"Ch{args.detect_ch}" if args.detect_ch else "auto"
    print(f"Detect ch: {detect_str}")
    print(f"ADC:       LSB = {LSB_UV:.5f} uV (VREF={VREF}V, gain={GAIN})")
    print(f"Filters:   60Hz notch ON, 1Hz HPF ON, 50Hz LPF ON")
    print(f"{'='*50}\n")

    app = QtWidgets.QApplication(sys.argv)

    window = BlinkTestWindow(
        host=args.host,
        port=args.port,
        port_filter=args.port_filter,
        device_filter=args.device_filter,
        output_dir=output_dir,
        duration=args.duration,
        server_port=args.port,
        threshold_std=args.threshold,
        detect_ch=args.detect_ch,
    )
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
