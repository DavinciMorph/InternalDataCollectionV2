#!/usr/bin/env uv run script
# /// script
# dependencies = [
#   "numpy",
#   "scipy",
#   "matplotlib",
# ]
# ///
"""
Noise Comparison: Device 1 vs Device 9
=======================================

Compares noise quality between two ADS1299 channel 8 recordings.
Produces a 6-panel figure (3x2) and prints a console metrics summary.

Usage:
    uv run compare_noise.py
"""

import numpy as np
from scipy import signal as sig
import matplotlib.pyplot as plt

# --- Constants ---
FS = 250              # Sample rate (Hz)
VREF = 4.5            # ADS1299 reference voltage
LSB_UV = VREF / (2**23 - 1) * 1e6  # ~0.536 uV per LSB

COLOR_D1 = '#2196F3'  # Blue
COLOR_D9 = '#E91E63'  # Pink/red

# --- Load & preprocess ---
def load_and_convert(filepath):
    """Load CSV, convert raw ADC codes to microvolts, remove DC offset."""
    data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
    raw_codes = data[:, 2]
    uv = raw_codes * LSB_UV
    uv -= np.mean(uv)  # Remove DC offset
    t = np.arange(len(uv)) / FS
    return t, uv

def notch_filter(x, freq=60.0, Q=30.0, fs=FS):
    """Apply 60 Hz notch filter using filtfilt."""
    b, a = sig.iirnotch(freq, Q, fs)
    return sig.filtfilt(b, a, x)

# Load both devices
t1, d1_raw = load_and_convert('device1_ch8_data.csv')
t9, d9_raw = load_and_convert('device9_ch8_data.csv')

# Apply 60 Hz notch
d1_filt = notch_filter(d1_raw)
d9_filt = notch_filter(d9_raw)

# --- Metrics ---
def compute_metrics(x):
    rms = np.sqrt(np.mean(x**2))
    std = np.std(x)
    pp = np.ptp(x)
    return rms, std, pp

m_d1_raw = compute_metrics(d1_raw)
m_d9_raw = compute_metrics(d9_raw)
m_d1_filt = compute_metrics(d1_filt)
m_d9_filt = compute_metrics(d9_filt)

print()
print("=" * 72)
print("  Noise Comparison: Device 1 vs Device 9  (units: uV)")
print("=" * 72)
header = f"{'Metric':<14} {'D1 Raw':>10} {'D9 Raw':>10} {'Ratio':>8}  |  {'D1 Notch':>10} {'D9 Notch':>10} {'Ratio':>8}"
print(header)
print("-" * 72)
labels = ['RMS', 'Std Dev', 'Peak-Peak']
for i, label in enumerate(labels):
    ratio_raw = m_d9_raw[i] / m_d1_raw[i] if m_d1_raw[i] != 0 else float('inf')
    ratio_filt = m_d9_filt[i] / m_d1_filt[i] if m_d1_filt[i] != 0 else float('inf')
    print(f"{label:<14} {m_d1_raw[i]:>10.2f} {m_d9_raw[i]:>10.2f} {ratio_raw:>7.2f}x  |  {m_d1_filt[i]:>10.2f} {m_d9_filt[i]:>10.2f} {ratio_filt:>7.2f}x")
print("-" * 72)
print("  Ratio = Device 9 / Device 1  (ratio > 1 means Device 1 is quieter)")
print()

# --- Figure ---
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('Noise Comparison: Device 1 vs Device 9', fontsize=14, fontweight='bold')

seg = 2 * FS  # 2-second segment (500 samples)

# Row 1: Time-domain overlay (2s segment)
for col, (label, d1, d9) in enumerate([
    ('Raw (DC-removed)', d1_raw, d9_raw),
    ('60 Hz Notch Filtered', d1_filt, d9_filt),
]):
    ax = axes[0, col]
    t_seg = np.arange(seg) / FS
    ax.plot(t_seg, d1[:seg], color=COLOR_D1, alpha=0.8, linewidth=0.7, label='Device 1')
    ax.plot(t_seg, d9[:seg], color=COLOR_D9, alpha=0.8, linewidth=0.7, label='Device 9')
    ax.set_title(f'Time Domain — {label}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Amplitude (uV)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

# Row 2: PSD via Welch
for col, (label, d1, d9) in enumerate([
    ('Raw', d1_raw, d9_raw),
    ('Notch Filtered', d1_filt, d9_filt),
]):
    ax = axes[1, col]
    f1, p1 = sig.welch(d1, fs=FS, nperseg=1024)
    f9, p9 = sig.welch(d9, fs=FS, nperseg=1024)
    ax.semilogy(f1, p1, color=COLOR_D1, alpha=0.8, linewidth=1, label='Device 1')
    ax.semilogy(f9, p9, color=COLOR_D9, alpha=0.8, linewidth=1, label='Device 9')
    for hz in [60, 120]:
        if hz < FS / 2:
            ax.axvline(hz, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)
            ax.text(hz + 1, ax.get_ylim()[0] * 2, f'{hz} Hz', fontsize=7, color='gray')
    ax.set_title(f'PSD (Welch) — {label}')
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Power (uV²/Hz)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

# Row 3: Histograms
for col, (label, d1, d9) in enumerate([
    ('Raw', d1_raw, d9_raw),
    ('Notch Filtered', d1_filt, d9_filt),
]):
    ax = axes[2, col]
    bins = 100
    ax.hist(d1, bins=bins, color=COLOR_D1, alpha=0.5, density=True, label='Device 1')
    ax.hist(d9, bins=bins, color=COLOR_D9, alpha=0.5, density=True, label='Device 9')
    ax.set_title(f'Amplitude Distribution — {label}')
    ax.set_xlabel('Amplitude (uV)')
    ax.set_ylabel('Density')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('noise_comparison.png', dpi=150, bbox_inches='tight')
print(f"Figure saved to noise_comparison.png")
plt.show()
