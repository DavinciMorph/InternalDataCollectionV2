#!/usr/bin/env python3
"""
Comprehensive EEG analysis for 336-channel ADS1299 acquisition system.
Generates channel quality assessment, PSD analysis, artifact removal, and summary reports.
"""

import os
import sys
import time
import warnings
import numpy as np
import pandas as pd
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import ListedColormap
from scipy import signal as sig
from scipy.signal import iirnotch, filtfilt, welch
from sklearn.decomposition import FastICA

warnings.filterwarnings('ignore', category=RuntimeWarning)

# ==============================================================================
# Configuration
# ==============================================================================
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-03-09_191431.csv"
OUTPUT_DIR = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_analysis_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

FS = 250.0          # Sampling rate Hz
GAIN = 24
LSB_UV = 4.5 / (2**23 * GAIN) * 1e6  # ~0.02235 uV/count
ADC_RAIL = 8388607   # 2^23 - 1
RAIL_THRESH = 0.50   # >50% at rail => "railed"
DEAD_THRESH_UV = 1.0 # std < 1 uV => "dead"
NOISE_THRESH_UV = 500.0  # std > 500 uV => "excessive noise"
BLINK_THRESH_UV = 150.0  # blink detection threshold
BLINK_WINDOW_MS = 100    # ms around blink to interpolate

PORT_DEVICE_COUNTS = {1: 8, 2: 7, 3: 5, 4: 5, 5: 5, 6: 5, 7: 7}

PLOT_DPI = 150

# ==============================================================================
# Helper functions
# ==============================================================================

def parse_channel_name(name):
    """Parse 'Port{N}_dev{M}_ch{K}' into (port, dev, ch) ints."""
    parts = name.split('_')
    port = int(parts[0].replace('Port', ''))
    dev = int(parts[1].replace('dev', ''))
    ch = int(parts[2].replace('ch', ''))
    return port, dev, ch


def apply_notch_filter(data, fs, freqs, Q=30):
    """Apply IIR notch filters at specified frequencies."""
    filtered = data.copy()
    for f0 in freqs:
        if f0 >= fs / 2:
            continue
        b, a = iirnotch(f0, Q, fs)
        # Apply along axis=0 (time)
        for i in range(filtered.shape[1]):
            filtered[:, i] = filtfilt(b, a, filtered[:, i])
    return filtered


def compute_psd(data, fs, nperseg=256):
    """Compute Welch PSD. Returns (freqs, psd_matrix) with psd in uV^2/Hz."""
    noverlap = nperseg // 2
    freqs, psd = welch(data, fs=fs, nperseg=nperseg, noverlap=noverlap, axis=0)
    return freqs, psd


def interpolate_artifacts(data_1d, artifact_mask):
    """Linear interpolation over artifact regions."""
    out = data_1d.copy()
    indices = np.arange(len(out))
    good = ~artifact_mask
    if np.sum(good) < 2:
        return out  # can't interpolate
    out[artifact_mask] = np.interp(indices[artifact_mask], indices[good], out[good])
    return out


# ==============================================================================
# MAIN ANALYSIS
# ==============================================================================
def main():
    t0 = time.time()
    print("=" * 80)
    print("COMPREHENSIVE EEG ANALYSIS")
    print("=" * 80)

    # ------------------------------------------------------------------
    # 1. Load data
    # ------------------------------------------------------------------
    print("\n[1] Loading data...")
    df = pd.read_csv(CSV_PATH)
    timestamps = df['timestamp'].values
    sample_nums = df['sample_number'].values
    ch_cols = [c for c in df.columns if c.startswith('Port')]
    n_samples = len(df)
    n_channels = len(ch_cols)
    duration_s = n_samples / FS
    print(f"    Samples: {n_samples:,}  Channels: {n_channels}  Duration: {duration_s:.1f}s ({duration_s/60:.2f} min)")
    print(f"    Timestamp range: {timestamps[0]:.3f} -- {timestamps[-1]:.3f}")
    print(f"    Sample number range: {sample_nums[0]} -- {sample_nums[-1]}")

    # Convert to numpy and then to uV
    raw_counts = df[ch_cols].values.astype(np.float64)  # (n_samples, n_channels)
    data_uv = raw_counts * LSB_UV
    print(f"    LSB = {LSB_UV:.6f} uV/count")
    print(f"    Conversion done. Shape: {data_uv.shape}")

    # Parse channel metadata
    ch_info = []
    for col in ch_cols:
        port, dev, ch = parse_channel_name(col)
        ch_info.append({'name': col, 'port': port, 'dev': dev, 'ch': ch})
    ch_df = pd.DataFrame(ch_info)

    # ------------------------------------------------------------------
    # 2. Basic statistics & channel classification
    # ------------------------------------------------------------------
    print("\n[2] Computing per-channel statistics...")
    ch_mean = np.mean(data_uv, axis=0)
    ch_std = np.std(data_uv, axis=0)
    ch_min = np.min(data_uv, axis=0)
    ch_max = np.max(data_uv, axis=0)
    ch_ptp = ch_max - ch_min

    # Rail detection on raw counts
    rail_pos = np.abs(raw_counts - ADC_RAIL) < 100  # within 100 counts of +rail
    rail_neg = np.abs(raw_counts + ADC_RAIL) < 100  # within 100 counts of -rail
    rail_frac = np.mean(rail_pos | rail_neg, axis=0)

    # Classify channels
    status = []
    for i in range(n_channels):
        if rail_frac[i] > RAIL_THRESH:
            status.append('Railed')
        elif ch_std[i] < DEAD_THRESH_UV:
            status.append('Dead')
        elif ch_std[i] > NOISE_THRESH_UV:
            status.append('Noisy')
        else:
            status.append('Usable')

    ch_df['mean_uv'] = ch_mean
    ch_df['std_uv'] = ch_std
    ch_df['min_uv'] = ch_min
    ch_df['max_uv'] = ch_max
    ch_df['ptp_uv'] = ch_ptp
    ch_df['rail_frac'] = rail_frac
    ch_df['status'] = status

    # Summary
    status_counts = ch_df['status'].value_counts()
    print(f"\n    Channel Classification:")
    for s in ['Usable', 'Railed', 'Dead', 'Noisy']:
        cnt = status_counts.get(s, 0)
        print(f"      {s:10s}: {cnt:3d} / {n_channels} ({100*cnt/n_channels:.1f}%)")

    usable_mask = np.array([s == 'Usable' for s in status])
    usable_idx = np.where(usable_mask)[0]
    n_usable = len(usable_idx)
    print(f"\n    Usable channels: {n_usable}")

    # Per-port breakdown
    print(f"\n    Per-Port Breakdown:")
    print(f"    {'Port':>6s} {'Total':>6s} {'Usable':>7s} {'Railed':>7s} {'Dead':>6s} {'Noisy':>6s} {'%Usable':>8s}")
    print(f"    {'-'*50}")
    for port in sorted(ch_df['port'].unique()):
        mask = ch_df['port'] == port
        total = mask.sum()
        usable_p = ((ch_df['status'] == 'Usable') & mask).sum()
        railed_p = ((ch_df['status'] == 'Railed') & mask).sum()
        dead_p = ((ch_df['status'] == 'Dead') & mask).sum()
        noisy_p = ((ch_df['status'] == 'Noisy') & mask).sum()
        pct = 100 * usable_p / total if total > 0 else 0
        print(f"    Port{port:>2d} {total:>5d}  {usable_p:>6d}  {railed_p:>6d} {dead_p:>5d} {noisy_p:>5d}  {pct:>7.1f}%")

    # Usable channel stats summary
    if n_usable > 0:
        print(f"\n    Usable channel statistics:")
        print(f"      Mean of means: {np.mean(ch_mean[usable_mask]):.1f} uV")
        print(f"      Mean std:      {np.mean(ch_std[usable_mask]):.1f} uV")
        print(f"      Median std:    {np.median(ch_std[usable_mask]):.1f} uV")
        print(f"      Min std:       {np.min(ch_std[usable_mask]):.1f} uV")
        print(f"      Max std:       {np.max(ch_std[usable_mask]):.1f} uV")
        print(f"      Mean PTP:      {np.mean(ch_ptp[usable_mask]):.1f} uV")

    # ------------------------------------------------------------------
    # 3. Notch filtering (60 Hz + 120 Hz)
    # ------------------------------------------------------------------
    print("\n[3] Applying 60 Hz + 120 Hz notch filter...")
    notch_freqs = [60.0, 120.0]
    data_notch = apply_notch_filter(data_uv, FS, notch_freqs, Q=30)
    print("    Notch filter applied.")

    # ------------------------------------------------------------------
    # 4. PSD Analysis
    # ------------------------------------------------------------------
    print("\n[4] Computing PSD (Welch, nperseg=256)...")
    nperseg = 256
    freqs_raw, psd_raw = compute_psd(data_uv, FS, nperseg=nperseg)
    freqs_notch, psd_notch = compute_psd(data_notch, FS, nperseg=nperseg)
    print(f"    Frequency resolution: {freqs_raw[1]-freqs_raw[0]:.3f} Hz")
    print(f"    Frequency range: {freqs_raw[0]:.1f} -- {freqs_raw[-1]:.1f} Hz")

    # --- Plot: PSD Raw (average of usable channels) ---
    if n_usable > 0:
        avg_psd_raw = np.mean(psd_raw[:, usable_idx], axis=1)
        avg_psd_notch = np.mean(psd_notch[:, usable_idx], axis=1)

        # Find notable peaks in raw PSD
        # Peaks above 2x median in 5-125 Hz range
        f_mask = (freqs_raw >= 5) & (freqs_raw <= 125)
        median_psd = np.median(avg_psd_raw[f_mask])
        peak_indices = np.where((avg_psd_raw > 2 * median_psd) & f_mask)[0]

        print(f"\n    Notable PSD peaks (>2x median) in raw data:")
        # Group nearby peaks
        if len(peak_indices) > 0:
            groups = []
            current = [peak_indices[0]]
            for pi in peak_indices[1:]:
                if pi - current[-1] <= 2:
                    current.append(pi)
                else:
                    groups.append(current)
                    current = [pi]
            groups.append(current)
            for g in groups:
                peak_i = g[np.argmax(avg_psd_raw[g])]
                print(f"      {freqs_raw[peak_i]:6.1f} Hz : {avg_psd_raw[peak_i]:.2f} uV^2/Hz  ({avg_psd_raw[peak_i]/median_psd:.1f}x median)")

        # --- Plot psd_raw.png ---
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.semilogy(freqs_raw, avg_psd_raw, 'b-', linewidth=0.8, label='Average PSD (usable channels)')
        ax.axhline(median_psd, color='gray', linestyle='--', alpha=0.5, label=f'Median = {median_psd:.2f}')
        for f_mark in [10, 25, 50, 60, 70, 120]:
            if f_mark < FS/2:
                idx = np.argmin(np.abs(freqs_raw - f_mark))
                ax.axvline(f_mark, color='red', alpha=0.3, linestyle=':')
                ax.annotate(f'{f_mark} Hz', xy=(f_mark, avg_psd_raw[idx]),
                           fontsize=7, color='red', ha='center', va='bottom')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('PSD (uV^2/Hz)')
        ax.set_title(f'Average PSD -- Raw Data ({n_usable} usable channels)')
        ax.set_xlim(0, FS/2)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'psd_raw.png'), dpi=PLOT_DPI)
        plt.close(fig)
        print("    Saved psd_raw.png")

        # --- Plot psd_notch_filtered.png ---
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.semilogy(freqs_raw, avg_psd_raw, 'b-', linewidth=0.5, alpha=0.4, label='Raw')
        ax.semilogy(freqs_notch, avg_psd_notch, 'r-', linewidth=0.8, label='After 60+120 Hz notch')
        for f_mark in [60, 120]:
            ax.axvline(f_mark, color='green', alpha=0.5, linestyle='--')
            ax.annotate(f'{f_mark} Hz notch', xy=(f_mark, ax.get_ylim()[1]*0.5),
                       fontsize=8, color='green', rotation=90, va='top')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('PSD (uV^2/Hz)')
        ax.set_title(f'Average PSD -- After 60+120 Hz Notch ({n_usable} usable channels)')
        ax.set_xlim(0, FS/2)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'psd_notch_filtered.png'), dpi=PLOT_DPI)
        plt.close(fig)
        print("    Saved psd_notch_filtered.png")

        # --- Plot psd_per_port_raw.png ---
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        port_colors = plt.cm.Set1(np.linspace(0, 1, 7))
        for port_num in range(1, 8):
            port_mask_ch = ch_df['port'].values == port_num
            port_usable = usable_mask & port_mask_ch
            if np.sum(port_usable) > 0:
                port_avg_raw = np.mean(psd_raw[:, port_usable], axis=1)
                axes[0].semilogy(freqs_raw, port_avg_raw, linewidth=0.8,
                                color=port_colors[port_num-1],
                                label=f'Port{port_num} ({np.sum(port_usable)} ch)')
        axes[0].set_xlabel('Frequency (Hz)')
        axes[0].set_ylabel('PSD (uV^2/Hz)')
        axes[0].set_title('Per-Port Average PSD -- Raw')
        axes[0].set_xlim(0, FS/2)
        axes[0].legend(fontsize=7, loc='upper right')
        axes[0].grid(True, alpha=0.3)

        for port_num in range(1, 8):
            port_mask_ch = ch_df['port'].values == port_num
            port_usable = usable_mask & port_mask_ch
            if np.sum(port_usable) > 0:
                port_avg_notch = np.mean(psd_notch[:, port_usable], axis=1)
                axes[1].semilogy(freqs_notch, port_avg_notch, linewidth=0.8,
                                color=port_colors[port_num-1],
                                label=f'Port{port_num} ({np.sum(port_usable)} ch)')
        axes[1].set_xlabel('Frequency (Hz)')
        axes[1].set_ylabel('PSD (uV^2/Hz)')
        axes[1].set_title('Per-Port Average PSD -- After 60+120 Hz Notch')
        axes[1].set_xlim(0, FS/2)
        axes[1].legend(fontsize=7, loc='upper right')
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'psd_per_port_raw.png'), dpi=PLOT_DPI)
        plt.close(fig)
        print("    Saved psd_per_port_raw.png")

        # --- Also save per-port filtered separately ---
        fig, ax = plt.subplots(figsize=(12, 6))
        for port_num in range(1, 8):
            port_mask_ch = ch_df['port'].values == port_num
            port_usable = usable_mask & port_mask_ch
            if np.sum(port_usable) > 0:
                port_avg = np.mean(psd_notch[:, port_usable], axis=1)
                ax.semilogy(freqs_notch, port_avg, linewidth=0.8,
                           color=port_colors[port_num-1],
                           label=f'Port{port_num} ({np.sum(port_usable)} ch)')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('PSD (uV^2/Hz)')
        ax.set_title('Per-Port Average PSD -- Notch Filtered (60+120 Hz)')
        ax.set_xlim(0, FS/2)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'psd_per_port_filtered.png'), dpi=PLOT_DPI)
        plt.close(fig)
        print("    Saved psd_per_port_filtered.png")

    # ------------------------------------------------------------------
    # 5. Blink / artifact removal
    # ------------------------------------------------------------------
    print("\n[5] Blink / artifact removal...")
    data_clean = data_notch.copy()
    blink_window_samples = int(BLINK_WINDOW_MS / 1000.0 * FS)

    if n_usable >= 10:
        # --- Approach A: ICA-based removal on usable channels ---
        print(f"    Using ICA approach ({n_usable} usable channels available)")
        # Subsample if too many channels for ICA speed -- use up to 64
        max_ica_ch = min(n_usable, 64)
        ica_idx = usable_idx[:max_ica_ch]
        ica_data = data_notch[:, ica_idx].copy()

        # High-pass filter at 1 Hz for ICA stability
        sos_hp = sig.butter(2, 1.0, btype='high', fs=FS, output='sos')
        ica_data_hp = sig.sosfiltfilt(sos_hp, ica_data, axis=0)

        # Fit ICA
        n_components = min(max_ica_ch, 20)
        print(f"    Fitting ICA with {n_components} components on {max_ica_ch} channels...")
        try:
            ica = FastICA(n_components=n_components, max_iter=500, random_state=42, tol=0.01)
            sources = ica.fit_transform(ica_data_hp)
            mixing = ica.mixing_  # (n_channels, n_components)

            # Identify blink-like components:
            # - High kurtosis (blinks are sparse, high amplitude)
            # - Large amplitude variance
            from scipy.stats import kurtosis as sp_kurtosis
            kurt = sp_kurtosis(sources, axis=0)
            # Also check max amplitude
            src_max = np.max(np.abs(sources), axis=0)
            src_std = np.std(sources, axis=0)

            # Score: high kurtosis + high amplitude = likely artifact
            scores = np.abs(kurt) * src_std
            # Flag top 2 components by score if kurtosis > 3 (excess kurtosis)
            artifact_comps = []
            sorted_comps = np.argsort(scores)[::-1]
            for ci in sorted_comps[:3]:
                if np.abs(kurt[ci]) > 3.0:
                    artifact_comps.append(ci)
                    print(f"      Component {ci}: kurtosis={kurt[ci]:.1f}, std={src_std[ci]:.1f}, score={scores[ci]:.1f} --> ARTIFACT")
                else:
                    print(f"      Component {ci}: kurtosis={kurt[ci]:.1f}, std={src_std[ci]:.1f}, score={scores[ci]:.1f} --> kept")

            if len(artifact_comps) > 0:
                # Remove artifact components
                print(f"    Removing {len(artifact_comps)} artifact component(s)...")
                # Zero out artifact components and reconstruct
                sources_clean = sources.copy()
                sources_clean[:, artifact_comps] = 0.0
                reconstructed = sources_clean @ mixing.T  # back to channel space (HP filtered)

                # Instead of using HP-filtered data, compute the artifact signal and subtract from notch data
                artifact_only = (sources[:, artifact_comps] @ mixing[:, artifact_comps].T)
                # Need to account for centering done by ICA
                data_clean[:, ica_idx] = ica_data - artifact_only  # subtract from non-HP data
                # Then re-apply the notch (artifact subtraction may re-introduce some, but minimal)
                print("    ICA artifact removal done.")
            else:
                print("    No high-kurtosis components found -- no ICA removal applied.")
                # Fall through to threshold approach
                n_usable = 0  # trick to use threshold below

        except Exception as e:
            print(f"    ICA failed: {e}")
            print("    Falling back to threshold-based approach.")
            n_usable = 0  # trick to use threshold below

    if n_usable < 10:
        # --- Approach B: Threshold-based artifact interpolation ---
        print(f"    Using threshold-based approach (threshold = +/-{BLINK_THRESH_UV} uV)")
        n_usable = len(usable_idx)  # restore
        total_artifacts = 0
        for i in usable_idx:
            ch_data = data_notch[:, i]
            # Detect samples exceeding threshold
            exceed = np.abs(ch_data - np.mean(ch_data)) > BLINK_THRESH_UV
            if np.any(exceed):
                # Expand window
                artifact_mask = np.zeros(n_samples, dtype=bool)
                exceed_idx = np.where(exceed)[0]
                for ei in exceed_idx:
                    start = max(0, ei - blink_window_samples)
                    end = min(n_samples, ei + blink_window_samples + 1)
                    artifact_mask[start:end] = True
                total_artifacts += np.sum(artifact_mask)
                data_clean[:, i] = interpolate_artifacts(ch_data, artifact_mask)
        print(f"    Interpolated {total_artifacts:,} artifact samples across usable channels.")

    # Recompute usable mask (original classification)
    usable_mask_orig = np.array([s == 'Usable' for s in status])
    usable_idx_orig = np.where(usable_mask_orig)[0]
    n_usable_orig = len(usable_idx_orig)

    # PSD after artifact removal
    if n_usable_orig > 0:
        freqs_clean, psd_clean = compute_psd(data_clean, FS, nperseg=nperseg)
        avg_psd_clean = np.mean(psd_clean[:, usable_idx_orig], axis=1)

        # --- Plot psd_artifact_removed.png ---
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.semilogy(freqs_raw, avg_psd_raw, 'b-', linewidth=0.5, alpha=0.3, label='Raw')
        ax.semilogy(freqs_notch, avg_psd_notch, 'orange', linewidth=0.5, alpha=0.5, label='Notch filtered')
        ax.semilogy(freqs_clean, avg_psd_clean, 'g-', linewidth=0.8, label='After artifact removal')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('PSD (uV^2/Hz)')
        ax.set_title(f'Average PSD -- After Artifact Removal ({n_usable_orig} usable channels)')
        ax.set_xlim(0, FS/2)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'psd_artifact_removed.png'), dpi=PLOT_DPI)
        plt.close(fig)
        print("    Saved psd_artifact_removed.png")

    # ------------------------------------------------------------------
    # 6. Channel summary heatmap
    # ------------------------------------------------------------------
    print("\n[6] Generating channel summary heatmap...")
    max_dev = max(PORT_DEVICE_COUNTS.values())
    max_ch = 8
    # Build grid: rows = ports (1-7), cols = device*8 + channel
    status_map = {'Usable': 0, 'Noisy': 1, 'Dead': 2, 'Railed': 3}
    cmap_colors = ['#2ecc71', '#f39c12', '#95a5a6', '#e74c3c']  # green, orange, gray, red
    cmap = ListedColormap(cmap_colors)

    grid = np.full((7, max_dev * max_ch), np.nan)
    for _, row in ch_df.iterrows():
        port = row['port']
        dev = row['dev']
        ch = row['ch']
        grid[port-1, (dev-1)*8 + (ch-1)] = status_map[row['status']]

    fig, ax = plt.subplots(figsize=(18, 5))
    im = ax.imshow(grid, aspect='auto', cmap=cmap, vmin=-0.5, vmax=3.5, interpolation='nearest')
    ax.set_yticks(range(7))
    ax.set_yticklabels([f'Port {i+1} ({PORT_DEVICE_COUNTS[i+1]} dev)' for i in range(7)])
    # X-axis: device boundaries
    for d in range(max_dev + 1):
        ax.axvline(d * 8 - 0.5, color='white', linewidth=1)
    xtick_pos = [d * 8 + 3.5 for d in range(max_dev)]
    xtick_labels = [f'dev{d+1}' for d in range(max_dev)]
    ax.set_xticks(xtick_pos)
    ax.set_xticklabels(xtick_labels, fontsize=7)
    ax.set_title('Channel Status by Port and Device')

    # Custom legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=cmap_colors[i], label=k) for k, i in status_map.items()]
    # Add "N/A" for empty slots
    legend_elements.append(Patch(facecolor='white', edgecolor='gray', label='No device'))
    ax.legend(handles=legend_elements, loc='upper right', fontsize=8, ncol=5)

    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'channel_summary.png'), dpi=PLOT_DPI)
    plt.close(fig)
    print("    Saved channel_summary.png")

    # ------------------------------------------------------------------
    # 7. Time-domain examples
    # ------------------------------------------------------------------
    print("\n[7] Generating time-domain example plots...")
    time_axis = np.arange(n_samples) / FS

    # Pick up to 6 usable channels from different ports for variety
    example_channels = []
    for port_num in range(1, 8):
        port_usable_idx = usable_idx_orig[ch_df.iloc[usable_idx_orig]['port'].values == port_num]
        if len(port_usable_idx) > 0:
            example_channels.append(port_usable_idx[0])
        if len(example_channels) >= 6:
            break
    # If we have fewer, pad with more usable channels
    if len(example_channels) < 6 and n_usable_orig > 0:
        for idx in usable_idx_orig:
            if idx not in example_channels:
                example_channels.append(idx)
            if len(example_channels) >= 6:
                break

    n_example = len(example_channels)
    if n_example > 0:
        # Show first 10 seconds (2500 samples)
        show_samples = min(n_samples, int(10 * FS))
        t_show = time_axis[:show_samples]

        fig, axes = plt.subplots(n_example, 3, figsize=(18, 3 * n_example), sharex=True)
        if n_example == 1:
            axes = axes.reshape(1, -1)

        for row, chi in enumerate(example_channels):
            ch_name = ch_cols[chi]
            # Raw
            axes[row, 0].plot(t_show, data_uv[:show_samples, chi], 'b-', linewidth=0.3)
            axes[row, 0].set_ylabel(f'{ch_name}\n(uV)', fontsize=7)
            if row == 0:
                axes[row, 0].set_title('Raw', fontsize=10)
            # Notch filtered
            axes[row, 1].plot(t_show, data_notch[:show_samples, chi], 'r-', linewidth=0.3)
            if row == 0:
                axes[row, 1].set_title('60+120 Hz Notch', fontsize=10)
            # Artifact removed
            axes[row, 2].plot(t_show, data_clean[:show_samples, chi], 'g-', linewidth=0.3)
            if row == 0:
                axes[row, 2].set_title('After Artifact Removal', fontsize=10)

        for col in range(3):
            axes[-1, col].set_xlabel('Time (s)')

        fig.suptitle('Time-Domain Examples (first 10 seconds)', fontsize=12, y=1.01)
        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'time_domain_examples.png'), dpi=PLOT_DPI, bbox_inches='tight')
        plt.close(fig)
        print("    Saved time_domain_examples.png")

    # ------------------------------------------------------------------
    # 8. Additional diagnostic: STD distribution + per-device bar chart
    # ------------------------------------------------------------------
    print("\n[8] Generating additional diagnostic plots...")

    # STD histogram for usable channels
    if n_usable_orig > 0:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

        ax1.hist(ch_std[usable_mask_orig], bins=50, color='steelblue', edgecolor='black', alpha=0.7)
        ax1.axvline(np.median(ch_std[usable_mask_orig]), color='red', linestyle='--',
                    label=f'Median = {np.median(ch_std[usable_mask_orig]):.1f} uV')
        ax1.set_xlabel('Channel STD (uV)')
        ax1.set_ylabel('Count')
        ax1.set_title('STD Distribution (Usable Channels)')
        ax1.legend()

        # Per-port usable fraction bar chart
        ports = sorted(ch_df['port'].unique())
        usable_fracs = []
        railed_fracs = []
        dead_fracs = []
        noisy_fracs = []
        for p in ports:
            pm = ch_df['port'] == p
            total = pm.sum()
            usable_fracs.append(((ch_df['status'] == 'Usable') & pm).sum() / total * 100)
            railed_fracs.append(((ch_df['status'] == 'Railed') & pm).sum() / total * 100)
            dead_fracs.append(((ch_df['status'] == 'Dead') & pm).sum() / total * 100)
            noisy_fracs.append(((ch_df['status'] == 'Noisy') & pm).sum() / total * 100)

        x = np.arange(len(ports))
        width = 0.6
        ax2.bar(x, usable_fracs, width, color='#2ecc71', label='Usable')
        ax2.bar(x, noisy_fracs, width, bottom=usable_fracs, color='#f39c12', label='Noisy')
        bottom2 = [u + n for u, n in zip(usable_fracs, noisy_fracs)]
        ax2.bar(x, dead_fracs, width, bottom=bottom2, color='#95a5a6', label='Dead')
        bottom3 = [b + d for b, d in zip(bottom2, dead_fracs)]
        ax2.bar(x, railed_fracs, width, bottom=bottom3, color='#e74c3c', label='Railed')

        ax2.set_xticks(x)
        ax2.set_xticklabels([f'Port{p}' for p in ports])
        ax2.set_ylabel('Percentage (%)')
        ax2.set_title('Channel Status by Port')
        ax2.legend(fontsize=8)
        ax2.set_ylim(0, 105)

        fig.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'channel_diagnostics.png'), dpi=PLOT_DPI)
        plt.close(fig)
        print("    Saved channel_diagnostics.png")

    # ------------------------------------------------------------------
    # 9. Write text report
    # ------------------------------------------------------------------
    print("\n[9] Writing text report...")
    report_path = os.path.join(OUTPUT_DIR, 'channel_report.txt')
    with open(report_path, 'w') as f:
        f.write("=" * 90 + "\n")
        f.write("COMPREHENSIVE EEG ANALYSIS REPORT\n")
        f.write(f"File: {CSV_PATH}\n")
        f.write(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 90 + "\n\n")

        f.write("RECORDING SUMMARY\n")
        f.write("-" * 40 + "\n")
        f.write(f"  Samples:       {n_samples:,}\n")
        f.write(f"  Channels:      {n_channels}\n")
        f.write(f"  Duration:      {duration_s:.1f} s ({duration_s/60:.2f} min)\n")
        f.write(f"  Sampling rate: {FS} Hz\n")
        f.write(f"  Gain:          {GAIN}x\n")
        f.write(f"  LSB:           {LSB_UV:.6f} uV/count\n")
        f.write(f"  Timestamp:     {timestamps[0]:.3f} -- {timestamps[-1]:.3f}\n")
        f.write(f"  Sample range:  {sample_nums[0]} -- {sample_nums[-1]}\n\n")

        # Check for gaps
        dt = np.diff(timestamps)
        dt_mean = np.mean(dt)
        dt_std = np.std(dt)
        dt_max = np.max(dt)
        dt_min = np.min(dt)
        gap_thresh = 2.0 / FS  # >2 sample periods
        n_gaps = np.sum(dt > gap_thresh)
        f.write("TIMING INTEGRITY\n")
        f.write("-" * 40 + "\n")
        f.write(f"  dt mean:  {dt_mean*1000:.4f} ms (expected {1000/FS:.4f} ms)\n")
        f.write(f"  dt std:   {dt_std*1000:.4f} ms\n")
        f.write(f"  dt min:   {dt_min*1000:.4f} ms\n")
        f.write(f"  dt max:   {dt_max*1000:.4f} ms\n")
        f.write(f"  Gaps (>2x period): {n_gaps}\n")
        sn_diff = np.diff(sample_nums)
        n_missed = np.sum(sn_diff != 1)
        f.write(f"  Missed samples:    {n_missed}\n\n")

        f.write("CHANNEL CLASSIFICATION\n")
        f.write("-" * 40 + "\n")
        for s in ['Usable', 'Railed', 'Dead', 'Noisy']:
            cnt = status_counts.get(s, 0)
            f.write(f"  {s:10s}: {cnt:3d} / {n_channels} ({100*cnt/n_channels:.1f}%)\n")
        f.write("\n")

        f.write("PER-PORT BREAKDOWN\n")
        f.write("-" * 70 + "\n")
        f.write(f"  {'Port':>6s} {'Total':>6s} {'Usable':>7s} {'Railed':>7s} {'Dead':>6s} {'Noisy':>6s} {'%Usable':>8s}\n")
        f.write(f"  {'-'*60}\n")
        for port in sorted(ch_df['port'].unique()):
            mask = ch_df['port'] == port
            total = mask.sum()
            usable_p = ((ch_df['status'] == 'Usable') & mask).sum()
            railed_p = ((ch_df['status'] == 'Railed') & mask).sum()
            dead_p = ((ch_df['status'] == 'Dead') & mask).sum()
            noisy_p = ((ch_df['status'] == 'Noisy') & mask).sum()
            pct = 100 * usable_p / total if total > 0 else 0
            f.write(f"  Port{port:>2d} {total:>5d}  {usable_p:>6d}  {railed_p:>6d} {dead_p:>5d} {noisy_p:>5d}  {pct:>7.1f}%\n")
        f.write("\n")

        if n_usable_orig > 0:
            f.write("USABLE CHANNEL STATISTICS\n")
            f.write("-" * 40 + "\n")
            f.write(f"  Mean of means: {np.mean(ch_mean[usable_mask_orig]):.1f} uV\n")
            f.write(f"  Mean std:      {np.mean(ch_std[usable_mask_orig]):.1f} uV\n")
            f.write(f"  Median std:    {np.median(ch_std[usable_mask_orig]):.1f} uV\n")
            f.write(f"  Min std:       {np.min(ch_std[usable_mask_orig]):.1f} uV\n")
            f.write(f"  Max std:       {np.max(ch_std[usable_mask_orig]):.1f} uV\n")
            f.write(f"  Mean PTP:      {np.mean(ch_ptp[usable_mask_orig]):.1f} uV\n\n")

        # 60 Hz power comparison
        if n_usable_orig > 0:
            f.write("60 Hz POWER REDUCTION\n")
            f.write("-" * 40 + "\n")
            idx_60 = np.argmin(np.abs(freqs_raw - 60))
            raw_60 = avg_psd_raw[idx_60]
            notch_60 = avg_psd_notch[idx_60]
            f.write(f"  Raw   PSD at 60 Hz: {raw_60:.4f} uV^2/Hz\n")
            f.write(f"  Notch PSD at 60 Hz: {notch_60:.4f} uV^2/Hz\n")
            f.write(f"  Reduction: {raw_60/notch_60:.1f}x ({10*np.log10(raw_60/notch_60):.1f} dB)\n\n")

        # Full channel table
        f.write("FULL CHANNEL TABLE\n")
        f.write("-" * 110 + "\n")
        f.write(f"{'Channel':<22s} {'Port':>4s} {'Dev':>4s} {'Ch':>3s} {'Status':>8s} "
                f"{'Mean(uV)':>10s} {'STD(uV)':>10s} {'PTP(uV)':>10s} {'Rail%':>7s}\n")
        f.write("-" * 110 + "\n")
        for _, row in ch_df.iterrows():
            f.write(f"{row['name']:<22s} {row['port']:>4d} {row['dev']:>4d} {row['ch']:>3d} "
                    f"{row['status']:>8s} {row['mean_uv']:>10.1f} {row['std_uv']:>10.1f} "
                    f"{row['ptp_uv']:>10.1f} {row['rail_frac']*100:>6.1f}%\n")

        f.write("\n" + "=" * 90 + "\n")
        f.write("END OF REPORT\n")

    print(f"    Saved channel_report.txt")

    # ------------------------------------------------------------------
    # 10. Summary printout
    # ------------------------------------------------------------------
    elapsed = time.time() - t0
    print(f"\n{'='*80}")
    print(f"ANALYSIS COMPLETE in {elapsed:.1f} seconds")
    print(f"{'='*80}")
    print(f"\nOutput directory: {OUTPUT_DIR}")
    print(f"Files generated:")
    for fname in sorted(os.listdir(OUTPUT_DIR)):
        fpath = os.path.join(OUTPUT_DIR, fname)
        fsize = os.path.getsize(fpath)
        print(f"  {fname:<35s}  {fsize/1024:.0f} KB")

    # Print key findings summary
    print(f"\n{'='*80}")
    print("KEY FINDINGS SUMMARY")
    print(f"{'='*80}")
    print(f"  Recording: {n_samples:,} samples, {duration_s:.1f}s, {n_channels} channels")
    print(f"  Timing: dt_mean={dt_mean*1000:.4f}ms, gaps={n_gaps}, missed={n_missed}")
    print(f"  Usable: {n_usable_orig}/{n_channels} ({100*n_usable_orig/n_channels:.1f}%)")
    print(f"  Railed: {status_counts.get('Railed', 0)}/{n_channels}")
    print(f"  Dead:   {status_counts.get('Dead', 0)}/{n_channels}")
    print(f"  Noisy:  {status_counts.get('Noisy', 0)}/{n_channels}")
    if n_usable_orig > 0:
        print(f"  Usable STD: median={np.median(ch_std[usable_mask_orig]):.1f} uV, "
              f"range=[{np.min(ch_std[usable_mask_orig]):.1f}, {np.max(ch_std[usable_mask_orig]):.1f}]")
        idx_60 = np.argmin(np.abs(freqs_raw - 60))
        print(f"  60 Hz notch: {avg_psd_raw[idx_60]/avg_psd_notch[idx_60]:.1f}x reduction")

    return 0


if __name__ == '__main__':
    sys.exit(main())
