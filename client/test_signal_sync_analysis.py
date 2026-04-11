"""
ADS1299 Test Signal Synchronization Analysis
=============================================
Analyzes a 336-channel, 42-device, 7-port EEG system's internal 1 Hz square wave
test signal for cross-device and cross-port synchronization.
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict
import os, re, warnings
warnings.filterwarnings('ignore')

CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-03-30_124736.csv"
OUT_DIR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\test_signal_analysis"
os.makedirs(OUT_DIR, exist_ok=True)

FS = 250  # Hz

# Port config
PORT_DEVICES = {1:8, 2:7, 3:5, 4:5, 5:5, 6:5, 7:7}

# ─── Load data ────────────────────────────────────────────────────────────────
print("Loading CSV...")
df = pd.read_csv(CSV_PATH)
print(f"  Shape: {df.shape}")
print(f"  Columns: timestamp, sample_number, + {df.shape[1]-2} channels")
print(f"  Duration: {(df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]):.1f} s")
print(f"  Sample range: {df['sample_number'].iloc[0]} - {df['sample_number'].iloc[-1]}")

# Extract channel columns
ch_cols = [c for c in df.columns if c.startswith('Port')]
print(f"  Channel columns found: {len(ch_cols)}")

# Parse port/device/channel from column names
def parse_col(col):
    m = re.match(r'Port(\d+)_dev(\d+)_ch(\d+)', col)
    return int(m.group(1)), int(m.group(2)), int(m.group(3))

ch_info = {col: parse_col(col) for col in ch_cols}

# ─── 1. SQUARE WAVE DETECTION & AMPLITUDE ─────────────────────────────────────
print("\n" + "="*80)
print("1. SQUARE WAVE DETECTION & AMPLITUDE")
print("="*80)

data = df[ch_cols].values.astype(np.float64)  # (N, 336)
timestamps = df['timestamp'].values
sample_nums = df['sample_number'].values

# Compute per-channel stats
ch_means = np.mean(data, axis=0)
ch_stds  = np.std(data, axis=0)
ch_mins  = np.min(data, axis=0)
ch_maxs  = np.max(data, axis=0)
ch_pp    = ch_maxs - ch_mins  # peak-to-peak

print(f"\nPeak-to-peak amplitude across all {len(ch_cols)} channels:")
print(f"  Mean:   {np.mean(ch_pp):.1f} ADC codes")
print(f"  Median: {np.median(ch_pp):.1f} ADC codes")
print(f"  Min:    {np.min(ch_pp):.1f} (channel: {ch_cols[np.argmin(ch_pp)]})")
print(f"  Max:    {np.max(ch_pp):.1f} (channel: {ch_cols[np.argmax(ch_pp)]})")
print(f"  Std:    {np.std(ch_pp):.1f}")

# Per-port amplitude summary
print("\nPer-port peak-to-peak amplitude:")
for port in sorted(PORT_DEVICES.keys()):
    port_cols = [c for c in ch_cols if ch_info[c][0] == port]
    port_idx  = [ch_cols.index(c) for c in port_cols]
    port_pp   = ch_pp[port_idx]
    print(f"  Port {port} ({len(port_cols):3d} ch): mean={np.mean(port_pp):.1f}, "
          f"std={np.std(port_pp):.1f}, min={np.min(port_pp):.1f}, max={np.max(port_pp):.1f}")

# Per-device amplitude
print("\nPer-device peak-to-peak amplitude (mean of 8 channels):")
device_pp = {}
for port in sorted(PORT_DEVICES.keys()):
    for dev in range(1, PORT_DEVICES[port]+1):
        dev_cols = [c for c in ch_cols if ch_info[c][0]==port and ch_info[c][1]==dev]
        dev_idx  = [ch_cols.index(c) for c in dev_cols]
        dev_mean_pp = np.mean(ch_pp[dev_idx])
        device_pp[(port, dev)] = dev_mean_pp
        status = ""
        if dev_mean_pp < 100:
            status = " *** DC / DEAD"
        elif abs(dev_mean_pp - np.median(ch_pp)) > 3 * np.std(ch_pp):
            status = " *** OUTLIER"
        print(f"  Port{port}_dev{dev}: {dev_mean_pp:.1f}{status}")

# ─── 2. TRANSITION DETECTION ──────────────────────────────────────────────────
print("\n" + "="*80)
print("2. CROSS-DEVICE SYNCHRONIZATION — TRANSITION DETECTION")
print("="*80)

# Binarize each channel: above/below its own median
ch_medians = np.median(data, axis=0)
binary = (data > ch_medians[np.newaxis, :]).astype(np.int8)  # (N, 336)

# Detect edges on each channel: diff of binary signal
# +1 = rising edge, -1 = falling edge
edges = np.diff(binary, axis=0)  # (N-1, 336)

# For polarity detection: correlate binary signal of each channel with channel 0
# If negative correlation, the channel is inverted
corr_with_ref = np.array([np.corrcoef(binary[:, 0], binary[:, i])[0,1] for i in range(len(ch_cols))])
inverted = corr_with_ref < 0
n_inverted = np.sum(inverted)
print(f"\nPolarity check vs channel 0 ({ch_cols[0]}):")
print(f"  Normal polarity: {np.sum(~inverted)}")
print(f"  Inverted:        {n_inverted}")
if n_inverted > 0:
    inv_chans = [ch_cols[i] for i in range(len(ch_cols)) if inverted[i]]
    # Group by port
    inv_by_port = defaultdict(list)
    for c in inv_chans:
        inv_by_port[ch_info[c][0]].append(c)
    for port in sorted(inv_by_port.keys()):
        print(f"    Port {port}: {len(inv_by_port[port])} channels inverted")
        # Show device-level
        devs = set(ch_info[c][1] for c in inv_by_port[port])
        for d in sorted(devs):
            dc = [c for c in inv_by_port[port] if ch_info[c][1]==d]
            print(f"      dev{d}: {len(dc)} ch — {', '.join(dc)}")

# Also check for NaN correlation (constant channels)
nan_corr = np.isnan(corr_with_ref)
if np.any(nan_corr):
    print(f"\n  Channels with NaN correlation (constant/DC): {np.sum(nan_corr)}")
    for i in np.where(nan_corr)[0]:
        print(f"    {ch_cols[i]}: constant value = {data[0, i]:.0f}")

# Align polarity: flip inverted channels
binary_aligned = binary.copy()
binary_aligned[:, inverted] = 1 - binary_aligned[:, inverted]

edges_aligned = np.diff(binary_aligned, axis=0)

# Find rising edges on each channel
N_TRANSITIONS = 200  # collect many
rising_edges_per_ch = {}
for i, col in enumerate(ch_cols):
    rising = np.where(edges_aligned[:, i] == 1)[0]
    rising_edges_per_ch[col] = rising

# How many transitions detected?
n_edges_list = [len(v) for v in rising_edges_per_ch.values()]
print(f"\nRising edges detected per channel:")
print(f"  Min: {min(n_edges_list)}, Max: {max(n_edges_list)}, Median: {np.median(n_edges_list):.0f}")

# Channels with 0 edges (DC channels)
dc_channels = [ch_cols[i] for i in range(len(ch_cols)) if n_edges_list[i] == 0]
if dc_channels:
    print(f"\n  DC channels (no transitions): {len(dc_channels)}")
    for c in dc_channels:
        print(f"    {c}: value range [{ch_mins[ch_cols.index(c)]:.0f}, {ch_maxs[ch_cols.index(c)]:.0f}]")
else:
    print(f"\n  All channels have detected transitions (no DC channels)")

# ─── 3. TRANSITION SYNCHRONIZATION ANALYSIS ────────────────────────────────────
print("\n" + "="*80)
print("3. TRANSITION SYNCHRONIZATION ANALYSIS")
print("="*80)

# For each transition, find the sample index where it occurs on each channel
active_channels = [c for c in ch_cols if len(rising_edges_per_ch[c]) >= 5]
print(f"\nActive channels (>=5 transitions): {len(active_channels)} / {len(ch_cols)}")

# Determine common number of transitions
min_transitions = min(len(rising_edges_per_ch[c]) for c in active_channels)
print(f"Common transitions available: {min_transitions}")

n_use = min(min_transitions, 20)
print(f"Using first {n_use} transitions for sync analysis")

# Build transition matrix: (n_active_channels, n_use)
trans_matrix = np.zeros((len(active_channels), n_use), dtype=np.int64)
for i, col in enumerate(active_channels):
    trans_matrix[i, :] = rising_edges_per_ch[col][:n_use]

# For each transition, compute the spread
print(f"\nPer-transition synchronization (rising edges):")
print(f"  {'Trans#':>6s} {'Median':>8s} {'Min':>8s} {'Max':>8s} {'Spread':>8s} {'Spread_ms':>10s}")
spreads = []
for t in range(n_use):
    col_vals = trans_matrix[:, t]
    med = np.median(col_vals)
    mn  = np.min(col_vals)
    mx  = np.max(col_vals)
    spread = mx - mn
    spreads.append(spread)
    print(f"  {t+1:6d} {med:8.0f} {mn:8d} {mx:8d} {spread:8d} {spread*1000/FS:10.1f}")

spreads = np.array(spreads)
print(f"\n  Overall spread: mean={np.mean(spreads):.1f} samples ({np.mean(spreads)*1000/FS:.1f} ms), "
      f"max={np.max(spreads)} samples ({np.max(spreads)*1000/FS:.1f} ms)")

# Which channels are the outliers? For the worst transition, who's early/late?
worst_t = np.argmax(spreads)
worst_vals = trans_matrix[:, worst_t]
worst_med  = np.median(worst_vals)
print(f"\n  Worst transition (#{worst_t+1}): spread = {spreads[worst_t]} samples")
# Channels that are more than 1 sample from median
far_chans = [(active_channels[i], worst_vals[i] - worst_med) for i in range(len(active_channels))
             if abs(worst_vals[i] - worst_med) >= 1]
if far_chans:
    far_chans.sort(key=lambda x: x[1])
    print(f"  Channels offset from median (>=1 sample):")
    for ch, off in far_chans[:10]:
        print(f"    {ch}: {off:+.0f} samples")
    if len(far_chans) > 10:
        print(f"    ... and {len(far_chans)-10} more")

# ─── 4. WITHIN-PORT vs CROSS-PORT SYNC ────────────────────────────────────────
print("\n" + "="*80)
print("4. WITHIN-PORT vs CROSS-PORT SYNCHRONIZATION")
print("="*80)

# Per-port transition medians
port_trans_medians = {}
port_within_spreads = {}
for port in sorted(PORT_DEVICES.keys()):
    port_active = [c for c in active_channels if ch_info[c][0] == port]
    if not port_active:
        continue
    port_idx = [active_channels.index(c) for c in port_active]
    port_trans = trans_matrix[port_idx, :]

    # Within-port spread
    within_spreads = []
    for t in range(n_use):
        ws = np.max(port_trans[:, t]) - np.min(port_trans[:, t])
        within_spreads.append(ws)
    within_spreads = np.array(within_spreads)
    port_within_spreads[port] = within_spreads

    port_medians = np.median(port_trans, axis=0)
    port_trans_medians[port] = port_medians

    print(f"\n  Port {port} ({len(port_active)} active channels):")
    print(f"    Within-port spread: mean={np.mean(within_spreads):.2f} samples "
          f"({np.mean(within_spreads)*1000/FS:.1f} ms), "
          f"max={np.max(within_spreads)} samples ({np.max(within_spreads)*1000/FS:.1f} ms)")

# Cross-port spread using port medians
print(f"\n  Cross-port synchronization (port median transition samples):")
print(f"  {'Trans#':>6s}", end="")
for port in sorted(port_trans_medians.keys()):
    print(f" {'Port'+str(port):>8s}", end="")
print(f" {'Spread':>8s} {'Spread_ms':>10s}")

cross_port_spreads = []
for t in range(n_use):
    port_vals = [port_trans_medians[p][t] for p in sorted(port_trans_medians.keys())]
    spread = max(port_vals) - min(port_vals)
    cross_port_spreads.append(spread)
    print(f"  {t+1:6d}", end="")
    for v in port_vals:
        print(f" {v:8.0f}", end="")
    print(f" {spread:8.1f} {spread*1000/FS:10.1f}")

cross_port_spreads = np.array(cross_port_spreads)
print(f"\n  Cross-port spread: mean={np.mean(cross_port_spreads):.2f} samples "
      f"({np.mean(cross_port_spreads)*1000/FS:.1f} ms), "
      f"max={np.max(cross_port_spreads):.1f} samples ({np.max(cross_port_spreads)*1000/FS:.1f} ms)")

# Summary table: within-port vs cross-port
print(f"\n  Comparison: within-port vs cross-port sync")
print(f"    Within-port mean spreads:")
for port in sorted(port_within_spreads.keys()):
    print(f"      Port {port}: {np.mean(port_within_spreads[port]):.2f} samples")
all_within = np.concatenate([port_within_spreads[p] for p in port_within_spreads])
print(f"      OVERALL: {np.mean(all_within):.2f} samples ({np.mean(all_within)*1000/FS:.1f} ms)")
print(f"    Cross-port: {np.mean(cross_port_spreads):.2f} samples ({np.mean(cross_port_spreads)*1000/FS:.1f} ms)")

# ─── 5. PHASE COHERENCE (CROSS-CORRELATION AT LAG 0) ──────────────────────────
print("\n" + "="*80)
print("5. PHASE COHERENCE — ZERO-LAG CROSS-CORRELATION")
print("="*80)

# Pick one representative channel per device (ch1)
rep_channels = []
rep_labels   = []
for port in sorted(PORT_DEVICES.keys()):
    for dev in range(1, PORT_DEVICES[port]+1):
        col = f"Port{port}_dev{dev}_ch1"
        if col in ch_cols:
            rep_channels.append(col)
            rep_labels.append(f"P{port}D{dev}")

print(f"\nUsing {len(rep_channels)} representative channels (ch1 per device)")

# Get aligned binary signals for representative channels
rep_data = np.zeros((len(df), len(rep_channels)))
for i, col in enumerate(rep_channels):
    idx = ch_cols.index(col)
    rep_data[:, i] = binary_aligned[:, idx].astype(float)

# Compute correlation matrix
corr_matrix = np.corrcoef(rep_data.T)

print(f"\nCorrelation matrix stats (zero-lag cross-correlation of binary signals):")
# Mask diagonal
mask = ~np.eye(len(rep_channels), dtype=bool)
off_diag = corr_matrix[mask]
print(f"  Mean:   {np.mean(off_diag):.6f}")
print(f"  Median: {np.median(off_diag):.6f}")
print(f"  Min:    {np.min(off_diag):.6f} (pair: ", end="")
# Find min pair (excluding diagonal)
corr_for_min = corr_matrix.copy()
np.fill_diagonal(corr_for_min, 999)
min_idx = np.unravel_index(np.argmin(corr_for_min), corr_matrix.shape)
print(f"{rep_labels[min_idx[0]]} vs {rep_labels[min_idx[1]]})")
print(f"  Max:    {np.max(off_diag):.6f}")

# Within-port vs cross-port correlation
within_corrs = []
cross_corrs  = []
for i in range(len(rep_channels)):
    for j in range(i+1, len(rep_channels)):
        pi = ch_info[rep_channels[i]][0]
        pj = ch_info[rep_channels[j]][0]
        r  = corr_matrix[i, j]
        if pi == pj:
            within_corrs.append(r)
        else:
            cross_corrs.append(r)

print(f"\n  Within-port pairs: {len(within_corrs)}, mean r = {np.mean(within_corrs):.6f}")
print(f"  Cross-port pairs:  {len(cross_corrs)}, mean r = {np.mean(cross_corrs):.6f}")

# Per-port-pair cross-correlation
print(f"\n  Cross-port pair correlations (mean of device-pair correlations):")
from itertools import combinations
for p1, p2 in combinations(sorted(PORT_DEVICES.keys()), 2):
    pair_corrs = []
    for i in range(len(rep_channels)):
        for j in range(len(rep_channels)):
            if i != j and ch_info[rep_channels[i]][0] == p1 and ch_info[rep_channels[j]][0] == p2:
                pair_corrs.append(corr_matrix[i, j])
    if pair_corrs:
        print(f"    Port{p1} vs Port{p2}: mean r = {np.mean(pair_corrs):.6f}, "
              f"min r = {np.min(pair_corrs):.6f}")

# Also compute on RAW analog signals (not just binary) for finer detail
print(f"\n  Raw analog correlation (Pearson on raw ADC values, ch1 per device):")
rep_raw = np.zeros((len(df), len(rep_channels)))
for i, col in enumerate(rep_channels):
    idx = ch_cols.index(col)
    rep_raw[:, i] = data[:, idx]
raw_corr = np.corrcoef(rep_raw.T)
raw_mask = ~np.eye(len(rep_channels), dtype=bool)
raw_off = raw_corr[raw_mask]
print(f"  Mean:   {np.mean(raw_off):.6f}")
print(f"  Median: {np.median(raw_off):.6f}")
print(f"  Min:    {np.min(raw_off):.6f}")

# Within vs cross for raw
within_raw = []
cross_raw  = []
for i in range(len(rep_channels)):
    for j in range(i+1, len(rep_channels)):
        pi = ch_info[rep_channels[i]][0]
        pj = ch_info[rep_channels[j]][0]
        r  = raw_corr[i, j]
        if pi == pj:
            within_raw.append(r)
        else:
            cross_raw.append(r)
print(f"  Within-port: mean r = {np.mean(within_raw):.6f}")
print(f"  Cross-port:  mean r = {np.mean(cross_raw):.6f}")

# ─── 6. ANOMALY DETECTION ─────────────────────────────────────────────────────
print("\n" + "="*80)
print("6. ANOMALY DETECTION")
print("="*80)

# a) Inverted channels (already detected above)
print(f"\n  a) Inverted channels: {n_inverted}")
if n_inverted > 0:
    inv_chans = [ch_cols[i] for i in range(len(ch_cols)) if inverted[i]]
    for c in inv_chans[:30]:
        print(f"     {c}")
    if n_inverted > 30:
        print(f"     ... and {n_inverted - 30} more")

# b) DC channels
print(f"\n  b) DC channels (no square wave): {len(dc_channels)}")
for c in dc_channels:
    idx = ch_cols.index(c)
    print(f"     {c}: range=[{ch_mins[idx]:.0f}, {ch_maxs[idx]:.0f}], std={ch_stds[idx]:.1f}")

# c) Different frequency
duration_s = timestamps[-1] - timestamps[0]
expected_edges = int(duration_s)  # ~1 rising edge per second for 1 Hz
print(f"\n  c) Frequency anomalies (expected ~{expected_edges} rising edges for {duration_s:.1f}s at 1 Hz):")
freq_anomalies = []
for i, col in enumerate(ch_cols):
    n_rising = len(rising_edges_per_ch[col])
    if n_rising > 0 and abs(n_rising - expected_edges) > max(3, expected_edges * 0.1):
        freq_anomalies.append((col, n_rising))
if freq_anomalies:
    for col, n in freq_anomalies:
        print(f"     {col}: {n} rising edges (freq = {n/duration_s:.3f} Hz)")
else:
    print(f"     None detected — all channels at ~1 Hz")

# d) Significantly different amplitude
print(f"\n  d) Amplitude outliers (>3 std from mean peak-to-peak):")
pp_mean = np.mean(ch_pp)
pp_std  = np.std(ch_pp)
amp_outliers = []
for i, col in enumerate(ch_cols):
    z = (ch_pp[i] - pp_mean) / pp_std if pp_std > 0 else 0
    if abs(z) > 3:
        amp_outliers.append((col, ch_pp[i], z))
if amp_outliers:
    amp_outliers.sort(key=lambda x: x[2])
    for col, pp, z in amp_outliers:
        print(f"     {col}: pp={pp:.0f} (z={z:+.1f})")
else:
    print(f"     None detected — all within 3 std")

# e) Channels with unusual transition timing
print(f"\n  e) Channels with consistent transition offset from consensus:")
if n_use > 0:
    consensus_transitions = np.median(trans_matrix, axis=0)
    ch_offsets = trans_matrix - consensus_transitions[np.newaxis, :]
    ch_mean_offset = np.mean(ch_offsets, axis=1)

    outlier_thresh = 1.5  # samples
    timing_outliers = []
    for i, col in enumerate(active_channels):
        if abs(ch_mean_offset[i]) > outlier_thresh:
            timing_outliers.append((col, ch_mean_offset[i]))
    if timing_outliers:
        timing_outliers.sort(key=lambda x: x[1])
        for col, off in timing_outliers:
            print(f"     {col}: mean offset = {off:+.1f} samples ({off*1000/FS:+.1f} ms)")
    else:
        print(f"     None — all channels within 1.5 samples of consensus")

# ─── 7. JITTER ANALYSIS ───────────────────────────────────────────────────────
print("\n" + "="*80)
print("7. TRANSITION JITTER ANALYSIS")
print("="*80)

print(f"\n  Expected interval: {FS} samples (1 Hz at {FS} Hz sample rate)")

iti_all = []
iti_per_ch = {}
for col in active_channels:
    edges_i = rising_edges_per_ch[col]
    if len(edges_i) >= 2:
        iti = np.diff(edges_i)
        iti_per_ch[col] = iti
        iti_all.extend(iti)

iti_all = np.array(iti_all)
print(f"\n  Inter-transition intervals (all channels pooled):")
print(f"    N intervals: {len(iti_all)}")
print(f"    Mean:   {np.mean(iti_all):.3f} samples ({np.mean(iti_all)*1000/FS:.2f} ms)")
print(f"    Median: {np.median(iti_all):.0f} samples")
print(f"    Std:    {np.std(iti_all):.3f} samples ({np.std(iti_all)*1000/FS:.2f} ms)")
print(f"    Min:    {np.min(iti_all)} samples ({np.min(iti_all)*1000/FS:.1f} ms)")
print(f"    Max:    {np.max(iti_all)} samples ({np.max(iti_all)*1000/FS:.1f} ms)")

# Distribution of intervals
unique_iti, counts = np.unique(iti_all, return_counts=True)
print(f"\n  Interval distribution:")
for val, cnt in zip(unique_iti, counts):
    pct = 100.0 * cnt / len(iti_all)
    if pct > 0.01:
        bar = '#' * int(pct / 2)
        print(f"    {val:6d} samples ({val*1000/FS:8.1f} ms): {cnt:6d} ({pct:5.1f}%) {bar}")

# Drift analysis: do transitions drift over time?
print(f"\n  Drift analysis (transition timing vs expected):")
# Use a well-behaved channel
ref_col = active_channels[0]
ref_edges = rising_edges_per_ch[ref_col]
n_ref = len(ref_edges)
if n_ref >= 3:
    # Expected: edges at edge[0], edge[0]+250, edge[0]+500, ...
    expected = ref_edges[0] + np.arange(n_ref) * FS
    drift = ref_edges[:n_ref] - expected
    print(f"    Reference channel: {ref_col}")
    print(f"    N transitions: {n_ref}")
    print(f"    Drift over {n_ref} transitions:")
    print(f"      Start: {drift[0]:+d} samples")
    print(f"      End:   {drift[-1]:+d} samples")
    print(f"      Max absolute:   {np.max(np.abs(drift))} samples ({np.max(np.abs(drift))*1000/FS:.1f} ms)")
    total_drift = drift[-1] - drift[0]
    if n_ref > 1:
        ppm = total_drift * 1e6 / (FS * (n_ref - 1))
        print(f"      Total drift: {total_drift:+d} samples over {n_ref-1} cycles = {ppm:+.1f} ppm")

    # Show drift every 25 transitions
    print(f"\n    Drift trajectory (every ~25 transitions):")
    step = max(1, n_ref // 8)
    for k in range(0, n_ref, step):
        t_sec = (ref_edges[k] - ref_edges[0]) / FS
        print(f"      Trans #{k+1:3d} (t={t_sec:6.1f}s): drift = {drift[k]:+d} samples")
    # Last one
    if (n_ref - 1) % step != 0:
        t_sec = (ref_edges[-1] - ref_edges[0]) / FS
        print(f"      Trans #{n_ref:3d} (t={t_sec:6.1f}s): drift = {drift[-1]:+d} samples")

# Per-port jitter comparison
print(f"\n  Per-port jitter (std of inter-transition intervals):")
for port in sorted(PORT_DEVICES.keys()):
    port_active = [c for c in active_channels if ch_info[c][0] == port and c in iti_per_ch]
    if port_active:
        port_itis = np.concatenate([iti_per_ch[c] for c in port_active])
        print(f"    Port {port}: mean={np.mean(port_itis):.3f}, std={np.std(port_itis):.3f}, "
              f"range=[{np.min(port_itis)}, {np.max(port_itis)}]")

# ─── 8. VISUALIZATIONS ────────────────────────────────────────────────────────
print("\n" + "="*80)
print("8. GENERATING VISUALIZATIONS")
print("="*80)

# Plot 1: One second of raw data from one channel per port
fig, axes = plt.subplots(7, 1, figsize=(14, 16), sharex=True)
# Pick a window around the 3rd transition
t_center = int(np.median(trans_matrix[:, 2]))
t_start  = max(0, t_center - 50)
t_end    = min(len(df), t_center + 300)

for ax_i, port in enumerate(sorted(PORT_DEVICES.keys())):
    ax = axes[ax_i]
    port_ch1 = f'Port{port}_dev1_ch1'
    if port_ch1 in ch_cols:
        idx = ch_cols.index(port_ch1)
        vals = data[t_start:t_end, idx]
        x = np.arange(t_start, t_end)
        ax.plot(x, vals, linewidth=0.8)
        ax.set_ylabel(f'Port {port}\n(ADC)')
        if port_ch1 in active_channels:
            trans_sample = trans_matrix[active_channels.index(port_ch1), 2]
            ax.axvline(x=trans_sample, color='red', alpha=0.5, linestyle='--', label='Transition')
    ax.set_title(f'Port {port} - dev1_ch1', fontsize=10)
axes[-1].set_xlabel('Sample index')
fig.suptitle('Raw Test Signal - One Channel Per Port (around transition #3)', fontsize=13)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'raw_signal_per_port.png'), dpi=150)
plt.close()
print("  Saved: raw_signal_per_port.png")

# Plot 2: Transition alignment — zoom on one transition
fig, ax = plt.subplots(figsize=(14, 8))
t_center = int(np.median(trans_matrix[:, 2]))
t_start = max(0, t_center - 10)
t_end   = min(len(df), t_center + 15)
colors = plt.cm.tab10(np.linspace(0, 1, 7))
for pi, port in enumerate(sorted(PORT_DEVICES.keys())):
    col = f'Port{port}_dev1_ch1'
    if col in ch_cols:
        idx = ch_cols.index(col)
        vals = binary_aligned[t_start:t_end, idx]
        x = np.arange(t_start, t_end)
        ax.step(x, vals + pi * 0.05, where='post', linewidth=2, color=colors[pi],
                label=f'Port {port}', alpha=0.8)
ax.set_xlabel('Sample index')
ax.set_ylabel('Binary state (offset for visibility)')
ax.set_title('Transition Alignment - Binary Signal Per Port (dev1_ch1)', fontsize=13)
ax.legend(loc='upper left')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'transition_alignment.png'), dpi=150)
plt.close()
print("  Saved: transition_alignment.png")

# Plot 3: Peak-to-peak amplitude per device
fig, ax = plt.subplots(figsize=(14, 6))
dev_labels = []
dev_pps    = []
dev_colors = []
color_map = {1:'#1f77b4', 2:'#ff7f0e', 3:'#2ca02c', 4:'#d62728', 5:'#9467bd', 6:'#8c564b', 7:'#e377c2'}
for port in sorted(PORT_DEVICES.keys()):
    for dev in range(1, PORT_DEVICES[port]+1):
        dev_labels.append(f'P{port}D{dev}')
        dev_pps.append(device_pp.get((port, dev), 0))
        dev_colors.append(color_map[port])

ax.bar(range(len(dev_labels)), dev_pps, color=dev_colors, edgecolor='black', linewidth=0.5)
ax.set_xticks(range(len(dev_labels)))
ax.set_xticklabels(dev_labels, rotation=90, fontsize=7)
ax.set_ylabel('Peak-to-Peak Amplitude (ADC codes)')
ax.set_title('Peak-to-Peak Amplitude Per Device (mean of 8 channels)')
ax.axhline(y=np.mean(dev_pps), color='red', linestyle='--', alpha=0.5, label=f'Mean={np.mean(dev_pps):.0f}')
ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'amplitude_per_device.png'), dpi=150)
plt.close()
print("  Saved: amplitude_per_device.png")

# Plot 4: Correlation matrix heatmap (device-level)
fig, ax = plt.subplots(figsize=(14, 12))
im = ax.imshow(corr_matrix, vmin=np.min(off_diag)-0.01, vmax=1, cmap='RdBu_r', aspect='auto')
ax.set_xticks(range(len(rep_labels)))
ax.set_xticklabels(rep_labels, rotation=90, fontsize=6)
ax.set_yticks(range(len(rep_labels)))
ax.set_yticklabels(rep_labels, fontsize=6)
ax.set_title('Zero-Lag Cross-Correlation (Binary Signal, ch1 per device)')
plt.colorbar(im, ax=ax, shrink=0.8, label='Correlation')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'correlation_matrix.png'), dpi=150)
plt.close()
print("  Saved: correlation_matrix.png")

# Plot 5: Transition spread over time
fig, ax = plt.subplots(figsize=(12, 5))
ax.bar(range(1, n_use+1), spreads, color='steelblue', edgecolor='black')
ax.set_xlabel('Transition #')
ax.set_ylabel('Spread (samples)')
ax.set_title('Cross-Device Transition Spread Over Time')
ax.axhline(y=np.mean(spreads), color='red', linestyle='--', label=f'Mean={np.mean(spreads):.1f}')
ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'transition_spread.png'), dpi=150)
plt.close()
print("  Saved: transition_spread.png")

# Plot 6: Jitter histogram
fig, ax = plt.subplots(figsize=(10, 5))
ax.hist(iti_all, bins=range(int(np.min(iti_all))-1, int(np.max(iti_all))+3),
        color='steelblue', edgecolor='black', alpha=0.8)
ax.axvline(x=FS, color='red', linestyle='--', linewidth=2, label=f'Expected ({FS} samples)')
ax.set_xlabel('Inter-transition interval (samples)')
ax.set_ylabel('Count')
ax.set_title('Inter-Transition Interval Distribution (all channels)')
ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'jitter_histogram.png'), dpi=150)
plt.close()
print("  Saved: jitter_histogram.png")

# Plot 7: Detailed transition alignment for ALL devices around one edge
fig, axes = plt.subplots(7, 1, figsize=(16, 20), sharex=True)
t_idx = 2  # transition #3
t_center = int(np.median(trans_matrix[:, t_idx]))
win = 8  # samples on each side
t_start = max(0, t_center - win)
t_end   = min(len(df), t_center + win)

for ax_i, port in enumerate(sorted(PORT_DEVICES.keys())):
    ax = axes[ax_i]
    for dev in range(1, PORT_DEVICES[port]+1):
        col = f'Port{port}_dev{dev}_ch1'
        if col in ch_cols and col in active_channels:
            idx_c = ch_cols.index(col)
            vals = binary_aligned[t_start:t_end, idx_c]
            x = np.arange(t_start, t_end)
            offset = (dev - 1) * 0.03
            ax.step(x, vals + offset, where='post', linewidth=1.5, alpha=0.8,
                    label=f'dev{dev}')
    ax.set_ylabel(f'Port {port}')
    ax.legend(fontsize=7, ncol=4, loc='upper left')
    ax.axvline(x=t_center, color='gray', linestyle=':', alpha=0.5)
axes[-1].set_xlabel('Sample index')
fig.suptitle(f'Transition #{t_idx+1} - All Devices (binary, ch1)', fontsize=13)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'transition_detail_all_devices.png'), dpi=150)
plt.close()
print("  Saved: transition_detail_all_devices.png")

# Plot 8: Within-port vs cross-port spread comparison
fig, ax = plt.subplots(figsize=(10, 5))
port_labels = [f'Port {p}\n(within)' for p in sorted(port_within_spreads.keys())]
port_mean_spreads = [np.mean(port_within_spreads[p]) for p in sorted(port_within_spreads.keys())]
port_labels.append('Cross-port')
port_mean_spreads.append(np.mean(cross_port_spreads))
colors = ['steelblue'] * 7 + ['orangered']
ax.bar(range(len(port_labels)), port_mean_spreads, color=colors, edgecolor='black')
ax.set_xticks(range(len(port_labels)))
ax.set_xticklabels(port_labels, fontsize=9)
ax.set_ylabel('Mean Transition Spread (samples)')
ax.set_title('Within-Port vs Cross-Port Synchronization')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'within_vs_cross_port.png'), dpi=150)
plt.close()
print("  Saved: within_vs_cross_port.png")

# Plot 9: Drift over time
if n_ref >= 3:
    fig, ax = plt.subplots(figsize=(12, 5))
    t_seconds = (ref_edges - ref_edges[0]) / FS
    ax.plot(t_seconds, drift, 'b.-', markersize=3, linewidth=0.8)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Drift (samples from expected)')
    ax.set_title(f'Transition Drift Over Time ({ref_col})')
    ax.axhline(y=0, color='red', linestyle='--', alpha=0.5)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, 'drift_over_time.png'), dpi=150)
    plt.close()
    print("  Saved: drift_over_time.png")

# ─── 9. SUMMARY ───────────────────────────────────────────────────────────────
print("\n" + "="*80)
print("SUMMARY")
print("="*80)
print(f"""
Dataset: {len(df)} samples, {len(ch_cols)} channels, {duration_s:.1f} s
Sample rate: {FS} Hz

AMPLITUDE:
  Mean peak-to-peak: {np.mean(ch_pp):.1f} ADC codes
  Amplitude CV: {np.std(ch_pp)/np.mean(ch_pp)*100:.1f}%

SYNCHRONIZATION:
  Overall transition spread: {np.mean(spreads):.1f} samples mean, {np.max(spreads)} max
    = {np.mean(spreads)*1000/FS:.1f} ms mean, {np.max(spreads)*1000/FS:.1f} ms max
  Within-port spread: {np.mean(all_within):.2f} samples mean
    = {np.mean(all_within)*1000/FS:.1f} ms mean
  Cross-port spread: {np.mean(cross_port_spreads):.2f} samples mean
    = {np.mean(cross_port_spreads)*1000/FS:.1f} ms mean

COHERENCE (binary):
  All-pair correlation: mean={np.mean(off_diag):.6f}, min={np.min(off_diag):.6f}
  Within-port: mean r = {np.mean(within_corrs):.6f}
  Cross-port:  mean r = {np.mean(cross_corrs):.6f}

COHERENCE (raw analog):
  Within-port: mean r = {np.mean(within_raw):.6f}
  Cross-port:  mean r = {np.mean(cross_raw):.6f}

JITTER:
  Inter-transition interval: {np.mean(iti_all):.3f} +/- {np.std(iti_all):.3f} samples
  Range: [{np.min(iti_all)}, {np.max(iti_all)}] samples

ANOMALIES:
  Inverted channels: {n_inverted}
  DC channels: {len(dc_channels)}
  Frequency anomalies: {len(freq_anomalies)}
  Amplitude outliers: {len(amp_outliers)}

Plots saved to: {OUT_DIR}
""")

print("Analysis complete.")
