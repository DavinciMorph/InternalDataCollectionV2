"""
FirmwareIssue1.csv comprehensive analysis
Tasks 1-4: Pre-transition anomaly detection, timestamp micro-analysis,
65mV shift analysis, clean window comparison
"""

import numpy as np
import pandas as pd
import sys
import os

DATA_FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv"

print("=" * 80)
print("FIRMWARE ISSUE 1 - COMPREHENSIVE DATA ANALYSIS")
print("=" * 80)

# ============================================================
# LOAD DATA
# ============================================================
print("\n[LOADING] Reading CSV...")
df = pd.read_csv(DATA_FILE)
print(f"  Shape: {df.shape}")
print(f"  Columns: {df.columns[:5].tolist()} ... {df.columns[-3:].tolist()}")

timestamps = df.iloc[:, 0].values.astype(np.float64)
sample_numbers = df.iloc[:, 1].values.astype(np.int64)
channel_data = df.iloc[:, 2:].values.astype(np.float64)
n_samples, n_channels = channel_data.shape

print(f"  Samples: {n_samples}, Channels: {n_channels}")
print(f"  Time range: {timestamps[0]:.3f} - {timestamps[-1]:.3f} s")
print(f"  Duration: {timestamps[-1] - timestamps[0]:.1f} s")

# ============================================================
# IDENTIFY RAILED CHANNELS (constant at ADC min/max)
# ============================================================
ch_std = np.std(channel_data[:1000], axis=0)
ch_range = np.ptp(channel_data[:1000], axis=0)
railed_mask = ch_std < 10  # essentially zero variation
n_railed = np.sum(railed_mask)
active_mask = ~railed_mask
n_active = np.sum(active_mask)
print(f"\n  Railed channels: {n_railed}, Active channels: {n_active}")

# ============================================================
# TASK 1: PRE-TRANSITION ANOMALY DETECTION
# ============================================================
print("\n" + "=" * 80)
print("TASK 1: PRE-TRANSITION ANOMALY DETECTION (t=1450-1480s)")
print("=" * 80)

# Transition timestamps
T_SHIFT_1 = 1480.0
T_CLEAN_2_START = 1607.0
T_SHIFT_2 = 1640.0

# Select pre-transition window: 1450-1480
mask_pre = (timestamps >= 1450) & (timestamps < 1480)
mask_clean = (timestamps >= 1400) & (timestamps < 1450)  # reference clean
mask_bad = (timestamps >= 1490) & (timestamps < 1550)    # deep in bad region

# Per-sample variance across active channels
active_data = channel_data[:, active_mask]
per_sample_var = np.var(active_data, axis=1)

# Compute in 1-second windows
window_sec = 1.0
t_starts = np.arange(1400, 1500, window_sec)
var_windows = []
mean_windows = []
std_windows = []
for t in t_starts:
    win_mask = (timestamps >= t) & (timestamps < t + window_sec)
    if np.sum(win_mask) > 0:
        win_data = active_data[win_mask]
        var_windows.append(np.mean(np.var(win_data, axis=1)))
        mean_windows.append(np.mean(win_data))
        std_windows.append(np.mean(np.std(win_data, axis=1)))
    else:
        var_windows.append(np.nan)
        mean_windows.append(np.nan)
        std_windows.append(np.nan)

print("\n  Per-second statistics (active channels), t=1400-1500:")
print(f"  {'Time':>8s} {'Mean Var':>12s} {'Mean Std':>12s} {'Grand Mean':>12s}")
for i, t in enumerate(t_starts):
    flag = " <-- SHIFT" if t >= T_SHIFT_1 else ""
    print(f"  {t:8.0f} {var_windows[i]:12.0f} {std_windows[i]:12.1f} {mean_windows[i]:12.0f}{flag}")

# Finer-grained analysis: 0.1s windows in the 5 seconds before transition
print("\n  Fine-grained 0.1s windows around transition (t=1478-1482):")
t_fine = np.arange(1478, 1482, 0.1)
for t in t_fine:
    win_mask = (timestamps >= t) & (timestamps < t + 0.1)
    n_win = np.sum(win_mask)
    if n_win > 0:
        win_data = active_data[win_mask]
        v = np.mean(np.var(win_data, axis=1))
        m = np.mean(win_data)
        s = np.mean(np.std(win_data, axis=1))
        flag = " <-- TRANSITION ZONE" if t >= T_SHIFT_1 - 0.2 else ""
        print(f"    t={t:8.2f} n={n_win:3d} mean={m:12.0f} std={s:10.1f} var={v:12.0f}{flag}")

# Per-channel analysis: detect drift in individual channels before transition
print("\n  Per-channel drift analysis (1450-1480 vs 1400-1450):")
clean_ref = active_data[mask_clean]
pre_trans = active_data[mask_pre]
clean_mean = np.mean(clean_ref, axis=0)
pre_mean = np.mean(pre_trans, axis=0)
drift = pre_mean - clean_mean
print(f"    Mean drift magnitude: {np.mean(np.abs(drift)):.1f} LSB")
print(f"    Max drift channel: {np.argmax(np.abs(drift))} ({np.max(np.abs(drift)):.1f} LSB)")
print(f"    Median drift: {np.median(drift):.1f} LSB")
print(f"    Drift std: {np.std(drift):.1f} LSB")

# Check if per-sample cross-channel variance starts increasing before the shift
print("\n  Per-sample cross-channel variance trend (1s windows):")
for t in [1470, 1475, 1478, 1479, 1479.5, 1480, 1480.5, 1481, 1482, 1485]:
    win = (timestamps >= t) & (timestamps < t + 1)
    if np.sum(win) > 0:
        v = np.mean(per_sample_var[win])
        print(f"    t={t:8.1f}: cross-ch variance = {v:14.0f}")

# ============================================================
# TASK 2: TIMESTAMP MICRO-ANALYSIS
# ============================================================
print("\n" + "=" * 80)
print("TASK 2: TIMESTAMP MICRO-ANALYSIS")
print("=" * 80)

dt = np.diff(timestamps) * 1000.0  # ms
dt_timestamps = timestamps[1:]

print(f"\n  Global dt statistics:")
print(f"    Mean: {np.mean(dt):.4f} ms")
print(f"    Std:  {np.std(dt):.4f} ms")
print(f"    Min:  {np.min(dt):.4f} ms")
print(f"    Max:  {np.max(dt):.4f} ms")
print(f"    Median: {np.median(dt):.4f} ms")

# Check for sample number gaps
sn_diff = np.diff(sample_numbers)
gaps = np.where(sn_diff != 1)[0]
print(f"\n  Sample number gaps: {len(gaps)}")
if len(gaps) > 0:
    for g in gaps[:10]:
        print(f"    At sample {sample_numbers[g]}: jump by {sn_diff[g]} (t={timestamps[g]:.3f}s)")

# 1-second windows around transitions
transitions = [
    ("Transition 1 (clean->bad)", 1479.5, 1480.5),
    ("Transition 2 (bad->clean)", 1606.5, 1607.5),
    ("Transition 3 (clean->bad)", 1639.5, 1640.5),
]

for name, t_start, t_end in transitions:
    print(f"\n  --- {name} (t={t_start:.1f}-{t_end:.1f}s) ---")
    mask_t = (dt_timestamps >= t_start) & (dt_timestamps < t_end)
    dt_window = dt[mask_t]
    t_window = dt_timestamps[mask_t]

    if len(dt_window) == 0:
        print("    No samples in window!")
        continue

    print(f"    Samples: {len(dt_window)}")
    print(f"    dt range: {np.min(dt_window):.4f} - {np.max(dt_window):.4f} ms")
    print(f"    dt mean: {np.mean(dt_window):.4f} ms, std: {np.std(dt_window):.4f} ms")

    # Flag any outliers
    outlier_mask = np.abs(dt_window - 4.0) > 0.3
    n_outliers = np.sum(outlier_mask)
    if n_outliers > 0:
        print(f"    dt OUTLIERS (|dt-4.0| > 0.3ms): {n_outliers}")
        for idx in np.where(outlier_mask)[0][:5]:
            print(f"      t={t_window[idx]:.4f}s dt={dt_window[idx]:.4f}ms")
    else:
        print(f"    NO dt outliers (all within 3.7-4.3ms)")

    # Sample-by-sample listing of extreme dt values
    sorted_idx = np.argsort(np.abs(dt_window - 4.0))[::-1]
    print(f"    Top 5 most deviant dt values:")
    for rank, idx in enumerate(sorted_idx[:5]):
        print(f"      #{rank+1}: t={t_window[idx]:.4f}s dt={dt_window[idx]:.4f}ms")

# dt statistics in 10-second windows across full recording
print("\n  dt statistics in 10-second windows:")
t_win_starts = np.arange(timestamps[0], timestamps[-1], 10)
print(f"  {'Window':>10s} {'Mean':>8s} {'Std':>8s} {'Min':>8s} {'Max':>8s} {'Region'}")
for t in t_win_starts:
    mask_w = (dt_timestamps >= t) & (dt_timestamps < t + 10)
    if np.sum(mask_w) < 10:
        continue
    dt_w = dt[mask_w]

    if t >= 1480 and t < 1607:
        region = "BAD-1"
    elif t >= 1607 and t < 1640:
        region = "CLEAN-2"
    elif t >= 1640 and t < 1663:
        region = "BAD-2"
    elif t >= 1400 and t < 1480:
        region = "CLEAN-1"
    else:
        region = ""

    print(f"  {t:10.0f} {np.mean(dt_w):8.4f} {np.std(dt_w):8.4f} {np.min(dt_w):8.4f} {np.max(dt_w):8.4f} {region}")

# ============================================================
# TASK 3: 65mV SHIFT ANALYSIS
# ============================================================
print("\n" + "=" * 80)
print("TASK 3: THE ~65mV SHIFT - ADS1299 FAILURE MODE ANALYSIS")
print("=" * 80)

# Get clean and bad region data
mask_clean1 = (timestamps >= 1400) & (timestamps < 1470)
mask_bad1 = (timestamps >= 1490) & (timestamps < 1600)

clean1_data = active_data[mask_clean1]
bad1_data = active_data[mask_bad1]

clean1_means = np.mean(clean1_data, axis=0)
bad1_means = np.mean(bad1_data, axis=0)

shift = bad1_means - clean1_means

print(f"\n  Shift statistics (active channels only, {n_active} channels):")
print(f"    Mean shift: {np.mean(shift):.0f} LSB")
print(f"    Median shift: {np.median(shift):.0f} LSB")
print(f"    Std of shift: {np.std(shift):.0f} LSB")
print(f"    Min shift: {np.min(shift):.0f} LSB")
print(f"    Max shift: {np.max(shift):.0f} LSB")

# Convert to voltage
# ADS1299: VREF = 4.5V, 24-bit signed -> 1 LSB = VREF / (2^23 * gain)
# At gain=24: 1 LSB = 4.5 / (8388608 * 24) = 22.35 nV
# At gain=1: 1 LSB = 4.5 / 8388608 = 536.4 nV
LSB_GAIN24_UV = 4.5 / (8388608 * 24) * 1e6  # microvolts
LSB_GAIN1_UV = 4.5 / 8388608 * 1e6

mean_shift_uv = np.mean(shift) * LSB_GAIN24_UV
mean_shift_mv = mean_shift_uv / 1000

print(f"\n  Voltage interpretation (assuming gain=24, VREF=4.5V):")
print(f"    1 LSB = {LSB_GAIN24_UV:.4f} uV = {LSB_GAIN24_UV*1e-3:.6f} mV")
print(f"    Mean shift: {mean_shift_uv:.1f} uV = {mean_shift_mv:.3f} mV")
print(f"    Mean shift: {np.mean(shift):.0f} LSB")

# Now check at gain=1
mean_shift_mv_g1 = np.mean(shift) * LSB_GAIN1_UV / 1000
print(f"\n  If this were gain=1: {mean_shift_mv_g1:.3f} mV")

# ADS1299 internal voltages analysis
print(f"\n  ADS1299 internal voltage comparison:")
print(f"    Shift magnitude: {abs(mean_shift_mv):.3f} mV at gain=24")
print(f"    Shift in input-referred terms: {abs(mean_shift_mv):.3f} mV / 24 = {abs(mean_shift_mv)/24:.4f} mV at electrode")
print(f"    Shift in input-referred terms: ~{abs(mean_shift_mv)/24*1000:.1f} uV at electrode")
print(f"    VREFP internal = 4.5V (way too large)")
print(f"    Internal test signal (1x) = +-1 * (VREFP / 2400) = +-1.875 mV")
print(f"    Internal test signal (2x) = +-2 * (VREFP / 2400) = +-3.75 mV")

# Histogram of shift values per channel
print(f"\n  Shift histogram (binned by 10K LSB):")
bins = np.arange(np.min(shift) - 50000, np.max(shift) + 50000, 10000)
hist, bin_edges = np.histogram(shift, bins=bins)
for i in range(len(hist)):
    if hist[i] > 0:
        print(f"    [{bin_edges[i]/1e6:.2f}M, {bin_edges[i+1]/1e6:.2f}M): {hist[i]} channels")

# Per-port analysis: is the shift identical across all ports?
print(f"\n  Per-port shift analysis:")
# Port structure (from memory): Port1=8dev, Port2=7dev, Port3=5dev, Port4=5dev, Port5=5dev, Port6=5dev, Port7=7dev
# 8 channels per device
port_devs = [8, 7, 5, 5, 5, 5, 7]
port_names = ["Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"]
ch_offset = 0
for pi, (name, ndev) in enumerate(zip(port_names, port_devs)):
    nch = ndev * 8
    # Get full channel indices for this port
    port_chs = list(range(ch_offset, ch_offset + nch))
    # Filter to active channels only
    # We need to map back to full channel space
    all_shift = bad1_means - clean1_means  # wait, this was on active_data
    # Recompute on full data
    ch_offset += nch

# Recompute per-port on FULL channel space
ch_offset = 0
full_clean1 = channel_data[mask_clean1]
full_bad1 = channel_data[mask_bad1]
full_clean1_means = np.mean(full_clean1, axis=0)
full_bad1_means = np.mean(full_bad1, axis=0)
full_shift = full_bad1_means - full_clean1_means
full_std = np.std(channel_data[mask_clean1], axis=0)

for pi, (name, ndev) in enumerate(zip(port_names, port_devs)):
    nch = ndev * 8
    port_shift = full_shift[ch_offset:ch_offset + nch]
    port_std = full_std[ch_offset:ch_offset + nch]
    # Separate railed vs active based on std
    railed_in_port = port_std < 10
    active_in_port = ~railed_in_port
    n_active_port = np.sum(active_in_port)
    if n_active_port > 0:
        mean_s = np.mean(port_shift[active_in_port])
        std_s = np.std(port_shift[active_in_port])
        print(f"    {name} ({ndev}dev, {n_active_port}/{nch}ch active): "
              f"mean shift={mean_s:.0f} LSB ({mean_s*LSB_GAIN24_UV/1000:.3f} mV), "
              f"std={std_s:.0f}")
    else:
        print(f"    {name} ({ndev}dev, all railed)")
    ch_offset += nch

# Cross-channel correlation analysis
print(f"\n  Cross-channel correlation (clean vs bad):")
# Sample 50 active channels for correlation
if n_active > 50:
    sample_idx = np.random.RandomState(42).choice(np.where(active_mask)[0], 50, replace=False)
else:
    sample_idx = np.where(active_mask)[0]

clean_corr = np.corrcoef(channel_data[mask_clean1][:, sample_idx].T)
bad_corr = np.corrcoef(channel_data[mask_bad1][:, sample_idx].T)

# Off-diagonal mean
clean_offdiag = clean_corr[np.triu_indices_from(clean_corr, k=1)]
bad_offdiag = bad_corr[np.triu_indices_from(bad_corr, k=1)]

print(f"    Clean region mean correlation: {np.mean(clean_offdiag):.6f}")
print(f"    Bad region mean correlation:   {np.mean(bad_offdiag):.6f}")
print(f"    Clean region std correlation:  {np.std(clean_offdiag):.6f}")
print(f"    Bad region std correlation:    {np.std(bad_offdiag):.6f}")

# Is the shift ADDITIVE or MULTIPLICATIVE?
print(f"\n  Additive vs multiplicative test:")
clean_range = np.mean(np.ptp(clean1_data[:250], axis=0))  # 1s window, per-ch range
bad_range = np.mean(np.ptp(bad1_data[:250], axis=0))
print(f"    Clean per-ch range (1s): {clean_range:.0f} LSB")
print(f"    Bad per-ch range (1s):   {bad_range:.0f} LSB")
print(f"    Ratio: {bad_range / clean_range:.4f} (1.0 = pure additive)")

# ============================================================
# TASK 4: THE 33-SECOND CLEAN WINDOW (1607-1640s)
# ============================================================
print("\n" + "=" * 80)
print("TASK 4: THE 33-SECOND CLEAN WINDOW (t=1607-1640s)")
print("=" * 80)

mask_clean2 = (timestamps >= 1610) & (timestamps < 1637)
clean2_data = active_data[mask_clean2]
clean2_means = np.mean(clean2_data, axis=0)

# Compare to original clean period
print(f"\n  Comparison: Clean-1 (1400-1470) vs Clean-2 (1610-1637):")
print(f"    Clean-1 grand mean: {np.mean(clean1_means):.0f} LSB")
print(f"    Clean-2 grand mean: {np.mean(clean2_means):.0f} LSB")
print(f"    Difference: {np.mean(clean2_means) - np.mean(clean1_means):.0f} LSB")

# Per-channel comparison
mean_diff = clean2_means - clean1_means
print(f"    Per-channel mean difference: {np.mean(np.abs(mean_diff)):.1f} LSB (mean abs)")
print(f"    Per-channel mean difference: {np.median(np.abs(mean_diff)):.1f} LSB (median abs)")
print(f"    Max per-channel difference: {np.max(np.abs(mean_diff)):.0f} LSB")

# Noise floor comparison
clean1_std_per_ch = np.std(clean1_data, axis=0)
clean2_std_per_ch = np.std(clean2_data, axis=0)
print(f"\n    Clean-1 noise floor (mean per-ch std): {np.mean(clean1_std_per_ch):.1f} LSB")
print(f"    Clean-2 noise floor (mean per-ch std): {np.mean(clean2_std_per_ch):.1f} LSB")
print(f"    Ratio: {np.mean(clean2_std_per_ch) / np.mean(clean1_std_per_ch):.4f}")

# Correlation comparison
clean2_corr = np.corrcoef(channel_data[mask_clean2][:, sample_idx].T)
clean2_offdiag = clean2_corr[np.triu_indices_from(clean2_corr, k=1)]
print(f"\n    Clean-1 mean cross-correlation: {np.mean(clean_offdiag):.6f}")
print(f"    Clean-2 mean cross-correlation: {np.mean(clean2_offdiag):.6f}")

# Per-sample variance comparison
clean1_psv = np.mean(per_sample_var[mask_clean1])
clean2_psv = np.mean(per_sample_var[mask_clean2])
bad1_psv = np.mean(per_sample_var[mask_bad1])
print(f"\n    Clean-1 per-sample variance: {clean1_psv:.0f}")
print(f"    Clean-2 per-sample variance: {clean2_psv:.0f}")
print(f"    Bad-1 per-sample variance:   {bad1_psv:.0f}")

# Is clean-2 fully recovered or partially?
recovery_pct = 1.0 - abs(np.mean(clean2_means) - np.mean(clean1_means)) / abs(np.mean(bad1_means) - np.mean(clean1_means))
print(f"\n    Recovery percentage (mean level): {recovery_pct*100:.1f}%")

# Transition sharpness: how many samples does the shift take?
print(f"\n  Transition sharpness analysis:")
# Around first transition (1480s)
trans_mask = (timestamps >= 1479) & (timestamps < 1482)
trans_data = channel_data[trans_mask]
trans_t = timestamps[trans_mask]

# Use the grand mean across active channels
trans_mean = np.mean(trans_data[:, active_mask], axis=1)

# Find the sample where the step happens
diffs = np.abs(np.diff(trans_mean))
max_diff_idx = np.argmax(diffs)
print(f"    Transition 1 (clean->bad):")
print(f"      Max single-sample jump at t={trans_t[max_diff_idx]:.4f}s")
print(f"      Jump magnitude: {diffs[max_diff_idx]:.0f} LSB")
print(f"      Sample-by-sample mean around transition:")
for i in range(max(0, max_diff_idx-3), min(len(trans_mean), max_diff_idx+5)):
    print(f"        t={trans_t[i]:.4f}s  mean={trans_mean[i]:.0f}")

# Around second transition (1607s)
trans_mask2 = (timestamps >= 1606) & (timestamps < 1609)
trans_data2 = channel_data[trans_mask2]
trans_t2 = timestamps[trans_mask2]
trans_mean2 = np.mean(trans_data2[:, active_mask], axis=1)
diffs2 = np.abs(np.diff(trans_mean2))
max_diff_idx2 = np.argmax(diffs2)
print(f"\n    Transition 2 (bad->clean):")
print(f"      Max single-sample jump at t={trans_t2[max_diff_idx2]:.4f}s")
print(f"      Jump magnitude: {diffs2[max_diff_idx2]:.0f} LSB")
print(f"      Sample-by-sample mean around transition:")
for i in range(max(0, max_diff_idx2-3), min(len(trans_mean2), max_diff_idx2+5)):
    print(f"        t={trans_t2[i]:.4f}s  mean={trans_mean2[i]:.0f}")

# Around third transition (1640s)
trans_mask3 = (timestamps >= 1639) & (timestamps < 1642)
trans_data3 = channel_data[trans_mask3]
trans_t3 = timestamps[trans_mask3]
trans_mean3 = np.mean(trans_data3[:, active_mask], axis=1)
diffs3 = np.abs(np.diff(trans_mean3))
max_diff_idx3 = np.argmax(diffs3)
print(f"\n    Transition 3 (clean->bad):")
print(f"      Max single-sample jump at t={trans_t3[max_diff_idx3]:.4f}s")
print(f"      Jump magnitude: {diffs3[max_diff_idx3]:.0f} LSB")
print(f"      Sample-by-sample mean around transition:")
for i in range(max(0, max_diff_idx3-3), min(len(trans_mean3), max_diff_idx3+5)):
    print(f"        t={trans_t3[i]:.4f}s  mean={trans_mean3[i]:.0f}")

# Check if ALL channels shift on the SAME sample
print(f"\n  Per-channel transition alignment (Transition 1):")
trans_wide = (timestamps >= 1479.5) & (timestamps < 1481)
if np.sum(trans_wide) > 0:
    tw_data = channel_data[trans_wide]
    tw_t = timestamps[trans_wide]
    tw_diffs = np.diff(tw_data[:, active_mask], axis=0)
    # For each active channel, find the sample with the largest absolute diff
    max_diff_per_ch = np.argmax(np.abs(tw_diffs), axis=0)
    unique_vals, counts = np.unique(max_diff_per_ch, return_counts=True)
    print(f"    Max-diff sample index distribution across active channels:")
    for v, c in sorted(zip(unique_vals, counts), key=lambda x: -x[1])[:10]:
        t_at_v = tw_t[v] if v < len(tw_t) else 0
        print(f"      Sample idx {v} (t~{t_at_v:.4f}s): {c} channels ({c/n_active*100:.1f}%)")

print("\n" + "=" * 80)
print("ANALYSIS COMPLETE")
print("=" * 80)
