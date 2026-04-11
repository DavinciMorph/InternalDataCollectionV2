"""
Deep analysis of FirmwareIssue1 transition mechanics.
Focus on:
1. Exact transition timing -- is it truly gradual?
2. The per-channel transition spread and alignment
3. Bad region signal characteristics (noise, correlation, railing)
4. Whether the 62mV shift is truly additive
5. Status byte analysis (column data)
"""

import numpy as np
import pandas as pd

DATA_FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv"

print("=" * 80)
print("DEEP ANALYSIS - TRANSITION MECHANICS & BAD REGION CHARACTERISTICS")
print("=" * 80)

df = pd.read_csv(DATA_FILE)
timestamps = df.iloc[:, 0].values.astype(np.float64)
sample_numbers = df.iloc[:, 1].values.astype(np.int64)
channel_data = df.iloc[:, 2:].values.astype(np.float64)
n_samples, n_channels = channel_data.shape
col_names = df.columns[2:].tolist()

# Identify railed channels using initial clean region
mask_init_clean = (timestamps >= 1350) & (timestamps < 1400)
ch_std_init = np.std(channel_data[mask_init_clean], axis=0)
railed_mask = ch_std_init < 100
active_mask = ~railed_mask
n_railed = np.sum(railed_mask)
n_active = np.sum(active_mask)

print(f"\nRailed channels (std < 100 in initial clean region): {n_railed}")
if n_railed > 0:
    for i in np.where(railed_mask)[0]:
        val = channel_data[mask_init_clean, i].mean()
        print(f"  {col_names[i]}: mean={val:.0f}, std={ch_std_init[i]:.1f}")

# ============================================================
# TRANSITION 1 DETAILED ANALYSIS: t=1480-1490s
# ============================================================
print("\n" + "=" * 80)
print("TRANSITION 1 DETAILED: Was the onset truly gradual?")
print("=" * 80)

# Grand mean across all active channels, in very fine time bins
active_data = channel_data[:, active_mask]
active_names = [col_names[i] for i in range(n_channels) if active_mask[i]]

# Sample-by-sample grand mean from t=1479 to t=1495
mask_t1 = (timestamps >= 1479) & (timestamps < 1495)
t1_data = active_data[mask_t1]
t1_t = timestamps[mask_t1]

# Compute grand mean per sample
t1_grand_mean = np.mean(t1_data, axis=1)

# Find where the mean starts deviating from the clean baseline
clean_baseline = np.mean(t1_grand_mean[:200])  # first ~0.8s
print(f"\nClean baseline (first 200 samples): {clean_baseline:.0f} LSB")

# Track deviation from baseline
deviations = t1_grand_mean - clean_baseline
threshold_1pct = abs(clean_baseline) * 0.01  # 1% deviation
threshold_10pct = abs(clean_baseline) * 0.10  # 10% deviation

# Find first sample exceeding each threshold (sustained for 10 samples)
for name, thresh in [("1% deviation", threshold_1pct), ("10% deviation", threshold_10pct)]:
    for i in range(len(deviations) - 10):
        if all(abs(deviations[i:i+10]) > thresh):
            print(f"  First sustained {name} at t={t1_t[i]:.4f}s (sample offset +{i})")
            break

# 2-second windows of grand mean
print(f"\n  Grand mean in 0.5s windows (t=1480-1492):")
for t_start in np.arange(1480, 1492, 0.5):
    mask_w = (timestamps >= t_start) & (timestamps < t_start + 0.5)
    if np.sum(mask_w) > 0:
        w_mean = np.mean(active_data[mask_w])
        w_std = np.std(np.mean(active_data[mask_w], axis=1))
        shift_from_base = w_mean - clean_baseline
        print(f"    t={t_start:7.1f}: mean={w_mean:12.0f}  shift={shift_from_base:10.0f}  "
              f"sample-mean-std={w_std:.0f}")

# ============================================================
# PER-DEVICE TRANSITION ANALYSIS
# ============================================================
print("\n" + "=" * 80)
print("PER-DEVICE TRANSITION ANALYSIS")
print("=" * 80)

port_devs = [8, 7, 5, 5, 5, 5, 7]
port_names = ["Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"]

# For each device, compute mean across its 8 channels over time
# Focus on t=1484-1490 where the big shift happens (from Task 1 we saw 1486-1490)
mask_transition = (timestamps >= 1484) & (timestamps < 1492)
trans_t = timestamps[mask_transition]
trans_data = channel_data[mask_transition]

ch_offset = 0
print(f"\nPer-device mean in 1s windows during transition (t=1484-1492):")
for pi, (pname, ndev) in enumerate(zip(port_names, port_devs)):
    for d in range(ndev):
        dev_chs = slice(ch_offset, ch_offset + 8)
        dev_data = trans_data[:, dev_chs]
        dev_std_init = ch_std_init[dev_chs]
        n_active_ch = np.sum(dev_std_init >= 100)

        if n_active_ch == 0:
            ch_offset += 8
            continue

        # Use only active channels for this device
        dev_active = dev_data[:, dev_std_init >= 100]
        dev_mean_per_sample = np.mean(dev_active, axis=1)

        # 1s windows
        vals = []
        for t_start in np.arange(1484, 1492, 1.0):
            mask_w = (trans_t >= t_start) & (trans_t < t_start + 1.0)
            if np.sum(mask_w) > 0:
                vals.append(np.mean(dev_mean_per_sample[mask_w]))
            else:
                vals.append(np.nan)

        # Only print if there's a significant shift
        if len(vals) >= 4 and not any(np.isnan(vals)):
            shift = vals[-1] - vals[0]
            if abs(shift) > 100000:
                times = [f"{v/1e6:7.2f}M" for v in vals]
                print(f"  {pname}_dev{d+1} ({n_active_ch}ch): " + " -> ".join(times))

        ch_offset += 8

# ============================================================
# IS THE SHIFT EXACTLY THE SAME ON ALL CHANNELS?
# ============================================================
print("\n" + "=" * 80)
print("SHIFT UNIFORMITY ANALYSIS")
print("=" * 80)

mask_clean1 = (timestamps >= 1400) & (timestamps < 1470)
mask_bad_deep = (timestamps >= 1550) & (timestamps < 1600)  # deep in bad region

clean1_means = np.mean(channel_data[mask_clean1], axis=0)
bad_deep_means = np.mean(channel_data[mask_bad_deep], axis=0)
full_shift = bad_deep_means - clean1_means

# Focus on active channels
active_shift = full_shift[active_mask]

# Distribution of shift values
print(f"\n  Active channel shifts (n={n_active}):")
print(f"    Mean:   {np.mean(active_shift):12.0f} LSB")
print(f"    Median: {np.median(active_shift):12.0f} LSB")
print(f"    Std:    {np.std(active_shift):12.0f} LSB")
print(f"    P5:     {np.percentile(active_shift, 5):12.0f} LSB")
print(f"    P25:    {np.percentile(active_shift, 25):12.0f} LSB")
print(f"    P75:    {np.percentile(active_shift, 75):12.0f} LSB")
print(f"    P95:    {np.percentile(active_shift, 95):12.0f} LSB")

# Check if channels near 0 in clean period shift differently
clean1_active_means = clean1_means[active_mask]
# Correlate shift magnitude with clean-period value
from scipy import stats as scipy_stats
r, p = scipy_stats.pearsonr(clean1_active_means, active_shift)
print(f"\n    Correlation(clean_mean, shift): r={r:.4f}, p={p:.2e}")
print(f"    -> {'Shift depends on channel baseline!' if abs(r) > 0.3 else 'Shift is independent of baseline'}")

# Check channels near ADC rail during bad period
bad_near_rail = np.sum(np.abs(bad_deep_means[active_mask]) > 8000000)
print(f"\n    Channels near ADC rail during bad period: {bad_near_rail}/{n_active}")

# ============================================================
# BAD REGION NOISE CHARACTERISTICS
# ============================================================
print("\n" + "=" * 80)
print("BAD REGION SIGNAL CHARACTERISTICS")
print("=" * 80)

# Per-channel std in clean vs bad
clean_std = np.std(channel_data[mask_clean1][:, active_mask], axis=0)
bad_std = np.std(channel_data[mask_bad_deep][:, active_mask], axis=0)

print(f"\n  Noise floor (per-channel std, active channels):")
print(f"    Clean: mean={np.mean(clean_std):.0f}, median={np.median(clean_std):.0f}, "
      f"P5={np.percentile(clean_std,5):.0f}, P95={np.percentile(clean_std,95):.0f}")
print(f"    Bad:   mean={np.mean(bad_std):.0f}, median={np.median(bad_std):.0f}, "
      f"P5={np.percentile(bad_std,5):.0f}, P95={np.percentile(bad_std,95):.0f}")
print(f"    Ratio: mean={np.mean(bad_std)/np.mean(clean_std):.3f}, "
      f"median={np.median(bad_std)/np.median(clean_std):.3f}")

# Channels with MUCH higher noise in bad region
bad_noise_ratio = bad_std / (clean_std + 1)
high_noise = np.sum(bad_noise_ratio > 3)
low_noise = np.sum(bad_noise_ratio < 0.5)
print(f"\n    Channels with >3x noise increase: {high_noise}/{n_active}")
print(f"    Channels with <0.5x noise decrease: {low_noise}/{n_active}")

# Channels that rail during bad period but not during clean
clean_vals = channel_data[mask_clean1][:, active_mask]
bad_vals = channel_data[mask_bad_deep][:, active_mask]

railed_in_bad = np.sum(np.min(bad_vals, axis=0) < -8300000)
railed_in_clean = np.sum(np.min(clean_vals, axis=0) < -8300000)
print(f"\n    Channels hitting ADC floor (-8.3M+) in clean: {railed_in_clean}")
print(f"    Channels hitting ADC floor (-8.3M+) in bad: {railed_in_bad}")
print(f"    NEW railed channels in bad period: {railed_in_bad - railed_in_clean}")

# ============================================================
# TRANSITION 1 PER-CHANNEL ONSET TIMING
# ============================================================
print("\n" + "=" * 80)
print("TRANSITION 1: PER-CHANNEL ONSET TIMING")
print("=" * 80)

# For each active channel, find the first sample where value deviates
# by more than 100K LSB from its baseline (computed in 1400-1470)
mask_pre = (timestamps >= 1400) & (timestamps < 1470)
baselines = np.mean(channel_data[mask_pre], axis=0)

# Scan from t=1484 onwards
mask_scan = (timestamps >= 1484) & (timestamps < 1495)
scan_data = channel_data[mask_scan]
scan_t = timestamps[mask_scan]

ONSET_THRESHOLD = 500000  # 500K LSB, well beyond noise

onset_times = []
for ci in range(n_channels):
    if not active_mask[ci]:
        continue
    deviations = np.abs(scan_data[:, ci] - baselines[ci])
    # Find first sample exceeding threshold sustained for 5 samples
    found = False
    for i in range(len(deviations) - 5):
        if np.all(deviations[i:i+5] > ONSET_THRESHOLD):
            onset_times.append((col_names[ci], scan_t[i], ci))
            found = True
            break
    if not found:
        # Check if it ever exceeds threshold
        max_dev = np.max(deviations)
        if max_dev < ONSET_THRESHOLD:
            onset_times.append((col_names[ci], np.nan, ci))  # never shifts

onset_df = pd.DataFrame(onset_times, columns=['channel', 'onset_time', 'col_idx'])
valid_onsets = onset_df.dropna(subset=['onset_time'])
no_onset = onset_df[onset_df['onset_time'].isna()]

print(f"\n  Channels with detectable shift (>{ONSET_THRESHOLD} LSB): {len(valid_onsets)}")
print(f"  Channels that never shift: {len(no_onset)}")

if len(valid_onsets) > 0:
    print(f"\n  Onset time statistics:")
    print(f"    Earliest: {valid_onsets['onset_time'].min():.4f}s")
    print(f"    Latest:   {valid_onsets['onset_time'].max():.4f}s")
    print(f"    Median:   {valid_onsets['onset_time'].median():.4f}s")
    print(f"    Span:     {valid_onsets['onset_time'].max() - valid_onsets['onset_time'].min():.4f}s")

    # Group by port
    ch_offset = 0
    for pi, (pname, ndev) in enumerate(zip(port_names, port_devs)):
        nch = ndev * 8
        port_ch_range = range(ch_offset, ch_offset + nch)
        port_onsets = valid_onsets[valid_onsets['col_idx'].isin(port_ch_range)]
        if len(port_onsets) > 0:
            print(f"\n    {pname}: onset range [{port_onsets['onset_time'].min():.4f}, "
                  f"{port_onsets['onset_time'].max():.4f}]s, "
                  f"median={port_onsets['onset_time'].median():.4f}s, n={len(port_onsets)}")
        ch_offset += nch

    # Time-ordered first 20 onset channels
    print(f"\n  First 20 channels to shift:")
    sorted_onsets = valid_onsets.nsmallest(20, 'onset_time')
    for _, row in sorted_onsets.iterrows():
        print(f"    t={row['onset_time']:.4f}s  {row['channel']}")

    # Last 20 channels to shift
    print(f"\n  Last 20 channels to shift:")
    sorted_last = valid_onsets.nlargest(20, 'onset_time')
    for _, row in sorted_last.iterrows():
        print(f"    t={row['onset_time']:.4f}s  {row['channel']}")

# ============================================================
# TRANSITION 2 (BAD->CLEAN) TIMING
# ============================================================
print("\n" + "=" * 80)
print("TRANSITION 2 (BAD->CLEAN): RECOVERY TIMING")
print("=" * 80)

# For each active channel, find when it returns to within 200K of baseline
mask_scan2 = (timestamps >= 1600) & (timestamps < 1612)
scan_data2 = channel_data[mask_scan2]
scan_t2 = timestamps[mask_scan2]

RECOVERY_THRESHOLD = 500000

recovery_times = []
for ci in range(n_channels):
    if not active_mask[ci]:
        continue
    deviations = np.abs(scan_data2[:, ci] - baselines[ci])
    # Find first sample where deviation drops below threshold sustained for 10 samples
    found = False
    for i in range(len(deviations) - 10):
        if np.all(deviations[i:i+10] < RECOVERY_THRESHOLD):
            recovery_times.append((col_names[ci], scan_t2[i], ci))
            found = True
            break
    if not found:
        recovery_times.append((col_names[ci], np.nan, ci))

rec_df = pd.DataFrame(recovery_times, columns=['channel', 'recovery_time', 'col_idx'])
valid_rec = rec_df.dropna(subset=['recovery_time'])
no_rec = rec_df[rec_df['recovery_time'].isna()]

print(f"\n  Channels that recover: {len(valid_rec)}")
print(f"  Channels that don't recover in window: {len(no_rec)}")

if len(valid_rec) > 0:
    print(f"\n  Recovery time statistics:")
    print(f"    Earliest: {valid_rec['recovery_time'].min():.4f}s")
    print(f"    Latest:   {valid_rec['recovery_time'].max():.4f}s")
    print(f"    Median:   {valid_rec['recovery_time'].median():.4f}s")
    print(f"    Span:     {valid_rec['recovery_time'].max() - valid_rec['recovery_time'].min():.4f}s")

# ============================================================
# TRANSITION 3 (CLEAN->BAD) - is it identical to Transition 1?
# ============================================================
print("\n" + "=" * 80)
print("TRANSITION 3 (CLEAN->BAD): COMPARISON WITH TRANSITION 1")
print("=" * 80)

# Shift in transition 3
mask_bad2 = (timestamps >= 1650) & (timestamps < 1662)
mask_clean2 = (timestamps >= 1615) & (timestamps < 1635)
clean2_means = np.mean(channel_data[mask_clean2], axis=0)
bad2_means = np.mean(channel_data[mask_bad2], axis=0)
shift2 = bad2_means - clean2_means

active_shift1 = full_shift[active_mask]
active_shift2 = shift2[active_mask]

print(f"\n  Shift 1 (clean1->bad1): mean={np.mean(active_shift1):.0f}, std={np.std(active_shift1):.0f}")
print(f"  Shift 2 (clean2->bad2): mean={np.mean(active_shift2):.0f}, std={np.std(active_shift2):.0f}")

# Correlation between the two shifts
r, p = scipy_stats.pearsonr(active_shift1, active_shift2)
print(f"\n  Correlation between shift1 and shift2: r={r:.6f}, p={p:.2e}")
print(f"  Mean shift ratio (shift2/shift1): {np.mean(active_shift2)/np.mean(active_shift1):.4f}")

# Per-channel ratio
ratio = active_shift2 / (active_shift1 + 0.1)  # avoid div by 0
print(f"  Per-channel ratio median: {np.median(ratio):.4f}")
print(f"  Per-channel ratio std:    {np.std(ratio):.4f}")

# ============================================================
# CHECK: Do channels that were railed in clean period shift too?
# ============================================================
print("\n" + "=" * 80)
print("RAILED CHANNEL BEHAVIOR")
print("=" * 80)

if n_railed > 0:
    for i in np.where(railed_mask)[0]:
        clean_val = clean1_means[i]
        bad_val = bad_deep_means[i]
        shift_val = bad_val - clean_val
        print(f"  {col_names[i]}: clean={clean_val:.0f}, bad={bad_val:.0f}, shift={shift_val:.0f}")

# ============================================================
# SAMPLE-LEVEL TRANSITION: EXACT 1-SAMPLE RESOLUTION
# ============================================================
print("\n" + "=" * 80)
print("SAMPLE-LEVEL TRANSITION: Port1_dev8_ch1 (col 58)")
print("=" * 80)

# Port1_dev8_ch1 = column index 58 (0-indexed in channel_data = CSV col 60)
ch58 = channel_data[:, 56]  # 0-indexed in channel_data (col 2 is index 0)
ch58_name = col_names[56]
print(f"  Channel: {ch58_name}")

# Actually let me find Port1_dev8_ch1 properly
target = "Port1_dev8_ch1"
target_idx = col_names.index(target)
print(f"  {target} -> channel_data column index: {target_idx}")

ch_target = channel_data[:, target_idx]

# Show sample-by-sample values around first transition
mask_around = (timestamps >= 1485) & (timestamps < 1490)
for i in np.where(mask_around)[0]:
    if i > 0:
        delta = ch_target[i] - ch_target[i-1]
        flag = " <-- LARGE STEP" if abs(delta) > 100000 else ""
        print(f"  t={timestamps[i]:.4f}s sn={sample_numbers[i]} val={ch_target[i]:10.0f} delta={delta:10.0f}{flag}")
    if i - np.where(mask_around)[0][0] > 50:
        print("  ... (truncated)")
        break

print("\n" + "=" * 80)
print("DEEP ANALYSIS COMPLETE")
print("=" * 80)
