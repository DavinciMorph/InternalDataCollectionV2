"""
Definitive transition analysis focusing on the REAL onset at ~1486.54s
and the exact sample where the shift begins.
"""

import numpy as np
import pandas as pd

DATA_FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv"

print("=" * 80)
print("DEFINITIVE TRANSITION ANALYSIS")
print("=" * 80)

df = pd.read_csv(DATA_FILE)
timestamps = df.iloc[:, 0].values.astype(np.float64)
sample_numbers = df.iloc[:, 1].values.astype(np.int64)
channel_data = df.iloc[:, 2:].values.astype(np.float64)
n_samples, n_channels = channel_data.shape
col_names = df.columns[2:].tolist()

# ============================================================
# EXACT TRANSITION SAMPLE IDENTIFICATION
# ============================================================
print("\n1. EXACT TRANSITION SAMPLE IDENTIFICATION")
print("   Scanning grand mean for step change around t=1486-1487")

# Compute grand mean of all channels (active + railed)
grand_mean = np.mean(channel_data, axis=1)

# Focus on t=1486.0 - 1487.5
mask_focus = (timestamps >= 1486.0) & (timestamps < 1487.5)
focus_idx = np.where(mask_focus)[0]
focus_gm = grand_mean[focus_idx]
focus_t = timestamps[focus_idx]
focus_sn = sample_numbers[focus_idx]

# Sample-by-sample deltas
deltas = np.diff(focus_gm)
# Find the 20 largest jumps
top_jumps = np.argsort(np.abs(deltas))[::-1][:20]

print(f"\n  Top 20 largest single-sample grand-mean jumps in [1486, 1487.5]:")
for rank, idx in enumerate(top_jumps):
    t = focus_t[idx]
    sn = focus_sn[idx]
    before = focus_gm[idx]
    after = focus_gm[idx + 1]
    delta = deltas[idx]
    dt_ms = (focus_t[idx+1] - focus_t[idx]) * 1000
    print(f"  #{rank+1:2d}: t={t:.4f}s sn={sn} delta={delta:12.0f} LSB  "
          f"before={before:12.0f} after={after:12.0f}  dt={dt_ms:.2f}ms")

# The transition is a ramp. Let's see the cumulative shift
print(f"\n  Cumulative shift in grand mean (sample-by-sample, t=1486.5-1487.0):")
mask_ramp = (timestamps >= 1486.48) & (timestamps < 1487.2)
ramp_idx = np.where(mask_ramp)[0]
baseline_val = grand_mean[ramp_idx[0]]
for i, idx in enumerate(ramp_idx):
    cumulative_shift = grand_mean[idx] - baseline_val
    if i % 5 == 0 or abs(cumulative_shift) > 100000:
        print(f"    t={timestamps[idx]:.4f}s sn={sample_numbers[idx]} "
              f"mean={grand_mean[idx]:12.0f}  cum_shift={cumulative_shift:12.0f}")
    if i > 100:
        print("    ... (continuing in 25-sample blocks)")
        break

# Continue in 25-sample blocks
for idx in ramp_idx[100::25]:
    cumulative_shift = grand_mean[idx] - baseline_val
    print(f"    t={timestamps[idx]:.4f}s sn={sample_numbers[idx]} "
          f"mean={grand_mean[idx]:12.0f}  cum_shift={cumulative_shift:12.0f}")

# ============================================================
# PER-SAMPLE PER-CHANNEL ANALYSIS AT THE TRANSITION
# ============================================================
print(f"\n\n2. PER-CHANNEL BEHAVIOR AT THE EXACT TRANSITION SAMPLE")
print(f"   Checking if all channels shift on the SAME sample")

# Find the sample where the biggest cumulative ramp starts
# From above, the ramp clearly starts around t=1486.54
# Let's check the exact sample

# Get 5 samples before and 10 after the onset
onset_t = 1486.535  # approximate from earlier analysis
onset_idx = np.searchsorted(timestamps, onset_t)

print(f"\n  Reference onset index: {onset_idx} (t={timestamps[onset_idx]:.4f}s)")
print(f"  Examining samples [{onset_idx-5}:{onset_idx+15}]")

# For a subset of channels from different ports, show values
port_devs = [8, 7, 5, 5, 5, 5, 7]
port_names = ["Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"]

# Pick first device, ch1 from each port
ch_offset = 0
representative_chs = []
for pi, (pname, ndev) in enumerate(zip(port_names, port_devs)):
    representative_chs.append((pname + "_dev1_ch1", ch_offset))
    ch_offset += ndev * 8

print(f"\n  Representative channels (dev1_ch1 from each port):")
print(f"  {'Sample':>8s} {'Timestamp':>12s}", end="")
for name, _ in representative_chs:
    print(f" {name:>15s}", end="")
print()

for i in range(onset_idx - 5, min(onset_idx + 20, n_samples)):
    line = f"  {sample_numbers[i]:8d} {timestamps[i]:12.4f}"
    for name, ci in representative_chs:
        line += f" {channel_data[i, ci]:15.0f}"
    # Flag rows with big delta from previous
    if i > onset_idx - 5:
        max_delta = max(abs(channel_data[i, ci] - channel_data[i-1, ci]) for _, ci in representative_chs)
        if max_delta > 50000:
            line += f" <-- STEP ({max_delta:.0f})"
    print(line)

# ============================================================
# THE KEY QUESTION: Does the shift ramp slowly or step?
# ============================================================
print(f"\n\n3. RAMP RATE ANALYSIS")
print(f"   Is the shift a smooth ramp or a series of discrete steps?")

# Compute per-sample delta of grand mean from t=1486 to 1492
mask_ramp_full = (timestamps >= 1486) & (timestamps < 1493)
ramp_full_idx = np.where(mask_ramp_full)[0]
ramp_gm = grand_mean[ramp_full_idx]
ramp_t = timestamps[ramp_full_idx]
ramp_deltas = np.diff(ramp_gm)

# Statistics of deltas
print(f"\n  Delta statistics during ramp (t=1486-1493):")
print(f"    Mean:   {np.mean(ramp_deltas):.1f}")
print(f"    Std:    {np.std(ramp_deltas):.1f}")
print(f"    Min:    {np.min(ramp_deltas):.0f}")
print(f"    Max:    {np.max(ramp_deltas):.0f}")
print(f"    P1:     {np.percentile(ramp_deltas, 1):.0f}")
print(f"    P99:    {np.percentile(ramp_deltas, 99):.0f}")

# Compare with clean region deltas
mask_clean_ref = (timestamps >= 1400) & (timestamps < 1450)
clean_ref_idx = np.where(mask_clean_ref)[0]
clean_gm = grand_mean[clean_ref_idx]
clean_deltas = np.diff(clean_gm)

print(f"\n  Delta statistics during clean (t=1400-1450):")
print(f"    Mean:   {np.mean(clean_deltas):.1f}")
print(f"    Std:    {np.std(clean_deltas):.1f}")
print(f"    Min:    {np.min(clean_deltas):.0f}")
print(f"    Max:    {np.max(clean_deltas):.0f}")

# The ramp duration
# From the 0.5s window analysis: t=1486.5 has shift -1.38M, t=1491.5 has -2.33M
# So the ramp takes about 5-6 seconds from onset to near-steady-state
# But is it TRULY continuous or are there step-wise components?

# 0.1s windows of cumulative shift
print(f"\n  Cumulative shift in 0.1s windows (t=1486.0-1492.0):")
baseline_gm = np.mean(grand_mean[(timestamps >= 1484) & (timestamps < 1486)])
for t in np.arange(1486.0, 1492.1, 0.1):
    w = (timestamps >= t) & (timestamps < t + 0.1)
    if np.sum(w) > 0:
        shift = np.mean(grand_mean[w]) - baseline_gm
        n_samp = np.sum(w)
        rate = shift / max(t - 1486.0, 0.1)  # LSB/s
        print(f"    t={t:.1f}: shift={shift:12.0f} LSB  ({shift/1e6:.3f}M)  "
              f"rate={rate:.0f} LSB/s  n={n_samp}")

# ============================================================
# TRANSITION 2 (BAD->CLEAN) EXACT ANALYSIS
# ============================================================
print(f"\n\n4. TRANSITION 2 (BAD->CLEAN) EXACT ANALYSIS")

# From earlier: recovery starts around t=1600-1602
mask_rec = (timestamps >= 1599) & (timestamps < 1612)
rec_idx = np.where(mask_rec)[0]
rec_gm = grand_mean[rec_idx]
rec_t = timestamps[rec_idx]

# Find the sample with the biggest positive jump (recovering)
rec_deltas = np.diff(rec_gm)
top_rec_jumps = np.argsort(rec_deltas)[::-1][:10]  # largest positive = recovery

print(f"\n  Top 10 largest positive jumps during recovery:")
for rank, idx in enumerate(top_rec_jumps):
    t = rec_t[idx]
    print(f"  #{rank+1}: t={t:.4f}s delta={rec_deltas[idx]:12.0f}")

# Recovery ramp rate
print(f"\n  Recovery ramp in 0.5s windows:")
bad_baseline = np.mean(grand_mean[(timestamps >= 1595) & (timestamps < 1599)])
clean_target = np.mean(grand_mean[(timestamps >= 1610) & (timestamps < 1630)])
for t in np.arange(1599, 1612, 0.5):
    w = (timestamps >= t) & (timestamps < t + 0.5)
    if np.sum(w) > 0:
        val = np.mean(grand_mean[w])
        recovery_pct = (val - bad_baseline) / (clean_target - bad_baseline) * 100
        print(f"    t={t:.1f}: mean={val:12.0f}  recovery={recovery_pct:6.1f}%")

# ============================================================
# TRANSITION 3 (CLEAN->BAD) EXACT ANALYSIS
# ============================================================
print(f"\n\n5. TRANSITION 3 (CLEAN->BAD) EXACT ANALYSIS")

mask_t3 = (timestamps >= 1639) & (timestamps < 1655)
t3_idx = np.where(mask_t3)[0]
t3_gm = grand_mean[t3_idx]
t3_t = timestamps[t3_idx]
t3_deltas = np.diff(t3_gm)

top_t3_jumps = np.argsort(np.abs(t3_deltas))[::-1][:10]
print(f"\n  Top 10 largest jumps during transition 3:")
for rank, idx in enumerate(top_t3_jumps):
    print(f"  #{rank+1}: t={t3_t[idx]:.4f}s delta={t3_deltas[idx]:12.0f}")

# Ramp rate
print(f"\n  Transition 3 ramp in 0.5s windows:")
clean2_baseline = np.mean(grand_mean[(timestamps >= 1635) & (timestamps < 1639)])
for t in np.arange(1639, 1655, 0.5):
    w = (timestamps >= t) & (timestamps < t + 0.5)
    if np.sum(w) > 0:
        val = np.mean(grand_mean[w])
        shift = val - clean2_baseline
        print(f"    t={t:.1f}: mean={val:12.0f}  shift={shift:12.0f}")

# ============================================================
# CRITICAL: dt analysis at EXACT transition moments
# ============================================================
print(f"\n\n6. dt MICRO-ANALYSIS AT EXACT TRANSITIONS")

dt = np.diff(timestamps) * 1000
dt_t = timestamps[1:]

# Transition 1 onset: t=1486.53-1486.55
print(f"\n  Transition 1 onset (t=1486.50-1486.60):")
mask_dt1 = (dt_t >= 1486.50) & (dt_t < 1486.60)
dt1 = dt[mask_dt1]
t1_dt = dt_t[mask_dt1]
for i in range(len(dt1)):
    flag = "" if abs(dt1[i] - 4.0) < 0.25 else f" <-- ANOMALY"
    print(f"    t={t1_dt[i]:.4f}s dt={dt1[i]:.4f}ms{flag}")

# Transition 2 recovery
print(f"\n  Transition 2 recovery region - checking for dt anomalies:")
mask_dt2 = (dt_t >= 1599) & (dt_t < 1610)
dt2 = dt[mask_dt2]
t2_dt = dt_t[mask_dt2]
outliers2 = np.abs(dt2 - 4.0) > 0.25
n_outliers2 = np.sum(outliers2)
print(f"    Samples in window: {len(dt2)}")
print(f"    dt outliers (|dt-4.0| > 0.25ms): {n_outliers2}")
if n_outliers2 > 0:
    for idx in np.where(outliers2)[0][:10]:
        print(f"      t={t2_dt[idx]:.4f}s dt={dt2[idx]:.4f}ms")

# ============================================================
# STATS THREAD I2C TIMING ANALYSIS
# ============================================================
print(f"\n\n7. STATS THREAD I2C TIMING CORRELATION")
print(f"   Stats thread runs every 10s and reads I2C")
print(f"   Does any 10s-periodic event correlate with the transitions?")

# Find dt anomalies across full recording and check if they have 10s periodicity
outlier_mask_full = np.abs(dt - 4.0) > 0.2
outlier_times = dt_t[outlier_mask_full]
outlier_vals = dt[outlier_mask_full]

print(f"\n  Total dt outliers (|dt-4.0| > 0.2ms): {len(outlier_times)}")
print(f"  dt > 4.2ms: {np.sum(dt > 4.2)}")
print(f"  dt < 3.8ms: {np.sum(dt < 3.8)}")

# Check for 10s periodicity in large dt values
dt_max_per_10s = []
for t in np.arange(timestamps[0], timestamps[-1], 10):
    w = (dt_t >= t) & (dt_t < t + 10)
    if np.sum(w) > 0:
        dt_max_per_10s.append(np.max(dt[w]))
    else:
        dt_max_per_10s.append(np.nan)

print(f"\n  Max dt per 10s window: mean={np.nanmean(dt_max_per_10s):.4f}ms, "
      f"std={np.nanstd(dt_max_per_10s):.4f}ms, max={np.nanmax(dt_max_per_10s):.4f}ms")

# ============================================================
# CHANNEL THAT SHIFTS EARLIEST: DEEPER LOOK
# ============================================================
print(f"\n\n8. EARLIEST-SHIFTING CHANNELS - PRECURSOR ANALYSIS")
print(f"   Port2_dev2_ch6, Port3_dev1_ch5, Port6_dev4_ch2, Port6_dev4_ch8")
print(f"   These shifted at t=1484.0s, 2.5s BEFORE the main transition")

early_channels = ["Port2_dev2_ch6", "Port3_dev1_ch5", "Port6_dev4_ch2", "Port6_dev4_ch8"]
for ch_name in early_channels:
    ci = col_names.index(ch_name)
    # Show values around t=1483-1487
    mask_ch = (timestamps >= 1483) & (timestamps < 1487)
    ch_vals = channel_data[mask_ch, ci]
    ch_t = timestamps[mask_ch]

    # Find the exact transition point
    ch_deltas = np.diff(ch_vals)
    big_jump_idx = np.argmax(np.abs(ch_deltas))
    print(f"\n  {ch_name}:")
    print(f"    Biggest single-sample jump: t={ch_t[big_jump_idx]:.4f}s "
          f"delta={ch_deltas[big_jump_idx]:.0f} LSB")
    print(f"    Before: {ch_vals[big_jump_idx]:.0f}, After: {ch_vals[big_jump_idx+1]:.0f}")

    # Show 5 samples around the jump
    start = max(0, big_jump_idx - 3)
    end = min(len(ch_vals), big_jump_idx + 5)
    for i in range(start, end):
        d = ch_vals[i] - ch_vals[i-1] if i > 0 else 0
        flag = " <-- JUMP" if i == big_jump_idx + 1 else ""
        print(f"      t={ch_t[i]:.4f}s val={ch_vals[i]:12.0f} delta={d:10.0f}{flag}")

# ============================================================
# NOISE STRUCTURE DURING BAD PERIOD
# ============================================================
print(f"\n\n9. NOISE STRUCTURE IN BAD PERIOD")
print(f"   Is the increased noise random or structured?")

# Compute inter-sample delta in bad period for a representative channel
target = "Port1_dev1_ch1"
ci = col_names.index(target)
mask_clean = (timestamps >= 1400) & (timestamps < 1450)
mask_bad = (timestamps >= 1500) & (timestamps < 1550)

clean_vals = channel_data[mask_clean, ci]
bad_vals = channel_data[mask_bad, ci]

clean_d = np.diff(clean_vals)
bad_d = np.diff(bad_vals)

print(f"\n  {target} inter-sample delta statistics:")
print(f"    Clean: std={np.std(clean_d):.0f}, max_abs={np.max(np.abs(clean_d)):.0f}")
print(f"    Bad:   std={np.std(bad_d):.0f}, max_abs={np.max(np.abs(bad_d)):.0f}")
print(f"    Ratio: {np.std(bad_d)/np.std(clean_d):.2f}x")

# Check autocorrelation of deltas in bad period
print(f"\n  Autocorrelation of deltas (bad period):")
from numpy.fft import fft
bad_d_norm = bad_d - np.mean(bad_d)
acf = np.correlate(bad_d_norm, bad_d_norm, mode='full')
acf = acf[len(acf)//2:]
acf = acf / acf[0]
print(f"    lag=1: {acf[1]:.4f}")
print(f"    lag=2: {acf[2]:.4f}")
print(f"    lag=5: {acf[5]:.4f}")
print(f"    lag=10: {acf[10]:.4f}")
print(f"    lag=25: {acf[25]:.4f}")
print(f"    lag=50: {acf[50]:.4f}")

# ============================================================
# PSD COMPARISON: CLEAN vs BAD
# ============================================================
print(f"\n\n10. POWER SPECTRAL DENSITY: CLEAN vs BAD")

from scipy import signal as sig

fs = 250.0
# Use 10-second segments for Welch PSD
clean_seg = channel_data[(timestamps >= 1410) & (timestamps < 1420), ci]
bad_seg = channel_data[(timestamps >= 1500) & (timestamps < 1510), ci]

f_clean, psd_clean = sig.welch(clean_seg, fs=fs, nperseg=1024, noverlap=512)
f_bad, psd_bad = sig.welch(bad_seg, fs=fs, nperseg=1024, noverlap=512)

print(f"\n  {target} PSD comparison:")
print(f"  {'Freq':>8s} {'Clean PSD':>14s} {'Bad PSD':>14s} {'Ratio':>8s}")
for fi in [0, 1, 2, 5, 10, 25, 50, 60, 70, 100, 125]:
    idx = np.argmin(np.abs(f_clean - fi))
    ratio = psd_bad[idx] / psd_clean[idx] if psd_clean[idx] > 0 else np.inf
    print(f"  {f_clean[idx]:8.1f} {psd_clean[idx]:14.0f} {psd_bad[idx]:14.0f} {ratio:8.2f}x")

# Total power
print(f"\n  Total power (0-125 Hz): clean={np.sum(psd_clean):.0f}, bad={np.sum(psd_bad):.0f}, "
      f"ratio={np.sum(psd_bad)/np.sum(psd_clean):.2f}x")

print("\n" + "=" * 80)
print("DEFINITIVE ANALYSIS COMPLETE")
print("=" * 80)
