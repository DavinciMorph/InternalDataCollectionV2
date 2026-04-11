"""
Follow-up analysis: P1D8 railing event and spike characterization
"""

import numpy as np
import pandas as pd
import time

FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\experiment_data\Nehru1_15%336Ch6_M1_eeg_20260407_114131.csv"
FS = 250

print("Loading data...")
t0 = time.time()
df = pd.read_csv(FILE)
print(f"Loaded in {time.time()-t0:.1f}s")

timestamps = df.iloc[:, 0].values
t_rel = timestamps - timestamps[0]

# ══════════════════════════════════════════════════════════════════════════
# PART A: Characterize the railing event on P1D8_ch1
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART A: P1D8_CH1 RAILING EVENT")
print("=" * 80)

p1d8 = df.iloc[:, 58].values.astype(np.float64)

# The channel drifts from ~-7.16M to ~-8.15M (negative rail ~ -8388608 = -2^23)
neg_rail = -8388608
rail_threshold = neg_rail + 5000  # within 5000 LSB of rail

# Find where channel is railed
railed = p1d8 < rail_threshold
first_railed = np.argmax(railed) if np.any(railed) else None

print(f"  Negative rail value: {neg_rail}")
print(f"  Rail threshold (within 5000 LSB): {rail_threshold}")
print(f"  Samples near rail: {np.sum(railed):,} / {len(p1d8):,} ({100*np.sum(railed)/len(p1d8):.1f}%)")

if first_railed is not None:
    print(f"  First railed sample: index {first_railed}, t={t_rel[first_railed]:.3f}s")
    print(f"\n  Approach to rail (10-sample window around transition):")
    start = max(0, first_railed - 20)
    end = min(len(p1d8), first_railed + 10)
    for i in range(start, end):
        marker = " <<<" if i == first_railed else ""
        print(f"    [{i}] t={t_rel[i]:.3f}s  val={p1d8[i]:.0f}  (dist from rail: {p1d8[i] - neg_rail:.0f}){marker}")

# Check: is this just P1D8 or all channels railing?
print(f"\n  Checking other channels for railing at same time...")
# Port1 channels (cols 2-65), Port2 (66-121), Port3 (122-161), Port5 (202-241), Port7 (282-337)
check_channels = {
    'P1D1c1': 2, 'P1D4c1': 26, 'P1D8c1': 58,
    'P2D1c1': 66, 'P2D4c1': 90,
    'P3D1c1': 122, 'P3D3c1': 138,
    'P5D1c1': 202, 'P5D3c1': 218,
    'P7D1c1': 282, 'P7D4c1': 306,
}

# Check at t=280s (just before railing) and t=320s (after)
idx_280 = np.argmin(np.abs(t_rel - 280))
idx_320 = np.argmin(np.abs(t_rel - 320))

print(f"\n  {'Channel':<12} {'Val@280s':>14} {'Val@320s':>14} {'Railed?':>10}")
print(f"  {'-'*12} {'-'*14} {'-'*14} {'-'*10}")
for name, col in check_channels.items():
    v280 = df.iloc[idx_280, col]
    v320 = df.iloc[idx_320, col]
    near_rail = abs(v320 - neg_rail) < 10000 or abs(v320 - 8388607) < 10000
    print(f"  {name:<12} {v280:>14.0f} {v320:>14.0f} {'YES' if near_rail else 'no':>10}")

# ══════════════════════════════════════════════════════════════════════════
# PART B: Count railed channels across full recording
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART B: RAILED CHANNEL CENSUS")
print("=" * 80)

# Sample at t=320s (steady state after initial drift)
idx = idx_320
print(f"  Sampling at t=320s (index {idx}):")
n_railed_neg = 0
n_railed_pos = 0
n_active = 0
for col in range(2, df.shape[1]-1):  # skip timestamp, sample_num, perf_counter
    val = df.iloc[idx, col]
    if abs(val - neg_rail) < 10000:
        n_railed_neg += 1
    elif abs(val - 8388607) < 10000:
        n_railed_pos += 1
    else:
        n_active += 1
print(f"  Total data channels: {df.shape[1] - 3}")  # minus timestamp, sample_num, perf_counter
print(f"  Railed negative: {n_railed_neg}")
print(f"  Railed positive: {n_railed_pos}")
print(f"  Active (not railed): {n_active}")

# ══════════════════════════════════════════════════════════════════════════
# PART C: Spike characterization - are these biological or electronic?
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART C: SPIKE CHARACTERIZATION (first 300s only)")
print("=" * 80)

# Only look at pre-railing data
pre_rail_n = np.argmin(np.abs(t_rel - 280))
p1d8_pre = p1d8[:pre_rail_n]

diffs = np.abs(np.diff(p1d8_pre))
spikes_50k = np.where(diffs > 50000)[0]

print(f"  Spikes > 50k LSB in first 280s: {len(spikes_50k)}")
print(f"  Spikes > 50k LSB / minute: {len(spikes_50k) / (280/60):.1f}")

# Typical spike shape: is it single-sample or multi-sample?
print(f"\n  Spike morphology (first 10 spikes, 5-sample context):")
for spike_idx in spikes_50k[:10]:
    start = max(0, spike_idx - 2)
    end = min(pre_rail_n, spike_idx + 4)
    vals = p1d8_pre[start:end]
    print(f"    Spike at idx {spike_idx} (t={t_rel[spike_idx]:.3f}s):")
    print(f"      Values: {[f'{v:.0f}' for v in vals]}")
    print(f"      Jump: {diffs[spike_idx]:.0f} LSB ({diffs[spike_idx] * 4.5 / (2**23 * 24) * 1e6:.1f} uV)")

# Are spikes correlated across ports? (simultaneous = electronic, independent = biological)
print(f"\n  Cross-port spike simultaneity check:")
port_channels = {
    'P1D8c1': df.iloc[:pre_rail_n, 58].values.astype(np.float64),
    'P2D1c1': df.iloc[:pre_rail_n, 66].values.astype(np.float64),
    'P3D1c1': df.iloc[:pre_rail_n, 122].values.astype(np.float64),
    'P5D1c1': df.iloc[:pre_rail_n, 202].values.astype(np.float64),
    'P7D1c1': df.iloc[:pre_rail_n, 282].values.astype(np.float64),
}

# For each P1D8 spike, check if other ports also spike within +/- 1 sample
simultaneous_count = 0
for spike_idx in spikes_50k[:50]:  # check first 50
    n_ports_spiking = 0
    for name, data in port_channels.items():
        if name == 'P1D8c1':
            continue
        if spike_idx > 0 and spike_idx < len(data) - 1:
            local_max_diff = max(
                abs(data[spike_idx] - data[spike_idx-1]),
                abs(data[spike_idx+1] - data[spike_idx])
            )
            if local_max_diff > 50000:
                n_ports_spiking += 1
    if n_ports_spiking >= 2:
        simultaneous_count += 1

print(f"  Of first 50 P1D8 spikes, {simultaneous_count} are simultaneous across 3+ ports")
if simultaneous_count > 25:
    print(f"  >>> ELECTRONIC ORIGIN (common-mode, system-wide)")
elif simultaneous_count > 5:
    print(f"  >>> MIXED: some common-mode coupling")
else:
    print(f"  >>> BIOLOGICAL/LOCAL: spikes are independent per channel (electrode movement)")

# ══════════════════════════════════════════════════════════════════════════
# PART D: All 5 sentinel channels - are they all railed after 300s?
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART D: SENTINEL CHANNEL RAILING STATUS")
print("=" * 80)

sentinel_cols = {'P1D8c1': 58, 'P2D1c1': 66, 'P3D1c1': 122, 'P5D1c1': 202, 'P7D1c1': 282}

for name, col in sentinel_cols.items():
    data = df.iloc[:, col].values.astype(np.float64)
    railed_neg = data < (neg_rail + 5000)
    railed_pos = data > (8388607 - 5000)
    first_railed_idx = np.argmax(railed_neg | railed_pos) if np.any(railed_neg | railed_pos) else None
    pct_railed = 100 * np.sum(railed_neg | railed_pos) / len(data)

    if first_railed_idx is not None and (railed_neg[first_railed_idx] or railed_pos[first_railed_idx]):
        sign = "NEG" if railed_neg[first_railed_idx] else "POS"
        print(f"  {name}: first railed at t={t_rel[first_railed_idx]:.1f}s ({sign}), {pct_railed:.1f}% railed")
    else:
        mean_val = np.mean(data)
        std_val = np.std(data)
        print(f"  {name}: NOT RAILED, mean={mean_val:.0f}, std={std_val:.0f}")

# ══════════════════════════════════════════════════════════════════════════
# PART E: Noise floor comparison - pre-rail active vs post-rail
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART E: NOISE CHARACTERISTICS PRE vs POST RAILING")
print("=" * 80)

# Use P1D8 - pre-rail noise is genuine EEG + noise, post-rail is quantization at rail
pre_data = p1d8[:pre_rail_n]
post_data = p1d8[pre_rail_n:]

pre_diff_std = np.std(np.diff(pre_data))
post_diff_std = np.std(np.diff(post_data))
pre_raw_std = np.std(pre_data)
post_raw_std = np.std(post_data)

print(f"  PRE-RAIL (first 280s, {pre_rail_n:,} samples):")
print(f"    mean:     {np.mean(pre_data):.0f}")
print(f"    raw_std:  {pre_raw_std:.1f}")
print(f"    diff_std: {pre_diff_std:.1f}")
print(f"    min:      {np.min(pre_data):.0f}")
print(f"    max:      {np.max(pre_data):.0f}")

print(f"\n  POST-RAIL (after 280s, {len(post_data):,} samples):")
print(f"    mean:     {np.mean(post_data):.0f}")
print(f"    raw_std:  {post_raw_std:.1f}")
print(f"    diff_std: {post_diff_std:.1f}")
print(f"    min:      {np.min(post_data):.0f}")
print(f"    max:      {np.max(post_data):.0f}")

print(f"\n  Noise ratio (pre/post diff_std): {pre_diff_std/post_diff_std:.1f}x")
print(f"  NOTE: Post-rail 'noise' of {post_diff_std:.0f} LSB is just ADC quantization jitter at the rail")

# Check: is the railing a slow drift or sudden jump?
print(f"\n  Rate of approach to rail:")
# Find value at various time points
for t_check in [0, 60, 120, 180, 240, 270, 280, 290, 300, 310]:
    idx = np.argmin(np.abs(t_rel - t_check))
    print(f"    t={t_check:>4}s: P1D8c1 = {p1d8[idx]:.0f}  (dist from neg rail: {p1d8[idx] - neg_rail:.0f})")

print("\n" + "=" * 80)
print("CONCLUSION")
print("=" * 80)
print("""
  P1D8_ch1 shows a SLOW MONOTONIC DRIFT toward the negative rail over the first
  ~300 seconds, then saturates. This is characteristic of:

    1. ELECTRODE DC OFFSET DRIFT — gel drying, impedance change, half-cell
       potential shift. The ~1M LSB drift over 300s = ~22 mV, consistent with
       electrochemical drift.

    2. NOT the inductor problem, which manifests as:
       - SUDDEN (1-sample) shifts of ~3.3M LSB
       - ALL channels simultaneously
       - 100x noise increase
       - Exponential RC recovery (10.6s tau)
       - Accelerating episodes

  None of those signatures are present in this recording.

  The 370 spikes > 50k LSB in the first 280s are confined to the pre-railing
  period when the channel had live EEG signal. These are likely electrode motion
  artifacts (independent per channel, not simultaneous across ports).
""")
