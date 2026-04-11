"""
Investigate the bimodal dt pattern: alternating 3.82ms and 4.22ms
and the key timing at sample 371625 (t=1486.5356s)
"""

import numpy as np
import pandas as pd

DATA_FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv"

print("=" * 80)
print("BIMODAL dt PATTERN & TRANSITION TIMING ANALYSIS")
print("=" * 80)

df = pd.read_csv(DATA_FILE)
timestamps = df.iloc[:, 0].values.astype(np.float64)
sample_numbers = df.iloc[:, 1].values.astype(np.int64)
channel_data = df.iloc[:, 2:].values.astype(np.float64)
col_names = df.columns[2:].tolist()

dt = np.diff(timestamps) * 1000.0  # ms

# ============================================================
# 1. BIMODAL dt CHARACTERIZATION
# ============================================================
print("\n1. BIMODAL dt CHARACTERIZATION")

# Histogram of dt values
hist, bin_edges = np.histogram(dt, bins=np.arange(3.5, 4.6, 0.01))
print(f"\n  dt histogram (0.01ms bins):")
for i in range(len(hist)):
    if hist[i] > 100:
        pct = hist[i] / len(dt) * 100
        print(f"    [{bin_edges[i]:.2f}, {bin_edges[i+1]:.2f}) ms: {hist[i]:6d} ({pct:.1f}%)")

# Two modes
mode1 = dt[dt < 4.0]
mode2 = dt[dt >= 4.0]
print(f"\n  Mode 1 (dt < 4.0ms): n={len(mode1)}, mean={np.mean(mode1):.4f}ms, std={np.std(mode1):.4f}ms")
print(f"  Mode 2 (dt >= 4.0ms): n={len(mode2)}, mean={np.mean(mode2):.4f}ms, std={np.std(mode2):.4f}ms")
print(f"  Mode1 + Mode2 = {np.mean(mode1) + np.mean(mode2):.4f}ms (expected: 8.0ms for perfect alternation)")

# Check alternation pattern
alt_violations = 0
for i in range(len(dt) - 1):
    if (dt[i] < 4.0 and dt[i+1] < 4.0) or (dt[i] >= 4.0 and dt[i+1] >= 4.0):
        alt_violations += 1
print(f"\n  Alternation violations (two same-mode in a row): {alt_violations}/{len(dt)-1} "
      f"({alt_violations/(len(dt)-1)*100:.2f}%)")

# ============================================================
# 2. dt PATTERN AROUND TRANSITION 1 (exact sample 371625)
# ============================================================
print(f"\n\n2. dt PATTERN AT EXACT TRANSITION ONSET (sample 371625)")

# Find the index of sample 371625
target_sn = 371625
target_idx = np.searchsorted(sample_numbers, target_sn)
print(f"  Sample {target_sn} at index {target_idx}, t={timestamps[target_idx]:.4f}s")

# Show dt values surrounding the transition
print(f"\n  dt values around the transition:")
for i in range(max(0, target_idx - 10), min(len(timestamps) - 1, target_idx + 15)):
    dt_val = (timestamps[i+1] - timestamps[i]) * 1000
    sn = sample_numbers[i]
    delta_mean = np.mean(channel_data[i+1, :]) - np.mean(channel_data[i, :])
    flag = ""
    if sn == target_sn:
        flag = " <-- TRANSITION ONSET"
    elif sn == target_sn + 1:
        flag = " <-- BIG STEP"
    print(f"  sn={sn} t={timestamps[i]:.4f}s dt={dt_val:.4f}ms  mean_delta={delta_mean:10.0f}{flag}")

# ============================================================
# 3. CHECK: Does the transition happen on a specific dt polarity?
# ============================================================
print(f"\n\n3. TRANSITION ON SHORT (3.82ms) OR LONG (4.22ms) dt?")

# The DRDY poll at 2500 Hz has 400us grid = alternating between
# catching the DRDY on the 9th or 10th poll cycle
# 3.82ms = below nominal -> DRDY caught early
# 4.22ms = above nominal -> DRDY caught late

dt_at_trans1 = (timestamps[target_idx + 1] - timestamps[target_idx]) * 1000
dt_before_trans1 = (timestamps[target_idx] - timestamps[target_idx - 1]) * 1000
print(f"  dt BEFORE transition onset: {dt_before_trans1:.4f}ms ({'short' if dt_before_trans1 < 4.0 else 'long'})")
print(f"  dt AT transition onset (sn {target_sn}->{target_sn+1}): {dt_at_trans1:.4f}ms ({'short' if dt_at_trans1 < 4.0 else 'long'})")

# ============================================================
# 4. STATS THREAD TIMING: Does the I2C read at t=1486.5 correlate?
# ============================================================
print(f"\n\n4. STATS THREAD I2C CONTENTION CHECK")

# Stats thread fires every 10s. If the engine started at time T,
# stats fires at T+10, T+20, T+30, ...
# The engine start time is timestamps[0] - start_time offset
# But we can check: does t=1486.54 align with a 10s boundary relative to any plausible start?

# The stats thread reads TCA9534 at 0x21 (START pin register) via I2C
# This CONTENDS with the DRDY poller's I2C read at 0x20
# The I2C device has a mutex -> one blocks the other

# Check: is there any dt anomaly at 10-second intervals?
print(f"  Recording starts at t={timestamps[0]:.3f}s")
print(f"  Transition at t=1486.536s")
print(f"  Offset from start: {1486.536 - timestamps[0]:.3f}s")
print(f"  Modulo 10s: {(1486.536 - timestamps[0]) % 10:.3f}s")

# Check if max-dt events cluster near 10s boundaries
# Stats thread reads I2C every ~10s
# This could add ~100-500us delay to DRDY poll if mutex contention occurs

# Find the 100 highest dt values
top_100_idx = np.argsort(dt)[-100:]
top_100_t = timestamps[1:][top_100_idx]
top_100_dt = dt[top_100_idx]

# Check their modulo-10 distribution
rel_times = top_100_t - timestamps[0]
mod10 = rel_times % 10
print(f"\n  Top 100 highest dt values - distribution modulo 10s:")
hist_mod10, bins_mod10 = np.histogram(mod10, bins=10, range=(0, 10))
for i in range(10):
    bar = "#" * hist_mod10[i]
    print(f"    {bins_mod10[i]:.0f}-{bins_mod10[i+1]:.0f}s: {hist_mod10[i]:3d} {bar}")

# ============================================================
# 5. THE BIG QUESTION: What happens between sn=371624 and 371625?
# ============================================================
print(f"\n\n5. SAMPLE 371624 -> 371625: THE TRANSITION SAMPLE")
print(f"   Every channel's delta at this exact sample")

# Compute per-channel delta at this sample
delta_at_trans = channel_data[target_idx + 1, :] - channel_data[target_idx, :]

# Sort by magnitude
sorted_idx = np.argsort(np.abs(delta_at_trans))[::-1]

print(f"\n  Top 30 channels by |delta| at transition sample:")
for rank in range(30):
    ci = sorted_idx[rank]
    print(f"    {col_names[ci]:25s}: delta={delta_at_trans[ci]:12.0f} LSB "
          f"(before={channel_data[target_idx, ci]:12.0f}, "
          f"after={channel_data[target_idx+1, ci]:12.0f})")

print(f"\n  Bottom 30 channels (smallest |delta|):")
for rank in range(30):
    ci = sorted_idx[-(rank+1)]
    print(f"    {col_names[ci]:25s}: delta={delta_at_trans[ci]:12.0f} LSB")

# Summary stats
print(f"\n  Delta at transition sample (all 336 channels):")
print(f"    Mean: {np.mean(delta_at_trans):.0f}")
print(f"    Median: {np.median(delta_at_trans):.0f}")
print(f"    Std: {np.std(delta_at_trans):.0f}")
print(f"    Min: {np.min(delta_at_trans):.0f}")
print(f"    Max: {np.max(delta_at_trans):.0f}")
print(f"    Channels with |delta| > 10000: {np.sum(np.abs(delta_at_trans) > 10000)}")
print(f"    Channels with |delta| > 50000: {np.sum(np.abs(delta_at_trans) > 50000)}")
print(f"    Channels with |delta| > 100000: {np.sum(np.abs(delta_at_trans) > 100000)}")

# ============================================================
# 6. PER-PORT SHIFT AT THE TRANSITION SAMPLE
# ============================================================
print(f"\n\n6. PER-PORT BEHAVIOR AT TRANSITION SAMPLE")

port_devs = [8, 7, 5, 5, 5, 5, 7]
port_names = ["Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"]
ch_offset = 0
for pi, (pname, ndev) in enumerate(zip(port_names, port_devs)):
    nch = ndev * 8
    port_delta = delta_at_trans[ch_offset:ch_offset + nch]
    port_before = channel_data[target_idx, ch_offset:ch_offset + nch]
    port_after = channel_data[target_idx + 1, ch_offset:ch_offset + nch]

    print(f"\n  {pname} ({ndev} devices, {nch} channels):")
    print(f"    Mean delta: {np.mean(port_delta):12.0f}")
    print(f"    Std delta:  {np.std(port_delta):12.0f}")
    print(f"    Max |delta|: {np.max(np.abs(port_delta)):12.0f}")
    print(f"    Channels |delta|>10K: {np.sum(np.abs(port_delta) > 10000)}/{nch}")

    ch_offset += nch

# ============================================================
# 7. NEXT SAMPLE (371625->371626): THE MAIN STEP
# ============================================================
print(f"\n\n7. SAMPLE 371625 -> 371626 (THE BIG STEP)")

delta_at_main = channel_data[target_idx + 2, :] - channel_data[target_idx + 1, :]

print(f"\n  Delta statistics (all 336 channels):")
print(f"    Mean: {np.mean(delta_at_main):.0f}")
print(f"    Median: {np.median(delta_at_main):.0f}")
print(f"    Std: {np.std(delta_at_main):.0f}")
print(f"    Channels with |delta| > 100000: {np.sum(np.abs(delta_at_main) > 100000)}")
print(f"    Channels with |delta| > 500000: {np.sum(np.abs(delta_at_main) > 500000)}")

# Per-port for the main step
ch_offset = 0
for pi, (pname, ndev) in enumerate(zip(port_names, port_devs)):
    nch = ndev * 8
    port_delta = delta_at_main[ch_offset:ch_offset + nch]
    print(f"    {pname}: mean delta = {np.mean(port_delta):12.0f}, max |delta| = {np.max(np.abs(port_delta)):12.0f}")
    ch_offset += nch

# ============================================================
# 8. EARLY CHANNELS: Port6_dev4 at t=1483
# ============================================================
print(f"\n\n8. EARLY ANOMALY: Port6_dev4 at t=1483s (3.5s BEFORE main transition)")

# Port6 starts at channel offset = (8+7+5+5+5)*8 = 240
# dev4 = channels 264-271
port6_dev4_offset = 240 + 3*8  # 264
for ch in range(8):
    ci = port6_dev4_offset + ch
    name = col_names[ci]
    # Find behavior at t=1483
    mask_1483 = (timestamps >= 1482.5) & (timestamps < 1484)
    vals = channel_data[mask_1483, ci]
    t_vals = timestamps[mask_1483]
    std_val = np.std(vals)
    max_delta = np.max(np.abs(np.diff(vals)))
    print(f"  {name}: std={std_val:.0f}, max_delta={max_delta:.0f}")

# ============================================================
# 9. RECOVERY: Is it the REVERSE of onset?
# ============================================================
print(f"\n\n9. RECOVERY COMPARISON (is recovery the reverse of onset?)")

# Onset: t=1486.54, gradual ramp ~5s
# Recovery: bulk of recovery at t=1601-1602
# Recovery onset is at ~1601.25 based on the earlier analysis

# Find recovery onset
mask_rec = (timestamps >= 1600.5) & (timestamps < 1603)
rec_gm = np.mean(channel_data[mask_rec], axis=1)
rec_t = timestamps[mask_rec]
rec_deltas = np.diff(rec_gm)

# Find the biggest positive jump
max_rec_idx = np.argmax(rec_deltas)
print(f"  Biggest recovery jump: t={rec_t[max_rec_idx]:.4f}s, delta={rec_deltas[max_rec_idx]:.0f}")

# Sample-by-sample around recovery onset
print(f"\n  Sample-by-sample grand mean around recovery:")
for i in range(max(0, max_rec_idx - 5), min(len(rec_gm), max_rec_idx + 10)):
    d = rec_gm[i] - rec_gm[i-1] if i > 0 else 0
    flag = " <-- MAX JUMP" if i == max_rec_idx + 1 else ""
    print(f"    t={rec_t[i]:.4f}s mean={rec_gm[i]:12.0f} delta={d:10.0f}{flag}")

# Compare: onset took ~5s to reach -1.5M shift
# Recovery: how long to recover 1.5M?
print(f"\n  Onset ramp: 0 -> -1.5M in ~5s ({1486.5 + 5:.1f}s)")
print(f"  Recovery: started at ~1601.25s, reached 50% by ~1601.5s, 92% by ~1602.0s")
print(f"  Recovery is ~5x FASTER than onset")

# ============================================================
# 10. CONTINUOUS DRIFT DURING BAD PERIOD
# ============================================================
print(f"\n\n10. IS THE BAD PERIOD STABLE OR STILL DRIFTING?")

# Grand mean in 10s windows during bad period
mask_bad_full = (timestamps >= 1490) & (timestamps < 1600)
bad_gm = np.mean(channel_data[mask_bad_full], axis=1)
bad_t = timestamps[mask_bad_full]

print(f"\n  Grand mean in 10s windows (bad period):")
for t in np.arange(1490, 1600, 10):
    w = (bad_t >= t) & (bad_t < t + 10)
    if np.sum(w) > 0:
        val = np.mean(bad_gm[w])
        std_val = np.std(bad_gm[w])
        print(f"    t={t:.0f}: mean={val:12.0f}  std={std_val:8.0f}")

print("\n" + "=" * 80)
print("ANALYSIS COMPLETE")
print("=" * 80)
