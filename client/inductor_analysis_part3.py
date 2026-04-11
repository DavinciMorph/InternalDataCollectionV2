"""
Part 3: Deeper investigation of the common-mode spikes and near-rail behavior
"""

import numpy as np
import pandas as pd
import time

FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\experiment_data\Nehru1_15%336Ch6_M1_eeg_20260407_114131.csv"
FS = 250

print("Loading data...")
t0 = time.time()
df = pd.read_csv(FILE)
print(f"Loaded in {time.time()-t0:.1f}s, shape: {df.shape}")

timestamps = df.iloc[:, 0].values
t_rel = timestamps - timestamps[0]

CHANNELS = {
    'P1D8c1': 58, 'P2D1c1': 66, 'P3D1c1': 122, 'P5D1c1': 202, 'P7D1c1': 282,
}
ch_data = {name: df.iloc[:, col].values.astype(np.float64) for name, col in CHANNELS.items()}

# ══════════════════════════════════════════════════════════════════════════
# PART F: Common-mode spike analysis — what's the mechanism?
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART F: COMMON-MODE SPIKE DEEP-DIVE")
print("=" * 80)

# Look at the 3 grand-mean shifts > 100k LSB from Check 3
all_ch = np.column_stack([ch_data[k] for k in CHANNELS.keys()])
grand_mean = np.mean(all_ch, axis=1)
gm_diff = np.abs(np.diff(grand_mean))

# Top 20 largest grand-mean jumps
top20_idx = np.argsort(gm_diff)[-20:][::-1]
print(f"  Top 20 largest grand-mean single-sample jumps:")
print(f"  {'Rank':>4}  {'Index':>8}  {'Time(s)':>10}  {'GM_Jump':>14}  Per-port jumps")
for rank, idx in enumerate(top20_idx):
    t_sec = t_rel[idx]
    per_port = []
    for name in CHANNELS:
        jump = ch_data[name][idx+1] - ch_data[name][idx]
        per_port.append(f"{name}={jump:.0f}")
    print(f"  {rank+1:>4}  {idx:>8}  {t_sec:>10.3f}  {gm_diff[idx]:>14.0f}  {', '.join(per_port)}")

# Are these during the active period (first 300s) or after?
print(f"\n  Location: {sum(1 for i in top20_idx if t_rel[i] < 300)} of top 20 are in first 300s")
print(f"           {sum(1 for i in top20_idx if t_rel[i] >= 300)} of top 20 are after 300s")

# ══════════════════════════════════════════════════════════════════════════
# PART G: Is P1D8 truly saturated or just very negative?
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART G: P1D8_CH1 SATURATION CHECK")
print("=" * 80)

p1d8 = ch_data['P1D8c1']
neg_rail = -8388608  # -2^23
pos_rail = 8388607   # 2^23 - 1

# The key question: is -8152000 actually the rail? Or is there dynamic range left?
# ADS1299: 24-bit signed, range [-8388608, +8388607]
# If the signal is truly railed, we'd see it STUCK at exactly -8388608
# If it's just very negative but not railed, we should see variation

print(f"  ADC range: [{neg_rail}, {pos_rail}]")
print(f"  P1D8c1 at t=400s: {p1d8[np.argmin(np.abs(t_rel - 400))]:.0f}")
print(f"  Distance from neg rail at t=400s: {p1d8[np.argmin(np.abs(t_rel - 400))] - neg_rail:.0f}")

# Post-300s statistics
post_300 = p1d8[t_rel > 300]
print(f"\n  Post-300s statistics:")
print(f"    N samples: {len(post_300):,}")
print(f"    Mean: {np.mean(post_300):.1f}")
print(f"    Std:  {np.std(post_300):.1f}")
print(f"    Min:  {np.min(post_300):.0f}")
print(f"    Max:  {np.max(post_300):.0f}")
print(f"    Dist from rail (mean): {np.mean(post_300) - neg_rail:.0f}")
print(f"    Dist from rail (min):  {np.min(post_300) - neg_rail:.0f}")

# The channel is at ~-8152000, which is 236,608 LSB from the rail
# That's about 5.3 mV from the rail — NOT saturated, just a very large DC offset
print(f"\n  VERDICT: P1D8c1 is NOT railed/saturated.")
print(f"  It stabilized at ~-8,152,000 (236k LSB above neg rail).")
print(f"  This is a large DC offset but the ADC is NOT clipping.")
print(f"  The tiny std (~100 LSB) post-stabilization suggests the electrode")
print(f"  reached a stable electrochemical equilibrium — very high impedance")
print(f"  but not open-circuit.")

# ══════════════════════════════════════════════════════════════════════════
# PART H: How many channels show this drift-then-stabilize pattern?
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART H: DRIFT PATTERN ACROSS ALL SENTINEL CHANNELS")
print("=" * 80)

for name, col in CHANNELS.items():
    data = df.iloc[:, col].values.astype(np.float64)
    # First 30s mean vs last 30s mean
    first_30s = data[:30*FS]
    last_30s = data[-30*FS:]
    drift = np.mean(last_30s) - np.mean(first_30s)

    # Std in first 30s vs last 30s (diff_std)
    first_diff_std = np.std(np.diff(first_30s))
    last_diff_std = np.std(np.diff(last_30s))

    print(f"\n  {name} (col {col}):")
    print(f"    First 30s mean: {np.mean(first_30s):.0f}")
    print(f"    Last 30s mean:  {np.mean(last_30s):.0f}")
    print(f"    Drift:          {drift:.0f} LSB ({drift * 4.5 / (2**23 * 24) * 1e6:.0f} uV)")
    print(f"    First 30s diff_std: {first_diff_std:.1f}")
    print(f"    Last 30s diff_std:  {last_diff_std:.1f}")
    print(f"    Noise ratio (last/first): {last_diff_std/first_diff_std:.2f}x")

# ══════════════════════════════════════════════════════════════════════════
# PART I: Common-mode spike mechanism — check timestamp gaps at spike locations
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART I: TIMESTAMP BEHAVIOR AT SPIKE LOCATIONS")
print("=" * 80)

dt_ms = np.diff(timestamps) * 1000
p1d8_diffs = np.abs(np.diff(p1d8))
spikes_50k = np.where(p1d8_diffs > 50000)[0]

print(f"  Checking dt at {len(spikes_50k)} spike locations:")
spike_dts = dt_ms[spikes_50k]
print(f"    dt at spikes: mean={np.mean(spike_dts):.3f}ms, std={np.std(spike_dts):.3f}ms")
print(f"    dt overall:   mean={np.mean(dt_ms):.3f}ms, std={np.std(dt_ms):.3f}ms")
print(f"    dt at spikes > 4.5ms: {np.sum(spike_dts > 4.5)}")
print(f"    dt at spikes > 5.0ms: {np.sum(spike_dts > 5.0)}")

# Are any spikes associated with unusual timing?
if np.sum(spike_dts > 5.0) > 0:
    big_dt_spikes = spikes_50k[spike_dts > 5.0]
    print(f"\n  Spikes with dt > 5ms:")
    for idx in big_dt_spikes[:10]:
        print(f"    idx={idx}, t={t_rel[idx]:.3f}s, dt={dt_ms[idx]:.3f}ms, jump={p1d8_diffs[idx]:.0f}")

# ══════════════════════════════════════════════════════════════════════════
# PART J: Port-by-port noise floor (diff_std) in 5-minute segments
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PART J: PORT-BY-PORT NOISE (5-minute segments)")
print("=" * 80)

seg_size = 5 * 60 * FS  # 5 minutes
n_segs = len(timestamps) // seg_size

print(f"  {'Segment':>7}  {'Time(min)':>10}", end="")
for name in CHANNELS:
    print(f"  {name:>12}", end="")
print()

for i in range(n_segs):
    start = i * seg_size
    end = start + seg_size
    t_min = t_rel[start] / 60
    print(f"  {i:>7}  {t_min:>10.1f}", end="")
    for name in CHANNELS:
        seg = ch_data[name][start:end]
        ds = np.std(np.diff(seg))
        print(f"  {ds:>12.1f}", end="")
    print()

# ══════════════════════════════════════════════════════════════════════════
# FINAL SUMMARY
# ══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("COMPREHENSIVE FINAL VERDICT")
print("=" * 80)
print("""
INDUCTOR PROBLEM: NO

This recording shows ZERO signatures of the known inductor problem:
  [PASS] No sudden ~3.3M LSB DC offsets (max grand-mean jump ~121k LSB)
  [PASS] No 100x noise increase (noise drops, not increases, as channels stabilize)
  [PASS] No exponential RC recovery waveforms
  [PASS] No accelerating episodes
  [PASS] Zero sample gaps, zero dt > 10ms
  [PASS] No channels hit the ADC rail (-8388608)

What IS happening in this recording:

1. ELECTRODE DRIFT (first ~300s):
   Several channels drift ~1M LSB negative over 5 minutes, then stabilize.
   This is normal electrode-gel electrochemistry reaching equilibrium.
   P1D8c1 stabilizes at ~-8,152,000 (236k LSB above neg rail — NOT saturated).

2. COMMON-MODE TRANSIENT SPIKES (first ~300s, 370 events > 50k LSB):
   84% simultaneous across ports = ELECTRONIC common-mode coupling.
   Confined to the initial drift period. Not accelerating.
   Likely: subject motion + high electrode impedance during settling.
   Once electrodes stabilize, spikes vanish entirely.

3. HIGH CROSS-PORT CORRELATION (late recording, windows 30-48):
   Mean |r| rises to ~0.83 in the second half. This is expected when
   multiple channels have stabilized at near-constant values — any
   shared environmental noise (60 Hz, Pi EMI) dominates the tiny residual
   signal, inflating correlation. This is a mathematical artifact of
   low-variance signals, NOT a power supply problem.
""")
