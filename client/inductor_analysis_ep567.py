"""
Closer look at Episodes 5-7 (t=1294-1326s) where ALL ports shifted together.
Are these the inductor signature, or something else?
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\experiment_data\Nehru1_15%336Ch4_M1_eeg_20260407_102850.csv"
OUT_DIR = Path(r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\inductor_analysis_output")

FS = 250

PROBES = {
    'P1D8c1': 58,
    'P1D1c1': 2,
    'P2D1c1': 66,
    'P3D1c1': 122,
    'P5D1c1': 202,
    'P7D1c1': 282,
}

print("Loading relevant columns...")
raw_header = pd.read_csv(FILE, nrows=0)
col_names = raw_header.columns.tolist()
col_indices = sorted(set([0, 1] + list(PROBES.values())))
df = pd.read_csv(FILE, usecols=col_indices)

timestamps = df.iloc[:, 0].values
t_rel = timestamps - timestamps[0]
N = len(timestamps)

ch = {}
for name, cidx in PROBES.items():
    cname = col_names[cidx]
    ch[name] = df[cname].values.astype(np.float64)

# Focus on t=1280-1340s (Episodes 5-7)
mask = (t_rel >= 1280) & (t_rel <= 1340)
idx_range = np.where(mask)[0]
print(f"Focusing on t=1280-1340s: {len(idx_range)} samples")

# Check if these are true simultaneous shifts
print("\n" + "=" * 80)
print("EPISODES 5-7 DETAIL")
print("=" * 80)

# Find all jumps > 40k in this region on any channel
for name in PROBES:
    d = np.abs(np.diff(ch[name][idx_range]))
    big = np.where(d > 40_000)[0]
    if len(big) > 0:
        print(f"\n  {name}: {len(big)} jumps > 40k LSB")
        for b in big[:10]:
            actual_idx = idx_range[b]
            t = t_rel[actual_idx]
            jump = ch[name][actual_idx+1] - ch[name][actual_idx]
            print(f"    t={t:.3f}s: {jump:+,.0f} LSB (before={ch[name][actual_idx]:,.0f}, after={ch[name][actual_idx+1]:,.0f})")

# Are the jumps synchronous across channels? Check sample-by-sample
print("\n\nSynchronous jump check:")
print("Looking for samples where ALL probes jump > 30k LSB simultaneously...")
all_diffs = {}
for name in PROBES:
    all_diffs[name] = np.abs(np.diff(ch[name][idx_range]))

# Find samples where multiple channels jump
for threshold in [30_000, 50_000]:
    print(f"\n  Threshold: {threshold:,} LSB")
    for i in range(len(idx_range)-1):
        channels_jumping = []
        for name in PROBES:
            if all_diffs[name][i] > threshold:
                channels_jumping.append(name)
        if len(channels_jumping) >= 3:
            actual_idx = idx_range[i]
            t = t_rel[actual_idx]
            print(f"    t={t:.3f}s: {len(channels_jumping)} channels jumping: {channels_jumping}")
            for name in PROBES:
                jump = ch[name][actual_idx+1] - ch[name][actual_idx]
                print(f"      {name}: {jump:+,.0f} LSB")

# Magnitude comparison: what's the biggest shift in this region?
print("\n\nPeak shifts in the t=1280-1340s region:")
for name in PROBES:
    segment = ch[name][idx_range]
    peak_to_peak = np.max(segment) - np.min(segment)
    max_jump = np.max(np.abs(np.diff(segment)))
    print(f"  {name}: p2p={peak_to_peak:,.0f}, max_jump={max_jump:,.0f}")

# Key question: magnitude. Inductor problem = ~3.3M LSB. These are ~60-100k.
print("\n\nMAGNITUDE COMPARISON:")
print(f"  Inductor problem signature: ~3,300,000 LSB simultaneous shift")
print(f"  Episodes 5-7 max shift:     ~60,000-100,000 LSB")
print(f"  Ratio: {3_300_000 / 100_000:.0f}x smaller than inductor signature")
print(f"\n  These are likely motion artifacts (physical electrode movement)")
print(f"  affecting all electrodes simultaneously -- consistent with head movement.")

# Plot the region
fig, axes = plt.subplots(len(PROBES), 1, figsize=(16, 12), sharex=True)
for i, (name, _) in enumerate(PROBES.items()):
    axes[i].plot(t_rel[idx_range], ch[name][idx_range], linewidth=0.6)
    axes[i].set_ylabel(f'{name}\n(LSB)', fontsize=8)
    axes[i].grid(True, alpha=0.3)
axes[0].set_title('Episodes 5-7 (t=1280-1340s) -- All Probe Channels')
axes[-1].set_xlabel('Time (seconds)')
plt.tight_layout()
plt.savefig(OUT_DIR / 'episodes_567_detail.png', dpi=150)
plt.close()
print(f"\nSaved: episodes_567_detail.png")
