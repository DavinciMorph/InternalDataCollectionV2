"""
Firmware Issue 1 — Task A.5: Recovery Dynamics (refined)
========================================================
The initial 3-sigma recovery detection failed because the "clean" period at t=1607
may not fully return to the original baseline. This script:
  1. Profiles the actual data in the t=1600-1650 region to understand what's happening
  2. Uses derivative-based transition detection instead of absolute z-score thresholds
  3. Detects both the recovery (~1607) and re-onset (~1640) transitions
"""

import numpy as np
import pandas as pd
import time

CSV_PATH = "C:/Users/cadav/Desktop/InternalDataCollection/CodeTesting/client/FirmwareIssue1.csv"

REP_CHANNELS = {
    'Port1': 'Port1_dev1_ch1',
    'Port2': 'Port2_dev1_ch1',
    'Port3': 'Port3_dev1_ch1',
    'Port4': 'Port4_dev1_ch4',
    'Port5': 'Port5_dev1_ch1',
    'Port6': 'Port6_dev1_ch1',
    'Port7': 'Port7_dev1_ch1',
}

SPI_BUS_MAP = {
    'Port1': 'SPI0', 'Port2': 'SPI0',
    'Port3': 'SPI3', 'Port4': 'SPI3',
    'Port5': 'SPI4', 'Port6': 'SPI4',
    'Port7': 'SPI5',
}

# Load data
print("Loading CSV...")
cols_to_load = ['timestamp', 'sample_number'] + list(REP_CHANNELS.values())
df = pd.read_csv(CSV_PATH, usecols=cols_to_load)
df['timestamp'] = df['timestamp'].astype(np.float64)
df['sample_number'] = df['sample_number'].astype(np.int64)
for ch in REP_CHANNELS.values():
    df[ch] = df[ch].astype(np.int64)

# ─── Compute baselines ───
clean_mask = (df['timestamp'] >= 1400.0) & (df['timestamp'] <= 1470.0)
bad_mask = (df['timestamp'] >= 1500.0) & (df['timestamp'] <= 1590.0)

clean_stats = {}
bad_stats = {}
for port, ch in REP_CHANNELS.items():
    clean_vals = df.loc[clean_mask, ch].values
    bad_vals = df.loc[bad_mask, ch].values
    clean_stats[port] = {'mean': np.mean(clean_vals), 'std': np.std(clean_vals)}
    bad_stats[port] = {'mean': np.mean(bad_vals), 'std': np.std(bad_vals)}


# ═══════════════════════════════════════════════════════════════════════
#  STEP 1: Profile the t=1595-1650 region to understand the recovery
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 1: PROFILING t=1595-1650s — What does the 'recovery' actually look like?")
print("="*80)

# Sample mean values in 2-second windows
for t_start in np.arange(1595, 1650, 2):
    t_end = t_start + 2
    mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
    n = mask.sum()
    if n == 0:
        continue

    z_vals = {}
    for port, ch in REP_CHANNELS.items():
        vals = df.loc[mask, ch].values
        z_clean = (np.mean(vals) - clean_stats[port]['mean']) / clean_stats[port]['std']
        z_bad = (np.mean(vals) - bad_stats[port]['mean']) / bad_stats[port]['std']
        z_vals[port] = (z_clean, z_bad)

    mean_z_clean = np.mean([v[0] for v in z_vals.values()])
    mean_z_bad = np.mean([v[1] for v in z_vals.values()])
    print(f"t={t_start:.0f}-{t_end:.0f}s (n={n}): mean z(clean)={mean_z_clean:>+7.1f}, mean z(bad)={mean_z_bad:>+7.1f}")


# ═══════════════════════════════════════════════════════════════════════
#  STEP 2: Use the absolute value difference from clean mean to detect transitions
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 2: TRANSITION DETECTION — Absolute deviation from clean baseline")
print("="*80)

# For each port, compute |value - clean_mean| and find where it drops below
# a threshold that indicates return toward clean. We'll use a rolling mean
# to smooth out noise.

# First, let's look at the actual per-sample z-scores in a narrow window around
# the claimed recovery time
print("\n--- Per-sample z(clean) at t=1604-1612s for all ports ---")
narrow_mask = (df['timestamp'] >= 1604.0) & (df['timestamp'] <= 1612.0)
narrow_df = df[narrow_mask].copy()

# Compute z-scores for the narrow window
for port, ch in REP_CHANNELS.items():
    narrow_df[f'z_{port}'] = (narrow_df[ch] - clean_stats[port]['mean']) / clean_stats[port]['std']

# Print every 50th sample to see the trend
print(f"\n{'sn':>10} {'timestamp':>10}", end="")
for port in REP_CHANNELS:
    print(f" {port:>8}", end="")
print()
print("-" * (22 + 9*7))

for i, (_, row) in enumerate(narrow_df.iterrows()):
    if i % 50 == 0:
        print(f"{int(row['sample_number']):>10d} {row['timestamp']:>10.3f}", end="")
        for port in REP_CHANNELS:
            print(f" {row[f'z_{port}']:>+8.1f}", end="")
        print()

# ═══════════════════════════════════════════════════════════════════════
#  STEP 3: Derivative-based transition detection
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 3: DERIVATIVE-BASED RECOVERY DETECTION")
print("="*80)

# The recovery is a large positive jump (values going from more-negative back toward
# less-negative). The onset was a large negative jump.
# Strategy: compute sample-to-sample difference, find the first sample where
# the diff exceeds N*clean_std (positive direction = recovery)

search_mask = (df['timestamp'] >= 1600.0) & (df['timestamp'] <= 1615.0)
search_df = df[search_mask].copy()
print(f"\nSearching t=1600-1615s ({search_mask.sum()} samples)")
print("Detecting recovery as first sample where diff > 10*clean_std (positive direction)")

recovery_results = {}
print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Timestamp':>12} {'Diff':>12} {'Diff/std':>10}")
print("-" * 62)

for port, ch in REP_CHANNELS.items():
    vals = search_df[ch].values
    diffs = np.diff(vals)  # diff[i] = vals[i+1] - vals[i]
    std_ = clean_stats[port]['std']

    # Recovery = large positive diff (values jumping back toward clean)
    thresh = 10 * std_
    exceed_idx = np.where(diffs > thresh)[0]

    if len(exceed_idx) > 0:
        # The transition sample is i+1 (the one that jumped)
        first_idx = exceed_idx[0] + 1  # +1 because diff[i] = vals[i+1]-vals[i], transition is at i+1
        global_idx = search_df.index[first_idx]
        sn = int(df.loc[global_idx, 'sample_number'])
        ts = df.loc[global_idx, 'timestamp']
        diff_val = int(diffs[exceed_idx[0]])
        diff_sigma = diff_val / std_
        recovery_results[port] = {'sample_number': sn, 'timestamp': ts, 'diff': diff_val, 'diff_sigma': diff_sigma}
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {sn:>10d} {ts:>12.3f} {diff_val:>+12d} {diff_sigma:>+10.1f}")
    else:
        recovery_results[port] = None
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {'NOT FOUND':>10}")

# Recovery offsets
print("\n--- Recovery Port-by-Port Offsets ---")
valid_recov = {p: r for p, r in recovery_results.items() if r is not None}
if len(valid_recov) >= 2:
    earliest_port = min(valid_recov, key=lambda p: valid_recov[p]['sample_number'])
    latest_port = max(valid_recov, key=lambda p: valid_recov[p]['sample_number'])
    earliest_sn = valid_recov[earliest_port]['sample_number']
    latest_sn = valid_recov[latest_port]['sample_number']

    print(f"\nEarliest recovery: {earliest_port} at sample {earliest_sn} (t={valid_recov[earliest_port]['timestamp']:.3f}s)")
    print(f"Latest recovery:   {latest_port} at sample {latest_sn} (t={valid_recov[latest_port]['timestamp']:.3f}s)")
    print(f"Total spread:      {latest_sn - earliest_sn} samples ({(latest_sn - earliest_sn)*4:.0f} ms)")

    print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Offset from earliest':>22} {'ms offset':>10}")
    print("-" * 62)
    for port in sorted(valid_recov.keys()):
        r = valid_recov[port]
        offset = r['sample_number'] - earliest_sn
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {r['sample_number']:>10d} {offset:>+22d} {offset*4:>+10.0f}")

    # SPI bus grouping
    print("\n--- Recovery SPI Bus Grouping ---")
    bus_groups = {}
    for port, r in valid_recov.items():
        bus = SPI_BUS_MAP[port]
        if bus not in bus_groups:
            bus_groups[bus] = []
        bus_groups[bus].append((port, r['sample_number']))

    for bus in sorted(bus_groups.keys()):
        ports = bus_groups[bus]
        sns = [sn for _, sn in ports]
        spread = max(sns) - min(sns)
        port_list = ', '.join([f"{p}(sn={sn})" for p, sn in ports])
        print(f"{bus}: {port_list} -> spread = {spread} samples ({spread*4}ms)")


# ═══════════════════════════════════════════════════════════════════════
#  STEP 4: Sample-by-sample detail around recovery
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 4: SAMPLE-BY-SAMPLE RECOVERY DETAIL")
print("="*80)

for port, ch in REP_CHANNELS.items():
    if recovery_results[port] is None:
        continue
    recov_sn = recovery_results[port]['sample_number']
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']

    mask = (df['sample_number'] >= recov_sn - 5) & (df['sample_number'] <= recov_sn + 5)
    local = df[mask]

    print(f"\n  {port} ({ch}), clean mean={clean_mean:.0f}, clean std={clean_std:.0f}:")
    prev_val = None
    for _, row in local.iterrows():
        val = int(row[ch])
        z = (val - clean_mean) / clean_std
        diff_str = ""
        if prev_val is not None:
            diff = val - prev_val
            diff_str = f"  diff={diff:>+10d} ({diff/clean_std:>+6.1f}s)"
        prev_val = val
        marker = " <<<" if row['sample_number'] == recov_sn else ""
        print(f"    sn={int(row['sample_number']):>8d}  t={row['timestamp']:>10.3f}  val={val:>10d}  z={z:>8.1f}{diff_str}{marker}")


# ═══════════════════════════════════════════════════════════════════════
#  STEP 5: Recovery step characterization
# ═══════════════════════════════════════════════════════════════════════
print("\n--- Recovery Step Characterization ---")
for port, ch in REP_CHANNELS.items():
    if recovery_results[port] is None:
        continue
    recov_sn = recovery_results[port]['sample_number']
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']

    mask = (df['sample_number'] >= recov_sn - 2) & (df['sample_number'] <= recov_sn + 2)
    local = df[mask].copy()

    pre_vals = local[local['sample_number'] < recov_sn][ch].values
    post_vals = local[local['sample_number'] >= recov_sn][ch].values

    if len(pre_vals) > 0 and len(post_vals) > 0:
        jump = post_vals[0] - pre_vals[-1]
        print(f"  {port}: pre={pre_vals[-1]:>10d}, recovery={post_vals[0]:>10d}, jump={jump:>+10d} LSB ({jump/clean_std:>+.1f} sigma)")


# ═══════════════════════════════════════════════════════════════════════
#  STEP 6: Onset vs Recovery comparison
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 6: ONSET vs RECOVERY vs RE-ONSET — TIMING COMPARISON")
print("="*80)

# Onset data from previous analysis
onset_results = {
    'Port1': {'sample_number': 371626, 'timestamp': 1486.539},
    'Port2': {'sample_number': 371625, 'timestamp': 1486.536},
    'Port3': {'sample_number': 371626, 'timestamp': 1486.539},
    'Port4': {'sample_number': 371626, 'timestamp': 1486.539},
    'Port5': {'sample_number': 371625, 'timestamp': 1486.536},
    'Port6': {'sample_number': 371626, 'timestamp': 1486.539},
    'Port7': {'sample_number': 371626, 'timestamp': 1486.539},
}

# Re-onset data from previous analysis
reonset_results = {
    'Port1': {'sample_number': 410476, 'timestamp': 1641.943},
    'Port2': {'sample_number': 410476, 'timestamp': 1641.943},
    'Port3': {'sample_number': 410477, 'timestamp': 1641.947},
    'Port4': {'sample_number': 410476, 'timestamp': 1641.943},
    'Port5': {'sample_number': 410476, 'timestamp': 1641.943},
    'Port6': {'sample_number': 410476, 'timestamp': 1641.943},
    'Port7': {'sample_number': 410476, 'timestamp': 1641.943},
}

print(f"\n{'Port':<8} {'SPI Bus':<8} {'Onset sn':>10} {'Recovery sn':>12} {'Re-onset sn':>12} {'Bad dur (s)':>12} {'Clean dur (s)':>14}")
print("-" * 80)
for port in REP_CHANNELS:
    o_sn = onset_results[port]['sample_number']
    r = recovery_results.get(port)
    r_sn = r['sample_number'] if r else None
    ro_sn = reonset_results[port]['sample_number']

    r_str = f"{r_sn:>12d}" if r_sn else f"{'N/A':>12}"
    bad_dur = f"{(r_sn - o_sn)/250:>12.1f}" if r_sn else f"{'N/A':>12}"
    clean_dur = f"{(ro_sn - r_sn)/250:>14.1f}" if r_sn else f"{'N/A':>14}"

    print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {o_sn:>10d} {r_str} {ro_sn:>12d} {bad_dur} {clean_dur}")


# Onset spread vs recovery spread vs re-onset spread
onset_sns = [onset_results[p]['sample_number'] for p in REP_CHANNELS]
reonset_sns = [reonset_results[p]['sample_number'] for p in REP_CHANNELS]
valid_recov_sns = [recovery_results[p]['sample_number'] for p in REP_CHANNELS if recovery_results[p] is not None]

print(f"\n--- SPREAD COMPARISON ---")
print(f"Onset spread:    {max(onset_sns) - min(onset_sns)} samples ({(max(onset_sns)-min(onset_sns))*4}ms)")
if valid_recov_sns:
    print(f"Recovery spread: {max(valid_recov_sns) - min(valid_recov_sns)} samples ({(max(valid_recov_sns)-min(valid_recov_sns))*4}ms)")
print(f"Re-onset spread: {max(reonset_sns) - min(reonset_sns)} samples ({(max(reonset_sns)-min(reonset_sns))*4}ms)")


# ═══════════════════════════════════════════════════════════════════════
#  STEP 7: Is the recovery a mirror of the onset? Compare ramp shapes
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 7: ONSET vs RECOVERY RAMP SHAPE COMPARISON")
print("="*80)

# For each port, extract the transition profile as z-scores relative to clean mean
# Onset: samples around 371626
# Recovery: samples around detected recovery sample
# Align them at t=0 (transition sample) and compare

print("\nOnset ramp (z-scores from clean, t=0 at transition):")
onset_center = 371626
onset_window = (df['sample_number'] >= onset_center - 3) & (df['sample_number'] <= onset_center + 8)
onset_df = df[onset_window].copy()

print(f"{'Offset':>8}", end="")
for port in REP_CHANNELS:
    print(f" {port:>8}", end="")
print()
for _, row in onset_df.iterrows():
    rel = int(row['sample_number']) - onset_center
    print(f"{rel:>+8d}", end="")
    for port, ch in REP_CHANNELS.items():
        z = (int(row[ch]) - clean_stats[port]['mean']) / clean_stats[port]['std']
        print(f" {z:>+8.1f}", end="")
    print()

# Recovery ramp
if valid_recov_sns:
    recov_center = int(np.median(valid_recov_sns))
    print(f"\nRecovery ramp (z-scores from clean, t=0 at transition, center sn={recov_center}):")
    recov_window = (df['sample_number'] >= recov_center - 3) & (df['sample_number'] <= recov_center + 8)
    recov_df = df[recov_window].copy()

    print(f"{'Offset':>8}", end="")
    for port in REP_CHANNELS:
        print(f" {port:>8}", end="")
    print()
    for _, row in recov_df.iterrows():
        rel = int(row['sample_number']) - recov_center
        print(f"{rel:>+8d}", end="")
        for port, ch in REP_CHANNELS.items():
            z = (int(row[ch]) - clean_stats[port]['mean']) / clean_stats[port]['std']
            print(f" {z:>+8.1f}", end="")
        print()

# Re-onset ramp
reonset_center = 410476
print(f"\nRe-onset ramp (z-scores from clean, t=0 at transition, center sn={reonset_center}):")
reonset_window = (df['sample_number'] >= reonset_center - 3) & (df['sample_number'] <= reonset_center + 8)
reonset_df = df[reonset_window].copy()

print(f"{'Offset':>8}", end="")
for port in REP_CHANNELS:
    print(f" {port:>8}", end="")
print()
for _, row in reonset_df.iterrows():
    rel = int(row['sample_number']) - reonset_center
    print(f"{rel:>+8d}", end="")
    for port, ch in REP_CHANNELS.items():
        z = (int(row[ch]) - clean_stats[port]['mean']) / clean_stats[port]['std']
        print(f" {z:>+8.1f}", end="")
    print()


# ═══════════════════════════════════════════════════════════════════════
#  STEP 8: Does the "clean" recovery window truly match original clean?
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("STEP 8: RECOVERY QUALITY — How clean is the 'clean' window?")
print("="*80)

recov_clean_mask = (df['timestamp'] >= 1615.0) & (df['timestamp'] <= 1635.0)
if recov_clean_mask.sum() > 0:
    print(f"\nRecovery-clean region: t=1615-1635s, {recov_clean_mask.sum()} samples")
    print(f"\n{'Port':<8} {'RecovMean':>14} {'CleanMean':>14} {'Offset':>14} {'RecovStd':>12} {'CleanStd':>12} {'StdRatio':>10}")
    print("-" * 78)
    for port, ch in REP_CHANNELS.items():
        vals = df.loc[recov_clean_mask, ch].values
        recov_mean = np.mean(vals)
        recov_std = np.std(vals)
        offset = recov_mean - clean_stats[port]['mean']
        ratio = recov_std / clean_stats[port]['std']
        print(f"{port:<8} {recov_mean:>14.0f} {clean_stats[port]['mean']:>14.0f} {offset:>+14.0f} {recov_std:>12.0f} {clean_stats[port]['std']:>12.0f} {ratio:>10.2f}x")


print("\n" + "="*80)
print("RECOVERY ANALYSIS COMPLETE")
print("="*80)
