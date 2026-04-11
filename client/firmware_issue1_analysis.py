"""
Firmware Issue 1 Analysis: Cross-Port Timing & Recovery Dynamics
================================================================
Analyzes an intermittent DC offset destabilization event affecting all 336 channels.
Two tasks:
  A.1 — Cross-port onset timing (do all ports shift simultaneously?)
  A.5 — Recovery dynamics at t=1607s and re-onset at t=1640s
"""

import numpy as np
import pandas as pd
import sys
import time

CSV_PATH = "C:/Users/cadav/Desktop/InternalDataCollection/CodeTesting/client/FirmwareIssue1.csv"
OUTPUT_DIR = "C:/Users/cadav/Desktop/InternalDataCollection/CodeTesting/client/firmware_issue1_output"

import os
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ─── Representative channels (one per port, confirmed non-railed) ───
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

# ─── Load data ───
print("Loading CSV (79k rows, 338 cols)...")
t0 = time.time()

cols_to_load = ['timestamp', 'sample_number'] + list(REP_CHANNELS.values())
df = pd.read_csv(CSV_PATH, usecols=cols_to_load)
df['timestamp'] = df['timestamp'].astype(np.float64)
df['sample_number'] = df['sample_number'].astype(np.int64)
for ch in REP_CHANNELS.values():
    df[ch] = df[ch].astype(np.int64)

print(f"  Loaded in {time.time()-t0:.1f}s. Shape: {df.shape}")
print(f"  Time range: {df['timestamp'].iloc[0]:.3f} - {df['timestamp'].iloc[-1]:.3f} s")
print(f"  Sample range: {df['sample_number'].iloc[0]} - {df['sample_number'].iloc[-1]}")


# ═══════════════════════════════════════════════════════════════════════
#  TASK A.1: Cross-Port Onset Timing
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("TASK A.1: CROSS-PORT ONSET TIMING")
print("="*80)

# Step 1: Compute clean-region statistics (t=1400-1470s)
clean_mask = (df['timestamp'] >= 1400.0) & (df['timestamp'] <= 1470.0)
clean_df = df[clean_mask]
print(f"\nClean region: t=1400-1470s, {clean_mask.sum()} samples")
print(f"  Sample range: {clean_df['sample_number'].iloc[0]} - {clean_df['sample_number'].iloc[-1]}")

clean_stats = {}
print(f"\n{'Port':<8} {'Channel':<20} {'Clean Mean':>14} {'Clean Std':>12} {'Clean Min':>14} {'Clean Max':>14}")
print("-" * 86)
for port, ch in REP_CHANNELS.items():
    vals = clean_df[ch].values
    mean_ = np.mean(vals)
    std_ = np.std(vals)
    min_ = np.min(vals)
    max_ = np.max(vals)
    clean_stats[port] = {'mean': mean_, 'std': std_}
    print(f"{port:<8} {ch:<20} {mean_:>14.1f} {std_:>12.1f} {min_:>14d} {max_:>14d}")

# Step 2: Find first onset (z > 10) in the transition region
# Search from t=1470 to t=1520 to catch the onset
search_mask_onset = (df['timestamp'] >= 1470.0) & (df['timestamp'] <= 1520.0)
search_df_onset = df[search_mask_onset].copy()

print(f"\n--- Onset Detection (searching t=1470-1520s, |z| > 10 threshold) ---")
print(f"Search region: {search_mask_onset.sum()} samples")

onset_results = {}
print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Timestamp':>12} {'Value':>14} {'z-score':>10}")
print("-" * 66)

for port, ch in REP_CHANNELS.items():
    mean_ = clean_stats[port]['mean']
    std_ = clean_stats[port]['std']
    vals = search_df_onset[ch].values
    z_scores = (vals - mean_) / std_

    # Find first sample where |z| > 10
    exceed_idx = np.where(np.abs(z_scores) > 10)[0]
    if len(exceed_idx) > 0:
        first_idx = exceed_idx[0]
        global_idx = search_df_onset.index[first_idx]
        sn = df.loc[global_idx, 'sample_number']
        ts = df.loc[global_idx, 'timestamp']
        val = df.loc[global_idx, ch]
        z = z_scores[first_idx]
        onset_results[port] = {'sample_number': int(sn), 'timestamp': ts, 'value': int(val), 'z_score': z}
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {sn:>10d} {ts:>12.3f} {val:>14d} {z:>10.1f}")
    else:
        onset_results[port] = None
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {'NOT FOUND':>10}")

# Step 3: Compute offsets between ports
print("\n--- Port-by-Port Onset Offsets ---")
valid_onsets = {p: r for p, r in onset_results.items() if r is not None}
if len(valid_onsets) >= 2:
    earliest_port = min(valid_onsets, key=lambda p: valid_onsets[p]['sample_number'])
    latest_port = max(valid_onsets, key=lambda p: valid_onsets[p]['sample_number'])
    earliest_sn = valid_onsets[earliest_port]['sample_number']
    latest_sn = valid_onsets[latest_port]['sample_number']

    print(f"\nEarliest onset: {earliest_port} at sample {earliest_sn} (t={valid_onsets[earliest_port]['timestamp']:.3f}s)")
    print(f"Latest onset:   {latest_port} at sample {latest_sn} (t={valid_onsets[latest_port]['timestamp']:.3f}s)")
    print(f"Total spread:   {latest_sn - earliest_sn} samples ({(latest_sn - earliest_sn)*4:.0f} ms)")

    print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Offset from earliest':>22} {'ms offset':>10}")
    print("-" * 62)
    for port in sorted(valid_onsets.keys()):
        r = valid_onsets[port]
        offset = r['sample_number'] - earliest_sn
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {r['sample_number']:>10d} {offset:>+22d} {offset*4:>+10.0f}")

# Step 4: Group by SPI bus
print("\n--- SPI Bus Grouping ---")
bus_groups = {}
for port, r in valid_onsets.items():
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


# ─── Finer-grained onset analysis: sample-by-sample around the transition ───
print("\n--- Sample-by-Sample Onset Detail (5 samples before & after first |z|>10) ---")
for port, ch in REP_CHANNELS.items():
    if onset_results[port] is None:
        continue
    onset_sn = onset_results[port]['sample_number']
    mean_ = clean_stats[port]['mean']
    std_ = clean_stats[port]['std']

    # Get samples around onset
    mask = (df['sample_number'] >= onset_sn - 5) & (df['sample_number'] <= onset_sn + 5)
    local = df[mask]

    print(f"\n  {port} ({ch}), clean mean={mean_:.0f}, clean std={std_:.0f}:")
    for _, row in local.iterrows():
        val = int(row[ch])
        z = (val - mean_) / std_
        marker = " <<<" if row['sample_number'] == onset_sn else ""
        print(f"    sn={int(row['sample_number']):>8d}  t={row['timestamp']:>10.3f}  val={val:>10d}  z={z:>8.1f}{marker}")


# ─── Additional: check if onset is a step function or gradual ramp ───
print("\n--- Onset Characterization: Step vs Ramp ---")
for port, ch in REP_CHANNELS.items():
    if onset_results[port] is None:
        continue
    onset_sn = onset_results[port]['sample_number']
    mean_ = clean_stats[port]['mean']
    std_ = clean_stats[port]['std']

    # Get 2 samples before onset and 2 after
    mask = (df['sample_number'] >= onset_sn - 2) & (df['sample_number'] <= onset_sn + 2)
    local = df[mask].copy()
    local['z'] = (local[ch] - mean_) / std_

    pre_vals = local[local['sample_number'] < onset_sn][ch].values
    post_vals = local[local['sample_number'] >= onset_sn][ch].values

    if len(pre_vals) > 0 and len(post_vals) > 0:
        jump = post_vals[0] - pre_vals[-1]
        print(f"  {port}: pre-onset={pre_vals[-1]:>10d}, onset={post_vals[0]:>10d}, jump={jump:>+10d} LSB ({jump/std_:>+.1f} sigma)")


# ═══════════════════════════════════════════════════════════════════════
#  TASK A.5: Recovery Dynamics (t=1600-1645s)
# ═══════════════════════════════════════════════════════════════════════
print("\n\n" + "="*80)
print("TASK A.5: RECOVERY DYNAMICS (t=1600-1645s)")
print("="*80)

# Step 1: Compute bad-region statistics (t=1500-1590s) for detecting recovery
bad_mask = (df['timestamp'] >= 1500.0) & (df['timestamp'] <= 1590.0)
bad_df = df[bad_mask]
print(f"\nBad region baseline: t=1500-1590s, {bad_mask.sum()} samples")

bad_stats = {}
print(f"\n{'Port':<8} {'Channel':<20} {'Bad Mean':>14} {'Bad Std':>12}")
print("-" * 58)
for port, ch in REP_CHANNELS.items():
    vals = bad_df[ch].values
    mean_ = np.mean(vals)
    std_ = np.std(vals)
    bad_stats[port] = {'mean': mean_, 'std': std_}
    print(f"{port:<8} {ch:<20} {mean_:>14.1f} {std_:>12.1f}")

# Show the DC offset
print(f"\n--- DC Offset (Bad Mean - Clean Mean) ---")
print(f"{'Port':<8} {'Offset (LSB)':>14} {'Offset (approx uV @ gain=24)':>30}")
print("-" * 56)
for port in REP_CHANNELS.keys():
    offset = bad_stats[port]['mean'] - clean_stats[port]['mean']
    # ADS1299: LSB = Vref/(2^23 * gain), Vref=4.5V, gain=24 => LSB = 0.02235 uV
    uv_offset = offset * 0.02235
    print(f"{port:<8} {offset:>+14.0f} {uv_offset:>+30.1f}")

# Noise increase
print(f"\n--- Noise Increase (Bad Std / Clean Std) ---")
print(f"{'Port':<8} {'Clean Std':>12} {'Bad Std':>12} {'Ratio':>8}")
print("-" * 44)
for port in REP_CHANNELS.keys():
    ratio = bad_stats[port]['std'] / clean_stats[port]['std']
    print(f"{port:<8} {clean_stats[port]['std']:>12.1f} {bad_stats[port]['std']:>12.1f} {ratio:>8.1f}x")


# Step 2: Recovery detection (bad -> clean at ~t=1607)
print(f"\n--- Recovery Detection (t=1595-1615s, return to within 3-sigma of clean baseline) ---")
search_mask_recov = (df['timestamp'] >= 1595.0) & (df['timestamp'] <= 1615.0)
search_df_recov = df[search_mask_recov].copy()
print(f"Search region: {search_mask_recov.sum()} samples")

recovery_results = {}
print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Timestamp':>12} {'Value':>14} {'z(clean)':>10}")
print("-" * 66)

for port, ch in REP_CHANNELS.items():
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']
    vals = search_df_recov[ch].values
    z_clean = (vals - clean_mean) / clean_std

    # Find first sample where |z_clean| < 3 (back to clean baseline)
    # But we need to be careful: require 3 consecutive samples within 3-sigma
    # to avoid false triggers from noisy oscillation
    within_3sig = np.abs(z_clean) < 3

    # Find first run of 3+ consecutive within-clean samples
    found = False
    for i in range(len(within_3sig) - 2):
        if within_3sig[i] and within_3sig[i+1] and within_3sig[i+2]:
            global_idx = search_df_recov.index[i]
            sn = df.loc[global_idx, 'sample_number']
            ts = df.loc[global_idx, 'timestamp']
            val = df.loc[global_idx, ch]
            z = z_clean[i]
            recovery_results[port] = {'sample_number': int(sn), 'timestamp': ts, 'value': int(val), 'z_score': z}
            print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {sn:>10d} {ts:>12.3f} {val:>14d} {z:>10.1f}")
            found = True
            break
    if not found:
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


# Recovery SPI bus grouping
print("\n--- Recovery SPI Bus Grouping ---")
bus_groups_recov = {}
for port, r in valid_recov.items():
    bus = SPI_BUS_MAP[port]
    if bus not in bus_groups_recov:
        bus_groups_recov[bus] = []
    bus_groups_recov[bus].append((port, r['sample_number']))

for bus in sorted(bus_groups_recov.keys()):
    ports = bus_groups_recov[bus]
    sns = [sn for _, sn in ports]
    spread = max(sns) - min(sns)
    port_list = ', '.join([f"{p}(sn={sn})" for p, sn in ports])
    print(f"{bus}: {port_list} -> spread = {spread} samples ({spread*4}ms)")


# Step 3: Sample-by-sample recovery detail
print("\n--- Sample-by-Sample Recovery Detail (5 samples before & after recovery) ---")
for port, ch in REP_CHANNELS.items():
    if recovery_results[port] is None:
        continue
    recov_sn = recovery_results[port]['sample_number']
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']

    mask = (df['sample_number'] >= recov_sn - 5) & (df['sample_number'] <= recov_sn + 5)
    local = df[mask]

    print(f"\n  {port} ({ch}), clean mean={clean_mean:.0f}, clean std={clean_std:.0f}:")
    for _, row in local.iterrows():
        val = int(row[ch])
        z = (val - clean_mean) / clean_std
        marker = " <<<" if row['sample_number'] == recov_sn else ""
        print(f"    sn={int(row['sample_number']):>8d}  t={row['timestamp']:>10.3f}  val={val:>10d}  z={z:>8.1f}{marker}")


# Step 4: Re-onset detection at ~t=1640
print(f"\n--- Re-Onset Detection (t=1630-1650s, |z| > 10 from clean baseline) ---")
search_mask_reonset = (df['timestamp'] >= 1630.0) & (df['timestamp'] <= 1650.0)
search_df_reonset = df[search_mask_reonset].copy()
print(f"Search region: {search_mask_reonset.sum()} samples")

# First verify data is clean at t=1630
print(f"\nVerification: data at t=1630-1632 (should be clean):")
verify_mask = (df['timestamp'] >= 1630.0) & (df['timestamp'] <= 1632.0)
verify_df = df[verify_mask]
for port, ch in REP_CHANNELS.items():
    vals = verify_df[ch].values
    z_vals = (vals - clean_stats[port]['mean']) / clean_stats[port]['std']
    print(f"  {port}: mean z={np.mean(z_vals):.1f}, max |z|={np.max(np.abs(z_vals)):.1f}")

reonset_results = {}
print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Timestamp':>12} {'Value':>14} {'z-score':>10}")
print("-" * 66)

for port, ch in REP_CHANNELS.items():
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']
    vals = search_df_reonset[ch].values
    z_scores = (vals - clean_mean) / clean_std

    exceed_idx = np.where(np.abs(z_scores) > 10)[0]
    if len(exceed_idx) > 0:
        first_idx = exceed_idx[0]
        global_idx = search_df_reonset.index[first_idx]
        sn = df.loc[global_idx, 'sample_number']
        ts = df.loc[global_idx, 'timestamp']
        val = df.loc[global_idx, ch]
        z = z_scores[first_idx]
        reonset_results[port] = {'sample_number': int(sn), 'timestamp': ts, 'value': int(val), 'z_score': z}
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {sn:>10d} {ts:>12.3f} {val:>14d} {z:>10.1f}")
    else:
        reonset_results[port] = None
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {'NOT FOUND':>10}")

# Re-onset offsets
print("\n--- Re-Onset Port-by-Port Offsets ---")
valid_reonset = {p: r for p, r in reonset_results.items() if r is not None}
if len(valid_reonset) >= 2:
    earliest_port = min(valid_reonset, key=lambda p: valid_reonset[p]['sample_number'])
    latest_port = max(valid_reonset, key=lambda p: valid_reonset[p]['sample_number'])
    earliest_sn = valid_reonset[earliest_port]['sample_number']
    latest_sn = valid_reonset[latest_port]['sample_number']

    print(f"\nEarliest re-onset: {earliest_port} at sample {earliest_sn} (t={valid_reonset[earliest_port]['timestamp']:.3f}s)")
    print(f"Latest re-onset:   {latest_port} at sample {latest_sn} (t={valid_reonset[latest_port]['timestamp']:.3f}s)")
    print(f"Total spread:      {latest_sn - earliest_sn} samples ({(latest_sn - earliest_sn)*4:.0f} ms)")

    print(f"\n{'Port':<8} {'SPI Bus':<8} {'Sample #':>10} {'Offset from earliest':>22} {'ms offset':>10}")
    print("-" * 62)
    for port in sorted(valid_reonset.keys()):
        r = valid_reonset[port]
        offset = r['sample_number'] - earliest_sn
        print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {r['sample_number']:>10d} {offset:>+22d} {offset*4:>+10.0f}")

# Re-onset SPI bus grouping
print("\n--- Re-Onset SPI Bus Grouping ---")
bus_groups_reonset = {}
for port, r in valid_reonset.items():
    bus = SPI_BUS_MAP[port]
    if bus not in bus_groups_reonset:
        bus_groups_reonset[bus] = []
    bus_groups_reonset[bus].append((port, r['sample_number']))

for bus in sorted(bus_groups_reonset.keys()):
    ports = bus_groups_reonset[bus]
    sns = [sn for _, sn in ports]
    spread = max(sns) - min(sns)
    port_list = ', '.join([f"{p}(sn={sn})" for p, sn in ports])
    print(f"{bus}: {port_list} -> spread = {spread} samples ({spread*4}ms)")


# Step 5: Sample-by-sample re-onset detail
print("\n--- Sample-by-Sample Re-Onset Detail (5 samples before & after) ---")
for port, ch in REP_CHANNELS.items():
    if reonset_results[port] is None:
        continue
    reonset_sn = reonset_results[port]['sample_number']
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']

    mask = (df['sample_number'] >= reonset_sn - 5) & (df['sample_number'] <= reonset_sn + 5)
    local = df[mask]

    print(f"\n  {port} ({ch}), clean mean={clean_mean:.0f}, clean std={clean_std:.0f}:")
    for _, row in local.iterrows():
        val = int(row[ch])
        z = (val - clean_mean) / clean_std
        marker = " <<<" if row['sample_number'] == reonset_sn else ""
        print(f"    sn={int(row['sample_number']):>8d}  t={row['timestamp']:>10.3f}  val={val:>10d}  z={z:>8.1f}{marker}")


# Step 6: Re-onset step characterization
print("\n--- Re-Onset Step Characterization ---")
for port, ch in REP_CHANNELS.items():
    if reonset_results[port] is None:
        continue
    reonset_sn = reonset_results[port]['sample_number']
    clean_mean = clean_stats[port]['mean']
    clean_std = clean_stats[port]['std']

    mask = (df['sample_number'] >= reonset_sn - 2) & (df['sample_number'] <= reonset_sn + 2)
    local = df[mask].copy()

    pre_vals = local[local['sample_number'] < reonset_sn][ch].values
    post_vals = local[local['sample_number'] >= reonset_sn][ch].values

    if len(pre_vals) > 0 and len(post_vals) > 0:
        jump = post_vals[0] - pre_vals[-1]
        print(f"  {port}: pre={pre_vals[-1]:>10d}, onset={post_vals[0]:>10d}, jump={jump:>+10d} LSB ({jump/clean_std:>+.1f} sigma)")


# ═══════════════════════════════════════════════════════════════════════
#  COMPARISON: Onset vs Recovery vs Re-Onset
# ═══════════════════════════════════════════════════════════════════════
print("\n\n" + "="*80)
print("COMPARISON: ONSET vs RECOVERY vs RE-ONSET")
print("="*80)

print(f"\n{'Port':<8} {'Onset sn':>10} {'Recovery sn':>12} {'Re-onset sn':>12} {'Onset->Recov':>14} {'Recov->Re-on':>14}")
print("-" * 74)
for port in REP_CHANNELS.keys():
    onset_sn = onset_results[port]['sample_number'] if onset_results[port] else None
    recov_sn = recovery_results[port]['sample_number'] if recovery_results[port] else None
    reonset_sn = reonset_results[port]['sample_number'] if reonset_results[port] else None

    o_str = f"{onset_sn:>10d}" if onset_sn else f"{'N/A':>10}"
    r_str = f"{recov_sn:>12d}" if recov_sn else f"{'N/A':>12}"
    ro_str = f"{reonset_sn:>12d}" if reonset_sn else f"{'N/A':>12}"

    dur1 = f"{recov_sn - onset_sn:>14d}" if onset_sn and recov_sn else f"{'N/A':>14}"
    dur2 = f"{reonset_sn - recov_sn:>14d}" if recov_sn and reonset_sn else f"{'N/A':>14}"

    print(f"{port:<8} {o_str} {r_str} {ro_str} {dur1} {dur2}")

# Duration of each phase
print("\n--- Phase Durations ---")
for port in REP_CHANNELS.keys():
    onset_sn = onset_results[port]['sample_number'] if onset_results[port] else None
    recov_sn = recovery_results[port]['sample_number'] if recovery_results[port] else None
    reonset_sn = reonset_results[port]['sample_number'] if reonset_results[port] else None

    if onset_sn and recov_sn and reonset_sn:
        bad_duration = (recov_sn - onset_sn) * 4 / 1000  # seconds
        clean_duration = (reonset_sn - recov_sn) * 4 / 1000
        print(f"  {port}: Bad phase = {bad_duration:.1f}s ({recov_sn-onset_sn} samples), Clean window = {clean_duration:.1f}s ({reonset_sn-recov_sn} samples)")


# ═══════════════════════════════════════════════════════════════════════
#  ONSET SYNCHRONY: Cross-correlation of onset transitions
# ═══════════════════════════════════════════════════════════════════════
print("\n\n" + "="*80)
print("ONSET SYNCHRONY: Detailed cross-port comparison")
print("="*80)

# For each port, extract a 20-sample window centered on the onset and compare the
# normalized transition shape
if all(onset_results[p] is not None for p in REP_CHANNELS):
    # Use median onset sample as center
    all_onset_sns = [onset_results[p]['sample_number'] for p in REP_CHANNELS]
    median_sn = int(np.median(all_onset_sns))

    print(f"\nOnset sample numbers: {all_onset_sns}")
    print(f"Median onset sample: {median_sn}")
    print(f"Range: {max(all_onset_sns) - min(all_onset_sns)} samples ({(max(all_onset_sns) - min(all_onset_sns))*4}ms)")

    # Extract +-20 samples around median onset for all ports
    window_mask = (df['sample_number'] >= median_sn - 20) & (df['sample_number'] <= median_sn + 20)
    window_df = df[window_mask]

    print(f"\nNormalized transition profiles (relative sample offset from median onset):")
    print(f"{'Offset':>8}", end="")
    for port in REP_CHANNELS:
        print(f"  {port:>10}", end="")
    print()
    print("-" * (8 + 12 * 7))

    for _, row in window_df.iterrows():
        rel_sn = int(row['sample_number']) - median_sn
        print(f"{rel_sn:>+8d}", end="")
        for port, ch in REP_CHANNELS.items():
            val = int(row[ch])
            z = (val - clean_stats[port]['mean']) / clean_stats[port]['std']
            print(f"  {z:>10.1f}", end="")
        print()


# ═══════════════════════════════════════════════════════════════════════
#  FINAL DIAGNOSTIC SUMMARY
# ═══════════════════════════════════════════════════════════════════════
print("\n\n" + "="*80)
print("DIAGNOSTIC SUMMARY")
print("="*80)

if all(onset_results[p] is not None for p in REP_CHANNELS):
    all_onset_sns = [onset_results[p]['sample_number'] for p in REP_CHANNELS]
    spread = max(all_onset_sns) - min(all_onset_sns)

    print(f"\n1. ONSET SPREAD: {spread} samples ({spread*4}ms)")
    if spread <= 1:
        print("   -> ALL 7 PORTS SHIFT WITHIN 0-1 SAMPLES (0-4ms)")
        print("   -> CONCLUSION: Cause is UPSTREAM of all SPI buses")
        print("   -> Consistent with: shared power supply / analog ground disturbance")
        print("   -> Inconsistent with: SPI bus issue, firmware bug, per-device failure")
    elif spread <= 5:
        print(f"   -> Ports shift within {spread} samples ({spread*4}ms)")
        print("   -> Likely upstream cause with minor propagation delay")
    else:
        print(f"   -> Ports shift over {spread} samples ({spread*4}ms)")
        print("   -> May indicate per-bus or per-port propagation")

if all(recovery_results[p] is not None for p in REP_CHANNELS):
    all_recov_sns = [recovery_results[p]['sample_number'] for p in REP_CHANNELS]
    recov_spread = max(all_recov_sns) - min(all_recov_sns)
    print(f"\n2. RECOVERY SPREAD: {recov_spread} samples ({recov_spread*4}ms)")

if all(reonset_results[p] is not None for p in REP_CHANNELS):
    all_reonset_sns = [reonset_results[p]['sample_number'] for p in REP_CHANNELS]
    reonset_spread = max(all_reonset_sns) - min(all_reonset_sns)
    print(f"\n3. RE-ONSET SPREAD: {reonset_spread} samples ({reonset_spread*4}ms)")

print("\n" + "="*80)
print("ANALYSIS COMPLETE")
print("="*80)
