"""
Task A.1: Cross-Port Timing Analysis
=====================================
Determine if all 7 ports shift at the EXACT same sample or with a port-by-port delay
when the destabilization event occurs at ~t=1480s.

Key question: Is the cause upstream of all SPI buses (shared power/ground)?
"""
import numpy as np
import pandas as pd
import time

CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv"

# Representative channels - one per port (non-railed)
REPRESENTATIVE_CHANNELS = {
    'Port1': 'Port1_dev1_ch1',
    'Port2': 'Port2_dev1_ch1',
    'Port3': 'Port3_dev1_ch1',
    'Port4': 'Port4_dev1_ch4',  # some Port4_dev1 channels may be railed
    'Port5': 'Port5_dev1_ch1',
    'Port6': 'Port6_dev1_ch1',
    'Port7': 'Port7_dev1_ch1',
}

SPI_BUS_GROUPS = {
    'SPI0': ['Port1', 'Port2'],
    'SPI3': ['Port3', 'Port4'],
    'SPI4': ['Port5', 'Port6'],
    'SPI5': ['Port7'],
}

# Load only the columns we need
print("Loading data...")
t0 = time.time()
cols_to_load = ['timestamp', 'sample_number'] + list(REPRESENTATIVE_CHANNELS.values())
df = pd.read_csv(CSV_PATH, usecols=cols_to_load)
print(f"  Loaded {len(df)} rows in {time.time()-t0:.1f}s")

# Basic stats
print(f"\n--- Data Overview ---")
print(f"  Timestamp range: {df['timestamp'].iloc[0]:.3f} - {df['timestamp'].iloc[-1]:.3f} s")
print(f"  Sample range: {df['sample_number'].iloc[0]} - {df['sample_number'].iloc[-1]}")
print(f"  Duration: {df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]:.1f} s")

# ============================================================
# Step 1: Compute clean-region stats (t=1400-1470s)
# ============================================================
clean_mask = (df['timestamp'] >= 1400) & (df['timestamp'] <= 1470)
clean_df = df[clean_mask]
print(f"\n--- Clean Region (t=1400-1470s): {clean_mask.sum()} samples ---")

clean_stats = {}
for port_name, col in REPRESENTATIVE_CHANNELS.items():
    vals = clean_df[col].values.astype(np.float64)
    mean = np.mean(vals)
    std = np.std(vals)
    print(f"  {port_name} ({col}): mean={mean:,.0f}  std={std:,.1f}  min={np.min(vals):,.0f}  max={np.max(vals):,.0f}")
    clean_stats[port_name] = {'mean': mean, 'std': std}

# Check for railed channels (std near 0 or values near ±8.14M)
print("\n--- Railed Channel Check ---")
for port_name, col in REPRESENTATIVE_CHANNELS.items():
    vals = clean_df[col].values.astype(np.float64)
    if clean_stats[port_name]['std'] < 10:
        print(f"  WARNING: {port_name} ({col}) has near-zero std ({clean_stats[port_name]['std']:.1f}) - possibly railed!")
    pct_near_rail = np.mean(np.abs(vals) > 8_000_000) * 100
    if pct_near_rail > 5:
        print(f"  WARNING: {port_name} ({col}) has {pct_near_rail:.1f}% samples near rail!")

# ============================================================
# Step 2: Z-score analysis around the onset (~t=1480s)
# ============================================================
print("\n\n========================================")
print("ONSET DETECTION (searching t=1470-1500s)")
print("========================================")

# Focus on the transition region
onset_mask = (df['timestamp'] >= 1470) & (df['timestamp'] <= 1500)
onset_df = df[onset_mask].copy()

onset_results = {}
for port_name, col in REPRESENTATIVE_CHANNELS.items():
    vals = onset_df[col].values.astype(np.float64)
    cmean = clean_stats[port_name]['mean']
    cstd = clean_stats[port_name]['std']

    if cstd < 1e-6:
        print(f"  SKIP {port_name}: zero std in clean region")
        continue

    z_scores = np.abs((vals - cmean) / cstd)

    # Find FIRST sample where |z| > 10
    threshold_idx = np.where(z_scores > 10)[0]
    if len(threshold_idx) > 0:
        first_idx = threshold_idx[0]
        global_idx = onset_df.index[first_idx]
        sample_num = df['sample_number'].iloc[global_idx]
        timestamp = df['timestamp'].iloc[global_idx]
        z_val = z_scores[first_idx]
        raw_val = vals[first_idx]
        onset_results[port_name] = {
            'sample_number': int(sample_num),
            'timestamp': float(timestamp),
            'z_score': float(z_val),
            'raw_value': float(raw_val),
            'global_idx': int(global_idx),
        }
    else:
        print(f"  {port_name}: No exceedance found in t=1470-1500s!")

# ============================================================
# Step 3: Report results
# ============================================================
print("\n--- First Exceedance (|z| > 10) per Port ---")
print(f"{'Port':<8} {'Sample#':<12} {'Timestamp':>14} {'Z-score':>10} {'Raw Value':>14}")
print("-" * 62)

earliest_sample = min(r['sample_number'] for r in onset_results.values())
latest_sample = max(r['sample_number'] for r in onset_results.values())

for port_name in sorted(onset_results.keys()):
    r = onset_results[port_name]
    offset = r['sample_number'] - earliest_sample
    print(f"{port_name:<8} {r['sample_number']:<12} {r['timestamp']:>14.6f} {r['z_score']:>10.1f} {r['raw_value']:>14,.0f}  (offset: +{offset} samples)")

print(f"\n--- Sample Offset Summary ---")
print(f"  Earliest exceedance: sample {earliest_sample}")
print(f"  Latest exceedance:   sample {latest_sample}")
print(f"  Spread:              {latest_sample - earliest_sample} samples ({(latest_sample - earliest_sample) * 4} ms)")

# ============================================================
# Step 4: Group by SPI bus
# ============================================================
print("\n--- SPI Bus Grouping ---")
for bus_name, ports in SPI_BUS_GROUPS.items():
    samples = []
    for p in ports:
        if p in onset_results:
            samples.append(onset_results[p]['sample_number'])
    if len(samples) > 1:
        spread = max(samples) - min(samples)
        print(f"  {bus_name} ({', '.join(ports)}): samples {samples}, spread={spread}")
    elif len(samples) == 1:
        print(f"  {bus_name} ({', '.join(ports)}): sample {samples[0]} (single port)")

# ============================================================
# Step 5: Detailed sample-by-sample view around the transition
# ============================================================
print("\n\n========================================")
print("SAMPLE-BY-SAMPLE VIEW AROUND ONSET")
print("========================================")

# Show 10 samples before and 10 after the earliest exceedance
# Use global index for slicing
earliest_global_idx = min(r['global_idx'] for r in onset_results.values())
start_idx = max(0, earliest_global_idx - 10)
end_idx = min(len(df), earliest_global_idx + 15)

print(f"\nShowing samples {df['sample_number'].iloc[start_idx]} to {df['sample_number'].iloc[end_idx-1]}")
print(f"{'Sample#':<10} {'Timestamp':>12}", end='')
for port_name in sorted(REPRESENTATIVE_CHANNELS.keys()):
    print(f"  {port_name:>14}", end='')
print()
print("-" * (24 + 16 * 7))

for i in range(start_idx, end_idx):
    sn = df['sample_number'].iloc[i]
    ts = df['timestamp'].iloc[i]
    marker = ""
    for port_name in sorted(onset_results.keys()):
        if onset_results[port_name]['sample_number'] == sn:
            marker = " <<<"
            break

    print(f"{int(sn):<10} {ts:>12.4f}", end='')
    for port_name in sorted(REPRESENTATIVE_CHANNELS.keys()):
        col = REPRESENTATIVE_CHANNELS[port_name]
        val = df[col].iloc[i]
        # Compute z-score for this sample
        cmean = clean_stats[port_name]['mean']
        cstd = clean_stats[port_name]['std']
        z = abs((val - cmean) / cstd) if cstd > 0 else 0
        if z > 10:
            print(f"  {val:>12,.0f}*", end='')
        else:
            print(f"  {val:>12,.0f} ", end='')
    print(marker)

print("\n(* = |z| > 10 relative to clean baseline)")

# ============================================================
# Step 6: Multi-threshold analysis for sub-sample precision
# ============================================================
print("\n\n========================================")
print("MULTI-THRESHOLD ONSET DETECTION")
print("========================================")
print("(Testing thresholds: 3, 5, 10, 20, 50 sigma)")

for threshold in [3, 5, 10, 20, 50]:
    print(f"\n--- Threshold: |z| > {threshold} ---")
    results_t = {}
    for port_name, col in REPRESENTATIVE_CHANNELS.items():
        vals = onset_df[col].values.astype(np.float64)
        cmean = clean_stats[port_name]['mean']
        cstd = clean_stats[port_name]['std']
        if cstd < 1e-6:
            continue
        z_scores = np.abs((vals - cmean) / cstd)
        threshold_idx = np.where(z_scores > threshold)[0]
        if len(threshold_idx) > 0:
            first_idx = threshold_idx[0]
            global_idx = onset_df.index[first_idx]
            sample_num = int(df['sample_number'].iloc[global_idx])
            results_t[port_name] = sample_num

    if results_t:
        earliest = min(results_t.values())
        print(f"  {'Port':<8} {'Sample#':<12} {'Offset':<8}")
        for p in sorted(results_t.keys()):
            print(f"  {p:<8} {results_t[p]:<12} +{results_t[p] - earliest}")
        spread = max(results_t.values()) - min(results_t.values())
        print(f"  Spread: {spread} samples")

# ============================================================
# Step 7: Check if Port4_dev1_ch4 is good; try alternates if not
# ============================================================
print("\n\n========================================")
print("PORT4 CHANNEL VALIDATION")
print("========================================")

port4_candidates = ['Port4_dev1_ch1', 'Port4_dev1_ch2', 'Port4_dev1_ch3',
                    'Port4_dev1_ch4', 'Port4_dev1_ch5', 'Port4_dev1_ch6',
                    'Port4_dev1_ch7', 'Port4_dev1_ch8']

# Load Port4 dev1 channels for validation
p4_cols = ['timestamp', 'sample_number'] + port4_candidates
p4_df = pd.read_csv(CSV_PATH, usecols=p4_cols)
p4_clean = p4_df[(p4_df['timestamp'] >= 1400) & (p4_df['timestamp'] <= 1470)]

for ch in port4_candidates:
    vals = p4_clean[ch].values.astype(np.float64)
    mean = np.mean(vals)
    std = np.std(vals)
    pct_rail = np.mean(np.abs(vals) > 8_000_000) * 100
    status = "RAILED" if pct_rail > 5 or std < 10 else "OK"
    print(f"  {ch}: mean={mean:>12,.0f}  std={std:>10,.1f}  rail%={pct_rail:>5.1f}%  [{status}]")

print("\nDone.")
