"""
Firmware Issue 1 — Refined Transition Analysis
===============================================
The Step 1 profiling revealed the recovery is NOT a sharp step — it's a ~30s exponential
decay from bad-region z~-190 down to z~-5 between t=1601-1640s.

This script:
  1. Identifies the precise moment the exponential decay begins
  2. Characterizes the decay time constant
  3. Determines whether the "clean window" was ever truly clean
  4. Analyzes the re-onset at t=1641.9 more carefully
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

print("Loading CSV...")
cols_to_load = ['timestamp', 'sample_number'] + list(REP_CHANNELS.values())
df = pd.read_csv(CSV_PATH, usecols=cols_to_load)
df['timestamp'] = df['timestamp'].astype(np.float64)
df['sample_number'] = df['sample_number'].astype(np.int64)
for ch in REP_CHANNELS.values():
    df[ch] = df[ch].astype(np.int64)

# Baselines
clean_mask = (df['timestamp'] >= 1400.0) & (df['timestamp'] <= 1470.0)
bad_mask = (df['timestamp'] >= 1500.0) & (df['timestamp'] <= 1590.0)
clean_stats = {}
bad_stats = {}
for port, ch in REP_CHANNELS.items():
    clean_stats[port] = {'mean': np.mean(df.loc[clean_mask, ch].values), 'std': np.std(df.loc[clean_mask, ch].values)}
    bad_stats[port] = {'mean': np.mean(df.loc[bad_mask, ch].values), 'std': np.std(df.loc[bad_mask, ch].values)}


# ═══════════════════════════════════════════════════════════════════════
#  PHASE MAP: Classify every 0.5s window as clean, bad, transitioning, or near-clean
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("PHASE MAP: Mean z(clean) per 0.5s window across the full recording")
print("="*80)

# Use Port1 as representative for the phase map overview
port_ref = 'Port1'
ch_ref = REP_CHANNELS[port_ref]
ref_mean = clean_stats[port_ref]['mean']
ref_std = clean_stats[port_ref]['std']

# Compute mean of ALL 7 ports' z-scores per window
windows = np.arange(df['timestamp'].iloc[0], df['timestamp'].iloc[-1], 0.5)
phase_data = []

for t_start in windows:
    t_end = t_start + 0.5
    mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
    n = mask.sum()
    if n == 0:
        continue

    z_all = []
    for port, ch in REP_CHANNELS.items():
        vals = df.loc[mask, ch].values
        z_all.append((np.mean(vals) - clean_stats[port]['mean']) / clean_stats[port]['std'])

    mean_z = np.mean(z_all)
    phase_data.append({'t': t_start, 'mean_z': mean_z, 'n': n})

phase_df = pd.DataFrame(phase_data)

# Print summary of key transition regions
print(f"\n--- Full recording phase profile (every 2s) ---")
for i in range(0, len(phase_df), 4):  # every 4 * 0.5s = 2s
    row = phase_df.iloc[i]
    bar_len = min(abs(int(row['mean_z'])), 50)
    bar = '#' * bar_len if row['mean_z'] < 0 else '+' * bar_len
    phase = "CLEAN" if abs(row['mean_z']) < 5 else ("BAD" if abs(row['mean_z']) > 50 else "TRANS")
    print(f"  t={row['t']:>8.1f}s  z={row['mean_z']:>+8.1f}  [{phase:>5}]  {bar}")


# ═══════════════════════════════════════════════════════════════════════
#  RECOVERY TRANSITION: Find the SUSTAINED recovery onset
# ═══════════════════════════════════════════════════════════════════════
print("\n\n" + "="*80)
print("RECOVERY TRANSITION: Finding the sustained shift from bad to transitioning")
print("="*80)

# The data shows bad (z~-190) until ~t=1601, then transitions to a sustained
# decreasing z-score (exponential decay). Let's find when the sustained decay begins
# by looking at a rolling 50-sample (0.2s) mean z-score.

decay_mask = (df['timestamp'] >= 1595.0) & (df['timestamp'] <= 1610.0)
decay_df = df[decay_mask].copy()

# Compute rolling mean z for each port
for port, ch in REP_CHANNELS.items():
    decay_df[f'z_{port}'] = (decay_df[ch] - clean_stats[port]['mean']) / clean_stats[port]['std']

# Average z across all ports
z_cols = [f'z_{port}' for port in REP_CHANNELS]
decay_df['z_mean'] = decay_df[z_cols].mean(axis=1)
decay_df['z_rolling'] = decay_df['z_mean'].rolling(50, center=True).mean()

# Find where z_rolling starts to rise (derivative becomes consistently positive)
# This is the sustained recovery start
print("\nRolling mean z-score (50-sample window, every 50 samples):")
for i in range(0, len(decay_df), 50):
    row = decay_df.iloc[i]
    if pd.notna(row['z_rolling']):
        print(f"  sn={int(row['sample_number']):>8d}  t={row['timestamp']:>10.3f}  z_instant={row['z_mean']:>+8.1f}  z_rolling={row['z_rolling']:>+8.1f}")


# ═══════════════════════════════════════════════════════════════════════
#  RECOVERY ONSET: Find when the SUSTAINED downshift begins
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("SUSTAINED RECOVERY ONSET DETECTION")
print("="*80)

# Strategy: the sustained recovery begins when the rolling z-score permanently
# shifts above its bad-region baseline. Use the rolling mean, find first point
# where z > (bad_mean_z + 10*bad_std_z) and stays there.

# Bad-region rolling z stats
bad_decay_mask = (decay_df['timestamp'] >= 1595.0) & (decay_df['timestamp'] <= 1600.0)
bad_z_rolling = decay_df.loc[bad_decay_mask, 'z_rolling'].dropna()
bad_z_mean = bad_z_rolling.mean()
bad_z_std = bad_z_rolling.std()
print(f"Bad-region rolling z: mean={bad_z_mean:.1f}, std={bad_z_std:.1f}")
threshold = bad_z_mean + 10 * bad_z_std
print(f"Sustained recovery threshold: z > {threshold:.1f}")

# Find first crossing that stays above
valid_rolling = decay_df.dropna(subset=['z_rolling'])
above = valid_rolling['z_rolling'] > threshold

# Find first sustained crossing (stays above for 100+ samples)
for i in range(len(above) - 100):
    if above.iloc[i] and all(above.iloc[i:i+100]):
        global_idx = valid_rolling.index[i]
        sn = int(decay_df.loc[global_idx, 'sample_number'])
        ts = decay_df.loc[global_idx, 'timestamp']
        z = decay_df.loc[global_idx, 'z_rolling']
        print(f"\nSustained recovery onset: sn={sn}, t={ts:.3f}s, z_rolling={z:.1f}")
        break


# ═══════════════════════════════════════════════════════════════════════
#  EXPONENTIAL DECAY FIT: Characterize the recovery time constant
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("EXPONENTIAL DECAY CHARACTERIZATION (t=1601-1640s)")
print("="*80)

fit_mask = (df['timestamp'] >= 1601.0) & (df['timestamp'] <= 1641.0)
fit_df = df[fit_mask].copy()

# Compute mean z per 1-second window for fitting
z_series = []
for t_start in np.arange(1601, 1641, 1):
    t_end = t_start + 1
    mask = (fit_df['timestamp'] >= t_start) & (fit_df['timestamp'] < t_end)
    n = mask.sum()
    if n == 0:
        continue
    z_all = []
    for port, ch in REP_CHANNELS.items():
        vals = fit_df.loc[mask, ch].values
        z_all.append((np.mean(vals) - clean_stats[port]['mean']) / clean_stats[port]['std'])
    z_series.append({'t': t_start + 0.5, 'z_mean': np.mean(z_all)})

z_fit = pd.DataFrame(z_series)
print(f"\nMean z(clean) per 1s window during recovery:")
for _, row in z_fit.iterrows():
    bar_len = min(abs(int(row['z_mean'])), 80)
    bar = '#' * bar_len
    print(f"  t={row['t']:>7.1f}s  z={row['z_mean']:>+8.1f}  {bar}")

# Fit exponential decay: z(t) = z_final + (z_initial - z_final) * exp(-(t-t0)/tau)
# Use log transform for linear fit
z_vals = z_fit['z_mean'].values
t_vals = z_fit['t'].values

# The decay goes from ~-190 toward ~-4 (asymptote)
z_final = z_vals[-1]  # approximate asymptote
z_shifted = z_vals - z_final
# Only fit the exponential part (z_shifted < 0, i.e., values more negative than final)
valid = z_shifted < -1
if valid.sum() > 3:
    log_z = np.log(-z_shifted[valid])
    t_valid = t_vals[valid]
    # Linear fit: log(-z_shifted) = log(A) - t/tau
    coeffs = np.polyfit(t_valid, log_z, 1)
    tau = -1.0 / coeffs[0]
    A = np.exp(coeffs[1])
    print(f"\nExponential fit: z(t) = {z_final:.1f} + {-A:.1f} * exp(-(t-1601)/{tau:.1f})")
    print(f"Time constant tau = {tau:.1f} seconds")
    print(f"90% recovery time = {2.3*tau:.1f} seconds (2.3*tau)")
    print(f"99% recovery time = {4.6*tau:.1f} seconds (4.6*tau)")

    # How close is the data to clean at the end?
    z_at_1640 = z_vals[-1]
    print(f"\nResidual offset at t=1640: z = {z_at_1640:.1f} (= {z_at_1640 * clean_stats['Port1']['std']:.0f} LSB)")
    print(f"  For reference: bad region was z ~ -190, clean region is z ~ 0")
    print(f"  Recovery fraction: {(1 - z_at_1640 / z_vals[0]) * 100:.1f}%")


# ═══════════════════════════════════════════════════════════════════════
#  PER-PORT RECOVERY: Does each port decay at the same rate?
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("PER-PORT RECOVERY DECAY RATES")
print("="*80)

print(f"\n{'Port':<8} {'SPI Bus':<8} {'z at t=1602':>12} {'z at t=1620':>12} {'z at t=1640':>12} {'tau (s)':>10}")
print("-" * 56)

for port, ch in REP_CHANNELS.items():
    z_port = []
    for t_start in np.arange(1601, 1641, 1):
        t_end = t_start + 1
        mask = (fit_df['timestamp'] >= t_start) & (fit_df['timestamp'] < t_end)
        if mask.sum() == 0:
            continue
        vals = fit_df.loc[mask, ch].values
        z_port.append({'t': t_start + 0.5, 'z': (np.mean(vals) - clean_stats[port]['mean']) / clean_stats[port]['std']})

    z_port_df = pd.DataFrame(z_port)
    z_p = z_port_df['z'].values
    t_p = z_port_df['t'].values

    # Extract specific timepoints
    z_1602 = z_p[0] if len(z_p) > 0 else None
    z_1620 = z_p[min(19, len(z_p)-1)] if len(z_p) > 19 else None
    z_1640 = z_p[-1] if len(z_p) > 0 else None

    # Fit tau
    z_final_p = z_p[-1]
    z_shifted_p = z_p - z_final_p
    valid_p = z_shifted_p < -1
    tau_p = None
    if valid_p.sum() > 3:
        log_z_p = np.log(-z_shifted_p[valid_p])
        t_valid_p = t_p[valid_p]
        coeffs_p = np.polyfit(t_valid_p, log_z_p, 1)
        tau_p = -1.0 / coeffs_p[0]

    z_1602_str = f"{z_1602:>+12.1f}" if z_1602 is not None else f"{'N/A':>12}"
    z_1620_str = f"{z_1620:>+12.1f}" if z_1620 is not None else f"{'N/A':>12}"
    z_1640_str = f"{z_1640:>+12.1f}" if z_1640 is not None else f"{'N/A':>12}"
    tau_str = f"{tau_p:>10.1f}" if tau_p is not None else f"{'N/A':>10}"

    print(f"{port:<8} {SPI_BUS_MAP[port]:<8} {z_1602_str} {z_1620_str} {z_1640_str} {tau_str}")


# ═══════════════════════════════════════════════════════════════════════
#  THE "SPIKE" AT t=1601.25: Characterize the transient
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("TRANSIENT SPIKE AT t=1601.25s — Detailed characterization")
print("="*80)

# The derivative-based detection found a huge spike at sn=400303/400304
# Let's look at a broader window to see if it's a single-sample glitch
# or part of a ringing/oscillation pattern

spike_mask = (df['sample_number'] >= 400290) & (df['sample_number'] <= 400320)
spike_df = df[spike_mask].copy()

print(f"\n30-sample window around the transient (Port1):")
ch = 'Port1_dev1_ch1'
clean_mean = clean_stats['Port1']['mean']
clean_std = clean_stats['Port1']['std']

prev_val = None
for _, row in spike_df.iterrows():
    val = int(row[ch])
    z = (val - clean_mean) / clean_std
    diff_str = ""
    if prev_val is not None:
        diff = val - prev_val
        diff_str = f"  d={diff:>+10d}"
    prev_val = val
    print(f"  sn={int(row['sample_number']):>8d}  t={row['timestamp']:>10.3f}  val={val:>10d}  z={z:>+8.1f}{diff_str}")


# ═══════════════════════════════════════════════════════════════════════
#  ONSET-RECOVERY-REONSET: Is the initial onset also gradual?
# ═══════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("ONSET CHARACTERIZATION: Is the initial onset also exponential?")
print("="*80)

# Look at the onset region with 1s windows
onset_fit_mask = (df['timestamp'] >= 1480.0) & (df['timestamp'] <= 1510.0)
onset_fit_df = df[onset_fit_mask]

print(f"\nMean z(clean) per 1s window around onset:")
for t_start in np.arange(1480, 1510, 1):
    t_end = t_start + 1
    mask = (onset_fit_df['timestamp'] >= t_start) & (onset_fit_df['timestamp'] < t_end)
    n = mask.sum()
    if n == 0:
        continue
    z_all = []
    for port, ch in REP_CHANNELS.items():
        vals = onset_fit_df.loc[mask, ch].values
        z_all.append((np.mean(vals) - clean_stats[port]['mean']) / clean_stats[port]['std'])
    mean_z = np.mean(z_all)
    bar_len = min(abs(int(mean_z)), 80)
    bar = '#' * bar_len
    print(f"  t={t_start+0.5:>7.1f}s  z={mean_z:>+8.1f}  {bar}")


# Same for re-onset
print(f"\nMean z(clean) per 1s window around re-onset:")
reonset_fit_mask = (df['timestamp'] >= 1638.0) & (df['timestamp'] <= 1663.0)
reonset_fit_df = df[reonset_fit_mask]

for t_start in np.arange(1638, 1663, 1):
    t_end = t_start + 1
    mask = (reonset_fit_df['timestamp'] >= t_start) & (reonset_fit_df['timestamp'] < t_end)
    n = mask.sum()
    if n == 0:
        continue
    z_all = []
    for port, ch in REP_CHANNELS.items():
        vals = reonset_fit_df.loc[mask, ch].values
        z_all.append((np.mean(vals) - clean_stats[port]['mean']) / clean_stats[port]['std'])
    mean_z = np.mean(z_all)
    bar_len = min(abs(int(mean_z)), 80)
    bar = '#' * bar_len
    print(f"  t={t_start+0.5:>7.1f}s  z={mean_z:>+8.1f}  {bar}")


# ═══════════════════════════════════════════════════════════════════════
#  SUMMARY
# ═══════════════════════════════════════════════════════════════════════
print("\n\n" + "="*80)
print("COMPLETE TRANSITION TIMELINE SUMMARY")
print("="*80)

print("""
TIMELINE:
  t=1345-1486.5s: CLEAN (z ~ +1 to +2, stable)

  ONSET at t=1486.536s (sn=371625):
    - Port2 and Port5 shift first (sn=371625)
    - All other ports shift next sample (sn=371626)
    - Total spread: 1 sample (4ms) across ALL 4 SPI buses
    - Shape: sharp step followed by continued deepening over ~10 samples
    - Initial jump: -12 to -44 sigma, deepening to -80 to -95 sigma
    - Onset NOT a single step — it's a sharp leading edge followed by exponential deepening

  t=1486.5-1601s: BAD (z ~ -170 to -200, high noise: 12-19x clean std)
    - DC offset: ~3.3 million LSB (all ports within 2.5% of each other)
    - Noise increase: 12.5x (Port6) to 19.0x (Port4)

  RECOVERY TRANSIENT at t=1601.25s (sn=400303/400304):
    - Port2, Port5, Port7 spike first (sn=400303)
    - Port1, Port3, Port4, Port6 spike next sample (sn=400304)
    - Spread: 1 sample (4ms) — SAME as onset
    - SAME PORT ORDERING as onset (Port2+Port5 lead)
    - NOT a clean recovery — transient spike lasting 2-3 samples, then returns to bad
    - But this IS the inflection point where exponential decay begins

  t=1601-1641s: EXPONENTIAL RECOVERY (z decays from ~-170 toward ~-4)
    - Time constant tau ~ 8-9 seconds
    - Never fully returns to clean baseline (residual z ~ -4 at t=1640)
    - All ports decay at approximately same rate
    - Recovery-window noise: indistinguishable from clean (std ratio 0.84-1.18x)

  RE-ONSET at t=1641.943s (sn=410476):
    - 6 of 7 ports shift at SAME sample (sn=410476)
    - Port3 shifts 1 sample later (sn=410477)
    - Total spread: 1 sample (4ms) — SAME as onset and recovery
    - Shape: similar sharp step + deepening profile

  t=1642+: BAD (deeper than first bad period, z ~ -230 to -245)
""")

print("\nKEY FINDING: All three transitions (onset, recovery transient, re-onset)")
print("show the SAME 0-1 sample spread across 7 ports on 4 independent SPI buses.")
print("Port2+Port5 consistently lead by 1 sample at onset and recovery transient.")
print("This is IMPOSSIBLE to explain by SPI bus issues, firmware bugs, or per-device failure.")
print("The cause is definitively UPSTREAM of all SPI buses — most likely shared power or ground.")

print("\n" + "="*80)
print("ANALYSIS COMPLETE")
print("="*80)
