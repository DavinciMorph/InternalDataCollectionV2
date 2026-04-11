#!/usr/bin/env python3
"""
Diagnostic analysis for intermittent ~2.9M LSB additive DC offset bug.

Reads FirmwareIssue1.csv and runs six analyses:
  A.1: Cross-port onset timing
  A.2: Transition dynamics (onset + recovery)
  A.3: Spectral analysis (PSD)
  A.4: Railed & intermediate channel mapping
  A.5: Full correlation matrix (clean vs bad)
  A.6: Clean-region drift

Usage:
    python analyze_offset_bug.py [path_to_csv]
    Default: client/FirmwareIssue1.csv (relative to script dir)
"""

import sys
import os
import numpy as np
import pandas as pd
from pathlib import Path

# Sampling rate
FS = 250  # Hz

# Port-to-SPI-bus mapping
PORT_SPI_BUS = {
    'Port1': 'SPI0', 'Port2': 'SPI0',
    'Port3': 'SPI3', 'Port4': 'SPI3',
    'Port5': 'SPI4', 'Port6': 'SPI4',
    'Port7': 'SPI5',
}

# Port device counts
PORT_DEVICES = {
    'Port1': 8, 'Port2': 7, 'Port3': 5, 'Port4': 5,
    'Port5': 5, 'Port6': 5, 'Port7': 7,
}

# ADS1299 24-bit signed rail values
ADC_MIN = -8388608
ADC_MAX = 8388607


def load_csv(path):
    """Load CSV, return DataFrame with timestamp index."""
    print(f"Loading {path}...")
    df = pd.read_csv(path)
    print(f"  {len(df)} samples, {len(df.columns) - 2} channels")
    print(f"  Time range: {df['timestamp'].iloc[0]:.3f} - {df['timestamp'].iloc[-1]:.3f} s")
    return df


def get_channel_columns(df):
    """Return list of channel column names (excluding timestamp and sample_number)."""
    return [c for c in df.columns if c.startswith('Port')]


def parse_channel_name(name):
    """Parse 'Port1_dev2_ch3' -> ('Port1', 2, 3)."""
    parts = name.split('_')
    port = parts[0]
    dev = int(parts[1].replace('dev', ''))
    ch = int(parts[2].replace('ch', ''))
    return port, dev, ch


def pick_representative_channels(ch_cols):
    """Pick one non-railed channel per port (dev1_ch1)."""
    reps = {}
    for c in ch_cols:
        port, dev, ch = parse_channel_name(c)
        if dev == 1 and ch == 1:
            reps[port] = c
    return reps


def find_onset_sample(series, baseline_mask, sigma_threshold=10):
    """Find first sample where first-derivative exceeds sigma_threshold * baseline_std."""
    diff = series.diff().values
    baseline_diff = diff[baseline_mask[:-1] if len(baseline_mask) > len(diff) else baseline_mask[:len(diff)]]
    baseline_diff = baseline_diff[np.isfinite(baseline_diff)]
    if len(baseline_diff) < 10:
        return None
    mu = np.mean(baseline_diff)
    sigma = np.std(baseline_diff)
    if sigma < 1e-10:
        return None
    threshold = mu + sigma_threshold * sigma
    neg_threshold = mu - sigma_threshold * sigma

    for i in range(len(diff)):
        if not baseline_mask[min(i, len(baseline_mask) - 1)]:
            if diff[i] > threshold or diff[i] < neg_threshold:
                return i
    return None


# =========================================================================
# A.1: Cross-Port Onset Timing
# =========================================================================
def analysis_a1(df, ch_cols):
    print("\n" + "=" * 70)
    print("A.1: CROSS-PORT ONSET TIMING")
    print("=" * 70)

    reps = pick_representative_channels(ch_cols)

    # Clean baseline: t=1345-1475s
    t = df['timestamp'].values
    baseline_mask = (t >= 1345) & (t <= 1475)

    onset_samples = {}
    for port in sorted(reps.keys()):
        col = reps[port]
        series = df[col]
        onset = find_onset_sample(series, baseline_mask)
        if onset is not None:
            onset_samples[port] = onset
            onset_time = t[onset] if onset < len(t) else float('nan')
            print(f"  {port}: onset at sample {onset} (t={onset_time:.4f}s)")
        else:
            print(f"  {port}: no clear onset detected")

    if len(onset_samples) >= 2:
        min_onset = min(onset_samples.values())
        print(f"\n  Sample offsets relative to first onset (sample {min_onset}):")
        for port in sorted(onset_samples.keys()):
            delta = onset_samples[port] - min_onset
            bus = PORT_SPI_BUS.get(port, '?')
            print(f"    {port} ({bus}): +{delta} samples")

        spread = max(onset_samples.values()) - min_onset
        print(f"\n  Total spread: {spread} samples ({spread / FS * 1000:.1f} ms)")
        if spread <= 1:
            print("  >> ALL PORTS SHIFT WITHIN 0-1 SAMPLES")
            print("  >> Cause is UPSTREAM of all SPI buses (power/ground)")
        elif spread <= 5:
            print("  >> Tight spread — likely analog ground/supply bounce")
        else:
            print("  >> Significant spread — may indicate cascading failure")

    return onset_samples


# =========================================================================
# A.2: Transition Dynamics
# =========================================================================
def analysis_a2(df, ch_cols):
    print("\n" + "=" * 70)
    print("A.2: TRANSITION DYNAMICS (onset + recovery)")
    print("=" * 70)

    t = df['timestamp'].values
    reps = pick_representative_channels(ch_cols)

    for label, t_start, t_end in [("ONSET", 1478, 1483), ("RECOVERY", 1604, 1610)]:
        mask = (t >= t_start) & (t <= t_end)
        if mask.sum() < 10:
            print(f"\n  {label}: insufficient data in t={t_start}-{t_end}s (only {mask.sum()} samples)")
            continue

        print(f"\n  --- {label} (t={t_start}-{t_end}s, {mask.sum()} samples) ---")

        for port in sorted(reps.keys()):
            col = reps[port]
            segment = df.loc[mask, col].values
            diff = np.diff(segment)
            abs_diff = np.abs(diff)
            max_diff_idx = np.argmax(abs_diff)
            max_diff_val = diff[max_diff_idx]

            # Classify: count how many samples have |derivative| > 10% of peak
            peak = abs_diff.max()
            if peak < 1:
                print(f"    {port}: flat (no transition)")
                continue

            active = np.sum(abs_diff > peak * 0.1)
            total_range = segment.max() - segment.min()

            if active <= 5:
                kind = "STEP (<5 active samples -- digital event)"
            elif active > 100:
                kind = "RAMP (>100 active samples -- analog drift)"
            else:
                kind = f"TRANSITION ({active} active samples)"

            # Check for oscillation: sign changes in derivative
            sign_changes = np.sum(np.diff(np.sign(diff)) != 0)
            if sign_changes > active * 0.5 and active > 10:
                kind += " + OSCILLATION (feedback loop?)"

            print(f"    {port}: {kind}")
            print(f"           range={total_range:.0f} LSB, peak_deriv={max_diff_val:.0f} LSB/sample, "
                  f"active={active} samples")


# =========================================================================
# A.3: Spectral Analysis (PSD)
# =========================================================================
def analysis_a3(df, ch_cols):
    print("\n" + "=" * 70)
    print("A.3: SPECTRAL ANALYSIS (PSD)")
    print("=" * 70)

    t = df['timestamp'].values
    reps = pick_representative_channels(ch_cols)

    # Pick 3-5 representative channels across different SPI buses
    rep_ports = ['Port1', 'Port3', 'Port5', 'Port7']
    rep_cols = [reps[p] for p in rep_ports if p in reps]

    clean_mask = (t >= 1400) & (t <= 1470)
    bad_mask = (t >= 1500) & (t <= 1590)

    try:
        from scipy.signal import welch, iirnotch, lfilter
    except ImportError:
        print("  scipy not available — skipping PSD analysis")
        return

    # 60 Hz + 120 Hz notch filter coefficients (Q=30)
    notch60_b, notch60_a = iirnotch(60.0, 30.0, FS)
    notch120_b, notch120_a = iirnotch(120.0, 30.0, FS)

    def apply_powerline_notch(x):
        y = lfilter(notch60_b, notch60_a, x)
        y = lfilter(notch120_b, notch120_a, y)
        return y

    for col in rep_cols:
        port = col.split('_')[0]
        print(f"\n  --- {port} ({col}) ---")

        for label, mask in [("CLEAN", clean_mask), ("BAD", bad_mask)]:
            data = df.loc[mask, col].values.astype(np.float64)
            if len(data) < 256:
                print(f"    {label}: insufficient samples ({len(data)})")
                continue

            # Raw PSD
            freqs, psd_raw = welch(data, fs=FS, nperseg=min(256, len(data)),
                                   noverlap=min(128, len(data) // 2))

            # Filtered PSD (60+120 Hz notch)
            filtered = apply_powerline_notch(data)
            _, psd_filt = welch(filtered, fs=FS, nperseg=min(256, len(data)),
                                noverlap=min(128, len(data) // 2))

            total_power_raw = np.trapezoid(psd_raw, freqs)
            total_power_filt = np.trapezoid(psd_filt, freqs)
            idx_50 = np.argmin(np.abs(freqs - 50))
            idx_60 = np.argmin(np.abs(freqs - 60))
            idx_70 = np.argmin(np.abs(freqs - 70))
            peak_freq = freqs[np.argmax(psd_raw)]
            peak_power = psd_raw.max()

            print(f"    {label} (raw):  total={total_power_raw:.2e}, peak={peak_freq:.1f}Hz ({peak_power:.2e})")
            print(f"           50Hz={psd_raw[idx_50]:.2e}, 60Hz={psd_raw[idx_60]:.2e}, 70Hz={psd_raw[idx_70]:.2e}")
            print(f"    {label} (60Hz notched): total={total_power_filt:.2e}, "
                  f"60Hz={psd_filt[idx_60]:.2e} ({psd_raw[idx_60]/max(psd_filt[idx_60],1):.0f}x reduction)")

        # Noise character: compare clean vs bad
        clean_data = df.loc[clean_mask, col].values.astype(np.float64)
        bad_data = df.loc[bad_mask, col].values.astype(np.float64)
        if len(clean_data) > 10 and len(bad_data) > 10:
            clean_std = np.std(clean_data - np.mean(clean_data))
            bad_std = np.std(bad_data - np.mean(bad_data))
            print(f"    Noise: clean_std={clean_std:.1f} LSB, bad_std={bad_std:.1f} LSB, "
                  f"ratio={bad_std / clean_std:.1f}x" if clean_std > 0 else
                  f"    Noise: clean_std={clean_std:.1f}, bad_std={bad_std:.1f}")


# =========================================================================
# A.4: Railed & Intermediate Channel Mapping
# =========================================================================
def analysis_a4(df, ch_cols):
    print("\n" + "=" * 70)
    print("A.4: RAILED & INTERMEDIATE CHANNEL MAPPING")
    print("=" * 70)

    t = df['timestamp'].values
    clean_mask = (t >= 1400) & (t <= 1470)
    bad_mask = (t >= 1500) & (t <= 1590)

    classifications = {'normal_shift': [], 'railed': [], 'intermediate': [], 'no_shift': []}
    offsets = []

    for col in ch_cols:
        clean_mean = df.loc[clean_mask, col].values.astype(np.float64).mean()
        bad_mean = df.loc[bad_mask, col].values.astype(np.float64).mean()
        shift = bad_mean - clean_mean

        # Check if channel was already railed in clean period
        clean_vals = df.loc[clean_mask, col].values
        is_railed_clean = (np.abs(clean_vals - ADC_MIN).mean() < 1000 or
                           np.abs(clean_vals - ADC_MAX).mean() < 1000)

        if is_railed_clean and abs(shift) < 100000:
            classifications['railed'].append((col, clean_mean, bad_mean, shift))
        elif abs(shift) > 2000000:  # ~2M+ LSB shift
            # Check if it's clamped at rail
            expected_bad = clean_mean + np.median(offsets) if offsets else clean_mean + shift
            if bad_mean <= ADC_MIN + 1000 or bad_mean >= ADC_MAX - 1000:
                classifications['intermediate'].append((col, clean_mean, bad_mean, shift))
            else:
                classifications['normal_shift'].append((col, clean_mean, bad_mean, shift))
                offsets.append(shift)
        elif abs(shift) < 50000:
            classifications['no_shift'].append((col, clean_mean, bad_mean, shift))
        else:
            # Moderate shift — might be clamped
            classifications['intermediate'].append((col, clean_mean, bad_mean, shift))

    # Report summary
    print(f"\n  Normal shift (~2.9M LSB): {len(classifications['normal_shift'])} channels")
    print(f"  Already railed (no shift): {len(classifications['railed'])} channels")
    print(f"  Intermediate (clamped):    {len(classifications['intermediate'])} channels")
    print(f"  No shift (<50k):           {len(classifications['no_shift'])} channels")

    # Offset statistics for normal-shift channels
    if offsets:
        offsets = np.array(offsets)
        print(f"\n  Normal-shift offset stats:")
        print(f"    Mean:   {np.mean(offsets):.0f} LSB ({np.mean(offsets) * 0.0223e-3:.3f} mV at gain=24)")
        print(f"    Std:    {np.std(offsets):.0f} LSB")
        print(f"    Min:    {np.min(offsets):.0f} LSB")
        print(f"    Max:    {np.max(offsets):.0f} LSB")
        print(f"    Spread: {np.max(offsets) - np.min(offsets):.0f} LSB")

    # Check intermediate channels for clamping
    if classifications['intermediate']:
        print(f"\n  Intermediate channels (checking for rail clamping):")
        median_offset = np.median(offsets) if len(offsets) > 0 else 0
        for col, cm, bm, shift in classifications['intermediate']:
            port, dev, ch = parse_channel_name(col)
            predicted = cm + median_offset
            clamped = np.clip(predicted, ADC_MIN, ADC_MAX)
            match = abs(bm - clamped) < 50000
            rail_hit = "MIN" if bm <= ADC_MIN + 1000 else ("MAX" if bm >= ADC_MAX - 1000 else "none")
            print(f"    {col}: clean={cm:.0f}, bad={bm:.0f}, predicted={clamped:.0f}, "
                  f"rail={rail_hit}, clamp_match={'YES' if match else 'NO'}")

    # Map railed channels by port/device
    if classifications['railed']:
        print(f"\n  Railed channels by port:")
        railed_by_port = {}
        for col, cm, bm, shift in classifications['railed']:
            port, dev, ch = parse_channel_name(col)
            railed_by_port.setdefault(port, []).append(f"dev{dev}_ch{ch}")
        for port in sorted(railed_by_port.keys()):
            print(f"    {port}: {len(railed_by_port[port])} channels — {', '.join(railed_by_port[port][:5])}"
                  + ("..." if len(railed_by_port[port]) > 5 else ""))

    return classifications, offsets


# =========================================================================
# A.5: Full Correlation Matrix
# =========================================================================
def analysis_a5(df, ch_cols):
    print("\n" + "=" * 70)
    print("A.5: FULL CORRELATION MATRIX (clean vs bad)")
    print("=" * 70)

    t = df['timestamp'].values

    # Use 5-second windows
    clean_mask = (t >= 1430) & (t <= 1435)
    bad_mask = (t >= 1540) & (t <= 1545)

    # Pick non-railed channels (skip channels near rails in clean period)
    good_cols = []
    for col in ch_cols:
        vals = df.loc[clean_mask, col].values.astype(np.float64)
        if abs(vals.mean() - ADC_MIN) > 500000 and abs(vals.mean() - ADC_MAX) > 500000:
            good_cols.append(col)

    if len(good_cols) < 10:
        print(f"  Only {len(good_cols)} non-railed channels — insufficient for correlation analysis")
        return

    print(f"  Using {len(good_cols)} non-railed channels")

    for label, mask in [("CLEAN", clean_mask), ("BAD", bad_mask)]:
        # AC-couple (mean-subtract each channel)
        data = df.loc[mask, good_cols].values.astype(np.float64)
        data = data - data.mean(axis=0)

        # Pearson correlation
        corr = np.corrcoef(data.T)

        # Compute within-device, within-port, cross-port means
        within_dev = []
        within_port = []
        cross_port = []
        cross_bus = []

        for i in range(len(good_cols)):
            pi, di, ci = parse_channel_name(good_cols[i])
            for j in range(i + 1, len(good_cols)):
                pj, dj, cj = parse_channel_name(good_cols[j])
                r = corr[i, j]
                if not np.isfinite(r):
                    continue
                if pi == pj and di == dj:
                    within_dev.append(r)
                elif pi == pj:
                    within_port.append(r)
                else:
                    cross_port.append(r)
                    if PORT_SPI_BUS.get(pi) != PORT_SPI_BUS.get(pj):
                        cross_bus.append(r)

        print(f"\n  {label} correlations:")
        for name, vals in [("Within-device", within_dev),
                           ("Within-port (cross-device)", within_port),
                           ("Cross-port", cross_port),
                           ("Cross-bus (cross-SPI)", cross_bus)]:
            if vals:
                arr = np.array(vals)
                print(f"    {name}: mean={arr.mean():.6f}, std={arr.std():.6f}, "
                      f"min={arr.min():.6f}, max={arr.max():.6f} (n={len(vals)})")
            else:
                print(f"    {name}: no pairs")

    if cross_bus:
        bad_cross_bus = np.array(cross_bus)
        if bad_cross_bus.mean() > 0.999:
            print("\n  >> Cross-bus r in BAD region ~1.0")
            print("  >> DEFINITIVELY power supply / ground issue")


# =========================================================================
# A.6: Clean-Region Drift
# =========================================================================
def analysis_a6(df, ch_cols):
    print("\n" + "=" * 70)
    print("A.6: CLEAN-REGION DRIFT")
    print("=" * 70)

    t = df['timestamp'].values
    clean_mask = (t >= 1345) & (t <= 1475)
    t_clean = t[clean_mask]

    if clean_mask.sum() < 100:
        print("  Insufficient clean-region data")
        return

    # Compute linear trend per channel
    slopes = {}
    for col in ch_cols:
        vals = df.loc[clean_mask, col].values.astype(np.float64)
        if np.std(vals) < 1:
            continue
        # Linear regression: slope in LSB/s
        coeffs = np.polyfit(t_clean, vals, 1)
        slopes[col] = coeffs[0]  # LSB/s

    if not slopes:
        print("  No channels with measurable drift")
        return

    slope_vals = np.array(list(slopes.values()))
    print(f"  Channels with drift: {len(slopes)}")
    print(f"  Drift rate (LSB/s): mean={np.mean(slope_vals):.1f}, std={np.std(slope_vals):.1f}")
    print(f"    min={np.min(slope_vals):.1f}, max={np.max(slope_vals):.1f}")

    # Check if drift is correlated (all channels drift together)
    positive_drift = np.sum(slope_vals > 0)
    negative_drift = np.sum(slope_vals < 0)
    print(f"\n  Drift direction: {positive_drift} positive, {negative_drift} negative")

    # Per-port mean drift
    port_drifts = {}
    for col, slope in slopes.items():
        port = col.split('_')[0]
        port_drifts.setdefault(port, []).append(slope)

    print(f"\n  Per-port mean drift (LSB/s):")
    for port in sorted(port_drifts.keys()):
        vals = np.array(port_drifts[port])
        print(f"    {port}: mean={np.mean(vals):.1f}, std={np.std(vals):.1f} ({len(vals)} channels)")

    # Cross-channel drift correlation
    if len(slopes) >= 2:
        rep_cols = list(slopes.keys())[:20]  # First 20 for efficiency
        drift_data = df.loc[clean_mask, rep_cols].values.astype(np.float64)
        # Detrend each and compute pairwise correlation of trends
        corr = np.corrcoef(drift_data.T)
        upper = corr[np.triu_indices_from(corr, k=1)]
        upper = upper[np.isfinite(upper)]
        if len(upper) > 0:
            print(f"\n  Cross-channel drift correlation (first 20 channels):")
            print(f"    mean r={np.mean(upper):.4f}, std={np.std(upper):.4f}")
            if np.mean(upper) > 0.9:
                print("    >> High drift correlation — systematic (not electrode-specific)")
            elif np.mean(upper) < 0.3:
                print("    >> Low drift correlation — independent (electrode impedance)")


# =========================================================================
# Main
# =========================================================================
def main():
    # Determine CSV path
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        script_dir = Path(__file__).parent
        csv_path = script_dir / "FirmwareIssue1.csv"

    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found")
        sys.exit(1)

    df = load_csv(csv_path)
    ch_cols = get_channel_columns(df)

    print(f"\n{len(ch_cols)} channel columns detected")
    print(f"Ports: {sorted(set(c.split('_')[0] for c in ch_cols))}")

    # Run all analyses
    onset_samples = analysis_a1(df, ch_cols)
    analysis_a2(df, ch_cols)
    analysis_a3(df, ch_cols)
    classifications, offsets = analysis_a4(df, ch_cols)
    analysis_a5(df, ch_cols)
    analysis_a6(df, ch_cols)

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print("If all of the following are true, the root cause is analog ground/supply shift:")
    print("  1. A.1: All ports shift within 0-1 samples")
    print("  2. A.2: Onset is a ramp (>100 samples), not a step")
    print("  3. A.4: Intermediate channels are exactly 'normal shift clamped at rail'")
    print("  4. A.5: Cross-bus correlation ~1.0 in bad region")
    print("  5. A.6: Clean-region drift is systematic (not electrode-specific)")
    print()


if __name__ == '__main__':
    main()
