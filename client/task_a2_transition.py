"""
Task A.2: Transition Dynamics Analysis (t=1478-1483s)
Analyzes the onset characteristics of the destabilization event.
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Load data ──
print("Loading data...")
df = pd.read_csv(r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv")
ts = df['timestamp'].values
sn = df['sample_number'].values
print(f"Loaded {len(df)} rows, timestamps {ts[0]:.3f} - {ts[-1]:.3f}s")

# ── Select channels (one per SPI bus + extra) ──
channels = {
    'Port1_dev1_ch1': 'SPI0-P1D1',
    'Port1_dev8_ch1': 'SPI0-P1D8',
    'Port3_dev1_ch1': 'SPI3-P3D1',
    'Port5_dev1_ch1': 'SPI4-P5D1',
    'Port7_dev1_ch1': 'SPI5-P7D1',
}

# ── First: scan a wide range to find where the shift actually starts ──
print("\n--- SCANNING FOR ONSET (wide range t=1470-1620s) ---")
mask_scan = (ts >= 1470.0) & (ts <= 1620.0)
idx_scan = np.where(mask_scan)[0]
scan_col = 'Port1_dev1_ch1'
scan_data = df[scan_col].values[idx_scan].astype(np.float64)
scan_baseline = df[scan_col].values[np.where((ts >= 1400.0) & (ts <= 1470.0))[0]].astype(np.float64)
scan_bl_mean = np.mean(scan_baseline)
scan_bl_std = np.std(scan_baseline)
scan_thresh = 10.0 * scan_bl_std
scan_dev = np.abs(scan_data - scan_bl_mean)
scan_exceed = scan_dev > scan_thresh
if np.any(scan_exceed):
    scan_first = np.argmax(scan_exceed)
    scan_global = idx_scan[scan_first]
    print(f"  First 10x-std exceedance on {scan_col}:")
    print(f"  sample #{sn[scan_global]}, t={ts[scan_global]:.6f}s, val={df[scan_col].values[scan_global]:,}")
    print(f"  deviation = {scan_dev[scan_first]:,.0f} LSB = {scan_dev[scan_first]/scan_bl_std:.1f}x std")
    # Set transition window around this point
    onset_time = ts[scan_global]
    win_start = onset_time - 5.0
    win_end = onset_time + 5.0
    print(f"  Setting transition window: t={win_start:.1f} - {win_end:.1f}s")
else:
    print("  ** NO 10x-std EXCEEDANCE FOUND in t=1470-1620s -- using default window **")
    win_start = 1476.0
    win_end = 1485.0

# ── Extract transition window ──
mask_window = (ts >= win_start) & (ts <= win_end)
idx_window = np.where(mask_window)[0]
print(f"\nTransition window: {idx_window[0]}..{idx_window[-1]} ({len(idx_window)} samples)")
print(f"  Time range: {ts[idx_window[0]]:.6f} - {ts[idx_window[-1]]:.6f}s")

# ── Baseline: clean region t=1400-1470 for computing std ──
mask_baseline = (ts >= 1400.0) & (ts <= 1470.0)
idx_baseline = np.where(mask_baseline)[0]
print(f"Baseline window: {len(idx_baseline)} samples (t=1400-1470s)")

# ── Analyze each channel ──
print("\n" + "="*100)
print("TRANSITION DYNAMICS ANALYSIS")
print("="*100)

fig, axes = plt.subplots(5, 3, figsize=(20, 18))
fig.suptitle("Task A.2: Transition Dynamics (t=1476-1485s)", fontsize=14, fontweight='bold')

onset_results = {}

for i, (col, label) in enumerate(channels.items()):
    data_window = df[col].values[idx_window].astype(np.float64)
    data_baseline = df[col].values[idx_baseline].astype(np.float64)
    ts_window = ts[idx_window]
    sn_window = sn[idx_window]

    # Baseline stats
    bl_mean = np.mean(data_baseline)
    bl_std = np.std(data_baseline)

    # First derivative (diff)
    d1 = np.diff(data_window)
    # Second derivative
    d2 = np.diff(d1)

    # ── Onset detection: first sample exceeding 10x clean std from baseline mean ──
    deviation = np.abs(data_window - bl_mean)
    threshold = 10.0 * bl_std
    exceed_mask = deviation > threshold

    if np.any(exceed_mask):
        onset_idx_local = np.argmax(exceed_mask)  # first True
        onset_idx_global = idx_window[onset_idx_local]
        onset_ts = ts[onset_idx_global]
        onset_sn = sn[onset_idx_global]
        onset_val = data_window[onset_idx_local]
        onset_dev = deviation[onset_idx_local]
    else:
        onset_idx_local = None
        onset_ts = None
        onset_sn = None

    # ── Determine transition type ──
    if onset_idx_local is not None:
        # Final offset: mean of last 500 samples in window
        n_tail = min(500, len(data_window) - onset_idx_local)
        final_val = np.mean(data_window[-n_tail:])
        final_offset = final_val - bl_mean
        target_90 = bl_mean + 0.9 * final_offset

        # Find when we reach 90% of final offset
        if final_offset > 0:
            reach_90_mask = data_window[onset_idx_local:] >= target_90
        else:
            reach_90_mask = data_window[onset_idx_local:] <= target_90

        if np.any(reach_90_mask):
            t90_local = onset_idx_local + np.argmax(reach_90_mask)
            n_samples_to_90 = t90_local - onset_idx_local
            time_constant = (ts_window[min(t90_local, len(ts_window)-1)] - ts_window[onset_idx_local])
        else:
            n_samples_to_90 = len(data_window) - onset_idx_local
            time_constant = ts_window[-1] - ts_window[onset_idx_local]

        # Classify transition
        if n_samples_to_90 < 5:
            transition_type = "STEP"
        elif n_samples_to_90 > 100:
            transition_type = "RAMP"
        else:
            # Check for oscillation
            d1_post_onset = d1[onset_idx_local:] if onset_idx_local < len(d1) else d1[-10:]
            n_check = min(50, len(d1_post_onset))
            sign_changes_early = np.sum(np.diff(np.sign(d1_post_onset[:n_check])) != 0)
            if sign_changes_early > 10:
                transition_type = "OSCILLATION (ringing)"
            else:
                transition_type = f"INTERMEDIATE ({n_samples_to_90} samples)"

        max_d1 = np.max(np.abs(d1))
        max_d1_idx = np.argmax(np.abs(d1))
    else:
        final_offset = 0
        n_samples_to_90 = 0
        time_constant = 0
        transition_type = "NO SHIFT DETECTED"
        max_d1 = 0
        max_d1_idx = 0

    onset_results[label] = {
        'channel': col,
        'onset_sample': onset_sn,
        'onset_timestamp': onset_ts,
        'onset_idx_local': onset_idx_local,
        'baseline_mean': bl_mean,
        'baseline_std': bl_std,
        'threshold_10x': threshold,
        'final_offset': final_offset,
        'n_samples_to_90': n_samples_to_90,
        'time_constant_s': time_constant,
        'transition_type': transition_type,
        'max_d1': max_d1,
    }

    print(f"\n--- {label} ({col}) ---")
    print(f"  Baseline mean: {bl_mean:,.0f} LSB  |  std: {bl_std:,.1f} LSB")
    print(f"  10x std threshold: {threshold:,.0f} LSB")
    if onset_ts is not None:
        print(f"  ONSET: sample #{onset_sn}, timestamp {onset_ts:.6f}s")
        print(f"  Onset value: {onset_val:,.0f} LSB  (deviation: {onset_dev:,.0f} LSB = {onset_dev/bl_std:.1f}x std)")
        print(f"  Final offset: {final_offset:,.0f} LSB")
        print(f"  Samples to 90%: {n_samples_to_90}  |  Time constant: {time_constant*1000:.1f}ms")
        print(f"  Transition type: {transition_type}")
        print(f"  Max |d1|: {max_d1:,.0f} LSB/sample at local idx {max_d1_idx}")
    else:
        print(f"  ** NO ONSET DETECTED in this window **")

    # ── Plot ──
    ax0 = axes[i, 0]
    ax0.plot(ts_window, data_window, 'b-', linewidth=0.5, alpha=0.8)
    ax0.axhline(bl_mean, color='g', linestyle='--', alpha=0.5, label='baseline mean')
    ax0.axhline(bl_mean + threshold, color='r', linestyle=':', alpha=0.5, label='+10x std')
    ax0.axhline(bl_mean - threshold, color='r', linestyle=':', alpha=0.5)
    if onset_ts is not None:
        ax0.axvline(onset_ts, color='r', linewidth=1.5, alpha=0.7, label=f'onset t={onset_ts:.3f}s')
    ax0.set_title(f'{label} - Raw Signal')
    ax0.set_xlabel('Time (s)')
    ax0.set_ylabel('ADC codes (LSB)')
    ax0.legend(fontsize=7)
    ax0.ticklabel_format(style='sci', axis='y', scilimits=(-3,3))

    ax1 = axes[i, 1]
    ax1.plot(ts_window[1:], d1, 'r-', linewidth=0.5, alpha=0.8)
    if onset_ts is not None:
        ax1.axvline(onset_ts, color='k', linewidth=1.5, alpha=0.7)
    ax1.set_title(f'{label} - 1st Derivative')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('d(ADC)/dt (LSB/sample)')
    ax1.ticklabel_format(style='sci', axis='y', scilimits=(-3,3))

    ax2 = axes[i, 2]
    ax2.plot(ts_window[2:], d2, 'm-', linewidth=0.5, alpha=0.8)
    if onset_ts is not None:
        ax2.axvline(onset_ts, color='k', linewidth=1.5, alpha=0.7)
    ax2.set_title(f'{label} - 2nd Derivative')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel("d2(ADC)/dt2 (LSB/sample^2)")
    ax2.ticklabel_format(style='sci', axis='y', scilimits=(-3,3))

plt.tight_layout()
outpath = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\task_a2_transition.png"
plt.savefig(outpath, dpi=150, bbox_inches='tight')
print(f"\nPlot saved to {outpath}")

# ── Summary table ──
print("\n" + "="*100)
print("SUMMARY: ONSET TIMING ACROSS ALL CHANNELS")
print("="*100)
print(f"{'Channel':<20} {'Sample#':>10} {'Timestamp':>14} {'Offset (LSB)':>15} {'Samples->90%':>12} {'Tau (ms)':>10} {'Type':<25}")
print("-"*110)
for label, r in onset_results.items():
    sn_str = f"{r['onset_sample']}" if r['onset_sample'] is not None else "N/A"
    ts_str = f"{r['onset_timestamp']:.6f}" if r['onset_timestamp'] is not None else "N/A"
    print(f"{label:<20} {sn_str:>10} {ts_str:>14} {r['final_offset']:>15,.0f} {r['n_samples_to_90']:>12} {r['time_constant_s']*1000:>10.1f} {r['transition_type']:<25}")

# ── Simultaneity check ──
print("\n" + "="*100)
print("SIMULTANEITY CHECK")
print("="*100)
onset_times = [r['onset_timestamp'] for r in onset_results.values() if r['onset_timestamp'] is not None]
onset_samples = [r['onset_sample'] for r in onset_results.values() if r['onset_sample'] is not None]
if len(onset_times) > 1:
    spread_ms = (max(onset_times) - min(onset_times)) * 1000
    spread_samples = max(onset_samples) - min(onset_samples)
    print(f"Onset time spread: {spread_ms:.3f}ms ({spread_samples} samples)")
    print(f"Earliest onset: sample {min(onset_samples)} at t={min(onset_times):.6f}s")
    print(f"Latest onset:   sample {max(onset_samples)} at t={max(onset_times):.6f}s")
    if spread_samples <= 1:
        print(">> ALL CHANNELS SHIFT WITHIN 1 SAMPLE (4ms) -- truly simultaneous")
    elif spread_samples <= 5:
        print(">> ALL CHANNELS SHIFT WITHIN 5 SAMPLES (20ms) -- near-simultaneous")
    else:
        print(f">> SPREAD OF {spread_samples} SAMPLES -- check for per-port staggering")

# ── Zoom into exact onset ──
print("\n" + "="*100)
print("ZOOM: 10 SAMPLES AROUND ONSET (Port1_dev1_ch1)")
print("="*100)
col0 = 'Port1_dev1_ch1'
r0 = onset_results['SPI0-P1D1']
if r0['onset_idx_local'] is not None:
    center = idx_window[0] + r0['onset_idx_local']
    for j in range(max(0, center-5), min(len(df), center+6)):
        marker = " <<<< ONSET" if j == center else ""
        dev_from_bl = df[col0].values[j] - r0['baseline_mean']
        print(f"  sample {sn[j]}, t={ts[j]:.6f}s, val={df[col0].values[j]:>12,}, dev={dev_from_bl:>12,.0f} LSB{marker}")

# ── Also zoom all 5 channels at onset ──
print("\n" + "="*100)
print("ZOOM: ALL 5 CHANNELS, 5 SAMPLES BEFORE AND AFTER EARLIEST ONSET")
print("="*100)
earliest_sn = min(onset_samples) if onset_samples else None
if earliest_sn is not None:
    earliest_global_idx = np.where(sn == earliest_sn)[0][0]
    for j in range(max(0, earliest_global_idx - 5), min(len(df), earliest_global_idx + 6)):
        vals = {}
        for col, label in channels.items():
            r = onset_results[label]
            dev = df[col].values[j] - r['baseline_mean']
            vals[label] = dev
        marker = ""
        if sn[j] == earliest_sn:
            marker = " <<<< EARLIEST ONSET"
        parts = "  ".join([f"{lbl}:{v:>12,.0f}" for lbl, v in vals.items()])
        print(f"  sample {sn[j]}, t={ts[j]:.6f}s  {parts}{marker}")

plt.close('all')
print("\nTask A.2 complete.")
