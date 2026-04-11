import numpy as np
import pandas as pd
import time

FILE = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\experiment_data\Nehru1_15%336Ch5_M1_eeg_20260407_110416.csv"
FS = 250

P1D8_CH1 = 58
P2D1_CH1 = 66
P3D1_CH1 = 122
P5D1_CH1 = 202
P7D1_CH1 = 282

CROSS_PORT_COLS = [P1D8_CH1, P2D1_CH1, P3D1_CH1, P5D1_CH1, P7D1_CH1]
CROSS_PORT_NAMES = ["P1D8c1", "P2D1c1", "P3D1c1", "P5D1c1", "P7D1c1"]

SEP = "=" * 80

print(SEP)
print("INDUCTOR PROBLEM ANALYSIS")
print(SEP)

t0 = time.time()
print("\nLoading CSV...")
df = pd.read_csv(FILE)
elapsed = time.time() - t0
print(f"Loaded in {elapsed:.1f}s: {df.shape[0]:,} rows x {df.shape[1]} cols")

timestamps = df.iloc[:, 0].values.astype(np.float64)
sample_nums = df.iloc[:, 1].values.astype(np.int64)
p1d8_ch1 = df.iloc[:, P1D8_CH1].values.astype(np.float64)

cross_port = {}
for col_idx, name in zip(CROSS_PORT_COLS, CROSS_PORT_NAMES):
    cross_port[name] = df.iloc[:, col_idx].values.astype(np.float64)

N = len(timestamps)

# === 1. BASIC STATS ===
print(f"\n{SEP}")
print("1. BASIC STATS")
print(SEP)

t_start = timestamps[0]
t_end = timestamps[-1]
duration = t_end - t_start
print(f"  Time range:      {t_start:.6f} -- {t_end:.6f}")
print(f"  Duration:        {duration:.3f}s  ({duration/60:.2f} min)")
print(f"  Samples:         {N:,}")
print(f"  Expected @250Hz: {int(duration * FS):,}")
print(f"  Actual rate:     {(N-1)/duration:.4f} Hz")

sample_diffs = np.diff(sample_nums)
gaps = np.where(sample_diffs != 1)[0]
print(f"\n  Sample number gaps (diff != 1): {len(gaps)}")
if len(gaps) > 0:
    for i, idx in enumerate(gaps[:10]):
        print(f"    [{idx}] sample {sample_nums[idx]} -> {sample_nums[idx+1]}  (diff={sample_diffs[idx]})")

dt = np.diff(timestamps) * 1000
print(f"\n  dt (ms):")
print(f"    mean:   {np.mean(dt):.4f}")
print(f"    std:    {np.std(dt):.4f}")
print(f"    min:    {np.min(dt):.4f}")
print(f"    max:    {np.max(dt):.4f}")
print(f"    median: {np.median(dt):.4f}")
print(f"    dt > 10ms: {np.sum(dt > 10)}")
print(f"    dt > 20ms: {np.sum(dt > 20)}")
print(f"    dt > 50ms: {np.sum(dt > 50)}")
for p in [1, 5, 25, 75, 95, 99]:
    print(f"    P{p:02d}:    {np.percentile(dt, p):.4f} ms")

# === 2. P1D8 TIMELINE ===
print(f"\n{SEP}")
print("2. P1D8_CH1 TIMELINE (30-second windows)")
print(SEP)

window_sec = 30
window_samples = window_sec * FS
n_windows = N // window_samples

print(f"  Window size: {window_sec}s = {window_samples} samples")
print(f"  Number of complete windows: {n_windows}")
print(f"\n  {'Win':>4} {'Time(s)':>10} {'Mean':>14} {'Std':>12} {'Min':>14} {'Max':>14}")
print("  " + "-" * 72)

window_means = []
window_stds = []
window_times = []

for w in range(n_windows):
    s_idx = w * window_samples
    e_idx = (w + 1) * window_samples
    seg = p1d8_ch1[s_idx:e_idx]
    t_mid = timestamps[s_idx + window_samples // 2] - t_start
    m = np.mean(seg)
    sd = np.std(seg)
    mn = np.min(seg)
    mx = np.max(seg)
    window_means.append(m)
    window_stds.append(sd)
    window_times.append(t_mid)
    print(f"  {w:>4d} {t_mid:>10.1f} {m:>14.1f} {sd:>12.1f} {mn:>14.1f} {mx:>14.1f}")

window_means = np.array(window_means)
window_stds = np.array(window_stds)

mean_range = np.max(window_means) - np.min(window_means)
std_range = np.max(window_stds) / (np.min(window_stds) + 1e-10)
print(f"\n  Mean range across windows: {mean_range:.1f} LSB")
print(f"  Std ratio (max/min):       {std_range:.2f}x")
print(f"  Mean of stds:              {np.mean(window_stds):.1f}")

if mean_range > 100000:
    print("  ** WARNING: Large mean shifts detected **")
else:
    print("  OK: No large mean shifts detected")
if std_range > 10:
    print("  ** WARNING: Noise floor varies >10x **")
else:
    print(f"  OK: Noise floor ratio {std_range:.2f}x is within normal range")

# === 3. CROSS-PORT COMMON-MODE ===
print(f"\n{SEP}")
print("3. CROSS-PORT COMMON-MODE CHECK")
print(SEP)

cp_matrix = np.column_stack([cross_port[n] for n in CROSS_PORT_NAMES])
grand_mean = np.mean(cp_matrix, axis=1)
grand_mean_diff = np.diff(grand_mean)
big_shifts = np.where(np.abs(grand_mean_diff) > 100000)[0]
print(f"\n  Grand mean single-sample jumps > 100k LSB: {len(big_shifts)}")

if len(big_shifts) > 0:
    print("  ** INDUCTOR SIGNATURE: Simultaneous cross-port shifts **")
    for idx in big_shifts[:20]:
        t_rel = timestamps[idx] - t_start
        shift = grand_mean_diff[idx]
        print(f"    t={t_rel:.3f}s  sample={sample_nums[idx]}  shift={shift:.0f} LSB")
else:
    print("  OK: No simultaneous cross-port shifts >100k LSB")

print(f"\n  Per-channel single-sample jumps > 100k LSB:")
for name in CROSS_PORT_NAMES:
    ch_diff = np.abs(np.diff(cross_port[name]))
    n_big = np.sum(ch_diff > 100000)
    print(f"    {name}: {n_big}")

corr_window = 60 * FS
n_corr_windows = N // corr_window
print(f"\n  Cross-port correlation (60s windows, P1D8c1 vs others):")
print(f"  {'Win':>4} {'Time(s)':>8} {'P2D1c1':>8} {'P3D1c1':>8} {'P5D1c1':>8} {'P7D1c1':>8}")
print("  " + "-" * 50)

for w in range(min(n_corr_windows, 30)):
    si = w * corr_window
    ei = si + corr_window
    ref = cross_port["P1D8c1"][si:ei]
    corrs = []
    for cname in ["P2D1c1", "P3D1c1", "P5D1c1", "P7D1c1"]:
        ch = cross_port[cname][si:ei]
        c = np.corrcoef(ref, ch)[0, 1]
        corrs.append(c)
    t_mid = timestamps[si + corr_window // 2] - t_start
    print(f"  {w:>4d} {t_mid:>8.1f} {corrs[0]:>8.3f} {corrs[1]:>8.3f} {corrs[2]:>8.3f} {corrs[3]:>8.3f}")

# === 4. SPIKE DETECTION ===
print(f"\n{SEP}")
print("4. SPIKE DETECTION (P1D8_CH1 single-sample jumps)")
print(SEP)

p1d8_diff = np.diff(p1d8_ch1)
abs_diff = np.abs(p1d8_diff)

for thresh in [50000, 100000, 500000, 1000000, 3000000]:
    n_spikes = np.sum(abs_diff > thresh)
    print(f"  Jumps > {thresh:>10,} LSB: {n_spikes}")

spike_50k = np.where(abs_diff > 50000)[0]
print(f"\n  Total spikes > 50k LSB: {len(spike_50k)}")

if len(spike_50k) > 0:
    print(f"\n  First 30 spikes > 50k LSB:")
    print(f"  {'Num':>4} {'Time(s)':>10} {'Sample':>10} {'Before':>14} {'After':>14} {'Jump':>14}")
    print("  " + "-" * 70)
    for i, idx in enumerate(spike_50k[:30]):
        t_rel = timestamps[idx] - t_start
        print(f"  {i:>4d} {t_rel:>10.3f} {sample_nums[idx]:>10d} {p1d8_ch1[idx]:>14.0f} {p1d8_ch1[idx+1]:>14.0f} {p1d8_diff[idx]:>14.0f}")

    if len(spike_50k) >= 3:
        spike_times = timestamps[spike_50k] - t_start
        isis = np.diff(spike_times)
        print(f"\n  Inter-spike intervals (first 20):")
        for i, isi in enumerate(isis[:20]):
            print(f"    spike {i} -> {i+1}: {isi:.3f}s")
        if len(isis) >= 2:
            fh = isis[:len(isis)//2]
            sh = isis[len(isis)//2:]
            if len(fh) > 0 and len(sh) > 0:
                print(f"\n  Mean ISI first half:  {np.mean(fh):.3f}s")
                print(f"  Mean ISI second half: {np.mean(sh):.3f}s")
                if np.mean(sh) < np.mean(fh) * 0.5:
                    print("  ** WARNING: Spikes ACCELERATING (thermal cycling pattern) **")
                else:
                    print("  OK: No clear acceleration pattern")
else:
    print("  OK: Zero spikes > 50k LSB on P1D8_CH1")

# === 5. NOISE FLOOR OVER TIME ===
print(f"\n{SEP}")
print("5. NOISE FLOOR OVER TIME (P1D8_CH1 diff_std, 30s windows)")
print(SEP)

print(f"\n  {'Win':>4} {'Time(s)':>10} {'diff_std':>12} {'diff_max':>12}")
print("  " + "-" * 44)

noise_stds = []
noise_times = []

for w in range(n_windows):
    si = w * window_samples
    ei = (w + 1) * window_samples
    seg_diff = np.diff(p1d8_ch1[si:ei])
    ds = np.std(seg_diff)
    dm = np.max(np.abs(seg_diff))
    t_mid = timestamps[si + window_samples // 2] - t_start
    noise_stds.append(ds)
    noise_times.append(t_mid)
    print(f"  {w:>4d} {t_mid:>10.1f} {ds:>12.1f} {dm:>12.1f}")

noise_stds = np.array(noise_stds)
noise_times = np.array(noise_times)

noise_ratio = np.max(noise_stds) / np.min(noise_stds)
print(f"\n  Noise floor ratio (max/min diff_std): {noise_ratio:.2f}x")

if n_windows >= 3:
    ns, ni = np.polyfit(noise_times, noise_stds, 1)
    print(f"  Noise floor linear trend: slope = {ns:.4f} LSB/s")
    print(f"  Over full recording: {ns * duration:.1f} LSB change")
    if abs(ns * duration) > np.mean(noise_stds) * 0.5:
        print("  ** WARNING: Noise floor significantly trending **")
    else:
        print("  OK: Noise floor is stable")

# === 6. DRIFT ===
print(f"\n{SEP}")
print("6. DRIFT ANALYSIS (P1D8_CH1)")
print(SEP)

t_rel_all = timestamps - t_start
slope, intercept = np.polyfit(t_rel_all, p1d8_ch1, 1)
total_drift = slope * duration

print(f"  Linear fit: y = {slope:.2f} * t + {intercept:.1f}")
print(f"  Drift rate:     {slope:.2f} LSB/s")
print(f"  Total drift:    {total_drift:.0f} LSB over {duration:.1f}s")
mean_val = np.mean(p1d8_ch1)
pct = abs(total_drift) / (abs(mean_val) + 1e-10) * 100
print(f"  Drift pct of mean: {pct:.4f}%")

coeffs2 = np.polyfit(t_rel_all, p1d8_ch1, 2)
print(f"\n  Quadratic fit: a={coeffs2[0]:.4f}, b={coeffs2[1]:.2f}, c={coeffs2[2]:.1f}")
print(f"  Quadratic term over recording: {abs(coeffs2[0]) * duration**2:.0f} LSB")

print(f"\n  P1D8_CH1 overall statistics:")
print(f"    Mean:   {np.mean(p1d8_ch1):.1f}")
print(f"    Std:    {np.std(p1d8_ch1):.1f}")
print(f"    Min:    {np.min(p1d8_ch1):.0f}")
print(f"    Max:    {np.max(p1d8_ch1):.0f}")
print(f"    Range:  {np.max(p1d8_ch1) - np.min(p1d8_ch1):.0f}")

# === 7. VERDICT ===
print(f"\n{SEP}")
print("7. INDUCTOR PROBLEM VERDICT")
print(SEP)

flags = []
if len(big_shifts) > 0:
    flags.append(f"CROSS-PORT SHIFTS: {len(big_shifts)} simultaneous jumps >100k LSB")
if len(spike_50k) > 0:
    flags.append(f"SPIKES: {len(spike_50k)} single-sample jumps >50k LSB on P1D8c1")
if noise_ratio > 5:
    flags.append(f"NOISE INSTABILITY: diff_std varies {noise_ratio:.1f}x")
if mean_range > 100000:
    flags.append(f"MEAN SHIFT: {mean_range:.0f} LSB range across 30s windows")

if len(spike_50k) >= 4:
    st_check = timestamps[spike_50k] - t_start
    isi_check = np.diff(st_check)
    if len(isi_check) >= 4:
        fq = isi_check[:len(isi_check)//4]
        lq = isi_check[-len(isi_check)//4:]
        if len(fq) > 0 and len(lq) > 0 and np.mean(lq) < np.mean(fq) * 0.3:
            flags.append("ACCELERATING SPIKES: classic thermal cycling pattern")

if len(flags) == 0:
    print("\n  *** NO SIGNS OF THE INDUCTOR PROBLEM ***")
    print("  Recording is clean w.r.t. power supply destabilization.")
    print("  - No simultaneous cross-port DC shifts")
    print("  - No large single-sample transients")
    print("  - Stable noise floor throughout recording")
else:
    print(f"\n  *** {len(flags)} WARNING FLAG(S) DETECTED ***")
    for f in flags:
        print(f"  - {f}")
    print("\n  Further investigation recommended.")

print(f"\n{SEP}")
print("Analysis complete.")
print(SEP)
