"""
Common-Average Reference (CAR) re-referencing test
===================================================
Subtract the mean-across-good-channels from every channel, then recompute the
336x336 Pearson correlation matrix and all grouping stats. Compare against
baseline (no re-reference) to determine if the shared supply/reference
common-mode identified in stage-3 coherence analysis can be removed by CAR.

Filter chain (exact order, matches channel_similarity.py):
  1. scipy.signal.detrend(axis=0, type='linear')
  2. 1 Hz HPF (10th-order Butterworth SOS, sosfiltfilt)
  3. 50 Hz LPF (10th-order Butterworth SOS, sosfiltfilt)
  4. 60 Hz notch (iirnotch Q=30, filtfilt)
  5. 70 Hz notch (iirnotch Q=30, filtfilt)

CAR step: after filtering, subtract the per-sample mean across GOOD channels
only. All 336 channels (good and bad) have the same common_avg subtracted.

Outputs (all prefixed `rereference_car_`):
  - rereference_car.py                        (this script)
  - rereference_car_correlation.npy
  - rereference_car_results.txt
  - rereference_car_correlation_matrix.png
  - rereference_car_distance_decay.png
  - rereference_car_port_pair_matrix.png
  - rereference_car_similarity_histogram.png
  - rereference_car_unique_info.png
"""
import os
import re
import time
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal as sig

# ----------------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------------
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
BAD_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis\bad_channels.txt"
BASELINE_CORR = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis\correlation_matrix.npy"
OUT_DIR = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client"

FS = 250.0

# Scalp order (left -> right), user-confirmed 2026-04-10
SCALP_ORDER = [4, 3, 2, 1, 7, 6, 5]  # P4, P3, P2, P1, P7, P6, P5
SCALP_RANK = {p: i for i, p in enumerate(SCALP_ORDER)}  # P4->0, ..., P5->6

# Baseline numbers from distance_analysis.txt
BASELINE = {
    'within_device': 0.7577,
    'within_port_cross_dev': 0.6370,
    'cross_port_rank1': 0.5423,
    'cross_port_rank2': 0.5173,
    'cross_port_rank3': 0.5301,
    'cross_port_rank4': 0.5365,
    'cross_port_rank5': 0.4809,
    'cross_port_rank6': 0.6223,
    'per_port_within_dev': {
        4: 0.7053, 3: 0.6758, 2: 0.7021, 1: 0.6616,
        7: 0.6294, 6: 0.4755, 5: 0.7408,
    },
    'uniq_median': 0.1749,
    'uniq_q75': 0.2848,
    'uniq_frac_gt25': 0.331,
}

# ----------------------------------------------------------------------------
# Load CSV
# ----------------------------------------------------------------------------
print(f"[load] Reading {CSV_PATH} ...")
t0 = time.time()
header = pd.read_csv(CSV_PATH, nrows=0).columns.tolist()
ch_cols = [c for c in header if c.startswith('Port')]
dtypes = {c: np.float32 for c in ch_cols}
if 'sample_number' in header:
    dtypes['sample_number'] = np.int64
df = pd.read_csv(CSV_PATH, dtype=dtypes)
n_samples = len(df)
N = len(ch_cols)
print(f"[load] {n_samples:,} rows x {N} channels ({time.time()-t0:.1f}s)")
print(f"[load] Duration: {n_samples/FS:.1f}s ({n_samples/FS/60:.2f} min)")

# ----------------------------------------------------------------------------
# Parse channel metadata
# ----------------------------------------------------------------------------
ch_info = []
for col in ch_cols:
    parts = col.split('_')
    port = int(parts[0].replace('Port', ''))
    dev = int(parts[1].replace('dev', ''))
    ch = int(parts[2].replace('ch', ''))
    ch_info.append({'col': col, 'port': port, 'dev': dev, 'ch': ch,
                    'device_id': f'P{port}D{dev}'})
port_id = np.array([c['port'] for c in ch_info], dtype=np.int16)
device_idx = np.array([c['device_id'] for c in ch_info])
unique_devices = list(dict.fromkeys(device_idx.tolist()))
print(f"[parse] {len(unique_devices)} devices, ports: {sorted(set(port_id.tolist()))}")

# ----------------------------------------------------------------------------
# Parse bad channel list
# ----------------------------------------------------------------------------
print(f"[bad ] Parsing {BAD_PATH}")
bad_names = set()
name_re = re.compile(r'^\s*(Port\d+_dev\d+_ch\d+)\s+std=')
with open(BAD_PATH, 'r', encoding='utf-8') as f:
    for line in f:
        m = name_re.match(line)
        if m:
            bad_names.add(m.group(1))
print(f"[bad ] Found {len(bad_names)} bad channel names in file")

bad_mask = np.array([c in bad_names for c in ch_cols], dtype=bool)
n_bad = int(bad_mask.sum())
n_good = N - n_bad
print(f"[bad ] Masked {n_bad}/{N} bad channels, {n_good} good")

# ----------------------------------------------------------------------------
# Extract raw + filter
# ----------------------------------------------------------------------------
print("[load] Extracting raw matrix ...")
t0 = time.time()
data = df[ch_cols].to_numpy(dtype=np.float32, copy=True)
print(f"[load] Raw matrix: {data.shape} ({data.nbytes/1e6:.1f} MB, {time.time()-t0:.1f}s)")

print("[filt] Linear detrend ...")
t0 = time.time()
data_filt = sig.detrend(data, axis=0, type='linear').astype(np.float32, copy=False)
print(f"[filt] Detrend done ({time.time()-t0:.1f}s)")

print("[filt] 1 Hz HPF (Butter10 SOS, sosfiltfilt) ...")
t0 = time.time()
hpf_sos = sig.butter(10, 1.0, btype='high', fs=FS, output='sos')
data_filt = sig.sosfiltfilt(hpf_sos, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] HPF done ({time.time()-t0:.1f}s)")

print("[filt] 50 Hz LPF (Butter10 SOS, sosfiltfilt) ...")
t0 = time.time()
lpf_sos = sig.butter(10, 50.0, btype='low', fs=FS, output='sos')
data_filt = sig.sosfiltfilt(lpf_sos, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] LPF done ({time.time()-t0:.1f}s)")

print("[filt] 60 Hz notch (filtfilt) ...")
t0 = time.time()
b60, a60 = sig.iirnotch(60.0, Q=30.0, fs=FS)
data_filt = sig.filtfilt(b60, a60, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] 60 Hz done ({time.time()-t0:.1f}s)")

print("[filt] 70 Hz notch (filtfilt) ...")
t0 = time.time()
b70, a70 = sig.iirnotch(70.0, Q=30.0, fs=FS)
data_filt = sig.filtfilt(b70, a70, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] 70 Hz done ({time.time()-t0:.1f}s)")

del data

# ----------------------------------------------------------------------------
# CAR re-reference: subtract mean across GOOD channels from every channel
# ----------------------------------------------------------------------------
print("[CAR ] Computing common average across good channels ...")
t0 = time.time()
good_idx = np.where(~bad_mask)[0]
# common_avg: shape (n_samples,)
common_avg = data_filt[:, good_idx].mean(axis=1).astype(np.float32)
print(f"[CAR ] common_avg: shape={common_avg.shape}, std={common_avg.std():.2f} uV ({time.time()-t0:.1f}s)")

print("[CAR ] Subtracting common_avg from all 336 channels ...")
t0 = time.time()
# Broadcast subtraction: (N_samples, 336) - (N_samples, 1)
data_car = data_filt - common_avg[:, None]
print(f"[CAR ] CAR applied ({time.time()-t0:.1f}s)")

# ----------------------------------------------------------------------------
# Correlation matrix
# ----------------------------------------------------------------------------
print("[corr] Computing 336x336 Pearson correlation (CAR data) ...")
t0 = time.time()
corr_car = np.corrcoef(data_car.T)
print(f"[corr] NaN entries: {int(np.isnan(corr_car).sum())} ({time.time()-t0:.1f}s)")
np.save(os.path.join(OUT_DIR, 'rereference_car_correlation.npy'),
        corr_car.astype(np.float32))
print(f"[corr] Saved -> rereference_car_correlation.npy")

# Load baseline for comparison plots / stats
print("[base] Loading baseline correlation matrix ...")
corr_base = np.load(BASELINE_CORR).astype(np.float64)
print(f"[base] baseline shape: {corr_base.shape}")

# ----------------------------------------------------------------------------
# Grouping: upper triangle with rank-distance, bad-excluded
# ----------------------------------------------------------------------------
print("[group] Building upper-triangle indices ...")
iu, ju = np.triu_indices(N, k=1)
same_port = port_id[iu] == port_id[ju]
same_dev = device_idx[iu] == device_idx[ju]
good_pair = (~bad_mask[iu]) & (~bad_mask[ju])

# Rank distance in scalp order
rank_arr = np.array([SCALP_RANK[int(p)] for p in port_id], dtype=np.int16)
rank_dist = np.abs(rank_arr[iu] - rank_arr[ju])  # 0..6

vals_car = corr_car[iu, ju]
vals_base = corr_base[iu, ju]

def group_stats(vals, mask, base_vals):
    v = vals[mask]
    b = base_vals[mask]
    v = v[np.isfinite(v)]
    b = b[np.isfinite(b)]
    return {
        'n': int(len(v)),
        'mean': float(v.mean()) if len(v) else np.nan,
        'median': float(np.median(v)) if len(v) else np.nan,
        'std': float(v.std()) if len(v) else np.nan,
        'abs_mean': float(np.abs(v).mean()) if len(v) else np.nan,
        'base_mean': float(b.mean()) if len(b) else np.nan,
        'base_median': float(np.median(b)) if len(b) else np.nan,
    }

within_device_mask = good_pair & same_port & same_dev
within_port_mask = good_pair & same_port & (~same_dev)
cross_port_mask = good_pair & (~same_port)

g_within_device = group_stats(vals_car, within_device_mask, vals_base)
g_within_port = group_stats(vals_car, within_port_mask, vals_base)
g_cross_port = group_stats(vals_car, cross_port_mask, vals_base)

# per-rank cross-port
rank_stats = {}
for r in range(1, 7):
    m = good_pair & (~same_port) & (rank_dist == r)
    rank_stats[r] = group_stats(vals_car, m, vals_base)

# ----------------------------------------------------------------------------
# Per-port within-device means
# ----------------------------------------------------------------------------
per_port_within_dev_car = {}
for p in SCALP_ORDER:
    m = good_pair & (port_id[iu] == p) & (port_id[ju] == p) & same_dev
    v = vals_car[m]
    v = v[np.isfinite(v)]
    per_port_within_dev_car[p] = float(v.mean()) if len(v) else np.nan

# ----------------------------------------------------------------------------
# 7x7 port-pair mean r matrix (scalp order P4 P3 P2 P1 P7 P6 P5)
# ----------------------------------------------------------------------------
port_pair_mat_car = np.full((7, 7), np.nan, dtype=np.float64)
port_pair_abs_mat_car = np.full((7, 7), np.nan, dtype=np.float64)
port_pair_n = np.zeros((7, 7), dtype=np.int64)
for a_i, pa in enumerate(SCALP_ORDER):
    for b_i, pb in enumerate(SCALP_ORDER):
        if pa == pb:
            m = good_pair & (port_id[iu] == pa) & (port_id[ju] == pb) & (~same_dev)
        else:
            m = good_pair & (
                ((port_id[iu] == pa) & (port_id[ju] == pb)) |
                ((port_id[iu] == pb) & (port_id[ju] == pa))
            )
        v = vals_car[m]
        v = v[np.isfinite(v)]
        if len(v):
            port_pair_mat_car[a_i, b_i] = v.mean()
            port_pair_abs_mat_car[a_i, b_i] = np.abs(v).mean()
            port_pair_n[a_i, b_i] = len(v)

# ----------------------------------------------------------------------------
# Per-channel unique information vs nearest neighbor (CAR)
# ----------------------------------------------------------------------------
print("[uniq] Computing per-channel unique information (CAR) ...")
# For each good channel, find its max |r| with any OTHER good channel, then
# unique = 1 - r_max^2
corr_good = corr_car[np.ix_(good_idx, good_idx)].copy()
np.fill_diagonal(corr_good, np.nan)
# use signed r (baseline used signed r to pick max) — but with CAR, strongest
# relationship can be anti-phase. Follow baseline convention: max by |r|, then
# unique = 1 - r_max^2.
abs_corr_good = np.abs(corr_good)
r_max_abs = np.nanmax(abs_corr_good, axis=1)
r_max_idx = np.nanargmax(abs_corr_good, axis=1)
r_max_signed = corr_good[np.arange(len(good_idx)), r_max_idx]
unique_info = 1.0 - r_max_abs ** 2
uniq_median = float(np.median(unique_info))
uniq_q25 = float(np.quantile(unique_info, 0.25))
uniq_q75 = float(np.quantile(unique_info, 0.75))
uniq_frac_gt25 = float((unique_info >= 0.25).mean())
uniq_frac_gt50 = float((unique_info >= 0.50).mean())

# ----------------------------------------------------------------------------
# Negative correlations after CAR
# ----------------------------------------------------------------------------
neg_pairs_cross = int(((vals_car[cross_port_mask] < 0)).sum())
total_cross = int(cross_port_mask.sum())
neg_pairs_within_port = int(((vals_car[within_port_mask] < 0)).sum())
total_within_port = int(within_port_mask.sum())
neg_pairs_within_dev = int(((vals_car[within_device_mask] < 0)).sum())
total_within_dev = int(within_device_mask.sum())

# ----------------------------------------------------------------------------
# Write results text
# ----------------------------------------------------------------------------
RES_PATH = os.path.join(OUT_DIR, 'rereference_car_results.txt')
print(f"[out ] Writing {RES_PATH}")

def fmt_delta(base, new):
    return f"{new - base:+.4f}"

with open(RES_PATH, 'w', encoding='utf-8') as f:
    f.write("Common-Average Reference (CAR) vs baseline -- eeg_data_2026-04-10_200737.csv\n")
    f.write(f"Channels: {N} (bad={n_bad}, good={n_good})\n")
    f.write(f"Samples: {n_samples:,} @ {FS:g} Hz ({n_samples/FS:.1f}s)\n")
    f.write("Filter chain: detrend -> 1 Hz HPF -> 50 Hz LPF -> 60 Hz notch -> 70 Hz notch\n")
    f.write("CAR: mean across all good channels subtracted from every channel.\n\n")

    f.write("=" * 78 + "\n")
    f.write("BASELINE vs CAR\n")
    f.write("=" * 78 + "\n")
    f.write(f"{'Group':<32s} | {'Baseline':>9s} | {'CAR':>9s} | {'Delta':>9s}\n")
    f.write("-" * 78 + "\n")
    rows = [
        ('within-device',                BASELINE['within_device'],        g_within_device['mean']),
        ('within-port cross-dev',        BASELINE['within_port_cross_dev'], g_within_port['mean']),
        ('cross-port rank=1 (adjacent)', BASELINE['cross_port_rank1'],     rank_stats[1]['mean']),
        ('cross-port rank=2',            BASELINE['cross_port_rank2'],     rank_stats[2]['mean']),
        ('cross-port rank=3',            BASELINE['cross_port_rank3'],     rank_stats[3]['mean']),
        ('cross-port rank=4',            BASELINE['cross_port_rank4'],     rank_stats[4]['mean']),
        ('cross-port rank=5',            BASELINE['cross_port_rank5'],     rank_stats[5]['mean']),
        ('cross-port rank=6 (P4<->P5)',  BASELINE['cross_port_rank6'],     rank_stats[6]['mean']),
    ]
    for name, base, new in rows:
        f.write(f"{name:<32s} | {base:>+9.4f} | {new:>+9.4f} | {fmt_delta(base, new):>9s}\n")

    f.write("\n")
    f.write("Cross-port absolute-value means (|r|, not signed)\n")
    f.write("-" * 78 + "\n")
    f.write(f"  all cross-port         signed mean = {g_cross_port['mean']:+.4f}   |r| mean = {g_cross_port['abs_mean']:.4f}\n")
    for r in range(1, 7):
        rs = rank_stats[r]
        f.write(f"  cross-port rank={r}       signed mean = {rs['mean']:+.4f}   |r| mean = {rs['abs_mean']:.4f}   (n={rs['n']})\n")

    f.write("\n")
    f.write("=" * 78 + "\n")
    f.write("Per-port within-device mean r (baseline -> CAR)\n")
    f.write("=" * 78 + "\n")
    for p in SCALP_ORDER:
        base = BASELINE['per_port_within_dev'][p]
        new = per_port_within_dev_car[p]
        f.write(f"  P{p}: {base:+.4f} -> {new:+.4f}   (delta {fmt_delta(base, new)})\n")

    f.write("\n")
    f.write("=" * 78 + "\n")
    f.write("Port-pair mean r (7x7, scalp-ordered, CAR, diagonal = within-port cross-dev)\n")
    f.write("=" * 78 + "\n")
    f.write("         " + "  ".join([f"   P{p}  " for p in SCALP_ORDER]) + "\n")
    for i, pa in enumerate(SCALP_ORDER):
        row = f"  P{pa}   "
        for j, pb in enumerate(SCALP_ORDER):
            v = port_pair_mat_car[i, j]
            row += f"  {v:+7.4f}"
        f.write(row + "\n")

    f.write("\n")
    f.write("Port-pair |r| matrix (absolute values)\n")
    f.write("-" * 78 + "\n")
    f.write("         " + "  ".join([f"   P{p}  " for p in SCALP_ORDER]) + "\n")
    for i, pa in enumerate(SCALP_ORDER):
        row = f"  P{pa}   "
        for j, pb in enumerate(SCALP_ORDER):
            v = port_pair_abs_mat_car[i, j]
            row += f"  {v:+7.4f}"
        f.write(row + "\n")

    f.write("\n")
    f.write("=" * 78 + "\n")
    f.write("Per-channel unique information after CAR (good channels only)\n")
    f.write("=" * 78 + "\n")
    f.write(f"  n                          = {len(good_idx)}\n")
    f.write(f"  median                     = {uniq_median:.4f}   (baseline {BASELINE['uniq_median']:.4f}, delta {uniq_median-BASELINE['uniq_median']:+.4f})\n")
    f.write(f"  Q25 / Q50 / Q75            = {uniq_q25:.4f} / {uniq_median:.4f} / {uniq_q75:.4f}   (baseline Q75 {BASELINE['uniq_q75']:.4f})\n")
    f.write(f"  >25% unique                = {int(uniq_frac_gt25*len(good_idx))}/{len(good_idx)} ({uniq_frac_gt25*100:.1f}%)   (baseline {BASELINE['uniq_frac_gt25']*100:.1f}%)\n")
    f.write(f"  >50% unique                = {int(uniq_frac_gt50*len(good_idx))}/{len(good_idx)} ({uniq_frac_gt50*100:.1f}%)\n")

    f.write("\n")
    f.write("=" * 78 + "\n")
    f.write("Negative correlations after CAR\n")
    f.write("=" * 78 + "\n")
    f.write(f"  within-device pairs:  {neg_pairs_within_dev:6d}/{total_within_dev:6d}  ({100*neg_pairs_within_dev/total_within_dev:.1f}%)\n")
    f.write(f"  within-port/cross-dev:{neg_pairs_within_port:6d}/{total_within_port:6d}  ({100*neg_pairs_within_port/total_within_port:.1f}%)\n")
    f.write(f"  cross-port:           {neg_pairs_cross:6d}/{total_cross:6d}  ({100*neg_pairs_cross/total_cross:.1f}%)\n")

print(f"[out ] {RES_PATH} written")

# ----------------------------------------------------------------------------
# Plots
# ----------------------------------------------------------------------------

# helper: build port boundary indices on the existing channel index order
port_boundaries = []
prev = port_id[0]
for i in range(1, N):
    if port_id[i] != prev:
        port_boundaries.append(i)
        prev = port_id[i]
port_centers = []
start = 0
for b in port_boundaries + [N]:
    port_centers.append((start + b - 1) / 2)
    start = b
port_labels = [f'P{p}' for p in sorted(set(port_id.tolist()))]

# ----- Plot 1: side-by-side baseline vs CAR full 336x336 heatmap -------------
print("[plot] rereference_car_correlation_matrix.png ...")
t0 = time.time()
fig, axes = plt.subplots(1, 2, figsize=(20, 9))
for ax, mat, title in [
    (axes[0], corr_base, 'Baseline (no re-reference)'),
    (axes[1], corr_car, 'After CAR'),
]:
    m = mat.copy()
    m[~np.isfinite(m)] = 0.0
    im = ax.imshow(m, vmin=-1, vmax=1, cmap='RdBu_r',
                   aspect='equal', interpolation='nearest')
    for b in port_boundaries:
        ax.axhline(b - 0.5, color='white', linewidth=1.3)
        ax.axvline(b - 0.5, color='white', linewidth=1.3)
    ax.set_xticks(port_centers)
    ax.set_yticks(port_centers)
    ax.set_xticklabels(port_labels)
    ax.set_yticklabels(port_labels)
    ax.set_title(title, fontsize=12)
    plt.colorbar(im, ax=ax, fraction=0.04, pad=0.02).set_label('Pearson r')
fig.suptitle(f'336 x 336 correlation matrix  -  {n_good} good channels  -  CAR = mean(good) subtracted',
             fontsize=13)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'rereference_car_correlation_matrix.png'), dpi=140)
plt.close()
print(f"[plot]   done ({time.time()-t0:.1f}s)")

# ----- Plot 2: distance decay baseline vs CAR --------------------------------
print("[plot] rereference_car_distance_decay.png ...")
t0 = time.time()
bins_lbl = ['within-dev', 'within-port', 'rank=1', 'rank=2', 'rank=3',
            'rank=4', 'rank=5', 'rank=6']
base_bins = [
    BASELINE['within_device'], BASELINE['within_port_cross_dev'],
    BASELINE['cross_port_rank1'], BASELINE['cross_port_rank2'],
    BASELINE['cross_port_rank3'], BASELINE['cross_port_rank4'],
    BASELINE['cross_port_rank5'], BASELINE['cross_port_rank6'],
]
car_bins = [
    g_within_device['mean'], g_within_port['mean'],
    rank_stats[1]['mean'], rank_stats[2]['mean'], rank_stats[3]['mean'],
    rank_stats[4]['mean'], rank_stats[5]['mean'], rank_stats[6]['mean'],
]
car_bins_abs = [
    np.abs(vals_car[within_device_mask]).mean(),
    np.abs(vals_car[within_port_mask]).mean(),
    rank_stats[1]['abs_mean'], rank_stats[2]['abs_mean'], rank_stats[3]['abs_mean'],
    rank_stats[4]['abs_mean'], rank_stats[5]['abs_mean'], rank_stats[6]['abs_mean'],
]

fig, ax = plt.subplots(figsize=(11, 6))
x = np.arange(len(bins_lbl))
ax.plot(x, base_bins, marker='o', linewidth=2, color='#333333', label='Baseline signed mean r')
ax.plot(x, car_bins, marker='s', linewidth=2, color='#d62728', label='CAR signed mean r')
ax.plot(x, car_bins_abs, marker='^', linewidth=2, linestyle='--', color='#1f77b4', label='CAR |r| mean')
ax.axhline(0, color='gray', linewidth=0.6)
ax.set_xticks(x)
ax.set_xticklabels(bins_lbl)
ax.set_ylabel('Mean Pearson r')
ax.set_title('Distance-decay: baseline vs CAR '
             '(CAR reveals anti-phase residuals at long scalp distance)')
ax.grid(alpha=0.3)
ax.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'rereference_car_distance_decay.png'), dpi=140)
plt.close()
print(f"[plot]   done ({time.time()-t0:.1f}s)")

# ----- Plot 3: 7x7 port-pair matrix ------------------------------------------
print("[plot] rereference_car_port_pair_matrix.png ...")
t0 = time.time()
fig, axes = plt.subplots(1, 2, figsize=(14, 6))
for ax, mat, title in [
    (axes[0], port_pair_mat_car, 'CAR signed mean r'),
    (axes[1], port_pair_abs_mat_car, 'CAR |r| mean'),
]:
    im = ax.imshow(mat, vmin=-0.5, vmax=0.5 if title.startswith('CAR signed') else 0.5,
                   cmap='RdBu_r', aspect='equal')
    ax.set_xticks(range(7)); ax.set_yticks(range(7))
    labels = [f'P{p}' for p in SCALP_ORDER]
    ax.set_xticklabels(labels); ax.set_yticklabels(labels)
    ax.set_title(title, fontsize=11)
    for i in range(7):
        for j in range(7):
            v = mat[i, j]
            if np.isfinite(v):
                col = 'white' if abs(v) > 0.35 else 'black'
                ax.text(j, i, f'{v:+.3f}', ha='center', va='center',
                        fontsize=8, color=col)
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
fig.suptitle('Port-pair correlation matrix after CAR (scalp order P4-P3-P2-P1-P7-P6-P5)',
             fontsize=12)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'rereference_car_port_pair_matrix.png'), dpi=140)
plt.close()
print(f"[plot]   done ({time.time()-t0:.1f}s)")

# ----- Plot 4: similarity histogram ------------------------------------------
print("[plot] rereference_car_similarity_histogram.png ...")
t0 = time.time()
within_dev_vals = vals_car[within_device_mask]
within_port_vals = vals_car[within_port_mask]
cross_port_vals = vals_car[cross_port_mask]
within_dev_vals = within_dev_vals[np.isfinite(within_dev_vals)]
within_port_vals = within_port_vals[np.isfinite(within_port_vals)]
cross_port_vals = cross_port_vals[np.isfinite(cross_port_vals)]

fig, ax = plt.subplots(figsize=(11, 6))
bins = np.linspace(-1, 1, 121)
ax.hist(cross_port_vals, bins=bins, alpha=0.5, color='#777777',
        label=f'cross-port (n={len(cross_port_vals):,}, mu={cross_port_vals.mean():+.3f})',
        density=True)
ax.hist(within_port_vals, bins=bins, alpha=0.55, color='#d9822b',
        label=f'within-port/cross-dev (n={len(within_port_vals):,}, mu={within_port_vals.mean():+.3f})',
        density=True)
ax.hist(within_dev_vals, bins=bins, alpha=0.6, color='#2b7bd9',
        label=f'within-device (n={len(within_dev_vals):,}, mu={within_dev_vals.mean():+.3f})',
        density=True)
for v, c in [(cross_port_vals, '#333333'),
             (within_port_vals, '#a65200'),
             (within_dev_vals, '#0b4e9c')]:
    ax.axvline(np.median(v), color=c, linestyle='--', linewidth=1.2)
ax.axvline(0, color='black', linewidth=0.6, alpha=0.7)
ax.set_xlim(-1, 1)
ax.set_xlabel('Pearson r (after CAR)')
ax.set_ylabel('Density')
ax.set_title('Channel similarity by group after CAR (dashed lines = medians)')
ax.legend(loc='upper left', fontsize=9)
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'rereference_car_similarity_histogram.png'), dpi=140)
plt.close()
print(f"[plot]   done ({time.time()-t0:.1f}s)")

# ----- Plot 5: unique info hist with baseline overlay ------------------------
print("[plot] rereference_car_unique_info.png ...")
t0 = time.time()
# Recompute baseline unique info the same way for overlay
corr_base_good = corr_base[np.ix_(good_idx, good_idx)].copy()
np.fill_diagonal(corr_base_good, np.nan)
abs_base = np.abs(corr_base_good)
r_max_base = np.nanmax(abs_base, axis=1)
uniq_base = 1.0 - r_max_base ** 2

fig, ax = plt.subplots(figsize=(11, 6))
bins = np.linspace(0, 1, 41)
ax.hist(uniq_base, bins=bins, alpha=0.55, color='#777777',
        label=f'Baseline (median={np.median(uniq_base):.3f}, >25%: {100*(uniq_base>=0.25).mean():.1f}%)')
ax.hist(unique_info, bins=bins, alpha=0.55, color='#d62728',
        label=f'CAR (median={uniq_median:.3f}, >25%: {100*uniq_frac_gt25:.1f}%)')
ax.axvline(np.median(uniq_base), color='#333333', linestyle='--', linewidth=1.2)
ax.axvline(uniq_median, color='#8b0000', linestyle='--', linewidth=1.2)
ax.axvline(0.25, color='green', linestyle=':', linewidth=1, alpha=0.6, label='25% threshold')
ax.set_xlabel('Unique information = 1 - r_max^2   (higher = more distinct from nearest neighbor)')
ax.set_ylabel('Channels')
ax.set_title('Per-channel unique information: baseline vs CAR')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'rereference_car_unique_info.png'), dpi=140)
plt.close()
print(f"[plot]   done ({time.time()-t0:.1f}s)")

# ----------------------------------------------------------------------------
# Final console summary
# ----------------------------------------------------------------------------
print("\n" + "=" * 78)
print("CAR RESULTS SUMMARY")
print("=" * 78)
print(f"Within-device:        {BASELINE['within_device']:+.4f} -> {g_within_device['mean']:+.4f}  (delta {g_within_device['mean']-BASELINE['within_device']:+.4f})")
print(f"Within-port cross-dv: {BASELINE['within_port_cross_dev']:+.4f} -> {g_within_port['mean']:+.4f}  (delta {g_within_port['mean']-BASELINE['within_port_cross_dev']:+.4f})")
for r in range(1, 7):
    base = BASELINE[f'cross_port_rank{r}']
    new = rank_stats[r]['mean']
    print(f"Cross-port rank={r}:     {base:+.4f} -> {new:+.4f}  (delta {new-base:+.4f})   |r|={rank_stats[r]['abs_mean']:.4f}")
print(f"Cross-port all:       signed mean = {g_cross_port['mean']:+.4f}   |r| mean = {g_cross_port['abs_mean']:.4f}")
print(f"P4<->P5 rank=6:       {BASELINE['cross_port_rank6']:+.4f} -> {rank_stats[6]['mean']:+.4f}")
print(f"Unique info median:   {BASELINE['uniq_median']:.4f} -> {uniq_median:.4f}")
print(f"Channels >=25% uniq:  {BASELINE['uniq_frac_gt25']*100:.1f}% -> {uniq_frac_gt25*100:.1f}%")
print(f"Negative cross-port:  {100*neg_pairs_cross/total_cross:.1f}%")
print("=" * 78)
print("[done] Artifacts:")
for fn in ['rereference_car_correlation.npy',
           'rereference_car_results.txt',
           'rereference_car_correlation_matrix.png',
           'rereference_car_distance_decay.png',
           'rereference_car_port_pair_matrix.png',
           'rereference_car_similarity_histogram.png',
           'rereference_car_unique_info.png']:
    p = os.path.join(OUT_DIR, fn)
    if os.path.exists(p):
        sz = os.path.getsize(p) / 1024.0
        print(f"  {p}  ({sz:.1f} KB)")
