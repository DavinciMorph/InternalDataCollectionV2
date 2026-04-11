"""
Re-reference test: subtract Port1_dev8_ch3 from every channel.

Goal: determine whether the broadband cross-port coupling (mean r ~= 0.53)
identified in prior analysis is driven by a globally-shared common-mode signal
in the analog front-end. If so, subtracting one channel that carries the full
common-mode component should collapse cross-port correlations.

Parallel companion analysis uses CAR; this one uses a single reference channel.

Outputs (all prefixed rereference_p1d8ch3_):
  - rereference_p1d8ch3_correlation.npy
  - rereference_p1d8ch3_results.txt
  - rereference_p1d8ch3_correlation_matrix.png
  - rereference_p1d8ch3_distance_decay.png
  - rereference_p1d8ch3_port_pair_matrix.png
  - rereference_p1d8ch3_similarity_histogram.png
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

# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------
CSV_PATH   = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
BAD_PATH   = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis\bad_channels.txt"
BASE_CORR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis\correlation_matrix.npy"
OUT_DIR    = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client"  # files sit directly in client/ with prefix
PREFIX     = "rereference_p1d8ch3_"
REF_COL    = "Port1_dev8_ch3"
FS         = 250.0

# Scalp layout (user-confirmed): left -> right
#   P4=0, P3=1, P2=2, P1=3, P7=4, P6=5, P5=6
SCALP_ORDER = [4, 3, 2, 1, 7, 6, 5]
PORT_TO_RANK = {p: i for i, p in enumerate(SCALP_ORDER)}

# Baseline values for side-by-side table (from distance_analysis.txt)
BASELINE = {
    'within-device':                 0.7577,
    'within-port cross-dev':         0.6370,
    'cross-port rank=1 (adjacent)':  0.5423,
    'cross-port rank=2':             0.5173,
    'cross-port rank=3':             0.5301,
    'cross-port rank=4':             0.5365,
    'cross-port rank=5':             0.4809,
    'cross-port rank=6 (P4<->P5)':   0.6223,
}
BASELINE_PER_PORT = {
    4: 0.7053, 3: 0.6758, 2: 0.7021, 1: 0.6616,
    7: 0.6294, 6: 0.4755, 5: 0.7408,
}


def _t(msg, t0):
    print(f"[{time.time()-t0:6.2f}s] {msg}", flush=True)


# -----------------------------------------------------------------------------
# 1. Load data
# -----------------------------------------------------------------------------
T0 = time.time()
print(f"[load] Reading {CSV_PATH} ...", flush=True)
header = pd.read_csv(CSV_PATH, nrows=0).columns.tolist()
ch_cols = [c for c in header if c.startswith('Port')]
dtypes = {c: np.float32 for c in ch_cols}
if 'sample_number' in header:
    dtypes['sample_number'] = np.int64
df = pd.read_csv(CSV_PATH, dtype=dtypes)
_t(f"loaded {len(df):,} rows x {len(ch_cols)} channels", T0)

data = df[ch_cols].to_numpy(dtype=np.float32, copy=True)
del df
_t(f"raw matrix: {data.shape} ({data.nbytes/1e6:.1f} MB)", T0)

# Parse channel metadata
pat = re.compile(r'Port(\d+)_dev(\d+)_ch(\d+)')
port_id = np.zeros(len(ch_cols), dtype=np.int16)
dev_id  = np.zeros(len(ch_cols), dtype=np.int16)
ch_num  = np.zeros(len(ch_cols), dtype=np.int16)
device_idx = np.empty(len(ch_cols), dtype=object)
for i, c in enumerate(ch_cols):
    m = pat.match(c)
    port_id[i] = int(m.group(1))
    dev_id[i]  = int(m.group(2))
    ch_num[i]  = int(m.group(3))
    device_idx[i] = f'P{port_id[i]}D{dev_id[i]}'
unique_devices = list(dict.fromkeys(device_idx.tolist()))
_t(f"parsed metadata: {len(unique_devices)} unique devices", T0)

# -----------------------------------------------------------------------------
# 2. Filter chain
# -----------------------------------------------------------------------------
_t("filter: linear detrend", T0)
data_f = sig.detrend(data, axis=0, type='linear').astype(np.float32, copy=False)
del data

_t("filter: 1 Hz Butterworth HPF (10th order SOS, sosfiltfilt)", T0)
hpf_sos = sig.butter(10, 1.0, btype='high', fs=FS, output='sos')
data_f = sig.sosfiltfilt(hpf_sos, data_f, axis=0).astype(np.float32, copy=False)

_t("filter: 50 Hz Butterworth LPF (10th order SOS, sosfiltfilt)", T0)
lpf_sos = sig.butter(10, 50.0, btype='low', fs=FS, output='sos')
data_f = sig.sosfiltfilt(lpf_sos, data_f, axis=0).astype(np.float32, copy=False)

_t("filter: 60 Hz iirnotch Q=30 (filtfilt)", T0)
b60, a60 = sig.iirnotch(60.0, Q=30.0, fs=FS)
data_f = sig.filtfilt(b60, a60, data_f, axis=0).astype(np.float32, copy=False)

_t("filter: 70 Hz iirnotch Q=30 (filtfilt)", T0)
b70, a70 = sig.iirnotch(70.0, Q=30.0, fs=FS)
data_f = sig.filtfilt(b70, a70, data_f, axis=0).astype(np.float32, copy=False)

# -----------------------------------------------------------------------------
# 3. Parse bad channels and find reference index
# -----------------------------------------------------------------------------
_t("parsing bad_channels.txt", T0)
bad_names = set()
with open(BAD_PATH, 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        m = re.match(r'(Port\d+_dev\d+_ch\d+)\b', line)
        if m:
            bad_names.add(m.group(1))
print(f"  -> {len(bad_names)} bad channel names loaded", flush=True)

col_to_idx = {c: i for i, c in enumerate(ch_cols)}
if REF_COL not in col_to_idx:
    raise RuntimeError(f"Reference column {REF_COL} not found in CSV")
ref_idx = col_to_idx[REF_COL]
_t(f"reference channel: {REF_COL} at index {ref_idx}", T0)

# Keep the ORIGINAL (pre-rereference) signal of the reference and the full
# data matrix around for the "spread of reference noise" question at the end.
ref_signal_original = data_f[:, ref_idx].copy()
# Also compute |r| between every channel and the reference, BEFORE subtraction.
# This tells us which channels were most contaminated by the reference's own
# content (and will get over-corrected).
_t("computing |r| between every channel and reference (pre-re-ref)", T0)
# Vectorized via corrcoef of ref against all channels
refmean = ref_signal_original.mean()
refstd  = ref_signal_original.std()
ref_centered = ref_signal_original - refmean
dmean = data_f.mean(axis=0)
dstd  = data_f.std(axis=0)
# cov(ref, col) / (std_ref * std_col)
covs = (ref_centered[:, None] * (data_f - dmean[None, :])).sum(axis=0) / len(data_f)
denom = refstd * dstd
pre_corr_to_ref = np.where(denom > 0, covs / denom, 0.0).astype(np.float32)
pre_corr_to_ref[ref_idx] = np.nan  # self — skip

# -----------------------------------------------------------------------------
# 4. Re-reference: subtract reference column from every channel
# -----------------------------------------------------------------------------
_t(f"re-referencing: subtracting {REF_COL} from every channel", T0)
data_r = data_f - data_f[:, ref_idx:ref_idx+1]
# Reference channel is now identically zero. Confirm.
assert np.allclose(data_r[:, ref_idx], 0.0), "ref channel should be zero after subtraction"
del data_f

# Build bad-channel mask for re-ref analysis:
#   original bad channels + reference (now zero)
bad_mask = np.zeros(len(ch_cols), dtype=bool)
for i, c in enumerate(ch_cols):
    if c in bad_names:
        bad_mask[i] = True
bad_mask[ref_idx] = True  # reference is zero; exclude from stats
n_bad = int(bad_mask.sum())
print(f"  -> bad mask: {n_bad}/{len(ch_cols)} channels excluded "
      f"(original bad + reference)", flush=True)

# -----------------------------------------------------------------------------
# 5. Correlation matrix on re-referenced data
# -----------------------------------------------------------------------------
_t("computing 336x336 Pearson correlation on re-referenced data", T0)
with np.errstate(invalid='ignore', divide='ignore'):
    corr = np.corrcoef(data_r.T)
# Replace NaN diagonals (from zero-variance ref) with zero for display
corr_display = corr.copy()
corr_display[~np.isfinite(corr_display)] = 0.0
np.save(os.path.join(OUT_DIR, PREFIX + 'correlation.npy'),
        corr_display.astype(np.float32))
_t(f"corr shape {corr.shape}, NaN entries: {int(np.isnan(corr).sum())}", T0)

# -----------------------------------------------------------------------------
# 6. Grouping stats (upper triangle, bad-channel exclusion)
# -----------------------------------------------------------------------------
_t("building grouped correlation distributions", T0)
N = len(ch_cols)
iu, ju = np.triu_indices(N, k=1)
same_port = port_id[iu] == port_id[ju]
same_dev  = device_idx[iu] == device_idx[ju]
good_pair = (~bad_mask[iu]) & (~bad_mask[ju])
vals_all  = corr[iu, ju]

def finite(x):
    return x[np.isfinite(x)]

within_device_vals = finite(vals_all[good_pair & same_port & same_dev])
within_port_vals   = finite(vals_all[good_pair & same_port & (~same_dev)])
cross_port_vals    = finite(vals_all[good_pair & (~same_port)])

# Distance-decay bins by rank distance on scalp ordering
rank_i = np.array([PORT_TO_RANK[p] for p in port_id[iu]])
rank_j = np.array([PORT_TO_RANK[p] for p in port_id[ju]])
rank_dist = np.abs(rank_i - rank_j)

rereref_stats = {}
rereref_stats['within-device']                 = within_device_vals.mean() if len(within_device_vals) else np.nan
rereref_stats['within-port cross-dev']         = within_port_vals.mean()   if len(within_port_vals)   else np.nan

for rd in (1, 2, 3, 4, 5, 6):
    m = good_pair & (~same_port) & (rank_dist == rd)
    v = finite(vals_all[m])
    if rd == 1:
        key = 'cross-port rank=1 (adjacent)'
    elif rd == 6:
        key = 'cross-port rank=6 (P4<->P5)'
    else:
        key = f'cross-port rank={rd}'
    rereref_stats[key] = v.mean() if len(v) else np.nan

# Per-port within-device means
per_port_within_dev_rereref = {}
for p in sorted(set(port_id.tolist())):
    m = good_pair & same_port & same_dev & (port_id[iu] == p)
    v = finite(vals_all[m])
    per_port_within_dev_rereref[p] = v.mean() if len(v) else np.nan

# Per-device within-device means (needed for P1 device-level comparison)
per_device_mean_corr = {}
for d in unique_devices:
    ch_idx = np.where(device_idx == d)[0]
    good_idx = ch_idx[~bad_mask[ch_idx]]
    if len(good_idx) < 2:
        per_device_mean_corr[d] = np.nan
        continue
    sub = corr[np.ix_(good_idx, good_idx)]
    tri = sub[np.triu_indices(len(good_idx), k=1)]
    tri = tri[np.isfinite(tri)]
    per_device_mean_corr[d] = float(tri.mean()) if len(tri) else np.nan

# Port1 per-device (needed for "how wrecked is P1D8")
p1_devices = [d for d in unique_devices if d.startswith('P1D')]

# 7x7 port-pair mean r matrix (scalp-ordered)
port_pair_mat = np.full((7, 7), np.nan, dtype=np.float64)
for a_idx, a in enumerate(SCALP_ORDER):
    for b_idx, b in enumerate(SCALP_ORDER):
        if a == b:
            # on-diagonal: within-port cross-device + within-device (all pairs same port)
            m = good_pair & (port_id[iu] == a) & (port_id[ju] == a)
        else:
            m = good_pair & (
                ((port_id[iu] == a) & (port_id[ju] == b)) |
                ((port_id[iu] == b) & (port_id[ju] == a))
            )
        v = finite(vals_all[m])
        if len(v):
            port_pair_mat[a_idx, b_idx] = v.mean()

_t("stats done", T0)

# -----------------------------------------------------------------------------
# 7. Spread of reference-correlation (on ORIGINAL pre-re-ref data)
# -----------------------------------------------------------------------------
good_channels_mask_for_refcorr = ~bad_mask  # exclude bad + ref
ref_corr_good = np.abs(pre_corr_to_ref[good_channels_mask_for_refcorr])
ref_corr_good = ref_corr_good[np.isfinite(ref_corr_good)]
mean_abs_r_to_ref = float(ref_corr_good.mean()) if len(ref_corr_good) else np.nan
max_abs_r_to_ref  = float(ref_corr_good.max())  if len(ref_corr_good) else np.nan

# Top-10 most-correlated channels (pre-re-ref)
abs_r = np.abs(pre_corr_to_ref)
abs_r[bad_mask] = -1  # suppress bad and ref
top10_idx = np.argsort(-abs_r)[:10]
top10 = [(ch_cols[i], float(pre_corr_to_ref[i])) for i in top10_idx]

# -----------------------------------------------------------------------------
# 8. Write results.txt
# -----------------------------------------------------------------------------
_t("writing results.txt", T0)
bar = '-' * 78
out_txt = os.path.join(OUT_DIR, PREFIX + 'results.txt')
with open(out_txt, 'w', encoding='utf-8') as f:
    f.write("RE-REFERENCE EXPERIMENT: subtract Port1_dev8_ch3 from every channel\n")
    f.write(f"Source CSV: {os.path.basename(CSV_PATH)}\n")
    f.write(f"Samples: {len(data_r):,} @ {FS:g} Hz  "
            f"(~{len(data_r)/FS/60:.2f} min)\n")
    f.write(f"Reference channel: {REF_COL} (index {ref_idx})\n")
    f.write(f"Bad channels (from bad_channels.txt + reference): "
            f"{n_bad}/{len(ch_cols)}\n")
    f.write("Filter chain: detrend -> 1 Hz HPF (Butter10 SOS) -> 50 Hz LPF "
            "(Butter10 SOS) -> 60 Hz notch (Q=30) -> 70 Hz notch (Q=30) -> "
            "SUBTRACT REFERENCE\n\n")

    f.write("BASELINE vs P1D8_ch3 REFERENCE\n")
    f.write(bar + "\n")
    f.write(f"{'Group':30s}| {'Baseline':9s}| {'Re-ref':9s}| {'Delta':9s}\n")
    f.write(bar + "\n")
    for k, base_v in BASELINE.items():
        cur = rereref_stats.get(k, np.nan)
        if np.isfinite(cur):
            delta = cur - base_v
            f.write(f"{k:30s}| {base_v:+.4f}  | {cur:+.4f}  | "
                    f"{delta:+.4f}\n")
        else:
            f.write(f"{k:30s}| {base_v:+.4f}  |    nan    |    nan\n")

    f.write("\nPer-port within-device means (baseline -> re-ref)\n")
    f.write(bar + "\n")
    for p in [4, 3, 2, 1, 7, 6, 5]:
        base_v = BASELINE_PER_PORT[p]
        cur = per_port_within_dev_rereref.get(p, np.nan)
        if np.isfinite(cur):
            f.write(f"  P{p}: {base_v:+.4f} -> {cur:+.4f}  "
                    f"(delta {cur - base_v:+.4f})\n")
        else:
            f.write(f"  P{p}: {base_v:+.4f} -> nan\n")

    f.write("\n7x7 port-pair mean r (re-referenced), scalp order "
            "[P4 P3 P2 P1 P7 P6 P5]\n")
    f.write(bar + "\n")
    f.write("          " + "".join(f"{'P'+str(p):>9s}" for p in SCALP_ORDER) + "\n")
    for i, p in enumerate(SCALP_ORDER):
        row = f"  P{p}     "
        for j in range(7):
            v = port_pair_mat[i, j]
            row += f"{v:+9.4f}" if np.isfinite(v) else "      nan"
        f.write(row + "\n")

    f.write("\nPort 1 per-device within-device mean r (re-referenced)\n")
    f.write(bar + "\n")
    f.write("  device   mean_r   n_good\n")
    for d in p1_devices:
        ch_idx = np.where(device_idx == d)[0]
        good_idx = ch_idx[~bad_mask[ch_idx]]
        mc = per_device_mean_corr[d]
        mc_str = f"{mc:+.4f}" if np.isfinite(mc) else "  nan"
        f.write(f"  {d:<8s} {mc_str}   {len(good_idx)}\n")

    f.write("\nSpread of reference-specific noise "
            "(|r| between each channel and the ORIGINAL reference, "
            "good channels only)\n")
    f.write(bar + "\n")
    f.write(f"  n (good channels, excluding ref) = {len(ref_corr_good)}\n")
    f.write(f"  mean |r| to reference = {mean_abs_r_to_ref:.4f}\n")
    f.write(f"  max  |r| to reference = {max_abs_r_to_ref:.4f}\n")
    f.write("  Top 10 channels most correlated with reference "
            "(pre-re-ref, signed r):\n")
    for name, r in top10:
        f.write(f"    {name:<22s}  r = {r:+.4f}\n")

    f.write("\n\nDistribution stats (re-referenced)\n")
    f.write(bar + "\n")
    for name, v in [('within-device', within_device_vals),
                    ('within-port cross-dev', within_port_vals),
                    ('cross-port', cross_port_vals)]:
        if len(v):
            f.write(f"  {name:23s}: n={len(v):6d}  "
                    f"mean={v.mean():+.4f}  median={np.median(v):+.4f}  "
                    f"std={v.std():.4f}\n")

print(f"[write] {out_txt}", flush=True)

# -----------------------------------------------------------------------------
# 9. Plots
# -----------------------------------------------------------------------------
_t("plot 1/4: correlation matrix side-by-side (baseline vs re-ref)", T0)
baseline_corr = np.load(BASE_CORR)
baseline_corr_plot = baseline_corr.copy()
baseline_corr_plot[~np.isfinite(baseline_corr_plot)] = 0.0

# port boundaries for annotation
port_boundaries = []
prev_p = port_id[0]
for i in range(1, N):
    if port_id[i] != prev_p:
        port_boundaries.append(i)
        prev_p = port_id[i]
port_centers = []
start = 0
for b in port_boundaries + [N]:
    port_centers.append((start + b - 1) / 2)
    start = b
port_labels_inorder = [f'P{p}' for p in sorted(set(port_id.tolist()))]

fig, axes = plt.subplots(1, 2, figsize=(20, 9))
for ax, mat, title in [
    (axes[0], baseline_corr_plot, 'Baseline (no re-reference)'),
    (axes[1], corr_display,       f'Re-referenced to {REF_COL}'),
]:
    im = ax.imshow(mat, vmin=-1, vmax=1, cmap='RdBu_r',
                   aspect='equal', interpolation='nearest')
    for b in port_boundaries:
        ax.axhline(b - 0.5, color='white', linewidth=1.4)
        ax.axvline(b - 0.5, color='white', linewidth=1.4)
    ax.set_xticks(port_centers)
    ax.set_yticks(port_centers)
    ax.set_xticklabels(port_labels_inorder)
    ax.set_yticklabels(port_labels_inorder)
    ax.set_title(title, fontsize=12)
    fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02).set_label('Pearson r')
fig.suptitle('336x336 correlation matrix -- baseline vs P1D8_ch3 re-reference',
             fontsize=13)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(os.path.join(OUT_DIR, PREFIX + 'correlation_matrix.png'), dpi=140)
plt.close()

_t("plot 2/4: distance decay overlay", T0)
# Build baseline decay curve from the baseline correlation matrix.
# Use same bad mask minus the reference exclusion (reference was not bad in
# baseline) for a fair baseline.
baseline_bad_mask = bad_mask.copy()
baseline_bad_mask[ref_idx] = (REF_COL in bad_names)  # only bad if it was in list
iu_b, ju_b = iu, ju  # same upper-triangle
same_port_b = same_port
same_dev_b  = same_dev
good_b      = (~baseline_bad_mask[iu_b]) & (~baseline_bad_mask[ju_b])
vals_b      = baseline_corr[iu_b, ju_b]

def mean_for(good_mask, vals):
    v = vals[good_mask]
    v = v[np.isfinite(v)]
    return v.mean() if len(v) else np.nan

x_categories = ['within\ndev', 'within\nport', 'rank 1', 'rank 2',
                'rank 3', 'rank 4', 'rank 5', 'rank 6']
baseline_curve = [
    mean_for(good_b & same_port_b & same_dev_b, vals_b),
    mean_for(good_b & same_port_b & ~same_dev_b, vals_b),
]
rereref_curve = [
    mean_for(good_pair & same_port & same_dev, vals_all),
    mean_for(good_pair & same_port & ~same_dev, vals_all),
]
for rd in (1, 2, 3, 4, 5, 6):
    baseline_curve.append(mean_for(good_b & ~same_port_b & (rank_dist == rd), vals_b))
    rereref_curve.append(mean_for(good_pair & ~same_port & (rank_dist == rd), vals_all))

fig, ax = plt.subplots(figsize=(10, 6))
x = np.arange(len(x_categories))
ax.plot(x, baseline_curve, 'o-', color='#555555', lw=2, ms=8,
        label='baseline (no re-ref)')
ax.plot(x, rereref_curve, 's-', color='#c0392b', lw=2, ms=8,
        label=f're-referenced to {REF_COL}')
ax.axhline(0, color='gray', lw=0.5)
ax.set_xticks(x)
ax.set_xticklabels(x_categories)
ax.set_ylabel('Mean Pearson r')
ax.set_title('Distance-decay: baseline vs P1D8_ch3 re-reference\n'
             '(rank distance measured on scalp order P4-P3-P2-P1-P7-P6-P5)')
ax.grid(alpha=0.3, axis='y')
ax.legend()
# annotate deltas
for xi, (a, b) in enumerate(zip(baseline_curve, rereref_curve)):
    if np.isfinite(a) and np.isfinite(b):
        ax.annotate(f'{b-a:+.2f}', xy=(xi, b), xytext=(0, -14),
                    textcoords='offset points', ha='center', fontsize=8,
                    color='#c0392b')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, PREFIX + 'distance_decay.png'), dpi=140)
plt.close()

_t("plot 3/4: 7x7 port-pair heatmap (scalp ordered)", T0)
fig, ax = plt.subplots(figsize=(8, 7))
im = ax.imshow(port_pair_mat, vmin=-1, vmax=1, cmap='RdBu_r',
               aspect='equal', interpolation='nearest')
ax.set_xticks(range(7))
ax.set_yticks(range(7))
ax.set_xticklabels([f'P{p}' for p in SCALP_ORDER])
ax.set_yticklabels([f'P{p}' for p in SCALP_ORDER])
for i in range(7):
    for j in range(7):
        v = port_pair_mat[i, j]
        if np.isfinite(v):
            color = 'white' if abs(v) > 0.5 else 'black'
            ax.text(j, i, f'{v:+.2f}', ha='center', va='center',
                    fontsize=9, color=color)
fig.colorbar(im, ax=ax, fraction=0.045, pad=0.04).set_label('Pearson r')
ax.set_title(f'7x7 port-pair mean r -- re-referenced to {REF_COL}\n'
             '(scalp order left->right)')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, PREFIX + 'port_pair_matrix.png'), dpi=140)
plt.close()

_t("plot 4/4: similarity histogram with baseline medians", T0)
# Baseline medians, recomputed from baseline matrix using same good mask
within_device_base = finite(vals_b[good_b & same_port_b & same_dev_b])
within_port_base   = finite(vals_b[good_b & same_port_b & ~same_dev_b])
cross_port_base    = finite(vals_b[good_b & ~same_port_b])
base_medians = {
    'within-device':    np.median(within_device_base) if len(within_device_base) else np.nan,
    'within-port':      np.median(within_port_base)   if len(within_port_base)   else np.nan,
    'cross-port':       np.median(cross_port_base)    if len(cross_port_base)    else np.nan,
}

fig, ax = plt.subplots(figsize=(11, 6))
bins = np.linspace(-1, 1, 121)
ax.hist(cross_port_vals, bins=bins, alpha=0.5, color='#777777',
        label=f'cross-port (n={len(cross_port_vals):,}, mu={cross_port_vals.mean():+.3f})',
        density=True)
ax.hist(within_port_vals, bins=bins, alpha=0.55, color='#d9822b',
        label=f'within-port (n={len(within_port_vals):,}, mu={within_port_vals.mean():+.3f})',
        density=True)
ax.hist(within_device_vals, bins=bins, alpha=0.6, color='#2b7bd9',
        label=f'within-device (n={len(within_device_vals):,}, mu={within_device_vals.mean():+.3f})',
        density=True)

# Baseline medians as vertical lines
for name, color in [('within-device', '#0b4e9c'),
                    ('within-port',   '#a65200'),
                    ('cross-port',    '#333333')]:
    bm = base_medians[name]
    if np.isfinite(bm):
        ax.axvline(bm, color=color, linestyle='--', linewidth=1.5,
                   label=f'{name} baseline median={bm:+.3f}')

ax.set_xlabel('Pearson r')
ax.set_ylabel('Density')
ax.set_xlim(-1, 1)
ax.grid(alpha=0.3)
ax.legend(loc='upper left', fontsize=8)
ax.set_title(f'Similarity by group after re-referencing to {REF_COL}\n'
             '(solid bars = re-ref; dashed lines = baseline medians)')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, PREFIX + 'similarity_histogram.png'), dpi=140)
plt.close()

_t("ALL DONE", T0)
print(f"\nOutputs in: {OUT_DIR}")
for suffix in ['correlation.npy', 'results.txt', 'correlation_matrix.png',
               'distance_decay.png', 'port_pair_matrix.png',
               'similarity_histogram.png']:
    print(f"  {PREFIX}{suffix}")
