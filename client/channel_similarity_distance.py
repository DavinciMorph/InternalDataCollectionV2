"""
Channel Similarity — Distance Decay Analysis

Extends channel_similarity.py by asking:
  1. Does channel correlation monotonically decay with scalp rank-distance?
  2. What is the specific contralateral temporal (P4<->P5) correlation?
  3. How much unique variance does each working channel carry vs its
     most-similar neighbor?

Does NOT recompute anything heavy — loads the 336x336 correlation matrix from
channel_similarity_analysis/correlation_matrix.npy and builds from there.

Outputs (client/channel_similarity_analysis/):
  - distance_decay.png
  - port_pair_matrix.png
  - unique_info_hist.png
  - distance_analysis.txt
"""
import os
import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
OUT_DIR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis"
CORR_NPY = os.path.join(OUT_DIR, 'correlation_matrix.npy')
BAD_TXT  = os.path.join(OUT_DIR, 'bad_channels.txt')

# Scalp ordering (user-confirmed 2026-04-10, left -> right):
#   P4 -> P3 -> P2 -> P1 -> P7 -> P6 -> P5
#   P4 = furthest left, P1 = centerline (Cz), P5 = furthest right
# Opposite-ends pair (rank_dist=6) is P4 <-> P5.
SCALP_ORDER = [4, 3, 2, 1, 7, 6, 5]
PORT_RANK = {p: i for i, p in enumerate(SCALP_ORDER)}

# ---------------------------------------------------------------------------
# 1. Load correlation matrix (do NOT recompute)
# ---------------------------------------------------------------------------
print(f"[load] Loading correlation matrix from {CORR_NPY}")
corr = np.load(CORR_NPY)
print(f"[load] corr.shape = {corr.shape}, dtype = {corr.dtype}")

# ---------------------------------------------------------------------------
# 2. Rebuild (port, device, ch) index from CSV header only (no full load)
# ---------------------------------------------------------------------------
print(f"[load] Reading header row from {CSV_PATH}")
header = pd.read_csv(CSV_PATH, nrows=0).columns.tolist()
ch_cols = [c for c in header if c.startswith('Port')]
assert len(ch_cols) == corr.shape[0], \
    f"Column count {len(ch_cols)} != corr dim {corr.shape[0]}"

port_id = np.zeros(len(ch_cols), dtype=np.int16)
dev_id  = np.zeros(len(ch_cols), dtype=np.int16)
ch_num  = np.zeros(len(ch_cols), dtype=np.int16)
for i, col in enumerate(ch_cols):
    parts = col.split('_')
    port_id[i] = int(parts[0].replace('Port', ''))
    dev_id[i]  = int(parts[1].replace('dev', ''))
    ch_num[i]  = int(parts[2].replace('ch', ''))

# Composite device key (port*100 + dev) — unique per ADS1299
device_key = port_id.astype(np.int32) * 100 + dev_id.astype(np.int32)

# ---------------------------------------------------------------------------
# 3. Parse bad channel list
# ---------------------------------------------------------------------------
print(f"[load] Parsing bad channel list from {BAD_TXT}")
bad_mask = np.zeros(len(ch_cols), dtype=bool)
bad_set = set()
pat = re.compile(r'^\s*(Port\d+_dev\d+_ch\d+)\b')
with open(BAD_TXT, 'r', encoding='utf-8') as f:
    in_list = False
    for line in f:
        if line.startswith('Flagged channels:'):
            in_list = True
            continue
        if not in_list:
            continue
        m = pat.match(line)
        if m:
            bad_set.add(m.group(1))
col_to_idx = {c: i for i, c in enumerate(ch_cols)}
for name in bad_set:
    if name in col_to_idx:
        bad_mask[col_to_idx[name]] = True
n_bad = int(bad_mask.sum())
n_good = len(ch_cols) - n_bad
print(f"[load] Bad channels: {n_bad}/{len(ch_cols)}  ({n_good} good)")

# ---------------------------------------------------------------------------
# 4. Upper-triangle index and per-pair labels
# ---------------------------------------------------------------------------
N = len(ch_cols)
iu, ju = np.triu_indices(N, k=1)

same_port = port_id[iu] == port_id[ju]
same_dev  = device_key[iu] == device_key[ju]
good_pair = (~bad_mask[iu]) & (~bad_mask[ju])

# rank-distance for each pair (0 = same port, 6 = opposite end of scalp)
rank_i = np.vectorize(PORT_RANK.get)(port_id[iu])
rank_j = np.vectorize(PORT_RANK.get)(port_id[ju])
rank_dist = np.abs(rank_i - rank_j).astype(np.int16)

vals = corr[iu, ju].astype(np.float64)
finite = np.isfinite(vals)

# ---------------------------------------------------------------------------
# 5. Distance-decay bins
# ---------------------------------------------------------------------------
print("[calc] Computing distance-decay bins")
bins = []

def bin_stats(name, mask):
    v = vals[mask & good_pair & finite]
    if len(v) == 0:
        return (name, 0, np.nan, np.nan, np.nan, np.nan)
    se = v.std(ddof=1) / np.sqrt(len(v)) if len(v) > 1 else 0.0
    return (name, len(v), float(v.mean()), float(np.median(v)),
            float(v.std(ddof=1) if len(v) > 1 else 0.0), float(se))

# within-device
bins.append(bin_stats('within-device',          same_port & same_dev))
# within-port / cross-device
bins.append(bin_stats('within-port cross-dev',  same_port & (~same_dev)))
# cross-port rank distances 1..6
for d in range(1, 7):
    bins.append(bin_stats(f'cross-port rank={d}', (~same_port) & (rank_dist == d)))

for name, n, mean_r, med_r, std_r, se_r in bins:
    print(f"[calc]   {name:<25s} n={n:<7d} mean={mean_r:+.4f} median={med_r:+.4f}  std={std_r:.4f}")

# ---------------------------------------------------------------------------
# 6. 7x7 port-pair mean-r matrix (ordered along scalp)
# ---------------------------------------------------------------------------
print("[calc] Computing 7x7 port-pair mean-r matrix (scalp-ordered)")
pp = np.full((7, 7), np.nan, dtype=np.float64)
pp_n = np.zeros((7, 7), dtype=np.int64)
for a_rank in range(7):
    port_a = SCALP_ORDER[a_rank]
    for b_rank in range(a_rank, 7):
        port_b = SCALP_ORDER[b_rank]
        if a_rank == b_rank:
            # diagonal: all within-port pairs (both within-device and cross-device)
            mask = (port_id[iu] == port_a) & (port_id[ju] == port_a) & good_pair & finite
        else:
            mask = (((port_id[iu] == port_a) & (port_id[ju] == port_b)) |
                    ((port_id[iu] == port_b) & (port_id[ju] == port_a))) & good_pair & finite
        v = vals[mask]
        if len(v) > 0:
            pp[a_rank, b_rank] = float(v.mean())
            pp[b_rank, a_rank] = pp[a_rank, b_rank]
            pp_n[a_rank, b_rank] = len(v)
            pp_n[b_rank, a_rank] = len(v)

# Key pair: Port4 <-> Port5 (contralateral temporal)
p4_rank = PORT_RANK[4]
p5_rank = PORT_RANK[5]
p4_p5_mean = pp[p4_rank, p5_rank]
p4_p5_n    = int(pp_n[p4_rank, p5_rank])
print(f"[calc] Port4<->Port5 mean r = {p4_p5_mean:+.4f}  (n={p4_p5_n})")

# ---------------------------------------------------------------------------
# 7. Per-channel unique-information estimate: 1 - r_max^2
# ---------------------------------------------------------------------------
print("[calc] Computing per-channel unique-information (1 - r_max^2)")
# mask out self and bad channels on both rows and cols
good_idx = np.where(~bad_mask)[0]
good_count = len(good_idx)

unique_info = np.full(N, np.nan, dtype=np.float64)
r_max_all   = np.full(N, np.nan, dtype=np.float64)
r_max_partner = np.full(N, -1, dtype=np.int32)
for i in good_idx:
    row = np.abs(corr[i, :]).copy()
    row[i] = np.nan                  # exclude self
    row[bad_mask] = np.nan           # exclude bad channels
    if np.all(~np.isfinite(row)):
        continue
    j = int(np.nanargmax(row))
    rmax = float(row[j])
    rmax = min(rmax, 1.0)  # guard against tiny fp overshoot
    r_max_all[i] = rmax
    r_max_partner[i] = j
    unique_info[i] = 1.0 - rmax * rmax

# Distribution summary
ui_good = unique_info[good_idx]
ui_good = ui_good[np.isfinite(ui_good)]
ui_med  = float(np.median(ui_good))
ui_q25  = float(np.quantile(ui_good, 0.25))
ui_q50  = float(np.quantile(ui_good, 0.50))
ui_q75  = float(np.quantile(ui_good, 0.75))
n_ui_above_025 = int((ui_good > 0.25).sum())
n_ui_above_050 = int((ui_good > 0.50).sum())
n_ui_above_075 = int((ui_good > 0.75).sum())
frac_above_025 = n_ui_above_025 / len(ui_good) if len(ui_good) else 0.0
print(f"[calc] unique-info median={ui_med:.3f}  "
      f"Q25={ui_q25:.3f}  Q50={ui_q50:.3f}  Q75={ui_q75:.3f}")
print(f"[calc] {n_ui_above_025}/{len(ui_good)} "
      f"({100*frac_above_025:.1f}%) channels have >25% unique variance")

# ---------------------------------------------------------------------------
# 8. Plots
# ---------------------------------------------------------------------------
print("[plot] distance_decay.png")
fig, ax = plt.subplots(figsize=(11, 6))
labels     = [b[0] for b in bins]
means      = np.array([b[2] for b in bins])
medians    = np.array([b[3] for b in bins])
stds       = np.array([b[4] for b in bins])
ns         = np.array([b[1] for b in bins])
sems       = np.array([b[5] for b in bins])

xs = np.arange(len(bins))
# color: first 2 bars (within-device / within-port) in blue, cross-port bars in a gradient
colors = ['#1f4e91', '#4b86c5'] + ['#d62728' if d == 6 else '#e0a050' if d >= 4 else '#6bbf59' for d in range(1, 7)]
bars = ax.bar(xs, means, yerr=sems, capsize=4,
              color=colors, edgecolor='black', linewidth=0.6)
ax.plot(xs, medians, marker='s', color='black', linestyle='--',
        label='median', markersize=6, linewidth=1.2)

# annotate counts above bars
for i, b in enumerate(bars):
    h = b.get_height()
    ax.text(b.get_x() + b.get_width() / 2,
            h + 0.015 + sems[i],
            f'n={ns[i]:,}\nμ={means[i]:+.3f}',
            ha='center', va='bottom', fontsize=8)

ax.axhline(0, color='gray', linewidth=0.5)
ax.set_xticks(xs)
ax.set_xticklabels(labels, rotation=25, ha='right')
ax.set_ylabel('Pearson r')
ax.set_ylim(-0.05, 1.0)
ax.set_title('Channel-pair correlation vs scalp rank-distance\n'
             f'Scalp order: {" -> ".join(f"P{p}" for p in SCALP_ORDER)}    (bad channels excluded)')
ax.grid(axis='y', alpha=0.3)
ax.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'distance_decay.png'), dpi=140)
plt.close()

print("[plot] port_pair_matrix.png")
fig, ax = plt.subplots(figsize=(8, 7))
vmin, vmax = -1.0, 1.0
im = ax.imshow(pp, vmin=vmin, vmax=vmax, cmap='RdBu_r', aspect='equal')
tick_labels = [f'P{p}' for p in SCALP_ORDER]
ax.set_xticks(np.arange(7))
ax.set_yticks(np.arange(7))
ax.set_xticklabels(tick_labels)
ax.set_yticklabels(tick_labels)
for i_ in range(7):
    for j_ in range(7):
        if np.isfinite(pp[i_, j_]):
            txt_color = 'white' if abs(pp[i_, j_]) > 0.55 else 'black'
            ax.text(j_, i_, f'{pp[i_, j_]:+.2f}',
                    ha='center', va='center', color=txt_color, fontsize=10)
# highlight P4 <-> P5
rect_i, rect_j = p4_rank, p5_rank
ax.add_patch(plt.Rectangle((rect_j - 0.5, rect_i - 0.5), 1, 1,
                           fill=False, edgecolor='lime', linewidth=2.5))
ax.add_patch(plt.Rectangle((rect_i - 0.5, rect_j - 0.5), 1, 1,
                           fill=False, edgecolor='lime', linewidth=2.5))
cbar = plt.colorbar(im, fraction=0.046, pad=0.04)
cbar.set_label('Mean Pearson r')
ax.set_title('Port-pair mean correlation (scalp-ordered)\n'
             'Green box: P4 <-> P5 (contralateral temporal)')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'port_pair_matrix.png'), dpi=140)
plt.close()

print("[plot] unique_info_hist.png")
fig, ax = plt.subplots(figsize=(11, 6))
ax.hist(ui_good, bins=np.linspace(0, 1, 51),
        color='#4b86c5', edgecolor='black', linewidth=0.5)
ax.axvline(ui_med, color='red', linestyle='--', linewidth=1.4,
           label=f'median = {ui_med:.3f}')
ax.axvline(0.25, color='black', linestyle=':', linewidth=1.2,
           label=f'25% threshold ({frac_above_025*100:.1f}% of channels exceed)')
ax.set_xlabel('1 - r_max^2  (fraction of variance NOT explained by most-similar channel)')
ax.set_ylabel('Channel count')
ax.set_title('Per-channel unique information estimate\n'
             f'(working channels only, n={len(ui_good)})')
ax.grid(alpha=0.3)
ax.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'unique_info_hist.png'), dpi=140)
plt.close()

# ---------------------------------------------------------------------------
# 9. distance_analysis.txt
# ---------------------------------------------------------------------------
print("[write] distance_analysis.txt")
bar = "-" * 78
lines = []
lines.append("Distance-decay analysis -- eeg_data_2026-04-10_200737.csv")
lines.append(f"Source matrix: {CORR_NPY}")
lines.append(f"Channels: {N} (bad={n_bad}, good={n_good})")
lines.append(f"Scalp order (left->right): {' -> '.join(f'P{p}' for p in SCALP_ORDER)}")
lines.append("")
lines.append("Distance-decay bins (bad channels excluded)")
lines.append(bar)
lines.append(f"  {'bin':<25s} {'n':>9s}  {'mean_r':>8s}  {'median_r':>9s}  {'std':>7s}")
for name, n, mean_r, med_r, std_r, se_r in bins:
    lines.append(f"  {name:<25s} {n:>9,d}  {mean_r:+8.4f}  {med_r:+9.4f}  {std_r:7.4f}")
lines.append("")

lines.append("Port-pair mean r (7x7, scalp-ordered)")
lines.append(bar)
header_ports = "         " + "  ".join(f"{f'P{p}':>7s}" for p in SCALP_ORDER)
lines.append(header_ports)
for i_ in range(7):
    row_port = f"P{SCALP_ORDER[i_]}"
    row_cells = []
    for j_ in range(7):
        if np.isfinite(pp[i_, j_]):
            row_cells.append(f"{pp[i_, j_]:+7.4f}")
        else:
            row_cells.append("    nan")
    lines.append(f"  {row_port:<5s}  " + "  ".join(row_cells))
lines.append("")

lines.append(f"Port-pair n (7x7, scalp-ordered)")
lines.append(bar)
lines.append(header_ports)
for i_ in range(7):
    row_port = f"P{SCALP_ORDER[i_]}"
    row_cells = [f"{pp_n[i_, j_]:>7d}" for j_ in range(7)]
    lines.append(f"  {row_port:<5s}  " + "  ".join(row_cells))
lines.append("")

lines.append("Key contralateral-temporal pair")
lines.append(bar)
lines.append(f"  Port4 <-> Port5  mean r = {p4_p5_mean:+.4f}  (n={p4_p5_n})")
lines.append("")

# Adjacency ladder (scalp-adjacent pairs only)
lines.append("Adjacent port pairs along scalp (rank_dist=1)")
lines.append(bar)
for k in range(6):
    a, b = SCALP_ORDER[k], SCALP_ORDER[k + 1]
    r = pp[k, k + 1]
    n_ab = int(pp_n[k, k + 1])
    lines.append(f"  P{a} <-> P{b}   mean r = {r:+.4f}  (n={n_ab})")
lines.append("")

lines.append("Per-channel unique information (1 - r_max^2, good channels only)")
lines.append(bar)
lines.append(f"  n                          = {len(ui_good)}")
lines.append(f"  median                     = {ui_med:.4f}")
lines.append(f"  Q25 / Q50 / Q75            = {ui_q25:.4f} / {ui_q50:.4f} / {ui_q75:.4f}")
lines.append(f"  min / max                  = {ui_good.min():.4f} / {ui_good.max():.4f}")
lines.append(f"  channels with >25% unique  = {n_ui_above_025}/{len(ui_good)} "
             f"({100*frac_above_025:.1f}%)")
lines.append(f"  channels with >50% unique  = {n_ui_above_050}/{len(ui_good)} "
             f"({100*n_ui_above_050/len(ui_good):.1f}%)")
lines.append(f"  channels with >75% unique  = {n_ui_above_075}/{len(ui_good)} "
             f"({100*n_ui_above_075/len(ui_good):.1f}%)")
lines.append("")

# List the tightest clique members (lowest unique info)
lines.append("Most-redundant channels (smallest unique info)")
lines.append(bar)
order = np.argsort(unique_info)
shown = 0
for idx in order:
    if not np.isfinite(unique_info[idx]):
        continue
    partner = int(r_max_partner[idx])
    if partner < 0:
        continue
    lines.append(f"  {ch_cols[idx]:<24s} unique={unique_info[idx]:.4f}  "
                 f"r_max={r_max_all[idx]:+.4f}  partner={ch_cols[partner]}")
    shown += 1
    if shown >= 20:
        break
lines.append("")

with open(os.path.join(OUT_DIR, 'distance_analysis.txt'), 'w', encoding='utf-8') as f:
    f.write("\n".join(lines))

print("[done] Wrote distance_decay.png, port_pair_matrix.png, "
      "unique_info_hist.png, distance_analysis.txt")
