"""
Channel Similarity Analysis — eeg_data_2026-04-10_200737.csv

Answers:
  1. How similar is every channel to every other channel?
  2. Do channels from different ADS1299 devices look meaningfully different?

Pipeline (per channel):
  1. Bad-channel detection (before filtering): flat / saturating / railed
  2. Linear detrend
  3. 60 Hz notch (iirnotch Q=30, filtfilt — zero-phase)
  4. 70 Hz notch (iirnotch Q=30, filtfilt) — residual GPU EMI
  5. Pearson corrcoef over the full recording
  6. Grouping: within-device / within-port cross-device / cross-port

Outputs (client/channel_similarity_analysis/):
  - correlation_matrix_full.png
  - per_device_grid.png
  - similarity_histogram.png
  - port_boxplot.png
  - device_mean_correlation.png
  - correlation_matrix.npy
  - bad_channels.txt
  - group_stats.txt

Coherence is skipped by default: 336*335/2 = 56,280 pair-wise scipy.signal.coherence
calls at nperseg=1024 is ~10-20 min single-threaded with no information Pearson misses
at this timescale. Noted in summary.
"""
import os
import sys
import time
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal as sig
from scipy import stats

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
OUT_DIR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis"
FS       = 250.0

# Bad-channel thresholds
# This recording is extremely noisy (median filtered std ~3300 uV — ~30x a
# typical EEG channel). Rather than use absolute thresholds that would flag
# 80%+ of the data, we use a robust MAD-based outlier detector on log10(std)
# PLUS absolute hard limits. Thresholds were tuned on the empirical
# distribution of this recording (see script comment in write-up).
FLAT_STD_ABS    = 0.1       # hard floor: std < 0.1 uV -> dead
SAT_STD_ABS     = 30000.0   # hard ceil: std > 30 mV -> pathological
ADC_MAX         = 8388607
RAIL_FRAC_THRESH = 0.50     # >50% samples at +/-8388607 -> railed
MAD_Z_THRESH    = 3.0       # |robust z| on log10(std) > 3 -> outlier

os.makedirs(OUT_DIR, exist_ok=True)
print(f"[init] Output directory: {OUT_DIR}")

# ─────────────────────────────────────────────────────────────────────────────
# 1. Load CSV
# ─────────────────────────────────────────────────────────────────────────────
print(f"[load] Reading {CSV_PATH} ...")
t0 = time.time()
# Peek header to decide dtypes
header = pd.read_csv(CSV_PATH, nrows=0).columns.tolist()
ch_cols = [c for c in header if c.startswith('Port')]
dtypes = {c: np.float32 for c in ch_cols}
if 'sample_number' in header:
    dtypes['sample_number'] = np.int64

df = pd.read_csv(CSV_PATH, dtype=dtypes)
print(f"[load] {len(df):,} rows, {len(ch_cols)} channels "
      f"({time.time()-t0:.1f}s)")

n_samples = len(df)
duration_s = n_samples / FS
print(f"[load] Duration: {duration_s:.1f}s ({duration_s/60:.2f} min)")

# ─────────────────────────────────────────────────────────────────────────────
# 2. Parse channel metadata (port / device / ch / device_id / SPI bus)
# ─────────────────────────────────────────────────────────────────────────────
ch_info = []
for col in ch_cols:
    parts = col.split('_')
    port = int(parts[0].replace('Port', ''))
    dev  = int(parts[1].replace('dev', ''))
    ch   = int(parts[2].replace('ch', ''))
    if port in (1, 2):
        spi = 'SPI0'
    elif port in (3, 4):
        spi = 'SPI3'
    elif port in (5, 6):
        spi = 'SPI4'
    elif port == 7:
        spi = 'SPI5'
    else:
        spi = 'unknown'
    ch_info.append({
        'col': col, 'port': port, 'dev': dev, 'ch': ch,
        'spi': spi, 'device_id': f'P{port}D{dev}',
    })

port_id    = np.array([c['port']      for c in ch_info], dtype=np.int16)
device_idx = np.array([c['device_id'] for c in ch_info])  # string key
dev_id     = np.array([c['dev']       for c in ch_info], dtype=np.int16)
ch_num     = np.array([c['ch']        for c in ch_info], dtype=np.int16)

unique_devices = list(dict.fromkeys(device_idx.tolist()))  # preserve order
print(f"[parse] Unique devices: {len(unique_devices)} "
      f"(expected 42 for P1=8,P2=7,P3=5,P4=5,P5=5,P6=5,P7=7)")

# ─────────────────────────────────────────────────────────────────────────────
# 3. Extract raw matrix and compute rail fraction (before any filtering)
# ─────────────────────────────────────────────────────────────────────────────
print("[load] Extracting raw matrix ...")
t0 = time.time()
# (n_samples, 336) float32.
data = df[ch_cols].to_numpy(dtype=np.float32, copy=True)
print(f"[load] Raw matrix: {data.shape} ({data.nbytes/1e6:.1f} MB, {time.time()-t0:.1f}s)")

# Rail fraction must be measured on raw (pre-filter) samples — filtering smears
# the ADC extremes away from +/-max.
raw_std = data.std(axis=0)
rail_frac = (np.abs(data) >= (ADC_MAX - 1)).mean(axis=0)

# ─────────────────────────────────────────────────────────────────────────────
# 4. Preprocess: detrend -> 60 Hz notch -> 70 Hz notch (filtfilt, zero phase)
# ─────────────────────────────────────────────────────────────────────────────
print("[filt] Linear detrend ...")
t0 = time.time()
# scipy.signal.detrend promotes to float64 internally; cast back to float32 after.
data_filt = sig.detrend(data, axis=0, type='linear').astype(np.float32, copy=False)
print(f"[filt] Detrend done ({time.time()-t0:.1f}s)")

print("[filt] 1 Hz Butterworth HPF (10th order, SOS, sosfiltfilt zero-phase) ...")
t0 = time.time()
# Matches client/simpleviz.py:386 HPF (10th order Butter, 1 Hz cutoff, fs=250).
# simpleviz uses sosfilt (causal, stateful, real-time); we use sosfiltfilt
# (zero-phase, offline) so there is no phase distortion in the analysis.
hpf_sos = sig.butter(10, 1.0, btype='high', fs=FS, output='sos')
data_filt = sig.sosfiltfilt(hpf_sos, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] HPF done ({time.time()-t0:.1f}s)")

print("[filt] 50 Hz Butterworth LPF (10th order, SOS, sosfiltfilt zero-phase) ...")
t0 = time.time()
# Matches client/simpleviz.py:386 LPF. Applied AFTER HPF and BEFORE the 60/70 Hz
# notches. The notches then become largely redundant but are cheap and left in.
lpf_sos = sig.butter(10, 50.0, btype='low', fs=FS, output='sos')
data_filt = sig.sosfiltfilt(lpf_sos, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] LPF done ({time.time()-t0:.1f}s)")

print("[filt] 60 Hz notch (filtfilt) ...")
t0 = time.time()
b60, a60 = sig.iirnotch(60.0, Q=30.0, fs=FS)
data_filt = sig.filtfilt(b60, a60, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] 60 Hz done ({time.time()-t0:.1f}s)")

print("[filt] 70 Hz notch (filtfilt, optional residual EMI) ...")
t0 = time.time()
b70, a70 = sig.iirnotch(70.0, Q=30.0, fs=FS)
data_filt = sig.filtfilt(b70, a70, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] 70 Hz done ({time.time()-t0:.1f}s)")

# Free raw copy — filtered array is the only one we need going forward
del data

# ─────────────────────────────────────────────────────────────────────────────
# 4b. Bad-channel detection on FILTERED data
# ─────────────────────────────────────────────────────────────────────────────
print("[bad] Computing per-channel statistics on filtered data ...")
t0 = time.time()
filt_std = data_filt.std(axis=0)
log_std = np.log10(np.maximum(filt_std, 1e-6))
med = np.median(log_std)
mad = np.median(np.abs(log_std - med))
# robust z (Gaussian-calibrated: 1.4826 * MAD ~= sigma)
robust_z = (log_std - med) / (1.4826 * mad if mad > 0 else 1.0)

bad_mask = np.zeros(len(ch_cols), dtype=bool)
bad_reasons = [''] * len(ch_cols)
for i in range(len(ch_cols)):
    reasons = []
    if not np.isfinite(filt_std[i]):
        reasons.append('nan/inf')
    if filt_std[i] < FLAT_STD_ABS:
        reasons.append(f'flat(std={filt_std[i]:.3g})')
    if filt_std[i] > SAT_STD_ABS:
        reasons.append(f'pathological(std={filt_std[i]:.3g})')
    if rail_frac[i] > RAIL_FRAC_THRESH:
        reasons.append(f'railed({rail_frac[i]*100:.0f}%@adc_max)')
    if np.isfinite(robust_z[i]) and robust_z[i] > MAD_Z_THRESH:
        reasons.append(f'loud(z={robust_z[i]:+.1f})')
    elif np.isfinite(robust_z[i]) and robust_z[i] < -MAD_Z_THRESH:
        reasons.append(f'quiet(z={robust_z[i]:+.1f})')
    if reasons:
        bad_mask[i] = True
        bad_reasons[i] = '|'.join(reasons)

n_bad = int(bad_mask.sum())
print(f"[bad] Flagged {n_bad}/{len(ch_cols)} channels  "
      f"(log-std median={med:.3f}, MAD={mad:.3f})  ({time.time()-t0:.1f}s)")

# Write bad_channels.txt
with open(os.path.join(OUT_DIR, 'bad_channels.txt'), 'w', encoding='utf-8') as f:
    f.write(f"Bad channel detection on {os.path.basename(CSV_PATH)}\n")
    f.write(f"Detection run on filtered data (detrend + 1 Hz HPF + 50 Hz LPF + 60 Hz + 70 Hz notch).\n")
    f.write(f"Rules: flat<{FLAT_STD_ABS}  OR  path>{SAT_STD_ABS}  "
            f"OR  rail_frac>{RAIL_FRAC_THRESH*100:.0f}%  "
            f"OR  |robust z on log10(std)|>{MAD_Z_THRESH}\n")
    f.write(f"log10(std) median={med:.3f}  MAD={mad:.3f}  "
            f"(median std = {10**med:.1f} uV)\n")
    f.write(f"Total flagged: {n_bad}/{len(ch_cols)}\n\n")
    per_port_total = {}
    per_port_bad = {}
    for i, info in enumerate(ch_info):
        p = info['port']
        per_port_total[p] = per_port_total.get(p, 0) + 1
        if bad_mask[i]:
            per_port_bad[p] = per_port_bad.get(p, 0) + 1
    f.write("Per-port tally (bad / total):\n")
    for p in sorted(per_port_total.keys()):
        f.write(f"  Port{p}: {per_port_bad.get(p,0):3d} / {per_port_total[p]:3d}\n")
    f.write("\nFlagged channels:\n")
    for i, info in enumerate(ch_info):
        if bad_mask[i]:
            f.write(f"  {info['col']:<22s}  std={filt_std[i]:12.3g}  "
                    f"z={robust_z[i]:+6.2f}  rail={rail_frac[i]*100:5.1f}%  "
                    f"{bad_reasons[i]}\n")

# ─────────────────────────────────────────────────────────────────────────────
# 5. Pearson correlation matrix (336x336)
# ─────────────────────────────────────────────────────────────────────────────
print("[corr] Computing 336 x 336 Pearson correlation ...")
t0 = time.time()
# Railed/flat channels have ~zero variance after detrend → corrcoef returns NaN.
# Compute on full matrix; NaN rows/cols stay NaN and we filter them in grouping.
corr = np.corrcoef(data_filt.T)
print(f"[corr] Shape {corr.shape}, NaN entries: {int(np.isnan(corr).sum())} "
      f"({time.time()-t0:.1f}s)")

# Save the raw matrix (with NaNs) for downstream work
np.save(os.path.join(OUT_DIR, 'correlation_matrix.npy'), corr.astype(np.float32))

# ─────────────────────────────────────────────────────────────────────────────
# 6. Grouping analysis (upper triangle, excluding bad channels)
# ─────────────────────────────────────────────────────────────────────────────
print("[group] Building within-device / within-port / cross-port distributions ...")
t0 = time.time()
N = len(ch_cols)
iu, ju = np.triu_indices(N, k=1)  # upper triangle, no diagonal

same_port = port_id[iu] == port_id[ju]
same_dev  = device_idx[iu] == device_idx[ju]

# good-pair mask: both channels clean
good = (~bad_mask[iu]) & (~bad_mask[ju])

within_device_mask = good & same_port & same_dev
within_port_mask   = good & same_port & (~same_dev)
cross_port_mask    = good & (~same_port)

vals = corr[iu, ju]

within_device_vals = vals[within_device_mask]
within_port_vals   = vals[within_port_mask]
cross_port_vals    = vals[cross_port_mask]

# Drop any lingering NaNs just in case
within_device_vals = within_device_vals[np.isfinite(within_device_vals)]
within_port_vals   = within_port_vals[np.isfinite(within_port_vals)]
cross_port_vals    = cross_port_vals[np.isfinite(cross_port_vals)]

def describe(x, name):
    if len(x) == 0:
        return f"{name:22s}: n=0"
    return (f"{name:22s}: n={len(x):6d}  mean={x.mean():+.4f}  "
            f"median={np.median(x):+.4f}  std={x.std():.4f}  "
            f"q05={np.quantile(x,0.05):+.4f}  q95={np.quantile(x,0.95):+.4f}")

print("[group] Group sizes:", len(within_device_vals), len(within_port_vals), len(cross_port_vals))
print("[group]", describe(within_device_vals, 'within-device'))
print("[group]", describe(within_port_vals,   'within-port/cross-dev'))
print("[group]", describe(cross_port_vals,    'cross-port'))

# Stats
kw_stat, kw_p = stats.kruskal(within_device_vals, within_port_vals, cross_port_vals)
u_wd_wp,  p_wd_wp  = stats.mannwhitneyu(within_device_vals, within_port_vals, alternative='two-sided')
u_wd_cp,  p_wd_cp  = stats.mannwhitneyu(within_device_vals, cross_port_vals,  alternative='two-sided')
u_wp_cp,  p_wp_cp  = stats.mannwhitneyu(within_port_vals,   cross_port_vals,  alternative='two-sided')

# Per-device mean within-device correlation
per_device_mean_corr = {}
per_device_n_good = {}
per_device_n_bad = {}
for d in unique_devices:
    ch_idx = np.where(device_idx == d)[0]
    good_idx = ch_idx[~bad_mask[ch_idx]]
    per_device_n_good[d] = len(good_idx)
    per_device_n_bad[d]  = len(ch_idx) - len(good_idx)
    if len(good_idx) < 2:
        per_device_mean_corr[d] = np.nan
        continue
    sub = corr[np.ix_(good_idx, good_idx)]
    tri = sub[np.triu_indices(len(good_idx), k=1)]
    tri = tri[np.isfinite(tri)]
    per_device_mean_corr[d] = float(tri.mean()) if len(tri) else np.nan

print(f"[group] Stats done ({time.time()-t0:.1f}s)")

# Write group_stats.txt
bar = "-" * 78
with open(os.path.join(OUT_DIR, 'group_stats.txt'), 'w', encoding='utf-8') as f:
    f.write(f"Channel similarity group stats -- {os.path.basename(CSV_PATH)}\n")
    f.write(f"Duration: {duration_s:.1f}s ({n_samples:,} samples @ {FS:g} Hz)\n")
    f.write(f"Channels: {N} (bad={n_bad}, good={N-n_bad})\n")
    f.write(f"Preprocessing: linear detrend -> 1 Hz Butterworth HPF (10th order SOS, sosfiltfilt) -> 50 Hz Butterworth LPF (10th order SOS, sosfiltfilt) -> 60 Hz notch (Q=30) -> 70 Hz notch (Q=30), zero-phase\n")
    f.write("\n")
    f.write("Groups (upper triangle, excluding any pair with a bad channel)\n")
    f.write(bar + "\n")
    f.write(describe(within_device_vals, 'within-device') + "\n")
    f.write(describe(within_port_vals,   'within-port/cross-dev') + "\n")
    f.write(describe(cross_port_vals,    'cross-port') + "\n")
    f.write("\n")
    f.write("Omnibus test\n")
    f.write(bar + "\n")
    f.write(f"  Kruskal-Wallis  H={kw_stat:.3f}  p={kw_p:.3e}\n")
    f.write("\n")
    f.write("Pairwise Mann-Whitney U (two-sided)\n")
    f.write(bar + "\n")
    f.write(f"  within-device vs within-port : U={u_wd_wp:.3e}  p={p_wd_wp:.3e}\n")
    f.write(f"  within-device vs cross-port  : U={u_wd_cp:.3e}  p={p_wd_cp:.3e}\n")
    f.write(f"  within-port   vs cross-port  : U={u_wp_cp:.3e}  p={p_wp_cp:.3e}\n")
    f.write("\n")
    f.write("Per-device mean within-device correlation (good channels only)\n")
    f.write(bar + "\n")
    f.write(f"  {'device':<8s} {'port':<5s} {'n_good':<7s} {'n_bad':<6s} {'mean_corr':<10s}\n")
    for d in unique_devices:
        port_of_d = int(d.split('D')[0].replace('P', ''))
        mc = per_device_mean_corr[d]
        mc_str = f"{mc:+.4f}" if np.isfinite(mc) else "  nan  "
        f.write(f"  {d:<8s} {port_of_d:<5d} {per_device_n_good[d]:<7d} "
                f"{per_device_n_bad[d]:<6d} {mc_str}\n")
    f.write("\n")
    per_port_device_means = {}
    for d in unique_devices:
        p = int(d.split('D')[0].replace('P', ''))
        if np.isfinite(per_device_mean_corr[d]):
            per_port_device_means.setdefault(p, []).append(per_device_mean_corr[d])
    f.write("Per-port summary (device-level means)\n")
    f.write(bar + "\n")
    f.write(f"  {'port':<6s} {'n_dev':<7s} {'mean':<10s} {'min':<10s} {'max':<10s}\n")
    for p in sorted(per_port_device_means.keys()):
        arr = np.asarray(per_port_device_means[p])
        f.write(f"  {p:<6d} {len(arr):<7d} {arr.mean():+.4f}    "
                f"{arr.min():+.4f}    {arr.max():+.4f}\n")

# ─────────────────────────────────────────────────────────────────────────────
# 7. Visualizations
# ─────────────────────────────────────────────────────────────────────────────
print("[plot] Building visualizations ...")

# ── Helper: port/device boundaries in channel-index space
port_boundaries = []
device_boundaries = []
prev_port = port_id[0]
prev_dev_id = device_idx[0]
for i in range(1, N):
    if port_id[i] != prev_port:
        port_boundaries.append(i)
        prev_port = port_id[i]
    if device_idx[i] != prev_dev_id:
        device_boundaries.append(i)
        prev_dev_id = device_idx[i]

# ── Plot 1: full 336×336 correlation heatmap
t0 = time.time()
plt.figure(figsize=(13, 11))
corr_plot = corr.copy()
corr_plot[~np.isfinite(corr_plot)] = 0.0
im = plt.imshow(corr_plot, vmin=-1, vmax=1, cmap='RdBu_r',
                aspect='equal', interpolation='nearest')
for b in port_boundaries:
    plt.axhline(b - 0.5, color='white', linewidth=1.4)
    plt.axvline(b - 0.5, color='white', linewidth=1.4)
for b in device_boundaries:
    if b not in port_boundaries:
        plt.axhline(b - 0.5, color='white', linewidth=0.4, alpha=0.6)
        plt.axvline(b - 0.5, color='white', linewidth=0.4, alpha=0.6)
# Mark bad channels with black ticks
bad_idx = np.where(bad_mask)[0]
if len(bad_idx) > 0:
    plt.scatter(np.full(len(bad_idx), -4), bad_idx,
                marker='s', s=6, color='black', clip_on=False)
    plt.scatter(bad_idx, np.full(len(bad_idx), -4),
                marker='s', s=6, color='black', clip_on=False)
cbar = plt.colorbar(im, fraction=0.04, pad=0.02)
cbar.set_label('Pearson r')
# Port tick labels
port_centers = []
start = 0
for b in port_boundaries + [N]:
    port_centers.append((start + b - 1) / 2)
    start = b
port_labels = [f'P{p}' for p in sorted(set(port_id.tolist()))]
plt.xticks(port_centers, port_labels)
plt.yticks(port_centers, port_labels)
plt.title(f'336×336 Pearson correlation\n'
          f'Filters: detrend + 1 Hz HPF + 50 Hz LPF (Butter10 SOS) + 60/70 Hz notch (zero-phase)  —  '
          f'{n_bad} bad channels marked')
plt.xlabel('Channel index (grouped by port)')
plt.ylabel('Channel index (grouped by port)')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'correlation_matrix_full.png'), dpi=140)
plt.close()
print(f"[plot]   correlation_matrix_full.png ({time.time()-t0:.1f}s)")

# ── Plot 2: similarity histogram
t0 = time.time()
plt.figure(figsize=(11, 6))
bins = np.linspace(-1, 1, 121)
plt.hist(cross_port_vals,    bins=bins, alpha=0.5, color='#777777',
         label=f'cross-port (n={len(cross_port_vals):,}, μ={cross_port_vals.mean():+.3f})',
         density=True)
plt.hist(within_port_vals,   bins=bins, alpha=0.55, color='#d9822b',
         label=f'within-port/cross-dev (n={len(within_port_vals):,}, μ={within_port_vals.mean():+.3f})',
         density=True)
plt.hist(within_device_vals, bins=bins, alpha=0.6, color='#2b7bd9',
         label=f'within-device (n={len(within_device_vals):,}, μ={within_device_vals.mean():+.3f})',
         density=True)
for vals_, c in [(cross_port_vals, '#333333'),
                 (within_port_vals, '#a65200'),
                 (within_device_vals, '#0b4e9c')]:
    plt.axvline(np.median(vals_), color=c, linestyle='--', linewidth=1.2)
plt.xlabel('Pearson r')
plt.ylabel('Density')
plt.title('Channel similarity by group (dashed lines = medians)')
plt.legend(loc='upper left', fontsize=9)
plt.xlim(-1, 1)
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'similarity_histogram.png'), dpi=140)
plt.close()
print(f"[plot]   similarity_histogram.png ({time.time()-t0:.1f}s)")

# ── Plot 3: port boxplot of within-device correlations
t0 = time.time()
per_port_within_dev = {}
for p in sorted(set(port_id.tolist())):
    pair_mask = (port_id[iu] == p) & (port_id[ju] == p) & same_dev & good
    v = vals[pair_mask]
    v = v[np.isfinite(v)]
    per_port_within_dev[p] = v

fig, ax = plt.subplots(figsize=(9, 6))
ports_sorted = sorted(per_port_within_dev.keys())
data_for_box = [per_port_within_dev[p] for p in ports_sorted]
bp = ax.boxplot(data_for_box, labels=[f'Port{p}' for p in ports_sorted],
                showmeans=True, meanline=True, patch_artist=True)
for patch in bp['boxes']:
    patch.set_facecolor('#a6cee3')
ax.axhline(0, color='gray', linewidth=0.5)
ax.set_ylabel('Pearson r (within-device pairs)')
ax.set_title('Within-device channel correlation by port\n'
             '(dashed = mean; solid = median)')
ax.grid(alpha=0.3, axis='y')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'port_boxplot.png'), dpi=140)
plt.close()
print(f"[plot]   port_boxplot.png ({time.time()-t0:.1f}s)")

# ── Plot 4: per-device mean within-device correlation (bar)
t0 = time.time()
fig, ax = plt.subplots(figsize=(14, 5))
dev_names = unique_devices
mean_vals = [per_device_mean_corr[d] if np.isfinite(per_device_mean_corr[d]) else 0.0
             for d in dev_names]
# Color by port
port_colors = {
    1: '#1f77b4', 2: '#ff7f0e', 3: '#2ca02c', 4: '#d62728',
    5: '#9467bd', 6: '#8c564b', 7: '#e377c2',
}
bar_colors = [port_colors[int(d.split('D')[0].replace('P', ''))] for d in dev_names]
x = np.arange(len(dev_names))
ax.bar(x, mean_vals, color=bar_colors, edgecolor='black', linewidth=0.3)
ax.axhline(np.nanmean(list(per_device_mean_corr.values())),
           color='black', linestyle='--', linewidth=1,
           label=f'overall mean={np.nanmean(list(per_device_mean_corr.values())):+.3f}')
ax.set_xticks(x)
ax.set_xticklabels(dev_names, rotation=90, fontsize=7)
ax.set_ylabel('Mean within-device Pearson r')
ax.set_title('Per-device mean within-device correlation '
             '(bar color = port)')
ax.set_ylim(-0.2, 1.05)
ax.grid(alpha=0.3, axis='y')
ax.legend(loc='lower right')
# Port legend
from matplotlib.patches import Patch
port_handles = [Patch(color=port_colors[p], label=f'Port{p}') for p in sorted(port_colors.keys())]
ax.legend(handles=port_handles + [
    plt.Line2D([0], [0], color='black', linestyle='--',
               label=f'overall mean={np.nanmean(list(per_device_mean_corr.values())):+.3f}')
], loc='lower right', ncol=2, fontsize=8)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'device_mean_correlation.png'), dpi=140)
plt.close()
print(f"[plot]   device_mean_correlation.png ({time.time()-t0:.1f}s)")

# ── Plot 5: per-device 8×8 correlation grid (6 rows × 7 cols facet)
t0 = time.time()
# Group devices by port, then rank by dev within port
port_to_devs = {}
for d in unique_devices:
    p = int(d.split('D')[0].replace('P', ''))
    port_to_devs.setdefault(p, []).append(d)
max_dev_per_port = max(len(v) for v in port_to_devs.values())
ports_sorted = sorted(port_to_devs.keys())
n_cols = len(ports_sorted)  # 7
n_rows = max_dev_per_port    # 8 (Port1 has 8)

fig, axes = plt.subplots(n_rows, n_cols, figsize=(n_cols * 2.0, n_rows * 2.0))
for col, p in enumerate(ports_sorted):
    devs_p = port_to_devs[p]
    for row in range(n_rows):
        ax = axes[row, col]
        if row >= len(devs_p):
            ax.axis('off')
            continue
        d = devs_p[row]
        ch_idx = np.where(device_idx == d)[0]
        sub = corr[np.ix_(ch_idx, ch_idx)].copy()
        sub[~np.isfinite(sub)] = 0.0
        ax.imshow(sub, vmin=-1, vmax=1, cmap='RdBu_r', aspect='equal',
                  interpolation='nearest')
        # mark bad channels within this device
        for k, gi in enumerate(ch_idx):
            if bad_mask[gi]:
                ax.add_patch(plt.Rectangle((k - 0.5, -0.5), 1, len(ch_idx),
                                           fill=False, edgecolor='black', lw=0.5))
                ax.add_patch(plt.Rectangle((-0.5, k - 0.5), len(ch_idx), 1,
                                           fill=False, edgecolor='black', lw=0.5))
        ax.set_xticks([]); ax.set_yticks([])
        mc = per_device_mean_corr[d]
        mc_str = f'{mc:+.2f}' if np.isfinite(mc) else 'nan'
        ax.set_title(f'{d}  μ={mc_str}', fontsize=8)
for col, p in enumerate(ports_sorted):
    axes[0, col].set_xlabel(f'Port{p}', fontsize=10, fontweight='bold')
    axes[0, col].xaxis.set_label_position('top')
fig.suptitle('Per-device 8×8 correlation matrices '
             '(one cell = one ADS1299, black outline = bad channel)',
             fontsize=11)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(os.path.join(OUT_DIR, 'per_device_grid.png'), dpi=130)
plt.close()
print(f"[plot]   per_device_grid.png ({time.time()-t0:.1f}s)")

print("[done] All artifacts written to:", OUT_DIR)
