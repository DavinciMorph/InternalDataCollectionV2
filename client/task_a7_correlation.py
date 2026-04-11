"""
Task A.7: Full 336x336 Correlation Matrix Analysis
Compares within-device, within-port, within-SPI-bus, and cross-SPI-bus correlations
between clean and bad regions.
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import time

# ── Load data ──
print("Loading data...")
t0 = time.time()
df = pd.read_csv(r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv")
ts = df['timestamp'].values
print(f"Loaded {len(df)} rows in {time.time()-t0:.1f}s")

# ── Build channel metadata ──
ch_cols = [c for c in df.columns if c.startswith('Port')]
print(f"Total data channels: {len(ch_cols)}")

# Parse channel info
ch_info = []
for col in ch_cols:
    parts = col.split('_')
    port = int(parts[0].replace('Port', ''))
    dev = int(parts[1].replace('dev', ''))
    ch = int(parts[2].replace('ch', ''))
    # SPI bus mapping
    if port in [1, 2]:
        spi = 'SPI0'
    elif port in [3, 4]:
        spi = 'SPI3'
    elif port in [5, 6]:
        spi = 'SPI4'
    elif port == 7:
        spi = 'SPI5'
    else:
        spi = 'unknown'
    ch_info.append({
        'col': col, 'port': port, 'dev': dev, 'ch': ch,
        'spi': spi,
        'device_id': f'P{port}D{dev}',  # unique device identifier
    })

# ── Extract 5-second windows (1250 samples) ──
# Clean: t=1450-1455s
mask_clean = (ts >= 1450.0) & (ts < 1455.0)
idx_clean = np.where(mask_clean)[0]
# Bad: t=1530-1535s
mask_bad = (ts >= 1530.0) & (ts < 1535.0)
idx_bad = np.where(mask_bad)[0]

print(f"\nClean window: {len(idx_clean)} samples (t=1450-1455s)")
print(f"Bad window:   {len(idx_bad)} samples (t=1530-1535s)")

# ── Extract data matrices and mean-subtract (AC coupling) ──
print("\nExtracting and mean-subtracting data...")
data_clean = df[ch_cols].values[idx_clean].astype(np.float64)
data_bad = df[ch_cols].values[idx_bad].astype(np.float64)

# Mean-subtract each channel
data_clean_ac = data_clean - data_clean.mean(axis=0, keepdims=True)
data_bad_ac = data_bad - data_bad.mean(axis=0, keepdims=True)

print(f"Clean data shape: {data_clean_ac.shape}")
print(f"Bad data shape:   {data_bad_ac.shape}")

# ── Compute 336x336 Pearson correlation matrices ──
print("\nComputing correlation matrices...")
t0 = time.time()
corr_clean = np.corrcoef(data_clean_ac.T)  # 336x336
corr_bad = np.corrcoef(data_bad_ac.T)

# Handle NaN (railed channels with zero variance after mean-subtraction)
n_nan_clean = np.sum(np.isnan(corr_clean))
n_nan_bad = np.sum(np.isnan(corr_bad))
print(f"NaN entries: clean={n_nan_clean}, bad={n_nan_bad}")
if n_nan_clean > 0 or n_nan_bad > 0:
    print("  (NaN = zero-variance channels, will be excluded from stats)")

print(f"Correlation matrices computed in {time.time()-t0:.1f}s")

# ── Build grouping masks ──
N = len(ch_info)
print(f"\nBuilding grouping masks for {N} channels...")

# Pre-compute grouping arrays
ports = np.array([c['port'] for c in ch_info])
devs = np.array([c['device_id'] for c in ch_info])
spis = np.array([c['spi'] for c in ch_info])

# Masks (upper triangle only, no diagonal)
mask_within_device = np.zeros((N, N), dtype=bool)
mask_within_port = np.zeros((N, N), dtype=bool)
mask_within_spi = np.zeros((N, N), dtype=bool)
mask_cross_spi = np.zeros((N, N), dtype=bool)

for i in range(N):
    for j in range(i+1, N):
        same_device = devs[i] == devs[j]
        same_port = ports[i] == ports[j]
        same_spi = spis[i] == spis[j]

        if same_device:
            mask_within_device[i, j] = True
        elif same_port:
            mask_within_port[i, j] = True  # same port, different device
        elif same_spi:
            mask_within_spi[i, j] = True   # same SPI bus, different port
        else:
            mask_cross_spi[i, j] = True     # different SPI bus

n_within_dev = mask_within_device.sum()
n_within_port = mask_within_port.sum()
n_within_spi = mask_within_spi.sum()
n_cross_spi = mask_cross_spi.sum()
print(f"  Within-device pairs:   {n_within_dev:,}")
print(f"  Within-port pairs:     {n_within_port:,}")
print(f"  Within-SPI-bus pairs:  {n_within_spi:,}")
print(f"  Cross-SPI-bus pairs:   {n_cross_spi:,}")
print(f"  Total unique pairs:    {n_within_dev + n_within_port + n_within_spi + n_cross_spi:,} (expected {N*(N-1)//2:,})")

# ── Compute summary statistics ──
def corr_stats(corr_matrix, mask, name):
    """Compute mean |r|, mean r, std r for a group."""
    vals = corr_matrix[mask]
    vals = vals[~np.isnan(vals)]
    if len(vals) == 0:
        return {'name': name, 'n': 0, 'mean_abs_r': np.nan, 'mean_r': np.nan, 'std_r': np.nan,
                'median_abs_r': np.nan, 'pct_above_0.9': np.nan}
    return {
        'name': name,
        'n': len(vals),
        'mean_abs_r': np.mean(np.abs(vals)),
        'mean_r': np.mean(vals),
        'std_r': np.std(vals),
        'median_abs_r': np.median(np.abs(vals)),
        'pct_above_0.9': 100.0 * np.mean(np.abs(vals) > 0.9),
        'pct_above_0.5': 100.0 * np.mean(np.abs(vals) > 0.5),
        'min_r': np.min(vals),
        'max_r': np.max(vals),
    }

print("\n" + "="*120)
print("CORRELATION ANALYSIS RESULTS")
print("="*120)

groups = [
    ('Within-device', mask_within_device),
    ('Within-port (cross-device)', mask_within_port),
    ('Within-SPI-bus (cross-port)', mask_within_spi),
    ('Cross-SPI-bus', mask_cross_spi),
]

for region_name, corr_mat in [('CLEAN (t=1450-1455s)', corr_clean), ('BAD (t=1530-1535s)', corr_bad)]:
    print(f"\n{'-'*60}")
    print(f"  {region_name}")
    print(f"{'-'*60}")
    print(f"  {'Group':<35} {'N pairs':>8} {'Mean|r|':>10} {'Med|r|':>10} {'Mean r':>10} {'Std r':>10} {'%|r|>0.9':>10} {'%|r|>0.5':>10} {'Min r':>10} {'Max r':>10}")
    print(f"  {'-'*115}")
    for gname, gmask in groups:
        s = corr_stats(corr_mat, gmask, gname)
        print(f"  {gname:<35} {s['n']:>8,} {s['mean_abs_r']:>10.4f} {s['median_abs_r']:>10.4f} {s['mean_r']:>10.4f} {s['std_r']:>10.4f} {s['pct_above_0.9']:>10.1f} {s['pct_above_0.5']:>10.1f} {s['min_r']:>10.4f} {s['max_r']:>10.4f}")

# ── Per-SPI-bus breakdown in bad region ──
print("\n" + "="*120)
print("PER-SPI-BUS BREAKDOWN (BAD REGION)")
print("="*120)

spi_names = ['SPI0', 'SPI3', 'SPI4', 'SPI5']
for spi_a in spi_names:
    for spi_b in spi_names:
        if spi_a > spi_b:
            continue
        mask_ab = np.zeros((N, N), dtype=bool)
        for i in range(N):
            for j in range(i+1, N):
                if (spis[i] == spi_a and spis[j] == spi_b) or (spis[i] == spi_b and spis[j] == spi_a):
                    mask_ab[i, j] = True
        n_pairs = mask_ab.sum()
        if n_pairs == 0:
            continue
        vals = corr_bad[mask_ab]
        vals = vals[~np.isnan(vals)]
        if len(vals) == 0:
            continue
        label = f"{spi_a} x {spi_b}" if spi_a != spi_b else f"{spi_a} (within)"
        print(f"  {label:<25} {len(vals):>8,} pairs  mean|r|={np.mean(np.abs(vals)):.4f}  mean_r={np.mean(vals):.4f}  %|r|>0.9={100*np.mean(np.abs(vals)>0.9):.1f}%")

# ── CRITICAL QUESTION ──
print("\n" + "="*120)
print("CRITICAL QUESTION: Is cross-SPI-bus correlation ~1.0 in bad region?")
print("="*120)
vals_cross_bad = corr_bad[mask_cross_spi]
vals_cross_bad = vals_cross_bad[~np.isnan(vals_cross_bad)]
vals_within_spi_bad = corr_bad[mask_within_spi]
vals_within_spi_bad = vals_within_spi_bad[~np.isnan(vals_within_spi_bad)]

mean_cross = np.mean(np.abs(vals_cross_bad))
mean_within = np.mean(np.abs(vals_within_spi_bad))
print(f"  Cross-SPI-bus mean |r| (bad):  {mean_cross:.4f}")
print(f"  Within-SPI-bus mean |r| (bad): {mean_within:.4f}")

if mean_cross > 0.8:
    print(f"\n  >> ANSWER: Cross-SPI-bus correlation is HIGH ({mean_cross:.3f})")
    print(f"  >> The noise source is UPSTREAM of all SPI buses")
    print(f"  >> This points to: POWER SUPPLY / ANALOG GROUND SHIFT / AVDD/AVSS instability")
elif mean_cross > 0.5:
    print(f"\n  >> ANSWER: Cross-SPI-bus correlation is MODERATE ({mean_cross:.3f})")
    print(f"  >> Partially shared noise source (power) + partially per-bus (SPI/digital)")
else:
    print(f"\n  >> ANSWER: Cross-SPI-bus correlation is LOW ({mean_cross:.3f})")
    print(f"  >> The noise is per-SPI-bus, NOT a global power supply issue")
    print(f"  >> This points to: per-bus SPI read errors / digital noise")

# ── Compare clean vs bad delta ──
print("\n" + "="*120)
print("CLEAN vs BAD COMPARISON (delta)")
print("="*120)
for gname, gmask in groups:
    s_clean = corr_stats(corr_clean, gmask, gname)
    s_bad = corr_stats(corr_bad, gmask, gname)
    delta = s_bad['mean_abs_r'] - s_clean['mean_abs_r']
    print(f"  {gname:<35} clean={s_clean['mean_abs_r']:.4f}  bad={s_bad['mean_abs_r']:.4f}  delta={delta:+.4f}")

# ── Plot correlation matrices ──
print("\nGenerating correlation matrix plots...")
fig, axes = plt.subplots(1, 2, figsize=(20, 9))

# Clean
im0 = axes[0].imshow(corr_clean, cmap='RdBu_r', vmin=-1, vmax=1, aspect='auto', interpolation='nearest')
axes[0].set_title('Clean Region (t=1450-1455s)', fontsize=12)
axes[0].set_xlabel('Channel index')
axes[0].set_ylabel('Channel index')
plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04, label='Pearson r')

# Add port boundaries
port_sizes = [64, 56, 40, 40, 40, 40, 56]
port_labels = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']
cumsum = np.cumsum([0] + port_sizes)
for ax in axes:
    for boundary in cumsum[1:-1]:
        ax.axhline(boundary-0.5, color='white', linewidth=0.5, alpha=0.7)
        ax.axvline(boundary-0.5, color='white', linewidth=0.5, alpha=0.7)
    # Label ports
    for k, (start, end) in enumerate(zip(cumsum[:-1], cumsum[1:])):
        mid = (start + end) / 2
        ax.text(mid, -5, port_labels[k], ha='center', fontsize=7, fontweight='bold')

# Bad
im1 = axes[1].imshow(corr_bad, cmap='RdBu_r', vmin=-1, vmax=1, aspect='auto', interpolation='nearest')
axes[1].set_title('Bad Region (t=1530-1535s)', fontsize=12)
axes[1].set_xlabel('Channel index')
axes[1].set_ylabel('Channel index')
plt.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04, label='Pearson r')

plt.suptitle('Task A.7: 336x336 Channel Correlation Matrix', fontsize=14, fontweight='bold')
plt.tight_layout()
outpath = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\task_a7_correlation.png"
plt.savefig(outpath, dpi=150, bbox_inches='tight')
print(f"Plot saved to {outpath}")

# ── Histogram of correlations ──
fig2, axes2 = plt.subplots(2, 2, figsize=(14, 10))
fig2.suptitle('Correlation Distributions by Group', fontsize=13, fontweight='bold')

for idx, (gname, gmask) in enumerate(groups):
    ax = axes2[idx // 2][idx % 2]
    vals_c = corr_clean[gmask]
    vals_b = corr_bad[gmask]
    vals_c = vals_c[~np.isnan(vals_c)]
    vals_b = vals_b[~np.isnan(vals_b)]
    ax.hist(vals_c, bins=100, alpha=0.5, color='tab:blue', label='Clean', density=True)
    ax.hist(vals_b, bins=100, alpha=0.5, color='tab:red', label='Bad', density=True)
    ax.set_title(gname)
    ax.set_xlabel('Pearson r')
    ax.set_ylabel('Density')
    ax.legend(fontsize=8)
    ax.set_xlim(-1, 1)

plt.tight_layout()
outpath2 = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\task_a7_corr_histograms.png"
plt.savefig(outpath2, dpi=150, bbox_inches='tight')
print(f"Histogram plot saved to {outpath2}")

plt.close('all')
print("\nTask A.7 complete.")
