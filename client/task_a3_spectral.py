"""
Task A.3: Spectral Analysis (PSD comparison: clean vs bad regions)
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import welch

# ── Load data ──
print("Loading data...")
df = pd.read_csv(r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\FirmwareIssue1.csv")
ts = df['timestamp'].values
print(f"Loaded {len(df)} rows, timestamps {ts[0]:.3f} - {ts[-1]:.3f}s")

FS = 250.0  # Hz

# ── Select 5 representative non-railed channels ──
candidate_channels = [
    'Port1_dev1_ch1',   # SPI0, Port1
    'Port2_dev1_ch1',   # SPI0, Port2
    'Port3_dev1_ch1',   # SPI3, Port3
    'Port5_dev1_ch1',   # SPI4, Port5
    'Port7_dev1_ch1',   # SPI5, Port7
    # backups
    'Port1_dev3_ch1',
    'Port4_dev1_ch1',
    'Port6_dev1_ch1',
]

# Check for railed channels in clean region
mask_clean = (ts >= 1400.0) & (ts <= 1470.0)
idx_clean = np.where(mask_clean)[0]

RAIL_THRESH = 8300000

selected = []
print("\nChannel selection (checking for railed values in clean region):")
for col in candidate_channels:
    vals = df[col].values[idx_clean].astype(np.float64)
    n_railed = np.sum(np.abs(vals) > RAIL_THRESH)
    pct_railed = 100.0 * n_railed / len(vals)
    status = "RAILED" if pct_railed > 5 else "OK"
    print(f"  {col}: {n_railed}/{len(vals)} railed ({pct_railed:.1f}%) -> {status}")
    if status == "OK" and len(selected) < 5:
        selected.append(col)

if len(selected) < 5:
    # Relax threshold
    for col in candidate_channels:
        if col not in selected:
            selected.append(col)
            if len(selected) >= 5:
                break
    print(f"  (relaxed selection to get 5 channels)")

print(f"\nSelected channels: {selected}")

# ── Extract regions ──
mask_bad = (ts >= 1500.0) & (ts <= 1590.0)
idx_bad = np.where(mask_bad)[0]
print(f"\nClean region: {len(idx_clean)} samples (t=1400-1470s)")
print(f"Bad region:   {len(idx_bad)} samples (t=1500-1590s)")

# ── Compute Welch PSD ──
NPERSEG = 1024

spi_bus_map = {}
for col in df.columns[2:]:
    port_num = int(col.split('_')[0].replace('Port', ''))
    if port_num in [1, 2]:
        spi_bus_map[col] = 'SPI0'
    elif port_num in [3, 4]:
        spi_bus_map[col] = 'SPI3'
    elif port_num in [5, 6]:
        spi_bus_map[col] = 'SPI4'
    elif port_num == 7:
        spi_bus_map[col] = 'SPI5'

# ADS1299 at gain=24, Vref=4.5V: LSB = 9V / (24 * 2^24) in V, convert to uV
LSB_UV = 9.0 / (24.0 * 2**24) * 1e6  # ~0.02235 uV/LSB

results = {}
print("\n" + "="*100)
print("WELCH PSD ANALYSIS")
print("="*100)

fig, axes = plt.subplots(len(selected), 1, figsize=(14, 4*len(selected)))
if len(selected) == 1:
    axes = [axes]

for i, col in enumerate(selected):
    bus = spi_bus_map.get(col, 'unknown')
    data_clean = df[col].values[idx_clean].astype(np.float64)
    data_bad = df[col].values[idx_bad].astype(np.float64)

    f_clean, psd_clean = welch(data_clean, fs=FS, nperseg=NPERSEG, noverlap=NPERSEG//2)
    f_bad, psd_bad = welch(data_bad, fs=FS, nperseg=NPERSEG, noverlap=NPERSEG//2)

    # Convert PSD to uV^2/Hz
    psd_clean_uv = psd_clean * (LSB_UV ** 2)
    psd_bad_uv = psd_bad * (LSB_UV ** 2)

    # ── Frequency bands ──
    bands = {
        '0-5 Hz':   (0, 5),
        '5-30 Hz':  (5, 30),
        '30-60 Hz': (30, 60),
        '60-125 Hz': (60, 125),
    }

    print(f"\n--- {col} ({bus}) ---")
    print(f"  Clean: mean={np.mean(data_clean):,.0f} LSB, std={np.std(data_clean):,.0f} LSB ({np.std(data_clean)*LSB_UV:.1f} uV)")
    print(f"  Bad:   mean={np.mean(data_bad):,.0f} LSB, std={np.std(data_bad):,.0f} LSB ({np.std(data_bad)*LSB_UV:.1f} uV)")
    print(f"  DC offset shift: {np.mean(data_bad) - np.mean(data_clean):,.0f} LSB ({(np.mean(data_bad)-np.mean(data_clean))*LSB_UV:,.1f} uV)")
    print(f"  Noise increase: {np.std(data_bad)/np.std(data_clean):.1f}x")

    print(f"\n  {'Band':<15} {'Clean (uV^2/Hz)':>18} {'Bad (uV^2/Hz)':>18} {'Ratio (bad/clean)':>18}")
    print(f"  {'-'*75}")
    band_results = {}
    for bname, (f_lo, f_hi) in bands.items():
        mask_f = (f_clean >= f_lo) & (f_clean < f_hi)
        p_clean = np.trapz(psd_clean_uv[mask_f], f_clean[mask_f]) if np.any(mask_f) else 0
        p_bad = np.trapz(psd_bad_uv[mask_f], f_bad[mask_f]) if np.any(mask_f) else 0
        ratio = p_bad / p_clean if p_clean > 0 else float('inf')
        band_results[bname] = {'clean': p_clean, 'bad': p_bad, 'ratio': ratio}
        print(f"  {bname:<15} {p_clean:>18.2f} {p_bad:>18.2f} {ratio:>18.1f}x")

    total_clean = np.trapz(psd_clean_uv, f_clean)
    total_bad = np.trapz(psd_bad_uv, f_bad)
    print(f"  {'TOTAL':<15} {total_clean:>18.2f} {total_bad:>18.2f} {total_bad/total_clean if total_clean > 0 else float('inf'):>18.1f}x")

    # ── Check specific peaks ──
    print(f"\n  Peak check at specific frequencies:")
    for target_f in [50, 60, 70]:
        idx_f = np.argmin(np.abs(f_clean - target_f))
        lo = max(0, idx_f-2)
        hi = min(len(psd_clean_uv), idx_f+3)
        peak_clean = np.max(psd_clean_uv[lo:hi])
        peak_bad = np.max(psd_bad_uv[lo:hi])
        # Compare to local median
        med_lo = max(0, idx_f-10)
        med_hi = min(len(psd_clean_uv), idx_f+11)
        median_clean = np.median(psd_clean_uv[med_lo:med_hi])
        median_bad = np.median(psd_bad_uv[med_lo:med_hi])
        snr_clean = peak_clean / median_clean if median_clean > 0 else 0
        snr_bad = peak_bad / median_bad if median_bad > 0 else 0
        print(f"    {target_f} Hz: clean={peak_clean:.4f} uV^2/Hz (SNR={snr_clean:.1f}x), "
              f"bad={peak_bad:.4f} uV^2/Hz (SNR={snr_bad:.1f}x), "
              f"abs_ratio={peak_bad/peak_clean if peak_clean > 0 else float('inf'):.1f}x")

    # ── Noise character: 1/f^alpha fit ──
    mask_fit = (f_clean >= 1) & (f_clean <= 100)
    log_f = np.log10(f_clean[mask_fit])
    log_psd_clean_fit = np.log10(psd_clean_uv[mask_fit] + 1e-30)
    log_psd_bad_fit = np.log10(psd_bad_uv[mask_fit] + 1e-30)

    coef_clean = np.polyfit(log_f, log_psd_clean_fit, 1)
    coef_bad = np.polyfit(log_f, log_psd_bad_fit, 1)
    alpha_clean = -coef_clean[0]
    alpha_bad = -coef_bad[0]
    print(f"\n  1/f^alpha fit (1-100 Hz): clean alpha={alpha_clean:.2f}, bad alpha={alpha_bad:.2f}")
    if alpha_bad < 0.3:
        noise_type = "WHITE (flat spectrum)"
    elif alpha_bad < 1.5:
        noise_type = f"PINK (1/f, alpha={alpha_bad:.2f})"
    else:
        noise_type = f"RED/BROWN (1/f^{alpha_bad:.1f})"
    print(f"  Bad region noise character: {noise_type}")

    results[col] = {
        'bus': bus,
        'bands': band_results,
        'total_ratio': total_bad / total_clean if total_clean > 0 else float('inf'),
        'alpha_clean': alpha_clean,
        'alpha_bad': alpha_bad,
        'noise_type': noise_type,
    }

    # ── Plot ──
    ax = axes[i]
    ax.semilogy(f_clean, psd_clean_uv, color='tab:blue', linewidth=0.8, alpha=0.8, label='Clean (1400-1470s)')
    ax.semilogy(f_bad, psd_bad_uv, color='tab:red', linewidth=0.8, alpha=0.8, label='Bad (1500-1590s)')
    # Show ratio on secondary y-axis
    for target_f in [50, 60, 70]:
        ax.axvline(target_f, color='gray', linestyle=':', alpha=0.4, linewidth=0.5)
        ax.text(target_f+0.5, ax.get_ylim()[1]*0.5, f'{target_f}Hz', fontsize=7, alpha=0.5)
    ax.set_title(f'{col} ({bus})')
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('PSD (uV^2/Hz)')
    ax.legend(fontsize=8)
    ax.set_xlim(0, 125)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
outpath = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\task_a3_spectral.png"
plt.savefig(outpath, dpi=150, bbox_inches='tight')
print(f"\nPlot saved to {outpath}")

# ── Cross-channel summary ──
print("\n" + "="*100)
print("CROSS-CHANNEL SUMMARY")
print("="*100)
print(f"{'Channel':<22} {'Bus':<6} {'Total':>8} {'0-5Hz':>10} {'5-30Hz':>10} {'30-60Hz':>10} {'60-125Hz':>10} {'a_clean':>8} {'a_bad':>8} {'Noise Type':<20}")
print("-"*120)
for col, r in results.items():
    b = r['bands']
    print(f"{col:<22} {r['bus']:<6} {r['total_ratio']:>8.1f}x {b['0-5 Hz']['ratio']:>10.1f}x {b['5-30 Hz']['ratio']:>10.1f}x {b['30-60 Hz']['ratio']:>10.1f}x {b['60-125 Hz']['ratio']:>10.1f}x {r['alpha_clean']:>8.2f} {r['alpha_bad']:>8.2f} {r['noise_type']:<20}")

# ── Overall assessment ──
print("\n" + "="*100)
print("NOISE CHARACTER ASSESSMENT")
print("="*100)
avg_alpha_bad = np.mean([r['alpha_bad'] for r in results.values()])
avg_alpha_clean = np.mean([r['alpha_clean'] for r in results.values()])
avg_ratio = np.mean([r['total_ratio'] for r in results.values()])
print(f"Average spectral slope: clean={avg_alpha_clean:.2f}, bad={avg_alpha_bad:.2f}")
print(f"Average total power ratio (bad/clean): {avg_ratio:.1f}x")
print(f"\nPer-band average ratios:")
for bname in ['0-5 Hz', '5-30 Hz', '30-60 Hz', '60-125 Hz']:
    avg_r = np.mean([r['bands'][bname]['ratio'] for r in results.values()])
    print(f"  {bname}: {avg_r:.1f}x")

# Determine which band has highest increase
band_avgs = {}
for bname in ['0-5 Hz', '5-30 Hz', '30-60 Hz', '60-125 Hz']:
    band_avgs[bname] = np.mean([r['bands'][bname]['ratio'] for r in results.values()])
max_band = max(band_avgs, key=band_avgs.get)
print(f"\nDominant noise increase band: {max_band} ({band_avgs[max_band]:.1f}x)")

if band_avgs['60-125 Hz'] > 2 * band_avgs['0-5 Hz']:
    print(">> High-frequency noise dominates -- suggests EMI/switching noise source")
elif band_avgs['0-5 Hz'] > 2 * band_avgs['60-125 Hz']:
    print(">> Low-frequency noise dominates -- suggests DC drift / reference instability")
else:
    print(">> Broadband increase -- suggests analog ground/power shift affecting all frequencies")

plt.close('all')
print("\nTask A.3 complete.")
