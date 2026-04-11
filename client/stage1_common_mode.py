"""
Stage 1: Common-Mode Source Diagnostic (336-channel EEG)
========================================================

Characterize the non-neural common-mode signal that couples 5 of 7 ports
together (r ~= 0.53). The 1 Hz HPF + 50 Hz LPF did not remove it, so the
source lives between ~1 and 50 Hz.

This stage computes the common-mode signal = mean across good channels,
along with per-sample std across good channels, and characterizes:
  - time domain (full, 30s, 10s, 3s zoom)
  - PSD (log-log full, linear 0.5-30 Hz zoom with peaks)
  - amplitude histogram with kurtosis/skewness
  - |cm| vs ch_std relationship

Filter chain (matches client/channel_similarity.py exactly):
  1. scipy.signal.detrend(axis=0, type='linear')
  2. 1 Hz HPF: butter(10, 1.0, 'high', fs=250, sos) -> sosfiltfilt
  3. 50 Hz LPF: butter(10, 50.0, 'low', fs=250, sos) -> sosfiltfilt
  4. 60 Hz notch: iirnotch(60, Q=30, fs=250) -> filtfilt
  5. 70 Hz notch: iirnotch(70, Q=30, fs=250) -> filtfilt
"""
import os
import re
import sys
import time
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal as sig
from scipy import stats

# ----------------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------------
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
BAD_TXT  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis\bad_channels.txt"
OUT_DIR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis"
FS       = 250.0

os.makedirs(OUT_DIR, exist_ok=True)
print(f"[init] Output directory: {OUT_DIR}")

# ----------------------------------------------------------------------------
# 1. Load CSV
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
duration_s = n_samples / FS
print(f"[load] {n_samples:,} rows, {len(ch_cols)} channels "
      f"({time.time()-t0:.1f}s)")
print(f"[load] Duration: {duration_s:.1f}s ({duration_s/60:.2f} min)")

print("[load] Extracting raw matrix ...")
t0 = time.time()
data = df[ch_cols].to_numpy(dtype=np.float64, copy=True)  # float64 for filter stability
del df  # free pandas frame
print(f"[load] Raw matrix: {data.shape} ({data.nbytes/1e6:.1f} MB, {time.time()-t0:.1f}s)")

# ----------------------------------------------------------------------------
# 2. Filter chain — detrend -> 1 Hz HPF -> 50 Hz LPF -> 60 Hz notch -> 70 Hz notch
# ----------------------------------------------------------------------------
print("[filt] Linear detrend ...")
t0 = time.time()
data = sig.detrend(data, axis=0, type='linear')
print(f"[filt] Detrend done ({time.time()-t0:.1f}s)")

print("[filt] 1 Hz Butterworth HPF (10th order SOS, sosfiltfilt) ...")
t0 = time.time()
hpf_sos = sig.butter(10, 1.0, btype='high', fs=FS, output='sos')
data = sig.sosfiltfilt(hpf_sos, data, axis=0)
print(f"[filt] HPF done ({time.time()-t0:.1f}s)")

print("[filt] 50 Hz Butterworth LPF (10th order SOS, sosfiltfilt) ...")
t0 = time.time()
lpf_sos = sig.butter(10, 50.0, btype='low', fs=FS, output='sos')
data = sig.sosfiltfilt(lpf_sos, data, axis=0)
print(f"[filt] LPF done ({time.time()-t0:.1f}s)")

print("[filt] 60 Hz notch (filtfilt) ...")
t0 = time.time()
b60, a60 = sig.iirnotch(60.0, Q=30.0, fs=FS)
data = sig.filtfilt(b60, a60, data, axis=0)
print(f"[filt] 60 Hz done ({time.time()-t0:.1f}s)")

print("[filt] 70 Hz notch (filtfilt) ...")
t0 = time.time()
b70, a70 = sig.iirnotch(70.0, Q=30.0, fs=FS)
data = sig.filtfilt(b70, a70, data, axis=0)
print(f"[filt] 70 Hz done ({time.time()-t0:.1f}s)")

# ----------------------------------------------------------------------------
# 3. Parse bad_channels.txt and build good-channel mask
# ----------------------------------------------------------------------------
print(f"[bad]  Parsing {BAD_TXT} ...")
t0 = time.time()
bad_names = set()
bad_pat = re.compile(r'^\s*(Port\d+_dev\d+_ch\d+)\b')
with open(BAD_TXT, 'r', encoding='utf-8') as f:
    in_list = False
    for line in f:
        if 'Flagged channels' in line:
            in_list = True
            continue
        if not in_list:
            continue
        m = bad_pat.match(line)
        if m:
            bad_names.add(m.group(1))

good_mask = np.array([c not in bad_names for c in ch_cols], dtype=bool)
n_good = int(good_mask.sum())
n_bad  = int((~good_mask).sum())
print(f"[bad]  Bad listed: {len(bad_names)}  "
      f"matched: {n_bad}/{len(ch_cols)}  good: {n_good}  ({time.time()-t0:.1f}s)")
if n_bad != 67:
    print(f"[bad]  WARNING: expected 67 bad channels, found {n_bad}")

# ----------------------------------------------------------------------------
# 4. Common-mode signal and per-sample channel std
# ----------------------------------------------------------------------------
print("[cm]   Computing common-mode signal across good channels ...")
t0 = time.time()
good_idx = np.where(good_mask)[0]
cm     = np.mean(data[:, good_idx], axis=1).astype(np.float64)    # (N,)
ch_std = np.std (data[:, good_idx], axis=1).astype(np.float64)    # (N,)
print(f"[cm]   cm.shape={cm.shape}  ch_std.shape={ch_std.shape}  "
      f"({time.time()-t0:.1f}s)")

# Save for downstream stages
np.save(os.path.join(OUT_DIR, 'stage1_common_mode.npy'), cm.astype(np.float32))
np.save(os.path.join(OUT_DIR, 'stage1_channel_std.npy'), ch_std.astype(np.float32))
print("[cm]   Saved stage1_common_mode.npy, stage1_channel_std.npy")

# Free full filtered matrix — everything below only needs cm and ch_std
del data

# ----------------------------------------------------------------------------
# 5. Statistics
# ----------------------------------------------------------------------------
cm_mean = float(np.mean(cm))
cm_std  = float(np.std(cm))
cm_min  = float(np.min(cm))
cm_max  = float(np.max(cm))
cm_p95  = float(np.percentile(np.abs(cm), 95))
cm_kurt = float(stats.kurtosis(cm, fisher=False))   # Pearson kurtosis (Normal = 3)
cm_skew = float(stats.skew(cm))

print(f"[stat] CM mean={cm_mean:.3f} std={cm_std:.3f} min={cm_min:.1f} max={cm_max:.1f}")
print(f"[stat] CM |P95|={cm_p95:.1f}  kurtosis={cm_kurt:.3f}  skewness={cm_skew:.3f}")

# ----------------------------------------------------------------------------
# 6. PSD (Welch)
# ----------------------------------------------------------------------------
print("[psd]  Welch PSD on common-mode ...")
t0 = time.time()
f_psd, p_psd = sig.welch(cm, fs=FS, nperseg=4096, scaling='density')
print(f"[psd]  PSD shape={p_psd.shape}  freq range=[{f_psd[0]:.2f}, {f_psd[-1]:.2f}] Hz  "
      f"({time.time()-t0:.1f}s)")

# Band powers via trapezoid integration
def band_power(f, p, lo, hi):
    mask = (f >= lo) & (f < hi)
    if mask.sum() < 2:
        return 0.0
    return float(np.trapz(p[mask], f[mask]))

total_power = band_power(f_psd, p_psd, 0.5, 50.0)
bands = {
    'delta (0.5-4 Hz)':   band_power(f_psd, p_psd, 0.5,  4.0),
    'theta (4-8 Hz)':     band_power(f_psd, p_psd, 4.0,  8.0),
    'alpha (8-13 Hz)':    band_power(f_psd, p_psd, 8.0, 13.0),
    'beta  (13-30 Hz)':   band_power(f_psd, p_psd, 13.0, 30.0),
    'gamma (30-50 Hz)':   band_power(f_psd, p_psd, 30.0, 50.0),
}
band_frac = {k: (v / total_power if total_power > 0 else 0.0) for k, v in bands.items()}
for k, frac in band_frac.items():
    print(f"[band] {k:22s} {bands[k]:12.3e}  ({frac*100:5.1f}%)")

# Peak detection on log PSD within 0.5-50 Hz
search_mask = (f_psd >= 0.5) & (f_psd <= 50.0)
f_search = f_psd[search_mask]
p_search = p_psd[search_mask]
log_p = np.log10(np.maximum(p_search, 1e-20))
# prominence relative to median noise floor
prom_thresh = max(0.3, (log_p.max() - np.median(log_p)) * 0.15)
pk_idx, pk_props = sig.find_peaks(log_p, prominence=prom_thresh, distance=3)
# Sort peaks by amplitude (descending)
if len(pk_idx) > 0:
    pk_order = np.argsort(p_search[pk_idx])[::-1]
    peak_freqs = f_search[pk_idx[pk_order]]
    peak_amps  = p_search[pk_idx[pk_order]]
else:
    peak_freqs = np.array([])
    peak_amps  = np.array([])

top_peaks = list(zip(peak_freqs[:10].tolist(), peak_amps[:10].tolist()))
print(f"[peak] Found {len(pk_idx)} peaks in 0.5-50 Hz, top 10:")
for i, (f, a) in enumerate(top_peaks, 1):
    print(f"[peak]   {i:2d}. {f:6.2f} Hz  amp={a:12.3e}")

# |cm| vs ch_std relationship
abs_cm = np.abs(cm)
if np.std(ch_std) > 0 and np.std(abs_cm) > 0:
    cm_chstd_r = float(np.corrcoef(abs_cm, ch_std)[0, 1])
else:
    cm_chstd_r = float('nan')
# linear fit for scatter
fit_slope, fit_intercept = np.polyfit(ch_std, abs_cm, 1)
ratio_chstd_to_abscm = float(np.median(ch_std) / max(np.median(abs_cm), 1e-9))
frac_cm_dominant = float(np.mean(abs_cm > ch_std))
print(f"[rel]  corr(|cm|, ch_std) = {cm_chstd_r:+.4f}")
print(f"[rel]  median(ch_std)/median(|cm|) = {ratio_chstd_to_abscm:.3f}")
print(f"[rel]  fraction samples with |cm| > ch_std = {frac_cm_dominant:.4f}")

# ----------------------------------------------------------------------------
# 7. Plots
# ----------------------------------------------------------------------------
t_axis = np.arange(n_samples) / FS

# ---- Plot 1: time domain (4 panels)
print("[plot] stage1_common_mode_timedomain.png ...")
tp = time.time()
fig, axes = plt.subplots(4, 1, figsize=(13, 11))

axes[0].plot(t_axis, cm, color='#1f77b4', linewidth=0.4)
axes[0].set_title(f'Common-mode signal — full recording ({duration_s:.1f}s, '
                  f'mean across {n_good} good channels)')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Amplitude (uV)')
axes[0].grid(alpha=0.3)

m30 = t_axis <= 30.0
axes[1].plot(t_axis[m30], cm[m30], color='#1f77b4', linewidth=0.6)
axes[1].set_title('First 30 s zoom')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Amplitude (uV)')
axes[1].grid(alpha=0.3)

m10 = t_axis <= 10.0
axes[2].plot(t_axis[m10], cm[m10], color='#1f77b4', linewidth=0.8)
axes[2].set_title('First 10 s zoom')
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('Amplitude (uV)')
axes[2].grid(alpha=0.3)

m3 = t_axis <= 3.0
axes[3].plot(t_axis[m3], cm[m3], color='#c0392b', linewidth=1.0)
axes[3].set_title('First 3 s zoom (check for QRS-like spikes or rhythmic structure)')
axes[3].set_xlabel('Time (s)')
axes[3].set_ylabel('Amplitude (uV)')
axes[3].grid(alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage1_common_mode_timedomain.png'), dpi=140)
plt.close()
print(f"[plot]   saved ({time.time()-tp:.1f}s)")

# ---- Plot 2: PSD (2 panels)
print("[plot] stage1_common_mode_psd.png ...")
tp = time.time()
fig, axes = plt.subplots(2, 1, figsize=(13, 10))

# (a) full log-log
m_full = (f_psd >= 0.5) & (f_psd <= 125.0)
axes[0].loglog(f_psd[m_full], p_psd[m_full], color='#1f77b4', linewidth=1.2)
axes[0].set_title('Common-mode PSD — full band (0.5-125 Hz), log-log')
axes[0].set_xlabel('Frequency (Hz)')
axes[0].set_ylabel('PSD (uV^2/Hz)')
axes[0].grid(alpha=0.3, which='both')
axes[0].set_xlim(0.5, 125)

# annotate peaks on log-log
for f, a in top_peaks[:5]:
    axes[0].axvline(f, color='red', alpha=0.25, linewidth=0.8)
    axes[0].annotate(f'{f:.2f} Hz', xy=(f, a),
                     xytext=(4, 4), textcoords='offset points',
                     fontsize=8, color='red')

# (b) 0.5-30 Hz linear-linear
m_lo = (f_psd >= 0.5) & (f_psd <= 30.0)
axes[1].plot(f_psd[m_lo], p_psd[m_lo], color='#2ca02c', linewidth=1.3)
axes[1].set_title('Common-mode PSD — 0.5-30 Hz zoom (linear)')
axes[1].set_xlabel('Frequency (Hz)')
axes[1].set_ylabel('PSD (uV^2/Hz)')
axes[1].grid(alpha=0.3)
axes[1].set_xlim(0.5, 30)

# annotate peaks within the zoom
for f, a in top_peaks:
    if 0.5 <= f <= 30.0:
        axes[1].axvline(f, color='red', alpha=0.3, linewidth=0.9)
        axes[1].annotate(f'{f:.2f} Hz', xy=(f, a),
                         xytext=(4, 6), textcoords='offset points',
                         fontsize=9, color='red')

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage1_common_mode_psd.png'), dpi=140)
plt.close()
print(f"[plot]   saved ({time.time()-tp:.1f}s)")

# ---- Plot 3: histogram with Gaussian overlay
print("[plot] stage1_common_mode_histogram.png ...")
tp = time.time()
fig, ax = plt.subplots(figsize=(11, 6))

# Clip for visualization (avoid a few extreme samples killing resolution)
clip = np.percentile(np.abs(cm), 99.5)
bins = np.linspace(-clip, clip, 201)
ax.hist(cm, bins=bins, density=True, color='#1f77b4', alpha=0.65,
        label=f'common-mode (n={len(cm):,})')

# Gaussian overlay with matched std
x_g = np.linspace(-clip, clip, 500)
g_pdf = (1.0 / (cm_std * np.sqrt(2 * np.pi))) * np.exp(-0.5 * (x_g / cm_std) ** 2)
ax.plot(x_g, g_pdf, color='red', linewidth=1.8,
        label=f'matched-std Gaussian (std={cm_std:.2f} uV)')

# Text box with moments
moment_text = (f'mean     = {cm_mean:+.3f}\n'
               f'std      = {cm_std:.3f}\n'
               f'kurtosis = {cm_kurt:.2f}  (Normal=3)\n'
               f'skewness = {cm_skew:+.3f}\n'
               f'|P95|    = {cm_p95:.2f}\n'
               f'|max|    = {max(abs(cm_min), abs(cm_max)):.1f}')
ax.text(0.02, 0.98, moment_text, transform=ax.transAxes,
        fontsize=10, va='top', ha='left', family='monospace',
        bbox=dict(facecolor='white', alpha=0.85, edgecolor='gray'))

heavy = "HEAVY-TAILED (spike-like: ECG/twitch)" if cm_kurt > 4.0 \
        else ("~Gaussian (drift/distributed)" if cm_kurt < 3.5 else "borderline")
ax.set_title(f'Common-mode amplitude histogram — kurtosis interpretation: {heavy}')
ax.set_xlabel('Amplitude (uV, clipped to |P99.5|)')
ax.set_ylabel('Probability density')
ax.legend(loc='upper right')
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage1_common_mode_histogram.png'), dpi=140)
plt.close()
print(f"[plot]   saved ({time.time()-tp:.1f}s)")

# ---- Plot 4: cm vs ch_std (2 panels)
print("[plot] stage1_cm_vs_ch_std.png ...")
tp = time.time()
fig, axes = plt.subplots(2, 1, figsize=(13, 10))

# (a) overlaid time series (first 60 s to keep it readable)
m60 = t_axis <= 60.0
axes[0].plot(t_axis[m60], abs_cm[m60], color='#1f77b4', linewidth=0.7,
             label='|cm|')
axes[0].plot(t_axis[m60], ch_std[m60], color='#e67e22', linewidth=0.7,
             alpha=0.85, label='ch_std (across good channels)')
axes[0].set_title(f'|cm| and per-sample ch_std — first 60 s '
                  f'(corr = {cm_chstd_r:+.3f}, '
                  f'frac |cm|>ch_std = {frac_cm_dominant*100:.1f}%)')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Amplitude (uV)')
axes[0].legend(loc='upper right')
axes[0].grid(alpha=0.3)

# (b) scatter of ch_std vs |cm| with linear fit (subsample for rendering)
n_plot = min(30000, len(cm))
idx_plot = np.random.default_rng(0).choice(len(cm), size=n_plot, replace=False)
axes[1].scatter(ch_std[idx_plot], abs_cm[idx_plot], s=2, alpha=0.25,
                color='#34495e', rasterized=True)
xs = np.linspace(ch_std.min(), ch_std.max(), 200)
axes[1].plot(xs, fit_slope * xs + fit_intercept, color='red', linewidth=1.5,
             label=f'fit: |cm| = {fit_slope:+.3f}*ch_std + {fit_intercept:+.2f}')
axes[1].plot(xs, xs, color='gray', linestyle='--', linewidth=1,
             label='|cm| = ch_std')
axes[1].set_xlabel('ch_std (uV, per-sample std across good channels)')
axes[1].set_ylabel('|cm| (uV)')
axes[1].set_title(f'ch_std vs |cm|  —  Pearson r = {cm_chstd_r:+.4f}')
axes[1].legend(loc='upper left')
axes[1].grid(alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage1_cm_vs_ch_std.png'), dpi=140)
plt.close()
print(f"[plot]   saved ({time.time()-tp:.1f}s)")

# ----------------------------------------------------------------------------
# 8. stage1_findings.txt
# ----------------------------------------------------------------------------
print("[out]  Writing stage1_findings.txt ...")

# Hypothesis selection logic
dominant_peak_hz = top_peaks[0][0] if top_peaks else float('nan')
delta_frac = band_frac['delta (0.5-4 Hz)']
theta_frac = band_frac['theta (4-8 Hz)']
alpha_frac = band_frac['alpha (8-13 Hz)']
beta_frac  = band_frac['beta  (13-30 Hz)']

if cm_kurt > 5 and 0.8 <= dominant_peak_hz <= 2.0:
    hypothesis = "CARDIAC (ECG)"
    rationale = (f"Kurtosis {cm_kurt:.2f} >> 3 (heavy-tailed), dominant peak at "
                 f"{dominant_peak_hz:.2f} Hz is in the heart-rate band, and the time "
                 f"domain should show QRS-like transients spaced ~1 s apart.")
elif 3.5 < cm_kurt <= 5 and 0.8 <= dominant_peak_hz <= 2.0:
    hypothesis = "CARDIAC (ECG) — moderate"
    rationale = (f"Moderately heavy tails (kurt={cm_kurt:.2f}) combined with a "
                 f"{dominant_peak_hz:.2f} Hz peak consistent with heart rate.")
elif cm_kurt < 4 and (theta_frac + alpha_frac + beta_frac) > 0.5 and delta_frac < 0.3:
    hypothesis = "SLOW POSTURAL MUSCLE / DISTRIBUTED EMG"
    rationale = (f"Near-Gaussian amplitude (kurt={cm_kurt:.2f}), broadband power "
                 f"concentrated above delta (theta+alpha+beta = "
                 f"{(theta_frac+alpha_frac+beta_frac)*100:.0f}%).")
elif delta_frac > 0.6 and cm_kurt < 4:
    hypothesis = "REFERENCE ELECTRODE DRIFT / 1/f"
    rationale = (f"Power dominated by delta band ({delta_frac*100:.0f}%), "
                 f"Gaussian-ish amplitude (kurt={cm_kurt:.2f}), no sharp peaks.")
elif len(top_peaks) > 0 and any(abs(f - 50.0) < 0.5 or abs(f - 60.0) < 0.5
                                 for f, _ in top_peaks):
    hypothesis = "LINE-RELATED EMI"
    rationale = "Spectral peak at line frequency (50 or 60 Hz) dominates."
elif cm_kurt > 10:
    hypothesis = "MOVEMENT / IMPEDANCE TRANSIENTS"
    rationale = (f"Extreme kurtosis ({cm_kurt:.2f}) suggests rare large bursts "
                 f"with quiet baseline — classic impedance-pop signature.")
else:
    hypothesis = "MIXED / UNCLEAR — manual review needed"
    rationale = (f"No single hypothesis fits cleanly. "
                 f"kurt={cm_kurt:.2f}, peak={dominant_peak_hz:.2f} Hz, "
                 f"delta frac={delta_frac*100:.0f}%.")

findings_path = os.path.join(OUT_DIR, 'stage1_findings.txt')
bar = "-" * 78
with open(findings_path, 'w', encoding='utf-8') as f:
    f.write("Stage 1: Common-Mode Source Diagnostic\n")
    f.write(f"Source CSV : {os.path.basename(CSV_PATH)}\n")
    f.write(f"Duration   : {duration_s:.1f} s ({n_samples:,} samples @ {FS:g} Hz)\n")
    f.write(f"Channels   : {len(ch_cols)} total  "
            f"| bad={n_bad}  good={n_good}\n")
    f.write("Filter chain: detrend -> 1 Hz HPF (Butter10) -> 50 Hz LPF (Butter10) "
            "-> 60 Hz notch (Q=30) -> 70 Hz notch (Q=30), all zero-phase\n")
    f.write("\n")

    f.write("1. Common-mode amplitude statistics\n")
    f.write(bar + "\n")
    f.write(f"   mean      : {cm_mean:+.4f} uV\n")
    f.write(f"   std       : {cm_std:.4f} uV\n")
    f.write(f"   min       : {cm_min:+.3f} uV\n")
    f.write(f"   max       : {cm_max:+.3f} uV\n")
    f.write(f"   |P95|     : {cm_p95:.3f} uV\n")
    f.write(f"   kurtosis  : {cm_kurt:.4f}   "
            f"(Normal=3; >>3 = heavy-tailed/spike-like, ~3 = Gaussian)\n")
    f.write(f"   skewness  : {cm_skew:+.4f}\n")
    f.write("\n")

    f.write("2. Top PSD peaks in 0.5 - 50 Hz (Welch, nperseg=4096)\n")
    f.write(bar + "\n")
    if len(top_peaks) == 0:
        f.write("   (no peaks detected above prominence threshold)\n")
    else:
        f.write(f"   {'rank':<5s}{'freq (Hz)':<12s}{'PSD (uV^2/Hz)':<18s}\n")
        for i, (fp, ap) in enumerate(top_peaks, 1):
            f.write(f"   {i:<5d}{fp:<12.3f}{ap:<18.4e}\n")
    f.write("\n")

    f.write("3. Fractional band power (0.5 - 50 Hz)\n")
    f.write(bar + "\n")
    for k in ['delta (0.5-4 Hz)', 'theta (4-8 Hz)', 'alpha (8-13 Hz)',
              'beta  (13-30 Hz)', 'gamma (30-50 Hz)']:
        f.write(f"   {k:22s} {bands[k]:12.4e}  ({band_frac[k]*100:5.2f}%)\n")
    f.write(f"   {'TOTAL (0.5-50 Hz)':22s} {total_power:12.4e}\n")
    f.write("\n")

    f.write("4. |cm| vs ch_std relationship\n")
    f.write(bar + "\n")
    f.write(f"   corr(|cm|, ch_std)           : {cm_chstd_r:+.4f}\n")
    f.write(f"   median(ch_std)               : {float(np.median(ch_std)):.3f} uV\n")
    f.write(f"   median(|cm|)                 : {float(np.median(abs_cm)):.3f} uV\n")
    f.write(f"   ratio median(ch_std)/|cm|    : {ratio_chstd_to_abscm:.3f}\n")
    f.write(f"   frac samples |cm| > ch_std   : {frac_cm_dominant*100:.2f}%\n")
    f.write(f"   linear fit |cm| vs ch_std    : slope={fit_slope:+.4f}  "
            f"intercept={fit_intercept:+.3f}\n")
    f.write("   Interpretation: if corr is positive and large, ch_std grows with\n")
    f.write("   |cm| -> shared external source; if ~0, CM is just noise averaging.\n")
    f.write("\n")

    f.write("5. Interpretation (most likely source)\n")
    f.write(bar + "\n")
    f.write(f"   HYPOTHESIS: {hypothesis}\n")
    f.write(f"   Rationale : {rationale}\n")
    f.write("\n")
    f.write("   Hypothesis reference card:\n")
    f.write("     Cardiac (ECG)         : ~1 Hz spikes, kurt>>3, peaks 1-1.8 Hz + harmonics, QRS transients\n")
    f.write("     Slow postural muscle  : broad 5-20 Hz bump, Gaussian amplitude, slow envelope\n")
    f.write("     Reference drift       : 1/f PSD, no clean peaks, Gaussian amplitude\n")
    f.write("     Line-related          : peaks at 50/60 Hz multiples\n")
    f.write("     Movement/impedance    : rare large bursts, extreme kurtosis, quiet between\n")

print(f"[out]  Wrote {findings_path}")

print("\n=============================================================")
print(f"STAGE 1 HYPOTHESIS: {hypothesis}")
print(f"  kurtosis={cm_kurt:.2f}  dominant peak={dominant_peak_hz:.2f} Hz  "
      f"std={cm_std:.2f} uV")
print("=============================================================")
print(f"[done] All artifacts written to: {OUT_DIR}")
