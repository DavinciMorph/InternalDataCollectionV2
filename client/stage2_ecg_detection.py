"""
Stage 2 — ECG contamination diagnostic for the 336-ch common-mode signal.

Hypothesis under test:
    The large non-neural common-mode signal (r ~ 0.53 cross-port, 1-50 Hz)
    in eeg_data_2026-04-10_200737.csv is cardiac (ECG) contamination
    propagating through body tissue and showing up on every electrode
    via the shared reference.

Method:
    1. Reproduce the reference filter chain from channel_similarity.py
       (detrend -> 1 Hz HPF -> 50 Hz LPF -> 60 Hz notch -> 70 Hz notch).
    2. Compute cm = mean(filtered[:, good_ch]).
    3. QRS-band isolation: 8-30 Hz bandpass on cm.
    4. Peak detection with physiological refractory.
    5. RR-interval analysis and BPM plausibility.
    6. QRS template matching (morphology consistency).
    7. Sanity null: random-phase shuffled cm_qrs.
    8. Variance share of cardiac trace in the common-mode.

Outputs: client/channel_similarity_analysis/stage2_*.png + stage2_ecg_findings.txt
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

# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
OUT_DIR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis"
BAD_PATH = os.path.join(OUT_DIR, 'bad_channels.txt')
FS       = 250.0

os.makedirs(OUT_DIR, exist_ok=True)
print(f"[init] Output dir: {OUT_DIR}")


# --------------------------------------------------------------------------
# 1. Parse bad-channel names from bad_channels.txt
#    Format: lines inside the "Flagged channels:" section start with spaces
#    then a name like "Port1_dev1_ch5".
# --------------------------------------------------------------------------
def parse_bad_channels(path):
    bad = set()
    in_flagged = False
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            s = line.rstrip('\n')
            if 'Flagged channels' in s:
                in_flagged = True
                continue
            if not in_flagged:
                continue
            tok = s.strip().split()
            if not tok:
                continue
            name = tok[0]
            if name.startswith('Port') and '_ch' in name:
                bad.add(name)
    return bad


print(f"[bad] Reading {BAD_PATH}")
bad_names = parse_bad_channels(BAD_PATH)
print(f"[bad] Parsed {len(bad_names)} bad channel names")


# --------------------------------------------------------------------------
# 2. Load CSV
# --------------------------------------------------------------------------
print(f"[load] Reading {CSV_PATH}")
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
      f"({time.time()-t0:.1f}s, duration={duration_s:.1f}s)")

data = df[ch_cols].to_numpy(dtype=np.float32, copy=True)
del df
print(f"[load] Raw matrix shape: {data.shape}")


# --------------------------------------------------------------------------
# 3. Filter chain (match channel_similarity.py exactly)
# --------------------------------------------------------------------------
print("[filt] detrend (linear, axis=0)")
t0 = time.time()
data = sig.detrend(data, axis=0, type='linear').astype(np.float32, copy=False)
print(f"[filt]   done ({time.time()-t0:.1f}s)")

print("[filt] 1 Hz HPF (Butter10 SOS, sosfiltfilt)")
t0 = time.time()
hpf_sos = sig.butter(10, 1.0, btype='high', fs=FS, output='sos')
data = sig.sosfiltfilt(hpf_sos, data, axis=0).astype(np.float32, copy=False)
print(f"[filt]   done ({time.time()-t0:.1f}s)")

print("[filt] 50 Hz LPF (Butter10 SOS, sosfiltfilt)")
t0 = time.time()
lpf_sos = sig.butter(10, 50.0, btype='low', fs=FS, output='sos')
data = sig.sosfiltfilt(lpf_sos, data, axis=0).astype(np.float32, copy=False)
print(f"[filt]   done ({time.time()-t0:.1f}s)")

print("[filt] 60 Hz iirnotch (filtfilt)")
t0 = time.time()
b60, a60 = sig.iirnotch(60.0, Q=30.0, fs=FS)
data = sig.filtfilt(b60, a60, data, axis=0).astype(np.float32, copy=False)
print(f"[filt]   done ({time.time()-t0:.1f}s)")

print("[filt] 70 Hz iirnotch (filtfilt)")
t0 = time.time()
b70, a70 = sig.iirnotch(70.0, Q=30.0, fs=FS)
data = sig.filtfilt(b70, a70, data, axis=0).astype(np.float32, copy=False)
print(f"[filt]   done ({time.time()-t0:.1f}s)")


# --------------------------------------------------------------------------
# 4. Build good-channel mask and common-mode
# --------------------------------------------------------------------------
good_mask = np.array([c not in bad_names for c in ch_cols], dtype=bool)
n_good = int(good_mask.sum())
print(f"[cm] good channels = {n_good} / {len(ch_cols)} "
      f"(bad = {len(ch_cols) - n_good})")

cm = data[:, good_mask].mean(axis=1).astype(np.float64)
del data
print(f"[cm] cm shape={cm.shape}  std={cm.std():.3f} uV  "
      f"min={cm.min():.1f}  max={cm.max():.1f}")


# --------------------------------------------------------------------------
# 5. QRS band isolation: 8-30 Hz
# --------------------------------------------------------------------------
print("[qrs] Bandpass 8-30 Hz (Butter4 SOS, sosfiltfilt)")
qrs_sos = sig.butter(4, [8.0, 30.0], btype='band', fs=FS, output='sos')
cm_qrs = sig.sosfiltfilt(qrs_sos, cm).astype(np.float64, copy=False)
rms_qrs = float(np.sqrt(np.mean(cm_qrs ** 2)))
print(f"[qrs] cm_qrs RMS = {rms_qrs:.3f} uV")


# --------------------------------------------------------------------------
# 6. Peak detection
# --------------------------------------------------------------------------
print("[peaks] find_peaks on |cm_qrs|")
abs_qrs = np.abs(cm_qrs)
peaks, props = sig.find_peaks(
    abs_qrs,
    height=2.0 * rms_qrs,
    distance=75,          # 300 ms refractory (min RR ~200 BPM)
    prominence=rms_qrs,
)
n_peaks = len(peaks)
print(f"[peaks] n_peaks = {n_peaks}")


# --------------------------------------------------------------------------
# 7. Inter-beat interval stats
# --------------------------------------------------------------------------
if n_peaks >= 2:
    rr_samples = np.diff(peaks)
    rr_ms = rr_samples * (1000.0 / FS)
    median_rr = float(np.median(rr_ms))
    mean_rr   = float(rr_ms.mean())
    std_rr    = float(rr_ms.std())
    min_rr    = float(rr_ms.min())
    max_rr    = float(rr_ms.max())
    bpm_median = 60000.0 / median_rr if median_rr > 0 else float('nan')
    cv_rr = std_rr / median_rr if median_rr > 0 else float('inf')
    print(f"[rr] n_rr={len(rr_ms)}  median={median_rr:.1f} ms  "
          f"mean={mean_rr:.1f}±{std_rr:.1f} ms  min={min_rr:.1f}  max={max_rr:.1f}")
    print(f"[rr] BPM (median) = {bpm_median:.1f}   CV(RR)={cv_rr:.3f}")

    physio_plausible = (40.0 <= bpm_median <= 180.0) and (cv_rr < 0.25)
    physio_verdict = "cardiac-plausible" if physio_plausible else "not cardiac"
    print(f"[rr] verdict: {physio_verdict}")
else:
    rr_ms = np.array([])
    median_rr = mean_rr = std_rr = min_rr = max_rr = bpm_median = cv_rr = float('nan')
    physio_plausible = False
    physio_verdict = "not cardiac (too few peaks)"


# --------------------------------------------------------------------------
# 8. QRS template matching
#    200 ms window = 50 samples -> 25 before + 25 after.
# --------------------------------------------------------------------------
WIN = 50
HALF = WIN // 2

def extract_windows(sig1d, idxs, half):
    valid = (idxs - half >= 0) & (idxs + half <= len(sig1d))
    keep_idxs = idxs[valid]
    if len(keep_idxs) == 0:
        return np.zeros((0, 2 * half)), keep_idxs
    out = np.empty((len(keep_idxs), 2 * half), dtype=np.float64)
    for i, k in enumerate(keep_idxs):
        out[i] = sig1d[k - half:k + half]
    return out, keep_idxs


windows, kept_peaks = extract_windows(cm_qrs, peaks, HALF)
print(f"[tmpl] extracted {len(windows)} valid windows "
      f"(width={2*HALF} samples = {2*HALF*1000/FS:.0f} ms)")

if len(windows) >= 2:
    template = windows.mean(axis=0)
    template_std = windows.std(axis=0)

    # Correlate each window with template (zero-lag Pearson)
    t0 = template - template.mean()
    t0_norm = np.linalg.norm(t0) + 1e-12
    corr_list = []
    for w in windows:
        w0 = w - w.mean()
        n = np.linalg.norm(w0) + 1e-12
        corr_list.append(float(np.dot(w0, t0) / (n * t0_norm)))
    corr_arr = np.asarray(corr_list)
    mean_r = float(corr_arr.mean())
    std_r  = float(corr_arr.std())
    frac_gt07 = float((corr_arr > 0.7).mean())
    print(f"[tmpl] mean r={mean_r:+.3f}  std={std_r:.3f}  "
          f"frac(r>0.7)={frac_gt07*100:.1f}%")
    template_consistent = frac_gt07 > 0.70
else:
    template = np.zeros(2 * HALF)
    template_std = np.zeros(2 * HALF)
    corr_arr = np.array([])
    mean_r = std_r = frac_gt07 = float('nan')
    template_consistent = False


# --------------------------------------------------------------------------
# 9. Null (random-phase surrogate of cm_qrs)
# --------------------------------------------------------------------------
print("[null] Building random-phase surrogate of cm_qrs")
rng = np.random.default_rng(42)
# FFT random-phase surrogate preserves PSD exactly.
F = np.fft.rfft(cm_qrs)
mags = np.abs(F)
phases = rng.uniform(0, 2 * np.pi, size=F.shape)
# keep DC (index 0) phase=0
phases[0] = 0.0
# For even length, Nyquist bin must be real
if len(cm_qrs) % 2 == 0:
    phases[-1] = 0.0
F_null = mags * np.exp(1j * phases)
cm_qrs_null = np.fft.irfft(F_null, n=len(cm_qrs)).astype(np.float64)

rms_null = float(np.sqrt(np.mean(cm_qrs_null ** 2)))
abs_null = np.abs(cm_qrs_null)
null_peaks, _ = sig.find_peaks(
    abs_null,
    height=2.0 * rms_null,
    distance=75,
    prominence=rms_null,
)
n_peaks_null = len(null_peaks)
print(f"[null] n_peaks_null = {n_peaks_null}  rms_null={rms_null:.3f}")

null_windows, _ = extract_windows(cm_qrs_null, null_peaks, HALF)
if len(null_windows) >= 2:
    null_template = null_windows.mean(axis=0)
    t0n = null_template - null_template.mean()
    t0n_norm = np.linalg.norm(t0n) + 1e-12
    null_corrs = []
    for w in null_windows:
        w0 = w - w.mean()
        nn = np.linalg.norm(w0) + 1e-12
        null_corrs.append(float(np.dot(w0, t0n) / (nn * t0n_norm)))
    null_corrs = np.asarray(null_corrs)
    null_mean_r = float(null_corrs.mean())
    null_frac07 = float((null_corrs > 0.7).mean())
else:
    null_mean_r = float('nan')
    null_frac07 = float('nan')
print(f"[null] template: mean r={null_mean_r:+.3f}  "
      f"frac(r>0.7)={null_frac07*100:.1f}%")


# --------------------------------------------------------------------------
# 10. Cardiac variance fraction
# --------------------------------------------------------------------------
cardiac_trace = np.zeros_like(cm)
if len(kept_peaks) > 0:
    for k in kept_peaks:
        lo = k - HALF
        hi = k + HALF
        a = max(lo, 0)
        b = min(hi, len(cardiac_trace))
        cardiac_trace[a:b] += template[(a - lo):(b - lo)]
var_cardiac = float(np.var(cardiac_trace))
var_cm = float(np.var(cm))
cardiac_var_frac = var_cardiac / var_cm if var_cm > 0 else float('nan')
print(f"[var] var(cardiac)={var_cardiac:.3f}  "
      f"var(cm)={var_cm:.3f}  frac={cardiac_var_frac:.4f}")


# --------------------------------------------------------------------------
# 11. Verdict
# --------------------------------------------------------------------------
# Present: physio-plausible RR AND template-consistent AND clearly beats null
null_beats = (not np.isnan(null_frac07)) and (frac_gt07 > null_frac07 + 0.15)
if physio_plausible and template_consistent and null_beats:
    verdict = "CARDIAC CONTAMINATION PRESENT"
elif physio_plausible and frac_gt07 > 0.5:
    verdict = "CARDIAC CONTAMINATION AMBIGUOUS"
else:
    verdict = "CARDIAC CONTAMINATION NOT PRESENT"
print(f"[verdict] {verdict}")


# --------------------------------------------------------------------------
# 12. Plots
# --------------------------------------------------------------------------
print("[plot] stage2_qrs_detection.png")
t_all = np.arange(len(cm_qrs)) / FS

def panel_cm(ax, tmax, title):
    mask = t_all <= tmax
    ax.plot(t_all[mask], cm_qrs[mask], color='#1f77b4', linewidth=0.8,
            label='cm_qrs (8-30 Hz)')
    pk_in = peaks[(peaks < mask.sum())]
    if len(pk_in) > 0:
        ax.plot(pk_in / FS, cm_qrs[pk_in], 'rx',
                markersize=6, label=f'peaks (n={len(pk_in)})')
    ax.axhline(2 * rms_qrs, color='gray', linestyle=':', linewidth=0.7)
    ax.axhline(-2 * rms_qrs, color='gray', linestyle=':', linewidth=0.7)
    ax.set_title(title)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('cm_qrs (uV)')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)


fig, axes = plt.subplots(3, 1, figsize=(13, 10))
panel_cm(axes[0], 30.0, f'Common-mode QRS band, first 30 s  '
                        f'(n_peaks total={n_peaks}, '
                        f'{"BPM=%.1f"%bpm_median if np.isfinite(bpm_median) else "BPM n/a"})')
panel_cm(axes[1], 10.0, 'Zoom: first 10 s')
panel_cm(axes[2], 3.0,  'Zoom: first 3 s (QRS shape check)')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage2_qrs_detection.png'), dpi=130)
plt.close()


print("[plot] stage2_rr_intervals.png")
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
if len(rr_ms) > 0:
    axes[0].hist(rr_ms, bins=40, color='#1f77b4', edgecolor='black', alpha=0.8)
    axes[0].axvline(median_rr, color='red', linestyle='--',
                    label=f'median={median_rr:.0f} ms  ({bpm_median:.1f} BPM)')
    axes[0].set_xlabel('RR interval (ms)')
    axes[0].set_ylabel('count')
    axes[0].set_title(f'RR interval histogram  (n={len(rr_ms)})\n'
                      f'CV(RR)={cv_rr:.3f}  verdict: {physio_verdict}')
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    axes[1].plot(np.arange(len(rr_ms)), rr_ms, 'o-',
                 color='#2ca02c', markersize=3, linewidth=0.6)
    axes[1].axhline(median_rr, color='red', linestyle='--',
                    label=f'median={median_rr:.0f} ms')
    axes[1].set_xlabel('beat #')
    axes[1].set_ylabel('RR (ms)')
    axes[1].set_title('RR interval time series')
    axes[1].legend()
    axes[1].grid(alpha=0.3)
else:
    for a in axes:
        a.text(0.5, 0.5, 'no peaks', ha='center', va='center',
               transform=a.transAxes)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage2_rr_intervals.png'), dpi=130)
plt.close()


print("[plot] stage2_qrs_template.png")
t_ms = (np.arange(2 * HALF) - HALF) * (1000.0 / FS)
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
if len(windows) >= 2:
    axes[0].plot(t_ms, template, color='#d62728', linewidth=2, label='mean template')
    axes[0].fill_between(t_ms, template - template_std, template + template_std,
                         color='#d62728', alpha=0.2, label='±1 std')
    axes[0].axvline(0, color='gray', linestyle=':', linewidth=0.7)
    axes[0].set_xlabel('time from peak (ms)')
    axes[0].set_ylabel('amplitude (uV)')
    axes[0].set_title(f'Averaged QRS template (n={len(windows)} windows)\n'
                      f'mean r vs template = {mean_r:+.3f}, '
                      f'frac(r>0.7)={frac_gt07*100:.1f}%')
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    overlay_n = min(50, len(windows))
    for w in windows[:overlay_n]:
        axes[1].plot(t_ms, w, color='gray', alpha=0.25, linewidth=0.5)
    axes[1].plot(t_ms, template, color='#d62728', linewidth=2, label='mean')
    axes[1].axvline(0, color='gray', linestyle=':', linewidth=0.7)
    axes[1].set_xlabel('time from peak (ms)')
    axes[1].set_ylabel('amplitude (uV)')
    axes[1].set_title(f'First {overlay_n} peak windows overlaid '
                      f'(morphology check)')
    axes[1].legend()
    axes[1].grid(alpha=0.3)
else:
    for a in axes:
        a.text(0.5, 0.5, 'insufficient peaks', ha='center', va='center',
               transform=a.transAxes)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage2_qrs_template.png'), dpi=130)
plt.close()


# --------------------------------------------------------------------------
# 13. Findings text
# --------------------------------------------------------------------------
out_txt = os.path.join(OUT_DIR, 'stage2_ecg_findings.txt')
bar = "-" * 78
with open(out_txt, 'w', encoding='utf-8') as f:
    f.write("Stage 2 -- ECG contamination diagnostic\n")
    f.write(f"CSV: {os.path.basename(CSV_PATH)}\n")
    f.write(f"Duration: {duration_s:.1f}s ({n_samples:,} samples @ {FS:g} Hz)\n")
    f.write(f"Channels: {len(ch_cols)}  good={n_good}  bad={len(ch_cols)-n_good}\n")
    f.write(f"Filter chain: detrend -> 1 Hz HPF (Butter10 SOS) -> 50 Hz LPF "
            f"(Butter10 SOS) -> 60 Hz notch (Q=30) -> 70 Hz notch (Q=30)\n")
    f.write("Common-mode: mean across good channels.\n")
    f.write(f"QRS band: 8-30 Hz Butter4 bandpass, sosfiltfilt.  "
            f"cm_qrs RMS = {rms_qrs:.3f} uV\n")
    f.write(f"find_peaks: height=2*RMS, distance=75 (300 ms), "
            f"prominence=RMS\n\n")

    f.write("Peak detection\n" + bar + "\n")
    f.write(f"  n_peaks                : {n_peaks}\n")
    f.write(f"  peaks per min          : "
            f"{n_peaks * 60.0 / duration_s:.2f}\n\n")

    f.write("Inter-beat intervals\n" + bar + "\n")
    f.write(f"  n_rr                   : {len(rr_ms)}\n")
    f.write(f"  median RR (ms)         : {median_rr:.2f}\n")
    f.write(f"  mean RR (ms)           : {mean_rr:.2f}\n")
    f.write(f"  std  RR (ms)           : {std_rr:.2f}\n")
    f.write(f"  min  RR (ms)           : {min_rr:.2f}\n")
    f.write(f"  max  RR (ms)           : {max_rr:.2f}\n")
    f.write(f"  CV(RR) (std/median)    : {cv_rr:.4f}\n")
    f.write(f"  BPM (from median RR)   : {bpm_median:.2f}\n")
    f.write(f"  physiological verdict  : {physio_verdict}\n")
    f.write(f"    rule                 : BPM in [40,180] AND CV(RR)<0.25\n\n")

    f.write("Template matching (200 ms windows centered on peaks)\n" + bar + "\n")
    f.write(f"  n_windows              : {len(windows)}\n")
    f.write(f"  mean r                 : {mean_r:+.4f}\n")
    f.write(f"  std  r                 : {std_r:.4f}\n")
    f.write(f"  frac(r > 0.7)          : {frac_gt07*100:.2f}%\n")
    f.write(f"  template consistency   : "
            f"{'YES (>70%)' if template_consistent else 'NO (<=70%)'}\n\n")

    f.write("Null (random-phase FFT surrogate of cm_qrs)\n" + bar + "\n")
    f.write(f"  null n_peaks           : {n_peaks_null}\n")
    f.write(f"  null mean r            : {null_mean_r:+.4f}\n")
    f.write(f"  null frac(r > 0.7)     : {null_frac07*100:.2f}%\n")
    f.write(f"  observed - null (r>0.7): "
            f"{(frac_gt07 - null_frac07)*100:+.2f}%\n\n")

    f.write("Cardiac variance contribution to common-mode\n" + bar + "\n")
    f.write(f"  var(cardiac trace)     : {var_cardiac:.3f} uV^2\n")
    f.write(f"  var(cm)                : {var_cm:.3f} uV^2\n")
    f.write(f"  cardiac fraction of CM : {cardiac_var_frac*100:.2f}%\n\n")

    f.write("VERDICT\n" + bar + "\n")
    f.write(f"  {verdict}\n")
    f.write("  Numerical justification:\n")
    f.write(f"    BPM={bpm_median:.1f} (plausible range 40-180)\n")
    f.write(f"    CV(RR)={cv_rr:.3f} (stability threshold 0.25)\n")
    f.write(f"    frac(r>0.7)={frac_gt07*100:.1f}%  "
            f"null={null_frac07*100:.1f}%  "
            f"delta={(frac_gt07-null_frac07)*100:+.1f}%\n")
    f.write(f"    cardiac var share   = {cardiac_var_frac*100:.2f}%\n")

print(f"[done] Wrote {out_txt}")
print(f"[done] Wrote stage2_qrs_detection.png, stage2_rr_intervals.png, "
      f"stage2_qrs_template.png")
print(f"[verdict] {verdict}")
