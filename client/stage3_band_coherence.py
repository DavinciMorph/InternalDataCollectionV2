"""
Stage 3: Band-localization of the common-mode source via cross-channel coherence.

Goal: find the frequency band where distant ports (P4 vs P5, etc) share signal.
The band tells us the source type:
    - 1-2 Hz + harmonics      -> cardiac / respiratory
    - broad 5-20 Hz plateau   -> muscle tone
    - flat 1-50 Hz            -> shared reference electrode (pure scaling)
    - discrete line multiples -> supply / ground
    - alpha (8-13 Hz) peak    -> real neural alpha

Filter chain matches client/channel_similarity.py exactly:
    detrend -> 1 Hz HPF (Butter10 SOS sosfiltfilt)
            -> 50 Hz LPF (Butter10 SOS sosfiltfilt)
            -> 60 Hz notch (iirnotch Q=30 filtfilt)
            -> 70 Hz notch (iirnotch Q=30 filtfilt)

Pair categories (30 pairs each, seed=42):
    within-device           (baseline — neighbors ~mm apart)
    within-port cross-dev   (same SPI chain, different ADS1299)
    P4 <-> P5               (opposite-ends temporal pair — key test)
    P1 <-> P5               (centerline to rightmost)
    P4 <-> P1               (leftmost to centerline)
    P6 <-> anything         (control — broken port, low coherence expected)

For each pair, compute magnitude-squared coherence via scipy.signal.coherence
with nperseg=1024 (~4.1 s), noverlap=512. Average the 30 coherence spectra per
category. At each category's peak-coherence frequency, compute cross-spectral
phase via scipy.signal.csd and report mean phase across the 30 pairs.

Outputs in client/channel_similarity_analysis/:
    stage3_coherence_spectra.png
    stage3_coherence_by_band.png
    stage3_phase_vs_category.png
    stage3_coherence_findings.txt
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
CSV_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\eeg_data_2026-04-10_200737.csv"
BAD_PATH = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis\bad_channels.txt"
OUT_DIR  = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting\client\channel_similarity_analysis"
FS = 250.0
NPERSEG = 1024
NOVERLAP = 512
N_PAIRS_PER_CAT = 30
SEED = 42

BANDS = [
    ('delta',     0.5,  4.0),
    ('theta',     4.0,  8.0),
    ('alpha',     8.0, 13.0),
    ('beta',     13.0, 30.0),
    ('low-gamma',30.0, 50.0),
]

os.makedirs(OUT_DIR, exist_ok=True)


# -----------------------------------------------------------------------------
# 1. Load bad channel list
# -----------------------------------------------------------------------------
print(f"[init] Loading bad channel list from {BAD_PATH}")
bad_set = set()
ch_re = re.compile(r'^(Port\d+_dev\d+_ch\d+)')
with open(BAD_PATH, 'r', encoding='utf-8') as f:
    for line in f:
        ls = line.strip()
        m = ch_re.match(ls)
        if m:
            bad_set.add(m.group(1))
print(f"[init] {len(bad_set)} bad channels parsed from report")


# -----------------------------------------------------------------------------
# 2. Load CSV
# -----------------------------------------------------------------------------
print(f"[load] Reading {CSV_PATH} ...")
t0 = time.time()
header = pd.read_csv(CSV_PATH, nrows=0).columns.tolist()
ch_cols = [c for c in header if c.startswith('Port')]
dtypes = {c: np.float32 for c in ch_cols}
if 'sample_number' in header:
    dtypes['sample_number'] = np.int64
df = pd.read_csv(CSV_PATH, dtype=dtypes)
n_samples = len(df)
print(f"[load] {n_samples:,} rows, {len(ch_cols)} channels "
      f"({time.time()-t0:.1f}s, {n_samples/FS:.1f}s recording)")


# -----------------------------------------------------------------------------
# 3. Parse channel metadata
# -----------------------------------------------------------------------------
col_re = re.compile(r'^Port(\d+)_dev(\d+)_ch(\d+)$')
ports = np.zeros(len(ch_cols), dtype=np.int16)
devs  = np.zeros(len(ch_cols), dtype=np.int16)
chs   = np.zeros(len(ch_cols), dtype=np.int16)
for i, c in enumerate(ch_cols):
    m = col_re.match(c)
    ports[i] = int(m.group(1))
    devs[i]  = int(m.group(2))
    chs[i]   = int(m.group(3))

good_mask = np.array([c not in bad_set for c in ch_cols], dtype=bool)
print(f"[init] Good channels: {good_mask.sum()}/{len(ch_cols)}")
for p in range(1, 8):
    total = int((ports == p).sum())
    good  = int(((ports == p) & good_mask).sum())
    print(f"[init]   Port{p}: {good}/{total} good")


# -----------------------------------------------------------------------------
# 4. Extract + filter (matches channel_similarity.py exactly)
# -----------------------------------------------------------------------------
print("[load] Extracting raw matrix ...")
t0 = time.time()
data = df[ch_cols].to_numpy(dtype=np.float32, copy=True)
print(f"[load] Raw matrix {data.shape} ({data.nbytes/1e6:.1f} MB, {time.time()-t0:.1f}s)")
del df

print("[filt] Linear detrend ...")
t0 = time.time()
data_filt = sig.detrend(data, axis=0, type='linear').astype(np.float32, copy=False)
print(f"[filt] Detrend ({time.time()-t0:.1f}s)")
del data

print("[filt] 1 Hz Butterworth HPF (10th order SOS, sosfiltfilt) ...")
t0 = time.time()
hpf_sos = sig.butter(10, 1.0, btype='high', fs=FS, output='sos')
data_filt = sig.sosfiltfilt(hpf_sos, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] HPF ({time.time()-t0:.1f}s)")

print("[filt] 50 Hz Butterworth LPF (10th order SOS, sosfiltfilt) ...")
t0 = time.time()
lpf_sos = sig.butter(10, 50.0, btype='low', fs=FS, output='sos')
data_filt = sig.sosfiltfilt(lpf_sos, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] LPF ({time.time()-t0:.1f}s)")

print("[filt] 60 Hz notch (iirnotch Q=30, filtfilt) ...")
t0 = time.time()
b60, a60 = sig.iirnotch(60.0, Q=30.0, fs=FS)
data_filt = sig.filtfilt(b60, a60, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] 60 Hz ({time.time()-t0:.1f}s)")

print("[filt] 70 Hz notch (iirnotch Q=30, filtfilt) ...")
t0 = time.time()
b70, a70 = sig.iirnotch(70.0, Q=30.0, fs=FS)
data_filt = sig.filtfilt(b70, a70, data_filt, axis=0).astype(np.float32, copy=False)
print(f"[filt] 70 Hz ({time.time()-t0:.1f}s)")


# -----------------------------------------------------------------------------
# 5. Build pair categories
# -----------------------------------------------------------------------------
rng = np.random.default_rng(SEED)

good_idx_all = np.where(good_mask)[0]

def idx_port(p):
    return np.where((ports == p) & good_mask)[0]

good_by_port = {p: idx_port(p) for p in range(1, 8)}

# device key: (port, dev)
dev_keys = np.array([(ports[i], devs[i]) for i in range(len(ch_cols))],
                    dtype=[('p', 'i2'), ('d', 'i2')])

def sample_within_device_pairs(n):
    """Pairs of good channels inside the same device."""
    # group good channels by (port, dev)
    groups = {}
    for gi in good_idx_all:
        key = (int(ports[gi]), int(devs[gi]))
        groups.setdefault(key, []).append(gi)
    # only groups with >=2 good channels qualify
    eligible_groups = [v for v in groups.values() if len(v) >= 2]
    if not eligible_groups:
        return []
    pairs = []
    attempts = 0
    max_attempts = n * 50
    seen = set()
    while len(pairs) < n and attempts < max_attempts:
        attempts += 1
        grp = eligible_groups[rng.integers(0, len(eligible_groups))]
        a, b = rng.choice(len(grp), size=2, replace=False)
        i, j = grp[a], grp[b]
        key = (min(i, j), max(i, j))
        if key in seen:
            continue
        seen.add(key)
        pairs.append((i, j))
    return pairs

def sample_within_port_cross_dev_pairs(n):
    """Pairs of good channels in the same port but different devices."""
    pairs = []
    seen = set()
    attempts = 0
    max_attempts = n * 200
    eligible_ports = [p for p in range(1, 8)
                      if len(set((int(devs[gi])) for gi in good_by_port[p])) >= 2]
    if not eligible_ports:
        return []
    while len(pairs) < n and attempts < max_attempts:
        attempts += 1
        p = eligible_ports[rng.integers(0, len(eligible_ports))]
        cand = good_by_port[p]
        if len(cand) < 2:
            continue
        a, b = rng.choice(len(cand), size=2, replace=False)
        i, j = cand[a], cand[b]
        if devs[i] == devs[j]:
            continue
        key = (min(i, j), max(i, j))
        if key in seen:
            continue
        seen.add(key)
        pairs.append((i, j))
    return pairs

def sample_cross_port_pairs(pa, pb, n):
    """Pairs of good channels, one from port pa, one from port pb."""
    ca = good_by_port[pa]
    cb = good_by_port[pb]
    if len(ca) == 0 or len(cb) == 0:
        return []
    pairs = []
    seen = set()
    attempts = 0
    max_attempts = n * 200
    while len(pairs) < n and attempts < max_attempts:
        attempts += 1
        i = ca[rng.integers(0, len(ca))]
        j = cb[rng.integers(0, len(cb))]
        if i == j:
            continue
        key = (min(i, j), max(i, j))
        if key in seen:
            continue
        seen.add(key)
        pairs.append((i, j))
    return pairs

def sample_p6_vs_anything_pairs(n):
    """P6 channel paired with any non-P6 good channel."""
    c6 = good_by_port[6]
    c_other = np.concatenate([good_by_port[p] for p in range(1, 8) if p != 6])
    if len(c6) == 0 or len(c_other) == 0:
        return []
    pairs = []
    seen = set()
    attempts = 0
    max_attempts = n * 200
    while len(pairs) < n and attempts < max_attempts:
        attempts += 1
        i = c6[rng.integers(0, len(c6))]
        j = c_other[rng.integers(0, len(c_other))]
        key = (min(i, j), max(i, j))
        if key in seen:
            continue
        seen.add(key)
        pairs.append((i, j))
    return pairs

print("[pairs] Sampling pair categories (seed=42, 30 pairs each) ...")
categories = {}
categories['within-device']           = sample_within_device_pairs(N_PAIRS_PER_CAT)
categories['within-port cross-dev']   = sample_within_port_cross_dev_pairs(N_PAIRS_PER_CAT)
categories['P4 <-> P5']               = sample_cross_port_pairs(4, 5, N_PAIRS_PER_CAT)
categories['P1 <-> P5']               = sample_cross_port_pairs(1, 5, N_PAIRS_PER_CAT)
categories['P4 <-> P1']               = sample_cross_port_pairs(4, 1, N_PAIRS_PER_CAT)
categories['P6 <-> anything']         = sample_p6_vs_anything_pairs(N_PAIRS_PER_CAT)

for name, pairs in categories.items():
    print(f"[pairs]   {name:30s} n={len(pairs)}")


# -----------------------------------------------------------------------------
# 6. Compute coherence + phase per category
# -----------------------------------------------------------------------------
print("[coh] Computing coherence + cross-spectra per category ...")
t0 = time.time()

results = {}  # name -> dict

# Use the first pair to get f axis
sample_i, sample_j = categories['within-device'][0]
f_axis, _ = sig.coherence(
    data_filt[:, sample_i].astype(np.float64),
    data_filt[:, sample_j].astype(np.float64),
    fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP,
)
print(f"[coh]   Frequency bins: {len(f_axis)} (df={f_axis[1]-f_axis[0]:.3f} Hz)")

for name, pairs in categories.items():
    if len(pairs) == 0:
        results[name] = {
            'f': f_axis, 'coh_mean': np.full_like(f_axis, np.nan),
            'coh_stack': np.zeros((0, len(f_axis))),
            'peak_f': np.nan, 'peak_c': np.nan,
            'phases_deg': np.array([]),
            'mean_phase_deg': np.nan,
            'std_phase_deg': np.nan,
            'n': 0,
        }
        continue
    stack = np.zeros((len(pairs), len(f_axis)), dtype=np.float64)
    for k, (i, j) in enumerate(pairs):
        x = data_filt[:, i].astype(np.float64)
        y = data_filt[:, j].astype(np.float64)
        _, cxy = sig.coherence(x, y, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP)
        stack[k] = cxy
    coh_mean = stack.mean(axis=0)

    # Peak coherence location (restricted to 0.5-50 Hz for fairness)
    band_mask = (f_axis >= 0.5) & (f_axis <= 50.0)
    local = np.where(band_mask, coh_mean, -np.inf)
    peak_bin = int(np.argmax(local))
    peak_f = float(f_axis[peak_bin])
    peak_c = float(coh_mean[peak_bin])

    # Phase at the peak bin for each pair
    phases = np.zeros(len(pairs), dtype=np.float64)
    for k, (i, j) in enumerate(pairs):
        x = data_filt[:, i].astype(np.float64)
        y = data_filt[:, j].astype(np.float64)
        f_csd, Pxy = sig.csd(x, y, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP)
        # Same f grid as coherence; use peak_bin directly
        phases[k] = float(np.angle(Pxy[peak_bin], deg=True))

    # Circular-ish mean for phase: use atan2 of mean complex unit vectors
    unit = np.exp(1j * np.deg2rad(phases))
    mean_phase_deg = float(np.rad2deg(np.angle(unit.mean())))
    # Circular std
    R = np.abs(unit.mean())
    if R > 1e-12 and R < 1.0:
        std_phase_deg = float(np.rad2deg(np.sqrt(-2.0 * np.log(R))))
    else:
        std_phase_deg = 0.0

    results[name] = {
        'f': f_axis,
        'coh_mean': coh_mean,
        'coh_stack': stack,
        'peak_f': peak_f,
        'peak_c': peak_c,
        'phases_deg': phases,
        'mean_phase_deg': mean_phase_deg,
        'std_phase_deg': std_phase_deg,
        'n': len(pairs),
    }
    print(f"[coh]   {name:30s} peak={peak_f:6.2f} Hz  C={peak_c:.3f}  "
          f"phase={mean_phase_deg:+7.1f} deg (sd {std_phase_deg:5.1f})")

print(f"[coh] Done ({time.time()-t0:.1f}s)")


# -----------------------------------------------------------------------------
# 7. Per-band mean coherence + 1-50 Hz integral
# -----------------------------------------------------------------------------
def band_means(f, coh):
    out = {}
    for name, lo, hi in BANDS:
        m = (f >= lo) & (f < hi)
        if m.any():
            out[name] = float(coh[m].mean())
        else:
            out[name] = np.nan
    return out

def band_integral(f, coh, lo=1.0, hi=50.0):
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return np.nan
    return float(np.trapz(coh[m], f[m]))

for name, r in results.items():
    r['band_means'] = band_means(r['f'], r['coh_mean'])
    r['integral_1_50'] = band_integral(r['f'], r['coh_mean'])


# -----------------------------------------------------------------------------
# 8. Plots
# -----------------------------------------------------------------------------
print("[plot] Building plots ...")

CAT_ORDER = [
    'within-device',
    'within-port cross-dev',
    'P4 <-> P5',
    'P1 <-> P5',
    'P4 <-> P1',
    'P6 <-> anything',
]
CAT_COLORS = {
    'within-device':         '#1f77b4',
    'within-port cross-dev': '#ff7f0e',
    'P4 <-> P5':             '#d62728',
    'P1 <-> P5':             '#9467bd',
    'P4 <-> P1':             '#2ca02c',
    'P6 <-> anything':       '#7f7f7f',
}

# ---- Plot 1: coherence spectra overlay ----
t0 = time.time()
fig, ax = plt.subplots(figsize=(11, 6.5))
for name in CAT_ORDER:
    r = results[name]
    if r['n'] == 0:
        continue
    m = (r['f'] >= 0.5) & (r['f'] <= 50.0)
    ax.plot(r['f'][m], r['coh_mean'][m],
            color=CAT_COLORS[name], lw=1.6,
            label=f"{name}  (n={r['n']}, peak {r['peak_f']:.1f} Hz C={r['peak_c']:.2f})")
# Band shading
for i, (bname, lo, hi) in enumerate(BANDS):
    ax.axvspan(lo, hi, color='lightgray',
               alpha=0.12 if i % 2 == 0 else 0.06, zorder=0)
    ax.text((lo + hi) / 2, 1.02, bname, ha='center', va='bottom',
            fontsize=8, color='gray', transform=ax.get_xaxis_transform())
ax.set_xlim(0.5, 50)
ax.set_ylim(0, 1.0)
ax.set_xlabel('Frequency (Hz)')
ax.set_ylabel('Magnitude-squared coherence')
ax.set_title('Stage 3: cross-channel coherence spectra by pair category\n'
             f'30 pairs/category, seed={SEED}, nperseg={NPERSEG}, noverlap={NOVERLAP}')
ax.grid(alpha=0.3)
ax.legend(loc='upper right', fontsize=8, framealpha=0.92)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage3_coherence_spectra.png'), dpi=140)
plt.close()
print(f"[plot]   stage3_coherence_spectra.png ({time.time()-t0:.1f}s)")

# ---- Plot 2: grouped bars by band ----
t0 = time.time()
band_names = [b[0] for b in BANDS]
n_cat = len(CAT_ORDER)
n_band = len(band_names)
bar_w = 0.8 / n_cat
x = np.arange(n_band)
fig, ax = plt.subplots(figsize=(11, 6))
for i, name in enumerate(CAT_ORDER):
    r = results[name]
    if r['n'] == 0:
        continue
    vals = [r['band_means'][b] for b in band_names]
    ax.bar(x + (i - (n_cat - 1) / 2) * bar_w, vals, bar_w,
           color=CAT_COLORS[name], edgecolor='black', linewidth=0.3,
           label=name)
ax.set_xticks(x)
ax.set_xticklabels(band_names)
ax.set_ylabel('Mean coherence in band')
ax.set_ylim(0, 1.0)
ax.set_title('Stage 3: mean coherence per band, by pair category')
ax.grid(alpha=0.3, axis='y')
ax.legend(loc='upper right', fontsize=8, framealpha=0.92)
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage3_coherence_by_band.png'), dpi=140)
plt.close()
print(f"[plot]   stage3_coherence_by_band.png ({time.time()-t0:.1f}s)")

# ---- Plot 3: mean phase at peak, with error bars ----
t0 = time.time()
fig, ax = plt.subplots(figsize=(10, 6))
means = [results[name]['mean_phase_deg'] for name in CAT_ORDER]
stds  = [results[name]['std_phase_deg']  for name in CAT_ORDER]
ns    = [results[name]['n']              for name in CAT_ORDER]
peak_fs = [results[name]['peak_f']       for name in CAT_ORDER]
xpos = np.arange(len(CAT_ORDER))
colors = [CAT_COLORS[n] for n in CAT_ORDER]
bars = ax.bar(xpos, means, yerr=stds, color=colors, edgecolor='black',
              linewidth=0.5, capsize=4)
ax.axhline(0, color='black', linewidth=0.8)
ax.set_xticks(xpos)
ax.set_xticklabels([f"{name}\n@ {pf:.1f} Hz\n(n={n})"
                    for name, pf, n in zip(CAT_ORDER, peak_fs, ns)],
                   fontsize=8)
ax.set_ylabel('Mean cross-spectral phase at peak coherence (deg)')
ax.set_title('Stage 3: phase at peak coherence frequency\n'
             '0 deg = in-phase (shared reference/supply), != 0 = propagation / mixing')
ax.grid(alpha=0.3, axis='y')
# Annotate phase values on each bar
for b, m in zip(bars, means):
    y = b.get_height()
    ax.text(b.get_x() + b.get_width() / 2, y + (5 if y >= 0 else -15),
            f'{m:+.1f}', ha='center', va='bottom' if y >= 0 else 'top',
            fontsize=9, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'stage3_phase_vs_category.png'), dpi=140)
plt.close()
print(f"[plot]   stage3_phase_vs_category.png ({time.time()-t0:.1f}s)")


# -----------------------------------------------------------------------------
# 9. Findings text
# -----------------------------------------------------------------------------
print("[report] Writing stage3_coherence_findings.txt ...")
out_txt = os.path.join(OUT_DIR, 'stage3_coherence_findings.txt')

def which_band(freq):
    for name, lo, hi in BANDS:
        if lo <= freq < hi:
            return name
    return 'out-of-band'

with open(out_txt, 'w', encoding='utf-8') as f:
    f.write("Stage 3: cross-channel coherence band-localization\n")
    f.write("=" * 78 + "\n")
    f.write(f"Input:  {os.path.basename(CSV_PATH)}\n")
    f.write(f"Samples: {n_samples:,}  Duration: {n_samples/FS:.1f}s  Fs: {FS:g} Hz\n")
    f.write(f"Good channels: {int(good_mask.sum())}/{len(ch_cols)} "
            f"({len(bad_set)} bad from stage-0 report)\n")
    f.write(f"Method: scipy.signal.coherence(fs=250, nperseg={NPERSEG}, noverlap={NOVERLAP})\n")
    f.write(f"Pairs per category: {N_PAIRS_PER_CAT} (seed={SEED})\n")
    f.write(f"Filters: detrend + 1 Hz HPF (Butter10 SOS sosfiltfilt) + "
            f"50 Hz LPF (Butter10 SOS sosfiltfilt) + 60 Hz notch (iirnotch Q=30 filtfilt) + "
            f"70 Hz notch (iirnotch Q=30 filtfilt)\n")
    f.write("\n")

    for name in CAT_ORDER:
        r = results[name]
        f.write("-" * 78 + "\n")
        f.write(f"Category: {name}\n")
        f.write(f"  n_pairs               : {r['n']}\n")
        if r['n'] == 0:
            f.write("  (no eligible pairs)\n\n")
            continue
        f.write(f"  Peak coherence freq   : {r['peak_f']:.2f} Hz  "
                f"(band: {which_band(r['peak_f'])})\n")
        f.write(f"  Peak coherence value  : {r['peak_c']:.4f}\n")
        f.write("  Per-band mean coherence:\n")
        for bname, _lo, _hi in BANDS:
            f.write(f"    {bname:10s}: {r['band_means'][bname]:.4f}\n")
        f.write(f"  Integral 1-50 Hz      : {r['integral_1_50']:.3f}\n")
        f.write(f"  Mean phase at peak    : {r['mean_phase_deg']:+7.2f} deg  "
                f"(circular sd {r['std_phase_deg']:.2f} deg)\n")
        f.write("\n")

    # ---- Summary ----
    f.write("=" * 78 + "\n")
    f.write("Summary\n")
    f.write("=" * 78 + "\n\n")

    p4p5 = results['P4 <-> P5']
    p6c  = results['P6 <-> anything']
    wp   = results['within-port cross-dev']
    wd   = results['within-device']
    p1p5 = results['P1 <-> P5']
    p4p1 = results['P4 <-> P1']

    if p4p5['n'] > 0:
        # Find the band with highest mean coherence for P4 <-> P5
        best_band = max(p4p5['band_means'].items(), key=lambda kv: kv[1])
        f.write(f"Q1. Which band has the highest cross-port coherence for P4<->P5?\n")
        f.write(f"    Answer: {best_band[0]} "
                f"(mean C = {best_band[1]:.3f}).\n")
        f.write(f"    Peak coherence frequency = {p4p5['peak_f']:.2f} Hz "
                f"(band: {which_band(p4p5['peak_f'])}), "
                f"peak C = {p4p5['peak_c']:.3f}\n\n")

        f.write(f"Q2. Is the P4<->P5 phase near 0 deg?\n")
        phi = p4p5['mean_phase_deg']
        near_zero = abs(phi) < 15.0
        f.write(f"    Mean phase = {phi:+.2f} deg (sd {p4p5['std_phase_deg']:.1f} deg).\n")
        f.write(f"    Near-zero (|phi|<15 deg)? {'YES' if near_zero else 'NO'}\n")
        if near_zero:
            f.write("    -> Consistent with shared reference / supply / volume-conducted source\n")
            f.write("       (phase ~= 0 means both channels see the same waveform instantaneously).\n\n")
        else:
            f.write("    -> Nonzero phase suggests propagation delay, multi-source mixing,\n"
                    "       or phase-shifted filter artifact rather than a pure reference tie.\n\n")

        # Spectral shape analysis
        vals = np.array(list(p4p5['band_means'].values()))
        dyn_range = float(vals.max() - vals.min())
        cv = float(vals.std() / (vals.mean() + 1e-12))
        # Fraction of integral 1-50 Hz coming from within 2 Hz of the peak
        f_arr = p4p5['f']
        c_arr = p4p5['coh_mean']
        bm = (f_arr >= 1.0) & (f_arr <= 50.0)
        total = float(np.trapz(c_arr[bm], f_arr[bm]))
        near = (f_arr >= max(1.0, p4p5['peak_f'] - 2.0)) & \
               (f_arr <= min(50.0, p4p5['peak_f'] + 2.0))
        near_int = float(np.trapz(c_arr[near], f_arr[near]))
        peak_frac = near_int / total if total > 0 else 0.0

        if dyn_range < 0.08 and cv < 0.15:
            shape_desc = ("FLAT across bands — consistent with broadband scaling "
                          "from a shared reference electrode / common amplifier node.")
        elif peak_frac > 0.30:
            shape_desc = (f"NARROW — {peak_frac*100:.0f}% of 1-50 Hz coherence mass is within "
                          f"+/-2 Hz of {p4p5['peak_f']:.1f} Hz. Consistent with a discrete "
                          f"contaminant at that frequency.")
        else:
            shape_desc = (f"BROAD but non-uniform — dynamic range {dyn_range:.3f}, "
                          f"peak-band fraction {peak_frac*100:.0f}%. Some structure but not "
                          f"a single tone.")
        f.write(f"Q3. Coherence shape for P4<->P5: {shape_desc}\n\n")

    if p6c['n'] > 0 and p4p5['n'] > 0:
        f.write(f"Q4. P6<->anything vs P4<->P5 control comparison:\n")
        f.write(f"    P6<->anything mean coherence (1-50 Hz): "
                f"{p6c['coh_mean'][(p6c['f']>=1)&(p6c['f']<=50)].mean():.4f}\n")
        f.write(f"    P4<->P5      mean coherence (1-50 Hz): "
                f"{p4p5['coh_mean'][(p4p5['f']>=1)&(p4p5['f']<=50)].mean():.4f}\n")
        ratio = (p4p5['coh_mean'][(p4p5['f']>=1)&(p4p5['f']<=50)].mean() /
                 max(1e-9, p6c['coh_mean'][(p6c['f']>=1)&(p6c['f']<=50)].mean()))
        f.write(f"    Ratio (P4<->P5) / (P6<->any) = {ratio:.1f}x\n")
        if ratio > 2.0:
            f.write("    -> P4<->P5 is clearly elevated above the broken-port control,\n"
                    "       so the cross-port coupling is a REAL shared signal, not amp noise.\n\n")
        else:
            f.write("    -> P4<->P5 is only marginally above P6 control; cross-port coupling\n"
                    "       is weak / within the noise floor of this test.\n\n")

    # Hypothesis verdict
    f.write("Hypothesis verdict\n")
    f.write("-" * 78 + "\n")
    if p4p5['n'] > 0:
        pf = p4p5['peak_f']
        phi = abs(p4p5['mean_phase_deg'])
        bm = p4p5['band_means']
        vals = np.array(list(bm.values()))
        dyn_range = float(vals.max() - vals.min())
        # Decide
        verdict = []
        if dyn_range < 0.08:
            verdict.append("REFERENCE: flat coherence across all bands is the signature "
                           "of a shared reference electrode (broadband scaling).")
        if 0.5 <= pf <= 3.0 and phi < 20:
            verdict.append("CARDIAC / RESPIRATORY: peak in delta with ~0 deg phase "
                           "is consistent with ECG/pulse contamination.")
        if 8.0 <= pf <= 13.0:
            verdict.append("NEURAL ALPHA: peak in alpha band — partially neural coupling "
                           "would be a positive result.")
        if 5.0 <= pf <= 20.0 and dyn_range >= 0.08 and bm.get('beta', 0) > 0.2:
            verdict.append("MUSCLE: broad 5-20 Hz plateau with elevated beta is "
                           "consistent with EMG/muscle tone.")
        if not verdict:
            verdict.append("Inconclusive from this view alone — see spectra plot.")
        for v in verdict:
            f.write(f"  * {v}\n")
    f.write("\n")
    f.write("Plots:\n")
    f.write("  stage3_coherence_spectra.png   — all 6 categories overlaid (the key plot)\n")
    f.write("  stage3_coherence_by_band.png   — grouped bars per band, per category\n")
    f.write("  stage3_phase_vs_category.png   — mean phase at peak coherence per category\n")

print(f"[report] Wrote {out_txt}")
print("[done] Stage 3 complete.")
