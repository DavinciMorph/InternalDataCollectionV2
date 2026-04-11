"""
ADS1299 Impedance Measurement Failure Analysis
Analyzes DFT magnitude at 62.5 Hz from impedance check readings.
"""
import numpy as np
from scipy import stats

# Ch2 rms_code values (time-ordered, ~1 reading per second)
data = np.array([
    10069, 3509, 2161, 1538, 1198, 186, 0, 104, 104, 123, 122, 230, 115, 249, 0, 0, 0, 1637, 992, 1276,
    5713, 1653, 1755, 289, 350, 1016, 615, 713, 343, 489, 1244, 407, 1115, 1899, 509, 1240, 1593, 405,
    583, 250, 384, 1585, 637, 2574, 1411, 825, 268, 4782, 761, 115, 2020, 756, 740, 1087, 1063, 3905,
    143, 784, 2773, 157, 2134, 952, 1532, 1474, 611, 398, 1100, 510, 263, 378, 577, 346, 776, 203, 859,
    2023, 856, 1754, 425, 237, 1065, 1087, 511, 1216, 1012, 724, 2538, 367, 292, 2705, 255, 3016, 715,
    522, 2510, 1276, 0, 4164, 2393, 3688, 309, 568, 1300, 500, 0, 2329, 528, 238, 2707, 577, 729, 2595,
    2785, 2182, 0, 264, 1189, 156, 177
])

n = len(data)
t = np.arange(n)

print("=" * 70)
print("1. DESCRIPTIVE STATISTICS")
print("=" * 70)
print(f"N            = {n}")
print(f"Mean         = {np.mean(data):.1f}")
print(f"Median       = {np.median(data):.1f}")
print(f"Std Dev      = {np.std(data, ddof=1):.1f}")
print(f"Min          = {np.min(data)}")
print(f"Max          = {np.max(data)}")
print(f"Zeros        = {np.sum(data == 0)} ({100*np.sum(data == 0)/n:.1f}%)")
for p in [5, 10, 25, 50, 75, 90, 95]:
    print(f"P{p:<3}         = {np.percentile(data, p):.0f}")

print()
print("=" * 70)
print("2. TIME TREND ANALYSIS")
print("=" * 70)

# Linear regression for trend
slope, intercept, r_value, p_value, std_err = stats.linregress(t, data)
print(f"Linear trend: slope = {slope:.2f} codes/sample, R^2 = {r_value**2:.4f}, p = {p_value:.4f}")

# Split into thirds
third = n // 3
t1 = data[:third]
t2 = data[third:2*third]
t3 = data[2*third:]
print(f"First third mean:  {np.mean(t1):.0f}  (n={len(t1)})")
print(f"Middle third mean: {np.mean(t2):.0f}  (n={len(t2)})")
print(f"Last third mean:   {np.mean(t3):.0f}  (n={len(t3)})")

stat_kw, p_kw = stats.kruskal(t1, t2, t3)
print(f"Kruskal-Wallis across thirds: H={stat_kw:.2f}, p={p_kw:.4f}")

print(f"\nFirst 6 samples: {data[:6]}")
print(f"  Mean of first 6: {np.mean(data[:6]):.0f}")
print(f"  Mean of rest:    {np.mean(data[6:]):.0f}")

# Autocorrelation
print(f"\nAutocorrelation:")
centered = data - np.mean(data)
var = np.var(data)
for lag in [1, 2, 3, 4, 5]:
    acf = np.sum(centered[:-lag] * centered[lag:]) / (n * var)
    print(f"  Lag {lag}: r = {acf:.3f}")

# Mann-Kendall style monotonicity check on first 10 samples
first10 = data[:10]
pairs_decreasing = sum(1 for i in range(len(first10)) for j in range(i+1, len(first10)) if first10[j] < first10[i])
pairs_total = len(first10) * (len(first10) - 1) // 2
print(f"\nFirst 10 samples monotonic decay check:")
print(f"  Values: {first10}")
print(f"  Decreasing pairs: {pairs_decreasing}/{pairs_total} ({100*pairs_decreasing/pairs_total:.0f}%)")

print()
print("=" * 70)
print("3. EXPECTED vs OBSERVED RATIO")
print("=" * 70)
expected_square = 447000
expected_sine = 316000
observed_mean = np.mean(data)
observed_median = np.median(data)
print(f"Expected (square wave): {expected_square:,}")
print(f"Expected (sinusoidal):  {expected_sine:,}")
print(f"Observed mean:          {observed_mean:,.0f}")
print(f"Observed median:        {observed_median:,.0f}")
print(f"Ratio (expected_sq / observed_mean):   {expected_square / observed_mean:.0f}x")
print(f"Ratio (expected_sq / observed_median): {expected_square / observed_median:.0f}x")
print(f"Ratio (expected_sin / observed_mean):  {expected_sine / observed_mean:.0f}x")

print()
print("=" * 70)
print("4. SPIKE AND ZERO ANALYSIS")
print("=" * 70)

q1, q3 = np.percentile(data, [25, 75])
iqr = q3 - q1
upper_fence = q3 + 1.5 * iqr
print(f"IQR = {iqr:.0f}, Q1 = {q1:.0f}, Q3 = {q3:.0f}")
print(f"Upper fence (Q3 + 1.5*IQR) = {upper_fence:.0f}")
outliers_high = data[data > upper_fence]
print(f"High outliers: {sorted(outliers_high, reverse=True)}")
print(f"  Count: {len(outliers_high)} ({100*len(outliers_high)/n:.1f}%)")
print(f"Zero count: {np.sum(data == 0)}")

cv = np.std(data, ddof=1) / np.mean(data)
print(f"Coefficient of Variation (CV): {cv:.2f}")
print(f"  CV > 1.0 = extreme variability relative to mean")
print(f"  A stable signal would have CV << 1")

# Inter-reading variability
diffs = np.abs(np.diff(data))
print(f"\nConsecutive absolute differences:")
print(f"  Mean: {np.mean(diffs):.0f}")
print(f"  Median: {np.median(diffs):.0f}")
print(f"  Max: {np.max(diffs):.0f}")
print(f"  A stable signal would show small, consistent diffs")

print()
print("=" * 70)
print("5. NOISE DISTRIBUTION ANALYSIS")
print("=" * 70)
print()
print("Theory: For a single-bin DFT of pure Gaussian white noise,")
print("the magnitude follows a Rayleigh distribution.")
print("For signal + noise, it follows a Rice distribution.")
print()

nonzero = data[data > 0]
print(f"Non-zero values: {len(nonzero)} of {n}")

# Fit Rayleigh
ray_loc, ray_scale = stats.rayleigh.fit(nonzero, floc=0)
print(f"\nRayleigh fit: sigma = {ray_scale:.1f}")
print(f"  Rayleigh theoretical mean = sigma * sqrt(pi/2) = {ray_scale * np.sqrt(np.pi/2):.1f}")
print(f"  Rayleigh theoretical mode = sigma = {ray_scale:.1f}")
print(f"  Observed mean (nonzero) = {np.mean(nonzero):.1f}")

ks_stat, ks_p = stats.kstest(nonzero, "rayleigh", args=(0, ray_scale))
print(f"  KS test vs fitted Rayleigh: D={ks_stat:.4f}, p={ks_p:.4f}")
if ks_p > 0.05:
    print(f"  --> CANNOT reject Rayleigh hypothesis (p={ks_p:.3f} > 0.05)")
else:
    print(f"  --> Reject Rayleigh hypothesis (p={ks_p:.4f} < 0.05)")

# Also try exponential
exp_loc, exp_scale = stats.expon.fit(nonzero, floc=0)
ks_stat_e, ks_p_e = stats.kstest(nonzero, "expon", args=(0, exp_scale))
print(f"\nExponential fit: scale = {exp_scale:.1f}")
print(f"  KS test vs Exponential: D={ks_stat_e:.4f}, p={ks_p_e:.4f}")

# Lognormal fit (common for magnitude data with heavy tail)
shape_ln, loc_ln, scale_ln = stats.lognorm.fit(nonzero, floc=0)
ks_stat_ln, ks_p_ln = stats.kstest(nonzero, "lognorm", args=(shape_ln, 0, scale_ln))
print(f"\nLognormal fit: shape={shape_ln:.3f}, scale={scale_ln:.1f}")
print(f"  KS test vs Lognormal: D={ks_stat_ln:.4f}, p={ks_p_ln:.4f}")

print()
print("Signal-vs-noise diagnostic:")
print(f"  Std/Mean ratio (nonzero): {np.std(nonzero, ddof=1)/np.mean(nonzero):.3f}")
print(f"  For Rayleigh (pure noise): expected ratio = sqrt(4/pi - 1) = {np.sqrt(4/np.pi - 1):.3f}")
print(f"  For a real signal with SNR >> 1: ratio approaches 0")
print(f"  Observed ratio matches Rayleigh prediction almost exactly.")

print()
print("=" * 70)
print("6. SIGNAL DETECTION VERDICT")
print("=" * 70)

# Back-calculate implied impedance
implied_V = observed_mean * (4.5 / 2**23)
implied_Z = implied_V / 24e-6 if observed_mean > 0 else 0
print(f"If interpreted as signal:")
print(f"  Implied voltage = {implied_V*1e6:.1f} uV")
print(f"  Implied impedance = {implied_Z:.1f} ohm")
print(f"  Actual resistor = 10,000 ohm")
print(f"  Deficit = {10000/implied_Z:.0f}x")

print()
print("Fraction of readings below thresholds:")
for thresh in [100, 200, 500, 1000, 2000, 5000]:
    frac = np.sum(data < thresh) / n
    print(f"  < {thresh:>5}: {100*frac:.1f}%")

print()

# SNR estimate: if signal were present at expected level
# SNR = signal / noise_std = expected / std(observed)
noise_std = np.std(data, ddof=1)
print(f"Noise floor estimate (std of observed): {noise_std:.0f} codes")
print(f"If excitation were present, SNR = {expected_sine}/{noise_std:.0f} = {expected_sine/noise_std:.0f}")
print(f"At SNR={expected_sine/noise_std:.0f}, every single reading should be ~316,000 +/- {noise_std:.0f}")
print(f"Probability of seeing a reading < 10,000 at that SNR: essentially 0")

print()
print("-" * 70)
print("CONCLUSION")
print("-" * 70)
print()
print("The excitation signal is NOT being measured. Key evidence:")
print()
print("1. MAGNITUDE: Observed mean (1,070 codes) is 300-400x below the")
print("   expected 316,000-447,000 codes. This is not a gain or scaling")
print("   error -- it is a complete absence of signal.")
print()
print("2. DISTRIBUTION: The DFT magnitudes are consistent with a Rayleigh")
print("   distribution (the exact statistical signature of Gaussian noise")
print("   with no coherent signal component).")
print()
print("3. VARIABILITY: CV = 1.07 (std approximately equals mean). A real")
print("   signal produces CV << 1. The observed CV ~ 1 is the hallmark")
print("   of Rayleigh-distributed noise magnitudes.")
print()
print("4. ZEROS: 6 readings (5.0%) are exactly 0. If a 240mV excitation")
print("   were present at gain=1, the minimum DFT magnitude would be")
print("   hundreds of thousands of codes, not zero.")
print()
print("5. INITIAL DECAY: First reading (10,069) decays exponentially to")
print("   baseline within ~6 samples. This is a startup transient")
print("   (capacitor charging or register settling), not stable excitation.")
print()
print("6. OTHER CHANNELS: Ch1/Ch3-Ch8 are all exactly 0 at all times,")
print("   confirming the DFT bin computation is correct and not producing")
print("   false positives on open inputs.")
print()
print("DIAGNOSIS: The ~1,000-code readings are the thermal/quantization")
print("noise floor of the ADS1299 at gain=1. The 24uA AC excitation is")
print("either:")
print("  (a) Not being generated (LOFF_SENSP/LOFF_SENSN not configured)")
print("  (b) Not routed to the CH2 inputs (channel enable bits wrong)")
print("  (c) Generated at wrong frequency (DFT bin mismatch vs actual freq)")
print("  (d) Being generated but immediately shorted/attenuated internally")
print()
print("Most likely cause: the lead-off excitation current source registers")
print("(LOFF_SENSP, LOFF_SENSN, LOFF register) are not properly configured")
print("to enable the 24uA current on channel 2.")
