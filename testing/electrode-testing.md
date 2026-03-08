# Electrode Testing

**Last Updated:** 2026-03-07

Consolidated electrode characterization results including noise floor baselines, orientation sweep, material comparison, power supply A/B testing, single-layer electrode log, and 3-layer electrode log.

---

## 1. Test Setup

| Parameter | Value |
|-----------|-------|
| Test position | Port1 Dev9 (9-deep chain -- worst case) |
| CONFIG3 | 0xE0 (internal reference) |
| CHnSET | 0x60 (gain=24, normal input) |
| Sample rate | 250 Hz |
| Duration per test | 120s (30,000 samples) |
| ADC LSB | 0.02235 uV (gain=24, Vref=4.5V) |
| Analysis filters | 60/70/120 Hz notch; 60 Hz power measured on raw data |

### Grading Criteria

| Grade | SNR | 60 Hz |
|-------|-----|-------|
| A* | >= 50 dB | < 0.15 mV |
| A | >= 43 dB | < 1.0 mV |
| B | >= 35 dB | < 2.0 mV |
| C | >= 25 dB | < 5.0 mV |
| F | < 25 dB or >= 5.0 mV |

---

## 2. Baseline Measurements (No Signal)

| Test | RMS (uV) | P2P (uV) | NSD (uV/rtHz) | Notes |
|------|----------|----------|----------------|-------|
| B-01: Shorted, internal ref | 0.150 | 1.217 | 0.0146 | Pure thermal noise, ~7 LSB |
| B-02: Shorted, external ref | 86.4 | 715 | 8.17 | 576x worse than internal |
| B-03: Floating, internal ref | 1,202 | 16,285 | 113 | 60 Hz dominant |
| B-04: Floating, external ref | 0.000 | 0.000 | 0.000 | All zeros -- non-functional |

**Conclusion:** External reference is broken/absent on this board. Internal reference only.

---

## 3. Orientation Sweep

Signal: 1 Hz square wave, -60dB (500 uV).

| Orientation | Mean RMS (uV) | Mean SNR (dB) | Notes |
|-------------|---------------|---------------|-------|
| 0 degrees | 326.9 | 46.3 | Best -- no outlier channels |
| 90 degrees | 337.3 (excl Ch6) | 45.0 | Ch6 outlier (bad contact from reinsertion) |
| 180 degrees | -- | -- | SKIPPED (user observed poor performance) |
| 270 degrees | -- | -- | SKIPPED |

**Best orientation: 0 degrees.** Reinsertion contact variability dominates over rotation angle.

---

## 4. Electrode Material Comparison

| Material | Signal | Mean RMS (uV) | Mean SNR (dB) | 60 Hz (mV) |
|----------|--------|---------------|---------------|------------|
| Old resin | -60dB | 281.8 | 49.4 | 4.4 |
| Old resin | -80dB | 63.2 | 32.8 | 3.96 |
| **New resin (3g Ag + 2.32g)** | **-60dB** | **270.0** | **60.0** | **0.46** |
| **New resin** | **-80dB** | **27.8** | **45.8** | **0.48** |
| Silver | -60dB | 126.2 | 28.1 | 7.36 |

**Best material: New resin formula (3g Ag + 2.32g Resin).**

New resin vs old resin at -80dB:
- 2.3x lower RMS
- +13 dB SNR
- 8.3x less 60 Hz pickup

New resin vs silver at -60dB:
- +32 dB SNR
- 16x less 60 Hz pickup

---

## 5. Power Supply A/B Comparison

**Date:** 2026-02-21. Signal: 1 Hz, 50 uV square wave. 8 runs across 2 battery supplies.

| Metric | Battery A (settled) | Battery B (cold) |
|--------|--------------------:|------------------:|
| SNR | ~44 dB | ~47 dB |
| NSD | 1.643 uV/rtHz | 1.548 uV/rtHz |
| Run-to-run SNR spread | 3.6 dB | 0.9 dB |
| Settling time | ~5 min | < 2 min |

Battery B cold-start outperforms Battery A fully settled. Battery B is 4x more consistent.

Thermal settling confirmed: SNR improves 2.4 dB over first 5 minutes. 60 Hz is purely radiated pickup (both supplies are batteries).

**Recommendation:** Use Battery B for all precision measurements. Allow 5+ minutes settling.

---

## 6. Single-Layer Electrode Characterization

27 electrodes tested (2026-02-21). Resin formula: 3g Ag + 2.32g Resin.

### Summary

| Batch | Pass Rate | Mean SNR | Median SNR | Mean 60 Hz |
|-------|-----------|----------|------------|------------|
| Day-old resin (E-01 to E-24) | 3/24 (12.5%) | 23.9 dB | 22.4 dB | 30.9 mV |
| **Fresh resin (E-25 to E-27)** | **3/3 (100%)** | **43.6 dB** | **44.3 dB** | **0.72 mV** |

### Key Finding: Resin Pot Life

Fresh resin = 100% pass rate. Day-old resin = 12.5% pass rate. Silver particles settle overnight, destroying shielding effectiveness. **Resin must be applied immediately after mixing.**

### Evidence
- 60 Hz shielding is the bottleneck: r=0.964 correlation between 60 Hz and RMS. 93% of noise variance explained by shielding alone.
- Day-old batch 60 Hz spans 0.41-103.9 mV (253x range, CV=95.6%)
- 3 day-old passes (E-06, E-09, E-14) scattered at random positions -- lucky silver-rich scoops
- Uncoated channels on E-26/E-27 show -2 to -3 dB SNR, proving resin coating provides the shielding

### Notable Electrodes

| Electrode | SNR (dB) | 60 Hz (mV) | Grade | Notes |
|-----------|----------|------------|-------|-------|
| E-06 | 42.8 | 0.41 | A* | Day-old, lucky silver-rich scoop |
| E-14 | 43.0 | 0.64 | A | Day-old, lucky scoop |
| E-25 | 45.1 | 0.39 | A | Fresh resin, coated=48dB, uncoated=42dB |
| E-27 | 44.3 | 0.68 | A | Fresh resin, 6ch (2 uncoated) |

---

## 7. 3-Layer Electrode Characterization

17 three-layer electrodes tested (2026-02-26). Fresh resin, Battery B.

### Reference Baseline
3-layer prototype: 26.0 uV RMS, 55.4 dB SNR, 0.097 mV 60 Hz.

### Results Summary

| Grade | Count | Percentage |
|-------|-------|-----------|
| A* (SNR>=50, 60Hz<0.15) | 7 | 41% |
| A (SNR>=43, 60Hz<1.0) | 9 | 53% |
| B (SNR>=35, 60Hz<2.0) | 1 | 6% |
| F | 0 | 0% |

**100% pass rate (A or above): 16/17.** One B grade (3L-11, short run with high 60 Hz).

### Best Electrodes

| Electrode | SNR (dB) | 60 Hz (mV) | Grade | Ch Coverage |
|-----------|----------|------------|-------|-------------|
| 3L-04 | 57.6 | 0.133 | A* | 7/8 (1 bad contact), 1.1 dB spread |
| 3L-08 | 57.5 | 0.113 | A* | **8/8 channels**, best overall, 1.6 dB spread |
| 3L-02 | 56.9 | 0.105 | A* | 6/8 (1 bad contact, 1 outlier) |
| 3L-17 | 56.6 | 0.124 | A* | 7/8 (1 bad contact) |
| 3L-16 | 55.5 | 0.137 | A* | 7/8 (1 bad contact) |

### Observations
- NSD remarkably consistent: 1.552-1.577 uV/rtHz across all 17 electrodes (dominated by ADC thermal noise)
- Bad contact channels (Ch4, Ch5, Ch8 most common) are connector/mechanical issues, not coating quality
- 3L-08 is the only electrode with 8/8 channels working simultaneously

---

## 8. Conclusions

### Optimal Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Reference | Internal (CONFIG3=0xE0) | External ref non-functional |
| Orientation | 0 degrees | Cleanest results |
| Electrode material | New resin (3g Ag + 2.32g) | 60 dB SNR at -60dB |
| Coating | 3 layers, fresh application | 100% pass rate, 55+ dB SNR |
| Power supply | Battery B | +3 dB SNR, 4x more consistent |
| Settling time | >= 5 minutes | Thermal + electrochemical settling |

### Noise Budget

| Source | Contribution |
|--------|-------------|
| ADC intrinsic (shorted) | 0.150 uV RMS |
| Electrode thermal | ~1.5 uV/rtHz (NSD, 3-layer) |
| 60 Hz EMI (with coating) | 0.1-0.25 mV |
| 60 Hz EMI (without coating) | 5-100 mV |
| Environmental 10 Hz interference | ~15 dB above noise floor |

**60 Hz shielding quality determines overall noise performance.** 93% of noise variance is explained by shielding alone (r=0.964).

### Manufacturing Recommendations

1. Apply resin immediately after mixing (never use day-old batch)
2. Three coating layers for best results
3. Use Battery B power supply
4. Allow 5+ minutes thermal settling before measurements
5. Inspect ch4/ch5/ch8 connector seating (most common bad-contact channels)
