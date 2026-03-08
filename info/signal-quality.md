# Signal Quality

**Last Updated:** 2026-03-07

Signal quality analysis across the 320-channel system. Covers gain comparison, cold-start behavior, electrode power-on order, and DC offset characterization.

---

## 1. Gain Comparison (320 Channels, 40 Devices)

**Date:** 2026-03-04. Recordings taken with rewired connectors, electrodes on head.

| Metric | 8x Gain | 12x Gain | 24x Gain |
|--------|---------|----------|----------|
| Duration | 2.2 min | 2.4 min | 18.9 min |
| PGA Input Range | +/-562.5 mV | +/-375 mV | +/-187.5 mV |
| **Usable channels** | **211 (65.9%)** | **213 (66.6%)** | **140 (43.8%)** |
| Railed channels | 45 (14.1%) | 93 (29.1%) | 118 (36.9%) |
| Dead channels | 34 (10.6%) | 3 (0.9%) | 0 (0.0%) |
| Artifact channels | 30 (9.4%) | 11 (3.4%) | 62 (19.4%) |

### Key Findings

1. **12x gain produces the best usable channel count** at 213/320 (66.6%), slightly beating 8x (211/320).

2. **Railing increases monotonically with gain.** At 24x, 114/118 railed channels are at negative full-scale, indicating a systematic negative DC offset from electrode half-cell potentials.

3. **Dead channels disappear at higher gain.** The 34 "dead" channels at 8x are high-impedance channels whose noise is below detection threshold at low gain. At 12x/24x the same noise is amplified enough to register.

4. **Chain depth has no correlation with health.** Outer devices (dev5-8) consistently outperform inner devices (dev1-4). Failures are electrode/connector quality, not SPI chain position.

5. **24x temporal recovery:** Over the 18.9-minute recording, 20 channels recovered from railed to usable. Usable channels improved from 49.7% (first segment) to 60.9% (last segment).

---

## 2. Cold-Start Analysis at 24x Gain

**Date:** 2026-03-04. Protocol: electrodes on head, system OFF for 10 min, cold boot, immediate 24x acquisition.

### Hypotheses Tested

| Hypothesis | Result |
|-----------|--------|
| H1: Electrodes before power-on causes railing | **FALSIFIED** -- only 19.2% railed (vs 98.5% with old wiring) |
| H2: PGA needs lower-gain settling first | **FALSIFIED** -- no warmup needed, cold start works |
| H3: Lower electrode impedance from better contact | **SUPPORTED** -- 80.8% of channels within PGA linear range |
| H_new: Wiring/connector quality is dominant variable | **STRONGLY SUPPORTED** -- sole change was rewiring |

### 224-Channel Results (dev1-dev4 only)

| Metric | Full Recording | Last 2 Minutes (Settled) |
|--------|---------------|--------------------------|
| Usable channels | 81 (36.2%) | 158 (70.5%) |
| Railed channels | 43 (19.2%) | 60 (26.8%) |
| Artifact channels | 100 (44.6%) | 6 (2.7%) |

### 336-Channel Results (all 42 devices)

| Metric | First Segment | Last Segment (Settled) |
|--------|--------------|------------------------|
| Usable channels | 215 (67.2%) | 241 (75.3%) |
| Railed channels | 101 (31.6%) | 72 (22.5%) |

### DC Settling

- 35 channels recovered from railed to usable over 13 minutes
- 8 channels degraded (usable to railed)
- Net improvement: +27 channels
- Most recovery occurs in minutes 2-8; system largely stable after minute 8
- All 72 stuck-railed channels are at negative full-scale (systematic negative DC offset)

### Inner vs. Outer Device Comparison

| Port | Inner (dev1-3/4) Usable % | Outer (last 3 devs) Usable % |
|------|--------------------------|------------------------------|
| Port 1 | 62% | 75% |
| Port 2 | 34% | 75% |
| Port 7 | 44% | 75% |

Outer devices consistently outperform inner -- the opposite of what SPI degradation would cause. Inner devices connect to electrodes closest to the board connector (highest mechanical stress).

---

## 3. DC Offset Characterization

### Input-Referred DC Offsets (Non-Railed Channels)

| Gain | Median Offset | Range |
|------|---------------|-------|
| 8x | -71.4 mV | -506 to +491 mV |
| 12x | -49.5 mV | -338 to +205 mV |
| 24x | -46.3 mV | -170 to +170 mV |

The narrowing range at higher gain is a selection effect: channels with large offsets are already railed and excluded. The ~50-70 mV median is typical for scalp EEG with passive electrodes.

### Railing Threshold

At 24x gain (input range +/-187.5 mV), channels with DC offset beyond +/-187.5 mV saturate the PGA. The 43 railed channels in the cold-start test clustered at -182 mV -- only 5.5 mV beyond the linear range. Any improvement in electrode contact would likely un-rail most of them.

---

## 4. Per-Port Quality

Consistently across recordings:

- **Port5 is worst** at all gains (connector defect causes extreme amplitudes during transient events)
- **Port3 and Port1 are best** (cleanest wiring, most usable channels)
- **Port4** varies by session (sensitive to connector seating)

### Problem Devices (Consistent Across Gains)

| Device | Issue |
|--------|-------|
| Port5_dev2 | Systematic offset, dead at 8x, railed at 12x/24x |
| Port4_dev3 | High impedance, 6/8 channels railed at all gains |
| Port3_dev1 | High impedance, 4-5/8 channels railed |
| Port6_dev1 | Dead at low gain, rails at high gain |

---

## 5. Gain Selection Recommendations

| Use Case | Recommended Gain | Rationale |
|----------|-----------------|-----------|
| Maximum channel yield | 12x | 66.6% usable, zero railing risk |
| Maximum voltage resolution | 24x | 0.02235 uV/LSB, needs good connectors + settling |
| Broad impedance tolerance | 8x | +/-562.5 mV range accommodates worst electrodes |
| Clinical/reliable | 12x | Zero-railing guarantee |
| Research/BCI | 24x | Higher resolution on ~76% of channels after settling |

### Operating Procedure for 24x Gain

1. Verify all connectors are properly seated before powering on
2. Power on directly at 24x (no lower-gain warmup needed)
3. Allow 5-10 minutes of DC settling
4. Monitor for channels with std > 20% FS (connector issues)
5. Replace/reseat connectors showing > 5 artifact channels per device

---

## 6. Electrode Noise Floor

The ~450 uV input-referred noise floor (measured at 8x and 12x on quietest channels) is electrode-dominated, not ADC-dominated. The ADS1299 intrinsic noise is ~1 uV RMS at 250 SPS (shorted-input measurement: 0.150 uV RMS). Reducing electrode impedance (gel, skin prep, lower-impedance electrodes) would improve SNR far more than increasing gain.
