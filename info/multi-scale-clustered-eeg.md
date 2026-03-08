# Multi-Scale Clustered EEG: Architecture and Implications

## System Overview

This system uses 28 ADS1299 analog front-ends (scalable to 41), each providing 8 channels of 24-bit EEG acquisition at 250 Hz. The key architectural distinction from conventional EEG is **electrode clustering**: each ADS1299's 8 electrodes are physically co-located within a few millimeters of each other, and these clusters are distributed across the scalp.

This geometry creates a multi-scale spatial sensor that captures two fundamentally different types of neural information simultaneously.

| Parameter | Value |
|-----------|-------|
| Devices | 28 ADS1299 (scalable to 41) |
| Channels per device | 8 |
| Total channels | 224 (scalable to 328) |
| Sample rate | 250 Hz |
| ADC resolution | 24-bit |
| PGA gain | 24x |
| Intra-cluster spacing | ~few mm |
| Inter-cluster spacing | ~cm (distributed across scalp) |

## Two Simultaneous Spatial Scales

### Macro Scale: Cluster-to-Cluster

Each cluster's mean signal (average of 8 co-located electrodes) acts as a single high-SNR virtual electrode. With 28-41 clusters distributed across the scalp, this produces a montage comparable in spatial density to the standard 10-20 system, but with significantly better signal-to-noise ratio at each position due to 8-channel averaging.

This scale captures **far-field signals** — electrical activity that is volume-conducted across large regions of the scalp:

- Eye blink artifacts (50-500 uV, frontal dipole)
- Event-related potentials (P300, N170, etc.)
- SSVEP (steady-state visually evoked potentials)
- Sensorimotor rhythms (mu/beta, 8-30 Hz)
- Alpha rhythm (8-13 Hz, posterior)
- Sleep spindles, K-complexes
- Seizure activity

**No spatial filtering (CMF) should be applied** when analyzing these signals. The raw cluster mean preserves them.

### Micro Scale: Within-Cluster Gradients

Applying a Common Mode Filter (CMF) — subtracting the cluster mean from each channel — removes the shared far-field signal and isolates the **local spatial gradient** across the few-millimeter electrode span. This is equivalent to a surface Laplacian or current source density (CSD) estimate at each cluster location.

This scale captures **near-field signals** — local cortical activity directly beneath the electrode cluster:

- Local cortical dipole orientation
- Current source density variations
- High-frequency cortical oscillations (gamma, high-gamma)
- Micro-scale spatial patterns invisible to conventional EEG

## Why This Architecture Is Novel

Conventional EEG systems place one electrode per position. A 256-channel system provides 256 spatial samples at one scale. This system provides:

1. **28-41 macro-scale samples** (cluster means) with ~3x better SNR than single electrodes (sqrt(8) noise reduction)
2. **28-41 independent micro-scale gradient maps** (within-cluster CMF) at locations where conventional EEG has no spatial resolution at all

These are not two ways of looking at the same data — they are two different physical measurements derived from the same electrode array.

## Unique Capabilities

### 1. Adaptive Single-Electrode Artifact Rejection

With 8 co-located electrodes, if 7 agree and 1 diverges, the divergent channel has an electrode-specific artifact (poor contact, movement, impedance change). The system can detect and reject bad electrodes in real-time without losing the measurement at that scalp location. Conventional EEG has no redundancy — every electrode is a single point of failure.

### 2. Quality-Weighted Cluster Averaging

Instead of a simple mean, each channel can be weighted by its instantaneous agreement with neighbors (cross-correlation). Channels with degrading contact are automatically downweighted. This produces better-than-sqrt(8) noise reduction and adapts continuously during recording.

### 3. Per-Cluster PCA Decomposition

Principal Component Analysis on each cluster's 8 channels provides an ordered decomposition:
- **PC1**: Best estimate of the shared signal (superior to simple mean)
- **PC2-8**: Local gradient modes, ordered by variance

This is a more principled decomposition than the binary raw/CMF split, and it adapts to the actual signal structure at each location.

### 4. Source Depth Estimation

The magnitude of the local spatial gradient (within-cluster CMF signal) relative to the cluster mean relates to source depth:
- **Steep gradient**: Shallow source (cortical surface directly under the cluster)
- **Flat gradient**: Deep source (subcortical) or distant far-field signal

This provides per-cluster, per-time-point depth information that is impossible with conventional single-electrode-per-position EEG.

### 5. Cortical Traveling Wave Detection

Neural activity propagates across the cortex as traveling waves. Detecting these requires both spatial coverage (to see the wave front) and high SNR at each location (to resolve timing differences). The cluster architecture provides both: macro-scale coverage across the scalp with micro-scale SNR enhancement at each node.

### 6. Real-Time Impedance Monitoring

The divergence of any single channel from its cluster neighbors is a direct, continuous measure of electrode impedance degradation — without injecting test currents or interrupting acquisition. This enables live quality monitoring during long recordings or BCI sessions.

## Implications for BCI

### What This System Can Do

- **Higher classification accuracy** than conventional EEG at the same electrode count, due to per-position noise reduction
- **Multi-scale feature extraction**: Feeding both raw and CMF-derived features into a decoder provides more information than either alone
- **Robustness**: Adaptive artifact rejection makes the system more reliable during real-world use (movement, sweat, long sessions)
- **Graceful degradation**: Losing one electrode in a cluster has minimal impact; losing one electrode in conventional EEG loses an entire spatial sample

### What This System Cannot Do

- **Match invasive ECoG performance**: The skull acts as a spatial low-pass filter with a cutoff around 2-3 cm. No surface electrode arrangement can recover the sub-millimeter spatial detail available to electrodes placed directly on the cortex. The amplitude gap between surface EEG and ECoG is approximately 10x, and the spatial resolution gap is 10-100x.

### Where It Sits

This architecture occupies a space between conventional surface EEG and invasive ECoG that doesn't currently exist in commercial or research systems. It cannot replace invasive recording, but it extracts more information from the scalp surface than any single-electrode-per-position system can — regardless of electrode count.

## Signal Processing Recommendations

| Analysis Goal | Use Raw (no CMF) | Use CMF | Use Both |
|---------------|:-:|:-:|:-:|
| Blink detection / EOG | X | | |
| P300 / ERP | X | | |
| SSVEP | X | | |
| Motor imagery BCI | X | | |
| Alpha neurofeedback | X | | |
| Local cortical mapping | | X | |
| Source depth estimation | | | X |
| Multi-scale BCI decoding | | | X |
| Artifact rejection | | | X |
| Noise characterization | | | X |

## File References

- `testing/blink_test.py` — Eye blink detection tool (uses raw signal, no CMF)
- `client/simpleviz.py` — Real-time visualizer (CMF toggle available for display)
- `testing/noise_test.py` — Noise characterization (raw signal analysis)
