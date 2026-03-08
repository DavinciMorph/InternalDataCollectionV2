# Artifact Investigation

**Last Updated:** 2026-03-07

Consolidated investigation of signal artifacts in the 320-channel EEG system. Covers identified root causes, diagnostic methods, and fixes applied.

---

## 1. 25 Hz Artifact (FIXED)

### Symptoms
- Exact 25.000 Hz (Fs/10) tone visible in PSD on all active channels
- Harmonics at 50, 75, 100, 125 Hz (non-sinusoidal periodic waveform)
- Within-device coherence r=0.97, cross-port coherence 0.94 (common-mode)

### Root Cause
`BATCH_SIZE=10` in TCP streaming created periodic batch boundaries. Artifact locked to array-index position within the 10-sample TCP batch, NOT sample numbers. Folded-sample analysis showed positions 1-3 of every 10-sample cycle had -15/+8/+8 uV offsets.

**Diagnostic:** `osc_13_sample_fold.png` was the smoking gun -- folding all samples modulo 10 revealed the systematic pattern.

### What It Was NOT
- NOT caused by LZ4 compression (incremental autoFlush=1 fix had no effect)
- NOT caused by power supply (artifact disappeared when BATCH_SIZE changed)

### Fix
Removed LZ4 compression entirely + set BATCH_SIZE=1 (per-sample TCP sends). Commit `549f54c`.

Current state in `protocol.hpp`: `BATCH_SIZE=1`, `compression=none`, `format=binary_raw`.

---

## 2. 70 Hz Artifact (PARTIALLY FIXED)

### Symptoms
- ~70 Hz PSD peak on all active channels (~7 uV^2/Hz excess)
- Absent on railed/shorted channels
- Cross-port coherence 0.4-0.9 (common-mode), phase varies by board position (3-174 degrees)

### Root Cause
Pi GPU display pipeline running at ~70 Hz refresh rate, EMI coupling into electrode traces on the PCB.

**Evidence:**
- Test-signal mode (CONFIG2=0xD0, CHnSET=0x05): 70 Hz completely absent -- enters via analog inputs only
- Not from firmware: exhaustive code review confirmed all periodic operations alias to DC at 250 Hz
- Not 60 Hz AM sideband: no 50 Hz companion

### Fix Applied
Disabled GPU output in `/boot/firmware/config.txt`:
- `max_framebuffers=0` -- no framebuffer allocation
- `hdmi_blanking=2` -- HDMI output blanked
- `dtoverlay=vc4-kms-v3d,noaudio` -- HDMI audio disabled

Additional peripherals disabled for further EMI reduction:
- `dtoverlay=disable-bt` -- Bluetooth radio off
- `dtparam=audio=off` -- BCM2835 audio driver off
- `camera_auto_detect=0` -- CSI camera probing off
- `display_auto_detect=0` -- DSI display probing off
- `enable_tvout=0` -- composite video DAC off

**Result:** 4x reduction on clean channels, 2.3x aggregate. GPU was the dominant source. 60 Hz pickup also reduced by 3.3x.

### Residual
~2.7 uV^2/Hz remains on cleanest channels. May be remaining Pi peripheral activity or environmental EMI. Full elimination would require shielding or moving the Pi further from the electrode board.

---

## 3. DRDY Poll Alias at 71.53 Hz (FIXED)

### Symptoms
Timestamp jitter peak at 71.53 Hz in the dt spectrum.

### Root Cause
Old `usleep(200)` in the DRDY polling loop gave a variable poll rate (~2857 Hz). The variable timing created I2C EMI that aliased to 71.53 Hz in the EEG bandwidth.

### Fix
Replaced with `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` on a fixed 400us grid (2500 Hz). 2500 Hz = 10 x 250 Hz, so the alias lands at 0 Hz (DC), invisible in AC-coupled EEG.

Implementation in `drdy_poller.hpp`/`drdy_poller.cpp`:
- `POLL_INTERVAL_NS = 400'000L`
- `static_assert(4'000'000L % POLL_INTERVAL_NS == 0)` prevents non-divisor values
- Overrun self-correction: if I2C read exceeds 400us, skips grid points forward

**Result:** 71.53 Hz timestamp jitter peak eliminated, dt max improved 4.66ms to 4.35ms.

---

## 4. PGA Saturation "Spikes" (NOT A FIRMWARE BUG)

### Symptoms
User-reported "periodic spikes" in the visualizer, especially on certain channels.

### Root Cause
High-impedance electrodes saturating the ADS1299 PGA to near-negative-rail values (0x83xxxx). The "spikes" are channels transitioning between their normal EEG signal and the near-negative-rail saturation state as electrode contact fluctuates.

**Evidence:**
- Each channel's 0x83xxxx offset is unique and stable (not random bit errors)
- Adjacent channels on the same device are NOT correlated (dev3_ch5 railed, dev3_ch6 normal)
- Noise spectrum shows real 60 Hz pickup (reading real analog input, not digital garbage)
- Zero simultaneous multi-channel transitions in File 1 (all 734 onsets are single-channel)
- ISIs are irregular (std >> mean), not periodic

### Channel-Specific Pattern
Channel 5 is systematically affected: 52-55% of all ch5 inputs across 42 devices are always railed. This indicates a systematic wiring/connector problem specific to the ch5 electrode position.

### No Fix Needed
This is an electrode/connector quality issue, not a firmware or SPI bug. The C++ acquisition engine shows perfect data integrity (zero gaps, zero missed samples, zero SPI corruption).

---

## 5. Growing Oscillation Artifact (UNDER INVESTIGATION)

### Symptoms
Broadband oscillation appears after ~5 minutes of continuous recording. Starts as discrete 1-3 second bursts that grow and merge into continuous oscillation. Amplitude up to ~1300 uV at gain=24. Simultaneous onset across all 4 SPI buses.

### What Clears It
SOFTWARE RESET restores clean signal for another ~5 minutes. Same electrodes, same cap, same body position.

### What Does Not Cause It
- Shorted inputs: completely clean with CHnSET=0x61 (no oscillation)
- BIAS/DRL: Port1 with full BIAS grew at same rate as other ports
- Gain setting: appears at both 12x and 24x
- DC offset saturation: ports with 300+ mV headroom still oscillate
- Register corruption: burst pattern (appears/disappears/reappears) inconsistent with register corruption
- Movement: user confirmed static; single-sample transients too fast for mechanical motion
- 60 Hz mains: oscillation is broadband (1-50 Hz), persists after 60+120 Hz notch

### Leading Theory: Electrode Polarization

The delta-sigma modulator's switched-capacitor inputs run at ~500 kHz, injecting tiny charge packets into electrodes. With real electrodes, charge accumulates at the electrode-skin double-layer capacitance, gradually polarizing the interface. After ~5 minutes:
- Electrode impedance rises past a threshold
- Instability appears as bursts
- RESET stops the modulator, allowing partial depolarization

This explains every observation: shorted inputs can't polarize, RESET clears it, it's gain-independent, gets worse over time, and requires electrodes.

### Proposed Diagnostic
Implement keyboard-triggered interventions at 4 levels:
1. STOP/START only (~10ms pause)
2. SDATAC/RDATAC cycle (~50ms)
3. SOFTWARE RESET without register rewrite (~500ms)
4. Full restart (current behavior)

If Level 1 works, a preventive periodic START toggle (10ms every 2-3 min, 0.006% data loss) would be a permanent fix.

---

## 6. "Periodic Spikes" at t=21s and t=66s (NOT REAL)

### Context
User reported spikes at t=21s and t=66s in a 10-minute recording (`client/10minFULLclean.csv`).

### Analysis
Six independent detection methods found NO periodic pattern at ~45s or any other period. Direct examination at t=21s and t=66s showed no discrete events on any port or channel.

**What was actually found:**
- 7 large transient events with irregular ISIs (9s to 267s, CV=1.054)
- These are physical movement/electrode-contact artifacts
- Event at t=149.15s was genuine: 10,320 uV, 18ms timestamp gap (physical electrode displacement)
- Permutation tests: p=0.32 (21s), p=0.80 (66s), p=1.0 (45s) -- all non-significant
- Folding SNR monotonically increasing (r=0.957) -- proves no periodic signal

### Conclusion
Zero sample gaps, zero corruption across 165,010 samples. The acquisition system is clean. The "spikes" the user saw were likely visualization artifacts from auto-scaling or filter transients in `simpleviz.py`.

### Filter Artifact Note
The 1 Hz Butterworth high-pass filter converts slow DC drift (from electrode impedance changes) into oscillating transients that look like spikes. This is a known property of causal HPF applied to non-stationary signals.

---

## 7. 500 SPS Alias Analysis (INFORMATIONAL)

At 500 SPS (tested but not currently used), the critical 10 Hz alpha-band contamination from 250 SPS is eliminated. However, new interference appears:

| Issue | 250 SPS | 500 SPS |
|-------|---------|---------|
| Aliases in brain bands (0-50 Hz) | 14 | 8 |
| Aliases in alpha (8-13 Hz) | 2 (at 10 Hz) | 0 |
| Brain-band aliases remaining | H8->20 Hz, H9->40 Hz | Same (notch-filterable) |

A 10 Hz environmental interference source was also found (discrete tones at multiples of 10 Hz), likely from power supply ripple or lighting. This is environmental, not aliasing, and persists at any sample rate.

**Current system uses 250 SPS.** The jump to 500 SPS would be the most impactful improvement if alpha-band purity is needed.
