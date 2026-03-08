# Corruption and Timing

**Last Updated:** 2026-03-07

Analysis of SPI data corruption mechanisms and timing integrity. Covers the Python-era corruption patterns and their elimination in the C++ rewrite.

---

## 1. SPI Bit-Shift Corruption (Historical -- Python Era)

### Mechanism

Corruption was caused by SPI bit-shift desynchronization: timing stalls on the acquisition engine caused the SPI read to start late. By that time, the ADS1299 shift registers had already advanced, producing data left-shifted by N bits.

### Signature

- Corrupt values approximately equal `expected * 2^N` where N = 1, 4, 6, or 7 bits
- ch2 of every affected device becomes a small constant (status register bits leaking through): `ch2 = 2^N - 1`
- Corruption always starts at the deepest device in the chain and works inward
- ch1 of every device is ALWAYS clean (status bytes absorb the shift)

### CS0/CS1 Asymmetry

On dual-CS SPI buses (SPI0, SPI3, SPI4), only CS1 ports were corrupted, never CS0. This was because the engine reads CS0 first -- when a stall occurs, CS0 reads complete within the safe window but CS1 reads start late.

### Quantitative Results (Python, Pre-DMA)

| Configuration | Corrupt Values per ~100k Samples |
|---------------|----------------------------------|
| xfer2, 6 MHz (baseline) | 0-3 |
| With client connected | 14-61 |
| Sluggish client | 36 jitter events, 61 corrupt |
| readbytes (WORSE) | 96 in 318k samples |
| Dual-speed SPI | ~34 (CS glitch from spi_setup ioctl) |
| Per-bus DRDY trigger | ~38 (Python overhead regression) |

### Root Causes

1. **PIO mode on SPI3/4/5:** CPU preemption during PIO transfers caused SPI clock stalls past the 4ms DRDY boundary. Fixed by enabling DMA on all SPI buses.

2. **CSV writes in hot loop:** SD card I/O stalls up to 1564ms when kernel page cache flushed. Fixed by async deque + writer thread.

3. **Python GC pauses:** Gen2 collection sweeps froze all threads. Fixed by `gc.disable()` during acquisition.

4. **Network activity:** WiFi interrupts on cores 0-2 preempted SPI bus workers. The SPI workers ran on shared cores while only the acquisition thread was on isolated core 3.

### C++ Elimination

The C++ rewrite eliminated ALL of these corruption sources:
- Direct SPI ioctl (no Python overhead)
- DMA on all buses (no PIO)
- SPSC ring buffer for CSV (no I/O in hot loop)
- SCHED_FIFO 50 on core 3, mlockall, /dev/cpu_dma_latency=0
- Eventfd-triggered bus workers (no polling overhead)

**First C++ run:** 23.3 min, 349k samples, zero corruption events.

---

## 2. Time Synchronization (VERIFIED PASS)

Analysis of a 312,990-sample recording (20.9 minutes, 328 channels) confirmed perfect time synchronization.

### Timestamp Consistency

| Metric | Value |
|--------|-------|
| Mean dt | 3.9999 ms |
| Std dev | 0.0939 ms |
| Min / Max dt | 3.78 / 4.22 ms |
| Large gaps (>6ms) | 0 |
| Effective rate | 250.0042 Hz (0.0017% error) |

### Sample Number Continuity

All 312,989 increments are exactly +1. Zero gaps, zero duplicates, zero missed DRDY cycles.

### Cross-Port Synchronization

Within-port lag between any two devices: 0 samples at noise-floor correlation r=0.9999. The daisy chain read is perfectly synchronized -- all devices in a chain are read in a single SPI transaction.

### Cross-Port Correlation Groups

The noise-floor analysis revealed three groups of ports that share analog signal paths:

| Group | Ports | Noise r |
|-------|-------|---------|
| A | Port1, Port2, Port5 | 0.9999 |
| B | Port3, Port7 | 1.0000 |
| C | Port4, Port6 | 1.0000 |

These correlations reflect physical electrode proximity, NOT timing skew. Evidence: some pairs show sign inversion (impossible from pure timing shift), and the groups span different SPI buses.

### Edge Offset Stability

The ~1 Hz test signal edge offsets between ports are perfectly constant across all 2,445 edges (zero standard deviation). These offsets reflect fixed cable lengths, not data alignment issues.

---

## 3. DRDY Timing Budget

At 250 Hz, DRDY fires every 4ms. Data stays valid until the next DRDY overwrites it.

```
Margin = 4ms - detection_delay - trigger_overhead - read_time
       = 4ms - detection_delay - ~0.1ms - ~0.288ms
       ~ 3.6ms - detection_delay
```

For 2-port buses, CS0 is read first (~144us), then CS1. CS1 has ~144us less margin. The real corruption threshold is ~7.7-7.8ms total gap (not 8ms).

### Current C++ Performance

- dt: mean 4.000ms, max typically <4.5ms
- Cycle time: ~0.6ms mean (well within 3ms headroom)
- DRDY timeout in hot loop: 8ms (`poll(0.008)`)
- DRDY poll rate: 2500 Hz fixed grid (400us intervals)

---

## 4. Corruption Detection

### check_corruption.py

Fast CSV corruption analysis tool. Usage:
```bash
python check_corruption.py recording.csv
```

Detects:
- Single-sample spikes (neighbor comparison)
- Rail/saturation values
- Bit-shift patterns (power-of-2 values)
- Startup artifacts
- Timestamp irregularities (gaps, non-monotonic)
- Missing samples
- Out-of-range values

### Per-Device Status Validation (C++ Hot Loop)

The acquisition engine validates the status byte of every device on every sample. Status byte format: top nibble should be 0xC (for CONFIG1[4:0] content). Any mismatch indicates a corrupted read.

### Post-Hoc Correction Possibility

Since the bit-shift amount is encoded in the ch2 sentinel value (ch2 = 2^N - 1), corrupted samples could theoretically be corrected by right-shifting the data by N bits. This would recover ~90% of information from corrupt samples. Not implemented since C++ eliminated corruption.
