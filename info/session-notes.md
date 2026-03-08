# Session Notes

**Last Updated:** 2026-03-07

Chronological log of major sessions and changes. For technical details, see the topic-specific documents.

---

## 2026-02-15: SPI DMA Enablement + CSV Fix

### DMA (COMPLETED)
Enabled DMA on SPI3, SPI4, and SPI5. SPI0 already had DMA.

- **SPI3:** Overlay only. DREQs 16/18 (mislabeled as "SPI1" in datasheet).
- **SPI4:** Overlay + PACTL_CS mux bit 24. DREQs 19/20 shared with UART3.
- **SPI5:** Overlay + PACTL_CS mux bit 25 + HDMI audio disabled. Uses DMA4 controller (`&dma40`), not legacy. DREQs 21/22 shared with UART5.
- **WiFi breakage:** Expanding legacy DMA mask (adding channels 1/3) broke WiFi. GPU uses those channels internally. Reverted.
- **DMA4 mask:** Firmware overwrites DMA4 channel masks after overlays, so expansion overlay has no effect.

See `pi/SPI-DMA-SETUP.md` for full setup documentation.

### CSV Write Stall Fix (COMPLETED)
Root cause: CSV writes in hot loop caused SD card I/O stalls up to 1564ms. Combined with Python GC pauses.

Fix: Async deque + writer thread (1MB buffer), GC disabled during acquisition.

Result: Max jitter 1564ms -> 8ms. Corruption rate 0.0098% -> 0.0001%.

### Init Zeros Issue (Identified)
Intermittent zeros on init (ports returning all zeros despite DRDY toggling). Root causes identified: insufficient SPI flush (32 bytes vs 108-byte frame), status check only validates device 0, single restart attempt during warmup. All fixed in C++ rewrite.

---

## 2026-02-16: SPI Read Optimization

Three optimization phases tested, all ultimately reverted:

1. **readbytes vs xfer2:** Controlled A/B test showed readbytes is 7x worse (96 vs 14 corrupt values in ~320k samples). MOSI floating during readbytes causes noise coupling. **USE xfer2.**

2. **Dual-speed SPI:** Writing `spi.max_speed_hz` triggers kernel `spi_setup()` ioctl that toggles CS, corrupting RDATAC frame boundary. Speed-independent (34 corrupt at both 5 and 6 MHz). **NEVER modify SPI config during RDATAC.**

3. **Per-bus DRDY triggering:** All devices share external clock, so DRDY edges are synchronous. No stagger to exploit. Added Python overhead reduced margin. **Wait-for-all pattern is optimal with shared clock.**

### int.from_bytes Optimization (KEPT)
Replaced manual bit-shifting with `int.from_bytes()` in `parse_raw()`. Reduced ~2800 Python bytecodes per sample to ~900. Seven consecutive clean runs (~535k samples total, zero corruption).

### System Config (Persistent)
- `isolcpus=3` in cmdline.txt
- CPU governor -> performance (systemd service)
- IRQ affinity off core 3 (systemd service)
- I2C bit-banged on bus 6 (GPIO22/23) -- hardware I2C pins used by SPI3

---

## 2026-02-18: C++ Acquisition Engine (Phase 1)

Rewrote Python acquisition loop in C++20. Key design:
- Direct SPI ioctl (no spidev Python wrapper)
- Eventfd-triggered bus workers
- SPSC ring buffer for CSV
- SCHED_FIFO 50 on isolated core 3
- mlockall, pre-allocated everything
- Built natively on Pi: GCC 14.2, `-mcpu=cortex-a72 -O2 -fno-exceptions -fno-rtti`, LTO

**First run validation:** 23.3 min, 349k samples, zero corruption, 250.0 Hz locked, dt mean 4.000ms.

### TCP Streaming (Phase 2)
Added TCP streaming as parallel output alongside CSV. Separate SPSC ring (2048 slots), accept + stream threads on core 1. TCP_NODELAY, SO_SNDBUF=262144, TCP keepalive.

---

## 2026-02-19: Tiered Recovery Init

### Problem
Dead ports not excluded from acquisition pipeline caused 100% corruption.
Old `recover_port()` never sent RESET, couldn't clear power-on latch.

### Fix
- Tiered recovery: Tier 1 (RDATAC cycle, 8 attempts) -> Tier 2 (RESET + reconfig, 5 attempts) -> Tier 3 (full re-init, 2 attempts)
- 7/7 hard gate (exit code 2 = power cycle needed)
- `--min-ports N` escape hatch
- Dead port exclusion from DRDY poller, bus workers, CSV, streaming
- Warmup increased 100 -> 250 samples
- Per-port DeviceConfig (not global)

---

## 2026-02-21: Noise Characterization + Electrode Testing

### Key Findings
- Internal reference noise floor: 0.150 uV RMS (shorted inputs, gain=24)
- External reference: non-functional (576x worse shorted, all-zeros floating)
- New resin formula (3g Ag + 2.32g Resin): 60 dB SNR at -60dB signal
- Silver electrodes: not competitive (21 dB worse SNR)
- Battery B consistently superior to Battery A (+3 dB SNR even cold)
- Chain depth negligible impact on noise (Port1 Dev9 matches Port5 Dev4)
- Fresh resin: 100% pass rate. Day-old resin: 12.5% pass rate (silver settling)

See `testing/electrode-testing.md` for full results.

---

## 2026-02-26: 3-Layer Electrode Characterization

17 three-layer electrodes tested. Results:
- 7 achieved A* grade (SNR >= 50 dB, 60Hz < 0.15 mV)
- Best: 3L-08 (57.5 dB SNR, 8/8 channels, 1.6 dB spread)
- Reference baseline: 55.4 dB SNR on Battery B

See `testing/electrode-testing.md` for full results.

---

## 2026-03-04: Signal Quality Analysis

Gain comparison (8x/12x/24x) and cold-start analysis on 320 channels:
- 12x optimal for channel yield (66.6% usable)
- 24x viable with good connectors after 5-10 min settling (76% usable)
- Wiring/connector quality is THE dominant variable for channel yield
- Power-on order does not matter, no lower-gain warmup needed

See `info/signal-quality.md` for full analysis.

---

## 2026-03-06: SPI3 Failure + Hardware Diagnosis

### Problem
SPI3 (Port3 + Port4) failed: register reads returned zeros, init loop never completed.

### Investigation
- GPIO24 (SPI3 CS1) stuck LOW even when driven HIGH via `/dev/mem`
- Changed to `spi3-1cs` as workaround (lost Port4)
- Write-once init (`--write-once`) got 6/7 ports working
- Extensive software debugging: DMA overlay changes, driver unbind/rebind, speed reduction -- all ineffective

### Root Cause
**A single bad ADS1299 with an internal short** pulled SPI3 bus lines low. GPIO24 reading 0 after GPSET was the diagnostic: fault is external to the Pi. Removing the bad amplifier restored SPI3.

### Lesson
Shorted ADS1299 can take down an entire SPI bus. Diagnostic: write GPSET via `/dev/mem`, if pin level stays 0, fault is external. Extensive software debugging was unnecessary.

### Debug Commands Reference
See `info/spi3-debug-commands.md` for the raw debug commands used during this session.

---

## 2026-03-07: EMI Reduction + Artifact Fixes

### GPU EMI (70 Hz artifact)
Disabled GPU output, Bluetooth, audio, camera, TV out in config.txt. 4x reduction in 70 Hz artifact on clean channels, 2.3x aggregate. 60 Hz pickup reduced 3.3x.

### DRDY Poll Rate Lock (71.53 Hz alias)
Fixed-interval polling at 2500 Hz (400us grid) via `clock_nanosleep(TIMER_ABSTIME)`. Alias moves to DC, invisible in AC-coupled EEG.

### 25 Hz Artifact (TCP batch boundary)
Removed LZ4 compression, set BATCH_SIZE=1. Artifact eliminated.

### BIAS Drive Support
Added `--bias-port N` CLI flag. A/B test showed minimal effect on 70 Hz with current electrode geometry (8 distributed BIAS electrodes on one port don't effectively cancel CM for distant ports).

### Periodic Spike Investigation
User-reported spikes at t=21s and t=66s in 10-min recording. Six independent detection methods found NO periodic pattern. The events are random electrode-contact artifacts. Zero sample gaps, zero corruption across 165,010 samples.

---

## Init Attempt Changes Log (Historical)

### Attempt 1: Aggressive SPI Changes (FAILED)
Added flush_spi calls, ID verification loop, pre-config SDATAC+flush. All destabilized SPI. System failed after 5 retries.

### Attempt 2: Minimal Timing Changes (SUCCESS)
System retry STOP delay 100ms -> 500ms, 50ms delay between port configs. Result: clean initialization, 0.004% corruption.

### Interpolation (Python Only)
Implemented last-valid-value substitution for corrupt samples in the Python server. Tracks `last_valid_samples` per device/channel. Client receives clean data stream. Not needed in C++ (zero corruption).
