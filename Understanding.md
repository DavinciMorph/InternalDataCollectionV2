# ADS1299 EEG Acquisition System -- Engineering Reference

**Last Updated:** 2026-03-07
**Branch:** `tiered-recovery-init-redesign`

---

## 1. System Architecture

### Hardware

- **ADC:** TI ADS1299 (24-bit, 8-channel delta-sigma)
- **Topology:** 7 SPI ports, 42 devices total in daisy chains, 336 channels @ 250 Hz
- **Platform:** Raspberry Pi CM4 (BCM2711 / Cortex-A72), IP 192.168.1.175

**Port layout:**

| Port | SPI Bus | CS | Devices | Channels | DMA |
|------|---------|----|---------|---------:|-----|
| Port1 | SPI0 | CS0 | 8 | 64 | Legacy DMA |
| Port2 | SPI0 | CS1 | 7 | 56 | Legacy DMA |
| Port3 | SPI3 | CS0 | 5 | 40 | Legacy DMA (DREQs 16/18) |
| Port4 | SPI3 | CS1 | 5 | 40 | Legacy DMA (DREQs 16/18) |
| Port5 | SPI4 | CS0 | 5 | 40 | Legacy DMA (DREQs 19/20) |
| Port6 | SPI4 | CS1 | 5 | 40 | Legacy DMA (DREQs 19/20) |
| Port7 | SPI5 | CS0 | 7 | 56 | DMA4 (DREQs 21/22) |

**I/O control:** DRDY via I2C GPIO expander TCA9534 (0x20), START via TCA9534 (0x21).
I2C is bit-banged (`i2c-gpio` on bus 6, GPIO22/23) because hardware I2C pins are used by SPI3.
V2 board has RESET/PWDN tied HIGH -- no hardware reset available, software RESET (0x06) only.

### Software

C++20 acquisition engine, built natively on Pi with GCC 14.2.
Binary: `~/ads1299-cpp/build/ads1299_acquire` (79 KB), requires `sudo`.

**Key design principles:**
- Direct SPI via `ioctl(SPI_IOC_MESSAGE)` -- no spidev Python overhead
- One persistent thread per SPI bus (eventfd-triggered bus workers)
- SPSC lock-free ring buffers for sample handoff (CSV + streaming)
- Zero heap allocation in hot loop -- all buffers pre-allocated
- Acquisition thread pinned to isolated core 3, SCHED_FIFO 50, `mlockall`

**Thread / core allocation:**

| Core | RT Thread (FIFO 49-50) | Non-RT |
|------|------------------------|--------|
| 0 | SPI0 worker (Port1+2) | SPI0 DMA IRQs |
| 1 | SPI3 worker (Port3+4) | CSV writer, accept, stream, stats threads, IRQs |
| 2 (isolated) | SPI4 + SPI5 workers | (none) |
| 3 (isolated) | Acquisition engine (FIFO 50) | (none) |

### Register Defaults

From `Cpp Implementation/include/ads1299/registers.hpp` (`DeviceConfig` struct):

| Register | Value | Meaning |
|----------|-------|---------|
| CONFIG1 | 0x96 | Daisy-chain mode, 250 SPS, external clock |
| CONFIG2 | 0xC0 | Normal mode, test signal OFF |
| CONFIG3 | 0xE0 | Internal BIASREF, bias buffer off |
| CONFIG4 | 0x00 | Continuous conversion, lead-off comp off |
| CH1-8SET | 0x60 | Normal electrode input, gain = 24 |
| MISC1 | 0x20 | SRB1 closed (reference electrode connected) |
| SPI | Mode 1, 6 MHz | CPOL=0, CPHA=1 |

### CLI Arguments

```
--ports bus,dev,name,num_daisy ...   Custom port config (default: 7 ports, 42 devices)
--full-csv                           Enable server-side CSV (default: off, client-side)
--duration SEC                       Auto-stop after SEC seconds
--min-ports N                        Minimum ports required (default: all = 7/7 gate)
--host ADDR --port PORT              TCP streaming (default: 0.0.0.0:8888)
--config3 0xHH                       Override CONFIG3 register
--chnset 0xHH                        Override all CHnSET registers
--sps RATE                           Sample rate (250/500/1000/2000/4000/8000/16000)
--test-signal                        Internal test signal (CONFIG2=0xD0, CHnSET=0x05)
--bias-port N                        Enable BIAS drive on port N (1-7)
```

### TCP Streaming Protocol

Raw uncompressed binary over TCP (LZ4 removed -- see Section 4).
Each sample sent individually (BATCH_SIZE=1).
Bandwidth: ~238 KB/s at 336 channels @ 250 Hz.

Wire format: metadata JSON (once per connection) + repeated `[u32 payload_size][u32 sample_count=1][raw sample data]`.
Metadata: `"format":"binary_raw","compression":"none"`.

### Pi Kernel / Boot Config

All persistent across reboots:
- `isolcpus=2,3` + `irqaffinity=0,1` + `nohz_full=2,3` + `rcu_nocbs=2,3` in `/boot/firmware/cmdline.txt`
- CPU governor locked to `performance` via systemd service
- IRQ affinity off cores 2-3 via systemd service
- WiFi scanning disabled via `wifi-noscan.service` (`wpa_cli scan_interval 9999`)
- EMI reduction: GPU display disabled (`max_framebuffers=0`, `hdmi_blanking=2`), HDMI audio off (`vc4-kms-v3d,noaudio`), Bluetooth off, audio off, camera/display probing off, composite video off
- Volatile journald (RAM-only, no SD writes from logging)
- VM writeback: `dirty_ratio=40`, `dirty_background_ratio=20`, `dirty_expire_centisecs=3000`

---

## 2. Key Root Causes Discovered

### 2.1 printf() in RT Hot Loop

`print_stats()` was called from the SCHED_FIFO acquisition thread on core 3. When SSH transport stalled, `write(STDOUT_FILENO)` blocked indefinitely, starving all bus workers. Combined with glibc stdout mutex priority inversion (non-RT streaming threads holding the lock), this caused multi-second stalls and burst corruption across all SPI buses.

**Fix:** Stats printing moved to a separate non-RT thread on core 1. Engine sets an atomic flag; stats thread polls it. Zero syscalls on the RT core.

### 2.2 AXI Bus Saturation from SD Card DMA

Server-side CSV writing (~825 KB/s) caused periodic EMMC2 DMA writebacks that saturated the shared AXI bus fabric. All 4 SPI DMA controllers stall simultaneously -- no CPU-level event can produce this pattern, only shared hardware bus contention.

**Fix:** CSV writing moved to client side (default: `--no-csv`). Server-side CSV available via `--full-csv` for special cases. Zero EMMC2 traffic during acquisition. Result: 20+ minute runs with zero corruption.

### 2.3 WiFi Background Scanning

WiFi scans on the Pi triggered DMA bursts on a ~23.4-second period, causing SPI corruption. Manifested as periodic corruption at exact sample intervals (~5850 samples apart).

**Fix:** `wpa_cli scan_interval 9999` via persistent systemd service.

### 2.4 Final Re-Sync Breaking Recovered Ports

The original Python init did STOP-all / flush / RDATAC / restart-all after warmup, undoing individual port recoveries. Since all devices share an external clock, DRDY is already synchronous -- re-sync is unnecessary.

**Fix:** Removed global re-sync. Per-port recovery operates independently.

### 2.5 SPI Bit-Shift Corruption Mechanism

When a stall causes the total DRDY-poll + SPI-transfer time to exceed ~3.9ms, the next DRDY edge overwrites the ADS1299 shift register mid-transfer. Status bytes pass (clocked out first in ~4 us) but channel data is corrupted by exactly 2^N, where N is the number of extra SCLK bit positions. Deeper devices in the daisy chain are always hit worse. CS0 ports (read first) are clean; CS1 ports (read second) are always the victims.

### 2.6 Power-On Latch Failures

Some ADS1299 devices fail to latch their digital core state at power-on. Simple RDATAC cycling never fixes this -- a software RESET (0x06 command) is required to clear the corrupted state.

**Fix:** Tier 2 recovery sends software RESET + full register reconfiguration (see Section 3).

### 2.7 DRDY Polling Rate Aliasing

Variable-rate `usleep(200)` polling produced a ~2857 Hz mean poll rate, aliasing I2C bit-bang EMI into the EEG band at 71.53 Hz.

**Fix:** `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` on a fixed 400 us grid (2500 Hz). Since 2500 = 10 x 250, the alias lands at 0 Hz (DC), invisible in AC-coupled EEG. Overrun self-correction skips grid points forward if I2C read exceeds 400 us.

### 2.8 TCP Batch Boundary Artifact (25 Hz)

`BATCH_SIZE=10` in TCP streaming created periodic batch boundaries that coupled as a 25.000 Hz artifact (Fs/10) with harmonics at 50, 75, 100, 125 Hz. Within-device coherence r=0.97, cross-port coherence 0.94. Artifact was locked to array-index position within the 10-sample TCP batch.

**Fix:** LZ4 compression removed entirely, BATCH_SIZE set to 1 (per-sample TCP sends). Raw uncompressed TCP at ~227 KB/s is trivial for WiFi.

### 2.9 GPU EMI at ~70 Hz

Pi GPU display pipeline running at ~70 Hz refresh rate coupled EMI into electrode traces. Confirmed by test-signal mode (70 Hz absent with internal test signal, present with external electrodes). Cross-port coherence 0.4-0.9 with position-dependent phase.

**Fix:** GPU output fully disabled (`max_framebuffers=0`, `hdmi_blanking=2`). 4x reduction on clean channels, 2.3x aggregate. Additional peripherals (BT, audio, camera, TV out) also disabled.

### 2.10 Shorted ADS1299 Taking Down SPI Bus

A single ADS1299 with an internal short pulled SPI3 bus lines low, taking down Port3+Port4 entirely. Software debugging found nothing wrong until GPIO pin state was checked via `/dev/mem` -- CS1 read 0 even when GPSET drove it HIGH.

**Diagnostic:** If GPSET write has no effect on pin level, the fault is external to the Pi.

---

## 3. Init and Recovery

### Tiered Recovery System

Init splits into phases: Phase 1 (register writes, always succeeds if hardware present) and Phase 2 (RDATAC + START, has a ~10% per-port timing race failure rate). Failed ports enter tiered recovery:

| Tier | Method | Time/attempt | Attempts | Targets |
|------|--------|-------------|----------|---------|
| 1 | RDATAC cycle (STOP/SDATAC/RDATAC/START) | ~200 ms | 8/port | RDATAC timing race (~90% of failures) |
| 2 | Software RESET + full register reconfig | ~500 ms | 5/port | Power-on latch failures |
| 3 | Full re-init ALL ports from scratch | ~30 s | 2 rounds | Cross-port state interference |

Between Tier 3 attempts, a quick Tier 1 pass runs on still-failing ports.

**Typical recovery time:** 3-5 seconds (most ports recover in Tier 1, attempts 1-3).
**Worst case:** ~89 seconds across all tiers before declaring ports dead.

### 7/7 Hard Gate

After all tiers, if `healthy_ports < min_ports`, the system exits with code 2 and prints "POWER CYCLE REQUIRED." Default requires all configured ports. `--min-ports N` provides an escape hatch for degraded-mode testing. Dead ports are excluded from all downstream components (DRDY poller, bus workers, CSV writer, streaming server, engine).

### BIAS Drive Support

`--bias-port N` enables BIAS/DRL on a specified port:
- Port N: CONFIG3=0xF0 (bias amp ON), BIAS_SENSP=0x7F (ch1-7 to bias), BIAS_SENSN=0x7F, CH8SET=0x06 (BIAS_DRP output)
- All other ports: unchanged (CONFIG3=0xE0)
- Per-port config array ensures recovery uses the correct config per port

A/B testing showed minimal effect with the current electrode geometry (8 distributed BIAS electrodes on one port). Proper DRL requires a single central electrode (earlobe/mastoid).

---

## 4. Signal Quality and Artifacts

### Resolved

| Artifact | Root Cause | Fix | Status |
|----------|-----------|-----|--------|
| Periodic corruption (~23.4s) | WiFi background scanning DMA | `wpa_cli scan_interval 9999` | Fixed |
| Burst corruption (t=500s+) | SD card EMMC2 DMA saturating AXI bus | Client-side CSV, `--no-csv` default | Fixed |
| Burst corruption on SSH stall | printf() blocking RT thread on stdout | Stats moved to non-RT thread | Fixed |
| 25.000 Hz + harmonics | TCP BATCH_SIZE=10 batch boundaries | BATCH_SIZE=1, LZ4 removed | Fixed |
| ~70 Hz PSD peak | Pi GPU display pipeline EMI | GPU + peripherals disabled | 4x reduced |
| 71.53 Hz jitter peak | Variable DRDY poll rate aliasing I2C EMI | Fixed 2500 Hz poll grid | Fixed |

### Residual / Unresolved

- **~70 Hz residual (~2.7 uV^2/Hz on cleanest channels):** Reduced but not eliminated after GPU disable. May be remaining Pi peripherals or environmental 60 Hz AM sidebands.
- **47-49 channels permanently railed:** Bad electrode contact, hardware/connector issue.
- **Port5 connector defect:** Extreme amplitudes during any transient event.
- **Electrode impedance drift:** RMS grows 3.3x over 11 minutes in dry-electrode recordings. Causes filter artifacts (1 Hz HPF converts slow DC drift into spike-like transients). Not a firmware issue -- fix with electrode gel and skin prep.
- **TCA9534 INT pin floating:** DRDY interrupt not viable without hardware change; polling required.

### Investigation Conclusions

**"Periodic spikes" at t=21s, t=66s (reported in 10-min recording):** Permutation tests non-significant (p=0.32, p=0.80, p=1.0). Folding SNR monotonically increasing (r=0.957), proving no periodic signal exists. Root cause: electrode impedance drift + 1 Hz HPF filter artifact. Zero sample gaps, zero corruption across 165,010 samples. No firmware fix needed.

---

## 5. Development Timeline

| Date | Milestone |
|------|-----------|
| 2026-02-17 | Runs 1-6: Python acquisition testing. Discovered re-sync breaks recovered ports, client stuck-detector perf bug, periodic ~23.4s corruption from WiFi. |
| 2026-02-17 | Init optimization Tests 2-12: reduced init delays from ~60s to ~2s. Accepted 6 optimizations, reverted 3 (Test 9, 11, 12). |
| 2026-02-18 | C++ engine Phase 1 (acquisition + CSV): zero corruption over 349k samples / 23.3 min. |
| 2026-02-18 | C++ engine Phase 2 (TCP streaming): zero drops, 250 Hz client delivery, clean disconnect handling. |
| 2026-02-18 | Phase A (init reliability): per-device validation, per-port re-sync, WiFi scan disable. |
| 2026-02-18 | Phase B/B2 (thread isolation): identified printf RT stall + AXI saturation root causes. Moved CSV to client side. Clean 20-min run (300k samples, zero corruption). |
| 2026-02-19 | Tiered recovery redesign: Tier 1/2/3, 7/7 hard gate, --min-ports, dead port exclusion. |
| 2026-02-19 | Register switch from test mode to normal acquisition (gain=24, normal electrode input). |
| 2026-03-06 | SPI3 debugging: discovered shorted ADS1299 pulling bus lines low. |
| 2026-03-07 | DRDY 2500 Hz poll lock, BIAS support (--bias-port), EMI reduction config, 25 Hz artifact fix (BATCH_SIZE=1, LZ4 removed), 70 Hz GPU EMI investigation. |

---

## 6. Validation Results

### Best Runs

**C++ Phase 2 (20.1 min, no server CSV):**
- 300,750 samples, 328 channels, 98.6M data points
- 250.0 Hz locked, zero corruption, zero gaps, zero jitter events
- dt: min 3.776 ms, max 4.221 ms
- Zero CSV ring drops, zero stream ring drops

**C++ Phase 1 (23.3 min, server CSV):**
- 349,369 samples, 224 channels
- 250.0 Hz locked, zero corruption, zero gaps
- dt: mean 4.000 ms, min 1.241 ms, max 4.660 ms

### Python vs C++ Comparison

| Metric | Python | C++ |
|--------|--------|-----|
| Max dt | 8.2 ms (caused corruption) | 4.22 ms |
| Typical cycle time | 2.5-3.0 ms | 0.35-0.65 ms |
| Headroom per cycle | 1.0-1.5 ms | > 3.0 ms |
| Corruption (20+ min) | 1+ events | 0 |
| GC/GIL stalls | Yes | N/A |

---

## 7. Known Issues / Open Items

1. **Health check validates status bytes only:** The 0xC0 status nibble check misses devices with valid status but garbage channel data. Channel-level validation not implemented.
2. **TCA9534 INT pin floating:** Prevents interrupt-driven DRDY; polling required.
3. **Residual ~70 Hz artifact:** Reduced 4x by GPU disable but not eliminated (~2.7 uV^2/Hz on cleanest channels, essentially at noise floor).

---

## 8. References

Detailed documentation lives in `info/`:

| File | Contents |
|------|----------|
| [`info/init-reliability.md`](info/init-reliability.md) | Init architecture, recovery tiers, health check details |
| [`info/corruption-and-timing.md`](info/corruption-and-timing.md) | Corruption mechanisms, bit-shift analysis, timing budgets |
| [`info/artifact-investigation.md`](info/artifact-investigation.md) | 25 Hz, 70 Hz, and periodic spike investigations |
| [`info/signal-quality.md`](info/signal-quality.md) | Signal quality analysis and electrode considerations |
| [`info/session-notes.md`](info/session-notes.md) | Chronological session logs and raw test data |
| [`info/ADS1299-reference.md`](info/ADS1299-reference.md) | ADS1299 datasheet reference and register map |
| [`info/multi-scale-clustered-eeg.md`](info/multi-scale-clustered-eeg.md) | Multi-scale EEG architecture (macro + micro) |
| [`pi/SPI-DMA-SETUP.md`](pi/SPI-DMA-SETUP.md) | DMA setup for SPI0/3/4/5 on BCM2711 |

**Key source files:**

| File | Role |
|------|------|
| `Cpp Implementation/src/main.cpp` | Entry point, arg parsing, tiered recovery, BIAS config |
| `Cpp Implementation/src/acquisition/engine.cpp` | RT hot loop (core 3, SCHED_FIFO 50) |
| `Cpp Implementation/src/acquisition/drdy_poller.cpp` | 2500 Hz fixed-grid DRDY polling |
| `Cpp Implementation/src/acquisition/bus_worker.cpp` | Per-SPI-bus worker threads |
| `Cpp Implementation/src/ads1299/controller.cpp` | Device init, tier 1/2 recovery |
| `Cpp Implementation/src/streaming/server.cpp` | TCP accept + streaming threads |
| `Cpp Implementation/src/streaming/protocol.cpp` | Wire protocol (raw binary, per-sample) |
| `Cpp Implementation/include/ads1299/registers.hpp` | Register defaults and constants |
| `client/simpleviz.py` | PyQt5 real-time EEG visualizer |
| `deploy_and_build.py` | SCP + remote build utility for Pi deployment |
