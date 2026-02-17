# Session Notes — 2026-02-15: SPI DMA Enablement & Acquisition Debugging

## 1. SPI DMA Enablement (COMPLETED)

### Problem
SPI3, SPI4, SPI5 on the BCM2711 (Raspberry Pi 4) were running in PIO mode. CPU preemption during PIO transfers caused SPI clock stalls. If a stall lasted past the next ADS1299 DRDY boundary (4ms at 250 SPS), the device overwrote its shift register, corrupting data for daisy-chained devices. This caused rare but consistent corruption on ports 3-7. SPI0 (ports 1-2) already had DMA and showed zero corruption.

### Solution Implemented

All three SPI buses now have DMA. The final working configuration:

| SPI Bus | Address    | DREQ TX | DREQ RX | DMA Controller | Notes |
|---------|-----------|---------|---------|----------------|-------|
| SPI0    | fe204000  | 6       | 7       | Legacy (`&dma`) | Already working |
| SPI3    | fe204600  | 16      | 18      | Legacy (`&dma`) | Overlay only, DREQs mislabeled as "SPI1" in datasheet |
| SPI4    | fe204800  | 19      | 20      | Legacy (`&dma`) | Overlay + PACTL_CS mux (bit 24) |
| SPI5    | fe204a00  | 21      | 22      | DMA4 (`&dma40`) | Overlay + PACTL_CS mux (bit 25) + HDMI audio disabled |

### DMA Channel Allocation (confirmed working)

```
dma0 (fe007000.dma-controller) — Legacy, 8 channels:
  dma0chan0  |  fe300000.mmcnr:rx-tx      (SD card)
  dma0chan1  |  fe204000.spi:tx            (SPI0 TX)
  dma0chan2  |  fe204000.spi:rx            (SPI0 RX)
  dma0chan3  |  fe204600.spi:tx            (SPI3 TX)
  dma0chan4  |  fe204600.spi:rx            (SPI3 RX)
  dma0chan5  |  fe204800.spi:tx            (SPI4 TX)
  dma0chan6  |  fe204800.spi:rx            (SPI4 RX)

dma1 (fe007b00.dma) — DMA4, 2 channels:
  dma1chan0  |  fe204a00.spi:tx            (SPI5 TX)
  dma1chan1  |  fe204a00.spi:rx            (SPI5 RX)
```

### Problems Encountered & Solved During DMA Setup

**SPI3 (straightforward):** DREQs 16/18 are dedicated (not shared). Overlay-only fix. Worked on first attempt.

**SPI4 (needed PACTL_CS mux):** DREQs 19/20 are shared with UART3. Required setting PACTL_CS register bit 24 (DMA_CNTRL_MUX_0) via devmem2 at boot. Used a systemd oneshot service (`spi45-dma-mux.service`) to run before SPI driver probes.

**SPI5 (most complex — three problems):**

1. **DREQs shared with UART5:** DREQs 21/22 needed PACTL_CS bit 25 (DMA_CNTRL_MUX_1). Combined with SPI4's bit 24, total MUX_BITS = `0x03000000`.

2. **Legacy DMA channel exhaustion:** The legacy DMA controller only has 8 usable channels. With SPI0 (2) + SPI3 (2) + SPI4 (2) + MMC (1) = 7 channels used, only 1 remained. SPI5 needs 2 (TX + RX). SPI5 TX would allocate the last channel, then RX would fail with `-ENODEV: no rx-dma configuration found`.

3. **Expanding legacy DMA mask broke WiFi:** Attempted to add GPU-reserved channels 1 and 3 to `brcm,dma-channel-mask` (0x07F5 → 0x07FF). This caused WiFi to fail on boot. The GPU uses these channels internally. Had to pull up a keyboard and run `sudo sed -i '/dma-expand/d' /boot/firmware/config.txt` to recover.

4. **DMA4 mask expansion ignored by firmware:** Created an overlay to expand DMA4's mask (0x3000 → 0x7800). The firmware overwrites DMA channel masks after overlays are applied, so the change had no effect.

5. **Final solution — disable HDMI audio:** Changed `dtoverlay=vc4-kms-v3d` to `dtoverlay=vc4-kms-v3d,noaudio` in config.txt. This freed the 2 DMA4 channels (previously used by HDMI audio-rx) for SPI5. The SPI5 overlay references `&dma40` instead of `&dma`.

### Device Tree Labels (confirmed on this Pi)

```
dma   → /soc/dma-controller@7e007000     (legacy, mask 0x07F5)
dma40 → /scb/dma@7e007b00                (DMA4, mask 0x3000)
```

Both have `#dma-cells = <1>`. Phandle for legacy dma = 0x0c, phandle for dma40 = 0x2e.

### PACTL_CS Register

- ARM physical address: `0xfe204e00`
- Bus address: `0x7e204e00`
- MUX_BITS set: `0x03000000` (bit 24 = MUX_0 for SPI4, bit 25 = MUX_1 for SPI5)
- Register value at boot (before mux): `0x00000000`
- Register value after mux: `0x03000000`
- Bit 23 (MUX_2) was tested but not needed for SPI4/SPI5

### devmem2

Not in Raspberry Pi OS default repos. Built from source:
```bash
wget https://raw.githubusercontent.com/VCTLabs/devmem2/master/devmem2.c
gcc -o devmem2 devmem2.c
sudo cp devmem2 /usr/local/bin/
```

### Files Created/Modified for DMA

| File | Status | Purpose |
|------|--------|---------|
| `spi3-dma-overlay.dts` | Existed, unchanged | DREQs 16/18, `&dma` |
| `spi4-dma-overlay.dts` | Updated DREQs 10/11 → 19/20 | DREQs 19/20, `&dma` |
| `spi5-dma-overlay.dts` | Created, then updated to use `&dma40` | DREQs 21/22, `&dma40` |
| `spi45-dma-mux.sh` | Updated MUX_BITS 0x01000000 → 0x03000000 | Sets PACTL_CS at boot |
| `spi45-dma-mux.service` | Created | Systemd unit for mux script |
| `dma-expand-overlay.dts` | Created (DO NOT USE — breaks WiFi) | Legacy mask expansion |
| `dma40-expand-overlay.dts` | Created (no effect — firmware overrides) | DMA4 mask expansion |
| `verify-spi3-dma.sh` | Existed, unchanged | Verification script |
| `SPI-DMA-SETUP.md` | Created | Full setup documentation |

### config.txt Changes (on Pi at /boot/firmware/config.txt)

```ini
dtoverlay=vc4-kms-v3d,noaudio    # was: dtoverlay=vc4-kms-v3d

dtoverlay=spi3-2cs
dtoverlay=spi3-dma

dtoverlay=spi4-2cs
dtoverlay=spi4-dma

dtoverlay=spi5-1cs
dtoverlay=spi5-dma
```

Also has `dtoverlay=dma40-expand` which has no effect but is harmless. Could be removed.

### Pi Access

- IP: `192.168.1.99`
- Username: `morph` (NOT `MorphV2` — case sensitive)
- Password: `morph`
- Files were scp'd to `~/` (home directory)
- Controller runs from `~/Desktop/DataCollection/`

### Tradeoffs Accepted

- UART3 DMA disabled (not used)
- UART5 DMA disabled (not used)
- HDMI audio disabled (not needed for EEG acquisition)
- GPU-reserved DMA channels (1, 3) cannot be reclaimed without breaking WiFi

---

## 2. CSV Write Stall Fix (COMPLETED)

### Problem
After DMA was working, a new run showed 58 corrupted samples with cascade failure starting at ~170 seconds. The corruption was a DIFFERENT type than the PIO corruption — large magnitude values (millions), byte-aligned patterns (0xNN0000), all-devices-same-value events, and partial frame patterns (4094, 63, 15, 2047, 3).

Root cause: **CSV writes in the acquisition thread's hot loop** caused SD card I/O stalls up to 1564ms when the kernel's page cache flushed dirty pages. Combined with Python's garbage collector freezing all threads for gen2 collection sweeps.

### Evidence

- Corruption density by time: sparse for 170s, then 73 corrupted values in a single 10-second window
- Top timestamp gap: 1564ms (391x normal 4ms interval)
- All large gaps concentrated in the 177-192s range
- Sample rate dropped from 250 Hz due to missed DRDY windows

### Fix Applied to Controller.py

1. **Async CSV writer thread:** Replaced direct `csv_writer.writerow()` in the hot loop with `deque.append()` (lock-free, nanoseconds). A separate thread drains the deque and writes to disk with a 1MB buffer (`buffering=1048576`).

2. **GC disabled during acquisition:** `gc.collect()` runs once before the loop to clean up setup allocations, then `gc.disable()` prevents pauses during the time-critical loop. GC is re-enabled in the `finally` block on shutdown.

### Code Changes (Controller.py)

- Added imports: `gc`, `collections.deque`
- Lines ~1305-1342: Replaced CSV file open + writer with `csv_queue = deque(maxlen=50000)` and a `csv_writer_thread()` that drains it
- Lines ~1416-1421: Replaced `csv_writer.writerow(row)` with `csv_queue.append(row)`
- Lines ~1359-1362: Added `gc.collect()` then `gc.disable()` before the hot loop
- Lines ~1431-1435: Added `gc.enable()`, `gc.collect()`, `csv_stop.set()`, `csv_thread.join()` in finally block

### Results After Fix

| Metric | Before fix | After fix |
|--------|-----------|-----------|
| Max timestamp jitter | 1,564 ms (391x) | 8 ms (2x) |
| Corrupt samples | 58 / 46,660 | 1 / 30,227 |
| Corrupt values | 128 | 1 |
| Jitter events (>2x) | 193 | 1 |
| Corruption rate | 0.0098% | 0.0001% |

### check_corruption.py

Created a fast analysis tool for CSV corruption detection. Usage:
```bash
python check_corruption.py                           # default: all_ports_ch1_data.csv
python check_corruption.py recording1.csv            # specific file
python check_corruption.py run1.csv run2.csv         # multiple files
```

Detects: single-sample spikes, rail/saturation values, bit-shift patterns, startup artifacts, timestamp irregularities, missing samples. Runs in ~3 seconds for 177K samples (vs ~8 minutes for the full agent analysis).

---

## 3. Intermittent Zeros on Init (OPEN ISSUE — NOT YET FIXED)

### Problem

On some runs, certain ports return all zeros after initialization even though DRDY is toggling. The restart/re-init logic sometimes recovers the port, sometimes doesn't. In the last observed failure:
- Port4 and Port6 returned 100% zeros
- Both failed restart AND aggressive re-init
- The system continued with those ports outputting zeros

This is a **pre-existing issue** unrelated to the DMA or CSV fixes.

### Hardware Context

- 7 SPI ports, each with 4 daisy-chained ADS1299 devices
- V2 board: RESET/PWDN pins are tied HIGH (no hardware reset possible)
- Only software reset via SPI command is available
- In this daisy chain, **all devices share MOSI** — commands reach ALL 4 devices simultaneously
- There is no way to address a single device in the chain; all register writes affect all devices identically
- All devices are configured with identical settings

### What the Init Sequence Does

1. **5 software resets** with 1.4s settling each (7 seconds per port)
2. **SDATAC** sent 6 times with 20ms gaps, verified via register read/write test
3. **Register configuration:** CONFIG3 → 500ms reference settle → CONFIG1, CONFIG2, CH1-8, MISC1, CONFIG4. Each register written and verified (read-back) up to 100 times.
4. **Synchronized START:** STOP all → SDATAC all → RDATAC all (twice) → 750ms settle → Assert all START pins simultaneously → 500ms sync wait
5. **DRDY verification** per port (100ms timeout)
6. **Zero check:** Read 5 samples per port. If >=3 zeros, restart that port (up to 3 attempts)
7. **Warmup:** Discard 500 samples. If any port accumulates 20+ consecutive zeros, restart it once
8. **Aggressive re-init** for ports with >10% warmup corruption: cycles STOP/SDATAC/RDATAC/START with longer delays (NOT a full register re-write despite the name)
9. **Final re-sync:** Stop all → 200ms → Restart all → 500ms → Discard 100 samples

### Init Code Issues Identified

1. **"Aggressive re-init" is NOT aggressive (lines ~1018-1033):** It only cycles STOP/SDATAC/RDATAC/START with longer delays. It does NOT re-write registers, does NOT re-send RESET. It's the same as a regular restart with more settling time. Name is misleading.

2. **Final re-sync does NOT re-send RDATAC (lines ~1098-1113):** After warmup, the code stops all devices and restarts them without sending RDATAC first. If a device dropped out of RDATAC during warmup, the final re-sync won't fix it.

3. **SPI flush is too short (line ~385):** `flush_spi()` sends 32 zero bytes, but a full data frame for 4 devices is 108 bytes (27 bytes × 4 devices). Partial flush can leave shift registers misaligned.

4. **Status byte check only validates device 0 (line ~568):** Restart verification checks `(status[0] & 0xF0) == 0xC0` which is only device 0's status. Devices 1-3 status bytes are in the rest of the 108-byte frame but are not checked.

5. **Single restart opportunity during warmup (line ~963):** Each port gets at most one restart attempt during the 500-sample warmup phase, even if the restart failed.

6. **No check for all-zero CHANNEL DATA (lines ~950-960):** Warmup corruption tracking only looks at the status byte (`status[0]`). A port could have valid status (0xC0) but all-zero channel data and not be flagged.

7. **No hardware reset available:** RESET/PWDN is tied HIGH on V2 board. If a device gets into a state that software RESET can't fix, the only recovery is a full power cycle.

### Possible Root Causes for Intermittent Zeros

Since commands DO reach all devices (shared MOSI), the problem is NOT about commands failing to propagate. Possible causes:

1. **Device fails to complete software reset:** With RESET/PWDN tied HIGH, if a device's internal state machine gets stuck, software RESET may not recover it. Only power cycle would help.

2. **Internal reference buffer not ready:** CONFIG3 reference buffer needs settling. If START is asserted before the reference is stable on all devices, some may output zeros. The code waits 500ms after CONFIG3, but this might not be enough in all conditions.

3. **SPI signal integrity on long daisy chains:** 4 devices with DOUT→DIN cascading means the last device's data travels through 3 intermediate devices. Marginal signal quality could cause intermittent failures on some chains.

4. **Race condition between RDATAC and START:** If the timing between entering RDATAC and asserting START is marginal, some devices might not be ready to output data.

5. **Stale data in shift registers after flush:** The 32-byte flush is less than one full frame (108 bytes). Residual data could cause misalignment.

### What Has NOT Been Tried Yet

- Making `flush_spi()` send a full frame length (108 bytes) instead of 32
- Adding RDATAC before the final re-sync's START assertion
- Making "aggressive re-init" actually do a full RESET + register rewrite
- Checking all 4 device status bytes (not just device 0) during restart verification
- Checking channel data (not just status) for zeros during warmup
- Allowing multiple restart attempts during warmup
- Longer reference buffer settling time
- Power cycle recovery (would require external relay/MOSFET on the power rail)

---

## 4. Streaming Throughput (KNOWN ISSUE — NOT YET INVESTIGATED)

The GUI client shows ~108 Hz instead of 250 Hz. The CSV captures all 250 Hz samples correctly — this is purely a network streaming throughput issue.

- Acquisition: 30,227 samples in 120.91s = 250.0 Hz (correct)
- Streamed to client: 12,860 samples (only 42% throughput)
- Cause: LZ4 compression + network send in the streaming thread can't keep up
- The `data_queue` (maxsize=10000) overflows, samples are dropped

This does NOT affect data quality — the CSV has every sample. Only the real-time GUI display is affected.

---

## 5. System Architecture Reference

### Port-to-SPI Mapping

| Port | SPI Bus | SPI Device | Daisy Devices | Channels | DMA Controller |
|------|---------|-----------|---------------|----------|----------------|
| Port1 | SPI 0.0 | spidev0.0 | 4 | 32 | Legacy (dma0) |
| Port2 | SPI 0.1 | spidev0.1 | 4 | 32 | Legacy (dma0) |
| Port3 | SPI 3.0 | spidev3.0 | 4 | 32 | Legacy (dma0) |
| Port4 | SPI 3.1 | spidev3.1 | 4 | 32 | Legacy (dma0) |
| Port5 | SPI 4.0 | spidev4.0 | 4 | 32 | Legacy (dma0) |
| Port6 | SPI 4.1 | spidev4.1 | 4 | 32 | Legacy (dma0) |
| Port7 | SPI 5.0 | spidev5.0 | 4 | 32 | DMA4 (dma1) |

**Note:** The header comment in Controller.py says 3/5/5/5/3 devices per port but the actual config and CSV show 4 devices per port, 7 ports, 28 devices, 224 channels.

### ADS1299 Configuration

- CONFIG1 = 0x96 (250 SPS, daisy-chain mode)
- CONFIG2 = 0xD0 (test signal enabled — internal square wave)
- CONFIG3 = 0xE0 (internal reference enabled)
- All channels set to test signal input
- DRDY polled via I2C GPIO expander (TCA9534 at 0x20/0x21)
- START pins controlled via I2C GPIO expander (TCA9534 at 0x21)

### Key Addresses

| Register/Device | Address |
|----------------|---------|
| PACTL_CS (DMA mux) | 0xfe204e00 |
| SPI3 | 0xfe204600 (bus: 0x7e204600) |
| SPI4 | 0xfe204800 (bus: 0x7e204800) |
| SPI5 | 0xfe204a00 (bus: 0x7e204a00) |
| Legacy DMA | 0xfe007000 (bus: 0x7e007000) |
| DMA4 | 0xfe007b00 (bus: 0x7e007b00) |
| DRDY I2C expander | 0x20 |
| START I2C expander | 0x21 |

### Timing Budget per Sample (at 250 SPS = 4ms window)

| Operation | Duration |
|-----------|----------|
| I2C DRDY poll (per poll) | ~200-300us |
| Post-DRDY settling | ~100us (coded 20us, actual ~100us) |
| SPI read per port (108 bytes @ 6MHz) | ~144us |
| SPI read all 7 ports | ~1ms |
| Total per-sample overhead | ~1.4ms |
| Remaining margin | ~2.6ms |

---

## 6. SPI Read Optimization — 3-Phase Results (2026-02-16)

### Goal

Reduce corruption by minimizing time spent in the SPI-critical window between DRDY edges. Three incremental optimizations were tested:

1. Switch `xfer2()` to `readbytes()` for data reads
2. Increase SPI clock speed (dual-speed approach)
3. Per-bus DRDY triggering

### Phase 1: readbytes() — ADOPTED (single line change)

**Change:** In `read_raw()` (~line 423), replaced:
```python
return self.spi.xfer2([0x00] * (self.config.num_daisy_devices * 27))
```
with:
```python
return self.spi.readbytes(self.config.num_daisy_devices * 27)
```

**Rationale:** In RDATAC mode, ADS1299 clocks data out on SCLK edges and ignores MOSI entirely. `xfer2()` allocates a 108-element Python list, copies it into a kernel TX buffer, and sets up TX DMA — all unnecessary. `readbytes()` skips the TX buffer entirely; the kernel drives MOSI low and only captures RX.

**Results:**

| Run | Code | Samples | Corrupt Values | Corrupt Samples | Notes |
|-----|------|---------|----------------|-----------------|-------|
| Baseline 1 | xfer2 (old) | 190,000 | 2 | 1 | Pre-optimization baseline |
| Phase 1 #1 | readbytes | 104,000 | 0 | 0 | First clean run |
| Phase 1 #2 | readbytes | 112,000 | 0 | 0 | Second clean run |
| Phase 1 #3 | readbytes | ~90,000 | 0 | 0 | Third clean run |
| Phase 1 #4 | readbytes | ~110,000 | 3 | 1 | Startup artifact only |
| Phase 1 #5 | readbytes | ~100,000 | 2 | 1 | ~0.001% rate |

**Key observation:** Jitter events (timestamp gaps up to 58ms / 14.5x) that previously always caused corruption now showed NO corruption. The hot loop stalls still happened (Linux scheduling, I2C contention), but readbytes reduced per-read overhead enough that the margin absorbed them.

**Conclusion:** readbytes eliminates ~34 corrupt values per run. The remaining 0-3 per run are attributed to DRDY phase alignment randomness at startup (different every session).

### Phase 2: Dual-Speed SPI — REJECTED

**Approach:** Use 6 MHz for init (register operations), switch to higher speed after entering RDATAC for data reads only. Added `set_data_speed()` and `set_init_speed()` methods, with speed switch call in `acquisition_thread()`.

**Results:**

| Speed | Corrupt Values | Notes |
|-------|----------------|-------|
| 8 MHz data / 6 MHz init | 50 | Much worse — signal integrity failure |
| 7 MHz data / 6 MHz init | 20 | Still worse than 6 MHz |
| 6 MHz data / 6 MHz init (no-op) | 34 | Infrastructure alone caused corruption |
| 5 MHz data / 6 MHz init | 34 | Same — speed irrelevant, infrastructure is the problem |

**Root cause — spi_setup() CS glitch:** Writing to `spi.max_speed_hz` in Python triggers a kernel `spi_setup()` ioctl. This reconfigures the SPI controller, which can momentarily toggle the CS line. During RDATAC mode, the ADS1299 shift register is actively loaded — a CS glitch corrupts the frame boundary, causing the device to output data starting from the wrong bit position.

The corruption count was speed-independent (34 at both 5 MHz and 6 MHz), confirming the ioctl itself — not the speed value — was the cause. The `set_data_speed()` call happened once at acquisition start, but the CS glitch disrupted the first frame(s) and could leave residual misalignment.

**Lessons learned:**
- **NEVER write to `spi.max_speed_hz` (or any SPI config) after entering RDATAC mode.** Any `spidev` attribute write triggers a kernel ioctl that reconfigures the controller.
- 6 MHz is the maximum reliable SPI speed for 4-deep daisy chains on this PCB. The limitation is board-level signal integrity (trace impedance, connector transitions, cascaded DOUT→DIN), not the ADS1299 chip (rated for 20 MHz SCLK).
- Init is MORE sensitive to speed than data reads because register operations involve command decode + response across 4 devices, while RDATAC is a pre-loaded pure shift.

**All Phase 2 code was removed from Controller.py.**

### Phase 3: Per-Bus DRDY Triggering — REJECTED

**Approach:** Instead of waiting for ALL 7 ports' DRDY to go low before triggering ANY worker, trigger each bus worker as soon as its own ports' DRDY pins are ready. Since all 7 DRDY pins are on the same I2C expander (0x20), one I2C read returns all states — check per-bus subsets with bitmasks.

**DRDY pin-to-bus mapping:**

| Bus  | Ports        | DRDY pins | Mask |
|------|--------------|-----------|------|
| SPI0 | Port1, Port2 | P0, P6    | 0x41 |
| SPI3 | Port3, Port4 | P5, P4    | 0x30 |
| SPI4 | Port5, Port7 | P3, P2    | 0x0C |
| SPI5 | Port6        | P1        | 0x02 |

**Results:** 38 corrupted values — significantly worse than Phase 1 alone (0-3 values).

**Root cause — wrong premise:** CONFIG1 = 0x96 has CLK_EN = 0, meaning all devices use the EXTERNAL clock (provided by a single oscillator on the board). Because all 28 ADS1299 share the same clock source, their DRDY edges are synchronous — they all fire at the same instant. There is no stagger to exploit.

Per-bus triggering added Python overhead in the hot loop:
- `set()` operations (creation, iteration, `discard()`)
- `dict` lookups for per-bus masks
- `list` allocations for `triggered_workers`
- Additional `time.time()` calls for deadline checking
- All with GC disabled, so allocated objects accumulate

This overhead reduced DRDY margin for zero benefit, since all buses were ready simultaneously anyway.

**Lesson learned:** Per-bus DRDY triggering is only useful when devices have independent oscillators (CLK_EN = 1, using internal clock). With a shared external clock, DRDY edges are phase-locked and the "wait for all, trigger all" pattern is optimal.

**All Phase 3 code was removed from Controller.py.**

### check_corruption.py Enhancement

Added **out-of-range detection** to catch corruption the spike detector missed:
- `RANGE_LOW = -5000`, `RANGE_HIGH = 3000` (test signal bounds)
- Flags any value outside this range
- Catches corruption near jitter events where consecutive corrupt samples fool the neighbor-comparison spike detector (e.g., `port6_dev4_ch1 = 8388472`)

### Final State

**CORRECTION (2026-02-16):** The "Phase 1 readbytes" runs in this section were actually collected using ControllerPhase1.py, which has `xfer2` — NOT `readbytes`. The user was unknowingly running the wrong file. All Phase 1 data above represents xfer2 performance, not readbytes. See Section 7 for the corrected A/B test.

Controller.py uses `xfer2` for data reads. All Phase 2 and Phase 3 code was reverted.

**Corrected corruption summary:**

| Configuration | Typical corrupt values per ~100k samples |
|---------------|------------------------------------------|
| xfer2, 6 MHz (baseline) | 0-3 |
| Phase 2 (dual-speed infra) | ~34 (CS glitch regression) |
| Phase 3 (per-bus DRDY) | ~38 (overhead regression) |

### Remaining Corruption Sources (Phase 2/3)

Phase 2 corruption was caused by `spi_setup()` CS glitch. Phase 3 corruption was caused by added Python overhead. Both were reverted.

The baseline xfer2 corruption (0-3 per ~100k) is analyzed in detail in Section 7.

---

## 7. Continued Optimization & Root Cause Analysis (2026-02-16)

### Discovery: ControllerPhase1.py Uses xfer2, Not readbytes

All "Phase 1" benchmark runs in Section 6 were collected with ControllerPhase1.py, which has `xfer2` at line 426 — NOT `readbytes`. The readbytes change had never been tested in isolation. This invalidated the conclusion that readbytes improved performance. A proper A/B test was conducted.

### A/B Test: readbytes vs xfer2 (Controlled)

Both runs used identical Pi configs (governor=performance, IRQ affinity off core 3). Client connected for both.

| Metric | xfer2 (ControllerPhase1) | readbytes (Controller) |
|--------|------------------------|----------------------|
| Samples | 322,240 (~21.5 min) | 318,188 (~21.2 min) |
| Corrupt values | **14** | **96** |
| Corrupt samples | **1** | **27** |
| Jitter events | 4 | 7 |
| Corruption rate | 0.000155% | 0.001078% |
| Ports affected | port1, port3, port5 | **all 7 ports** |

**readbytes is ~7x worse than xfer2.** Corruption spread to ALL ports including SPI0 (historically clean with xfer2).

**Why xfer2 wins:** In full-duplex mode, xfer2 drives MOSI with explicit zeros while clocking data in. With readbytes, the kernel may leave MOSI floating or use a different DMA code path. The stable MOSI signal during xfer2 likely reduces noise coupling to MISO on the PCB, especially in the daisy chain where the signal traverses 4 devices.

**Conclusion:** readbytes reverted. xfer2 is the correct choice for data reads. DO NOT use readbytes.

### Pi System Configuration (All Persistent)

Four system-level changes, all surviving reboots:

| Change | How | Persistent |
|--------|-----|-----------|
| `isolcpus=3` | `/boot/firmware/cmdline.txt` | Yes (boot param) |
| CPU governor → `performance` | `cpu-performance.service` (systemd) | Yes |
| IRQ affinity off core 3 | `irq-affinity.service` (systemd) | Yes |
| HDMI audio disabled | `vc4-kms-v3d,noaudio` in config.txt | Yes (boot param) |

Additional hardware note: I2C is bit-banged (`i2c-gpio` overlay on bus 6, GPIO22/23) because the hardware I2C pins (GPIO2/3) are used by SPI3. The `dtparam=i2c_baudrate` approach doesn't work for bit-banged I2C.

TCA9534 INT pin is left floating on the PCB — DRDY interrupt not viable without hardware change.

### Network Activity is the Dominant Corruption Source

No-client vs client-connected comparison:

| Run | Client | Samples | Jitter events | Corrupt values | Corrupt samples |
|-----|--------|---------|--------------|----------------|-----------------|
| No client #1 | None | 146k | **0** | **1** | 1 |
| No client #2 | None | 99k | **0** | **0** | 0 |
| No client #3 | None | 174k | 7 | 10 | 1 |
| Connected (clean) | Normal | 322k | 4 | 14 | 1 |
| Connected (moderate) | Normal | 117k | 6 | 38 | 10 |
| Connected (sluggish) | Slow | 107k | **36** | **61** | 18 |

**Key findings:**
- No-client runs are dramatically cleaner but NOT perfectly clean (no-client #3 had jitter and corruption)
- Client-connected runs have higher jitter and corruption, scaling with client responsiveness
- Sluggish client → TCP retransmits → WiFi interrupt storm on cores 0-2 → SPI worker preemption

**Root cause:** The acquisition thread is protected on isolated core 3, but it only polls DRDY and triggers/waits workers there. The actual SPI reads happen in SPIBusWorker threads on **cores 0-2** — shared with WiFi interrupts, SSH, TCP stack, and streaming thread. Network activity preempts SPI workers mid-transfer.

### Two Distinct Corruption Mechanisms Identified

**1. Timing-related (DRDY overrun):**
- ~50% of corrupt samples occur 1 sample before a jitter event (sample N corrupt, jitter at N+1)
- System stall delays the SPI read past the next DRDY edge → ADS1299 overwrites shift register mid-transfer
- Device 1 usually clean (read first), devices 2-4 corrupted (read later in 108-byte frame)
- Heavily influenced by network activity — client streaming makes it worse

**2. Electrical/DMA (non-timing-correlated):**
- ~50% of corrupt samples have NO nearby jitter event
- Occurs even with no client connected (no-client #3: port5/port7 corruption at t=303s, jitter burst 4 seconds later at t=307s)
- Can corrupt device 1 (no-client #3: port7_dev1 = -1048576 = -0x100000, clean power-of-2 pattern)
- Corruption values are often exact powers of 2 or multiples of 0x100000 → SPI framing/alignment error
- Likely irreducible floor from PCB signal integrity or DMA controller transients

### DRDY Timing Margin Analysis

At 250 Hz, DRDY fires every 4ms. Data stays valid until the next DRDY overwrites it. The margin is:

```
Margin = 4ms - detection_delay - trigger_overhead - read_time
       = 4ms - detection_delay - ~0.1ms - ~0.288ms
       ≈ 3.6ms - detection_delay
```

For 2-port buses, CS0 is read first (~144us), then CS1 (~144us). CS1 has ~144us less margin:

| Timestamp gap | Detection delay | CS0 margin | CS1 margin | Result |
|--------------|----------------|------------|------------|--------|
| 4.0ms (normal) | ~0ms | 3.6ms | 3.5ms | Safe |
| 6.3ms | 2.3ms | 1.3ms | 1.2ms | Safe |
| 7.6ms | 3.6ms | 156us | 12us | Barely safe |
| 7.8ms | 3.8ms | 56us | -88us | **CS1 corrupts** |
| 8.0ms | 4.0ms | -44us | -188us | **Both corrupt** |

The real corruption threshold is ~7.7-7.8ms gap, not 8ms. The `check_corruption.py` jitter threshold of 2.0x (8ms) is slightly generous.

This explains the historical pattern where **CS1 ports (port2, port4, port6) see more corruption** — they're always read second on each bus.

### Proposed Next Step: Core Pinning for SPI Workers

Move SPI workers off the network-shared cores to isolate them from WiFi interrupts:

| Core | Role |
|------|------|
| Core 0-1 | WiFi IRQs, SSH, streaming, CSV writer, system services |
| Core 2 | All 4 SPI workers (SCHED_FIFO, shielded from network) |
| Core 3 | Acquisition thread (isolated via isolcpus=3) |

Workers spend most time blocked (Event.wait or kernel ioctl). All 4 on one core is fine — they enter the kernel quickly and block while DMA runs in hardware. The staggered kernel entry (~60-280us total) adds negligible overhead vs the 4ms budget.

**Status:** Not yet implemented.

---

## 8. Ctrl+C Termination Failure Without Client (OPEN ISSUE — 2026-02-16)

### Problem

When running without a client connected, the process cannot be terminated with Ctrl+C. Requires power cycling the Pi. When a client IS connected, Ctrl+C works normally.

### Root Cause (Likely)

The difference is the main thread's blocking call:

- **With client:** Main thread is in `time.sleep(1)` — SIGINT cleanly interrupts `nanosleep()`, KeyboardInterrupt raised immediately.
- **Without client:** Main thread is in `wait_for_client()` → `socket.accept()` with 1s timeout — SIGINT interrupts `select()`/`poll()`, but reacquiring the GIL to process the signal is contested by the SCHED_FIFO acquisition thread (priority 50), which wins every GIL race.

The acquisition thread releases the GIL frequently (during I2C `read_byte_data` and SPI `xfer2`), but at SCHED_FIFO priority it immediately reacquires it, starving the normal-priority main thread of the GIL time needed to raise `KeyboardInterrupt`.

### Proposed Fix

Register a signal handler that sets `stop_event` directly, bypassing the need for the main thread to process `KeyboardInterrupt` through normal Python exception flow:

```python
import signal
signal.signal(signal.SIGINT, lambda sig, frame: server.stop_event.set())
```

**Status:** Not yet implemented.

---

## 9. parse_raw Optimization & Streaming Degradation Analysis (2026-02-16)

### parse_raw Optimization: int.from_bytes (IMPLEMENTED — ControllerPhase1.py)

Replaced manual bit shifting in `parse_raw()` with `int.from_bytes()`:

**Before (per channel):**
```python
value = (raw_data[byte_offset] << 16) | (raw_data[byte_offset + 1] << 8) | raw_data[byte_offset + 2]
if value & 0x800000:
    value = value - 0x1000000
```

**After (per channel):**
```python
raw_bytes = bytes(raw_data)  # once per port
value = int.from_bytes(raw_bytes[offset:offset + 3], 'big', signed=True)
```

Reduces ~2,800 Python bytecodes per sample to ~900. Estimated savings: ~600μs of GIL-held time per sample. This directly increases DRDY timing margin because the acquisition loop returns to polling sooner.

### Test Results: 7 Consecutive Clean Runs

| Run | Client | Samples | Corrupt Values | Duration |
|-----|--------|---------|----------------|----------|
| 1 | None | 75,696 | 0 | 5.0 min |
| 2 | None | 70,159 | 0 | 4.7 min |
| 3 | None | 78,904 | 0 | 5.3 min |
| 4 | None | 84,509 | 0 | 5.6 min |
| 5 | Connected (250 Hz) | ~75k | 0 | ~5 min |
| 6 | Connected (250 Hz) | ~75k | 0 | ~5 min |
| 7 | Connected (250 Hz) | ~75k | 0 | ~5 min |

**Total: ~535k samples, zero corruption, zero jitter events.**

Runs 5-7 were consecutive client-connected sessions with no power cycle between them. All three displayed 250 Hz on the GUI and terminated cleanly with Ctrl+C.

### Statistical Significance

Previous baseline (ControllerPhase1.py with xfer2, no int.from_bytes):
- Best client-connected run: 14 corrupt values in 322k samples (rate: 0.00435%)
- Expected corrupt values in 225k client-connected samples: ~9.8
- Probability of getting 0 by chance: e^(-9.8) ≈ 0.006%

**Conclusion:** The `int.from_bytes` change has a statistically significant effect on corruption. The ~600μs of recovered DRDY margin absorbs network-induced jitter that previously caused DRDY overrun corruption.

### Streaming Degradation on 8th Run (OPEN ISSUE)

On a subsequent run (8th overall), the client connection degraded:

1. **0-60s:** Clean operation at 250 Hz
2. **~60s:** GUI rate dropped to 130 Hz
3. **~60-160s:** Rate continued falling to 100 Hz
4. **~160s:** GUI froze, no new data plotted
5. **Shortly after:** Pi terminal printed "CLIENT DISCONNECTED"
6. **During reconnect attempt:** Massive corruption spikes across ALL channels and ALL devices
7. **After closing GUI:** Ctrl+C took several minutes to take effect on Pi

### Analysis: Why Streaming Degrades

The 250→130→100→freeze pattern is **TCP backpressure**. Something stops consuming data fast enough, causing a cascade:

1. Consumer falls behind → TCP receive buffer fills
2. TCP advertises smaller window → Pi's `send()` starts blocking
3. Pi's `_send_nonblocking()` enters retry loop (send + sleep(0.0001), up to 2 seconds)
4. Retry loop generates TCP retransmits over WiFi → interrupt storm on cores 0-2
5. SPI workers on cores 0-2 get preempted → corruption begins
6. On disconnect: TCP RST/FIN burst → massive interrupt spike → corruption across all ports

### Root Cause: Unknown — Two Hypotheses

**Hypothesis 1 — Client-side (GUI):**
If the GUI accumulates all received data for plotting, after 60 seconds at 250 Hz × 224 channels = 3.36 million data points in the plot buffer. Rendering time grows linearly until the GUI can't consume data as fast as it arrives. This is the most common cause of this exact degradation pattern.

**Hypothesis 2 — Server-side (Pi) memory/allocator:**
With GC disabled, the streaming thread creates ~4 multi-KB allocations per batch (bytearray, bytes copy, LZ4 compressed, frame concatenation), 25 times/second. If any internal objects from `lz4.frame` or `struct` have circular references, they'd never be freed. Over 60 seconds: 1,500 batches of potentially uncollectable objects could fragment the Python allocator and slow down malloc/free, causing the streaming thread to fall behind.

### Key Observation

The first 3 client sessions (runs 5-7) were clean. The degradation happened on a later run. This means the problem is NOT inherent to the streaming architecture — something changes either:
- Within a single run after ~60 seconds of streaming (time-dependent)
- Across multiple runs within one power cycle (state-dependent)

### Diagnostic Steps (Not Yet Performed)

1. **Check client-side:** Monitor GUI's CPU and memory usage on Windows during streaming. If memory climbs steadily, the GUI is accumulating plot data.
2. **Check server-side:** Monitor Pi's RSS during streaming:
   ```bash
   watch -n 5 'ps -o pid,rss,vsz,%mem -p $(pgrep -f Controller)'
   ```
   If RSS grows steadily, something is leaking on the Pi.
3. **Check WiFi power management:** `iwconfig wlan0` — power management can introduce latency after sustained traffic.
4. **Longer single session:** Run for 20+ minutes to confirm whether degradation always starts at ~60 seconds.

### Ctrl+C Behavior Summary

| Scenario | Ctrl+C behavior |
|----------|----------------|
| Client connected, 250 Hz | Immediate termination |
| Client degraded, then disconnected | Delayed several minutes |
| No client connected | Delayed, required power cycle on earlier runs |

The pattern is consistent: Ctrl+C works when the main thread is in `time.sleep(1)` (client connected), fails when in `accept()` loop (no client). After client disconnect, the main thread transitions to `accept()` and the same starvation applies.

**Status:** int.from_bytes optimization implemented and verified. Streaming degradation root cause under investigation.
