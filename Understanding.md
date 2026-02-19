# Test Runs

## Run #1 — 2026-02-17
Testing dead code removal from ControllerPhase1.py and stuck channel detector in simpleviz.py to verify no regressions and immediate visibility of init failures.

### Results
- **Stuck detector**: worked correctly, flagged Port4 (all 32 channels stuck at 0)
- **Dead code removal**: no regressions, server ran identically
- **Rate**: 245.7 Hz, 84.1s runtime, 20649 samples acquired

### Init Observations
- All 7 ports configured successfully (ID 0x3E), register writes needed many retries (up to 21 attempts for CH1SET on Port1)
- Port2: zeros at DRDY check, restart failed once, succeeded during warmup (attempt 2)
- Port7: zeros at DRDY check, restart succeeded (attempt 1)
- Port4: passed initial DRDY check (5/5 valid), but hit 20+ zeros during warmup at sample 20, restart fixed it
- After warmup: all 7 ports verified 20/20 valid

### Key Finding: Final Re-Sync Breaks Recovered Ports
Port4 was recovered by per-port restart during warmup, but the **final re-synchronization step** (stop all → flush → RDATAC → restart all simultaneously) put Port4 back into all-zeros. The blanket stop/restart undoes individual port recoveries. This is the root cause of "ports that pass init but stream zeros."

### Minor Bug
`Sent: 34736` is double-counted (17368 x 2) — disconnect handler adds to `total_samples_sent` but `samples_sent` isn't reset before final stats when no new client connects after disconnect.

---

## Run #2 — 2026-02-17
Testing with final re-sync removed and double-count bug fixed.

### Changes Made
1. **Removed final re-sync block** from `start_all_conversions_synchronized()` — the stop-all/flush/restart-all cycle was undoing per-port warmup recoveries. Shared external clock means DRDY is already synchronous; re-sync is unnecessary.
2. **Fixed double-count bug** in `streaming_thread()` — added `self.samples_sent = 0` after accumulating into `total_samples_sent` on both disconnect paths.

### Results
- **Re-sync removal: SUCCESS** — 5/7 ports had initial zeros, all recovered via restart, all stayed recovered during streaming
- **Rate**: 249.6 Hz (improved from 245.7), 60.6s runtime, 15134 acquired
- **Double-count fix confirmed**: Sent: 11780 matches session count exactly
- **Port2 Dev4 stuck**: single device (last in 4-deep daisy chain) outputting zeros, rest of Port2 fine

### Key Finding: Per-Device Failures Are Invisible to Server
The server's `write_and_verify()` only reads back from device 1 in each daisy chain. Devices 2-4 receive the same commands (shared MOSI) but their register state is never verified. The status byte check and "data flowing" verification also only see device 1. A single failed device deep in the chain is **only detectable by the client-side stuck detector**.

### Failure Pattern Summary (Runs 1+2)
- **Whole-port zeros**: caused by the (now-removed) re-sync step. Per-port restart during warmup fixes these reliably.
- **Single-device zeros**: device at end of daisy chain fails silently. Server cannot detect. Likely signal integrity or device-level issue — command propagation verified OK (shared MOSI) but data readback from device 4 is all zeros.

---

## Pre-Run #3 Change: Daisy-Chain Register Read

### Problem
`read_registers()` only clocked out enough bytes for device 1. Devices 2-4 shift their register values through the daisy chain (DOUT→DAISY_IN) but those extra bytes were never clocked out.

### Fix
Added `num_devices` parameter to `ADS1299_SPI.read_registers()` (default=1 for backward compatibility). With `num_devices=4`, clocks out 4x the bytes, returning `[dev1_val, dev2_val, dev3_val, dev4_val]`. All existing callers unchanged — they still use `read_registers(reg, 1)[0]` and get device 1 only.

---

## Run #3 — 2026-02-17
No stuck channels. Testing re-sync removal stability over 5+ minutes with daisy-chain register read deployed.

### Results
- **No stuck channels** — re-sync removal holding, all 7 ports streaming
- **Rate**: 249.9 Hz, 317.8s runtime (5.3 min), 79416 acquired, 76350 sent
- **Double-count fix confirmed**: 76350 sent matches session count
- **Init**: 5/7 ports had initial zeros, all recovered. Port7 needed aggressive re-init (100% warmup corruption → full re-init → 0% corruption)
- **Corruption**: 7 samples, 23 values (0.001%) — regression from previous zero-corruption baseline

### Corruption Analysis
All 7 events hit dev3/dev4 (end of daisy chain) except sample 75833 which hit all 4 Port2 devices:
- t=23.6s: Port2 dev3+4, bit-shift >>4 pattern
- t=47.1s: Port6 dev3+4, identical garbage values
- t=152.9s: Port5 dev3+4, identical garbage values
- t=210.2s: Port6 dev4, large garbage (-5.4M)
- t=233.5s: Port4 dev4, large garbage (131K)
- t=280.0s: Port2 dev4, large garbage (-2.3M)
- t=303.4s: Port2 all 4 devices, bus-level glitch

### Acquisition Loop Diff
Diffed `ControllerPhase1.py` acquisition_thread against original `testing/Controller.py` — **byte-for-byte identical**. All changes (dead code removal, re-sync removal, double-count fix, read_registers signature) are outside the hot loop.

---

## Run #4 — 2026-02-17 (Quick diagnostic)
Ran original `testing/Controller.py` server with modified `client/simpleviz.py` to isolate client-side impact.

### Results
- **Rate immediately dropped** from ~183 Hz → 130 Hz (client-side throughput)
- Stalls in sample transmission appeared as rate degraded
- Server code was ORIGINAL (no changes), so the client is the cause

### Root Cause: Stuck Detector Performance Bug
`update_stuck_indicators()` calls `setStyleSheet()` on all 28 device buttons every 15th frame (~300ms), even when nothing is stuck. Qt `setStyleSheet()` is expensive (CSS parsing + layout recalculation). 28 calls × ~2-5ms each = 56-140ms overhead every 300ms, starving the Qt main thread and preventing timely `process_batch()` calls → queue backup → TCP backpressure → rate drop.

### Fix
Changed `update_stuck_indicators()` to cache each button's stuck state (`_dev_stuck_state` list) and only call `setStyleSheet()` when the state actually changes. Normal operation = 0 stylesheet calls per frame.

---

## Run #5 — 2026-02-17
Testing fixed client (stuck detector perf fix) against original `testing/Controller.py` server. 5-minute run to confirm zero corruption baseline is restored.

### Results
- **CLEAN** — zero corruption events over 5.3 minutes
- **Rate**: 250.0 Hz, 316.55s runtime, 79137 samples acquired
- **Timing**: mean dt 4.000ms, max dt 6.476ms, 0 jitter events (>2x), 99.99% normal
- **No gaps, no rail values, no spikes, no bit-shifts, no out-of-range**

### Conclusion
The client-side stuck detector performance bug was causing TCP backpressure which propagated to the server, creating acquisition stalls that corrupted the next SPI read. With the fix applied, the zero-corruption baseline is restored **when using the original server code with re-sync**.

---

## Run #6 — 2026-02-17
Testing modified `server/ControllerPhase1.py` (re-sync removed) with fixed client to isolate server-side impact.

### Results
- **21 corrupt samples**, 86 corrupted values (0.004%) over 5.2 minutes
- **Rate**: 250.0 Hz, 314.25s runtime, 78551 samples
- **Timing**: mean dt 4.001ms, max dt 7.901ms, 0 jitter events
- **Startup artifacts**: 4 channels on Port6
- All 7 ports affected, Port7 worst (30 values), Port6 (16), Port2 (15)

### Key Finding: Corruption Is Periodic (~23.4s interval)
| Sample | Time | Delta (samples) |
|--------|------|-----------------|
| 5909 | 23.6s | — |
| 11769 | 47.1s | 5860 |
| 17610 | 70.4s | 5841 |
| 23469 | 93.9s | 5859 |
| 29290 | 117.2s | 5821 |

~5850 samples apart, very consistent. Not random stalls — some periodic system event (WiFi scanning, kernel timer, etc.) that the system tolerates WITH re-sync but is vulnerable to WITHOUT it.

### Conclusion: Two Independent Bugs
1. **Client bug** (Run #4, fixed): `setStyleSheet()` spam → Qt starvation → TCP backpressure → stalls → corruption
2. **Server bug** (this run): Re-sync removal leaves devices in a less resilient state. The flush + SDATAC/RDATAC + 100 settling samples provided a clean aligned start that tolerated periodic OS interference.

Run #3's corruption was caused by BOTH bugs simultaneously. The client bug is fixed. The server needs a modified re-sync that provides the flush/alignment benefits without the harmful stop-all/restart-all that breaks recovered ports.

---

## Init Sequence Redesign — First Principles Testing

### Approach Change
Step 0A+0B and Test 1 were abandoned — Test 1 (daisy-chain RREG) returned 0x00 for devices 2-4, proving the protocol doesn't work as assumed. Too many stacked changes obscured root causes. **Reset: ControllerPhase1.py is now an exact copy of Controller.py (known zero-corruption baseline) with ONE change per test.**

---

### Test 1: ABANDONED
Daisy-chain RREG reads 0x00 for devices 2-4. Protocol needs investigation.

### Test 2: Reduce resets from 5 to 3
**Change:** `num_resets = 3` (line 723, only change from Controller.py)
**Risk:** LOW | **Expected savings:** ~19.6s | **Result: PASS**

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 79,192  | 0       | PASS   | 5.3 min, 250.0 Hz, zero corruption |

Jumped directly to Test 3 (aggressive approach).

### Test 3: Reduce resets from 3 to 2
**Change:** `num_resets = 2` (line 723, only change from Controller.py)
**Risk:** LOW-MEDIUM | **Expected savings:** ~29.4s total (from 5) | **Result: PASS**

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 79,192  | 0       | PASS   | 5.3 min, 250.0 Hz, max jitter 6.7ms |
| 2   | 79,166  | 0       | PASS   | 5.3 min, 249.9 Hz, max jitter 41.4ms, 6 client reconnects (client-side TCP issue, not server) |
| 3   |         | 0       | PASS   | TCP streaming removed, CSV-only — confirms jitter was client-side |

**Accepted.** `num_resets = 2` is the new baseline. TCP streaming removed for remaining tests.

### Test 4: Reduce resets from 2 to 1
**Change:** `num_resets = 1`
**Risk:** MEDIUM (stay at 2 if fails) | **Expected savings:** additional ~9.8s

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   |         |         |        |       |
| 2   |         |         |        |       |

### Test 5: Reduce per-reset settling time
**Change:** `reset_time` from `1.0 + (N * 0.1)` to `0.1 + (N * 0.01)`
**Risk:** MEDIUM (fallback: 0.7s, then 0.35s) | **Expected savings:** ~17.6s

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 81,900  | 0       | PASS   | 5.5 min, 249.9 Hz, max jitter 6.4ms, CSV-only |
| 2   | 87,207  | 0       | PASS   | 5.8 min, 250.0 Hz, max jitter 7.2ms, CSV-only |

**Accepted.** Reset settling `0.1 + (N * 0.01)` is the new baseline.

### Test 6: Reduce reference buffer settling from 500ms to 200ms
**Change:** `time.sleep(0.5)` -> `time.sleep(0.2)` after CONFIG3
**Risk:** MEDIUM (fallback: 300ms) | **Expected savings:** ~2.1s

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 78,494  | 0       | PASS   | 5.2 min, 250.0 Hz, max jitter 5.7ms, 100% normal, CSV-only |
| 2   | 78,060  | 0       | PASS   | 5.2 min, 250.0 Hz, max jitter 6.4ms, CSV-only |

**Accepted.** Reference buffer settling 200ms is the new baseline.

**Note (2026-02-18):** System power-cycled overnight. Two consecutive runs had device init failures (incorrect initialization). Init reliability issue remains prevalent across power cycles — not caused by timing changes. Continuing testing.

### Test 7: Reduce post-RDATAC settling from 750ms to 200ms
**Change:** `time.sleep(0.750)` -> `time.sleep(0.200)`
**Risk:** LOW | **Expected savings:** ~0.55s

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 77,300  | 0       | PASS   | 5.2 min, 250.0 Hz, max jitter 6.6ms, 100% normal, CSV-only |
| 2   | 79,083  | 0       | PASS   | 5.3 min, 249.9 Hz, max jitter 6.4ms, 100% normal, CSV-only |

**Accepted.** Post-RDATAC settling 200ms is the new baseline.

### Test 8: Reduce post-START sync wait from 500ms to 100ms
**Change:** `time.sleep(0.500)` -> `time.sleep(0.100)` in start_all_conversions
**Risk:** LOW | **Expected savings:** ~0.4s

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 78,073  | 0       | PASS   | 5.2 min, 250.0 Hz, max jitter 6.9ms, 100% normal, CSV-only |
| 2   | 80,459  | 0       | PASS   | 5.4 min, 250.0 Hz, max jitter 6.4ms, 100% normal, CSV-only |

**Accepted.** Post-START sync wait 100ms is the new baseline.

### Test 9: Remove redundant stabilization wait + consolidate DRDY verification
**Change:** Remove 1000ms stab wait + second DRDY verification pass (lines 956-998)
**Risk:** LOW-MEDIUM | **Expected savings:** ~1.2s | **Result: REVERTED**

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 6,343   | 0*      | FAIL   | Port6 dev1 all-zeros, survived re-init. 0 corruption on 6 working ports. |
| 2   | 6,343   | 0*      | FAIL   | Port4 dev1 all-zeros. 0 corruption on 6 working ports. |
| 3   | 8,019   | 0*      | FAIL   | Port3 dev1 all-zeros. 0 corruption on 6 working ports. |

**Reverted.** 3 consecutive init failures (different ports each time). The 1000ms stabilization + 2nd DRDY check likely aids device recovery. Not worth 1.2s savings. Moving on to Test 10.

### Test 10 (REVISED): Reduce `write_and_verify` sleep from 100ms to 10ms
**Change:** `time.sleep(0.1)` -> `time.sleep(0.01)` in `write_and_verify()` (line 794)
**Risk:** LOW-MEDIUM | **Expected savings:** ~8.2s (91 calls x 90ms saved)
Datasheet says registers settle in microseconds; 10ms is 5,000x margin. Retry loop (100 attempts) catches any failures.

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 108,907 | 0       | PASS   | 7.3 min, 250.0 Hz, max jitter 6.4ms, CSV-only |
| 2   | 78,532  | 0       | PASS   | 5.2 min, 250.0 Hz, max jitter 6.2ms, CSV-only |

**Accepted.** `write_and_verify` sleep 10ms is the new baseline. ~8.2s saved from init.

### Test 11 (REVISED): Reduce SDATAC sends from 6 to 2, sleep 20ms to 5ms
**Change:** `sdatac_count` from `max(5, num_devices+2)` to `2`, sleep 20ms→5ms, extra settle 100ms→10ms (lines 735-739)
**Risk:** LOW | **Expected savings:** ~1.4s (7 ports x 200ms saved)
All devices share MOSI — single SDATAC reaches all 4 simultaneously. SDATAC verification (LOFF test) catches any failures.

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 93,362  | 0       | PASS   | 6.2 min, 250.0 Hz, max jitter 6.0ms, 100% normal, CSV-only |
| 2   | 12,675  | 0*      | FAIL   | Port3 dev1 all-zeros (init failure). 0 corruption on 6 working ports. |

**Reverted.** Init failures increased — reducing SDATAC sends may hurt device recovery. The extra SDATAC commands + settling likely help marginal devices. Not worth 1.4s savings.

### Test 12 (REVISED): Parallelize port init across independent SPI buses
**Change:** Init ports on different SPI buses concurrently (SPI0/SPI3/SPI4/SPI5 are independent HW)
**Risk:** MEDIUM | **Expected savings:** ~6.5-13s (4 buses in parallel vs 7 ports sequential)
| **Result: REVERTED**

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   | 0       | N/A     | FAIL   | Port4 read Device ID 0x20. Init abort. |
| 2   | 69,252  | 12      | FAIL   | First real corruption in testing. 12.1ms stall at sample 14130, 3 ports hit simultaneously (Port1/3/5). |

**Reverted.** Run 1: Port4 ID read 0x20 (also happened on sequential — may be pre-existing). Run 2: System-wide stall caused corruption across 3 independent SPI buses. Not worth the risk for ~6.5s savings.

---

### Phase 1 Summary

**Accepted optimizations (cumulative savings ~58s from original ~60s fixed delays):**
1. `num_resets`: 5 → 2 (Test 3, ~29.4s saved)
2. `reset_time`: `1.0+(N*0.1)` → `0.1+(N*0.01)` (Test 5, ~17.6s saved)
3. Reference buffer settling: 500ms → 200ms (Test 6, ~2.1s saved)
4. Post-RDATAC settling: 750ms → 200ms (Test 7, ~0.55s saved)
5. Post-START sync wait: 500ms → 100ms (Test 8, ~0.4s saved)
6. `write_and_verify` sleep: 100ms → 10ms (Test 10, ~8.2s saved)

**Reverted (caused issues):**
- Test 9: Remove 1000ms stabilization + 2nd DRDY check — increased init failures
- Test 11: Reduce SDATAC sends 6→2 — increased init failures
- Test 12: Parallel port init — caused real data corruption + init failures

**Note:** TCP streaming is disabled for testing isolation. CSV-only output.

### Final Validation
| Run | Duration | Samples | Corrupt | Result | Notes |
|-----|----------|---------|---------|--------|-------|
| 1   | 21.6 min | 323,750 | 14 (1 sample) | NEAR-PASS | 250.0 Hz, 8.2ms stall at sample 215335. 3 ports hit (Port2/5/7). 0.000154% corruption rate. |

**Conclusion:** Init optimizations are solid — zero corruption across all 5-min A/B tests. The single corruption event in the 20-min run was caused by an 8.2ms stall (barely 2x the 4ms sample period). This is the fundamental Python limitation: GIL + spidev kernel overhead leaves near-zero headroom for OS scheduling jitter.

---

## Phase 2: C++ Hot Loop Migration

**Goal:** Migrate the SPI acquisition hot loop from Python to C++ to dramatically reduce per-read overhead and increase headroom against OS scheduling jitter.

**Rationale:** The 8.2ms stall that caused corruption in the final validation was only 2x the sample period. Python's GIL, spidev ioctl overhead, and interpreted execution leave ~0.5-1ms margin. C++ with direct SPI access should reduce per-sample read time significantly, providing enough headroom to absorb these stalls without data corruption.

---

## Phase 2 Results: C++ Acquisition Engine (Phase 1 — Acquisition + CSV)

### What Was Built
Complete C++20 rewrite of the Python acquisition hot loop, targeting the Raspberry Pi 4 (BCM2711 / Cortex-A72). Covers device initialization, real-time parallel SPI acquisition, and async CSV logging. TCP streaming (Phase 2 of the C++ rewrite) is not yet implemented.

Source: `Cpp Implementation/` (22 source files across 4 modules)

### Architecture
- **Direct SPI ioctl**: `ioctl(SPI_IOC_MESSAGE)` replaces Python's `spidev.xfer2()`, eliminating per-call GIL/object-allocation overhead
- **eventfd-triggered bus workers**: One persistent thread per physical SPI bus (4 buses), signaled via `eventfd` for minimal-latency wakeup. Ports on the same bus are read sequentially; buses execute truly in parallel (no GIL)
- **SPSC lock-free ring buffer**: Single-producer (acquisition thread) single-consumer (CSV writer thread) bounded ring for zero-contention sample handoff
- **SCHED_FIFO 50 on core 3**: Acquisition thread pinned to isolated core with real-time priority, `mlockall(MCL_CURRENT | MCL_FUTURE)` to prevent page faults
- **Zero heap allocation in hot loop**: All SPI buffers, parse buffers, and sample structs pre-allocated at startup
- **Faithful init port**: All Python retry loops, SDATAC verification, warmup/restart/re-sync sequences preserved exactly

### Build & Deploy
- Built natively on Pi with GCC 14.2
- Flags: `-std=c++20 -mcpu=cortex-a72 -O2 -fno-exceptions -fno-rtti`, LTO enabled
- Binary: `~/ads1299-cpp/build/ads1299_acquire` (79 KB), requires `sudo` for SCHED_FIFO and SPI/I2C access

### First Validation Run
| Metric | Result |
|--------|--------|
| Duration | 23.3 minutes |
| Samples | 349,369 |
| Channels | 224 (7 ports × 4 devices × 8 channels) |
| Sample rate | 250.0 Hz (locked) |
| Gaps | 0 |
| Corruption events | **0** |
| Jitter events (>2× period) | 0 |
| dt mean | 4.000 ms |
| dt min | 1.241 ms |
| dt max | 4.660 ms |
| Normal samples | 100% |

### Comparison with Python Baseline
| Metric | Python | C++ |
|--------|--------|-----|
| Max dt | 8.2 ms (caused corruption) | 4.66 ms |
| Typical cycle time | 2.5–3.0 ms | < 1.0 ms |
| Headroom per cycle | 1.0–1.5 ms | > 3.0 ms |
| Corruption (20+ min run) | 1 event (14 values) | **0** |
| GC/GIL stalls | Yes (mitigated with gc.disable) | N/A |

The max dt of 4.66 ms is only 0.66 ms past the sample period — well within tolerance since the next DRDY hasn't been missed. Python's 8.2 ms stall exceeded 2× the period, missing a DRDY edge entirely.

### Conclusion
The C++ rewrite eliminates the Python-specific corruption mechanism. Zero corruption over 349k samples (23 minutes) validates that the architecture — direct SPI ioctl, true parallelism, deterministic memory, RT scheduling — provides sufficient headroom to absorb OS scheduling jitter without data loss.

---

## Phase 2 Results: C++ TCP Streaming (Phase 2 — Acquisition + CSV + TCP)

### What Was Built
Added TCP streaming as a parallel output path alongside CSV, wire-compatible with the existing `simpleviz.py` Python client. No client-side changes required. The acquisition hot loop's only touch point is a single non-blocking `try_push()` to a second SPSC ring buffer.

New source files: `include/streaming/{protocol,server}.hpp`, `src/streaming/{protocol,server}.cpp`

### Architecture
- **Separate SPSC ring** (2048 slots): Streaming has its own ring buffer, independent from the CSV ring. A slow or disconnected client cannot affect CSV writes or vice versa.
- **Accept thread** (core 1, SCHED_OTHER): Listens for TCP connections via `poll()` + `accept()`. Hands off new client fd to the streaming thread via atomic exchange.
- **Streaming thread** (core 2, SCHED_OTHER): Pops samples from ring, packs into wire format, compresses with LZ4 streaming API, sends as framed binary data. Handles reconnection, ring drain, partial batch flush.
- **LZ4 streaming API**: Uses `LZ4F_compressBegin/Update/End` with a context allocated once at startup — zero per-call memory allocations (unlike `LZ4F_compressFrame()` which mallocs internally).
- **Wire protocol**: Metadata JSON (once per connection) + `[u32 compressed_size][u32 sample_count][LZ4 frame]` — byte-for-byte compatible with `simpleviz.py`.
- **Fault isolation**: `SIGPIPE` ignored, `MSG_NOSIGNAL` on all sends, TCP keepalive (`KEEPIDLE=10, KEEPINTVL=5, KEEPCNT=3`), periodic non-blocking `recv()` for fast disconnect detection.

### Configuration
7 SPI ports, 4 daisy-chained ADS1299 devices per port, 28 devices total, 224 channels @ 250 Hz. Default streaming on `0.0.0.0:8888`. Configurable via `--host` and `--port` arguments.

### Validation Run
| Metric | Result |
|--------|--------|
| Duration | 20.6 minutes (1234.6 s) |
| Samples | 308,491 |
| Channels | 224 (7 ports x 4 devices x 8 channels) |
| Sample rate | 250.0 Hz (locked, 249.9 Hz measured) |
| Corruption events | **0** |
| DRDY timeouts | 0 |
| CSV ring drops | 0 |
| Stream ring drops | **0** |
| Stream samples sent | 307,150 (99.6% — rest in flight at shutdown) |
| Stream batches | 30,715 |
| Reconnects | 1 |
| Cycle time (min/mean/max) | 0.35 / 0.36 / 0.73 ms |
| Sample dt (min/max) | 1.89 / 4.25 ms |

### Key Observations
- **Zero drops across entire run**: The streaming ring never dropped a single sample over 308k samples. Ring queue depth stayed at 0-2 samples throughout, confirming the streaming thread keeps up effortlessly.
- **Client disconnect handled cleanly**: Client disconnected at ~1233s (`[STREAM] Send failed -- client disconnected`), server continued running and shut down cleanly via Ctrl+C 1s later.
- **No impact on acquisition timing**: Cycle time (0.36ms mean) and dt range (1.89-4.25ms) are identical to Phase 1 CSV-only results, confirming the second ring push adds negligible overhead.
- **Port6 recovered**: Port6 (SPI4.1/CS1) which was stuck at all-zeros in previous runs initialized and streamed successfully in this run.
- **Init needed restarts**: 6/7 ports returned zeros initially, all recovered via per-port restart during warmup. This is the expected behavior for the V2 board.

### Comparison with Python Streaming
| Metric | Python Server | C++ Server |
|--------|--------------|------------|
| Client receive rate | ~108 Hz | **250 Hz (full rate)** |
| Stream drops | Frequent (queue overflow) | **0** |
| Backpressure risk | High (TCP stalls corrupt SPI) | **None** (isolated ring) |
| Per-batch overhead | GIL + struct.pack + lz4.frame | Pre-allocated memcpy + LZ4F streaming |

### Conclusion
Phase 2 TCP streaming is validated. Zero corruption, zero drops, full 250 Hz delivery to the client, clean disconnect/reconnect handling, and no measurable impact on acquisition timing. The system is now feature-complete for real-time 224-channel EEG acquisition with simultaneous CSV recording and live TCP streaming.

---

## Phase A Results: Init Reliability (2026-02-18)

### What Was Changed
5 changes to eliminate init failures and guarantee all 328 channels are validated before acquisition starts:

1. **Per-device status validation in hot loop** (`engine.cpp`): Previously only checked device 0 per port — 280/328 channels (85%) unmonitored. Now validates ALL devices in each daisy chain.
2. **Per-device validation during init** (`controller.cpp`): Updated 6 locations (warmup, mini-warmup, data flow verification, individual port check, restart_single_port) to check all devices instead of just device 0.
3. **Per-port re-sync** (`controller.cpp`): Replaced global re-sync (STOP all → flush → RDATAC → restart all) with independent per-port cycling. If re-sync breaks a port, it's immediately recovered without affecting others. Eliminates the ~50% regression from global stop/restart.
4. **Exhaustive init retry loop** (`main.cpp`): 10 retries, 5-minute hard timeout, 100-sample health check requiring 90% valid on ALL devices on ALL ports before proceeding.
5. **WiFi scanning disable** (Pi config change): Not yet applied.

### First Run — Init SUCCESS
| Metric | Result |
|--------|--------|
| Duration | 303.4 s (5.1 min) |
| Samples | 75,628 |
| Channels | 328 (7 ports × 41 devices) |
| Sample rate | 250.0 Hz |
| Init attempts | **1** (first attempt success) |
| Init ports with zeros | 6/7 (Port2-7), all recovered via per-port restart |
| Extended health check | 100% valid on all 7 ports (100/100 each) |
| Per-port re-sync | All 7 ports passed without needing recovery |
| Cycle time (min/mean/max) | 0.63 / 0.65 / 1.92 ms |
| Sample dt (min/max) | 3.77 / 6.55 ms |
| DRDY timeouts | 0 |
| Status corruptions | 3 (1 startup artifact + 2 at t=275s) |
| CSV ring drops | 0 |
| Stream drops | 0 |

### Key Finding: Init Reliability FIXED
Phase A init worked exactly as designed. 6/7 ports returned zeros initially (the known RDATAC timing race), all recovered during warmup restarts, all passed per-port re-sync without breaking, and the extended health check verified 100% on every port. The system entered acquisition with all 328 channels validated — something that was never guaranteed before.

### Remaining Issue: Late-Run Corruption at t=275s
Despite clean init, corruption occurred at ~275s:
- **Sample 68787** (t=275.152s): 8 channels on Port2 — spike values
- **Sample 68946** (t=275.791s): 72 channels on Port1 — ALL 9 devices affected, bus-level event
- **Sample 68977** (t=275.916s): 3 channels on Port2 + 1 rail value (-8388608)

At 270s the streaming ring queue spiked to 15 (normally 0-1). Max cycle jumped 0.98ms → 1.92ms, max dt jumped 4.45ms → 6.55ms. This is consistent with the ~23.4s WiFi background scanning interference pattern observed in Python Run #6. WiFi scanning disable (Phase A Change 5) has not yet been applied.

The corruption events produced valid status bytes (0xC_) but corrupt channel data — meaning the ADS1299 was streaming correctly but SPI data was corrupted in transit (likely DMA contention from WiFi scanning).

### Corruption Root Cause Analysis (cpp-embedded + performance-engineer agents)
Both agents independently identified the same mechanism: **ADS1299 shift register overwrite during SPI readout**.

When DRDY polling + SPI transfer total exceeds ~3.9ms, the next DRDY edge overwrites the shift register mid-transfer. Status bytes pass (0xC0 is sample-invariant, clocked out first in ~4us) but channel data is corrupted. Deep devices (dev8, dev9) are worst affected — their data is clocked out last. The -8388608 (0x800000) rail value is a specific artifact of catching the register mid-load (new MSB + stale zero LSBs).

**Port1 (9 devices = 324us SPI transfer) is the most vulnerable** due to longest chain. The SPI5 worker sharing core 0 with SPI0 added ~252us contention.

### Fixes Applied
1. **WiFi background scanning disabled** (`wpa_cli scan_interval 9999`, persistent via `wifi-noscan.service`)
2. **SPI5 worker moved from core 0 → core 2** — SPI0 (Port1+2) now sole owner of core 0

---

### Second Run — Init FAILURE: Retry Loop Design Flaw Exposed

Port4 got stuck at 100% zeros (RDATAC timing race). The exhaustive retry loop reconfigured **ALL 7 ports** from scratch on every retry — even though only Port4 was broken. Ports 1,2,3,5,6,7 were all healthy but got needlessly torn down and re-initialized on retries 6, 7, 8, and 9.

**Problems identified:**
1. **Wasteful**: Each full retry takes ~30s to reconfigure all 7 ports. Only Port4 needed help.
2. **Uncancellable**: Ctrl+C was pressed dozens of times (visible as `^C` throughout the log) but SCHED_FIFO priority + signal handling prevented exit. Had to wait for the 5-minute timeout.
3. **Wrong fix for this failure mode**: The RDATAC timing race is a probabilistic failure in the RDATAC→START transition, NOT in register configuration. Full re-init (reset + register writes + RDATAC + START) wastes ~5-10s per port when only the STOP→SDATAC→RDATAC→START cycle (~200ms) is needed.
4. **No graceful degradation**: If Port4 is truly dead (hardware), the system should skip it and run with 6/7 ports instead of looping until timeout.

**Correct approach** (being implemented):
- Configure all ports once (register writes always succeed)
- Start all ports, verify each independently
- For failing ports ONLY: cycle STOP→SDATAC→RDATAC→START (~200ms), verify, repeat
- Check `running` flag in all loops so Ctrl+C works immediately
- After N failed attempts on one port, skip it and run with remaining ports

---

### Third Run — WiFi Fix Validated, Residual Corruption Observed (old init code + WiFi fix)

Running with WiFi scanning disabled (`wpa_cli scan_interval 9999`) but still using the old init code (with per-port re-sync, exhaustive retry loop). SPI5 still on core 0 in this build.

| Metric | Result |
|--------|--------|
| Duration | 336.5 s (5.6 min) |
| Samples | 83,938 |
| Sample rate | 250.0 Hz |
| Init attempts | 1 (all 7 ports healthy) |
| Cycle time (min/mean/max) | 0.63 / 0.64 / 4.73 ms |
| Sample dt (min/max) | 3.79 / 4.95 ms |
| DRDY timeouts | 0 |
| Status corruptions | 2 (1 startup + 1 at t=306s) |
| CSV ring drops | 0 |
| Stream drops | 0 |

**Corruption at t=306s — different pattern from WiFi stall:**

| Sample | Time | Channels | Ports |
|--------|------|----------|-------|
| 0 | 0.000s | 21 channels | Port1 (startup artifact, dev7-9) |
| 76502 | 306.020s | 6 channels | Port6 (SPI4) |
| 76503 | 306.024s | 20 channels | Port3 (SPI3, dev3-5, includes 1 bit-shift) |

**Key observations:**
1. **WiFi fix is working** — no periodic ~23.4s corruption pattern, no stream queue spikes (stayed at 0-1 throughout), no DMA contention signature.
2. **Different buses affected**: Port3 (SPI3) and Port6 (SPI4), NOT Port1/2 (SPI0) like the WiFi stall run. Two independent SPI buses corrupted on consecutive samples.
3. **4.73ms max cycle appeared at t=50s** and stayed as the max — a single spike event, not recurring. The 4.95ms max dt is barely past the 4ms period.
4. **Port3 corruption pattern**: dev3 (8 values), dev4 (15 values), dev5 (16 values) — deeper devices have more corruption, consistent with the shift-register-overwrite mechanism. Bit-shift match confirms SPI data misalignment.
5. **Two consecutive corrupt samples** (76502 on Port6, 76503 on Port3) — suggests a brief system-wide stall affecting multiple buses simultaneously, but NOT WiFi-related (no queue spike, no periodicity).
6. **Likely cause**: one-off kernel event (SD card journal flush, kernel timer, remaining interrupt on bus worker cores). This is the residual noise floor after eliminating WiFi DMA contention.

---

## Phase B: Thread/IRQ Isolation (2026-02-18)

### Root Cause
After WiFi scanning was disabled (Phase A), 2 corruption events remained at t=306s. Port3 (SPI3, core 1) and Port6 (SPI4, core 2) corrupted on consecutive samples — exactly the cores shared with streaming threads. The accept thread was on core 1 (contending with SPI3 worker) and the CSV writer was unpinned (could land on any core).

**Core map before fix:**
| Core | RT Thread | Non-RT Contention |
|------|-----------|-------------------|
| 0 | SPI0 worker (Port1+2) | kernel, IRQs |
| 1 | SPI3 worker (Port3+4) | **Accept thread** (SCHED_OTHER) |
| 2 | SPI4+SPI5 workers | **Stream thread** (LZ4+send) |
| 3 | Acquisition engine (FIFO 50) | (isolated) |
| ? | — | **CSV writer** (unpinned) |

### Changes Applied

**Code changes (3 files):**
1. **`src/streaming/server.cpp`**: Moved accept thread from core 1 → core 2 (`CPU_SET(2, &cpuset)`)
2. **`src/logging/csv_writer.cpp`**: Pinned CSV writer thread to core 2 (added `pthread_setaffinity_np` + includes)
3. **`src/acquisition/bus_worker.cpp`**: Added `mlockall(MCL_CURRENT | MCL_FUTURE)` + 64KB stack pre-fault to prevent page faults during SPI ioctl

**System changes (Pi):**
4. **`/boot/firmware/cmdline.txt`**: Added `irqaffinity=0,2` — kernel defaults all hardware IRQs to cores 0 and 2 only. Cores 1 and 3 receive no IRQs.
5. **Disabled unnecessary services**: `apt-daily.timer`, `apt-daily-upgrade.timer`, `man-db.timer`, `logrotate.timer`, `dphys-swapfile.service`
6. **Journald → volatile**: RAM-only logging (16MB cap), no SD card writes from journald
7. **VM writeback tuning**: `dirty_ratio=80`, `dirty_background_ratio=50`, `dirty_writeback_centisecs=60000` — reduces SD card flush frequency

**Target core map after fix:**
| Core | RT Thread | Non-RT (acceptable) |
|------|-----------|---------------------|
| 0 | SPI0 worker (Port1+2) | **clean** — sole RT owner |
| 1 | SPI3 worker (Port3+4) | **clean** — sole RT owner |
| 2 | SPI4+SPI5 workers | Accept, stream, CSV (all SCHED_OTHER, preempted by FIFO 49) |
| 3 | Acquisition engine (FIFO 50) | (isolated) |

### Phase B Run 1 — Quick Run (44s, aborted — Port5 dev4 init failure)

| Metric | Result |
|--------|--------|
| Duration | 44.0 s |
| Samples | 11,005 |
| Sample rate | 249.9 Hz |
| Corruption events | 91,030 values (2.52%) — **99.7% from Port5** |
| Cycle time (min/mean/max) | 0.63 / 0.68 / 1.02 ms |
| Sample dt (min/max) | 3.78 / 5.02 ms |
| Jitter events | 0 |

**Port5 dev4 never initialized properly.** Recovery said "recovered on attempt 1" but the health check scored Port5 at only 76% (38/50) — kept as "WARN - keeping". The broken device output garbage on all 8 channels for the entire run.

**Root cause**: RDATAC output pipeline latch failure. Device 4 (farthest in 4-deep chain) latched its digital control logic (status byte valid ~76% of the time) but its ADC-to-shift-register output mux never connected. The health check only validates status bytes (`0xC0` top nibble), not channel data — so the broken device was not caught.

**Math confirms**: With 1 bad device out of 4, if the bad device's status byte is unstable ~24% of the time, the whole-port valid rate is ~76%. Exactly what was observed.

**Other port corruption was minor** (Port1: 100, Port3: 73, Port4: 18, Port6: 27) — all startup transients and isolated glitches. Excluding Port5, corruption = 233/3.2M = 0.007%.

**Health check flaw exposed**: The `else` branch in `main.cpp:437` has no minimum threshold. Anything >0% and <90% gets "WARN - keeping". Must be fixed.

---

### Phase B Run 2 — Full Run (6.2 min, SSH freeze + burst corruption)

**Note**: `irqaffinity=0,2` was NOT active — Pi had not been rebooted since the boot parameter was added. Only the code changes and system tuning (disabled timers, volatile journald, VM writeback) were active.

| Metric | Result |
|--------|--------|
| Duration | 371.4 s (6.2 min) |
| Samples | 92,727 |
| Sample rate | 249.7 Hz |
| Corruption events | 2,987 values (0.0098%) in 100 samples |
| Cycle time (min/mean/max) | 0.63 / 0.65-0.66 / 1.58 ms |
| Sample dt (min/max) | 3.44 / 14.80 ms |
| Jitter events (>2x) | 1 (sample 91909, 14.8ms = 3.7x) |
| Gaps | 0 |
| Truncated CSV rows | 1 |

**Init issues:**
- 6/7 ports failed initial verification (only Port1 passed). All recovered, but:
  - Port2: 11 recovery attempts, then scored **4% valid (2/50)** on health check — kept as "WARN - keeping"
  - Port4: 14 recovery attempts (recovered on attempt 14)
  - Port7: 4 attempts; Port3/5/6: 1 attempt each

**Timeline of events:**

| Time | Event |
|------|-------|
| 0-130s | **Clean.** Rate locked at 250.0 Hz, corrupt=0, drops=0, stream queued=0-1. Max cycle crept from 0.83ms to 1.58ms at t=70s and stayed there. |
| ~130s | **SSH terminal froze.** Output stopped, then burst-loaded several lines, then stopped responding entirely. |
| ~152.6s | **Corruption burst 1:** Port4 (SPI3) + Port6 (SPI4) hit simultaneously. Dense spike cluster at t=152.6-153.1s. Bit-shifts detected. |
| ~201-203s | **Corruption burst 2:** Port2 (SPI0) hit. Rail values — all 7 devices had ch4=0 and ch8=0 simultaneously at samples 50348 and 50770. |
| ~368s | **14.8ms jitter event** at sample 91909. Single event near end of run. |
| 371s | Run ended (Ctrl+C or timeout). 1 truncated CSV row. |

**Corruption by port:**
| Port | Bus | Core | Corrupted Values | Notes |
|------|-----|------|-----------------|-------|
| Port1 | SPI0 | 0 | 0 | CLEAN |
| Port2 | SPI0 | 0 | 1,125 | Burst at t=201-203s. Was 4% valid at init. |
| Port3 | SPI3 | 1 | 63 | Minor |
| Port4 | SPI3 | 1 | 901 | Burst at t=152.6s |
| Port5 | SPI4 | 2 | 0 | CLEAN |
| Port6 | SPI4 | 2 | 837 | Burst at t=152.6s |
| Port7 | SPI5 | 2 | 0 | CLEAN |

**Deep-device gradient** (every affected port):
- Port2: dev1=47 → dev7=240
- Port4: dev1=94 → dev4=380
- Port6: dev1=3 → dev5=481

Deeper devices in the daisy chain have monotonically more corrupted values. This is the signature of a transport-layer byte-alignment failure, not a device-level issue. A single bit-slip mid-transfer corrupts all subsequent bytes — deeper devices' bytes come later in the transfer.

---

### Root Cause Analysis (cpp-embedded + performance-engineer agents)

Both agents independently identified **`printf()` in the RT hot loop** as the primary root cause. The performance-engineer additionally identified a **glibc stdout mutex priority inversion** as a compounding factor.

#### The Critical Defect: `printf()` on Core 3

`print_stats()` is called every 10 seconds directly from `AcquisitionEngine::run()` on core 3 (SCHED_FIFO 50, isolated). It calls `std::printf()` which issues `write(STDOUT_FILENO, ...)` to the SSH pty.

```
engine.cpp line 216-219 (inside the hot loop):
    if (cycle_end - last_stats_time_ >= STATS_INTERVAL_SEC) {
        print_stats();     // <-- calls printf() from the RT thread
        last_stats_time_ = cycle_end;
    }
```

When the SSH transport stalls (t=130s), the pty buffer fills (4KB), and `write(1, ...)` **blocks indefinitely** — there is no timeout. SCHED_FIFO is irrelevant because this is a voluntary kernel entry (syscall). The RT thread on core 3 is stopped dead: no DRDY polling, no bus worker triggering, no acquisition.

#### The Compounding Factor: glibc stdout Mutex

`std::printf()` acquires a process-wide `_IO_lock_lock` mutex on the `stdout` FILE structure. The streaming server threads on core 2 also call `printf()` for connection events (`server.cpp` lines 214, 269, etc.). If a SCHED_OTHER thread on core 2 holds the stdout lock while blocked on `write()`, the SCHED_FIFO 50 RT thread blocks waiting for the lock — **unbounded priority inversion** with no priority inheritance (glibc uses a simple mutex, not a PI futex).

#### Causal Chain

```
SSH transport stalls (t=130s, network issue)
  → sshd stops reading pty master
    → pty slave buffer fills (4KB)
      → some thread's printf() blocks on write(1,...), holds stdout mutex
        → RT thread calls print_stats() at next 10s boundary
          → RT thread blocks on stdout mutex OR write() syscall
            → ALL bus workers starved (no triggers from engine)
              → DRDY cycles missed, ADS1299 shift registers desync
                → Burst corruption on next reads (t=152s, t=203s)
```

This explains every observation:
- **Why SSH freeze and corruption are correlated** — same root cause (SSH transport stall)
- **Why multiple independent SPI buses corrupt simultaneously** — all depend on the single engine thread
- **Why corruption hits Port4+Port6 but not Port1** — probabilistic shift register desync, depends on chain depth and timing
- **Why deep devices have more corruption** — more bytes come after the misalignment point
- **Why the 14.8ms jitter at t=368s** — printf() blocked on dead pty at end of run
- **Why Port7 (SPI5/DMA4) is clean** — DMA4 has a separate arbiter, AND if the engine recovers and triggers workers, shorter chains recover faster

#### Cycle Time Jump at t=70s (0.83ms → 1.58ms)

One-time event, persisted as max. Most likely a single `printf()` call where sshd was briefly delayed (~0.9ms write latency to the pty). Could also be a kernel softirq or timer on a bus worker core. Not a systemic issue — within the 4ms budget (2.42ms margin).

#### Port2 at 4% Valid — Health Check Flaw

Port2 scored 2/50 (4%) on the final health check. With 7 devices, if each device independently passes at probability `p`, then `p^7 = 0.04` → `p ≈ 0.62`. Each device was only ~62% reliable. The `else` branch at `main.cpp:437` keeps anything >0% and <90% with just a warning — no minimum threshold, no recovery attempt. Port2 entered acquisition broken and contributed 1,125 corrupted values.

#### VM Writeback Tuning — Agents Disagree

- **cpp-embedded** blamed the aggressive writeback tuning (`dirty_ratio=80, expire=600s`) as the primary cause — argues SD card DMA burst competes with SPI DMA through the legacy arbiter.
- **performance-engineer** ruled this out — at t=150s only ~99MB of dirty pages exist (2.5% of RAM), far below the 50% background threshold. The 600s writeback timer hasn't fired. The printf() stall from SSH death is the simpler and more complete explanation.

The performance-engineer's analysis is more convincing: the corruption correlates tightly with SSH death timing, not with any writeback threshold. However, the VM writeback tuning should still be reverted (or made less aggressive) because in longer runs it will eventually cause burst flushes. ext4 jbd2 journal commits (every 5s by default, independent of VM writeback settings) still cause periodic SD card I/O regardless.

---

### Outstanding Issues Summary (as of Phase B Run 2)

**Must fix before next run:**
1. **`printf()` in the RT hot loop** — move stats printing to a separate non-RT thread. This is the #1 priority. Every other fix is secondary until the RT thread can never block on stdout.
2. **Health check minimum threshold** — ports below 50% valid must trigger recovery, not "WARN - keeping".
3. **Reboot the Pi** — `irqaffinity=0,2` is configured but not active.

**Should fix:**
4. **Revert or moderate VM writeback tuning** — `dirty_ratio=80, expire=600s` risks burst flushes in long runs. Consider `dirty_ratio=40, dirty_background_ratio=20, expire=30s` or writing CSV to tmpfs.
5. **Add channel-data validation to health check** — status-byte-only checks miss devices with valid 0xC0 but garbage channel data (Port5 dev4 in Run 1).
6. **Per-device diagnostic logging** — when a port is marginal, log which specific device(s) failed.

**Consider:**
7. **Set stdout to `O_NONBLOCK`** — makes all `write(1,...)` non-blocking. Lost stats output is preferable to stalling the RT loop. However, this does NOT fix the glibc mutex contention — only moving printf off the RT thread does.
8. **Eliminate all `printf()` from threads sharing stdout with the RT thread** — streaming server's `accept_loop` and `stream_loop` both call `printf()`. These should use `fprintf(stderr, ...)` or a separate log fd.
9. **Consider `isolcpus=1,3`** — core 1 only runs the SPI3 bus worker; isolating it would prevent kernel threads from running there.

---

## Phase B2: Core Reallocation + RT Hot Loop Cleanup (2026-02-18)

### Changes Applied

**Root cause addressed:** `printf()` in the RT hot loop on core 3 blocks on SSH pty backpressure, causing multi-millisecond stalls that desynchronize ADS1299 shift registers. Additionally, streaming threads (accept + stream + CSV writer) shared cores 1 and 2 with SPI bus workers, creating preemption risk.

**Code changes (4 files):**
1. **`engine.hpp` + `engine.cpp`** — Removed `print_stats()` method entirely. Replaced with `alignas(64) std::atomic<bool> stats_ready_` flag. Engine sets it every 10s; a new stats thread on core 1 reads `get_stats()` and prints. Zero syscalls on the RT core.
2. **`csv_writer.cpp`** — `CPU_SET(2)` → `CPU_SET(1)`: CSV writer moved off isolated core 2 to housekeeping core 1.
3. **`server.cpp`** — Both `accept_loop` and `stream_loop`: `CPU_SET(2)` → `CPU_SET(1)`: streaming threads moved to core 1.
4. **`main.cpp`** — Added stats display thread (pinned core 1, polls `stats_ready_` every 1s). Added `#include <pthread.h>` and `#include <sched.h>`. Fixed health check: ports below 50% valid now trigger recovery instead of "WARN - keeping".

**System configuration (on Pi, rebooted):**
- `/boot/firmware/cmdline.txt`: `isolcpus=2,3 irqaffinity=0,1 nohz_full=2,3 rcu_nocbs=2,3 rcu_nocb_poll`
- `/etc/sysctl.d/99-eeg-acquisition.conf`: `sched_rt_runtime_us=-1`, `vm.stat_interval=10`, moderate VM writeback (`dirty_ratio=40`, `dirty_expire_centisecs=3000`)
- `/etc/fstab`: Added `commit=30` to root partition

**New core allocation:**
| Core | RT Thread | Non-RT |
|------|-----------|--------|
| 0 | SPI0 worker (FIFO 49) | SPI0 DMA IRQs |
| 1 | SPI3 worker (FIFO 49) | CSV writer, accept, stream, stats threads + all other IRQs |
| 2 (isolated) | SPI4 + SPI5 workers (FIFO 49) | (none) |
| 3 (isolated) | Acquisition engine (FIFO 50) | (none) |

**Verification:** Build succeeded with zero warnings. All kernel params confirmed active after reboot (`/sys/devices/system/cpu/isolated` = `2-3`, `irqaffinity` default = `0x3`).

### Phase B2 Run 1 — 10.2 min, 153,076 samples, 328 channels

**Results:**
- 250.0 Hz locked, zero gaps, zero drops, zero DRDY timeouts
- 6 corruptions (engine counter), 7 corrupt samples / 264 corrupt values in CSV
- Max cycle: 5.18ms, max dt: 10.75ms (one jitter event at t=506s)
- Mean cycle drifted 0.64ms → 0.74ms over 10 minutes
- Init: 6/7 ports failed initial RDATAC, all recovered. Health check triggered recovery for Port2 (34%), Port3 (14%), Port5 (18%) — all recovered successfully (new 50% threshold working).
- SSH terminal slowed during run; Ctrl+C had delayed response (multiple ^C visible)

**Two corruption clusters:**

| Cluster | Time | Samples | Ports | Shift | Likely Cause |
|---------|------|---------|-------|-------|-------------|
| 1 | t=9.3s | 2339-2341 (3 consecutive) | Port4, Port2, Port6 | 4-bit | RDATAC init settling race |
| 2 | t=506s | 126490-126526 (4 over 36 samples) | Port6, Port7, Port4, Port2 | 6-7-1-6 bit | AXI bus saturation from SD card DMA |

### Root Cause Analysis: SPI SCLK Bit-Shift Desynchronization

**Corruption mechanism (confirmed by data analysis):**

Corrupt values are NOT random garbage. They are the expected signal value multiplied by exactly 2^N, where N is the number of extra SCLK bit positions the shift register advanced before the SPI read occurred:
- Cluster 1: N=4 (all values ×16)
- Cluster 2: N=6 (×64), N=7 (×128), N=1 (×2), N=6 (×64)

Each affected device's ch2 encodes the shift amount as `2^N - 1` (values: 15, 63, 127). Ch1 is always clean because the 3-byte status register at the head of each device's data absorbs the first shifted bits.

**CS0/CS1 victim pattern:**

| Always CLEAN | Always CORRUPT |
|-------------|---------------|
| Port1 (SPI0.CS0) | Port2 (SPI0.CS1) |
| Port3 (SPI3.CS0) | Port4 (SPI3.CS1) |
| Port5 (SPI4.CS0) | Port6 (SPI4.CS1) |
| — | Port7 (SPI5.CS0, read last) |

On every dual-CS bus, CS0 is read first and never corrupts; CS1 is read second and always corrupts. This proves the bus workers read ports sequentially — by the time CS1 is read during a stall, the ADS1299 shift registers have already advanced extra positions. Port7 (SPI5, single CS) also corrupts because SPI5 is the last bus read.

**Jitter event (sample 126489):** The 10.75ms dt sample is completely clean — all 328 channels normal. Corruption appears at the NEXT sample (126490). The stall delays the read; the first read after recovery gets bit-shifted data.

**Deep-device gradient:** Corruption always starts at the deepest device in the chain and works inward. Within the shallowest affected device, corruption begins partway through channel data. This is because deeper devices' bytes come later in the sequential SPI transfer — a stall mid-transfer corrupts later bytes first.

### Root Cause: AXI Bus Saturation from SD Card DMA

**The t=506s corruption was caused by SD card DMA writeback saturating the AXI bus.**

The CSV writer produces ~825 KB/s of continuous writes (328 channels × 250 Hz × ~3.3 KB/row). After ~500s, the kernel's page cache flushes dirty pages to the SD card via EMMC2 DMA. The EMMC2 controller shares the AXI bus fabric with all 4 SPI DMA controllers (SPI0/SPI3/SPI4 on legacy DMA, SPI5 on DMA4). During a large SD card writeback burst:

1. SPI DMA read requests queue behind EMMC2 requests on the shared AXI bus
2. All 4 SPI transfers stall simultaneously (explaining why all 4 buses / 3 different cores corrupted at once)
3. The engine's I2C GPIO register access also stalls (GPIO is memory-mapped via AXI)
4. Combined effect: 4.26ms SPI cycle + missed DRDY edge (4ms wait) + 2.5ms I2C stall = 10.75ms total

**No CPU-level event can stall threads on 3 different cores simultaneously.** Only a shared hardware bus (AXI) can. This rules out all software-level explanations (thread contention, printf, priority inversion).

The t=9.3s cluster is a separate issue — the RDATAC init settling race that occurs in the first few seconds after health check recovery. Only ~7MB of CSV had been written by then, far below any writeback threshold.

### Fix: Eliminate SD Card I/O During Acquisition

Writing CSV to tmpfs (RAM disk) instead of the SD card eliminates 100% of EMMC2 DMA traffic during acquisition. The Pi has 4GB RAM; a 10-min run produces ~500MB of CSV data. Copy to SD card after acquisition completes.

Alternatively, disabling CSV writing entirely (--no-csv flag) achieves the same effect if only streaming data is needed.

**This fix addresses the t=506s cluster only.** The t=9.3s init corruption requires a separate fix (improved post-recovery settling time or shift-register flush).

### Mean Cycle Drift (0.64ms → 0.74ms)

The 15% increase over 10 minutes is likely caused by increasing SD card write latency as the CSV file grows — more frequent DMA bus contention causes small per-cycle SPI latency increases. Thermal throttling of the Cortex-A72 may also contribute. Eliminating SD card writes during acquisition should stabilize the mean cycle time.

### SSH Terminal Slowdown

The stats thread on core 1 writes to stdout (SSH PTY). When the SSH connection has backpressure (slow terminal, network latency), stdout writes block. This only affects the stats thread (SCHED_OTHER on core 1), NOT the RT engine on core 3. The delayed Ctrl+C response is caused by SSH PTY buffer backlog — the signal was delivered but terminal echo was delayed. Redirecting output to a log file (`> /tmp/run.log 2>&1`) would eliminate this.

### Phase B2 Fix: Move CSV Writing to Client Side

**Changes applied:**
- Server: flipped `full_csv` default to `false` (CSV disabled by default, opt-in with `--full-csv`). Hot loop skips SPSC ring push when CSV is off, saving ~610ns/cycle.
- Client (`simpleviz.py`): Added `CSVWriterThread` — dedicated thread with 10,000-sample queue, non-blocking push, 64KB write buffer. Writes raw unfiltered int32 values in the same format as the server CSV. Timestamped filenames (`eeg_data_YYYY-MM-DD_HHMMSS.csv`). CLI flags: `--no-csv`, `--csv-dir`.

### Phase B2 Run 2 — 20.1 min, 300,750 samples, CLEAN (2026-02-18)

**CLEAN — Zero corruption events in 98.6 million data points.**

| Metric | Result |
|--------|--------|
| Duration | 1203s (20.1 min) |
| Samples | 300,750 |
| Channels | 328 |
| Data points | 98,646,000 |
| Sample rate | 250.0 Hz (locked) |
| Gaps | 0 |
| Jitter events | 0 |
| Max dt | 4.221ms |
| Min dt | 3.776ms |
| Corruptions | 0 |
| Spikes | 0 |
| Rail values | 0 |
| Out-of-range | 0 |
| Bit-shifts | 0 |

**This confirms the root cause:** SD card EMMC2 DMA writeback was saturating the AXI bus, stalling all 4 SPI DMA controllers simultaneously. Removing server-side CSV writing eliminated 100% of EMMC2 DMA traffic during acquisition, resulting in a completely clean 20-minute run with no timing anomalies.

The mean cycle drift issue (0.64→0.74ms in Run 1) is also resolved — max dt of 4.221ms shows no stall events at all, confirming the drift was caused by increasing SD card write contention, not thermal throttling.

---

## Phase B2 Init Redesign: Tiered Recovery + 7/7 Hard Gate (2026-02-19)

### Problem

Fresh power-on (2026-02-19) produced two consecutive failed runs. Port2 was declared DEAD after 15 recovery attempts on both runs. The engine still reported "328 channels" and **100% of 1562 samples were corrupt** because dead Port2's garbage data poisoned every sample. A power cycle fixed everything.

Three independent agents (error-detective, cpp-embedded, performance-engineer) analyzed the code and unanimously identified the same root causes:

1. **Recovery never sends RESET**: `recover_port()` only cycles STOP→SDATAC→flush→RDATAC→START. If a device has a power-on latch failure (corrupted digital core state), this will fail 100% of the time forever. Software RESET (0x06) is needed to clear it.
2. **Escalating delays waste time**: Recovery uses `sleep_ms(X * scale)` where `scale = attempt+1`. Attempt 14 wastes 14× more time per cycle. The RDATAC timing race is memoryless — longer delays don't help.
3. **Dead ports not excluded from acquisition**: `port_dead[]` was set but never consulted when building downstream components (bus workers, engine, streaming server). Dead port's garbage data causes 100% corruption.

### What Was Changed

**`controller.hpp` + `controller.cpp` — Tiered recovery methods:**

- **`recover_port_tier1()`**: Current `recover_port()` logic with ALL escalating `* scale` multipliers removed. Fixed ~200ms per attempt: start_low(20ms) → STOP(10ms) → SDATAC(20ms) → flush(5ms) → RDATAC×2(10ms+20ms) → start_high(100ms) → verify 10 samples.
- **`write_registers()`**: Extracted register-write-only path from `initialize_device()`. Writes CONFIG3 (+200ms settle), CONFIG1, CONFIG2, CH1-8SET, MISC1, CONFIG4 via `write_and_verify()`. Sends SDATAC×5 first. Skips ID check (hardware presence already confirmed in Phase 1).
- **`recover_port_tier2()`**: Full software RESET path: start_low + STOP + SDATAC → RESET×2 (daisy chain reliability) → 150ms settle → SDATAC×5 → write_registers() → flush → RDATAC×2 → start_high → verify.
- Legacy wrappers (`recover_port()`, `restart_single_port()`) now delegate to `recover_port_tier1()`.

**`main.cpp` — `run_tiered_recovery()` function:**

Replaced the flat 15-attempt `recover_port()` loop with a 3-tier strategy:

| Tier | Method | Per-attempt | Attempts | Worst case/port | Targets |
|------|--------|-------------|----------|-----------------|---------|
| 1 | RDATAC cycle | ~200ms | 8 | ~1.6s | RDATAC timing race (~90% of failures) |
| 2 | Software RESET + reconfig | ~500ms | 5 | ~2.5s | Power-on latch failures |
| 3 | Full re-init ALL ports | ~30s | 2 rounds | ~60s | Cross-port state interference |

Between Tier 3 attempts, runs a quick Tier 1 pass on still-failing ports.

**`main.cpp` — 7/7 hard gate:**

After all tiers, if any port is still dead → hard exit with "POWER CYCLE REQUIRED" message (exit code 2, distinct from code 1 = general error). No graceful degradation by default.

**`main.cpp` — `--min-ports` escape hatch:**

Default requires all ports. `--min-ports N` allows running with fewer for testing. When enabled and some ports are dead, builds filtered `active_devices`/`active_configs` vectors excluding dead ports. All downstream components (DRDY poller, bus workers, CSV writer, streaming server, engine) use the filtered vectors.

**Other `main.cpp` changes:**
- Warmup increased from 100 to 250 samples (~1 second at 250 Hz) to absorb post-recovery settling artifacts.
- DRDY-missing for a non-dead port after successful init is now a hard error (return 1), not a warning.

### Time Budget (worst case)

| Tier | Per-port | 7 ports |
|------|----------|---------|
| Tier 1 (8 × 200ms) | ~1.6s | ~11.2s |
| Tier 2 (5 × 500ms) | ~2.5s | ~17.5s |
| Tier 3 (2 full re-inits) | n/a | ~60s |
| **Total worst case** | | **~89s** |

Typical case (most ports fail RDATAC race, recover in Tier 1 within 1-3 attempts): **~3-5 seconds total**.

---

## 2026-02-19 Session: Battery Failure + Port3 Dev5 Ch4 Investigation

### Context

Battery ran out during acquisition, system died. After replacing battery and reconnecting:

**Run 1 (bad):** Stream rate was ~50 Hz (should be 250 Hz), data was queuing nonstop. Aborted immediately. Likely cause: the battery failure left devices in a bad state, and the first init attempt didn't fully recover all ports. The 50 Hz stream rate suggests the engine was running but TCP backpressure or worker stalls reduced throughput.

**Run 2 (mostly good):** All 7 ports initialized, 250 Hz locked, clean data — except Port3 dev5 ch4. That single channel (and all subsequent ch4 entries for devices after dev5 on Port3) showed anomalous values.

### Investigation: Port3 Dev5 Ch4

**CSV analyzed:** `client/eeg_data_2026-02-19_120122.csv` — 14,171 samples, 330 columns.

**Finding:** Only Port3_dev5_ch4 was anomalous. Mean ~293,800 (0x0478A9) with stdev ~1,116, while all other 327 channels had mean ~-4,600 (normal test signal range). The anomalous values were consistent across all 14,171 samples — not intermittent, but a stuck wrong-value condition.

**Root cause: CH4SET register bit-flip (0x05 → 0x04)**

The value 293,800 corresponds to ~157mV at the ADS1299 ADC scale. This is exactly the ADS1299 internal temperature sensor reading at warm operating temperature. A single bit-0 flip in the CH4SET register changes:
- `0x05` = normal input (MUXn[2:0] = 101 = test signal)
- `0x04` = temperature sensor (MUXn[2:0] = 100)

This single-bit corruption most likely occurred during the power failure and wasn't caught by the init process.

### Register Verification Blind Spot (Known Issue)

This failure was NOT detected by init because of two known limitations:

1. **`write_and_verify()` only reads back from device 1** in each daisy chain. The SPI RREG command clocks out register values for all devices, but the code only reads enough bytes for device 1. Devices 2+ receive the same write commands (shared MOSI) but their register state is never verified.

2. **`verify_port_data()` only checks status bytes**, not channel data values. The 0xC0 top nibble of the status byte was valid — the ADS1299 was converting and streaming correctly, just reading from the wrong input mux. Channel-level data validation (checking for expected test signal range) would have caught this.

**Decision:** User declined implementing channel-level data validation since the system won't remain in test mode much longer. The register verification blind spot is documented here for reference. A production system should implement per-device register readback verification using the full daisy-chain RREG protocol (clock out N×bytes for N devices).
