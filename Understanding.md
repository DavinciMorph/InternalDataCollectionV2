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
SPI objects already constructed sequentially. Only `initialize_device()` calls parallelized (pure SPI, no I2C). START pins pre-set LOW by `force_all_start_pins_low()`.

| Run | Samples | Corrupt | Result | Notes |
|-----|---------|---------|--------|-------|
| 1   |         |         |        |       |
| 2   |         |         |        |       |

---

### Final Validation (after all A/B tests)
| Run | Duration | Samples | Corrupt | Init Time | Result |
|-----|----------|---------|---------|-----------|--------|
| 1   | 5 min    |         |         |           |        |
| 2   | 5 min    |         |         |           |        |
| 3   | 5 min    |         |         |           |        |
| 4   | 5 min    |         |         |           |        |
| 5   | 5 min    |         |         |           |        |
| 6   | 20 min   |         |         |           |        |
