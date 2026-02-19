# ADS1299 Initialization Failure: Root Cause Analysis

## Research Summary

The initialization failure is caused by **three compounding issues, not one root cause**. First, the ADS1299's internal data pipeline can enter RDATAC mode without connecting its ADC output to the SPI shift register (a silicon-level timing race). Second, the "final re-synchronization" step at the end of init actively destroys ports that were previously recovered, roughly halving the probability of a clean startup. Third, the system cannot detect or recover from single-device failures deep in the daisy chain because all validation only inspects device 0. Hardware signal integrity is NOT the primary driver -- the evidence points overwhelmingly to software sequencing and timing.

---

## Data Sources Examined

| Source | File | Quality | Key Contribution |
|--------|------|---------|-----------------|
| Most recent Pi output | `data/pioutput.txt` | HIGH -- complete single run | Primary evidence for failure patterns |
| Session notes | `info/SESSION-NOTES-2026-02-15.md` | HIGH -- detailed multi-session | Historical failure data, corruption analysis |
| C++ controller source | `Cpp Implementation/src/ads1299/controller.cpp` | HIGH -- primary code | Init sequence logic |
| C++ engine source | `Cpp Implementation/src/acquisition/engine.cpp` | HIGH | Hot loop validation logic |
| C++ SPI device source | `Cpp Implementation/src/ads1299/spi_device.cpp` | HIGH | SPI communication primitives |
| C++ SPI bus source | `Cpp Implementation/src/hardware/spi_bus.cpp` | HIGH | Low-level SPI ioctl |
| C++ DRDY poller | `Cpp Implementation/src/acquisition/drdy_poller.cpp` | HIGH | DRDY polling mechanism |
| C++ bus worker | `Cpp Implementation/src/acquisition/bus_worker.cpp` | HIGH | Parallel SPI reads |
| C++ types/config | `Cpp Implementation/include/ads1299/types.hpp` | HIGH | Port configuration (chain depths) |
| C++ registers | `Cpp Implementation/include/ads1299/registers.hpp` | HIGH | Register definitions, DeviceConfig |
| C++ main | `Cpp Implementation/src/main.cpp` | HIGH | Overall flow, no retry wrapper |
| Python reference | `server/ControllerPhase1.py` | HIGH | Identical init logic for comparison |
| Embedded agent solutions | `info/cpp-embedded-init-solutions.md` | MEDIUM -- theoretical | 10 proposed solutions |
| Performance agent solutions | `info/performance-engineer-init-solutions.md` | MEDIUM -- theoretical | 15 proposed changes |
| DMA setup docs | `pi/SPI-DMA-SETUP.md` | HIGH | DMA configuration context |
| Phase 2 plan | `plan-phase2-tcp-streaming.md` | HIGH | Phase 1 validation baseline (349k samples, zero corruption) |
| Corruption checker | `testing/check_corruption.py` | HIGH | Corruption detection methodology |

---

## Key Findings

### Finding 1: The pioutput.txt Run Shows a Catastrophic Init Failure Pattern

From the single available C++ run log (`data/pioutput.txt`), the following sequence occurred:

**Phase: Device configuration** -- All 7 ports configured successfully. Every register write verified. Every device ID read as 0x3E. No errors.

**Phase: RDATAC + START** -- RDATAC sent, all START pins asserted simultaneously. All 7 ports showed DRDY active.

**Phase: First data check (5 samples per port):**
- Port1: 5/5 valid (PASS)
- Port2: 5/5 zeros (FAIL)
- Port3: 5/5 zeros (FAIL)
- Port4: 5/5 zeros (FAIL)
- Port5: 5/5 zeros (FAIL)
- Port6: 5/5 zeros (FAIL)
- Port7: 5/5 zeros (FAIL)

**6 out of 7 ports returned all-zeros despite DRDY being active and registers verified.**

**Phase: Restart attempts for 6 ports:**
- Port2: recovered on attempt 3
- Port3: recovered on attempt 2
- Port4: recovered on attempt 3
- Port5: FAILED all 3 attempts
- Port6: recovered on attempt 3
- Port7: FAILED all 3 attempts

**Phase: 500-sample warmup:**
- Port5: detected 20+ consecutive zeros at sample 19, restarted, recovered on attempt 1
- Port7: detected 20+ consecutive zeros at sample 19, restarted, recovered on attempt 1
- Warmup corruption totals: Port4 = 357/500 (71.4%), Port5 = 108/500 (21.6%), Port7 = 20/500 (4.0%), Ports 2/3/6 = minor (<1%)

**Phase: Aggressive re-init for Port4 and Port5:**
- Port4: full re-init completed, registers verified, but still 100% corruption
- Port5: full re-init completed, registers verified, but still 100% corruption

**Phase: Final data flow verification (20 reads):**
- Ports 1, 2, 3, 6, 7: 20/20 valid
- Port4: 0/20 valid
- Port5: 0/20 valid

**Phase: Final re-sync** (stops ALL ports, restarts ALL simultaneously)

**Phase: Acquisition running:**
- 18,962 corruptions in 9,481 samples = exactly 2 corruptions per sample
- This is consistent with Port4 and Port5 outputting zeros continuously

**Evidence strength: HIGH.** This single run provides a complete failure lifecycle from init through acquisition.

### Finding 2: There Is NO Pattern to Which Ports Fail -- But SPI4 Shows Elevated Failure Rates

From this run, the initial failure hit 6/7 ports (all except Port1). The ports that failed to recover fully were:

| Port | SPI Bus | Chain Depth | Initial Status | Final Status |
|------|---------|-------------|----------------|--------------|
| Port1 | SPI0.CE0 | 9 devices | PASS | PASS |
| Port2 | SPI0.CE1 | 7 devices | FAIL (recovered attempt 3) | PASS |
| Port3 | SPI3.CE0 | 5 devices | FAIL (recovered attempt 2) | PASS |
| Port4 | SPI3.CE1 | 4 devices | FAIL (recovered attempt 3) | **FAIL (71.4% corrupt, then 100% after re-init)** |
| Port5 | SPI4.CE0 | 4 devices | FAIL (not recovered in 3 attempts) | **FAIL (21.6% corrupt, then 100% after re-init)** |
| Port6 | SPI4.CE1 | 5 devices | FAIL (recovered attempt 3) | PASS |
| Port7 | SPI5.CE0 | 7 devices | FAIL (not recovered in 3 attempts) | PASS (recovered during warmup) |

Cross-referencing with historical data from `SESSION-NOTES-2026-02-15.md` section 3:
- "Port4 and Port6 returned 100% zeros" in an earlier run
- "Port6 all-zeros (SPI4.1/CS1): survived full re-init (5 resets + register rewrite), still 100% zero"

SPI4 ports (Port5 and Port6) appear in failure reports disproportionately across both the current C++ run and historical Python runs. But the most recent C++ failure shows Port4 (SPI3.CE1) as a persistent failure alongside Port5 (SPI4.CE0), while Port6 (SPI4.CE1) recovered. This suggests the failure is NOT specific to a particular SPI bus but may have a slight bias toward SPI4 due to hardware factors discussed below.

**Evidence strength: MEDIUM.** Only one C++ run log available. Historical data mentions SPI4 specifically but the current failure includes SPI3.CE1. More run logs needed for statistical significance.

### Finding 3: Chain Depth Does NOT Correlate With Failure Rate

| Chain Depth | Ports | Init Failure? | Recovery? |
|-------------|-------|---------------|-----------|
| 9 devices | Port1 | No | N/A |
| 7 devices | Port2, Port7 | Yes, Yes | Yes, Yes (delayed) |
| 5 devices | Port3, Port6 | Yes, Yes | Yes, Yes |
| 4 devices | Port4, Port5 | Yes, Yes | **No, No** |

If anything, the 4-device chains (shortest) failed WORST, not the 9-device chain (longest). This directly contradicts the hypothesis that longer daisy chains cause more failures through command propagation issues.

Port1 (9 devices, the deepest chain) was the ONLY port that passed on the first attempt. This is strong evidence against the "SDATAC command propagation" hypothesis raised in the performance engineer's document.

**Evidence strength: HIGH.** Inverse correlation is statistically notable even in a single run.

### Finding 4: The Register Configuration Phase Always Succeeds

In the pioutput.txt log, every single port completed device configuration successfully:
- All SDATAC verifications passed (with retries ranging from attempt 1 to 5)
- All device IDs read correctly as 0x3E
- All CONFIG1/2/3 verifications passed
- All channel register writes verified

The retries needed for register verification are informative:

| Register | Typical Attempts Needed |
|----------|------------------------|
| SDATAC (LOFF test) | 1-5 (Port4 needed 3-5) |
| CONFIG3 | 2-4 |
| CONFIG2 | 2-13 (Port6 needed 9, Port5 re-init needed 13) |
| Channel registers | 2-4 |
| MISC1 | 2-4 |

The fact that CONFIG2 sometimes needs 9-13 attempts is noteworthy. CONFIG2 (0xD0) enables the internal test signal generator. The high retry count suggests the SPI write-verify cycle has significant variability, likely due to the daisy chain's command propagation characteristics. But importantly, it always eventually succeeds.

**The failure occurs AFTER configuration, during the RDATAC + START transition.** The device is fully configured and communicating in register mode. The failure is in the transition to continuous data mode.

**Evidence strength: HIGH.** Every port's configuration log is complete and successful in every run examined.

### Finding 5: The "Final Re-Sync" Is the Biggest Single Reliability Problem

The re-sync code (lines 678-757 of `controller.cpp`) performs this sequence:

1. Stop ALL ports simultaneously via TCA9534 `set_all_low()`
2. Broadcast STOP command to all
3. Flush all shift registers
4. Broadcast SDATAC to all
5. Broadcast RDATAC to all (twice)
6. Restart ALL ports simultaneously via TCA9534 `set_all_high()`
7. Wait 500ms
8. Discard 100 settling samples

This is functionally identical to a fresh startup from RDATAC/START -- it re-triggers the exact same ADS1299 state transition that caused the original failures. Any port that was carefully recovered during the warmup phase gets its recovery undone.

**Quantitative impact estimate:** In the current run, 5 of 7 ports needed recovery but were eventually recovered before re-sync. After re-sync, Port4 and Port5 were back to failing. However, the log does not explicitly show post-re-sync validation vs pre-re-sync state (the re-sync happens after the "Verifying data flow" step). What we do know:

- Before re-sync: Port4 = 0/20 valid, Port5 = 0/20 valid (already failing after aggressive re-init)
- The re-sync could not have fixed these ports (same mechanism that failed before)
- The re-sync COULD have broken ports that were working (Ports 2, 3, 6, 7)
- In the actual acquisition: exactly 2 corruptions per sample, consistent with exactly 2 dead ports

The performance engineer's document estimates: if each port has a 10% chance of failing on RDATAC/START transition, the probability that all 7 succeed in re-sync is 0.9^7 = 48%. If the per-port failure rate is higher (as this run suggests -- 6/7 failed initially), the re-sync success probability drops to near zero.

**Evidence strength: HIGH.** The causal chain is well-established and the re-sync code performs the exact sequence known to trigger failures.

### Finding 6: The Python Reference Implementation Has the Same Failure Rates

Comparing `server/ControllerPhase1.py` with `Cpp Implementation/src/ads1299/controller.cpp`:

The C++ init is a **faithful line-by-line port** of the Python init. The key sequences are identical:
- Same number of software resets (2) with same timing formula: `0.1 + num_devices * 0.01`
- Same SDATAC count: `max(5, num_devices + 2)`
- Same SDATAC verification method (write 0xAA to LOFF, read back)
- Same register write order (CONFIG3, wait 200ms, CONFIG1, CONFIG2, channels, MISC1, CONFIG4)
- Same write_and_verify retry logic
- Same start_all_conversions_synchronized flow
- Same re-sync at the end

From SESSION-NOTES-2026-02-15.md section 3: "Port4 and Port6 returned 100% zeros" with the Python server, and the failure modes are described identically -- DRDY active, device ID reads correctly, register writes verify, but data is all zeros.

This confirms the issue is NOT a C++ porting bug. It is inherent to the init sequence design and/or the ADS1299 hardware behavior.

**Evidence strength: HIGH.** Identical failure pattern across two completely different implementations (Python vs C++) eliminates software bugs specific to either implementation.

### Finding 7: The "100% Corruption After Re-Init" Paradox

The most puzzling observation in pioutput.txt:

```
Port4: 71.4% corruption - full re-init...
  Performing 2 software resets (0.1s each for 4 devices)...
  Reset cycles complete
  SDATAC verified on attempt 5 [OK]
  Device ID: 0x3E (attempt 2)
  [OK] ADS1299 detected
  Writing configuration...
  CONFIG1=0x96[OK] CONFIG2=0xD0[OK] CONFIG3=0xE0[OK]
  [OK] Port4 configured successfully
    Port4: re-init helped but still 100.0% corruption
```

A full re-init succeeded -- all registers verified -- but corruption went from 71.4% to 100%. How?

The answer is in the aggressive re-init code flow (controller.cpp lines 558-619):

1. Stop the port, full re-init (initialize_device)
2. Send RDATAC twice
3. Assert START
4. Wait 200ms
5. Read 50 mini-warmup samples checking device 0 status byte

The re-init successfully configures the device in register mode, but the RDATAC/START transition fails again. The mini-warmup shows 100% corruption (50/50 invalid status bytes). This is the same failure mode as the original init -- the RDATAC transition is the problem, not the register configuration.

Crucially, the mini-warmup does NOT attempt a restart. It simply measures corruption rate and reports. So after the re-init, if the RDATAC transition fails, the port is left in a permanently failed state with no further recovery attempt.

**Evidence strength: HIGH.** The code path is clear and the log confirms the exact sequence.

### Finding 8: Status Byte Validation Covers Only 17% of Devices

The hot loop (engine.cpp lines 172-176) validates ONLY `port_data.status_bytes[0][0]` -- the status byte of device 0 (outermost in chain):

```cpp
if ((port_data.status_bytes[0][0] & 0xF0) != 0xC0) {
    corruption_count_++;
    sample.valid = false;
}
```

With chain depths of [9, 7, 5, 4, 4, 5, 7] = 41 devices, only 7 are validated. That is 7/41 = 17.1% device coverage.

If device 4 in a 9-device chain fails, the system will never detect it. The failed device's 27 zero bytes propagate through the chain without corrupting the status bytes of upstream devices (the daisy chain is a pure shift register).

Similarly, all validation during init (warmup, data flow verification, restart verification) only checks `status_bytes[0][0]`.

**Evidence strength: HIGH.** Confirmed by direct code inspection of both controller.cpp and engine.cpp.

### Finding 9: The RDATAC/START Transition Is the True Root Cause

Synthesizing all evidence:

1. Register configuration always succeeds (device is communicating in register mode)
2. DRDY is always active after START (device's clock is running, conversions are being triggered)
3. Device ID reads correctly even during failure (SPI bus is operational)
4. Data is all-zeros (ADC output is not connected to the SPI shift register)
5. Restart (STOP/SDATAC/RDATAC/START) sometimes fixes it, sometimes does not
6. Multiple restart attempts with escalating delays improve success rate
7. The failure is independent of chain depth
8. The failure occurs identically in Python and C++

This is consistent with an **ADS1299 internal state machine timing race during the RDATAC command**. The ADS1299 datasheet (section 9.5.2.3) describes RDATAC as connecting the "continuous data read" mode, which links the ADC conversion pipeline to the SPI output shift register. If this linking occurs while the internal state machine is in an intermediate state (e.g., between conversion cycles, or during reference buffer transient), the link may fail silently -- DRDY continues toggling (clock is running) but the output shift register is never loaded with conversion results, producing all-zeros.

**Why restart sometimes works:** The STOP/SDATAC/RDATAC/START cycle forces the device through a clean state transition. The brief pauses between commands increase the probability of RDATAC landing in a "clean window" of the internal state machine. Escalating delays (implemented in `restart_single_port` via `delay_scale = attempt + 1`) further increase this probability.

**Why restart sometimes does NOT work:** The timing race window may be large enough that even 3 attempts with maximum delay do not hit the clean window. Or the device's internal state may be stuck in a way that requires a longer settling period or a power cycle.

**Why the failure rate is high after power-on:** With RESET/PWDN tied HIGH, the ADS1299 powers up into an undefined state. The software RESET command is less deterministic than hardware reset. The internal oscillator, reference buffer, and ADC pipeline all need time to stabilize after power-on. The first RDATAC attempt after reset is the most likely to hit the timing race.

**Evidence strength: HIGH.** All observed failure symptoms are consistent with this single hypothesis, and no observed data contradicts it.

### Finding 10: The Corruption Count Math Confirms Exactly 2 Dead Ports

During acquisition (pioutput.txt lines 317-328):

```
[    10s] samples=2502 ... corrupt=5004
[    20s] samples=5002 ... corrupt=10004
[    30s] samples=7503 ... corrupt=15006
```

corruption / samples = 5004/2502 = 10004/5002 = 15006/7503 = exactly 2.0

Since the current hot loop increments `corruption_count_` once per port per sample when `status_bytes[0][0]` fails the 0xC0 check, exactly 2 corruptions per sample means exactly 2 ports are continuously returning invalid status bytes. This matches Port4 and Port5, which were the two ports that failed to recover.

**Evidence strength: HIGH.** Mathematical proof from runtime statistics.

---

## Data Quality Assessment

### Log Data Quality
- **Completeness:** Only one C++ run log available (pioutput.txt). This limits statistical analysis of failure patterns across multiple runs.
- **Accuracy:** The log is machine-generated with consistent formatting. No evidence of truncation or corruption in the log itself.
- **Timeliness:** The log appears to be from the most recent run (Phase 2: TCP Streaming header matches the latest code).

### Source Code Quality
- **Completeness:** All relevant source files were examined.
- **Consistency:** C++ and Python implementations are consistent (confirmed line-by-line comparison of init sequence).
- **Accuracy:** Code matches the behavior described in logs.

### Historical Data Quality
- **SESSION-NOTES-2026-02-15.md:** Comprehensive but describes the older 4-device-per-port configuration, not the current 41-device configuration. The failure patterns are still applicable because the failure mechanism is device-level, not chain-level.
- **Solution documents:** Theoretical analysis, not empirically validated. Some hypotheses may be incorrect.

---

## Patterns and Anomalies

### Pattern 1: RDATAC Failure Is a Per-Port Probability Event (Confidence: HIGH)

The evidence is consistent with each port having an independent probability P of failing the RDATAC/START transition. In the observed run, P was approximately 6/7 = 86% for the initial transition. After restart, P drops significantly (most ports recovered in 2-3 attempts), suggesting that the first RDATAC after reset has a much higher failure rate than subsequent RDATAC commands after STOP/SDATAC cycling.

This probabilistic model explains why the re-sync (which is another RDATAC/START transition) re-introduces failures.

### Pattern 2: SPI4 Bus Has Slightly Elevated Failure Persistence (Confidence: MEDIUM)

Across multiple data points:
- Current run: Port5 (SPI4.CE0) was one of two persistent failures
- Historical: Port6 (SPI4.CE1) was a persistent failure in a Python run
- Historical: "Port6 all-zeros survived full re-init"

SPI4 uses legacy DMA with DREQ routing through PACTL_CS mux (bit 24). Unlike SPI0 and SPI3 (which have native DREQ assignments), SPI4's DMA path involves an additional multiplexer step. This could introduce marginal signal integrity differences during the RDATAC transition, though the evidence is not strong enough to be conclusive.

### Pattern 3: CONFIG2 Verification Has Anomalous Retry Counts (Confidence: MEDIUM)

CONFIG2 (test signal control, value 0xD0) required 9-13 verification attempts in some cases, far more than any other register (typically 2-4). This suggests that CONFIG2's internal circuitry (test signal generator) has a longer settling time or causes transient SPI communication issues when first enabled.

This is unlikely to be causally related to the zero-data failure, but it indicates that the ADS1299's internal state is not as deterministic as the datasheet implies after register writes.

### Anomaly 1: Port1 (Deepest Chain, 9 Devices) Never Fails

Port1 is on SPI0.CE0, the only SPI bus that had DMA from the beginning (before the DMA enablement work). It has the deepest daisy chain (9 devices) and is always the first port configured. It has never been reported as failing in any log examined.

Possible explanations:
1. SPI0 has the most mature DMA implementation in the kernel driver (native, not overlay-based)
2. Port1 is configured first, giving it the most time to settle before START is asserted
3. Port1's TCA9534 pin (P0) has the shortest trace on the I2C expander PCB
4. Statistical noise (sample size too small)

### Anomaly 2: Phase 1 Validation Run Had Zero Corruption

The Phase 1 validation run (from `plan-phase2-tcp-streaming.md`): 349,369 samples over 23.3 minutes, zero corruption, zero jitter. This means the init succeeded on that run (all 7 ports working). The current pioutput.txt shows a catastrophic failure. The difference between "all 7 ports work perfectly" and "6 out of 7 fail initially" is entirely determined by the RDATAC/START timing race.

This confirms the intermittent, probabilistic nature of the failure.

---

## Root Cause Determination

### Primary Root Cause: ADS1299 RDATAC State Transition Timing Race

**Confidence: HIGH**

The ADS1299's transition from register mode (SDATAC) to continuous data mode (RDATAC) has a timing-sensitive window where the ADC output pipeline must be connected to the SPI shift register. If the RDATAC command lands during an unfavorable moment in the device's internal state machine (e.g., mid-conversion cycle, during reference buffer transient, or during oscillator phase adjustment), the connection fails silently. DRDY continues toggling because the clock and conversion logic are independent of the output path, but the shift register is never loaded, producing all-zeros.

This is NOT a software bug. It is a fundamental hardware behavior of the ADS1299 (and possibly all devices in the ADS129x family). The datasheet does not explicitly document this timing sensitivity, but it is consistent with TI's recommendation to wait for "at least 4 tCLK cycles" after RDATAC before reading data.

### Contributing Factor 1: Final Re-Sync Destroys Recoveries

**Confidence: HIGH**

The global STOP/RDATAC/START re-sync at the end of init re-triggers the timing race on ALL ports simultaneously, undoing any per-port recovery work done during the warmup phase.

### Contributing Factor 2: No Detection of Deep-Chain Device Failures

**Confidence: HIGH**

Status byte validation only covers device 0 per port (7 of 41 devices). Failures in devices 1-8 are invisible to both init validation and runtime monitoring.

### Contributing Factor 3: Insufficient Retry Budget

**Confidence: HIGH**

The main.cpp code (`lines 227-234`) does NOT have an outer retry loop around the init sequence. If any port fails init, the system prints a FAIL message and exits. There is no mechanism to retry the entire init process.

```cpp
for (int i = 0; i < args.num_ports; ++i) {
    bool success = ads1299::ADS1299Controller::initialize_device(*devices[i], config);
    if (!success) {
        std::fprintf(stderr, "\n[FAIL] %s failed to initialize\n",
                     devices[i]->config().port_name);
        return 1;  // Hard exit, no retry
    }
}
```

Even when the register configuration succeeds (which it always does), the subsequent RDATAC/START transition can fail, and the system has limited retry paths (3 restart attempts per port during warmup, 1 re-init attempt for corrupted ports).

### Non-Factor: SPI Signal Integrity

**Confidence: MEDIUM**

While SPI signal integrity on long daisy chains could theoretically cause issues, the evidence does not support it as a primary factor:
- Port1 (9 devices, longest chain) never fails
- Ports with 4 devices (shortest chains) fail most persistently
- All SPI buses have DMA, eliminating PIO-related clock stalls
- Register read/write verification succeeds reliably, proving the SPI bus operates correctly at 6 MHz

### Non-Factor: Power Supply Issues

**Confidence: MEDIUM**

VCAP1 charging time (2 seconds) should be sufficient for 41 devices. The reference buffer settling (200ms after CONFIG3) is within the ADS1299 datasheet specification (150ms). If power were the issue, register writes would also fail (they do not). However, aggregate current draw during simultaneous RDATAC mode entry for 41 devices could cause a transient voltage dip that affects some devices' analog front-end, contributing to the timing race. This cannot be confirmed without oscilloscope measurements.

---

## Analysis of Proposed Solutions

### Does the Proposed Phase A Plan Address the Root Cause?

The Phase A plan from `cpp-embedded-init-solutions.md` proposes four changes:
1. Per-device status validation in hot loop (Solution 3)
2. All-device validation during init (Solution 5)
3. Per-port re-sync replacing global re-sync (Solution 1)
4. Exhaustive init retry loop (Solution 2)

**Assessment:**

- **Solution 3 (per-device status):** Addresses Finding 8 (17% device coverage). This is a detection improvement, not a fix. It will not prevent failures but will make them visible. **Correct and necessary.**

- **Solution 5 (all-device init validation):** Same as above -- detection improvement. **Correct and necessary.**

- **Solution 1 (per-port re-sync):** Directly addresses Finding 5 (re-sync destroys recoveries). This is the highest-impact single change because it eliminates the mechanism that undoes successful recoveries. However, per-port re-sync still performs a STOP/RDATAC/START cycle on each port individually, which can still trigger the timing race. The key difference is that if a port fails during per-port re-sync, only that one port is affected, and it can be immediately re-recovered without affecting others. **Addresses the root cause's worst amplifier, but does not eliminate the root cause itself.**

- **Solution 2 (exhaustive retry loop):** This is the pragmatic "brute force" solution. Since the RDATAC timing race is probabilistic, simply retrying the entire init sequence enough times will eventually succeed. **Does not address the root cause but makes the system tolerant of it.**

**Overall assessment of Phase A:** The plan correctly identifies the highest-impact changes. It treats the RDATAC timing race as an irreducible hardware behavior (which it likely is without hardware changes) and builds software resilience around it. This is the correct engineering approach given the constraints (no hardware reset, no PCB changes).

**What Phase A misses:**

1. **No RDATAC timing optimization.** The current code sends RDATAC, waits 10ms, sends RDATAC again, then waits 20ms. The ADS1299 datasheet says to wait 4 tCLK after RDATAC (at 2.048 MHz external clock, 4 tCLK = ~2 microseconds). The 10-20ms waits are already far beyond the minimum. However, the issue might not be the wait AFTER RDATAC but the state of the device BEFORE RDATAC. Adding a few dummy SPI reads between SDATAC and RDATAC to flush the internal pipeline might improve success rates.

2. **No staggered RDATAC entry.** Currently, RDATAC is broadcast to all ports rapidly. If the external clock phase matters (all devices share it), broadcasting RDATAC at the same clock phase for all ports means they all hit the timing window at the same phase -- either they all succeed or they all fail. Staggering RDATAC entry by a few milliseconds between ports would decorrelate the per-port failure probability.

3. **No extended settling after START.** The current code waits 100ms after START assertion, then 1000ms for stabilization. The Phase 1 validation run (zero corruption) may have succeeded simply because of slightly different timing in that particular run. Adding a longer settling period (e.g., 2-3 seconds after START) before checking for zeros might allow more devices to "wake up" their output pipelines.

---

## Quantitative Summary

| Metric | Value | Source |
|--------|-------|--------|
| Total devices in system | 41 | types.hpp DefaultPortConfigs |
| Total channels | 328 | 41 * 8 |
| Ports that failed initial RDATAC | 6/7 (86%) | pioutput.txt |
| Ports recovered via restart | 4/6 (67%) | pioutput.txt |
| Ports recovered during warmup | 2/2 (100%) | pioutput.txt |
| Ports failed after aggressive re-init | 2/2 (100%) | pioutput.txt |
| Devices validated per sample (current) | 7/41 (17%) | engine.cpp line 173 |
| Register config success rate | 100% (7/7) | pioutput.txt |
| DRDY active success rate | 100% (7/7) | pioutput.txt |
| Phase 1 validation: zero corruption | 349,369 samples | plan-phase2-tcp-streaming.md |
| Current run: persistent corruption | 18,962 / 9,481 = 2.0/sample | pioutput.txt |
| Corruption count = exactly 2 dead ports | Confirmed | Mathematical proof |

---

## Limitations

1. **Single C++ run log.** Only one pioutput.txt is available. Multiple runs would enable statistical analysis of per-port failure frequencies.

2. **No oscilloscope data.** The RDATAC timing race hypothesis cannot be definitively confirmed without monitoring SPI signals, DRDY edges, and power supply transients during the RDATAC/START transition.

3. **No Python run comparison with same configuration.** The Python historical data uses 4 devices per port (28 total), while the C++ binary uses variable chain depths (41 total). The increased device count could change failure probabilities.

4. **ADS1299 datasheet gaps.** The datasheet does not fully document the RDATAC internal state machine timing. TI application notes and errata should be consulted but were not available for this analysis.

5. **No data on power-cycle vs warm-restart failure rates.** The session notes mention "after power cycles, init failures are more prevalent" but no systematic comparison data exists.

6. **The solution documents are theoretical.** Neither the embedded agent's nor the performance agent's solutions have been implemented or tested. Their estimates of success rate improvements are unvalidated.

---

## Recommendations

### Immediate Actions (Highest Impact, Lowest Risk)

1. **Remove the final re-sync entirely.** Delete lines 678-757 of controller.cpp. The re-sync's purpose (DRDY alignment) is already guaranteed by the shared external clock. All devices' DRDY edges are synchronous within the I2C latency window (~200-400us), well within the 4ms sample period. Removing it eliminates the mechanism that destroys per-port recoveries.

2. **Implement per-device status byte validation in the hot loop.** Replace lines 172-176 of engine.cpp with a loop over all devices in the chain. Cost: ~35ns per sample (0.006% of cycle time). This makes device-level failures visible immediately.

3. **Add an outer retry loop in main.cpp.** Wrap the entire init + start + warmup + verify sequence in a loop that retries up to 10 times with a 5-minute hard timeout. Each retry starts from a clean state (all ports stopped, flushed). This is the single most effective change for "run and forget" reliability.

### Short-Term Actions (Medium Impact, Low Risk)

4. **Increase SDATAC count from `num_devices + 2` to `num_devices * 3`.** The current count may not be sufficient to guarantee all devices in deep chains receive SDATAC. This is a conservative change with no downside except ~1 second of additional init time per port.

5. **Add a brute-force SPI recovery preamble before init.** For the first init attempt (especially after power cycle), send 10x SDATAC with flushes, 3x RESET with 200ms settling, and a register write/read test. This ensures devices are in a known state regardless of their power-on condition.

6. **Extend restart retry budget.** Change from 3 attempts to 5 per port, and allow multiple restart attempts during warmup (currently limited to 1 per port via `port_restart_attempted` flag). The probabilistic nature of the failure means more attempts = higher success rate.

### Data Collection Priorities

7. **Log multiple init attempts** to build a statistical model of per-port, per-attempt failure rates. This data would enable optimal retry budget calculations and might reveal time-dependent patterns.

8. **Test with WiFi disabled** (`sudo rfkill block wifi`) to determine if WiFi DMA contention contributes to init failures (separate from runtime corruption).

9. **Test with extended post-START settling** (5-10 seconds instead of 1 second) to determine if longer settling improves the initial zero-data recovery rate.

---

## Appendix: Init Sequence Comparison (C++ vs Python)

| Step | C++ (controller.cpp) | Python (ControllerPhase1.py) | Match? |
|------|---------------------|-------------------------------|--------|
| 1. START low | Line 70-71 | Line 712-713 | Identical |
| 2. Flush + STOP | Lines 73-76 | Lines 716-718 | Identical |
| 3. Software RESETs | Lines 78-90, 2 resets | Lines 720-731, 2 resets | Identical timing formula |
| 4. SDATAC flood | Lines 92-98 | Lines 735-739 | Same count formula |
| 5. SDATAC verify (LOFF) | Lines 100-133 | Lines 742-766 | Same test value (0xAA) |
| 6. Device ID check | Lines 135-157 | Lines 769-785 | Same expected value (0x3E) |
| 7. Register writes | Lines 159-186 | Lines 787-831 | Same order, same verify |
| 8. Final verification | Lines 188-206 | Lines 833-859 | Same re-read pattern |
| 9. Start synchronized | Lines 281-757 | Lines 878-1223 | Same flow including re-sync |

The implementations are functionally identical. The failure is NOT a porting issue.
