# ADS1299 Init Reliability: Analysis and Proposed Solutions

## System Overview

- 7 SPI ports, variable daisy-chain depths (9,7,5,4,4,5,7 = 41 devices, 328 channels @ 250 Hz)
- 4 physical SPI buses: SPI0 (Port1+2), SPI3 (Port3+4), SPI4 (Port5+6), SPI5 (Port7)
- V2 board: RESET/PWDN tied HIGH (no hardware reset, software reset only)
- DRDY via TCA9534 I2C expander (0x20), START via TCA9534 (0x21)
- TCA9534 INT pin floating (no interrupt-driven DRDY)
- All devices on a port share MOSI: commands reach ALL devices simultaneously
- RREG in daisy chain only returns device 1 registers (devices 2-N not readable)
- `write_and_verify` only validates device 1

## Failure Mode Analysis

### Failure Mode 1: Whole-Port Zeros

**Symptoms:** An entire port returns all-zero bytes after initialization. DRDY is active, device ID reads correctly (0x3E), register writes verify successfully on device 1. But once RDATAC + START are asserted, data reads return zeros.

**Root Cause Analysis:** The ADS1299 has an internal state machine that governs transitions between register mode (SDATAC) and continuous data mode (RDATAC). The RDATAC command must be received by the device while it is in a clean state. If a device's internal data pipeline is stalled or its internal oscillator has not locked to the external clock, entering RDATAC produces a data-ready signal (DRDY toggles) but the output shift register is never loaded with conversion results.

The key evidence: registers verify fine (device is communicating), DRDY is active (clock is running, conversions are triggering), but data is all zeros (the output path from the ADC core to the SPI shift register is not connected). This is consistent with an ADS1299 entering RDATAC before its analog front-end has fully settled after reset.

**Why restart fixes it:** The STOP-SDATAC-RDATAC-START cycle forces the device through a clean state transition. The brief pause between commands gives the internal data pipeline time to re-link. Multiple restart attempts with escalating delays increase the probability of catching the right timing window.

### Failure Mode 2: Single-Device Zeros

**Symptoms:** One device deep in a daisy chain (e.g., dev4 of 4) outputs zeros while other devices on the same port work fine. The server cannot detect this because `write_and_verify` and status byte checks only inspect device 1.

**Root Cause Analysis:** In a daisy chain, all devices receive commands simultaneously on shared MOSI. But the SPI data read only clocks out device 1's status bytes for validation. Device N (the physically furthest from the master) may:

1. Have marginal signal integrity on the DOUT-to-DAISY_IN chain between devices
2. Have entered RDATAC slightly out of phase relative to device 1
3. Have a register write that did not stick (but this cannot be verified since RREG returns 0x00 for devices 2-N, as confirmed in Test 1)

The daisy chain protocol means device N's data is shifted through devices N-1, N-2, ... , 1 before reaching MISO. If device N's output is stuck at zero, the zeros propagate cleanly through the chain without corrupting other devices' data.

### Failure Mode 3: Re-Sync Breaking Recovered Ports

**Symptoms:** Ports that were individually recovered via `restart_single_port()` during warmup revert to all-zeros when the final re-synchronization step stops all ports simultaneously and restarts them.

**Root Cause Analysis:** The final re-sync calls `set_all_low()` on START, then broadcasts STOP, SDATAC, RDATAC, and finally `set_all_high()` on START. This is functionally identical to a fresh startup sequence. The problem is that whatever conditions caused the port to fail initially (e.g., internal timing race in the ADS1299) are re-triggered by this global stop-restart cycle. The individually-recovered port had been nursed through a successful state transition via carefully timed per-port restarts; the blanket stop-restart undoes this.

The re-sync was originally added because removing it exposed a ~23.4s periodic corruption pattern (likely WiFi scanning or kernel timer). The re-sync's flush-SDATAC-RDATAC-settling sequence provides resilience by giving the SPI shift registers a clean starting alignment.

### Failure Mode 4: Post-Power-Cycle Unreliability

**Symptoms:** After power cycles, init failures are more prevalent. Sometimes 2-3 binary restarts are needed.

**Root Cause Analysis:** With RESET/PWDN tied HIGH, the ADS1299 devices power up into an undefined state. The internal analog reference (CONFIG3 bit 7, PD_REFBUF) needs time to charge. The internal oscillator needs time to lock to the external clock. Software reset (RESET command) is less deterministic than hardware reset because it depends on the device being in a state where it can receive and process SPI commands. After a power cycle, the device may be in RDATAC mode (if it was running when power was cut), which means it ignores register commands until SDATAC is sent. But if the SPI bus itself was in an undefined state, even the SDATAC command may not be properly framed.

### Failure Mode 5: Periodic ~23.4s Corruption

**Symptoms:** When re-sync is removed, corruption occurs every ~5850 samples with high consistency. The re-sync's settling sequence provides resilience against this.

**Root Cause Analysis:** A periodic system event (WiFi scanning, kernel timer, cgroup maintenance) causes a brief CPU or bus contention event. The re-sync provides a clean SPI shift register alignment that gives the system enough tolerance to absorb the glitch. Without re-sync, a slight byte-level misalignment in the SPI stream means the periodic system event is enough to push the misalignment over the corruption threshold.

---

## Proposed Solutions

Solutions are ordered by impact (high to low) and annotated with implementation complexity.

---

### Solution 1: Per-Port Re-Sync (Replaces Global Re-Sync)

**Impact:** HIGH | **Complexity:** LOW | **Addresses:** Failure modes 1, 3, 5

**Problem it solves:** The global re-sync breaks recovered ports (FM3) but provides essential alignment resilience (FM5). A per-port re-sync preserves both benefits.

**Design:** After all warmup recoveries are complete, re-sync each port individually rather than stopping all ports simultaneously. This ensures that a port that was successfully recovered by `restart_single_port()` is never touched by the re-sync of another port.

**Pseudocode:**
```cpp
// Replace the global re-sync block (lines 678-754 in controller.cpp) with:

for (size_t i = 0; i < devices.size(); ++i) {
    auto* dev = devices[i];

    // Stop this port only
    dev->start_low();
    sleep_ms(50);
    dev->send_command(Cmd::STOP);
    sleep_ms(10);

    // Flush shift register
    dev->flush_spi();

    // Clean state transition
    dev->send_command(Cmd::SDATAC);
    sleep_ms(20);
    dev->send_command(Cmd::RDATAC);
    sleep_ms(10);
    dev->send_command(Cmd::RDATAC);  // redundant for reliability
    sleep_ms(20);

    // Restart this port
    dev->start_high();
    sleep_ms(100);

    // Verify this port still works
    if (dev->wait_for_drdy(0.1)) {
        PortData pd;
        dev->read_data(pd);
        uint8_t status0 = pd.status_bytes[0][0];
        if ((status0 & 0xF0) != 0xC0) {
            // Re-sync broke this port — use restart_single_port to recover
            for (int attempt = 0; attempt < 5; ++attempt) {
                if (restart_single_port(*dev, attempt)) {
                    break;
                }
            }
        }
    }
}

// Discard settling samples (per-port, no global stop needed)
// ... existing 100-sample settling loop ...
```

**Why this works:** Each port is independently cycled through STOP-SDATAC-RDATAC-START, getting the shift register alignment benefit. If the re-sync breaks a port, that port is immediately re-recovered without affecting others. The key insight is that ports do not need to be synchronized with each other because they all share an external clock source that keeps DRDY edges synchronous.

**Risk:** LOW. The external clock ensures DRDY synchrony regardless of per-port START timing. The only risk is that per-port re-sync takes longer (7x sequential vs 1x global), adding roughly 1.5 seconds to init.

---

### Solution 2: Exhaustive Init Retry Loop (Outer Wrapper)

**Impact:** HIGH | **Complexity:** LOW | **Addresses:** Failure modes 1, 2, 3, 4

**Problem it solves:** The current code attempts init once per port, with limited retries inside. If a port fails, the system logs a warning and continues. For a "run and forget" system, init must retry the entire sequence until all ports are confirmed working.

**Design:** Wrap the entire init + start + warmup + verify sequence in an outer retry loop that does not exit until all ports produce valid data, or a hard timeout (e.g., 5 minutes) is reached.

**Pseudocode:**
```cpp
bool all_ports_healthy(std::vector<ADS1299Device*>& devices, int num_reads = 50) {
    // Use longest-chain port as DRDY reference
    ADS1299Device* ref = devices[0];
    for (auto* d : devices) {
        if (d->config().num_devices > ref->config().num_devices)
            ref = d;
    }

    // Per-port valid counters — tracks ALL devices, not just device 0
    int valid[MAX_PORTS] = {};
    int reads_done = 0;

    for (int i = 0; i < num_reads; ++i) {
        if (!ref->wait_for_drdy(0.02)) continue;
        reads_done++;

        for (size_t p = 0; p < devices.size(); ++p) {
            PortData pd;
            devices[p]->read_data(pd);

            bool port_ok = true;
            for (int d = 0; d < devices[p]->config().num_devices; ++d) {
                // Check status byte for EACH device in the chain
                if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
                    port_ok = false;
                    break;
                }
                // Check for all-zero channel data (stuck device)
                bool all_zero = true;
                for (int c = 0; c < CHANNELS_PER_DEVICE; ++c) {
                    if (pd.channel_values[d][c] != 0) {
                        all_zero = false;
                        break;
                    }
                }
                if (all_zero) {
                    port_ok = false;
                    break;
                }
            }
            if (port_ok) valid[p]++;
        }
    }

    // Require >= 90% valid reads for every port
    for (size_t p = 0; p < devices.size(); ++p) {
        if (reads_done == 0 || valid[p] < reads_done * 9 / 10) {
            return false;
        }
    }
    return true;
}

// In main():
constexpr int MAX_FULL_RETRIES = 10;
constexpr double HARD_TIMEOUT_SEC = 300.0;  // 5 minutes

double init_start = clock_now();
bool system_ready = false;

for (int full_attempt = 0; full_attempt < MAX_FULL_RETRIES; ++full_attempt) {
    printf("\n=== INIT ATTEMPT %d/%d ===\n", full_attempt + 1, MAX_FULL_RETRIES);

    if (clock_now() - init_start > HARD_TIMEOUT_SEC) {
        fprintf(stderr, "FATAL: Init timeout after %.0fs\n", HARD_TIMEOUT_SEC);
        return 1;
    }

    // 1. Force all START low, flush all SPI
    force_all_start_pins_low(devices);
    for (auto* dev : devices) dev->flush_spi();
    sleep_ms(100);

    // 2. Initialize all devices
    bool init_ok = true;
    for (int i = 0; i < num_ports; ++i) {
        if (!initialize_device(*devices[i], config)) {
            init_ok = false;
            // Don't abort — continue configuring other ports
        }
    }

    // 3. Start conversions (with per-port re-sync — Solution 1)
    start_all_conversions_synchronized(devices, i2c, &config);

    // 4. Extended health check — validates ALL devices on ALL ports
    if (all_ports_healthy(devices, 100)) {
        printf("[OK] All %d ports healthy after attempt %d\n",
               num_ports, full_attempt + 1);
        system_ready = true;
        break;
    }

    printf("[RETRY] Not all ports healthy, restarting init...\n");

    // 5. Full shutdown before retry
    for (auto* dev : devices) {
        dev->start_low();
        dev->send_command(Cmd::STOP);
        dev->send_command(Cmd::SDATAC);
        dev->flush_spi();
    }
    sleep_sec(1.0);  // Allow devices to fully settle
}

if (!system_ready) {
    fprintf(stderr, "FATAL: Could not initialize all ports after %d attempts\n",
            MAX_FULL_RETRIES);
    return 1;
}
```

**Why this works:** Rather than hoping the first init succeeds, the system simply retries. Each retry starts from a clean state (all ports stopped, flushed). The probabilistic nature of the failure means multiple attempts will eventually succeed. Empirical data shows that 2-3 restarts typically suffice, so 10 retries with a 5-minute timeout provides ample margin.

**Critical detail:** The health check validates ALL devices on ALL ports (not just device 0), catching single-device zeros (FM2) that the current code misses entirely.

---

### Solution 3: Per-Device Status Validation in the Hot Loop

**Impact:** HIGH | **Complexity:** MEDIUM | **Addresses:** Failure mode 2

**Problem it solves:** The current hot loop only checks `status_bytes[0][0]` (device 0, outermost in chain). A device deep in the chain can output zeros indefinitely without the server ever detecting it.

**Design:** Validate status bytes for ALL devices in the chain, not just device 0. This is zero-cost in the hot loop because `parse_raw()` already extracts all status bytes; they just are not checked.

**Implementation in engine.cpp:**
```cpp
// Current code (line 172-176):
// Status byte validation (first device)
if ((port_data.status_bytes[0][0] & 0xF0) != 0xC0) {
    corruption_count_++;
    sample.valid = false;
}

// Proposed replacement:
for (int d = 0; d < num_dev; ++d) {
    if ((port_data.status_bytes[d][0] & 0xF0) != 0xC0) {
        corruption_count_++;
        sample.valid = false;
        break;  // One bad device invalidates the whole sample
    }
}
```

**Cost analysis:** The inner loop adds at most 8 additional byte comparisons per port per sample (for the longest 9-device chain). At 250 Hz, this is 250 * 7 * 8 = 14,000 extra comparisons per second. Each comparison is a load + mask + compare, roughly 3 ARM64 instructions. Total: ~42,000 instructions/sec, completely negligible on a 1.5 GHz Cortex-A72.

**ADS1299 status byte format:** Byte 0 of each device's 27-byte frame is `1100_GGGG` where `GGGG` is the GPIO state. The `0xC` upper nibble is a fixed marker. If a device is outputting zeros, this byte will be `0x00`, immediately detectable.

**Important caveat:** In a daisy chain, the status bytes of interior devices are shifted through the DOUT-DAISY_IN chain. If device N is stuck at zero, its 27 zero bytes propagate correctly through the chain (the daisy-in path is purely a shift register, not data-dependent). So `status_bytes[N][0] == 0x00` reliably indicates device N is dead. The status bytes of upstream devices (0 through N-1) remain valid because they are loaded into the shift register independently.

---

### Solution 4: Runtime Self-Healing (Hot Loop Recovery)

**Impact:** HIGH | **Complexity:** HIGH | **Addresses:** Failure modes 1, 2

**Problem it solves:** Even with a perfect init, a device can fail during acquisition (e.g., due to the ~23.4s periodic system event, or transient power issues). Currently, the only recovery is to restart the entire binary.

**Design:** Monitor per-port and per-device health during the hot loop. When a device or port is detected as failed, schedule a recovery action that executes without stopping other ports.

**Architecture:**

```
Hot Loop (core 3, SCHED_FIFO 50)
  |
  +-- Per-sample: check all status bytes (Solution 3)
  |
  +-- Per-port consecutive-zero counter
  |     If consecutive_zeros[port] >= THRESHOLD (e.g., 50):
  |       Mark port as "needs recovery"
  |       Remove port's DRDY pin from poller mask
  |       Post recovery request to recovery thread
  |
  +-- Continue acquiring from remaining healthy ports

Recovery Thread (core 1, SCHED_OTHER)
  |
  +-- Wait on recovery request queue (condvar or eventfd)
  |
  +-- For each failed port:
  |     1. START low (already not in DRDY mask)
  |     2. STOP, SDATAC
  |     3. Full re-init: reset, register write, verify
  |     4. RDATAC, START high
  |     5. Read 50 samples, validate
  |     6. If healthy: re-add DRDY pin to poller mask
  |     7. If still failed: retry (up to 5 attempts)
  |     8. If all retries exhausted: log permanent failure
```

**Key design decisions:**

1. **DRDY mask removal:** When a port is marked for recovery, its DRDY pin is removed from the poller mask. This means the hot loop no longer waits for that port's DRDY, so recovery of one port does not stall acquisition of the other 6.

2. **No shared SPI bus contention:** Two ports on the same SPI bus (e.g., Port1 and Port2 on SPI0) share SCLK/MOSI/MISO. The recovery thread must NOT perform SPI operations on a bus that the hot loop is actively reading. Two approaches:
   - **Option A (simpler):** The recovery thread sends a "pause bus" request to the hot loop. The hot loop temporarily skips the other port on the same bus for one DRDY cycle while recovery performs its SPI commands. Cost: one missed sample on the sibling port per recovery SPI transaction.
   - **Option B (zero-miss):** Queue all recovery SPI commands and execute them in the hot loop's idle window (between DRDY reads). The hot loop has ~3ms of headroom; recovery commands (register writes, reads) take ~100us each.

3. **Per-device recovery:** For single-device zeros (FM2), the server cannot selectively reset one device in a daisy chain (shared MOSI). The recovery must restart the entire port. This means all devices on that port experience a brief interruption. This is acceptable because the alternative is a permanently dead channel.

**Pseudocode for hot loop integration:**
```cpp
// Added to AcquisitionEngine:
struct PortHealth {
    int consecutive_zeros;     // Per-port consecutive zero counter
    int device_zero_count[MAX_DEVICES_PER_PORT];  // Per-device
    bool needs_recovery;
    bool permanently_failed;
};
PortHealth port_health_[MAX_PORTS];

// Recovery request queue (lock-free SPSC from hot loop to recovery thread)
SPSCRing<int> recovery_requests_;  // port index

// In the hot loop, after parsing each port's data:
bool port_all_valid = true;
for (int d = 0; d < num_dev; ++d) {
    if ((port_data.status_bytes[d][0] & 0xF0) != 0xC0) {
        port_all_valid = false;
    }
    // Track per-device zero streaks
    bool dev_zero = (port_data.status_bytes[d][0] == 0x00);
    for (int c = 0; c < CHANNELS_PER_DEVICE && !dev_zero; ++c) {
        if (port_data.channel_values[d][c] != 0) break;
        if (c == CHANNELS_PER_DEVICE - 1) dev_zero = true;
    }
    if (dev_zero) {
        port_health_[p].device_zero_count[d]++;
    } else {
        port_health_[p].device_zero_count[d] = 0;
    }
}

if (port_all_valid) {
    port_health_[p].consecutive_zeros = 0;
} else {
    port_health_[p].consecutive_zeros++;
}

// Trigger recovery after sustained failure
constexpr int RECOVERY_THRESHOLD = 50;  // 50 samples = 200ms at 250Hz
if (port_health_[p].consecutive_zeros >= RECOVERY_THRESHOLD
    && !port_health_[p].needs_recovery
    && !port_health_[p].permanently_failed) {
    port_health_[p].needs_recovery = true;
    drdy_poller_.remove_pin(devices_[p]->config().drdy_pin);
    recovery_requests_.try_push(p);
    printf("[HEAL] Port %d: scheduling recovery (%d consecutive zeros)\n",
           p, port_health_[p].consecutive_zeros);
}
```

**SPI bus contention resolution (Option B detail):**
```cpp
// Recovery thread prepares a command sequence but does not execute it directly.
// Instead, it posts the sequence to a per-bus command queue that the hot loop
// executes during its idle window.

struct RecoveryCommand {
    int port_idx;
    enum Action { STOP, SDATAC, RESET, RDATAC, START_HIGH, START_LOW,
                  WRITE_REG, READ_DATA_CHECK, FLUSH };
    Action action;
    uint8_t reg;
    uint8_t value;
};

// The hot loop checks for pending recovery commands after each sample:
RecoveryCommand cmd;
if (recovery_cmd_ring_.try_pop(cmd)) {
    execute_recovery_step(cmd);
}
```

This approach is more complex but guarantees zero missed samples on healthy ports during recovery. The tradeoff is that recovery takes longer (each command executes once per 4ms DRDY cycle instead of back-to-back).

**Recommended approach:** Start with Option A (pause sibling port for one cycle). It is simpler, the data loss is minimal (one sample on one port), and it can be upgraded to Option B later if needed.

---

### Solution 5: All-Device Channel Data Validation During Init

**Impact:** MEDIUM | **Complexity:** LOW | **Addresses:** Failure mode 2

**Problem it solves:** The current init verifies data flow by checking `status_bytes[0][0]` only. A port can pass init with device 4 dead and the server will never know.

**Design:** During the 500-sample warmup and the 20-sample data flow verification, check status bytes and channel data for ALL devices in each chain.

**Implementation:**
```cpp
// Replace the warmup status check (line 474 in controller.cpp):
// Current: checks only device 0
if ((status0 & 0xF0) != 0xC0) { warmup_corruptions[p]++; }

// Proposed: check all devices
for (int d = 0; d < dev->config().num_devices; ++d) {
    if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
        warmup_corruptions[p]++;
        break;  // One bad device = port is corrupt
    }
    // Also check for all-zero channel data
    bool all_zero = true;
    for (int c = 0; c < CHANNELS_PER_DEVICE; ++c) {
        if (pd.channel_values[d][c] != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero && d > 0) {
        // Device 0 might legitimately read zero on test signal at zero crossing.
        // But consecutive zeros on an interior device is suspicious.
        // Track per-device zero counts separately.
        device_zero_counts[p][d]++;
    }
}

// After warmup, check per-device zero counts:
for (size_t p = 0; p < devices.size(); ++p) {
    for (int d = 0; d < devices[p]->config().num_devices; ++d) {
        if (device_zero_counts[p][d] > warmup_count * 0.9) {
            printf("  [FAIL] %s device %d: %.0f%% zeros during warmup\n",
                   devices[p]->config().port_name, d + 1,
                   device_zero_counts[p][d] * 100.0 / warmup_count);
            // Add to ports_needing_reinit
        }
    }
}
```

**Important note on all-zero detection:** When running with internal test signal (CONFIG2 = 0xD0, channels set to 0x05 = test signal input), the test signal is a 1.96 Hz square wave. At 250 Hz sampling, the signal transitions every ~64 samples. A channel reading exactly zero for 64 consecutive samples would be unusual (the test signal amplitude is +/- full scale). For normal electrode input, zero detection needs a longer window because flat-line EEG is possible but consecutive zeros across all 8 channels of a device simultaneously is not physiologically plausible.

**Detection threshold:** If a device has >90% zero samples over the 500-sample warmup window (450+ zeros out of 500), flag it as dead. The test signal guarantees non-zero values for at least 50% of samples.

---

### Solution 6: Power-Cycle-Resilient Init Preamble

**Impact:** MEDIUM | **Complexity:** LOW | **Addresses:** Failure mode 4

**Problem it solves:** After power cycles, devices may be in RDATAC mode or have partially loaded registers. The current init assumes devices are in a known state after software reset, but RESET does not fully clear RDATAC mode on all ADS1299 silicon revisions.

**Design:** Add a "brute-force SPI recovery" preamble before the normal init sequence. This forces the device out of any possible state.

**Implementation:**
```cpp
void brute_force_spi_recovery(ADS1299Device& dev) {
    int num_dev = dev.config().num_devices;

    // 1. Clock out any stale data in the shift register.
    //    Worst case: device is mid-RDATAC frame. Need to clock out
    //    num_devices * 27 bytes to clear the pipeline.
    for (int i = 0; i < 3; ++i) {
        dev.flush_spi();
        sleep_ms(1);
    }

    // 2. Send SDATAC repeatedly. The ADS1299 datasheet (section 9.5.2.3)
    //    says SDATAC is a single-byte command (0x11) that can be sent at
    //    any time during RDATAC mode. But in a daisy chain, the SDATAC
    //    byte must arrive at the right position in the shift register.
    //    Sending it multiple times with flushes in between maximizes the
    //    probability of it landing correctly.
    for (int i = 0; i < 10; ++i) {
        dev.send_command(Cmd::SDATAC);
        sleep_ms(5);
        // Flush any data that arrived between SDATAC sends
        dev.flush_spi();
        sleep_ms(1);
    }

    // 3. Send STOP to ensure no conversions are running
    dev.send_command(Cmd::STOP);
    sleep_ms(10);

    // 4. START pin low
    dev.start_low();
    sleep_ms(10);

    // 5. Multiple software resets with generous settling
    for (int i = 0; i < 3; ++i) {
        dev.send_command(Cmd::RESET);
        sleep_ms(200);  // Generous settling for post-power-cycle
        dev.send_command(Cmd::SDATAC);
        sleep_ms(50);
    }

    // 6. Verify SPI communication with register write/read test
    dev.write_register(Reg::LOFF, 0x55);
    sleep_ms(10);
    uint8_t readback = dev.read_register(Reg::LOFF);
    if (readback == 0x55) {
        printf("  %s: SPI recovery successful\n", dev.config().port_name);
    } else {
        printf("  %s: SPI recovery readback 0x%02X (expected 0x55)\n",
               dev.config().port_name, readback);
    }
    dev.write_register(Reg::LOFF, 0x00);
    sleep_ms(10);
}

// Call before initialize_device() for each port:
for (int i = 0; i < num_ports; ++i) {
    brute_force_spi_recovery(*devices[i]);
}
```

**Why more aggressive than current init:** The current init sends 2 software resets with `0.1 + N*0.01` second settling. After a power cycle, the device may need more time and more reset cycles. The recovery preamble uses 3 resets with 200ms settling (an additional ~600ms per port during the preamble). This is a one-time cost at startup.

**Optimization:** The brute-force preamble should only run on the first init attempt. Subsequent retries (from Solution 2) can use the faster normal init since the device is in a known state after the first attempt's shutdown sequence.

---

### Solution 7: Smart Re-Sync with Recovery Verification

**Impact:** MEDIUM | **Complexity:** MEDIUM | **Addresses:** Failure modes 3, 5

**Problem it solves:** The re-sync dilemma: global re-sync breaks recovered ports, but removing re-sync exposes periodic corruption. Solution 1 (per-port re-sync) addresses this partially. This solution adds a verification-and-recovery layer on top.

**Design:** After re-sync (whether global or per-port), verify each port individually and immediately recover any that broke. The re-sync is treated as a "might break things" operation with mandatory post-verification.

**Implementation:**
```cpp
struct PortState {
    bool was_healthy_before_resync;
    bool is_healthy_after_resync;
};

void verified_resync(std::vector<ADS1299Device*>& devices, I2CDevice& i2c,
                     const DeviceConfig& config) {
    const int num_ports = static_cast<int>(devices.size());
    PortState states[MAX_PORTS] = {};

    // 1. Record pre-resync health
    ADS1299Device* ref = devices[0];
    for (auto* d : devices) {
        if (d->config().num_devices > ref->config().num_devices) ref = d;
    }

    for (int i = 0; i < 20; ++i) {
        if (!ref->wait_for_drdy(0.02)) continue;
        for (int p = 0; p < num_ports; ++p) {
            PortData pd;
            devices[p]->read_data(pd);
            if ((pd.status_bytes[0][0] & 0xF0) == 0xC0) {
                states[p].was_healthy_before_resync = true;
            }
        }
    }

    // 2. Per-port re-sync (Solution 1)
    for (int p = 0; p < num_ports; ++p) {
        auto* dev = devices[p];
        dev->start_low();
        sleep_ms(50);
        dev->send_command(Cmd::STOP);
        sleep_ms(10);
        dev->flush_spi();
        dev->send_command(Cmd::SDATAC);
        sleep_ms(20);
        dev->send_command(Cmd::RDATAC);
        sleep_ms(10);
        dev->send_command(Cmd::RDATAC);
        sleep_ms(20);
        dev->start_high();
        sleep_ms(100);
    }

    sleep_ms(500);  // Let all ports settle

    // 3. Post-resync health check
    for (int i = 0; i < 20; ++i) {
        if (!ref->wait_for_drdy(0.02)) continue;
        for (int p = 0; p < num_ports; ++p) {
            PortData pd;
            devices[p]->read_data(pd);
            if ((pd.status_bytes[0][0] & 0xF0) == 0xC0) {
                states[p].is_healthy_after_resync = true;
            }
        }
    }

    // 4. Recover any ports that broke
    for (int p = 0; p < num_ports; ++p) {
        if (states[p].was_healthy_before_resync &&
            !states[p].is_healthy_after_resync) {
            printf("  [WARN] %s: broke during re-sync, recovering...\n",
                   devices[p]->config().port_name);

            for (int attempt = 0; attempt < 5; ++attempt) {
                if (restart_single_port(*devices[p], attempt)) {
                    // Verify recovery
                    sleep_ms(100);
                    bool recovered = false;
                    for (int v = 0; v < 10; ++v) {
                        if (devices[p]->wait_for_drdy(0.02)) {
                            PortData pd;
                            devices[p]->read_data(pd);
                            if ((pd.status_bytes[0][0] & 0xF0) == 0xC0) {
                                recovered = true;
                                break;
                            }
                        }
                    }
                    if (recovered) {
                        printf("    %s: recovered (attempt %d) [OK]\n",
                               devices[p]->config().port_name, attempt + 1);
                        break;
                    }
                }
            }
        }
    }

    // 5. Settling samples
    for (int i = 0; i < 100; ++i) {
        if (ref->wait_for_drdy(0.02)) {
            for (auto* dev : devices) {
                PortData pd;
                dev->read_data(pd);
            }
        }
    }
}
```

---

### Solution 8: Watchdog-Based Auto-Restart

**Impact:** MEDIUM | **Complexity:** LOW | **Addresses:** All failure modes (last resort)

**Problem it solves:** Despite all init improvements, edge cases may still cause init failure. For a "run and forget" system, the binary itself should be capable of restarting without human intervention.

**Design:** Use a systemd service with automatic restart on failure. The binary exits with a non-zero code if init fails after all retries, and systemd restarts it.

**Implementation:**

systemd service file (`/etc/systemd/system/ads1299.service`):
```ini
[Unit]
Description=ADS1299 EEG Acquisition
After=network.target

[Service]
Type=simple
ExecStart=/home/morph/ads1299-cpp/build/ads1299_acquire
Restart=on-failure
RestartSec=5
StartLimitBurst=10
StartLimitIntervalSec=600
User=root
CPUAffinity=3
Nice=-20

# Environment for clean startup
Environment=LD_LIBRARY_PATH=/usr/local/lib

[Install]
WantedBy=multi-user.target
```

Combined with Solution 2 (exhaustive init retry loop), the binary retries internally up to 10 times. If it still fails, it exits with code 1, and systemd restarts it after 5 seconds. The `StartLimitBurst=10` allows up to 10 restarts within 10 minutes before systemd gives up.

**Total retry budget:** 10 internal attempts * 10 systemd restarts = 100 init attempts before permanent failure. Each attempt takes roughly 15-30 seconds, so the system will try for up to ~50 minutes before giving up.

---

### Solution 9: Per-Device Consecutive-Zero Tracking with Targeted Re-Init

**Impact:** MEDIUM | **Complexity:** MEDIUM | **Addresses:** Failure mode 2

**Problem it solves:** Single-device failures are currently invisible. Even with Solution 3 (status byte validation), the hot loop detects the failure but has no mechanism to fix it. Solution 4 provides the general framework; this solution describes the specific per-device detection logic and its interaction with the daisy chain.

**Design:** Track consecutive all-zero samples per device (not just per port). When a specific device hits the threshold, log which device failed and schedule port recovery. The recovery restart affects all devices on the port, but only the failed device triggered it.

**Detection heuristic for per-device zeros:**

```cpp
// In the hot loop, after parse_raw:
for (int d = 0; d < num_dev; ++d) {
    // A device is "zero" if status byte is 0x00 AND all 8 channels are 0
    bool is_zero = (port_data.status_bytes[d][0] == 0x00);
    if (!is_zero) {
        // Also check for status OK but channels all zero (partial failure)
        bool channels_zero = true;
        for (int c = 0; c < CHANNELS_PER_DEVICE; ++c) {
            if (port_data.channel_values[d][c] != 0) {
                channels_zero = false;
                break;
            }
        }
        // If status is valid but all channels are zero for extended period,
        // the ADC core is not producing data
        is_zero = channels_zero &&
                  per_device_zero_streak_[p][d] > 250;  // > 1 second
    }

    if (is_zero) {
        per_device_zero_streak_[p][d]++;
    } else {
        per_device_zero_streak_[p][d] = 0;
    }
}
```

**Threshold selection:** 50 consecutive zeros (200ms) is a good default. The test signal transitions every ~64 samples, so any healthy device must produce a non-zero value within 128 samples. 50 zeros guarantees the device is dead (not merely at a zero crossing).

**Reporting:** When scheduling recovery, log the specific failed device:
```
[HEAL] Port2 device 4: 50 consecutive zeros, scheduling port recovery
```

This gives the operator actionable information about which physical device in which daisy chain position is failing, enabling targeted hardware debugging.

---

### Solution 10: Staggered VCAP1 Charging and Reference Settling

**Impact:** LOW | **Complexity:** LOW | **Addresses:** Failure mode 4

**Problem it solves:** The current init waits 2 seconds for VCAP1 charging across all devices simultaneously. Long daisy chains (9 devices) have higher aggregate capacitance on the VCAP1 rail if they share a reference voltage. Staggering the init per-bus with explicit reference buffer settling may improve post-power-cycle reliability.

**Design:** After the 2-second global VCAP1 wait, add per-port reference buffer verification.

```cpp
// After CONFIG3 write (enables internal reference):
write_and_verify(dev, Reg::CONFIG3, config.config3, "CONFIG3");

// Verify reference is actually settled by checking CONFIG3 readback
// The ADS1299 PD_REFBUF bit (bit 7) must be set and stable
sleep_ms(200);
for (int attempt = 0; attempt < 10; ++attempt) {
    uint8_t c3 = dev.read_register(Reg::CONFIG3);
    if (c3 == config.config3) break;
    printf("  Reference buffer settling... (attempt %d, read 0x%02X)\n",
           attempt + 1, c3);
    sleep_ms(100);
}
```

This adds at most 1 second of extra settling for ports with slow reference startup, and costs nothing for ports that settle immediately.

---

## Implementation Priority

| Priority | Solution | Impact | Complexity | Addresses |
|----------|----------|--------|------------|-----------|
| 1 | Solution 3: Per-device status validation in hot loop | HIGH | LOW | FM2 |
| 2 | Solution 5: All-device validation during init | MEDIUM | LOW | FM2 |
| 3 | Solution 1: Per-port re-sync | HIGH | LOW | FM1, FM3, FM5 |
| 4 | Solution 2: Exhaustive init retry loop | HIGH | LOW | FM1-4 |
| 5 | Solution 6: Power-cycle-resilient init preamble | MEDIUM | LOW | FM4 |
| 6 | Solution 7: Smart re-sync with recovery verification | MEDIUM | MEDIUM | FM3, FM5 |
| 7 | Solution 8: Watchdog auto-restart | MEDIUM | LOW | All (last resort) |
| 8 | Solution 4: Runtime self-healing | HIGH | HIGH | FM1, FM2 |
| 9 | Solution 9: Per-device zero tracking | MEDIUM | MEDIUM | FM2 |
| 10 | Solution 10: Staggered reference settling | LOW | LOW | FM4 |

**Recommended implementation order:**

**Phase A (immediate, ~1 day):** Solutions 3, 5, 1, 2. These are all low-complexity changes that collectively address every failure mode. Solution 3 is a 5-line change in the hot loop. Solution 5 extends the warmup validation. Solution 1 replaces the global re-sync. Solution 2 wraps everything in a retry loop.

**Phase B (short-term, ~1 day):** Solutions 6, 7, 8. The power-cycle preamble, verified re-sync, and systemd watchdog add defense in depth.

**Phase C (when needed, ~2-3 days):** Solutions 4, 9. Runtime self-healing is the most complex change but provides the ultimate reliability guarantee. Only needed if Phase A+B do not achieve 100% init reliability in practice.

---

## Expected Outcome After Phase A

With Solutions 1-3 and 5 implemented:

1. **Whole-port zeros:** Detected during init warmup (extended validation) and either recovered via restart or caught by the exhaustive retry loop that re-initializes the entire system. The per-port re-sync prevents the "recovery undone by global restart" failure.

2. **Single-device zeros:** Detected during init warmup (per-device channel data validation) and flagged for port re-init. Detected during acquisition (per-device status byte check) and logged. The exhaustive retry loop ensures the system does not start acquisition until all devices are confirmed working.

3. **Re-sync breakage:** Eliminated by per-port re-sync that verifies each port individually and recovers immediately if broken.

4. **Post-power-cycle failures:** Addressed by the exhaustive retry loop. Even if the first init attempt fails on 3 ports, subsequent attempts will likely succeed. The retry budget (10 attempts) handles even the worst observed failure patterns.

5. **Periodic corruption:** The per-port re-sync provides the same shift register alignment benefit as the global re-sync, maintaining resilience against the ~23.4s periodic system event.

The goal of "run the binary once, all 328 channels work, no manual intervention needed" should be achievable with Phase A alone for the vast majority of power-cycle scenarios. Phase B adds the safety net for edge cases. Phase C provides runtime recovery for the rare case where a device fails mid-acquisition.
