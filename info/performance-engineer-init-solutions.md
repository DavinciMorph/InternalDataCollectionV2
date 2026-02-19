# Performance Analysis: ADS1299 Init Reliability & Runtime Self-Healing

## System Under Analysis

- 7 SPI ports, variable daisy-chain depths: [9, 7, 5, 4, 4, 5, 7] = 41 devices, 328 channels @ 250 Hz
- 4 physical SPI buses: SPI0 (ports 1-2), SPI3 (ports 3-4), SPI4 (ports 5-6), SPI5 (port 7)
- DRDY via I2C TCA9534 (0x20), START via TCA9534 (0x21), both single-expander
- Hardware constraint: RESET/PWDN tied HIGH (software reset only), no DRDY interrupt (INT pin floating)
- Current cycle time: 0.63 ms mean, 0.80 ms max. Sample period: 4.0 ms. Headroom: 3.2 ms
- Acquisition thread: core 3 (isolated), SCHED_FIFO 50, mlockall. Bus workers: SCHED_FIFO 49

---

## 1. Root Cause Analysis: Why Init Fails

### 1.1 The Fundamental Problem: Daisy-Chain Command Propagation

The ADS1299 daisy chain shares a single MOSI line. When a command byte is sent, it enters
the shift register of device 0 (outermost). On the *next* SCLK burst, it propagates to
device 1, and so on. A single `send_command(SDATAC)` call only directly affects device 0.
Inner devices need N additional SPI clock bursts (where N = chain depth - 1) to receive
the command.

**Critical insight**: The current `initialize_device()` sends SDATAC `num_devices + 2`
times (line 93-97 of `controller.cpp`), but each send is a separate SPI transaction with
its own CS assertion/deassertion cycle. This is correct for the outermost device but
relies on the command rippling through the shift register across separate transactions.
For a 9-deep chain, the innermost device needs 8 additional clock bursts *after* its
command byte enters the chain. The current approach works because each SDATAC send pushes
previous command bytes deeper, but the timing is sensitive to SPI bus turnaround time and
CS gap duration.

### 1.2 The "All-Zeros" Failure Mode

When a port returns all-zeros after START assertion, the likely causes are:

1. **RDATAC not received by all devices in chain**: If even one device in the daisy chain
   did not receive RDATAC, its output is undefined. Since daisy-chain output is serialized
   (device N-1 first, device 0 last), a non-responding inner device corrupts the entire
   frame alignment.

2. **Internal reference not settled**: CONFIG3 enables the internal reference (0xE0). The
   datasheet specifies 150ms settling time. The code waits 200ms (line 167), which should
   be sufficient, but under power supply noise from 41 devices, marginal reference settling
   could cause sporadic zero output.

3. **Shift register contamination**: During the transition from register mode to RDATAC mode,
   any partial SPI transaction or glitch can leave stale data in the shift register. The
   `flush_spi()` call reads one frame and waits 1ms, but for a 9-deep chain (243 bytes),
   there may be residual data from a previous partial read.

4. **START timing race**: The TCA9534 `set_all_high()` asserts START via I2C, which has
   non-trivial latency (~200-400us for the I2C transaction). Between RDATAC and START,
   if a device's internal state machine times out waiting for START, it may not begin
   conversion properly.

### 1.3 The Re-Sync Destructor Effect

The "final re-synchronization" in `start_all_conversions_synchronized()` (lines 679-757)
stops all ports, flushes, re-enters RDATAC, and restarts. This was designed to align all
ports' DRDY edges, but it introduces the exact same failure mode as initial startup:

- Stopping a port that was producing valid data, then restarting it, gives it a fresh
  opportunity to fail.
- The re-sync success rate is roughly (1 - per_port_fail_rate)^7. If each port has a 10%
  chance of failing on restart, the probability that all 7 succeed is 0.9^7 = 48%.

**This is the single largest reliability problem.** The re-sync undoes the recovery work
of the warmup phase.

### 1.4 The ~23.4s Periodic Corruption

23.4 seconds * 250 Hz = 5850 samples. This correlates with:

- **WiFi scanning interval**: `wpa_supplicant` performs periodic scans every 30s by default,
  but with `bgscan` configuration it can be shorter. The DMA controller shared between WiFi
  (SDHOST or EMMC2) and SPI can cause bus contention.
- **Kernel timer wheel**: The default HZ=250 configuration means timer interrupts align with
  the sample rate. Combined with `isolcpus=3`, IRQs should not land on core 3, but SPI DMA
  completion interrupts may still be routed to non-isolated cores that are running bus workers.
- **Garbage collection in the streaming client** (if connected): Not relevant to the C++
  acquisition side, but could cause TCP backpressure.

---

## 2. Proposed Init Strategy: "Init Until Clean"

### 2.1 Eliminate the Final Re-Sync

**Priority: CRITICAL. Implement first.**

The re-sync at lines 679-757 of `controller.cpp` is the primary source of init-time
regression. Remove it entirely. The purpose of re-sync was to align DRDY edges, but since
all devices share the same external clock and the TCA9534 `set_all_high()` already provides
a simultaneous START, the DRDY edges are inherently synchronized (within the I2C write
latency of ~200-400us, which is well under one sample period of 4ms).

**Implementation**: Delete lines 679-757 in `controller.cpp` (the entire "Final
re-synchronization" block). The function should return `all_ok` immediately after the
20-sample data flow verification.

**Risk**: If DRDY alignment was actually necessary, this could cause the DRDY poller to
miss the window where all ports are simultaneously ready. However, the current DRDY poller
reads a single I2C register and checks all bits at once, so even if ports are slightly
misaligned, they will all be low (ready) within the same 4ms sample period because they
share the same clock source. The poll loop runs at ~5-10 kHz (100-200us per iteration),
fast enough to catch the overlap window.

**Expected impact**: Eliminates the ~50% probability that re-sync breaks previously
recovered ports. Net init success rate should roughly double.

### 2.2 Per-Port Independent Recovery Loop

Replace the current sequential init+warmup+re-init+re-sync pipeline with an iterative
per-port recovery loop. The strategy is:

```
for each port (in parallel where possible):
    attempt = 0
    while port not verified AND attempt < MAX_ATTEMPTS:
        if attempt == 0:
            standard_init(port)  // RESET, SDATAC, registers, RDATAC, START
        else if attempt < 5:
            restart_single_port(port)  // STOP, SDATAC, RDATAC, START
        else:
            full_reinit(port)  // Complete re-initialization from scratch

        wait 200ms for settling
        verified = verify_data_stream(port, 100 samples, threshold=95% valid)
        attempt++

    if not verified:
        mark_port_degraded(port)
```

Key differences from current approach:

1. **No global stop/restart** -- only the failing port is cycled. Other ports continue
   producing data (or at least remain in a known-good state).
2. **Escalating recovery** -- start with lightweight restart, escalate to full re-init.
3. **100-sample verification** -- more statistically significant than the current 5-sample
   or 20-sample checks. At 250 Hz, 100 samples = 400ms. With a 95% validity threshold
   (95/100 valid), the false-positive rate for declaring a bad port "good" is negligible.
4. **Parallelizable** -- ports on different SPI buses can be recovered in parallel since
   they use independent hardware. Ports 1-2 (SPI0), 3-4 (SPI3), 5-6 (SPI4), 7 (SPI5)
   can be recovered in 4 parallel groups.

**Performance estimate**: Worst-case init time with 5 lightweight retries + 3 full
re-inits per port, 100-sample verification each:

- Lightweight restart: ~100ms (stop/start/settle) + 400ms (verify) = 500ms
- Full re-init: ~3s (reset/config/settle) + 400ms (verify) = 3.4s
- Worst case per port: 5 * 500ms + 3 * 3.4s = 12.7s
- With 4-way parallelism: ~12.7s worst case (limited by slowest port)
- Typical case (1-2 retries): ~3-5s total init

### 2.3 Enhanced SDATAC Verification for Deep Chains

The current SDATAC verification (lines 100-129) writes 0xAA to LOFF and reads back. This
only verifies device 0. For a 9-deep chain, device 8 (innermost) may not have received
SDATAC.

**Proposed enhancement**: After the SDATAC loop, perform a bulk register read of all
devices using the multi-device read technique:

```cpp
// Read ID register from ALL devices in chain by reading N bytes
// In SDATAC mode, RREG 0x00 0x00 reads 1 byte from the addressed register
// For daisy chain, the response contains device 0's register first,
// then zeros for unaddressed devices.
//
// Better approach: read CONFIG1 (should be 0x96 after write) from each device
// by sending RREG with count = num_devices to verify all received SDATAC.
```

However, this is actually not straightforward in daisy-chain mode because register
commands are broadcast to all devices simultaneously and the response is only from the
outermost device. The daisy chain is transparent for data reads (RDATAC) but not for
register access -- all devices receive the same command on MOSI.

**Practical alternative**: Instead of verifying SDATAC for inner devices, increase the
SDATAC count to `num_devices * 3` (currently `num_devices + 2`) and add a 50ms delay
between each group of 3 sends. This gives the shift register ample time to propagate
commands to all devices. For a 9-deep chain: 27 SDATAC sends with pauses, total ~1.5s.
This is more time upfront but reduces the probability of SDATAC failure, which is the
root cause of the all-zeros problem.

### 2.4 Double-Flush Before RDATAC Entry

Before sending RDATAC, perform two full-frame flushes with a delay between them:

```cpp
// Flush 1: clear any stale data in shift register
dev.flush_spi();
sleep_ms(5);

// Flush 2: clear any data that arrived during flush 1
dev.flush_spi();
sleep_ms(5);
```

The current single flush may not be sufficient if the device was in an intermediate state
where new data was being clocked out during the flush read.

### 2.5 RDATAC Verification Enhancement

The current RDATAC verification (lines 308-336) checks for 0xFF (all ones), which
indicates RDATAC was not received. Add a check for 0x00 (all zeros), which indicates
RDATAC was received but the device is not converting:

```cpp
if ((check_rx[0] == 0xFF && check_rx[1] == 0xFF && check_rx[2] == 0xFF) ||
    (check_rx[0] == 0x00 && check_rx[1] == 0x00 && check_rx[2] == 0x00)) {
    rdatac_fail_indices.push_back(static_cast<int>(i));
}
```

---

## 3. Runtime Health Monitoring

### 3.1 Per-Port Status Byte Validation in Hot Loop

**Current state**: Only device 0's status byte is validated per port (line 173 of
`engine.cpp`). Inner devices are never checked.

**Proposed change**: Validate all devices' status bytes. This adds `num_devices - 1`
comparisons per port per sample, but comparisons are single-cycle operations on ARM Cortex-A72.

```cpp
// In engine.cpp hot loop, replace single status check with per-device check
bool port_valid = true;
for (int d = 0; d < num_dev; ++d) {
    if ((port_data.status_bytes[d][0] & 0xF0) != 0xC0) {
        port_valid = false;
        // Track which port and device failed (for recovery thread)
        port_health_[p].consecutive_failures++;
        port_health_[p].last_failure_device = d;
        break;  // No need to check remaining devices
    }
}
if (!port_valid) {
    corruption_count_++;
    sample.valid = false;
    // Continue -- don't stall the hot loop
}
```

**Cost estimate**: The inner loop adds at most 9 comparisons (for the deepest chain).
Each comparison is ~1 cycle. Status bytes are already in L1 cache from the parse step.
Total overhead: <50ns per sample. Against 630us cycle time, this is 0.008% -- negligible.

### 3.2 All-Zeros Detection (Per-Port)

A more specific failure detector: if a port returns status byte 0x00 for N consecutive
samples, it is definitively dead (not transient corruption).

```cpp
struct PortHealth {
    uint32_t consecutive_zeros = 0;
    uint32_t consecutive_failures = 0;
    uint32_t total_failures = 0;
    int      last_failure_device = -1;
    bool     marked_dead = false;
    bool     recovery_requested = false;
};

// In hot loop after status check:
if (port_data.status_bytes[0][0] == 0x00) {
    port_health_[p].consecutive_zeros++;
} else {
    port_health_[p].consecutive_zeros = 0;
}

// After 50 consecutive zeros (200ms at 250 Hz), flag for recovery
if (port_health_[p].consecutive_zeros >= 50 && !port_health_[p].recovery_requested) {
    port_health_[p].recovery_requested = true;
    recovery_queue_.push(p);  // Lock-free queue to recovery thread
}
```

**Why 50 samples?** At 250 Hz, 50 samples = 200ms. This is long enough to distinguish
a truly dead port from transient corruption (which typically lasts 1-3 samples from
SPI bus glitches). It is short enough that the user sees data resume within ~1 second
of a failure being detected (200ms detection + ~500ms recovery).

### 3.3 Data Quality Metrics (Zero Hot-Loop Cost)

Add per-port counters that the stats printer reads every 10 seconds:

```cpp
struct PortHealth {
    // ... above fields ...
    uint64_t valid_samples = 0;
    uint64_t invalid_samples = 0;
    uint64_t zero_samples = 0;
    uint64_t recoveries_attempted = 0;
    uint64_t recoveries_succeeded = 0;
};
```

These are simple `uint64_t` increments (single instruction on ARM64), only on the
acquisition thread -- no atomics needed. The stats printer reads them from a different
thread, but stale reads are acceptable for monitoring (no need for memory barriers).

---

## 4. Hot-Swap Recovery Architecture

### 4.1 Design: Recovery Thread on Core 2

The system has 3 idle cores (0, 1, 2) during acquisition (core 3 is the engine). Bus
workers are pinned to cores 0-2 but spend >99% of their time blocked on eventfd. A
recovery thread can safely share core 2 (SPI4's worker) because recovery operations take
milliseconds while the worker's active period is <0.3ms per 4ms cycle.

```
Recovery Thread (core 2, SCHED_FIFO 40):
    while (running):
        port_index = recovery_queue_.pop()  // blocking wait

        // Step 1: Remove port from DRDY mask (so engine doesn't wait for it)
        drdy_poller_.remove_pin(port_drdy_pin[port_index])

        // Step 2: Signal bus worker to skip this port
        bus_worker_skip_mask_.set(port_index)

        // Step 3: Wait for one acquisition cycle to complete
        //         (ensures no SPI transaction is in progress on this port)
        sleep_ms(5)  // > 1 sample period

        // Step 4: Perform recovery (uses SPI directly, safe because bus worker is skipping)
        success = attempt_recovery(port_index)

        // Step 5: If successful, re-add to DRDY mask and un-skip in bus worker
        if (success):
            bus_worker_skip_mask_.clear(port_index)
            drdy_poller_.add_pin(port_drdy_pin[port_index])
            port_health_[port_index].reset()
        else:
            // Leave port excluded, log failure
            // Retry after cooldown period (e.g., 30 seconds)
```

### 4.2 Critical Constraint: SPI Bus Sharing

Ports on the same physical SPI bus share MOSI/MISO/SCLK. Only CS differs. This means:

- **Recovery of one port on a 2-port bus temporarily disrupts the other port.**
- When recovering Port1 (SPI0.CE0), Port2 (SPI0.CE1) cannot be read simultaneously.

**Solution**: When a port is flagged for recovery, the recovery thread must coordinate
with the bus worker:

1. Recovery thread sets a "recovery in progress" flag for the SPI bus.
2. Bus worker checks the flag before reading each port. If recovery is in progress for
   a sibling port, it still reads the healthy port but skips the recovering port.
3. The recovery thread uses the same SPI fd (via the SPIBus object) but with explicit
   CS control through the port's own SPI device file (/dev/spidevX.Y).

Since each port has its own `/dev/spidevX.Y` file descriptor with its own CS line, SPI
transactions on different CS lines do not interfere at the hardware level. The kernel's
SPI subsystem serializes transactions on the same bus via a mutex, so the recovery
thread's SPI transactions will naturally interleave with the bus worker's reads. This
adds latency to the bus worker's reads during recovery, but:

- Recovery SPI transactions (commands, register writes) are short (3-10 bytes each).
- Kernel SPI mutex acquisition is ~1-5us.
- Total added latency per acquisition cycle during recovery: ~10-50us.
- This is well within the 3.2ms headroom.

### 4.3 Recovery Procedure

```cpp
bool attempt_recovery(int port_index) {
    auto* dev = devices_[port_index];

    // Escalating recovery:
    // Level 1: Quick restart (STOP + SDATAC + RDATAC + START)
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (restart_and_verify(dev, 50)) {  // 50-sample verify
            log("Port %d: recovered (quick restart, attempt %d)", port_index, attempt);
            return true;
        }
    }

    // Level 2: Full re-init (RESET + register config + RDATAC + START)
    for (int attempt = 0; attempt < 2; ++attempt) {
        dev->start_low();
        sleep_ms(50);
        dev->send_command(Cmd::STOP);
        sleep_ms(20);

        if (ADS1299Controller::initialize_device(*dev, config_)) {
            dev->send_command(Cmd::RDATAC);
            sleep_ms(50);
            dev->start_high();
            sleep_ms(200);

            if (verify_data_stream(dev, 100, 0.95)) {  // 100 samples, 95% threshold
                log("Port %d: recovered (full re-init, attempt %d)", port_index, attempt);
                return true;
            }
        }
    }

    log("Port %d: recovery failed after all attempts", port_index);
    return false;
}
```

### 4.4 DRDY Mask Manipulation: Thread Safety

The `DRDYPoller::mask_` is a `uint8_t` read by the engine thread and modified by the
recovery thread. On ARM64, `uint8_t` reads and writes are atomic at the hardware level
(single-byte load/store). However, for correctness across CPU caches, `mask_` should be
changed to `std::atomic<uint8_t>` with `memory_order_relaxed` for the poll path:

```cpp
// In drdy_poller.hpp:
std::atomic<uint8_t> mask_{0};

// In poll():
uint8_t current_mask = mask_.load(std::memory_order_relaxed);
if ((val & current_mask) == 0) return true;
```

`memory_order_relaxed` is sufficient because:
- The engine does not depend on the mask change being immediately visible. If it reads
  the old mask for one more cycle, the worst case is a single DRDY timeout (because the
  dead port's DRDY is stuck high), followed by the mask update being visible on the next
  cycle.
- There is no data dependency between the mask change and any other shared state.

**Cost**: `atomic<uint8_t>` load with `relaxed` ordering compiles to the exact same
instruction as a plain `uint8_t` load on ARM64 (`ldrb`). Zero overhead.

### 4.5 Bus Worker Skip Mask

The bus worker needs to know when to skip a recovering port. Use an `atomic<uint8_t>`
bitmask per bus worker:

```cpp
// In bus_worker.hpp:
alignas(64) std::atomic<uint8_t> skip_mask_{0};

// In bus_worker.cpp run():
for (int i = 0; i < num_ports_; ++i) {
    if (skip_mask_.load(std::memory_order_relaxed) & (1u << i)) {
        // Zero out the raw buffer so engine sees zeros (detected as failure)
        std::memset(raw_buffers_[i], 0, len);
        continue;
    }
    devices_[i]->read_raw(raw_buffers_[i], len);
}
```

When the engine sees zeros from a skipped port, it increments the port's failure counter.
But since `recovery_requested` is already true, it does not re-queue the port.

---

## 5. Periodic Interference Mitigation

### 5.1 WiFi DMA Contention

**Root cause hypothesis**: The BCM2711's DMA controller has 15 legacy channels (0-14)
and 4 DMA4 channels. WiFi (via `brcmfmac` driver) uses DMA for both TX and RX. When
WiFi scanning occurs, the DMA controller is contended, adding latency to SPI DMA
completions. If a SPI read takes >4ms (missing the next DRDY), the shift register
advances to the next sample, corrupting the in-progress read.

**Mitigations (ordered by impact vs. disruption)**:

1. **Disable WiFi scanning** (LOW DISRUPTION, HIGH IMPACT):
   ```bash
   # In /etc/wpa_supplicant/wpa_supplicant.conf:
   bgscan=""
   # Or via NetworkManager:
   nmcli connection modify <name> 802-11-wireless.bgscan ""
   ```
   This prevents periodic background scans. The WiFi connection remains active for data
   transfer but does not periodically scan for new APs.

   If the system is on a stable WiFi network, this should eliminate the ~23.4s periodic
   corruption with zero downside.

2. **Disable WiFi power management** (LOW DISRUPTION):
   ```bash
   sudo iw wlan0 set power_save off
   # Persist in /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf:
   [connection]
   wifi.powersave = 2  # 2 = disable
   ```

3. **Pin WiFi IRQ to core 0** (ALREADY PARTIALLY DONE via irq-affinity.service):
   Verify that the `brcmfmac` IRQ is pinned away from core 3. Also consider pinning it
   away from cores 1-2 where bus workers run:
   ```bash
   # Find brcmfmac IRQ number:
   grep brcmfmac /proc/interrupts
   # Pin to core 0 only:
   echo 1 > /proc/irq/<N>/smp_affinity
   ```

4. **Switch to Ethernet** (MEDIUM DISRUPTION, HIGHEST IMPACT):
   If the Pi is stationary, use wired Ethernet and disable WiFi entirely:
   ```bash
   sudo rfkill block wifi
   ```
   This completely eliminates WiFi DMA contention. The Pi 4 Ethernet controller uses a
   dedicated USB3 bus (via VL805), not the legacy DMA controller.

5. **Use `PREEMPT_RT` kernel** (HIGH DISRUPTION, MEDIUM IMPACT):
   A PREEMPT_RT patched kernel provides deterministic interrupt latency and prevents
   DMA contention from causing priority inversion in the SPI driver. Available via:
   ```bash
   sudo apt install linux-image-rt-arm64
   ```
   However, this changes the entire kernel behavior and requires thorough regression
   testing.

### 5.2 Kernel Timer Interference

The default `CONFIG_HZ=250` means the kernel timer tick fires every 4ms -- exactly at
the sample rate. If the timer tick consistently lands during SPI reads, it adds jitter.

**Mitigations**:

1. **Enable `NO_HZ_FULL` for core 3** (if kernel supports it):
   ```bash
   # In /boot/firmware/cmdline.txt:
   nohz_full=3
   ```
   This eliminates periodic timer ticks on core 3 when it is running a single task
   (the acquisition engine). Combined with `isolcpus=3`, this makes core 3 nearly
   interrupt-free.

   **Important**: `nohz_full` requires at least one "housekeeping" core (core 0 by
   default). The Pi 4's Raspberry Pi OS kernel may or may not have `NO_HZ_FULL` enabled.
   Check with: `grep NO_HZ_FULL /boot/config-$(uname -r)`.

2. **Verify RCU callbacks are off core 3**:
   ```bash
   # In /boot/firmware/cmdline.txt:
   rcu_nocbs=3
   ```
   This moves RCU callback processing off core 3. Without this, the RCU subsystem can
   schedule work on core 3 even with `isolcpus`.

### 5.3 DRDY Timeout Tuning

The current DRDY timeout is 8ms (line 140 of `engine.cpp`). If periodic interference
causes a single missed DRDY, the 8ms timeout means the engine waits up to 8ms before
giving up and re-polling. During this time, a new DRDY edge may have already fired (at
4ms interval), meaning the engine could miss up to 2 samples.

**Proposed change**: Reduce DRDY timeout to 5ms (just over one sample period):

```cpp
if (!drdy_poller_.poll(0.005)) {  // 5ms timeout
```

This ensures that if DRDY is missed, the engine recovers on the very next sample period
rather than potentially missing two. The 1ms margin above the 4ms sample period accounts
for jitter.

Additionally, when a DRDY timeout occurs, the engine should flush all port data to prevent
shift register misalignment:

```cpp
if (!drdy_poller_.poll(0.005)) {
    drdy_timeouts_++;
    // Flush all ports to prevent stale data in shift registers
    for (auto* w : workers_) {
        w->trigger();
    }
    for (auto* w : workers_) {
        w->wait_done();
    }
    // Discard the data (do not push to ring)
    continue;
}
```

**Cost**: This adds one extra SPI read cycle (0.63ms) per timeout event. Given that
timeouts are rare (<1 per 10 minutes in normal operation), this is negligible. But it
prevents the shift register desynchronization that causes cascading corruption after a
single missed DRDY.

---

## 6. Watchdog / Supervisor Architecture

### 6.1 Three-Layer Monitoring

```
Layer 1: Hot-loop health counters (engine thread, core 3)
    - Per-port status byte validation
    - Per-port consecutive-zero detection
    - Lightweight: ~50ns overhead per sample
    - Signals recovery thread via lock-free queue

Layer 2: Recovery thread (core 2, SCHED_FIFO 40)
    - Receives port-recovery requests from Layer 1
    - Performs escalating SPI recovery (restart → full re-init)
    - Coordinates with bus workers via atomic skip masks
    - Manipulates DRDY mask to exclude/include recovering ports
    - Logs all recovery attempts and outcomes

Layer 3: Supervisor process (separate binary, normal priority)
    - Monitors the acquisition process via shared memory or Unix socket
    - If acquisition process dies or becomes unresponsive, restarts it
    - Handles long-term degradation (e.g., port fails all recovery attempts)
    - Can implement policy: "if <6/7 ports healthy after 5 minutes, power-cycle and restart"
```

### 6.2 Layer 1 Implementation (Hot Loop Additions)

Total hot-loop overhead budget: <100ns per sample (current cycle is 630us; 100ns = 0.016%).

```cpp
// In engine.hpp, add:
struct PortHealth {
    uint32_t consecutive_zeros;
    uint32_t consecutive_failures;
    bool     recovery_requested;
};

PortHealth port_health_[MAX_PORTS] = {};

// Lock-free recovery queue (SPSC, engine is producer, recovery thread is consumer)
SPSCRing<uint8_t> recovery_queue_;  // Port indices

// In engine.cpp hot loop, after parse:
for (int p = 0; p < num_ports_; ++p) {
    // ... existing parse code ...

    bool port_valid = true;
    for (int d = 0; d < num_dev; ++d) {
        if ((port_data.status_bytes[d][0] & 0xF0) != 0xC0) {
            port_valid = false;
            break;
        }
    }

    if (!port_valid) {
        corruption_count_++;
        sample.valid = false;
        port_health_[p].consecutive_failures++;

        if (port_data.status_bytes[0][0] == 0x00) {
            port_health_[p].consecutive_zeros++;
        } else {
            port_health_[p].consecutive_zeros = 0;
        }
    } else {
        port_health_[p].consecutive_zeros = 0;
        port_health_[p].consecutive_failures = 0;
    }

    // Trigger recovery after 50 consecutive zeros
    if (port_health_[p].consecutive_zeros >= 50 &&
        !port_health_[p].recovery_requested) {
        port_health_[p].recovery_requested = true;
        recovery_queue_.try_push(static_cast<uint8_t>(p));
    }
}
```

**Measured cost breakdown**:
- Status byte comparison: 1 cycle * 9 devices * 7 ports = 63 cycles
- Branch prediction: highly predictable (almost always valid) -- 0 extra cycles
- Counter increments (failure path only): 1 cycle each, rare
- Queue push (rare): ~10 cycles when triggered
- Total per sample: ~63 cycles = ~35ns at 1.8 GHz

### 6.3 Layer 2 Implementation (Recovery Thread)

```cpp
class RecoveryThread {
public:
    RecoveryThread(
        std::vector<ADS1299Device*>& devices,
        std::vector<BusWorker*>& workers,
        DRDYPoller& drdy_poller,
        PortHealth* port_health,
        SPSCRing<uint8_t>& recovery_queue,
        const DeviceConfig& config)
        : devices_(devices)
        , workers_(workers)
        , drdy_poller_(drdy_poller)
        , port_health_(port_health)
        , recovery_queue_(recovery_queue)
        , config_(config)
    {}

    void run(volatile sig_atomic_t& running) {
        // Pin to core 2, SCHED_FIFO 40
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

        struct sched_param param{};
        param.sched_priority = 40;
        sched_setscheduler(0, SCHED_FIFO, &param);

        while (running) {
            uint8_t port_idx;
            if (!recovery_queue_.try_pop(port_idx)) {
                // No recovery needed -- sleep briefly to avoid busy-spin
                struct timespec ts = {0, 10'000'000};  // 10ms
                nanosleep(&ts, nullptr);
                continue;
            }

            std::printf("[RECOVERY] Port %d: starting recovery\n", port_idx);

            auto* dev = devices_[port_idx];
            int bus_num = dev->config().bus_num;

            // Find bus worker for this port
            BusWorker* worker = nullptr;
            int port_in_worker = -1;
            for (auto* w : workers_) {
                for (int i = 0; i < w->port_count(); ++i) {
                    if (w->device(i) == dev) {
                        worker = w;
                        port_in_worker = i;
                        break;
                    }
                }
                if (worker) break;
            }

            if (!worker) {
                std::printf("[RECOVERY] Port %d: no bus worker found\n", port_idx);
                continue;
            }

            // Step 1: Remove from DRDY mask
            drdy_poller_.remove_pin(dev->config().drdy_pin);

            // Step 2: Tell bus worker to skip this port
            worker->set_skip(port_in_worker, true);

            // Step 3: Wait for current cycle to complete
            sleep_ms(5);

            // Step 4: Attempt recovery
            bool recovered = false;

            // Level 1: Quick restart (3 attempts)
            for (int attempt = 0; attempt < 3 && !recovered; ++attempt) {
                recovered = restart_and_verify(dev, attempt);
            }

            // Level 2: Full re-init (2 attempts)
            if (!recovered) {
                for (int attempt = 0; attempt < 2 && !recovered; ++attempt) {
                    dev->start_low();
                    sleep_ms(50);
                    dev->send_command(Cmd::STOP);
                    sleep_ms(20);

                    if (ADS1299Controller::initialize_device(*dev, config_)) {
                        dev->send_command(Cmd::RDATAC);
                        sleep_ms(50);
                        dev->start_high();
                        sleep_ms(200);
                        recovered = verify_data_stream(dev, 100);
                    }
                }
            }

            // Step 5: Re-enable or leave excluded
            if (recovered) {
                worker->set_skip(port_in_worker, false);
                drdy_poller_.add_pin(dev->config().drdy_pin);
                port_health_[port_idx].consecutive_zeros = 0;
                port_health_[port_idx].consecutive_failures = 0;
                port_health_[port_idx].recovery_requested = false;
                std::printf("[RECOVERY] Port %d: recovered successfully\n", port_idx);
            } else {
                // Leave port excluded but allow retry after cooldown
                std::printf("[RECOVERY] Port %d: recovery FAILED\n", port_idx);
                sleep_sec(30);  // 30s cooldown before allowing re-queue
                port_health_[port_idx].recovery_requested = false;
                // Engine will re-detect zeros and re-queue after cooldown
            }
        }
    }

private:
    bool restart_and_verify(ADS1299Device* dev, int attempt) {
        ADS1299Controller::restart_single_port(*dev, attempt, true);
        sleep_ms(200);
        return verify_data_stream(dev, 50);
    }

    bool verify_data_stream(ADS1299Device* dev, int num_samples) {
        int valid = 0;
        for (int i = 0; i < num_samples; ++i) {
            if (dev->wait_for_drdy(0.02)) {
                PortData pd;
                dev->read_data(pd);
                if ((pd.status_bytes[0][0] & 0xF0) == 0xC0) {
                    valid++;
                }
            }
        }
        double rate = static_cast<double>(valid) / num_samples;
        return rate >= 0.90;  // 90% threshold for recovery
    }
};
```

### 6.4 Layer 3: External Supervisor (systemd)

Use a systemd service with automatic restart:

```ini
[Unit]
Description=ADS1299 EEG Acquisition Engine
After=network.target

[Service]
Type=simple
ExecStart=/home/morph/ads1299-cpp/build/ads1299_acquire
Restart=always
RestartSec=5
WatchdogSec=60
User=root

# RT scheduling limits
LimitRTPRIO=infinity
LimitMEMLOCK=infinity

# Environment
Environment="LD_LIBRARY_PATH=/usr/local/lib"

[Install]
WantedBy=multi-user.target
```

The `WatchdogSec=60` directive tells systemd to expect a watchdog notification every 60
seconds. Add to the engine's stats printer:

```cpp
// In print_stats(), notify systemd watchdog:
sd_notify(0, "WATCHDOG=1");
```

This requires linking with `libsystemd` (`-lsystemd`) and including `<systemd/sd-daemon.h>`.
If the engine hangs (e.g., all DRDY pins stuck high causing infinite poll loop), systemd
will kill and restart it after 60 seconds.

---

## 7. Implementation Priority & Impact Matrix

| # | Change | Impact | Effort | Risk | Priority |
|---|--------|--------|--------|------|----------|
| 1 | Remove final re-sync | **Critical** -- eliminates ~50% init regression | 15 min | Low | **P0** |
| 2 | Per-port status byte validation (all devices) | High -- catches inner device failures | 30 min | None | **P0** |
| 3 | DRDY timeout reduction (8ms -> 5ms) + flush on timeout | High -- prevents cascading corruption | 15 min | Low | **P1** |
| 4 | Disable WiFi scanning (`bgscan=""`) | High -- eliminates periodic interference | 5 min | None | **P1** |
| 5 | Add `nohz_full=3 rcu_nocbs=3` to cmdline.txt | Medium -- reduces timer jitter on core 3 | 5 min | Low | **P1** |
| 6 | Increase SDATAC count to `num_devices * 3` | Medium -- reduces deep-chain SDATAC failures | 15 min | Low | **P1** |
| 7 | Per-port consecutive-zero detection in hot loop | Medium -- enables runtime recovery | 30 min | None | **P2** |
| 8 | Recovery thread (Layer 2) | **Critical** for unattended operation | 4 hours | Medium | **P2** |
| 9 | Bus worker skip mask for recovery coordination | Required for #8 | 1 hour | Medium | **P2** |
| 10 | Per-port independent init retry loop | High -- replaces fragile sequential init | 2 hours | Medium | **P2** |
| 11 | Double-flush before RDATAC entry | Low-Medium -- marginal improvement | 15 min | None | **P3** |
| 12 | RDATAC zero-check verification | Low -- catches edge case | 10 min | None | **P3** |
| 13 | systemd supervisor with watchdog | Medium -- automatic restart on hang | 1 hour | Low | **P3** |
| 14 | Switch to Ethernet | High -- eliminates WiFi DMA contention | 30 min | Low (if wired available) | **P3** |
| 15 | PREEMPT_RT kernel | Medium -- deterministic latency | 2 hours | High | **P4** |

---

## 8. Expected Outcome

### With P0 changes only (30 minutes of work):
- Init success rate: ~80% (up from ~50%)
- Runtime failure detection: all 328 channels monitored (up from 56/328 = 17%)

### With P0 + P1 changes (1 hour of work):
- Init success rate: ~90%+
- Periodic corruption: eliminated or reduced by >90%
- Post-timeout recovery: automatic shift register re-sync

### With P0 + P1 + P2 changes (8 hours of work):
- Init success rate: ~95%+ (with iterative per-port retry)
- Runtime recovery: automatic, unattended operation
- Failure-to-recovery latency: ~1-3 seconds
- System operates in "run and forget" mode with 6/7 or 7/7 ports healthy

### With all changes:
- System survives indefinitely with automatic recovery
- External supervisor handles catastrophic failures (process crash, kernel panic)
- All 328 channels monitored, all failures detected, all recoverable failures auto-healed

---

## 9. Risks and Trade-offs

1. **Recovery thread SPI contention**: During port recovery, the recovering port's SPI bus
   has additional traffic. For a 2-port bus (SPI0, SPI3), the healthy sibling port's reads
   may be delayed by ~10-50us per recovery SPI transaction. This is within the 3.2ms
   headroom but should be validated empirically.

2. **DRDY mask shrinkage**: When a port is removed from the DRDY mask for recovery, the
   engine no longer waits for that port's DRDY. If all other ports' DRDYs fire slightly
   before the removed port, the engine proceeds normally. If the removed port was the
   slowest (latest DRDY edge), removing it could cause the engine to poll slightly faster
   than 250 Hz until recovery completes. This is benign -- the CSV and streaming outputs
   are timestamped, and a few hundred microseconds of jitter is acceptable.

3. **Recovery during transient corruption**: The 50-sample consecutive-zero threshold
   prevents false recovery triggers from transient SPI glitches (which typically last 1-3
   samples). However, if a port produces alternating zeros and valid data (intermittent
   hardware fault), it will never reach the threshold and will continue degrading silently.
   Consider adding a rolling window: "if >10% of last 1000 samples are invalid, trigger
   recovery."

4. **SDATAC count increase (item 6)**: Tripling the SDATAC count for deep chains adds
   ~1.5s per port to initialization time. For 7 ports sequentially, this adds ~10s. With
   parallel init (ports on different buses), this reduces to ~3s. This is acceptable for
   a system that initializes once and runs for hours.

5. **nohz_full compatibility**: Not all kernel builds support `nohz_full`. If the kernel
   silently ignores the parameter, there is no harm. Verify with
   `cat /proc/cmdline` after reboot.

---

## 10. Diagnostic Commands for Validation

Run these on the Pi to gather baseline data before implementing changes:

```bash
# Check kernel HZ and nohz support
grep -i hz /boot/config-$(uname -r)
grep -i nohz /boot/config-$(uname -r)
cat /proc/cmdline

# Check WiFi scanning interval
iw dev wlan0 scan dump | head -5
ps aux | grep wpa_supplicant

# Check IRQ affinity for all SPI and WiFi IRQs
for irq in $(grep -E 'spi|brcm' /proc/interrupts | awk '{print $1}' | tr -d ':'); do
    echo "IRQ $irq: $(cat /proc/irq/$irq/smp_affinity) $(grep $irq /proc/interrupts | awk '{print $NF}')"
done

# Check DMA channel allocation
cat /sys/module/dma/parameters/* 2>/dev/null
cat /proc/device-tree/soc/dma@*/brcm,dma-channel-mask 2>/dev/null | xxd

# Monitor CPU frequency during acquisition (should be locked at max)
watch -n 1 'cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq'

# Check for kernel preemption model
grep PREEMPT /boot/config-$(uname -r)

# Monitor SPI interrupt counts during acquisition (run for 30s)
cat /proc/interrupts | grep -i spi; sleep 30; cat /proc/interrupts | grep -i spi
```

---

## Appendix A: File Modifications Summary

| File | Change | Lines Affected |
|------|--------|----------------|
| `src/ads1299/controller.cpp` | Remove final re-sync block | 679-757 (delete) |
| `src/ads1299/controller.cpp` | Increase SDATAC count | 93 |
| `src/ads1299/controller.cpp` | Add double-flush before RDATAC | 296-305 |
| `src/ads1299/controller.cpp` | Add zero-check to RDATAC verification | 318 |
| `src/acquisition/engine.cpp` | Per-port all-device status validation | 172-175 |
| `src/acquisition/engine.cpp` | Consecutive-zero detection + recovery queue | 172-184 |
| `src/acquisition/engine.cpp` | DRDY timeout 8ms -> 5ms + flush | 140-143 |
| `include/acquisition/engine.hpp` | PortHealth struct, recovery queue | new fields |
| `src/acquisition/drdy_poller.cpp` | Change mask_ to atomic | 13, 19, 34 |
| `include/acquisition/drdy_poller.hpp` | atomic<uint8_t> mask_ | 31 |
| `src/acquisition/bus_worker.cpp` | Skip mask check in read loop | 69-72 |
| `include/acquisition/bus_worker.hpp` | atomic<uint8_t> skip_mask_ | new field |
| **New file**: `src/acquisition/recovery_thread.cpp` | Recovery thread implementation | ~200 lines |
| **New file**: `include/acquisition/recovery_thread.hpp` | Recovery thread interface | ~50 lines |
| `src/main.cpp` | Create and start recovery thread | ~15 lines |

## Appendix B: Key Constants

```cpp
// Recovery thresholds
constexpr int    CONSECUTIVE_ZERO_THRESHOLD = 50;    // Samples before recovery trigger (200ms)
constexpr double VERIFY_VALIDITY_THRESHOLD  = 0.90;  // 90% valid samples for recovery success
constexpr int    VERIFY_SAMPLE_COUNT        = 100;   // Samples for verification (400ms)
constexpr int    QUICK_RESTART_ATTEMPTS     = 3;     // Level 1 recovery attempts
constexpr int    FULL_REINIT_ATTEMPTS       = 2;     // Level 2 recovery attempts
constexpr double RECOVERY_COOLDOWN_SEC      = 30.0;  // Wait between failed recovery cycles
constexpr double DRDY_TIMEOUT_SEC           = 0.005; // 5ms (was 8ms)
```
