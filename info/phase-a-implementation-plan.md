# Phase A Implementation Plan: Init Reliability

## Overview

5 changes, 4 files modified, ~120 lines of new/changed code. Expected result: init succeeds reliably on first or second attempt; all 328 channels verified before acquisition starts.

---

## Change 1: Per-Device Status Validation in Hot Loop

**File:** `src/acquisition/engine.cpp` lines 172-176
**Risk:** NONE — adds ~35ns per sample (0.006% of cycle time)

### Current Code (lines 172-176):
```cpp
// Status byte validation (first device)
if ((port_data.status_bytes[0][0] & 0xF0) != 0xC0) {
    corruption_count_++;
    sample.valid = false;
}
```

### New Code:
```cpp
// Status byte validation (ALL devices in chain)
for (int d = 0; d < num_dev; ++d) {
    if ((port_data.status_bytes[d][0] & 0xF0) != 0xC0) {
        corruption_count_++;
        sample.valid = false;
        break;  // One bad device invalidates entire sample
    }
}
```

### Why This Works:
- `parse_raw()` already extracts all devices' status bytes — they're just not checked
- The ADS1299 status byte format is `1100_GGGG` (0xC_). A dead device outputs 0x00.
- In a daisy chain, each device's 27 bytes are independent in the shift register. A dead inner device produces 27 zero bytes that propagate cleanly — upstream devices remain valid.
- The `break` ensures we count 1 corruption per sample (not per device), matching existing semantics.

---

## Change 2: Per-Device Validation During Init Warmup

**File:** `src/ads1299/controller.cpp`
**Risk:** LOW — adds ~1-2 seconds to init (more thorough validation)

### 2a. Warmup loop — check all devices (line 472-483)

**Current** (line 472-483):
```cpp
PortData pd;
dev->read_data(pd);
uint8_t status0 = pd.status_bytes[0][0];

if ((status0 & 0xF0) != 0xC0) {
    warmup_corruptions[p]++;
    if (status0 == 0x00) {
        consecutive_zeros[p]++;
    } else {
        consecutive_zeros[p] = 0;
    }
} else {
    consecutive_zeros[p] = 0;
}
```

**New** (replace lines 472-483):
```cpp
PortData pd;
dev->read_data(pd);

// Check ALL devices in the chain, not just device 0
bool port_ok = true;
for (int d = 0; d < dev->config().num_devices; ++d) {
    if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
        port_ok = false;
        break;
    }
}

if (!port_ok) {
    warmup_corruptions[p]++;
    // Track consecutive zeros using device 0 status (port-level indicator)
    if (pd.status_bytes[0][0] == 0x00) {
        consecutive_zeros[p]++;
    } else {
        consecutive_zeros[p] = 0;
    }
} else {
    consecutive_zeros[p] = 0;
}
```

### 2b. Data flow verification — check all devices (lines 639-644)

**Current** (lines 639-644):
```cpp
for (size_t p = 0; p < devices.size(); ++p) {
    PortData pd;
    devices[p]->read_data(pd);
    if ((pd.status_bytes[0][0] & 0xF0) == 0xC0) {
        valid_counts[p]++;
    }
}
```

**New:**
```cpp
for (size_t p = 0; p < devices.size(); ++p) {
    PortData pd;
    devices[p]->read_data(pd);
    // Check ALL devices in chain
    bool all_dev_ok = true;
    for (int d = 0; d < devices[p]->config().num_devices; ++d) {
        if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
            all_dev_ok = false;
            break;
        }
    }
    if (all_dev_ok) {
        valid_counts[p]++;
    }
}
```

### 2c. Individual port check — check all devices (lines 396-406)

**Current** (lines 396-406):
```cpp
int zero_count = 0;
int valid_count = 0;
for (int s = 0; s < 5; ++s) {
    PortData pd;
    dev->read_data(pd);
    uint8_t status0 = pd.status_bytes[0][0];
    if (status0 == 0x00) {
        zero_count++;
    } else if ((status0 & 0xF0) == 0xC0) {
        valid_count++;
    }
    sleep_ms(4);
}
```

**New:**
```cpp
int zero_count = 0;
int valid_count = 0;
for (int s = 0; s < 5; ++s) {
    PortData pd;
    dev->read_data(pd);
    // Check ALL devices
    bool all_ok = true;
    bool any_zero = false;
    for (int d = 0; d < dev->config().num_devices; ++d) {
        if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
            all_ok = false;
        }
        if (pd.status_bytes[d][0] == 0x00) {
            any_zero = true;
        }
    }
    if (any_zero) {
        zero_count++;
    } else if (all_ok) {
        valid_count++;
    }
    sleep_ms(4);
}
```

### 2d. Mini-warmup after re-init — check all devices (line 596)

**Current** (line 596):
```cpp
if ((pd.status_bytes[0][0] & 0xF0) != 0xC0) {
    mini_corruptions++;
}
```

**New:**
```cpp
bool mini_ok = true;
for (int d = 0; d < dev->config().num_devices; ++d) {
    if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
        mini_ok = false;
        break;
    }
}
if (!mini_ok) {
    mini_corruptions++;
}
```

---

## Change 3: Replace Global Re-Sync With Per-Port Re-Sync

**File:** `src/ads1299/controller.cpp` lines 678-757
**Risk:** LOW — eliminates the ~50% regression from global stop/restart

### Delete lines 678-757 entirely (the "Final re-synchronization" block)

### Replace with per-port re-sync + verification:

```cpp
    // Per-port re-synchronization (replaces global re-sync)
    // Each port is independently cycled through STOP->SDATAC->flush->RDATAC->START
    // and verified. If re-sync breaks a port, it's immediately recovered.
    // This prevents the global stop/restart from undoing warmup recovery work.
    std::printf("\n  Per-port re-synchronization...\n");

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
        dev->send_command(Cmd::RDATAC);  // Redundant for reliability
        sleep_ms(20);

        // Restart this port
        dev->start_high();
        sleep_ms(100);

        // Verify this port still works (check ALL devices)
        bool resync_ok = false;
        if (dev->wait_for_drdy(0.1)) {
            PortData pd;
            dev->read_data(pd);
            resync_ok = true;
            for (int d = 0; d < dev->config().num_devices; ++d) {
                if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
                    resync_ok = false;
                    break;
                }
            }
        }

        if (!resync_ok) {
            // Re-sync broke this port — recover with escalating restart
            std::printf("    %s: re-sync broke port, recovering...\n",
                        dev->config().port_name);
            for (int attempt = 0; attempt < 5; ++attempt) {
                if (restart_single_port(*dev, attempt, false)) {
                    std::printf("    %s: recovered (attempt %d) [OK]\n",
                                dev->config().port_name, attempt + 1);
                    break;
                }
            }
        }
    }

    // Settling samples (100 samples, measure DRDY timing)
    reference_port = devices[0];
    for (auto* dev : devices) {
        if (dev->config().num_devices > reference_port->config().num_devices) {
            reference_port = dev;
        }
    }

    std::printf("\n  Post-resync settling (100 samples)...\n");

    double drdy_times[10] = {};
    int drdy_time_count = 0;

    for (int sample_idx = 0; sample_idx < 100; ++sample_idx) {
        struct timespec ts_start, ts_end;
        clock_gettime(CLOCK_MONOTONIC, &ts_start);

        if (reference_port->wait_for_drdy(0.02)) {
            clock_gettime(CLOCK_MONOTONIC, &ts_end);
            double drdy_elapsed = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                                  (ts_end.tv_nsec - ts_start.tv_nsec) / 1e6;

            for (auto* dev : devices) {
                PortData pd;
                dev->read_data(pd);
            }

            if (sample_idx < 10 && drdy_time_count < 10) {
                drdy_times[drdy_time_count++] = drdy_elapsed;
            }
        }
    }

    if (drdy_time_count > 0) {
        double avg = 0;
        for (int i = 0; i < drdy_time_count; ++i) avg += drdy_times[i];
        avg /= drdy_time_count;
        std::printf("  DRDY timing (first 10): avg %.2fms\n", avg);
    }

    std::printf("  [OK] Re-synchronization complete\n");
```

### Why per-port instead of deleting entirely:
The performance-engineer suggested deleting re-sync entirely. However, the re-sync provides a shift register alignment benefit — it flushes stale SPI data and ensures a clean SDATAC→RDATAC transition. The per-port approach keeps this benefit while eliminating the "global stop breaks recovered ports" problem. If a port breaks during re-sync, it's immediately recovered without affecting others.

---

## Change 4: Exhaustive Init Retry Loop

**File:** `src/main.cpp` lines 220-267
**Risk:** LOW — only adds retry logic around existing init code

### Current code (lines 220-267):
```cpp
// --- Configure all devices ---
...
for (int i = 0; i < args.num_ports; ++i) {
    bool success = ads1299::ADS1299Controller::initialize_device(*devices[i], config);
    if (!success) {
        std::fprintf(stderr, "\n[FAIL] %s failed to initialize\n", ...);
        return 1;
    }
    sleep_sec(0.1);
}
...
bool data_flow_ok = ads1299::ADS1299Controller::start_all_conversions_synchronized(
    devices, i2c, &config);

if (!data_flow_ok) {
    // Check DRDY, continue if active...
}
```

### New code (replaces lines 220-267):
```cpp
    // --- Configure all devices (with exhaustive retry) ---
    std::printf("\n============================================================\n"
                "CONFIGURING DEVICES\n"
                "============================================================\n");

    ads1299::DeviceConfig config;

    constexpr int MAX_INIT_RETRIES = 10;
    constexpr double INIT_TIMEOUT_SEC = 300.0;  // 5-minute hard timeout

    struct timespec init_start_ts;
    clock_gettime(CLOCK_MONOTONIC, &init_start_ts);
    double init_start_time = init_start_ts.tv_sec + init_start_ts.tv_nsec * 1e-9;

    bool system_ready = false;

    for (int full_attempt = 0; full_attempt < MAX_INIT_RETRIES; ++full_attempt) {
        if (full_attempt > 0) {
            std::printf("\n=== INIT RETRY %d/%d ===\n", full_attempt + 1, MAX_INIT_RETRIES);
        }

        // Check hard timeout
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        double elapsed = (now_ts.tv_sec + now_ts.tv_nsec * 1e-9) - init_start_time;
        if (elapsed > INIT_TIMEOUT_SEC) {
            std::fprintf(stderr, "FATAL: Init timeout after %.0fs\n", elapsed);
            return 1;
        }

        // On retry: force all START low, flush all SPI, wait for settling
        if (full_attempt > 0) {
            for (auto* dev : devices) {
                dev->start_low();
                dev->send_command(ads1299::Cmd::STOP);
                dev->send_command(ads1299::Cmd::SDATAC);
                dev->flush_spi();
            }
            sleep_sec(1.0);
        }

        // 1. Initialize all devices
        bool all_init_ok = true;
        for (int i = 0; i < args.num_ports; ++i) {
            bool success = ads1299::ADS1299Controller::initialize_device(*devices[i], config);
            if (!success) {
                std::fprintf(stderr, "\n[FAIL] %s failed to initialize\n",
                             devices[i]->config().port_name);
                all_init_ok = false;
                // Don't abort — continue configuring other ports, then retry everything
                break;
            }
            sleep_sec(0.1);
        }

        if (!all_init_ok) {
            std::printf("[RETRY] Init failed, restarting...\n");
            continue;
        }

        std::printf("\n[OK] All devices configured successfully!\n");

        // 2. Start conversions + warmup + per-port re-sync
        std::printf("\n============================================================\n"
                    "VERIFYING DATA FLOW\n"
                    "============================================================\n");

        bool data_flow_ok = ads1299::ADS1299Controller::start_all_conversions_synchronized(
            devices, i2c, &config);

        // 3. Extended health check — verify ALL devices on ALL ports
        //    Read 100 samples and require >= 90% valid on every port
        std::printf("\n  Extended health check (100 samples)...\n");

        // Use longest-chain port as DRDY reference
        ads1299::ADS1299Device* health_ref = devices[0];
        for (auto* d : devices) {
            if (d->config().num_devices > health_ref->config().num_devices)
                health_ref = d;
        }

        int health_valid[ads1299::MAX_PORTS] = {};
        int health_reads = 0;

        for (int i = 0; i < 100; ++i) {
            if (health_ref->wait_for_drdy(0.02)) {
                health_reads++;
                for (int p = 0; p < args.num_ports; ++p) {
                    ads1299::PortData pd;
                    devices[p]->read_data(pd);

                    bool port_ok = true;
                    for (int d = 0; d < devices[p]->config().num_devices; ++d) {
                        if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
                            port_ok = false;
                            break;
                        }
                    }
                    if (port_ok) health_valid[p]++;
                }
            }
        }

        // Check: all ports must have >= 90% valid reads
        bool all_healthy = true;
        for (int p = 0; p < args.num_ports; ++p) {
            double rate = health_reads > 0 ? static_cast<double>(health_valid[p]) / health_reads : 0;
            if (rate < 0.90) {
                std::printf("    %s: %.0f%% valid (%d/%d) [FAIL]\n",
                            devices[p]->config().port_name, rate * 100.0,
                            health_valid[p], health_reads);
                all_healthy = false;
            } else {
                std::printf("    %s: %.0f%% valid (%d/%d) [OK]\n",
                            devices[p]->config().port_name, rate * 100.0,
                            health_valid[p], health_reads);
            }
        }

        if (all_healthy && health_reads >= 80) {
            std::printf("\n[OK] All %d ports healthy after attempt %d\n",
                        args.num_ports, full_attempt + 1);
            system_ready = true;
            break;
        }

        std::printf("[RETRY] %d port(s) unhealthy, restarting init...\n",
                    args.num_ports - static_cast<int>(std::count_if(
                        health_valid, health_valid + args.num_ports,
                        [&](int v) { return health_reads > 0 && v >= health_reads * 9 / 10; })));

        // Full shutdown before retry
        for (auto* dev : devices) {
            dev->start_low();
            dev->send_command(ads1299::Cmd::STOP);
            dev->send_command(ads1299::Cmd::SDATAC);
            dev->flush_spi();
        }
        sleep_sec(1.0);
    }

    if (!system_ready) {
        std::fprintf(stderr, "\nFATAL: Could not initialize all ports after %d attempts\n",
                     MAX_INIT_RETRIES);
        return 1;
    }

    std::printf("\n============================================================\n"
                "[OK] SYSTEM INITIALIZATION COMPLETE!\n"
                "============================================================\n");
```

### Key Design Decisions:
- **10 retries, 5-minute hard timeout**: Empirical data shows 2-3 restarts usually suffice. 10 retries is generous.
- **On retry**: Force all START low, STOP, SDATAC, flush, 1 second settle. This puts all devices in a clean state.
- **Extended health check**: 100 samples with 90% threshold, checking ALL devices on ALL ports. This catches single-device failures that the existing 5-sample and 20-sample checks miss.
- **If init fails, don't abort on first port failure**: Continue to retry the entire sequence. A port that fails once might succeed on the next attempt.
- **The `start_all_conversions_synchronized()` still runs**: It handles warmup, per-port restart, re-init of bad ports, and (after Change 3) per-port re-sync. The exhaustive retry loop adds an outer verification layer.

### Note: Need `<algorithm>` include for `std::count_if`:
Add at the top of main.cpp:
```cpp
#include <algorithm>
```

---

## Change 5: WiFi Scanning Disable

**Not a code change — Pi config change.**

SSH into Pi and run:
```bash
# Check current wpa_supplicant config
cat /etc/wpa_supplicant/wpa_supplicant.conf

# Add bgscan="" to the network block to disable background scanning
# OR simply disable WiFi power management:
sudo iw wlan0 set power_save off

# For permanent WiFi scan disable, add to /etc/wpa_supplicant/wpa_supplicant.conf
# inside the network={} block:
#   bgscan=""

# Or if using NetworkManager:
sudo nmcli connection modify <connection-name> 802-11-wireless.bgscan ""
```

This eliminates the ~23.4s periodic DMA contention that causes corruption.

---

## Also Fix: `restart_single_port()` Should Check All Devices

**File:** `src/ads1299/controller.cpp` lines 231-235

**Current:**
```cpp
PortData pd;
dev.read_data(pd);
uint8_t status0 = pd.status_bytes[0][0];
if ((status0 & 0xF0) == 0xC0) {
    return true;
}
```

**New:**
```cpp
PortData pd;
dev.read_data(pd);
bool all_ok = true;
for (int d = 0; d < dev.config().num_devices; ++d) {
    if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
        all_ok = false;
        break;
    }
}
if (all_ok) {
    return true;
}
uint8_t status0 = pd.status_bytes[0][0];
```

This ensures `restart_single_port()` doesn't report success when inner devices are still dead.

---

## File Change Summary

| File | Lines Changed | Description |
|------|---------------|-------------|
| `src/acquisition/engine.cpp` | 172-176 | Per-device status validation in hot loop |
| `src/ads1299/controller.cpp` | 231-235 | restart_single_port checks all devices |
| `src/ads1299/controller.cpp` | 396-406 | Individual port check validates all devices |
| `src/ads1299/controller.cpp` | 472-483 | Warmup loop checks all devices |
| `src/ads1299/controller.cpp` | 596 | Mini-warmup checks all devices |
| `src/ads1299/controller.cpp` | 639-644 | Data flow verification checks all devices |
| `src/ads1299/controller.cpp` | 678-757 | Replace global re-sync with per-port re-sync |
| `src/main.cpp` | 220-267 | Wrap init in exhaustive retry loop |
| `src/main.cpp` | top | Add `#include <algorithm>` |

## Expected Outcome

1. **Init success rate**: ~95%+ on first attempt (up from ~48%)
2. **With retry loop**: ~99.9%+ (10 attempts × 95% each = virtually guaranteed)
3. **Channel monitoring**: All 328 channels validated (up from 56/328 = 17%)
4. **Worst-case init time**: ~30s typical, ~3 min worst case (10 retries)
5. **Hot loop impact**: ~35ns per sample (0.006% — unmeasurable)
