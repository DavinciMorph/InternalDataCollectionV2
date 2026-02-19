# ADS1299 Init Redesign: 4-Phase Architecture

## What Changed and Why

The old init sequence had three critical problems:

1. **All-or-nothing retries**: When a single port failed the RDATAC timing race, the
   entire system (all 7 ports) was torn down and reconfigured from scratch. This
   wasted 5-10 seconds per attempt even though register configuration always succeeds
   and the other 6 ports were already healthy.

2. **Uncancellable**: The retry loop ran under `SCHED_FIFO` on an isolated core and
   did not check the `volatile sig_atomic_t running` flag between steps. Ctrl+C was
   blocked for up to 5 minutes while the loop exhausted its retries.

3. **Wrong recovery target**: The ~10% per-port RDATAC timing race is the ONLY failure
   mode after registers are configured. The output pipeline inside the ADS1299 fails
   to connect to the SPI shift register during the RDATAC entry transition. Full
   re-init (reset, SDATAC, register writes, RDATAC, START) does not improve the odds
   -- the failure is specifically in the RDATAC->START transition, and the fix is simply
   to cycle STOP->SDATAC->flush->RDATAC->START on the one failing port (~200ms).

## New 4-Phase Architecture

**Phase 1: Configure registers (do once, always succeeds)**
- Software reset, SDATAC verify, device ID check, register writes + readback verify
- `ADS1299Controller::initialize_device()` per port
- If this fails, it is a hardware problem -- abort immediately

**Phase 2: Start conversions + verify (probabilistic)**
- `ADS1299Controller::start_and_verify()` sends RDATAC to all, asserts START atomically
  via TCA9534MultiPin, then reads 10 samples per port and checks status bytes
- Returns per-port `PortHealth` structs -- does NOT retry anything
- `g_running` is checked between every step so Ctrl+C works

**Phase 3: Per-port recovery (only failing ports)**
- For each unhealthy port: `ADS1299Controller::recover_port()` up to 15 times
- Each attempt cycles STOP->SDATAC->flush->RDATAC->START (~200ms)
- Escalating delays (attempt N scales delays by N+1)
- Healthy ports are never touched
- Dead ports (15 failures) are shut down and skipped; system continues with remaining

**Phase 4: Warmup + final health check**
- Discard 100 settling samples from all active ports
- Read 50 samples and verify >=90% valid status bytes per port
- Ports that regressed to 0% valid get 5 more recovery attempts
- Final summary: active ports, dead ports, system ready or abort

## Additional Fix: SPI5 Core Reassignment

SPI5 (Port7) was previously assigned to CPU core 0 alongside SPI0 (Port1+Port2).
Port1's 9-device deep daisy chain has the tightest timing margin, and sharing core 0
with SPI5 added ~252us of contention delay. SPI5 is now assigned to core 2 (shared
with SPI4), keeping core 0 as the sole owner of SPI0's time-critical path.

```
Core assignments:
  SPI0 (Port1+2) -> core 0  (sole owner, fastest path to Port1's 9-deep chain)
  SPI3 (Port3+4) -> core 1
  SPI4 (Port5+6) -> core 2
  SPI5 (Port7)   -> core 2  (was core 0, moved to avoid SPI0 contention)
```

---

## Complete Source Files

Copy-paste these directly onto the Pi. File paths are relative to the C++ project root
(`~/ads1299-cpp/` on the Pi, or `Cpp Implementation/` in the repo).

---

### `include/ads1299/controller.hpp`

```cpp
#pragma once

#include "ads1299/registers.hpp"
#include "ads1299/types.hpp"

#include <csignal>
#include <vector>

namespace ads1299 {

class ADS1299Device;
class TCA9534MultiPin;
class I2CDevice;

// Per-port health status after start_and_verify
struct PortHealth {
    bool healthy;           // true if port is producing valid data
    bool drdy_active;       // true if DRDY is toggling
    int  valid_samples;     // out of verification_samples
    int  total_samples;     // total DRDY events captured
};

// ADS1299 device initialization and control logic.
//
// Init strategy (redesigned 2026-02-18):
//   1. initialize_device() — register configuration. Always succeeds. Do once.
//   2. start_and_verify()  — RDATAC + START + verify. Returns per-port health.
//   3. recover_port()      — STOP->SDATAC->flush->RDATAC->START for ONE port.
//      Called in a loop from main.cpp for each failing port individually.
//
// The RDATAC timing race (~10% per-port failure rate) is the ONLY failure mode
// after step 1. Full re-init is never needed — just cycle the RDATAC transition.
class ADS1299Controller {
public:
    // Initialize a single ADS1299 port (full reset, SDATAC verify, register write+verify).
    // Returns true on success. This always succeeds if hardware is present.
    static bool initialize_device(ADS1299Device& dev, const DeviceConfig& config);

    // Start all conversions with synchronized START, then verify each port.
    // Does NOT retry or recover anything — just reports per-port health.
    // Caller (main.cpp) handles per-port recovery.
    //
    // healthy_out: array of PortHealth, one per device. Must have devices.size() entries.
    // running: checked between steps so Ctrl+C works during startup.
    static void start_and_verify(
        std::vector<ADS1299Device*>& devices,
        I2CDevice& i2c,
        PortHealth* healthy_out,
        const volatile sig_atomic_t& running);

    // Recover a single port by cycling STOP->SDATAC->flush->RDATAC->START.
    // Reads verify_samples after restart and checks status bytes.
    // Returns true if port is now producing valid data.
    // This is fast (~200ms) and targets the RDATAC timing race specifically.
    static bool recover_port(ADS1299Device& dev, int attempt = 0,
                             int verify_samples = 5);

    // Force all START pins LOW
    static void force_all_start_pins_low(std::vector<ADS1299Device*>& devices);

    // Send command to all devices
    static void broadcast_command(std::vector<ADS1299Device*>& devices,
                                  uint8_t cmd, double delay_ms = 10.0);

    // Create a TCA9534MultiPin for all START pins
    static TCA9534MultiPin create_start_controller(
        std::vector<ADS1299Device*>& devices, I2CDevice& i2c);

    // Legacy — kept for backward compatibility but should not be used
    static bool restart_single_port(ADS1299Device& dev, int attempt = 0,
                                    bool verbose = true);

private:
    // Write register and verify readback (with retry)
    static bool write_and_verify(ADS1299Device& dev, Reg reg, uint8_t value,
                                 const char* name, int max_attempts = 100);

    // Verify register matches expected value (with retry + SDATAC recovery)
    static bool verify_register(ADS1299Device& dev, Reg reg, uint8_t expected,
                                const char* name, uint8_t& actual);

    // Check if a single port is producing valid data (reads N samples, checks status).
    // Returns number of valid samples out of N.
    static int verify_port_data(ADS1299Device& dev, int num_samples);
};

} // namespace ads1299
```

---

### `src/ads1299/controller.cpp`

```cpp
#include "ads1299/controller.hpp"
#include "ads1299/spi_device.hpp"
#include "hardware/spi_bus.hpp"
#include "hardware/tca9534.hpp"
#include "hardware/i2c_device.hpp"

#include <cstdio>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include <algorithm>

namespace ads1299 {

// --- Helpers ---

static void sleep_sec(double sec) {
    if (sec <= 0) return;
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(sec);
    ts.tv_nsec = static_cast<long>((sec - ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
}

static void sleep_ms(double ms) {
    sleep_sec(ms / 1000.0);
}

// --- ADS1299Controller: Register helpers ---

bool ADS1299Controller::write_and_verify(ADS1299Device& dev, Reg reg, uint8_t value,
                                          const char* name, int max_attempts) {
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        dev.write_register(reg, value);
        sleep_ms(10);
        uint8_t readback = dev.read_register(reg);
        if (readback == value) {
            if (attempt > 0) {
                std::printf("    %s verified on attempt %d\n", name, attempt + 1);
            }
            return true;
        }
        sleep_ms(50);  // Brief pause before retry
    }
    uint8_t readback = dev.read_register(reg);
    std::printf("  %s failed after %d attempts: wrote 0x%02X, read 0x%02X\n",
                name, max_attempts, value, readback);
    return false;
}

bool ADS1299Controller::verify_register(ADS1299Device& dev, Reg reg, uint8_t expected,
                                         const char* /*name*/, uint8_t& actual) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        actual = dev.read_register(reg);
        if (actual == expected) return true;
        sleep_ms(20);
        // Send SDATAC in case device slipped back
        dev.send_command(Cmd::SDATAC);
        sleep_ms(20);
    }
    return false;
}

// --- ADS1299Controller: Device initialization ---
// This configures registers only. Always succeeds if hardware is present.
// Does NOT enter RDATAC or assert START.

bool ADS1299Controller::initialize_device(ADS1299Device& dev, const DeviceConfig& config) {
    std::printf("\nConfiguring %s...\n", dev.config().port_name);

    int num_devices = dev.config().num_devices;

    // 1. Ensure START is low
    dev.start_low();
    sleep_ms(50);

    // 2. Flush SPI and send STOP to clear any stale state
    dev.flush_spi();
    dev.send_command(Cmd::STOP);
    sleep_ms(10);

    // 3. Multiple software RESETs to ensure all devices settle properly
    double reset_time = 0.1 + (num_devices * 0.01);
    int num_resets = 2;
    std::printf("  Performing %d software resets (%.1fs each for %d devices)...\n",
                num_resets, reset_time, num_devices);
    for (int r = 0; r < num_resets; ++r) {
        dev.send_command(Cmd::RESET);
        sleep_sec(reset_time);
        // Send SDATAC between resets to clear any pending data mode
        dev.send_command(Cmd::SDATAC);
        sleep_ms(50);
    }
    std::printf("  Reset cycles complete\n");

    // 4. SDATAC to enable register access
    int sdatac_count = (num_devices + 2 > 5) ? num_devices + 2 : 5;
    for (int i = 0; i < sdatac_count; ++i) {
        dev.send_command(Cmd::SDATAC);
        sleep_ms(20);
    }
    sleep_ms(100);  // Extra settling for long chains

    // 4b. Verify SDATAC with LOFF write/read test - retry until it works
    uint8_t test_val = 0xAA;
    bool sdatac_ok = false;
    uint8_t readback = 0;
    for (int attempt = 0; attempt < 100; ++attempt) {
        dev.write_register(Reg::LOFF, test_val);
        sleep_ms(100);
        readback = dev.read_register(Reg::LOFF);
        if (readback == test_val) {
            sdatac_ok = true;
            if (attempt > 0) {
                std::printf("  SDATAC verified on attempt %d [OK]\n", attempt + 1);
            } else {
                std::printf("  SDATAC verified [OK]\n");
            }
            break;
        }
        // Retry - send more SDATAC commands
        for (int s = 0; s < 3; ++s) {
            dev.send_command(Cmd::SDATAC);
            sleep_ms(20);
        }
        sleep_ms(100);
    }

    if (!sdatac_ok) {
        std::printf("  SDATAC verification failed after 100 attempts: wrote 0x%02X, read 0x%02X\n",
                    test_val, readback);
        return false;
    }

    // Restore LOFF
    dev.write_register(Reg::LOFF, 0x00);
    sleep_ms(100);

    // 5. Verify device ID - retry if corrupted read
    bool id_ok = false;
    uint8_t id_value = 0;
    for (int attempt = 0; attempt < 100; ++attempt) {
        id_value = dev.read_register(Reg::ID);
        if (id_value == ADS1299_DEVICE_ID) {
            id_ok = true;
            if (attempt > 0) {
                std::printf("  Device ID: 0x%02X (attempt %d)\n", id_value, attempt + 1);
            } else {
                std::printf("  Device ID: 0x%02X\n", id_value);
            }
            break;
        }
        sleep_ms(50);
    }

    if (!id_ok) {
        std::printf("  Device ID: 0x%02X\n", id_value);
        std::printf("  [FAIL] Expected ID 0x3E after 100 attempts\n");
        return false;
    }
    std::printf("  [OK] ADS1299 detected\n");

    // 6. Write CONFIG registers - retry each until it sticks
    std::printf("  Writing configuration...\n");

    // CONFIG3 first - enables internal reference
    if (!write_and_verify(dev, Reg::CONFIG3, config.config3, "CONFIG3")) return false;

    // Wait for reference buffer to stabilize
    std::printf("  Waiting for reference buffer...\n");
    sleep_ms(200);

    // CONFIG1 - sample rate and daisy settings
    if (!write_and_verify(dev, Reg::CONFIG1, config.config1, "CONFIG1")) return false;

    // CONFIG2 - test signal settings
    if (!write_and_verify(dev, Reg::CONFIG2, config.config2, "CONFIG2")) return false;

    // 7. Write channel settings - retry each until it sticks
    auto ch_settings = config.get_channel_settings();
    for (int i = 0; i < 8; ++i) {
        char name[16];
        std::snprintf(name, sizeof(name), "CH%dSET", i + 1);
        Reg reg = static_cast<Reg>(static_cast<uint8_t>(Reg::CH1SET) + i);
        if (!write_and_verify(dev, reg, ch_settings[i], name)) return false;
    }

    // 8. Write MISC1 and CONFIG4
    if (!write_and_verify(dev, Reg::MISC1, config.misc1, "MISC1")) return false;
    if (!write_and_verify(dev, Reg::CONFIG4, config.config4, "CONFIG4")) return false;

    // 9. Final verification - re-verify each CONFIG with retry if needed
    uint8_t c1_read, c2_read, c3_read;
    bool c1_ok = verify_register(dev, Reg::CONFIG1, config.config1, "CONFIG1", c1_read);
    bool c2_ok = verify_register(dev, Reg::CONFIG2, config.config2, "CONFIG2", c2_read);
    bool c3_ok = verify_register(dev, Reg::CONFIG3, config.config3, "CONFIG3", c3_read);

    std::printf("  CONFIG1=0x%02X%s CONFIG2=0x%02X%s CONFIG3=0x%02X%s\n",
                c1_read, c1_ok ? "[OK]" : "[FAIL]",
                c2_read, c2_ok ? "[OK]" : "[FAIL]",
                c3_read, c3_ok ? "[OK]" : "[FAIL]");

    if (c1_ok && c2_ok && c3_ok) {
        std::printf("  [OK] %s configured successfully\n", dev.config().port_name);
        return true;
    }

    std::printf("  [FAIL] %s configuration failed\n", dev.config().port_name);
    return false;
}

// --- ADS1299Controller: Port data verification ---
// Reads num_samples from a single port, returns count of valid samples
// (where ALL devices in the daisy chain have status byte high nibble == 0xC0).

int ADS1299Controller::verify_port_data(ADS1299Device& dev, int num_samples) {
    int valid = 0;
    for (int i = 0; i < num_samples; ++i) {
        if (!dev.wait_for_drdy(0.02)) {
            continue;  // DRDY timeout — count as invalid
        }
        PortData pd;
        dev.read_data(pd);

        bool sample_ok = true;
        for (int d = 0; d < dev.config().num_devices; ++d) {
            if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
                sample_ok = false;
                break;
            }
        }
        if (sample_ok) {
            valid++;
        }
    }
    return valid;
}

// --- ADS1299Controller: Start and verify (no recovery) ---
// Sends RDATAC to all ports, asserts START atomically, then verifies
// each port independently. Returns per-port health in healthy_out.
// Does NOT attempt any recovery — that is the caller's job.

void ADS1299Controller::start_and_verify(
    std::vector<ADS1299Device*>& devices,
    I2CDevice& i2c,
    PortHealth* healthy_out,
    const volatile sig_atomic_t& running) {

    int num_ports = static_cast<int>(devices.size());

    // Initialize all health entries to unhealthy
    for (int i = 0; i < num_ports; ++i) {
        healthy_out[i] = {false, false, 0, 0};
    }

    // 1. Stop everything cleanly
    std::printf("  Stopping any ongoing conversions...\n");
    force_all_start_pins_low(devices);
    broadcast_command(devices, static_cast<uint8_t>(Cmd::STOP), 10);

    if (!running) return;

    // 2. Ensure SDATAC on all devices
    std::printf("  Ensuring all devices in SDATAC mode...\n");
    broadcast_command(devices, static_cast<uint8_t>(Cmd::SDATAC), 50);

    if (!running) return;

    // 3. Flush all SPI shift registers
    for (auto* dev : devices) {
        dev->flush_spi();
    }
    sleep_ms(10);

    // 4. Send RDATAC to all devices (fast sequential, twice for reliability)
    std::printf("  Sending RDATAC to all devices...\n");
    for (auto* dev : devices) {
        dev->send_command(Cmd::RDATAC);
    }
    sleep_ms(50);
    for (auto* dev : devices) {
        dev->send_command(Cmd::RDATAC);
    }
    sleep_ms(100);

    if (!running) return;

    // 5. Assert START on all ports atomically via TCA9534MultiPin
    {
        TCA9534MultiPin start_controller(i2c);
        for (auto* dev : devices) {
            start_controller.add_pin(dev->config().start_i2c_addr, dev->config().start_pin);
        }
        start_controller.set_all_high();
    }
    std::printf("  [All START pins asserted simultaneously]\n");

    // 6. Wait for conversions to start (100ms is sufficient — first DRDY at 4ms)
    sleep_ms(100);

    if (!running) return;

    // 7. Verify each port independently: DRDY active + valid status bytes
    std::printf("\n  Verifying each port...\n");

    constexpr int VERIFY_SAMPLES = 10;

    for (int p = 0; p < num_ports; ++p) {
        if (!running) return;

        auto* dev = devices[p];
        auto& health = healthy_out[p];

        // Check DRDY
        health.drdy_active = dev->wait_for_drdy(0.1);
        if (!health.drdy_active) {
            std::printf("    %s: DRDY not active [FAIL]\n", dev->config().port_name);
            continue;
        }

        // Read VERIFY_SAMPLES and check status bytes on all devices in chain
        health.valid_samples = verify_port_data(*dev, VERIFY_SAMPLES);
        health.total_samples = VERIFY_SAMPLES;

        // Require >= 80% valid (8/10) to mark healthy
        health.healthy = (health.valid_samples >= (VERIFY_SAMPLES * 8 / 10));

        if (health.healthy) {
            std::printf("    %s: %d/%d valid [OK]\n",
                        dev->config().port_name, health.valid_samples, VERIFY_SAMPLES);
        } else {
            std::printf("    %s: %d/%d valid [FAIL]\n",
                        dev->config().port_name, health.valid_samples, VERIFY_SAMPLES);
        }
    }
}

// --- ADS1299Controller: Recover a single port ---
// Cycles one port through STOP->SDATAC->flush->RDATAC->START and verifies.
// This targets the RDATAC timing race specifically. Fast (~200ms per attempt).
// Does NOT touch register configuration — that is already correct.

bool ADS1299Controller::recover_port(ADS1299Device& dev, int attempt, int verify_samples) {
    // Escalating delay: attempt 0 = 1x, attempt 1 = 2x, etc.
    int scale = attempt + 1;

    // 1. De-assert START
    dev.start_low();
    sleep_ms(20.0 * scale);

    // 2. STOP command
    dev.send_command(Cmd::STOP);
    sleep_ms(10);

    // 3. Exit RDATAC
    dev.send_command(Cmd::SDATAC);
    sleep_ms(20.0 * scale);

    // 4. Flush SPI shift register (clears stale data)
    dev.flush_spi();
    sleep_ms(5);

    // 5. Re-enter RDATAC (send twice for reliability across daisy chain)
    dev.send_command(Cmd::RDATAC);
    sleep_ms(10);
    dev.send_command(Cmd::RDATAC);
    sleep_ms(20.0 * scale);

    // 6. Assert START
    dev.start_high();
    sleep_ms(50.0 * scale);

    // 7. Verify: read verify_samples and check status bytes on all devices
    int valid = verify_port_data(dev, verify_samples);

    // Require >= 80% valid
    return valid >= (verify_samples * 8 / 10);
}

// --- ADS1299Controller: Legacy restart_single_port ---
// Kept for backward compatibility. Calls recover_port internally.

bool ADS1299Controller::restart_single_port(ADS1299Device& dev, int attempt, bool verbose) {
    bool ok = recover_port(dev, attempt, 5);
    if (!ok && verbose) {
        std::printf("    %s: restart failed (attempt %d)\n",
                    dev.config().port_name, attempt + 1);
    }
    return ok;
}

// --- ADS1299Controller: Utility functions ---

void ADS1299Controller::broadcast_command(std::vector<ADS1299Device*>& devices,
                                           uint8_t cmd, double delay_ms_val) {
    for (auto* dev : devices) {
        dev->send_command(cmd);
    }
    if (delay_ms_val > 0) {
        sleep_ms(delay_ms_val);
    }
}

void ADS1299Controller::force_all_start_pins_low(std::vector<ADS1299Device*>& devices) {
    for (auto* dev : devices) {
        dev->start_low();
    }
    sleep_ms(10);
}

TCA9534MultiPin ADS1299Controller::create_start_controller(
    std::vector<ADS1299Device*>& devices, I2CDevice& i2c) {
    TCA9534MultiPin ctrl(i2c);
    for (auto* dev : devices) {
        ctrl.add_pin(dev->config().start_i2c_addr, dev->config().start_pin);
    }
    return ctrl;
}

} // namespace ads1299
```

---

### `src/main.cpp`

```cpp
#include "ads1299/registers.hpp"
#include "ads1299/types.hpp"
#include "ads1299/spi_device.hpp"
#include "ads1299/controller.hpp"
#include "hardware/spi_bus.hpp"
#include "hardware/i2c_device.hpp"
#include "hardware/tca9534.hpp"
#include "acquisition/engine.hpp"
#include "acquisition/bus_worker.hpp"
#include "acquisition/drdy_poller.hpp"
#include "logging/csv_writer.hpp"
#include "logging/spsc_ring.hpp"
#include "streaming/server.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <vector>
#include <time.h>
#include <unistd.h>

// Async-signal-safe shutdown flag (not std::atomic<bool> — volatile sig_atomic_t is correct here)
static volatile sig_atomic_t g_running = 1;

static void signal_handler(int /*sig*/) {
    g_running = 0;
}

static void sleep_sec(double sec) {
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(sec);
    ts.tv_nsec = static_cast<long>((sec - ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
}

// --- Argument parsing ---

struct Args {
    ads1299::PortConfig ports[ads1299::MAX_PORTS];
    int num_ports = 0;
    bool full_csv = true;   // Default: full 328-channel CSV
    int duration_sec = 0;   // 0 = run until Ctrl+C
    char host[64] = "0.0.0.0";
    int port = 8888;
};

static bool parse_port(const char* str, ads1299::PortConfig& out) {
    // Format: "bus,device,name,num_daisy"
    int bus = 0, dev = 0, num_daisy = 0;
    char name[16] = {};

    // Manual CSV parsing (no sscanf dependency on format)
    const char* p = str;
    bus = static_cast<int>(std::strtol(p, const_cast<char**>(&p), 10));
    if (*p != ',') return false;
    ++p;
    dev = static_cast<int>(std::strtol(p, const_cast<char**>(&p), 10));
    if (*p != ',') return false;
    ++p;

    int ni = 0;
    while (*p && *p != ',' && ni < 15) {
        name[ni++] = *p++;
    }
    name[ni] = '\0';
    if (*p != ',') return false;
    ++p;
    num_daisy = static_cast<int>(std::strtol(p, nullptr, 10));

    return ads1299::make_port_config(bus, dev, name, num_daisy, out);
}

static Args parse_args(int argc, char* argv[]) {
    Args args;

    bool ports_specified = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ports") == 0) {
            ports_specified = true;
            // Consume all following args that look like port configs (contain commas)
            while (i + 1 < argc && std::strchr(argv[i + 1], ',') != nullptr) {
                ++i;
                if (args.num_ports < ads1299::MAX_PORTS) {
                    if (!parse_port(argv[i], args.ports[args.num_ports])) {
                        std::fprintf(stderr, "Error: Invalid port format '%s'\n", argv[i]);
                        std::fprintf(stderr, "Expected: bus,device,name,num_daisy\n");
                        std::exit(1);
                    }
                    args.num_ports++;
                }
            }
        } else if (std::strcmp(argv[i], "--full-csv") == 0) {
            args.full_csv = true;
        } else if (std::strcmp(argv[i], "--no-csv") == 0) {
            args.full_csv = false;
        } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            args.duration_sec = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
            std::strncpy(args.host, argv[++i], sizeof(args.host) - 1);
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            args.port = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::printf("Usage: %s [--ports bus,dev,name,num_daisy ...] "
                        "[--full-csv] [--no-csv] [--duration SEC] "
                        "[--host ADDR] [--port PORT]\n\n"
                        "Default: 7 ports x 41 devices = 328 channels @ 250 Hz\n"
                        "         TCP streaming on 0.0.0.0:8888\n\n"
                        "Port format: bus,device,name,num_daisy\n"
                        "  bus: SPI bus (0, 3, 4, 5)\n"
                        "  device: SPI device (0-1)\n"
                        "  name: Port name (e.g. Port1)\n"
                        "  num_daisy: Number of daisy-chained ADS1299s\n", argv[0]);
            std::exit(0);
        }
    }

    // Default 7-port configuration if no ports specified
    if (!ports_specified) {
        ads1299::DefaultPortConfigs defaults;
        std::memcpy(args.ports, defaults.ports, sizeof(ads1299::PortConfig) * defaults.count);
        args.num_ports = defaults.count;
    }

    return args;
}

// --- Main ---

int main(int argc, char* argv[]) {
    Args args = parse_args(argc, argv);

    int total_devices = 0;
    for (int i = 0; i < args.num_ports; ++i) {
        total_devices += args.ports[i].num_devices;
    }
    int total_channels = total_devices * ads1299::CHANNELS_PER_DEVICE;

    std::printf("\n============================================================\n"
                "ADS1299 C++ ACQUISITION ENGINE (Phase 2: TCP Streaming)\n"
                "============================================================\n"
                "Config: %d ports, %d devices, %d channels @ 250Hz\n"
                "Stream: %s:%d\n\n",
                args.num_ports, total_devices, total_channels,
                args.host, args.port);

    // --- Signal handling ---
    // SIGPIPE: disconnected TCP client must not kill the process
    signal(SIGPIPE, SIG_IGN);

    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // --- Open I2C bus ---
    std::printf("Opening I2C bus %d...\n", ads1299::I2C_BUS);
    ads1299::I2CDevice i2c(ads1299::I2C_BUS);
    if (!i2c.is_open()) {
        std::fprintf(stderr, "Failed to open I2C bus\n");
        return 1;
    }

    // --- Create TCA9534 pins ---
    // We need DRDY (input) and START (output) pins for each port.
    // Stored in vectors of unique_ptr for lifetime management.
    struct PinSet {
        std::unique_ptr<ads1299::TCA9534Pin> drdy;
        std::unique_ptr<ads1299::TCA9534Pin> start;
    };
    std::vector<PinSet> pin_sets;
    pin_sets.reserve(args.num_ports);

    for (int i = 0; i < args.num_ports; ++i) {
        auto& cfg = args.ports[i];
        auto drdy = std::make_unique<ads1299::TCA9534Pin>(
            i2c, cfg.drdy_i2c_addr, cfg.drdy_pin, true);   // input
        auto start = std::make_unique<ads1299::TCA9534Pin>(
            i2c, cfg.start_i2c_addr, cfg.start_pin, false); // output (init LOW)
        pin_sets.push_back({std::move(drdy), std::move(start)});
    }
    std::printf("TCA9534 pins configured (%d DRDY inputs, %d START outputs)\n",
                args.num_ports, args.num_ports);

    // --- Force all START pins LOW ---
    std::printf("Forcing all START pins LOW...\n");
    for (auto& ps : pin_sets) {
        ps.start->set_low();
    }
    sleep_sec(0.01);

    // --- Power-on wait ---
    std::printf("\nWaiting for VCAP1 charging and device ready (2s)...\n");
    sleep_sec(2.0);
    std::printf("Device power-on wait complete [OK]\n");

    // --- Open SPI devices ---
    std::printf("\nInitializing SPI devices...\n");
    std::vector<std::unique_ptr<ads1299::SPIBus>> spi_buses;
    std::vector<std::unique_ptr<ads1299::ADS1299Device>> device_owners;
    std::vector<ads1299::ADS1299Device*> devices;

    for (int i = 0; i < args.num_ports; ++i) {
        auto& cfg = args.ports[i];
        auto spi = std::make_unique<ads1299::SPIBus>(cfg.bus_num, cfg.device_num);
        if (!spi->is_open()) {
            std::fprintf(stderr, "Failed to open SPI%d.%d\n", cfg.bus_num, cfg.device_num);
            return 1;
        }
        auto dev = std::make_unique<ads1299::ADS1299Device>(
            *spi, cfg, *pin_sets[i].drdy, *pin_sets[i].start);
        devices.push_back(dev.get());
        device_owners.push_back(std::move(dev));
        spi_buses.push_back(std::move(spi));
    }

    // --- Phase 1: Configure all devices (register writes) ---
    // This always succeeds if hardware is present. Do it once.
    std::printf("\n============================================================\n"
                "PHASE 1: CONFIGURING DEVICES (registers)\n"
                "============================================================\n");

    ads1299::DeviceConfig config;

    for (int i = 0; i < args.num_ports; ++i) {
        if (!g_running) {
            std::printf("\n[CANCELLED] Ctrl+C during configuration\n");
            return 1;
        }
        bool success = ads1299::ADS1299Controller::initialize_device(*devices[i], config);
        if (!success) {
            std::fprintf(stderr, "\n[FAIL] %s failed to configure registers\n",
                         devices[i]->config().port_name);
            std::fprintf(stderr, "This is a hardware failure, not the RDATAC timing race.\n");
            return 1;
        }
        sleep_sec(0.1);
    }
    std::printf("\n[OK] All %d ports configured successfully (registers verified)\n",
                args.num_ports);

    if (!g_running) {
        std::printf("\n[CANCELLED] Ctrl+C after configuration\n");
        return 1;
    }

    // --- Phase 2: Start conversions + per-port recovery ---
    // RDATAC + START has a ~10% per-port failure rate (timing race).
    // Strategy: start all, verify each, recover only failing ports.
    std::printf("\n============================================================\n"
                "PHASE 2: STARTING CONVERSIONS + VERIFICATION\n"
                "============================================================\n");

    ads1299::PortHealth port_health[ads1299::MAX_PORTS] = {};

    // Start all ports and get initial health status
    ads1299::ADS1299Controller::start_and_verify(devices, i2c, port_health, g_running);

    if (!g_running) {
        std::printf("\n[CANCELLED] Ctrl+C during start\n");
        return 1;
    }

    // Count initial results
    int healthy_count = 0;
    int failing_count = 0;
    for (int p = 0; p < args.num_ports; ++p) {
        if (port_health[p].healthy) {
            healthy_count++;
        } else {
            failing_count++;
        }
    }
    std::printf("\n  Initial result: %d/%d ports healthy", healthy_count, args.num_ports);
    if (failing_count > 0) {
        std::printf(" (%d need recovery)", failing_count);
    }
    std::printf("\n");

    // --- Phase 3: Per-port recovery for failing ports ---
    // Only failing ports are cycled. Working ports are never touched.
    constexpr int MAX_RECOVERY_ATTEMPTS = 15;
    bool port_dead[ads1299::MAX_PORTS] = {};  // true = gave up on this port

    if (failing_count > 0) {
        std::printf("\n  --- Per-port recovery (max %d attempts each) ---\n",
                    MAX_RECOVERY_ATTEMPTS);
    }

    for (int p = 0; p < args.num_ports; ++p) {
        if (port_health[p].healthy) continue;  // Skip healthy ports
        if (!g_running) break;

        auto* dev = devices[p];
        bool recovered = false;

        for (int attempt = 0; attempt < MAX_RECOVERY_ATTEMPTS; ++attempt) {
            if (!g_running) break;

            bool ok = ads1299::ADS1299Controller::recover_port(*dev, attempt, 10);
            if (ok) {
                std::printf("    %s: recovered on attempt %d [OK]\n",
                            dev->config().port_name, attempt + 1);
                port_health[p].healthy = true;
                port_health[p].drdy_active = true;
                healthy_count++;
                failing_count--;
                recovered = true;
                break;
            }

            // Brief log every few attempts
            if (attempt == 2 || attempt == 5 || attempt == 9 || attempt == 14) {
                std::printf("    %s: still failing after %d attempts...\n",
                            dev->config().port_name, attempt + 1);
            }
        }

        if (!recovered && g_running) {
            port_dead[p] = true;
            failing_count--;
            std::printf("    %s: DEAD after %d attempts - skipping\n",
                        dev->config().port_name, MAX_RECOVERY_ATTEMPTS);
            // Shut down the dead port cleanly
            dev->start_low();
            dev->send_command(ads1299::Cmd::STOP);
            dev->send_command(ads1299::Cmd::SDATAC);
        }
    }

    if (!g_running) {
        std::printf("\n[CANCELLED] Ctrl+C during recovery\n");
        return 1;
    }

    // --- Phase 4: Warmup + final health check ---
    std::printf("\n  --- Warmup: discarding 100 settling samples ---\n");

    // Find a healthy reference port for DRDY timing
    ads1299::ADS1299Device* ref_port = nullptr;
    for (int p = 0; p < args.num_ports; ++p) {
        if (port_health[p].healthy) {
            if (!ref_port || devices[p]->config().num_devices > ref_port->config().num_devices) {
                ref_port = devices[p];
            }
        }
    }

    if (!ref_port) {
        std::fprintf(stderr, "\n[FAIL] No healthy ports available\n");
        return 1;
    }

    // Discard 100 warmup samples (read from all healthy ports)
    for (int i = 0; i < 100 && g_running; ++i) {
        if (ref_port->wait_for_drdy(0.02)) {
            for (int p = 0; p < args.num_ports; ++p) {
                if (!port_dead[p]) {
                    ads1299::PortData pd;
                    devices[p]->read_data(pd);
                }
            }
        }
    }

    if (!g_running) {
        std::printf("\n[CANCELLED] Ctrl+C during warmup\n");
        return 1;
    }

    // Final health check: 50 samples, verify all healthy ports are still healthy
    std::printf("  --- Final health check (50 samples) ---\n");

    int final_valid[ads1299::MAX_PORTS] = {};
    int final_reads = 0;

    for (int i = 0; i < 50 && g_running; ++i) {
        if (ref_port->wait_for_drdy(0.02)) {
            final_reads++;
            for (int p = 0; p < args.num_ports; ++p) {
                if (port_dead[p]) continue;
                ads1299::PortData pd;
                devices[p]->read_data(pd);

                bool ok = true;
                for (int d = 0; d < devices[p]->config().num_devices; ++d) {
                    if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
                        ok = false;
                        break;
                    }
                }
                if (ok) final_valid[p]++;
            }
        }
    }

    // Report and handle any ports that degraded during warmup
    int active_ports = 0;
    for (int p = 0; p < args.num_ports; ++p) {
        if (port_dead[p]) {
            std::printf("    %s: DEAD (skipped)\n", devices[p]->config().port_name);
            continue;
        }

        double rate = final_reads > 0 ? static_cast<double>(final_valid[p]) / final_reads : 0;
        if (rate >= 0.90) {
            std::printf("    %s: %.0f%% valid (%d/%d) [OK]\n",
                        devices[p]->config().port_name, rate * 100.0,
                        final_valid[p], final_reads);
            active_ports++;
        } else if (rate == 0 && final_reads > 10) {
            // Port regressed to zeros during warmup — one more recovery attempt
            std::printf("    %s: 0%% valid — attempting final recovery...\n",
                        devices[p]->config().port_name);
            bool last_chance = false;
            for (int a = 0; a < 5 && g_running; ++a) {
                if (ads1299::ADS1299Controller::recover_port(*devices[p], a, 10)) {
                    std::printf("    %s: recovered [OK]\n", devices[p]->config().port_name);
                    last_chance = true;
                    break;
                }
            }
            if (last_chance) {
                active_ports++;
            } else {
                port_dead[p] = true;
                std::printf("    %s: DEAD after final recovery attempt\n",
                            devices[p]->config().port_name);
                devices[p]->start_low();
                devices[p]->send_command(ads1299::Cmd::STOP);
                devices[p]->send_command(ads1299::Cmd::SDATAC);
            }
        } else {
            // Partially valid — keep it, it may stabilize during acquisition
            std::printf("    %s: %.0f%% valid (%d/%d) [WARN - keeping]\n",
                        devices[p]->config().port_name, rate * 100.0,
                        final_valid[p], final_reads);
            active_ports++;
        }
    }

    if (active_ports == 0) {
        std::fprintf(stderr, "\n[FAIL] No active ports after initialization\n");
        return 1;
    }

    // Count dead ports for summary
    int dead_count = 0;
    for (int p = 0; p < args.num_ports; ++p) {
        if (port_dead[p]) dead_count++;
    }

    if (dead_count > 0) {
        std::printf("\n[WARN] %d port(s) dead, running with %d/%d ports\n",
                    dead_count, active_ports, args.num_ports);
        std::printf("  Dead:");
        for (int p = 0; p < args.num_ports; ++p) {
            if (port_dead[p]) std::printf(" %s", devices[p]->config().port_name);
        }
        std::printf("\n");
    }

    std::printf("\n============================================================\n"
                "[OK] SYSTEM INITIALIZATION COMPLETE (%d/%d ports active)\n"
                "============================================================\n",
                active_ports, args.num_ports);

    // --- Create DRDY poller ---
    ads1299::DRDYPoller drdy_poller(i2c, ads1299::TCA9534_DRDY_ADDR);
    for (int i = 0; i < args.num_ports; ++i) {
        // Only add ports that have active DRDY
        if (devices[i]->wait_for_drdy(0.1)) {
            drdy_poller.add_pin(args.ports[i].drdy_pin);
        } else {
            std::printf("  [WARN] %s: DRDY not active - excluding from acquisition\n",
                        devices[i]->config().port_name);
        }
    }

    if (!drdy_poller.has_pins()) {
        std::fprintf(stderr, "[FAIL] No ports have active DRDY\n");
        return 1;
    }

    // --- Create SPSC ring buffer ---
    auto ring = std::make_unique<ads1299::SPSCRing<ads1299::Sample>>();

    // --- Create bus workers ---
    // Group ports by physical SPI bus, assign CPU cores:
    // SPI0→core 0, SPI3→core 1, SPI4→core 2, SPI5→core 2
    // SPI5 moved off core 0 to avoid contending with SPI0 (Port1's 9-deep chain
    // has tightest timing margin; sharing core 0 with SPI5 added ~252us delay)
    auto bus_groups = ads1299::group_ports_by_bus(args.ports, args.num_ports);

    auto get_core = [](int bus_num) -> int {
        switch (bus_num) {
            case 0: return 0;  // SPI0 (Port1+2): sole owner, fastest path to Port1
            case 3: return 1;
            case 4: return 2;
            case 5: return 2;  // SPI5 (Port7): shares core 2 with SPI4
            default: return 0;
        }
    };

    std::vector<std::unique_ptr<ads1299::BusWorker>> worker_owners;
    std::vector<ads1299::BusWorker*> workers;

    for (auto& group : bus_groups) {
        std::vector<ads1299::ADS1299Device*> group_devices;
        for (int idx : group.port_indices) {
            group_devices.push_back(devices[idx]);
        }
        int core = get_core(group.bus_num);
        auto worker = std::make_unique<ads1299::BusWorker>(
            group.bus_num, group_devices, core);
        workers.push_back(worker.get());
        worker_owners.push_back(std::move(worker));
    }

    std::printf("  Parallel SPI workers: %zu buses [", workers.size());
    for (size_t i = 0; i < workers.size(); ++i) {
        if (i > 0) std::printf(", ");
        std::printf("SPI%d(%d)", workers[i]->bus_num(), workers[i]->port_count());
    }
    std::printf("]\n");

    // --- Create CSV writer ---
    ads1299::CSVWriter* csv_writer = nullptr;
    std::unique_ptr<ads1299::CSVWriter> csv_owner;
    if (args.full_csv) {
        // Build channel names: portN_devM_ch1 ... portN_devM_ch8
        std::vector<std::string> names;
        for (int p = 0; p < args.num_ports; ++p) {
            for (int d = 0; d < args.ports[p].num_devices; ++d) {
                for (int c = 0; c < ads1299::CHANNELS_PER_DEVICE; ++c) {
                    char name[64];
                    std::snprintf(name, sizeof(name), "%s_dev%d_ch%d",
                                  args.ports[p].port_name, d + 1, c + 1);
                    names.push_back(name);
                }
            }
        }

        // Convert to const char* array
        std::vector<const char*> name_ptrs;
        name_ptrs.reserve(names.size());
        for (auto& n : names) {
            name_ptrs.push_back(n.c_str());
        }

        csv_owner = std::make_unique<ads1299::CSVWriter>(
            "all_channels_data.csv", *ring, total_channels,
            name_ptrs.data(), static_cast<int>(name_ptrs.size()));
        csv_writer = csv_owner.get();
        csv_writer->start();
        std::printf("  CSV writer: all %d channels -> all_channels_data.csv (async)\n",
                    total_channels);
    }

    // --- Create streaming server ---
    ads1299::StreamingServer streaming_server(args.ports, args.num_ports,
                                             total_channels, total_devices);
    streaming_server.start(args.host, args.port);

    // --- Create and run acquisition engine ---
    std::printf("\n============================================================\n"
                "ACQUISITION RUNNING - Ctrl+C to stop\n"
                "============================================================\n\n");

    ads1299::AcquisitionEngine engine(devices, workers, drdy_poller, *ring);
    engine.set_streaming_server(&streaming_server);

    // If duration specified, set up a timer thread
    if (args.duration_sec > 0) {
        std::printf("  Auto-stop after %d seconds\n", args.duration_sec);
        std::thread timer_thread([&]() {
            sleep_sec(args.duration_sec);
            g_running = 0;
        });
        timer_thread.detach();
    }

    engine.run(g_running);

    // --- Shutdown ---
    std::printf("\n\nShutdown...\n");

    // Stop bus workers
    for (auto& w : worker_owners) {
        w->stop();
    }

    // Stop streaming server (drains ring, joins threads)
    streaming_server.stop();

    // Stop CSV writer (drains remaining, flushes, closes)
    if (csv_writer) {
        csv_writer->stop();
    }

    // Stop ADS1299 devices
    for (auto* dev : devices) {
        dev->start_low();
        dev->send_command(ads1299::Cmd::STOP);
    }

    // Final statistics
    auto stats = engine.get_stats();
    double rate = stats.runtime_sec > 0 ? stats.total_samples / stats.runtime_sec : 0;

    auto ss = streaming_server.get_stats();

    std::printf("\n============================================================\n"
                "FINAL STATISTICS\n"
                "============================================================\n"
                "Samples:    %lu\n"
                "Runtime:    %.1f s\n"
                "Rate:       %.1f Hz\n"
                "Cycle time: %.2f / %.2f / %.2f ms (min/mean/max)\n"
                "Sample dt:  %.2f / %.2f ms (min/max)\n"
                "DRDY timeouts:  %lu\n"
                "Corruptions:    %lu\n"
                "Ring drops:     %lu\n"
                "CSV written:    %lu\n"
                "Stream sent:    %lu (batches: %lu, drops: %lu, reconnects: %lu)\n"
                "============================================================\n\n",
                static_cast<unsigned long>(stats.total_samples),
                stats.runtime_sec,
                rate,
                stats.min_cycle_ms, stats.mean_cycle_ms, stats.max_cycle_ms,
                stats.min_dt_ms, stats.max_dt_ms,
                static_cast<unsigned long>(stats.drdy_timeouts),
                static_cast<unsigned long>(stats.corruption_count),
                static_cast<unsigned long>(stats.drop_count),
                csv_writer ? static_cast<unsigned long>(csv_writer->total_written()) : 0UL,
                static_cast<unsigned long>(ss.samples_sent),
                static_cast<unsigned long>(ss.batches_sent),
                static_cast<unsigned long>(ss.drops),
                static_cast<unsigned long>(ss.reconnects));

    std::printf("System shutdown complete\n");
    return 0;
}
```
