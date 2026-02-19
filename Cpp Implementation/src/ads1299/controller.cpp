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
