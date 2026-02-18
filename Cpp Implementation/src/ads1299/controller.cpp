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

// --- ADS1299Controller ---

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

bool ADS1299Controller::restart_single_port(ADS1299Device& dev, int attempt, bool verbose) {
    int delay_scale = attempt + 1;

    // Stop the port
    dev.start_low();
    sleep_ms(20.0 * delay_scale);

    dev.send_command(Cmd::STOP);
    sleep_ms(10);
    dev.send_command(Cmd::SDATAC);
    sleep_ms(20.0 * delay_scale);

    // Re-enter RDATAC (send twice for reliability)
    dev.send_command(Cmd::RDATAC);
    sleep_ms(10);
    dev.send_command(Cmd::RDATAC);
    sleep_ms(20.0 * delay_scale);

    // Assert START
    dev.start_high();
    sleep_ms(50.0 * delay_scale);

    // Verify DRDY and data
    if (dev.wait_for_drdy(0.1)) {
        PortData pd;
        dev.read_data(pd);
        uint8_t status0 = pd.status_bytes[0][0];
        if ((status0 & 0xF0) == 0xC0) {
            return true;
        } else if (verbose) {
            if (status0 == 0x00) {
                std::printf("    %s: DRDY active but returning zeros (attempt %d)\n",
                            dev.config().port_name, attempt + 1);
            } else {
                std::printf("    %s: DRDY active, status=0x%02X (attempt %d)\n",
                            dev.config().port_name, status0, attempt + 1);
            }
        }
    } else if (verbose) {
        std::printf("    %s: DRDY still not active (attempt %d)\n",
                    dev.config().port_name, attempt + 1);
    }

    return false;
}

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
    std::printf("Forcing all START pins LOW...\n");
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

bool ADS1299Controller::start_all_conversions_synchronized(
    std::vector<ADS1299Device*>& devices,
    I2CDevice& i2c,
    const DeviceConfig* config) {

    std::printf("\n  Starting conversions with synchronized START...\n");

    std::printf("    Stopping any ongoing conversions...\n");
    force_all_start_pins_low(devices);
    broadcast_command(devices, static_cast<uint8_t>(Cmd::STOP), 10);

    std::printf("    Ensuring all devices in SDATAC mode...\n");
    broadcast_command(devices, static_cast<uint8_t>(Cmd::SDATAC), 50);

    std::printf("    Sending RDATAC to all devices (fast sequential)...\n");
    // Send RDATAC as quickly as possible to minimize desync
    for (auto* dev : devices) {
        dev->send_command(Cmd::RDATAC);
    }
    sleep_ms(50);
    // Second RDATAC pass to ensure all devices received it
    for (auto* dev : devices) {
        dev->send_command(Cmd::RDATAC);
    }
    sleep_ms(200);  // 200ms settling after RDATAC

    // Verify device communication after RDATAC
    std::printf("    Verifying device communication after RDATAC...\n");
    std::vector<int> rdatac_fail_indices;
    for (size_t i = 0; i < devices.size(); ++i) {
        auto* dev = devices[i];
        dev->flush_spi();
        sleep_ms(2);
        // Read 3 bytes to check for 0xFF (RDATAC failed)
        uint8_t check_tx[3] = {0, 0, 0};
        uint8_t check_rx[3] = {0, 0, 0};
        dev->spi().transfer(check_tx, check_rx, 3);
        if (check_rx[0] == 0xFF && check_rx[1] == 0xFF && check_rx[2] == 0xFF) {
            rdatac_fail_indices.push_back(static_cast<int>(i));
            std::printf("      [WARN] %s: SPI returns 0xFF - RDATAC may have failed\n",
                        dev->config().port_name);
        }
    }

    if (!rdatac_fail_indices.empty()) {
        std::printf("    Retrying RDATAC for failed ports...\n");
        for (int idx : rdatac_fail_indices) {
            auto* dev = devices[idx];
            dev->send_command(Cmd::SDATAC);
            sleep_ms(10);
            dev->send_command(Cmd::RDATAC);
            sleep_ms(20);
            dev->send_command(Cmd::RDATAC);
            sleep_ms(20);
        }
    }
    sleep_ms(50);

    // Assert START on all ports atomically via TCA9534MultiPin
    {
        TCA9534MultiPin start_controller(i2c);
        for (auto* dev : devices) {
            start_controller.add_pin(dev->config().start_i2c_addr, dev->config().start_pin);
        }
        start_controller.set_all_high();
    }
    std::printf("    [All START pins asserted simultaneously]\n");
    sleep_ms(100);  // 100ms synchronization after START

    for (auto* dev : devices) {
        std::printf("    %s: START asserted [OK]\n", dev->config().port_name);
    }

    // Verify DRDY is actually toggling on each port
    std::printf("\n  Verifying DRDY signals are active...\n");
    std::vector<int> drdy_failure_indices;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (devices[i]->wait_for_drdy(0.1)) {
            std::printf("    %s: DRDY active [OK]\n", devices[i]->config().port_name);
        } else {
            std::printf("    %s: DRDY NOT active [FAIL]\n", devices[i]->config().port_name);
            drdy_failure_indices.push_back(static_cast<int>(i));
        }
    }

    if (!drdy_failure_indices.empty()) {
        std::printf("  [WARN] DRDY failed on %zu port(s) - attempting restart...\n",
                    drdy_failure_indices.size());
        for (int idx : drdy_failure_indices) {
            bool restart_success = false;
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (restart_single_port(*devices[idx], attempt)) {
                    std::printf("    %s: DRDY now active [OK] (attempt %d)\n",
                                devices[idx]->config().port_name, attempt + 1);
                    restart_success = true;
                    break;
                }
            }
            if (!restart_success) {
                std::printf("    %s: Restart failed after 3 attempts [FAIL]\n",
                            devices[idx]->config().port_name);
            }
        }
    }

    std::printf("\n  Waiting 1000ms for stabilization...\n");
    sleep_ms(1000);

    // Check individual port DRDY signals and data
    std::printf("\n  Checking individual port DRDY signals...\n");
    std::vector<ADS1299Device*> ports_needing_restart;
    for (auto* dev : devices) {
        bool drdy_active = dev->wait_for_drdy(0.05);
        if (drdy_active) {
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

            if (valid_count >= 3) {
                std::printf("    %s: DRDY [OK], valid status [OK] (%d/5 valid)\n",
                            dev->config().port_name, valid_count);
            } else if (zero_count >= 3) {
                std::printf("    %s: DRDY [OK], but returning zeros (%d/5) [FAIL]\n",
                            dev->config().port_name, zero_count);
                ports_needing_restart.push_back(dev);
            } else {
                std::printf("    %s: DRDY [OK], mixed status (valid=%d, zero=%d)\n",
                            dev->config().port_name, valid_count, zero_count);
                if (zero_count > valid_count) {
                    ports_needing_restart.push_back(dev);
                }
            }
        } else {
            std::printf("    %s: DRDY not active [FAIL]\n", dev->config().port_name);
            ports_needing_restart.push_back(dev);
        }
    }

    if (!ports_needing_restart.empty()) {
        std::printf("\n  Restarting %zu ports...\n", ports_needing_restart.size());
        for (auto* dev : ports_needing_restart) {
            bool restart_success = false;
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (restart_single_port(*dev, attempt, false)) {
                    std::printf("    %s: restart ok (attempt %d) [OK]\n",
                                dev->config().port_name, attempt + 1);
                    restart_success = true;
                    break;
                }
            }
            if (!restart_success) {
                std::printf("    %s: restart failed\n", dev->config().port_name);
            }
        }
        sleep_ms(200);
    }

    // 500-sample warmup
    std::printf("\n  Discarding 500 warmup samples...\n");
    // Use port with longest daisy chain as reference
    ADS1299Device* reference_port = devices[0];
    for (auto* dev : devices) {
        if (dev->config().num_devices > reference_port->config().num_devices) {
            reference_port = dev;
        }
    }

    int warmup_count = 0;
    int warmup_drdy_timeouts = 0;

    // Per-port corruption counters
    int warmup_corruptions[MAX_PORTS] = {};
    int consecutive_zeros[MAX_PORTS] = {};
    bool port_restart_attempted[MAX_PORTS] = {};

    for (int i = 0; i < 500; ++i) {
        if (reference_port->wait_for_drdy(0.02)) {
            for (size_t p = 0; p < devices.size(); ++p) {
                auto* dev = devices[p];
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

                // Auto-restart port after 20 consecutive zeros
                if (consecutive_zeros[p] >= 20 && !port_restart_attempted[p]) {
                    port_restart_attempted[p] = true;
                    std::printf("    [WARN] %s: 20+ zeros at sample %d, restarting...\n",
                                dev->config().port_name, i);
                    bool restart_success = false;
                    for (int ra = 0; ra < 3; ++ra) {
                        if (restart_single_port(*dev, ra, false)) {
                            std::printf("      %s: restart ok (attempt %d) [OK]\n",
                                        dev->config().port_name, ra + 1);
                            consecutive_zeros[p] = 0;
                            restart_success = true;
                            break;
                        }
                    }
                    if (!restart_success) {
                        std::printf("      %s: restart failed\n", dev->config().port_name);
                    }
                }
            }
            warmup_count++;
        } else {
            warmup_drdy_timeouts++;
            if (warmup_drdy_timeouts <= 5) {
                std::printf("    Warmup sample %d: DRDY timeout on reference port\n", i);
            }
        }
    }

    if (warmup_drdy_timeouts > 0) {
        std::printf("  [WARN] %d DRDY timeouts during warmup (%.1f%%)\n",
                    warmup_drdy_timeouts, warmup_drdy_timeouts / 500.0 * 100.0);
    }

    int total_warmup_corruptions = 0;
    for (size_t p = 0; p < devices.size(); ++p) {
        total_warmup_corruptions += warmup_corruptions[p];
    }

    if (total_warmup_corruptions > 0) {
        std::printf("  [WARN] Warmup STATUS corruption: %d total\n", total_warmup_corruptions);
        for (size_t p = 0; p < devices.size(); ++p) {
            if (warmup_corruptions[p] > 0) {
                double pct = warmup_count > 0 ? warmup_corruptions[p] * 100.0 / warmup_count : 0;
                std::printf("    %s: %d (%.1f%%)\n",
                            devices[p]->config().port_name, warmup_corruptions[p], pct);
            }
        }
    }
    std::printf("  Discarded %d warmup samples\n", warmup_count);

    // Check for ports with elevated warmup corruption (>10%) and try aggressive re-init
    constexpr double WARMUP_REINIT_THRESHOLD = 0.10;
    struct ReinitEntry {
        ADS1299Device* dev;
        size_t port_idx;
        double corruption_rate;
    };
    std::vector<ReinitEntry> ports_needing_reinit;

    for (size_t p = 0; p < devices.size(); ++p) {
        if (warmup_count > 0) {
            double rate = static_cast<double>(warmup_corruptions[p]) / warmup_count;
            if (rate > WARMUP_REINIT_THRESHOLD) {
                ports_needing_reinit.push_back({devices[p], p, rate});
            }
        }
    }

    if (!ports_needing_reinit.empty()) {
        std::printf("\n  [WARN] %zu port(s) with >10%% warmup corruption - "
                    "attempting aggressive re-init...\n", ports_needing_reinit.size());

        for (auto& entry : ports_needing_reinit) {
            auto* dev = entry.dev;
            std::printf("    %s: %.1f%% corruption - full re-init...\n",
                        dev->config().port_name, entry.corruption_rate * 100.0);

            // 1. Stop the port completely
            dev->start_low();
            sleep_ms(50);
            dev->send_command(Cmd::STOP);
            sleep_ms(20);

            // 2. Full re-initialization
            bool reinit_ok = false;
            if (config != nullptr) {
                reinit_ok = initialize_device(*dev, *config);
            } else {
                // Fallback: just cycle commands
                dev->flush_spi();
                dev->send_command(Cmd::SDATAC);
                sleep_ms(100);
            }

            if (reinit_ok) {
                // 3. Enter RDATAC and start conversion
                dev->send_command(Cmd::RDATAC);
                sleep_ms(10);
                dev->send_command(Cmd::RDATAC);
                sleep_ms(50);
                dev->start_high();
                sleep_ms(200);

                // 4. Mini-warmup: verify data is clean
                if (dev->wait_for_drdy(0.1)) {
                    int mini_corruptions = 0;
                    for (int s = 0; s < 50; ++s) {
                        if (dev->wait_for_drdy(0.02)) {
                            PortData pd;
                            dev->read_data(pd);
                            if ((pd.status_bytes[0][0] & 0xF0) != 0xC0) {
                                mini_corruptions++;
                            }
                        }
                    }
                    double mini_rate = mini_corruptions / 50.0;
                    if (mini_rate < 0.05) {
                        std::printf("      %s: re-init successful (%.1f%% corruption) [OK]\n",
                                    dev->config().port_name, mini_rate * 100.0);
                        warmup_corruptions[entry.port_idx] = static_cast<int>(mini_rate * warmup_count);
                    } else {
                        std::printf("      %s: re-init helped but still %.1f%% corruption\n",
                                    dev->config().port_name, mini_rate * 100.0);
                        warmup_corruptions[entry.port_idx] = static_cast<int>(mini_rate * warmup_count);
                    }
                } else {
                    std::printf("      %s: DRDY not active after re-init [FAIL]\n",
                                dev->config().port_name);
                }
            } else {
                std::printf("      %s: re-initialization failed [FAIL]\n",
                            dev->config().port_name);
            }
        }
    }

    // 20-sample data flow verification
    std::printf("\n  Verifying data flow (20 reads)...\n");
    reference_port = devices[0];
    for (auto* dev : devices) {
        if (dev->config().num_devices > reference_port->config().num_devices) {
            reference_port = dev;
        }
    }

    int valid_counts[MAX_PORTS] = {};
    int total_reads = 0;
    int drdy_timeouts = 0;

    for (int read_num = 0; read_num < 20; ++read_num) {
        if (reference_port->wait_for_drdy(0.02)) {
            total_reads++;
            for (size_t p = 0; p < devices.size(); ++p) {
                PortData pd;
                devices[p]->read_data(pd);
                if ((pd.status_bytes[0][0] & 0xF0) == 0xC0) {
                    valid_counts[p]++;
                }
            }
        } else {
            drdy_timeouts++;
            if (drdy_timeouts <= 3) {
                std::printf("    Read %d: DRDY timeout on reference port\n", read_num);
            }
        }
    }

    if (drdy_timeouts > 0) {
        std::printf("  [WARN] %d DRDY timeouts during verification\n", drdy_timeouts);
    }

    bool all_ok = true;
    for (size_t p = 0; p < devices.size(); ++p) {
        int count = valid_counts[p];
        if (count >= 10) {
            std::printf("    [OK] %s: Data flowing (%d/%d valid)\n",
                        devices[p]->config().port_name, count, total_reads);
        } else {
            std::printf("    [WARN] %s: Low valid (%d/%d)\n",
                        devices[p]->config().port_name, count, total_reads);
            if (count == 0) {
                all_ok = false;
            }
        }
    }

    if (all_ok) {
        std::printf("\n  [OK] All %zu devices verified!\n", devices.size());
    } else {
        std::printf("\n  [WARN] Some ports had issues but may self-correct during streaming\n");
    }

    // Final re-synchronization
    std::printf("\n  Final re-synchronization...\n");

    // Stop all conversions atomically
    {
        TCA9534MultiPin start_controller(i2c);
        for (auto* dev : devices) {
            start_controller.add_pin(dev->config().start_i2c_addr, dev->config().start_pin);
        }
        start_controller.set_all_low();
    }
    sleep_ms(100);

    broadcast_command(devices, static_cast<uint8_t>(Cmd::STOP), 10);
    sleep_ms(200);

    // Flush all shift registers
    for (auto* dev : devices) {
        dev->flush_spi();
    }

    // Re-enter RDATAC
    broadcast_command(devices, static_cast<uint8_t>(Cmd::SDATAC), 20);
    broadcast_command(devices, static_cast<uint8_t>(Cmd::RDATAC), 10);
    broadcast_command(devices, static_cast<uint8_t>(Cmd::RDATAC), 20);

    // Restart all simultaneously
    {
        TCA9534MultiPin start_controller(i2c);
        for (auto* dev : devices) {
            start_controller.add_pin(dev->config().start_i2c_addr, dev->config().start_pin);
        }
        start_controller.set_all_high();
    }
    sleep_ms(500);

    // Discard first 100 post-resync samples and measure DRDY timing
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

    return all_ok;
}

} // namespace ads1299
