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
#include <pthread.h>
#include <sched.h>
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
    bool full_csv = false;  // Default: no CSV (client-side recording)
    int duration_sec = 0;   // 0 = run until Ctrl+C
    char host[64] = "0.0.0.0";
    int port = 8888;
    int min_ports = -1;     // -1 = require all ports (7/7 gate)
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
        } else if (std::strcmp(argv[i], "--min-ports") == 0 && i + 1 < argc) {
            args.min_ports = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
            std::strncpy(args.host, argv[++i], sizeof(args.host) - 1);
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            args.port = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::printf("Usage: %s [--ports bus,dev,name,num_daisy ...] "
                        "[--full-csv] [--no-csv] [--duration SEC] "
                        "[--host ADDR] [--port PORT] [--min-ports N]\n\n"
                        "Default: 7 ports x 41 devices = 328 channels @ 250 Hz\n"
                        "         TCP streaming on 0.0.0.0:8888\n"
                        "         CSV disabled (use --full-csv to enable server-side CSV)\n"
                        "         All ports required (--min-ports = num_ports)\n\n"
                        "Port format: bus,device,name,num_daisy\n"
                        "  bus: SPI bus (0, 3, 4, 5)\n"
                        "  device: SPI device (0-1)\n"
                        "  name: Port name (e.g. Port1)\n"
                        "  num_daisy: Number of daisy-chained ADS1299s\n\n"
                        "  --min-ports N: minimum required ports (default: all)\n"
                        "                 Use to allow degraded operation for testing\n", argv[0]);
            std::exit(0);
        }
    }

    // Default 7-port configuration if no ports specified
    if (!ports_specified) {
        ads1299::DefaultPortConfigs defaults;
        std::memcpy(args.ports, defaults.ports, sizeof(ads1299::PortConfig) * defaults.count);
        args.num_ports = defaults.count;
    }

    // Default min_ports: require all configured ports (7/7 gate)
    if (args.min_ports < 0) {
        args.min_ports = args.num_ports;
    }

    return args;
}

// --- Tiered recovery ---
// Returns number of healthy ports after all tiers. Sets port_dead[] for failures.

static int run_tiered_recovery(
    std::vector<ads1299::ADS1299Device*>& devices,
    ads1299::I2CDevice& i2c,
    ads1299::PortHealth* port_health,
    bool* port_dead,
    int num_ports,
    const ads1299::DeviceConfig& config,
    const volatile sig_atomic_t& running)
{
    auto count_healthy = [&]() {
        int n = 0;
        for (int p = 0; p < num_ports; ++p)
            if (port_health[p].healthy) n++;
        return n;
    };

    int healthy = count_healthy();

    // ---------------------------------------------------------------
    // TIER 1: RDATAC cycling — ~200ms/attempt, 8 attempts/port
    // ---------------------------------------------------------------
    constexpr int TIER1_ATTEMPTS = 8;

    std::printf("\n  --- Tier 1: RDATAC cycling (%d attempts/port) ---\n",
                TIER1_ATTEMPTS);

    for (int p = 0; p < num_ports; ++p) {
        if (port_health[p].healthy) continue;
        if (!running) return count_healthy();

        auto* dev = devices[p];

        for (int a = 0; a < TIER1_ATTEMPTS && running; ++a) {
            if (ads1299::ADS1299Controller::recover_port_tier1(*dev, 10)) {
                std::printf("    %s: Tier 1 recovered on attempt %d [OK]\n",
                            dev->config().port_name, a + 1);
                port_health[p].healthy = true;
                port_health[p].drdy_active = true;
                break;
            }
        }

        if (!port_health[p].healthy && running) {
            std::printf("    %s: Tier 1 exhausted — escalating\n",
                        dev->config().port_name);
        }
    }

    healthy = count_healthy();
    if (healthy == num_ports || !running) return healthy;

    std::printf("\n  %d/%d ports after Tier 1. %d still failing.\n",
                healthy, num_ports, num_ports - healthy);

    // ---------------------------------------------------------------
    // TIER 2: Software RESET + register reconfig — ~500ms/attempt, 5 attempts/port
    // ---------------------------------------------------------------
    constexpr int TIER2_ATTEMPTS = 5;

    std::printf("\n  --- Tier 2: software RESET + reconfig (%d attempts/port) ---\n",
                TIER2_ATTEMPTS);

    for (int p = 0; p < num_ports; ++p) {
        if (port_health[p].healthy) continue;
        if (!running) return count_healthy();

        auto* dev = devices[p];

        for (int a = 0; a < TIER2_ATTEMPTS && running; ++a) {
            std::printf("    %s: Tier 2 attempt %d/%d...\n",
                        dev->config().port_name, a + 1, TIER2_ATTEMPTS);
            if (ads1299::ADS1299Controller::recover_port_tier2(*dev, config, 10)) {
                std::printf("    %s: Tier 2 recovered on attempt %d [OK]\n",
                            dev->config().port_name, a + 1);
                port_health[p].healthy = true;
                port_health[p].drdy_active = true;
                break;
            }
        }

        if (!port_health[p].healthy && running) {
            std::printf("    %s: Tier 2 exhausted — will escalate to Tier 3\n",
                        dev->config().port_name);
        }
    }

    healthy = count_healthy();
    if (healthy == num_ports || !running) return healthy;

    std::printf("\n  %d/%d ports after Tier 2. %d still failing.\n",
                healthy, num_ports, num_ports - healthy);

    // ---------------------------------------------------------------
    // TIER 3: Full re-init ALL ports from scratch — 2 attempts
    // ---------------------------------------------------------------
    constexpr int TIER3_ATTEMPTS = 2;

    std::printf("\n  --- Tier 3: full re-init ALL ports (%d attempts) ---\n",
                TIER3_ATTEMPTS);
    std::printf("  NOTE: This re-initializes all %d ports from scratch.\n", num_ports);

    for (int t3 = 0; t3 < TIER3_ATTEMPTS && running; ++t3) {
        std::printf("\n  Tier 3 attempt %d/%d — re-initializing all ports...\n",
                    t3 + 1, TIER3_ATTEMPTS);

        // Re-configure all ports (Phase 1 redo)
        bool all_configured = true;
        for (int p = 0; p < num_ports && running; ++p) {
            bool ok = ads1299::ADS1299Controller::initialize_device(*devices[p], config);
            if (!ok) {
                std::printf("    %s: register re-init failed\n",
                            devices[p]->config().port_name);
                all_configured = false;
            }
        }

        if (!running) break;

        if (!all_configured) {
            std::printf("  WARNING: Some ports failed register re-init\n");
        }

        // Start all + verify (Phase 2 redo)
        for (int p = 0; p < num_ports; ++p) {
            port_health[p] = {false, false, 0, 0};
        }

        ads1299::ADS1299Controller::start_and_verify(devices, i2c, port_health, running);

        if (!running) break;

        healthy = count_healthy();
        std::printf("  Tier 3 attempt %d: %d/%d ports healthy\n",
                    t3 + 1, healthy, num_ports);

        if (healthy == num_ports) return healthy;

        // Quick Tier 1 pass on still-failing ports before next T3 attempt
        if (t3 + 1 < TIER3_ATTEMPTS) {
            std::printf("  Quick Tier 1 pass before next Tier 3 attempt...\n");
            for (int p = 0; p < num_ports && running; ++p) {
                if (port_health[p].healthy) continue;
                for (int a = 0; a < 5 && running; ++a) {
                    if (ads1299::ADS1299Controller::recover_port_tier1(*devices[p], 10)) {
                        std::printf("    %s: recovered in post-T3 Tier 1 [OK]\n",
                                    devices[p]->config().port_name);
                        port_health[p].healthy = true;
                        port_health[p].drdy_active = true;
                        break;
                    }
                }
            }
            healthy = count_healthy();
            if (healthy == num_ports) return healthy;
        }
    }

    // Mark remaining failures as dead
    for (int p = 0; p < num_ports; ++p) {
        if (!port_health[p].healthy) {
            port_dead[p] = true;
            devices[p]->start_low();
            devices[p]->send_command(ads1299::Cmd::STOP);
            devices[p]->send_command(ads1299::Cmd::SDATAC);
        }
    }

    return count_healthy();
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

    // --- Phase 3: Tiered recovery ---
    bool port_dead[ads1299::MAX_PORTS] = {};

    if (failing_count > 0) {
        std::printf("\n============================================================\n"
                    "PHASE 3: TIERED RECOVERY (%d/%d ports need recovery)\n"
                    "============================================================\n"
                    "Target: %d/%d ports (--min-ports %d)\n",
                    failing_count, args.num_ports,
                    args.min_ports, args.num_ports, args.min_ports);

        healthy_count = run_tiered_recovery(
            devices, i2c, port_health, port_dead,
            args.num_ports, config, g_running);

        if (!g_running) {
            std::printf("\n[CANCELLED] Ctrl+C during recovery\n");
            return 1;
        }
    }

    // Count dead ports
    int dead_count = 0;
    for (int p = 0; p < args.num_ports; ++p) {
        if (port_dead[p]) dead_count++;
    }

    // --- 7/7 hard gate (or --min-ports gate) ---
    if (healthy_count < args.min_ports) {
        // Clean shutdown all ports
        for (auto* dev : devices) {
            dev->start_low();
            dev->send_command(ads1299::Cmd::STOP);
            dev->send_command(ads1299::Cmd::SDATAC);
        }

        std::fprintf(stderr,
            "\n"
            "============================================================\n"
            "INITIALIZATION FAILED -- POWER CYCLE REQUIRED\n"
            "============================================================\n"
            "\n"
            "The following port(s) did not respond after all recovery attempts:\n");
        for (int p = 0; p < args.num_ports; ++p) {
            if (port_dead[p]) {
                std::fprintf(stderr, "  - %s\n", devices[p]->config().port_name);
            }
        }
        std::fprintf(stderr,
            "\n"
            "This is a known hardware condition. To fix it:\n"
            "\n"
            "  1. Turn off the power supply to the acquisition board\n"
            "  2. Wait 5 seconds\n"
            "  3. Turn the power supply back on\n"
            "  4. Run this program again\n"
            "\n"
            "To run with %d/%d ports anyway, use: --min-ports %d\n"
            "============================================================\n\n",
            healthy_count, args.num_ports, healthy_count);
        return 2;  // exit code 2 = power cycle needed
    }

    if (healthy_count == 0) {
        std::fprintf(stderr, "\n[FAIL] No active ports after recovery\n");
        return 1;
    }

    // --- Build active port lists (excludes dead ports) ---
    std::vector<ads1299::ADS1299Device*> active_devices;
    std::vector<ads1299::PortConfig> active_configs;
    for (int p = 0; p < args.num_ports; ++p) {
        if (!port_dead[p]) {
            active_devices.push_back(devices[p]);
            active_configs.push_back(args.ports[p]);
        }
    }
    int active_num_ports = static_cast<int>(active_devices.size());
    int active_total_devices = 0;
    for (int i = 0; i < active_num_ports; ++i) {
        active_total_devices += active_devices[i]->config().num_devices;
    }
    int active_total_channels = active_total_devices * ads1299::CHANNELS_PER_DEVICE;

    if (dead_count > 0) {
        std::printf("\n  [NOTE] Running with %d/%d ports (%d channels). Dead:",
                    active_num_ports, args.num_ports, active_total_channels);
        for (int p = 0; p < args.num_ports; ++p) {
            if (port_dead[p]) std::printf(" %s", devices[p]->config().port_name);
        }
        std::printf("\n");
    }

    // --- Phase 4: Warmup + final health check ---
    std::printf("\n  --- Warmup: discarding 250 settling samples ---\n");

    // Find a healthy reference port for DRDY timing
    ads1299::ADS1299Device* ref_port = nullptr;
    for (int i = 0; i < active_num_ports; ++i) {
        if (!ref_port || active_devices[i]->config().num_devices > ref_port->config().num_devices) {
            ref_port = active_devices[i];
        }
    }

    if (!ref_port) {
        std::fprintf(stderr, "\n[FAIL] No healthy ports available\n");
        return 1;
    }

    // Discard 250 warmup samples (~1s at 250 Hz)
    for (int i = 0; i < 250 && g_running; ++i) {
        if (ref_port->wait_for_drdy(0.02)) {
            for (int ap = 0; ap < active_num_ports; ++ap) {
                ads1299::PortData pd;
                active_devices[ap]->read_data(pd);
            }
        }
    }

    if (!g_running) {
        std::printf("\n[CANCELLED] Ctrl+C during warmup\n");
        return 1;
    }

    // Final health check: 50 samples
    std::printf("  --- Final health check (50 samples) ---\n");

    int final_valid[ads1299::MAX_PORTS] = {};
    int final_reads = 0;

    for (int i = 0; i < 50 && g_running; ++i) {
        if (ref_port->wait_for_drdy(0.02)) {
            final_reads++;
            for (int ap = 0; ap < active_num_ports; ++ap) {
                ads1299::PortData pd;
                active_devices[ap]->read_data(pd);

                bool ok = true;
                for (int d = 0; d < active_devices[ap]->config().num_devices; ++d) {
                    if ((pd.status_bytes[d][0] & 0xF0) != 0xC0) {
                        ok = false;
                        break;
                    }
                }
                if (ok) final_valid[ap]++;
            }
        }
    }

    // Report health
    for (int ap = 0; ap < active_num_ports; ++ap) {
        double rate = final_reads > 0 ? static_cast<double>(final_valid[ap]) / final_reads : 0;
        std::printf("    %s: %.0f%% valid (%d/%d) %s\n",
                    active_devices[ap]->config().port_name, rate * 100.0,
                    final_valid[ap], final_reads,
                    rate >= 0.90 ? "[OK]" : "[WARN]");
    }

    std::printf("\n============================================================\n"
                "[OK] ALL %d PORTS ACTIVE -- SYSTEM READY (%d channels)\n"
                "============================================================\n",
                active_num_ports, active_total_channels);

    // --- Create DRDY poller (using active ports only) ---
    ads1299::DRDYPoller drdy_poller(i2c, ads1299::TCA9534_DRDY_ADDR);
    for (int i = 0; i < active_num_ports; ++i) {
        if (active_devices[i]->wait_for_drdy(0.1)) {
            drdy_poller.add_pin(active_configs[i].drdy_pin);
        } else {
            // With 7/7 gate, missing DRDY after successful init is a hard error
            std::fprintf(stderr, "[FAIL] %s: DRDY not active after successful init -- hardware error\n",
                         active_devices[i]->config().port_name);
            return 1;
        }
    }

    if (!drdy_poller.has_pins()) {
        std::fprintf(stderr, "[FAIL] No ports have active DRDY\n");
        return 1;
    }

    // --- Create SPSC ring buffer ---
    auto ring = std::make_unique<ads1299::SPSCRing<ads1299::Sample>>();

    // --- Create bus workers (using active ports only) ---
    auto bus_groups = ads1299::group_ports_by_bus(active_configs.data(), active_num_ports);

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
            group_devices.push_back(active_devices[idx]);
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

    // --- Create CSV writer (using active ports only) ---
    ads1299::CSVWriter* csv_writer = nullptr;
    std::unique_ptr<ads1299::CSVWriter> csv_owner;
    if (args.full_csv) {
        std::vector<std::string> names;
        for (int ap = 0; ap < active_num_ports; ++ap) {
            for (int d = 0; d < active_configs[ap].num_devices; ++d) {
                for (int c = 0; c < ads1299::CHANNELS_PER_DEVICE; ++c) {
                    char name[64];
                    std::snprintf(name, sizeof(name), "%s_dev%d_ch%d",
                                  active_configs[ap].port_name, d + 1, c + 1);
                    names.push_back(name);
                }
            }
        }

        std::vector<const char*> name_ptrs;
        name_ptrs.reserve(names.size());
        for (auto& n : names) {
            name_ptrs.push_back(n.c_str());
        }

        csv_owner = std::make_unique<ads1299::CSVWriter>(
            "all_channels_data.csv", *ring, active_total_channels,
            name_ptrs.data(), static_cast<int>(name_ptrs.size()));
        csv_writer = csv_owner.get();
        csv_writer->start();
        std::printf("  CSV writer: all %d channels -> all_channels_data.csv (async)\n",
                    active_total_channels);
    }

    // --- Create streaming server (using active ports only) ---
    ads1299::StreamingServer streaming_server(active_configs.data(), active_num_ports,
                                             active_total_channels, active_total_devices);
    streaming_server.start(args.host, args.port);

    // --- Create and run acquisition engine ---
    std::printf("\n============================================================\n"
                "ACQUISITION RUNNING - Ctrl+C to stop\n"
                "============================================================\n\n");

    ads1299::AcquisitionEngine engine(active_devices, workers, drdy_poller, *ring);
    engine.set_csv_enabled(args.full_csv);
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

    // Stats display thread: pinned to core 1 (housekeeping core)
    // Polls engine.stats_ready_ flag every 1s, prints stats when signaled.
    // Replaces the old print_stats() call that was inside the RT hot loop.
    std::thread stats_thread([&]() {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

        while (g_running) {
            sleep_sec(1.0);

            if (engine.stats_ready_.exchange(false, std::memory_order_acquire)) {
                auto s = engine.get_stats();
                double rate = s.runtime_sec > 0 ? s.total_samples / s.runtime_sec : 0;

                std::printf("\n  [%6.0fs] samples=%lu rate=%.1fHz cycle=%.2f/%.2f/%.2fms "
                            "dt=%.2f/%.2fms timeouts=%lu corrupt=%lu drops=%lu\n",
                            s.runtime_sec,
                            static_cast<unsigned long>(s.total_samples),
                            rate, s.min_cycle_ms, s.mean_cycle_ms, s.max_cycle_ms,
                            s.min_dt_ms, s.max_dt_ms,
                            static_cast<unsigned long>(s.drdy_timeouts),
                            static_cast<unsigned long>(s.corruption_count),
                            static_cast<unsigned long>(s.drop_count));

                auto ss = streaming_server.get_stats();
                std::printf("           stream: sent=%lu batches=%lu drops=%lu queued=%zu %s\n",
                            static_cast<unsigned long>(ss.samples_sent),
                            static_cast<unsigned long>(ss.batches_sent),
                            static_cast<unsigned long>(ss.drops),
                            ss.ring_queued,
                            ss.connected ? "[connected]" : "[no client]");
            }
        }
    });

    engine.run(g_running);

    // Join stats thread after engine.run() returns
    if (stats_thread.joinable()) {
        stats_thread.join();
    }

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
                "Corruptions:    %lu\n",
                static_cast<unsigned long>(stats.total_samples),
                stats.runtime_sec,
                rate,
                stats.min_cycle_ms, stats.mean_cycle_ms, stats.max_cycle_ms,
                stats.min_dt_ms, stats.max_dt_ms,
                static_cast<unsigned long>(stats.drdy_timeouts),
                static_cast<unsigned long>(stats.corruption_count));

    if (csv_writer) {
        std::printf("Ring drops:     %lu\n"
                    "CSV written:    %lu\n",
                    static_cast<unsigned long>(stats.drop_count),
                    static_cast<unsigned long>(csv_writer->total_written()));
    }

    std::printf("Stream sent:    %lu (batches: %lu, drops: %lu, reconnects: %lu)\n"
                "============================================================\n\n",
                static_cast<unsigned long>(ss.samples_sent),
                static_cast<unsigned long>(ss.batches_sent),
                static_cast<unsigned long>(ss.drops),
                static_cast<unsigned long>(ss.reconnects));

    std::printf("System shutdown complete\n");
    return 0;
}
