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
    bool full_csv = true;   // Default: full 224-channel CSV
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
                        "Default: 7 ports x 4 devices = 224 channels @ 250 Hz\n"
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

    // --- Configure all devices ---
    std::printf("\n============================================================\n"
                "CONFIGURING DEVICES\n"
                "============================================================\n");

    ads1299::DeviceConfig config;

    for (int i = 0; i < args.num_ports; ++i) {
        bool success = ads1299::ADS1299Controller::initialize_device(*devices[i], config);
        if (!success) {
            std::fprintf(stderr, "\n[FAIL] %s failed to initialize\n",
                         devices[i]->config().port_name);
            return 1;
        }
        sleep_sec(0.1);  // Gap between ports
    }

    std::printf("\n[OK] All devices configured successfully!\n");

    // --- Start conversions ---
    std::printf("\n============================================================\n"
                "VERIFYING DATA FLOW\n"
                "============================================================\n");

    bool data_flow_ok = ads1299::ADS1299Controller::start_all_conversions_synchronized(
        devices, i2c, &config);

    if (!data_flow_ok) {
        std::printf("\n[WARN] Data flow verification had issues\n");
        // Check if DRDY is at least active
        bool drdy_active = false;
        for (auto* dev : devices) {
            if (dev->wait_for_drdy(0.1)) {
                drdy_active = true;
                break;
            }
        }
        if (drdy_active) {
            std::printf("DRDY active - continuing...\n");
        } else {
            std::fprintf(stderr, "DRDY NOT active - aborting\n");
            return 1;
        }
    } else {
        std::printf("\n============================================================\n"
                    "[OK] SYSTEM INITIALIZATION COMPLETE!\n"
                    "============================================================\n");
    }

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
    // SPI0→core 0, SPI3→core 1, SPI4→core 2, SPI5→core 0 (1 port, finishes fast)
    auto bus_groups = ads1299::group_ports_by_bus(args.ports, args.num_ports);

    auto get_core = [](int bus_num) -> int {
        switch (bus_num) {
            case 0: return 0;
            case 3: return 1;
            case 4: return 2;
            case 5: return 0;  // SPI5 has only 1 port, safe to share core 0
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
