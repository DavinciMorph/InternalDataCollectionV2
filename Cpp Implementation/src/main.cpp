#include "ads1299/registers.hpp"
#include "ads1299/types.hpp"
#include "ads1299/spi_device.hpp"
#include "ads1299/controller.hpp"
#include "ads1299/impedance.hpp"
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
    int config3 = -1;       // -1 = use default (0xE0); override with --config3 0x60
    int chnset = -1;        // -1 = use default (0x60); override with --chnset 0x61
    int sps = -1;           // -1 = use default (250); override with --sps 500
    bool test_signal = false; // --test-signal: CONFIG2=0xD0, CHnSET=0x65
    int bias_port = 0;      // 0 = disabled; 1-7 = enable BIAS drive on that port number
    bool check_impedance = false; // --check-impedance: run continuous impedance check mode
    int monitor_supply = 0;     // 0 = disabled; 1-7 = port number to use ch8 as MVDD monitor
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
        } else if (std::strcmp(argv[i], "--config3") == 0 && i + 1 < argc) {
            args.config3 = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
        } else if (std::strcmp(argv[i], "--chnset") == 0 && i + 1 < argc) {
            args.chnset = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
        } else if (std::strcmp(argv[i], "--sps") == 0 && i + 1 < argc) {
            args.sps = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--test-signal") == 0) {
            args.test_signal = true;
        } else if (std::strcmp(argv[i], "--bias-port") == 0 && i + 1 < argc) {
            args.bias_port = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--check-impedance") == 0) {
            args.check_impedance = true;
        } else if (std::strcmp(argv[i], "--monitor-supply") == 0 && i + 1 < argc) {
            args.monitor_supply = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--dev") == 0) {
            // Dev mode: single ADS1299 on SPI0.0 (Port1, 1 device)
            ports_specified = true;
            args.num_ports = 0;
            if (ads1299::make_port_config(0, 0, "Port1", 1, args.ports[0])) {
                args.num_ports = 1;
            }
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::printf("Usage: %s [--ports bus,dev,name,num_daisy ...] "
                        "[--full-csv] [--no-csv] [--duration SEC] "
                        "[--host ADDR] [--port PORT] [--min-ports N]\n"
                        "       [--config3 0xHH] [--chnset 0xHH] [--sps RATE] "
                        "[--test-signal] [--bias-port N]\n\n"
                        "Default: 7 ports x 41 devices = 328 channels @ 250 Hz\n"
                        "         TCP streaming on 0.0.0.0:8888\n"
                        "         CSV disabled (use --full-csv to enable server-side CSV)\n"
                        "         All ports required (--min-ports = num_ports)\n\n"
                        "Port format: bus,device,name,num_daisy\n"
                        "  bus: SPI bus (0, 3, 4, 5)\n"
                        "  device: SPI device (0-1)\n"
                        "  name: Port name (e.g. Port1)\n"
                        "  num_daisy: Number of daisy-chained ADS1299s\n\n"
                        "  --min-ports N:  minimum required ports (default: all)\n"
                        "                  Use to allow degraded operation for testing\n"
                        "  --config3 0xHH: override CONFIG3 register (default: 0xE0)\n"
                        "                  0xE0 = internal ref, 0x60 = external ref\n"
                        "  --chnset 0xHH:  override all CHnSET registers (default: 0x60)\n"
                        "                  0x60 = normal input gain=24, 0x61 = shorted input\n"
                        "  --sps RATE:     sample rate (default: 250)\n"
                        "                  250, 500, 1000, 2000, 4000, 8000, 16000\n"
                        "  --test-signal:  enable internal test signal\n"
                        "                  sets CONFIG2=0xD0, CHnSET=0x05 (gain=1, test mux)\n"
                        "  --bias-port N:  enable BIAS drive on port N (1-7, default: disabled)\n"
                        "                  Channels 1-7 route to bias amp, ch8 drives BIAS out\n"
                        "                  Sets CONFIG3=0xF0, BIAS_SENSP/N=0x7F, CH8SET=0x06\n"
                        "  --monitor-supply N: configure last device ch8 on port N (1-7) as MVDD monitor\n"
                        "                  Sets CH8SET=0x03 (gain=1, MUX=MVDD) for in-band supply tracking\n"
                        "                  Expected AVDD=5.0V reads as ~9.3M LSB; 65mV shift = ~121K LSB\n",
                        argv[0]);
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
    const ads1299::DeviceConfig* port_configs,
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
    // TIER 1: RDATAC cycling — ~200ms/attempt, 100 attempts/port
    // ---------------------------------------------------------------
    constexpr int TIER1_ATTEMPTS = 100;

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
            if (ads1299::ADS1299Controller::recover_port_tier2(*dev, port_configs[p], 10)) {
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
            bool ok = ads1299::ADS1299Controller::initialize_device(*devices[p], port_configs[p]);
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

    int display_sps = args.sps > 0 ? args.sps : 250;
    std::printf("\n============================================================\n"
                "ADS1299 C++ ACQUISITION ENGINE (Phase 2: TCP Streaming)\n"
                "============================================================\n"
                "Config: %d ports, %d devices, %d channels @ %dHz\n"
                "Stream: %s:%d\n\n",
                args.num_ports, total_devices, total_channels, display_sps,
                args.host, args.port);

    // --- Disable WiFi power save ---
    // WiFi beacon wakeups (100 TU = 102.4ms = 9.766 Hz) cause periodic current
    // spikes on the 3.3V rail that couple EMI into ADS1299 analog inputs, creating
    // a harmonic comb at 9.766 Hz intervals (Q>230). Disabling power save keeps
    // the radio continuously active, eliminating the periodic transients.
    if (std::system("iw dev wlan0 set power_save off 2>/dev/null") == 0) {
        std::printf("WiFi power save disabled (anti-EMI)\n");
    }

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

    // Build base config (shared by all ports)
    ads1299::DeviceConfig base_config;

    // Apply CLI overrides (test-signal first so --config3/--chnset can still override)
    if (args.test_signal) {
        base_config.config2 = 0xD0;  // Internal test signal, 1x amplitude, slow pulse
        base_config.ch1set = 0x05;   // Gain=1, MUX=test signal
        base_config.ch2set = 0x05;
        base_config.ch3set = 0x05;
        base_config.ch4set = 0x05;
        base_config.ch5set = 0x05;
        base_config.ch6set = 0x05;
        base_config.ch7set = 0x05;
        base_config.ch8set = 0x05;
        std::printf("  Test signal enabled: CONFIG2=0xD0, CHnSET=0x05 (gain=1, test mux)\n");
    }
    if (args.sps > 0) {
        // CONFIG1[2:0] = DR bits. Upper bits preserved (0x96 = daisy + 250 SPS).
        // DR: 000=16k, 001=8k, 010=4k, 011=2k, 100=1k, 101=500, 110=250
        uint8_t dr_bits;
        switch (args.sps) {
            case 250:   dr_bits = 0x06; break;
            case 500:   dr_bits = 0x05; break;
            case 1000:  dr_bits = 0x04; break;
            case 2000:  dr_bits = 0x03; break;
            case 4000:  dr_bits = 0x02; break;
            case 8000:  dr_bits = 0x01; break;
            case 16000: dr_bits = 0x00; break;
            default:
                std::fprintf(stderr, "Error: Invalid --sps %d. "
                             "Valid: 250, 500, 1000, 2000, 4000, 8000, 16000\n",
                             args.sps);
                return 1;
        }
        base_config.config1 = (base_config.config1 & 0xF8) | dr_bits;
        std::printf("  Sample rate override: %d SPS (CONFIG1=0x%02X)\n",
                    args.sps, base_config.config1);
    }
    if (args.config3 >= 0) {
        base_config.config3 = static_cast<uint8_t>(args.config3);
        std::printf("  CONFIG3 override: 0x%02X\n", base_config.config3);
    }
    if (args.chnset >= 0) {
        uint8_t val = static_cast<uint8_t>(args.chnset);
        base_config.ch1set = val;
        base_config.ch2set = val;
        base_config.ch3set = val;
        base_config.ch4set = val;
        base_config.ch5set = val;
        base_config.ch6set = val;
        base_config.ch7set = val;
        base_config.ch8set = val;
        std::printf("  CHnSET override: 0x%02X (all 8 channels)\n", val);
    }

    // Build per-port config array. All ports start with base_config.
    // The BIAS port gets modified register values.
    ads1299::DeviceConfig port_configs[ads1299::MAX_PORTS];
    for (int i = 0; i < args.num_ports; ++i) {
        port_configs[i] = base_config;
    }

    // Apply BIAS configuration to the specified port (1-indexed)
    if (args.bias_port > 0) {
        int bias_idx = args.bias_port - 1;  // Convert to 0-indexed
        if (bias_idx >= args.num_ports) {
            std::fprintf(stderr, "Error: --bias-port %d exceeds number of ports (%d)\n",
                         args.bias_port, args.num_ports);
            return 1;
        }

        auto& bc = port_configs[bias_idx];

        // CONFIG3 bit 4 (PD_BIAS): power on the bias amplifier
        // Base: 0xE0 = 1110_0000 -> 0xF0 = 1111_0000
        bc.config3 = static_cast<uint8_t>(bc.config3 | 0x10);

        // BIAS_SENSP: route P-inputs of channels 1-7 into the bias amplifier
        // Bit 0=ch1, bit 1=ch2, ..., bit 6=ch7, bit 7=ch8
        // Ch8 is the bias OUTPUT electrode, so exclude it from inputs
        bc.bias_sensp = 0x7F;  // Channels 1-7 P-inputs

        // BIAS_SENSN: route N-inputs of channels 1-7 into the bias amplifier
        // Same routing as SENSP for maximum common-mode rejection
        bc.bias_sensn = 0x7F;  // Channels 1-7 N-inputs

        // CH8SET: MUX=BIAS_DRP (0b110) to drive the bias amplifier output
        // through channel 8's positive electrode. Gain=1 (safe for bias output).
        // ADS1299 datasheet Table 18: MUX[2:0]=110 = BIAS_DRP
        bc.ch8set = 0x06;  // PGA gain=1 (bits[6:4]=000), MUX=BIAS_DRP (bits[2:0]=110)

        std::printf("  BIAS drive enabled on Port%d:\n", args.bias_port);
        std::printf("    CONFIG3  = 0x%02X (PD_BIAS=1, bias amplifier powered on)\n", bc.config3);
        std::printf("    BIAS_SENSP = 0x%02X (ch1-7 P-inputs routed to bias amp)\n", bc.bias_sensp);
        std::printf("    BIAS_SENSN = 0x%02X (ch1-7 N-inputs routed to bias amp)\n", bc.bias_sensn);
        std::printf("    CH8SET   = 0x%02X (gain=1, MUX=BIAS_DRP — bias output)\n", bc.ch8set);
    }

    // B.6: Apply supply monitor config to the last device's ch8 on the specified port
    if (args.monitor_supply > 0) {
        int mon_idx = args.monitor_supply - 1;  // Convert to 0-indexed
        if (mon_idx >= args.num_ports) {
            std::fprintf(stderr, "Error: --monitor-supply %d exceeds number of ports (%d)\n",
                         args.monitor_supply, args.num_ports);
            return 1;
        }
        // ADS1299 datasheet Table 18: MUX[2:0]=011 = MVDD for supply measurement
        // PGA[6:4]=000 = gain 1 (full-scale ±4.5V with VREF=4.5V)
        // MVDD reading: 24-bit signed, full-scale = ±VREF/gain = ±4.5V
        // Expected AVDD = 5.0V → ADC code ≈ (5.0/4.5) * 2^23 ≈ 9,320,675
        // A 65mV shift would appear as Δcode ≈ (0.065/4.5) * 2^23 ≈ 121,099
        port_configs[mon_idx].ch8set = 0x03;
        std::printf("  Supply monitor on Port%d last device ch8: CH8SET=0x03 (gain=1, MUX=MVDD)\n",
                    args.monitor_supply);
    }

    for (int i = 0; i < args.num_ports; ++i) {
        if (!g_running) {
            std::printf("\n[CANCELLED] Ctrl+C during configuration\n");
            return 1;
        }
        bool success = ads1299::ADS1299Controller::initialize_device(*devices[i], port_configs[i]);
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
            args.num_ports, port_configs, g_running);

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

    // --- Impedance check mode: reconfigure for impedance then fall through to normal streaming ---
    if (args.check_impedance) {
        std::fprintf(stderr,
            "\n============================================================\n"
            "IMPEDANCE MODE: reconfiguring for impedance measurement\n"
            "============================================================\n");

        // Stop all ports before reconfiguring
        for (auto* dev : active_devices) {
            dev->start_low();
        }
        sleep_sec(0.02);
        for (auto* dev : active_devices) {
            dev->send_command(ads1299::Cmd::STOP);
        }
        sleep_sec(0.01);
        for (int rep = 0; rep < 10; ++rep) {
            for (auto* dev : active_devices) {
                dev->send_command(ads1299::Cmd::SDATAC);
            }
            sleep_sec(0.01);
        }
        sleep_sec(0.05);

        // Reconfigure each port for impedance measurement:
        //   - Gain=1 (CHnSET=0x00) — allows measurement up to ~750kΩ at 6µA
        //   - SRB1 ON (MISC1=0x20) — provides current return path through body
        //   - P-side excitation only (LOFF_SENSP=0xFF, LOFF_SENSN=0x00)
        //     Current flows: P electrode → skin → body → SRB1 electrode → return
        //   - LOFF: 6µA AC at fDR/4 (62.5Hz at 250 SPS)
        //   - Lead-off comparators enabled (CONFIG4=0x02)
        for (auto* dev : active_devices) {
            // Gain=1 on all channels
            for (int ch = 0; ch < 8; ++ch) {
                ads1299::Reg reg = static_cast<ads1299::Reg>(
                    static_cast<uint8_t>(ads1299::Reg::CH1SET) + ch);
                dev->write_register(reg, 0x00);
                sleep_sec(0.002);
            }
            // SRB1 ON — connects all N inputs
            dev->write_register(ads1299::Reg::MISC1, 0x20);
            sleep_sec(0.01);
            // LOFF: 6µA AC excitation at fDR/4 (62.5 Hz)
            // bits[3:2]=10 (6µA) | bits[1:0]=11 (fDR/4) = 0x0B
            dev->write_register(ads1299::Reg::LOFF, 0x0B);
            sleep_sec(0.01);
            // P-side only, SRB1 ON
            dev->write_register(ads1299::Reg::LOFF_SENSP, 0xFF);
            sleep_sec(0.01);
            dev->write_register(ads1299::Reg::LOFF_SENSN, 0x00);
            sleep_sec(0.01);
            // Enable lead-off comparators
            dev->write_register(ads1299::Reg::CONFIG4, 0x02);
            sleep_sec(0.01);

            // Verify critical registers (readback only works for first device in chain)
            uint8_t rb_ch7  = dev->read_register(ads1299::Reg::CH7SET);
            uint8_t rb_loff = dev->read_register(ads1299::Reg::LOFF);
            uint8_t rb_sensp = dev->read_register(ads1299::Reg::LOFF_SENSP);
            uint8_t rb_misc1 = dev->read_register(ads1299::Reg::MISC1);
            uint8_t rb_cfg4 = dev->read_register(ads1299::Reg::CONFIG4);
            uint8_t rb_sensn = dev->read_register(ads1299::Reg::LOFF_SENSN);
            uint8_t rb_cfg3 = dev->read_register(ads1299::Reg::CONFIG3);
            std::fprintf(stderr, "  %s verify: CH7SET=0x%02X(exp 0x00) LOFF=0x%02X(exp 0x0B) "
                                 "SENSP=0x%02X(exp 0xFF) SENSN=0x%02X(exp 0x00) MISC1=0x%02X(exp 0x20) "
                                 "CONFIG3=0x%02X CONFIG4=0x%02X(exp 0x02)\n",
                         dev->config().port_name, rb_ch7, rb_loff, rb_sensp, rb_sensn, rb_misc1, rb_cfg3, rb_cfg4);
        }

        // Re-enter RDATAC + START on all ports
        for (auto* dev : active_devices) {
            dev->send_command(ads1299::Cmd::RDATAC);
        }
        sleep_sec(0.05);
        {
            ads1299::TCA9534MultiPin start_ctrl(i2c);
            for (auto* dev : active_devices) {
                start_ctrl.add_pin(dev->config().start_i2c_addr, dev->config().start_pin);
            }
            start_ctrl.set_all_high();
        }
        sleep_sec(0.1);

        std::fprintf(stderr, "Impedance mode: gain=1, SRB1 on, LOFF 6µA AC P-side @ 62.5Hz\n"
                             "Streaming on %s:%d\n", args.host, args.port);

        // Fall through to normal Phase 4 + streaming — the impedance data is
        // in the raw sample stream, client-side DFT extracts it
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

    // B.6: Wire up supply monitor channel if configured
    if (args.monitor_supply > 0) {
        int mon_idx = args.monitor_supply - 1;
        // The supply channel is ch8 of the last device on the monitored port.
        // Compute global channel index: sum all channels of ports before mon_idx,
        // then add (num_devices - 1) * 8 + 7 for the last device's ch8.
        int global_ch = 0;
        for (int p = 0; p < active_num_ports; ++p) {
            int n_dev = active_devices[p]->config().num_devices;
            int n_ch = n_dev * ads1299::CHANNELS_PER_DEVICE;
            if (p == mon_idx) {
                // ch8 of last device = last channel of this port
                global_ch += n_ch - 1;
                break;
            }
            global_ch += n_ch;
        }
        engine.set_supply_monitor_channel(global_ch);
        std::printf("  Supply monitor: global channel %d (Port%d last device ch8)\n",
                    global_ch, args.monitor_supply);
    }

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

                // B.1: DC offset watchdog
                if (s.offset_alert) {
                    std::printf("           [OFFSET ALERT] %d/%d channels shifted, max_offset=%d LSB (%.3f mV)",
                                s.offset_alert_count, active_total_channels,
                                s.max_offset, s.max_offset * 0.0223e-3);
                    if (s.offset_alert_onset > 0.0) {
                        std::printf(" onset=%.1fs", s.offset_alert_onset);
                    }
                    std::printf("\n");
                    // B.1b: Per-port breakdown
                    std::printf("           port offsets:");
                    for (int p = 0; p < s.num_offset_ports; ++p) {
                        std::printf(" P%d=%+dK", p + 1, s.port_offset[p] / 1000);
                    }
                    std::printf(" LSB\n");
                }

                // B.5: Status byte tracking
                bool status_anomaly = false;
                for (int p = 0; p < s.num_status_ports; ++p) {
                    if ((s.status_bytes[p] & 0xF0) != 0xC0) {
                        status_anomaly = true;
                        break;
                    }
                }
                if (status_anomaly) {
                    std::printf("           [STATUS ALERT] status bytes:");
                    for (int p = 0; p < s.num_status_ports; ++p) {
                        std::printf(" 0x%02X", s.status_bytes[p]);
                    }
                    std::printf("\n");
                }

                // B.3: I2C error counter
                uint64_t i2c_errs = i2c.error_count();
                // B.4: DRDY poll iteration counter
                uint32_t drdy_last_iter = drdy_poller.last_poll_iterations();
                uint32_t drdy_max_iter = drdy_poller.max_poll_iterations();
                std::printf("           diag: i2c_errors=%lu drdy_iter(last=%u max=%u)",
                            static_cast<unsigned long>(i2c_errs),
                            drdy_last_iter, drdy_max_iter);

                // B.2: START pin watchdog — read TCA9534 output register
                uint8_t start_reg = i2c.read_byte(0x21, ads1299::TCA9534_OUTPUT_PORT);
                uint8_t expected_start = 0;
                for (int ap = 0; ap < active_num_ports; ++ap) {
                    expected_start |= static_cast<uint8_t>(1u << active_configs[ap].start_pin);
                }
                if ((start_reg & expected_start) != expected_start) {
                    std::printf(" [ALERT] START pins changed! reg=0x%02X expected=0x%02X",
                                start_reg, expected_start);
                }
                std::printf("\n");

                // B.6: Supply voltage monitor
                if (s.supply_monitor_active) {
                    // ADS1299 MVDD measurement: gain=1, VREF=4.5V (internal)
                    // Full-scale = 2^23 = 8,388,608 corresponds to VREF = 4.5V
                    // Voltage = code * 4.5 / 2^23
                    constexpr double LSB_TO_V = 4.5 / 8388608.0;
                    double v_now = s.supply_value * LSB_TO_V;
                    double v_min = s.supply_min * LSB_TO_V;
                    double v_max = s.supply_max * LSB_TO_V;
                    std::printf("           supply: %.4fV (min=%.4fV max=%.4fV range=%.1fmV code=%d)\n",
                                v_now, v_min, v_max,
                                (v_max - v_min) * 1000.0,
                                s.supply_value);
                }
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

    // Stop ADS1299 devices — full cleanup to leave hardware in clean state
    for (auto* dev : devices) {
        dev->start_low();
        dev->send_command(ads1299::Cmd::STOP);
        // SDATAC multiple times to ensure all devices exit RDATAC mode
        for (int i = 0; i < 10; ++i) {
            dev->send_command(ads1299::Cmd::SDATAC);
        }
        dev->flush_spi();
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
