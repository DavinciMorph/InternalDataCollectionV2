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
// Init strategy — tiered recovery (revised 2026-02-19):
//
//   Phase 1: initialize_device()    — register configuration. Always succeeds. Do once.
//   Phase 2: start_and_verify()     — RDATAC + START + verify all ports.
//   Phase 3: tiered recovery (driven by main.cpp):
//     Tier 1: recover_port_tier1()  — RDATAC cycle (~200ms). Targets timing race.
//     Tier 2: recover_port_tier2()  — Software RESET + reconfig (~500ms). Targets latch failures.
//     Tier 3: Full re-init all ports (main.cpp calls initialize_device + start_and_verify).
//     Tier 4: Exit with "power cycle required" message.
//
// V2 board: RESET/PWDN tied HIGH — software RESET (0x06) is the most aggressive
// reset available without a physical power cycle.
class ADS1299Controller {
public:
    // Initialize a single ADS1299 port (full reset, SDATAC verify, register write+verify).
    // Returns true on success. This always succeeds if hardware is present.
    static bool initialize_device(ADS1299Device& dev, const DeviceConfig& config);

    // Start all conversions with synchronized START, then verify each port.
    // Does NOT retry or recover anything — just reports per-port health.
    // Caller (main.cpp) handles per-port recovery via the tiered strategy.
    //
    // healthy_out: array of PortHealth, one per device. Must have devices.size() entries.
    // running: checked between steps so Ctrl+C works during startup.
    static void start_and_verify(
        std::vector<ADS1299Device*>& devices,
        I2CDevice& i2c,
        PortHealth* healthy_out,
        const volatile sig_atomic_t& running);

    // Tier 1 recovery: STOP->SDATAC->flush->RDATAC->START, then verify.
    // Fixed ~200ms per attempt (no escalating delay). Targets RDATAC timing race.
    // Returns true if port is now producing valid data.
    static bool recover_port_tier1(ADS1299Device& dev, int verify_samples = 10);

    // Tier 2 recovery: software RESET + register reconfig + RDATAC->START.
    // ~500ms per attempt. Targets digital core latch failures that Tier 1 cannot clear.
    // Returns true if port is now producing valid data.
    static bool recover_port_tier2(ADS1299Device& dev, const DeviceConfig& config,
                                   int verify_samples = 10);

    // Force all START pins LOW
    static void force_all_start_pins_low(std::vector<ADS1299Device*>& devices);

    // Send command to all devices
    static void broadcast_command(std::vector<ADS1299Device*>& devices,
                                  uint8_t cmd, double delay_ms = 10.0);

    // Create a TCA9534MultiPin for all START pins
    static TCA9534MultiPin create_start_controller(
        std::vector<ADS1299Device*>& devices, I2CDevice& i2c);

    // Legacy wrappers — call recover_port_tier1() internally
    static bool recover_port(ADS1299Device& dev, int attempt = 0,
                             int verify_samples = 5);
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

    // Write all config registers without SDATAC verify loop or ID check.
    // Used by Tier 2 where hardware presence is already confirmed.
    static bool write_registers(ADS1299Device& dev, const DeviceConfig& config);
};

} // namespace ads1299
