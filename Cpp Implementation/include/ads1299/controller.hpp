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
