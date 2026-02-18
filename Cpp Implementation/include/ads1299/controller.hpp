#pragma once

#include "ads1299/registers.hpp"
#include "ads1299/types.hpp"

#include <vector>

namespace ads1299 {

class ADS1299Device;
class TCA9534MultiPin;
class I2CDevice;

// ADS1299 device initialization and control logic.
// Faithful port of Python's ADS1299_Controller static methods.
// ALL retry loops, sleep durations, and warmup sequences are preserved exactly.
class ADS1299Controller {
public:
    // Initialize a single ADS1299 port (full reset, SDATAC verify, register write+verify).
    // Returns true on success.
    static bool initialize_device(ADS1299Device& dev, const DeviceConfig& config);

    // Start all conversions with synchronized START assertion.
    // Includes RDATAC, START, DRDY verify, warmup, re-init, re-sync.
    // Returns true if all ports verified OK.
    static bool start_all_conversions_synchronized(
        std::vector<ADS1299Device*>& devices,
        I2CDevice& i2c,
        const DeviceConfig* config = nullptr);

    // Force all START pins LOW before initialization
    static void force_all_start_pins_low(std::vector<ADS1299Device*>& devices);

    // Restart a single port by cycling STOP/SDATAC/RDATAC/START
    static bool restart_single_port(ADS1299Device& dev, int attempt = 0,
                                    bool verbose = true);

    // Send command to all devices
    static void broadcast_command(std::vector<ADS1299Device*>& devices,
                                  uint8_t cmd, double delay_ms = 10.0);

    // Create a TCA9534MultiPin for all START pins
    static TCA9534MultiPin create_start_controller(
        std::vector<ADS1299Device*>& devices, I2CDevice& i2c);

private:
    // Write register and verify readback (with retry)
    static bool write_and_verify(ADS1299Device& dev, Reg reg, uint8_t value,
                                 const char* name, int max_attempts = 100);

    // Verify register matches expected value (with retry + SDATAC recovery)
    static bool verify_register(ADS1299Device& dev, Reg reg, uint8_t expected,
                                const char* name, uint8_t& actual);
};

} // namespace ads1299
