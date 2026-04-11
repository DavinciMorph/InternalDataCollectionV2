#pragma once

#include <cstdint>
#include <vector>

namespace ads1299 {

class I2CDevice;

// Single-pin control for a TCA9534 GPIO expander.
// Ported from Python's TCA9534_Pin class.
class TCA9534Pin {
public:
    // Configure pin direction (input for DRDY, output for START).
    // Output pins are initialized LOW.
    TCA9534Pin(I2CDevice& i2c, uint8_t addr, uint8_t pin, bool is_input);

    bool read();                 // Read pin state (true = HIGH)
    void set_high();             // Set output HIGH (read-modify-write)
    void set_low();              // Set output LOW (read-modify-write)

    uint8_t addr() const { return addr_; }
    uint8_t pin() const { return pin_; }
    uint8_t mask() const { return pin_mask_; }

private:
    I2CDevice& i2c_;
    uint8_t addr_;
    uint8_t pin_;
    uint8_t pin_mask_;
};

// Atomic multi-pin control for synchronized START assertion.
// Ported from Python's TCA9534_MultiPin class.
class TCA9534MultiPin {
public:
    explicit TCA9534MultiPin(I2CDevice& i2c);

    void add_pin(uint8_t addr, uint8_t pin);
    void set_all_high();         // Single I2C write per address for all pins atomically
    void set_all_low();

private:
    I2CDevice& i2c_;

    struct AddrMask {
        uint8_t addr;
        uint8_t mask;
    };
    std::vector<AddrMask> entries_;

    // Find or create entry for address
    AddrMask& get_entry(uint8_t addr);
};

} // namespace ads1299
