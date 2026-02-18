#pragma once

#include <cstdint>

namespace ads1299 {

class I2CDevice;

// Reads I2C expander and checks combined DRDY mask for all active ports.
// No sleep/yield between polls — engine runs on isolated core 3 with SCHED_FIFO.
// Polls at ~5-10 kHz (limited by I2C wire time), consuming 1-2ms before DRDY fires.
class DRDYPoller {
public:
    DRDYPoller(I2CDevice& i2c, uint8_t expander_addr);

    // Add a port's DRDY pin to the combined mask
    void add_pin(uint8_t pin);

    // Remove a pin from the mask (for dead ports)
    void remove_pin(uint8_t pin);

    // Poll until all DRDYs are LOW (active). Returns true if ready, false on timeout.
    // Uses CLOCK_MONOTONIC for timeout. No sleep between polls.
    bool poll(double timeout_sec);

    uint8_t mask() const { return mask_; }
    bool has_pins() const { return mask_ != 0; }

private:
    I2CDevice& i2c_;
    uint8_t addr_;
    uint8_t mask_;  // Combined mask of all DRDY pins to check
};

} // namespace ads1299
