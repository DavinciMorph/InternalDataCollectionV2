#pragma once

#include <cstdint>
#include <time.h>

namespace ads1299 {

class I2CDevice;

// Reads I2C expander and checks combined DRDY mask for all active ports.
// Polls on a fixed 400µs grid (2500 Hz) using clock_nanosleep(TIMER_ABSTIME).
// 2500 Hz = 10 × 250 Hz sample rate, so I2C EMI aliases land at 0 Hz (DC),
// invisible in AC-coupled EEG. Variable-rate polling aliases at ~70 Hz.
class DRDYPoller {
public:
    // Poll interval: 400µs = 2500 Hz. Must evenly divide the 4ms sample period
    // so that I2C bit-bang EMI aliases land exactly at DC (0 Hz).
    static constexpr long POLL_INTERVAL_NS = 400'000L;
    static_assert(4'000'000L % POLL_INTERVAL_NS == 0,
                  "Poll interval must evenly divide 4ms sample period");

    DRDYPoller(I2CDevice& i2c, uint8_t expander_addr);

    // Add a port's DRDY pin to the combined mask
    void add_pin(uint8_t pin);

    // Remove a pin from the mask (for dead ports)
    void remove_pin(uint8_t pin);

    // Poll until all DRDYs are LOW (active). Returns true if ready, false on timeout.
    // Uses CLOCK_MONOTONIC absolute-time grid for fixed-interval polling.
    bool poll(double timeout_sec);

    uint8_t mask() const { return mask_; }
    bool has_pins() const { return mask_ != 0; }

    // Poll iteration diagnostics: how many I2C reads per DRDY event
    uint32_t last_poll_iterations() const { return last_poll_iterations_; }
    uint32_t max_poll_iterations() const { return max_poll_iterations_; }

private:
    I2CDevice& i2c_;
    uint8_t addr_;
    uint8_t mask_;  // Combined mask of all DRDY pins to check

    uint32_t last_poll_iterations_ = 0;
    uint32_t max_poll_iterations_ = 0;

    // Advance timespec by ns nanoseconds (handles tv_nsec overflow)
    static void advance_timespec(struct timespec& ts, long ns);
};

} // namespace ads1299
