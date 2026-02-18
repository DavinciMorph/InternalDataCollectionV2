#include "acquisition/drdy_poller.hpp"
#include "hardware/i2c_device.hpp"
#include "ads1299/registers.hpp"

#include <time.h>

namespace ads1299 {

DRDYPoller::DRDYPoller(I2CDevice& i2c, uint8_t expander_addr)
    : i2c_(i2c), addr_(expander_addr), mask_(0)
{}

void DRDYPoller::add_pin(uint8_t pin) {
    mask_ |= static_cast<uint8_t>(1u << pin);
}

void DRDYPoller::remove_pin(uint8_t pin) {
    mask_ &= ~static_cast<uint8_t>(1u << pin);
}

bool DRDYPoller::poll(double timeout_sec) {
    if (mask_ == 0) return false;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double deadline = ts.tv_sec + ts.tv_nsec * 1e-9 + timeout_sec;

    // No sleep/yield between polls — engine runs on isolated core 3 with SCHED_FIFO.
    // Polls at ~5-10 kHz (limited by I2C wire time ~100-200us per read).
    for (;;) {
        // Read TCA9534 input register once, check all DRDY bits
        uint8_t val = i2c_.read_byte(addr_, TCA9534_INPUT_PORT);
        // DRDY is active LOW: bit=0 means ready. All masked bits must be 0.
        if ((val & mask_) == 0) {
            return true;
        }

        clock_gettime(CLOCK_MONOTONIC, &ts);
        double now = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (now >= deadline) {
            return false;
        }
    }
}

} // namespace ads1299
