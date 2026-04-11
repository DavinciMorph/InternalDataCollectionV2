#include "acquisition/drdy_poller.hpp"
#include "hardware/i2c_device.hpp"
#include "ads1299/registers.hpp"

#include <time.h>
#include <errno.h>

namespace ads1299 {

// Compare timespecs: true if a <= b
static inline bool timespec_le(const struct timespec& a, const struct timespec& b) {
    return (a.tv_sec < b.tv_sec) ||
           (a.tv_sec == b.tv_sec && a.tv_nsec <= b.tv_nsec);
}

void DRDYPoller::advance_timespec(struct timespec& ts, long ns) {
    ts.tv_nsec += ns;
    if (ts.tv_nsec >= 1'000'000'000L) {
        ts.tv_sec += ts.tv_nsec / 1'000'000'000L;
        ts.tv_nsec = ts.tv_nsec % 1'000'000'000L;
    }
}

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

    // Compute deadline using integer timespec (no floating-point precision loss)
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    long timeout_ns = static_cast<long>(timeout_sec * 1e9);
    advance_timespec(deadline, timeout_ns);

    uint32_t iterations = 0;

    // First I2C read is immediate (no sleep before first check)
    uint8_t val = i2c_.read_byte(addr_, TCA9534_INPUT_PORT);
    ++iterations;
    if ((val & mask_) == 0) {
        last_poll_iterations_ = iterations;
        if (iterations > max_poll_iterations_) max_poll_iterations_ = iterations;
        return true;
    }

    // Set up fixed-interval polling grid at 2500 Hz (400µs).
    // Absolute-time grid absorbs variable I2C read duration into the sleep window,
    // so poll rate stays locked regardless of I2C wire time jitter.
    struct timespec next_poll;
    clock_gettime(CLOCK_MONOTONIC, &next_poll);
    advance_timespec(next_poll, POLL_INTERVAL_NS);

    for (;;) {
        // Sleep until the next grid point
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_poll, nullptr);
        // EINTR from signals is harmless — we just re-loop

        // Read TCA9534 input register, check all DRDY bits
        val = i2c_.read_byte(addr_, TCA9534_INPUT_PORT);
        ++iterations;
        // DRDY is active LOW: bit=0 means ready. All masked bits must be 0.
        if ((val & mask_) == 0) {
            last_poll_iterations_ = iterations;
            if (iterations > max_poll_iterations_) max_poll_iterations_ = iterations;
            return true;
        }

        // Advance grid to next interval
        advance_timespec(next_poll, POLL_INTERVAL_NS);

        // Overrun handling: if I2C read took longer than one interval,
        // skip forward until next_poll is in the future (self-correcting)
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        while (timespec_le(next_poll, now)) {
            advance_timespec(next_poll, POLL_INTERVAL_NS);
        }

        // Check timeout
        if (!timespec_le(now, deadline)) {
            last_poll_iterations_ = iterations;
            if (iterations > max_poll_iterations_) max_poll_iterations_ = iterations;
            return false;
        }
    }
}

} // namespace ads1299
