#pragma once

#include <cstdint>
#include <mutex>

namespace ads1299 {

// Linux I2C ioctl wrapper using I2C_RDWR (saves one syscall per operation
// vs I2C_SLAVE + separate read/write — includes slave address per message).
// Thread-safe: mutex around all operations (single I2C bus shared by DRDY + START).
// During acquisition, only DRDY reads happen (no START writes) — no contention in hot path.
class I2CDevice {
public:
    explicit I2CDevice(int bus_num);
    ~I2CDevice();

    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;

    // Read a single byte from register at slave address
    uint8_t read_byte(uint8_t addr, uint8_t reg);

    // Write a single byte to register at slave address
    bool write_byte(uint8_t addr, uint8_t reg, uint8_t val);

    int fd() const { return fd_; }
    bool is_open() const { return fd_ >= 0; }

    // Access mutex for external callers that need to batch operations
    std::mutex& mutex() { return mutex_; }

private:
    int fd_;
    std::mutex mutex_;
};

} // namespace ads1299
