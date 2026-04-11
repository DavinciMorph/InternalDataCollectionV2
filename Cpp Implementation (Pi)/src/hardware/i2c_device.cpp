#include "hardware/i2c_device.hpp"

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

namespace ads1299 {

I2CDevice::I2CDevice(int bus_num) : fd_(-1) {
    char path[32];
    std::snprintf(path, sizeof(path), "/dev/i2c-%d", bus_num);
    fd_ = ::open(path, O_RDWR);
    if (fd_ < 0) {
        std::perror(path);
    }
}

I2CDevice::~I2CDevice() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

uint8_t I2CDevice::read_byte(uint8_t addr, uint8_t reg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return 0xFF;

    // Use I2C_RDWR ioctl: write register address, then read 1 byte.
    // Single ioctl with both messages — saves one syscall vs I2C_SLAVE + separate ops.
    uint8_t write_buf = reg;
    uint8_t read_buf = 0;

    struct i2c_msg msgs[2] = {
        {
            .addr = addr,
            .flags = 0,           // Write
            .len = 1,
            .buf = &write_buf,
        },
        {
            .addr = addr,
            .flags = I2C_M_RD,    // Read
            .len = 1,
            .buf = &read_buf,
        },
    };

    struct i2c_rdwr_ioctl_data data = {
        .msgs = msgs,
        .nmsgs = 2,
    };

    if (::ioctl(fd_, I2C_RDWR, &data) < 0) {
        return 0xFF;
    }

    return read_buf;
}

bool I2CDevice::write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return false;

    // Write register address + value in a single I2C message
    uint8_t buf[2] = {reg, val};

    struct i2c_msg msg = {
        .addr = addr,
        .flags = 0,     // Write
        .len = 2,
        .buf = buf,
    };

    struct i2c_rdwr_ioctl_data data = {
        .msgs = &msg,
        .nmsgs = 1,
    };

    return ::ioctl(fd_, I2C_RDWR, &data) >= 0;
}

} // namespace ads1299
