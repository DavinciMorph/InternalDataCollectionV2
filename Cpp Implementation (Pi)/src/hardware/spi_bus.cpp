#include "hardware/spi_bus.hpp"

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>

namespace ads1299 {

SPIBus::SPIBus(int bus, int device) : fd_(-1) {
    std::memset(zero_tx_, 0, sizeof(zero_tx_));
    std::memset(data_xfer_storage_, 0, sizeof(data_xfer_storage_));

    // Open /dev/spidevB.D
    char path[32];
    std::snprintf(path, sizeof(path), "/dev/spidev%d.%d", bus, device);
    fd_ = ::open(path, O_RDWR);
    if (fd_ < 0) {
        std::perror(path);
        return;
    }

    // Set SPI mode 1 (CPOL=0, CPHA=1)
    uint8_t mode = SPI_MODE_1;
    if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
        std::perror("SPI_IOC_WR_MODE");
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Set bits per word
    uint8_t bits = 8;
    if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::perror("SPI_IOC_WR_BITS_PER_WORD");
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Set max speed — do this ONCE, never change during RDATAC
    uint32_t speed = 6'000'000;
    if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Pre-initialize the spi_ioc_transfer struct for hot-path reads.
    // tx_buf points to zero_tx_ (drives MOSI with zeros).
    // rx_buf and len will be set per-call in read_raw().
    auto* xfer = reinterpret_cast<struct spi_ioc_transfer*>(data_xfer_storage_);
    std::memset(xfer, 0, sizeof(struct spi_ioc_transfer));
    xfer->tx_buf = reinterpret_cast<uintptr_t>(zero_tx_);
    xfer->speed_hz = speed;
    xfer->bits_per_word = 8;
    xfer->delay_usecs = 0;
}

SPIBus::~SPIBus() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

bool SPIBus::read_raw(uint8_t* rx_buf, size_t len) {
    if (fd_ < 0 || len > MAX_RAW_BYTES) return false;

    // Hot path: update only rx_buf and len in pre-initialized struct, then single ioctl
    auto* xfer = reinterpret_cast<struct spi_ioc_transfer*>(data_xfer_storage_);
    xfer->rx_buf = reinterpret_cast<uintptr_t>(rx_buf);
    xfer->len = static_cast<uint32_t>(len);

    return ::ioctl(fd_, SPI_IOC_MESSAGE(1), xfer) >= 0;
}

bool SPIBus::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (fd_ < 0) return false;

    // Init-only path: stack-local transfer struct
    struct spi_ioc_transfer xfer{};
    xfer.tx_buf = reinterpret_cast<uintptr_t>(tx);
    xfer.rx_buf = reinterpret_cast<uintptr_t>(rx);
    xfer.len = static_cast<uint32_t>(len);
    xfer.speed_hz = 6'000'000;
    xfer.bits_per_word = 8;
    xfer.delay_usecs = 0;

    return ::ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer) >= 0;
}

bool SPIBus::send_command(uint8_t cmd) {
    if (fd_ < 0) return false;

    uint8_t tx = cmd;
    uint8_t rx = 0;
    struct spi_ioc_transfer xfer{};
    xfer.tx_buf = reinterpret_cast<uintptr_t>(&tx);
    xfer.rx_buf = reinterpret_cast<uintptr_t>(&rx);
    xfer.len = 1;
    xfer.speed_hz = 6'000'000;
    xfer.bits_per_word = 8;
    xfer.delay_usecs = 0;

    if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer) < 0) {
        return false;
    }

    // 2us busy-spin delay (minimum 4 tCLK @ 2.048 MHz per ADS1299 datasheet)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t start_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
        if (now_ns - start_ns >= 2000) break;  // 2000 ns = 2 us
    }
    return true;
}

} // namespace ads1299
