#pragma once

#include <cstddef>
#include <cstdint>

namespace ads1299 {

// Linux SPI ioctl wrapper for ADS1299 communication.
// Pre-initializes spi_ioc_transfer struct at construction — hot path is a single ioctl.
// NEVER changes spi.max_speed_hz after init (kernel spi_setup() ioctl causes CS glitches during RDATAC).
class SPIBus {
public:
    // Open /dev/spidevB.D, configure mode 1, 6 MHz, 8 bits/word
    SPIBus(int bus, int device);
    ~SPIBus();

    SPIBus(const SPIBus&) = delete;
    SPIBus& operator=(const SPIBus&) = delete;

    // Hot path: read num_devices*27 bytes (drives MOSI with pre-allocated zeros).
    // Single ioctl call, zero allocation. Buffer must be pre-allocated by caller.
    bool read_raw(uint8_t* rx_buf, size_t len);

    // Init-only: full-duplex transfer for register reads/writes.
    // Uses stack-local spi_ioc_transfer — not optimized for hot path.
    bool transfer(const uint8_t* tx, uint8_t* rx, size_t len);

    // Send a single-byte command + 2us busy-spin delay
    bool send_command(uint8_t cmd);

    int fd() const { return fd_; }
    bool is_open() const { return fd_ >= 0; }

private:
    int fd_;

    // Pre-allocated zero TX buffer for read_raw() — drives MOSI with zeros
    // (proven better than floating MOSI in A/B testing: 14 vs 96 corrupt values)
    static constexpr size_t MAX_RAW_BYTES = 9 * 27;  // 243 bytes max (MAX_DEVICES_PER_PORT * 27)
    uint8_t zero_tx_[MAX_RAW_BYTES];

    // Pre-initialized transfer struct for hot path (read_raw)
    // Stored as raw bytes to avoid including linux headers in the header
    alignas(8) uint8_t data_xfer_storage_[64];  // sizeof(spi_ioc_transfer) = 32 on ARM64
};

} // namespace ads1299
