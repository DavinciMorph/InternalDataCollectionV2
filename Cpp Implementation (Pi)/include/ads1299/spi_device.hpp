#pragma once

#include "ads1299/registers.hpp"
#include "ads1299/types.hpp"

#include <cstdint>
#include <vector>

namespace ads1299 {

class SPIBus;
class TCA9534Pin;

// ADS1299 SPI communication class — port of Python's ADS1299_SPI.
// Wraps an SPIBus with daisy-chain awareness, DRDY/START pin control,
// and pre-allocated buffers for zero-allocation hot-path reads.
class ADS1299Device {
public:
    ADS1299Device(SPIBus& spi, const PortConfig& config,
                  TCA9534Pin& drdy_pin, TCA9534Pin& start_pin);

    // --- Command layer ---
    bool send_command(Cmd cmd);
    bool send_command(uint8_t cmd);

    // --- Register access (init-only) ---
    // Read registers from first device in chain. Returns count bytes.
    bool read_registers(Reg start, uint8_t count, uint8_t* out);
    bool write_registers(Reg start, const uint8_t* values, uint8_t count);

    // Convenience: read single register
    uint8_t read_register(Reg reg);
    bool write_register(Reg reg, uint8_t value);

    // --- Hot-path data read ---
    // Read raw bytes from all daisy-chained devices (num_devices * 27 bytes).
    // Writes directly to pre-allocated buffer. Returns pointer to internal buffer.
    uint8_t* read_raw();

    // Read raw into caller-provided buffer
    bool read_raw(uint8_t* buf, size_t len);

    // Parse raw SPI bytes into PortData struct
    static void parse_raw(const uint8_t* raw, int num_devices, PortData& out);

    // Combined read + parse
    bool read_data(PortData& out);

    // --- DRDY / START control ---
    bool is_drdy();                          // DRDY is active LOW
    bool wait_for_drdy(double timeout_sec);

    void start_high();
    void start_low();

    // --- Utility ---
    void flush_spi();

    // Accessors
    const PortConfig& config() const { return config_; }
    SPIBus& spi() { return spi_; }
    TCA9534Pin& drdy() { return drdy_; }
    TCA9534Pin& start() { return start_; }

    // Pre-allocated raw buffer
    uint8_t* raw_buffer() { return raw_buf_; }
    int raw_buffer_len() const { return config_.num_devices * BYTES_PER_DEVICE; }

private:
    SPIBus& spi_;
    PortConfig config_;
    TCA9534Pin& drdy_;
    TCA9534Pin& start_;

    // Pre-allocated buffers (no heap allocation in hot path)
    uint8_t raw_buf_[PortData::MAX_RAW_BYTES];
};

} // namespace ads1299
