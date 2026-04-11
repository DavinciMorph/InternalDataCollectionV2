#include "ads1299/spi_device.hpp"
#include "hardware/spi_bus.hpp"
#include "hardware/tca9534.hpp"

#include <cstdio>
#include <cstring>
#include <time.h>

namespace ads1299 {

ADS1299Device::ADS1299Device(SPIBus& spi, const PortConfig& config,
                             TCA9534Pin& drdy_pin, TCA9534Pin& start_pin)
    : spi_(spi), config_(config), drdy_(drdy_pin), start_(start_pin)
{
    std::memset(raw_buf_, 0, sizeof(raw_buf_));
}

bool ADS1299Device::send_command(Cmd cmd) {
    return spi_.send_command(static_cast<uint8_t>(cmd));
}

bool ADS1299Device::send_command(uint8_t cmd) {
    return spi_.send_command(cmd);
}

bool ADS1299Device::read_registers(Reg start, uint8_t count, uint8_t* out) {
    // Command: 0x20 | reg, count-1, then read count bytes
    const int packet_len = 2 + count;
    uint8_t tx[32];
    uint8_t rx[32];
    std::memset(tx, 0, sizeof(tx));
    std::memset(rx, 0, sizeof(rx));

    tx[0] = 0x20 | (static_cast<uint8_t>(start) & 0x1F);
    tx[1] = (count - 1) & 0x1F;

    if (!spi_.transfer(tx, rx, packet_len)) return false;

    // 2us delay after register access
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t start_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
        if (now - start_ns >= 2000) break;
    }

    // Response starts at byte 2
    std::memcpy(out, rx + 2, count);
    return true;
}

bool ADS1299Device::write_registers(Reg start, const uint8_t* values, uint8_t count) {
    // Command: 0x40 | reg, count-1, then write values
    const int packet_len = 2 + count;
    uint8_t tx[32];
    uint8_t rx[32];
    std::memset(tx, 0, sizeof(tx));
    std::memset(rx, 0, sizeof(rx));

    tx[0] = 0x40 | (static_cast<uint8_t>(start) & 0x1F);
    tx[1] = (count - 1) & 0x1F;
    std::memcpy(tx + 2, values, count);

    if (!spi_.transfer(tx, rx, packet_len)) return false;

    // 2us delay after register access
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t start_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
        if (now - start_ns >= 2000) break;
    }

    return true;
}

uint8_t ADS1299Device::read_register(Reg reg) {
    uint8_t val = 0;
    read_registers(reg, 1, &val);
    return val;
}

bool ADS1299Device::write_register(Reg reg, uint8_t value) {
    return write_registers(reg, &value, 1);
}

uint8_t* ADS1299Device::read_raw() {
    int len = config_.num_devices * BYTES_PER_DEVICE;
    if (spi_.read_raw(raw_buf_, len)) {
        return raw_buf_;
    }
    return nullptr;
}

bool ADS1299Device::read_raw(uint8_t* buf, size_t len) {
    return spi_.read_raw(buf, len);
}

void ADS1299Device::parse_raw(const uint8_t* raw, int num_devices, PortData& out) {
    out.num_devices = num_devices;

    for (int dev = 0; dev < num_devices; ++dev) {
        const int offset = dev * BYTES_PER_DEVICE;

        // Status bytes (3 bytes per device)
        out.status_bytes[dev][0] = raw[offset];
        out.status_bytes[dev][1] = raw[offset + 1];
        out.status_bytes[dev][2] = raw[offset + 2];

        // Channel data: 8 channels, 3 bytes each, big-endian signed 24-bit
        const uint8_t* ch_data = raw + offset + 3;
        for (int ch = 0; ch < CHANNELS_PER_DEVICE; ++ch) {
            const uint8_t* p = ch_data + ch * 3;
            // Branchless sign extension: shift left 8 into int32, then arithmetic right shift 8
            // Compiles to LSL+ASR on ARM64 — 2 instructions, zero branches
            uint32_t raw24 = (static_cast<uint32_t>(p[0]) << 16) |
                             (static_cast<uint32_t>(p[1]) << 8) |
                              static_cast<uint32_t>(p[2]);
            out.channel_values[dev][ch] = static_cast<int32_t>(raw24 << 8) >> 8;
        }
    }
}

bool ADS1299Device::read_data(PortData& out) {
    int len = config_.num_devices * BYTES_PER_DEVICE;
    if (!spi_.read_raw(raw_buf_, len)) return false;
    parse_raw(raw_buf_, config_.num_devices, out);
    return true;
}

bool ADS1299Device::is_drdy() {
    // DRDY is active LOW
    return !drdy_.read();
}

bool ADS1299Device::wait_for_drdy(double timeout_sec) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double start = ts.tv_sec + ts.tv_nsec * 1e-9;

    while (!is_drdy()) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double now = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (now - start > timeout_sec) return false;
    }
    return true;
}

void ADS1299Device::start_high() {
    start_.set_high();
}

void ADS1299Device::start_low() {
    start_.set_low();
}

void ADS1299Device::flush_spi() {
    int frame_size = config_.num_devices * BYTES_PER_DEVICE;
    uint8_t dummy[PortData::MAX_RAW_BYTES];
    spi_.read_raw(dummy, frame_size);

    // 1ms settle
    struct timespec ts = {0, 1'000'000};
    nanosleep(&ts, nullptr);
}

} // namespace ads1299
