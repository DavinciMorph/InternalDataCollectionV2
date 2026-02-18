#pragma once

#include <array>
#include <cstdint>

namespace ads1299 {

// ADS1299 SPI command definitions (ported from Python ADS1299_Cmd)
enum class Cmd : uint8_t {
    WAKEUP  = 0x02,
    STANDBY = 0x04,
    RESET   = 0x06,
    START   = 0x08,
    STOP    = 0x0A,
    RDATAC  = 0x10,
    SDATAC  = 0x11,
    RDATA   = 0x12,
};

// ADS1299 register addresses (ported from Python ADS1299_Reg)
enum class Reg : uint8_t {
    ID         = 0x00,
    CONFIG1    = 0x01,
    CONFIG2    = 0x02,
    CONFIG3    = 0x03,
    LOFF       = 0x04,
    CH1SET     = 0x05,
    CH2SET     = 0x06,
    CH3SET     = 0x07,
    CH4SET     = 0x08,
    CH5SET     = 0x09,
    CH6SET     = 0x0A,
    CH7SET     = 0x0B,
    CH8SET     = 0x0C,
    BIAS_SENSP = 0x0D,
    BIAS_SENSN = 0x0E,
    LOFF_SENSP = 0x0F,
    LOFF_SENSN = 0x10,
    LOFF_FLIP  = 0x11,
    MISC1      = 0x15,
    CONFIG4    = 0x17,
};

// Standard register configuration for ADS1299
// TEST MODE values matching Python's ADS1299_Config defaults
struct DeviceConfig {
    uint8_t config1    = 0x96;  // Daisy-chain mode, 250 SPS, external clock
    uint8_t config2    = 0xD0;  // TEST MODE: internal test signal ON
    uint8_t config3    = 0xE0;  // Internal BIASREF, BIAS buffer off
    uint8_t config4    = 0x00;  // Continuous conversion, lead-off comp off

    uint8_t ch1set     = 0x05;  // TEST MODE: gain=1, test signal input
    uint8_t ch2set     = 0x05;
    uint8_t ch3set     = 0x05;
    uint8_t ch4set     = 0x05;
    uint8_t ch5set     = 0x05;
    uint8_t ch6set     = 0x05;
    uint8_t ch7set     = 0x05;
    uint8_t ch8set     = 0x05;

    uint8_t bias_sensp = 0x00;
    uint8_t bias_sensn = 0x00;
    uint8_t loff_sensp = 0x00;
    uint8_t loff_sensn = 0x00;
    uint8_t loff_flip  = 0x00;

    uint8_t misc1      = 0x20;

    std::array<uint8_t, 8> get_channel_settings() const {
        return {ch1set, ch2set, ch3set, ch4set,
                ch5set, ch6set, ch7set, ch8set};
    }
};

// TCA9534 register addresses
constexpr uint8_t TCA9534_INPUT_PORT   = 0x00;
constexpr uint8_t TCA9534_OUTPUT_PORT  = 0x01;
constexpr uint8_t TCA9534_POLARITY_INV = 0x02;
constexpr uint8_t TCA9534_CONFIG_REG   = 0x03;

// I2C addresses
constexpr uint8_t TCA9534_DRDY_ADDR  = 0x20;
constexpr uint8_t TCA9534_START_ADDR = 0x21;

// I2C bus number (bit-banged on GPIO22/23)
constexpr int I2C_BUS = 6;

// SPI configuration
constexpr uint8_t SPI_MODE     = 0x01;  // Mode 1: CPOL=0, CPHA=1
constexpr uint32_t SPI_SPEED_HZ = 6'000'000;  // 6 MHz

// ADS1299 expected device ID
constexpr uint8_t ADS1299_DEVICE_ID = 0x3E;

// Bytes per device per sample: 3 status + 8 channels * 3 bytes = 27
constexpr int BYTES_PER_DEVICE = 27;

} // namespace ads1299
