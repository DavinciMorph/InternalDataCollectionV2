#include "hardware/tca9534.hpp"
#include "hardware/i2c_device.hpp"
#include "ads1299/registers.hpp"

namespace ads1299 {

// --- TCA9534Pin ---

TCA9534Pin::TCA9534Pin(I2CDevice& i2c, uint8_t addr, uint8_t pin, bool is_input)
    : i2c_(i2c), addr_(addr), pin_(pin), pin_mask_(static_cast<uint8_t>(1u << pin))
{
    // Configure pin direction in TCA9534 config register
    uint8_t config = i2c_.read_byte(addr_, TCA9534_CONFIG_REG);
    if (is_input) {
        config |= pin_mask_;   // Set bit = input
    } else {
        config &= ~pin_mask_;  // Clear bit = output
    }
    i2c_.write_byte(addr_, TCA9534_CONFIG_REG, config);

    // Initialize output pins LOW
    if (!is_input) {
        set_low();
    }
}

bool TCA9534Pin::read() {
    uint8_t value = i2c_.read_byte(addr_, TCA9534_INPUT_PORT);
    return (value & pin_mask_) != 0;
}

void TCA9534Pin::set_high() {
    uint8_t value = i2c_.read_byte(addr_, TCA9534_OUTPUT_PORT);
    value |= pin_mask_;
    i2c_.write_byte(addr_, TCA9534_OUTPUT_PORT, value);
}

void TCA9534Pin::set_low() {
    uint8_t value = i2c_.read_byte(addr_, TCA9534_OUTPUT_PORT);
    value &= ~pin_mask_;
    i2c_.write_byte(addr_, TCA9534_OUTPUT_PORT, value);
}

// --- TCA9534MultiPin ---

TCA9534MultiPin::TCA9534MultiPin(I2CDevice& i2c) : i2c_(i2c) {}

TCA9534MultiPin::AddrMask& TCA9534MultiPin::get_entry(uint8_t addr) {
    for (auto& e : entries_) {
        if (e.addr == addr) return e;
    }
    entries_.push_back({addr, 0});
    return entries_.back();
}

void TCA9534MultiPin::add_pin(uint8_t addr, uint8_t pin) {
    auto& entry = get_entry(addr);
    entry.mask |= static_cast<uint8_t>(1u << pin);
}

void TCA9534MultiPin::set_all_high() {
    for (const auto& e : entries_) {
        uint8_t current = i2c_.read_byte(e.addr, TCA9534_OUTPUT_PORT);
        uint8_t new_val = current | e.mask;
        i2c_.write_byte(e.addr, TCA9534_OUTPUT_PORT, new_val);
    }
}

void TCA9534MultiPin::set_all_low() {
    for (const auto& e : entries_) {
        uint8_t current = i2c_.read_byte(e.addr, TCA9534_OUTPUT_PORT);
        uint8_t new_val = current & ~e.mask;
        i2c_.write_byte(e.addr, TCA9534_OUTPUT_PORT, new_val);
    }
}

} // namespace ads1299
