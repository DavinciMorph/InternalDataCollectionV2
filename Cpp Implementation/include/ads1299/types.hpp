#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace ads1299 {

// Maximum system dimensions
constexpr int MAX_PORTS          = 7;
constexpr int MAX_DEVICES_PER_PORT = 9;
constexpr int CHANNELS_PER_DEVICE = 8;
constexpr int MAX_TOTAL_CHANNELS = MAX_PORTS * MAX_DEVICES_PER_PORT * CHANNELS_PER_DEVICE;  // 504

// Configuration for a single SPI port (maps to one daisy chain)
struct PortConfig {
    int bus_num;
    int device_num;
    char port_name[16];
    int num_devices;          // Number of daisy-chained ADS1299s
    uint8_t drdy_i2c_addr;
    uint8_t drdy_pin;
    uint8_t start_i2c_addr;
    uint8_t start_pin;
};

// Group of ports sharing the same physical SPI bus
struct SPIBusGroup {
    int bus_num;
    std::vector<int> port_indices;  // Indices into the global ports array
};

// A single complete sample from all ports
// Must be trivially copyable for the SPSC ring buffer
struct alignas(8) Sample {
    double   timestamp;                                 // 8 bytes
    uint32_t sample_number;                             // 4 bytes
    uint16_t num_channels;                              // 2 bytes
    bool     valid;                                     // 1 byte
    uint8_t  _pad;                                      // 1 byte
    std::array<int32_t, MAX_TOTAL_CHANNELS> channels;   // 2016 bytes
    // Total: ~2032 bytes
};
static_assert(sizeof(Sample) <= 2048, "Sample too large");

// Per-port raw data and parsed results
struct PortData {
    static constexpr int MAX_RAW_BYTES = MAX_DEVICES_PER_PORT * 27;  // 243

    uint8_t raw[MAX_RAW_BYTES];
    int     raw_len;

    // Parsed results per device
    uint8_t status_bytes[MAX_DEVICES_PER_PORT][3];
    int32_t channel_values[MAX_DEVICES_PER_PORT][CHANNELS_PER_DEVICE];
    int     num_devices;
};

// Hardware mapping: (bus, device) -> output number -> TCA9534 pin number
// Ported from Python's get_bus_config()
//
// Output Port Mapping:
//   OUT0 -> SPI0.CE0 (0,0), OUT1 -> SPI0.CE1 (0,1)
//   OUT2 -> SPI3.CE0 (3,0), OUT3 -> SPI3.CE1 (3,1)
//   OUT4 -> SPI4.CE0 (4,0), OUT5 -> SPI4.CE1 (4,1)
//   OUT6 -> SPI5.CE0 (5,0)
//
// TCA9534 Pin Mapping (same for DRDY @ 0x20 and START @ 0x21):
//   P0=OUT0, P1=OUT6, P2=OUT5, P3=OUT4, P4=OUT3, P5=OUT2, P6=OUT1
inline int get_output_number(int bus, int device) {
    // Map (bus, device) -> output number
    if (bus == 0 && device == 0) return 0;
    if (bus == 0 && device == 1) return 1;
    if (bus == 3 && device == 0) return 2;
    if (bus == 3 && device == 1) return 3;
    if (bus == 4 && device == 0) return 4;
    if (bus == 4 && device == 1) return 5;
    if (bus == 5 && device == 0) return 6;
    return -1;  // Invalid
}

inline int get_tca9534_pin(int output_num) {
    // Map output number -> TCA9534 pin number
    constexpr int output_to_pin[] = {0, 6, 5, 4, 3, 2, 1};
    if (output_num < 0 || output_num > 6) return -1;
    return output_to_pin[output_num];
}

// Build a PortConfig from bus/device/name/num_daisy
inline bool make_port_config(int bus, int device, const char* name, int num_daisy,
                             PortConfig& out) {
    int output = get_output_number(bus, device);
    if (output < 0) return false;
    int pin = get_tca9534_pin(output);
    if (pin < 0) return false;

    out.bus_num = bus;
    out.device_num = device;
    out.num_devices = num_daisy;
    out.drdy_i2c_addr  = 0x20;
    out.drdy_pin       = static_cast<uint8_t>(pin);
    out.start_i2c_addr = 0x21;
    out.start_pin      = static_cast<uint8_t>(pin);

    // Copy name safely
    int i = 0;
    while (name[i] && i < 15) {
        out.port_name[i] = name[i];
        ++i;
    }
    out.port_name[i] = '\0';

    return true;
}

// Default 7-port configuration: 41 devices across 7 ports
struct DefaultPortConfigs {
    PortConfig ports[MAX_PORTS];
    int count;

    DefaultPortConfigs() : count(0) {
        struct Entry { int bus; int dev; const char* name; int num_daisy; };
        const Entry defaults[] = {
            {0, 0, "Port1", 9},
            {0, 1, "Port2", 7},
            {3, 0, "Port3", 5},
            {3, 1, "Port4", 4},
            {4, 0, "Port5", 4},
            {4, 1, "Port6", 5},
            {5, 0, "Port7", 7},
        };

        for (const auto& e : defaults) {
            if (make_port_config(e.bus, e.dev, e.name, e.num_daisy, ports[count])) {
                ++count;
            }
        }
    }
};

// Group ports by physical SPI bus for parallel reading
inline std::vector<SPIBusGroup> group_ports_by_bus(const PortConfig* ports, int num_ports) {
    std::vector<SPIBusGroup> groups;
    for (int i = 0; i < num_ports; ++i) {
        bool found = false;
        for (auto& g : groups) {
            if (g.bus_num == ports[i].bus_num) {
                g.port_indices.push_back(i);
                found = true;
                break;
            }
        }
        if (!found) {
            SPIBusGroup g;
            g.bus_num = ports[i].bus_num;
            g.port_indices.push_back(i);
            groups.push_back(g);
        }
    }
    return groups;
}

} // namespace ads1299
