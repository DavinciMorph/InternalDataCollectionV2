#pragma once

#include "ads1299/registers.hpp"
#include "ads1299/types.hpp"

#include <csignal>
#include <cstdint>
#include <vector>

namespace ads1299 {

class ADS1299Device;
class I2CDevice;

// --- Impedance check configuration ---

// LOFF register (0x04) field definitions (ADS1299 datasheet Table 16)
//
// Bits [7:5]: COMP_TH threshold (positive side)
//   000 = 95%,  001 = 92.5%,  010 = 90%,  011 = 87.5%
//   100 = 85%,  101 = 80%,    110 = 75%,  111 = 70%
//
// Bit  [4]:   reserved (0)
//
// Bits [3:2]: ILEAD_OFF — excitation current magnitude
//   00 = 6 nA,  01 = 24 nA,  10 = 6 uA,  11 = 24 uA
//
// Bit  [1]:   FLEAD_OFF — excitation frequency
//   0 = AC at fDR/4 (62.5 Hz at 250 SPS)
//   1 = DC lead-off detection
//
// Bit  [0]:   COMP_POL — comparator polarity (0 = default)

enum class LeadOffCurrent : uint8_t {
    ILEAD_6nA  = 0x00,  // bits [3:2] = 00
    ILEAD_24nA = 0x04,  // bits [3:2] = 01
    ILEAD_6uA  = 0x08,  // bits [3:2] = 10
    ILEAD_24uA = 0x0C,  // bits [3:2] = 11
};

// Returns the excitation current in nanoamps for the given setting
constexpr double excitation_current_nA(LeadOffCurrent ilead) {
    switch (ilead) {
        case LeadOffCurrent::ILEAD_6nA:  return 6.0;
        case LeadOffCurrent::ILEAD_24nA: return 24.0;
        case LeadOffCurrent::ILEAD_6uA:  return 6000.0;
        case LeadOffCurrent::ILEAD_24uA: return 24000.0;
        default:                          return 6.0;
    }
}

struct ImpedanceConfig {
    LeadOffCurrent excitation = LeadOffCurrent::ILEAD_6uA;

    // Comparator threshold (LOFF[7:5]): 95% = 0x00 (most sensitive)
    uint8_t comp_th = 0x00;

    // Use AC excitation at fDR/4 (62.5 Hz at 250 SPS). FLEAD_OFF=0.
    bool ac_mode = true;

    // Number of samples to collect for impedance calculation.
    // At 250 SPS, 250 samples = 1 second (4 full cycles of 62.5 Hz).
    int num_samples = 250;

    // Enable P-side lead-off for all 8 channels
    uint8_t loff_sensp = 0xFF;

    // Enable N-side lead-off for all 8 channels
    uint8_t loff_sensn = 0xFF;

    // Build the LOFF register value from config fields
    [[nodiscard]] constexpr uint8_t loff_register() const {
        uint8_t val = comp_th & 0xE0;                        // bits [7:5]
        val |= static_cast<uint8_t>(excitation) & 0x0C;     // bits [3:2]
        if (!ac_mode) val |= 0x02;                           // bit [1] = FLEAD_OFF
        return val;
    }

    // CONFIG4 register value: PD_LOFF_COMP=1 (bit 1) enables comparators
    [[nodiscard]] static constexpr uint8_t config4_loff() {
        return 0x02;  // Bit 1 = PD_LOFF_COMP
    }
};

// --- Impedance result structures ---

enum class ImpedanceStatus : uint8_t {
    EXCELLENT,  // < 5 kOhm
    GOOD,       // 5-20 kOhm
    MARGINAL,   // 20-50 kOhm
    POOR,       // 50-100 kOhm
    BAD,        // > 100 kOhm or lead-off detected
};

constexpr const char* impedance_status_str(ImpedanceStatus s) {
    switch (s) {
        case ImpedanceStatus::EXCELLENT: return "excellent";
        case ImpedanceStatus::GOOD:      return "good";
        case ImpedanceStatus::MARGINAL:  return "marginal";
        case ImpedanceStatus::POOR:      return "poor";
        case ImpedanceStatus::BAD:       return "bad";
        default:                          return "unknown";
    }
}

constexpr ImpedanceStatus classify_impedance(double impedance_kohms, bool loff_detected) {
    if (loff_detected)        return ImpedanceStatus::BAD;
    if (impedance_kohms < 5)  return ImpedanceStatus::EXCELLENT;
    if (impedance_kohms < 20) return ImpedanceStatus::GOOD;
    if (impedance_kohms < 50) return ImpedanceStatus::MARGINAL;
    if (impedance_kohms < 100) return ImpedanceStatus::POOR;
    return ImpedanceStatus::BAD;
}

struct ChannelImpedance {
    int    channel;          // 1-indexed
    double impedance_ohms;
    double impedance_kohms;
    double voltage_rms_uv;
    bool   loff_p;           // P-side lead-off detected (comparator)
    bool   loff_n;           // N-side lead-off detected (comparator)
    ImpedanceStatus status;
};

struct DeviceImpedance {
    int device;              // 1-indexed
    ChannelImpedance channels[CHANNELS_PER_DEVICE];
};

struct PortImpedance {
    char name[16];
    int  num_devices;
    DeviceImpedance devices[MAX_DEVICES_PER_PORT];
};

struct ImpedanceSummary {
    int total_channels;
    int excellent;
    int good;
    int marginal;
    int poor;
    int bad;
};

// --- Impedance check runner ---

class ImpedanceCheck {
public:
    // Run a single impedance check on all active devices.
    // Configures LOFF, collects samples, computes, prints JSON, restores config, returns.
    static bool run(
        std::vector<ADS1299Device*>& devices,
        I2CDevice& i2c,
        const DeviceConfig* port_configs,
        int num_ports,
        const ImpedanceConfig& icfg,
        const volatile sig_atomic_t& running);

    // Run continuous impedance checking until Ctrl+C.
    // Configures LOFF once, then loops: collect → compute → print JSON → repeat.
    // Each JSON result is flushed to stdout immediately for the client to consume.
    // On exit (Ctrl+C), restores original configuration.
    static bool run_continuous(
        std::vector<ADS1299Device*>& devices,
        I2CDevice& i2c,
        const DeviceConfig* port_configs,
        int num_ports,
        const ImpedanceConfig& icfg,
        const volatile sig_atomic_t& running);

private:
    // Stop all devices and put them in SDATAC mode
    static void stop_all(std::vector<ADS1299Device*>& devices);

    // Configure LOFF registers on all devices for impedance measurement
    static bool configure_loff(
        std::vector<ADS1299Device*>& devices,
        const ImpedanceConfig& icfg);

    // Start conversions on all devices (RDATAC + START) with verify + recovery.
    // Returns number of ports producing valid data.
    static int start_and_verify(
        std::vector<ADS1299Device*>& devices,
        I2CDevice& i2c,
        const volatile sig_atomic_t& running);

    // Verify a single port is producing valid data.
    // Reads num_samples and checks status bytes. Returns count of valid samples.
    static int verify_port(ADS1299Device& dev, int num_samples);

    // Tier 1 recovery for a single port: RDATAC cycling.
    static bool recover_port(ADS1299Device& dev, int max_attempts = 8);

    // Collect raw samples from all ports.
    // data_out[port][sample][device][channel] stored as flat int32 arrays.
    // Returns number of samples actually collected.
    static int collect_samples(
        std::vector<ADS1299Device*>& devices,
        int num_samples,
        std::vector<std::vector<PortData>>& data_out,
        const volatile sig_atomic_t& running);

    // Read LOFF_STATP/N registers (SDATAC mode, first device per port only)
    static void read_loff_status(
        std::vector<ADS1299Device*>& devices,
        uint8_t* statp_out,   // array of num_ports bytes
        uint8_t* statn_out);

    // Compute RMS amplitude at the excitation frequency from collected samples.
    // Uses single-bin DFT at fDR/4 = 62.5 Hz (for 250 SPS).
    // Returns RMS magnitude in ADC code units (caller converts to voltage).
    static double compute_rms_at_excitation(
        const int32_t* samples,  // array of num_samples ADC code values
        int num_samples,
        double fs_hz);           // sample rate (for documentation)

    // Restore original configuration on all devices
    static bool restore_config(
        std::vector<ADS1299Device*>& devices,
        const DeviceConfig* port_configs,
        int num_ports);

    // Print results as JSON to stdout
    static void print_json(
        const PortImpedance* port_results,
        int num_ports,
        const ImpedanceSummary& summary,
        const ImpedanceConfig& icfg,
        int actual_samples,
        int cycle = 0);

    // Compute impedance for all channels from collected data.
    // Populates port_results and summary.
    static void compute_impedance(
        std::vector<ADS1299Device*>& devices,
        const DeviceConfig* port_configs,
        int num_ports,
        const std::vector<std::vector<PortData>>& collected_data,
        int actual_samples,
        const ImpedanceConfig& icfg,
        const uint8_t* loff_statp,
        const uint8_t* loff_statn,
        PortImpedance* port_results,
        ImpedanceSummary& summary);
};

} // namespace ads1299
