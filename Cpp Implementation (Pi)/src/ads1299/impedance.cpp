#include "ads1299/impedance.hpp"
#include "ads1299/controller.hpp"
#include "ads1299/spi_device.hpp"
#include "hardware/i2c_device.hpp"
#include "hardware/tca9534.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <time.h>
#include <unistd.h>

namespace ads1299 {

// --- Helpers ---

static void sleep_ms(double ms) {
    if (ms <= 0) return;
    double sec = ms / 1000.0;
    struct timespec ts;
    ts.tv_sec  = static_cast<time_t>(sec);
    ts.tv_nsec = static_cast<long>((sec - ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
}

// ADS1299 voltage conversion: ADC code magnitude -> microvolts
// V = (code / 2^23) * VREF / gain
// With VREF = 4.5V, gain = 24: LSB = 4.5 / (2^23 * 24) = 22.35 nV
static constexpr double VREF_V = 4.5;

static double code_to_uv(double code_magnitude, int gain) {
    double v = (code_magnitude / 8388608.0) * VREF_V / static_cast<double>(gain);
    return v * 1.0e6;  // convert to microvolts
}

// --- ImpedanceCheck ---

void ImpedanceCheck::stop_all(std::vector<ADS1299Device*>& devices) {
    // De-assert START on all ports
    for (auto* dev : devices) {
        dev->start_low();
    }
    sleep_ms(20);

    // STOP command
    for (auto* dev : devices) {
        dev->send_command(Cmd::STOP);
    }
    sleep_ms(10);

    // SDATAC multiple times per port for reliability across daisy chains
    for (int rep = 0; rep < 10; ++rep) {
        for (auto* dev : devices) {
            dev->send_command(Cmd::SDATAC);
        }
        sleep_ms(10);
    }
    sleep_ms(50);

    // Flush SPI shift registers
    for (int rep = 0; rep < 3; ++rep) {
        for (auto* dev : devices) {
            dev->flush_spi();
        }
        sleep_ms(5);
    }
}

bool ImpedanceCheck::configure_loff(
    std::vector<ADS1299Device*>& devices,
    const ImpedanceConfig& icfg)
{
    uint8_t loff_val = icfg.loff_register();
    uint8_t config4_val = ImpedanceConfig::config4_loff();

    // Configure all ports for impedance measurement (quiet)

    for (auto* dev : devices) {
        // All devices in daisy chain receive WREG commands simultaneously
        // (parallel MOSI topology).

        // Disable SRB1 before enabling lead-off excitation.
        // SRB1 (MISC1 bit 5) connects all N-inputs to the internal SRB1 bus.
        // With SRB1 enabled + LOFF_SENSN=0xFF, all N-side current sinks share
        // the same node — disable for independent per-channel measurement.
        dev->write_register(Reg::MISC1, 0x00);
        sleep_ms(10);

        // Set all channels to gain=1 (CHnSET=0x00: normal input, PGA gain=1).
        // At gain=24 (normal acquisition), PGA input range is only ±187.5mV.
        // With 6µA excitation, impedances > 31kΩ produce > 187.5mV → PGA saturates.
        // Gain=1 gives ±4.5V range, allowing measurement up to ~750kΩ at 6µA.
        for (int ch = 0; ch < 8; ++ch) {
            Reg reg = static_cast<Reg>(static_cast<uint8_t>(Reg::CH1SET) + ch);
            dev->write_register(reg, 0x00);  // gain=1, normal input
            sleep_ms(2);
        }
        sleep_ms(10);

        // Write LOFF register -- excitation current and frequency
        dev->write_register(Reg::LOFF, loff_val);
        sleep_ms(10);

        // Write LOFF_SENSP -- enable P-side excitation per channel
        dev->write_register(Reg::LOFF_SENSP, icfg.loff_sensp);
        sleep_ms(10);

        // Write LOFF_SENSN -- enable N-side excitation per channel
        dev->write_register(Reg::LOFF_SENSN, icfg.loff_sensn);
        sleep_ms(10);

        // Write CONFIG4 -- enable lead-off comparators
        dev->write_register(Reg::CONFIG4, config4_val);
        sleep_ms(10);

        // Verify on first device in chain (RREG only reaches first)
        uint8_t readback = dev->read_register(Reg::LOFF);
        if (readback != loff_val) {
            std::fprintf(stderr, "    %s: LOFF verify FAIL (wrote 0x%02X, read 0x%02X)\n",
                        dev->config().port_name, loff_val, readback);
        }

        uint8_t c4_readback = dev->read_register(Reg::CONFIG4);
        if (c4_readback != config4_val) {
            std::fprintf(stderr, "    %s: CONFIG4 verify FAIL (wrote 0x%02X, read 0x%02X)\n",
                        dev->config().port_name, config4_val, c4_readback);
        }

        uint8_t misc1_readback = dev->read_register(Reg::MISC1);
        if (misc1_readback != 0x00) {
            std::fprintf(stderr, "    %s: MISC1 verify FAIL (wrote 0x00, read 0x%02X)\n",
                        dev->config().port_name, misc1_readback);
        }
    }

    return true;
}

int ImpedanceCheck::verify_port(ADS1299Device& dev, int num_samples) {
    int valid = 0;
    for (int i = 0; i < num_samples; ++i) {
        if (!dev.wait_for_drdy(0.02)) continue;
        PortData pd;
        if (!dev.read_data(pd)) continue;
        // Status byte should have 0xC0 prefix (bits 7:6 = 11) for valid data
        if ((pd.status_bytes[0][0] & 0xC0) == 0xC0) {
            valid++;
        }
    }
    return valid;
}

bool ImpedanceCheck::recover_port(ADS1299Device& dev, int max_attempts) {
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        // STOP → SDATAC × 10 → flush → RDATAC × 3 → START
        dev.send_command(Cmd::STOP);
        sleep_ms(10);

        for (int i = 0; i < 10; ++i) {
            dev.send_command(Cmd::SDATAC);
            sleep_ms(10);
        }

        dev.flush_spi();
        sleep_ms(20);

        for (int i = 0; i < 3; ++i) {
            dev.send_command(Cmd::RDATAC);
            sleep_ms(10);
        }

        dev.start_high();
        sleep_ms(100);

        int valid = verify_port(dev, 10);
        if (valid >= 5) {
            std::fprintf(stderr, "    %s: recovered on attempt %d\n",
                        dev.config().port_name, attempt + 1);
            return true;
        }
    }
    return false;
}

int ImpedanceCheck::start_and_verify(
    std::vector<ADS1299Device*>& devices,
    I2CDevice& i2c,
    const volatile sig_atomic_t& running)
{
    int num_ports = static_cast<int>(devices.size());

    // Ensure clean SDATAC state before RDATAC.
    // RREG calls in configure_loff() may leave SPI shift register in
    // a corrupted state on Pi .175 (known ~97% RREG failure rate).
    // Extra SDATAC + flush clears any residual state.
    for (int rep = 0; rep < 5; ++rep) {
        for (auto* dev : devices) {
            dev->send_command(Cmd::SDATAC);
        }
        sleep_ms(10);
    }
    for (int rep = 0; rep < 3; ++rep) {
        for (auto* dev : devices) {
            dev->flush_spi();
        }
        sleep_ms(5);
    }

    // Send RDATAC to all devices (multiple times for reliability)
    for (int rep = 0; rep < 3; ++rep) {
        for (auto* dev : devices) {
            dev->send_command(Cmd::RDATAC);
        }
        sleep_ms(20);
    }
    sleep_ms(50);

    // Assert START on all ports atomically via TCA9534MultiPin
    {
        TCA9534MultiPin start_controller(i2c);
        for (auto* dev : devices) {
            start_controller.add_pin(dev->config().start_i2c_addr,
                                     dev->config().start_pin);
        }
        start_controller.set_all_high();
    }
    sleep_ms(100);

    // Verify each port and recover failing ones (RDATAC timing race)
    int healthy = 0;
    for (int p = 0; p < num_ports && running; ++p) {
        int valid = verify_port(*devices[p], 10);
        if (valid >= 5) {
            healthy++;
            continue;
        }

        std::fprintf(stderr, "    %s: %d/10 valid — recovering...\n",
                    devices[p]->config().port_name, valid);

        if (recover_port(*devices[p], 8)) {
            healthy++;
        } else {
            std::fprintf(stderr, "    %s: FAILED after 8 recovery attempts\n",
                        devices[p]->config().port_name);
        }
    }

    std::fprintf(stderr, "  %d/%d ports producing valid data\n", healthy, num_ports);
    return healthy;
}

int ImpedanceCheck::collect_samples(
    std::vector<ADS1299Device*>& devices,
    int num_samples,
    std::vector<std::vector<PortData>>& data_out,
    const volatile sig_atomic_t& running)
{
    int num_ports = static_cast<int>(devices.size());

    // Pre-allocate storage
    data_out.resize(num_ports);
    for (int p = 0; p < num_ports; ++p) {
        data_out[p].resize(num_samples);
    }

    // Use first port with most devices as DRDY reference
    ADS1299Device* ref_port = devices[0];
    for (int p = 1; p < num_ports; ++p) {
        if (devices[p]->config().num_devices > ref_port->config().num_devices) {
            ref_port = devices[p];
        }
    }

    int collected = 0;
    int timeouts = 0;

    for (int s = 0; s < num_samples && running; ++s) {
        // Wait for DRDY (20ms timeout -- 250 Hz = 4ms period, 5x margin)
        if (!ref_port->wait_for_drdy(0.02)) {
            timeouts++;
            if (timeouts > num_samples / 2) {
                std::fprintf(stderr, "  [WARN] Excessive DRDY timeouts (%d) -- aborting collection\n",
                            timeouts);
                break;
            }
            continue;
        }

        // Read all ports (check return values)
        bool all_ok = true;
        for (int p = 0; p < num_ports; ++p) {
            if (!devices[p]->read_data(data_out[p][collected])) {
                all_ok = false;
            }
        }
        if (!all_ok) {
            // SPI read failed for at least one port — skip this sample
            timeouts++;
            if (timeouts > num_samples / 2) {
                std::fprintf(stderr, "  [WARN] Excessive SPI read failures (%d) — aborting collection\n",
                            timeouts);
                break;
            }
            continue;
        }
        collected++;
    }

    if (timeouts > 0) {
        std::fprintf(stderr, "  DRDY timeouts during collection: %d\n", timeouts);
    }

    return collected;
}

void ImpedanceCheck::read_loff_status(
    std::vector<ADS1299Device*>& devices,
    uint8_t* statp_out,
    uint8_t* statn_out)
{
    // Must be in SDATAC mode to read registers.
    // LOFF_STATP/N are read-only registers that reflect the comparator output.
    // In daisy chain, RREG only reads from the FIRST device.
    // For devices deeper in the chain, we rely on voltage-based impedance instead.

    for (size_t p = 0; p < devices.size(); ++p) {
        // Ensure SDATAC
        for (int rep = 0; rep < 5; ++rep) {
            devices[p]->send_command(Cmd::SDATAC);
            sleep_ms(10);
        }
        sleep_ms(20);

        // Read LOFF_STATP (first device only in daisy chain)
        statp_out[p] = devices[p]->read_register(Reg::LOFF_STATP);
        sleep_ms(5);

        // Read LOFF_STATN (first device only in daisy chain)
        statn_out[p] = devices[p]->read_register(Reg::LOFF_STATN);
        sleep_ms(5);
    }
}

double ImpedanceCheck::compute_rms_at_excitation(
    const int32_t* samples,
    int num_samples,
    double fs_hz)
{
    // Single-bin DFT at the excitation frequency fDR/4.
    //
    // The ADS1299 AC lead-off excitation is at fDR/4 = Fs/4 = 62.5 Hz (at 250 SPS).
    // We compute the DFT magnitude at exactly that frequency.
    //
    // For N samples at Fs Hz, the DFT bin k corresponds to frequency f = k * Fs / N.
    // We want f = Fs/4, so k = N/4.
    //
    // DFT at bin k:
    //   X[k] = sum_{n=0}^{N-1} x[n] * exp(-j * 2*pi*k*n / N)
    //
    // For a sinusoid x[n] = A*cos(2*pi*k0*n/N + phi):
    //   |X[k0]| = A*N/2 (for 0 < k0 < N/2)
    //   Peak amplitude A = 2*|X[k]|/N
    //   RMS = A/sqrt(2) = sqrt(2)*|X[k]|/N
    //
    // Since f_target = fs/4, the phase angle per sample is pi/2 radians.
    // cos(pi/2 * n) cycles through: +1, 0, -1, 0, +1, ...
    // sin(pi/2 * n) cycles through: 0, +1, 0, -1, 0, ...
    //
    // This means we can compute the DFT without any trig calls --
    // just accumulate samples with mod-4 sign patterns.

    if (num_samples < 4) return 0.0;

    // Subtract DC mean to prevent leakage into the Fs/4 bin
    // (N=250 is not divisible by 4, so the DFT at Fs/4 has imperfect DC rejection)
    double dc_sum = 0.0;
    for (int n = 0; n < num_samples; ++n) {
        dc_sum += static_cast<double>(samples[n]);
    }
    double dc_mean = dc_sum / num_samples;

    double sum_real = 0.0;
    double sum_imag = 0.0;

    for (int n = 0; n < num_samples; ++n) {
        double x = static_cast<double>(samples[n]) - dc_mean;
        switch (n & 3) {  // n % 4, but branchless on ARM
            case 0: sum_real += x; break;  // cos(0)=+1, sin(0)=0
            case 1: sum_imag += x; break;  // cos(pi/2)=0, sin(pi/2)=+1
            case 2: sum_real -= x; break;  // cos(pi)=-1, sin(pi)=0
            case 3: sum_imag -= x; break;  // cos(3pi/2)=0, sin(3pi/2)=-1
        }
    }

    // Magnitude of single-bin DFT
    double magnitude = std::sqrt(sum_real * sum_real + sum_imag * sum_imag);

    // Convert to RMS: peak A = 2*|X|/N, RMS = A/sqrt(2) = sqrt(2)*|X|/N
    double rms_code = std::sqrt(2.0) * magnitude / static_cast<double>(num_samples);

    (void)fs_hz;  // Documented parameter, used for clarity in the API

    return rms_code;
}

bool ImpedanceCheck::restore_config(
    std::vector<ADS1299Device*>& devices,
    const DeviceConfig* port_configs,
    int num_ports)
{
    // Restoring original configuration (quiet)

    // Stop everything
    stop_all(devices);

    // Rewrite all registers for each port using the original config
    for (int p = 0; p < num_ports; ++p) {
        auto* dev = devices[p];
        const auto& cfg = port_configs[p];

        // Silently restore all registers per port

        // Write registers in the same order as write_registers_simple
        dev->write_register(Reg::CONFIG3, cfg.config3);
        sleep_ms(200);  // Reference buffer settling

        dev->write_register(Reg::CONFIG1, cfg.config1);
        sleep_ms(10);

        dev->write_register(Reg::CONFIG2, cfg.config2);
        sleep_ms(10);

        auto ch_settings = cfg.get_channel_settings();
        for (int i = 0; i < 8; ++i) {
            Reg reg = static_cast<Reg>(static_cast<uint8_t>(Reg::CH1SET) + i);
            dev->write_register(reg, ch_settings[i]);
            sleep_ms(10);
        }

        // BIAS routing registers (only written when non-zero)
        if (cfg.bias_sensp != 0x00 || cfg.bias_sensn != 0x00) {
            dev->write_register(Reg::BIAS_SENSP, cfg.bias_sensp);
            sleep_ms(10);
            dev->write_register(Reg::BIAS_SENSN, cfg.bias_sensn);
            sleep_ms(10);
        }

        // Restore LOFF registers to original values (usually all zeros)
        dev->write_register(Reg::LOFF, 0x00);
        sleep_ms(10);
        dev->write_register(Reg::LOFF_SENSP, cfg.loff_sensp);
        sleep_ms(10);
        dev->write_register(Reg::LOFF_SENSN, cfg.loff_sensn);
        sleep_ms(10);
        dev->write_register(Reg::LOFF_FLIP, cfg.loff_flip);
        sleep_ms(10);

        dev->write_register(Reg::MISC1, cfg.misc1);
        sleep_ms(10);
        dev->write_register(Reg::CONFIG4, cfg.config4);
        sleep_ms(10);

    }

    // Configuration restored
    return true;
}

void ImpedanceCheck::compute_impedance(
    std::vector<ADS1299Device*>& devices,
    const DeviceConfig* port_configs,
    int num_ports,
    const std::vector<std::vector<PortData>>& collected_data,
    int actual_samples,
    const ImpedanceConfig& icfg,
    const uint8_t* loff_statp,
    const uint8_t* loff_statn,
    PortImpedance* port_results,
    ImpedanceSummary& summary)
{
    double excitation_nA_val = excitation_current_nA(icfg.excitation);
    double excitation_A = excitation_nA_val * 1.0e-9;

    summary = {};

    for (int p = 0; p < num_ports; ++p) {
        auto* dev = devices[p];
        int n_devs = dev->config().num_devices;
        auto& port_res = port_results[p];

        std::strncpy(port_res.name, dev->config().port_name, sizeof(port_res.name) - 1);
        port_res.name[sizeof(port_res.name) - 1] = '\0';
        port_res.num_devices = n_devs;

        (void)port_configs;  // Gain is fixed at 1 during impedance measurement

        for (int d = 0; d < n_devs; ++d) {
            auto& dev_res = port_res.devices[d];
            dev_res.device = d + 1;

            for (int c = 0; c < CHANNELS_PER_DEVICE; ++c) {
                auto& ch_res = dev_res.channels[c];
                ch_res.channel = c + 1;

                // Impedance mode uses gain=1 (CHnSET=0x00 set in configure_loff)
                int gain = 1;

                int32_t channel_samples[4096];
                int n_samp = (actual_samples < 4096) ? actual_samples : 4096;

                for (int s = 0; s < n_samp; ++s) {
                    channel_samples[s] = collected_data[p][s].channel_values[d][c];
                }

                double rms_code = compute_rms_at_excitation(
                    channel_samples, n_samp, 250.0);

                // Also compute broadband RMS (DC-removed) as a robustness check.
                // If the excitation signal is absent or too weak, broadband noise
                // still correlates with impedance (Johnson-Nyquist noise).
                double dc_sum = 0.0;
                for (int s = 0; s < n_samp; ++s) {
                    dc_sum += static_cast<double>(channel_samples[s]);
                }
                double dc_mean = dc_sum / n_samp;
                double bb_sum_sq = 0.0;
                for (int s = 0; s < n_samp; ++s) {
                    double v = static_cast<double>(channel_samples[s]) - dc_mean;
                    bb_sum_sq += v * v;
                }
                double broadband_rms = std::sqrt(bb_sum_sq / n_samp);

                // Use ONLY the single-bin DFT at 62.5 Hz — NOT broadband RMS.
                // Broadband RMS includes environmental noise (60 Hz powerline, EMI)
                // which is unrelated to excitation current flow. Using broadband RMS
                // causes floating electrodes to appear as low impedance because
                // environmental noise voltage gets divided by excitation current.
                //
                // Minimum voltage threshold: if the 62.5 Hz DFT magnitude is below
                // a noise floor, the electrode is disconnected (no excitation return
                // path). Without this, Z = 0V / 6uA = 0 ohms for open circuits.
                constexpr double MIN_EXCITATION_CODE = 100.0;  // ~53 uV at gain=1

                double rms_uv = code_to_uv(rms_code, gain);
                if (rms_uv < 0) rms_uv = -rms_uv;

                ch_res.voltage_rms_uv = rms_uv;

                bool no_excitation = (rms_code < MIN_EXCITATION_CODE);

                double impedance_ohms = 0.0;
                if (!no_excitation && excitation_A > 0) {
                    double v_rms = rms_uv * 1.0e-6;
                    impedance_ohms = v_rms / excitation_A;
                } else {
                    impedance_ohms = 1.0e9;  // 1 GOhm = effectively infinite (disconnected)
                }

                ch_res.impedance_ohms = impedance_ohms;
                ch_res.impedance_kohms = impedance_ohms / 1000.0;

                // Lead-off detection: if no excitation signal or very high impedance
                if (d == 0 && !no_excitation) {
                    ch_res.loff_p = (loff_statp[p] >> c) & 0x01;
                    ch_res.loff_n = (loff_statn[p] >> c) & 0x01;
                } else {
                    ch_res.loff_p = no_excitation || (ch_res.impedance_kohms > 200.0);
                    ch_res.loff_n = no_excitation || (ch_res.impedance_kohms > 200.0);
                }

                bool loff_detected = ch_res.loff_p || ch_res.loff_n || no_excitation;
                ch_res.status = classify_impedance(ch_res.impedance_kohms, loff_detected);

                summary.total_channels++;
                switch (ch_res.status) {
                    case ImpedanceStatus::EXCELLENT: summary.excellent++; break;
                    case ImpedanceStatus::GOOD:      summary.good++; break;
                    case ImpedanceStatus::MARGINAL:   summary.marginal++; break;
                    case ImpedanceStatus::POOR:       summary.poor++; break;
                    case ImpedanceStatus::BAD:        summary.bad++; break;
                }
            }
        }
    }
}

void ImpedanceCheck::print_json(
    const PortImpedance* port_results,
    int num_ports,
    const ImpedanceSummary& summary,
    const ImpedanceConfig& icfg,
    int actual_samples,
    int cycle)
{
    // Get current timestamp
    time_t now = time(nullptr);
    struct tm tm_buf;
    localtime_r(&now, &tm_buf);
    char timestamp[32];
    std::snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

    double excitation_nA_val = excitation_current_nA(icfg.excitation);
    double excitation_freq = icfg.ac_mode ? 62.5 : 0.0;  // fDR/4 at 250 SPS

    // Print JSON to stdout (all diagnostic output goes to stderr).
    // Built manually -- no JSON library, no exceptions.

    std::printf("{\n");
    std::printf("  \"type\": \"impedance_result\",\n");
    std::printf("  \"cycle\": %d,\n", cycle);
    std::printf("  \"timestamp\": \"%s\",\n", timestamp);
    std::printf("  \"excitation_current_nA\": %.0f,\n", excitation_nA_val);
    std::printf("  \"excitation_freq_hz\": %.1f,\n", excitation_freq);
    std::printf("  \"num_samples\": %d,\n", actual_samples);
    std::printf("  \"ports\": [\n");

    for (int p = 0; p < num_ports; ++p) {
        const auto& port = port_results[p];
        std::printf("    {\n");
        std::printf("      \"name\": \"%s\",\n", port.name);
        std::printf("      \"num_devices\": %d,\n", port.num_devices);
        std::printf("      \"devices\": [\n");

        for (int d = 0; d < port.num_devices; ++d) {
            const auto& dev = port.devices[d];
            std::printf("        {\n");
            std::printf("          \"device\": %d,\n", dev.device);
            std::printf("          \"channels\": [\n");

            for (int c = 0; c < CHANNELS_PER_DEVICE; ++c) {
                const auto& ch = dev.channels[c];
                std::printf("            {\n");
                std::printf("              \"channel\": %d,\n", ch.channel);
                std::printf("              \"impedance_ohms\": %.0f,\n", ch.impedance_ohms);
                std::printf("              \"impedance_kohms\": %.2f,\n", ch.impedance_kohms);
                std::printf("              \"voltage_rms_uv\": %.3f,\n", ch.voltage_rms_uv);
                std::printf("              \"loff_p\": %s,\n", ch.loff_p ? "true" : "false");
                std::printf("              \"loff_n\": %s,\n", ch.loff_n ? "true" : "false");
                std::printf("              \"status\": \"%s\"\n",
                            impedance_status_str(ch.status));
                std::printf("            }%s\n",
                            (c + 1 < CHANNELS_PER_DEVICE) ? "," : "");
            }

            std::printf("          ]\n");
            std::printf("        }%s\n", (d + 1 < port.num_devices) ? "," : "");
        }

        std::printf("      ]\n");
        std::printf("    }%s\n", (p + 1 < num_ports) ? "," : "");
    }

    std::printf("  ],\n");

    // Summary
    std::printf("  \"summary\": {\n");
    std::printf("    \"total_channels\": %d,\n", summary.total_channels);
    std::printf("    \"excellent\": %d,\n", summary.excellent);
    std::printf("    \"good\": %d,\n", summary.good);
    std::printf("    \"marginal\": %d,\n", summary.marginal);
    std::printf("    \"poor\": %d,\n", summary.poor);
    std::printf("    \"bad\": %d\n", summary.bad);
    std::printf("  }\n");

    std::printf("}\n");
}

// --- Main impedance check entry point ---

bool ImpedanceCheck::run(
    std::vector<ADS1299Device*>& devices,
    I2CDevice& i2c,
    const DeviceConfig* port_configs,
    int num_ports,
    const ImpedanceConfig& icfg,
    const volatile sig_atomic_t& running)
{
    if (!running) return false;

    stop_all(devices);
    if (!running) return false;

    if (!configure_loff(devices, icfg)) {
        std::fprintf(stderr, "[FAIL] LOFF configuration failed\n");
        restore_config(devices, port_configs, num_ports);
        return false;
    }
    if (!running) { restore_config(devices, port_configs, num_ports); return false; }

    int healthy = start_and_verify(devices, i2c, running);
    if (healthy == 0) {
        std::fprintf(stderr, "[FAIL] No ports producing valid data\n");
        restore_config(devices, port_configs, num_ports);
        return false;
    }
    if (!running) { restore_config(devices, port_configs, num_ports); return false; }

    // Discard warmup samples
    {
        ADS1299Device* ref_port = devices[0];
        for (int p = 1; p < num_ports; ++p) {
            if (devices[p]->config().num_devices > ref_port->config().num_devices)
                ref_port = devices[p];
        }
        for (int i = 0; i < 50 && running; ++i) {
            if (ref_port->wait_for_drdy(0.02)) {
                for (int p = 0; p < num_ports; ++p) { PortData pd; devices[p]->read_data(pd); }
            }
        }
    }
    if (!running) { restore_config(devices, port_configs, num_ports); return false; }

    // Collect and compute
    std::vector<std::vector<PortData>> collected_data;
    int actual_samples = collect_samples(devices, icfg.num_samples, collected_data, running);
    if (actual_samples < 4) { restore_config(devices, port_configs, num_ports); return false; }

    stop_all(devices);

    uint8_t loff_statp[MAX_PORTS] = {};
    uint8_t loff_statn[MAX_PORTS] = {};
    read_loff_status(devices, loff_statp, loff_statn);

    PortImpedance port_results[MAX_PORTS] = {};
    ImpedanceSummary summary = {};
    compute_impedance(devices, port_configs, num_ports, collected_data, actual_samples,
                      icfg, loff_statp, loff_statn, port_results, summary);

    restore_config(devices, port_configs, num_ports);

    // Output
    print_json(port_results, num_ports, summary, icfg, actual_samples);

    return true;
}

// --- Continuous impedance check ---
// Configures LOFF once, then loops: collect → compute → print JSON → repeat.
// Conversions stay running between cycles — no reconfiguration overhead.

bool ImpedanceCheck::run_continuous(
    std::vector<ADS1299Device*>& devices,
    I2CDevice& i2c,
    const DeviceConfig* port_configs,
    int num_ports,
    const ImpedanceConfig& icfg,
    const volatile sig_atomic_t& running)
{
    std::fprintf(stderr, "Impedance check: %.0f nA AC @ %.1f Hz, %d samples/cycle\n",
                excitation_current_nA(icfg.excitation),
                icfg.ac_mode ? 62.5 : 0.0, icfg.num_samples);

    if (!running) return false;

    // One-time setup: stop, configure LOFF, start conversions, warmup
    stop_all(devices);
    if (!running) return false;

    if (!configure_loff(devices, icfg)) {
        std::fprintf(stderr, "[FAIL] LOFF configuration failed\n");
        restore_config(devices, port_configs, num_ports);
        return false;
    }
    if (!running) { restore_config(devices, port_configs, num_ports); return false; }

    int healthy = start_and_verify(devices, i2c, running);
    if (healthy == 0) {
        std::fprintf(stderr, "[FAIL] No ports producing valid data\n");
        restore_config(devices, port_configs, num_ports);
        return false;
    }
    if (!running) { stop_all(devices); restore_config(devices, port_configs, num_ports); return false; }

    // Warmup — discard 50 settling samples
    {
        ADS1299Device* ref_port = devices[0];
        for (int p = 1; p < num_ports; ++p) {
            if (devices[p]->config().num_devices > ref_port->config().num_devices)
                ref_port = devices[p];
        }
        for (int i = 0; i < 50 && running; ++i) {
            if (ref_port->wait_for_drdy(0.02)) {
                for (int p = 0; p < num_ports; ++p) {
                    PortData pd;
                    devices[p]->read_data(pd);
                }
            }
        }
    }
    if (!running) { stop_all(devices); restore_config(devices, port_configs, num_ports); return false; }

    std::fprintf(stderr, "Impedance streaming started (%d ports). Ctrl+C to stop.\n", healthy);

    // --- Measurement loop ---

    // Read LOFF status once (first device per port only — daisy chain limitation).
    // In continuous mode we don't stop between cycles, so we can't re-read these.
    // Use zeros — impedance is computed from voltage data for all devices.
    uint8_t loff_statp[MAX_PORTS] = {};
    uint8_t loff_statn[MAX_PORTS] = {};

    int cycle = 0;

    while (running) {
        cycle++;

        // Collect samples (quiet — no per-sample output)
        std::vector<std::vector<PortData>> collected_data;
        int actual_samples = collect_samples(devices, icfg.num_samples,
                                              collected_data, running);

        if (!running) break;

        if (actual_samples < 4) continue;

        // Compute impedance
        PortImpedance port_results[MAX_PORTS] = {};
        ImpedanceSummary summary = {};

        compute_impedance(devices, port_configs, num_ports,
                          collected_data, actual_samples, icfg,
                          loff_statp, loff_statn,
                          port_results, summary);

        // Print full JSON to stdout for client consumption
        print_json(port_results, num_ports, summary, icfg, actual_samples, cycle);
        std::fflush(stdout);

        // Compact one-line summary to stderr
        std::fprintf(stderr, "[cycle %d] %d ch: %d excellent, %d good, %d marginal, %d poor, %d bad\n",
                     cycle, summary.total_channels,
                     summary.excellent, summary.good, summary.marginal,
                     summary.poor, summary.bad);

        // Discard a few samples between cycles to ensure fresh data
        {
            ADS1299Device* ref_port = devices[0];
            for (int p = 1; p < num_ports; ++p) {
                if (devices[p]->config().num_devices > ref_port->config().num_devices) {
                    ref_port = devices[p];
                }
            }
            for (int i = 0; i < 5 && running; ++i) {
                if (ref_port->wait_for_drdy(0.02)) {
                    for (int p = 0; p < num_ports; ++p) {
                        PortData pd;
                        devices[p]->read_data(pd);
                    }
                }
            }
        }
    }

    // Cleanup
    stop_all(devices);
    restore_config(devices, port_configs, num_ports);
    std::fprintf(stderr, "Impedance check stopped after %d cycles.\n", cycle);
    return true;
}

} // namespace ads1299
