# ADS1299 Reference

**Last Updated:** 2026-03-07

Consolidated reference for the ADS1299 analog front-end as used in our 320-channel EEG acquisition system. Covers chip operation, register map, startup sequence, and EVM-derived application notes.

---

## 1. Device Overview

The ADS1299 is a low-noise, 8-channel, 24-bit analog front-end for biopotential measurements. Each channel has a programmable gain amplifier (PGA) and a 24-bit delta-sigma ADC. Our system uses 40 ADS1299 devices across 7 SPI ports in daisy-chain configuration.

| Parameter | Value |
|-----------|-------|
| Channels per device | 8 differential |
| Resolution | 24-bit signed (2's complement) |
| Sample rates | 250, 500, 1000, 2000, 4000, 8000, 16000 SPS |
| PGA gain options | 1, 2, 4, 6, 8, 12, 24 |
| Input noise (gain=24, 250 SPS) | ~1 uVpp typical |
| VREF internal | 4.5V (AVDD - AVSS) |
| SPI clock max | 20 MHz (chip spec), 6 MHz (our PCB limit) |
| Device ID register | 0x3E (ADS1299-8) |

### Voltage and Resolution

| Gain | Input Range | LSB (uV) |
|------|-------------|-----------|
| 1 | +/-4.5V | 0.5364 |
| 6 | +/-750mV | 0.0894 |
| 12 | +/-375mV | 0.0447 |
| 24 | +/-187.5mV | 0.02235 |

Full-scale code: +/-8,388,607 (2^23 - 1).

---

## 2. Register Map (Current System Defaults)

These are the values written by our firmware (`registers.hpp`). Some differ from chip power-on defaults by design.

| Register | Address | Our Default | Chip Default | Purpose |
|----------|---------|-------------|--------------|---------|
| CONFIG1 | 0x01 | 0x96 | 0x96 | 250 SPS, daisy-chain mode, internal osc |
| CONFIG2 | 0x02 | 0xC0 | 0xC0 | Test signals OFF |
| CONFIG3 | 0x03 | 0xE0 | 0x60 | Internal reference ON, BIAS buffer OFF |
| CH1-8SET | 0x05-0x0C | 0x60 | 0x61 | Gain=24, normal input, channel ON |
| BIAS_SENSP | 0x0D | 0x00 | 0x00 | No channels routed to BIAS |
| BIAS_SENSN | 0x0E | 0x00 | 0x00 | No channels routed to BIAS |
| MISC1 | 0x15 | 0x20 | 0x00 | SRB1 connected (common negative ref) |
| CONFIG4 | 0x17 | 0x00 | 0x00 | Continuous conversion, lead-off comp OFF |

### Key Register Overrides

- **CONFIG3 = 0xE0** (not chip default 0x60): Enables internal reference buffer (bit 7) and powers on the internal reference (bit 5). The external reference is non-functional on our board (576x worse noise when tested).
- **CHnSET = 0x60** (not chip default 0x61): Normal electrode input (mux=000) instead of shorted input (mux=001). Gain=24 (bits 6:4 = 110).
- **MISC1 = 0x20** (not chip default 0x00): SRB1 connected (bit 5), providing a common reference to all negative inputs.

### BIAS Drive Override (--bias-port N)

When `--bias-port N` is specified, port N gets:

| Register | Value | Effect |
|----------|-------|--------|
| CONFIG3 | 0xF0 | BIAS amp ON (bit 4 = PD_BIAS) |
| BIAS_SENSP | 0x7F | Channels 1-7 positive inputs to BIAS |
| BIAS_SENSN | 0x7F | Channels 1-7 negative inputs to BIAS |
| CH8SET | 0x06 | BIAS_DRP output (mux=110) |

All other ports remain at default (CONFIG3=0xE0, no BIAS routing).

---

## 3. SPI Protocol

### Mode and Speed

- SPI Mode 1 (CPOL=0, CPHA=1): data captured on falling SCLK edge
- Speed: 6 MHz (limited by PCB signal integrity on 4-8 deep daisy chains)
- Full-duplex `xfer2` with explicit zeros on MOSI (NOT `readbytes` -- see below)

### Daisy-Chain Data Format

In RDATAC mode, each DRDY cycle outputs all devices concatenated on DOUT:

```
[Dev_N status(3B) ch1(3B)..ch8(3B)] [Dev_N-1 ...] ... [Dev_1 ...]
```

Per device: 3 bytes status + 8 x 3 bytes channel = 27 bytes.
Total per read = 27 x num_devices bytes.

Deepest device data arrives first (MSB of the stream).

### Why xfer2, Not readbytes

`readbytes()` leaves MOSI floating during the transfer. On our PCB with daisy-chained devices, the floating MOSI couples noise into MISO traces, causing 7x more corruption than `xfer2()` which drives MOSI with explicit zeros. Verified via controlled A/B test: xfer2 = 14 corrupt values in 322k samples, readbytes = 96 corrupt in 318k samples.

### Critical Rule: Never Modify SPI Config During RDATAC

Writing to `spi.max_speed_hz` (or any spidev attribute) triggers a kernel `spi_setup()` ioctl that momentarily toggles the CS line. During RDATAC, this corrupts the shift register frame boundary. Verified: corruption count was speed-independent (34 at both 5 MHz and 6 MHz), proving the ioctl itself, not the speed value, was the cause.

---

## 4. Startup Sequence

Our firmware (`controller.cpp`) uses the following initialization order. This differs from the EVM user guide's recommended order in one important way: we write CONFIG3 FIRST (not CONFIG1 first) to allow the internal reference buffer to settle before configuring anything else.

### Per-Device Init (`initialize_device`)

1. **START low** -- de-assert START via I2C expander (TCA9534 at 0x21)
2. **Flush + STOP** -- clear any stale shift register state
3. **2x software RESET** -- each followed by 100ms delay
4. **SDATAC flood** -- send max(5, num_devices+2) SDATAC commands with 20ms gaps
5. **SDATAC verify** -- write LOFF=0xAA, read back, up to 100 attempts. Confirms device is in command mode
6. **ID check** -- read ID register, verify 0x3E (up to 100 attempts)
7. **Write CONFIG3** (0xE0) -- enables internal reference buffer
8. **Wait 200ms** -- reference buffer settling time
9. **Write remaining registers** -- CONFIG1, CONFIG2, CH1-8SET (with per-port BIAS config), MISC1, CONFIG4
10. **Final verification** -- read back all written registers, confirm they match

### Write-Once Variant (`initialize_device_simple`)

A simplified path that skips readback verification. Used when SPI bus signal integrity makes RREG unreliable (discovered during SPI3 debugging -- excessive WREG/RREG traffic can corrupt the ADS1299 data pipeline).

### Synchronized Start (`start_and_verify`)

1. STOP all ports
2. SDATAC x10 on all ports
3. Flush x3 on all ports
4. RDATAC x3 on all ports
5. Atomic START via TCA9534MultiPin (all ports asserted simultaneously)
6. Verify 10 samples per port (80% must have valid status bytes)

---

## 5. Recovery Procedures

### Tier 1: RDATAC Cycle (~200ms, 8 attempts)

Light recovery for transient RDATAC mode issues:
- STOP -> SDATAC x10 -> flush x3 -> RDATAC x3 -> START

### Tier 2: Software RESET + Reconfig (~500ms, 5 attempts)

Full register rewrite for persistent issues:
- Software RESET x2 -> wait 150ms -> SDATAC x5 -> rewrite all registers -> flush/RDATAC -> START

### Tier 3: Full Re-init (2 attempts)

Complete system re-initialization of all ports. Used when individual port recovery fails.

### 7/7 Hard Gate

The system refuses to start unless all ports pass health checks. Exit code 2 indicates a power cycle is needed. The `--min-ports N` flag provides an escape hatch for testing with fewer ports; dead ports are excluded from DRDY polling, bus workers, CSV, and streaming.

---

## 6. EVM Application Notes

From the ADS1299EEG-FE evaluation module user guide (SLAU443B). These notes apply to the EVM board, not directly to our custom PCB, but contain useful reference information.

### BIAS Drive Setup (EVM)

The EVM implements BIAS drive through an inverting amplifier. The BIAS_DRV signal is available at the BIAS electrode connector. The BIAS drive loop includes a stability resistor (R8 = 392k) in the feedback path. After configuration, the voltage on either side of R8 should be close to mid-supply.

### Lead-Off Detection

The ADS1299 supports DC and AC lead-off detection:
- **DC lead-off**: Current source (6 nA, 24 nA, 6 uA, 24 uA) with comparator threshold (95%)
- **AC lead-off (in-band)**: 7.8 Hz or 31.25 Hz excitation for electrode impedance measurement
- **AC lead-off (out-of-band)**: fDR/4 frequency, can measure impedance concurrently with EEG using nA-range currents

Our system does not currently use lead-off detection (CONFIG4 = 0x00, lead-off comparator OFF).

### Test Signals

Internal test signal (CONFIG2 bit 4): 1x amplitude square wave at fDR/2^21 (~0.12 Hz at 250 SPS) or 2x amplitude. Enabled via `--test-signal` flag (sets CONFIG2=0xD0, CHnSET=0x05 for test signal input mux).

### Noise Specifications (from EVM testing)

- Input short (gain=24, 500 SPS, 10s): ~1 uVpp, offset ~23 uV
- External input short (5k resistors): ~1.27 uVpp (resistor thermal noise adds ~0.67 uVpp)
- With SRB1 reference (5k resistors): ~1.28 uVpp

### Power Supply

Battery power strongly recommended over wall adapter. Wall-powered supply makes the system more susceptible to 50/60 Hz noise. Our system validates this: Battery B (minimal battery) achieved +3 dB SNR over Battery A even at cold start.

---

## 7. Our System Noise Floor

Measured on Port1 Dev9 (9-deep chain, worst case), internal reference (CONFIG3=0xE0):

| Condition | RMS (uV) | P2P (uV) | NSD (uV/rtHz) |
|-----------|----------|----------|----------------|
| Shorted inputs (0x61) | 0.150 | 1.217 | 0.0146 |
| Floating inputs | 1,202 | 16,285 | 113 |
| External ref (shorted) | 86.4 | 715 | 8.17 |
| External ref (floating) | 0.000 | 0.000 | 0.000 (all zeros) |

**Internal reference is mandatory.** External reference is non-functional on our board (576x worse shorted, all-zeros floating).

**Chain depth has negligible impact:** Port1 Dev9 (9-deep) matches Port5 Dev4 (4-deep) within 1% RMS and 0.2 dB SNR.

---

## 8. Hardware Constants

From `registers.hpp`:

```
SPI_SPEED_HZ    = 6,000,000
SPI_MODE        = 0x01
ADS1299_DEVICE_ID = 0x3E
BYTES_PER_DEVICE = 27
I2C_BUS         = 6
TCA9534_INPUT_PORT = 0x00
TCA9534_OUTPUT_PORT = 0x01
TCA9534_CONFIG = 0x03
```

I2C addresses:
- DRDY expander: 0x20
- START expander: 0x21
