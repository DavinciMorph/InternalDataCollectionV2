# ADS1299 / ADS1299-4 / ADS1299-6 — Comprehensive Markdown Datasheet

**Source:** Texas Instruments *ADS1299, ADS1299-4, ADS1299-6 — SBAS499C (July 2012, revised Jan 2017)*  
**Package:** TQFP-64 (10 mm × 10 mm)

> This document is a cleaned, structured Markdown version of the key content found in the TI datasheet.  
> For authoritative details (figures, full electrical tables, ordering addendum), refer to the original PDF.

---

## Table of contents

1. [Overview](#overview)  
2. [Key features](#key-features)  
3. [Device variants](#device-variants)  
4. [System architecture](#system-architecture)  
5. [Pin functions (high-level)](#pin-functions-high-level)  
6. [Power & reference](#power--reference)  
   - [Power supplies](#power-supplies)  
   - [Decoupling / VCAP requirements](#decoupling--vcap-requirements)  
   - [Reference (internal vs external)](#reference-internal-vs-external)  
   - [Clocking (internal vs external)](#clocking-internal-vs-external)  
   - [Power-up sequencing (explicit steps)](#power-up-sequencing-explicit-steps)  
7. [Analog front-end](#analog-front-end)  
8. [Lead-off detection](#lead-off-detection)  
9. [BIAS drive amplifier](#bias-drive-amplifier)  
10. [Digital interface (SPI)](#digital-interface-spi)  
11. [Data framing](#data-framing)  
12. [Command set](#command-set)  
13. [Register map summary](#register-map-summary)  
14. [Design guidance](#design-guidance)  
15. [Package / ordering notes](#package--ordering-notes)

---

## Overview

ADS1299-x is a low-noise, low-power, multichannel **simultaneous-sampling 24-bit delta-sigma ADC** with an **integrated programmable-gain amplifier (PGA)** and EEG/biopotential-oriented features (bias drive, lead-off detection, test-signal injection, per-channel muxing, GPIO, and multi-device synchronization).

---

## Key features

- Up to **8 channels** (variant-dependent): simultaneous 24-bit ADCs + PGAs  
- Data rate: **250 SPS to 16 kSPS**  
- PGA gain: **1, 2, 4, 6, 8, 12, 24**  
- Typical headline input-referred noise (example): **~1 µVpp in 70 Hz BW** (see noise tables by data rate/gain)  
- Analog supply: **4.75–5.25 V**; Digital supply: **1.8–3.6 V**  
- Internal oscillator (nominal **2.048 MHz**) and internal reference option (nominal **4.5 V**)  
- Lead-off detection options: DC / AC, continuous/out-of-band and one-time/in-band modes  
- Daisy-chain & sync support for multi-device systems

---

## Device variants

- **ADS1299-4**: 4-channel  
- **ADS1299-6**: 6-channel  
- **ADS1299**: 8-channel  

The ID register encodes the channel count.

---

## System architecture

High-level blocks:
- Per-channel: input multiplexer → PGA → delta-sigma modulator → digital decimation filter → 24-bit output word
- Shared: clocking, reference, bias drive amplifier, lead-off excitation/status, SPI interface, GPIO, synchronization

---

## Pin functions (high-level)

(Refer to the original datasheet pin tables for the complete pin number mapping.)

**Key analog pins**
- INxP / INxN: differential inputs per channel  
- SRB1: shared reference bus (common reference node for montage options)  
- SRB2: per-channel switchable connection (selected through CHnSET.SRB2)  
- VREFP / VREFN: reference pins  
- BIASIN / BIASINV / BIASOUT / BIASREF: bias drive amplifier nodes  
- AVDD / AVSS (+ AVDD1 / AVSS1 where present): analog supplies/returns  
- DVDD / DGND: digital supply/ground

**Key digital pins**
- CS, SCLK, DIN, DOUT: SPI bus  
- DRDY: data-ready indicator  
- START: conversion start / sync input  
- RESET: hardware reset  
- PWDN: power-down control (active-low)  
- CLK, CLKSEL: master clock input and select (external vs internal oscillator)  
- GPIO1..GPIO4: general-purpose I/O  

---

## Power & reference

### Power supplies

The device uses **three** supplies: **AVDD**, **AVDD1**, and **DVDD**.  
For best performance:
- Keep AVDD and AVDD1 as quiet as possible.
- AVDD1 powers a charge pump block and has transients at fCLK.
- **Star-connect** AVDD1 to AVDD pins and AVSS1 to AVSS pins.
- Use local bypass capacitors (10 µF and 0.1 µF ceramics) on each supply pin group.
- Place high-current digital processing (MCUs/FPGAs/DSPs) so return currents do not pollute the analog return.

### Decoupling / VCAP requirements

The datasheet specifies required capacitors for internal nodes (VCAP pins). Typical required connections include:
- VCAP1: 100 µF to AVSS  
- VCAP2: 1 µF to AVSS  
- VCAP3: 1 µF || 0.1 µF to AVSS  
- VCAP4: 1 µF to AVSS  

**Important:** When using the internal reference, noise on VCAP1 degrades performance.

### Reference (internal vs external)

- **Default power-up reference mode:** external reference mode.
- **Internal reference option:** enable the internal reference buffer through CONFIG3.PD_REFBUF.
- Internal reference characteristics (typical): **4.5 V**, startup time **~150 ms**.

### Clocking (internal vs external)

Two modes:
- **External clock:** CLKSEL = 0, provide a master clock at CLK (nominal 2.048 MHz; allowed range in datasheet).  
- **Internal oscillator:** CLKSEL = 1, the device uses internal oscillator (nominal 2.048 MHz).

---

## Power-up sequencing (explicit steps)

This section is intentionally explicit and aligns with:
- **Section 11.1 Power-Up Sequencing (Figure 76 + Table 30)**  
- **Figure 67 “Initial Flow at Power-Up”**  
- SPI command timing notes (RESET command, RDATAC default, SDATAC before register writes)

### A) Pre-power conditions (before applying supplies)

1. **Hold all digital and analog inputs LOW before device power-up.**  
2. At the time of power-up, **keep these signals low until the power supplies have stabilized** (datasheet requirement).

> In practice, “digital inputs” includes at least: CS, SCLK, DIN, START, RESET, PWDN, CLKSEL, GPIOs.  
> “Analog inputs” refers to any externally-driven analog pins that could be forced low in your design.

### B) Apply power rails and let them stabilize

3. Apply **AVDD/AVSS**, **AVDD1/AVSS1** (if used), and **DVDD/DGND**.  
4. Allow time for supply voltages to reach final values (stable).

### C) Provide a valid master clock

5. Select clock source and ensure the clock is present/stable:

   **External clock path**
   - Set **CLKSEL = 0**
   - Provide external clock on **CLK**
   - **Important note:** For external clock use, the datasheet notes that the **tPOR timing does not start until CLK is present and valid**.

   **Internal oscillator path**
   - Set **CLKSEL = 1**
   - Wait for internal oscillator to wake up (the datasheet notes DRDY toggling behavior during this stage depending on START wiring).

6. Ensure **PWDN is deasserted** (i.e., pin high; not in power-down).  
7. Keep **RESET high** (not held in reset), except when pulsing reset in the next step.

### D) Wait the required POR/VCAP time before issuing reset

8. **Wait at least tPOR** (datasheet minimum: **218 × tCLK**) *after power-up* (and after CLK is valid if using external clock).  
9. **Also ensure VCAP1 has charged to at least 1.1 V** *before issuing reset*.  
   - If VCAP1 < 1.1 V at tPOR, **continue waiting until VCAP1 ≥ 1.1 V**.  
   - VCAP1 charge time is determined by the RC constant associated with the VCAP1 capacitor value.

**Timing requirements (from Figure 76 / Table 30)**
- **tPOR** (wait after power-up until reset): **min 218 × tCLK**  
- **tRST** (reset low duration): **min 2 × tCLK**

### E) Reset the digital core

10. Issue reset using **either**:
   - **Hardware reset pin:** pulse RESET low for **≥ tRST (2 × tCLK)**, then release high  
   - **SPI RESET command:** send the RESET opcode on SPI

11. After issuing reset, **wait for reset completion** before sending other commands.  
    - The datasheet describes reset command execution on the order of **18 × tCLK** cycles; do not issue new commands during this time.

### F) Stop RDATAC (required before register writes)

12. The device **wakes up in RDATAC mode** by default.  
13. To write registers safely, **send SDATAC** first.  
    - Register RREG/WREG operations should not be used while the device is actively in RDATAC mode.

### G) Configure reference and core registers

14. Program configuration registers (WREG), as needed for your design.

**Reference configuration**
- If using **internal reference**, set **CONFIG3.PD_REFBUF = 1** and ensure reserved bits [6:5] are written as required by the datasheet (always write `11b`).  
  - Example shown in TI’s “Initial Flow at Power-Up”: **WREG CONFIG3 = 0xE0** (enables internal reference buffer; reserved bits correct).  
- After enabling internal reference, **allow reference startup/settling time** (datasheet: **~150 ms** typical).

**Core configuration**
- Set data rate in CONFIG1 (DR bits).  
- Configure CONFIG2 if using internal test signals.  
- Configure each channel CHnSET (power, gain, mux, SRB2 as desired).  
- Configure BIAS and lead-off registers if used (BIAS_SENSP/N, LOFF, LOFF_SENSP/N, etc.).  
- Configure GPIO if needed.

### H) Start conversions and read data

15. Start conversions via:
- **START pin** (assert high), or
- **START SPI command**

16. If you want continuous streaming, (re-)enable continuous read mode:
- Send **RDATAC**

17. Read data frames on **DRDY**:
- On each DRDY event, clock out:
  - 24-bit status word
  - 24-bit channel words (N channels)

---

## Analog front-end

### PGA + ADC

- PGA gain: 1 / 2 / 4 / 6 / 8 / 12 / 24  
- Full-scale differential input (typical relationship): **± VREF / gain**  
- Input mux supports normal electrode input, input short, test signal, temperature sensor, and other internal routing options.

---

## Lead-off detection

- Lead-off can use programmable current sources/sinks with selectable magnitude and frequency (DC and AC options).
- Status is reported via LOFF_STATP and LOFF_STATN registers.
- Additional BIAS lead-off detection functionality exists specifically at power-up.

---

## BIAS drive amplifier

- Bias drive supports deriving an average common-mode from selected electrodes and driving the subject reference via BIASOUT.
- Recommended stabilization networks are shown in the datasheet (values depend on your electrode/skin model and cable capacitance).

---

## Digital interface (SPI)

- SPI signals: CS, SCLK, DIN, DOUT
- Data shifts:
  - into DIN on **SCLK falling edge**
  - out of DOUT on **SCLK rising edge**
- CS must remain low for an entire command/transaction; DOUT is Hi-Z when CS is high.

---

## Data framing

A conversion frame includes:
- 24-bit status word  
- N × 24-bit channel words  
All channel words are two’s complement, MSB-first.

---

## Command set

Common commands include:
- WAKEUP, STANDBY, RESET, START, STOP  
- RDATAC, SDATAC, RDATA  
- RREG, WREG  

**Note:** SDATAC is required before register accesses when the device is in RDATAC mode.

---

## Register map summary

Addresses and defaults (summary):
- 00h ID  
- 01h CONFIG1 (default 0x96)  
- 02h CONFIG2 (default 0xC0)  
- 03h CONFIG3 (default 0x60)  
- 04h LOFF (default 0x00)  
- 05h–0Ch CH1SET–CH8SET (default 0x61 each)  
- 0Dh BIAS_SENSP (0x00)  
- 0Eh BIAS_SENSN (0x00)  
- 0Fh LOFF_SENSP (0x00)  
- 10h LOFF_SENSN (0x00)  
- 11h LOFF_FLIP (0x00)  
- 12h LOFF_STATP (RO)  
- 13h LOFF_STATN (RO)  
- 14h GPIO (default 0x0F)  
- 15h MISC1 (includes SRB1 enable)  
- 16h MISC2  
- 17h CONFIG4 (SINGLE_SHOT, PD_LOFF_COMP)

---

## Design guidance

- Keep analog/digital grounds carefully managed; avoid digital return currents crossing analog return paths.
- Use recommended decoupling values and place capacitors close to pins.
- Pay special attention to VCAP1 noise if using internal reference.
- Use montage routing (SRB1 / SRB2) appropriately for referential vs differential measurements.

---

## Package / ordering notes

See the datasheet ordering addendum for exact orderable part numbers and package options.

---

### Revision / provenance

This Markdown file is derived from the TI ADS1299 family datasheet **SBAS499C** (revised Jan 2017).
