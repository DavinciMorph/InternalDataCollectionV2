# ADS1299 Startup Sequence for V2 Board

## Overview

This document describes the correct startup sequence for ADS1299 devices based on the TI datasheet (SBAS499C). The V2 board has RESET/PWDN pins tied HIGH, so software reset is used.

---

## Key Timing Constants (from Datasheet)

| Parameter | Value | Description |
|-----------|-------|-------------|
| tCLK | 0.488 μs | Clock period at 2.048 MHz internal clock |
| tPOR | 218 × tCLK (~106 μs) | Power-on reset time after CLK valid |
| VCAP1 | ≥1.1V required | Must charge before reset/communication |
| t4CLK | ~2 μs | Minimum wait after most commands |
| t18CLK | ~9 μs | Wait after RESET command (18 clock cycles) |
| Reference settle | 150 ms | Internal reference buffer settling time |
| Daisy-chain margin | 10-20 μs | Additional settling for last devices in chain |

---

## Phase 0: VCAP1 Charging (CRITICAL)

**Per datasheet Section 11.1 and Figure 76:**

Before ANY SPI communication or reset can occur, VCAP1 must charge to ≥1.1V.

```
1. Apply power to all supplies (AVDD, AVDD1, DVDD)
2. Wait for VCAP1 to charge to ≥1.1V
   - With 100μF capacitor, this takes ~500-1000ms
   - Do NOT issue reset or SPI commands before this
3. Wait tPOR (218 × tCLK) after CLK is valid
```

**VCAP1 is required for the internal reference and analog circuits to function properly.**

---

## Phase 1: Ensure START Pins are LOW

Before any configuration, all START pins must be LOW to prevent devices from converting during setup.

```
4. Set all START pins LOW (via I2C GPIO expanders)
5. Wait 10 ms for pins to settle
```

**Rationale**: Devices in conversion mode ignore register writes.

---

## Phase 2: Software Reset

Since RESET/PWDN are tied HIGH on V2 board, use software reset.

```
6. Send RESET command (0x06) via SPI
7. Wait for reset completion (18 tCLK = ~9μs minimum)
   - Use 1ms for generous margin
```

**Note**: The datasheet specifies 18 tCLK minimum. Excessive waits are not necessary if VCAP1 is properly charged.

---

## Phase 3: Exit Continuous Data Mode

After reset, device wakes in RDATAC mode. All register operations require SDATAC mode.

```
8. Send SDATAC command (0x11)
9. Wait 4 tCLK (~2μs minimum, use 1ms for margin)
```

**Critical**: Always send SDATAC before any register read/write operations.

---

## Phase 4: Verify Communication

```
10. Read ID register (address 0x00)
    - Expected: 0x3E for ADS1299 (8-channel)
    - Bits [4:2] = 0b111 indicates ADS1299

11. If ID = 0x00: No communication (VCAP1 not charged, check wiring)
12. If ID = 0xFF: MISO floating or disconnected
13. If ID valid: Continue to configuration
```

---

## Phase 5: Register Configuration (Order Matters)

### Step 1: CONFIG1 - Data Rate and Daisy-Chain Mode
```
14. Write CONFIG1 (0x01) = 0x96
    - Bit 7: Reserved = 1
    - Bit 6: DAISY_EN = 0 (daisy-chain mode)
    - Bit 5: CLK_EN = 0 (oscillator output disabled)
    - Bits [2:0]: DR = 110 (250 SPS)
15. Wait 10 μs
```

### Step 2: CONFIG2 - Test Signal Configuration
```
16. Write CONFIG2 (0x02) = 0xD0 (test signal) or 0xC0 (normal)
    - Bit 7: Reserved = 1
    - Bit 6: Reserved = 1
    - Bit 5: INT_CAL = 0 (external calibration) or 1 (internal test)
    - Bit 4: CAL_AMP = 1 (1× amplitude)
    - Bits [1:0]: CAL_FREQ = 00 (fCLK/2^21)
17. Wait 10 μs
```

### Step 3: CONFIG3 - Reference Buffer (CRITICAL TIMING)
```
18. Write CONFIG3 (0x03) = 0xE0
    - Bit 7: PD_REFBUF = 1 (internal reference buffer enabled)
    - Bit 6: Reserved = 1 (MUST be written as 1)
    - Bit 5: BIAS_MEAS = 1 (BIAS measurement enabled)
    - Bit 4: BIASREF_INT = 0 (external BIASREF)
    - Bit 3: PD_BIAS = 0 (BIAS buffer powered down)
    - Bit 2: BIAS_LOFF_SENS = 0
    - Bit 1: BIAS_STAT = 0
19. Wait 150 ms for reference buffer to power up and settle
```

**Per datasheet**: Internal reference startup time is ~150ms typical.

### Step 4: Re-write CONFIG2 and CONFIG3
```
20. Write CONFIG2 again (ensures it sticks after reference settling)
21. Write CONFIG3 again (ensures reference is stable)
22. Wait 10 μs
```

**Note**: Re-writing these registers after reference settling improves reliability.

### Step 5: Channel Settings (CH1SET - CH8SET)
```
23. Write CH1SET through CH8SET (0x05-0x0C)
    - For test signal: 0x05 (gain=1, test signal input)
    - For normal operation: 0x00 (gain=24, normal input)
24. Wait 10 μs
```

**Daisy-chain note**: These writes broadcast to ALL devices in the chain.

### Step 6: Verify Channel Settings
```
25. Read CH1SET through CH8SET
26. Compare to expected values
27. If mismatch: Re-write and verify again
```

**Limitation**: Only first device in daisy-chain can be read back.

### Step 7: MISC1 and CONFIG4
```
28. Write MISC1 (0x15) = 0x20
    - Bit 5: SRB1 = 1 (SRB1 connected to all channels)
29. Write CONFIG4 (0x17) = 0x00
    - Continuous conversion mode
    - Lead-off comparator disabled
30. Wait 10 μs
```

### Step 8: Additional Reference Settling
```
31. Wait 50 ms additional margin
    (Total reference settling: ~200 ms since CONFIG3 write)
```

---

## Phase 6: Verify Configuration

```
32. Read CONFIG1 - verify = 0x96
33. Read CONFIG2 - verify = 0xD0 or 0xC0
34. Read CONFIG3 - verify = 0xE0

35. If CONFIG3 ≠ expected:
    - Reference buffer not ready
    - Re-write CONFIG3
    - Wait 500 ms
    - Retry verification
```

---

## Phase 7: Enter Continuous Data Mode

```
36. Send RDATAC command (0x10)
37. Wait 10 μs
```

Device now outputs data automatically on every DRDY pulse.

---

## Phase 8: Start Synchronized Conversion

### For Multiple Ports (Synchronized)

```
38. Collect all START pin references
39. Assert ALL START pins HIGH simultaneously (atomic I2C write)
40. Wait 500 ms for first conversion cycles to stabilize
41. All DRDY signals should now pulse together at 250 Hz
```

**Critical**: START pins must be asserted simultaneously for synchronized DRDY.

---

## Phase 9: Warmup and Discard

```
42. Discard first 50 samples (~200 ms at 250 Hz)
    - Initial samples may have settling artifacts
43. Reset corruption counters after warmup
```

---

## Phase 10: Data Acquisition Loop

### Correct Read Sequence

```
44. Wait for ALL DRDY signals = LOW (all ports ready)
45. Wait 20 μs for daisy-chain data to stabilize
46. For each SPI port:
    a. Wait 10 μs (settling)
    b. Transfer (num_devices × 27) bytes
    c. Wait 10 μs before next port
47. Validate status bytes (should be 0xCx)
48. Parse channel data
49. Repeat from step 44
```

### Status Byte Validation

```
Valid status byte: Upper nibble = 0xC (1100 binary)
- 0xC0, 0xC1, 0xC2, ... 0xCF are valid
- 0x00 = No data / communication error
- 0xFF = MISO floating
- Other values = Bit-shift corruption (daisy-chain timing issue)
```

---

## Daisy-Chain Specific Requirements

### Signal Propagation

Each device in the chain adds propagation delay:
- Device 1: ~2 ns
- Device 3: ~8 ns
- Device 5: ~30+ ns

At 2 MHz (500 ns/bit), 30 ns is ~6% of a clock cycle.

### Required Margins

| Operation | Single Device | 3-Device Chain | 5-Device Chain |
|-----------|---------------|----------------|----------------|
| Post-command wait | 2 μs | 5 μs | 10 μs |
| Pre-read settling | 2 μs | 5 μs | 10 μs |
| Post-DRDY wait | 5 μs | 10 μs | 20 μs |

### Data Order

In daisy-chain mode, data comes out in order:
1. First device (closest to master) - bytes 0-26
2. Second device - bytes 27-53
3. Last device - bytes (n-1)×27 to n×27-1

---

## Error Recovery Sequence

When corruption is detected or DRDY timeouts occur:

```
1. Assert all START pins LOW
2. Send STOP command (0x0A) to all ports
3. Wait 100 ms
4. Send SDATAC command (0x11) to all ports
5. Wait 50 ms
6. Re-initialize from Phase 2 (Software Reset)
7. Restart synchronized conversion
```

---

## Complete Timing Summary

| Phase | Duration |
|-------|----------|
| VCAP1 charging | 1000 ms |
| START pins LOW | 10 ms |
| Software reset | 1 ms |
| SDATAC | 1 ms |
| ID verification | 1 ms |
| CONFIG1-2 writes | <1 ms |
| CONFIG3 + settling | 150 ms |
| Re-write CONFIG2/3 | <1 ms |
| Channel settings | <1 ms |
| MISC1/CONFIG4 | <1 ms |
| Additional settling | 50 ms |
| Config verification | 1 ms |
| RDATAC | <1 ms |
| START + stabilize | 500 ms |
| Warmup discard | 200 ms |
| **Total** | **~1.9 seconds** |

---

## Common Failure Modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| ID = 0x00 | VCAP1 not charged / No SPI | Wait for VCAP1, check wiring |
| ID = 0xFF | MISO disconnected | Check MISO connection |
| CONFIG3 wrong | Reference not settled | Wait longer, re-write |
| CH8SET = 0x61 | Timing issue | Add delays between writes |
| Status = 0x00 | Data not ready | Wait for DRDY, add settling |
| Status = 0xEE/0x3F | Bit-shift corruption | Increase daisy-chain delays |
| Last device corrupted | Propagation delay | Add 10-20 μs pre-read delay |
| Some ports work, others don't | VCAP1 timing varies | Increase VCAP1 wait time |

---

## References

- Texas Instruments ADS1299 Datasheet (SBAS499C)
- Section 11.1: Power-Up Sequencing
- Figure 76: Power-Up Timing
- Figure 67: Initial Flow at Power-Up
