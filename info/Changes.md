# Changes Made

## Attempt 1: Aggressive Changes (FAILED)

| Change | Purpose | Outcome |
|--------|---------|---------|
| `flush_spi()` calls added | Clear SPI garbage | **DESTABILIZED SPI** |
| ID verification loop | Wait for bus ready | Made things worse |
| Pre-config SDATAC + flush | Ensure command mode | Made things worse |

**Result**: System failed to initialize after all 5 retries.

---

## Attempt 2: Minimal Changes (SUCCESS)

| Change | Purpose | Outcome |
|--------|---------|---------|
| System retry STOP delay: 100ms → 500ms | Allow SPI bus to settle | **Working** |
| 50ms delay between port configurations | Allow I2C bus to settle | **Working** |

**Result**:
- Run 1: Initialized successfully (with some port restarts), 0.04% corruption
- Run 2: Clean initialization, **0.004% corruption** (6 corruptions in 31,250 reads)

---

## Remaining Corruption Analysis

The remaining corruption is **hardware-related**, not software:

| Pattern | Evidence | Cause |
|---------|----------|-------|
| Last device in chain affected | Device 3/3, Device 5/5 always | Daisy-chain signal propagation |
| Bursty (2-3 consecutive) | Long clean periods, then spike | Timing/electrical glitch |
| Max ADC values (0x7FFFFF) | Saturation values | Signal not settling |
| Valid status, bad data | Status=0xC0 but OOR values | Data propagation delay |

---

## Can We Achieve 0% Corruption?

**A buffer system will NOT help** - the corruption source is electrical, not software timing.

### Options:

| Option | Approach | Trade-off |
|--------|----------|-----------|
| **Interpolation** (recommended) | Detect bad samples, substitute interpolated values | Slightly modified data |
| Hardware changes | Shorter chains, better termination | Cost/complexity |
| Redundant reads | Read 3x, majority vote | Cannot maintain 250Hz |

### Recommended: Interpolation - IMPLEMENTED

| Change | Description |
|--------|-------------|
| `last_valid_samples` | Track last valid value per device/channel |
| `interpolation_count` | Track how many samples were interpolated |
| Status byte invalid | Use all last valid values for that device |
| OUT_OF_RANGE value | Substitute with last valid value |
| Logging | Shows "interpolated" in OOR messages, total in stats |

**Result**: Client receives clean data stream with no visible spikes
