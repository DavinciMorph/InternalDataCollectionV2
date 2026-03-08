# Init Reliability

**Last Updated:** 2026-03-07

History and current state of ADS1299 initialization reliability. Documents the evolution from unreliable Python init to the current C++ tiered recovery system.

---

## 1. Root Causes of Init Failure

Two distinct root causes were identified across the project history:

### Dead Ports Not Excluded (FIXED)

The original init code would attempt to start acquisition even when ports returned all zeros. Dead ports were included in the DRDY mask, causing the poller to wait forever or time out every cycle. This produced 100% corruption on all ports because the hot loop stalled waiting for a dead port's DRDY.

**Fix:** Dead port exclusion. After health checks, `active_devices` and `active_configs` vectors are built containing only healthy ports. Dead ports are removed from the DRDY poller mask, bus worker list, CSV writer, and streaming server.

### Power-On Latch (FIXED)

The old `recover_port()` function only cycled RDATAC mode (STOP/SDATAC/RDATAC/START). If a device was in a latched power-on failure state, this was insufficient. The device needed a full SOFTWARE RESET to clear the latch, followed by complete register reconfiguration.

**Fix:** Tier 2 recovery sends 2x SOFTWARE RESET, waits 150ms, then rewrites all registers.

### SPI Bus Contention from Bad Hardware

A single shorted ADS1299 device can pull SPI bus lines (MISO, CS) low, causing all devices on that bus to fail. This was discovered during the SPI3 debugging session (2026-03-06) when GPIO24 (SPI3 CS1) read 0 even when driven HIGH via `/dev/mem`. Diagnostic: if a GPIO pin reads 0 after writing GPSET, the fault is external to the Pi. The fix was physical removal of the shorted device.

---

## 2. Current Init Architecture (C++)

Source: `main.cpp`, `controller.cpp`, `controller.hpp`

### Startup Flow

```
1. Parse CLI args (--min-ports, --bias-port, --config3, --chnset, etc.)
2. Build per-port DeviceConfig array (DefaultPortConfigs)
3. For each port: initialize_device() or initialize_device_simple()
4. Run tiered recovery for any failed ports
5. Apply 7/7 hard gate (or --min-ports N)
6. Build active_devices / active_configs (exclude dead ports)
7. start_and_verify() -- synchronized START + sample verification
8. Warmup: 250 samples
9. Final health check: 50 samples
10. Enter acquisition hot loop
```

### Tiered Recovery (`run_tiered_recovery`)

When a port fails initialization, it enters the tiered recovery pipeline:

| Tier | Action | Duration | Max Attempts |
|------|--------|----------|-------------|
| 1 | RDATAC cycle (STOP/SDATAC x10/flush x3/RDATAC x3/START) | ~200ms | 8 |
| 2 | Software RESET x2 + full register reconfig | ~500ms | 5 |
| 3 | Full re-init of ALL ports | ~5s | 2 |

Constants in `main.cpp`: `TIER1_ATTEMPTS=8`, `TIER2_ATTEMPTS=5`, `TIER3_ATTEMPTS=2`.

### 7/7 Hard Gate

After tiered recovery, the system checks that all expected ports are healthy. If any port is still dead:

- **Default behavior:** Exit with code 2 (signals "power cycle needed")
- **`--min-ports N`:** Accept N or more healthy ports; dead ports excluded from pipeline

### Per-Port Configuration

Each port has its own `DeviceConfig` struct with independent register values. This allows:
- Different gain per port (e.g., `--chnset 0x60` as default, per-port override possible)
- BIAS routing on a specific port (`--bias-port N`)
- Recovery uses the correct per-port config (not a global singleton)

---

## 3. Solutions Proposed vs. Implemented

During the init reliability investigation, multiple solutions were proposed in analysis documents. Here is their status against the current codebase:

### Implemented in Current Code

| Solution | Where | Status |
|----------|-------|--------|
| Tiered recovery (3 tiers) | `main.cpp`, `controller.cpp` | ACTIVE |
| Dead port exclusion from pipeline | `main.cpp` (active_devices) | ACTIVE |
| 7/7 hard gate with --min-ports escape | `main.cpp` | ACTIVE |
| Per-port DeviceConfig (not global) | `types.hpp`, `main.cpp` | ACTIVE |
| SDATAC flood (max(5, num_devices+2)) | `controller.cpp` | ACTIVE |
| SDATAC verification via LOFF 0xAA test | `controller.cpp` | ACTIVE |
| CONFIG3 written FIRST (ref buffer settle) | `controller.cpp` | ACTIVE |
| 200ms reference buffer wait | `controller.cpp` | ACTIVE |
| Write-once init variant | `controller.cpp` (initialize_device_simple) | AVAILABLE |
| Warmup increased to 250 samples | `main.cpp` | ACTIVE |

### Not Implemented (Proposed Only)

| Proposal | Reason Not Implemented |
|----------|----------------------|
| Hardware RESET via GPIO | V2 board ties RESET/PWDN HIGH, no GPIO access |
| Periodic register readback during acquisition | Would require exiting RDATAC, causing data gaps |
| Per-port rolling RESET during acquisition | Not needed since oscillation root cause identified |
| Escalating delays between retries | Found to be counterproductive (memoryless timing race) |
| Global re-sync after each tier | Replaced by atomic START via TCA9534MultiPin |
| SPI flush extended to full frame length | Fixed in C++ (flushes num_devices*27 bytes) |

---

## 4. Init Sequence Details

### initialize_device (Verified Write)

1. START low via I2C
2. Flush + STOP
3. 2x SOFTWARE RESET (100ms each)
4. SDATAC flood: max(5, num_devices+2) with 20ms gaps
5. SDATAC verify: WREG LOFF=0xAA, RREG LOFF, check 0xAA (100 attempts)
6. ID check: RREG ID, verify 0x3E (100 attempts)
7. WREG CONFIG3 (ref buffer)
8. Wait 200ms
9. WREG CONFIG1, CONFIG2, CH1-8SET (with BIAS config if applicable), MISC1, CONFIG4
10. Final readback verification of all registers

### initialize_device_simple (Write-Once)

Simplified variant that skips all readback verification. Used when excessive SPI traffic during init corrupts the data pipeline (discovered during SPI3 debugging).

1. 2x SOFTWARE RESET (100ms each)
2. SDATAC x10
3. Write all registers once (no readback)

### start_and_verify

1. STOP all ports
2. SDATAC x10 on all ports
3. Flush x3 on all ports
4. RDATAC x3 on all ports
5. Atomic START via TCA9534MultiPin
6. Verify 10 samples per port (80% threshold for valid status bytes)

### recover_port_tier1

STOP -> SDATAC x10 -> flush x3 -> RDATAC x3 -> START

### recover_port_tier2

SOFTWARE RESET x2 -> wait 150ms -> SDATAC x5 -> rewrite all registers (per-port config) -> flush/RDATAC -> START

---

## 5. Python Init Issues (Historical)

The original Python init (`Controller.py`, `ControllerPhase1.py`) had several issues, all addressed in the C++ rewrite:

| Issue | Python Behavior | C++ Fix |
|-------|----------------|---------|
| "Aggressive re-init" was not aggressive | Only cycled STOP/SDATAC/RDATAC/START with longer delays; no RESET or register rewrite | Tier 2 does full RESET + register rewrite |
| Final re-sync skipped RDATAC | Could leave devices in command mode | start_and_verify sends RDATAC x3 |
| SPI flush too short (32 bytes) | Full frame for 4 devices = 108 bytes | Flushes num_devices * 27 bytes |
| Status check only validated device 0 | Devices 1-3 status unchecked | Per-device status validation in hot loop |
| Single restart during warmup | At most one retry per port | Tiered recovery with multiple attempts |
| No check for all-zero channel data | Could have valid status but zero channels | Health check verifies actual sample values |
| CSV writes in hot loop | I/O stalls up to 1564ms | SPSC ring buffer, async writer thread |
| GC pauses during acquisition | Random gen2 collection freezes | mlockall + no GC (C++ has no GC) |

---

## 6. Validation Results

### C++ First Run (2026-02-18)

- Duration: 23.3 min, 349k samples, 224 channels
- Sample rate: 250.0 Hz locked, zero gaps, zero corruption, zero jitter events
- dt: mean 4.000ms, min 1.241ms, max 4.660ms
- Result: **CLEAN -- zero corruption events**

### After Tiered Recovery (2026-02-19)

- 6 ports operational (Port4 excluded due to bad hardware)
- 14,999 samples at 250.0 Hz for 60 seconds
- Cycle time: 0.60ms mean, 0.70ms max
- Zero timeouts, zero drops, 3 corruption events (0.02%)
- All 6 ports: 100% valid on final health check

### Current System (7/7 ports, 40 devices)

When all hardware is functional, the system achieves 7/7 port initialization with zero-corruption acquisition at 250 Hz.
