# C++ Rewrite of ADS1299 EEG Acquisition Controller

## Objective

Rewrite the Python `controller.py` (ADS1299 headset server) into C++ targeting a Raspberry Pi CM4 running Linux. The two primary goals are:

1. **Reduce SPI read latency** — Python's `spidev.xfer2()` has per-call overhead from the Python→C boundary, GIL acquisition, and object allocation. A native C++ implementation using direct `ioctl(SPI_IOC_MESSAGE)` eliminates this entirely.
2. **Eliminate data corruption events** — Python's GC pauses (50–200ms) cause missed DRDY windows and stale shift register reads. C++ with deterministic memory management and `SCHED_FIFO` real-time scheduling removes this class of failure.

## Implementation Phases

**This plan covers Phase 1 only.** Phase 2 (TCP streaming server with binary LZ4 protocol) will be added after Phase 1 is validated on hardware.

### Phase 1 — Acquisition + CSV Validation (this plan)
- Full ADS1299 initialization with all retry/verify sequences
- Real-time acquisition loop with parallel SPI bus workers
- Async CSV writer logging all channels
- Timing instrumentation (cycle time, dt, DRDY timeouts, corruption rate)
- **Validation criteria**: run for 10+ minutes, confirm 250 Hz lock, zero corruption, cycle time < 1.0 ms, CSV data matches Python output for the same hardware configuration

### Phase 2 — TCP Streaming (future)
- Binary LZ4 framing protocol (wire-compatible with existing Python client)
- TCP server with client reconnection support
- Lock-free sample handoff from acquisition to streaming thread
- Will be added as a layer on top of the validated Phase 1 engine

The streaming server, protocol, and LZ4 compression are explicitly **out of scope** for Phase 1. Do not implement them. The acquisition engine should push samples to the CSV writer only.

## Hardware Context

This runs on a custom CM4 expansion board (V2) with these specifics:

### ADS1299 Devices
- Up to 7 SPI ports, each with up to 4 daisy-chained ADS1299 bioamplifiers
- Current default: 7 ports × 4 devices = 28 devices = 224 channels @ 250 Hz
- Each device returns 27 bytes per sample (3 status + 8×3 channel data)
- Daisy-chain read: `num_devices × 27` bytes per SPI transfer
- ADS1299 SPI Mode 1 (CPOL=0, CPHA=1), clock speed 6 MHz
- V2 board: RESET/PWDN pins are hardwired HIGH — software reset only (`0x06` command)

### SPI Bus Layout
```
OUT0 → SPI0.CE0 (bus=0, device=0)
OUT1 → SPI0.CE1 (bus=0, device=1)
OUT2 → SPI3.CE0 (bus=3, device=0)
OUT3 → SPI3.CE1 (bus=3, device=1)
OUT4 → SPI4.CE0 (bus=4, device=0)
OUT5 → SPI4.CE1 (bus=4, device=1)
OUT6 → SPI5.CE0 (bus=5, device=0)
```

Ports on the SAME physical bus share SCLK/MOSI/MISO and must be read sequentially. Ports on DIFFERENT buses can be read in true parallel (separate kernel SPI controllers).

### I2C GPIO Expanders (TCA9534)
- Bus: I2C-6
- DRDY expander at `0x20`: active-LOW data-ready signals, one pin per port
- START expander at `0x21`: output pins to assert/deassert START on each port
- Pin mapping (output→pin): OUT0→P0, OUT1→P6, OUT2→P5, OUT3→P4, OUT4→P3, OUT5→P2, OUT6→P1

### Timing Budget
- Sample period: 4.0 ms (250 Hz)
- SPI transfer for 4-device daisy chain at 6 MHz: ~144 μs
- 7 ports across 4 physical buses → worst-case bus has 2 ports = ~288 μs sequential
- I2C DRDY poll: ~100–200 μs per read
- Total budget for DRDY poll + parallel SPI read + parse: must complete well within 4 ms

## Architecture Requirements

### 1. Project Structure

Use CMake. Target C++20 (`-std=c++20`). Cross-compile or native-compile on the Pi. Suggested layout:

```
ads1299-cpp/
├── CMakeLists.txt
├── include/
│   ├── ads1299/
│   │   ├── registers.hpp      — Register/command enums, config struct
│   │   ├── spi_device.hpp     — Low-level SPI communication class
│   │   ├── controller.hpp     — Device initialization and control logic
│   │   └── types.hpp          — Shared types (Sample, PortConfig, etc.)
│   ├── hardware/
│   │   ├── spi_bus.hpp        — Linux SPI ioctl wrapper
│   │   ├── i2c_device.hpp     — Linux I2C ioctl wrapper
│   │   └── tca9534.hpp        — TCA9534 GPIO expander driver
│   ├── acquisition/
│   │   ├── engine.hpp         — Main acquisition loop + timing instrumentation
│   │   ├── bus_worker.hpp     — Per-bus parallel SPI reader thread
│   │   └── drdy_poller.hpp    — DRDY polling via I2C
│   └── logging/
│       └── csv_writer.hpp     — Async CSV logger (lock-free SPSC ring buffer)
├── src/
│   ├── main.cpp
│   ├── ads1299/*.cpp
│   ├── hardware/*.cpp
│   ├── acquisition/*.cpp
│   └── logging/*.cpp
└── config/
    └── default_ports.json     — Default port configuration (optional)
```

### 2. SPI Communication Layer (`hardware/spi_bus.hpp`)

**This is the most performance-critical component.** Replace Python's `spidev` with direct Linux SPI ioctls:

```cpp
// Open: open("/dev/spidevB.D", O_RDWR)
// Configure: ioctl(fd, SPI_IOC_WR_MODE, &mode)
// Transfer: ioctl(fd, SPI_IOC_MESSAGE(1), &spi_ioc_transfer)
```

Key requirements:
- Use `spi_ioc_transfer` struct for zero-copy full-duplex transfers
- Pre-allocate TX/RX buffers (no heap allocation in hot path)
- Support both command-only transfers (TX only) and data reads (TX zeros, read RX)
- The `xfer` method must be as lean as possible — single ioctl call, no allocation

### 3. I2C Communication Layer (`hardware/i2c_device.hpp`)

Replace Python's `smbus2` with direct I2C ioctls:

```cpp
// Open: open("/dev/i2c-6", O_RDWR)
// Set slave: ioctl(fd, I2C_SLAVE, addr)
// Read/write: read(fd, buf, len) / write(fd, buf, len)
// Or use i2c_smbus_read_byte_data / i2c_smbus_write_byte_data from linux/i2c-dev.h
```

Key requirements:
- Single I2C bus instance shared across TCA9534 pin objects
- Thread-safe access (mutex around I2C operations — single bus)
- Batch reads: read entire port register once, check multiple DRDY pins from the byte

### 4. TCA9534 Driver (`hardware/tca9534.hpp`)

Port the `TCA9534_Pin` and `TCA9534_MultiPin` classes:
- `TCA9534Pin`: single-pin read/write with I2C bus reference
- `TCA9534MultiPin`: atomic multi-pin set-high/set-low for synchronized START assertion
- Configure pin direction (input for DRDY, output for START) in constructor

### 5. ADS1299 Device Layer (`ads1299/`)

**`registers.hpp`**: `enum class` for commands and register addresses (port the Python `ADS1299_Cmd` and `ADS1299_Reg` enums directly). Config struct with all register values and a `get_channel_settings()` method.

**`spi_device.hpp`**: Port `ADS1299_SPI` class. Key methods:
- `send_command(cmd)` — single-byte SPI transfer + 2μs delay
- `read_registers(start, count)` → `std::array` or fixed buffer
- `write_registers(start, values)`
- `read_raw()` → fills pre-allocated buffer (no allocation)
- `parse_raw(buffer, num_devices)` → fills pre-allocated `Sample` struct
- `is_data_ready()` → reads DRDY pin via TCA9534
- `wait_for_drdy(timeout)` → polling loop with `clock_gettime(CLOCK_MONOTONIC)`
- `flush_spi()` — send zeros to clear shift registers

**`controller.hpp`**: Port `ADS1299_Controller` static methods. This is complex initialization logic — port it faithfully, preserving:
- The multi-reset sequence with per-device-count timing
- SDATAC verification loop (write `0xAA` to LOFF, readback until match)
- Device ID verification (`0x3E`)
- Per-register write-and-verify with retry (the `write_and_verify` inner function)
- CONFIG3-first ordering (reference buffer needs settling time)
- `start_all_conversions_synchronized()` with its full warmup/verification/restart/resync sequence

**Important**: The initialization logic is battle-tested against real hardware flakiness. Do not simplify or skip any of the retry loops, SDATAC re-sends, or warmup sequences. Port them exactly.

### 6. Parallel Bus Workers (`acquisition/bus_worker.hpp`)

Port the `SPIBusWorker` pattern:
- One persistent `std::thread` per physical SPI bus
- Thread waits on a condition variable (or eventfd), triggered by the main acquisition loop
- Worker reads all ports on its bus sequentially (they share MISO/MOSI/SCLK)
- Workers on different buses execute truly in parallel (no GIL — this is C++)
- Pre-allocated raw buffers per port, written in-place by the worker
- Barrier synchronization: main thread triggers all workers, then waits for all to complete

Use `std::thread` + `std::condition_variable` or a lighter-weight signaling mechanism (eventfd + epoll for lower latency). The Python version uses `threading.Event`; the C++ equivalent should be at least as fast.

### 7. Acquisition Engine (`acquisition/engine.hpp`)

This is the real-time hot loop. Port `acquisition_thread()` with these requirements:

- **Thread pinning**: `pthread_setaffinity_np()` to pin to an isolated core (core 3)
- **RT scheduling**: `sched_setscheduler(SCHED_FIFO, priority=50)`
- **No heap allocation** in the hot loop — all buffers pre-allocated
- **No GC** (obviously — but also no `std::string` construction, no `std::vector` resizing)
- **DRDY polling**: Read I2C expander register, check combined mask for all active ports. Use `CLOCK_MONOTONIC` for timeout, not `gettimeofday()`.
- **Parallel SPI read**: Trigger all bus workers, wait for completion
- **Parse**: Convert raw bytes to int32 channel values (big-endian 24-bit signed → int32)
- **Queue**: Push completed `Sample` to the CSV writer's lock-free SPSC ring buffer (single consumer: the CSV writer thread). No streaming queue in Phase 1.
- **CSV**: The ring buffer is the only output path. Non-blocking push; if full, increment a drop counter (do not block the acquisition thread under any circumstance).

The 24-bit signed → int32 conversion is critical and appears in the hot path:
```cpp
// Python: int.from_bytes(raw[offset:offset+3], 'big', signed=True)
// C++: 
int32_t val = (static_cast<int32_t>(buf[0]) << 16) | (buf[1] << 8) | buf[2];
if (val & 0x800000) val |= 0xFF000000; // sign extension
```

### 8. CSV Writer (`logging/csv_writer.hpp`)

This is the **primary data output for Phase 1** and must not introduce jitter into the acquisition loop.

Architecture:
- Dedicated writer thread consuming from a **bounded SPSC (single-producer single-consumer) lock-free ring buffer**
- The acquisition thread does a single non-blocking `push()` per sample — if the ring is full, drop the oldest (log the drop count, don't block)
- Writer thread drains the ring in batches, writes to a file with a large buffer (`fopen` + `setvbuf` with 1 MB+ buffer, or direct `write()` with a userspace buffer)
- **No `std::string` construction per row** — format directly into a pre-allocated char buffer using `snprintf` or `std::to_chars`

CSV format (matching current Python output for comparison):
```
timestamp,sample_number,port1_dev1_ch1,port1_dev2_ch1,...,port7_dev4_ch1
```

The ring buffer should hold at least 2–5 seconds of samples (500–1250 entries at 250 Hz) to absorb filesystem write stalls (SD card can stall 50–100 ms on page flushes).

Additionally, write a **full-channel CSV** (all 224 channels per row) for validation:
```
timestamp,sample_number,ch0,ch1,ch2,...,ch223
```

This will be large (~60 KB/s at 250 Hz × 224 channels × ~12 digits per channel) but is essential for verifying the C++ output matches the Python output sample-for-sample. Can be disabled via a `--full-csv` flag after validation.

### 9. Main Entry Point (`main.cpp`)

Port the `main()` function:
- Argument parsing (use a lightweight library or manual parsing): `--ports`, `--full-csv` (enable all-channel CSV), `--duration` (optional auto-stop after N seconds for validation runs)
- Same port configuration format: `"bus,device,name,num_daisy"`
- Same initialization sequence: GPIO init → force START low → power-on wait → SPI device creation → sequential device configuration → synchronized start → acquisition loop
- Signal handling: `SIGINT`/`SIGTERM` for clean shutdown (stop acquisition, flush CSV, close SPI/I2C, report final stats)
- On shutdown, print: total samples acquired, runtime, achieved sample rate, min/max/mean cycle time, total DRDY timeouts, total corruption events

## Dependencies (Phase 1)

- **Linux headers**: `linux/spi/spidev.h`, `linux/i2c-dev.h`, `linux/i2c.h`
- **pthreads**: For thread affinity and RT scheduling (`-lpthread`)
- **Standard C++20**: `<thread>`, `<mutex>`, `<condition_variable>`, `<atomic>`, `<chrono>`, `<array>`, `<span>`
- No LZ4 (Phase 2), no Boost, no external frameworks — bare Linux userspace only

## Measured Python Performance & Why C++ Is Necessary

This section contains empirical timing data from real hardware runs. These numbers are the ground truth for understanding the problem and validating the C++ implementation.

### The Timing Budget

At 250 Hz, DRDY fires every **4.000 ms**. Each cycle the hot loop must:
1. Detect DRDY via I2C GPIO expander read (~100–200 μs)
2. Read all 7 SPI ports (4 daisy-chained devices × 27 bytes = 108 bytes per port)
3. Parse 224 channels of 24-bit big-endian data into int32
4. Push the completed sample to the async CSV writer deque

### Measured Python Timings

- **Typical read cycle**: 2.5–3.0 ms (consistently)
- **Headroom remaining**: only 1.0–1.5 ms before next DRDY
- **Min dt observed**: 1.3–2.0 ms (loop catching up after a slow cycle — proves the typical cycle eats most of the budget)
- **Max dt on clean runs**: 5.7–7.2 ms (1.4–1.8× the sample period) — **zero corruption** at these stalls
- **Corruption threshold**: a single event at **8.2 ms** (2.05× sample period) — a stall of merely ~4.2 ms beyond the sample period missed a DRDY edge and corrupted data across **3 independent SPI buses simultaneously**

This means the Python implementation has essentially zero safety margin. Any OS scheduling jitter, I2C bus contention, or page fault pushes past the corruption threshold.

### Root Causes (Python-Specific)

1. **GIL serialization**: The "parallel" `SPIBusWorker` threads are serialized by the GIL during `xfer2()`'s Python-side setup/teardown, even though the kernel ioctl itself releases the GIL. The workers are only truly parallel during the kernel `ioctl()` call — the Python bytecode before and after each call is serialized.
2. **spidev wrapper overhead**: Python's `spidev.xfer2()` adds ~50–100 μs of overhead per call over the raw `ioctl(SPI_IOC_MESSAGE)` due to list→buffer conversion, argument validation, and object allocation.
3. **Interpreted parsing**: The 24-bit→int32 conversion loop runs 224 iterations in CPython bytecode per sample.
4. **GC pauses**: Python's garbage collector can stall 50–200 ms (disabled in the current code via `gc.disable()`, but this leaks memory over long runs if any cyclic references form).

### C++ Performance Targets

| Metric | Python (measured) | C++ (target) | Why achievable |
|--------|------------------|--------------|----------------|
| Full read cycle (DRDY→sample queued) | 2.5–3.0 ms | **< 1.0 ms** | Direct ioctl, true parallelism, native parsing |
| Headroom per cycle | 1.0–1.5 ms | **> 3.0 ms** | 4.0 ms period − < 1.0 ms cycle |
| Stall tolerance before corruption | ~4.2 ms beyond period | **> 12 ms** (3× OS stall) | 3.0 ms+ headroom absorbs scheduling jitter |
| SPI transfer per port (108 bytes @ 6 MHz) | ~200–400 μs (with wrapper) | **~144 μs** (wire time only) | Single ioctl, no Python overhead |
| Worst-case parallel read (2 ports on slowest bus) | ~800 μs+ (GIL serialized) | **~288 μs** (truly parallel) | pthreads without GIL |
| 24-bit parse (224 channels) | ~100–200 μs (bytecode) | **< 10 μs** (native) | Tight loop, possibly SIMD |
| GC-induced pauses | 50–200 ms (or disabled) | **0** | No GC in C++ |
| Corruption rate | ~1 event per long run | **0** target | Deterministic timing |

### SPI Bus Parallelism Groups

The 4 independent SPI controllers can execute truly simultaneously in C++:

```
Group 0: SPI0 → Port1 (CE0) + Port2 (CE1)  — 2 ports, sequential within group
Group 1: SPI3 → Port3 (CE0) + Port4 (CE1)  — 2 ports, sequential within group
Group 2: SPI4 → Port5 (CE0) + Port6 (CE1)  — 2 ports, sequential within group
Group 3: SPI5 → Port7 (CE0)                 — 1 port
```

**Critical-path bus**: Any group with 2 ports takes ~288 μs (2 × 144 μs wire time). Group 3 takes only ~144 μs. The wall-clock time for all reads is determined by the slowest group.

In Python, the GIL means groups 0–3 are effectively serialized during the Python-side call setup, so actual wall time is closer to the sum of all transfers rather than the max. In C++, wall time equals the slowest group.

## Critical Implementation Notes

1. **Do not simplify the initialization logic.** The retry loops, multiple SDATAC sends, warmup sample discarding, and per-port restart sequences exist because ADS1299 daisy chains are temperamental. Every sleep duration and retry count was tuned against real hardware. Port them faithfully.

2. **The DRDY polling must be fast.** The Python version polls I2C at ~50 μs intervals. In C++, consider using a tighter poll loop (busy-wait with `sched_yield()` backoff), or if the kernel supports it, GPIO interrupt via `gpiod` for the I2C expander's INT pin (TCA9534 has an interrupt output). For v1, polling is fine.

3. **24-bit sign extension in the parse loop** is called `num_devices × 8` times per sample (224 times at full config). Make sure it compiles to efficient assembly — no branches if possible (arithmetic sign extension).

4. **Lock-free queue between acquisition and streaming threads.** The Python version uses `queue.Queue` which is mutex-locked. Use a bounded SPSC (single-producer single-consumer) ring buffer for zero-contention handoff.

5. **Pre-allocate everything.** Each port's raw SPI buffer, the parsed sample struct, the CSV row buffer, the LZ4 compression buffer — all allocated once at startup.

6. **Byte ordering.** ADS1299 sends data MSB-first (big-endian). The Pi is little-endian. The 24-bit→int32 conversion must handle this correctly.

7. **SPI bus locking.** Ports sharing a physical bus (e.g., SPI0.CE0 and SPI0.CE1) must not transfer simultaneously. The bus worker pattern handles this — one worker per bus reads its ports sequentially. But also consider that the Linux SPI driver may internally serialize access to the same controller, in which case explicit locking may be redundant (but keep it for safety).

8. **`volatile` is not needed** for SPI/I2C buffers — the kernel ioctl is a system call boundary that acts as a compiler barrier. Don't cargo-cult `volatile` onto data buffers.

9. **Instrument the hot loop from day one.** The acquisition engine must track and report: min/max/mean cycle time (DRDY→sample queued), min/max dt between consecutive samples, DRDY timeout count, and per-port SPI transfer duration. Print a summary every N seconds (e.g., every 10s). This is how we validated the Python implementation and how we'll confirm the C++ version meets the < 1.0 ms target. Use `clock_gettime(CLOCK_MONOTONIC)` for all timing — never `gettimeofday()` (it's not monotonic and NTP adjustments can cause backwards jumps).

10. **The corruption pattern is bus-wide, not port-specific.** When a DRDY edge is missed, the stale data in the shift registers corrupts ALL ports read in that cycle — the single corruption event at 8.2 ms hit 3 independent SPI buses simultaneously. This means the failure mode is "entire sample is garbage," not "one channel is off." The C++ implementation should detect this (status byte check: upper nibble must be `0xC0`) and log it rather than silently passing corrupt data downstream.

## Deliverables (Phase 1)

Produce a complete, compilable C++ project that:
1. Initializes all configured ADS1299 ports with the full retry/verify sequence
2. Runs a real-time acquisition loop reading all ports in parallel
3. Logs **all 224 channels** to CSV asynchronously via lock-free SPSC ring buffer
4. Prints live timing diagnostics every 10 seconds: min/max/mean cycle time, sample rate, DRDY timeout count, corruption count (status byte ≠ `0xC0` upper nibble)
5. Handles clean shutdown on SIGINT (stop acquisition, flush CSV, close SPI/I2C, print final stats)
6. Reports the same initialization diagnostic output (port status, register verification, warmup corruption rates) as the Python version

### Phase 1 Validation Checklist
- [ ] Compiles with `-std=c++20 -Wall -Wextra -Wpedantic` with zero warnings
- [ ] Initializes all 7 ports with 4 daisy-chained devices each
- [ ] Achieves locked 250 Hz sample rate
- [ ] Mean cycle time < 1.0 ms (measured and reported)
- [ ] Zero corruption over 10-minute run
- [ ] CSV output parseable and values match Python output for same hardware config
- [ ] Clean shutdown on Ctrl+C with final statistics

### What Is Explicitly Out of Scope
- TCP streaming server
- LZ4 compression
- Binary framing protocol
- Client reconnection logic
- Network socket handling of any kind

These will be added in Phase 2 after Phase 1 is validated on hardware.
