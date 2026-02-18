# ADS1299 C++ Acquisition Engine

Real-time 224-channel EEG acquisition from 28 daisy-chained ADS1299 devices on Raspberry Pi 4.

## Motivation

The Python acquisition server (`server/ControllerPhase1.py`) achieves 250 Hz but with only 1.0-1.5 ms headroom per cycle. An 8.2 ms OS scheduling stall (2x the 4 ms sample period) caused data corruption across 3 SPI buses simultaneously. Python's GIL serializes the "parallel" bus workers, and spidev adds ~50-100 us overhead per call.

The C++ rewrite eliminates these bottlenecks: direct SPI ioctl, true thread parallelism, zero heap allocation in the hot loop, and RT scheduling on an isolated core.

## Architecture

```
ads1299-cpp/
├── CMakeLists.txt
├── include/
│   ├── ads1299/
│   │   ├── registers.hpp      — Register/command enums, config struct
│   │   ├── types.hpp          — Shared types (Sample, PortConfig, etc.)
│   │   ├── spi_device.hpp     — ADS1299 SPI communication + parsing
│   │   └── controller.hpp     — Device init and control logic
│   ├── hardware/
│   │   ├── spi_bus.hpp        — Linux SPI ioctl wrapper
│   │   ├── i2c_device.hpp     — Linux I2C ioctl wrapper
│   │   └── tca9534.hpp        — TCA9534 GPIO expander driver
│   ├── acquisition/
│   │   ├── engine.hpp         — Real-time hot loop + timing instrumentation
│   │   ├── bus_worker.hpp     — Per-bus parallel SPI reader thread
│   │   └── drdy_poller.hpp    — DRDY polling via I2C
│   └── logging/
│       ├── csv_writer.hpp     — Async CSV logger
│       └── spsc_ring.hpp      — Lock-free SPSC ring buffer
└── src/
    ├── main.cpp               — Entry point, arg parsing, orchestration
    ├── ads1299/*.cpp
    ├── hardware/*.cpp
    ├── acquisition/*.cpp
    └── logging/*.cpp
```

### Key Design Decisions

- **Direct SPI ioctl** (`SPI_IOC_MESSAGE`) — bypasses spidev Python wrapper overhead
- **eventfd-triggered bus workers** — one thread per physical SPI bus (4 buses), truly parallel reads
- **SPSC lock-free ring buffer** — zero-contention handoff from acquisition to CSV writer thread
- **SCHED_FIFO 50 on core 3** — pinned to isolated core (`isolcpus=3`), `mlockall` prevents page faults
- **Zero allocation in hot loop** — all buffers pre-allocated at startup
- **Faithful init sequence** — all Python retry loops, SDATAC verification, warmup/restart preserved exactly

## Building

Build natively on the Pi (requires GCC 14+ for C++20):

```bash
# Copy source to Pi
scp -r "Cpp Implementation/" morph@192.168.1.99:~/ads1299-cpp/

# On the Pi
cd ~/ads1299-cpp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

Produces `ads1299_acquire` (~79 KB with LTO).

Compiler flags: `-std=c++20 -mcpu=cortex-a72 -O2 -fno-exceptions -fno-rtti`

## Running

```bash
# Default: 7 ports x 4 devices = 224 channels
sudo ./ads1299_acquire

# Custom ports
sudo ./ads1299_acquire --ports 0,0,Port1,4 0,1,Port2,4 3,0,Port3,4

# Timed run (auto-stop after 300 seconds)
sudo ./ads1299_acquire --duration 300

# Disable CSV output
sudo ./ads1299_acquire --no-csv
```

`sudo` is required for SCHED_FIFO real-time scheduling and SPI/I2C device access.

Output CSV: `all_channels_data.csv` in the working directory.

## Validation Results

First hardware run: **23.3 minutes, 349,369 samples, 224 channels**

| Metric | Result |
|--------|--------|
| Sample rate | 250.0 Hz (locked) |
| Gaps | 0 |
| Corruption events | 0 |
| Jitter events (>2x period) | 0 |
| dt mean / min / max | 4.000 / 1.241 / 4.660 ms |
| Normal samples | 100% |

### vs Python Baseline

| Metric | Python | C++ |
|--------|--------|-----|
| Max dt | 8.2 ms (caused corruption) | 4.66 ms |
| Typical cycle time | 2.5-3.0 ms | < 1.0 ms |
| Headroom | 1.0-1.5 ms | > 3.0 ms |
| Corruption (20+ min) | 1 event | 0 |

## Prerequisites

- Raspberry Pi 4 / CM4 with custom ADS1299 expansion board (V2)
- Kernel configured: `isolcpus=3`, DMA enabled for SPI3/4/5, HDMI audio disabled
- I2C bus 6 (bit-banged GPIO22/23) for TCA9534 expanders
- See `pi/SPI-DMA-SETUP.md` for full kernel/boot configuration

## Phase 2 (Future)

TCP streaming server with binary LZ4 framing protocol, wire-compatible with the existing PyQt5 visualizer (`client/simpleviz.py`). Will add a streaming layer on top of the validated Phase 1 acquisition engine.
