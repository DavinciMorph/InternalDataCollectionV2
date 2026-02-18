# ADS1299 C++ Acquisition Engine

Real-time 224-channel EEG acquisition engine for Raspberry Pi 4, with simultaneous CSV recording and live TCP streaming to the PyQt5 visualizer.

**Hardware:** 7 SPI ports, 4 daisy-chained ADS1299 per port, 28 devices, 224 channels @ 250 Hz.

## Prerequisites

Before building this code, the Pi must have the following already configured:

### Hardware
- Raspberry Pi 4 (BCM2711 / Cortex-A72) with the custom ADS1299 V2 expansion board
- 7 SPI buses wired: SPI0 (ports 1-2), SPI3 (ports 3-4), SPI4 (ports 5-6), SPI5 (port 7)
- TCA9534 I2C GPIO expanders at 0x20 (DRDY) and 0x21 (START)
- Bit-banged I2C on bus 6 (GPIO22=SDA, GPIO23=SCL) — hardware I2C pins are used by SPI3

### Kernel / Boot Config

These settings are in `/boot/firmware/config.txt` and `/boot/firmware/cmdline.txt` on the Pi. They persist across reboots. See `pi/SPI-DMA-SETUP.md` for the full setup guide.

| Setting | File | Purpose |
|---------|------|---------|
| `isolcpus=3` | cmdline.txt | Reserves CPU core 3 for the acquisition thread |
| `vc4-kms-v3d,noaudio` | config.txt | Disables HDMI audio to free DMA4 channels for SPI5 |
| SPI DMA overlays | config.txt | Enables DMA for SPI3, SPI4, SPI5 (SPI0 has DMA by default) |
| PACTL_CS mux bits | systemd service | Routes DMA DREQs to SPI4/SPI5 at boot |
| `i2c-gpio` overlay | config.txt | Bit-banged I2C on bus 6 (GPIO22/23) |

### Systemd Services (already installed on the Pi)

| Service | Purpose |
|---------|---------|
| `cpu-performance.service` | Sets CPU governor to `performance` mode |
| `irq-affinity.service` | Moves IRQ handlers off core 3 |
| `spi45-dma-mux.service` | Sets PACTL_CS DMA mux bits for SPI4/SPI5 |

### Software

| Package | How to install |
|---------|---------------|
| GCC 14+ (C++20 support) | Included in Raspberry Pi OS Bookworm |
| CMake 3.16+ | `sudo apt install cmake` |
| pkg-config | `sudo apt install pkg-config` |
| liblz4-dev | `sudo apt install liblz4-dev` |

## Setup and Build

### Step 1: Install dependencies (one time)

SSH into the Pi and install the required packages:

```bash
ssh morph@192.168.1.99
sudo apt update
sudo apt install -y cmake pkg-config liblz4-dev
```

### Step 2: Copy source files to the Pi

From the Windows development machine:

```bash
# Clean any previous build
ssh morph@192.168.1.99 'rm -rf ~/ads1299-cpp && mkdir ~/ads1299-cpp'

# Copy the contents of "Cpp Implementation/" to ~/ads1299-cpp/
scp -r "Cpp Implementation/"* morph@192.168.1.99:~/ads1299-cpp/
```

After copying, the Pi should have this structure:

```
~/ads1299-cpp/
├── CMakeLists.txt
├── include/
│   ├── acquisition/
│   │   ├── engine.hpp         — Real-time hot loop + timing instrumentation
│   │   ├── bus_worker.hpp     — Per-bus parallel SPI reader thread
│   │   └── drdy_poller.hpp    — DRDY polling via I2C
│   ├── ads1299/
│   │   ├── registers.hpp      — Register/command enums, config struct
│   │   ├── types.hpp          — Shared types (Sample, PortConfig, etc.)
│   │   ├── spi_device.hpp     — ADS1299 SPI communication + parsing
│   │   └── controller.hpp     — Device init and control logic
│   ├── hardware/
│   │   ├── spi_bus.hpp        — Linux SPI ioctl wrapper
│   │   ├── i2c_device.hpp     — Linux I2C ioctl wrapper
│   │   └── tca9534.hpp        — TCA9534 GPIO expander driver
│   ├── logging/
│   │   ├── csv_writer.hpp     — Async CSV logger thread
│   │   └── spsc_ring.hpp      — Lock-free SPSC ring buffer
│   └── streaming/
│       ├── protocol.hpp       — Wire protocol (pack, metadata JSON, send_all)
│       └── server.hpp         — TCP streaming server class
└── src/
    ├── main.cpp               — Entry point, arg parsing, orchestration
    ├── acquisition/*.cpp
    ├── ads1299/*.cpp
    ├── hardware/*.cpp
    ├── logging/*.cpp
    └── streaming/*.cpp
```

### Step 3: Build

SSH into the Pi and build:

```bash
ssh morph@192.168.1.99

cd ~/ads1299-cpp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

Expected output:

```
-- Found liblz4, version 1.10.0
-- Configuring done
-- Build files have been written to: /home/morph/ads1299-cpp/build
[  7%] Building CXX object ... src/main.cpp.o
...
[100%] Linking CXX executable ads1299_acquire
[100%] Built target ads1299_acquire
```

The binary is `~/ads1299-cpp/build/ads1299_acquire` (~80 KB with LTO).

Compiler flags: `-std=c++20 -mcpu=cortex-a72 -O2 -fno-exceptions -fno-rtti`, LTO enabled.

### Alternative: Use the build script

There is a `build_on_pi.sh` script in the repo root that automates the dependency check and build:

```bash
# From Windows:
scp build_on_pi.sh morph@192.168.1.99:~/build_on_pi.sh
ssh morph@192.168.1.99 'bash ~/build_on_pi.sh'
```

## Running

The binary requires `sudo` for SCHED_FIFO real-time scheduling and SPI/I2C device access.

```bash
cd ~/ads1299-cpp/build

# Default: 7 ports x 4 devices = 224 channels, TCP on 0.0.0.0:8888
sudo ./ads1299_acquire

# Custom TCP host/port
sudo ./ads1299_acquire --host 0.0.0.0 --port 9000

# Timed run (auto-stop after 300 seconds)
sudo ./ads1299_acquire --duration 300

# Disable CSV output (streaming only)
sudo ./ads1299_acquire --no-csv

# Custom port configuration
sudo ./ads1299_acquire --ports 0,0,Port1,4 0,1,Port2,4 3,0,Port3,4
```

### What happens on startup

1. Opens I2C bus 6 and configures TCA9534 DRDY/START pins
2. Waits 2s for VCAP1 charging
3. Configures all 28 ADS1299 devices (software reset, register writes with retry)
4. Starts conversions with synchronized START
5. Verifies DRDY signals and data flow on all ports, restarts any that return zeros
6. Runs 500-sample warmup, restarts ports that produce zeros during warmup
7. Begins real-time acquisition on core 3 (SCHED_FIFO priority 50)
8. Writes CSV to `all_channels_data.csv` in the working directory
9. Listens for TCP client connections on the configured port

### Connecting the visualizer

From the Windows client machine:

```bash
cd client/
uv run simpleviz.py --host 192.168.1.99
```

The visualizer auto-detects the port configuration from the server metadata. No client changes are needed.

### Stopping

Press `Ctrl+C`. The engine prints final statistics and shuts down cleanly:

```
Samples:    308491
Runtime:    1234.6 s
Rate:       249.9 Hz
Cycle time: 0.35 / 0.36 / 0.73 ms (min/mean/max)
Corruptions:    0
Ring drops:     0
Stream sent:    307150 (batches: 30715, drops: 0, reconnects: 1)
```

### Command-line options

| Option | Default | Description |
|--------|---------|-------------|
| `--host ADDR` | `0.0.0.0` | TCP listen address |
| `--port PORT` | `8888` | TCP listen port |
| `--ports ...` | 7-port default | Custom port config (format: `bus,device,name,num_daisy`) |
| `--full-csv` | enabled | Write all channels to CSV |
| `--no-csv` | — | Disable CSV output |
| `--duration SEC` | infinite | Auto-stop after N seconds |
| `-h`, `--help` | — | Show usage |

## Architecture

### Threading Model

| Thread | Core | Priority | Role |
|--------|------|----------|------|
| Acquisition | 3 (isolated) | SCHED_FIFO 50 | Polls DRDY, triggers bus workers, parses data, pushes to rings |
| Bus Worker x4 | 0, 1, 2, 0 | SCHED_OTHER | One per SPI bus, reads raw bytes via ioctl, signaled by eventfd |
| CSV Writer | any | SCHED_OTHER | Pops from CSV SPSC ring, formats and writes to file |
| Stream Accept | 1 | SCHED_OTHER | Listens for TCP connections, hands off fd via atomic |
| Stream Send | 2 | SCHED_OTHER | Pops from streaming SPSC ring, LZ4 compresses, sends framed data |

### Data Flow

```
DRDY (I2C) ──> Acquisition Thread ──> Bus Workers (parallel SPI reads)
                       │
                       ├──> CSV SPSC Ring (2048) ──> CSV Writer ──> all_channels_data.csv
                       │
                       └──> Stream SPSC Ring (2048) ──> Stream Thread ──> LZ4 ──> TCP client
```

Each ring is independent. A slow client or full disk cannot cause backpressure on the acquisition loop — samples are dropped from the affected ring only.

### Wire Protocol (TCP)

Metadata (sent once per connection, newline-terminated JSON):
```json
{"format":"binary_lz4","batch_size":10,"sample_rate":250,"num_channels":224,"num_devices":28,"ports":["Port1",...,"Port7"],"port_config":[{"name":"Port1","num_devices":4},...],"sample_size":908,"sample_struct":"<dI224i"}
```

Data frames (continuous):
```
[uint32_le compressed_size][uint32_le sample_count][LZ4 frame]
```

Each sample in the LZ4 payload: `[float64_le timestamp][uint32_le sample_number][int32_le x 224 channels]` = 908 bytes.

## Validation

### Phase 2 Run (20.6 minutes, with TCP client connected)

| Metric | Result |
|--------|--------|
| Duration | 1234.6 s (20.6 min) |
| Samples | 308,491 |
| Sample rate | 250.0 Hz |
| Corruption events | 0 |
| DRDY timeouts | 0 |
| CSV ring drops | 0 |
| Stream ring drops | 0 |
| Stream samples sent | 307,150 |
| Cycle time (min/mean/max) | 0.35 / 0.36 / 0.73 ms |
| Sample dt (min/max) | 1.89 / 4.25 ms |

### vs Python Server

| Metric | Python | C++ |
|--------|--------|-----|
| Max cycle time | 2.5-3.0 ms | 0.73 ms |
| Headroom per cycle | 1.0-1.5 ms | > 3.0 ms |
| Client receive rate | ~108 Hz | 250 Hz (full rate) |
| Corruption (20+ min) | 1 event (8.2ms stall) | 0 |
| Stream drops | Frequent | 0 |

## Troubleshooting

### Build fails: "liblz4 not found"
```bash
sudo apt install liblz4-dev
```

### Build fails: CMake version too old
```bash
sudo apt install cmake
```
Requires CMake 3.16+. Raspberry Pi OS Bookworm ships 3.25+.

### "Permission denied" or "Operation not permitted"
Run with `sudo`. The binary needs root for SCHED_FIFO, mlockall, and /dev/spidev*/i2c* access.

### Ports return all zeros on startup
This is a known intermittent issue with the V2 board. The init sequence automatically retries and restarts affected ports. If a port stays at all-zeros after init, stop and restart the binary. Multiple restarts may be needed.

### "Failed to open I2C bus"
Check that the bit-banged I2C overlay is loaded:
```bash
ls /dev/i2c-6
```
If missing, verify `dtoverlay=i2c-gpio,bus=6,i2c_gpio_sda=22,i2c_gpio_scl=23` is in `/boot/firmware/config.txt`.

### "Failed to open SPI"
Check that SPI buses are enabled:
```bash
ls /dev/spidev*
```
Expected: `spidev0.0 spidev0.1 spidev3.0 spidev3.1 spidev4.0 spidev4.1 spidev5.0`

### Client can't connect
- Verify the Pi's IP: `hostname -I`
- Check the binary is running and shows `[STREAM] Listening on 0.0.0.0:8888`
- Check firewall: `sudo iptables -L` (should have no DROP rules on port 8888)
