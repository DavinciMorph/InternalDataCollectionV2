# Phase 2: Add TCP Streaming to C++ ADS1299 Acquisition Engine

## Context

Phase 1 is complete and validated. The C++ acquisition engine runs at a locked 250 Hz with sub-1ms cycle times, zero corruption, and writes all 224 channels to CSV. This phase adds TCP streaming as a **parallel output path** alongside CSV.

Read the `cpp-embedded` agent file (`.claude/agents/cpp-embedded.md`) before planning or writing any code. Use it to review your architectural plan and to implement performance-critical components (binary packing, LZ4 framing, non-blocking socket I/O). The agent has deep expertise in real-time C++ on embedded Linux — lean on it for anything touching threading, lock-free data structures, or latency-sensitive code paths.

## Validated Phase 1 Baseline (Do Not Degrade)

The C++ acquisition engine has been validated on hardware. These are the measured numbers that TCP streaming must not impact:

| Metric | Measured Value |
|--------|---------------|
| Duration | 23.3 minutes continuous |
| Samples | 349,369 |
| Sample rate | 250.0 Hz (locked) |
| Corruption events | **0** |
| Jitter events (>2× period) | **0** |
| dt mean | 4.000 ms |
| dt min | 1.241 ms |
| dt max | 4.660 ms |
| Normal samples | 100% |
| Typical cycle time | < 1.0 ms |
| Headroom per cycle | > 3.0 ms |

The max dt of 4.660 ms is only 0.66 ms past the 4.0 ms sample period — the next DRDY has not been missed. For comparison, Python's 8.2 ms stall (2.05× period) missed a DRDY edge and corrupted 3 SPI buses simultaneously.

### Architecture that achieved this
- **Direct SPI ioctl**: `ioctl(SPI_IOC_MESSAGE)` — no Python wrapper overhead
- **eventfd-triggered bus workers**: One persistent thread per physical SPI bus (4 buses), `eventfd` signaling for minimal-latency wakeup
- **SPSC lock-free ring buffer**: Zero-contention sample handoff to CSV writer
- **SCHED_FIFO 50 on isolated core 3**: `mlockall(MCL_CURRENT | MCL_FUTURE)` prevents page faults
- **Zero heap allocation in hot loop**: All buffers pre-allocated at startup
- **Build**: GCC 14.2, `-std=c++20 -mcpu=cortex-a72 -O2 -fno-exceptions -fno-rtti`, LTO enabled, 79 KB binary

Any Phase 2 change that causes max dt to exceed ~7 ms (approaching the 8.2 ms corruption threshold from Python testing) is a regression and must be reverted.

## The One Rule That Cannot Be Broken

**The TCP streaming layer must NEVER cause backpressure that stalls the acquisition loop.**

This is not a theoretical concern. It was proven empirically across multiple test runs:

1. **Run #4**: A client-side Qt bug (`setStyleSheet()` called 28× every 300ms) starved the client's recv loop → TCP send buffer filled → server's `send()` blocked → acquisition thread stalled → **rate dropped from 183 Hz to 130 Hz** and corrupted SPI reads.

2. **Run #6**: After fixing the client bug, removing the server's re-sync step revealed a **periodic corruption pattern every ~23.4 seconds** (5850 samples). A periodic OS event (likely WiFi scanning or kernel timer) caused ~8ms stalls. With the Python server's ~1ms headroom, these stalls missed DRDY edges and corrupted data across multiple independent SPI buses simultaneously.

3. **Final Python validation**: A single 8.2 ms stall (2.05× the 4ms sample period) corrupted 14 channel values across 3 SPI buses in one event.

The C++ engine eliminated this class of failure by providing > 3 ms headroom per cycle. **The TCP layer must not erode that headroom.** A single non-blocking `push_sample()` to an SPSC ring buffer adds < 100 ns — this is acceptable. Any design that involves the acquisition thread waiting on a mutex, checking socket state, or touching network buffers is unacceptable.

The architecture must guarantee:
1. The acquisition thread does a **single non-blocking push** per sample — never blocks, never waits for the network
2. If the streaming consumer falls behind, **drop samples** — never apply backpressure upstream
3. If the client disconnects or the socket blocks, the acquisition loop continues without any observable timing impact
4. The TCP send path must be on a **separate thread** with its own error handling — a broken pipe or slow client must not propagate to acquisition

## What Already Exists (Phase 1 — Validated, 22 source files across 4 modules)

Review the current codebase before planning. The key components are:

- **Acquisition engine** (`acquisition/engine.hpp/.cpp`): Real-time hot loop running on SCHED_FIFO 50, pinned to isolated core 3 with `mlockall()`. DRDY poll via I2C, triggers `eventfd`-signaled bus workers for parallel SPI reads, 24-bit big-endian parsing, pushes `Sample` structs to a lock-free SPSC ring buffer consumed by the CSV writer. Cycle time < 1.0 ms measured. **Do not modify the hot loop.** You may add a second ring buffer push (to a streaming consumer), but it must be non-blocking and zero-cost when no client is connected.
- **CSV writer** (`logging/csv_writer.hpp/.cpp`): Async writer thread consuming from SPSC ring buffer with 1 MB+ write buffer. This stays as-is — CSV and TCP streaming run in parallel as independent consumers.
- **Sample struct** (`ads1299/types.hpp`): Contains timestamp (double, seconds since start), sample number (uint32), and channel data (int32 array). This is what gets pushed to consumers.
- **Bus workers** (`acquisition/bus_worker.hpp/.cpp`): One persistent `std::thread` per physical SPI bus (4 buses: SPI0, SPI3, SPI4, SPI5), woken via `eventfd` write. Ports on same bus read sequentially; buses execute truly in parallel. Do not touch.
- **SPSC ring buffer**: Bounded lock-free ring (already implemented for CSV). Reuse this same implementation for the streaming ring buffer — instantiate a second one.
- **Main** (`main.cpp`): Initialization sequence, SIGINT handling, argument parsing. Add `--host` and `--port` args here. Currently runs with `sudo` for SCHED_FIFO and SPI/I2C access.

## Protocol Specification (Wire-Compatible with Existing Python Client)

The TCP streaming protocol must be **byte-identical** to the Python server's output so the existing `simpleviz.py` client works without changes.

### Connection Handshake

1. Server listens on `host:port` (default `0.0.0.0:8888`)
2. Client connects via TCP
3. Server sends a JSON metadata line (UTF-8, newline-terminated):

```json
{
  "format": "binary_lz4",
  "batch_size": 10,
  "sample_rate": 250,
  "num_channels": 224,
  "num_devices": 28,
  "ports": ["Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"],
  "port_config": [
    {"name": "Port1", "num_devices": 4},
    {"name": "Port2", "num_devices": 4},
    {"name": "Port3", "num_devices": 4},
    {"name": "Port4", "num_devices": 4},
    {"name": "Port5", "num_devices": 4},
    {"name": "Port6", "num_devices": 4},
    {"name": "Port7", "num_devices": 4}
  ],
  "sample_size": 908,
  "sample_struct": "<dI224i"
}
```

The values must be computed dynamically from the actual port configuration (not hardcoded). `num_channels = sum(port.num_devices * 8)`, `sample_size` = 8 (float64) + 4 (uint32) + num_channels * 4 (int32), `sample_struct` = `<dI{num_channels}i`.

### Data Framing

After the metadata handshake, data flows as a stream of LZ4-compressed frames:

```
┌─────────────────────────────┐
│ Header (8 bytes)            │
│   uint32_le compressed_size │
│   uint32_le sample_count    │
├─────────────────────────────┤
│ LZ4 compressed payload      │
│   (compressed_size bytes)   │
└─────────────────────────────┘
```

The payload, when decompressed, contains `sample_count` contiguous sample records:

```
Per sample (little-endian):
  float64   timestamp         (seconds since acquisition start)
  uint32    sample_number     (monotonically increasing)
  int32[N]  channels          (N = num_channels, in port order)
```

Channel ordering: Port1 dev1 ch1-8, Port1 dev2 ch1-8, ..., Port1 dev4 ch1-8, Port2 dev1 ch1-8, ..., Port7 dev4 ch1-8.

### Batching

Accumulate `batch_size` samples (default 10) before compressing and sending. On queue timeout (100ms with no new sample), flush a partial batch so the client doesn't stall waiting for data. This matches the Python behavior.

### Compression

Use `LZ4_compress_default()` from `liblz4`. The EEG data compresses well (~4-6x) because adjacent channels have correlated values. Pre-allocate the compression output buffer to `LZ4_compressBound(batch_buffer_size)` at startup.

## Architecture

### New Files

```
include/streaming/
    server.hpp         — StreamingServer class
    protocol.hpp       — Constants, frame packing helpers
src/streaming/
    server.cpp
    protocol.cpp
```

### StreamingServer Class

```
StreamingServer
├── server_socket         (listening socket, SO_REUSEADDR)
├── client_socket         (current connected client, or -1)
├── client_mutex          (protects client_socket)
├── sample_ring           (SPSC ring buffer, acquisition → streaming)
├── streaming_thread      (packs, compresses, sends)
├── accept_thread         (listens for new connections)
├── stop_flag             (atomic<bool>)
├── stats                 (samples_sent, batches_sent, drops, reconnects)
│
├── start()               → starts accept_thread + streaming_thread
├── stop()                → signals shutdown, joins threads
├── push_sample(sample)   → non-blocking push to ring (called from acq thread)
└── is_client_connected() → atomic check
```

### Threading Model

```
                    ┌─────────────────┐
                    │  Acquisition    │  (SCHED_FIFO, core 3)
                    │  Engine         │
                    └────┬───────┬────┘
                         │       │
              non-blocking push  non-blocking push
                         │       │
                         ▼       ▼
                    ┌─────┐  ┌──────────────┐
                    │ CSV │  │ Streaming    │
                    │Ring │  │ Ring Buffer  │
                    └──┬──┘  └──────┬───────┘
                       │            │
                       ▼            ▼
                    ┌─────┐  ┌──────────────┐
                    │ CSV │  │ Streaming    │  (normal priority)
                    │Write│  │ Thread       │
                    │Thrd │  │ pack+LZ4+send│
                    └─────┘  └──────────────┘
                                    │
                                    ▼ TCP
                             ┌──────────────┐
                             │   Client     │
                             └──────────────┘
```

The streaming ring buffer is a **second instance of the existing SPSC ring buffer** (same implementation as CSV, separate instance). The acquisition thread pushes to both. If the streaming ring is full (client is slow), the push silently fails and a drop counter increments. The acquisition thread does not check `is_client_connected()` in the hot path — it always pushes, and the ring buffer absorbs/drops as needed.

Size the streaming ring to hold 5 seconds of samples (1250 entries at 250 Hz). This absorbs client-side processing stalls and network jitter without dropping samples under normal conditions. When drops do occur, the server logs the count but never stalls.

### Client Reconnection

The accept thread runs in a loop:
1. `accept()` with 1-second timeout (so it checks `stop_flag` periodically)
2. On new connection: close old client socket (if any), set new socket, send metadata JSON
3. Clear the streaming ring buffer (discard stale samples)
4. Log session start with sequence number

The streaming thread:
1. Pops samples from ring buffer (blocking wait with 100ms timeout)
2. Accumulates into batch buffer
3. When batch is full (or 100ms timeout for partial flush): pack, compress, send
4. On send failure (broken pipe, timeout): log disconnect, clear client socket, continue popping (ring buffer drains to prevent stale data on reconnect)

### Non-Blocking Send

Use a non-blocking socket (`fcntl(fd, F_SETFL, O_NONBLOCK)`) with a send loop:
- Try `send()`, handle `EAGAIN`/`EWOULDBLOCK` with short sleep (100μs) and retry
- Total timeout: 2 seconds — if a frame can't be sent in 2s, the client is dead
- On timeout or error: disconnect, don't retry the frame

Set `TCP_NODELAY` (disable Nagle) and `SO_SNDBUF = 262144` (256KB send buffer) to match the Python configuration.

### Integration with Acquisition Engine

The acquisition engine needs minimal changes:

1. Add a `StreamingServer*` pointer (nullable) to the engine
2. After pushing to the CSV ring buffer, also push to the streaming ring buffer:
   ```cpp
   // In hot loop, after csv_ring.try_push(sample):
   if (streaming_server_) {
       streaming_server_->push_sample(sample);  // non-blocking, drops if full
   }
   ```
3. That's it. No other changes to the acquisition engine.

### Integration with Main

In `main.cpp`:
1. Add `--host` (default `0.0.0.0`) and `--port` (default `8888`) arguments
2. After acquisition engine is constructed but before `run()`:
   - Create `StreamingServer` with the SPSC ring buffer
   - Pass it to the acquisition engine
   - Call `server.start()` (launches accept + streaming threads)
3. On shutdown: `server.stop()` after acquisition stops

## Dependencies

Add to Phase 1:
- **LZ4**: `liblz4-dev` (`apt install liblz4-dev`), link with `-llz4`
- **CMake**: Add `find_package` or direct `-llz4` link

No other new dependencies.

## Validation Checklist

### Functional
- [ ] Existing `simpleviz.py` client connects and receives data without modification
- [ ] Metadata JSON matches Python server output for same port configuration (field names, types, computed values)
- [ ] Sample data in TCP stream matches CSV output (same timestamps, same sample numbers, same channel values — compare first 1000 samples)
- [ ] Channel ordering in TCP stream: Port1 dev1 ch1-8, Port1 dev2 ch1-8, ..., Port7 dev4 ch1-8 (matches Python ordering)
- [ ] Client disconnect + reconnect works cleanly (no stale data from previous session, no crash)
- [ ] Multiple disconnect/reconnect cycles work (10+ cycles, session counter increments correctly)
- [ ] Server runs indefinitely with no client connected (CSV-only mode, no errors, no resource leaks)
- [ ] Partial batch flush works: disconnect client, reconnect — first data arrives within 100ms (not waiting for full batch of 10)

### Performance (the critical ones — compared against validated Phase 1 baseline)
- [ ] dt mean remains 4.000 ms (±0.001 ms)
- [ ] dt max stays below 5.0 ms (Phase 1 achieved 4.660 ms — TCP must not add latency)
- [ ] dt max never exceeds 7.0 ms (hard ceiling — 8.2 ms caused corruption in Python)
- [ ] Cycle time remains < 1.0 ms mean (the `push_sample()` call adds negligible overhead)
- [ ] Zero corruption over 20-minute run with client connected and streaming
- [ ] Zero corruption when client disconnects mid-stream
- [ ] Zero corruption when client reconnects repeatedly (10 cycles)
- [ ] Simulated slow client (add 50ms delay in client recv loop) causes sample drops in server stats but zero corruption and zero dt impact

### Backpressure Isolation
- [ ] Kill the client process with `kill -9` during streaming → server logs disconnect, acquisition continues, no corruption
- [ ] Suspend the client with `kill -STOP` for 5 seconds → server drops samples, logs drops, acquisition unaffected, client resumes on `kill -CONT`
- [ ] Connect client over a bandwidth-limited link (use `tc` to limit to 1 Mbps) → server drops samples gracefully, no acquisition impact

## What NOT to Change

- The acquisition hot loop (DRDY polling, SPI reads, parsing)
- The bus worker threading model
- The CSV writer
- The initialization sequence
- Any timing or retry logic in the controller
- Compiler flags or RT scheduling configuration

## Statistics Reporting

On client disconnect and on shutdown, print:
```
Session #N: X samples sent, Y batches, Z drops (D% drop rate)
  Mean batch compress: Wμs, mean send: Vμs
  Client connected: Ts
```

On periodic stats (every 10s), add streaming stats to existing timing report:
```
  Streaming: X sent, Y drops, Z queued | client: connected/disconnected
```
