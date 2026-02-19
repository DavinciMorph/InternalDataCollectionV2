# Phase B2: Core Reallocation + RT Hot Loop Cleanup

## Summary

Relocate all non-RT threads (CSV writer, TCP streaming, stats display) to core 1, isolate
cores 2 and 3 for RT-only work (SPI4/5 bus workers and the acquisition engine respectively),
remove `printf()` from the RT hot loop, and harden the kernel/OS configuration for reduced
jitter on the isolated cores.

## New Core Allocation

```
Core 0 - SPI0 bus worker (FIFO 49) + SPI0 DMA IRQs
Core 1 - SPI3 bus worker (FIFO 49) + ALL non-RT threads (CSV, stream, accept, stats) + all other IRQs
Core 2 - SPI4 + SPI5 bus workers (FIFO 49) only (ISOLATED) + SPI4/5 DMA IRQs
Core 3 - Acquisition engine (FIFO 50) only (ISOLATED)
```

---

## Change 1: Remove `print_stats()` from RT Hot Loop (engine.cpp + engine.hpp)

### Problem

`engine.cpp` line 216-219 calls `print_stats()` every 10 seconds from the SCHED_FIFO 50
hot loop on isolated core 3. `std::printf()` is a blocking syscall that can stall for
hundreds of microseconds (or milliseconds if the terminal is slow). This is incompatible
with a RT hot loop that must never exceed ~3ms cycle time.

### Solution

Replace inline `print_stats()` with an atomic flag (`stats_ready_`) that a separate stats
thread on core 1 polls. The engine sets the flag; the stats thread reads stats via the
existing `get_stats()` method and prints them. The engine's `print_stats()` method is
removed entirely.

### File: `Cpp Implementation/include/acquisition/engine.hpp`

**Change 1a: Add `stats_ready_` atomic flag and `stats_requested_` flag**

BEFORE (lines 80-86):
```cpp
    // Stats printing interval
    static constexpr double STATS_INTERVAL_SEC = 10.0;
    double last_stats_time_;

    void print_stats();
    static double clock_now();
};
```

AFTER:
```cpp
    // Stats printing interval
    static constexpr double STATS_INTERVAL_SEC = 10.0;
    double last_stats_time_;

    // Atomic flag: engine sets to true every STATS_INTERVAL_SEC.
    // External stats thread reads get_stats() and clears the flag.
    alignas(64) std::atomic<bool> stats_ready_{false};

    static double clock_now();
};
```

Summary of header changes:
- Remove `void print_stats();` declaration (line 84)
- Add `alignas(64) std::atomic<bool> stats_ready_{false};` in its place
- The `alignas(64)` prevents false sharing with adjacent members

### File: `Cpp Implementation/src/acquisition/engine.cpp`

**Change 1b: Remove the `print_stats()` method definition entirely**

REMOVE (lines 68-92):
```cpp
void AcquisitionEngine::print_stats() {
    double elapsed = clock_now() - start_time_;
    double rate = total_samples_ > 0 ? total_samples_ / elapsed : 0;
    double mean = total_samples_ > 0 ? cycle_sum_ms_ / total_samples_ : 0;

    std::printf("\n  [%6.0fs] samples=%lu rate=%.1fHz cycle=%.2f/%.2f/%.2fms "
                "dt=%.2f/%.2fms timeouts=%lu corrupt=%lu drops=%lu\n",
                elapsed,
                static_cast<unsigned long>(total_samples_),
                rate, min_cycle_ms_, mean, max_cycle_ms_,
                min_dt_ms_, max_dt_ms_,
                static_cast<unsigned long>(drdy_timeouts_),
                static_cast<unsigned long>(corruption_count_),
                static_cast<unsigned long>(drop_count_));

    if (streaming_) {
        auto ss = streaming_->get_stats();
        std::printf("           stream: sent=%lu batches=%lu drops=%lu queued=%zu %s\n",
                    static_cast<unsigned long>(ss.samples_sent),
                    static_cast<unsigned long>(ss.batches_sent),
                    static_cast<unsigned long>(ss.drops),
                    ss.ring_queued,
                    ss.connected ? "[connected]" : "[no client]");
    }
}
```

This entire method is deleted. No replacement in engine.cpp -- the stats thread in main.cpp
will handle printing.

**Change 1c: Replace `print_stats()` call in hot loop with atomic flag set**

BEFORE (lines 215-219):
```cpp
        // 7. Print stats every 10 seconds
        if (cycle_end - last_stats_time_ >= STATS_INTERVAL_SEC) {
            print_stats();
            last_stats_time_ = cycle_end;
        }
```

AFTER:
```cpp
        // 7. Signal stats thread every 10 seconds (zero syscalls on RT core)
        if (cycle_end - last_stats_time_ >= STATS_INTERVAL_SEC) {
            stats_ready_.store(true, std::memory_order_release);
            last_stats_time_ = cycle_end;
        }
```

**Change 1d: Replace `print_stats()` call at shutdown with atomic flag set**

BEFORE (lines 222-223):
```cpp
    // Final stats on shutdown
    print_stats();
```

AFTER:
```cpp
    // Signal final stats
    stats_ready_.store(true, std::memory_order_release);
```

**Change 1e: Remove `#include "streaming/server.hpp"` from engine.cpp**

The `print_stats()` method was the only reason engine.cpp needed to include `streaming/server.hpp`
(to call `streaming_->get_stats()`). With `print_stats()` removed, this include is no longer
needed.

BEFORE (line 6):
```cpp
#include "streaming/server.hpp"
```

AFTER:
```cpp
// (line removed)
```

Note: `engine.hpp` forward-declares `StreamingServer` and the engine only calls
`streaming_->push_sample()` which is declared inline in `streaming/server.hpp`.
However, `push_sample()` is called from engine.cpp line 196-197, so we need to check
whether the forward declaration is sufficient or if the full header is needed.

Looking at `engine.hpp` line 29: `void set_streaming_server(StreamingServer* s) { streaming_ = s; }`
and `engine.hpp` line 61: `StreamingServer* streaming_ = nullptr;` -- both only use a pointer,
which works with a forward declaration.

But `engine.cpp` line 196 calls `streaming_->push_sample(sample)`, which requires the full
class definition. So `engine.cpp` still needs the include. Let me re-check...

Actually, looking again at `streaming/server.hpp` line 26-30, `push_sample()` is defined
inline in the header. Since `engine.cpp` needs to call `push_sample()`, it needs the full
class definition from `streaming/server.hpp`. **Keep the include.**

Correction: **Do NOT remove `#include "streaming/server.hpp"` from engine.cpp.** It is still
needed for the `streaming_->push_sample(sample)` call on line 196.

### Full engine.cpp after all changes

For clarity, here is the complete `engine.cpp` after changes 1b, 1c, and 1d:

```cpp
#include "acquisition/engine.hpp"
#include "acquisition/bus_worker.hpp"
#include "acquisition/drdy_poller.hpp"
#include "ads1299/spi_device.hpp"
#include "ads1299/registers.hpp"
#include "streaming/server.hpp"

#include <cstdio>
#include <cstring>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <float.h>

namespace ads1299 {

double AcquisitionEngine::clock_now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

AcquisitionEngine::AcquisitionEngine(
    std::vector<ADS1299Device*>& devices,
    std::vector<BusWorker*>& workers,
    DRDYPoller& drdy_poller,
    SPSCRing<Sample>& ring)
    : devices_(devices)
    , workers_(workers)
    , drdy_poller_(drdy_poller)
    , ring_(ring)
    , num_ports_(static_cast<int>(devices.size()))
    , total_channels_(0)
    , total_samples_(0)
    , drdy_timeouts_(0)
    , corruption_count_(0)
    , drop_count_(0)
    , cycle_sum_ms_(0)
    , min_cycle_ms_(1e9)
    , max_cycle_ms_(0)
    , min_dt_ms_(1e9)
    , max_dt_ms_(0)
    , start_time_(0)
    , last_sample_time_(0)
    , last_stats_time_(0)
{
    std::memset(port_map_, 0, sizeof(port_map_));

    // Compute total channels and build port-to-worker mapping
    for (int p = 0; p < num_ports_; ++p) {
        total_channels_ += devices_[p]->config().num_devices * CHANNELS_PER_DEVICE;

        // Find which worker owns this port
        for (size_t w = 0; w < workers_.size(); ++w) {
            for (int wp = 0; wp < workers_[w]->port_count(); ++wp) {
                if (workers_[w]->device(wp) == devices_[p]) {
                    port_map_[p].worker_idx = static_cast<int>(w);
                    port_map_[p].port_idx_in_worker = wp;
                }
            }
        }
    }
}

void AcquisitionEngine::run(volatile sig_atomic_t& running) {
    // --- RT Hardening ---

    // Lock all pages -- prevents page faults in hot loop
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Pre-fault stack (128KB)
    volatile char stack_prefault[128 * 1024];
    for (size_t i = 0; i < sizeof(stack_prefault); i += 4096) {
        stack_prefault[i] = 0;
    }

    // Prevent deep C-states (keep /dev/cpu_dma_latency open for process lifetime)
    int pm_fd = ::open("/dev/cpu_dma_latency", O_WRONLY);
    if (pm_fd >= 0) {
        int32_t zero = 0;
        ::write(pm_fd, &zero, sizeof(zero));
        // Keep fd open -- closing would re-enable deep C-states
    }

    // Pin to isolated core 3
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    // Set SCHED_FIFO priority 50
    struct sched_param param{};
    param.sched_priority = 50;
    sched_setscheduler(0, SCHED_FIFO, &param);

    std::printf("  Engine: core 3, SCHED_FIFO 50, mlockall, %d channels\n", total_channels_);

    // Pre-allocated sample for the hot loop
    Sample sample;
    std::memset(&sample, 0, sizeof(sample));
    sample.num_channels = static_cast<uint16_t>(total_channels_);

    start_time_ = clock_now();
    last_stats_time_ = start_time_;
    last_sample_time_ = 0;

    PortData port_data;

    while (running) {
        // 1. Poll DRDY via I2C (8ms timeout = corruption threshold)
        if (!drdy_poller_.poll(0.008)) {
            drdy_timeouts_++;
            continue;
        }

        double cycle_start = clock_now();

        // 2. Trigger all bus workers in parallel
        for (auto* w : workers_) {
            w->trigger();
        }

        // 3. Wait for all bus workers to complete
        for (auto* w : workers_) {
            w->wait_done();
        }

        // 4. Parse raw data into Sample struct
        double timestamp = cycle_start - start_time_;
        sample.timestamp = timestamp;
        sample.sample_number = static_cast<uint32_t>(total_samples_);
        sample.valid = true;

        int ch_idx = 0;
        for (int p = 0; p < num_ports_; ++p) {
            auto& pm = port_map_[p];
            auto* worker = workers_[pm.worker_idx];
            uint8_t* raw = worker->raw_buffer(pm.port_idx_in_worker);
            int num_dev = devices_[p]->config().num_devices;

            ADS1299Device::parse_raw(raw, num_dev, port_data);

            // Status byte validation (ALL devices in chain)
            for (int d = 0; d < num_dev; ++d) {
                if ((port_data.status_bytes[d][0] & 0xF0) != 0xC0) {
                    corruption_count_++;
                    sample.valid = false;
                    break;  // One bad device invalidates entire sample
                }
            }

            // Copy channel values into sample
            for (int d = 0; d < num_dev; ++d) {
                for (int c = 0; c < CHANNELS_PER_DEVICE; ++c) {
                    sample.channels[ch_idx++] = port_data.channel_values[d][c];
                }
            }
        }

        // 5. Non-blocking push to SPSC ring buffer (CSV)
        if (!ring_.try_push(sample)) {
            drop_count_++;
        }

        // 5b. Non-blocking push to streaming ring buffer (TCP)
        if (streaming_) {
            streaming_->push_sample(sample);
        }

        // 6. Timing instrumentation
        double cycle_end = clock_now();
        double cycle_ms = (cycle_end - cycle_start) * 1000.0;
        cycle_sum_ms_ += cycle_ms;
        if (cycle_ms < min_cycle_ms_) min_cycle_ms_ = cycle_ms;
        if (cycle_ms > max_cycle_ms_) max_cycle_ms_ = cycle_ms;

        if (last_sample_time_ > 0) {
            double dt_ms = (cycle_start - last_sample_time_) * 1000.0;
            if (dt_ms < min_dt_ms_) min_dt_ms_ = dt_ms;
            if (dt_ms > max_dt_ms_) max_dt_ms_ = dt_ms;
        }
        last_sample_time_ = cycle_start;

        total_samples_++;

        // 7. Signal stats thread every 10 seconds (zero syscalls on RT core)
        if (cycle_end - last_stats_time_ >= STATS_INTERVAL_SEC) {
            stats_ready_.store(true, std::memory_order_release);
            last_stats_time_ = cycle_end;
        }
    }

    // Signal final stats
    stats_ready_.store(true, std::memory_order_release);

    if (pm_fd >= 0) {
        ::close(pm_fd);
    }
}

AcquisitionEngine::Stats AcquisitionEngine::get_stats() const {
    Stats s{};
    s.total_samples = total_samples_;
    s.drdy_timeouts = drdy_timeouts_;
    s.corruption_count = corruption_count_;
    s.drop_count = drop_count_;
    s.min_cycle_ms = min_cycle_ms_;
    s.max_cycle_ms = max_cycle_ms_;
    s.mean_cycle_ms = total_samples_ > 0 ? cycle_sum_ms_ / total_samples_ : 0;
    s.min_dt_ms = min_dt_ms_;
    s.max_dt_ms = max_dt_ms_;
    s.runtime_sec = clock_now() - start_time_;
    return s;
}

} // namespace ads1299
```

### Full engine.hpp after all changes

```cpp
#pragma once

#include "ads1299/types.hpp"
#include "logging/spsc_ring.hpp"

#include <atomic>
#include <cstdint>
#include <signal.h>
#include <vector>

namespace ads1299 {

class ADS1299Device;
class BusWorker;
class DRDYPoller;
class StreamingServer;

// Real-time acquisition hot loop.
// Pins to core 3, sets SCHED_FIFO priority 50, zero heap allocation in hot loop.
// Polls DRDY, triggers parallel bus workers, parses data, pushes to SPSC ring.
class AcquisitionEngine {
public:
    AcquisitionEngine(std::vector<ADS1299Device*>& devices,
                      std::vector<BusWorker*>& workers,
                      DRDYPoller& drdy_poller,
                      SPSCRing<Sample>& ring);

    // Set optional TCP streaming server (called before run())
    void set_streaming_server(StreamingServer* s) { streaming_ = s; }

    // Run the hot loop until running flag is cleared (by signal handler)
    void run(volatile sig_atomic_t& running);

    // Statistics
    struct Stats {
        uint64_t total_samples;
        uint64_t drdy_timeouts;
        uint64_t corruption_count;
        uint64_t drop_count;
        double   min_cycle_ms;
        double   max_cycle_ms;
        double   mean_cycle_ms;
        double   min_dt_ms;
        double   max_dt_ms;
        double   runtime_sec;
    };

    Stats get_stats() const;

    // Atomic flag: set by engine every STATS_INTERVAL_SEC.
    // External stats thread checks this, reads get_stats(), and clears it.
    // Public so the stats thread in main.cpp can access it directly.
    alignas(64) std::atomic<bool> stats_ready_{false};

private:
    // Build mapping from port index to (worker_index, port_within_worker)
    struct PortWorkerMap {
        int worker_idx;
        int port_idx_in_worker;
    };

    std::vector<ADS1299Device*>& devices_;
    std::vector<BusWorker*>& workers_;
    DRDYPoller& drdy_poller_;
    SPSCRing<Sample>& ring_;
    StreamingServer* streaming_ = nullptr;

    PortWorkerMap port_map_[MAX_PORTS];
    int num_ports_;
    int total_channels_;

    // Timing instrumentation
    uint64_t total_samples_;
    uint64_t drdy_timeouts_;
    uint64_t corruption_count_;
    uint64_t drop_count_;
    double   cycle_sum_ms_;
    double   min_cycle_ms_;
    double   max_cycle_ms_;
    double   min_dt_ms_;
    double   max_dt_ms_;
    double   start_time_;
    double   last_sample_time_;

    // Stats printing interval
    static constexpr double STATS_INTERVAL_SEC = 10.0;
    double last_stats_time_;

    static double clock_now();
};

} // namespace ads1299
```

Note: `stats_ready_` is moved to the **public** section so the stats thread in `main.cpp`
can access it directly without adding a friend or getter method. This is intentional --
it is a simple atomic flag with clear semantics. The alternative would be adding a
`bool check_stats_ready()` method, but that adds indirection for no safety benefit since
the field is already `std::atomic<bool>`.

---

## Change 2: Move CSV Writer from Core 2 to Core 1 (csv_writer.cpp)

### File: `Cpp Implementation/src/logging/csv_writer.cpp`

**Change 2a: Update CPU_SET from 2 to 1**

BEFORE (lines 101-106):
```cpp
    // Pin CSV writer to core 2 (housekeeping core)
    // Keeps SD card I/O off SPI0 (core 0) and SPI3 (core 1) worker cores
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

AFTER:
```cpp
    // Pin CSV writer to core 1 (housekeeping core, shared with SPI3 worker + streaming)
    // Core 2 is now isolated for SPI4/SPI5 bus workers only
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

This is a single-line change: `CPU_SET(2, &cpuset)` becomes `CPU_SET(1, &cpuset)`, plus
an updated comment.

---

## Change 3: Move Streaming Server Threads from Core 2 to Core 1 (server.cpp)

### File: `Cpp Implementation/src/streaming/server.cpp`

**Change 3a: Update accept_loop CPU_SET from 2 to 1**

BEFORE (lines 168-173):
```cpp
// Accept thread: pinned core 2 (housekeeping core, shared with stream thread + CSV writer)
void StreamingServer::accept_loop() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

AFTER:
```cpp
// Accept thread: pinned core 1 (housekeeping core, shared with stream thread + CSV writer)
void StreamingServer::accept_loop() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

**Change 3b: Update stream_loop CPU_SET from 2 to 1**

BEFORE (lines 226-231):
```cpp
// Streaming thread: pinned core 2, SCHED_OTHER
void StreamingServer::stream_loop() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

AFTER:
```cpp
// Streaming thread: pinned core 1, SCHED_OTHER
void StreamingServer::stream_loop() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

---

## Change 4: Add Stats Display Thread in main.cpp

### File: `Cpp Implementation/src/main.cpp`

**Change 4a: Add stats thread between engine creation and engine.run()**

The stats thread will be added right after the engine is constructed and before `engine.run()`
is called. It runs on core 1, polling `engine.stats_ready_` every second.

BEFORE (lines 574-587):
```cpp
    ads1299::AcquisitionEngine engine(devices, workers, drdy_poller, *ring);
    engine.set_streaming_server(&streaming_server);

    // If duration specified, set up a timer thread
    if (args.duration_sec > 0) {
        std::printf("  Auto-stop after %d seconds\n", args.duration_sec);
        std::thread timer_thread([&]() {
            sleep_sec(args.duration_sec);
            g_running = 0;
        });
        timer_thread.detach();
    }

    engine.run(g_running);
```

AFTER:
```cpp
    ads1299::AcquisitionEngine engine(devices, workers, drdy_poller, *ring);
    engine.set_streaming_server(&streaming_server);

    // If duration specified, set up a timer thread
    if (args.duration_sec > 0) {
        std::printf("  Auto-stop after %d seconds\n", args.duration_sec);
        std::thread timer_thread([&]() {
            sleep_sec(args.duration_sec);
            g_running = 0;
        });
        timer_thread.detach();
    }

    // Stats display thread: pinned to core 1 (housekeeping core)
    // Polls engine.stats_ready_ flag every 1s, prints stats when signaled.
    // Replaces the old print_stats() call that was inside the RT hot loop.
    std::thread stats_thread([&]() {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

        while (g_running) {
            // Sleep 1 second between checks (does not need to be precise)
            sleep_sec(1.0);

            if (engine.stats_ready_.exchange(false, std::memory_order_acquire)) {
                auto s = engine.get_stats();
                double rate = s.runtime_sec > 0 ? s.total_samples / s.runtime_sec : 0;

                std::printf("\n  [%6.0fs] samples=%lu rate=%.1fHz cycle=%.2f/%.2f/%.2fms "
                            "dt=%.2f/%.2fms timeouts=%lu corrupt=%lu drops=%lu\n",
                            s.runtime_sec,
                            static_cast<unsigned long>(s.total_samples),
                            rate, s.min_cycle_ms, s.mean_cycle_ms, s.max_cycle_ms,
                            s.min_dt_ms, s.max_dt_ms,
                            static_cast<unsigned long>(s.drdy_timeouts),
                            static_cast<unsigned long>(s.corruption_count),
                            static_cast<unsigned long>(s.drop_count));

                auto ss = streaming_server.get_stats();
                std::printf("           stream: sent=%lu batches=%lu drops=%lu queued=%zu %s\n",
                            static_cast<unsigned long>(ss.samples_sent),
                            static_cast<unsigned long>(ss.batches_sent),
                            static_cast<unsigned long>(ss.drops),
                            ss.ring_queued,
                            ss.connected ? "[connected]" : "[no client]");
            }
        }
    });

    engine.run(g_running);

    // Join stats thread after engine.run() returns (g_running is 0)
    if (stats_thread.joinable()) {
        stats_thread.join();
    }

    // Print final stats one more time (engine sets stats_ready_ on exit)
    if (engine.stats_ready_.exchange(false, std::memory_order_acquire)) {
        auto s = engine.get_stats();
        double rate = s.runtime_sec > 0 ? s.total_samples / s.runtime_sec : 0;

        std::printf("\n  [%6.0fs] samples=%lu rate=%.1fHz cycle=%.2f/%.2f/%.2fms "
                    "dt=%.2f/%.2fms timeouts=%lu corrupt=%lu drops=%lu\n",
                    s.runtime_sec,
                    static_cast<unsigned long>(s.total_samples),
                    rate, s.min_cycle_ms, s.mean_cycle_ms, s.max_cycle_ms,
                    s.min_dt_ms, s.max_dt_ms,
                    static_cast<unsigned long>(s.drdy_timeouts),
                    static_cast<unsigned long>(s.corruption_count),
                    static_cast<unsigned long>(s.drop_count));

        auto ss = streaming_server.get_stats();
        std::printf("           stream: sent=%lu batches=%lu drops=%lu queued=%zu %s\n",
                    static_cast<unsigned long>(ss.samples_sent),
                    static_cast<unsigned long>(ss.batches_sent),
                    static_cast<unsigned long>(ss.drops),
                    ss.ring_queued,
                    ss.connected ? "[connected]" : "[no client]");
    }
```

Note: The `sleep_sec()` helper already exists at line 32-37 of `main.cpp`. The stats thread
uses it for its 1-second poll interval. The `std::printf` and `#include <cstdio>` are
already present in `main.cpp`.

New includes needed: `#include <pthread.h>` and `#include <sched.h>` for `CPU_SET` and
`pthread_setaffinity_np`. Checking current includes in main.cpp:

Lines 1-23 show: No `<pthread.h>` or `<sched.h>` currently included in main.cpp. They are
needed for the stats thread's CPU pinning.

**Change 4b: Add missing includes to main.cpp**

BEFORE (lines 15-23):
```cpp
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <vector>
#include <time.h>
#include <unistd.h>
```

AFTER:
```cpp
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <vector>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
```

---

## Change 5: Fix Health Check Threshold in main.cpp

### Problem

Currently (lines 437-443), ports with less than 90% valid samples but more than 0% are
kept with a `[WARN - keeping]` message. This allows partially broken ports into the
acquisition loop, producing corrupted data. Ports below 50% valid should trigger recovery,
not be silently kept.

### File: `Cpp Implementation/src/main.cpp`

BEFORE (lines 437-443):
```cpp
        } else {
            // Partially valid -- keep it, it may stabilize during acquisition
            std::printf("    %s: %.0f%% valid (%d/%d) [WARN - keeping]\n",
                        devices[p]->config().port_name, rate * 100.0,
                        final_valid[p], final_reads);
            active_ports++;
        }
```

AFTER:
```cpp
        } else if (rate >= 0.50) {
            // Majority valid -- keep it, it may stabilize during acquisition
            std::printf("    %s: %.0f%% valid (%d/%d) [WARN - keeping]\n",
                        devices[p]->config().port_name, rate * 100.0,
                        final_valid[p], final_reads);
            active_ports++;
        } else {
            // Below 50% valid -- attempt recovery
            std::printf("    %s: %.0f%% valid (%d/%d) -- attempting recovery...\n",
                        devices[p]->config().port_name, rate * 100.0,
                        final_valid[p], final_reads);
            bool recovered = false;
            for (int a = 0; a < 5 && g_running; ++a) {
                if (ads1299::ADS1299Controller::recover_port(*devices[p], a, 10)) {
                    std::printf("    %s: recovered [OK]\n", devices[p]->config().port_name);
                    recovered = true;
                    break;
                }
            }
            if (recovered) {
                active_ports++;
            } else {
                port_dead[p] = true;
                std::printf("    %s: DEAD after recovery attempt\n",
                            devices[p]->config().port_name);
                devices[p]->start_low();
                devices[p]->send_command(ads1299::Cmd::STOP);
                devices[p]->send_command(ads1299::Cmd::SDATAC);
            }
        }
```

---

## Change 6: System Configuration -- Boot Command Line

### File (on Pi): `/boot/firmware/cmdline.txt`

This change must be made directly on the Raspberry Pi.

BEFORE (current):
```
console=tty1 console=serial0,115200 root=PARTUUID=0cdd5cfe-02 rootfstype=ext4 fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles cfg80211.ieee80211_regdom=GB isolcpus=3 irqaffinity=0,2
```

AFTER:
```
console=tty1 console=serial0,115200 root=PARTUUID=0cdd5cfe-02 rootfstype=ext4 fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles cfg80211.ieee80211_regdom=GB isolcpus=2,3 irqaffinity=0,1 nohz_full=2,3 rcu_nocbs=2,3 rcu_nocb_poll
```

### What changed

| Parameter | Before | After | Why |
|-----------|--------|-------|-----|
| `isolcpus` | `3` | `2,3` | Isolate core 2 for SPI4/SPI5 bus workers |
| `irqaffinity` | `0,2` | `0,1` | Move IRQ processing off core 2 to core 1 |
| `nohz_full` | (absent) | `2,3` | Disable scheduler tick on isolated cores (reduces jitter) |
| `rcu_nocbs` | (absent) | `2,3` | Offload RCU callbacks from isolated cores |
| `rcu_nocb_poll` | (absent) | (present) | Use polling instead of wake-up for RCU callback threads |

### Command to apply

```bash
ssh morph@192.168.1.99
sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.bak
sudo nano /boot/firmware/cmdline.txt
# Replace the line with the new content (single line, no newlines)
# Then reboot:
sudo reboot
```

**IMPORTANT**: `cmdline.txt` must be a single line. No line breaks.

---

## Change 7: System Configuration -- Sysctl RT Tuning

### File (on Pi): `/etc/sysctl.d/99-eeg-acquisition.conf`

This file may or may not already exist. If it exists, replace its contents entirely. If it
does not exist, create it.

AFTER (full file contents):
```
# EEG Acquisition RT tuning
# Allow RT tasks to use 100% of CPU (no throttling)
kernel.sched_rt_runtime_us = -1

# Reduce vmstat timer frequency (less jitter on isolated cores)
vm.stat_interval = 10

# Moderate VM writeback tuning (not overly aggressive)
vm.dirty_writeback_centisecs = 1500
vm.dirty_expire_centisecs = 3000
vm.dirty_ratio = 40
vm.dirty_background_ratio = 10
```

### Command to apply

```bash
ssh morph@192.168.1.99
sudo tee /etc/sysctl.d/99-eeg-acquisition.conf << 'EOF'
# EEG Acquisition RT tuning
# Allow RT tasks to use 100% of CPU (no throttling)
kernel.sched_rt_runtime_us = -1

# Reduce vmstat timer frequency (less jitter on isolated cores)
vm.stat_interval = 10

# Moderate VM writeback tuning (not overly aggressive)
vm.dirty_writeback_centisecs = 1500
vm.dirty_expire_centisecs = 3000
vm.dirty_ratio = 40
vm.dirty_background_ratio = 10
EOF
sudo sysctl --system
```

---

## Change 8: System Configuration -- fstab noatime

### File (on Pi): `/etc/fstab`

Add `noatime,commit=30` to the root partition mount options. This reduces filesystem
metadata writes, which reduces SD card write stalls that could propagate to jitter.

### Before (typical Pi fstab):
```
PARTUUID=0cdd5cfe-01  /boot/firmware  vfat  defaults  0  2
PARTUUID=0cdd5cfe-02  /               ext4  defaults,noatime  0  1
```

Note: The Pi may already have `noatime`. Check first.

### After:
```
PARTUUID=0cdd5cfe-01  /boot/firmware  vfat  defaults  0  2
PARTUUID=0cdd5cfe-02  /               ext4  defaults,noatime,commit=30  0  1
```

### Command to apply

```bash
ssh morph@192.168.1.99
cat /etc/fstab  # Check current state first
# If noatime is already present, just add commit=30:
sudo sed -i 's|defaults,noatime|defaults,noatime,commit=30|' /etc/fstab
# If noatime is NOT present:
# sudo sed -i 's|defaults|defaults,noatime,commit=30|' /etc/fstab
# Verify:
cat /etc/fstab
# Apply immediately without reboot:
sudo mount -o remount /
```

---

## Change 9: Bus Worker Core Assignments (NO CHANGE NEEDED)

The bus worker core assignments in `main.cpp` lines 499-507:

```cpp
    auto get_core = [](int bus_num) -> int {
        switch (bus_num) {
            case 0: return 0;  // SPI0 (Port1+2): sole owner, fastest path to Port1
            case 3: return 1;
            case 4: return 2;
            case 5: return 2;  // SPI5 (Port7): shares core 2 with SPI4
            default: return 0;
        }
    };
```

These are **already correct** for the new core allocation plan:
- SPI0 -> core 0 (correct)
- SPI3 -> core 1 (correct)
- SPI4 -> core 2 (correct, now isolated)
- SPI5 -> core 2 (correct, now isolated)

No code change needed here.

---

## Summary of All File Changes

| # | File | Change | Lines Affected |
|---|------|--------|----------------|
| 1a | `include/acquisition/engine.hpp` | Make `stats_ready_` public, remove `print_stats()` decl | 80-86 |
| 1b | `src/acquisition/engine.cpp` | Remove `print_stats()` method definition | 68-92 |
| 1c | `src/acquisition/engine.cpp` | Replace `print_stats()` call with `stats_ready_.store()` | 215-219 |
| 1d | `src/acquisition/engine.cpp` | Replace shutdown `print_stats()` with `stats_ready_.store()` | 222-223 |
| 2a | `src/logging/csv_writer.cpp` | `CPU_SET(2,...)` -> `CPU_SET(1,...)` | 101-106 |
| 3a | `src/streaming/server.cpp` | accept_loop: `CPU_SET(2,...)` -> `CPU_SET(1,...)` | 168-173 |
| 3b | `src/streaming/server.cpp` | stream_loop: `CPU_SET(2,...)` -> `CPU_SET(1,...)` | 226-231 |
| 4a | `src/main.cpp` | Add stats display thread + join + final print | After line 576 |
| 4b | `src/main.cpp` | Add `#include <pthread.h>` and `#include <sched.h>` | 15-23 |
| 5 | `src/main.cpp` | Health check: add 50% threshold + recovery for low ports | 437-443 |
| 6 | Pi: `/boot/firmware/cmdline.txt` | `isolcpus=2,3 irqaffinity=0,1 nohz_full=2,3 rcu_nocbs=2,3 rcu_nocb_poll` | Entire line |
| 7 | Pi: `/etc/sysctl.d/99-eeg-acquisition.conf` | RT + VM tuning | New/replace file |
| 8 | Pi: `/etc/fstab` | Add `noatime,commit=30` | Root partition line |

---

## Build and Verification Steps

### 1. Build on Pi

```bash
ssh morph@192.168.1.99
cd ~/ads1299-cpp/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

Expected: Zero warnings with `-Wall -Wextra -Wpedantic`. Binary at `./ads1299_acquire`.

### 2. Verify system config changes

```bash
# After reboot (required for cmdline.txt changes):

# Check isolcpus
cat /sys/devices/system/cpu/isolated
# Expected: 2-3

# Check nohz_full
cat /sys/devices/system/cpu/nohz_full
# Expected: 2-3

# Check rcu_nocbs
cat /sys/devices/system/cpu/rcu_nocbs
# Expected: 2-3 (or "2,3")

# Check IRQ affinity default
cat /proc/irq/default_smp_affinity
# Expected: 3 (binary 0011 = cores 0,1)

# Check RT scheduling
sysctl kernel.sched_rt_runtime_us
# Expected: -1

# Check fstab applied
mount | grep " / "
# Expected: ...noatime,commit=30...

# Check sysctl values
sysctl vm.dirty_writeback_centisecs vm.dirty_expire_centisecs vm.dirty_ratio vm.dirty_background_ratio vm.stat_interval
```

### 3. Runtime verification

```bash
cd ~/ads1299-cpp/build
sudo ./ads1299_acquire --duration 120
```

Watch for:
- Stats output appearing every ~10 seconds (printed by stats thread, not engine)
- No jitter spikes > 4.5ms in `max_cycle_ms`
- Zero corruption events
- Rate locked at 250.0 Hz

### 4. Verify core assignments

While the acquisition is running, in another terminal:

```bash
# Check which cores threads are on
ps -eLo pid,lwp,comm,psr | grep ads1299

# Expected output (approximate):
# <pid> <lwp> ads1299_acquire  3    <- engine (core 3)
# <pid> <lwp> ads1299_acquire  0    <- SPI0 bus worker (core 0)
# <pid> <lwp> ads1299_acquire  1    <- SPI3 bus worker (core 1)
# <pid> <lwp> ads1299_acquire  2    <- SPI4 bus worker (core 2)
# <pid> <lwp> ads1299_acquire  2    <- SPI5 bus worker (core 2)
# <pid> <lwp> ads1299_acquire  1    <- CSV writer (core 1)
# <pid> <lwp> ads1299_acquire  1    <- accept thread (core 1)
# <pid> <lwp> ads1299_acquire  1    <- stream thread (core 1)
# <pid> <lwp> ads1299_acquire  1    <- stats thread (core 1)

# Check RT priorities
chrt -p <pid_of_engine_lwp>
# Expected: SCHED_FIFO priority 50

# Check no IRQs on cores 2-3
cat /proc/interrupts | awk 'NR==1 || /spi/'
# Expected: SPI4/SPI5 DMA IRQs should still land on core 2 (affinity set at DMA driver level)
# All other IRQs should be on cores 0-1 only
```

---

## Implementation Checklist

### Code Changes
- [ ] **1a**: `engine.hpp` -- Move `stats_ready_` to public, remove `print_stats()` declaration
- [ ] **1b**: `engine.cpp` -- Remove `print_stats()` method definition (lines 68-92)
- [ ] **1c**: `engine.cpp` -- Replace hot loop `print_stats()` with `stats_ready_.store()` (lines 215-219)
- [ ] **1d**: `engine.cpp` -- Replace shutdown `print_stats()` with `stats_ready_.store()` (lines 222-223)
- [ ] **2a**: `csv_writer.cpp` -- Change `CPU_SET(2, ...)` to `CPU_SET(1, ...)` (line 105)
- [ ] **3a**: `server.cpp` -- accept_loop: Change `CPU_SET(2, ...)` to `CPU_SET(1, ...)` (line 172)
- [ ] **3b**: `server.cpp` -- stream_loop: Change `CPU_SET(2, ...)` to `CPU_SET(1, ...)` (line 230)
- [ ] **4a**: `main.cpp` -- Add stats display thread (pinned core 1) + join + final print
- [ ] **4b**: `main.cpp` -- Add `#include <pthread.h>` and `#include <sched.h>`
- [ ] **5**: `main.cpp` -- Health check: add 50% threshold, trigger recovery for low ports

### System Configuration (on Pi, requires reboot)
- [ ] **6**: `/boot/firmware/cmdline.txt` -- Back up old, write new with `isolcpus=2,3`, etc.
- [ ] **7**: `/etc/sysctl.d/99-eeg-acquisition.conf` -- Create/replace with RT + VM tuning
- [ ] **8**: `/etc/fstab` -- Add `noatime,commit=30` to root partition

### Verification
- [ ] Build succeeds with zero warnings
- [ ] Binary size similar to before (~79KB)
- [ ] `cat /sys/devices/system/cpu/isolated` shows `2-3`
- [ ] `cat /sys/devices/system/cpu/nohz_full` shows `2-3`
- [ ] `sysctl kernel.sched_rt_runtime_us` shows `-1`
- [ ] `mount | grep " / "` shows `noatime,commit=30`
- [ ] 2-minute test run: rate 250.0 Hz, zero corruption, stats print every ~10s
- [ ] `ps -eLo pid,lwp,comm,psr | grep ads1299` confirms core assignments
- [ ] No jitter regression compared to Phase 2 baseline (max_cycle_ms < 4.7ms)
- [ ] Stats thread output matches format of old `print_stats()` (no UI regressions)

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Core 1 overloaded (SPI3 worker + CSV + streaming + stats + all IRQs) | SPI3 read latency increases, widening max_cycle_ms | SPI3 worker is FIFO 49, preempts all non-RT threads. CSV/streaming sleep when idle. Monitor max_cycle_ms. |
| `nohz_full` causes issues on Pi kernel | Boot failure or unexpected behavior | Backup cmdline.txt first. Can revert by removing nohz_full/rcu_nocbs parameters. |
| Stats thread 1s poll misses the 10s window | Stats printed up to 1s late | Acceptable -- stats are informational, not RT. Could reduce to 500ms if needed. |
| `stats_ready_` flag read torn across cache lines | Impossible -- `alignas(64)` ensures own cache line, `std::atomic<bool>` is always atomic | No mitigation needed. |
| fstab `commit=30` causes data loss on power failure | Up to 30s of CSV data lost if Pi loses power | Acceptable for research data collection. Can reduce to `commit=10` if concerned. |
| Health check recovery (Change 5) delays startup | Up to 1s extra per degraded port | Only fires for ports between 0-50% valid, which is rare. Better than running with bad data. |

---

## Thread Summary After Changes

| Thread | Core | Priority | Purpose |
|--------|------|----------|---------|
| Engine | 3 (isolated) | FIFO 50 | RT hot loop: DRDY poll, trigger workers, parse, push to rings |
| SPI0 worker | 0 | FIFO 49 | Read Port1 + Port2 (9+7=16 devices, tightest timing) |
| SPI3 worker | 1 | FIFO 49 | Read Port3 + Port4 (5+4=9 devices) |
| SPI4 worker | 2 (isolated) | FIFO 49 | Read Port5 + Port6 (4+5=9 devices) |
| SPI5 worker | 2 (isolated) | FIFO 49 | Read Port7 (7 devices) |
| CSV writer | 1 | SCHED_OTHER | Drain SPSC ring, format, write to SD card |
| Accept thread | 1 | SCHED_OTHER | TCP listen, accept clients |
| Stream thread | 1 | SCHED_OTHER | Drain stream ring, LZ4 compress, send to client |
| Stats thread | 1 | SCHED_OTHER | Poll engine stats_ready_, print to stdout |
| Timer thread | (any) | SCHED_OTHER | Optional: auto-stop after --duration seconds |
