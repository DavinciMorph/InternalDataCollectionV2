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

    void print_stats();
    static double clock_now();
};

} // namespace ads1299
