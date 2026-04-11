#pragma once

#include "ads1299/types.hpp"
#include "logging/spsc_ring.hpp"

#include <atomic>
#include <climits>
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

    // Enable/disable CSV ring buffer push in hot loop.
    // When disabled, the SPSC ring push is skipped entirely (saves ~610ns/cycle
    // and avoids inflating drop_count when no consumer is draining the ring).
    void set_csv_enabled(bool enabled) { csv_enabled_ = enabled; }

    // Set the channel index for supply monitoring (--monitor-supply).
    // When >= 0, the engine tracks this channel's value in the stats output.
    // channel_idx is the global channel index in the sample (0-based).
    void set_supply_monitor_channel(int channel_idx) { supply_channel_idx_ = channel_idx; }

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
        // B.1: DC offset watchdog
        bool     offset_alert;
        int      offset_alert_count;     // number of channels currently exceeding threshold
        int32_t  max_offset;             // largest |current - baseline| across all channels
        double   offset_alert_onset;     // timestamp of alert onset (0 = no alert active)
        // B.1b: Per-port mean offset from baseline (integer, in LSB)
        int32_t  port_offset[MAX_PORTS];
        int      num_offset_ports;
        // B.6: Supply voltage monitor (MVDD channel, if configured)
        bool     supply_monitor_active;
        int32_t  supply_value;           // Latest raw ADC code from MVDD channel
        int32_t  supply_min;             // Min since last stats read
        int32_t  supply_max;             // Max since last stats read
        // B.5: Status byte tracking (first device per port)
        uint8_t  status_bytes[MAX_PORTS];
        int      num_status_ports;
    };

    Stats get_stats() const;

    // Atomic flag: set by engine every STATS_INTERVAL_SEC.
    // External stats thread checks this, reads get_stats(), and clears it.
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
    bool csv_enabled_ = false;

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

    // B.1: DC offset watchdog — all pre-allocated, zero-alloc in hot loop
    static constexpr uint32_t BASELINE_WINDOW = 2500;   // 10s at 250 Hz
    static constexpr int32_t  OFFSET_THRESHOLD = 500000; // ~11 mV at gain=24
    int64_t  channel_running_sum_[MAX_TOTAL_CHANNELS]{};
    int32_t  channel_baseline_[MAX_TOTAL_CHANNELS]{};
    uint32_t baseline_samples_ = 0;
    bool     baseline_locked_ = false;
    bool     offset_alert_ = false;
    int      offset_alert_count_ = 0;
    int32_t  max_offset_ = 0;

    // B.1b: Per-port mean offset tracking (for diagnosing global vs per-port shifts)
    //   Stores running exponential moving average of mean channel value per port.
    //   EMA coefficient alpha = 1/64 (shift-based, no division in hot loop).
    //   port_mean_ema_[p] is Q8 fixed-point (shifted left 8 to avoid float).
    int64_t  port_mean_ema_[MAX_PORTS]{};       // EMA of mean channel value per port (Q8)
    int64_t  port_baseline_ema_[MAX_PORTS]{};   // Baseline snapshot of EMA per port (Q8)
    bool     port_baseline_set_ = false;

    // B.1c: Alert onset tracking
    double   offset_alert_onset_time_ = 0.0;    // Timestamp when alert first triggered (0 = no alert)
    bool     offset_alert_prev_ = false;        // Previous cycle's alert state (for edge detection)

    // B.5: Status byte tracking — first device per port
    uint8_t  last_status_bytes_[MAX_PORTS]{};

    // B.6: Supply voltage monitor — tracks MVDD channel value
    int      supply_channel_idx_ = -1;  // Global channel index, -1 = disabled
    int32_t  supply_value_ = 0;         // Latest raw value
    int32_t  supply_min_ = INT32_MAX;   // Min since last stats read
    int32_t  supply_max_ = INT32_MIN;   // Max since last stats read

    static double clock_now();
};

} // namespace ads1299
