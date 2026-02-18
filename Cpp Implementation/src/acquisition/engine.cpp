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

void AcquisitionEngine::run(volatile sig_atomic_t& running) {
    // --- RT Hardening ---

    // Lock all pages — prevents page faults in hot loop
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
        // Keep fd open — closing would re-enable deep C-states
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

            // Status byte validation (first device)
            if ((port_data.status_bytes[0][0] & 0xF0) != 0xC0) {
                corruption_count_++;
                sample.valid = false;
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

        // 7. Print stats every 10 seconds
        if (cycle_end - last_stats_time_ >= STATS_INTERVAL_SEC) {
            print_stats();
            last_stats_time_ = cycle_end;
        }
    }

    // Final stats on shutdown
    print_stats();

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
