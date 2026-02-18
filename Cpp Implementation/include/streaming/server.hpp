#pragma once

#include "ads1299/types.hpp"
#include "logging/spsc_ring.hpp"
#include "streaming/protocol.hpp"

#include <atomic>
#include <cstdint>
#include <thread>

// Forward declare LZ4F types to avoid including lz4frame.h in header
typedef struct LZ4F_cctx_s LZ4F_cctx;

namespace ads1299 {

class StreamingServer {
public:
    StreamingServer(const PortConfig* ports, int num_ports,
                    int total_channels, int total_devices);
    ~StreamingServer();

    StreamingServer(const StreamingServer&) = delete;
    StreamingServer& operator=(const StreamingServer&) = delete;

    // Called from acquisition hot loop — just ring_.try_push()
    void push_sample(const Sample& s) {
        if (!ring_.try_push(s)) {
            stream_drops_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void start(const char* host, int port);
    void stop();

    // Stats for display
    struct Stats {
        uint64_t samples_sent;
        uint64_t batches_sent;
        uint64_t drops;
        uint64_t reconnects;
        size_t   ring_queued;
        bool     connected;
    };
    Stats get_stats() const;

private:
    void accept_loop();
    void stream_loop();

    // Port config for metadata JSON
    const PortConfig* ports_;
    int num_ports_;
    int total_channels_;
    int total_devices_;
    int wire_sample_size_;

    // SPSC ring — separate instance from CSV ring
    SPSCRing<Sample, 2048> ring_;

    // Threads
    std::thread accept_thread_;
    std::thread stream_thread_;
    std::atomic<bool> stop_flag_;

    // Accept -> stream handoff (fd or -1)
    std::atomic<int> pending_client_fd_;

    // Sockets (owned solely by their respective threads)
    int listen_fd_;
    int client_fd_;  // owned by stream_thread only

    // Pre-allocated buffers
    uint8_t* batch_buf_;   // BATCH_SIZE * wire_sample_size
    uint8_t* lz4_buf_;     // LZ4 compressed output
    uint8_t* frame_buf_;   // 8 (header) + lz4 output
    size_t   lz4_cap_;
    size_t   frame_cap_;

    // LZ4 streaming context (allocated once)
    LZ4F_cctx* lz4_ctx_;

    // Metadata JSON (built once at construction)
    char metadata_buf_[1024];
    int  metadata_len_;

    // Stats (atomics for cross-thread reads)
    std::atomic<uint64_t> samples_sent_;
    std::atomic<uint64_t> batches_sent_;
    std::atomic<uint64_t> stream_drops_;
    std::atomic<uint64_t> reconnects_;
    std::atomic<bool>     connected_;

    // Listen address
    char host_[64];
    int  port_;
};

} // namespace ads1299
