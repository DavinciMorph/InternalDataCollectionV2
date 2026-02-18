#pragma once

#include "ads1299/types.hpp"
#include "logging/spsc_ring.hpp"

#include <atomic>
#include <cstdio>
#include <thread>

namespace ads1299 {

// Dedicated writer thread consuming from SPSC ring buffer.
// Formats directly into pre-allocated char buffer via std::to_chars (no std::string per row).
// File opened with fopen + setvbuf 1MB buffer (NOT std::ofstream).
class CSVWriter {
public:
    CSVWriter(const char* filename, SPSCRing<Sample>& ring,
              int num_channels, const char* const* channel_names, int num_names);
    ~CSVWriter();

    CSVWriter(const CSVWriter&) = delete;
    CSVWriter& operator=(const CSVWriter&) = delete;

    // Start the writer thread
    void start();

    // Signal thread to stop, drain remaining, flush, close
    void stop();

    uint64_t total_written() const { return total_written_; }

private:
    void run();

    SPSCRing<Sample>& ring_;
    FILE* file_;
    int num_channels_;

    std::atomic<bool> running_;
    std::thread thread_;
    uint64_t total_written_;

    // Pre-allocated row buffer (8KB — enough for 224 channels @ ~12 digits each)
    static constexpr size_t ROW_BUF_SIZE = 8192;
    char row_buf_[ROW_BUF_SIZE];

    // 1MB file write buffer for setvbuf (member, not static — safe for multiple instances)
    char file_buf_[1024 * 1024];
};

} // namespace ads1299
