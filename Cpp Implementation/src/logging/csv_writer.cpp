#include "logging/csv_writer.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <time.h>

namespace ads1299 {

CSVWriter::CSVWriter(const char* filename, SPSCRing<Sample>& ring,
                     int num_channels, const char* const* channel_names, int num_names)
    : ring_(ring)
    , file_(nullptr)
    , num_channels_(num_channels)
    , running_(false)
    , total_written_(0)
{
    std::memset(row_buf_, 0, sizeof(row_buf_));

    // Open with fopen + 1MB setvbuf (NOT std::ofstream — allocates internally, locale overhead)
    file_ = std::fopen(filename, "w");
    if (!file_) {
        std::perror(filename);
        return;
    }

    // 1MB write buffer to absorb SD card write stalls
    std::setvbuf(file_, file_buf_, _IOFBF, sizeof(file_buf_));

    // Write header
    std::fprintf(file_, "timestamp,sample_number");
    for (int i = 0; i < num_names; ++i) {
        std::fprintf(file_, ",%s", channel_names[i]);
    }
    // If channel_names doesn't cover all channels, add generic names
    for (int i = num_names; i < num_channels; ++i) {
        std::fprintf(file_, ",ch%d", i);
    }
    std::fprintf(file_, "\n");
}

CSVWriter::~CSVWriter() {
    stop();
}

void CSVWriter::start() {
    if (!file_) return;
    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&CSVWriter::run, this);
}

void CSVWriter::stop() {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }

    // Drain any remaining samples
    if (file_) {
        Sample sample;
        while (ring_.try_pop(sample)) {
            // Format and write final samples
            char* p = row_buf_;
            char* end = row_buf_ + ROW_BUF_SIZE - 2;  // Leave room for \n\0

            // Timestamp (use snprintf for double — to_chars float support varies)
            int n = std::snprintf(p, end - p, "%.6f,", sample.timestamp);
            if (n > 0) p += n;

            // Sample number (use to_chars — 2-3x faster than snprintf on ARM64)
            auto [ptr1, ec1] = std::to_chars(p, end, sample.sample_number);
            if (ec1 == std::errc()) p = ptr1;

            // Channel values
            int ch_count = (num_channels_ < static_cast<int>(sample.num_channels))
                           ? num_channels_ : static_cast<int>(sample.num_channels);
            for (int i = 0; i < ch_count; ++i) {
                *p++ = ',';
                auto [ptr, ec] = std::to_chars(p, end, sample.channels[i]);
                if (ec == std::errc()) {
                    p = ptr;
                }
            }
            *p++ = '\n';
            std::fwrite(row_buf_, 1, p - row_buf_, file_);
            total_written_++;
        }

        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
    }

    std::printf("  CSV writer: %lu samples written\n",
                static_cast<unsigned long>(total_written_));
}

void CSVWriter::run() {
    Sample sample;

    // Timing for idle sleep
    struct timespec idle_ts = {0, 50'000'000};  // 50ms idle sleep when no data

    while (running_.load(std::memory_order_acquire)) {
        int rows_written = 0;

        // Drain all pending samples (batch write)
        while (ring_.try_pop(sample)) {
            char* p = row_buf_;
            char* end = row_buf_ + ROW_BUF_SIZE - 2;

            // Timestamp
            int n = std::snprintf(p, end - p, "%.6f,", sample.timestamp);
            if (n > 0) p += n;

            // Sample number
            auto [ptr1, ec1] = std::to_chars(p, end, sample.sample_number);
            if (ec1 == std::errc()) p = ptr1;

            // Channel values
            int ch_count = (num_channels_ < static_cast<int>(sample.num_channels))
                           ? num_channels_ : static_cast<int>(sample.num_channels);
            for (int i = 0; i < ch_count; ++i) {
                *p++ = ',';
                auto [ptr, ec] = std::to_chars(p, end, sample.channels[i]);
                if (ec == std::errc()) {
                    p = ptr;
                }
            }
            *p++ = '\n';

            std::fwrite(row_buf_, 1, p - row_buf_, file_);
            total_written_++;
            rows_written++;
        }

        if (rows_written == 0) {
            nanosleep(&idle_ts, nullptr);
        }
    }
}

} // namespace ads1299
