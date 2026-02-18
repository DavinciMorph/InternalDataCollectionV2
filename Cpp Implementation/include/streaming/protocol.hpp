#pragma once

#include "ads1299/types.hpp"

#include <cstddef>
#include <cstdint>

namespace ads1299 {

// Streaming protocol constants
constexpr int    BATCH_SIZE        = 10;
constexpr int    FLUSH_TIMEOUT_MS  = 100;
constexpr double SEND_TIMEOUT_SEC  = 2.0;
constexpr int    LISTEN_BACKLOG    = 1;

// Wire sample size: timestamp (8) + sample_number (4) + channels (N*4)
inline constexpr size_t wire_sample_size(int num_channels) {
    return 8 + 4 + static_cast<size_t>(num_channels) * 4;
}

// Pack a Sample into the wire format buffer.
// dst must have at least wire_sample_size(num_channels) bytes.
// Returns number of bytes written.
size_t pack_sample(uint8_t* dst, const Sample& s, int num_channels);

// Build the metadata JSON sent once per connection.
// Returns the number of bytes written (including trailing '\n').
int build_metadata_json(char* buf, size_t cap,
                        const PortConfig* ports, int num_ports,
                        int total_channels, int total_devices);

// Non-blocking send loop with timeout using poll(POLLOUT).
// Returns true if all data was sent, false on timeout or error.
// Always uses MSG_NOSIGNAL.
bool send_all(int fd, const uint8_t* data, size_t len, double timeout_sec);

} // namespace ads1299
