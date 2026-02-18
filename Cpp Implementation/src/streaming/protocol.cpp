#include "streaming/protocol.hpp"

#include <cstdio>
#include <cstring>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>

namespace ads1299 {

size_t pack_sample(uint8_t* dst, const Sample& s, int num_channels) {
    size_t off = 0;

    // timestamp: float64 LE (8 bytes)
    std::memcpy(dst + off, &s.timestamp, 8);
    off += 8;

    // sample_number: uint32 LE (4 bytes)
    std::memcpy(dst + off, &s.sample_number, 4);
    off += 4;

    // channels: int32 LE x num_channels
    size_t ch_bytes = static_cast<size_t>(num_channels) * 4;
    std::memcpy(dst + off, s.channels.data(), ch_bytes);
    off += ch_bytes;

    return off;
}

int build_metadata_json(char* buf, size_t cap,
                        const PortConfig* ports, int num_ports,
                        int total_channels, int total_devices) {
    // Build ports array: ["Port1","Port2",...]
    char ports_arr[256];
    int po = 0;
    ports_arr[po++] = '[';
    for (int i = 0; i < num_ports; ++i) {
        if (i > 0) ports_arr[po++] = ',';
        po += std::snprintf(ports_arr + po, sizeof(ports_arr) - po,
                            "\"%s\"", ports[i].port_name);
    }
    ports_arr[po++] = ']';
    ports_arr[po] = '\0';

    // Build port_config array: [{"name":"Port1","num_devices":4},...]
    char port_cfg[512];
    int pc = 0;
    port_cfg[pc++] = '[';
    for (int i = 0; i < num_ports; ++i) {
        if (i > 0) port_cfg[pc++] = ',';
        pc += std::snprintf(port_cfg + pc, sizeof(port_cfg) - pc,
                            "{\"name\":\"%s\",\"num_devices\":%d}",
                            ports[i].port_name, ports[i].num_devices);
    }
    port_cfg[pc++] = ']';
    port_cfg[pc] = '\0';

    int sample_size = 12 + total_channels * 4;

    int len = std::snprintf(buf, cap,
        "{\"format\":\"binary_lz4\",\"batch_size\":%d,\"sample_rate\":250,"
        "\"num_channels\":%d,\"num_devices\":%d,"
        "\"ports\":%s,\"port_config\":%s,"
        "\"sample_size\":%d,\"sample_struct\":\"<dI%di\"}\n",
        BATCH_SIZE, total_channels, total_devices,
        ports_arr, port_cfg,
        sample_size, total_channels);

    return len;
}

static double clock_monotonic() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

bool send_all(int fd, const uint8_t* data, size_t len, double timeout_sec) {
    double deadline = clock_monotonic() + timeout_sec;
    size_t sent = 0;

    while (sent < len) {
        double remaining = deadline - clock_monotonic();
        if (remaining <= 0) return false;

        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLOUT;

        int timeout_ms = static_cast<int>(remaining * 1000);
        if (timeout_ms < 1) timeout_ms = 1;

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret <= 0) return false;
        if (pfd.revents & (POLLERR | POLLHUP)) return false;

        ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return false;
        }
        sent += static_cast<size_t>(n);
    }

    return true;
}

} // namespace ads1299
