#include "streaming/server.hpp"
#include "streaming/protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include <lz4frame.h>

namespace ads1299 {

static double clock_monotonic() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

StreamingServer::StreamingServer(const PortConfig* ports, int num_ports,
                                 int total_channels, int total_devices)
    : ports_(ports)
    , num_ports_(num_ports)
    , total_channels_(total_channels)
    , total_devices_(total_devices)
    , wire_sample_size_(static_cast<int>(wire_sample_size(total_channels)))
    , stop_flag_(false)
    , pending_client_fd_(-1)
    , listen_fd_(-1)
    , client_fd_(-1)
    , batch_buf_(nullptr)
    , lz4_buf_(nullptr)
    , frame_buf_(nullptr)
    , lz4_cap_(0)
    , frame_cap_(0)
    , lz4_ctx_(nullptr)
    , metadata_len_(0)
    , samples_sent_(0)
    , batches_sent_(0)
    , stream_drops_(0)
    , reconnects_(0)
    , connected_(false)
    , port_(0)
{
    std::memset(host_, 0, sizeof(host_));
    std::memset(metadata_buf_, 0, sizeof(metadata_buf_));

    // Build metadata JSON once
    metadata_len_ = build_metadata_json(metadata_buf_, sizeof(metadata_buf_),
                                        ports, num_ports,
                                        total_channels, total_devices);

    // Pre-allocate batch buffer
    batch_buf_ = static_cast<uint8_t*>(
        std::malloc(static_cast<size_t>(BATCH_SIZE) * wire_sample_size_));

    // Pre-allocate LZ4 context (allocated once, reused per batch)
    LZ4F_errorCode_t err = LZ4F_createCompressionContext(&lz4_ctx_, LZ4F_VERSION);
    if (LZ4F_isError(err)) {
        std::fprintf(stderr, "  [STREAM] LZ4 context creation failed: %s\n",
                     LZ4F_getErrorName(err));
        lz4_ctx_ = nullptr;
    }

    // LZ4 output buffer: compressFrameBound gives the total max for a complete frame
    size_t max_payload = static_cast<size_t>(BATCH_SIZE) * wire_sample_size_;
    lz4_cap_ = LZ4F_compressFrameBound(max_payload, nullptr);
    lz4_buf_ = static_cast<uint8_t*>(std::malloc(lz4_cap_));

    // Frame buffer: 8-byte wire header + max compressed size
    frame_cap_ = 8 + lz4_cap_;
    frame_buf_ = static_cast<uint8_t*>(std::malloc(frame_cap_));
}

StreamingServer::~StreamingServer() {
    stop();

    if (lz4_ctx_) {
        LZ4F_freeCompressionContext(lz4_ctx_);
    }
    std::free(batch_buf_);
    std::free(lz4_buf_);
    std::free(frame_buf_);
}

void StreamingServer::start(const char* host, int port) {
    std::strncpy(host_, host, sizeof(host_) - 1);
    port_ = port;

    // Create listening socket
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::fprintf(stderr, "  [STREAM] socket() failed: %s\n", strerror(errno));
        return;
    }

    int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (std::strcmp(host, "0.0.0.0") == 0) {
        addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, host, &addr.sin_addr);
    }

    if (::bind(listen_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "  [STREAM] bind(%s:%d) failed: %s\n", host, port, strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return;
    }

    if (::listen(listen_fd_, LISTEN_BACKLOG) < 0) {
        std::fprintf(stderr, "  [STREAM] listen() failed: %s\n", strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return;
    }

    std::printf("  [STREAM] Listening on %s:%d\n", host, port);

    stop_flag_.store(false, std::memory_order_relaxed);
    accept_thread_ = std::thread(&StreamingServer::accept_loop, this);
    stream_thread_ = std::thread(&StreamingServer::stream_loop, this);
}

void StreamingServer::stop() {
    stop_flag_.store(true, std::memory_order_relaxed);

    // Close listening socket to unblock accept()
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }
}

StreamingServer::Stats StreamingServer::get_stats() const {
    Stats s{};
    s.samples_sent = samples_sent_.load(std::memory_order_relaxed);
    s.batches_sent = batches_sent_.load(std::memory_order_relaxed);
    s.drops = stream_drops_.load(std::memory_order_relaxed);
    s.reconnects = reconnects_.load(std::memory_order_relaxed);
    s.ring_queued = ring_.size_approx();
    s.connected = connected_.load(std::memory_order_relaxed);
    return s;
}

// Accept thread: pinned core 1 (housekeeping core, shared with stream thread + CSV writer)
void StreamingServer::accept_loop() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        // Poll with 1s timeout so we check stop_flag periodically
        struct pollfd pfd{};
        pfd.fd = listen_fd_;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 1000);
        if (ret <= 0) continue;
        if (pfd.revents & (POLLERR | POLLHUP)) break;

        struct sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        int fd = ::accept(listen_fd_,
                          reinterpret_cast<struct sockaddr*>(&client_addr),
                          &addr_len);
        if (fd < 0) continue;

        // Configure accepted socket
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        int sndbuf = 262144;
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        // Set non-blocking
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        // TCP keepalive: detect dead clients within ~25s
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        int idle = 10;
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        int intvl = 5;
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        int cnt = 3;
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

        char addr_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, addr_str, sizeof(addr_str));
        std::printf("  [STREAM] Client connected: %s:%d\n",
                    addr_str, ntohs(client_addr.sin_port));

        // Hand off to streaming thread via atomic
        int old_fd = pending_client_fd_.exchange(fd, std::memory_order_release);
        if (old_fd >= 0) {
            // Previous pending fd was never consumed — close it
            ::close(old_fd);
        }
    }
}

// Streaming thread: pinned core 1, SCHED_OTHER
void StreamingServer::stream_loop() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    Sample sample;
    int batch_count = 0;
    double last_flush_time = clock_monotonic();
    double last_recv_check = clock_monotonic();

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        // Check for new client connection
        int new_fd = pending_client_fd_.exchange(-1, std::memory_order_acquire);
        if (new_fd >= 0) {
            // Close old client if any
            if (client_fd_ >= 0) {
                ::close(client_fd_);
            }
            client_fd_ = new_fd;

            // Drain ring (discard stale data)
            while (ring_.try_pop(sample)) {}

            // Send metadata JSON
            bool ok = send_all(client_fd_,
                               reinterpret_cast<const uint8_t*>(metadata_buf_),
                               static_cast<size_t>(metadata_len_),
                               SEND_TIMEOUT_SEC);
            if (!ok) {
                std::printf("  [STREAM] Failed to send metadata\n");
                ::close(client_fd_);
                client_fd_ = -1;
                connected_.store(false, std::memory_order_relaxed);
                continue;
            }

            batch_count = 0;
            last_flush_time = clock_monotonic();
            last_recv_check = last_flush_time;
            connected_.store(true, std::memory_order_relaxed);
            reconnects_.fetch_add(1, std::memory_order_relaxed);
            std::printf("  [STREAM] Metadata sent, streaming...\n");
        }

        // No client — drain ring and sleep
        if (client_fd_ < 0) {
            while (ring_.try_pop(sample)) {}
            struct timespec ts{0, 10'000'000};  // 10ms
            nanosleep(&ts, nullptr);
            continue;
        }

        // Periodic recv() to detect client disconnect faster
        double now = clock_monotonic();
        if (now - last_recv_check >= 1.0) {
            char dummy;
            ssize_t r = ::recv(client_fd_, &dummy, 1, MSG_DONTWAIT);
            if (r == 0) {
                // Client sent FIN
                std::printf("  [STREAM] Client disconnected (FIN)\n");
                ::close(client_fd_);
                client_fd_ = -1;
                batch_count = 0;
                connected_.store(false, std::memory_order_relaxed);
                continue;
            } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::printf("  [STREAM] Client disconnected (error: %s)\n", strerror(errno));
                ::close(client_fd_);
                client_fd_ = -1;
                batch_count = 0;
                connected_.store(false, std::memory_order_relaxed);
                continue;
            }
            last_recv_check = now;
        }

        // Try to pop samples and accumulate into batch
        bool got_sample = ring_.try_pop(sample);
        if (got_sample) {
            size_t offset = static_cast<size_t>(batch_count) * wire_sample_size_;
            pack_sample(batch_buf_ + offset, sample, total_channels_);
            batch_count++;
        }

        // Send batch when full or on timeout with partial data
        now = clock_monotonic();
        bool batch_full = (batch_count >= BATCH_SIZE);
        bool flush_timeout = (batch_count > 0 &&
                              (now - last_flush_time) * 1000.0 >= FLUSH_TIMEOUT_MS);

        if (batch_full || flush_timeout) {
            size_t batch_bytes = static_cast<size_t>(batch_count) * wire_sample_size_;

            // LZ4 streaming compression: compressBegin + compressUpdate + compressEnd
            size_t hdr_sz = LZ4F_compressBegin(lz4_ctx_, lz4_buf_, lz4_cap_, nullptr);
            if (LZ4F_isError(hdr_sz)) {
                std::fprintf(stderr, "  [STREAM] LZ4 compressBegin error: %s\n",
                             LZ4F_getErrorName(hdr_sz));
                batch_count = 0;
                last_flush_time = now;
                continue;
            }

            size_t body_sz = LZ4F_compressUpdate(lz4_ctx_,
                                                  lz4_buf_ + hdr_sz,
                                                  lz4_cap_ - hdr_sz,
                                                  batch_buf_, batch_bytes,
                                                  nullptr);
            if (LZ4F_isError(body_sz)) {
                std::fprintf(stderr, "  [STREAM] LZ4 compressUpdate error: %s\n",
                             LZ4F_getErrorName(body_sz));
                batch_count = 0;
                last_flush_time = now;
                continue;
            }

            size_t ftr_sz = LZ4F_compressEnd(lz4_ctx_,
                                              lz4_buf_ + hdr_sz + body_sz,
                                              lz4_cap_ - hdr_sz - body_sz,
                                              nullptr);
            if (LZ4F_isError(ftr_sz)) {
                std::fprintf(stderr, "  [STREAM] LZ4 compressEnd error: %s\n",
                             LZ4F_getErrorName(ftr_sz));
                batch_count = 0;
                last_flush_time = now;
                continue;
            }

            size_t compressed_size = hdr_sz + body_sz + ftr_sz;

            // Build frame: [uint32 compressed_size][uint32 sample_count][lz4 data]
            uint32_t cs = static_cast<uint32_t>(compressed_size);
            uint32_t sc = static_cast<uint32_t>(batch_count);
            std::memcpy(frame_buf_, &cs, 4);
            std::memcpy(frame_buf_ + 4, &sc, 4);
            std::memcpy(frame_buf_ + 8, lz4_buf_, compressed_size);

            size_t frame_size = 8 + compressed_size;

            bool ok = send_all(client_fd_, frame_buf_, frame_size, SEND_TIMEOUT_SEC);
            if (!ok) {
                std::printf("  [STREAM] Send failed -- client disconnected\n");
                ::close(client_fd_);
                client_fd_ = -1;
                connected_.store(false, std::memory_order_relaxed);
            } else {
                samples_sent_.fetch_add(batch_count, std::memory_order_relaxed);
                batches_sent_.fetch_add(1, std::memory_order_relaxed);
            }

            batch_count = 0;
            last_flush_time = now;
        }

        // If no sample popped and no batch pending, sleep briefly
        if (!got_sample && batch_count == 0) {
            struct timespec ts{0, 1'000'000};  // 1ms
            nanosleep(&ts, nullptr);
        }
    }

    // Cleanup
    if (client_fd_ >= 0) {
        ::close(client_fd_);
        client_fd_ = -1;
    }
    connected_.store(false, std::memory_order_relaxed);
}

} // namespace ads1299
