#include "acquisition/bus_worker.hpp"
#include "ads1299/spi_device.hpp"
#include "ads1299/registers.hpp"

#include <cstdio>
#include <cstring>
#include <sched.h>
#include <pthread.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <unistd.h>

namespace ads1299 {

BusWorker::BusWorker(int bus_num, const std::vector<ADS1299Device*>& devices, int cpu_core)
    : bus_num_(bus_num)
    , num_ports_(static_cast<int>(devices.size()))
    , cpu_core_(cpu_core)
    , event_fd_(-1)
    , done_(true)   // Start in "done" state (ready for first trigger)
    , running_(true)
{
    std::memset(devices_, 0, sizeof(devices_));
    std::memset(raw_buffers_, 0, sizeof(raw_buffers_));

    for (int i = 0; i < num_ports_ && i < 2; ++i) {
        devices_[i] = devices[i];
    }

    // Create eventfd for low-latency wake signaling (~3-10us vs condvar ~10-50us).
    // Blocking mode (no EFD_NONBLOCK) — workers should sleep when idle, not spin.
    event_fd_ = eventfd(0, 0);
    if (event_fd_ < 0) {
        std::perror("eventfd");
    }

    thread_ = std::thread(&BusWorker::run, this);
}

BusWorker::~BusWorker() {
    stop();
    if (event_fd_ >= 0) {
        ::close(event_fd_);
    }
}

void BusWorker::run() {
    // Lock all pages (process-wide, idempotent with engine's mlockall)
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Pre-fault 64KB of stack — prevents minor page faults during SPI ioctl
    volatile char stack_prefault[64 * 1024];
    for (size_t i = 0; i < sizeof(stack_prefault); i += 4096) {
        stack_prefault[i] = 0;
    }
    (void)stack_prefault;

    // Set RT priority: SCHED_FIFO 49 (one below engine's 50)
    struct sched_param param{};
    param.sched_priority = 49;
    sched_setscheduler(0, SCHED_FIFO, &param);

    // Pin to assigned CPU core
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core_, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    while (running_.load(std::memory_order_relaxed)) {
        // Wait on eventfd (blocks — NOT busy-spin — workers share non-isolated cores)
        uint64_t val;
        ssize_t ret;
        do {
            ret = ::read(event_fd_, &val, sizeof(val));
        } while (ret < 0 && running_.load(std::memory_order_relaxed));

        if (!running_.load(std::memory_order_relaxed)) break;

        // Read all ports on this bus sequentially (they share MISO/MOSI/SCLK)
        for (int i = 0; i < num_ports_; ++i) {
            int len = devices_[i]->config().num_devices * BYTES_PER_DEVICE;
            devices_[i]->read_raw(raw_buffers_[i], len);
        }

        // Signal completion — engine spin-waits on this with acquire ordering
        done_.store(true, std::memory_order_release);
    }
}

void BusWorker::trigger() {
    done_.store(false, std::memory_order_release);
    // Write to eventfd to wake worker thread
    uint64_t val = 1;
    ::write(event_fd_, &val, sizeof(val));
}

void BusWorker::wait_done() {
    // Engine spins on atomic (isolated core 3, sub-ms wait — appropriate for 144-288us)
    while (!done_.load(std::memory_order_acquire)) {
        // Tight spin — no yield, no sleep. Engine is on dedicated core 3.
    }
}

void BusWorker::stop() {
    if (!running_.load(std::memory_order_relaxed)) return;
    running_.store(false, std::memory_order_release);

    // Wake the thread so it can exit
    uint64_t val = 1;
    if (event_fd_ >= 0) {
        ::write(event_fd_, &val, sizeof(val));
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

} // namespace ads1299
