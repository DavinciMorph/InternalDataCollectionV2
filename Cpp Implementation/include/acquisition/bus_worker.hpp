#pragma once

#include "ads1299/types.hpp"

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

namespace ads1299 {

class ADS1299Device;

// Persistent thread that reads all ports on a single SPI bus.
// One BusWorker per physical SPI bus. Workers on different buses execute truly in parallel.
// Ports on the SAME bus are read sequentially (shared SCLK/MOSI/MISO).
//
// Signaling: eventfd for wake (lower latency than condvar: ~3-10us vs ~10-50us).
// Completion: atomic<bool> done_ flag — engine spin-waits with acquire ordering.
// Workers wait on eventfd when idle (NOT busy-spin — they share non-isolated cores).
class BusWorker {
public:
    BusWorker(int bus_num, const std::vector<ADS1299Device*>& devices, int cpu_core);
    ~BusWorker();

    BusWorker(const BusWorker&) = delete;
    BusWorker& operator=(const BusWorker&) = delete;

    // Signal worker to start reading (non-blocking)
    void trigger();

    // Spin-wait until reads are complete (called from engine on isolated core)
    void wait_done();

    // Shut down the worker thread
    void stop();

    // Access raw buffers after wait_done()
    uint8_t* raw_buffer(int port_index) { return raw_buffers_[port_index]; }
    int port_count() const { return num_ports_; }
    int bus_num() const { return bus_num_; }

    ADS1299Device* device(int port_index) { return devices_[port_index]; }

private:
    void run();

    int bus_num_;
    int num_ports_;
    int cpu_core_;
    int event_fd_;

    ADS1299Device* devices_[2];  // Max 2 ports per bus (CE0, CE1)

    // Pre-allocated raw buffers per port
    uint8_t raw_buffers_[2][PortData::MAX_RAW_BYTES];

    alignas(64) std::atomic<bool> done_;
    std::atomic<bool> running_;
    std::thread thread_;
};

} // namespace ads1299
