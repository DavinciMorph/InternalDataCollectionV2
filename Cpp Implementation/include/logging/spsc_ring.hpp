#pragma once

#include <atomic>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace ads1299 {

// Bounded single-producer single-consumer lock-free ring buffer.
// Power-of-2 capacity enables bitwise AND for index wrapping (no division).
// Cache-line padding on indices prevents false sharing (Cortex-A72: 64-byte lines).
// Uses acquire/release ordering (LDAR/STLR on ARM64) — NOT seq_cst (avoids DMB).
template <typename T, size_t Capacity = 2048>
class SPSCRing {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");

public:
    SPSCRing() : write_idx_(0), read_idx_(0) {}

    // Non-blocking push. Returns false if full (caller should increment drop counter).
    bool try_push(const T& item) {
        const size_t w = write_idx_.load(std::memory_order_relaxed);
        const size_t next_w = (w + 1) & MASK;
        if (next_w == read_idx_.load(std::memory_order_acquire)) {
            return false;  // Full
        }
        std::memcpy(&buffer_[w], &item, sizeof(T));
        write_idx_.store(next_w, std::memory_order_release);
        return true;
    }

    // Non-blocking pop. Returns false if empty.
    bool try_pop(T& item) {
        const size_t r = read_idx_.load(std::memory_order_relaxed);
        if (r == write_idx_.load(std::memory_order_acquire)) {
            return false;  // Empty
        }
        std::memcpy(&item, &buffer_[r], sizeof(T));
        read_idx_.store((r + 1) & MASK, std::memory_order_release);
        return true;
    }

    size_t capacity() const { return Capacity; }

    size_t size_approx() const {
        const size_t w = write_idx_.load(std::memory_order_relaxed);
        const size_t r = read_idx_.load(std::memory_order_relaxed);
        return (w - r) & MASK;
    }

private:
    static constexpr size_t MASK = Capacity - 1;

    // Cache-line aligned indices to prevent false sharing.
    // Cortex-A72 cache line = 64 bytes. Hard-coded (GCC doesn't define
    // std::hardware_destructive_interference_size on aarch64).
    alignas(64) std::atomic<size_t> write_idx_;
    alignas(64) std::atomic<size_t> read_idx_;

    // Buffer storage
    alignas(64) T buffer_[Capacity];
};

} // namespace ads1299
