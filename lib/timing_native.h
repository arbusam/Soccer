#ifndef SOCCER_TIMING_NATIVE_H
#define SOCCER_TIMING_NATIVE_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <time.h>

namespace timing {
using Ns = std::int64_t;
inline Ns now() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return Ns(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
struct Row { const char* name; Ns start; Ns end; Ns value; };
struct Buffer {
    std::mutex mutex;
    std::vector<Row> rows;
    std::atomic<size_t> dropped{0};
};
struct State {
    std::atomic<bool> enabled{false};
    bool configured = false;
    Ns start = 0, end = 0;
    size_t capacity = 100000;
    std::mutex registry;
    std::vector<std::shared_ptr<Buffer>> buffers;
};
inline State& state() { static State s; return s; }
// Configure once: enabled publishes the immutable window to existing workers.
// Drain only after disabling capture.
inline void record(const char* name, Ns start, Ns end) {
    auto& s = state();
    if (!s.enabled.load(std::memory_order_acquire) || end >= s.end) return;
    thread_local std::shared_ptr<Buffer> buffer;
    if (!buffer) {
        buffer = std::make_shared<Buffer>();
        buffer->rows.reserve(s.capacity);
        std::lock_guard<std::mutex> lock(s.registry);
        s.buffers.push_back(buffer);
    }
    if (end < s.start) return;
    std::unique_lock<std::mutex> lock(buffer->mutex, std::try_to_lock);
    if (!lock.owns_lock() || buffer->rows.size() >= s.capacity) {
        buffer->dropped.fetch_add(1);
        return;
    }
    buffer->rows.push_back({name, start, end, end - start});
}
struct Scope {
    const char* name;
    Ns start;
    explicit Scope(const char* n) : name(n), start(state().enabled.load() ? now() : 0) {}
    ~Scope() { if (start) record(name, start, now()); }
};
// Release the application mutex before emitting records.
struct Lock {
    std::mutex& mutex;
    const char* wait_name;
    const char* hold_name;
    Ns begin, acquired;
    Lock(std::mutex& m, const char* w, const char* h)
        : mutex(m), wait_name(w), hold_name(h),
          begin(state().enabled.load() ? now() : 0), acquired(0) {
        mutex.lock();
        if (begin) acquired = now();
    }
    ~Lock() {
        Ns end = begin ? now() : 0;
        mutex.unlock();
        if (begin) { record(wait_name, begin, acquired); record(hold_name, acquired, end); }
    }
};
}
#endif
