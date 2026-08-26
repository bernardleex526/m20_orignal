#pragma once
/**
 * @file thread_safe_queue.hpp
 * @brief Lock-free and mutex-based concurrent queues for sensor data pipelines.
 *
 * Provides:
 *  - SPSCQueue: single-producer single-consumer lock-free ring buffer (IMU buffer).
 *  - MPSCQueue: multi-producer single-consumer queue with mutex (LiDAR buffer).
 */

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <vector>

namespace m20 {

// =============================================================================
// SPSC Lock-Free Ring Buffer — for high-throughput IMU data (200Hz+)
// =============================================================================
template <typename T, std::size_t Capacity = 4096>
class SPSCQueue {
  static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
  static constexpr std::size_t kMask = Capacity - 1;

public:
  SPSCQueue() : buffer_(Capacity) {}

  /// Try to push (non-blocking). Returns false if full.
  bool try_push(const T& item) {
    const std::size_t write = write_pos_.load(std::memory_order_relaxed);
    const std::size_t next  = (write + 1) & kMask;
    if (next == read_pos_.load(std::memory_order_acquire)) {
      return false;  // full
    }
    buffer_[write] = item;
    write_pos_.store(next, std::memory_order_release);
    return true;
  }

  /// Try to push with move semantics
  bool try_push(T&& item) {
    const std::size_t write = write_pos_.load(std::memory_order_relaxed);
    const std::size_t next  = (write + 1) & kMask;
    if (next == read_pos_.load(std::memory_order_acquire)) {
      return false;
    }
    buffer_[write] = std::move(item);
    write_pos_.store(next, std::memory_order_release);
    return true;
  }

  /// Try to pop (non-blocking). Returns std::nullopt if empty.
  std::optional<T> try_pop() {
    const std::size_t read = read_pos_.load(std::memory_order_relaxed);
    if (read == write_pos_.load(std::memory_order_acquire)) {
      return std::nullopt;  // empty
    }
    T item = std::move(buffer_[read]);
    read_pos_.store((read + 1) & kMask, std::memory_order_release);
    return item;
  }

  /// Pop all available items into a vector
  std::vector<T> drain() {
    std::vector<T> out;
    while (auto item = try_pop()) {
      out.push_back(std::move(*item));
    }
    return out;
  }

  bool empty() const {
    return read_pos_.load(std::memory_order_acquire) ==
           write_pos_.load(std::memory_order_acquire);
  }

private:
  std::vector<T>           buffer_;
  alignas(64) std::atomic<std::size_t> write_pos_{0};  // cache-line isolated
  alignas(64) std::atomic<std::size_t> read_pos_{0};
};

// =============================================================================
// MPSC Thread-Safe Queue with condition variable — for LiDAR frames (10-20Hz)
// =============================================================================
template <typename T>
class MPSCQueue {
public:
  /// Push (any thread). Notifies one consumer.
  void push(T item) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push(std::move(item));
    }
    cv_.notify_one();
  }

  /// Pop with timeout (ms). Returns std::nullopt if timeout.
  template <typename Rep, typename Period>
  std::optional<T> pop_for(const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
      return std::nullopt;
    }
    T item = std::move(queue_.front());
    queue_.pop();
    return item;
  }

  /// Pop (blocking indefinitely)
  T pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return !queue_.empty(); });
    T item = std::move(queue_.front());
    queue_.pop();
    return item;
  }

  /// Try to pop (non-blocking)
  std::optional<T> try_pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return std::nullopt;
    T item = std::move(queue_.front());
    queue_.pop();
    return item;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:
  mutable std::mutex      mutex_;
  std::condition_variable cv_;
  std::queue<T>           queue_;
};

}  // namespace m20