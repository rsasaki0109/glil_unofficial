#pragma once

#ifdef GLIL_PROFILE_TIMING

#include <atomic>
#include <chrono>
#include <cstdint>

namespace glil::perftime {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

enum class Counter {
  CreateFactors,
  UpdateCorresp,
  ExtractCoreset,
  Coreset_ValidFilter,
  Coreset_BuildJE,
  Coreset_Caratheodory,
  Coreset_CarathCalls,
  Coreset_SelectedRows,
  Evaluate,
  Smoother,
};

struct Snapshot {
  std::uint64_t create_factors_us = 0;
  std::uint64_t update_corresp_us = 0;
  std::uint64_t extract_coreset_us = 0;
  std::uint64_t coreset_filter_us = 0;
  std::uint64_t coreset_buildje_us = 0;
  std::uint64_t coreset_carath_us = 0;
  std::uint64_t coreset_carath_calls = 0;
  std::uint64_t coreset_rows_us = 0;
  std::uint64_t evaluate_us = 0;
  std::uint64_t smoother_us = 0;
};

struct Counters {
  std::atomic<std::uint64_t> create_factors_us{0};
  std::atomic<std::uint64_t> update_corresp_us{0};
  std::atomic<std::uint64_t> extract_coreset_us{0};
  std::atomic<std::uint64_t> coreset_filter_us{0};
  std::atomic<std::uint64_t> coreset_buildje_us{0};
  std::atomic<std::uint64_t> coreset_carath_us{0};
  std::atomic<std::uint64_t> coreset_carath_calls{0};
  std::atomic<std::uint64_t> coreset_rows_us{0};
  std::atomic<std::uint64_t> evaluate_us{0};
  std::atomic<std::uint64_t> smoother_us{0};
};

inline Counters counters;

inline TimePoint now() {
  return Clock::now();
}

inline std::uint64_t elapsed_us(const TimePoint& start, const TimePoint& end) {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
}

inline void add(const Counter counter, const std::uint64_t us) {
  switch (counter) {
    case Counter::CreateFactors:
      counters.create_factors_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::UpdateCorresp:
      counters.update_corresp_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::ExtractCoreset:
      counters.extract_coreset_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Coreset_ValidFilter:
      counters.coreset_filter_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Coreset_BuildJE:
      counters.coreset_buildje_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Coreset_Caratheodory:
      counters.coreset_carath_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Coreset_CarathCalls:
      counters.coreset_carath_calls.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Coreset_SelectedRows:
      counters.coreset_rows_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Evaluate:
      counters.evaluate_us.fetch_add(us, std::memory_order_relaxed);
      break;
    case Counter::Smoother:
      counters.smoother_us.fetch_add(us, std::memory_order_relaxed);
      break;
  }
}

inline void reset() {
  counters.create_factors_us.store(0, std::memory_order_relaxed);
  counters.update_corresp_us.store(0, std::memory_order_relaxed);
  counters.extract_coreset_us.store(0, std::memory_order_relaxed);
  counters.coreset_filter_us.store(0, std::memory_order_relaxed);
  counters.coreset_buildje_us.store(0, std::memory_order_relaxed);
  counters.coreset_carath_us.store(0, std::memory_order_relaxed);
  counters.coreset_carath_calls.store(0, std::memory_order_relaxed);
  counters.coreset_rows_us.store(0, std::memory_order_relaxed);
  counters.evaluate_us.store(0, std::memory_order_relaxed);
  counters.smoother_us.store(0, std::memory_order_relaxed);
}

inline Snapshot snapshot() {
  Snapshot snap;
  snap.create_factors_us = counters.create_factors_us.load(std::memory_order_relaxed);
  snap.update_corresp_us = counters.update_corresp_us.load(std::memory_order_relaxed);
  snap.extract_coreset_us = counters.extract_coreset_us.load(std::memory_order_relaxed);
  snap.coreset_filter_us = counters.coreset_filter_us.load(std::memory_order_relaxed);
  snap.coreset_buildje_us = counters.coreset_buildje_us.load(std::memory_order_relaxed);
  snap.coreset_carath_us = counters.coreset_carath_us.load(std::memory_order_relaxed);
  snap.coreset_carath_calls = counters.coreset_carath_calls.load(std::memory_order_relaxed);
  snap.coreset_rows_us = counters.coreset_rows_us.load(std::memory_order_relaxed);
  snap.evaluate_us = counters.evaluate_us.load(std::memory_order_relaxed);
  snap.smoother_us = counters.smoother_us.load(std::memory_order_relaxed);
  return snap;
}

class ScopedTimer {
public:
  explicit ScopedTimer(const Counter counter) : counter(counter), start(now()) {}

  ~ScopedTimer() {
    add(counter, elapsed_us(start, now()));
  }

private:
  Counter counter;
  TimePoint start;
};

}  // namespace glil::perftime

#endif  // GLIL_PROFILE_TIMING
