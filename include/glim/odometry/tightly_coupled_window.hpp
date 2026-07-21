#pragma once

#include <algorithm>
#include <cstddef>

namespace glim
{

// Return the first preceding frame that is both requested by the connection
// window and still resident in IndexedSlidingWindow. The latter matters after
// a long LiDAR blackout: fixed-lag marginalization can leave fewer resident
// frames than full_connection_window_size when the first valid scan returns.
inline int tightlyCoupledFirstTarget(
  int current, int window_size, std::size_t resident_frame_count)
{
  const int requested = std::max(0, current - std::max(0, window_size));
  const int resident = current + 1 - static_cast<int>(
    std::min<std::size_t>(resident_frame_count, static_cast<std::size_t>(current + 1)));
  return std::max(requested, resident);
}

}  // namespace glim
