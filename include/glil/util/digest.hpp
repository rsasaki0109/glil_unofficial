#pragma once

#include <cstddef>
#include <cstdint>

namespace glil {

inline uint64_t fnv1a_u64(const void* data, const std::size_t bytes) {
  constexpr uint64_t offset_basis = 14695981039346656037ull;
  constexpr uint64_t prime = 1099511628211ull;

  uint64_t hash = offset_basis;
  const auto* ptr = static_cast<const uint8_t*>(data);
  for (std::size_t i = 0; i < bytes; i++) {
    hash ^= static_cast<uint64_t>(ptr[i]);
    hash *= prime;
  }

  return hash;
}

inline uint64_t fnv1a_doubles(const double* v, const std::size_t n) {
  return fnv1a_u64(v, n * sizeof(double));
}

}  // namespace glil
