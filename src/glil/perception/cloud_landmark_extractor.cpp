// SPDX-License-Identifier: MIT
#include <glil/perception/cloud_landmark_extractor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>

namespace glil {
namespace {

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey& rhs) const { return x == rhs.x && y == rhs.y && z == rhs.z; }
};

std::uint64_t splitmix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

std::uint64_t rotate_left(std::uint64_t value, int shift) {
  return (value << shift) | (value >> (64 - shift));
}

std::uint64_t stable_string_hash(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char c : value) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t encode_signed(std::int64_t value) {
  return static_cast<std::uint64_t>(value) ^ 0x8000000000000000ULL;
}

std::uint64_t stable_landmark_id(const VoxelKey& key, const std::string& class_id) {
  std::uint64_t hash = stable_string_hash(class_id.empty() ? std::string("cloud_landmark") : class_id);
  hash ^= splitmix64(encode_signed(key.x));
  hash ^= rotate_left(splitmix64(encode_signed(key.y)), 21);
  hash ^= rotate_left(splitmix64(encode_signed(key.z)), 42);

  // gtsam::Symbol stores the index in the lower 56 bits of a Key.
  hash = splitmix64(hash) & 0x00ffffffffffffffULL;
  return hash == 0 ? 1 : hash;
}

struct VoxelKeyHasher {
  std::size_t operator()(const VoxelKey& key) const {
    std::uint64_t hash = splitmix64(encode_signed(key.x));
    hash ^= rotate_left(splitmix64(encode_signed(key.y)), 21);
    hash ^= rotate_left(splitmix64(encode_signed(key.z)), 42);
    return static_cast<std::size_t>(hash);
  }
};

struct Accumulator {
  std::size_t count = 0;
  Eigen::Vector3d sum_sensor = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_outer_sensor = Eigen::Matrix3d::Zero();
};

struct Candidate {
  PerceptionObservation observation;
  std::size_t count = 0;
};

std::int64_t voxel_coord(double value, double voxel_resolution) {
  return static_cast<std::int64_t>(std::floor(value / voxel_resolution));
}

bool passes_range(const gtsam::Point3& point_sensor, double min_range, double max_range) {
  const double range = point_sensor.norm();
  if (min_range > 0.0 && range < min_range) {
    return false;
  }
  if (max_range > 0.0 && range > max_range) {
    return false;
  }
  return true;
}

Eigen::Matrix3d covariance_for(const Accumulator& acc, const Eigen::Vector3d& mean, double min_covariance) {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * min_covariance;
  if (acc.count > 1) {
    Eigen::Matrix3d scatter = acc.sum_outer_sensor - static_cast<double>(acc.count) * mean * mean.transpose();
    scatter /= static_cast<double>(acc.count - 1);
    covariance = scatter / static_cast<double>(acc.count);
  }

  if (!covariance.allFinite()) {
    covariance = Eigen::Matrix3d::Identity() * min_covariance;
  }

  covariance = 0.5 * (covariance + covariance.transpose());
  for (int i = 0; i < 3; i++) {
    covariance(i, i) = std::max(covariance(i, i), min_covariance);
  }
  return covariance;
}

}  // namespace

CloudLandmarkExtractorResult extract_cloud_landmark_observations(
  const std::vector<gtsam::Point3>& points_sensor,
  const gtsam::Pose3& pose_world_sensor,
  double stamp,
  const CloudLandmarkExtractorParams& params) {
  CloudLandmarkExtractorResult result;
  result.input_points = points_sensor.size();

  const double voxel_resolution = std::max(params.voxel_resolution, 1e-6);
  const int min_points_per_landmark = std::max(params.min_points_per_landmark, 1);
  const double min_covariance = std::max(params.min_covariance, 1e-12);
  const double confidence_point_count = std::max(params.confidence_point_count, 1.0);
  const std::string class_id = params.class_id.empty() ? std::string("cloud_landmark") : params.class_id;

  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHasher> voxels;
  voxels.reserve(points_sensor.size());

  for (const auto& point_sensor : points_sensor) {
    if (!point_sensor.allFinite()) {
      continue;
    }
    result.finite_points++;

    if (!passes_range(point_sensor, params.min_range, params.max_range)) {
      result.range_rejected++;
      continue;
    }

    const gtsam::Point3 point_world = pose_world_sensor.transformFrom(point_sensor);
    if (!point_world.allFinite()) {
      continue;
    }

    const VoxelKey key{
      voxel_coord(point_world.x(), voxel_resolution),
      voxel_coord(point_world.y(), voxel_resolution),
      voxel_coord(point_world.z(), voxel_resolution)};

    auto& acc = voxels[key];
    acc.count++;
    acc.sum_sensor += point_sensor;
    acc.sum_outer_sensor += point_sensor * point_sensor.transpose();
  }

  result.voxel_candidates = voxels.size();

  std::vector<Candidate> candidates;
  candidates.reserve(voxels.size());
  for (const auto& item : voxels) {
    const VoxelKey& key = item.first;
    const Accumulator& acc = item.second;
    if (static_cast<int>(acc.count) < min_points_per_landmark) {
      result.rejected_min_points++;
      continue;
    }

    const Eigen::Vector3d mean = acc.sum_sensor / static_cast<double>(acc.count);
    PerceptionObservation observation;
    observation.stamp = stamp;
    observation.class_id = class_id;
    observation.landmark_id = stable_landmark_id(key, class_id);
    observation.position_sensor = mean;
    observation.covariance = covariance_for(acc, mean, min_covariance);
    observation.confidence = std::min(1.0, static_cast<double>(acc.count) / confidence_point_count);
    candidates.push_back({observation, acc.count});
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
    if (lhs.count != rhs.count) {
      return lhs.count > rhs.count;
    }
    return lhs.observation.landmark_id < rhs.observation.landmark_id;
  });

  if (params.max_landmarks > 0 && candidates.size() > params.max_landmarks) {
    candidates.resize(params.max_landmarks);
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
    return lhs.observation.landmark_id < rhs.observation.landmark_id;
  });

  result.observations.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    result.observations.push_back(candidate.observation);
  }
  return result;
}

}  // namespace glil
