#pragma once

#include <Eigen/Geometry>

namespace glim {

inline bool imuPredictionDiscontinuous(
  const Eigen::Isometry3d& last_pose,
  const Eigen::Isometry3d& predicted_pose,
  const Eigen::Vector3d& predicted_velocity,
  double max_translation_m) {
  if (!predicted_pose.matrix().allFinite() || !predicted_velocity.allFinite()) {
    return true;
  }
  return max_translation_m > 0.0 &&
         (last_pose.inverse() * predicted_pose).translation().norm() > max_translation_m;
}

}  // namespace glim
