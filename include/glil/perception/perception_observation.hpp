// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <string>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>

namespace glil {

/**
 * @brief External perception observation that can be converted into graph factors.
 *
 * The observation is intentionally frontend-agnostic: a detector, tracker, ROS
 * node, or offline parser can populate it without linking a perception model into
 * the GLIL core.
 */
struct PerceptionObservation {
  double stamp = 0.0;                    ///< Sensor timestamp in seconds.
  std::string class_id;                  ///< Optional semantic label, e.g. pole, sign, cone.
  std::uint64_t landmark_id = 0;         ///< Stable landmark/track ID from the perception frontend.
  gtsam::Point3 position_sensor = gtsam::Point3::Zero();  ///< Landmark position in the sensor/body frame.
  gtsam::Matrix3 covariance = gtsam::Matrix3::Identity(); ///< 3x3 measurement covariance in sensor frame.
  double confidence = 1.0;               ///< Frontend confidence in [0, 1].

  bool valid(double min_confidence = 0.0) const;
};

/**
 * @brief Build a diagonal noise model from observation covariance and confidence.
 *
 * Confidence is mapped as sigma / sqrt(confidence), so lower confidence weakens
 * the factor while preserving the per-axis covariance scale. Values are clamped to
 * keep the resulting model numerically well-conditioned.
 */
gtsam::SharedNoiseModel make_perception_noise_model(
  const PerceptionObservation& observation,
  double min_sigma = 1e-3,
  double min_confidence = 1e-3);

}  // namespace glil
