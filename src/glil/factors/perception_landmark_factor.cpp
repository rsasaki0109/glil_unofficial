// SPDX-License-Identifier: MIT
#include <glil/factors/perception_landmark_factor.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

namespace glil {

bool PerceptionObservation::valid(double min_confidence) const {
  if (!std::isfinite(stamp) || !std::isfinite(confidence) || confidence < min_confidence || !position_sensor.allFinite() || !covariance.allFinite()) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    if (covariance(i, i) < 0.0) {
      return false;
    }
  }
  return true;
}

gtsam::SharedNoiseModel make_perception_noise_model(const PerceptionObservation& observation, double min_sigma, double min_confidence) {
  const double safe_min_sigma = std::max(min_sigma, 1e-9);
  const double safe_min_confidence = std::max(min_confidence, 1e-9);
  const double safe_confidence = std::max(observation.confidence, safe_min_confidence);

  gtsam::Vector3 sigmas;
  for (int i = 0; i < 3; i++) {
    const double variance = std::max(observation.covariance(i, i), safe_min_sigma * safe_min_sigma);
    sigmas[i] = std::max(std::sqrt(variance / safe_confidence), safe_min_sigma);
  }

  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

PerceptionLandmarkFactor::PerceptionLandmarkFactor(
  gtsam::Key pose_key,
  gtsam::Key landmark_key,
  const gtsam::Point3& measured_position_sensor,
  const gtsam::SharedNoiseModel& noise_model,
  double stamp,
  std::string class_id,
  std::uint64_t landmark_id,
  double confidence)
: Base(noise_model, pose_key, landmark_key),
  measured_position_sensor_(measured_position_sensor),
  stamp_(stamp),
  class_id_(std::move(class_id)),
  landmark_id_(landmark_id),
  confidence_(confidence) {}

PerceptionLandmarkFactor::PerceptionLandmarkFactor(
  gtsam::Key pose_key,
  gtsam::Key landmark_key,
  const PerceptionObservation& observation,
  const gtsam::SharedNoiseModel& noise_model)
: PerceptionLandmarkFactor(
    pose_key,
    landmark_key,
    observation.position_sensor,
    noise_model,
    observation.stamp,
    observation.class_id,
    observation.landmark_id,
    observation.confidence) {}

gtsam::Vector PerceptionLandmarkFactor::evaluateError(
  const gtsam::Pose3& pose,
  const gtsam::Point3& landmark_world,
  gtsam::OptionalMatrixType H_pose,
  gtsam::OptionalMatrixType H_landmark) const {
  return pose.transformTo(landmark_world, H_pose, H_landmark) - measured_position_sensor_;
}

gtsam::NonlinearFactor::shared_ptr PerceptionLandmarkFactor::clone() const {
  return std::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new PerceptionLandmarkFactor(*this)));
}

void PerceptionLandmarkFactor::print(const std::string& s, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << s << "PerceptionLandmarkFactor(" << key_formatter(this->key1()) << ", " << key_formatter(this->key2()) << ")\n"
            << "  measured_position_sensor: " << measured_position_sensor_.transpose() << "\n"
            << "  stamp: " << stamp_ << "\n"
            << "  class_id: " << class_id_ << "\n"
            << "  landmark_id: " << landmark_id_ << "\n"
            << "  confidence: " << confidence_ << std::endl;
  if (this->noiseModel_) {
    this->noiseModel_->print("  noise model: ");
  }
}

bool PerceptionLandmarkFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const {
  const auto* factor = dynamic_cast<const PerceptionLandmarkFactor*>(&expected);
  if (factor == nullptr) {
    return false;
  }

  return Base::equals(*factor, tol) && (measured_position_sensor_ - factor->measured_position_sensor_).norm() <= tol &&
         std::abs(stamp_ - factor->stamp_) <= tol && class_id_ == factor->class_id_ && landmark_id_ == factor->landmark_id_ &&
         std::abs(confidence_ - factor->confidence_) <= tol;
}

}  // namespace glil
