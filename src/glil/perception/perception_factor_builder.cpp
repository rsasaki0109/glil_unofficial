// SPDX-License-Identifier: MIT
#include <glil/perception/perception_factor_builder.hpp>

#include <algorithm>
#include <cctype>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glil/factors/perception_landmark_factor.hpp>

namespace glil {
namespace {

std::string normalized_robust_loss(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return value;
}

gtsam::noiseModel::mEstimator::Base::shared_ptr make_perception_robust_loss(const PerceptionFactorBuilderParams& params) {
  if (params.robust_loss_width <= 0.0) {
    return nullptr;
  }

  const std::string robust_loss = normalized_robust_loss(params.robust_loss);
  if (robust_loss.empty() || robust_loss == "NONE" || robust_loss == "OFF" || robust_loss == "L2") {
    return nullptr;
  }
  if (robust_loss == "HUBER") {
    return gtsam::noiseModel::mEstimator::Huber::Create(params.robust_loss_width);
  }
  if (robust_loss == "CAUCHY") {
    return gtsam::noiseModel::mEstimator::Cauchy::Create(params.robust_loss_width);
  }
  if (robust_loss == "TUKEY") {
    return gtsam::noiseModel::mEstimator::Tukey::Create(params.robust_loss_width);
  }

  return nullptr;
}

gtsam::SharedNoiseModel make_builder_noise_model(const PerceptionObservation& observation, const PerceptionFactorBuilderParams& params) {
  auto noise = make_perception_noise_model(observation, params.min_sigma, params.min_noise_confidence);
  const auto robust = make_perception_robust_loss(params);
  if (!robust) {
    return noise;
  }
  return gtsam::noiseModel::Robust::Create(robust, noise);
}

}  // namespace

PerceptionFactorBuilder::PerceptionFactorBuilder(PerceptionFactorBuilderParams params) : params_(std::move(params)) {}

gtsam::Key PerceptionFactorBuilder::landmark_key(std::uint64_t landmark_id) const {
  return gtsam::Symbol(params_.landmark_symbol, landmark_id);
}

bool PerceptionFactorBuilder::accepts_class(const std::string& class_id) const {
  if (!params_.allowed_class_ids.empty() && params_.allowed_class_ids.count(class_id) == 0) {
    return false;
  }
  return params_.rejected_class_ids.count(class_id) == 0;
}

bool PerceptionFactorBuilder::accepts(const PerceptionObservation& observation) const {
  return observation.valid(params_.min_confidence) && accepts_class(observation.class_id);
}

PerceptionFactorBuilderResult PerceptionFactorBuilder::add_observation(
  gtsam::NonlinearFactorGraph& graph,
  gtsam::Values& values,
  gtsam::Key pose_key,
  const gtsam::Pose3& pose_world_sensor,
  const PerceptionObservation& observation,
  const gtsam::Values* existing_values) const {
  PerceptionFactorBuilderResult result;

  if (!observation.valid(params_.min_confidence)) {
    result.rejected_invalid++;
    return result;
  }

  if (!accepts_class(observation.class_id)) {
    result.rejected_class++;
    return result;
  }

  const gtsam::Key l_key = landmark_key(observation.landmark_id);
  const bool exists_in_pending = values.exists(l_key);
  const bool exists_in_graph = existing_values != nullptr && existing_values->exists(l_key);
  if (!exists_in_pending && !exists_in_graph) {
    if (!params_.initialize_missing_landmarks) {
      result.rejected_missing_landmark++;
      return result;
    }

    values.insert(l_key, pose_world_sensor.transformFrom(observation.position_sensor));
    result.inserted_landmarks++;
  } else {
    result.reused_landmarks++;
  }

  const auto noise = make_builder_noise_model(observation, params_);
  graph.emplace_shared<PerceptionLandmarkFactor>(pose_key, l_key, observation, noise);
  result.accepted++;
  return result;
}

PerceptionFactorBuilderResult PerceptionFactorBuilder::add_observations(
  gtsam::NonlinearFactorGraph& graph,
  gtsam::Values& values,
  gtsam::Key pose_key,
  const gtsam::Pose3& pose_world_sensor,
  const std::vector<PerceptionObservation>& observations,
  const gtsam::Values* existing_values) const {
  PerceptionFactorBuilderResult result;
  for (const auto& observation : observations) {
    result += add_observation(graph, values, pose_key, pose_world_sensor, observation, existing_values);
  }
  return result;
}

PerceptionFactorBuilderResult& operator+=(PerceptionFactorBuilderResult& lhs, const PerceptionFactorBuilderResult& rhs) {
  lhs.accepted += rhs.accepted;
  lhs.rejected_invalid += rhs.rejected_invalid;
  lhs.rejected_class += rhs.rejected_class;
  lhs.rejected_missing_landmark += rhs.rejected_missing_landmark;
  lhs.inserted_landmarks += rhs.inserted_landmarks;
  lhs.reused_landmarks += rhs.reused_landmarks;
  return lhs;
}

}  // namespace glil
