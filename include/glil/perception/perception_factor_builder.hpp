// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>

#include <glil/perception/perception_observation.hpp>

namespace gtsam {
class NonlinearFactorGraph;
class Values;
}  // namespace gtsam

namespace glil {

struct PerceptionFactorBuilderParams {
  double min_confidence = 0.0;
  double min_sigma = 1e-3;
  double min_noise_confidence = 1e-3;
  char landmark_symbol = 'l';
  bool initialize_missing_landmarks = true;
  std::unordered_set<std::string> allowed_class_ids;
  std::unordered_set<std::string> rejected_class_ids;
};

struct PerceptionFactorBuilderResult {
  std::size_t accepted = 0;
  std::size_t rejected_invalid = 0;
  std::size_t rejected_class = 0;
  std::size_t rejected_missing_landmark = 0;
  std::size_t inserted_landmarks = 0;
  std::size_t reused_landmarks = 0;
};

class PerceptionFactorBuilder {
public:
  explicit PerceptionFactorBuilder(PerceptionFactorBuilderParams params = PerceptionFactorBuilderParams());

  const PerceptionFactorBuilderParams& params() const { return params_; }

  gtsam::Key landmark_key(std::uint64_t landmark_id) const;
  bool accepts_class(const std::string& class_id) const;
  bool accepts(const PerceptionObservation& observation) const;

  PerceptionFactorBuilderResult add_observation(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values,
    gtsam::Key pose_key,
    const gtsam::Pose3& pose_world_sensor,
    const PerceptionObservation& observation,
    const gtsam::Values* existing_values = nullptr) const;

  PerceptionFactorBuilderResult add_observations(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values,
    gtsam::Key pose_key,
    const gtsam::Pose3& pose_world_sensor,
    const std::vector<PerceptionObservation>& observations,
    const gtsam::Values* existing_values = nullptr) const;

private:
  PerceptionFactorBuilderParams params_;
};

PerceptionFactorBuilderResult& operator+=(PerceptionFactorBuilderResult& lhs, const PerceptionFactorBuilderResult& rhs);

}  // namespace glil
