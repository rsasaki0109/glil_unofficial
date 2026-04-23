// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <glil/perception/perception_observation.hpp>

namespace glil {

struct CloudLandmarkExtractorParams {
  double voxel_resolution = 1.0;
  int min_points_per_landmark = 5;
  std::size_t max_landmarks = 0;
  double min_range = 0.0;
  double max_range = 0.0;
  double min_covariance = 0.01;
  double confidence_point_count = 20.0;
  std::string class_id = "cloud_landmark";
};

struct CloudLandmarkExtractorResult {
  std::vector<PerceptionObservation> observations;
  std::size_t input_points = 0;
  std::size_t finite_points = 0;
  std::size_t range_rejected = 0;
  std::size_t voxel_candidates = 0;
  std::size_t rejected_min_points = 0;
};

CloudLandmarkExtractorResult extract_cloud_landmark_observations(
  const std::vector<gtsam::Point3>& points_sensor,
  const gtsam::Pose3& pose_world_sensor,
  double stamp,
  const CloudLandmarkExtractorParams& params = CloudLandmarkExtractorParams());

}  // namespace glil
