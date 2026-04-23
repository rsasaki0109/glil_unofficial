// SPDX-License-Identifier: MIT
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glil/factors/integrated_vgicp_coreset_factor.hpp>

namespace {
using gtsam::symbol_shorthand::X;

std::vector<Eigen::Vector4d> make_points(const int rows, const int cols) {
  std::vector<Eigen::Vector4d> points;
  points.reserve(rows * cols);
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      points.emplace_back(0.25 * c, 0.25 * r, 0.05 * ((r + c) % 3), 1.0);
    }
  }
  return points;
}

gtsam_points::PointCloudCPU::Ptr make_cloud() {
  const auto points = make_points(6, 8);
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);

  std::vector<Eigen::Matrix4d> covariances(points.size(), Eigen::Matrix4d::Zero());
  for (auto& covariance : covariances) {
    covariance.topLeftCorner<3, 3>() = 0.01 * Eigen::Matrix3d::Identity();
  }
  cloud->add_covs(covariances);
  return cloud;
}

void test_stats_after_linearize() {
  auto target_cloud = make_cloud();
  auto source_cloud = make_cloud();

  auto target_voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
  target_voxels->insert(*target_cloud);

  glil::IntegratedVGICPCoresetFactor::Params params;
  params.coreset_target_size = 32;
  params.coreset_num_clusters = 64;
  params.coreset_method = "exact_caratheodory";
  params.num_threads = 1;

  glil::IntegratedVGICPCoresetFactor factor(gtsam::Pose3(), X(0), target_voxels, source_cloud, params);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3());
  const auto linearized = factor.linearize(values);
  assert(linearized);

  const auto stats = factor.coreset_stats();
  assert(stats.valid);
  assert(!stats.immutable_snapshot);
  assert(stats.source_points == static_cast<int>(source_cloud->size()));
  assert(stats.valid_correspondences > 0);
  assert(stats.selected_residual_rows > 0);
  assert(stats.selected_residual_rows <= stats.valid_correspondences * 3);
  assert(stats.selected_points > 0);
  assert(stats.target_size == 32);
  assert(stats.num_clusters == 64);
  assert(stats.method == "exact_caratheodory");
  assert(stats.correspondence_update_count >= 1);
  assert(stats.coreset_extraction_count >= 1);
  assert(stats.weight_sum > 0.0);
  assert(std::abs(stats.last_translation_norm) < 1e-12);
  assert(std::abs(stats.last_rotation_deg) < 1e-12);
}

void test_empty_stats_before_linearize() {
  auto source_cloud = make_cloud();
  auto target_voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);

  glil::IntegratedVGICPCoresetFactor factor(gtsam::Pose3(), X(0), target_voxels, source_cloud);
  const auto stats = factor.coreset_stats();
  assert(!stats.valid);
  assert(stats.source_points == static_cast<int>(source_cloud->size()));
  assert(stats.valid_correspondences == 0);
  assert(stats.selected_residual_rows == 0);
  assert(stats.coreset_extraction_count == 0);
}
}  // namespace

int main() {
  test_empty_stats_before_linearize();
  test_stats_after_linearize();
  return 0;
}
