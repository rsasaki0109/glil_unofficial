// SPDX-License-Identifier: MIT
// IntegratedVGICPCoresetFactor implementation
// Reference: Koide et al., ICRA 2025

#include <glil/factors/integrated_vgicp_coreset_factor.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <caratheodory.hpp>

namespace glil {

using gtsam_points::GaussianVoxel;
using gtsam_points::GaussianVoxelMapCPU;

namespace {
template <typename T>
int point_size(const T& cloud) { return gtsam_points::frame::size(cloud); }

template <typename T>
const Eigen::Vector4d& point_at(const T& cloud, int i) { return gtsam_points::frame::point(cloud, i); }

template <typename T>
const Eigen::Matrix4d& cov_at(const T& cloud, int i) { return gtsam_points::frame::cov(cloud, i); }
}  // namespace

IntegratedVGICPCoresetFactor::IntegratedVGICPCoresetFactor(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxels,
  const gtsam_points::PointCloud::ConstPtr& source,
  const Params& params)
: IntegratedMatchingCostFactor(target_key, source_key),
  coreset_params(params),
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source),
  coreset_valid(false) {
  linearization_point.setIdentity();
  last_coreset_delta.setIdentity();
}

IntegratedVGICPCoresetFactor::IntegratedVGICPCoresetFactor(
  const gtsam::Pose3& fixed_target_pose,
  gtsam::Key source_key,
  const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxels,
  const gtsam_points::PointCloud::ConstPtr& source,
  const Params& params)
: IntegratedMatchingCostFactor(fixed_target_pose, source_key),
  coreset_params(params),
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source),
  coreset_valid(false) {
  linearization_point.setIdentity();
  last_coreset_delta.setIdentity();
}

IntegratedVGICPCoresetFactor::~IntegratedVGICPCoresetFactor() {}

gtsam::NonlinearFactor::shared_ptr IntegratedVGICPCoresetFactor::clone() const {
  return std::make_shared<IntegratedVGICPCoresetFactor>(*this);
}

void IntegratedVGICPCoresetFactor::update_correspondences(const Eigen::Isometry3d& delta) const {
  linearization_point = delta;
  const int N = point_size(*source);
  correspondences.resize(N);
  mahalanobis.resize(N);

#pragma omp parallel for num_threads(coreset_params.num_threads) schedule(guided, 8)
  for (int i = 0; i < N; i++) {
    Eigen::Vector4d pt = delta * point_at(*source, i);
    Eigen::Vector3i coord = target_voxels->voxel_coord(pt);
    const auto voxel_id = target_voxels->lookup_voxel_index(coord);

    if (voxel_id < 0) {
      correspondences[i] = nullptr;
      mahalanobis[i].setZero();
    } else {
      const auto voxel = &target_voxels->lookup_voxel(voxel_id);
      correspondences[i] = voxel;

      const Eigen::Matrix4d RCR = voxel->cov + delta.matrix() * cov_at(*source, i) * delta.matrix().transpose();
      mahalanobis[i].setZero();
      mahalanobis[i].template topLeftCorner<3, 3>() = RCR.topLeftCorner<3, 3>().inverse();
    }
  }
}

bool IntegratedVGICPCoresetFactor::needs_coreset_update(const Eigen::Isometry3d& delta) const {
  if (!coreset_valid) {
    return true;
  }
  const Eigen::Isometry3d diff = last_coreset_delta.inverse() * delta;
  const double trans_diff = diff.translation().norm();
  const double rot_diff = Eigen::AngleAxisd(diff.linear()).angle() * 180.0 / M_PI;
  return trans_diff > coreset_params.relinearize_thresh_trans || rot_diff > coreset_params.relinearize_thresh_rot;
}

void IntegratedVGICPCoresetFactor::extract_coreset(const Eigen::Isometry3d& delta) const {
  const int N = point_size(*source);

  // Collect per-point Jacobians and residuals for valid correspondences
  // For VGICP, each point contributes a weighted residual
  // We compute J_source (6x1 per dimension, 3 residual dims) and pack into Nx6 Jacobian

  // First pass: count valid correspondences
  std::vector<int> valid_indices;
  valid_indices.reserve(N);
  for (int i = 0; i < N; i++) {
    if (correspondences[i] != nullptr) {
      valid_indices.push_back(i);
    }
  }

  const int num_valid = valid_indices.size();
  if (num_valid < coreset_params.coreset_target_size) {
    // Too few points for coreset extraction, use all
    coreset_indices.resize(num_valid);
    coreset_weights.resize(num_valid);
    for (int i = 0; i < num_valid; i++) {
      coreset_indices[i] = valid_indices[i];
      coreset_weights[i] = 1.0;
    }
    coreset_valid = true;
    last_coreset_delta = delta;
    return;
  }

  // For each valid point, compute the 3 residual components and their Jacobians w.r.t. source pose (6 DOF)
  // Total: 3*num_valid rows, 6 columns
  const int num_residuals = 3 * num_valid;
  Eigen::Matrix<double, -1, 6> J(num_residuals, 6);
  Eigen::VectorXd e(num_residuals);

  for (int idx = 0; idx < num_valid; idx++) {
    const int i = valid_indices[idx];
    const auto& mean_A = point_at(*source, i);
    const auto& mean_B = correspondences[i]->mean;

    Eigen::Vector4d transed_mean_A = delta * mean_A;
    Eigen::Vector4d residual = mean_B - transed_mean_A;

    // Mahalanobis sqrt decomposition: M = L * L^T (Cholesky)
    // Weighted residual: L^T * r, Weighted Jacobian: L^T * J
    Eigen::Matrix3d M3 = mahalanobis[i].topLeftCorner<3, 3>();

    // Use Cholesky decomposition for weighting
    Eigen::LLT<Eigen::Matrix3d> llt(M3);
    if (llt.info() != Eigen::Success) {
      // Fallback: use sqrt of diagonal
      Eigen::Matrix3d L = M3.diagonal().cwiseSqrt().asDiagonal();
      Eigen::Vector3d weighted_r = L * residual.head<3>();

      Eigen::Matrix<double, 3, 6> J_source_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
      J_source_3x6.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
      J_source_3x6.block<3, 3>(0, 3) = -delta.linear();
      Eigen::Matrix<double, 3, 6> weighted_J = L * J_source_3x6;

      J.block<3, 6>(idx * 3, 0) = weighted_J;
      e.segment<3>(idx * 3) = weighted_r;
    } else {
      Eigen::Matrix3d L = llt.matrixL();
      Eigen::Vector3d weighted_r = L.transpose() * residual.head<3>();

      Eigen::Matrix<double, 3, 6> J_source_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
      J_source_3x6.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
      J_source_3x6.block<3, 3>(0, 3) = -delta.linear();
      Eigen::Matrix<double, 3, 6> weighted_J = L.transpose() * J_source_3x6;

      J.block<3, 6>(idx * 3, 0) = weighted_J;
      e.segment<3>(idx * 3) = weighted_r;
    }
  }

  // Run Caratheodory coreset extraction
  Eigen::VectorXi raw_indices;
  Eigen::VectorXd raw_weights;
  const int target_N = std::max(29, coreset_params.coreset_target_size);

  exd::fast_caratheodory_quadratic(J, e, coreset_params.coreset_num_clusters, raw_indices, raw_weights, target_N);

  // Map coreset residual indices back to point indices
  // Each point generates 3 residuals (rows idx*3, idx*3+1, idx*3+2)
  // The coreset selects individual residual rows
  coreset_indices.resize(raw_indices.size());
  coreset_weights.resize(raw_indices.size());
  for (int i = 0; i < raw_indices.size(); i++) {
    // Map residual index to point index in valid_indices
    int residual_idx = raw_indices[i];
    int point_local_idx = residual_idx / 3;
    coreset_indices[i] = valid_indices[point_local_idx];
    coreset_weights[i] = raw_weights[i];
  }

  coreset_valid = true;
  last_coreset_delta = delta;
}

double IntegratedVGICPCoresetFactor::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  //
  if (correspondences.size() != static_cast<size_t>(point_size(*source))) {
    update_correspondences(delta);
  }

  // Check if we need to re-extract the coreset
  if (needs_coreset_update(delta)) {
    extract_coreset(delta);
  }

  double sum_errors = 0.0;

  if (!coreset_valid || coreset_indices.size() == 0) {
    return 0.0;
  }

  // Evaluate using coreset points only
  for (int ci = 0; ci < coreset_indices.size(); ci++) {
    const int i = coreset_indices[ci];
    const double w = coreset_weights[ci];

    const auto& target_voxel = correspondences[i];
    if (target_voxel == nullptr) {
      continue;
    }

    const auto& mean_A = point_at(*source, i);
    const auto& mean_B = target_voxel->mean;

    Eigen::Vector4d transed_mean_A = delta * mean_A;
    Eigen::Vector4d residual = mean_B - transed_mean_A;

    const Eigen::Matrix4d& M = mahalanobis[i];
    const double error = w * static_cast<double>(residual.transpose() * M * residual);
    sum_errors += error;

    if (!H_target) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.template head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    Eigen::Matrix<double, 6, 4> J_target_mahalanobis = w * J_target.transpose() * M;
    Eigen::Matrix<double, 6, 4> J_source_mahalanobis = w * J_source.transpose() * M;

    *H_target += J_target_mahalanobis * J_target;
    *H_source += J_source_mahalanobis * J_source;
    *H_target_source += J_target_mahalanobis * J_source;
    *b_target += J_target_mahalanobis * residual;
    *b_source += J_source_mahalanobis * residual;
  }

  return sum_errors;
}

}  // namespace glil
