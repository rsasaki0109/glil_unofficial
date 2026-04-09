// SPDX-License-Identifier: MIT
// IntegratedVGICPCoresetFactor: VGICP with Caratheodory coreset extraction
// Reference: Koide et al., ICRA 2025

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>

namespace glil {

/**
 * @brief VGICP factor with Caratheodory coreset extraction.
 *
 * Instead of iterating over ALL source points for linearization,
 * extracts a small coreset (~O(d^2) points) that preserves the exact
 * quadratic error function (H, b, c).
 *
 * Deferred sampling: the coreset is only re-extracted when the pose
 * drifts beyond configurable thresholds.
 */
class IntegratedVGICPCoresetFactor : public gtsam_points::IntegratedMatchingCostFactor {
public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

  struct Params {
    double relinearize_thresh_trans;
    double relinearize_thresh_rot;
    int coreset_target_size;
    int coreset_num_clusters;
    int num_threads;

    Params() : relinearize_thresh_trans(0.25), relinearize_thresh_rot(0.25), coreset_target_size(256), coreset_num_clusters(64), num_threads(2) {}
  };

  /**
   * @brief Binary factor constructor
   */
  IntegratedVGICPCoresetFactor(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxels,
    const gtsam_points::PointCloud::ConstPtr& source,
    const Params& params = Params());

  /**
   * @brief Unary factor constructor
   */
  IntegratedVGICPCoresetFactor(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxels,
    const gtsam_points::PointCloud::ConstPtr& source,
    const Params& params = Params());

  virtual ~IntegratedVGICPCoresetFactor() override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

  void set_num_threads(int n) { coreset_params.num_threads = n; }

private:
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;

  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override;

  bool needs_coreset_update(const Eigen::Isometry3d& delta) const;
  void extract_coreset(const Eigen::Isometry3d& delta) const;

private:
  Params coreset_params;

  std::shared_ptr<const gtsam_points::GaussianVoxelMapCPU> target_voxels;
  gtsam_points::PointCloud::ConstPtr source;

  // Correspondence data (mutable for const linearize)
  mutable Eigen::Isometry3d linearization_point;
  mutable std::vector<const gtsam_points::GaussianVoxel*> correspondences;
  mutable std::vector<Eigen::Matrix4d> mahalanobis;

  // Coreset data
  mutable bool coreset_valid;
  mutable Eigen::Isometry3d last_coreset_delta;
  mutable Eigen::VectorXi coreset_indices;
  mutable Eigen::VectorXd coreset_weights;
};

}  // namespace glil
