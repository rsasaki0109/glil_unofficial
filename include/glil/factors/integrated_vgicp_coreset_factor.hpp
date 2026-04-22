// SPDX-License-Identifier: MIT
// IntegratedVGICPCoresetFactor: VGICP with Caratheodory coreset extraction
// Reference: Koide et al., ICRA 2025

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
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
    std::string coreset_method;
    double correspondence_sample_ratio;
    int num_threads;
    bool coreset_factor_lock;
    bool coreset_immutable_snapshot;
    bool debug_digest;
    int debug_frame_id;
    int debug_factor_idx;

    Params()
    : relinearize_thresh_trans(0.25),
      relinearize_thresh_rot(0.25),
      coreset_target_size(256),
      coreset_num_clusters(64),
      coreset_method("exact_caratheodory"),
      correspondence_sample_ratio(1.0),
      num_threads(2),
      coreset_factor_lock(false),
      coreset_immutable_snapshot(false),
      debug_digest(false),
      debug_frame_id(-1),
      debug_factor_idx(-1) {}
  };

  struct CoresetSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Isometry3d linearization_point = Eigen::Isometry3d::Identity();
    Eigen::VectorXi coreset_indices;
    Eigen::VectorXi coreset_residual_rows;
    Eigen::VectorXd coreset_weights;
    Eigen::Matrix<double, Eigen::Dynamic, 6> J_target_rows;
    Eigen::Matrix<double, Eigen::Dynamic, 6> J_source_rows;
    Eigen::VectorXd e_rows;
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

  IntegratedVGICPCoresetFactor(const IntegratedVGICPCoresetFactor& other);

  virtual ~IntegratedVGICPCoresetFactor() override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

  void set_num_threads(int n) { coreset_params.num_threads = n; }

private:
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;
  void update_correspondences_unlocked(const Eigen::Isometry3d& delta) const;

  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override;

  void extract_coreset(const Eigen::Isometry3d& delta) const;
  void extract_coreset_unlocked(const Eigen::Isometry3d& delta) const;
  std::shared_ptr<const CoresetSnapshot> update_snapshot(const Eigen::Isometry3d& delta) const;
  std::shared_ptr<const CoresetSnapshot> load_snapshot() const;
  void populate_snapshot_rows(CoresetSnapshot& snapshot) const;
  double evaluate_snapshot(
    const CoresetSnapshot& snapshot,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const;

private:
  Params coreset_params;

  std::shared_ptr<const gtsam_points::GaussianVoxelMapCPU> target_voxels;
  gtsam_points::PointCloud::ConstPtr source;

  // Correspondence data (mutable for const linearize)
  mutable std::mutex coreset_mutex_;
  mutable Eigen::Isometry3d linearization_point;
  mutable std::vector<const gtsam_points::GaussianVoxel*> correspondences;
  mutable std::vector<Eigen::Matrix4d> mahalanobis;

  // Coreset data
  mutable bool coreset_valid;
  mutable Eigen::Isometry3d last_coreset_delta;
  mutable Eigen::VectorXi coreset_indices;      // point index for each coreset entry
  mutable Eigen::VectorXi coreset_residual_rows; // residual row within point (0,1,2) for each coreset entry
  mutable Eigen::VectorXd coreset_weights;
  mutable std::shared_ptr<const CoresetSnapshot> snapshot_;

  // Debug verification counter
  mutable int debug_count = 0;
};

}  // namespace glil
