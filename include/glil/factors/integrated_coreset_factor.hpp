// SPDX-License-Identifier: MIT
// Coreset-based exact downsampling wrapper for IntegratedMatchingCostFactor
// Reference: Koide et al., "Tightly Coupled Range Inertial Odometry and Mapping
//            with Exact Point Cloud Downsampling", ICRA 2025

#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>

namespace glil {

/**
 * @brief Coreset-based exact downsampling wrapper for IntegratedMatchingCostFactor.
 *
 * Wraps an existing VGICP/GICP factor and applies Caratheodory coreset extraction
 * to dramatically reduce the number of points used in linearization while preserving
 * the EXACT Hessian, gradient, and error.
 *
 * Uses deferred sampling: coreset is only re-extracted when the pose drifts
 * beyond configurable thresholds from the last extraction point.
 */
class IntegratedCoresetFactor : public gtsam::NonlinearFactor {
public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedCoresetFactor>;

  struct Params {
    double relinearize_thresh_trans;  ///< Translation threshold for re-extraction (m)
    double relinearize_thresh_rot;    ///< Rotation threshold for re-extraction (deg)
    int coreset_target_size;          ///< Target coreset size
    int coreset_num_clusters;         ///< Number of clusters for Fast-Caratheodory

    Params() : relinearize_thresh_trans(0.25), relinearize_thresh_rot(0.25), coreset_target_size(256), coreset_num_clusters(64) {}
  };

  /**
   * @brief Binary factor constructor
   */
  IntegratedCoresetFactor(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_points::IntegratedMatchingCostFactor::shared_ptr& inner_factor,
    const Params& params = Params());

  /**
   * @brief Unary factor constructor (fixed target pose)
   */
  IntegratedCoresetFactor(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const gtsam_points::IntegratedMatchingCostFactor::shared_ptr& inner_factor,
    const Params& params = Params());

  virtual ~IntegratedCoresetFactor() override;

  virtual size_t dim() const override { return 6; }
  virtual double error(const gtsam::Values& values) const override;
  virtual gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;
  virtual gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  bool needs_relinearization(const Eigen::Isometry3d& current_delta) const;

  Params coreset_params;
  gtsam_points::IntegratedMatchingCostFactor::shared_ptr inner_factor;
  bool is_binary;

  // Cached coreset data (mutable for const linearize)
  mutable bool coreset_valid;
  mutable Eigen::Isometry3d last_extraction_delta;

  // Cached linearization result
  mutable Eigen::Matrix<double, 6, 6> cached_H_target;
  mutable Eigen::Matrix<double, 6, 6> cached_H_source;
  mutable Eigen::Matrix<double, 6, 6> cached_H_target_source;
  mutable Eigen::Matrix<double, 6, 1> cached_b_target;
  mutable Eigen::Matrix<double, 6, 1> cached_b_source;
  mutable double cached_error;
};

}  // namespace glil
