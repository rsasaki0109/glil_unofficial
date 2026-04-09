// SPDX-License-Identifier: MIT
// Coreset-based exact downsampling wrapper for IntegratedMatchingCostFactor
// Reference: Koide et al., ICRA 2025

#include <glil/factors/integrated_coreset_factor.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>

namespace glil {

IntegratedCoresetFactor::IntegratedCoresetFactor(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const gtsam_points::IntegratedMatchingCostFactor::shared_ptr& inner_factor,
  const Params& params)
: gtsam::NonlinearFactor(gtsam::KeyVector{target_key, source_key}),
  coreset_params(params),
  inner_factor(inner_factor),
  is_binary(true),
  coreset_valid(false) {
  last_extraction_delta.setIdentity();
}

IntegratedCoresetFactor::IntegratedCoresetFactor(
  const gtsam::Pose3& fixed_target_pose,
  gtsam::Key source_key,
  const gtsam_points::IntegratedMatchingCostFactor::shared_ptr& inner_factor,
  const Params& params)
: gtsam::NonlinearFactor(gtsam::KeyVector{source_key}),
  coreset_params(params),
  inner_factor(inner_factor),
  is_binary(false),
  coreset_valid(false) {
  last_extraction_delta.setIdentity();
}

IntegratedCoresetFactor::~IntegratedCoresetFactor() {}

bool IntegratedCoresetFactor::needs_relinearization(const Eigen::Isometry3d& current_delta) const {
  if (!coreset_valid) {
    return true;
  }

  const Eigen::Isometry3d diff = last_extraction_delta.inverse() * current_delta;
  const double trans_diff = diff.translation().norm();
  const double rot_diff = Eigen::AngleAxisd(diff.linear()).angle() * 180.0 / M_PI;

  return trans_diff > coreset_params.relinearize_thresh_trans || rot_diff > coreset_params.relinearize_thresh_rot;
}

double IntegratedCoresetFactor::error(const gtsam::Values& values) const {
  return inner_factor->error(values);
}

gtsam::GaussianFactor::shared_ptr IntegratedCoresetFactor::linearize(const gtsam::Values& values) const {
  // Compute current delta (T_target_source)
  const Eigen::Isometry3d current_delta = inner_factor->calc_delta(values);

  if (needs_relinearization(current_delta)) {
    // Full linearization: delegate to inner factor
    // This computes over ALL points and returns exact H, b, c
    inner_factor->update_correspondences(current_delta);

    cached_H_target.setZero();
    cached_H_source.setZero();
    cached_H_target_source.setZero();
    cached_b_target.setZero();
    cached_b_source.setZero();

    cached_error = inner_factor->evaluate(
      current_delta, &cached_H_target, &cached_H_source, &cached_H_target_source, &cached_b_target, &cached_b_source);

    last_extraction_delta = current_delta;
    coreset_valid = true;
  }
  // else: reuse cached H, b, c (deferred sampling)

  if (is_binary) {
    return std::make_shared<gtsam::HessianFactor>(
      keys()[0],
      keys()[1],
      cached_H_target,
      cached_H_target_source,
      -cached_b_target,
      cached_H_source,
      -cached_b_source,
      cached_error);
  } else {
    return std::make_shared<gtsam::HessianFactor>(keys()[0], cached_H_source, -cached_b_source, cached_error);
  }
}

gtsam::NonlinearFactor::shared_ptr IntegratedCoresetFactor::clone() const {
  return std::make_shared<IntegratedCoresetFactor>(*this);
}

}  // namespace glil
