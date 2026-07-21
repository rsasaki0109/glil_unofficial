// SPDX-License-Identifier: MIT
// Fork of gtsam_points::IntegratedGICPFactor_ (Segal et al., "Generalized-ICP",
// RSS2005) adding exact-coreset (fast Caratheodory) linearization, a
// reproduction of:
//   Koide et al., "Tightly Coupled Range Inertial Odometry and Mapping with
//   Exact Point Cloud Downsampling", ICRA 2025 (arXiv:2505.01017).
//
// gtsam_points::IntegratedGICPFactor_'s data members are private, so it
// cannot be usefully subclassed; this is a sibling class deriving directly
// from gtsam_points::IntegratedMatchingCostFactor, forking the
// update_correspondences()/evaluate() implementation from
// gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp (gtsam_points
// v1.2.2).
#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>

#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/nearest_neighbor_search.hpp>
#include <gtsam_points/factors/impl/scan_matching_reduction.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_points/types/frame_traits.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/util/parallelism.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

#include <glim/odometry/exact_quadratic_coreset.hpp>

namespace glim {

namespace frame = gtsam_points::frame;

/**
 * @brief GICP matching cost factor with exact-coreset linearization.
 *
 * This behaves exactly like gtsam_points::IntegratedGICPFactor_ except that,
 * once a linearization has been performed at a given point, subsequent
 * linearize() calls whose delta stays within `set_coreset_reuse_tolerance()`
 * of that point reuse a small weighted "exact coreset" of source points
 * instead of the full point set: the coreset's weighted Hessian/b/error
 * contributions exactly reproduce (up to floating point round-off) those of
 * the full set at the anchor linearization point (Koide et al., ICRA2025,
 * arXiv:2505.01017). This turns the O(N) correspondence search + Hessian
 * accumulation into an O(coreset_size) pass for as long as the coreset stays
 * valid.
 *
 * IMPORTANT: gtsam::NonlinearFactor::error() (used by LM for the trust-region
 * accept/reject decision) always goes through the FULL cached correspondence
 * set -- only the Hessian/b accumulation path inside linearize() may use the
 * coreset. Feeding LM's step-accept logic a coreset-approximated cost would
 * break its trust-region control, so error() (i.e., evaluate() called with
 * null Hessian pointers) never touches the coreset.
 */
template <typename TargetFrame = gtsam_points::PointCloud, typename SourceFrame = gtsam_points::PointCloud>
class IntegratedGICPFactorCoreset_ : public gtsam_points::IntegratedMatchingCostFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = gtsam_points::shared_ptr<IntegratedGICPFactorCoreset_>;

  /// @brief Create a binary ICP factor between target and source poses.
  IntegratedGICPFactorCoreset_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree)
  : gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),
    num_threads(1),
    max_correspondence_distance_sq(1.0),
    coreset_size(32),
    coreset_reuse_tolerance_rot(0.0),
    coreset_reuse_tolerance_trans(0.0),
    contribution_dim(0),
    full_cache_valid(false),
    full_cache_anchor(Eigen::Isometry3d::Identity()),
    coreset_valid(false),
    coreset_anchor(Eigen::Isometry3d::Identity()),
    target(target),
    source(source) {
    init(target_tree);
  }

  /// @brief Create a binary ICP factor between target and source poses.
  IntegratedGICPFactorCoreset_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source)
  : IntegratedGICPFactorCoreset_(target_key, source_key, target, source, nullptr) {}

  /// @brief Create a unary GICP factor between a fixed target pose and an active source pose.
  IntegratedGICPFactorCoreset_(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree)
  : gtsam_points::IntegratedMatchingCostFactor(fixed_target_pose, source_key),
    num_threads(1),
    max_correspondence_distance_sq(1.0),
    coreset_size(32),
    coreset_reuse_tolerance_rot(0.0),
    coreset_reuse_tolerance_trans(0.0),
    contribution_dim(0),
    full_cache_valid(false),
    full_cache_anchor(Eigen::Isometry3d::Identity()),
    coreset_valid(false),
    coreset_anchor(Eigen::Isometry3d::Identity()),
    target(target),
    source(source) {
    init(target_tree);
  }

  /// @brief Create a unary GICP factor between a fixed target pose and an active source pose.
  IntegratedGICPFactorCoreset_(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source)
  : IntegratedGICPFactorCoreset_(fixed_target_pose, source_key, target, source, nullptr) {}

  virtual ~IntegratedGICPFactorCoreset_() override {}

  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "IntegratedGICPFactorCoreset";
    if (is_binary) {
      std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
    } else {
      std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
    }
    std::cout << "|target|=" << frame::size(*target) << "pts, |source|=" << frame::size(*source) << "pts" << std::endl;
    std::cout << "num_threads=" << num_threads << ", max_corr_dist=" << std::sqrt(max_correspondence_distance_sq) << ", coreset_size=" << coreset_size
              << ", coreset_valid=" << coreset_valid << std::endl;
  }

  /**
   * @brief  Calculate the memory usage of this factor
   * @note   The result is approximate and does not account for objects not owned by this factor (e.g., point clouds)
   * @return Memory usage in bytes (Approximate size in bytes)
   */
  virtual size_t memory_usage() const override {
    return sizeof(*this) + sizeof(long) * correspondences.capacity() + sizeof(Eigen::Matrix4d) * mahalanobis.capacity() +
           sizeof(int) * coreset_indices.capacity() + sizeof(double) * coreset_weights.capacity();
  }

  /// @brief Set the number of threads used for the full-set linearization of this factor.
  void set_num_threads(int n) { num_threads = n; }

  /// @brief Set the maximum distance between corresponding points.
  ///        Correspondences with distances larger than this will be rejected (i.e., correspondence trimming).
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq = dist * dist; }

  /// @brief Target size of the exact coreset (upper bound on the number of points retained per
  ///        linearization). The theoretical minimum (contribution_dim + 1) is used if this is smaller.
  void set_coreset_size(int n) { coreset_size = n; }

  /// @brief The cached correspondences/coreset are reused (i.e., both the KNN search and the full
  ///        Hessian/b accumulation are skipped) as long as the SE3 displacement between the current
  ///        linearization point and the coreset's anchor point stays within these thresholds.
  /// @note  Default values are angle=trans=0, meaning the coreset is rebuilt on every linearize() call.
  void set_coreset_reuse_tolerance(double rot_rad, double trans_m) {
    coreset_reuse_tolerance_rot = rot_rad;
    coreset_reuse_tolerance_trans = trans_m;
  }

  /// @brief Compute the fraction of inlier points that have correspondences with a distance smaller than the trimming threshold.
  double inlier_fraction() const {
    const int outliers = std::count(correspondences.begin(), correspondences.end(), -1L);
    const int inliers = correspondences.size() - outliers;
    return static_cast<double>(inliers) / correspondences.size();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new IntegratedGICPFactorCoreset_(*this)); }

private:
  void init(const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree) {
    if (!frame::has_points(*target) || !frame::has_covs(*target)) {
      std::cerr << "error: target frame doesn't have required attributes for gicp" << std::endl;
      abort();
    }

    if (!frame::has_points(*source) || !frame::has_covs(*source)) {
      std::cerr << "error: source frame doesn't have required attributes for gicp" << std::endl;
      abort();
    }

    if (target_tree) {
      this->target_tree = target_tree;
    } else {
      this->target_tree.reset(new gtsam_points::KdTree2<TargetFrame>(target));
    }

    // GICP depends on the six-DoF relative pose. At a fixed relative pose,
    // every target-pose Jacobian is the source-pose Jacobian multiplied by
    // the same adjoint matrix. Therefore preserving H_source, b_source and
    // the scalar error also preserves every binary H/b block exactly; the
    // redundant 12-DoF packing would inflate the Caratheodory dimension
    // from 28 to 91 without adding information.
    contribution_dim = 21 + 6 + 1;
  }

  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override {
    linearization_point = delta;

    if (coreset_valid) {
      const Eigen::Isometry3d diff = delta.inverse() * coreset_anchor;
      const double diff_rot = Eigen::AngleAxisd(diff.linear()).angle();
      const double diff_trans = diff.translation().norm();
      if (diff_rot < coreset_reuse_tolerance_rot && diff_trans < coreset_reuse_tolerance_trans) {
        // The coreset built at coreset_anchor is still valid at this linearization point: skip
        // the full KNN + mahalanobis rebuild entirely (this is the whole point of the coreset).
        return;
      }
      // Displacement exceeded the reuse tolerance: invalidate and fall through to a full rebuild.
      coreset_valid = false;
    }

    // Deferred sampling from Koide et al. (ICRA 2025): the first
    // linearization only caches the full point contributions. If the next
    // linearization stays close, extract an exact coreset at that cached
    // anchor and use it at the new point. Large optimizer steps discard the
    // cache and trigger another full correspondence refresh instead of
    // spending time extracting a coreset that would be invalid immediately.
    if (full_cache_valid) {
      const Eigen::Isometry3d diff = delta.inverse() * full_cache_anchor;
      constexpr double extraction_tolerance_rot = 0.00436332313;  // 0.25 deg
      constexpr double extraction_tolerance_trans = 0.25;         // metres
      if (
        Eigen::AngleAxisd(diff.linear()).angle() < extraction_tolerance_rot &&
        diff.translation().norm() < extraction_tolerance_trans)
      {
        build_coreset(full_cache_anchor);
        return;
      }
      full_cache_valid = false;
    }

    // Full correspondence + mahalanobis rebuild (identical maths to stock IntegratedGICPFactor_,
    // restricted to the FULL FusedCovCacheMode::FULL cache mode).
    correspondences.resize(frame::size(*source));
    mahalanobis.resize(frame::size(*source));

    const auto perpoint_task = [&](int i) {
      const Eigen::Vector4d pt = delta * frame::point(*source, i);

      size_t k_index = -1;
      double k_sq_dist = -1;
      const size_t num_found = target_tree->knn_search(pt.data(), 1, &k_index, &k_sq_dist, max_correspondence_distance_sq);
      correspondences[i] = (num_found && k_sq_dist < max_correspondence_distance_sq) ? static_cast<long>(k_index) : -1;

      if (correspondences[i] < 0) {
        mahalanobis[i].setZero();
      } else {
        const auto& target_cov = frame::cov(*target, correspondences[i]);
        const Eigen::Matrix4d RCR = (target_cov + delta.matrix() * frame::cov(*source, i) * delta.matrix().transpose());
        mahalanobis[i].setZero();
        mahalanobis[i].template topLeftCorner<3, 3>() = RCR.topLeftCorner<3, 3>().inverse();
      }
    };

    if (gtsam_points::is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
      for (int i = 0; i < frame::size(*source); i++) {
        perpoint_task(i);
      }
    } else {
#ifdef GTSAM_POINTS_USE_TBB
      tbb::parallel_for(tbb::blocked_range<int>(0, frame::size(*source), 8), [&](const tbb::blocked_range<int>& range) {
        for (int i = range.begin(); i < range.end(); i++) {
          perpoint_task(i);
        }
      });
#else
      std::cerr << "error: TBB is not available" << std::endl;
      abort();
#endif
    }

    full_cache_anchor = delta;
    full_cache_valid = true;
  }

  /// @brief Evaluate the residual and (if any H pointer is non-null) the Jacobian-derived
  ///        Hessian/b contributions of a single source point `i` at linearization point `delta`,
  ///        using the cached correspondence/mahalanobis. Output arguments are OVERWRITTEN (not
  ///        accumulated). Mirrors the per-point maths of stock IntegratedGICPFactor_::evaluate().
  double perpoint_evaluate(
    int i,
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* h_target,
    Eigen::Matrix<double, 6, 6>* h_source,
    Eigen::Matrix<double, 6, 6>* h_target_source,
    Eigen::Matrix<double, 6, 1>* b_target,
    Eigen::Matrix<double, 6, 1>* b_source) const {
    const long target_index = correspondences[i];
    if (target_index < 0) {
      if (h_source) {
        h_source->setZero();
        b_source->setZero();
        if (is_binary) {
          h_target->setZero();
          h_target_source->setZero();
          b_target->setZero();
        }
      }
      return 0.0;
    }

    const auto& mean_A = frame::point(*source, i);
    const auto& mean_B = frame::point(*target, target_index);

    const Eigen::Vector4d transed_mean_A = delta * mean_A;
    const Eigen::Vector4d residual = mean_B - transed_mean_A;

    const Eigen::Matrix4d& maha = mahalanobis[i];
    const double error = residual.transpose() * maha * residual;

    if (h_source == nullptr) {
      return error;
    }

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();
    const Eigen::Matrix<double, 6, 4> J_source_mahalanobis = J_source.transpose() * maha;

    *h_source = J_source_mahalanobis * J_source;
    *b_source = J_source_mahalanobis * residual;

    if (is_binary) {
      Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
      J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
      J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
      const Eigen::Matrix<double, 6, 4> J_target_mahalanobis = J_target.transpose() * maha;

      *h_target = J_target_mahalanobis * J_target;
      *h_target_source = J_target_mahalanobis * J_source;
      *b_target = J_target_mahalanobis * residual;
    }

    return error;
  }

  /// @brief Pack one point's H/b/error contribution into the fixed-order coreset vector (see
  ///        contribution_dim's comment in init() for the layout and rationale).
  Eigen::VectorXd pack_contribution(
    const Eigen::Matrix<double, 6, 6>& h_target,
    const Eigen::Matrix<double, 6, 6>& h_source,
    const Eigen::Matrix<double, 6, 6>& h_target_source,
    const Eigen::Matrix<double, 6, 1>& b_target,
    const Eigen::Matrix<double, 6, 1>& b_source,
    double error) const {
    (void)h_target;
    (void)h_target_source;
    (void)b_target;
    Eigen::VectorXd v(contribution_dim);
    int k = 0;
    for (int r = 0; r < 6; ++r) {
      for (int c = r; c < 6; ++c) {
        v(k++) = h_source(r, c);
      }
    }
    for (int r = 0; r < 6; ++r) {
      v(k++) = b_source(r);
    }
    v(k++) = error;
    return v;
  }

  /// @brief Full O(N) evaluation of a source point, in the accumulate-into-thread-local-Hessian
  ///        style expected by scan_matching_reduce_omp()/scan_matching_reduce_tbb().
  double reduce_task(
    int i,
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target,
    Eigen::Matrix<double, 6, 6>* H_source,
    Eigen::Matrix<double, 6, 6>* H_target_source,
    Eigen::Matrix<double, 6, 1>* b_target,
    Eigen::Matrix<double, 6, 1>* b_source) const {
    if (H_source == nullptr) {
      return perpoint_evaluate(i, delta, nullptr, nullptr, nullptr, nullptr, nullptr);
    }

    Eigen::Matrix<double, 6, 6> h_t = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> h_s = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> h_ts = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b_t = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> b_s = Eigen::Matrix<double, 6, 1>::Zero();
    const double error = perpoint_evaluate(i, delta, &h_t, &h_s, &h_ts, &b_t, &b_s);

    *H_source += h_s;
    *b_source += b_s;
    if (is_binary) {
      *H_target += h_t;
      *H_target_source += h_ts;
      *b_target += b_t;
    }
    return error;
  }

  /// @brief Build a fresh exact coreset out of the current (freshly rebuilt) full correspondence
  ///        set at linearization point `delta`, storing it as the new anchor.
  void build_coreset(const Eigen::Isometry3d& delta) const {
    const int n = frame::size(*source);
    Eigen::MatrixXd contributions(contribution_dim, n);

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (int i = 0; i < n; ++i) {
      Eigen::Matrix<double, 6, 6> h_t = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 6> h_s = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 6> h_ts = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b_t = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> b_s = Eigen::Matrix<double, 6, 1>::Zero();
      const double error = perpoint_evaluate(i, delta, &h_t, &h_s, &h_ts, &b_t, &b_s);
      contributions.col(i) = pack_contribution(h_t, h_s, h_ts, b_t, b_s, error);
    }

    const ExactCoreset coreset = extractCoreset(contributions, coreset_size, 64);

    coreset_indices = coreset.indices;
    coreset_weights = coreset.weights;
    coreset_anchor = delta;
    coreset_valid = true;
    full_cache_valid = false;
  }

  /// @brief O(coreset_size) evaluation: recompute each coreset point's residual/Jacobian at the
  ///        current `delta` (using the cached correspondence + mahalanobis from the coreset's
  ///        anchor build), weight by its coreset weight, and accumulate.
  double evaluate_coreset(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target,
    Eigen::Matrix<double, 6, 6>* H_source,
    Eigen::Matrix<double, 6, 6>* H_target_source,
    Eigen::Matrix<double, 6, 1>* b_target,
    Eigen::Matrix<double, 6, 1>* b_source) const {
    H_source->setZero();
    b_source->setZero();
    if (is_binary) {
      H_target->setZero();
      H_target_source->setZero();
      b_target->setZero();
    }
    double weighted_error = 0.0;

    // Deliberately a plain serial loop: the coreset only has O(tens) of points, and this is the
    // whole point of using it (avoid the O(N) parallel reduction).
    for (size_t k = 0; k < coreset_indices.size(); ++k) {
      const int i = coreset_indices[k];
      const double w = coreset_weights[k];

      Eigen::Matrix<double, 6, 6> h_t = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 6> h_s = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 6> h_ts = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b_t = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> b_s = Eigen::Matrix<double, 6, 1>::Zero();
      const double error = perpoint_evaluate(i, delta, &h_t, &h_s, &h_ts, &b_t, &b_s);

      *H_source += w * h_s;
      *b_source += w * b_s;
      if (is_binary) {
        *H_target += w * h_t;
        *H_target_source += w * h_ts;
        *b_target += w * b_t;
      }
      weighted_error += w * error;
    }

    return weighted_error;
  }

  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override {
    if (correspondences.size() != static_cast<size_t>(frame::size(*source))) {
      update_correspondences(delta);
    }

    const int n = frame::size(*source);

    if (H_source == nullptr) {
      // error()/cost path: ALWAYS full-set evaluation over the cached correspondences. The
      // coreset must never be used here -- LM's step accept/reject logic compares this value
      // before and after a trial step, and a coreset-approximated cost (which is only exact at
      // its own anchor point) would break that trust-region control.
      const auto task = [&](
                           int i,
                           Eigen::Matrix<double, 6, 6>*,
                           Eigen::Matrix<double, 6, 6>*,
                           Eigen::Matrix<double, 6, 6>*,
                           Eigen::Matrix<double, 6, 1>*,
                           Eigen::Matrix<double, 6, 1>*) { return reduce_task(i, delta, nullptr, nullptr, nullptr, nullptr, nullptr); };

      if (gtsam_points::is_omp_default() || num_threads == 1) {
        return gtsam_points::scan_matching_reduce_omp(task, n, num_threads, nullptr, nullptr, nullptr, nullptr, nullptr);
      } else {
        return gtsam_points::scan_matching_reduce_tbb(task, n, nullptr, nullptr, nullptr, nullptr, nullptr);
      }
    }

    // Hessian/b requested (i.e., called from linearize()).
    if (coreset_valid) {
      return evaluate_coreset(delta, H_target, H_source, H_target_source, b_target, b_source);
    }

    // No valid coreset yet (or a large optimizer step invalidated the
    // previous one): do a stock-equivalent full evaluation. Coreset
    // extraction is deliberately deferred until a later, nearby
    // linearization in update_correspondences().
    const auto task = [&](
                         int i,
                         Eigen::Matrix<double, 6, 6>* ht,
                         Eigen::Matrix<double, 6, 6>* hs,
                         Eigen::Matrix<double, 6, 6>* hts,
                         Eigen::Matrix<double, 6, 1>* bt,
                         Eigen::Matrix<double, 6, 1>* bs) { return reduce_task(i, delta, ht, hs, hts, bt, bs); };

    double error;
    if (gtsam_points::is_omp_default() || num_threads == 1) {
      error = gtsam_points::scan_matching_reduce_omp(task, n, num_threads, H_target, H_source, H_target_source, b_target, b_source);
    } else {
      error = gtsam_points::scan_matching_reduce_tbb(task, n, H_target, H_source, H_target_source, b_target, b_source);
    }
    return error;
  }

private:
  int num_threads;
  double max_correspondence_distance_sq;
  int coreset_size;
  double coreset_reuse_tolerance_rot;
  double coreset_reuse_tolerance_trans;
  int contribution_dim;

  std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;

  // I'm unhappy to have mutable members... (same caveat as stock IntegratedGICPFactor_)
  mutable Eigen::Isometry3d linearization_point;
  mutable std::vector<long> correspondences;
  mutable std::vector<Eigen::Matrix4d> mahalanobis;
  mutable bool full_cache_valid;
  mutable Eigen::Isometry3d full_cache_anchor;

  mutable bool coreset_valid;
  mutable Eigen::Isometry3d coreset_anchor;
  mutable std::vector<int> coreset_indices;
  mutable std::vector<double> coreset_weights;

  std::shared_ptr<const TargetFrame> target;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedGICPFactorCoreset = IntegratedGICPFactorCoreset_<>;

}  // namespace glim
