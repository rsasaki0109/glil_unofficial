// SPDX-License-Identifier: MIT
// Exact-coreset extraction via fast Caratheodory pruning.
//
// Given N per-point contribution vectors (columns of a D x N matrix) whose
// weighted sum (with all initial weights equal to 1) reproduces some
// aggregate quantity (e.g., the stacked upper-triangular Hessian blocks, b
// vector, and scalar error of a quadratic cost function), this extracts a
// weighted subset of at most D+1 points (or a caller-specified target size)
// whose weighted sum reproduces the same aggregate EXACTLY (up to floating
// point round-off).
//
// This is a reproduction of the fast-Caratheodory coreset construction used
// in:
//  - Koide et al., "Tightly Coupled Range Inertial Odometry and Mapping with
//    Exact Point Cloud Downsampling", ICRA 2025 (arXiv:2505.01017).
//  - Maalouf, Jubran, Feldman, "Fast and Accurate Least-Mean-Squares
//    Solvers", NeurIPS 2019 (fast Caratheodory clustering scheme).
//
// The core `caratheodoryPrune` routine and the contiguous-clustering outer
// loop are adapted from a clean-room Caratheodory implementation used
// elsewhere in this workspace (lidar_localization/exact_quadratic_coreset.hpp),
// generalized here to operate directly on caller-supplied contribution
// vectors instead of internally re-deriving them from a stacked Jacobian.

#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

namespace glim {

/// @brief Result of an exact-coreset extraction: a weighted subset of the
///        input columns (indices into the original N points) that
///        reproduces the full weighted sum.
struct ExactCoreset {
  std::vector<int> indices;    ///< Indices of the retained points (into the original N columns).
  std::vector<double> weights;  ///< Positive weight to apply to each retained point.
};

namespace exact_coreset_detail {

/// @brief One in-place Caratheodory pruning pass: given points (columns) and
///        positive weights whose weighted sum is s = sum_i w_i p_i, reduce
///        the number of positively weighted points to at most dim + 1 while
///        keeping s unchanged. Returns the indices (into the input columns,
///        i.e., local indices 0..points.cols()-1) that keep a positive
///        weight; `weights` is updated in place (pruned columns are left at
///        weight 0).
inline std::vector<int> caratheodoryPrune(const Eigen::MatrixXd& points, Eigen::VectorXd& weights) {
  const int dim = static_cast<int>(points.rows());
  std::vector<int> active(points.cols());
  std::iota(active.begin(), active.end(), 0);

  while (static_cast<int>(active.size()) > dim + 1) {
    const int m = static_cast<int>(active.size());
    // Null combination: sum_i v_i p_i = 0 with sum_i v_i = 0. Build the
    // difference matrix B = [p_1 - p_0, ..., p_{m-1} - p_0] and take any
    // kernel vector; v_0 absorbs the negated sum.
    Eigen::MatrixXd differences(dim, m - 1);
    for (int i = 1; i < m; ++i) {
      differences.col(i - 1) = points.col(active[i]) - points.col(active[0]);
    }
    const Eigen::FullPivLU<Eigen::MatrixXd> lu(differences);
    const Eigen::MatrixXd kernel = lu.kernel();
    if (lu.dimensionOfKernel() == 0) {
      break;  // Numerically full rank: cannot prune further.
    }
    Eigen::VectorXd v(m);
    v.tail(m - 1) = kernel.col(0);
    v(0) = -v.tail(m - 1).sum();

    double alpha = std::numeric_limits<double>::infinity();
    int drop = -1;
    for (int i = 0; i < m; ++i) {
      if (v(i) > 1e-12) {
        const double ratio = weights(active[i]) / v(i);
        if (ratio < alpha) {
          alpha = ratio;
          drop = i;
        }
      }
    }
    if (drop < 0) {
      // Kernel vector has no positive entry; flip it (its negation is also a
      // null combination) and retry.
      for (int i = 0; i < m; ++i) {
        v(i) = -v(i);
      }
      for (int i = 0; i < m; ++i) {
        if (v(i) > 1e-12) {
          const double ratio = weights(active[i]) / v(i);
          if (ratio < alpha) {
            alpha = ratio;
            drop = i;
          }
        }
      }
      if (drop < 0) {
        break;  // Degenerate; keep the current (already valid) subset.
      }
    }
    for (int i = 0; i < m; ++i) {
      weights(active[i]) -= alpha * v(i);
    }
    weights(active[drop]) = 0.0;  // Exact removal of the limiting point.
    std::vector<int> next;
    next.reserve(active.size() - 1);
    for (int i = 0; i < m; ++i) {
      if (i != drop && weights(active[i]) > 0.0) {
        next.push_back(active[i]);
      }
    }
    active.swap(next);
  }
  return active;
}

}  // namespace exact_coreset_detail

/// @brief Extract a weighted exact coreset from a set of per-point
///        contribution vectors.
/// @param points        D x N matrix; column i is the contribution vector of
///                       point i. All points are implicitly weighted 1.
/// @param target_size    Desired coreset size (upper bound on the number of
///                       retained points). If <= 0, or smaller than the
///                       theoretical minimum (D+1), the minimum exact size
///                       (D+1) is used instead.
/// @param cluster_count  Number of clusters used per fast-Caratheodory round.
///                       Must exceed D+2 for the clustering rounds to make
///                       progress; the default (64) is adequate for the
///                       dimensions used by the GICP coreset factor.
/// @return               Indices (into the N columns of `points`) and
///                       positive weights such that
///                         sum_k weights[k] * points.col(indices[k])
///                       equals points.rowwise().sum() up to floating point
///                       round-off, with indices.size() <= max(target_size, D+1).
inline ExactCoreset extractCoreset(const Eigen::MatrixXd& points, int target_size = 0, int cluster_count = 64) {
  const int dim = static_cast<int>(points.rows());
  const int point_count = static_cast<int>(points.cols());
  const int minimum_size = dim + 1;
  const int target = std::max(target_size, minimum_size);

  std::vector<int> indices(point_count);
  std::iota(indices.begin(), indices.end(), 0);
  std::vector<double> weights(point_count, 1.0);

  if (point_count <= target) {
    ExactCoreset result;
    result.indices = indices;
    result.weights = weights;
    return result;
  }

  const int effective_clusters = std::max(cluster_count, minimum_size + 1);

  while (static_cast<int>(indices.size()) > target) {
    const int m = static_cast<int>(indices.size());

    if (m <= effective_clusters) {
      // Direct pruning pass on the remaining points.
      Eigen::MatrixXd pts(dim, m);
      for (int i = 0; i < m; ++i) {
        pts.col(i) = points.col(indices[i]) * weights[indices[i]];
      }
      Eigen::VectorXd unit = Eigen::VectorXd::Ones(m);
      const std::vector<int> kept = exact_coreset_detail::caratheodoryPrune(pts, unit);

      std::vector<int> next_indices;
      next_indices.reserve(kept.size());
      for (const int i : kept) {
        weights[indices[i]] *= unit(i);
        next_indices.push_back(indices[i]);
      }
      indices.swap(next_indices);
      break;  // A direct pass always reaches the minimum size.
    }

    // Fast round: contiguous clusters, prune their weighted sums.
    const int chunk = (m + effective_clusters - 1) / effective_clusters;
    const int cluster_total = (m + chunk - 1) / chunk;
    Eigen::MatrixXd cluster_sums = Eigen::MatrixXd::Zero(dim, cluster_total);
    for (int i = 0; i < m; ++i) {
      cluster_sums.col(i / chunk) += weights[indices[i]] * points.col(indices[i]);
    }
    Eigen::VectorXd cluster_weights = Eigen::VectorXd::Ones(cluster_total);
    const std::vector<int> kept_clusters = exact_coreset_detail::caratheodoryPrune(cluster_sums, cluster_weights);

    std::vector<int> next_indices;
    next_indices.reserve(kept_clusters.size() * chunk);
    for (const int c : kept_clusters) {
      const int begin = c * chunk;
      const int end = std::min(begin + chunk, m);
      for (int i = begin; i < end; ++i) {
        weights[indices[i]] *= cluster_weights(c);
        next_indices.push_back(indices[i]);
      }
    }
    if (next_indices.size() >= indices.size()) {
      break;  // No progress (degenerate data); keep the current subset.
    }
    indices.swap(next_indices);
  }

  ExactCoreset result;
  result.indices.reserve(indices.size());
  result.weights.reserve(indices.size());
  for (const int index : indices) {
    if (weights[index] > 0.0) {
      result.indices.push_back(index);
      result.weights.push_back(weights[index]);
    }
  }
  return result;
}

}  // namespace glim
