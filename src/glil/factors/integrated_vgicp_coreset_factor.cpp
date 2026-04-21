// SPDX-License-Identifier: MIT
// IntegratedVGICPCoresetFactor implementation
// Reference: Koide et al., ICRA 2025

#include <glil/factors/integrated_vgicp_coreset_factor.hpp>

#include <algorithm>
#include <numeric>
#include <random>
#include <unordered_set>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <caratheodory.hpp>

#ifdef GLIL_PROFILE_TIMING
#include <glil/util/digest.hpp>
#include <glil/util/logging.hpp>
#include <glil/util/perftime.hpp>
#endif

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

bool same_pose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return a.matrix().isApprox(b.matrix(), 0.0);
}

#ifdef GLIL_PROFILE_TIMING
void log_coreset_digest(
  const IntegratedVGICPCoresetFactor::Params& params,
  const int num_valid,
  const uint64_t j_digest,
  const uint64_t e_digest,
  const Eigen::VectorXi& indices,
  const Eigen::VectorXi& residual_rows) {
  if (!params.debug_digest) {
    return;
  }

  std::vector<int> idx_payload;
  idx_payload.reserve(static_cast<std::size_t>(indices.size()) * 2);
  for (int i = 0; i < indices.size(); i++) {
    idx_payload.push_back(indices[i]);
    idx_payload.push_back(residual_rows[i]);
  }

  const void* idx_data = idx_payload.empty() ? nullptr : idx_payload.data();
  const uint64_t idx_digest = fnv1a_u64(idx_data, idx_payload.size() * sizeof(int));
  create_module_logger("odom")->info(
    "[digest] frame={} factor_idx={} valid={} j_digest={:016x} e_digest={:016x} idx_digest={:016x}",
    params.debug_frame_id,
    params.debug_factor_idx,
    num_valid,
    j_digest,
    e_digest,
    idx_digest);
}
#endif
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

IntegratedVGICPCoresetFactor::IntegratedVGICPCoresetFactor(const IntegratedVGICPCoresetFactor& other)
: IntegratedMatchingCostFactor(other),
  coreset_params(other.coreset_params),
  target_voxels(other.target_voxels),
  source(other.source) {
  linearization_point.setIdentity();
  last_coreset_delta.setIdentity();
  coreset_valid = false;
  debug_count = 0;
}

IntegratedVGICPCoresetFactor::~IntegratedVGICPCoresetFactor() {}

gtsam::NonlinearFactor::shared_ptr IntegratedVGICPCoresetFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new IntegratedVGICPCoresetFactor(*this));
}

double IntegratedVGICPCoresetFactor::error(const gtsam::Values& values) const {
  if (!coreset_params.coreset_immutable_snapshot) {
    return IntegratedMatchingCostFactor::error(values);
  }

  const auto snapshot = update_snapshot(calc_delta(values));
  return snapshot ? evaluate_snapshot(*snapshot) : 0.0;
}

gtsam::GaussianFactor::shared_ptr IntegratedVGICPCoresetFactor::linearize(const gtsam::Values& values) const {
  if (!coreset_params.coreset_immutable_snapshot) {
    return IntegratedMatchingCostFactor::linearize(values);
  }

  const auto snapshot = update_snapshot(calc_delta(values));

  Eigen::Matrix<double, 6, 6> H_target = Eigen::Matrix<double, 6, 6>::Zero(), H_source = Eigen::Matrix<double, 6, 6>::Zero(), H_target_source = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> b_target = Eigen::Matrix<double, 6, 1>::Zero(), b_source = Eigen::Matrix<double, 6, 1>::Zero();
  const double error = snapshot ? evaluate_snapshot(*snapshot, &H_target, &H_source, &H_target_source, &b_target, &b_source) : 0.0;

  if (is_binary) {
    return gtsam::GaussianFactor::shared_ptr(new gtsam::HessianFactor(keys()[0], keys()[1], H_target, H_target_source, -b_target, H_source, -b_source, error));
  }
  return gtsam::GaussianFactor::shared_ptr(new gtsam::HessianFactor(keys()[0], H_source, -b_source, error));
}

void IntegratedVGICPCoresetFactor::update_correspondences(const Eigen::Isometry3d& delta) const {
  if (coreset_params.coreset_immutable_snapshot) {
    update_snapshot(delta);
    return;
  }

  if (coreset_params.coreset_factor_lock) {
    std::lock_guard<std::mutex> lock(coreset_mutex_);
    update_correspondences_unlocked(delta);
    return;
  }

  update_correspondences_unlocked(delta);
}

void IntegratedVGICPCoresetFactor::update_correspondences_unlocked(const Eigen::Isometry3d& delta) const {
#ifdef GLIL_PROFILE_TIMING
  perftime::ScopedTimer profile_update_corresp(perftime::Counter::UpdateCorresp);
#endif
  linearization_point = delta;
  coreset_valid = false;
  const int N = point_size(*source);
  correspondences.resize(N);
  mahalanobis.resize(N);

  const double correspondence_sample_ratio =
    std::min(1.0, std::max(0.0, coreset_params.correspondence_sample_ratio));
  if (correspondence_sample_ratio < 1.0) {
    std::fill(correspondences.begin(), correspondences.end(), nullptr);

    if (N == 0 || correspondence_sample_ratio <= 0.0) {
      return;
    }

    const int sample_N = std::min(
      N,
      std::max(1, static_cast<int>(static_cast<double>(N) * correspondence_sample_ratio)));

    std::vector<int> sampled_indices;
    sampled_indices.reserve(sample_N);
    std::unordered_set<int> selected_indices;
    selected_indices.reserve(static_cast<std::size_t>(sample_N) * 2);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> point_dist(0, N - 1);
    while (static_cast<int>(sampled_indices.size()) < sample_N) {
      const int i = point_dist(rng);
      if (selected_indices.insert(i).second) {
        sampled_indices.push_back(i);
      }
    }

#pragma omp parallel for num_threads(coreset_params.num_threads) schedule(guided, 8)
    for (int idx = 0; idx < static_cast<int>(sampled_indices.size()); idx++) {
      const int i = sampled_indices[idx];
      Eigen::Vector4d pt = delta * point_at(*source, i);
      Eigen::Vector3i coord = target_voxels->voxel_coord(pt);
      const auto voxel_id = target_voxels->lookup_voxel_index(coord);

      if (voxel_id < 0) {
        mahalanobis[i].setZero();
      } else {
        const auto voxel = &target_voxels->lookup_voxel(voxel_id);
        correspondences[i] = voxel;

        const Eigen::Matrix4d RCR = voxel->cov + delta.matrix() * cov_at(*source, i) * delta.matrix().transpose();
        mahalanobis[i].setZero();
        mahalanobis[i].template topLeftCorner<3, 3>() = RCR.topLeftCorner<3, 3>().inverse();
      }
    }
    return;
  }

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

void IntegratedVGICPCoresetFactor::extract_coreset(const Eigen::Isometry3d& delta) const {
  if (coreset_params.coreset_factor_lock) {
    std::lock_guard<std::mutex> lock(coreset_mutex_);
    extract_coreset_unlocked(delta);
    return;
  }

  extract_coreset_unlocked(delta);
}

void IntegratedVGICPCoresetFactor::extract_coreset_unlocked(const Eigen::Isometry3d& delta) const {
#ifdef GLIL_PROFILE_TIMING
  perftime::ScopedTimer profile_extract_coreset(perftime::Counter::ExtractCoreset);
#endif
  const int N = point_size(*source);

  // Collect per-point Jacobians and residuals for valid correspondences
  // For VGICP, each point contributes a weighted residual
  // We compute J_source (6x1 per dimension, 3 residual dims) and pack into Nx6 Jacobian

  // First pass: count valid correspondences
  std::vector<int> valid_indices;
  {
#ifdef GLIL_PROFILE_TIMING
    perftime::ScopedTimer profile_coreset_valid_filter(perftime::Counter::Coreset_ValidFilter);
#endif
    valid_indices.reserve(N);
    for (int i = 0; i < N; i++) {
      if (correspondences[i] != nullptr) {
        valid_indices.push_back(i);
      }
    }
  }

  const int num_valid = valid_indices.size();
  if (num_valid < coreset_params.coreset_target_size) {
    // Too few points for coreset extraction, use all (3 residual rows per point)
    {
#ifdef GLIL_PROFILE_TIMING
      perftime::ScopedTimer profile_coreset_selected_rows(perftime::Counter::Coreset_SelectedRows);
#endif
      coreset_indices.resize(3 * num_valid);
      coreset_residual_rows.resize(3 * num_valid);
      coreset_weights.resize(3 * num_valid);
      for (int i = 0; i < num_valid; i++) {
        for (int r = 0; r < 3; r++) {
          coreset_indices[i * 3 + r] = valid_indices[i];
          coreset_residual_rows[i * 3 + r] = r;
          coreset_weights[i * 3 + r] = 1.0;
        }
      }
      coreset_valid = true;
      last_coreset_delta = delta;
#ifdef GLIL_PROFILE_TIMING
      log_coreset_digest(coreset_params, num_valid, 0, 0, coreset_indices, coreset_residual_rows);
#endif
    }
    return;
  }

  const int target_N = std::max(29, coreset_params.coreset_target_size);

  const auto build_weighted_rows = [&](const std::vector<int>& source_indices, Eigen::Matrix<double, -1, 6>& J, Eigen::VectorXd& e) {
    const int num_residuals = 3 * static_cast<int>(source_indices.size());
    J.resize(num_residuals, 6);
    e.resize(num_residuals);

    for (int idx = 0; idx < static_cast<int>(source_indices.size()); idx++) {
      const int i = source_indices[idx];
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
  };

  if (coreset_params.coreset_method == "uniform_sample_early") {
    const int sample_N = std::min(target_N, num_valid);
    std::vector<int> selected_indices;
    selected_indices.reserve(sample_N);
    std::unordered_set<int> selected_local_indices;
    selected_local_indices.reserve(static_cast<std::size_t>(sample_N) * 2);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> point_dist(0, num_valid - 1);
    while (static_cast<int>(selected_indices.size()) < sample_N) {
      const int local_idx = point_dist(rng);
      if (selected_local_indices.insert(local_idx).second) {
        selected_indices.push_back(valid_indices[local_idx]);
      }
    }

    Eigen::Matrix<double, -1, 6> J;
    Eigen::VectorXd e;
    {
#ifdef GLIL_PROFILE_TIMING
      perftime::ScopedTimer profile_coreset_buildje(perftime::Counter::Coreset_BuildJE);
#endif
      build_weighted_rows(selected_indices, J, e);
    }

#ifdef GLIL_PROFILE_TIMING
    const uint64_t j_digest =
      coreset_params.debug_digest ? fnv1a_doubles(J.data(), static_cast<std::size_t>(J.size())) : 0;
    const uint64_t e_digest =
      coreset_params.debug_digest ? fnv1a_doubles(e.data(), static_cast<std::size_t>(e.size())) : 0;
#endif

    {
#ifdef GLIL_PROFILE_TIMING
      perftime::ScopedTimer profile_coreset_selected_rows(perftime::Counter::Coreset_SelectedRows);
#endif
      const double weight = static_cast<double>(num_valid) / static_cast<double>(sample_N);
      coreset_indices.resize(3 * sample_N);
      coreset_residual_rows.resize(3 * sample_N);
      coreset_weights.resize(3 * sample_N);
      for (int i = 0; i < sample_N; i++) {
        for (int r = 0; r < 3; r++) {
          coreset_indices[i * 3 + r] = selected_indices[i];
          coreset_residual_rows[i * 3 + r] = r;
          coreset_weights[i * 3 + r] = weight;
        }
      }

      coreset_valid = true;
      last_coreset_delta = delta;
    }
#ifdef GLIL_PROFILE_TIMING
    log_coreset_digest(coreset_params, num_valid, j_digest, e_digest, coreset_indices, coreset_residual_rows);
#endif
    return;
  }

  // For each valid point, compute the 3 residual components and their Jacobians w.r.t. source pose (6 DOF)
  // Total: 3*num_valid rows, 6 columns
  Eigen::Matrix<double, -1, 6> J;
  Eigen::VectorXd e;
  {
#ifdef GLIL_PROFILE_TIMING
    perftime::ScopedTimer profile_coreset_buildje(perftime::Counter::Coreset_BuildJE);
#endif
    build_weighted_rows(valid_indices, J, e);
  }

#ifdef GLIL_PROFILE_TIMING
  const uint64_t j_digest =
    coreset_params.debug_digest ? fnv1a_doubles(J.data(), static_cast<std::size_t>(J.size())) : 0;
  const uint64_t e_digest =
    coreset_params.debug_digest ? fnv1a_doubles(e.data(), static_cast<std::size_t>(e.size())) : 0;
#endif

  // Select coreset rows
  Eigen::VectorXi raw_indices;
  Eigen::VectorXd raw_weights;
  if (coreset_params.coreset_method == "uniform_sample") {
    const int num_residual_rows = static_cast<int>(J.rows());
    raw_indices.resize(target_N);
    raw_weights.setConstant(target_N, static_cast<double>(num_residual_rows) / static_cast<double>(target_N));

    std::vector<int> shuffled_indices(num_residual_rows);
    std::iota(shuffled_indices.begin(), shuffled_indices.end(), 0);
    std::mt19937 rng(42);
    std::shuffle(shuffled_indices.begin(), shuffled_indices.end(), rng);

    for (int i = 0; i < target_N; i++) {
      raw_indices[i] = shuffled_indices[i];
    }
  } else if (coreset_params.coreset_method == "residual_weighted") {
    const int num_residual_rows = static_cast<int>(J.rows());
    const double eps = 1e-12;
    std::vector<double> row_masses(num_residual_rows);
    double sum_mass = 0.0;

    for (int i = 0; i < num_valid; i++) {
      const double mass = e.segment<3>(3 * i).norm() + eps;
      for (int r = 0; r < 3; r++) {
        row_masses[3 * i + r] = mass;
        sum_mass += mass;
      }
    }

    raw_indices.resize(target_N);
    raw_weights.resize(target_N);

    std::mt19937 rng(42);
    std::discrete_distribution<int> row_dist(row_masses.begin(), row_masses.end());

    for (int i = 0; i < target_N; i++) {
      const int residual_idx = row_dist(rng);
      raw_indices[i] = residual_idx;
      raw_weights[i] = sum_mass / (static_cast<double>(target_N) * row_masses[residual_idx]);
    }

    const double weight_sum = raw_weights.sum();
    if (weight_sum > 0.0) {
      raw_weights *= static_cast<double>(num_residual_rows) / weight_sum;
    }
  } else {
#ifdef GLIL_PROFILE_TIMING
    perftime::ScopedTimer profile_coreset_caratheodory(perftime::Counter::Coreset_Caratheodory);
    perftime::add(perftime::Counter::Coreset_CarathCalls, 1);
#endif
    exd::fast_caratheodory_quadratic(J, e, coreset_params.coreset_num_clusters, raw_indices, raw_weights, target_N);
  }

  // Map coreset residual indices back to point indices and row offsets
  // Each point generates 3 residuals (rows idx*3, idx*3+1, idx*3+2)
  // The coreset selects individual residual rows, so we must track
  // which row within the point (0, 1, or 2) was selected
  {
#ifdef GLIL_PROFILE_TIMING
    perftime::ScopedTimer profile_coreset_selected_rows(perftime::Counter::Coreset_SelectedRows);
#endif
    coreset_indices.resize(raw_indices.size());
    coreset_residual_rows.resize(raw_indices.size());
    coreset_weights.resize(raw_indices.size());
    for (int i = 0; i < raw_indices.size(); i++) {
      int residual_idx = raw_indices[i];
      int point_local_idx = residual_idx / 3;
      int row_within_point = residual_idx % 3;
      coreset_indices[i] = valid_indices[point_local_idx];
      coreset_residual_rows[i] = row_within_point;
      coreset_weights[i] = raw_weights[i];
    }

    coreset_valid = true;
    last_coreset_delta = delta;
  }
#ifdef GLIL_PROFILE_TIMING
  log_coreset_digest(coreset_params, num_valid, j_digest, e_digest, coreset_indices, coreset_residual_rows);
#endif
}

std::shared_ptr<const IntegratedVGICPCoresetFactor::CoresetSnapshot> IntegratedVGICPCoresetFactor::load_snapshot() const {
  std::lock_guard<std::mutex> lock(coreset_mutex_);
  return snapshot_;
}

std::shared_ptr<const IntegratedVGICPCoresetFactor::CoresetSnapshot> IntegratedVGICPCoresetFactor::update_snapshot(const Eigen::Isometry3d& delta) const {
  std::lock_guard<std::mutex> lock(coreset_mutex_);
  if (snapshot_ && same_pose(snapshot_->linearization_point, delta)) {
    return snapshot_;
  }

  update_correspondences_unlocked(delta);
  extract_coreset_unlocked(delta);

  auto snapshot = std::make_shared<CoresetSnapshot>();
  snapshot->linearization_point = delta;
  snapshot->coreset_indices = coreset_indices;
  snapshot->coreset_residual_rows = coreset_residual_rows;
  snapshot->coreset_weights = coreset_weights;
  populate_snapshot_rows(*snapshot);
  snapshot_ = snapshot;
  return snapshot_;
}

void IntegratedVGICPCoresetFactor::populate_snapshot_rows(CoresetSnapshot& snapshot) const {
  const int rows = snapshot.coreset_indices.size();
  snapshot.J_target_rows.resize(rows, 6);
  snapshot.J_source_rows.resize(rows, 6);
  snapshot.e_rows.resize(rows);

  const Eigen::Isometry3d& delta = snapshot.linearization_point;
  for (int ci = 0; ci < rows; ci++) {
    const int i = snapshot.coreset_indices[ci];
    const int row = snapshot.coreset_residual_rows[ci];
    snapshot.J_target_rows.row(ci).setZero(); snapshot.J_source_rows.row(ci).setZero(); snapshot.e_rows[ci] = 0.0;

    if (i < 0 || i >= static_cast<int>(correspondences.size()) || correspondences[i] == nullptr) {
      continue;
    }

    const auto& mean_A = point_at(*source, i);
    const auto& mean_B = correspondences[i]->mean;
    const Eigen::Vector4d transed_mean_A = delta * mean_A;
    const Eigen::Vector4d residual = mean_B - transed_mean_A;

    const Eigen::Matrix3d M3 = mahalanobis[i].topLeftCorner<3, 3>();
    Eigen::LLT<Eigen::Matrix3d> llt(M3);
    Eigen::Matrix3d L;
    if (llt.info() == Eigen::Success) {
      L = llt.matrixL();
    } else {
      L = M3.diagonal().cwiseSqrt().asDiagonal();
    }

    const Eigen::Vector3d w_res = L.transpose() * residual.head<3>();
    snapshot.e_rows[ci] = w_res[row];

    Eigen::Matrix<double, 3, 6> J_target_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
    J_target_3x6.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.template head<3>());
    J_target_3x6.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 6> J_source_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
    J_source_3x6.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source_3x6.block<3, 3>(0, 3) = -delta.linear();

    const Eigen::Matrix<double, 3, 6> wJ_target = L.transpose() * J_target_3x6;
    const Eigen::Matrix<double, 3, 6> wJ_source = L.transpose() * J_source_3x6;
    snapshot.J_target_rows.row(ci) = wJ_target.row(row);
    snapshot.J_source_rows.row(ci) = wJ_source.row(row);
  }
}

double IntegratedVGICPCoresetFactor::evaluate_snapshot(
  const CoresetSnapshot& snapshot,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  if (H_target) {
    H_target->setZero();
    H_source->setZero();
    H_target_source->setZero();
    b_target->setZero();
    b_source->setZero();
  }

  double sum_errors = 0.0;
  for (int ci = 0; ci < snapshot.coreset_weights.size(); ci++) {
    const double w = snapshot.coreset_weights[ci];
    const double e_row = snapshot.e_rows[ci];
    sum_errors += w * e_row * e_row;

    if (!H_target) {
      continue;
    }

    const Eigen::Matrix<double, 1, 6> j_target_row = snapshot.J_target_rows.row(ci);
    const Eigen::Matrix<double, 1, 6> j_source_row = snapshot.J_source_rows.row(ci);
    *H_target += w * j_target_row.transpose() * j_target_row;
    *H_source += w * j_source_row.transpose() * j_source_row;
    *H_target_source += w * j_target_row.transpose() * j_source_row;
    *b_target += w * j_target_row.transpose() * e_row;
    *b_source += w * j_source_row.transpose() * e_row;
  }

  if (H_target) {
    const double reg = 1e-6;
    *H_target += reg * Eigen::Matrix<double, 6, 6>::Identity();
    *H_source += reg * Eigen::Matrix<double, 6, 6>::Identity();
  }

  return sum_errors;
}

double IntegratedVGICPCoresetFactor::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  if (coreset_params.coreset_immutable_snapshot) {
    const auto snapshot = load_snapshot();
    if (!snapshot) {
      if (H_target) {
        H_target->setZero();
        H_source->setZero();
        H_target_source->setZero();
        b_target->setZero();
        b_source->setZero();
      }
      return 0.0;
    }
    return evaluate_snapshot(*snapshot, H_target, H_source, H_target_source, b_target, b_source);
  }

  //
  if (correspondences.size() != static_cast<size_t>(point_size(*source))) {
    update_correspondences(delta);
  }

  // Correspondence updates invalidate the cached coreset, so reuse is only
  // allowed while the same linearization/correspondence state is active.
  if (!coreset_valid) {
    extract_coreset(delta);
  }

  if (!coreset_valid || coreset_indices.size() == 0) {
    if (H_target) {
      H_target->setZero();
      H_source->setZero();
      H_target_source->setZero();
      b_target->setZero();
      b_source->setZero();
    }
    return 0.0;
  }

  // Zero-initialize output matrices (caller may pass uninitialized memory)
  if (H_target) {
    H_target->setZero();
    H_source->setZero();
    H_target_source->setZero();
    b_target->setZero();
    b_source->setZero();
  }

  // Debug verification: compare coreset evaluation against full evaluation
  // Only for the first few calls to avoid performance impact
  if (false && debug_count < 3 && H_target) {
    Eigen::Matrix<double, 6, 6> H_full_target = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> H_full_source = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> H_full_ts = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b_full_target = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> b_full_source = Eigen::Matrix<double, 6, 1>::Zero();
    double c_full = 0.0;

    const int N = point_size(*source);
    for (int i = 0; i < N; i++) {
      if (correspondences[i] == nullptr) continue;
      const auto& mean_A = point_at(*source, i);
      const auto& mean_B = correspondences[i]->mean;
      Eigen::Vector4d transed_mean_A = delta * mean_A;
      Eigen::Vector4d residual = mean_B - transed_mean_A;
      const Eigen::Matrix4d& M = mahalanobis[i];
      c_full += residual.transpose() * M * residual;

      Eigen::Matrix<double, 4, 6> J_t = Eigen::Matrix<double, 4, 6>::Zero();
      J_t.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
      J_t.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
      Eigen::Matrix<double, 4, 6> J_s = Eigen::Matrix<double, 4, 6>::Zero();
      J_s.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.head<3>());
      J_s.block<3, 3>(0, 3) = -delta.linear();

      Eigen::Matrix<double, 6, 4> J_t_M = J_t.transpose() * M;
      Eigen::Matrix<double, 6, 4> J_s_M = J_s.transpose() * M;
      H_full_target += J_t_M * J_t;
      H_full_source += J_s_M * J_s;
      H_full_ts += J_t_M * J_s;
      b_full_target += J_t_M * residual;
      b_full_source += J_s_M * residual;
    }

    // Now compute coreset evaluation for comparison (same as below)
    Eigen::Matrix<double, 6, 6> H_cs_target = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> H_cs_source = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> H_cs_ts = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b_cs_target = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> b_cs_source = Eigen::Matrix<double, 6, 1>::Zero();
    double c_cs = 0.0;

    for (int ci = 0; ci < coreset_indices.size(); ci++) {
      const int i = coreset_indices[ci];
      const int row = coreset_residual_rows[ci];
      const double w = coreset_weights[ci];
      if (correspondences[i] == nullptr) continue;

      const auto& mean_A = point_at(*source, i);
      const auto& mean_B = correspondences[i]->mean;
      Eigen::Vector4d transed_mean_A = delta * mean_A;
      Eigen::Vector4d residual = mean_B - transed_mean_A;

      // Cholesky decomposition of Mahalanobis
      Eigen::Matrix3d M3 = mahalanobis[i].topLeftCorner<3, 3>();
      Eigen::LLT<Eigen::Matrix3d> llt(M3);
      Eigen::Matrix3d L;
      if (llt.info() == Eigen::Success) {
        L = llt.matrixL();
      } else {
        L = M3.diagonal().cwiseSqrt().asDiagonal();
      }

      // L^T * residual gives the whitened residual (3-vector)
      Eigen::Vector3d w_res = L.transpose() * residual.head<3>();
      double e_row = w_res[row];
      c_cs += w * e_row * e_row;

      // Jacobians
      Eigen::Matrix<double, 3, 6> J_t_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
      J_t_3x6.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
      J_t_3x6.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
      Eigen::Matrix<double, 3, 6> J_s_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
      J_s_3x6.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.head<3>());
      J_s_3x6.block<3, 3>(0, 3) = -delta.linear();

      Eigen::Matrix<double, 3, 6> wJ_t = L.transpose() * J_t_3x6;
      Eigen::Matrix<double, 3, 6> wJ_s = L.transpose() * J_s_3x6;
      Eigen::Matrix<double, 1, 6> j_t_row = wJ_t.row(row);
      Eigen::Matrix<double, 1, 6> j_s_row = wJ_s.row(row);

      H_cs_target += w * j_t_row.transpose() * j_t_row;
      H_cs_source += w * j_s_row.transpose() * j_s_row;
      H_cs_ts += w * j_t_row.transpose() * j_s_row;
      b_cs_target += w * j_t_row.transpose() * e_row;
      b_cs_source += w * j_s_row.transpose() * e_row;
    }

    double H_diff = (H_full_source - H_cs_source).norm();
    double b_diff = (b_full_source - b_cs_source).norm();
    double c_diff = std::abs(c_full - c_cs);
    double H_rel = H_full_source.norm() > 0 ? H_diff / H_full_source.norm() : H_diff;
    double b_rel = b_full_source.norm() > 0 ? b_diff / b_full_source.norm() : b_diff;
    double c_rel = c_full > 0 ? c_diff / c_full : c_diff;

    // Write debug output to both stderr and a file
    fprintf(stderr, "[CORESET_DEBUG] eval #%d: coreset_size=%ld, full_points=%d\n", debug_count, coreset_indices.size(), N);
    fprintf(stderr, "[CORESET_DEBUG]   H_source: full_norm=%.6e, diff=%.6e, rel=%.6e\n", H_full_source.norm(), H_diff, H_rel);
    fprintf(stderr, "[CORESET_DEBUG]   b_source: full_norm=%.6e, diff=%.6e, rel=%.6e\n", b_full_source.norm(), b_diff, b_rel);
    fprintf(stderr, "[CORESET_DEBUG]   c (error): full=%.6e, coreset=%.6e, diff=%.6e, rel=%.6e\n", c_full, c_cs, c_diff, c_rel);
    fflush(stderr);

    FILE* dbg_file = fopen("/tmp/coreset_debug.log", "a");
    if (dbg_file) {
      fprintf(dbg_file, "[CORESET_DEBUG] eval #%d: coreset_size=%ld, full_points=%d\n", debug_count, coreset_indices.size(), N);
      fprintf(dbg_file, "[CORESET_DEBUG]   H_source: full_norm=%.6e, diff=%.6e, rel=%.6e\n", H_full_source.norm(), H_diff, H_rel);
      fprintf(dbg_file, "[CORESET_DEBUG]   b_source: full_norm=%.6e, diff=%.6e, rel=%.6e\n", b_full_source.norm(), b_diff, b_rel);
      fprintf(dbg_file, "[CORESET_DEBUG]   c (error): full=%.6e, coreset=%.6e, diff=%.6e, rel=%.6e\n", c_full, c_cs, c_diff, c_rel);
      fclose(dbg_file);
    }

    debug_count++;
  }

  // Evaluate using coreset points with per-row decomposition
  // The coreset was built on whitened residual rows (L^T * r)[row],
  // so we must evaluate in the same decomposed form
  double sum_errors = 0.0;

#ifdef GLIL_PROFILE_TIMING
  const auto profile_evaluate_t0 = perftime::now();
#endif
  for (int ci = 0; ci < coreset_indices.size(); ci++) {
    const int i = coreset_indices[ci];
    const int row = coreset_residual_rows[ci];
    const double w = coreset_weights[ci];

    const auto& target_voxel = correspondences[i];
    if (target_voxel == nullptr) {
      continue;
    }

    const auto& mean_A = point_at(*source, i);
    const auto& mean_B = target_voxel->mean;

    Eigen::Vector4d transed_mean_A = delta * mean_A;
    Eigen::Vector4d residual = mean_B - transed_mean_A;

    // Cholesky decomposition of Mahalanobis matrix
    Eigen::Matrix3d M3 = mahalanobis[i].topLeftCorner<3, 3>();
    Eigen::LLT<Eigen::Matrix3d> llt(M3);
    Eigen::Matrix3d L;
    if (llt.info() == Eigen::Success) {
      L = llt.matrixL();
    } else {
      L = M3.diagonal().cwiseSqrt().asDiagonal();
    }

    // Whitened residual: (L^T * r)[row]
    Eigen::Vector3d w_res = L.transpose() * residual.head<3>();
    double e_row = w_res[row];

    // Per-row error contribution
    sum_errors += w * e_row * e_row;

    if (!H_target) {
      continue;
    }

    // 3x6 Jacobians (same as original VGICP)
    Eigen::Matrix<double, 3, 6> J_target_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
    J_target_3x6.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.template head<3>());
    J_target_3x6.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 6> J_source_3x6 = Eigen::Matrix<double, 3, 6>::Zero();
    J_source_3x6.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source_3x6.block<3, 3>(0, 3) = -delta.linear();

    // Whiten the Jacobians: L^T * J, then take the selected row
    Eigen::Matrix<double, 3, 6> wJ_target = L.transpose() * J_target_3x6;
    Eigen::Matrix<double, 3, 6> wJ_source = L.transpose() * J_source_3x6;
    Eigen::Matrix<double, 1, 6> j_target_row = wJ_target.row(row);
    Eigen::Matrix<double, 1, 6> j_source_row = wJ_source.row(row);

    // Accumulate per-row Hessian and gradient contributions
    *H_target += w * j_target_row.transpose() * j_target_row;
    *H_source += w * j_source_row.transpose() * j_source_row;
    *H_target_source += w * j_target_row.transpose() * j_source_row;
    *b_target += w * j_target_row.transpose() * e_row;
    *b_source += w * j_source_row.transpose() * e_row;
  }

  // Regularize the Hessian to prevent ill-conditioning in ISAM2
  if (H_target) {
    const double reg = 1e-6;
    *H_target += reg * Eigen::Matrix<double, 6, 6>::Identity();
    *H_source += reg * Eigen::Matrix<double, 6, 6>::Identity();
  }
#ifdef GLIL_PROFILE_TIMING
  perftime::add(perftime::Counter::Evaluate, perftime::elapsed_us(profile_evaluate_t0, perftime::now()));
#endif

  return sum_errors;
}

}  // namespace glil
