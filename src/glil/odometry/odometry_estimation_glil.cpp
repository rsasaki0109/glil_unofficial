#include <glil/odometry/odometry_estimation_glil.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/ann/fast_occupancy_grid.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glil/util/config.hpp>
#ifdef GLIL_PROFILE_TIMING
#include <glil/util/perftime.hpp>
#endif
#include <glil/common/imu_integration.hpp>
#include <glil/common/cloud_deskewing.hpp>
#include <glil/common/cloud_covariance_estimation.hpp>
#include <glil/factors/integrated_coreset_factor.hpp>
#include <glil/factors/integrated_vgicp_coreset_factor.hpp>

#include <glil/odometry/callbacks.hpp>

namespace glil {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity
using gtsam::symbol_shorthand::X;  // IMU pose

OdometryEstimationGLILParams::OdometryEstimationGLILParams() : OdometryEstimationIMUParams() {
  Config config(GlobalConfig::get_config_path("config_odometry"));

  voxel_resolution = config.param<double>("odometry_estimation", "voxel_resolution", 0.5);
  voxelmap_levels = config.param<int>("odometry_estimation", "voxelmap_levels", 2);
  voxelmap_scaling_factor = config.param<double>("odometry_estimation", "voxelmap_scaling_factor", 2.0);

  max_num_keyframes = config.param<int>("odometry_estimation", "max_num_keyframes", 10);
  full_connection_window_size = config.param<int>("odometry_estimation", "full_connection_window_size", 3);

  const std::string strategy = config.param<std::string>("odometry_estimation", "keyframe_update_strategy", "DISPLACEMENT");
  if (strategy == "OVERLAP") {
    keyframe_strategy = KeyframeUpdateStrategy::OVERLAP;
  } else if (strategy == "DISPLACEMENT") {
    keyframe_strategy = KeyframeUpdateStrategy::DISPLACEMENT;
  } else {
    spdlog::error("unknown keyframe update strategy {}", strategy);
    keyframe_strategy = KeyframeUpdateStrategy::DISPLACEMENT;
  }

  keyframe_min_overlap = config.param<double>("odometry_estimation", "keyframe_min_overlap", 0.1);
  keyframe_max_overlap = config.param<double>("odometry_estimation", "keyframe_max_overlap", 0.9);
  keyframe_delta_trans = config.param<double>("odometry_estimation", "keyframe_delta_trans", 1.0);
  keyframe_delta_rot = config.param<double>("odometry_estimation", "keyframe_delta_rot", 0.25);

  occupancy_grid_resolution = config.param<double>("odometry_estimation", "occupancy_grid_resolution", 0.5);

  coreset_params.relinearize_thresh_trans = config.param<double>("odometry_estimation", "coreset_relinearize_thresh_trans", 0.25);
  coreset_params.relinearize_thresh_rot = config.param<double>("odometry_estimation", "coreset_relinearize_thresh_rot", 0.25);
  coreset_params.coreset_target_size = config.param<int>("odometry_estimation", "coreset_target_size", 256);
  coreset_params.coreset_num_clusters = config.param<int>("odometry_estimation", "coreset_num_clusters", 64);
  coreset_method = config.param<std::string>("odometry_estimation", "coreset_method", "exact_caratheodory");
  correspondence_sample_ratio = config.param<double>("odometry_estimation", "correspondence_sample_ratio", 1.0);
  coreset_factor_lock = config.param<bool>("odometry_estimation", "coreset_factor_lock", false);
  coreset_immutable_snapshot = config.param<bool>("odometry_estimation", "coreset_immutable_snapshot", false);
  if (coreset_method != "exact_caratheodory" && coreset_method != "uniform_sample" &&
      coreset_method != "uniform_sample_early" && coreset_method != "residual_weighted") {
    spdlog::warn("unknown coreset_method={}; falling back to exact_caratheodory", coreset_method);
    coreset_method = "exact_caratheodory";
  }
  if (correspondence_sample_ratio < 0.0) {
    spdlog::warn("correspondence_sample_ratio={} is below 0.0; clamping to 0.0", correspondence_sample_ratio);
    correspondence_sample_ratio = 0.0;
  } else if (correspondence_sample_ratio > 1.0) {
    spdlog::warn("correspondence_sample_ratio={} is above 1.0; clamping to 1.0", correspondence_sample_ratio);
    correspondence_sample_ratio = 1.0;
  }
}

OdometryEstimationGLILParams::~OdometryEstimationGLILParams() {}

OdometryEstimationGLIL::OdometryEstimationGLIL(const OdometryEstimationGLILParams& params) : OdometryEstimationIMU(std::make_unique<OdometryEstimationGLILParams>(params)) {
  marginalized_cursor = 0;
}

OdometryEstimationGLIL::~OdometryEstimationGLIL() {
  frames.clear();
  keyframes.clear();
  smoother.reset();
}

void OdometryEstimationGLIL::create_frame(EstimationFrame::Ptr& new_frame) {
  const auto params = static_cast<OdometryEstimationGLILParams*>(this->params.get());

  // Create CPU voxelmaps for the frame (same structure as GPU version but using CPU voxelmaps)
  for (int i = 0; i < params->voxelmap_levels; i++) {
    if (!new_frame->frame->size()) {
      break;
    }

    const double resolution = params->voxel_resolution * std::pow(params->voxelmap_scaling_factor, i);
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    voxelmap->insert(*new_frame->frame);
    new_frame->voxelmaps.push_back(voxelmap);
  }
}

void OdometryEstimationGLIL::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationIMU::update_frames(current, new_factors);

  const auto params = static_cast<OdometryEstimationGLILParams*>(this->params.get());
  switch (params->keyframe_strategy) {
    case OdometryEstimationGLILParams::KeyframeUpdateStrategy::OVERLAP:
      update_keyframes_overlap(current);
      break;
    case OdometryEstimationGLILParams::KeyframeUpdateStrategy::DISPLACEMENT:
      update_keyframes_displacement(current);
      break;
  }

  Callbacks::on_update_keyframes(keyframes);
}

gtsam::NonlinearFactorGraph OdometryEstimationGLIL::create_factors(const int current, const std::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
#ifdef GLIL_PROFILE_TIMING
  perftime::ScopedTimer profile_create_factors(perftime::Counter::CreateFactors);
#endif
  if (current == 0 || !frames[current]->frame->size()) {
    return gtsam::NonlinearFactorGraph();
  }

  const auto glil_params = static_cast<OdometryEstimationGLILParams*>(this->params.get());
  const bool digest_frame =
    glil_params->debug_digest &&
    (debug_frame_enabled(current, frames[current]->stamp) ||
     (glil_params->debug_frame_window_start < 0 && glil_params->debug_stamp_window_start < 0.0));

  // Coreset params for VGICP factors
  IntegratedVGICPCoresetFactor::Params vgicp_coreset_params;
  vgicp_coreset_params.relinearize_thresh_trans = glil_params->coreset_params.relinearize_thresh_trans;
  vgicp_coreset_params.relinearize_thresh_rot = glil_params->coreset_params.relinearize_thresh_rot;
  vgicp_coreset_params.coreset_target_size = glil_params->coreset_params.coreset_target_size;
  vgicp_coreset_params.coreset_num_clusters = glil_params->coreset_params.coreset_num_clusters;
  vgicp_coreset_params.coreset_method = glil_params->coreset_method;
  vgicp_coreset_params.correspondence_sample_ratio = glil_params->correspondence_sample_ratio;
  vgicp_coreset_params.num_threads = glil_params->registration_num_threads;
  vgicp_coreset_params.coreset_factor_lock = glil_params->coreset_factor_lock;
  vgicp_coreset_params.coreset_immutable_snapshot = glil_params->coreset_immutable_snapshot;
  vgicp_coreset_params.debug_digest = digest_frame;
  vgicp_coreset_params.debug_frame_id = current;
  int debug_factor_idx = 0;

  // Lambda to create a binary VGICP factor with true coreset extraction
  const auto create_binary_factor = [&](gtsam::NonlinearFactorGraph& factors, gtsam::Key target_key, gtsam::Key source_key, const EstimationFrame::ConstPtr& target,
                                        const EstimationFrame::ConstPtr& source) {
    for (const auto& voxelmap : target->voxelmaps) {
      auto factor_params = vgicp_coreset_params;
      factor_params.debug_factor_idx = debug_factor_idx++;
      auto factor = std::make_shared<IntegratedVGICPCoresetFactor>(target_key, source_key, voxelmap, source->frame, factor_params);
      factors.add(factor);
    }
  };

  // Lambda to create a unary VGICP factor with true coreset extraction
  const auto create_unary_factor = [&](gtsam::NonlinearFactorGraph& factors, const gtsam::Pose3& fixed_target_pose, gtsam::Key source_key,
                                       const EstimationFrame::ConstPtr& target, const EstimationFrame::ConstPtr& source) {
    for (const auto& voxelmap : target->voxelmaps) {
      auto factor_params = vgicp_coreset_params;
      factor_params.debug_factor_idx = debug_factor_idx++;
      auto factor = std::make_shared<IntegratedVGICPCoresetFactor>(fixed_target_pose, source_key, voxelmap, source->frame, factor_params);
      factors.add(factor);
    }
  };

  gtsam::NonlinearFactorGraph factors;

  // Factors between consecutive frames in the full connection window
  for (int target = current - glil_params->full_connection_window_size; target < current; target++) {
    if (target < 0) {
      continue;
    }
    create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
  }

  // Factors to keyframes
  for (const auto& keyframe : keyframes) {
    if (keyframe->id >= current - glil_params->full_connection_window_size) {
      continue;
    }

    double span = frames[current]->stamp - keyframe->stamp;
    if (span > glil_params->smoother_lag - 0.1) {
      const gtsam::Pose3 key_T_world_imu(keyframe->T_world_imu.matrix());
      create_unary_factor(factors, key_T_world_imu, X(current), keyframe, frames[current]);
    } else {
      const int target = keyframe->id;
      create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
    }
  }

  return factors;
}

double OdometryEstimationGLIL::calc_overlap(
  const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
  const gtsam_points::PointCloud::ConstPtr& source,
  const Eigen::Isometry3d& delta) const {
  const auto glil_params = static_cast<OdometryEstimationGLILParams*>(this->params.get());
  // Use FastOccupancyGrid for efficient overlap computation (GLIL paper, ~2.5x faster)
  gtsam_points::FastOccupancyGrid grid(glil_params->occupancy_grid_resolution);
  grid.insert(*source, Eigen::Isometry3d::Identity());
  return grid.calc_overlap_rate(*source, delta);
}

double OdometryEstimationGLIL::calc_overlap_multi(
  const std::vector<gtsam_points::GaussianVoxelMap::ConstPtr>& target_voxelmaps,
  const gtsam_points::PointCloud::ConstPtr& source,
  const std::vector<Eigen::Isometry3d>& deltas) const {
  const auto glil_params = static_cast<OdometryEstimationGLILParams*>(this->params.get());
  // Build a single FastOccupancyGrid from all target voxelmaps' points, then compute overlap
  // For multi-keyframe overlap, we insert all keyframe points into one grid
  double max_overlap = 0.0;
  for (size_t i = 0; i < target_voxelmaps.size(); i++) {
    gtsam_points::FastOccupancyGrid grid(glil_params->occupancy_grid_resolution);
    // Insert source points at identity, then check overlap at delta
    grid.insert(*source, Eigen::Isometry3d::Identity());
    max_overlap = std::max(max_overlap, grid.calc_overlap_rate(*source, deltas[i]));
  }
  return max_overlap;
}

/**
 * @brief Keyframe management based on overlap metric (CPU version using overlap())
 */
void OdometryEstimationGLIL::update_keyframes_overlap(int current) {
  const auto params = static_cast<OdometryEstimationGLILParams*>(this->params.get());

  if (!frames[current]->frame->size()) {
    return;
  }

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  // Calculate overlap with all keyframes
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> keyframe_voxelmaps(keyframes.size());
  std::vector<Eigen::Isometry3d> delta_from_keyframes(keyframes.size());
  for (size_t i = 0; i < keyframes.size(); i++) {
    keyframe_voxelmaps[i] = keyframes[i]->voxelmaps.back();
    delta_from_keyframes[i] = keyframes[i]->T_world_imu.inverse() * frames[current]->T_world_imu;
  }

  const double overlap = calc_overlap_multi(keyframe_voxelmaps, frames[current]->frame, delta_from_keyframes);
  if (overlap > params->keyframe_max_overlap) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= static_cast<size_t>(params->max_num_keyframes)) {
    return;
  }

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;

  // Remove keyframes without overlap to the new keyframe
  for (size_t i = 0; i < keyframes.size(); i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    gtsam_points::FastOccupancyGrid grid(params->occupancy_grid_resolution);
    grid.insert(*keyframes[i]->frame, Eigen::Isometry3d::Identity());
    const double ov = grid.calc_overlap_rate(*new_keyframe->frame, delta);
    if (ov < params->keyframe_min_overlap) {
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      i--;
    }
  }

  if (keyframes.size() <= static_cast<size_t>(params->max_num_keyframes)) {
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    return;
  }

  // Remove the keyframe with the minimum score
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (size_t i = 0; i < keyframes.size() - 1; i++) {
    const auto& keyframe = keyframes[i];
    gtsam_points::FastOccupancyGrid score_grid(params->occupancy_grid_resolution);
    score_grid.insert(*keyframe->frame, Eigen::Isometry3d::Identity());
    const double overlap_latest = score_grid.calc_overlap_rate(*new_keyframe->frame, keyframe->T_world_imu.inverse() * new_keyframe->T_world_imu);

    std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> other_keyframes;
    std::vector<Eigen::Isometry3d> delta_from_others;
    for (size_t j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }
      const auto& other = keyframes[j];
      other_keyframes.push_back(other->voxelmaps.back());
      delta_from_others.push_back(other->T_world_imu.inverse() * keyframe->T_world_imu);
    }

    const double overlap_others = calc_overlap_multi(other_keyframes, keyframe->frame, delta_from_others);
    scores[i] = overlap_latest * (1.0 - overlap_others);
  }

  double min_score = scores[0];
  int frame_to_eliminate = 0;
  for (size_t i = 1; i < scores.size(); i++) {
    if (scores[i] < min_score) {
      min_score = scores[i];
      frame_to_eliminate = i;
    }
  }

  marginalized_keyframes.push_back(keyframes[frame_to_eliminate]);
  keyframes.erase(keyframes.begin() + frame_to_eliminate);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/**
 * @brief Keyframe management based on displacement criteria (CPU version)
 */
void OdometryEstimationGLIL::update_keyframes_displacement(int current) {
  const auto params = static_cast<OdometryEstimationGLILParams*>(this->params.get());

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  const Eigen::Isometry3d delta_from_last = keyframes.back()->T_world_imu.inverse() * frames[current]->T_world_imu;
  const double delta_trans = delta_from_last.translation().norm();
  const double delta_rot = Eigen::AngleAxisd(delta_from_last.linear()).angle();

  if (delta_trans < params->keyframe_delta_trans && delta_rot < params->keyframe_delta_rot) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= static_cast<size_t>(params->max_num_keyframes)) {
    return;
  }

  // Remove keyframes with very low overlap
  for (size_t i = 0; i < keyframes.size() - 1; i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    gtsam_points::FastOccupancyGrid disp_grid(params->occupancy_grid_resolution);
    disp_grid.insert(*keyframes[i]->frame, Eigen::Isometry3d::Identity());
    const double overlap = disp_grid.calc_overlap_rate(*new_keyframe->frame, delta);

    if (overlap < 0.01) {
      std::vector<EstimationFrame::ConstPtr> marginalized;
      marginalized.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      Callbacks::on_marginalized_keyframes(marginalized);
      return;
    }
  }

  // Score-based removal
  const int leave_window = 2;
  const double eps = 1e-3;
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (size_t i = leave_window; i < keyframes.size() - 1; i++) {
    double sum_inv_dist = 0.0;
    for (size_t j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }
      const double dist = (keyframes[i]->T_world_imu.translation() - keyframes[j]->T_world_imu.translation()).norm();
      sum_inv_dist += 1.0 / (dist + eps);
    }

    const double d0 = (keyframes[i]->T_world_imu.translation() - new_keyframe->T_world_imu.translation()).norm();
    scores[i] = std::sqrt(d0) * sum_inv_dist;
  }

  const auto max_score_loc = std::max_element(scores.begin(), scores.end());
  const int max_score_index = std::distance(scores.begin(), max_score_loc);

  std::vector<EstimationFrame::ConstPtr> marginalized;
  marginalized.push_back(keyframes[max_score_index]);
  keyframes.erase(keyframes.begin() + max_score_index);
  Callbacks::on_marginalized_keyframes(marginalized);
}

}  // namespace glil
