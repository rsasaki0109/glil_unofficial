#include <glil/mapping/global_mapping.hpp>

#include <map>
#include <sstream>
#include <unordered_set>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/rotate_vector3_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glil/util/config.hpp>
#include <glil/util/serialization.hpp>
#include <glil/factors/integrated_coreset_factor.hpp>
#include <glil/factors/integrated_vgicp_coreset_factor.hpp>
#include <glil/common/imu_integration.hpp>
#include <glil/mapping/callbacks.hpp>

namespace glil {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

namespace {

std::string summarize_keys(const gtsam::KeyVector& keys, const size_t max_keys = 8) {
  std::ostringstream oss;

  for (size_t i = 0; i < keys.size() && i < max_keys; i++) {
    if (i) {
      oss << ", ";
    }
    oss << gtsam::Symbol(keys[i]);
  }

  if (keys.size() > max_keys) {
    oss << ", ...";
  }

  return oss.str();
}

std::string summarize_values(const gtsam::Values& values, const size_t max_keys = 8) {
  gtsam::KeyVector keys;
  keys.reserve(values.size());

  for (const auto& value : values) {
    keys.push_back(value.key);
  }

  return summarize_keys(keys, max_keys);
}

std::string classify_factor(const gtsam::NonlinearFactor::shared_ptr& factor) {
  if (!factor) {
    return "null";
  }
  if (std::dynamic_pointer_cast<gtsam_points::LinearDampingFactor>(factor)) {
    return "LinearDamping";
  }
  if (std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor)) {
    return "BetweenPose3";
  }
  if (std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Vector3>>(factor)) {
    return "BetweenVector3";
  }
  if (std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(factor)) {
    return "BetweenBias";
  }
  if (std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor)) {
    return "PriorPose3";
  }
  if (std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(factor)) {
    return "PriorVector3";
  }
  if (std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(factor)) {
    return "PriorBias";
  }
  if (std::dynamic_pointer_cast<gtsam::ImuFactor>(factor)) {
    return "ImuFactor";
  }
  if (std::dynamic_pointer_cast<gtsam_points::RotateVector3Factor>(factor)) {
    return "RotateVector3";
  }
  if (std::dynamic_pointer_cast<IntegratedVGICPCoresetFactor>(factor)) {
    return "IntegratedVGICPCoreset";
  }
  if (std::dynamic_pointer_cast<gtsam_points::IntegratedMatchingCostFactor>(factor)) {
    return "IntegratedMatchingCost";
  }

  return "Other";
}

std::string matching_cost_factor_type(const gtsam::NonlinearFactor::shared_ptr& factor) {
  if (std::dynamic_pointer_cast<IntegratedVGICPCoresetFactor>(factor)) {
    return "vgicp_coreset";
  }
  if (std::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactor>(factor)) {
    return "vgicp";
  }
#ifdef BUILD_GTSAM_POINTS_GPU
  if (std::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactorGPU>(factor)) {
    return "vgicp_gpu";
  }
#endif
  return "";
}

std::string summarize_factor_type_counts(const gtsam::NonlinearFactorGraph& graph) {
  std::map<std::string, int> counts;

  for (const auto& factor : graph) {
    counts[classify_factor(factor)]++;
  }

  std::ostringstream oss;
  bool first = true;
  for (const auto& [type, count] : counts) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << type << ":" << count;
  }

  return oss.str();
}

std::string summarize_key_support(const gtsam::NonlinearFactorGraph& graph, const char symbol, const size_t max_keys = 8) {
  std::map<size_t, int> counts;

  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }

    for (const auto key : factor->keys()) {
      const gtsam::Symbol sym(key);
      if (sym.chr() == symbol) {
        counts[sym.index()]++;
      }
    }
  }

  std::ostringstream oss;
  size_t emitted = 0;
  for (const auto& [index, count] : counts) {
    if (emitted) {
      oss << ", ";
    }
    oss << symbol << index << ":" << count;
    emitted++;
    if (emitted >= max_keys) {
      if (counts.size() > max_keys) {
        oss << ", ...";
      }
      break;
    }
  }

  return oss.str();
}

std::string summarize_factor_graph(const gtsam::NonlinearFactorGraph& graph, const size_t max_factors = 8) {
  std::ostringstream oss;

  for (size_t i = 0; i < graph.size() && i < max_factors; i++) {
    if (i) {
      oss << " | ";
    }

    const auto& factor = graph[i];
    if (!factor) {
      oss << "null";
      continue;
    }

    oss << classify_factor(factor) << "(" << summarize_keys(factor->keys()) << ", dim=" << factor->dim() << ")";
  }

  if (graph.size() > max_factors) {
    oss << " | ...";
  }

  return oss.str();
}

template <typename Derived>
std::string summarize_vector(const Eigen::MatrixBase<Derived>& values, const size_t max_values = 6) {
  std::ostringstream oss;

  for (size_t i = 0; i < static_cast<size_t>(values.size()) && i < max_values; i++) {
    if (i) {
      oss << ", ";
    }
    oss << values.derived()(static_cast<Eigen::Index>(i));
  }

  if (static_cast<size_t>(values.size()) > max_values) {
    oss << ", ...";
  }

  return oss.str();
}

void log_pending_update_summary(
  spdlog::logger* logger,
  const int current,
  const size_t between_factor_count,
  const size_t matching_factor_count,
  const size_t imu_factor_count,
  const gtsam::NonlinearFactorGraph& existing_factors,
  const gtsam::NonlinearFactorGraph& pending_factors,
  const gtsam::Values& pending_values) {
  logger->debug(
    "global update current={} existing_factors={} pending_factors={} (between={} matching={} imu={}) pending_values={} [{}]",
    current,
    existing_factors.size(),
    pending_factors.size(),
    between_factor_count,
    matching_factor_count,
    imu_factor_count,
    pending_values.size(),
    summarize_values(pending_values));

  logger->debug(
    "global update current={} pending_types=[{}] existing_x_support=[{}] pending_x_support=[{}]",
    current,
    summarize_factor_type_counts(pending_factors),
    summarize_key_support(existing_factors, 'x'),
    summarize_key_support(pending_factors, 'x'));

  if (current <= 2 || pending_factors.size() <= 8) {
    logger->debug("global update current={} pending_factor_list={}", current, summarize_factor_graph(pending_factors, 16));
  }
}

}  // namespace

GlobalMappingParams::GlobalMappingParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_imu = config.param<bool>("global_mapping", "enable_imu", true);
  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);

  enable_between_factors = config.param<bool>("global_mapping", "create_between_factors", false);
  between_registration_type = config.param<std::string>("global_mapping", "between_registration_type", "GICP");
  registration_error_factor_type = config.param<std::string>("global_mapping", "registration_error_factor_type", "VGICP");
  submap_voxel_resolution = config.param<double>("global_mapping", "submap_voxel_resolution", 1.0);
  submap_voxelmap_levels = config.param<int>("global_mapping", "submap_voxelmap_levels", 2);
  submap_voxelmap_scaling_factor = config.param<double>("global_mapping", "submap_voxelmap_scaling_factor", 2.0);

  randomsampling_rate = config.param<double>("global_mapping", "randomsampling_rate", 1.0);
  max_implicit_loop_distance = config.param<double>("global_mapping", "max_implicit_loop_distance", 100.0);
  min_implicit_loop_overlap = config.param<double>("global_mapping", "min_implicit_loop_overlap", 0.1);

  enable_gpu = registration_error_factor_type.find("GPU") != std::string::npos;

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);
}

GlobalMappingParams::~GlobalMappingParams() {}

GlobalMapping::GlobalMapping(const GlobalMappingParams& params) : params(params) {
#ifndef BUILD_GTSAM_POINTS_GPU
  if (params.enable_gpu) {
    logger->error("GPU-based factors cannot be used because GLIM is built without GPU option!!");
  }
#endif

  imu_integration.reset(new IMUIntegration);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);
  last_good_graph.reset(new gtsam::NonlinearFactorGraph);
  last_good_values.reset(new gtsam::Values);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }
#ifdef BUILD_GTSAM_POINTS_GPU
  stream_buffer_roundrobin = std::make_shared<gtsam_points::StreamTempBufferRoundRobin>(64);
#endif
}

GlobalMapping::~GlobalMapping() {}

void GlobalMapping::refresh_last_good_snapshot() {
  try {
    last_good_graph.reset(new gtsam::NonlinearFactorGraph(isam2->getFactorsUnsafe()));
    last_good_values.reset(new gtsam::Values(isam2->calculateEstimate()));
    last_good_snapshot_valid = true;
  } catch (std::exception& e) {
    logger->warn("failed to refresh last-good iSAM2 snapshot: {}", e.what());
  }
}

void GlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  if (params.enable_imu) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);
  }
}

void GlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={} |frame|={}", submap->id, submap->frame->size());

  const int current = submaps.size();
  const int last = current - 1;
  insert_submap(current, submap);

  // If iSAM2 is unavailable for new submaps, keep exporting the odometry chain.
  if (isam2_corrupted) {
    if (current != 0) {
      const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
      const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
      const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
      const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();
      submap->T_world_origin = submaps[last]->T_world_origin * T_origin0_origin1;
    }
    Callbacks::on_insert_submap(submap);
    submap->drop_frame_points();
    return;
  }

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      try {
        last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
      } catch (std::exception& e) {
        logger->warn("failed to get last submap pose from ISAM2, using cached: {}", e.what());
        last_T_world_submap = gtsam::Pose3(submaps[last]->T_world_origin.matrix());
      }
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  size_t between_factor_count = 0;
  size_t matching_factor_count = 0;
  if (current == 0) {
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(0),
      current_T_world_submap,
      gtsam::noiseModel::Isotropic::Precision(6, params.init_pose_damping_scale));
  } else {
    const auto between_factors = create_between_factors(current);
    const auto matching_factors = create_matching_cost_factors(current);
    between_factor_count = between_factors->size();
    matching_factor_count = matching_factors->size();
    new_factors->add(*between_factors);
    new_factors->add(*matching_factors);
  }

  const size_t num_factors_before_imu = new_factors->size();
  if (params.enable_imu) {
    if (submap->odom_frames.front()->frame_id != FrameID::IMU) {
      logger->warn("odom frames are not estimated in the IMU frame while global mapping requires IMU estimation");
    }

    // Local velocities
    const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);

    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

    const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
    const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

    if (current > 0) {
      new_values->insert(E(current * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      new_values->insert(V(current * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      new_values->insert(B(current * 2), imu_biasL);

      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2), v_origin_imuL, prior_noise3);
      new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), imu_biasL, prior_noise6);
      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), B(current * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
    }

    new_values->insert(E(current * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    new_values->insert(V(current * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    new_values->insert(B(current * 2 + 1), imu_biasR);

    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2 + 1), v_origin_imuR, prior_noise3);
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2 + 1), imu_biasR, prior_noise6);

    if (current != 0) {
      const double stampL = submaps[last]->frames.back()->stamp;
      const double stampR = submaps[current]->frames.front()->stamp;

      int num_integrated;
      const int imu_read_cursor = imu_integration->integrate_imu(stampL, stampR, imu_biasL, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      if (num_integrated < 2) {
        logger->warn("insufficient IMU data between submaps (global_mapping)!!");
        new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last * 2 + 1), V(current * 2), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      } else {
        new_factors
          ->emplace_shared<gtsam::ImuFactor>(E(last * 2 + 1), V(last * 2 + 1), E(current * 2), V(current * 2), B(last * 2 + 1), imu_integration->integrated_measurements());
      }
    }
  }

  if (!isam2_corrupted) {
    const size_t imu_factor_count = new_factors->size() - num_factors_before_imu;
    log_pending_update_summary(logger.get(), current, between_factor_count, matching_factor_count, imu_factor_count, isam2->getFactorsUnsafe(), *new_factors, *new_values);
    try {
      Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
      auto result = isam2->update(*new_factors, *new_values);
      Callbacks::on_smoother_update_result(*isam2, result);
      refresh_last_good_snapshot();
    } catch (std::exception& e) {
      logger->error("an exception was caught during global map optimization!!");
      logger->error(e.what());
      logger->error(
        "global update failure current={} pending_values={} [{}] pending_types=[{}] pending_x_support=[{}]",
        current,
        new_values->size(),
        summarize_values(*new_values),
        summarize_factor_type_counts(*new_factors),
        summarize_key_support(*new_factors, 'x'));
      logger->error("global update failure current={} pending_factor_list={}", current, summarize_factor_graph(*new_factors, 16));
      logger->error("disabling further ISAM2 updates to prevent cascading failures");
      isam2_corrupted = true;
    }
  }
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  if (!isam2_corrupted) {
    update_submaps();
    Callbacks::on_update_submaps(submaps);
  }
}

void GlobalMapping::insert_submap(int current, const SubMap::Ptr& submap) {
  submap->voxelmaps.clear();

  gtsam_points::PointCloud::Ptr subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);

#ifdef BUILD_GTSAM_POINTS_GPU
  if (params.enable_gpu && !submap->frame->points_gpu) {
    submap->frame = gtsam_points::PointCloudGPU::clone(*submap->frame);
  }

  if (params.enable_gpu) {
    subsampled_submap = gtsam_points::PointCloudGPU::clone(*subsampled_submap);

    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = params.submap_voxel_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
      voxelmap->insert(*subsampled_submap);
      submap->voxelmaps.push_back(voxelmap);
    }
  }
#endif

  if (submap->voxelmaps.empty()) {
    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = params.submap_voxel_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*subsampled_submap);
      submap->voxelmaps.push_back(voxelmap);
    }
  }

  submaps.push_back(submap);
  subsampled_submaps.push_back(subsampled_submap);
}

void GlobalMapping::find_overlapping_submaps(double min_overlap) {
  if (submaps.empty()) {
    return;
  }

  // Between factors are Vector2i actually. A bad use of Vector3i
  std::unordered_set<Eigen::Vector3i, gtsam_points::Vector3iHash> existing_factors;
  for (const auto& factor : isam2->getFactorsUnsafe()) {
    if (factor->keys().size() != 2) {
      continue;
    }

    gtsam::Symbol sym1(factor->keys()[0]);
    gtsam::Symbol sym2(factor->keys()[1]);
    if (sym1.chr() != 'x' || sym2.chr() != 'x') {
      continue;
    }

    existing_factors.emplace(sym1.index(), sym2.index(), 0);
  }

  gtsam::NonlinearFactorGraph new_factors;

  for (int i = 0; i < submaps.size(); i++) {
    for (int j = i + 1; j < submaps.size(); j++) {
      if (existing_factors.count(Eigen::Vector3i(i, j, 0))) {
        continue;
      }

      const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * submaps[j]->T_world_origin;
      const double dist = delta.translation().norm();
      if (dist > params.max_implicit_loop_distance) {
        continue;
      }

      const double overlap = gtsam_points::overlap_auto(submaps[i]->voxelmaps.back(), subsampled_submaps[j], delta);
      if (overlap < min_overlap) {
        continue;
      }

      if (params.registration_error_factor_type == "VGICP_CORESET") {
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors.emplace_shared<IntegratedVGICPCoresetFactor>(X(i), X(j), voxelmap, subsampled_submaps[j]);
        }
      }
#ifdef BUILD_GTSAM_POINTS_GPU
      else if (std::dynamic_pointer_cast<gtsam_points::GaussianVoxelMapGPU>(submaps[i]->voxelmaps.back()) && subsampled_submaps[j]->points_gpu) {
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(j), voxelmap, subsampled_submaps[j], stream, buffer);
        }
      }
#endif
      else {
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors.emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(j), voxelmap, subsampled_submaps[j]);
        }
      }
    }
  }

  logger->info("new overlapping {} submap pairs found", new_factors.size());

  if (!isam2_corrupted) {
    gtsam::Values new_values;
    try {
      Callbacks::on_smoother_update(*isam2, new_factors, new_values);
      auto result = isam2->update(new_factors, new_values);
      Callbacks::on_smoother_update_result(*isam2, result);
      refresh_last_good_snapshot();
    } catch (std::exception& e) {
      logger->error("an exception was caught during global map optimization (find_overlapping)!!");
      logger->error(e.what());
      isam2_corrupted = true;
    }

    if (!isam2_corrupted) {
      update_submaps();
      Callbacks::on_update_submaps(submaps);
    }
  }
}

void GlobalMapping::optimize() {
  if (isam2_corrupted || isam2->empty()) {
    return;
  }

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  try {
    Callbacks::on_smoother_update(*isam2, new_factors, new_values);
    auto result = isam2->update(new_factors, new_values);
    Callbacks::on_smoother_update_result(*isam2, result);
    refresh_last_good_snapshot();
  } catch (std::exception& e) {
    logger->error("an exception was caught during global map final optimization!!");
    logger->error(e.what());
    isam2_corrupted = true;
  }

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_between_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0 || !params.enable_between_factors) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());

  if (params.between_registration_type == "NONE") {
    factors->add(std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
    return factors;
  }

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), init_delta);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = std::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), submaps[last]->frame, submaps[current]->frame);
  factor->set_max_correspondence_distance(0.5);
  factor->set_num_threads(2);
  graph.add(factor);

  logger->debug("--- LM optimization ---");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(10);
  lm_params.callback = [this](const auto& status, const auto& values) { logger->debug(status.to_string()); };

  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  const gtsam::Pose3 estimated_delta = values.at<gtsam::Pose3>(X(1));
  const auto linearized = factor->linearize(values);
  const gtsam::Matrix6 raw_H = linearized->hessianBlockDiagonal()[X(1)];
  const double raw_symmetry_error = (raw_H - raw_H.transpose()).cwiseAbs().maxCoeff();
  Eigen::LLT<gtsam::Matrix6> raw_llt(raw_H);
  const bool raw_h_spd = raw_llt.info() == Eigen::Success;

  gtsam::Matrix6 H = 0.5 * (raw_H + raw_H.transpose());
  H += 1e6 * gtsam::Matrix6::Identity();

  Eigen::LLT<gtsam::Matrix6> llt(H);
  if (llt.info() != Eigen::Success) {
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(H);
    if (eig.info() == Eigen::Success) {
      const double min_eig = eig.eigenvalues().minCoeff();
      const double jitter = std::max(1e-6, -min_eig + 1e-6);
      logger->warn(
        "between factor current={} required additional SPD jitter: min_eig={:.6e} jitter={:.6e}",
        current,
        min_eig,
        jitter);
      H += jitter * gtsam::Matrix6::Identity();
    } else {
      logger->warn("between factor current={} eigen decomposition failed while stabilizing H, falling back to isotropic information", current);
      H = 1e6 * gtsam::Matrix6::Identity();
    }
    llt.compute(H);
  }
  const bool h_spd = llt.info() == Eigen::Success;

  if (!raw_h_spd || raw_symmetry_error > 1e-3 || !H.allFinite() || !h_spd) {
    logger->warn(
      "between factor current={} last={} points=({}, {}) init_trans_norm={:.6f} est_trans_norm={:.6f} raw_H_spd={} raw_symmetry_error={:.3e} H_finite={} H_spd={} H_diag=[{}]",
      current,
      last,
      submaps[last]->frame->size(),
      submaps[current]->frame->size(),
      init_delta.translation().norm(),
      estimated_delta.translation().norm(),
      raw_h_spd,
      raw_symmetry_error,
      H.allFinite(),
      h_spd,
      summarize_vector(H.diagonal()));
  } else if (current <= 2) {
    logger->debug(
      "between factor current={} last={} points=({}, {}) init_trans_norm={:.6f} est_trans_norm={:.6f} raw_H_spd={} raw_symmetry_error={:.3e} H_finite={} H_spd={} H_diag=[{}]",
      current,
      last,
      submaps[last]->frame->size(),
      submaps[current]->frame->size(),
      init_delta.translation().norm(),
      estimated_delta.translation().norm(),
      raw_h_spd,
      raw_symmetry_error,
      H.allFinite(),
      h_spd,
      summarize_vector(H.diagonal()));
  }

  factors->add(std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), estimated_delta, gtsam::noiseModel::Gaussian::Information(H)));
  return factors;
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_matching_cost_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();

  for (int i = 0; i < current; i++) {
    const double dist = (submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).norm();
    if (dist > params.max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = gtsam_points::overlap_auto(submaps[i]->voxelmaps.back(), current_submap->frame, delta);

    if (overlap < params.min_implicit_loop_overlap) {
      continue;
    }

    if (params.registration_error_factor_type == "VGICP") {
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(current), voxelmap, subsampled_submaps[current]);
      }
    } else if (params.registration_error_factor_type == "VGICP_CORESET") {
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<IntegratedVGICPCoresetFactor>(X(i), X(current), voxelmap, subsampled_submaps[current]);
      }
    }
#ifdef BUILD_GTSAM_POINTS_GPU
    else if (params.registration_error_factor_type == "VGICP_GPU") {
      const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(current), voxelmap, subsampled_submaps[current], stream, buffer);
      }
    }
#endif
    else {
      logger->warn("unknown registration error type ({})", params.registration_error_factor_type);
    }
  }

  return factors;
}

void GlobalMapping::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    try {
      submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
    } catch (std::exception& e) {
      logger->warn("failed to update submap {} pose: {}", i, e.what());
    }
  }
}

void GlobalMapping::save(const std::string& path) {
  optimize();

  boost::filesystem::create_directories(path);
  std::vector<std::tuple<std::string, int, int>> serialized_matching_cost_factors;

  const bool serialize_from_snapshot = isam2_corrupted && last_good_snapshot_valid;
  if (isam2_corrupted && !last_good_snapshot_valid) {
    logger->warn("ISAM2 was corrupted during processing and no last-good snapshot is available; skipping graph/values serialization");
  }

  if (!isam2_corrupted || serialize_from_snapshot) {
    const gtsam::NonlinearFactorGraph& source_factors = serialize_from_snapshot ? *last_good_graph : isam2->getFactorsUnsafe();

    gtsam::NonlinearFactorGraph serializable_factors;
    std::unordered_map<std::string, gtsam::NonlinearFactor::shared_ptr> matching_cost_factors;

    for (const auto& factor : source_factors) {
      bool serializable = !std::dynamic_pointer_cast<gtsam_points::IntegratedMatchingCostFactor>(factor)
#ifdef BUILD_GTSAM_POINTS_GPU
                          && !std::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactorGPU>(factor)
#endif
        ;

      if (serializable) {
        serializable_factors.push_back(factor);
      } else {
        const gtsam::Symbol symbol0(factor->keys()[0]);
        const gtsam::Symbol symbol1(factor->keys()[1]);
        const std::string key = std::to_string(symbol0.index()) + "_" + std::to_string(symbol1.index());

        matching_cost_factors[key] = factor;
      }
    }

    serialized_matching_cost_factors.reserve(matching_cost_factors.size());
    for (const auto& [_, factor] : matching_cost_factors) {
      const std::string type = matching_cost_factor_type(factor);
      if (type.empty()) {
        logger->warn("skip unsupported matching cost factor during serialization: {}", classify_factor(factor));
        continue;
      }

      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);
      serialized_matching_cost_factors.emplace_back(type, symbol0.index(), symbol1.index());
    }

    if (serialize_from_snapshot) {
      logger->warn(
        "ISAM2 was corrupted during processing; serializing last-good snapshot (factors={}, values={}) to {}/graph.bin and values.bin",
        serializable_factors.size(),
        last_good_values->size(),
        path);
    } else {
      logger->info("serializing factor graph to {}/graph.bin", path);
    }
    serializeToBinaryFile(serializable_factors, path + "/graph.bin");
    try {
      if (serialize_from_snapshot) {
        serializeToBinaryFile(*last_good_values, path + "/values.bin");
      } else {
        serializeToBinaryFile(isam2->calculateEstimate(), path + "/values.bin");
      }
    } catch (std::exception& e) {
      logger->error("failed to serialize values: {}", e.what());
    }
  }

  std::ofstream ofs(path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->frames.size(); }) << std::endl;
  ofs << "num_matching_cost_factors: " << serialized_matching_cost_factors.size() << std::endl;
  for (const auto& [type, first, second] : serialized_matching_cost_factors) {
    ofs << "matching_cost_factor: " << type << " " << first << " " << second << std::endl;
  }

  std::ofstream odom_lidar_ofs(path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    submaps[i]->save((boost::format("%s/%06d") % path % i).str());
  }
}

std::vector<Eigen::Vector4d> GlobalMapping::export_points() {
  int num_all_points = 0;
  for (const auto& submap : submaps) {
    num_all_points += submap->frame->size();
  }

  std::vector<Eigen::Vector4d> all_points;
  all_points.reserve(num_all_points);

  for (const auto& submap : submaps) {
    std::transform(submap->frame->points, submap->frame->points + submap->frame->size(), std::back_inserter(all_points), [&](const Eigen::Vector4d& p) {
      return submap->T_world_origin * p;
    });
  }

  return all_points;
}

bool GlobalMapping::load(const std::string& path) {
  std::ifstream ifs(path + "/graph.txt");
  if (!ifs) {
    logger->error("failed to open {}/graph.txt", path);
    return false;
  }

  std::string token;
  int num_submaps, num_all_frames, num_matching_cost_factors;

  ifs >> token >> num_submaps;
  ifs >> token >> num_all_frames;
  ifs >> token >> num_matching_cost_factors;

  std::vector<std::tuple<std::string, int, int>> matching_cost_factors(num_matching_cost_factors);
  for (int i = 0; i < num_matching_cost_factors; i++) {
    auto& factor = matching_cost_factors[i];
    ifs >> token >> std::get<0>(factor) >> std::get<1>(factor) >> std::get<2>(factor);
  }

  logger->info("Load submaps");
  submaps.resize(num_submaps);
  subsampled_submaps.resize(num_submaps);
  for (int i = 0; i < num_submaps; i++) {
    auto submap = SubMap::load((boost::format("%s/%06d") % path % i).str());
    if (!submap) {
      return false;
    }

    gtsam_points::PointCloud::Ptr subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);

    submaps[i] = submap;
    submaps[i]->voxelmaps.clear();
    subsampled_submaps[i] = subsampled_submap;

    if (params.enable_gpu) {
#ifdef BUILD_GTSAM_POINTS_GPU
      subsampled_submaps[i] = gtsam_points::PointCloudGPU::clone(*subsampled_submaps[i]);

      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = params.submap_voxel_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
        voxelmap->insert(*subsampled_submaps[i]);
        submaps[i]->voxelmaps.push_back(voxelmap);
      }
#else
      logger->warn("GPU is enabled for global_mapping but gtsam_points was built without CUDA!!");
#endif
    } else {
      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = params.submap_voxel_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*subsampled_submaps[i]);
        submaps[i]->voxelmaps.push_back(voxelmap);
      }
    }

    Callbacks::on_insert_submap(submap);
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  try {
    logger->info("deserializing factor graph");
    gtsam::deserializeFromBinaryFile(path + "/graph.bin", graph);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
  }
  try {
    logger->info("deserializing values");
    gtsam::deserializeFromBinaryFile(path + "/values.bin", values);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
  }

  logger->info("creating matching cost factors");
  for (const auto& factor : matching_cost_factors) {
    const auto type = std::get<0>(factor);
    const auto first = std::get<1>(factor);
    const auto second = std::get<2>(factor);

    if (type == "vgicp" || type == "vgicp_gpu") {
      if (params.enable_gpu) {
#ifdef BUILD_GTSAM_POINTS_GPU
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;

        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(first), X(second), voxelmap, subsampled_submaps[second], stream, buffer);
        }
#else
        logger->warn("GPU is enabled but gtsam_points was built without CUDA!!");
#endif
      } else {
        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(first), X(second), voxelmap, subsampled_submaps[second]);
        }
      }
    } else if (type == "vgicp_coreset") {
      for (const auto& voxelmap : submaps[first]->voxelmaps) {
        graph.emplace_shared<IntegratedVGICPCoresetFactor>(X(first), X(second), voxelmap, subsampled_submaps[second]);
      }
    } else {
      logger->warn("unsupported matching cost factor type ({})", type);
    }
  }

  logger->info("optimize");
  Callbacks::on_smoother_update(*isam2, graph, values);
  auto result = isam2->update(graph, values);
  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);

  logger->info("done");

  return true;
}

}  // namespace glil
