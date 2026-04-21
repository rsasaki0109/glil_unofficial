#include <glil/odometry/odometry_estimation_imu.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glil/util/config.hpp>
#include <glil/util/convert_to_string.hpp>
#ifdef GLIL_PROFILE_TIMING
#include <glil/util/digest.hpp>
#include <glil/util/perftime.hpp>
#endif
#include <glil/common/imu_integration.hpp>
#include <glil/common/cloud_deskewing.hpp>
#include <glil/common/cloud_covariance_estimation.hpp>
#include <glil/odometry/initial_state_estimation.hpp>
#include <glil/odometry/loose_initial_state_estimation.hpp>
#include <glil/odometry/callbacks.hpp>

namespace glil {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

#ifdef GLIL_PROFILE_TIMING
namespace {
uint64_t digest_pose(const Eigen::Isometry3d& pose) {
  const Eigen::Matrix4d matrix = pose.matrix();
  return fnv1a_doubles(matrix.data(), static_cast<std::size_t>(matrix.size()));
}

uint64_t digest_vector3(const Eigen::Vector3d& v) {
  return fnv1a_doubles(v.data(), static_cast<std::size_t>(v.size()));
}

uint64_t digest_vector6(const Eigen::Matrix<double, 6, 1>& v) {
  return fnv1a_doubles(v.data(), static_cast<std::size_t>(v.size()));
}
}  // namespace
#endif

OdometryEstimationIMUParams::OdometryEstimationIMUParams() {
  // sensor config
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
  T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
  imu_bias_noise = sensor_config.param<double>("sensors", "imu_bias_noise", 1e-3);
  auto bias = sensor_config.param<std::vector<double>>("sensors", "imu_bias");
  if (bias && bias->size() == 6) {
    imu_bias = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(bias->data());
  } else {
    imu_bias.setZero();
  }

  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  fix_imu_bias = config.param<bool>("odometry_estimation", "fix_imu_bias", false);

  initialization_mode = config.param<std::string>("odometry_estimation", "initialization_mode", "LOOSE");
  const auto init_T_world_imu = config.param<Eigen::Isometry3d>("odometry_estimation", "init_T_world_imu");
  const auto init_v_world_imu = config.param<Eigen::Vector3d>("odometry_estimation", "init_v_world_imu");
  this->estimate_init_state = !init_T_world_imu && !init_v_world_imu;
  this->init_T_world_imu = init_T_world_imu.value_or(Eigen::Isometry3d::Identity());
  this->init_v_world_imu = init_v_world_imu.value_or(Eigen::Vector3d::Zero());
  this->init_pose_damping_scale = config.param<double>("odometry_estimation", "init_pose_damping_scale", 1e10);

  smoother_lag = config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  use_isam2_dogleg = config.param<bool>("odometry_estimation", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1);
  force_fallback_interval = config.param<int>("odometry_estimation", "force_fallback_interval", 0);

  save_imu_rate_trajectory = config.param<bool>("odometry_estimation", "save_imu_rate_trajectory", false);
  debug_frame_window_start = config.param<int>("odometry_estimation", "debug_frame_window_start", -1);
  debug_frame_window_end = config.param<int>("odometry_estimation", "debug_frame_window_end", -1);
  debug_stamp_window_start = config.param<double>("odometry_estimation", "debug_stamp_window_start", -1.0);
  debug_stamp_window_end = config.param<double>("odometry_estimation", "debug_stamp_window_end", -1.0);
  trace_history_size = std::max(0, config.param<int>("odometry_estimation", "trace_history_size", 0));
  trace_following_frames = std::max(0, config.param<int>("odometry_estimation", "trace_following_frames", 4));
  trace_filtered_points_threshold = config.param<int>("odometry_estimation", "trace_filtered_points_threshold", 256);
  skip_sparse_frame_threshold = config.param<int>("odometry_estimation", "skip_sparse_frame_threshold", -1);
  trace_on_empty_frame = config.param<bool>("odometry_estimation", "trace_on_empty_frame", true);
  trace_on_sparse_frame = config.param<bool>("odometry_estimation", "trace_on_sparse_frame", true);
  trace_on_imu_starvation = config.param<bool>("odometry_estimation", "trace_on_imu_starvation", true);
  trace_on_smoother_fallback = config.param<bool>("odometry_estimation", "trace_on_smoother_fallback", true);

  Config logging_config(GlobalConfig::get_config_path("config_logging"));
  debug_digest = logging_config.param<bool>("logging", "debug_digest").value_or(false);
  debug_digest = config.param<bool>("odometry_estimation", "debug_digest", debug_digest);

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  registration_num_threads = config.param<int>("odometry_estimation", "registration_num_threads", num_threads);
  covariance_num_threads = config.param<int>("odometry_estimation", "covariance_num_threads", num_threads);
  initialization_num_threads = config.param<int>("odometry_estimation", "initialization_num_threads", num_threads);
}

OdometryEstimationIMUParams::~OdometryEstimationIMUParams() {}

OdometryEstimationIMU::OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params_) : params(std::move(params_)) {
  marginalized_cursor = 0;
  T_lidar_imu.setIdentity();
  T_imu_lidar.setIdentity();

  if (!params->estimate_init_state || params->initialization_mode == "NAIVE") {
    auto init_estimation = new NaiveInitialStateEstimation(params->T_lidar_imu, params->imu_bias);
    if (!params->estimate_init_state) {
      init_estimation->set_init_state(params->init_T_world_imu, params->init_v_world_imu);
    }
    this->init_estimation.reset(init_estimation);
  } else if (params->initialization_mode == "LOOSE") {
    auto init_estimation = new LooseInitialStateEstimation(params->T_lidar_imu, params->imu_bias);
    this->init_estimation.reset(init_estimation);
  } else {
    logger->error("unknown initialization mode {}", params->initialization_mode);
  }

  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation(params->covariance_num_threads));

  gtsam::ISAM2Params isam2_params;
  if (params->use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.findUnusedFactorSlots = true;
  isam2_params.relinearizeSkip = params->isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params->isam2_relinearize_thresh);
  smoother.reset(new FixedLagSmootherExt(params->smoother_lag, isam2_params));
  smoother->set_fix_variable_types({{'x', 0}, {'v', 1}, {'b', 2}});
  smoother->set_force_fallback_interval(params->force_fallback_interval);
}

OdometryEstimationIMU::~OdometryEstimationIMU() {}

bool OdometryEstimationIMU::debug_frame_enabled(int frame_id, double stamp) const {
  bool frame_enabled = false;
  if (params->debug_frame_window_start >= 0 && frame_id >= params->debug_frame_window_start) {
    frame_enabled = params->debug_frame_window_end < params->debug_frame_window_start || frame_id <= params->debug_frame_window_end;
  }

  bool stamp_enabled = false;
  if (params->debug_stamp_window_start >= 0.0 && stamp >= params->debug_stamp_window_start) {
    stamp_enabled = params->debug_stamp_window_end < params->debug_stamp_window_start || stamp <= params->debug_stamp_window_end;
  }

  return frame_enabled || stamp_enabled;
}

bool OdometryEstimationIMU::compact_trace_enabled() const {
  return params->trace_history_size > 0;
}

std::string OdometryEstimationIMU::join_trace_reasons(const std::vector<std::string>& reasons) const {
  std::string joined;
  for (std::size_t i = 0; i < reasons.size(); i++) {
    if (i) {
      joined += ",";
    }
    joined += reasons[i];
  }
  return joined;
}

void OdometryEstimationIMU::dump_compact_trace_entry(const CompactTraceEntry& entry, const char* phase) const {
  logger->info(
    "odom-trace-entry phase={} frame={} seq={} stamp={:.6f} input_stamp={:.6f} scan_end={:.6f} dt={:.6f} scan_duration={:.6f} raw_points={} filtered_points={} num_imu={} pred_source={} last_v_norm={:.6f} pred_v_norm={:.6f} pred_rel_speed={:.6f} pred_accel={:.6f} est_v_norm={:.6f} last_lidar_pose={} predicted_lidar_pose={} estimated_lidar_pose={} fallback={}",
    phase,
    entry.frame_id,
    entry.sequence_id,
    entry.stamp,
    entry.input_stamp,
    entry.scan_end_time,
    entry.prediction_dt,
    entry.scan_duration,
    entry.raw_points,
    entry.filtered_points,
    entry.num_imu_integrated,
    entry.prediction_source,
    entry.last_velocity_norm,
    entry.predicted_velocity_norm,
    entry.predicted_relative_speed,
    entry.predicted_acceleration,
    entry.estimated_velocity_norm,
    entry.last_lidar_pose,
    entry.predicted_lidar_pose,
    entry.estimated_lidar_pose,
    entry.smoother_fallback ? "true" : "false");
}

void OdometryEstimationIMU::record_compact_trace(const CompactTraceEntry& entry) {
  if (!compact_trace_enabled()) {
    return;
  }

  compact_trace_history.push_back(entry);
  while (compact_trace_history.size() > static_cast<std::size_t>(params->trace_history_size)) {
    compact_trace_history.pop_front();
  }

  if (compact_trace_active_until >= 0 && entry.frame_id > compact_trace_last_dumped_frame && entry.frame_id <= compact_trace_active_until) {
    dump_compact_trace_entry(entry, "follow");
    compact_trace_last_dumped_frame = entry.frame_id;

    if (entry.frame_id == compact_trace_active_until) {
      logger->info("odom-trace-end last_frame={}", entry.frame_id);
      compact_trace_active_until = -1;
    }
  }
}

void OdometryEstimationIMU::trigger_compact_trace(const CompactTraceEntry& entry, const std::vector<std::string>& reasons) {
  if (!compact_trace_enabled() || reasons.empty()) {
    return;
  }

  const std::string reason_text = join_trace_reasons(reasons);
  const int new_active_until = entry.frame_id + params->trace_following_frames;

  if (entry.frame_id > compact_trace_active_until) {
    logger->info("odom-trace-begin frame={} seq={} reasons={} history={} follow={}",
                 entry.frame_id,
                 entry.sequence_id,
                 reason_text,
                 compact_trace_history.size(),
                 params->trace_following_frames);

    for (const auto& history_entry : compact_trace_history) {
      dump_compact_trace_entry(history_entry, history_entry.frame_id == entry.frame_id ? "trigger" : "history");
    }

    compact_trace_last_dumped_frame = entry.frame_id;
    compact_trace_active_until = new_active_until;
  } else {
    compact_trace_active_until = std::max(compact_trace_active_until, new_active_until);
    logger->info("odom-trace-trigger frame={} seq={} reasons={} active_until={}",
                 entry.frame_id,
                 entry.sequence_id,
                 reason_text,
                 compact_trace_active_until);

    if (entry.frame_id > compact_trace_last_dumped_frame) {
      dump_compact_trace_entry(entry, "trigger");
      compact_trace_last_dumped_frame = entry.frame_id;
    }
  }

  if (params->trace_following_frames == 0) {
    logger->info("odom-trace-end last_frame={}", entry.frame_id);
    compact_trace_active_until = -1;
  }
}

void OdometryEstimationIMU::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr OdometryEstimationIMU::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  const int current = frames.size();
#ifdef GLIL_PROFILE_TIMING
  perftime::reset();
  const auto profile_total_t0 = perftime::now();
#endif
  const bool debug_frame = debug_frame_enabled(current, raw_frame->stamp);
  const bool digest_frame =
    params->debug_digest && (debug_frame || (params->debug_frame_window_start < 0 && params->debug_stamp_window_start < 0.0));
  CompactTraceEntry trace_entry;
  trace_entry.frame_id = current;
  trace_entry.sequence_id = raw_frame->debug_sequence_id;
  trace_entry.stamp = raw_frame->stamp;
  trace_entry.input_stamp = raw_frame->debug_input_stamp;
  trace_entry.scan_end_time = raw_frame->scan_end_time;
  trace_entry.scan_duration = raw_frame->scan_end_time - raw_frame->stamp;
  trace_entry.raw_points = raw_frame->debug_raw_points;
  trace_entry.filtered_points = raw_frame->size();
  std::vector<std::string> trace_reasons;

  if (params->trace_on_sparse_frame && params->trace_filtered_points_threshold >= 0 && raw_frame->size() > 0 &&
      raw_frame->size() <= params->trace_filtered_points_threshold) {
    trace_reasons.emplace_back("sparse_frame");
  }

  if (debug_frame) {
    logger->info("frame-dbg-input frame={} seq={} stamp={:.6f} input_stamp={:.6f} scan_end={:.6f} raw_points={} filtered_points={} scan_duration={:.6f}",
                 current,
                 raw_frame->debug_sequence_id,
                 raw_frame->stamp,
                 raw_frame->debug_input_stamp,
                 raw_frame->scan_end_time,
                 raw_frame->debug_raw_points,
                 raw_frame->size(),
                 raw_frame->scan_end_time - raw_frame->stamp);
  }
  if (raw_frame->size()) {
    logger->trace("insert_frame points={} times={} ~ {}", raw_frame->size(), raw_frame->times.front(), raw_frame->times.back());
  } else {
    if (params->trace_on_empty_frame) {
      trace_reasons.emplace_back("empty_frame");
    }
    record_compact_trace(trace_entry);
    trigger_compact_trace(trace_entry, trace_reasons);
    logger->warn("insert_frame frame={} seq={} stamp={:.6f} input_stamp={:.6f} raw_points={} points={} -- skipping empty frame",
                 current,
                 raw_frame->debug_sequence_id,
                 raw_frame->stamp,
                 raw_frame->debug_input_stamp,
                 raw_frame->debug_raw_points,
                 raw_frame->size());
    return nullptr;
  }
  if (params->skip_sparse_frame_threshold >= 0 && raw_frame->size() <= params->skip_sparse_frame_threshold) {
    record_compact_trace(trace_entry);
    trigger_compact_trace(trace_entry, trace_reasons);
    logger->warn(
      "insert_frame frame={} seq={} stamp={:.6f} input_stamp={:.6f} raw_points={} points={} sparse_skip_threshold={} -- skipping sparse frame",
      current,
      raw_frame->debug_sequence_id,
      raw_frame->stamp,
      raw_frame->debug_input_stamp,
      raw_frame->debug_raw_points,
      raw_frame->size(),
      params->skip_sparse_frame_threshold);
    return nullptr;
  }
  Callbacks::on_insert_frame(raw_frame);

  const int last = current - 1;

  // The very first frame
  if (frames.empty()) {
    init_estimation->insert_frame(raw_frame);
    auto init_state = init_estimation->initial_pose();
    if (init_state == nullptr) {
      logger->debug("waiting for initial IMU state estimation to be finished");
      return nullptr;
    }
    init_estimation.reset();

    logger->info("initial IMU state estimation result");
    logger->info("T_world_imu={}", convert_to_string(init_state->T_world_imu));
    logger->info("v_world_imu={}", convert_to_string(init_state->v_world_imu));
    logger->info("imu_bias={}", convert_to_string(init_state->imu_bias));

    // Initialize the first frame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    T_lidar_imu = init_state->T_lidar_imu;
    T_imu_lidar = T_lidar_imu.inverse();

    new_frame->T_lidar_imu = init_state->T_lidar_imu;
    new_frame->T_world_lidar = init_state->T_world_lidar;
    new_frame->T_world_imu = init_state->T_world_imu;

    new_frame->v_world_imu = init_state->v_world_imu;
    new_frame->imu_bias = init_state->imu_bias;
    new_frame->raw_frame = raw_frame;

    // Transform points into IMU frame
    std::vector<Eigen::Vector4d> points_imu(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points_imu[i] = T_imu_lidar * raw_frame->points[i];
    }

    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation->estimate(points_imu, raw_frame->neighbors, normals, covs);

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points_imu);
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::IMU;
    create_frame(new_frame);

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);

    // Initialize the estimator
    gtsam::Values new_values;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

    new_stamps[X(0)] = raw_frame->stamp;
    new_stamps[V(0)] = raw_frame->stamp;
    new_stamps[B(0)] = raw_frame->stamp;

    new_values.insert(X(0), gtsam::Pose3(new_frame->T_world_imu.matrix()));
    new_values.insert(V(0), new_frame->v_world_imu);
    new_values.insert(B(0), gtsam::imuBias::ConstantBias(new_frame->imu_bias));

    // Prior for initial IMU states
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params->init_pose_damping_scale);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(B(0), 6, 1e6);
    new_factors.add(create_factors(current, nullptr, new_values));

#ifdef GLIL_PROFILE_TIMING
    const auto profile_smoother_t0 = perftime::now();
#endif
    smoother->update(new_factors, new_values, new_stamps);
#ifdef GLIL_PROFILE_TIMING
    perftime::add(perftime::Counter::Smoother, perftime::elapsed_us(profile_smoother_t0, perftime::now()));
#endif
    update_frames(current, new_factors);

    trace_entry.estimated_velocity_norm = frames.back()->v_world_imu.norm();
    record_compact_trace(trace_entry);
    trigger_compact_trace(trace_entry, trace_reasons);

#ifdef GLIL_PROFILE_TIMING
    if (digest_frame) {
      const int digest_sequence = raw_frame->debug_sequence_id >= 0 ? raw_frame->debug_sequence_id : current;
      logger->info(
        "[digest] phase=post frame={} seq={} stamp={:.9f} pose_digest={:016x} v_digest={:016x}",
        current,
        digest_sequence,
        raw_frame->stamp,
        digest_pose(frames.back()->T_world_lidar),
        digest_vector3(frames.back()->v_world_imu));
    }
#endif

#ifdef GLIL_PROFILE_TIMING
    const auto profile_snapshot = perftime::snapshot();
    const auto profile_total_us = perftime::elapsed_us(profile_total_t0, perftime::now());
    logger->info(
      "[perftime] frame={} total_us={} create_factors_us={} update_corresp_us={} extract_coreset_us={} evaluate_us={} smoother_us={} coreset_filter_us={} coreset_buildJE_us={} coreset_carath_us={} coreset_carath_calls={} coreset_rows_us={}",
      current,
      profile_total_us,
      profile_snapshot.create_factors_us,
      profile_snapshot.update_corresp_us,
      profile_snapshot.extract_coreset_us,
      profile_snapshot.evaluate_us,
      profile_snapshot.smoother_us,
      profile_snapshot.coreset_filter_us,
      profile_snapshot.coreset_buildje_us,
      profile_snapshot.coreset_carath_us,
      profile_snapshot.coreset_carath_calls,
      profile_snapshot.coreset_rows_us);
#endif

    return frames.back();
  }

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  const double last_stamp = frames[last]->stamp;
  const auto last_T_world_imu_ = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_T_world_imu = gtsam::Pose3(last_T_world_imu_.rotation().normalized(), last_T_world_imu_.translation());
  const auto last_v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(last));
  // Work around a GTSAM 4.3 / fixed-lag smoother issue where reading the bias
  // estimate can throw even though pose/velocity remain queryable.
  const auto last_imu_bias = gtsam::imuBias::ConstantBias(frames[last]->imu_bias);
  const double prediction_dt = raw_frame->stamp - last_stamp;
  const double cached_gyro_bias_norm_deg_s = last_imu_bias.gyroscope().norm() * 180.0 / M_PI;
  const double cached_accel_bias_norm = last_imu_bias.accelerometer().norm();
  const gtsam::NavState last_nav_world_imu(last_T_world_imu, last_v_world_imu);
  trace_entry.prediction_dt = prediction_dt;
  trace_entry.last_velocity_norm = last_v_world_imu.norm();
  trace_entry.last_lidar_pose = convert_to_string(Eigen::Isometry3d(last_T_world_imu.matrix()) * T_imu_lidar);

  if (debug_frame) {
    logger->info(
      "frame-dbg-pre frame={} seq={} stamp={:.6f} dt={:.6f} last_state_source=smoother_pose_velocity_cached_bias smoother_v_norm={:.6f} cached_gyro_bias_norm_deg_s={:.6f} cached_accel_bias_norm={:.6f}",
      current,
      raw_frame->debug_sequence_id,
      raw_frame->stamp,
      prediction_dt,
      last_v_world_imu.norm(),
      cached_gyro_bias_norm_deg_s,
      cached_accel_bias_norm);
  }

  // IMU integration between LiDAR scans (inter-scan)
  int num_imu_integrated = 0;
  const int imu_read_cursor = imu_integration->integrate_imu(last_stamp, raw_frame->stamp, last_imu_bias, &num_imu_integrated);
  imu_integration->erase_imu_data(imu_read_cursor);
  logger->trace("num_imu_integrated={}", num_imu_integrated);
  std::string prediction_source = "imu";

  // IMU state prediction
  const gtsam::NavState predicted_nav_world_imu = imu_integration->integrated_measurements().predict(last_nav_world_imu, last_imu_bias);
  gtsam::Pose3 predicted_T_world_imu = predicted_nav_world_imu.pose();
  gtsam::Vector3 predicted_v_world_imu = predicted_nav_world_imu.velocity();

  new_stamps[X(current)] = raw_frame->stamp;
  new_stamps[V(current)] = raw_frame->stamp;
  new_stamps[B(current)] = raw_frame->stamp;

  if (num_imu_integrated < 2 && last > 1) {
    const Eigen::Isometry3d T_delta = frames[last - 1]->T_lidar_imu.inverse() * frames[last]->T_lidar_imu;
    predicted_T_world_imu = gtsam::Pose3((frames[last]->T_world_imu * T_delta).matrix());
    predicted_v_world_imu = frames[last]->v_world_imu;
    prediction_source = "scan_delta_extrapolation";
  }

  const double predicted_velocity_norm = predicted_v_world_imu.norm();
  const double predicted_relative_speed =
    prediction_dt > 1e-6 ? (last_T_world_imu.inverse() * predicted_T_world_imu).translation().norm() / prediction_dt : std::numeric_limits<double>::quiet_NaN();
  const double predicted_acceleration =
    prediction_dt > 1e-6 ? (predicted_v_world_imu - last_v_world_imu).norm() / prediction_dt : std::numeric_limits<double>::quiet_NaN();
  trace_entry.num_imu_integrated = num_imu_integrated;
  trace_entry.prediction_source = prediction_source;
  trace_entry.predicted_velocity_norm = predicted_velocity_norm;
  trace_entry.predicted_relative_speed = predicted_relative_speed;
  trace_entry.predicted_acceleration = predicted_acceleration;
  trace_entry.predicted_lidar_pose = convert_to_string(Eigen::Isometry3d(predicted_T_world_imu.matrix()) * T_imu_lidar);

  if (debug_frame) {
    logger->info(
      "frame-dbg-pred-candidate frame={} seq={} source={} num_imu_integrated={} velocity_norm={:.6f} relative_speed={:.6f} acceleration={:.6f} eval_gyro_bias_norm_deg_s={:.6f} eval_accel_bias_norm={:.6f}",
      current,
      raw_frame->debug_sequence_id,
      prediction_source,
      num_imu_integrated,
      predicted_velocity_norm,
      predicted_relative_speed,
      predicted_acceleration,
      cached_gyro_bias_norm_deg_s,
      cached_accel_bias_norm);
    logger->info(
      "frame-dbg-pred frame={} seq={} source={} num_imu_integrated={} predicted_velocity_norm={:.6f} predicted_relative_speed={:.6f} predicted_acceleration={:.6f} prediction_gyro_bias_norm_deg_s={:.6f} prediction_accel_bias_norm={:.6f}",
      current,
      raw_frame->debug_sequence_id,
      prediction_source,
      num_imu_integrated,
      predicted_velocity_norm,
      predicted_relative_speed,
      predicted_acceleration,
      cached_gyro_bias_norm_deg_s,
      cached_accel_bias_norm);
  }

  new_values.insert(X(current), predicted_T_world_imu);
  new_values.insert(V(current), predicted_v_world_imu);
  new_values.insert(B(current), last_imu_bias);

#ifdef GLIL_PROFILE_TIMING
  if (digest_frame) {
    const int digest_sequence = raw_frame->debug_sequence_id >= 0 ? raw_frame->debug_sequence_id : current;
    logger->info(
      "[digest] phase=predict frame={} seq={} stamp={:.9f} last_pose_digest={:016x} last_v_digest={:016x} pred_pose_digest={:016x} pred_v_digest={:016x} bias_digest={:016x} prediction_source={} num_imu={}",
      current,
      digest_sequence,
      raw_frame->stamp,
      digest_pose(Eigen::Isometry3d(last_T_world_imu.matrix())),
      digest_vector3(last_v_world_imu),
      digest_pose(Eigen::Isometry3d(predicted_T_world_imu.matrix())),
      digest_vector3(predicted_v_world_imu),
      digest_vector6(last_imu_bias.vector()),
      prediction_source,
      num_imu_integrated);
  }
#endif

  // Constant IMU bias assumption
  new_factors.add(
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, params->imu_bias_noise)));
  if (params->fix_imu_bias) {
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(current), gtsam::imuBias::ConstantBias(params->imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e3)));
  }

  // Create IMU factor
  std::shared_ptr<gtsam::ImuFactor> imu_factor;
  if (num_imu_integrated >= 2) {
    imu_factor = std::make_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
    new_factors.add(imu_factor);
  } else {
    if (params->trace_on_imu_starvation) {
      trace_reasons.emplace_back("imu_starvation");
    }
    logger->warn("insufficient number of IMU data between LiDAR scans!! (odometry_estimation frame={} seq={})", current, raw_frame->debug_sequence_id);
    logger->warn("t_last={:.6f} t_current={:.6f} input_stamp={:.6f} num_imu={} raw_points={} filtered_points={}",
                 last_stamp,
                 raw_frame->stamp,
                 raw_frame->debug_input_stamp,
                 num_imu_integrated,
                 raw_frame->debug_raw_points,
                 raw_frame->size());
    new_factors.add(gtsam::BetweenFactor<gtsam::Vector3>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
  }

  // Motion prediction for deskewing (intra-scan)
  std::vector<double> pred_imu_times;
  std::vector<Eigen::Isometry3d> pred_imu_poses;
  imu_integration->integrate_imu(raw_frame->stamp, raw_frame->scan_end_time, predicted_nav_world_imu, last_imu_bias, pred_imu_times, pred_imu_poses);

  // Create EstimationFrame
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;

  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->T_world_imu = Eigen::Isometry3d(predicted_T_world_imu.matrix());
  new_frame->T_world_lidar = Eigen::Isometry3d(predicted_T_world_imu.matrix()) * T_imu_lidar;
  new_frame->v_world_imu = predicted_v_world_imu;
  new_frame->imu_bias = last_imu_bias.vector();
  new_frame->raw_frame = raw_frame;

  if (params->save_imu_rate_trajectory) {
    new_frame->imu_rate_trajectory.resize(8, pred_imu_times.size());

    for (int i = 0; i < pred_imu_times.size(); i++) {
      const Eigen::Vector3d trans = pred_imu_poses[i].translation();
      const Eigen::Quaterniond quat(pred_imu_poses[i].linear());
      new_frame->imu_rate_trajectory.col(i) << pred_imu_times[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
    }
  }

  // Deskew and tranform points into IMU frame
  auto deskewed = deskewing->deskew(T_imu_lidar, pred_imu_times, pred_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  for (auto& pt : deskewed) {
    pt = T_imu_lidar * pt;
  }

  std::vector<Eigen::Vector4d> deskewed_normals;
  std::vector<Eigen::Matrix4d> deskewed_covs;
  covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);

  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
  frame->add_covs(deskewed_covs);
  frame->add_normals(deskewed_normals);
  new_frame->frame = frame;
  new_frame->frame_id = FrameID::IMU;
  create_frame(new_frame);

  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);

  new_factors.add(create_factors(current, imu_factor, new_values));

  // Update smoother
#ifdef GLIL_PROFILE_TIMING
  const auto profile_smoother_t0 = perftime::now();
#endif
  Callbacks::on_smoother_update(*smoother, new_factors, new_values, new_stamps);
  smoother->update(new_factors, new_values, new_stamps);
  smoother->update();
  Callbacks::on_smoother_update_finish(*smoother);
#ifdef GLIL_PROFILE_TIMING
  perftime::add(perftime::Counter::Smoother, perftime::elapsed_us(profile_smoother_t0, perftime::now()));
#endif

  // Find out marginalized frames
  while (marginalized_cursor < current) {
    double span = frames[current]->stamp - frames[marginalized_cursor]->stamp;
    if (span < params->smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }
  Callbacks::on_marginalized_frames(marginalized_frames);

  // Update frames
  update_frames(current, new_factors);

#ifdef GLIL_PROFILE_TIMING
  const auto profile_snapshot = perftime::snapshot();
  const auto profile_total_us = perftime::elapsed_us(profile_total_t0, perftime::now());
  logger->info(
    "[perftime] frame={} total_us={} create_factors_us={} update_corresp_us={} extract_coreset_us={} evaluate_us={} smoother_us={} coreset_filter_us={} coreset_buildJE_us={} coreset_carath_us={} coreset_carath_calls={} coreset_rows_us={}",
    current,
    profile_total_us,
    profile_snapshot.create_factors_us,
    profile_snapshot.update_corresp_us,
    profile_snapshot.extract_coreset_us,
    profile_snapshot.evaluate_us,
    profile_snapshot.smoother_us,
    profile_snapshot.coreset_filter_us,
    profile_snapshot.coreset_buildje_us,
    profile_snapshot.coreset_carath_us,
    profile_snapshot.coreset_carath_calls,
    profile_snapshot.coreset_rows_us);
#endif

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.begin() + marginalized_cursor, frames.end());
  Callbacks::on_update_frames(active_frames);
  logger->trace("frames updated");

  trace_entry.smoother_fallback = smoother->fallbackHappened();
  trace_entry.estimated_velocity_norm = frames[current]->v_world_imu.norm();
  trace_entry.estimated_lidar_pose = convert_to_string(frames[current]->T_world_lidar);
  if (trace_entry.smoother_fallback && params->trace_on_smoother_fallback) {
    trace_reasons.emplace_back("smoother_fallback");
  }
  record_compact_trace(trace_entry);
  trigger_compact_trace(trace_entry, trace_reasons);

#ifdef GLIL_PROFILE_TIMING
  if (digest_frame) {
    const int digest_sequence = raw_frame->debug_sequence_id >= 0 ? raw_frame->debug_sequence_id : current;
    logger->info(
      "[digest] phase=post frame={} seq={} stamp={:.9f} pose_digest={:016x} v_digest={:016x}",
      current,
      digest_sequence,
      raw_frame->stamp,
      digest_pose(frames[current]->T_world_lidar),
      digest_vector3(frames[current]->v_world_imu));
  }
#endif

  if (debug_frame) {
    logger->info("frame-dbg-post frame={} seq={} fallback_happened={} estimated_v_norm={:.6f}",
                 current,
                 raw_frame->debug_sequence_id,
                 trace_entry.smoother_fallback,
                 trace_entry.estimated_velocity_norm);
  }

  if (smoother->fallbackHappened()) {
    logger->warn("odometry estimation smoother fallback happened (time={})", raw_frame->stamp);
  }

  return frames[current];
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationIMU::get_remaining_frames() {
  // Perform a few optimization iterations at the end
  // for(int i=0; i<5; i++) {
  //   smoother->update();
  // }
  // OdometryEstimationIMU::update_frames(frames.size() - 1, gtsam::NonlinearFactorGraph());

  std::vector<EstimationFrame::ConstPtr> marginalized_frames;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    marginalized_frames.push_back(frames[i]);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);

  return marginalized_frames;
}

void OdometryEstimationIMU::update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) {
  logger->trace("update frames current={} marginalized_cursor={}", current, marginalized_cursor);

#ifdef GLIL_PROFILE_TIMING
  const bool digest_frame =
    params->debug_digest && current >= 0 && current < static_cast<int>(frames.size()) && frames[current] &&
    (debug_frame_enabled(current, frames[current]->stamp) || (params->debug_frame_window_start < 0 && params->debug_stamp_window_start < 0.0));
#endif

  for (int i = marginalized_cursor; i < frames.size(); i++) {
    try {
      Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
      Eigen::Vector3d v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(i));
      frames[i]->T_world_imu = T_world_imu;
      frames[i]->T_world_lidar = T_world_imu * T_imu_lidar;
      frames[i]->v_world_imu = v_world_imu;
#ifdef GLIL_PROFILE_TIMING
      if (digest_frame) {
        logger->info(
          "[digest] phase=update-active current={} frame={} pose_digest={:016x} v_digest={:016x}",
          current,
          i,
          digest_pose(frames[i]->T_world_lidar),
          digest_vector3(frames[i]->v_world_imu));
      }
#endif
    } catch (std::out_of_range& e) {
      logger->error("caught {}", e.what());
      logger->error("current={}", current);
      logger->error("marginalized_cursor={}", marginalized_cursor);
      Callbacks::on_smoother_corruption(frames[current]->stamp);
      fallback_smoother();
      break;
    }
  }
}

}  // namespace glil
