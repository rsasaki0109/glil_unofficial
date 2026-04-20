#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <glil/odometry/odometry_estimation_base.hpp>


namespace gtsam {
class Pose3;
class Values;
class ImuFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glil {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;
class InitialStateEstimation;

/**
 * @brief Parameters for OdometryEstimationIMU
 */
struct OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMUParams();
  virtual ~OdometryEstimationIMUParams();

public:
  // Sensor params;
  bool fix_imu_bias;
  double imu_bias_noise;
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  // Init state
  std::string initialization_mode;
  bool estimate_init_state;
  Eigen::Isometry3d init_T_world_imu;
  Eigen::Vector3d init_v_world_imu;
  double init_pose_damping_scale;

  // Optimization params
  double smoother_lag;
  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;
  int force_fallback_interval;

  // Logging params
  bool save_imu_rate_trajectory;
  int debug_frame_window_start;
  int debug_frame_window_end;
  double debug_stamp_window_start;
  double debug_stamp_window_end;
  int trace_history_size;
  int trace_following_frames;
  int trace_filtered_points_threshold;
  bool trace_on_empty_frame;
  bool trace_on_sparse_frame;
  bool trace_on_imu_starvation;
  bool trace_on_smoother_fallback;
  bool debug_digest;

  int num_threads;
  int registration_num_threads;
  int covariance_num_threads;
  int initialization_num_threads;
};

/**
 * @brief Base class for LiDAR-IMU odometry estimation
 */
class OdometryEstimationIMU : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params);
  virtual ~OdometryEstimationIMU() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

protected:
  virtual void create_frame(EstimationFrame::Ptr& frame) {}
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const std::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) = 0;

  virtual void fallback_smoother() {}
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors);
  bool debug_frame_enabled(int frame_id, double stamp) const;
  bool compact_trace_enabled() const;

  struct CompactTraceEntry {
    int frame_id = -1;
    int sequence_id = -1;
    double stamp = std::numeric_limits<double>::quiet_NaN();
    double input_stamp = std::numeric_limits<double>::quiet_NaN();
    double scan_end_time = std::numeric_limits<double>::quiet_NaN();
    double prediction_dt = std::numeric_limits<double>::quiet_NaN();
    double scan_duration = std::numeric_limits<double>::quiet_NaN();
    int raw_points = 0;
    int filtered_points = 0;
    int num_imu_integrated = -1;
    std::string prediction_source = "n/a";
    double last_velocity_norm = std::numeric_limits<double>::quiet_NaN();
    double predicted_velocity_norm = std::numeric_limits<double>::quiet_NaN();
    double predicted_relative_speed = std::numeric_limits<double>::quiet_NaN();
    double predicted_acceleration = std::numeric_limits<double>::quiet_NaN();
    double estimated_velocity_norm = std::numeric_limits<double>::quiet_NaN();
    std::string last_lidar_pose = "n/a";
    std::string predicted_lidar_pose = "n/a";
    std::string estimated_lidar_pose = "n/a";
    bool smoother_fallback = false;
  };

  void record_compact_trace(const CompactTraceEntry& entry);
  void trigger_compact_trace(const CompactTraceEntry& entry, const std::vector<std::string>& reasons);
  void dump_compact_trace_entry(const CompactTraceEntry& entry, const char* phase) const;
  std::string join_trace_reasons(const std::vector<std::string>& reasons) const;

protected:
  std::unique_ptr<OdometryEstimationIMUParams> params;

  // Sensor extrinsic params
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // Frames & keyframes
  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;
  std::deque<CompactTraceEntry> compact_trace_history;
  int compact_trace_active_until = -1;
  int compact_trace_last_dumped_frame = -1;
};

}  // namespace glil
