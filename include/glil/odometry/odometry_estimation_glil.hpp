#pragma once

#include <glil/odometry/odometry_estimation_imu.hpp>
#include <glil/factors/integrated_coreset_factor.hpp>

#include <string>

namespace gtsam_points {
class GaussianVoxelMapCPU;
class FastOccupancyGrid;
}  // namespace gtsam_points

namespace glil {

/**
 * @brief Parameters for OdometryEstimationGLIL
 */
struct OdometryEstimationGLILParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationGLILParams();
  virtual ~OdometryEstimationGLILParams();

  enum class KeyframeUpdateStrategy { OVERLAP, DISPLACEMENT };

public:
  // Registration params
  double voxel_resolution;
  int voxelmap_levels;
  double voxelmap_scaling_factor;

  int max_num_keyframes;
  int full_connection_window_size;

  // Keyframe management params
  KeyframeUpdateStrategy keyframe_strategy;
  double keyframe_min_overlap;
  double keyframe_max_overlap;
  double keyframe_delta_trans;
  double keyframe_delta_rot;

  // Fast occupancy grid resolution for overlap computation
  double occupancy_grid_resolution;

  // Coreset params
  IntegratedCoresetFactor::Params coreset_params;
  std::string coreset_method;
  double correspondence_sample_ratio;
  bool coreset_factor_lock;
  bool coreset_immutable_snapshot;
};

/**
 * @brief CPU-based tightly coupled LiDAR-IMU odometry with coreset downsampling
 *
 * Architecture follows OdometryEstimationGPU (keyframe-based sliding window),
 * but uses CPU voxelmaps + FastOccupancyGrid + coreset-wrapped factors.
 * Reference: Koide et al., ICRA 2025
 */
class OdometryEstimationGLIL : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationGLIL(const OdometryEstimationGLILParams& params = OdometryEstimationGLILParams());
  virtual ~OdometryEstimationGLIL() override;

private:
  virtual void create_frame(EstimationFrame::Ptr& frame) override;
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const std::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) override;

  void update_keyframes_overlap(int current);
  void update_keyframes_displacement(int current);

  double calc_overlap(
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
    const gtsam_points::PointCloud::ConstPtr& source,
    const Eigen::Isometry3d& delta) const;

  double calc_overlap_multi(
    const std::vector<gtsam_points::GaussianVoxelMap::ConstPtr>& target_voxelmaps,
    const gtsam_points::PointCloud::ConstPtr& source,
    const std::vector<Eigen::Isometry3d>& deltas) const;

private:
  std::vector<EstimationFrame::ConstPtr> keyframes;
};

}  // namespace glil
