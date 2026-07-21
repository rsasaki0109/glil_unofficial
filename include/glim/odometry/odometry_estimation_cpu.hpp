#pragma once

#include <glim/odometry/odometry_estimation_imu.hpp>

namespace gtsam_points {

class GaussianVoxelMapCPU;
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Parameters for OdometryEstimationCPU
 */
struct OdometryEstimationCPUParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  virtual ~OdometryEstimationCPUParams();

public:
  // Registration params
  std::string registration_type;    ///< Registration type (GICP or VGICP)
  int max_iterations;               ///< Maximum number of iterations
  int lru_thresh;                   ///< LRU cache threshold
  double target_downsampling_rate;  ///< Downsampling rate for points to be inserted into the target

  double ivox_resolution;  ///< iVox resolution (for GICP)
  double ivox_min_dist;    ///< Minimum distance between points in an iVox cell (for GICP)

  double vgicp_resolution;               ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;             ///< Multi-resolution voxelmap levels (for VGICP)
  double vgicp_voxelmap_scaling_factor;  ///< Multi-resolution voxelmap scaling factor (for VGICP)

  bool use_gicp_coreset;                     ///< Use exact-coreset GICP linearization (ICRA2025, arXiv:2505.01017) (for GICP)
  int coreset_size;                          ///< Target size of the exact coreset (for GICP, if use_gicp_coreset)
  double coreset_reuse_tolerance_trans;      ///< Coreset reuse translation tolerance [m] (for GICP, if use_gicp_coreset)
  double coreset_reuse_tolerance_rot;        ///< Coreset reuse rotation tolerance [rad] (for GICP, if use_gicp_coreset)
  bool use_tightly_coupled_coreset;          ///< Insert binary GICP factors directly into the fixed-lag graph
  int full_connection_window_size;           ///< Number of preceding frames connected to the latest frame
};

/**
 * @brief CPU-based semi-tightly coupled LiDAR-IMU odometry
 */
class OdometryEstimationCPU : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

private:
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;

  virtual void fallback_smoother() override;

  void update_target(const int current, const Eigen::Isometry3d& T_target_imu);

private:
  // Registration params
  std::mt19937 mt;                                                                   ///< RNG
  Eigen::Isometry3d last_T_target_imu;                                               ///< Last IMU pose w.r.t. target model
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> target_voxelmaps;  ///< VGICP target voxelmap
  std::shared_ptr<gtsam_points::iVox> target_ivox;                                   ///< GICP target iVox
  EstimationFrame::ConstPtr target_ivox_frame;                                       ///< Target points (just for visualization)
};

}  // namespace glim
