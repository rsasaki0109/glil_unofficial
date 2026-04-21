#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <glil/util/raw_points.hpp>

namespace glil {

/**
 * @brief Preprocessed point cloud
 *
 */
struct PreprocessedFrame {
public:
  using Ptr = std::shared_ptr<PreprocessedFrame>;
  using ConstPtr = std::shared_ptr<const PreprocessedFrame>;

  /**
   * @brief Number of points
   * @return Number of points
   */
  int size() const { return points.size(); }

public:
  double stamp;          // Timestamp at the beginning of the scan
  double scan_end_time;  // Timestamp at the end of the scan

  int debug_sequence_id = -1;    // Sequence id assigned at ROS ingress
  int debug_raw_points = 0;      // Number of raw points before preprocessing
  double debug_input_stamp = -1; // Timestamp after ROS offset and before TimeKeeper correction

  std::vector<double> times;            // Point timestamps w.r.t. the first pt
  std::vector<double> intensities;      // Point intensities
  std::vector<Eigen::Vector4d> points;  // Points (homogeneous coordinates)

  int k_neighbors;             // Number of neighbors of each point
  std::vector<int> neighbors;  // k-nearest neighbors of each point

  RawPoints::ConstPtr raw_points;
};

}  // namespace glil
