#pragma once

#include <atomic>
#include <thread>
#include <glil/mapping/sub_mapping.hpp>
#include <glil/util/concurrent_vector.hpp>

namespace glil {

struct AsyncSubMappingStats {
  int workload = 0;
  int max_workload = 0;
  int max_batch_size = 0;
  int output_submap_queue_size = 0;
  int max_output_submap_queue_size = 0;
  double current_frame_lag_sec = 0.0;
  double max_frame_lag_sec = 0.0;
};

/**
 * @brief SubMapping executor to wrap and asynchronously run a sub mapping object
 * @note  All the exposed public methods are thread-safe
 */
class AsyncSubMapping {
public:
  /**
   * @brief Construct a new Async Sub Mapping object
   * @param sub_mapping sub mapping object
   */
  AsyncSubMapping(const std::shared_ptr<glil::SubMappingBase>& sub_mapping);

  /**
   * @brief Destroy the Async Sub Mapping object
   *
   */
  ~AsyncSubMapping();

  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  void insert_image(const double stamp, const cv::Mat& image);

  /**
   * @brief Insert an IMU data
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert an odometry estimation frame
   * @param odom_frame  Marginalized odometry estimation frame
   */
  void insert_frame(const EstimationFrame::ConstPtr& odom_frame);

  /**
   * @brief Wait for the sub mapping thread
   *
   */
  void join();

  /**
   * @brief Number of data in the input queue (for load control)
   * @return Input queue size
   */
  int workload() const;

  /**
   * @brief Snapshot async workload / lag stats
   */
  AsyncSubMappingStats stats() const;

  /**
   * @brief Get the created submaps
   * @return Created submaps
   */
  std::vector<SubMap::Ptr> get_results();

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;

  ConcurrentVector<SubMap::Ptr> output_submap_queue;

  std::atomic_int internal_frame_queue_size;
  std::atomic_int max_frame_queue_size;
  std::atomic_int max_batch_frame_count;
  std::atomic_int current_output_submap_queue_size;
  std::atomic_int max_output_submap_queue_size;
  std::atomic<double> latest_input_frame_stamp;
  std::atomic<double> latest_processed_frame_stamp;
  std::atomic<double> max_frame_lag_sec;

  std::shared_ptr<glil::SubMappingBase> sub_mapping;
};

}  // namespace  glil
