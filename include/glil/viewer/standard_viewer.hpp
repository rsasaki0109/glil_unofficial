#pragma once

#include <cstddef>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glil/util/extension_module.hpp>

namespace spdlog {
class logger;
}

namespace glil {

class TrajectoryManager;
struct EstimationFrame;

class StandardViewer : public ExtensionModule {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StandardViewer();
  virtual ~StandardViewer();

  virtual bool ok() const override;

  void invoke(const std::function<void()>& task);

private:
  struct GraphOptimizationUpdate {
    std::vector<Eigen::Vector3f> submap_positions;
    std::vector<unsigned int> factor_indices;
    std::vector<Eigen::Vector3f> delta_lines;
    int moved_submaps = 0;
    float max_translation_delta = 0.0f;
    float max_rotation_delta_deg = 0.0f;
  };

  Eigen::Isometry3f resolve_pose(const std::shared_ptr<const EstimationFrame>& frame);
  static float rotation_delta_deg(const Eigen::Isometry3f& before, const Eigen::Isometry3f& after);
  std::vector<unsigned int> collect_global_factor_indices(size_t num_submaps) const;
  GraphOptimizationUpdate create_graph_optimization_update(const std::vector<int>& submap_ids, const std::vector<Eigen::Isometry3f>& submap_poses);
  void update_submap_drawables(const std::vector<int>& submap_ids, const std::vector<Eigen::Isometry3f>& submap_poses);
  void update_graph_optimization_drawables(const GraphOptimizationUpdate& update);
  void update_graph_optimization_status(const GraphOptimizationUpdate& update);

  void set_callbacks();
  void viewer_loop();

  bool drawable_filter(const std::string& name);
  void drawable_selection();

private:
  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  bool enable_partial_rendering;
  int partial_rendering_budget;

  bool track;
  bool show_current_coord;
  bool show_current_points;
  int current_color_mode;

  bool show_odometry_scans;
  bool show_odometry_keyframes;
  bool show_submaps;
  bool show_factors;
  bool show_graph_optimization_deltas;

  bool show_odometry_status;
  bool show_graph_optimization_status;
  int last_id;
  int last_num_points;
  std::pair<double, double> last_point_stamps;
  Eigen::Vector3d last_imu_vel;
  Eigen::Matrix<double, 6, 1> last_imu_bias;
  int last_graph_update_num_values;
  int last_graph_update_num_factors;
  int last_graph_moved_submaps;
  float last_graph_max_translation_delta;
  float last_graph_max_rotation_delta_deg;
  float graph_optimization_delta_min_trans;

  Eigen::Vector2f z_range;
  Eigen::Vector2f auto_z_range;

  std::unique_ptr<TrajectoryManager> trajectory;
  std::vector<Eigen::Isometry3f> submap_keyframes;
  std::vector<Eigen::Isometry3f> previous_submap_poses;

  std::vector<std::pair<int, int>> global_between_factors;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
  
  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}
