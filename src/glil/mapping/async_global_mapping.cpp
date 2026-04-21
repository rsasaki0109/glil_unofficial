#include <glil/mapping/async_global_mapping.hpp>

#include <spdlog/spdlog.h>

#include <glil/util/logging.hpp>
#include <glil/util/config.hpp>
#include <glil/mapping/callbacks.hpp>

namespace glil {

namespace {
bool deterministic_async_mapping_enabled() {
  if (!GlobalConfig::inst) {
    return false;
  }

  Config config(GlobalConfig::get_config_path("config_ros"));
  return config.param<bool>("glil_ros", "deterministic_async_mapping", false);
}

void update_max(std::atomic_int& target, int value) {
  int current = target.load();
  while (current < value && !target.compare_exchange_weak(current, value)) {
  }
}

void update_max(std::atomic<double>& target, double value) {
  double current = target.load();
  while (current < value && !target.compare_exchange_weak(current, value)) {
  }
}

double submap_stamp(const SubMap::Ptr& submap) {
  if (!submap || submap->odom_frames.empty()) {
    return 0.0;
  }
  return submap->odom_frames.back()->stamp;
}
}  // namespace

AsyncGlobalMapping::AsyncGlobalMapping(const std::shared_ptr<glil::GlobalMappingBase>& global_mapping, const int optimization_interval)
: AsyncGlobalMapping(global_mapping, optimization_interval, deterministic_async_mapping_enabled()) {}

AsyncGlobalMapping::AsyncGlobalMapping(
  const std::shared_ptr<glil::GlobalMappingBase>& global_mapping,
  const int optimization_interval,
  bool deterministic_batching)
: optimization_interval(optimization_interval),
  deterministic_batching(deterministic_batching),
  global_mapping(global_mapping),
  logger(create_module_logger("global")) {
  request_to_optimize = false;
  request_to_find_overlapping_submaps.store(-1.0);
  internal_submap_queue_size = 0;
  max_submap_queue_size = 0;
  max_batch_submap_count = 0;
  latest_input_submap_stamp = 0.0;
  latest_processed_submap_stamp = 0.0;
  max_submap_lag_sec = 0.0;

  GlobalMappingCallbacks::request_to_optimize.add([this] { request_to_optimize = true; });
  GlobalMappingCallbacks::request_to_find_overlapping_submaps.add([this](double min_overlap) { request_to_find_overlapping_submaps.store(min_overlap); });

  kill_switch = false;
  end_of_sequence = false;
  thread = std::thread([this] { run(); });
}

AsyncGlobalMapping::~AsyncGlobalMapping() {
  kill_switch = true;
  join();
}

void AsyncGlobalMapping::insert_image(const double stamp, const cv::Mat& image) {
  input_image_queue.push_back(std::make_pair(stamp, image));
}

void AsyncGlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncGlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  input_submap_queue.push_back(submap);
  latest_input_submap_stamp.store(submap_stamp(submap));
  update_max(max_submap_queue_size, workload());
}

void AsyncGlobalMapping::join() {
  end_of_sequence = true;
  input_image_queue.submit_end_of_data();
  input_imu_queue.submit_end_of_data();
  input_submap_queue.submit_end_of_data();
  if (thread.joinable()) {
    thread.join();
  }

  // for (int i = 0; i < 64; i++) {
  //   global_mapping->optimize();
  // }
}

int AsyncGlobalMapping::workload() const {
  return input_submap_queue.size() + internal_submap_queue_size.load();
}

AsyncGlobalMappingStats AsyncGlobalMapping::stats() const {
  AsyncGlobalMappingStats stats;
  stats.workload = workload();
  stats.max_workload = max_submap_queue_size.load();
  stats.max_batch_size = max_batch_submap_count.load();
  const double latest_input = latest_input_submap_stamp.load();
  const double latest_processed = latest_processed_submap_stamp.load();
  stats.current_submap_lag_sec = latest_processed > 0.0 ? std::max(0.0, latest_input - latest_processed) : 0.0;
  stats.max_submap_lag_sec = max_submap_lag_sec.load();
  return stats;
}

void AsyncGlobalMapping::save(const std::string& path) {
  logger->info("saving to {}...", path);
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  global_mapping->save(path);
  logger->info("saved");
}

std::vector<Eigen::Vector4d> AsyncGlobalMapping::export_points() {
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  logger->info("exporting points");
  auto points = global_mapping->export_points();
  return points;
}

void AsyncGlobalMapping::run() {
  auto last_optimization_time = std::chrono::high_resolution_clock::now();

  while (!kill_switch) {
    std::vector<std::pair<double, cv::Mat>> images;
    std::vector<Eigen::Matrix<double, 7, 1>> imu_frames;
    std::vector<SubMap::Ptr> submaps;

    if (deterministic_batching) {
      auto submap = input_submap_queue.pop_wait();
      if (submap) {
        submaps.push_back(*submap);
      }
      images = input_image_queue.get_all_and_clear();
      imu_frames = input_imu_queue.get_all_and_clear();
    } else {
      images = input_image_queue.get_all_and_clear();
      imu_frames = input_imu_queue.get_all_and_clear();
      submaps = input_submap_queue.get_all_and_clear();
    }

    internal_submap_queue_size.store(submaps.size());
    update_max(max_batch_submap_count, submaps.size());
    update_max(max_submap_queue_size, workload());

    if (images.empty() && imu_frames.empty() && submaps.empty()) {
      if (end_of_sequence) {
        break;
      }

      const double min_overlap = request_to_find_overlapping_submaps.exchange(-1.0);
      if (min_overlap > 0.0) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        global_mapping->find_overlapping_submaps(min_overlap);
      }

      if (request_to_optimize || std::chrono::high_resolution_clock::now() - last_optimization_time > std::chrono::seconds(optimization_interval)) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        request_to_optimize = false;
        global_mapping->optimize();
        last_optimization_time = std::chrono::high_resolution_clock::now();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    std::lock_guard<std::mutex> lock(global_mapping_mutex);
    for (const auto& imu : imu_frames) {
      const double stamp = imu[0];
      const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
      const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
      global_mapping->insert_imu(stamp, linear_acc, angular_vel);
    }

    for (const auto& image : images) {
      global_mapping->insert_image(image.first, image.second);
    }

    for (int i = 0; i < static_cast<int>(submaps.size()); i++) {
      const auto& submap = submaps[i];
      global_mapping->insert_submap(submap);
      const double stamp = submap_stamp(submap);
      latest_processed_submap_stamp.store(stamp);
      internal_submap_queue_size.store(static_cast<int>(submaps.size()) - i - 1);
      const double lag = std::max(0.0, latest_input_submap_stamp.load() - stamp);
      update_max(max_submap_lag_sec, lag);
    }

    last_optimization_time = std::chrono::high_resolution_clock::now();
  }
}

}  // namespace glil
