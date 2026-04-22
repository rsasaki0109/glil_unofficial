#include <glil/mapping/async_sub_mapping.hpp>

#include <glil/util/config.hpp>

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
}  // namespace

AsyncSubMapping::AsyncSubMapping(const std::shared_ptr<glil::SubMappingBase>& sub_mapping)
: AsyncSubMapping(sub_mapping, deterministic_async_mapping_enabled()) {}

AsyncSubMapping::AsyncSubMapping(const std::shared_ptr<glil::SubMappingBase>& sub_mapping, bool deterministic_batching)
: deterministic_batching(deterministic_batching),
  sub_mapping(sub_mapping) {
  kill_switch = false;
  end_of_sequence = false;
  internal_frame_queue_size = 0;
  max_frame_queue_size = 0;
  max_batch_frame_count = 0;
  current_output_submap_queue_size = 0;
  max_output_submap_queue_size = 0;
  latest_input_frame_stamp = 0.0;
  latest_processed_frame_stamp = 0.0;
  max_frame_lag_sec = 0.0;
  thread = std::thread([this] { run(); });
}

AsyncSubMapping::~AsyncSubMapping() {
  kill_switch = true;
  join();
}

void AsyncSubMapping::insert_image(const double stamp, const cv::Mat& image) {
  input_image_queue.push_back(std::make_pair(stamp, image));
}

void AsyncSubMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncSubMapping::insert_frame(const EstimationFrame::ConstPtr& odom_frame) {
  input_frame_queue.push_back(odom_frame);
  latest_input_frame_stamp.store(odom_frame->stamp);
  update_max(max_frame_queue_size, workload());
}

void AsyncSubMapping::join() {
  end_of_sequence = true;
  input_image_queue.submit_end_of_data();
  input_imu_queue.submit_end_of_data();
  input_frame_queue.submit_end_of_data();
  if (thread.joinable()) {
    thread.join();
  }
}

int AsyncSubMapping::workload() const {
  return input_frame_queue.size() + internal_frame_queue_size.load();
}

AsyncSubMappingStats AsyncSubMapping::stats() const {
  AsyncSubMappingStats stats;
  stats.workload = workload();
  stats.max_workload = max_frame_queue_size.load();
  stats.max_batch_size = max_batch_frame_count.load();
  stats.output_submap_queue_size = current_output_submap_queue_size.load();
  stats.max_output_submap_queue_size = max_output_submap_queue_size.load();
  const double latest_input = latest_input_frame_stamp.load();
  const double latest_processed = latest_processed_frame_stamp.load();
  stats.current_frame_lag_sec = latest_processed > 0.0 ? std::max(0.0, latest_input - latest_processed) : 0.0;
  stats.max_frame_lag_sec = max_frame_lag_sec.load();
  return stats;
}

std::vector<SubMap::Ptr> AsyncSubMapping::get_results() {
  return output_submap_queue.get_all_and_clear();
}

void AsyncSubMapping::run() {
  while (!kill_switch) {
    auto submaps = sub_mapping->get_submaps();
    output_submap_queue.insert(submaps);
    current_output_submap_queue_size.store(output_submap_queue.size());
    update_max(max_output_submap_queue_size, current_output_submap_queue_size.load());

    std::vector<std::pair<double, cv::Mat>> images;
    std::vector<Eigen::Matrix<double, 7, 1>> imu_frames;
    std::vector<EstimationFrame::ConstPtr> odom_frames;

    if (deterministic_batching) {
      auto odom_frame = input_frame_queue.pop_wait();
      if (odom_frame) {
        odom_frames.push_back(*odom_frame);
      }
      images = input_image_queue.get_all_and_clear();
      imu_frames = input_imu_queue.get_all_and_clear();
    } else {
      images = input_image_queue.get_all_and_clear();
      imu_frames = input_imu_queue.get_all_and_clear();
      odom_frames = input_frame_queue.get_all_and_clear();
    }

    internal_frame_queue_size.store(odom_frames.size());
    update_max(max_batch_frame_count, odom_frames.size());
    update_max(max_frame_queue_size, workload());

    if (images.empty() && imu_frames.empty() && odom_frames.empty()) {
      if (end_of_sequence) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    for (const auto& imu : imu_frames) {
      const double stamp = imu[0];
      const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
      const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
      sub_mapping->insert_imu(stamp, linear_acc, angular_vel);
    }

    for (const auto& image : images) {
      sub_mapping->insert_image(image.first, image.second);
    }

    for (int i = 0; i < static_cast<int>(odom_frames.size()); i++) {
      const auto& frame = odom_frames[i];
      sub_mapping->insert_frame(frame);
      latest_processed_frame_stamp.store(frame->stamp);
      internal_frame_queue_size.store(static_cast<int>(odom_frames.size()) - i - 1);
      const double lag = std::max(0.0, latest_input_frame_stamp.load() - frame->stamp);
      update_max(max_frame_lag_sec, lag);
    }
  }

  output_submap_queue.insert(sub_mapping->submit_end_of_sequence());
  current_output_submap_queue_size.store(output_submap_queue.size());
  update_max(max_output_submap_queue_size, current_output_submap_queue_size.load());
}
}  // namespace glil
