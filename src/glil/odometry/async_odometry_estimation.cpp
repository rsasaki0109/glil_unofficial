#include <glil/odometry/async_odometry_estimation.hpp>

#include <spdlog/spdlog.h>
#include <glil/util/config.hpp>
#include <glil/util/logging.hpp>

namespace glil {

AsyncOdometryEstimation::AsyncOdometryEstimation(const std::shared_ptr<OdometryEstimationBase>& odometry_estimation, bool enable_imu)
: odometry_estimation(odometry_estimation),
  logger(create_module_logger("odom")) {
  Config config(GlobalConfig::get_config_path("config_odometry"));
  this->enable_imu = enable_imu;
  debug_stamp_window_start = config.param<double>("odometry_estimation", "debug_stamp_window_start", -1.0);
  debug_stamp_window_end = config.param<double>("odometry_estimation", "debug_stamp_window_end", -1.0);
  kill_switch = false;
  end_of_sequence = false;
  internal_frame_queue_size = 0;
  thread = std::thread([this] { run(); });
}

AsyncOdometryEstimation::~AsyncOdometryEstimation() {
  kill_switch = true;
  join();
}

void AsyncOdometryEstimation::insert_image(const double stamp, const cv::Mat& image) {
  input_image_queue.push_back(std::make_pair(stamp, image));
}

void AsyncOdometryEstimation::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncOdometryEstimation::insert_twist(const double stamp, const double linear_vel) {
  Eigen::Matrix<double, 2, 1> twist;
  twist << stamp, linear_vel;
  input_twist_queue.push_back(twist);
}

void AsyncOdometryEstimation::insert_frame(const PreprocessedFrame::Ptr& frame) {
  input_frame_queue.push_back(frame);
  if (debug_frame_enabled(frame)) {
    logger->info("odom-async-enqueue seq={} stamp={:.6f} input_stamp={:.6f} scan_end={:.6f} raw_points={} points={} queue_after={}",
                 frame->debug_sequence_id,
                 frame->stamp,
                 frame->debug_input_stamp,
                 frame->scan_end_time,
                 frame->debug_raw_points,
                 frame->size(),
                 input_frame_queue.size());
  }
}

void AsyncOdometryEstimation::join() {
  end_of_sequence = true;
  if (thread.joinable()) {
    thread.join();
  }
}

int AsyncOdometryEstimation::workload() const {
  return input_frame_queue.size() + internal_frame_queue_size;
}

void AsyncOdometryEstimation::get_results(std::vector<EstimationFrame::ConstPtr>& estimation_results, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  estimation_results = output_estimation_results.get_all_and_clear();
  marginalized_frames = output_marginalized_frames.get_all_and_clear();
}

bool AsyncOdometryEstimation::debug_frame_enabled(const PreprocessedFrame::ConstPtr& frame) const {
  if (!frame || debug_stamp_window_start < 0.0) {
    return false;
  }
  if (frame->stamp < debug_stamp_window_start) {
    return false;
  }
  if (debug_stamp_window_end >= debug_stamp_window_start && frame->stamp > debug_stamp_window_end) {
    return false;
  }
  return true;
}

void AsyncOdometryEstimation::run() {
  double last_imu_time = enable_imu ? 0.0 : std::numeric_limits<double>::max();
  double last_twist_time = std::numeric_limits<double>::max();
  std::deque<std::pair<double, cv::Mat>> images;
  std::deque<PreprocessedFrame::Ptr> raw_frames;

  while (!kill_switch) {
    auto imu_frames = input_imu_queue.get_all_and_clear();
    auto twist_frames = input_twist_queue.get_all_and_clear();
    auto new_images = input_image_queue.get_all_and_clear();
    auto new_raw_frames = input_frame_queue.get_all_and_clear();

    images.insert(images.end(), new_images.begin(), new_images.end());
    raw_frames.insert(raw_frames.end(), new_raw_frames.begin(), new_raw_frames.end());
    internal_frame_queue_size = raw_frames.size();

    if (images.empty() && imu_frames.empty() && twist_frames.empty() && raw_frames.empty()) {
      if (end_of_sequence) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    for (const auto& imu : imu_frames) {
      const double stamp = imu[0];
      const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
      const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
      odometry_estimation->insert_imu(stamp, linear_acc, angular_vel);

      last_imu_time = stamp;
    }

    for (const auto& twist : twist_frames) {
      const double stamp = twist[0];
      const double linear_vel = twist[1];
      odometry_estimation->insert_twist(stamp, linear_vel);

      last_twist_time = stamp;
    }

    while (!images.empty()) {
      if (!end_of_sequence && images.front().first > last_imu_time) {
        logger->debug("waiting for IMU data (image_time={:.6f}, last_imu_time={:.6f})", images.front().first, last_imu_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }

      const auto image = images.front();
      odometry_estimation->insert_image(image.first, image.second);
      images.pop_front();
    }

    while (!raw_frames.empty()) {
      const auto& frame = raw_frames.front();
      const bool debug_frame = debug_frame_enabled(frame);
      if (!end_of_sequence && frame->scan_end_time > last_imu_time) {
        logger->debug("waiting for IMU data (scan_end_time={:.6f}, last_imu_time={:.6f})", frame->scan_end_time, last_imu_time);
        if (debug_frame) {
          logger->info("odom-async-wait-imu seq={} stamp={:.6f} input_stamp={:.6f} scan_end={:.6f} raw_points={} points={} queue={} last_imu_time={:.6f}",
                       frame->debug_sequence_id,
                       frame->stamp,
                       frame->debug_input_stamp,
                       frame->scan_end_time,
                       frame->debug_raw_points,
                       frame->size(),
                       raw_frames.size(),
                       last_imu_time);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }

      if (debug_frame) {
        logger->info("odom-async-dequeue seq={} stamp={:.6f} input_stamp={:.6f} scan_end={:.6f} raw_points={} points={} queue_before={} last_imu_time={:.6f}",
                     frame->debug_sequence_id,
                     frame->stamp,
                     frame->debug_input_stamp,
                     frame->scan_end_time,
                     frame->debug_raw_points,
                     frame->size(),
                     raw_frames.size(),
                     last_imu_time);
      }
      std::vector<EstimationFrame::ConstPtr> marginalized;
      auto state = odometry_estimation->insert_frame(frame, marginalized);

      output_estimation_results.push_back(state);
      output_marginalized_frames.insert(marginalized);
      raw_frames.pop_front();
      internal_frame_queue_size = raw_frames.size();
      if (debug_frame) {
        logger->info("odom-async-result seq={} stamp={:.6f} state={} marginalized={} queue_after={}",
                     frame->debug_sequence_id,
                     frame->stamp,
                     state ? state->id : -1,
                     marginalized.size(),
                     raw_frames.size());
      }
    }
  }

  auto marginalized = odometry_estimation->get_remaining_frames();
  output_marginalized_frames.insert(marginalized);
}
}  // namespace glil
