#include <glil/preprocess/cloud_preprocessor.hpp>

#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glil/util/config.hpp>
#ifdef GLIL_PROFILE_TIMING
#include <atomic>

#include <glil/util/digest.hpp>
#include <glil/util/logging.hpp>
#endif

namespace glil {

#ifdef GLIL_PROFILE_TIMING
namespace {
bool debug_digest_enabled() {
  static const bool enabled = [] {
    Config logging_config(GlobalConfig::get_config_path("config_logging"));
    return logging_config.param<bool>("logging", "debug_digest").value_or(false);
  }();
  return enabled;
}

int next_digest_frame_id() {
  static std::atomic<int> frame_id{0};
  return frame_id++;
}
}  // namespace
#endif

CloudPreprocessorParams::CloudPreprocessorParams() {
  Config config(GlobalConfig::get_config_path("config_preprocess"));
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));

  global_shutter = sensor_config.param<bool>("sensors", "global_shutter_lidar", false);

  distance_near_thresh = config.param<double>("preprocess", "distance_near_thresh", 1.0);
  distance_far_thresh = config.param<double>("preprocess", "distance_far_thresh", 100.0);
  use_random_grid_downsampling = config.param<bool>("preprocess", "use_random_grid_downsampling", false);
  downsample_resolution = config.param<double>("preprocess", "downsample_resolution", 0.15);
  downsample_target = config.param<int>("preprocess", "random_downsample_target", 0);
  downsample_rate = config.param<double>("preprocess", "random_downsample_rate", 0.3);
  enable_outlier_removal = config.param<bool>("preprocess", "enable_outlier_removal", false);
  outlier_removal_k = config.param<int>("preprocess", "outlier_removal_k", 10);
  outlier_std_mul_factor = config.param<double>("preprocess", "outlier_std_mul_factor", 2.0);

  k_correspondences = config.param<int>("preprocess", "k_correspondences", 8);

  num_threads = config.param<int>("preprocess", "num_threads", 10);
}

CloudPreprocessorParams::~CloudPreprocessorParams() {}

CloudPreprocessor::CloudPreprocessor(const CloudPreprocessorParams& params) : params(params) {}

CloudPreprocessor::~CloudPreprocessor() {}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess(const RawPoints::ConstPtr& raw_points) {
  spdlog::trace("preprocessing input: {} points", raw_points->size());
#ifdef GLIL_PROFILE_TIMING
  const bool debug_digest = debug_digest_enabled();
  const int digest_frame_id = debug_digest ? next_digest_frame_id() : -1;
#endif

  gtsam_points::PointCloud::Ptr frame(new gtsam_points::PointCloud);
  frame->num_points = raw_points->size();
  frame->times = const_cast<double*>(raw_points->times.data());
  frame->points = const_cast<Eigen::Vector4d*>(raw_points->points.data());
  if (raw_points->intensities.size()) {
    frame->intensities = const_cast<double*>(raw_points->intensities.data());
  }

  // Downsampling
  if (params.use_random_grid_downsampling) {
    const double rate = params.downsample_target > 0 ? static_cast<double>(params.downsample_target) / frame->size() : params.downsample_rate;
    frame = gtsam_points::randomgrid_sampling(frame, params.downsample_resolution, rate, mt, params.num_threads);
  } else {
    frame = gtsam_points::voxelgrid_sampling(frame, params.downsample_resolution, params.num_threads);
  }

  if (frame->size() < 100) {
    spdlog::warn("too few points in the downsampled cloud ({} points)", frame->size());
  }

  // Distance filter
  std::vector<int> indices;
  indices.reserve(frame->size());
  for (int i = 0; i < frame->size(); i++) {
    const bool is_finite = frame->points[i].allFinite();
    const double dist = (Eigen::Vector4d() << frame->points[i].head<3>(), 0.0).finished().norm();
    if (dist > params.distance_near_thresh && dist < params.distance_far_thresh && is_finite) {
      indices.push_back(i);
    }
  }

  if (indices.size() < 100) {
    spdlog::warn("too few points in the filtered cloud ({} points)", indices.size());
  }

  // Sort by time
  std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return frame->times[lhs] < frame->times[rhs]; });
  frame = gtsam_points::sample(frame, indices);

  if (params.global_shutter) {
    std::fill(frame->times, frame->times + frame->size(), 0.0);
  }

  // Outlier removal
  if (params.enable_outlier_removal) {
    frame = gtsam_points::remove_outliers(frame, params.outlier_removal_k, params.outlier_std_mul_factor, params.num_threads);
  }

  // Create a preprocessed frame
  PreprocessedFrame::Ptr preprocessed(new PreprocessedFrame);
  preprocessed->stamp = raw_points->stamp;
  preprocessed->scan_end_time = frame->size() ? raw_points->stamp + frame->times[frame->size() - 1] : raw_points->stamp;

  preprocessed->times.assign(frame->times, frame->times + frame->size());
  preprocessed->points.assign(frame->points, frame->points + frame->size());
  if (frame->intensities) {
    preprocessed->intensities.assign(frame->intensities, frame->intensities + frame->size());
  }

  preprocessed->k_neighbors = params.k_correspondences;
  preprocessed->neighbors = find_neighbors(frame->points, frame->size(), params.k_correspondences);

#ifdef GLIL_PROFILE_TIMING
  if (debug_digest) {
    const double* point_data = preprocessed->points.empty() ? nullptr : preprocessed->points.front().data();
    const uint64_t points_digest = fnv1a_doubles(point_data, static_cast<std::size_t>(preprocessed->points.size()) * 4);
    create_module_logger("odom")->info("[digest] frame={} filtered_size={} digest_points={:016x}", digest_frame_id, preprocessed->size(), points_digest);
  }
#endif

  spdlog::trace("preprocessed: {} -> {} points", raw_points->size(), preprocessed->size());

  return preprocessed;
}

std::vector<int> CloudPreprocessor::find_neighbors(const Eigen::Vector4d* points, const int num_points, const int k) const {
  gtsam_points::KdTree tree(points, num_points);

  std::vector<int> neighbors(num_points * k);

#pragma omp parallel for num_threads(params.num_threads) schedule(guided, 8)
  for (int i = 0; i < num_points; i++) {
    std::vector<size_t> k_indices(k);
    std::vector<double> k_sq_dists(k);
    tree.knn_search(points[i].data(), k, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
  }

  return neighbors;
}

}  // namespace glil
