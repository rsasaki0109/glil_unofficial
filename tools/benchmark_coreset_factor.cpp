// SPDX-License-Identifier: MIT
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glil/factors/integrated_vgicp_coreset_factor.hpp>

namespace {
using gtsam::symbol_shorthand::X;
using Clock = std::chrono::steady_clock;

struct Options {
  int rows = 18;
  int cols = 18;
  int repeat = 8;
  int target_size = 64;
  int clusters = 64;
  int threads = 1;
  double spacing = 0.20;
  double voxel_resolution = 0.50;
  double fallback_covariance = 0.01;
  bool csv = false;
  std::string target_cloud_path;
  std::string source_cloud_path;
  std::string target_format = "auto";
  std::string source_format = "auto";
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
};

struct Dataset {
  gtsam_points::PointCloudCPU::Ptr target_cloud;
  gtsam_points::PointCloudCPU::Ptr source_cloud;
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> target_voxels;
  bool synthetic = true;
};

struct Result {
  std::string mode;
  double first_us = 0.0;
  double reuse_mean_us = 0.0;
  double constant_error = 0.0;
  double rel_augmented_error = 0.0;
  int inliers = 0;
  glil::IntegratedVGICPCoresetFactor::CoresetStats stats;
};

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [options]\n"
            << "  --rows N            Synthetic grid rows (default: 18)\n"
            << "  --cols N            Synthetic grid cols (default: 18)\n"
            << "  --repeat N          Linearization repetitions (default: 8)\n"
            << "  --target N          Coreset target residual rows (default: 64)\n"
            << "  --clusters N        Fast-Caratheodory clusters (default: 64)\n"
            << "  --threads N         Factor threads (default: 1)\n"
            << "  --voxel M           Target voxel resolution in meters (default: 0.50)\n"
            << "  --spacing M         Synthetic point spacing in meters (default: 0.20)\n"
            << "  --target-cloud P    Real target cloud path\n"
            << "  --source-cloud P    Real source cloud path\n"
            << "  --target-format F   auto|gtsam|pcd|xyz|kitti-bin|compact-bin (default: auto)\n"
            << "  --source-format F   auto|gtsam|pcd|xyz|kitti-bin|compact-bin (default: auto)\n"
            << "  --initial-xyzrpy    tx ty tz roll pitch yaw initial source pose in radians\n"
            << "  --fallback-covariance V  Isotropic covariance for point-only inputs (default: 0.01)\n"
            << "  --csv               Print CSV instead of a fixed-width table\n";
}

int parse_int(const char* value, const char* name) {
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    std::cerr << "invalid integer for " << name << ": " << value << std::endl;
    std::exit(2);
  }
  return static_cast<int>(parsed);
}

double parse_double(const char* value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    std::cerr << "invalid number for " << name << ": " << value << std::endl;
    std::exit(2);
  }
  return parsed;
}

std::string lower_string(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string extension_of(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
    return "";
  }
  return lower_string(path.substr(dot + 1));
}

bool is_directory(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool is_supported_format(const std::string& format) {
  return format == "auto" || format == "gtsam" || format == "pcd" || format == "xyz" || format == "kitti-bin" || format == "compact-bin";
}

Eigen::Isometry3d make_xyzrpy_pose(double tx, double ty, double tz, double roll, double pitch, double yaw) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(tx, ty, tz);
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  pose.linear() = rotation;
  return pose;
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    const auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << std::endl;
        std::exit(2);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_help(argv[0]);
      std::exit(0);
    } else if (arg == "--rows") {
      options.rows = parse_int(next("--rows"), "--rows");
    } else if (arg == "--cols") {
      options.cols = parse_int(next("--cols"), "--cols");
    } else if (arg == "--repeat") {
      options.repeat = parse_int(next("--repeat"), "--repeat");
    } else if (arg == "--target") {
      options.target_size = parse_int(next("--target"), "--target");
    } else if (arg == "--clusters") {
      options.clusters = parse_int(next("--clusters"), "--clusters");
    } else if (arg == "--threads") {
      options.threads = parse_int(next("--threads"), "--threads");
    } else if (arg == "--voxel") {
      options.voxel_resolution = parse_double(next("--voxel"), "--voxel");
    } else if (arg == "--spacing") {
      options.spacing = parse_double(next("--spacing"), "--spacing");
    } else if (arg == "--fallback-covariance") {
      options.fallback_covariance = parse_double(next("--fallback-covariance"), "--fallback-covariance");
    } else if (arg == "--target-cloud") {
      options.target_cloud_path = next("--target-cloud");
    } else if (arg == "--source-cloud") {
      options.source_cloud_path = next("--source-cloud");
    } else if (arg == "--target-format") {
      options.target_format = lower_string(next("--target-format"));
    } else if (arg == "--source-format") {
      options.source_format = lower_string(next("--source-format"));
    } else if (arg == "--initial-xyzrpy") {
      const double tx = parse_double(next("--initial-xyzrpy tx"), "--initial-xyzrpy tx");
      const double ty = parse_double(next("--initial-xyzrpy ty"), "--initial-xyzrpy ty");
      const double tz = parse_double(next("--initial-xyzrpy tz"), "--initial-xyzrpy tz");
      const double roll = parse_double(next("--initial-xyzrpy roll"), "--initial-xyzrpy roll");
      const double pitch = parse_double(next("--initial-xyzrpy pitch"), "--initial-xyzrpy pitch");
      const double yaw = parse_double(next("--initial-xyzrpy yaw"), "--initial-xyzrpy yaw");
      options.initial_pose = make_xyzrpy_pose(tx, ty, tz, roll, pitch, yaw);
    } else if (arg == "--csv") {
      options.csv = true;
    } else {
      std::cerr << "unknown option: " << arg << std::endl;
      std::exit(2);
    }
  }

  options.rows = std::max(2, options.rows);
  options.cols = std::max(2, options.cols);
  options.repeat = std::max(1, options.repeat);
  options.target_size = std::max(29, options.target_size);
  options.clusters = std::max(1, options.clusters);
  options.threads = std::max(1, options.threads);
  options.spacing = std::max(1e-3, options.spacing);
  options.voxel_resolution = std::max(1e-3, options.voxel_resolution);
  options.fallback_covariance = std::max(1e-9, options.fallback_covariance);

  if (options.target_cloud_path.empty() != options.source_cloud_path.empty()) {
    std::cerr << "--target-cloud and --source-cloud must be provided together" << std::endl;
    std::exit(2);
  }
  if (!is_supported_format(options.target_format)) {
    std::cerr << "unsupported --target-format: " << options.target_format << std::endl;
    std::exit(2);
  }
  if (!is_supported_format(options.source_format)) {
    std::cerr << "unsupported --source-format: " << options.source_format << std::endl;
    std::exit(2);
  }
  return options;
}

void add_fallback_covariances(const gtsam_points::PointCloudCPU::Ptr& cloud, double variance) {
  if (!cloud) {
    throw std::runtime_error("cloud load returned null");
  }
  if (cloud->size() <= 0 || !cloud->points) {
    throw std::runtime_error("cloud has no points");
  }
  if (cloud->covs) {
    return;
  }

  std::vector<Eigen::Matrix4d> covariances(static_cast<std::size_t>(cloud->size()), Eigen::Matrix4d::Zero());
  for (auto& covariance : covariances) {
    covariance.topLeftCorner<3, 3>() = variance * Eigen::Matrix3d::Identity();
  }
  cloud->add_covs(covariances);
}

gtsam_points::PointCloudCPU::Ptr make_cloud(const std::vector<Eigen::Vector4d>& points, double covariance) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  add_fallback_covariances(cloud, covariance);
  return cloud;
}

std::vector<Eigen::Vector4d> make_points(const Options& options) {
  std::vector<Eigen::Vector4d> points;
  points.reserve(static_cast<std::size_t>(options.rows * options.cols));
  for (int r = 0; r < options.rows; r++) {
    for (int c = 0; c < options.cols; c++) {
      const double x = options.spacing * static_cast<double>(c);
      const double y = options.spacing * static_cast<double>(r);
      const double z = 0.04 * std::sin(0.4 * static_cast<double>(r)) + 0.03 * std::cos(0.3 * static_cast<double>(c));
      points.emplace_back(x, y, z, 1.0);
    }
  }
  return points;
}

std::vector<std::string> split_words(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> words;
  std::string word;
  while (iss >> word) {
    words.push_back(word);
  }
  return words;
}

gtsam_points::PointCloudCPU::Ptr load_xyz_cloud(const std::string& path, double covariance) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open xyz cloud: " + path);
  }

  std::vector<Eigen::Vector4d> points;
  std::string line;
  int line_no = 0;
  while (std::getline(ifs, line)) {
    line_no++;
    for (char& c : line) {
      if (c == ',' || c == ';') {
        c = ' ';
      }
    }

    const auto words = split_words(line);
    if (words.empty() || words.front()[0] == '#') {
      continue;
    }

    char* end = nullptr;
    const double x = std::strtod(words[0].c_str(), &end);
    if (end == words[0].c_str() || *end != '\0') {
      if (points.empty()) {
        continue;
      }
      throw std::runtime_error("invalid x value in " + path + " line " + std::to_string(line_no));
    }
    if (words.size() < 3) {
      throw std::runtime_error("expected at least three columns in " + path + " line " + std::to_string(line_no));
    }
    const double y = std::strtod(words[1].c_str(), &end);
    if (end == words[1].c_str() || *end != '\0') {
      throw std::runtime_error("invalid y value in " + path + " line " + std::to_string(line_no));
    }
    const double z = std::strtod(words[2].c_str(), &end);
    if (end == words[2].c_str() || *end != '\0') {
      throw std::runtime_error("invalid z value in " + path + " line " + std::to_string(line_no));
    }
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points.emplace_back(x, y, z, 1.0);
    }
  }

  if (points.empty()) {
    throw std::runtime_error("xyz cloud has no valid points: " + path);
  }
  return make_cloud(points, covariance);
}

gtsam_points::PointCloudCPU::Ptr load_pcd_ascii_cloud(const std::string& path, double covariance) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open pcd cloud: " + path);
  }

  std::vector<std::string> fields;
  bool data_ascii = false;
  std::string line;
  while (std::getline(ifs, line)) {
    const auto words = split_words(line);
    if (words.empty() || words.front()[0] == '#') {
      continue;
    }

    const std::string key = lower_string(words.front());
    if (key == "fields") {
      fields.assign(words.begin() + 1, words.end());
      for (auto& field : fields) {
        field = lower_string(field);
      }
    } else if (key == "data") {
      if (words.size() < 2 || lower_string(words[1]) != "ascii") {
        throw std::runtime_error("only ASCII PCD is supported: " + path);
      }
      data_ascii = true;
      break;
    }
  }

  if (!data_ascii) {
    throw std::runtime_error("PCD header does not contain DATA ascii: " + path);
  }

  const auto find_field = [&](const std::string& name) {
    const auto it = std::find(fields.begin(), fields.end(), name);
    return it == fields.end() ? -1 : static_cast<int>(std::distance(fields.begin(), it));
  };
  const int x_idx = find_field("x");
  const int y_idx = find_field("y");
  const int z_idx = find_field("z");
  if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
    throw std::runtime_error("PCD header must contain x/y/z fields: " + path);
  }
  const int max_idx = std::max(x_idx, std::max(y_idx, z_idx));

  std::vector<Eigen::Vector4d> points;
  while (std::getline(ifs, line)) {
    const auto words = split_words(line);
    if (static_cast<int>(words.size()) <= max_idx) {
      continue;
    }
    char* end = nullptr;
    const double x = std::strtod(words[x_idx].c_str(), &end);
    if (end == words[x_idx].c_str() || *end != '\0') {
      continue;
    }
    const double y = std::strtod(words[y_idx].c_str(), &end);
    if (end == words[y_idx].c_str() || *end != '\0') {
      continue;
    }
    const double z = std::strtod(words[z_idx].c_str(), &end);
    if (end == words[z_idx].c_str() || *end != '\0') {
      continue;
    }
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points.emplace_back(x, y, z, 1.0);
    }
  }

  if (points.empty()) {
    throw std::runtime_error("ASCII PCD has no valid points: " + path);
  }
  return make_cloud(points, covariance);
}

gtsam_points::PointCloudCPU::Ptr load_binary_float_cloud(const std::string& path, const std::string& format, double covariance) {
  std::ifstream ifs(path, std::ios::binary | std::ios::ate);
  if (!ifs) {
    throw std::runtime_error("failed to open binary cloud: " + path);
  }

  const std::streamsize bytes = ifs.tellg();
  if (bytes <= 0 || bytes % static_cast<std::streamsize>(sizeof(float)) != 0) {
    throw std::runtime_error("binary cloud size is not a positive float array: " + path);
  }

  int floats_per_point = 0;
  if (format == "kitti-bin") {
    floats_per_point = 4;
  } else if (format == "compact-bin") {
    floats_per_point = 3;
  } else if (bytes % static_cast<std::streamsize>(4 * sizeof(float)) == 0) {
    floats_per_point = 4;
  } else if (bytes % static_cast<std::streamsize>(3 * sizeof(float)) == 0) {
    floats_per_point = 3;
  } else {
    throw std::runtime_error("auto .bin detection failed; use kitti-bin or compact-bin format: " + path);
  }

  const std::size_t num_floats = static_cast<std::size_t>(bytes / sizeof(float));
  if (num_floats % static_cast<std::size_t>(floats_per_point) != 0) {
    throw std::runtime_error("binary cloud size does not match selected point stride: " + path);
  }

  std::vector<float> raw(num_floats);
  ifs.seekg(0, std::ios::beg);
  ifs.read(reinterpret_cast<char*>(raw.data()), bytes);

  std::vector<Eigen::Vector4d> points;
  points.reserve(num_floats / static_cast<std::size_t>(floats_per_point));
  for (std::size_t i = 0; i < raw.size(); i += static_cast<std::size_t>(floats_per_point)) {
    const double x = raw[i];
    const double y = raw[i + 1];
    const double z = raw[i + 2];
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points.emplace_back(x, y, z, 1.0);
    }
  }

  if (points.empty()) {
    throw std::runtime_error("binary cloud has no valid points: " + path);
  }
  return make_cloud(points, covariance);
}

gtsam_points::PointCloudCPU::Ptr load_gtsam_cloud(const std::string& path, double covariance) {
  auto cloud = gtsam_points::PointCloudCPU::load(path);
  add_fallback_covariances(cloud, covariance);
  return cloud;
}

gtsam_points::PointCloudCPU::Ptr load_cloud(const std::string& path, const std::string& requested_format, double covariance) {
  std::string format = requested_format;
  if (format == "auto") {
    if (is_directory(path)) {
      format = "gtsam";
    } else {
      const std::string ext = extension_of(path);
      if (ext == "pcd") {
        format = "pcd";
      } else if (ext == "bin") {
        format = "kitti-bin";
      } else {
        format = "xyz";
      }
    }
  }

  if (format == "gtsam") {
    return load_gtsam_cloud(path, covariance);
  }
  if (format == "pcd") {
    return load_pcd_ascii_cloud(path, covariance);
  }
  if (format == "xyz") {
    return load_xyz_cloud(path, covariance);
  }
  if (format == "kitti-bin" || format == "compact-bin") {
    return load_binary_float_cloud(path, format, covariance);
  }
  throw std::runtime_error("unsupported cloud format: " + requested_format);
}

Dataset make_dataset(const Options& options) {
  Dataset dataset;
  if (!options.target_cloud_path.empty()) {
    dataset.synthetic = false;
    dataset.target_cloud = load_cloud(options.target_cloud_path, options.target_format, options.fallback_covariance);
    dataset.source_cloud = load_cloud(options.source_cloud_path, options.source_format, options.fallback_covariance);
  } else {
    const auto points = make_points(options);
    dataset.target_cloud = make_cloud(points, options.fallback_covariance);
    dataset.source_cloud = make_cloud(points, options.fallback_covariance);
  }

  dataset.target_voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(options.voxel_resolution);
  dataset.target_voxels->insert(*dataset.target_cloud);
  return dataset;
}

template <typename Factor>
std::pair<std::vector<double>, gtsam::Matrix> time_linearize(Factor& factor, const gtsam::Values& values, int repeat) {
  std::vector<double> samples_us;
  samples_us.reserve(static_cast<std::size_t>(repeat));
  gtsam::GaussianFactor::shared_ptr linearized;

  for (int i = 0; i < repeat; i++) {
    const auto start = Clock::now();
    linearized = factor.linearize(values);
    const auto stop = Clock::now();
    samples_us.push_back(static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count()) / 1000.0);
  }

  return {samples_us, linearized ? linearized->augmentedInformation() : gtsam::Matrix()};
}

double mean_tail(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  if (values.size() == 1) {
    return values.front();
  }
  return std::accumulate(values.begin() + 1, values.end(), 0.0) / static_cast<double>(values.size() - 1);
}

double constant_from_augmented(const gtsam::Matrix& augmented) {
  if (augmented.size() == 0) {
    return 0.0;
  }
  return augmented(augmented.rows() - 1, augmented.cols() - 1);
}

double relative_augmented_error(const gtsam::Matrix& reference, const gtsam::Matrix& candidate) {
  if (reference.size() == 0 || candidate.rows() != reference.rows() || candidate.cols() != reference.cols()) {
    return 0.0;
  }
  const double denom = std::max(1e-12, reference.norm());
  return (reference - candidate).norm() / denom;
}

Result run_full_vgicp(const Dataset& dataset, const gtsam::Values& values, const Options& options) {
  gtsam_points::IntegratedVGICPFactor factor(gtsam::Pose3(), X(0), dataset.target_voxels, dataset.source_cloud);
  factor.set_num_threads(options.threads);

  const auto [samples, augmented] = time_linearize(factor, values, options.repeat);
  Result result;
  result.mode = "full_vgicp";
  result.first_us = samples.front();
  result.reuse_mean_us = mean_tail(samples);
  result.constant_error = constant_from_augmented(augmented);
  result.inliers = factor.num_inliers();
  result.stats.source_points = static_cast<int>(dataset.source_cloud->size());
  result.stats.valid_correspondences = result.inliers;
  result.stats.selected_points = result.inliers;
  result.stats.selected_residual_rows = result.inliers * 3;
  result.stats.weight_sum = static_cast<double>(result.stats.selected_residual_rows);
  return result;
}

Result run_coreset(
  const std::string& mode,
  const std::string& method,
  const Dataset& dataset,
  const gtsam::Values& values,
  const Options& options,
  const gtsam::Matrix& reference_augmented) {
  glil::IntegratedVGICPCoresetFactor::Params params;
  params.coreset_target_size = options.target_size;
  params.coreset_num_clusters = options.clusters;
  params.coreset_method = method;
  params.num_threads = options.threads;
  params.coreset_immutable_snapshot = true;

  glil::IntegratedVGICPCoresetFactor factor(gtsam::Pose3(), X(0), dataset.target_voxels, dataset.source_cloud, params);
  const auto [samples, augmented] = time_linearize(factor, values, options.repeat);

  Result result;
  result.mode = mode;
  result.first_us = samples.front();
  result.reuse_mean_us = mean_tail(samples);
  result.constant_error = constant_from_augmented(augmented);
  result.rel_augmented_error = relative_augmented_error(reference_augmented, augmented);
  result.stats = factor.coreset_stats();
  result.inliers = result.stats.valid_correspondences;
  return result;
}

void print_csv(const Dataset& dataset, const std::vector<Result>& results) {
  std::cout << "mode,target_points,source_points,first_us,reuse_mean_us,constant_error,rel_aug_error,inliers,selected_rows,selected_points,weight_sum,extract_count,update_count\n";
  for (const auto& result : results) {
    std::cout << result.mode << ',' << dataset.target_cloud->size() << ',' << dataset.source_cloud->size() << ',' << result.first_us << ',' << result.reuse_mean_us << ','
              << result.constant_error << ',' << result.rel_augmented_error << ',' << result.inliers << ',' << result.stats.selected_residual_rows << ','
              << result.stats.selected_points << ',' << result.stats.weight_sum << ',' << result.stats.coreset_extraction_count << ','
              << result.stats.correspondence_update_count << '\n';
  }
}

void print_table(const Options& options, const Dataset& dataset, const std::vector<Result>& results) {
  if (dataset.synthetic) {
    std::cout << "input=synthetic synthetic_points=" << options.rows * options.cols;
  } else {
    std::cout << "input=cloud target_points=" << dataset.target_cloud->size() << " source_points=" << dataset.source_cloud->size();
  }
  std::cout << " target_rows=" << options.target_size << " clusters=" << options.clusters << " repeat=" << options.repeat << '\n';
  std::cout << std::left << std::setw(22) << "mode" << std::right << std::setw(12) << "first_us" << std::setw(14) << "reuse_us" << std::setw(14)
            << "rel_aug" << std::setw(10) << "inliers" << std::setw(10) << "rows" << std::setw(10) << "points" << std::setw(12) << "extracts" << '\n';
  for (const auto& result : results) {
    std::cout << std::left << std::setw(22) << result.mode << std::right << std::setw(12) << std::fixed << std::setprecision(1) << result.first_us
              << std::setw(14) << result.reuse_mean_us << std::setw(14) << std::scientific << std::setprecision(2) << result.rel_augmented_error << std::fixed
              << std::setw(10) << result.inliers << std::setw(10) << result.stats.selected_residual_rows << std::setw(10) << result.stats.selected_points
              << std::setw(12) << result.stats.coreset_extraction_count << '\n';
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const Dataset dataset = make_dataset(options);

    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3(options.initial_pose.matrix()));

    std::vector<Result> results;
    results.reserve(3);
    results.push_back(run_full_vgicp(dataset, values, options));

    gtsam_points::IntegratedVGICPFactor reference_factor(gtsam::Pose3(), X(0), dataset.target_voxels, dataset.source_cloud);
    reference_factor.set_num_threads(options.threads);
    const auto reference = time_linearize(reference_factor, values, 1).second;

    results.push_back(run_coreset("uniform_sample", "uniform_sample", dataset, values, options, reference));
    results.push_back(run_coreset("exact_caratheodory", "exact_caratheodory", dataset, values, options, reference));

    if (options.csv) {
      print_csv(dataset, results);
    } else {
      print_table(options, dataset, results);
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
