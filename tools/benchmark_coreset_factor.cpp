// SPDX-License-Identifier: MIT
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
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
  bool csv = false;
};

struct Dataset {
  gtsam_points::PointCloudCPU::Ptr target_cloud;
  gtsam_points::PointCloudCPU::Ptr source_cloud;
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> target_voxels;
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
            << "  --rows N       Synthetic grid rows (default: 18)\n"
            << "  --cols N       Synthetic grid cols (default: 18)\n"
            << "  --repeat N     Linearization repetitions (default: 8)\n"
            << "  --target N     Coreset target residual rows (default: 64)\n"
            << "  --clusters N   Fast-Caratheodory clusters (default: 64)\n"
            << "  --threads N    Factor threads (default: 1)\n"
            << "  --voxel M      Target voxel resolution in meters (default: 0.50)\n"
            << "  --spacing M    Synthetic point spacing in meters (default: 0.20)\n"
            << "  --csv          Print CSV instead of a fixed-width table\n";
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
  return options;
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

gtsam_points::PointCloudCPU::Ptr make_cloud(const std::vector<Eigen::Vector4d>& points) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  std::vector<Eigen::Matrix4d> covariances(points.size(), Eigen::Matrix4d::Zero());
  for (auto& covariance : covariances) {
    covariance.topLeftCorner<3, 3>() = 0.01 * Eigen::Matrix3d::Identity();
  }
  cloud->add_covs(covariances);
  return cloud;
}

Dataset make_dataset(const Options& options) {
  const auto points = make_points(options);
  Dataset dataset;
  dataset.target_cloud = make_cloud(points);
  dataset.source_cloud = make_cloud(points);
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

void print_csv(const std::vector<Result>& results) {
  std::cout << "mode,first_us,reuse_mean_us,constant_error,rel_aug_error,inliers,selected_rows,selected_points,weight_sum,extract_count,update_count\n";
  for (const auto& result : results) {
    std::cout << result.mode << ',' << result.first_us << ',' << result.reuse_mean_us << ',' << result.constant_error << ','
              << result.rel_augmented_error << ',' << result.inliers << ',' << result.stats.selected_residual_rows << ','
              << result.stats.selected_points << ',' << result.stats.weight_sum << ',' << result.stats.coreset_extraction_count << ','
              << result.stats.correspondence_update_count << '\n';
  }
}

void print_table(const Options& options, const std::vector<Result>& results) {
  std::cout << "synthetic_points=" << options.rows * options.cols << " target_rows=" << options.target_size
            << " clusters=" << options.clusters << " repeat=" << options.repeat << '\n';
  std::cout << std::left << std::setw(22) << "mode" << std::right << std::setw(12) << "first_us" << std::setw(14)
            << "reuse_us" << std::setw(14) << "rel_aug" << std::setw(10) << "inliers" << std::setw(10)
            << "rows" << std::setw(10) << "points" << std::setw(12) << "extracts" << '\n';
  for (const auto& result : results) {
    std::cout << std::left << std::setw(22) << result.mode << std::right << std::setw(12) << std::fixed << std::setprecision(1)
              << result.first_us << std::setw(14) << result.reuse_mean_us << std::setw(14) << std::scientific
              << std::setprecision(2) << result.rel_augmented_error << std::fixed << std::setw(10) << result.inliers
              << std::setw(10) << result.stats.selected_residual_rows << std::setw(10) << result.stats.selected_points
              << std::setw(12) << result.stats.coreset_extraction_count << '\n';
  }
}
}  // namespace

int main(int argc, char** argv) {
  const Options options = parse_options(argc, argv);
  const Dataset dataset = make_dataset(options);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3());

  std::vector<Result> results;
  results.reserve(3);
  results.push_back(run_full_vgicp(dataset, values, options));

  gtsam_points::IntegratedVGICPFactor reference_factor(gtsam::Pose3(), X(0), dataset.target_voxels, dataset.source_cloud);
  reference_factor.set_num_threads(options.threads);
  const auto reference = time_linearize(reference_factor, values, 1).second;

  results.push_back(run_coreset("uniform_sample", "uniform_sample", dataset, values, options, reference));
  results.push_back(run_coreset("exact_caratheodory", "exact_caratheodory", dataset, values, options, reference));

  if (options.csv) {
    print_csv(results);
  } else {
    print_table(options, results);
  }
  return 0;
}
