// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glil/perception/cloud_landmark_extractor.hpp>

namespace {

struct Options {
  std::string input_path;
  std::string output_path;
  std::string format = "auto";
  double stamp = 0.0;
  gtsam::Pose3 pose_world_sensor;
  glil::CloudLandmarkExtractorParams extractor;
  bool full_covariance = false;
  bool quiet = false;
};

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " --input cloud [options]\n"
            << "  --input P              Input point cloud path\n"
            << "  --output P             Output perception CSV path (default: stdout)\n"
            << "  --format F             auto|gtsam|pcd|xyz|kitti-bin|compact-bin (default: auto)\n"
            << "  --stamp T              Observation timestamp in seconds (default: 0)\n"
            << "  --pose-xyzrpy          tx ty tz roll pitch yaw sensor pose in world, radians\n"
            << "  --voxel M              World-frame landmark voxel size in meters (default: 1.0)\n"
            << "  --min-points N         Minimum points per landmark voxel (default: 5)\n"
            << "  --max-landmarks N      Keep the densest N landmarks; 0 keeps all (default: 0)\n"
            << "  --min-range M          Reject sensor-frame points closer than M; 0 disables\n"
            << "  --max-range M          Reject sensor-frame points farther than M; 0 disables\n"
            << "  --min-covariance V     Minimum covariance diagonal variance (default: 0.01)\n"
            << "  --confidence-points N  Points needed for confidence 1.0 (default: 20)\n"
            << "  --class-id NAME        CSV class_id label (default: cloud_landmark)\n"
            << "  --full-covariance      Write 3x3 covariance columns instead of diagonal CSV\n"
            << "  --quiet                Do not print extraction summary to stderr\n";
}

int parse_int(const char* value, const char* name) {
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    throw std::runtime_error(std::string("invalid integer for ") + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

std::size_t parse_size(const char* value, const char* name) {
  if (value[0] == '-') {
    throw std::runtime_error(std::string("invalid size for ") + name + ": " + value);
  }
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value, &end, 10);
  if (end == value || *end != '\0') {
    throw std::runtime_error(std::string("invalid size for ") + name + ": " + value);
  }
  return static_cast<std::size_t>(parsed);
}

double parse_double(const char* value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    throw std::runtime_error(std::string("invalid number for ") + name + ": " + value);
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

gtsam::Pose3 make_xyzrpy_pose(double tx, double ty, double tz, double roll, double pitch, double yaw) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(tx, ty, tz);
  pose.linear() =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  return gtsam::Pose3(pose.matrix());
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    const auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_help(argv[0]);
      std::exit(0);
    } else if (arg == "--input" || arg == "-i") {
      options.input_path = next("--input");
    } else if (arg == "--output" || arg == "-o") {
      options.output_path = next("--output");
    } else if (arg == "--format") {
      options.format = lower_string(next("--format"));
    } else if (arg == "--stamp") {
      options.stamp = parse_double(next("--stamp"), "--stamp");
    } else if (arg == "--pose-xyzrpy") {
      const double tx = parse_double(next("--pose-xyzrpy tx"), "--pose-xyzrpy tx");
      const double ty = parse_double(next("--pose-xyzrpy ty"), "--pose-xyzrpy ty");
      const double tz = parse_double(next("--pose-xyzrpy tz"), "--pose-xyzrpy tz");
      const double roll = parse_double(next("--pose-xyzrpy roll"), "--pose-xyzrpy roll");
      const double pitch = parse_double(next("--pose-xyzrpy pitch"), "--pose-xyzrpy pitch");
      const double yaw = parse_double(next("--pose-xyzrpy yaw"), "--pose-xyzrpy yaw");
      options.pose_world_sensor = make_xyzrpy_pose(tx, ty, tz, roll, pitch, yaw);
    } else if (arg == "--voxel") {
      options.extractor.voxel_resolution = parse_double(next("--voxel"), "--voxel");
    } else if (arg == "--min-points") {
      options.extractor.min_points_per_landmark = parse_int(next("--min-points"), "--min-points");
    } else if (arg == "--max-landmarks") {
      options.extractor.max_landmarks = parse_size(next("--max-landmarks"), "--max-landmarks");
    } else if (arg == "--min-range") {
      options.extractor.min_range = parse_double(next("--min-range"), "--min-range");
    } else if (arg == "--max-range") {
      options.extractor.max_range = parse_double(next("--max-range"), "--max-range");
    } else if (arg == "--min-covariance") {
      options.extractor.min_covariance = parse_double(next("--min-covariance"), "--min-covariance");
    } else if (arg == "--confidence-points") {
      options.extractor.confidence_point_count = parse_double(next("--confidence-points"), "--confidence-points");
    } else if (arg == "--class-id") {
      options.extractor.class_id = next("--class-id");
    } else if (arg == "--full-covariance") {
      options.full_covariance = true;
    } else if (arg == "--quiet") {
      options.quiet = true;
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  if (options.input_path.empty()) {
    throw std::runtime_error("--input is required");
  }
  if (!is_supported_format(options.format)) {
    throw std::runtime_error("unsupported --format: " + options.format);
  }
  options.extractor.voxel_resolution = std::max(options.extractor.voxel_resolution, 1e-6);
  options.extractor.min_points_per_landmark = std::max(options.extractor.min_points_per_landmark, 1);
  options.extractor.min_covariance = std::max(options.extractor.min_covariance, 1e-12);
  options.extractor.confidence_point_count = std::max(options.extractor.confidence_point_count, 1.0);
  return options;
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

bool parse_token_double(const std::string& token, double& value) {
  char* end = nullptr;
  value = std::strtod(token.c_str(), &end);
  return end != token.c_str() && *end == '\0';
}

void append_finite_point(std::vector<gtsam::Point3>& points, double x, double y, double z) {
  if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
    points.emplace_back(x, y, z);
  }
}

std::vector<gtsam::Point3> load_xyz_points(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open xyz cloud: " + path);
  }

  std::vector<gtsam::Point3> points;
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
    if (words.size() < 3) {
      throw std::runtime_error("expected at least three columns in " + path + " line " + std::to_string(line_no));
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!parse_token_double(words[0], x)) {
      if (points.empty()) {
        continue;
      }
      throw std::runtime_error("invalid x value in " + path + " line " + std::to_string(line_no));
    }
    if (!parse_token_double(words[1], y) || !parse_token_double(words[2], z)) {
      throw std::runtime_error("invalid y/z value in " + path + " line " + std::to_string(line_no));
    }
    append_finite_point(points, x, y, z);
  }

  if (points.empty()) {
    throw std::runtime_error("xyz cloud has no valid points: " + path);
  }
  return points;
}

std::vector<gtsam::Point3> load_pcd_ascii_points(const std::string& path) {
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

  std::vector<gtsam::Point3> points;
  while (std::getline(ifs, line)) {
    const auto words = split_words(line);
    if (static_cast<int>(words.size()) <= max_idx) {
      continue;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!parse_token_double(words[x_idx], x) || !parse_token_double(words[y_idx], y) || !parse_token_double(words[z_idx], z)) {
      continue;
    }
    append_finite_point(points, x, y, z);
  }

  if (points.empty()) {
    throw std::runtime_error("ASCII PCD has no valid points: " + path);
  }
  return points;
}

std::vector<gtsam::Point3> load_binary_float_points(const std::string& path, const std::string& format) {
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

  std::vector<gtsam::Point3> points;
  points.reserve(num_floats / static_cast<std::size_t>(floats_per_point));
  for (std::size_t i = 0; i < raw.size(); i += static_cast<std::size_t>(floats_per_point)) {
    append_finite_point(points, raw[i], raw[i + 1], raw[i + 2]);
  }

  if (points.empty()) {
    throw std::runtime_error("binary cloud has no valid points: " + path);
  }
  return points;
}

std::vector<gtsam::Point3> load_gtsam_points(const std::string& path) {
  auto cloud = gtsam_points::PointCloudCPU::load(path);
  if (!cloud || cloud->size() == 0 || !cloud->points) {
    throw std::runtime_error("gtsam cloud has no valid points: " + path);
  }

  std::vector<gtsam::Point3> points;
  points.reserve(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); i++) {
    const Eigen::Vector4d& point = cloud->points[i];
    append_finite_point(points, point.x(), point.y(), point.z());
  }
  if (points.empty()) {
    throw std::runtime_error("gtsam cloud has no finite points: " + path);
  }
  return points;
}

std::vector<gtsam::Point3> load_points(const std::string& path, const std::string& requested_format) {
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
    return load_gtsam_points(path);
  }
  if (format == "pcd") {
    return load_pcd_ascii_points(path);
  }
  if (format == "xyz") {
    return load_xyz_points(path);
  }
  if (format == "kitti-bin" || format == "compact-bin") {
    return load_binary_float_points(path, format);
  }
  throw std::runtime_error("unsupported cloud format: " + requested_format);
}

void write_csv(std::ostream& output, const std::vector<glil::PerceptionObservation>& observations, bool full_covariance) {
  output << std::setprecision(17);
  if (full_covariance) {
    output << "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,confidence\n";
  } else {
    output << "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_yy,cov_zz,confidence\n";
  }

  for (const auto& observation : observations) {
    output << observation.stamp << ',' << observation.class_id << ',' << observation.landmark_id << ',' << observation.position_sensor.x() << ','
           << observation.position_sensor.y() << ',' << observation.position_sensor.z() << ',';
    if (full_covariance) {
      output << observation.covariance(0, 0) << ',' << observation.covariance(0, 1) << ',' << observation.covariance(0, 2) << ','
             << observation.covariance(1, 0) << ',' << observation.covariance(1, 1) << ',' << observation.covariance(1, 2) << ','
             << observation.covariance(2, 0) << ',' << observation.covariance(2, 1) << ',' << observation.covariance(2, 2) << ',';
    } else {
      output << observation.covariance(0, 0) << ',' << observation.covariance(1, 1) << ',' << observation.covariance(2, 2) << ',';
    }
    output << observation.confidence << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const std::vector<gtsam::Point3> points = load_points(options.input_path, options.format);
    const auto result = glil::extract_cloud_landmark_observations(points, options.pose_world_sensor, options.stamp, options.extractor);

    std::ofstream file;
    std::ostream* output = &std::cout;
    if (!options.output_path.empty()) {
      file.open(options.output_path);
      if (!file) {
        throw std::runtime_error("failed to open output CSV: " + options.output_path);
      }
      output = &file;
    }
    write_csv(*output, result.observations, options.full_covariance);

    if (!options.quiet) {
      std::cerr << "loaded_points=" << points.size() << " finite_points=" << result.finite_points << " range_rejected=" << result.range_rejected
                << " voxel_candidates=" << result.voxel_candidates << " rejected_min_points=" << result.rejected_min_points
                << " observations=" << result.observations.size() << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
