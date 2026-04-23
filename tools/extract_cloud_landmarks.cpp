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
  std::string batch_csv_path;
  std::string output_path;
  std::string format = "auto";
  double stamp = 0.0;
  gtsam::Pose3 pose_world_sensor;
  glil::CloudLandmarkExtractorParams extractor;
  std::string base_dir;
  std::string path_column = "path";
  std::string stamp_column = "stamp";
  std::vector<std::string> pose_columns = {"tx", "ty", "tz", "roll", "pitch", "yaw"};
  bool skip_invalid_rows = false;
  bool full_covariance = false;
  bool quiet = false;
};

struct BatchFrame {
  std::string cloud_path;
  double stamp = 0.0;
  gtsam::Pose3 pose_world_sensor;
  int line = 0;
};

struct BatchSummary {
  std::size_t frames = 0;
  std::size_t failed_frames = 0;
  std::size_t loaded_points = 0;
  std::size_t finite_points = 0;
  std::size_t range_rejected = 0;
  std::size_t voxel_candidates = 0;
  std::size_t rejected_min_points = 0;
  std::size_t observations = 0;
};

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " --input cloud [options]\n"
            << "       " << argv0 << " --batch-csv frames.csv --output observations.csv [options]\n"
            << "  --input P              Single input point cloud path\n"
            << "  --batch-csv P          Batch CSV with path,stamp,tx,ty,tz,roll,pitch,yaw columns\n"
            << "  --output P             Output perception CSV path (default: stdout)\n"
            << "  --format F             auto|gtsam|pcd|xyz|kitti-bin|compact-bin (default: auto)\n"
            << "  --stamp T              Single-mode observation timestamp in seconds (default: 0)\n"
            << "  --pose-xyzrpy          Single-mode tx ty tz roll pitch yaw sensor pose in world, radians\n"
            << "  --base-dir P           Base directory for relative batch cloud paths (default: batch CSV directory)\n"
            << "  --path-column NAME     Batch cloud path column name (default: path)\n"
            << "  --stamp-column NAME    Batch timestamp column name (default: stamp)\n"
            << "  --pose-columns NAMES   Batch pose columns tx,ty,tz,roll,pitch,yaw (comma-separated)\n"
            << "  --skip-invalid-rows    Continue after invalid batch rows or failed clouds\n"
            << "  --voxel M              World-frame landmark voxel size in meters (default: 1.0)\n"
            << "  --min-points N         Minimum points per landmark voxel (default: 5)\n"
            << "  --max-landmarks N      Keep the densest N landmarks per cloud; 0 keeps all\n"
            << "  --min-range M          Reject sensor-frame points closer than M; 0 disables\n"
            << "  --max-range M          Reject sensor-frame points farther than M; 0 disables\n"
            << "  --min-covariance V     Minimum covariance diagonal variance (default: 0.01)\n"
            << "  --confidence-points N  Points needed for confidence 1.0 (default: 20)\n"
            << "  --class-id NAME        CSV class_id label (default: cloud_landmark)\n"
            << "  --full-covariance      Write 3x3 covariance columns instead of diagonal CSV\n"
            << "  --quiet                Do not print extraction summary to stderr\n";
}

std::string trim(std::string value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c) != 0; });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
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

std::string directory_of(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  if (slash == std::string::npos) {
    return "";
  }
  return path.substr(0, slash);
}

bool is_absolute_path(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  if (path.front() == '/' || path.front() == '\\') {
    return true;
  }
  return path.size() >= 2 && std::isalpha(static_cast<unsigned char>(path[0])) != 0 && path[1] == ':';
}

std::string join_path(const std::string& base, const std::string& path) {
  if (path.empty() || base.empty() || is_absolute_path(path)) {
    return path;
  }
  const char last = base.back();
  if (last == '/' || last == '\\') {
    return base + path;
  }
  return base + "/" + path;
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

std::vector<std::string> split_comma_list(const std::string& value) {
  std::vector<std::string> items;
  std::string item;
  std::istringstream stream(value);
  while (std::getline(stream, item, ',')) {
    items.push_back(trim(item));
  }
  return items;
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
    } else if (arg == "--batch-csv") {
      options.batch_csv_path = next("--batch-csv");
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
    } else if (arg == "--base-dir") {
      options.base_dir = next("--base-dir");
    } else if (arg == "--path-column") {
      options.path_column = next("--path-column");
    } else if (arg == "--stamp-column") {
      options.stamp_column = next("--stamp-column");
    } else if (arg == "--pose-columns") {
      options.pose_columns = split_comma_list(next("--pose-columns"));
    } else if (arg == "--skip-invalid-rows") {
      options.skip_invalid_rows = true;
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

  if (options.input_path.empty() == options.batch_csv_path.empty()) {
    throw std::runtime_error("provide exactly one of --input or --batch-csv");
  }
  if (!is_supported_format(options.format)) {
    throw std::runtime_error("unsupported --format: " + options.format);
  }
  if (options.pose_columns.size() != 6) {
    throw std::runtime_error("--pose-columns must contain exactly six comma-separated names");
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

std::vector<std::string> split_csv_line(const std::string& line, char delimiter = ',') {
  std::vector<std::string> tokens;
  std::string token;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); i++) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        token.push_back('"');
        i++;
      } else {
        quoted = !quoted;
      }
    } else if (c == delimiter && !quoted) {
      tokens.push_back(trim(token));
      token.clear();
    } else {
      token.push_back(c);
    }
  }
  tokens.push_back(trim(token));
  return tokens;
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

int find_column(const std::vector<std::string>& header, const std::string& name) {
  const auto exact = std::find(header.begin(), header.end(), name);
  if (exact != header.end()) {
    return static_cast<int>(std::distance(header.begin(), exact));
  }

  const std::string lowered_name = lower_string(name);
  for (std::size_t i = 0; i < header.size(); i++) {
    if (lower_string(header[i]) == lowered_name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string token_at(const std::vector<std::string>& tokens, int index, const std::string& name, int line) {
  if (index < 0 || static_cast<std::size_t>(index) >= tokens.size() || tokens[index].empty()) {
    throw std::runtime_error("missing batch column '" + name + "' on line " + std::to_string(line));
  }
  return tokens[index];
}

double parse_batch_double(const std::vector<std::string>& tokens, int index, const std::string& name, int line) {
  double value = 0.0;
  const std::string token = token_at(tokens, index, name, line);
  if (!parse_token_double(token, value) || !std::isfinite(value)) {
    throw std::runtime_error("invalid batch column '" + name + "' on line " + std::to_string(line));
  }
  return value;
}

std::vector<BatchFrame> load_batch_frames(const Options& options) {
  std::ifstream input(options.batch_csv_path);
  if (!input) {
    throw std::runtime_error("failed to open batch CSV: " + options.batch_csv_path);
  }

  const std::string base_dir = options.base_dir.empty() ? directory_of(options.batch_csv_path) : options.base_dir;
  std::vector<std::string> header;
  std::vector<BatchFrame> frames;
  std::string line;
  int line_no = 0;

  while (std::getline(input, line)) {
    line_no++;
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped.front() == '#') {
      continue;
    }

    if (header.empty()) {
      header = split_csv_line(stripped);
      continue;
    }

    try {
      const std::vector<std::string> tokens = split_csv_line(stripped);
      const int path_index = find_column(header, options.path_column);
      const int stamp_index = find_column(header, options.stamp_column);
      std::vector<int> pose_indices;
      pose_indices.reserve(options.pose_columns.size());
      for (const auto& name : options.pose_columns) {
        pose_indices.push_back(find_column(header, name));
      }

      if (path_index < 0) {
        throw std::runtime_error("batch CSV is missing path column '" + options.path_column + "'");
      }
      if (stamp_index < 0) {
        throw std::runtime_error("batch CSV is missing stamp column '" + options.stamp_column + "'");
      }
      for (std::size_t i = 0; i < pose_indices.size(); i++) {
        if (pose_indices[i] < 0) {
          throw std::runtime_error("batch CSV is missing pose column '" + options.pose_columns[i] + "'");
        }
      }

      const std::string cloud_path = join_path(base_dir, token_at(tokens, path_index, options.path_column, line_no));
      const double stamp = parse_batch_double(tokens, stamp_index, options.stamp_column, line_no);
      const double tx = parse_batch_double(tokens, pose_indices[0], options.pose_columns[0], line_no);
      const double ty = parse_batch_double(tokens, pose_indices[1], options.pose_columns[1], line_no);
      const double tz = parse_batch_double(tokens, pose_indices[2], options.pose_columns[2], line_no);
      const double roll = parse_batch_double(tokens, pose_indices[3], options.pose_columns[3], line_no);
      const double pitch = parse_batch_double(tokens, pose_indices[4], options.pose_columns[4], line_no);
      const double yaw = parse_batch_double(tokens, pose_indices[5], options.pose_columns[5], line_no);
      frames.push_back({cloud_path, stamp, make_xyzrpy_pose(tx, ty, tz, roll, pitch, yaw), line_no});
    } catch (const std::exception& e) {
      if (!options.skip_invalid_rows) {
        throw;
      }
      if (!options.quiet) {
        std::cerr << "skip batch row line=" << line_no << " reason=" << e.what() << '\n';
      }
    }
  }

  if (header.empty()) {
    throw std::runtime_error("batch CSV has no header row: " + options.batch_csv_path);
  }
  if (frames.empty()) {
    throw std::runtime_error("batch CSV produced no valid frames: " + options.batch_csv_path);
  }
  return frames;
}

void write_csv_header(std::ostream& output, bool full_covariance) {
  output << std::setprecision(17);
  if (full_covariance) {
    output << "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,confidence\n";
  } else {
    output << "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_yy,cov_zz,confidence\n";
  }
}

void write_csv_rows(std::ostream& output, const std::vector<glil::PerceptionObservation>& observations, bool full_covariance) {
  output << std::setprecision(17);
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

std::ostream* open_output(const std::string& path, std::ofstream& file) {
  if (path.empty()) {
    return &std::cout;
  }
  file.open(path);
  if (!file) {
    throw std::runtime_error("failed to open output CSV: " + path);
  }
  return &file;
}

void run_single(const Options& options, std::ostream& output) {
  const std::vector<gtsam::Point3> points = load_points(options.input_path, options.format);
  const auto result = glil::extract_cloud_landmark_observations(points, options.pose_world_sensor, options.stamp, options.extractor);
  write_csv_header(output, options.full_covariance);
  write_csv_rows(output, result.observations, options.full_covariance);

  if (!options.quiet) {
    std::cerr << "loaded_points=" << points.size() << " finite_points=" << result.finite_points << " range_rejected=" << result.range_rejected
              << " voxel_candidates=" << result.voxel_candidates << " rejected_min_points=" << result.rejected_min_points
              << " observations=" << result.observations.size() << '\n';
  }
}

void run_batch(const Options& options, std::ostream& output) {
  const std::vector<BatchFrame> frames = load_batch_frames(options);
  BatchSummary summary;
  write_csv_header(output, options.full_covariance);

  for (const auto& frame : frames) {
    try {
      const std::vector<gtsam::Point3> points = load_points(frame.cloud_path, options.format);
      const auto result = glil::extract_cloud_landmark_observations(points, frame.pose_world_sensor, frame.stamp, options.extractor);
      write_csv_rows(output, result.observations, options.full_covariance);

      summary.frames++;
      summary.loaded_points += points.size();
      summary.finite_points += result.finite_points;
      summary.range_rejected += result.range_rejected;
      summary.voxel_candidates += result.voxel_candidates;
      summary.rejected_min_points += result.rejected_min_points;
      summary.observations += result.observations.size();
    } catch (const std::exception& e) {
      summary.failed_frames++;
      if (!options.skip_invalid_rows) {
        throw;
      }
      if (!options.quiet) {
        std::cerr << "skip batch cloud line=" << frame.line << " path=" << frame.cloud_path << " reason=" << e.what() << '\n';
      }
    }
  }

  if (!options.quiet) {
    std::cerr << "frames=" << summary.frames << " failed_frames=" << summary.failed_frames << " loaded_points=" << summary.loaded_points
              << " finite_points=" << summary.finite_points << " range_rejected=" << summary.range_rejected
              << " voxel_candidates=" << summary.voxel_candidates << " rejected_min_points=" << summary.rejected_min_points
              << " observations=" << summary.observations << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::ofstream file;
    std::ostream* output = open_output(options.output_path, file);

    if (!options.batch_csv_path.empty()) {
      run_batch(options, *output);
    } else {
      run_single(options, *output);
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
