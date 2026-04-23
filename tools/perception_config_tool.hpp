// SPDX-License-Identifier: MIT
#pragma once

#include <cctype>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <nlohmann/json.hpp>

namespace glil::tools {

struct PerceptionConfigOptions {
  std::string config_root;
  std::string csv_path;
  std::string config_json = "config.json";
  std::string config_ros = "config_ros.json";
  std::string config_perception = "config_perception.json";
  std::string module_name = "libperception_csv_injector.so";
  double time_tolerance = 0.05;
  double min_confidence = 0.5;
  double min_sigma = 0.001;
  double min_noise_confidence = 0.001;
  std::string robust_loss = "HUBER";
  double robust_loss_width = 1.5;
  std::string landmark_symbol = "l";
  std::vector<std::string> allowed_class_ids = {"cloud_landmark"};
  std::vector<std::string> rejected_class_ids = {"car", "person", "truck", "bus", "bicycle", "motorcycle"};
  bool consume_once = true;
  bool skip_header = true;
  bool skip_invalid_rows = false;
  bool fail_on_load_error = false;
  bool initialize_missing_landmarks = true;
  bool update_ros = true;
  bool update_config_json = true;
};

inline bool config_path_exists(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0;
}

inline bool is_config_directory(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

inline bool is_config_absolute_path(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  if (path.front() == '/' || path.front() == '\\') {
    return true;
  }
  return path.size() >= 2 && std::isalpha(static_cast<unsigned char>(path[0])) != 0 && path[1] == ':';
}

inline std::string join_config_path(const std::string& base, const std::string& path) {
  if (path.empty() || base.empty() || is_config_absolute_path(path)) {
    return path;
  }
  const char last = base.back();
  if (last == '/' || last == '\\') {
    return base + path;
  }
  return base + "/" + path;
}

inline void create_config_directories(const std::string& path) {
  if (path.empty() || is_config_directory(path)) {
    return;
  }

  std::string current;
  std::size_t pos = 0;
  if (path.front() == '/') {
    current = "/";
    pos = 1;
  }

  while (pos <= path.size()) {
    const std::size_t slash = path.find('/', pos);
    const std::string part = path.substr(pos, slash == std::string::npos ? std::string::npos : slash - pos);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/') {
        current += '/';
      }
      current += part;
      if (!is_config_directory(current) && mkdir(current.c_str(), 0775) != 0 && !is_config_directory(current)) {
        throw std::runtime_error("failed to create directory: " + current);
      }
    }
    if (slash == std::string::npos) {
      break;
    }
    pos = slash + 1;
  }
}

inline void validate_perception_config_options(const PerceptionConfigOptions& options) {
  if (options.config_root.empty()) {
    throw std::runtime_error("--config-root is required");
  }
  if (options.csv_path.empty()) {
    throw std::runtime_error("--csv is required");
  }
  if (options.landmark_symbol.empty()) {
    throw std::runtime_error("--landmark-symbol cannot be empty");
  }
  if (options.time_tolerance < 0.0) {
    throw std::runtime_error("--time-tolerance must be non-negative");
  }
  if (options.min_confidence < 0.0 || options.min_confidence > 1.0) {
    throw std::runtime_error("--min-confidence must be in [0, 1]");
  }
  if (options.min_sigma <= 0.0) {
    throw std::runtime_error("--min-sigma must be positive");
  }
  if (options.min_noise_confidence <= 0.0) {
    throw std::runtime_error("--min-noise-confidence must be positive");
  }
}

inline nlohmann::json load_json_or_object(const std::string& path) {
  if (!config_path_exists(path)) {
    return nlohmann::json::object();
  }

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open JSON: " + path);
  }
  return nlohmann::json::parse(input, nullptr, true, true);
}

inline void write_json(const std::string& path, const nlohmann::json& json) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write JSON: " + path);
  }
  output << std::setw(2) << json << '\n';
}

inline nlohmann::json make_perception_injector_config(const PerceptionConfigOptions& options) {
  nlohmann::json injector;
  injector["enabled"] = true;
  injector["csv_path"] = options.csv_path;
  injector["time_tolerance"] = options.time_tolerance;
  injector["consume_once"] = options.consume_once;
  injector["skip_header"] = options.skip_header;
  injector["skip_invalid_rows"] = options.skip_invalid_rows;
  injector["fail_on_load_error"] = options.fail_on_load_error;
  injector["min_confidence"] = options.min_confidence;
  injector["min_sigma"] = options.min_sigma;
  injector["min_noise_confidence"] = options.min_noise_confidence;
  injector["robust_loss"] = options.robust_loss;
  injector["robust_loss_width"] = options.robust_loss_width;
  injector["landmark_symbol"] = options.landmark_symbol.substr(0, 1);
  injector["initialize_missing_landmarks"] = options.initialize_missing_landmarks;
  injector["allowed_class_ids"] = options.allowed_class_ids;
  injector["rejected_class_ids"] = options.rejected_class_ids;

  return injector;
}

inline void update_perception_config(const PerceptionConfigOptions& options, const std::string& perception_path) {
  nlohmann::json config = load_json_or_object(perception_path);
  if (!config.is_object()) {
    throw std::runtime_error("perception config must be a JSON object: " + perception_path);
  }
  config["perception_csv_injector"] = make_perception_injector_config(options);
  write_json(perception_path, config);
}

inline void update_config_json(const PerceptionConfigOptions& options, const std::string& config_json_path) {
  nlohmann::json config = load_json_or_object(config_json_path);
  if (!config.is_object()) {
    throw std::runtime_error("root config must be a JSON object: " + config_json_path);
  }
  if (!config.contains("global") || !config["global"].is_object()) {
    config["global"] = nlohmann::json::object();
  }
  config["global"]["config_perception"] = options.config_perception;
  config["global"]["config_ros"] = options.config_ros;
  write_json(config_json_path, config);
}

inline void update_ros_config(const PerceptionConfigOptions& options, const std::string& ros_path) {
  nlohmann::json config = load_json_or_object(ros_path);
  if (!config.is_object()) {
    throw std::runtime_error("ROS config must be a JSON object: " + ros_path);
  }
  if (!config.contains("glil_ros") || !config["glil_ros"].is_object()) {
    config["glil_ros"] = nlohmann::json::object();
  }

  nlohmann::json& modules = config["glil_ros"]["extension_modules"];
  if (!modules.is_array()) {
    modules = nlohmann::json::array();
  }

  bool found = false;
  for (const auto& module : modules) {
    if (module.is_string() && module.get<std::string>() == options.module_name) {
      found = true;
      break;
    }
  }
  if (!found) {
    modules.push_back(options.module_name);
  }
  write_json(ros_path, config);
}

inline void write_perception_config_root(const PerceptionConfigOptions& options) {
  validate_perception_config_options(options);
  create_config_directories(options.config_root);

  const std::string perception_path = join_config_path(options.config_root, options.config_perception);
  update_perception_config(options, perception_path);

  if (options.update_config_json) {
    update_config_json(options, join_config_path(options.config_root, options.config_json));
  }
  if (options.update_ros) {
    update_ros_config(options, join_config_path(options.config_root, options.config_ros));
  }
}

}  // namespace glil::tools
