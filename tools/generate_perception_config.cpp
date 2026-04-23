// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

struct Options {
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
  bool quiet = false;
};

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " --config-root DIR --csv cloud_landmarks.csv [options]\n"
            << "  --config-root DIR       Config root to write/update\n"
            << "  --csv PATH              Perception observation CSV path for config_perception.json\n"
            << "  --config-json NAME      Root config filename (default: config.json)\n"
            << "  --config-ros NAME       ROS config filename (default: config_ros.json)\n"
            << "  --config-perception NAME  Perception config filename (default: config_perception.json)\n"
            << "  --module-name NAME      Extension module name (default: libperception_csv_injector.so)\n"
            << "  --time-tolerance SEC    CSV/submap timestamp tolerance (default: 0.05)\n"
            << "  --min-confidence V      Minimum observation confidence (default: 0.5)\n"
            << "  --min-sigma V           Minimum covariance sigma floor (default: 0.001)\n"
            << "  --min-noise-confidence V  Confidence floor for noise scaling (default: 0.001)\n"
            << "  --robust-loss NAME      NONE|HUBER|CAUCHY|TUKEY (default: HUBER)\n"
            << "  --robust-loss-width V   Robust loss width (default: 1.5)\n"
            << "  --allowed-class-ids CSV Comma-separated allow list (default: cloud_landmark)\n"
            << "  --rejected-class-ids CSV  Comma-separated reject list for dynamic classes\n"
            << "  --landmark-symbol C     GTSAM symbol prefix for landmarks (default: l)\n"
            << "  --skip-invalid-rows     Skip invalid observation CSV rows in injector\n"
            << "  --fail-on-load-error    Disable injector if CSV load has errors\n"
            << "  --no-consume-once       Allow an observation to match multiple submaps\n"
            << "  --no-skip-header        Treat the first CSV row as data\n"
            << "  --no-update-ros         Do not update config_ros.json extension_modules\n"
            << "  --no-update-config-json Do not update config.json links\n"
            << "  --quiet                 Only print errors\n";
}

bool path_exists(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0;
}

bool is_directory(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
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

void create_directories(const std::string& path) {
  if (path.empty() || is_directory(path)) {
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
      if (!is_directory(current) && mkdir(current.c_str(), 0775) != 0 && !is_directory(current)) {
        throw std::runtime_error("failed to create directory: " + current);
      }
    }
    if (slash == std::string::npos) {
      break;
    }
    pos = slash + 1;
  }
}

char* parse_end(const char* value) {
  return const_cast<char*>(value) + std::char_traits<char>::length(value);
}

double parse_double(const char* value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || end != parse_end(value)) {
    throw std::runtime_error(std::string("invalid number for ") + name + ": " + value);
  }
  return parsed;
}

std::string trim(std::string value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c) != 0; });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::vector<std::string> split_comma_list(const std::string& value) {
  std::vector<std::string> items;
  std::string item;
  std::istringstream stream(value);
  while (std::getline(stream, item, ',')) {
    item = trim(item);
    if (!item.empty()) {
      items.push_back(item);
    }
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
    } else if (arg == "--config-root") {
      options.config_root = next("--config-root");
    } else if (arg == "--csv") {
      options.csv_path = next("--csv");
    } else if (arg == "--config-json") {
      options.config_json = next("--config-json");
    } else if (arg == "--config-ros") {
      options.config_ros = next("--config-ros");
    } else if (arg == "--config-perception") {
      options.config_perception = next("--config-perception");
    } else if (arg == "--module-name") {
      options.module_name = next("--module-name");
    } else if (arg == "--time-tolerance") {
      options.time_tolerance = parse_double(next("--time-tolerance"), "--time-tolerance");
    } else if (arg == "--min-confidence") {
      options.min_confidence = parse_double(next("--min-confidence"), "--min-confidence");
    } else if (arg == "--min-sigma") {
      options.min_sigma = parse_double(next("--min-sigma"), "--min-sigma");
    } else if (arg == "--min-noise-confidence") {
      options.min_noise_confidence = parse_double(next("--min-noise-confidence"), "--min-noise-confidence");
    } else if (arg == "--robust-loss") {
      options.robust_loss = next("--robust-loss");
    } else if (arg == "--robust-loss-width") {
      options.robust_loss_width = parse_double(next("--robust-loss-width"), "--robust-loss-width");
    } else if (arg == "--allowed-class-ids") {
      options.allowed_class_ids = split_comma_list(next("--allowed-class-ids"));
    } else if (arg == "--rejected-class-ids") {
      options.rejected_class_ids = split_comma_list(next("--rejected-class-ids"));
    } else if (arg == "--landmark-symbol") {
      options.landmark_symbol = next("--landmark-symbol");
    } else if (arg == "--skip-invalid-rows") {
      options.skip_invalid_rows = true;
    } else if (arg == "--fail-on-load-error") {
      options.fail_on_load_error = true;
    } else if (arg == "--no-consume-once") {
      options.consume_once = false;
    } else if (arg == "--no-skip-header") {
      options.skip_header = false;
    } else if (arg == "--no-update-ros") {
      options.update_ros = false;
    } else if (arg == "--no-update-config-json") {
      options.update_config_json = false;
    } else if (arg == "--quiet") {
      options.quiet = true;
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

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
  return options;
}

nlohmann::json load_json_or_object(const std::string& path) {
  if (!path_exists(path)) {
    return nlohmann::json::object();
  }

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open JSON: " + path);
  }
  return nlohmann::json::parse(input, nullptr, true, true);
}

void write_json(const std::string& path, const nlohmann::json& json) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write JSON: " + path);
  }
  output << std::setw(2) << json << '\n';
}

nlohmann::json make_perception_injector_config(const Options& options) {
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

void update_perception_config(const Options& options, const std::string& perception_path) {
  nlohmann::json config = load_json_or_object(perception_path);
  if (!config.is_object()) {
    throw std::runtime_error("perception config must be a JSON object: " + perception_path);
  }
  config["perception_csv_injector"] = make_perception_injector_config(options);
  write_json(perception_path, config);
}

void update_config_json(const Options& options, const std::string& config_json_path) {
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

void update_ros_config(const Options& options, const std::string& ros_path) {
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

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    create_directories(options.config_root);

    const std::string perception_path = join_path(options.config_root, options.config_perception);
    update_perception_config(options, perception_path);

    if (options.update_config_json) {
      update_config_json(options, join_path(options.config_root, options.config_json));
    }
    if (options.update_ros) {
      update_ros_config(options, join_path(options.config_root, options.config_ros));
    }

    if (!options.quiet) {
      std::cerr << "wrote " << perception_path;
      if (options.update_config_json) {
        std::cerr << " updated " << join_path(options.config_root, options.config_json);
      }
      if (options.update_ros) {
        std::cerr << " updated " << join_path(options.config_root, options.config_ros);
      }
      std::cerr << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
