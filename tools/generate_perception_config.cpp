// SPDX-License-Identifier: MIT
#include "perception_config_tool.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
  glil::tools::PerceptionConfigOptions config;
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

double parse_double(const char* value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
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
      options.config.config_root = next("--config-root");
    } else if (arg == "--csv") {
      options.config.csv_path = next("--csv");
    } else if (arg == "--config-json") {
      options.config.config_json = next("--config-json");
    } else if (arg == "--config-ros") {
      options.config.config_ros = next("--config-ros");
    } else if (arg == "--config-perception") {
      options.config.config_perception = next("--config-perception");
    } else if (arg == "--module-name") {
      options.config.module_name = next("--module-name");
    } else if (arg == "--time-tolerance") {
      options.config.time_tolerance = parse_double(next("--time-tolerance"), "--time-tolerance");
    } else if (arg == "--min-confidence") {
      options.config.min_confidence = parse_double(next("--min-confidence"), "--min-confidence");
    } else if (arg == "--min-sigma") {
      options.config.min_sigma = parse_double(next("--min-sigma"), "--min-sigma");
    } else if (arg == "--min-noise-confidence") {
      options.config.min_noise_confidence = parse_double(next("--min-noise-confidence"), "--min-noise-confidence");
    } else if (arg == "--robust-loss") {
      options.config.robust_loss = next("--robust-loss");
    } else if (arg == "--robust-loss-width") {
      options.config.robust_loss_width = parse_double(next("--robust-loss-width"), "--robust-loss-width");
    } else if (arg == "--allowed-class-ids") {
      options.config.allowed_class_ids = split_comma_list(next("--allowed-class-ids"));
    } else if (arg == "--rejected-class-ids") {
      options.config.rejected_class_ids = split_comma_list(next("--rejected-class-ids"));
    } else if (arg == "--landmark-symbol") {
      options.config.landmark_symbol = next("--landmark-symbol");
    } else if (arg == "--skip-invalid-rows") {
      options.config.skip_invalid_rows = true;
    } else if (arg == "--fail-on-load-error") {
      options.config.fail_on_load_error = true;
    } else if (arg == "--no-consume-once") {
      options.config.consume_once = false;
    } else if (arg == "--no-skip-header") {
      options.config.skip_header = false;
    } else if (arg == "--no-update-ros") {
      options.config.update_ros = false;
    } else if (arg == "--no-update-config-json") {
      options.config.update_config_json = false;
    } else if (arg == "--quiet") {
      options.quiet = true;
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  glil::tools::validate_perception_config_options(options.config);
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    glil::tools::write_perception_config_root(options.config);

    if (!options.quiet) {
      const std::string perception_path = glil::tools::join_config_path(options.config.config_root, options.config.config_perception);
      std::cerr << "wrote " << perception_path;
      if (options.config.update_config_json) {
        std::cerr << " updated " << glil::tools::join_config_path(options.config.config_root, options.config.config_json);
      }
      if (options.config.update_ros) {
        std::cerr << " updated " << glil::tools::join_config_path(options.config.config_root, options.config.config_ros);
      }
      std::cerr << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
