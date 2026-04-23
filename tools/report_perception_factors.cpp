// SPDX-License-Identifier: MIT
#include "perception_config_tool.hpp"

#include <glil/perception/perception_observation_io.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

struct Options {
  std::string csv_path;
  std::string config_root;
  std::string config_perception = "config_perception.json";
  std::string submap_stamps_path;
  std::string stamp_column = "stamp";
  std::string output_path;
  std::string format = "markdown";
  bool config_requested = false;
  bool skip_header = true;
  bool skip_invalid_rows = true;
  bool skip_header_explicit = false;
  bool skip_invalid_rows_explicit = false;
  double min_confidence = 0.5;
  double min_sigma = 0.001;
  double min_noise_confidence = 0.001;
  double time_tolerance = 0.05;
  std::string robust_loss = "NONE";
  double robust_loss_width = 1.0;
  std::vector<std::string> allowed_class_ids;
  std::vector<std::string> rejected_class_ids = {"car", "person", "truck", "bus", "bicycle", "motorcycle"};
  bool min_confidence_explicit = false;
  bool min_sigma_explicit = false;
  bool min_noise_confidence_explicit = false;
  bool time_tolerance_explicit = false;
  bool allowed_class_ids_explicit = false;
  bool rejected_class_ids_explicit = false;
};

struct ClassStats {
  std::size_t total = 0;
  std::size_t accepted = 0;
  std::size_t rejected_low_confidence = 0;
  std::size_t rejected_class = 0;
  std::unordered_set<std::uint64_t> landmarks;
};

struct MatchStats {
  bool enabled = false;
  std::size_t submap_stamps = 0;
  std::size_t matched_observations = 0;
  std::size_t matched_accepted_observations = 0;
  std::set<std::size_t> covered_submaps;
  std::set<std::size_t> accepted_covered_submaps;
};

struct ReportStats {
  std::size_t total_observations = 0;
  std::size_t load_errors = 0;
  std::size_t accepted_observations = 0;
  std::size_t rejected_low_confidence = 0;
  std::size_t rejected_class = 0;
  std::unordered_set<std::uint64_t> landmarks;
  std::unordered_set<std::uint64_t> accepted_landmarks;
  std::map<std::string, ClassStats> classes;
  std::vector<double> stamps;
  std::vector<double> confidences;
  std::vector<double> covariance_diagonal;
  std::vector<double> factor_sigmas;
  MatchStats matches;
};

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " --csv observations.csv [options]\n"
            << "  --csv PATH              Perception observation CSV to inspect\n"
            << "  --config-root DIR       Read DIR/config_perception.json and resolve csv_path from DIR\n"
            << "  --config-perception P   Config perception filename or path (default: config_perception.json)\n"
            << "  --submap-stamps PATH    CSV/list of submap timestamps for time_tolerance matching\n"
            << "  --stamp-column NAME     Timestamp column in --submap-stamps (default: stamp)\n"
            << "  --time-tolerance SEC    Override injector timestamp tolerance\n"
            << "  --min-confidence V      Override injector minimum confidence\n"
            << "  --min-sigma V           Override noise sigma floor\n"
            << "  --min-noise-confidence V Override noise confidence floor\n"
            << "  --allowed-class-ids CSV Override injector class allow list\n"
            << "  --rejected-class-ids CSV Override injector class reject list\n"
            << "  --skip-invalid-rows     Continue after invalid observation rows\n"
            << "  --strict                Stop loading at the first invalid observation row\n"
            << "  --no-skip-header        Treat the first CSV row as data\n"
            << "  --format F              markdown|csv (default: markdown)\n"
            << "  --output PATH           Write report to file instead of stdout\n";
}

std::string trim(std::string value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c) != 0; });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::string lower_string(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

double parse_double_arg(const char* value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || !std::isfinite(parsed)) {
    throw std::runtime_error(std::string("invalid number for ") + name + ": " + value);
  }
  return parsed;
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
  return end != token.c_str() && *end == '\0' && std::isfinite(value);
}

std::string directory_of(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  if (slash == std::string::npos) {
    return "";
  }
  return path.substr(0, slash);
}

std::string resolve_relative_to(const std::string& base_dir, const std::string& path) {
  return glil::tools::join_config_path(base_dir, path);
}

template <typename T>
T json_value(const nlohmann::json& json, const char* key, const T& fallback) {
  if (!json.contains(key) || json[key].is_null()) {
    return fallback;
  }
  return json[key].get<T>();
}

void apply_config(Options& options) {
  if (!options.config_requested) {
    return;
  }

  const std::string config_path = options.config_root.empty() ? options.config_perception : glil::tools::join_config_path(options.config_root, options.config_perception);
  if (!glil::tools::config_path_exists(config_path)) {
    throw std::runtime_error("config perception file does not exist: " + config_path);
  }

  const auto config = glil::tools::load_json_or_object(config_path);
  if (!config.is_object() || !config.contains("perception_csv_injector") || !config["perception_csv_injector"].is_object()) {
    throw std::runtime_error("config has no perception_csv_injector object: " + config_path);
  }

  const auto& injector = config["perception_csv_injector"];
  if (options.csv_path.empty()) {
    const std::string configured_csv = json_value<std::string>(injector, "csv_path", "");
    if (!configured_csv.empty()) {
      const std::string base_dir = options.config_root.empty() ? directory_of(config_path) : options.config_root;
      options.csv_path = resolve_relative_to(base_dir, configured_csv);
    }
  }

  if (!options.skip_header_explicit) {
    options.skip_header = json_value<bool>(injector, "skip_header", options.skip_header);
  }
  if (!options.skip_invalid_rows_explicit) {
    options.skip_invalid_rows = json_value<bool>(injector, "skip_invalid_rows", options.skip_invalid_rows);
  }
  if (!options.min_confidence_explicit) {
    options.min_confidence = json_value<double>(injector, "min_confidence", options.min_confidence);
  }
  if (!options.min_sigma_explicit) {
    options.min_sigma = json_value<double>(injector, "min_sigma", options.min_sigma);
  }
  if (!options.min_noise_confidence_explicit) {
    options.min_noise_confidence = json_value<double>(injector, "min_noise_confidence", options.min_noise_confidence);
  }
  if (!options.time_tolerance_explicit) {
    options.time_tolerance = json_value<double>(injector, "time_tolerance", options.time_tolerance);
  }
  if (!options.allowed_class_ids_explicit) {
    options.allowed_class_ids = json_value<std::vector<std::string>>(injector, "allowed_class_ids", options.allowed_class_ids);
  }
  if (!options.rejected_class_ids_explicit) {
    options.rejected_class_ids = json_value<std::vector<std::string>>(injector, "rejected_class_ids", options.rejected_class_ids);
  }
  options.robust_loss = json_value<std::string>(injector, "robust_loss", options.robust_loss);
  options.robust_loss_width = json_value<double>(injector, "robust_loss_width", options.robust_loss_width);
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
    } else if (arg == "--csv") {
      options.csv_path = next("--csv");
    } else if (arg == "--config-root") {
      options.config_root = next("--config-root");
      options.config_requested = true;
    } else if (arg == "--config-perception") {
      options.config_perception = next("--config-perception");
      options.config_requested = true;
    } else if (arg == "--submap-stamps") {
      options.submap_stamps_path = next("--submap-stamps");
    } else if (arg == "--stamp-column") {
      options.stamp_column = next("--stamp-column");
    } else if (arg == "--time-tolerance") {
      options.time_tolerance = parse_double_arg(next("--time-tolerance"), "--time-tolerance");
      options.time_tolerance_explicit = true;
    } else if (arg == "--min-confidence") {
      options.min_confidence = parse_double_arg(next("--min-confidence"), "--min-confidence");
      options.min_confidence_explicit = true;
    } else if (arg == "--min-sigma") {
      options.min_sigma = parse_double_arg(next("--min-sigma"), "--min-sigma");
      options.min_sigma_explicit = true;
    } else if (arg == "--min-noise-confidence") {
      options.min_noise_confidence = parse_double_arg(next("--min-noise-confidence"), "--min-noise-confidence");
      options.min_noise_confidence_explicit = true;
    } else if (arg == "--allowed-class-ids") {
      options.allowed_class_ids = split_comma_list(next("--allowed-class-ids"));
      options.allowed_class_ids_explicit = true;
    } else if (arg == "--rejected-class-ids") {
      options.rejected_class_ids = split_comma_list(next("--rejected-class-ids"));
      options.rejected_class_ids_explicit = true;
    } else if (arg == "--skip-invalid-rows") {
      options.skip_invalid_rows = true;
      options.skip_invalid_rows_explicit = true;
    } else if (arg == "--strict") {
      options.skip_invalid_rows = false;
      options.skip_invalid_rows_explicit = true;
    } else if (arg == "--no-skip-header") {
      options.skip_header = false;
      options.skip_header_explicit = true;
    } else if (arg == "--format") {
      options.format = lower_string(next("--format"));
    } else if (arg == "--output") {
      options.output_path = next("--output");
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  apply_config(options);

  if (options.csv_path.empty()) {
    throw std::runtime_error("--csv is required unless --config-root/--config-perception provides csv_path");
  }
  if (options.format != "markdown" && options.format != "csv") {
    throw std::runtime_error("--format must be markdown or csv");
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

std::vector<double> load_submap_stamps(const Options& options) {
  std::ifstream input(options.submap_stamps_path);
  if (!input) {
    throw std::runtime_error("failed to open submap stamps CSV: " + options.submap_stamps_path);
  }

  std::vector<double> stamps;
  std::vector<std::string> header;
  int stamp_index = 0;
  std::string line;
  int line_no = 0;
  while (std::getline(input, line)) {
    line_no++;
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped.front() == '#') {
      continue;
    }

    const auto tokens = split_csv_line(stripped);
    if (tokens.empty()) {
      continue;
    }

    double stamp = 0.0;
    if (header.empty() && !parse_token_double(tokens.front(), stamp)) {
      header = tokens;
      stamp_index = find_column(header, options.stamp_column);
      if (stamp_index < 0) {
        throw std::runtime_error("submap stamps CSV is missing stamp column '" + options.stamp_column + "'");
      }
      continue;
    }

    if (!header.empty()) {
      if (stamp_index >= static_cast<int>(tokens.size()) || !parse_token_double(tokens[stamp_index], stamp)) {
        throw std::runtime_error("invalid submap stamp on line " + std::to_string(line_no));
      }
    }
    stamps.push_back(stamp);
  }

  std::sort(stamps.begin(), stamps.end());
  return stamps;
}

bool class_allowed(const Options& options, const std::string& class_id) {
  if (!options.allowed_class_ids.empty() && std::find(options.allowed_class_ids.begin(), options.allowed_class_ids.end(), class_id) == options.allowed_class_ids.end()) {
    return false;
  }
  return std::find(options.rejected_class_ids.begin(), options.rejected_class_ids.end(), class_id) == options.rejected_class_ids.end();
}

double factor_sigma(double variance, double confidence, const Options& options) {
  const double safe_variance = std::max(variance, options.min_sigma * options.min_sigma);
  const double safe_confidence = std::max(confidence, options.min_noise_confidence);
  return std::max(std::sqrt(safe_variance / safe_confidence), options.min_sigma);
}

std::size_t nearest_stamp_index(const std::vector<double>& stamps, double stamp, double& nearest_delta) {
  nearest_delta = std::numeric_limits<double>::infinity();
  if (stamps.empty()) {
    return 0;
  }

  const auto it = std::lower_bound(stamps.begin(), stamps.end(), stamp);
  std::size_t best_index = 0;
  const auto consider = [&](std::vector<double>::const_iterator candidate) {
    const double delta = std::abs(*candidate - stamp);
    if (delta < nearest_delta) {
      nearest_delta = delta;
      best_index = static_cast<std::size_t>(std::distance(stamps.begin(), candidate));
    }
  };

  if (it != stamps.end()) {
    consider(it);
  }
  if (it != stamps.begin()) {
    consider(std::prev(it));
  }
  return best_index;
}

ReportStats build_report(const Options& options, const glil::PerceptionObservationCsvLoadResult& loaded, const std::vector<double>& submap_stamps) {
  ReportStats stats;
  stats.load_errors = loaded.errors.size();
  stats.matches.enabled = !submap_stamps.empty();
  stats.matches.submap_stamps = submap_stamps.size();

  for (const auto& obs : loaded.observations) {
    stats.total_observations++;
    stats.landmarks.insert(obs.landmark_id);
    stats.stamps.push_back(obs.stamp);
    stats.confidences.push_back(obs.confidence);

    ClassStats& cls = stats.classes[obs.class_id];
    cls.total++;
    cls.landmarks.insert(obs.landmark_id);

    for (int i = 0; i < 3; i++) {
      stats.covariance_diagonal.push_back(obs.covariance(i, i));
      stats.factor_sigmas.push_back(factor_sigma(obs.covariance(i, i), obs.confidence, options));
    }

    const bool low_confidence = obs.confidence < options.min_confidence;
    const bool accepted_class = class_allowed(options, obs.class_id);
    const bool accepted = !low_confidence && accepted_class;

    if (low_confidence) {
      stats.rejected_low_confidence++;
      cls.rejected_low_confidence++;
    }
    if (!accepted_class) {
      stats.rejected_class++;
      cls.rejected_class++;
    }
    if (accepted) {
      stats.accepted_observations++;
      stats.accepted_landmarks.insert(obs.landmark_id);
      cls.accepted++;
    }

    if (stats.matches.enabled) {
      double delta = 0.0;
      const std::size_t nearest = nearest_stamp_index(submap_stamps, obs.stamp, delta);
      if (delta <= options.time_tolerance) {
        stats.matches.matched_observations++;
        stats.matches.covered_submaps.insert(nearest);
        if (accepted) {
          stats.matches.matched_accepted_observations++;
          stats.matches.accepted_covered_submaps.insert(nearest);
        }
      }
    }
  }

  return stats;
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  if (values.size() % 2 == 0) {
    return 0.5 * (values[mid - 1] + values[mid]);
  }
  return values[mid];
}

double mean(const std::vector<double>& values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double min_value(const std::vector<double>& values) {
  return values.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::min_element(values.begin(), values.end());
}

double max_value(const std::vector<double>& values) {
  return values.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(values.begin(), values.end());
}

std::string format_double(double value) {
  if (!std::isfinite(value)) {
    return "NA";
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << value;
  return oss.str();
}

std::string format_rate(std::size_t numerator, std::size_t denominator) {
  if (denominator == 0) {
    return "NA";
  }
  return format_double(100.0 * static_cast<double>(numerator) / static_cast<double>(denominator)) + "%";
}

std::string yes_no(bool value) {
  return value ? "yes" : "no";
}

void write_markdown_report(std::ostream& output, const Options& options, const ReportStats& stats) {
  const bool injectable = stats.accepted_observations > 0 && (stats.load_errors == 0 || options.skip_invalid_rows) && (!stats.matches.enabled || stats.matches.matched_accepted_observations > 0);

  output << "# Perception Factor Report\n\n";
  output << "## Summary\n\n";
  output << "| Metric | Value |\n|---|---:|\n";
  output << "| CSV path | `" << options.csv_path << "` |\n";
  output << "| Injectable | " << yes_no(injectable) << " |\n";
  output << "| Observations | " << stats.total_observations << " |\n";
  output << "| Load errors | " << stats.load_errors << " |\n";
  output << "| Accepted observations | " << stats.accepted_observations << " |\n";
  output << "| Accepted rate | " << format_rate(stats.accepted_observations, stats.total_observations) << " |\n";
  output << "| Rejected low confidence | " << stats.rejected_low_confidence << " |\n";
  output << "| Rejected class | " << stats.rejected_class << " |\n";
  output << "| Unique landmarks | " << stats.landmarks.size() << " |\n";
  output << "| Accepted unique landmarks | " << stats.accepted_landmarks.size() << " |\n";
  output << "| Classes | " << stats.classes.size() << " |\n";
  output << "| Stamp min | " << format_double(min_value(stats.stamps)) << " |\n";
  output << "| Stamp max | " << format_double(max_value(stats.stamps)) << " |\n";
  output << "| Confidence min | " << format_double(min_value(stats.confidences)) << " |\n";
  output << "| Confidence median | " << format_double(median(stats.confidences)) << " |\n";
  output << "| Confidence mean | " << format_double(mean(stats.confidences)) << " |\n";
  output << "| Confidence max | " << format_double(max_value(stats.confidences)) << " |\n";
  output << "| Covariance diagonal median | " << format_double(median(stats.covariance_diagonal)) << " |\n";
  output << "| Factor sigma median | " << format_double(median(stats.factor_sigmas)) << " |\n";
  output << "| Min confidence | " << format_double(options.min_confidence) << " |\n";
  output << "| Time tolerance | " << format_double(options.time_tolerance) << " |\n";
  output << "| Robust loss | `" << options.robust_loss << "` |\n";
  output << "| Robust loss width | " << format_double(options.robust_loss_width) << " |\n";

  if (stats.matches.enabled) {
    output << "| Submap stamps | " << stats.matches.submap_stamps << " |\n";
    output << "| Matched observations | " << stats.matches.matched_observations << " |\n";
    output << "| Matched observation rate | " << format_rate(stats.matches.matched_observations, stats.total_observations) << " |\n";
    output << "| Matched accepted observations | " << stats.matches.matched_accepted_observations << " |\n";
    output << "| Accepted match rate | " << format_rate(stats.matches.matched_accepted_observations, stats.accepted_observations) << " |\n";
    output << "| Covered submaps | " << stats.matches.covered_submaps.size() << " |\n";
    output << "| Accepted covered submaps | " << stats.matches.accepted_covered_submaps.size() << " |\n";
  }

  output << "\n## Class Breakdown\n\n";
  output << "| class_id | total | accepted | rejected_low_confidence | rejected_class | unique_landmarks |\n";
  output << "|---|---:|---:|---:|---:|---:|\n";
  for (const auto& item : stats.classes) {
    output << "| `" << item.first << "` | " << item.second.total << " | " << item.second.accepted << " | " << item.second.rejected_low_confidence << " | "
           << item.second.rejected_class << " | " << item.second.landmarks.size() << " |\n";
  }
}

void write_csv_metric(std::ostream& output, const std::string& key, const std::string& value) {
  output << "metric," << key << ',' << value << '\n';
}

void write_csv_report(std::ostream& output, const Options& options, const ReportStats& stats) {
  const bool injectable = stats.accepted_observations > 0 && (stats.load_errors == 0 || options.skip_invalid_rows) && (!stats.matches.enabled || stats.matches.matched_accepted_observations > 0);
  output << "section,key,value\n";
  write_csv_metric(output, "csv_path", options.csv_path);
  write_csv_metric(output, "injectable", yes_no(injectable));
  write_csv_metric(output, "observations", std::to_string(stats.total_observations));
  write_csv_metric(output, "load_errors", std::to_string(stats.load_errors));
  write_csv_metric(output, "accepted_observations", std::to_string(stats.accepted_observations));
  write_csv_metric(output, "accepted_rate", format_rate(stats.accepted_observations, stats.total_observations));
  write_csv_metric(output, "rejected_low_confidence", std::to_string(stats.rejected_low_confidence));
  write_csv_metric(output, "rejected_class", std::to_string(stats.rejected_class));
  write_csv_metric(output, "unique_landmarks", std::to_string(stats.landmarks.size()));
  write_csv_metric(output, "accepted_unique_landmarks", std::to_string(stats.accepted_landmarks.size()));
  write_csv_metric(output, "confidence_median", format_double(median(stats.confidences)));
  write_csv_metric(output, "factor_sigma_median", format_double(median(stats.factor_sigmas)));
  if (stats.matches.enabled) {
    write_csv_metric(output, "matched_observations", std::to_string(stats.matches.matched_observations));
    write_csv_metric(output, "matched_observation_rate", format_rate(stats.matches.matched_observations, stats.total_observations));
    write_csv_metric(output, "matched_accepted_observations", std::to_string(stats.matches.matched_accepted_observations));
    write_csv_metric(output, "accepted_match_rate", format_rate(stats.matches.matched_accepted_observations, stats.accepted_observations));
  }

  output << "section,class_id,total,accepted,rejected_low_confidence,rejected_class,unique_landmarks\n";
  for (const auto& item : stats.classes) {
    output << "class," << item.first << ',' << item.second.total << ',' << item.second.accepted << ',' << item.second.rejected_low_confidence << ','
           << item.second.rejected_class << ',' << item.second.landmarks.size() << '\n';
  }
}

std::ostream* open_output(const std::string& path, std::ofstream& file) {
  if (path.empty()) {
    return &std::cout;
  }
  file.open(path);
  if (!file) {
    throw std::runtime_error("failed to open report output: " + path);
  }
  return &file;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);

    glil::PerceptionObservationCsvOptions csv_options;
    csv_options.skip_header = options.skip_header;
    csv_options.skip_invalid_rows = options.skip_invalid_rows;
    csv_options.min_confidence = 0.0;

    auto loaded = glil::load_perception_observations_csv(options.csv_path, csv_options);
    std::vector<double> submap_stamps;
    if (!options.submap_stamps_path.empty()) {
      submap_stamps = load_submap_stamps(options);
    }

    const ReportStats stats = build_report(options, loaded, submap_stamps);

    std::ofstream file;
    std::ostream* output = open_output(options.output_path, file);
    if (options.format == "markdown") {
      write_markdown_report(*output, options, stats);
    } else {
      write_csv_report(*output, options, stats);
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
