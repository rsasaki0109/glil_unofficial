// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
  std::vector<std::string> report_specs;
  std::string format = "markdown";
  std::string output_path;
  std::size_t min_accepted = 1;
  double min_accepted_match_rate = 0.0;
};

struct ReportInput {
  std::string label;
  std::string path;
};

struct ReportSummary {
  std::string label;
  std::string path;
  std::map<std::string, std::string> metrics;
};

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " --report LABEL=perception_report.csv [options]\n"
            << "  --report SPEC              Report CSV from glil_perception_factor_report --format csv\n"
            << "                             SPEC can be PATH or LABEL=PATH; repeat for multiple runs\n"
            << "  --format F                 markdown|csv (default: markdown)\n"
            << "  --output PATH              Write summary to file instead of stdout\n"
            << "  --min-accepted N           Minimum accepted observations for PASS (default: 1)\n"
            << "  --min-accepted-match-rate P Minimum accepted match rate percent when present (default: 0)\n";
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

std::string basename_without_extension(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
  const auto dot = name.find_last_of('.');
  if (dot != std::string::npos && dot != 0) {
    name = name.substr(0, dot);
  }
  return name.empty() ? std::string("report") : name;
}

ReportInput parse_report_spec(const std::string& spec) {
  const auto equal = spec.find('=');
  if (equal != std::string::npos && equal > 0) {
    const std::string label = trim(spec.substr(0, equal));
    const std::string path = trim(spec.substr(equal + 1));
    if (label.empty() || path.empty()) {
      throw std::runtime_error("invalid --report spec: " + spec);
    }
    return {label, path};
  }
  return {basename_without_extension(spec), spec};
}

std::size_t parse_size_arg(const char* value, const char* name) {
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

double parse_double_arg(const char* value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    throw std::runtime_error(std::string("invalid number for ") + name + ": " + value);
  }
  return parsed;
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
    } else if (arg == "--report") {
      options.report_specs.push_back(next("--report"));
    } else if (arg == "--format") {
      options.format = lower_string(next("--format"));
    } else if (arg == "--output") {
      options.output_path = next("--output");
    } else if (arg == "--min-accepted") {
      options.min_accepted = parse_size_arg(next("--min-accepted"), "--min-accepted");
    } else if (arg == "--min-accepted-match-rate") {
      options.min_accepted_match_rate = parse_double_arg(next("--min-accepted-match-rate"), "--min-accepted-match-rate");
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  if (options.report_specs.empty()) {
    throw std::runtime_error("at least one --report is required");
  }
  if (options.format != "markdown" && options.format != "csv") {
    throw std::runtime_error("--format must be markdown or csv");
  }
  if (options.min_accepted_match_rate < 0.0 || options.min_accepted_match_rate > 100.0) {
    throw std::runtime_error("--min-accepted-match-rate must be in [0, 100]");
  }
  return options;
}

ReportSummary load_report(const ReportInput& input) {
  std::ifstream stream(input.path);
  if (!stream) {
    throw std::runtime_error("failed to open report: " + input.path);
  }

  ReportSummary summary;
  summary.label = input.label;
  summary.path = input.path;

  std::string line;
  int line_number = 0;
  while (std::getline(stream, line)) {
    line_number++;
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped.front() == '#') {
      continue;
    }
    const auto tokens = split_csv_line(stripped);
    if (tokens.empty()) {
      continue;
    }
    if (tokens[0] == "section") {
      continue;
    }
    if (tokens[0] == "metric") {
      if (tokens.size() < 3) {
        throw std::runtime_error(input.path + ": metric row has fewer than 3 columns at line " + std::to_string(line_number));
      }
      summary.metrics[tokens[1]] = tokens[2];
    }
  }

  if (summary.metrics.empty()) {
    throw std::runtime_error("report has no metric rows: " + input.path);
  }
  return summary;
}

std::string metric(const ReportSummary& report, const std::string& key, const std::string& fallback = "NA") {
  const auto found = report.metrics.find(key);
  if (found == report.metrics.end() || found->second.empty()) {
    return fallback;
  }
  return found->second;
}

std::size_t metric_size(const ReportSummary& report, const std::string& key, std::size_t fallback = 0) {
  const std::string value = metric(report, key, "");
  if (value.empty() || value == "NA") {
    return fallback;
  }
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
  if (end == value.c_str()) {
    return fallback;
  }
  return static_cast<std::size_t>(parsed);
}

double metric_percent(const ReportSummary& report, const std::string& key, double fallback = -1.0) {
  std::string value = metric(report, key, "");
  if (value.empty() || value == "NA") {
    return fallback;
  }
  if (!value.empty() && value.back() == '%') {
    value.pop_back();
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str()) {
    return fallback;
  }
  return parsed;
}

std::string readiness_status(const ReportSummary& report, const Options& options) {
  const std::string injectable = lower_string(metric(report, "injectable", "no"));
  const std::size_t accepted = metric_size(report, "accepted_observations");
  const double accepted_match_rate = metric_percent(report, "accepted_match_rate", -1.0);
  const bool match_rate_ok = accepted_match_rate < 0.0 || accepted_match_rate >= options.min_accepted_match_rate;

  if (injectable == "yes" && accepted >= options.min_accepted && match_rate_ok) {
    return "PASS";
  }
  if (accepted > 0) {
    return "WARN";
  }
  return "FAIL";
}

std::string csv_escape(const std::string& value) {
  const bool needs_quotes = value.find_first_of(",\n\r\"") != std::string::npos;
  if (!needs_quotes) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(c);
    }
  }
  escaped.push_back('"');
  return escaped;
}

std::string markdown_escape(std::string value) {
  std::replace(value.begin(), value.end(), '\n', ' ');
  std::replace(value.begin(), value.end(), '\r', ' ');
  std::string escaped;
  escaped.reserve(value.size());
  for (const char c : value) {
    if (c == '|') {
      escaped += "\\|";
    } else {
      escaped.push_back(c);
    }
  }
  return escaped;
}

const std::vector<std::string>& summary_keys() {
  static const std::vector<std::string> keys = {
    "observations",
    "accepted_observations",
    "accepted_rate",
    "rejected_low_confidence",
    "rejected_class",
    "unique_landmarks",
    "accepted_unique_landmarks",
    "matched_accepted_observations",
    "accepted_match_rate",
    "confidence_median",
    "factor_sigma_median",
    "load_warnings",
    "warnings_duplicate_stamp_landmark",
    "warnings_landmark_class_collision",
    "warnings_degenerate_covariance",
    "warnings_non_positive_definite_covariance",
  };
  return keys;
}

void write_markdown(std::ostream& output, const std::vector<ReportSummary>& reports, const Options& options) {
  output << "# Perception Report Summary\n\n";
  output << "| run | status | injectable | observations | accepted | accepted rate | rejected low confidence | rejected class | unique landmarks | accepted landmarks | matched accepted | accepted match rate | confidence median | factor sigma median | load warnings | warn dup stamp/lid | warn class collision | warn degenerate cov | warn non-PD cov | source CSV |\n";
  output << "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n";
  for (const auto& report : reports) {
    output << "| `" << markdown_escape(report.label) << "` "
           << "| " << readiness_status(report, options) << ' '
           << "| " << markdown_escape(metric(report, "injectable")) << ' '
           << "| " << markdown_escape(metric(report, "observations")) << ' '
           << "| " << markdown_escape(metric(report, "accepted_observations")) << ' '
           << "| " << markdown_escape(metric(report, "accepted_rate")) << ' '
           << "| " << markdown_escape(metric(report, "rejected_low_confidence")) << ' '
           << "| " << markdown_escape(metric(report, "rejected_class")) << ' '
           << "| " << markdown_escape(metric(report, "unique_landmarks")) << ' '
           << "| " << markdown_escape(metric(report, "accepted_unique_landmarks")) << ' '
           << "| " << markdown_escape(metric(report, "matched_accepted_observations")) << ' '
           << "| " << markdown_escape(metric(report, "accepted_match_rate")) << ' '
           << "| " << markdown_escape(metric(report, "confidence_median")) << ' '
           << "| " << markdown_escape(metric(report, "factor_sigma_median")) << ' '
           << "| " << markdown_escape(metric(report, "load_warnings")) << ' '
           << "| " << markdown_escape(metric(report, "warnings_duplicate_stamp_landmark")) << ' '
           << "| " << markdown_escape(metric(report, "warnings_landmark_class_collision")) << ' '
           << "| " << markdown_escape(metric(report, "warnings_degenerate_covariance")) << ' '
           << "| " << markdown_escape(metric(report, "warnings_non_positive_definite_covariance")) << ' '
           << "| `" << markdown_escape(metric(report, "csv_path")) << "` |\n";
  }
  output << "\nStatus is PASS when `injectable=yes`, accepted observations meet `--min-accepted`, and the accepted match rate meets `--min-accepted-match-rate` when that metric is present.\n";
}

void write_csv(std::ostream& output, const std::vector<ReportSummary>& reports, const Options& options) {
  output << "run,status,injectable";
  for (const auto& key : summary_keys()) {
    output << ',' << key;
  }
  output << ",csv_path,report_path\n";

  for (const auto& report : reports) {
    output << csv_escape(report.label) << ',' << readiness_status(report, options) << ',' << csv_escape(metric(report, "injectable"));
    for (const auto& key : summary_keys()) {
      output << ',' << csv_escape(metric(report, key));
    }
    output << ',' << csv_escape(metric(report, "csv_path")) << ',' << csv_escape(report.path) << '\n';
  }
}

std::ostream& output_stream(const std::string& path, std::ofstream& file) {
  if (path.empty()) {
    return std::cout;
  }
  file.open(path);
  if (!file) {
    throw std::runtime_error("failed to open summary output: " + path);
  }
  return file;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::vector<ReportSummary> reports;
    reports.reserve(options.report_specs.size());
    for (const auto& spec : options.report_specs) {
      reports.push_back(load_report(parse_report_spec(spec)));
    }

    std::ofstream file;
    std::ostream& output = output_stream(options.output_path, file);
    if (options.format == "markdown") {
      write_markdown(output, reports, options);
    } else {
      write_csv(output, reports, options);
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
}
