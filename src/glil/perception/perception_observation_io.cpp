// SPDX-License-Identifier: MIT
#include <glil/perception/perception_observation_io.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Dense>

namespace glil {
namespace {

std::string trim(std::string value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c) != 0; });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::vector<std::string> split_csv_line(const std::string& line, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream stream(line);
  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(trim(token));
  }
  if (!line.empty() && line.back() == delimiter) {
    tokens.emplace_back();
  }
  return tokens;
}

bool parse_double(const std::string& value, double& parsed) {
  try {
    std::size_t pos = 0;
    parsed = std::stod(value, &pos);
    return pos == value.size();
  } catch (const std::exception&) {
    return false;
  }
}

bool parse_uint64(const std::string& value, std::uint64_t& parsed) {
  try {
    std::size_t pos = 0;
    parsed = std::stoull(value, &pos);
    return pos == value.size();
  } catch (const std::exception&) {
    return false;
  }
}

bool looks_like_header(const std::vector<std::string>& tokens) {
  if (tokens.empty()) {
    return false;
  }

  double stamp = 0.0;
  if (parse_double(tokens.front(), stamp)) {
    return false;
  }

  std::string first = tokens.front();
  std::transform(first.begin(), first.end(), first.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return first == "stamp" || first == "time" || first == "timestamp";
}

std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

struct HeaderSchema {
  enum class CovarianceLayout { None, Diagonal, Full };

  bool valid = false;
  CovarianceLayout covariance_layout = CovarianceLayout::None;
  std::unordered_map<std::string, std::size_t> column_index;
};

HeaderSchema build_header_schema(const std::vector<std::string>& tokens) {
  HeaderSchema schema;
  for (std::size_t i = 0; i < tokens.size(); ++i) {
    const std::string name = to_lower(tokens[i]);
    if (name.empty()) {
      continue;
    }
    schema.column_index.emplace(name, i);
  }

  static const std::unordered_set<std::string> kRequiredBase = {"stamp", "class_id", "landmark_id", "x", "y", "z", "confidence"};
  for (const auto& name : kRequiredBase) {
    if (schema.column_index.find(name) == schema.column_index.end()) {
      return schema;
    }
  }

  static const std::unordered_set<std::string> kDiagCov = {"cov_xx", "cov_yy", "cov_zz"};
  static const std::unordered_set<std::string> kFullCov = {"cov_xx", "cov_xy", "cov_xz", "cov_yx", "cov_yy", "cov_yz", "cov_zx", "cov_zy", "cov_zz"};

  const bool has_full = std::all_of(kFullCov.begin(), kFullCov.end(), [&](const std::string& key) { return schema.column_index.count(key) != 0; });
  const bool has_diag = std::all_of(kDiagCov.begin(), kDiagCov.end(), [&](const std::string& key) { return schema.column_index.count(key) != 0; });

  if (has_full) {
    schema.covariance_layout = HeaderSchema::CovarianceLayout::Full;
    schema.valid = true;
  } else if (has_diag) {
    schema.covariance_layout = HeaderSchema::CovarianceLayout::Diagonal;
    schema.valid = true;
  }
  return schema;
}

bool parse_observation_by_header(
  const std::vector<std::string>& tokens,
  const HeaderSchema& schema,
  PerceptionObservation& observation,
  std::string* error_message) {
  const auto lookup = [&](const std::string& key, std::string* out_value) {
    const auto it = schema.column_index.find(key);
    if (it == schema.column_index.end() || it->second >= tokens.size()) {
      return false;
    }
    *out_value = tokens[it->second];
    return true;
  };

  std::string text;

  if (!lookup("stamp", &text) || !parse_double(text, observation.stamp)) {
    if (error_message) *error_message = "invalid stamp";
    return false;
  }
  if (!lookup("class_id", &observation.class_id) || observation.class_id.empty()) {
    if (error_message) *error_message = "missing class_id";
    return false;
  }
  if (!lookup("landmark_id", &text) || !parse_uint64(text, observation.landmark_id)) {
    if (error_message) *error_message = "invalid landmark_id";
    return false;
  }

  static const char* kAxes[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    double value = 0.0;
    if (!lookup(kAxes[i], &text) || !parse_double(text, value)) {
      if (error_message) *error_message = "invalid position column";
      return false;
    }
    observation.position_sensor[i] = value;
  }

  observation.covariance.setZero();
  if (schema.covariance_layout == HeaderSchema::CovarianceLayout::Full) {
    static const char* kFullKeys[3][3] = {
      {"cov_xx", "cov_xy", "cov_xz"},
      {"cov_yx", "cov_yy", "cov_yz"},
      {"cov_zx", "cov_zy", "cov_zz"},
    };
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        double value = 0.0;
        if (!lookup(kFullKeys[row][col], &text) || !parse_double(text, value)) {
          if (error_message) *error_message = "invalid full covariance column";
          return false;
        }
        observation.covariance(row, col) = value;
      }
    }
  } else {
    static const char* kDiagKeys[] = {"cov_xx", "cov_yy", "cov_zz"};
    for (int i = 0; i < 3; ++i) {
      double value = 0.0;
      if (!lookup(kDiagKeys[i], &text) || !parse_double(text, value)) {
        if (error_message) *error_message = "invalid diagonal covariance column";
        return false;
      }
      observation.covariance(i, i) = value;
    }
  }

  if (!lookup("confidence", &text) || !parse_double(text, observation.confidence)) {
    if (error_message) *error_message = "invalid confidence";
    return false;
  }
  return true;
}

}  // namespace

bool parse_perception_observation_csv_row(
  const std::string& line,
  PerceptionObservation& observation,
  std::string* error_message,
  char delimiter) {
  const std::vector<std::string> tokens = split_csv_line(line, delimiter);
  if (tokens.size() != 10 && tokens.size() != 16) {
    if (error_message) {
      *error_message = "expected 10 columns for diagonal covariance or 16 columns for full 3x3 covariance";
    }
    return false;
  }

  PerceptionObservation parsed;
  if (!parse_double(tokens[0], parsed.stamp)) {
    if (error_message) {
      *error_message = "invalid stamp";
    }
    return false;
  }

  parsed.class_id = tokens[1];
  if (!parse_uint64(tokens[2], parsed.landmark_id)) {
    if (error_message) {
      *error_message = "invalid landmark_id";
    }
    return false;
  }

  for (int i = 0; i < 3; i++) {
    double value = 0.0;
    if (!parse_double(tokens[3 + i], value)) {
      if (error_message) {
        *error_message = "invalid position column";
      }
      return false;
    }
    parsed.position_sensor[i] = value;
  }

  parsed.covariance.setZero();
  if (tokens.size() == 10) {
    for (int i = 0; i < 3; i++) {
      double value = 0.0;
      if (!parse_double(tokens[6 + i], value)) {
        if (error_message) {
          *error_message = "invalid diagonal covariance column";
        }
        return false;
      }
      parsed.covariance(i, i) = value;
    }

    if (!parse_double(tokens[9], parsed.confidence)) {
      if (error_message) {
        *error_message = "invalid confidence";
      }
      return false;
    }
  } else {
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        double value = 0.0;
        if (!parse_double(tokens[6 + row * 3 + col], value)) {
          if (error_message) {
            *error_message = "invalid full covariance column";
          }
          return false;
        }
        parsed.covariance(row, col) = value;
      }
    }

    if (!parse_double(tokens[15], parsed.confidence)) {
      if (error_message) {
        *error_message = "invalid confidence";
      }
      return false;
    }
  }

  observation = parsed;
  return true;
}

PerceptionObservationCsvLoadResult load_perception_observations_csv(std::istream& input, const PerceptionObservationCsvOptions& options) {
  PerceptionObservationCsvLoadResult result;

  std::string line;
  std::size_t line_number = 0;
  bool first_data_row = true;
  HeaderSchema header_schema;
  while (std::getline(input, line)) {
    line_number++;

    const std::string stripped = trim(line);
    if (stripped.empty() || stripped.front() == '#') {
      continue;
    }

    const std::vector<std::string> tokens = split_csv_line(stripped, options.delimiter);
    if (options.skip_header && first_data_row && looks_like_header(tokens)) {
      first_data_row = false;
      HeaderSchema detected = build_header_schema(tokens);
      if (detected.valid) {
        header_schema = std::move(detected);
      }
      continue;
    }
    first_data_row = false;

    PerceptionObservation observation;
    std::string error;
    bool parse_ok = false;
    if (header_schema.valid) {
      parse_ok = parse_observation_by_header(tokens, header_schema, observation, &error);
    } else {
      parse_ok = parse_perception_observation_csv_row(stripped, observation, &error, options.delimiter);
    }

    if (!parse_ok) {
      result.errors.push_back({line_number, error, line});
      if (!options.skip_invalid_rows) {
        break;
      }
      continue;
    }

    if (!observation.valid(options.min_confidence)) {
      result.errors.push_back({line_number, "invalid observation values", line});
      if (!options.skip_invalid_rows) {
        break;
      }
      continue;
    }

    result.observations.push_back(observation);
  }

  struct LandmarkEntry {
    std::string class_id;
    std::size_t first_index = 0;
  };
  std::map<std::pair<double, std::uint64_t>, std::size_t> seen_stamps;
  std::map<std::uint64_t, LandmarkEntry> landmark_classes;

  for (std::size_t i = 0; i < result.observations.size(); ++i) {
    const auto& obs = result.observations[i];

    const auto stamp_key = std::make_pair(obs.stamp, obs.landmark_id);
    const auto stamp_hit = seen_stamps.find(stamp_key);
    if (stamp_hit != seen_stamps.end()) {
      std::ostringstream message;
      message << "duplicate (stamp, landmark_id) pair; first seen at observation " << stamp_hit->second;
      result.warnings.push_back({PerceptionObservationCsvWarning::Kind::DuplicateStampLandmark, i, obs.landmark_id, obs.class_id, message.str()});
    } else {
      seen_stamps.emplace(stamp_key, i);
    }

    const auto class_hit = landmark_classes.find(obs.landmark_id);
    if (class_hit != landmark_classes.end()) {
      if (class_hit->second.class_id != obs.class_id) {
        std::ostringstream message;
        message << "landmark_id previously observed as class '" << class_hit->second.class_id << "' at observation " << class_hit->second.first_index;
        result.warnings.push_back({PerceptionObservationCsvWarning::Kind::LandmarkClassCollision, i, obs.landmark_id, obs.class_id, message.str()});
      }
    } else {
      landmark_classes.emplace(obs.landmark_id, LandmarkEntry{obs.class_id, i});
    }

    bool degenerate = false;
    for (int k = 0; k < 3; ++k) {
      const double variance = obs.covariance(k, k);
      if (!std::isfinite(variance) || variance <= 0.0) {
        degenerate = true;
        break;
      }
    }
    if (degenerate) {
      result.warnings.push_back({
        PerceptionObservationCsvWarning::Kind::DegenerateCovariance,
        i,
        obs.landmark_id,
        obs.class_id,
        "covariance diagonal has zero, negative, or non-finite values",
      });
      continue;
    }

    constexpr double kSymmetryTol = 1e-9;
    bool symmetric = true;
    for (int row = 0; row < 3 && symmetric; ++row) {
      for (int col = row + 1; col < 3; ++col) {
        if (std::abs(obs.covariance(row, col) - obs.covariance(col, row)) > kSymmetryTol) {
          symmetric = false;
          break;
        }
      }
    }

    bool non_pd = !symmetric;
    if (symmetric) {
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(obs.covariance);
      if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= 0.0) {
        non_pd = true;
      }
    }

    if (non_pd) {
      result.warnings.push_back({
        PerceptionObservationCsvWarning::Kind::NonPositiveDefiniteCovariance,
        i,
        obs.landmark_id,
        obs.class_id,
        symmetric
          ? "covariance is symmetric but not positive-definite"
          : "covariance is not symmetric; full 3x3 entries disagree across the diagonal",
      });
    }
  }

  return result;
}

PerceptionObservationCsvLoadResult load_perception_observations_csv(const std::string& path, const PerceptionObservationCsvOptions& options) {
  std::ifstream input(path);
  PerceptionObservationCsvLoadResult result;
  if (!input) {
    result.errors.push_back({0, "failed to open file", path});
    return result;
  }
  return load_perception_observations_csv(input, options);
}

}  // namespace glil
