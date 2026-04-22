// SPDX-License-Identifier: MIT
#include <glil/perception/perception_observation_io.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

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
  while (std::getline(input, line)) {
    line_number++;

    const std::string stripped = trim(line);
    if (stripped.empty() || stripped.front() == '#') {
      continue;
    }

    const std::vector<std::string> tokens = split_csv_line(stripped, options.delimiter);
    if (options.skip_header && first_data_row && looks_like_header(tokens)) {
      first_data_row = false;
      continue;
    }
    first_data_row = false;

    PerceptionObservation observation;
    std::string error;
    if (!parse_perception_observation_csv_row(stripped, observation, &error, options.delimiter)) {
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
