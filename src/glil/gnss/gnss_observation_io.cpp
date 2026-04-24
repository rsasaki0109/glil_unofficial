// SPDX-License-Identifier: MIT
#include <glil/gnss/gnss_observation_io.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

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

bool parse_gnss_observation_csv_row(
  const std::string& line,
  GNSSObservation& observation,
  std::string* error_message,
  char delimiter) {
  const std::vector<std::string> tokens = split_csv_line(line, delimiter);
  if (tokens.size() != 5 && tokens.size() != 7) {
    if (error_message) {
      *error_message = "expected 5 columns (stamp,x,y,z,sigma) or 7 columns (stamp,x,y,z,sigma_x,sigma_y,sigma_z)";
    }
    return false;
  }

  GNSSObservation parsed;
  if (!parse_double(tokens[0], parsed.stamp)) {
    if (error_message) {
      *error_message = "invalid stamp";
    }
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    double value = 0.0;
    if (!parse_double(tokens[1 + i], value)) {
      if (error_message) {
        *error_message = "invalid position column";
      }
      return false;
    }
    parsed.position[i] = value;
  }

  if (tokens.size() == 5) {
    double iso_sigma = 0.0;
    if (!parse_double(tokens[4], iso_sigma)) {
      if (error_message) {
        *error_message = "invalid sigma";
      }
      return false;
    }
    parsed.sigma = gtsam::Vector3::Constant(iso_sigma);
  } else {
    for (int i = 0; i < 3; ++i) {
      double value = 0.0;
      if (!parse_double(tokens[4 + i], value)) {
        if (error_message) {
          *error_message = "invalid sigma column";
        }
        return false;
      }
      parsed.sigma[i] = value;
    }
  }

  observation = parsed;
  return true;
}

GNSSObservationCsvLoadResult load_gnss_observations_csv(std::istream& input, const GNSSObservationCsvOptions& options) {
  GNSSObservationCsvLoadResult result;

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

    GNSSObservation observation;
    std::string error;
    if (!parse_gnss_observation_csv_row(stripped, observation, &error, options.delimiter)) {
      result.errors.push_back({line_number, error, line});
      if (!options.skip_invalid_rows) {
        break;
      }
      continue;
    }

    if (!observation.valid()) {
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

GNSSObservationCsvLoadResult load_gnss_observations_csv(const std::string& path, const GNSSObservationCsvOptions& options) {
  std::ifstream input(path);
  GNSSObservationCsvLoadResult result;
  if (!input) {
    result.errors.push_back({0, "failed to open file", path});
    return result;
  }
  return load_gnss_observations_csv(input, options);
}

}  // namespace glil
