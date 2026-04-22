// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <istream>
#include <string>
#include <vector>

#include <glil/perception/perception_observation.hpp>

namespace glil {

struct PerceptionObservationCsvOptions {
  char delimiter = ',';
  bool skip_header = true;
  bool skip_invalid_rows = false;
  double min_confidence = 0.0;
};

struct PerceptionObservationCsvError {
  std::size_t line = 0;
  std::string message;
  std::string text;
};

struct PerceptionObservationCsvLoadResult {
  std::vector<PerceptionObservation> observations;
  std::vector<PerceptionObservationCsvError> errors;
};

bool parse_perception_observation_csv_row(
  const std::string& line,
  PerceptionObservation& observation,
  std::string* error_message = nullptr,
  char delimiter = ',');

PerceptionObservationCsvLoadResult load_perception_observations_csv(
  std::istream& input,
  const PerceptionObservationCsvOptions& options = PerceptionObservationCsvOptions());

PerceptionObservationCsvLoadResult load_perception_observations_csv(
  const std::string& path,
  const PerceptionObservationCsvOptions& options = PerceptionObservationCsvOptions());

}  // namespace glil
