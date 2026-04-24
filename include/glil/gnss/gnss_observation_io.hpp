// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <istream>
#include <string>
#include <vector>

#include <glil/gnss/gnss_observation.hpp>

namespace glil {

struct GNSSObservationCsvOptions {
  char delimiter = ',';
  bool skip_header = true;
  bool skip_invalid_rows = false;
};

struct GNSSObservationCsvError {
  std::size_t line = 0;
  std::string message;
  std::string text;
};

struct GNSSObservationCsvLoadResult {
  std::vector<GNSSObservation> observations;
  std::vector<GNSSObservationCsvError> errors;
};

/**
 * Parse a single CSV row describing one GNSS observation.
 * Accepted shapes (all values in the local navigation frame, meters):
 *   stamp,x,y,z,sigma_x,sigma_y,sigma_z  (7 columns, per-axis sigma)
 *   stamp,x,y,z,sigma                    (5 columns, isotropic sigma)
 */
bool parse_gnss_observation_csv_row(
  const std::string& line,
  GNSSObservation& observation,
  std::string* error_message = nullptr,
  char delimiter = ',');

GNSSObservationCsvLoadResult load_gnss_observations_csv(
  std::istream& input,
  const GNSSObservationCsvOptions& options = GNSSObservationCsvOptions());

GNSSObservationCsvLoadResult load_gnss_observations_csv(
  const std::string& path,
  const GNSSObservationCsvOptions& options = GNSSObservationCsvOptions());

}  // namespace glil
