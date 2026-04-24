// SPDX-License-Identifier: MIT
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glil/gnss/gnss_observation.hpp>
#include <glil/gnss/gnss_observation_io.hpp>

namespace {
using gtsam::symbol_shorthand::X;

void test_observation_validity() {
  glil::GNSSObservation ok;
  ok.stamp = 1.0;
  ok.position = gtsam::Vector3(10.0, 20.0, 5.0);
  ok.sigma = gtsam::Vector3(0.05, 0.05, 0.1);
  assert(ok.valid());

  glil::GNSSObservation zero_sigma = ok;
  zero_sigma.sigma[2] = 0.0;
  assert(!zero_sigma.valid());

  glil::GNSSObservation negative_sigma = ok;
  negative_sigma.sigma[0] = -0.1;
  assert(!negative_sigma.valid());

  glil::GNSSObservation nan_pos = ok;
  nan_pos.position[1] = std::nan("");
  assert(!nan_pos.valid());
}

void test_csv_parse_per_axis_sigma() {
  const std::string csv =
    "stamp,x,y,z,sigma_x,sigma_y,sigma_z\n"
    "1.0,10.0,20.0,5.0,0.05,0.05,0.10\n"
    "1.1,10.2,20.1,5.0,0.06,0.06,0.12\n";

  std::istringstream input(csv);
  const auto loaded = glil::load_gnss_observations_csv(input);
  assert(loaded.errors.empty());
  assert(loaded.observations.size() == 2);
  assert(std::abs(loaded.observations.front().position[0] - 10.0) < 1e-12);
  assert(std::abs(loaded.observations.front().sigma[2] - 0.10) < 1e-12);
  assert(std::abs(loaded.observations.back().sigma[1] - 0.06) < 1e-12);
}

void test_csv_parse_isotropic_sigma() {
  const std::string csv =
    "stamp,x,y,z,sigma\n"
    "2.0,1.0,2.0,3.0,0.25\n";

  std::istringstream input(csv);
  const auto loaded = glil::load_gnss_observations_csv(input);
  assert(loaded.errors.empty());
  assert(loaded.observations.size() == 1);
  const auto& obs = loaded.observations.front();
  for (int i = 0; i < 3; ++i) {
    assert(std::abs(obs.sigma[i] - 0.25) < 1e-12);
  }
}

void test_csv_skip_invalid_rows() {
  const std::string csv =
    "stamp,x,y,z,sigma_x,sigma_y,sigma_z\n"
    "1.0,10.0,20.0,5.0,0.05,0.05,0.10\n"
    "1.1,bad,20.1,5.0,0.06,0.06,0.12\n"
    "1.2,10.3,20.2,5.0,0.07,0.07,0.14\n";

  glil::GNSSObservationCsvOptions options;
  options.skip_invalid_rows = true;

  std::istringstream input(csv);
  const auto loaded = glil::load_gnss_observations_csv(input, options);
  assert(loaded.observations.size() == 2);
  assert(loaded.errors.size() == 1);
  assert(loaded.errors.front().line == 3);
}

void test_gps_factor_from_observation() {
  glil::GNSSObservation obs;
  obs.stamp = 3.0;
  obs.position = gtsam::Vector3(100.0, 50.0, 10.0);
  obs.sigma = gtsam::Vector3(0.1, 0.1, 0.2);
  assert(obs.valid());

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::GPSFactor>(
    X(0),
    gtsam::Point3(obs.position),
    gtsam::noiseModel::Diagonal::Sigmas(obs.sigma));

  const auto factor = std::dynamic_pointer_cast<gtsam::GPSFactor>(graph.at(0));
  assert(factor);
  assert(std::abs(factor->measurementIn().x() - 100.0) < 1e-12);
  assert(std::abs(factor->measurementIn().z() - 10.0) < 1e-12);
}

}  // namespace

int main() {
  test_observation_validity();
  test_csv_parse_per_axis_sigma();
  test_csv_parse_isotropic_sigma();
  test_csv_skip_invalid_rows();
  test_gps_factor_from_observation();
  return 0;
}
