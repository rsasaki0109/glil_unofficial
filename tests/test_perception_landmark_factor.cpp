// SPDX-License-Identifier: MIT
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <glil/factors/perception_landmark_factor.hpp>

namespace {
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

constexpr double kPi = 3.14159265358979323846;

void expect_near(const gtsam::Vector3& actual, const gtsam::Vector3& expected, double tol) {
  const double err = (actual - expected).norm();
  if (err > tol) {
    std::cerr << "expected " << expected.transpose() << ", got " << actual.transpose() << ", err=" << err << std::endl;
    std::abort();
  }
}

void test_zero_error_and_jacobians() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  glil::PerceptionLandmarkFactor factor(X(0), L(0), gtsam::Point3(1.0, 0.0, 0.0), noise, 10.0, "pole", 42, 0.8);

  const gtsam::Pose3 pose;
  const gtsam::Point3 landmark(1.0, 0.0, 0.0);
  gtsam::Matrix H_pose;
  gtsam::Matrix H_landmark;
  const gtsam::Vector err = factor.evaluateError(pose, landmark, H_pose, H_landmark);

  expect_near(err, gtsam::Vector3::Zero(), 1e-12);
  assert(H_pose.rows() == 3 && H_pose.cols() == 6);
  assert(H_landmark.rows() == 3 && H_landmark.cols() == 3);
  assert(factor.class_id() == "pole");
  assert(factor.landmark_id() == 42);
}

void test_noise_from_observation() {
  glil::PerceptionObservation observation;
  observation.position_sensor = gtsam::Point3(1.0, 2.0, 3.0);
  observation.covariance.diagonal() << 0.04, 0.09, 0.16;
  observation.confidence = 0.25;
  assert(observation.valid());

  const auto noise = glil::make_perception_noise_model(observation);
  const gtsam::Vector whitened = noise->whiten(gtsam::Vector3(0.4, 0.6, 0.8));

  // confidence=0.25 doubles sigma, so the residuals whiten to 1.0.
  expect_near(whitened, gtsam::Vector3::Ones(), 1e-12);
}

void test_landmark_optimization() {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.0, 0.0, kPi / 2.0), gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Point3 measured_sensor(2.0, 0.0, 0.0);
  const gtsam::Point3 true_landmark = pose.transformFrom(measured_sensor);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), pose, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.emplace_shared<glil::PerceptionLandmarkFactor>(X(0), L(0), measured_sensor, gtsam::noiseModel::Isotropic::Sigma(3, 0.05));

  gtsam::Values initial;
  initial.insert(X(0), pose);
  initial.insert(L(0), true_landmark + gtsam::Point3(0.4, -0.3, 0.2));

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  expect_near(result.at<gtsam::Point3>(L(0)), true_landmark, 1e-6);
}
}  // namespace

int main() {
  test_zero_error_and_jacobians();
  test_noise_from_observation();
  test_landmark_optimization();
  return 0;
}
