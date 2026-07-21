#include <glim/odometry/exact_quadratic_coreset.hpp>
#include <glim/odometry/imu_prediction_guard.hpp>
#include <glim/odometry/tightly_coupled_window.hpp>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <limits>
#include <random>

namespace {

bool verify_exact_sum(const int dimensions, const int point_count, const int target_size)
{
  std::mt19937 generator(42);
  std::normal_distribution<double> distribution;
  Eigen::MatrixXd contributions(dimensions, point_count);
  for (int column = 0; column < point_count; ++column) {
    for (int row = 0; row < dimensions; ++row) {
      contributions(row, column) = distribution(generator);
    }
  }

  const glim::ExactCoreset coreset =
    glim::extractCoreset(contributions, target_size, 64);
  if (coreset.indices.size() != coreset.weights.size() ||
      coreset.indices.size() > static_cast<std::size_t>(std::max(target_size, dimensions + 1))) {
    return false;
  }

  Eigen::VectorXd reduced = Eigen::VectorXd::Zero(dimensions);
  for (std::size_t index = 0; index < coreset.indices.size(); ++index) {
    if (!(coreset.weights[index] > 0.0) ||
        coreset.indices[index] < 0 || coreset.indices[index] >= point_count) {
      return false;
    }
    reduced += coreset.weights[index] * contributions.col(coreset.indices[index]);
  }

  const Eigen::VectorXd full = contributions.rowwise().sum();
  const double relative_error = (reduced - full).norm() / std::max(1.0, full.norm());
  if (relative_error > 1e-10) {
    std::cerr << "relative exact-sum error: " << relative_error << '\n';
    return false;
  }

  double weight_sum = 0.0;
  double max_weight = 0.0;
  for (const double weight : coreset.weights) {
    weight_sum += weight;
    max_weight = std::max(max_weight, weight);
  }
  if (std::abs(weight_sum - point_count) > 1e-9 * point_count ||
      max_weight > point_count * (1.0 + 1e-12)) {
    std::cerr << "invalid coreset mass: sum=" << weight_sum
              << " max=" << max_weight << " point_count=" << point_count << '\n';
    return false;
  }

  return true;
}

}  // namespace

int main()
{
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d far_prediction = identity;
  far_prediction.translation().x() = 100.0;
  if (glim::imuPredictionDiscontinuous(identity, identity, Eigen::Vector3d::Zero(), 20.0) ||
      !glim::imuPredictionDiscontinuous(
        identity, far_prediction, Eigen::Vector3d::Zero(), 20.0)) {
    std::cerr << "IMU prediction discontinuity guard failed\n";
    return 1;
  }
  Eigen::Isometry3d nonfinite_prediction = identity;
  nonfinite_prediction.translation().x() = std::numeric_limits<double>::quiet_NaN();
  if (!glim::imuPredictionDiscontinuous(
        identity, nonfinite_prediction, Eigen::Vector3d::Zero(), 0.0)) {
    std::cerr << "non-finite IMU prediction guard failed\n";
    return 1;
  }
  if (glim::tightlyCoupledFirstTarget(10, 3, 11) != 7 ||
      glim::tightlyCoupledFirstTarget(10, 3, 2) != 9 ||
      glim::tightlyCoupledFirstTarget(176, 3, 2) != 175 ||
      glim::tightlyCoupledFirstTarget(0, 3, 1) != 0) {
    std::cerr << "tightly coupled resident-window policy failed\n";
    return 1;
  }

  // A six-DoF relative-pose quadratic has 21 independent Hessian entries,
  // six gradient entries, and one scalar error entry.
  if (!verify_exact_sum(28, 6500, 32)) {
    return 1;
  }
  if (!verify_exact_sum(28, 100, 1)) {
    return 1;
  }
  // Exercise uneven fast-Caratheodory clusters.  This used to preserve the
  // vector sum while allowing the positive weight mass to drift and grow.
  if (!verify_exact_sum(3, 6503, 5)) {
    return 1;
  }
  std::cout << "exact quadratic coreset tests passed\n";
  return 0;
}
