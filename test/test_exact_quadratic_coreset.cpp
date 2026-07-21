#include <glim/odometry/exact_quadratic_coreset.hpp>
#include <glim/odometry/tightly_coupled_window.hpp>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
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

  return true;
}

}  // namespace

int main()
{
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
  std::cout << "exact quadratic coreset tests passed\n";
  return 0;
}
