#include <glil/odometry/odometry_estimation_ct.hpp>

extern "C" glil::OdometryEstimationBase* create_odometry_estimation_module() {
  glil::OdometryEstimationCTParams params;
  return new glil::OdometryEstimationCT(params);
}