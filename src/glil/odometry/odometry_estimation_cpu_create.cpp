#include <glil/odometry/odometry_estimation_cpu.hpp>

extern "C" glil::OdometryEstimationBase* create_odometry_estimation_module() {
  glil::OdometryEstimationCPUParams params;
  return new glil::OdometryEstimationCPU(params);
}