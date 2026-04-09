#include <glil/odometry/odometry_estimation_gpu.hpp>

extern "C" glil::OdometryEstimationBase* create_odometry_estimation_module() {
  glil::OdometryEstimationGPUParams params;
  return new glil::OdometryEstimationGPU(params);
}