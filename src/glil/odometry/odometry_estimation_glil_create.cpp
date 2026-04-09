#include <glil/odometry/odometry_estimation_glil.hpp>

extern "C" glil::OdometryEstimationBase* create_odometry_estimation_module() {
  glil::OdometryEstimationGLILParams params;
  return new glil::OdometryEstimationGLIL(params);
}
