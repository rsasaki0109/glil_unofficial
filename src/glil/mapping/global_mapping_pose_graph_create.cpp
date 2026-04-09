#include <glil/mapping/global_mapping_pose_graph.hpp>

extern "C" glil::GlobalMappingBase* create_global_mapping_module() {
  glil::GlobalMappingPoseGraphParams params;
  return new glil::GlobalMappingPoseGraph(params);
}
