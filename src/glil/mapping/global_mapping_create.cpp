#include <glil/mapping/global_mapping.hpp>

extern "C" glil::GlobalMappingBase* create_global_mapping_module() {
  glil::GlobalMappingParams params;
  return new glil::GlobalMapping(params);
}
