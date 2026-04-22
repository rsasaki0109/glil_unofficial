#include <glil/mapping/sub_mapping.hpp>

extern "C" glil::SubMappingBase* create_sub_mapping_module() {
  glil::SubMappingParams params;
  return new glil::SubMapping(params);
}
