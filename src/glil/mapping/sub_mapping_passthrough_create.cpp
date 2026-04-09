#include <glil/mapping/sub_mapping_passthrough.hpp>

extern "C" glil::SubMappingBase* create_sub_mapping_module() {
  glil::SubMappingPassthroughParams params;
  return new glil::SubMappingPassthrough(params);
}
