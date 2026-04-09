#include <glil/util/extension_module.hpp>

#include <spdlog/spdlog.h>
#include <glil/util/load_module.hpp>

namespace glil {

std::shared_ptr<ExtensionModule> ExtensionModule::load_module(const std::string& so_name) {
  return load_module_from_so<ExtensionModule>(so_name, "create_extension_module");
}

}  // namespace glil