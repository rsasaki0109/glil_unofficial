cmake_minimum_required(VERSION 3.5)
project(glil VERSION 1.0.1 LANGUAGES CXX)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" OFF)
option(BUILD_WITH_CUDA_MULTIARCH "Build with GPU cross-platform support" OFF)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_TOOLS "Build command line tools" ON)
option(GLIL_PROFILE_TIMING "Emit per-frame TRUE GLIL timing logs" ON)

if(BUILD_WITH_MARCH_NATIVE)
  add_definitions(-march=native)
  add_definitions(-DBUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

find_package(Boost REQUIRED serialization)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(GTSAM REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)

if(BUILD_WITH_VIEWER)
  find_package(Iridescence REQUIRED)
endif()

###########
## Build ##
###########

# GPU-related
if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_POINTS_GPU)
  set(GPU_SRCS src/glil/odometry/odometry_estimation_gpu.cpp)
endif()

# Caratheodory2 coreset library (for GLIL exact downsampling)
# Build inline instead of add_subdirectory to avoid install/export issues
add_library(caratheodory SHARED
  thirdparty/caratheodory2/src/caratheodory.cpp
)
target_include_directories(caratheodory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/caratheodory2/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(caratheodory Eigen3::Eigen OpenMP::OpenMP_CXX)
target_compile_features(caratheodory PUBLIC cxx_std_20)

add_library(glil SHARED
  # factors
  src/glil/factors/integrated_coreset_factor.cpp
  src/glil/factors/integrated_vgicp_coreset_factor.cpp
  src/glil/factors/perception_landmark_factor.cpp
  # perception
  src/glil/perception/perception_factor_builder.cpp
  src/glil/perception/perception_observation_io.cpp
  src/glil/perception/cloud_landmark_extractor.cpp
  # util
  src/glil/util/config.cpp
  src/glil/util/logging.cpp
  src/glil/util/export_factors.cpp
  src/glil/util/time_keeper.cpp
  src/glil/util/extension_module.cpp
  src/glil/util/load_module.cpp
  src/glil/util/data_validator.cpp
  src/glil/util/serialization.cpp
  # preprocess
  src/glil/preprocess/cloud_preprocessor.cpp
  # common process
  src/glil/common/imu_integration.cpp
  src/glil/common/cloud_deskewing.cpp
  src/glil/common/cloud_covariance_estimation.cpp
  # odometry
  src/glil/odometry/callbacks.cpp
  src/glil/odometry/estimation_frame.cpp
  src/glil/odometry/initial_state_estimation.cpp
  src/glil/odometry/loose_initial_state_estimation.cpp
  src/glil/odometry/odometry_estimation_base.cpp
  src/glil/odometry/async_odometry_estimation.cpp
  src/glil/odometry/odometry_estimation_imu.cpp
  src/glil/odometry/odometry_estimation_ct.cpp
  src/glil/odometry/odometry_estimation_cpu.cpp
  src/glil/odometry/odometry_estimation_glil.cpp
  # mapping
  src/glil/mapping/callbacks.cpp
  src/glil/mapping/sub_map.cpp
  src/glil/mapping/sub_mapping_base.cpp
  src/glil/mapping/global_mapping_base.cpp
  src/glil/mapping/async_sub_mapping.cpp
  src/glil/mapping/async_global_mapping.cpp
  src/glil/mapping/sub_mapping.cpp
  src/glil/mapping/sub_mapping_passthrough.cpp
  src/glil/mapping/global_mapping.cpp
  src/glil/mapping/global_mapping_pose_graph.cpp
  # GPU-related
  ${GPU_SRCS}
)
if(GLIL_PROFILE_TIMING)
  target_compile_definitions(glil PRIVATE GLIL_PROFILE_TIMING)
endif()
target_include_directories(glil PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/json/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(glil
  Boost::boost
  Boost::serialization
  Eigen3::Eigen
  gtsam
  gtsam_points::gtsam_points
  OpenMP::OpenMP_CXX
  spdlog::spdlog
  caratheodory
  $<TARGET_NAME_IF_EXISTS:Iridescence::Iridescence>
  ${OpenCV_LIBRARIES}
)
list(APPEND glil_LIBRARIES glil)

# Create shared libraries for estimation modules
list(APPEND module_srcs
  src/glil/odometry/odometry_estimation_ct_create.cpp
  src/glil/odometry/odometry_estimation_cpu_create.cpp
  src/glil/odometry/odometry_estimation_glil_create.cpp
  src/glil/mapping/sub_mapping_create.cpp
  src/glil/mapping/sub_mapping_passthrough_create.cpp
  src/glil/mapping/global_mapping_create.cpp
  src/glil/mapping/global_mapping_pose_graph_create.cpp
)
if(BUILD_WITH_CUDA)
  list(APPEND module_srcs
    src/glil/odometry/odometry_estimation_gpu_create.cpp
  )
endif()

foreach(module_src IN LISTS module_srcs)
  get_filename_component(module_src_filename ${module_src} NAME_WE)
  string(REGEX REPLACE "_create" "" module_name ${module_src_filename})

  add_library(${module_name} SHARED
    ${module_src}
  )
  target_link_libraries(${module_name}
    glil
  )
  list(APPEND glil_LIBRARIES ${module_name})
endforeach()

# Optional extension modules
add_library(perception_csv_injector SHARED
  src/glil/perception/perception_csv_injector.cpp
)
target_link_libraries(perception_csv_injector
  glil
)
list(APPEND glil_LIBRARIES perception_csv_injector)


if(BUILD_WITH_VIEWER)
  # Standard viewer
  add_library(standard_viewer SHARED
    src/glil/viewer/standard_viewer.cpp
  )
  target_link_libraries(standard_viewer
    glil
  )

  # Interactive viewer
  add_library(interactive_viewer SHARED
    src/glil/viewer/interactive_viewer.cpp
    src/glil/viewer/interactive/manual_loop_close_modal.cpp
    src/glil/viewer/interactive/bundle_adjustment_modal.cpp
    src/glil/viewer/offline_viewer.cpp
  )
  target_link_libraries(interactive_viewer
    glil
  )

  # Memory monitor
  add_library(memory_monitor SHARED
    src/glil/viewer/memory_monitor.cpp
  )
  target_link_libraries(memory_monitor
    glil
  )

  list(APPEND glil_LIBRARIES standard_viewer interactive_viewer memory_monitor)
endif()


###########
## Tools ##
###########

if(BUILD_TOOLS)
  add_executable(glil_coreset_benchmark
    tools/benchmark_coreset_factor.cpp
  )
  target_link_libraries(glil_coreset_benchmark
    glil
  )

  add_executable(glil_cloud_landmark_extractor
    tools/extract_cloud_landmarks.cpp
  )
  target_link_libraries(glil_cloud_landmark_extractor
    glil
  )

  add_executable(glil_perception_config_generator
    tools/generate_perception_config.cpp
  )
  target_link_libraries(glil_perception_config_generator
    glil
  )

  list(APPEND glil_TOOL_TARGETS glil_coreset_benchmark glil_cloud_landmark_extractor glil_perception_config_generator)
endif()

###########
## Tests ##
###########

include(CTest)
if(BUILD_TESTING)
  add_executable(test_perception_landmark_factor
    tests/test_perception_landmark_factor.cpp
  )
  target_link_libraries(test_perception_landmark_factor
    glil
  )
  add_test(NAME perception_landmark_factor COMMAND test_perception_landmark_factor)

  add_executable(test_integrated_vgicp_coreset_factor
    tests/test_integrated_vgicp_coreset_factor.cpp
  )
  target_link_libraries(test_integrated_vgicp_coreset_factor
    glil
  )
  add_test(NAME integrated_vgicp_coreset_factor COMMAND test_integrated_vgicp_coreset_factor)
endif()

#############
## Install ##
#############

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS caratheodory ${glil_LIBRARIES}
  EXPORT glil-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
if(BUILD_TOOLS)
  install(TARGETS ${glil_TOOL_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()
set(CMAKE_CONFIG_INSTALL_DIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/glil"
  CACHE PATH "Install directory for CMake config files"
)
include(CMakePackageConfigHelpers)
install(EXPORT glil-targets
  FILE glil-targets.cmake
  NAMESPACE glil::
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/glil-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/glil-config.cmake"
  INSTALL_DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/glil-config-version.cmake"
  VERSION ${VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/glil-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/glil-config-version.cmake"
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)

#################
## ROS-related ##
#################
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY config DESTINATION share/glil)
    ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    ament_export_libraries(${glil_LIBRARIES} ${GTSAM_LIBRARIES} fmt spdlog::spdlog)
    ament_package()
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${glil_LIBRARIES} ${GTSAM_LIBRARIES} fmt spdlog::spdlog
    )
  endif()
endif()
