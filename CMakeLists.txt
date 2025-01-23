cmake_minimum_required(VERSION 3.5)
project(range_based_downsample_filter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(range_based_downsample_filter SHARED
  src/range_based_downsample_filter_node.cpp)

target_link_libraries(range_based_downsample_filter ${PCL_LIBRARIES})

rclcpp_components_register_node(range_based_downsample_filter
  PLUGIN "range_based_downsample_filter::RangeBasedDownsampleFilterNode"
  EXECUTABLE range_based_downsample_filter_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
