cmake_minimum_required(VERSION 3.5)
project(scanmatcher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(scanmatcher_component SHARED
src/scanmatcher_component.cpp
)

target_compile_definitions(scanmatcher_component PRIVATE "GS_SM_BUILDING_DLL")

ament_target_dependencies(scanmatcher_component
  rclcpp rclcpp_components tf2_ros)

add_executable(scanmatcher_node
src/scanmatcher_node.cpp
)

target_link_libraries(scanmatcher_node
scanmatcher_component)

ament_target_dependencies(scanmatcher_node
  rclcpp rclcpp_components tf2_ros)

include_directories(
    include
)

rclcpp_components_register_nodes(scanmatcher_component 
  "graphslam::ScanMatcherComponent")

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS
  scanmatcher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  scanmatcher_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
