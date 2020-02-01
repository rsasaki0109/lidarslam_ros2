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
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "RELEASE")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package (PCL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(scanmatcher_component SHARED
src/scanmatcher_component.cpp
)

target_compile_definitions(scanmatcher_component PRIVATE "GS_SM_BUILDING_DLL")

ament_target_dependencies(scanmatcher_component
  rclcpp rclcpp_components tf2_ros tf2_geometry_msgs tf2_eigen sensor_msgs)

add_executable(scanmatcher_node
src/scanmatcher_node.cpp
)

target_link_libraries(scanmatcher_node
scanmatcher_component ${PCL_LIBRARIES})

ament_target_dependencies(scanmatcher_node
  rclcpp)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_nodes(scanmatcher_component 
  "graphslam::ScanMatcherComponent")

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
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
