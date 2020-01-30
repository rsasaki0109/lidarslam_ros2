cmake_minimum_required(VERSION 3.5)
project(graphslam_main)

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
find_package(scanmatcher REQUIRED)
#find_package(graphslam REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(graphslam_node
src/main.cpp
)

ament_target_dependencies(graphslam_node
  rclcpp
  scanmatcher
  #graphslam
  )

install(TARGETS 
  graphslam_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
