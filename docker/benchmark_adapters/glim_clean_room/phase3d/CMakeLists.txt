cmake_minimum_required(VERSION 3.16)
project(glim_clean_room_phase3d LANGUAGES CXX)

option(GLIM_CLEAN_ROOM_PHASE3D_SANITIZERS
  "Enable AddressSanitizer and UndefinedBehaviorSanitizer"
  OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GLIM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 exact-core GLIM install prefix")
set(GTSAM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 GTSAM install prefix")
set(GTSAM_POINTS_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 gtsam_points install prefix")
set(GLIM_JSON_INCLUDE_DIR "" CACHE PATH
  "Exact pinned GLIM thirdparty/json/include directory")

if(NOT IS_DIRECTORY "${GLIM_INSTALL_PREFIX}" OR
   NOT EXISTS "${GLIM_INSTALL_PREFIX}/include/glim/odometry/estimation_frame.hpp" OR
   NOT EXISTS "${GLIM_INSTALL_PREFIX}/include/glim/odometry/odometry_estimation_cpu.hpp")
  message(FATAL_ERROR
    "GLIM_INSTALL_PREFIX must be the preserved exact-core install prefix")
endif()
if(NOT IS_DIRECTORY "${GTSAM_INSTALL_PREFIX}" OR
   NOT IS_DIRECTORY "${GTSAM_POINTS_INSTALL_PREFIX}")
  message(FATAL_ERROR
    "Phase 3d requires preserved GTSAM and gtsam_points prefixes")
endif()
if(NOT IS_DIRECTORY "${GLIM_JSON_INCLUDE_DIR}/nlohmann" OR
   NOT EXISTS "${GLIM_JSON_INCLUDE_DIR}/nlohmann/json.hpp")
  message(FATAL_ERROR
    "Phase 3d requires the exact pinned GLIM nlohmann/json include directory")
endif()

list(PREPEND CMAKE_PREFIX_PATH
  "${GLIM_INSTALL_PREFIX}"
  "${GTSAM_POINTS_INSTALL_PREFIX}"
  "${GTSAM_INSTALL_PREFIX}")
find_package(glim CONFIG REQUIRED NO_DEFAULT_PATH
  PATHS "${GLIM_INSTALL_PREFIX}/lib/cmake/glim")

set(PHASE3D_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(ADAPTER_ROOT "${PHASE3D_ROOT}/..")
set(ADAPTER_INCLUDE_ROOT "${ADAPTER_ROOT}/include")
set(PHASE3A_INCLUDE_ROOT "${ADAPTER_ROOT}/phase3a/include")
set(PHASE3B_INCLUDE_ROOT "${ADAPTER_ROOT}/phase3b/include")

add_library(glim_clean_room_phase3d_core STATIC
  src/phase3d_ingress.cpp
  src/phase3d_output_contract.cpp
  "${ADAPTER_ROOT}/src/contract.cpp"
  "${ADAPTER_ROOT}/src/pointcloud2_parser.cpp"
  "${ADAPTER_ROOT}/phase3a/src/phase3a_conversion.cpp"
  "${ADAPTER_ROOT}/phase3b/src/phase3b_session.cpp")
target_include_directories(glim_clean_room_phase3d_core PUBLIC
  "$<BUILD_INTERFACE:${PHASE3D_ROOT}/include>"
  "$<BUILD_INTERFACE:${PHASE3B_INCLUDE_ROOT}>"
  "$<BUILD_INTERFACE:${PHASE3A_INCLUDE_ROOT}>"
  "$<BUILD_INTERFACE:${ADAPTER_INCLUDE_ROOT}>"
  "${GLIM_JSON_INCLUDE_DIR}"
  "${GLIM_INSTALL_PREFIX}/include"
  "${GTSAM_INSTALL_PREFIX}/include"
  "${GTSAM_POINTS_INSTALL_PREFIX}/include")
target_compile_features(glim_clean_room_phase3d_core PUBLIC cxx_std_17)
target_compile_options(glim_clean_room_phase3d_core PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
target_link_libraries(glim_clean_room_phase3d_core PUBLIC
  glim::glim
  glim::odometry_estimation_cpu)

add_library(glim_clean_room_phase3d_node STATIC
  src/phase3d_node.cpp)
target_include_directories(glim_clean_room_phase3d_node PUBLIC
  "$<BUILD_INTERFACE:${PHASE3D_ROOT}/include>"
  "$<BUILD_INTERFACE:${PHASE3B_INCLUDE_ROOT}>"
  "$<BUILD_INTERFACE:${PHASE3A_INCLUDE_ROOT}>"
  "$<BUILD_INTERFACE:${ADAPTER_INCLUDE_ROOT}>")
target_compile_features(glim_clean_room_phase3d_node PUBLIC cxx_std_17)
target_compile_options(glim_clean_room_phase3d_node PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(glim_clean_room_phase3d_node PRIVATE
  GLIM_CLEAN_ROOM_WITH_ROS2=1)
target_link_libraries(glim_clean_room_phase3d_node PUBLIC
  glim_clean_room_phase3d_core
  rclcpp::rclcpp
  diagnostic_msgs::diagnostic_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  std_srvs::std_srvs__rosidl_generator_cpp)
ament_target_dependencies(glim_clean_room_phase3d_node PUBLIC
  diagnostic_msgs nav_msgs rclcpp sensor_msgs std_srvs)

add_executable(glim_clean_room_phase3d_node_main src/main.cpp)
target_compile_features(glim_clean_room_phase3d_node_main PRIVATE cxx_std_17)
target_compile_options(glim_clean_room_phase3d_node_main PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(glim_clean_room_phase3d_node_main PRIVATE
  glim_clean_room_phase3d_node)

if(GLIM_CLEAN_ROOM_PHASE3D_SANITIZERS)
  foreach(target glim_clean_room_phase3d_core glim_clean_room_phase3d_node
                 glim_clean_room_phase3d_node_main)
    target_compile_options(${target} PRIVATE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE
      -fsanitize=address,undefined)
  endforeach()
endif()

set_target_properties(glim_clean_room_phase3d_node_main PROPERTIES
  BUILD_RPATH
    "${GLIM_INSTALL_PREFIX}/lib;${GTSAM_POINTS_INSTALL_PREFIX}/lib;${GTSAM_INSTALL_PREFIX}/lib")

install(TARGETS glim_clean_room_phase3d_node_main
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${ADAPTER_INCLUDE_ROOT}/" DESTINATION include)
install(DIRECTORY "${PHASE3A_INCLUDE_ROOT}/" DESTINATION include)
install(DIRECTORY "${PHASE3B_INCLUDE_ROOT}/" DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(glim_clean_room_phase3d_ingress_tests
    tests/phase3d_ingress_tests.cpp)
  target_link_libraries(glim_clean_room_phase3d_ingress_tests
    glim_clean_room_phase3d_core)
  target_compile_features(glim_clean_room_phase3d_ingress_tests PRIVATE cxx_std_17)
  target_compile_options(glim_clean_room_phase3d_ingress_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
  if(GLIM_CLEAN_ROOM_PHASE3D_SANITIZERS)
    target_compile_options(glim_clean_room_phase3d_ingress_tests PRIVATE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(glim_clean_room_phase3d_ingress_tests PRIVATE
      -fsanitize=address,undefined)
  endif()
  set_target_properties(glim_clean_room_phase3d_ingress_tests PROPERTIES
    BUILD_RPATH
      "${GLIM_INSTALL_PREFIX}/lib;${GTSAM_POINTS_INSTALL_PREFIX}/lib;${GTSAM_INSTALL_PREFIX}/lib")
  set_tests_properties(glim_clean_room_phase3d_ingress_tests PROPERTIES
    ENVIRONMENT
      "LD_LIBRARY_PATH=/opt/ros/jazzy/lib:${GLIM_INSTALL_PREFIX}/lib:${GTSAM_POINTS_INSTALL_PREFIX}/lib:${GTSAM_INSTALL_PREFIX}/lib")

  ament_add_gtest(glim_clean_room_phase3d_ros_integration_tests
    tests/phase3d_ros_integration_tests.cpp)
  target_link_libraries(glim_clean_room_phase3d_ros_integration_tests
    glim_clean_room_phase3d_node)
  target_compile_features(glim_clean_room_phase3d_ros_integration_tests PRIVATE cxx_std_17)
  target_compile_options(glim_clean_room_phase3d_ros_integration_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
  if(GLIM_CLEAN_ROOM_PHASE3D_SANITIZERS)
    target_compile_options(glim_clean_room_phase3d_ros_integration_tests PRIVATE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(glim_clean_room_phase3d_ros_integration_tests PRIVATE
      -fsanitize=address,undefined)
  endif()
  ament_target_dependencies(glim_clean_room_phase3d_ros_integration_tests
    diagnostic_msgs nav_msgs rclcpp sensor_msgs std_srvs)
  set_target_properties(glim_clean_room_phase3d_ros_integration_tests PROPERTIES
    BUILD_RPATH
      "${GLIM_INSTALL_PREFIX}/lib;${GTSAM_POINTS_INSTALL_PREFIX}/lib;${GTSAM_INSTALL_PREFIX}/lib")
  set_tests_properties(glim_clean_room_phase3d_ros_integration_tests PROPERTIES
    ENVIRONMENT
      "LD_LIBRARY_PATH=/opt/ros/jazzy/lib:${GLIM_INSTALL_PREFIX}/lib:${GTSAM_POINTS_INSTALL_PREFIX}/lib:${GTSAM_INSTALL_PREFIX}/lib")

  add_test(NAME glim_clean_room_phase3d_source_guard
    COMMAND "${Python3_EXECUTABLE}"
            "${ADAPTER_ROOT}/tests/source_guard.py"
            "${ADAPTER_ROOT}")
endif()

ament_export_include_directories(include)
ament_export_dependencies(diagnostic_msgs nav_msgs rclcpp sensor_msgs std_srvs)
ament_package()
