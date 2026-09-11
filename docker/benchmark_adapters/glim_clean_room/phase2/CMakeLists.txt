cmake_minimum_required(VERSION 3.16)
project(glim_clean_room_phase2_api_audit LANGUAGES CXX)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GLIM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 GLIM install prefix used only for compile/link probes")
set(GTSAM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 GTSAM install prefix")
set(GTSAM_POINTS_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 gtsam_points install prefix")

if(NOT IS_DIRECTORY "${GLIM_INSTALL_PREFIX}")
  message(FATAL_ERROR "GLIM_INSTALL_PREFIX must be an existing preserved install")
endif()
if(NOT EXISTS "${GLIM_INSTALL_PREFIX}/include/glim/util/raw_points.hpp")
  message(FATAL_ERROR "the preserved install lacks the public RawPoints header")
endif()
if(NOT IS_DIRECTORY "${GTSAM_INSTALL_PREFIX}" OR
   NOT IS_DIRECTORY "${GTSAM_POINTS_INSTALL_PREFIX}")
  message(FATAL_ERROR "all preserved dependency install prefixes are required")
endif()

list(PREPEND CMAKE_PREFIX_PATH
  "${GLIM_INSTALL_PREFIX}"
  "${GTSAM_POINTS_INSTALL_PREFIX}"
  "${GTSAM_INSTALL_PREFIX}")

find_package(glim CONFIG REQUIRED NO_DEFAULT_PATH
  PATHS "${GLIM_INSTALL_PREFIX}/lib/cmake/glim")

add_executable(glim_phase2_public_api_compile_probe api_compile_probe.cpp)
target_compile_features(glim_phase2_public_api_compile_probe PRIVATE cxx_std_17)
target_compile_options(glim_phase2_public_api_compile_probe PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
target_include_directories(glim_phase2_public_api_compile_probe PRIVATE
  "${GLIM_INSTALL_PREFIX}/include"
  "${GTSAM_INSTALL_PREFIX}/include"
  "${GTSAM_POINTS_INSTALL_PREFIX}/include")
target_link_libraries(glim_phase2_public_api_compile_probe PRIVATE
  glim::glim
  glim::odometry_estimation_cpu
  glim::sub_mapping
  glim::global_mapping
  ${CMAKE_DL_LIBS})
set_target_properties(glim_phase2_public_api_compile_probe PROPERTIES
  BUILD_RPATH
    "${GLIM_INSTALL_PREFIX}/lib;${GTSAM_POINTS_INSTALL_PREFIX}/lib;${GTSAM_INSTALL_PREFIX}/lib")

enable_testing()
add_test(NAME glim_phase2_public_api_compile_probe
  COMMAND glim_phase2_public_api_compile_probe)
set_tests_properties(glim_phase2_public_api_compile_probe PROPERTIES
  ENVIRONMENT
    "LD_LIBRARY_PATH=${GLIM_INSTALL_PREFIX}/lib:${GTSAM_POINTS_INSTALL_PREFIX}/lib:${GTSAM_INSTALL_PREFIX}/lib")
add_test(NAME glim_phase2_clean_room_source_guard
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tests/source_guard.py"
          "${CMAKE_CURRENT_SOURCE_DIR}/..")
