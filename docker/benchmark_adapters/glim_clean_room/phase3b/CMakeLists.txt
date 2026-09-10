cmake_minimum_required(VERSION 3.16)
project(glim_clean_room_phase3b_runtime LANGUAGES CXX)

option(GLIM_CLEAN_ROOM_PHASE3B_SANITIZERS
  "Enable AddressSanitizer and UndefinedBehaviorSanitizer"
  OFF)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GLIM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 exact-core install prefix")
set(GTSAM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 GTSAM install prefix")
set(GTSAM_POINTS_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 gtsam_points install prefix")
set(GLIM_JSON_INCLUDE_DIR "" CACHE PATH
  "Exact pinned GLIM thirdparty/json/include directory")

if(NOT IS_DIRECTORY "${GLIM_INSTALL_PREFIX}" OR
   NOT EXISTS "${GLIM_INSTALL_PREFIX}/include/glim/odometry/odometry_estimation_cpu.hpp")
  message(FATAL_ERROR
    "GLIM_INSTALL_PREFIX must be the preserved exact-core install prefix")
endif()
if(NOT IS_DIRECTORY "${GTSAM_INSTALL_PREFIX}" OR
   NOT IS_DIRECTORY "${GTSAM_POINTS_INSTALL_PREFIX}")
  message(FATAL_ERROR
    "Phase 3b requires preserved GTSAM and gtsam_points prefixes")
endif()
if(NOT IS_DIRECTORY "${GLIM_JSON_INCLUDE_DIR}/nlohmann" OR
   NOT EXISTS "${GLIM_JSON_INCLUDE_DIR}/nlohmann/json.hpp")
  message(FATAL_ERROR
    "Phase 3b requires the exact pinned GLIM nlohmann/json include directory")
endif()

list(PREPEND CMAKE_PREFIX_PATH
  "${GLIM_INSTALL_PREFIX}"
  "${GTSAM_POINTS_INSTALL_PREFIX}"
  "${GTSAM_INSTALL_PREFIX}")
find_package(glim CONFIG REQUIRED NO_DEFAULT_PATH
  PATHS "${GLIM_INSTALL_PREFIX}/lib/cmake/glim")

add_library(glim_clean_room_phase3b_runtime STATIC
  src/phase3b_session.cpp
  ../phase3a/src/phase3a_conversion.cpp
  ../src/contract.cpp)
target_include_directories(glim_clean_room_phase3b_runtime PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../phase3a/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "${GLIM_JSON_INCLUDE_DIR}"
  "${GLIM_INSTALL_PREFIX}/include"
  "${GTSAM_INSTALL_PREFIX}/include"
  "${GTSAM_POINTS_INSTALL_PREFIX}/include")
target_compile_features(glim_clean_room_phase3b_runtime PUBLIC cxx_std_17)
target_compile_options(glim_clean_room_phase3b_runtime PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
target_compile_definitions(glim_clean_room_phase3b_runtime PRIVATE
  GLIM_CLEAN_ROOM_TESTING=1)
target_link_libraries(glim_clean_room_phase3b_runtime PUBLIC
  glim::glim
  glim::odometry_estimation_cpu)

if(GLIM_CLEAN_ROOM_PHASE3B_SANITIZERS)
  target_compile_options(glim_clean_room_phase3b_runtime PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(glim_clean_room_phase3b_runtime PRIVATE
    -fsanitize=address,undefined)
endif()

add_executable(glim_clean_room_phase3b_runtime_tests
  tests/phase3b_session_tests.cpp)
target_link_libraries(glim_clean_room_phase3b_runtime_tests PRIVATE
  glim_clean_room_phase3b_runtime)
target_compile_features(glim_clean_room_phase3b_runtime_tests PRIVATE cxx_std_17)
target_compile_options(glim_clean_room_phase3b_runtime_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(glim_clean_room_phase3b_runtime_tests PRIVATE
  GLIM_CLEAN_ROOM_TESTING=1
  GLIM_CLEAN_ROOM_PHASE3B_TEST_ROOT="${CMAKE_CURRENT_BINARY_DIR}/runtime-fixture")
if(GLIM_CLEAN_ROOM_PHASE3B_SANITIZERS)
  target_compile_options(glim_clean_room_phase3b_runtime_tests PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(glim_clean_room_phase3b_runtime_tests PRIVATE
    -fsanitize=address,undefined)
endif()
set_target_properties(glim_clean_room_phase3b_runtime_tests PROPERTIES
  BUILD_RPATH
    "${GLIM_INSTALL_PREFIX}/lib;${GTSAM_POINTS_INSTALL_PREFIX}/lib;${GTSAM_INSTALL_PREFIX}/lib")

enable_testing()
add_test(NAME glim_clean_room_phase3b_runtime_tests
  COMMAND glim_clean_room_phase3b_runtime_tests)
set_tests_properties(glim_clean_room_phase3b_runtime_tests PROPERTIES
  ENVIRONMENT
    "LD_LIBRARY_PATH=${GLIM_INSTALL_PREFIX}/lib:${GTSAM_POINTS_INSTALL_PREFIX}/lib:${GTSAM_INSTALL_PREFIX}/lib")
add_test(NAME glim_clean_room_phase3b_source_guard
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tests/source_guard.py"
          "${CMAKE_CURRENT_SOURCE_DIR}/..")
