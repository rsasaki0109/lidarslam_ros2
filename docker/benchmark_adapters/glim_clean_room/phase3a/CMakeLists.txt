cmake_minimum_required(VERSION 3.16)
project(glim_clean_room_phase3a_conversion LANGUAGES CXX)

option(GLIM_CLEAN_ROOM_PHASE3A_SANITIZERS
  "Enable AddressSanitizer and UndefinedBehaviorSanitizer"
  OFF)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GLIM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 GLIM install prefix used by the opt-in conversion target")
set(GTSAM_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 GTSAM install prefix")
set(GTSAM_POINTS_INSTALL_PREFIX "" CACHE PATH
  "Preserved Phase 1 gtsam_points install prefix")

if(NOT IS_DIRECTORY "${GLIM_INSTALL_PREFIX}" OR
   NOT EXISTS "${GLIM_INSTALL_PREFIX}/include/glim/odometry/estimation_frame.hpp")
  message(FATAL_ERROR
    "GLIM_INSTALL_PREFIX must be the preserved exact-core install prefix")
endif()
if(NOT IS_DIRECTORY "${GTSAM_INSTALL_PREFIX}" OR
   NOT IS_DIRECTORY "${GTSAM_POINTS_INSTALL_PREFIX}")
  message(FATAL_ERROR
    "Phase 3a requires the preserved GTSAM and gtsam_points prefixes")
endif()

list(PREPEND CMAKE_PREFIX_PATH
  "${GLIM_INSTALL_PREFIX}"
  "${GTSAM_POINTS_INSTALL_PREFIX}"
  "${GTSAM_INSTALL_PREFIX}")
find_package(glim CONFIG REQUIRED NO_DEFAULT_PATH
  PATHS "${GLIM_INSTALL_PREFIX}/lib/cmake/glim")

add_library(glim_clean_room_phase3a_conversion STATIC
  src/phase3a_conversion.cpp
  ../src/contract.cpp)
target_include_directories(glim_clean_room_phase3a_conversion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "${GLIM_INSTALL_PREFIX}/include"
  "${GTSAM_INSTALL_PREFIX}/include"
  "${GTSAM_POINTS_INSTALL_PREFIX}/include")
target_compile_features(glim_clean_room_phase3a_conversion PUBLIC cxx_std_17)
target_compile_options(glim_clean_room_phase3a_conversion PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
target_link_libraries(glim_clean_room_phase3a_conversion PUBLIC
  glim::glim)
if(GLIM_CLEAN_ROOM_PHASE3A_SANITIZERS)
  target_compile_options(glim_clean_room_phase3a_conversion PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(glim_clean_room_phase3a_conversion PRIVATE
    -fsanitize=address,undefined)
endif()

add_executable(glim_clean_room_phase3a_conversion_tests
  tests/phase3a_conversion_tests.cpp)
target_link_libraries(glim_clean_room_phase3a_conversion_tests PRIVATE
  glim_clean_room_phase3a_conversion)
target_compile_features(glim_clean_room_phase3a_conversion_tests PRIVATE cxx_std_17)
target_compile_options(glim_clean_room_phase3a_conversion_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
if(GLIM_CLEAN_ROOM_PHASE3A_SANITIZERS)
  target_compile_options(glim_clean_room_phase3a_conversion_tests PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(glim_clean_room_phase3a_conversion_tests PRIVATE
    -fsanitize=address,undefined)
endif()
set_target_properties(glim_clean_room_phase3a_conversion_tests PROPERTIES
  BUILD_RPATH
    "${GLIM_INSTALL_PREFIX}/lib;${GTSAM_POINTS_INSTALL_PREFIX}/lib;${GTSAM_INSTALL_PREFIX}/lib")

enable_testing()
add_test(NAME glim_clean_room_phase3a_conversion_tests
  COMMAND glim_clean_room_phase3a_conversion_tests)
set_tests_properties(glim_clean_room_phase3a_conversion_tests PROPERTIES
  ENVIRONMENT
    "LD_LIBRARY_PATH=${GLIM_INSTALL_PREFIX}/lib:${GTSAM_POINTS_INSTALL_PREFIX}/lib:${GTSAM_INSTALL_PREFIX}/lib")
add_test(NAME glim_clean_room_phase3a_source_guard
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tests/source_guard.py"
          "${CMAKE_CURRENT_SOURCE_DIR}/..")
