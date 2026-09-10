cmake_minimum_required(VERSION 3.16)
project(glim_clean_room_adapter VERSION 0.1.0 LANGUAGES CXX)

option(GLIM_CLEAN_ROOM_SANITIZERS
  "Enable AddressSanitizer and UndefinedBehaviorSanitizer for contract tests"
  OFF)

# This tree is deliberately independent of the ROS bridge.  The default
# target has no ROS or GLIM dependency and exercises only the host-owned,
# typed boundary.  GLIM_CORE_* are opt-in inputs for an exact upstream public
# API compile/link smoke; this project never fetches or builds a dependency.
set(GLIM_CORE_COMMIT "faa264a1bce1bda406f73457e35511f56cdc2eaa" CACHE STRING
    "Expected immutable GLIM core commit for the opt-in smoke")
set(GLIM_CORE_INCLUDE_DIR "" CACHE PATH
    "Pinned GLIM core include directory (contains glim/util/raw_points.hpp)")
set(GLIM_CORE_LIBRARY "" CACHE FILEPATH
    "Pinned GLIM core shared/static library for the opt-in link smoke")
set(GLIM_CORE_LINK_LIBRARIES "" CACHE STRING
    "Additional already-pinned GLIM dependency libraries for the opt-in smoke")
set(GLIM_CORE_CMAKE_PREFIX "" CACHE PATH
    "Pinned GLIM core install prefix for an exported target link smoke")
set(GLIM_BOOST_PREFIX "" CACHE PATH
    "Pinned Boost install prefix used by exported GLIM/GTSAM targets")

if(GLIM_BOOST_PREFIX)
  if(NOT IS_DIRECTORY "${GLIM_BOOST_PREFIX}")
    message(FATAL_ERROR "GLIM_BOOST_PREFIX must be an existing pinned install prefix")
  endif()
  if(NOT EXISTS "${GLIM_BOOST_PREFIX}/include/boost/version.hpp")
    message(FATAL_ERROR "GLIM_BOOST_PREFIX does not contain Boost headers")
  endif()
  if(NOT IS_DIRECTORY "${GLIM_BOOST_PREFIX}/lib")
    message(FATAL_ERROR "GLIM_BOOST_PREFIX does not contain a lib directory")
  endif()
  set(BOOST_ROOT "${GLIM_BOOST_PREFIX}" CACHE PATH
      "Pinned Boost root for dependency discovery" FORCE)
  set(BOOST_INCLUDEDIR "${GLIM_BOOST_PREFIX}/include" CACHE PATH
      "Pinned Boost include directory" FORCE)
  set(BOOST_LIBRARYDIR "${GLIM_BOOST_PREFIX}/lib" CACHE PATH
      "Pinned Boost library directory" FORCE)
  set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL
      "Do not fall back to system Boost when a pinned prefix is supplied" FORCE)
endif()

string(LENGTH "${GLIM_CORE_COMMIT}" GLIM_CORE_COMMIT_LENGTH)
if(NOT GLIM_CORE_COMMIT_LENGTH EQUAL 40 OR
   GLIM_CORE_COMMIT MATCHES "[^0-9a-f]")
  message(FATAL_ERROR "GLIM_CORE_COMMIT must be a 40-hex immutable commit")
endif()

add_library(glim_clean_room_contract INTERFACE)
target_include_directories(glim_clean_room_contract INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_compile_features(glim_clean_room_contract INTERFACE cxx_std_17)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

enable_testing()
add_executable(glim_clean_room_contract_tests
  tests/contract_tests.cpp
  src/contract.cpp
  src/pointcloud2_parser.cpp)
target_link_libraries(glim_clean_room_contract_tests PRIVATE glim_clean_room_contract)
target_compile_features(glim_clean_room_contract_tests PRIVATE cxx_std_17)
target_compile_definitions(glim_clean_room_contract_tests PRIVATE
  GLIM_CLEAN_ROOM_TESTING=1)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(glim_clean_room_contract_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
  if(GLIM_CLEAN_ROOM_SANITIZERS)
    target_compile_options(glim_clean_room_contract_tests PRIVATE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(glim_clean_room_contract_tests PRIVATE
      -fsanitize=address,undefined)
  endif()
endif()
add_test(NAME glim_clean_room_contract_tests
  COMMAND glim_clean_room_contract_tests)

add_test(NAME glim_clean_room_source_guard
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/source_guard.py"
          "${CMAKE_CURRENT_SOURCE_DIR}")

if(GLIM_CORE_CMAKE_PREFIX)
  if(NOT IS_DIRECTORY "${GLIM_CORE_CMAKE_PREFIX}")
    message(FATAL_ERROR
      "GLIM_CORE_CMAKE_PREFIX must be an existing pinned install prefix")
  endif()
  if(NOT EXISTS "${GLIM_CORE_CMAKE_PREFIX}/include/glim/util/raw_points.hpp")
    message(FATAL_ERROR
      "GLIM_CORE_CMAKE_PREFIX does not contain the pinned public RawPoints header")
  endif()
  find_package(glim CONFIG REQUIRED NO_DEFAULT_PATH
    PATHS
      "${GLIM_CORE_CMAKE_PREFIX}/lib/cmake/glim"
      "${GLIM_CORE_CMAKE_PREFIX}/share/glim")
  if(NOT TARGET glim::glim)
    message(FATAL_ERROR "GLIM install does not export the glim::glim target")
  endif()
  add_executable(glim_core_public_api_link_smoke
    tests/glim_core_public_api_link_smoke.cpp)
  target_include_directories(glim_core_public_api_link_smoke PRIVATE
    "${GLIM_CORE_CMAKE_PREFIX}/include")
  target_compile_features(glim_core_public_api_link_smoke PRIVATE cxx_std_17)
  target_link_libraries(glim_core_public_api_link_smoke PRIVATE glim::glim)
  target_compile_definitions(glim_core_public_api_link_smoke PRIVATE
    GLIM_CLEAN_ROOM_EXPECTED_COMMIT="${GLIM_CORE_COMMIT}")
  add_test(NAME glim_core_public_api_link_smoke
    COMMAND glim_core_public_api_link_smoke)
  message(STATUS
    "Configured exported-target GLIM public API link smoke for ${GLIM_CORE_COMMIT}")
elseif(GLIM_CORE_INCLUDE_DIR OR GLIM_CORE_LIBRARY OR GLIM_CORE_LINK_LIBRARIES)
  if(NOT GLIM_CORE_INCLUDE_DIR OR NOT IS_DIRECTORY "${GLIM_CORE_INCLUDE_DIR}")
    message(FATAL_ERROR
      "GLIM_CORE_INCLUDE_DIR must be an existing pinned include directory")
  endif()
  if(NOT EXISTS "${GLIM_CORE_INCLUDE_DIR}/glim/util/raw_points.hpp")
    message(FATAL_ERROR
      "GLIM_CORE_INCLUDE_DIR does not contain the pinned public RawPoints header")
  endif()
  if(NOT GLIM_CORE_LIBRARY OR NOT EXISTS "${GLIM_CORE_LIBRARY}")
    message(FATAL_ERROR
      "GLIM_CORE_LIBRARY must be an existing pinned library for Phase 1")
  endif()
  add_executable(glim_core_public_api_link_smoke
    tests/glim_core_public_api_link_smoke.cpp)
  target_include_directories(glim_core_public_api_link_smoke PRIVATE
    "${GLIM_CORE_INCLUDE_DIR}")
  target_compile_features(glim_core_public_api_link_smoke PRIVATE cxx_std_17)
  target_link_libraries(glim_core_public_api_link_smoke PRIVATE
    "${GLIM_CORE_LIBRARY}" ${GLIM_CORE_LINK_LIBRARIES})
  target_compile_definitions(glim_core_public_api_link_smoke PRIVATE
    GLIM_CLEAN_ROOM_EXPECTED_COMMIT="${GLIM_CORE_COMMIT}")
  add_test(NAME glim_core_public_api_link_smoke
    COMMAND glim_core_public_api_link_smoke)
  message(STATUS "Configured opt-in GLIM public API link smoke for ${GLIM_CORE_COMMIT}")
else()
  message(STATUS
    "GLIM core not supplied; Phase 1 public API link smoke is NOT_RUN (Phase 0 only)")
endif()
