cmake_minimum_required(VERSION 3.5)
project(graph_based_slam)

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
  #add_compile_options(-Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
add_compile_definitions(G2O_USE_VENDORED_CERES)

# Find missing symbols during link time
add_link_options(-Wl,--no-undefined)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED)
find_package(g2o QUIET
  COMPONENTS
    g2o::core
    g2o::types_slam3d
    g2o::solver_eigen
)
if(NOT g2o_FOUND)
  # Fallback: find g2o libraries directly (Ubuntu/Debian packages without cmake config)
  find_path(G2O_INCLUDE_DIR g2o/core/sparse_optimizer.h PATHS /usr/include /usr/local/include)
  find_library(G2O_CORE_LIB g2o_core)
  find_library(G2O_TYPES_SLAM3D_LIB g2o_types_slam3d)
  find_library(G2O_SOLVER_EIGEN_LIB g2o_solver_eigen)
  find_library(G2O_STUFF_LIB g2o_stuff)
  if(G2O_INCLUDE_DIR AND G2O_CORE_LIB AND G2O_TYPES_SLAM3D_LIB AND G2O_SOLVER_EIGEN_LIB)
    message(STATUS "Found g2o via fallback: ${G2O_CORE_LIB}")
    # Create imported targets to match CMake config mode
    foreach(_tgt core types_slam3d solver_eigen stuff)
      string(TOUPPER "${_tgt}" _TGT_UPPER)
      if(NOT TARGET g2o::${_tgt})
        add_library(g2o::${_tgt} SHARED IMPORTED)
        set_target_properties(g2o::${_tgt} PROPERTIES
          IMPORTED_LOCATION "${G2O_${_TGT_UPPER}_LIB}"
          INTERFACE_INCLUDE_DIRECTORIES "${G2O_INCLUDE_DIR}"
        )
      endif()
    endforeach()
    set(g2o_FOUND TRUE)
  else()
    message(FATAL_ERROR "Could not find g2o libraries")
  endif()
endif()
find_package(OpenMP)

option(
  GRAPH_BASED_SLAM_ENABLE_3D_BBS
  "Build optional 3D-BBS loop-closure verifier support"
  ON
)
set(
  GRAPH_BASED_SLAM_3D_BBS_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/3d_bbs"
  CACHE PATH
  "Path to the vendored 3D-BBS source tree"
)

set(GRAPH_BASED_SLAM_HAVE_3D_BBS FALSE)
if(GRAPH_BASED_SLAM_ENABLE_3D_BBS AND
  EXISTS "${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/include/cpu_bbs3d/bbs3d.hpp")
  add_library(graph_based_slam_3d_bbs_vendor STATIC
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/src/cpu_bbs3d/bbs3d.cpp
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps.cpp
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
  )
  target_include_directories(graph_based_slam_3d_bbs_vendor PUBLIC
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/include
    ${EIGEN3_INCLUDE_DIR}
  )
  if(OPENMP_FOUND)
    target_link_libraries(graph_based_slam_3d_bbs_vendor PUBLIC OpenMP::OpenMP_CXX)
  endif()
  set(GRAPH_BASED_SLAM_HAVE_3D_BBS TRUE)
  message(STATUS "Building graph_based_slam with optional 3D-BBS support")
else()
  message(STATUS "Building graph_based_slam without optional 3D-BBS support")
endif()

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(graph_slam_config STATIC
  src/graph_slam_config.cpp
)
set_target_properties(graph_slam_config PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(graph_slam_config PRIVATE src)
ament_target_dependencies(graph_slam_config rclcpp)

add_library(graph_slam_application STATIC
  src/graph_slam_application.cpp
)
set_target_properties(graph_slam_application PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  graph_slam_application PUBLIC include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(graph_slam_application ndt_omp_ros2)
target_link_libraries(
  graph_slam_application
  ${PCL_LIBRARIES}
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
  g2o::stuff
)

add_library(graph_slam_io_adapters STATIC
  src/filesystem_io_ports.cpp
)
set_target_properties(graph_slam_io_adapters PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(graph_slam_io_adapters PRIVATE cxx_std_17)
target_include_directories(
  graph_slam_io_adapters PUBLIC include PRIVATE src ${PCL_INCLUDE_DIRS}
)
target_link_libraries(graph_slam_io_adapters ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  if(DEFINED ENV{ROS_DISTRO} AND "$ENV{ROS_DISTRO}" STREQUAL "humble")
    # humble ships an older uncrustify binary whose formatting output diverges
    # from the version used on newer distros (e.g. jazzy) for a handful of
    # files. Since a single formatting cannot satisfy both uncrustify
    # versions at once, skip the redundant style check on humble only;
    # jazzy (and any other supported distro) still enforces it.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  endif()
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_graph_slam_architecture
    test/test_graph_slam_architecture.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_bounded_filesystem_exhaustion
    test/test_bounded_filesystem_exhaustion.py
    TIMEOUT 120
  )
  ament_add_gtest(
    test_graph_slam_config
    test/test_graph_slam_config.cpp
  )
  if(TARGET test_graph_slam_config)
    target_include_directories(test_graph_slam_config PRIVATE src)
    target_link_libraries(test_graph_slam_config graph_slam_config)
    ament_target_dependencies(test_graph_slam_config rclcpp)
  endif()
  ament_add_gtest(
    test_gnss_weighting
    test/test_gnss_weighting.cpp
  )
  if(TARGET test_gnss_weighting)
    target_include_directories(test_gnss_weighting PRIVATE include)
    ament_target_dependencies(test_gnss_weighting sensor_msgs)
  endif()
  ament_add_gtest(
    test_gnss_geometry
    test/test_gnss_geometry.cpp
  )
  if(TARGET test_gnss_geometry)
    target_include_directories(test_gnss_geometry PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_gnss_origin_accumulator
    test/test_gnss_origin_accumulator.cpp
  )
  if(TARGET test_gnss_origin_accumulator)
    target_include_directories(test_gnss_origin_accumulator PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_adjacent_edge_auto_scale
    test/test_adjacent_edge_auto_scale.cpp
  )
  if(TARGET test_adjacent_edge_auto_scale)
    target_include_directories(test_adjacent_edge_auto_scale PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_scan_context
    test/test_scan_context.cpp
  )
  if(TARGET test_scan_context)
    target_include_directories(test_scan_context PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_solid_descriptor
    test/test_solid_descriptor.cpp
  )
  if(TARGET test_solid_descriptor)
    target_include_directories(test_solid_descriptor PRIVATE include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_solid_descriptor ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_submap_bev_descriptor
    test/test_submap_bev_descriptor.cpp
  )
  if(TARGET test_submap_bev_descriptor)
    target_include_directories(test_submap_bev_descriptor PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_bev_mutual_visibility
    test/test_bev_mutual_visibility.cpp
  )
  if(TARGET test_bev_mutual_visibility)
    target_include_directories(test_bev_mutual_visibility PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_loop_edge_robustifier
    test/test_loop_edge_robustifier.cpp
  )
  if(TARGET test_loop_edge_robustifier)
    target_include_directories(test_loop_edge_robustifier PRIVATE include)
  endif()
  ament_add_gtest(
    test_loop_search_schedule
    test/test_loop_search_schedule.cpp
  )
  if(TARGET test_loop_search_schedule)
    target_include_directories(test_loop_search_schedule PRIVATE include)
  endif()
  ament_add_gtest(
    test_serialized_work_drain
    test/test_serialized_work_drain.cpp
  )
  if(TARGET test_serialized_work_drain)
    target_include_directories(test_serialized_work_drain PRIVATE include)
  endif()
  ament_add_gtest(
    test_graph_state_store
    test/test_graph_state_store.cpp
  )
  if(TARGET test_graph_state_store)
    target_include_directories(test_graph_state_store PRIVATE include ${EIGEN3_INCLUDE_DIR})
    ament_target_dependencies(test_graph_state_store lidarslam_msgs)
  endif()
  ament_add_gtest(
    test_triangle_descriptor
    test/test_triangle_descriptor.cpp
  )
  if(TARGET test_triangle_descriptor)
    target_include_directories(test_triangle_descriptor PRIVATE include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_triangle_descriptor ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_triangle_descriptor_database
    test/test_triangle_descriptor_database.cpp
  )
  if(TARGET test_triangle_descriptor_database)
    target_include_directories(
      test_triangle_descriptor_database PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(test_triangle_descriptor_database ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_submap_creation
    test/test_submap_creation.cpp
  )
  if(TARGET test_submap_creation)
    target_include_directories(test_submap_creation PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_gnss_alignment
    test/test_gnss_alignment.cpp
  )
  if(TARGET test_gnss_alignment)
    target_include_directories(test_gnss_alignment PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_candidate_aggregator
    test/test_candidate_aggregator.cpp
  )
  if(TARGET test_candidate_aggregator)
    target_include_directories(test_candidate_aggregator PRIVATE include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_candidate_aggregator ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_loop_verifier
    test/test_loop_verifier.cpp
  )
  if(TARGET test_loop_verifier)
    target_include_directories(test_loop_verifier PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_map_saver
    test/test_map_saver.cpp
  )
  if(TARGET test_map_saver)
    target_include_directories(test_map_saver PRIVATE include)
  endif()
  ament_add_gtest(
    test_backend_core
    test/test_backend_core.cpp
    src/three_d_bbs_loop_verifier.cpp
  )
  if(TARGET test_backend_core)
    target_include_directories(test_backend_core PRIVATE include ${EIGEN3_INCLUDE_DIR})
    ament_target_dependencies(test_backend_core ndt_omp_ros2)
    target_link_libraries(
      test_backend_core ${PCL_LIBRARIES} graph_based_slam_3d_bbs_vendor
    )
  endif()
  ament_add_gtest(
    test_graph_slam_application
    test/test_graph_slam_application.cpp
    src/three_d_bbs_loop_verifier.cpp
  )
  if(TARGET test_graph_slam_application)
    target_include_directories(
      test_graph_slam_application PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
    ament_target_dependencies(test_graph_slam_application ndt_omp_ros2)
    target_link_libraries(
      test_graph_slam_application
      graph_slam_application
      ${PCL_LIBRARIES}
      graph_based_slam_3d_bbs_vendor
    )
  endif()
  ament_add_gtest(
    test_external_io_ports
    test/test_external_io_ports.cpp
  )
  if(TARGET test_external_io_ports)
    target_include_directories(
      test_external_io_ports PRIVATE include src ${PCL_INCLUDE_DIRS}
    )
    target_compile_features(test_external_io_ports PRIVATE cxx_std_17)
    target_link_libraries(
      test_external_io_ports graph_slam_io_adapters ${PCL_LIBRARIES}
    )
  endif()
  ament_add_gtest(
    test_pointcloud_conversion
    test/test_pointcloud_conversion.cpp
  )
  if(TARGET test_pointcloud_conversion)
    target_include_directories(
      test_pointcloud_conversion PRIVATE include ${PCL_INCLUDE_DIRS}
    )
    ament_target_dependencies(
      test_pointcloud_conversion sensor_msgs pcl_conversions
    )
    target_link_libraries(test_pointcloud_conversion ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_pose_graph_optimization
    test/test_pose_graph_optimization.cpp
  )
  if(TARGET test_pose_graph_optimization)
    target_include_directories(
      test_pose_graph_optimization PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(
      test_pose_graph_optimization
      g2o::core
      g2o::types_slam3d
      g2o::solver_eigen
      g2o::stuff
    )
  endif()
  ament_add_gtest(
    test_se3_lie
    test/test_se3_lie.cpp
  )
  if(TARGET test_se3_lie)
    target_include_directories(test_se3_lie PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_point_cluster
    test/test_point_cluster.cpp
  )
  if(TARGET test_point_cluster)
    target_include_directories(test_point_cluster PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_scatter_eigen_cost
    test/test_scatter_eigen_cost.cpp
  )
  if(TARGET test_scatter_eigen_cost)
    target_include_directories(test_scatter_eigen_cost PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_plane_ba
    test/test_plane_ba.cpp
  )
  if(TARGET test_plane_ba)
    target_include_directories(test_plane_ba PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_synthetic_degeneracy_fixtures
    test/test_synthetic_degeneracy_fixtures.cpp
  )
  if(TARGET test_synthetic_degeneracy_fixtures)
    target_include_directories(
      test_synthetic_degeneracy_fixtures PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_localizability_analysis
    test/test_localizability_analysis.cpp
  )
  if(TARGET test_localizability_analysis)
    target_include_directories(
      test_localizability_analysis PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_degeneracy_aware_solve
    test/test_degeneracy_aware_solve.cpp
  )
  if(TARGET test_degeneracy_aware_solve)
    target_include_directories(
      test_degeneracy_aware_solve PRIVATE include ../Thirdparty/rko_lio ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_persistent_weak_direction
    test/test_persistent_weak_direction.cpp
  )
  if(TARGET test_persistent_weak_direction)
    target_include_directories(
      test_persistent_weak_direction PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_rko_multiscan_observability
    test/test_rko_multiscan_observability.cpp
  )
  if(TARGET test_rko_multiscan_observability)
    target_include_directories(
      test_rko_multiscan_observability PRIVATE ../Thirdparty/rko_lio ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_selective_visual_fusion
    test/test_selective_visual_fusion.cpp
  )
  if(TARGET test_selective_visual_fusion)
    target_include_directories(
      test_selective_visual_fusion PRIVATE ../Thirdparty/rko_lio ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_radar_ego_velocity
    test/test_radar_ego_velocity.cpp
  )
  if(TARGET test_radar_ego_velocity)
    target_include_directories(
      test_radar_ego_velocity PRIVATE ../Thirdparty/rko_lio ${EIGEN3_INCLUDE_DIR}
    )
    # radar_ego_velocity.hpp uses std::optional; this package otherwise defaults to C++14.
    target_compile_features(test_radar_ego_velocity PRIVATE cxx_std_17)
  endif()
  ament_add_gtest(
    test_radar_continuous_fusion
    test/test_radar_continuous_fusion.cpp
  )
  if(TARGET test_radar_continuous_fusion)
    target_include_directories(
      test_radar_continuous_fusion PRIVATE ../Thirdparty/rko_lio ${EIGEN3_INCLUDE_DIR}
    )
    # radar_ego_velocity.hpp uses std::optional; this package otherwise defaults to C++14
    # (see test_radar_ego_velocity above).
    target_compile_features(test_radar_continuous_fusion PRIVATE cxx_std_17)
  endif()
  ament_add_gtest(
    test_intensity_profile
    test/test_intensity_profile.cpp
  )
  if(TARGET test_intensity_profile)
    target_include_directories(
      test_intensity_profile PRIVATE ../Thirdparty/rko_lio ${EIGEN3_INCLUDE_DIR}
    )
    # intensity_profile.hpp uses std::clamp/structured bindings; this package otherwise
    # defaults to C++14 (see test_radar_ego_velocity above).
    target_compile_features(test_intensity_profile PRIVATE cxx_std_17)
  endif()
  ament_add_gtest(
    test_odometry_covariance_localizability
    test/test_odometry_covariance_localizability.cpp
  )
  if(TARGET test_odometry_covariance_localizability)
    target_include_directories(
      test_odometry_covariance_localizability PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_degeneracy_diagnostics_csv
    test/test_degeneracy_diagnostics_csv.cpp
  )
  if(TARGET test_degeneracy_diagnostics_csv)
    target_include_directories(
      test_degeneracy_diagnostics_csv PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_degeneracy_report_summary
    test/test_degeneracy_report_summary.cpp
  )
  if(TARGET test_degeneracy_report_summary)
    target_include_directories(
      test_degeneracy_report_summary PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_map_refiner
    test/test_map_refiner.cpp
  )
  if(TARGET test_map_refiner)
    target_include_directories(test_map_refiner PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_hba_pyramid
    test/test_hba_pyramid.cpp
  )
  if(TARGET test_hba_pyramid)
    target_include_directories(test_hba_pyramid PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_plane_feature_association
    test/test_plane_feature_association.cpp
  )
  if(TARGET test_plane_feature_association)
    target_include_directories(
      test_plane_feature_association PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_adaptive_voxel_plane_extractor
    test/test_adaptive_voxel_plane_extractor.cpp
  )
  if(TARGET test_adaptive_voxel_plane_extractor)
    target_include_directories(
      test_adaptive_voxel_plane_extractor PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_map_quality_metrics
    test/test_map_quality_metrics.cpp
  )
  if(TARGET test_map_quality_metrics)
    target_include_directories(
      test_map_quality_metrics PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_registration_determinism
    test/test_registration_determinism.cpp
  )
  if(TARGET test_registration_determinism)
    target_include_directories(
      test_registration_determinism PRIVATE ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_registration_determinism ndt_omp_ros2)
    target_link_libraries(test_registration_determinism ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_dynamic_object_filter
    test/test_dynamic_object_filter.cpp
  )
  if(TARGET test_dynamic_object_filter)
    target_include_directories(
      test_dynamic_object_filter PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_gtest(
    test_planar_map_filter
    test/test_planar_map_filter.cpp
  )
  if(TARGET test_planar_map_filter)
    target_include_directories(
      test_planar_map_filter PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_pytest_test(
    test_autoware_map_tools
    test/test_autoware_map_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_bundle_verifier
    test/test_map_bundle_verifier.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_edit
    test/test_map_edit.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_merge
    test/test_map_merge.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_competitive_slam_profile
    test/test_competitive_slam_profile.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_competitive_candidate_provenance
    test/test_competitive_candidate_provenance.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_fast_livo2_benchmark_runner
    test/test_fast_livo2_benchmark_runner.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_glim_benchmark_runner
    test/test_glim_benchmark_runner.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_convert_ply_xyz_to_pcd
    test/test_convert_ply_xyz_to_pcd.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_freeze_hilti2022_holdout_inputs
    test/test_freeze_hilti2022_holdout_inputs.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_compare_rosbag_semantic_inputs
    test/test_compare_rosbag_semantic_inputs.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_validate_fast_livo2_processing_rtf
    test/test_validate_fast_livo2_processing_rtf.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_compare_checkpoint_trajectory_errors
    test/test_compare_checkpoint_trajectory_errors.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_summarize_repeated_map_quality
    test/test_summarize_repeated_map_quality.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_summarize_repeated_visual_quality
    test/test_summarize_repeated_visual_quality.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_repose_posed_images
    test/test_repose_posed_images.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_extract_selective_visual_constraints
    test/test_extract_selective_visual_constraints.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_evaluate_competitive_sequence_gate
    test/test_evaluate_competitive_sequence_gate.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_compose_competitive_result
    test/test_compose_competitive_result.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_evaluate_competitive_suite_gate
    test/test_evaluate_competitive_suite_gate.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_quality_profiles
    test/test_map_quality_profiles.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_summarize_degeneracy_csv
    test/test_summarize_degeneracy_csv.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_tools
    test/test_mid360_robot_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_loop_alignment_analyzer
    test/test_mid360_robot_loop_alignment_analyzer.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_dashboard
    test/test_mid360_robot_dashboard.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_production_candidate_bundle
    test/test_mid360_robot_production_candidate_bundle.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_record_tools
    test/test_mid360_robot_record_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_production_readiness
    test/test_mid360_robot_production_readiness.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_rko_config_adoption
    test/test_mid360_robot_rko_config_adoption.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_rko_quality_report
    test/test_mid360_robot_public_rko_quality_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_rko_sweep
    test/test_mid360_robot_public_rko_sweep.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_rko_adoption_gate
    test/test_mid360_robot_public_rko_adoption_gate.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_production_candidate_session
    test/test_mid360_robot_production_candidate_session.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_production_candidate_bundle_import
    test/test_mid360_robot_production_candidate_bundle_import.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_recording_check
    test/test_mid360_robot_recording_check.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_jetson_mid360_host_readiness
    test/test_jetson_mid360_host_readiness.py
    TIMEOUT 120
  )
  # NOTE: the test_gaussian_splatting_* suites below cover tools/gaussian_splatting/
  # (the opt-in 3DGS post-process toolkit), NOT this package. They live here only
  # to borrow graph_based_slam's ament pytest harness, and reach the code under
  # test via Path(__file__).parents[2] + sys.path. Consequences a maintainer
  # should know: (1) `colcon test --packages-select graph_based_slam` runs these
  # gsplat tests, and (2) moving/renaming tools/gaussian_splatting/ silently
  # breaks the parents[2] path here. Relocating them to a dedicated test target
  # next to the tool is tracked as a v0.4 structural cleanup.
  ament_add_pytest_test(
    test_gaussian_splatting_posed_images
    test/test_gaussian_splatting_posed_images.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_extract
    test/test_gaussian_splatting_extract.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_train
    test/test_gaussian_splatting_train.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_pointcloud
    test/test_gaussian_splatting_pointcloud.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_lidar_camera_alignment
    test/test_lidar_camera_alignment.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_spatiotemporal_calibration
    test/test_spatiotemporal_calibration.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_geometry_aware_fusion
    test/test_geometry_aware_fusion.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_attach_dynamic_image_masks
    test/test_attach_dynamic_image_masks.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_recolor_pointcloud
    test/test_recolor_pointcloud.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_heldout_point_colors
    test/test_heldout_point_colors.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_colored_map_quality_gate
    test/test_colored_map_quality_gate.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_render
    test/test_gaussian_splatting_render.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_sidebyside
    test/test_gaussian_splatting_sidebyside.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_map_flythrough
    test/test_gaussian_splatting_map_flythrough.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_ply_to_splat
    test/test_gaussian_splatting_ply_to_splat.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_sim2real_gap
    test/test_gaussian_splatting_sim2real_gap.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_renderer
    test/test_gaussian_splatting_renderer.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_dataset
    test/test_gaussian_splatting_dataset.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_actor
    test/test_gaussian_splatting_actor.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_sprite
    test/test_gaussian_splatting_sprite.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_detect
    test/test_gaussian_splatting_detect.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_drive_env
    test/test_gaussian_splatting_drive_env.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_gaussian_splatting_scene_camera
    test/test_gaussian_splatting_scene_camera.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_preflight
    test/test_mid360_robot_preflight.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_sample_bag
    test/test_mid360_robot_sample_bag.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_bag_stamp_rewriter
    test/test_mid360_robot_bag_stamp_rewriter.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_split_bag_merge
    test/test_mid360_robot_public_split_bag_merge.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_loop_cloud_analyzer
    test/test_mid360_robot_public_loop_cloud_analyzer.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_loop_segment_reset
    test/test_mid360_robot_public_loop_segment_reset.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_segment_map_cloud_alignment
    test/test_mid360_robot_public_segment_map_cloud_alignment.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_completion_gate
    test/test_mid360_robot_public_completion_gate.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_mid360_robot_public_continuous_relocalization_gate
    test/test_mid360_robot_public_continuous_relocalization_gate.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_navsatfix_covariance_inspector
    test/test_navsatfix_covariance_inspector.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_gsof50_inspector
    test/test_applanix_gsof50_inspector.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_navsatfix_bridge
    test/test_applanix_navsatfix_bridge.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_imu_bridge
    test/test_applanix_imu_bridge.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_static_transform_extractor
    test/test_static_transform_extractor.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_gsof49_reference_extractor
    test/test_applanix_gsof49_reference_extractor.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rtk_slam_reference
    test/test_rtk_slam_reference.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_newer_college_reference
    test/test_newer_college_reference.py
    TIMEOUT 120
  )
  set(GRAPH_BASED_SLAM_ADDITIONAL_PYTESTS
    test_aist_ouster_rgb_map_benchmark
    test_analyze_colored_point_cloud
    test_applanix_tf_bridge
    test_attached_storage
    test_autoware_dogfood_script
    test_autoware_map_preflight
    test_autoware_map_run_diagnosis
    test_autoware_quickstart_script
    test_autoware_viewer_scripts
    test_benchmark_report_modules
    test_colored_map_appearance
    test_colored_map_pipeline
    test_colorize_from_bag
    test_compare_fast_livo2
    test_cross_repo_slam_benchmark
    test_densify_corrected_trajectory
    test_docker_demo_script
    test_download_verified_file
    test_freeze_ntu_viral_holdout_inputs
    test_freeze_ntu_viral_input_suite
    test_freeze_rtkslam_holdout_inputs
    test_frontend_determinism_script
    test_generate_ntu_viral_csv_reference
    test_graph_slam_offline_ablation
    test_mid360_robot_field_session
    test_mid360_robot_map_runner
    test_mid360_robot_plan_cli
    test_mid360_robot_profile_cli
    test_mid360_robot_public_bag_clip
    test_mid360_robot_public_bag_segments
    test_mid360_robot_public_dataset_map_runner
    test_mid360_robot_public_dataset_report
    test_mid360_robot_public_datasets
    test_mid360_robot_public_loop_bag
    test_mid360_robot_public_loop_candidates
    test_mid360_robot_public_map_run_diagnosis
    test_mid360_robot_readiness_cli
    test_mid360_robot_record_cli
    test_mid360_robot_runbook_smoke
    test_mid360_robot_sample_session
    test_mid360_robot_sample_session_matrix
    test_normalize_ntu_viral_rosbag
    test_ntu_viral_download_script
    test_offline_determinism_script
    test_ours_competitive_benchmark_runner
    test_pcd_sequence_to_rosbag2
    test_plane_patch_warp
    test_pose_graph_loop_residuals
    test_prepare_ntu_viral_holdout
    test_public_suite_gate
    test_radarless_tunnel_benchmark_tools
    test_realtime_colored_map_evaluator
    test_rko_lio_graph_benchmark_script
    test_rtk_slam_download_script
    test_run_lidar_slam
    test_run_ntu_viral_formal_measurements
    test_runtime_report
    test_select_rosbag_topic
    test_slice_rosbag2
  )
  foreach(PYTEST_NAME IN LISTS GRAPH_BASED_SLAM_ADDITIONAL_PYTESTS)
    ament_add_pytest_test(${PYTEST_NAME} test/${PYTEST_NAME}.py TIMEOUT 120)
  endforeach()
  ament_add_pytest_test(
    test_benchmark_reporting_tools
    test/test_benchmark_reporting_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rko_lio_benchmark_tools
    test/test_rko_lio_benchmark_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rko_lio_offline_completion_source
    test/test_rko_lio_offline_completion_source.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_docs_entrypoints
    test/test_docs_entrypoints.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_default_ci_checks_script
    test/test_default_ci_checks_script.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_product_python_tests_script
    test/test_product_python_tests_script.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_enwide_benchmark_assets
    test/test_enwide_benchmark_assets.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_score_position_only_trajectory
    test/test_score_position_only_trajectory.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_summarize_intensity_peak_diagnostics
    test/test_summarize_intensity_peak_diagnostics.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_validate_real_data_e2e
    test/test_validate_real_data_e2e.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_build_mid360_onboarding_fixture
    test/test_build_mid360_onboarding_fixture.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_lidarslam_product_cli
    test/test_lidarslam_product_cli.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_aligned_trajectory_metrics
    test/test_aligned_trajectory_metrics.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_v2_beta_readiness_report
    test/test_v2_beta_readiness_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_stress_validation_report
    test/test_stress_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_place_recognition_report
    test/test_place_recognition_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_packet_imu_deskew_validation_report
    test/test_packet_imu_deskew_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_dynamic_object_filter_report
    test/test_dynamic_object_filter_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_dynamic_object_filter_validation_report
    test/test_dynamic_object_filter_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_submission_bundle_tools
    test/test_submission_bundle_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_authoring_report
    test/test_map_authoring_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_classic_path_report
    test/test_classic_path_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_odom_prior_validation_report
    test/test_odom_prior_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_exploration_closeout_report
    test/test_exploration_closeout_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_triangle_descriptor_yaml_defaults
    test/test_triangle_descriptor_yaml_defaults.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_event_driven_loop_search_defaults
    test/test_event_driven_loop_search_defaults.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_triangle_ablation_runner
    test/test_triangle_ablation_runner.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_mid360_robot_3d_map_preview
    test/test_mid360_robot_3d_map_preview.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_bim_export
    test/test_bim_export.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_bim_regression_gate
    test/test_bim_regression_gate.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_plane_revisit_ab_gate
    test/test_plane_revisit_ab_gate.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_slam_candidate_regression
    test/test_slam_candidate_regression.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_plane_revisit_candidate_benchmark_script
    test/test_plane_revisit_candidate_benchmark_script.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_record_backend_input_script
    test/test_record_backend_input_script.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_hilti_overlap_crossval_script
    test/test_hilti_overlap_crossval_script.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_sparse_checkpoint_errors
    test/test_sparse_checkpoint_errors.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_las_export
    test/test_las_export.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_map_export_gis
    test/test_map_export_gis.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_mesh_export
    test/test_mesh_export.py
    TIMEOUT 300
  )
endif()

add_library(graph_based_slam_component SHARED
  src/graph_based_slam_component.cpp
  src/graph_slam_ros_adapter.cpp
  src/graph_slam_composition.cpp
  src/three_d_bbs_loop_verifier.cpp
)
target_compile_features(graph_based_slam_component PRIVATE cxx_std_17)

target_compile_definitions(graph_based_slam_component PRIVATE "GS_GBS_BUILDING_DLL")
if(GRAPH_BASED_SLAM_HAVE_3D_BBS)
  target_compile_definitions(graph_based_slam_component PRIVATE GRAPH_BASED_SLAM_HAVE_3D_BBS)
  target_link_libraries(graph_based_slam_component graph_based_slam_3d_bbs_vendor)
endif()

ament_target_dependencies(graph_based_slam_component
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  lidarslam_msgs
  message_filters
  ndt_omp_ros2
)

target_link_libraries(graph_based_slam_component
  graph_slam_config
  graph_slam_application
  graph_slam_io_adapters
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
  g2o::stuff
)

add_executable(graph_based_slam_node
  src/graph_based_slam_node.cpp
)

add_executable(map_quality_report
  src/map_quality_report_main.cpp
)

add_executable(planar_map_filter
  src/planar_map_filter_main.cpp
)

target_include_directories(map_quality_report PRIVATE include ${EIGEN3_INCLUDE_DIR})

target_compile_features(map_quality_report PRIVATE cxx_std_17)

target_link_libraries(map_quality_report
  ${PCL_LIBRARIES}
)

target_include_directories(planar_map_filter PRIVATE include ${EIGEN3_INCLUDE_DIR})

target_compile_features(planar_map_filter PRIVATE cxx_std_17)

target_link_libraries(planar_map_filter
  ${PCL_LIBRARIES}
)

target_link_libraries(graph_based_slam_node
  graph_based_slam_component
)

ament_target_dependencies(graph_based_slam_node
  rclcpp)

add_executable(graph_slam_offline_runner
  src/graph_slam_offline_runner.cpp
  src/three_d_bbs_loop_verifier.cpp
)
target_compile_features(graph_slam_offline_runner PRIVATE cxx_std_17)

if(GRAPH_BASED_SLAM_HAVE_3D_BBS)
  target_compile_definitions(graph_slam_offline_runner PRIVATE GRAPH_BASED_SLAM_HAVE_3D_BBS)
  target_link_libraries(graph_slam_offline_runner graph_based_slam_3d_bbs_vendor)
endif()

ament_target_dependencies(graph_slam_offline_runner
  rclcpp
  rosbag2_cpp
  tf2_eigen
  pcl_conversions
  nav_msgs
  sensor_msgs
  ndt_omp_ros2
)

target_link_libraries(graph_slam_offline_runner
  graph_slam_application
  graph_slam_io_adapters
  ${PCL_LIBRARIES}
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
  g2o::stuff
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_gtest(
    test_graph_slam_composition
    test/test_graph_slam_composition.cpp
  )
  if(TARGET test_graph_slam_composition)
    target_include_directories(
      test_graph_slam_composition PRIVATE src include ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(test_graph_slam_composition graph_based_slam_component)
    ament_target_dependencies(test_graph_slam_composition rclcpp)
  endif()
endif()

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_nodes(graph_based_slam_component
  "graphslam::GraphBasedSlamComponent")

ament_export_libraries(graph_based_slam_component)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  graph_based_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  graph_based_slam_node
  graph_slam_offline_runner
  map_quality_report
  planar_map_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
