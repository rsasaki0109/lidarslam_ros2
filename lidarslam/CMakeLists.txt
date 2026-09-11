cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

project(lidarslam)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(scanmatcher REQUIRED)
find_package(graph_based_slam REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(g2o QUIET
  COMPONENTS
    g2o::core
    g2o::types_slam3d
    g2o::types_slam3d_addons
    g2o::solver_eigen
)
if(NOT g2o_FOUND)
  find_path(G2O_INCLUDE_DIR g2o/core/sparse_optimizer.h PATHS /usr/include /usr/local/include)
  find_library(G2O_CORE_LIB g2o_core)
  find_library(G2O_TYPES_SLAM3D_LIB g2o_types_slam3d)
  find_library(G2O_TYPES_SLAM3D_ADDONS_LIB g2o_types_slam3d_addons)
  find_library(G2O_SOLVER_EIGEN_LIB g2o_solver_eigen)
  find_library(G2O_STUFF_LIB g2o_stuff)
  if(G2O_INCLUDE_DIR AND G2O_CORE_LIB AND G2O_TYPES_SLAM3D_LIB AND G2O_SOLVER_EIGEN_LIB)
    message(STATUS "Found g2o via fallback: ${G2O_CORE_LIB}")
    foreach(_tgt core types_slam3d types_slam3d_addons solver_eigen stuff)
      string(TOUPPER "${_tgt}" _TGT_UPPER)
      if(NOT TARGET g2o::${_tgt} AND G2O_${_TGT_UPPER}_LIB)
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

set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_rko_lio_launch_config
    test/test_rko_lio_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_bundled_dependency_version_alignment
    test/test_bundled_dependency_version_alignment.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_lidarslam_launch_config
    test/test_lidarslam_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_lo_slam_launch_config
    test/test_lo_slam_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_small_gicp_lo_slam_launch_config
    test/test_small_gicp_lo_slam_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_gnss_smoke_script
    test/test_open_data_gnss_smoke_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_applanix_velodyne_gnss_smoke_script
    test/test_open_data_applanix_velodyne_gnss_smoke_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_applanix_velodyne_gnss_benchmark_script
    test/test_open_data_applanix_velodyne_gnss_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_prepare_rosbag2_playback
    test/test_prepare_rosbag2_playback.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_packet_imu_deskew_validation_matrix_script
    test/test_open_data_packet_imu_deskew_validation_matrix_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_dynamic_object_filter_benchmark_script
    test/test_dynamic_object_filter_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_classic_path_benchmark_suite_script
    test/test_open_data_classic_path_benchmark_suite_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rtk_slam_accuracy_suite
    test/test_rtk_slam_accuracy_suite.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_place_recognition_benchmark_script
    test/test_place_recognition_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rko_lio_mid360_crossval_benchmark_script
    test/test_rko_lio_mid360_crossval_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_installed_product_cli_contract
    test/test_installed_product_cli_contract.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_collect_runtime_apt_packages
    test/test_collect_runtime_apt_packages.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_runtime_docker_image_contract
    test/test_runtime_docker_image_contract.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_product_recovery_tools
    test/test_product_recovery_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_release_image_promotion
    test/test_release_image_promotion.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_soak_runner
    test/test_map_soak_runner.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_sparse_trajectory_scoring
    test/test_sparse_trajectory_scoring.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_external_first_map_readiness
    test/test_external_first_map_readiness.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_first_map_validation_receipt
    test/test_first_map_validation_receipt.py
    TIMEOUT 120
  )
  set(LIDARSLAM_ADDITIONAL_PYTESTS
    test_aggregate_kitti_metrics
    test_audit_candidate_image_set
    test_audit_published_fixture
    test_autoware_map_runner_script
    test_benchmark_provenance
    test_benchmark_summary_profiles
    test_build_docker_launcher_asset
    test_cli_completion
    test_cli_interactive_home
    test_cli_option_contract
    test_canonical_ndt_convergence
    test_candidate_environment_readiness
    test_candidate_image_gate
    test_check_publication_slice_plan
    test_check_g0_readiness
    test_product_draft_review_routing
    test_product_draft_review_ledger
    test_contributor_starter_queue
    test_check_fixture_publication
    test_check_issue_triage_proposal
    test_check_onboarding_trial
    test_check_onboarding_trial_matrix
    test_check_usability_scorecard
    test_collect_growth_snapshot
    test_complete_onboarding_measurements
    test_docker_map_bag
    test_docs_deployment_provenance
    test_evaluate_degeneracy_trajectory
    test_external_first_map_acceptance
    test_first_map_demo
    test_first_map_validator_cohort
    test_first_map_verification_package
    test_glim_reference_cache
    test_github_api_auth
    test_lidarslam_guided
    test_lidarslam_doctor
    test_install_source_dependencies
    test_source_quickstart
    test_install_upgrade_contract
    test_kitti_metrics
    test_measure_oci_archive
    test_ndt_omp_release_readiness
    test_prepare_candidate_trial
    test_prepare_issue_triage_application
    test_prepare_onboarding_matrix_packet
    test_prepare_usability_scorecard
    test_prepare_usability_scorecard_pair
    test_record_usability_scorecard_pair
    test_package_manager_install
    test_package_manager_release_readiness
    test_product_build_info
    test_published_onboarding_identity
    test_published_release
    test_release_channel
    test_rko_lio_graph_benchmark_script
    test_ros_apt_dependency_readiness
    test_run_candidate_trial
    test_run_docker_onboarding_probe
    test_run_source_onboarding_probe
    test_start_candidate_trial
    test_sensor_setup_wizard
    test_session_compare
    test_session_history
    test_support_bundle
    test_simple_lanelet2_generator
    test_social_demo_media
    test_v1_readiness
    test_view_autoware_map
  )
  foreach(PYTEST_NAME IN LISTS LIDARSLAM_ADDITIONAL_PYTESTS)
    ament_add_pytest_test(${PYTEST_NAME} test/${PYTEST_NAME}.py TIMEOUT 120)
  endforeach()
endif()

add_executable(lidarslam
  src/lidarslam.cpp
)

target_link_libraries(lidarslam
  ${PCL_LIBRARIES}
  g2o::core
  g2o::types_slam3d
  g2o::types_slam3d_addons
  g2o::solver_eigen
)

ament_target_dependencies(lidarslam
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  nav_msgs
  pcl_conversions
  std_srvs
  scanmatcher
  graph_based_slam
  lidarslam_msgs
  ndt_omp_ros2
)

install(
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
)

install(TARGETS
  lidarslam
  DESTINATION lib/${PROJECT_NAME}
)

file(
  STRINGS
  ${CMAKE_CURRENT_SOURCE_DIR}/product-runtime-files.txt
  PRODUCT_CLI_SCRIPT_NAMES
)
list(FILTER PRODUCT_CLI_SCRIPT_NAMES EXCLUDE REGEX "^[ \t]*(#|$)")
set(PRODUCT_CLI_SCRIPTS)
foreach(PRODUCT_CLI_SCRIPT_NAME IN LISTS PRODUCT_CLI_SCRIPT_NAMES)
  list(
    APPEND PRODUCT_CLI_SCRIPTS
    ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/${PRODUCT_CLI_SCRIPT_NAME}
  )
endforeach()

set(
  LIDARSLAM_SOURCE_REVISION
  ""
  CACHE STRING
  "Exact 40-character source revision for builds without Git metadata"
)
set(
  LIDARSLAM_SOURCE_DIRTY
  ""
  CACHE STRING
  "Source dirty state for an explicit revision: true or false"
)
set(PRODUCT_BUILD_INFO_ARGS
  --source-dir ${CMAKE_CURRENT_SOURCE_DIR}/..
  --output ${CMAKE_CURRENT_BINARY_DIR}/product-build-info.json
)
if(NOT "${LIDARSLAM_SOURCE_REVISION}" STREQUAL "")
  list(
    APPEND PRODUCT_BUILD_INFO_ARGS
    --revision ${LIDARSLAM_SOURCE_REVISION}
  )
endif()
if(NOT "${LIDARSLAM_SOURCE_DIRTY}" STREQUAL "")
  list(
    APPEND PRODUCT_BUILD_INFO_ARGS
    --dirty ${LIDARSLAM_SOURCE_DIRTY}
  )
endif()
add_custom_target(
  lidarslam_product_build_info ALL
  COMMAND
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/generate_product_build_info.py
    ${PRODUCT_BUILD_INFO_ARGS}
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/product-build-info.json
  COMMENT "Generating installed product source provenance"
  VERBATIM
)

# Keep the historical ROS executable untouched at lib/lidarslam/lidarslam.
# The PATH command and ros2-run shim deliberately use distinct names.
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/lidarslam
  DESTINATION bin
  RENAME lidarslam-map
)
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/lidarslam
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidarslam-cli
)
install(
  PROGRAMS ${PRODUCT_CLI_SCRIPTS}
  DESTINATION share/${PROJECT_NAME}/product/scripts
)
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../VERSION
    ${CMAKE_CURRENT_SOURCE_DIR}/product-runtime-files.txt
    ${CMAKE_CURRENT_BINARY_DIR}/product-build-info.json
  DESTINATION share/${PROJECT_NAME}/product
)
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/run-manifest-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/run-manifest-v2.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/release-image-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/rollback-plan-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/first-map-validation-receipt-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/first-map-handoff-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/first-map-demo-plan-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/sensor-setup-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/sensor-setup-rejection-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-session-recovery-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-session-index-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-session-catalog-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-session-comparison-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/support-bundle-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-edit-plan-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-edit-receipt-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-project-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/map-merge-receipt-v1.schema.json
    ${CMAKE_CURRENT_SOURCE_DIR}/../docs/schemas/system-doctor-v1.schema.json
  DESTINATION share/${PROJECT_NAME}/product/schemas
)
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/completions/lidarslam-map.bash
  DESTINATION share/${PROJECT_NAME}/product/completions
)

ament_package()
