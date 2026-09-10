"""Launch RKO-LIO offline_node with graph_based_slam loop closure."""

# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _parse_bool_arg(value):
    """Parse a launch boolean override."""
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _add_optional_param_override(overrides, context, arg_name, parser):
    """Append an override only when the launch argument is explicitly set."""
    value = LaunchConfiguration(arg_name).perform(context)
    if value == '':
        return
    overrides[arg_name] = parser(value)


def create_rko_offline_node(context, *args, **kwargs):
    """Create the RKO-LIO offline node with optional parameter injection."""
    del args
    del kwargs

    parameters = []
    parameters.append({
        'bag_path': LaunchConfiguration('bag_path'),
        'lidar_topic': LaunchConfiguration('lidar_topic'),
        'imu_topic': LaunchConfiguration('imu_topic'),
        'base_frame': LaunchConfiguration('base_frame'),
        'odom_frame': LaunchConfiguration('odom_frame'),
        'lidar_frame': LaunchConfiguration('lidar_frame'),
        'imu_frame': LaunchConfiguration('imu_frame'),
        'deskew': LaunchConfiguration('deskew'),
        'voxel_size': LaunchConfiguration('voxel_size'),
        'max_range': LaunchConfiguration('max_range'),
        'min_range': LaunchConfiguration('min_range'),
        'initialization_phase': LaunchConfiguration('initialization_phase'),
        'skip_to_time': LaunchConfiguration('skip_to_time'),
        'publish_deskewed_scan': True,
        'dump_results': LaunchConfiguration('dump_results'),
        'results_dir': LaunchConfiguration('results_dir'),
        'run_name': LaunchConfiguration('run_name'),
    })

    # Let the optional YAML file override launch defaults such as
    # initialization_phase and sensor extrinsics.
    rko_param_file = LaunchConfiguration('rko_param_file').perform(context)
    if rko_param_file:
        parameters.append(rko_param_file)

    # The v2 benchmark contract is an additive process-level opt-in.  Keep
    # these values as the last parameter source so a launch default cannot
    # erase the wrapper's explicit evidence path/mode.  With no v2
    # environment the dictionary is empty and ordinary launches remain
    # parameter-compatible.
    benchmark_environment = {
        'm6a10_phase_contract_version': 'M6A10_PHASE_CONTRACT_VERSION',
        'm6a10_phase_mode': 'M6A10_PHASE_MODE',
        'm6a10_consumer_evidence_path': 'M6A10_CONSUMER_EVIDENCE',
        'm6a10_drain_timeout_seconds': 'M6A10_DRAIN_TIMEOUT_SECONDS',
        'm6a10_drain_diagnostic_path': 'M6A10_DRAIN_DIAGNOSTIC',
    }
    benchmark_parameters = {
        parameter: (
            float(os.environ[environment])
            if parameter == 'm6a10_drain_timeout_seconds'
            else os.environ[environment]
        )
        for parameter, environment in benchmark_environment.items()
        if os.environ.get(environment)
    }
    if benchmark_parameters:
        parameters.append(benchmark_parameters)

    return [
        Node(
            package='rko_lio',
            executable='offline_node',
            name='rko_lio_offline_node',
            parameters=parameters,
            output='screen',
            emulate_tty=True,
        ),
    ]


def create_graph_based_slam_node(context, *args, **kwargs):
    """Create graph_based_slam without overriding custom YAML defaults."""
    del args
    del kwargs

    parameters = [
        LaunchConfiguration('main_param_dir'),
        {
            'global_frame_id': LaunchConfiguration('global_frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_odom_input': True,
            'gnss_topic': LaunchConfiguration('gnss_topic'),
            'map_save_dir': LaunchConfiguration('save_dir'),
            'save_pose_graph_path': PathJoinSubstitution([
                LaunchConfiguration('save_dir'),
                'pose_graph.g2o',
            ]),
            'save_map_path': PathJoinSubstitution([
                LaunchConfiguration('save_dir'),
                'map.pcd',
            ]),
        },
    ]

    overrides = {}
    _add_optional_param_override(
        overrides, context, 'adjacent_edge_info_weight', float,
    )
    _add_optional_param_override(
        overrides, context, 'use_scan_context', _parse_bool_arg,
    )
    _add_optional_param_override(
        overrides, context, 'use_pcd_cache', _parse_bool_arg,
    )
    _add_optional_param_override(
        overrides, context, 'threshold_loop_closure_score', float,
    )
    _add_optional_param_override(
        overrides, context, 'distance_loop_closure', float,
    )
    # M6a10's GT-blind trajectory/consumer contract is benchmark-only.  A
    # ``--skip-map-save`` runner must also suppress loop-triggered map writes:
    # the graph component otherwise calls ``doPoseAdjustment(..., true)`` for
    # accepted loop edges, independently of the /map_save service.  Keep the
    # ordinary launch/YAML defaults untouched unless the wrapper explicitly
    # exports this opt-in environment marker.
    if os.environ.get('M6A10_BENCHMARK_NO_MAP_ARTIFACTS') == '1':
        overrides['use_save_map_in_loop'] = False
        # An empty path makes optimizePoseGraph retain no save target when
        # do_save_map is false, so the benchmark cannot leak pose_graph.g2o.
        overrides['save_pose_graph_path'] = ''
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='graph_based_slam',
            executable='graph_based_slam_node',
            parameters=parameters,
            remappings=[
                ('odom_input', '/rko_lio/odometry'),
                ('cloud_input', '/rko_lio/frame'),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    """Create the combined RKO-LIO + graph_based_slam launch description."""
    pkg_share = get_package_share_directory('lidarslam')
    main_param_dir_default = os.path.join(pkg_share, 'param', 'lidarslam.yaml')
    rviz_config_default = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir_default,
            description=(
                'Full path to the lidarslam parameter YAML '
                '(graph_based_slam section).'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (/clock).',
        ),
        DeclareLaunchArgument(
            'rko_param_file',
            default_value='',
            description=(
                'Full path to an optional RKO-LIO parameter YAML '
                '(extrinsics, initialization, etc.).'
            ),
        ),
        DeclareLaunchArgument(
            'global_frame_id',
            default_value='map',
            description='Global frame id.',
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='.',
            description='Directory for graph_based_slam outputs (pose_graph.g2o / map.pcd).',
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Path to the rosbag to play (required for offline_node).',
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/os_cloud_node/points',
            description='LiDAR point-cloud topic.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/os_cloud_node/imu',
            description='IMU topic.',
        ),
        DeclareLaunchArgument(
            'gnss_topic',
            default_value='/gnss/fix',
            description='NavSatFix topic for graph_based_slam when use_gnss:=true.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame (odometry is expressed in this frame).',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry parent frame.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='',
            description='LiDAR frame id (empty = read from message header).',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='',
            description='IMU frame id (empty = read from message header).',
        ),
        DeclareLaunchArgument(
            'deskew',
            default_value='true',
            description='Enable point-cloud deskewing.',
        ),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='1.0',
            description='Local-map voxel size (m).',
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='100.0',
            description='Max valid LiDAR range (m).',
        ),
        DeclareLaunchArgument(
            'min_range',
            default_value='1.0',
            description='Min valid LiDAR range (m).',
        ),
        DeclareLaunchArgument(
            'initialization_phase',
            default_value='false',
            description='Use IMU data between first two frames to initialise bias/orientation.',
        ),
        DeclareLaunchArgument(
            'skip_to_time',
            default_value='0.0',
            description='Skip to this timestamp in the bag (seconds).',
        ),
        DeclareLaunchArgument(
            'dump_results',
            default_value='false',
            description='Dump RKO-LIO trajectory to disk on exit.',
        ),
        DeclareLaunchArgument(
            'results_dir',
            default_value='results',
            description='Output directory for RKO-LIO results.',
        ),
        DeclareLaunchArgument(
            'run_name',
            default_value='rko_lio_run',
            description='Run name tag for RKO-LIO results.',
        ),
        DeclareLaunchArgument(
            'adjacent_edge_info_weight',
            default_value='',
            description='Optional override for adjacent_edge_info_weight.',
        ),
        DeclareLaunchArgument(
            'use_scan_context',
            default_value='',
            description='Optional override for use_scan_context.',
        ),
        DeclareLaunchArgument(
            'use_pcd_cache',
            default_value='',
            description='Optional override for use_pcd_cache.',
        ),
        DeclareLaunchArgument(
            'threshold_loop_closure_score',
            default_value='',
            description='Optional override for threshold_loop_closure_score.',
        ),
        DeclareLaunchArgument(
            'distance_loop_closure',
            default_value='',
            description='Optional override for distance_loop_closure.',
        ),
        DeclareLaunchArgument(
            'publish_static_tf',
            default_value='true',
            description='Publish a static TF between two configurable frames.',
        ),
        DeclareLaunchArgument(
            'static_tf_parent',
            default_value='os_sensor',
            description='Parent frame for the static TF.',
        ),
        DeclareLaunchArgument(
            'static_tf_child',
            default_value='os_imu',
            description='Child frame for the static TF.',
        ),
        DeclareLaunchArgument('static_tf_x', default_value='0.006253'),
        DeclareLaunchArgument('static_tf_y', default_value='-0.011775'),
        DeclareLaunchArgument('static_tf_z', default_value='0.007645'),
        DeclareLaunchArgument('static_tf_qx', default_value='0'),
        DeclareLaunchArgument('static_tf_qy', default_value='0'),
        DeclareLaunchArgument('static_tf_qz', default_value='0'),
        DeclareLaunchArgument('static_tf_qw', default_value='1'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='Full path to the RViz config file.',
        ),
        OpaqueFunction(function=create_rko_offline_node),
        OpaqueFunction(function=create_graph_based_slam_node),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                LaunchConfiguration('static_tf_x'),
                LaunchConfiguration('static_tf_y'),
                LaunchConfiguration('static_tf_z'),
                LaunchConfiguration('static_tf_qx'),
                LaunchConfiguration('static_tf_qy'),
                LaunchConfiguration('static_tf_qz'),
                LaunchConfiguration('static_tf_qw'),
                LaunchConfiguration('static_tf_parent'),
                LaunchConfiguration('static_tf_child'),
            ],
            condition=IfCondition(LaunchConfiguration('publish_static_tf')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ])
