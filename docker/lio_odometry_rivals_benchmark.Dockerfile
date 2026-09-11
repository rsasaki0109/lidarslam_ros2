# Reproducible ROS 1 build environments for the two odometry references in
# competitive_slam_sota_v2. The base is content-addressed; each official rival
# source and the compile-only Livox message dependency is pinned below.
FROM fast-livo2-benchmark@sha256:2c1ff787788d599889df00fd36850ab9764f6734720e7363fb8d1e5a326bd168 AS common

ARG LIVOX_ROS_DRIVER_REVISION=3d240d5666129e1a3052e78ee8487a04b08fdda3
ARG LIVOX_SDK_REVISION=9306596a2bf15c1343bc023b497465ed0a32909d

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      git python3-catkin-tools ros-noetic-eigen-conversions \
      ros-noetic-message-generation ros-noetic-pcl-ros \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/lio_ws/src \
 && git clone --filter=blob:none \
      https://github.com/Livox-SDK/livox_ros_driver.git \
      /opt/lio_ws/src/livox_ros_driver \
 && git -C /opt/lio_ws/src/livox_ros_driver checkout \
      "${LIVOX_ROS_DRIVER_REVISION}" \
 && git clone --filter=blob:none \
      https://github.com/Livox-SDK/Livox-SDK.git \
      /opt/lio_ws/src/livox_ros_driver/livox_ros_driver/Livox-SDK \
 && git -C /opt/lio_ws/src/livox_ros_driver/livox_ros_driver/Livox-SDK \
      checkout "${LIVOX_SDK_REVISION}" \
 && cmake -S /opt/lio_ws/src/livox_ros_driver/livox_ros_driver/Livox-SDK \
      -B /opt/lio_ws/src/livox_ros_driver/livox_ros_driver/Livox-SDK/build \
      -DCMAKE_BUILD_TYPE=Release \
 && cmake --build \
      /opt/lio_ws/src/livox_ros_driver/livox_ros_driver/Livox-SDK/build -j1 \
 && cmake --install \
      /opt/lio_ws/src/livox_ros_driver/livox_ros_driver/Livox-SDK/build

RUN printf '%s\n' '#!/usr/bin/env bash' 'set -e' \
    'source /opt/ros/noetic/setup.bash' \
    'source /opt/lio_ws/devel/setup.bash' 'exec "$@"' \
    > /sota_entrypoint.sh \
 && chmod +x /sota_entrypoint.sh


FROM common AS fast_lio2

ARG RIVAL_REVISION=7cc4175de6f8ba2edf34bab02a42195b141027e9

RUN git clone --filter=blob:none --no-checkout \
      https://github.com/hku-mars/FAST_LIO.git \
      /opt/lio_ws/src/FAST_LIO \
 && git -C /opt/lio_ws/src/FAST_LIO sparse-checkout init --cone \
 && git -C /opt/lio_ws/src/FAST_LIO sparse-checkout set \
      src include config launch msg \
 && git -C /opt/lio_ws/src/FAST_LIO checkout "${RIVAL_REVISION}" \
 && git -C /opt/lio_ws/src/FAST_LIO submodule update --init --depth 1 \
      include/ikd-Tree \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws fast_lio_generate_messages_cpp -j1 \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1

LABEL benchmark.rival="fast_lio2" \
      benchmark.rival.revision="7cc4175de6f8ba2edf34bab02a42195b141027e9" \
      benchmark.ikd_tree.revision="e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4" \
      benchmark.livox_ros_driver.revision="3d240d5666129e1a3052e78ee8487a04b08fdda3" \
      benchmark.livox_sdk.revision="9306596a2bf15c1343bc023b497465ed0a32909d"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM common AS voxel_slam

ARG GTSAM_REVISION=6425000775da29c93719f37dce3f2de38a0064ec
ARG RIVAL_REVISION=70fc8a28d63823d5989ff184daeea0787b672398

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      libboost-all-dev ninja-build \
 && rm -rf /var/lib/apt/lists/* \
 && git clone --filter=blob:none https://github.com/borglab/gtsam.git /src/gtsam \
 && git -C /src/gtsam checkout "${GTSAM_REVISION}" \
 && cmake -S /src/gtsam -B /src/gtsam/build -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_WRAP=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_WITH_TBB=OFF \
 && cmake --build /src/gtsam/build --parallel 2 \
 && cmake --install /src/gtsam/build \
 && ldconfig

RUN git clone --filter=blob:none --no-checkout \
      https://github.com/hku-mars/Voxel-SLAM.git \
      /opt/lio_ws/src/Voxel-SLAM \
 && git -C /opt/lio_ws/src/Voxel-SLAM sparse-checkout init --cone \
 && git -C /opt/lio_ws/src/Voxel-SLAM sparse-checkout set VoxelSLAM \
 && git -C /opt/lio_ws/src/Voxel-SLAM checkout "${RIVAL_REVISION}" \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1

LABEL benchmark.rival="voxel_slam" \
      benchmark.rival.revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.gtsam.revision="6425000775da29c93719f37dce3f2de38a0064ec" \
      benchmark.livox_ros_driver.revision="3d240d5666129e1a3052e78ee8487a04b08fdda3" \
      benchmark.livox_sdk.revision="9306596a2bf15c1343bc023b497465ed0a32909d"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam AS voxel_slam_v17

COPY docker/patches/voxel_slam_v17/weak_axis_bounded_map.patch \
     /tmp/weak_axis_bounded_map.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/weak_axis_bounded_map.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/weak_axis_bounded_map.patch

LABEL benchmark.candidate="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="6f8a5121f2a76c7d8abc75325abebe7c55ab0a0b81ae1b2e62d58d746a010c11" \
      benchmark.candidate.voxel_map_hpp.sha256="e3416197e6c04a0bdbeddc8ec0e2102c09e764f9bd5ca9574222d074c5fbde3f"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v18

COPY docker/patches/voxel_slam_v18/causal_deceleration_bridge.patch \
     /tmp/causal_deceleration_bridge.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/causal_deceleration_bridge.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/causal_deceleration_bridge.patch

LABEL benchmark.candidate="voxel_slam_causal_deceleration_bridge_v18" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="e15a15c40499ea24601e371da5c36de3b6afad83b88772c3a5253bc076b6eadd" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.v18_patch.sha256="e15a15c40499ea24601e371da5c36de3b6afad83b88772c3a5253bc076b6eadd" \
      benchmark.candidate.voxelslam_cpp.sha256="bc4ef859828457267a48719e47b3f5f3e514ef41ab048a66ddf5061f2d5b1b6b" \
      benchmark.candidate.voxel_map_hpp.sha256="e3416197e6c04a0bdbeddc8ec0e2102c09e764f9bd5ca9574222d074c5fbde3f"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v19

COPY docker/patches/voxel_slam_v19/preentry_bias_calibrated_bridge.patch \
     /tmp/preentry_bias_calibrated_bridge.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply --recount \
      /tmp/preentry_bias_calibrated_bridge.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/preentry_bias_calibrated_bridge.patch

LABEL benchmark.candidate="voxel_slam_preentry_bias_calibrated_bridge_v19" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="c2b155522a2d263745d448cc56d05e0c49d60169dbc53b5d15076a4edd09ee26" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.v19_patch.sha256="c2b155522a2d263745d448cc56d05e0c49d60169dbc53b5d15076a4edd09ee26" \
      benchmark.candidate.voxelslam_cpp.sha256="7af27e8c5d7ffbbb740fe9be9bcdebe4c00645a2ada87c87aae99134a62a42b4" \
      benchmark.candidate.voxel_map_hpp.sha256="e3416197e6c04a0bdbeddc8ec0e2102c09e764f9bd5ca9574222d074c5fbde3f"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v33

COPY docker/patches/voxel_slam_dev/v33.patch \
     /tmp/v33.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v33.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v33.patch

LABEL benchmark.candidate="voxel_slam_vertical_velocity_bias_consistency_v33" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="def93f40643c4ace23ec91665edc9d46fc720160274307657685f052ca1e05a7" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="163398cc62d163b37028c1d254e49e918d58c326901741d2d6d0c5452eb7be67"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v34

COPY docker/patches/voxel_slam_dev/v34.patch \
     /tmp/v34.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v34.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v34.patch

LABEL benchmark.candidate="voxel_slam_vertical_accel_bias_state_update_v34" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="591149278e87e1c8d2a7bbe778086c3e7fdf07e53680f3bd77b06f502a54d7e6" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="55c940b33292c002acc847c050ce43a6a97d63d03275eef21358f5cd97fb2e4d"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v35

COPY docker/patches/voxel_slam_dev/v35.patch \
     /tmp/v35.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v35.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v35.patch

LABEL benchmark.candidate="voxel_slam_observability_gated_vertical_accel_bias_v35" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="cd2de53047d3deded15ab4ecca01b75b7d019eb656c73a0eaea16eb1b931ad6f" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="039a21be9b75f24b65ecbbff7c8822d0ae26fe9d19ada26b09da40050abea7a7"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v36

COPY docker/patches/voxel_slam_dev/v36.patch \
     /tmp/v36.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v36.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v36.patch

LABEL benchmark.candidate="voxel_slam_observability_diagnostic_v36" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="be5c7e0550fa7ea727f27d3b424dda9c0c1c434a3c6e2d23560c4fa827d6b2e4" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v37

COPY docker/patches/voxel_slam_dev/v37.patch \
     /tmp/v37.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v37.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v37.patch

LABEL benchmark.candidate="voxel_slam_quarantined_vertical_accel_bias_v37" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="31a9ab38bbac99240a2a70df67b6b0cf75541555d28f09092dd0ea44301d7db1" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v38

COPY docker/patches/voxel_slam_dev/v38_visual_longitudinal_shadow.patch \
     /tmp/v38_visual_longitudinal_shadow.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v38_visual_longitudinal_shadow.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v38_visual_longitudinal_shadow.patch

LABEL benchmark.candidate="voxel_slam_visual_longitudinal_shadow_v38" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="fe9c313a9cacaffce333f9518f4c0c26cf955741dc9c44961ade9995b6a6e3b8" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="60ebeb7a63a8b1785b85a36b71d106520199af58f8e89a07f94a9ecf3fbe9e66"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v40

COPY docker/patches/voxel_slam_dev/v40.patch \
     /tmp/v40.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v40.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v40.patch

LABEL benchmark.candidate="voxel_slam_full_scan_gba_graph_v40" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="890c351d05938921c427c577ce89f348c5ee39f393bf6a9e890724c1a6b3b4e6" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="185be253dcf42a2d1917739fdc643b20b02a84358cdc4c31cef3171a3a48f86f"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM voxel_slam_v17 AS voxel_slam_v41

COPY docker/patches/voxel_slam_dev/v41.patch \
     /tmp/v41.patch

RUN git -C /opt/lio_ws/src/Voxel-SLAM apply \
      /tmp/v41.patch \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1 \
 && rm /tmp/v41.patch

LABEL benchmark.candidate="voxel_slam_cancellable_resource_guard_v41" \
      benchmark.candidate.base="voxel_slam_reclaiming_window_weak_axis_bridge_v17" \
      benchmark.candidate.parent_concept="voxel_slam_full_scan_gba_graph_v40" \
      benchmark.candidate.base_revision="70fc8a28d63823d5989ff184daeea0787b672398" \
      benchmark.candidate.patch.sha256="69dcfede4d80fcabc4bc04d8846d4297f116798eb68ed8f2fc9d098a0d776c77" \
      benchmark.candidate.v17_patch.sha256="62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25" \
      benchmark.candidate.voxelslam_cpp.sha256="e6516064abe6a16876af2b9b3e7cfb61519d992562b96f64bec484a5f963d66e"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]


FROM common AS point_lio

ARG RIVAL_REVISION=4b86a469eb5572e70ed575af25b5f15dd06e8e3c

RUN git clone --filter=blob:none \
      https://github.com/hku-mars/Point-LIO.git \
      /opt/lio_ws/src/Point-LIO \
 && git -C /opt/lio_ws/src/Point-LIO checkout "${RIVAL_REVISION}" \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -C /opt/lio_ws -DCMAKE_BUILD_TYPE=Release -j1

LABEL benchmark.rival="point_lio" \
      benchmark.rival.revision="4b86a469eb5572e70ed575af25b5f15dd06e8e3c" \
      benchmark.livox_ros_driver.revision="3d240d5666129e1a3052e78ee8487a04b08fdda3" \
      benchmark.livox_sdk.revision="9306596a2bf15c1343bc023b497465ed0a32909d"

ENTRYPOINT ["/sota_entrypoint.sh"]
CMD ["bash"]
