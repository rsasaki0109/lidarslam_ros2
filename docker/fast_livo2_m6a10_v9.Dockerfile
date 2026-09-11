# Reproducible FAST-LIVO2 ROS 1 M6a10 v9 image.
#
# v9 is an additive source identity over the immutable v8 service-queue
# image: the base evidence patch, v8 queue delta, and v9 wall-time delta are
# verified and applied in that exact order.  This recipe is a build gate only;
# replay and scoring are host-side contracts.
FROM docker.io/library/ros:noetic-ros-base@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

ARG FAST_LIVO2_REVISION=0d2c0346107b75b59934975adec9a6eeeb913c64
ARG RPG_VIKIT_REVISION=6c886c8e5d83997806e00294826d528cea3581dd
ARG SOPHUS_REVISION=a621ff2e56c56c839a6c40418d42c3c254424b5c
ARG FAST_LIVO2_M6A10_PATCH_SHA256=33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297
ARG FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256=e4a3b2b9d981ec0f740365695f53a662797aaf9400e1d7186551687143c4fd87
ARG FAST_LIVO2_M6A10_V9_DELTA_PATCH_SHA256=1cf3067a634a34d5c3a2126f9f495fcb675068cb94edf562eb7e84b1e470146b
ARG FAST_LIVO2_M6A10_VARIANT=v9
ARG FAST_LIVO2_M6A10_OBSERVABILITY_CONTRACT=m6a10-fast-livo2-observable-supervision-v2-watchdog
ARG FAST_LIVO2_M6A10_CONSUMER_CONTRACT=m6a10-v2c-fast-livo2-wallrate-v9
ARG FAST_LIVO2_M6A10_SERVICE_HANDSHAKE_CONTRACT=m6a10-v2c-fast-livo2-synthetic-lidar-handshake-v1
ARG FAST_LIVO2_M6A10_WATCHDOG_CONTRACT=m6a10-fast-livo2-host-watchdog-v2
ARG FAST_LIVO2_CPU_FLAGS_POLICY=portable_x86_64_v1
ARG BENCHMARK_CPU_THREADS=8

ENV DEBIAN_FRONTEND=noninteractive \
    BENCHMARK_CPU_ONLY=1 \
    OMP_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    OPENBLAS_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    MKL_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    TBB_NUM_THREADS=${BENCHMARK_CPU_THREADS}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
 && apt-get install --no-install-recommends -y \
      build-essential ca-certificates cmake git ninja-build pkg-config \
      python3-catkin-tools python3-pip time \
      libboost-all-dev libeigen3-dev libgoogle-glog-dev libomp-dev \
      libsuitesparse-dev libopencv-dev libpcl-dev \
      ros-noetic-cmake-modules \
      ros-noetic-cv-bridge ros-noetic-eigen-conversions \
      ros-noetic-image-transport ros-noetic-message-generation \
      ros-noetic-nav-msgs ros-noetic-pcl-conversions ros-noetic-pcl-ros \
      ros-noetic-rosbag ros-noetic-roscpp ros-noetic-roslaunch \
      ros-noetic-rospy ros-noetic-sensor-msgs ros-noetic-std-msgs \
      ros-noetic-std-srvs ros-noetic-tf ros-noetic-visualization-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/fast_livo_ws/src \
 && git clone --filter=blob:none https://github.com/hku-mars/FAST-LIVO2.git \
      /opt/fast_livo_ws/src/FAST-LIVO2 \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 checkout "${FAST_LIVO2_REVISION}" \
 && git clone --filter=blob:none https://github.com/xuankuzcr/rpg_vikit.git \
      /opt/fast_livo_ws/src/rpg_vikit \
 && git -C /opt/fast_livo_ws/src/rpg_vikit checkout "${RPG_VIKIT_REVISION}"

# Keep the portable CPU policy from v8; no host-derived ISA flags are allowed.
RUN test "${FAST_LIVO2_CPU_FLAGS_POLICY}" = portable_x86_64_v1 \
 && for cmake_file in \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_common/CMakeLists.txt \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_ros/CMakeLists.txt; do \
      test -f "${cmake_file}" \
      && sed -i \
           -e 's/-march=native/-march=x86-64 -mtune=generic/g' \
           -e 's/-mtune=native/-mtune=generic/g' \
           -e 's/ -funroll-loops//g' -e 's/ -fsee//g' "${cmake_file}"; \
    done \
 && ! grep -R -n -E -- '-march=(native|tigerlake)|-mtune=native|-m(avx512|avx2)' \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_common/CMakeLists.txt \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_ros/CMakeLists.txt

# Apply the immutable base instrumentation first.
COPY docker/patches/fast_livo2.m6a10-v2c.patch /tmp/fast_livo2.m6a10-v2c.patch
RUN test "$(sha256sum /tmp/fast_livo2.m6a10-v2c.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_PATCH_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c.patch \
 && rm -f /tmp/fast_livo2.m6a10-v2c.patch

# Apply the immutable v8 service-queue delta second.
COPY docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch \
     /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch
RUN test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch \
 && grep -q 'm6a10_service_spinner' \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp \
 && rm -f /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch

# v9 changes only the idle loop clock source; all sensor/estimator code stays
# inherited from v8.  Verify and apply this final source delta independently.
COPY docker/patches/fast_livo2.m6a10-v2c-v9-wallrate.patch \
     /tmp/fast_livo2.m6a10-v2c-v9-wallrate.patch
RUN test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v9-wallrate.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V9_DELTA_PATCH_SHA256}" \
 && test "${FAST_LIVO2_M6A10_VARIANT}" = v9 \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v9-wallrate.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v9-wallrate.patch \
 && grep -q 'ros::WallRate rate(5000)' \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp \
 && rm -f /tmp/fast_livo2.m6a10-v2c-v9-wallrate.patch

# Build the pinned legacy Sophus release and FAST-LIVO2 workspace.
RUN git clone --filter=blob:none https://github.com/strasdat/Sophus.git /tmp/Sophus \
 && git -C /tmp/Sophus checkout "${SOPHUS_REVISION}" \
 && sed -i -e 's/unit_complex_\.real() = 1\.;/unit_complex_.real(1.);/' \
           -e 's/unit_complex_\.imag() = 0\.;/unit_complex_.imag(0.);/' \
      /tmp/Sophus/sophus/so2.cpp \
 && cmake -S /tmp/Sophus -B /tmp/Sophus/build -GNinja \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
 && cmake --build /tmp/Sophus/build --parallel 2 \
 && cmake --install /tmp/Sophus/build \
 && rm -rf /tmp/Sophus \
 && install -d /usr/local/lib/cmake/Sophus \
 && printf '%s\n' 'set(Sophus_FOUND TRUE)' \
      'set(Sophus_INCLUDE_DIRS /usr/local/include)' \
      'set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)' \
      > /usr/local/lib/cmake/Sophus/SophusConfig.cmake

RUN sed -i -e 's/-march=native -mtune=native -funroll-loops/-ffp-contract=off/g' \
           -e 's/-ffast-math/-ffp-contract=off/g' \
      /opt/fast_livo_ws/src/FAST-LIVO2/CMakeLists.txt \
 && source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

RUN printf '%s\n' '#!/usr/bin/env bash' 'set -euo pipefail' \
      'source /opt/ros/noetic/setup.bash' \
      'source /opt/fast_livo_ws/devel/setup.bash' 'exec "$@"' \
      > /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh

ARG FAST_LIVO2_M6A10_FEEDER_SHA256=1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831
ARG FAST_LIVO2_M6A10_RUNNER_SHA256=f7efa0e45eb5849df8ad7b62e5be86d072e14c76ee9edcf00b79cca724b17fd4
LABEL org.opencontainers.image.source="https://github.com/hku-mars/FAST-LIVO2" \
      benchmark.fast_livo2.revision="0d2c0346107b75b59934975adec9a6eeeb913c64" \
      benchmark.rpg_vikit.revision="6c886c8e5d83997806e00294826d528cea3581dd" \
      benchmark.sophus.revision="a621ff2e56c56c839a6c40418d42c3c254424b5c" \
      benchmark.base_image="docker.io/library/ros:noetic-ros-base" \
      benchmark.base_digest="sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d" \
      benchmark.fast_livo2.m6a10_patch_sha256="${FAST_LIVO2_M6A10_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_base_patch_sha256="${FAST_LIVO2_M6A10_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_M6A10_VARIANT}" \
      benchmark.fast_livo2.m6a10_v8_delta_patch_sha256="${FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_v9_delta_patch_sha256="${FAST_LIVO2_M6A10_V9_DELTA_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_feeder_sha256="${FAST_LIVO2_M6A10_FEEDER_SHA256}" \
      benchmark.fast_livo2.m6a10_runner_sha256="${FAST_LIVO2_M6A10_RUNNER_SHA256}" \
      benchmark.fast_livo2.m6a10_consumer_capability="callback_acceptance_eof_service_v1" \
      benchmark.fast_livo2.m6a10_consumer_contract="${FAST_LIVO2_M6A10_CONSUMER_CONTRACT}" \
      benchmark.fast_livo2.m6a10_service_handshake_contract="${FAST_LIVO2_M6A10_SERVICE_HANDSHAKE_CONTRACT}" \
      benchmark.fast_livo2.m6a10_queue_overflow_capability="single_inflight_exact_count_observed" \
      benchmark.fast_livo2.m6a10_watchdog_contract="${FAST_LIVO2_M6A10_WATCHDOG_CONTRACT}" \
      benchmark.fast_livo2.m6a10_observability_contract="${FAST_LIVO2_M6A10_OBSERVABILITY_CONTRACT}" \
      benchmark.fast_livo2.cpu_flags_policy="${FAST_LIVO2_CPU_FLAGS_POLICY}" \
      benchmark.cpu_policy="cpu_only;threads=8;native_flags=disabled"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
