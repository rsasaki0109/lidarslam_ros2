# Reproducible FAST-LIVO2 ROS 1 benchmark image.
#
# The historical benchmark image was layered on an untracked local
# ``hdl_localization_noetic:local`` image.  This recipe owns the complete
# dependency closure needed by the pinned FAST-LIVO2 checkout instead.
FROM docker.io/library/ros:noetic-ros-base@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

ARG FAST_LIVO2_REVISION=0d2c0346107b75b59934975adec9a6eeeb913c64
ARG RPG_VIKIT_REVISION=6c886c8e5d83997806e00294826d528cea3581dd
# FAST-LIVO2's upstream README and the historical image use this Sophus pin.
# It is intentionally kept as a named build argument until the upstream
# project publishes a durable full-length ref for this legacy release.
ARG SOPHUS_REVISION=a621ff2e56c56c839a6c40418d42c3c254424b5c
ARG FAST_LIVO2_M6A10_PATCH_SHA256=33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297
ARG FAST_LIVO2_M6A10_VARIANT=v8
ARG FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256=e4a3b2b9d981ec0f740365695f53a662797aaf9400e1d7186551687143c4fd87
ARG FAST_LIVO2_M6A10_OBSERVABILITY_CONTRACT=m6a10-fast-livo2-observable-supervision-v2-watchdog
ARG FAST_LIVO2_M6A10_CONSUMER_CONTRACT=m6a10-v2c-fast-livo2-service-queue-v8
ARG FAST_LIVO2_M6A10_SERVICE_HANDSHAKE_CONTRACT=m6a10-v2c-fast-livo2-service-queue-handshake-v1
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
      ros-noetic-std-srvs \
      ros-noetic-tf ros-noetic-visualization-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/fast_livo_ws/src \
 && git clone --filter=blob:none https://github.com/hku-mars/FAST-LIVO2.git \
      /opt/fast_livo_ws/src/FAST-LIVO2 \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 checkout "${FAST_LIVO2_REVISION}" \
 && git clone --filter=blob:none https://github.com/xuankuzcr/rpg_vikit.git \
      /opt/fast_livo_ws/src/rpg_vikit \
 && git -C /opt/fast_livo_ws/src/rpg_vikit checkout "${RPG_VIKIT_REVISION}"

# rpg_vikit writes ``-march=native`` directly into both of its CMake files.
# That would silently encode the builder's host ISA (and has previously
# produced tigerlake instructions despite the portable image label).  Keep
# this replacement explicit, deterministic, and scoped to the pinned source;
# no host-derived flags are accepted in the resulting compile commands.
RUN test "${FAST_LIVO2_CPU_FLAGS_POLICY}" = portable_x86_64_v1 \
 && for cmake_file in \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_common/CMakeLists.txt \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_ros/CMakeLists.txt; do \
      test -f "${cmake_file}" \
      && sed -i \
           -e 's/-march=native/-march=x86-64 -mtune=generic/g' \
           -e 's/-mtune=native/-mtune=generic/g' \
           -e 's/ -funroll-loops//g' \
           -e 's/ -fsee//g' \
           "${cmake_file}"; \
    done \
 && ! grep -R -n -E -- '-march=(native|tigerlake)|-mtune=native|-m(avx512|avx2)' \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_common/CMakeLists.txt \
      /opt/fast_livo_ws/src/rpg_vikit/vikit_ros/CMakeLists.txt

# The M6a10 path is benchmark-only.  Keep the upstream source revision
# immutable and apply the repo-owned patch only after verifying its bytes.
COPY docker/patches/fast_livo2.m6a10-v2c.patch /tmp/fast_livo2.m6a10-v2c.patch
RUN test "$(sha256sum /tmp/fast_livo2.m6a10-v2c.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_PATCH_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c.patch \
 && test -f /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_consumer_evidence.h \
 && rm -f /tmp/fast_livo2.m6a10-v2c.patch

# v8 is an additive source delta over the immutable v7 base patch.  Keeping
# the two files separate makes the v7 provenance hash stable while requiring
# the v8 image build to verify and apply both layers in this exact order.
COPY docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch \
     /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch
RUN test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256}" \
 && test "${FAST_LIVO2_M6A10_VARIANT}" = v8 \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch \
 && test -f /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_consumer_evidence.h \
 && grep -q 'm6a10_service_spinner' \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp \
 && rm -f /tmp/fast_livo2.m6a10-v2c-v8-service-queue.patch

# Build the legacy Sophus release required by FAST-LIVO2.  The two source
# edits are the same compatibility edits used by the historical image.
RUN git clone --filter=blob:none https://github.com/strasdat/Sophus.git /tmp/Sophus \
 && git -C /tmp/Sophus checkout "${SOPHUS_REVISION}" \
 && sed -i \
      -e 's/unit_complex_\.real() = 1\.;/unit_complex_.real(1.);/' \
      -e 's/unit_complex_\.imag() = 0\.;/unit_complex_.imag(0.);/' \
      /tmp/Sophus/sophus/so2.cpp \
 && cmake -S /tmp/Sophus -B /tmp/Sophus/build -GNinja \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
 && cmake --build /tmp/Sophus/build --parallel 2 \
 && cmake --install /tmp/Sophus/build \
 && rm -rf /tmp/Sophus \
 && install -d /usr/local/lib/cmake/Sophus \
 && printf '%s\n' \
      'set(Sophus_FOUND TRUE)' \
      'set(Sophus_INCLUDE_DIRS /usr/local/include)' \
      'set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)' \
      > /usr/local/lib/cmake/Sophus/SophusConfig.cmake

# The upstream CMakeLists enables -march=native/-ffast-math on the host.  A
# benchmark image must not silently encode host-specific CPU instructions, so
# retain Release optimization while replacing those flags with a portable
# deterministic floating-point contract.
RUN sed -i \
      -e 's/-march=native -mtune=native -funroll-loops/-ffp-contract=off/g' \
      -e 's/-ffast-math/-ffp-contract=off/g' \
      /opt/fast_livo_ws/src/FAST-LIVO2/CMakeLists.txt \
 && source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

RUN printf '%s\n' \
      '#!/usr/bin/env bash' \
      'set -euo pipefail' \
      'source /opt/ros/noetic/setup.bash' \
      'source /opt/fast_livo_ws/devel/setup.bash' \
      'exec "$@"' \
      > /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh

# This argument only affects the provenance label. Keep it immediately before
# LABEL so a feeder-only identity update does not invalidate dependency,
# source, and build layers above it.
ARG FAST_LIVO2_M6A10_FEEDER_SHA256=1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831
ARG FAST_LIVO2_M6A10_RUNNER_SHA256=cf1120f79c0cf2c10eab309ec56e6edc93f6045ec2d80c5821263c8af2f2cf30
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
