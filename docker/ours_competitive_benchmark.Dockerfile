# Reproducible ours benchmark image for the pinned ROS 2 Jazzy track.
FROM docker.io/library/ros:jazzy-ros-base@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

ARG OURS_REVISION=866f733677e92ecb08d67126e463da99dd140d46
ARG OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git
ARG NDT_OMP_SUBMODULE_REVISION=497411279593eb261a3e3d04cdcbb4717af33ca3
ARG RKO_LIO_GITLINK_REVISION=622b74778a41f753d47aa5918043755ebcbd4c75
ARG RKO_LIO_ARCHIVE_SHA256=95783dca0cdac394052fbdb03cf1636cf51ee65a2316fbaef3a67bfe4c88d09f
ARG RKO_LIO_PATCH_SHA256=67a7b0f9c0118e51604fd89690f682b40f1c24fcd5b39ed216e04bc47e8ee503
ARG OURS_LAUNCH_SHA256=d45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c
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
      ca-certificates git python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ours_ws

# The build context contains only this recipe and a separately hashed archive
# of the exact rko_lio gitlink.  Fetch the declared superproject revision
# inside the image so a dirty host checkout cannot become benchmark input.
# The public mirror does not expose the pinned rko_lio object, so the archive
# is verified before extraction rather than silently substituting a revision.
COPY rko_lio.tar /tmp/rko_lio.tar
COPY rko_lio.m6a10-v2a.patch /tmp/rko_lio.m6a10-v2a.patch
COPY rko_lio_slam.launch.py /tmp/rko_lio_slam.launch.py
RUN git clone --no-checkout "$OURS_REPOSITORY" /opt/ours_ws/src/lidar_slam_ros2 \
 && git -C /opt/ours_ws/src/lidar_slam_ros2 checkout --detach "$OURS_REVISION" \
 && git -C /opt/ours_ws/src/lidar_slam_ros2 submodule sync --recursive \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 ls-tree HEAD Thirdparty/ndt_omp_ros2 | awk '{ print $3 }')" = "$NDT_OMP_SUBMODULE_REVISION" \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 ls-tree HEAD Thirdparty/rko_lio | awk '{ print $3 }')" = "$RKO_LIO_GITLINK_REVISION" \
 && git -C /opt/ours_ws/src/lidar_slam_ros2 submodule update --init --recursive -- Thirdparty/ndt_omp_ros2 \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 rev-parse HEAD)" = "$OURS_REVISION" \
 && test -z "$(git -C /opt/ours_ws/src/lidar_slam_ros2 status --porcelain=v1 --untracked-files=all)" \
 && test -z "$(git -C /opt/ours_ws/src/lidar_slam_ros2 submodule status --recursive -- Thirdparty/ndt_omp_ros2 \
      | awk '$1 ~ /^[-+U]/ { print; bad=1 } END { exit bad }')" \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 submodule status --recursive -- Thirdparty/rko_lio \
      | awk '{ print substr($1, 1, 1) }')" = "-" \
 && test "$(sha256sum /tmp/rko_lio.tar | awk '{ print $1 }')" = "$RKO_LIO_ARCHIVE_SHA256" \
 && test "$(sha256sum /tmp/rko_lio_slam.launch.py | awk '{ print $1 }')" = "$OURS_LAUNCH_SHA256" \
 && mkdir -p /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio \
 && tar -xf /tmp/rko_lio.tar -C /opt/ours_ws/src/lidar_slam_ros2/Thirdparty \
 && test -f /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/package.xml \
 && rm -f /tmp/rko_lio.tar

RUN set -eux \
 && test "$(sha256sum /tmp/rko_lio.m6a10-v2a.patch | awk '{ print $1 }')" = "$RKO_LIO_PATCH_SHA256" \
 && cd /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio \
 && patch --batch --forward --strip=1 < /tmp/rko_lio.m6a10-v2a.patch \
 && test -f /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/rko_lio/ros/offline_node.cpp \
 && grep -Fq 'BenchmarkConsumerCounters' \
      /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/rko_lio/ros/node.hpp \
 && grep -Fq 'target_link_libraries(offline_node PRIVATE rko_lio::ros nlohmann_json::nlohmann_json)' \
      /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/rko_lio/ros/CMakeLists.txt \
 && colcon list --base-paths /opt/ours_ws/src/lidar_slam_ros2 \
      | awk '$1 == "rko_lio" { found=1 } END { exit !found }' \
 && rm -f /tmp/rko_lio.m6a10-v2a.patch

# The algorithm checkout is pinned to OURS_REVISION, while this benchmark-only
# launch overlay is separately content-addressed.  Do not COPY the dirty host
# checkout: only the explicitly hashed launch file enters the build context.
RUN install -m 0644 /tmp/rko_lio_slam.launch.py \
      /opt/ours_ws/src/lidar_slam_ros2/lidarslam/launch/rko_lio_slam.launch.py \
 && test "$(sha256sum /opt/ours_ws/src/lidar_slam_ros2/lidarslam/launch/rko_lio_slam.launch.py \
      | awk '{ print $1 }')" = "$OURS_LAUNCH_SHA256" \
 && rm -f /tmp/rko_lio_slam.launch.py

RUN apt-get update \
 && if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
 && rosdep update --rosdistro jazzy \
 && source /opt/ros/jazzy/setup.bash \
 && rosdep install -r -y --from-paths /opt/ours_ws/src/lidar_slam_ros2 \
      --ignore-src --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --base-paths /opt/ours_ws/src/lidar_slam_ros2 \
      --packages-up-to rko_lio lidarslam graph_based_slam \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Keep the benchmark image fail-closed if a later dependency/build step ever
# replaces the patched source with the uninstrumented archive.
RUN grep -Fq 'BenchmarkConsumerCounters' \
      /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/rko_lio/ros/node.hpp \
 && grep -Fq 'target_link_libraries(offline_node PRIVATE rko_lio::ros nlohmann_json::nlohmann_json)' \
      /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/rko_lio/ros/CMakeLists.txt \
 && strings /opt/ours_ws/install/rko_lio/lib/rko_lio/offline_node \
      | grep -F 'm6a10-online-compute-v2' >/dev/null

LABEL org.opencontainers.image.source="https://github.com/rsasaki0109/lidar_slam_ros2" \
      benchmark.ours.repository="https://github.com/rsasaki0109/lidar_slam_ros2.git" \
      benchmark.ours.revision="866f733677e92ecb08d67126e463da99dd140d46" \
      benchmark.ndt_omp.submodule_revision="497411279593eb261a3e3d04cdcbb4717af33ca3" \
      benchmark.rko_lio.gitlink_revision="622b74778a41f753d47aa5918043755ebcbd4c75" \
      benchmark.rko_lio.initialized="true" \
      benchmark.rko_lio.archive_sha256="95783dca0cdac394052fbdb03cf1636cf51ee65a2316fbaef3a67bfe4c88d09f" \
      benchmark.rko_lio.patch_sha256="67a7b0f9c0118e51604fd89690f682b40f1c24fcd5b39ed216e04bc47e8ee503" \
      benchmark.ours.launch_path="lidarslam/launch/rko_lio_slam.launch.py" \
      benchmark.ours.launch_sha256="d45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c" \
      benchmark.base_image="docker.io/library/ros:jazzy-ros-base" \
      benchmark.base_digest="sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f" \
      benchmark.cpu_policy="cpu_only;threads=8"

CMD ["bash"]
