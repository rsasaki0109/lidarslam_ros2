# FAST-LIVO2 M6a10 v10 additive compile/runtime image.
#
# The v9 image is the pinned source/dependency lineage.  v10 adds only the
# terminal support-context observation patch and recompiles that exact source
# tree.  No bag, ground truth, or scorer is present in this image.
ARG FAST_LIVO2_V9_BASE_IMAGE=m6a10-v2c-v9wallrate-20260823t040642-fast-livo2-benchmark:ros1-pinned
FROM ${FAST_LIVO2_V9_BASE_IMAGE}

ARG FAST_LIVO2_V9_BASE_IMAGE
ARG FAST_LIVO2_V9_BASE_ID
ARG FAST_LIVO2_M6A10_V10_PATCH_SHA256
ARG FAST_LIVO2_M6A10_VARIANT=v10
ARG FAST_LIVO2_M6A10_TERMINAL_CONTRACT=m6a10-online-compute-v3-terminal-support-context

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch \
     /tmp/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch
COPY tools/m6a10_terminal_support_context_selftest.cpp \
     /tmp/m6a10_terminal_support_context_selftest.cpp

RUN test -n "${FAST_LIVO2_M6A10_V10_PATCH_SHA256}" \
 && test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V10_PATCH_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch \
 && grep -q 'm6a10_terminal_status_service' \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp \
 && grep -q 'm6a10-online-compute-v3-terminal-support-context' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h \
 && rm -f /tmp/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch

# The inherited v9 workspace already contains all dependencies and build
# prerequisites.  Re-run catkin_make so the image contains the compiled v10
# executable; network access is unnecessary for this layer.
RUN source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

# This executable is a benchmark-only synthetic gate.  It links the exact
# header emitted above; it does not open a bag or participate in the mapper.
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /opt/fast_livo_ws/devel/lib/fast_livo \
 && g++ -std=c++17 -O0 \
      -I/opt/ros/noetic/include \
      -I/opt/fast_livo_ws/src/FAST-LIVO2/include \
      /tmp/m6a10_terminal_support_context_selftest.cpp \
      -o /opt/fast_livo_ws/devel/lib/fast_livo/m6a10_terminal_support_context_selftest \
      $(pkg-config --libs roscpp std_srvs) \
 && chmod 0755 /opt/fast_livo_ws/devel/lib/fast_livo/m6a10_terminal_support_context_selftest \
 && rm -f /tmp/m6a10_terminal_support_context_selftest.cpp

LABEL benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_M6A10_VARIANT}" \
      benchmark.fast_livo2.m6a10_terminal_contract="${FAST_LIVO2_M6A10_TERMINAL_CONTRACT}" \
      benchmark.fast_livo2.m6a10_v9_base_image="${FAST_LIVO2_V9_BASE_IMAGE}" \
      benchmark.fast_livo2.m6a10_v9_base_id="${FAST_LIVO2_V9_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v10_patch_sha256="${FAST_LIVO2_M6A10_V10_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_ground_truth_present="false" \
      benchmark.fast_livo2.m6a10_scorer_present="false"
