# FAST-LIVO2 M6a10 v11 bounded-end-gap build candidate.
#
# This image is a source/build candidate layered on the exact v10 image.  It
# contains no bag, ground-truth, scorer, or replay entrypoint.
ARG FAST_LIVO2_V10_BASE_IMAGE=m6a10-v2c-v10-callback-v3-20260823t094100z-fast-livo2-benchmark:ros1-pinned
FROM ${FAST_LIVO2_V10_BASE_IMAGE}

ARG FAST_LIVO2_V10_BASE_IMAGE
ARG FAST_LIVO2_V10_BASE_ID
ARG FAST_LIVO2_V10_EXPECTED_BASE_ID=sha256:3e087acc5ef116f03357a73927c18b2059068b093ae1cce6cb41c1baf1fbf759
ARG FAST_LIVO2_M6A10_V11_PATCH_SHA256
ARG FAST_LIVO2_M6A10_V11_SELFTEST_SHA256
ARG FAST_LIVO2_M6A10_VARIANT=v11
ARG FAST_LIVO2_M6A10_TERMINAL_CONTRACT=m6a10-online-compute-v4-terminal-bounded-end-gap

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch \
     /tmp/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch
COPY tools/m6a10_terminal_support_context_v11_selftest.cpp \
     /tmp/m6a10_terminal_support_context_v11_selftest.cpp

RUN test -n "${FAST_LIVO2_V10_BASE_ID}" \
 && test "${FAST_LIVO2_V10_BASE_ID}" = "${FAST_LIVO2_V10_EXPECTED_BASE_ID}" \
 && test -n "${FAST_LIVO2_M6A10_V11_PATCH_SHA256}" \
 && test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V11_PATCH_SHA256}" \
 && test -n "${FAST_LIVO2_M6A10_V11_SELFTEST_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_terminal_support_context_v11_selftest.cpp | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V11_SELFTEST_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 add -A \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch \
 && grep -q 'm6a10-online-compute-v4-terminal-bounded-end-gap' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h \
 && ! grep -q 'boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h \
 && grep -q 'contract_value == "m6a10-online-compute-v4-terminal-bounded-end-gap"' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_consumer_evidence.h \
 && rm -f /tmp/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch

# The inherited v10 workspace contains the pinned dependencies and source
# lineage.  Rebuild only to materialize this v11 source candidate.
RUN source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

# Compile the exact production-header synthetic gate; do not execute it here.
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /opt/fast_livo_ws/devel/lib/fast_livo \
 && g++ -std=c++17 -O0 \
      -I/opt/ros/noetic/include \
      -I/opt/fast_livo_ws/src/FAST-LIVO2/include \
      /tmp/m6a10_terminal_support_context_v11_selftest.cpp \
      -o /opt/fast_livo_ws/devel/lib/fast_livo/m6a10_terminal_support_context_v11_selftest \
      $(pkg-config --libs roscpp std_srvs) \
 && chmod 0755 /opt/fast_livo_ws/devel/lib/fast_livo/m6a10_terminal_support_context_v11_selftest \
 && rm -f /tmp/m6a10_terminal_support_context_v11_selftest.cpp

LABEL benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_M6A10_VARIANT}" \
      benchmark.fast_livo2.m6a10_terminal_contract="${FAST_LIVO2_M6A10_TERMINAL_CONTRACT}" \
      benchmark.fast_livo2.m6a10_v10_base_image="${FAST_LIVO2_V10_BASE_IMAGE}" \
      benchmark.fast_livo2.m6a10_v10_base_id="${FAST_LIVO2_V10_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v10_expected_base_id="${FAST_LIVO2_V10_EXPECTED_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v11_patch_sha256="${FAST_LIVO2_M6A10_V11_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_v11_selftest_sha256="${FAST_LIVO2_M6A10_V11_SELFTEST_SHA256}" \
      benchmark.fast_livo2.m6a10_ground_truth_present="false" \
      benchmark.fast_livo2.m6a10_scorer_present="false" \
      benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"
