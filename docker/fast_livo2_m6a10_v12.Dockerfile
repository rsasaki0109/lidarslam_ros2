# FAST-LIVO2 M6a10 v12 non-LiDAR boundary/transport build candidate.
#
# The v11 image is an immutable, already-gated lineage.  This recipe applies
# only the v12 delta and contains no input, ground truth, scorer, or replay
# entrypoint.  The image is not built by the repository tests.
ARG FAST_LIVO2_V11_BASE_IMAGE=m6a10-v2c-v11-bounded-end-gap-20260823t121749z-fast-livo2-benchmark:ros1-pinned
FROM ${FAST_LIVO2_V11_BASE_IMAGE}

ARG FAST_LIVO2_V11_BASE_IMAGE
ARG FAST_LIVO2_V11_BASE_ID
ARG FAST_LIVO2_V11_EXPECTED_BASE_ID=sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a
ARG FAST_LIVO2_M6A10_V12_PATCH_SHA256
ARG FAST_LIVO2_M6A10_V12_TERMINAL_SELFTEST_SHA256
ARG FAST_LIVO2_M6A10_V12_CONSUMER_SELFTEST_SHA256
ARG FAST_LIVO2_M6A10_V12_WRAPPER_SHA256
ARG FAST_LIVO2_M6A10_V12_STUB_ROS_SHA256
ARG FAST_LIVO2_M6A10_V12_STUB_TRIGGER_SHA256
ARG FAST_LIVO2_M6A10_VARIANT=v12
ARG FAST_LIVO2_M6A10_TERMINAL_CONTRACT=m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary
ARG FAST_LIVO2_M6A10_TRANSPORT_CONTRACT=m6a10-v12-callback-ack-transport-outstanding-v1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch \
     /tmp/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch
COPY tools/m6a10_terminal_support_context_v12_selftest.cpp \
     /tmp/m6a10_terminal_support_context_v12_selftest.cpp
COPY tools/m6a10_consumer_evidence_v12_selftest.cpp \
     /tmp/m6a10_consumer_evidence_v12_selftest.cpp
COPY tools/m6a10_v12_test_stubs /tmp/m6a10_v12_test_stubs
COPY scripts/fast_livo2_m6a10_v12_formal_container_run.sh \
     /tmp/fast_livo2_m6a10_v12_formal_container_run.sh

RUN test -n "${FAST_LIVO2_V11_BASE_ID}" \
 && test "${FAST_LIVO2_V11_BASE_ID}" = "${FAST_LIVO2_V11_EXPECTED_BASE_ID}" \
 && test -n "${FAST_LIVO2_M6A10_V12_PATCH_SHA256}" \
 && test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V12_PATCH_SHA256}" \
 && test -n "${FAST_LIVO2_M6A10_V12_TERMINAL_SELFTEST_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_terminal_support_context_v12_selftest.cpp | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V12_TERMINAL_SELFTEST_SHA256}" \
 && test -n "${FAST_LIVO2_M6A10_V12_CONSUMER_SELFTEST_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_consumer_evidence_v12_selftest.cpp | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V12_CONSUMER_SELFTEST_SHA256}" \
 && test -n "${FAST_LIVO2_M6A10_V12_WRAPPER_SHA256}" \
 && test "$(sha256sum /tmp/fast_livo2_m6a10_v12_formal_container_run.sh | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V12_WRAPPER_SHA256}" \
 && test -n "${FAST_LIVO2_M6A10_V12_STUB_ROS_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_v12_test_stubs/ros/ros.h | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V12_STUB_ROS_SHA256}" \
 && test -n "${FAST_LIVO2_M6A10_V12_STUB_TRIGGER_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_v12_test_stubs/std_srvs/Trigger.h | awk '{print $1}')" = \
      "${FAST_LIVO2_M6A10_V12_STUB_TRIGGER_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 add -A \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch \
 && grep -q "${FAST_LIVO2_M6A10_TERMINAL_CONTRACT}" \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h \
 && grep -q "${FAST_LIVO2_M6A10_TRANSPORT_CONTRACT}" \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_consumer_evidence.h \
 && grep -q 'transport_contract_version' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_consumer_evidence.h \
 && grep -q 'maximum_transport_outstanding_messages' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_consumer_evidence.h

# Compile and run both production-header selftests against the patched v11
# source before the workspace rebuild.  The copied stubs are host-test
# fixtures; actual image compilation uses the pinned ROS headers.
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /tmp/m6a10-v12-selftest \
 && g++ -std=c++17 -Wall -Wextra -Werror -O2 \
      -I/opt/ros/noetic/include \
      -I/opt/fast_livo_ws/src/FAST-LIVO2/include \
      /tmp/m6a10_terminal_support_context_v12_selftest.cpp \
      -o /tmp/m6a10_terminal_support_context_v12_selftest \
      $(pkg-config --libs roscpp std_srvs) \
 && g++ -std=c++17 -Wall -Wextra -Werror -O2 \
      -I/opt/ros/noetic/include \
      -I/opt/fast_livo_ws/src/FAST-LIVO2/include \
      /tmp/m6a10_consumer_evidence_v12_selftest.cpp \
      -o /tmp/m6a10_consumer_evidence_v12_selftest \
      $(pkg-config --libs roscpp std_srvs) \
 && M6A10_SELFTEST_OUTPUT_DIR=/tmp/m6a10-v12-selftest \
      /tmp/m6a10_terminal_support_context_v12_selftest \
 && M6A10_SELFTEST_OUTPUT_DIR=/tmp/m6a10-v12-selftest \
      /tmp/m6a10_consumer_evidence_v12_selftest

RUN source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

LABEL benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_M6A10_VARIANT}" \
      benchmark.fast_livo2.m6a10_terminal_contract="${FAST_LIVO2_M6A10_TERMINAL_CONTRACT}" \
      benchmark.fast_livo2.m6a10_transport_contract="${FAST_LIVO2_M6A10_TRANSPORT_CONTRACT}" \
      benchmark.fast_livo2.m6a10_v11_base_image="${FAST_LIVO2_V11_BASE_IMAGE}" \
      benchmark.fast_livo2.m6a10_v11_base_id="${FAST_LIVO2_V11_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v11_expected_base_id="${FAST_LIVO2_V11_EXPECTED_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v12_patch_sha256="${FAST_LIVO2_M6A10_V12_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_v12_terminal_selftest_sha256="${FAST_LIVO2_M6A10_V12_TERMINAL_SELFTEST_SHA256}" \
      benchmark.fast_livo2.m6a10_v12_consumer_selftest_sha256="${FAST_LIVO2_M6A10_V12_CONSUMER_SELFTEST_SHA256}" \
      benchmark.fast_livo2.m6a10_v12_wrapper_sha256="${FAST_LIVO2_M6A10_V12_WRAPPER_SHA256}" \
      benchmark.fast_livo2.m6a10_v12_stubs_sha256="${FAST_LIVO2_M6A10_V12_STUB_ROS_SHA256}:${FAST_LIVO2_M6A10_V12_STUB_TRIGGER_SHA256}" \
      benchmark.fast_livo2.m6a10_network_expectation="none" \
      benchmark.fast_livo2.m6a10_rootfs_expectation="read_only_runtime" \
      benchmark.fast_livo2.m6a10_ground_truth_present="false" \
      benchmark.fast_livo2.m6a10_scorer_present="false" \
      benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"
