# FAST-LIVO2 M6A10 v17 production retry-boundary candidate.
#
# The v15 image is immutable.  This layer applies only the v17 producer patch
# to the actual FAST-LIVO2 header and LIVMapper call sites, recompiles the
# workspace, and runs production-header/selftest gates.  No sensor input,
# ground-truth, scorer, map, or formal replay is copied or executed here.
ARG FAST_LIVO2_V15_BASE_IMAGE=m6a10-v2c-v15-schema3-feeder-20260823t203230z-correction-fast-livo2-benchmark:ros1-pinned
FROM ${FAST_LIVO2_V15_BASE_IMAGE}

ARG FAST_LIVO2_V15_BASE_IMAGE
ARG FAST_LIVO2_V15_BASE_ID=sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a
ARG FAST_LIVO2_V17_PATCH_SHA256
ARG FAST_LIVO2_V17_SELFTEST_SHA256
ARG FAST_LIVO2_V17_WRAPPER_SHA256
ARG FAST_LIVO2_V17_PAYLOAD_SHA256
ARG FAST_LIVO2_V17_PROFILE_SHA256
ARG FAST_LIVO2_V15_FEEDER_SHA256=6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7
ARG FAST_LIVO2_V12_TERMINAL_SELFTEST_SHA256
ARG FAST_LIVO2_V12_CONSUMER_SELFTEST_SHA256
ARG FAST_LIVO2_V12_STUB_ROS_SHA256
ARG FAST_LIVO2_V12_STUB_TRIGGER_SHA256
ARG FAST_LIVO2_V12_FIXTURE_SHA256
ARG FAST_LIVO2_PHASE_CONTRACT=m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary
ARG FAST_LIVO2_TRANSPORT_CONTRACT=m6a10-v12-callback-ack-transport-outstanding-v1
ARG FAST_LIVO2_V17_VARIANT=v17-retryable-empty-synchronization-abort-correction
ARG FAST_LIVO2_V17_HEADER_SHA256=e45aace02cf153e1eeb00981628e545b8cc5fd279c185a627ce6b3ba18a16778
ARG FAST_LIVO2_V17_MAPPER_SHA256=c2a62cf0a6943a8c68084e70585955212bbe944b8788d285e4ca41845492ba94

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN mkdir -p /opt/fast_livo_v17 /runner/scripts \
    /runner/configs/slam_benchmark_profiles

COPY docker/patches/fast_livo2.m6a10-v2c-v17-retryable-abort.patch \
     /tmp/fast_livo2.m6a10-v2c-v17-retryable-abort.patch
COPY tools/m6a10_terminal_support_context_v17_production_selftest.cpp \
     /tmp/m6a10_terminal_support_context_v17_production_selftest.cpp
COPY tools/m6a10_terminal_support_context_v12_selftest.cpp \
     /tmp/m6a10_terminal_support_context_v12_selftest.cpp
COPY tools/m6a10_consumer_evidence_v12_selftest.cpp \
     /tmp/m6a10_consumer_evidence_v12_selftest.cpp
COPY tools/m6a10_v12_test_stubs /tmp/m6a10_v12_test_stubs
COPY scripts/fast_livo2_m6a10_v17_formal_container_run.sh \
     /runner/scripts/fast_livo2_m6a10_v17_formal_container_run.sh
COPY scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh \
     /runner/scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh
COPY scripts/fast_livo2_m6a10_v15_feeder.py \
     /runner/scripts/fast_livo2_m6a10_v15_feeder.py
COPY configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml \
     /runner/configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml
COPY graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json \
     /opt/fast_livo_v17/m6a10_v17_consumer_status_pass.json

RUN test -n "${FAST_LIVO2_V15_BASE_ID}" \
 && test "${FAST_LIVO2_V15_BASE_ID}" = \
      "sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a" \
 && test "$(sha256sum /tmp/fast_livo2.m6a10-v2c-v17-retryable-abort.patch | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_PATCH_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_terminal_support_context_v17_production_selftest.cpp | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_SELFTEST_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_v17_formal_container_run.sh | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_WRAPPER_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_PAYLOAD_SHA256}" \
 && test "$(sha256sum /runner/configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_PROFILE_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_v15_feeder.py | awk '{print $1}')" = \
      "${FAST_LIVO2_V15_FEEDER_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_v12_test_stubs/ros/ros.h | awk '{print $1}')" = \
      "${FAST_LIVO2_V12_STUB_ROS_SHA256}" \
 && test "$(sha256sum /tmp/m6a10_v12_test_stubs/std_srvs/Trigger.h | awk '{print $1}')" = \
      "${FAST_LIVO2_V12_STUB_TRIGGER_SHA256}" \
 && test "$(sha256sum /opt/fast_livo_v17/m6a10_v17_consumer_status_pass.json | awk '{print $1}')" = \
      "${FAST_LIVO2_V12_FIXTURE_SHA256}" \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 add -A \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount \
      /tmp/fast_livo2.m6a10-v2c-v17-retryable-abort.patch \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount \
      /tmp/fast_livo2.m6a10-v2c-v17-retryable-abort.patch \
 && test "$(sha256sum /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_HEADER_SHA256}" \
 && test "$(sha256sum /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp | awk '{print $1}')" = \
      "${FAST_LIVO2_V17_MAPPER_SHA256}" \
 && grep -q 'void abort_retryable_synchronization_unit()' \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h \
 && test "$(grep -c 'abort_retryable_synchronization_unit()' \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp)" = 3 \
 && grep -q 'abort_synchronization_unit();' \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp \
 && chmod 0755 /runner/scripts/fast_livo2_m6a10_v17_formal_container_run.sh \
      /runner/scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh

RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /tmp/m6a10-v17-selftest \
 && g++ -std=c++17 -Wall -Wextra -Werror -O2 \
      -I/opt/ros/noetic/include \
      -I/opt/fast_livo_ws/src/FAST-LIVO2/include \
      /tmp/m6a10_terminal_support_context_v17_production_selftest.cpp \
      -o /opt/fast_livo_v17/m6a10_terminal_support_context_v17_production_selftest \
      $(pkg-config --libs roscpp std_srvs) \
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
 && M6A10_SELFTEST_OUTPUT_DIR=/tmp/m6a10-v17-selftest \
      /opt/fast_livo_v17/m6a10_terminal_support_context_v17_production_selftest \
 && M6A10_SELFTEST_OUTPUT_DIR=/tmp/m6a10-v17-selftest \
      /tmp/m6a10_terminal_support_context_v12_selftest \
 && M6A10_SELFTEST_OUTPUT_DIR=/tmp/m6a10-v17-selftest \
      /tmp/m6a10_consumer_evidence_v12_selftest \
 && PYTHONPATH=/runner/scripts python3 -m py_compile \
      /runner/scripts/fast_livo2_m6a10_v15_feeder.py \
 && PYTHONPATH=/runner/scripts python3 - "${FAST_LIVO2_V15_FEEDER_SHA256}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path
from fast_livo2_m6a10_v15_feeder import validate_consumer_status
fixture = Path('/opt/fast_livo_v17/m6a10_v17_consumer_status_pass.json')
value = json.loads(fixture.read_bytes().decode('utf-8'))
assert validate_consumer_status(value)['schema_version'] == 3
assert hashlib.sha256(Path('/runner/scripts/fast_livo2_m6a10_v15_feeder.py').read_bytes()).hexdigest() == sys.argv[1]
PY

RUN source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

RUN sha256sum \
      /opt/fast_livo_ws/src/FAST-LIVO2/include/m6a10_terminal_support_context.h \
      /opt/fast_livo_ws/src/FAST-LIVO2/src/LIVMapper.cpp \
      > /opt/fast_livo_v17/installed_source_sha256.txt \
 && test -s /opt/fast_livo_v17/installed_source_sha256.txt

LABEL benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_V17_VARIANT}" \
      benchmark.fast_livo2.m6a10_v15_base_image="${FAST_LIVO2_V15_BASE_IMAGE}" \
      benchmark.fast_livo2.m6a10_v15_base_id="${FAST_LIVO2_V15_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v17_patch_sha256="${FAST_LIVO2_V17_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_v17_header_sha256="${FAST_LIVO2_V17_HEADER_SHA256}" \
      benchmark.fast_livo2.m6a10_v17_mapper_sha256="${FAST_LIVO2_V17_MAPPER_SHA256}" \
      benchmark.fast_livo2.m6a10_v17_selftest_sha256="${FAST_LIVO2_V17_SELFTEST_SHA256}" \
      benchmark.fast_livo2.m6a10_v17_wrapper_sha256="${FAST_LIVO2_V17_WRAPPER_SHA256}" \
      benchmark.fast_livo2.m6a10_v17_payload_sha256="${FAST_LIVO2_V17_PAYLOAD_SHA256}" \
      benchmark.fast_livo2.m6a10_v17_profile_sha256="${FAST_LIVO2_V17_PROFILE_SHA256}" \
      benchmark.fast_livo2.m6a10_v15_feeder_sha256="${FAST_LIVO2_V15_FEEDER_SHA256}" \
      benchmark.fast_livo2.m6a10_phase_contract="${FAST_LIVO2_PHASE_CONTRACT}" \
      benchmark.fast_livo2.m6a10_transport_contract="${FAST_LIVO2_TRANSPORT_CONTRACT}" \
      benchmark.fast_livo2.m6a10_network_expectation="none" \
      benchmark.fast_livo2.m6a10_rootfs_expectation="read_only_runtime" \
      benchmark.fast_livo2.m6a10_input_present="false" \
      benchmark.fast_livo2.m6a10_ground_truth_present="false" \
      benchmark.fast_livo2.m6a10_scorer_present="false" \
      benchmark.fast_livo2.m6a10_map_present="false" \
      benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"
