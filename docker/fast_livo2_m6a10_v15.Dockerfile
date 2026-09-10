# FAST-LIVO2 M6a10 v15 additive schema-compatible feeder image.
#
# The v12 image is immutable.  v15 changes only the feeder source placement:
# the legacy implementation is retained under a private module name and the
# v15 adapter occupies the path used by the production wrapper.  No input,
# GT, scorer, map, or formal replay is part of this image build gate.
ARG FAST_LIVO2_V12_BASE_IMAGE=m6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned
FROM ${FAST_LIVO2_V12_BASE_IMAGE}

ARG FAST_LIVO2_V12_BASE_IMAGE
ARG FAST_LIVO2_V12_BASE_ID=sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7
ARG FAST_LIVO2_V15_FEEDER_SHA256
ARG FAST_LIVO2_LEGACY_FEEDER_SHA256
ARG FAST_LIVO2_V15_WRAPPER_SHA256
ARG FAST_LIVO2_V15_PROFILE_SHA256
ARG FAST_LIVO2_V15_FIXTURE_SHA256
ARG FAST_LIVO2_V12_PATCH_SHA256=39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333
ARG FAST_LIVO2_PHASE_CONTRACT=m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary
ARG FAST_LIVO2_TRANSPORT_CONTRACT=m6a10-v12-callback-ack-transport-outstanding-v1
ARG FAST_LIVO2_V15_VARIANT=v15-schema3-feeder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN mkdir -p /opt/fast_livo_v15 /runner/scripts /runner/configs/slam_benchmark_profiles

COPY scripts/fast_livo2_m6a10_feeder.py \
     /runner/scripts/fast_livo2_m6a10_feeder_legacy.py
COPY scripts/fast_livo2_m6a10_v15_feeder.py \
     /runner/scripts/fast_livo2_m6a10_feeder.py
COPY scripts/fast_livo2_m6a10_v15_feeder.py \
     /runner/scripts/fast_livo2_m6a10_v15_feeder.py
COPY scripts/fast_livo2_m6a10_v15_formal_container_run.sh \
     /runner/scripts/fast_livo2_m6a10_v15_formal_container_run.sh
COPY scripts/fast_livo2_m6a10_v12_formal_container_run.sh \
     /runner/scripts/fast_livo2_m6a10_v12_formal_container_run.sh
COPY configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml \
     /runner/configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml
COPY graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json \
     /opt/fast_livo_v15/m6a10_v15_consumer_status_pass.json

RUN test -n "${FAST_LIVO2_V12_BASE_ID}" \
 && test "${FAST_LIVO2_V12_BASE_ID}" = \
      "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7" \
 && test -n "${FAST_LIVO2_V15_FEEDER_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_feeder.py | awk '{print $1}')" = \
      "${FAST_LIVO2_V15_FEEDER_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_v15_feeder.py | awk '{print $1}')" = \
      "${FAST_LIVO2_V15_FEEDER_SHA256}" \
 && test -n "${FAST_LIVO2_LEGACY_FEEDER_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_feeder_legacy.py | awk '{print $1}')" = \
      "${FAST_LIVO2_LEGACY_FEEDER_SHA256}" \
 && test -n "${FAST_LIVO2_V15_WRAPPER_SHA256}" \
 && test "$(sha256sum /runner/scripts/fast_livo2_m6a10_v15_formal_container_run.sh | awk '{print $1}')" = \
      "${FAST_LIVO2_V15_WRAPPER_SHA256}" \
 && test -n "${FAST_LIVO2_V15_PROFILE_SHA256}" \
 && test "$(sha256sum /runner/configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml | awk '{print $1}')" = \
      "${FAST_LIVO2_V15_PROFILE_SHA256}" \
 && test -n "${FAST_LIVO2_V15_FIXTURE_SHA256}" \
 && test "$(sha256sum /opt/fast_livo_v15/m6a10_v15_consumer_status_pass.json | awk '{print $1}')" = \
      "${FAST_LIVO2_V15_FIXTURE_SHA256}" \
 && test -n "${FAST_LIVO2_V12_PATCH_SHA256}" \
 && test "${FAST_LIVO2_V12_PATCH_SHA256}" = \
      "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333" \
 && test "${FAST_LIVO2_PHASE_CONTRACT}" = \
      "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary" \
 && test "${FAST_LIVO2_TRANSPORT_CONTRACT}" = \
      "m6a10-v12-callback-ack-transport-outstanding-v1" \
 && python3 -m py_compile /runner/scripts/fast_livo2_m6a10_feeder.py \
      /runner/scripts/fast_livo2_m6a10_feeder_legacy.py \
 && PYTHONPATH=/runner/scripts python3 -c \
      "import json; from fast_livo2_m6a10_v15_feeder import validate_consumer_status; p='/opt/fast_livo_v15/m6a10_v15_consumer_status_pass.json'; v=json.load(open(p)); assert validate_consumer_status(v)['schema_version'] == 3" \
 && test -f /runner/scripts/fast_livo2_m6a10_v15_formal_container_run.sh \
 && test ! -L /runner/scripts/fast_livo2_m6a10_feeder.py \
 && test ! -L /runner/scripts/fast_livo2_m6a10_feeder_legacy.py

LABEL benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_V15_VARIANT}" \
      benchmark.fast_livo2.m6a10_v12_base_image="${FAST_LIVO2_V12_BASE_IMAGE}" \
      benchmark.fast_livo2.m6a10_v12_base_id="${FAST_LIVO2_V12_BASE_ID}" \
      benchmark.fast_livo2.m6a10_v12_patch_sha256="${FAST_LIVO2_V12_PATCH_SHA256}" \
      benchmark.fast_livo2.m6a10_v15_feeder_sha256="${FAST_LIVO2_V15_FEEDER_SHA256}" \
      benchmark.fast_livo2.m6a10_legacy_feeder_sha256="${FAST_LIVO2_LEGACY_FEEDER_SHA256}" \
      benchmark.fast_livo2.m6a10_v15_wrapper_sha256="${FAST_LIVO2_V15_WRAPPER_SHA256}" \
      benchmark.fast_livo2.m6a10_v15_profile_sha256="${FAST_LIVO2_V15_PROFILE_SHA256}" \
      benchmark.fast_livo2.m6a10_v15_fixture_sha256="${FAST_LIVO2_V15_FIXTURE_SHA256}" \
      benchmark.fast_livo2.m6a10_phase_contract="${FAST_LIVO2_PHASE_CONTRACT}" \
      benchmark.fast_livo2.m6a10_transport_contract="${FAST_LIVO2_TRANSPORT_CONTRACT}" \
      benchmark.fast_livo2.m6a10_network_expectation="none" \
      benchmark.fast_livo2.m6a10_rootfs_expectation="read_only_runtime" \
      benchmark.fast_livo2.m6a10_input_present="false" \
      benchmark.fast_livo2.m6a10_ground_truth_present="false" \
      benchmark.fast_livo2.m6a10_scorer_present="false" \
      benchmark.fast_livo2.m6a10_map_present="false" \
      benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"
