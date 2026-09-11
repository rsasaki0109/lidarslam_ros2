#!/usr/bin/env bash
# Exactly one additive v17 image build; no input or formal replay.
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
BASE_IMAGE="${FAST_LIVO2_V17_BASE_IMAGE:-m6a10-v2c-v15-schema3-feeder-20260823t203230z-correction-fast-livo2-benchmark:ros1-pinned}"
IMAGE_TAG="${FAST_LIVO2_V17_IMAGE_TAG:-m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned}"
DOCKERFILE_PATH="${ROOT_DIR}/docker/fast_livo2_m6a10_v17.Dockerfile"
PROFILE_PATH="${ROOT_DIR}/configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml"
PATCH_PATH="${ROOT_DIR}/docker/patches/fast_livo2.m6a10-v2c-v17-retryable-abort.patch"
SELFTEST_PATH="${ROOT_DIR}/tools/m6a10_terminal_support_context_v17_production_selftest.cpp"
WRAPPER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v17_formal_container_run.sh"
PAYLOAD_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh"
FEEDER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v15_feeder.py"
V12_TERM="${ROOT_DIR}/tools/m6a10_terminal_support_context_v12_selftest.cpp"
V12_CONSUMER="${ROOT_DIR}/tools/m6a10_consumer_evidence_v12_selftest.cpp"
STUB_ROS="${ROOT_DIR}/tools/m6a10_v12_test_stubs/ros/ros.h"
STUB_TRIGGER="${ROOT_DIR}/tools/m6a10_v12_test_stubs/std_srvs/Trigger.h"
FIXTURE="${ROOT_DIR}/graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json"

BASE_ID_EXPECTED="sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
PATCH_SHA="c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001"
SELFTEST_SHA="f81bde7cabe54ebfa702908afa9ef071f2436ce5fee2f50bf67e3c494972bc84"
WRAPPER_SHA="506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e"
PAYLOAD_SHA="a8043f751db0638f34e79254133ac5974527c609f7574c616ee219a685cd8ed5"
PROFILE_SHA="1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b"
FEEDER_SHA="6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
V12_TERM_SHA="1b42ed713dbb2a8614902ae6214d5790ab9c73ac59792f4d68753320cef47a91"
V12_CONSUMER_SHA="02078640496daa28f0be471355c6e2ad2822a76199cda1b54e738eeff5fe48de"
STUB_ROS_SHA="40c3811b6225da6ffdb2c98ed86b9d218f168ef7c1c0294825f7ad216387bb0a"
STUB_TRIGGER_SHA="34704c7662d96d395972ac2cc3a1a874331055bb48c5b16fdfb2494f5145dc24"
FIXTURE_SHA="a30a5d50ea9a02fe9d7e0edc230801b33ee57676a06ac5e3a9fe30d0eede4a0a"
PHASE_CONTRACT='m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
TRANSPORT_CONTRACT='m6a10-v12-callback-ack-transport-outstanding-v1'
RECEIPT="${FAST_LIVO2_V17_BUILD_RECEIPT_PATH:-/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json}"
ROOT_RECEIPT="$(dirname "${RECEIPT}")"
LOG="${FAST_LIVO2_V17_BUILD_LOG_PATH:-${ROOT_RECEIPT}/docker_build.log}"
INSPECT="${FAST_LIVO2_V17_BUILD_INSPECT_PATH:-${ROOT_RECEIPT}/image.inspect.json}"

sha() { sha256sum "$1" | awk '{print $1}'; }
fail() { echo "FAST-LIVO2 v17 build gate failed: $*" >&2; exit 1; }
for file in "${DOCKERFILE_PATH}" "${PROFILE_PATH}" "${PATCH_PATH}" "${SELFTEST_PATH}" \
  "${WRAPPER_PATH}" "${PAYLOAD_PATH}" "${FEEDER_PATH}" "${V12_TERM}" \
  "${V12_CONSUMER}" "${STUB_ROS}" "${STUB_TRIGGER}" "${FIXTURE}"; do
  test -f "${file}" || fail "missing source ${file}"
done
test "$(sha "${PATCH_PATH}")" = "${PATCH_SHA}"
test "$(sha "${SELFTEST_PATH}")" = "${SELFTEST_SHA}"
test "$(sha "${WRAPPER_PATH}")" = "${WRAPPER_SHA}"
test "$(sha "${PAYLOAD_PATH}")" = "${PAYLOAD_SHA}"
test "$(sha "${PROFILE_PATH}")" = "${PROFILE_SHA}"
test "$(sha "${FEEDER_PATH}")" = "${FEEDER_SHA}"
test "$(sha "${V12_TERM}")" = "${V12_TERM_SHA}"
test "$(sha "${V12_CONSUMER}")" = "${V12_CONSUMER_SHA}"
test "$(sha "${STUB_ROS}")" = "${STUB_ROS_SHA}"
test "$(sha "${STUB_TRIGGER}")" = "${STUB_TRIGGER_SHA}"
test "$(sha "${FIXTURE}")" = "${FIXTURE_SHA}"
[[ "${IMAGE_TAG}" == *v17* && "${IMAGE_TAG}" != "${BASE_IMAGE}" ]] || fail 'tag lineage'
test -z "${BAG_PATH:-}" && test -z "${GROUND_TRUTH_PATH:-}" && test -z "${SCORER_PATH:-}"
mkdir -p "${ROOT_RECEIPT}"
for file in "${RECEIPT}" "${RECEIPT}.sha256" "${LOG}" "${INSPECT}"; do
  test ! -e "${file}" && test ! -L "${file}" || fail "evidence exists ${file}"
done
docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1 && fail 'target image exists'
docker image inspect "${BASE_IMAGE}" >/dev/null || fail 'base image missing'
BASE_ID="$(docker image inspect "${BASE_IMAGE}" --format '{{.Id}}')"
test "${BASE_ID}" = "${BASE_ID_EXPECTED}" || fail 'base identity'

# This is intentionally the only build call; failures are preserved and not retried.
set +e
docker build --network none --pull=false -t "${IMAGE_TAG}" -f "${DOCKERFILE_PATH}" \
  --build-arg "FAST_LIVO2_V15_BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "FAST_LIVO2_V15_BASE_ID=${BASE_ID}" \
  --build-arg "FAST_LIVO2_V17_PATCH_SHA256=${PATCH_SHA}" \
  --build-arg "FAST_LIVO2_V17_SELFTEST_SHA256=${SELFTEST_SHA}" \
  --build-arg "FAST_LIVO2_V17_WRAPPER_SHA256=${WRAPPER_SHA}" \
  --build-arg "FAST_LIVO2_V17_PAYLOAD_SHA256=${PAYLOAD_SHA}" \
  --build-arg "FAST_LIVO2_V17_PROFILE_SHA256=${PROFILE_SHA}" \
  --build-arg "FAST_LIVO2_V15_FEEDER_SHA256=${FEEDER_SHA}" \
  --build-arg "FAST_LIVO2_V12_TERMINAL_SELFTEST_SHA256=${V12_TERM_SHA}" \
  --build-arg "FAST_LIVO2_V12_CONSUMER_SELFTEST_SHA256=${V12_CONSUMER_SHA}" \
  --build-arg "FAST_LIVO2_V12_STUB_ROS_SHA256=${STUB_ROS_SHA}" \
  --build-arg "FAST_LIVO2_V12_STUB_TRIGGER_SHA256=${STUB_TRIGGER_SHA}" \
  --build-arg "FAST_LIVO2_V12_FIXTURE_SHA256=${FIXTURE_SHA}" \
  --build-arg "FAST_LIVO2_PHASE_CONTRACT=${PHASE_CONTRACT}" \
  --build-arg "FAST_LIVO2_TRANSPORT_CONTRACT=${TRANSPORT_CONTRACT}" \
  "${ROOT_DIR}" >"${LOG}" 2>&1
BUILD_RC=$?
set -e
if (( BUILD_RC != 0 )); then
  echo "v17 build failed rc=${BUILD_RC}; log=${LOG}" >&2
  exit "${BUILD_RC}"
fi

IMAGE_ID="$(docker image inspect "${IMAGE_TAG}" --format '{{.Id}}')"
test -n "${IMAGE_ID}" && test "${IMAGE_ID}" != "${BASE_ID}"
docker image inspect "${IMAGE_TAG}" >"${INSPECT}"
LABELS="$(docker image inspect "${IMAGE_TAG}" --format '{{json .Config.Labels}}')"
python3 - "${LABELS}" <<'PY'
import json, sys
labels = json.loads(sys.argv[1])
expected = {
 'benchmark.fast_livo2.m6a10_v15_base_id': 'sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a',
 'benchmark.fast_livo2.m6a10_v17_patch_sha256': 'c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001',
 'benchmark.fast_livo2.m6a10_v17_header_sha256': 'e45aace02cf153e1eeb00981628e545b8cc5fd279c185a627ce6b3ba18a16778',
 'benchmark.fast_livo2.m6a10_v17_mapper_sha256': 'c2a62cf0a6943a8c68084e70585955212bbe944b8788d285e4ca41845492ba94',
 'benchmark.fast_livo2.m6a10_v17_selftest_sha256': 'f81bde7cabe54ebfa702908afa9ef071f2436ce5fee2f50bf67e3c494972bc84',
 'benchmark.fast_livo2.m6a10_v17_wrapper_sha256': '506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e',
 'benchmark.fast_livo2.m6a10_v17_payload_sha256': 'a8043f751db0638f34e79254133ac5974527c609f7574c616ee219a685cd8ed5',
 'benchmark.fast_livo2.m6a10_v17_profile_sha256': '1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b',
 'benchmark.fast_livo2.m6a10_phase_contract': 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary',
 'benchmark.fast_livo2.m6a10_transport_contract': 'm6a10-v12-callback-ack-transport-outstanding-v1',
 'benchmark.fast_livo2.m6a10_network_expectation': 'none',
 'benchmark.fast_livo2.m6a10_rootfs_expectation': 'read_only_runtime',
 'benchmark.fast_livo2.m6a10_input_present': 'false',
 'benchmark.fast_livo2.m6a10_ground_truth_present': 'false',
 'benchmark.fast_livo2.m6a10_scorer_present': 'false',
 'benchmark.fast_livo2.m6a10_map_present': 'false',
 'benchmark.fast_livo2.m6a10_formal_replay_forbidden': 'true'}
for key, value in expected.items():
    if labels.get(key) != value:
        raise SystemExit('label mismatch: ' + key)
PY

LOG_SHA="$(sha "${LOG}")"
INSPECT_SHA="$(sha "${INSPECT}")"
RECEIPT="${RECEIPT}" IMAGE_TAG="${IMAGE_TAG}" IMAGE_ID="${IMAGE_ID}" \
BASE_IMAGE="${BASE_IMAGE}" BASE_ID="${BASE_ID}" LOG_SHA="${LOG_SHA}" \
INSPECT_SHA="${INSPECT_SHA}" python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
target = Path(os.environ['RECEIPT'])
if target.exists() or target.is_symlink():
    raise SystemExit('receipt overwrite')
value = {
 'schema_version': 1, 'receipt_kind': 'v17_build_identity', 'status': 'PASS',
 'candidate': 'v17-retryable-empty-synchronization-abort',
 'image': {'tag': os.environ['IMAGE_TAG'], 'id': os.environ['IMAGE_ID'],
           'base_tag': os.environ['BASE_IMAGE'], 'base_id': os.environ['BASE_ID']},
 'source': {
  'v17_patch_sha256': 'c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001',
  'v17_header_sha256': 'e45aace02cf153e1eeb00981628e545b8cc5fd279c185a627ce6b3ba18a16778',
  'v17_mapper_sha256': 'c2a62cf0a6943a8c68084e70585955212bbe944b8788d285e4ca41845492ba94',
  'v17_selftest_sha256': 'f81bde7cabe54ebfa702908afa9ef071f2436ce5fee2f50bf67e3c494972bc84',
  'v17_wrapper_sha256': '506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e',
  'v17_payload_sha256': 'a8043f751db0638f34e79254133ac5974527c609f7574c616ee219a685cd8ed5',
  'v17_profile_sha256': '1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b',
  'v15_feeder_sha256': '6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7',
  'phase_contract': 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary',
  'transport_contract': 'm6a10-v12-callback-ack-transport-outstanding-v1'},
 'build': {'command_count': 1, 'network': 'none', 'pull': False,
           'docker_build_log_sha256': os.environ['LOG_SHA'],
           'image_inspect_sha256': os.environ['INSPECT_SHA'],
           'catkin_rebuild': True,
           'production_selftests': ['v12_terminal', 'v12_consumer', 'v17_retry_boundary'],
           'installed_source_sha256_verified': True},
 'safety': {'input_opened': False, 'ground_truth_content_opened': False,
            'scorer_invoked': False, 'map_saved': False,
            'formal_replay_started': False}}
raw = (json.dumps(value, sort_keys=True, indent=2) + '\n').encode()
part = target.with_name('.' + target.name + '.part')
if part.exists() or part.is_symlink():
    raise SystemExit('staging overwrite')
fd = os.open(str(part), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
try:
    os.write(fd, raw); os.fsync(fd)
finally:
    os.close(fd)
os.link(str(part), str(target)); os.unlink(str(part)); os.chmod(str(target), 0o444)
digest = hashlib.sha256(raw).hexdigest()
sidecar = Path(str(target) + '.sha256')
if sidecar.exists() or sidecar.is_symlink():
    raise SystemExit('sidecar overwrite')
side_raw = ('%s  %s\n' % (digest, target.name)).encode('ascii')
fd = os.open(str(sidecar), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
try:
    os.write(fd, side_raw); os.fsync(fd)
finally:
    os.close(fd)
os.chmod(str(sidecar), 0o444)
print(str(target), digest)
PY
printf 'v17_image_tag=%s\nv17_image_id=%s\nv17_build_receipt=%s\n' \
  "${IMAGE_TAG}" "${IMAGE_ID}" "${RECEIPT}"
