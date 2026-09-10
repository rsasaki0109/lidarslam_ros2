#!/usr/bin/env bash
set -Eeuo pipefail

# Exactly one additive v15 build plus one no-input identity container.  This
# script never opens a bag, mounts input/GT/scorer/map, or authorizes replay.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
BASE_IMAGE="${FAST_LIVO2_V15_BASE_IMAGE:-m6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned}"
IMAGE_TAG="${FAST_LIVO2_V15_IMAGE_TAG:-m6a10-v2c-v15-schema3-feeder-20260823t202247z-fast-livo2-benchmark:ros1-pinned}"
DOCKERFILE_PATH="${ROOT_DIR}/docker/fast_livo2_m6a10_v15.Dockerfile"
V15_FEEDER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v15_feeder.py"
LEGACY_FEEDER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_feeder.py"
V15_WRAPPER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v15_formal_container_run.sh"
V12_WRAPPER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v12_formal_container_run.sh"
PROFILE_PATH="${ROOT_DIR}/configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml"
FIXTURE_PATH="${ROOT_DIR}/graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json"
V12_PATCH_PATH="${ROOT_DIR}/docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch"

EXPECTED_BASE_ID="sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
EXPECTED_V15_FEEDER_SHA256="6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
EXPECTED_LEGACY_FEEDER_SHA256="869ca54921c86310af5cefc4ef0c4f8626b5fdcc125dcd60e865fdf1e677ddbf"
EXPECTED_PROFILE_SHA256="070fb2b762881a7126caf021c7a98ab59c8fa55e10be220b6482d7f2ef97899f"
EXPECTED_V12_PATCH_SHA256="39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"
EXPECTED_V12_WRAPPER_SHA256="32fce2c054b92f0d695fd4d32c7737ab75dd3a467fdee1c579c82c63398894e6"
EXPECTED_PHASE_CONTRACT="m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
EXPECTED_TRANSPORT_CONTRACT="m6a10-v12-callback-ack-transport-outstanding-v1"
RECEIPT_PATH="${FAST_LIVO2_V15_BUILD_RECEIPT_PATH:-/tmp/fast_livo2_v15_build_20260823t202247z/build_identity.receipt.json}"
RECEIPT_ROOT="$(dirname "${RECEIPT_PATH}")"
NO_INPUT_RECEIPT_PATH="${FAST_LIVO2_V15_NO_INPUT_RECEIPT_PATH:-${RECEIPT_ROOT}/no_input_identity.receipt.json}"
BUILD_LOG_PATH="${FAST_LIVO2_V15_BUILD_LOG_PATH:-${RECEIPT_ROOT}/docker_build.log}"
NO_INPUT_LOG_PATH="${FAST_LIVO2_V15_NO_INPUT_LOG_PATH:-${RECEIPT_ROOT}/no_input_identity.log}"
INSPECT_PATH="${FAST_LIVO2_V15_INSPECT_PATH:-${RECEIPT_ROOT}/no_input_container.inspect.json}"
CONTAINER_NAME="${FAST_LIVO2_V15_IDENTITY_CONTAINER_NAME:-fast-livo2-v15-identity-20260823t202247z}"

fail() { echo "FAST-LIVO2 v15 build gate failed: $*" >&2; exit 1; }
sha256_file() { sha256sum "$1" | awk '{print $1}'; }

test -f "${DOCKERFILE_PATH}" && test -f "${V15_FEEDER_PATH}" &&
  test -f "${LEGACY_FEEDER_PATH}" && test -f "${V15_WRAPPER_PATH}" &&
  test -f "${V12_WRAPPER_PATH}" && test -f "${PROFILE_PATH}" &&
  test -f "${FIXTURE_PATH}" && test -f "${V12_PATCH_PATH}"
test "$(sha256_file "${V15_FEEDER_PATH}")" = "${EXPECTED_V15_FEEDER_SHA256}"
test "$(sha256_file "${LEGACY_FEEDER_PATH}")" = "${EXPECTED_LEGACY_FEEDER_SHA256}"
test "$(sha256_file "${PROFILE_PATH}")" = "${EXPECTED_PROFILE_SHA256}"
test "$(sha256_file "${V12_PATCH_PATH}")" = "${EXPECTED_V12_PATCH_SHA256}"
test "$(sha256_file "${V12_WRAPPER_PATH}")" = "${EXPECTED_V12_WRAPPER_SHA256}"
test "${IMAGE_TAG}" != "${BASE_IMAGE}" && [[ "${IMAGE_TAG}" == *v15* ]]
test -z "${BAG_PATH:-}" && test -z "${GROUND_TRUTH_PATH:-}" && test -z "${SCORER_PATH:-}"

mkdir -p "${RECEIPT_ROOT}"
for path in "${RECEIPT_PATH}" "${RECEIPT_PATH}.sha256" \
            "${NO_INPUT_RECEIPT_PATH}" "${NO_INPUT_RECEIPT_PATH}.sha256" \
            "${BUILD_LOG_PATH}" "${NO_INPUT_LOG_PATH}" "${INSPECT_PATH}"; do
  test ! -e "${path}" && test ! -L "${path}" || fail "existing evidence path: ${path}"
done
docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1 && fail "identity container name exists"
docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1 && fail "target image tag exists"
docker image inspect "${BASE_IMAGE}" >/dev/null || fail "pinned base image missing"
BASE_ID="$(docker image inspect "${BASE_IMAGE}" --format '{{.Id}}')"
test "${BASE_ID}" = "${EXPECTED_BASE_ID}"

V15_WRAPPER_SHA256="$(sha256_file "${V15_WRAPPER_PATH}")"
V15_FIXTURE_SHA256="$(sha256_file "${FIXTURE_PATH}")"
V15_DOCKERFILE_SHA256="$(sha256_file "${DOCKERFILE_PATH}")"

# There is exactly one docker build command and no retry path.
set +e
docker build --network none --pull=false -t "${IMAGE_TAG}" \
  -f "${DOCKERFILE_PATH}" \
  --build-arg "FAST_LIVO2_V12_BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "FAST_LIVO2_V12_BASE_ID=${BASE_ID}" \
  --build-arg "FAST_LIVO2_V15_FEEDER_SHA256=${EXPECTED_V15_FEEDER_SHA256}" \
  --build-arg "FAST_LIVO2_LEGACY_FEEDER_SHA256=${EXPECTED_LEGACY_FEEDER_SHA256}" \
  --build-arg "FAST_LIVO2_V15_WRAPPER_SHA256=${V15_WRAPPER_SHA256}" \
  --build-arg "FAST_LIVO2_V15_PROFILE_SHA256=${EXPECTED_PROFILE_SHA256}" \
  --build-arg "FAST_LIVO2_V15_FIXTURE_SHA256=${V15_FIXTURE_SHA256}" \
  --build-arg "FAST_LIVO2_V12_PATCH_SHA256=${EXPECTED_V12_PATCH_SHA256}" \
  --build-arg "FAST_LIVO2_PHASE_CONTRACT=${EXPECTED_PHASE_CONTRACT}" \
  --build-arg "FAST_LIVO2_TRANSPORT_CONTRACT=${EXPECTED_TRANSPORT_CONTRACT}" \
  "${ROOT_DIR}" >"${BUILD_LOG_PATH}" 2>&1
BUILD_RC=$?
set -e
if (( BUILD_RC != 0 )); then
  echo "v15 build failed rc=${BUILD_RC}; log=${BUILD_LOG_PATH}" >&2
  exit "${BUILD_RC}"
fi

IMAGE_ID="$(docker image inspect "${IMAGE_TAG}" --format '{{.Id}}')"
test -n "${IMAGE_ID}" && test "${IMAGE_ID}" != "${BASE_ID}"
LABELS_JSON="$(docker image inspect "${IMAGE_TAG}" --format '{{json .Config.Labels}}')"
python3 - "${LABELS_JSON}" "${V15_WRAPPER_SHA256}" "${V15_FIXTURE_SHA256}" <<'PY'
import json
import sys
labels = json.loads(sys.argv[1])
expected = {
    'benchmark.fast_livo2.m6a10_v12_base_id': 'sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7',
    'benchmark.fast_livo2.m6a10_v15_feeder_sha256': '6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7',
    'benchmark.fast_livo2.m6a10_legacy_feeder_sha256': '869ca54921c86310af5cefc4ef0c4f8626b5fdcc125dcd60e865fdf1e677ddbf',
    'benchmark.fast_livo2.m6a10_v15_wrapper_sha256': sys.argv[2],
    'benchmark.fast_livo2.m6a10_v15_profile_sha256': '070fb2b762881a7126caf021c7a98ab59c8fa55e10be220b6482d7f2ef97899f',
    'benchmark.fast_livo2.m6a10_v15_fixture_sha256': sys.argv[3],
    'benchmark.fast_livo2.m6a10_phase_contract': 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary',
    'benchmark.fast_livo2.m6a10_transport_contract': 'm6a10-v12-callback-ack-transport-outstanding-v1',
    'benchmark.fast_livo2.m6a10_network_expectation': 'none',
    'benchmark.fast_livo2.m6a10_rootfs_expectation': 'read_only_runtime',
    'benchmark.fast_livo2.m6a10_formal_replay_forbidden': 'true',
}
for key, value in expected.items():
    if labels.get(key) != value:
        raise SystemExit('label mismatch: %s' % key)
PY

# One no-input identity/selftest container: no bind mounts and no ROS/bag.
set +e
docker run --name "${CONTAINER_NAME}" --network none --read-only \
  --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev \
  --tmpfs /out:rw,nosuid,nodev "${IMAGE_ID}" /bin/bash -lc \
  "PYTHONPATH=/runner/scripts python3 -c \"import hashlib,json; from pathlib import Path; import fast_livo2_m6a10_v15_feeder as f; p=Path('/opt/fast_livo_v15/m6a10_v15_consumer_status_pass.json'); v=json.loads(p.read_text()); assert f.validate_consumer_status(v)['schema_version']==3; assert hashlib.sha256(Path('/runner/scripts/fast_livo2_m6a10_feeder.py').read_bytes()).hexdigest()=='${EXPECTED_V15_FEEDER_SHA256}'; print('V15_NO_INPUT_IDENTITY_PASS')\"" \
  >"${NO_INPUT_LOG_PATH}" 2>&1
NO_INPUT_RC=$?
set -e
docker inspect "${CONTAINER_NAME}" >"${INSPECT_PATH}"
CONTAINER_STATUS="$(docker inspect --format '{{.State.Status}}' "${CONTAINER_NAME}")"
set +e
python3 - "${INSPECT_PATH}" "${NO_INPUT_RC}" "${CONTAINER_STATUS}" <<'PY'
import json
import sys
doc = json.load(open(sys.argv[1], encoding='utf-8'))[0]
if sys.argv[3] != 'exited':
    raise SystemExit('identity container did not exit')
state = doc.get('State', {})
if int(sys.argv[2]) != 0 or state.get('ExitCode') != 0 or state.get('OOMKilled') is not False:
    raise SystemExit('identity container failure')
host = doc.get('HostConfig', {})
if host.get('NetworkMode') != 'none' or host.get('ReadonlyRootfs') is not True:
    raise SystemExit('identity network/rootfs contract')
mounts = doc.get('Mounts', [])
if any(m.get('Type') != 'tmpfs' for m in mounts):
    raise SystemExit('identity has a host bind mount')
if {m.get('Destination') for m in mounts} != {'/tmp', '/root/.ros', '/out'}:
    raise SystemExit('identity tmpfs set mismatch')
PY
IDENTITY_VALIDATION_RC=$?
set -e
if [[ "${CONTAINER_STATUS}" == exited ]]; then
  docker rm "${CONTAINER_NAME}" >/dev/null
  CONTAINER_REMOVED=true
else
  CONTAINER_REMOVED=false
fi

BUILD_LOG_SHA256="$(sha256_file "${BUILD_LOG_PATH}")"
NO_INPUT_LOG_SHA256="$(sha256_file "${NO_INPUT_LOG_PATH}")"
INSPECT_SHA256="$(sha256_file "${INSPECT_PATH}")"
NO_INPUT_STATUS=PASS
if (( NO_INPUT_RC != 0 || IDENTITY_VALIDATION_RC != 0 )); then
  NO_INPUT_STATUS=FAIL_CLOSED
fi

seal_receipt() {
  local target="$1" kind="$2" status="$3"
  RECEIPT_TARGET="${target}" RECEIPT_KIND="${kind}" RECEIPT_STATUS="${status}" \
  RECEIPT_IMAGE_TAG="${IMAGE_TAG}" RECEIPT_IMAGE_ID="${IMAGE_ID}" \
  RECEIPT_BASE_TAG="${BASE_IMAGE}" RECEIPT_BASE_ID="${BASE_ID}" \
  RECEIPT_WRAPPER_SHA="${V15_WRAPPER_SHA256}" RECEIPT_FIXTURE_SHA="${V15_FIXTURE_SHA256}" \
  RECEIPT_DOCKERFILE_SHA="${V15_DOCKERFILE_SHA256}" RECEIPT_BUILD_LOG_SHA="${BUILD_LOG_SHA256}" \
  RECEIPT_NO_INPUT_LOG_SHA="${NO_INPUT_LOG_SHA256}" RECEIPT_INSPECT_SHA="${INSPECT_SHA256}" \
  RECEIPT_CONTAINER="${CONTAINER_NAME}" RECEIPT_REMOVED="${CONTAINER_REMOVED}" \
  RECEIPT_NO_INPUT_RC="${NO_INPUT_RC}" python3 - <<'PY'
import hashlib
import json
import os
from pathlib import Path
target = Path(os.environ['RECEIPT_TARGET'])
if target.exists() or target.is_symlink():
    raise SystemExit('receipt overwrite refused')
target.parent.mkdir(parents=True, exist_ok=True)
value = {
    'schema_version': 1, 'receipt_kind': os.environ['RECEIPT_KIND'],
    'status': os.environ['RECEIPT_STATUS'], 'candidate': 'v15-schema3-feeder',
    'image': {'tag': os.environ['RECEIPT_IMAGE_TAG'], 'id': os.environ['RECEIPT_IMAGE_ID'],
              'base_tag': os.environ['RECEIPT_BASE_TAG'], 'base_id': os.environ['RECEIPT_BASE_ID']},
    'source': {'v15_feeder_sha256': '6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7',
               'legacy_feeder_sha256': '869ca54921c86310af5cefc4ef0c4f8626b5fdcc125dcd60e865fdf1e677ddbf',
               'v15_wrapper_sha256': os.environ['RECEIPT_WRAPPER_SHA'],
               'v15_profile_sha256': '070fb2b762881a7126caf021c7a98ab59c8fa55e10be220b6482d7f2ef97899f',
               'v15_dockerfile_sha256': os.environ['RECEIPT_DOCKERFILE_SHA'],
               'v15_fixture_sha256': os.environ['RECEIPT_FIXTURE_SHA'],
               'v12_patch_sha256': '39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333',
               'phase_contract': 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary',
               'transport_contract': 'm6a10-v12-callback-ack-transport-outstanding-v1'},
    'execution': {'network': 'none', 'rootfs': 'read_only', 'input_mounts': 0,
                  'ground_truth_mount': False, 'scorer_mount': False, 'map_mount': False,
                  'formal_replay_forbidden': True, 'formal_replay_started': False,
                  'build_command_count': 1, 'no_input_container_count': 1,
                  'no_input_exit_code': int(os.environ['RECEIPT_NO_INPUT_RC']),
                  'container_removed_stopped_only': os.environ['RECEIPT_REMOVED'] == 'true'},
    'artifacts': {'docker_build_log_sha256': os.environ['RECEIPT_BUILD_LOG_SHA'],
                  'no_input_log_sha256': os.environ['RECEIPT_NO_INPUT_LOG_SHA'],
                  'container_inspect_sha256': os.environ['RECEIPT_INSPECT_SHA']},
    'safety': {'input_opened': False, 'ground_truth_content_opened': False,
               'scorer_invoked': False, 'map_saved': False},
}
raw = (json.dumps(value, sort_keys=True, indent=2) + '\n').encode('utf-8')
part = target.with_name('.' + target.name + '.part')
if part.exists() or part.is_symlink():
    raise SystemExit('receipt staging overwrite refused')
fd = os.open(str(part), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
try:
    os.write(fd, raw); os.fsync(fd)
finally:
    os.close(fd)
os.link(str(part), str(target)); os.unlink(str(part)); os.chmod(str(target), 0o444)
digest = hashlib.sha256(raw).hexdigest()
sidecar = Path(str(target) + '.sha256')
if sidecar.exists() or sidecar.is_symlink():
    raise SystemExit('receipt sidecar overwrite refused')
side_raw = ('%s  %s\n' % (digest, target.name)).encode('ascii')
fd = os.open(str(sidecar), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
try:
    os.write(fd, side_raw); os.fsync(fd)
finally:
    os.close(fd)
os.chmod(str(sidecar), 0o444)
print('%s %s' % (target, digest))
PY
}

seal_receipt "${RECEIPT_PATH}" "v15_build_identity" "PASS"
seal_receipt "${NO_INPUT_RECEIPT_PATH}" "v15_no_input_identity" "${NO_INPUT_STATUS}"
printf 'v15_image_tag=%s\nv15_image_id=%s\nv15_build_receipt=%s\nv15_no_input_receipt=%s\n' \
  "${IMAGE_TAG}" "${IMAGE_ID}" "${RECEIPT_PATH}" "${NO_INPUT_RECEIPT_PATH}"
if [[ "${NO_INPUT_STATUS}" != PASS ]]; then
  exit 1
fi
