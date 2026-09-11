#!/usr/bin/env bash
set -euo pipefail

# Build-only v12 candidate.  Invocation is intentionally outside the current
# gate task; this script is never run by the repository tests.  It does not
# open an input, run a container, invoke a scorer, or authorize replay.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="${FAST_LIVO2_V12_BASE_IMAGE:-m6a10-v2c-v11-bounded-end-gap-20260823t121749z-fast-livo2-benchmark:ros1-pinned}"
IMAGE_TAG="${FAST_LIVO2_V12_IMAGE_TAG:-m6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned}"
PATCH_PATH="${ROOT_DIR}/docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch"
TERMINAL_SELFTEST_PATH="${ROOT_DIR}/tools/m6a10_terminal_support_context_v12_selftest.cpp"
CONSUMER_SELFTEST_PATH="${ROOT_DIR}/tools/m6a10_consumer_evidence_v12_selftest.cpp"
STUB_ROS_PATH="${ROOT_DIR}/tools/m6a10_v12_test_stubs/ros/ros.h"
STUB_TRIGGER_PATH="${ROOT_DIR}/tools/m6a10_v12_test_stubs/std_srvs/Trigger.h"
WRAPPER_PATH="${ROOT_DIR}/scripts/fast_livo2_m6a10_v12_formal_container_run.sh"
DOCKERFILE_PATH="${ROOT_DIR}/docker/fast_livo2_m6a10_v12.Dockerfile"
EXPECTED_BASE_ID="sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a"
EXPECTED_PATCH_SHA256="39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"
EXPECTED_TERMINAL_SELFTEST_SHA256="1b42ed713dbb2a8614902ae6214d5790ab9c73ac59792f4d68753320cef47a91"
EXPECTED_CONSUMER_SELFTEST_SHA256="02078640496daa28f0be471355c6e2ad2822a76199cda1b54e738eeff5fe48de"
EXPECTED_STUB_ROS_SHA256="40c3811b6225da6ffdb2c98ed86b9d218f168ef7c1c0294825f7ad216387bb0a"
EXPECTED_STUB_TRIGGER_SHA256="34704c7662d96d395972ac2cc3a1a874331055bb48c5b16fdfb2494f5145dc24"
EXPECTED_WRAPPER_SHA256="$(sha256sum "${WRAPPER_PATH}" | awk '{print $1}')"
EXPECTED_DOCKERFILE_SHA256="f579d6b09868a67fa33c626d23295de41c5517490f7849c2fafdb52203aaae26"
EXPECTED_CONTRACT="m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
EXPECTED_TRANSPORT_CONTRACT="m6a10-v12-callback-ack-transport-outstanding-v1"
FORMAL_REPLAY_FORBIDDEN=true
RECEIPT_PATH="${FAST_LIVO2_V12_BUILD_RECEIPT_PATH:-${ROOT_DIR}/docker/fast_livo2_m6a10_v12_build.receipt.json}"
SIDECAR_PATH="${RECEIPT_PATH}.sha256"

test -f "${PATCH_PATH}" && test -f "${TERMINAL_SELFTEST_PATH}" &&
  test -f "${CONSUMER_SELFTEST_PATH}" && test -f "${WRAPPER_PATH}" &&
  test -f "${DOCKERFILE_PATH}"
test "$(sha256sum "${PATCH_PATH}" | awk '{print $1}')" = "${EXPECTED_PATCH_SHA256}"
test "$(sha256sum "${TERMINAL_SELFTEST_PATH}" | awk '{print $1}')" = "${EXPECTED_TERMINAL_SELFTEST_SHA256}"
test "$(sha256sum "${CONSUMER_SELFTEST_PATH}" | awk '{print $1}')" = "${EXPECTED_CONSUMER_SELFTEST_SHA256}"
test "$(sha256sum "${STUB_ROS_PATH}" | awk '{print $1}')" = "${EXPECTED_STUB_ROS_SHA256}"
test "$(sha256sum "${STUB_TRIGGER_PATH}" | awk '{print $1}')" = "${EXPECTED_STUB_TRIGGER_SHA256}"
test "$(sha256sum "${WRAPPER_PATH}" | awk '{print $1}')" = "${EXPECTED_WRAPPER_SHA256}"
test "$(sha256sum "${DOCKERFILE_PATH}" | awk '{print $1}')" = "${EXPECTED_DOCKERFILE_SHA256}"
test "${FORMAL_REPLAY_FORBIDDEN}" = true

# A candidate build has no input/evaluation surface.  A distinct tag and an
# absent receipt make accidental rerun or overwrite fail closed.
test -z "${BAG_PATH:-}" && test -z "${GROUND_TRUTH_PATH:-}" && test -z "${SCORER_PATH:-}"
[[ "${IMAGE_TAG}" == *v12* ]] && [[ "${IMAGE_TAG}" != "${BASE_IMAGE}" ]]
test ! -e "${RECEIPT_PATH}" && test ! -e "${SIDECAR_PATH}"

docker image inspect "${BASE_IMAGE}" >/dev/null
BASE_ID="$(docker image inspect "${BASE_IMAGE}" --format '{{.Id}}')"
test "${BASE_ID}" = "${EXPECTED_BASE_ID}"

docker build --network none --pull=false \
  --build-arg "FAST_LIVO2_V11_BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "FAST_LIVO2_V11_BASE_ID=${BASE_ID}" \
  --build-arg "FAST_LIVO2_M6A10_V12_PATCH_SHA256=${EXPECTED_PATCH_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_V12_TERMINAL_SELFTEST_SHA256=${EXPECTED_TERMINAL_SELFTEST_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_V12_CONSUMER_SELFTEST_SHA256=${EXPECTED_CONSUMER_SELFTEST_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_V12_WRAPPER_SHA256=${EXPECTED_WRAPPER_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_V12_STUB_ROS_SHA256=${EXPECTED_STUB_ROS_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_V12_STUB_TRIGGER_SHA256=${EXPECTED_STUB_TRIGGER_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_TERMINAL_CONTRACT=${EXPECTED_CONTRACT}" \
  --build-arg "FAST_LIVO2_M6A10_TRANSPORT_CONTRACT=${EXPECTED_TRANSPORT_CONTRACT}" \
  -f "${DOCKERFILE_PATH}" -t "${IMAGE_TAG}" "${ROOT_DIR}"

IMAGE_ID="$(docker image inspect "${IMAGE_TAG}" --format '{{.Id}}')"
test -n "${IMAGE_ID}" && [[ "${IMAGE_ID}" == sha256:* ]]
LABELS="$(docker image inspect "${IMAGE_TAG}" --format '{{json .Config.Labels}}')"
[[ "${LABELS}" == *"${EXPECTED_CONTRACT}"* ]] &&
  [[ "${LABELS}" == *"${EXPECTED_PATCH_SHA256}"* ]] &&
  [[ "${LABELS}" == *"${EXPECTED_WRAPPER_SHA256}"* ]] &&
  [[ "${LABELS}" == *'ground_truth_present":"false"'* ]] &&
  [[ "${LABELS}" == *'scorer_present":"false"'* ]]

RECEIPT_PARENT="$(dirname "${RECEIPT_PATH}")"
mkdir -p "${RECEIPT_PARENT}"
TMP_DIR="$(mktemp -d "${RECEIPT_PARENT}/.m6a10-v12-build.XXXXXX")"
cleanup() { [[ -n "${TMP_DIR:-}" && -d "${TMP_DIR}" ]] && rm -rf -- "${TMP_DIR}"; }
trap cleanup EXIT

python3 - "${TMP_DIR}/build.receipt.json" "${ROOT_DIR}" "${BASE_IMAGE}" \
  "${BASE_ID}" "${IMAGE_TAG}" "${IMAGE_ID}" "${EXPECTED_PATCH_SHA256}" \
  "${EXPECTED_TERMINAL_SELFTEST_SHA256}" "${EXPECTED_CONSUMER_SELFTEST_SHA256}" \
  "${EXPECTED_WRAPPER_SHA256}" "${DOCKERFILE_PATH}" "${EXPECTED_CONTRACT}" \
  "${EXPECTED_TRANSPORT_CONTRACT}" <<'PY'
import hashlib
import json
import sys

(receipt, root, base_image, base_id, image_tag, image_id, patch_sha,
 terminal_sha, consumer_sha, wrapper_sha, dockerfile, contract,
 transport_contract) = sys.argv[1:]
def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()
value = {
    'schema_version': 1,
    'status': 'PASS',
    'kind': 'fast_livo2_m6a10_v12_build_candidate',
    'variant': 'v12',
    'contract_version': contract,
    'transport_contract_version': transport_contract,
    'base_image': base_image,
    'base_image_id': base_id,
    'expected_base_image_id': 'sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a',
    'image_tag': image_tag,
    'image_id': image_id,
    'patch_sha256': patch_sha,
    'terminal_selftest_sha256': terminal_sha,
    'consumer_selftest_sha256': consumer_sha,
    'wrapper_sha256': wrapper_sha,
    'dockerfile_sha256': sha256(dockerfile),
    'network': 'none',
    'pull': False,
    'input_opened': False,
    'ground_truth_content_opened': False,
    'scorer_invoked': False,
    'formal_replay_forbidden': True,
}
with open(receipt, 'w', encoding='utf-8') as stream:
    json.dump(value, stream, indent=2, sort_keys=True)
    stream.write('\n')
PY

RECEIPT_SHA256="$(sha256sum "${TMP_DIR}/build.receipt.json" | awk '{print $1}')"
printf '%s  %s\n' "${RECEIPT_SHA256}" "$(basename "${RECEIPT_PATH}")" \
  >"${TMP_DIR}/build.receipt.json.sha256"
ln "${TMP_DIR}/build.receipt.json" "${RECEIPT_PATH}"
ln "${TMP_DIR}/build.receipt.json.sha256" "${SIDECAR_PATH}"
chmod 0444 "${RECEIPT_PATH}" "${SIDECAR_PATH}"
printf 'v12_image=%s base=%s patch=%s receipt=%s\n' \
  "${IMAGE_ID}" "${BASE_ID}" "${EXPECTED_PATCH_SHA256}" "${RECEIPT_PATH}"
