#!/usr/bin/env bash
set -euo pipefail

# Build-only v11 candidate.  This script never opens a bag, mounts data, runs
# a container, starts ROS, invokes a scorer, or authorizes formal replay.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="${FAST_LIVO2_V11_BASE_IMAGE:-m6a10-v2c-v10-callback-v3-20260823t094100z-fast-livo2-benchmark:ros1-pinned}"
IMAGE_TAG="${FAST_LIVO2_V11_IMAGE_TAG:-m6a10-v2c-v11-bounded-end-gap-20260823-fast-livo2-benchmark:ros1-pinned}"
PATCH_PATH="${ROOT_DIR}/docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch"
SELFTEST_PATH="${ROOT_DIR}/tools/m6a10_terminal_support_context_v11_selftest.cpp"
DOCKERFILE_PATH="${ROOT_DIR}/docker/fast_livo2_m6a10_v11.Dockerfile"
EXPECTED_BASE_ID="sha256:3e087acc5ef116f03357a73927c18b2059068b093ae1cce6cb41c1baf1fbf759"
EXPECTED_PATCH_SHA256="2cbca3a7bb465981cb5e3072efd1713302e002127d4ae5872413ed5749d80ba2"
EXPECTED_SELFTEST_SHA256="7eaf8fbc5d522870d184857a77771ba12800816adf376ddb2554bc5649c85981"
EXPECTED_DOCKERFILE_SHA256="4cf6ebd26abf389e2deaf318576b0d467c67491e54224ea56b5198f1be0561e1"
EXPECTED_CONTRACT="m6a10-online-compute-v4-terminal-bounded-end-gap"
FORMAL_REPLAY_FORBIDDEN=true
RECEIPT_PATH="${FAST_LIVO2_V11_BUILD_RECEIPT_PATH:-${ROOT_DIR}/docker/fast_livo2_m6a10_v11_build.receipt.json}"
SIDECAR_PATH="${RECEIPT_PATH}.sha256"

test -f "${PATCH_PATH}"
test -f "${SELFTEST_PATH}"
test -f "${DOCKERFILE_PATH}"
test "$(sha256sum "${PATCH_PATH}" | awk '{print $1}')" = "${EXPECTED_PATCH_SHA256}"
test "$(sha256sum "${SELFTEST_PATH}" | awk '{print $1}')" = "${EXPECTED_SELFTEST_SHA256}"
test "$(sha256sum "${DOCKERFILE_PATH}" | awk '{print $1}')" = "${EXPECTED_DOCKERFILE_SHA256}"
test "${FORMAL_REPLAY_FORBIDDEN}" = true

# A candidate build has no input, evaluation, or replay surface.
test -z "${BAG_PATH:-}"
test -z "${GROUND_TRUTH_PATH:-}"
test -z "${SCORER_PATH:-}"
[[ "${IMAGE_TAG}" == *v11* ]]
[[ "${IMAGE_TAG}" != "${BASE_IMAGE}" ]]
test ! -e "${RECEIPT_PATH}"
test ! -e "${SIDECAR_PATH}"

docker image inspect "${BASE_IMAGE}" >/dev/null
BASE_ID="$(docker image inspect "${BASE_IMAGE}" --format '{{.Id}}')"
test "${BASE_ID}" = "${EXPECTED_BASE_ID}"

docker build --network none --pull=false \
  --build-arg "FAST_LIVO2_V10_BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "FAST_LIVO2_V10_BASE_ID=${BASE_ID}" \
  --build-arg "FAST_LIVO2_M6A10_V11_PATCH_SHA256=${EXPECTED_PATCH_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_V11_SELFTEST_SHA256=${EXPECTED_SELFTEST_SHA256}" \
  --build-arg "FAST_LIVO2_M6A10_TERMINAL_CONTRACT=${EXPECTED_CONTRACT}" \
  -f "${DOCKERFILE_PATH}" \
  -t "${IMAGE_TAG}" "${ROOT_DIR}"

IMAGE_ID="$(docker image inspect "${IMAGE_TAG}" --format '{{.Id}}')"
test -n "${IMAGE_ID}"

RECEIPT_PARENT="$(dirname "${RECEIPT_PATH}")"
mkdir -p "${RECEIPT_PARENT}"
TMP_DIR="$(mktemp -d "${RECEIPT_PARENT}/.m6a10-v11-build.XXXXXX")"
cleanup() {
  if [[ -n "${TMP_DIR:-}" && -d "${TMP_DIR}" ]]; then
    rm -rf -- "${TMP_DIR}"
  fi
}
trap cleanup EXIT

python3 - "${TMP_DIR}/build.receipt.json" "${ROOT_DIR}" "${BASE_IMAGE}" \
  "${BASE_ID}" "${IMAGE_TAG}" "${IMAGE_ID}" "${EXPECTED_PATCH_SHA256}" \
  "${EXPECTED_SELFTEST_SHA256}" "${DOCKERFILE_PATH}" <<'PY'
import hashlib
import json
import os
import sys

receipt_path, root, base_image, base_id, image_tag, image_id, patch_sha, selftest_sha, dockerfile = sys.argv[1:]
def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()

value = {
    'schema_version': 1,
    'status': 'PASS',
    'kind': 'fast_livo2_m6a10_v11_build_candidate',
    'variant': 'v11',
    'contract_version': 'm6a10-online-compute-v4-terminal-bounded-end-gap',
    'base_image': base_image,
    'base_image_id': base_id,
    'expected_base_image_id': 'sha256:3e087acc5ef116f03357a73927c18b2059068b093ae1cce6cb41c1baf1fbf759',
    'image_tag': image_tag,
    'image_id': image_id,
    'patch_sha256': patch_sha,
    'selftest_sha256': selftest_sha,
    'dockerfile_sha256': sha256(dockerfile),
    'network': 'none',
    'input_opened': False,
    'ground_truth_content_opened': False,
    'scorer_invoked': False,
    'formal_replay_forbidden': True,
}
with open(receipt_path, 'w', encoding='utf-8') as stream:
    json.dump(value, stream, indent=2, sort_keys=True)
    stream.write('\n')
PY

RECEIPT_SHA256="$(sha256sum "${TMP_DIR}/build.receipt.json" | awk '{print $1}')"
printf '%s  %s\n' "${RECEIPT_SHA256}" "$(basename "${RECEIPT_PATH}")" \
  > "${TMP_DIR}/build.receipt.json.sha256"

# Publish both artifacts with no-overwrite hard-link creation.  A rerun or a
# race with an existing result fails closed and never mutates prior receipts.
ln "${TMP_DIR}/build.receipt.json" "${RECEIPT_PATH}"
ln "${TMP_DIR}/build.receipt.json.sha256" "${SIDECAR_PATH}"
chmod 0444 "${RECEIPT_PATH}" "${SIDECAR_PATH}"
printf 'v11_image=%s base=%s patch=%s selftest=%s receipt=%s\n' \
  "${IMAGE_ID}" "${BASE_ID}" "${EXPECTED_PATCH_SHA256}" \
  "${EXPECTED_SELFTEST_SHA256}" "${RECEIPT_PATH}"
